// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices LTC2378 ADC family driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

 #include <linux/unaligned.h>
 #include <linux/bitfield.h>
 #include <linux/bitops.h>
 #include <linux/clk.h>
 #include <linux/delay.h>
 #include <linux/string.h>
 #include <linux/device.h>
 #include <linux/dma-mapping.h>
 #include <linux/dmaengine.h>
 #include <linux/err.h>
 #include <linux/jiffies.h>
 #include <linux/kernel.h>
 #include <linux/module.h>
 #include <linux/property.h>
 #include <linux/pwm.h>
 #include <linux/spi/spi.h>
 #include <linux/spi/legacy-spi-engine.h>
 #include <linux/regulator/consumer.h>
 
 #include <linux/iio/buffer.h>
 #include <linux/iio/buffer-dma.h>
 #include <linux/iio/buffer-dmaengine.h>
 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/io.h>

struct ltc2378_chip_info {
        const char *name;
        int num_channels;
        int resolution;
        int sclk_rate;
        int max_rate;
};

static const struct ltc2378_chip_info ltc2378_chip_info = {
        .name = "ltc2378",
        .resolution = 20,
        .num_channels = 1,
        .max_rate = 1000000,
        .sclk_rate = 90000000
};

struct ltc2378_adc {
        const struct ltc2378_chip_info *info;
        struct iio_chan_spec *channels;
        struct spi_transfer seq_xfer[1];
        unsigned int cfg;
        unsigned long ref_clk_rate;
        struct pwm_device *cnv;
        struct spi_device *spi;
        struct regulator *vref;
        int spi_speed_hz;
        int samp_freq;
        int device_id;
        u8 rx_buf[4]; // for 32-bit data

        /*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

         __be32 spi_rx_word __aligned(IIO_DMA_MINALIGN);

};

static const struct iio_chan_spec ltc2378_chan = {
        .type = IIO_VOLTAGE,
        .indexed = 1,
        .channel = 0,
        .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE),
        .scan_index = 0,
        .scan_type = {
                .sign = 's',  // LTC2378-20 two's complement
                .storagebits = 32,
                .realbits = 20,
                .shift = 0,
                .endianness = IIO_CPU,
        },
};

static int ltc2378_read_channel(struct ltc2378_adc *adc, unsigned int *val)
{
    int ret;
    u32 raw;
    s32 signed_val;

    struct spi_transfer xfer = {
        .rx_buf = &adc->spi_rx_word,   // Aligned 32-bit buffer
        .len = 1,                      // Single 32-bit word
        .bits_per_word = 20,          // Exactly 20 clock pulses
        .speed_hz = adc->info->sclk_rate,
        .delay = {
            .value = 600,             // 600ns delay after CS assertion
            .unit = SPI_DELAY_UNIT_NSECS,
        },
    };

    ret = spi_sync_transfer(adc->spi, &xfer, 1);
    if (ret)
        return ret;

    raw = be32_to_cpu(adc->spi_rx_word); // Handle byte order
    raw >>= 12;                          // shift 20-bit value to LSB
    raw &= 0xFFFFF;                      // Mask to keep only the 20 valid bits

    // Sign-extend 20-bit two's complement to 32-bit signed
    if (raw & BIT(19))
        signed_val = raw | 0xFFF00000;   // Fill top 12 bits with 1s
    else
        signed_val = raw;

    *val = signed_val;

    return 0;
}

static int ltc2378_set_samp_freq(struct ltc2378_adc *adc, int freq)
{
        unsigned long long ref_clk_period_ns;
        struct pwm_state cnv_state;
        int ret;
        u32 rem;

        printk(KERN_INFO "ltc2378: freq input = %d\n", freq);
        printk(KERN_INFO "ltc2378: max_rate = %d\n", adc->info->max_rate);
        freq = clamp(freq, 1, adc->info->max_rate); // freq between 1 Hz and max_rate
        printk(KERN_INFO "ltc2378: freq clamped = %d\n", freq);
        
        printk(KERN_INFO "ltc2378: ref_clk_rate = %lu\n", adc->ref_clk_rate);
        ref_clk_period_ns = DIV_ROUND_UP(NSEC_PER_SEC, adc->ref_clk_rate);
        printk(KERN_INFO "ltc2378: ref_clk_period_ns = %llu\n", ref_clk_period_ns);

        pwm_get_state(adc->cnv, &cnv_state);
        printk(KERN_INFO "ltc2378: PWM current period = %llu\n", cnv_state.period);
        printk(KERN_INFO "ltc2378: PWM current duty_cycle = %llu\n", cnv_state.duty_cycle);
        printk(KERN_INFO "ltc2378: PWM current enabled = %d\n", cnv_state.enabled);

        cnv_state.duty_cycle = ref_clk_period_ns;
        cnv_state.enabled = true;
        printk(KERN_INFO "ltc2378: PWM new duty_cycle = %llu\n", cnv_state.duty_cycle);

        printk(KERN_INFO "ltc2378: DIV_ROUND_UP(ref_clk_rate, freq) = %u\n", DIV_ROUND_UP(adc->ref_clk_rate, freq));
        cnv_state.period = div_u64_rem((u64)DIV_ROUND_UP(adc->ref_clk_rate, freq) * NSEC_PER_SEC, // compute PWM period
                                        adc->ref_clk_rate, &rem);
        printk(KERN_INFO "ltc2378: PWM new period = %llu\n", cnv_state.period);
        printk(KERN_INFO "ltc2378: rem = %u\n", rem);
        
        /* Special case for 1MHz: treat as 999999 Hz */
        if (freq == 1000000) {
                cnv_state.period = div_u64_rem((u64)DIV_ROUND_UP(adc->ref_clk_rate, 999999) * NSEC_PER_SEC,
                                               adc->ref_clk_rate, &rem);
                if (rem) {
                        cnv_state.period += 1;
                }
                printk(KERN_INFO "ltc2378: 1MHz special case - calculated as 999999 Hz, period = %llu\n", cnv_state.period);
        } else if (rem) {
                cnv_state.period += 1;
                printk(KERN_INFO "ltc2378: PWM period adjusted = %llu\n", cnv_state.period);
        }

        ret = pwm_apply_might_sleep(adc->cnv, &cnv_state);  // apply calculated PWM state
        printk(KERN_INFO "ltc2378: pwm_apply ret = %d\n", ret);
        if (ret)
                return ret;

        adc->samp_freq = freq;
        printk(KERN_INFO "ltc2378: samp_freq set to = %d\n", adc->samp_freq);

        return ret;
}

static int ltc2378_read_raw(struct iio_dev *indio_dev,
                            const struct iio_chan_spec *chan,
                            int *val, int *val2, long info)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);
        int ret;

        switch (info) {
        case IIO_CHAN_INFO_RAW: 
                ret = ltc2378_read_channel(adc, val);
                if (ret)
                        return ret;
                
                return IIO_VAL_INT;

        case IIO_CHAN_INFO_SAMP_FREQ:
                *val = adc->samp_freq;
                return IIO_VAL_INT;
        
        case IIO_CHAN_INFO_SCALE:
                 // Get the reference voltage in mV
                 ret = regulator_get_voltage(adc->vref);
                 if (ret < 0)
                        return ret;
                
                *val = ret / 1000; // convert uV to mV

                *val2 = adc->info->resolution;

                return IIO_VAL_FRACTIONAL_LOG2;
        
        default:
                return -EINVAL;
        }
}

static int ltc2378_write_raw(struct iio_dev *indio_dev,
                             const struct iio_chan_spec *chan,
                             int val, int val2, long info)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);
        
        switch (info) {
        case IIO_CHAN_INFO_SAMP_FREQ:
                return ltc2378_set_samp_freq(adc, val);
        default:
                return -EINVAL;
        }
}

static int ltc2378_buffer(struct iio_dev *indio_dev, struct spi_message *msg)
{
    struct ltc2378_adc *adc = iio_priv(indio_dev);
    struct spi_transfer *xfer = &adc->seq_xfer[0];

    spi_message_init(msg);

    xfer->tx_buf = NULL;
    xfer->rx_buf = &adc->spi_rx_word;
    xfer->len = 1;
    xfer->bits_per_word = 20;
    xfer->speed_hz = adc->info->sclk_rate;
    xfer->cs_change = 0;
    
    /* Add 600ns delay after CS assertion before SCLK starts */
    xfer->delay.value = 600;  /* Delay before transfer starts (after CS assertion) */
    xfer->delay.unit = SPI_DELAY_UNIT_NSECS;
    
    /* No additional delays for other timing */
   // xfer->cs_change_delay.value = 0;
   // xfer->cs_change_delay.unit = SPI_DELAY_UNIT_NSECS;
    
    //xfer->word_delay.value = 0;
    //xfer->word_delay.unit = SPI_DELAY_UNIT_NSECS;

    spi_message_add_tail(xfer, msg);

    return 0;
}

static int ltc2378_buffer_preenable(struct iio_dev *indio_dev)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);
        struct spi_message msg;
        int ret;

        ret = ltc2378_buffer(indio_dev, &msg);
        if (ret)
                return ret;
        
        spi_bus_lock(adc->spi->controller);
        ret = legacy_spi_engine_offload_load_msg(adc->spi, &msg);
        if (ret)
                return ret;

        legacy_spi_engine_offload_enable(adc->spi, true);

        return 0;
}

static int ltc2378_buffer_postdisable(struct iio_dev *indio_dev)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);

        legacy_spi_engine_offload_enable(adc->spi, false);
        spi_bus_unlock(adc->spi->controller);

        return 0;
}

static const struct iio_buffer_setup_ops ltc2378_buffer_ops = {
        .preenable = &ltc2378_buffer_preenable,
        .postdisable = &ltc2378_buffer_postdisable,
};

static const struct iio_info ltc2378_iio_info = {
        .read_raw = &ltc2378_read_raw,
        .write_raw = &ltc2378_write_raw,
};

static int ltc2378_parse_channels(struct iio_dev *indio_dev)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);
        struct device *dev = indio_dev->dev.parent;

        /* Allocate space for one channel */
        adc->channels = devm_kcalloc(dev, 1, sizeof(struct iio_chan_spec),
				     GFP_KERNEL);
        if (!adc->channels)
                return -ENOMEM;
        
        /* Setup the single channel */
        *adc->channels = ltc2378_chan;
        adc->channels->scan_type.realbits = adc->info->resolution;

        indio_dev->channels = adc->channels;
        indio_dev->num_channels = 1;

        return 0;
}

static void ltc2378_reg_disable(void *data)
{
        regulator_disable(data);
}

static void ltc2378_pwm_disable(void *data)
{
        pwm_disable(data);
}

static void ltc2378_clk_disable(void *data)
{
        clk_disable_unprepare(data);
}

static int ltc2378_probe(struct spi_device *spi)
{
        struct ltc2378_adc *adc;
        struct iio_dev *indio_dev;
        struct clk *ref_clk;
        int ret;

        indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
        if (!indio_dev)
                return -ENOMEM;
        
        adc =  iio_priv(indio_dev);
        adc->spi = spi;
        adc->vref = devm_regulator_get(&spi->dev, "vref");
        if (IS_ERR(adc->vref))
                return PTR_ERR(adc->vref);
        ret = regulator_enable(adc->vref);
        if (ret) {
                dev_err(&spi->dev, "Failed to enable VREF regulator");
                return ret;
        }

        ret = devm_add_action_or_reset(&spi->dev, ltc2378_reg_disable, adc->vref);
        if (ret)
                return ret;
        ref_clk = devm_clk_get(&spi->dev, "spi_clk");
        if (IS_ERR(ref_clk))
                return PTR_ERR(ref_clk);
        ret = clk_prepare_enable(ref_clk);
        if (ret)
                return ret;

        ret = devm_add_action_or_reset(&spi->dev, ltc2378_clk_disable, ref_clk);
        if (ret)
                return ret;
        
        adc->ref_clk_rate = clk_get_rate(ref_clk);
        adc->cnv = devm_pwm_get(&spi->dev, "cnv");
        if (IS_ERR(adc->cnv))
                return PTR_ERR(adc->cnv);
        
        ret = devm_add_action_or_reset(&spi->dev, ltc2378_pwm_disable, adc->cnv);
        if (ret)
                return ret;

        adc->info = device_get_match_data(&spi->dev);
        if (!adc->info) {
                adc->info = (struct ltc2378_chip_info *)
                                spi_get_device_id(spi)->driver_data;
                if (!adc->info)
                        return ret;
        }
        ret = ltc2378_parse_channels(indio_dev);
        if (ret)
                return -EINVAL;

        indio_dev->name = adc->info->name;
        indio_dev->info = &ltc2378_iio_info;
        indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
        indio_dev->setup_ops = &ltc2378_buffer_ops;
        ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
                                              indio_dev, "rx");
        if (ret)
                return ret;
        ret = ltc2378_set_samp_freq(adc, adc->info->max_rate);
        if (ret)
                return ret;

        dev_info(&spi->dev, "ltc2378 - bits_per_word_mask: 0x%lx\n", spi->controller->bits_per_word_mask); // check if SPI supports 20 bit transfers
        
        return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ltc2378_of_match[] = {
        { .compatible = "adi,ltc2378", .data = &ltc2378_chip_info},
        { },
};
MODULE_DEVICE_TABLE(of, ltc2378_of_match);

static const struct spi_device_id ltc2378_spi_id[] = {
        { "ltc2378", (kernel_ulong_t)&ltc2378_chip_info },
        { },
};
MODULE_DEVICE_TABLE(spi, ltc2378_spi_id);

static struct spi_driver ltc2378_driver = {
        .driver = {
                .name = "ltc2378_adc",
                .of_match_table = ltc2378_of_match
        },
        .probe = ltc2378_probe,
        .id_table = ltc2378_spi_id,
};
module_spi_driver(ltc2378_driver);

MODULE_AUTHOR("Pop Ioan Daniel <pop.ioan-daniel@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC2378");
MODULE_LICENSE("GPL");
