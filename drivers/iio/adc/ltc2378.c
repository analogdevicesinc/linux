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
 #include <linux/spi/offload/consumer.h>
 #include <linux/regulator/consumer.h>
#include <linux/units.h>
 
 #include <linux/iio/buffer.h>
 #include <linux/iio/buffer-dma.h>
 #include <linux/iio/buffer-dmaengine.h>
 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/io.h>

#define LTC2378_TCYC_NS			MILLI

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

/* SPI offload configuration for LTC2378 */
static const struct spi_offload_config ltc2378_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

struct ltc2378_adc {
        const struct ltc2378_chip_info *info;
        struct iio_chan_spec *channels;
        struct spi_transfer seq_xfer[1];
        unsigned int cfg;
        unsigned long ref_clk_rate;
        struct spi_device *spi;
        struct regulator *vref;
        int spi_speed_hz;
        int samp_freq;
        int device_id;
        u8 rx_buf[4]; // for 32-bit data

        /* New SPI offload API fields */
        struct spi_transfer offload_xfer;
        struct spi_message offload_msg;
        struct spi_offload *offload;
        struct spi_offload_trigger *offload_trigger;
        struct spi_offload_trigger_config offload_trigger_config;

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
        /*.delay = {
            .value = 0,
            .unit = SPI_DELAY_UNIT_NSECS,
        }, */
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
	struct spi_offload_trigger_config *config = &adc->offload_trigger_config;
        u64 period_ns;
        u64 adjusted_freq;
	int ret;

        printk(KERN_INFO "ltc2378: freq input = %d\n", freq);
        printk(KERN_INFO "ltc2378: max_rate = %d\n", adc->info->max_rate);
	if (!in_range(freq, 1, adc->info->max_rate))
		return -EINVAL;

        printk(KERN_INFO "ltc2378: freq clamped = %d\n", freq);

        /* LTC2378-20 requires minimum 1000ns cycle time (1 MSPS max).
         * Due to rounding in PWM hardware (10ns clock period), 1MHz can become
         * 990ns (99 clocks) instead of 1000ns (100 clocks). To ensure we meet
         * the minimum timing requirement, adjust the frequency slightly lower
         * so the period rounds up to the next clock cycle.
         */
        period_ns = DIV_ROUND_UP_ULL(NSEC_PER_SEC, (u64)freq);
        printk(KERN_INFO "ltc2378: calculated period_ns = %llu\n", period_ns);

	config->periodic.frequency_hz = freq;
	ret = spi_offload_trigger_validate(adc->offload_trigger, config);
	if (ret)
		return ret;

        // if (period_ns <= 1000) {
        //         /* At or near 1 MSPS - adjust frequency to ensure period >= 1000ns
        //          * after PWM hardware quantization. Use 999kHz to get 1001ns period,
        //          * which rounds to 100 clocks (1000ns) instead of 99 clocks (990ns).
        //          */
        //         adjusted_freq = DIV_ROUND_UP_ULL(NSEC_PER_SEC, 1001);
        //         printk(KERN_INFO "ltc2378: adjusted freq from %d to %llu to meet min period\n",
        //                freq, adjusted_freq);
        //         config.periodic.frequency_hz = adjusted_freq;
        // } else {
                //config.periodic.frequency_hz = freq; //already set above
        //}
        
        /* Store frequency - trigger framework controls PWM */
        adc->samp_freq = freq;

        printk(KERN_INFO "ltc2378: frequency set = %d\n", adc->samp_freq);

        return 0;
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

static int ltc2378_buffer_postenable(struct iio_dev *indio_dev)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);
        int ret;

        /* Setup offload transfer */
        adc->offload_xfer.rx_buf = NULL;  /* For streaming, DMA framework handles buffer */
        adc->offload_xfer.tx_buf = NULL;
        adc->offload_xfer.len = 4;  /* 4 bytes for 20-bit data (32-bit aligned) */
        adc->offload_xfer.bits_per_word = 20;
        adc->offload_xfer.offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;  /* Enable RX streaming */

        /* Additional SLEEP instruction needed for tQUIET timing requirements. */
        adc->offload_xfer.delay.value = 15;
        adc->offload_xfer.delay.unit = SPI_DELAY_UNIT_NSECS;

        /* Initialize message with offload */
        spi_message_init_with_transfers(&adc->offload_msg, &adc->offload_xfer, 1);
        adc->offload_msg.offload = adc->offload;

        /* Optimize message for offload */
        ret = spi_optimize_message(adc->spi, &adc->offload_msg);
        if (ret) {
                printk(KERN_ERR "ltc2378: spi_optimize_message failed: %d\n", ret);
                return ret;
        }

        ret = spi_offload_trigger_enable(adc->offload, adc->offload_trigger,
                                        &adc->offload_trigger_config);
        if (ret) {
                printk(KERN_ERR "ltc2378: failed to enable SPI offload trigger: %d\n", ret);
                spi_unoptimize_message(&adc->offload_msg);
                return ret;
        }

        return 0;
}

static int ltc2378_buffer_predisable(struct iio_dev *indio_dev)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);

        spi_offload_trigger_disable(adc->offload, adc->offload_trigger);

        spi_unoptimize_message(&adc->offload_msg);

        return 0;
}

static const struct iio_buffer_setup_ops ltc2378_buffer_ops = {
        .postenable = &ltc2378_buffer_postenable,
        .predisable = &ltc2378_buffer_predisable,
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
        if (!indio_dev) {
                printk(KERN_ERR "ltc2378: failed to allocate iio device\n");
                return -ENOMEM;
        }
        
        adc =  iio_priv(indio_dev);
        adc->spi = spi;
        adc->vref = devm_regulator_get(&spi->dev, "vref");

        if (IS_ERR(adc->vref)) {
                printk(KERN_ERR "ltc2378: failed to get vref regulator: %ld\n", PTR_ERR(adc->vref));
                return PTR_ERR(adc->vref);
        }

        ret = regulator_enable(adc->vref);

        if (ret) {
                printk(KERN_ERR "ltc2378: failed to enable VREF regulator: %d\n", ret);
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

        /* Get SPI offload support */
        adc->offload = devm_spi_offload_get(&spi->dev, spi, &ltc2378_offload_config);
        if (IS_ERR(adc->offload)) {
                dev_err(&spi->dev, "Failed to get SPI offload\n");
                return PTR_ERR(adc->offload);
        }

        /* Get offload trigger - but don't let it override HDL PWM settings */
        adc->offload_trigger = devm_spi_offload_trigger_get(&spi->dev,
                                                            adc->offload,
                                                            SPI_OFFLOAD_TRIGGER_PERIODIC);
        if (IS_ERR(adc->offload_trigger)) {
                dev_err(&spi->dev, "Failed to get offload trigger\n");
                return PTR_ERR(adc->offload_trigger);
        }

        /* Configure trigger timing */
        adc->offload_trigger_config.type = SPI_OFFLOAD_TRIGGER_PERIODIC;
        adc->offload_trigger_config.periodic.frequency_hz = adc->info->max_rate;

        indio_dev->name = adc->info->name;
        indio_dev->info = &ltc2378_iio_info;
        indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
        indio_dev->setup_ops = &ltc2378_buffer_ops;

        struct dma_chan *rx_dma;
        rx_dma = devm_spi_offload_rx_stream_request_dma_chan(&spi->dev, adc->offload);
        if (IS_ERR(rx_dma)) {
                printk(KERN_ERR "ltc2378: failed to get offload RX DMA: %ld\n", PTR_ERR(rx_dma));
                return PTR_ERR(rx_dma);
        }

        ret = devm_iio_dmaengine_buffer_setup_with_handle(&spi->dev, indio_dev,
                                                          rx_dma, IIO_BUFFER_DIRECTION_IN);
        if (ret) {
                printk(KERN_ERR "ltc2378: devm_iio_dmaengine_buffer_setup_with_handle failed: %d\n", ret);
                return ret;
        }

        ret = ltc2378_set_samp_freq(adc, adc->info->max_rate);
        if (ret)
                return ret;

        ret = devm_iio_device_register(&spi->dev, indio_dev);
        if (ret) {
                printk(KERN_ERR "ltc2378: devm_iio_device_register failed: %d\n", ret);
                return ret;
        }
        
        return 0;
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
