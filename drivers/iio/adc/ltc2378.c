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
        u8 rx_buf[3]; // for 24-bit data

        /*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

         u8 spi_rx_data[4] __aligned(IIO_DMA_MINALIGN);
         u8 spi_tx_data[4];
};

static const struct iio_chan_spec ltc2378_chan = {
        .type = IIO_VOLTAGE,
        .indexed = 1,
        .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE),
        .scan_type.sign = 'u',
        .scan_type.storagebits = 32,
        .scan_type.realbits = 20,
};

static int ltc2378_read_channel(struct ltc2378_adc *adc, unsigned int *val)
{
	struct spi_transfer xfer = {
		.rx_buf = adc->rx_buf,
		.len = 3, // LTC2378 outputs 20 bits, aligned MSB-first in a 24-bit word
		.speed_hz = adc->info->sclk_rate,
		.bits_per_word = 8, // Standard SPI byte-wise transfer
	};
	int ret;
	u32 raw = 0;

	// Read 3 bytes (24 bits)
	ret = spi_sync_transfer(adc->spi, &xfer, 1);
	if (ret)
		return ret;

	// Combine bytes into 24-bit word, MSB first
	raw = (adc->rx_buf[0] << 16) | (adc->rx_buf[1] << 8) | adc->rx_buf[2];

	// The 20-bit result is in the top 20 bits of this 24-bit word
	*val = raw >> 4;

	return 0;
}


static int ltc2378_set_samp_freq(struct ltc2378_adc *adc, int freq)
{
        unsigned long long ref_clk_period_ns;
        struct pwm_state cnv_state;
        int ret;
        u32 rem;

        freq = clamp(freq, 1, adc->info->max_rate); // freq between 1 Hz and max_rate
        ref_clk_period_ns = DIV_ROUND_UP(NSEC_PER_SEC, adc->ref_clk_rate);

        pwm_get_state(adc->cnv, &cnv_state);

        cnv_state.duty_cycle = ref_clk_period_ns;
        cnv_state.enabled = true;

        cnv_state.period = div_u64_rem((u64)DIV_ROUND_UP(adc->ref_clk_rate, freq) * NSEC_PER_SEC, // compute PWM period
                                        adc->ref_clk_rate, &rem);
        if (rem)
                cnv_state.period += 1;

        ret = pwm_apply_might_sleep(adc->cnv, &cnv_state);  // apply calculated PWM state
        if (ret)
                return ret;

        adc->samp_freq = freq;

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
        struct spi_transfer *xfer = &adc->seq_xfer[0]; // only 1 channel

        /* Clear the SPI message structure */
        spi_message_init(msg);

        /*Setup transfer - reading 3 bytes*/

        xfer->rx_buf = adc->rx_buf;
        xfer->len = 3; //sends 20 bits in 24-bit frame (mSB-aligned)
        xfer->cs_change = 0; // Release chip select after transfer
        xfer->word_delay.value = 2; //insert a 2us delay between the words during SPI transfer
        xfer->word_delay.unit = SPI_DELAY_UNIT_USECS;

        /* Add the transfer to the message */
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
        printk("ltc2378 - before vref");
        adc->vref = devm_regulator_get(&spi->dev, "vref"); // check with devicetree
        if (IS_ERR(adc->vref))
                return PTR_ERR(adc->vref);
        printk("ltc2378 - before enable vref");
        ret = regulator_enable(adc->vref);
        if (ret) {
                dev_err(&spi->dev, "Failed to enable VREF regulator");
                return ret;
        }

        ret = devm_add_action_or_reset(&spi->dev, ltc2378_reg_disable, adc->vref);
        if (ret)
                return ret;
        printk("ltc2378 - before ref_clk");
        ref_clk = devm_clk_get(&spi->dev, "spi_clk"); // check with devicetree
        if (IS_ERR(ref_clk))
                return PTR_ERR(ref_clk);
        printk("ltc2378 - before enable ref_clk");
        ret = clk_prepare_enable(ref_clk);
        if (ret)
                return ret;
        
        ret = devm_add_action_or_reset(&spi->dev, ltc2378_clk_disable, ref_clk);
        if (ret)
                return ret;
        
        adc->ref_clk_rate = clk_get_rate(ref_clk);
        printk("ltc2378 - before cnv");
        adc->cnv = devm_pwm_get(&spi->dev, "cnv"); //check with devicetree
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
        printk("ltc2378 - before parse channel");
        ret = ltc2378_parse_channels(indio_dev);
        if (ret)
                return -EINVAL;
        
        indio_dev->name = adc->info->name;
        indio_dev->info = &ltc2378_iio_info;
        indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
        indio_dev->setup_ops = &ltc2378_buffer_ops;
        printk("ltc2378 - after indio_dev set");
        ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
                                              indio_dev, "rx");
        if (ret)
                return ret;
        printk("ltc2378 - before set_samp_freq");
        ret = ltc2378_set_samp_freq(adc, adc->info->max_rate);
        if (ret)
                return ret;
        printk("ltc2378 - after set_samp_freq");
                
        printk("ltc2378 - before exit probe");
        
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
