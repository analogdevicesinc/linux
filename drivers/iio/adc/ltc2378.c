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
 
 #include <linux/iio/buffer.h>
 #include <linux/iio/buffer-dma.h>
 #include <linux/iio/buffer-dmaengine.h>
 #include <linux/iio/iio.h>
 #include <linux/iio/sysfs.h>
 #include <linux/io.h>

/* LTC2378 timing constants from datasheet */
#define LTC2378_TCNV_HIGH_NS  20  /* Minimum CNV pulse width for reliable operation */

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
        struct pwm_device *cnv_pwm;  /* Direct PWM control for pulse width */
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
        struct pwm_waveform conv_wf;  /* PWM waveform like AD4630 */

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
            .value = 0,             // 600ns delay after CS assertion
            //.value = 690,
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
        struct spi_offload_trigger_config config = {
                .type = SPI_OFFLOAD_TRIGGER_PERIODIC,
                .periodic = {
                        .frequency_hz = freq,
                },
        };
        int ret;

        printk(KERN_INFO "ltc2378: freq input = %d\n", freq);
        printk(KERN_INFO "ltc2378: max_rate = %d\n", adc->info->max_rate);
        freq = clamp(freq, 1, adc->info->max_rate);
        printk(KERN_INFO "ltc2378: freq clamped = %d\n", freq);
        
        config.periodic.frequency_hz = freq;
        
        /* Store frequency directly - no trigger validation needed with direct PWM */
        adc->samp_freq = freq;
        adc->offload_trigger_config = config;
        
        printk(KERN_INFO "ltc2378: frequency set = %d\n", adc->samp_freq);

        /* Configure PWM waveform like AD4630 does */
        if (adc->cnv_pwm) {
                struct pwm_waveform conv_wf = { };
                u64 target = LTC2378_TCNV_HIGH_NS;  /* Start with 20ns */
                
                /* Set period based on validated frequency - use ARM32-safe division */
                conv_wf.period_length_ns = div_u64(NSEC_PER_SEC + (config.periodic.frequency_hz >> 1), config.periodic.frequency_hz);
                
                /* Set narrow pulse width like AD4630 algorithm */
                do {
                        conv_wf.duty_length_ns = target;
                        ret = pwm_round_waveform_might_sleep(adc->cnv_pwm, &conv_wf);
                        if (ret) {
                                printk(KERN_ERR "ltc2378: PWM round waveform failed: %d\n", ret);
                                return ret;
                        }
                        target += 10;
                } while (conv_wf.duty_length_ns < LTC2378_TCNV_HIGH_NS);  /* Ensure at least 20ns */
                
                /* Apply the waveform */
                ret = pwm_set_waveform_might_sleep(adc->cnv_pwm, &conv_wf, false);
                if (ret) {
                        printk(KERN_ERR "ltc2378: failed to set PWM waveform: %d\n", ret);
                        return ret;
                }
                
                printk(KERN_INFO "ltc2378: PWM waveform set - period=%llu ns, duty=%llu ns\n",
                       conv_wf.period_length_ns, conv_wf.duty_length_ns);
        } else {
                printk(KERN_INFO "ltc2378: No PWM device available for waveform control\n");
        }

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

/* ltc2378_buffer function no longer needed with new offload API */

static int ltc2378_buffer_postenable(struct iio_dev *indio_dev)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);
        int ret;
        
        printk(KERN_INFO "ltc2378: buffer_postenable called\n");

        /* Setup offload transfer */
        printk(KERN_INFO "ltc2378: setting up offload transfer\n");
        adc->offload_xfer.rx_buf = NULL;  /* For streaming, DMA framework handles buffer */
        adc->offload_xfer.tx_buf = NULL;
        adc->offload_xfer.len = 4;  /* 4 bytes for 20-bit data (32-bit aligned) */
        adc->offload_xfer.bits_per_word = 20;
        adc->offload_xfer.speed_hz = adc->info->sclk_rate;
        adc->offload_xfer.offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;  /* Enable RX streaming */
        
        /* Setup delay after CS assertion */
        adc->offload_xfer.delay.value = 0;  /* Start with no delay for testing */
        adc->offload_xfer.delay.unit = SPI_DELAY_UNIT_NSECS;

        /* Initialize message with offload */
        printk(KERN_INFO "ltc2378: initializing SPI message with transfers\n");
        spi_message_init_with_transfers(&adc->offload_msg, &adc->offload_xfer, 1);
        adc->offload_msg.offload = adc->offload;

        /* Optimize message for offload */
        printk(KERN_INFO "ltc2378: optimizing SPI message\n");
        ret = spi_optimize_message(adc->spi, &adc->offload_msg);
        if (ret) {
                printk(KERN_ERR "ltc2378: spi_optimize_message failed: %d\n", ret);
                return ret;
        }
        printk(KERN_INFO "ltc2378: SPI message optimized successfully\n");

        /* Enable offload trigger - but PWM pulse width controlled by HDL */
        printk(KERN_INFO "ltc2378: enabling SPI offload trigger\n");
        ret = spi_offload_trigger_enable(adc->offload, adc->offload_trigger,
                                        &adc->offload_trigger_config);
        if (ret) {
                printk(KERN_ERR "ltc2378: failed to enable SPI offload trigger: %d\n", ret);
                spi_unoptimize_message(&adc->offload_msg);
                return ret;
        }
        
        /* CRITICAL: Set exact PWM waveform like AD4630 - before trigger enable */
        if (adc->cnv_pwm) {
                struct pwm_waveform conv_wf = { };
                u64 target = LTC2378_TCNV_HIGH_NS;  /* Start with 20ns */
                
                /* Set period from frequency - use ARM32-safe division */
                conv_wf.period_length_ns = div_u64(NSEC_PER_SEC + (adc->samp_freq >> 1), adc->samp_freq);
                
                /* Set narrow pulse width like AD4630 does */
                do {
                        conv_wf.duty_length_ns = target;
                        ret = pwm_round_waveform_might_sleep(adc->cnv_pwm, &conv_wf);
                        if (ret) {
                                printk(KERN_ERR "ltc2378: PWM round waveform failed: %d\n", ret);
                                break;
                        }
                        target += 10;
                } while (conv_wf.duty_length_ns < LTC2378_TCNV_HIGH_NS);  /* Ensure at least 20ns */
                
                /* Apply the waveform */
                ret = pwm_set_waveform_might_sleep(adc->cnv_pwm, &conv_wf, false);
                if (ret == 0) {
                        printk(KERN_INFO "ltc2378: Set PWM waveform - period=%llu ns, duty=%llu ns\n", 
                               conv_wf.period_length_ns, conv_wf.duty_length_ns);
                } else {
                        printk(KERN_ERR "ltc2378: PWM set waveform failed: %d\n", ret);
                }
        } else {
                printk(KERN_INFO "ltc2378: No PWM device available for waveform control\n");
        }
        
        printk(KERN_INFO "ltc2378: SPI offload trigger enabled successfully\n");

        return 0;
}

static int ltc2378_buffer_predisable(struct iio_dev *indio_dev)
{
        struct ltc2378_adc *adc = iio_priv(indio_dev);

        printk(KERN_INFO "ltc2378: buffer_predisable called\n");
        
        /* Disable offload trigger */
        printk(KERN_INFO "ltc2378: disabling SPI offload trigger\n");
        spi_offload_trigger_disable(adc->offload, adc->offload_trigger);
        
        /* Unoptimize message */
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

static void ltc2378_pwm_disable(void *data)
{
        pwm_disable(data);
}

static int ltc2378_probe(struct spi_device *spi)
{
        struct ltc2378_adc *adc;
        struct iio_dev *indio_dev;
        struct clk *ref_clk;
        int ret;

        printk(KERN_INFO "ltc2378: probe start\n");

        indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
        if (!indio_dev) {
                printk(KERN_ERR "ltc2378: failed to allocate iio device\n");
                return -ENOMEM;
        }
        
        printk(KERN_INFO "ltc2378: iio device allocated\n");
        adc =  iio_priv(indio_dev);
        adc->spi = spi;
        printk(KERN_INFO "ltc2378: getting vref regulator\n");
        adc->vref = devm_regulator_get(&spi->dev, "vref");
        if (IS_ERR(adc->vref)) {
                printk(KERN_ERR "ltc2378: failed to get vref regulator: %ld\n", PTR_ERR(adc->vref));
                return PTR_ERR(adc->vref);
        }
        printk(KERN_INFO "ltc2378: enabling vref regulator\n");
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
        
        /* Get PWM for direct pulse width control (like AD4630) */
        adc->cnv_pwm = devm_pwm_get(&spi->dev, "cnv");
        if (IS_ERR(adc->cnv_pwm)) {
                /* PWM is optional - offload framework can still work without it */
                printk(KERN_INFO "ltc2378: No direct PWM control, using offload defaults\n");
                adc->cnv_pwm = NULL;
        } else {
                printk(KERN_INFO "ltc2378: PWM device acquired for pulse width control\n");
                ret = devm_add_action_or_reset(&spi->dev, ltc2378_pwm_disable, adc->cnv_pwm);
                if (ret)
                        return ret;
        }

        printk("ltc2378 - before device_get_match_data");
        adc->info = device_get_match_data(&spi->dev);
        if (!adc->info) {
                adc->info = (struct ltc2378_chip_info *)
                                spi_get_device_id(spi)->driver_data;
                if (!adc->info)
                        return ret;
        }
        printk("ltc2378 - before parse channels");
        ret = ltc2378_parse_channels(indio_dev);
        if (ret)
                return -EINVAL;

        printk("ltc2378 - before devm_spi_offload_get");
        /* Get SPI offload support */
        adc->offload = devm_spi_offload_get(&spi->dev, spi, &ltc2378_offload_config);
        if (IS_ERR(adc->offload)) {
                dev_err(&spi->dev, "Failed to get SPI offload\n");
                return PTR_ERR(adc->offload);
        }
        
        printk("ltc2378 - before offload trigger get");
        /* Get offload trigger - but don't let it override HDL PWM settings */
        adc->offload_trigger = devm_spi_offload_trigger_get(&spi->dev,
                                                            adc->offload,
                                                            SPI_OFFLOAD_TRIGGER_PERIODIC);
        if (IS_ERR(adc->offload_trigger)) {
                dev_err(&spi->dev, "Failed to get offload trigger\n");
                return PTR_ERR(adc->offload_trigger);
        }
        
        printk("ltc2378 - before configure trigger timing");
        /* Configure trigger timing */
        adc->offload_trigger_config.type = SPI_OFFLOAD_TRIGGER_PERIODIC;
        adc->offload_trigger_config.periodic.frequency_hz = adc->info->max_rate;
        adc->offload_trigger_config.periodic.offset_ns = 0;  /* No offset initially */

        printk("ltc2378 - before indio_dev assignments");
        indio_dev->name = adc->info->name;
        indio_dev->info = &ltc2378_iio_info;
        indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
        indio_dev->setup_ops = &ltc2378_buffer_ops;
        printk("ltc2378 - before getting offload RX DMA");
        struct dma_chan *rx_dma;
        rx_dma = devm_spi_offload_rx_stream_request_dma_chan(&spi->dev, adc->offload);
        if (IS_ERR(rx_dma)) {
                printk(KERN_ERR "ltc2378: failed to get offload RX DMA: %ld\n", PTR_ERR(rx_dma));
                return PTR_ERR(rx_dma);
        }
        
        printk("ltc2378 - before devm_iio_dmaengine_buffer_setup_with_handle");
        ret = devm_iio_dmaengine_buffer_setup_with_handle(&spi->dev, indio_dev,
                                                          rx_dma, IIO_BUFFER_DIRECTION_IN);
        if (ret) {
                printk(KERN_ERR "ltc2378: devm_iio_dmaengine_buffer_setup_with_handle failed: %d\n", ret);
                return ret;
        }
        printk("ltc2378 - DMA buffer setup success");
        printk("ltc2378 - before set_sam_freq"); 
        ret = ltc2378_set_samp_freq(adc, adc->info->max_rate);
        if (ret)
                return ret;

        dev_info(&spi->dev, "ltc2378 - bits_per_word_mask: 0x%x\n", spi->controller->bits_per_word_mask); // check if SPI supports 20 bit transfers
        
        printk("ltc2378 - before devm_iio_device_register");
        ret = devm_iio_device_register(&spi->dev, indio_dev);
        if (ret) {
                printk(KERN_ERR "ltc2378: devm_iio_device_register failed: %d\n", ret);
                return ret;
        }
        printk("ltc2378 - probe completed successfully!");
        
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
