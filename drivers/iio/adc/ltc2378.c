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

/* Conversion time (TCONV) */
#define LTC237X_TCONV_2MSPS_NS		322	/* 2 MSPS parts: LTC2380-16, LTC2370-16 */
#define LTC237X_TCONV_1_6MSPS_NS	412	/* 1.6 MSPS parts: LTC2379-18, LTC2369-18 */
#define LTC237X_TCONV_1MSPS_NS		527	/* 1 MSPS parts: LTC2378/2368/2338 */
#define LTC237X_TCONV_1MSPS_20BIT_NS	675	/* 1 MSPS 20-bit: LTC2378-20 */
#define LTC237X_TCONV_500KSPS_NS	1500	/* 500 kSPS parts: LTC2377/2367 */
#define LTC237X_TCONV_250KSPS_NS	3000	/* 250 kSPS parts: LTC2376/2364 */

#define LTC2378_TDSDOBUSYL_NS		5
#define LTC2378_TBUSYLH_NS		13
#define LTC2378_TCNV_HIGH_NS		40	/* Minimum CNV high time (with margin) */
#define LTC2378_TRIGGER_TO_SCLK_CYCLES	8	/* SPI clock cycles from trigger to SCLK start -> 9 - 1 margin*/

/* Input mode for different variants */
enum ltc237x_input_mode {
	LTC237X_DIFFERENTIAL,		/* Fully differential (±VREF range) */
	LTC237X_PSEUDO_DIFF,		/* Pseudo-differential unipolar (0 to VREF) */
};

/* Device IDs for all supported variants */
enum ltc237x_supported_device_ids {
	/* Fully differential variants */
	ID_LTC2376_16,
	ID_LTC2376_18,
	ID_LTC2376_20,
	ID_LTC2377_16,
	ID_LTC2377_18,
	ID_LTC2377_20,
	ID_LTC2378_16,
	ID_LTC2378_18,
	ID_LTC2378_20,
	ID_LTC2379_18,
	ID_LTC2380_16,
	/* Pseudo-differential unipolar variants */
	ID_LTC2364_16,
	ID_LTC2364_18,
	ID_LTC2367_16,
	ID_LTC2367_18,
	ID_LTC2368_16,
	ID_LTC2368_18,
	ID_LTC2369_18,
	ID_LTC2370_16,
	/* LTC2338 series - similar interface */
	ID_LTC2338_18,
};

struct ltc2378_chip_info {
        const char *name;
        int num_channels;
        int resolution;
        int sclk_rate;
        int max_rate;
        int tconv_ns;
        enum ltc237x_input_mode input_mode;
};

/* SPI offload configuration for LTC2378 */
static const struct spi_offload_config ltc2378_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

/* Chip information for all supported variants */
static const struct ltc2378_chip_info ltc237x_chip_info[] = {
	/* Fully differential 250 kSPS variants */
	[ID_LTC2376_16] = {
		.name = "ltc2376-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 250000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_250KSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	[ID_LTC2376_18] = {
		.name = "ltc2376-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 250000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_250KSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	[ID_LTC2376_20] = {
		.name = "ltc2376-20",
		.resolution = 20,
		.num_channels = 1,
		.max_rate = 250000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_250KSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	/* Fully differential 500 kSPS variants */
	[ID_LTC2377_16] = {
		.name = "ltc2377-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 500000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_500KSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	[ID_LTC2377_18] = {
		.name = "ltc2377-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 500000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_500KSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	[ID_LTC2377_20] = {
		.name = "ltc2377-20",
		.resolution = 20,
		.num_channels = 1,
		.max_rate = 500000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_500KSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	/* Fully differential 1 MSPS variants */
	[ID_LTC2378_16] = {
		.name = "ltc2378-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 1000000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1MSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	[ID_LTC2378_18] = {
		.name = "ltc2378-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 1000000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1MSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	[ID_LTC2378_20] = {
		.name = "ltc2378-20",
		.resolution = 20,
		.num_channels = 1,
		.max_rate = 1000000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1MSPS_20BIT_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	/* Fully differential high-speed variants */
	[ID_LTC2379_18] = {
		.name = "ltc2379-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 1600000,	/* 1.6 MSPS */
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1_6MSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	[ID_LTC2380_16] = {
		.name = "ltc2380-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 2000000,	/* 2 MSPS */
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_2MSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
	/* Pseudo-differential 250 kSPS variants */
	[ID_LTC2364_16] = {
		.name = "ltc2364-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 250000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_250KSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	[ID_LTC2364_18] = {
		.name = "ltc2364-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 250000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_250KSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	/* Pseudo-differential 500 kSPS variants */
	[ID_LTC2367_16] = {
		.name = "ltc2367-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 500000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_500KSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	[ID_LTC2367_18] = {
		.name = "ltc2367-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 500000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_500KSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	/* Pseudo-differential 1 MSPS variants */
	[ID_LTC2368_16] = {
		.name = "ltc2368-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 1000000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1MSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	[ID_LTC2368_18] = {
		.name = "ltc2368-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 1000000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1MSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	/* Pseudo-differential high-speed variants */
	[ID_LTC2369_18] = {
		.name = "ltc2369-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 1600000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1_6MSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	[ID_LTC2370_16] = {
		.name = "ltc2370-16",
		.resolution = 16,
		.num_channels = 1,
		.max_rate = 2000000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_2MSPS_NS,
		.input_mode = LTC237X_PSEUDO_DIFF,
	},
	[ID_LTC2338_18] = {
		.name = "ltc2338-18",
		.resolution = 18,
		.num_channels = 1,
		.max_rate = 1000000,
		.sclk_rate = 70000000,
		.tconv_ns = LTC237X_TCONV_1MSPS_NS,
		.input_mode = LTC237X_DIFFERENTIAL,
	},
};

struct ltc2378_adc {
        const struct ltc2378_chip_info *info;
        struct spi_transfer seq_xfer[1];
        unsigned int cfg;
        unsigned long ref_clk_rate;
        struct spi_device *spi;
        struct regulator *vref;
        int spi_speed_hz;
        int samp_freq;
        int device_id;
        u8 rx_buf[4]; // for 32-bit data

        /* PWM control for CNV signal (dual PWM architecture) */
        struct pwm_device *conv_trigger;
        struct pwm_waveform conv_wf;

        /* New SPI offload API fields */
        struct spi_transfer offload_xfer[2];
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

static int ltc2378_read_channel(struct ltc2378_adc *adc, unsigned int *val)
{
    int ret;
    u32 raw;
    s32 signed_val;
    unsigned int shift_bits;
    unsigned int mask;

    struct spi_transfer xfer = {
        .rx_buf = &adc->spi_rx_word,   // Aligned 32-bit buffer
        .len = 4,                      // Always 4 bytes (32-bit aligned)
        .bits_per_word = adc->info->resolution,  // Dynamic based on chip
        .speed_hz = adc->info->sclk_rate,
    };

    ret = spi_sync_transfer(adc->spi, &xfer, 1);
    if (ret)
        return ret;

    raw = be32_to_cpu(adc->spi_rx_word); // Handle byte order
    shift_bits = 32 - adc->info->resolution;
    raw >>= shift_bits;                  // shift to LSB based on resolution
    mask = (1 << adc->info->resolution) - 1;
    raw &= mask;                         // Mask to keep only valid bits

    if (adc->info->input_mode == LTC237X_DIFFERENTIAL) {
        /* Two's complement for differential mode */
        unsigned int sign_bit = adc->info->resolution - 1;
        if (raw & BIT(sign_bit))
            signed_val = raw | ~mask;    // Sign extend
        else
            signed_val = raw;
    } else {
        /* Straight binary for pseudo-differential unipolar */
        signed_val = raw;
    }

    *val = signed_val;

    return 0;
}

static int ltc2378_pwm_get(struct ltc2378_adc *adc)
{
	struct device *dev = &adc->spi->dev;

	/* Get PWM channel 1 for CNV control from device tree */
	adc->conv_trigger = devm_pwm_get(dev, "cnv");
	if (IS_ERR(adc->conv_trigger))
		return dev_err_probe(dev, PTR_ERR(adc->conv_trigger),
				     "Failed to get cnv pwm\n");

	/* Preemptively disable - only enable with buffer */
	pwm_disable(adc->conv_trigger);

	return 0;
}

static int ltc2378_set_samp_freq(struct ltc2378_adc *adc, int freq)
{
	struct spi_offload_trigger_config *config = &adc->offload_trigger_config;
	struct pwm_waveform conv_wf = { };
	u64 target = LTC2378_TCNV_HIGH_NS;
	int ret;

	if (!in_range(freq, 1, adc->info->max_rate))
		return -EINVAL;

	/* Configure PWM1 (CNV) waveform */
	conv_wf.period_length_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, freq);

	/*
	 * Ensure CNV high time meets minimum requirement (20ns).
	 * The PWM hardware may round the duty cycle, so iterate
	 * until we get at least the minimum required high time.
	 */
	do {
		conv_wf.duty_length_ns = target;
		ret = pwm_round_waveform_might_sleep(adc->conv_trigger, &conv_wf);
		if (ret)
			return ret;
		target += 10;  /* Increment by PWM clock period (10ns) */
	} while (conv_wf.duty_length_ns < LTC2378_TCNV_HIGH_NS);

	/*
	 * Configure PWM0 (SPI offload trigger).
	 * Trigger should fire after CNV falls + conversion time + BUSY timing.
	 * Account for the delay from trigger to SCLK start (hardware pipeline).
	 *
	 * CRITICAL: Use the SAME period as CNV PWM to avoid 1-cycle mismatch!
	 * Convert back from period to frequency for the SPI offload API.
	 *
	 * Total timing needed: TCONV (part-specific) + TBUSYLH (13ns) + TDSDOBUSYL (5ns)
	 * Subtract trigger-to-SCLK delay: 8 SPI clock cycles (9 - 1 margin clock)
	 */
	u64 spi_clk_period_ns = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC, adc->ref_clk_rate);
	u64 trigger_to_sclk_delay_ns = LTC2378_TRIGGER_TO_SCLK_CYCLES * spi_clk_period_ns;
	u64 actual_freq_hz = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC, conv_wf.period_length_ns);

	dev_info(&adc->spi->dev,
		 "SPI clock: %lu Hz, period: %llu ns, trigger-to-SCLK delay: %llu ns, PWM period: %llu ns (actual freq: %llu Hz)\n",
		 adc->ref_clk_rate, spi_clk_period_ns, trigger_to_sclk_delay_ns,
		 conv_wf.period_length_ns, actual_freq_hz);

	config->periodic.frequency_hz = actual_freq_hz;
	config->periodic.offset_ns = adc->info->tconv_ns + LTC2378_TBUSYLH_NS +
				      LTC2378_TDSDOBUSYL_NS - trigger_to_sclk_delay_ns;

	ret = spi_offload_trigger_validate(adc->offload_trigger, config);
	if (ret)
		return ret;

	/* Store configuration for later use in buffer enable */
	adc->conv_wf = conv_wf;
	adc->samp_freq = freq;

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

                /* Scale depends on input mode:
                 * - Differential: ±VREF range = 2*VREF total
                 * - Pseudo-differential: 0 to VREF = VREF total
                 */
                if (adc->info->input_mode == LTC237X_DIFFERENTIAL)
                        *val = 2 * (ret / 1000); // convert uV to mV, factor of 2
                else
                        *val = ret / 1000;       // convert uV to mV, no factor

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

static int ltc2378_prepare_offload_message(struct device *dev,
					   struct ltc2378_adc *adc)
{
	/*
	 * No initial delay needed - PWM timing controls when trigger fires.
	 * The trigger offset in ltc2378_set_samp_freq() ensures proper timing.
	 */
	adc->offload_xfer[0].delay.value = 0;
	adc->offload_xfer[0].delay.unit = SPI_DELAY_UNIT_NSECS;

        /* Setup data read transfer */
        adc->offload_xfer[1].rx_buf = NULL;  /* For streaming, DMA framework handles buffer */
        adc->offload_xfer[1].tx_buf = NULL;
        adc->offload_xfer[1].len = 4;  /* Always 4 bytes (32-bit aligned) */
        adc->offload_xfer[1].bits_per_word = adc->info->resolution;  /* Dynamic resolution */
        adc->offload_xfer[1].offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;  /* Enable RX streaming */
        /* Additional SLEEP instruction needed for tQUIET timing requirements. */
        //adc->offload_xfer[1].delay.value = 20;
        //adc->offload_xfer[1].delay.unit = SPI_DELAY_UNIT_NSECS;

        /* Initialize message with offload */
        spi_message_init_with_transfers(&adc->offload_msg, adc->offload_xfer,
					ARRAY_SIZE(adc->offload_xfer));
        adc->offload_msg.offload = adc->offload;

	return devm_spi_optimize_message(dev, adc->spi, &adc->offload_msg);
}

static int ltc2378_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ltc2378_adc *adc = iio_priv(indio_dev);
	int ret;

	spi_bus_lock(adc->spi->controller);

	/* Start PWM1 (CNV) - begins conversions */
	ret = pwm_set_waveform_might_sleep(adc->conv_trigger, &adc->conv_wf, false);
	if (ret)
		goto out_unlock;

	/* Enable PWM0 (trigger) - begins SPI readouts */
	ret = spi_offload_trigger_enable(adc->offload, adc->offload_trigger,
					&adc->offload_trigger_config);
	if (ret)
		goto out_pwm_disable;

	spi_bus_unlock(adc->spi->controller);

	return 0;

out_pwm_disable:
	pwm_disable(adc->conv_trigger);
out_unlock:
	spi_bus_unlock(adc->spi->controller);
	return ret;
}

static int ltc2378_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ltc2378_adc *adc = iio_priv(indio_dev);

	spi_bus_lock(adc->spi->controller);

	/* Disable PWM0 (trigger) first - stops new SPI transactions */
	spi_offload_trigger_disable(adc->offload, adc->offload_trigger);

	/* Stop PWM1 (CNV) - stops conversions */
	pwm_disable(adc->conv_trigger);

	spi_bus_unlock(adc->spi->controller);

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

        /* Setup the single channel dynamically based on chip resolution */
        struct iio_chan_spec *chan;

        chan = devm_kzalloc(&spi->dev, sizeof(*chan), GFP_KERNEL);
        if (!chan)
                return -ENOMEM;

        /* Initialize channel spec based on chip info */
        chan->type = IIO_VOLTAGE;
        chan->indexed = 1;
        chan->channel = 0;
        chan->info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ);
        chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE);
        chan->scan_index = 0;

        /* Set scan type based on chip resolution and input mode */
        chan->scan_type.sign = (adc->info->input_mode == LTC237X_DIFFERENTIAL) ? 's' : 'u';
        chan->scan_type.storagebits = 32;  /* Always 32-bit aligned */
        chan->scan_type.realbits = adc->info->resolution;
        chan->scan_type.shift = 0;
        chan->scan_type.endianness = IIO_CPU;

        indio_dev->channels = chan;
        indio_dev->num_channels = 1;

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

	/* Get PWM for CNV control */
	ret = ltc2378_pwm_get(adc);
	if (ret)
		return ret;

        ret = ltc2378_set_samp_freq(adc, adc->info->max_rate);
        if (ret)
                return ret;

	ret = ltc2378_prepare_offload_message(&spi->dev, adc);
	if (ret)
		return ret;

        ret = devm_iio_device_register(&spi->dev, indio_dev);
        if (ret) {
                printk(KERN_ERR "ltc2378: devm_iio_device_register failed: %d\n", ret);
                return ret;
        }
        
        return 0;
}

static const struct of_device_id ltc237x_of_match[] = {
        /* Fully differential variants */
        { .compatible = "adi,ltc2376-16", .data = &ltc237x_chip_info[ID_LTC2376_16] },
        { .compatible = "adi,ltc2376-18", .data = &ltc237x_chip_info[ID_LTC2376_18] },
        { .compatible = "adi,ltc2376-20", .data = &ltc237x_chip_info[ID_LTC2376_20] },
        { .compatible = "adi,ltc2377-16", .data = &ltc237x_chip_info[ID_LTC2377_16] },
        { .compatible = "adi,ltc2377-18", .data = &ltc237x_chip_info[ID_LTC2377_18] },
        { .compatible = "adi,ltc2377-20", .data = &ltc237x_chip_info[ID_LTC2377_20] },
        { .compatible = "adi,ltc2378-16", .data = &ltc237x_chip_info[ID_LTC2378_16] },
        { .compatible = "adi,ltc2378-18", .data = &ltc237x_chip_info[ID_LTC2378_18] },
        { .compatible = "adi,ltc2378-20", .data = &ltc237x_chip_info[ID_LTC2378_20] },
        { .compatible = "adi,ltc2379-18", .data = &ltc237x_chip_info[ID_LTC2379_18] },
        { .compatible = "adi,ltc2380-16", .data = &ltc237x_chip_info[ID_LTC2380_16] },
        /* Pseudo-differential variants */
        { .compatible = "adi,ltc2364-16", .data = &ltc237x_chip_info[ID_LTC2364_16] },
        { .compatible = "adi,ltc2364-18", .data = &ltc237x_chip_info[ID_LTC2364_18] },
        { .compatible = "adi,ltc2367-16", .data = &ltc237x_chip_info[ID_LTC2367_16] },
        { .compatible = "adi,ltc2367-18", .data = &ltc237x_chip_info[ID_LTC2367_18] },
        { .compatible = "adi,ltc2368-16", .data = &ltc237x_chip_info[ID_LTC2368_16] },
        { .compatible = "adi,ltc2368-18", .data = &ltc237x_chip_info[ID_LTC2368_18] },
        { .compatible = "adi,ltc2369-18", .data = &ltc237x_chip_info[ID_LTC2369_18] },
        { .compatible = "adi,ltc2370-16", .data = &ltc237x_chip_info[ID_LTC2370_16] },
        { .compatible = "adi,ltc2338-18", .data = &ltc237x_chip_info[ID_LTC2338_18] },
        { },
};
MODULE_DEVICE_TABLE(of, ltc237x_of_match);

static const struct spi_device_id ltc237x_spi_id[] = {
        /* Fully differential */
        { "ltc2376-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2376_16] },
        { "ltc2376-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2376_18] },
        { "ltc2376-20", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2376_20] },
        { "ltc2377-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2377_16] },
        { "ltc2377-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2377_18] },
        { "ltc2377-20", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2377_20] },
        { "ltc2378-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2378_16] },
        { "ltc2378-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2378_18] },
        { "ltc2378-20", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2378_20] },
        { "ltc2379-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2379_18] },
        { "ltc2380-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2380_16] },
        /* Pseudo-differential */
        { "ltc2364-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2364_16] },
        { "ltc2364-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2364_18] },
        { "ltc2367-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2367_16] },
        { "ltc2367-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2367_18] },
        { "ltc2368-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2368_16] },
        { "ltc2368-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2368_18] },
        { "ltc2369-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2369_18] },
        { "ltc2370-16", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2370_16] },
        { "ltc2338-18", (kernel_ulong_t)&ltc237x_chip_info[ID_LTC2338_18] },
        { },
};
MODULE_DEVICE_TABLE(spi, ltc237x_spi_id);

static struct spi_driver ltc237x_driver = {
        .driver = {
                .name = "ltc237x",
                .of_match_table = ltc237x_of_match
        },
        .probe = ltc2378_probe,
        .id_table = ltc237x_spi_id,
};
module_spi_driver(ltc237x_driver);

MODULE_AUTHOR("Pop Ioan Daniel <pop.ioan-daniel@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC237x/236x ADC family driver");
MODULE_LICENSE("GPL");
