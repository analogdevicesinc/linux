// SPDX-License-Identifier: GPL-2.0
/*
 * AD7606 SPI ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/units.h>
#include <linux/util_macros.h>

#include <linux/iio/backend.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "ad7606.h"

#define AD7606_CALIB_GAIN_MIN	0
#define AD7606_CALIB_GAIN_STEP	1024
#define AD7606_CALIB_GAIN_MAX	(63 * AD7606_CALIB_GAIN_STEP)

/*
 * Scales are computed as 5000/32768 and 10000/32768 respectively,
 * so that when applied to the raw values they provide mV values.
 * The scale arrays are kept as IIO_VAL_INT_PLUS_MICRO, so index
 * X is the integer part and X + 1 is the fractional part.
 */
static const unsigned int ad7606_16bit_hw_scale_avail[2][2] = {
	{ 0, 152588 }, { 0, 305176 }
};

static const unsigned int ad7606_18bit_hw_scale_avail[2][2] = {
	{ 0, 38147 }, { 0, 76294 }
};

static const unsigned int ad7606c_16bit_single_ended_unipolar_scale_avail[3][2] = {
	{ 0, 76294 }, { 0, 152588 }, { 0, 190735 }
};

static const unsigned int ad7606c_16bit_single_ended_bipolar_scale_avail[5][2] = {
	{ 0, 76294 }, { 0, 152588 }, { 0, 190735 }, { 0, 305176 }, { 0, 381470 }
};

static const unsigned int ad7606c_16bit_differential_bipolar_scale_avail[4][2] = {
	{ 0, 152588 }, { 0, 305176 }, { 0, 381470 }, { 0, 610352 }
};

static const unsigned int ad7606c_18bit_single_ended_unipolar_scale_avail[3][2] = {
	{ 0, 19073 }, { 0, 38147 }, { 0, 47684 }
};

static const unsigned int ad7606c_18bit_single_ended_bipolar_scale_avail[5][2] = {
	{ 0, 19073 }, { 0, 38147 }, { 0, 47684 }, { 0, 76294 }, { 0, 95367 }
};

static const unsigned int ad7606c_18bit_differential_bipolar_scale_avail[4][2] = {
	{ 0, 38147 }, { 0, 76294 }, { 0, 95367 }, { 0, 152588 }
};

static const unsigned int ad7606_16bit_sw_scale_avail[3][2] = {
	{ 0, 76293 }, { 0, 152588 }, { 0, 305176 }
};

static const unsigned int ad7607_hw_scale_avail[2][2] = {
	{ 0, 610352 }, { 1, 220703 }
};

static const unsigned int ad7609_hw_scale_avail[2][2] = {
	{ 0, 152588 }, { 0, 305176 }
};

static const unsigned int ad7606_oversampling_avail[7] = {
	1, 2, 4, 8, 16, 32, 64,
};

static const unsigned int ad7606b_oversampling_avail[9] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256,
};

static const unsigned int ad7616_oversampling_avail[8] = {
	1, 2, 4, 8, 16, 32, 64, 128,
};

static const int ad7606_calib_offset_avail[3] = {
	-128, 1, 127,
};

static const int ad7606c_18bit_calib_offset_avail[3] = {
	-512, 4, 508,
};

static const int ad7606b_calib_phase_avail[][2] = {
	{ 0, 0 }, { 0, 1250 }, { 0, 318750 },
};

static const int ad7606c_calib_phase_avail[][2] = {
	{ 0, 0 }, { 0, 1000 }, { 0, 255000 },
};

static int ad7606c_18bit_chan_scale_setup(struct iio_dev *indio_dev,
					  struct iio_chan_spec *chan);
static int ad7606c_16bit_chan_scale_setup(struct iio_dev *indio_dev,
					  struct iio_chan_spec *chan);
static int ad7606_16bit_chan_scale_setup(struct iio_dev *indio_dev,
					 struct iio_chan_spec *chan);
static int ad7607_chan_scale_setup(struct iio_dev *indio_dev,
				   struct iio_chan_spec *chan);
static int ad7608_chan_scale_setup(struct iio_dev *indio_dev,
				   struct iio_chan_spec *chan);
static int ad7609_chan_scale_setup(struct iio_dev *indio_dev,
				   struct iio_chan_spec *chan);
static int ad7616_sw_mode_setup(struct iio_dev *indio_dev);
static int ad7606b_sw_mode_setup(struct iio_dev *indio_dev);

const struct ad7606_chip_info ad7605_4_info = {
	.max_samplerate = 300 * KILO,
	.name = "ad7605-4",
	.bits = 16,
	.num_adc_channels = 4,
	.scale_setup_cb = ad7606_16bit_chan_scale_setup,
};
EXPORT_SYMBOL_NS_GPL(ad7605_4_info, "IIO_AD7606");

const struct ad7606_chip_info ad7606_8_info = {
	.max_samplerate = 200 * KILO,
	.name = "ad7606-8",
	.bits = 16,
	.num_adc_channels = 8,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7606_16bit_chan_scale_setup,
};
EXPORT_SYMBOL_NS_GPL(ad7606_8_info, "IIO_AD7606");

const struct ad7606_chip_info ad7606_6_info = {
	.max_samplerate = 200 * KILO,
	.name = "ad7606-6",
	.bits = 16,
	.num_adc_channels = 6,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7606_16bit_chan_scale_setup,
	.offload_storagebits = 32,
};
EXPORT_SYMBOL_NS_GPL(ad7606_6_info, "IIO_AD7606");

const struct ad7606_chip_info ad7606_4_info = {
	.max_samplerate = 200 * KILO,
	.name = "ad7606-4",
	.bits = 16,
	.num_adc_channels = 4,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7606_16bit_chan_scale_setup,
	.offload_storagebits = 32,
};
EXPORT_SYMBOL_NS_GPL(ad7606_4_info, "IIO_AD7606");

const struct ad7606_chip_info ad7606b_info = {
	.max_samplerate = 800 * KILO,
	.name = "ad7606b",
	.bits = 16,
	.num_adc_channels = 8,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7606_16bit_chan_scale_setup,
	.sw_setup_cb = ad7606b_sw_mode_setup,
	.offload_storagebits = 32,
	.calib_gain_avail = true,
	.calib_offset_avail = ad7606_calib_offset_avail,
	.calib_phase_avail = ad7606b_calib_phase_avail,
};
EXPORT_SYMBOL_NS_GPL(ad7606b_info, "IIO_AD7606");

const struct ad7606_chip_info ad7606c_16_info = {
	.max_samplerate = 1 * MEGA,
	.name = "ad7606c16",
	.bits = 16,
	.num_adc_channels = 8,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7606c_16bit_chan_scale_setup,
	.sw_setup_cb = ad7606b_sw_mode_setup,
	.offload_storagebits = 32,
	.calib_gain_avail = true,
	.calib_offset_avail = ad7606_calib_offset_avail,
	.calib_phase_avail = ad7606c_calib_phase_avail,
};
EXPORT_SYMBOL_NS_GPL(ad7606c_16_info, "IIO_AD7606");

const struct ad7606_chip_info ad7607_info = {
	.max_samplerate = 200 * KILO,
	.name = "ad7607",
	.bits = 14,
	.num_adc_channels = 8,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7607_chan_scale_setup,
	.offload_storagebits = 32,
};
EXPORT_SYMBOL_NS_GPL(ad7607_info, "IIO_AD7606");

const struct ad7606_chip_info ad7608_info = {
	.max_samplerate = 200 * KILO,
	.name = "ad7608",
	.bits = 18,
	.num_adc_channels = 8,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7608_chan_scale_setup,
	.offload_storagebits = 32,
};
EXPORT_SYMBOL_NS_GPL(ad7608_info, "IIO_AD7606");

const struct ad7606_chip_info ad7609_info = {
	.max_samplerate = 200 * KILO,
	.name = "ad7609",
	.bits = 18,
	.num_adc_channels = 8,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7609_chan_scale_setup,
	.offload_storagebits = 32,
};
EXPORT_SYMBOL_NS_GPL(ad7609_info, "IIO_AD7606");

const struct ad7606_chip_info ad7606c_18_info = {
	.max_samplerate = 1 * MEGA,
	.name = "ad7606c18",
	.bits = 18,
	.num_adc_channels = 8,
	.oversampling_avail = ad7606_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	.scale_setup_cb = ad7606c_18bit_chan_scale_setup,
	.sw_setup_cb = ad7606b_sw_mode_setup,
	.offload_storagebits = 32,
	.calib_gain_avail = true,
	.calib_offset_avail = ad7606c_18bit_calib_offset_avail,
	.calib_phase_avail = ad7606c_calib_phase_avail,
};
EXPORT_SYMBOL_NS_GPL(ad7606c_18_info, "IIO_AD7606");

const struct ad7606_chip_info ad7616_info = {
	.max_samplerate = 1 * MEGA,
	.init_delay_ms = 15,
	.name = "ad7616",
	.bits = 16,
	.num_adc_channels = 16,
	.oversampling_avail = ad7616_oversampling_avail,
	.oversampling_num = ARRAY_SIZE(ad7616_oversampling_avail),
	.os_req_reset = true,
	.scale_setup_cb = ad7606_16bit_chan_scale_setup,
	.sw_setup_cb = ad7616_sw_mode_setup,
	.offload_storagebits = 16,
};
EXPORT_SYMBOL_NS_GPL(ad7616_info, "IIO_AD7606");

int ad7606_reset(struct ad7606_state *st)
{
	if (st->gpio_reset) {
		gpiod_set_value(st->gpio_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		gpiod_set_value(st->gpio_reset, 0);
		return 0;
	}

	return -ENODEV;
}
EXPORT_SYMBOL_NS_GPL(ad7606_reset, "IIO_AD7606");

static int ad7606_16bit_chan_scale_setup(struct iio_dev *indio_dev,
					 struct iio_chan_spec *chan)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci = &st->chan_info[chan->scan_index];

	if (!st->sw_mode_en) {
		/* tied to logic low, analog input range is +/- 5V */
		ci->range = 0;
		ci->scale_avail = ad7606_16bit_hw_scale_avail;
		ci->num_scales = ARRAY_SIZE(ad7606_16bit_hw_scale_avail);
		return 0;
	}

	/* Scale of 0.076293 is only available in sw mode */
	/* After reset, in software mode, ±10 V is set by default */
	ci->range = 2;
	ci->scale_avail = ad7606_16bit_sw_scale_avail;
	ci->num_scales = ARRAY_SIZE(ad7606_16bit_sw_scale_avail);

	return 0;
}

static int ad7606_get_chan_config(struct iio_dev *indio_dev, int ch,
				  bool *bipolar, bool *differential)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci;
	unsigned int num_channels = st->chip_info->num_adc_channels;
	struct device *dev = st->dev;
	int ret;

	*bipolar = false;
	*differential = false;

	device_for_each_child_node_scoped(dev, child) {
		u32 pins[2];
		int reg;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return ret;

		/* channel number (here) is from 1 to num_channels */
		if (reg < 1 || reg > num_channels)
			return -EINVAL;

		/* Loop until we are in the right channel. */
		if (reg != (ch + 1))
			continue;

		*bipolar = fwnode_property_read_bool(child, "bipolar");

		ret = fwnode_property_read_u32_array(child, "diff-channels",
						     pins, ARRAY_SIZE(pins));
		/* Channel is differential, if pins are the same as 'reg' */
		if (ret == 0 && (pins[0] != reg || pins[1] != reg)) {
			dev_err(dev,
				"Differential pins must be the same as 'reg'");
			return -EINVAL;
		}

		*differential = (ret == 0);

		if (*differential && !*bipolar) {
			dev_err(dev,
				"'bipolar' must be added for diff channel %d\n",
				reg);
			return -EINVAL;
		}

		ci = &st->chan_info[reg - 1];

		ci->r_gain = 0;
		ret = fwnode_property_read_u32(child, "adi,rfilter-ohms",
					       &ci->r_gain);
		if (ret == 0 && ci->r_gain > AD7606_CALIB_GAIN_MAX)
			return -EINVAL;

		return 0;
	}

	return 0;
}

static int ad7606c_18bit_chan_scale_setup(struct iio_dev *indio_dev,
					  struct iio_chan_spec *chan)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci = &st->chan_info[chan->scan_index];
	bool bipolar, differential;
	int ret;

	if (!st->sw_mode_en) {
		ci->range = 0;
		ci->scale_avail = ad7606_18bit_hw_scale_avail;
		ci->num_scales = ARRAY_SIZE(ad7606_18bit_hw_scale_avail);
		return 0;
	}

	ret = ad7606_get_chan_config(indio_dev, chan->scan_index, &bipolar,
				     &differential);
	if (ret)
		return ret;

	if (differential) {
		ci->scale_avail = ad7606c_18bit_differential_bipolar_scale_avail;
		ci->num_scales =
			ARRAY_SIZE(ad7606c_18bit_differential_bipolar_scale_avail);
		/* Bipolar differential ranges start at 8 (b1000) */
		ci->reg_offset = 8;
		ci->range = 1;
		chan->differential = 1;
		chan->channel2 = chan->channel;

		return 0;
	}

	chan->differential = 0;

	if (bipolar) {
		ci->scale_avail = ad7606c_18bit_single_ended_bipolar_scale_avail;
		ci->num_scales =
			ARRAY_SIZE(ad7606c_18bit_single_ended_bipolar_scale_avail);
		/* Bipolar single-ended ranges start at 0 (b0000) */
		ci->reg_offset = 0;
		ci->range = 3;
		chan->scan_type.sign = 's';

		return 0;
	}

	ci->scale_avail = ad7606c_18bit_single_ended_unipolar_scale_avail;
	ci->num_scales =
		ARRAY_SIZE(ad7606c_18bit_single_ended_unipolar_scale_avail);
	/* Unipolar single-ended ranges start at 5 (b0101) */
	ci->reg_offset = 5;
	ci->range = 1;
	chan->scan_type.sign = 'u';

	return 0;
}

static int ad7606c_16bit_chan_scale_setup(struct iio_dev *indio_dev,
					  struct iio_chan_spec *chan)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci = &st->chan_info[chan->scan_index];
	bool bipolar, differential;
	int ret;

	if (!st->sw_mode_en) {
		ci->range = 0;
		ci->scale_avail = ad7606_16bit_hw_scale_avail;
		ci->num_scales = ARRAY_SIZE(ad7606_16bit_hw_scale_avail);
		return 0;
	}

	ret = ad7606_get_chan_config(indio_dev, chan->scan_index, &bipolar,
				     &differential);
	if (ret)
		return ret;

	if (differential) {
		ci->scale_avail = ad7606c_16bit_differential_bipolar_scale_avail;
		ci->num_scales =
			ARRAY_SIZE(ad7606c_16bit_differential_bipolar_scale_avail);
		/* Bipolar differential ranges start at 8 (b1000) */
		ci->reg_offset = 8;
		ci->range = 1;
		chan->differential = 1;
		chan->channel2 = chan->channel;
		chan->scan_type.sign = 's';

		return 0;
	}

	chan->differential = 0;

	if (bipolar) {
		ci->scale_avail = ad7606c_16bit_single_ended_bipolar_scale_avail;
		ci->num_scales =
			ARRAY_SIZE(ad7606c_16bit_single_ended_bipolar_scale_avail);
		/* Bipolar single-ended ranges start at 0 (b0000) */
		ci->reg_offset = 0;
		ci->range = 3;
		chan->scan_type.sign = 's';

		return 0;
	}

	ci->scale_avail = ad7606c_16bit_single_ended_unipolar_scale_avail;
	ci->num_scales =
		ARRAY_SIZE(ad7606c_16bit_single_ended_unipolar_scale_avail);
	/* Unipolar single-ended ranges start at 5 (b0101) */
	ci->reg_offset = 5;
	ci->range = 1;
	chan->scan_type.sign = 'u';

	return 0;
}

static int ad7607_chan_scale_setup(struct iio_dev *indio_dev,
				   struct iio_chan_spec *chan)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci = &st->chan_info[chan->scan_index];

	ci->range = 0;
	ci->scale_avail = ad7607_hw_scale_avail;
	ci->num_scales = ARRAY_SIZE(ad7607_hw_scale_avail);
	return 0;
}

static int ad7608_chan_scale_setup(struct iio_dev *indio_dev,
				   struct iio_chan_spec *chan)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci = &st->chan_info[chan->scan_index];

	ci->range = 0;
	ci->scale_avail = ad7606_18bit_hw_scale_avail;
	ci->num_scales = ARRAY_SIZE(ad7606_18bit_hw_scale_avail);
	return 0;
}

static int ad7609_chan_scale_setup(struct iio_dev *indio_dev,
				   struct iio_chan_spec *chan)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci = &st->chan_info[chan->scan_index];

	ci->range = 0;
	ci->scale_avail = ad7609_hw_scale_avail;
	ci->num_scales = ARRAY_SIZE(ad7609_hw_scale_avail);
	return 0;
}

static int ad7606_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);

	if (readval) {
		ret = st->bops->reg_read(st, reg);
		if (ret < 0)
			return ret;
		*readval = ret;
		return 0;
	} else {
		return st->bops->reg_write(st, reg, writeval);
	}
}

static int ad7606_pwm_set_high(struct ad7606_state *st)
{
	struct pwm_state cnvst_pwm_state;
	int ret;

	pwm_get_state(st->cnvst_pwm, &cnvst_pwm_state);
	cnvst_pwm_state.enabled = true;
	cnvst_pwm_state.duty_cycle = cnvst_pwm_state.period;

	ret = pwm_apply_might_sleep(st->cnvst_pwm, &cnvst_pwm_state);

	return ret;
}

int ad7606_pwm_set_low(struct ad7606_state *st)
{
	struct pwm_state cnvst_pwm_state;
	int ret;

	pwm_get_state(st->cnvst_pwm, &cnvst_pwm_state);
	cnvst_pwm_state.enabled = true;
	cnvst_pwm_state.duty_cycle = 0;

	ret = pwm_apply_might_sleep(st->cnvst_pwm, &cnvst_pwm_state);

	return ret;
}
EXPORT_SYMBOL_NS_GPL(ad7606_pwm_set_low, "IIO_AD7606");

int ad7606_pwm_set_swing(struct ad7606_state *st)
{
	struct pwm_state cnvst_pwm_state;

	pwm_get_state(st->cnvst_pwm, &cnvst_pwm_state);
	cnvst_pwm_state.enabled = true;
	cnvst_pwm_state.duty_cycle = cnvst_pwm_state.period / 2;

	return pwm_apply_might_sleep(st->cnvst_pwm, &cnvst_pwm_state);
}
EXPORT_SYMBOL_NS_GPL(ad7606_pwm_set_swing, "IIO_AD7606");

static bool ad7606_pwm_is_swinging(struct ad7606_state *st)
{
	struct pwm_state cnvst_pwm_state;

	pwm_get_state(st->cnvst_pwm, &cnvst_pwm_state);

	return cnvst_pwm_state.duty_cycle != cnvst_pwm_state.period &&
	       cnvst_pwm_state.duty_cycle != 0;
}

static int ad7606_set_sampling_freq(struct ad7606_state *st, unsigned long freq)
{
	struct pwm_state cnvst_pwm_state;
	bool is_swinging = ad7606_pwm_is_swinging(st);
	bool is_high;

	if (freq == 0)
		return -EINVAL;

	/* Retrieve the previous state. */
	pwm_get_state(st->cnvst_pwm, &cnvst_pwm_state);
	is_high = cnvst_pwm_state.duty_cycle == cnvst_pwm_state.period;

	cnvst_pwm_state.period = DIV_ROUND_UP_ULL(NSEC_PER_SEC, freq);
	cnvst_pwm_state.polarity = PWM_POLARITY_NORMAL;
	if (is_high)
		cnvst_pwm_state.duty_cycle = cnvst_pwm_state.period;
	else if (is_swinging)
		cnvst_pwm_state.duty_cycle = cnvst_pwm_state.period / 2;
	else
		cnvst_pwm_state.duty_cycle = 0;

	return pwm_apply_might_sleep(st->cnvst_pwm, &cnvst_pwm_state);
}

static int ad7606_read_samples(struct ad7606_state *st)
{
	unsigned int num = st->chip_info->num_adc_channels;

	return st->bops->read_block(st->dev, num, &st->data);
}

static irqreturn_t ad7606_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);

	ret = ad7606_read_samples(st);
	if (ret)
		goto error_ret;

	iio_push_to_buffers_with_ts(indio_dev, &st->data, sizeof(st->data),
				    iio_get_time_ns(indio_dev));
error_ret:
	iio_trigger_notify_done(indio_dev->trig);
	/* The rising edge of the CONVST signal starts a new conversion. */
	gpiod_set_value(st->gpio_convst, 1);

	return IRQ_HANDLED;
}

static int ad7606_scan_direct(struct iio_dev *indio_dev, unsigned int ch,
			      int *val)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	const struct iio_chan_spec *chan;
	unsigned int realbits;
	int ret;

	if (st->gpio_convst) {
		gpiod_set_value(st->gpio_convst, 1);
	} else {
		ret = ad7606_pwm_set_high(st);
		if (ret < 0)
			return ret;
	}

	/*
	 * If no backend, wait for the interruption on busy pin, otherwise just add
	 * a delay to leave time for the data to be available. For now, the latter
	 * will not happen because IIO_CHAN_INFO_RAW is not supported for the backend.
	 * TODO: Add support for reading a single value when the backend is used.
	 */
	if (st->trig) {
		ret = wait_for_completion_timeout(&st->completion,
						  msecs_to_jiffies(1000));
		if (!ret) {
			ret = -ETIMEDOUT;
			goto error_ret;
		}
	} else {
		/*
		 * If the BUSY interrupt is not available, wait enough time for
		 * the longest possible conversion (max for the whole family is
		 * around 350us).
		 */
		fsleep(400);
	}

	ret = ad7606_read_samples(st);
	if (ret)
		goto error_ret;

	chan = &indio_dev->channels[ch];
	realbits = chan->scan_type.realbits;

	if (realbits > 16)
		*val = st->data.buf32[ch];
	else
		*val = st->data.buf16[ch];

	*val &= GENMASK(realbits - 1, 0);

	if (chan->scan_type.sign == 's')
		*val = sign_extend32(*val, realbits - 1);

error_ret:
	if (!st->gpio_convst) {
		ret = ad7606_pwm_set_low(st);
		if (ret < 0)
			return ret;
	}
	gpiod_set_value(st->gpio_convst, 0);

	return ret;
}

static int ad7606_get_calib_offset(struct ad7606_state *st, int ch, int *val)
{
	int ret;

	ret = st->bops->reg_read(st, AD7606_CALIB_OFFSET(ch));
	if (ret < 0)
		return ret;

	*val = st->chip_info->calib_offset_avail[0] +
	       ret * st->chip_info->calib_offset_avail[1];

	return 0;
}

static int ad7606_get_calib_phase(struct ad7606_state *st, int ch, int *val,
				  int *val2)
{
	int ret;

	ret = st->bops->reg_read(st, AD7606_CALIB_PHASE(ch));
	if (ret < 0)
		return ret;

	*val = 0;

	/*
	 * ad7606b: phase delay from 0 to 318.75 μs in steps of 1.25 μs.
	 * ad7606c-16/18: phase delay from 0 µs to 255 µs in steps of 1 µs.
	 */
	*val2 = ret * st->chip_info->calib_phase_avail[1][1];

	return 0;
}

static int ad7606_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret, ch = 0;
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci;
	struct pwm_state cnvst_pwm_state;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = ad7606_scan_direct(indio_dev, chan->scan_index, val);
		iio_device_release_direct(indio_dev);
		if (ret < 0)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (st->sw_mode_en)
			ch = chan->scan_index;
		ci = &st->chan_info[ch];
		*val = ci->scale_avail[ci->range][0];
		*val2 = ci->scale_avail[ci->range][1];
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->oversampling;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		pwm_get_state(st->cnvst_pwm, &cnvst_pwm_state);
		*val = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC, cnvst_pwm_state.period);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = ad7606_get_calib_offset(st, chan->scan_index, val);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CONVDELAY:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = ad7606_get_calib_phase(st, chan->scan_index, val, val2);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		return IIO_VAL_INT_PLUS_NANO;
	}
	return -EINVAL;
}

static ssize_t in_voltage_scale_available_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci = &st->chan_info[0];
	const unsigned int (*vals)[2] = ci->scale_avail;
	unsigned int i;
	size_t len = 0;

	for (i = 0; i < ci->num_scales; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u.%06u ",
				 vals[i][0], vals[i][1]);
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEVICE_ATTR_RO(in_voltage_scale_available, 0);

static int ad7606_write_scale_hw(struct iio_dev *indio_dev, int ch, int val)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	gpiod_set_value(st->gpio_range, val);

	return 0;
}

static int ad7606_write_os_hw(struct iio_dev *indio_dev, int val)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	DECLARE_BITMAP(values, 3);

	values[0] = val & GENMASK(2, 0);

	gpiod_multi_set_value_cansleep(st->gpio_os, values);

	/* AD7616 requires a reset to update value */
	if (st->chip_info->os_req_reset)
		ad7606_reset(st);

	return 0;
}

static int ad7606_set_calib_offset(struct ad7606_state *st, int ch, int val)
{
	int start_val, step_val, stop_val;
	int offset;

	start_val = st->chip_info->calib_offset_avail[0];
	step_val = st->chip_info->calib_offset_avail[1];
	stop_val = st->chip_info->calib_offset_avail[2];

	if (val < start_val || val > stop_val)
		return -EINVAL;

	offset = (val - start_val) / step_val;

	return st->bops->reg_write(st, AD7606_CALIB_OFFSET(ch), offset);
}

static int ad7606_set_calib_phase(struct ad7606_state *st, int ch, int val,
				  int val2)
{
	int wreg, start_ns, step_ns, stop_ns;

	if (val != 0)
		return -EINVAL;

	start_ns = st->chip_info->calib_phase_avail[0][1];
	step_ns = st->chip_info->calib_phase_avail[1][1];
	stop_ns = st->chip_info->calib_phase_avail[2][1];

	/*
	 * ad7606b: phase delay from 0 to 318.75 μs in steps of 1.25 μs.
	 * ad7606c-16/18: phase delay from 0 µs to 255 µs in steps of 1 µs.
	 */
	if (val2 < start_ns || val2 > stop_ns)
		return -EINVAL;

	wreg = val2 / step_ns;

	return st->bops->reg_write(st, AD7606_CALIB_PHASE(ch), wreg);
}

static int ad7606_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
	case IIO_CHAN_INFO_CALIBBIAS:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CONVDELAY:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return -EINVAL;
	}
}

static int ad7606_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	unsigned int scale_avail_uv[AD760X_MAX_SCALES];
	struct ad7606_chan_info *ci;
	int i, ret, ch = 0;

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (st->sw_mode_en)
			ch = chan->scan_index;
		ci = &st->chan_info[ch];
		for (i = 0; i < ci->num_scales; i++) {
			scale_avail_uv[i] = ci->scale_avail[i][0] * MICRO +
					    ci->scale_avail[i][1];
		}
		val = (val * MICRO) + val2;
		i = find_closest(val, scale_avail_uv, ci->num_scales);

		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = st->write_scale(indio_dev, ch, i + ci->reg_offset);
		iio_device_release_direct(indio_dev);
		if (ret < 0)
			return ret;
		ci->range = i;

		return 0;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		if (val2)
			return -EINVAL;
		i = find_closest(val, st->oversampling_avail,
				 st->num_os_ratios);

		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = st->write_os(indio_dev, i);
		iio_device_release_direct(indio_dev);
		if (ret < 0)
			return ret;
		st->oversampling = st->oversampling_avail[i];

		return 0;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val < 0 && val2 != 0)
			return -EINVAL;
		return ad7606_set_sampling_freq(st, val);
	case IIO_CHAN_INFO_CALIBBIAS:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = ad7606_set_calib_offset(st, chan->scan_index, val);
		iio_device_release_direct(indio_dev);
		return ret;
	case IIO_CHAN_INFO_CONVDELAY:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = ad7606_set_calib_phase(st, chan->scan_index, val, val2);
		iio_device_release_direct(indio_dev);
		return ret;
	default:
		return -EINVAL;
	}
}

static ssize_t ad7606_oversampling_ratio_avail(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7606_state *st = iio_priv(indio_dev);
	const unsigned int *vals = st->oversampling_avail;
	unsigned int i;
	size_t len = 0;

	for (i = 0; i < st->num_os_ratios; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u ", vals[i]);
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEVICE_ATTR(oversampling_ratio_available, 0444,
		       ad7606_oversampling_ratio_avail, NULL, 0);

static struct attribute *ad7606_attributes_os_and_range[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7606_attribute_group_os_and_range = {
	.attrs = ad7606_attributes_os_and_range,
};

static struct attribute *ad7606_attributes_os[] = {
	&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7606_attribute_group_os = {
	.attrs = ad7606_attributes_os,
};

static struct attribute *ad7606_attributes_range[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7606_attribute_group_range = {
	.attrs = ad7606_attributes_range,
};

static int ad7606_request_gpios(struct ad7606_state *st)
{
	struct device *dev = st->dev;

	st->gpio_convst = devm_gpiod_get_optional(dev, "adi,conversion-start",
						  GPIOD_OUT_LOW);

	if (IS_ERR(st->gpio_convst))
		return PTR_ERR(st->gpio_convst);

	st->gpio_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	st->gpio_range = devm_gpiod_get_optional(dev, "adi,range",
						 GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_range))
		return PTR_ERR(st->gpio_range);

	st->gpio_standby = devm_gpiod_get_optional(dev, "standby",
						   GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_standby))
		return PTR_ERR(st->gpio_standby);

	st->gpio_frstdata = devm_gpiod_get_optional(dev, "adi,first-data",
						    GPIOD_IN);
	if (IS_ERR(st->gpio_frstdata))
		return PTR_ERR(st->gpio_frstdata);

	if (!st->chip_info->oversampling_num)
		return 0;

	st->gpio_os = devm_gpiod_get_array_optional(dev,
						    "adi,oversampling-ratio",
						    GPIOD_OUT_LOW);
	return PTR_ERR_OR_ZERO(st->gpio_os);
}

/*
 * The BUSY signal indicates when conversions are in progress, so when a rising
 * edge of CONVST is applied, BUSY goes logic high and transitions low at the
 * end of the entire conversion process. The falling edge of the BUSY signal
 * triggers this interrupt.
 */
static irqreturn_t ad7606_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	if (iio_buffer_enabled(indio_dev)) {
		if (st->gpio_convst) {
			gpiod_set_value(st->gpio_convst, 0);
		} else {
			ret = ad7606_pwm_set_low(st);
			if (ret < 0) {
				dev_err(st->dev, "PWM set low failed");
				goto done;
			}
		}
		iio_trigger_poll_nested(st->trig);
	} else {
		complete(&st->completion);
	}

done:
	return IRQ_HANDLED;
};

static int ad7606_validate_trigger(struct iio_dev *indio_dev,
				   struct iio_trigger *trig)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static int ad7606_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	gpiod_set_value(st->gpio_convst, 1);

	return 0;
}

static int ad7606_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	gpiod_set_value(st->gpio_convst, 0);

	return 0;
}

static int ad7606_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct ad7606_chan_info *ci;
	unsigned int ch = 0;

	switch (info) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*vals = st->oversampling_avail;
		*length = st->num_os_ratios;
		*type = IIO_VAL_INT;

		return IIO_AVAIL_LIST;

	case IIO_CHAN_INFO_SCALE:
		if (st->sw_mode_en)
			ch = chan->scan_index;

		ci = &st->chan_info[ch];
		*vals = (int *)ci->scale_avail;
		*length = ci->num_scales * 2;
		*type = IIO_VAL_INT_PLUS_MICRO;

		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_CALIBBIAS:
		*vals = st->chip_info->calib_offset_avail;
		*type = IIO_VAL_INT;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_CONVDELAY:
		*vals = (const int *)st->chip_info->calib_phase_avail;
		*type = IIO_VAL_INT_PLUS_NANO;
		return IIO_AVAIL_RANGE;
	}
	return -EINVAL;
}

static int ad7606_backend_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	return ad7606_pwm_set_swing(st);
}

static int ad7606_backend_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	return ad7606_pwm_set_low(st);
}

static int ad7606_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	/*
	 * The update scan mode is only for iio backend compatible drivers.
	 * If the specific update_scan_mode is not defined in the bus ops,
	 * just do nothing and return 0.
	 */
	if (!st->bops->update_scan_mode)
		return 0;

	return st->bops->update_scan_mode(indio_dev, scan_mask);
}

static const struct iio_buffer_setup_ops ad7606_buffer_ops = {
	.postenable = &ad7606_buffer_postenable,
	.predisable = &ad7606_buffer_predisable,
};

static const struct iio_buffer_setup_ops ad7606_backend_buffer_ops = {
	.postenable = &ad7606_backend_buffer_postenable,
	.predisable = &ad7606_backend_buffer_predisable,
};

static const struct iio_info ad7606_info_no_os_or_range = {
	.read_raw = &ad7606_read_raw,
	.validate_trigger = &ad7606_validate_trigger,
	.update_scan_mode = &ad7606_update_scan_mode,
};

static const struct iio_info ad7606_info_os_and_range = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_os_and_range,
	.validate_trigger = &ad7606_validate_trigger,
	.update_scan_mode = &ad7606_update_scan_mode,
};

static const struct iio_info ad7606_info_sw_mode = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.read_avail = &ad7606_read_avail,
	.write_raw_get_fmt = ad7606_write_raw_get_fmt,
	.debugfs_reg_access = &ad7606_reg_access,
	.validate_trigger = &ad7606_validate_trigger,
	.update_scan_mode = &ad7606_update_scan_mode,
};

static const struct iio_info ad7606_info_os = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_os,
	.validate_trigger = &ad7606_validate_trigger,
	.update_scan_mode = &ad7606_update_scan_mode,
};

static const struct iio_info ad7606_info_range = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_range,
	.validate_trigger = &ad7606_validate_trigger,
	.update_scan_mode = &ad7606_update_scan_mode,
};

static const struct iio_trigger_ops ad7606_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static int ad7606_write_mask(struct ad7606_state *st, unsigned int addr,
			     unsigned long mask, unsigned int val)
{
	int readval;

	readval = st->bops->reg_read(st, addr);
	if (readval < 0)
		return readval;

	readval &= ~mask;
	readval |= val;

	return st->bops->reg_write(st, addr, readval);
}

static int ad7616_write_scale_sw(struct iio_dev *indio_dev, int ch, int val)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	unsigned int ch_addr, mode, ch_index;

	/*
	 * Ad7616 has 16 channels divided in group A and group B.
	 * The range of channels from A are stored in registers with address 4
	 * while channels from B are stored in register with address 6.
	 * The last bit from channels determines if it is from group A or B
	 * because the order of channels in iio is 0A, 0B, 1A, 1B...
	 */
	ch_index = ch >> 1;

	ch_addr = AD7616_RANGE_CH_ADDR(ch_index);

	if ((ch & 0x1) == 0) /* channel A */
		ch_addr += AD7616_RANGE_CH_A_ADDR_OFF;
	else	/* channel B */
		ch_addr += AD7616_RANGE_CH_B_ADDR_OFF;

	/* 0b01 for 2.5v, 0b10 for 5v and 0b11 for 10v */
	mode = AD7616_RANGE_CH_MODE(ch_index, ((val + 1) & 0b11));

	return ad7606_write_mask(st, ch_addr, AD7616_RANGE_CH_MSK(ch_index),
				 mode);
}

static int ad7616_write_os_sw(struct iio_dev *indio_dev, int val)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	return ad7606_write_mask(st, AD7616_CONFIGURATION_REGISTER,
				 AD7616_OS_MASK, val << 2);
}

static int ad7606_write_scale_sw(struct iio_dev *indio_dev, int ch, int val)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	return ad7606_write_mask(st, AD7606_RANGE_CH_ADDR(ch),
				 AD7606_RANGE_CH_MSK(ch),
				 AD7606_RANGE_CH_MODE(ch, val));
}

static int ad7606_write_os_sw(struct iio_dev *indio_dev, int val)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	return st->bops->reg_write(st, AD7606_OS_MODE, val);
}

static int ad7616_sw_mode_setup(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	/*
	 * Scale can be configured individually for each channel
	 * in software mode.
	 */

	st->write_scale = ad7616_write_scale_sw;
	st->write_os = &ad7616_write_os_sw;

	if (st->bops->sw_mode_config) {
		ret = st->bops->sw_mode_config(indio_dev);
		if (ret)
			return ret;
	}

	/* Activate Burst mode and SEQEN MODE */
	return ad7606_write_mask(st, AD7616_CONFIGURATION_REGISTER,
				 AD7616_BURST_MODE | AD7616_SEQEN_MODE,
				 AD7616_BURST_MODE | AD7616_SEQEN_MODE);
}

static int ad7606b_sw_mode_setup(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	DECLARE_BITMAP(os, 3);

	bitmap_fill(os, 3);
	/*
	 * Software mode is enabled when all three oversampling
	 * pins are set to high. If oversampling gpios are defined
	 * in the device tree, then they need to be set to high,
	 * otherwise, they must be hardwired to VDD
	 */
	if (st->gpio_os)
		gpiod_multi_set_value_cansleep(st->gpio_os, os);

	/* OS of 128 and 256 are available only in software mode */
	st->oversampling_avail = ad7606b_oversampling_avail;
	st->num_os_ratios = ARRAY_SIZE(ad7606b_oversampling_avail);

	st->write_scale = ad7606_write_scale_sw;
	st->write_os = &ad7606_write_os_sw;

	if (!st->bops->sw_mode_config)
		return 0;

	return st->bops->sw_mode_config(indio_dev);
}

static int ad7606_set_gain_calib(struct ad7606_state *st)
{
	struct ad7606_chan_info *ci;
	int i, ret;

	for (i = 0; i < st->chip_info->num_adc_channels; i++) {
		ci = &st->chan_info[i];
		ret = st->bops->reg_write(st, AD7606_CALIB_GAIN(i),
					  DIV_ROUND_CLOSEST(ci->r_gain,
						AD7606_CALIB_GAIN_STEP));
		if (ret)
			return ret;
	}

	return 0;
}

static int ad7606_probe_channels(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	struct iio_chan_spec *channels;
	bool slow_bus;
	int ret, i;

	slow_bus = !(st->bops->iio_backend_config || st->offload_en);
	indio_dev->num_channels = st->chip_info->num_adc_channels;

	/* Slow buses also get 1 more channel for soft timestamp */
	if (slow_bus)
		indio_dev->num_channels++;

	channels = devm_kcalloc(dev, indio_dev->num_channels, sizeof(*channels),
				GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	for (i = 0; i < st->chip_info->num_adc_channels; i++) {
		struct iio_chan_spec *chan = &channels[i];

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = i;
		chan->scan_index = i;
		chan->scan_type.sign = 's';
		chan->scan_type.realbits = st->chip_info->bits;
		/*
		 * If in SPI offload mode, storagebits are set based
		 * on the spi-engine hw implementation.
		 */
		chan->scan_type.storagebits = st->offload_en ?
			st->chip_info->offload_storagebits :
			(st->chip_info->bits > 16 ? 32 : 16);

		chan->scan_type.endianness = IIO_CPU;

		if (indio_dev->modes & INDIO_DIRECT_MODE)
			chan->info_mask_separate |= BIT(IIO_CHAN_INFO_RAW);

		if (st->sw_mode_en) {
			chan->info_mask_separate |= BIT(IIO_CHAN_INFO_SCALE);
			chan->info_mask_separate_available |=
				BIT(IIO_CHAN_INFO_SCALE);

			if (st->chip_info->calib_offset_avail) {
				chan->info_mask_separate |=
					BIT(IIO_CHAN_INFO_CALIBBIAS) |
					BIT(IIO_CHAN_INFO_CONVDELAY);
				chan->info_mask_separate_available |=
					BIT(IIO_CHAN_INFO_CALIBBIAS) |
					BIT(IIO_CHAN_INFO_CONVDELAY);
			}

			/*
			 * All chips with software mode support oversampling,
			 * so we skip the oversampling_available check. And the
			 * shared_by_type instead of shared_by_all on slow
			 * buses is for backward compatibility.
			 */
			if (slow_bus)
				chan->info_mask_shared_by_type |=
					BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO);
			else
				chan->info_mask_shared_by_all |=
					BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO);

			chan->info_mask_shared_by_all_available |=
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO);
		} else {
			chan->info_mask_shared_by_type |=
				BIT(IIO_CHAN_INFO_SCALE);

			if (st->chip_info->oversampling_avail)
				chan->info_mask_shared_by_all |=
					BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO);
		}

		if (!slow_bus)
			chan->info_mask_shared_by_all |=
				BIT(IIO_CHAN_INFO_SAMP_FREQ);

		ret = st->chip_info->scale_setup_cb(indio_dev, chan);
		if (ret)
			return ret;
	}

	if (slow_bus)
		channels[i] = (struct iio_chan_spec)IIO_CHAN_SOFT_TIMESTAMP(i);

	indio_dev->channels = channels;

	return 0;
}

static void ad7606_pwm_disable(void *data)
{
	pwm_disable(data);
}

int ad7606_probe(struct device *dev, int irq, void __iomem *base_address,
		 const struct ad7606_chip_info *chip_info,
		 const struct ad7606_bus_ops *bops)
{
	struct ad7606_state *st;
	int ret;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->dev = dev;
	st->bops = bops;
	st->base_address = base_address;
	st->oversampling = 1;
	st->sw_mode_en = device_property_read_bool(dev, "adi,sw-mode");

	if (st->sw_mode_en && !chip_info->sw_setup_cb)
		return dev_err_probe(dev, -EINVAL,
			"Software mode is not supported for this chip\n");

	ret = devm_regulator_get_enable(dev, "avcc");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable specified AVcc supply\n");

	ret = devm_regulator_get_enable(dev, "vdrive");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable Vdrive supply\n");

	ret = devm_regulator_get_enable_optional(dev, "refin");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret,
				     "Failed to enable REFIN supply\n");

	st->chip_info = chip_info;

	if (st->chip_info->oversampling_num) {
		st->oversampling_avail = st->chip_info->oversampling_avail;
		st->num_os_ratios = st->chip_info->oversampling_num;
	}

	ret = ad7606_request_gpios(st);
	if (ret)
		return ret;

	if (st->gpio_os) {
		if (st->gpio_range)
			indio_dev->info = &ad7606_info_os_and_range;
		else
			indio_dev->info = &ad7606_info_os;
	} else {
		if (st->gpio_range)
			indio_dev->info = &ad7606_info_range;
		else
			indio_dev->info = &ad7606_info_no_os_or_range;
	}

	/* AXI ADC backend doesn't support single read. */
	indio_dev->modes = st->bops->iio_backend_config ? 0 : INDIO_DIRECT_MODE;
	indio_dev->name = chip_info->name;

	/* Using spi-engine with offload support ? */
	if (st->bops->offload_config) {
		ret = st->bops->offload_config(dev, indio_dev);
		if (ret)
			return ret;
	}

	ret = ad7606_probe_channels(indio_dev);
	if (ret)
		return ret;

	ret = ad7606_reset(st);
	if (ret)
		dev_warn(st->dev, "failed to RESET: no RESET GPIO specified\n");

	/* AD7616 requires al least 15ms to reconfigure after a reset */
	if (st->chip_info->init_delay_ms) {
		if (msleep_interruptible(st->chip_info->init_delay_ms))
			return -ERESTARTSYS;
	}

	/* If convst pin is not defined, setup PWM. */
	if (!st->gpio_convst || st->offload_en) {
		st->cnvst_pwm = devm_pwm_get(dev, NULL);
		if (IS_ERR(st->cnvst_pwm))
			return PTR_ERR(st->cnvst_pwm);

		/* The PWM is initialized at 1MHz to have a fast enough GPIO emulation. */
		ret = ad7606_set_sampling_freq(st, 1 * MEGA);
		if (ret)
			return ret;

		ret = ad7606_pwm_set_low(st);
		if (ret)
			return ret;

		/*
		 * PWM is not disabled when sampling stops, but instead its duty cycle is set
		 * to 0% to be sure we have a "low" state. After we unload the driver, let's
		 * disable the PWM.
		 */
		ret = devm_add_action_or_reset(dev, ad7606_pwm_disable,
					       st->cnvst_pwm);
		if (ret)
			return ret;
	}

	if (st->bops->iio_backend_config) {
		/*
		 * If there is a backend, the PWM should not overpass the maximum sampling
		 * frequency the chip supports.
		 */
		ret = ad7606_set_sampling_freq(st, chip_info->max_samplerate);
		if (ret)
			return ret;

		ret = st->bops->iio_backend_config(dev, indio_dev);
		if (ret)
			return ret;

		indio_dev->setup_ops = &ad7606_backend_buffer_ops;
	} else if (!st->offload_en) {
		/* Reserve the PWM use only for backend (force gpio_convst definition) */
		if (!st->gpio_convst)
			return dev_err_probe(dev, -EINVAL,
					     "No backend, connect convst to a GPIO");

		init_completion(&st->completion);
		st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
						  indio_dev->name,
						  iio_device_id(indio_dev));
		if (!st->trig)
			return -ENOMEM;

		st->trig->ops = &ad7606_trigger_ops;
		iio_trigger_set_drvdata(st->trig, indio_dev);
		ret = devm_iio_trigger_register(dev, st->trig);
		if (ret)
			return ret;

		indio_dev->trig = iio_trigger_get(st->trig);

		ret = devm_request_threaded_irq(dev, irq, NULL, &ad7606_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						chip_info->name, indio_dev);
		if (ret)
			return ret;

		ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
						      &iio_pollfunc_store_time,
						      &ad7606_trigger_handler,
						      &ad7606_buffer_ops);
		if (ret)
			return ret;
	}

	st->write_scale = ad7606_write_scale_hw;
	st->write_os = ad7606_write_os_hw;

	/* Offload needs 1 DOUT line, applying this setting in sw_setup_cb. */
	if (st->sw_mode_en || st->offload_en) {
		indio_dev->info = &ad7606_info_sw_mode;
		st->chip_info->sw_setup_cb(indio_dev);
	}

	if (st->sw_mode_en && st->chip_info->calib_gain_avail) {
		ret = ad7606_set_gain_calib(st);
		if (ret)
			return ret;
	}

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_NS_GPL(ad7606_probe, "IIO_AD7606");

#ifdef CONFIG_PM_SLEEP

static int ad7606_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	if (st->gpio_standby) {
		gpiod_set_value(st->gpio_range, 1);
		gpiod_set_value(st->gpio_standby, 1);
	}

	return 0;
}

static int ad7606_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	if (st->gpio_standby) {
		gpiod_set_value(st->gpio_range, st->chan_info[0].range);
		gpiod_set_value(st->gpio_standby, 1);
		ad7606_reset(st);
	}

	return 0;
}

SIMPLE_DEV_PM_OPS(ad7606_pm_ops, ad7606_suspend, ad7606_resume);
EXPORT_SYMBOL_NS_GPL(ad7606_pm_ops, "IIO_AD7606");

#endif

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
