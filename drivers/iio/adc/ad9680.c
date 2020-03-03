/*
 * Driver for AD9680 and similar high-speed Analog-to-Digital converters
 *
 * Copyright 2012-2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#define AD9680_REG_CHIP_ID_LOW		0x004
#define AD9680_REG_CHIP_ID_HIGH		0x005
#define AD9680_REG_DEVICE_INDEX		0x008
#define AD9680_REG_PAIR_INDEX		0x009
#define AD9680_REG_INPUT_FS_RANGE	0x025
#define AD9680_REG_CHIP_PIN_CTRL	0x040

#define AD9680_REG_OUTPUT_MODE		0x561
#define AD9680_REG_TEST_MODE		0x550

#define AD9680_REG_THRESH_CTRL		0x245
#define AD9680_REG_THRESH_HI_LSB	0x247
#define AD9680_REG_THRESH_HI_MSB	0x248
#define AD9680_REG_THRESH_LOW_LSB	0x249
#define AD9680_REG_THRESH_LOW_MSB	0x24A

#define AD9680_REG_CHIP_PIN_CTRL_MASK(chn)	(0x07 << (3 * (chn)))

#define AD9680_TESTMODE_OFF			0x0
#define AD9680_TESTMODE_MIDSCALE_SHORT		0x1
#define AD9680_TESTMODE_POS_FULLSCALE		0x2
#define AD9680_TESTMODE_NEG_FULLSCALE		0x3
#define AD9680_TESTMODE_ALT_CHECKERBOARD	0x4
#define AD9680_TESTMODE_PN23_SEQ		0x5
#define AD9680_TESTMODE_PN9_SEQ			0x6
#define AD9680_TESTMODE_ONE_ZERO_TOGGLE		0x7
#define AD9680_TESTMODE_USER			0x8
#define AD9680_TESTMODE_RAMP			0xF

#define AD9680_OUTPUT_MODE_OFFSET_BINARY	0x0
#define AD9680_OUTPUT_MODE_TWOS_COMPLEMENT	0x1

#define CHIPID_AD9680			0xC5
#define CHIPID_AD9684			0xD2
#define CHIPID_AD9234			0xCE
#define CHIPID_AD9694			0xDB
#define CHIPID_AD9094			0xE8

enum {
	ID_AD9234,
	ID_AD9680,
	ID_AD9680_x2,
	ID_AD9684,
	ID_AD9694,
	ID_AD9094,
};

enum ad9680_sysref_mode {
	AD9680_SYSREF_DISABLED,
	AD9680_SYSREF_CONTINUOUS,
	AD9680_SYSREF_ONESHOT
};

struct ad9680_sysref_config {
	enum ad9680_sysref_mode mode;
	bool capture_falling_edge;
	bool valid_falling_edge;
};

struct ad9680_jesd204_link_config {
	uint8_t did;
	uint8_t bid;

	uint8_t num_lanes;
	uint8_t num_converters;
	uint8_t octets_per_frame;
	uint8_t frames_per_multiframe;

	uint8_t bits_per_sample;
	uint8_t converter_resolution;

	uint8_t lid[4];
	uint8_t lane_mux[4];

	bool scrambling;
	uint8_t subclass;

	struct ad9680_sysref_config sysref;
};

static int ad9680_spi_read(struct spi_device *spi, unsigned int reg)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = 0x80 | (reg >> 8);
		buf[1] = reg & 0xFF;

		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);

		dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
			__func__, reg, buf[2], ret);

		if (ret < 0)
			return ret;

		return buf[2];
	}
	return -ENODEV;
}

static int ad9680_spi_write(struct spi_device *spi, unsigned int reg,
	unsigned int val)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = reg >> 8;
		buf[1] = reg & 0xFF;
		buf[2] = val;
		ret = spi_write_then_read(spi, buf, 3, NULL, 0);
		if (ret < 0)
			return ret;

		dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
			__func__, reg, val, ret);

		return 0;
	}

	return -ENODEV;
}

static int ad9680_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	if (readval == NULL)
		return ad9680_spi_write(spi, reg, writeval);

	ret = ad9680_spi_read(spi, reg);
	if (ret < 0)
		return ret;
	*readval = ret;

	return 0;
}

static int ad9680_select_channel(struct axiadc_converter *conv,
	int chan)
{
	unsigned int device, pair;
	int ret;

	if (chan >= 0) {
		device = BIT(chan & 0x1);
		pair = BIT((chan >> 1) & 1);
	} else {
		device = 0x3;
		pair = 0x3;
	}

	ret = ad9680_spi_write(conv->spi, AD9680_REG_DEVICE_INDEX, device);
	if (ret < 0)
		return ret;
	return ad9680_spi_write(conv->spi, AD9680_REG_PAIR_INDEX, pair);
}

static int ad9680_channel_write(struct axiadc_converter *conv,
	unsigned int chan, unsigned int reg, unsigned int val)
{
	int ret;

	ret = ad9680_select_channel(conv, chan);
	ret |= ad9680_spi_write(conv->spi, reg, val);
	ret |= ad9680_select_channel(conv, -1);

	return ret;
}

static int ad9680_channel_read(struct axiadc_converter *conv,
	unsigned int chan, unsigned int reg)
{
	int ret;

	ad9680_select_channel(conv, chan);
	ret = ad9680_spi_read(conv->spi, reg);
	ad9680_select_channel(conv, -1);

	return ret;
}

static unsigned int ad9680_pnsel_to_testmode(enum adc_pn_sel sel)
{
	switch (sel) {
	case ADC_PN9:
		return AD9680_TESTMODE_PN9_SEQ;
	case ADC_PN23A:
		return AD9680_TESTMODE_PN23_SEQ;
	default:
		return AD9680_TESTMODE_OFF;
	}
}

static int ad9680_outputmode_set(struct spi_device *spi, unsigned int mode)
{
	int ret;

	ret = ad9680_spi_write(spi, AD9680_REG_OUTPUT_MODE, mode);
	if (ret < 0)
		return ret;

	return ad9680_spi_write(spi, AD9680_REG_TEST_MODE,
				AD9680_TESTMODE_OFF);
}

static int ad9680_testmode_set(struct iio_dev *indio_dev, unsigned int chan,
	unsigned int mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	ad9680_channel_write(conv, chan, AD9680_REG_TEST_MODE, mode);
	conv->testmode[chan] = mode;

	return 0;
}

static int ad9680_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ad9680_pnsel_to_testmode(sel);
	unsigned int output_mode;
	int ret;

	output_mode = conv->adc_output_mode;
	if (mode != AD9680_TESTMODE_OFF)
		output_mode &= ~AD9680_OUTPUT_MODE_TWOS_COMPLEMENT;

	ret = ad9680_spi_write(conv->spi, AD9680_REG_OUTPUT_MODE, output_mode);
	if (ret < 0)
		return ret;

	return ad9680_testmode_set(indio_dev, chan, mode);
}

static irqreturn_t ad9680_event_handler(struct axiadc_converter *conv,
	unsigned int chn)
{
	u64 event = IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, chn,
			IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING);
	s64 timestamp = iio_get_time_ns(conv->indio_dev);

	if (conv->indio_dev)
		iio_push_event(conv->indio_dev, event, timestamp);

	return IRQ_HANDLED;
}

static irqreturn_t ad9680_fdA_handler(int irq, void *private)
{
	return ad9680_event_handler(private, 0);
}

static irqreturn_t ad9680_fdB_handler(int irq, void *private)
{
	return ad9680_event_handler(private, 1);
}

static int ad9680_read_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	u16 low, high;

	mutex_lock(&indio_dev->mlock);
	low = (ad9680_spi_read(spi, AD9680_REG_THRESH_LOW_MSB) << 8) |
		ad9680_spi_read(spi, AD9680_REG_THRESH_LOW_LSB);
	high = (ad9680_spi_read(spi, AD9680_REG_THRESH_HI_MSB) << 8) |
		ad9680_spi_read(spi, AD9680_REG_THRESH_HI_LSB);
	mutex_unlock(&indio_dev->mlock);

	switch (info) {
	case IIO_EV_INFO_HYSTERESIS:
		*val = high - low;
		break;
	case IIO_EV_INFO_VALUE:
		*val = high;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int ad9680_read_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	ret = ad9680_spi_read(spi, AD9680_REG_CHIP_PIN_CTRL);
	if (ret < 0)
		return ret;
	else
		return !(ret & AD9680_REG_CHIP_PIN_CTRL_MASK(chan->channel));
}

static int ad9680_write_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret = 0;
	int low, high;

	mutex_lock(&indio_dev->mlock);
	high = (ad9680_spi_read(spi, AD9680_REG_THRESH_HI_MSB) << 8) |
		ad9680_spi_read(spi, AD9680_REG_THRESH_HI_LSB);

	switch (info) {
	case IIO_EV_INFO_HYSTERESIS:
		if (val < 0) {
			ret = -EINVAL;
			goto unlock;
		}

		low = high - val;
		break;

	case IIO_EV_INFO_VALUE:
		if (val > 0x7FF) {
			ret = -EINVAL;
			goto unlock;
		}

		ad9680_spi_write(spi, AD9680_REG_THRESH_HI_MSB, val >> 8);
		ad9680_spi_write(spi, AD9680_REG_THRESH_HI_LSB, val & 0xFF);

		/* Calculate the new lower threshold limit */
		low = (ad9680_spi_read(spi, AD9680_REG_THRESH_LOW_MSB) << 8) |
			ad9680_spi_read(spi, AD9680_REG_THRESH_LOW_LSB);
		low = val - high + low;
		break;

	default:
		ret = -EINVAL;
		goto unlock;
	}

	if (low < 0)
		low = 0;
	ad9680_spi_write(spi, AD9680_REG_THRESH_LOW_MSB, low >> 8);
	ad9680_spi_write(spi, AD9680_REG_THRESH_LOW_LSB, low & 0xFF);

unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ad9680_write_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	mutex_lock(&indio_dev->mlock);

	ret = ad9680_spi_read(spi, AD9680_REG_CHIP_PIN_CTRL);
	if (ret < 0)
		goto err_unlock;

	if (state)
		ret &= ~AD9680_REG_CHIP_PIN_CTRL_MASK(chan->channel);
	else
		ret |= AD9680_REG_CHIP_PIN_CTRL_MASK(chan->channel);

	ret = ad9680_spi_write(spi, AD9680_REG_CHIP_PIN_CTRL, ret);
err_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static const int ad9680_scale_table[][2] = {
	{1460, 0x08}, {1580, 0x09}, {1700, 0x0A}, {1820, 0x0B},
	{1940, 0x00}, {2060, 0x0C},
};

static const int ad9694_scale_table[][2] = {
	{1440, 0xa}, {1560, 0xb}, {1680, 0xc}, {1800, 0xd},
	{1920, 0xe}, {2040, 0xf}, {2160, 0x0},
};

static void ad9680_scale(struct axiadc_converter *conv, int index,
	unsigned int *val, unsigned int *val2)
{
	unsigned int tmp;

	if (index > conv->chip_info->num_scales) {
		*val = 0;
		*val2 = 0;
		return;
	}

	tmp = (conv->chip_info->scale_table[index][0] * 1000000ULL) >>
		    conv->chip_info->channel[0].scan_type.realbits;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;
}

static ssize_t ad9680_show_scale_available(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int scale[2];
	int i, len = 0;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ad9680_scale(conv, i, &scale[0], &scale[1]);
		len += sprintf(buf + len, "%u.%06u ", scale[0], scale[1]);
	}

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}

static int ad9680_get_scale(struct axiadc_converter *conv,
	const struct iio_chan_spec *chan, int *val, int *val2)
{
	unsigned int vref_val;
	unsigned int i;

	switch (conv->id) {
	case CHIPID_AD9694:
	case CHIPID_AD9094:
		vref_val = ad9680_channel_read(conv, chan->channel, 0x1910);
		break;
	default:
		vref_val = ad9680_spi_read(conv->spi, AD9680_REG_INPUT_FS_RANGE);
		break;
	}
	vref_val &= 0xf;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		if (vref_val == conv->chip_info->scale_table[i][1])
			break;
	}

	ad9680_scale(conv, i, val, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

static int ad9680_set_scale(struct axiadc_converter *conv,
	const struct iio_chan_spec *chan, int val, int val2)
{
	unsigned int scale_val[2];
	unsigned int scale_raw;
	unsigned int i;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ad9680_scale(conv, i, &scale_val[0], &scale_val[1]);
		if (scale_val[0] != val || scale_val[1] != val2)
			continue;

		scale_raw = conv->chip_info->scale_table[i][1];

		switch (conv->id) {
		case CHIPID_AD9694:
		case CHIPID_AD9094:
			ad9680_channel_write(conv, chan->channel, 0x1910,
					     scale_raw);
			break;
		default:
			ad9680_spi_write(conv->spi, AD9680_REG_INPUT_FS_RANGE,
					 scale_raw);
			break;
		}
		return 0;
	}

	return -EINVAL;
}

static int ad9680_testmode_read(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return conv->testmode[chan->channel];
}

static int ad9680_testmode_write(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	ret = ad9680_testmode_set(indio_dev, chan->channel, item);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const char * const ad9680_testmodes[] = {
	[AD9680_TESTMODE_OFF] = "off",
	[AD9680_TESTMODE_MIDSCALE_SHORT] = "midscale_short",
	[AD9680_TESTMODE_POS_FULLSCALE] = "pos_fullscale",
	[AD9680_TESTMODE_NEG_FULLSCALE] = "neg_fullscale",
	[AD9680_TESTMODE_ALT_CHECKERBOARD] = "checkerboard",
	[AD9680_TESTMODE_PN23_SEQ] = "pn_long",
	[AD9680_TESTMODE_PN9_SEQ] = "pn_short",
	[AD9680_TESTMODE_ONE_ZERO_TOGGLE] = "one_zero_toggle",
	[AD9680_TESTMODE_USER] = "user",
	[AD9680_TESTMODE_RAMP] = "ramp",
};

static const struct iio_enum ad9680_testmode_enum = {
	.items = ad9680_testmodes,
	.num_items = ARRAY_SIZE(ad9680_testmodes),
	.set = ad9680_testmode_write,
	.get = ad9680_testmode_read,
};

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	IIO_ENUM("test_mode", IIO_SEPARATE, &ad9680_testmode_enum),
	IIO_ENUM_AVAILABLE("test_mode", &ad9680_testmode_enum),
	{
		.name = "scale_available",
		.read = ad9680_show_scale_available,
		.shared = true,
	},
	{},
};

static const struct iio_event_spec ad9680_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
				       BIT(IIO_EV_INFO_HYSTERESIS),
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

#define AD9680_CHAN(_chan, _si, _bits, _sign, _shift, _ev, _nb_ev)	\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = _shift,				\
	  },								\
	  .event_spec = _ev,						\
	  .num_event_specs = _nb_ev,					\
	}

#define AD9694_CHAN(_chan) {						\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _chan,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.ext_info = axiadc_ext_info,					\
	.scan_index = _chan,						\
	.scan_type = {							\
		.sign = 'S',						\
		.realbits = 8,						\
		.storagebits = 8,					\
		.shift = 0,						\
	},								\
	.event_spec = ad9680_events,					\
	.num_event_specs = ARRAY_SIZE(ad9680_events),			\
}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9234] = {
		.name = "AD9234",
		.max_rate = 1000000000UL,
		.scale_table = ad9680_scale_table,
		.num_scales = ARRAY_SIZE(ad9680_scale_table),
		.num_channels = 2,
		.channel[0] = AD9680_CHAN(0, 0, 12, 'S', 0, NULL, 0),
		.channel[1] = AD9680_CHAN(1, 1, 12, 'S', 0, NULL, 0),
	},
	[ID_AD9680] = {
		.name = "AD9680",
		.max_rate = 1250000000UL,
		.scale_table = ad9680_scale_table,
		.num_scales = ARRAY_SIZE(ad9680_scale_table),
		.num_channels = 2,
		.channel[0] = AD9680_CHAN(0, 0, 14, 'S', 0,
			ad9680_events, ARRAY_SIZE(ad9680_events)),
		.channel[1] = AD9680_CHAN(1, 1, 14, 'S', 0,
			ad9680_events, ARRAY_SIZE(ad9680_events)),
	},
	[ID_AD9680_x2] = {
		.name = "AD9680",
		.max_rate = 1250000000UL,
		.scale_table = ad9680_scale_table,
		.num_scales = ARRAY_SIZE(ad9680_scale_table),
		.num_channels = 4,
		.num_shadow_slave_channels = 2,
		.channel[0] = AD9680_CHAN(0, 0, 14, 'S', 0,
			ad9680_events, ARRAY_SIZE(ad9680_events)),
		.channel[1] = AD9680_CHAN(1, 1, 14, 'S', 0,
			ad9680_events, ARRAY_SIZE(ad9680_events)),
		.channel[2] = AD9680_CHAN(2, 2, 14, 'S', 0,
			ad9680_events, ARRAY_SIZE(ad9680_events)),
		.channel[3] = AD9680_CHAN(3, 3, 14, 'S', 0,
			ad9680_events, ARRAY_SIZE(ad9680_events)),
	},
	[ID_AD9684] = {
		.name = "AD9684",
		.max_rate = 1250000000UL,
		.scale_table = ad9680_scale_table,
		.num_scales = ARRAY_SIZE(ad9680_scale_table),
		.num_channels = 2,
		.channel[0] = AD9680_CHAN(0, 0, 14, 'S', 0, NULL, 0),
		.channel[1] = AD9680_CHAN(1, 1, 14, 'S', 0, NULL, 0),
	},
	[ID_AD9694] = {
		.name = "AD9694",
		.max_rate = 1000000000UL,
		.scale_table = ad9694_scale_table,
		.num_scales = ARRAY_SIZE(ad9694_scale_table),
		.num_channels = 4,
		.channel[0] = AD9694_CHAN(0),
		.channel[1] = AD9694_CHAN(1),
		.channel[2] = AD9694_CHAN(2),
		.channel[3] = AD9694_CHAN(3),
	},
	[ID_AD9094] = {
		.name = "AD9094",
		.max_rate = 1000000000UL,
		.scale_table = ad9694_scale_table,
		.num_scales = ARRAY_SIZE(ad9694_scale_table),
		.num_channels = 4,
		.channel[0] = AD9694_CHAN(0),
		.channel[1] = AD9694_CHAN(1),
		.channel[2] = AD9694_CHAN(2),
		.channel[3] = AD9694_CHAN(3),
	},
};

static bool ad9680_check_sysref_rate(unsigned int lmfc, unsigned int sysref)
{
	unsigned int div, mod;

	div = lmfc / sysref;
	mod = lmfc % sysref;

	/* Ignore minor deviations that can be introduced by rounding. */
	return mod <= div || mod >= sysref - div;
}

static int ad9680_update_sysref(struct axiadc_converter *conv,
				unsigned int lmfc)
{
	unsigned int n;
	int rate;

	/* No clock, no problem */
	if (!conv->sysref_clk)
		return 0;

	rate = clk_get_rate(conv->sysref_clk);
	if (rate < 0)
		return rate;

	/* If the current rate is OK, keep it */
	if (ad9680_check_sysref_rate(lmfc, rate))
		return 0;

	/*
	 * Try to find a rate that integer divides the LMFC. Starting with a low
	 * rate is a good idea and then slowly go up in case the clock generator
	 * can't generate such slow rates.
	 */
	for (n = 64; n > 0; n--) {
		rate = clk_round_rate(conv->sysref_clk, lmfc / n);
		if (ad9680_check_sysref_rate(lmfc, rate))
			break;
	}

	if (n == 0) {
		dev_err(&conv->spi->dev,
			"Could not find suitable SYSREF rate for LMFC of %u\n",
			lmfc);
		return -EINVAL;
	}

	return clk_set_rate(conv->sysref_clk, rate);
}

static ssize_t ad9680_status_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct axiadc_converter *conv = dev_get_drvdata(dev);
	const char *hold_setup_desc;
	unsigned int hold, setup;
	int val;
	int ret;

	switch (conv->id) {
	case CHIPID_AD9694:
	case CHIPID_AD9094:
		val = ad9680_spi_read(conv->spi, 0x11b);
		break;
	default:
		val = ad9680_spi_read(conv->spi, 0x11c);
		break;
	}

	ret = scnprintf(buf, PAGE_SIZE, "Input clock %sdetected\n",
		(val & 0x01) ? "" : "not ");

	if (conv->id == CHIPID_AD9684)
		return ret;

	val = ad9680_spi_read(conv->spi, 0x56f);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"JESD204 PLL is %slocked\n",
		(val & 0x80) ? "" : "not ");

	val = ad9680_spi_read(conv->spi, 0x12a);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"SYSREF counter: %d\n", val);

	val = ad9680_spi_read(conv->spi, 0x128);
	hold = (val >> 4) & 0xf;
	setup = val & 0xf;

	if (hold == 0x0 && setup <= 0x7)
		hold_setup_desc = "Possible setup error";
	else if (hold <= 0x8 && setup == 0x8)
		hold_setup_desc = "No setup or hold error (best hold margin)";
	else if (hold == 0x8 && setup >= 0x9)
		hold_setup_desc = "No setup or hold error (best setup and hold margin)";
	else if (hold == 0x8 && setup == 0x0)
		hold_setup_desc = "No setup or hold error (best setup margin)";
	else if (hold >= 0x9 && setup == 0x0)
		hold_setup_desc = "Possible hold error";
	else
		hold_setup_desc = "Possible setup or hold error";

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"SYSREF hold/setup status: %s (%x/%x)\n",
		hold_setup_desc, hold, setup);

	return ret;
}

static DEVICE_ATTR(status, 0444, ad9680_status_read, NULL);

static int ad9680_setup_jesd204_link(struct axiadc_converter *conv,
	unsigned int sample_rate)
{
	unsigned long lane_rate_kHz;
	unsigned long sysref_rate;
	int ret;

	sysref_rate = DIV_ROUND_CLOSEST(sample_rate, 32);
	lane_rate_kHz = DIV_ROUND_CLOSEST(sample_rate, 100);

	if (lane_rate_kHz < 3125000 || lane_rate_kHz > 12500000) {
		dev_err(&conv->spi->dev, "Lane rate %lu Mbps out of bounds. Must be between 3125 and 12500 Mbps",
			lane_rate_kHz / 1000);
		return -EINVAL;
	}

	if (lane_rate_kHz < 6250000)
		ad9680_spi_write(conv->spi, 0x56e, 0x10);	// low line rate mode must be enabled
	else
		ad9680_spi_write(conv->spi, 0x56e, 0x00);	// low line rate mode must be disabled

	ret = ad9680_update_sysref(conv, sysref_rate);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set SYSREF clock to %lu kHz: %d\n",
			sysref_rate / 1000, ret);
		return ret;
	}

	ret = clk_set_rate(conv->lane_clk, lane_rate_kHz);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set lane rate to %lu kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	return 0;
}

static int ad9680_set_sample_rate(struct axiadc_converter *conv,
	unsigned int sample_rate)
{
	unsigned int pll_stat;
	int ret = 0;

	/*
	 * Minimum ADC samplerate is 300 MSPS. But the minimum lane rate is
	 * 3.125 Gbps, which results in a minumum ADC samplerate of 312.5 Msps when
	 * using 4 lanes. Lower the minimum here once support for dynamic lane
	 * enable/disable has been implemented.
	 */
	sample_rate = clamp(sample_rate, 312500000U, 1000000000U);
	sample_rate = clk_round_rate(conv->clk, sample_rate);

	/* Disable link */
	ad9680_spi_write(conv->spi, 0x571, 0x15);

	if (conv->running) {
		clk_disable_unprepare(conv->lane_clk);
		clk_disable_unprepare(conv->sysref_clk);
		clk_disable_unprepare(conv->clk);
		conv->running = false;
	}

	ret = clk_set_rate(conv->clk, sample_rate);
	if (ret) {
		dev_err(&conv->spi->dev, "Failed to set converter clock rate to %u kHz: %d\n",
			sample_rate / 1000, ret);
		return ret;
	}

	ret = ad9680_setup_jesd204_link(conv, sample_rate);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(conv->clk);
	if (ret) {
		dev_err(&conv->spi->dev, "Failed to enable converter clock: %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(conv->sysref_clk);
	if (ret) {
		clk_disable_unprepare(conv->clk);
		dev_err(&conv->spi->dev, "Failed to enable SYSREF clock: %d\n", ret);
		return ret;
	}

	// Enable link
	ad9680_spi_write(conv->spi, 0x571, 0x14);

	mdelay(20);
	pll_stat = ad9680_spi_read(conv->spi, 0x56f);

	dev_info(&conv->spi->dev, "PLL %s\n",
		 (pll_stat & 0x80) ? "LOCKED" : "UNLOCKED");

	ret = clk_prepare_enable(conv->lane_clk);
	if (ret < 0) {
		clk_disable_unprepare(conv->clk);
		clk_disable_unprepare(conv->sysref_clk);
		dev_err(&conv->spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	conv->adc_clk = sample_rate;
	conv->running = true;

	return 0;
}

static int ad9680_request_clks(struct axiadc_converter *conv)
{
	int ret;

	conv->sysref_clk = devm_clk_get(&conv->spi->dev, "adc_sysref");
	if (IS_ERR(conv->sysref_clk)) {
		if (PTR_ERR(conv->sysref_clk) != -ENOENT)
			return PTR_ERR(conv->sysref_clk);
		conv->sysref_clk = NULL;
	} else {
		ret = clk_prepare_enable(conv->sysref_clk);
		if (ret < 0)
			return ret;
	}

	conv->clk = devm_clk_get(&conv->spi->dev, "adc_clk");
	if (IS_ERR(conv->clk)) {
		if (PTR_ERR(conv->clk) != -ENOENT) {
			clk_disable_unprepare(conv->sysref_clk);
			return PTR_ERR(conv->clk);
		}
		conv->clk = NULL;
	} else {
		ret = clk_prepare_enable(conv->clk);
		if (ret < 0) {
			clk_disable_unprepare(conv->sysref_clk);
			return ret;
		}

		conv->adc_clk = clk_get_rate(conv->clk);
	}

	conv->lane_clk = devm_clk_get(&conv->spi->dev, "jesd_adc_clk");
	if (IS_ERR(conv->lane_clk)) {
		if (PTR_ERR(conv->lane_clk) != -ENOENT) {
			clk_disable_unprepare(conv->clk);
			clk_disable_unprepare(conv->sysref_clk);
			return PTR_ERR(conv->lane_clk);
		}
		conv->lane_clk = NULL;
	}

	return 0;
}

static int ad9680_setup_link(struct spi_device *spi,
	const struct ad9680_jesd204_link_config *config)
{
	unsigned int val;
	unsigned int i;
	int ret = 0;

	val = ilog2(config->octets_per_frame);
	val |= ilog2(config->num_converters) << 3;
	val |= ilog2(config->num_lanes) << 6;

	ret |= ad9680_spi_write(spi, 0x580, config->did);
	ret |= ad9680_spi_write(spi, 0x581, config->bid);

	ret = ad9680_spi_write(spi, 0x570, val); // Quick config

	for (i = 0; i < config->num_lanes; i++) {
		ret |= ad9680_spi_write(spi, 0x583 + i, config->lid[i]);

		val = config->lane_mux[i];
		val |= val << 4;
		ret |= ad9680_spi_write(spi, 0x5b2 + i + (i / 2), val);
	}

	val = config->num_lanes - 1;
	val |= config->scrambling ? 0x80 : 0x00;
	ret |= ad9680_spi_write(spi, 0x58b, val);

	ret |= ad9680_spi_write(spi, 0x58d, config->frames_per_multiframe - 1);
	ret |= ad9680_spi_write(spi, 0x58f, config->converter_resolution - 1);

	val = config->bits_per_sample - 1;
	val |= config->subclass ? 0x20 : 0x00;
	ret |= ad9680_spi_write(spi, 0x590, val);

	/* Disable SYSREF */
	ret |= ad9680_spi_write(spi, 0x120, 0x00);

	ret |= ad9680_spi_write(spi, 0x121, 0x0f);

	switch (config->sysref.mode) {
	case AD9680_SYSREF_CONTINUOUS:
		val = 0x02;
		break;
	case AD9680_SYSREF_ONESHOT:
		val = 0x04;
		break;
	default:
		val = 0x00;
		break;
	}

	if (config->sysref.capture_falling_edge)
		val |= 0x08;

	if (config->sysref.valid_falling_edge)
		val |= 0x10;
	ret |= ad9680_spi_write(spi, 0x120, val);

	return ret;
}

static int ad9680_setup(struct spi_device *spi, bool ad9234)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9680_jesd204_link_config link_config;
	unsigned int pll_stat;
	unsigned int i;
	int ret, tmp = 1;
	static const u32 sfdr_optim_regs[] = {
		0x16, 0x18, 0x19, 0x1A, 0x30, 0x11A, 0x934, 0x935
	};
	u32 sfdr_optim_vals[ARRAY_SIZE(sfdr_optim_regs)];

	ret = ad9680_request_clks(conv);
	if (ret)
		return ret;

#ifdef CONFIG_OF
	if (spi->dev.of_node)
		tmp = of_property_read_u32_array(
			spi->dev.of_node, "adi,sfdr-optimization-config",
			sfdr_optim_vals, ARRAY_SIZE(sfdr_optim_regs));
#endif

	ad9680_spi_write(spi, 0x000, 0x81);	// RESET
	mdelay(5);
	ad9680_spi_write(spi, 0x001, 0x01);	// RESET
	mdelay(1);

	ret = ad9680_spi_write(spi, 0x008, 0x03);	// select both channels
	ret |= ad9680_spi_write(spi, 0x201, 0x00);	// full sample rate (decimation = 1)

	if (tmp == 0) {
		for (; tmp < ARRAY_SIZE(sfdr_optim_regs); tmp++)
			ret |= ad9680_spi_write(spi, sfdr_optim_regs[tmp],
						sfdr_optim_vals[tmp]);
	}

	memset(&link_config, 0x00, sizeof(link_config));
	link_config.did = 0;
	link_config.bid = 1;
	link_config.num_lanes = 4;
	for (i = 0; i < link_config.num_lanes; i++) {
		link_config.lid[i] = i;
		link_config.lane_mux[i] = i;
	}
	link_config.num_converters = 2;
	link_config.octets_per_frame = 1;
	link_config.frames_per_multiframe = 32;
	link_config.converter_resolution = ad9234 ? 12 : 14;
	link_config.bits_per_sample = 16;
	link_config.scrambling = true;

	if (conv->sysref_clk) {
		link_config.subclass = 1;
		link_config.sysref.mode = AD9680_SYSREF_CONTINUOUS;
	} else {
		link_config.subclass = 0;
		link_config.sysref.mode = AD9680_SYSREF_DISABLED;
	}

	link_config.sysref.capture_falling_edge = true;
	link_config.sysref.valid_falling_edge = false;

	ret = ad9680_setup_link(spi, &link_config);
	if (ret < 0)
		goto err;

	ret = ad9680_setup_jesd204_link(conv, conv->adc_clk);
	if (ret < 0)
		goto err;
	mdelay(20);
	pll_stat = ad9680_spi_read(conv->spi, 0x56f);

	dev_info(&conv->spi->dev, "AD9680 PLL %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED");

	ret = clk_prepare_enable(conv->lane_clk);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		goto err;
	}

	return 0;
err:
	clk_disable_unprepare(conv->clk);
	clk_disable_unprepare(conv->sysref_clk);

	return ret;
}

static int ad9684_setup(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	unsigned int clk_stat;
	int ret;

	conv->clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(conv->clk))
		return PTR_ERR(conv->clk);

	ret = ad9680_spi_write(spi, 0x000, 0x81);
	mdelay(10);
	ret |= ad9680_spi_write(spi, 0x001, 0x02);
	mdelay(10);

	ret |= ad9680_spi_write(spi, 0x03f, 0x80);
	ret |= ad9680_spi_write(spi, 0x040, 0xbf);
	ret |= ad9680_spi_write(spi, 0x568, 0x01);

	clk_stat = ad9680_spi_read(spi, 0x11c);
	dev_info(&spi->dev, "AD9684 input clock %s\n",
		 clk_stat & 0x01 ? "DETECTED" : "NOT DETECTED");

	return ret;
}

static int ad9694_setup_jesd204_link(struct axiadc_converter *conv,
	unsigned int sample_rate)
{
	unsigned long lane_rate_kHz;
	unsigned long sysref_rate;
	unsigned int val;
	int ret;

	if (conv->id == CHIPID_AD9094)
		sysref_rate = DIV_ROUND_CLOSEST(sample_rate, 128);
	else
		sysref_rate = DIV_ROUND_CLOSEST(sample_rate, 32);
	lane_rate_kHz = DIV_ROUND_CLOSEST(sample_rate, 100);

	if (lane_rate_kHz < 1687500 || lane_rate_kHz > 15000000) {
		dev_err(&conv->spi->dev, "Lane rate %lu Mbps out of bounds. Must be between 1687.5 and 15000 Mbps",
			lane_rate_kHz / 1000);
		return -EINVAL;;
	}

	if (lane_rate_kHz < 3375000)
		val = 0x5;
	else if (lane_rate_kHz < 6750000)
		val = 0x1;
	else if(lane_rate_kHz < 13500000)
		val = 0x0;
	else
		val = 0x3;

	ad9680_spi_write(conv->spi, 0x56e, val << 4);

	/* Required sequence after link reset */
	ad9680_spi_write(conv->spi, 0x1228, 0x4f);
	ad9680_spi_write(conv->spi, 0x1228, 0x0f);
	ad9680_spi_write(conv->spi, 0x1222, 0x04);
	ad9680_spi_write(conv->spi, 0x1222, 0x00);
	ad9680_spi_write(conv->spi, 0x1262, 0x08);
	ad9680_spi_write(conv->spi, 0x1262, 0x00);

	ret = clk_set_rate(conv->sysref_clk, sysref_rate);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set SYSREF clock to %lu kHz: %d\n",
			sysref_rate / 1000, ret);
		return ret;
	}

	ret = clk_set_rate(conv->lane_clk, lane_rate_kHz);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set lane rate to %lu kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	return 0;
}

static int ad9694_setup(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9680_jesd204_link_config link_config;
	unsigned int pll_stat;
	unsigned int val;
	unsigned int i;
	int ret;

	ret = ad9680_request_clks(conv);
	if (ret)
		return ret;

	ad9680_spi_write(spi, 0x000, 0x81); /* RESET */
	mdelay(5);

	/* Configure A/B */
	ret |= ad9680_spi_write(spi, 0x009, 0x03); /* select pair A/B */
	ret |= ad9680_spi_write(spi, 0x008, 0x03); /* select both channels */

	ret |= ad9680_spi_write(spi, 0x108, 0x00); /* Clock divider = 1 */

	memset(&link_config, 0x00, sizeof(link_config));
	link_config.did = 0;
	link_config.bid = 0;
	link_config.num_lanes = 2;
	for (i = 0; i < link_config.num_lanes; i++) {
		link_config.lid[i] = i;
		link_config.lane_mux[i] = i;
	}
	link_config.num_converters = 2;
	link_config.octets_per_frame = 1;
	link_config.frames_per_multiframe = 32;
	link_config.converter_resolution = 8;
	link_config.bits_per_sample = 8;
	link_config.scrambling = true;

	if (conv->sysref_clk) {
		link_config.subclass = 1;
		link_config.sysref.mode = AD9680_SYSREF_ONESHOT;
	} else {
		link_config.subclass = 0;
		link_config.sysref.mode = AD9680_SYSREF_DISABLED;
	}

	link_config.sysref.capture_falling_edge = true;
	link_config.sysref.valid_falling_edge = false;

	ret = ad9680_setup_link(spi, &link_config);
	if (ret < 0)
		goto err;

	ret |= ad9680_spi_write(spi, 0x001, 0x02); /* datapath soft reset */
	mdelay(1);

	ret = ad9694_setup_jesd204_link(conv, conv->adc_clk);
	if (ret < 0)
		goto err;
	mdelay(20);
	pll_stat = ad9680_spi_read(conv->spi, 0x56f);

	dev_info(&conv->spi->dev, "%s PLL %s\n",
		 (conv->id == CHIPID_AD9094) ? "AD9094" : "AD9694",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED");

	/* Re-arm the SYSREF in oneshot mode */
	if (link_config.sysref.mode == AD9680_SYSREF_ONESHOT) {
		val = 0x04;

		if (link_config.sysref.capture_falling_edge)
			val |= 0x08;

		if (link_config.sysref.valid_falling_edge)
			val |= 0x10;
		ad9680_spi_write(spi, 0x120, val);
	}

	if (conv->id == CHIPID_AD9094) {
		/* Export the common-mode voltage to the VCM_CD/VREF */
		ad9680_spi_write(spi, 0x1908, 0x04);
		ad9680_spi_write(spi, 0x18A6, 0x00);
		ad9680_spi_write(spi, 0x18E6, 0x00);
		ad9680_spi_write(spi, 0x18E0, 0x04);
		ad9680_spi_write(spi, 0x18E1, 0x1c);
		ad9680_spi_write(spi, 0x18E2, 0x14);
		ad9680_spi_write(spi, 0x18E3, 0x56);

		/* Set buffer 1 & 2 currents to 440 uA */
		ad9680_spi_write(spi, 0x1A4C, 0x16);
		ad9680_spi_write(spi, 0x1A4D, 0x16);

		/* Set input full-scale range to 2.16Vpp */
		ad9680_spi_write(spi, 0x1910, 0x00);
	}

	ret = clk_prepare_enable(conv->lane_clk);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		goto err;
	}

	schedule_delayed_work(&conv->watchdog_work, HZ);

	conv->sample_rate_read_only = true;
	conv->running = true;

	return 0;
err:
	clk_disable_unprepare(conv->clk);
	clk_disable_unprepare(conv->sysref_clk);
	return ret;
}

static void ad9694_serdes_pll_watchdog(struct work_struct *work)
{
	struct axiadc_converter *conv =
		container_of(work, struct axiadc_converter, watchdog_work.work);
	unsigned int clock_detected, serdes_locked;
	int ret;

	clock_detected = ad9680_spi_read(conv->spi, 0x11b);
	serdes_locked = ad9680_spi_read(conv->spi, 0x56f);

	/* Restart if clock is detected, but SERDES is not locked */
	if ((clock_detected & 0x01) && !(serdes_locked & 0x80)) {
		dev_err(&conv->spi->dev, "Lost SERDES PLL lock, re-initializing.");

		if (conv->running) {
			clk_disable_unprepare(conv->lane_clk);
			conv->running = false;
		}

		 /* datapath soft reset */
		ad9680_spi_write(conv->spi, 0x001, 0x02);
		mdelay(1);

		/* Required sequence after link reset */
		ad9680_spi_write(conv->spi, 0x1228, 0x4f);
		ad9680_spi_write(conv->spi, 0x1228, 0x0f);
		ad9680_spi_write(conv->spi, 0x1222, 0x04);
		ad9680_spi_write(conv->spi, 0x1222, 0x00);
		ad9680_spi_write(conv->spi, 0x1262, 0x08);
		ad9680_spi_write(conv->spi, 0x1262, 0x00);

		mdelay(20);
		serdes_locked = ad9680_spi_read(conv->spi, 0x56f);

		dev_info(&conv->spi->dev, "AD9694 PLL %s\n",
			 (serdes_locked & 0x80) ? "LOCKED" : "UNLOCKED");

		ret = clk_prepare_enable(conv->lane_clk);
		WARN_ON(!!ret);
		if (ret == 0)
			conv->running = true;
	}

	schedule_delayed_work(&conv->watchdog_work, HZ);
}



static int ad9680_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return ad9680_get_scale(conv, chan, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate_scaled(conv->clk, &conv->adc_clkscale);

		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int ad9680_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return ad9680_set_scale(conv, chan, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		if (conv->sample_rate_read_only)
			return -EPERM;

		if (conv->id == CHIPID_AD9680)
			return ad9680_set_sample_rate(conv, val);

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				 "Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9680_request_fd_irqs(struct axiadc_converter *conv)
{
	struct device *dev = &conv->spi->dev;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get(dev, "fastdetect-a", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);
		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9680_fdA_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-a", conv);
		if (ret < 0)
			return ret;
	}

	gpio = devm_gpiod_get(dev, "fastdetect-b", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);
		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9680_fdB_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-b", conv);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9680_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_IQCOR_ENB | ADI_ENABLE);
	}

	return 0;
}

static int ad9680_probe(struct spi_device *spi)
{
	bool master_slave_2x_quirk = false;
	struct axiadc_converter *conv;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;

	INIT_DELAYED_WORK(&conv->watchdog_work, ad9694_serdes_pll_watchdog);

	spi_set_drvdata(spi, conv);
	conv->spi = spi;

	conv->pwrdown_gpio = devm_gpiod_get_optional(&spi->dev, "powerdown",
		GPIOD_OUT_LOW);
	if (IS_ERR(conv->pwrdown_gpio))
		return PTR_ERR(conv->pwrdown_gpio);

	conv->id = ad9680_spi_read(spi, AD9680_REG_CHIP_ID_LOW);
	if (conv->id != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		return -ENODEV;
	}

	switch (conv->id) {
	case CHIPID_AD9234:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9234];
		ret = ad9680_setup(spi, true);
		break;
	case CHIPID_AD9680:
#ifdef CONFIG_OF
		if (spi->dev.of_node)
			master_slave_2x_quirk = of_property_read_bool(
				spi->dev.of_node, "adi,master-slave-2x-quirk");
#endif
		conv->chip_info = &axiadc_chip_info_tbl[master_slave_2x_quirk ?
			ID_AD9680_x2 : ID_AD9680];
		ret = ad9680_setup(spi, false);
		break;
	case CHIPID_AD9684:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9684];
		ret = ad9684_setup(spi);
		break;
	case CHIPID_AD9694:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9694];
		ret = ad9694_setup(spi);
		break;
	case CHIPID_AD9094:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9094];
		ret = ad9694_setup(spi);
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		return -ENODEV;
	}

	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
		return ret;
	}

	conv->adc_output_mode = AD9680_OUTPUT_MODE_TWOS_COMPLEMENT;
	ret = ad9680_outputmode_set(spi, conv->adc_output_mode);
	if (ret < 0)
		return ret;

	conv->reg_access = ad9680_reg_access;
	conv->write_raw = ad9680_write_raw;
	conv->read_raw = ad9680_read_raw;
	conv->read_event_value = ad9680_read_thresh,
	conv->write_event_value = ad9680_write_thresh,
	conv->read_event_config = ad9680_read_thresh_en,
	conv->write_event_config = ad9680_write_thresh_en,
	conv->post_setup = ad9680_post_setup;
	conv->set_pnsel = ad9680_set_pnsel;

	device_create_file(&spi->dev, &dev_attr_status);

	if (conv->id == CHIPID_AD9680) {
		ret = ad9680_request_fd_irqs(conv);
		if (ret < 0)
			return ret;

		/* Enable Fast Detect output */
		ret = ad9680_spi_write(spi, AD9680_REG_THRESH_CTRL, 0x1);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9680_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&conv->watchdog_work);
	if (conv->running) {
		clk_disable_unprepare(conv->lane_clk);
		clk_disable_unprepare(conv->sysref_clk);
		clk_disable_unprepare(conv->clk);
		conv->running = false;
	}

	return 0;
}

static const struct spi_device_id ad9680_id[] = {
	{ "ad9680", CHIPID_AD9680 },
	{ "ad9234", CHIPID_AD9234 },
	{ "ad9684", CHIPID_AD9684 },
	{ "ad9694", CHIPID_AD9694 },
	{ "ad9094", CHIPID_AD9094 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9680_id);

static struct spi_driver ad9680_driver = {
	.driver = {
		   .name = "ad9680",
		   .owner = THIS_MODULE,
	},
	.probe = ad9680_probe,
	.remove = ad9680_remove,
	.id_table = ad9680_id,
};
module_spi_driver(ad9680_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9680 ADC");
MODULE_LICENSE("GPL v2");
