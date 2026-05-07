// SPDX-License-Identifier: GPL-2.0
/*
 * Core driver for AD5686 and similar Digital to Analog Converters
 *
 * Copyright 2011-2026 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/sysfs.h>
#include <linux/units.h>
#include <linux/wordpart.h>

#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "ad5686.h"

static const char * const ad5686_powerdown_modes[] = {
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"three_state"
};

static int ad5310_control_sync(struct ad5686_state *st)
{
	unsigned int pd_val = st->pwr_down_mask & st->pwr_down_mode;

	return ad5686_write(st, AD5686_CMD_CONTROL_REG, 0,
			    FIELD_PREP(AD5310_PD_MSK, pd_val) |
			    FIELD_PREP(AD5310_REF_BIT_MSK, st->use_internal_vref ? 0 : 1) |
			    FIELD_PREP(AD5310_GAIN_BIT_MSK, st->double_scale));
}

static int ad5683_control_sync(struct ad5686_state *st)
{
	unsigned int pd_val = st->pwr_down_mask & st->pwr_down_mode;

	return ad5686_write(st, AD5686_CMD_CONTROL_REG, 0,
			    FIELD_PREP(AD5683_PD_MSK, pd_val) |
			    FIELD_PREP(AD5683_REF_BIT_MSK, st->use_internal_vref ? 0 : 1) |
			    FIELD_PREP(AD5683_GAIN_BIT_MSK, st->double_scale));
}

static inline unsigned int ad5686_pd_mask_shift(const struct iio_chan_spec *chan)
{
	if (chan->channel == chan->address)
		return chan->channel * 2;

	/* one-hot encoding is used in dual/quad channel devices */
	return __ffs(chan->address) * 2;
}

static int ad5686_get_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	unsigned int shift = ad5686_pd_mask_shift(chan);
	struct ad5686_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return ((st->pwr_down_mode >> shift) & 0x3) - 1;
}

static int ad5686_set_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int mode)
{
	unsigned int shift = ad5686_pd_mask_shift(chan);
	struct ad5686_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	st->pwr_down_mode &= ~(0x3 << shift);
	st->pwr_down_mode |= ((mode + 1) << shift);

	return 0;
}

static const struct iio_enum ad5686_powerdown_mode_enum = {
	.items = ad5686_powerdown_modes,
	.num_items = ARRAY_SIZE(ad5686_powerdown_modes),
	.get = ad5686_get_powerdown_mode,
	.set = ad5686_set_powerdown_mode,
};

static ssize_t ad5686_read_dac_powerdown(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	unsigned int shift = ad5686_pd_mask_shift(chan);
	struct ad5686_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return sysfs_emit(buf, "%d\n", !!(st->pwr_down_mask & (0x3 << shift)));
}

static ssize_t ad5686_write_dac_powerdown(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  const char *buf,
					  size_t len)
{
	unsigned int val, shift = ad5686_pd_mask_shift(chan);
	struct ad5686_state *st = iio_priv(indio_dev);
	bool readin;
	u8 address;
	int ret;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	if (readin)
		st->pwr_down_mask |= (0x3 << shift);
	else
		st->pwr_down_mask &= ~(0x3 << shift);

	switch (st->chip_info->regmap_type) {
	case AD5310_REGMAP:
		ret = ad5310_control_sync(st);
		break;
	case AD5683_REGMAP:
		ret = ad5683_control_sync(st);
		break;
	case AD5686_REGMAP:
		/* AD5674R/AD5679R have 16 channels and 2 powerdown registers */
		val = st->pwr_down_mask & st->pwr_down_mode;
		if (chan->channel > 0x7) {
			address = 0x8;
			val = upper_16_bits(val);
		} else {
			address = 0x0;
			val = lower_16_bits(val);
		}
		ret = ad5686_write(st, AD5686_CMD_POWERDOWN_DAC, address, val);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static int ad5686_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5686_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = ad5686_read(st, chan->address);
		if (ret < 0)
			return ret;
		*val = (ret >> chan->scan_type.shift) &
			GENMASK(chan->scan_type.realbits - 1, 0);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (st->double_scale) {
			*val = st->scale_avail[2];
			*val2 = st->scale_avail[3];
		} else {
			*val = st->scale_avail[0];
			*val2 = st->scale_avail[1];
		}
		return IIO_VAL_INT_PLUS_NANO;
	}
	return -EINVAL;
}

static int ad5686_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!in_range(val, 0, 1 << chan->scan_type.realbits))
			return -EINVAL;

		return ad5686_write(st, AD5686_CMD_WRITE_INPUT_N_UPDATE_N,
				    chan->address, val << chan->scan_type.shift);
	case IIO_CHAN_INFO_SCALE:
		if (val == st->scale_avail[0] && val2 == st->scale_avail[1])
			st->double_scale = false;
		else if (val == st->scale_avail[2] && val2 == st->scale_avail[3])
			st->double_scale = true;
		else
			return -EINVAL;

		switch (st->chip_info->regmap_type) {
		case AD5310_REGMAP:
			return ad5310_control_sync(st);
		case AD5683_REGMAP:
			return ad5683_control_sync(st);
		case AD5686_REGMAP:
			gpiod_set_value_cansleep(st->gain_gpio, st->double_scale);
			return 0;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int ad5686_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return -EINVAL;
	}
}

static int ad5686_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long mask)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*type = IIO_VAL_INT_PLUS_NANO;
		*vals = st->scale_avail;
		*length = ARRAY_SIZE(st->scale_avail);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad5686_info = {
	.read_raw = ad5686_read_raw,
	.write_raw = ad5686_write_raw,
	.write_raw_get_fmt = ad5686_write_raw_get_fmt,
	.read_avail = ad5686_read_avail,
};

static const struct iio_chan_spec_ext_info ad5686_ext_info[] = {
	{
		.name = "powerdown",
		.read = ad5686_read_dac_powerdown,
		.write = ad5686_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &ad5686_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", IIO_SHARED_BY_TYPE, &ad5686_powerdown_mode_enum),
	{ }
};

#define AD5868_CHANNEL(chan, addr, bits, _shift) {		\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.output = 1,					\
		.channel = chan,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
		.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SCALE),\
		.address = addr,				\
		.scan_index = chan,				\
		.scan_type = {					\
			.sign = 'u',				\
			.realbits = (bits),			\
			.storagebits = 16,			\
			.shift = (_shift),			\
		},						\
		.ext_info = ad5686_ext_info,			\
}

#define DECLARE_AD5683_CHANNELS(name, bits, _shift)		\
static const struct iio_chan_spec name[] = {			\
		AD5868_CHANNEL(0, 0, bits, _shift),		\
}

#define DECLARE_AD5338_CHANNELS(name, bits, _shift)		\
static const struct iio_chan_spec name[] = {			\
		AD5868_CHANNEL(0, 1, bits, _shift),		\
		AD5868_CHANNEL(1, 8, bits, _shift),		\
}

#define DECLARE_AD5686_CHANNELS(name, bits, _shift)		\
static const struct iio_chan_spec name[] = {			\
		AD5868_CHANNEL(0, 1, bits, _shift),		\
		AD5868_CHANNEL(1, 2, bits, _shift),		\
		AD5868_CHANNEL(2, 4, bits, _shift),		\
		AD5868_CHANNEL(3, 8, bits, _shift),		\
}

#define DECLARE_AD5676_CHANNELS(name, bits, _shift)		\
static const struct iio_chan_spec name[] = {			\
		AD5868_CHANNEL(0, 0, bits, _shift),		\
		AD5868_CHANNEL(1, 1, bits, _shift),		\
		AD5868_CHANNEL(2, 2, bits, _shift),		\
		AD5868_CHANNEL(3, 3, bits, _shift),		\
		AD5868_CHANNEL(4, 4, bits, _shift),		\
		AD5868_CHANNEL(5, 5, bits, _shift),		\
		AD5868_CHANNEL(6, 6, bits, _shift),		\
		AD5868_CHANNEL(7, 7, bits, _shift),		\
}

#define DECLARE_AD5679_CHANNELS(name, bits, _shift)		\
static const struct iio_chan_spec name[] = {			\
		AD5868_CHANNEL(0, 0, bits, _shift),		\
		AD5868_CHANNEL(1, 1, bits, _shift),		\
		AD5868_CHANNEL(2, 2, bits, _shift),		\
		AD5868_CHANNEL(3, 3, bits, _shift),		\
		AD5868_CHANNEL(4, 4, bits, _shift),		\
		AD5868_CHANNEL(5, 5, bits, _shift),		\
		AD5868_CHANNEL(6, 6, bits, _shift),		\
		AD5868_CHANNEL(7, 7, bits, _shift),		\
		AD5868_CHANNEL(8, 8, bits, _shift),		\
		AD5868_CHANNEL(9, 9, bits, _shift),		\
		AD5868_CHANNEL(10, 10, bits, _shift),		\
		AD5868_CHANNEL(11, 11, bits, _shift),		\
		AD5868_CHANNEL(12, 12, bits, _shift),		\
		AD5868_CHANNEL(13, 13, bits, _shift),		\
		AD5868_CHANNEL(14, 14, bits, _shift),		\
		AD5868_CHANNEL(15, 15, bits, _shift),		\
}

/* single-channel */
DECLARE_AD5683_CHANNELS(ad5310r_channels, 10, 2);
DECLARE_AD5683_CHANNELS(ad5311r_channels, 10, 6);
DECLARE_AD5683_CHANNELS(ad5681r_channels, 12, 4);
DECLARE_AD5683_CHANNELS(ad5682r_channels, 14, 2);
DECLARE_AD5683_CHANNELS(ad5683r_channels, 16, 0);

/* dual-channel */
DECLARE_AD5338_CHANNELS(ad5337r_channels, 8, 8);
DECLARE_AD5338_CHANNELS(ad5338r_channels, 10, 6);
DECLARE_AD5338_CHANNELS(ad5687r_channels, 12, 4);
DECLARE_AD5338_CHANNELS(ad5689r_channels, 16, 0);

/* quad-channel */
DECLARE_AD5686_CHANNELS(ad5317r_channels, 10, 6);
DECLARE_AD5686_CHANNELS(ad5684r_channels, 12, 4);
DECLARE_AD5686_CHANNELS(ad5685r_channels, 14, 2);
DECLARE_AD5686_CHANNELS(ad5686r_channels, 16, 0);

/* 8-channel */
DECLARE_AD5676_CHANNELS(ad5672r_channels, 12, 4);
DECLARE_AD5676_CHANNELS(ad5676r_channels, 16, 0);

/* 16-channel */
DECLARE_AD5679_CHANNELS(ad5674r_channels, 12, 4);
DECLARE_AD5679_CHANNELS(ad5679r_channels, 16, 0);

const struct ad5686_chip_info ad5310r_chip_info = {
	.channels = ad5310r_channels,
	.int_vref_mv = 2500,
	.num_channels = 1,
	.regmap_type = AD5310_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5310r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5311r_chip_info = {
	.channels = ad5311r_channels,
	.int_vref_mv = 2500,
	.num_channels = 1,
	.regmap_type = AD5683_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5311r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5681r_chip_info = {
	.channels = ad5681r_channels,
	.int_vref_mv = 2500,
	.num_channels = 1,
	.regmap_type = AD5683_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5681r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5682r_chip_info = {
	.channels = ad5682r_channels,
	.int_vref_mv = 2500,
	.num_channels = 1,
	.regmap_type = AD5683_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5682r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5683_chip_info = {
	.channels = ad5683r_channels,
	.num_channels = 1,
	.regmap_type = AD5683_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5683_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5683r_chip_info = {
	.channels = ad5683r_channels,
	.int_vref_mv = 2500,
	.num_channels = 1,
	.regmap_type = AD5683_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5683r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5337r_chip_info = {
	.channels = ad5337r_channels,
	.int_vref_mv = 2500,
	.num_channels = 2,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5337r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5338r_chip_info = {
	.channels = ad5338r_channels,
	.int_vref_mv = 2500,
	.num_channels = 2,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5338r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5687_chip_info = {
	.channels = ad5687r_channels,
	.num_channels = 2,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5687_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5687r_chip_info = {
	.channels = ad5687r_channels,
	.int_vref_mv = 2500,
	.num_channels = 2,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5687r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5689_chip_info = {
	.channels = ad5689r_channels,
	.num_channels = 2,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5689_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5689r_chip_info = {
	.channels = ad5689r_channels,
	.int_vref_mv = 2500,
	.num_channels = 2,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5689r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5317r_chip_info = {
	.channels = ad5317r_channels,
	.int_vref_mv = 2500,
	.num_channels = 4,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5317r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5684_chip_info = {
	.channels = ad5684r_channels,
	.num_channels = 4,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5684_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5684r_chip_info = {
	.channels = ad5684r_channels,
	.int_vref_mv = 2500,
	.num_channels = 4,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5684r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5685r_chip_info = {
	.channels = ad5685r_channels,
	.int_vref_mv = 2500,
	.num_channels = 4,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5685r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5686_chip_info = {
	.channels = ad5686r_channels,
	.num_channels = 4,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5686_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5686r_chip_info = {
	.channels = ad5686r_channels,
	.int_vref_mv = 2500,
	.num_channels = 4,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5686r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5672r_chip_info = {
	.channels = ad5672r_channels,
	.int_vref_mv = 2500,
	.num_channels = 8,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5672r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5676_chip_info = {
	.channels = ad5676r_channels,
	.num_channels = 8,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5676_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5676r_chip_info = {
	.channels = ad5676r_channels,
	.int_vref_mv = 2500,
	.num_channels = 8,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5676r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5674_chip_info = {
	.channels = ad5674r_channels,
	.num_channels = 16,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5674_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5674r_chip_info = {
	.channels = ad5674r_channels,
	.int_vref_mv = 2500,
	.num_channels = 16,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5674r_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5679_chip_info = {
	.channels = ad5679r_channels,
	.num_channels = 16,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5679_chip_info, "IIO_AD5686");

const struct ad5686_chip_info ad5679r_chip_info = {
	.channels = ad5679r_channels,
	.int_vref_mv = 2500,
	.num_channels = 16,
	.regmap_type = AD5686_REGMAP,
};
EXPORT_SYMBOL_NS_GPL(ad5679r_chip_info, "IIO_AD5686");

static void ad5686_init_scale_avail(struct ad5686_state *st)
{
	int realbits = st->chip_info->channels[0].scan_type.realbits;
	s64 tmp;

	tmp = 2ULL * st->vref_mv * NANO >> realbits;
	st->scale_avail[2] = div_s64_rem(tmp, NANO, &st->scale_avail[3]);
	st->scale_avail[0] = div_s64_rem(tmp >> 1, NANO, &st->scale_avail[1]);
}

static irqreturn_t ad5686_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct iio_buffer *buffer = indio_dev->buffer;
	struct ad5686_state *st = iio_priv(indio_dev);
	const struct iio_chan_spec *chan;
	u16 val[AD5686_MAX_CHANNELS];
	int ret, ch, i = 0;
	bool async_update;
	u8 cmd;

	ret = iio_pop_from_buffer(buffer, val);
	if (ret)
		goto out;

	mutex_lock(&st->lock);

	async_update = st->ldac_gpio && bitmap_weight(indio_dev->active_scan_mask,
						      iio_get_masklength(indio_dev)) > 1;
	if (async_update) {
		/* use ldac to update all channels simultaneously */
		cmd = AD5686_CMD_WRITE_INPUT_N;
		gpiod_set_value_cansleep(st->ldac_gpio, 0);
	} else {
		cmd = AD5686_CMD_WRITE_INPUT_N_UPDATE_N;
	}

	iio_for_each_active_channel(indio_dev, ch) {
		chan = &indio_dev->channels[ch];
		ret = st->ops->write(st, cmd, chan->address, val[i++]);
		if (ret)
			break;
	}

	if (!ret && st->ops->sync)
		ret = st->ops->sync(st); /* flush all pending transfers */

	if (async_update)
		gpiod_set_value_cansleep(st->ldac_gpio, 1);

	mutex_unlock(&st->lock);
out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

int ad5686_probe(struct device *dev,
		 const struct ad5686_chip_info *chip_info,
		 const char *name, const struct ad5686_bus_ops *ops,
		 void *bus_data)
{
	struct reset_control *rstc;
	struct iio_dev *indio_dev;
	struct ad5686_state *st;
	unsigned int i, shift;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return  -ENOMEM;

	st = iio_priv(indio_dev);

	st->dev = dev;
	st->ops = ops;
	st->bus_data = bus_data;
	st->chip_info = chip_info;

	ret = devm_regulator_get_enable_optional(dev, "vdd");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to enable vdd supply\n");

	ret = devm_regulator_get_enable_optional(dev, "vlogic");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to enable vlogic supply\n");

	ret = devm_regulator_get_enable_read_voltage(dev, "vref");
	if (ret == -ENODEV) /* vcc-supply is deprecated, but supported still */
		ret = devm_regulator_get_enable_read_voltage(dev, "vcc");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to read vref voltage\n");

	st->use_internal_vref = ret == -ENODEV;
	st->vref_mv = st->use_internal_vref ? st->chip_info->int_vref_mv : ret / 1000;

	if (!st->vref_mv)
		return dev_err_probe(dev, -EINVAL,
				     "invalid or not provided vref voltage\n");

	rstc = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc),
				     "Failed to get reset controller\n");

	st->ldac_gpio = devm_gpiod_get_optional(dev, "ldac", GPIOD_OUT_HIGH);
	if (IS_ERR(st->ldac_gpio))
		return dev_err_probe(dev, PTR_ERR(st->ldac_gpio),
				     "Failed to get LDAC GPIO\n");

	st->gain_gpio = devm_gpiod_get_optional(dev, "gain", GPIOD_OUT_LOW);
	if (IS_ERR(st->gain_gpio))
		return dev_err_probe(dev, PTR_ERR(st->gain_gpio),
				     "Failed to get GAIN GPIO\n");

	ad5686_init_scale_avail(st);

	/* Set all the power down mode for all channels to 1K pulldown */
	st->pwr_down_mode = ~0U;
	st->pwr_down_mask = ~0U;
	for (i = 0; i < st->chip_info->num_channels; i++) {
		shift = ad5686_pd_mask_shift(&st->chip_info->channels[i]);
		st->pwr_down_mask &= ~(0x3 << shift); /* powered up state */
		st->pwr_down_mode &= ~(0x3 << shift);
		st->pwr_down_mode |= (0x01 << shift);
	}

	indio_dev->name = name;
	indio_dev->info = &ad5686_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	switch (st->chip_info->regmap_type) {
	case AD5310_REGMAP:
		ret = ad5310_control_sync(st);
		if (ret)
			return ret;
		break;
	case AD5683_REGMAP:
		ret = ad5683_control_sync(st);
		if (ret)
			return ret;
		break;
	case AD5686_REGMAP:
		ret = ad5686_write(st, AD5686_CMD_INTERNAL_REFER_SETUP, 0,
				   !st->use_internal_vref);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	ret = devm_iio_triggered_buffer_setup_ext(dev, indio_dev, NULL,
						  &ad5686_trigger_handler,
						  IIO_BUFFER_DIRECTION_OUT,
						  NULL, NULL);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_NS_GPL(ad5686_probe, "IIO_AD5686");

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5686/85/84 DAC");
MODULE_LICENSE("GPL v2");
