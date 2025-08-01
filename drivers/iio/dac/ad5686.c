// SPDX-License-Identifier: GPL-2.0
/*
 * AD5686R, AD5685R, AD5684R Digital to analog converters  driver
 *
 * Copyright 2011 Analog Devices Inc.
 */

#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "ad5686.h"

static const char * const ad5686_powerdown_modes[] = {
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"three_state"
};

static int ad5686_get_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	return ((st->pwr_down_mode >> (chan->channel * 2)) & 0x3) - 1;
}

static int ad5686_set_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int mode)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	st->pwr_down_mode &= ~(0x3 << (chan->channel * 2));
	st->pwr_down_mode |= ((mode + 1) << (chan->channel * 2));

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
	struct ad5686_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", !!(st->pwr_down_mask &
				       (0x3 << (chan->channel * 2))));
}

static ssize_t ad5686_write_dac_powerdown(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  const char *buf,
					  size_t len)
{
	bool readin;
	int ret;
	struct ad5686_state *st = iio_priv(indio_dev);
	unsigned int val, ref_bit_msk;
	u8 shift, address = 0;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	if (readin)
		st->pwr_down_mask |= (0x3 << (chan->channel * 2));
	else
		st->pwr_down_mask &= ~(0x3 << (chan->channel * 2));

	switch (st->chip_info->regmap_type) {
	case AD5310_REGMAP:
		shift = 9;
		ref_bit_msk = AD5310_REF_BIT_MSK;
		break;
	case AD5683_REGMAP:
		shift = 13;
		ref_bit_msk = AD5683_REF_BIT_MSK;
		break;
	case AD5686_REGMAP:
		shift = 0;
		ref_bit_msk = 0;
		/* AD5674R/AD5679R have 16 channels and 2 powerdown registers */
		if (chan->channel > 0x7)
			address = 0x8;
		break;
	case AD5693_REGMAP:
		shift = 13;
		ref_bit_msk = AD5693_REF_BIT_MSK;
		break;
	default:
		return -EINVAL;
	}

	val = ((st->pwr_down_mask & st->pwr_down_mode) << shift);
	if (!st->use_internal_vref)
		val |= ref_bit_msk;

	ret = st->write(st, AD5686_CMD_POWERDOWN_DAC,
			address, val >> (address * 2));

	return ret ? ret : len;
}

static int ad5686_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5686_state *st = iio_priv(indio_dev);
	struct pwm_state state;
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = st->read(st, chan->address);
		mutex_unlock(&st->lock);
		if (ret < 0)
			return ret;
		*val = (ret >> chan->scan_type.shift) &
			GENMASK(chan->scan_type.realbits - 1, 0);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!st->pwm)
			return -EINVAL;
		pwm_get_state(st->pwm, &state);
		*val = DIV_ROUND_CLOSEST_ULL(1000000000ULL, state.period);
		return IIO_VAL_INT;
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
	struct pwm_state state;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val > (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		mutex_lock(&st->lock);
		ret = st->write(st,
				AD5686_CMD_WRITE_INPUT_N_UPDATE_N,
				chan->address,
				val << chan->scan_type.shift);
		mutex_unlock(&st->lock);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!st->pwm)
			return -EINVAL;
		pwm_get_state(st->pwm, &state);

		state.period = DIV_ROUND_CLOSEST_ULL(1000000000ULL, val);
		pwm_set_relative_duty_cycle(&state, 50, 100);

		ret = pwm_apply_might_sleep(st->pwm, &state);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ad5686_trig_set_state(struct iio_trigger *trig,
				 bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ad5686_state *st = iio_priv(indio_dev);
	struct pwm_state pwm_st;

	if (!st->pwm)
		return -EINVAL;

	pwm_get_state(st->pwm, &pwm_st);
	pwm_st.enabled = state;

	return pwm_apply_might_sleep(st->pwm, &pwm_st);
}

static int ad5686_validate_trigger(struct iio_dev *indio_dev,
				    struct iio_trigger *trig)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_trigger_ops ad5686_trigger_ops = {
	.validate_device = &iio_trigger_validate_own_device,
	.set_trigger_state = &ad5686_trig_set_state,
};

static const struct iio_info ad5686_info = {
	.validate_trigger = &ad5686_validate_trigger,
	.read_raw = ad5686_read_raw,
	.write_raw = ad5686_write_raw,
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
	{ },
};

#define AD5868_CHANNEL(chan, addr, bits, _shift) {		\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.output = 1,					\
		.channel = chan,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | \
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),\
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

#define DECLARE_AD5693_CHANNELS(name, bits, _shift)		\
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

DECLARE_AD5693_CHANNELS(ad5310r_channels, 10, 2);
DECLARE_AD5693_CHANNELS(ad5311r_channels, 10, 6);
DECLARE_AD5338_CHANNELS(ad5337r_channels, 8, 8);
DECLARE_AD5338_CHANNELS(ad5338r_channels, 10, 6);
DECLARE_AD5676_CHANNELS(ad5672_channels, 12, 4);
DECLARE_AD5679_CHANNELS(ad5674r_channels, 12, 4);
DECLARE_AD5676_CHANNELS(ad5676_channels, 16, 0);
DECLARE_AD5679_CHANNELS(ad5679r_channels, 16, 0);
DECLARE_AD5686_CHANNELS(ad5684_channels, 12, 4);
DECLARE_AD5686_CHANNELS(ad5685r_channels, 14, 2);
DECLARE_AD5686_CHANNELS(ad5686_channels, 16, 0);
DECLARE_AD5693_CHANNELS(ad5693_channels, 16, 0);
DECLARE_AD5693_CHANNELS(ad5692r_channels, 14, 2);
DECLARE_AD5693_CHANNELS(ad5691r_channels, 12, 4);

static const struct ad5686_chip_info ad5686_chip_info_tbl[] = {
	[ID_AD5310R] = {
		.channels = ad5310r_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5310_REGMAP,
	},
	[ID_AD5311R] = {
		.channels = ad5311r_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5693_REGMAP,
	},
	[ID_AD5337R] = {
		.channels = ad5337r_channels,
		.int_vref_mv = 2500,
		.num_channels = 2,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5338R] = {
		.channels = ad5338r_channels,
		.int_vref_mv = 2500,
		.num_channels = 2,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5671R] = {
		.channels = ad5672_channels,
		.int_vref_mv = 2500,
		.num_channels = 8,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5672R] = {
		.channels = ad5672_channels,
		.int_vref_mv = 2500,
		.num_channels = 8,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5673R] = {
		.channels = ad5674r_channels,
		.int_vref_mv = 2500,
		.num_channels = 16,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5674R] = {
		.channels = ad5674r_channels,
		.int_vref_mv = 2500,
		.num_channels = 16,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5675R] = {
		.channels = ad5676_channels,
		.int_vref_mv = 2500,
		.num_channels = 8,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5676] = {
		.channels = ad5676_channels,
		.num_channels = 8,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5676R] = {
		.channels = ad5676_channels,
		.int_vref_mv = 2500,
		.num_channels = 8,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5677R] = {
		.channels = ad5679r_channels,
		.int_vref_mv = 2500,
		.num_channels = 16,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5679R] = {
		.channels = ad5679r_channels,
		.int_vref_mv = 2500,
		.num_channels = 16,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5681R] = {
		.channels = ad5691r_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5683_REGMAP,
	},
	[ID_AD5682R] = {
		.channels = ad5692r_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5683_REGMAP,
	},
	[ID_AD5683] = {
		.channels = ad5693_channels,
		.num_channels = 1,
		.regmap_type = AD5683_REGMAP,
	},
	[ID_AD5683R] = {
		.channels = ad5693_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5683_REGMAP,
	},
	[ID_AD5684] = {
		.channels = ad5684_channels,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5684R] = {
		.channels = ad5684_channels,
		.int_vref_mv = 2500,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5685R] = {
		.channels = ad5685r_channels,
		.int_vref_mv = 2500,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5686] = {
		.channels = ad5686_channels,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5686R] = {
		.channels = ad5686_channels,
		.int_vref_mv = 2500,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5691R] = {
		.channels = ad5691r_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5693_REGMAP,
	},
	[ID_AD5692R] = {
		.channels = ad5692r_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5693_REGMAP,
	},
	[ID_AD5693] = {
		.channels = ad5693_channels,
		.num_channels = 1,
		.regmap_type = AD5693_REGMAP,
	},
	[ID_AD5693R] = {
		.channels = ad5693_channels,
		.int_vref_mv = 2500,
		.num_channels = 1,
		.regmap_type = AD5693_REGMAP,
	},
	[ID_AD5694] = {
		.channels = ad5684_channels,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5694R] = {
		.channels = ad5684_channels,
		.int_vref_mv = 2500,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5696] = {
		.channels = ad5686_channels,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
	[ID_AD5696R] = {
		.channels = ad5686_channels,
		.int_vref_mv = 2500,
		.num_channels = 4,
		.regmap_type = AD5686_REGMAP,
	},
};

static irqreturn_t ad5686_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	const struct iio_chan_spec *chan;
	struct iio_buffer *buffer = indio_dev->buffer;
	struct ad5686_state *st = iio_priv(indio_dev);
	u8 sample[2];
	unsigned int i;
	u16 val;
	int ret;

	ret = iio_pop_from_buffer(buffer, sample);
	if (ret < 0)
		goto out;

	mutex_lock(&st->lock);
	for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
		val = (sample[1] << 8) + sample[0];

		chan = &indio_dev->channels[i];
		ret = st->write(st, AD5686_CMD_WRITE_INPUT_N_UPDATE_N,
				chan->address, val << chan->scan_type.shift);
	}
	mutex_unlock(&st->lock);

out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t ad5686_irq_handler(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ad5686_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trig);

	return IRQ_HANDLED;
}

int ad5686_probe(struct device *dev,
		 enum ad5686_supported_device_ids chip_type,
		 const char *name, ad5686_write_func write,
		 ad5686_read_func read, int irq)
{
	struct ad5686_state *st;
	struct iio_dev *indio_dev;
	struct pwm_state state;
	unsigned int val, ref_bit_msk;
	u8 cmd;
	int ret, i, voltage_uv = 0;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return  -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	st->write = write;
	st->read = read;

	mutex_init(&st->lock);

	st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d", name,
					  iio_device_id(indio_dev));
	if (st->trig == NULL)
		ret = -ENOMEM;

	st->trig->ops = &ad5686_trigger_ops;
	st->trig->dev.parent = dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);

	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret)
		return ret;

	/* select default trigger */
	indio_dev->trig = iio_trigger_get(st->trig);

	st->reg = devm_regulator_get_optional(dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;

		ret = regulator_get_voltage(st->reg);
		if (ret < 0)
			goto error_disable_reg;

		voltage_uv = ret;
	}

	/* PWM configuration */
	st->pwm = devm_pwm_get(dev, "pwm-trigger");
	if (!IS_ERR(st->pwm)) {
		/* Set a default pwm frequency of 1kHz and 50% duty cycle */
		pwm_init_state(st->pwm, &state);
		state.enabled = false;
		state.period = 1000000;
		pwm_set_relative_duty_cycle(&state, 50, 100);
		ret = pwm_apply_might_sleep(st->pwm, &state);
		if (ret < 0)
			return ret;
	} else {
		st->pwm = NULL;
	}

	/* Configure IRQ */
	if (irq) {
		ret = devm_request_threaded_irq(dev, irq, NULL, ad5686_irq_handler,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"ad5686 irq", indio_dev);
		if (ret)
			return ret;

		st->irq = irq;
	}

	st->chip_info = &ad5686_chip_info_tbl[chip_type];

	if (voltage_uv)
		st->vref_mv = voltage_uv / 1000;
	else
		st->vref_mv = st->chip_info->int_vref_mv;

	/* Set all the power down mode for all channels to 1K pulldown */
	for (i = 0; i < st->chip_info->num_channels; i++)
		st->pwr_down_mode |= (0x01 << (i * 2));

	indio_dev->name = name;
	indio_dev->info = &ad5686_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	mutex_init(&st->lock);

	switch (st->chip_info->regmap_type) {
	case AD5310_REGMAP:
		cmd = AD5686_CMD_CONTROL_REG;
		ref_bit_msk = AD5310_REF_BIT_MSK;
		st->use_internal_vref = !voltage_uv;
		break;
	case AD5683_REGMAP:
		cmd = AD5686_CMD_CONTROL_REG;
		ref_bit_msk = AD5683_REF_BIT_MSK;
		st->use_internal_vref = !voltage_uv;
		break;
	case AD5686_REGMAP:
		cmd = AD5686_CMD_INTERNAL_REFER_SETUP;
		ref_bit_msk = 0;
		break;
	case AD5693_REGMAP:
		cmd = AD5686_CMD_CONTROL_REG;
		ref_bit_msk = AD5693_REF_BIT_MSK;
		st->use_internal_vref = !voltage_uv;
		break;
	default:
		ret = -EINVAL;
		goto error_disable_reg;
	}

	val = (voltage_uv | ref_bit_msk);

	ret = st->write(st, cmd, 0, !!val);
	if (ret)
		goto error_disable_reg;

	ret = devm_iio_triggered_buffer_setup_ext(dev, indio_dev, NULL,
						  &ad5686_trigger_handler,
						  IIO_BUFFER_DIRECTION_OUT,
						  NULL, NULL);
	if (ret)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
	return ret;
}
EXPORT_SYMBOL_NS_GPL(ad5686_probe, IIO_AD5686);

void ad5686_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5686_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
}
EXPORT_SYMBOL_NS_GPL(ad5686_remove, IIO_AD5686);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5686/85/84 DAC");
MODULE_LICENSE("GPL v2");
