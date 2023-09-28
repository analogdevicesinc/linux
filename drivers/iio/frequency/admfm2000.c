// SPDX-License-Identifier: GPL-2.0
/*
 * ADMFM2000 Dual Microwave Down Converter
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>

struct admfm2000_chip_info {
	const char			*name;
	const struct iio_chan_spec	*channels;
	unsigned int			num_channels;
	unsigned int			mode_gpios;
	unsigned int			dsa_gpios;
	int				gain_min;
	int				gain_max;
	int				default_gain;
};

struct admfm2000_state {
	struct regulator		*reg;
	struct mutex			lock; /* protect sensor state */
	struct admfm2000_chip_info	*chip_info;
	struct gpio_descs		*sw_ch[2];
	struct gpio_descs		*dsa_gpios[2];
	u32				gain[2];
};

static int admfm2000_mode(struct iio_dev *indio_dev, u32 reg, u32 mode)
{
	struct admfm2000_state *st = iio_priv(indio_dev);
	DECLARE_BITMAP(values, 2);

	switch (mode) {
	case 0:
		values[0] = (reg == 0) ? 1 : 2;
		gpiod_set_array_value_cansleep(st->sw_ch[reg]->ndescs,
					       st->sw_ch[reg]->desc,
					       NULL, values);
		break;
	case 1:
		values[0] = (reg == 0) ? 2 : 1;
		gpiod_set_array_value_cansleep(st->sw_ch[reg]->ndescs,
					       st->sw_ch[reg]->desc,
					       NULL, values);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int admfm2000_attenuation(struct iio_dev *indio_dev, u32 chan,
				 u32 value)
{
	struct admfm2000_state *st = iio_priv(indio_dev);
	DECLARE_BITMAP(values, BITS_PER_TYPE(value));

	values[0] = value;

	gpiod_set_array_value_cansleep(st->dsa_gpios[chan]->ndescs,
				       st->dsa_gpios[chan]->desc,
				       NULL, values);
	return 0;
}

static int admfm2000_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int *val,
			      int *val2, long mask)
{
	struct admfm2000_state *st = iio_priv(indio_dev);
	int gain, ret;

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		gain = ~(st->gain[chan->channel]) * -1000;
		*val = gain / 1000;
		*val2 = (gain % 1000) * 1000;

		ret =  IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
};

static int admfm2000_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct admfm2000_state *st = iio_priv(indio_dev);
	struct admfm2000_chip_info *info = st->chip_info;
	int gain, ret;

	if (val < 0)
		gain = (val * 1000) - (val2 / 1000);
	else
		gain = (val * 1000) + (val2 / 1000);

	if (gain > info->gain_max || gain < info->gain_min)
		return -EINVAL;

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		st->gain[chan->channel] = ~((abs(gain) / 1000) & 0x1F);

		ret = admfm2000_attenuation(indio_dev, chan->channel,
					    st->gain[chan->channel]);
		if (ret)
			return ret;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int admfm2000_write_raw_get_fmt(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	default:
		return -EINVAL;
	}
}

static const struct iio_info admfm2000_info = {
	.read_raw = &admfm2000_read_raw,
	.write_raw = &admfm2000_write_raw,
	.write_raw_get_fmt = &admfm2000_write_raw_get_fmt,
};

#define ADMFM2000_CHAN(_channel) {					\
	.type = IIO_VOLTAGE,						\
	.output = 1,							\
	.indexed = 1,							\
	.channel = _channel,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
}

static const struct iio_chan_spec admfm2000_channels[] = {
	ADMFM2000_CHAN(0),
	ADMFM2000_CHAN(1),
};

static int admfm2000_channel_config(struct admfm2000_state *st,
				    struct iio_dev *indio_dev)
{
	struct platform_device *pdev = to_platform_device(indio_dev->dev.parent);
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	u32 reg, mode;
	int ret;

	device_for_each_child_node(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, ret,
					     "Failed to get reg property\n");
		}

		if (reg >= indio_dev->num_channels) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL, "reg bigger than: %d\n",
					     indio_dev->num_channels);
		}

		mode = fwnode_property_read_bool(child, "adi,direct-if-mode");
		ret = admfm2000_mode(indio_dev, reg, mode);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}
	}

	return 0;
}

static void admfm2000_reg_disable(void *data)
{
	struct admfm2000_state *st = data;

	regulator_disable(st->reg);
}

static struct admfm2000_chip_info admfm2000_chip_info_tbl = {
	.name = "admfm2000",
	.channels = admfm2000_channels,
	.num_channels = ARRAY_SIZE(admfm2000_channels),
	.dsa_gpios = 5,
	.mode_gpios = 2,
	.gain_min = -31000,
	.gain_max = 0,
	.default_gain = -0x20, /* set default gain -31db */
};

static int admfm2000_setup(struct admfm2000_state *st,
			   struct iio_dev *indio_dev)
{
	struct platform_device *pdev = to_platform_device(indio_dev->dev.parent);
	struct device *dev = &pdev->dev;

	st->sw_ch[0] = devm_gpiod_get_array(dev, "switch1", GPIOD_OUT_LOW);
	if (IS_ERR(st->sw_ch[0]))
		return dev_err_probe(dev, PTR_ERR(st->sw_ch[0]),
				     "Failed to get gpios\n");

	if (st->sw_ch[0]->ndescs != st->chip_info->mode_gpios) {
		dev_err(dev, "%d GPIOs needed to operate\n",
			st->chip_info->mode_gpios);
		return -ENODEV;
	}

	st->sw_ch[1] = devm_gpiod_get_array(dev, "switch2", GPIOD_OUT_LOW);
	if (IS_ERR(st->sw_ch[1]))
		return dev_err_probe(dev, PTR_ERR(st->sw_ch[1]),
				     "Failed to get gpios\n");

	if (st->sw_ch[1]->ndescs != st->chip_info->mode_gpios) {
		dev_err(dev, "%d GPIOs needed to operate\n",
			st->chip_info->mode_gpios);
		return -ENODEV;
	}

	st->dsa_gpios[0] = devm_gpiod_get_array(dev, "attenuation1",
						GPIOD_OUT_LOW);
	if (IS_ERR(st->dsa_gpios[0]))
		return dev_err_probe(dev, PTR_ERR(st->dsa_gpios[0]),
				     "Failed to get gpios\n");

	if (st->dsa_gpios[0]->ndescs != st->chip_info->dsa_gpios) {
		dev_err(dev, "%d GPIOs needed to operate\n",
			st->chip_info->dsa_gpios);
		return -ENODEV;
	}

	st->dsa_gpios[1] = devm_gpiod_get_array(dev, "attenuation2",
						GPIOD_OUT_LOW);
	if (IS_ERR(st->dsa_gpios[1]))
		return dev_err_probe(dev, PTR_ERR(st->dsa_gpios[1]),
				     "Failed to get gpios\n");

	if (st->dsa_gpios[1]->ndescs != st->chip_info->dsa_gpios) {
		dev_err(dev, "%d GPIOs needed to operate\n",
			st->chip_info->dsa_gpios);
		return -ENODEV;
	}

	return 0;
}

static int admfm2000_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct admfm2000_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->chip_info = &admfm2000_chip_info_tbl;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->name = st->chip_info->name;
	indio_dev->info = &admfm2000_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	st->gain[0] = st->chip_info->default_gain;
	st->gain[1] = st->chip_info->default_gain;

	st->reg = devm_regulator_get(dev, "vcc-supply");
	if (IS_ERR(st->reg))
		return PTR_ERR(st->reg);

	ret = regulator_enable(st->reg);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, admfm2000_reg_disable, st);
	if (ret)
		return ret;

	mutex_init(&st->lock);

	ret = admfm2000_setup(st, indio_dev);
	if (ret)
		return ret;

	ret = admfm2000_channel_config(st, indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id admfm2000_of_match[] = {
	{ .compatible = "adi,admfm2000" },
	{ }
};
MODULE_DEVICE_TABLE(of, admfm2000_of_match);

static struct platform_driver admfm2000_driver = {
	.driver = {
		.name = "admfm2000",
		.of_match_table = admfm2000_of_match,
	},
	.probe = admfm2000_probe,
};
module_platform_driver(admfm2000_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("ADMFM2000 Dual Microwave Down Converter");
MODULE_LICENSE("GPL v2");
