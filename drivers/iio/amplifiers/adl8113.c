// SPDX-License-Identifier: GPL-2.0
/*
 * ADL8113 Low Noise Amplifier with integrated bypass switches
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

enum adl8113_mode {
	ADL8113_INTERNAL_AMPLIFIER,
	ADL8113_INTERNAL_BYPASS,
	ADL8113_EXTERNAL_BYPASS_A,
	ADL8113_EXTERNAL_BYPASS_B
};

struct adl8113_state {
	struct mutex lock; /* protect sensor state */
	struct gpio_desc *gpio_va;
	struct gpio_desc *gpio_vb;
	enum adl8113_mode current_mode;
};

static const char * const adl8113_supply_names[] = {
	"vdd1",
	"vdd2",
	"vss2"
};

static const char * const adl8113_mode_names[] = {
	[ADL8113_INTERNAL_AMPLIFIER] = "internal_amplifier",
	[ADL8113_INTERNAL_BYPASS] = "internal_bypass",
	[ADL8113_EXTERNAL_BYPASS_A] = "external_bypass_a",
	[ADL8113_EXTERNAL_BYPASS_B] = "external_bypass_b",
};

static int adl8113_set_mode(struct adl8113_state *st, enum adl8113_mode mode)
{
	switch (mode) {
	case ADL8113_INTERNAL_AMPLIFIER:
		gpiod_set_value(st->gpio_va, 0);
		gpiod_set_value(st->gpio_vb, 0);
		break;
	case ADL8113_INTERNAL_BYPASS:
		gpiod_set_value(st->gpio_va, 1);
		gpiod_set_value(st->gpio_vb, 1);
		break;
	case ADL8113_EXTERNAL_BYPASS_A:
		gpiod_set_value(st->gpio_va, 0);
		gpiod_set_value(st->gpio_vb, 1);
		break;
	case ADL8113_EXTERNAL_BYPASS_B:
		gpiod_set_value(st->gpio_va, 1);
		gpiod_set_value(st->gpio_vb, 0);
		break;
	default:
		return -EINVAL;
	}

	st->current_mode = mode;
	return 0;
}

static int adl8113_get_mode(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan)
{
	struct adl8113_state *st = iio_priv(indio_dev);

	return st->current_mode;
}

static int adl8113_set_mode_enum(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct adl8113_state *st = iio_priv(indio_dev);
	int ret;

	if (mode >= ARRAY_SIZE(adl8113_mode_names))
		return -EINVAL;

	mutex_lock(&st->lock);
	ret = adl8113_set_mode(st, mode);
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_enum adl8113_mode_enum = {
	.items = adl8113_mode_names,
	.num_items = ARRAY_SIZE(adl8113_mode_names),
	.get = adl8113_get_mode,
	.set = adl8113_set_mode_enum,
};

static const struct iio_chan_spec_ext_info adl8113_ext_info[] = {
	IIO_ENUM("mode", IIO_SHARED_BY_ALL, &adl8113_mode_enum),
	IIO_ENUM_AVAILABLE("mode", IIO_SHARED_BY_ALL, &adl8113_mode_enum),
	{ },
};

static const struct iio_chan_spec adl8113_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.indexed = 1,
		.channel = 0,
		.ext_info = adl8113_ext_info,
	},
};

static int adl8113_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adl8113_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		mutex_lock(&st->lock);
		switch (st->current_mode) {
		case ADL8113_INTERNAL_AMPLIFIER:
			*val = 14;
			*val2 = 0;
			ret = IIO_VAL_INT_PLUS_MICRO_DB;
			break;
		case ADL8113_INTERNAL_BYPASS:
		case ADL8113_EXTERNAL_BYPASS_A:
		case ADL8113_EXTERNAL_BYPASS_B:
			*val = 0;
			*val2 = 0;
			ret = IIO_VAL_INT_PLUS_MICRO_DB;
			break;
		default:
			ret = -EINVAL;
		}
		mutex_unlock(&st->lock);
		return ret;
	default:
		return -EINVAL;
	}
}

static const struct iio_info adl8113_info = {
	.read_raw = adl8113_read_raw,
};

static int adl8113_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adl8113_state *st;
	struct iio_dev *indio_dev;
	u32 initial_mode = ADL8113_INTERNAL_AMPLIFIER;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	st->gpio_va = devm_gpiod_get(dev, "va", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_va))
		return dev_err_probe(dev, PTR_ERR(st->gpio_va),
				     "failed to get VA GPIO\n");

	st->gpio_vb = devm_gpiod_get(dev, "vb", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_vb))
		return dev_err_probe(dev, PTR_ERR(st->gpio_vb),
				     "failed to get VB GPIO\n");

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(adl8113_supply_names),
					     adl8113_supply_names);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get and enable supplies\n");

	device_property_read_u32(dev, "adi,initial-mode", &initial_mode);
	if (initial_mode >= ARRAY_SIZE(adl8113_mode_names))
		return -EINVAL;

	ret = adl8113_set_mode(st, initial_mode);
	if (ret)
		return ret;

	indio_dev->info = &adl8113_info;
	indio_dev->name = "adl8113";
	indio_dev->channels = adl8113_channels;
	indio_dev->num_channels = ARRAY_SIZE(adl8113_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	dev_info(dev, "ADL8113 registered, initial mode: %s\n",
		 adl8113_mode_names[initial_mode]);

	return 0;
}

static const struct of_device_id adl8113_of_match[] = {
	{ .compatible = "adi,adl8113" },
	{}
};
MODULE_DEVICE_TABLE(of, adl8113_of_match);

static struct platform_driver adl8113_driver = {
	.driver = {
		.name = "adl8113",
		.of_match_table = adl8113_of_match,
	},
	.probe = adl8113_probe,
};

module_platform_driver(adl8113_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADL8113 Low Noise Amplifier");
MODULE_LICENSE("GPL");
