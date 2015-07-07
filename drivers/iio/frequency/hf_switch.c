/*
 * HMC922 SPI SWITCH driver
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


/**
 * struct hmc922_state - driver instance specific data
 * @pdev:		pdev_device
 * @chip_info:		chip model specific constants, available modes etc
 * @reg:		supply regulator
 * @vref_mv:		actual reference voltage used
 */

struct hmc922_state {
	struct regulator		*reg;
	struct gpio_desc		*sel_a_gpio;
	struct gpio_desc		*sel_b_gpio;
	u32			select;
	u32			id;

};

enum hmc922_type {
	ID_HMC922,
	ID_HMC349,
};


static const char * const hmc922_select_modes[] = {
	"RF1", "RF2"
};

static int hmc922_set_select_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct hmc922_state *st = iio_priv(indio_dev);

	st->select = mode + 1;

	switch (st->id) {
	case ID_HMC922:
		gpiod_set_value(st->sel_a_gpio, st->select == 1);
		gpiod_set_value(st->sel_b_gpio, st->select != 1);
	case ID_HMC349:
		gpiod_set_value(st->sel_a_gpio, st->select == 1);
		break;
	}

	return 0;
}

static int hmc922_get_select_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct hmc922_state *st = iio_priv(indio_dev);

	return st->select - 1;
}

static const struct iio_enum hmc922_select_mode_enum = {
	.items = hmc922_select_modes,
	.num_items = ARRAY_SIZE(hmc922_select_modes),
	.get = hmc922_get_select_mode,
	.set = hmc922_set_select_mode,
};

static const struct iio_chan_spec_ext_info hmc922_ext_info_select[] = {
	IIO_ENUM("input_select", IIO_SEPARATE, &hmc922_select_mode_enum),
	IIO_ENUM_AVAILABLE("input_select", &hmc922_select_mode_enum),
	{ },
};

static int hmc922_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	return 0;
};

static int hmc922_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{

	return 0;
}

static const struct iio_info hmc922_info = {
 	.read_raw = &hmc922_read_raw,
 	.write_raw = &hmc922_write_raw,
	.driver_module = THIS_MODULE,
};

static const struct iio_chan_spec hmc922_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.output = 1,
		.indexed = 1,
		.channel = 0,
		.ext_info = hmc922_ext_info_select,
	},
};

/* Match table for of_platform binding */
static const struct of_device_id hmc922_of_match[] = {
	{ .compatible = "adi,hmc922", .data = (void*) ID_HMC922},
	{ .compatible = "adi,hmc349", .data = (void*) ID_HMC349},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, hmc922_of_match);



static int hmc922_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct hmc922_state *st;
	struct iio_dev *indio_dev;
	struct regulator *reg;
	int ret;

	const struct of_device_id *of_id =
			of_match_device(hmc922_of_match, &pdev->dev);

	reg = devm_regulator_get(&pdev->dev, "vcc");
	if (!IS_ERR(reg)) {
		ret = regulator_enable(reg);
		if (ret)
			return ret;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_disable_reg;
	}
	st = iio_priv(indio_dev);

	platform_set_drvdata(pdev, indio_dev);
	st->reg = reg;
	st->id = (u32) of_id->data;

	st->select = 1;
	of_property_read_u32(np, "adi,rf-input-select", &st->select);

	switch (st->id) {
	case ID_HMC922:
		st->sel_a_gpio = devm_gpiod_get(&pdev->dev, "select_a");
		if (!IS_ERR(st->sel_a_gpio))
			gpiod_direction_output(st->sel_a_gpio, st->select == 1);
		else
			goto error_disable_reg;

		st->sel_b_gpio = devm_gpiod_get(&pdev->dev, "select_b");
		if (!IS_ERR(st->sel_b_gpio))
			gpiod_direction_output(st->sel_b_gpio, st->select != 1);
		else
			goto error_disable_reg;
		break;

	case ID_HMC349:
		st->sel_a_gpio = devm_gpiod_get(&pdev->dev, "select");
		if (!IS_ERR(st->sel_a_gpio))
			gpiod_direction_output(st->sel_a_gpio, st->select == 1);
		else
			goto error_disable_reg;

		st->sel_b_gpio = devm_gpiod_get(&pdev->dev, "enable");
		if (!IS_ERR(st->sel_b_gpio))
			gpiod_direction_output(st->sel_b_gpio, 0);

		break;
	default:
		ret = -ENODEV;
		goto error_disable_reg;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->info = &hmc922_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = hmc922_channels;
	indio_dev->num_channels = 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(reg))
		regulator_disable(reg);
	return ret;
}

static int hmc922_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct hmc922_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return 0;
}

static struct platform_driver hmc922_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = hmc922_of_match,
	},
	.probe		= hmc922_probe,
	.remove		= hmc922_remove,
};

module_platform_driver(hmc922_of_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices HMC922 SWITCH");
MODULE_LICENSE("GPL v2");
