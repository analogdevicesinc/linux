// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7768 GPIO auxiliary driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/auxiliary_bus.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/regmap.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#define AD7768_REG_GPIO_CONTROL		0x0E
#define AD7768_REG_GPIO_WRITE		0x0F
#define AD7768_REG_GPIO_READ		0x10

#define AD7768_GPIO_UGPIO_ENABLE	BIT(7)

#define AD7768_NUM_GPIOS		5

static int ad7768_gpio_reg_mask_xlate(struct gpio_regmap *gpio,
				      unsigned int base, unsigned int offset,
				      unsigned int *reg, unsigned int *mask)
{
	struct regmap *regmap = gpio_regmap_get_drvdata(gpio);
	int ret;

	*reg = base;
	*mask = BIT(offset);

	if (base != AD7768_REG_GPIO_READ)
		return 0;

	/*
	 * AD7768 has separate input-state and output-latch registers. For an
	 * output line, report the programmed value from the output latch;
	 * input lines continue to use the input-state register.
	 */
	ret = regmap_test_bits(regmap, AD7768_REG_GPIO_CONTROL, *mask);
	if (ret < 0)
		return ret;
	if (ret)
		*reg = AD7768_REG_GPIO_WRITE;

	return 0;
}

static int ad7768_gpio_probe(struct auxiliary_device *adev,
			     const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct gpio_regmap_config config = {
		.parent = dev,
		.label = dev_name(dev->parent),
		.ngpio = AD7768_NUM_GPIOS,
		.reg_dat_base = AD7768_REG_GPIO_READ,
		.reg_set_base = AD7768_REG_GPIO_WRITE,
		.reg_dir_out_base = AD7768_REG_GPIO_CONTROL,
		.pm_dev = dev->parent,
		.reg_mask_xlate = ad7768_gpio_reg_mask_xlate,
	};
	struct gpio_regmap *gpio;
	struct regmap *regmap;
	int ret;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return -ENODEV;

	PM_RUNTIME_ACQUIRE_IF_ENABLED_AUTOSUSPEND(dev->parent, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	ret = regmap_set_bits(regmap, AD7768_REG_GPIO_CONTROL,
			      AD7768_GPIO_UGPIO_ENABLE);
	if (ret)
		return ret;

	config.regmap = regmap;
	config.drvdata = regmap;
	gpio = devm_gpio_regmap_register(dev, &config);

	return PTR_ERR_OR_ZERO(gpio);
}

static const struct auxiliary_device_id ad7768_gpio_ids[] = {
	{ .name = "ad7768.gpio" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, ad7768_gpio_ids);

static struct auxiliary_driver ad7768_gpio_driver = {
	.probe = ad7768_gpio_probe,
	.id_table = ad7768_gpio_ids,
};
module_auxiliary_driver(ad7768_gpio_driver);

MODULE_AUTHOR("Janani Sunil <janani.sunil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 GPIO auxiliary driver");
MODULE_LICENSE("GPL");
