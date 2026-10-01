// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * GPIO driver for Analog Devices MAX20355/MAX20357 PMICs
 */

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "../mfd/maxim/max2035x/max2035x.h"

#define MAX2035X_REG_GPIO1		0x58
#define MAX2035X_REG_GPIO_RDB1		0x5C

#define MAX2035X_GPIO_ENRES_BIT		BIT(2)
#define MAX2035X_GPIO_DOUT_BIT		BIT(0)

#define MAX2035X_GPIO_NGPIO		4
#define MAX2035X_GPIO_RDB1_INP_SHIFT	4

struct max2035x_gpio {
	struct gpio_chip gc;
	struct regmap *regmap;
};

static int max2035x_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct max2035x_gpio *gpio = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(gpio->regmap, MAX2035X_REG_GPIO1 + offset, &val);
	if (ret)
		return ret;

	return (val & MAX2035X_GPIO_ENRES_BIT) ?
		GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int max2035x_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct max2035x_gpio *gpio = gpiochip_get_data(gc);

	return regmap_update_bits(gpio->regmap, MAX2035X_REG_GPIO1 + offset,
				  MAX2035X_GPIO_ENRES_BIT, MAX2035X_GPIO_ENRES_BIT);
}

static int max2035x_gpio_direction_output(struct gpio_chip *gc,
					   unsigned int offset, int value)
{
	struct max2035x_gpio *gpio = gpiochip_get_data(gc);
	u8 val = value ? 0 : MAX2035X_GPIO_DOUT_BIT;

	return regmap_update_bits(gpio->regmap, MAX2035X_REG_GPIO1 + offset,
				  MAX2035X_GPIO_ENRES_BIT | MAX2035X_GPIO_DOUT_BIT,
				  val);
}

static int max2035x_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct max2035x_gpio *gpio = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(gpio->regmap, MAX2035X_REG_GPIO_RDB1, &val);
	if (ret)
		return ret;

	return !!(val & BIT(MAX2035X_GPIO_RDB1_INP_SHIFT + offset));
}

static int max2035x_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct max2035x_gpio *gpio = gpiochip_get_data(gc);
	u8 val = value ? 0 : MAX2035X_GPIO_DOUT_BIT;

	return regmap_update_bits(gpio->regmap, MAX2035X_REG_GPIO1 + offset,
				  MAX2035X_GPIO_DOUT_BIT, val);
}

static int max2035x_gpio_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct max2035x_gpio *gpio;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->regmap = chip->regmap;

	gpio->gc.base = -1;
	gpio->gc.ngpio = MAX2035X_GPIO_NGPIO;
	gpio->gc.label = dev_name(&pdev->dev);
	gpio->gc.parent = &pdev->dev;
	gpio->gc.owner = THIS_MODULE;
	gpio->gc.can_sleep = true;
	gpio->gc.get_direction = max2035x_gpio_get_direction;
	gpio->gc.direction_input = max2035x_gpio_direction_input;
	gpio->gc.direction_output = max2035x_gpio_direction_output;
	gpio->gc.get = max2035x_gpio_get;
	gpio->gc.set = max2035x_gpio_set;

	return devm_gpiochip_add_data(&pdev->dev, &gpio->gc, gpio);
}

static const struct platform_device_id max2035x_gpio_id[] = {
	{ "max20355-gpio" },
	{ "max20357-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max2035x_gpio_id);

static struct platform_driver max2035x_gpio_driver = {
	.probe = max2035x_gpio_probe,
	.driver = {
		.name = "max2035x-gpio",
	},
	.id_table = max2035x_gpio_id,
};
module_platform_driver(max2035x_gpio_driver);

MODULE_DESCRIPTION("GPIO driver for Analog Devices MAX20355/MAX20357");
MODULE_AUTHOR("Judy Na <judy.na@analog.com>");
MODULE_LICENSE("GPL");
