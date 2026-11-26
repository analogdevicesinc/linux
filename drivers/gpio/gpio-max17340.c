// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices MAX17340 GPIO Driver
 *
 * Copyright (C) 2024-2026 Analog Devices, Inc.
 */

#include <linux/gpio/driver.h>
#include <linux/mfd/max17340.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct max17340_gpio {
	struct gpio_chip gc;
	struct max17340_dev *mfd;
};

static int max17340_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct max17340_gpio *gpio = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(gpio->mfd->regmap_func, MAX17340_GPIO_MODE, &val);
	if (ret)
		return ret;

	val = (val >> (offset * 2)) & MAX17340_GPIO_MODE_MASK;

	switch (val) {
	case MAX17340_GPIO_MODE_INPUT:
		return GPIO_LINE_DIRECTION_IN;
	case MAX17340_GPIO_MODE_DEDICATED:
		/* AGPIO1/2/3 (TH1,TH2,PB) = input, AGPIO4 (ALRT) = output */
		return (offset < 3) ? GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
	default:
		return GPIO_LINE_DIRECTION_OUT;
	}
}

static int max17340_gpio_direction_input(struct gpio_chip *gc,
					 unsigned int offset)
{
	struct max17340_gpio *gpio = gpiochip_get_data(gc);

	/* GPIO4 (ALRT) doesn't support input mode */
	if (offset == 3)
		return -ENOTSUPP;

	return regmap_update_bits(gpio->mfd->regmap_func, MAX17340_GPIO_MODE,
				  MAX17340_GPIO_MODE_MASK << (offset * 2),
				  MAX17340_GPIO_MODE_INPUT << (offset * 2));
}

static int max17340_gpio_direction_output(struct gpio_chip *gc,
					  unsigned int offset, int value)
{
	struct max17340_gpio *gpio = gpiochip_get_data(gc);
	u8 mode;
	int ret;

	ret = regmap_update_bits(gpio->mfd->regmap_func, MAX17340_GPIO_DATA,
				 BIT(offset), value ? BIT(offset) : 0);
	if (ret)
		return ret;

	/* GPIO4 (ALRT) only supports open-drain output */
	mode = (offset == 3) ? MAX17340_GPIO_MODE_OPENDRAIN :
			       MAX17340_GPIO_MODE_PUSHPULL;

	return regmap_update_bits(gpio->mfd->regmap_func, MAX17340_GPIO_MODE,
				  MAX17340_GPIO_MODE_MASK << (offset * 2),
				  mode << (offset * 2));
}

static int max17340_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct max17340_gpio *gpio = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(gpio->mfd->regmap_func, MAX17340_GPIO_STS, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset));
}

static void max17340_gpio_set(struct gpio_chip *gc, unsigned int offset,
			      int value)
{
	struct max17340_gpio *gpio = gpiochip_get_data(gc);

	regmap_update_bits(gpio->mfd->regmap_func, MAX17340_GPIO_DATA,
			   BIT(offset), value ? BIT(offset) : 0);
}

static int max17340_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				    unsigned long config)
{
	struct max17340_gpio *gpio = gpiochip_get_data(gc);
	enum pin_config_param param = pinconf_to_config_param(config);
	u8 mode;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		mode = MAX17340_GPIO_MODE_OPENDRAIN;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		/* GPIO4 (ALRT) doesn't support push-pull */
		if (offset == 3)
			return -ENOTSUPP;
		mode = MAX17340_GPIO_MODE_PUSHPULL;
		break;
	default:
		return -ENOTSUPP;
	}

	return regmap_update_bits(gpio->mfd->regmap_func, MAX17340_GPIO_MODE,
				  MAX17340_GPIO_MODE_MASK << (offset * 2),
				  mode << (offset * 2));
}

static int max17340_gpio_set_mode(struct max17340_gpio *gpio,
				  unsigned int offset, u8 mode)
{
	if (offset >= MAX17340_NUM_GPIOS || mode > MAX17340_GPIO_MODE_OPENDRAIN)
		return -EINVAL;

	/* GPIO4 only supports dedicated (0) or open-drain (3) */
	if (offset == 3 && mode != MAX17340_GPIO_MODE_DEDICATED &&
	    mode != MAX17340_GPIO_MODE_OPENDRAIN)
		return -EINVAL;

	return regmap_update_bits(gpio->mfd->regmap_func, MAX17340_GPIO_MODE,
				  MAX17340_GPIO_MODE_MASK << (offset * 2),
				  mode << (offset * 2));
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct max17340_gpio *gpio = dev_get_drvdata(dev);
	unsigned int val;
	int ret, i, len = 0;
	static const char * const mode_names[] = {
		"dedicated", "input", "push-pull", "open-drain"
	};
	static const char * const gpio_names[] = {
		"TH1/AGPIO1", "TH2/AGPIO2", "PB/AGPIO3", "ALRT/AGPIO4"
	};

	ret = regmap_read(gpio->mfd->regmap_func, MAX17340_GPIO_MODE, &val);
	if (ret)
		return ret;

	for (i = 0; i < MAX17340_NUM_GPIOS; i++) {
		u8 mode = (val >> (i * 2)) & MAX17340_GPIO_MODE_MASK;

		len += sysfs_emit_at(buf, len, "GPIO%d (%s): %d (%s)\n",
				     i + 1, gpio_names[i], mode, mode_names[mode]);
	}

	return len;
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct max17340_gpio *gpio = dev_get_drvdata(dev);
	unsigned int gpio_num, mode;
	int ret;

	/* GPIO number is 1-based (GPIO1~4) */
	if (sscanf(buf, "%u %u", &gpio_num, &mode) != 2)
		return -EINVAL;

	if (gpio_num < 1 || gpio_num > MAX17340_NUM_GPIOS)
		return -EINVAL;

	ret = max17340_gpio_set_mode(gpio, gpio_num - 1, mode);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(mode);

static int max17340_gpio_probe(struct platform_device *pdev)
{
	struct max17340_dev *mfd = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct max17340_gpio *gpio;
	int ret;

	if (!mfd)
		return -ENODEV;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->mfd = mfd;

	gpio->gc.label = "max17340-gpio";
	gpio->gc.parent = dev;
	gpio->gc.owner = THIS_MODULE;
	gpio->gc.base = -1;
	gpio->gc.ngpio = MAX17340_NUM_GPIOS;
	gpio->gc.can_sleep = true;
	gpio->gc.get_direction = max17340_gpio_get_direction;
	gpio->gc.direction_input = max17340_gpio_direction_input;
	gpio->gc.direction_output = max17340_gpio_direction_output;
	gpio->gc.get = max17340_gpio_get;
	gpio->gc.set = max17340_gpio_set;
	gpio->gc.set_config = max17340_gpio_set_config;

	ret = devm_gpiochip_add_data(dev, &gpio->gc, gpio);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register GPIO chip\n");

	dev_set_drvdata(dev, gpio);

	ret = device_create_file(dev, &dev_attr_mode);
	if (ret)
		dev_warn(dev, "Failed to create mode sysfs attribute\n");

	return 0;
}

static const struct platform_device_id max17340_gpio_id[] = {
	{ "max17340-gpio", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max17340_gpio_id);

static struct platform_driver max17340_gpio_driver = {
	.driver = {
		.name = "max17340-gpio",
	},
	.probe = max17340_gpio_probe,
	.id_table = max17340_gpio_id,
};
module_platform_driver(max17340_gpio_driver);

MODULE_DESCRIPTION("Analog Devices MAX17340 GPIO Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joan Na <joan.na@analog.com>");
