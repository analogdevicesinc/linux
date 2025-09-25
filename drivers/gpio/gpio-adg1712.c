// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADG1712 quad SPST switch GPIO driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define ADG1712_NUM_GPIOS	4

struct adg1712 {
	struct gpio_chip chip;
	struct gpio_desc *switch_gpios[ADG1712_NUM_GPIOS];
};

static int adg1712_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int adg1712_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return -EINVAL;
}

static int adg1712_direction_output(struct gpio_chip *chip, unsigned int offset,
				    int value)
{
	struct adg1712 *adg1712 = gpiochip_get_data(chip);

	if (offset >= ADG1712_NUM_GPIOS)
		return -EINVAL;

	gpiod_set_value_cansleep(adg1712->switch_gpios[offset], value);
	return 0;
}

static void adg1712_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct adg1712 *adg1712 = gpiochip_get_data(chip);

	if (offset >= ADG1712_NUM_GPIOS)
		return;

	gpiod_set_value_cansleep(adg1712->switch_gpios[offset], value);
}

static int adg1712_get(struct gpio_chip *chip, unsigned int offset)
{
	struct adg1712 *adg1712 = gpiochip_get_data(chip);

	if (offset >= ADG1712_NUM_GPIOS)
		return -EINVAL;

	return gpiod_get_value_cansleep(adg1712->switch_gpios[offset]);
}

static void adg1712_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				 unsigned long *bits)
{
	struct adg1712 *adg1712 = gpiochip_get_data(chip);
	int i;

	for_each_set_bit(i, mask, ADG1712_NUM_GPIOS) {
		gpiod_set_value_cansleep(adg1712->switch_gpios[i],
					 test_bit(i, bits));
	}
}

static const struct gpio_chip adg1712_gpio_chip = {
	.label			= "adg1712",
	.owner			= THIS_MODULE,
	.get_direction		= adg1712_get_direction,
	.direction_input	= adg1712_direction_input,
	.direction_output	= adg1712_direction_output,
	.get			= adg1712_get,
	.set			= adg1712_set,
	.set_multiple		= adg1712_set_multiple,
	.base			= -1,
	.ngpio			= ADG1712_NUM_GPIOS,
	.can_sleep		= true,
};

static int adg1712_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adg1712 *adg1712;
	int ret, i;
	char gpio_name[16];

	adg1712 = devm_kzalloc(dev, sizeof(*adg1712), GFP_KERNEL);
	if (!adg1712)
		return -ENOMEM;

	adg1712->chip = adg1712_gpio_chip;
	adg1712->chip.parent = dev;

	for (i = 0; i < ADG1712_NUM_GPIOS; i++) {
		snprintf(gpio_name, sizeof(gpio_name), "switch%d", i + 1);
		adg1712->switch_gpios[i] = devm_gpiod_get(dev, gpio_name,
							  GPIOD_OUT_LOW);
		if (IS_ERR(adg1712->switch_gpios[i]))
			return dev_err_probe(dev, PTR_ERR(adg1712->switch_gpios[i]),
					     "failed to get %s gpio\n", gpio_name);
	}

	ret = devm_gpiochip_add_data(dev, &adg1712->chip, adg1712);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add gpio chip\n");

	dev_info(dev, "ADG1712 %u-GPIO expander registered\n",
		 adg1712->chip.ngpio);

	return 0;
}

static const struct of_device_id adg1712_dt_ids[] = {
	{ .compatible = "adi,adg1712", },
	{ }
};
MODULE_DEVICE_TABLE(of, adg1712_dt_ids);

static struct platform_driver adg1712_driver = {
	.driver = {
		.name = "adg1712",
		.of_match_table = adg1712_dt_ids,
	},
	.probe = adg1712_probe,
};
module_platform_driver(adg1712_driver);

MODULE_DESCRIPTION("Analog Devices ADG1712 quad SPST switch GPIO driver");
MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_LICENSE("GPL");