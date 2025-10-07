// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADG1736 dual SPDT switch GPIO driver
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

#define ADG1736_NUM_GPIOS	2

struct adg1736 {
	struct gpio_chip chip;
	struct gpio_desc *switch_gpios[ADG1736_NUM_GPIOS];
};

static int adg1736_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int adg1736_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return -EINVAL;
}

static int adg1736_direction_output(struct gpio_chip *chip, unsigned int offset,
				    int value)
{
	struct adg1736 *adg1736 = gpiochip_get_data(chip);

	if (offset >= ADG1736_NUM_GPIOS)
		return -EINVAL;

	gpiod_set_value_cansleep(adg1736->switch_gpios[offset], value);
	return 0;
}

static void adg1736_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct adg1736 *adg1736 = gpiochip_get_data(chip);

	if (offset >= ADG1736_NUM_GPIOS)
		return;

	gpiod_set_value_cansleep(adg1736->switch_gpios[offset], value);
}

static int adg1736_get(struct gpio_chip *chip, unsigned int offset)
{
	struct adg1736 *adg1736 = gpiochip_get_data(chip);

	if (offset >= ADG1736_NUM_GPIOS)
		return -EINVAL;

	return gpiod_get_value_cansleep(adg1736->switch_gpios[offset]);
}

static void adg1736_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				 unsigned long *bits)
{
	struct adg1736 *adg1736 = gpiochip_get_data(chip);
	int i;

	for_each_set_bit(i, mask, ADG1736_NUM_GPIOS) {
		gpiod_set_value_cansleep(adg1736->switch_gpios[i],
					 test_bit(i, bits));
	}
}

static const struct gpio_chip adg1736_gpio_chip = {
	.label			= "adg1736",
	.owner			= THIS_MODULE,
	.get_direction		= adg1736_get_direction,
	.direction_input	= adg1736_direction_input,
	.direction_output	= adg1736_direction_output,
	.get			= adg1736_get,
	.set			= adg1736_set,
	.set_multiple		= adg1736_set_multiple,
	.base			= -1,
	.ngpio			= ADG1736_NUM_GPIOS,
	.can_sleep		= true,
};

static int adg1736_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adg1736 *adg1736;
	int ret, i;
	char gpio_name[16];

	adg1736 = devm_kzalloc(dev, sizeof(*adg1736), GFP_KERNEL);
	if (!adg1736)
		return -ENOMEM;

	adg1736->chip = adg1736_gpio_chip;
	adg1736->chip.parent = dev;

	for (i = 0; i < ADG1736_NUM_GPIOS; i++) {
		snprintf(gpio_name, sizeof(gpio_name), "switch%d", i + 1);
		adg1736->switch_gpios[i] = devm_gpiod_get(dev, gpio_name,
							  GPIOD_OUT_LOW);
		if (IS_ERR(adg1736->switch_gpios[i]))
			return dev_err_probe(dev, PTR_ERR(adg1736->switch_gpios[i]),
					     "failed to get %s gpio\n", gpio_name);
	}

	ret = devm_gpiochip_add_data(dev, &adg1736->chip, adg1736);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add gpio chip\n");

	dev_info(dev, "ADG1736 %u-GPIO expander registered\n",
		 adg1736->chip.ngpio);

	return 0;
}

static const struct of_device_id adg1736_dt_ids[] = {
	{ .compatible = "adi,adg1736", },
	{ }
};
MODULE_DEVICE_TABLE(of, adg1736_dt_ids);

static struct platform_driver adg1736_driver = {
	.driver = {
		.name = "adg1736",
		.of_match_table = adg1736_dt_ids,
	},
	.probe = adg1736_probe,
};
module_platform_driver(adg1736_driver);

MODULE_DESCRIPTION("Analog Devices ADG1736 dual SPDT switch GPIO driver");
MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_LICENSE("GPL");