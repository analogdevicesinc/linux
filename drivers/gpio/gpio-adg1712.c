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
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define ADG1712_NUM_GPIOS	4

static const char * const adg1712_switch_names[ADG1712_NUM_GPIOS] = {
	"switch0", "switch1", "switch2", "switch3"
};

struct adg1712 {
	struct gpio_chip chip;
	struct gpio_desc *switch_gpios[ADG1712_NUM_GPIOS];
};

static int adg1712_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int adg1712_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	/* Switches cannot be set like regular GPIOs - use .set_config() */
	return -EOPNOTSUPP;
}

static int adg1712_get(struct gpio_chip *chip, unsigned int offset)
{
	/* Switches cannot be read like regular GPIOs */
	return -EOPNOTSUPP;
}

static int adg1712_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				unsigned long *bits)
{
	/* Switches cannot be set like regular GPIOs - use .set_config() */
	return -EOPNOTSUPP;
}

static int adg1712_init_valid_mask(struct gpio_chip *chip,
				   unsigned long *valid_mask,
				   unsigned int ngpios)
{
	struct adg1712 *adg1712 = gpiochip_get_data(chip);
	int i;

	/* Clear all bits first */
	bitmap_zero(valid_mask, ngpios);

	/* Set bits for connected switches only */
	for (i = 0; i < ADG1712_NUM_GPIOS; i++) {
		if (adg1712->switch_gpios[i])
			set_bit(i, valid_mask);
	}

	return 0;
}

static int adg1712_set_config(struct gpio_chip *chip, unsigned int offset,
			      unsigned long config)
{
	struct adg1712 *adg1712 = gpiochip_get_data(chip);
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);

	switch (param) {
	case PIN_CONFIG_OUTPUT_ENABLE:
		return gpiod_set_value_cansleep(adg1712->switch_gpios[offset],
						arg ? 1 : 0);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct gpio_chip adg1712_gpio_chip = {
	.label			= "adg1712",
	.owner			= THIS_MODULE,
	.get_direction		= adg1712_get_direction,
	.get			= adg1712_get,
	.set			= adg1712_set,
	.set_multiple		= adg1712_set_multiple,
	.set_config		= adg1712_set_config,
	.init_valid_mask	= adg1712_init_valid_mask,
	.base			= -1,
	.ngpio			= ADG1712_NUM_GPIOS,
	.can_sleep		= true,
};

static int adg1712_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adg1712 *adg1712;
	int ret;

	adg1712 = devm_kzalloc(dev, sizeof(*adg1712), GFP_KERNEL);
	if (!adg1712)
		return -ENOMEM;

	adg1712->chip = adg1712_gpio_chip;
	adg1712->chip.parent = dev;

	for (int i = 0; i < ADG1712_NUM_GPIOS; i++) {
		adg1712->switch_gpios[i] = devm_gpiod_get_optional(dev, adg1712_switch_names[i],
								   GPIOD_ASIS);
		if (IS_ERR(adg1712->switch_gpios[i]))
			return dev_err_probe(dev,
					     PTR_ERR(adg1712->switch_gpios[i]),
					     "failed to get %s gpio\n",
					     adg1712_switch_names[i]);
	}

	ret = devm_gpiochip_add_data(dev, &adg1712->chip, adg1712);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add gpio chip\n");

	dev_dbg(dev, "ADG1712 %u-GPIO expander registered\n",
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
