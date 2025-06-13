// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices LTC4283 GPIO driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/mfd/ltc4283.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define LTC4283_PGIO_CONFIG_2		0x11

#define LTC42823_ADIO_CONFIG		0x12
  #define LTC4283_ADIOX_OUT_MASK(off)	BIT((off) - LTC4283_ADIOX_START_NR)

struct ltc4283_gpio {
	struct gpio_chip gpio_chip;
	struct regmap *regmap;
};

static int ltc4283_pgio_get_direction(const struct ltc4283_gpio *st, unsigned int off)
{
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, LTC4283_PGIO_CONFIG, &val);
	if (ret)
		return ret;

	val = field_get(LTC4283_PGIO_CFG_MASK(off), val);
	if (val == 3)
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int ltc4283_gpio_get_direction(struct gpio_chip *gc, unsigned int off)
{
	struct ltc4283_gpio *st = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	if (off < LTC4283_ADIOX_START_NR)
		return ltc4283_pgio_get_direction(st, off);

	ret = regmap_read(st->regmap, LTC42823_ADIO_CONFIG, &val);
	if (ret)
		return ret;

	if (val & BIT(off))
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int ltc4283_gpio_direction_set(const struct ltc4283_gpio *st,
				      unsigned int off, bool input)
{
	if (off < LTC4283_ADIOX_START_NR) {
		unsigned int val;

		val = field_prep(LTC4283_PGIO_CFG_MASK(off), input ? 3 : 2);
		return regmap_update_bits(st->regmap, LTC4283_PGIO_CONFIG,
					  LTC4283_PGIO_CFG_MASK(off), val);
	}

	return regmap_update_bits(st->regmap, LTC42823_ADIO_CONFIG, BIT(off),
				  field_prep(BIT(off), input));
}

static int __ltc4283_gpio_set_value(const struct ltc4283_gpio *st,
				    unsigned int off, int val)
{
	if (off < LTC4283_ADIOX_START_NR) {
		val = field_prep(LTC4283_PGIO_OUT_MASK(off), val);
		return regmap_update_bits(st->regmap, LTC4283_PGIO_CONFIG_2,
					  LTC4283_PGIO_OUT_MASK(off), val);
	}

	val = field_prep(LTC4283_ADIOX_OUT_MASK(off), !!val);
	return regmap_update_bits(st->regmap, LTC42823_ADIO_CONFIG,
				  LTC4283_ADIOX_OUT_MASK(off), val);
}

static int ltc4283_gpio_direction_input(struct gpio_chip *gc, unsigned int off)
{
	struct ltc4283_gpio *st = gpiochip_get_data(gc);

	return ltc4283_gpio_direction_set(st, off, true);
}

static int ltc4283_gpio_direction_output(struct gpio_chip *gc, unsigned int off, int val)
{
	struct ltc4283_gpio *st = gpiochip_get_data(gc);
	int ret;

	ret = ltc4283_gpio_direction_set(st, off, false);
	if (ret)
		return ret;

	return __ltc4283_gpio_set_value(st, off, val);
}

static int ltc4283_gpio_get_value(struct gpio_chip *gc, unsigned int off)
{
	struct ltc4283_gpio *st = gpiochip_get_data(gc);
	unsigned int val;
	int ret;

	if (off < LTC4283_ADIOX_START_NR) {
		ret = regmap_read(st->regmap, LTC4283_PGIO_CONFIG_2, &val);
		if (ret)
			return ret;

		return field_get(LTC4283_PGIO_OUT_MASK(off), val);
	}

	ret = regmap_read(st->regmap, LTC42823_ADIO_CONFIG, &val);
	if (ret)
		return ret;

	return field_get(LTC4283_ADIOX_OUT_MASK(off), val);
}

// needs to use the new callback that returns error codes
static void ltc4283_gpio_set_value(struct gpio_chip *gc, unsigned int off, int val)
{
	struct ltc4283_gpio *st = gpiochip_get_data(gc);

	__ltc4283_gpio_set_value(st, off, val);
}

static int ltc4283_init_valid_mask(struct gpio_chip *gc, unsigned long *valid_mask,
				   unsigned int ngpios)
{
	/* get mask from top level device */
	unsigned long *mask = dev_get_drvdata(gc->parent->parent);

	bitmap_copy(valid_mask, mask, ngpios);
	return 0;
}

static int ltc4283_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ltc4283_gpio *st;
	struct gpio_chip *gc;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->regmap = dev_get_regmap(dev->parent, NULL);

	gc = &st->gpio_chip;
	gc->parent = dev;
	gc->get_direction = ltc4283_gpio_get_direction;
	gc->direction_input = ltc4283_gpio_direction_input;
	gc->direction_output = ltc4283_gpio_direction_output;
	gc->get = ltc4283_gpio_get_value;
	gc->set = ltc4283_gpio_set_value;
	gc->init_valid_mask = ltc4283_init_valid_mask;
	gc->can_sleep = true;

	gc->base = -1;
	gc->ngpio = LTC4283_GPIO_MAX;
	gc->label = pdev->name;
	gc->owner = THIS_MODULE;

	return devm_gpiochip_add_data(dev, &st->gpio_chip, st);
}

static const struct platform_device_id ltc4283_gpio_id_table[] = {
	{ "ltc4283-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(platform, ltc4283_gpio_id_table);

static const struct of_device_id ltc4283_of_id_table[] = {
	{ "adi,ltc4283-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc4283_of_id_table);

static struct platform_driver ltc4283_gpio_driver = {
	.driver	= {
		.name = "ltc4283-gpio",
		.of_match_table = ltc4283_of_id_table,
	},
	.probe = ltc4283_gpio_probe,
	.id_table = ltc4283_gpio_id_table,
};
module_platform_driver(ltc4283_gpio_driver);

MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("GPIO LTC4283 Driver");
MODULE_LICENSE("GPL");
