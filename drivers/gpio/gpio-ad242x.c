// SPDX-License-Identifier: GPL-2.0-only

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

struct ad242x_gpio {
	struct gpio_chip chip;
	struct a2b_node *node;
	u32 gpio_od_mask;
};

static int ad242x_gpio_request(struct gpio_chip *chip, unsigned int gpio)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);

	if (gpio == 0 && a2b_node_is_main(ad242x_gpio->node))
		return -EBUSY;

	if (ad242x_gpio->gpio_od_mask & BIT(gpio))
		return -EBUSY;

	return 0;
}

static int ad242x_gpio_get_value(struct gpio_chip *chip, unsigned int gpio)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	unsigned int val, reg, out_en;
	int ret;

	ret = regmap_read(regmap, A2B_GPIOOEN, &out_en);
	if (out_en & BIT(gpio))
		reg = A2B_GPIODAT;
	else
		reg = A2B_GPIODAT_IN;

	ret = regmap_read(regmap, reg, &val);
	if (ret < 0)
		return ret;

	return !!(val & BIT(gpio));
}

static void ad242x_gpio_set_value(struct gpio_chip *chip, unsigned int gpio,
				  int value)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	uint8_t bit = BIT(gpio);
	int ret;

	if (value)
		ret = regmap_write(regmap, A2B_GPIODAT_SET, bit);
	else
		ret = regmap_write(regmap, A2B_GPIODAT_CLR, bit);

	if (ret < 0)
		dev_err(&ad242x_gpio->node->dev, "Unable to set GPIO #%d: %d\n",
			gpio, ret);
}

static int ad242x_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int gpio)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	uint8_t bit = BIT(gpio);
	int ret;

	ret = regmap_update_bits(regmap, A2B_GPIOOEN, bit, 0);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, A2B_GPIOIEN, bit, bit);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, A2B_INTMSK1, bit, bit);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad242x_gpio_direction_output(struct gpio_chip *chip,
					unsigned int gpio, int value)
{
	struct ad242x_gpio *ad242x_gpio = gpiochip_get_data(chip);
	struct regmap *regmap = ad242x_gpio->node->regmap;
	uint8_t bit = BIT(gpio);
	int ret;

	ret = regmap_update_bits(regmap, A2B_GPIOIEN, bit, 0);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, A2B_GPIOOEN, bit, bit);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, A2B_INTMSK1, bit, 0);
	if (ret < 0)
		return ret;

	ad242x_gpio_set_value(chip, gpio, value);

	return 0;
}

static int ad242x_gpio_over_distance_init(struct device *dev,
					  struct ad242x_gpio *ad242x_gpio)
{
	struct regmap *regmap = ad242x_gpio->node->regmap;
	struct device_node *np, *child_np;
	int ret = 0;

	np = of_get_child_by_name(dev->of_node, "gpio-over-distance");
	if (!np)
		return 0;

	for_each_available_child_of_node (np, child_np) {
		u32 reg, port_mask, bit;
		bool output, inv;

		ret = of_property_read_u32(child_np, "reg", &reg);
		if (ret < 0)
			continue;

		ret = of_property_read_u32(child_np, "adi,virtual-port-mask",
					   &port_mask);
		if (ret < 0)
			continue;

		if (reg > 7) {
			ret = -EINVAL;
			break;
		}

		bit = BIT(reg);

		ret = regmap_update_bits(regmap, A2B_GPIODEN, bit, bit);
		if (ret < 0)
			break;

		ret = regmap_write(regmap, A2B_GPIOD_MSK(reg), port_mask);
		if (ret < 0)
			break;

		output = of_property_read_bool(child_np, "adi,gpio-output");
		ret = regmap_update_bits(regmap, A2B_GPIOOEN, bit,
					 output ? bit : 0);
		if (ret < 0)
			break;

		inv = of_property_read_bool(child_np, "adi,gpio-inverted");
		ret = regmap_update_bits(regmap, A2B_GPIODINV, bit,
					 inv ? bit : 0);
		if (ret < 0)
			break;

		ad242x_gpio->gpio_od_mask |= bit;
		dev_info(
			dev,
			"pin %d set up as gpio-over-distance, port mask 0x%02x\n",
			reg, port_mask);
	}

	of_node_put(np);

	return ret;
}

static int ad242x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad242x_gpio *ad242x_gpio;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	ad242x_gpio = devm_kzalloc(dev, sizeof(*ad242x_gpio), GFP_KERNEL);
	if (!ad242x_gpio)
		return -ENOMEM;

	ad242x_gpio->node = dev_to_a2b_node(dev->parent);

	ad242x_gpio->chip.request = ad242x_gpio_request;
	ad242x_gpio->chip.direction_input = ad242x_gpio_direction_input;
	ad242x_gpio->chip.direction_output = ad242x_gpio_direction_output;
	ad242x_gpio->chip.get = ad242x_gpio_get_value;
	ad242x_gpio->chip.set = ad242x_gpio_set_value;
	ad242x_gpio->chip.can_sleep = 1;
	ad242x_gpio->chip.base = -1;
	ad242x_gpio->chip.ngpio = 8;
	ad242x_gpio->chip.label = "ad242x-gpio";
	ad242x_gpio->chip.owner = THIS_MODULE;
	ad242x_gpio->chip.parent = dev;

	dev_info(dev, "A2B node ID %d\n", ad242x_gpio->node->id);

	ret = ad242x_gpio_over_distance_init(dev, ad242x_gpio);
	if (ret < 0) {
		dev_err(dev, "GPIO over distance init failed: %d\n", ret);
		return ret;
	}

	return devm_gpiochip_add_data(dev, &ad242x_gpio->chip, ad242x_gpio);
}

static const struct of_device_id ad242x_gpio_of_match[] = {
	{
		.compatible = "adi,ad2428w-gpio",
	},
	{}
};
MODULE_DEVICE_TABLE(of, ad242x_gpio_of_match);

static struct platform_driver ad242x_gpio_driver = {
	.driver = {
		.name = "ad242x-gpio",
		.of_match_table = ad242x_gpio_of_match,
	},
	.probe = ad242x_gpio_probe,
};
module_platform_driver(ad242x_gpio_driver);

MODULE_DESCRIPTION("AD242x GPIO driver");
MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_LICENSE("GPL v2");
