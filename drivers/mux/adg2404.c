// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADG2404 4:1 multiplexer driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define ADG2404_CHANNELS	4

struct adg2404_mux {
	struct gpio_desc *gpio_a0;
	struct gpio_desc *gpio_a1;
	struct gpio_desc *gpio_en;
};

static int adg2404_set(struct mux_control *mux, int state)
{
	struct adg2404_mux *adg2404 = mux_chip_priv(mux->chip);

	if (state == MUX_IDLE_DISCONNECT) {
		gpiod_set_value_cansleep(adg2404->gpio_en, 0);
		return 0;
	}

	gpiod_set_value_cansleep(adg2404->gpio_a0, state & 0x01);

	gpiod_set_value_cansleep(adg2404->gpio_a1, (state >> 1) & 0x01);

	gpiod_set_value_cansleep(adg2404->gpio_en, 1);

	return 0;
}

static const struct mux_control_ops adg2404_ops = {
	.set = adg2404_set,
};

static int adg2404_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mux_chip *mux_chip;
	struct adg2404_mux *adg2404;
	s32 idle_state;
	int ret;

	mux_chip = devm_mux_chip_alloc(dev, 1, sizeof(*adg2404));
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	adg2404 = mux_chip_priv(mux_chip);
	mux_chip->ops = &adg2404_ops;

	adg2404->gpio_a0 = devm_gpiod_get(dev, "a0", GPIOD_OUT_LOW);
	if (IS_ERR(adg2404->gpio_a0))
		return dev_err_probe(dev, PTR_ERR(adg2404->gpio_a0),
				     "failed to get a0 gpio\n");

	adg2404->gpio_a1 = devm_gpiod_get(dev, "a1", GPIOD_OUT_LOW);
	if (IS_ERR(adg2404->gpio_a1))
		return dev_err_probe(dev, PTR_ERR(adg2404->gpio_a1),
				     "failed to get a1 gpio\n");

	adg2404->gpio_en = devm_gpiod_get(dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(adg2404->gpio_en))
		return dev_err_probe(dev, PTR_ERR(adg2404->gpio_en),
				     "failed to get en gpio\n");

	mux_chip->mux->states = ADG2404_CHANNELS;

	ret = device_property_read_u32(dev, "idle-state", (u32 *)&idle_state);
	if (ret >= 0 && idle_state != MUX_IDLE_AS_IS) {
		if (idle_state < 0 || idle_state >= mux_chip->mux->states) {
			if (idle_state != MUX_IDLE_DISCONNECT) {
				dev_err(dev, "invalid idle-state %u\n", idle_state);
				return -EINVAL;
			}
		}
		mux_chip->mux->idle_state = idle_state;
	}

	ret = devm_mux_chip_register(dev, mux_chip);
	if (ret < 0)
		return ret;

	dev_info(dev, "ADG2404 %u-way mux-controller registered\n",
		 mux_chip->mux->states);

	return 0;
}

static const struct of_device_id adg2404_dt_ids[] = {
	{ .compatible = "adi,adg2404", },
	{ }
};
MODULE_DEVICE_TABLE(of, adg2404_dt_ids);

static struct platform_driver adg2404_driver = {
	.driver = {
		.name = "adg2404",
		.of_match_table	= adg2404_dt_ids,
	},
	.probe = adg2404_probe,
};
module_platform_driver(adg2404_driver);

MODULE_DESCRIPTION("Analog Devices ADG2404 multiplexer driver");
MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_LICENSE("GPL");