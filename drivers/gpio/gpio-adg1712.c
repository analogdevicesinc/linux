// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADG1712 quad SPST switch driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define ADG1712_NUM_SWITCHES	4

struct adg1712 {
	struct gpio_descs *switch_gpios;
};

static int adg1712_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adg1712 *adg1712;
	u32 switch_states[ADG1712_NUM_SWITCHES] = {0}; /* Default all switches off */
	int ret, i;

	adg1712 = devm_kzalloc(dev, sizeof(*adg1712), GFP_KERNEL);
	if (!adg1712)
		return -ENOMEM;

	adg1712->switch_gpios = devm_gpiod_get_array(dev, "switch", GPIOD_OUT_LOW);
	if (IS_ERR(adg1712->switch_gpios))
		return dev_err_probe(dev, PTR_ERR(adg1712->switch_gpios),
				     "failed to get switch gpios\n");

	if (adg1712->switch_gpios->ndescs != ADG1712_NUM_SWITCHES)
		return dev_err_probe(dev, -EINVAL,
				     "expected %d gpios, got %d\n",
				     ADG1712_NUM_SWITCHES,
				     adg1712->switch_gpios->ndescs);

	ret = device_property_read_u32_array(dev, "switch-states", switch_states,
					     ADG1712_NUM_SWITCHES);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "failed to read switch-states\n");

	for (i = 0; i < ADG1712_NUM_SWITCHES; i++) {
		if (switch_states[i] > 1) {
			dev_warn(dev, "invalid switch state %u for switch %d, using 0\n",
				 switch_states[i], i);
			switch_states[i] = 0;
		}

		ret = gpiod_set_value_cansleep(adg1712->switch_gpios->desc[i],
					       switch_states[i]);
		if (ret)
			return dev_err_probe(dev, ret, "failed to set switch %d\n", i);
	}

	platform_set_drvdata(pdev, adg1712);

	dev_info(dev, "ADG1712 switch controller configured\n");

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

MODULE_DESCRIPTION("Analog Devices ADG1712 quad SPST switch driver");
MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_LICENSE("GPL");
