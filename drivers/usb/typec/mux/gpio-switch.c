// SPDX-License-Identifier: GPL-2.0
/**
 * gpio-switch.c - typec switch via a simple GPIO control.
 *
 * Copyright 2019 NXP
 * Author: Jun Li <jun.li@nxp.com>
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/usb/typec_mux.h>

struct gpio_typec_switch {
	struct typec_switch *sw;
	struct mutex lock;
	struct gpio_desc *ss_sel;
	struct gpio_desc *ss_reset;
};

static int switch_gpio_set(struct typec_switch *sw,
			   enum typec_orientation orientation)
{
	struct gpio_typec_switch *gpio_sw = typec_switch_get_drvdata(sw);

	mutex_lock(&gpio_sw->lock);

	switch (orientation) {
	case TYPEC_ORIENTATION_NORMAL:
		gpiod_set_value_cansleep(gpio_sw->ss_sel, 1);
		break;
	case TYPEC_ORIENTATION_REVERSE:
		gpiod_set_value_cansleep(gpio_sw->ss_sel, 0);
		break;
	case TYPEC_ORIENTATION_NONE:
		break;
	}

	mutex_unlock(&gpio_sw->lock);

	return 0;
}

static int typec_switch_gpio_probe(struct platform_device *pdev)
{
	struct gpio_typec_switch	*gpio_sw;
	struct device			*dev = &pdev->dev;
	struct typec_switch_desc sw_desc;

	gpio_sw = devm_kzalloc(dev, sizeof(*gpio_sw), GFP_KERNEL);
	if (!gpio_sw)
		return -ENOMEM;

	platform_set_drvdata(pdev, gpio_sw);

	sw_desc.drvdata = gpio_sw;
	sw_desc.fwnode = dev->fwnode;
	sw_desc.set = switch_gpio_set;
	mutex_init(&gpio_sw->lock);

	/* Get the super speed mux reset GPIO, it's optional */
	gpio_sw->ss_reset = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_sw->ss_reset))
		return PTR_ERR(gpio_sw->ss_reset);

	if (gpio_sw->ss_reset)
		usleep_range(700, 1000);

	/* Get the super speed active channel selection GPIO */
	gpio_sw->ss_sel = devm_gpiod_get(dev, "switch", GPIOD_OUT_LOW);
	if (IS_ERR(gpio_sw->ss_sel))
		return PTR_ERR(gpio_sw->ss_sel);

	gpio_sw->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(gpio_sw->sw)) {
		dev_err(dev, "Error registering typec switch: %ld\n", PTR_ERR(gpio_sw->sw));
		return PTR_ERR(gpio_sw->sw);
	}

	return 0;
}

static int typec_switch_gpio_remove(struct platform_device *pdev)
{
	struct gpio_typec_switch *gpio_sw = platform_get_drvdata(pdev);

	typec_switch_unregister(gpio_sw->sw);

	return 0;
}

static const struct of_device_id of_typec_switch_gpio_match[] = {
	{ .compatible = "nxp,ptn36043" },
	{ .compatible = "nxp,cbtl04gp" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_typec_switch_gpio_match);

static struct platform_driver typec_switch_gpio_driver = {
	.probe		= typec_switch_gpio_probe,
	.remove		= typec_switch_gpio_remove,
	.driver		= {
		.name	= "typec-switch-gpio",
		.of_match_table = of_typec_switch_gpio_match,
	},
};

module_platform_driver(typec_switch_gpio_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TypeC Super Speed Switch GPIO driver");
MODULE_AUTHOR("Jun Li <jun.li@nxp.com>");
