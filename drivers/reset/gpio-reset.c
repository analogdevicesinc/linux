/*
 * GPIO Reset Controller driver
 *
 * Copyright 2013 Philipp Zabel, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/reset-controller.h>

struct gpio_reset_data {
	struct reset_controller_dev rcdev;
	struct gpio_desc *gpiod;
	s32 delay_us;
	s32 post_delay_ms;
};

static int gpio_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct gpio_reset_data *drvdata = container_of(rcdev,
			struct gpio_reset_data, rcdev);

	if (drvdata->delay_us < 0)
		return -ENOSYS;

	gpiod_set_value_cansleep(drvdata->gpiod, 1);
	udelay(drvdata->delay_us);
	gpiod_set_value_cansleep(drvdata->gpiod, 0);

	if (drvdata->post_delay_ms < 0)
		return 0;

	msleep(drvdata->post_delay_ms);
	return 0;
}

static int gpio_reset_assert(struct reset_controller_dev *rcdev,
		unsigned long id)
{
	struct gpio_reset_data *drvdata = container_of(rcdev,
			struct gpio_reset_data, rcdev);

	gpiod_set_value_cansleep(drvdata->gpiod, 1);
	return 0;
}

static int gpio_reset_deassert(struct reset_controller_dev *rcdev,
		unsigned long id)
{
	struct gpio_reset_data *drvdata = container_of(rcdev,
			struct gpio_reset_data, rcdev);

	gpiod_set_value_cansleep(drvdata->gpiod, 0);

	return 0;
}

static struct reset_control_ops gpio_reset_ops = {
	.reset = gpio_reset,
	.assert = gpio_reset_assert,
	.deassert = gpio_reset_deassert,
};

static int of_gpio_reset_xlate(struct reset_controller_dev *rcdev,
			       const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static int gpio_reset_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct gpio_reset_data *drvdata;
	enum gpiod_flags flags;
	bool initially_in_reset;
	int ret;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	if (gpiod_count(&pdev->dev, "reset") != 1) {
		dev_err(&pdev->dev,
			"reset-gpios property missing, or not a single gpio\n");
		return -EINVAL;
	}

	initially_in_reset = of_property_read_bool(np, "initially-in-reset");
	flags = initially_in_reset ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW;
	drvdata->gpiod = devm_gpiod_get(&pdev->dev, "reset", flags);
	if (IS_ERR(drvdata->gpiod)) {
		ret = PTR_ERR(drvdata->gpiod);
		if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(&pdev->dev, "invalid reset gpio: %d\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u32(np, "reset-delay-us", &drvdata->delay_us);
	if (ret < 0)
		drvdata->delay_us = -1;
	else if (drvdata->delay_us < 0)
		dev_warn(&pdev->dev, "reset delay too high\n");

	/* It is optional.
	 * Some devices need some milliseconds to wait after reset.
	 */
	ret = of_property_read_u32(np, "reset-post-delay-ms", &drvdata->post_delay_ms);
	if (ret < 0)
		drvdata->post_delay_ms = -1;

	platform_set_drvdata(pdev, drvdata);

	drvdata->rcdev.of_node = np;
	drvdata->rcdev.owner = THIS_MODULE;
	drvdata->rcdev.nr_resets = 1;
	drvdata->rcdev.ops = &gpio_reset_ops;
	drvdata->rcdev.of_xlate = of_gpio_reset_xlate;
	reset_controller_register(&drvdata->rcdev);

	return 0;
}

static int gpio_reset_remove(struct platform_device *pdev)
{
	struct gpio_reset_data *drvdata = platform_get_drvdata(pdev);

	reset_controller_unregister(&drvdata->rcdev);

	return 0;
}

static struct of_device_id gpio_reset_dt_ids[] = {
	{ .compatible = "gpio-reset" },
	{ }
};

#ifdef CONFIG_PM_SLEEP
static int gpio_reset_suspend(struct device *dev)
{
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}
static int gpio_reset_resume(struct device *dev)
{
	pinctrl_pm_select_default_state(dev);

	return 0;
}
#endif

static const struct dev_pm_ops gpio_reset_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(gpio_reset_suspend, gpio_reset_resume)
};

static struct platform_driver gpio_reset_driver = {
	.probe = gpio_reset_probe,
	.remove = gpio_reset_remove,
	.driver = {
		.name = "gpio-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_reset_dt_ids),
		.pm = &gpio_reset_pm_ops,
	},
};

static int __init gpio_reset_init(void)
{
	return platform_driver_register(&gpio_reset_driver);
}
arch_initcall(gpio_reset_init);

static void __exit gpio_reset_exit(void)
{
	platform_driver_unregister(&gpio_reset_driver);
}
module_exit(gpio_reset_exit);

MODULE_AUTHOR("Philipp Zabel <p.zabel@pengutronix.de>");
MODULE_DESCRIPTION("gpio reset controller");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-reset");
MODULE_DEVICE_TABLE(of, gpio_reset_dt_ids);
