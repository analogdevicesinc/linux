// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * This driver is an example of using the GPIO as external interrupt,
 * when pressing the push botton on the board repeatedly,
 * LED will turn on and off repeatedly.
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/types.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/gpio.h>

#define DRIVER_NAME "button-led"

struct button_led {
	struct miscdevice mdev;
	int irq;
	unsigned int button;
	unsigned int led;
	unsigned int led_on;
};


#ifdef CONFIG_OF
static const struct of_device_id button_led_of_match[] = {
	{
		.compatible = "adi,button-led",
	},
	{},
};
MODULE_DEVICE_TABLE(of, button_led_of_match);
#endif

/*
 * button_interrupt_handler - button interrupt handler
 *
 */
static irqreturn_t button_interrupt_handler(int irq, void *dev_id)
{
	struct button_led *gpio_dev = dev_id;

	if (gpio_is_valid(gpio_dev->led)) {
		gpio_dev->led_on = gpio_get_value(gpio_dev->led);
		gpio_set_value(gpio_dev->led, !gpio_dev->led_on);
	} else
		return IRQ_NONE;

	return IRQ_HANDLED;
}
/**
 * button_led_probe - Initialize module
 *
 * Register the misc device and notifier handler.
 *
 */
static int button_led_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *match;
	struct button_led *gpio_dev = NULL;
	int ret = 0;

	gpio_dev = devm_kzalloc(&pdev->dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	/*match device from dts*/
#ifdef CONFIG_OF
	match = of_match_device(of_match_ptr(button_led_of_match), &pdev->dev);
	if (match) {
		if (of_property_read_u32(node, "button_gpio", &gpio_dev->button)) {
			dev_err(&pdev->dev, "No button_gpio specified\n");
			return -ENOENT;
		}

		if (of_property_read_u32(node, "led_gpio", &gpio_dev->led)) {
			dev_err(&pdev->dev, "No led_gpio specified\n");
			return -ENOENT;
		}
/*
 *		ret = softconfig_of_set_group_active_pins_output(&pdev->dev, node,
 *						"en-pins", true);
 *		if (ret)
 *			return ret;
 */
	}
#else
	dev_err(&pdev->dev, "No button-led driver specified\n");
	return -ENOENT;
#endif /* CONFIG_OF */

	/* register device */
	gpio_dev->mdev.minor = MISC_DYNAMIC_MINOR;
	gpio_dev->mdev.name = dev_name(&pdev->dev);
	ret = misc_register(&gpio_dev->mdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register button_led miscdev\n");
		return ret;
	}
	dev_info(&pdev->dev, "Registered ADI button-led device %s\n",
				gpio_dev->mdev.name);

	/* request button_gpio */
	ret = devm_gpio_request(&pdev->dev, gpio_dev->button, "button_gpio");
	if (ret) {
		dev_err(&pdev->dev, "Cannot request button_pin\n");
		goto out;
	}
	/* request led_gpio and set the direction to output */
	ret = devm_gpio_request_one(&pdev->dev, gpio_dev->led, GPIOF_OUT_INIT_LOW,
				"led_gpio");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request led_pin\n");
		goto out;
	}

	/* register irq */
	gpio_dev->irq = gpio_to_irq(gpio_dev->button);
	ret = devm_request_irq(&pdev->dev, gpio_dev->irq, button_interrupt_handler,
				(IRQF_SHARED | IRQF_TRIGGER_RISING), DRIVER_NAME, gpio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot request irq\n");
		goto out;
	}
	platform_set_drvdata(pdev, gpio_dev);
	return 0;

out:
	misc_deregister(&gpio_dev->mdev);
	return ret;
}

/**
 * button_led_remove - Initialize module
 *
 * Unregister the misc device.
 */
static int button_led_remove(struct platform_device *pdev)
{
	struct button_led *gpio_dev = platform_get_drvdata(pdev);

/*
 *	softconfig_of_set_group_active_pins_output(&pdev->dev,
 *							pdev->dev.of_node, "en-pins", false);
 */
	misc_deregister(&gpio_dev->mdev);
	return 0;
}

static struct platform_driver button_led_driver = {
	.probe = button_led_probe,
	.remove = button_led_remove,
	.driver = {
		.name = DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(button_led_of_match),
#endif
	},
};

/*
 * button_led_init - Initialize module
 *
 * Register the button-led driver.
 */

static int __init button_led_init(void)
{
	int ret;

	pr_info("Button interrupt trigger LED driver\n");

	ret = platform_driver_register(&button_led_driver);
	if (ret) {
		pr_info("unable to register driver\n");
		return ret;
	}

	return 0;
}

/*
 * button_led_exit - Deinitialize module
 *
 * Unregister the button-led driver.
 */
static void __exit button_led_exit(void)
{
	platform_driver_unregister(&button_led_driver);
}

module_init(button_led_init);
module_exit(button_led_exit);

MODULE_AUTHOR("hfeng <huanhuan.feng@analog.com>");
MODULE_DESCRIPTION("Button interrupt trigger LED driver");
MODULE_LICENSE("GPL");
