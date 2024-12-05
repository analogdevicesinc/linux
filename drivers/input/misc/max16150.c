// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices MAX16150/MAX16169 Pushbutton Driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

static int irq_number;
static ktime_t prev_time;
static const ktime_t short_pulse = 32 * NSEC_PER_MSEC;
static const ktime_t long_pulse = 128 * NSEC_PER_MSEC;

static irqreturn_t max16150_isr(int irq, void *dev_id)
{
	ktime_t now = ktime_get();
	ktime_t duration;
	struct input_dev *button = dev_id;
	static bool falling_edge = false;

	if (gpio_get_value(irq) == 0) {
		prev_time = now;
		falling_edge = true;
	} else if (falling_edge) {
		duration = ktime_sub(now, prev_time);
		if (duration >= long_pulse)
			input_report_key(button, KEY_POWER, 1);
		else
			input_report_key(button, KEY_WAKEUP, 1);

		input_sync(button);
		falling_edge = false;
	}

	return IRQ_HANDLED;
}

static int get_irq(struct platform_device *pdev)
{
	return platform_get_irq(pdev, 0);
}

static int max16150_probe(struct platform_device *pdev)
{
	struct input_dev *button;
	int ret;

	button = devm_input_allocate_device(&pdev->dev);
	if (!button) {
		dev_err(&pdev->dev, "Can't allocate input device\n");
		return -ENOMEM;
	}

	button->name = "max16150";
	button->phys = "max16150/input0";
	button->id.bustype = BUS_HOST;
	input_set_capability(button, EV_KEY, KEY_POWER);
	input_set_capability(button, EV_KEY, KEY_WAKEUP);

	ret = input_register_device(button);
	if (ret) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", ret);
		return ret;
	}

	irq_number = get_irq(pdev);
	if (irq_number < 0) {
		dev_err(&pdev->dev, "Can't get IRQ\n");
		return irq_number;
	}

	ret = devm_request_irq(&pdev->dev, irq_number, max16150_isr,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "max16150_irq", button);
	if (ret)
		return ret;

	prev_time = ktime_get();
	platform_set_drvdata(pdev, button);
	device_init_wakeup(&pdev->dev, true);

	return 0;
}

static int max16150_remove(struct platform_device *pdev)
{
	struct input_dev *button = platform_get_drvdata(pdev);

	input_unregister_device(button);
	return 0;
}

static const struct of_device_id max16150_of_match[] = {
	{ .compatible = "adi,max16150" },
	{ .compatible = "adi,max16169" },
	{ }
};
MODULE_DEVICE_TABLE(of, max16150_of_match);

static struct platform_driver max16150_driver = {
	.probe = max16150_probe,
	.remove = max16150_remove,
	.driver = {
		.name = "max16150",
		.of_match_table = max16150_of_match,
	},
};

module_platform_driver(max16150_driver);

MODULE_AUTHOR("Marc Paolo Sosa <marcpaolo.sosa@analog.com>");
MODULE_DESCRIPTION("MAX16150/MAX16169 Pushbutton Driver");
MODULE_LICENSE("GPL");
