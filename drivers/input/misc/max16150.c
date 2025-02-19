// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices MAX16150/MAX16169 Pushbutton Driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>

#define SHORT_PRESS_MIN_MS 32
#define SHORT_PRESS_MAX_MS 127
#define LONG_PRESS_MIN_MS 128

struct max16150_data {
	struct input_dev *bttn;
	int key_event;
	ktime_t irq_rising_time;
	ktime_t irq_falling_time;
};

static irqreturn_t max16150_rise_irq(int irq, void *_data)
{
	struct max16150_data *data = _data;
	struct input_dev *bttn = data->bttn;

	data->irq_rising_time = ktime_get();

	input_report_key(bttn, data->key_event, 1);
	input_sync(bttn);

	return IRQ_HANDLED;
}

static irqreturn_t max16150_fall_irq(int irq, void *_data)
{
	struct max16150_data *data = _data;
	struct input_dev *bttn = data->bttn;
	struct device *dev = bttn->dev.parent;
	ktime_t irq_duration;
	unsigned long duration_ms;

	data->irq_falling_time = ktime_get();
	irq_duration = ktime_sub(data->irq_falling_time, data->irq_rising_time);
	duration_ms = ktime_to_ms(irq_duration);

	dev_dbg_ratelimited(dev, "Interrupt duration: %lu ms\n", duration_ms);

	if (duration_ms >= SHORT_PRESS_MIN_MS && duration_ms <= SHORT_PRESS_MAX_MS) {
		dev_info(dev, "Short press detected\n");
		input_event(bttn, EV_MSC, MSC_SCAN, 1);
	} else if (duration_ms >= LONG_PRESS_MIN_MS) {
		dev_info(dev, "Long press detected\n");
		input_event(bttn, EV_MSC, MSC_SCAN, 2);
	}

	input_report_key(bttn, data->key_event, 0);
	input_sync(bttn);

	return IRQ_HANDLED;
}

static int max16150_probe(struct platform_device *pdev)
{
	struct max16150_data *data;
	struct input_dev *bttn;
	int fall_irq, rise_irq, err;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	bttn = devm_input_allocate_device(&pdev->dev);
	if (!bttn) {
		dev_err(&pdev->dev, "Can't allocate input device\n");
		return -ENOMEM;
	}

	data->bttn = bttn;
	if (of_property_read_u32(np, "key-event", &data->key_event))
		data->key_event = KEY_POWER;

	bttn->name = "max16150";
	bttn->phys = "max16150/input0";
	bttn->id.bustype = BUS_HOST;
	input_set_capability(bttn, EV_KEY, data->key_event);
	input_set_capability(bttn, EV_MSC, MSC_SCAN);

	fall_irq = platform_get_irq(pdev, 0);
	if (fall_irq < 0)
		return fall_irq;

	rise_irq = platform_get_irq(pdev, 1);
	if (rise_irq < 0)
		return rise_irq;

	err = devm_request_threaded_irq(&pdev->dev, rise_irq, NULL,
					max16150_rise_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"max16150_rising", data);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't register rise irq: %d\n", err);
		return err;
	}

	err = devm_request_threaded_irq(&pdev->dev, fall_irq, NULL,
					max16150_fall_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"max16150_falling", data);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't register fall irq: %d\n", err);
		return err;
	}

	err = input_register_device(bttn);
	if (err) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, data);
	device_init_wakeup(&pdev->dev, true);

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
	.driver = {
		.name = "max16150",
		.of_match_table = max16150_of_match,
	},
};
module_platform_driver(max16150_driver);

MODULE_AUTHOR("Marc Paolo Sosa <marcpaolo.sosa@analog.com>");
MODULE_DESCRIPTION("MAX16150/MAX16169 Pushbutton Driver");
MODULE_LICENSE("GPL");
