// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices MAX16150/MAX16169 Pushbutton On/Off Controller
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

enum max161x_variant {
	MAX161X_A,
	MAX161X_B,
	MAX161X_C,
};

enum max161x_type {
	MAX16150,
	MAX16169,
};

struct max161x_data {
	struct device *dev;
	struct gpio_desc *gpio_pb_in;
	struct gpio_desc *gpio_out;
	struct gpio_desc *gpio_clr;
	struct gpio_desc *gpio_int;

	struct hrtimer debounce_timer;
	struct hrtimer shutdown_timer;
	ktime_t debounce_time;
	ktime_t shutdown_time;

	bool out_asserted;
	enum max161x_variant variant;
	enum max161x_type type;
};

/* Debounce handler */
static enum hrtimer_restart max161x_debounce_timer_cb(struct hrtimer *timer)
{
	struct max161x_data *data = container_of(timer, struct max161x_data,
				    debounce_timer);

	if (gpiod_get_value(data->gpio_pb_in)) {
		dev_info(data->dev, "Debounced button press detected. Asserting OUT.");
		gpiod_set_value(data->gpio_clr, 1);
		data->out_asserted = true;

		/* Start shutdown timer for A variant */
		if (data->variant == MAX161X_A)
			hrtimer_start(&data->shutdown_timer, data->shutdown_time,
				      HRTIMER_MODE_REL);
	} else {
		dev_info(data->dev, "Button released before debounce time elapsed.");
	}

	return HRTIMER_NORESTART;
}

/* Shutdown handler */
static enum hrtimer_restart max161x_shutdown_timer_cb(struct hrtimer *timer)
{
	struct max161x_data *data = container_of(timer, struct max161x_data,
				    shutdown_timer);

	if (data->variant == MAX161X_A && data->out_asserted) {
		dev_info(data->dev, "Shutdown time exceeded. Deasserting OUT.");
		gpiod_set_value(data->gpio_clr, 0);
		data->out_asserted = false;
	}

	return HRTIMER_NORESTART;
}

static irqreturn_t max161x_pb_in_irq_handler(int irq, void *dev_id)
{
	struct max161x_data *data = dev_id;

	if (gpiod_get_value(data->gpio_pb_in)) {
		/* Button pressed */
		hrtimer_start(&data->debounce_timer, data->debounce_time,
			      HRTIMER_MODE_REL);
	} else {
		/* Button released */
		if (data->variant == MAX161X_A && data->out_asserted) {
			dev_info(data->dev, "Cancelling shutdown timer.");
			hrtimer_cancel(&data->shutdown_timer);
		}
	}

	return IRQ_HANDLED;
}

static int max161x_probe(struct platform_device *pdev)
{
	struct max161x_data *data;
	const char *variant, *type;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;

	ret = device_property_read_string(&pdev->dev, "compatible", &type);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read device type\n");
		return ret;
	}

	if (!strcmp(type, "adi,max16150")) {
		data->type = MAX16150;
	} else if (!strcmp(type, "adi,max16169")) {
		data->type = MAX16169;
	} else {
		dev_err(&pdev->dev, "Unknown device type: %s\n", type);
		return -EINVAL;
	}

	ret = device_property_read_string(&pdev->dev, "adi,variant", &variant);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read device variant\n");
		return ret;
	}

	switch (variant[0]) {
	case 'A':
		data->variant = MAX161X_A;
		data->debounce_time = ktime_set(0, 50 * NSEC_PER_MSEC);
		data->shutdown_time = (data->type == MAX16150) ? ktime_set(8,
				      0) : ktime_set(16, 0);
		break;
	case 'B':
		data->variant = MAX161X_B;
		data->debounce_time = (data->type == MAX16150) ? ktime_set(2,
				      0) : ktime_set(50, 0);
		data->shutdown_time = ktime_set(16, 0);
		break;
	case 'C':
		data->variant = MAX161X_C;
		data->debounce_time = ktime_set(0, 50 * NSEC_PER_MSEC);
		data->shutdown_time = ktime_set(16, 0);
		break;
	default:
		dev_err(&pdev->dev, "Unknown device variant: %s\n", variant);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "Detected %s variant %s\n",
		 (data->type == MAX16150) ? "MAX16150" : "MAX16169", variant);

	data->gpio_pb_in = devm_gpiod_get(&pdev->dev, "pb_in", GPIOD_IN);
	if (IS_ERR(data->gpio_pb_in))
		return PTR_ERR(data->gpio_pb_in);

	data->gpio_out = devm_gpiod_get(&pdev->dev, "out", GPIOD_OUT_LOW);
	if (IS_ERR(data->gpio_out))
		return PTR_ERR(data->gpio_out);

	data->gpio_clr = devm_gpiod_get(&pdev->dev, "clr", GPIOD_OUT_LOW);
	if (IS_ERR(data->gpio_clr))
		return PTR_ERR(data->gpio_clr);

	data->gpio_int = devm_gpiod_get(&pdev->dev, "int", GPIOD_IN);
	if (IS_ERR(data->gpio_int))
		return PTR_ERR(data->gpio_int);

	hrtimer_init(&data->debounce_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->debounce_timer.function = max161x_debounce_timer_cb;

	hrtimer_init(&data->shutdown_timer, CLOCKS_MONOTONIC, HRTIMER_MODE_REL);
	data->shutdown_timer.function = max161x_shutdown_timer_cb;

	ret = devm_request_irq(&pdev->dev, gpiod_to_irq(data->gpio_pb_in),
			       max161x_pb_in_irq_handler,
			       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "max161x_pb_in", data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ for PB_IN\n");
		return ret;
	}

	platform_set_drvdata(pdev, data);
	dev_info(&pdev->dev, "MAX161x driver initialized\n");

	return 0;
}

static int max161x_remove(struct platform_device *pdev)
{
	struct max161x_data *data = platform_get_drvdata(pdev);

	hrtimer_cancel(&data->debounce_timer);
	hrtimer_cancel(&data->shutdown_timer);
	return 0;
}

static const struct of_device_id max161x_dt_ids[] = {
	{ .compatible = "adi,max16150" },
	{ .compatible = "adi,max16169" },
	{ }
};
MODULE_DEVICE_TABLE(of, max161x_dt_ids);

static struct platform_driver max161x_driver = {
	.driver = {
		.name = "max161x",
		.of_match_table = max161x_dt_ids,
	},
	.probe = max161x_probe,
	.remove = max161x_remove,
};
module_platform_driver(max161x_driver);

MODULE_AUTHOR("Marc Paolo Sosa <marcpaolo.sosa@analog.com");
MODULE_DESCRIPTION("MAX16150/MAX16169 Pushbutton Controller Driver");
MODULE_LICENSE("GPL");
