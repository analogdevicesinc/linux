// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices MAX16150/MAX16169 Pushbutton Driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define MAX16150_LONG_INTERRUPT 120000000

struct max16150_chip_info {
	bool has_clr_gpio;
};

struct max16150_device {
	struct input_dev *input;
	struct gpio_desc *gpiod;
	struct gpio_desc *clr_gpiod;
	const struct max16150_chip_info *chip_info;
	u64 low, high, duration;
	unsigned int keycode;
};

static irqreturn_t max16150_irq_handler(int irq, void *_max16150)
{
	struct max16150_device *max16150 = _max16150;
	int value;

	value = gpiod_get_value(max16150->gpiod);

	if (!value) {
		max16150->low = ktime_get_ns();
		return IRQ_HANDLED;
	}

	max16150->high = ktime_get_ns();
	if (max16150->low) {
		max16150->duration = max16150->high - max16150->low;

		if (max16150->duration > MAX16150_LONG_INTERRUPT) {
			gpiod_set_value(max16150->clr_gpiod, 1);
			input_report_key(max16150->input, max16150->keycode, 1);
			input_sync(max16150->input);
			input_report_key(max16150->input, max16150->keycode, 0);
			input_sync(max16150->input);
		}

		max16150->low = 0;
	}

	return IRQ_HANDLED;
}

static const struct max16150_chip_info max16150_variant_a = {
	.has_clr_gpio = true,
};

static const struct max16150_chip_info max16150_variant_b = {
	.has_clr_gpio = false,
};

static int max16150_probe(struct platform_device *pdev)
{
	const struct max16150_chip_info *chip_info;
	struct max16150_device *max16150;
	struct device *dev = &pdev->dev;
	int err, irq, ret;
	u32 keycode;

	chip_info = device_get_match_data(dev);
	if (!chip_info)
		return -EINVAL;

	max16150 = devm_kzalloc(dev, sizeof(*max16150), GFP_KERNEL);
	if (!max16150)
		return -ENOMEM;

	max16150->chip_info = chip_info;

	max16150->input = devm_input_allocate_device(dev);
	if (!max16150->input)
		return -ENOMEM;

	max16150->input->name = "MAX16150 Pushbutton";
	max16150->input->phys = "max16150/input0";
	max16150->input->id.bustype = BUS_HOST;

	keycode = KEY_POWER;
	ret = device_property_read_u32(dev, "linux,code", &keycode);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get keycode\n");

	max16150->keycode = keycode;

	input_set_capability(max16150->input, EV_KEY, max16150->keycode);

	max16150->gpiod = devm_gpiod_get(dev, "interrupt", GPIOD_IN);
	if (IS_ERR(max16150->gpiod))
		return dev_err_probe(dev, PTR_ERR(max16150->gpiod),
				     "Failed to get interrupt GPIO\n");

	if (chip_info->has_clr_gpio) {
		max16150->clr_gpiod = devm_gpiod_get(dev, "clr", GPIOD_OUT_HIGH);
		if (IS_ERR(max16150->clr_gpiod))
			return dev_err_probe(dev, PTR_ERR(max16150->clr_gpiod),
					     "Failed to get clr GPIO\n");

		if (!max16150->clr_gpiod)
			return dev_err_probe(dev, -ENODEV,
						 "clr GPIO is mandatory\n");

		if (max16150->clr_gpiod) {
			fsleep(1000);
			gpiod_set_value(max16150->clr_gpiod, 0);
		}
	}

	irq = gpiod_to_irq(max16150->gpiod);
	if (irq < 0)
		return dev_err_probe(dev, irq,
				     "MAX16150: Failed to map GPIO to IRQ");

	err = devm_request_irq(dev, irq, max16150_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "max16150_irq", max16150);
	if (err)
		return err;

	return input_register_device(max16150->input);
}

static const struct of_device_id max16150_of_match[] = {
	{ .compatible = "adi,max16150a", .data = &max16150_variant_a },
	{ .compatible = "adi,max16150b", .data = &max16150_variant_b },
	{ .compatible = "adi,max16169a", .data = &max16150_variant_a },
	{ .compatible = "adi,max16169b", .data = &max16150_variant_b },
	{ }
};
MODULE_DEVICE_TABLE(of, max16150_of_match);

static struct platform_driver max16150_driver = {
	.probe  = max16150_probe,
	.driver = {
		.name = "max16150",
		.of_match_table = max16150_of_match,
	},
};
module_platform_driver(max16150_driver);

MODULE_AUTHOR("Marc Paolo Sosa <marcpaolo.sosa@analog.com>");
MODULE_DESCRIPTION("MAX16150/MAX16169 Pushbutton Driver");
MODULE_LICENSE("GPL");
