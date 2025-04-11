// SPDX-License-Identifier: GPL-2.0
/*
 * ADG1414 Serially-Controlled Octal SPST Switches Driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#define ADG1414_MAX_DEVICES 4

struct adg1414_state {
	struct spi_device *spi;
	struct gpio_chip chip;
	/* lock to protect against multiple access to the device and shared data */
	struct mutex lock;

	u8 buffer[];
};

static int adg1414_write_config(struct adg1414_state *st)
{
	return spi_write(st->spi, st->buffer, st->chip.ngpio / 8);
}

static int adg1414_get_value(struct gpio_chip *chip, unsigned int offset)
{
	struct adg1414_state *st = gpiochip_get_data(chip);
	u8 bank, pin;

	guard(mutex)(&st->lock);
	bank = (chip->ngpio / 8) - 1 - offset / 8;
	pin = offset % 8;

	return (st->buffer[bank] >> pin) & 0x1;
}

static void adg1414_set_value(struct gpio_chip *chip, unsigned int offset,
			      int val)
{
	struct adg1414_state *st = gpiochip_get_data(chip);
	u8 bank, pin;

	guard(mutex)(&st->lock);
	bank = (chip->ngpio / 8) - 1 - offset / 8;
	pin = offset % 8;

	if (val)
		st->buffer[bank] |= BIT(pin);
	else
		st->buffer[bank] &= ~BIT(pin);

	adg1414_write_config(st);
}

static void adg1414_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				 unsigned long *bits)
{
	struct adg1414_state *st = gpiochip_get_data(chip);
	unsigned long offset, bankmask, bitmask;
	size_t bank;

	guard(mutex)(&st->lock);
	for_each_set_clump8(offset, bankmask, mask, chip->ngpio) {
		bank = (chip->ngpio / 8) - 1 - offset / 8;
		bitmask = bitmap_get_value8(bits, offset) & bankmask;

		st->buffer[bank] &= ~bankmask;
		st->buffer[bank] |= bitmask;
	}

	adg1414_write_config(st);
}

static int adg1414_direction_output(struct gpio_chip *chip, unsigned int offset,
				    int val)
{
	adg1414_set_value(chip, offset, val);
	return 0;
}

static int adg1414_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adg1414_state *st;
	struct gpio_desc *reset;
	u32 num_devices;
	int ret;

	num_devices = 1;
	ret = device_property_read_u32(dev, "#daisy-chained-devices",
				       &num_devices);
	if (!ret) {
		if (!num_devices || num_devices > ADG1414_MAX_DEVICES)
			return dev_err_probe(dev, ret,
			       "Failed to get daisy-chained-devices property\n");
	}

	st = devm_kzalloc(dev, sizeof(*st) + num_devices, GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->spi = spi;

	/* Use reset pin to reset the device */
	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "Failed to get reset gpio");

	if (reset) {
		fsleep(1);
		gpiod_set_value_cansleep(reset, 0);
	}

	st->chip.label = "adg1414";
	st->chip.parent = dev;
	st->chip.direction_output = adg1414_direction_output;
	st->chip.set = adg1414_set_value;
	st->chip.get = adg1414_get_value;
	st->chip.set_multiple = adg1414_set_multiple;
	st->chip.base = -1;
	st->chip.ngpio =  num_devices * 8;
	st->chip.can_sleep = true;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	return devm_gpiochip_add_data(dev, &st->chip, st);
}

static const struct of_device_id adg1414_of_match[] = {
	{ .compatible = "adi,adg1414-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, adg1414_of_match);

static struct spi_driver adg1414_driver = {
	.driver = {
		.name = "adg1414-gpio",
		.of_match_table = adg1414_of_match,
	},
	.probe = adg1414_probe,
};
module_spi_driver(adg1414_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("ADG1414 Serially-Controlled Octal SPST Switches Driver");
MODULE_LICENSE("GPL");
