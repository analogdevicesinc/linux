// SPDX-License-Identifier: GPL-2.0
/*
 * ADG1414 SPST Switch Driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>

#define ADG1414_MAX_DEVICES		4

struct adg1414_state {
	struct spi_device		*spi;
	struct gpio_chip		chip;
	struct mutex			lock;
	u8				buffer[32];

	__be32				tx __aligned(ARCH_KMALLOC_MINALIGN);
};

static void adg1414_set(struct gpio_chip *chip,
			unsigned int offset,
			int value)
{
	struct adg1414_state *st = gpiochip_get_data(chip);
	u32 tx = 0;
	int i, ret;

	struct spi_transfer xfer = {
		.tx_buf = &st->tx,
		.len = st->chip.ngpio / 8,
	};

	mutex_lock(&st->lock);

	st->buffer[offset] = value;

	for (i = 0; i < st->chip.ngpio; i++)
		tx |= st->buffer[i] << i;

	st->tx = cpu_to_be32(tx << (32 - st->chip.ngpio));

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		goto out;

out:
	mutex_unlock(&st->lock);
}

static int adg1414_get(struct gpio_chip *chip,
		       unsigned int offset)
{
	struct adg1414_state *st = gpiochip_get_data(chip);
	int value;

	mutex_lock(&st->lock);

	value = st->buffer[offset];

	mutex_unlock(&st->lock);

	return value;
}

static int adg1414_get_direction(struct gpio_chip *chip,
				 unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int adg1414_probe(struct spi_device *spi)
{
	struct adg1414_state *st;
	struct gpio_desc *reset;
	struct device *dev = &spi->dev;
	u32 num_devices;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->spi = spi;

	/* Use reset pin to reset the device */
	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "Failed to get reset gpio");

	if (reset) {
		fsleep(0.015);
		gpiod_set_value_cansleep(reset, 0);
	}

	num_devices = 1;
	ret = device_property_read_u32(dev, "#daisy-chained-devices",
				       &num_devices);
	if (!ret) {
		if (num_devices > ADG1414_MAX_DEVICES || num_devices < 1)
			return dev_err_probe(dev, ret,
			       "Failed to get daisy-chained-devices property\n");
	}

	st->chip.label = "adg1414";
	st->chip.parent = dev;
	st->chip.get_direction = adg1414_get_direction;
	st->chip.set = adg1414_set;
	st->chip.get = adg1414_get;
	st->chip.base = -1;
	st->chip.ngpio =  num_devices * 8;
	st->chip.can_sleep = true;

	mutex_init(&st->lock);

	return devm_gpiochip_add_data(dev, &st->chip, st);
}

static const struct of_device_id adg1414_of_match[] = {
	{ .compatible = "adi,adg1414" },
	{ }
};
MODULE_DEVICE_TABLE(of, adg1414_of_match);

static struct spi_driver adg1414_driver = {
	.driver = {
		.name = "adg1414",
		.of_match_table = adg1414_of_match,
	},
	.probe = adg1414_probe,
};
module_spi_driver(adg1414_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("ADG1414 SPST Switch Driver");
MODULE_LICENSE("GPL");
