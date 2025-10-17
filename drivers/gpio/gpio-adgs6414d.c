// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADI ADGS6414D Octal SPST Switch GPIO Driver
 *
 * Copyright 2025 Analog Devices Inc.
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
 */

#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

/* ADGS6414D Register Map */
#define ADGS6414D_REG_SW_DATA		0x01
#define ADGS6414D_REG_ERR_FLAGS		0x03
#define ADGS6414D_REG_BURST_EN		0x05
#define ADGS6414D_REG_SOFT_RESETB	0x0B

/* SPI Command Format */
#define ADGS6414D_CMD_WRITE		0x00
#define ADGS6414D_CMD_READ		0x80

/* Soft Reset Values */
#define ADGS6414D_SOFT_RESET_VAL1	0xA3
#define ADGS6414D_SOFT_RESET_VAL2	0x05

/* Number of switches */
#define ADGS6414D_NUM_SWITCHES		8

/**
 * struct adgs6414d_gpio - Device structure
 * @chip: GPIO chip structure
 * @spi: SPI device
 * @lock: Mutex for synchronization
 * @switch_state: Current switch states cache
 */
struct adgs6414d_gpio {
	struct gpio_chip chip;
	struct spi_device *spi;
	struct mutex lock;
	u8 switch_state;
};

/**
 * adgs6414d_spi_write - Write to device register via SPI
 * @adgs6414d: The device structure
 * @reg_addr: Register address
 * @data: Data to write
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgs6414d_spi_write(struct adgs6414d_gpio *adgs6414d,
			       u8 reg_addr, u8 data)
{
	u8 buf[2];

	buf[0] = ADGS6414D_CMD_WRITE | (reg_addr & 0x7F);
	buf[1] = data;

	return spi_write(adgs6414d->spi, buf, 2);
}

/**
 * adgs6414d_spi_read - Read from device register via SPI
 * @adgs6414d: The device structure
 * @reg_addr: Register address
 * @data: Pointer to store read data
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgs6414d_spi_read(struct adgs6414d_gpio *adgs6414d,
			      u8 reg_addr, u8 *data)
{
	u8 buf[2];
	int ret;

	buf[0] = ADGS6414D_CMD_READ | (reg_addr & 0x7F);
	buf[1] = 0x00;

	ret = spi_write_then_read(adgs6414d->spi, buf, 2, buf, 2);
	if (ret)
		return ret;

	*data = buf[1];

	return 0;
}

/**
 * adgs6414d_soft_reset - Perform software reset
 * @adgs6414d: The device structure
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgs6414d_soft_reset(struct adgs6414d_gpio *adgs6414d)
{
	int ret;

	ret = adgs6414d_spi_write(adgs6414d, ADGS6414D_REG_SOFT_RESETB,
				  ADGS6414D_SOFT_RESET_VAL1);
	if (ret)
		return ret;

	ret = adgs6414d_spi_write(adgs6414d, ADGS6414D_REG_SOFT_RESETB,
				  ADGS6414D_SOFT_RESET_VAL2);
	if (ret)
		return ret;

	adgs6414d->switch_state = 0x00;

	return 0;
}

/**
 * adgs6414d_gpio_get - Get GPIO value
 * @chip: GPIO chip
 * @offset: GPIO offset (0-7)
 *
 * Return: GPIO value (0 or 1), negative error code on failure
 */
static int adgs6414d_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct adgs6414d_gpio *adgs6414d = gpiochip_get_data(chip);
	int ret;

	if (offset >= ADGS6414D_NUM_SWITCHES)
		return -EINVAL;

	mutex_lock(&adgs6414d->lock);

	/* Read current switch states from device */
	ret = adgs6414d_spi_read(adgs6414d, ADGS6414D_REG_SW_DATA,
				 &adgs6414d->switch_state);
	if (ret) {
		mutex_unlock(&adgs6414d->lock);
		return ret;
	}

	ret = !!(adgs6414d->switch_state & BIT(offset));

	mutex_unlock(&adgs6414d->lock);
	return ret;
}

/**
 * adgs6414d_gpio_set - Set GPIO value
 * @chip: GPIO chip
 * @offset: GPIO offset (0-7)
 * @value: Value to set (0 or 1)
 */
static void adgs6414d_gpio_set(struct gpio_chip *chip, unsigned int offset,
			       int value)
{
	struct adgs6414d_gpio *adgs6414d = gpiochip_get_data(chip);
	u8 new_state;
	int ret;

	if (offset >= ADGS6414D_NUM_SWITCHES)
		return;

	mutex_lock(&adgs6414d->lock);

	if (value)
		new_state = adgs6414d->switch_state | BIT(offset);
	else
		new_state = adgs6414d->switch_state & ~BIT(offset);

	ret = adgs6414d_spi_write(adgs6414d, ADGS6414D_REG_SW_DATA, new_state);
	if (!ret)
		adgs6414d->switch_state = new_state;

	mutex_unlock(&adgs6414d->lock);
}

/**
 * adgs6414d_gpio_direction_input - Set GPIO direction to input
 * @chip: GPIO chip
 * @offset: GPIO offset (0-7)
 *
 * Return: Always -EINVAL (switches are output-only)
 */
static int adgs6414d_gpio_direction_input(struct gpio_chip *chip,
					  unsigned int offset)
{
	return -EINVAL; /* Switches are output-only */
}

/**
 * adgs6414d_gpio_direction_output - Set GPIO direction to output
 * @chip: GPIO chip
 * @offset: GPIO offset (0-7)
 * @value: Initial value to set
 *
 * Return: 0 on success
 */
static int adgs6414d_gpio_direction_output(struct gpio_chip *chip,
					   unsigned int offset, int value)
{
	adgs6414d_gpio_set(chip, offset, value);
	return 0;
}

/**
 * adgs6414d_gpio_get_direction - Get GPIO direction
 * @chip: GPIO chip
 * @offset: GPIO offset (0-7)
 *
 * Return: Always GPIO_LINE_DIRECTION_OUT (switches are output-only)
 */
static int adgs6414d_gpio_get_direction(struct gpio_chip *chip,
					unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

/**
 * adgs6414d_gpio_set_multiple - Set multiple GPIO values at once
 * @chip: GPIO chip
 * @mask: Bitmask of GPIOs to change
 * @bits: Values to set
 */
static void adgs6414d_gpio_set_multiple(struct gpio_chip *chip,
					unsigned long *mask,
					unsigned long *bits)
{
	struct adgs6414d_gpio *adgs6414d = gpiochip_get_data(chip);
	u8 new_state;
	int ret;

	mutex_lock(&adgs6414d->lock);

	new_state = adgs6414d->switch_state;
	new_state &= ~(*mask & 0xFF);
	new_state |= (*bits & *mask & 0xFF);

	ret = adgs6414d_spi_write(adgs6414d, ADGS6414D_REG_SW_DATA, new_state);
	if (!ret)
		adgs6414d->switch_state = new_state;

	mutex_unlock(&adgs6414d->lock);
}

/**
 * adgs6414d_probe - SPI probe function
 * @spi: SPI device
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgs6414d_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adgs6414d_gpio *adgs6414d;
	u8 initial_state = 0;
	int ret;

	adgs6414d = devm_kzalloc(dev, sizeof(*adgs6414d), GFP_KERNEL);
	if (!adgs6414d)
		return -ENOMEM;

	adgs6414d->spi = spi;
	mutex_init(&adgs6414d->lock);

	/* Get initial state from device tree */
	of_property_read_u8(dev->of_node, "initial-state", &initial_state);

	/* Perform soft reset */
	ret = adgs6414d_soft_reset(adgs6414d);
	if (ret) {
		dev_err(dev, "Failed to perform soft reset: %d\n", ret);
		return ret;
	}

	/* Set initial switch states */
	ret = adgs6414d_spi_write(adgs6414d, ADGS6414D_REG_SW_DATA,
				  initial_state);
	if (ret) {
		dev_err(dev, "Failed to set initial state: %d\n", ret);
		return ret;
	}

	adgs6414d->switch_state = initial_state;

	/* Initialize GPIO chip */
	adgs6414d->chip.label = "adgs6414d";
	adgs6414d->chip.parent = dev;
	adgs6414d->chip.owner = THIS_MODULE;
	adgs6414d->chip.base = -1;
	adgs6414d->chip.ngpio = ADGS6414D_NUM_SWITCHES;
	adgs6414d->chip.can_sleep = true;
	adgs6414d->chip.get = adgs6414d_gpio_get;
	adgs6414d->chip.set = adgs6414d_gpio_set;
	adgs6414d->chip.direction_input = adgs6414d_gpio_direction_input;
	adgs6414d->chip.direction_output = adgs6414d_gpio_direction_output;
	adgs6414d->chip.get_direction = adgs6414d_gpio_get_direction;
	adgs6414d->chip.set_multiple = adgs6414d_gpio_set_multiple;

	ret = devm_gpiochip_add_data(dev, &adgs6414d->chip, adgs6414d);
	if (ret) {
		dev_err(dev, "Failed to add GPIO chip: %d\n", ret);
		return ret;
	}

	spi_set_drvdata(spi, adgs6414d);

	dev_info(dev, "ADGS6414D GPIO driver initialized with %d switches\n",
		 ADGS6414D_NUM_SWITCHES);

	return 0;
}

static const struct of_device_id adgs6414d_of_match[] = {
	{ .compatible = "adi,adgs6414d" },
	{ }
};
MODULE_DEVICE_TABLE(of, adgs6414d_of_match);

static const struct spi_device_id adgs6414d_spi_id[] = {
	{ "adgs6414d", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, adgs6414d_spi_id);

static struct spi_driver adgs6414d_driver = {
	.driver = {
		.name = "adgs6414d",
		.of_match_table = adgs6414d_of_match,
	},
	.probe = adgs6414d_probe,
	.id_table = adgs6414d_spi_id,
};
module_spi_driver(adgs6414d_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("ADI ADGS6414D Octal SPST Switch GPIO Driver");
MODULE_LICENSE("GPL");