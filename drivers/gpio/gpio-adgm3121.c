// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADI ADGM3121/ADGM3144 MEMS RF Switch GPIO Driver
 *
 * Copyright 2025 Analog Devices Inc.
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

/* ADGM3121 Register Addresses */
#define ADGM3121_REG_SWITCH_DATA	0x20

/* ADGM3121 Register Bits */
#define ADGM3121_SW1_EN			BIT(0)
#define ADGM3121_SW2_EN			BIT(1)
#define ADGM3121_SW3_EN			BIT(2)
#define ADGM3121_SW4_EN			BIT(3)
#define ADGM3121_INTERNAL_ERROR		GENMASK(7, 6)
#define ADGM3121_RESERVED		GENMASK(5, 4)

/* SPI Command Format */
#define ADGM3121_SPI_READ		BIT(15)
#define ADGM3121_SPI_WRITE		0
#define ADGM3121_SPI_ADDR_MSK		GENMASK(14, 8)
#define ADGM3121_SPI_DAISY_CHAIN_CMD	0x2500

/* Default timeout values */
#define ADGM3121_SWITCHING_TIME_US	200
#define ADGM3121_POWER_UP_TIME_MS	45

/* Number of GPIOs */
#define ADGM3121_NUM_GPIOS		4

/**
 * enum adgm3121_mode - Control mode enumeration
 * @ADGM3121_MODE_PARALLEL: GPIO parallel control mode
 * @ADGM3121_MODE_SPI: SPI control mode
 */
enum adgm3121_mode {
	ADGM3121_MODE_PARALLEL,
	ADGM3121_MODE_SPI,
};

/**
 * struct adgm3121_gpio - Device structure
 * @chip: GPIO chip structure
 * @spi: SPI device (SPI mode only)
 * @mode: Control mode (parallel or SPI)
 * @lock: Mutex for synchronization
 * @pin_spi_gpio: PIN/SPI mode select GPIO
 * @in_gpios: Array of GPIO descriptors for parallel mode
 * @switch_states: Current switch states cache
 */
struct adgm3121_gpio {
	struct gpio_chip chip;
	struct spi_device *spi;
	enum adgm3121_mode mode;
	struct mutex lock;
	struct gpio_desc *pin_spi_gpio;
	struct gpio_desc *in_gpios[ADGM3121_NUM_GPIOS];
	u8 switch_states;
};

/**
 * adgm3121_spi_write_register - Write to device register via SPI
 * @adgm3121: The device structure
 * @reg_addr: Register address
 * @data: Data to write
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgm3121_spi_write_register(struct adgm3121_gpio *adgm3121,
				       u8 reg_addr, u8 data)
{
	u8 buf[2];
	u16 cmd;

	if (adgm3121->mode != ADGM3121_MODE_SPI || !adgm3121->spi)
		return -EINVAL;

	/* Build 16-bit SPI command: [RW][ADDR][DATA] */
	cmd = ADGM3121_SPI_WRITE |
	      FIELD_PREP(ADGM3121_SPI_ADDR_MSK, reg_addr) |
	      data;

	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;

	return spi_write(adgm3121->spi, buf, 2);
}

/**
 * adgm3121_spi_read_register - Read from device register via SPI
 * @adgm3121: The device structure
 * @reg_addr: Register address
 * @data: Pointer to store read data
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgm3121_spi_read_register(struct adgm3121_gpio *adgm3121,
				      u8 reg_addr, u8 *data)
{
	u8 buf[2];
	u16 cmd;
	int ret;

	if (adgm3121->mode != ADGM3121_MODE_SPI || !adgm3121->spi)
		return -EINVAL;

	/* Build 16-bit SPI command: [RW][ADDR][DUMMY] */
	cmd = ADGM3121_SPI_READ |
	      FIELD_PREP(ADGM3121_SPI_ADDR_MSK, reg_addr);

	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;

	ret = spi_write_then_read(adgm3121->spi, buf, 2, buf, 2);
	if (ret)
		return ret;

	*data = buf[1];

	return 0;
}

/**
 * adgm3121_gpio_get - Get GPIO value
 * @chip: GPIO chip
 * @offset: GPIO offset (0-3)
 *
 * Return: GPIO value (0 or 1), negative error code on failure
 */
static int adgm3121_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct adgm3121_gpio *adgm3121 = gpiochip_get_data(chip);
	int ret;

	if (offset >= ADGM3121_NUM_GPIOS)
		return -EINVAL;

	mutex_lock(&adgm3121->lock);

	if (adgm3121->mode == ADGM3121_MODE_PARALLEL) {
		if (!adgm3121->in_gpios[offset]) {
			ret = -EINVAL;
			goto unlock;
		}
		ret = gpiod_get_value(adgm3121->in_gpios[offset]);
	} else {
		u8 switch_data;

		ret = adgm3121_spi_read_register(adgm3121,
						 ADGM3121_REG_SWITCH_DATA,
						 &switch_data);
		if (ret)
			goto unlock;

		adgm3121->switch_states = switch_data & 0x0F;
		ret = !!(switch_data & BIT(offset));
	}

unlock:
	mutex_unlock(&adgm3121->lock);
	return ret;
}

/**
 * adgm3121_gpio_set - Set GPIO value
 * @chip: GPIO chip
 * @offset: GPIO offset (0-3)
 * @value: Value to set (0 or 1)
 */
static void adgm3121_gpio_set(struct gpio_chip *chip, unsigned int offset,
			      int value)
{
	struct adgm3121_gpio *adgm3121 = gpiochip_get_data(chip);
	int ret;

	if (offset >= ADGM3121_NUM_GPIOS)
		return;

	mutex_lock(&adgm3121->lock);

	if (adgm3121->mode == ADGM3121_MODE_PARALLEL) {
		if (adgm3121->in_gpios[offset])
			gpiod_set_value(adgm3121->in_gpios[offset], value);
	} else {
		u8 switch_data;

		if (value)
			switch_data = adgm3121->switch_states | BIT(offset);
		else
			switch_data = adgm3121->switch_states & ~BIT(offset);

		ret = adgm3121_spi_write_register(adgm3121,
						  ADGM3121_REG_SWITCH_DATA,
						  switch_data);
		if (!ret) {
			adgm3121->switch_states = switch_data;
			/* Wait for switching time */
			usleep_range(ADGM3121_SWITCHING_TIME_US,
				     ADGM3121_SWITCHING_TIME_US + 50);
		}
	}

	mutex_unlock(&adgm3121->lock);
}

/**
 * adgm3121_gpio_direction_input - Set GPIO direction to input
 * @chip: GPIO chip
 * @offset: GPIO offset (0-3)
 *
 * Return: Always -EINVAL (switches are output-only)
 */
static int adgm3121_gpio_direction_input(struct gpio_chip *chip,
					 unsigned int offset)
{
	return -EINVAL; /* Switches are output-only */
}

/**
 * adgm3121_gpio_direction_output - Set GPIO direction to output
 * @chip: GPIO chip
 * @offset: GPIO offset (0-3)
 * @value: Initial value to set
 *
 * Return: 0 on success
 */
static int adgm3121_gpio_direction_output(struct gpio_chip *chip,
					  unsigned int offset, int value)
{
	adgm3121_gpio_set(chip, offset, value);
	return 0;
}

/**
 * adgm3121_gpio_get_direction - Get GPIO direction
 * @chip: GPIO chip
 * @offset: GPIO offset (0-3)
 *
 * Return: Always GPIO_LINE_DIRECTION_OUT (switches are output-only)
 */
static int adgm3121_gpio_get_direction(struct gpio_chip *chip,
				       unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

/**
 * adgm3121_reset_switches - Reset all switches to off state
 * @adgm3121: The device structure
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgm3121_reset_switches(struct adgm3121_gpio *adgm3121)
{
	int i, ret = 0;

	mutex_lock(&adgm3121->lock);

	if (adgm3121->mode == ADGM3121_MODE_PARALLEL) {
		for (i = 0; i < ADGM3121_NUM_GPIOS; i++) {
			if (adgm3121->in_gpios[i])
				gpiod_set_value(adgm3121->in_gpios[i], 0);
		}
	} else {
		ret = adgm3121_spi_write_register(adgm3121,
						  ADGM3121_REG_SWITCH_DATA,
						  0x00);
		if (!ret) {
			adgm3121->switch_states = 0x00;
			usleep_range(ADGM3121_SWITCHING_TIME_US,
				     ADGM3121_SWITCHING_TIME_US + 50);
		}
	}

	mutex_unlock(&adgm3121->lock);
	return ret;
}

/**
 * adgm3121_spi_probe - SPI probe function
 * @spi: SPI device
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgm3121_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adgm3121_gpio *adgm3121;
	int ret;

	adgm3121 = devm_kzalloc(dev, sizeof(*adgm3121), GFP_KERNEL);
	if (!adgm3121)
		return -ENOMEM;

	adgm3121->spi = spi;
	adgm3121->mode = ADGM3121_MODE_SPI;
	adgm3121->switch_states = 0;

	mutex_init(&adgm3121->lock);

	/* Get PIN/SPI mode select GPIO */
	adgm3121->pin_spi_gpio = devm_gpiod_get(dev, "pin-spi", GPIOD_OUT_HIGH);
	if (IS_ERR(adgm3121->pin_spi_gpio)) {
		ret = PTR_ERR(adgm3121->pin_spi_gpio);
		dev_err(dev, "Failed to get pin-spi GPIO: %d\n", ret);
		return ret;
	}

	/* Wait for power-up time */
	msleep(ADGM3121_POWER_UP_TIME_MS);

	/* Initialize GPIO chip */
	adgm3121->chip.label = "adgm3121";
	adgm3121->chip.parent = dev;
	adgm3121->chip.owner = THIS_MODULE;
	adgm3121->chip.base = -1;
	adgm3121->chip.ngpio = ADGM3121_NUM_GPIOS;
	adgm3121->chip.can_sleep = true;
	adgm3121->chip.get = adgm3121_gpio_get;
	adgm3121->chip.set = adgm3121_gpio_set;
	adgm3121->chip.direction_input = adgm3121_gpio_direction_input;
	adgm3121->chip.direction_output = adgm3121_gpio_direction_output;
	adgm3121->chip.get_direction = adgm3121_gpio_get_direction;

	/* Reset all switches to off state */
	ret = adgm3121_reset_switches(adgm3121);
	if (ret) {
		dev_err(dev, "Failed to reset switches: %d\n", ret);
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, &adgm3121->chip, adgm3121);
	if (ret) {
		dev_err(dev, "Failed to add GPIO chip: %d\n", ret);
		return ret;
	}

	spi_set_drvdata(spi, adgm3121);

	dev_info(dev, "ADGM3121 SPI GPIO driver initialized\n");

	return 0;
}

/**
 * adgm3121_platform_probe - Platform probe function
 * @pdev: Platform device
 *
 * Return: 0 on success, negative error code otherwise
 */
static int adgm3121_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adgm3121_gpio *adgm3121;
	int i, ret;

	adgm3121 = devm_kzalloc(dev, sizeof(*adgm3121), GFP_KERNEL);
	if (!adgm3121)
		return -ENOMEM;

	adgm3121->mode = ADGM3121_MODE_PARALLEL;
	adgm3121->switch_states = 0;

	mutex_init(&adgm3121->lock);

	/* Get PIN/SPI mode select GPIO */
	adgm3121->pin_spi_gpio = devm_gpiod_get(dev, "pin-spi", GPIOD_OUT_LOW);
	if (IS_ERR(adgm3121->pin_spi_gpio)) {
		ret = PTR_ERR(adgm3121->pin_spi_gpio);
		dev_err(dev, "Failed to get pin-spi GPIO: %d\n", ret);
		return ret;
	}

	/* Get IN1-IN4 GPIOs for parallel mode */
	for (i = 0; i < ADGM3121_NUM_GPIOS; i++) {
		char gpio_name[8];

		snprintf(gpio_name, sizeof(gpio_name), "in%d", i + 1);
		adgm3121->in_gpios[i] = devm_gpiod_get(dev, gpio_name,
						       GPIOD_OUT_LOW);
		if (IS_ERR(adgm3121->in_gpios[i])) {
			ret = PTR_ERR(adgm3121->in_gpios[i]);
			dev_err(dev, "Failed to get %s GPIO: %d\n",
				gpio_name, ret);
			return ret;
		}
	}

	/* Wait for power-up time */
	msleep(ADGM3121_POWER_UP_TIME_MS);

	/* Initialize GPIO chip */
	adgm3121->chip.label = "adgm3121";
	adgm3121->chip.parent = dev;
	adgm3121->chip.owner = THIS_MODULE;
	adgm3121->chip.base = -1;
	adgm3121->chip.ngpio = ADGM3121_NUM_GPIOS;
	adgm3121->chip.can_sleep = false;
	adgm3121->chip.get = adgm3121_gpio_get;
	adgm3121->chip.set = adgm3121_gpio_set;
	adgm3121->chip.direction_input = adgm3121_gpio_direction_input;
	adgm3121->chip.direction_output = adgm3121_gpio_direction_output;
	adgm3121->chip.get_direction = adgm3121_gpio_get_direction;

	/* Reset all switches to off state */
	ret = adgm3121_reset_switches(adgm3121);
	if (ret) {
		dev_err(dev, "Failed to reset switches: %d\n", ret);
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, &adgm3121->chip, adgm3121);
	if (ret) {
		dev_err(dev, "Failed to add GPIO chip: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, adgm3121);

	dev_info(dev, "ADGM3121 parallel GPIO driver initialized\n");

	return 0;
}

/* SPI driver */
static const struct of_device_id adgm3121_spi_of_match[] = {
	{ .compatible = "adi,adgm3121" },
	{ .compatible = "adi,adgm3144" },
	{ }
};
MODULE_DEVICE_TABLE(of, adgm3121_spi_of_match);

static const struct spi_device_id adgm3121_spi_id[] = {
	{ "adgm3121", 0 },
	{ "adgm3144", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, adgm3121_spi_id);

static struct spi_driver adgm3121_spi_driver = {
	.driver = {
		.name = "adgm3121",
		.of_match_table = adgm3121_spi_of_match,
	},
	.probe = adgm3121_spi_probe,
	.id_table = adgm3121_spi_id,
};

/* Platform driver */
static const struct of_device_id adgm3121_platform_of_match[] = {
	{ .compatible = "adi,adgm3121" },
	{ .compatible = "adi,adgm3144" },
	{ }
};
MODULE_DEVICE_TABLE(of, adgm3121_platform_of_match);

static struct platform_driver adgm3121_platform_driver = {
	.driver = {
		.name = "adgm3121",
		.of_match_table = adgm3121_platform_of_match,
	},
	.probe = adgm3121_platform_probe,
};

/**
 * adgm3121_init - Module initialization
 *
 * Return: 0 on success, negative error code otherwise
 */
static int __init adgm3121_init(void)
{
	int ret;

	ret = spi_register_driver(&adgm3121_spi_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&adgm3121_platform_driver);
	if (ret)
		spi_unregister_driver(&adgm3121_spi_driver);

	return ret;
}
module_init(adgm3121_init);

/**
 * adgm3121_exit - Module cleanup
 */
static void __exit adgm3121_exit(void)
{
	platform_driver_unregister(&adgm3121_platform_driver);
	spi_unregister_driver(&adgm3121_spi_driver);
}
module_exit(adgm3121_exit);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("ADI ADGM3121/ADGM3144 MEMS RF Switch GPIO Driver");
MODULE_LICENSE("GPL");