// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal SPI driver template for Analog Devices devices with IIO support
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* Register definitions - replace with your device's registers */
#define ADI_DEV_REG_CHIP_ID		0x00
#define ADI_DEV_REG_CHIP_VERSION	0x01
#define ADI_DEV_REG_SPI_CONFIG		0x02
#define ADI_DEV_REG_STATUS		0x03

/* Expected chip ID - replace with your device's ID */
#define ADI_DEV_CHIP_ID			0x1234

/* SPI configuration bits */
#define ADI_DEV_SPI_SOFTRESET		BIT(7)
#define ADI_DEV_SPI_LSBFIRST		BIT(6)
#define ADI_DEV_SPI_ADDR_INC		BIT(5)

/* SPI transfer flags (for instruction byte) */
#define ADI_DEV_SPI_READ		BIT(7)
#define ADI_DEV_SPI_WRITE		0

struct adi_dev_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct gpio_desc	*reset_gpio;
	struct mutex		lock;
	u16			chip_id;
	u8			chip_version;
};

/*
 * Regmap configuration for SPI
 * Adjust reg_bits, val_bits, and flags based on your device's protocol
 */
static const struct regmap_config adi_dev_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.read_flag_mask = ADI_DEV_SPI_READ,
	.write_flag_mask = ADI_DEV_SPI_WRITE,
	.max_register = 0xFFFF,
};

static int adi_dev_reset(struct adi_dev_state *st)
{
	int ret;

	if (st->reset_gpio) {
		/* Hardware reset via GPIO */
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		usleep_range(10, 20);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
		usleep_range(1000, 2000);
	} else {
		/* Software reset via SPI register */
		ret = regmap_write(st->regmap, ADI_DEV_REG_SPI_CONFIG,
				   ADI_DEV_SPI_SOFTRESET);
		if (ret)
			return ret;
		usleep_range(1000, 2000);
	}

	return 0;
}

static int adi_dev_verify_chip_id(struct adi_dev_state *st)
{
	struct device *dev = &st->spi->dev;
	unsigned int id_lo, version;
	int ret;

	/*
	 * Read chip ID - adjust register addresses and bit manipulation
	 * based on your device's register layout
	 */
	ret = regmap_read(st->regmap, ADI_DEV_REG_CHIP_ID, &id_lo);
	if (ret)
		return ret;

	/* For 8-bit chip ID */
	st->chip_id = id_lo;

	ret = regmap_read(st->regmap, ADI_DEV_REG_CHIP_VERSION, &version);
	if (ret)
		return ret;

	st->chip_version = version;

	dev_info(dev, "Chip ID: 0x%04X, Version: 0x%02X\n",
		 st->chip_id, st->chip_version);

	/* Uncomment to enforce chip ID check */
	/*
	if (st->chip_id != ADI_DEV_CHIP_ID) {
		dev_err(dev, "Unrecognized chip ID: 0x%04X (expected 0x%04X)\n",
			st->chip_id, ADI_DEV_CHIP_ID);
		return -ENODEV;
	}
	*/

	return 0;
}

static int adi_dev_setup(struct adi_dev_state *st)
{
	int ret;

	ret = adi_dev_reset(st);
	if (ret) {
		dev_err(&st->spi->dev, "Reset failed: %d\n", ret);
		return ret;
	}

	ret = adi_dev_verify_chip_id(st);
	if (ret)
		return ret;

	/* Add device-specific initialization here */

	return 0;
}

/*
 * IIO debugfs register access
 * Access via: /sys/kernel/debug/iio/iio:deviceX/direct_reg_access
 * Read:  echo <reg> > direct_reg_access; cat direct_reg_access
 * Write: echo <reg> <val> > direct_reg_access
 */
static int adi_dev_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg, unsigned int writeval,
			      unsigned int *readval)
{
	struct adi_dev_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	if (readval)
		ret = regmap_read(st->regmap, reg, readval);
	else
		ret = regmap_write(st->regmap, reg, writeval);

	mutex_unlock(&st->lock);

	return ret;
}

/*
 * IIO channel definitions - customize for your device
 * This minimal example has no actual channels, only debugfs register access
 */
static const struct iio_chan_spec adi_dev_channels[] = {
	/* Add channels here if needed, e.g.:
	 * {
	 *     .type = IIO_VOLTAGE,
	 *     .indexed = 1,
	 *     .channel = 0,
	 *     .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	 * },
	 */
};

static int adi_dev_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	/* struct adi_dev_state *st = iio_priv(indio_dev); */

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		/* Add raw value reading here */
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static int adi_dev_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	/* struct adi_dev_state *st = iio_priv(indio_dev); */

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		/* Add raw value writing here */
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static const struct iio_info adi_dev_iio_info = {
	.read_raw = adi_dev_read_raw,
	.write_raw = adi_dev_write_raw,
	.debugfs_reg_access = adi_dev_reg_access,
};

static int adi_dev_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct adi_dev_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	mutex_init(&st->lock);
	spi_set_drvdata(spi, indio_dev);

	/* Optional reset GPIO */
	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	/* Initialize regmap for SPI access */
	st->regmap = devm_regmap_init_spi(spi, &adi_dev_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	ret = adi_dev_setup(st);
	if (ret)
		return ret;

	indio_dev->name = spi->dev.of_node ?
			  spi->dev.of_node->name : spi_get_device_id(spi)->name;
	indio_dev->info = &adi_dev_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adi_dev_channels;
	indio_dev->num_channels = ARRAY_SIZE(adi_dev_channels);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register IIO device\n");

	dev_info(dev, "Probed successfully\n");

	return 0;
}

static const struct of_device_id adi_dev_of_match[] = {
	{ .compatible = "adi,adi-spi-template" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_dev_of_match);

static const struct spi_device_id adi_dev_id_table[] = {
	{ "adi-spi-template", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, adi_dev_id_table);

static struct spi_driver adi_dev_driver = {
	.driver = {
		.name = "adi-spi-template",
		.of_match_table = adi_dev_of_match,
	},
	.probe = adi_dev_probe,
	.id_table = adi_dev_id_table,
};
module_spi_driver(adi_dev_driver);

MODULE_AUTHOR("Your Name <your.email@analog.com>");
MODULE_DESCRIPTION("Analog Devices SPI Template Driver");
MODULE_LICENSE("GPL");
