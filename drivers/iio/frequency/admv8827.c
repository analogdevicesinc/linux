// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADMV8827
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>

struct admv8827_priv {
	struct spi_device	*spi;
	struct gpio_desc	*cen_gpio;
	struct gpio_desc	*reset_gpio;
	struct regmap		*regmap;
};

static int admv8827_debug_reg_access(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int write_val, unsigned int *read_val)
{
	struct admv8827_priv *priv = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(priv->regmap, reg, read_val);
	else
		return regmap_write(priv->regmap, reg, write_val);
}

static const struct iio_info admv8827_info = {
	.debugfs_reg_access = &admv8827_debug_reg_access,
};

static const struct regmap_config admv8827_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int admv8827_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admv8827_priv *priv;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);

	priv->spi = spi;

	priv->cen_gpio = devm_gpiod_get_optional(&spi->dev,
						 "chip-enable",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(priv->cen_gpio))
		return PTR_ERR(priv->cen_gpio);

	priv->reset_gpio = devm_gpiod_get_optional(&spi->dev,
						  "reset",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	priv->regmap = devm_regmap_init_spi(spi, &admv8827_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	//indio_dev->name = "admv8827";
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->info = &admv8827_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver admv8827_driver = {
	.driver = {
		.name = "admv8827",
	},
	.probe = admv8827_probe,
};
module_spi_driver(admv8827_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("ADMV8827");
MODULE_LICENSE("GPL v2");
