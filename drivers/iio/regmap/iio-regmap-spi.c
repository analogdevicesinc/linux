// SPDX-License-Identifier: GPL-2.0
/*
 * Generic IIO access driver via SPI
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "iio-regmap.h"

static int iio_regmap_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap_config *regmap_cfg;
	struct regmap *regmap;

	regmap_cfg = iio_regmap_read_config(&spi->dev);
	if (IS_ERR(regmap_cfg)) {
		dev_err(&spi->dev, "Reading regmap config failed!\n");
		return PTR_ERR(regmap_cfg);
	}

	regmap = devm_regmap_init_spi(spi, regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "devm_regmap_init_spi failed!\n");
		return PTR_ERR(regmap);
	}

	return iio_regmap_probe(&spi->dev, regmap, id->name);
}

static const struct spi_device_id iio_regmap_spi_id[] = {
	{
		.name = "iio-regmap-spi",
	},
	{}
};

static struct spi_driver iio_regmap_spi_driver = {
	.driver = {
		.name	= "iio-regmap-spi",
	},
	.probe	       = iio_regmap_spi_probe,
	.id_table      = iio_regmap_spi_id
};

module_spi_driver(iio_regmap_spi_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("IIO Regmap SPI");
MODULE_LICENSE("GPL v2");
