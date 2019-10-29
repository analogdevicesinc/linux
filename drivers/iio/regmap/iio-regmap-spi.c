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

static const struct regmap_config iio_regmap_spi_regmap_config = {
};

static int iio_regmap_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_spi(spi, &iio_regmap_spi_regmap_config);
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
