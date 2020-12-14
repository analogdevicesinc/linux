// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include "clk-ad9545.h"
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#define AD9545_CONFIG_0			0x0000

#define AD9545_4WIRE_SPI		0x3
#define AD9545_4WIRE_SPI_MSK		GENMASK(4, 3)

static const struct regmap_config ad9545_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x3A3B,
	.use_single_rw = true,
};

static int ad9545_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_spi(spi, &ad9545_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "devm_regmap_init_spi failed!\n");
		return PTR_ERR(regmap);
	}

	if (!(spi->mode & SPI_3WIRE)) {
		ret = regmap_write(regmap, AD9545_CONFIG_0,
				   FIELD_PREP(AD9545_4WIRE_SPI_MSK, AD9545_4WIRE_SPI));
		if (ret < 0)
			return ret;
	}

	return ad9545_probe(&spi->dev, regmap);
}

static const struct of_device_id ad9545_spi_of_match[] = {
	{ .compatible = "adi,ad9545" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9545_spi_of_match);

static const struct spi_device_id ad9545_spi_id[] = {
	{"ad9545", 0},
	{ }
};
MODULE_DEVICE_TABLE(spi, ad9545_spi_id);

static struct spi_driver ad9545_spi_driver = {
	.driver = {
		.name	= "ad9545",
		.of_match_table = ad9545_spi_of_match,
	},
	.probe		= ad9545_spi_probe,
	.id_table	= ad9545_spi_id,
};
module_spi_driver(ad9545_spi_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9545 SPI");
MODULE_LICENSE("Dual BSD/GPL");
