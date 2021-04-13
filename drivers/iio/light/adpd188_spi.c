// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADPD188 SPI driver
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * 7-bit I2C slave address: 0x64 
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/of.h>

#include "adpd188.h"

const struct regmap_config adpd188_spi_regmap_config = {
	.reg_bits = 7,
	.pad_bits = 1,
	.write_flag_mask = BIT(0),
	.val_bits = 16,
	.max_register = 0x7F,
};

static int adpd188_spi_probe(struct spi_device *spi)
{
        const struct spi_device_id *id = spi_get_device_id(spi);
        struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &adpd188_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return adpd188_core_probe(&spi->dev, regmap, id->name);
}

static int adpd188_spi_remove(struct spi_device *spi)
{
        return adpd188_core_remove(&spi->dev);
}

static const struct spi_device_id adpd188_spi_id[] = {
	{ "adpd188", ADPD188 },
        {}
};
MODULE_DEVICE_TABLE(spi, adpd188_spi_id);

static const struct of_device_id adpd188_of_match[] = {
	{ .compatible = "adi,adpd188" },
	{},
};
MODULE_DEVICE_TABLE(of, adpd188_of_match);

static struct spi_driver adpd188_spi_driver = {
	.driver = {
			.name = "adpd188_spi",
			.of_match_table = of_match_ptr(adpd188_of_match),
		},
	.probe = adpd188_spi_probe,
	.remove = adpd188_spi_remove,
	.id_table = adpd188_spi_id,
};
module_spi_driver(adpd188_spi_driver);

MODULE_AUTHOR("Andrei Drimbarean <andrei.drimbarean@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADPD188 SPI driver");
MODULE_LICENSE("GPL v2");

