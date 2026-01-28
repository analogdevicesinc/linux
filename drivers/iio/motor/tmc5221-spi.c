// SPDX-License-Identifier: GPL-2.0
/*
 * TMC5221 Stepper Motor Controller Driver - SPI Interface
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/of.h>

#include "tmc5222.h"

static const struct regmap_config tmc5221_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.max_register = 0xFF,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0x00,
	.write_flag_mask = 0x80,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int tmc5221_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	int ret;

	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "SPI setup failed: %d\n", ret);
		return ret;
	}

	regmap = devm_regmap_init_spi(spi, &tmc5221_spi_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	ret = tmc5222_probe_common(&spi->dev, regmap);
	if (ret)
		dev_err(&spi->dev, "Common probe failed: %d\n", ret);

	return ret;
}

static void tmc5221_spi_remove(struct spi_device *spi)
{
	tmc5222_remove_common(&spi->dev);
}

static const struct spi_device_id tmc5221_spi_id[] = {
	{ "tmc5221", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, tmc5221_spi_id);

static const struct of_device_id tmc5221_spi_of_match[] = {
	{ .compatible = "adi,tmc5221" },
	{ }
};
MODULE_DEVICE_TABLE(of, tmc5221_spi_of_match);

static struct spi_driver tmc5221_spi_driver = {
	.driver = {
		.name = "tmc5221",
		.of_match_table = tmc5221_spi_of_match,
	},
	.probe = tmc5221_spi_probe,
	.remove = tmc5221_spi_remove,
	.id_table = tmc5221_spi_id,
};
module_spi_driver(tmc5221_spi_driver);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("TMC5221 Stepper Motor Controller SPI Driver");
MODULE_LICENSE("GPL");
