// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for ADAU1860 codec
 *
 * Copyright 2022 Analog Devices Inc.
 * Author: Bogdan Togorean <bogdan.togorean@analog.com>
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <sound/soc.h>

#include "adau1860.h"

static void adau1860_spi_switch_mode(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);

	/*
	 * To get the device into SPI mode CLATCH has to be pulled low three
	 * times.  Do this by issuing three dummy reads.
	 */
	spi_w8r8(spi, 0x00);
	spi_w8r8(spi, 0x00);
	spi_w8r8(spi, 0x00);
}

static int adau1860_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap_config config;

	if (!id)
		return -EINVAL;

	config = adau1860_regmap_config;
	config.read_flag_mask = 0x1;

	return adau1860_probe(&spi->dev,
		devm_regmap_init_spi(spi, &config),
		id->driver_data, adau1860_spi_switch_mode);
}

static void adau1860_spi_remove(struct spi_device *spi)
{
	return;
}

static const struct spi_device_id adau1860_spi_id[] = {
	{ "adau1860", ADAU1860 },
	{ }
};
MODULE_DEVICE_TABLE(spi, adau1860_spi_id);

#if defined(CONFIG_OF)
static const struct of_device_id adau1860_spi_dt_ids[] = {
	{ .compatible = "adi,adau1860", },
	{ },
};
MODULE_DEVICE_TABLE(of, adau1860_spi_dt_ids);
#endif

static struct spi_driver adau1860_spi_driver = {
	.driver = {
		.name = "adau1860",
		.of_match_table = of_match_ptr(adau1860_spi_dt_ids),
	},
	.probe = adau1860_spi_probe,
	.remove = adau1860_spi_remove,
	.id_table = adau1860_spi_id,
};
module_spi_driver(adau1860_spi_driver);

MODULE_DESCRIPTION("ASoC ADAU1860 CODEC SPI driver");
MODULE_AUTHOR("Bogdan Togorean <bogdan.togorean@analog.com>");
MODULE_LICENSE("GPL");
