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
	struct adau18x0 *adau = dev->driver_data;
	uint32_t i, val;
	/*
	 * To get the device into SPI mode SS has to be pulled low three
	 * times.  Do this by issuing three dummy reads.
	 */
	regcache_cache_bypass(adau->regmap, true);
	for (i=0; i<5; i++) {
		regmap_read(adau->regmap, ADAU1860_VENDOR_ID, &val);
		if (val == 0x41) {
			dev_dbg(dev, "SPI mode engaged: %x", val);
			break;
		}
	}
	regcache_cache_bypass(adau->regmap, false);
}

static int adau1860_spi_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message	message;
	struct spi_transfer	t[2];
	uint8_t rw_byte = 0x0;

	spi_message_init(&message);
	memset(t, 0, sizeof(t));

	t[0].len = sizeof(uint8_t);
	t[0].tx_buf = &rw_byte;
	spi_message_add_tail(&t[0], &message);

	t[1].len = count;
	t[1].tx_buf = data;
	spi_message_add_tail(&t[1], &message);

	/* Do the i/o */
	return spi_sync(spi, &message);
}

static int adau1860_spi_read(void *context, const void *reg_buf, size_t reg_size,
							  void *val_buf, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message	message;
	struct spi_transfer	t[3];
	uint8_t rw_byte = 0x1;

	spi_message_init(&message);
	memset(t, 0, sizeof(t));

	t[0].len = sizeof(uint8_t);
	t[0].tx_buf = &rw_byte;
	spi_message_add_tail(&t[0], &message);

	t[1].len = reg_size;
	t[1].tx_buf = reg_buf;
	spi_message_add_tail(&t[1], &message);

	t[2].len = val_size;
	t[2].rx_buf = val_buf;
	spi_message_add_tail(&t[2], &message);

	/* Do the i/o */
	return spi_sync(spi, &message);
}

static struct regmap_bus adau1860_spi_bus_config = {
	.write = adau1860_spi_write,
	.read = adau1860_spi_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static int adau1860_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap_config config;

	if (!id)
		return -EINVAL;

	config = adau1860_regmap_config;
	config.pad_bits = 8;
	return adau1860_probe(&spi->dev, devm_regmap_init(&spi->dev, &adau1860_spi_bus_config, &spi->dev, &config),
			      id->driver_data, adau1860_spi_switch_mode);
}

static int adau1860_spi_remove(struct spi_device *spi)
{
	return 0;
}

static const struct spi_device_id adau1860_spi_id[] = { 
	{ "adau1860", ADAU1860 },
	{}
};
MODULE_DEVICE_TABLE(spi, adau1860_spi_id);

#if defined(CONFIG_OF)
static const struct of_device_id adau1860_spi_dt_ids[] = {
	{ .compatible = "adi,adau1860", },
	{},
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
