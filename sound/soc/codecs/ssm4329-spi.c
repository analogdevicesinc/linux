/*
 * SSM4329 SPI audio driver
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include <sound/soc.h>

#include "ssm4329.h"

static int ssm4329_spi_probe(struct spi_device *spi)
{
	struct regmap_config config;

	config = ssm4329_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 24;
	config.read_flag_mask = 0x01;

	return ssm4329_probe(&spi->dev,
		devm_regmap_init_spi(spi, &ssm4329_regmap_config));
}

static int ssm4329_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	return 0;
}

static const struct of_device_id ssm4329_of_ids[] = {
	{ .compatible = "adi,ssm4329", },
	{ },
};
MODULE_DEVICE_TABLE(of, ssm4329_of_ids);

static struct spi_driver ssm4329_spi_driver = {
	.driver = {
		.name = "ssm4329",
		.owner = THIS_MODULE,
		.of_match_table = ssm4329_of_ids,
	},
	.probe = ssm4329_spi_probe,
	.remove = ssm4329_spi_remove,
};
module_spi_driver(ssm4329_spi_driver);

MODULE_DESCRIPTION("ASoC SSM4329 SPI driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");
