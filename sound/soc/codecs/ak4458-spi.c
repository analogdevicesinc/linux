/*
 * ak4458-spi.c  --  AK4458 DAC - SPI
 *
 * Copyright 2017 NXP
 *
 * Author: Mihai Serban <mihai.serban@nxp.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "ak4458.h"

static int ak4458_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &ak4458_spi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return ak4458_probe(&spi->dev, regmap);
}

static int ak4458_spi_remove(struct spi_device *spi)
{
	ak4458_remove(&spi->dev);
	return 0;
}

static const struct of_device_id ak4458_of_match[] = {
	{ .compatible = "asahi-kasei,ak4458", },
	{ },
};
MODULE_DEVICE_TABLE(of, ak4458_of_match);

static struct spi_driver ak4458_spi_driver = {
	.driver = {
		.name = "ak4458",
		.pm = &ak4458_pm,
		.of_match_table = ak4458_of_match,
	},
	.probe = ak4458_spi_probe,
	.remove = ak4458_spi_remove
};

module_spi_driver(ak4458_spi_driver);

MODULE_DESCRIPTION("ASoC AK4458 driver - SPI");
MODULE_AUTHOR("Mihai Serban <mihai.serban@nxp.com>");
MODULE_LICENSE("GPL");
