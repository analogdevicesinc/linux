// SPDX-License-Identifier: GPL-2.0
/*
 * Generic IIO access driver via I2C
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "iio-regmap.h"

static int iio_regmap_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct regmap *regmap;
	int ret;
	struct regmap_config *regmap_cfg;

	regmap_cfg = devm_kzalloc(&client->dev, sizeof(*regmap_cfg),
				  GFP_KERNEL);
	if (!regmap_cfg)
		return -ENOMEM;

	ret = config_regmap(&client->dev, regmap_cfg);
	if (ret < 0)
		return ret;

	regmap = devm_regmap_init_i2c(client, regmap_cfg);
	if (!regmap) {
		dev_err(&client->dev, "devm_regmap_init_i2c failed!\n");
		return -1;
	}

	return iio_regmap_probe(&client->dev, regmap, client->name);
}

static const struct i2c_device_id iio_regmap_i2c_id[] = {
	{
		.name = "iio-regmap-i2c",
	},
	{}
};

static struct i2c_driver iio_regmap_i2c_driver = {
	.driver = {
		.name	= "iio-regmap-i2c",
	},
	.probe	       = iio_regmap_i2c_probe,
	.id_table      = iio_regmap_i2c_id
};

module_i2c_driver(iio_regmap_i2c_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("IIO Regmap I2C");
MODULE_LICENSE("GPL v2");
