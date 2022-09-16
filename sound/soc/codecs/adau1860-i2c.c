// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for ADAU1860 codec
 *
 * Copyright 2022 Analog Devices Inc.
 * Author: Bogdan Togorean <bogdan.togorean@analog.com>
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "adau1860.h"

static const struct i2c_device_id adau1860_i2c_ids[];

static int adau1860_i2c_probe(struct i2c_client *client)
{
	struct regmap_config config;
	const struct i2c_device_id *id = i2c_match_id(adau1860_i2c_ids, client);

	if (!id)
		return -EINVAL;

	config = adau1860_regmap_config;

	return adau1860_probe(&client->dev,
		devm_regmap_init_i2c(client, &config),
		id->driver_data, NULL);
}

static int adau1860_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id adau1860_i2c_ids[] = {
	{ "adau1860", ADAU1860 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1860_i2c_ids);

#if defined(CONFIG_OF)
static const struct of_device_id adau1860_i2c_dt_ids[] = {
	{ .compatible = "adi,adau1860", },
	{ },
};
MODULE_DEVICE_TABLE(of, adau1860_i2c_dt_ids);
#endif

static struct i2c_driver adau1860_i2c_driver = {
	.driver = {
		.name = "adau1860",
		.of_match_table = of_match_ptr(adau1860_i2c_dt_ids),
	},
	.probe_new = adau1860_i2c_probe,
	.remove = adau1860_i2c_remove,
	.id_table = adau1860_i2c_ids,
};
module_i2c_driver(adau1860_i2c_driver);

MODULE_DESCRIPTION("ASoC ADAU1860 CODEC I2C driver");
MODULE_AUTHOR("Bogdan Togorean <bogdan.togorean@analog.com>");
MODULE_LICENSE("GPL");
