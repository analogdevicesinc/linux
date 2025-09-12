// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices adau1962 codec driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "adau1962.h"

static int adau1962_i2c_probe(struct i2c_client *client)
{
	struct regmap_config config;

	config = adau1962_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 8;

	return adau1962_probe(&client->dev,
		devm_regmap_init_i2c(client, &config), NULL);
}

static void adau1962_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	adau1962_remove(&client->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id adau1962_dt_ids[] = {
	{ .compatible = "adi,adau1962", },
	{ }
};
MODULE_DEVICE_TABLE(of, adau1962_dt_ids);
#endif

static const struct i2c_device_id adau1962_i2c_ids[] = {
	{"adau1962", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1962_i2c_ids);

static struct i2c_driver adau1962_i2c_driver = {
	.driver = {
		.name = "adau1962",
		.of_match_table = of_match_ptr(adau1962_dt_ids),
	},
	.probe = adau1962_i2c_probe,
	.remove = adau1962_i2c_remove,
	.id_table = adau1962_i2c_ids,
};
module_i2c_driver(adau1962_i2c_driver);

MODULE_DESCRIPTION("Analog Devices ADAU1962 driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
