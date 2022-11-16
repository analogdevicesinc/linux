// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADAU1977/ADAU1978/ADAU1979 driver
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "adau1977.h"

static int adau1977_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct regmap_config config;

	config = adau1977_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 8;

	return adau1977_probe(&client->dev,
		devm_regmap_init_i2c(client, &config),
		id->driver_data, NULL);
}

static int adau1977_i2c_remove(struct i2c_client *client)
{
	// DBB snd_soc_unregister_ card codec (&client->dev);
	adau1977_remove(&client->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id adau1977_dt_ids[] = {
	{ .compatible = "adi,adau1977", .data = (const void *)ADAU1977 },
	{ .compatible = "adi,adau1978", .data = (const void *)ADAU1978 },
	{ .compatible = "adi,adau1979", .data = (const void *)ADAU1979 },
	{ }
};
MODULE_DEVICE_TABLE(of, adau1977_dt_ids);
#endif

static const struct i2c_device_id adau1977_i2c_ids[] = {
	{ "adau1977", ADAU1977 },
	{ "adau1978", ADAU1978 },
	{ "adau1979", ADAU1979 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1977_i2c_ids);

static struct i2c_driver adau1977_i2c_driver = {
	.driver = {
		.name = "adau1977",
		.of_match_table = of_match_ptr(adau1977_dt_ids),
	},
	.probe = adau1977_i2c_probe,
	.remove = adau1977_i2c_remove,
	.id_table = adau1977_i2c_ids,
};
module_i2c_driver(adau1977_i2c_driver);

MODULE_DESCRIPTION("ASoC ADAU1977/ADAU1978/ADAU1979 driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
