// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADPD188 I2C driver
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * 7-bit I2C slave address: 0x64 
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "adpd188.h"

const struct regmap_config adpd188_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x7F,
};

static int adpd188_i2c_probe(struct i2c_client *i2c)
{
	//const struct i2c_device_id *id;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(i2c, &adpd188_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&i2c->dev, "Error initializing i2c regmap: %ld\n",
                        PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return adpd188_core_probe(&i2c->dev, regmap, i2c->name);
}

static int adpd188_i2c_remove(struct i2c_client *client)
{
        return adpd188_core_remove(&client->dev);
}

static const struct i2c_device_id adpd188_i2c_id[] = {
	{ "adpd188", ADPD188 },
        {}
};
MODULE_DEVICE_TABLE(i2c, adpd188_i2c_id);

static const struct of_device_id adpd188_of_match[] = {
	{ .compatible = "adi,adpd188" },
	{},
};
MODULE_DEVICE_TABLE(of, adpd188_of_match);

static struct i2c_driver adpd188_i2c_driver = {
	.driver = {
			.name = "adpd188_i2c",
			.of_match_table = of_match_ptr(adpd188_of_match),
		},
	.probe_new = adpd188_i2c_probe,
	.remove = adpd188_i2c_remove,
	.id_table = adpd188_i2c_id,
};
module_i2c_driver(adpd188_i2c_driver);

MODULE_AUTHOR("Andrei Drimbarean <andrei.drimbarean@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADPD188 I2C driver");
MODULE_LICENSE("GPL v2");

