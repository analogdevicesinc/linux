// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include "clk-ad9545.h"
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

static const struct regmap_config ad9545_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x3A3B,
	.use_single_rw = true,
};

static int ad9545_i2c_probe(struct i2c_client *client)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &ad9545_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "devm_regmap_init_i2c failed!\n");
		return PTR_ERR(regmap);
	}

	return ad9545_probe(&client->dev, regmap);
}

static const struct of_device_id ad9545_i2c_of_match[] = {
	{ .compatible = "adi,ad9545" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9545_i2c_of_match);

static const struct i2c_device_id ad9545_i2c_id[] = {
	{"ad9545"},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad9545_i2c_id);

static struct i2c_driver ad9545_i2c_driver = {
	.driver = {
		.name	= "ad9545",
		.of_match_table = ad9545_i2c_of_match,
	},
	.probe_new	= ad9545_i2c_probe,
	.id_table	= ad9545_i2c_id,
};
module_i2c_driver(ad9545_i2c_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9545 I2C");
MODULE_LICENSE("Dual BSD/GPL");
