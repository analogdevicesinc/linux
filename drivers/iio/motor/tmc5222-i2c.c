// SPDX-License-Identifier: GPL-2.0
/*
 * TMC5222 Stepper Motor Controller Driver - I2C Interface
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of.h>

#include "tmc5222.h"

static const struct regmap_config tmc5222_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.max_register = 0xFF,
	.cache_type = REGCACHE_NONE,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int tmc5222_i2c_probe(struct i2c_client *client)
{
	struct regmap *regmap;
	int ret;

	dev_info(&client->dev, "TMC5222 I2C probe started\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C adapter doesn't support I2C_FUNC_I2C\n");
		return -EOPNOTSUPP;
	}

	regmap = devm_regmap_init_i2c(client, &tmc5222_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	ret = tmc5222_probe_common(&client->dev, regmap);
	if (ret)
		dev_err(&client->dev, "Common probe failed: %d\n", ret);

	return ret;
}

static void tmc5222_i2c_remove(struct i2c_client *client)
{
	tmc5222_remove_common(&client->dev);
}

static const struct i2c_device_id tmc5222_i2c_id[] = {
	{ "tmc5222", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmc5222_i2c_id);

static const struct of_device_id tmc5222_i2c_of_match[] = {
	{ .compatible = "adi,tmc5222" },
	{ }
};
MODULE_DEVICE_TABLE(of, tmc5222_i2c_of_match);

static struct i2c_driver tmc5222_i2c_driver = {
	.driver = {
		.name = "tmc5222",
		.of_match_table = tmc5222_i2c_of_match,
	},
	.probe = tmc5222_i2c_probe,
	.remove = tmc5222_i2c_remove,
	.id_table = tmc5222_i2c_id,
};
module_i2c_driver(tmc5222_i2c_driver);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("TMC5222 Stepper Motor Controller I2C Driver");
MODULE_LICENSE("GPL");
