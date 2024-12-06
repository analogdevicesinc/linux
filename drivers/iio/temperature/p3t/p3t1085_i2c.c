// SPDX-License-Identifier: GPL-2.0-only
/*
 * NXP P3T1085 Temperature Sensor Driver
 *
 * Copyright 2024 NXP
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include "p3t1085.h"

static const struct regmap_config p3t1085_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
};

static int p3t1085_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id;
	struct regmap *regmap;

	id = i2c_client_get_device_id(client);
	if (!id)
		return -EINVAL;

	regmap = devm_regmap_init_i2c(client, &p3t1085_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap %ld\n", PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	dev_info(&client->dev, "registering %s temperature sensor", id->name);
	return p3t1085_probe(&client->dev, client->irq, id->driver_data, regmap);
}

static const struct of_device_id p3t1085_i2c_of_match[] = {
	{ .compatible = "nxp,p3t1085", },
	{ }
};
MODULE_DEVICE_TABLE(of, p3t1085_i2c_of_match);

static const struct i2c_device_id p3t1085_i2c_id_table[] = {
	{ "p3t1085", P3T1085_ID },
	{ }
};
MODULE_DEVICE_TABLE(i2c, p3t1085_i2c_id_table);

static struct i2c_driver p3t1085_driver = {
	.driver = {
		.name = "p3t1085_i2c",
		.of_match_table = p3t1085_i2c_of_match,
	},
	.probe = p3t1085_i2c_probe,
	.id_table = p3t1085_i2c_id_table,
};
module_i2c_driver(p3t1085_driver);

MODULE_AUTHOR("Xiaoning Wang <xiaoning.wang@nxp.com>");
MODULE_DESCRIPTION("NXP P3T1085 i2c driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_P3T1085);
