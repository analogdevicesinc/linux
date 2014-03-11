/*
 * SSM4329 driver
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "ssm4329.h"

static int ssm4329_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct regmap_config config;

	config = ssm4329_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 16;

	return ssm4329_probe(&client->dev,
		devm_regmap_init_i2c(client, &config));
}

static int ssm4329_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id ssm4329_i2c_ids[] = {
	{ "ssm4329", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ssm4329_i2c_ids);

static const struct of_device_id ssm4329_of_ids[] = {
	{ .compatible = "adi,ssm4329", },
	{ },
};
MODULE_DEVICE_TABLE(of, ssm4329_of_ids);

static struct i2c_driver ssm4329_i2c_driver = {
	.driver = {
		.name = "ssm4329",
		.owner = THIS_MODULE,
		.of_match_table = ssm4329_of_ids,
	},
	.probe = ssm4329_i2c_probe,
	.remove = ssm4329_i2c_remove,
	.id_table = ssm4329_i2c_ids,
};
module_i2c_driver(ssm4329_i2c_driver);

MODULE_DESCRIPTION("ASoC SSM4329 driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
