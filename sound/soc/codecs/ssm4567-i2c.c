/*
 * SSM4567 amplifier audio driver
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "ssm4567.h"

static int ssm4567_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	return ssm4567_probe(&i2c->dev, false,
		devm_regmap_init_i2c(i2c, &ssm4567_regmap_config));
}

static int ssm4567_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id ssm4567_i2c_ids[] = {
	{ "ssm4567", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ssm4567_i2c_ids);

static const struct of_device_id ssm4567_of_ids[] = {
	{ .compatible = "adi,ssm4567", },
	{ }
};
MODULE_DEVICE_TABLE(of, ssm4567_of_ids);

static struct i2c_driver ssm4567_driver = {
	.driver = {
		.name = "ssm4567",
		.owner = THIS_MODULE,
		.of_match_table = ssm4567_of_ids,
	},
	.probe = ssm4567_i2c_probe,
	.remove = ssm4567_i2c_remove,
	.id_table = ssm4567_i2c_ids,
};
module_i2c_driver(ssm4567_driver);

MODULE_DESCRIPTION("ASoC SSM4567 I2C driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
