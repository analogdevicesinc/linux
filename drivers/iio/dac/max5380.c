// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  max5380.c support for Maxim MAX5380
 *
 *  Copyright (C) 2023 Marc Paolo Sosa <marcpaolososa@analog.com>
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>

enum max5380_device_ids {
	ID_MAX5380,
};

struct max5380_data {
	struct i2c_client *client;
	u8 value;
};

static int max5380_i2c_write(struct max5380_data *data, u8 value)
{
	int ret;

	data->value = value & 0xff;

	ret = i2c_master_send(data->client, &data->value, sizeof(data->value));

	return 0;
}

static int max5380_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct max5380_data *data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	data = devm_kzalloc(&client->dev, sizeof(struct max5380_data), GFP_KERNEL);
	i2c_set_clientdata(client, data);
	data->client = client;

	max5380_i2c_write(data, 0);

	return 0;
}

static const struct i2c_device_id max5380_id[] = {
	{"max5380", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, max5380_id);

static struct i2c_driver max5380_driver = {
	.driver = {
		.name = "max5380",
	},
	.probe = max5380_i2c_probe,
	.id_table = max5380_id,
};
module_i2c_driver(max5380_driver);

MODULE_AUTHOR("Marc Paolo Sosa <marcpaolo.sosa@analog.com>");
MODULE_DESCRIPTION("MAX5380/5381/5382 8-bit DAC");
MODULE_LICENSE("GPL");
