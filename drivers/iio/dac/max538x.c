//-------------------------------------------------------------------------------/
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  max538x.c - Support for MAX5380/1/2 DACs
 *
 *  Copyright (c) 2023 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>

#define MAX538X_RESOLUTION 		0xFF

enum max538x_device_ids {
	MAX5380L,
	MAX5380M,
	MAX5380N,
	MAX5380K,
	MAX5381L,
	MAX5381M,
	MAX5381N,
	MAX5381K,
	MAX5382L,
	MAX5382M,
	MAX5382N,
	MAX5382K,
};

struct max538x_data {
    struct i2c_client *client;
	u8 max538x_vdd ;
	u8 max538x_vref ;
    const struct max538x_chip_info *chip_info;
	enum max538x_device_ids active_device;
};

struct max538x_chip_info {
	u8 vfactor;
    u8 addr;
};

const struct max538x_chip_info chip_info[] = {
	[MAX5380L] = {
		.vfactor = 2,
		.addr = 0x30,
	},
	[MAX5380M] = {
		.vfactor = 2,
		.addr = 0x31,
	},
	[MAX5380N] = {
		.vfactor = 2,
		.addr = 0x32,
	},
	[MAX5380K] = {
		.vfactor = 2,
		.addr = 0x33,
	},
	[MAX5381L] = {
		.vfactor = 4,
		.addr = 0x30,
	},
	[MAX5381M] = {
		.vfactor = 4,
		.addr = 0x31,
	},
	[MAX5381N] = {
		.vfactor = 4,
		.addr = 0x32,
	},
	[MAX5381K] = {
		.vfactor = 4,
		.addr = 0x33,
	},
	[MAX5382L] = {
		.vfactor = 0.9,
		.addr = 0x30,
	},
	[MAX5382M] = {
		.vfactor = 0.9,
		.addr = 0x31,
	},
	[MAX5382N] = {
		.vfactor = 0.9,
		.addr = 0x32,
	},
	[MAX5382K] = {
		.vfactor = 0.9,
		.addr = 0x33,
	},
};

static int max538x_remove(struct i2c_client *client)
{
	struct max538x_data *dev = i2c_get_clientdata(client);

	if (!dev)
        return -EINVAL;
 
	return 0;

}

static int max538x_set_voutput(struct max538x_data *data, u8 vout)
{
	int ret;
	u8 buf[2];

	buf[0] = data->chip_info->addr;
	buf[1] = data->max538x_vdd;
	buf[2] = data->max538x_vref;
	ret = i2c_master_send(data->client, buf, 3);

	if (ret < 0)
		return ret;

	data->max538x_vref = (MAX538X_RESOLUTION / data->chip_info->vfactor) * (vout / MAX538X_RESOLUTION);
	data->max538x_vdd = (MAX538X_RESOLUTION / data->chip_info->vfactor) - data->max538x_vref;

	return 0;
}

static int max538x_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    struct max538x_data *data;
    int ret;

    data = devm_kzalloc(&client->dev, sizeof(struct max538x_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->client = client;
    data->active_device = id->driver_data - MAX5380L;
    data->chip_info = &chip_info[data->active_device];

    i2c_set_clientdata(client, data);

  ret = max538x_set_voutput(data, 0x00);
if (ret < 0) {
    dev_err(&client->dev, "Failed to set initial voltage output: %d\n", ret);
    return ret;
}

    return 0;
}

static const struct i2c_device_id max538x_id[] = {
    { "max5380l", MAX5380L },
    { "max5380m", MAX5380M },
    { "max5380n", MAX5380N },
    { "max5380k", MAX5380K },
    { "max5381l", MAX5381L },
    { "max5381m", MAX5381M },
    { "max5381n", MAX5381N },
    { "max5381k", MAX5381K },
    { "max5382l", MAX5382L },
    { "max5382m", MAX5382M },
    { "max5382n", MAX5382N },
    { "max5382k", MAX5382K },
    { }
};
MODULE_DEVICE_TABLE(i2c, max538x_id);

static struct i2c_driver max538x_driver = {
    .driver = {
        .name = "max538x",
    },
    .probe = max538x_probe,
    .remove = max538x_remove,
    .id_table = max538x_id,
};
module_i2c_driver(max538x_driver);

MODULE_AUTHOR("Marc Paolo Sosa <marcpaolo.sosa@analog.com>");
MODULE_DESCRIPTION("MAX5380/1/2 DAC driver");
MODULE_LICENSE("GPL v2");
