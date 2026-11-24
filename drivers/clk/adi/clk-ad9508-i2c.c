// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9508 1.65 GHz Clock Fanout Buffer, I2C bus driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/unaligned.h>

#include "clk-ad9508.h"

/*
 * The device uses 16-bit register addresses, high byte first. The address
 * order of multi-byte I2C transfers is not documented, so each byte is
 * accessed with its own transaction.
 */
static int ad9508_i2c_read_byte(struct i2c_client *client, unsigned int reg,
				u8 *val)
{
	u8 addr[2];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.len = sizeof(addr),
			.buf = addr,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};
	int ret;

	put_unaligned_be16(reg, addr);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static int ad9508_i2c_read(void *context, unsigned int reg, unsigned int len,
			   unsigned int *val)
{
	struct i2c_client *client = context;
	unsigned int i;
	u8 byte;
	int ret;

	*val = 0;
	for (i = 0; i < len; i++) {
		ret = ad9508_i2c_read_byte(client, reg + i, &byte);
		if (ret)
			return ret;

		*val |= byte << (8 * i);
	}

	return 0;
}

static int ad9508_i2c_write(void *context, unsigned int reg, unsigned int len,
			    unsigned int val)
{
	struct i2c_client *client = context;
	unsigned int i;
	u8 buf[3];
	int ret;

	/* Most significant byte first, as the SPI bus does */
	for (i = len; i-- > 0;) {
		put_unaligned_be16(reg + i, buf);
		buf[2] = val >> (8 * i);

		ret = i2c_master_send(client, buf, sizeof(buf));
		if (ret < 0)
			return ret;
		if (ret != sizeof(buf))
			return -EIO;
	}

	return 0;
}

static const struct ad9508_bus_ops ad9508_i2c_bus_ops = {
	.read = ad9508_i2c_read,
	.write = ad9508_i2c_write,
};

static int ad9508_i2c_probe(struct i2c_client *client)
{
	/* SDO enable is a SPI port setting, it has no effect in I2C mode */
	return ad9508_probe(&client->dev, &ad9508_i2c_bus_ops, client, false);
}

static const struct of_device_id ad9508_i2c_of_match[] = {
	{ .compatible = "adi,ad9508" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9508_i2c_of_match);

static const struct i2c_device_id ad9508_i2c_id[] = {
	{ "ad9508" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad9508_i2c_id);

static struct i2c_driver ad9508_i2c_driver = {
	.driver = {
		.name = "ad9508",
		.of_match_table = ad9508_i2c_of_match,
	},
	.probe = ad9508_i2c_probe,
	.id_table = ad9508_i2c_id,
};
module_i2c_driver(ad9508_i2c_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9508 clock fanout buffer I2C driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CLK_AD9508");
