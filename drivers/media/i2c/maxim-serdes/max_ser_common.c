// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Serializer Common Driver
 *
 * Copyright (C) 2024 Analog Devices Inc.
 */

#include "max_ser_common.h"

const struct regmap_config max_ser_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1f00,
};
EXPORT_SYMBOL_GPL(max_ser_i2c_regmap);

int max_ser_reset(struct regmap *regmap)
{
	int ret;

	ret = regmap_update_bits(regmap, 0x10, 0x80, 0x80);
	if (ret)
		return ret;

	msleep(50);

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_reset);

int max_ser_wait_for_multiple(struct i2c_client *client, struct regmap *regmap,
			      u8 *addrs, unsigned int num_addrs)
{
	unsigned int i, j, val;
	int ret;

	for (i = 0; i < 10; i++) {
		for (j = 0; j < num_addrs; j++) {
			client->addr = addrs[j];

			ret = regmap_read(regmap, 0x0, &val);
			if (ret >= 0)
				return 0;
		}

		msleep(100);

		dev_err(&client->dev, "Retry %u waiting for serializer: %d\n", i, ret);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(max_ser_wait_for_multiple);

int max_ser_wait(struct i2c_client *client, struct regmap *regmap, u8 addr)
{
	return max_ser_wait_for_multiple(client, regmap, &addr, 1);
}
EXPORT_SYMBOL_GPL(max_ser_wait);

static int max_ser_get_dev_id(struct regmap *regmap, unsigned int *dev_id)
{
	return regmap_read(regmap, 0xd, dev_id);
}

static int max_ser_fix_tx_ids(struct regmap *regmap, u8 addr)
{
	unsigned int addr_regs[] = { 0x7b, 0x83, 0x8b, 0x93, 0xa3, 0xab };
	unsigned int dev_id;
	unsigned int i;
	int ret;

	ret = max_ser_get_dev_id(regmap, &dev_id);
	if (ret)
		return ret;

	switch (dev_id) {
	case MAX_SER_MAX9265A_DEV_ID:
		for (i = 0; i < ARRAY_SIZE(addr_regs); i++) {
			ret = regmap_write(regmap, addr_regs[i], addr);
			if (ret)
				return ret;
		}

		break;
	default:
		return 0;
	}

	return 0;
}

int max_ser_change_address(struct i2c_client *client, struct regmap *regmap, u8 addr,
			   bool fix_tx_ids)
{
	int ret;

	ret = regmap_write(regmap, 0x0, addr << 1);
	if (ret)
		return ret;

	client->addr = addr;

	if (fix_tx_ids) {
		ret = max_ser_fix_tx_ids(regmap, addr);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_change_address);

MODULE_LICENSE("GPL");
