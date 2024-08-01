// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#ifndef MAX_SER_COMMON_H
#define MAX_SER_COMMON_H

extern const struct regmap_config max_ser_i2c_regmap;

#define MAX_SER_MAX96717_DEV_ID			0xbf
#define MAX_SER_MAX9265A_DEV_ID			0x91

int max_ser_reset(struct regmap *regmap);
int max_ser_wait(struct i2c_client *client, struct regmap *regmap, u8 addr);
int max_ser_wait_for_multiple(struct i2c_client *client, struct regmap *regmap,
			      u8 *addrs, unsigned int num_addrs);
int max_ser_change_address(struct i2c_client *client, struct regmap *regmap, u8 addr,
			   bool fix_tx_ids);

#endif // MAX_SER_COMMON_H
