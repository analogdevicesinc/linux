// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * MAX2035x RAM driver definitions
 */

#ifndef __MAX2035X_RAM_H__
#define __MAX2035X_RAM_H__

struct max2035x;

struct max2035x_ram {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_client *i2c;

	struct max2035x *chip;
};

int max2035x_write_ram_data(struct max2035x *chip, const u8 *data, size_t len);
int max2035x_read_ram_data(struct max2035x *chip, u8 *data, size_t len);

#endif	/* __MAX2035X_RAM_H__ */