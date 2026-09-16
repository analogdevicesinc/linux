// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * MAX2035x Fuelgauge driver definitions
 */

#ifndef __MAX2035X_FUELGAUGE_H__
#define __MAX2035X_FUELGAUGE_H__

#define INI_FILE_NAME		"max2035x.ini"

struct max2035x_battery_data {
	u16 designcap;
	u16 dpacc;
	u16	dqacc;
	u16 ichgterm;
	u16 learncfg;
	u16 misccfg;
	u16 qr_table00;
	u16 qr_table10;
	u16 qr_table20;
	u16 qr_table30;
	u16 rcomp0;
	u16 relaxcfg;
	u16 tempco;
	u16 vempty;
	u16 rcompseg;
	u16 fullcaprep;
	u16 fullcapnom;
	u16 cycles;
	u16 mixcap;
	u16 config;
	u16 config2;
	u16 fullsocthr;
	u16 tgain;
	u16 toff;
	u16 curve;

    u16 model_data[2][16];
};

struct max2035x_fuelgauge {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_client *i2c;

	struct max2035x *chip;
	struct max2035x_battery_data *battery_data;

	int rsense_mohm;
};

#endif	/* __MAX2035X_FUELGAUGE_H__ */