// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * MAX2035x common definitions for Core and Sub-devices
 */

#ifndef __LINUX_MFD_MAX2035X_H__
#define __LINUX_MFD_MAX2035X_H__

#include <linux/device.h>
#include <linux/regmap.h>

/* I2C Slave Addresses (7-bit) */
#define MAX20355_I2C_ADDR_PLC	0x28
#define MAX20357_I2C_ADDR_PLC	0x15
#define MAX2035X_I2C_ADDR_FG	0x36
#define MAX20355_I2C_ADDR_RAM	0x50
#define MAX20357_I2C_ADDR_RAM	0x55

enum max2035x_type {
	MAX20355,
	MAX20357,
};

struct max2035x_plc;

typedef void (*max20355_plc_notify_t)(struct max2035x_plc *plc_data, u8 *int_status);
typedef void (*max20357_plc_notify_t)(struct max2035x_plc *plc_data, u8 *int_status);

struct max2035x {
	struct device *dev;
	struct regmap *regmap;

	struct i2c_client *fuelgauge;	/* Fuelgauge (0x36) */
	struct i2c_client *ram;			/* RAM (MAX20355:0x50, MAX20357:0x55) */

	struct max2035x_plc *plc_data;
	struct max2035x_ram *ram_data;

	int irq;
	u32 channel_id;
	enum max2035x_type type;

	max20355_plc_notify_t plc_notify_20355;
    max20357_plc_notify_t plc_notify_20357;
};

struct max2035x *max2035x_get_device(enum max2035x_type type, int channel);
void max2035x_register_device(struct max2035x *chip);
void max2035x_unregister_device(struct max2035x *chip);

int max2035x_plc_init(void);
void max2035x_plc_exit(void);
int max2035x_fuelgauge_init(void);
void max2035x_fuelgauge_exit(void);
int max2035x_ram_init(void);
void max2035x_ram_exit(void);

#endif /* __LINUX_MFD_MAX2035X_H__ */
