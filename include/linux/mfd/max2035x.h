/* SPDX-License-Identifier: GPL-2.0-only */
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
#define MAX20355_I2C_ADDR_RAM	0x50
#define MAX20357_I2C_ADDR_RAM	0x55

enum max2035x_type {
	MAX20355,
	MAX20357,
};

struct max2035x_plc;
struct max2035x_plc_irq_map;
struct mfd_cell;
struct regmap_irq_chip;

struct max2035x_variant {
	const char *name;
	u8 rev_reg;
	u8 max_register;
	u16 i2c_addr_ram;
	const struct mfd_cell *cells;
	int num_cells;
	const struct regmap_irq_chip *irq_chip;
	const struct max2035x_plc_irq_map *plc_irq_map;
	int plc_irq_map_size;
	void (*plc_handle_events)(struct max2035x_plc *plc, unsigned long events);
};

struct max2035x {
	struct device *dev;
	struct regmap *regmap;
	const struct max2035x_variant *info;

	struct max2035x_plc *plc_data;

	struct regmap_irq_chip_data *irq_data;
	int irq;
	u32 channel_id;
	enum max2035x_type type;
};

#endif /* __LINUX_MFD_MAX2035X_H__ */
