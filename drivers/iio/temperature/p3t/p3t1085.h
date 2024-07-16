/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NXP P3T1085 Temperature Sensor Driver
 *
 * Copyright 2024 NXP
 */
#ifndef P3T1085_H
#define P3T1085_H

#include <linux/device.h>
#include <linux/iio/iio.h>

#define P3T1085_REG_TEMP		0x0
#define P3T1085_REG_CFGR		0x1
#define P3T1085_REG_HIGH_LIM		0x2
#define P3T1085_REG_LOW_LIM		0x3

#define P3T1085_RESOLUTION_10UC		62500

enum p3t1085_hw_id {
	P3T1085_ID,
};

struct p3t1085_data {
	struct device *dev;
	struct regmap *regmap;
};

int p3t1085_probe(struct device *dev, int irq, int hw_id, struct regmap *regmap);

#endif /* P3T1085_H */
