/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices adau1962 codec driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef __ADAU1962_H__
#define __ADAU1962_H__

#include <linux/regmap.h>

struct device;

int adau1962_probe(struct device *dev, struct regmap *regmap,
	void (*switch_mode)(struct device *dev));

void adau1962_remove(struct device *dev);

extern const struct regmap_config adau1962_regmap_config;

enum adau1962_clk_id {
	ADAU1962_SYSCLK,
};

enum adau1962_sysclk_src {
	ADAU1962_SYSCLK_SRC_MCLK,
	ADAU1962_SYSCLK_SRC_LRCLK,
};

#endif
