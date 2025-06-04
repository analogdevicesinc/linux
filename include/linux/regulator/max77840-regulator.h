/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 Maxim Integrated Products, Inc.
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Author : Analog Devices <joan.na@analog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MAX77840_REGULATOR_H
#define __LINUX_MAX77840_REGULATOR_H

struct max77840_regulator_platform_data {
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};

enum max77840_regulators {
	MAX77840_SAFEOUT1 = 0,
	MAX77840_REG_MAX,
};

#endif
