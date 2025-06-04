/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Maxim Integrated
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Author : Analog Devices <joan.na@analog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77840_BATTERY_H_
#define __MAX77840_BATTERY_H_

#define __TEST_DEVICE_NODE__

/* Register map */
#define REG_STATUS        0x00
#define BIT_SMX           BIT(14)
#define BIT_TMX           BIT(13)
#define BIT_VMX           BIT(12)
#define BIT_SMN           BIT(10)
#define BIT_TMN           BIT(9)
#define BIT_VMN           BIT(8)
#define BIT_DSOCI         BIT(7)

#define REG_VALRT_TH      0x01
#define REG_TALRT_TH      0x02
#define REG_SALRT_TH      0x03
#define REG_TEMP          0x08
#define REG_VCELL         0x09
#define REG_AVGVCELL      0x19
#define REG_CONFIG        0x1D
#define BIT_Aen           BIT(2)

#define REG_VERSION       0x21
#define REG_LEARNCFG      0x28
#define REG_FILTERCFG     0x29
#define REG_MISCCFG       0x2B
#define REG_CGAIN         0x2E
#define REG_RCOMP0        0x38
#define REG_CONFIG2       0xBB
#define BIT_DSOCEN        BIT(7)

#define REG_VFOCV         0xFB
#define REG_VFSOC         0xFF

struct max77840_fg_platform_data {
	int	soc_alert_threshold;
};
#endif
