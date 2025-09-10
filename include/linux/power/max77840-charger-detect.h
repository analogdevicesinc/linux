/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Maxim Integrated Products, Inc.
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Author : Analog Devices <joan.na@analog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77840_CHARGER_DETECT_H__
#define __MAX77840_CHARGER_DETECT_H__

#define __TEST_DEVICE_NODE__

/* Register map */
#define REG_CHGDET_INT           0x01
#define REG_CHGDET_INT_MASK      0x03
#define BIT_CHGTYPE_I            BIT(0)
#define BIT_CHGDETRUN_I          BIT(1)
#define BIT_DCDTMR_I             BIT(2)
#define BIT_DXOVP_I              BIT(3)
#define BIT_VDNMON_I             BIT(4)

#define REG_CHGDET_STATUS        0x02
#define BIT_CHGTYPE              BITS(2, 0)
#define BIT_CHGDETRUN            BIT(3)
#define BIT_DCDTMR               BIT(4)
#define BIT_DXOVP                BIT(5)
#define BIT_VDNMON               BIT(6)

#define REG_CHGDET_CHGIN_ILIM    0x04
#define BIT_CHGIN_ILIM           BITS(6, 0)

#define REG_CHGDET_CTRL1         0x05
#define BIT_CHGTYPMAN            BIT(1)
#define BIT_RFU                  BIT(2)
#define BIT_DCD2SCT              BIT(3)
#define BIT_CDDELAY              BIT(4)
#define BIT_DCDCPL               BIT(5)
#define BIT_CDPDET               BIT(7)

#define REG_CHGDET_CTRL2         0x06
#define BIT_SFOUTASRT            BIT(0)
#define BIT_SFOUTORD             BIT(1)
#define BIT_DPDNVDEN             BIT(2)
#define BIT_DXOVPEN              BIT(3)
#define BIT_NOBCCOMP             BIT(4)
#define BIT_NOAUTOIBUS           BIT(5)
#define BIT_DISENU               BIT(6)

#define REG_CHGDET_CTRL3         0x07
#define BIT_USBLOWSP             BIT(0)
#define BIT_CDP_500MA            BIT(1)
#define BIT_DCP_IN_MAX           BIT(2)
#define BIT_APPLE_1A             BIT(3)

#define REG_CHGDET_QCNTRL        0x08
#define BIT_DNVD                 BITS(1, 0)
#define BIT_DPVD                 BITS(3, 2)
#define BIT_DPDVDEN              BIT(4)
#define BIT_ENU_EN               BIT(6)
#define BIT_ENUCTRLEN            BIT(7)

/* detail register bit description */
enum {
	CHGDET_DNVD_HI_Z,
	CHGDET_DNVD_GND,
	CHGDET_DNVD_VDP_SRC,
	CHGDET_DNVD_VD33,
};

enum {
	CHGDET_CHGTYP_NONE,
	CHGDET_CHGTYP_USB,
	CHGDET_CHGTYP_DP,
	CHGDET_CHGTYP_DEDICATED,
	CHGDET_CHGTYP_APPLE_500MA,
	CHGDET_CHGTYP_APPLE_1A,
	CHGDET_CHGTYP_SPECIAL,
	CHGDET_CHGTYP_CHARGER_LAST
};

enum {
	CHGDET_INT_CHGTYPE_I,
	CHGDET_INT_CHGDETRUN_I,
	CHGDET_INT_DCDTMR_I,
	CHGDET_INT_DXOVP_I,
	CHGDET_INT_VDNMON_I,
};

struct max77840_chgdet_platform_data {
	int input_current_limit;
};

#endif /* !__MAX77840_CHARGER_DETECT_H__ */
