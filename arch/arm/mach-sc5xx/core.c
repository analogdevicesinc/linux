// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Common functionality needed on all ezkits. Things like phy fixup functions, etc.
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/init.h>
#include <linux/phy.h>

#include "core.h"

#define DP83865_PHY_ID          0x20005c7a
#define REG_DP83865_AUX_CTRL    0x12
#define BITP_AUX_CTRL_RGMII_EN  12
#define RGMII_3COM_MODE         3
static int dp83865_fixup(struct phy_device *phydev)
{
	int  phy_data = 0;

	phy_data = phy_read(phydev, REG_DP83865_AUX_CTRL);

	/* enable 3com mode for RGMII */
	phy_write(phydev, REG_DP83865_AUX_CTRL,
			     (RGMII_3COM_MODE << BITP_AUX_CTRL_RGMII_EN) | phy_data);

	return 0;
}

#define DP83848_PHY_ID          0x20005c90
#define REG_DP83848_PHY_MICR    0x11
#define BITM_PHY_MICR_INTEN     0x2
#define BITM_PHY_MICR_INT_OE    0x1
static int dp83848_fixup(struct phy_device *phydev)
{
	phy_write(phydev, REG_DP83848_PHY_MICR,
				BITM_PHY_MICR_INTEN | BITM_PHY_MICR_INT_OE);

	return 0;
}

void __init sc5xx_init_ethernet(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(DP83865_PHY_ID, 0xffffffff, dp83865_fixup);
		phy_register_fixup_for_uid(DP83848_PHY_ID, 0xffffffff, dp83848_fixup);
	}
}
