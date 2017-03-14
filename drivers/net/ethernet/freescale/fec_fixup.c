/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/netdevice.h>
#include <linux/phy.h>
#include "fec.h"

#define PHY_ID_AR8031   0x004dd074

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Set RGMII IO voltage to 1.8V */
	phy_write(dev, 0x1d, 0x1f);
	phy_write(dev, 0x1e, 0x8);

	/* Disable phy AR8031 SmartEEE function */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	phy_write(dev, 0x1e, 0x100);

	return 0;
}

void fec_enet_register_fixup(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	static int registered = 0;
	int err;

	if (!IS_BUILTIN(CONFIG_PHYLIB))
		return;

	if (fep->fixups & FEC_QUIRK_AR8031_FIXUP) {
		static int ar8031_registered = 0;

		if (ar8031_registered)
			return;
		err = phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffef,
					ar8031_phy_fixup);
		if (err)
			netdev_info(ndev, "Cannot register PHY board fixup\n");
		registered = 1;
	}
}

int of_fec_enet_parse_fixup(struct device_node *np)
{
	int fixups = 0;

	if (of_get_property(np, "fsl,ar8031-phy-fixup", NULL))
		fixups |= FEC_QUIRK_AR8031_FIXUP;

	return fixups;
}
