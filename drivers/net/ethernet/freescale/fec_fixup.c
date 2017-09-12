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
#include <linux/of.h>
#include <linux/of_address.h>
#ifdef CONFIG_ARCH_MXC_ARM64
#include <soc/imx8/sc/sci.h>
#endif
#include "fec.h"

#define PHY_ID_AR8031   0x004dd074

#define IMX8QM_FUSE_MAC0_WORD0		452
#define IMX8QM_FUSE_MAC0_WORD1		453
#define IMX8QM_FUSE_MAC1_WORD0		454
#define IMX8QM_FUSE_MAC1_WORD1		455
#define IMX8M_OCOTP_MAC_ADDR0_OFF	0x640
#define IMX8M_OCOTP_MAC_ADDR1_OFF	0x650

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
		ar8031_registered = 1;
	}
}

int of_fec_enet_parse_fixup(struct device_node *np)
{
	int fixups = 0;

	if (of_get_property(np, "fsl,ar8031-phy-fixup", NULL))
		fixups |= FEC_QUIRK_AR8031_FIXUP;

	return fixups;
}

static void imx8mq_get_mac_from_fuse(int dev_id, unsigned char *mac)
{
	struct device_node *ocotp_np;
	void __iomem *base;
	u32 value;

	ocotp_np = of_find_compatible_node(NULL, NULL, "fsl,imx7d-ocotp");
	if (!ocotp_np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(ocotp_np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_ocotp_node;
	}

	value = readl_relaxed(base + IMX8M_OCOTP_MAC_ADDR1_OFF);
	mac[0] = (value >> 8);
	mac[1] = value;

	value = readl_relaxed(base + IMX8M_OCOTP_MAC_ADDR0_OFF);
	mac[2] = value >> 24;
	mac[3] = value >> 16;
	mac[4] = value >> 8;
	mac[5] = value;

put_ocotp_node:
	of_node_put(ocotp_np);
}

#ifdef CONFIG_ARCH_MXC_ARM64
static void imx8qm_get_mac_from_fuse(int dev_id, unsigned char *mac)
{
	uint32_t mu_id;
	sc_ipc_t ipc_handle;
	sc_err_t sc_err = SC_ERR_NONE;
	uint32_t val1 = 0, val2 = 0;
	uint32_t word1, word2;

	sc_err = sc_ipc_getMuID(&mu_id);
	if (sc_err != SC_ERR_NONE) {
		pr_err("FEC MAC fuse: Get MU ID failed\n");
		return;
	}

	sc_err = sc_ipc_open(&ipc_handle, mu_id);
	if (sc_err != SC_ERR_NONE) {
		pr_err("FEC MAC fuse: Open MU channel failed\n");
		return;
	}

	if (dev_id == 0) {
		word1 = IMX8QM_FUSE_MAC0_WORD0;
		word2 = IMX8QM_FUSE_MAC0_WORD1;
	} else {
		word1 = IMX8QM_FUSE_MAC1_WORD0;
		word2 = IMX8QM_FUSE_MAC1_WORD1;
	}

	sc_err = sc_misc_otp_fuse_read(ipc_handle, word1, &val1);
	if (sc_err != SC_ERR_NONE) {
		pr_err("FEC MAC fuse %d read error: %d\n", word1, sc_err);
		return;
	}

	sc_err = sc_misc_otp_fuse_read(ipc_handle, word2, &val2);
	if (sc_err != SC_ERR_NONE) {
		pr_err("FEC MAC fuse %d read error: %d\n", word2, sc_err);
		return;
	}

	mac[0] = val1;
	mac[1] = val1 >> 8;
	mac[2] = val1 >> 16;
	mac[3] = val1 >> 24;
	mac[4] = val2;
	mac[5] = val2 >> 8;
}
#else
static void imx8qm_get_mac_from_fuse(int dev_id, unsigned char *mac) {}
#endif

void fec_enet_get_mac_from_fuse(struct device_node *np, unsigned char *mac)
{
	int idx;

	if (!np)
		return;

	idx = of_alias_get_id(np, "ethernet");
	if (idx < 0)
		idx = 0;

	if (of_machine_is_compatible("fsl,imx8qm") ||
	    of_machine_is_compatible("fsl,imx8qxp"))
		imx8qm_get_mac_from_fuse(idx, mac);
	else if (of_machine_is_compatible("fsl,imx8mq"))
		imx8mq_get_mac_from_fuse(idx, mac);
}
