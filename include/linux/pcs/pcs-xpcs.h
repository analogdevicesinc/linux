/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Synopsys, Inc. and/or its affiliates.
 * Synopsys DesignWare XPCS helpers
 */

#ifndef __LINUX_PCS_XPCS_H
#define __LINUX_PCS_XPCS_H

#include <linux/clk.h>
#include <linux/fwnode.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/types.h>

/* AN mode */
#define DW_AN_C73			1
#define DW_AN_C37_SGMII			2
#define DW_2500BASEX			3
#define DW_AN_C37_1000BASEX		4
#define DW_10GBASER			5

struct dw_xpcs_desc;

enum dw_xpcs_pcs_id {
	DW_XPCS_ID_NATIVE = 0,
	NXP_SJA1105_XPCS_ID = 0x00000010,
	NXP_SJA1110_XPCS_ID = 0x00000020,
	DW_XPCS_ID = 0x7996ced0,
	DW_XPCS_ID_MASK = 0xffffffff,
};

enum dw_xpcs_pma_id {
	DW_XPCS_PMA_ID_NATIVE = 0,
	DW_XPCS_PMA_GEN1_3G_ID,
	DW_XPCS_PMA_GEN2_3G_ID,
	DW_XPCS_PMA_GEN2_6G_ID,
	DW_XPCS_PMA_GEN4_3G_ID,
	DW_XPCS_PMA_GEN4_6G_ID,
	DW_XPCS_PMA_GEN5_10G_ID,
	DW_XPCS_PMA_GEN5_12G_ID,
	WX_TXGBE_XPCS_PMA_10G_ID = 0x0018fc80,
};

struct dw_xpcs_info {
	u32 pcs;
	u32 pma;
};

enum dw_xpcs_clock {
	DW_XPCS_CORE_CLK,
	DW_XPCS_PAD_CLK,
	DW_XPCS_NUM_CLKS,
};

struct dw_xpcs {
	struct dw_xpcs_info info;
	const struct dw_xpcs_desc *desc;
	struct mdio_device *mdiodev;
	struct clk_bulk_data clks[DW_XPCS_NUM_CLKS];
	struct phylink_pcs pcs;
	phy_interface_t interface;
};

int xpcs_get_an_mode(struct dw_xpcs *xpcs, phy_interface_t interface);
void xpcs_link_up(struct phylink_pcs *pcs, unsigned int neg_mode,
		  phy_interface_t interface, int speed, int duplex);
int xpcs_do_config(struct dw_xpcs *xpcs, phy_interface_t interface,
		   const unsigned long *advertising, unsigned int neg_mode);
void xpcs_get_interfaces(struct dw_xpcs *xpcs, unsigned long *interfaces);
int xpcs_config_eee(struct dw_xpcs *xpcs, int mult_fact_100ns,
		    int enable);
struct dw_xpcs *xpcs_create_mdiodev(struct mii_bus *bus, int addr,
				    phy_interface_t interface);
struct dw_xpcs *xpcs_create_fwnode(struct fwnode_handle *fwnode,
				   phy_interface_t interface);
void xpcs_destroy(struct dw_xpcs *xpcs);

#endif /* __LINUX_PCS_XPCS_H */
