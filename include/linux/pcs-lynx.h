/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2020 NXP
 * Lynx PCS helpers
 */

#ifndef __LINUX_PCS_LYNX_H
#define __LINUX_PCS_LYNX_H

#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>

struct phy;

struct phylink_pcs *lynx_pcs_create_mdiodev(struct mii_bus *bus, int addr,
					    struct phy **phys, size_t num_phys);
struct phylink_pcs *lynx_pcs_create_fwnode(struct fwnode_handle *node,
					   struct phy **phys, size_t num_phys);

void lynx_pcs_destroy(struct phylink_pcs *pcs);

void lynx_pcs_set_supported_interfaces(struct phylink_pcs *pcs,
				       phy_interface_t default_interface,
				       unsigned long *supported_interfaces);

#endif /* __LINUX_PCS_LYNX_H */
