// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_SERDES_H__
#define __ADRV906X_SERDES_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/bitfield.h>

typedef void (*adrv906x_serdes_cb)(struct phy_device *phydev, bool enable);

int adrv906x_serdes_open(struct phy_device *phydev,
			 adrv906x_serdes_cb tx_cb, adrv906x_serdes_cb rx_cb);
int adrv906x_serdes_close(struct phy_device *phydev);
int adrv906x_serdes_lnk_up_req(struct phy_device *phydev);
int adrv906x_serdes_lnk_down_req(struct phy_device *phydev);
int adrv906x_serdes_genl_register_family(void);
int adrv906x_serdes_genl_unregister_family(void);

#endif /* __ADRV906X_SERDES_H__ */
