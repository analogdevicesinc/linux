// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_SERDES_H__
#define __ADRV906X_SERDES_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/bitfield.h>

typedef void (*adrv906x_serdes_cal_done_cb)(struct phy_device *phydev);

struct adrv906x_serdes {
	struct phy_device *phydev;
	struct delayed_work send_req;
	adrv906x_serdes_cal_done_cb cb;
	int state;
	int lane;
	int speed;
};

int adrv906x_serdes_open(struct phy_device *phydev, struct adrv906x_serdes *serdes,
			 adrv906x_serdes_cal_done_cb cb);
int adrv906x_serdes_close(struct phy_device *phydev);
int adrv906x_serdes_cal_start(struct phy_device *phydev);
int adrv906x_serdes_cal_stop(struct phy_device *phydev);
int adrv906x_serdes_genl_register_family(void);
int adrv906x_serdes_genl_unregister_family(void);

#endif /* __ADRV906X_SERDES_H__ */
