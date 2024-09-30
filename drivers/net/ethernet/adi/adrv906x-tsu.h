// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_TSU_H__
#define __ADRV906X_TSU_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/netdevice.h>

#define ADRV906X_TSU_STATIC_PHY_DELAY_RX                 0x0000003C
#define ADRV906X_TSU_STATIC_PHY_DELAY_TX                 0x00000040
#define ADRV906X_TSU_TIMESTAMPING_MODE                   0x00000038
#define   ADRV906X_CORE_SPEED                            BIT(8)
#define   ADRV906X_CORE_SPEED_10G                        0x00000000
#define   ADRV906X_CORE_SPEED_25G                        0x00000100
#define   ADRV906X_PTP_TIMESTAMPING_MODE                 GENMASK(1, 0)
#define   ADRV906X_PTP_TIMESTAMPING_MODE_TWO_STEP        0x00000000     /* Two-step */
#define   ADRV906X_PTP_TIMESTAMPING_MODE_ONE_STEP        0x00000001     /* One-step */
#define   ADRV906X_PTP_TIMESTAMPING_MODE_TRANSP          0x00000002     /* Transparent Clock */

struct adrv906x_tsu {
	void __iomem *base;
	u32 phy_delay_tx;
	u32 phy_delay_rx;
};

void adrv906x_tsu_set_speed(struct adrv906x_tsu *tsu, struct phy_device *phydev);
int adrv906x_tsu_setup(struct platform_device *pdev, struct adrv906x_tsu *tsu, struct device_node *eth_port_np);
void adrv906x_tsu_set_phy_delay(void __iomem *base, u32 phy_delay_tx, u32 phy_delay_rx);
void adrv906x_tsu_set_ptp_timestamping_mode(void __iomem *base);

#endif /* __ADRV906X_TSU_H__ */
