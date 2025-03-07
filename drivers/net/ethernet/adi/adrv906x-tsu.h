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

/* T_div66 is PCS RX output clock period, its [31:16] is nanosecond part
 * its [15:0] is the fractional nanosecond part
 * So it can be calculated as (1,000,000,000 * 0x10000) / frequency
 * where frequency is 156.25MHz for 10G, 390.625MHz for 25G
 */
#define T_DIV66_10G                                      419430         /* 0x66666 */
#define T_DIV66_25G                                      167772         /* 0x28f5c */

/* T_div64 is PCS RX input clock period. T_div64 = T_div66 * 64 / 66 */
#define T_DIV64_10G                                      406720         /* 0x634c0 */
#define T_DIV64_25G                                      162688         /* 0x27b80 */

extern int adrv906x_tod_cfg_cdc_delay;

struct adrv906x_tsu {
	void __iomem *base;
	unsigned long hsdig_clk_rate;
	u32 pcb_delay_tx;       /* Upper 16 is ns and lower 16 is fractional ns */
	u32 pcb_delay_rx;       /* Upper 16 is ns and lower 16 is fractional ns */
	u32 phy_delay_tx;       /* Upper 16 is ns and lower 16 is fractional ns */
	u32 phy_delay_rx;       /* Upper 16 is ns and lower 16 is fractional ns */
};

void adrv906x_tsu_set_speed(struct adrv906x_tsu *tsu, int speed);
int adrv906x_tsu_setup(struct platform_device *pdev, struct adrv906x_tsu *tsu,
		       struct device_node *eth_port_np);
void adrv906x_tsu_calculate_phy_delay(struct adrv906x_tsu *tsu, int speed,
				      bool rs_fec_enabled, u32 bit_slip,
				      u32 buf_delay_tx, u32 buf_delay_rx);
void adrv906x_tsu_set_phy_delay(struct adrv906x_tsu *tsu);
void adrv906x_tsu_set_ptp_timestamping_mode(void __iomem *base);
void adrv906x_tsu_compensate_tx_tstamp(struct adrv906x_tsu *tsu, struct timespec64 *ts);

#endif /* __ADRV906X_TSU_H__ */
