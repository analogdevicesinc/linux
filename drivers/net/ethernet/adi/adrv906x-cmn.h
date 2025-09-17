// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024-2025, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_CMN_H__
#define __ADRV906X_CMN_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include "adrv906x-net.h"

void adrv906x_eth_cmn_pll_reset(struct net_device *ndev);
void adrv906x_eth_cmn_ser_tx_sync_trigger(struct net_device *ndev);
void adrv906x_eth_cmn_ser_pwr_down(struct net_device *ndev);
void adrv906x_eth_cmn_ser_pwr_up_and_reset(struct net_device *ndev);
void adrv906x_eth_cmn_deser_pwr_down(struct net_device *ndev);
void adrv906x_eth_cmn_deser_pwr_up_and_reset(struct net_device *ndev);
int adrv906x_eth_cmn_rst_reg(void __iomem *regs);
void adrv906x_eth_cmn_recovered_clk_config(struct adrv906x_eth_dev *adrv906x_dev);
void adrv906x_eth_cmn_mode_cfg(struct net_device *ndev);
void adrv906x_eth_cmn_init(void __iomem *regs, bool switch_enabled, bool macsec_enabled);
void adrv906x_cmn_pcs_link_drop_cnt_clear(struct adrv906x_eth_if *adrv906x_eth);
ssize_t adrv906x_cmn_pcs_link_drop_cnt_get(struct adrv906x_eth_if *adrv906x_eth, char *buf);
ssize_t adrv906x_cmn_recovered_clock_output_set(struct device *dev, const char *buf, size_t cnt);
ssize_t adrv906x_cmn_recovered_clock_output_get(struct device *dev, char *buf);
void adrv906x_cmn_set_mac_loopback(struct adrv906x_eth_dev *adrv906x_dev, bool enable);
void adrv906x_cmn_set_phy_loopback(struct adrv906x_eth_dev *adrv906x_dev, bool enable);

#endif /* __ADRV906X_CMN_H__ */
