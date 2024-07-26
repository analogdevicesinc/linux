// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_ETHTOOL_H__
#define __ADRV906X_ETHTOOL_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/ethtool.h>

extern const struct ethtool_ops adrv906x_ethtool_ops;

int adrv906x_eth_set_link_ksettings(struct net_device *ndev, const struct ethtool_link_ksettings *cmd);
int adrv906x_eth_get_ts_info(struct net_device *ndev, struct ethtool_ts_info *info);
int adrv906x_eth_get_sset_count(struct net_device *netdev, int sset);
void adrv906x_eth_get_strings(struct net_device *netdev, u32 sset, u8 *buf);
void adrv906x_eth_get_ethtool_stats(struct net_device *netdev, struct ethtool_stats *stats, u64 *data);
void adrv906x_eth_selftest_run(struct net_device *ndev, struct ethtool_test *etest, u64 *buf);

#endif /* __ADRV906X_ETHTOOL_H__ */
