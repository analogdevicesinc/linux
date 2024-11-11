// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_NET_H__
#define __ADRV906X_NET_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/net_tstamp.h>
#include "adrv906x-ndma.h"
#include "adrv906x-mac.h"
#include "adrv906x-switch.h"
#include "adrv906x-macsec-ext.h"
#include "adrv906x-tsu.h"

#define REGMAP_RESET_SWITCH                  BIT(0)
#define REGMAP_RESET_PCS_MAC0                BIT(4)
#define REGMAP_RESET_PCS_MAC1                BIT(5)
#define REGMAP_RESET_MACSEC0                 BIT(8)
#define REGMAP_RESET_MACSEC1                 BIT(9)

#define MAX_NETDEV_NUM                       2
#define MAX_MULTICAST_FILTERS                3
#define MAX_MTU_SIZE                         (NDMA_MAX_FRAME_SIZE_VALUE - ETH_HLEN - VLAN_HLEN \
					      - ETH_FCS_LEN)

struct adrv906x_oran_if {
	void __iomem *oif_rx;
	void __iomem *oif_tx;
};

struct adrv906x_eth_dev {
	struct net_device *ndev;
	struct device *dev;
	struct platform_device *pdev;
	struct adrv906x_ndma_dev *ndma_dev;
	struct hwtstamp_config tstamp_config;
	struct adrv906x_mac mac;
	struct adrv906x_oran_if oif;
	struct adrv906x_tsu tsu;
#if IS_ENABLED(CONFIG_MACSEC)
	struct adrv906x_macsec_priv macsec;
#endif // IS_ENABLED(CONFIG_MACSEC)
	int port;
	struct adrv906x_eth_if *parent;
	struct rtnl_link_stats64 rtnl_stats;
	int link_speed;
	int link_duplex;
	int tx_frames_pending;
	spinlock_t lock; /* protects struct access */
};

struct adrv906x_eth_if {
	struct adrv906x_eth_dev *adrv906x_dev[MAX_NETDEV_NUM];
	struct device *dev;
	struct adrv906x_eth_switch ethswitch;
	void __iomem *emac_cmn_regs;
	int tx_max_frames_pending;
	struct mutex mtx; /* protects regs access*/
	u32 recovered_clk_div_10g;
	u32 recovered_clk_div_25g;
};

#endif /* __ADRV906X_NET_H__ */
