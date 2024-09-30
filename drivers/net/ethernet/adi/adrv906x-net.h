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

#define EMAC_CMN_DIGITAL_CTRL0               0x0010
#define   EMAC_CMN_RX_LINK0_EN               BIT(0)
#define   EMAC_CMN_RX_LINK1_EN               BIT(1)
#define   EMAC_CMN_TX_LINK0_EN               BIT(4)
#define   EMAC_CMN_TX_LINK1_EN               BIT(5)
#define   EMAC_CMN_SW_LINK0_BYPASS_EN        BIT(8)
#define   EMAC_CMN_SW_LINK1_BYPASS_EN        BIT(9)
#define   EMAC_CMN_SW_PORT0_EN               BIT(12)
#define   EMAC_CMN_SW_PORT1_EN               BIT(13)
#define   EMAC_CMN_SW_PORT2_EN               BIT(14)
#define   EMAC_CMN_MACSEC_BYPASS_EN          BIT(16)
#define   EMAC_CMN_CDR_DIV_PORT0_EN          BIT(20)
#define   EMAC_CMN_CDR_DIV_PORT1_EN          BIT(21)
#define   EMAC_CMN_CDR_SEL                   BIT(24)
#define EMAC_CMN_DIGITAL_CTRL1               0x0014
#define   EMAC_CMN_RECOVERED_CLK_DIV_0       GENMASK(12, 0)
#define   EMAC_CMN_RECOVERED_CLK_DIV_1       GENMASK(28, 16)
#define EMAC_CMN_DIGITAL_CTRL2               0x0018
#define   EMAC_CMN_LOOPBACK_BYPASS_MAC_0     BIT(28)
#define   EMAC_CMN_LOOPBACK_BYPASS_MAC_1     BIT(29)
#define   EMAC_CMN_LOOPBACK_BYPASS_PCS_0     BIT(24)
#define   EMAC_CMN_LOOPBACK_BYPASS_PCS_1     BIT(25)
#define   EMAC_CMN_LOOPBACK_BYPASS_DESER_0   BIT(20)
#define   EMAC_CMN_LOOPBACK_BYPASS_DESER_1   BIT(21)
#define   EMAC_CMN_TX_BIT_REPEAT_RATIO       BIT(0)
#define EMAC_CMN_DIGITAL_CTRL3               0x001c
#define   EMAC_CMN_SW_PORT2_DSA_INSERT_EN    BIT(20)
#define EMAC_CMN_DIGITAL_CTRL4               0x0020
#define   EMAC_CMN_PCS_STATUS_NE_CNT_0       GENMASK(7, 0)
#define   EMAC_CMN_PCS_STATUS_NE_CNT_1       GENMASK(15, 8)
#define   EMAC_CMN_CLEAR_PCS_STATUS_NE_CNT   BIT(16)
#define EMAC_CMN_RST_REG                     0x0030
#define EMAC_CMN_PHY_CTRL                    0x0040
#define   EMAC_CMN_RXDES_DIG_RESET_N_0       BIT(0)
#define   EMAC_CMN_RXDES_DIG_RESET_N_1       BIT(1)
#define   EMAC_CMN_RXDES_FORCE_LANE_PD_0     BIT(4)
#define   EMAC_CMN_RXDES_FORCE_LANE_PD_1     BIT(5)
#define   EMAC_CMN_TXSER_DIG_RESET_N_0       BIT(8)
#define   EMAC_CMN_TXSER_DIG_RESET_N_1       BIT(9)
#define   EMAC_CMN_TXSER_FORCE_LANE_PD_0     BIT(12)
#define   EMAC_CMN_TXSER_FORCE_LANE_PD_1     BIT(13)
#define   EMAC_CMN_SERDES_REG_RESET_N        BIT(16)
#define   EMAC_CMN_TXSER_SYNC_TRIGGER_0      BIT(20)
#define   EMAC_CMN_TXSER_SYNC_TRIGGER_1      BIT(21)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_EN_0  BIT(24)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_EN_1  BIT(25)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_VAL_0 BIT(28)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_VAL_1 BIT(29)
#define EMAC_CMN_PLL_CTRL                    0x0050
#define EMAC_CMN_GPIO_SELECT                 0x0060
#define EMAC_CMN_EMAC_SPARE                  0x3000

#define MAX_NETDEV_NUM                       2
#define MAX_MULTICAST_FILTERS                3

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

void adrv906x_eth_cmn_serdes_tx_sync_trigger(struct net_device *ndev, u32 lane);
void adrv906x_eth_cmn_serdes_reset_4pack(struct net_device *ndev);

#endif /* __ADRV906X_NET_H__ */
