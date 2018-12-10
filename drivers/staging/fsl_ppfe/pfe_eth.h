/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#ifndef _PFE_ETH_H_
#define _PFE_ETH_H_
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#define PFE_ETH_NAPI_STATS
#define PFE_ETH_TX_STATS

#define PFE_ETH_FRAGS_MAX (65536 / HIF_RX_PKT_MIN_SIZE)
#define LRO_LEN_COUNT_MAX	32
#define LRO_NB_COUNT_MAX	32

#define PFE_PAUSE_FLAG_ENABLE		1
#define PFE_PAUSE_FLAG_AUTONEG		2

/* GEMAC configured by SW */
/* GEMAC configured by phy lines (not for MII/GMII) */

#define GEMAC_SW_FULL_DUPLEX    BIT(9)
#define GEMAC_SW_SPEED_10M      (0 << 12)
#define GEMAC_SW_SPEED_100M     BIT(12)
#define GEMAC_SW_SPEED_1G       (2 << 12)

#define GEMAC_NO_PHY            BIT(0)

struct ls1012a_eth_platform_data {
	/* board specific information */
	u32 mii_config;
	u32 phy_flags;
	u32 gem_id;
	u32 phy_id;
	u32 mdio_muxval;
	u8 mac_addr[ETH_ALEN];
	struct device_node	*phy_node;
};

struct ls1012a_mdio_platform_data {
	int id;
	int irq[32];
	u32 phy_mask;
	int mdc_div;
};

struct ls1012a_pfe_platform_data {
	struct ls1012a_eth_platform_data ls1012a_eth_pdata[3];
	struct ls1012a_mdio_platform_data ls1012a_mdio_pdata[3];
};

#define NUM_GEMAC_SUPPORT	2
#define DRV_NAME		"pfe-eth"
#define DRV_VERSION		"1.0"

#define LS1012A_TX_FAST_RECOVERY_TIMEOUT_MS	3
#define TX_POLL_TIMEOUT_MS	1000

#define EMAC_TXQ_CNT	16
#define EMAC_TXQ_DEPTH	(HIF_TX_DESC_NT)

#define JUMBO_FRAME_SIZE_V1	1900
#define JUMBO_FRAME_SIZE_V2	10258
/*
 * Client Tx queue threshold, for txQ flush condition.
 * It must be smaller than the queue size (in case we ever change it in the
 * future).
 */
#define HIF_CL_TX_FLUSH_MARK	32

/*
 * Max number of TX resources (HIF descriptors or skbs) that will be released
 * in a single go during batch recycling.
 * Should be lower than the flush mark so the SW can provide the HW with a
 * continuous stream of packets instead of bursts.
 */
#define TX_FREE_MAX_COUNT 16
#define EMAC_RXQ_CNT	3
#define EMAC_RXQ_DEPTH	HIF_RX_DESC_NT
/* make sure clients can receive a full burst of packets */
#define EMAC_RMON_TXBYTES_POS	0x00
#define EMAC_RMON_RXBYTES_POS	0x14

#define EMAC_QUEUENUM_MASK      (emac_txq_cnt - 1)
#define EMAC_MDIO_TIMEOUT	1000
#define MAX_UC_SPEC_ADDR_REG 31

struct pfe_eth_fast_timer {
	int queuenum;
	struct hrtimer timer;
	void *base;
};

struct  pfe_eth_priv_s {
	struct pfe		*pfe;
	struct hif_client_s	client;
	struct napi_struct	lro_napi;
	struct napi_struct	low_napi;
	struct napi_struct	high_napi;
	int			low_tmu_q;
	int			high_tmu_q;
	struct net_device_stats stats;
	struct net_device	*ndev;
	int			id;
	int			promisc;
	unsigned int		msg_enable;
	unsigned int		usr_features;

	spinlock_t		lock; /* protect member variables */
	unsigned int		event_status;
	int			irq;
	void			*EMAC_baseaddr;
	void			*GPI_baseaddr;
	/* PHY stuff */
	struct phy_device	*phydev;
	int			oldspeed;
	int			oldduplex;
	int			oldlink;
	struct device_node	*phy_node;
	struct clk		*gemtx_clk;
	int			wol;
	int			pause_flag;

	int			default_priority;
	struct pfe_eth_fast_timer fast_tx_timeout[EMAC_TXQ_CNT];

	struct ls1012a_eth_platform_data *einfo;
	struct sk_buff *skb_inflight[EMAC_RXQ_CNT + 6];

#ifdef PFE_ETH_TX_STATS
	unsigned int stop_queue_total[EMAC_TXQ_CNT];
	unsigned int stop_queue_hif[EMAC_TXQ_CNT];
	unsigned int stop_queue_hif_client[EMAC_TXQ_CNT];
	unsigned int stop_queue_credit[EMAC_TXQ_CNT];
	unsigned int clean_fail[EMAC_TXQ_CNT];
	unsigned int was_stopped[EMAC_TXQ_CNT];
#endif

#ifdef PFE_ETH_NAPI_STATS
	unsigned int napi_counters[NAPI_MAX_COUNT];
#endif
	unsigned int frags_inflight[EMAC_RXQ_CNT + 6];
};

struct pfe_eth {
	struct pfe_eth_priv_s *eth_priv[3];
};

struct pfe_mdio_priv_s {
	void __iomem *mdio_base;
	int			mdc_div;
	struct mii_bus		*mii_bus;
};

struct pfe_mdio {
	struct pfe_mdio_priv_s *mdio_priv[3];
};

int pfe_eth_init(struct pfe *pfe);
void pfe_eth_exit(struct pfe *pfe);
int pfe_eth_suspend(struct net_device *dev);
int pfe_eth_resume(struct net_device *dev);
int pfe_eth_mdio_reset(struct mii_bus *bus);

#endif /* _PFE_ETH_H_ */
