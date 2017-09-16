/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* @pfe_eth.c.
 *  Ethernet driver for to handle exception path for PFE.
 *  - uses HIF functions to send/receive packets.
 *  - uses ctrl function to start/stop interfaces.
 *  - uses direct register accesses to control phy operation.
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>

#include <net/ip.h>
#include <net/sock.h>

#include <linux/io.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/i2c.h>

#if defined(CONFIG_NF_CONNTRACK_MARK)
#include <net/netfilter/nf_conntrack.h>
#endif

#include "pfe_mod.h"
#include "pfe_eth.h"

static void *cbus_emac_base[3];
static void *cbus_gpi_base[3];

/* Forward Declaration */
static void pfe_eth_exit_one(struct pfe_eth_priv_s *priv);
static void pfe_eth_flush_tx(struct pfe_eth_priv_s *priv);
static void pfe_eth_flush_txQ(struct pfe_eth_priv_s *priv, int tx_q_num, int
				from_tx, int n_desc);

unsigned int gemac_regs[] = {
	0x0004, /* Interrupt event */
	0x0008, /* Interrupt mask */
	0x0024, /* Ethernet control */
	0x0064, /* MIB Control/Status */
	0x0084, /* Receive control/status */
	0x00C4, /* Transmit control */
	0x00E4, /* Physical address low */
	0x00E8, /* Physical address high */
	0x0144, /* Transmit FIFO Watermark and Store and Forward Control*/
	0x0190, /* Receive FIFO Section Full Threshold */
	0x01A0, /* Transmit FIFO Section Empty Threshold */
	0x01B0, /* Frame Truncation Length */
};

/********************************************************************/
/*                   SYSFS INTERFACE				    */
/********************************************************************/

#ifdef PFE_ETH_NAPI_STATS
/*
 * pfe_eth_show_napi_stats
 */
static ssize_t pfe_eth_show_napi_stats(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct pfe_eth_priv_s *priv = netdev_priv(to_net_dev(dev));
	ssize_t len = 0;

	len += sprintf(buf + len, "sched:  %u\n",
			priv->napi_counters[NAPI_SCHED_COUNT]);
	len += sprintf(buf + len, "poll:   %u\n",
			priv->napi_counters[NAPI_POLL_COUNT]);
	len += sprintf(buf + len, "packet: %u\n",
			priv->napi_counters[NAPI_PACKET_COUNT]);
	len += sprintf(buf + len, "budget: %u\n",
			priv->napi_counters[NAPI_FULL_BUDGET_COUNT]);
	len += sprintf(buf + len, "desc:   %u\n",
			priv->napi_counters[NAPI_DESC_COUNT]);

	return len;
}

/*
 * pfe_eth_set_napi_stats
 */
static ssize_t pfe_eth_set_napi_stats(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct pfe_eth_priv_s *priv = netdev_priv(to_net_dev(dev));

	memset(priv->napi_counters, 0, sizeof(priv->napi_counters));

	return count;
}
#endif
#ifdef PFE_ETH_TX_STATS
/* pfe_eth_show_tx_stats
 *
 */
static ssize_t pfe_eth_show_tx_stats(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct pfe_eth_priv_s *priv = netdev_priv(to_net_dev(dev));
	ssize_t len = 0;
	int i;

	len += sprintf(buf + len, "TX queues stats:\n");

	for (i = 0; i < emac_txq_cnt; i++) {
		struct netdev_queue *tx_queue = netdev_get_tx_queue(priv->ndev,
									i);

		len += sprintf(buf + len, "\n");
		__netif_tx_lock_bh(tx_queue);

		hif_tx_lock(&pfe->hif);
		len += sprintf(buf + len,
				"Queue %2d :  credits               = %10d\n"
				, i, hif_lib_tx_credit_avail(pfe, priv->id, i));
		len += sprintf(buf + len,
				 "            tx packets            = %10d\n"
				,  pfe->tmu_credit.tx_packets[priv->id][i]);
		hif_tx_unlock(&pfe->hif);

		/* Don't output additionnal stats if queue never used */
		if (!pfe->tmu_credit.tx_packets[priv->id][i])
			goto skip;

		len += sprintf(buf + len,
				 "            clean_fail            = %10d\n"
				, priv->clean_fail[i]);
		len += sprintf(buf + len,
				 "            stop_queue            = %10d\n"
				, priv->stop_queue_total[i]);
		len += sprintf(buf + len,
				 "            stop_queue_hif        = %10d\n"
				, priv->stop_queue_hif[i]);
		len += sprintf(buf + len,
				"            stop_queue_hif_client = %10d\n"
				, priv->stop_queue_hif_client[i]);
		len += sprintf(buf + len,
				 "            stop_queue_credit     = %10d\n"
				, priv->stop_queue_credit[i]);
skip:
		__netif_tx_unlock_bh(tx_queue);
	}
	return len;
}

/* pfe_eth_set_tx_stats
 *
 */
static ssize_t pfe_eth_set_tx_stats(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct pfe_eth_priv_s *priv = netdev_priv(to_net_dev(dev));
	int i;

	for (i = 0; i < emac_txq_cnt; i++) {
		struct netdev_queue *tx_queue = netdev_get_tx_queue(priv->ndev,
									i);

		__netif_tx_lock_bh(tx_queue);
		priv->clean_fail[i] = 0;
		priv->stop_queue_total[i] = 0;
		priv->stop_queue_hif[i] = 0;
		priv->stop_queue_hif_client[i] = 0;
		priv->stop_queue_credit[i] = 0;
		__netif_tx_unlock_bh(tx_queue);
	}

	return count;
}
#endif
/* pfe_eth_show_txavail
 *
 */
static ssize_t pfe_eth_show_txavail(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct pfe_eth_priv_s *priv = netdev_priv(to_net_dev(dev));
	ssize_t len = 0;
	int i;

	for (i = 0; i < emac_txq_cnt; i++) {
		struct netdev_queue *tx_queue = netdev_get_tx_queue(priv->ndev,
									i);

		__netif_tx_lock_bh(tx_queue);

		len += sprintf(buf + len, "%d",
				hif_lib_tx_avail(&priv->client, i));

		__netif_tx_unlock_bh(tx_queue);

		if (i == (emac_txq_cnt - 1))
			len += sprintf(buf + len, "\n");
		else
			len += sprintf(buf + len, " ");
	}

	return len;
}

/* pfe_eth_show_default_priority
 *
 */
static ssize_t pfe_eth_show_default_priority(struct device *dev,
					     struct device_attribute *attr,
						char *buf)
{
	struct pfe_eth_priv_s *priv = netdev_priv(to_net_dev(dev));
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&priv->lock, flags);
	rc = sprintf(buf, "%d\n", priv->default_priority);
	spin_unlock_irqrestore(&priv->lock, flags);

	return rc;
}

/* pfe_eth_set_default_priority
 *
 */

static ssize_t pfe_eth_set_default_priority(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct pfe_eth_priv_s *priv = netdev_priv(to_net_dev(dev));
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->default_priority = kstrtoul(buf, 0, 0);
	spin_unlock_irqrestore(&priv->lock, flags);

	return count;
}

static DEVICE_ATTR(txavail, 0444, pfe_eth_show_txavail, NULL);
static DEVICE_ATTR(default_priority, 0644, pfe_eth_show_default_priority,
			pfe_eth_set_default_priority);

#ifdef PFE_ETH_NAPI_STATS
static DEVICE_ATTR(napi_stats, 0644, pfe_eth_show_napi_stats,
			pfe_eth_set_napi_stats);
#endif

#ifdef PFE_ETH_TX_STATS
static DEVICE_ATTR(tx_stats, 0644, pfe_eth_show_tx_stats,
			pfe_eth_set_tx_stats);
#endif

/*
 * pfe_eth_sysfs_init
 *
 */
static int pfe_eth_sysfs_init(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	int err;

	/* Initialize the default values */

	/*
	 * By default, packets without conntrack will use this default high
	 * priority queue
	 */
	priv->default_priority = 15;

	/* Create our sysfs files */
	err = device_create_file(&ndev->dev, &dev_attr_default_priority);
	if (err) {
		netdev_err(ndev,
			   "failed to create default_priority sysfs files\n");
		goto err_priority;
	}

	err = device_create_file(&ndev->dev, &dev_attr_txavail);
	if (err) {
		netdev_err(ndev,
			   "failed to create default_priority sysfs files\n");
		goto err_txavail;
	}

#ifdef PFE_ETH_NAPI_STATS
	err = device_create_file(&ndev->dev, &dev_attr_napi_stats);
	if (err) {
		netdev_err(ndev, "failed to create napi stats sysfs files\n");
		goto err_napi;
	}
#endif

#ifdef PFE_ETH_TX_STATS
	err = device_create_file(&ndev->dev, &dev_attr_tx_stats);
	if (err) {
		netdev_err(ndev, "failed to create tx stats sysfs files\n");
		goto err_tx;
	}
#endif

	return 0;

#ifdef PFE_ETH_TX_STATS
err_tx:
#endif
#ifdef PFE_ETH_NAPI_STATS
	device_remove_file(&ndev->dev, &dev_attr_napi_stats);

err_napi:
#endif
	device_remove_file(&ndev->dev, &dev_attr_txavail);

err_txavail:
	device_remove_file(&ndev->dev, &dev_attr_default_priority);

err_priority:
	return -1;
}

/* pfe_eth_sysfs_exit
 *
 */
void pfe_eth_sysfs_exit(struct net_device *ndev)
{
#ifdef PFE_ETH_TX_STATS
	device_remove_file(&ndev->dev, &dev_attr_tx_stats);
#endif

#ifdef PFE_ETH_NAPI_STATS
	device_remove_file(&ndev->dev, &dev_attr_napi_stats);
#endif
	device_remove_file(&ndev->dev, &dev_attr_txavail);
	device_remove_file(&ndev->dev, &dev_attr_default_priority);
}

/*************************************************************************/
/*		ETHTOOL INTERCAE					 */
/*************************************************************************/

/*MTIP GEMAC */
static const struct fec_stat {
	char name[ETH_GSTRING_LEN];
	u16 offset;
} fec_stats[] = {
	/* RMON TX */
	{ "tx_dropped", RMON_T_DROP },
	{ "tx_packets", RMON_T_PACKETS },
	{ "tx_broadcast", RMON_T_BC_PKT },
	{ "tx_multicast", RMON_T_MC_PKT },
	{ "tx_crc_errors", RMON_T_CRC_ALIGN },
	{ "tx_undersize", RMON_T_UNDERSIZE },
	{ "tx_oversize", RMON_T_OVERSIZE },
	{ "tx_fragment", RMON_T_FRAG },
	{ "tx_jabber", RMON_T_JAB },
	{ "tx_collision", RMON_T_COL },
	{ "tx_64byte", RMON_T_P64 },
	{ "tx_65to127byte", RMON_T_P65TO127 },
	{ "tx_128to255byte", RMON_T_P128TO255 },
	{ "tx_256to511byte", RMON_T_P256TO511 },
	{ "tx_512to1023byte", RMON_T_P512TO1023 },
	{ "tx_1024to2047byte", RMON_T_P1024TO2047 },
	{ "tx_GTE2048byte", RMON_T_P_GTE2048 },
	{ "tx_octets", RMON_T_OCTETS },

	/* IEEE TX */
	{ "IEEE_tx_drop", IEEE_T_DROP },
	{ "IEEE_tx_frame_ok", IEEE_T_FRAME_OK },
	{ "IEEE_tx_1col", IEEE_T_1COL },
	{ "IEEE_tx_mcol", IEEE_T_MCOL },
	{ "IEEE_tx_def", IEEE_T_DEF },
	{ "IEEE_tx_lcol", IEEE_T_LCOL },
	{ "IEEE_tx_excol", IEEE_T_EXCOL },
	{ "IEEE_tx_macerr", IEEE_T_MACERR },
	{ "IEEE_tx_cserr", IEEE_T_CSERR },
	{ "IEEE_tx_sqe", IEEE_T_SQE },
	{ "IEEE_tx_fdxfc", IEEE_T_FDXFC },
	{ "IEEE_tx_octets_ok", IEEE_T_OCTETS_OK },

	/* RMON RX */
	{ "rx_packets", RMON_R_PACKETS },
	{ "rx_broadcast", RMON_R_BC_PKT },
	{ "rx_multicast", RMON_R_MC_PKT },
	{ "rx_crc_errors", RMON_R_CRC_ALIGN },
	{ "rx_undersize", RMON_R_UNDERSIZE },
	{ "rx_oversize", RMON_R_OVERSIZE },
	{ "rx_fragment", RMON_R_FRAG },
	{ "rx_jabber", RMON_R_JAB },
	{ "rx_64byte", RMON_R_P64 },
	{ "rx_65to127byte", RMON_R_P65TO127 },
	{ "rx_128to255byte", RMON_R_P128TO255 },
	{ "rx_256to511byte", RMON_R_P256TO511 },
	{ "rx_512to1023byte", RMON_R_P512TO1023 },
	{ "rx_1024to2047byte", RMON_R_P1024TO2047 },
	{ "rx_GTE2048byte", RMON_R_P_GTE2048 },
	{ "rx_octets", RMON_R_OCTETS },

	/* IEEE RX */
	{ "IEEE_rx_drop", IEEE_R_DROP },
	{ "IEEE_rx_frame_ok", IEEE_R_FRAME_OK },
	{ "IEEE_rx_crc", IEEE_R_CRC },
	{ "IEEE_rx_align", IEEE_R_ALIGN },
	{ "IEEE_rx_macerr", IEEE_R_MACERR },
	{ "IEEE_rx_fdxfc", IEEE_R_FDXFC },
	{ "IEEE_rx_octets_ok", IEEE_R_OCTETS_OK },
};

static void pfe_eth_fill_stats(struct net_device *ndev, struct ethtool_stats
				*stats, u64 *data)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	int i;

	for (i = 0; i < ARRAY_SIZE(fec_stats); i++)
		data[i] = readl(priv->EMAC_baseaddr + fec_stats[i].offset);
}

static void pfe_eth_gstrings(struct net_device *netdev,
			     u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(fec_stats); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       fec_stats[i].name, ETH_GSTRING_LEN);
		break;
	}
}

static int pfe_eth_stats_count(struct net_device *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(fec_stats);
	default:
		return -EOPNOTSUPP;
	}
}

/*
 * pfe_eth_gemac_reglen - Return the length of the register structure.
 *
 */
static int pfe_eth_gemac_reglen(struct net_device *ndev)
{
	pr_info("%s()\n", __func__);
	return (sizeof(gemac_regs) / sizeof(u32));
}

/*
 * pfe_eth_gemac_get_regs - Return the gemac register structure.
 *
 */
static void  pfe_eth_gemac_get_regs(struct net_device *ndev, struct ethtool_regs
					*regs, void *regbuf)
{
	int i;

	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	u32 *buf = (u32 *)regbuf;

	pr_info("%s()\n", __func__);
	for (i = 0; i < sizeof(gemac_regs) / sizeof(u32); i++)
		buf[i] = readl(priv->EMAC_baseaddr + gemac_regs[i]);
}

/*
 * pfe_eth_set_wol - Set the magic packet option, in WoL register.
 *
 */
static int pfe_eth_set_wol(struct net_device *ndev, struct ethtool_wolinfo *wol)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	if (wol->wolopts & ~WAKE_MAGIC)
		return -EOPNOTSUPP;

	/* for MTIP we store wol->wolopts */
	priv->wol = wol->wolopts;

	device_set_wakeup_enable(&ndev->dev, wol->wolopts & WAKE_MAGIC);

	return 0;
}

/*
 *
 * pfe_eth_get_wol - Get the WoL options.
 *
 */
static void pfe_eth_get_wol(struct net_device *ndev, struct ethtool_wolinfo
				*wol)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;

	if (priv->wol & WAKE_MAGIC)
		wol->wolopts = WAKE_MAGIC;

	memset(&wol->sopass, 0, sizeof(wol->sopass));
}

/*
 * pfe_eth_get_drvinfo -  Fills in the drvinfo structure with some basic info
 *
 */
static void pfe_eth_get_drvinfo(struct net_device *ndev, struct ethtool_drvinfo
				*drvinfo)
{
	strlcpy(drvinfo->driver, DRV_NAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, DRV_VERSION, sizeof(drvinfo->version));
	strlcpy(drvinfo->fw_version, "N/A", sizeof(drvinfo->fw_version));
	strlcpy(drvinfo->bus_info, "N/A", sizeof(drvinfo->bus_info));
}

/*
 * pfe_eth_set_settings - Used to send commands to PHY.
 *
 */
static int pfe_eth_set_settings(struct net_device *ndev,
				const struct ethtool_link_ksettings *cmd)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	struct phy_device *phydev = priv->phydev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_ksettings_set(phydev, cmd);
}

/*
 * pfe_eth_getsettings - Return the current settings in the ethtool_cmd
 * structure.
 *
 */
static int pfe_eth_get_settings(struct net_device *ndev,
				struct ethtool_link_ksettings *cmd)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	struct phy_device *phydev = priv->phydev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_ksettings_get(phydev, cmd);
}

/*
 * pfe_eth_get_msglevel - Gets the debug message mask.
 *
 */
static uint32_t pfe_eth_get_msglevel(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	return priv->msg_enable;
}

/*
 * pfe_eth_set_msglevel - Sets the debug message mask.
 *
 */
static void pfe_eth_set_msglevel(struct net_device *ndev, uint32_t data)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	priv->msg_enable = data;
}

#define HIF_RX_COAL_MAX_CLKS		(~(1 << 31))
#define HIF_RX_COAL_CLKS_PER_USEC	(pfe->ctrl.sys_clk / 1000)
#define HIF_RX_COAL_MAX_USECS		(HIF_RX_COAL_MAX_CLKS	/ \
						HIF_RX_COAL_CLKS_PER_USEC)

/*
 * pfe_eth_set_coalesce - Sets rx interrupt coalescing timer.
 *
 */
static int pfe_eth_set_coalesce(struct net_device *ndev,
				struct ethtool_coalesce *ec)
{
	if (ec->rx_coalesce_usecs > HIF_RX_COAL_MAX_USECS)
		return -EINVAL;

	if (!ec->rx_coalesce_usecs) {
		writel(0, HIF_INT_COAL);
		return 0;
	}

	writel((ec->rx_coalesce_usecs * HIF_RX_COAL_CLKS_PER_USEC) |
			HIF_INT_COAL_ENABLE, HIF_INT_COAL);

	return 0;
}

/*
 * pfe_eth_get_coalesce - Gets rx interrupt coalescing timer value.
 *
 */
static int pfe_eth_get_coalesce(struct net_device *ndev,
				struct ethtool_coalesce *ec)
{
	int reg_val = readl(HIF_INT_COAL);

	if (reg_val & HIF_INT_COAL_ENABLE)
		ec->rx_coalesce_usecs = (reg_val & HIF_RX_COAL_MAX_CLKS) /
						HIF_RX_COAL_CLKS_PER_USEC;
	else
		ec->rx_coalesce_usecs = 0;

	return 0;
}

/*
 * pfe_eth_set_pauseparam - Sets pause parameters
 *
 */
static int pfe_eth_set_pauseparam(struct net_device *ndev,
				  struct ethtool_pauseparam *epause)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	if (epause->tx_pause != epause->rx_pause) {
		netdev_info(ndev,
			    "hardware only support enable/disable both tx and rx\n");
		return -EINVAL;
	}

	priv->pause_flag = 0;
	priv->pause_flag |= epause->rx_pause ? PFE_PAUSE_FLAG_ENABLE : 0;
	priv->pause_flag |= epause->autoneg ? PFE_PAUSE_FLAG_AUTONEG : 0;

	if (epause->rx_pause || epause->autoneg) {
		gemac_enable_pause_rx(priv->EMAC_baseaddr);
		writel((readl(priv->GPI_baseaddr + GPI_TX_PAUSE_TIME) |
					EGPI_PAUSE_ENABLE),
				priv->GPI_baseaddr + GPI_TX_PAUSE_TIME);
		if (priv->phydev) {
			priv->phydev->supported |= ADVERTISED_Pause |
							ADVERTISED_Asym_Pause;
			priv->phydev->advertising |= ADVERTISED_Pause |
							ADVERTISED_Asym_Pause;
		}
	} else {
		gemac_disable_pause_rx(priv->EMAC_baseaddr);
		writel((readl(priv->GPI_baseaddr + GPI_TX_PAUSE_TIME) &
					~EGPI_PAUSE_ENABLE),
				priv->GPI_baseaddr + GPI_TX_PAUSE_TIME);
		if (priv->phydev) {
			priv->phydev->supported &= ~(ADVERTISED_Pause |
							ADVERTISED_Asym_Pause);
			priv->phydev->advertising &= ~(ADVERTISED_Pause |
							ADVERTISED_Asym_Pause);
		}
	}

	return 0;
}

/*
 * pfe_eth_get_pauseparam - Gets pause parameters
 *
 */
static void pfe_eth_get_pauseparam(struct net_device *ndev,
				   struct ethtool_pauseparam *epause)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	epause->autoneg = (priv->pause_flag & PFE_PAUSE_FLAG_AUTONEG) != 0;
	epause->tx_pause = (priv->pause_flag & PFE_PAUSE_FLAG_ENABLE) != 0;
	epause->rx_pause = epause->tx_pause;
}

/*
 * pfe_eth_get_hash
 */
#define PFE_HASH_BITS	6		/* #bits in hash */
#define CRC32_POLY	0xEDB88320

static int pfe_eth_get_hash(u8 *addr)
{
	unsigned int i, bit, data, crc, hash;

	/* calculate crc32 value of mac address */
	crc = 0xffffffff;

	for (i = 0; i < 6; i++) {
		data = addr[i];
		for (bit = 0; bit < 8; bit++, data >>= 1) {
			crc = (crc >> 1) ^
				(((crc ^ data) & 1) ? CRC32_POLY : 0);
		}
	}

	/*
	 * only upper 6 bits (PFE_HASH_BITS) are used
	 * which point to specific bit in the hash registers
	 */
	hash = (crc >> (32 - PFE_HASH_BITS)) & 0x3f;

	return hash;
}

const struct ethtool_ops pfe_ethtool_ops = {
	.get_drvinfo = pfe_eth_get_drvinfo,
	.get_regs_len = pfe_eth_gemac_reglen,
	.get_regs = pfe_eth_gemac_get_regs,
	.get_link = ethtool_op_get_link,
	.get_wol  = pfe_eth_get_wol,
	.set_wol  = pfe_eth_set_wol,
	.set_pauseparam = pfe_eth_set_pauseparam,
	.get_pauseparam = pfe_eth_get_pauseparam,
	.get_strings = pfe_eth_gstrings,
	.get_sset_count = pfe_eth_stats_count,
	.get_ethtool_stats = pfe_eth_fill_stats,
	.get_msglevel = pfe_eth_get_msglevel,
	.set_msglevel = pfe_eth_set_msglevel,
	.set_coalesce = pfe_eth_set_coalesce,
	.get_coalesce = pfe_eth_get_coalesce,
	.get_link_ksettings = pfe_eth_get_settings,
	.set_link_ksettings = pfe_eth_set_settings,
};

/* pfe_eth_mdio_reset
 */
int pfe_eth_mdio_reset(struct mii_bus *bus)
{
	struct pfe_eth_priv_s *priv = (struct pfe_eth_priv_s *)bus->priv;
	u32 phy_speed;

	netif_info(priv, hw, priv->ndev, "%s\n", __func__);

	mutex_lock(&bus->mdio_lock);

	/*
	 * Set MII speed to 2.5 MHz (= clk_get_rate() / 2 * phy_speed)
	 *
	 * The formula for FEC MDC is 'ref_freq / (MII_SPEED x 2)' while
	 * for ENET-MAC is 'ref_freq / ((MII_SPEED + 1) x 2)'.
	 */
	phy_speed = (DIV_ROUND_UP((pfe->ctrl.sys_clk * 1000), 4000000)
		     << EMAC_MII_SPEED_SHIFT);
	phy_speed |= EMAC_HOLDTIME(0x5);
	__raw_writel(phy_speed, priv->PHY_baseaddr + EMAC_MII_CTRL_REG);

	mutex_unlock(&bus->mdio_lock);

	return 0;
}

/* pfe_eth_gemac_phy_timeout
 *
 */
static int pfe_eth_gemac_phy_timeout(struct pfe_eth_priv_s *priv, int timeout)
{
	while (!(__raw_readl(priv->PHY_baseaddr + EMAC_IEVENT_REG) &
			EMAC_IEVENT_MII)) {
		if (timeout-- <= 0)
			return -1;
		usleep_range(10, 20);
	}
	__raw_writel(EMAC_IEVENT_MII, priv->PHY_baseaddr + EMAC_IEVENT_REG);
	return 0;
}

static int pfe_eth_mdio_mux(u8 muxval)
{
	struct i2c_adapter *a;
	struct i2c_msg msg;
	unsigned char buf[2];
	int ret;

	a = i2c_get_adapter(0);
	if (!a)
		return -ENODEV;

	/* set bit 1 (the second bit) of chip at 0x09, register 0x13 */
	buf[0] = 0x54; /* reg number */
	buf[1] = (muxval << 6) | 0x3; /* data */
	msg.addr = 0x66;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0;
	ret = i2c_transfer(a, &msg, 1);
	i2c_put_adapter(a);
	if (ret != 1)
		return -ENODEV;
	return 0;
}

static int pfe_eth_mdio_write_addr(struct mii_bus *bus, int mii_id,
				   int dev_addr, int regnum)
{
	struct pfe_eth_priv_s *priv = (struct pfe_eth_priv_s *)bus->priv;

	__raw_writel(EMAC_MII_DATA_PA(mii_id) |
		     EMAC_MII_DATA_RA(dev_addr) |
		     EMAC_MII_DATA_TA | EMAC_MII_DATA(regnum),
		     priv->PHY_baseaddr + EMAC_MII_DATA_REG);

	if (pfe_eth_gemac_phy_timeout(priv, EMAC_MDIO_TIMEOUT)) {
		netdev_err(priv->ndev, "%s: phy MDIO address write timeout\n",
			   __func__);
		return -1;
	}

	return 0;
}

static int pfe_eth_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			      u16 value)
{
	struct pfe_eth_priv_s *priv = (struct pfe_eth_priv_s *)bus->priv;

	/*To access external PHYs on QDS board mux needs to be configured*/
	if ((mii_id) && (pfe->mdio_muxval[mii_id]))
		pfe_eth_mdio_mux(pfe->mdio_muxval[mii_id]);

	if (regnum & MII_ADDR_C45) {
		pfe_eth_mdio_write_addr(bus, mii_id, (regnum >> 16) & 0x1f,
					regnum & 0xffff);
		__raw_writel(EMAC_MII_DATA_OP_CL45_WR |
			     EMAC_MII_DATA_PA(mii_id) |
			     EMAC_MII_DATA_RA((regnum >> 16) & 0x1f) |
			     EMAC_MII_DATA_TA | EMAC_MII_DATA(value),
			     priv->PHY_baseaddr + EMAC_MII_DATA_REG);
	} else {
		/* start a write op */
		__raw_writel(EMAC_MII_DATA_ST | EMAC_MII_DATA_OP_WR |
			     EMAC_MII_DATA_PA(mii_id) |
			     EMAC_MII_DATA_RA(regnum) |
			     EMAC_MII_DATA_TA | EMAC_MII_DATA(value),
			     priv->PHY_baseaddr + EMAC_MII_DATA_REG);
	}

	if (pfe_eth_gemac_phy_timeout(priv, EMAC_MDIO_TIMEOUT)) {
		netdev_err(priv->ndev, "%s: phy MDIO write timeout\n",
			   __func__);
		return -1;
	}
	netif_info(priv, hw, priv->ndev, "%s: phy %x reg %x val %x\n", __func__,
		   mii_id, regnum, value);

	return 0;
}

static int pfe_eth_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct pfe_eth_priv_s *priv = (struct pfe_eth_priv_s *)bus->priv;
	u16 value = 0;

	/*To access external PHYs on QDS board mux needs to be configured*/
	if ((mii_id) && (pfe->mdio_muxval[mii_id]))
		pfe_eth_mdio_mux(pfe->mdio_muxval[mii_id]);

	if (regnum & MII_ADDR_C45) {
		pfe_eth_mdio_write_addr(bus, mii_id, (regnum >> 16) & 0x1f,
					regnum & 0xffff);
		__raw_writel(EMAC_MII_DATA_OP_CL45_RD |
			     EMAC_MII_DATA_PA(mii_id) |
			     EMAC_MII_DATA_RA((regnum >> 16) & 0x1f) |
			     EMAC_MII_DATA_TA,
			     priv->PHY_baseaddr + EMAC_MII_DATA_REG);
	} else {
		/* start a read op */
		__raw_writel(EMAC_MII_DATA_ST | EMAC_MII_DATA_OP_RD |
			     EMAC_MII_DATA_PA(mii_id) |
			     EMAC_MII_DATA_RA(regnum) |
			     EMAC_MII_DATA_TA, priv->PHY_baseaddr +
			     EMAC_MII_DATA_REG);
	}

	if (pfe_eth_gemac_phy_timeout(priv, EMAC_MDIO_TIMEOUT)) {
		netdev_err(priv->ndev, "%s: phy MDIO read timeout\n", __func__);
		return -1;
	}

	value = EMAC_MII_DATA(__raw_readl(priv->PHY_baseaddr +
						EMAC_MII_DATA_REG));
	netif_info(priv, hw, priv->ndev, "%s: phy %x reg %x val %x\n", __func__,
		   mii_id, regnum, value);
	return value;
}

static int pfe_eth_mdio_init(struct pfe_eth_priv_s *priv,
			     struct ls1012a_mdio_platform_data *minfo)
{
	struct mii_bus *bus;
	int rc;

	netif_info(priv, drv, priv->ndev, "%s\n", __func__);
	pr_info("%s\n", __func__);

	bus = mdiobus_alloc();
	if (!bus) {
		netdev_err(priv->ndev, "mdiobus_alloc() failed\n");
		rc = -ENOMEM;
		goto err0;
	}

	bus->name = "ls1012a MDIO Bus";
	bus->read = &pfe_eth_mdio_read;
	bus->write = &pfe_eth_mdio_write;
	bus->reset = &pfe_eth_mdio_reset;
	snprintf(bus->id, MII_BUS_ID_SIZE, "ls1012a-%x", priv->id);
	bus->priv = priv;

	bus->phy_mask = minfo->phy_mask;
	priv->mdc_div = minfo->mdc_div;

	if (!priv->mdc_div)
		priv->mdc_div = 64;

	bus->irq[0] = minfo->irq[0];

	bus->parent = priv->pfe->dev;

	netif_info(priv, drv, priv->ndev, "%s: mdc_div: %d, phy_mask: %x\n",
		   __func__, priv->mdc_div, bus->phy_mask);
	rc = mdiobus_register(bus);
	if (rc) {
		netdev_err(priv->ndev, "mdiobus_register(%s) failed\n",
			   bus->name);
		goto err1;
	}

	priv->mii_bus = bus;
	pfe_eth_mdio_reset(bus);

	return 0;

err1:
	mdiobus_free(bus);
err0:
	return rc;
}

/* pfe_eth_mdio_exit
 */
static void pfe_eth_mdio_exit(struct mii_bus *bus)
{
	if (!bus)
		return;

	netif_info((struct pfe_eth_priv_s *)bus->priv, drv, ((struct
			pfe_eth_priv_s *)(bus->priv))->ndev, "%s\n", __func__);

	mdiobus_unregister(bus);
	mdiobus_free(bus);
}

/* pfe_get_phydev_speed
 */
static int pfe_get_phydev_speed(struct phy_device *phydev)
{
	switch (phydev->speed) {
	case 10:
			return SPEED_10M;
	case 100:
			return SPEED_100M;
	case 1000:
	default:
			return SPEED_1000M;
	}
}

/* pfe_set_rgmii_speed
 */
#define RGMIIPCR	0x434
/* RGMIIPCR bit definitions*/
#define SCFG_RGMIIPCR_EN_AUTO           (0x00000008)
#define SCFG_RGMIIPCR_SETSP_1000M       (0x00000004)
#define SCFG_RGMIIPCR_SETSP_100M        (0x00000000)
#define SCFG_RGMIIPCR_SETSP_10M         (0x00000002)
#define SCFG_RGMIIPCR_SETFD             (0x00000001)

static void pfe_set_rgmii_speed(struct phy_device *phydev)
{
	u32 rgmii_pcr;

	regmap_read(pfe->scfg, RGMIIPCR, &rgmii_pcr);
	rgmii_pcr  &= ~(SCFG_RGMIIPCR_SETSP_1000M | SCFG_RGMIIPCR_SETSP_10M);

	switch (phydev->speed) {
	case 10:
			rgmii_pcr |= SCFG_RGMIIPCR_SETSP_10M;
			break;
	case 1000:
			rgmii_pcr |= SCFG_RGMIIPCR_SETSP_1000M;
			break;
	case 100:
	default:
			/* Default is 100M */
			break;
	}
	regmap_write(pfe->scfg, RGMIIPCR, rgmii_pcr);
}

/* pfe_get_phydev_duplex
 */
static int pfe_get_phydev_duplex(struct phy_device *phydev)
{
	/*return (phydev->duplex == DUPLEX_HALF) ? DUP_HALF:DUP_FULL ; */
	return DUPLEX_FULL;
}

/* pfe_eth_adjust_link
 */
static void pfe_eth_adjust_link(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	unsigned long flags;
	struct phy_device *phydev = priv->phydev;
	int new_state = 0;

	netif_info(priv, drv, ndev, "%s\n", __func__);

	spin_lock_irqsave(&priv->lock, flags);

	if (phydev->link) {
		/*
		 * Now we make sure that we can be in full duplex mode.
		 * If not, we operate in half-duplex mode.
		 */
		if (phydev->duplex != priv->oldduplex) {
			new_state = 1;
			gemac_set_duplex(priv->EMAC_baseaddr,
					 pfe_get_phydev_duplex(phydev));
			priv->oldduplex = phydev->duplex;
		}

		if (phydev->speed != priv->oldspeed) {
			new_state = 1;
			gemac_set_speed(priv->EMAC_baseaddr,
					pfe_get_phydev_speed(phydev));
			if (priv->einfo->mii_config == PHY_INTERFACE_MODE_RGMII)
				pfe_set_rgmii_speed(phydev);
			priv->oldspeed = phydev->speed;
		}

		if (!priv->oldlink) {
			new_state = 1;
			priv->oldlink = 1;
		}

	} else if (priv->oldlink) {
		new_state = 1;
		priv->oldlink = 0;
		priv->oldspeed = 0;
		priv->oldduplex = -1;
	}

	if (new_state && netif_msg_link(priv))
		phy_print_status(phydev);

	spin_unlock_irqrestore(&priv->lock, flags);
}

/* pfe_phy_exit
 */
static void pfe_phy_exit(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	netif_info(priv, drv, ndev, "%s\n", __func__);

	phy_disconnect(priv->phydev);
	priv->phydev = NULL;
}

/* pfe_eth_stop
 */
static void pfe_eth_stop(struct net_device *ndev, int wake)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	netif_info(priv, drv, ndev, "%s\n", __func__);

	if (wake) {
		gemac_tx_disable(priv->EMAC_baseaddr);
	} else {
		gemac_disable(priv->EMAC_baseaddr);
		gpi_disable(priv->GPI_baseaddr);

		if (priv->phydev)
			phy_stop(priv->phydev);
	}
}

/* pfe_eth_start
 */
static int pfe_eth_start(struct pfe_eth_priv_s *priv)
{
	netif_info(priv, drv, priv->ndev, "%s\n", __func__);

	if (priv->phydev)
		phy_start(priv->phydev);

	gpi_enable(priv->GPI_baseaddr);
	gemac_enable(priv->EMAC_baseaddr);

	return 0;
}

/*
 * Configure on chip serdes through mdio
 */
static void ls1012a_configure_serdes(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = pfe->eth.eth_priv[0];
	int sgmii_2500 = 0;
	struct mii_bus *bus = priv->mii_bus;

	if (priv->einfo->mii_config == PHY_INTERFACE_MODE_SGMII_2500)
		sgmii_2500 = 1;

	netif_info(priv, drv, ndev, "%s\n", __func__);
	/* PCS configuration done with corresponding GEMAC */

	pfe_eth_mdio_read(bus, 0, 0);
	pfe_eth_mdio_read(bus, 0, 1);

       /*These settings taken from validtion team */
	pfe_eth_mdio_write(bus, 0, 0x0, 0x8000);
	if (sgmii_2500) {
		pfe_eth_mdio_write(bus, 0, 0x14, 0x9);
		pfe_eth_mdio_write(bus, 0, 0x4, 0x4001);
		pfe_eth_mdio_write(bus, 0, 0x12, 0xa120);
		pfe_eth_mdio_write(bus, 0, 0x13, 0x7);
	} else {
		pfe_eth_mdio_write(bus, 0, 0x14, 0xb);
		pfe_eth_mdio_write(bus, 0, 0x4, 0x1a1);
		pfe_eth_mdio_write(bus, 0, 0x12, 0x400);
		pfe_eth_mdio_write(bus, 0, 0x13, 0x0);
	}

	pfe_eth_mdio_write(bus, 0, 0x0, 0x1140);
}

/*
 * pfe_phy_init
 *
 */
static int pfe_phy_init(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	struct phy_device *phydev;
	char phy_id[MII_BUS_ID_SIZE + 3];
	char bus_id[MII_BUS_ID_SIZE];
	phy_interface_t interface;

	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	snprintf(bus_id, MII_BUS_ID_SIZE, "ls1012a-%d", 0);
	snprintf(phy_id, MII_BUS_ID_SIZE + 3, PHY_ID_FMT, bus_id,
		 priv->einfo->phy_id);

	netif_info(priv, drv, ndev, "%s: %s\n", __func__, phy_id);
	interface = priv->einfo->mii_config;
	if ((interface == PHY_INTERFACE_MODE_SGMII) ||
	    (interface == PHY_INTERFACE_MODE_SGMII_2500)) {
		/*Configure SGMII PCS */
		if (pfe->scfg) {
			/*Config MDIO from serdes */
			regmap_write(pfe->scfg, 0x484, 0x00000000);
		}
		ls1012a_configure_serdes(ndev);
	}

	if (pfe->scfg) {
		/*Config MDIO from PAD */
		regmap_write(pfe->scfg, 0x484, 0x80000000);
	}

	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;
	pr_info("%s interface %x\n", __func__, interface);
	phydev = phy_connect(ndev, phy_id, &pfe_eth_adjust_link, interface);

	if (IS_ERR(phydev)) {
		netdev_err(ndev, "phy_connect() failed\n");
		return PTR_ERR(phydev);
	}

	priv->phydev = phydev;
	phydev->irq = PHY_POLL;

	return 0;
}

/* pfe_gemac_init
 */
static int pfe_gemac_init(struct pfe_eth_priv_s *priv)
{
	struct gemac_cfg cfg;

	netif_info(priv, ifup, priv->ndev, "%s\n", __func__);

	cfg.speed = SPEED_1000M;
	cfg.duplex = DUPLEX_FULL;

	gemac_set_config(priv->EMAC_baseaddr, &cfg);
	gemac_allow_broadcast(priv->EMAC_baseaddr);
	gemac_enable_1536_rx(priv->EMAC_baseaddr);
	gemac_enable_rx_jmb(priv->EMAC_baseaddr);
	gemac_enable_stacked_vlan(priv->EMAC_baseaddr);
	gemac_enable_pause_rx(priv->EMAC_baseaddr);
	gemac_set_bus_width(priv->EMAC_baseaddr, 64);

	/*GEM will perform checksum verifications*/
	if (priv->ndev->features & NETIF_F_RXCSUM)
		gemac_enable_rx_checksum_offload(priv->EMAC_baseaddr);
	else
		gemac_disable_rx_checksum_offload(priv->EMAC_baseaddr);

	return 0;
}

/* pfe_eth_event_handler
 */
static int pfe_eth_event_handler(void *data, int event, int qno)
{
	struct pfe_eth_priv_s *priv = data;

	switch (event) {
	case EVENT_RX_PKT_IND:

		if (qno == 0) {
			if (napi_schedule_prep(&priv->high_napi)) {
				netif_info(priv, intr, priv->ndev,
					   "%s: schedule high prio poll\n"
					   , __func__);

#ifdef PFE_ETH_NAPI_STATS
				priv->napi_counters[NAPI_SCHED_COUNT]++;
#endif

				__napi_schedule(&priv->high_napi);
			}
		} else if (qno == 1) {
			if (napi_schedule_prep(&priv->low_napi)) {
				netif_info(priv, intr, priv->ndev,
					   "%s: schedule low prio poll\n"
					   , __func__);

#ifdef PFE_ETH_NAPI_STATS
				priv->napi_counters[NAPI_SCHED_COUNT]++;
#endif
				__napi_schedule(&priv->low_napi);
			}
		} else if (qno == 2) {
			if (napi_schedule_prep(&priv->lro_napi)) {
				netif_info(priv, intr, priv->ndev,
					   "%s: schedule lro prio poll\n"
					   , __func__);

#ifdef PFE_ETH_NAPI_STATS
				priv->napi_counters[NAPI_SCHED_COUNT]++;
#endif
				__napi_schedule(&priv->lro_napi);
			}
		}

		break;

	case EVENT_TXDONE_IND:
		pfe_eth_flush_tx(priv);
		hif_lib_event_handler_start(&priv->client, EVENT_TXDONE_IND, 0);
		break;
	case EVENT_HIGH_RX_WM:
	default:
		break;
	}

	return 0;
}

/* pfe_eth_open
 */
static int pfe_eth_open(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	struct hif_client_s *client;
	int rc;

	netif_info(priv, ifup, ndev, "%s\n", __func__);

	/* Register client driver with HIF */
	client = &priv->client;
	memset(client, 0, sizeof(*client));
	client->id = PFE_CL_GEM0 + priv->id;
	client->tx_qn = emac_txq_cnt;
	client->rx_qn = EMAC_RXQ_CNT;
	client->priv = priv;
	client->pfe = priv->pfe;
	client->event_handler = pfe_eth_event_handler;

	client->tx_qsize = EMAC_TXQ_DEPTH;
	client->rx_qsize = EMAC_RXQ_DEPTH;

	rc = hif_lib_client_register(client);
	if (rc) {
		netdev_err(ndev, "%s: hif_lib_client_register(%d) failed\n",
			   __func__, client->id);
		goto err0;
	}

	netif_info(priv, drv, ndev, "%s: registered client: %p\n", __func__,
		   client);

	pfe_gemac_init(priv);

	if (!is_valid_ether_addr(ndev->dev_addr)) {
		netdev_err(ndev, "%s: invalid MAC address\n", __func__);
		rc = -EADDRNOTAVAIL;
		goto err1;
	}

	gemac_set_laddrN(priv->EMAC_baseaddr,
			 (struct pfe_mac_addr *)ndev->dev_addr, 1);

	napi_enable(&priv->high_napi);
	napi_enable(&priv->low_napi);
	napi_enable(&priv->lro_napi);

	rc = pfe_eth_start(priv);

	netif_tx_wake_all_queues(ndev);

	return rc;

err1:
	hif_lib_client_unregister(&priv->client);

err0:
	return rc;
}

/*
 *  pfe_eth_shutdown
 */
int pfe_eth_shutdown(struct net_device *ndev, int wake)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	int i, qstatus;
	unsigned long next_poll = jiffies + 1, end = jiffies +
				(TX_POLL_TIMEOUT_MS * HZ) / 1000;
	int tx_pkts, prv_tx_pkts;

	netif_info(priv, ifdown, ndev, "%s\n", __func__);

	for (i = 0; i < emac_txq_cnt; i++)
		hrtimer_cancel(&priv->fast_tx_timeout[i].timer);

	netif_tx_stop_all_queues(ndev);

	do {
		tx_pkts = 0;
		pfe_eth_flush_tx(priv);

		for (i = 0; i < emac_txq_cnt; i++)
			tx_pkts += hif_lib_tx_pending(&priv->client, i);

		if (tx_pkts) {
			/*Don't wait forever, break if we cross max timeout */
			if (time_after(jiffies, end)) {
				pr_err(
					"(%s)Tx is not complete after %dmsec\n",
					ndev->name, TX_POLL_TIMEOUT_MS);
				break;
			}

			pr_info("%s : (%s) Waiting for tx packets to free. Pending tx pkts = %d.\n"
				, __func__, ndev->name, tx_pkts);
			if (need_resched())
				schedule();
		}

	} while (tx_pkts);

	end = jiffies + (TX_POLL_TIMEOUT_MS * HZ) / 1000;

	prv_tx_pkts = tmu_pkts_processed(priv->id);
	/*
	 * Wait till TMU transmits all pending packets
	 * poll tmu_qstatus and pkts processed by TMU for every 10ms
	 * Consider TMU is busy, If we see TMU qeueu pending or any packets
	 * processed by TMU
	 */
	while (1) {
		if (time_after(jiffies, next_poll)) {
			tx_pkts = tmu_pkts_processed(priv->id);
			qstatus = tmu_qstatus(priv->id) & 0x7ffff;

			if (!qstatus && (tx_pkts == prv_tx_pkts))
				break;
			/* Don't wait forever, break if we cross max
			 * timeout(TX_POLL_TIMEOUT_MS)
			 */
			if (time_after(jiffies, end)) {
				pr_err("TMU%d is busy after %dmsec\n",
				       priv->id, TX_POLL_TIMEOUT_MS);
				break;
			}
			prv_tx_pkts = tx_pkts;
			next_poll++;
		}
		if (need_resched())
			schedule();
	}
	/* Wait for some more time to complete transmitting packet if any */
	next_poll = jiffies + 1;
	while (1) {
		if (time_after(jiffies, next_poll))
			break;
		if (need_resched())
			schedule();
	}

	pfe_eth_stop(ndev, wake);

	napi_disable(&priv->lro_napi);
	napi_disable(&priv->low_napi);
	napi_disable(&priv->high_napi);

	hif_lib_client_unregister(&priv->client);

	return 0;
}

/* pfe_eth_close
 *
 */
static int pfe_eth_close(struct net_device *ndev)
{
	pfe_eth_shutdown(ndev, 0);

	return 0;
}

/* pfe_eth_suspend
 *
 * return value : 1 if netdevice is configured to wakeup system
 *                0 otherwise
 */
int pfe_eth_suspend(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	int retval = 0;

	if (priv->wol) {
		gemac_set_wol(priv->EMAC_baseaddr, priv->wol);
		retval = 1;
	}
	pfe_eth_shutdown(ndev, priv->wol);

	return retval;
}

/* pfe_eth_resume
 *
 */
int pfe_eth_resume(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	if (priv->wol)
		gemac_set_wol(priv->EMAC_baseaddr, 0);
	gemac_tx_enable(priv->EMAC_baseaddr);

	return pfe_eth_open(ndev);
}

/* pfe_eth_get_queuenum
 */
static int pfe_eth_get_queuenum(struct pfe_eth_priv_s *priv, struct sk_buff
					*skb)
{
	int queuenum = 0;
	unsigned long flags;

	/* Get the Fast Path queue number */
	/*
	 * Use conntrack mark (if conntrack exists), then packet mark (if any),
	 * then fallback to default
	 */
#if defined(CONFIG_IP_NF_CONNTRACK_MARK) || defined(CONFIG_NF_CONNTRACK_MARK)
	if (skb->_nfct) {
		enum ip_conntrack_info cinfo;
		struct nf_conn *ct;

		ct = nf_ct_get(skb, &cinfo);

		if (ct) {
			u32 connmark;

			connmark = ct->mark;

			if ((connmark & 0x80000000) && priv->id != 0)
				connmark >>= 16;

			queuenum = connmark & EMAC_QUEUENUM_MASK;
		}
	} else  {/* continued after #endif ... */
#endif
		if (skb->mark) {
			queuenum = skb->mark & EMAC_QUEUENUM_MASK;
		} else {
			spin_lock_irqsave(&priv->lock, flags);
			queuenum = priv->default_priority & EMAC_QUEUENUM_MASK;
			spin_unlock_irqrestore(&priv->lock, flags);
		}
#if defined(CONFIG_IP_NF_CONNTRACK_MARK) || defined(CONFIG_NF_CONNTRACK_MARK)
	}
#endif
	return queuenum;
}

/* pfe_eth_might_stop_tx
 *
 */
static int pfe_eth_might_stop_tx(struct pfe_eth_priv_s *priv, int queuenum,
				 struct netdev_queue *tx_queue,
				 unsigned int n_desc,
				 unsigned int n_segs)
{
	ktime_t kt;

	if (unlikely((__hif_tx_avail(&pfe->hif) < n_desc) ||
		     (hif_lib_tx_avail(&priv->client, queuenum) < n_desc) ||
	(hif_lib_tx_credit_avail(pfe, priv->id, queuenum) < n_segs))) {
#ifdef PFE_ETH_TX_STATS
		if (__hif_tx_avail(&pfe->hif) < n_desc) {
			priv->stop_queue_hif[queuenum]++;
		} else if (hif_lib_tx_avail(&priv->client, queuenum) < n_desc) {
			priv->stop_queue_hif_client[queuenum]++;
		} else if (hif_lib_tx_credit_avail(pfe, priv->id, queuenum) <
			n_segs) {
			priv->stop_queue_credit[queuenum]++;
		}
		priv->stop_queue_total[queuenum]++;
#endif
		netif_tx_stop_queue(tx_queue);

		kt = ktime_set(0, LS1012A_TX_FAST_RECOVERY_TIMEOUT_MS *
				NSEC_PER_MSEC);
		hrtimer_start(&priv->fast_tx_timeout[queuenum].timer, kt,
			      HRTIMER_MODE_REL);
		return -1;
	} else {
		return 0;
	}
}

#define SA_MAX_OP 2
/* pfe_hif_send_packet
 *
 * At this level if TX fails we drop the packet
 */
static void pfe_hif_send_packet(struct sk_buff *skb, struct  pfe_eth_priv_s
					*priv, int queuenum)
{
	struct skb_shared_info *sh = skb_shinfo(skb);
	unsigned int nr_frags;
	u32 ctrl = 0;

	netif_info(priv, tx_queued, priv->ndev, "%s\n", __func__);

	if (skb_is_gso(skb)) {
		priv->stats.tx_dropped++;
		return;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		ctrl = HIF_CTRL_TX_CHECKSUM;

	nr_frags = sh->nr_frags;

	if (nr_frags) {
		skb_frag_t *f;
		int i;

		__hif_lib_xmit_pkt(&priv->client, queuenum, skb->data,
				   skb_headlen(skb), ctrl, HIF_FIRST_BUFFER,
				   skb);

		for (i = 0; i < nr_frags - 1; i++) {
			f = &sh->frags[i];
			__hif_lib_xmit_pkt(&priv->client, queuenum,
					   skb_frag_address(f),
					   skb_frag_size(f),
					   0x0, 0x0, skb);
		}

		f = &sh->frags[i];

		__hif_lib_xmit_pkt(&priv->client, queuenum,
				   skb_frag_address(f), skb_frag_size(f),
				   0x0, HIF_LAST_BUFFER | HIF_DATA_VALID,
				   skb);

		netif_info(priv, tx_queued, priv->ndev,
			   "%s: pkt sent successfully skb:%p nr_frags:%d len:%d\n",
			   __func__, skb, nr_frags, skb->len);
	} else {
		__hif_lib_xmit_pkt(&priv->client, queuenum, skb->data,
				   skb->len, ctrl, HIF_FIRST_BUFFER |
				   HIF_LAST_BUFFER | HIF_DATA_VALID,
				   skb);
		netif_info(priv, tx_queued, priv->ndev,
			   "%s: pkt sent successfully skb:%p len:%d\n",
			   __func__, skb, skb->len);
	}
	hif_tx_dma_start();
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;
	hif_lib_tx_credit_use(pfe, priv->id, queuenum, 1);
}

/* pfe_eth_flush_txQ
 */
static void pfe_eth_flush_txQ(struct pfe_eth_priv_s *priv, int tx_q_num, int
				from_tx, int n_desc)
{
	struct sk_buff *skb;
	struct netdev_queue *tx_queue = netdev_get_tx_queue(priv->ndev,
								tx_q_num);
	unsigned int flags;

	netif_info(priv, tx_done, priv->ndev, "%s\n", __func__);

	if (!from_tx)
		__netif_tx_lock_bh(tx_queue);

	/* Clean HIF and client queue */
	while ((skb = hif_lib_tx_get_next_complete(&priv->client,
						   tx_q_num, &flags,
						   HIF_TX_DESC_NT))) {
		if (flags & HIF_DATA_VALID)
			dev_kfree_skb_any(skb);
	}
	if (!from_tx)
		__netif_tx_unlock_bh(tx_queue);
}

/* pfe_eth_flush_tx
 */
static void pfe_eth_flush_tx(struct pfe_eth_priv_s *priv)
{
	int ii;

	netif_info(priv, tx_done, priv->ndev, "%s\n", __func__);

	for (ii = 0; ii < emac_txq_cnt; ii++)
		pfe_eth_flush_txQ(priv, ii, 0, 0);
}

void pfe_tx_get_req_desc(struct sk_buff *skb, unsigned int *n_desc, unsigned int
				*n_segs)
{
	struct skb_shared_info *sh = skb_shinfo(skb);

	/* Scattered data */
	if (sh->nr_frags) {
		*n_desc = sh->nr_frags + 1;
		*n_segs = 1;
	/* Regular case */
	} else {
		*n_desc = 1;
		*n_segs = 1;
	}
}

/* pfe_eth_send_packet
 */
static int pfe_eth_send_packet(struct sk_buff *skb, struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	int tx_q_num = skb_get_queue_mapping(skb);
	int n_desc, n_segs;
	struct netdev_queue *tx_queue = netdev_get_tx_queue(priv->ndev,
								tx_q_num);

	netif_info(priv, tx_queued, ndev, "%s\n", __func__);

	if ((!skb_is_gso(skb)) && (skb_headroom(skb) < (PFE_PKT_HEADER_SZ +
			sizeof(unsigned long)))) {
		netif_warn(priv, tx_err, priv->ndev, "%s: copying skb\n",
			   __func__);

		if (pskb_expand_head(skb, (PFE_PKT_HEADER_SZ + sizeof(unsigned
					long)), 0, GFP_ATOMIC)) {
			/* No need to re-transmit, no way to recover*/
			kfree_skb(skb);
			priv->stats.tx_dropped++;
			return NETDEV_TX_OK;
		}
	}

	pfe_tx_get_req_desc(skb, &n_desc, &n_segs);

	hif_tx_lock(&pfe->hif);
	if (unlikely(pfe_eth_might_stop_tx(priv, tx_q_num, tx_queue, n_desc,
					   n_segs))) {
#ifdef PFE_ETH_TX_STATS
		if (priv->was_stopped[tx_q_num]) {
			priv->clean_fail[tx_q_num]++;
			priv->was_stopped[tx_q_num] = 0;
		}
#endif
		hif_tx_unlock(&pfe->hif);
		return NETDEV_TX_BUSY;
	}

	pfe_hif_send_packet(skb, priv, tx_q_num);

	hif_tx_unlock(&pfe->hif);

	tx_queue->trans_start = jiffies;

#ifdef PFE_ETH_TX_STATS
	priv->was_stopped[tx_q_num] = 0;
#endif

	return NETDEV_TX_OK;
}

/* pfe_eth_select_queue
 *
 */
static u16 pfe_eth_select_queue(struct net_device *ndev, struct sk_buff *skb,
				struct net_device *sb_dev,
				select_queue_fallback_t fallback)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	return pfe_eth_get_queuenum(priv, skb);
}

/* pfe_eth_get_stats
 */
static struct net_device_stats *pfe_eth_get_stats(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);

	netif_info(priv, drv, ndev, "%s\n", __func__);

	return &priv->stats;
}

/* pfe_eth_set_mac_address
 */
static int pfe_eth_set_mac_address(struct net_device *ndev, void *addr)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	struct sockaddr *sa = addr;

	netif_info(priv, drv, ndev, "%s\n", __func__);

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, sa->sa_data, ETH_ALEN);

	gemac_set_laddrN(priv->EMAC_baseaddr,
			 (struct pfe_mac_addr *)ndev->dev_addr, 1);

	return 0;
}

/* pfe_eth_enet_addr_byte_mac
 */
int pfe_eth_enet_addr_byte_mac(u8 *enet_byte_addr,
			       struct pfe_mac_addr *enet_addr)
{
	if (!enet_byte_addr || !enet_addr) {
		return -1;

	} else {
		enet_addr->bottom = enet_byte_addr[0] |
			(enet_byte_addr[1] << 8) |
			(enet_byte_addr[2] << 16) |
			(enet_byte_addr[3] << 24);
		enet_addr->top = enet_byte_addr[4] |
			(enet_byte_addr[5] << 8);
		return 0;
	}
}

/* pfe_eth_set_multi
 */
static void pfe_eth_set_multi(struct net_device *ndev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	struct pfe_mac_addr    hash_addr; /* hash register structure */
	/* specific mac address	register structure */
	struct pfe_mac_addr    spec_addr;
	int		result; /* index into hash register to set.. */
	int		uc_count = 0;
	struct netdev_hw_addr *ha;

	if (ndev->flags & IFF_PROMISC) {
		netif_info(priv, drv, ndev, "entering promiscuous mode\n");

		priv->promisc = 1;
		gemac_enable_copy_all(priv->EMAC_baseaddr);
	} else {
		priv->promisc = 0;
		gemac_disable_copy_all(priv->EMAC_baseaddr);
	}

	/* Enable broadcast frame reception if required. */
	if (ndev->flags & IFF_BROADCAST) {
		gemac_allow_broadcast(priv->EMAC_baseaddr);
	} else {
		netif_info(priv, drv, ndev,
			   "disabling broadcast frame reception\n");

		gemac_no_broadcast(priv->EMAC_baseaddr);
	}

	if (ndev->flags & IFF_ALLMULTI) {
		/* Set the hash to rx all multicast frames */
		hash_addr.bottom = 0xFFFFFFFF;
		hash_addr.top = 0xFFFFFFFF;
		gemac_set_hash(priv->EMAC_baseaddr, &hash_addr);
		netdev_for_each_uc_addr(ha, ndev) {
			if (uc_count >= MAX_UC_SPEC_ADDR_REG)
				break;
			pfe_eth_enet_addr_byte_mac(ha->addr, &spec_addr);
			gemac_set_laddrN(priv->EMAC_baseaddr, &spec_addr,
					 uc_count + 2);
			uc_count++;
		}
	} else if ((netdev_mc_count(ndev) > 0)  || (netdev_uc_count(ndev))) {
		u8 *addr;

		hash_addr.bottom = 0;
		hash_addr.top = 0;

		netdev_for_each_mc_addr(ha, ndev) {
			addr = ha->addr;

			netif_info(priv, drv, ndev,
				   "adding multicast address %X:%X:%X:%X:%X:%X to gem filter\n",
				addr[0], addr[1], addr[2],
				addr[3], addr[4], addr[5]);

			result = pfe_eth_get_hash(addr);

			if (result < EMAC_HASH_REG_BITS) {
				if (result < 32)
					hash_addr.bottom |= (1 << result);
				else
					hash_addr.top |= (1 << (result - 32));
			} else {
				break;
			}
		}

		uc_count = -1;
		netdev_for_each_uc_addr(ha, ndev) {
			addr = ha->addr;

			if (++uc_count < MAX_UC_SPEC_ADDR_REG)   {
				netdev_info(ndev,
					    "adding unicast address %02x:%02x:%02x:%02x:%02x:%02x to gem filter\n",
					    addr[0], addr[1], addr[2],
					    addr[3], addr[4], addr[5]);
				pfe_eth_enet_addr_byte_mac(addr, &spec_addr);
				gemac_set_laddrN(priv->EMAC_baseaddr,
						 &spec_addr, uc_count + 2);
			} else {
				netif_info(priv, drv, ndev,
					   "adding unicast address %02x:%02x:%02x:%02x:%02x:%02x to gem hash\n",
					   addr[0], addr[1], addr[2],
					   addr[3], addr[4], addr[5]);

				result = pfe_eth_get_hash(addr);
				if (result >= EMAC_HASH_REG_BITS) {
					break;

				} else {
					if (result < 32)
						hash_addr.bottom |= (1 <<
								result);
					else
						hash_addr.top |= (1 <<
								(result - 32));
				}
			}
		}

		gemac_set_hash(priv->EMAC_baseaddr, &hash_addr);
	}

	if (!(netdev_uc_count(ndev) >= MAX_UC_SPEC_ADDR_REG)) {
		/*
		 *  Check if there are any specific address HW registers that
		 * need to be flushed
		 */
		for (uc_count = netdev_uc_count(ndev); uc_count <
			MAX_UC_SPEC_ADDR_REG; uc_count++)
			gemac_clear_laddrN(priv->EMAC_baseaddr, uc_count + 2);
	}

	if (ndev->flags & IFF_LOOPBACK)
		gemac_set_loop(priv->EMAC_baseaddr, LB_LOCAL);
}

/* pfe_eth_set_features
 */
static int pfe_eth_set_features(struct net_device *ndev, netdev_features_t
					features)
{
	struct pfe_eth_priv_s *priv = netdev_priv(ndev);
	int rc = 0;

	if (features & NETIF_F_RXCSUM)
		gemac_enable_rx_checksum_offload(priv->EMAC_baseaddr);
	else
		gemac_disable_rx_checksum_offload(priv->EMAC_baseaddr);
	return rc;
}

/* pfe_eth_fast_tx_timeout
 */
static enum hrtimer_restart pfe_eth_fast_tx_timeout(struct hrtimer *timer)
{
	struct pfe_eth_fast_timer *fast_tx_timeout = container_of(timer, struct
							pfe_eth_fast_timer,
							timer);
	struct pfe_eth_priv_s *priv =  container_of(fast_tx_timeout->base,
							struct pfe_eth_priv_s,
							fast_tx_timeout);
	struct netdev_queue *tx_queue = netdev_get_tx_queue(priv->ndev,
						fast_tx_timeout->queuenum);

	if (netif_tx_queue_stopped(tx_queue)) {
#ifdef PFE_ETH_TX_STATS
		priv->was_stopped[fast_tx_timeout->queuenum] = 1;
#endif
		netif_tx_wake_queue(tx_queue);
	}

	return HRTIMER_NORESTART;
}

/* pfe_eth_fast_tx_timeout_init
 */
static void pfe_eth_fast_tx_timeout_init(struct pfe_eth_priv_s *priv)
{
	int i;

	for (i = 0; i < emac_txq_cnt; i++) {
		priv->fast_tx_timeout[i].queuenum = i;
		hrtimer_init(&priv->fast_tx_timeout[i].timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL);
		priv->fast_tx_timeout[i].timer.function =
				pfe_eth_fast_tx_timeout;
		priv->fast_tx_timeout[i].base = priv->fast_tx_timeout;
	}
}

static struct sk_buff *pfe_eth_rx_skb(struct net_device *ndev,
				      struct	pfe_eth_priv_s *priv,
				      unsigned int qno)
{
	void *buf_addr;
	unsigned int rx_ctrl;
	unsigned int desc_ctrl = 0;
	struct hif_ipsec_hdr *ipsec_hdr = NULL;
	struct sk_buff *skb;
	struct sk_buff *skb_frag, *skb_frag_last = NULL;
	int length = 0, offset;

	skb = priv->skb_inflight[qno];

	if (skb) {
		skb_frag_last = skb_shinfo(skb)->frag_list;
		if (skb_frag_last) {
			while (skb_frag_last->next)
				skb_frag_last = skb_frag_last->next;
		}
	}

	while (!(desc_ctrl & CL_DESC_LAST)) {
		buf_addr = hif_lib_receive_pkt(&priv->client, qno, &length,
					       &offset, &rx_ctrl, &desc_ctrl,
					       (void **)&ipsec_hdr);
		if (!buf_addr)
			goto incomplete;

#ifdef PFE_ETH_NAPI_STATS
		priv->napi_counters[NAPI_DESC_COUNT]++;
#endif

		/* First frag */
		if (desc_ctrl & CL_DESC_FIRST) {
			skb = build_skb(buf_addr, 0);
			if (unlikely(!skb))
				goto pkt_drop;

			skb_reserve(skb, offset);
			skb_put(skb, length);
			skb->dev = ndev;

			if ((ndev->features & NETIF_F_RXCSUM) && (rx_ctrl &
					HIF_CTRL_RX_CHECKSUMMED))
				skb->ip_summed = CHECKSUM_UNNECESSARY;
			else
				skb_checksum_none_assert(skb);

		} else {
			/* Next frags */
			if (unlikely(!skb)) {
				pr_err("%s: NULL skb_inflight\n",
				       __func__);
				goto pkt_drop;
			}

			skb_frag = build_skb(buf_addr, 0);

			if (unlikely(!skb_frag)) {
				kfree(buf_addr);
				goto pkt_drop;
			}

			skb_reserve(skb_frag, offset);
			skb_put(skb_frag, length);

			skb_frag->dev = ndev;

			if (skb_shinfo(skb)->frag_list)
				skb_frag_last->next = skb_frag;
			else
				skb_shinfo(skb)->frag_list = skb_frag;

			skb->truesize += skb_frag->truesize;
			skb->data_len += length;
			skb->len += length;
			skb_frag_last = skb_frag;
		}
	}

	priv->skb_inflight[qno] = NULL;
	return skb;

incomplete:
	priv->skb_inflight[qno] = skb;
	return NULL;

pkt_drop:
	priv->skb_inflight[qno] = NULL;

	if (skb)
		kfree_skb(skb);
	else
		kfree(buf_addr);

	priv->stats.rx_errors++;

	return NULL;
}

/* pfe_eth_poll
 */
static int pfe_eth_poll(struct pfe_eth_priv_s *priv, struct napi_struct *napi,
			unsigned int qno, int budget)
{
	struct net_device *ndev = priv->ndev;
	struct sk_buff *skb;
	int work_done = 0;
	unsigned int len;

	netif_info(priv, intr, priv->ndev, "%s\n", __func__);

#ifdef PFE_ETH_NAPI_STATS
	priv->napi_counters[NAPI_POLL_COUNT]++;
#endif

	do {
		skb = pfe_eth_rx_skb(ndev, priv, qno);

		if (!skb)
			break;

		len = skb->len;

		/* Packet will be processed */
		skb->protocol = eth_type_trans(skb, ndev);

		netif_receive_skb(skb);

		priv->stats.rx_packets++;
		priv->stats.rx_bytes += len;

		work_done++;

#ifdef PFE_ETH_NAPI_STATS
		priv->napi_counters[NAPI_PACKET_COUNT]++;
#endif

	} while (work_done < budget);

	/*
	 * If no Rx receive nor cleanup work was done, exit polling mode.
	 * No more netif_running(dev) check is required here , as this is
	 * checked in net/core/dev.c (2.6.33.5 kernel specific).
	 */
	if (work_done < budget) {
		napi_complete(napi);

		hif_lib_event_handler_start(&priv->client, EVENT_RX_PKT_IND,
					    qno);
	}
#ifdef PFE_ETH_NAPI_STATS
	else
		priv->napi_counters[NAPI_FULL_BUDGET_COUNT]++;
#endif

	return work_done;
}

/*
 * pfe_eth_lro_poll
 */
static int pfe_eth_lro_poll(struct napi_struct *napi, int budget)
{
	struct pfe_eth_priv_s *priv = container_of(napi, struct pfe_eth_priv_s,
							lro_napi);

	netif_info(priv, intr, priv->ndev, "%s\n", __func__);

	return pfe_eth_poll(priv, napi, 2, budget);
}

/* pfe_eth_low_poll
 */
static int pfe_eth_low_poll(struct napi_struct *napi, int budget)
{
	struct pfe_eth_priv_s *priv = container_of(napi, struct pfe_eth_priv_s,
							low_napi);

	netif_info(priv, intr, priv->ndev, "%s\n", __func__);

	return pfe_eth_poll(priv, napi, 1, budget);
}

/* pfe_eth_high_poll
 */
static int pfe_eth_high_poll(struct napi_struct *napi, int budget)
{
	struct pfe_eth_priv_s *priv = container_of(napi, struct pfe_eth_priv_s,
							high_napi);

	netif_info(priv, intr, priv->ndev, "%s\n", __func__);

	return pfe_eth_poll(priv, napi, 0, budget);
}

static const struct net_device_ops pfe_netdev_ops = {
	.ndo_open = pfe_eth_open,
	.ndo_stop = pfe_eth_close,
	.ndo_start_xmit = pfe_eth_send_packet,
	.ndo_select_queue = pfe_eth_select_queue,
	.ndo_get_stats = pfe_eth_get_stats,
	.ndo_set_mac_address = pfe_eth_set_mac_address,
	.ndo_set_rx_mode = pfe_eth_set_multi,
	.ndo_set_features = pfe_eth_set_features,
	.ndo_validate_addr = eth_validate_addr,
};

/* pfe_eth_init_one
 */
static int pfe_eth_init_one(struct pfe *pfe, int id)
{
	struct net_device *ndev = NULL;
	struct pfe_eth_priv_s *priv = NULL;
	struct ls1012a_eth_platform_data *einfo;
	struct ls1012a_mdio_platform_data *minfo;
	struct ls1012a_pfe_platform_data *pfe_info;
	int err;

	/* Extract pltform data */
	pfe_info = (struct ls1012a_pfe_platform_data *)
					pfe->dev->platform_data;
	if (!pfe_info) {
		pr_err(
			"%s: pfe missing additional platform data\n"
			, __func__);
		err = -ENODEV;
		goto err0;
	}

	einfo = (struct ls1012a_eth_platform_data *)
				pfe_info->ls1012a_eth_pdata;

	/* einfo never be NULL, but no harm in having this check */
	if (!einfo) {
		pr_err(
			"%s: pfe missing additional gemacs platform data\n"
			, __func__);
		err = -ENODEV;
		goto err0;
	}

	minfo = (struct ls1012a_mdio_platform_data *)
				pfe_info->ls1012a_mdio_pdata;

	/* einfo never be NULL, but no harm in having this check */
	if (!minfo) {
		pr_err(
			"%s: pfe missing additional mdios platform data\n",
			 __func__);
		err = -ENODEV;
		goto err0;
	}

	/* Create an ethernet device instance */
	ndev = alloc_etherdev_mq(sizeof(*priv), emac_txq_cnt);

	if (!ndev) {
		pr_err("%s: gemac %d device allocation failed\n",
		       __func__, einfo[id].gem_id);
		err = -ENOMEM;
		goto err0;
	}

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->id = einfo[id].gem_id;
	priv->pfe = pfe;

	SET_NETDEV_DEV(priv->ndev, priv->pfe->dev);

	pfe->eth.eth_priv[id] = priv;

	/* Set the info in the priv to the current info */
	priv->einfo = &einfo[id];
	priv->EMAC_baseaddr = cbus_emac_base[id];
	priv->PHY_baseaddr = cbus_emac_base[0];
	priv->GPI_baseaddr = cbus_gpi_base[id];

#define HIF_GEMAC_TMUQ_BASE	6
	priv->low_tmu_q	=  HIF_GEMAC_TMUQ_BASE + (id * 2);
	priv->high_tmu_q	=  priv->low_tmu_q + 1;

	spin_lock_init(&priv->lock);

	pfe_eth_fast_tx_timeout_init(priv);

	/* Copy the station address into the dev structure, */
	memcpy(ndev->dev_addr, einfo[id].mac_addr, ETH_ALEN);

	/* Initialize mdio */
	if (minfo[id].enabled) {
		err = pfe_eth_mdio_init(priv, &minfo[id]);
		if (err) {
			netdev_err(ndev, "%s: pfe_eth_mdio_init() failed\n",
				   __func__);
			goto err2;
		}
	}

	ndev->mtu = 1500;

	/* Set MTU limits */
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = JUMBO_FRAME_SIZE;

	/* supported features */
	ndev->hw_features = NETIF_F_SG;

	/*Enable after checksum offload is validated */
	ndev->hw_features = NETIF_F_RXCSUM | NETIF_F_IP_CSUM |
		NETIF_F_IPV6_CSUM | NETIF_F_SG;

	/* enabled by default */
	ndev->features = ndev->hw_features;

	priv->usr_features = ndev->features;

	ndev->netdev_ops = &pfe_netdev_ops;

	ndev->ethtool_ops = &pfe_ethtool_ops;

	/* Enable basic messages by default */
	priv->msg_enable = NETIF_MSG_IFUP | NETIF_MSG_IFDOWN | NETIF_MSG_LINK |
				NETIF_MSG_PROBE;

	netif_napi_add(ndev, &priv->low_napi, pfe_eth_low_poll);
	netif_napi_add(ndev, &priv->high_napi, pfe_eth_high_poll);
	netif_napi_add(ndev, &priv->lro_napi, pfe_eth_lro_poll);

	err = register_netdev(ndev);

	if (err) {
		netdev_err(ndev, "register_netdev() failed\n");
		goto err3;
	}
	device_init_wakeup(&ndev->dev, WAKE_MAGIC);

	if (!(priv->einfo->phy_flags & GEMAC_NO_PHY)) {
		err = pfe_phy_init(ndev);
		if (err) {
			netdev_err(ndev, "%s: pfe_phy_init() failed\n",
				   __func__);
			goto err4;
		}
	}

	netif_carrier_on(ndev);

	/* Create all the sysfs files */
	if (pfe_eth_sysfs_init(ndev))
		goto err4;

	netif_info(priv, probe, ndev, "%s: created interface, baseaddr: %p\n",
		   __func__, priv->EMAC_baseaddr);

	return 0;
err4:
	unregister_netdev(ndev);
err3:
	pfe_eth_mdio_exit(priv->mii_bus);
err2:
	free_netdev(priv->ndev);
err0:
	return err;
}

/* pfe_eth_init
 */
int pfe_eth_init(struct pfe *pfe)
{
	int ii = 0;
	int err;

	pr_info("%s\n", __func__);

	cbus_emac_base[0] = EMAC1_BASE_ADDR;
	cbus_emac_base[1] = EMAC2_BASE_ADDR;

	cbus_gpi_base[0] = EGPI1_BASE_ADDR;
	cbus_gpi_base[1] = EGPI2_BASE_ADDR;

	for (ii = 0; ii < NUM_GEMAC_SUPPORT; ii++) {
		err = pfe_eth_init_one(pfe, ii);
		if (err)
			goto err0;
	}

	return 0;

err0:
	while (ii--)
		pfe_eth_exit_one(pfe->eth.eth_priv[ii]);

	/* Register three network devices in the kernel */
	return err;
}

/* pfe_eth_exit_one
 */
static void pfe_eth_exit_one(struct pfe_eth_priv_s *priv)
{
	netif_info(priv, probe, priv->ndev, "%s\n", __func__);

	pfe_eth_sysfs_exit(priv->ndev);

	unregister_netdev(priv->ndev);

	if (!(priv->einfo->phy_flags & GEMAC_NO_PHY))
		pfe_phy_exit(priv->ndev);

	if (priv->mii_bus)
		pfe_eth_mdio_exit(priv->mii_bus);

	free_netdev(priv->ndev);
}

/* pfe_eth_exit
 */
void pfe_eth_exit(struct pfe *pfe)
{
	int ii;

	pr_info("%s\n", __func__);

	for (ii = NUM_GEMAC_SUPPORT - 1; ii >= 0; ii--)
		pfe_eth_exit_one(pfe->eth.eth_priv[ii]);
}
