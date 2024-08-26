// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/phy.h>
#include <linux/net_tstamp.h>
#include <linux/ethtool.h>
#include <linux/bitrev.h>
#include <linux/completion.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <net/pkt_cls.h>
#include <net/pkt_sched.h>
#include <net/tcp.h>
#include <net/udp.h>
#include "adrv906x-net.h"
#include "adrv906x-mac.h"
#include "adrv906x-ethtool.h"

#define NDMA_LOOPBACK_TEST_PATTERN              0x12
#define NDMA_LOOPBACK_TEST_SIZE                 1024

#define ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN        BIT(0)

/* TODO: Ugly global variable, need to be changed */
#if IS_BUILTIN(CONFIG_PTP_1588_CLOCK_ADRV906X)
/* The adi ptp module will set this variable */
extern int adrv906x_phc_index;
#endif
extern const struct ethtool_ops adrv906x_ethtool_ops;

static const char adrv906x_gstrings_stats_names[][ETH_GSTRING_LEN] = {
	"mac_rx_drop_events",
	"mac_rx_octets",
	"mac_rx_pkts",
	"mac_rx_broadcast_pkts",
	"mac_rx_multicast_pkts",
	"mac_rx_unicast_pkts",
	"mac_rx_undersize_pkts",
	"mac_rx_oversize_pkts",
	"mac_rx_pkts_64_octets",
	"mac_rx_pkts_65to127_octets",
	"mac_rx_pkts_128to255_octets",
	"mac_rx_pkts_256to511_octets",
	"mac_rx_pkts_512to1023_octets",
	"mac_rx_pkts_1024to1518_octets",
	"mac_rx_pkts_1519tox_octets",
	"mac_rx_overflow",
	"mac_rx_crc_error",
	"mac_rx_mc_drop",
	"mac_rx_fragments",
	"mac_rx_jabbers",
	"mac_rx_mac_framing_error",
	"mac_rx_rs_framing_error",
	"mac_tx_drop_events",
	"mac_tx_octets",
	"mac_tx_pkts",
	"mac_tx_broadcast_pkts",
	"mac_tx_multicast_pkts",
	"mac_tx_unicast_pkts",
	"mac_tx_undersize_pkts",
	"mac_tx_oversize_pkts",
	"mac_tx_pkts_64_octets",
	"mac_tx_pkts_65to127_octets",
	"mac_tx_pkts_128to255_octets",
	"mac_tx_pkts_256to511_octets",
	"mac_tx_pkts_512to1023_octets",
	"mac_tx_pkts_1024to1518_octets",
	"mac_tx_pkts_1519tox_octets",
	"mac_tx_underflow",
	"mac_tx_padded",
	"ndma_rx_frame_error",
	"ndma_rx_frame_Size_error",
	"ndma_rx_frame_dropped_error",
	"ndma_rx_frame_dropped_s_plane",
	"ndma_rx_frame_dropped_m_plane",
	"ndma_rx_seqnumb_mismatch_error",
	"ndma_rx_status_header_error",
	"ndma_rx_unknown_error",
	"ndma_rx_pending_work_unit",
	"ndma_rx_done_work_unit",
	"ndma_rx_dma_error",
	"ndma_tx_frame_size_error",
	"ndma_tx_data_header_error",
	"ndma_tx_status_header_error",
	"ndma_tx_tstamp_timeout_error",
	"ndma_tx_seqnumb_mismatch_error",
	"ndma_tx_unknown_error",
	"ndma_tx_pending_work_unit",
	"ndma_tx_done_work_unit",
	"ndma_tx_data_dma_error",
	"ndma_tx_status_dma_error",
};

static const char adrv906x_gstrings_selftest_names[][ETH_GSTRING_LEN] = {
	"Internal loopback (offline):    ",
	"MAC loopback on/off (online):   ",
	"NDMA loopback (offline):        ",
};

#define ADRV906X_NUM_STATS ARRAY_SIZE(adrv906x_gstrings_stats_names)
#define ADRV906X_NUM_SELFTEST ARRAY_SIZE(adrv906x_gstrings_selftest_names)

struct adrv906x_test {
	char name[ETH_GSTRING_LEN];
	int (*fn)(struct net_device *ndev);
	unsigned char etest_flag;
};

struct payload_hdr {
	__be32 version;
	__be64 magic;
	u8 id;
} __packed;

struct adrv906x_packet_attrs {
	unsigned char *src;
	unsigned char *dst;
	u32 ip_src;
	u32 ip_dst;
	int sport;
	int dport;
	u32 exp_hash;
	int dont_wait;
	int timeout;
	int size;
	int max_size;
	u8 id;
	u16 queue_mapping;
	u64 timestamp;
	bool negative_test_flag;
};

struct adrv906x_test_priv {
	union {
		struct adrv906x_packet_attrs *packet;
		struct net_device *ndev;
	};
	struct packet_type pt;
	struct completion comp;
	int ok;
};

#define ADRV906X_TEST_PKT_SIZE (sizeof(struct ethhdr) + sizeof(struct iphdr) + \
				sizeof(struct udphdr) + sizeof(struct payload_hdr))
#define ADRV906X_TEST_PKT_MAGIC 0xdeadcafecafedeadULL
#define ADRV906X_LB_TIMEOUT       msecs_to_jiffies(200)

static u8 adrv906x_packet_next_id;

int adrv906x_ethtool_set_link_ksettings(struct net_device *ndev,
					const struct ethtool_link_ksettings *cmd)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(advertising);
	u8 autoneg = cmd->base.autoneg;
	u8 duplex = cmd->base.duplex;
	u32 speed = cmd->base.speed;
	struct phy_device *phydev = ndev->phydev;

	if (!phydev)
		return -ENODEV;

	if (cmd->base.phy_address != phydev->mdio.addr)
		return -EINVAL;

	linkmode_copy(advertising, cmd->link_modes.advertising);

	/* We make sure that we don't pass unsupported values in to the PHY */
	linkmode_and(advertising, advertising, phydev->supported);

	/* Verify the settings we care about. */
	if (autoneg == AUTONEG_ENABLE)
		return -EINVAL;

	if ((speed != SPEED_10000 && speed != SPEED_25000) || duplex != DUPLEX_FULL)
		return -EINVAL;

	mutex_lock(&phydev->lock);
	phydev->autoneg = autoneg;
	phydev->speed = speed;
	phydev->duplex = duplex;

	linkmode_copy(phydev->advertising, advertising);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			 phydev->advertising, autoneg == AUTONEG_ENABLE);

	if (phy_is_started(phydev)) {
		phydev->state = PHY_UP;
		phy_start_machine(phydev);
	}
	mutex_unlock(&phydev->lock);

	return 0;
}

int adrv906x_ethtool_get_ts_info(struct net_device *ndev, struct ethtool_ts_info *info)
{
	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON);
	info->rx_filters =
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_SYNC) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_SYNC) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ) |
		(1 << HWTSTAMP_FILTER_PTP_V2_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_SYNC) |
		(1 << HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
		(1 << HWTSTAMP_FILTER_ALL);

#if IS_BUILTIN(CONFIG_PTP_1588_CLOCK_ADRV906X)
	info->phc_index = adrv906x_phc_index;
#else
	info->phc_index = -1;
#endif
	return 0;
}

int adrv906x_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset == ETH_SS_STATS)
		return ADRV906X_NUM_STATS;

	if (sset == ETH_SS_TEST)
		return ADRV906X_NUM_SELFTEST;

	return -EOPNOTSUPP;
}

void adrv906x_ethtool_get_strings(struct net_device *ndev, u32 sset, u8 *buf)
{
	if (sset == ETH_SS_STATS)
		memcpy(buf, &adrv906x_gstrings_stats_names,
		       sizeof(adrv906x_gstrings_stats_names));

	if (sset == ETH_SS_TEST)
		memcpy(buf, &adrv906x_gstrings_selftest_names,
		       sizeof(adrv906x_gstrings_selftest_names));
}

void adrv906x_ethtool_get_stats(struct net_device *ndev, struct ethtool_stats *stats,
				u64 *data)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	union adrv906x_ndma_chan_stats *ndma_rx_stats = &adrv906x_dev->ndma_dev->rx_chan.stats;
	union adrv906x_ndma_chan_stats *ndma_tx_stats = &adrv906x_dev->ndma_dev->tx_chan.stats;
	struct adrv906x_mac_rx_stats *mac_rx_stats = &adrv906x_dev->mac.hw_stats_rx;
	struct adrv906x_mac_tx_stats *mac_tx_stats = &adrv906x_dev->mac.hw_stats_tx;

	data[0] = mac_rx_stats->general_stats.drop_events;
	data[1] = mac_rx_stats->general_stats.octets;
	data[2] = mac_rx_stats->general_stats.pkts;
	data[3] = mac_rx_stats->general_stats.broadcast_pkts;
	data[4] = mac_rx_stats->general_stats.multicast_pkts;
	data[5] = mac_rx_stats->general_stats.unicast_pkts;
	data[6] = mac_rx_stats->general_stats.undersize_pkts;
	data[7] = mac_rx_stats->general_stats.oversize_pkts;
	data[8] = mac_rx_stats->general_stats.pkts_64_octets;
	data[9] = mac_rx_stats->general_stats.pkts_65to127_octets;
	data[10] = mac_rx_stats->general_stats.pkts_128to255_octets;
	data[11] = mac_rx_stats->general_stats.pkts_256to511_octets;
	data[12] = mac_rx_stats->general_stats.pkts_512to1023_octets;
	data[13] = mac_rx_stats->general_stats.pkts_1024to1518_octets;
	data[14] = mac_rx_stats->general_stats.pkts_1519tox_octets;
	data[15] = mac_rx_stats->overflow;
	data[16] = mac_rx_stats->crc_errors;
	data[17] = mac_rx_stats->mc_drop;
	data[18] = mac_rx_stats->fragments;
	data[19] = mac_rx_stats->jabbers;
	data[20] = mac_rx_stats->mac_framing_error;
	data[21] = mac_rx_stats->rs_framing_error;
	data[22] = mac_tx_stats->general_stats.drop_events;
	data[23] = mac_tx_stats->general_stats.octets;
	data[24] = mac_tx_stats->general_stats.pkts;
	data[25] = mac_tx_stats->general_stats.broadcast_pkts;
	data[26] = mac_tx_stats->general_stats.multicast_pkts;
	data[27] = mac_tx_stats->general_stats.unicast_pkts;
	data[28] = mac_tx_stats->general_stats.undersize_pkts;
	data[29] = mac_tx_stats->general_stats.oversize_pkts;
	data[30] = mac_tx_stats->general_stats.pkts_64_octets;
	data[31] = mac_tx_stats->general_stats.pkts_65to127_octets;
	data[32] = mac_tx_stats->general_stats.pkts_128to255_octets;
	data[33] = mac_tx_stats->general_stats.pkts_256to511_octets;
	data[34] = mac_tx_stats->general_stats.pkts_512to1023_octets;
	data[35] = mac_tx_stats->general_stats.pkts_1024to1518_octets;
	data[36] = mac_tx_stats->general_stats.pkts_1519tox_octets;
	data[37] = mac_tx_stats->underflow;
	data[38] = mac_tx_stats->padded;
	data[39] = ndma_rx_stats->rx.frame_errors;
	data[40] = ndma_rx_stats->rx.frame_size_errors;
	data[41] = ndma_rx_stats->rx.frame_dropped_errors;
	data[42] = ndma_rx_stats->rx.frame_dropped_splane_errors;
	data[43] = ndma_rx_stats->rx.frame_dropped_mplane_errors;
	data[44] = ndma_rx_stats->rx.seqnumb_mismatch_errors;
	data[45] = ndma_rx_stats->rx.status_header_errors;
	data[46] = ndma_rx_stats->rx.unknown_errors;
	data[47] = ndma_rx_stats->rx.pending_work_units;
	data[48] = ndma_rx_stats->rx.done_work_units;
	data[49] = ndma_rx_stats->rx.dma_errors;
	data[50] = ndma_tx_stats->tx.frame_size_errors;
	data[51] = ndma_tx_stats->tx.data_header_errors;
	data[52] = ndma_tx_stats->tx.status_header_errors;
	data[53] = ndma_tx_stats->tx.tstamp_timeout_errors;
	data[54] = ndma_tx_stats->tx.seqnumb_mismatch_errors;
	data[55] = ndma_tx_stats->tx.unknown_errors;
	data[56] = ndma_tx_stats->tx.pending_work_units;
	data[57] = ndma_tx_stats->tx.done_work_units;
	data[58] = ndma_tx_stats->tx.data_dma_errors;
	data[59] = ndma_tx_stats->tx.status_dma_errors;
}

static int adrv906x_ethtool_get_fecparam(struct net_device *ndev,
					 struct ethtool_fecparam *fecparam)
{
	struct phy_device *phydev = ndev->phydev;

	fecparam->fec = ETHTOOL_FEC_RS;

	mutex_lock(&phydev->lock);
	if (phydev->speed == SPEED_25000 && phydev->dev_flags & ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN)
		fecparam->active_fec = ETHTOOL_FEC_RS;
	else
		fecparam->active_fec = ETHTOOL_FEC_OFF;
	mutex_unlock(&phydev->lock);

	return 0;
}

static int adrv906x_ethtool_set_fecparam(struct net_device *ndev,
					 struct ethtool_fecparam *fecparam)
{
	struct phy_device *phydev = ndev->phydev;
	bool fec_cur_en, fec_new_en;

	if (!(fecparam->fec & ETHTOOL_FEC_OFF) && !(fecparam->fec & ETHTOOL_FEC_RS))
		return -EOPNOTSUPP;

	mutex_lock(&phydev->lock);
	fec_cur_en = !!(phydev->dev_flags & ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN);
	fec_new_en = !!(fecparam->fec & ETHTOOL_FEC_RS);

	if (fec_cur_en != fec_new_en) {
		phydev->dev_flags ^= ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN;

		if (phy_is_started(phydev)) {
			phydev->state = PHY_UP;
			phy_start_machine(phydev);
		}
	}
	mutex_unlock(&phydev->lock);

	return 0;
}

static struct sk_buff *adrv906x_test_get_udp_skb(struct net_device *ndev,
						 struct adrv906x_packet_attrs *attr)
{
	struct sk_buff *skb = NULL;
	struct udphdr *uhdr = NULL;
	struct payload_hdr *phdr;
	struct ethhdr *ehdr;
	struct iphdr *ihdr;
	int iplen, size;

	size = ADRV906X_TEST_PKT_SIZE;

	skb = netdev_alloc_skb(ndev, size);
	if (!skb)
		return NULL;

	prefetchw(skb->data);

	/* corrupt ethernet frame for negative test case */
	if (attr->negative_test_flag)
		ehdr = skb_push(skb, ETH_HLEN - 2);
	else
		ehdr = skb_push(skb, ETH_HLEN);

	skb_reset_mac_header(skb);

	skb_set_network_header(skb, skb->len);
	ihdr = skb_put(skb, sizeof(*ihdr));

	skb_set_transport_header(skb, skb->len);
	uhdr = skb_put(skb, sizeof(*uhdr));

	eth_zero_addr(ehdr->h_dest);
	if (attr->src)
		ether_addr_copy(ehdr->h_source, attr->src);
	if (attr->dst)
		ether_addr_copy(ehdr->h_dest, attr->dst);

	ehdr->h_proto = htons(ETH_P_IP);

	uhdr->len = htons(sizeof(*phdr) + sizeof(*uhdr) + attr->size);
	uhdr->check = 0;

	ihdr->ihl = 5;
	ihdr->ttl = 32;
	ihdr->version = 4;
	ihdr->protocol = IPPROTO_UDP;
	iplen = sizeof(*ihdr) + sizeof(*phdr) + attr->size;
	iplen += sizeof(*uhdr);

	ihdr->tot_len = htons(iplen);
	ihdr->frag_off = 0;
	ihdr->saddr = htonl(attr->ip_src);
	ihdr->daddr = htonl(attr->ip_dst);
	ihdr->tos = 0;
	ihdr->id = 0;
	ip_send_check(ihdr);

	phdr = skb_put(skb, sizeof(*phdr));
	phdr->version = 0;
	phdr->magic = cpu_to_be64(ADRV906X_TEST_PKT_MAGIC);
	attr->id = adrv906x_packet_next_id;
	phdr->id = adrv906x_packet_next_id++;

	skb->csum = 0;
	skb->ip_summed = CHECKSUM_PARTIAL;
	udp4_hwcsum(skb, ihdr->saddr, ihdr->daddr);

	skb->protocol = htons(ETH_P_IP);
	skb->pkt_type = PACKET_HOST;
	skb->dev = ndev;

	return skb;
}

static int adrv906x_test_loopback_validate(struct sk_buff *skb, struct net_device *ndev,
					   struct packet_type *pt, struct net_device *orig_ndev)
{
	struct adrv906x_test_priv *tpriv = pt->af_packet_priv;
	unsigned char *src = tpriv->packet->src;
	unsigned char *dst = tpriv->packet->dst;

	struct payload_hdr *phdr;
	struct ethhdr *ehdr;
	struct udphdr *uhdr;
	struct iphdr *ihdr;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (skb_linearize(skb))
		goto out;
	if (skb_headlen(skb) < (ADRV906X_TEST_PKT_SIZE - ETH_HLEN))
		goto out;

	ehdr = (struct ethhdr *)skb_mac_header(skb);

	if (dst)
		if (!ether_addr_equal_unaligned(ehdr->h_dest, dst))
			goto out;

	if (src)
		if (!ether_addr_equal_unaligned(ehdr->h_source, src))
			goto out;

	ihdr = ip_hdr(skb);

	if (ihdr->protocol != IPPROTO_UDP)
		goto out;

	uhdr = (struct udphdr *)((u8 *)ihdr + 4 * ihdr->ihl);

	phdr = (struct payload_hdr *)((u8 *)uhdr + sizeof(*uhdr));

	if (phdr->magic != cpu_to_be64(ADRV906X_TEST_PKT_MAGIC))
		goto out;
	if (tpriv->packet->exp_hash && !skb->hash)
		goto out;
	if (tpriv->packet->id != phdr->id)
		goto out;

	tpriv->ok = true;
	complete(&tpriv->comp);
out:
	kfree_skb(skb);
	return 0;
}

static int adrv906x_test_phy_loopback_run(struct net_device *ndev,
					  struct adrv906x_packet_attrs *attr)
{
	struct adrv906x_test_priv *tpriv;
	struct sk_buff *skb = NULL;
	int ret = 0;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;

	tpriv->ok = false;
	init_completion(&tpriv->comp);

	tpriv->pt.type = htons(ETH_P_IP);
	tpriv->pt.func = adrv906x_test_loopback_validate;
	tpriv->pt.dev = ndev;
	tpriv->pt.af_packet_priv = tpriv;
	tpriv->packet = attr;

	if (!attr->dont_wait)
		dev_add_pack(&tpriv->pt);

	skb = adrv906x_test_get_udp_skb(ndev, attr);
	if (!skb) {
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = dev_direct_xmit(skb, attr->queue_mapping);
	if (ret)
		goto cleanup;

	if (attr->dont_wait)
		goto cleanup;

	if (!attr->timeout)
		attr->timeout = ADRV906X_LB_TIMEOUT;

	wait_for_completion_timeout(&tpriv->comp, attr->timeout);
	ret = tpriv->ok ? 0 : -ETIMEDOUT;

cleanup:
	if (!attr->dont_wait)
		dev_remove_pack(&tpriv->pt);
	kfree(tpriv);

	if (attr->negative_test_flag)
		ret = 0;

	return ret;
}

static int adrv906x_test_enable_phy_loopback(struct net_device *ndev, bool enable)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	void __iomem *regs;
	u32 val;

	regs = adrv906x_dev->parent->emac_cmn_regs;
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);

	if (enable) {
		if (adrv906x_dev->port == 0)
			val |= BIT(20);
		else
			val |= BIT(21);
	} else {
		if (adrv906x_dev->port == 0)
			val &= ~BIT(20);
		else
			val &= ~BIT(21);
	}

	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL2);

	return 0;
}

static int adrv906x_pcs_internal_loopback_test(struct net_device *ndev)
{
	struct adrv906x_packet_attrs attr = { };
	int ret;

	attr.dst = ndev->dev_addr;

	netdev_info(ndev, "adrv906x_pcs_internal_loopback_test");
/* TODO: decide which one to use after testing on hw
 *	ret = phy_loopback(ndev->phydev, true);
 *	if (ret)
 *		return ret;
 */
	ret = adrv906x_test_enable_phy_loopback(ndev, true);

	mdelay(2000);
	/* negative test case */
	attr.negative_test_flag = true;
	ret = adrv906x_test_phy_loopback_run(ndev, &attr);
	if (ret) {
		(void)phy_loopback(ndev->phydev, false);
		return ret;
	}

	/* normal test case */
	attr.negative_test_flag = false;
	ret = adrv906x_test_phy_loopback_run(ndev, &attr);
	if (ret) {
		(void)phy_loopback(ndev->phydev, false);
		return ret;
	}

/* TODO: decide which one to use after testing on hw
 *	ret = phy_loopback(ndev->phydev, false);
 *	if (ret)
 *		return ret;
 */
	ret = adrv906x_test_enable_phy_loopback(ndev, false);

	return 0;
}

static int adrv906x_mac_external_loopback_test(struct net_device *ndev)
{
	static bool loopback_state;
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	void __iomem *regs;
	u32 val = 0;

	netdev_info(ndev, "adrv906x_mac_external_loopback_test");
	regs = adrv906x_dev->parent->emac_cmn_regs;

	if (loopback_state) {
		val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);
		iowrite32(val & (~EMAC_CMN_LOOPBACK_BYPASS_MAC), regs + EMAC_CMN_DIGITAL_CTRL2);
		loopback_state = 0;
		netif_start_queue(ndev);
		netdev_info(ndev, "Turn off MAC loopback");
	} else {
		netif_stop_queue(ndev);
		val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);
		val |= EMAC_CMN_LOOPBACK_BYPASS_MAC;
		iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL2);
		loopback_state = 1;
		netdev_info(ndev, "Turn on MAC loopback");
	}

	return 0;
}

static void adrv906x_ndma_loopback_tx_callback(struct sk_buff *skb, unsigned int port_id,
					       struct timespec64 ts, void *cb_param)
{
	dev_kfree_skb(skb);
}

static void adrv906x_ndma_loopback_rx_callback(struct sk_buff *skb, unsigned int port_id,
					       struct timespec64 ts, void *cb_param)
{
	struct adrv906x_test_priv *tpriv = (struct adrv906x_test_priv *)cb_param;
	struct net_device *ndev = tpriv->ndev;
	struct adrv906x_eth_dev *adrv906x_dev;
	struct adrv906x_ndma_dev *ndma_dev;
	struct adrv906x_eth_if *eth_if;
	struct device *dev;
	unsigned char *p;
	int i;

	adrv906x_dev = netdev_priv(ndev);
	eth_if = adrv906x_dev->parent;

	adrv906x_dev = eth_if->adrv906x_dev[port_id];
	ndma_dev = adrv906x_dev->ndma_dev;
	dev = ndma_dev->dev;

	p = skb->data;
	for (i = 0; i < NDMA_LOOPBACK_TEST_SIZE; i++) {
		if (*p != NDMA_LOOPBACK_TEST_PATTERN) {
			dev_err(dev, "rx:0x%x tx:0x%x", *p, i);
			tpriv->ok = false;
			break;
		}
		p++;
	}
	dev_kfree_skb(skb);
	complete(&tpriv->comp);
}

static int adrv906x_ndma_loopback_test(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_ndma_dev *ndma_dev = adrv906x_dev->ndma_dev;
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct adrv906x_eth_dev *temp_eth_dev;
	struct device *dev = ndma_dev->dev;
	struct adrv906x_test_priv *tpriv;
	int port = adrv906x_dev->port;
	struct net_device *temp_ndev;
	struct sk_buff *tx_data1;
	bool all_down = false;
	int i, ret = 0, tmo;
	unsigned char *p;

	tpriv = kzalloc(sizeof(*tpriv), GFP_KERNEL);
	if (!tpriv)
		return -ENOMEM;
	tpriv->ok = true;
	init_completion(&tpriv->comp);
	tpriv->ndev = ndev;

	if (eth_if->ethswitch.enabled) {
		if (kref_read(&ndma_dev->refcount) > 1) {
			netdev_info(ndev, "switch enabled, shut down both interfaces");
			all_down = true;
			for (i = 0; i < MAX_NETDEV_NUM; i++) {
				temp_eth_dev = eth_if->adrv906x_dev[i];
				temp_ndev = temp_eth_dev->ndev;
				ndev->netdev_ops->ndo_stop(temp_ndev);
			}
		}
	} else {
		ndev->netdev_ops->ndo_stop(ndev);
	}
	adrv906x_ndma_open(ndma_dev,
			   adrv906x_ndma_loopback_tx_callback,
			   adrv906x_ndma_loopback_rx_callback,
			   tpriv, true);
	tx_data1 = netdev_alloc_skb(ndev, NDMA_LOOPBACK_TEST_SIZE);
	skb_put(tx_data1, NDMA_LOOPBACK_TEST_SIZE);
	p = tx_data1->data;
	for (i = 0; i < NDMA_LOOPBACK_TEST_SIZE; i++) {
		*p = NDMA_LOOPBACK_TEST_PATTERN;
		p++;
	}
	ret = adrv906x_ndma_start_xmit(ndma_dev, tx_data1, port, 0, 0);
	if (ret) {
		dev_err(dev, "ndma tx failed to send frame:0x%x", ret);
		ret = -EIO;
		dev_kfree_skb(tx_data1);
		goto out;
	}
	tmo = wait_for_completion_timeout(&tpriv->comp, msecs_to_jiffies(2000));
	if (!tmo) {
		dev_err(dev, "ndma loopback test timeout");
		ret = -ETIMEDOUT;
	}
	if (!tpriv->ok) {
		dev_err(dev, "ndma loopback test failed");
		ret = -EINVAL;
	}
out:
	adrv906x_ndma_close(ndma_dev);
	if (all_down) {
		for (i = 0; i < MAX_NETDEV_NUM; i++) {
			temp_eth_dev = eth_if->adrv906x_dev[i];
			temp_ndev = temp_eth_dev->ndev;
			ndev->netdev_ops->ndo_open(temp_ndev);
		}
	} else {
		ndev->netdev_ops->ndo_open(ndev);
	}
	kfree(tpriv);
	return ret;
}

struct adrv906x_test adrv906x_ethtool_selftests[] = {
	{
		.name = "PCS internal loopback",
		.fn = adrv906x_pcs_internal_loopback_test,
		.etest_flag = ETH_TEST_FL_OFFLINE,
	},
	{
		.name = "MAC external loopback",
		.fn = adrv906x_mac_external_loopback_test,
		.etest_flag = 0,
	},
	{
		.name = "NDMA internal loopback",
		.fn = adrv906x_ndma_loopback_test,
		.etest_flag = ETH_TEST_FL_OFFLINE,
	},
};

void adrv906x_ethtool_selftest_run(struct net_device *ndev, struct ethtool_test *etest, u64 *buf)
{
	int i, ret;
	unsigned char etest_flags = etest->flags;

	for (i = 0; i < ARRAY_SIZE(adrv906x_ethtool_selftests); i++) {
		ret = 0;
		if (etest_flags == adrv906x_ethtool_selftests[i].etest_flag)
			ret = adrv906x_ethtool_selftests[i].fn(ndev);
		if (ret)
			etest->flags |= ETH_TEST_FL_FAILED;
		buf[i] = ret;
	}
}

const struct ethtool_ops adrv906x_ethtool_ops = {
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= adrv906x_ethtool_set_link_ksettings,
	.get_fecparam		= adrv906x_ethtool_get_fecparam,
	.set_fecparam		= adrv906x_ethtool_set_fecparam,
	.get_ts_info		= adrv906x_ethtool_get_ts_info,
	.get_sset_count		= adrv906x_ethtool_get_sset_count,
	.get_strings		= adrv906x_ethtool_get_strings,
	.get_ethtool_stats	= adrv906x_ethtool_get_stats,
	.self_test		= adrv906x_ethtool_selftest_run,
};

MODULE_LICENSE("GPL");
