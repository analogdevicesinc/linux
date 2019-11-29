// SPDX-License-Identifier: GPL-2.0
/* Ethtool support for Intel FPGA Quad-Speed Ethernet MAC driver
 * Copyright (C) 2019 Intel Corporation. All rights reserved
 *
 * Contributors:
 *   Roman Bulgakov
 *   Yu Ying Choo
 *   Dalon Westergreen
 *   Joyce Ooi
 *
 * Original driver contributed by GlobalLogic.
 */

#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phylink.h>

#include "altera_eth_dma.h"
#include "intel_fpga_qse_ll.h"
#include "altera_utils.h"

#define QSE_STATS_LEN	ARRAY_SIZE(stat_gstrings)
#define QSE_NUM_REGS	196

static char const stat_gstrings[][ETH_GSTRING_LEN] = {
	"tx_packets",
	"rx_packets",
	"rx_crc_errors",
	"tx_bytes",
	"rx_bytes",
	"tx_pause",
	"rx_pause",
	"rx_errors",
	"tx_errors",
	"rx_unicast",
	"rx_multicast",
	"rx_broadcast",
	"tx_unicast",
	"tx_multicast",
	"tx_broadcast",
	"ether_drops",
	"rx_total_bytes",
	"rx_total_packets",
	"rx_undersize",
	"rx_oversize",
	"rx_64_bytes",
	"rx_65_127_bytes",
	"rx_128_255_bytes",
	"rx_256_511_bytes",
	"rx_512_1023_bytes",
	"rx_1024_1518_bytes",
	"rx_gte_1519_bytes",
	"rx_jabbers",
	"rx_runts",
};

static void qse_get_drvinfo(struct net_device *dev,
			    struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "intel_fpga_qse_ll", ETH_GSTRING_LEN);
	strscpy(info->version, "v1.0", ETH_GSTRING_LEN);
	strscpy(info->bus_info, "platform", ETH_GSTRING_LEN);
}

/* Fill in a buffer with the strings which correspond to the
 * stats
 */
static void qse_gstrings(struct net_device *dev, u32 stringset, u8 *buf)
{
	memcpy(buf, stat_gstrings, QSE_STATS_LEN * ETH_GSTRING_LEN);
}

static void qse_fill_stats(struct net_device *dev, struct ethtool_stats *dummy,
			   u64 *buf)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);
	u32 lsb;
	u32 msb;

	/* aFramesTransmittedOK */
	lsb = csrrd32(priv->mac_dev, qse_txstat_csroffs(frames_ok_lsb));
	msb = csrrd32(priv->mac_dev, qse_txstat_csroffs(frames_ok_msb));
	buf[0] = ((u64)msb << 32) | lsb;

	/* aFramesReceivedOK */
	lsb = csrrd32(priv->mac_dev, qse_rxstat_csroffs(frames_ok_lsb));
	msb = csrrd32(priv->mac_dev, qse_rxstat_csroffs(frames_ok_msb));
	buf[1] = ((u64)msb << 32) | lsb;

	/* aFrameCheckSequenceErrors */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(frames_crc_err_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(frames_crc_err_msb));
	buf[2] = ((u64)msb << 32) | lsb;

	/* aOctetsTransmittedOK */
	lsb = csrrd32(priv->mac_dev, qse_txstat_csroffs(octets_ok_lsb));
	msb = csrrd32(priv->mac_dev, qse_txstat_csroffs(octets_ok_msb));
	buf[3] = ((u64)msb << 32) | lsb;

	/* aOctetsReceivedOK */
	lsb = csrrd32(priv->mac_dev, qse_rxstat_csroffs(octets_ok_lsb));
	msb = csrrd32(priv->mac_dev, qse_rxstat_csroffs(octets_ok_msb));
	buf[4] = ((u64)msb << 32) | lsb;

	/* aPAUSEMACCtrlFramesTransmitted */
	lsb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(pause_mac_ctrl_frames_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(pause_mac_ctrl_frames_msb));
	buf[5] = ((u64)msb << 32) | lsb;

	/* aPAUSEMACCtrlFramesReceived */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(pause_mac_ctrl_frames_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(pause_mac_ctrl_frames_msb));
	buf[6] = ((u64)msb << 32) | lsb;

	/* ifInErrors */
	lsb = csrrd32(priv->mac_dev, qse_rxstat_csroffs(if_errors_lsb));
	msb = csrrd32(priv->mac_dev, qse_rxstat_csroffs(if_errors_msb));
	buf[7] = ((u64)msb << 32) | lsb;

	/* ifOutErrors */
	lsb = csrrd32(priv->mac_dev, qse_txstat_csroffs(if_errors_lsb));
	msb = csrrd32(priv->mac_dev, qse_txstat_csroffs(if_errors_msb));
	buf[8] = ((u64)msb << 32) | lsb;

	/* ifInUcastPkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(unicast_frames_ok_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(unicast_frames_ok_msb));
	buf[9] = ((u64)msb << 32) | lsb;

	/* ifInMulticastPkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(multicast_frames_ok_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(multicast_frames_ok_msb));
	buf[10] = ((u64)msb << 32) | lsb;

	/* ifInBroadcastPkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(broadcast_frames_ok_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(broadcast_frames_ok_msb));
	buf[11] = ((u64)msb << 32) | lsb;

	/* ifOutUcastPkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(unicast_frames_ok_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(unicast_frames_ok_msb));
	buf[12] = ((u64)msb << 32) | lsb;

	/* ifOutMulticastPkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(multicast_frames_ok_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(multicast_frames_ok_msb));
	buf[13] = ((u64)msb << 32) | lsb;

	/* ifOutBroadcastPkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(broadcast_frames_ok_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_txstat_csroffs(broadcast_frames_ok_msb));
	buf[14] = ((u64)msb << 32) | lsb;

	/* etherStatsDropEvents */
	lsb = csrrd32(priv->mac_dev,
		      qse_csroffs(rx_pktovrflow_eth_stats_dropevents0));
	msb = csrrd32(priv->mac_dev,
		      qse_csroffs(rx_pktovrflow_eth_stats_dropevents1));
	buf[15] = ((u64)msb << 32) | lsb;

	/* etherStatsOctets */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_oct_msb));
	buf[16] = ((u64)msb << 32) | lsb;

	/* etherStatsPkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_msb));
	buf[17] = ((u64)msb << 32) | lsb;

	/* etherStatsUndersizePkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_undersize_pkts_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_undersize_pkts_msb));
	buf[18] = ((u64)msb << 32) | lsb;

	/* etherStatsOversizePkts */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_oversize_pkts_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_oversize_pkts_msb));
	buf[19] = ((u64)msb << 32) | lsb;

	/* etherStatsPkts64Octets */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_64_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_64_oct_msb));
	buf[20] = ((u64)msb << 32) | lsb;

	/* etherStatsPkts65to127Octets */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_65to127_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_65to127_oct_msb));
	buf[21] = ((u64)msb << 32) | lsb;

	/* etherStatsPkts128to255Octets */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_128to255_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_128to255_oct_msb));
	buf[22] = ((u64)msb << 32) | lsb;

	/* etherStatsPkts256to511Octets */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_256to511_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_256to511_oct_msb));
	buf[23] = ((u64)msb << 32) | lsb;

	/* etherStatsPkts512to1023Octets */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_512to1023_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_512to1023_oct_msb));
	buf[24] = ((u64)msb << 32) | lsb;

	/* etherStatsPkts1024to1518Octets */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_1024to1518_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_1024to1518_oct_msb));
	buf[25] = ((u64)msb << 32) | lsb;

	/* This statistics counts the number of received good and errored
	 * frames between the length of 1519 and the maximum frame length
	 * configured in the rx_frame_maxlength register.
	 */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_1519tox_oct_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_pkts_1519tox_oct_msb));
	buf[26] = ((u64)msb << 32) | lsb;

	/* etherStatsJabbers */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_jabbers_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_jabbers_msb));
	buf[27] = ((u64)msb << 32) | lsb;

	/* etherStatsFragments */
	lsb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_fragments_lsb));
	msb = csrrd32(priv->mac_dev,
		      qse_rxstat_csroffs(eth_stats_fragments_msb));
	buf[28] = ((u64)msb << 32) | lsb;
}

static int qse_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return QSE_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static u32 qse_get_msglevel(struct net_device *dev)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void qse_set_msglevel(struct net_device *dev, uint32_t data)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);

	priv->msg_enable = data;
}

static int qse_reglen(struct net_device *dev)
{
	return QSE_NUM_REGS * sizeof(u32);
}

static void qse_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			 void *regbuf)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);
	u32 *buf = regbuf;

	/* Set version to a known value, so ethtool knows
	 * how to do any special formatting of this data.
	 * This version number will need to change if and
	 * when this register table is changed.
	 *
	 * version[31:0] = 1: Dump the 10GbE MAC IP Registers
	 *      Upper bits are all 0 by default
	 *
	 * Upper 16-bits will indicate feature presence for
	 * Ethtool register decoding in future version.
	 */

	regs->version = 1;

	buf[0] = csrrd32(priv->mac_dev, qse_csroffs(primary_mac_addr0));
	buf[1] = csrrd32(priv->mac_dev, qse_csroffs(primary_mac_addr1));
	buf[2] = csrrd32(priv->mac_dev, qse_csroffs(mac_reset_control));
	buf[3] = csrrd32(priv->mac_dev, qse_csroffs(tx_packet_control));
	buf[4] = csrrd32(priv->mac_dev, qse_csroffs(tx_transfer_status));

	buf[5] = csrrd32(priv->mac_dev, qse_csroffs(tx_pad_control));
	buf[6] = csrrd32(priv->mac_dev, qse_csroffs(tx_crc_control));
	buf[7] = csrrd32(priv->mac_dev, qse_csroffs(tx_preamble_control));
	buf[8] = csrrd32(priv->mac_dev, qse_csroffs(tx_src_addr_override));
	buf[9] = csrrd32(priv->mac_dev, qse_csroffs(tx_frame_maxlength));

	buf[10] = csrrd32(priv->mac_dev, qse_csroffs(tx_vlan_detection));
	buf[11] = csrrd32(priv->mac_dev, qse_csroffs(tx_ipg_10g));
	buf[12] = csrrd32(priv->mac_dev, qse_csroffs(tx_ipg_10m_100m_1g));
	buf[13] = csrrd32(priv->mac_dev,
			  qse_csroffs(tx_underflow_counter0));
	buf[14] = csrrd32(priv->mac_dev,
			  qse_csroffs(tx_underflow_counter1));

	buf[15] = csrrd32(priv->mac_dev, qse_csroffs(tx_pauseframe_control));
	buf[16] = csrrd32(priv->mac_dev, qse_csroffs(tx_pauseframe_quanta));
	buf[17] = csrrd32(priv->mac_dev,
			  qse_csroffs(tx_pauseframe_holdoff_quanta));
	buf[18] = csrrd32(priv->mac_dev, qse_csroffs(tx_pauseframe_enable));
	buf[19] = csrrd32(priv->mac_dev,
			  qse_csroffs(tx_pfc_priority_enable));

	buf[20] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_0));
	buf[21] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_1));
	buf[22] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_2));
	buf[23] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_3));
	buf[24] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_4));

	buf[25] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_5));
	buf[26] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_6));
	buf[27] = csrrd32(priv->mac_dev, qse_csroffs(pfc_pause_quanta_7));
	buf[28] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_0));
	buf[29] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_1));

	buf[30] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_2));
	buf[31] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_3));
	buf[32] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_4));
	buf[33] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_5));
	buf[34] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_6));

	buf[35] = csrrd32(priv->mac_dev, qse_csroffs(pfc_holdoff_quanta_7));
	buf[36] = csrrd32(priv->mac_dev, qse_csroffs(tx_unidir_control));
	buf[37] = csrrd32(priv->mac_dev, qse_csroffs(rx_transfer_control));
	buf[38] = csrrd32(priv->mac_dev, qse_csroffs(rx_transfer_status));
	buf[39] = csrrd32(priv->mac_dev, qse_csroffs(rx_padcrc_control));

	buf[40] = csrrd32(priv->mac_dev, qse_csroffs(rx_crccheck_control));
	buf[41] = csrrd32(priv->mac_dev,
			  qse_csroffs(rx_custom_preamble_forward));
	buf[42] = csrrd32(priv->mac_dev, qse_csroffs(rx_preamble_control));
	buf[43] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_control));
	buf[44] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_maxlength));

	buf[45] = csrrd32(priv->mac_dev, qse_csroffs(rx_vlan_detection));
	buf[46] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr0_0));
	buf[47] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr0_1));
	buf[48] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr1_0));
	buf[49] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr1_1));

	buf[50] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr2_0));
	buf[51] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr2_1));
	buf[52] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr3_0));
	buf[53] = csrrd32(priv->mac_dev, qse_csroffs(rx_frame_spaddr3_1));

	buf[54] = csrrd32(priv->mac_dev, qse_csroffs(rx_pfc_control));
	buf[55] = csrrd32(priv->mac_dev, qse_csroffs(rx_pktovrflow_error0));
	buf[56] = csrrd32(priv->mac_dev, qse_csroffs(rx_pktovrflow_error1));
	buf[57] = csrrd32(priv->mac_dev,
			  qse_csroffs(rx_pktovrflow_eth_stats_dropevents0));
	buf[58] = csrrd32(priv->mac_dev,
			  qse_csroffs(rx_pktovrflow_eth_stats_dropevents1));

	/* TX Timestamp */
	buf[59] = csrrd32(priv->mac_dev, qse_tx_ts_csroffs(period_10g));
	buf[60] = csrrd32(priv->mac_dev,
			  qse_tx_ts_csroffs(fns_adjustment_10g));
	buf[61] = csrrd32(priv->mac_dev, qse_tx_ts_csroffs(ns_adjustment_10g));
	buf[62] = csrrd32(priv->mac_dev, qse_tx_ts_csroffs(period_mult_speed));
	buf[63] = csrrd32(priv->mac_dev,
			  qse_tx_ts_csroffs(fns_adjustment_mult_speed));
	buf[64] = csrrd32(priv->mac_dev,
			  qse_tx_ts_csroffs(ns_adjustment_mult_speed));
	buf[65] = csrrd32(priv->mac_dev, qse_csroffs(tx_asymmetry));

	/* RX Timestamp */
	buf[66] = csrrd32(priv->mac_dev, qse_rx_ts_csroffs(period_10g));
	buf[67] = csrrd32(priv->mac_dev,
			  qse_rx_ts_csroffs(fns_adjustment_10g));
	buf[68] = csrrd32(priv->mac_dev, qse_rx_ts_csroffs(ns_adjustment_10g));
	buf[69] = csrrd32(priv->mac_dev, qse_rx_ts_csroffs(period_mult_speed));
	buf[70] = csrrd32(priv->mac_dev,
			  qse_rx_ts_csroffs(fns_adjustment_mult_speed));
	buf[71] = csrrd32(priv->mac_dev,
			  qse_rx_ts_csroffs(ns_adjustment_mult_speed));

	/* TX Stats */
	buf[72] = csrrd32(priv->mac_dev, qse_txstat_csroffs(clear));
	buf[73] = csrrd32(priv->mac_dev, qse_txstat_csroffs(frames_ok_lsb));
	buf[74] = csrrd32(priv->mac_dev, qse_txstat_csroffs(frames_ok_msb));
	buf[75] = csrrd32(priv->mac_dev, qse_txstat_csroffs(frames_err_lsb));
	buf[76] = csrrd32(priv->mac_dev, qse_txstat_csroffs(frames_err_msb));
	buf[77] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(frames_crc_err_lsb));
	buf[78] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(frames_crc_err_msb));
	buf[79] = csrrd32(priv->mac_dev, qse_txstat_csroffs(octets_ok_lsb));
	buf[80] = csrrd32(priv->mac_dev, qse_txstat_csroffs(octets_ok_msb));
	buf[81] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(pause_mac_ctrl_frames_lsb));
	buf[82] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(pause_mac_ctrl_frames_msb));
	buf[83] = csrrd32(priv->mac_dev, qse_txstat_csroffs(if_errors_lsb));
	buf[84] = csrrd32(priv->mac_dev, qse_txstat_csroffs(if_errors_msb));
	buf[85] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(unicast_frames_ok_lsb));
	buf[86] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(unicast_frames_ok_msb));
	buf[87] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(unicast_frames_err_lsb));
	buf[88] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(unicast_frames_err_msb));
	buf[89] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(multicast_frames_ok_lsb));
	buf[90] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(multicast_frames_ok_msb));
	buf[91] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(multicast_frames_err_lsb));
	buf[92] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(multicast_frames_err_msb));
	buf[93] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(broadcast_frames_ok_lsb));
	buf[94] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(broadcast_frames_ok_msb));
	buf[95] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(broadcast_frames_err_lsb));
	buf[96] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(broadcast_frames_err_msb));
	buf[97] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(eth_stats_oct_lsb));
	buf[98] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(eth_stats_oct_msb));
	buf[99] = csrrd32(priv->mac_dev,
			  qse_txstat_csroffs(eth_stats_pkts_lsb));
	buf[100] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_msb));
	buf[101] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_undersize_pkts_lsb));
	buf[102] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_undersize_pkts_msb));
	buf[103] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_oversize_pkts_lsb));
	buf[104] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_oversize_pkts_msb));
	buf[105] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_64_oct_lsb));
	buf[106] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_64_oct_msb));
	buf[107] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_65to127_oct_lsb));
	buf[108] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_65to127_oct_msb));
	buf[109] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_128to255_oct_lsb));
	buf[100] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_128to255_oct_msb));
	buf[111] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_256to511_oct_lsb));
	buf[112] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_256to511_oct_msb));
	buf[113] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_512to1023_oct_lsb));
	buf[114] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_512to1023_oct_msb));
	buf[115] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_1024to1518_oct_lsb));
	buf[116] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_1024to1518_oct_msb));
	buf[117] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_1519tox_oct_lsb));
	buf[118] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_pkts_1519tox_oct_msb));
	buf[119] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_fragments_lsb));
	buf[120] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_fragments_msb));
	buf[121] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_jabbers_lsb));
	buf[122] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_jabbers_msb));
	buf[123] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_crc_err_lsb));
	buf[124] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(eth_stats_crc_err_msb));
	buf[125] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(unicast_mac_ctrl_frames_lsb));
	buf[126] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(unicast_mac_ctrl_frames_msb));
	buf[127] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(multicast_mac_ctrl_frames_lsb));
	buf[128] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(multicast_mac_ctrl_frames_msb));
	buf[129] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(broadcast_mac_ctrl_frames_lsb));
	buf[130] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(broadcast_mac_ctrl_frames_msb));
	buf[131] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(pfc_mac_ctrl_frames_lsb));
	buf[132] = csrrd32(priv->mac_dev,
			   qse_txstat_csroffs(pfc_mac_ctrl_frames_msb));

	/* RX Stats */
	buf[133] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(clear));
	buf[134] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(frames_ok_lsb));
	buf[135] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(frames_ok_msb));
	buf[136] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(frames_err_lsb));
	buf[137] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(frames_err_msb));
	buf[138] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(frames_crc_err_lsb));
	buf[139] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(frames_crc_err_msb));
	buf[140] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(octets_ok_lsb));
	buf[141] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(octets_ok_msb));
	buf[142] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(pause_mac_ctrl_frames_lsb));
	buf[143] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(pause_mac_ctrl_frames_msb));
	buf[144] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(if_errors_lsb));
	buf[145] = csrrd32(priv->mac_dev, qse_rxstat_csroffs(if_errors_msb));
	buf[146] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(unicast_frames_ok_lsb));
	buf[147] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(unicast_frames_ok_msb));
	buf[148] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(unicast_frames_err_lsb));
	buf[149] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(unicast_frames_err_msb));
	buf[150] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(multicast_frames_ok_lsb));
	buf[151] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(multicast_frames_ok_msb));
	buf[152] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(multicast_frames_err_lsb));
	buf[153] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(multicast_frames_err_msb));
	buf[154] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(broadcast_frames_ok_lsb));
	buf[155] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(broadcast_frames_ok_msb));
	buf[156] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(broadcast_frames_err_lsb));
	buf[157] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(broadcast_frames_err_msb));
	buf[158] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_oct_lsb));
	buf[159] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_oct_msb));
	buf[160] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_lsb));
	buf[161] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_msb));
	buf[162] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_undersize_pkts_lsb));
	buf[163] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_undersize_pkts_msb));
	buf[164] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_oversize_pkts_lsb));
	buf[165] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_oversize_pkts_msb));
	buf[166] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_64_oct_lsb));
	buf[167] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_64_oct_msb));
	buf[168] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_65to127_oct_lsb));
	buf[169] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_65to127_oct_msb));
	buf[170] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_128to255_oct_lsb));
	buf[171] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_128to255_oct_msb));
	buf[172] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_256to511_oct_lsb));
	buf[173] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_256to511_oct_msb));
	buf[174] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_512to1023_oct_lsb));
	buf[175] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_512to1023_oct_msb));
	buf[176] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_1024to1518_oct_lsb));
	buf[177] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_1024to1518_oct_msb));
	buf[178] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_1519tox_oct_lsb));
	buf[179] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_pkts_1519tox_oct_msb));
	buf[180] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_fragments_lsb));
	buf[181] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_fragments_msb));
	buf[182] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_jabbers_lsb));
	buf[183] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_jabbers_msb));
	buf[184] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_crc_err_lsb));
	buf[185] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(eth_stats_crc_err_msb));
	buf[186] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(unicast_mac_ctrl_frames_lsb));
	buf[187] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(unicast_mac_ctrl_frames_msb));
	buf[188] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(multicast_mac_ctrl_frames_lsb));
	buf[189] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(multicast_mac_ctrl_frames_msb));
	buf[190] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(broadcast_mac_ctrl_frames_lsb));
	buf[191] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(broadcast_mac_ctrl_frames_msb));
	buf[192] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(pfc_mac_ctrl_frames_lsb));
	buf[193] = csrrd32(priv->mac_dev,
			   qse_rxstat_csroffs(pfc_mac_ctrl_frames_msb));

	buf[194] = csrrd32(priv->mac_dev, qse_csroffs(ecc_status));
	buf[195] = csrrd32(priv->mac_dev, qse_csroffs(ecc_enable));
}

static void qse_get_pauseparam(struct net_device *dev,
			       struct ethtool_pauseparam *pauseparam)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);

	pauseparam->rx_pause = 0;
	pauseparam->tx_pause = 0;
	pauseparam->autoneg = 0;

	if (priv->flow_ctrl & FLOW_RX)
		pauseparam->rx_pause = 1;
	if (priv->flow_ctrl & FLOW_TX)
		pauseparam->tx_pause = 1;
}

static int qse_set_pauseparam(struct net_device *dev,
			      struct ethtool_pauseparam *pauseparam)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);
	int new_pause = FLOW_OFF;
	int ret = 0;

	spin_lock(&priv->mac_cfg_lock);

	if (pauseparam->autoneg != 0) {
		ret = -EINVAL;
		goto out;
	}

	if (pauseparam->rx_pause) {
		new_pause |= FLOW_RX;
		tse_clear_bit(priv->mac_dev, qse_csroffs(rx_frame_control),
			      MAC_RX_FRM_CTL_IGNORE_PAUSE);
	} else {
		tse_set_bit(priv->mac_dev, qse_csroffs(rx_frame_control),
			    MAC_RX_FRM_CTL_IGNORE_PAUSE);
	}

	if (pauseparam->tx_pause) {
		new_pause |= FLOW_TX;
		tse_set_bit(priv->mac_dev,
			    qse_csroffs(tx_pauseframe_enable),
			    MAC_TX_PAUSEFRAME_ENA);
	} else {
		tse_clear_bit(priv->mac_dev,
			      qse_csroffs(tx_pauseframe_enable),
			      MAC_TX_PAUSEFRAME_ENA);
	}

	csrwr32(priv->pause, priv->mac_dev,
		qse_csroffs(tx_pauseframe_quanta));
	priv->flow_ctrl = new_pause;
out:
	spin_unlock(&priv->mac_cfg_lock);
	return ret;
}

static int qse_get_ts_info(struct net_device *dev,
			   struct ethtool_ts_info *info)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);

	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;

	if (priv->ptp_priv.ptp_clock)
		info->phc_index = ptp_clock_index(priv->ptp_priv.ptp_clock);
	else
		info->phc_index = -1;

	info->tx_types = (1 << HWTSTAMP_TX_OFF) |
			 (1 << HWTSTAMP_TX_ON) |
			 (1 << HWTSTAMP_TX_ONESTEP_SYNC);

	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);

	return 0;
}

/* Set link ksettings (phy address, speed) for ethtools */
static int qse_set_link_ksettings(struct net_device *dev,
				  const struct ethtool_link_ksettings *cmd)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

/* Get link ksettings (phy address, speed) for ethtools */
static int qse_get_link_ksettings(struct net_device *dev,
				  struct ethtool_link_ksettings *cmd)
{
	struct intel_fpga_qse_private *priv = netdev_priv(dev);

	if (!priv)
		return -ENODEV;

	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static const struct ethtool_ops qse_ll_ethtool_ops = {
	.get_drvinfo = qse_get_drvinfo,
	.get_regs_len = qse_reglen,
	.get_regs = qse_get_regs,
	.get_link = ethtool_op_get_link,
	.get_link = ethtool_op_get_link,
	.get_strings = qse_gstrings,
	.get_sset_count = qse_sset_count,
	.get_ethtool_stats = qse_fill_stats,
	.get_msglevel = qse_get_msglevel,
	.set_msglevel = qse_set_msglevel,
	.get_pauseparam = qse_get_pauseparam,
	.set_pauseparam = qse_set_pauseparam,
	.get_ts_info = qse_get_ts_info,
	.get_link_ksettings = qse_get_link_ksettings,
	.set_link_ksettings = qse_set_link_ksettings,
};

void intel_fpga_qse_ll_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &qse_ll_ethtool_ops;
}
