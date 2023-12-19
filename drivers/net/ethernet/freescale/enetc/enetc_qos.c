// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2019 NXP */

#include "enetc_pf.h"

#include <net/pkt_sched.h>
#include <linux/math64.h>
#include <linux/refcount.h>
#include <net/pkt_cls.h>
#include <net/tc_act/tc_gate.h>

static u16 enetc_get_max_gcl_len(struct enetc_hw *hw)
{
	return enetc_rd(hw, ENETC_PTGCAPR) & ENETC_PTGCAPR_MAX_GCL_LEN_MASK;
}

static int enetc_setup_taprio(struct enetc_ndev_priv *priv,
			      struct tc_taprio_qopt_offload *admin_conf)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &priv->si->hw;
	struct enetc_cbd cbd = {.cmd = 0};
	struct tgs_gcl_conf *gcl_config;
	struct tgs_gcl_data *gcl_data;
	dma_addr_t dma;
	struct gce *gce;
	u16 data_size;
	u16 gcl_len;
	void *tmp;
	int err;
	int i;

	if (!pf->hw_ops->set_time_gating || !pf->hw_ops->set_tc_msdu)
		return -EOPNOTSUPP;

	if (admin_conf->num_entries > enetc_get_max_gcl_len(hw))
		return -EINVAL;

	/* Configure the (administrative) gate control list using the
	 * control BD descriptor.
	 */
	gcl_config = &cbd.gcl_conf;
	gcl_len = admin_conf->num_entries;

	data_size = struct_size(gcl_data, entry, gcl_len);
	tmp = enetc_cbd_alloc_data_mem(priv->si, &cbd, data_size,
				       &dma, (void *)&gcl_data);
	if (!tmp)
		return -ENOMEM;

	gce = (struct gce *)(gcl_data + 1);

	/* Set all gates open as default */
	gcl_config->atc = 0xff;
	gcl_config->acl_len = cpu_to_le16(gcl_len);

	gcl_data->btl = cpu_to_le32(lower_32_bits(admin_conf->base_time));
	gcl_data->bth = cpu_to_le32(upper_32_bits(admin_conf->base_time));
	gcl_data->ct = cpu_to_le32(admin_conf->cycle_time);
	gcl_data->cte = cpu_to_le32(admin_conf->cycle_time_extension);

	for (i = 0; i < gcl_len; i++) {
		struct tc_taprio_sched_entry *temp_entry;
		struct gce *temp_gce = gce + i;

		temp_entry = &admin_conf->entries[i];

		temp_gce->gate = (u8)temp_entry->gate_mask;
		temp_gce->period = cpu_to_le32(temp_entry->interval);
	}

	cbd.status_flags = 0;

	cbd.cls = BDCR_CMD_PORT_GCL;
	cbd.status_flags = 0;

	pf->hw_ops->set_time_gating(hw, true);

	err = enetc_send_cmd(priv->si, &cbd);
	if (err)
		pf->hw_ops->set_time_gating(hw, false);

	enetc_cbd_free_data_mem(priv->si, data_size, tmp, &dma);

	if (err)
		return err;

	pf->hw_ops->set_tc_msdu(hw, admin_conf->max_sdu);
	priv->active_offloads |= ENETC_F_QBV;

	return 0;
}

static u32 enetc4_get_tgst_free_words(struct enetc_hw *hw)
{
	u32 words_in_use;
	u32 total_words;

	/* Notice that the admin gate list should be delete first before call
	 * this function, so the ENETC4_PTGAGLLR[ADMIN_GATE_LIST_LENGTH] equal
	 * to zero. That is, the ENETC4_TGSTMOR only contains the words of the
	 * operational gate control list.
	 */
	words_in_use = enetc_port_rd(hw, ENETC4_TGSTMOR) & TGSTMOR_NUM_WORDS;
	total_words = enetc_port_rd(hw, ENETC4_TGSTCAPR) & TGSTCAPR_NUM_WORDS;

	return total_words - words_in_use;
}

static int enetc4_setup_taprio(struct enetc_ndev_priv *priv,
			       struct tc_taprio_qopt_offload *admin_conf)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	struct ntmp_tgst_cfg *cfg;
	u64 max_cycle_time;
	int port, i, err;
	bool tge_enable;
	u32 size;

	if (!pf->hw_ops->set_time_gating || !pf->hw_ops->get_time_gating ||
	    !pf->hw_ops->set_tc_msdu)
		return -EOPNOTSUPP;

	port = enetc4_pf_to_port(si->pdev);
	if (port < 0)
		return -EINVAL;

	tge_enable = pf->hw_ops->get_time_gating(hw);
	if (!tge_enable)
		pf->hw_ops->set_time_gating(hw, true);

	/* Delete the pending administrative control list if it exists */
	err = ntmp_tgst_delete_admin_gate_list(&si->cbdr, port);
	if (err)
		goto disable_tge;

	if (admin_conf->num_entries > enetc4_get_tgst_free_words(hw)) {
		err = -EINVAL;
		goto disable_tge;
	}

	max_cycle_time = admin_conf->cycle_time + admin_conf->cycle_time_extension;
	if (max_cycle_time > NTMP_TGST_MAX_CT_PLUS_CT_EXT) {
		err = -EINVAL;
		goto disable_tge;
	}

	size = struct_size(cfg, entries, admin_conf->num_entries);
	cfg = kzalloc(size, GFP_KERNEL);
	if (!cfg) {
		err = -ENOMEM;
		goto disable_tge;
	}

	cfg->base_time = admin_conf->base_time;
	cfg->cycle_time = admin_conf->cycle_time;
	cfg->cycle_time_extension = admin_conf->cycle_time_extension;
	cfg->num_entries = admin_conf->num_entries;
	for (i = 0; i < cfg->num_entries; i++) {
		struct tc_taprio_sched_entry *temp_entry = &admin_conf->entries[i];
		struct ntmp_tgst_ge *temp_ge = &cfg->entries[i];

		temp_ge->tc_gates = temp_entry->gate_mask;
		temp_ge->interval = temp_entry->interval;
	}

	/* Set the maximum frame size for each traffic class */
	pf->hw_ops->set_tc_msdu(hw, admin_conf->max_sdu);

	err = ntmp_tgst_update_admin_gate_list(&si->cbdr, port, cfg);
	if (err) {
		kfree(cfg);
		goto disable_tge;
	}

	kfree(cfg);
	priv->active_offloads |= ENETC_F_QBV;

	return 0;

disable_tge:
	/* We should disable tge if its initial state is disabled */
	if (!tge_enable)
		pf->hw_ops->set_time_gating(hw, false);

	return err;
}

static void enetc_reset_taprio_stats(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_tx_rings; i++)
		priv->tx_ring[i]->stats.win_drop = 0;
}

static void enetc_reset_taprio(struct enetc_ndev_priv *priv)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &priv->si->hw;

	if (pf->hw_ops->set_time_gating)
		pf->hw_ops->set_time_gating(hw, false);

	if (pf->hw_ops->reset_tc_msdu)
		pf->hw_ops->reset_tc_msdu(hw);

	priv->active_offloads &= ~ENETC_F_QBV;
}

static void enetc_taprio_destroy(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	enetc_reset_taprio(priv);
	enetc_reset_tc_mqprio(ndev);
	enetc_reset_taprio_stats(priv);
}

static void enetc_taprio_stats(struct net_device *ndev,
			       struct tc_taprio_qopt_stats *stats)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	u64 window_drops = 0;
	int i;

	for (i = 0; i < priv->num_tx_rings; i++)
		window_drops += priv->tx_ring[i]->stats.win_drop;

	stats->window_drops = window_drops;
}

static void enetc_taprio_queue_stats(struct net_device *ndev,
				     struct tc_taprio_qopt_queue_stats *queue_stats)
{
	struct tc_taprio_qopt_stats *stats = &queue_stats->stats;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int queue = queue_stats->queue;

	stats->window_drops = priv->tx_ring[queue]->stats.win_drop;
}

static int enetc_taprio_replace(struct net_device *ndev,
				struct tc_taprio_qopt_offload *offload)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int i, err;

	err = enetc_setup_tc_mqprio(ndev, &offload->mqprio);
	if (err)
		return err;

	/* TSD and Qbv are mutually exclusive in hardware */
	for (i = 0; i < priv->num_tx_rings; i++)
		if (priv->tx_ring[i]->tsd_enable)
			return -EBUSY;

	if (offload->cycle_time > U32_MAX ||
	    offload->cycle_time_extension > U32_MAX)
		return -EINVAL;

	if (is_enetc_rev1(si))
		err = enetc_setup_taprio(priv, offload);
	else
		err = enetc4_setup_taprio(priv, offload);
	if (err)
		enetc_reset_tc_mqprio(ndev);

	return err;
}

int enetc_setup_tc_taprio(struct net_device *ndev, void *type_data)
{
	struct tc_taprio_qopt_offload *offload = type_data;
	int err = 0;

	switch (offload->cmd) {
	case TAPRIO_CMD_REPLACE:
		err = enetc_taprio_replace(ndev, offload);
		break;
	case TAPRIO_CMD_DESTROY:
		enetc_taprio_destroy(ndev);
		break;
	case TAPRIO_CMD_STATS:
		enetc_taprio_stats(ndev, &offload->stats);
		break;
	case TAPRIO_CMD_QUEUE_STATS:
		enetc_taprio_queue_stats(ndev, &offload->queue_stats);
		break;
	default:
		err = -EOPNOTSUPP;
	}

	return err;
}

static u32 enetc_get_cbs_enable(struct enetc_hw *hw, u8 tc)
{
	return enetc_port_rd(hw, ENETC_PTCCBSR0(tc)) & ENETC_CBSE;
}

static u8 enetc_get_cbs_bw(struct enetc_hw *hw, u8 tc)
{
	return enetc_port_rd(hw, ENETC_PTCCBSR0(tc)) & ENETC_CBS_BW_MASK;
}

static int enetc_configure_tc_cbs(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_cbs_qopt_offload *cbs = type_data;
	u32 port_transmit_rate = priv->speed;
	u8 tc_nums = netdev_get_num_tc(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	u32 hi_credit_bit, hi_credit_reg;
	u32 max_interference_size;
	u32 port_frame_max_size;
	u8 tc = cbs->queue;
	u8 prio_top, prio_next;
	int bw_sum = 0;
	u8 bw;

	prio_top = tc_nums - 1;
	prio_next = tc_nums - 2;

	/* Support highest prio and second prio tc in cbs mode */
	if (tc != prio_top && tc != prio_next)
		return -EOPNOTSUPP;

	if (!cbs->enable) {
		/* Make sure the other TC that are numerically
		 * lower than this TC have been disabled.
		 */
		if (tc == prio_top &&
		    enetc_get_cbs_enable(hw, prio_next)) {
			dev_err(&ndev->dev,
				"Disable TC%d before disable TC%d\n",
				prio_next, tc);
			return -EINVAL;
		}

		enetc_port_wr(hw, ENETC_PTCCBSR1(tc), 0);
		enetc_port_wr(hw, ENETC_PTCCBSR0(tc), 0);

		return 0;
	}

	if (cbs->idleslope - cbs->sendslope != port_transmit_rate * 1000L ||
	    cbs->idleslope < 0 || cbs->sendslope > 0)
		return -EOPNOTSUPP;

	port_frame_max_size = ndev->mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;

	bw = cbs->idleslope / (port_transmit_rate * 10UL);

	/* Make sure the other TC that are numerically
	 * higher than this TC have been enabled.
	 */
	if (tc == prio_next) {
		if (!enetc_get_cbs_enable(hw, prio_top)) {
			dev_err(&ndev->dev,
				"Enable TC%d first before enable TC%d\n",
				prio_top, prio_next);
			return -EINVAL;
		}
		bw_sum += enetc_get_cbs_bw(hw, prio_top);
	}

	if (bw_sum + bw >= 100) {
		dev_err(&ndev->dev,
			"The sum of all CBS Bandwidth can't exceed 100\n");
		return -EINVAL;
	}

	enetc_port_rd(hw, ENETC_PTCMSDUR(tc));

	/* For top prio TC, the max_interfrence_size is maxSizedFrame.
	 *
	 * For next prio TC, the max_interfrence_size is calculated as below:
	 *
	 *      max_interference_size = M0 + Ma + Ra * M0 / (R0 - Ra)
	 *
	 *	- RA: idleSlope for AVB Class A
	 *	- R0: port transmit rate
	 *	- M0: maximum sized frame for the port
	 *	- MA: maximum sized frame for AVB Class A
	 */

	if (tc == prio_top) {
		max_interference_size = port_frame_max_size * 8;
	} else {
		u32 m0, ma, r0, ra;

		m0 = port_frame_max_size * 8;
		ma = enetc_port_rd(hw, ENETC_PTCMSDUR(prio_top)) * 8;
		ra = enetc_get_cbs_bw(hw, prio_top) *
			port_transmit_rate * 10000ULL;
		r0 = port_transmit_rate * 1000000ULL;
		max_interference_size = m0 + ma +
			(u32)div_u64((u64)ra * m0, r0 - ra);
	}

	/* hiCredit bits calculate by:
	 *
	 * maxSizedFrame * (idleSlope/portTxRate)
	 */
	hi_credit_bit = max_interference_size * bw / 100;

	/* hiCredit bits to hiCredit register need to calculated as:
	 *
	 * (enetClockFrequency / portTransmitRate) * 100
	 */
	hi_credit_reg = (u32)div_u64((ENETC_CLK * 100ULL) * hi_credit_bit,
				     port_transmit_rate * 1000000ULL);

	enetc_port_wr(hw, ENETC_PTCCBSR1(tc), hi_credit_reg);

	/* Set bw register and enable this traffic class */
	enetc_port_wr(hw, ENETC_PTCCBSR0(tc), bw | ENETC_CBSE);

	return 0;
}

static inline u32 enetc4_get_cbs_enable(struct enetc_hw *hw, int tc)
{
	return enetc_port_rd(hw, ENETC4_PTCCBSR0(tc)) & PTCCBSR0_CBSE;
}

static void enetc4_set_tc_cbs_params(struct enetc_hw *hw, int tc,
				     bool en, u32 bw, u32 hi_credit)
{
	if (en) {
		u32 val = PTCCBSR0_CBSE;

		val |= (bw / 10) & PTCCBSR0_BW;
		val |= (bw % 10) << 16;

		enetc_port_wr(hw, ENETC4_PTCCBSR1(tc), hi_credit);
		enetc_port_wr(hw, ENETC4_PTCCBSR0(tc), val);

	} else {
		enetc_port_wr(hw, ENETC4_PTCCBSR1(tc), 0);
		enetc_port_wr(hw, ENETC4_PTCCBSR0(tc), 0);
	}
}

static u32 enetc4_get_cbs_bw(struct enetc_hw *hw, int tc)
{
	u32 val, bw;

	val = enetc_port_rd(hw, ENETC4_PTCCBSR0(tc));
	bw = (val & PTCCBSR0_BW) * 10 + PTCCBSR0_GET_FRACT(val);

	return bw;
}

static inline u32 enetc4_get_tc_msdu(struct enetc_hw *hw, int tc)
{
	return enetc_port_rd(hw, ENETC4_PTCTMSDUR(tc)) & PTCTMSDUR_MAXSDU;
}

static int enetc4_configure_tc_cbs(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_cbs_qopt_offload *cbs = type_data;
	u32 port_transmit_rate = priv->speed;
	u8 tc_nums = netdev_get_num_tc(ndev);
	u32 hi_credit_bit, hi_credit_reg;
	u8 high_prio_tc, second_prio_tc;
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	u32 max_interference_size;
	u32 port_frame_max_size;
	u32 bw, bw_sum;
	u8 tc;

	high_prio_tc = tc_nums - 1;
	second_prio_tc = tc_nums - 2;

	tc = netdev_txq_to_tc(ndev, cbs->queue);

	/* Support highest prio and second prio tc in cbs mode */
	if (tc != high_prio_tc && tc != second_prio_tc)
		return -EOPNOTSUPP;

	if (!cbs->enable) {
		/* Make sure the other TC that are numerically
		 * lower than this TC have been disabled.
		 */
		if (tc == high_prio_tc &&
		    enetc4_get_cbs_enable(hw, second_prio_tc)) {
			dev_err(&ndev->dev,
				"Disable TC%d before disable TC%d\n",
				second_prio_tc, tc);
			return -EINVAL;
		}

		enetc4_set_tc_cbs_params(hw, tc, false, 0, 0);

		return 0;
	}

	/* The unit of idleslope and sendslope is kbps. And the sendslope should be
	 * a negative number, it can be calculated as follows, IEEE 802.1Q-2014
	 * Section 8.6.8.2 item g):
	 * sendslope = idleslope - port_transmit_rate
	 */
	if (cbs->idleslope - cbs->sendslope != port_transmit_rate * 1000L ||
	    cbs->idleslope < 0 || cbs->sendslope > 0)
		return -EOPNOTSUPP;

	port_frame_max_size = ndev->mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;

	/* The unit of port_transmit_rate is Mbps, the unit of bw is 1/1000 */
	bw = cbs->idleslope / port_transmit_rate;
	bw_sum = bw;

	/* Make sure the credit-based shaper of highest priority TC has been enabled
	 * before the secondary priority TC.
	 */
	if (tc == second_prio_tc) {
		if (!enetc4_get_cbs_enable(hw, high_prio_tc)) {
			dev_err(&ndev->dev,
				"Enable TC%d first before enable TC%d\n",
				high_prio_tc, second_prio_tc);
			return -EINVAL;
		}
		bw_sum += enetc4_get_cbs_bw(hw, high_prio_tc);
	}

	if (bw_sum >= 1000) {
		dev_err(&ndev->dev,
			"The sum of all CBS Bandwidth can't exceed 1000\n");
		return -EINVAL;
	}

	/* For the AVB Class A (highest priority TC), the max_interfrence_size is
	 * maximum sized frame for the port.
	 * For the AVB Class B (second highest priority TC), the max_interfrence_size
	 * is calculated as below:
	 *
	 *      max_interference_size = (Ra * M0) / (R0 - Ra) + MA + M0
	 *
	 *	- RA: idleSlope for AVB Class A
	 *	- R0: port transmit rate
	 *	- M0: maximum sized frame for the port
	 *	- MA: maximum sized frame for AVB Class A
	 */

	if (tc == high_prio_tc) {
		max_interference_size = port_frame_max_size * 8;
	} else {
		u32 m0, ma;
		u64 ra, r0;

		m0 = port_frame_max_size * 8;
		ma = enetc4_get_tc_msdu(hw, high_prio_tc) * 8;
		ra = enetc4_get_cbs_bw(hw, high_prio_tc) *
		     port_transmit_rate * 1000ULL;
		r0 = port_transmit_rate * 1000000ULL;
		max_interference_size = m0 + ma + (u32)div_u64(ra * m0, r0 - ra);
	}

	/* hiCredit bits calculate by:
	 *
	 * max_interference_size * (idleslope / port_transmit_rate)
	 */
	hi_credit_bit = max_interference_size * bw / 1000;

	/* Number of credits per bit is calculated as follows:
	 *
	 * (enetClockFrequency / port_transmit_rate) * 100
	 */
	hi_credit_reg = (u32)div_u64((ENETC4_CLK * 1000ULL) * hi_credit_bit,
				     port_transmit_rate * 1000000ULL);

	enetc4_set_tc_cbs_params(hw, tc, true, bw, hi_credit_reg);

	return 0;
}

int enetc_setup_tc_cbs(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	if (is_enetc_rev1(priv->si))
		return enetc_configure_tc_cbs(ndev, type_data);
	else
		return enetc4_configure_tc_cbs(ndev, type_data);
}

int enetc_setup_tc_txtime(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct tc_etf_qopt_offload *qopt = type_data;
	u8 tc_nums = netdev_get_num_tc(ndev);
	struct enetc_hw *hw = &pf->si->hw;
	int i, tc;

	if (!tc_nums || !pf->hw_ops->set_tc_tsd)
		return -EOPNOTSUPP;

	if (qopt->queue < 0 || qopt->queue >= ndev->real_num_tx_queues)
		return -EINVAL;

	/* TSD and Qbv are mutually exclusive in hardware */
	if (pf->hw_ops->get_time_gating && pf->hw_ops->get_time_gating(hw))
		return -EBUSY;

	tc = netdev_txq_to_tc(ndev, qopt->queue);
	/* According to the NETC block guide, time specific departure operation
	 * should only be used on the highest priority traffic class.
	 */
	if (tc != tc_nums - 1) {
		dev_err(&ndev->dev,
			"TSD should be used on the highest priority TC:%d!\n",
			tc_nums - 1);
		return -EINVAL;
	}

	/* Accordiing to the NETC block guide, all traffic on the traffic class
	 * should use time specific departure operation.
	 */
	for (i = 0; i < ndev->tc_to_txq[tc].count; i++) {
		u16 offset = ndev->tc_to_txq[tc].offset + i;

		priv->tx_ring[offset]->tsd_enable = qopt->enable;
	}

	pf->hw_ops->set_tc_tsd(hw, tc, qopt->enable);

	return 0;
}

enum streamid_type {
	STREAMID_TYPE_RESERVED = 0,
	STREAMID_TYPE_NULL,
	STREAMID_TYPE_SMAC,
};

enum streamid_vlan_tagged {
	STREAMID_VLAN_RESERVED = 0,
	STREAMID_VLAN_TAGGED,
	STREAMID_VLAN_UNTAGGED,
	STREAMID_VLAN_ALL,
};

#define ENETC_PSFP_WILDCARD -1
#define HANDLE_OFFSET 100

enum forward_type {
	FILTER_ACTION_TYPE_PSFP = BIT(0),
	FILTER_ACTION_TYPE_ACL = BIT(1),
	FILTER_ACTION_TYPE_BOTH = GENMASK(1, 0),
};

/* This is for limit output type for input actions */
struct actions_fwd {
	u64 actions;
	u64 keys;	/* include the must needed keys */
	enum forward_type output;
};

struct psfp_streamfilter_counters {
	u64 matching_frames_count;
	u64 passing_frames_count;
	u64 not_passing_frames_count;
	u64 passing_sdu_count;
	u64 not_passing_sdu_count;
	u64 red_frames_count;
};

struct enetc_streamid {
	u32 index;
	union {
		u8 src_mac[6];
		u8 dst_mac[6];
	};
	u8 filtertype;
	u16 vid;
	u8 tagged;
	s32 handle;
};

struct enetc_psfp_filter {
	u32 index;
	s32 handle;
	s8 prio;
	u32 maxsdu;
	u32 gate_id;
	s32 meter_id;
	refcount_t refcount;
	struct hlist_node node;
};

struct enetc_psfp_gate {
	u32 index;
	s8 init_ipv;
	u64 basetime;
	u64 cycletime;
	u64 cycletimext;
	u32 num_entries;
	refcount_t refcount;
	struct hlist_node node;
	struct action_gate_entry entries[] __counted_by(num_entries);
};

/* Only enable the green color frame now
 * Will add eir and ebs color blind, couple flag etc when
 * policing action add more offloading parameters
 */
struct enetc_psfp_meter {
	u32 index;
	u32 cir;
	u32 cbs;
	refcount_t refcount;
	struct hlist_node node;
};

#define ENETC_PSFP_FLAGS_FMI BIT(0)

struct enetc_stream_filter {
	struct enetc_streamid sid;
	u32 sfi_index;
	u32 sgi_index;
	u32 flags;
	u32 fmi_index;
	struct flow_stats stats;
	struct hlist_node node;
};

struct enetc_psfp {
	unsigned long dev_bitmap;
	unsigned long *psfp_sfi_bitmap;
	struct hlist_head stream_list;
	struct hlist_head psfp_filter_list;
	struct hlist_head psfp_gate_list;
	struct hlist_head psfp_meter_list;
	spinlock_t psfp_lock; /* spinlock for the struct enetc_psfp r/w */
};

static struct actions_fwd enetc_act_fwd[] = {
	{
		BIT(FLOW_ACTION_GATE),
		BIT_ULL(FLOW_DISSECTOR_KEY_ETH_ADDRS),
		FILTER_ACTION_TYPE_PSFP
	},
	{
		BIT(FLOW_ACTION_POLICE) |
		BIT(FLOW_ACTION_GATE),
		BIT_ULL(FLOW_DISSECTOR_KEY_ETH_ADDRS),
		FILTER_ACTION_TYPE_PSFP
	},
	/* example for ACL actions */
	{
		BIT(FLOW_ACTION_DROP),
		0,
		FILTER_ACTION_TYPE_ACL
	}
};

static struct enetc_psfp epsfp = {
	.dev_bitmap = 0,
	.psfp_sfi_bitmap = NULL,
};

static LIST_HEAD(enetc_block_cb_list);

/* Stream Identity Entry Set Descriptor */
static int enetc_streamid_hw_set(struct enetc_ndev_priv *priv,
				 struct enetc_streamid *sid,
				 u8 enable)
{
	struct enetc_cbd cbd = {.cmd = 0};
	struct streamid_data *si_data;
	struct streamid_conf *si_conf;
	dma_addr_t dma;
	u16 data_size;
	void *tmp;
	int port;
	int err;

	port = enetc_pf_to_port(priv->si->pdev);
	if (port < 0)
		return -EINVAL;

	if (sid->index >= priv->psfp_cap.max_streamid)
		return -EINVAL;

	if (sid->filtertype != STREAMID_TYPE_NULL &&
	    sid->filtertype != STREAMID_TYPE_SMAC)
		return -EOPNOTSUPP;

	/* Disable operation before enable */
	cbd.index = cpu_to_le16((u16)sid->index);
	cbd.cls = BDCR_CMD_STREAM_IDENTIFY;
	cbd.status_flags = 0;

	data_size = sizeof(struct streamid_data);
	tmp = enetc_cbd_alloc_data_mem(priv->si, &cbd, data_size,
				       &dma, (void *)&si_data);
	if (!tmp)
		return -ENOMEM;

	eth_broadcast_addr(si_data->dmac);
	si_data->vid_vidm_tg = (ENETC_CBDR_SID_VID_MASK
			       + ((0x3 << 14) | ENETC_CBDR_SID_VIDM));

	si_conf = &cbd.sid_set;
	/* Only one port supported for one entry, set itself */
	si_conf->iports = cpu_to_le32(1 << port);
	si_conf->id_type = 1;
	si_conf->oui[2] = 0x0;
	si_conf->oui[1] = 0x80;
	si_conf->oui[0] = 0xC2;

	err = enetc_send_cmd(priv->si, &cbd);
	if (err)
		goto out;

	if (!enable)
		goto out;

	/* Enable the entry overwrite again incase space flushed by hardware */
	cbd.status_flags = 0;

	si_conf->en = 0x80;
	si_conf->stream_handle = cpu_to_le32(sid->handle);
	si_conf->iports = cpu_to_le32(1 << port);
	si_conf->id_type = sid->filtertype;
	si_conf->oui[2] = 0x0;
	si_conf->oui[1] = 0x80;
	si_conf->oui[0] = 0xC2;

	memset(si_data, 0, data_size);

	/* VIDM default to be 1.
	 * VID Match. If set (b1) then the VID must match, otherwise
	 * any VID is considered a match. VIDM setting is only used
	 * when TG is set to b01.
	 */
	if (si_conf->id_type == STREAMID_TYPE_NULL) {
		ether_addr_copy(si_data->dmac, sid->dst_mac);
		si_data->vid_vidm_tg = (sid->vid & ENETC_CBDR_SID_VID_MASK) +
				       ((((u16)(sid->tagged) & 0x3) << 14)
				       | ENETC_CBDR_SID_VIDM);
	} else if (si_conf->id_type == STREAMID_TYPE_SMAC) {
		ether_addr_copy(si_data->smac, sid->src_mac);
		si_data->vid_vidm_tg = (sid->vid & ENETC_CBDR_SID_VID_MASK) +
				       ((((u16)(sid->tagged) & 0x3) << 14)
				       | ENETC_CBDR_SID_VIDM);
	}

	err = enetc_send_cmd(priv->si, &cbd);
out:
	enetc_cbd_free_data_mem(priv->si, data_size, tmp, &dma);

	return err;
}

/* Stream Filter Instance Set Descriptor */
static int enetc_streamfilter_hw_set(struct enetc_ndev_priv *priv,
				     struct enetc_psfp_filter *sfi,
				     u8 enable)
{
	struct enetc_cbd cbd = {.cmd = 0};
	struct sfi_conf *sfi_config;
	int port;

	port = enetc_pf_to_port(priv->si->pdev);
	if (port < 0)
		return -EINVAL;

	cbd.index = cpu_to_le16(sfi->index);
	cbd.cls = BDCR_CMD_STREAM_FILTER;
	cbd.status_flags = 0x80;
	cbd.length = cpu_to_le16(1);

	sfi_config = &cbd.sfi_conf;
	if (!enable)
		goto exit;

	sfi_config->en = 0x80;

	if (sfi->handle >= 0) {
		sfi_config->stream_handle =
			cpu_to_le32(sfi->handle);
		sfi_config->sthm |= 0x80;
	}

	sfi_config->sg_inst_table_index = cpu_to_le16(sfi->gate_id);
	sfi_config->input_ports = cpu_to_le32(1 << port);

	/* The priority value which may be matched against the
	 * frame’s priority value to determine a match for this entry.
	 */
	if (sfi->prio >= 0)
		sfi_config->multi |= (sfi->prio & 0x7) | 0x8;

	/* Filter Type. Identifies the contents of the MSDU/FM_INST_INDEX
	 * field as being either an MSDU value or an index into the Flow
	 * Meter Instance table.
	 */
	if (sfi->maxsdu) {
		sfi_config->msdu =
		cpu_to_le16(sfi->maxsdu);
		sfi_config->multi |= 0x40;
	}

	if (sfi->meter_id >= 0) {
		sfi_config->fm_inst_table_index = cpu_to_le16(sfi->meter_id);
		sfi_config->multi |= 0x80;
	}

exit:
	return enetc_send_cmd(priv->si, &cbd);
}

static int enetc_streamcounter_hw_get(struct enetc_ndev_priv *priv,
				      u32 index,
				      struct psfp_streamfilter_counters *cnt)
{
	struct enetc_cbd cbd = { .cmd = 2 };
	struct sfi_counter_data *data_buf;
	dma_addr_t dma;
	u16 data_size;
	void *tmp;
	int err;

	cbd.index = cpu_to_le16((u16)index);
	cbd.cmd = 2;
	cbd.cls = BDCR_CMD_STREAM_FILTER;
	cbd.status_flags = 0;

	data_size = sizeof(struct sfi_counter_data);

	tmp = enetc_cbd_alloc_data_mem(priv->si, &cbd, data_size,
				       &dma, (void *)&data_buf);
	if (!tmp)
		return -ENOMEM;

	err = enetc_send_cmd(priv->si, &cbd);
	if (err)
		goto exit;

	cnt->matching_frames_count =
		((u64)le32_to_cpu(data_buf->matchh) << 32) +
		le32_to_cpu(data_buf->matchl);

	cnt->not_passing_sdu_count =
		((u64)le32_to_cpu(data_buf->msdu_droph) << 32) +
		le32_to_cpu(data_buf->msdu_dropl);

	cnt->passing_sdu_count =
		cnt->matching_frames_count - cnt->not_passing_sdu_count;

	cnt->not_passing_frames_count =
		((u64)le32_to_cpu(data_buf->stream_gate_droph) << 32) +
		le32_to_cpu(data_buf->stream_gate_dropl);

	cnt->passing_frames_count = cnt->matching_frames_count -
				    cnt->not_passing_sdu_count -
				    cnt->not_passing_frames_count;

	cnt->red_frames_count =
		((u64)le32_to_cpu(data_buf->flow_meter_droph) << 32) +
		le32_to_cpu(data_buf->flow_meter_dropl);

exit:
	enetc_cbd_free_data_mem(priv->si, data_size, tmp, &dma);

	return err;
}

static u64 get_ptp_now(struct enetc_hw *hw)
{
	u64 now_lo, now_hi, now;

	now_lo = enetc_rd(hw, ENETC_SICTR0);
	now_hi = enetc_rd(hw, ENETC_SICTR1);
	now = now_lo | now_hi << 32;

	return now;
}

static int get_start_ns(u64 now, u64 cycle, u64 *start)
{
	u64 n;

	if (!cycle)
		return -EFAULT;

	n = div64_u64(now, cycle);

	*start = (n + 1) * cycle;

	return 0;
}

/* Stream Gate Instance Set Descriptor */
static int enetc_streamgate_hw_set(struct enetc_ndev_priv *priv,
				   struct enetc_psfp_gate *sgi,
				   u8 enable)
{
	struct enetc_cbd cbd = { .cmd = 0 };
	struct sgi_table *sgi_config;
	struct sgcl_conf *sgcl_config;
	struct sgcl_data *sgcl_data;
	struct sgce *sgce;
	dma_addr_t dma;
	u16 data_size;
	int err, i;
	void *tmp;
	u64 now;

	cbd.index = cpu_to_le16(sgi->index);
	cbd.cmd = 0;
	cbd.cls = BDCR_CMD_STREAM_GCL;
	cbd.status_flags = 0x80;

	/* disable */
	if (!enable)
		return enetc_send_cmd(priv->si, &cbd);

	if (!sgi->num_entries)
		return 0;

	if (sgi->num_entries > priv->psfp_cap.max_psfp_gatelist ||
	    !sgi->cycletime)
		return -EINVAL;

	/* enable */
	sgi_config = &cbd.sgi_table;

	/* Keep open before gate list start */
	sgi_config->ocgtst = 0x80;

	sgi_config->oipv = (sgi->init_ipv < 0) ?
				0x0 : ((sgi->init_ipv & 0x7) | 0x8);

	sgi_config->en = 0x80;

	/* Basic config */
	err = enetc_send_cmd(priv->si, &cbd);
	if (err)
		return -EINVAL;

	memset(&cbd, 0, sizeof(cbd));

	cbd.index = cpu_to_le16(sgi->index);
	cbd.cmd = 1;
	cbd.cls = BDCR_CMD_STREAM_GCL;
	cbd.status_flags = 0;

	sgcl_config = &cbd.sgcl_conf;

	sgcl_config->acl_len = (sgi->num_entries - 1) & 0x3;

	data_size = struct_size(sgcl_data, sgcl, sgi->num_entries);
	tmp = enetc_cbd_alloc_data_mem(priv->si, &cbd, data_size,
				       &dma, (void *)&sgcl_data);
	if (!tmp)
		return -ENOMEM;

	sgce = &sgcl_data->sgcl[0];

	sgcl_config->agtst = 0x80;

	sgcl_data->ct = cpu_to_le32(sgi->cycletime);
	sgcl_data->cte = cpu_to_le32(sgi->cycletimext);

	if (sgi->init_ipv >= 0)
		sgcl_config->aipv = (sgi->init_ipv & 0x7) | 0x8;

	for (i = 0; i < sgi->num_entries; i++) {
		struct action_gate_entry *from = &sgi->entries[i];
		struct sgce *to = &sgce[i];

		if (from->gate_state)
			to->multi |= 0x10;

		if (from->ipv >= 0)
			to->multi |= ((from->ipv & 0x7) << 5) | 0x08;

		if (from->maxoctets >= 0) {
			to->multi |= 0x01;
			to->msdu[0] = from->maxoctets & 0xFF;
			to->msdu[1] = (from->maxoctets >> 8) & 0xFF;
			to->msdu[2] = (from->maxoctets >> 16) & 0xFF;
		}

		to->interval = cpu_to_le32(from->interval);
	}

	/* If basetime is less than now, calculate start time */
	now = get_ptp_now(&priv->si->hw);

	if (sgi->basetime < now) {
		u64 start;

		err = get_start_ns(now, sgi->cycletime, &start);
		if (err)
			goto exit;
		sgcl_data->btl = cpu_to_le32(lower_32_bits(start));
		sgcl_data->bth = cpu_to_le32(upper_32_bits(start));
	} else {
		u32 hi, lo;

		hi = upper_32_bits(sgi->basetime);
		lo = lower_32_bits(sgi->basetime);
		sgcl_data->bth = cpu_to_le32(hi);
		sgcl_data->btl = cpu_to_le32(lo);
	}

	err = enetc_send_cmd(priv->si, &cbd);

exit:
	enetc_cbd_free_data_mem(priv->si, data_size, tmp, &dma);
	return err;
}

static int enetc_flowmeter_hw_set(struct enetc_ndev_priv *priv,
				  struct enetc_psfp_meter *fmi,
				  u8 enable)
{
	struct enetc_cbd cbd = { .cmd = 0 };
	struct fmi_conf *fmi_config;
	u64 temp = 0;

	cbd.index = cpu_to_le16((u16)fmi->index);
	cbd.cls = BDCR_CMD_FLOW_METER;
	cbd.status_flags = 0x80;

	if (!enable)
		return enetc_send_cmd(priv->si, &cbd);

	fmi_config = &cbd.fmi_conf;
	fmi_config->en = 0x80;

	if (fmi->cir) {
		temp = (u64)8000 * fmi->cir;
		temp = div_u64(temp, 3725);
	}

	fmi_config->cir = cpu_to_le32((u32)temp);
	fmi_config->cbs = cpu_to_le32(fmi->cbs);

	/* Default for eir ebs disable */
	fmi_config->eir = 0;
	fmi_config->ebs = 0;

	/* Default:
	 * mark red disable
	 * drop on yellow disable
	 * color mode disable
	 * couple flag disable
	 */
	fmi_config->conf = 0;

	return enetc_send_cmd(priv->si, &cbd);
}

static struct enetc_stream_filter *enetc_get_stream_by_index(u32 index)
{
	struct enetc_stream_filter *f;

	hlist_for_each_entry(f, &epsfp.stream_list, node)
		if (f->sid.index == index)
			return f;

	return NULL;
}

static struct enetc_psfp_gate *enetc_get_gate_by_index(u32 index)
{
	struct enetc_psfp_gate *g;

	hlist_for_each_entry(g, &epsfp.psfp_gate_list, node)
		if (g->index == index)
			return g;

	return NULL;
}

static struct enetc_psfp_filter *enetc_get_filter_by_index(u32 index)
{
	struct enetc_psfp_filter *s;

	hlist_for_each_entry(s, &epsfp.psfp_filter_list, node)
		if (s->index == index)
			return s;

	return NULL;
}

static struct enetc_psfp_meter *enetc_get_meter_by_index(u32 index)
{
	struct enetc_psfp_meter *m;

	hlist_for_each_entry(m, &epsfp.psfp_meter_list, node)
		if (m->index == index)
			return m;

	return NULL;
}

static struct enetc_psfp_filter
	*enetc_psfp_check_sfi(struct enetc_psfp_filter *sfi)
{
	struct enetc_psfp_filter *s;

	hlist_for_each_entry(s, &epsfp.psfp_filter_list, node)
		if (s->gate_id == sfi->gate_id &&
		    s->prio == sfi->prio &&
		    s->maxsdu == sfi->maxsdu &&
		    s->meter_id == sfi->meter_id)
			return s;

	return NULL;
}

static int enetc_get_free_index(struct enetc_ndev_priv *priv)
{
	u32 max_size = priv->psfp_cap.max_psfp_filter;
	unsigned long index;

	index = find_first_zero_bit(epsfp.psfp_sfi_bitmap, max_size);
	if (index == max_size)
		return -1;

	return index;
}

static void stream_filter_unref(struct enetc_ndev_priv *priv, u32 index)
{
	struct enetc_psfp_filter *sfi;
	u8 z;

	sfi = enetc_get_filter_by_index(index);
	WARN_ON(!sfi);
	z = refcount_dec_and_test(&sfi->refcount);

	if (z) {
		enetc_streamfilter_hw_set(priv, sfi, false);
		hlist_del(&sfi->node);
		kfree(sfi);
		clear_bit(index, epsfp.psfp_sfi_bitmap);
	}
}

static void stream_gate_unref(struct enetc_ndev_priv *priv, u32 index)
{
	struct enetc_psfp_gate *sgi;
	u8 z;

	sgi = enetc_get_gate_by_index(index);
	WARN_ON(!sgi);
	z = refcount_dec_and_test(&sgi->refcount);
	if (z) {
		enetc_streamgate_hw_set(priv, sgi, false);
		hlist_del(&sgi->node);
		kfree(sgi);
	}
}

static void flow_meter_unref(struct enetc_ndev_priv *priv, u32 index)
{
	struct enetc_psfp_meter *fmi;
	u8 z;

	fmi = enetc_get_meter_by_index(index);
	WARN_ON(!fmi);
	z = refcount_dec_and_test(&fmi->refcount);
	if (z) {
		enetc_flowmeter_hw_set(priv, fmi, false);
		hlist_del(&fmi->node);
		kfree(fmi);
	}
}

static void remove_one_chain(struct enetc_ndev_priv *priv,
			     struct enetc_stream_filter *filter)
{
	if (filter->flags & ENETC_PSFP_FLAGS_FMI)
		flow_meter_unref(priv, filter->fmi_index);

	stream_gate_unref(priv, filter->sgi_index);
	stream_filter_unref(priv, filter->sfi_index);

	hlist_del(&filter->node);
	kfree(filter);
}

static int enetc_psfp_hw_set(struct enetc_ndev_priv *priv,
			     struct enetc_streamid *sid,
			     struct enetc_psfp_filter *sfi,
			     struct enetc_psfp_gate *sgi,
			     struct enetc_psfp_meter *fmi)
{
	int err;

	err = enetc_streamid_hw_set(priv, sid, true);
	if (err)
		return err;

	if (sfi) {
		err = enetc_streamfilter_hw_set(priv, sfi, true);
		if (err)
			goto revert_sid;
	}

	err = enetc_streamgate_hw_set(priv, sgi, true);
	if (err)
		goto revert_sfi;

	if (fmi) {
		err = enetc_flowmeter_hw_set(priv, fmi, true);
		if (err)
			goto revert_sgi;
	}

	return 0;

revert_sgi:
	enetc_streamgate_hw_set(priv, sgi, false);
revert_sfi:
	if (sfi)
		enetc_streamfilter_hw_set(priv, sfi, false);
revert_sid:
	enetc_streamid_hw_set(priv, sid, false);
	return err;
}

static struct actions_fwd *
enetc_check_flow_actions(u64 acts, unsigned long long inputkeys)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(enetc_act_fwd); i++)
		if (acts == enetc_act_fwd[i].actions &&
		    inputkeys & enetc_act_fwd[i].keys)
			return &enetc_act_fwd[i];

	return NULL;
}

static int enetc_psfp_policer_validate(const struct flow_action *action,
				       const struct flow_action_entry *act,
				       struct netlink_ext_ack *extack)
{
	if (act->police.exceed.act_id != FLOW_ACTION_DROP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when exceed action is not drop");
		return -EOPNOTSUPP;
	}

	if (act->police.notexceed.act_id != FLOW_ACTION_PIPE &&
	    act->police.notexceed.act_id != FLOW_ACTION_ACCEPT) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is not pipe or ok");
		return -EOPNOTSUPP;
	}

	if (act->police.notexceed.act_id == FLOW_ACTION_ACCEPT &&
	    !flow_action_is_last_entry(action, act)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is ok, but action is not last");
		return -EOPNOTSUPP;
	}

	if (act->police.peakrate_bytes_ps ||
	    act->police.avrate || act->police.overhead) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when peakrate/avrate/overhead is configured");
		return -EOPNOTSUPP;
	}

	if (act->police.rate_pkt_ps) {
		NL_SET_ERR_MSG_MOD(extack,
				   "QoS offload not support packets per second");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int enetc4_get_free_ist_entry_id(struct enetc_ndev_priv *priv)
{
	u32 max_size = priv->psfp_cap.ntmp.max_ist_entries;
	unsigned long index;

	index = find_first_zero_bit(priv->ist_bitmap, max_size);
	if (index == max_size)
		return -1;

	return index;
}

static int enetc4_get_free_isct_entry_id(struct enetc_ndev_priv *priv)
{
	u32 max_size = priv->psfp_cap.ntmp.max_isct_entries;
	unsigned long index;

	index = find_first_zero_bit(priv->isct_bitmap, max_size);
	if (index == max_size)
		return -1;

	return index;
}

static void enetc4_clear_sgclt_used_words(struct enetc_ndev_priv *priv,
					  u32 entry_id, u32 entry_size)
{
	u32 i;

	for (i = entry_id; i < entry_id + entry_size; i++)
		clear_bit(i, priv->sgclt_used_words);
}

static int enetc4_get_free_sgclt_entry_id(struct enetc_ndev_priv *priv,
					  u32 entry_size)
{
	u32 entry_id, next_eid, size, i;

	do {
		entry_id = find_first_zero_bit(priv->sgclt_used_words,
					       priv->psfp_cap.ntmp.sgcl_num_words);
		if (entry_id == priv->psfp_cap.ntmp.sgcl_num_words)
			return -1;

		next_eid = find_next_bit(priv->sgclt_used_words,
					 priv->psfp_cap.ntmp.sgcl_num_words,
					 entry_id + 1);
		size = next_eid - entry_id;
	} while (size < entry_size &&
		 next_eid != priv->psfp_cap.ntmp.sgcl_num_words);

	if (size < entry_size)
		return -1;

	for (i = entry_id; i < entry_id + entry_size; i++)
		set_bit(i, priv->sgclt_used_words);

	return entry_id;
}

static struct enetc_psfp_node *
enetc4_get_psfp_node_by_isit_entry(struct enetc_ndev_priv *priv,
				   struct ntmp_isit_cfg *cfg)
{
	struct enetc_psfp_node *psfp;

	hlist_for_each_entry(psfp, &priv->psfp_chain.isit_list, node)
		if (ether_addr_equal(cfg->mac, psfp->isit_cfg.mac) &&
		    cfg->key_type == psfp->isit_cfg.key_type &&
		    cfg->tagged == psfp->isit_cfg.tagged &&
		    cfg->vid == psfp->isit_cfg.vid)
			return psfp;

	return NULL;
}

static struct enetc_psfp_node *
enetc4_get_psfp_node_by_chain_index(struct enetc_ndev_priv *priv,
				    u32 chain_index)
{
	struct enetc_psfp_node *psfp;

	hlist_for_each_entry(psfp, &priv->psfp_chain.isit_list, node)
		if (psfp->chain_index == chain_index)
			return psfp;

	return NULL;
}

static struct ntmp_sgit_cfg *
enetc4_get_sgit_entry_by_entry_id(struct enetc_ndev_priv *priv,
				  u32 entry_id)
{
	struct ntmp_sgit_cfg *cfg;

	hlist_for_each_entry(cfg, &priv->psfp_chain.sgit_list, node)
		if (cfg->entry_id == entry_id)
			return cfg;

	return NULL;
}

static struct ntmp_rpt_cfg *
enetc4_get_rpt_entry_by_entry_id(struct enetc_ndev_priv *priv,
				 u32 entry_id)
{
	struct ntmp_rpt_cfg *cfg;

	hlist_for_each_entry(cfg, &priv->psfp_chain.rpt_list, node)
		if (cfg->entry_id == entry_id)
			return cfg;

	return NULL;
}

static int enetc4_delete_sgit_entry(struct enetc_ndev_priv *priv, u32 entry_id)
{
	struct ntmp_sgclt_info *sgcl_info;
	struct ntmp_sgit_info *sgi_info;
	struct ntmp_sgit_cfg sgi_cfg;
	struct netc_cbdr *cbdr;
	u32 entry_size;
	int err;

	cbdr = &priv->si->cbdr;
	sgi_info = kzalloc(sizeof(*sgi_info), GFP_KERNEL);
	if (!sgi_info)
		return -ENOMEM;

	/* Step1: Query the stream gate instance table entry to retrieve
	 * the entry id of the administrative gate control list and the
	 * opertational gate control list.
	 */
	err = ntmp_sgit_query_entry(cbdr, entry_id, sgi_info);
	if (err)
		goto free_sgi_info;

	/* Step2: Update the stream gate instance table entry to set
	 * the entry id of the administrative gate control list to NULL.
	 */
	memset(&sgi_cfg, 0, sizeof(sgi_cfg));
	sgi_cfg.entry_id = entry_id;
	sgi_cfg.admin_sgcl_eid = NTMP_NULL_ENTRY_ID;
	err = ntmp_sgit_add_or_update_entry(cbdr, &sgi_cfg);
	if (err)
		goto free_sgi_info;

	/* Step3: Delete the stream gate instance table entry. */
	err = ntmp_sgit_delete_entry(cbdr, entry_id);
	if (err)
		goto free_sgi_info;

	/* Step4: Delete the administrative gate control list
	 * and the operational gate control list.
	 */
	sgcl_info = kzalloc(sizeof(*sgcl_info), GFP_KERNEL);
	if (!sgcl_info) {
		err = -ENOMEM;
		goto free_sgi_info;
	}

	if (sgi_info->admin_sgcl_eid != NTMP_NULL_ENTRY_ID) {
		err = ntmp_sgclt_query_entry(cbdr, sgi_info->admin_sgcl_eid, sgcl_info);
		if (err)
			goto free_sgcl_info;

		/* entry_size equals to 1 + ROUNDUP(N / 2) where N is number of gates */
		entry_size = 1 + DIV_ROUND_UP(sgcl_info->list_len, 2);
		err = ntmp_sgclt_delete_entry(cbdr, sgi_info->admin_sgcl_eid);
		if (err)
			goto free_sgcl_info;

		enetc4_clear_sgclt_used_words(priv, sgi_info->admin_sgcl_eid, entry_size);
	}

	if (sgi_info->oper_sgcl_eid != NTMP_NULL_ENTRY_ID) {
		memset(sgcl_info, 0, sizeof(*sgcl_info));
		err = ntmp_sgclt_query_entry(cbdr, sgi_info->oper_sgcl_eid, sgcl_info);
		if (err)
			goto free_sgcl_info;

		entry_size = 1 + DIV_ROUND_UP(sgcl_info->list_len, 2);
		err = ntmp_sgclt_delete_entry(cbdr, sgi_info->oper_sgcl_eid);
		if (err)
			goto free_sgcl_info;

		enetc4_clear_sgclt_used_words(priv, sgi_info->oper_sgcl_eid, entry_size);
	}

free_sgcl_info:
	kfree(sgcl_info);

free_sgi_info:
	kfree(sgi_info);

	return err;
}

static void enetc4_decrease_sgit_entry_refcount(struct enetc_ndev_priv *priv,
						u32 entry_id)
{
	struct ntmp_sgit_cfg *cfg;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	cfg = enetc4_get_sgit_entry_by_entry_id(priv, entry_id);
	if (!cfg)
		return;

	if (refcount_dec_and_test(&cfg->refcount)) {
		enetc4_delete_sgit_entry(priv, entry_id);
		hlist_del(&cfg->node);
		kfree(cfg);
	}
}

static void enetc4_decrease_rpt_entry_refcount(struct enetc_ndev_priv *priv,
					       u32 entry_id)
{
	struct ntmp_rpt_cfg *cfg;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	cfg = enetc4_get_rpt_entry_by_entry_id(priv, entry_id);
	if (!cfg)
		return;

	if (refcount_dec_and_test(&cfg->refcount)) {
		ntmp_rpt_delete_entry(&priv->si->cbdr, entry_id);
		hlist_del(&cfg->node);
		kfree(cfg);
	}
}

static void enetc4_psfp_delete_chain(struct enetc_ndev_priv *priv,
				     struct enetc_psfp_node *psfp)
{
	struct enetc_si *si = priv->si;
	struct netc_cbdr *cbdr;

	cbdr = &si->cbdr;

	ntmp_isit_delete_entry(cbdr, psfp->isit_cfg.entry_id);
	if (psfp->isf_eid != NTMP_NULL_ENTRY_ID)
		ntmp_isft_delete_entry(cbdr, psfp->isf_eid);

	ntmp_ist_delete_entry(cbdr, psfp->isit_cfg.is_eid);
	clear_bit(psfp->isit_cfg.is_eid, priv->ist_bitmap);
	ntmp_isct_operate_entry(cbdr, psfp->isc_eid, NTMP_CMD_DELETE, NULL);
	clear_bit(psfp->isc_eid, priv->isct_bitmap);

	spin_lock(&priv->psfp_chain.psfp_lock);

	enetc4_decrease_sgit_entry_refcount(priv, psfp->sgi_eid);
	enetc4_decrease_rpt_entry_refcount(priv, psfp->rp_eid);
	hlist_del(&psfp->node);

	spin_unlock(&priv->psfp_chain.psfp_lock);
	kfree(psfp);
}

static int enetc4_psfp_set_related_table_entries(struct enetc_ndev_priv *priv,
						 struct enetc_psfp_cfg psfp)
{
	struct netc_cbdr *cbdr = &priv->si->cbdr;
	struct ntmp_sgit_cfg *sgi;
	int err;

	err = ntmp_isct_operate_entry(cbdr, psfp.isct_cfg->entry_id,
				      NTMP_CMD_ADD, NULL);
	if (err)
		return err;

	err = ntmp_sgclt_add_entry(cbdr, psfp.sgclt_cfg);
	if (err)
		goto delete_isct_entry;

	err = ntmp_sgit_add_or_update_entry(cbdr, psfp.sgit_cfg);
	if (err) {
		ntmp_sgclt_delete_entry(cbdr, psfp.sgclt_cfg->entry_id);
		goto delete_isct_entry;
	}

	if (psfp.rpt_cfg) {
		err = ntmp_rpt_add_or_update_entry(cbdr, psfp.rpt_cfg);
		if (err)
			goto delete_sgit_entry;
	}

	err = ntmp_ist_add_or_update_entry(cbdr, psfp.ist_cfg);
	if (err)
		goto delete_rpt_entry;

	if (psfp.isft_cfg) {
		err = ntmp_isft_add_or_update_entry(cbdr, psfp.isft_cfg, true);
		if (err)
			goto delete_ist_entry;
	}

	err = ntmp_isit_add_or_update_entry(cbdr, psfp.isit_cfg, true);
	if (err)
		goto delete_isft_entry;

	return 0;

delete_isft_entry:
	if (psfp.isft_cfg)
		ntmp_isft_delete_entry(cbdr, psfp.isft_cfg->entry_id);

delete_ist_entry:
	ntmp_ist_delete_entry(cbdr, psfp.ist_cfg->entry_id);

delete_rpt_entry:
	if (psfp.rpt_cfg)
		ntmp_rpt_delete_entry(cbdr, psfp.rpt_cfg->entry_id);

delete_sgit_entry:
	sgi = enetc4_get_sgit_entry_by_entry_id(priv, psfp.sgit_cfg->entry_id);
	/* If the SGI entry has already existed which indicates that the SGI entry
	 * has been referred by other table entries, so we can not delete it
	 */
	if (!sgi)
		enetc4_delete_sgit_entry(priv, psfp.sgit_cfg->entry_id);

delete_isct_entry:
	ntmp_isct_operate_entry(cbdr, psfp.isct_cfg->entry_id,
				NTMP_CMD_DELETE, NULL);

	return err;
}

static int enetc_psfp_parse_clsflower(struct enetc_ndev_priv *priv,
				      struct flow_cls_offload *f)
{
	struct flow_action_entry *entryg = NULL, *entryp = NULL;
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct netlink_ext_ack *extack = f->common.extack;
	struct enetc_stream_filter *filter, *old_filter;
	struct enetc_psfp_meter *fmi = NULL, *old_fmi;
	struct enetc_psfp_filter *sfi, *old_sfi;
	struct enetc_psfp_gate *sgi, *old_sgi;
	struct flow_action_entry *entry;
	struct action_gate_entry *e;
	u8 sfi_overwrite = 0;
	int entries_size;
	int i, err;

	if (f->common.chain_index >= priv->psfp_cap.max_streamid) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream identify resource!");
		return -ENOSPC;
	}

	flow_action_for_each(i, entry, &rule->action)
		if (entry->id == FLOW_ACTION_GATE)
			entryg = entry;
		else if (entry->id == FLOW_ACTION_POLICE)
			entryp = entry;

	/* Not support without gate action */
	if (!entryg)
		return -EINVAL;

	filter = kzalloc(sizeof(*filter), GFP_KERNEL);
	if (!filter)
		return -ENOMEM;

	filter->sid.index = f->common.chain_index;

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		struct flow_match_eth_addrs match;

		flow_rule_match_eth_addrs(rule, &match);

		if (!is_zero_ether_addr(match.mask->dst) &&
		    !is_zero_ether_addr(match.mask->src)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Cannot match on both source and destination MAC");
			err = -EINVAL;
			goto free_filter;
		}

		if (!is_zero_ether_addr(match.mask->dst)) {
			if (!is_broadcast_ether_addr(match.mask->dst)) {
				NL_SET_ERR_MSG_MOD(extack,
						   "Masked matching on destination MAC not supported");
				err = -EINVAL;
				goto free_filter;
			}
			ether_addr_copy(filter->sid.dst_mac, match.key->dst);
			filter->sid.filtertype = STREAMID_TYPE_NULL;
		}

		if (!is_zero_ether_addr(match.mask->src)) {
			if (!is_broadcast_ether_addr(match.mask->src)) {
				NL_SET_ERR_MSG_MOD(extack,
						   "Masked matching on source MAC not supported");
				err = -EINVAL;
				goto free_filter;
			}
			ether_addr_copy(filter->sid.src_mac, match.key->src);
			filter->sid.filtertype = STREAMID_TYPE_SMAC;
		}
	} else {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported, must include ETH_ADDRS");
		err = -EINVAL;
		goto free_filter;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_VLAN)) {
		struct flow_match_vlan match;

		flow_rule_match_vlan(rule, &match);
		if (match.mask->vlan_priority) {
			if (match.mask->vlan_priority !=
			    (VLAN_PRIO_MASK >> VLAN_PRIO_SHIFT)) {
				NL_SET_ERR_MSG_MOD(extack, "Only full mask is supported for VLAN priority");
				err = -EINVAL;
				goto free_filter;
			}
		}

		if (match.mask->vlan_id) {
			if (match.mask->vlan_id != VLAN_VID_MASK) {
				NL_SET_ERR_MSG_MOD(extack, "Only full mask is supported for VLAN id");
				err = -EINVAL;
				goto free_filter;
			}

			filter->sid.vid = match.key->vlan_id;
			if (!filter->sid.vid)
				filter->sid.tagged = STREAMID_VLAN_UNTAGGED;
			else
				filter->sid.tagged = STREAMID_VLAN_TAGGED;
		}
	} else {
		filter->sid.tagged = STREAMID_VLAN_ALL;
	}

	/* parsing gate action */
	if (entryg->hw_index >= priv->psfp_cap.max_psfp_gate) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream Gate resource!");
		err = -ENOSPC;
		goto free_filter;
	}

	if (entryg->gate.num_entries >= priv->psfp_cap.max_psfp_gatelist) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream Gate resource!");
		err = -ENOSPC;
		goto free_filter;
	}

	entries_size = struct_size(sgi, entries, entryg->gate.num_entries);
	sgi = kzalloc(entries_size, GFP_KERNEL);
	if (!sgi) {
		err = -ENOMEM;
		goto free_filter;
	}

	refcount_set(&sgi->refcount, 1);
	sgi->index = entryg->hw_index;
	sgi->init_ipv = entryg->gate.prio;
	sgi->basetime = entryg->gate.basetime;
	sgi->cycletime = entryg->gate.cycletime;
	sgi->num_entries = entryg->gate.num_entries;

	e = sgi->entries;
	for (i = 0; i < entryg->gate.num_entries; i++) {
		e[i].gate_state = entryg->gate.entries[i].gate_state;
		e[i].interval = entryg->gate.entries[i].interval;
		e[i].ipv = entryg->gate.entries[i].ipv;
		e[i].maxoctets = entryg->gate.entries[i].maxoctets;
	}

	filter->sgi_index = sgi->index;

	sfi = kzalloc(sizeof(*sfi), GFP_KERNEL);
	if (!sfi) {
		err = -ENOMEM;
		goto free_gate;
	}

	refcount_set(&sfi->refcount, 1);
	sfi->gate_id = sgi->index;
	sfi->meter_id = ENETC_PSFP_WILDCARD;

	/* Flow meter and max frame size */
	if (entryp) {
		err = enetc_psfp_policer_validate(&rule->action, entryp, extack);
		if (err)
			goto free_sfi;

		if (entryp->police.burst) {
			fmi = kzalloc(sizeof(*fmi), GFP_KERNEL);
			if (!fmi) {
				err = -ENOMEM;
				goto free_sfi;
			}
			refcount_set(&fmi->refcount, 1);
			fmi->cir = entryp->police.rate_bytes_ps;
			fmi->cbs = entryp->police.burst;
			fmi->index = entryp->hw_index;
			filter->flags |= ENETC_PSFP_FLAGS_FMI;
			filter->fmi_index = fmi->index;
			sfi->meter_id = fmi->index;
		}

		if (entryp->police.mtu)
			sfi->maxsdu = entryp->police.mtu;
	}

	/* prio ref the filter prio */
	if (f->common.prio && f->common.prio <= BIT(3))
		sfi->prio = f->common.prio - 1;
	else
		sfi->prio = ENETC_PSFP_WILDCARD;

	old_sfi = enetc_psfp_check_sfi(sfi);
	if (!old_sfi) {
		int index;

		index = enetc_get_free_index(priv);
		if (index < 0) {
			NL_SET_ERR_MSG_MOD(extack, "No Stream Filter resource!");
			err = -ENOSPC;
			goto free_fmi;
		}

		sfi->index = index;
		sfi->handle = index + HANDLE_OFFSET;
		/* Update the stream filter handle also */
		filter->sid.handle = sfi->handle;
		filter->sfi_index = sfi->index;
		sfi_overwrite = 0;
	} else {
		filter->sfi_index = old_sfi->index;
		filter->sid.handle = old_sfi->handle;
		sfi_overwrite = 1;
	}

	err = enetc_psfp_hw_set(priv, &filter->sid,
				sfi_overwrite ? NULL : sfi, sgi, fmi);
	if (err)
		goto free_fmi;

	spin_lock(&epsfp.psfp_lock);
	if (filter->flags & ENETC_PSFP_FLAGS_FMI) {
		old_fmi = enetc_get_meter_by_index(filter->fmi_index);
		if (old_fmi) {
			fmi->refcount = old_fmi->refcount;
			refcount_set(&fmi->refcount,
				     refcount_read(&old_fmi->refcount) + 1);
			hlist_del(&old_fmi->node);
			kfree(old_fmi);
		}
		hlist_add_head(&fmi->node, &epsfp.psfp_meter_list);
	}

	/* Remove the old node if exist and update with a new node */
	old_sgi = enetc_get_gate_by_index(filter->sgi_index);
	if (old_sgi) {
		refcount_set(&sgi->refcount,
			     refcount_read(&old_sgi->refcount) + 1);
		hlist_del(&old_sgi->node);
		kfree(old_sgi);
	}

	hlist_add_head(&sgi->node, &epsfp.psfp_gate_list);

	if (!old_sfi) {
		hlist_add_head(&sfi->node, &epsfp.psfp_filter_list);
		set_bit(sfi->index, epsfp.psfp_sfi_bitmap);
	} else {
		kfree(sfi);
		refcount_inc(&old_sfi->refcount);
	}

	old_filter = enetc_get_stream_by_index(filter->sid.index);
	if (old_filter)
		remove_one_chain(priv, old_filter);

	filter->stats.lastused = jiffies;
	hlist_add_head(&filter->node, &epsfp.stream_list);

	spin_unlock(&epsfp.psfp_lock);

	return 0;

free_fmi:
	kfree(fmi);
free_sfi:
	kfree(sfi);
free_gate:
	kfree(sgi);
free_filter:
	kfree(filter);

	return err;
}

static int enetc4_psfp_parse_clsflower(struct enetc_ndev_priv *priv,
				       struct flow_cls_offload *f)
{
	struct flow_action_entry *gate_entry = NULL, *police_entry = NULL;
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct netlink_ext_ack *extack = f->common.extack;
	struct ntmp_rpt_cfg *rpt_cfg = NULL, *old_rpt_cfg;
	struct ntmp_sgit_cfg *sgit_cfg, *old_sgit_cfg;
	struct enetc_psfp_node *psfp, *old_psfp;
	struct ntmp_isft_cfg *isft_cfg = NULL;
	struct ntmp_sgclt_cfg *sgclt_cfg;
	struct flow_action_entry *entry;
	struct enetc_psfp_cfg psfp_cfg;
	struct ntmp_isct_cfg *isct_cfg;
	struct ntmp_ist_cfg *ist_cfg;
	u32 sgclt_entry_size;
	int i, err, entry_id;
	u64 max_cycle_time;
	int sgclt_cfg_size;
	int priority = -1;

	if (f->common.chain_index >= priv->psfp_cap.ntmp.max_isit_entries) {
		NL_SET_ERR_MSG_MOD(extack,
				   "No ingress stream identification resource!");
		return -ENOSPC;
	}

	/* Delete old PSFP node if the chain index already exists. */
	old_psfp = enetc4_get_psfp_node_by_chain_index(priv, f->common.chain_index);
	if (old_psfp)
		/* If the chain has already existed, we delete the chain first and
		 * then add the new chain.
		 */
		enetc4_psfp_delete_chain(priv, old_psfp);

	/* Find gate action entry and police action entry*/
	flow_action_for_each(i, entry, &rule->action)
		if (entry->id == FLOW_ACTION_GATE)
			gate_entry = entry;
		else if (entry->id == FLOW_ACTION_POLICE)
			police_entry = entry;

	/* Not support without gate action */
	if (!gate_entry)
		return -EINVAL;

	psfp = kzalloc(sizeof(*psfp), GFP_KERNEL);
	if (!psfp)
		return -ENOMEM;

	psfp->chain_index = f->common.chain_index;
	psfp->isf_eid = NTMP_NULL_ENTRY_ID;

	/* if key of rule is ethernet address */
	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		struct flow_match_eth_addrs match;

		/* get the ethernet address key and mask of rule */
		flow_rule_match_eth_addrs(rule, &match);

		/* Determine if give Ethernet address is all zeros. */
		if (!is_zero_ether_addr(match.mask->dst) &&
		    !is_zero_ether_addr(match.mask->src)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Cannot match on both source and destination MAC");
			err = -EINVAL;
			goto free_psfp;
		}

		if (!is_zero_ether_addr(match.mask->dst)) {
			/* Determine if the destination ethernet address mast is broadcast.
			 * Must be 0xffffff for ENETC.
			 */
			if (!is_broadcast_ether_addr(match.mask->dst)) {
				NL_SET_ERR_MSG_MOD(extack,
						   "Masked matching on destination MAC not supported");
				err = -EINVAL;
				goto free_psfp;
			}
			ether_addr_copy(psfp->isit_cfg.mac, match.key->dst);
			psfp->isit_cfg.key_type = NTMP_ISIT_KEY_TYPE1_DMAC_VLAN;
		}

		if (!is_zero_ether_addr(match.mask->src)) {
			/* Determine if the source ethernet address mast is broadcast.
			 * Must be 0xffffff for ENETC.
			 */
			if (!is_broadcast_ether_addr(match.mask->src)) {
				NL_SET_ERR_MSG_MOD(extack,
						   "Masked matching on source MAC not supported");
				err = -EINVAL;
				goto free_psfp;
			}
			ether_addr_copy(psfp->isit_cfg.mac, match.key->src);
			psfp->isit_cfg.key_type = NTMP_ISIT_KEY_TYPE0_SMAC_VLAN;
		}
	} else {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported, must include ETH_ADDRS");
		err = -EINVAL;
		goto free_psfp;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_VLAN)) {
		struct flow_match_vlan match;

		/* get the key and mask of vlan of the rule */
		flow_rule_match_vlan(rule, &match);

		/* the mask of vlan id must be full mask */
		if (match.mask->vlan_id) {
			if (match.mask->vlan_id != VLAN_VID_MASK) {
				NL_SET_ERR_MSG_MOD(extack,
						   "Only full mask is supported for VLAN id");
				err = -EINVAL;
				goto free_psfp;
			}

			psfp->isit_cfg.vid = match.key->vlan_id;
			psfp->isit_cfg.tagged = NTMP_ISIT_FRMAE_TAG;
		}

		if (match.mask->vlan_priority) {
			if (match.mask->vlan_priority != VLAN_PRIO_MASK >> VLAN_PRIO_SHIFT) {
				NL_SET_ERR_MSG_MOD(extack,
						   "Only full mask is supported for VLAN priority");
				err = -EINVAL;
				goto free_psfp;
			}

			priority = match.key->vlan_priority;
		}
	} else {
		psfp->isit_cfg.tagged = NTMP_ISIT_FRMAE_UNTAG;
	}

	/* Delete old PSFP node if the ISI entry already exists. */
	old_psfp = enetc4_get_psfp_node_by_isit_entry(priv, &psfp->isit_cfg);
	if (old_psfp)
		/* If the ISI entry has already existed, correspondingly, the psfp chain
		 * associated with it should be delete first, then add the new chain.
		 */
		enetc4_psfp_delete_chain(priv, old_psfp);

	ist_cfg = kzalloc(sizeof(*ist_cfg), GFP_KERNEL);
	if (!ist_cfg) {
		err = -ENOMEM;
		goto free_psfp;
	}

	entry_id = enetc4_get_free_ist_entry_id(priv);
	if (entry_id < 0) {
		NL_SET_ERR_MSG_MOD(extack, "No ingress stream resource!");
		err = -ENOSPC;
		goto free_ist_cfg;
	}
	set_bit(entry_id, priv->ist_bitmap);
	ist_cfg->entry_id = entry_id;
	psfp->isit_cfg.is_eid = ist_cfg->entry_id;
	ist_cfg->rp_eid = NTMP_NULL_ENTRY_ID;
	ist_cfg->sgi_eid = NTMP_NULL_ENTRY_ID;
	ist_cfg->isc_eid = NTMP_NULL_ENTRY_ID;
	ist_cfg->fa = NTMP_IST_FA_NO_SI_BITMAP;

	/* parsing gate action */
	if (gate_entry->hw_index >= priv->psfp_cap.ntmp.max_sgit_entries) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream Gate Instance resource!");
		err = -ENOSPC;
		goto clear_ist_bit;
	}

	max_cycle_time = gate_entry->gate.cycletime + gate_entry->gate.cycletimeext;
	if (max_cycle_time > NTMP_SGIT_MAX_CT_PLUS_CT_EXT) {
		NL_SET_ERR_MSG_MOD(extack,
				   "cycle time + cycle time extension > 0x3ffffff ns.");
		err = -EINVAL;
		goto clear_ist_bit;
	}

	sgit_cfg = kzalloc(sizeof(*sgit_cfg), GFP_KERNEL);
	if (!sgit_cfg) {
		err = -ENOMEM;
		goto clear_ist_bit;
	}
	refcount_set(&sgit_cfg->refcount, 1);
	sgit_cfg->entry_id = gate_entry->hw_index;
	sgit_cfg->init_ipv = gate_entry->gate.prio;
	sgit_cfg->admin_bt = gate_entry->gate.basetime;
	sgit_cfg->admin_sgcl_eid = NTMP_NULL_ENTRY_ID;	//initialize to NULL entry ID.
	sgit_cfg->admin_ct_ext = gate_entry->gate.cycletimeext;
	psfp->sgi_eid = sgit_cfg->entry_id;

	if (gate_entry->gate.num_entries > NTMP_SGCLT_MAX_GE_NUM) {
		NL_SET_ERR_MSG_MOD(extack, "Gate entries number exceeds maximum 256!");
		err = -ENOSPC;
		goto free_sgit_cfg;
	}

	sgclt_cfg_size = struct_size(sgclt_cfg, entries, gate_entry->gate.num_entries);
	sgclt_cfg = kzalloc(sgclt_cfg_size, GFP_KERNEL);
	if (!sgclt_cfg) {
		err = -ENOMEM;
		goto free_sgit_cfg;
	}

	sgclt_entry_size = 1 + DIV_ROUND_UP(gate_entry->gate.num_entries, 2);
	entry_id = enetc4_get_free_sgclt_entry_id(priv, sgclt_entry_size);
	if (entry_id < 0) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream Gate Control List resource!");
		err = -ENOSPC;
		goto free_sgclt_cfg;
	}

	sgclt_cfg->entry_id = entry_id;
	sgclt_cfg->ct = gate_entry->gate.cycletime;
	sgclt_cfg->num_gates = gate_entry->gate.num_entries;
	sgclt_cfg->init_ipv = gate_entry->gate.prio;

	memcpy(sgclt_cfg->entries, gate_entry->gate.entries,
	       sizeof(struct action_gate_entry) * gate_entry->gate.num_entries);

	sgit_cfg->admin_sgcl_eid = sgclt_cfg->entry_id;
	sgit_cfg->sgcl_ct = sgclt_cfg->ct;
	psfp->sgcl_eid = sgclt_cfg->entry_id;

	isct_cfg = kzalloc(sizeof(*isct_cfg), GFP_KERNEL);
	if (!isct_cfg) {
		err = -ENOMEM;
		goto free_sgclt_used_words;
	}

	entry_id = enetc4_get_free_isct_entry_id(priv);
	if (entry_id < 0) {
		NL_SET_ERR_MSG_MOD(extack, "No ingress stream count resource!");
		err = -ENOSPC;
		goto free_isct_cfg;
	}
	set_bit(entry_id, priv->isct_bitmap);
	isct_cfg->entry_id = entry_id;
	psfp->isc_eid = isct_cfg->entry_id;

	/* Determine if a ingress stream filter entry is required. */
	if (priority >= 0 && priority < BIT(3)) {
		isft_cfg = kzalloc(sizeof(*isft_cfg), GFP_KERNEL);
		if (!isft_cfg) {
			err = -ENOMEM;
			goto clear_isct_bit;
		}
		isft_cfg->is_eid = ist_cfg->entry_id;
		isft_cfg->isc_eid = isct_cfg->entry_id;
		isft_cfg->sgi_eid = sgit_cfg->entry_id;
		isft_cfg->rp_eid = NTMP_NULL_ENTRY_ID;
		isft_cfg->or_flags |= NTMP_ISFT_FLAG_OSGI;
		isft_cfg->priority = priority;
		/* Enable stream filter */
		ist_cfg->sfe = 1;
		ist_cfg->osgi = 0;
	} else {
		ist_cfg->osgi = 1;
		ist_cfg->isc_eid = isct_cfg->entry_id;
		ist_cfg->sgi_eid = sgit_cfg->entry_id;
	}

	if (police_entry) {
		err = enetc_psfp_policer_validate(&rule->action, police_entry, extack);
		if (err)
			goto free_isft_cfg;

		if (police_entry->hw_index > priv->psfp_cap.ntmp.max_rpt_entries) {
			NL_SET_ERR_MSG_MOD(extack, "No rate policer resource!");
			err = -ENOSPC;
			goto free_isft_cfg;
		}

		if (police_entry->police.burst) {
			u64 rate_bps;

			rpt_cfg = kzalloc(sizeof(*rpt_cfg), GFP_KERNEL);
			if (!rpt_cfg) {
				err = -ENOMEM;
				goto free_isft_cfg;
			}
			refcount_set(&rpt_cfg->refcount, 1);
			/* The unit of rate_bytes_ps is 1Bps, the uint of cir is 3.725bps,
			 * so convert it.
			 */
			rate_bps = police_entry->police.rate_bytes_ps * 8;
			rpt_cfg->cir = div_u64(rate_bps * 1000, 3725);
			rpt_cfg->cbs = police_entry->police.burst;
			rpt_cfg->entry_id = police_entry->hw_index;
			psfp->rp_eid = rpt_cfg->entry_id;
			if (isft_cfg) {
				isft_cfg->is_eid = ist_cfg->entry_id;
				isft_cfg->rp_eid = rpt_cfg->entry_id;
				isft_cfg->or_flags |= NTMP_ISFT_FLAG_ORP;
				ist_cfg->orp = 0;
			} else {
				ist_cfg->orp = 1;
				ist_cfg->rp_eid = rpt_cfg->entry_id;
			}
		}

		if (police_entry->police.mtu) {
			if (isft_cfg)
				isft_cfg->msdu = police_entry->police.mtu;
			else
				ist_cfg->msdu = police_entry->police.mtu;
		}
	} else {
		psfp->rp_eid = NTMP_NULL_ENTRY_ID;
	}

	psfp_cfg.ist_cfg = ist_cfg;
	psfp_cfg.rpt_cfg = rpt_cfg;
	psfp_cfg.isit_cfg = &psfp->isit_cfg;
	psfp_cfg.isft_cfg = isft_cfg;
	psfp_cfg.sgit_cfg = sgit_cfg;
	psfp_cfg.isct_cfg = isct_cfg;
	psfp_cfg.sgclt_cfg = sgclt_cfg;
	err = enetc4_psfp_set_related_table_entries(priv, psfp_cfg);
	if (err)
		goto free_rpt_cfg;

	spin_lock(&priv->psfp_chain.psfp_lock);

	old_sgit_cfg = enetc4_get_sgit_entry_by_entry_id(priv, sgit_cfg->entry_id);
	if (old_sgit_cfg) {
		refcount_set(&sgit_cfg->refcount,
			     refcount_read(&old_sgit_cfg->refcount) + 1);
		hlist_del(&old_sgit_cfg->node);
		kfree(old_sgit_cfg);
	}
	hlist_add_head(&sgit_cfg->node, &priv->psfp_chain.sgit_list);

	if (rpt_cfg) {
		old_rpt_cfg = enetc4_get_rpt_entry_by_entry_id(priv, rpt_cfg->entry_id);
		if (old_rpt_cfg) {
			refcount_set(&rpt_cfg->refcount,
				     refcount_read(&old_rpt_cfg->refcount) + 1);
			hlist_del(&old_rpt_cfg->node);
			kfree(old_rpt_cfg);
		}
		hlist_add_head(&rpt_cfg->node, &priv->psfp_chain.rpt_list);
	}

	psfp->stats.lastused = jiffies;
	if (isft_cfg)
		psfp->isf_eid = isft_cfg->entry_id;

	hlist_add_head(&psfp->node, &priv->psfp_chain.isit_list);

	spin_unlock(&priv->psfp_chain.psfp_lock);

	kfree(ist_cfg);
	kfree(isft_cfg);
	kfree(sgclt_cfg);
	kfree(isct_cfg);

	return 0;

free_rpt_cfg:
	kfree(rpt_cfg);
free_isft_cfg:
	kfree(isft_cfg);
clear_isct_bit:
	clear_bit(isct_cfg->entry_id, priv->isct_bitmap);
free_isct_cfg:
	kfree(isct_cfg);
free_sgclt_used_words:
	enetc4_clear_sgclt_used_words(priv, sgclt_cfg->entry_id,
				      sgclt_entry_size);
free_sgclt_cfg:
	kfree(sgclt_cfg);
free_sgit_cfg:
	kfree(sgit_cfg);
clear_ist_bit:
	clear_bit(ist_cfg->entry_id, priv->ist_bitmap);
free_ist_cfg:
	kfree(ist_cfg);
free_psfp:
	kfree(psfp);

	return err;
}

static int enetc_config_clsflower(struct enetc_ndev_priv *priv,
				  struct flow_cls_offload *cls_flower)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls_flower);
	struct netlink_ext_ack *extack = cls_flower->common.extack;
	struct flow_dissector *dissector = rule->match.dissector;
	struct flow_action *action = &rule->action;
	struct flow_action_entry *entry;
	struct actions_fwd *fwd;
	u64 actions = 0;
	int i, err;

	if (!flow_action_has_entries(action)) {
		NL_SET_ERR_MSG_MOD(extack, "At least one action is needed");
		return -EINVAL;
	}

	flow_action_for_each(i, entry, action)
		actions |= BIT(entry->id);

	fwd = enetc_check_flow_actions(actions, dissector->used_keys);
	if (!fwd) {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported filter type!");
		return -EOPNOTSUPP;
	}

	if (fwd->output & FILTER_ACTION_TYPE_PSFP) {
		if (is_enetc_rev1(priv->si))
			err = enetc_psfp_parse_clsflower(priv, cls_flower);
		else
			err = enetc4_psfp_parse_clsflower(priv, cls_flower);
		if (err) {
			NL_SET_ERR_MSG_MOD(extack, "Invalid PSFP inputs");
			return err;
		}
	} else {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported actions");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int enetc_psfp_destroy_clsflower(struct enetc_ndev_priv *priv,
					struct flow_cls_offload *f)
{
	struct enetc_stream_filter *filter;
	struct netlink_ext_ack *extack = f->common.extack;
	int err;

	if (f->common.chain_index >= priv->psfp_cap.max_streamid) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream identify resource!");
		return -ENOSPC;
	}

	filter = enetc_get_stream_by_index(f->common.chain_index);
	if (!filter)
		return -EINVAL;

	err = enetc_streamid_hw_set(priv, &filter->sid, false);
	if (err)
		return err;

	remove_one_chain(priv, filter);

	return 0;
}

static int enetc4_psfp_destroy_clsflower(struct enetc_ndev_priv *priv,
					 struct flow_cls_offload *f)
{
	struct netlink_ext_ack *extack = f->common.extack;
	struct enetc_psfp_node *psfp;

	if (f->common.chain_index >= priv->psfp_cap.ntmp.max_isit_entries) {
		NL_SET_ERR_MSG_MOD(extack,
				   "No ingress stream identification resource!");
		return -ENOSPC;
	}

	psfp = enetc4_get_psfp_node_by_chain_index(priv, f->common.chain_index);
	if (!psfp)
		return -EINVAL;

	enetc4_psfp_delete_chain(priv, psfp);

	return 0;
}

static int enetc_destroy_clsflower(struct enetc_ndev_priv *priv,
				   struct flow_cls_offload *f)
{
	if (is_enetc_rev1(priv->si))
		return enetc_psfp_destroy_clsflower(priv, f);
	else
		return enetc4_psfp_destroy_clsflower(priv, f);
}

static int enetc_psfp_get_stats(struct enetc_ndev_priv *priv,
				struct flow_cls_offload *f)
{
	struct psfp_streamfilter_counters counters = {};
	struct enetc_stream_filter *filter;
	struct flow_stats stats = {};
	int err;

	filter = enetc_get_stream_by_index(f->common.chain_index);
	if (!filter)
		return -EINVAL;

	err = enetc_streamcounter_hw_get(priv, filter->sfi_index, &counters);
	if (err)
		return -EINVAL;

	spin_lock(&epsfp.psfp_lock);
	stats.pkts = counters.matching_frames_count +
		     counters.not_passing_sdu_count -
		     filter->stats.pkts;
	stats.drops = counters.not_passing_frames_count +
		      counters.not_passing_sdu_count +
		      counters.red_frames_count -
		      filter->stats.drops;
	stats.lastused = filter->stats.lastused;
	filter->stats.pkts += stats.pkts;
	filter->stats.drops += stats.drops;
	spin_unlock(&epsfp.psfp_lock);

	flow_stats_update(&f->stats, 0x0, stats.pkts, stats.drops,
			  stats.lastused, FLOW_ACTION_HW_STATS_DELAYED);

	return 0;
}

static int enetc4_psfp_get_stats(struct enetc_ndev_priv *priv,
				 struct flow_cls_offload *f)
{
	struct ntmp_isct_info counters = {};
	struct enetc_psfp_node *psfp;
	struct flow_stats stats = {};
	int err;

	psfp = enetc4_get_psfp_node_by_chain_index(priv, f->common.chain_index);
	if (!psfp)
		return -EINVAL;

	err = ntmp_isct_operate_entry(&priv->si->cbdr, psfp->isc_eid,
				      NTMP_CMD_QUERY, &counters);
	if (err)
		return -EINVAL;

	spin_lock(&priv->psfp_chain.psfp_lock);
	stats.pkts = counters.rx_count - psfp->stats.pkts;
	stats.drops = counters.msdu_drop_count +
		      counters.sg_drop_count +
		      counters.policer_drop_count -
		      psfp->stats.drops;
	stats.lastused = psfp->stats.lastused;
	psfp->stats.pkts += stats.pkts;
	psfp->stats.drops += stats.drops;
	spin_unlock(&priv->psfp_chain.psfp_lock);

	flow_stats_update(&f->stats, 0x0, stats.pkts, stats.drops,
			  stats.lastused, FLOW_ACTION_HW_STATS_DELAYED);

	return 0;
}

static int enetc_setup_tc_cls_flower(struct enetc_ndev_priv *priv,
				     struct flow_cls_offload *cls_flower)
{
	switch (cls_flower->command) {
	case FLOW_CLS_REPLACE:
		return enetc_config_clsflower(priv, cls_flower);
	case FLOW_CLS_DESTROY:
		return enetc_destroy_clsflower(priv, cls_flower);
	case FLOW_CLS_STATS:
		if (is_enetc_rev1(priv->si))
			return enetc_psfp_get_stats(priv, cls_flower);
		else
			return enetc4_psfp_get_stats(priv, cls_flower);
	default:
		return -EOPNOTSUPP;
	}
}

static inline void clean_psfp_sfi_bitmap(void)
{
	bitmap_free(epsfp.psfp_sfi_bitmap);
	epsfp.psfp_sfi_bitmap = NULL;
}

static void clean_stream_list(void)
{
	struct enetc_stream_filter *s;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(s, tmp, &epsfp.stream_list, node) {
		hlist_del(&s->node);
		kfree(s);
	}
}

static void clean_sfi_list(void)
{
	struct enetc_psfp_filter *sfi;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(sfi, tmp, &epsfp.psfp_filter_list, node) {
		hlist_del(&sfi->node);
		kfree(sfi);
	}
}

static void clean_sgi_list(void)
{
	struct enetc_psfp_gate *sgi;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(sgi, tmp, &epsfp.psfp_gate_list, node) {
		hlist_del(&sgi->node);
		kfree(sgi);
	}
}

static void clean_psfp_all(void)
{
	/* Disable all list nodes and free all memory */
	clean_sfi_list();
	clean_sgi_list();
	clean_stream_list();
	epsfp.dev_bitmap = 0;
	clean_psfp_sfi_bitmap();
}

static int enetc_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				   void *cb_priv)
{
	struct net_device *ndev = cb_priv;

	if (!tc_can_offload(ndev))
		return -EOPNOTSUPP;

	switch (type) {
	case TC_SETUP_CLSFLOWER:
		return enetc_setup_tc_cls_flower(netdev_priv(ndev), type_data);
	default:
		return -EOPNOTSUPP;
	}
}

int enetc_set_psfp(struct net_device *ndev, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int err;

	if (en) {
		err = enetc_psfp_enable(priv);
		if (err)
			return err;

		priv->active_offloads |= ENETC_F_QCI;
		return 0;
	}

	err = enetc_psfp_disable(priv);
	if (err)
		return err;

	priv->active_offloads &= ~ENETC_F_QCI;

	return 0;
}

int enetc_psfp_init(struct enetc_ndev_priv *priv)
{
	if (epsfp.psfp_sfi_bitmap)
		return 0;

	epsfp.psfp_sfi_bitmap = bitmap_zalloc(priv->psfp_cap.max_psfp_filter,
					      GFP_KERNEL);
	if (!epsfp.psfp_sfi_bitmap)
		return -ENOMEM;

	spin_lock_init(&epsfp.psfp_lock);

	if (list_empty(&enetc_block_cb_list))
		epsfp.dev_bitmap = 0;

	return 0;
}

int enetc4_psfp_init(struct enetc_ndev_priv *priv)
{
	priv->sgclt_used_words = bitmap_zalloc(priv->psfp_cap.ntmp.sgcl_num_words,
					       GFP_KERNEL);
	if (!priv->sgclt_used_words)
		return -ENOMEM;

	priv->ist_bitmap = bitmap_zalloc(priv->psfp_cap.ntmp.max_ist_entries,
					 GFP_KERNEL);
	if (!priv->ist_bitmap)
		return -ENOMEM;

	priv->isct_bitmap = bitmap_zalloc(priv->psfp_cap.ntmp.max_isct_entries,
					  GFP_KERNEL);
	if (!priv->isct_bitmap)
		return -ENOMEM;

	spin_lock_init(&priv->psfp_chain.psfp_lock);

	return 0;
}

static void enetc_clean_isit_list(struct enetc_ndev_priv *priv)
{
	struct enetc_psfp_node *psfp;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(psfp, tmp, &priv->psfp_chain.isit_list, node) {
		hlist_del(&psfp->node);
		kfree(psfp);
	}
}

static void enetc_clean_sgit_list(struct enetc_ndev_priv *priv)
{
	struct ntmp_sgit_cfg *sgi;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(sgi, tmp, &priv->psfp_chain.sgit_list, node) {
		hlist_del(&sgi->node);
		kfree(sgi);
	}
}

static void enetc_clean_rpt_list(struct enetc_ndev_priv *priv)
{
	struct ntmp_rpt_cfg *rp;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(rp, tmp, &priv->psfp_chain.rpt_list, node) {
		hlist_del(&rp->node);
		kfree(rp);
	}
}

static void enetc4_clean_psfp_all(struct enetc_ndev_priv *priv)
{
	enetc_clean_isit_list(priv);
	enetc_clean_sgit_list(priv);
	enetc_clean_rpt_list(priv);

	bitmap_free(priv->sgclt_used_words);
	priv->sgclt_used_words = NULL;
	bitmap_free(priv->ist_bitmap);
	priv->ist_bitmap = NULL;
	bitmap_free(priv->isct_bitmap);
	priv->isct_bitmap = NULL;
}

int enetc4_psfp_clean(struct enetc_ndev_priv *priv)
{
	if (!list_empty(&enetc_block_cb_list))
		return -EBUSY;

	enetc4_clean_psfp_all(priv);

	return 0;
}

int enetc_psfp_clean(struct enetc_ndev_priv *priv)
{
	if (!list_empty(&enetc_block_cb_list))
		return -EBUSY;

	clean_psfp_all();

	return 0;
}

int enetc_setup_tc_psfp(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct flow_block_offload *f = type_data;
	int port, err;

	err = flow_block_cb_setup_simple(f, &enetc_block_cb_list,
					 enetc_setup_tc_block_cb,
					 ndev, ndev, true);
	if (err)
		return err;

	if (!is_enetc_rev1(priv->si))
		return 0;

	switch (f->command) {
	case FLOW_BLOCK_BIND:
		port = enetc_pf_to_port(priv->si->pdev);
		if (port < 0)
			return -EINVAL;

		set_bit(port, &epsfp.dev_bitmap);
		break;
	case FLOW_BLOCK_UNBIND:
		port = enetc_pf_to_port(priv->si->pdev);
		if (port < 0)
			return -EINVAL;

		clear_bit(port, &epsfp.dev_bitmap);
		if (!epsfp.dev_bitmap)
			clean_psfp_all();
		break;
	}

	return 0;
}

int enetc_qos_query_caps(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_query_caps_base *base = type_data;
	struct enetc_si *si = priv->si;

	switch (base->type) {
	case TC_SETUP_QDISC_MQPRIO: {
		struct tc_mqprio_caps *caps = base->caps;

		caps->validate_queue_counts = true;

		return 0;
	}
	case TC_SETUP_QDISC_TAPRIO: {
		struct tc_taprio_caps *caps = base->caps;

		if (si->hw_features & ENETC_SI_F_QBV)
			caps->supports_queue_max_sdu = true;

		return 0;
	}
	default:
		return -EOPNOTSUPP;
	}
}
