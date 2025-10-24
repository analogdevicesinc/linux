// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 AIROHA Inc
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 */

#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/rhashtable.h>
#include <net/ipv6.h>
#include <net/pkt_cls.h>

#include "airoha_regs.h"
#include "airoha_eth.h"

static DEFINE_MUTEX(flow_offload_mutex);
static DEFINE_SPINLOCK(ppe_lock);

static const struct rhashtable_params airoha_flow_table_params = {
	.head_offset = offsetof(struct airoha_flow_table_entry, node),
	.key_offset = offsetof(struct airoha_flow_table_entry, cookie),
	.key_len = sizeof(unsigned long),
	.automatic_shrinking = true,
};

static const struct rhashtable_params airoha_l2_flow_table_params = {
	.head_offset = offsetof(struct airoha_flow_table_entry, l2_node),
	.key_offset = offsetof(struct airoha_flow_table_entry, data.bridge),
	.key_len = 2 * ETH_ALEN,
	.automatic_shrinking = true,
};

static int airoha_ppe_get_num_stats_entries(struct airoha_ppe *ppe)
{
	if (!IS_ENABLED(CONFIG_NET_AIROHA_FLOW_STATS))
		return -EOPNOTSUPP;

	if (airoha_is_7583(ppe->eth))
		return -EOPNOTSUPP;

	return PPE_STATS_NUM_ENTRIES;
}

static int airoha_ppe_get_total_num_stats_entries(struct airoha_ppe *ppe)
{
	int num_stats = airoha_ppe_get_num_stats_entries(ppe);

	if (num_stats > 0) {
		struct airoha_eth *eth = ppe->eth;

		num_stats = num_stats * eth->soc->num_ppe;
	}

	return num_stats;
}

static u32 airoha_ppe_get_total_sram_num_entries(struct airoha_ppe *ppe)
{
	struct airoha_eth *eth = ppe->eth;

	return PPE_SRAM_NUM_ENTRIES * eth->soc->num_ppe;
}

u32 airoha_ppe_get_total_num_entries(struct airoha_ppe *ppe)
{
	u32 sram_num_entries = airoha_ppe_get_total_sram_num_entries(ppe);

	return sram_num_entries + PPE_DRAM_NUM_ENTRIES;
}

bool airoha_ppe_is_enabled(struct airoha_eth *eth, int index)
{
	if (index >= eth->soc->num_ppe)
		return false;

	return airoha_fe_rr(eth, REG_PPE_GLO_CFG(index)) & PPE_GLO_CFG_EN_MASK;
}

static u32 airoha_ppe_get_timestamp(struct airoha_ppe *ppe)
{
	u16 timestamp = airoha_fe_rr(ppe->eth, REG_FE_FOE_TS);

	return FIELD_GET(AIROHA_FOE_IB1_BIND_TIMESTAMP, timestamp);
}

static void airoha_ppe_hw_init(struct airoha_ppe *ppe)
{
	u32 sram_ppe_num_data_entries = PPE_SRAM_NUM_ENTRIES, sram_num_entries;
	u32 sram_tb_size, dram_num_entries;
	struct airoha_eth *eth = ppe->eth;
	int i, sram_num_stats_entries;

	sram_num_entries = airoha_ppe_get_total_sram_num_entries(ppe);
	sram_tb_size = sram_num_entries * sizeof(struct airoha_foe_entry);
	dram_num_entries = PPE_RAM_NUM_ENTRIES_SHIFT(PPE_DRAM_NUM_ENTRIES);

	sram_num_stats_entries = airoha_ppe_get_num_stats_entries(ppe);
	if (sram_num_stats_entries > 0)
		sram_ppe_num_data_entries -= sram_num_stats_entries;
	sram_ppe_num_data_entries =
		PPE_RAM_NUM_ENTRIES_SHIFT(sram_ppe_num_data_entries);

	for (i = 0; i < eth->soc->num_ppe; i++) {
		int p;

		airoha_fe_wr(eth, REG_PPE_TB_BASE(i),
			     ppe->foe_dma + sram_tb_size);

		airoha_fe_rmw(eth, REG_PPE_BND_AGE0(i),
			      PPE_BIND_AGE0_DELTA_NON_L4 |
			      PPE_BIND_AGE0_DELTA_UDP,
			      FIELD_PREP(PPE_BIND_AGE0_DELTA_NON_L4, 1) |
			      FIELD_PREP(PPE_BIND_AGE0_DELTA_UDP, 12));
		airoha_fe_rmw(eth, REG_PPE_BND_AGE1(i),
			      PPE_BIND_AGE1_DELTA_TCP_FIN |
			      PPE_BIND_AGE1_DELTA_TCP,
			      FIELD_PREP(PPE_BIND_AGE1_DELTA_TCP_FIN, 1) |
			      FIELD_PREP(PPE_BIND_AGE1_DELTA_TCP, 7));

		airoha_fe_rmw(eth, REG_PPE_TB_HASH_CFG(i),
			      PPE_SRAM_TABLE_EN_MASK |
			      PPE_SRAM_HASH1_EN_MASK |
			      PPE_DRAM_TABLE_EN_MASK |
			      PPE_SRAM_HASH0_MODE_MASK |
			      PPE_SRAM_HASH1_MODE_MASK |
			      PPE_DRAM_HASH0_MODE_MASK |
			      PPE_DRAM_HASH1_MODE_MASK,
			      FIELD_PREP(PPE_SRAM_TABLE_EN_MASK, 1) |
			      FIELD_PREP(PPE_SRAM_HASH1_EN_MASK, 1) |
			      FIELD_PREP(PPE_SRAM_HASH1_MODE_MASK, 1) |
			      FIELD_PREP(PPE_DRAM_HASH1_MODE_MASK, 3));

		airoha_fe_rmw(eth, REG_PPE_TB_CFG(i),
			      PPE_TB_CFG_SEARCH_MISS_MASK |
			      PPE_SRAM_TB_NUM_ENTRY_MASK |
			      PPE_DRAM_TB_NUM_ENTRY_MASK |
			      PPE_TB_CFG_KEEPALIVE_MASK |
			      PPE_TB_ENTRY_SIZE_MASK,
			      FIELD_PREP(PPE_TB_CFG_SEARCH_MISS_MASK, 3) |
			      FIELD_PREP(PPE_TB_ENTRY_SIZE_MASK, 0) |
			      FIELD_PREP(PPE_SRAM_TB_NUM_ENTRY_MASK,
					 sram_ppe_num_data_entries) |
			      FIELD_PREP(PPE_DRAM_TB_NUM_ENTRY_MASK,
					 dram_num_entries));

		airoha_fe_wr(eth, REG_PPE_HASH_SEED(i), PPE_HASH_SEED);

		for (p = 0; p < ARRAY_SIZE(eth->ports); p++)
			airoha_fe_rmw(eth, REG_PPE_MTU(i, p),
				      FP0_EGRESS_MTU_MASK |
				      FP1_EGRESS_MTU_MASK,
				      FIELD_PREP(FP0_EGRESS_MTU_MASK,
						 AIROHA_MAX_MTU) |
				      FIELD_PREP(FP1_EGRESS_MTU_MASK,
						 AIROHA_MAX_MTU));
	}
}

static void airoha_ppe_flow_mangle_eth(const struct flow_action_entry *act, void *eth)
{
	void *dest = eth + act->mangle.offset;
	const void *src = &act->mangle.val;

	if (act->mangle.offset > 8)
		return;

	if (act->mangle.mask == 0xffff) {
		src += 2;
		dest += 2;
	}

	memcpy(dest, src, act->mangle.mask ? 2 : 4);
}

static int airoha_ppe_flow_mangle_ports(const struct flow_action_entry *act,
					struct airoha_flow_data *data)
{
	u32 val = be32_to_cpu((__force __be32)act->mangle.val);

	switch (act->mangle.offset) {
	case 0:
		if ((__force __be32)act->mangle.mask == ~cpu_to_be32(0xffff))
			data->dst_port = cpu_to_be16(val);
		else
			data->src_port = cpu_to_be16(val >> 16);
		break;
	case 2:
		data->dst_port = cpu_to_be16(val);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int airoha_ppe_flow_mangle_ipv4(const struct flow_action_entry *act,
				       struct airoha_flow_data *data)
{
	__be32 *dest;

	switch (act->mangle.offset) {
	case offsetof(struct iphdr, saddr):
		dest = &data->v4.src_addr;
		break;
	case offsetof(struct iphdr, daddr):
		dest = &data->v4.dst_addr;
		break;
	default:
		return -EINVAL;
	}

	memcpy(dest, &act->mangle.val, sizeof(u32));

	return 0;
}

static int airoha_ppe_get_wdma_info(struct net_device *dev, const u8 *addr,
				    struct airoha_wdma_info *info)
{
	struct net_device_path_stack stack;
	struct net_device_path *path;
	int err;

	if (!dev)
		return -ENODEV;

	err = dev_fill_forward_path(dev, addr, &stack);
	if (err)
		return err;

	path = &stack.path[stack.num_paths - 1];
	if (path->type != DEV_PATH_MTK_WDMA)
		return -1;

	info->idx = path->mtk_wdma.wdma_idx;
	info->bss = path->mtk_wdma.bss;
	info->wcid = path->mtk_wdma.wcid;

	return 0;
}

static int airoha_get_dsa_port(struct net_device **dev)
{
#if IS_ENABLED(CONFIG_NET_DSA)
	struct dsa_port *dp = dsa_port_from_netdev(*dev);

	if (IS_ERR(dp))
		return -ENODEV;

	*dev = dsa_port_to_conduit(dp);
	return dp->index;
#else
	return -ENODEV;
#endif
}

static void airoha_ppe_foe_set_bridge_addrs(struct airoha_foe_bridge *br,
					    struct ethhdr *eh)
{
	br->dest_mac_hi = get_unaligned_be32(eh->h_dest);
	br->dest_mac_lo = get_unaligned_be16(eh->h_dest + 4);
	br->src_mac_hi = get_unaligned_be16(eh->h_source);
	br->src_mac_lo = get_unaligned_be32(eh->h_source + 2);
}

static int airoha_ppe_foe_entry_prepare(struct airoha_eth *eth,
					struct airoha_foe_entry *hwe,
					struct net_device *dev, int type,
					struct airoha_flow_data *data,
					int l4proto)
{
	u32 qdata = FIELD_PREP(AIROHA_FOE_SHAPER_ID, 0x7f), ports_pad, val;
	int wlan_etype = -EINVAL, dsa_port = airoha_get_dsa_port(&dev);
	struct airoha_foe_mac_info_common *l2;
	u8 smac_id = 0xf;

	memset(hwe, 0, sizeof(*hwe));

	val = FIELD_PREP(AIROHA_FOE_IB1_BIND_STATE, AIROHA_FOE_STATE_BIND) |
	      FIELD_PREP(AIROHA_FOE_IB1_BIND_PACKET_TYPE, type) |
	      FIELD_PREP(AIROHA_FOE_IB1_BIND_UDP, l4proto == IPPROTO_UDP) |
	      FIELD_PREP(AIROHA_FOE_IB1_BIND_VLAN_LAYER, data->vlan.num) |
	      FIELD_PREP(AIROHA_FOE_IB1_BIND_VPM, data->vlan.num) |
	      FIELD_PREP(AIROHA_FOE_IB1_BIND_PPPOE, data->pppoe.num) |
	      AIROHA_FOE_IB1_BIND_TTL;
	hwe->ib1 = val;

	val = FIELD_PREP(AIROHA_FOE_IB2_PORT_AG, 0x1f);
	if (dev) {
		struct airoha_wdma_info info = {};

		if (!airoha_ppe_get_wdma_info(dev, data->eth.h_dest, &info)) {
			val |= FIELD_PREP(AIROHA_FOE_IB2_NBQ, info.idx) |
			       FIELD_PREP(AIROHA_FOE_IB2_PSE_PORT,
					  FE_PSE_PORT_CDM4);
			qdata |= FIELD_PREP(AIROHA_FOE_ACTDP, info.bss);
			wlan_etype = FIELD_PREP(AIROHA_FOE_MAC_WDMA_BAND,
						info.idx) |
				     FIELD_PREP(AIROHA_FOE_MAC_WDMA_WCID,
						info.wcid);
		} else {
			struct airoha_gdm_port *port = netdev_priv(dev);
			u8 pse_port;

			if (!airoha_is_valid_gdm_port(eth, port))
				return -EINVAL;

			if (dsa_port >= 0)
				pse_port = port->id == 4 ? FE_PSE_PORT_GDM4
							 : port->id;
			else
				pse_port = 2; /* uplink relies on GDM2
					       * loopback
					       */

			val |= FIELD_PREP(AIROHA_FOE_IB2_PSE_PORT, pse_port) |
			       AIROHA_FOE_IB2_PSE_QOS;
			/* For downlink traffic consume SRAM memory for hw
			 * forwarding descriptors queue.
			 */
			if (airhoa_is_lan_gdm_port(port))
				val |= AIROHA_FOE_IB2_FAST_PATH;
			if (dsa_port >= 0)
				val |= FIELD_PREP(AIROHA_FOE_IB2_NBQ,
						  dsa_port);

			smac_id = port->id;
		}
	}

	if (is_multicast_ether_addr(data->eth.h_dest))
		val |= AIROHA_FOE_IB2_MULTICAST;

	ports_pad = 0xa5a5a500 | (l4proto & 0xff);
	if (type == PPE_PKT_TYPE_IPV4_ROUTE)
		hwe->ipv4.orig_tuple.ports = ports_pad;
	if (type == PPE_PKT_TYPE_IPV6_ROUTE_3T)
		hwe->ipv6.ports = ports_pad;

	if (type == PPE_PKT_TYPE_BRIDGE) {
		airoha_ppe_foe_set_bridge_addrs(&hwe->bridge, &data->eth);
		hwe->bridge.data = qdata;
		hwe->bridge.ib2 = val;
		l2 = &hwe->bridge.l2.common;
	} else if (type >= PPE_PKT_TYPE_IPV6_ROUTE_3T) {
		hwe->ipv6.data = qdata;
		hwe->ipv6.ib2 = val;
		l2 = &hwe->ipv6.l2;
		l2->etype = ETH_P_IPV6;
	} else {
		hwe->ipv4.data = qdata;
		hwe->ipv4.ib2 = val;
		l2 = &hwe->ipv4.l2.common;
		l2->etype = ETH_P_IP;
	}

	l2->dest_mac_hi = get_unaligned_be32(data->eth.h_dest);
	l2->dest_mac_lo = get_unaligned_be16(data->eth.h_dest + 4);
	if (type <= PPE_PKT_TYPE_IPV4_DSLITE) {
		struct airoha_foe_mac_info *mac_info;

		l2->src_mac_hi = get_unaligned_be32(data->eth.h_source);
		hwe->ipv4.l2.src_mac_lo =
			get_unaligned_be16(data->eth.h_source + 4);

		mac_info = (struct airoha_foe_mac_info *)l2;
		mac_info->pppoe_id = data->pppoe.sid;
	} else {
		l2->src_mac_hi = FIELD_PREP(AIROHA_FOE_MAC_SMAC_ID, smac_id) |
				 FIELD_PREP(AIROHA_FOE_MAC_PPPOE_ID,
					    data->pppoe.sid);
	}

	if (data->vlan.num) {
		l2->vlan1 = data->vlan.hdr[0].id;
		if (data->vlan.num == 2)
			l2->vlan2 = data->vlan.hdr[1].id;
	}

	if (wlan_etype >= 0) {
		l2->etype = wlan_etype;
	} else if (dsa_port >= 0) {
		l2->etype = BIT(dsa_port);
		l2->etype |= !data->vlan.num ? BIT(15) : 0;
	} else if (data->pppoe.num) {
		l2->etype = ETH_P_PPP_SES;
	}

	return 0;
}

static int airoha_ppe_foe_entry_set_ipv4_tuple(struct airoha_foe_entry *hwe,
					       struct airoha_flow_data *data,
					       bool egress)
{
	int type = FIELD_GET(AIROHA_FOE_IB1_BIND_PACKET_TYPE, hwe->ib1);
	struct airoha_foe_ipv4_tuple *t;

	switch (type) {
	case PPE_PKT_TYPE_IPV4_HNAPT:
		if (egress) {
			t = &hwe->ipv4.new_tuple;
			break;
		}
		fallthrough;
	case PPE_PKT_TYPE_IPV4_DSLITE:
	case PPE_PKT_TYPE_IPV4_ROUTE:
		t = &hwe->ipv4.orig_tuple;
		break;
	default:
		WARN_ON_ONCE(1);
		return -EINVAL;
	}

	t->src_ip = be32_to_cpu(data->v4.src_addr);
	t->dest_ip = be32_to_cpu(data->v4.dst_addr);

	if (type != PPE_PKT_TYPE_IPV4_ROUTE) {
		t->src_port = be16_to_cpu(data->src_port);
		t->dest_port = be16_to_cpu(data->dst_port);
	}

	return 0;
}

static int airoha_ppe_foe_entry_set_ipv6_tuple(struct airoha_foe_entry *hwe,
					       struct airoha_flow_data *data)

{
	int type = FIELD_GET(AIROHA_FOE_IB1_BIND_PACKET_TYPE, hwe->ib1);
	u32 *src, *dest;

	switch (type) {
	case PPE_PKT_TYPE_IPV6_ROUTE_5T:
	case PPE_PKT_TYPE_IPV6_6RD:
		hwe->ipv6.src_port = be16_to_cpu(data->src_port);
		hwe->ipv6.dest_port = be16_to_cpu(data->dst_port);
		fallthrough;
	case PPE_PKT_TYPE_IPV6_ROUTE_3T:
		src = hwe->ipv6.src_ip;
		dest = hwe->ipv6.dest_ip;
		break;
	default:
		WARN_ON_ONCE(1);
		return -EINVAL;
	}

	ipv6_addr_be32_to_cpu(src, data->v6.src_addr.s6_addr32);
	ipv6_addr_be32_to_cpu(dest, data->v6.dst_addr.s6_addr32);

	return 0;
}

static u32 airoha_ppe_foe_get_entry_hash(struct airoha_ppe *ppe,
					 struct airoha_foe_entry *hwe)
{
	int type = FIELD_GET(AIROHA_FOE_IB1_BIND_PACKET_TYPE, hwe->ib1);
	u32 ppe_hash_mask = airoha_ppe_get_total_num_entries(ppe) - 1;
	u32 hash, hv1, hv2, hv3;

	switch (type) {
	case PPE_PKT_TYPE_IPV4_ROUTE:
	case PPE_PKT_TYPE_IPV4_HNAPT:
		hv1 = hwe->ipv4.orig_tuple.ports;
		hv2 = hwe->ipv4.orig_tuple.dest_ip;
		hv3 = hwe->ipv4.orig_tuple.src_ip;
		break;
	case PPE_PKT_TYPE_IPV6_ROUTE_3T:
	case PPE_PKT_TYPE_IPV6_ROUTE_5T:
		hv1 = hwe->ipv6.src_ip[3] ^ hwe->ipv6.dest_ip[3];
		hv1 ^= hwe->ipv6.ports;

		hv2 = hwe->ipv6.src_ip[2] ^ hwe->ipv6.dest_ip[2];
		hv2 ^= hwe->ipv6.dest_ip[0];

		hv3 = hwe->ipv6.src_ip[1] ^ hwe->ipv6.dest_ip[1];
		hv3 ^= hwe->ipv6.src_ip[0];
		break;
	case PPE_PKT_TYPE_BRIDGE: {
		struct airoha_foe_mac_info *l2 = &hwe->bridge.l2;

		hv1 = l2->common.src_mac_hi & 0xffff;
		hv1 = hv1 << 16 | l2->src_mac_lo;

		hv2 = l2->common.dest_mac_lo;
		hv2 = hv2 << 16;
		hv2 = hv2 | ((l2->common.src_mac_hi & 0xffff0000) >> 16);

		hv3 = l2->common.dest_mac_hi;
		break;
	}
	case PPE_PKT_TYPE_IPV4_DSLITE:
	case PPE_PKT_TYPE_IPV6_6RD:
	default:
		WARN_ON_ONCE(1);
		return ppe_hash_mask;
	}

	hash = (hv1 & hv2) | ((~hv1) & hv3);
	hash = (hash >> 24) | ((hash & 0xffffff) << 8);
	hash ^= hv1 ^ hv2 ^ hv3;
	hash ^= hash >> 16;
	hash &= ppe_hash_mask;

	return hash;
}

static int airoha_ppe_foe_get_flow_stats_index(struct airoha_ppe *ppe,
					       u32 hash, u32 *index)
{
	int ppe_num_stats_entries;

	ppe_num_stats_entries = airoha_ppe_get_total_num_stats_entries(ppe);
	if (ppe_num_stats_entries < 0)
		return ppe_num_stats_entries;

	*index = hash >= ppe_num_stats_entries ? hash - PPE_STATS_NUM_ENTRIES
					       : hash;

	return 0;
}

static void airoha_ppe_foe_flow_stat_entry_reset(struct airoha_ppe *ppe,
						 struct airoha_npu *npu,
						 int index)
{
	memset_io(&npu->stats[index], 0, sizeof(*npu->stats));
	memset(&ppe->foe_stats[index], 0, sizeof(*ppe->foe_stats));
}

static void airoha_ppe_foe_flow_stats_reset(struct airoha_ppe *ppe,
					    struct airoha_npu *npu)
{
	int i, ppe_num_stats_entries;

	ppe_num_stats_entries = airoha_ppe_get_total_num_stats_entries(ppe);
	if (ppe_num_stats_entries < 0)
		return;

	for (i = 0; i < ppe_num_stats_entries; i++)
		airoha_ppe_foe_flow_stat_entry_reset(ppe, npu, i);
}

static void airoha_ppe_foe_flow_stats_update(struct airoha_ppe *ppe,
					     struct airoha_npu *npu,
					     struct airoha_foe_entry *hwe,
					     u32 hash)
{
	int type = FIELD_GET(AIROHA_FOE_IB1_BIND_PACKET_TYPE, hwe->ib1);
	u32 index, pse_port, val, *data, *ib2, *meter;
	int ppe_num_stats_entries;
	u8 nbq;

	ppe_num_stats_entries = airoha_ppe_get_total_num_stats_entries(ppe);
	if (ppe_num_stats_entries < 0)
		return;

	if (airoha_ppe_foe_get_flow_stats_index(ppe, hash, &index))
		return;

	if (index >= ppe_num_stats_entries)
		return;

	if (type == PPE_PKT_TYPE_BRIDGE) {
		data = &hwe->bridge.data;
		ib2 = &hwe->bridge.ib2;
		meter = &hwe->bridge.l2.meter;
	} else if (type >= PPE_PKT_TYPE_IPV6_ROUTE_3T) {
		data = &hwe->ipv6.data;
		ib2 = &hwe->ipv6.ib2;
		meter = &hwe->ipv6.meter;
	} else {
		data = &hwe->ipv4.data;
		ib2 = &hwe->ipv4.ib2;
		meter = &hwe->ipv4.l2.meter;
	}

	pse_port = FIELD_GET(AIROHA_FOE_IB2_PSE_PORT, *ib2);
	if (pse_port == FE_PSE_PORT_CDM4)
		return;

	airoha_ppe_foe_flow_stat_entry_reset(ppe, npu, index);

	val = FIELD_GET(AIROHA_FOE_CHANNEL | AIROHA_FOE_QID, *data);
	*data = (*data & ~AIROHA_FOE_ACTDP) |
		FIELD_PREP(AIROHA_FOE_ACTDP, val);

	val = *ib2 & (AIROHA_FOE_IB2_NBQ | AIROHA_FOE_IB2_PSE_PORT |
		      AIROHA_FOE_IB2_PSE_QOS | AIROHA_FOE_IB2_FAST_PATH);
	*meter |= FIELD_PREP(AIROHA_FOE_TUNNEL_MTU, val);

	nbq = pse_port == 1 ? 6 : 5;
	*ib2 &= ~(AIROHA_FOE_IB2_NBQ | AIROHA_FOE_IB2_PSE_PORT |
		  AIROHA_FOE_IB2_PSE_QOS);
	*ib2 |= FIELD_PREP(AIROHA_FOE_IB2_PSE_PORT, 6) |
		FIELD_PREP(AIROHA_FOE_IB2_NBQ, nbq);
}

static struct airoha_foe_entry *
airoha_ppe_foe_get_entry_locked(struct airoha_ppe *ppe, u32 hash)
{
	u32 sram_num_entries = airoha_ppe_get_total_sram_num_entries(ppe);

	lockdep_assert_held(&ppe_lock);

	if (hash < sram_num_entries) {
		u32 *hwe = ppe->foe + hash * sizeof(struct airoha_foe_entry);
		bool ppe2 = hash >= PPE_SRAM_NUM_ENTRIES;
		struct airoha_eth *eth = ppe->eth;
		u32 val;
		int i;

		airoha_fe_wr(ppe->eth, REG_PPE_RAM_CTRL(ppe2),
			     FIELD_PREP(PPE_SRAM_CTRL_ENTRY_MASK, hash) |
			     PPE_SRAM_CTRL_REQ_MASK);
		if (read_poll_timeout_atomic(airoha_fe_rr, val,
					     val & PPE_SRAM_CTRL_ACK_MASK,
					     10, 100, false, eth,
					     REG_PPE_RAM_CTRL(ppe2)))
			return NULL;

		for (i = 0; i < sizeof(struct airoha_foe_entry) / sizeof(*hwe);
		     i++)
			hwe[i] = airoha_fe_rr(eth,
					      REG_PPE_RAM_ENTRY(ppe2, i));
	}

	return ppe->foe + hash * sizeof(struct airoha_foe_entry);
}

struct airoha_foe_entry *airoha_ppe_foe_get_entry(struct airoha_ppe *ppe,
						  u32 hash)
{
	struct airoha_foe_entry *hwe;

	spin_lock_bh(&ppe_lock);
	hwe = airoha_ppe_foe_get_entry_locked(ppe, hash);
	spin_unlock_bh(&ppe_lock);

	return hwe;
}

static bool airoha_ppe_foe_compare_entry(struct airoha_flow_table_entry *e,
					 struct airoha_foe_entry *hwe)
{
	int type = FIELD_GET(AIROHA_FOE_IB1_BIND_PACKET_TYPE, e->data.ib1);
	int len;

	if ((hwe->ib1 ^ e->data.ib1) & AIROHA_FOE_IB1_BIND_UDP)
		return false;

	if (type > PPE_PKT_TYPE_IPV4_DSLITE)
		len = offsetof(struct airoha_foe_entry, ipv6.data);
	else
		len = offsetof(struct airoha_foe_entry, ipv4.ib2);

	return !memcmp(&e->data.d, &hwe->d, len - sizeof(hwe->ib1));
}

static int airoha_ppe_foe_commit_sram_entry(struct airoha_ppe *ppe, u32 hash)
{
	struct airoha_foe_entry *hwe = ppe->foe + hash * sizeof(*hwe);
	bool ppe2 = hash >= PPE_SRAM_NUM_ENTRIES;
	u32 *ptr = (u32 *)hwe, val;
	int i;

	for (i = 0; i < sizeof(*hwe) / sizeof(*ptr); i++)
		airoha_fe_wr(ppe->eth, REG_PPE_RAM_ENTRY(ppe2, i), ptr[i]);

	wmb();
	airoha_fe_wr(ppe->eth, REG_PPE_RAM_CTRL(ppe2),
		     FIELD_PREP(PPE_SRAM_CTRL_ENTRY_MASK, hash) |
		     PPE_SRAM_CTRL_WR_MASK | PPE_SRAM_CTRL_REQ_MASK);

	return read_poll_timeout_atomic(airoha_fe_rr, val,
					val & PPE_SRAM_CTRL_ACK_MASK,
					10, 100, false, ppe->eth,
					REG_PPE_RAM_CTRL(ppe2));
}

static int airoha_ppe_foe_commit_entry(struct airoha_ppe *ppe,
				       struct airoha_foe_entry *e,
				       u32 hash, bool rx_wlan)
{
	u32 sram_num_entries = airoha_ppe_get_total_sram_num_entries(ppe);
	struct airoha_foe_entry *hwe = ppe->foe + hash * sizeof(*hwe);
	u32 ts = airoha_ppe_get_timestamp(ppe);
	struct airoha_eth *eth = ppe->eth;
	struct airoha_npu *npu;
	int err = 0;

	memcpy(&hwe->d, &e->d, sizeof(*hwe) - sizeof(hwe->ib1));
	wmb();

	e->ib1 &= ~AIROHA_FOE_IB1_BIND_TIMESTAMP;
	e->ib1 |= FIELD_PREP(AIROHA_FOE_IB1_BIND_TIMESTAMP, ts);
	hwe->ib1 = e->ib1;

	rcu_read_lock();

	npu = rcu_dereference(eth->npu);
	if (!npu) {
		err = -ENODEV;
		goto unlock;
	}

	if (!rx_wlan)
		airoha_ppe_foe_flow_stats_update(ppe, npu, hwe, hash);

	if (hash < sram_num_entries)
		err = airoha_ppe_foe_commit_sram_entry(ppe, hash);
unlock:
	rcu_read_unlock();

	return err;
}

static void airoha_ppe_foe_remove_flow(struct airoha_ppe *ppe,
				       struct airoha_flow_table_entry *e)
{
	lockdep_assert_held(&ppe_lock);

	hlist_del_init(&e->list);
	if (e->hash != 0xffff) {
		e->data.ib1 &= ~AIROHA_FOE_IB1_BIND_STATE;
		e->data.ib1 |= FIELD_PREP(AIROHA_FOE_IB1_BIND_STATE,
					  AIROHA_FOE_STATE_INVALID);
		airoha_ppe_foe_commit_entry(ppe, &e->data, e->hash, false);
		e->hash = 0xffff;
	}
	if (e->type == FLOW_TYPE_L2_SUBFLOW) {
		hlist_del_init(&e->l2_subflow_node);
		kfree(e);
	}
}

static void airoha_ppe_foe_remove_l2_flow(struct airoha_ppe *ppe,
					  struct airoha_flow_table_entry *e)
{
	struct hlist_head *head = &e->l2_flows;
	struct hlist_node *n;

	lockdep_assert_held(&ppe_lock);

	rhashtable_remove_fast(&ppe->l2_flows, &e->l2_node,
			       airoha_l2_flow_table_params);
	hlist_for_each_entry_safe(e, n, head, l2_subflow_node)
		airoha_ppe_foe_remove_flow(ppe, e);
}

static void airoha_ppe_foe_flow_remove_entry(struct airoha_ppe *ppe,
					     struct airoha_flow_table_entry *e)
{
	spin_lock_bh(&ppe_lock);

	if (e->type == FLOW_TYPE_L2)
		airoha_ppe_foe_remove_l2_flow(ppe, e);
	else
		airoha_ppe_foe_remove_flow(ppe, e);

	spin_unlock_bh(&ppe_lock);
}

static int
airoha_ppe_foe_commit_subflow_entry(struct airoha_ppe *ppe,
				    struct airoha_flow_table_entry *e,
				    u32 hash, bool rx_wlan)
{
	u32 mask = AIROHA_FOE_IB1_BIND_PACKET_TYPE | AIROHA_FOE_IB1_BIND_UDP;
	struct airoha_foe_entry *hwe_p, hwe;
	struct airoha_flow_table_entry *f;
	int type;

	hwe_p = airoha_ppe_foe_get_entry_locked(ppe, hash);
	if (!hwe_p)
		return -EINVAL;

	f = kzalloc(sizeof(*f), GFP_ATOMIC);
	if (!f)
		return -ENOMEM;

	hlist_add_head(&f->l2_subflow_node, &e->l2_flows);
	f->type = FLOW_TYPE_L2_SUBFLOW;
	f->hash = hash;

	memcpy(&hwe, hwe_p, sizeof(*hwe_p));
	hwe.ib1 = (hwe.ib1 & mask) | (e->data.ib1 & ~mask);

	type = FIELD_GET(AIROHA_FOE_IB1_BIND_PACKET_TYPE, hwe.ib1);
	if (type >= PPE_PKT_TYPE_IPV6_ROUTE_3T) {
		memcpy(&hwe.ipv6.l2, &e->data.bridge.l2, sizeof(hwe.ipv6.l2));
		hwe.ipv6.ib2 = e->data.bridge.ib2;
		/* setting smac_id to 0xf instruct the hw to keep original
		 * source mac address
		 */
		hwe.ipv6.l2.src_mac_hi = FIELD_PREP(AIROHA_FOE_MAC_SMAC_ID,
						    0xf);
	} else {
		memcpy(&hwe.bridge.l2, &e->data.bridge.l2,
		       sizeof(hwe.bridge.l2));
		hwe.bridge.ib2 = e->data.bridge.ib2;
		if (type == PPE_PKT_TYPE_IPV4_HNAPT)
			memcpy(&hwe.ipv4.new_tuple, &hwe.ipv4.orig_tuple,
			       sizeof(hwe.ipv4.new_tuple));
	}

	hwe.bridge.data = e->data.bridge.data;
	airoha_ppe_foe_commit_entry(ppe, &hwe, hash, rx_wlan);

	return 0;
}

static void airoha_ppe_foe_insert_entry(struct airoha_ppe *ppe,
					struct sk_buff *skb,
					u32 hash, bool rx_wlan)
{
	struct airoha_flow_table_entry *e;
	struct airoha_foe_bridge br = {};
	struct airoha_foe_entry *hwe;
	bool commit_done = false;
	struct hlist_node *n;
	u32 index, state;

	spin_lock_bh(&ppe_lock);

	hwe = airoha_ppe_foe_get_entry_locked(ppe, hash);
	if (!hwe)
		goto unlock;

	state = FIELD_GET(AIROHA_FOE_IB1_BIND_STATE, hwe->ib1);
	if (state == AIROHA_FOE_STATE_BIND)
		goto unlock;

	index = airoha_ppe_foe_get_entry_hash(ppe, hwe);
	hlist_for_each_entry_safe(e, n, &ppe->foe_flow[index], list) {
		if (e->type == FLOW_TYPE_L2_SUBFLOW) {
			state = FIELD_GET(AIROHA_FOE_IB1_BIND_STATE, hwe->ib1);
			if (state != AIROHA_FOE_STATE_BIND) {
				e->hash = 0xffff;
				airoha_ppe_foe_remove_flow(ppe, e);
			}
			continue;
		}

		if (!airoha_ppe_foe_compare_entry(e, hwe))
			continue;

		airoha_ppe_foe_commit_entry(ppe, &e->data, hash, rx_wlan);
		commit_done = true;
		e->hash = hash;
	}

	if (commit_done)
		goto unlock;

	airoha_ppe_foe_set_bridge_addrs(&br, eth_hdr(skb));
	e = rhashtable_lookup_fast(&ppe->l2_flows, &br,
				   airoha_l2_flow_table_params);
	if (e)
		airoha_ppe_foe_commit_subflow_entry(ppe, e, hash, rx_wlan);
unlock:
	spin_unlock_bh(&ppe_lock);
}

static int
airoha_ppe_foe_l2_flow_commit_entry(struct airoha_ppe *ppe,
				    struct airoha_flow_table_entry *e)
{
	struct airoha_flow_table_entry *prev;

	e->type = FLOW_TYPE_L2;
	prev = rhashtable_lookup_get_insert_fast(&ppe->l2_flows, &e->l2_node,
						 airoha_l2_flow_table_params);
	if (!prev)
		return 0;

	if (IS_ERR(prev))
		return PTR_ERR(prev);

	return rhashtable_replace_fast(&ppe->l2_flows, &prev->l2_node,
				       &e->l2_node,
				       airoha_l2_flow_table_params);
}

static int airoha_ppe_foe_flow_commit_entry(struct airoha_ppe *ppe,
					    struct airoha_flow_table_entry *e)
{
	int type = FIELD_GET(AIROHA_FOE_IB1_BIND_PACKET_TYPE, e->data.ib1);
	u32 hash;

	if (type == PPE_PKT_TYPE_BRIDGE)
		return airoha_ppe_foe_l2_flow_commit_entry(ppe, e);

	hash = airoha_ppe_foe_get_entry_hash(ppe, &e->data);
	e->type = FLOW_TYPE_L4;
	e->hash = 0xffff;

	spin_lock_bh(&ppe_lock);
	hlist_add_head(&e->list, &ppe->foe_flow[hash]);
	spin_unlock_bh(&ppe_lock);

	return 0;
}

static int airoha_ppe_get_entry_idle_time(struct airoha_ppe *ppe, u32 ib1)
{
	u32 state = FIELD_GET(AIROHA_FOE_IB1_BIND_STATE, ib1);
	u32 ts, ts_mask, now = airoha_ppe_get_timestamp(ppe);
	int idle;

	if (state == AIROHA_FOE_STATE_BIND) {
		ts = FIELD_GET(AIROHA_FOE_IB1_BIND_TIMESTAMP, ib1);
		ts_mask = AIROHA_FOE_IB1_BIND_TIMESTAMP;
	} else {
		ts = FIELD_GET(AIROHA_FOE_IB1_UNBIND_TIMESTAMP, ib1);
		now = FIELD_GET(AIROHA_FOE_IB1_UNBIND_TIMESTAMP, now);
		ts_mask = AIROHA_FOE_IB1_UNBIND_TIMESTAMP;
	}
	idle = now - ts;

	return idle < 0 ? idle + ts_mask + 1 : idle;
}

static void
airoha_ppe_foe_flow_l2_entry_update(struct airoha_ppe *ppe,
				    struct airoha_flow_table_entry *e)
{
	int min_idle = airoha_ppe_get_entry_idle_time(ppe, e->data.ib1);
	struct airoha_flow_table_entry *iter;
	struct hlist_node *n;

	lockdep_assert_held(&ppe_lock);

	hlist_for_each_entry_safe(iter, n, &e->l2_flows, l2_subflow_node) {
		struct airoha_foe_entry *hwe;
		u32 ib1, state;
		int idle;

		hwe = airoha_ppe_foe_get_entry_locked(ppe, iter->hash);
		if (!hwe)
			continue;

		ib1 = READ_ONCE(hwe->ib1);
		state = FIELD_GET(AIROHA_FOE_IB1_BIND_STATE, ib1);
		if (state != AIROHA_FOE_STATE_BIND) {
			iter->hash = 0xffff;
			airoha_ppe_foe_remove_flow(ppe, iter);
			continue;
		}

		idle = airoha_ppe_get_entry_idle_time(ppe, ib1);
		if (idle >= min_idle)
			continue;

		min_idle = idle;
		e->data.ib1 &= ~AIROHA_FOE_IB1_BIND_TIMESTAMP;
		e->data.ib1 |= ib1 & AIROHA_FOE_IB1_BIND_TIMESTAMP;
	}
}

static void airoha_ppe_foe_flow_entry_update(struct airoha_ppe *ppe,
					     struct airoha_flow_table_entry *e)
{
	struct airoha_foe_entry *hwe_p, hwe = {};

	spin_lock_bh(&ppe_lock);

	if (e->type == FLOW_TYPE_L2) {
		airoha_ppe_foe_flow_l2_entry_update(ppe, e);
		goto unlock;
	}

	if (e->hash == 0xffff)
		goto unlock;

	hwe_p = airoha_ppe_foe_get_entry_locked(ppe, e->hash);
	if (!hwe_p)
		goto unlock;

	memcpy(&hwe, hwe_p, sizeof(*hwe_p));
	if (!airoha_ppe_foe_compare_entry(e, &hwe)) {
		e->hash = 0xffff;
		goto unlock;
	}

	e->data.ib1 = hwe.ib1;
unlock:
	spin_unlock_bh(&ppe_lock);
}

static int airoha_ppe_entry_idle_time(struct airoha_ppe *ppe,
				      struct airoha_flow_table_entry *e)
{
	airoha_ppe_foe_flow_entry_update(ppe, e);

	return airoha_ppe_get_entry_idle_time(ppe, e->data.ib1);
}

static int airoha_ppe_flow_offload_replace(struct airoha_eth *eth,
					   struct flow_cls_offload *f)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct airoha_flow_table_entry *e;
	struct airoha_flow_data data = {};
	struct net_device *odev = NULL;
	struct flow_action_entry *act;
	struct airoha_foe_entry hwe;
	int err, i, offload_type;
	u16 addr_type = 0;
	u8 l4proto = 0;

	if (rhashtable_lookup(&eth->flow_table, &f->cookie,
			      airoha_flow_table_params))
		return -EEXIST;

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_META))
		return -EOPNOTSUPP;

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_CONTROL)) {
		struct flow_match_control match;

		flow_rule_match_control(rule, &match);
		addr_type = match.key->addr_type;
		if (flow_rule_has_control_flags(match.mask->flags,
						f->common.extack))
			return -EOPNOTSUPP;
	} else {
		return -EOPNOTSUPP;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_BASIC)) {
		struct flow_match_basic match;

		flow_rule_match_basic(rule, &match);
		l4proto = match.key->ip_proto;
	} else {
		return -EOPNOTSUPP;
	}

	switch (addr_type) {
	case 0:
		offload_type = PPE_PKT_TYPE_BRIDGE;
		if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
			struct flow_match_eth_addrs match;

			flow_rule_match_eth_addrs(rule, &match);
			memcpy(data.eth.h_dest, match.key->dst, ETH_ALEN);
			memcpy(data.eth.h_source, match.key->src, ETH_ALEN);
		} else {
			return -EOPNOTSUPP;
		}
		break;
	case FLOW_DISSECTOR_KEY_IPV4_ADDRS:
		offload_type = PPE_PKT_TYPE_IPV4_HNAPT;
		break;
	case FLOW_DISSECTOR_KEY_IPV6_ADDRS:
		offload_type = PPE_PKT_TYPE_IPV6_ROUTE_5T;
		break;
	default:
		return -EOPNOTSUPP;
	}

	flow_action_for_each(i, act, &rule->action) {
		switch (act->id) {
		case FLOW_ACTION_MANGLE:
			if (offload_type == PPE_PKT_TYPE_BRIDGE)
				return -EOPNOTSUPP;

			if (act->mangle.htype == FLOW_ACT_MANGLE_HDR_TYPE_ETH)
				airoha_ppe_flow_mangle_eth(act, &data.eth);
			break;
		case FLOW_ACTION_REDIRECT:
			odev = act->dev;
			break;
		case FLOW_ACTION_CSUM:
			break;
		case FLOW_ACTION_VLAN_PUSH:
			if (data.vlan.num == 2 ||
			    act->vlan.proto != htons(ETH_P_8021Q))
				return -EOPNOTSUPP;

			data.vlan.hdr[data.vlan.num].id = act->vlan.vid;
			data.vlan.hdr[data.vlan.num].proto = act->vlan.proto;
			data.vlan.num++;
			break;
		case FLOW_ACTION_VLAN_POP:
			break;
		case FLOW_ACTION_PPPOE_PUSH:
			if (data.pppoe.num == 1 || data.vlan.num == 2)
				return -EOPNOTSUPP;

			data.pppoe.sid = act->pppoe.sid;
			data.pppoe.num++;
			break;
		default:
			return -EOPNOTSUPP;
		}
	}

	if (!is_valid_ether_addr(data.eth.h_source) ||
	    !is_valid_ether_addr(data.eth.h_dest))
		return -EINVAL;

	err = airoha_ppe_foe_entry_prepare(eth, &hwe, odev, offload_type,
					   &data, l4proto);
	if (err)
		return err;

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_PORTS)) {
		struct flow_match_ports ports;

		if (offload_type == PPE_PKT_TYPE_BRIDGE)
			return -EOPNOTSUPP;

		flow_rule_match_ports(rule, &ports);
		data.src_port = ports.key->src;
		data.dst_port = ports.key->dst;
	} else if (offload_type != PPE_PKT_TYPE_BRIDGE) {
		return -EOPNOTSUPP;
	}

	if (addr_type == FLOW_DISSECTOR_KEY_IPV4_ADDRS) {
		struct flow_match_ipv4_addrs addrs;

		flow_rule_match_ipv4_addrs(rule, &addrs);
		data.v4.src_addr = addrs.key->src;
		data.v4.dst_addr = addrs.key->dst;
		airoha_ppe_foe_entry_set_ipv4_tuple(&hwe, &data, false);
	}

	if (addr_type == FLOW_DISSECTOR_KEY_IPV6_ADDRS) {
		struct flow_match_ipv6_addrs addrs;

		flow_rule_match_ipv6_addrs(rule, &addrs);

		data.v6.src_addr = addrs.key->src;
		data.v6.dst_addr = addrs.key->dst;
		airoha_ppe_foe_entry_set_ipv6_tuple(&hwe, &data);
	}

	flow_action_for_each(i, act, &rule->action) {
		if (act->id != FLOW_ACTION_MANGLE)
			continue;

		if (offload_type == PPE_PKT_TYPE_BRIDGE)
			return -EOPNOTSUPP;

		switch (act->mangle.htype) {
		case FLOW_ACT_MANGLE_HDR_TYPE_TCP:
		case FLOW_ACT_MANGLE_HDR_TYPE_UDP:
			err = airoha_ppe_flow_mangle_ports(act, &data);
			break;
		case FLOW_ACT_MANGLE_HDR_TYPE_IP4:
			err = airoha_ppe_flow_mangle_ipv4(act, &data);
			break;
		case FLOW_ACT_MANGLE_HDR_TYPE_ETH:
			/* handled earlier */
			break;
		default:
			return -EOPNOTSUPP;
		}

		if (err)
			return err;
	}

	if (addr_type == FLOW_DISSECTOR_KEY_IPV4_ADDRS) {
		err = airoha_ppe_foe_entry_set_ipv4_tuple(&hwe, &data, true);
		if (err)
			return err;
	}

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;

	e->cookie = f->cookie;
	memcpy(&e->data, &hwe, sizeof(e->data));

	err = airoha_ppe_foe_flow_commit_entry(eth->ppe, e);
	if (err)
		goto free_entry;

	err = rhashtable_insert_fast(&eth->flow_table, &e->node,
				     airoha_flow_table_params);
	if (err < 0)
		goto remove_foe_entry;

	return 0;

remove_foe_entry:
	airoha_ppe_foe_flow_remove_entry(eth->ppe, e);
free_entry:
	kfree(e);

	return err;
}

static int airoha_ppe_flow_offload_destroy(struct airoha_eth *eth,
					   struct flow_cls_offload *f)
{
	struct airoha_flow_table_entry *e;

	e = rhashtable_lookup(&eth->flow_table, &f->cookie,
			      airoha_flow_table_params);
	if (!e)
		return -ENOENT;

	airoha_ppe_foe_flow_remove_entry(eth->ppe, e);
	rhashtable_remove_fast(&eth->flow_table, &e->node,
			       airoha_flow_table_params);
	kfree(e);

	return 0;
}

void airoha_ppe_foe_entry_get_stats(struct airoha_ppe *ppe, u32 hash,
				    struct airoha_foe_stats64 *stats)
{
	struct airoha_eth *eth = ppe->eth;
	int ppe_num_stats_entries;
	struct airoha_npu *npu;
	u32 index;

	ppe_num_stats_entries = airoha_ppe_get_total_num_stats_entries(ppe);
	if (ppe_num_stats_entries < 0)
		return;

	if (airoha_ppe_foe_get_flow_stats_index(ppe, hash, &index))
		return;

	if (index >= ppe_num_stats_entries)
		return;

	rcu_read_lock();

	npu = rcu_dereference(eth->npu);
	if (npu) {
		u64 packets = ppe->foe_stats[index].packets;
		u64 bytes = ppe->foe_stats[index].bytes;
		struct airoha_foe_stats npu_stats;

		memcpy_fromio(&npu_stats, &npu->stats[index],
			      sizeof(*npu->stats));
		stats->packets = packets << 32 | npu_stats.packets;
		stats->bytes = bytes << 32 | npu_stats.bytes;
	}

	rcu_read_unlock();
}

static int airoha_ppe_flow_offload_stats(struct airoha_eth *eth,
					 struct flow_cls_offload *f)
{
	struct airoha_flow_table_entry *e;
	u32 idle;

	e = rhashtable_lookup(&eth->flow_table, &f->cookie,
			      airoha_flow_table_params);
	if (!e)
		return -ENOENT;

	idle = airoha_ppe_entry_idle_time(eth->ppe, e);
	f->stats.lastused = jiffies - idle * HZ;

	if (e->hash != 0xffff) {
		struct airoha_foe_stats64 stats = {};

		airoha_ppe_foe_entry_get_stats(eth->ppe, e->hash, &stats);
		f->stats.pkts += (stats.packets - e->stats.packets);
		f->stats.bytes += (stats.bytes - e->stats.bytes);
		e->stats = stats;
	}

	return 0;
}

static int airoha_ppe_flow_offload_cmd(struct airoha_eth *eth,
				       struct flow_cls_offload *f)
{
	switch (f->command) {
	case FLOW_CLS_REPLACE:
		return airoha_ppe_flow_offload_replace(eth, f);
	case FLOW_CLS_DESTROY:
		return airoha_ppe_flow_offload_destroy(eth, f);
	case FLOW_CLS_STATS:
		return airoha_ppe_flow_offload_stats(eth, f);
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int airoha_ppe_flush_sram_entries(struct airoha_ppe *ppe)
{
	u32 sram_num_entries = airoha_ppe_get_total_sram_num_entries(ppe);
	struct airoha_foe_entry *hwe = ppe->foe;
	int i, err = 0;

	for (i = 0; i < sram_num_entries; i++) {
		int err;

		memset(&hwe[i], 0, sizeof(*hwe));
		err = airoha_ppe_foe_commit_sram_entry(ppe, i);
		if (err)
			break;
	}

	return err;
}

static struct airoha_npu *airoha_ppe_npu_get(struct airoha_eth *eth)
{
	struct airoha_npu *npu = airoha_npu_get(eth->dev);

	if (IS_ERR(npu)) {
		request_module("airoha-npu");
		npu = airoha_npu_get(eth->dev);
	}

	return npu;
}

static int airoha_ppe_offload_setup(struct airoha_eth *eth)
{
	struct airoha_npu *npu = airoha_ppe_npu_get(eth);
	struct airoha_ppe *ppe = eth->ppe;
	int err, ppe_num_stats_entries;

	if (IS_ERR(npu))
		return PTR_ERR(npu);

	err = npu->ops.ppe_init(npu);
	if (err)
		goto error_npu_put;

	ppe_num_stats_entries = airoha_ppe_get_total_num_stats_entries(ppe);
	if (ppe_num_stats_entries > 0) {
		err = npu->ops.ppe_init_stats(npu, ppe->foe_stats_dma,
					      ppe_num_stats_entries);
		if (err)
			goto error_npu_put;
	}

	airoha_ppe_hw_init(ppe);
	airoha_ppe_foe_flow_stats_reset(ppe, npu);

	rcu_assign_pointer(eth->npu, npu);
	synchronize_rcu();

	return 0;

error_npu_put:
	airoha_npu_put(npu);

	return err;
}

int airoha_ppe_setup_tc_block_cb(struct airoha_ppe_dev *dev, void *type_data)
{
	struct airoha_ppe *ppe = dev->priv;
	struct airoha_eth *eth = ppe->eth;
	int err = 0;

	mutex_lock(&flow_offload_mutex);

	if (!eth->npu)
		err = airoha_ppe_offload_setup(eth);
	if (!err)
		err = airoha_ppe_flow_offload_cmd(eth, type_data);

	mutex_unlock(&flow_offload_mutex);

	return err;
}

void airoha_ppe_check_skb(struct airoha_ppe_dev *dev, struct sk_buff *skb,
			  u16 hash, bool rx_wlan)
{
	struct airoha_ppe *ppe = dev->priv;
	u32 ppe_hash_mask = airoha_ppe_get_total_num_entries(ppe) - 1;
	u16 now, diff;

	if (hash > ppe_hash_mask)
		return;

	now = (u16)jiffies;
	diff = now - ppe->foe_check_time[hash];
	if (diff < HZ / 10)
		return;

	ppe->foe_check_time[hash] = now;
	airoha_ppe_foe_insert_entry(ppe, skb, hash, rx_wlan);
}

void airoha_ppe_init_upd_mem(struct airoha_gdm_port *port)
{
	struct airoha_eth *eth = port->qdma->eth;
	struct net_device *dev = port->dev;
	const u8 *addr = dev->dev_addr;
	u32 val;

	val = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	airoha_fe_wr(eth, REG_UPDMEM_DATA(0), val);
	airoha_fe_wr(eth, REG_UPDMEM_CTRL(0),
		     FIELD_PREP(PPE_UPDMEM_ADDR_MASK, port->id) |
		     PPE_UPDMEM_WR_MASK | PPE_UPDMEM_REQ_MASK);

	val = (addr[0] << 8) | addr[1];
	airoha_fe_wr(eth, REG_UPDMEM_DATA(0), val);
	airoha_fe_wr(eth, REG_UPDMEM_CTRL(0),
		     FIELD_PREP(PPE_UPDMEM_ADDR_MASK, port->id) |
		     FIELD_PREP(PPE_UPDMEM_OFFSET_MASK, 1) |
		     PPE_UPDMEM_WR_MASK | PPE_UPDMEM_REQ_MASK);
}

struct airoha_ppe_dev *airoha_ppe_get_dev(struct device *dev)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct airoha_eth *eth;

	np = of_parse_phandle(dev->of_node, "airoha,eth", 0);
	if (!np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_err(dev, "cannot find device node %s\n", np->name);
		of_node_put(np);
		return ERR_PTR(-ENODEV);
	}
	of_node_put(np);

	if (!try_module_get(THIS_MODULE)) {
		dev_err(dev, "failed to get the device driver module\n");
		goto error_pdev_put;
	}

	eth = platform_get_drvdata(pdev);
	if (!eth)
		goto error_module_put;

	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_SUPPLIER)) {
		dev_err(&pdev->dev,
			"failed to create device link to consumer %s\n",
			dev_name(dev));
		goto error_module_put;
	}

	return &eth->ppe->dev;

error_module_put:
	module_put(THIS_MODULE);
error_pdev_put:
	platform_device_put(pdev);

	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(airoha_ppe_get_dev);

void airoha_ppe_put_dev(struct airoha_ppe_dev *dev)
{
	struct airoha_ppe *ppe = dev->priv;
	struct airoha_eth *eth = ppe->eth;

	module_put(THIS_MODULE);
	put_device(eth->dev);
}
EXPORT_SYMBOL_GPL(airoha_ppe_put_dev);

int airoha_ppe_init(struct airoha_eth *eth)
{
	int foe_size, err, ppe_num_stats_entries;
	u32 ppe_num_entries;
	struct airoha_ppe *ppe;

	ppe = devm_kzalloc(eth->dev, sizeof(*ppe), GFP_KERNEL);
	if (!ppe)
		return -ENOMEM;

	ppe->dev.ops.setup_tc_block_cb = airoha_ppe_setup_tc_block_cb;
	ppe->dev.ops.check_skb = airoha_ppe_check_skb;
	ppe->dev.priv = ppe;
	ppe->eth = eth;
	eth->ppe = ppe;

	ppe_num_entries = airoha_ppe_get_total_num_entries(ppe);
	foe_size = ppe_num_entries * sizeof(struct airoha_foe_entry);
	ppe->foe = dmam_alloc_coherent(eth->dev, foe_size, &ppe->foe_dma,
				       GFP_KERNEL);
	if (!ppe->foe)
		return -ENOMEM;

	ppe->foe_flow = devm_kzalloc(eth->dev,
				     ppe_num_entries * sizeof(*ppe->foe_flow),
				     GFP_KERNEL);
	if (!ppe->foe_flow)
		return -ENOMEM;

	ppe_num_stats_entries = airoha_ppe_get_total_num_stats_entries(ppe);
	if (ppe_num_stats_entries > 0) {
		foe_size = ppe_num_stats_entries * sizeof(*ppe->foe_stats);
		ppe->foe_stats = dmam_alloc_coherent(eth->dev, foe_size,
						     &ppe->foe_stats_dma,
						     GFP_KERNEL);
		if (!ppe->foe_stats)
			return -ENOMEM;
	}

	ppe->foe_check_time = devm_kzalloc(eth->dev, ppe_num_entries,
					   GFP_KERNEL);
	if (!ppe->foe_check_time)
		return -ENOMEM;

	err = airoha_ppe_flush_sram_entries(ppe);
	if (err)
		return err;

	err = rhashtable_init(&eth->flow_table, &airoha_flow_table_params);
	if (err)
		return err;

	err = rhashtable_init(&ppe->l2_flows, &airoha_l2_flow_table_params);
	if (err)
		goto error_flow_table_destroy;

	err = airoha_ppe_debugfs_init(ppe);
	if (err)
		goto error_l2_flow_table_destroy;

	return 0;

error_l2_flow_table_destroy:
	rhashtable_destroy(&ppe->l2_flows);
error_flow_table_destroy:
	rhashtable_destroy(&eth->flow_table);

	return err;
}

void airoha_ppe_deinit(struct airoha_eth *eth)
{
	struct airoha_npu *npu;

	rcu_read_lock();
	npu = rcu_dereference(eth->npu);
	if (npu) {
		npu->ops.ppe_deinit(npu);
		airoha_npu_put(npu);
	}
	rcu_read_unlock();

	rhashtable_destroy(&eth->ppe->l2_flows);
	rhashtable_destroy(&eth->flow_table);
	debugfs_remove(eth->ppe->debugfs_dir);
}
