// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>

#include "fbnic.h"
#include "fbnic_netdev.h"
#include "fbnic_rpc.h"

void fbnic_reset_indir_tbl(struct fbnic_net *fbn)
{
	unsigned int num_rx = fbn->num_rx_queues;
	unsigned int i;

	for (i = 0; i < FBNIC_RPC_RSS_TBL_SIZE; i++) {
		fbn->indir_tbl[0][i] = ethtool_rxfh_indir_default(i, num_rx);
		fbn->indir_tbl[1][i] = ethtool_rxfh_indir_default(i, num_rx);
	}
}

void fbnic_rss_key_fill(u32 *buffer)
{
	static u32 rss_key[FBNIC_RPC_RSS_KEY_DWORD_LEN];

	net_get_random_once(rss_key, sizeof(rss_key));
	rss_key[FBNIC_RPC_RSS_KEY_LAST_IDX] &= FBNIC_RPC_RSS_KEY_LAST_MASK;

	memcpy(buffer, rss_key, sizeof(rss_key));
}

#define RX_HASH_OPT_L4 \
	(RXH_IP_SRC | RXH_IP_DST | RXH_L4_B_0_1 | RXH_L4_B_2_3)
#define RX_HASH_OPT_L3 \
	(RXH_IP_SRC | RXH_IP_DST)
#define RX_HASH_OPT_L2 RXH_L2DA

void fbnic_rss_init_en_mask(struct fbnic_net *fbn)
{
	fbn->rss_flow_hash[FBNIC_TCP4_HASH_OPT] = RX_HASH_OPT_L4;
	fbn->rss_flow_hash[FBNIC_TCP6_HASH_OPT] = RX_HASH_OPT_L4;

	fbn->rss_flow_hash[FBNIC_UDP4_HASH_OPT] = RX_HASH_OPT_L3;
	fbn->rss_flow_hash[FBNIC_UDP6_HASH_OPT] = RX_HASH_OPT_L3;
	fbn->rss_flow_hash[FBNIC_IPV4_HASH_OPT] = RX_HASH_OPT_L3;
	fbn->rss_flow_hash[FBNIC_IPV6_HASH_OPT] = RX_HASH_OPT_L3;

	fbn->rss_flow_hash[FBNIC_ETHER_HASH_OPT] = RX_HASH_OPT_L2;
}

void fbnic_rss_disable_hw(struct fbnic_dev *fbd)
{
	/* Disable RPC by clearing enable bit and configuration */
	if (!fbnic_bmc_present(fbd))
		wr32(fbd, FBNIC_RPC_RMI_CONFIG,
		     FIELD_PREP(FBNIC_RPC_RMI_CONFIG_OH_BYTES, 20));
}

#define FBNIC_FH_2_RSSEM_BIT(_fh, _rssem, _val)		\
	FIELD_PREP(FBNIC_RPC_ACT_TBL1_RSS_ENA_##_rssem,	\
		   FIELD_GET(RXH_##_fh, _val))
static u16 fbnic_flow_hash_2_rss_en_mask(struct fbnic_net *fbn, int flow_type)
{
	u32 flow_hash = fbn->rss_flow_hash[flow_type];
	u32 rss_en_mask = 0;

	rss_en_mask |= FBNIC_FH_2_RSSEM_BIT(L2DA, L2_DA, flow_hash);
	rss_en_mask |= FBNIC_FH_2_RSSEM_BIT(IP_SRC, IP_SRC, flow_hash);
	rss_en_mask |= FBNIC_FH_2_RSSEM_BIT(IP_DST, IP_DST, flow_hash);
	rss_en_mask |= FBNIC_FH_2_RSSEM_BIT(L4_B_0_1, L4_SRC, flow_hash);
	rss_en_mask |= FBNIC_FH_2_RSSEM_BIT(L4_B_2_3, L4_DST, flow_hash);

	return rss_en_mask;
}

void fbnic_rss_reinit_hw(struct fbnic_dev *fbd, struct fbnic_net *fbn)
{
	unsigned int i;

	for (i = 0; i < FBNIC_RPC_RSS_TBL_SIZE; i++) {
		wr32(fbd, FBNIC_RPC_RSS_TBL(0, i), fbn->indir_tbl[0][i]);
		wr32(fbd, FBNIC_RPC_RSS_TBL(1, i), fbn->indir_tbl[1][i]);
	}

	for (i = 0; i < FBNIC_RPC_RSS_KEY_DWORD_LEN; i++)
		wr32(fbd, FBNIC_RPC_RSS_KEY(i), fbn->rss_key[i]);

	/* Default action for this to drop w/ no destination */
	wr32(fbd, FBNIC_RPC_ACT_TBL0_DEFAULT, FBNIC_RPC_ACT_TBL0_DROP);
	wrfl(fbd);

	wr32(fbd, FBNIC_RPC_ACT_TBL1_DEFAULT, 0);

	/* If it isn't already enabled set the RMI Config value to enable RPC */
	wr32(fbd, FBNIC_RPC_RMI_CONFIG,
	     FIELD_PREP(FBNIC_RPC_RMI_CONFIG_MTU, FBNIC_MAX_JUMBO_FRAME_SIZE) |
	     FIELD_PREP(FBNIC_RPC_RMI_CONFIG_OH_BYTES, 20) |
	     FBNIC_RPC_RMI_CONFIG_ENABLE);
}

void fbnic_bmc_rpc_all_multi_config(struct fbnic_dev *fbd,
				    bool enable_host)
{
	struct fbnic_act_tcam *act_tcam;
	struct fbnic_mac_addr *mac_addr;
	int j;

	/* We need to add the all multicast filter at the end of the
	 * multicast address list. This way if there are any that are
	 * shared between the host and the BMC they can be directed to
	 * both. Otherwise the remainder just get sent directly to the
	 * BMC.
	 */
	mac_addr = &fbd->mac_addr[fbd->mac_addr_boundary - 1];
	if (fbnic_bmc_present(fbd) && fbd->fw_cap.all_multi) {
		if (mac_addr->state != FBNIC_TCAM_S_VALID) {
			eth_zero_addr(mac_addr->value.addr8);
			eth_broadcast_addr(mac_addr->mask.addr8);
			mac_addr->value.addr8[0] ^= 1;
			mac_addr->mask.addr8[0] ^= 1;
			set_bit(FBNIC_MAC_ADDR_T_BMC, mac_addr->act_tcam);
			mac_addr->state = FBNIC_TCAM_S_ADD;
		}
		if (enable_host)
			set_bit(FBNIC_MAC_ADDR_T_ALLMULTI,
				mac_addr->act_tcam);
		else
			clear_bit(FBNIC_MAC_ADDR_T_ALLMULTI,
				  mac_addr->act_tcam);
	} else if (!test_bit(FBNIC_MAC_ADDR_T_BMC, mac_addr->act_tcam) &&
		   !is_zero_ether_addr(mac_addr->mask.addr8) &&
		   mac_addr->state == FBNIC_TCAM_S_VALID) {
		clear_bit(FBNIC_MAC_ADDR_T_ALLMULTI, mac_addr->act_tcam);
		clear_bit(FBNIC_MAC_ADDR_T_BMC, mac_addr->act_tcam);
		mac_addr->state = FBNIC_TCAM_S_DELETE;
	}

	/* We have to add a special handler for multicast as the
	 * BMC may have an all-multi rule already in place. As such
	 * adding a rule ourselves won't do any good so we will have
	 * to modify the rules for the ALL MULTI below if the BMC
	 * already has the rule in place.
	 */
	act_tcam = &fbd->act_tcam[FBNIC_RPC_ACT_TBL_BMC_ALL_MULTI_OFFSET];

	/* If we are not enabling the rule just delete it. We will fall
	 * back to the RSS rules that support the multicast addresses.
	 */
	if (!fbnic_bmc_present(fbd) || !fbd->fw_cap.all_multi || enable_host) {
		if (act_tcam->state == FBNIC_TCAM_S_VALID)
			act_tcam->state = FBNIC_TCAM_S_DELETE;
		return;
	}

	/* Rewrite TCAM rule 23 to handle BMC all-multi traffic */
	act_tcam->dest = FIELD_PREP(FBNIC_RPC_ACT_TBL0_DEST_MASK,
				    FBNIC_RPC_ACT_TBL0_DEST_BMC);
	act_tcam->mask.tcam[0] = 0xffff;

	/* MACDA 0 - 3 is reserved for the BMC MAC address */
	act_tcam->value.tcam[1] =
			FIELD_PREP(FBNIC_RPC_TCAM_ACT1_L2_MACDA_IDX,
				   fbd->mac_addr_boundary - 1) |
			FBNIC_RPC_TCAM_ACT1_L2_MACDA_VALID;
	act_tcam->mask.tcam[1] = 0xffff &
			 ~FBNIC_RPC_TCAM_ACT1_L2_MACDA_IDX &
			 ~FBNIC_RPC_TCAM_ACT1_L2_MACDA_VALID;

	for (j = 2; j < FBNIC_RPC_TCAM_ACT_WORD_LEN; j++)
		act_tcam->mask.tcam[j] = 0xffff;

	act_tcam->state = FBNIC_TCAM_S_UPDATE;
}

void fbnic_bmc_rpc_init(struct fbnic_dev *fbd)
{
	int i = FBNIC_RPC_TCAM_MACDA_BMC_ADDR_IDX;
	struct fbnic_act_tcam *act_tcam;
	struct fbnic_mac_addr *mac_addr;
	int j;

	/* Check if BMC is present */
	if (!fbnic_bmc_present(fbd))
		return;

	/* Fetch BMC MAC addresses from firmware capabilities */
	for (j = 0; j < 4; j++) {
		u8 *bmc_mac = fbd->fw_cap.bmc_mac_addr[j];

		/* Validate BMC MAC addresses */
		if (is_zero_ether_addr(bmc_mac))
			continue;

		if (is_multicast_ether_addr(bmc_mac))
			mac_addr = __fbnic_mc_sync(fbd, bmc_mac);
		else
			mac_addr = &fbd->mac_addr[i++];

		if (!mac_addr) {
			netdev_err(fbd->netdev,
				   "No slot for BMC MAC address[%d]\n", j);
			continue;
		}

		ether_addr_copy(mac_addr->value.addr8, bmc_mac);
		eth_zero_addr(mac_addr->mask.addr8);

		set_bit(FBNIC_MAC_ADDR_T_BMC, mac_addr->act_tcam);
		mac_addr->state = FBNIC_TCAM_S_ADD;
	}

	/* Validate Broadcast is also present, record it and tag it */
	mac_addr = &fbd->mac_addr[FBNIC_RPC_TCAM_MACDA_BROADCAST_IDX];
	eth_broadcast_addr(mac_addr->value.addr8);
	set_bit(FBNIC_MAC_ADDR_T_BMC, mac_addr->act_tcam);
	mac_addr->state = FBNIC_TCAM_S_ADD;

	/* Rewrite TCAM rule 0 if it isn't present to relocate BMC rules */
	act_tcam = &fbd->act_tcam[FBNIC_RPC_ACT_TBL_BMC_OFFSET];
	act_tcam->dest = FIELD_PREP(FBNIC_RPC_ACT_TBL0_DEST_MASK,
				    FBNIC_RPC_ACT_TBL0_DEST_BMC);
	act_tcam->mask.tcam[0] = 0xffff;

	/* MACDA 0 - 3 is reserved for the BMC MAC address
	 * to account for that we have to mask out the lower 2 bits
	 * of the macda by performing an &= with 0x1c.
	 */
	act_tcam->value.tcam[1] = FBNIC_RPC_TCAM_ACT1_L2_MACDA_VALID;
	act_tcam->mask.tcam[1] = 0xffff &
			~FIELD_PREP(FBNIC_RPC_TCAM_ACT1_L2_MACDA_IDX, 0x1c) &
			~FBNIC_RPC_TCAM_ACT1_L2_MACDA_VALID;

	for (j = 2; j < FBNIC_RPC_TCAM_ACT_WORD_LEN; j++)
		act_tcam->mask.tcam[j] = 0xffff;

	act_tcam->state = FBNIC_TCAM_S_UPDATE;

	fbnic_bmc_rpc_all_multi_config(fbd, false);
}

#define FBNIC_ACT1_INIT(_l4, _udp, _ip, _v6)		\
	(((_l4) ? FBNIC_RPC_TCAM_ACT1_L4_VALID : 0) |	\
	 ((_udp) ? FBNIC_RPC_TCAM_ACT1_L4_IS_UDP : 0) |	\
	 ((_ip) ? FBNIC_RPC_TCAM_ACT1_IP_VALID : 0) |	\
	 ((_v6) ? FBNIC_RPC_TCAM_ACT1_IP_IS_V6 : 0))

void fbnic_rss_reinit(struct fbnic_dev *fbd, struct fbnic_net *fbn)
{
	static const u32 act1_value[FBNIC_NUM_HASH_OPT] = {
		FBNIC_ACT1_INIT(1, 1, 1, 1),	/* UDP6 */
		FBNIC_ACT1_INIT(1, 1, 1, 0),	/* UDP4 */
		FBNIC_ACT1_INIT(1, 0, 1, 1),	/* TCP6 */
		FBNIC_ACT1_INIT(1, 0, 1, 0),	/* TCP4 */
		FBNIC_ACT1_INIT(0, 0, 1, 1),	/* IP6 */
		FBNIC_ACT1_INIT(0, 0, 1, 0),	/* IP4 */
		0				/* Ether */
	};
	unsigned int i;

	/* To support scenarios where a BMC is present we must write the
	 * rules twice, once for the unicast cases, and once again for
	 * the broadcast/multicast cases as we have to support 2 destinations.
	 */
	BUILD_BUG_ON(FBNIC_RSS_EN_NUM_UNICAST * 2 != FBNIC_RSS_EN_NUM_ENTRIES);
	BUILD_BUG_ON(ARRAY_SIZE(act1_value) != FBNIC_NUM_HASH_OPT);

	/* Program RSS hash enable mask for host in action TCAM/table. */
	for (i = fbnic_bmc_present(fbd) ? 0 : FBNIC_RSS_EN_NUM_UNICAST;
	     i < FBNIC_RSS_EN_NUM_ENTRIES; i++) {
		unsigned int idx = i + FBNIC_RPC_ACT_TBL_RSS_OFFSET;
		struct fbnic_act_tcam *act_tcam = &fbd->act_tcam[idx];
		u32 flow_hash, dest, rss_en_mask;
		int flow_type, j;
		u16 value = 0;

		flow_type = i % FBNIC_RSS_EN_NUM_UNICAST;
		flow_hash = fbn->rss_flow_hash[flow_type];

		/* Set DEST_HOST based on absence of RXH_DISCARD */
		dest = FIELD_PREP(FBNIC_RPC_ACT_TBL0_DEST_MASK,
				  !(RXH_DISCARD & flow_hash) ?
				  FBNIC_RPC_ACT_TBL0_DEST_HOST : 0);

		if (i >= FBNIC_RSS_EN_NUM_UNICAST && fbnic_bmc_present(fbd))
			dest |= FIELD_PREP(FBNIC_RPC_ACT_TBL0_DEST_MASK,
					   FBNIC_RPC_ACT_TBL0_DEST_BMC);

		if (!dest)
			dest = FBNIC_RPC_ACT_TBL0_DROP;

		if (act1_value[flow_type] & FBNIC_RPC_TCAM_ACT1_L4_VALID)
			dest |= FIELD_PREP(FBNIC_RPC_ACT_TBL0_DMA_HINT,
					   FBNIC_RCD_HDR_AL_DMA_HINT_L4);

		rss_en_mask = fbnic_flow_hash_2_rss_en_mask(fbn, flow_type);

		act_tcam->dest = dest;
		act_tcam->rss_en_mask = rss_en_mask;
		act_tcam->state = FBNIC_TCAM_S_UPDATE;

		act_tcam->mask.tcam[0] = 0xffff;

		/* We reserve the upper 8 MACDA TCAM entries for host
		 * unicast. So we set the value to 24, and the mask the
		 * lower bits so that the lower entries can be used as
		 * multicast or BMC addresses.
		 */
		if (i < FBNIC_RSS_EN_NUM_UNICAST)
			value = FIELD_PREP(FBNIC_RPC_TCAM_ACT1_L2_MACDA_IDX,
					   fbd->mac_addr_boundary);
		value |= FBNIC_RPC_TCAM_ACT1_L2_MACDA_VALID;

		flow_type = i % FBNIC_RSS_EN_NUM_UNICAST;
		value |= act1_value[flow_type];

		act_tcam->value.tcam[1] = value;
		act_tcam->mask.tcam[1] = ~value;

		for (j = 2; j < FBNIC_RPC_TCAM_ACT_WORD_LEN; j++)
			act_tcam->mask.tcam[j] = 0xffff;

		act_tcam->state = FBNIC_TCAM_S_UPDATE;
	}
}

struct fbnic_mac_addr *__fbnic_uc_sync(struct fbnic_dev *fbd,
				       const unsigned char *addr)
{
	struct fbnic_mac_addr *avail_addr = NULL;
	unsigned int i;

	/* Scan from middle of list to bottom, filling bottom up.
	 * Skip the first entry which is reserved for dev_addr and
	 * leave the last entry to use for promiscuous filtering.
	 */
	for (i = fbd->mac_addr_boundary - 1;
	     i < FBNIC_RPC_TCAM_MACDA_HOST_ADDR_IDX; i++) {
		struct fbnic_mac_addr *mac_addr = &fbd->mac_addr[i];

		if (mac_addr->state == FBNIC_TCAM_S_DISABLED) {
			avail_addr = mac_addr;
		} else if (ether_addr_equal(mac_addr->value.addr8, addr)) {
			avail_addr = mac_addr;
			break;
		}
	}

	if (avail_addr && avail_addr->state == FBNIC_TCAM_S_DISABLED) {
		ether_addr_copy(avail_addr->value.addr8, addr);
		eth_zero_addr(avail_addr->mask.addr8);
		avail_addr->state = FBNIC_TCAM_S_ADD;
	}

	return avail_addr;
}

struct fbnic_mac_addr *__fbnic_mc_sync(struct fbnic_dev *fbd,
				       const unsigned char *addr)
{
	struct fbnic_mac_addr *avail_addr = NULL;
	unsigned int i;

	/* Scan from middle of list to top, filling top down.
	 * Skip over the address reserved for the BMC MAC and
	 * exclude index 0 as that belongs to the broadcast address
	 */
	for (i = fbd->mac_addr_boundary;
	     --i > FBNIC_RPC_TCAM_MACDA_BROADCAST_IDX;) {
		struct fbnic_mac_addr *mac_addr = &fbd->mac_addr[i];

		if (mac_addr->state == FBNIC_TCAM_S_DISABLED) {
			avail_addr = mac_addr;
		} else if (ether_addr_equal(mac_addr->value.addr8, addr)) {
			avail_addr = mac_addr;
			break;
		}
	}

	/* Scan the BMC addresses to see if it may have already
	 * reserved the address.
	 */
	while (--i) {
		struct fbnic_mac_addr *mac_addr = &fbd->mac_addr[i];

		if (!is_zero_ether_addr(mac_addr->mask.addr8))
			continue;

		/* Only move on if we find a match */
		if (!ether_addr_equal(mac_addr->value.addr8, addr))
			continue;

		/* We need to pull this address to the shared area */
		if (avail_addr) {
			memcpy(avail_addr, mac_addr, sizeof(*mac_addr));
			mac_addr->state = FBNIC_TCAM_S_DELETE;
			avail_addr->state = FBNIC_TCAM_S_ADD;
		}

		break;
	}

	if (avail_addr && avail_addr->state == FBNIC_TCAM_S_DISABLED) {
		ether_addr_copy(avail_addr->value.addr8, addr);
		eth_zero_addr(avail_addr->mask.addr8);
		avail_addr->state = FBNIC_TCAM_S_ADD;
	}

	return avail_addr;
}

int __fbnic_xc_unsync(struct fbnic_mac_addr *mac_addr, unsigned int tcam_idx)
{
	if (!test_and_clear_bit(tcam_idx, mac_addr->act_tcam))
		return -ENOENT;

	if (bitmap_empty(mac_addr->act_tcam, FBNIC_RPC_TCAM_ACT_NUM_ENTRIES))
		mac_addr->state = FBNIC_TCAM_S_DELETE;

	return 0;
}

void fbnic_sift_macda(struct fbnic_dev *fbd)
{
	int dest, src;

	/* Move BMC only addresses back into BMC region */
	for (dest = FBNIC_RPC_TCAM_MACDA_BMC_ADDR_IDX,
	     src = FBNIC_RPC_TCAM_MACDA_MULTICAST_IDX;
	     ++dest < FBNIC_RPC_TCAM_MACDA_BROADCAST_IDX &&
	     src < fbd->mac_addr_boundary;) {
		struct fbnic_mac_addr *dest_addr = &fbd->mac_addr[dest];

		if (dest_addr->state != FBNIC_TCAM_S_DISABLED)
			continue;

		while (src < fbd->mac_addr_boundary) {
			struct fbnic_mac_addr *src_addr = &fbd->mac_addr[src++];

			/* Verify BMC bit is set */
			if (!test_bit(FBNIC_MAC_ADDR_T_BMC, src_addr->act_tcam))
				continue;

			/* Verify filter isn't already disabled */
			if (src_addr->state == FBNIC_TCAM_S_DISABLED ||
			    src_addr->state == FBNIC_TCAM_S_DELETE)
				continue;

			/* Verify only BMC bit is set */
			if (bitmap_weight(src_addr->act_tcam,
					  FBNIC_RPC_TCAM_ACT_NUM_ENTRIES) != 1)
				continue;

			/* Verify we are not moving wildcard address */
			if (!is_zero_ether_addr(src_addr->mask.addr8))
				continue;

			memcpy(dest_addr, src_addr, sizeof(*src_addr));
			src_addr->state = FBNIC_TCAM_S_DELETE;
			dest_addr->state = FBNIC_TCAM_S_ADD;
		}
	}
}

static void fbnic_clear_macda_entry(struct fbnic_dev *fbd, unsigned int idx)
{
	int i;

	/* Invalidate entry and clear addr state info */
	for (i = 0; i <= FBNIC_RPC_TCAM_MACDA_WORD_LEN; i++)
		wr32(fbd, FBNIC_RPC_TCAM_MACDA(idx, i), 0);
}

static void fbnic_clear_macda(struct fbnic_dev *fbd)
{
	int idx;

	for (idx = ARRAY_SIZE(fbd->mac_addr); idx--;) {
		struct fbnic_mac_addr *mac_addr = &fbd->mac_addr[idx];

		if (mac_addr->state == FBNIC_TCAM_S_DISABLED)
			continue;

		if (test_bit(FBNIC_MAC_ADDR_T_BMC, mac_addr->act_tcam)) {
			if (fbnic_bmc_present(fbd))
				continue;
			dev_warn_once(fbd->dev,
				      "Found BMC MAC address w/ BMC not present\n");
		}

		fbnic_clear_macda_entry(fbd, idx);

		/* If rule was already destined for deletion just wipe it now */
		if (mac_addr->state == FBNIC_TCAM_S_DELETE) {
			memset(mac_addr, 0, sizeof(*mac_addr));
			continue;
		}

		/* Change state to update so that we will rewrite
		 * this tcam the next time fbnic_write_macda is called.
		 */
		mac_addr->state = FBNIC_TCAM_S_UPDATE;
	}
}

static void fbnic_write_macda_entry(struct fbnic_dev *fbd, unsigned int idx,
				    struct fbnic_mac_addr *mac_addr)
{
	__be16 *mask, *value;
	int i;

	mask = &mac_addr->mask.addr16[FBNIC_RPC_TCAM_MACDA_WORD_LEN - 1];
	value = &mac_addr->value.addr16[FBNIC_RPC_TCAM_MACDA_WORD_LEN - 1];

	for (i = 0; i < FBNIC_RPC_TCAM_MACDA_WORD_LEN; i++)
		wr32(fbd, FBNIC_RPC_TCAM_MACDA(idx, i),
		     FIELD_PREP(FBNIC_RPC_TCAM_MACDA_MASK, ntohs(*mask--)) |
		     FIELD_PREP(FBNIC_RPC_TCAM_MACDA_VALUE, ntohs(*value--)));

	wrfl(fbd);

	wr32(fbd, FBNIC_RPC_TCAM_MACDA(idx, i), FBNIC_RPC_TCAM_VALIDATE);
}

void fbnic_write_macda(struct fbnic_dev *fbd)
{
	int idx;

	for (idx = ARRAY_SIZE(fbd->mac_addr); idx--;) {
		struct fbnic_mac_addr *mac_addr = &fbd->mac_addr[idx];

		/* Check if update flag is set else exit. */
		if (!(mac_addr->state & FBNIC_TCAM_S_UPDATE))
			continue;

		/* Clear by writing 0s. */
		if (mac_addr->state == FBNIC_TCAM_S_DELETE) {
			/* Invalidate entry and clear addr state info */
			fbnic_clear_macda_entry(fbd, idx);
			memset(mac_addr, 0, sizeof(*mac_addr));

			continue;
		}

		fbnic_write_macda_entry(fbd, idx, mac_addr);

		mac_addr->state = FBNIC_TCAM_S_VALID;
	}
}

static void fbnic_clear_act_tcam(struct fbnic_dev *fbd, unsigned int idx)
{
	int i;

	/* Invalidate entry and clear addr state info */
	for (i = 0; i <= FBNIC_RPC_TCAM_ACT_WORD_LEN; i++)
		wr32(fbd, FBNIC_RPC_TCAM_ACT(idx, i), 0);
}

void fbnic_clear_rules(struct fbnic_dev *fbd)
{
	u32 dest = FIELD_PREP(FBNIC_RPC_ACT_TBL0_DEST_MASK,
			      FBNIC_RPC_ACT_TBL0_DEST_BMC);
	int i = FBNIC_RPC_TCAM_ACT_NUM_ENTRIES - 1;
	struct fbnic_act_tcam *act_tcam;

	/* Clear MAC rules */
	fbnic_clear_macda(fbd);

	/* If BMC is present we need to preserve the last rule which
	 * will be used to route traffic to the BMC if it is received.
	 *
	 * At this point it should be the only MAC address in the MACDA
	 * so any unicast or multicast traffic received should be routed
	 * to it. So leave the last rule in place.
	 *
	 * It will be rewritten to add the host again when we bring
	 * the interface back up.
	 */
	if (fbnic_bmc_present(fbd)) {
		act_tcam = &fbd->act_tcam[i];

		if (act_tcam->state == FBNIC_TCAM_S_VALID &&
		    (act_tcam->dest & dest)) {
			wr32(fbd, FBNIC_RPC_ACT_TBL0(i), dest);
			wr32(fbd, FBNIC_RPC_ACT_TBL1(i), 0);

			act_tcam->state = FBNIC_TCAM_S_UPDATE;

			i--;
		}
	}

	/* Work from the bottom up deleting all other rules from hardware */
	do {
		act_tcam = &fbd->act_tcam[i];

		if (act_tcam->state != FBNIC_TCAM_S_VALID)
			continue;

		fbnic_clear_act_tcam(fbd, i);
		act_tcam->state = FBNIC_TCAM_S_UPDATE;
	} while (i--);
}

static void fbnic_delete_act_tcam(struct fbnic_dev *fbd, unsigned int idx)
{
	fbnic_clear_act_tcam(fbd, idx);
	memset(&fbd->act_tcam[idx], 0, sizeof(struct fbnic_act_tcam));
}

static void fbnic_update_act_tcam(struct fbnic_dev *fbd, unsigned int idx)
{
	struct fbnic_act_tcam *act_tcam = &fbd->act_tcam[idx];
	int i;

	/* Update entry by writing the destination and RSS mask */
	wr32(fbd, FBNIC_RPC_ACT_TBL0(idx), act_tcam->dest);
	wr32(fbd, FBNIC_RPC_ACT_TBL1(idx), act_tcam->rss_en_mask);

	/* Write new TCAM rule to hardware */
	for (i = 0; i < FBNIC_RPC_TCAM_ACT_WORD_LEN; i++)
		wr32(fbd, FBNIC_RPC_TCAM_ACT(idx, i),
		     FIELD_PREP(FBNIC_RPC_TCAM_ACT_MASK,
				act_tcam->mask.tcam[i]) |
		     FIELD_PREP(FBNIC_RPC_TCAM_ACT_VALUE,
				act_tcam->value.tcam[i]));

	wrfl(fbd);

	wr32(fbd, FBNIC_RPC_TCAM_ACT(idx, i), FBNIC_RPC_TCAM_VALIDATE);
	act_tcam->state = FBNIC_TCAM_S_VALID;
}

void fbnic_write_rules(struct fbnic_dev *fbd)
{
	int i;

	/* Flush any pending action table rules */
	for (i = 0; i < FBNIC_RPC_ACT_TBL_NUM_ENTRIES; i++) {
		struct fbnic_act_tcam *act_tcam = &fbd->act_tcam[i];

		/* Check if update flag is set else exit. */
		if (!(act_tcam->state & FBNIC_TCAM_S_UPDATE))
			continue;

		if (act_tcam->state == FBNIC_TCAM_S_DELETE)
			fbnic_delete_act_tcam(fbd, i);
		else
			fbnic_update_act_tcam(fbd, i);
	}
}
