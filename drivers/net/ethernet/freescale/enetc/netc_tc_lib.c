// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2024 NXP
 */
#include <linux/fsl/netc_lib.h>

#define SDU_TYPE_MPDU				1

struct netc_flower_rule *
netc_find_flower_rule_by_cookie(struct ntmp_priv *priv, int port_id,
				unsigned long cookie)
{
	struct netc_flower_rule *rule;

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		if (priv->dev_type == NETC_DEV_SWITCH) {
			if (rule->port_id == port_id && rule->cookie == cookie)
				return rule;
		} else {
			if (rule->cookie == cookie)
				return rule;
		}
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(netc_find_flower_rule_by_cookie);

static bool netc_flower_isit_key_matched(const struct isit_keye_data *key1,
					 const struct isit_keye_data *key2)
{
	if (memcmp(key1, key2, sizeof(*key1)))
		return false;

	return true;
}

static bool netc_flower_ipft_key_matched(const struct ipft_keye_data *key1,
					 const struct ipft_keye_data *key2)
{
	const void *key1_start = &key1->frm_attr_flags;
	const void *key2_start = &key2->frm_attr_flags;
	u32 size = sizeof(*key1) - 8;

	if (memcmp(key1_start, key2_start, size))
		return false;

	return true;
}

struct netc_flower_rule *
netc_find_flower_rule_by_key(struct ntmp_priv *priv,
			     enum netc_key_tbl_type tbl_type,
			     void *key)
{
	static struct netc_flower_key_tbl *key_tbl;
	struct netc_flower_rule *rule;

	hlist_for_each_entry(rule, &priv->flower_list, node) {
		key_tbl = rule->key_tbl;
		if (key_tbl->tbl_type != tbl_type)
			continue;

		if (tbl_type == FLOWER_KEY_TBL_ISIT &&
		    netc_flower_isit_key_matched(key, &key_tbl->isit_entry->keye))
			return rule;
		else if (tbl_type == FLOWER_KEY_TBL_IPFT &&
			 netc_flower_ipft_key_matched(key, &key_tbl->ipft_entry->keye))
			return rule;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(netc_find_flower_rule_by_key);

static int netc_psfp_flower_key_validate(struct ntmp_priv *priv,
					 struct isit_keye_data *keye, int prio,
					 struct netc_flower_key_tbl **key_tbl,
					 struct netlink_ext_ack *extack)
{
	struct netc_flower_rule *rule, *tmp_rule;
	struct netc_flower_key_tbl *tmp_tbl;

	/* Find the first rule with the same ISIT key */
	rule = netc_find_flower_rule_by_key(priv, FLOWER_KEY_TBL_ISIT, keye);
	if (!rule)
		return 0;

	if (rule->flower_type != FLOWER_TYPE_PSFP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot add new rule with different flower type");
		return -EINVAL;
	}

	if (prio < 0) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Rule conflicts with existing rules");
		return -EINVAL;
	}

	/* Unsupport if existing rule does not have ISFT entry */
	if (!rule->isft_entry) {
		NL_SET_ERR_MSG_MOD(extack,
				   "VLAN pbit in rule conflicts with existing rule");
		return -EINVAL;
	}

	/* If there are other rules using the same key, an error is returned */
	hlist_for_each_entry(tmp_rule, &priv->flower_list, node) {
		tmp_tbl = tmp_rule->key_tbl;
		if (tmp_tbl->tbl_type != FLOWER_KEY_TBL_ISIT)
			continue;

		if (!netc_flower_isit_key_matched(keye, &tmp_tbl->isit_entry->keye))
			continue;

		if (tmp_rule->isft_entry &&
		    tmp_rule->isft_entry->keye.pcp == prio) {
			NL_SET_ERR_MSG_MOD(extack,
					   "The same key has been used by existing rule");
			return -EINVAL;
		}
	}

	*key_tbl = rule->key_tbl;

	return 0;
}

static int netc_psfp_gate_entry_validate(struct ntmp_priv *priv,
					 struct flow_action_entry *gate_entry,
					 struct netlink_ext_ack *extack)
{
	u64 max_cycle_time;
	u32 num_gates;

	if (!gate_entry) {
		NL_SET_ERR_MSG_MOD(extack, "No gate entries");
		return -EINVAL;
	}

	num_gates = gate_entry->gate.num_entries;
	if (num_gates > NTMP_SGCLT_MAX_GE_NUM) {
		NL_SET_ERR_MSG_MOD(extack, "Gate number exceeds 256");
		return -EINVAL;
	}

	max_cycle_time = gate_entry->gate.cycletime + gate_entry->gate.cycletimeext;
	if (max_cycle_time > NTMP_SGIT_MAX_CT_PLUS_CT_EXT) {
		NL_SET_ERR_MSG_MOD(extack, "Max cycle time exceeds 0x3ffffff ns");
		return -EINVAL;
	}

	if (gate_entry->hw_index >= priv->caps.sgit_num_entries) {
		NL_SET_ERR_MSG_FMT_MOD(extack, "Gate hw index cannot exceed %u",
				       priv->caps.sgit_num_entries - 1);
		return -EINVAL;
	}

	if (test_and_set_bit(gate_entry->hw_index, priv->sgit_eid_bitmap)) {
		NL_SET_ERR_MSG_MOD(extack, "Current gate hw index has been used");
		return -EINVAL;
	}

	return 0;
}

int netc_police_entry_validate(struct ntmp_priv *priv,
			       const struct flow_action *action,
			       const struct flow_action_entry *police_entry,
			       struct netlink_ext_ack *extack)
{
	/* Police entry is not necessary for PSFP */
	if (!police_entry)
		return 0;

	if (police_entry->police.exceed.act_id != FLOW_ACTION_DROP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when exceed action is not drop");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.notexceed.act_id != FLOW_ACTION_PIPE &&
	    police_entry->police.notexceed.act_id != FLOW_ACTION_ACCEPT) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is not pipe or ok");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.notexceed.act_id == FLOW_ACTION_ACCEPT &&
	    !flow_action_is_last_entry(action, police_entry)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is ok, but action is not last");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.peakrate_bytes_ps ||
	    police_entry->police.avrate || police_entry->police.overhead) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when peakrate/avrate/overhead is configured");
		return -EOPNOTSUPP;
	}

	if (police_entry->police.rate_pkt_ps) {
		NL_SET_ERR_MSG_MOD(extack,
				   "QoS offload not support packets per second");
		return -EOPNOTSUPP;
	}

	if (police_entry->hw_index >= priv->caps.rpt_num_entries) {
		NL_SET_ERR_MSG_FMT_MOD(extack, "Police index cannot exceed %u",
				       priv->caps.rpt_num_entries - 1);
		return -EINVAL;
	}

	if (test_and_set_bit(police_entry->hw_index, priv->rpt_eid_bitmap)) {
		NL_SET_ERR_MSG_MOD(extack, "Current police hw index has been used");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_police_entry_validate);

static int netc_psfp_isit_keye_construct(struct flow_rule *rule, int port_index,
					 struct isit_keye_data *keye, int *prio,
					 struct netlink_ext_ack *extack)
{
	struct flow_match_eth_addrs addr_match = {0};
	struct flow_match_vlan vlan_match = {0};
	struct isit_psfp_frame_key *frame_key;
	u16 vlan;

	frame_key = (struct isit_psfp_frame_key *)keye->frame_key;
	/* For ENETC, the port_index should be 0 */
	keye->src_port_id = port_index;

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported, must include ETH_ADDRS");
		return -EINVAL;
	}

	flow_rule_match_eth_addrs(rule, &addr_match);
	if (!is_zero_ether_addr(addr_match.mask->dst) &&
	    !is_zero_ether_addr(addr_match.mask->src)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot match on both source and destination MAC");
		return -EINVAL;
	}

	if (!is_zero_ether_addr(addr_match.mask->dst)) {
		if (!is_broadcast_ether_addr(addr_match.mask->dst)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Masked matching on destination MAC not supported");
			return -EINVAL;
		}

		ether_addr_copy(frame_key->mac, addr_match.key->dst);
		keye->key_type = NTMP_ISIT_KEY_TYPE1_DMAC_VLAN;
	}

	if (!is_zero_ether_addr(addr_match.mask->src)) {
		if (!is_broadcast_ether_addr(addr_match.mask->src)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Masked matching on source MAC not supported");
			return -EINVAL;
		}

		ether_addr_copy(frame_key->mac, addr_match.key->src);
		keye->key_type = NTMP_ISIT_KEY_TYPE0_SMAC_VLAN;
	}

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_VLAN))
		return 0;

	flow_rule_match_vlan(rule, &vlan_match);
	if (vlan_match.mask->vlan_id) {
		if (vlan_match.mask->vlan_id != VLAN_VID_MASK) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Only full mask is supported for VLAN ID");
			return -EINVAL;
		}

		vlan = vlan_match.key->vlan_id;
		vlan |= BIT(15);
		frame_key->vlan_h = (vlan >> 8) & 0xff;
		frame_key->vlan_l = vlan & 0xff;
	}

	if (vlan_match.mask->vlan_priority) {
		if (vlan_match.mask->vlan_priority !=
		    (VLAN_PRIO_MASK >> VLAN_PRIO_SHIFT)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Only full mask is supported for VLAN priority");
			return -EINVAL;
		}

		*prio = vlan_match.key->vlan_priority;
	}

	return 0;
}

static void netc_psfp_gate_entry_config(struct ntmp_priv *priv,
					struct flow_action_entry *gate_entry,
					struct ntmp_sgit_entry *sgit_entry,
					struct ntmp_sgclt_entry *sgclt_entry)
{
	u32 cycle_time, cycle_time_ext, num_gates;
	u64 base_time = gate_entry->gate.basetime;
	int i;

	num_gates = gate_entry->gate.num_entries;
	cycle_time = gate_entry->gate.cycletime;
	cycle_time_ext = gate_entry->gate.cycletimeext;

	if (gate_entry->gate.prio >= 0) {
		sgit_entry->icfge.ipv = gate_entry->gate.prio;
		sgit_entry->icfge.oipv = 1;
	}

	if (priv->adjust_base_time)
		base_time = priv->adjust_base_time(priv, base_time,
						   gate_entry->gate.cycletime);

	sgit_entry->acfge.admin_base_time = cpu_to_le64(base_time);
	sgit_entry->acfge.admin_sgcl_eid = cpu_to_le32(sgclt_entry->entry_id);
	sgit_entry->acfge.admin_cycle_time_ext = cpu_to_le32(cycle_time_ext);
	sgit_entry->cfge.sdu_type = SDU_TYPE_MPDU;
	sgit_entry->icfge.gst = NTMP_STREAM_GATE_STATE_OPEN;

	sgclt_entry->cfge.cycle_time = cpu_to_le32(cycle_time);
	sgclt_entry->cfge.list_length = num_gates - 1;
	sgclt_entry->cfge.ext_gtst = NTMP_STREAM_GATE_STATE_OPEN;
	if (gate_entry->gate.prio >= 0) {
		sgclt_entry->cfge.ext_ipv = gate_entry->gate.prio;
		sgclt_entry->cfge.ext_oipv = 1;
	}

	for (i = 0; i < num_gates; i++) {
		struct action_gate_entry *from = &gate_entry->gate.entries[i];
		struct sgclt_ge *to = &sgclt_entry->cfge.ge[i];

		if (from->gate_state)
			to->gtst = NTMP_STREAM_GATE_STATE_OPEN;

		if (from->ipv >= 0) {
			to->oipv = 1;
			to->ipv = from->ipv & 0x7;
		}

		if (from->maxoctets >= 0) {
			to->iomen = 1;
			to->iom[0] = from->maxoctets & 0xff;
			to->iom[1] = (from->maxoctets >> 8) & 0xff;
			to->iom[2] = (from->maxoctets >> 16) & 0xff;
		}

		to->interval = cpu_to_le32(from->interval);
	}
}

void netc_rpt_entry_config(struct flow_action_entry *police_entry,
			   struct ntmp_rpt_entry *rpt_entry)
{
	u64 rate_bps;
	u32 cir, cbs;

	rpt_entry->entry_id = police_entry->hw_index;

	/* The unit of rate_bytes_ps is 1Bps, the uint of cir is 3.725bps,
	 * so convert it.
	 */
	rate_bps = police_entry->police.rate_bytes_ps * 8;
	cir = div_u64(rate_bps * 1000, 3725);
	cbs = police_entry->police.burst;
	rpt_entry->cfge.cir = cpu_to_le32(cir);
	rpt_entry->cfge.cbs = cpu_to_le32(cbs);
	rpt_entry->cfge.sdu_type = SDU_TYPE_MPDU;
	rpt_entry->fee.fen = 1;
}
EXPORT_SYMBOL_GPL(netc_rpt_entry_config);

static int netc_delete_sgclt_entry(struct ntmp_priv *priv, u32 entry_id)
{
	struct ntmp_sgclt_entry *sgclt_entry __free(kfree);
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	u32 max_data_size, max_cfge_size;
	u32 num_gates, entry_size;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return 0;

	max_cfge_size = struct_size_t(struct sgclt_cfge_data, ge,
				      NTMP_SGCLT_MAX_GE_NUM);
	max_data_size = struct_size(sgclt_entry, cfge.ge, NTMP_SGCLT_MAX_GE_NUM);
	sgclt_entry = kzalloc(max_data_size, GFP_KERNEL);
	if (!sgclt_entry)
		return -ENOMEM;

	err = ntmp_sgclt_query_entry(cbdrs, entry_id, sgclt_entry, max_cfge_size);
	if (err)
		return err;

	/* entry_size equals to 1 + ROUNDUP(N / 2) where N is number of gates */
	num_gates = sgclt_entry->cfge.list_length + 1;
	entry_size = 1 + DIV_ROUND_UP(num_gates, 2);
	err = ntmp_sgclt_delete_entry(cbdrs, entry_id);
	if (err)
		return err;

	ntmp_clear_words_bitmap(priv->sgclt_word_bitmap, entry_id, entry_size);

	return 0;
}

static int netc_delete_sgit_entry(struct ntmp_priv *priv, u32 entry_id)
{
	struct ntmp_sgit_entry *entry __free(kfree);
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	struct ntmp_sgit_entry new_entry = {0};
	u32 sgcl_eid;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return 0;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	/* Step1: Query the stream gate instance table entry to retrieve
	 * the entry id of the administrative gate control list and the
	 * opertational gate control list.
	 */
	err = ntmp_sgit_query_entry(cbdrs, entry_id, entry);
	if (err)
		return err;

	/* Step2: Update the stream gate instance table entry to set
	 * the entry id of the administrative gate control list to NULL.
	 */
	new_entry.acfge.admin_sgcl_eid = NTMP_NULL_ENTRY_ID;
	new_entry.entry_id = entry_id;
	err = ntmp_sgit_add_or_update_entry(cbdrs, &new_entry);
	if (err)
		return err;

	/* Step3: Delete the stream gate instance table entry. */
	err = ntmp_sgit_delete_entry(cbdrs, entry_id);
	if (err)
		return err;

	ntmp_clear_eid_bitmap(priv->sgit_eid_bitmap, entry_id);

	/* Step4: Delete the administrative gate control list
	 * and the operational gate control list.
	 */
	sgcl_eid = le32_to_cpu(entry->acfge.admin_sgcl_eid);
	err = netc_delete_sgclt_entry(priv, sgcl_eid);
	if (err)
		return err;

	sgcl_eid = le32_to_cpu(entry->sgise.oper_sgcl_eid);
	err = netc_delete_sgclt_entry(priv, sgcl_eid);

	return err;
}

static int netc_psfp_set_related_tables(struct ntmp_priv *priv,
					struct netc_psfp_tbl_entries *tbl)
{
	struct ntmp_sgclt_entry *sgclt_entry = tbl->sgclt_entry;
	struct ntmp_isit_entry *isit_entry = tbl->isit_entry;
	struct ntmp_isft_entry *isft_entry = tbl->isft_entry;
	struct ntmp_sgit_entry *sgit_entry = tbl->sgit_entry;
	struct ntmp_isct_entry *isct_entry = tbl->isct_entry;
	struct ntmp_ist_entry *ist_entry = tbl->ist_entry;
	struct ntmp_rpt_entry *rpt_entry = tbl->rpt_entry;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	int err;

	err = ntmp_isct_operate_entry(cbdrs, isct_entry->entry_id,
				      NTMP_CMD_ADD, NULL);
	if (err)
		return err;

	err = ntmp_sgclt_add_entry(cbdrs, sgclt_entry);
	if (err)
		goto delete_isct_entry;

	err = ntmp_sgit_add_or_update_entry(cbdrs, sgit_entry);
	if (err) {
		ntmp_sgclt_delete_entry(cbdrs, sgclt_entry->entry_id);
		goto delete_isct_entry;
	}

	if (rpt_entry) {
		err = ntmp_rpt_add_or_update_entry(cbdrs, rpt_entry);
		if (err)
			goto delete_sgit_entry;
	}

	if (ist_entry) {
		err = ntmp_ist_add_or_update_entry(cbdrs, ist_entry);
		if (err)
			goto delete_rpt_entry;
	}

	if (isft_entry) {
		err = ntmp_isft_add_or_update_entry(cbdrs, true, isft_entry);
		if (err)
			goto delete_ist_entry;
	}

	if (isit_entry) {
		err = ntmp_isit_add_or_update_entry(cbdrs, true, isit_entry);
		if (err)
			goto delete_isft_entry;
	}

	return 0;

delete_isft_entry:
	if (isft_entry)
		ntmp_isft_delete_entry(cbdrs, isft_entry->entry_id);

delete_ist_entry:
	if (ist_entry)
		ntmp_ist_delete_entry(cbdrs, ist_entry->entry_id);

delete_rpt_entry:
	if (rpt_entry)
		ntmp_rpt_delete_entry(cbdrs, rpt_entry->entry_id);

delete_sgit_entry:
	netc_delete_sgit_entry(priv, sgit_entry->entry_id);

delete_isct_entry:
	ntmp_isct_operate_entry(cbdrs, ist_entry->entry_id, NTMP_CMD_DELETE, NULL);

	return err;
}

void netc_init_ist_entry_eids(struct ntmp_priv *priv,
			      struct ntmp_ist_entry *ist_entry)
{
	struct ist_cfge_data *cfge = &ist_entry->cfge;

	cfge->rp_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	cfge->sgi_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	cfge->isc_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);

	if (priv->dev_type == NETC_DEV_SWITCH) {
		cfge->isqg_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
		cfge->ifm_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
		cfge->et_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	}
}
EXPORT_SYMBOL_GPL(netc_init_ist_entry_eids);

static int netc_add_psfp_key_tbl(struct ntmp_priv *priv,
				 struct netc_flower_key_tbl **key_tbl,
				 struct isit_keye_data *isit_key,
				 struct netlink_ext_ack *extack)
{
	struct netc_flower_key_tbl *new_tbl __free(kfree);
	struct ntmp_isit_entry *isit_entry __free(kfree);
	struct ntmp_ist_entry *ist_entry __free(kfree);
	struct ist_cfge_data *cfge;

	new_tbl = kzalloc(sizeof(*new_tbl), GFP_KERNEL);
	if (!new_tbl)
		return -ENOMEM;

	isit_entry = kzalloc(sizeof(*isit_entry), GFP_KERNEL);
	if (!isit_entry)
		return -ENOMEM;

	ist_entry = kzalloc(sizeof(*ist_entry), GFP_KERNEL);
	if (!ist_entry)
		return -ENOMEM;

	new_tbl->tbl_type = FLOWER_KEY_TBL_ISIT;
	refcount_set(&new_tbl->refcount, 1);

	ist_entry->entry_id = ntmp_lookup_free_eid(priv->ist_eid_bitmap,
						   priv->caps.ist_num_entries);
	if (ist_entry->entry_id == NTMP_NULL_ENTRY_ID) {
		NL_SET_ERR_MSG_MOD(extack, "No available IST entry is found");
		return -ENOSPC;
	}

	cfge = &ist_entry->cfge;
	switch (priv->cbdrs.tbl.ist_ver) {
	case NTMP_TBL_VER1:
		if (priv->dev_type == NETC_DEV_SWITCH)
			cfge->v1.fa = NTMP_IST_SWITCH_FA_BF;
		else
			cfge->v1.fa = NTMP_IST_FA_NO_SI_BITMAP;
		cfge->v1.sdu_type = SDU_TYPE_MPDU;
		break;
	case NTMP_TBL_VER0:
		if (priv->dev_type == NETC_DEV_SWITCH)
			cfge->fa = NTMP_IST_SWITCH_FA_BF;
		else
			cfge->fa = NTMP_IST_FA_NO_SI_BITMAP;
		cfge->sdu_type = SDU_TYPE_MPDU;
		break;
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unknown IST version");
		ntmp_clear_eid_bitmap(priv->ist_eid_bitmap,
				      ist_entry->entry_id);

		return -EINVAL;
	}

	netc_init_ist_entry_eids(priv, ist_entry);

	isit_entry->is_eid = cpu_to_le32(ist_entry->entry_id);
	isit_entry->keye = *isit_key;

	new_tbl->isit_entry = no_free_ptr(isit_entry);
	new_tbl->ist_entry = no_free_ptr(ist_entry);
	*key_tbl = no_free_ptr(new_tbl);

	return 0;
}

void netc_free_flower_key_tbl(struct ntmp_priv *priv,
			      struct netc_flower_key_tbl *key_tbl)
{
	struct ntmp_ist_entry *ist_entry;

	if (!key_tbl)
		return;

	ist_entry = key_tbl->ist_entry;
	if (ist_entry) {
		ntmp_clear_eid_bitmap(priv->ist_eid_bitmap, ist_entry->entry_id);
		kfree(key_tbl->ist_entry);
	}

	switch (key_tbl->tbl_type) {
	case FLOWER_KEY_TBL_ISIT:
		kfree(key_tbl->isit_entry);
		break;
	case FLOWER_KEY_TBL_IPFT:
		kfree(key_tbl->ipft_entry);
		break;
	}

	kfree(key_tbl);
}
EXPORT_SYMBOL_GPL(netc_free_flower_key_tbl);

int netc_setup_psfp(struct ntmp_priv *priv, int port_id,
		    struct flow_cls_offload *f)
{
	struct flow_action_entry *gate_entry = NULL, *police_entry = NULL;
	struct flow_rule *cls_rule = flow_cls_offload_flow_rule(f);
	struct ntmp_isft_entry *isft_entry __free(kfree) = NULL;
	struct ntmp_rpt_entry *rpt_entry __free(kfree) = NULL;
	struct netc_flower_rule *rule __free(kfree) = NULL;
	struct ntmp_sgclt_entry *sgclt_entry __free(kfree);
	struct netc_flower_key_tbl *reused_key_tbl = NULL;
	struct netlink_ext_ack *extack = f->common.extack;
	struct ntmp_sgit_entry *sgit_entry __free(kfree);
	struct ntmp_isct_entry *isct_entry __free(kfree);
	struct netc_flower_key_tbl *key_tbl = NULL;
	struct ntmp_ist_entry *ist_entry = NULL;
	struct flow_action_entry *action_entry;
	u32 sgclt_entry_size, sgclt_data_size;
	struct netc_psfp_tbl_entries psfp_tbl;
	struct isit_keye_data isit_keye = {0};
	struct ntmp_isit_entry *isit_entry;
	u32 ist_eid, sgclt_eid, isct_eid;
	unsigned long cookie = f->cookie;
	int i, err, priority = -1;
	u32 num_gates;
	u16 msdu = 0;

	guard(mutex)(&priv->flower_lock);
	if (netc_find_flower_rule_by_cookie(priv, port_id, cookie)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot add new rule with same cookie");
		return -EINVAL;
	}

	rule = kzalloc(sizeof(*rule), GFP_KERNEL);
	if (!rule)
		return -ENOMEM;

	rule->port_id = port_id;
	rule->cookie = cookie;
	rule->flower_type = FLOWER_TYPE_PSFP;

	/* Find gate action entry and police action entry*/
	flow_action_for_each(i, action_entry, &cls_rule->action)
		if (action_entry->id == FLOW_ACTION_GATE)
			gate_entry = action_entry;
		else if (action_entry->id == FLOW_ACTION_POLICE)
			police_entry = action_entry;

	err = netc_psfp_gate_entry_validate(priv, gate_entry, extack);
	if (err)
		return err;

	err = netc_police_entry_validate(priv, &cls_rule->action,
					 police_entry, extack);
	if (err)
		goto clear_sgit_eid_bit;

	err = netc_psfp_isit_keye_construct(cls_rule, port_id, &isit_keye,
					    &priority, extack);
	if (err)
		goto clear_rpt_eid_bit;

	err = netc_psfp_flower_key_validate(priv, &isit_keye, priority,
					    &reused_key_tbl, extack);
	if (err)
		goto clear_rpt_eid_bit;

	if (!reused_key_tbl) {
		err = netc_add_psfp_key_tbl(priv, &key_tbl, &isit_keye, extack);
		if (err)
			goto clear_rpt_eid_bit;

		isit_entry = key_tbl->isit_entry;
		ist_entry = key_tbl->ist_entry;
		ist_eid = ist_entry->entry_id;
	} else {
		ist_eid = reused_key_tbl->ist_entry->entry_id;
	}

	if (police_entry) {
		msdu = police_entry->police.mtu;

		if (police_entry->police.rate_bytes_ps ||
		    police_entry->police.burst) {
			rpt_entry = kzalloc(sizeof(*rpt_entry), GFP_KERNEL);
			if (!rpt_entry) {
				err = -ENOMEM;
				goto free_psfp_key_tbl;
			}

			netc_rpt_entry_config(police_entry, rpt_entry);
		}
	}

	sgit_entry = kzalloc(sizeof(*sgit_entry), GFP_KERNEL);
	if (!sgit_entry) {
		err = -ENOMEM;
		goto free_psfp_key_tbl;
	}

	sgit_entry->entry_id = gate_entry->hw_index;
	num_gates = gate_entry->gate.num_entries;
	sgclt_entry_size = 1 + DIV_ROUND_UP(num_gates, 2);
	sgclt_eid = ntmp_lookup_free_words(priv->sgclt_word_bitmap,
					   priv->caps.sgclt_num_words,
					   sgclt_entry_size);
	if (sgclt_eid == NTMP_NULL_ENTRY_ID) {
		NL_SET_ERR_MSG_MOD(extack, "No Stream Gate Control List resource");
		err = -ENOSPC;
		goto free_psfp_key_tbl;
	}

	sgclt_data_size = struct_size(sgclt_entry, cfge.ge, num_gates);
	sgclt_entry = kzalloc(sgclt_data_size, GFP_KERNEL);
	if (!sgclt_entry) {
		err = -ENOMEM;
		goto clear_sgclt_eid_words;
	}

	sgclt_entry->entry_id = sgclt_eid;
	rule->sgclt_eid = sgclt_eid;
	netc_psfp_gate_entry_config(priv, gate_entry, sgit_entry, sgclt_entry);

	isct_eid = ntmp_lookup_free_eid(priv->isct_eid_bitmap,
					priv->caps.isct_num_entries);
	if (isct_eid == NTMP_NULL_ENTRY_ID) {
		NL_SET_ERR_MSG_MOD(extack, "No available ISCT entry is found");
		err = -ENOSPC;
		goto clear_sgclt_eid_words;
	}

	isct_entry = kzalloc(sizeof(*isct_entry), GFP_KERNEL);
	if (!isct_entry) {
		err = -ENOMEM;
		goto clear_isct_eid_bit;
	}

	isct_entry->entry_id = isct_eid;

	/* Determine if an ingress stream filter entry is required */
	if (priority >= 0) {
		isft_entry = kzalloc(sizeof(*isft_entry), GFP_KERNEL);
		if (!isft_entry) {
			err = -ENOMEM;
			goto clear_isct_eid_bit;
		}

		isft_entry->keye.is_eid = cpu_to_le32(ist_eid);
		isft_entry->keye.pcp = priority;
		isft_entry->cfge.sdu_type = SDU_TYPE_MPDU;
		isft_entry->cfge.msdu = cpu_to_le16(msdu);
		isft_entry->cfge.isc_eid = cpu_to_le32(isct_eid);
		isft_entry->cfge.sgi_eid = cpu_to_le32(sgit_entry->entry_id);
		isft_entry->cfge.rp_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
		isft_entry->cfge.osgi = 1;

		if (rpt_entry) {
			isft_entry->cfge.orp = 1;
			isft_entry->cfge.rp_eid = cpu_to_le32(rpt_entry->entry_id);
		}

		if (ist_entry)
			ist_entry->cfge.sfe = 1; /* Enable stream filter */
	} else if (ist_entry) {
		ist_entry->cfge.msdu = cpu_to_le16(msdu);
		ist_entry->cfge.osgi = 1;
		ist_entry->cfge.isc_eid = cpu_to_le32(isct_eid);
		ist_entry->cfge.sgi_eid = cpu_to_le32(sgit_entry->entry_id);

		if (rpt_entry) {
			ist_entry->cfge.orp = 1;
			ist_entry->cfge.rp_eid = cpu_to_le32(rpt_entry->entry_id);
		}
	}

	psfp_tbl.ist_entry = ist_entry;
	psfp_tbl.rpt_entry = rpt_entry;
	psfp_tbl.isit_entry = isit_entry;
	psfp_tbl.isft_entry = isft_entry;
	psfp_tbl.sgit_entry = sgit_entry;
	psfp_tbl.isct_entry = isct_entry;
	psfp_tbl.sgclt_entry = sgclt_entry;
	err = netc_psfp_set_related_tables(priv, &psfp_tbl);
	if (err)
		goto clear_isct_eid_bit;

	rule->lastused = jiffies;
	rule->isft_entry = no_free_ptr(isft_entry);

	if (reused_key_tbl) {
		rule->key_tbl = reused_key_tbl;
		refcount_inc(&reused_key_tbl->refcount);
	} else {
		rule->key_tbl = key_tbl;
	}

	hlist_add_head(&no_free_ptr(rule)->node, &priv->flower_list);

	return 0;

clear_isct_eid_bit:
	ntmp_clear_eid_bitmap(priv->isct_eid_bitmap, isct_eid);

clear_sgclt_eid_words:
	ntmp_clear_words_bitmap(priv->sgclt_word_bitmap, sgclt_eid,
				sgclt_entry_size);

free_psfp_key_tbl:
	netc_free_flower_key_tbl(priv, key_tbl);

clear_rpt_eid_bit:
	if (police_entry)
		ntmp_clear_eid_bitmap(priv->rpt_eid_bitmap,
				      police_entry->hw_index);

clear_sgit_eid_bit:
	ntmp_clear_eid_bitmap(priv->sgit_eid_bitmap, gate_entry->hw_index);

	return err;
}
EXPORT_SYMBOL_GPL(netc_setup_psfp);

void netc_delete_psfp_flower_rule(struct ntmp_priv *priv,
				  struct netc_flower_rule *rule)
{
	struct ntmp_isft_entry *isft_entry = rule->isft_entry;
	struct netc_flower_key_tbl *key_tbl = rule->key_tbl;
	struct netc_cbdrs *cbdrs = &priv->cbdrs;
	struct ntmp_isit_entry *isit_entry;
	struct ntmp_ist_entry *ist_entry;
	u32 rpt_eid, isct_eid, sgit_eid;

	if (refcount_dec_and_test(&key_tbl->refcount)) {
		isit_entry = key_tbl->isit_entry;
		ist_entry = key_tbl->ist_entry;
		ntmp_isit_delete_entry(cbdrs, isit_entry->entry_id);
		ntmp_ist_delete_entry(cbdrs, ist_entry->entry_id);

		rpt_eid = le32_to_cpu(ist_entry->cfge.rp_eid);
		isct_eid = le32_to_cpu(ist_entry->cfge.isc_eid);
		sgit_eid = le32_to_cpu(ist_entry->cfge.sgi_eid);

		netc_free_flower_key_tbl(priv, key_tbl);
	}

	if (isft_entry) {
		rpt_eid = le32_to_cpu(isft_entry->cfge.rp_eid);
		isct_eid = le32_to_cpu(isft_entry->cfge.isc_eid);
		sgit_eid = le32_to_cpu(isft_entry->cfge.sgi_eid);

		ntmp_isft_delete_entry(cbdrs, isft_entry->entry_id);
		kfree(isft_entry);
	}

	ntmp_isct_operate_entry(cbdrs, isct_eid, NTMP_CMD_DELETE, NULL);
	ntmp_clear_eid_bitmap(priv->isct_eid_bitmap, isct_eid);
	netc_delete_sgit_entry(priv, sgit_eid);
	ntmp_clear_eid_bitmap(priv->sgit_eid_bitmap, sgit_eid);
	if (rpt_eid != NTMP_NULL_ENTRY_ID) {
		ntmp_rpt_delete_entry(cbdrs, rpt_eid);
		ntmp_clear_eid_bitmap(priv->rpt_eid_bitmap, rpt_eid);
	}

	hlist_del(&rule->node);
	kfree(rule);
}
EXPORT_SYMBOL_GPL(netc_delete_psfp_flower_rule);

int netc_psfp_flower_stat(struct ntmp_priv *priv, struct netc_flower_rule *rule,
			  u64 *byte_cnt, u64 *pkt_cnt, u64 *drop_cnt)
{
	struct ntmp_ist_entry *ist_entry = rule->key_tbl->ist_entry;
	struct ntmp_isft_entry *isft_entry = rule->isft_entry;
	struct isct_stse_data stse = {0};
	u32 isct_eid, sg_drop_cnt;
	int err;

	if (isft_entry)
		isct_eid = le32_to_cpu(isft_entry->cfge.isc_eid);
	else
		isct_eid = le32_to_cpu(ist_entry->cfge.isc_eid);

	/* Query, followed by update will reset statistics */
	err = ntmp_isct_operate_entry(&priv->cbdrs, isct_eid,
				      NTMP_CMD_QU, &stse);
	if (err)
		return err;

	sg_drop_cnt = le32_to_cpu(stse.sg_drop_count);
	/* Workaround for ERR052134 on i.MX95 platform */
	if (priv->errata & NTMP_ERR052134) {
		u32 tmp;

		sg_drop_cnt >>= 9;

		tmp = le32_to_cpu(stse.resv3) & 0x1ff;
		sg_drop_cnt |= (tmp << 23);
	}

	*pkt_cnt = le32_to_cpu(stse.rx_count);
	*drop_cnt = le32_to_cpu(stse.msdu_drop_count) + sg_drop_cnt +
		    le32_to_cpu(stse.policer_drop_count);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_psfp_flower_stat);

int netc_setup_taprio(struct ntmp_priv *priv, u32 entry_id,
		      struct tc_taprio_qopt_offload *f)
{
	struct tgst_cfge_data *cfge __free(kfree) = NULL;
	struct netlink_ext_ack *extack = f->extack;
	u64 base_time = f->base_time;
	u64 max_cycle_time;
	int i, err;
	u32 size;

	if (!priv->get_tgst_free_words) {
		NL_SET_ERR_MSG_MOD(extack, "get_tgst_free_words() is undefined");
		return -EINVAL;
	}

	max_cycle_time = f->cycle_time + f->cycle_time_extension;
	if (max_cycle_time > U32_MAX) {
		NL_SET_ERR_MSG_MOD(extack, "Max cycle time exceeds U32_MAX");
		return -EINVAL;
	}

	/* Delete the pending administrative control list if it exists */
	err = ntmp_tgst_delete_admin_gate_list(&priv->cbdrs, entry_id);
	if (err)
		return err;

	if (f->num_entries > priv->get_tgst_free_words(priv)) {
		NL_SET_ERR_MSG_MOD(extack, "TGST doesn't have enough free words");
		return -EINVAL;
	}

	size = struct_size(cfge, ge, f->num_entries);
	cfge = kzalloc(size, GFP_KERNEL);
	if (!cfge)
		return -ENOMEM;

	if (priv->adjust_base_time)
		base_time = priv->adjust_base_time(priv, base_time, f->cycle_time);

	cfge->admin_bt = cpu_to_le64(base_time);
	cfge->admin_ct = cpu_to_le32(f->cycle_time);
	cfge->admin_ct_ext = cpu_to_le32(f->cycle_time_extension);
	cfge->admin_cl_len = cpu_to_le16(f->num_entries);
	for (i = 0; i < f->num_entries; i++) {
		struct tc_taprio_sched_entry *temp_entry = &f->entries[i];

		cfge->ge[i].tc_state = temp_entry->gate_mask;
		cfge->ge[i].interval = cpu_to_le32(temp_entry->interval);
	}

	err = ntmp_tgst_update_admin_gate_list(&priv->cbdrs, entry_id, cfge);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Update control list failed");
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_setup_taprio);
