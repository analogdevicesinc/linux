// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2024 NXP
 */
#include <linux/fsl/netc_lib.h>

int netc_kstrtouint(const char __user *buffer, size_t count,
		    loff_t *ppos, u32 *val)
{
	char cmd_buffer[256];
	int len, err;

	if (*ppos != 0 || !count)
		return -EINVAL;

	if (count >= sizeof(cmd_buffer))
		return -ENOSPC;

	len = simple_write_to_buffer(cmd_buffer, sizeof(cmd_buffer) - 1,
				     ppos, buffer, count);
	if (len < 0)
		return len;

	cmd_buffer[len] = '\0';
	err = kstrtouint(cmd_buffer, 16, val);
	if (err)
		return err;

	return len;
}
EXPORT_SYMBOL_GPL(netc_kstrtouint);

void netc_show_psfp_flower(struct seq_file *s, struct netc_flower_rule *rule)
{
	struct ntmp_isit_entry *isit_entry = rule->key_tbl->isit_entry;
	struct ntmp_ist_entry *ist_entry = rule->key_tbl->ist_entry;
	struct ntmp_isft_entry *isft_entry = rule->isft_entry;
	u32 rpt_eid, sgit_eid, isct_eid;

	seq_printf(s, "ISIT entry ID:0x%x\n", isit_entry->entry_id);
	seq_printf(s, "IST entry ID: 0x%x\n", ist_entry->entry_id);

	if (isft_entry) {
		rpt_eid = le32_to_cpu(isft_entry->cfge.rp_eid);
		isct_eid = le32_to_cpu(isft_entry->cfge.isc_eid);
		sgit_eid = le32_to_cpu(isft_entry->cfge.sgi_eid);
		seq_printf(s, "ISFT entry ID: 0x%x\n", isft_entry->entry_id);
	} else {
		rpt_eid = le32_to_cpu(ist_entry->cfge.rp_eid);
		isct_eid = le32_to_cpu(ist_entry->cfge.isc_eid);
		sgit_eid = le32_to_cpu(ist_entry->cfge.sgi_eid);
	}

	seq_printf(s, "RPT entry ID: 0x%x\n", rpt_eid);
	seq_printf(s, "SGIT entry ID: 0x%x\n", sgit_eid);
	seq_printf(s, "ISCT entry ID: 0x%x\n", isct_eid);
	seq_printf(s, "SGCLT entry ID: 0x%x\n", rule->sgclt_eid);
}
EXPORT_SYMBOL_GPL(netc_show_psfp_flower);

int netc_show_isit_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_isit_entry *isit_entry __free(kfree);
	struct isit_keye_data *keye;
	int i, err;

	isit_entry = kzalloc(sizeof(*isit_entry), GFP_KERNEL);
	if (!isit_entry)
		return -ENOMEM;

	err = ntmp_isit_query_entry(&priv->cbdrs, entry_id, isit_entry);
	if (err) {
		seq_printf(s, "Query ISIT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	keye = &isit_entry->keye;
	seq_printf(s, "Show ingress stream identification table entry 0x%x\n",
		   entry_id);
	seq_printf(s, "Key type: %u, Source Port ID: %u, IS_EID: %u\n",
		   keye->key_type, keye->src_port_id,
		   le32_to_cpu(isit_entry->is_eid));
	seq_puts(s, "Keys: ");
	for (i = 0; i < NTMP_ISIT_FRAME_KEY_LEN; i++)
		seq_printf(s, "%02x", keye->frame_key[i]);
	seq_puts(s, "\n\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_isit_entry);

int netc_show_ist_entry(struct ntmp_priv *priv, struct seq_file *s,
			u32 entry_id)
{
	struct ist_cfge_data *cfge __free(kfree);
	union ist_switch_cfg switch_cfg;
	u32 bitmap_evmeid;
	int err;

	cfge = kzalloc(sizeof(*cfge), GFP_KERNEL);
	if (!cfge)
		return -ENOMEM;

	err = ntmp_ist_query_entry(&priv->cbdrs, entry_id, cfge);
	if (err) {
		seq_printf(s, "Query IST entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	switch_cfg.val = le16_to_cpu(cfge->switch_cfg);
	bitmap_evmeid = le32_to_cpu(cfge->bitmap_evmeid);
	seq_printf(s, "Show ingress stream table entry 0x%x\n", entry_id);
	seq_printf(s, "Stream Filtering: %s, Report Receive Timestamp: %s\n",
		   is_en(cfge->sfe), is_en(cfge->rrt));
	seq_printf(s, "OIPV: %s, IPV: %u, ODR: %s, DR: %u\n", is_en(cfge->oipv),
		   cfge->ipv, is_en(cfge->odr), cfge->dr);
	seq_printf(s, "IMIRE: %s, TIMECAPE: %s, SPPD: %u, ISQGA: %u\n",
		   is_en(cfge->imire), is_en(cfge->timecape), cfge->sppd,
		   cfge->isqga);
	seq_printf(s, "ORP: %s, OSGI: %s, SDU type:%u\n", is_en(cfge->orp),
		   is_en(cfge->osgi), cfge->v1.sdu_type);
	seq_printf(s, "Host Reason: %u, Forwarding Action: %u\n", cfge->hr,
		   cfge->v1.fa);
	seq_printf(s, "OSDFA: %s, SDFA: %u, MSDU:%u\n", is_en(cfge->v1.osdfa),
		   cfge->v1.sdfa, le16_to_cpu(cfge->msdu));
	seq_printf(s, "IFME_LEN_CHANGE: 0x%x, Egress Port: %u\n",
		   switch_cfg.ifme_len_change, switch_cfg.eport);
	seq_printf(s, "Override ET_EID: %u, CTD: %u\n",
		   switch_cfg.oeteid, switch_cfg.ctd);
	seq_printf(s, "ISQG_EID: 0x%x, RP_EID: 0x%x\n", le32_to_cpu(cfge->isqg_eid),
		   le32_to_cpu(cfge->rp_eid));
	seq_printf(s, "SGI_EID: 0x%x, IFM_EID: 0x%x\n", le32_to_cpu(cfge->sgi_eid),
		   le32_to_cpu(cfge->ifm_eid));
	seq_printf(s, "ET_EID: 0x%x, ISC_EID: 0x%x\n", le32_to_cpu(cfge->et_eid),
		   le32_to_cpu(cfge->isc_eid));
	seq_printf(s, "Egress Port bitmap: 0x%x, Event Monitor Event ID: %u\n",
		   bitmap_evmeid & 0xffffff, (bitmap_evmeid >> 24) & 0xf);
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_ist_entry);

int netc_show_isft_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_isft_entry *isft_entry __free(kfree);
	struct isft_cfge_data *cfge;
	struct isft_keye_data *keye;
	int err;

	isft_entry = kzalloc(sizeof(*isft_entry), GFP_KERNEL);
	if (!isft_entry)
		return -ENOMEM;

	keye = &isft_entry->keye;
	cfge = &isft_entry->cfge;
	err = ntmp_isft_query_entry(&priv->cbdrs, entry_id, isft_entry);
	if (err) {
		seq_printf(s, "Query ISFT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	seq_printf(s, "Show ingress stream filter table entry 0x%x\n", entry_id);
	seq_printf(s, "IS_EID: 0x%x, PCP: %u\n",
		   le32_to_cpu(keye->is_eid), keye->pcp);
	seq_printf(s, "OIPV: %s, IPV: %u, ODR: %s, DR: %u\n", is_en(cfge->oipv),
		   cfge->ipv, is_en(cfge->odr), cfge->dr);
	seq_printf(s, "IMIRE: %s, TIMECAPE:%s, OSGI: %s, CTD: %u\n",
		   is_en(cfge->imire), is_en(cfge->timecape), is_en(cfge->osgi),
		   cfge->ctd);
	seq_printf(s, "ORP: %s, SDU type: %u, MSDU: %u\n", is_en(cfge->orp),
		   cfge->sdu_type, le16_to_cpu(cfge->msdu));
	seq_printf(s, "RP_EID: 0x%x, SGI_EID: 0x%x, ISC_EID: 0x%x\n",
		   le32_to_cpu(cfge->rp_eid), le32_to_cpu(cfge->sgi_eid),
		   le32_to_cpu(cfge->isc_eid));
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_isft_entry);

int netc_show_sgit_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_sgit_entry *sgit_entry __free(kfree);
	struct sgit_acfge_data *acfge;
	struct sgit_icfge_data *icfge;
	struct sgit_sgise_data *sgise;
	struct sgit_cfge_data *cfge;
	int err;

	sgit_entry = kzalloc(sizeof(*sgit_entry), GFP_KERNEL);
	if (!sgit_entry)
		return -ENOMEM;

	err = ntmp_sgit_query_entry(&priv->cbdrs, entry_id, sgit_entry);
	if (err) {
		seq_printf(s, "Query SGIT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	acfge = &sgit_entry->acfge;
	icfge = &sgit_entry->icfge;
	sgise = &sgit_entry->sgise;
	cfge = &sgit_entry->cfge;
	seq_printf(s, "Show stream gate instance table entry 0x%x\n", entry_id);
	seq_printf(s, "OPER_SGCL_EID: 0x%x, CONFIG_CHANGE_TIME: %llu\n",
		   le32_to_cpu(sgise->oper_sgcl_eid),
		   le64_to_cpu(sgise->config_change_time));
	seq_printf(s, "OPER_BASE_TIME: %llu, OPER_CYCLE_TIME_EXT: %u\n",
		   le64_to_cpu(sgise->oper_base_time),
		   le32_to_cpu(sgise->oper_cycle_time_ext));
	seq_printf(s, "OEX: %u, IRX: %u, state: %u\n", sgise->oex, sgise->irx,
		   sgise->state);
	seq_printf(s, "OEXEN: %s, IRXEN: %s, SDU type:%u\n", is_en(cfge->oexen),
		   is_en(cfge->irxen), cfge->sdu_type);
	seq_printf(s, "OIPV: %s, IPV: %u, GST: %u, CTD: %u\n", is_en(icfge->oipv),
		   icfge->ipv, icfge->gst, icfge->ctd);
	seq_printf(s, "ADMIN_SGCL_EID: 0x%x, ADMIN_BASE_TIME: %llu\n",
		   le32_to_cpu(acfge->admin_sgcl_eid),
		   le64_to_cpu(acfge->admin_base_time));
	seq_printf(s, "ADMIN_CYCLE_TIME_EXT: %u\n",
		   le32_to_cpu(acfge->admin_cycle_time_ext));
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_sgit_entry);

int netc_show_sgclt_entry(struct ntmp_priv *priv, struct seq_file *s,
			  u32 entry_id)
{
	struct ntmp_sgclt_entry *sgclt_entry __free(kfree);
	u32 sgclt_data_size, sgclt_cfge_size;
	struct sgclt_cfge_data *cfge;
	int i, err;
	u32 iom;

	sgclt_cfge_size = struct_size_t(struct sgclt_cfge_data, ge,
					NTMP_SGCLT_MAX_GE_NUM);
	sgclt_data_size = struct_size(sgclt_entry, cfge.ge,
				      NTMP_SGCLT_MAX_GE_NUM);
	sgclt_entry = kzalloc(sgclt_data_size, GFP_KERNEL);
	if (!sgclt_entry)
		return -ENOMEM;

	err = ntmp_sgclt_query_entry(&priv->cbdrs, entry_id, sgclt_entry,
				     sgclt_cfge_size);
	if (err) {
		seq_printf(s, "Query SGCLT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	cfge = &sgclt_entry->cfge;
	seq_printf(s, "Show stream gate control list table entry 0x%x\n", entry_id);
	seq_printf(s, "REF_COUNT: %u, CYCLE_TIME: %u, LIST_LENGTH: %u\n",
		   sgclt_entry->ref_count, le32_to_cpu(cfge->cycle_time),
		   cfge->list_length);
	seq_printf(s, "EXT_OIPV: %s, EXT_IPV: %u, EXT_CTD: %u, EXT_GTST: %u\n",
		   is_en(cfge->ext_oipv), cfge->ext_ipv, cfge->ext_ctd, cfge->ext_gtst);

	for (i = 0; i < cfge->list_length + 1; i++) {
		iom = (u32)cfge->ge[i].iom[2] << 16;
		iom |= (u32)cfge->ge[i].iom[1] << 8;
		iom |= (u32)cfge->ge[i].iom[0];
		seq_printf(s, "Gate Entry: %d, Time Interval: %u, IOMEN: %s, IOM: %u\n",
			   i, le32_to_cpu(cfge->ge[i].interval),
			   is_en(cfge->ge[i].iomen), iom);
		seq_printf(s, "OIPV: %s, IPV: %u, CTD: %u, GTST: %u\n",
			   is_en(cfge->ge[i].oipv), cfge->ge[i].ipv,
			   cfge->ge[i].ctd, cfge->ge[i].gtst);
	}
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_sgclt_entry);

int netc_show_isct_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct isct_stse_data *stse __free(kfree);
	u32 sg_drop_cnt;
	int err;

	stse = kzalloc(sizeof(*stse), GFP_KERNEL);
	if (!stse)
		return -ENOMEM;

	err = ntmp_isct_operate_entry(&priv->cbdrs, entry_id,
				      NTMP_CMD_QUERY, stse);
	if (err) {
		seq_printf(s, "Query ISCT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	sg_drop_cnt = le32_to_cpu(stse->sg_drop_count);
	/* Workaround for ERR052134 on i.MX95 platform */
	if (priv->errata & NTMP_ERR052134) {
		u32 tmp;

		sg_drop_cnt >>= 9;

		tmp = le32_to_cpu(stse->resv3) & 0x1ff;
		sg_drop_cnt |= (tmp << 23);
	}

	seq_printf(s, "Show ingress stream count table entry 0x%x\n", entry_id);
	seq_printf(s, "RX_COUNT: %u, MSDU_DROP_COUNT: %u\n",
		   le32_to_cpu(stse->rx_count), le32_to_cpu(stse->msdu_drop_count));
	seq_printf(s, "POLICER_DROP_COUNT: %u, SG_DROP_COUNT: %u\n",
		   le32_to_cpu(stse->policer_drop_count), sg_drop_cnt);
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_isct_entry);

int netc_show_rpt_entry(struct ntmp_priv *priv, struct seq_file *s,
			u32 entry_id)
{
	struct ntmp_rpt_entry *rpt_entry __free(kfree);
	struct rpt_cfge_data *cfge;
	struct rpt_stse_data *stse;
	u32 bcf_bcs, bef_bes;
	int err;

	rpt_entry = kzalloc(sizeof(*rpt_entry), GFP_KERNEL);
	if (!rpt_entry)
		return -ENOMEM;

	err = ntmp_rpt_query_entry(&priv->cbdrs, entry_id, rpt_entry);
	if (err) {
		seq_printf(s, "Query RPT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	cfge = &rpt_entry->cfge;
	stse = &rpt_entry->stse;
	bcf_bcs = le32_to_cpu(stse->bcf_bcs);
	bef_bes = le32_to_cpu(stse->bef_bes);
	seq_printf(s, "Show rate policer table entry 0x%x\n", entry_id);
	seq_printf(s, "BYTE_COUNT: %llu, DROP_FRAMES: %u\n",
		   le64_to_cpu(stse->byte_count), le32_to_cpu(stse->drop_frames));
	seq_printf(s, "DR0_GRN_FRAMES: %u, DR1_GRN_FRAMES: %u\n",
		   le32_to_cpu(stse->dr0_grn_frames),
		   le32_to_cpu(stse->dr1_grn_frames));
	seq_printf(s, "DR2_YLW_FRAMES: %u, REMARK_YLW_FRAMES: %u\n",
		   le32_to_cpu(stse->dr2_ylw_frames),
		   le32_to_cpu(stse->remark_ylw_frames));
	seq_printf(s, "DR3_RED_FRAMES: %u, REMARK_RED_FRAMES: %u\n",
		   le32_to_cpu(stse->dr3_red_frames),
		   le32_to_cpu(stse->remark_red_frames));
	seq_printf(s, "LTS: 0x%x, BCI: %u, BEI: %u\n", le32_to_cpu(stse->lts),
		   le32_to_cpu(stse->bci), le32_to_cpu(stse->bei));
	seq_printf(s, "BCS: %u, BCF: 0x%x\n", bcf_bcs >> 31, bcf_bcs & 0x7fffffff);
	seq_printf(s, "BEF: %u, BEI: 0x%x\n", bef_bes >> 31, bef_bes & 0x7fffffff);
	seq_printf(s, "CIR: %u, CBS: %u, EIR: %u, EBS: %u\n",
		   le32_to_cpu(cfge->cir), le32_to_cpu(cfge->cbs),
		   le32_to_cpu(cfge->eir), le32_to_cpu(cfge->ebs));
	seq_printf(s, "MREN: %s, DOY: %s, CM: %u, CF: %u\n",
		   is_en(cfge->mren), is_en(cfge->doy), cfge->cm, cfge->cf);
	seq_printf(s, "NDOR: %s, SDU type:%u, FEN: %s, MR: %u\n", is_en(cfge->ndor),
		   cfge->sdu_type, is_en(rpt_entry->fee.fen), rpt_entry->pse.mr);
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_rpt_entry);

int netc_show_ipft_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_ipft_entry *ipft_entry __free(kfree);
	union ipft_src_port src_port;
	struct ipft_keye_data *keye;
	struct ipft_cfge_data *cfge;
	u16 dscp, dscp_mask;
	int i, err;
	u8 rpr;

	ipft_entry = kzalloc(sizeof(*ipft_entry), GFP_KERNEL);
	if (!ipft_entry)
		return -ENOMEM;

	err = ntmp_ipft_query_entry(&priv->cbdrs, entry_id, false, ipft_entry);
	if (err)
		return err;

	keye = &ipft_entry->keye;
	cfge = &ipft_entry->cfge;

	dscp = le16_to_cpu(keye->dscp) & NTMP_IPFT_DSCP;
	dscp_mask = (le16_to_cpu(keye->dscp) & NTMP_IPFT_DSCP_MASK) >> 6;
	rpr = cfge->rpr_l;
	rpr |= cfge->rpr_h << 1;

	seq_printf(s, "Show ingress port filter table entry:%u\n", entry_id);

	/* KEYE_DATA */
	seq_printf(s, "Precedence:%u, Frame attribute flags:0x%04x, mask:0x%04x\n",
		   keye->precedence, keye->frm_attr_flags, keye->frm_attr_flags_mask);
	seq_printf(s, "DSCP:0x%x, mask:0x%x\n", dscp, dscp_mask);

	if (priv->dev_type == NETC_DEV_SWITCH) {
		src_port.val = le16_to_cpu(keye->src_port);
		seq_printf(s, "Switch Source Port ID:%d, mask:0x%02x\n",
			   src_port.id, src_port.mask);
	}

	seq_printf(s, "Outer VLAN TCI:0x%04x, mask:0x%04x\n",
		   ntohs(keye->outer_vlan_tci), ntohs(keye->outer_vlan_tci_mask));
	seq_printf(s, "Inner VLAN TCI:0x%04x, mask:0x%04x\n",
		   ntohs(keye->inner_vlan_tci), ntohs(keye->inner_vlan_tci_mask));
	seq_printf(s, "Destination MAC:%pM\n", keye->dmac);
	seq_printf(s, "Destination MAC mask:%pM\n", keye->dmac_mask);
	seq_printf(s, "Source MAC:%pM\n", keye->smac);
	seq_printf(s, "Source MAC mask:%pM\n", keye->smac_mask);
	seq_printf(s, "Ether Type:0x%04x, mask:0x%04x\n", ntohs(keye->ethertype),
		   ntohs(keye->ethertype_mask));
	seq_printf(s, "IP protocol:%u, mask:0x%02x\n",  keye->ip_protocol,
		   keye->ip_protocol_mask);
	seq_printf(s, "IP Source Address:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_src[0]), ntohl(keye->ip_src[1]),
		   ntohl(keye->ip_src[2]), ntohl(keye->ip_src[3]));
	seq_printf(s, "IP Source Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_src_mask[0]), ntohl(keye->ip_src_mask[1]),
		   ntohl(keye->ip_src_mask[2]), ntohl(keye->ip_src_mask[3]));
	seq_printf(s, "IP Destination Address:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_dst[0]), ntohl(keye->ip_dst[1]),
		   ntohl(keye->ip_dst[2]), ntohl(keye->ip_dst[3]));
	seq_printf(s, "IP Destination Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_dst_mask[0]), ntohl(keye->ip_dst_mask[1]),
		   ntohl(keye->ip_dst_mask[2]), ntohl(keye->ip_dst_mask[3]));
	seq_printf(s, "L4 Source Port:%x, mask:0x%04x\n",
		   ntohs(keye->l4_src_port), ntohs(keye->l4_src_port_mask));
	seq_printf(s, "L4 Destination Port:%x, mask:0x%04x\n",
		   ntohs(keye->l4_dst_port), ntohs(keye->l4_dst_port_mask));
	for (i = 0; i < NTMP_IPFT_MAX_PLD_LEN; i = i + 6) {
		seq_printf(s, "Payload %d~%d: %02x %02x %02x %02x %02x %02x\n",
			   i, i + 5, keye->byte[i].data, keye->byte[i + 1].data,
			   keye->byte[i + 2].data, keye->byte[i + 3].data,
			   keye->byte[i + 4].data, keye->byte[i + 5].data);
		seq_printf(s, "Payload Mask %d~%d: %02x %02x %02x %02x %02x %02x\n",
			   i, i + 5, keye->byte[i].mask, keye->byte[i + 1].mask,
			   keye->byte[i + 2].mask, keye->byte[i + 3].mask,
			   keye->byte[i + 4].mask, keye->byte[i + 5].mask);
	}

	/* STSE_DATA */
	seq_printf(s, "Match Count:%llu\n", le64_to_cpu(ipft_entry->match_count));

	/* CFGE_DATA */
	seq_printf(s, "Override internal Priority %s:%u\n", is_en(cfge->oipv),
		   cfge->ipv);
	seq_printf(s, "Override Drop Resilience %s:%u\n", is_en(cfge->odr),
		   cfge->dr);
	seq_printf(s, "Filter Forwarding Action:%u\n", cfge->fltfa);
	seq_printf(s, "Wake-on-LAN Trigger %s\n", is_en(cfge->wolte));
	seq_printf(s, "Filter Action:%u\n", cfge->flta);
	seq_printf(s, "Relative Precedent Resolution:%u\n", rpr);
	seq_printf(s, "Target For Selected Filter Action:0x%x\n",
		   le32_to_cpu(cfge->flta_tgt));

	if (priv->dev_type == NETC_DEV_SWITCH) {
		seq_printf(s, "Ingress Mirroring %s, Cut through disable: %d\n",
			   is_en(cfge->imire), cfge->ctd);
		seq_printf(s, "Host Reason: %d, Timestamp Capture %s\n",
			   cfge->hr, is_en(cfge->timecape));
		seq_printf(s, "Report Receive Timestamp: %s\n", is_yes(cfge->rrt));
	}

	seq_puts(s, "\n");

	return err;
}
EXPORT_SYMBOL_GPL(netc_show_ipft_entry);

int netc_show_tgst_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct tgst_query_data *qdata __free(kfree);
	int i, err;

	qdata = kzalloc(sizeof(*qdata), GFP_KERNEL);
	if (!qdata)
		return -ENOMEM;

	err = ntmp_tgst_query_entry(&priv->cbdrs, entry_id, qdata);
	if (err)
		return err;

	seq_puts(s, "Dump Time Gate Scheduling Table Entry:\n");
	seq_printf(s, "Entry ID:%d\n", entry_id);
	seq_printf(s, "Admin Base Time:%llu\n", le64_to_cpu(qdata->admin_bt));
	seq_printf(s, "Admin Cycle Time:%u\n", le32_to_cpu(qdata->admin_ct));
	seq_printf(s, "Admin Cycle Extend Time:%u\n",
		   le32_to_cpu(qdata->admin_ct_ext));
	seq_printf(s, "Admin Control List Length:%u\n",
		   le16_to_cpu(qdata->admin_cl_len));
	for (i = 0; i < le16_to_cpu(qdata->admin_cl_len); i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tAdmin time interval:%u\n",
			   le32_to_cpu(qdata->cfge_ge[i].interval));
		seq_printf(s, "\tAdmin Traffic Class states:%02x\n",
			   qdata->cfge_ge[i].tc_state);
		seq_printf(s, "\tAdministrative gate operation type:%u\n",
			   qdata->cfge_ge[i].hr_cb);
	}

	seq_printf(s, "Config Change Time:%llu\n", le64_to_cpu(qdata->oper_cfg_ct));
	seq_printf(s, "Config Change Error:%llu\n", le64_to_cpu(qdata->oper_cfg_ce));
	seq_printf(s, "Operation Base Time:%llu\n", le64_to_cpu(qdata->oper_bt));
	seq_printf(s, "Operation Cycle Time:%u\n", le32_to_cpu(qdata->oper_ct));
	seq_printf(s, "Operation Cycle Extend Time:%u\n",
		   le32_to_cpu(qdata->oper_ct_ext));
	seq_printf(s, "Operation Control List Length:%u\n",
		   le16_to_cpu(qdata->oper_cl_len));
	for (i = 0; i < le16_to_cpu(qdata->oper_cl_len); i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tOperation time interval:%u\n",
			   le32_to_cpu(qdata->olse_ge[i].interval));
		seq_printf(s, "\tOperation Traffic Class states:%02x\n",
			   qdata->olse_ge[i].tc_state);
		seq_printf(s, "\tOperation gate operation type:%u\n",
			   qdata->olse_ge[i].hr_cb);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_tgst_entry);
