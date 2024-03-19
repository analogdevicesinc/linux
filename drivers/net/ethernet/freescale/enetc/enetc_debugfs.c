// SPDX-License-Identifier: GPL-2.0+
/* Copyright 2023 NXP
 */
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "enetc_pf.h"

#define is_en(x)	(x) ? "enable" : "disable"

static int enetc_tgst_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct ntmp_tgst_info *info;
	int i, err, port;
	u32 val;

	val = enetc_port_rd(&si->hw, ENETC4_PTGSCR);
	if (!(val & PTGSCR_TGE)) {
		seq_puts(s, "Time Gating Disable\n");
		return 0;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	port = enetc4_pf_to_port(si->pdev);
	if (port < 0) {
		err = -EINVAL;
		goto end;
	}

	err = ntmp_tgst_query_entry(&si->cbdr, port, info);
	if (err)
		goto end;

	seq_puts(s, "Dump Time Gate Scheduling Table Info:\n");
	seq_printf(s, "Entry ID:%d\n", info->entry_id);
	seq_printf(s, "Admin Base Time:%llu\n", info->admin_bt);
	seq_printf(s, "Admin Cycle Time:%u\n", info->admin_ct);
	seq_printf(s, "Admin Cycle Extend Time:%u\n", info->admin_ct_ext);
	seq_printf(s, "Admin Control List Length:%u\n", info->admin_cl_len);
	for (i = 0; i < info->admin_cl_len; i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tAdmin time interval:%u\n", info->admin[i].interval);
		seq_printf(s, "\tAdmin Traffic Class states:%02x\n", info->admin[i].tc_gates);
		seq_printf(s, "\tAdministrative gate operation type:%d\n",
			   info->admin[i].oper_type);
	}

	seq_printf(s, "Config Change Time:%llu\n", info->cfg_ct);
	seq_printf(s, "Config Change Error:%llu\n", info->cfg_ce);
	seq_printf(s, "Operation Base Time:%llu\n", info->oper_bt);
	seq_printf(s, "Operation Cycle Time:%u\n", info->oper_ct);
	seq_printf(s, "Operation Cycle Externd Time:%u\n", info->oper_ct_ext);
	seq_printf(s, "Operation Control List Length:%u\n", info->oper_cl_len);
	for (i = 0; i < info->oper_cl_len; i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tOperation time interval:%u\n", info->oper[i].interval);
		seq_printf(s, "\tOperation Traffic Class states:%02x\n", info->oper[i].tc_gates);
		seq_printf(s, "\tOperation gate operation type:%d\n", info->oper[i].oper_type);
	}

end:
	kfree(info);
	return err;
}
DEFINE_SHOW_ATTRIBUTE(enetc_tgst);

static void enetc_get_txq_config(struct seq_file *s, struct enetc_si *si, int index)
{
	u32 wrr, prio, val;
	bool en, fwb, vih;

	val = enetc_txbdr_rd(&si->hw, index, ENETC_TBMR);
	en = !!(val & ENETC_TBMR_EN);
	fwb = !!(val & ENETC_TBMR_FWB);
	vih = !!(val & ENETC_TBMR_VIH);
	wrr = (val & ENETC_TBMR_WRR) >> 4;
	prio = val & ENETC_TBMR_PRIO_MASK;

	seq_printf(s, "txq %d:%s Force Writeback:%s VLAN Insert:%s WRR:%u Priority:%u\n",
		   index, is_en(en), is_en(fwb), is_en(vih), wrr, prio);
}

static int enetc_txq_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct net_device *ndev;
	u32 txq_num;
	u8 tc_num;
	int i, j;

	ndev = si->ndev;
	txq_num = ndev->real_num_tx_queues;
	tc_num = ndev->num_tc;

	seq_printf(s, "%s txq number:%u, tc num:%u\n", netdev_name(ndev), txq_num, tc_num);
	/* TC MQPRIO HW OFFLOAD */
	if (tc_num) {
		for (i = 0; i < tc_num; i++) {
			int offset, count, end;

			offset = ndev->tc_to_txq[i].offset;
			count = ndev->tc_to_txq[i].count;
			end = offset + count - 1;
			seq_printf(s, "txq %d to %d map to traffic class %d\n", offset, end, i);

			for (j = 0; j < count; j++)
				enetc_get_txq_config(s, si, offset + j);
		}
	} else {
		for (i = 0; i < txq_num; i++)
			enetc_get_txq_config(s, si, i);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_txq);

static void enetc_show_si_mac_hash_filter(struct seq_file *s, struct enetc_hw *hw, int si)
{
	u32 hash_h, hash_l;

	hash_l = enetc_port_rd(hw, ENETC4_PSIUMHFR0(si));
	hash_h = enetc_port_rd(hw, ENETC4_PSIUMHFR1(si));
	seq_printf(s, "SI%d unicast MAC hash filter:0x%08x%08x\n", si, hash_h, hash_l);

	hash_l = enetc_port_rd(hw, ENETC4_PSIMMHFR0(si));
	hash_h = enetc_port_rd(hw, ENETC4_PSIMMHFR1(si));
	seq_printf(s, "SI%d multicast MAC hash filter:0x%08x%08x\n", si, hash_h, hash_l);
}

static int enetc_rx_mode_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct enetc_hw *hw = &si->hw;
	struct enetc_pf *pf;
	int i, err;
	u32 val;

	pf = enetc_si_priv(si);

	val = enetc_port_rd(hw, ENETC4_PSIPMMR);
	seq_printf(s, "Unicast Promisc:0x%lx. Multicast Promisc:0x%lx.\n",
		   val & PSIPMMR_SI_MAC_UP, (val & PSIPMMR_SI_MAC_MP) >> 16);

	/* Use MAC hash filter */
	if (!pf->num_mac_fe) {
		for (i = 0; i < pf->num_vfs + 1; i++)
			enetc_show_si_mac_hash_filter(s, hw, i);

		return 0;
	}

	seq_printf(s, "The total number of entries in MAC filter table is %d.\n",
		   pf->num_mac_fe);
	/* Use MAC exact match table */
	for (i = 0; i < pf->num_mac_fe; i++) {
		struct ntmp_mfe entry;

		err = ntmp_maft_query_entry(&si->cbdr, i, &entry);
		if (err)
			return err;

		seq_printf(s, "Entry %d: %02x:%02x:%02x:%02x:%02x:%02x SI bitmap:0x%04x.\n",
			   i, entry.mac[0], entry.mac[1], entry.mac[2], entry.mac[3],
			   entry.mac[4], entry.mac[5], entry.si_bitmap);
	}

	for (i = 0; i < pf->num_vfs; i++)
		enetc_show_si_mac_hash_filter(s, hw, i + 1);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_rx_mode);

static void enetc_psfp_isi_show(struct seq_file *s, struct ntmp_isit_info *info)
{
	int i;

	seq_printf(s, "Key type:%u, IS_EID:%u.\n", info->key_type, info->is_eid);
	seq_puts(s, "Keys:");
	for (i = 0; i < NTMP_ISIT_FRAME_KEY_LEN; i++)
		seq_printf(s, "%02x", info->key[i]);
	seq_puts(s, "\n\n");
}

static void enetc_psfp_is_show(struct seq_file *s, struct ntmp_ist_info *info)
{
	seq_printf(s, "Stream Filtering %s. OIPV %s, IPV:%u.\n",
		   is_en(info->sfe), is_en(info->oipv), info->ipv);
	seq_printf(s, "ODR %s, DR:%u. ORP %s OSGI %s.\n", is_en(info->odr),
		   info->dr, is_en(info->orp), is_en(info->osgi));
	seq_printf(s, "Forwarding Action:%u. SDU type:%u. MSDU:%u.\n",
		   info->fa, info->sdu_type, info->msdu);
	seq_printf(s, "RP_EID:%u. SGI_EID:%u. ISC_EID:%u. SI_BITMAP:0x%04x.\n",
		   info->rp_eid, info->sgi_eid, info->isc_eid, info->si_bitmap);
	seq_puts(s, "\n");
}

static void enetc_psfp_isf_show(struct seq_file *s, struct ntmp_isft_info *info)
{
	seq_printf(s, "IS_EID:%u. PCP:%u. SDU type:%u. MSDU:%u.\n",
		   info->is_eid, info->pcp, info->sdu_type, info->msdu);
	seq_printf(s, "OIPV %s, IPV:%u. ODR %s, DR:%u. OSGI %s. ORP %s.\n",
		   is_en(info->oipv), info->ipv, is_en(info->odr), info->dr,
		   is_en(info->osgi), is_en(info->orp));
	seq_printf(s, "RP_EID:%u. SGI_EID:%u. ISC_EID:%u.\n", info->rp_eid,
		   info->sgi_eid, info->isc_eid);
	seq_puts(s, "\n");
}

static void enetc_psfp_rp_show(struct seq_file *s, struct ntmp_rpt_info *info)
{
	seq_printf(s, "Byte count:%llu, drop frames:%u.\n", info->sts.byte_cnt,
		   info->sts.drop_frames);
	seq_printf(s, "DR0 green frames:%u, DR1 green frames:%u.\n",
		   info->sts.dr0_grn_frames, info->sts.dr1_grn_frames);
	seq_printf(s, "DR2 yellow frames:%u, remark yellow frames:%u.\n",
		   info->sts.dr2_ylw_frames, info->sts.remark_ylw_frames);
	seq_printf(s, "DR3 red frames:%u, remark red frames:%u.\n",
		   info->sts.dr3_red_frames, info->sts.remark_red_frames);
	seq_printf(s, "CIR:%u, CBS:%u, EIR:%u, EBS%u.\n", info->cfg.cir,
		   info->cfg.cbs, info->cfg.eir, info->cfg.ebs);
	seq_printf(s, "Mark all frames red %s. Drop on yellow %s.\n",
		   is_en(info->cfg.mren), is_en(info->cfg.doy));
	seq_printf(s, "Coupling flag:%u. Color mode:%s.\n", info->cfg.cf,
		   info->cfg.cm ? "aware" : "blind");
	seq_printf(s, "No drop on red %s. SDU type:%u.\n", is_en(info->cfg.ndor),
		   info->cfg.sdu_type);
	seq_puts(s, "\n");
}

static void enetc_psfp_sgi_show(struct seq_file *s, struct ntmp_sgit_info *info)
{
	seq_printf(s, "OPER_SGCL_EID:%u, Configuration Change Time:%llu.\n",
		   info->oper_sgcl_eid, info->cfg_ct);
	seq_printf(s, "Operational Base Time:%llu, Cycle Time Extension:%u.\n",
		   info->oper_bt, info->oper_ct_ext);
	seq_printf(s, "Octets Exceeded %s, Octets Exceeded Flag:%u.\n",
		   is_en(info->oexen), info->oex);
	seq_printf(s, "Invalid Receive %s, Invalid Receive Flag:%u.\n",
		   is_en(info->irxen), info->irx);
	seq_printf(s, "Current Gate Instance State:%u, SDU type:%u.\n",
		   info->state, info->sdu_type);
	seq_printf(s, "OIPV %s, IPV:%u. Gate State:%s.\n", is_en(info->oipv),
		   info->ipv, info->gst ? "Open" : "Closed");
	seq_printf(s, "ADMIN_SGCL_EID:%u, Admin Base Time:%llu, Cycle Time Extension:%u.\n",
		   info->admin_sgcl_eid, info->admin_bt, info->admin_ct_ext);
	seq_puts(s, "\n");
}

static void enetc_psfp_sgcl_show(struct seq_file *s, struct ntmp_sgclt_info *info)
{
	int i;

	seq_printf(s, "Reference Count:%u, Cycle Time:%u, List Length:%u.\n",
		   info->ref_count, info->cycle_time, info->list_len);
	seq_printf(s, "EXT_OIPV %s, EXT_IPV:%u. Extension Gate State:%s.\n",
		   is_en(info->ext_oipv), info->ext_ipv, info->ext_gtst ? "Open" : "Closed");

	for (i = 0; i < info->list_len; i++) {
		seq_printf(s, "Gate Entry %d Time Interval:%u, Interval Octets Maximum %s:%u.\n",
			   i, info->ge[i].interval, is_en(info->ge[i].iomen), info->ge[i].iom);
		seq_printf(s, "Override Interval Priority Value %s:%u. Gate State:%s\n",
			   is_en(info->ge[i].oipv), info->ge[i].ipv,
			   info->ge[i].gtst ? "Open" : "Closed");
	}
	seq_puts(s, "\n");
}

static void enetc_psfp_isc_show(struct seq_file *s, struct ntmp_isct_info *info)
{
	seq_printf(s, "Receive Count:%u, MSDU Drop Count:%u.\n",
		   info->rx_count, info->msdu_drop_count);
	seq_printf(s, "Policer Drop Count:%u, Stream Gating Drop Count:%u.\n",
		   info->policer_drop_count, info->sg_drop_count);
	seq_puts(s, "\n");
}

static int enetc_psfp_show(struct seq_file *s, void *data)
{
	struct ntmp_sgclt_info *sgcl_info;
	struct ntmp_isit_info *isi_info;
	struct ntmp_isft_info *isf_info;
	struct ntmp_sgit_info *sgi_info;
	struct ntmp_isct_info *isc_info;
	struct ntmp_ist_info *is_info;
	struct ntmp_rpt_info *rp_info;
	struct enetc_si *si = s->private;
	struct enetc_ndev_priv *priv;
	struct enetc_psfp_node *psfp;
	int err = 0, i = 0;

	priv = netdev_priv(si->ndev);

	isi_info = kmalloc(sizeof(*isi_info), GFP_KERNEL);
	if (!isi_info)
		return -ENOMEM;

	is_info = kmalloc(sizeof(*is_info), GFP_KERNEL);
	if (!is_info) {
		err = -ENOMEM;
		goto free_isi_info;
	}

	isf_info = kmalloc(sizeof(*isf_info), GFP_KERNEL);
	if (!isf_info) {
		err = -ENOMEM;
		goto free_is_info;
	}

	sgi_info = kmalloc(sizeof(*sgi_info), GFP_KERNEL);
	if (!sgi_info) {
		err = -ENOMEM;
		goto free_isf_info;
	}

	sgcl_info = kmalloc(sizeof(*sgcl_info), GFP_KERNEL);
	if (!sgcl_info) {
		err = -ENOMEM;
		goto free_sgi_info;
	}

	rp_info = kmalloc(sizeof(*rp_info), GFP_KERNEL);
	if (!rp_info) {
		err = -ENOMEM;
		goto free_sgcl_info;
	}

	isc_info = kmalloc(sizeof(*isc_info), GFP_KERNEL);
	if (!isc_info) {
		err = -ENOMEM;
		goto free_rp_info;
	}

	hlist_for_each_entry(psfp, &priv->psfp_chain.isit_list, node) {
		if (!psfp)
			continue;

		memset(isi_info, 0, sizeof(*isi_info));
		memset(is_info, 0, sizeof(*is_info));
		memset(isf_info, 0, sizeof(*isf_info));
		memset(sgi_info, 0, sizeof(*sgi_info));
		memset(sgcl_info, 0, sizeof(*sgcl_info));
		memset(rp_info, 0, sizeof(*rp_info));
		memset(isc_info, 0, sizeof(*isc_info));

		seq_printf(s, "Show PSFP entry %d information.\n", i++);

		err = ntmp_isit_query_entry(&si->cbdr, psfp->isit_cfg.entry_id,
					    isi_info);
		if (err)
			goto free_isc_info;
		seq_printf(s, "Show ingress stream identification table entry %u:\n",
			   psfp->isit_cfg.entry_id);
		enetc_psfp_isi_show(s, isi_info);

		err = ntmp_ist_query_entry(&si->cbdr, psfp->isit_cfg.is_eid,
					   is_info);
		if (err)
			goto free_isc_info;
		seq_printf(s, "Show ingress stream table entry %u:\n",
			   psfp->isit_cfg.is_eid);
		enetc_psfp_is_show(s, is_info);

		if (psfp->isf_eid != NTMP_NULL_ENTRY_ID) {
			err = ntmp_isft_query_entry(&si->cbdr, psfp->isf_eid, isf_info);
			if (err)
				goto free_isc_info;
			seq_printf(s, "Show ingress stream filter table entry %u:\n",
				   psfp->isf_eid);
			enetc_psfp_isf_show(s, isf_info);
		}

		if (psfp->rp_eid != NTMP_NULL_ENTRY_ID) {
			err = ntmp_rpt_query_entry(&si->cbdr, psfp->rp_eid, rp_info);
			if (err)
				goto free_isc_info;
			seq_printf(s, "Show rate policer table entry %u:\n", psfp->rp_eid);
			enetc_psfp_rp_show(s, rp_info);
		}

		if (psfp->sgi_eid != NTMP_NULL_ENTRY_ID) {
			err = ntmp_sgit_query_entry(&si->cbdr, psfp->sgi_eid, sgi_info);
			if (err)
				goto free_isc_info;
			seq_printf(s, "Show stream gate instance table entry %u:\n",
				   psfp->sgi_eid);
			enetc_psfp_sgi_show(s, sgi_info);
		}

		if (psfp->sgcl_eid != NTMP_NULL_ENTRY_ID) {
			err = ntmp_sgclt_query_entry(&si->cbdr, psfp->sgcl_eid, sgcl_info);
			if (err)
				goto free_isc_info;
			seq_printf(s, "Show stream gate control list table entry %u:\n",
				   psfp->sgcl_eid);
			enetc_psfp_sgcl_show(s, sgcl_info);
		}

		if (psfp->isc_eid != NTMP_NULL_ENTRY_ID) {
			err = ntmp_isct_operate_entry(&si->cbdr, psfp->isc_eid, NTMP_CMD_QUERY,
						      isc_info);
			if (err)
				goto free_isc_info;
			seq_printf(s, "Show ingress stream count table entry %u:\n", psfp->isc_eid);
			enetc_psfp_isc_show(s, isc_info);
		}
	}

free_isc_info:
	kfree(isc_info);
free_rp_info:
	kfree(rp_info);
free_sgcl_info:
	kfree(sgcl_info);
free_sgi_info:
	kfree(sgi_info);
free_isf_info:
	kfree(isf_info);
free_is_info:
	kfree(is_info);
free_isi_info:
	kfree(isi_info);

	return err;
}
DEFINE_SHOW_ATTRIBUTE(enetc_psfp);

static int enetc_ipf_entry_show(struct seq_file *s, struct enetc_si *si, u32 entry_id)
{
	struct ntmp_ipft_info *info;
	u16 src_port, src_port_mask;
	u16 dscp, dscp_mask;
	u8 fltfa, flta, rpr;
	int i, err;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	err = ntmp_ipft_query_entry(&si->cbdr, entry_id, info);
	if (err)
		goto end;

	dscp = info->key.dscp & NTMP_IPFT_DSCP;
	dscp_mask = (info->key.dscp & NTMP_IPFT_DSCP_MASK) >> 6;
	src_port = info->key.src_port & NTMP_IPFT_SRC_PORT;
	src_port_mask = (info->key.src_port & NTMP_IPFT_SRC_PORT_MASK) >> 5;
	fltfa = info->cfg.filter & NTMP_IPFT_FLTFA;
	flta = (info->cfg.filter & NTMP_IPFT_FLTA) >> 5;
	rpr = (info->cfg.filter & NTMP_IPFT_RPR) >> 7;

	seq_printf(s, "Show ingress port filter table entry:%u\n", entry_id);
	seq_printf(s, "Precedence:%u, Frame attribute flags:0x%04x, mask:0x%04x\n",
		   info->key.precedence, info->key.frm_attr_flags,
		   info->key.frm_attr_flags_mask);
	seq_printf(s, "DSCP:%u, mask:0x%02x. Source port ID:%u, mask:0x%02x\n",
		   dscp, dscp_mask, src_port, src_port_mask);
	seq_printf(s, "Outer VLAN TCI:%u, mask:0x%04x. Inner VLAN TCI:%u, mask:0x%04x\n",
		   ntohs(info->key.outer_vlan_tci), ntohs(info->key.outer_vlan_tci_mask),
		   ntohs(info->key.inner_vlan_tci), ntohs(info->key.inner_vlan_tci_mask));
	seq_printf(s, "Destination MAC:%02x:%02x:%02x:%02x:%02x:%02x\n",
		   info->key.dmac[0], info->key.dmac[1], info->key.dmac[2],
		   info->key.dmac[3], info->key.dmac[4], info->key.dmac[5]);
	seq_printf(s, "Destination MAC mask:%02x:%02x:%02x:%02x:%02x:%02x\n",
		   info->key.dmac_mask[0], info->key.dmac_mask[1], info->key.dmac_mask[2],
		   info->key.dmac_mask[3], info->key.dmac_mask[4], info->key.dmac_mask[5]);
	seq_printf(s, "Source MAC:%02x:%02x:%02x:%02x:%02x:%02x\n",
		   info->key.smac[0], info->key.smac[1], info->key.smac[2],
		   info->key.smac[3], info->key.smac[4], info->key.smac[5]);
	seq_printf(s, "Source MAC mask:%02x:%02x:%02x:%02x:%02x:%02x\n",
		   info->key.smac_mask[0], info->key.smac_mask[1], info->key.smac_mask[2],
		   info->key.smac_mask[3], info->key.smac_mask[4], info->key.smac_mask[5]);
	seq_printf(s, "Ether Type:0x%04x, mask:0x%04x. IP protocol:%u, mask:0x%02x\n",
		   ntohs(info->key.ethertype), ntohs(info->key.ethertype_mask),
		   info->key.ip_protocol, info->key.ip_protocol_mask);
	seq_printf(s, "IP Source Address:%08x:%08x:%08x:%08x\n",
		   ntohl(info->key.ip_src[0]), ntohl(info->key.ip_src[1]),
		   ntohl(info->key.ip_src[2]), ntohl(info->key.ip_src[3]));
	seq_printf(s, "IP Source Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(info->key.ip_src_mask[0]), ntohl(info->key.ip_src_mask[1]),
		   ntohl(info->key.ip_src_mask[2]), ntohl(info->key.ip_src_mask[3]));
	seq_printf(s, "IP Destination Address:%08x:%08x:%08x:%08x\n",
		   ntohl(info->key.ip_dst[0]), ntohl(info->key.ip_dst[1]),
		   ntohl(info->key.ip_dst[2]), ntohl(info->key.ip_dst[3]));
	seq_printf(s, "IP Destination Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(info->key.ip_dst_mask[0]), ntohl(info->key.ip_dst_mask[1]),
		   ntohl(info->key.ip_dst_mask[2]), ntohl(info->key.ip_dst_mask[3]));
	seq_printf(s, "L4 Source Port:%u, mask:0x%04x. Destination Port:%u, mask:0x%04x\n",
		   ntohs(info->key.l4_src_port), ntohs(info->key.l4_src_port_mask),
		   ntohs(info->key.l4_dst_port), ntohs(info->key.l4_dst_port_mask));
	for (i = 0; i < NTMP_IPFT_MAX_PLD_LEN; i = i + 6) {
		seq_printf(s, "Payload byte %d~%d:%02x%02x%02x%02x%02x%02x\n", i, i + 5,
			   info->key.byte[i].data, info->key.byte[i + 1].data,
			   info->key.byte[i + 2].data, info->key.byte[i + 3].data,
			   info->key.byte[i + 4].data, info->key.byte[i + 5].data);
		seq_printf(s, "Payload byte %d~%d mask:%02x%02x%02x%02x%02x%02x\n", i, i + 5,
			   info->key.byte[i].mask, info->key.byte[i + 1].mask,
			   info->key.byte[i + 2].mask, info->key.byte[i + 3].mask,
			   info->key.byte[i + 4].mask, info->key.byte[i + 5].mask);
	}
	seq_printf(s, "Match Count:%llu\n", info->match_count);
	seq_printf(s, "Override internal Priority %s:%u\n", is_en(info->cfg.oipv), info->cfg.ipv);
	seq_printf(s, "Override Drop Resilience %s:%u\n", is_en(info->cfg.odr), info->cfg.dr);
	seq_printf(s, "Filter Forwarding Action:%u\n", fltfa);
	seq_printf(s, "Wake-on-LAN Trigger %s\n", is_en(info->cfg.filter & NTMP_IPFT_WOLTE));
	seq_printf(s, "Filter Action:%u\n", flta);
	seq_printf(s, "Relative Precedent Resolution:%u\n", rpr);
	seq_printf(s, "Target For Selected Filter Action:0x%x\n", info->cfg.flta_tgt);
	seq_puts(s, "\n");

end:
	kfree(info);

	return err;
}

static int enetc_ipf_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct enetc_hw *hw = &si->hw;
	struct enetc_ndev_priv *priv;
	int i, err;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_PIPFCR);
	seq_printf(s, "Ingress port filter is %s\n", is_en(val & PIPFCR_EN));
	val = enetc_port_rd(hw, ENETC4_IPFTCAPR) & IPFTCAPR_NUM_WORDS;
	seq_printf(s, "Number of ternary words supported: %u\n", val);
	val = enetc_port_rd(hw, ENETC4_IPFTMOR) & IPFTMOR_NUM_WORDS;
	seq_printf(s, "Number of words in-use: %u\n", val);

	priv = netdev_priv(si->ndev);
	for (i = 0; i < priv->max_ipf_entries; i++) {
		if (priv->cls_rules[i].used) {
			err = enetc_ipf_entry_show(s, si, priv->cls_rules[i].entry_id);
			if (err)
				return err;
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_ipf);

static void enetc_txr_bd_show(struct seq_file *s, union enetc_tx_bd *txbd, int index)
{
	__le32 *p = (__le32 *)txbd;

	seq_printf(s, "TX BD%d: %08x %08x %08x %08x\n",
		   index, le32_to_cpu(*p),
		   le32_to_cpu(*(p + 1)),
		   le32_to_cpu(*(p + 2)),
		   le32_to_cpu(*(p + 3)));
}

static int enetc_txr_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct enetc_hw *hw = &si->hw;
	struct enetc_ndev_priv *priv;
	int i, j;
	u32 val;

	priv = netdev_priv(si->ndev);

	for (i = 0; i < priv->num_tx_rings; i++) {
		struct enetc_bdr *txr = priv->tx_ring[i];
		union enetc_tx_bd *txbd;

		seq_printf(s, "TX Ring%d Info:\n", i);
		seq_printf(s, "SW next_to_clean:%d\n", txr->next_to_clean);
		seq_printf(s, "SW next_to_use:%d\n", txr->next_to_use);

		val = enetc_txbdr_rd(hw, i, ENETC_TBCIR);
		seq_printf(s, "HW CIR:%u\n", val);
		val = enetc_txbdr_rd(hw, i, ENETC_TBPIR);
		seq_printf(s, "HW PIR:%u\n", val);
		val = enetc_txbdr_rd(hw, i, ENETC_TBMR);
		seq_printf(s, "TX BDR mode:0x%x\n", val);

		for (j = 0; j < txr->bd_count; j++) {
			txbd = ENETC_TXBD(*txr, j);
			enetc_txr_bd_show(s, txbd, j);
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_txr);

static void enetc_rxr_bd_show(struct seq_file *s, union enetc_rx_bd *rxbd, int index)
{
	__le32 *p = (__le32 *)rxbd;

	seq_printf(s, "RX BD%d: %08x %08x %08x %08x\n",
		   index, le32_to_cpu(*p),
		   le32_to_cpu(*(p + 1)),
		   le32_to_cpu(*(p + 2)),
		   le32_to_cpu(*(p + 3)));
}

static int enetc_rxr_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct enetc_hw *hw = &si->hw;
	struct enetc_ndev_priv *priv;
	int i, j;
	u32 val;

	priv = netdev_priv(si->ndev);

	for (i = 0; i < priv->num_rx_rings; i++) {
		struct enetc_bdr *rxr = priv->rx_ring[i];
		union enetc_rx_bd *rxbd;

		seq_printf(s, "RX Ring%d Info:\n", i);
		seq_printf(s, "Extended RX BD: %s\n", is_en(rxr->ext_en));
		seq_printf(s, "SW next_to_clean:%d\n", rxr->next_to_clean);
		seq_printf(s, "SW next_to_use:%d\n", rxr->next_to_use);
		seq_printf(s, "SW next_to_alloc:%d\n", rxr->next_to_alloc);

		val = enetc_rxbdr_rd(hw, i, ENETC_RBCIR);
		seq_printf(s, "HW CIR:%u\n", val);
		val = enetc_rxbdr_rd(hw, i, ENETC_RBPIR);
		seq_printf(s, "HW PIR:%u\n", val);
		val = enetc_rxbdr_rd(hw, i, ENETC_RBMR);
		seq_printf(s, "RX BDR mode:0x%x\n", val);

		for (j = 0; j < rxr->bd_count; j++) {
			rxbd = &((union enetc_rx_bd *)rxr->bd_base)[j];
			enetc_rxr_bd_show(s, rxbd, j);
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_rxr);

void enetc_create_debugfs(struct enetc_si *si)
{
	struct net_device *ndev = si->ndev;
	struct dentry *root;

	root = debugfs_create_dir(netdev_name(ndev), NULL);
	if (IS_ERR(root))
		return;

	si->debugfs_root = root;

	if (si->hw.port) {
		debugfs_create_file("enetc_tgst", 0444, root, si, &enetc_tgst_fops);
		debugfs_create_file("rx_mode", 0444, root, si, &enetc_rx_mode_fops);
		debugfs_create_file("enetc_psfp", 0444, root, si, &enetc_psfp_fops);
		debugfs_create_file("enetc_ipf", 0444, root, si, &enetc_ipf_fops);
	}

	debugfs_create_file("enetc_txq", 0444, root, si, &enetc_txq_fops);
	debugfs_create_file("tx_bdr", 0444, root, si, &enetc_txr_fops);
	debugfs_create_file("rx_bdr", 0444, root, si, &enetc_rxr_fops);
}

void enetc_remove_debugfs(struct enetc_si *si)
{
	debugfs_remove_recursive(si->debugfs_root);
	si->debugfs_root = NULL;
}
