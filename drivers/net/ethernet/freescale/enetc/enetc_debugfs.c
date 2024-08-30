// SPDX-License-Identifier: GPL-2.0+
/* Copyright 2023 NXP
 */
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "enetc_pf.h"

#define DEFINE_ENETC_DEBUGFS(name, write_op)			\
static int name##_open(struct inode *inode, struct file *file)		\
{									\
	return single_open(file, name##_show, inode->i_private);	\
}									\
									\
static const struct file_operations name##_fops = {			\
	.owner		= THIS_MODULE,					\
	.open		= name##_open,					\
	.read		= seq_read,					\
	.write		= enetc_##write_op##_write,			\
	.llseek		= seq_lseek,					\
	.release	= single_release,				\
}

static int enetc_tgst_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	int port;
	u32 val;

	val = enetc_port_rd(&si->hw, ENETC4_PTGSCR);
	if (!(val & PTGSCR_TGE)) {
		seq_puts(s, "Time Gating Disable\n");
		return 0;
	}

	port = enetc4_pf_to_port(si->pdev);
	if (port < 0)
		return -EINVAL;

	return netc_show_tgst_entry(&si->ntmp, s, port);
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
	struct maft_entry_data maft_data;
	struct enetc_si *si = s->private;
	struct enetc_hw *hw = &si->hw;
	struct maft_keye_data *keye;
	struct enetc_pf *pf;
	int i, err;
	u32 val;

	pf = enetc_si_priv(si);

	val = enetc_port_rd(hw, ENETC4_PSIPMMR);
	seq_printf(s, "Unicast Promisc:0x%lx. Multicast Promisc:0x%lx\n",
		   val & PSIPMMR_SI_MAC_UP, (val & PSIPMMR_SI_MAC_MP) >> 16);

	/* Use MAC hash filter */
	if (!pf->num_mac_fe) {
		for (i = 0; i < pf->num_vfs + 1; i++)
			enetc_show_si_mac_hash_filter(s, hw, i);

		return 0;
	}

	seq_printf(s, "The total number of entries in MAC filter table is %d\n",
		   pf->num_mac_fe);
	/* Use MAC exact match table */
	for (i = 0; i < pf->num_mac_fe; i++) {
		memset(&maft_data, 0, sizeof(maft_data));
		err = ntmp_maft_query_entry(&si->ntmp.cbdrs, i, &maft_data);
		if (err)
			return err;

		keye = &maft_data.keye;
		seq_printf(s, "Entry %d, MAC %pM, SI bitmap:0x%04x\n", i,
			   keye->mac_addr, le16_to_cpu(maft_data.cfge.si_bitmap));
	}

	for (i = 0; i < pf->num_vfs; i++)
		enetc_show_si_mac_hash_filter(s, hw, i + 1);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_rx_mode);

static int enetc_flower_list_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct netc_flower_rule *rule;

	guard(mutex)(&si->ntmp.flower_lock);
	hlist_for_each_entry(rule, &si->ntmp.flower_list, node) {
		seq_printf(s, "Cookie:0x%lx\n", rule->cookie);
		seq_printf(s, "Flower type:%d\n", rule->flower_type);

		if (rule->flower_type == FLOWER_TYPE_PSFP)
			netc_show_psfp_flower(s, rule);

		seq_puts(s, "\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_flower_list);

static ssize_t enetc_isit_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.isit_eid);
}

static int enetc_isit_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_isit_entry(&si->ntmp, s, si->dbg_params.isit_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_isit_entry, isit_eid);

static ssize_t enetc_ist_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.ist_eid);
}

static int enetc_ist_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_ist_entry(&si->ntmp, s, si->dbg_params.ist_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_ist_entry, ist_eid);

static ssize_t enetc_isft_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.isft_eid);
}

static int enetc_isft_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_isft_entry(&si->ntmp, s, si->dbg_params.isft_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_isft_entry, isft_eid);

static ssize_t enetc_sgit_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.sgit_eid);
}

static int enetc_sgit_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_sgit_entry(&si->ntmp, s, si->dbg_params.sgit_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_sgit_entry, sgit_eid);

static ssize_t enetc_sgclt_eid_write(struct file *filp, const char __user *buffer,
				     size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.sgclt_eid);
}

static int enetc_sgclt_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_sgclt_entry(&si->ntmp, s, si->dbg_params.sgclt_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_sgclt_entry, sgclt_eid);

static ssize_t enetc_isct_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.isct_eid);
}

static int enetc_isct_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_isct_entry(&si->ntmp, s, si->dbg_params.isct_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_isct_entry, isct_eid);

static ssize_t enetc_rpt_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.rpt_eid);
}

static int enetc_rpt_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_rpt_entry(&si->ntmp, s, si->dbg_params.rpt_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_rpt_entry, rpt_eid);

static int enetc_ipft_show(struct seq_file *s, void *data)
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
			err = netc_show_ipft_entry(&si->ntmp, s,
						   priv->cls_rules[i].entry_id);
			if (err)
				return err;
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_ipft);

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
		debugfs_create_file("isit_entry", 0600, root, si, &enetc_isit_entry_fops);
		debugfs_create_file("ist_entry", 0600, root, si, &enetc_ist_entry_fops);
		debugfs_create_file("isft_entry", 0600, root, si, &enetc_isft_entry_fops);
		debugfs_create_file("sgit_entry", 0600, root, si, &enetc_sgit_entry_fops);
		debugfs_create_file("sgclt_entry", 0600, root, si, &enetc_sgclt_entry_fops);
		debugfs_create_file("isct_entry", 0600, root, si, &enetc_isct_entry_fops);
		debugfs_create_file("rpt_entry", 0600, root, si, &enetc_rpt_entry_fops);
		debugfs_create_file("flower_list", 0444, root, si, &enetc_flower_list_fops);
		debugfs_create_file("enetc_ipft", 0444, root, si, &enetc_ipft_fops);
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
