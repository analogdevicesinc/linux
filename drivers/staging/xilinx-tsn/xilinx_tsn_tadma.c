// SPDX-License-Identifier: GPL-2.0

/* Xilinx FPGA Xilinx TADMA driver.
 *
 * Copyright (C) 2018 Xilinx, Inc. All rights reserved.
 *
 * Author: Syed Syed <syeds@xilinx.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/list.h>
#include <linux/hash.h>
#include <linux/platform_device.h>
#include "xilinx_axienet_tsn.h"
#include "xilinx_tsn_tadma.h"

/* max packets that can be sent in a time trigger */
#define MAX_TRIG_COUNT 4

/* This driver assumes the num_streams configured in HW is always 2^n.
 * TADMA IP has three master AXI4-Stream Interfaces. The names of these three
 * interfaces are mentioned in the TADMA PG as ST, RES and BE. Although these
 * names might be misleading since TADMA is used exclusively for ST traffic, we
 * maintain consistency with the PG by using the same identifiers in the
 * software, such as qt_st, qt_res, and qt_be
 */

typedef u32 pm_entry_t;

struct tadma_stream {
	u8 dmac[6];
	short vid;
	u32 trigger;
	u32 count;
	u8 qno;
};

struct tadma_stream_entry {
	u8 macvlan[8];
	u32 tticks;
	struct hlist_node hash_link;
	int sid;
	int count;
	enum qtype qtype;
};

#define sfm_entry_offset(lp, sid) \
	((lp->active_sfm == SFM_UPPER) ? \
	(XTADMA_USFM_OFFSET) +  ((sid) * sizeof(struct sfm_entry)) : \
	(XTADMA_LSFM_OFFSET) +  ((sid) * sizeof(struct sfm_entry)))

static inline bool tadma_queue_enabled(struct axienet_local *lp, enum qtype qt)
{
	return lp->tadma_queues & BIT(qt);
}

static inline int tadma_qt_to_txq_idx(struct axienet_local *lp, enum qtype qt)
{
	int qno;

	for (qno = 0; qno < lp->num_tc; qno++) {
		if (lp->txqs[qno].is_tadma &&
		    lp->txqs[qno].dmaq_idx == qt) {
			return qno;
		}
	}

	return -EINVAL;
}

static inline u32 tadma_macvlan_hash(struct axienet_local *lp,
				     const unsigned char *addr)
{
	u64 value = get_unaligned((u64 *)addr);

	return hash_64(value, lp->tadma_hash_bits);
}

static inline bool mac_vlan_equal(const u8 addr1[8],
				  const u8 addr2[8])
{
	const u16 *a = (const u16 *)addr1;
	const u16 *b = (const u16 *)addr2;
	bool ret = 1;

	if ((a[0] ^ b[0]) | (a[1] ^ b[1]) |
		(a[2] ^ b[2]) | (a[3] ^ a[3])) {
		ret = 0;
	}

	return ret;
}

static struct tadma_stream_entry *tadma_hash_lookup_stream(struct hlist_head *head,
							   const unsigned char *mac_vlan)
{
	struct tadma_stream_entry *entry;

	hlist_for_each_entry(entry, head, hash_link) {
		if (mac_vlan_equal(entry->macvlan, mac_vlan))
			return entry;
	}

	return NULL;
}

static u32 tadma_stream_alm_offset_irq(int sid, u32 tx_bd_rd, struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	u32 alm_offset;

	alm_offset = XTADMA_ALM_OFFSET +
		sid * sizeof(struct alm_entry) * lp->num_tadma_buffers;

	return alm_offset + (tx_bd_rd * sizeof(struct alm_entry));
}

static void tadma_xmit_done(struct net_device *ndev, u8 sid, u32 cnt)
{
	struct axienet_local *lp = netdev_priv(ndev);
	u32 size = 0, packets = 0;

	spin_lock(&lp->tadma_tx_lock);

	if (lp->tx_bd_head[sid] == lp->tx_bd_tail[sid]) {
		spin_unlock(&lp->tadma_tx_lock);
		return;
	}

	while ((lp->tx_bd_head[sid] != lp->tx_bd_tail[sid]) && cnt) {
		if (lp->tx_bd[sid][lp->tx_bd_tail[sid]].tx_desc_mapping ==
		    DESC_DMA_MAP_PAGE) {
			dma_unmap_page(ndev->dev.parent,
				       lp->tx_bd[sid][lp->tx_bd_tail[sid]].phys,
				       lp->tx_bd[sid][lp->tx_bd_tail[sid]].len,
				       DMA_TO_DEVICE);
		} else {
			dma_unmap_single(ndev->dev.parent,
					 lp->tx_bd[sid][lp->tx_bd_tail[sid]].
					 phys,
					 lp->tx_bd[sid][lp->tx_bd_tail[sid]].
					 len,
					 DMA_TO_DEVICE);
		}
		if (lp->tx_bd[sid][lp->tx_bd_tail[sid]].tx_skb) {
			dev_kfree_skb_irq(lp->tx_bd[sid][lp->tx_bd_tail[sid]].
					  tx_skb);
		}

		size += lp->tx_bd[sid][lp->tx_bd_tail[sid]].len;
		packets++;
		lp->tx_bd_tail[sid]++;
		lp->tx_bd_tail[sid] %= lp->num_tadma_buffers;
		cnt--;
	}

	ndev->stats.tx_packets += packets;
	ndev->stats.tx_bytes += size;
	spin_unlock(&lp->tadma_tx_lock);
}

static irqreturn_t tadma_irq(int irq, void *_ndev)
{
	struct axienet_local *lp = netdev_priv(_ndev);
	struct alm_entry alm;
	u32 status, sid = 0, alm_offset;
	int cnt = 0;

	status = tadma_ior(lp, XTADMA_INT_STA_OFFSET);

	/* clear interrupt */
	tadma_iow(lp, XTADMA_INT_CLR_OFFSET, status);

	if (status & XTADMA_FFI_INT_EN) {
		for (sid = 0; sid < lp->num_streams; sid++) {
			cnt = 0;
			spin_lock(&lp->tadma_tx_lock);
			alm_offset = tadma_stream_alm_offset_irq(sid, lp->tx_bd_rd[sid], _ndev);
			alm.cfg = tadma_ior(lp, alm_offset + 4);
			while (((alm.cfg & XTADMA_ALM_UFF) == 0) &&
			       (cnt < lp->num_tadma_buffers) &&
			       (lp->tx_bd_rd[sid] != lp->tx_bd_head[sid])) {
				lp->tx_bd_rd[sid]++;
				lp->tx_bd_rd[sid] = lp->tx_bd_rd[sid] % lp->num_tadma_buffers;
				alm_offset = tadma_stream_alm_offset_irq(sid,
									 lp->tx_bd_rd[sid], _ndev);
				alm.cfg = tadma_ior(lp, alm_offset + 4);
				cnt++;
			}
			spin_unlock(&lp->tadma_tx_lock);
			if (cnt) {
				tadma_xmit_done(_ndev, sid, cnt);
				if (__netif_subqueue_stopped(_ndev,
							     lp->sid_txq_idx[sid])) {
					netif_wake_subqueue(_ndev,
							    lp->sid_txq_idx[sid]);
				}
			}
		}
	}

	return IRQ_HANDLED;
}

static int tadma_sfm_hash_init(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct tadma_cb *cb;
	int i;

	cb = kzalloc(sizeof(*cb), GFP_KERNEL);
	if (!cb)
		return -ENOMEM;

	cb->stream_hash = kcalloc(lp->num_entries, sizeof(struct hlist_head *),
				  GFP_KERNEL);

	if (!cb->stream_hash) {
		kfree(cb);
		return -ENOMEM;
	}

	for (i = 0; i < lp->num_entries; i++)
		INIT_HLIST_HEAD(&cb->stream_hash[i]);

	lp->t_cb = cb;

	return 0;
}

static int tadma_sfm_program(struct net_device *ndev, int sid,
			     enum qtype qtype, u32 tticks, u32 count)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct sfm_entry sfm = {0, };
	int txq_idx;
	u32 offset;

	pr_debug("%s sid: %d, qtype %d, count: %d\n", __func__, sid, qtype,
		 count);
	txq_idx = tadma_qt_to_txq_idx(lp, qtype);
	if (txq_idx < 0) {
		dev_err(&ndev->dev, "Failed to get txq for TADMA qtype %d\n",
			qtype);
		return -EINVAL;
	}

	lp->sid_txq_idx[sid] = txq_idx;

	offset = sfm_entry_offset(lp, sid);

	/* each tick is 8ns */
	sfm.tticks = tticks / 8;

	/*clear strid and queue type */
	sfm.cfg &= ~(XTADMA_STR_ID_MASK | XTADMA_STR_QUE_TYPE_MASK);

	sfm.cfg |= (sid << XTADMA_STR_ID_SHIFT) & XTADMA_STR_ID_MASK;
	sfm.cfg |= (qtype << XTADMA_STR_QUE_TYPE_SHIFT) &
		   XTADMA_STR_QUE_TYPE_MASK;
	if (count != 0)
		sfm.cfg &= ~XTADMA_STR_CONT_FETCH_EN;
	else
		sfm.cfg |= XTADMA_STR_CONT_FETCH_EN;
	sfm.cfg |= XTADMA_STR_ENTRY_VALID;

	count  = (count > 0) ? (count - 1) : count;
	/* hw xmits 1 more than what is programmed, so use count */
	sfm.cfg |= (count << XTADMA_STR_NUM_FRM_SHIFT) &
			XTADMA_STR_NUM_FRM_MASK;
	pr_debug("sfm cfg: %x\n", sfm.cfg);
	tadma_iow(lp, offset, sfm.tticks);
	tadma_iow(lp, offset + 4, sfm.cfg);
	return 0;
}

static int tadma_set_contiguous_mode(struct net_device *ndev, enum qtype qtype)
{
	struct axienet_local *lp = netdev_priv(ndev);
	int sid, ret;

	if (lp->get_sid >= lp->num_streams - 1) {
		dev_info(&ndev->dev, "Can't support more than %d streams\n",
			 lp->get_sid + 1);
		return -EINVAL;
	}

	if (lp->get_sfm >= lp->num_entries) {
		dev_info(&ndev->dev, "Can't support more than %d SFM entries\n",
			 lp->get_sfm);
		return -EINVAL;
	}

	sid = lp->get_sid++;
	lp->get_sfm++;
	ret = tadma_sfm_program(ndev, sid, qtype, NSEC_PER_MSEC, 0);
	if (ret)
		return ret;

	return sid;
}

static int tadma_set_contiguous_mode_all(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);

	if (tadma_queue_enabled(lp, qt_res)) {
		lp->default_res_sid = tadma_set_contiguous_mode(ndev, qt_res);
		if (lp->default_res_sid < 0)
			return lp->default_res_sid;
	}

	if (tadma_queue_enabled(lp, qt_st)) {
		lp->default_st_sid = tadma_set_contiguous_mode(ndev, qt_st);
		if (lp->default_st_sid < 0)
			return lp->default_st_sid;
	}

	return 0;
}

static int tadma_sfm_init(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);

	lp->active_sfm = SFM_UPPER;
	lp->get_sid = 0;
	lp->get_sfm = 0;
	return tadma_set_contiguous_mode_all(ndev);
}

int axienet_tadma_stop(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	u8 i = 0;

	for (i = 0; i < lp->num_streams ; i++)
		kfree(lp->tx_bd[i]);

	kfree(lp->tx_bd);

	free_irq(lp->tadma_irq, ndev);

	return 0;
}

int axienet_tadma_open(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	static char irq_name[32];
	u8 i = 0;
	int ret;
	u32 cr;

	if (lp->tadma_irq) {
		snprintf(irq_name, sizeof(irq_name), "%s_tadma_tx", ndev->name);
		ret = request_irq(lp->tadma_irq, tadma_irq, IRQF_SHARED,
				  irq_name, ndev);
		if (ret)
			return ret;
	}
	pr_debug("%s TADMA irq %d\n", __func__, lp->tadma_irq);

	/* enable all interrupts */
	tadma_iow(lp, XTADMA_INT_EN_OFFSET, XTADMA_FFI_INT_EN |
		  XTADMA_IE_INT_EN);

	tadma_sfm_init(ndev);
	tadma_sfm_hash_init(ndev);
	cr = XTADMA_CFG_DONE;
	tadma_iow(lp, XTADMA_CR_OFFSET, cr);

	lp->tx_bd = kmalloc_array(lp->num_streams, sizeof(struct axitadma_bd *),
				  GFP_KERNEL);
	if (!lp->tx_bd)
		ret = -ENOMEM;

	for (i = 0; i < lp->num_streams ; i++) {
		lp->tx_bd_head[i] = 0;
		lp->tx_bd_tail[i] = 0;
		lp->tx_bd_rd[i] = 0;
		lp->tx_bd[i] = kmalloc_array(lp->num_tadma_buffers,
					     sizeof(*lp->tx_bd[i]), GFP_KERNEL);
		if (!lp->tx_bd[i])
			return -ENOMEM;
	}

	return ret;
}

/*TODO: Fix TADMA probe error handling path */
int __maybe_unused axienet_tadma_probe(struct platform_device *pdev,
				       struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct resource tadma_res;
	struct device_node *np;
	u8 count = 0;
	u16 num_tc;
	int ret;

	ret = of_property_read_u16(pdev->dev.of_node, "xlnx,num-tc",
				   &num_tc);
	if (ret) {
		dev_err(&pdev->dev, "xlnx,num-tc parameter not defined\n");
		return ret;
	}

	np = of_parse_phandle(pdev->dev.of_node, "axistream-connected-tx",
			      num_tc - 1);
	if (!np) {
		dev_err(&pdev->dev, "could not find TA-DMA node\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(np, 0, &tadma_res);
	if (ret)
		goto err_put_node;

	lp->tadma_regs = devm_ioremap_resource(&pdev->dev, &tadma_res);
	if (IS_ERR(lp->tadma_regs)) {
		ret = PTR_ERR(lp->tadma_regs);
		goto err_put_node;
	}

	lp->tadma_irq = irq_of_parse_and_map(np, 0);
	if (!lp->tadma_irq) {
		ret = -EINVAL;
		goto err_put_node;
	}

	ret = of_property_read_u32(np, "xlnx,num-buffers-per-stream",
				   &lp->num_tadma_buffers);
	if (ret)
		lp->num_tadma_buffers = 64;

	ret = of_property_read_u32(np, "xlnx,num-streams", &lp->num_streams);
	if (ret)
		lp->num_streams = 8;
	ret = of_property_read_u32(np, "xlnx,num-fetch-entries",
				   &lp->num_entries);
	if (ret)
		lp->num_entries = 8;

	while (!((lp->num_streams >> count) & 1))
		count++;

	lp->get_sid = 0;
	lp->get_sfm = 0;
	lp->default_res_sid = -1;
	lp->default_st_sid = -1;
	lp->tadma_hash_bits = count;
	pr_debug("%s num_stream: %d hash_bits: %d\n", __func__, lp->num_streams,
		 lp->tadma_hash_bits);
	pr_info("TADMA probe done\n");
	spin_lock_init(&lp->tadma_tx_lock);
	of_node_put(np);

	return 0;
err_put_node:
	of_node_put(np);
	return ret;
}

static int axienet_check_pm_space(int sid, int num_frag,
				  u32 wr, u32 rd, int total)
{
	int avail;

	avail = rd - wr;

	if (avail < 0)
		avail = total + avail;

	return (avail >= num_frag);
}

static int tadma_get_strid(struct sk_buff *skb,
			   struct net_device *ndev,
			   enum qtype qt)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct tadma_cb *cb = lp->t_cb;
	struct tadma_stream_entry *entry;
	struct vlan_ethhdr *vhdr = (struct vlan_ethhdr *)skb->data;
	int sid = -1; /* BE entry is always 0 */
	u32 idx;
	u16 vlan_tci;
	u8 mac_vlan[8];

	memcpy(mac_vlan, vhdr->h_dest, 6);

	vlan_tci = ntohs(vhdr->h_vlan_TCI);

	mac_vlan[6] = (vlan_tci >> 8) & 0x0f;
	mac_vlan[7] = (vlan_tci & 0xff);
	if (qt == qt_st && lp->default_st_sid >= 0)
		return lp->default_st_sid;
	else if (qt == qt_res && lp->default_res_sid >= 0)
		return lp->default_res_sid;

	idx = tadma_macvlan_hash(lp, mac_vlan);
	entry = tadma_hash_lookup_stream(&cb->stream_hash[idx],
					 mac_vlan);
	if (entry)
		return entry->sid;

	return sid;
}

static u32 tadma_stream_alm_offset(int sid, u32 wr, struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	u32 alm_offset;

	alm_offset = XTADMA_ALM_OFFSET +
		sid * sizeof(struct alm_entry) * lp->num_tadma_buffers;

	wr = (wr + lp->num_tadma_buffers - 1) & (lp->num_tadma_buffers - 1);

	return alm_offset + (wr * sizeof(struct alm_entry));
}

int axienet_tadma_xmit(struct sk_buff *skb, struct net_device *ndev,
		       u16 queue_type)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct alm_entry alm, alm_fframe = {0};
	dma_addr_t phys_addr;
	pm_entry_t pm;
	u32 num_frag, len, tot_len, alm_offset, alm_offset_fframe, write_p, read_p;
	unsigned long flags;
	int sid, ii, tot_sz8;
	static int chk_ptr;

	/* fetch stream ID */
	sid = tadma_get_strid(skb, ndev, lp->txqs[queue_type].dmaq_idx);

	if (sid < 0) {
		dev_kfree_skb_irq(skb);
		return NETDEV_TX_OK;
	}
	num_frag = skb_shinfo(skb)->nr_frags;

	spin_lock_irqsave(&lp->tadma_tx_lock, flags);
	pm = tadma_ior(lp, XTADMA_PM_OFFSET + (sid * sizeof(pm_entry_t)));

	read_p  = pm & XTADMA_PM_RD_MASK;
	write_p = (pm & XTADMA_PM_WR_MASK) >> XTADMA_PM_WR_SHIFT;

	if (!axienet_check_pm_space(sid, num_frag + 1, write_p, read_p,
				    lp->num_tadma_buffers)) {
		if (!chk_ptr) {
			pr_err("%s NO SPACE rd: %x wd: %x\n", __func__, read_p,
			       write_p);
			chk_ptr = 1;
		}

		netif_stop_subqueue(ndev, queue_type);
		spin_unlock_irqrestore(&lp->tadma_tx_lock, flags);
		return NETDEV_TX_BUSY;
	}
	if (((lp->tx_bd_head[sid] + (num_frag + 1)) % lp->num_tadma_buffers) ==
	    lp->tx_bd_tail[sid]) {
		netif_stop_subqueue(ndev, queue_type);
		spin_unlock_irqrestore(&lp->tadma_tx_lock, flags);
		return NETDEV_TX_BUSY;
	}

	/* get current alm offset */
	alm_offset_fframe = tadma_stream_alm_offset(sid, write_p, ndev);

	pr_debug("%d: num_frag: %d len: %d\n", sid, num_frag,
		 skb_headlen(skb));
	pr_debug("w:%d r:%d\n", write_p, read_p);

	tot_len = skb_headlen(skb);
	len = skb_headlen(skb);
	phys_addr = dma_map_single(ndev->dev.parent, skb->data,
				   len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ndev->dev.parent, phys_addr))) {
		dev_err(&ndev->dev, "tadma map error\n");
		spin_unlock_irqrestore(&lp->tadma_tx_lock, flags);
		return NETDEV_TX_BUSY;
	}

	alm_fframe.addr = (u32)phys_addr;

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	alm_fframe.cfg |= (u32)(phys_addr >> 32);
#endif

	lp->tx_bd[sid][lp->tx_bd_head[sid]].num_frag = num_frag + 1;
	if (num_frag == 0) {
		lp->tx_bd[sid][lp->tx_bd_head[sid]].tx_skb = skb;
		alm_fframe.cfg |= XTADMA_ALM_SOP | XTADMA_ALM_EOP;
	} else {
		lp->tx_bd[sid][lp->tx_bd_head[sid]].tx_skb = 0;
		alm_fframe.cfg |= XTADMA_ALM_SOP;
	}
	alm_fframe.cfg &= ~XTADMA_ALM_FETCH_SZ_MASK;
	alm_fframe.cfg |= ((len << XTADMA_ALM_FETCH_SZ_SHIFT) &
			   XTADMA_ALM_FETCH_SZ_MASK);
	lp->tx_bd[sid][lp->tx_bd_head[sid]].phys = phys_addr;
	lp->tx_bd[sid][lp->tx_bd_head[sid]].len = len;
	lp->tx_bd[sid][lp->tx_bd_head[sid]].tx_desc_mapping =
							DESC_DMA_MAP_SINGLE;
	lp->tx_bd_head[sid]++;
	lp->tx_bd_head[sid] %= lp->num_tadma_buffers;

	for (ii = 0; ii < num_frag; ii++) {
		skb_frag_t *frag;

		frag = &skb_shinfo(skb)->frags[ii];
		len = skb_frag_size(frag);
		tot_len += len;
		phys_addr = skb_frag_dma_map(ndev->dev.parent, frag, 0,
					     len, DMA_TO_DEVICE);
		memset(&alm, 0, sizeof(struct alm_entry));
		alm.addr = (u32)phys_addr;
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		alm.cfg |= (u32)(phys_addr >> 32);
#endif
		lp->tx_bd[sid][lp->tx_bd_head[sid]].tx_skb = 0;
		if (ii == (num_frag - 1)) {
			alm.cfg |= XTADMA_ALM_EOP;
			lp->tx_bd[sid][lp->tx_bd_head[sid]].tx_skb = skb;
		}
		alm.cfg &= ~XTADMA_ALM_FETCH_SZ_MASK;
		alm.cfg |= ((len << XTADMA_ALM_FETCH_SZ_SHIFT) &
				XTADMA_ALM_FETCH_SZ_MASK);
		alm.cfg |= XTADMA_ALM_UFF;
		lp->tx_bd[sid][lp->tx_bd_head[sid]].num_frag = 0;
		lp->tx_bd[sid][lp->tx_bd_head[sid]].phys = phys_addr;
		lp->tx_bd[sid][lp->tx_bd_head[sid]].len = len;
		lp->tx_bd[sid][lp->tx_bd_head[sid]].tx_desc_mapping =
							DESC_DMA_MAP_PAGE;
		lp->tx_bd_head[sid]++;
		lp->tx_bd_head[sid] %= lp->num_tadma_buffers;

		/* increment write */
		write_p = (write_p + 1) & (lp->num_tadma_buffers - 1);
		/* get current alm offset */
		alm_offset = tadma_stream_alm_offset(sid, write_p, ndev);

		tadma_iow(lp, alm_offset, alm.addr);
		tadma_iow(lp, alm_offset + 4, alm.cfg);
	}
	tot_sz8 = tot_len / 8 + 1;
	alm_fframe.cfg &= ~XTADMA_ALM_TOT_PKT_SZ_BY8_MASK;
	alm_fframe.cfg |= ((tot_sz8 << XTADMA_ALM_TOT_PKT_SZ_BY8_SHIFT) &
			  XTADMA_ALM_TOT_PKT_SZ_BY8_MASK);
	alm_fframe.cfg |= XTADMA_ALM_UFF;

	tadma_iow(lp, alm_offset_fframe, alm_fframe.addr);
	tadma_iow(lp, alm_offset_fframe + 4, alm_fframe.cfg);

	/* increment write */
	write_p = (write_p + 1) & (lp->num_tadma_buffers - 1);

	pm &= ~XTADMA_PM_WR_MASK;
	pm |= (write_p << XTADMA_PM_WR_SHIFT);

	tadma_iow(lp, (XTADMA_PM_OFFSET + (sid * sizeof(pm_entry_t))), pm);
	spin_unlock_irqrestore(&lp->tadma_tx_lock, flags);

	return NETDEV_TX_OK;
}

int axienet_tadma_program(struct net_device *ndev, void __user *useraddr)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct tadma_stream_entry *entry;
	struct tadma_cb *cb = lp->t_cb;
	struct hlist_head *bucket;
	bool res_enabled = false;
	bool st_enabled = false;
	struct hlist_node *tmp;
	u32 cr, hash = 0;

	for (hash = 0; hash < lp->num_entries; hash++) {
		int ret;

		bucket = &cb->stream_hash[hash];
		hlist_for_each_entry_safe(entry, tmp, bucket, hash_link) {
			ret = tadma_sfm_program(ndev, entry->sid, entry->qtype,
						entry->tticks, entry->count);
			if (ret)
				return ret;

			if (entry->qtype == qt_st) {
				lp->default_st_sid = -1;
				st_enabled = true;
			} else if (entry->qtype == qt_res) {
				lp->default_res_sid = -1;
				res_enabled = true;
			}
		}
	}

	if (!res_enabled && tadma_queue_enabled(lp, qt_res))
		lp->default_res_sid = tadma_set_contiguous_mode(ndev, qt_res);

	if (!st_enabled && tadma_queue_enabled(lp, qt_st))
		lp->default_st_sid = tadma_set_contiguous_mode(ndev, qt_st);

	/* flip memory first so access other sfm bank
	 * cr = tadma_ior(lp, XTADMA_CR_OFFSET);
	 * cr |= XTADMA_FLIP_FETCH_MEM;
	 * tadma_iow(lp, XTADMA_CR_OFFSET, cr);
	 */

	/* re-enable interrupts */
	tadma_iow(lp, XTADMA_INT_EN_OFFSET, XTADMA_FFI_INT_EN |
		  XTADMA_IE_INT_EN);
	/* enable schedule */
	cr = XTADMA_CFG_DONE | XTADMA_SCHED_ENABLE;
	tadma_iow(lp, XTADMA_CR_OFFSET, cr);

	return 0;
}

int axienet_tadma_off(struct net_device *ndev, void __user *useraddr)
{
	struct axienet_local *lp = netdev_priv(ndev);
	int ret;

	tadma_iow(lp, XTADMA_INT_EN_OFFSET, XTADMA_FFI_INT_EN |
		  XTADMA_IE_INT_EN);
	ret = tadma_sfm_init(ndev);
	tadma_iow(lp, XTADMA_CR_OFFSET, XTADMA_CFG_DONE);
	return ret;
}

int axienet_tadma_flush_stream(struct net_device *ndev, void __user *useraddr)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct tadma_cb *cb = lp->t_cb;
	struct tadma_stream_entry *entry;
	struct hlist_head *bucket;
	struct hlist_node *tmp;
	u32 offset;
	int hash;

	lp->get_sid = 0;
	lp->get_sfm = 0;
	/* set CFG_DONE to 0 */
	tadma_iow(lp, XTADMA_CR_OFFSET, 0);

	for (hash = 0; hash < lp->num_entries; hash++) {
		offset = sfm_entry_offset(lp, hash);
		tadma_iow(lp, offset, 0);
		tadma_iow(lp, offset + 4, 0);

		bucket = &cb->stream_hash[hash];
		hlist_for_each_entry_safe(entry, tmp, bucket, hash_link) {
			hlist_del(&entry->hash_link);
			kfree(entry);
		}
	}

	return 0;
}

int axienet_tadma_add_stream(struct net_device *ndev, void __user *useraddr)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct tadma_stream_entry *entry;
	struct tadma_cb *cb = lp->t_cb;
	struct tadma_stream stream;
	enum qtype qtype;
	u8 mac_vlan[8];
	u32 idx, sid;
	u16 vlan_tci;

	if (copy_from_user(&stream, useraddr, sizeof(struct tadma_stream)))
		return -EFAULT;

	if (stream.count > MAX_TRIG_COUNT)
		return -EINVAL;

	if (lp->num_tc <= XAE_MAX_LEGACY_TSN_TC) {
		dev_dbg(&ndev->dev, "Legacy design with single ST queue\n");
		qtype = qt_st;
	} else {
		if (stream.qno >= lp->num_tc)
			return -EINVAL;

		if (!lp->txqs[stream.qno].is_tadma) {
			dev_err(&ndev->dev,
				"Queue %d is not an ST traffic queue\n",
				stream.qno);
			return -EINVAL;
		}

		qtype = lp->txqs[stream.qno].dmaq_idx;
	}

	memcpy(mac_vlan, stream.dmac, 6);

	vlan_tci = stream.vid & VLAN_VID_MASK;
	mac_vlan[6] = (vlan_tci >> 8) & 0x0f;
	mac_vlan[7] = (vlan_tci & 0xff);

	idx = tadma_macvlan_hash(lp, mac_vlan);

	entry = tadma_hash_lookup_stream(&cb->stream_hash[idx], mac_vlan);
	if (entry && entry->count == stream.count &&
	    entry->tticks == stream.trigger) {
		return -EEXIST;	/*same entry*/
	}

	if (entry)
		sid = entry->sid;	/*same sid diff entry*/
	else
		sid = lp->get_sid++;

	if (sid >= lp->num_streams) {
		pr_err("More no. of streams %d\n", sid);
		return -EINVAL;
	}

	if (lp->get_sfm >= lp->num_entries) {
		pr_err("\nMore no. of entries %d\n", lp->get_sfm + 1);
		return -EINVAL;
	}

	lp->get_sfm++;
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->tticks = stream.trigger;
	entry->count = stream.count;
	entry->sid = sid;
	entry->qtype = qtype;
	memcpy(entry->macvlan, mac_vlan, 8);

	pr_debug("%s sid: %d, qtype %d\n", __func__, sid, entry->qtype);
	hlist_add_head(&entry->hash_link, &cb->stream_hash[idx]);

	return 0;
}
