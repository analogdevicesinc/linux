/* Copyright 2008-2016 Freescale Semiconductor Inc.
 * Copyright 2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/init.h>
#include "dpaa_eth_ceetm.h"

const struct nla_policy ceetm_policy[TCA_CEETM_MAX + 1] = {
	[TCA_CEETM_COPT] = { .len = sizeof(struct tc_ceetm_copt) },
	[TCA_CEETM_QOPS] = { .len = sizeof(struct tc_ceetm_qopt) },
};

struct Qdisc_ops ceetm_qdisc_ops;
EXPORT_SYMBOL(ceetm_qdisc_ops);

/* Obtain the DCP and the SP ids from the FMan port */
static void get_dcp_and_sp(struct net_device *dev, enum qm_dc_portal *dcp_id,
			   unsigned int *sp_id)
{
	struct dpa_priv_s *dpa_priv = netdev_priv(dev);
	struct mac_device *mac_dev = dpa_priv->mac_dev;
	t_LnxWrpFmPortDev *port_dev;
	uint32_t channel;

	port_dev = (t_LnxWrpFmPortDev *)mac_dev->port_dev[TX];
	channel = port_dev->txCh;

	*sp_id = channel & CHANNEL_SP_MASK;
	pr_debug(KBUILD_BASENAME " : FM sub-portal ID %d\n", *sp_id);

	if (channel < DCP0_MAX_CHANNEL) {
		*dcp_id = qm_dc_portal_fman0;
		pr_debug(KBUILD_BASENAME " : DCP ID 0\n");
	} else {
		*dcp_id = qm_dc_portal_fman1;
		pr_debug(KBUILD_BASENAME " : DCP ID 1\n");
	}
}

/* Wait for the DPAA Eth driver WQ TX FQs to empty */
static void dpaa_drain_fqs(struct net_device *dev)
{
	const struct dpa_priv_s *priv = netdev_priv(dev);
	struct qm_mcr_queryfq_np np;
	struct qman_fq *fq;
	int ret, i;

	for (i = 0; i < DPAA_ETH_TX_QUEUES; i++) {
		fq = priv->egress_fqs[i];
		while (true) {
			ret = qman_query_fq_np(fq, &np);
			if (unlikely(ret)) {
				pr_err(KBUILD_BASENAME
				       " : %s : unable to query FQ %x: %d\n",
				       __func__, fq->fqid, ret);
				break;
			}

			if (np.frm_cnt == 0)
				break;
		}
	}
}

/* Wait for the DPAA CEETM TX CQs to empty */
static void ceetm_drain_class(struct ceetm_class *cl)
{
	struct qm_mcr_ceetm_cq_query cq_query;
	struct qm_ceetm_cq *cq = NULL;
	unsigned int idx;
	int ret;

	if (!cl)
		return;

	switch (cl->type) {
	case CEETM_ROOT:
		/* The ROOT classes aren't directly linked to CEETM CQs */
		return;
	case CEETM_PRIO:
		cq = (struct qm_ceetm_cq *)cl->prio.cq;
		break;
	case CEETM_WBFS:
		cq = (struct qm_ceetm_cq *)cl->wbfs.cq;
		break;
	}

	if (!cq || !cl->ch)
		return;

	/* Build the query CQID by merging the channel and the CQ IDs */
	idx = (cq->parent->idx << 4) | cq->idx;

	while (true) {
		ret = qman_ceetm_query_cq(idx,
					  cl->ch->dcp_idx,
					  &cq_query);
		if (unlikely(ret)) {
			pr_err(KBUILD_BASENAME
			       " : %s : unable to query CQ %x: %d\n",
			       __func__, idx, ret);
			break;
		}

		if (cq_query.frm_cnt == 0)
			break;
	}
}

/* Enqueue Rejection Notification callback */
static void ceetm_ern(struct qman_portal *portal, struct qman_fq *fq,
		      const struct qm_mr_entry *msg)
{
	struct dpa_percpu_priv_s *dpa_percpu_priv;
	struct ceetm_class_stats *cstats = NULL;
	const struct dpa_priv_s *dpa_priv;
	struct qm_fd fd = msg->ern.fd;
	struct net_device *net_dev;
	struct ceetm_fq *ceetm_fq;
	struct ceetm_class *cls;
	struct sk_buff *skb;

	ceetm_fq = container_of(fq, struct ceetm_fq, fq);
	net_dev = ceetm_fq->net_dev;
	dpa_priv = netdev_priv(net_dev);
	dpa_percpu_priv = raw_cpu_ptr(dpa_priv->percpu_priv);

	/* Increment DPA counters */
	dpa_percpu_priv->stats.tx_dropped++;
	dpa_percpu_priv->stats.tx_fifo_errors++;
	count_ern(dpa_percpu_priv, msg);

	/* Increment CEETM counters */
	cls = ceetm_fq->ceetm_cls;
	switch (cls->type) {
	case CEETM_PRIO:
		cstats = this_cpu_ptr(cls->prio.cstats);
		break;
	case CEETM_WBFS:
		cstats = this_cpu_ptr(cls->wbfs.cstats);
		break;
	}

	if (cstats)
		cstats->ern_drop_count++;

	/* Release the buffers that were supposed to be recycled. */
	if (fd.bpid != 0xff) {
		dpa_fd_release(net_dev, &fd);
		return;
	}

	/* Release the frames that were supposed to return on the
	 * confirmation path.
	 */
	skb = _dpa_cleanup_tx_fd(dpa_priv, &fd);
	dev_kfree_skb_any(skb);
}

/* Congestion State Change Notification callback */
static void ceetm_cscn(struct qm_ceetm_ccg *ccg, void *cb_ctx, int congested)
{
	struct ceetm_class_stats *cstats = NULL;
	struct dpa_priv_s *dpa_priv;
	struct ceetm_fq *ceetm_fq;
	struct ceetm_class *cls;

	ceetm_fq = (struct ceetm_fq *)cb_ctx;
	dpa_priv = netdev_priv(ceetm_fq->net_dev);
	cls = ceetm_fq->ceetm_cls;

	switch (cls->type) {
	case CEETM_PRIO:
		cstats = this_cpu_ptr(cls->prio.cstats);
		break;
	case CEETM_WBFS:
		cstats = this_cpu_ptr(cls->wbfs.cstats);
		break;
	}

	ceetm_fq->congested = congested;

	if (congested) {
		dpa_priv->cgr_data.congestion_start_jiffies = jiffies;
		dpa_priv->cgr_data.cgr_congested_count++;
		if (cstats)
			cstats->congested_count++;
	} else {
		dpa_priv->cgr_data.congested_jiffies +=
			(jiffies - dpa_priv->cgr_data.congestion_start_jiffies);
	}
}

/* Allocate a ceetm fq */
static int ceetm_alloc_fq(struct ceetm_fq **fq, struct net_device *dev,
			  struct ceetm_class *cls)
{
	*fq = kzalloc(sizeof(**fq), GFP_KERNEL);
	if (!*fq)
		return -ENOMEM;

	(*fq)->net_dev = dev;
	(*fq)->ceetm_cls = cls;
	(*fq)->congested = 0;
	return 0;
}

/* Configure a ceetm Class Congestion Group */
static int ceetm_config_ccg(struct qm_ceetm_ccg **ccg,
			    struct qm_ceetm_channel *channel, unsigned int id,
			    struct ceetm_fq *fq, struct dpa_priv_s *dpa_priv)
{
	struct qm_ceetm_ccg_params ccg_params;
	u16 ccg_mask;
	u32 cs_th;
	int err;

	err = qman_ceetm_ccg_claim(ccg, channel, id, ceetm_cscn, fq);
	if (err)
		return err;

	/* Configure the count mode (frames/bytes), enable congestion state
	 * notifications, configure the congestion entry and exit thresholds,
	 * enable tail-drop, configure the tail-drop mode, and set the
	 * overhead accounting limit
	 */
	ccg_mask = QM_CCGR_WE_MODE |
		   QM_CCGR_WE_CSCN_EN |
		   QM_CCGR_WE_CS_THRES_IN | QM_CCGR_WE_CS_THRES_OUT |
		   QM_CCGR_WE_TD_EN | QM_CCGR_WE_TD_MODE |
		   QM_CCGR_WE_OAL;

	memset(&ccg_params, 0, sizeof(ccg_params));
	ccg_params.mode = 0; /* count bytes */
	ccg_params.cscn_en = 1; /* generate notifications */
	ccg_params.td_en = 1; /* enable tail-drop */
	ccg_params.td_mode = 0; /* tail-drop on congestion state */
	ccg_params.oal = (signed char)(min(sizeof(struct sk_buff) +
			  dpa_priv->tx_headroom, (size_t)FSL_QMAN_MAX_OAL));

	/* Set the congestion state thresholds according to the link speed */
	if (dpa_priv->mac_dev->if_support & SUPPORTED_10000baseT_Full)
		cs_th = CONFIG_FSL_DPAA_CEETM_CCS_THRESHOLD_10G;
	else
		cs_th = CONFIG_FSL_DPAA_CEETM_CCS_THRESHOLD_1G;

	qm_cgr_cs_thres_set64(&ccg_params.cs_thres_in, cs_th, 1);
	qm_cgr_cs_thres_set64(&ccg_params.cs_thres_out,
			      cs_th * CEETM_CCGR_RATIO, 1);

	err = qman_ceetm_ccg_set(*ccg, ccg_mask, &ccg_params);
	if (err)
		return err;

	return 0;
}

/* Configure a ceetm Logical Frame Queue */
static int ceetm_config_lfq(struct qm_ceetm_cq *cq, struct ceetm_fq *fq,
			    struct qm_ceetm_lfq **lfq)
{
	u64 context_a;
	u32 context_b;
	int err;

	err = qman_ceetm_lfq_claim(lfq, cq);
	if (err)
		return err;

	/* Get the former contexts in order to preserve context B */
	err = qman_ceetm_lfq_get_context(*lfq, &context_a, &context_b);
	if (err)
		return err;

	context_a = CEETM_CONTEXT_A;
	err = qman_ceetm_lfq_set_context(*lfq, context_a, context_b);
	if (err)
		return err;

	(*lfq)->ern = ceetm_ern;

	err = qman_ceetm_create_fq(*lfq, &fq->fq);
	if (err)
		return err;

	return 0;
}

/* Configure a prio ceetm class */
static int ceetm_config_prio_cls(struct ceetm_class *cls,
				 struct net_device *dev,
				 unsigned int id)
{
	struct dpa_priv_s *dpa_priv = netdev_priv(dev);
	int err;

	err = ceetm_alloc_fq(&cls->prio.fq, dev, cls);
	if (err)
		return err;

	/* Claim and configure the CCG */
	err = ceetm_config_ccg(&cls->prio.ccg, cls->ch, id, cls->prio.fq,
			       dpa_priv);
	if (err)
		return err;

	/* Claim and configure the CQ */
	err = qman_ceetm_cq_claim(&cls->prio.cq, cls->ch, id, cls->prio.ccg);
	if (err)
		return err;

	if (cls->shaped) {
		err = qman_ceetm_channel_set_cq_cr_eligibility(cls->ch, id, 1);
		if (err)
			return err;

		err = qman_ceetm_channel_set_cq_er_eligibility(cls->ch, id, 1);
		if (err)
			return err;
	}

	/* Claim and configure a LFQ */
	err = ceetm_config_lfq(cls->prio.cq, cls->prio.fq, &cls->prio.lfq);
	if (err)
		return err;

	return 0;
}

/* Configure a wbfs ceetm class */
static int ceetm_config_wbfs_cls(struct ceetm_class *cls,
				 struct net_device *dev,
				 unsigned int id, int type)
{
	struct dpa_priv_s *dpa_priv = netdev_priv(dev);
	int err;

	err = ceetm_alloc_fq(&cls->wbfs.fq, dev, cls);
	if (err)
		return err;

	/* Claim and configure the CCG */
	err = ceetm_config_ccg(&cls->wbfs.ccg, cls->ch, id, cls->wbfs.fq,
			       dpa_priv);
	if (err)
		return err;

	/* Claim and configure the CQ */
	if (type == WBFS_GRP_B)
		err = qman_ceetm_cq_claim_B(&cls->wbfs.cq, cls->ch, id,
					    cls->wbfs.ccg);
	else
		err = qman_ceetm_cq_claim_A(&cls->wbfs.cq, cls->ch, id,
					    cls->wbfs.ccg);
	if (err)
		return err;

	/* Configure the CQ weight: real number multiplied by 100 to get rid
	 * of the fraction
	 */
	err = qman_ceetm_set_queue_weight_in_ratio(cls->wbfs.cq,
						   cls->wbfs.weight * 100);
	if (err)
		return err;

	/* Claim and configure a LFQ */
	err = ceetm_config_lfq(cls->wbfs.cq, cls->wbfs.fq, &cls->wbfs.lfq);
	if (err)
		return err;

	return 0;
}

/* Find class in qdisc hash table using given handle */
static inline struct ceetm_class *ceetm_find(u32 handle, struct Qdisc *sch)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct Qdisc_class_common *clc;

	pr_debug(KBUILD_BASENAME " : %s : find class %X in qdisc %X\n",
		 __func__, handle, sch->handle);

	clc = qdisc_class_find(&priv->clhash, handle);
	return clc ? container_of(clc, struct ceetm_class, common) : NULL;
}

/* Insert a class in the qdisc's class hash */
static void ceetm_link_class(struct Qdisc *sch,
			     struct Qdisc_class_hash *clhash,
			     struct Qdisc_class_common *common)
{
	sch_tree_lock(sch);
	qdisc_class_hash_insert(clhash, common);
	sch_tree_unlock(sch);
	qdisc_class_hash_grow(sch, clhash);
}

/* Destroy a ceetm class */
static void ceetm_cls_destroy(struct Qdisc *sch, struct ceetm_class *cl)
{
	struct net_device *dev = qdisc_dev(sch);

	if (!cl)
		return;

	pr_debug(KBUILD_BASENAME " : %s : destroy class %X from under %X\n",
		 __func__, cl->common.classid, sch->handle);

	switch (cl->type) {
	case CEETM_ROOT:
		if (cl->root.child) {
			qdisc_put(cl->root.child);
			cl->root.child = NULL;
		}

		if (cl->ch && qman_ceetm_channel_release(cl->ch))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the channel %d\n",
			       __func__, cl->ch->idx);

		break;

	case CEETM_PRIO:
		if (cl->prio.child) {
			qdisc_put(cl->prio.child);
			cl->prio.child = NULL;
		}

		/* We must make sure the CQ is empty before releasing it.
		 * Pause all transmissions while we wait for it to drain.
		 */
		netif_tx_stop_all_queues(dev);
		ceetm_drain_class(cl);

		if (cl->prio.lfq && qman_ceetm_lfq_release(cl->prio.lfq))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the LFQ %d\n",
			       __func__, cl->prio.lfq->idx);

		if (cl->prio.cq && qman_ceetm_cq_release(cl->prio.cq))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the CQ %d\n",
			       __func__, cl->prio.cq->idx);

		if (cl->prio.ccg && qman_ceetm_ccg_release(cl->prio.ccg))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the CCG %d\n",
			       __func__, cl->prio.ccg->idx);

		kfree(cl->prio.fq);

		if (cl->prio.cstats)
			free_percpu(cl->prio.cstats);

		netif_tx_wake_all_queues(dev);
		break;

	case CEETM_WBFS:
		/* We must make sure the CQ is empty before releasing it.
		 * Pause all transmissions while we wait for it to drain.
		 */
		netif_tx_stop_all_queues(dev);
		ceetm_drain_class(cl);

		if (cl->wbfs.lfq && qman_ceetm_lfq_release(cl->wbfs.lfq))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the LFQ %d\n",
			       __func__, cl->wbfs.lfq->idx);

		if (cl->wbfs.cq && qman_ceetm_cq_release(cl->wbfs.cq))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the CQ %d\n",
			       __func__, cl->wbfs.cq->idx);

		if (cl->wbfs.ccg && qman_ceetm_ccg_release(cl->wbfs.ccg))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the CCG %d\n",
			       __func__, cl->wbfs.ccg->idx);

		kfree(cl->wbfs.fq);

		if (cl->wbfs.cstats)
			free_percpu(cl->wbfs.cstats);

		netif_tx_wake_all_queues(dev);
	}

	tcf_block_put(cl->block);
	kfree(cl);
}

/* Destroy a ceetm qdisc */
static void ceetm_destroy(struct Qdisc *sch)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct net_device *dev = qdisc_dev(sch);
	struct hlist_node *next;
	struct ceetm_class *cl;
	unsigned int ntx, i;

	pr_debug(KBUILD_BASENAME " : %s : destroy qdisc %X\n",
		 __func__, sch->handle);

	/* All filters need to be removed before destroying the classes */
	tcf_block_put(priv->block);

	for (i = 0; i < priv->clhash.hashsize; i++) {
		hlist_for_each_entry(cl, &priv->clhash.hash[i], common.hnode) {
			tcf_block_put(cl->block);
			cl->block = NULL;
		}
	}

	for (i = 0; i < priv->clhash.hashsize; i++) {
		hlist_for_each_entry_safe(cl, next, &priv->clhash.hash[i],
					  common.hnode)
			ceetm_cls_destroy(sch, cl);
	}

	qdisc_class_hash_destroy(&priv->clhash);

	switch (priv->type) {
	case CEETM_ROOT:
		dpa_disable_ceetm(dev);

		if (priv->root.lni && qman_ceetm_lni_release(priv->root.lni))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the LNI %d\n",
			       __func__, priv->root.lni->idx);

		if (priv->root.sp && qman_ceetm_sp_release(priv->root.sp))
			pr_err(KBUILD_BASENAME
			       " : %s : error releasing the SP %d\n",
			       __func__, priv->root.sp->idx);

		if (priv->root.qstats)
			free_percpu(priv->root.qstats);

		if (!priv->root.qdiscs)
			break;

		/* Destroy the pfifo qdiscs in case they haven't been attached
		 * to the netdev queues yet.
		 */
		for (ntx = 0; ntx < dev->num_tx_queues; ntx++)
			if (priv->root.qdiscs[ntx])
				qdisc_put(priv->root.qdiscs[ntx]);

		kfree(priv->root.qdiscs);
		break;

	case CEETM_PRIO:
		if (priv->prio.parent)
			priv->prio.parent->root.child = NULL;
		break;

	case CEETM_WBFS:
		/* Reset the WBFS groups and priorities */
		if (priv->wbfs.ch)
			qman_ceetm_channel_set_group(priv->wbfs.ch, 1, 0, 0);

		if (priv->wbfs.parent)
			priv->wbfs.parent->prio.child = NULL;
		break;
	}
}

static int ceetm_dump(struct Qdisc *sch, struct sk_buff *skb)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct net_device *dev = qdisc_dev(sch);
	struct ceetm_qdisc_stats *qstats;
	struct tc_ceetm_qopt qopt;
	struct Qdisc *qdisc;
	unsigned int ntx, i;
	struct nlattr *nest;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	sch_tree_lock(sch);
	memset(&qopt, 0, sizeof(qopt));
	qopt.type = priv->type;
	qopt.shaped = priv->shaped;

	switch (priv->type) {
	case CEETM_ROOT:
		/* Gather statistics from the underlying pfifo qdiscs */
		sch->q.qlen = 0;
		gnet_stats_basic_sync_init(&sch->bstats);
		memset(&sch->qstats, 0, sizeof(sch->qstats));

		for (ntx = 0; ntx < dev->num_tx_queues; ntx++) {
			qdisc = netdev_get_tx_queue(dev, ntx)->qdisc_sleeping;
			sch->q.qlen		+= qdisc->q.qlen;
			u64_stats_add(&sch->bstats.bytes,
				      u64_stats_read(&qdisc->bstats.bytes));
			u64_stats_add(&sch->bstats.packets,
				      u64_stats_read(&qdisc->bstats.packets));
			sch->qstats.qlen	+= qdisc->qstats.qlen;
			sch->qstats.backlog	+= qdisc->qstats.backlog;
			sch->qstats.drops	+= qdisc->qstats.drops;
			sch->qstats.requeues	+= qdisc->qstats.requeues;
			sch->qstats.overlimits	+= qdisc->qstats.overlimits;
		}

		for_each_online_cpu(i) {
			qstats = per_cpu_ptr(priv->root.qstats, i);
			sch->qstats.drops += qstats->drops;
		}

		qopt.rate = priv->root.rate;
		qopt.ceil = priv->root.ceil;
		qopt.overhead = priv->root.overhead;
		break;

	case CEETM_PRIO:
		qopt.qcount = priv->prio.qcount;
		break;

	case CEETM_WBFS:
		qopt.qcount = priv->wbfs.qcount;
		qopt.cr = priv->wbfs.cr;
		qopt.er = priv->wbfs.er;
		break;

	default:
		pr_err(KBUILD_BASENAME " : %s : invalid qdisc\n", __func__);
		sch_tree_unlock(sch);
		return -EINVAL;
	}

	nest = nla_nest_start_noflag(skb, TCA_OPTIONS);
	if (!nest)
		goto nla_put_failure;
	if (nla_put(skb, TCA_CEETM_QOPS, sizeof(qopt), &qopt))
		goto nla_put_failure;
	nla_nest_end(skb, nest);

	sch_tree_unlock(sch);
	return skb->len;

nla_put_failure:
	sch_tree_unlock(sch);
	nla_nest_cancel(skb, nest);
	return -EMSGSIZE;
}

/* Configure a root ceetm qdisc */
static int ceetm_init_root(struct Qdisc *sch, struct ceetm_qdisc *priv,
			   struct tc_ceetm_qopt *qopt,
			   struct netlink_ext_ack *extack)
{
	struct net_device *dev = qdisc_dev(sch);
	unsigned int i, sp_id, parent_id;
	struct netdev_queue *dev_queue;
	struct dpa_priv_s *dpa_priv;
	struct mac_device *mac_dev;
	enum qm_dc_portal dcp_id;
	struct qm_ceetm_lni *lni;
	struct qm_ceetm_sp *sp;
	struct Qdisc *qdisc;
	int err;
	u64 bps;

	dpa_priv = netdev_priv(dev);
	mac_dev = dpa_priv->mac_dev;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	/* Validate inputs */
	if (sch->parent != TC_H_ROOT) {
		pr_err("CEETM: a root ceetm qdisc must be root\n");
		return -EINVAL;
	}

	if (!mac_dev) {
		pr_err("CEETM: the interface is lacking a mac\n");
		return -EINVAL;
	}

	/* Pre-allocate underlying pfifo qdiscs.
	 *
	 * We want to offload shaping and scheduling decisions to the hardware.
	 * The pfifo qdiscs will be attached to the netdev queues and will
	 * guide the traffic from the IP stack down to the driver with minimum
	 * interference.
	 *
	 * The CEETM qdiscs and classes will be crossed when the traffic
	 * reaches the driver.
	 */
	priv->root.qdiscs = kcalloc(dev->num_tx_queues,
				    sizeof(priv->root.qdiscs[0]),
				    GFP_KERNEL);
	if (!priv->root.qdiscs)
		return -ENOMEM;

	for (i = 0; i < dev->num_tx_queues; i++) {
		dev_queue = netdev_get_tx_queue(dev, i);
		parent_id = TC_H_MAKE(TC_H_MAJ(sch->handle),
				      TC_H_MIN(i + PFIFO_MIN_OFFSET));

		qdisc = qdisc_create_dflt(dev_queue, &pfifo_qdisc_ops,
					  parent_id, extack);
		if (!qdisc)
			return -ENOMEM;

		priv->root.qdiscs[i] = qdisc;
		qdisc->flags |= TCQ_F_ONETXQUEUE;
	}

	sch->flags |= TCQ_F_MQROOT;

	priv->root.qstats = alloc_percpu(struct ceetm_qdisc_stats);
	if (!priv->root.qstats) {
		pr_err(KBUILD_BASENAME " : %s : alloc_percpu() failed\n",
		       __func__);
		return -ENOMEM;
	}

	priv->shaped = qopt->shaped;
	priv->root.rate = qopt->rate;
	priv->root.ceil = qopt->ceil;
	priv->root.overhead = qopt->overhead;

	/* Claim the SP */
	get_dcp_and_sp(dev, &dcp_id, &sp_id);
	err = qman_ceetm_sp_claim(&sp, dcp_id, sp_id);
	if (err) {
		pr_err(KBUILD_BASENAME " : %s : failed to claim the SP\n",
		       __func__);
		return err;
	}

	priv->root.sp = sp;

	/* Claim the LNI - will use the same id as the SP id since SPs 0-7
	 * are connected to the TX FMan ports
	 */
	err = qman_ceetm_lni_claim(&lni, dcp_id, sp_id);
	if (err) {
		pr_err(KBUILD_BASENAME " : %s : failed to claim the LNI\n",
		       __func__);
		return err;
	}

	priv->root.lni = lni;

	err = qman_ceetm_sp_set_lni(sp, lni);
	if (err) {
		pr_err(KBUILD_BASENAME " : %s : failed to link the SP and LNI\n",
		       __func__);
		return err;
	}

	lni->sp = sp;

	/* Configure the LNI shaper */
	if (priv->shaped) {
		err = qman_ceetm_lni_enable_shaper(lni, 1, priv->root.overhead);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to configure the LNI shaper\n",
			       __func__);
			return err;
		}

		bps = priv->root.rate << 3; /* Bps -> bps */
		err = qman_ceetm_lni_set_commit_rate_bps(lni, bps, dev->mtu);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to configure the LNI shaper\n",
			       __func__);
			return err;
		}

		bps = priv->root.ceil << 3; /* Bps -> bps */
		err = qman_ceetm_lni_set_excess_rate_bps(lni, bps, dev->mtu);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to configure the LNI shaper\n",
			       __func__);
			return err;
		}
	}

	/* TODO default configuration */

	dpa_enable_ceetm(dev);
	return 0;
}

/* Configure a prio ceetm qdisc */
static int ceetm_init_prio(struct Qdisc *sch, struct ceetm_qdisc *priv,
			   struct tc_ceetm_qopt *qopt)
{
	struct ceetm_class *parent_cl, *child_cl;
	struct net_device *dev = qdisc_dev(sch);
	struct Qdisc *root_qdisc = dev->qdisc;
	struct ceetm_class_stats *cstats;
	unsigned int i, j;
	int err;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	if (sch->parent == TC_H_ROOT) {
		pr_err("CEETM: a prio ceetm qdisc can not be root\n");
		return -EINVAL;
	}

	if (TC_H_MAJ(sch->parent) != TC_H_MAJ(root_qdisc->handle)) {
		pr_err("CEETM: a prio ceetm qdiscs can be added only under a root ceetm class\n");
		return -EINVAL;
	}

	if (strcmp(root_qdisc->ops->id, ceetm_qdisc_ops.id)) {
		pr_err("CEETM: a ceetm qdisc can not be attached to other qdisc/class types\n");
		return -EINVAL;
	}

	/* Obtain the parent root ceetm_class */
	parent_cl = ceetm_find(sch->parent, root_qdisc);

	if (!parent_cl || parent_cl->type != CEETM_ROOT) {
		pr_err("CEETM: a prio ceetm qdiscs can be added only under a root ceetm class\n");
		return -EINVAL;
	}

	priv->prio.parent = parent_cl;
	parent_cl->root.child = sch;

	priv->shaped = parent_cl->shaped;
	priv->prio.qcount = qopt->qcount;
	priv->prio.ch = parent_cl->ch;

	/* Create and configure qcount child classes */
	for (i = 0; i < priv->prio.qcount; i++) {
		child_cl = kzalloc(sizeof(*child_cl), GFP_KERNEL);
		if (!child_cl)
			return -ENOMEM;

		child_cl->prio.cstats = alloc_percpu(struct ceetm_class_stats);
		if (!child_cl->prio.cstats) {
			pr_err(KBUILD_BASENAME " : %s : alloc_percpu() failed\n",
			       __func__);
			err = -ENOMEM;
			goto err_init_prio_cls;
		}

		for_each_online_cpu(j) {
			cstats = per_cpu_ptr(child_cl->prio.cstats, j);
			gnet_stats_basic_sync_init(&cstats->bstats);
		}

		child_cl->common.classid = TC_H_MAKE(sch->handle, (i + 1));
		child_cl->parent = sch;
		child_cl->type = CEETM_PRIO;
		child_cl->shaped = priv->shaped;
		child_cl->prio.child = NULL;
		child_cl->ch = priv->prio.ch;

		/* All shaped CQs have CR and ER enabled by default */
		child_cl->prio.cr = child_cl->shaped;
		child_cl->prio.er = child_cl->shaped;
		child_cl->prio.fq = NULL;
		child_cl->prio.cq = NULL;

		/* Configure the corresponding hardware CQ */
		err = ceetm_config_prio_cls(child_cl, dev, i);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to configure the ceetm prio class %X\n",
			       __func__, child_cl->common.classid);
			goto err_init_prio_cls;
		}

		/* Add class handle in Qdisc */
		ceetm_link_class(sch, &priv->clhash, &child_cl->common);
		pr_debug(KBUILD_BASENAME " : %s : added ceetm prio class %X associated with CQ %d and CCG %d\n",
			 __func__, child_cl->common.classid,
			child_cl->prio.cq->idx, child_cl->prio.ccg->idx);
	}

	return 0;

err_init_prio_cls:
	ceetm_cls_destroy(sch, child_cl);
	/* Note: ceetm_destroy() will be called by our caller */
	return err;
}

/* Configure a wbfs ceetm qdisc */
static int ceetm_init_wbfs(struct Qdisc *sch, struct ceetm_qdisc *priv,
			   struct tc_ceetm_qopt *qopt)
{
	struct ceetm_class *parent_cl, *child_cl, *tmp_cl, *root_cl = NULL;
	struct Qdisc *root_qdisc, *parent_qdisc = NULL;
	struct net_device *dev = qdisc_dev(sch);
	unsigned int i, j, id, prio_a, prio_b;
	struct ceetm_class_stats *cstats;
	int err, group_b, small_group;
	struct ceetm_qdisc *root_priv;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	/* Validate inputs */
	if (sch->parent == TC_H_ROOT) {
		pr_err("CEETM: a wbfs ceetm qdiscs can not be root\n");
		return -EINVAL;
	}

	root_qdisc = dev->qdisc;

	if (strcmp(root_qdisc->ops->id, ceetm_qdisc_ops.id)) {
		pr_err("CEETM: a ceetm qdisc can not be attached to other qdisc/class types\n");
		return -EINVAL;
	}

	root_priv = qdisc_priv(root_qdisc);

	/* Obtain the root ceetm class and the parent prio ceetm qdisc */
	for (i = 0; i < root_priv->clhash.hashsize; i++) {
		hlist_for_each_entry(tmp_cl, &root_priv->clhash.hash[i],
				     common.hnode) {
			if (tmp_cl->root.child &&
			    (TC_H_MAJ(tmp_cl->root.child->handle) ==
			    TC_H_MAJ(sch->parent))) {
				parent_qdisc = tmp_cl->root.child;
				root_cl = tmp_cl;
				break;
			}
		}
	}

	if (!parent_qdisc ||
	    strcmp(parent_qdisc->ops->id, ceetm_qdisc_ops.id)) {
		pr_err("CEETM: a wbfs ceetm qdiscs can be added only under a prio ceetm class\n");
		return -EINVAL;
	}

	/* Obtain the parent prio ceetm class */
	parent_cl = ceetm_find(sch->parent, parent_qdisc);

	if (!parent_cl || parent_cl->type != CEETM_PRIO) {
		pr_err("CEETM: a wbfs ceetm qdiscs can be added only under a prio ceetm class\n");
		return -EINVAL;
	}

	if (!qopt->qcount || !qopt->qweight[0]) {
		pr_err("CEETM: qcount and qweight are mandatory for a wbfs ceetm qdisc\n");
		return -EINVAL;
	}

	priv->shaped = parent_cl->shaped;

	if (!priv->shaped && (qopt->cr || qopt->er)) {
		pr_err("CEETM: CR/ER can be enabled only for shaped wbfs ceetm qdiscs\n");
		return -EINVAL;
	}

	if (priv->shaped && !(qopt->cr || qopt->er)) {
		pr_err("CEETM: either CR or ER must be enabled for shaped wbfs ceetm qdiscs\n");
		return -EINVAL;
	}

	if ((root_cl->root.wbfs_grp_a && root_cl->root.wbfs_grp_b) ||
	    root_cl->root.wbfs_grp_large) {
		pr_err("CEETM: no more wbfs classes are available\n");
		return -EINVAL;
	}

	if ((root_cl->root.wbfs_grp_a || root_cl->root.wbfs_grp_b) &&
	    qopt->qcount == CEETM_MAX_WBFS_QCOUNT) {
		pr_err("CEETM: only %d wbfs classes are available\n",
		       CEETM_MIN_WBFS_QCOUNT);
		return -EINVAL;
	}

	priv->wbfs.parent = parent_cl;
	parent_cl->prio.child = sch;

	priv->wbfs.qcount = qopt->qcount;
	priv->wbfs.cr = qopt->cr;
	priv->wbfs.er = qopt->er;
	priv->wbfs.ch = parent_cl->ch;

	/* Configure the hardware wbfs channel groups */
	if (priv->wbfs.qcount == CEETM_MAX_WBFS_QCOUNT) {
		/* Configure the large group A */
		priv->wbfs.group_type = WBFS_GRP_LARGE;
		small_group = false;
		group_b = false;
		prio_a = TC_H_MIN(parent_cl->common.classid) - 1;
		prio_b = prio_a;

	} else if (root_cl->root.wbfs_grp_a) {
		/* Configure the group B */
		priv->wbfs.group_type = WBFS_GRP_B;

		err = qman_ceetm_channel_get_group(priv->wbfs.ch, &small_group,
						   &prio_a, &prio_b);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to get group details\n",
			       __func__);
			return err;
		}

		small_group = true;
		group_b = true;
		prio_b = TC_H_MIN(parent_cl->common.classid) - 1;
		/* If group A isn't configured, configure it as group B */
		prio_a = prio_a ? : prio_b;

	} else {
		/* Configure the small group A */
		priv->wbfs.group_type = WBFS_GRP_A;

		err = qman_ceetm_channel_get_group(priv->wbfs.ch, &small_group,
						   &prio_a, &prio_b);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to get group details\n",
			       __func__);
			return err;
		}

		small_group = true;
		group_b = false;
		prio_a = TC_H_MIN(parent_cl->common.classid) - 1;
		/* If group B isn't configured, configure it as group A */
		prio_b = prio_b ? : prio_a;
	}

	err = qman_ceetm_channel_set_group(priv->wbfs.ch, small_group, prio_a,
					   prio_b);
	if (err)
		return err;

	if (priv->shaped) {
		err = qman_ceetm_channel_set_group_cr_eligibility(priv->wbfs.ch,
								  group_b,
								priv->wbfs.cr);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to set group CR eligibility\n",
			       __func__);
			return err;
		}

		err = qman_ceetm_channel_set_group_er_eligibility(priv->wbfs.ch,
								  group_b,
								priv->wbfs.er);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to set group ER eligibility\n",
			       __func__);
			return err;
		}
	}

	/* Create qcount child classes */
	for (i = 0; i < priv->wbfs.qcount; i++) {
		child_cl = kzalloc(sizeof(*child_cl), GFP_KERNEL);
		if (!child_cl)
			return -ENOMEM;

		child_cl->wbfs.cstats = alloc_percpu(struct ceetm_class_stats);
		if (!child_cl->wbfs.cstats) {
			pr_err(KBUILD_BASENAME " : %s : alloc_percpu() failed\n",
			       __func__);
			err = -ENOMEM;
			goto err_init_wbfs_cls;
		}

		for_each_online_cpu(j) {
			cstats = per_cpu_ptr(child_cl->wbfs.cstats, j);
			gnet_stats_basic_sync_init(&cstats->bstats);
		}

		child_cl->common.classid = TC_H_MAKE(sch->handle, (i + 1));
		child_cl->parent = sch;
		child_cl->type = CEETM_WBFS;
		child_cl->shaped = priv->shaped;
		child_cl->wbfs.fq = NULL;
		child_cl->wbfs.cq = NULL;
		child_cl->wbfs.weight = qopt->qweight[i];
		child_cl->ch = priv->wbfs.ch;

		if (priv->wbfs.group_type == WBFS_GRP_B)
			id = WBFS_GRP_B_OFFSET + i;
		else
			id = WBFS_GRP_A_OFFSET + i;

		err = ceetm_config_wbfs_cls(child_cl, dev, id,
					    priv->wbfs.group_type);
		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to configure the ceetm wbfs class %X\n",
			       __func__, child_cl->common.classid);
			goto err_init_wbfs_cls;
		}

		/* Add class handle in Qdisc */
		ceetm_link_class(sch, &priv->clhash, &child_cl->common);
		pr_debug(KBUILD_BASENAME " : %s : added ceetm wbfs class %X associated with CQ %d and CCG %d\n",
			 __func__, child_cl->common.classid,
			 child_cl->wbfs.cq->idx, child_cl->wbfs.ccg->idx);
	}

	/* Signal the root class that a group has been configured */
	switch (priv->wbfs.group_type) {
	case WBFS_GRP_LARGE:
		root_cl->root.wbfs_grp_large = true;
		break;
	case WBFS_GRP_A:
		root_cl->root.wbfs_grp_a = true;
		break;
	case WBFS_GRP_B:
		root_cl->root.wbfs_grp_b = true;
		break;
	}

	return 0;

err_init_wbfs_cls:
	ceetm_cls_destroy(sch, child_cl);
	/* Note: ceetm_destroy() will be called by our caller */
	return err;
}

/* Configure a generic ceetm qdisc */
static int ceetm_init(struct Qdisc *sch, struct nlattr *opt,
		      struct netlink_ext_ack *extack)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct net_device *dev = qdisc_dev(sch);
	struct nlattr *tb[TCA_CEETM_QOPS + 1];
	struct tc_ceetm_qopt *qopt;
	int ret;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	if (!netif_is_multiqueue(dev))
		return -EOPNOTSUPP;

	if (!opt) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return -EINVAL;
	}

	ret = tcf_block_get(&priv->block, &priv->filter_list, sch, extack);
	if (ret)
		return ret;

	ret = nla_parse_nested_deprecated(tb, TCA_CEETM_QOPS, opt,
					  ceetm_policy, NULL);
	if (ret < 0) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return ret;
	}

	if (!tb[TCA_CEETM_QOPS]) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return -EINVAL;
	}

	if (TC_H_MIN(sch->handle)) {
		pr_err("CEETM: a qdisc should not have a minor\n");
		return -EINVAL;
	}

	qopt = nla_data(tb[TCA_CEETM_QOPS]);

	/* Initialize the class hash list. Each qdisc has its own class hash */
	ret = qdisc_class_hash_init(&priv->clhash);
	if (ret < 0) {
		pr_err(KBUILD_BASENAME " : %s : qdisc_class_hash_init failed\n",
		       __func__);
		return ret;
	}

	priv->type = qopt->type;

	switch (priv->type) {
	case CEETM_ROOT:
		netif_tx_stop_all_queues(dev);
		dpaa_drain_fqs(dev);
		ret = ceetm_init_root(sch, priv, qopt, extack);
		netif_tx_wake_all_queues(dev);
		break;
	case CEETM_PRIO:
		ret = ceetm_init_prio(sch, priv, qopt);
		break;
	case CEETM_WBFS:
		ret = ceetm_init_wbfs(sch, priv, qopt);
		break;
	default:
		pr_err(KBUILD_BASENAME " : %s : invalid qdisc\n", __func__);
		/* Note: ceetm_destroy() will be called by our caller */
		ret = -EINVAL;
	}

	return ret;
}

/* Edit a root ceetm qdisc */
static int ceetm_change_root(struct Qdisc *sch, struct ceetm_qdisc *priv,
			     struct net_device *dev,
			     struct tc_ceetm_qopt *qopt)
{
	int err = 0;
	u64 bps;

	if (priv->shaped != (bool)qopt->shaped) {
		pr_err("CEETM: qdisc %X is %s\n", sch->handle,
		       priv->shaped ? "shaped" : "unshaped");
		return -EINVAL;
	}

	/* Nothing to modify for unshaped qdiscs */
	if (!priv->shaped)
		return 0;

	/* Configure the LNI shaper */
	if (priv->root.overhead != qopt->overhead) {
		err = qman_ceetm_lni_enable_shaper(priv->root.lni, 1,
						   qopt->overhead);
		if (err)
			goto change_err;
		priv->root.overhead = qopt->overhead;
	}

	if (priv->root.rate != qopt->rate) {
		bps = qopt->rate << 3; /* Bps -> bps */
		err = qman_ceetm_lni_set_commit_rate_bps(priv->root.lni, bps,
							 dev->mtu);
		if (err)
			goto change_err;
		priv->root.rate = qopt->rate;
	}

	if (priv->root.ceil != qopt->ceil) {
		bps = qopt->ceil << 3; /* Bps -> bps */
		err = qman_ceetm_lni_set_excess_rate_bps(priv->root.lni, bps,
							 dev->mtu);
		if (err)
			goto change_err;
		priv->root.ceil = qopt->ceil;
	}

	return 0;

change_err:
	pr_err(KBUILD_BASENAME " : %s : failed to configure the root ceetm qdisc %X\n",
	       __func__, sch->handle);
	return err;
}

/* Edit a wbfs ceetm qdisc */
static int ceetm_change_wbfs(struct Qdisc *sch, struct ceetm_qdisc *priv,
			     struct tc_ceetm_qopt *qopt)
{
	bool group_b;
	int err;

	if (qopt->qcount) {
		pr_err("CEETM: the qcount can not be modified\n");
		return -EINVAL;
	}

	if (qopt->qweight[0]) {
		pr_err("CEETM: the qweight can be modified through the wbfs classes\n");
		return -EINVAL;
	}

	if (!priv->shaped && (qopt->cr || qopt->er)) {
		pr_err("CEETM: CR/ER can be enabled only for shaped wbfs ceetm qdiscs\n");
		return -EINVAL;
	}

	if (priv->shaped && !(qopt->cr || qopt->er)) {
		pr_err("CEETM: either CR or ER must be enabled for shaped wbfs ceetm qdiscs\n");
		return -EINVAL;
	}

	/* Nothing to modify for unshaped qdiscs */
	if (!priv->shaped)
		return 0;

	group_b = priv->wbfs.group_type == WBFS_GRP_B;

	if (qopt->cr != priv->wbfs.cr) {
		err = qman_ceetm_channel_set_group_cr_eligibility(priv->wbfs.ch,
								  group_b,
								  qopt->cr);
		if (err)
			goto change_err;
		priv->wbfs.cr = qopt->cr;
	}

	if (qopt->er != priv->wbfs.er) {
		err = qman_ceetm_channel_set_group_er_eligibility(priv->wbfs.ch,
								  group_b,
								  qopt->er);
		if (err)
			goto change_err;
		priv->wbfs.er = qopt->er;
	}

	return 0;

change_err:
	pr_err(KBUILD_BASENAME " : %s : failed to configure the wbfs ceetm qdisc %X\n",
	       __func__, sch->handle);
	return err;
}

/* Edit a ceetm qdisc */
static int ceetm_change(struct Qdisc *sch, struct nlattr *opt,
			struct netlink_ext_ack *extack)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct net_device *dev = qdisc_dev(sch);
	struct nlattr *tb[TCA_CEETM_QOPS + 1];
	struct tc_ceetm_qopt *qopt;
	int ret;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	ret = nla_parse_nested_deprecated(tb, TCA_CEETM_QOPS, opt,
					  ceetm_policy, NULL);
	if (ret < 0) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return ret;
	}

	if (!tb[TCA_CEETM_QOPS]) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return -EINVAL;
	}

	if (TC_H_MIN(sch->handle)) {
		pr_err("CEETM: a qdisc should not have a minor\n");
		return -EINVAL;
	}

	qopt = nla_data(tb[TCA_CEETM_QOPS]);

	if (priv->type != qopt->type) {
		pr_err("CEETM: qdisc %X is not of the provided type\n",
		       sch->handle);
		return -EINVAL;
	}

	switch (priv->type) {
	case CEETM_ROOT:
		ret = ceetm_change_root(sch, priv, dev, qopt);
		break;
	case CEETM_PRIO:
		pr_err("CEETM: prio qdiscs can not be modified\n");
		ret = -EINVAL;
		break;
	case CEETM_WBFS:
		ret = ceetm_change_wbfs(sch, priv, qopt);
		break;
	default:
		pr_err(KBUILD_BASENAME " : %s : invalid qdisc\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

/* Graft the underlying pfifo qdiscs to the netdev queues.
 * It's safe to remove our references at this point, since the kernel will
 * destroy the qdiscs on its own and no cleanup from our part is required.
 */
static void ceetm_attach(struct Qdisc *sch)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct net_device *dev = qdisc_dev(sch);
	struct Qdisc *qdisc, *old_qdisc;
	unsigned int i;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	for (i = 0; i < dev->num_tx_queues; i++) {
		qdisc = priv->root.qdiscs[i];
		old_qdisc = dev_graft_qdisc(qdisc->dev_queue, qdisc);
		if (old_qdisc)
			qdisc_put(old_qdisc);
	}

	kfree(priv->root.qdiscs);
	priv->root.qdiscs = NULL;
}

static unsigned long ceetm_cls_search(struct Qdisc *sch, u32 handle)
{
	return (unsigned long)ceetm_find(handle, sch);
}

static int ceetm_cls_change_root(struct ceetm_class *cl,
				 struct tc_ceetm_copt *copt,
				 struct net_device *dev)
{
	int err;
	u64 bps;

	if ((bool)copt->shaped != cl->shaped) {
		pr_err("CEETM: class %X is %s\n", cl->common.classid,
		       cl->shaped ? "shaped" : "unshaped");
		return -EINVAL;
	}

	if (cl->shaped && cl->root.rate != copt->rate) {
		bps = copt->rate << 3; /* Bps -> bps */
		err = qman_ceetm_channel_set_commit_rate_bps(cl->ch, bps,
							     dev->mtu);
		if (err)
			goto change_cls_err;
		cl->root.rate = copt->rate;
	}

	if (cl->shaped && cl->root.ceil != copt->ceil) {
		bps = copt->ceil << 3; /* Bps -> bps */
		err = qman_ceetm_channel_set_excess_rate_bps(cl->ch, bps,
							     dev->mtu);
		if (err)
			goto change_cls_err;
		cl->root.ceil = copt->ceil;
	}

	if (!cl->shaped && cl->root.tbl != copt->tbl) {
		err = qman_ceetm_channel_set_weight(cl->ch, copt->tbl);
		if (err)
			goto change_cls_err;
		cl->root.tbl = copt->tbl;
	}

	return 0;

change_cls_err:
	pr_err(KBUILD_BASENAME " : %s : failed to configure the ceetm root class %X\n",
	       __func__, cl->common.classid);
	return err;
}

static int ceetm_cls_change_prio(struct ceetm_class *cl,
				 struct tc_ceetm_copt *copt)
{
	int err;

	if (!cl->shaped && (copt->cr || copt->er)) {
		pr_err("CEETM: only shaped classes can have CR and ER enabled\n");
		return -EINVAL;
	}

	if (cl->prio.cr != (bool)copt->cr) {
		err = qman_ceetm_channel_set_cq_cr_eligibility
						(cl->prio.cq->parent,
						cl->prio.cq->idx,
						copt->cr);
		if (err)
			goto change_cls_err;
		cl->prio.cr = copt->cr;
	}

	if (cl->prio.er != (bool)copt->er) {
		err = qman_ceetm_channel_set_cq_er_eligibility
						(cl->prio.cq->parent,
						cl->prio.cq->idx,
						copt->er);
		if (err)
			goto change_cls_err;
		cl->prio.er = copt->er;
	}

	return 0;

change_cls_err:
	pr_err(KBUILD_BASENAME " : %s : failed to configure the ceetm prio class %X\n",
	       __func__, cl->common.classid);
	return err;
}

static int ceetm_cls_change_wbfs(struct ceetm_class *cl,
				 struct tc_ceetm_copt *copt)
{
	int err;

	if (copt->weight != cl->wbfs.weight) {
		/* Configure the CQ weight: real number multiplied by 100 to
		 * get rid of the fraction
		 */
		err = qman_ceetm_set_queue_weight_in_ratio(cl->wbfs.cq,
							   copt->weight * 100);

		if (err) {
			pr_err(KBUILD_BASENAME " : %s : failed to configure the ceetm wbfs class %X\n",
			       __func__, cl->common.classid);
			return err;
		}

		cl->wbfs.weight = copt->weight;
	}

	return 0;
}

/* Add a ceetm root class or configure a ceetm root/prio/wbfs class */
static int ceetm_cls_change(struct Qdisc *sch, u32 classid, u32 parentid,
			    struct nlattr **tca, unsigned long *arg,
			    struct netlink_ext_ack *extack)
{
	struct ceetm_class *cl = (struct ceetm_class *)*arg;
	struct net_device *dev = qdisc_dev(sch);
	struct nlattr *opt = tca[TCA_OPTIONS];
	struct nlattr *tb[__TCA_CEETM_MAX];
	struct qm_ceetm_channel *channel;
	struct tc_ceetm_copt *copt;
	struct ceetm_qdisc *priv;
	int err;
	u64 bps;

	pr_debug(KBUILD_BASENAME " : %s : classid %X under qdisc %X\n",
		 __func__, classid, sch->handle);

	if (strcmp(sch->ops->id, ceetm_qdisc_ops.id)) {
		pr_err("CEETM: a ceetm class can not be attached to other qdisc/class types\n");
		return -EINVAL;
	}

	priv = qdisc_priv(sch);

	if (!opt) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return -EINVAL;
	}

	if (!cl && sch->handle != parentid) {
		pr_err("CEETM: classes can be attached to the root ceetm qdisc only\n");
		return -EINVAL;
	}

	if (!cl && priv->type != CEETM_ROOT) {
		pr_err("CEETM: root ceetm classes can be attached to the root ceetm qdisc only\n");
		return -EINVAL;
	}

	err = nla_parse_nested_deprecated(tb, TCA_CEETM_COPT, opt,
					  ceetm_policy, NULL);
	if (err < 0) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return -EINVAL;
	}

	if (!tb[TCA_CEETM_COPT]) {
		pr_err(KBUILD_BASENAME " : %s : tc error\n", __func__);
		return -EINVAL;
	}

	if (TC_H_MIN(classid) >= PFIFO_MIN_OFFSET) {
		pr_err("CEETM: only minors 0x01 to 0x20 can be used for ceetm root classes\n");
		return -EINVAL;
	}

	copt = nla_data(tb[TCA_CEETM_COPT]);

	/* Configure an existing ceetm class */
	if (cl) {
		if (copt->type != cl->type) {
			pr_err("CEETM: class %X is not of the provided type\n",
			       cl->common.classid);
			return -EINVAL;
		}

		switch (copt->type) {
		case CEETM_ROOT:
			return ceetm_cls_change_root(cl, copt, dev);

		case CEETM_PRIO:
			return ceetm_cls_change_prio(cl, copt);

		case CEETM_WBFS:
			return ceetm_cls_change_wbfs(cl, copt);

		default:
			pr_err(KBUILD_BASENAME " : %s : invalid class\n",
			       __func__);
			return -EINVAL;
		}
	}

	/* Add a new root ceetm class */
	if (copt->type != CEETM_ROOT) {
		pr_err("CEETM: only root ceetm classes can be attached to the root ceetm qdisc\n");
		return -EINVAL;
	}

	if (copt->shaped && !priv->shaped) {
		pr_err("CEETM: can not add a shaped ceetm root class under an unshaped ceetm root qdisc\n");
		return -EINVAL;
	}

	cl = kzalloc(sizeof(*cl), GFP_KERNEL);
	if (!cl)
		return -ENOMEM;

	err = tcf_block_get(&cl->block, &cl->filter_list, sch, extack);
	if (err) {
		kfree(cl);
		return err;
	}

	cl->type = copt->type;
	cl->shaped = copt->shaped;
	cl->root.rate = copt->rate;
	cl->root.ceil = copt->ceil;
	cl->root.tbl = copt->tbl;

	cl->common.classid = classid;
	cl->parent = sch;
	cl->root.child = NULL;
	cl->root.wbfs_grp_a = false;
	cl->root.wbfs_grp_b = false;
	cl->root.wbfs_grp_large = false;

	/* Claim a CEETM channel */
	err = qman_ceetm_channel_claim(&channel, priv->root.lni);
	if (err) {
		pr_err(KBUILD_BASENAME " : %s : failed to claim a channel\n",
		       __func__);
		goto claim_err;
	}

	cl->ch = channel;

	if (cl->shaped) {
		/* Configure the channel shaper */
		err = qman_ceetm_channel_enable_shaper(channel, 1);
		if (err)
			goto channel_err;

		bps = cl->root.rate << 3; /* Bps -> bps */
		err = qman_ceetm_channel_set_commit_rate_bps(channel, bps,
							     dev->mtu);
		if (err)
			goto channel_err;

		bps = cl->root.ceil << 3; /* Bps -> bps */
		err = qman_ceetm_channel_set_excess_rate_bps(channel, bps,
							     dev->mtu);
		if (err)
			goto channel_err;

	} else {
		/* Configure the uFQ algorithm */
		err = qman_ceetm_channel_set_weight(channel, cl->root.tbl);
		if (err)
			goto channel_err;
	}

	/* Add class handle in Qdisc */
	ceetm_link_class(sch, &priv->clhash, &cl->common);

	pr_debug(KBUILD_BASENAME " : %s : configured class %X associated with channel %d\n",
		 __func__, classid, channel->idx);
	*arg = (unsigned long)cl;
	return 0;

channel_err:
	pr_err(KBUILD_BASENAME " : %s : failed to configure the channel %d\n",
	       __func__, channel->idx);
	if (qman_ceetm_channel_release(channel))
		pr_err(KBUILD_BASENAME " : %s : failed to release the channel %d\n",
		       __func__, channel->idx);
claim_err:
	tcf_block_put(cl->block);
	kfree(cl);
	return err;
}

static void ceetm_cls_walk(struct Qdisc *sch, struct qdisc_walker *arg)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct ceetm_class *cl;
	unsigned int i;

	pr_debug(KBUILD_BASENAME " : %s : qdisc %X\n", __func__, sch->handle);

	if (arg->stop)
		return;

	for (i = 0; i < priv->clhash.hashsize; i++) {
		hlist_for_each_entry(cl, &priv->clhash.hash[i], common.hnode) {
			if (arg->count < arg->skip) {
				arg->count++;
				continue;
			}
			if (arg->fn(sch, (unsigned long)cl, arg) < 0) {
				arg->stop = 1;
				return;
			}
			arg->count++;
		}
	}
}

static int ceetm_cls_dump(struct Qdisc *sch, unsigned long arg,
			  struct sk_buff *skb, struct tcmsg *tcm)
{
	struct ceetm_class *cl = (struct ceetm_class *)arg;
	struct tc_ceetm_copt copt;
	struct nlattr *nest;

	pr_debug(KBUILD_BASENAME " : %s : class %X under qdisc %X\n",
		 __func__, cl->common.classid, sch->handle);

	sch_tree_lock(sch);

	tcm->tcm_parent = ((struct Qdisc *)cl->parent)->handle;
	tcm->tcm_handle = cl->common.classid;

	memset(&copt, 0, sizeof(copt));

	copt.shaped = cl->shaped;
	copt.type = cl->type;

	switch (cl->type) {
	case CEETM_ROOT:
		if (cl->root.child)
			tcm->tcm_info = cl->root.child->handle;

		copt.rate = cl->root.rate;
		copt.ceil = cl->root.ceil;
		copt.tbl = cl->root.tbl;
		break;

	case CEETM_PRIO:
		if (cl->prio.child)
			tcm->tcm_info = cl->prio.child->handle;

		copt.cr = cl->prio.cr;
		copt.er = cl->prio.er;
		break;

	case CEETM_WBFS:
		copt.weight = cl->wbfs.weight;
		break;
	}

	nest = nla_nest_start_noflag(skb, TCA_OPTIONS);
	if (!nest)
		goto nla_put_failure;
	if (nla_put(skb, TCA_CEETM_COPT, sizeof(copt), &copt))
		goto nla_put_failure;
	nla_nest_end(skb, nest);
	sch_tree_unlock(sch);
	return skb->len;

nla_put_failure:
	sch_tree_unlock(sch);
	nla_nest_cancel(skb, nest);
	return -EMSGSIZE;
}

static int ceetm_cls_delete(struct Qdisc *sch, unsigned long arg,
			    struct netlink_ext_ack *extack)
{
	struct ceetm_class *cl = (struct ceetm_class *)arg;
	struct ceetm_qdisc *priv = qdisc_priv(sch);

	pr_debug(KBUILD_BASENAME " : %s : class %X under qdisc %X\n",
		 __func__, cl->common.classid, sch->handle);

	sch_tree_lock(sch);
	qdisc_class_hash_remove(&priv->clhash, &cl->common);

	sch_tree_unlock(sch);
	ceetm_cls_destroy(sch, cl);
	return 0;
}

/* Get the class' child qdisc, if any */
static struct Qdisc *ceetm_cls_leaf(struct Qdisc *sch, unsigned long arg)
{
	struct ceetm_class *cl = (struct ceetm_class *)arg;

	pr_debug(KBUILD_BASENAME " : %s : class %X under qdisc %X\n",
		 __func__, cl->common.classid, sch->handle);

	switch (cl->type) {
	case CEETM_ROOT:
		return cl->root.child;

	case CEETM_PRIO:
		return cl->prio.child;
	}

	return NULL;
}

static int ceetm_cls_graft(struct Qdisc *sch, unsigned long arg,
			   struct Qdisc *new, struct Qdisc **old,
			   struct netlink_ext_ack *extack)
{
	if (new && strcmp(new->ops->id, ceetm_qdisc_ops.id)) {
		pr_err("CEETM: only ceetm qdiscs can be attached to ceetm classes\n");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ceetm_cls_dump_stats(struct Qdisc *sch, unsigned long arg,
				struct gnet_dump *d)
{
	struct ceetm_class *cl = (struct ceetm_class *)arg;
	struct gnet_stats_basic_sync tmp_bstats;
	struct ceetm_class_stats *cstats = NULL;
	struct qm_ceetm_cq *cq = NULL;
	struct tc_ceetm_xstats xstats;
	unsigned int i;

	memset(&xstats, 0, sizeof(xstats));
	gnet_stats_basic_sync_init(&tmp_bstats);

	switch (cl->type) {
	case CEETM_ROOT:
		return 0;
	case CEETM_PRIO:
		cq = cl->prio.cq;
		break;
	case CEETM_WBFS:
		cq = cl->wbfs.cq;
		break;
	}

	for_each_online_cpu(i) {
		switch (cl->type) {
		case CEETM_PRIO:
			cstats = per_cpu_ptr(cl->prio.cstats, i);
			break;
		case CEETM_WBFS:
			cstats = per_cpu_ptr(cl->wbfs.cstats, i);
			break;
		}

		if (cstats) {
			xstats.ern_drop_count += cstats->ern_drop_count;
			xstats.congested_count += cstats->congested_count;
			u64_stats_add(&tmp_bstats.bytes,
				      u64_stats_read(&cstats->bstats.bytes));
			u64_stats_add(&tmp_bstats.packets,
				      u64_stats_read(&cstats->bstats.packets));
		}
	}

	if (gnet_stats_copy_basic(d, NULL, &tmp_bstats, true))
		return -1;

	if (cq && qman_ceetm_cq_get_dequeue_statistics(cq, 0,
						       &xstats.frame_count,
						       &xstats.byte_count))
		return -1;

	return gnet_stats_copy_app(d, &xstats, sizeof(xstats));
}

static struct tcf_block *ceetm_tcf_block(struct Qdisc *sch, unsigned long arg,
					 struct netlink_ext_ack *extack)
{
	struct ceetm_class *cl = (struct ceetm_class *)arg;
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct tcf_block *block;

	block = cl ? cl->block : priv->block;

	pr_debug(KBUILD_BASENAME " : %s : class %X under qdisc %X\n", __func__,
		 cl ? cl->common.classid : 0, sch->handle);
	return block;
}

static unsigned long ceetm_tcf_bind(struct Qdisc *sch, unsigned long parent,
				    u32 classid)
{
	struct ceetm_class *cl = ceetm_find(classid, sch);

	pr_debug(KBUILD_BASENAME " : %s : class %X under qdisc %X\n", __func__,
		 cl ? cl->common.classid : 0, sch->handle);
	return (unsigned long)cl;
}

static void ceetm_tcf_unbind(struct Qdisc *sch, unsigned long arg)
{
	struct ceetm_class *cl = (struct ceetm_class *)arg;

	pr_debug(KBUILD_BASENAME " : %s : class %X under qdisc %X\n", __func__,
		 cl ? cl->common.classid : 0, sch->handle);
}

const struct Qdisc_class_ops ceetm_cls_ops = {
	.graft		=	ceetm_cls_graft,
	.leaf		=	ceetm_cls_leaf,
	.find		=	ceetm_cls_search,
	.change		=	ceetm_cls_change,
	.delete		=	ceetm_cls_delete,
	.walk		=	ceetm_cls_walk,
	.tcf_block	=	ceetm_tcf_block,
	.bind_tcf	=	ceetm_tcf_bind,
	.unbind_tcf	=	ceetm_tcf_unbind,
	.dump		=	ceetm_cls_dump,
	.dump_stats	=	ceetm_cls_dump_stats,
};

struct Qdisc_ops ceetm_qdisc_ops __read_mostly = {
	.id		=	"ceetm",
	.priv_size	=	sizeof(struct ceetm_qdisc),
	.cl_ops		=	&ceetm_cls_ops,
	.init		=	ceetm_init,
	.destroy	=	ceetm_destroy,
	.change		=	ceetm_change,
	.dump		=	ceetm_dump,
	.attach		=	ceetm_attach,
	.owner		=	THIS_MODULE,
};

/* Run the filters and classifiers attached to the qdisc on the provided skb */
static struct ceetm_class *ceetm_classify(struct sk_buff *skb,
					  struct Qdisc *sch, int *qerr,
					  bool *act_drop)
{
	struct ceetm_qdisc *priv = qdisc_priv(sch);
	struct ceetm_class *cl = NULL, *wbfs_cl;
	struct tcf_result res;
	struct tcf_proto *tcf;
	int result;

	*qerr = NET_XMIT_SUCCESS | __NET_XMIT_BYPASS;
	tcf = priv->filter_list;
	while (tcf && (result = tcf_classify(skb, priv->block, tcf, &res, false)) >= 0) {
#ifdef CONFIG_NET_CLS_ACT
		switch (result) {
		case TC_ACT_QUEUED:
		case TC_ACT_STOLEN:
		case TC_ACT_TRAP:
			*qerr = NET_XMIT_SUCCESS | __NET_XMIT_STOLEN;
			fallthrough;
		case TC_ACT_SHOT:
			/* No valid class found due to action */
			*act_drop = true;
			return NULL;
		}
#endif
		cl = (void *)res.class;
		if (!cl) {
			if (res.classid == sch->handle) {
				/* The filter leads to the qdisc */
				/* TODO default qdisc */
				return NULL;
			}

			cl = ceetm_find(res.classid, sch);
			if (!cl)
				/* The filter leads to an invalid class */
				break;
		}

		/* The class might have its own filters attached */
		tcf = cl->filter_list;
	}

	if (!cl) {
		/* No valid class found */
		/* TODO default qdisc */
		return NULL;
	}

	switch (cl->type) {
	case CEETM_ROOT:
		if (cl->root.child) {
			/* Run the prio qdisc classifiers */
			return ceetm_classify(skb, cl->root.child, qerr,
						act_drop);
		} else {
			/* The root class does not have a child prio qdisc */
			/* TODO default qdisc */
			return NULL;
		}
	case CEETM_PRIO:
		if (cl->prio.child) {
			/* If filters lead to a wbfs class, return it.
			 * Otherwise, return the prio class
			 */
			wbfs_cl = ceetm_classify(skb, cl->prio.child, qerr,
						 act_drop);
			/* A NULL result might indicate either an erroneous
			 * filter, or no filters at all. We will assume the
			 * latter
			 */
			return wbfs_cl ? : cl;
		}
	}

	/* For wbfs and childless prio classes, return the class directly */
	return cl;
}

int __hot ceetm_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	int queue_mapping = dpa_get_queue_mapping(skb);
	struct Qdisc *sch = net_dev->qdisc;
	struct ceetm_class_stats *cstats;
	struct ceetm_qdisc_stats *qstats;
	struct dpa_priv_s *priv_dpa;
	struct ceetm_fq *ceetm_fq;
	struct ceetm_qdisc *priv;
	struct qman_fq *conf_fq;
	struct ceetm_class *cl;
	spinlock_t *root_lock;
	bool act_drop = false;
	int ret;

	root_lock = qdisc_lock(sch);
	priv = qdisc_priv(sch);
	qstats = this_cpu_ptr(priv->root.qstats);

	spin_lock(root_lock);
	cl = ceetm_classify(skb, sch, &ret, &act_drop);
	spin_unlock(root_lock);

#ifdef CONFIG_NET_CLS_ACT
	if (act_drop) {
		if (ret & __NET_XMIT_BYPASS)
			qstats->drops++;
		goto drop;
	}
#endif
	/* TODO default class */
	if (unlikely(!cl)) {
		qstats->drops++;
		goto drop;
	}

	if (unlikely(queue_mapping >= DPAA_ETH_TX_QUEUES))
		queue_mapping = queue_mapping % DPAA_ETH_TX_QUEUES;

	priv_dpa = netdev_priv(net_dev);
	conf_fq = priv_dpa->conf_fqs[queue_mapping];

	/* Choose the proper tx fq and update the basic stats (bytes and
	 * packets sent by the class)
	 */
	switch (cl->type) {
	case CEETM_PRIO:
		ceetm_fq = cl->prio.fq;
		cstats = this_cpu_ptr(cl->prio.cstats);
		break;
	case CEETM_WBFS:
		ceetm_fq = cl->wbfs.fq;
		cstats = this_cpu_ptr(cl->wbfs.cstats);
		break;
	default:
		qstats->drops++;
		goto drop;
	}

	/* If the FQ is congested, avoid enqueuing the frame and dropping it
	 * when it returns on the ERN path. Drop it here directly instead.
	 */
	if (unlikely(ceetm_fq->congested)) {
		qstats->drops++;
		goto drop;
	}

	bstats_update(&cstats->bstats, skb);
	return dpa_tx_extended(skb, net_dev, &ceetm_fq->fq, conf_fq);

drop:
	dev_kfree_skb_any(skb);
	return NET_XMIT_SUCCESS;
}
