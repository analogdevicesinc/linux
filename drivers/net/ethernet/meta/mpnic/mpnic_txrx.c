// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "mpnic.h"
#include "mpnic_netdev.h"
#include "mpnic_txrx.h"

struct mpnic_xmit_cb {
	u32 bytecount;
	u8 desc_count;
};

#define MPNIC_XMIT_CB(__skb) ((struct mpnic_xmit_cb *)((__skb)->cb))

/* Leave the interrupt moderation counters alone when arming or masking */
#define MPNIC_TIM_PARAM_CFG_PRESERVE_MASK \
	(MPNIC_TIM_CTL1_UPD_IGN_LONG_EVENT_CNT | \
	 MPNIC_TIM_CTL1_UPD_IGN_LONG_TIME_CNT | \
	 MPNIC_TIM_CTL1_UPD_IGN_SHORT_TIME_CNT)

static void mpnic_nv_irq_disable(struct mpnic_napi_vector *nv)
{
	mpnic_wr64(nv->mpd, MPNIC_TIM_CTL1(nv->qt[0].cmpl.q_idx),
		   MPNIC_TIM_PARAM_CFG_PRESERVE_MASK |
		   MPNIC_TIM_CTL1_MASK_EN | MPNIC_TIM_CTL1_MASK);
}

static void mpnic_nv_irq_rearm(struct mpnic_napi_vector *nv)
{
	/* Rearming a single queue on a given IRQ rearms all the other
	 * queues mapped to the same IRQ.
	 */
	mpnic_wr64(nv->mpd, MPNIC_TIM_CTL1(nv->qt[0].cmpl.q_idx),
		   MPNIC_TIM_PARAM_CFG_PRESERVE_MASK | MPNIC_TIM_CTL1_MASK_EN);
}

static void mpnic_nv_irq_trigger(struct mpnic_napi_vector *nv)
{
	mpnic_wr64(nv->mpd, MPNIC_TIM_CTL1(nv->qt[0].cmpl.q_idx),
		   MPNIC_TIM_PARAM_CFG_PRESERVE_MASK |
		   MPNIC_TIM_CTL1_MASK_EN | MPNIC_TIM_CTL1_TRIGGER);
}

static unsigned int mpnic_desc_unused(struct mpnic_ring *ring)
{
	return (ring->head - ring->tail - 1) & ring->size_mask;
}

static struct netdev_queue *mpnic_txring_txq(const struct net_device *dev,
					     const struct mpnic_ring *ring)
{
	return netdev_get_tx_queue(dev, ring->q_idx);
}

static void mpnic_unmap_single_twd(struct device *dev, __le64 *twd)
{
	u64 raw_twd = le64_to_cpu(*twd);

	dma_unmap_single(dev, FIELD_GET(MPNIC_TWD_ADDR, raw_twd),
			 FIELD_GET(MPNIC_TWD_LEN, raw_twd), DMA_TO_DEVICE);
}

static void mpnic_unmap_page_twd(struct device *dev, __le64 *twd)
{
	u64 raw_twd = le64_to_cpu(*twd);

	dma_unmap_page(dev, FIELD_GET(MPNIC_TWD_ADDR, raw_twd),
		       FIELD_GET(MPNIC_TWD_LEN, raw_twd), DMA_TO_DEVICE);
}

static void mpnic_clean_twq0(struct mpnic_napi_vector *nv, int napi_budget,
			     struct mpnic_ring *ring, bool discard,
			     unsigned int hw_head)
{
	u64 total_bytes = 0, total_packets = 0;
	unsigned int head = ring->head;
	struct netdev_queue *txq;
	unsigned int clean_desc;

	clean_desc = (hw_head - head) & ring->size_mask;

	while (clean_desc) {
		struct sk_buff *skb = ring->tx_buf[head];
		unsigned int desc_cnt;

		desc_cnt = MPNIC_XMIT_CB(skb)->desc_count;
		if (desc_cnt > clean_desc)
			break;

		ring->tx_buf[head] = NULL;

		clean_desc -= desc_cnt;

		/* Step over the metadata descriptor */
		head++;
		head &= ring->size_mask;
		desc_cnt--;

		mpnic_unmap_single_twd(nv->dev, &ring->desc[head]);
		head++;
		head &= ring->size_mask;
		desc_cnt--;

		while (desc_cnt--) {
			mpnic_unmap_page_twd(nv->dev, &ring->desc[head]);
			head++;
			head &= ring->size_mask;
		}

		total_bytes += MPNIC_XMIT_CB(skb)->bytecount;
		total_packets++;

		napi_consume_skb(skb, napi_budget);
	}

	if (!total_bytes)
		return;

	ring->head = head;

	if (discard)
		return;

	txq = mpnic_txring_txq(nv->napi.dev, ring);
	netif_txq_completed_wake(txq, total_packets, total_bytes,
				 mpnic_desc_unused(ring),
				 MPNIC_TX_DESC_WAKEUP);
}

static void mpnic_commit_cq_head(struct mpnic_ring *cmpl)
{
	u32 head = cmpl->head;

	/* The tail shadows the last value written to the doorbell, so a
	 * completion queue which has not moved costs no MMIO write.
	 */
	if (cmpl->tail != head) {
		cmpl->tail = head;
		writeq(head & cmpl->size_mask, cmpl->doorbell);
	}
}

static void mpnic_clean_tcq(struct mpnic_napi_vector *nv,
			    struct mpnic_q_triad *qt, int napi_budget)
{
	struct mpnic_ring *cmpl = &qt->cmpl;
	__le64 *raw_tcd, done;
	u32 head = cmpl->head;
	s32 head0 = -1;

	done = (head & (cmpl->size_mask + 1)) ? 0 : cpu_to_le64(MPNIC_TCD_DONE);
	raw_tcd = &cmpl->desc[head & cmpl->size_mask];

	/* Walk the completion queue collecting the heads reported by NIC.
	 * Only the first work queue is enabled and no packet asks for a
	 * timestamp, so every completion is a plain head update and the
	 * descriptor type does not have to be decoded.
	 */
	while ((*raw_tcd & cpu_to_le64(MPNIC_TCD_DONE)) == done) {
		u64 tcd;

		dma_rmb();

		tcd = le64_to_cpu(*raw_tcd);
		head0 = FIELD_GET(MPNIC_TCD_TYPE0_HEAD0, tcd);

		raw_tcd++;
		head++;

		if (unlikely(!(head & cmpl->size_mask))) {
			done ^= cpu_to_le64(MPNIC_TCD_DONE);
			raw_tcd = &cmpl->desc[0];
		}
	}

	cmpl->head = head;

	if (head0 >= 0)
		mpnic_clean_twq0(nv, napi_budget, &qt->sub0, false, head0);
}

static int mpnic_poll(struct napi_struct *napi, int budget)
{
	struct mpnic_napi_vector *nv = container_of(napi,
						    struct mpnic_napi_vector,
						    napi);
	int i;

	for (i = 0; i < nv->txt_count; i++)
		mpnic_clean_tcq(nv, &nv->qt[i], budget);

	for (i = 0; i < nv->txt_count; i++)
		mpnic_commit_cq_head(&nv->qt[i].cmpl);

	if (likely(napi_complete_done(napi, 0)))
		mpnic_nv_irq_rearm(nv);

	return 0;
}

static irqreturn_t mpnic_msix_clean_rings(int __always_unused irq, void *data)
{
	struct mpnic_napi_vector *nv = data;

	napi_schedule_irqoff(&nv->napi);

	return IRQ_HANDLED;
}

static void mpnic_free_napi_vector(struct mpnic_net *mpn,
				   struct mpnic_napi_vector *nv)
{
	int i;

	for (i = 0; i < nv->txt_count; i++)
		mpn->tx[nv->qt[i].sub0.q_idx] = NULL;

	mpnic_free_irq(nv->mpd, nv->v_idx, nv);
	netif_napi_del_locked(&nv->napi);
	mpn->napi[nv->v_idx - MPNIC_NON_NAPI_VECTORS] = NULL;
	kfree(nv);
}

void mpnic_free_napi_vectors(struct mpnic_net *mpn)
{
	int i;

	for (i = 0; i < mpn->num_napi; i++)
		if (mpn->napi[i])
			mpnic_free_napi_vector(mpn, mpn->napi[i]);
}

static void mpnic_ring_init(struct mpnic_ring *ring, u32 __iomem *doorbell,
			    int q_idx)
{
	ring->doorbell = doorbell;
	ring->q_idx = q_idx;
}

static int mpnic_alloc_napi_vector(struct mpnic_dev *mpd,
				   struct mpnic_net *mpn, unsigned int idx)
{
	u32 __iomem *uc_addr = READ_ONCE(mpd->uc_addr0);
	struct mpnic_napi_vector *nv;
	int err;

	/* Doorbells are plain pointers into the register window, they have
	 * no way of noticing that it went away.
	 */
	if (!uc_addr)
		return -EIO;

	nv = kzalloc_flex(*nv, qt, 1);
	if (!nv)
		return -ENOMEM;

	nv->txt_count = 1;
	nv->mpd = mpd;
	nv->dev = mpd->dev;
	nv->v_idx = idx + MPNIC_NON_NAPI_VECTORS;

	mpn->napi[idx] = nv;
	netif_napi_add_config_locked(mpn->netdev, &nv->napi, mpnic_poll, idx);
	netif_napi_set_irq_locked(&nv->napi,
				  pci_irq_vector(to_pci_dev(mpd->dev),
						 nv->v_idx));

	snprintf(nv->name, sizeof(nv->name), "%s-TxRx-%u",
		 mpn->netdev->name, idx);

	err = mpnic_request_irq(mpd, nv->v_idx, mpnic_msix_clean_rings, 0,
				nv->name, nv);
	if (err)
		goto err_napi_del;

	mpnic_ring_init(&nv->qt[0].sub0, &uc_addr[MPNIC_TWQ_TAIL(idx, 0)], idx);
	mpnic_ring_init(&nv->qt[0].cmpl, &uc_addr[MPNIC_TCQ_HEAD(idx)], idx);
	mpn->tx[idx] = &nv->qt[0].sub0;

	return 0;

err_napi_del:
	netif_napi_del_locked(&nv->napi);
	mpn->napi[idx] = NULL;
	kfree(nv);
	return err;
}

int mpnic_alloc_napi_vectors(struct mpnic_net *mpn)
{
	unsigned int i;
	int err;

	for (i = 0; i < mpn->num_napi; i++) {
		err = mpnic_alloc_napi_vector(mpn->mpd, mpn, i);
		if (err)
			goto err_free_vectors;
	}

	return 0;

err_free_vectors:
	mpnic_free_napi_vectors(mpn);

	return err;
}

static void mpnic_free_ring_resources(struct device *dev,
				      struct mpnic_ring *ring)
{
	kvfree(ring->tx_buf);
	ring->tx_buf = NULL;

	/* If size is not set there are no descriptors present */
	if (!ring->size)
		return;

	dma_free_coherent(dev, ring->size, ring->desc, ring->dma);
	ring->size_mask = 0;
	ring->size = 0;
}

static int mpnic_alloc_ring_desc(struct mpnic_net *mpn,
				 struct mpnic_ring *ring, u32 count)
{
	struct device *dev = mpn->netdev->dev.parent;
	size_t size;

	size = ALIGN(array_size(sizeof(*ring->desc), count), 4096);

	ring->desc = dma_alloc_coherent(dev, size, &ring->dma,
					GFP_KERNEL | __GFP_NOWARN);
	if (!ring->desc)
		return -ENOMEM;

	ring->size_mask = count - 1;
	ring->size = size;

	return 0;
}

static void mpnic_free_tx_qt_resources(struct mpnic_net *mpn,
				       struct mpnic_q_triad *qt)
{
	struct device *dev = mpn->netdev->dev.parent;

	mpnic_free_ring_resources(dev, &qt->cmpl);
	mpnic_free_ring_resources(dev, &qt->sub0);
}

static int mpnic_alloc_tx_qt_resources(struct mpnic_net *mpn,
				       struct mpnic_q_triad *qt)
{
	int err;

	err = mpnic_alloc_ring_desc(mpn, &qt->sub0, mpn->txq_size);
	if (err)
		return err;

	qt->sub0.tx_buf = kvzalloc_objs(*qt->sub0.tx_buf, mpn->txq_size,
					GFP_KERNEL | __GFP_NOWARN);
	if (!qt->sub0.tx_buf) {
		err = -ENOMEM;
		goto err_free_qt;
	}

	err = mpnic_alloc_ring_desc(mpn, &qt->cmpl, mpn->txq_size);
	if (err)
		goto err_free_qt;

	return 0;

err_free_qt:
	mpnic_free_tx_qt_resources(mpn, qt);
	return err;
}

static void mpnic_free_nv_resources(struct mpnic_net *mpn,
				    struct mpnic_napi_vector *nv)
{
	int i;

	for (i = 0; i < nv->txt_count; i++)
		mpnic_free_tx_qt_resources(mpn, &nv->qt[i]);
}

static int mpnic_alloc_nv_resources(struct mpnic_net *mpn,
				    struct mpnic_napi_vector *nv)
{
	int i, err;

	for (i = 0; i < nv->txt_count; i++) {
		err = mpnic_alloc_tx_qt_resources(mpn, &nv->qt[i]);
		if (err)
			goto err_free_qt_resources;
	}

	return 0;

err_free_qt_resources:
	while (i--)
		mpnic_free_tx_qt_resources(mpn, &nv->qt[i]);
	return err;
}

void mpnic_free_resources(struct mpnic_net *mpn)
{
	int i;

	for (i = 0; i < mpn->num_napi; i++)
		mpnic_free_nv_resources(mpn, mpn->napi[i]);
}

int mpnic_alloc_resources(struct mpnic_net *mpn)
{
	int i, err;

	for (i = 0; i < mpn->num_napi; i++) {
		err = mpnic_alloc_nv_resources(mpn, mpn->napi[i]);
		if (err)
			goto err_free_resources;
	}

	return 0;

err_free_resources:
	while (i--)
		mpnic_free_nv_resources(mpn, mpn->napi[i]);

	return err;
}

int mpnic_set_netif_queues(struct mpnic_net *mpn)
{
	int i, j, err;

	err = netif_set_real_num_tx_queues(mpn->netdev, mpn->num_tx_queues);
	if (err)
		return err;

	for (i = 0; i < mpn->num_napi; i++) {
		struct mpnic_napi_vector *nv = mpn->napi[i];

		for (j = 0; j < nv->txt_count; j++)
			netif_queue_set_napi(mpn->netdev, nv->qt[j].sub0.q_idx,
					     NETDEV_QUEUE_TYPE_TX, &nv->napi);
	}

	return 0;
}

void mpnic_reset_netif_queues(struct mpnic_net *mpn)
{
	int i, j;

	for (i = 0; i < mpn->num_napi; i++) {
		struct mpnic_napi_vector *nv = mpn->napi[i];

		for (j = 0; j < nv->txt_count; j++)
			netif_queue_set_napi(mpn->netdev, nv->qt[j].sub0.q_idx,
					     NETDEV_QUEUE_TYPE_TX, NULL);
	}
}

void mpnic_napi_disable(struct mpnic_net *mpn)
{
	int i;

	for (i = 0; i < mpn->num_napi; i++) {
		napi_disable_locked(&mpn->napi[i]->napi);

		mpnic_nv_irq_disable(mpn->napi[i]);
	}
}

void mpnic_napi_enable(struct mpnic_net *mpn)
{
	int i;

	for (i = 0; i < mpn->num_napi; i++)
		napi_enable_locked(&mpn->napi[i]->napi);

	/* Force the first interrupt on each vector to guarantee that any
	 * completions posted during bringup are processed.
	 */
	for (i = 0; i < mpn->num_napi; i++)
		mpnic_nv_irq_trigger(mpn->napi[i]);

	mpnic_wrfl(mpn->mpd);
}
