/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef _MPNIC_TXRX_H_
#define _MPNIC_TXRX_H_

#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <net/netdev_queues.h>

#include "mpnic.h"

struct mpnic_net;

/* Space we have to have available in a work queue to take a packet:
 *	1 descriptor per page
 *	+ 1 descriptor for the skb head
 *	+ 1 descriptor for the metadata
 *	+ 7 descriptors to keep the tail out of the head's cacheline
 * If we cannot guarantee that we return NETDEV_TX_BUSY.
 */
#define MPNIC_MAX_SKB_DESC		(MAX_SKB_FRAGS + 9)
#define MPNIC_TX_DESC_WAKEUP		(MPNIC_MAX_SKB_DESC * 2)

#define MPNIC_MAX_NAPI_VECTORS		1024u

struct mpnic_ring {
	void **tx_buf;			/* Packets outstanding in a TWQ */

	u32 __iomem *doorbell;		/* Pointer to CSR space for ring */
	__le64 *desc;			/* Descriptor ring memory */
	u16 size_mask;			/* Size of ring in descriptors - 1 */
	u16 q_idx;			/* Hardware queue index */

	u32 head, tail;			/* Head/Tail of ring */

	/* TWQ only, index of the metadata descriptor of the last packet
	 * placed in the ring without ringing the doorbell, -1 if the
	 * doorbell is in sync with the tail.
	 */
	s32 deferred_meta;

	/* Slow path fields follow */
	dma_addr_t dma;			/* Phys addr of descriptor memory */
	size_t size;			/* Size of descriptor ring in memory */
};

/* The device pairs two work queues with one completion queue. On the Tx
 * side only the first work queue is used for now, the second one becomes
 * the XDP ring.
 */
struct mpnic_q_triad {
	struct mpnic_ring sub0, sub1, cmpl;
};

struct mpnic_napi_vector {
	struct napi_struct napi;
	struct device *dev;		/* Device for DMA unmapping */
	struct mpnic_dev *mpd;

	u16 v_idx;
	u16 txt_count;

	char name[IFNAMSIZ + 11];

	struct mpnic_q_triad qt[];
};

int mpnic_alloc_napi_vectors(struct mpnic_net *mpn);
void mpnic_free_napi_vectors(struct mpnic_net *mpn);
int mpnic_alloc_resources(struct mpnic_net *mpn);
void mpnic_free_resources(struct mpnic_net *mpn);
int mpnic_set_netif_queues(struct mpnic_net *mpn);
void mpnic_reset_netif_queues(struct mpnic_net *mpn);
void mpnic_napi_enable(struct mpnic_net *mpn);
void mpnic_napi_disable(struct mpnic_net *mpn);
void mpnic_enable(struct mpnic_net *mpn);
void mpnic_disable(struct mpnic_net *mpn);
void mpnic_wait_all_queues_idle(struct mpnic_dev *mpd);
void mpnic_flush(struct mpnic_net *mpn);

#endif /* _MPNIC_TXRX_H_ */
