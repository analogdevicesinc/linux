/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Copyright 2019 NXP
 */
#ifndef _PXP_DEVICE
#define _PXP_DEVICE

#include <linux/idr.h>
#include <linux/hash.h>
#include <uapi/linux/pxp_device.h>

struct pxp_irq_info {
	wait_queue_head_t waitq;
	atomic_t irq_pending;
	int hist_status;
};

struct pxp_buffer_hash {
	struct hlist_head *hash_table;
	u32 order;
	spinlock_t hash_lock;
};

struct pxp_buf_obj {
	uint32_t handle;

	uint32_t size;
	uint32_t mem_type;

	unsigned long offset;
	void *virtual;

	struct hlist_node item;
};

struct pxp_chan_obj {
	uint32_t handle;
	struct dma_chan *chan;
};

/* File private data */
struct pxp_file {
	struct file *filp;

	/* record allocated dma buffer */
	struct idr buffer_idr;
	spinlock_t buffer_lock;

	/* record allocated dma channel */
	struct idr channel_idr;
	spinlock_t channel_lock;
};

#endif
