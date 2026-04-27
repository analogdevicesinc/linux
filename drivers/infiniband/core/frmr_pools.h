/* SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB
 *
 * Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#ifndef RDMA_CORE_FRMR_POOLS_H
#define RDMA_CORE_FRMR_POOLS_H

#include <rdma/frmr_pools.h>
#include <linux/rbtree_types.h>
#include <linux/spinlock_types.h>
#include <linux/types.h>
#include <asm/page.h>
#include <linux/workqueue.h>

#define NUM_HANDLES_PER_PAGE \
	((PAGE_SIZE - sizeof(struct list_head)) / sizeof(u32))

struct frmr_handles_page {
	struct list_head list;
	u32 handles[NUM_HANDLES_PER_PAGE];
};

/* FRMR queue holds a list of frmr_handles_page.
 * num_pages: number of pages in the queue.
 * ci: current index in the handles array across all pages.
 */
struct frmr_queue {
	struct list_head pages_list;
	u32 num_pages;
	unsigned long ci;
};

struct ib_frmr_pool {
	struct rb_node node;
	struct ib_frmr_key key; /* Pool key */

	/* Protect access to the queue */
	spinlock_t lock;
	struct frmr_queue queue;
	struct frmr_queue inactive_queue;

	struct delayed_work aging_work;
	struct ib_device *device;

	u32 max_in_use;
	u32 in_use;
	u32 pinned_handles;
};

struct ib_frmr_pools {
	struct rb_root rb_root;
	rwlock_t rb_lock;
	const struct ib_frmr_pool_ops *pool_ops;

	struct workqueue_struct *aging_wq;
	u32 aging_period_sec;
};

int ib_frmr_pools_set_pinned(struct ib_device *device, struct ib_frmr_key *key,
			     u32 pinned_handles);
int ib_frmr_pools_set_aging_period(struct ib_device *device, u32 period_sec);
#endif /* RDMA_CORE_FRMR_POOLS_H */
