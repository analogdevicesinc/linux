/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018,2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef __DPU_BLIT_H__
#define __DPU_BLIT_H__

#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/sync_file.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-array.h>

#define COMMAND_BUFFER_SIZE	65536 /* up to 64k bytes */
#define CMDSEQ_FIFO_SPACE_THRESHOLD   192
#define WORD_SIZE   4

struct dpu_be_fence {
	struct dma_fence base;
	spinlock_t lock;
	atomic_t refcnt;
	bool signaled;
};

struct dpu_bliteng {
	struct device		*dev;
	void __iomem *base;
	s32 id;
	struct mutex mutex;

	s32 irq_comctrl_sw[4];

	struct semaphore sema[4];
	struct dpu_be_fence *fence[4];
	s32 next_fence_idx;

	atomic64_t seqno;
	spinlock_t lock;
	u64 context;

	void *buffer_addr_virt;
	u32 buffer_addr_phy;

	u32 *cmd_list;

	struct dpu_soc *dpu;

	struct dprc *dprc[2];

	bool start;
	bool sync;

	u64 modifier;
};

#endif
