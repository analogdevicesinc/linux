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

void dpu_be_configure_prefetch(struct dpu_bliteng *dpu_be,
							u32 width, u32 height,
							u32 x_offset, u32 y_offset,
							u32 stride, u32 format, u64 modifier,
							u64 baddr, u64 uv_addr);
int dpu_bliteng_get_empty_instance(struct dpu_bliteng **dpu_be,
							struct device *dev);
u32 *dpu_bliteng_get_cmd_list(struct dpu_bliteng *dpu_be);
s32 dpu_bliteng_get_id(struct dpu_bliteng *dpu_be);
void dpu_bliteng_set_id(struct dpu_bliteng *dpu_be, int id);
void dpu_bliteng_set_dev(struct dpu_bliteng *dpu_be, struct device *dev);
void dpu_be_get(struct dpu_bliteng *dpu_be);
void dpu_be_put(struct dpu_bliteng *dpu_be);
int dpu_be_get_fence(struct dpu_bliteng *dpu_be, int dpu_num);
int dpu_be_set_fence(struct dpu_bliteng *dpu_be, int fd);
int dpu_be_blit(struct dpu_bliteng *dpu_be, u32 *cmdlist, u32 cmdnum);
int dpu_bliteng_init(struct dpu_bliteng *dpu_bliteng);
void dpu_bliteng_fini(struct dpu_bliteng *dpu_bliteng);

#endif
