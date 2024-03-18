/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2023 NXP
 */

#ifndef __DPU95_BLIT_H__
#define __DPU95_BLIT_H__

#define COMMAND_BUFFER_SIZE	16384 /* ocram_d 16k bytes */
#define CMDSEQ_OCRAM_D_ADDR 0x300000
#define CMDSEQ_FIFO_SPACE_THRESHOLD   192
#define WORD_SIZE   4

#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/sync_file.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-array.h>
#include <drm/drm_ioctl.h>

struct dpu_be_fence {
	struct dma_fence base;
	spinlock_t lock;
	bool signaled;
};

struct dpu_bliteng {
	struct device		*dev;
	void __iomem *base;
	struct mutex mutex;

	s32 irq_comctrl_sw[4];

	struct semaphore sema[4];
	struct dpu_be_fence *fence[4];
	s32 next_fence_idx;

	atomic64_t seqno;
	spinlock_t lock;
	u64 context;

	u32 *cmd_list;

	struct dpu95_soc *dpu;

	bool ready;
};

struct dpu95_drm_device;

extern const struct drm_ioctl_desc imx_drm_dpu95_ioctls[4];

int dpu95_bliteng_load(struct dpu95_drm_device *dpu_drm);
void dpu95_bliteng_unload(struct dpu95_drm_device *dpu_drm);
int dpu95_bliteng_runtime_suspend(struct dpu95_drm_device *dpu_drm);
int dpu95_bliteng_runtime_resume(struct dpu95_drm_device *dpu_drm);

#endif
