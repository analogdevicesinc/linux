/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2023 NXP
 */

#ifndef __DPU95_BLIT_H__
#define __DPU95_BLIT_H__

#define COMMAND_BUFFER_SIZE	65536 /* up to 64k bytes */
#define CMDSEQ_FIFO_SPACE_THRESHOLD   192
#define WORD_SIZE   4

#include <drm/drm_ioctl.h>

struct dpu_bliteng {
	struct device		*dev;
	void __iomem *base;
	struct mutex mutex;

	void *buffer_addr_virt;
	u32 buffer_addr_phy;

	u32 *cmd_list;

	struct dpu95_soc *dpu;
};

struct dpu95_drm_device;

extern const struct drm_ioctl_desc imx_drm_dpu95_ioctls[4];

int dpu95_bliteng_load(struct dpu95_drm_device *dpu_drm);
void dpu95_bliteng_unload(struct dpu95_drm_device *dpu_drm);
int dpu95_bliteng_runtime_suspend(struct dpu95_drm_device *dpu_drm);
int dpu95_bliteng_runtime_resume(struct dpu95_drm_device *dpu_drm);

#endif
