/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2023 NXP
 */

#ifndef __DPU95_DRV_H__
#define __DPU95_DRV_H__

#include <drm/drm_device.h>
#include <drm/drm_encoder.h>

#include "dpu95-crtc.h"
#include "dpu95-plane.h"
#include "dpu95-blit.h"

#define DPU95_CRTCS	2
#define DPU95_ENCODERS	DPU95_CRTCS
#define DPU95_PRIMARYS	DPU95_CRTCS
#define DPU95_OVERLAYS	5
#define DPU95_HW_PLANES	6

struct dpu95_drm_device {
	struct drm_device	base;
	struct dpu95_soc	dpu_soc;
	struct dpu_bliteng	dpu_be;
	struct dpu95_crtc	dpu_crtc[DPU95_CRTCS];
	struct dpu95_plane	dpu_primary[DPU95_PRIMARYS];
	struct dpu95_plane	dpu_overlay[DPU95_OVERLAYS];
	struct dpu95_plane_grp	dpu_plane_grp;
	struct drm_encoder	encoder[DPU95_ENCODERS];
	u32			crtc_mask;
};

static inline struct dpu95_drm_device *
to_dpu95_drm_device(struct drm_device *drm)
{
	return container_of(drm, struct dpu95_drm_device, base);
}

int dpu95_core_init(struct dpu95_drm_device *dpu_drm);

int dpu95_kms_prepare(struct dpu95_drm_device *dpu_drm);
void dpu95_kms_unprepare(struct dpu95_drm_device *dpu_drm);

#endif /* __DPU95_DRV_H__ */
