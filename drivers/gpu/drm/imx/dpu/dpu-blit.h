/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2021 NXP
 */

#ifndef _DPU_DRM_BLIT_H_
#define _DPU_DRM_BLIT_H_

#include <drm/drm_ioctl.h>

#if IS_ENABLED(CONFIG_DRM_IMX_DPU)
extern const struct drm_ioctl_desc imx_drm_dpu_ioctls[3];
#else
const struct drm_ioctl_desc imx_drm_dpu_ioctls[] = {};
#endif

#endif /* _DPU_DRM_BLIT_H_ */
