/*
 * Copyright 2017 NXP
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _UAPI_IMX_DRM_H_
#define _UAPI_IMX_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_IMX_DPU_SET_CMDLIST                 0x00
#define DRM_IMX_DPU_WAIT                        0x01
#define DRM_IMX_DPU_GET_PARAM                   0x02

#define DRM_IOCTL_IMX_DPU_SET_CMDLIST   DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_IMX_DPU_SET_CMDLIST, struct drm_imx_dpu_set_cmdlist)
#define DRM_IOCTL_IMX_DPU_WAIT          DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_IMX_DPU_WAIT, struct drm_imx_dpu_wait)
#define DRM_IOCTL_IMX_DPU_GET_PARAM     DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_IMX_DPU_GET_PARAM, enum drm_imx_dpu_param)

/**
 * struct drm_imx_dpu_set_cmdlist - ioctl argument for
 * DRM_IMX_DPU_SET_CMDLIST.
 */
struct drm_imx_dpu_set_cmdlist {
	__u64	cmd;
	__u32	cmd_nr;

	/* reserved */
	__u64	user_data;
};

/**
 * struct drm_imx_dpu_wait - ioctl argument for
 * DRM_IMX_DPU_WAIT.
 *
 */
struct drm_imx_dpu_wait {
	/* reserved */
	__u64   user_data;
};

/**
 * enum drm_imx_dpu_param - ioctl argument for
 * DRM_IMX_DPU_GET_PARAM.
 *
 */
enum drm_imx_dpu_param {
	DRM_IMX_MAX_DPUS,
};

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_IMX_DRM_H_ */
