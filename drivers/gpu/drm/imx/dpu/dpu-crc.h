/*
 * Copyright 2019,2020 NXP
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

#ifndef _DPU_CRC_H_
#define _DPU_CRC_H_

#include "dpu-crtc.h"

enum {
	DPU_DUAL_CRC_FLAG_DUAL,
	DPU_DUAL_CRC_FLAG_LEFT,
	DPU_DUAL_CRC_FLAG_RIGHT,
	DPU_DUAL_CRC_FLAG_ERR_NONE,
};

static inline bool to_enable_dpu_crc(struct dpu_crtc_state *new_dcstate,
				     struct dpu_crtc_state *old_dcstate)
{
	return old_dcstate->crc.source == DPU_CRC_SRC_NONE &&
	       new_dcstate->crc.source != DPU_CRC_SRC_NONE;
}

static inline bool to_disable_dpu_crc(struct dpu_crtc_state *new_dcstate,
				      struct dpu_crtc_state *old_dcstate)
{
	return old_dcstate->crc.source != DPU_CRC_SRC_NONE &&
	       new_dcstate->crc.source == DPU_CRC_SRC_NONE;
}

static inline void dpu_copy_roi(struct drm_rect *from, struct drm_rect *to)
{
	to->x1 = from->x1;
	to->y1 = from->y1;
	to->x2 = from->x2;
	to->y2 = from->y2;
}

#ifdef CONFIG_DEBUG_FS
int dpu_crtc_verify_crc_source(struct drm_crtc *crtc, const char *source_name,
			       size_t *values_cnt);
int dpu_crtc_set_crc_source(struct drm_crtc *crtc, const char *source_name);
irqreturn_t dpu_crc_valid_irq_threaded_handler(int irq, void *dev_id);
void dpu_crtc_enable_crc_source(struct drm_crtc *crtc,
				enum dpu_crc_source source,
				struct drm_rect *roi);
void dpu_crtc_disable_crc_source(struct drm_crtc *crtc, bool dual_crc);
#else
#define dpu_crtc_verify_crc_source NULL
#define dpu_crtc_set_crc_source	NULL
irqreturn_t dpu_crc_valid_irq_threaded_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
void dpu_crtc_enable_crc_source(struct drm_crtc *crtc,
				enum dpu_crc_source source,
				struct drm_rect *roi)
{
}
void dpu_crtc_disable_crc_source(struct drm_crtc *crtc, bool dual_crc)
{
}
#endif

#endif
