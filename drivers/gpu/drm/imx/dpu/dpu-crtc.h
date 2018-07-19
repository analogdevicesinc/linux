/*
 * Copyright 2017-2018 NXP
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

#ifndef _DPU_CRTC_H_
#define _DPU_CRTC_H_

#include <video/dpu.h>
#include "dpu-plane.h"
#include "imx-drm.h"

struct dpu_crtc {
	struct device		*dev;
	struct drm_crtc		base;
	struct imx_drm_crtc	*imx_crtc;
	struct dpu_constframe	*cf;
	struct dpu_disengcfg	*dec;
	struct dpu_extdst	*ed;
	struct dpu_framegen	*fg;
	struct dpu_tcon		*tcon;
	struct dpu_store	*st;
	struct dpu_constframe	*aux_cf;
	struct dpu_disengcfg	*aux_dec;
	struct dpu_extdst	*aux_ed;
	struct dpu_framegen	*aux_fg;
	struct dpu_tcon		*aux_tcon;
	/* master */
	struct dpu_constframe	*m_cf;
	struct dpu_disengcfg	*m_dec;
	struct dpu_extdst	*m_ed;
	struct dpu_framegen	*m_fg;
	struct dpu_tcon		*m_tcon;
	/* slave */
	struct dpu_constframe	*s_cf;
	struct dpu_disengcfg	*s_dec;
	struct dpu_extdst	*s_ed;
	struct dpu_framegen	*s_fg;
	struct dpu_tcon		*s_tcon;
	struct dpu_plane	**plane;
	unsigned int		hw_plane_num;
	unsigned int		stream_id;
	unsigned int		crtc_grp_id;
	unsigned int		syncmode_min_prate;
	unsigned int		singlemode_max_width;
	int			vbl_irq;
	int			safety_shdld_irq;
	int			content_shdld_irq;
	int			dec_shdld_irq;

	bool			has_pc;

	struct completion	safety_shdld_done;
	struct completion	content_shdld_done;
	struct completion	dec_shdld_done;
};

struct dpu_crtc_state {
	struct imx_crtc_state	imx_crtc_state;
	struct dpu_plane_state	**dpu_plane_states;
	bool			use_pc;
};

static inline struct dpu_crtc_state *to_dpu_crtc_state(struct imx_crtc_state *s)
{
	return container_of(s, struct dpu_crtc_state, imx_crtc_state);
}

static inline struct dpu_crtc *to_dpu_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct dpu_crtc, base);
}

struct dpu_plane_state **
crtc_state_get_dpu_plane_states(struct drm_crtc_state *state);

#endif
