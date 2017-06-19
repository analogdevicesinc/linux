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
 */

#ifndef __DPU_PLANE_H__
#define __DPU_PLANE_H__

#include <video/dpu.h>
#include "imx-drm.h"

#define MAX_DPU_PLANE_GRP	(MAX_CRTC / 2)

struct dpu_plane {
	struct drm_plane	base;
	struct dpu_plane_grp	*grp;
	struct list_head	head;
	unsigned int		stream_id;
};

struct dpu_plane_state {
	struct drm_plane_state	base;
	lb_prim_sel_t		stage;
	lb_sec_sel_t		source;
	dpu_block_id_t		blend;
	unsigned int		layer_x;
	unsigned int		layer_y;
	unsigned int		base_x;
	unsigned int		base_y;
	unsigned int		base_w;
	unsigned int		base_h;
	bool			is_top;
	bool			use_prefetch;
};

static const lb_prim_sel_t cf_stages[] = {LB_PRIM_SEL__CONSTFRAME0,
					  LB_PRIM_SEL__CONSTFRAME1};
static const lb_prim_sel_t stages[] = {LB_PRIM_SEL__LAYERBLEND0,
				       LB_PRIM_SEL__LAYERBLEND1,
				       LB_PRIM_SEL__LAYERBLEND2,
				       LB_PRIM_SEL__LAYERBLEND3,
				       LB_PRIM_SEL__LAYERBLEND4,
				       LB_PRIM_SEL__LAYERBLEND5};
static const lb_sec_sel_t sources[] = {LB_SEC_SEL__FETCHDECODE0,
				       LB_SEC_SEL__FETCHDECODE1,
				       LB_SEC_SEL__FETCHDECODE2,
				       LB_SEC_SEL__FETCHDECODE3};
static const dpu_block_id_t blends[] = {ID_LAYERBLEND0, ID_LAYERBLEND1,
					ID_LAYERBLEND2, ID_LAYERBLEND3,
					ID_LAYERBLEND4, ID_LAYERBLEND5};

static inline struct dpu_plane *to_dpu_plane(struct drm_plane *plane)
{
	return container_of(plane, struct dpu_plane, base);
}

static inline struct dpu_plane_state *
to_dpu_plane_state(struct drm_plane_state *plane_state)
{
	return container_of(plane_state, struct dpu_plane_state, base);
}

static inline int source_to_id(lb_sec_sel_t source)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sources); i++) {
		if (source == sources[i])
			return i;
	}

	WARN_ON(1);
	return -EINVAL;
}

static inline int blend_to_id(dpu_block_id_t blend)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(blends); i++) {
		if (blend == blends[i])
			return i;
	}

	WARN_ON(1);
	return -EINVAL;
}

struct dpu_plane *dpu_plane_init(struct drm_device *drm,
				 unsigned int possible_crtcs,
				 unsigned int stream_id,
				 struct dpu_plane_grp *grp,
				 enum drm_plane_type type);
#endif
