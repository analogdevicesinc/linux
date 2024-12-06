/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2017-2020,2022,2023 NXP
 */

#ifndef __DPU95_PLANE_H__
#define __DPU95_PLANE_H__

#include <linux/kernel.h>

#include <drm/drm_device.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>

#include "dpu95.h"

#define dpu95_plane_dbg(plane, fmt, ...)				\
do {									\
	typeof(plane) _plane = (plane);					\
	drm_dbg_kms(_plane->dev, "[PLANE:%d:%s] " fmt,			\
		    _plane->base.id, _plane->name, ##__VA_ARGS__);	\
} while (0)

#define for_each_old_plane_state_in_state(__state, old_plane_state, __i) \
	for ((__i) = 0;							\
	     (__i) < (__state)->dev->mode_config.num_total_plane;	\
	     (__i)++)							\
		for_each_if ((__state)->planes[__i].ptr &&		\
			     ((old_plane_state) = (__state)->planes[__i].old_state, 1))

struct dpu95_drm_device;

struct dpu95_plane {
	struct drm_plane	base;
	struct dpu95_plane_grp	*grp;
};

union dpu95_plane_stage {
	struct dpu95_constframe	*cf;
	struct dpu95_layerblend	*lb;
	void			*ptr;
};

struct dpu95_plane_state {
	struct drm_plane_state	base;
	union dpu95_plane_stage	stage;
	struct dpu95_fetchunit	*source;
	struct dpu95_layerblend	*blend;
	bool			is_top;
};

static inline struct dpu95_plane *to_dpu95_plane(struct drm_plane *plane)
{
	return container_of(plane, struct dpu95_plane, base);
}

static inline struct dpu95_plane_state *
to_dpu95_plane_state(struct drm_plane_state *plane_state)
{
	return container_of(plane_state, struct dpu95_plane_state, base);
}

int dpu95_plane_initialize(struct dpu95_drm_device *dpu_drm,
			   struct dpu95_plane *dpu_plane, u32 possible_crtcs,
			   enum drm_plane_type type);

#endif /* __DPU95_PLANE_H__ */
