/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019,2020,2022 NXP
 */

#ifndef __LCDIFV3_PLANE_H
#define __LCDIFV3_PLANE_H

#include <drm/drm_atomic.h>
#include <drm/drm_plane.h>
#include <video/imx-lcdifv3.h>

struct lcdifv3_plane {
	struct drm_plane base;
	struct lcdifv3_soc *lcdifv3;
};

#define to_lcdifv3_plane(plane) container_of(plane, struct lcdifv3_plane, base)

void lcdifv3_plane_atomic_update(struct drm_plane *plane,
				 struct drm_atomic_state *state);

struct lcdifv3_plane *lcdifv3_plane_init(struct drm_device *drm,
				     struct lcdifv3_soc *lcdifv3,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos);
#endif
