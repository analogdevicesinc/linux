/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LCDIF_PLANE_H
#define __LCDIF_PLANE_H

#include <drm/drm_plane.h>
#include <video/imx-lcdif.h>

struct lcdif_plane {
	struct drm_plane base;
	struct lcdif_soc *lcdif;
};

#define to_lcdif_plane(plane) container_of(plane, struct lcdif_plane, base)

struct lcdif_plane *lcdif_plane_init(struct drm_device *drm,
				     struct lcdif_soc *lcdif,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos);

void lcdif_plane_deinit(struct drm_device *dev,
			struct lcdif_plane *lcdif_plane);

#endif
