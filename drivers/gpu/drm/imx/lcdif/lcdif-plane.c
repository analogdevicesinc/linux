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

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_rect.h>
#include <video/imx-lcdif.h>

#include "lcdif-plane.h"

static uint32_t lcdif_pixel_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ABGR1555,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_BGR565,
};

static int lcdif_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *plane_state)
{
	int ret;
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_crtc_state *crtc_state;
	struct drm_display_mode *mode;
	struct drm_rect clip = { 0 };
	unsigned int flags;

	/* 'fb' should also be NULL which has been checked in
	 * the core sanity check function 'drm_atomic_plane_check()'
	 */
	if (!plane_state->crtc) {
		WARN_ON(fb);
		return 0;
	}

	/* lcdif crtc can only display from (0,0) for each plane */
	if (plane_state->crtc_x || plane_state->crtc_y)
		return -EINVAL;

	crtc_state = drm_atomic_get_existing_crtc_state(plane_state->state,
							plane_state->crtc);
	mode = &crtc_state->adjusted_mode;

	/* check fb pixel format matches bus format */
	flags = mode->private_flags & 0xffff;

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		if (flags != MEDIA_BUS_FMT_RGB565_1X16)
			return -EINVAL;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		if (flags != MEDIA_BUS_FMT_RGB888_1X24)
			return -EINVAL;
		break;
	default:
		/* TODO: add other formats support later */
		return -EINVAL;
	}

	clip.x2 = mode->hdisplay;
	clip.y2 = mode->vdisplay;

	ret = drm_plane_helper_check_state(plane_state, &clip,
					   DRM_PLANE_HELPER_NO_SCALING,
					   DRM_PLANE_HELPER_NO_SCALING,
					   false, true);
	if (ret)
		return ret;

	if (!plane_state->visible)
		return -EINVAL;

	return 0;
}

static void lcdif_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct lcdif_plane *lcdif_plane = to_lcdif_plane(plane);
	struct lcdif_soc *lcdif = lcdif_plane->lcdif;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_framebuffer *old_fb = old_state->fb;
	struct drm_gem_cma_object *gem_obj = NULL;
	u32 fb_addr, src_off, fb_idx;

	/* plane and crtc is disabling */
	if (!fb)
		return;

	/* TODO: for now we just update the next buf addr
	 * and the fb pixel format, since the mode set will
	 * be done in crtc's ->enable() helper func
	 */
	if (plane->type == DRM_PLANE_TYPE_PRIMARY &&
	    (!old_fb || fb->format->format != old_fb->format->format))
		lcdif_set_pix_fmt(lcdif, fb->format->format);

	switch (plane->type) {
	case DRM_PLANE_TYPE_PRIMARY:
		/* TODO: only support RGB */
		gem_obj = drm_fb_cma_get_gem_obj(fb, 0);
		src_off = (state->src_y >> 16) * fb->pitches[0] +
			  (state->src_x >> 16) * fb->format->cpp[0];
		fb_addr = gem_obj->paddr + fb->offsets[0] + src_off;
		fb_idx  = 0;
		break;
	default:
		/* TODO: add overlay later */
		return;
	}

	lcdif_set_fb_addr(lcdif, fb_idx, fb_addr);

	lcdif_enable_controller(lcdif);
}

static void lcdif_plane_atomic_disable(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;

	WARN_ON(fb);

	/* TODO: CRTC disabled has been done by CRTC helper function,
	 * so it seems that no more required, the only possible thing
	 * is to set next buf addr to 0 in CRTC
	 */
}

static const struct drm_plane_helper_funcs lcdif_plane_helper_funcs = {
	.atomic_check	= lcdif_plane_atomic_check,
	.atomic_update	= lcdif_plane_atomic_update,
	.atomic_disable	= lcdif_plane_atomic_disable,
};

static void lcdif_plane_destroy(struct drm_plane *plane)
{
	struct lcdif_plane *lcdif_plane = to_lcdif_plane(plane);

	drm_plane_cleanup(plane);
	kfree(lcdif_plane);
}

static const struct drm_plane_funcs lcdif_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= lcdif_plane_destroy,
	.reset		= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

struct lcdif_plane *lcdif_plane_init(struct drm_device *dev,
				     struct lcdif_soc *lcdif,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos)
{
	int ret;
	struct lcdif_plane *lcdif_plane;

	/* lcdif doesn't support fb modifiers */
	if (zpos || dev->mode_config.allow_fb_modifiers)
		return ERR_PTR(-EINVAL);

	lcdif_plane = kzalloc(sizeof(*lcdif_plane), GFP_KERNEL);
	if (!lcdif_plane)
		return ERR_PTR(-ENOMEM);

	lcdif_plane->lcdif = lcdif;

	drm_plane_helper_add(&lcdif_plane->base, &lcdif_plane_helper_funcs);
	ret = drm_universal_plane_init(dev, &lcdif_plane->base, possible_crtcs,
				       &lcdif_plane_funcs, lcdif_pixel_formats,
				       ARRAY_SIZE(lcdif_pixel_formats), NULL,
				       type, NULL);
	if (ret) {
		kfree(lcdif_plane);
		return ERR_PTR(ret);
	}

	ret = drm_plane_create_zpos_immutable_property(&lcdif_plane->base, zpos);
	if (ret) {
		kfree(lcdif_plane);
		return ERR_PTR(ret);
	}

	return lcdif_plane;
}

void lcdif_plane_deinit(struct drm_device *dev,
			struct lcdif_plane *lcdif_plane)
{
	struct drm_plane *plane = &lcdif_plane->base;

	if (plane->zpos_property)
		drm_property_destroy(dev, plane->zpos_property);

	lcdif_plane_destroy(plane);
}
