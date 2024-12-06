/*
 * Copyright 2018,2021 NXP
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
#include <drm/drm_vblank.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_rect.h>
#include <video/imx-lcdif.h>

#include "lcdif-plane.h"
#include "lcdif-kms.h"

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
				    struct drm_atomic_state *state)
{
	int ret;
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state,
									     plane);
	struct drm_plane_state *old_state = plane->state;
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_framebuffer *old_fb = old_state->fb;
	struct drm_crtc_state *crtc_state;
	bool use_i80;

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

	crtc_state = drm_atomic_get_existing_crtc_state(state,
							plane_state->crtc);

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);

	if (ret)
		return ret;

	if (!plane_state->visible)
		return -EINVAL;

	/* force 'mode_changed' when fb pitches or format
	 * changed, since the pitch and format related
	 * registers configuration of LCDIF can not be
	 * done when LCDIF is running and 'mode_changed'
	 * means a full modeset is required.
	 */
	if (old_fb && likely(!crtc_state->mode_changed)) {
		if (old_fb->pitches[0] != fb->pitches[0] ||
		    old_fb->format->format != fb->format->format)
			crtc_state->mode_changed = true;
	}

	/* Add affected connectors to check if we use i80 mode or not. */
	ret = drm_atomic_add_affected_connectors(state, plane_state->crtc);
	if (ret)
		return ret;

	use_i80 = lcdif_drm_connector_is_self_refresh_aware(state);

	/* Do not support cropping in i80 mode. */
	if (use_i80 && (plane_state->src_w >> 16 != fb->width))
		return -EINVAL;

	return 0;
}

static void lcdif_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct lcdif_plane *lcdif_plane = to_lcdif_plane(plane);
	struct lcdif_soc *lcdif = lcdif_plane->lcdif;
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct drm_framebuffer *fb = new_plane_state->fb;
	struct drm_gem_dma_object *gem_obj = NULL;
	u32 fb_addr, src_off, src_w, fb_idx, cpp, stride;
	bool crop;
	bool use_i80 = lcdif_drm_connector_is_self_refresh_aware(state);

	/* plane and crtc is disabling */
	if (!fb)
		return;

	/* TODO: for now we just update the next buf addr
	 * and the fb pixel format, since the mode set will
	 * be done in crtc's ->enable() helper func
	 */
	switch (plane->type) {
	case DRM_PLANE_TYPE_PRIMARY:
		/* TODO: only support RGB */
		gem_obj = drm_fb_dma_get_gem_obj(fb, 0);
		src_off = (new_plane_state->src_y >> 16) * fb->pitches[0] +
			  (new_plane_state->src_x >> 16) * fb->format->cpp[0];
		fb_addr = gem_obj->dma_addr + fb->offsets[0] + src_off;
		fb_idx  = 0;
		break;
	default:
		/* TODO: add overlay later */
		return;
	}

	lcdif_set_fb_addr(lcdif, fb_idx, fb_addr, use_i80);

	/* Config pixel format and horizontal cropping
	 * if CRTC needs a full modeset which needs to
	 * enable LCDIF to run at the end.
	 */
	if (unlikely(drm_atomic_crtc_needs_modeset(new_plane_state->crtc->state))) {
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			lcdif_set_pix_fmt(lcdif, fb->format->format);

		cpp = fb->format->cpp[0];
		stride = DIV_ROUND_UP(fb->pitches[0], cpp);

		src_w = new_plane_state->src_w >> 16;
		WARN_ON(src_w > fb->width);

		crop  = src_w != stride ? true : false;
		lcdif_set_fb_hcrop(lcdif, src_w, stride, crop);

		lcdif_enable_controller(lcdif, use_i80);
	} else if (use_i80) {
		lcdif_enable_controller(lcdif, use_i80);
	}
}

static void lcdif_plane_atomic_disable(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct drm_framebuffer *fb = new_plane_state->fb;

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
	dev->mode_config.fb_modifiers_not_supported = true;

	if (zpos)
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
