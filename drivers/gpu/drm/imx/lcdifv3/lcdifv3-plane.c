// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019,2020,2022 NXP
 */

#include <linux/module.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_rect.h>
#include <drm/drm_vblank.h>
#include <video/imx-lcdifv3.h>

#include "lcdifv3-plane.h"

static uint32_t lcdifv3_pixel_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_XRGB1555,
};

static int lcdifv3_plane_atomic_check(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	int ret;
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state,
									     plane);
	struct drm_plane_state *old_state = plane->state;
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_framebuffer *old_fb = old_state->fb;
	struct drm_crtc_state *crtc_state;

	/* 'fb' should also be NULL which has been checked in
	 * the core sanity check function 'drm_atomic_plane_check()'
	 */
	if (!plane_state->crtc) {
		WARN_ON(fb);
		return 0;
	}

	/* lcdifv3 crtc can only display from (0,0) for each plane */
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

	/* force 'mode_changed' when fb pitches changed, since
	 * the pitch related registers configuration of LCDIF
	 * can not be done when LCDIF is running.
	 */
	if (old_fb && likely(!crtc_state->mode_changed)) {
		if (old_fb->pitches[0] != fb->pitches[0])
			crtc_state->mode_changed = true;
	}

	return 0;
}

void lcdifv3_plane_atomic_update(struct drm_plane *plane,
				 struct drm_atomic_state *state)
{
	struct lcdifv3_plane *lcdifv3_plane = to_lcdifv3_plane(plane);
	struct lcdifv3_soc *lcdifv3 = lcdifv3_plane->lcdifv3;
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct drm_framebuffer *fb = new_plane_state->fb;
	struct drm_gem_dma_object *gem_obj = NULL;
	u32 fb_addr, src_off, fb_idx;

	/* plane and crtc is disabling */
	if (!fb)
		return;

	/* TODO: for now we just update the next buf addr
	 * and the fb pixel format, since the mode set will
	 * be done in crtc's ->enable() helper func
	 */
	if (plane->type == DRM_PLANE_TYPE_PRIMARY)
		lcdifv3_set_pix_fmt(lcdifv3, fb->format->format);

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

	lcdifv3_set_fb_addr(lcdifv3, fb_idx, fb_addr);
}

static void lcdifv3_plane_atomic_disable(struct drm_plane *plane,
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

static const struct drm_plane_helper_funcs lcdifv3_plane_helper_funcs = {
	.atomic_check	= lcdifv3_plane_atomic_check,
	.atomic_update	= lcdifv3_plane_atomic_update,
	.atomic_disable	= lcdifv3_plane_atomic_disable,
};

static void lcdifv3_plane_destroy(struct drm_plane *plane)
{
	struct lcdifv3_plane *lcdifv3_plane = to_lcdifv3_plane(plane);

	drm_plane_cleanup(plane);
	kfree(lcdifv3_plane);
}

static const struct drm_plane_funcs lcdifv3_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= lcdifv3_plane_destroy,
	.reset		= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

struct lcdifv3_plane *lcdifv3_plane_init(struct drm_device *dev,
				     struct lcdifv3_soc *lcdifv3,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos)
{
	int ret;
	struct lcdifv3_plane *lcdifv3_plane;

	/* lcdifv3 doesn't support fb modifiers */
	dev->mode_config.fb_modifiers_not_supported = true;

	if (zpos)
		return ERR_PTR(-EINVAL);

	lcdifv3_plane = kzalloc(sizeof(*lcdifv3_plane), GFP_KERNEL);
	if (!lcdifv3_plane)
		return ERR_PTR(-ENOMEM);

	lcdifv3_plane->lcdifv3 = lcdifv3;

	drm_plane_helper_add(&lcdifv3_plane->base, &lcdifv3_plane_helper_funcs);
	ret = drm_universal_plane_init(dev, &lcdifv3_plane->base, possible_crtcs,
				       &lcdifv3_plane_funcs, lcdifv3_pixel_formats,
				       ARRAY_SIZE(lcdifv3_pixel_formats), NULL,
				       type, NULL);
	if (ret) {
		kfree(lcdifv3_plane);
		return ERR_PTR(ret);
	}

	ret = drm_plane_create_zpos_immutable_property(&lcdifv3_plane->base, zpos);
	if (ret) {
		kfree(lcdifv3_plane);
		return ERR_PTR(ret);
	}

	return lcdifv3_plane;
}
