// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2020,2021 NXP
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_rect.h>

#include "dcnano-drv.h"
#include "dcnano-reg.h"

#define DCNANO_FB_PITCH_ALIGN	128	/* in byte */

#define dcnano_plane_dbg(plane, fmt, ...)				\
	drm_dbg_kms((plane)->dev, "[PLANE:%d:%s] " fmt,			\
		    (plane)->base.id, (plane)->name, ##__VA_ARGS__)

/* primary plane formats */
static const u32 dcnano_primary_plane_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
};

static unsigned int
dcnano_primary_plane_format_count = ARRAY_SIZE(dcnano_primary_plane_formats);

static inline struct dcnano_dev *plane_to_dcnano_dev(struct drm_plane *plane)
{
	return to_dcnano_dev(plane->dev);
}

static inline dma_addr_t
drm_plane_state_to_baseaddr(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *dma_obj;
	unsigned int x = state->src.x1 >> 16;
	unsigned int y = state->src.y1 >> 16;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 0);

	return dma_obj->dma_addr + fb->offsets[0] + fb->pitches[0] * y +
	       fb->format->cpp[0] * x;
}

/***************************/
/* primary plane functions */
/***************************/

static int dcnano_primary_plane_atomic_check(struct drm_plane *plane,
					     struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state,
									   plane);
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(state,
									   plane);
	struct dcnano_dev *dcnano = plane_to_dcnano_dev(plane);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_framebuffer *old_fb = old_state->fb;
	u32 src_w;
	unsigned int pitch_no_padding;
	int ret;

	/* ok to disable */
	if (!fb)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, &dcnano->crtc);
	if (WARN_ON(!crtc_state))
		return -EINVAL;

	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret)
		return ret;

	if (fb->pitches[0] > FBSTRIDE_MAX) {
		dcnano_plane_dbg(plane, "fb pitches[0] 0x%08x is out of range\n",
				 fb->pitches[0]);
		return -EINVAL;
	}

	/*
	 * The primary plane's stride value in register has to be 128byte
	 * aligned, _but_ no dedicated padding is allowed.
	 */
	src_w = drm_rect_width(&new_state->src) >> 16;
	pitch_no_padding = fb->format->cpp[0] * src_w;
	if (fb->pitches[0] != pitch_no_padding) {
		dcnano_plane_dbg(plane,
				 "fb pitches[0] 0x%08x should be no padding - 0x%08x\n",
				 fb->pitches[0], pitch_no_padding);
		return -EINVAL;
	}

	/*
	 * Force CRTC mode change if framebuffer stride or pixel format
	 * are changed.
	 */
	if (old_fb &&
	    (fb->pitches[0] != old_fb->pitches[0] ||
	     fb->format->format != old_fb->format->format))
		crtc_state->mode_changed = true;

	return 0;
}

static void dcnano_primary_plane_atomic_update(struct drm_plane *plane,
					       struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state,
									   plane);
	struct dcnano_dev *dcnano = plane_to_dcnano_dev(plane);
	struct drm_framebuffer *fb = new_state->fb;
	dma_addr_t baseaddr;

	if (!fb) {
		dcnano_plane_dbg(plane, "no fb\n");
		return;
	}

	baseaddr = drm_plane_state_to_baseaddr(new_state);

	dcnano_plane_dbg(plane, "fb address %pad, pitch 0x%08x\n",
			 &baseaddr, fb->pitches[0]);

	dcnano_write(dcnano, DCNANO_FRAMEBUFFERADDRESS, baseaddr);

	dcnano_write(dcnano, DCNANO_FRAMEBUFFERSTRIDE,
		     ALIGN(fb->pitches[0], DCNANO_FB_PITCH_ALIGN));
}

static const struct drm_plane_helper_funcs dcnano_primary_plane_helper_funcs = {
	.prepare_fb	= drm_gem_plane_helper_prepare_fb,
	.atomic_check	= dcnano_primary_plane_atomic_check,
	.atomic_update	= dcnano_primary_plane_atomic_update,
};

static const struct drm_plane_funcs dcnano_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

int dcnano_plane_init(struct dcnano_dev *dcnano)
{
	int ret;

	/* primary plane */
	drm_plane_helper_add(&dcnano->primary,
			     &dcnano_primary_plane_helper_funcs);
	ret = drm_universal_plane_init(&dcnano->base, &dcnano->primary, 0,
				       &dcnano_plane_funcs,
				       dcnano_primary_plane_formats,
				       dcnano_primary_plane_format_count,
				       NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		drm_err(&dcnano->base,
			"failed to initialize primary plane: %d\n", ret);

	return ret;
}
