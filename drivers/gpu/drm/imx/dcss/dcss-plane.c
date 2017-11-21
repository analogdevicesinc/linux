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

#include <drm/drmP.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_atomic.h>

#include "video/imx-dcss.h"
#include "dcss-plane.h"

static const u32 dcss_common_formats[] = {
	/* RGB */
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_XRGB2101010,
	DRM_FORMAT_XBGR2101010,
	DRM_FORMAT_RGBX1010102,
	DRM_FORMAT_BGRX1010102,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBA1010102,
	DRM_FORMAT_BGRA1010102,

	/* YUV444 */
	DRM_FORMAT_AYUV,

	/* YUV422 */
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,

	/* YUV420 */
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

static inline struct dcss_plane *to_dcss_plane(struct drm_plane *p)
{
	return container_of(p, struct dcss_plane, base);
}

static void dcss_plane_destroy(struct drm_plane *plane)
{
	struct dcss_plane *dcss_plane = container_of(plane, struct dcss_plane,
						     base);

	DRM_DEBUG_KMS("destroy plane\n");

	drm_plane_cleanup(plane);
	kfree(dcss_plane);
}

static int dcss_plane_atomic_set_property(struct drm_plane *plane,
					  struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t val)
{
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);

	if (property == dcss_plane->alpha_prop)
		dcss_plane->alpha_val = val;
	else
		return -EINVAL;

	return 0;
}

static int dcss_plane_atomic_get_property(struct drm_plane *plane,
					  const struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t *val)
{
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);

	if (property == dcss_plane->alpha_prop)
		*val = dcss_plane->alpha_val;
	else
		return -EINVAL;

	return 0;
}

static const struct drm_plane_funcs dcss_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= dcss_plane_destroy,
	.reset		= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.atomic_set_property = dcss_plane_atomic_set_property,
	.atomic_get_property = dcss_plane_atomic_get_property,
};

static int dcss_plane_atomic_check(struct drm_plane *plane,
				   struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *cma_obj;
	struct drm_crtc_state *crtc_state;
	int hdisplay, vdisplay;

	if (!fb)
		return 0;

	if (!state->crtc)
		return -EINVAL;

	cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	WARN_ON(!cma_obj);

	crtc_state = drm_atomic_get_existing_crtc_state(state->state,
							state->crtc);

	hdisplay = crtc_state->adjusted_mode.hdisplay;
	vdisplay = crtc_state->adjusted_mode.vdisplay;

	/* We don't support cropping yet */
	if (state->crtc_x < 0 || state->crtc_y < 0 ||
	    state->crtc_x + state->crtc_w > hdisplay ||
	    state->crtc_y + state->crtc_h > vdisplay)
		return -EINVAL;

	return 0;
}

static void dcss_plane_atomic_set_base(struct dcss_plane *dcss_plane)
{
	struct drm_plane *plane = &dcss_plane->base;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	unsigned long p1_ba, p2_ba;
	bool modifiers_present = !!(fb->flags & DRM_MODE_FB_MODIFIERS);
	u32 pix_format = state->fb->format->format;

	BUG_ON(!cma_obj);

	p1_ba = cma_obj->paddr + fb->offsets[0] +
		fb->pitches[0] * (state->src_y >> 16) +
		fb->format->cpp[0] * (state->src_x >> 16);

	p2_ba = cma_obj->paddr + fb->offsets[1] +
		fb->pitches[1] * (state->src_y >> 16) +
		fb->format->cpp[0] * (state->src_x >> 16);

	dcss_dpr_addr_set(dcss_plane->dcss, dcss_plane->ch_num,
			  p1_ba, p2_ba, fb->pitches[0]);

	if (!modifiers_present || (pix_format != DRM_FORMAT_NV12 &&
				   pix_format != DRM_FORMAT_NV21)) {
		dcss_dtrc_bypass(dcss_plane->dcss, dcss_plane->ch_num);
		return;
	}

	dcss_dtrc_addr_set(dcss_plane->dcss, dcss_plane->ch_num, p1_ba, p2_ba);
}

static bool dcss_plane_needs_setup(struct drm_plane_state *state,
				   struct drm_plane_state *old_state)
{
	return state->crtc_x != old_state->crtc_x ||
	       state->crtc_y != old_state->crtc_y ||
	       state->crtc_w != old_state->crtc_w ||
	       state->crtc_h != old_state->crtc_h ||
	       state->src_x  != old_state->src_x  ||
	       state->src_y  != old_state->src_y  ||
	       state->src_w  != old_state->src_w  ||
	       state->src_h  != old_state->src_h  ||
	       state->fb->format->format != old_state->fb->format->format;
}

static void dcss_plane_atomic_update(struct drm_plane *plane,
				     struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);
	u32 pixel_format = state->fb->format->format;
	struct drm_crtc_state *crtc_state = state->crtc->state;

	if (!state->fb)
		return;

	if (old_state->fb && !drm_atomic_crtc_needs_modeset(crtc_state) &&
	    !dcss_plane_needs_setup(state, old_state) &&
	    !dcss_dtg_global_alpha_changed(dcss_plane->dcss, dcss_plane->ch_num,
					   pixel_format,
					   dcss_plane->alpha_val)) {
		dcss_plane_atomic_set_base(dcss_plane);
		return;
	}

	dcss_dtrc_set_res(dcss_plane->dcss, dcss_plane->ch_num,
			  state->src_w >> 16, state->src_h >> 16);

	dcss_dpr_format_set(dcss_plane->dcss, dcss_plane->ch_num, pixel_format);
	dcss_dpr_set_res(dcss_plane->dcss, dcss_plane->ch_num,
			 state->src_w >> 16, state->src_h >> 16);
	dcss_plane_atomic_set_base(dcss_plane);

	dcss_scaler_setup(dcss_plane->dcss, dcss_plane->ch_num,
			  pixel_format, state->src_w >> 16,
			  state->src_h >> 16, state->crtc_w, state->crtc_h);

	/*
	 * TODO: retrieve the output colorspace format from somewhere... For
	 * now, assume RGB.
	 */
	dcss_hdr10_pipe_csc_setup(dcss_plane->dcss, dcss_plane->ch_num,
				  dcss_drm_fourcc_to_colorspace(pixel_format),
				  DCSS_COLORSPACE_RGB);

	dcss_dtg_plane_pos_set(dcss_plane->dcss, dcss_plane->ch_num,
			       state->crtc_x, state->crtc_y,
			       state->crtc_w, state->crtc_h);
	dcss_dtg_plane_alpha_set(dcss_plane->dcss, dcss_plane->ch_num,
				 pixel_format, dcss_plane->alpha_val);

	dcss_dpr_enable(dcss_plane->dcss, dcss_plane->ch_num, true);
	dcss_scaler_enable(dcss_plane->dcss, dcss_plane->ch_num, true);
	dcss_dtg_ch_enable(dcss_plane->dcss, dcss_plane->ch_num, true);
}

static void dcss_plane_atomic_disable(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);

	dcss_dpr_enable(dcss_plane->dcss, dcss_plane->ch_num, false);
	dcss_scaler_enable(dcss_plane->dcss, dcss_plane->ch_num, false);
	dcss_dtg_plane_pos_set(dcss_plane->dcss, dcss_plane->ch_num,
			       0, 0, 0, 0);
	dcss_dtg_ch_enable(dcss_plane->dcss, dcss_plane->ch_num, false);
}

static const struct drm_plane_helper_funcs dcss_plane_helper_funcs = {
	.atomic_check = dcss_plane_atomic_check,
	.atomic_update = dcss_plane_atomic_update,
	.atomic_disable = dcss_plane_atomic_disable,
};

struct dcss_plane *dcss_plane_init(struct drm_device *drm,
				   struct dcss_soc *dcss,
				   unsigned int possible_crtcs,
				   enum drm_plane_type type,
				   unsigned int zpos)
{
	struct dcss_plane *dcss_plane;
	int ret;

	if (zpos > 2)
		return ERR_PTR(-EINVAL);

	dcss_plane = kzalloc(sizeof(*dcss_plane), GFP_KERNEL);
	if (!dcss_plane) {
		DRM_ERROR("failed to allocate plane\n");
		return ERR_PTR(-ENOMEM);
	}

	dcss_plane->dcss = dcss;

	ret = drm_universal_plane_init(drm, &dcss_plane->base, possible_crtcs,
				       &dcss_plane_funcs, dcss_common_formats,
				       ARRAY_SIZE(dcss_common_formats),
				       NULL, type, NULL);
	if (ret) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(dcss_plane);
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(&dcss_plane->base, &dcss_plane_helper_funcs);

	ret = drm_plane_create_zpos_immutable_property(&dcss_plane->base, zpos);
	if (ret)
		return ERR_PTR(ret);

	dcss_plane->ch_num = 2 - zpos;
	dcss_plane->alpha_val = 255;

	return dcss_plane;
}
