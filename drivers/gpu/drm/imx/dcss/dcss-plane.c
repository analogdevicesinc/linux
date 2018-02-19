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
#include "dcss-crtc.h"

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
	DRM_FORMAT_P010,
};

static const u64 dcss_video_format_modifiers[] = {
	DRM_FORMAT_MOD_VSI_G1_TILED,
	DRM_FORMAT_MOD_VSI_G2_TILED,
	DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED,
	DRM_FORMAT_MOD_INVALID,
};

static const u64 dcss_graphics_format_modifiers[] = {
	DRM_FORMAT_MOD_VIVANTE_TILED,
	DRM_FORMAT_MOD_VIVANTE_SUPER_TILED,
	DRM_FORMAT_MOD_VIVANTE_SUPER_TILED_FC,
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
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
	else if (property == dcss_plane->use_global_prop)
		dcss_plane->use_global_val = val;
	else if (property == dcss_plane->dtrc_table_ofs_prop)
		dcss_plane->dtrc_table_ofs_val = val;
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
	else if (property == dcss_plane->use_global_prop)
		*val = dcss_plane->use_global_val;
	else if (property == dcss_plane->dtrc_table_ofs_prop)
		*val = dcss_plane->dtrc_table_ofs_val;
	else
		return -EINVAL;

	return 0;
}

static bool dcss_plane_format_mod_supported(struct drm_plane *plane,
					    uint32_t format,
					    uint64_t modifier)
{
	switch (plane->type) {
	case DRM_PLANE_TYPE_PRIMARY:
		switch (format) {
		case DRM_FORMAT_ARGB8888:
		case DRM_FORMAT_XRGB8888:
		case DRM_FORMAT_ARGB2101010:
			return modifier == DRM_FORMAT_MOD_LINEAR ||
			       modifier == DRM_FORMAT_MOD_VIVANTE_TILED ||
			       modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED ||
			       modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED_FC;
		default:
			return modifier == DRM_FORMAT_MOD_LINEAR;
		}
		break;
	case DRM_PLANE_TYPE_OVERLAY:
		switch (format) {
		case DRM_FORMAT_NV12:
		case DRM_FORMAT_NV21:
		case DRM_FORMAT_P010:
			return modifier == DRM_FORMAT_MOD_VSI_G1_TILED ||
			       modifier == DRM_FORMAT_MOD_VSI_G2_TILED ||
			       modifier == DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED;
		default:
			return false;
		}
		break;
	default:
		return false;
	}
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
	.format_mod_supported = dcss_plane_format_mod_supported,
};

static int dcss_plane_atomic_check(struct drm_plane *plane,
				   struct drm_plane_state *state)
{
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *cma_obj;
	struct drm_crtc_state *crtc_state;
	int hdisplay, vdisplay;
	struct drm_rect crtc_rect, disp_rect;

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

	crtc_rect.x1 = state->crtc_x;
	crtc_rect.x2 = state->crtc_x + state->crtc_w;
	crtc_rect.y1 = state->crtc_y;
	crtc_rect.y2 = state->crtc_y + state->crtc_h;

	disp_rect.x1 = 0;
	disp_rect.y1 = 0;
	disp_rect.x2 = hdisplay;
	disp_rect.y2 = vdisplay;

	/* make sure the crtc is visible */
	if (!drm_rect_intersect(&crtc_rect, &disp_rect))
		return -EINVAL;

	/* cropping is only available on overlay planes when DTRC is used */
	if (state->crtc_x < 0 || state->crtc_y < 0 ||
	    state->crtc_x + state->crtc_w > hdisplay ||
	    state->crtc_y + state->crtc_h > vdisplay) {
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			return -EINVAL;
		else if (!(fb->flags & DRM_MODE_FB_MODIFIERS))
			return -EINVAL;
	}

	if (!dcss_scaler_can_scale(dcss_plane->dcss, dcss_plane->ch_num,
				   state->src_w >> 16, state->src_h >> 16,
				   state->crtc_w, state->crtc_h)) {
		DRM_DEBUG_KMS("Invalid upscale/downscale ratio.");
		return -EINVAL;
	}

	if ((fb->flags & DRM_MODE_FB_MODIFIERS) &&
	    !plane->funcs->format_mod_supported(plane,
				fb->format->format,
				fb->modifier)) {
		DRM_INFO("Invalid modifier: %llx", fb->modifier);
		return -EINVAL;
	}

	return 0;
}

static void dcss_plane_atomic_set_base(struct dcss_plane *dcss_plane)
{
	int mod_idx;
	struct drm_plane *plane = &dcss_plane->base;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	unsigned long p1_ba, p2_ba;
	dma_addr_t caddr;
	bool modifiers_present = !!(fb->flags & DRM_MODE_FB_MODIFIERS);
	u32 pix_format = state->fb->format->format;

	BUG_ON(!cma_obj);

	p1_ba = cma_obj->paddr + fb->offsets[0] +
		fb->pitches[0] * (state->src_y >> 16) +
		fb->format->cpp[0] * (state->src_x >> 16);

	p2_ba = cma_obj->paddr + fb->offsets[1] +
		fb->pitches[1] * (state->src_y >> 16) +
		fb->format->cpp[0] * (state->src_x >> 16);

	dcss_dpr_addr_set(dcss_plane->dcss, dcss_plane->ch_num, p1_ba, p2_ba,
			  fb->pitches[0]);

	switch (plane->type) {
	case DRM_PLANE_TYPE_PRIMARY:
		if (!modifiers_present) {
			/* No modifier: bypass dec400d */
			dcss_dec400d_bypass(dcss_plane->dcss);
			return;
		}

		for (mod_idx = 0; mod_idx < 4; mod_idx++)
			dcss_dec400d_set_format_mod(dcss_plane->dcss,
					pix_format,
					mod_idx,
					fb->modifier);

		switch (fb->modifier) {
		case DRM_FORMAT_MOD_LINEAR:
		case DRM_FORMAT_MOD_VIVANTE_TILED:
		case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
			/* Bypass dec400d */
			dcss_dec400d_bypass(dcss_plane->dcss);
			return;
		case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED_FC:
			caddr = cma_obj->paddr + ALIGN(fb->height, 64) * fb->pitches[0];
			dcss_dec400d_read_config(dcss_plane->dcss, 0, true);
			dcss_dec400d_addr_set(dcss_plane->dcss, p1_ba, caddr);
			break;
		default:
			WARN_ON(1);
			return;
		}

		break;
	case DRM_PLANE_TYPE_OVERLAY:
		if (!modifiers_present ||
		    (pix_format != DRM_FORMAT_NV12 &&
		     pix_format != DRM_FORMAT_NV21 &&
		     pix_format != DRM_FORMAT_P010)) {
			dcss_dtrc_bypass(dcss_plane->dcss, dcss_plane->ch_num);
			return;
		}

		dcss_dtrc_set_format_mod(dcss_plane->dcss, dcss_plane->ch_num,
					 pix_format, fb->modifier);
		dcss_dtrc_addr_set(dcss_plane->dcss, dcss_plane->ch_num,
				   p1_ba, p2_ba, dcss_plane->dtrc_table_ofs_val);
		break;
	default:
		WARN_ON(1);
		return;
	}
}

static bool dcss_plane_needs_setup(struct drm_plane_state *state,
				   struct drm_plane_state *old_state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_framebuffer *old_fb = old_state->fb;

	return state->crtc_x != old_state->crtc_x ||
	       state->crtc_y != old_state->crtc_y ||
	       state->crtc_w != old_state->crtc_w ||
	       state->crtc_h != old_state->crtc_h ||
	       state->src_x  != old_state->src_x  ||
	       state->src_y  != old_state->src_y  ||
	       state->src_w  != old_state->src_w  ||
	       state->src_h  != old_state->src_h  ||
	       fb->format->format != old_fb->format->format ||
	       fb->modifier  != old_fb->modifier;
}

static void dcss_plane_adjust(struct drm_rect *dis_rect,
			      struct drm_rect *crtc,
			      struct drm_rect *src)
{
	struct drm_rect new_crtc = *dis_rect, new_src;
	u32 hscale, vscale;

	hscale = ((src->x2 - src->x1) << 16) / (crtc->x2 - crtc->x1);
	vscale = ((src->y2 - src->y1) << 16) / (crtc->y2 - crtc->y1);

	drm_rect_intersect(&new_crtc, crtc);

	new_src.x1 = ((new_crtc.x1 - crtc->x1) * hscale + (1 << 15)) >> 16;
	new_src.x2 = ((new_crtc.x2 - crtc->x1) * hscale + (1 << 15)) >> 16;
	new_src.y1 = ((new_crtc.y1 - crtc->y1) * vscale + (1 << 15)) >> 16;
	new_src.y2 = ((new_crtc.y2 - crtc->y1) * vscale + (1 << 15)) >> 16;

	*crtc = new_crtc;
	*src = new_src;
}

static void dcss_plane_atomic_update(struct drm_plane *plane,
				     struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);
	struct drm_framebuffer *fb = state->fb;
	u32 pixel_format = state->fb->format->format;
	struct drm_crtc_state *crtc_state = state->crtc->state;
	bool modifiers_present = !!(fb->flags & DRM_MODE_FB_MODIFIERS);
	u32 src_w, src_h, adj_w, adj_h;
	struct drm_rect disp, crtc, src, old_src;
	u32 scaler_w, scaler_h;
	struct dcss_hdr10_pipe_cfg ipipe_cfg, opipe_cfg;

	if (!state->fb)
		return;

	if (old_state->fb && !drm_atomic_crtc_needs_modeset(crtc_state) &&
	    !dcss_plane_needs_setup(state, old_state) &&
	    !dcss_dtg_global_alpha_changed(dcss_plane->dcss, dcss_plane->ch_num,
					   pixel_format, dcss_plane->alpha_val,
					   dcss_plane->use_global_val)) {
		dcss_plane_atomic_set_base(dcss_plane);
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			dcss_dec400d_shadow_trig(dcss_plane->dcss);
		return;
	}

	dcss_dpr_enable(dcss_plane->dcss, dcss_plane->ch_num, false);
	dcss_scaler_enable(dcss_plane->dcss, dcss_plane->ch_num, false);
	dcss_dtg_ch_enable(dcss_plane->dcss, dcss_plane->ch_num, false);

	disp.x1 = 0;
	disp.y1 = 0;
	disp.x2 = crtc_state->adjusted_mode.hdisplay;
	disp.y2 = crtc_state->adjusted_mode.vdisplay;

	crtc.x1 = state->crtc_x;
	crtc.y1 = state->crtc_y;
	crtc.x2 = state->crtc_x + state->crtc_w;
	crtc.y2 = state->crtc_y + state->crtc_h;

	src.x1 = state->src_x >> 16;
	src.y1 = state->src_y >> 16;
	src.x2 = (state->src_x >> 16) + (state->src_w >> 16);
	src.y2 = (state->src_y >> 16) + (state->src_h >> 16);

	old_src = src;

	dcss_plane_adjust(&disp, &crtc, &src);

	/*
	 * The width and height after clipping, if image was partially
	 * outside the display area.
	 */
	src_w = src.x2 - src.x1;
	src_h = src.y2 - src.y1;

	if (plane->type == DRM_PLANE_TYPE_OVERLAY)
		dcss_dtrc_set_res(dcss_plane->dcss, dcss_plane->ch_num,
				  &src, &old_src);

	/* DTRC has probably aligned the sizes. */
	adj_w = src.x2 - src.x1;
	adj_h = src.y2 - src.y1;

	dcss_dpr_format_set(dcss_plane->dcss, dcss_plane->ch_num, pixel_format,
				modifiers_present);
	if (!modifiers_present)
		dcss_dpr_tile_derive(dcss_plane->dcss,
				     dcss_plane->ch_num,
				     DRM_FORMAT_MOD_LINEAR);
	else
		dcss_dpr_tile_derive(dcss_plane->dcss,
				     dcss_plane->ch_num,
				     fb->modifier);

	dcss_dpr_set_res(dcss_plane->dcss, dcss_plane->ch_num,
			 src_w, src_h, adj_w, adj_h);
	dcss_plane_atomic_set_base(dcss_plane);

	if (fb->modifier == DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED) {
		scaler_w = src.x1 ? adj_w : src_w;
		scaler_h = src.y1 ? adj_h : src_h;
	} else {
		scaler_w = src_w;
		scaler_h = src_h;
	}

	dcss_scaler_setup(dcss_plane->dcss, dcss_plane->ch_num,
			  pixel_format, scaler_w, scaler_h,
			  crtc.x2 - crtc.x1,
			  crtc.y2 - crtc.y1,
			  drm_mode_vrefresh(&crtc_state->mode));

	ipipe_cfg.pixel_format = pixel_format;
	ipipe_cfg.nl = NL_REC709;
	ipipe_cfg.pr = PR_LIMITED;
	ipipe_cfg.g = G_REC709;

	dcss_crtc_get_opipe_cfg(state->crtc, &opipe_cfg);

	/* apparently the other settins that are read from connector are not good,
	 * so hardcode */
	opipe_cfg.nl = NL_REC709;
	opipe_cfg.pr = PR_FULL;
	opipe_cfg.g = G_REC2020;

	dcss_hdr10_setup(dcss_plane->dcss, dcss_plane->ch_num,
			 &ipipe_cfg, &opipe_cfg);

	dcss_dtg_plane_pos_set(dcss_plane->dcss, dcss_plane->ch_num,
			       crtc.x1, crtc.y1,
			       crtc.x2 - crtc.x1,
			       crtc.y2 - crtc.y1);
	dcss_dtg_plane_alpha_set(dcss_plane->dcss, dcss_plane->ch_num,
				 pixel_format, dcss_plane->alpha_val,
				 dcss_plane->use_global_val);

	switch (plane->type) {
	case DRM_PLANE_TYPE_PRIMARY:
		dcss_dec400d_enable(dcss_plane->dcss);
		break;
	case DRM_PLANE_TYPE_OVERLAY:
		dcss_dtrc_enable(dcss_plane->dcss, dcss_plane->ch_num, true);
		break;
	default:
		WARN_ON(1);
		break;
	}
	dcss_dpr_enable(dcss_plane->dcss, dcss_plane->ch_num, true);
	dcss_scaler_enable(dcss_plane->dcss, dcss_plane->ch_num, true);
	dcss_dtg_ch_enable(dcss_plane->dcss, dcss_plane->ch_num, true);
}

static void dcss_plane_atomic_disable(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);

	dcss_dtrc_enable(dcss_plane->dcss, dcss_plane->ch_num, false);
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
	const u64 *format_modifiers = dcss_video_format_modifiers;
	int ret;

	if (zpos > 2)
		return ERR_PTR(-EINVAL);

	dcss_plane = kzalloc(sizeof(*dcss_plane), GFP_KERNEL);
	if (!dcss_plane) {
		DRM_ERROR("failed to allocate plane\n");
		return ERR_PTR(-ENOMEM);
	}

	dcss_plane->dcss = dcss;

	if (type == DRM_PLANE_TYPE_PRIMARY)
		format_modifiers = dcss_graphics_format_modifiers;

	ret = drm_universal_plane_init(drm, &dcss_plane->base, possible_crtcs,
				       &dcss_plane_funcs, dcss_common_formats,
				       ARRAY_SIZE(dcss_common_formats),
				       format_modifiers, type, NULL);
	if (ret) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(dcss_plane);
		return ERR_PTR(ret);
	}

	if (type == DRM_PLANE_TYPE_OVERLAY)
		dcss_plane->base.hdr_supported = true;

	drm_plane_helper_add(&dcss_plane->base, &dcss_plane_helper_funcs);

	ret = drm_plane_create_zpos_immutable_property(&dcss_plane->base, zpos);
	if (ret)
		return ERR_PTR(ret);

	dcss_plane->ch_num = 2 - zpos;
	dcss_plane->alpha_val = 255;

	return dcss_plane;
}
