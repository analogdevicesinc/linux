/*
 * Copyright 2017-2019 NXP
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
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_plane_helper.h>
#include <video/dpu.h>
#include "dpu-plane.h"
#include "imx-drm.h"

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

/*
 * RGB and packed/2planar YUV formats
 * are widely supported by many fetch units.
 */
static const uint32_t dpu_primary_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,

	DRM_FORMAT_YUYV,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

static const uint32_t dpu_overlay_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,

	DRM_FORMAT_YUYV,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

static void dpu_plane_destroy(struct drm_plane *plane)
{
	struct dpu_plane *dpu_plane = to_dpu_plane(plane);

	drm_plane_cleanup(plane);
	kfree(dpu_plane);
}

static void dpu_plane_reset(struct drm_plane *plane)
{
	struct dpu_plane_state *state;

	if (plane->state) {
		__drm_atomic_helper_plane_destroy_state(plane->state);
		kfree(to_dpu_plane_state(plane->state));
		plane->state = NULL;
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return;

	state->base.zpos = plane->type == DRM_PLANE_TYPE_PRIMARY ? 0 : 1;

	plane->state = &state->base;
	plane->state->plane = plane;
	plane->state->rotation = DRM_MODE_ROTATE_0;
}

static struct drm_plane_state *
dpu_drm_atomic_plane_duplicate_state(struct drm_plane *plane)
{
	struct dpu_plane_state *state, *copy;

	if (WARN_ON(!plane->state))
		return NULL;

	copy = kmalloc(sizeof(*state), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);
	state = to_dpu_plane_state(plane->state);
	copy->stage = state->stage;
	copy->source = state->source;
	copy->blend = state->blend;
	copy->layer_x = state->layer_x;
	copy->layer_y = state->layer_y;
	copy->base_x = state->base_x;
	copy->base_y = state->base_y;
	copy->base_w = state->base_w;
	copy->base_h = state->base_h;
	copy->is_top = state->is_top;

	return &copy->base;
}

static void dpu_drm_atomic_plane_destroy_state(struct drm_plane *plane,
					       struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_dpu_plane_state(state));
}

static const struct drm_plane_funcs dpu_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= dpu_plane_destroy,
	.reset		= dpu_plane_reset,
	.atomic_duplicate_state	= dpu_drm_atomic_plane_duplicate_state,
	.atomic_destroy_state	= dpu_drm_atomic_plane_destroy_state,
};

static inline dma_addr_t
drm_plane_state_to_baseaddr(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *dma_obj;
	unsigned int x = state->src.x1 >> 16;
	unsigned int y = state->src.y1 >> 16;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	BUG_ON(!dma_obj);

	if (fb->flags & DRM_MODE_FB_INTERLACED)
		y /= 2;

	return dma_obj->dma_addr + fb->offsets[0] + fb->pitches[0] * y +
	       fb->format->cpp[0] * x;
}

static inline dma_addr_t
drm_plane_state_to_uvbaseaddr(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *dma_obj;
	int x = state->src.x1 >> 16;
	int y = state->src.y1 >> 16;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 1);
	BUG_ON(!dma_obj);

	x /= fb->format->hsub;
	y /= fb->format->vsub;

	if (fb->flags & DRM_MODE_FB_INTERLACED)
		y /= 2;

	return dma_obj->dma_addr + fb->offsets[1] + fb->pitches[1] * y +
	       fb->format->cpp[1] * x;
}

static int dpu_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct dpu_plane *dplane = to_dpu_plane(plane);
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	struct dpu_plane_state *old_dpstate = to_dpu_plane_state(plane->state);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = state->fb;
	struct dpu_fetchunit *fu;
	dma_addr_t baseaddr, uv_baseaddr;
	u32 src_w, src_h, src_x, src_y;
	int min_scale, bpp, ret;
	bool fb_is_interlaced;

	/* pure software check */
	if (plane->type != DRM_PLANE_TYPE_PRIMARY)
		if (WARN_ON(dpstate->base_x || dpstate->base_y ||
			    dpstate->base_w || dpstate->base_h))
			return -EINVAL;

	/* ok to disable */
	if (!fb) {
		dpstate->stage = LB_PRIM_SEL__DISABLE;
		dpstate->source = LB_SEC_SEL__DISABLE;
		dpstate->blend = ID_NONE;
		dpstate->layer_x = 0;
		dpstate->layer_y = 0;
		dpstate->base_x = 0;
		dpstate->base_y = 0;
		dpstate->base_w = 0;
		dpstate->base_h = 0;
		dpstate->is_top = false;
		return 0;
	}

	if (!state->crtc) {
		DRM_DEBUG_KMS("[PLANE:%d:%s] has no CRTC in plane state\n",
				plane->base.id, plane->name);
		return -EINVAL;
	}

	src_w = drm_rect_width(&state->src) >> 16;
	src_h = drm_rect_height(&state->src) >> 16;
	src_x = state->src.x1 >> 16;
	src_y = state->src.y1 >> 16;

	fb_is_interlaced = !!(fb->flags & DRM_MODE_FB_INTERLACED);

	crtc_state =
		drm_atomic_get_existing_crtc_state(state->state, state->crtc);
	if (WARN_ON(!crtc_state))
		return -EINVAL;

	min_scale = dplane->grp->has_vproc ?
				FRAC_16_16(min(src_w, src_h), 8192) :
				DRM_PLANE_NO_SCALING;
	ret = drm_atomic_helper_check_plane_state(state, crtc_state,
						  min_scale,
						  DRM_PLANE_NO_SCALING,
						  true, false);
	if (ret) {
		DRM_DEBUG_KMS("[PLANE:%d:%s] failed to check plane state\n",
				plane->base.id, plane->name);
		return ret;
	}

	/* mode set is needed when base x/y is changed */
	if (plane->type == DRM_PLANE_TYPE_PRIMARY)
		if ((dpstate->base_x != old_dpstate->base_x) ||
		    (dpstate->base_y != old_dpstate->base_y))
			crtc_state->mode_changed = true;

	/* no off screen */
	if (state->dst.x1 < 0 || state->dst.y1 < 0 ||
	    (state->dst.x2 > crtc_state->adjusted_mode.hdisplay) ||
	    (state->dst.y2 > crtc_state->adjusted_mode.vdisplay)) {
		DRM_DEBUG_KMS("[PLANE:%d:%s] no off screen\n",
				plane->base.id, plane->name);
		return -EINVAL;
	}

	/* pixel/line count and position parameters check */
	if (fb->format->hsub == 2) {
		if ((src_w % 2) || (src_x % 2)) {
			DRM_DEBUG_KMS("[PLANE:%d:%s] bad uv width or xoffset\n",
					plane->base.id, plane->name);
			return -EINVAL;
		}
	}
	if (fb->format->vsub == 2) {
		if (src_h % (fb_is_interlaced ? 4 : 2)) {
			DRM_DEBUG_KMS("[PLANE:%d:%s] bad uv height\n",
					plane->base.id, plane->name);
			return -EINVAL;
		}
		if (src_y % (fb_is_interlaced ? 4 : 2)) {
			DRM_DEBUG_KMS("[PLANE:%d:%s] bad uv yoffset\n",
					plane->base.id, plane->name);
			return -EINVAL;
		}
	}

	fu = source_to_fu(res, dpstate->source);
	if (!fu) {
		DRM_DEBUG_KMS("[PLANE:%d:%s] cannot get fetch unit\n",
				plane->base.id, plane->name);
		return -EINVAL;
	}

	/* base address alignment check */
	baseaddr = drm_plane_state_to_baseaddr(state);
	switch (fb->format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		bpp = 8;
		break;
	default:
		bpp = fb->format->cpp[0] * 8;
		break;
	}
	if (((bpp == 32) && (baseaddr & 0x3)) ||
	    ((bpp == 16) && (baseaddr & 0x1))) {
		DRM_DEBUG_KMS("[PLANE:%d:%s] %dbpp fb bad baddr alignment\n",
				plane->base.id, plane->name, bpp);
		return -EINVAL;
	}

	if (fb->pitches[0] > 0x10000) {
		DRM_DEBUG_KMS("[PLANE:%d:%s] fb pitch[0] is too big\n",
				plane->base.id, plane->name);
		return -EINVAL;
	}

	/* UV base address alignment check, assuming 16bpp */
	if (fb->format->num_planes > 1) {
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state);
		if (uv_baseaddr & 0x1) {
			DRM_DEBUG_KMS("[PLANE:%d:%s] bad uv baddr alignment\n",
					plane->base.id, plane->name);
			return -EINVAL;
		}

		if (fb->pitches[1] > 0x10000) {
			DRM_DEBUG_KMS("[PLANE:%d:%s] fb pitch[1] is too big\n",
					plane->base.id, plane->name);
			return -EINVAL;
		}
	}

	return 0;
}

static void dpu_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *old_state)
{
	struct dpu_plane *dplane = to_dpu_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_fetchunit *fu;
	struct dpu_fetchunit *fe = NULL;
	struct dpu_hscaler *hs = NULL;
	struct dpu_vscaler *vs = NULL;
	struct dpu_layerblend *lb;
	struct dpu_constframe *cf;
	struct dpu_extdst *ed;
	struct dpu_framegen *fg;
	dma_addr_t baseaddr, uv_baseaddr;
	dpu_block_id_t fe_id, vs_id = ID_NONE, hs_id;
	lb_sec_sel_t source = dpstate->source;
	unsigned int src_w, src_h, dst_w, dst_h;
	int bpp, lb_id;
	bool need_fetcheco = false, need_hscaler = false, need_vscaler = false;
	bool need_modeset;
	bool fb_is_interlaced;

	/*
	 * Do nothing since the plane is disabled by
	 * crtc_func->atomic_begin/flush.
	 */
	if (!fb)
		return;

	need_modeset = drm_atomic_crtc_needs_modeset(state->crtc->state);
	fb_is_interlaced = !!(fb->flags & DRM_MODE_FB_INTERLACED);

	fg = res->fg[dplane->stream_id];

	fu = source_to_fu(res, source);
	if (!fu)
		return;

	lb_id = blend_to_id(dpstate->blend);
	if (lb_id < 0)
		return;

	lb = res->lb[lb_id];

	src_w = drm_rect_width(&state->src) >> 16;
	src_h = drm_rect_height(&state->src) >> 16;
	dst_w = drm_rect_width(&state->dst);
	dst_h = drm_rect_height(&state->dst);

	if (fetchunit_is_fetchdecode(fu)) {
		if (fetchdecode_need_fetcheco(fu, fb->format->format)) {
			need_fetcheco = true;
			fe = fetchdecode_get_fetcheco(fu);
			if (IS_ERR(fe))
				return;
		}

		if (src_w != dst_w) {
			need_hscaler = true;
			hs = fetchdecode_get_hscaler(fu);
			if (IS_ERR(hs))
				return;
		}

		if ((src_h != dst_h) || fb_is_interlaced) {
			need_vscaler = true;
			vs = fetchdecode_get_vscaler(fu);
			if (IS_ERR(vs))
				return;
		}
	}

	switch (fb->format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		bpp = 8;
		break;
	default:
		bpp = fb->format->cpp[0] * 8;
		break;
	}

	baseaddr = drm_plane_state_to_baseaddr(state);
	if (need_fetcheco)
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state);

	fu->ops->set_burstlength(fu);
	fu->ops->set_src_bpp(fu, bpp);
	fu->ops->set_src_stride(fu, fb->pitches[0]);
	fu->ops->set_src_buf_dimensions(fu, src_w, src_h, 0, fb_is_interlaced);
	fu->ops->set_fmt(fu, fb->format->format, fb_is_interlaced);
	fu->ops->enable_src_buf(fu);
	fu->ops->set_framedimensions(fu, src_w, src_h, fb_is_interlaced);
	fu->ops->set_baseaddress(fu, baseaddr);
	fu->ops->set_stream_id(fu, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

	DRM_DEBUG_KMS("[PLANE:%d:%s] %s-0x%02x\n",
				plane->base.id, plane->name, fu->name, fu->id);

	if (need_fetcheco) {
		fe_id = fetcheco_get_block_id(fe);
		if (fe_id == ID_NONE)
			return;

		fetchdecode_pixengcfg_dynamic_src_sel(fu,
						(fd_dynamic_src_sel_t)fe_id);
		fe->ops->set_burstlength(fe);
		fe->ops->set_src_bpp(fe, 16);
		fe->ops->set_src_stride(fe, fb->pitches[1]);
		fe->ops->set_fmt(fe, fb->format->format, fb_is_interlaced);
		fe->ops->set_src_buf_dimensions(fe, src_w, src_h,
						fb->format->format,
						fb_is_interlaced);
		fe->ops->set_framedimensions(fe, src_w, src_h,
						fb_is_interlaced);
		fe->ops->set_baseaddress(fe, uv_baseaddr);
		fe->ops->enable_src_buf(fe);
		fe->ops->set_stream_id(fe, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		DRM_DEBUG_KMS("[PLANE:%d:%s] %s-0x%02x\n",
				plane->base.id, plane->name, fe->name, fe_id);
	} else {
		if (fetchunit_is_fetchdecode(fu))
			fetchdecode_pixengcfg_dynamic_src_sel(fu,
								FD_SRC_DISABLE);
	}

	/* vscaler comes first */
	if (need_vscaler) {
		vs_id = vscaler_get_block_id(vs);
		if (vs_id == ID_NONE)
			return;

		vscaler_pixengcfg_dynamic_src_sel(vs, (vs_src_sel_t)source);
		vscaler_pixengcfg_clken(vs, CLKEN__AUTOMATIC);
		vscaler_setup1(vs, src_h, state->crtc_h, fb_is_interlaced);
		vscaler_setup2(vs, fb_is_interlaced);
		vscaler_setup3(vs, fb_is_interlaced);
		vscaler_output_size(vs, dst_h);
		vscaler_field_mode(vs, fb_is_interlaced ?
						SCALER_ALWAYS0 : SCALER_INPUT);
		vscaler_filter_mode(vs, SCALER_LINEAR);
		vscaler_scale_mode(vs, SCALER_UPSCALE);
		vscaler_mode(vs, SCALER_ACTIVE);
		vscaler_set_stream_id(vs, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		source = (lb_sec_sel_t)vs_id;

		DRM_DEBUG_KMS("[PLANE:%d:%s] vscaler-0x%02x\n",
					plane->base.id, plane->name, vs_id);
	}

	/* and then, hscaler */
	if (need_hscaler) {
		hs_id = hscaler_get_block_id(hs);
		if (hs_id == ID_NONE)
			return;

		hscaler_pixengcfg_dynamic_src_sel(hs, need_vscaler ?
							(hs_src_sel_t)vs_id :
							(hs_src_sel_t)source);
		hscaler_pixengcfg_clken(hs, CLKEN__AUTOMATIC);
		hscaler_setup1(hs, src_w, dst_w);
		hscaler_output_size(hs, dst_w);
		hscaler_filter_mode(hs, SCALER_LINEAR);
		hscaler_scale_mode(hs, SCALER_UPSCALE);
		hscaler_mode(hs, SCALER_ACTIVE);
		hscaler_set_stream_id(hs, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		source = (lb_sec_sel_t)hs_id;

		DRM_DEBUG_KMS("[PLANE:%d:%s] hscaler-0x%02x\n",
					plane->base.id, plane->name, hs_id);
	}

	layerblend_pixengcfg_dynamic_prim_sel(lb, dpstate->stage);
	layerblend_pixengcfg_dynamic_sec_sel(lb, source);
	layerblend_control(lb, LB_BLEND);
	layerblend_blendcontrol(lb, need_hscaler || need_vscaler);
	layerblend_pixengcfg_clken(lb, CLKEN__AUTOMATIC);
	layerblend_position(lb, dpstate->layer_x, dpstate->layer_y);

	if (plane->type == DRM_PLANE_TYPE_PRIMARY) {
		cf = res->cf[dplane->stream_id];
		constframe_framedimensions(cf,
					dpstate->base_w, dpstate->base_h);
		constframe_constantcolor(cf, 0, 0, 0, 0);

		framegen_sacfg(fg, dpstate->base_x, dpstate->base_y);
	}

	if (dpstate->is_top) {
		ed = res->ed[dplane->stream_id];
		extdst_pixengcfg_src_sel(ed, (extdst_src_sel_t)dpstate->blend);
	}

	DRM_DEBUG_KMS("[PLANE:%d:%s] source-0x%02x stage-0x%02x blend-0x%02x\n",
			plane->base.id, plane->name,
			source, dpstate->stage, dpstate->blend);
}

static const struct drm_plane_helper_funcs dpu_plane_helper_funcs = {
	.prepare_fb = drm_gem_fb_prepare_fb,
	.atomic_check = dpu_plane_atomic_check,
	.atomic_update = dpu_plane_atomic_update,
};

struct dpu_plane *dpu_plane_create(struct drm_device *drm,
				   unsigned int possible_crtcs,
				   unsigned int stream_id,
				   struct dpu_plane_grp *grp,
				   enum drm_plane_type type)
{
	struct dpu_plane *dpu_plane;
	struct drm_plane *plane;
	const uint32_t *formats;
	unsigned int format_count, ov_num;
	int ret;

	dpu_plane = kzalloc(sizeof(*dpu_plane), GFP_KERNEL);
	if (!dpu_plane)
		return ERR_PTR(-ENOMEM);

	dpu_plane->stream_id = stream_id;
	dpu_plane->grp = grp;

	plane = &dpu_plane->base;

	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		formats = dpu_primary_formats;
		format_count = ARRAY_SIZE(dpu_primary_formats);
		break;
	case DRM_PLANE_TYPE_OVERLAY:
		formats = dpu_overlay_formats;
		format_count = ARRAY_SIZE(dpu_overlay_formats);
		break;
	default:
		kfree(dpu_plane);
		return ERR_PTR(-EINVAL);
	}

	ret = drm_universal_plane_init(drm, plane, possible_crtcs,
				       &dpu_plane_funcs, formats, format_count,
				       NULL, type, NULL);
	if (ret) {
		kfree(dpu_plane);
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(plane, &dpu_plane_helper_funcs);

	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		ret = drm_plane_create_zpos_immutable_property(plane, 0);
		break;
	case DRM_PLANE_TYPE_OVERLAY:
		/* filter out the primary plane */
		ov_num = grp->hw_plane_num - 1;

		ret = drm_plane_create_zpos_property(plane, 1, 1, ov_num);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret) {
		kfree(dpu_plane);
		return ERR_PTR(ret);
	}

	return dpu_plane;
}
