/*
 * Copyright 2017-2018 NXP
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
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <video/dpu.h>
#include <video/imx8-prefetch.h>
#include "dpu-plane.h"
#include "imx-drm.h"

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

static const uint64_t dpu_format_modifiers[] = {
	DRM_FORMAT_MOD_VIVANTE_TILED,
	DRM_FORMAT_MOD_VIVANTE_SUPER_TILED,
	DRM_FORMAT_MOD_AMPHION_TILED,
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
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
	copy->use_prefetch = state->use_prefetch;

	return &copy->base;
}

static void dpu_drm_atomic_plane_destroy_state(struct drm_plane *plane,
					       struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_dpu_plane_state(state));
}

static bool dpu_drm_plane_format_mod_supported(struct drm_plane *plane,
					       uint32_t format,
					       uint64_t modifier)
{
	if (WARN_ON(modifier == DRM_FORMAT_MOD_INVALID))
		return false;

	switch (format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		return modifier == DRM_FORMAT_MOD_LINEAR;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_RGB565:
		return modifier == DRM_FORMAT_MOD_LINEAR ||
		       modifier == DRM_FORMAT_MOD_VIVANTE_TILED ||
		       modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		return modifier == DRM_FORMAT_MOD_LINEAR ||
		       modifier == DRM_FORMAT_MOD_AMPHION_TILED;
	default:
		return false;
	}
}

static const struct drm_plane_funcs dpu_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= dpu_plane_destroy,
	.reset		= dpu_plane_reset,
	.atomic_duplicate_state	= dpu_drm_atomic_plane_duplicate_state,
	.atomic_destroy_state	= dpu_drm_atomic_plane_destroy_state,
	.format_mod_supported	= dpu_drm_plane_format_mod_supported,
};

static inline dma_addr_t
drm_plane_state_to_baseaddr(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *cma_obj;
	unsigned int x = state->src_x >> 16;
	unsigned int y = state->src_y >> 16;

	cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	BUG_ON(!cma_obj);

	if (fb->modifier)
		return cma_obj->paddr + fb->offsets[0];

	if (fb->flags & DRM_MODE_FB_INTERLACED)
		y /= 2;

	return cma_obj->paddr + fb->offsets[0] + fb->pitches[0] * y +
	       drm_format_plane_cpp(fb->format->format, 0) * x;
}

static inline dma_addr_t
drm_plane_state_to_uvbaseaddr(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *cma_obj;
	int x = state->src_x >> 16;
	int y = state->src_y >> 16;

	cma_obj = drm_fb_cma_get_gem_obj(fb, 1);
	BUG_ON(!cma_obj);

	if (fb->modifier)
		return cma_obj->paddr + fb->offsets[1];

	x /= drm_format_horz_chroma_subsampling(fb->format->format);
	y /= drm_format_vert_chroma_subsampling(fb->format->format);

	if (fb->flags & DRM_MODE_FB_INTERLACED)
		y /= 2;

	return cma_obj->paddr + fb->offsets[1] + fb->pitches[1] * y +
	       drm_format_plane_cpp(fb->format->format, 1) * x;
}

static int dpu_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct dpu_plane *dplane = to_dpu_plane(plane);
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = state->fb;
	struct dpu_fetchunit *fu;
	struct dprc *dprc;
	dma_addr_t baseaddr, uv_baseaddr = 0;
	u32 src_w = state->src_w >> 16, src_h = state->src_h >> 16,
	    src_x = state->src_x >> 16, src_y = state->src_y >> 16;
	int bpp;
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
		dpstate->use_prefetch = false;
		return 0;
	}

	if (!state->crtc)
		return -EINVAL;

	fu = dpstate_to_fu(dpstate);
	if (!fu)
		return -EINVAL;

	dprc = fu->dprc;

	fb_is_interlaced = !!(fb->flags & DRM_MODE_FB_INTERLACED);

	if (fb->modifier &&
	    fb->modifier != DRM_FORMAT_MOD_AMPHION_TILED &&
	    fb->modifier != DRM_FORMAT_MOD_VIVANTE_TILED &&
	    fb->modifier != DRM_FORMAT_MOD_VIVANTE_SUPER_TILED)
		return -EINVAL;

	if (fb->modifier && (src_x || src_y) &&
	    !dplane->has_prefetch_fixup)
		return -EINVAL;

	if (dplane->grp->has_vproc) {
		/* no down scaling */
		if (src_w > state->crtc_w || src_h > state->crtc_h)
			return -EINVAL;
	} else {
		/* no scaling */
		if (src_w != state->crtc_w || src_h != state->crtc_h)
			return -EINVAL;
	}

	/* no off screen */
	if (state->crtc_x < 0 || state->crtc_y < 0)
		return -EINVAL;

	crtc_state =
		drm_atomic_get_existing_crtc_state(state->state, state->crtc);
	if (WARN_ON(!crtc_state))
		return -EINVAL;

	if (state->crtc_x + state->crtc_w >
	    crtc_state->adjusted_mode.hdisplay)
		return -EINVAL;
	if (state->crtc_y + state->crtc_h >
	    crtc_state->adjusted_mode.vdisplay)
		return -EINVAL;

	/* pixel/line count and position parameters check */
	if (drm_format_horz_chroma_subsampling(fb->format->format) == 2 &&
	    ((src_w % 2) || (src_x % 2)))
		return -EINVAL;
	if (drm_format_vert_chroma_subsampling(fb->format->format) == 2) {
		if (src_h % (fb_is_interlaced ? 4 : 2))
			return -EINVAL;
		if (src_y % (fb_is_interlaced ? 4 : 2))
			return -EINVAL;
	}

	/* for tile formats, framebuffer has to be tile aligned */
	switch (fb->modifier) {
	case DRM_FORMAT_MOD_AMPHION_TILED:
		if (fb->width % 8)
			return -EINVAL;
		if (fb->height % 256)
			return -EINVAL;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
		if (fb->width % 4)
			return -EINVAL;
		if (fb->height % 4)
			return -EINVAL;
		break;
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		if (fb->width % 64)
			return -EINVAL;
		if (fb->height % 64)
			return -EINVAL;
		break;
	default:
		break;
	}

	if (dprc &&
	    dprc_format_supported(dprc, fb->format->format, fb->modifier) &&
	    dprc_stride_supported(dprc, fb->pitches[0], fb->pitches[1],
					src_w, fb->format->format))
		dpstate->use_prefetch = true;
	else
		dpstate->use_prefetch = false;

	if (fb->modifier && !dpstate->use_prefetch)
		return -EINVAL;

	/*
	 * base address alignment check
	 *
	 * The (uv) base address offset introduced by PRG x/y
	 * offset(for tile formats) would not impact the alignment
	 * check, so we don't take the offset into consideration.
	 */
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
		bpp = drm_format_plane_cpp(fb->format->format, 0) * 8;
		break;
	}
	switch (bpp) {
	case 32:
		if (baseaddr & 0x3)
			return -EINVAL;
		break;
	case 16:
		if (fb->modifier) {
			if (baseaddr & 0x1)
				return -EINVAL;
		} else {
			if (baseaddr & (dpstate->use_prefetch ? 0x7 : 0x1))
				return -EINVAL;
		}
		break;
	}

	if (fb->pitches[0] > 0x10000)
		return -EINVAL;

	/* UV base address alignment check, assuming 16bpp */
	if (drm_format_num_planes(fb->format->format) > 1) {
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state);
		if (fb->modifier) {
			if (uv_baseaddr & 0x1)
				return -EINVAL;
		} else {
			if (uv_baseaddr & (dpstate->use_prefetch ? 0x7 : 0x1))
				return -EINVAL;
		}

		if (fb->pitches[1] > 0x10000)
			return -EINVAL;
	}

	if (dpstate->use_prefetch &&
	    !dprc_stride_double_check(dprc, src_w, src_x, fb->format->format,
					fb->modifier,
					baseaddr, uv_baseaddr)) {
		if (fb->modifier)
			return -EINVAL;

		if (bpp == 16 && (baseaddr & 0x1))
			return -EINVAL;

		if (uv_baseaddr & 0x1)
			return -EINVAL;

		dpstate->use_prefetch = false;
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
	struct dprc *dprc;
	struct dpu_hscaler *hs = NULL;
	struct dpu_vscaler *vs = NULL;
	struct dpu_layerblend *lb;
	struct dpu_constframe *cf;
	struct dpu_extdst *ed;
	struct dpu_framegen *fg = res->fg[dplane->stream_id];
	struct device *dev = plane->dev->dev;
	dma_addr_t baseaddr, uv_baseaddr = 0;
	dpu_block_id_t fe_id, vs_id = ID_NONE, hs_id;
	lb_sec_sel_t lb_src = dpstate->source;
	unsigned int src_w, src_h, src_x, src_y;
	unsigned int mt_w = 0, mt_h = 0;	/* w/h in a micro-tile */
	int bpp, lb_id;
	bool need_fetcheco = false, need_hscaler = false, need_vscaler = false;
	bool prefetch_start = false, aux_prefetch_start = false;
	bool need_modeset;
	bool is_overlay = plane->type == DRM_PLANE_TYPE_OVERLAY;
	bool fb_is_interlaced;

	/*
	 * Do nothing since the plane is disabled by
	 * crtc_func->atomic_begin/flush.
	 */
	if (!fb)
		return;

	need_modeset = drm_atomic_crtc_needs_modeset(state->crtc->state);
	fb_is_interlaced = !!(fb->flags & DRM_MODE_FB_INTERLACED);

	fu = dpstate_to_fu(dpstate);
	if (!fu)
		return;

	dprc = fu->dprc;

	lb_id = blend_to_id(dpstate->blend);
	if (lb_id < 0)
		return;

	lb = res->lb[lb_id];

	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;
	src_x = state->src_x >> 16;
	src_y = state->src_y >> 16;

	if (fetchunit_is_fetchdecode(fu)) {
		if (fetchdecode_need_fetcheco(fu, fb->format->format)) {
			need_fetcheco = true;
			fe = fetchdecode_get_fetcheco(fu);
			if (IS_ERR(fe))
				return;
		}

		if (src_w != state->crtc_w) {
			need_hscaler = true;
			hs = fetchdecode_get_hscaler(fu);
			if (IS_ERR(hs))
				return;
		}

		if ((src_h != state->crtc_h) || fb_is_interlaced) {
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
		bpp = drm_format_plane_cpp(fb->format->format, 0) * 8;
		break;
	}

	switch (fb->modifier) {
	case DRM_FORMAT_MOD_AMPHION_TILED:
		mt_w = 8;
		mt_h = 8;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		mt_w = (bpp == 16) ? 8 : 4;
		mt_h = 4;
		break;
	default:
		break;
	}

	baseaddr = drm_plane_state_to_baseaddr(state);
	if (need_fetcheco)
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state);

	if (dpstate->use_prefetch &&
	    (fu->ops->get_stream_id(fu) == DPU_PLANE_SRC_DISABLED ||
	     need_modeset))
		prefetch_start = true;

	fu->ops->set_burstlength(fu, src_x, mt_w, bpp,
				 baseaddr, dpstate->use_prefetch);
	fu->ops->set_src_bpp(fu, bpp);
	fu->ops->set_src_stride(fu, src_w, src_x, mt_w, bpp, fb->pitches[0],
				baseaddr, dpstate->use_prefetch);
	fu->ops->set_src_buf_dimensions(fu, src_w, src_h, 0, fb_is_interlaced);
	fu->ops->set_fmt(fu, fb->format->format, fb_is_interlaced);
	fu->ops->enable_src_buf(fu);
	fu->ops->set_framedimensions(fu, src_w, src_h, fb_is_interlaced);
	fu->ops->set_baseaddress(fu, src_w, src_x, src_y, mt_w, mt_h, bpp,
				 baseaddr);
	fu->ops->set_stream_id(fu, dplane->stream_id ?
						DPU_PLANE_SRC_TO_DISP_STREAM1 :
						DPU_PLANE_SRC_TO_DISP_STREAM0);
	fu->ops->unpin_off(fu);

	dev_dbg(dev, "[PLANE:%d:%s] %s-0x%02x\n",
				plane->base.id, plane->name, fu->name, fu->id);

	if (need_fetcheco) {
		fe_id = fetcheco_get_block_id(fe);
		if (fe_id == ID_NONE)
			return;

		if (dpstate->use_prefetch &&
		    (fe->ops->get_stream_id(fe) == DPU_PLANE_SRC_DISABLED ||
		     need_modeset))
			aux_prefetch_start = true;

		fetchdecode_pixengcfg_dynamic_src_sel(fu,
						(fd_dynamic_src_sel_t)fe_id);
		fe->ops->set_burstlength(fe, src_x, mt_w, bpp, uv_baseaddr,
					 dpstate->use_prefetch);
		fe->ops->set_src_bpp(fe, 16);
		fe->ops->set_src_stride(fe, src_w, src_x, mt_w, bpp,
					fb->pitches[1],
					uv_baseaddr, dpstate->use_prefetch);
		fe->ops->set_fmt(fe, fb->format->format, fb_is_interlaced);
		fe->ops->set_src_buf_dimensions(fe, src_w, src_h,
						fb->format->format,
						fb_is_interlaced);
		fe->ops->set_framedimensions(fe, src_w, src_h,
						fb_is_interlaced);
		fe->ops->set_baseaddress(fe, src_w, src_x, src_y / 2,
					 mt_w, mt_h, bpp, uv_baseaddr);
		fe->ops->enable_src_buf(fe);
		fe->ops->set_stream_id(fe, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);
		fe->ops->unpin_off(fe);

		dev_dbg(dev, "[PLANE:%d:%s] %s-0x%02x\n",
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

		vscaler_pixengcfg_dynamic_src_sel(vs,
					(vs_src_sel_t)(dpstate->source));
		vscaler_pixengcfg_clken(vs, CLKEN__AUTOMATIC);
		vscaler_setup1(vs, src_h, state->crtc_h, fb_is_interlaced);
		vscaler_setup2(vs, fb_is_interlaced);
		vscaler_setup3(vs, fb_is_interlaced);
		vscaler_output_size(vs, state->crtc_h);
		vscaler_field_mode(vs, fb_is_interlaced ?
						SCALER_ALWAYS0 : SCALER_INPUT);
		vscaler_filter_mode(vs, SCALER_LINEAR);
		vscaler_scale_mode(vs, SCALER_UPSCALE);
		vscaler_mode(vs, SCALER_ACTIVE);
		vscaler_set_stream_id(vs, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		lb_src = (lb_sec_sel_t)vs_id;

		dev_dbg(dev, "[PLANE:%d:%s] vscaler-0x%02x\n",
					plane->base.id, plane->name, vs_id);
	}

	/* and then, hscaler */
	if (need_hscaler) {
		hs_id = hscaler_get_block_id(hs);
		if (hs_id == ID_NONE)
			return;

		hscaler_pixengcfg_dynamic_src_sel(hs, need_vscaler ?
					(hs_src_sel_t)(vs_id) :
					(hs_src_sel_t)(dpstate->source));
		hscaler_pixengcfg_clken(hs, CLKEN__AUTOMATIC);
		hscaler_setup1(hs, src_w, state->crtc_w);
		hscaler_output_size(hs, state->crtc_w);
		hscaler_filter_mode(hs, SCALER_LINEAR);
		hscaler_scale_mode(hs, SCALER_UPSCALE);
		hscaler_mode(hs, SCALER_ACTIVE);
		hscaler_set_stream_id(hs, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		lb_src = (lb_sec_sel_t)hs_id;

		dev_dbg(dev, "[PLANE:%d:%s] hscaler-0x%02x\n",
					plane->base.id, plane->name, hs_id);
	}

	if (dpstate->use_prefetch) {
		dprc_configure(dprc, dplane->stream_id,
			       src_w, src_h, src_x, src_y,
			       fb->pitches[0], fb->format->format,
			       fb->modifier, baseaddr, uv_baseaddr,
			       prefetch_start, aux_prefetch_start,
			       fb_is_interlaced);
		if (prefetch_start || aux_prefetch_start)
			dprc_enable(dprc);

		dprc_reg_update(dprc);

		if (prefetch_start || aux_prefetch_start) {
			dprc_first_frame_handle(dprc);

			if (!need_modeset && is_overlay)
				framegen_wait_for_frame_counter_moving(fg);
		}

		dev_dbg(dev, "[PLANE:%d:%s] use prefetch\n",
					plane->base.id, plane->name);
	} else if (dprc) {
		dprc_disable(dprc);

		dev_dbg(dev, "[PLANE:%d:%s] bypass prefetch\n",
					plane->base.id, plane->name);
	}

	layerblend_pixengcfg_dynamic_prim_sel(lb, dpstate->stage);
	layerblend_pixengcfg_dynamic_sec_sel(lb, lb_src);
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

	dev_dbg(dev, "[PLANE:%d:%s] source-0x%02x stage-0x%02x blend-0x%02x\n",
			plane->base.id, plane->name,
			dpstate->source, dpstate->stage, dpstate->blend);
}

static const struct drm_plane_helper_funcs dpu_plane_helper_funcs = {
	.atomic_check = dpu_plane_atomic_check,
	.atomic_update = dpu_plane_atomic_update,
};

struct dpu_plane *dpu_plane_init(struct drm_device *drm,
				 unsigned int possible_crtcs,
				 unsigned int stream_id,
				 struct dpu_plane_grp *grp,
				 enum drm_plane_type type,
				 bool has_prefetch_fixup)
{
	struct dpu_plane *dpu_plane;
	struct drm_plane *plane;
	unsigned int ov_num;
	int ret;

	dpu_plane = kzalloc(sizeof(*dpu_plane), GFP_KERNEL);
	if (!dpu_plane)
		return ERR_PTR(-ENOMEM);

	dpu_plane->stream_id = stream_id;
	dpu_plane->grp = grp;
	dpu_plane->has_prefetch_fixup = has_prefetch_fixup;

	plane = &dpu_plane->base;

	if (type == DRM_PLANE_TYPE_PRIMARY)
		ret = drm_universal_plane_init(drm, plane, possible_crtcs,
					       &dpu_plane_funcs,
					       dpu_primary_formats,
					       ARRAY_SIZE(dpu_primary_formats),
					       dpu_format_modifiers,
					       type, NULL);
	else
		ret = drm_universal_plane_init(drm, plane, possible_crtcs,
					       &dpu_plane_funcs,
					       dpu_overlay_formats,
					       ARRAY_SIZE(dpu_overlay_formats),
					       dpu_format_modifiers,
					       type, NULL);
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

	if (ret)
		return ERR_PTR(ret);

	return dpu_plane;
}
