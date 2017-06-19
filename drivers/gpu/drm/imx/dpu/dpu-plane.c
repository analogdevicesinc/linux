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
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <video/dpu.h>
#include "dpu-plane.h"
#include "imx-drm.h"

/*
 * RGB and packed/2planar YUV formats
 * are widely supported by many fetch units.
 */
static const uint32_t dpu_common_formats[] = {
	/* DRM_FORMAT_ARGB8888, */
	DRM_FORMAT_XRGB8888,
	/* DRM_FORMAT_ABGR8888, */
	DRM_FORMAT_XBGR8888,
	/* DRM_FORMAT_RGBA8888, */
	DRM_FORMAT_RGBX8888,
	/* DRM_FORMAT_BGRA8888, */
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,

	DRM_FORMAT_YUYV,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV61,
	DRM_FORMAT_NV24,
	DRM_FORMAT_NV42,
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
	struct drm_gem_cma_object *cma_obj;
	unsigned int x = state->src_x >> 16;
	unsigned int y = state->src_y >> 16;

	cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	BUG_ON(!cma_obj);

	/* round down x/y to be tile aligned at left/top */
	switch (fb->modifier) {
	case DRM_FORMAT_MOD_NONE:
		break;
	case DRM_FORMAT_MOD_AMPHION_TILED:
		x = round_down(x, 8);
		y = round_down(y, 256);
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
		x = round_down(x, 4);
		y = round_down(y, 4);
		break;
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		x = round_down(x, 64);
		y = round_down(y, 64);
		break;
	default:
		WARN_ON(1);
		break;
	}

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

	x /= drm_format_horz_chroma_subsampling(fb->format->format);
	y /= drm_format_vert_chroma_subsampling(fb->format->format);

	/* round down x/y to be tile aligned at left/top */
	if (fb->modifier == DRM_FORMAT_MOD_AMPHION_TILED) {
		x = round_down(x, 8);
		y = round_down(y, 128);
	}

	return cma_obj->paddr + fb->offsets[1] + fb->pitches[1] * y +
	       drm_format_plane_cpp(fb->format->format, 1) * x;
}

static int dpu_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct dpu_plane *dplane = to_dpu_plane(plane);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = state->fb;
	struct dpu_fetchdecode *fd;
	dma_addr_t baseaddr, uv_baseaddr = 0;
	u32 src_w = state->src_w >> 16, src_h = state->src_h >> 16,
	    src_x = state->src_x >> 16, src_y = state->src_y >> 16;
	int bpp, fd_id;

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

	if (fb->modifier &&
	    fb->modifier != DRM_FORMAT_MOD_AMPHION_TILED &&
	    fb->modifier != DRM_FORMAT_MOD_VIVANTE_TILED &&
	    fb->modifier != DRM_FORMAT_MOD_VIVANTE_SUPER_TILED)
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
	if (drm_format_vert_chroma_subsampling(fb->format->format) == 2 &&
	    ((src_h % 2) || (src_y % 2)))
		return -EINVAL;

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

	fd_id = source_to_id(dpstate->source);
	if (fd_id < 0)
		return -EINVAL;

	fd = res->fd[fd_id];

	if (fetchdecode_has_prefetch(fd) &&
	    fetchdecode_prefetch_format_supported(fd, fb->format->format,
						  fb->modifier) &&
	    fetchdecode_prefetch_stride_supported(fd, fb->pitches[0],
						  fb->pitches[1],
						  src_w,
						  fb->format->format) &&
	    fetchdecode_prefetch_crop_supported(fd, fb->modifier, src_y))
		dpstate->use_prefetch = true;
	else
		dpstate->use_prefetch = false;

	if (fb->modifier && !dpstate->use_prefetch)
		return -EINVAL;

	/* base address alignment check */
	baseaddr = drm_plane_state_to_baseaddr(state);
	switch (fb->format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
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
		if (baseaddr & (dpstate->use_prefetch ? 0x7 : 0x1))
			return -EINVAL;
		break;
	}

	if (fb->pitches[0] > 0x10000)
		return -EINVAL;

	/* UV base address alignment check, assuming 16bpp */
	if (drm_format_num_planes(fb->format->format) > 1) {
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state);
		if (uv_baseaddr & (dpstate->use_prefetch ? 0x7 : 0x1))
			return -EINVAL;

		if (fb->pitches[1] > 0x10000)
			return -EINVAL;
	}

	if (dpstate->use_prefetch &&
	    !fetchdecode_prefetch_stride_double_check(fd, fb->pitches[0],
						      fb->pitches[1],
						      src_w, fb->format->format,
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
	struct dpu_fetchdecode *fd;
	struct dpu_fetcheco *fe = NULL;
	struct dpu_hscaler *hs = NULL;
	struct dpu_vscaler *vs = NULL;
	struct dpu_layerblend *lb;
	struct dpu_constframe *cf;
	struct dpu_extdst *ed;
	struct device *dev = plane->dev->dev;
	dma_addr_t baseaddr, uv_baseaddr = 0;
	dpu_block_id_t fe_id, vs_id = ID_NONE, hs_id;
	lb_sec_sel_t lb_src = dpstate->source;
	unsigned int src_w, src_h, src_x, src_y;
	int bpp, fd_id, lb_id;
	bool need_fetcheco = false, need_hscaler = false, need_vscaler = false;
	bool prefetch_start = false, aux_prefetch_start = false;

	/*
	 * Do nothing since the plane is disabled by
	 * crtc_func->atomic_begin/flush.
	 */
	if (!fb)
		return;

	fd_id = source_to_id(dpstate->source);
	if (fd_id < 0)
		return;

	lb_id = blend_to_id(dpstate->blend);
	if (lb_id < 0)
		return;

	fd = res->fd[fd_id];
	lb = res->lb[lb_id];

	src_w = state->src_w >> 16;
	src_h = state->src_h >> 16;
	src_x = state->src_x >> 16;
	src_y = state->src_y >> 16;

	if (fetchdecode_need_fetcheco(fd, fb->format->format)) {
		need_fetcheco = true;
		fe = fetchdecode_get_fetcheco(fd);
		if (IS_ERR(fe))
			return;
	}

	if (src_w != state->crtc_w) {
		need_hscaler = true;
		hs = fetchdecode_get_hscaler(fd);
		if (IS_ERR(hs))
			return;
	}

	if (src_h != state->crtc_h) {
		need_vscaler = true;
		vs = fetchdecode_get_vscaler(fd);
		if (IS_ERR(vs))
			return;
	}

	switch (fb->format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		bpp = 8;
		break;
	default:
		bpp = drm_format_plane_cpp(fb->format->format, 0) * 8;
		break;
	}

	baseaddr = drm_plane_state_to_baseaddr(state);
	if (need_fetcheco)
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state);

	if (dpstate->use_prefetch &&
	    (fetchdecode_get_stream_id(fd) == DPU_PLANE_SRC_DISABLED ||
	     drm_atomic_crtc_needs_modeset(state->crtc->state)))
		prefetch_start = true;

	fetchdecode_set_burstlength(fd, baseaddr, dpstate->use_prefetch);
	fetchdecode_source_bpp(fd, bpp);
	fetchdecode_source_stride(fd, src_w, bpp, fb->pitches[0], baseaddr,
							dpstate->use_prefetch);
	fetchdecode_src_buf_dimensions(fd, src_w, src_h);
	fetchdecode_set_fmt(fd, fb->format->format);
	fetchdecode_source_buffer_enable(fd);
	fetchdecode_framedimensions(fd, src_w, src_h);
	fetchdecode_baseaddress(fd, baseaddr);
	fetchdecode_set_stream_id(fd, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);
	fetchdecode_unpin_off(fd);

	if (need_fetcheco) {
		fe_id = fetcheco_get_block_id(fe);
		if (fe_id == ID_NONE)
			return;

		if (dpstate->use_prefetch &&
		    (fetcheco_get_stream_id(fe) == DPU_PLANE_SRC_DISABLED ||
		     drm_atomic_crtc_needs_modeset(state->crtc->state)))
			aux_prefetch_start = true;

		fetchdecode_pixengcfg_dynamic_src_sel(fd,
						(fd_dynamic_src_sel_t)fe_id);
		fetcheco_set_burstlength(fe, uv_baseaddr,
						dpstate->use_prefetch);
		fetcheco_source_bpp(fe, 16);
		fetcheco_source_stride(fe, src_w, bpp, fb->pitches[1],
					uv_baseaddr, dpstate->use_prefetch);
		fetcheco_set_fmt(fe, fb->format->format);
		fetcheco_src_buf_dimensions(fe, src_w, src_h, fb->format->format);
		fetcheco_framedimensions(fe, src_w, src_h);
		fetcheco_baseaddress(fe, uv_baseaddr);
		fetcheco_source_buffer_enable(fe);
		fetcheco_set_stream_id(fe, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);
		fetcheco_unpin_off(fe);

		dev_dbg(dev, "[PLANE:%d:%s] fetcheco-0x%02x\n",
					plane->base.id, plane->name, fe_id);
	} else {
		fetchdecode_pixengcfg_dynamic_src_sel(fd, FD_SRC_DISABLE);
	}

	/* vscaler comes first */
	if (need_vscaler) {
		vs_id = vscaler_get_block_id(vs);
		if (vs_id == ID_NONE)
			return;

		vscaler_pixengcfg_dynamic_src_sel(vs,
					(vs_src_sel_t)(dpstate->source));
		vscaler_pixengcfg_clken(vs, CLKEN__AUTOMATIC);
		vscaler_setup1(vs, src_h, state->crtc_h);
		vscaler_output_size(vs, state->crtc_h);
		vscaler_field_mode(vs, SCALER_INPUT);
		vscaler_filter_mode(vs, SCALER_LINEAR);
		vscaler_scale_mode(vs, SCLAER_UPSCALE);
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
		hscaler_scale_mode(hs, SCLAER_UPSCALE);
		hscaler_mode(hs, SCALER_ACTIVE);
		hscaler_set_stream_id(hs, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		lb_src = (lb_sec_sel_t)hs_id;

		dev_dbg(dev, "[PLANE:%d:%s] hscaler-0x%02x\n",
					plane->base.id, plane->name, hs_id);
	}

	if (dpstate->use_prefetch) {
		fetchdecode_configure_prefetch(fd, dplane->stream_id,
					       src_w, src_h, src_x, src_y,
					       fb->pitches[0], fb->format->format,
					       fb->modifier,
					       baseaddr, uv_baseaddr,
					       prefetch_start,
					       aux_prefetch_start);
		if (prefetch_start || aux_prefetch_start)
			fetchdecode_enable_prefetch(fd);

		fetchdecode_reg_update_prefetch(fd);

		if (prefetch_start || aux_prefetch_start)
			fetchdecode_prefetch_enable_first_frame_irq(fd);

		dev_dbg(dev, "[PLANE:%d:%s] use prefetch\n",
					plane->base.id, plane->name);
	} else if (fetchdecode_has_prefetch(fd)) {
		fetchdecode_disable_prefetch(fd);

		dev_dbg(dev, "[PLANE:%d:%s] bypass prefetch\n",
					plane->base.id, plane->name);
	}

	layerblend_pixengcfg_dynamic_prim_sel(lb, dpstate->stage);
	layerblend_pixengcfg_dynamic_sec_sel(lb, lb_src);
	layerblend_control(lb, LB_BLEND);
	layerblend_pixengcfg_clken(lb, CLKEN__AUTOMATIC);
	layerblend_position(lb, dpstate->layer_x, dpstate->layer_y);

	if (plane->type == DRM_PLANE_TYPE_PRIMARY) {
		cf = res->cf[dplane->stream_id];
		constframe_framedimensions(cf,
					dpstate->base_w, dpstate->base_h);
		constframe_constantcolor(cf, 0, 0, 0, 0);

		framegen_sacfg(res->fg, dpstate->base_x, dpstate->base_y);
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
				 enum drm_plane_type type)
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

	plane = &dpu_plane->base;

	ret = drm_universal_plane_init(drm, plane, possible_crtcs,
				       &dpu_plane_funcs, dpu_common_formats,
				       ARRAY_SIZE(dpu_common_formats), NULL,
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
