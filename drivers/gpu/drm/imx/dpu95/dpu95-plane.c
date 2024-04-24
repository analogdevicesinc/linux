// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2017-2020,2022,2023 NXP
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane_helper.h>

#include "dpu95.h"
#include "dpu95-crtc.h"
#include "dpu95-drv.h"
#include "dpu95-plane.h"

#define FRAC_16_16(mult, div)			(((mult) << 16) / (div))

#define DPU95_PLANE_MAX_PITCH			0x10000
#define DPU95_PLANE_MAX_PIX_CNT			8192
#define DPU95_PLANE_MAX_PIX_CNT_WITH_SCALER	2048

static const uint32_t dpu95_plane_formats[] = {
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

static unsigned int dpu95_plane_get_default_zpos(enum drm_plane_type type)
{
	if (type == DRM_PLANE_TYPE_PRIMARY)
		return 0;
	else if (type == DRM_PLANE_TYPE_OVERLAY)
		return 1;

	return 0;
}

static void dpu95_plane_reset(struct drm_plane *plane)
{
	struct dpu95_plane_state *state;

	if (plane->state) {
		__drm_atomic_helper_plane_destroy_state(plane->state);
		kfree(to_dpu95_plane_state(plane->state));
		plane->state = NULL;
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return;

	__drm_atomic_helper_plane_reset(plane, &state->base);

	plane->state->zpos = dpu95_plane_get_default_zpos(plane->type);
	plane->state->color_encoding = DRM_COLOR_YCBCR_BT709;
	plane->state->color_range = DRM_COLOR_YCBCR_LIMITED_RANGE;
}

static struct drm_plane_state *
dpu95_drm_atomic_plane_duplicate_state(struct drm_plane *plane)
{
	struct dpu95_plane_state *state, *copy;

	if (WARN_ON(!plane->state))
		return NULL;

	copy = kmalloc(sizeof(*state), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);
	state = to_dpu95_plane_state(plane->state);
	copy->stage = state->stage;
	copy->source = state->source;
	copy->blend = state->blend;
	copy->is_top = state->is_top;

	return &copy->base;
}

static void dpu95_drm_atomic_plane_destroy_state(struct drm_plane *plane,
						 struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_dpu95_plane_state(state));
}

static bool dpu95_drm_plane_format_mod_supported(struct drm_plane *plane,
						 uint32_t format,
						 uint64_t modifier)
{
	return modifier == DRM_FORMAT_MOD_LINEAR;
}

static const struct drm_plane_funcs dpu95_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= dpu95_plane_reset,
	.atomic_duplicate_state	= dpu95_drm_atomic_plane_duplicate_state,
	.atomic_destroy_state	= dpu95_drm_atomic_plane_destroy_state,
	.format_mod_supported	= dpu95_drm_plane_format_mod_supported,
};

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

static inline dma_addr_t
drm_plane_state_to_uvbaseaddr(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *dma_obj;
	int x = state->src.x1 >> 16;
	int y = state->src.y1 >> 16;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 1);

	x /= fb->format->hsub;
	y /= fb->format->vsub;

	return dma_obj->dma_addr + fb->offsets[1] + fb->pitches[1] * y +
	       fb->format->cpp[1] * x;
}

static int dpu95_plane_check_no_off_screen(struct drm_plane_state *state,
					   struct drm_crtc_state *crtc_state)
{
	if (state->dst.x1 < 0 || state->dst.y1 < 0 ||
	    state->dst.x2 > crtc_state->adjusted_mode.hdisplay ||
	    state->dst.y2 > crtc_state->adjusted_mode.vdisplay) {
		dpu95_plane_dbg(state->plane, "no off screen\n");
		return -EINVAL;
	}

	return 0;
}

static int dpu95_plane_check_no_vscaling(struct drm_plane_state *state)
{
	u32 src_h = drm_rect_height(&state->src) >> 16;
	u32 dst_h = drm_rect_height(&state->dst);

	if (src_h != dst_h) {
		dpu95_plane_dbg(state->plane, "no vertical scaling\n");
		return -EINVAL;
	}

	return 0;
}

static int dpu95_plane_check_no_deinterlacing(struct drm_plane_state *state)
{
	if (state->fb->flags & DRM_MODE_FB_INTERLACED) {
		dpu95_plane_dbg(state->plane, "no deinterlacing\n");
		return -EINVAL;
	}

	return 0;
}

static int dpu95_plane_check_max_source_resolution(struct drm_plane_state *state)
{
	u32 src_w = drm_rect_width(&state->src) >> 16;
	u32 src_h = drm_rect_height(&state->src) >> 16;
	u32 dst_w = drm_rect_width(&state->dst);
	u32 dst_h = drm_rect_height(&state->dst);

	if (src_w == dst_w || src_h == dst_h) {
		/* without scaling */
		if (src_w > DPU95_PLANE_MAX_PIX_CNT ||
		    src_h > DPU95_PLANE_MAX_PIX_CNT) {
			dpu95_plane_dbg(state->plane,
					"invalid source resolution\n");
			return -EINVAL;
		}
	} else {
		/* with scaling */
		if (src_w > DPU95_PLANE_MAX_PIX_CNT_WITH_SCALER) {
			dpu95_plane_dbg(state->plane,
					"invalid source resolution with scale\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int dpu95_plane_check_source_alignment(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	u32 src_w = drm_rect_width(&state->src) >> 16;
	u32 src_h = drm_rect_height(&state->src) >> 16;
	u32 src_x = state->src.x1 >> 16;
	u32 src_y = state->src.y1 >> 16;

	if (fb->format->hsub == 2) {
		if (src_w % 2) {
			dpu95_plane_dbg(state->plane, "bad uv width\n");
			return -EINVAL;
		}
		if (src_x % 2) {
			dpu95_plane_dbg(state->plane, "bad uv xoffset\n");
			return -EINVAL;
		}
	}
	if (fb->format->vsub == 2) {
		if (src_h % 2) {
			dpu95_plane_dbg(state->plane, "bad uv height\n");
			return -EINVAL;
		}
		if (src_y % 2) {
			dpu95_plane_dbg(state->plane, "bad uv yoffset\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int dpu95_plane_check_no_bt709_full_range(struct drm_plane_state *state)
{
	if (state->fb->format->is_yuv &&
	    state->color_encoding == DRM_COLOR_YCBCR_BT709 &&
	    state->color_range == DRM_COLOR_YCBCR_FULL_RANGE) {
		dpu95_plane_dbg(state->plane, "no BT709 full range support\n");
		return -EINVAL;
	}

	return 0;
}

static int dpu95_plane_check_fb_plane_1(struct drm_plane_state *state)
{
	struct drm_plane *plane = state->plane;
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t baseaddr = drm_plane_state_to_baseaddr(state);
	int bpp;

	/* base address alignment */
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
	if ((bpp == 32 && (baseaddr & 0x3)) ||
	    (bpp == 16 && (baseaddr & 0x1))) {
		dpu95_plane_dbg(plane, "%dbpp fb bad baddr alignment\n", bpp);
		return -EINVAL;
	}
	switch (bpp) {
	case 32:
		if (baseaddr & 0x3) {
			dpu95_plane_dbg(plane, "32bpp fb bad baddr alignment\n");
			return -EINVAL;
		}
		break;
	case 16:
		if (baseaddr & 0x7) {
			dpu95_plane_dbg(plane, "16bpp fb bad baddr alignment\n");
			return -EINVAL;
		}
		break;
	}

	/* pitches[0] range */
	if (fb->pitches[0] > DPU95_PLANE_MAX_PITCH) {
		dpu95_plane_dbg(plane, "fb pitches[0] is out of range\n");
		return -EINVAL;
	}

	/* pitches[0] alignment */
	if ((bpp == 32 && (fb->pitches[0] & 0x3)) ||
	    (bpp == 16 && (fb->pitches[0] & 0x1))) {
		dpu95_plane_dbg(plane, "%dbpp fb bad pitches[0] alignment\n", bpp);
		return -EINVAL;
	}

	return 0;
}

/* UV planar check, assuming 16bpp */
static int dpu95_plane_check_fb_plane_2(struct drm_plane_state *state)
{
	struct drm_plane *plane = state->plane;
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t uv_baseaddr = drm_plane_state_to_uvbaseaddr(state);

	/* base address alignment */
	if (uv_baseaddr & 0x7) {
		dpu95_plane_dbg(plane, "bad uv baddr alignment\n");
		return -EINVAL;
	}

	/* pitches[1] range */
	if (fb->pitches[1] > DPU95_PLANE_MAX_PITCH) {
		dpu95_plane_dbg(plane, "fb pitches[1] is out of range\n");
		return -EINVAL;
	}

	/* pitches[1] alignment */
	if (fb->pitches[1] & 0x1) {
		dpu95_plane_dbg(plane, "fb bad pitches[1] alignment\n");
		return -EINVAL;
	}

	return 0;
}

static int dpu95_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state =
				drm_atomic_get_new_plane_state(state, plane);
	struct dpu95_plane_state *new_dpstate =
				to_dpu95_plane_state(new_plane_state);
	struct drm_framebuffer *fb = new_plane_state->fb;
	struct drm_crtc_state *crtc_state;
	int min_scale, ret;

	/* ok to disable */
	if (!fb) {
		new_dpstate->source = NULL;
		new_dpstate->stage.ptr = NULL;
		new_dpstate->blend = NULL;
		return 0;
	}

	if (!new_plane_state->crtc) {
		dpu95_plane_dbg(plane, "no CRTC in plane state\n");
		return -EINVAL;
	}

	crtc_state =
		drm_atomic_get_existing_crtc_state(state, new_plane_state->crtc);
	if (WARN_ON(!crtc_state))
		return -EINVAL;

	min_scale = FRAC_16_16(1, DPU95_PLANE_MAX_PIX_CNT_WITH_SCALER);
	ret = drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						  min_scale, DRM_PLANE_NO_SCALING,
						  true, false);
	if (ret) {
		dpu95_plane_dbg(plane, "failed to check plane state: %d\n", ret);
		return ret;
	}

	ret = dpu95_plane_check_no_off_screen(new_plane_state, crtc_state);
	if (ret)
		return ret;

	ret = dpu95_plane_check_no_vscaling(new_plane_state);
	if (ret)
		return ret;

	ret = dpu95_plane_check_no_deinterlacing(new_plane_state);
	if (ret)
		return ret;

	ret = dpu95_plane_check_max_source_resolution(new_plane_state);
	if (ret)
		return ret;

	ret = dpu95_plane_check_source_alignment(new_plane_state);
	if (ret)
		return ret;

	ret = dpu95_plane_check_no_bt709_full_range(new_plane_state);
	if (ret)
		return ret;

	ret = dpu95_plane_check_fb_plane_1(new_plane_state);
	if (ret)
		return ret;

	if (fb->format->num_planes > 1) {
		ret = dpu95_plane_check_fb_plane_2(new_plane_state);
		if (ret)
			return ret;
	}

	return 0;
}

static void dpu95_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct dpu95_plane *dplane = to_dpu95_plane(plane);
	struct drm_plane_state *new_state =
				drm_atomic_get_new_plane_state(state, plane);
	struct dpu95_plane_state *new_dpstate = to_dpu95_plane_state(new_state);
	struct dpu95_plane_grp *grp = dplane->grp;
	struct dpu95_crtc *dpu_crtc;
	struct drm_framebuffer *fb = new_state->fb;
	struct dpu95_fetchunit *fu = new_dpstate->source;
	struct dpu95_layerblend *lb = new_dpstate->blend;
	const struct dpu95_fetchunit_ops *fu_ops;
	dma_addr_t baseaddr, uv_baseaddr;
	enum dpu95_link_id fu_link;
	enum dpu95_link_id lb_src_link, stage_link;
	unsigned int src_w, src_h, dst_w;
	bool need_fetcheco = false, need_hscaler = false;

	/*
	 * Do nothing since the plane is disabled by
	 * crtc_func->atomic_begin/flush.
	 */
	if (!fb)
		return;

	/* Do nothing if CRTC is inactive. */
	if (!new_state->crtc->state->active)
		return;

	src_w = drm_rect_width(&new_state->src) >> 16;
	src_h = drm_rect_height(&new_state->src) >> 16;
	dst_w = drm_rect_width(&new_state->dst);

	if (fb->format->num_planes > 1)
		need_fetcheco = true;

	if (src_w != dst_w)
		need_hscaler = true;

	baseaddr = drm_plane_state_to_baseaddr(new_state);
	if (need_fetcheco)
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(new_state);

	dpu_crtc = to_dpu95_crtc(new_state->crtc);

	fu_ops = dpu95_fu_get_ops(fu);

	fu_ops->set_layerblend(fu, lb);
	fu_ops->set_numbuffers(fu, 16);
	fu_ops->set_burstlength(fu, 16);
	fu_ops->set_src_stride(fu, fb->pitches[0]);
	fu_ops->set_src_buf_dimensions(fu, src_w, src_h, fb->format, false);
	fu_ops->set_fmt(fu, fb->format, new_state->color_encoding,
			new_state->color_range, false);
	fu_ops->set_pixel_blend_mode(fu, new_state->pixel_blend_mode,
				     new_state->alpha, fb->format->has_alpha);
	fu_ops->enable_src_buf(fu);
	fu_ops->set_framedimensions(fu, src_w, src_h, false);
	fu_ops->set_baseaddress(fu, baseaddr);
	fu_ops->set_stream_id(fu, dpu_crtc->stream_id);

	fu_link = fu_ops->get_link_id(fu);
	lb_src_link = fu_link;

	dpu95_plane_dbg(plane, "uses %s\n", fu_ops->get_name(fu));

	if (need_fetcheco) {
		struct dpu95_fetchunit *fe = fu_ops->get_fetcheco(fu);
		const struct dpu95_fetchunit_ops *fe_ops;

		fe_ops = dpu95_fu_get_ops(fe);

		fu_ops->set_pec_dynamic_src_sel(fu, fe_ops->get_link_id(fe));

		fe_ops->set_numbuffers(fu, 16);
		fe_ops->set_burstlength(fu, 16);
		fe_ops->set_src_stride(fe, fb->pitches[1]);
		fe_ops->set_fmt(fe, fb->format, new_state->color_encoding,
				new_state->color_range, false);
		fe_ops->set_src_buf_dimensions(fe, src_w, src_h,
					       fb->format, false);
		fe_ops->set_framedimensions(fe, src_w, src_h, false);
		fe_ops->set_baseaddress(fe, uv_baseaddr);
		fe_ops->enable_src_buf(fe);

		dpu95_plane_dbg(plane, "uses %s\n", fe_ops->get_name(fe));
	} else {
		if (fu_ops->set_pec_dynamic_src_sel)
			fu_ops->set_pec_dynamic_src_sel(fu, DPU95_LINK_ID_NONE);
	}

	if (need_hscaler) {
		struct dpu95_hscaler *hs = fu_ops->get_hscaler(fu);
		const struct dpu95_hscaler_ops *hs_ops;

		dpu95_hs_pec_dynamic_src_sel(hs, fu_link);
		dpu95_hs_pec_clken(hs, CLKEN_AUTOMATIC);
		dpu95_hs_setup1(hs, src_w, dst_w);
		dpu95_hs_output_size(hs, dst_w);
		dpu95_hs_filter_mode(hs, SCALER_LINEAR);
		dpu95_hs_scale_mode(hs, SCALER_UPSCALE);
		dpu95_hs_mode(hs, SCALER_ACTIVE);

		hs_ops = dpu95_hs_get_ops(hs);
		hs_ops->set_stream_id(hs, dpu_crtc->stream_id);

		lb_src_link = dpu95_hs_get_link_id(hs);

		dpu95_plane_dbg(plane, "uses HScaler%u\n", dpu95_hs_get_id(hs));
	}

	if (new_state->normalized_zpos == 0)
		stage_link = dpu95_cf_get_link_id(new_dpstate->stage.cf);
	else
		stage_link = dpu95_lb_get_link_id(new_dpstate->stage.lb);

	dpu95_lb_pec_dynamic_prim_sel(lb, stage_link);
	dpu95_lb_pec_dynamic_sec_sel(lb, lb_src_link);
	dpu95_lb_mode(lb, LB_BLEND);
	dpu95_lb_blendcontrol(lb, new_state->normalized_zpos,
			      new_state->pixel_blend_mode, new_state->alpha);
	dpu95_lb_pec_clken(lb, CLKEN_AUTOMATIC);
	dpu95_lb_position(lb, new_state->dst.x1, new_state->dst.y1);

	dpu95_plane_dbg(plane, "uses LayerBlend%u\n", dpu95_lb_get_id(lb));

	if (new_dpstate->is_top)
		dpu95_ed_pec_src_sel(grp->ed[dpu_crtc->stream_id],
				     dpu95_lb_get_link_id(lb));
}

static const struct drm_plane_helper_funcs dpu95_plane_helper_funcs = {
	.prepare_fb	= drm_gem_plane_helper_prepare_fb,
	.atomic_check	= dpu95_plane_atomic_check,
	.atomic_update	= dpu95_plane_atomic_update,
};

int dpu95_plane_initialize(struct dpu95_drm_device *dpu_drm,
			   struct dpu95_plane *dpu_plane, u32 possible_crtcs,
			   enum drm_plane_type type)
{
	struct dpu95_plane_grp *plane_grp = &dpu_drm->dpu_plane_grp;
	struct drm_plane *plane = &dpu_plane->base;
	struct drm_device *drm = &dpu_drm->base;
	unsigned int zpos;
	int ret;

	dpu_plane->grp = plane_grp;

	ret = drm_universal_plane_init(drm, plane, possible_crtcs,
				       &dpu95_plane_funcs,
				       dpu95_plane_formats,
				       ARRAY_SIZE(dpu95_plane_formats),
				       NULL, type, NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(plane, &dpu95_plane_helper_funcs);

	zpos = dpu95_plane_get_default_zpos(type);
	ret = drm_plane_create_zpos_property(plane,
					     zpos, 0, DPU95_HW_PLANES - 1);
	if (ret)
		return ret;

	ret = drm_plane_create_alpha_property(plane);
	if (ret)
		return ret;

	ret = drm_plane_create_blend_mode_property(plane,
					BIT(DRM_MODE_BLEND_PIXEL_NONE) |
					BIT(DRM_MODE_BLEND_PREMULTI) |
					BIT(DRM_MODE_BLEND_COVERAGE));
	if (ret)
		return ret;

	return drm_plane_create_color_properties(plane,
					BIT(DRM_COLOR_YCBCR_BT601) |
					BIT(DRM_COLOR_YCBCR_BT709),
					BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) |
					BIT(DRM_COLOR_YCBCR_FULL_RANGE),
					DRM_COLOR_YCBCR_BT709,
					DRM_COLOR_YCBCR_LIMITED_RANGE);
}
