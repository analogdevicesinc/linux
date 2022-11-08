// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2022 NXP.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <linux/dma-buf.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

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
	DRM_FORMAT_NV15,
};

static const u64 dcss_overlay_format_modifiers[] = {
	DRM_FORMAT_MOD_VSI_G1_TILED,
	DRM_FORMAT_MOD_VSI_G2_TILED,
	DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED,
	DRM_FORMAT_MOD_VIVANTE_TILED,
	DRM_FORMAT_MOD_VIVANTE_SUPER_TILED,
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static const u64 dcss_primary_format_modifiers[] = {
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

static inline bool dcss_plane_fb_is_linear(const struct drm_framebuffer *fb)
{
	return ((fb->flags & DRM_MODE_FB_MODIFIERS) == 0) ||
	       ((fb->flags & DRM_MODE_FB_MODIFIERS) != 0 &&
		fb->modifier == DRM_FORMAT_MOD_LINEAR);
}

static void dcss_plane_destroy(struct drm_plane *plane)
{
	struct dcss_plane *dcss_plane = container_of(plane, struct dcss_plane,
						     base);

	drm_plane_cleanup(plane);
	kfree(dcss_plane);
}

static int dcss_plane_atomic_set_property(struct drm_plane *plane,
					  struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t val)
{
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);

	if (property == dcss_plane->dtrc_table_ofs_prop)
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

	if (property == dcss_plane->dtrc_table_ofs_prop)
		*val = dcss_plane->dtrc_table_ofs_val;
	else
		return -EINVAL;

	return 0;
}

static bool dcss_plane_format_mod_supported(struct drm_plane *plane,
					    u32 format,
					    u64 modifier)
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
		case DRM_FORMAT_NV15:
			return modifier == DRM_FORMAT_MOD_LINEAR ||
			       modifier == DRM_FORMAT_MOD_VSI_G1_TILED ||
			       modifier == DRM_FORMAT_MOD_VSI_G2_TILED ||
			       modifier == DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED;
		case DRM_FORMAT_ARGB8888:
		case DRM_FORMAT_XRGB8888:
		case DRM_FORMAT_ARGB2101010:
			return modifier == DRM_FORMAT_MOD_LINEAR ||
			       modifier == DRM_FORMAT_MOD_VIVANTE_TILED ||
			       modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED;
		default:
			return modifier == DRM_FORMAT_MOD_LINEAR;
		}
		break;
	default:
		return false;
	}

	return false;
}

static const struct drm_plane_funcs dcss_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= dcss_plane_destroy,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.atomic_set_property	= dcss_plane_atomic_set_property,
	.atomic_get_property	= dcss_plane_atomic_get_property,
	.format_mod_supported	= dcss_plane_format_mod_supported,
};

static bool dcss_plane_can_rotate(const struct drm_format_info *format,
				  bool mod_present, u64 modifier,
				  unsigned int rotation)
{
	bool linear_format = !mod_present || modifier == DRM_FORMAT_MOD_LINEAR;
	u32 supported_rotation = DRM_MODE_ROTATE_0;

	if (!format->is_yuv && linear_format)
		supported_rotation = DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_180 |
				     DRM_MODE_REFLECT_MASK;
	else if (!format->is_yuv &&
		 (modifier == DRM_FORMAT_MOD_VIVANTE_TILED ||
		  modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED ||
		  modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED_FC))
		supported_rotation = DRM_MODE_ROTATE_MASK |
				     DRM_MODE_REFLECT_MASK;
	else if (format->is_yuv && linear_format &&
		 (format->format == DRM_FORMAT_NV12 ||
		  format->format == DRM_FORMAT_NV21))
		supported_rotation = DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_180 |
				     DRM_MODE_REFLECT_MASK;
	else if (format->is_yuv && linear_format &&
		 format->format == DRM_FORMAT_NV15)
		supported_rotation = DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_Y;

	return !!(rotation & supported_rotation);
}

static void dcss_plane_get_hdr10_pipe_cfg(struct drm_plane_state *plane_state,
					  struct drm_crtc_state *crtc_state,
					  struct dcss_hdr10_pipe_cfg *ipipe_cfg,
					  struct dcss_hdr10_pipe_cfg *opipe_cfg)
{
	struct dcss_crtc_state *dcss_crtc_state = to_dcss_crtc_state(crtc_state);
	struct drm_framebuffer *fb = plane_state->fb;

	opipe_cfg->is_yuv =
			dcss_crtc_state->output_encoding != DCSS_PIPE_OUTPUT_RGB;
	opipe_cfg->g = dcss_crtc_state->opipe_g;
	opipe_cfg->nl = dcss_crtc_state->opipe_nl;
	opipe_cfg->pr = dcss_crtc_state->opipe_pr;

	ipipe_cfg->is_yuv = fb->format->is_yuv;

	if (!fb->format->is_yuv) {
		ipipe_cfg->pr = PR_FULL;
		if (fb->format->depth == 30) {
			ipipe_cfg->nl = NL_REC2084;
			ipipe_cfg->g = G_REC2020;
		} else {
			ipipe_cfg->nl = NL_REC709;
			ipipe_cfg->g = G_REC709;
		}
		return;
	}

	switch (plane_state->color_encoding) {
	case DRM_COLOR_YCBCR_BT709:
		ipipe_cfg->nl = NL_REC709;
		ipipe_cfg->g = G_REC709;
		break;
	case DRM_COLOR_YCBCR_BT2020:
		ipipe_cfg->nl = NL_REC2084;
		ipipe_cfg->g = G_REC2020;
		break;
	default:
		ipipe_cfg->nl = NL_REC709;
		ipipe_cfg->g = G_REC709;
		break;
	}

	ipipe_cfg->pr = plane_state->color_range == DRM_COLOR_YCBCR_FULL_RANGE ?
			PR_FULL : PR_LIMITED;
}

static bool
dcss_plane_hdr10_pipe_cfg_is_supported(struct drm_plane_state *plane_state,
				       struct drm_crtc_state *crtc_state)
{
	struct dcss_dev *dcss = plane_state->plane->dev->dev_private;
	struct dcss_hdr10_pipe_cfg ipipe_cfg, opipe_cfg;

	dcss_plane_get_hdr10_pipe_cfg(plane_state, crtc_state,
				      &ipipe_cfg, &opipe_cfg);

	return dcss_hdr10_pipe_cfg_is_supported(dcss->hdr10,
						&ipipe_cfg, &opipe_cfg);
}

static bool dcss_plane_is_source_size_allowed(u16 src_w, u16 src_h, u32 pix_fmt)
{
	if (src_w < 64 &&
	    (pix_fmt == DRM_FORMAT_NV12 || pix_fmt == DRM_FORMAT_NV21 ||
	     pix_fmt == DRM_FORMAT_NV15))
		return false;
	else if (src_w < 32 &&
		 (pix_fmt == DRM_FORMAT_UYVY || pix_fmt == DRM_FORMAT_VYUY ||
		  pix_fmt == DRM_FORMAT_YUYV || pix_fmt == DRM_FORMAT_YVYU))
		return false;

	return src_w >= 16 && src_h >= 8;
}

static inline bool dcss_plane_use_dtrc(struct drm_framebuffer *fb,
				       enum drm_plane_type type)
{
	u64 pix_format = fb->format->format;

	return !dcss_plane_fb_is_linear(fb) &&
		type == DRM_PLANE_TYPE_OVERLAY &&
		(pix_format == DRM_FORMAT_NV12 ||
		pix_format == DRM_FORMAT_NV21 ||
		pix_format == DRM_FORMAT_NV15);
}

static int dcss_plane_atomic_check(struct drm_plane *plane,
				   struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);
	struct dcss_dev *dcss = plane->dev->dev_private;
	struct drm_framebuffer *fb = new_plane_state->fb;
	struct drm_gem_dma_object *dma_obj;
	struct drm_crtc_state *crtc_state;
	int hdisplay, vdisplay;
	int min, max;
	int ret;

	if (!fb || !new_plane_state->crtc)
		return 0;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	WARN_ON(!dma_obj);

	crtc_state = drm_atomic_get_existing_crtc_state(state,
							new_plane_state->crtc);

	hdisplay = crtc_state->adjusted_mode.hdisplay;
	vdisplay = crtc_state->adjusted_mode.vdisplay;

	if (!dcss_plane_is_source_size_allowed(new_plane_state->src_w >> 16,
					       new_plane_state->src_h >> 16,
					       fb->format->format)) {
		DRM_DEBUG_KMS("Source plane size is not allowed!\n");
		return -EINVAL;
	}

	dcss_scaler_get_min_max_ratios(dcss->scaler, dcss_plane->ch_num,
				       &min, &max);

	ret = drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						  min, max, true, false);
	if (ret)
		return ret;

	if (!new_plane_state->visible)
		return 0;

	if (!dcss_plane_can_rotate(fb->format,
				   !!(fb->flags & DRM_MODE_FB_MODIFIERS),
				   fb->modifier,
				   new_plane_state->rotation)) {
		DRM_DEBUG_KMS("requested rotation is not allowed!\n");
		return -EINVAL;
	}

	if (!dcss_plane_hdr10_pipe_cfg_is_supported(new_plane_state,
						    crtc_state)) {
		DRM_DEBUG_KMS("requested hdr10 pipe cfg is not supported!\n");
		return -EINVAL;
	}

	if ((fb->flags & DRM_MODE_FB_MODIFIERS) &&
	    !plane->funcs->format_mod_supported(plane,
				fb->format->format,
				fb->modifier)) {
		DRM_DEBUG_KMS("Invalid modifier: %llx", fb->modifier);
		return -EINVAL;
	}

	if (fb->modifier == DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED &&
	    dcss_plane->dtrc_table_ofs_val == 0) {
		DRM_ERROR_RATELIMITED("No DTRC decompression table offset set, reject plane.\n");
		return -EINVAL;
	}

	dcss_plane->use_dtrc = dcss_plane_use_dtrc(fb, plane->type);

	return 0;
}

static struct drm_gem_object *dcss_plane_gem_import(struct drm_device *dev,
						    struct dma_buf *dma_buf)
{
	struct drm_gem_object *obj;

	if (IS_ERR(dma_buf))
		return ERR_CAST(dma_buf);

	mutex_lock(&dev->object_name_lock);

	obj = dev->driver->gem_prime_import(dev, dma_buf);

	mutex_unlock(&dev->object_name_lock);

	return obj;
}

static void dcss_plane_set_primary_base(struct dcss_plane *dcss_plane,
					u32 baddr)
{
	struct drm_plane *plane = &dcss_plane->base;
	struct dcss_dev *dcss = plane->dev->dev_private;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_dma_object *dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	struct dma_buf *dma_buf = dma_obj->base.dma_buf;
	struct drm_gem_object *gem_obj;
	dma_addr_t caddr;
	bool compressed = true;
	u32 compressed_format = _VIV_CFMT_ARGB8;
	_VIV_VIDMEM_METADATA *mdata;

	if (dcss_plane_fb_is_linear(fb) ||
	    ((fb->flags & DRM_MODE_FB_MODIFIERS) &&
	     (fb->modifier == DRM_FORMAT_MOD_VIVANTE_TILED ||
	      fb->modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED))) {
		dcss_dec400d_bypass(dcss->dec400d);
		return;
	}

	if (!dma_buf) {
		caddr = dma_obj->dma_addr + ALIGN(fb->height, 64) * fb->pitches[0];
	} else {
		mdata = dma_buf->priv;
		if (!mdata || mdata->magic != VIV_VIDMEM_METADATA_MAGIC)
			return;

		gem_obj = dcss_plane_gem_import(plane->dev, mdata->ts_dma_buf);
		if (IS_ERR(gem_obj))
			return;

		caddr = to_drm_gem_dma_obj(gem_obj)->dma_addr;

		/* release gem_obj */
		drm_gem_object_put(gem_obj);

		dcss_dec400d_fast_clear_config(dcss->dec400d, mdata->fc_value,
					       mdata->fc_enabled);

		compressed = !!mdata->compressed;
		compressed_format = mdata->compress_format;
	}

	dcss_dec400d_read_config(dcss->dec400d, 0, compressed,
				 compressed_format);
	dcss_dec400d_addr_set(dcss->dec400d, baddr, caddr);
}

static void dcss_plane_set_dtrc_base(struct dcss_plane *dcss_plane,
				     u32 p1_ba, u32 p2_ba)
{
	struct drm_plane *plane = &dcss_plane->base;
	struct dcss_dev *dcss = plane->dev->dev_private;

	if (!dcss_plane->use_dtrc) {
		dcss_dtrc_bypass(dcss->dtrc, dcss_plane->ch_num);
		return;
	}

	dcss_dtrc_addr_set(dcss->dtrc, dcss_plane->ch_num,
			   p1_ba, p2_ba, dcss_plane->dtrc_table_ofs_val);
}

static void dcss_plane_atomic_set_base(struct dcss_plane *dcss_plane)
{
	struct drm_plane *plane = &dcss_plane->base;
	struct drm_plane_state *state = plane->state;
	struct dcss_dev *dcss = plane->dev->dev_private;
	struct drm_framebuffer *fb = state->fb;
	const struct drm_format_info *format = fb->format;
	struct drm_gem_dma_object *dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	unsigned long p1_ba = 0, p2_ba = 0;
	u16 x1, y1;

	x1 = state->src.x1 >> 16;
	y1 = state->src.y1 >> 16;

	if (!format->is_yuv ||
	    format->format == DRM_FORMAT_NV12 ||
	    format->format == DRM_FORMAT_NV21)
		p1_ba = dma_obj->dma_addr + fb->offsets[0] +
			fb->pitches[0] * y1 +
			format->char_per_block[0] * x1;
	else if (format->format == DRM_FORMAT_NV15)
		p1_ba = dma_obj->dma_addr + fb->offsets[0] +
			fb->pitches[0] * y1 +
			format->char_per_block[0] * (x1 >> 2);
	else if (format->format == DRM_FORMAT_UYVY ||
		 format->format == DRM_FORMAT_VYUY ||
		 format->format == DRM_FORMAT_YUYV ||
		 format->format == DRM_FORMAT_YVYU)
		p1_ba = dma_obj->dma_addr + fb->offsets[0] +
			fb->pitches[0] * y1 +
			2 * format->char_per_block[0] * (x1 >> 1);

	if (format->format == DRM_FORMAT_NV12 ||
	    format->format == DRM_FORMAT_NV21)
		p2_ba = dma_obj->dma_addr + fb->offsets[1] +
			(((fb->pitches[1] >> 1) * (y1 >> 1) +
			(x1 >> 1)) << 1);
	else if (format->format == DRM_FORMAT_NV15)
		p2_ba = dma_obj->dma_addr + fb->offsets[1] +
			(((fb->pitches[1] >> 1) * (y1 >> 1)) << 1) +
			format->char_per_block[1] * (x1 >> 2);

	if (plane->type == DRM_PLANE_TYPE_PRIMARY)
		dcss_plane_set_primary_base(dcss_plane, p1_ba);
	else
		dcss_plane_set_dtrc_base(dcss_plane,
					 dma_obj->dma_addr + fb->offsets[0],
					 dma_obj->dma_addr + fb->offsets[1]);

	dcss_dpr_addr_set(dcss->dpr, dcss_plane->ch_num, p1_ba, p2_ba,
			  fb->pitches[0]);
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
	       fb->modifier  != old_fb->modifier ||
	       state->rotation != old_state->rotation ||
	       state->scaling_filter != old_state->scaling_filter;
}

static void dcss_plane_setup_hdr10_pipes(struct drm_plane *plane)
{
	struct dcss_dev *dcss = plane->dev->dev_private;
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);
	struct dcss_hdr10_pipe_cfg ipipe_cfg, opipe_cfg;

	dcss_plane_get_hdr10_pipe_cfg(plane->state,
				      plane->state->crtc->state,
				      &ipipe_cfg, &opipe_cfg);

	dcss_hdr10_setup(dcss->hdr10, dcss_plane->ch_num,
			 &ipipe_cfg, &opipe_cfg);
}

static void dcss_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(state,
									   plane);
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state,
									   plane);
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);
	struct dcss_dev *dcss = plane->dev->dev_private;
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_crtc_state *crtc_state;
	bool modifiers_present;
	u32 src_w, src_h, dst_w, dst_h;
	struct drm_rect src, dst;
	bool enable = true;
	bool is_rotation_90_or_270;

	if (!fb || !new_state->crtc || !new_state->visible)
		return;

	crtc_state = new_state->crtc->state;
	modifiers_present = !!(fb->flags & DRM_MODE_FB_MODIFIERS);

	if (old_state->fb && !drm_atomic_crtc_needs_modeset(crtc_state) &&
	    !dcss_plane_needs_setup(new_state, old_state) &&
	    !dcss_dtg_global_alpha_changed(dcss->dtg, dcss_plane->ch_num,
					   new_state->alpha >> 8)) {
		dcss_plane_atomic_set_base(dcss_plane);
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			dcss_dec400d_shadow_trig(dcss->dec400d);
		return;
	}

	src = plane->state->src;
	dst = plane->state->dst;

	/*
	 * The width and height after clipping.
	 */
	src_w = drm_rect_width(&src) >> 16;
	src_h = drm_rect_height(&src) >> 16;
	dst_w = drm_rect_width(&dst);
	dst_h = drm_rect_height(&dst);

	dcss_dpr_format_set(dcss->dpr, dcss_plane->ch_num,
			    new_state->fb->format,
			    modifiers_present ? fb->modifier :
						DRM_FORMAT_MOD_LINEAR);

	if (dcss_plane->use_dtrc) {
		u32 dtrc_w, dtrc_h;

		dcss_dtrc_set_res(dcss->dtrc, dcss_plane->ch_num, new_state,
				  &dtrc_w, &dtrc_h);
		dcss_dpr_set_res(dcss->dpr, dcss_plane->ch_num, dtrc_w, dtrc_h);
	} else {
		dcss_dpr_set_res(dcss->dpr, dcss_plane->ch_num, src_w, src_h);
	}

	dcss_dpr_set_rotation(dcss->dpr, dcss_plane->ch_num,
			      new_state->rotation);

	dcss_plane_atomic_set_base(dcss_plane);

	is_rotation_90_or_270 = new_state->rotation & (DRM_MODE_ROTATE_90 |
						   DRM_MODE_ROTATE_270);

	dcss_scaler_set_filter(dcss->scaler, dcss_plane->ch_num,
			       new_state->scaling_filter);

	dcss_scaler_setup(dcss->scaler, dcss_plane->ch_num,
			  new_state->fb->format,
			  is_rotation_90_or_270 ? src_h : src_w,
			  is_rotation_90_or_270 ? src_w : src_h,
			  dst_w, dst_h,
			  drm_mode_vrefresh(&crtc_state->mode));

	dcss_plane_setup_hdr10_pipes(plane);

	dcss_dtg_plane_pos_set(dcss->dtg, dcss_plane->ch_num,
			       dst.x1, dst.y1, dst_w, dst_h);
	dcss_dtg_plane_alpha_set(dcss->dtg, dcss_plane->ch_num,
				 fb->format, new_state->alpha >> 8);

	if (plane->type == DRM_PLANE_TYPE_PRIMARY)
		dcss_dec400d_enable(dcss->dec400d);
	else if (dcss_plane->use_dtrc)
		dcss_dtrc_enable(dcss->dtrc, dcss_plane->ch_num, true);

	if (!dcss_plane->ch_num && (new_state->alpha >> 8) == 0)
		enable = false;

	dcss_dpr_enable(dcss->dpr, dcss_plane->ch_num, enable);
	dcss_scaler_ch_enable(dcss->scaler, dcss_plane->ch_num, enable);

	if (!enable)
		dcss_dtg_plane_pos_set(dcss->dtg, dcss_plane->ch_num,
				       0, 0, 0, 0);

	dcss_dtg_ch_enable(dcss->dtg, dcss_plane->ch_num, enable);
}

static void dcss_plane_atomic_disable(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct dcss_plane *dcss_plane = to_dcss_plane(plane);
	struct dcss_dev *dcss = plane->dev->dev_private;

	if (dcss_plane->use_dtrc)
		dcss_dtrc_enable(dcss->dtrc, dcss_plane->ch_num, false);
	dcss_dpr_enable(dcss->dpr, dcss_plane->ch_num, false);
	dcss_scaler_ch_enable(dcss->scaler, dcss_plane->ch_num, false);
	dcss_dtg_plane_pos_set(dcss->dtg, dcss_plane->ch_num, 0, 0, 0, 0);
	dcss_dtg_ch_enable(dcss->dtg, dcss_plane->ch_num, false);
}

static const struct drm_plane_helper_funcs dcss_plane_helper_funcs = {
	.atomic_check = dcss_plane_atomic_check,
	.atomic_update = dcss_plane_atomic_update,
	.atomic_disable = dcss_plane_atomic_disable,
};

struct dcss_plane *dcss_plane_init(struct drm_device *drm,
				   unsigned int possible_crtcs,
				   enum drm_plane_type type,
				   unsigned int zpos)
{
	struct dcss_plane *dcss_plane;
	const u64 *format_modifiers = dcss_overlay_format_modifiers;
	struct drm_property *prop;
	int ret;

	if (zpos > 2)
		return ERR_PTR(-EINVAL);

	dcss_plane = kzalloc(sizeof(*dcss_plane), GFP_KERNEL);
	if (!dcss_plane) {
		DRM_ERROR("failed to allocate plane\n");
		return ERR_PTR(-ENOMEM);
	}

	if (type == DRM_PLANE_TYPE_PRIMARY)
		format_modifiers = dcss_primary_format_modifiers;

	ret = drm_universal_plane_init(drm, &dcss_plane->base, possible_crtcs,
				       &dcss_plane_funcs, dcss_common_formats,
				       ARRAY_SIZE(dcss_common_formats),
				       format_modifiers, type, NULL);
	if (ret) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(dcss_plane);
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(&dcss_plane->base, &dcss_plane_helper_funcs);

	ret = drm_plane_create_zpos_immutable_property(&dcss_plane->base, zpos);
	if (ret)
		return ERR_PTR(ret);

	drm_plane_create_scaling_filter_property(&dcss_plane->base,
					BIT(DRM_SCALING_FILTER_DEFAULT) |
					BIT(DRM_SCALING_FILTER_NEAREST_NEIGHBOR));

	drm_plane_create_rotation_property(&dcss_plane->base,
					   DRM_MODE_ROTATE_0,
					   DRM_MODE_ROTATE_0   |
					   DRM_MODE_ROTATE_90  |
					   DRM_MODE_ROTATE_180 |
					   DRM_MODE_ROTATE_270 |
					   DRM_MODE_REFLECT_X  |
					   DRM_MODE_REFLECT_Y);

	dcss_plane->ch_num = 2 - zpos;
	dcss_plane->type = type;

	if (type == DRM_PLANE_TYPE_PRIMARY)
		return dcss_plane;

	prop = drm_property_create_range(drm, 0, "dtrc_table_ofs",
					 0, ULLONG_MAX);
	if (!prop) {
		DRM_ERROR("cannot create dtrc_table_ofs property\n");
		return ERR_PTR(-ENOMEM);
	}

	dcss_plane->dtrc_table_ofs_prop = prop;
	drm_object_attach_property(&dcss_plane->base.base, prop, 0);

	return dcss_plane;
}
