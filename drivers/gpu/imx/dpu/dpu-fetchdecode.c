/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
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

#include <drm/drm_blend.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include "dpu-prv.h"

static const u32 fd_vproc_cap[2] = {
	DPU_VPROC_CAP_HSCALER4 | DPU_VPROC_CAP_VSCALER4 |
	DPU_VPROC_CAP_FETCHECO0,
	DPU_VPROC_CAP_HSCALER5 | DPU_VPROC_CAP_VSCALER5 |
	DPU_VPROC_CAP_FETCHECO1,
};

#define PIXENGCFG_DYNAMIC		0x8
static const fd_dynamic_src_sel_t fd_srcs[2][4] = {
	{
	  FD_SRC_DISABLE,	FD_SRC_FETCHECO0,
	  FD_SRC_FETCHDECODE1,	FD_SRC_FETCHWARP2
	}, {
	  FD_SRC_DISABLE,	FD_SRC_FETCHECO1,
	  FD_SRC_FETCHDECODE0,	FD_SRC_FETCHWARP2
	},
};

#define PIXENGCFG_STATUS		0xC

#define RINGBUFSTARTADDR0		0x10
#define RINGBUFWRAPADDR0		0x14
#define FRAMEPROPERTIES0		0x18
#define BASEADDRESS0			0x1C
#define SOURCEBUFFERATTRIBUTES0		0x20
#define SOURCEBUFFERDIMENSION0		0x24
#define COLORCOMPONENTBITS0		0x28
#define COLORCOMPONENTSHIFT0		0x2C
#define LAYEROFFSET0			0x30
#define CLIPWINDOWOFFSET0		0x34
#define CLIPWINDOWDIMENSIONS0		0x38
#define CONSTANTCOLOR0			0x3C
#define LAYERPROPERTY0			0x40
#define FRAMEDIMENSIONS			0x44
#define FRAMERESAMPLING			0x48
#define DECODECONTROL			0x4C
#define SOURCEBUFFERLENGTH		0x50
#define CONTROL				0x54
#define CONTROLTRIGGER			0x58
#define START				0x5C
#define FETCHTYPE			0x60
#define DECODERSTATUS			0x64
#define READADDRESS0			0x68
#define BURSTBUFFERPROPERTIES		0x6C
#define STATUS				0x70
#define HIDDENSTATUS			0x74

struct dpu_fetchdecode {
	struct dpu_fetchunit fu;
	fetchtype_t fetchtype;
};

int fetchdecode_pixengcfg_dynamic_src_sel(struct dpu_fetchunit *fu,
					  fd_dynamic_src_sel_t src)
{
	int i;

	mutex_lock(&fu->mutex);
	for (i = 0; i < 4; i++) {
		if (fd_srcs[fu->id][i] == src) {
			dpu_pec_fu_write(fu, PIXENGCFG_DYNAMIC, src);
			mutex_unlock(&fu->mutex);
			return 0;
		}
	}
	mutex_unlock(&fu->mutex);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(fetchdecode_pixengcfg_dynamic_src_sel);

static void
fetchdecode_set_baseaddress(struct dpu_fetchunit *fu, unsigned int width,
			    unsigned int x_offset, unsigned int y_offset,
			    unsigned int mt_w, unsigned int mt_h,
			    int bpp, dma_addr_t baddr)
{
	unsigned int burst_size, stride;
	bool nonzero_mod = !!mt_w;

	if (nonzero_mod) {
		/* consider PRG x offset to calculate buffer address */
		baddr += (x_offset % mt_w) * (bpp / 8);

		burst_size = fetchunit_burst_size_fixup_tkt343664(baddr);

		stride = width * (bpp / 8);
		stride = fetchunit_stride_fixup_tkt339017(stride, burst_size,
							  baddr, nonzero_mod);

		/* consider PRG y offset to calculate buffer address */
		baddr += (y_offset % mt_h) * stride;
	}

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, BASEADDRESS0, baddr);
	mutex_unlock(&fu->mutex);
}

static void fetchdecode_set_src_bpp(struct dpu_fetchunit *fu, int bpp)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, SOURCEBUFFERATTRIBUTES0);
	val &= ~0x3f0000;
	val |= BITSPERPIXEL(bpp);
	dpu_fu_write(fu, SOURCEBUFFERATTRIBUTES0, val);
	mutex_unlock(&fu->mutex);
}

static void
fetchdecode_set_src_stride(struct dpu_fetchunit *fu,
			   unsigned int width, unsigned int x_offset,
			   unsigned int mt_w, int bpp, unsigned int stride,
			   dma_addr_t baddr, bool use_prefetch)
{
	unsigned int burst_size;
	bool nonzero_mod = !!mt_w;
	u32 val;

	if (use_prefetch) {
		/* consider PRG x offset to calculate buffer address */
		if (nonzero_mod)
			baddr += (x_offset % mt_w) * (bpp / 8);

		burst_size = fetchunit_burst_size_fixup_tkt343664(baddr);

		stride = width * (bpp / 8);
		stride = fetchunit_stride_fixup_tkt339017(stride, burst_size,
							  baddr, nonzero_mod);
	}

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, SOURCEBUFFERATTRIBUTES0);
	val &= ~0xffff;
	val |= STRIDE(stride);
	dpu_fu_write(fu, SOURCEBUFFERATTRIBUTES0, val);
	mutex_unlock(&fu->mutex);
}

static void
fetchdecode_set_src_buf_dimensions(struct dpu_fetchunit *fu,
				   unsigned int w, unsigned int h,
				   u32 unused, bool deinterlace)
{
	u32 val;

	if (deinterlace)
		h /= 2;

	val = LINEWIDTH(w) | LINECOUNT(h);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, SOURCEBUFFERDIMENSION0, val);
	mutex_unlock(&fu->mutex);
}

static void fetchdecode_set_fmt(struct dpu_fetchunit *fu,
				u32 fmt,
				enum drm_color_encoding color_encoding,
				enum drm_color_range color_range,
				bool deinterlace)
{
	u32 val, bits, shift;
	bool is_planar_yuv = false, is_rastermode_yuv422 = false;
	bool is_yuv422upsamplingmode_interpolate = false;
	bool is_inputselect_compact = false;
	bool need_csc = false;
	int i;

	switch (fmt) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		is_rastermode_yuv422 = true;
		is_yuv422upsamplingmode_interpolate = true;
		need_csc = true;
		break;
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
		is_yuv422upsamplingmode_interpolate = true;
		/* fall-through */
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		if (deinterlace)
			is_yuv422upsamplingmode_interpolate = true;
		is_planar_yuv = true;
		is_rastermode_yuv422 = true;
		is_inputselect_compact = true;
		need_csc = true;
		break;
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		is_planar_yuv = true;
		is_yuv422upsamplingmode_interpolate = true;
		is_inputselect_compact = true;
		need_csc = true;
		break;
	default:
		break;
	}

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, CONTROL);
	val &= ~YUV422UPSAMPLINGMODE_MASK;
	val &= ~INPUTSELECT_MASK;
	val &= ~RASTERMODE_MASK;
	if (is_yuv422upsamplingmode_interpolate)
		val |= YUV422UPSAMPLINGMODE(YUV422UPSAMPLINGMODE__INTERPOLATE);
	else
		val |= YUV422UPSAMPLINGMODE(YUV422UPSAMPLINGMODE__REPLICATE);
	if (is_inputselect_compact)
		val |= INPUTSELECT(INPUTSELECT__COMPPACK);
	else
		val |= INPUTSELECT(INPUTSELECT__INACTIVE);
	if (is_rastermode_yuv422)
		val |= RASTERMODE(RASTERMODE__YUV422);
	else
		val |= RASTERMODE(RASTERMODE__NORMAL);
	dpu_fu_write(fu, CONTROL, val);

	val = dpu_fu_read(fu, LAYERPROPERTY0);
	val &= ~YUVCONVERSIONMODE_MASK;
	if (need_csc) {
		/* assuming fetchdecode always ouputs RGB pixel formats */
		if (color_encoding == DRM_COLOR_YCBCR_BT709)
			val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE__ITU709);
		else if (color_encoding == DRM_COLOR_YCBCR_BT601 &&
			 color_range == DRM_COLOR_YCBCR_FULL_RANGE)
			val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE__ITU601_FR);
		else
			val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE__ITU601);
	} else {
		val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE__OFF);
	}
	dpu_fu_write(fu, LAYERPROPERTY0, val);
	mutex_unlock(&fu->mutex);

	for (i = 0; i < ARRAY_SIZE(dpu_pixel_format_matrix); i++) {
		if (dpu_pixel_format_matrix[i].pixel_format == fmt) {
			bits = dpu_pixel_format_matrix[i].bits;
			shift = dpu_pixel_format_matrix[i].shift;

			if (is_planar_yuv) {
				bits &= ~(U_BITS_MASK | V_BITS_MASK);
				shift &= ~(U_SHIFT_MASK | V_SHIFT_MASK);
			}

			mutex_lock(&fu->mutex);
			dpu_fu_write(fu, COLORCOMPONENTBITS0, bits);
			dpu_fu_write(fu, COLORCOMPONENTSHIFT0, shift);
			mutex_unlock(&fu->mutex);
			return;
		}
	}

	WARN_ON(1);
}

void fetchdecode_layeroffset(struct dpu_fetchunit *fu, unsigned int x,
			     unsigned int y)
{
	u32 val;

	val = LAYERXOFFSET(x) | LAYERYOFFSET(y);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, LAYEROFFSET0, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_layeroffset);

void fetchdecode_clipoffset(struct dpu_fetchunit *fu, unsigned int x,
			    unsigned int y)
{
	u32 val;

	val = CLIPWINDOWXOFFSET(x) | CLIPWINDOWYOFFSET(y);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CLIPWINDOWOFFSET0, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_clipoffset);

static void
fetchdecode_set_pixel_blend_mode(struct dpu_fetchunit *fu,
				 unsigned int pixel_blend_mode, u16 alpha,
				 u32 fb_format)
{
	u32 mode = 0, val;

	if (pixel_blend_mode == DRM_MODE_BLEND_PREMULTI ||
	    pixel_blend_mode == DRM_MODE_BLEND_COVERAGE) {
		mode = ALPHACONSTENABLE;

		switch (fb_format) {
		case DRM_FORMAT_ARGB8888:
		case DRM_FORMAT_ABGR8888:
		case DRM_FORMAT_RGBA8888:
		case DRM_FORMAT_BGRA8888:
			mode |= ALPHASRCENABLE;
			break;
		}
	}

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY0);
	val &= ~(PREMULCONSTRGB | ALPHA_ENABLE_MASK | RGB_ENABLE_MASK);
	val |= mode;
	dpu_fu_write(fu, LAYERPROPERTY0, val);

	val = dpu_fu_read(fu, CONSTANTCOLOR0);
	val &= ~CONSTANTALPHA_MASK;
	val |= CONSTANTALPHA(alpha >> 8);
	dpu_fu_write(fu, CONSTANTCOLOR0, val);
	mutex_unlock(&fu->mutex);
}

static void fetchdecode_enable_src_buf(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY0);
	val |= SOURCEBUFFERENABLE;
	dpu_fu_write(fu, LAYERPROPERTY0, val);
	mutex_unlock(&fu->mutex);
}

static void fetchdecode_disable_src_buf(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY0);
	val &= ~SOURCEBUFFERENABLE;
	dpu_fu_write(fu, LAYERPROPERTY0, val);
	mutex_unlock(&fu->mutex);
}

static bool fetchdecode_is_enabled(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY0);
	mutex_unlock(&fu->mutex);

	return !!(val & SOURCEBUFFERENABLE);
}

void fetchdecode_clipdimensions(struct dpu_fetchunit *fu, unsigned int w,
				unsigned int h)
{
	u32 val;

	val = CLIPWINDOWWIDTH(w) | CLIPWINDOWHEIGHT(h);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CLIPWINDOWDIMENSIONS0, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_clipdimensions);

static void
fetchdecode_set_framedimensions(struct dpu_fetchunit *fu,
				unsigned int w, unsigned int h,
				bool deinterlace)
{
	u32 val;

	if (deinterlace)
		h /= 2;

	val = FRAMEWIDTH(w) | FRAMEHEIGHT(h);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, FRAMEDIMENSIONS, val);
	mutex_unlock(&fu->mutex);
}

void fetchdecode_rgb_constantcolor(struct dpu_fetchunit *fu,
					u8 r, u8 g, u8 b, u8 a)
{
	u32 val;

	val = rgb_color(r, g, b, a);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CONSTANTCOLOR0, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_rgb_constantcolor);

void fetchdecode_yuv_constantcolor(struct dpu_fetchunit *fu, u8 y, u8 u, u8 v)
{
	u32 val;

	val = yuv_color(y, u, v);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CONSTANTCOLOR0, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_yuv_constantcolor);

static void fetchdecode_set_controltrigger(struct dpu_fetchunit *fu)
{
	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CONTROLTRIGGER, SHDTOKGEN);
	mutex_unlock(&fu->mutex);
}

int fetchdecode_fetchtype(struct dpu_fetchunit *fu, fetchtype_t *type)
{
	struct dpu_soc *dpu = fu->dpu;
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, FETCHTYPE);
	val &= FETCHTYPE_MASK;
	mutex_unlock(&fu->mutex);

	switch (val) {
	case FETCHTYPE__DECODE:
	case FETCHTYPE__LAYER:
	case FETCHTYPE__WARP:
	case FETCHTYPE__ECO:
	case FETCHTYPE__PERSP:
	case FETCHTYPE__ROT:
	case FETCHTYPE__DECODEL:
	case FETCHTYPE__LAYERL:
	case FETCHTYPE__ROTL:
		break;
	default:
		dev_warn(dpu->dev, "Invalid fetch type %u for FetchDecode%d\n",
				val, fu->id);
		return -EINVAL;
	}

	*type = val;
	return 0;
}
EXPORT_SYMBOL_GPL(fetchdecode_fetchtype);

u32 fetchdecode_get_vproc_mask(struct dpu_fetchunit *fu)
{
	return fd_vproc_cap[fu->id];
}
EXPORT_SYMBOL_GPL(fetchdecode_get_vproc_mask);

struct dpu_fetchunit *fetchdecode_get_fetcheco(struct dpu_fetchunit *fu)
{
	struct dpu_soc *dpu = fu->dpu;

	switch (fu->id) {
	case 0:
	case 1:
		return dpu->fe_priv[fu->id];
	default:
		WARN_ON(1);
	}

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(fetchdecode_get_fetcheco);

bool fetchdecode_need_fetcheco(struct dpu_fetchunit *fu, u32 fmt)
{
	struct dpu_fetchunit *fe = fetchdecode_get_fetcheco(fu);

	if (IS_ERR_OR_NULL(fe))
		return false;

	switch (fmt) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(fetchdecode_need_fetcheco);

struct dpu_hscaler *fetchdecode_get_hscaler(struct dpu_fetchunit *fu)
{
	struct dpu_soc *dpu = fu->dpu;

	switch (fu->id) {
	case 0:
	case 2:
		return dpu->hs_priv[0];
	case 1:
	case 3:
		return dpu->hs_priv[1];
	default:
		WARN_ON(1);
	}

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(fetchdecode_get_hscaler);

struct dpu_vscaler *fetchdecode_get_vscaler(struct dpu_fetchunit *fu)
{
	struct dpu_soc *dpu = fu->dpu;

	switch (fu->id) {
	case 0:
	case 2:
		return dpu->vs_priv[0];
	case 1:
	case 3:
		return dpu->vs_priv[1];
	default:
		WARN_ON(1);
	}

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(fetchdecode_get_vscaler);

struct dpu_fetchunit *dpu_fd_get(struct dpu_soc *dpu, int id)
{
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(fd_ids); i++)
		if (fd_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fd_ids))
		return ERR_PTR(-EINVAL);

	fu = dpu->fd_priv[i];

	mutex_lock(&fu->mutex);

	if (fu->inuse) {
		mutex_unlock(&fu->mutex);
		return ERR_PTR(-EBUSY);
	}

	fu->inuse = true;

	mutex_unlock(&fu->mutex);

	return fu;
}
EXPORT_SYMBOL_GPL(dpu_fd_get);

void dpu_fd_put(struct dpu_fetchunit *fu)
{
	mutex_lock(&fu->mutex);

	fu->inuse = false;

	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(dpu_fd_put);

static const struct dpu_fetchunit_ops fd_ops = {
	.set_burstlength	= fetchunit_set_burstlength,
	.set_baseaddress	= fetchdecode_set_baseaddress,
	.set_src_bpp		= fetchdecode_set_src_bpp,
	.set_src_stride		= fetchdecode_set_src_stride,
	.set_src_buf_dimensions	= fetchdecode_set_src_buf_dimensions,
	.set_fmt		= fetchdecode_set_fmt,
	.set_pixel_blend_mode	= fetchdecode_set_pixel_blend_mode,
	.enable_src_buf		= fetchdecode_enable_src_buf,
	.disable_src_buf	= fetchdecode_disable_src_buf,
	.is_enabled		= fetchdecode_is_enabled,
	.set_framedimensions	= fetchdecode_set_framedimensions,
	.set_controltrigger	= fetchdecode_set_controltrigger,
	.get_stream_id		= fetchunit_get_stream_id,
	.set_stream_id		= fetchunit_set_stream_id,
};

void _dpu_fd_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(fd_ids); i++)
		if (fd_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(fd_ids)))
		return;

	fu = dpu->fd_priv[i];

	fetchdecode_pixengcfg_dynamic_src_sel(fu, FD_SRC_DISABLE);
	fetchunit_baddr_autoupdate(fu, 0x0);
	fetchunit_shden(fu, true);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, BURSTBUFFERMANAGEMENT,
			SETNUMBUFFERS(16) | SETBURSTLENGTH(16));
	mutex_unlock(&fu->mutex);
}

int dpu_fd_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetchdecode *fd;
	struct dpu_fetchunit *fu;
	int ret;

	fd = devm_kzalloc(dpu->dev, sizeof(*fd), GFP_KERNEL);
	if (!fd)
		return -ENOMEM;

	fu = &fd->fu;
	dpu->fd_priv[id] = fu;

	fu->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!fu->pec_base)
		return -ENOMEM;

	fu->base = devm_ioremap(dpu->dev, base, SZ_1K);
	if (!fu->base)
		return -ENOMEM;

	fu->dpu = dpu;
	fu->id = id;
	fu->type = FU_T_FD;
	fu->ops = &fd_ops;
	fu->name = "fetchdecode";

	mutex_init(&fu->mutex);

	ret = fetchdecode_pixengcfg_dynamic_src_sel(fu, FD_SRC_DISABLE);
	if (ret < 0)
		return ret;

	ret = fetchdecode_fetchtype(fu, &fd->fetchtype);
	if (ret < 0)
		return ret;

	_dpu_fd_init(dpu, id);

	return 0;
}
