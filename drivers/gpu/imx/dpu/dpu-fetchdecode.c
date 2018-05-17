/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
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

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include "dpu-prv.h"

#define FD_NUM_V1			4
#define FD_NUM_V2			2

static const u32 fd_vproc_cap_v1[FD_NUM_V1] = {
	DPU_VPROC_CAP_HSCALER4 | DPU_VPROC_CAP_VSCALER4 |
	DPU_VPROC_CAP_FETCHECO0,
	DPU_VPROC_CAP_HSCALER5 | DPU_VPROC_CAP_VSCALER5 |
	DPU_VPROC_CAP_FETCHECO1,
	DPU_VPROC_CAP_HSCALER4 | DPU_VPROC_CAP_VSCALER4 |
	DPU_VPROC_CAP_FETCHECO0,
	DPU_VPROC_CAP_HSCALER5 | DPU_VPROC_CAP_VSCALER5 |
	DPU_VPROC_CAP_FETCHECO1,
};

static const u32 fd_vproc_cap_v2[FD_NUM_V2] = {
	DPU_VPROC_CAP_HSCALER4 | DPU_VPROC_CAP_VSCALER4 |
	DPU_VPROC_CAP_FETCHECO0,
	DPU_VPROC_CAP_HSCALER5 | DPU_VPROC_CAP_VSCALER5 |
	DPU_VPROC_CAP_FETCHECO1,
};

#define PIXENGCFG_DYNAMIC		0x8
#define SRC_NUM_V1			3
#define SRC_NUM_V2			4
static const fd_dynamic_src_sel_t fd_srcs_v1[FD_NUM_V1][SRC_NUM_V1] = {
	{ FD_SRC_DISABLE, FD_SRC_FETCHECO0, FD_SRC_FETCHDECODE2 },
	{ FD_SRC_DISABLE, FD_SRC_FETCHECO1, FD_SRC_FETCHDECODE3 },
	{ FD_SRC_DISABLE, FD_SRC_FETCHECO0, FD_SRC_FETCHECO2 },
	{ FD_SRC_DISABLE, FD_SRC_FETCHECO1, FD_SRC_FETCHECO2 },
};

static const fd_dynamic_src_sel_t fd_srcs_v2[FD_NUM_V2][SRC_NUM_V2] = {
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

static const shadow_load_req_t fd_shdlreqs[] = {
	SHLDREQID_FETCHDECODE0, SHLDREQID_FETCHDECODE1,
	SHLDREQID_FETCHDECODE2, SHLDREQID_FETCHDECODE3,
};

struct dpu_fetchdecode {
	struct dpu_fetchunit fu;
	fetchtype_t fetchtype;
	shadow_load_req_t shdlreq;
};

int fetchdecode_pixengcfg_dynamic_src_sel(struct dpu_fetchunit *fu,
					  fd_dynamic_src_sel_t src)
{
	struct dpu_soc *dpu = fu->dpu;
	const struct dpu_devtype *devtype = dpu->devtype;
	int i;

	mutex_lock(&fu->mutex);
	if (devtype->version == DPU_V1) {
		for (i = 0; i < SRC_NUM_V1; i++) {
			if (fd_srcs_v1[fu->id][i] == src) {
				dpu_pec_fu_write(fu, src, PIXENGCFG_DYNAMIC);
				mutex_unlock(&fu->mutex);
				return 0;
			}
		}
	} else if (devtype->version == DPU_V2) {
		const unsigned int *block_id_map = devtype->sw2hw_block_id_map;
		u32 mapped_src;

		if (WARN_ON(!block_id_map))
			return -EINVAL;

		for (i = 0; i < SRC_NUM_V2; i++) {
			if (fd_srcs_v2[fu->id][i] == src) {
				mapped_src = block_id_map[src];
				if (WARN_ON(mapped_src == NA))
					return -EINVAL;

				dpu_pec_fu_write(fu, mapped_src,
							PIXENGCFG_DYNAMIC);
				mutex_unlock(&fu->mutex);
				return 0;
			}
		}
	} else {
		WARN_ON(1);
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

		/*
		 * address TKT343664:
		 * fetch unit base address has to align to burst size
		 */
		burst_size = 1 << (ffs(baddr) - 1);
		burst_size = round_up(burst_size, 8);
		burst_size = min(burst_size, 128U);

		stride = width * (bpp >> 3);
		/*
		 * address TKT339017:
		 * fixup for burst size vs stride mismatch
		 */
		stride = round_up(stride + round_up(baddr % 8, 8), burst_size);

		/* consider PRG y offset to calculate buffer address */
		baddr += (y_offset % mt_h) * stride;
	}

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, baddr, BASEADDRESS0);
	mutex_unlock(&fu->mutex);
}

static void fetchdecode_set_src_bpp(struct dpu_fetchunit *fu, int bpp)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, SOURCEBUFFERATTRIBUTES0);
	val &= ~0x3f0000;
	val |= BITSPERPIXEL(bpp);
	dpu_fu_write(fu, val, SOURCEBUFFERATTRIBUTES0);
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

		/*
		 * address TKT343664:
		 * fetch unit base address has to align to burst size
		 */
		burst_size = 1 << (ffs(baddr) - 1);
		burst_size = round_up(burst_size, 8);
		burst_size = min(burst_size, 128U);

		stride = width * (bpp >> 3);
		/*
		 * address TKT339017:
		 * fixup for burst size vs stride mismatch
		 */
		if (nonzero_mod)
			stride = round_up(stride + round_up(baddr % 8, 8),
								burst_size);
		else
			stride = round_up(stride, burst_size);
	}

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, SOURCEBUFFERATTRIBUTES0);
	val &= ~0xffff;
	val |= STRIDE(stride);
	dpu_fu_write(fu, val, SOURCEBUFFERATTRIBUTES0);
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
	dpu_fu_write(fu, val, SOURCEBUFFERDIMENSION0);
	mutex_unlock(&fu->mutex);
}

static void
fetchdecode_set_fmt(struct dpu_fetchunit *fu, u32 fmt, bool deinterlace)
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
	dpu_fu_write(fu, val, CONTROL);

	val = dpu_fu_read(fu, LAYERPROPERTY0);
	val &= ~YUVCONVERSIONMODE_MASK;
	if (need_csc)
		/*
		 * assuming fetchdecode always ouputs RGB pixel formats
		 *
		 * FIXME:
		 * determine correct standard here - ITU601 or ITU601_FR
		 * or ITU709
		 */
		val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE__ITU601_FR);
	else
		val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE__OFF);
	dpu_fu_write(fu, val, LAYERPROPERTY0);
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
			dpu_fu_write(fu, bits, COLORCOMPONENTBITS0);
			dpu_fu_write(fu, shift, COLORCOMPONENTSHIFT0);
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
	dpu_fu_write(fu, val, LAYEROFFSET0);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_layeroffset);

void fetchdecode_clipoffset(struct dpu_fetchunit *fu, unsigned int x,
			    unsigned int y)
{
	u32 val;

	val = CLIPWINDOWXOFFSET(x) | CLIPWINDOWYOFFSET(y);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, val, CLIPWINDOWOFFSET0);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_clipoffset);

static void fetchdecode_enable_src_buf(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY0);
	val |= SOURCEBUFFERENABLE;
	dpu_fu_write(fu, val, LAYERPROPERTY0);
	mutex_unlock(&fu->mutex);
}

static void fetchdecode_disable_src_buf(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY0);
	val &= ~SOURCEBUFFERENABLE;
	dpu_fu_write(fu, val, LAYERPROPERTY0);
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
	dpu_fu_write(fu, val, CLIPWINDOWDIMENSIONS0);
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
	dpu_fu_write(fu, val, FRAMEDIMENSIONS);
	mutex_unlock(&fu->mutex);
}

void fetchdecode_rgb_constantcolor(struct dpu_fetchunit *fu,
					u8 r, u8 g, u8 b, u8 a)
{
	u32 val;

	val = rgb_color(r, g, b, a);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, val, CONSTANTCOLOR0);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_rgb_constantcolor);

void fetchdecode_yuv_constantcolor(struct dpu_fetchunit *fu, u8 y, u8 u, u8 v)
{
	u32 val;

	val = yuv_color(y, u, v);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, val, CONSTANTCOLOR0);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_yuv_constantcolor);

static void fetchdecode_set_controltrigger(struct dpu_fetchunit *fu)
{
	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, SHDTOKGEN, CONTROLTRIGGER);
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
		dev_dbg(dpu->dev, "FetchDecode%d with RL and RLAD decoder\n",
				fu->id);
		break;
	case FETCHTYPE__LAYER:
		dev_dbg(dpu->dev, "FetchDecode%d with fractional "
				"plane(8 layers)\n", fu->id);
		break;
	case FETCHTYPE__WARP:
		dev_dbg(dpu->dev, "FetchDecode%d with arbitrary warping and "
				"fractional plane(8 layers)\n", fu->id);
		break;
	case FETCHTYPE__ECO:
		dev_dbg(dpu->dev, "FetchDecode%d with minimum feature set for "
				"alpha, chroma and coordinate planes\n",
				fu->id);
		break;
	case FETCHTYPE__PERSP:
		dev_dbg(dpu->dev, "FetchDecode%d with affine, perspective and "
				"arbitrary warping\n", fu->id);
		break;
	case FETCHTYPE__ROT:
		dev_dbg(dpu->dev, "FetchDecode%d with affine and arbitrary "
				"warping\n", fu->id);
		break;
	case FETCHTYPE__DECODEL:
		dev_dbg(dpu->dev, "FetchDecode%d with RL and RLAD decoder, "
				"reduced feature set\n", fu->id);
		break;
	case FETCHTYPE__LAYERL:
		dev_dbg(dpu->dev, "FetchDecode%d with fractional "
				"plane(8 layers), reduced feature set\n",
				fu->id);
		break;
	case FETCHTYPE__ROTL:
		dev_dbg(dpu->dev, "FetchDecode%d with affine and arbitrary "
				"warping, reduced feature set\n", fu->id);
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

shadow_load_req_t fetchdecode_to_shdldreq_t(struct dpu_fetchunit *fu)
{
	shadow_load_req_t t = 0;

	switch (fu->id) {
	case 0:
		t = SHLDREQID_FETCHDECODE0;
		break;
	case 1:
		t = SHLDREQID_FETCHDECODE1;
		break;
	case 2:
		t = SHLDREQID_FETCHDECODE2;
		break;
	case 3:
		t = SHLDREQID_FETCHDECODE3;
		break;
	default:
		break;
	}

	return t;
}
EXPORT_SYMBOL_GPL(fetchdecode_to_shdldreq_t);

u32 fetchdecode_get_vproc_mask(struct dpu_fetchunit *fu)
{
	struct dpu_soc *dpu = fu->dpu;
	const struct dpu_devtype *devtype = dpu->devtype;

	return devtype->version == DPU_V1 ?
			fd_vproc_cap_v1[fu->id] : fd_vproc_cap_v2[fu->id];
}
EXPORT_SYMBOL_GPL(fetchdecode_get_vproc_mask);

struct dpu_fetchunit *fetchdecode_get_fetcheco(struct dpu_fetchunit *fu)
{
	struct dpu_soc *dpu = fu->dpu;

	switch (fu->id) {
	case 0:
	case 1:
		return dpu->fe_priv[fu->id];
	case 2:
	case 3:
		/* TODO: for DPU v1, add FetchEco2 support */
		return dpu->fe_priv[fu->id - 2];
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
		fu = ERR_PTR(-EBUSY);
		goto out;
	}

	fu->inuse = true;
out:
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
	.enable_src_buf		= fetchdecode_enable_src_buf,
	.disable_src_buf	= fetchdecode_disable_src_buf,
	.is_enabled		= fetchdecode_is_enabled,
	.set_framedimensions	= fetchdecode_set_framedimensions,
	.set_controltrigger	= fetchdecode_set_controltrigger,
	.get_stream_id		= fetchunit_get_stream_id,
	.set_stream_id		= fetchunit_set_stream_id,
	.pin_off		= fetchunit_pin_off,
	.unpin_off		= fetchunit_unpin_off,
	.is_pinned_off		= fetchunit_is_pinned_off,
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
	dpu_fu_write(fu, SETNUMBUFFERS(16) | SETBURSTLENGTH(16),
			BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fu->mutex);
}

int dpu_fd_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetchdecode *fd;
	struct dpu_fetchunit *fu;
	int ret, i;

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
	for (i = 0; i < ARRAY_SIZE(fd_ids); i++) {
		if (fd_ids[i] == id) {
			fd->shdlreq = fd_shdlreqs[i];
			break;
		}
	}
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
