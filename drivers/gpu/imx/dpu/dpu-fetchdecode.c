/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
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

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include <video/imx8-prefetch.h>
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
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	bool pin_off;
	struct dpu_soc *dpu;
	fetchtype_t fetchtype;
	shadow_load_req_t shdlreq;
	/* see DPU_PLANE_SRC_xxx */
	unsigned int stream_id;
	struct dprc *dprc;
};

static inline u32 dpu_pec_fd_read(struct dpu_fetchdecode *fd,
				  unsigned int offset)
{
	return readl(fd->pec_base + offset);
}

static inline void dpu_pec_fd_write(struct dpu_fetchdecode *fd, u32 value,
				    unsigned int offset)
{
	writel(value, fd->pec_base + offset);
}

static inline u32 dpu_fd_read(struct dpu_fetchdecode *fd, unsigned int offset)
{
	return readl(fd->base + offset);
}

static inline void dpu_fd_write(struct dpu_fetchdecode *fd, u32 value,
				unsigned int offset)
{
	writel(value, fd->base + offset);
}

int fetchdecode_pixengcfg_dynamic_src_sel(struct dpu_fetchdecode *fd,
					  fd_dynamic_src_sel_t src)
{
	struct dpu_soc *dpu = fd->dpu;
	const struct dpu_devtype *devtype = dpu->devtype;
	int i;

	mutex_lock(&fd->mutex);
	if (devtype->version == DPU_V1) {
		for (i = 0; i < SRC_NUM_V1; i++) {
			if (fd_srcs_v1[fd->id][i] == src) {
				dpu_pec_fd_write(fd, src, PIXENGCFG_DYNAMIC);
				mutex_unlock(&fd->mutex);
				return 0;
			}
		}
	} else if (devtype->version == DPU_V2) {
		const unsigned int *block_id_map = devtype->sw2hw_block_id_map;
		u32 mapped_src;

		if (WARN_ON(!block_id_map))
			return -EINVAL;

		for (i = 0; i < SRC_NUM_V2; i++) {
			if (fd_srcs_v2[fd->id][i] == src) {
				mapped_src = block_id_map[src];
				if (WARN_ON(mapped_src == NA))
					return -EINVAL;

				dpu_pec_fd_write(fd, mapped_src,
							PIXENGCFG_DYNAMIC);
				mutex_unlock(&fd->mutex);
				return 0;
			}
		}
	} else {
		WARN_ON(1);
	}
	mutex_unlock(&fd->mutex);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(fetchdecode_pixengcfg_dynamic_src_sel);

static inline u32 rgb_color(u8 r, u8 g, u8 b, u8 a)
{
	return (r << 24) | (g << 16) | (b << 8) | a;
}

static inline u32 yuv_color(u8 y, u8 u, u8 v)
{
	return (y << 24) | (u << 16) | (v << 8);
}

void fetchdecode_shden(struct dpu_fetchdecode *fd, bool enable)
{
	u32 val;

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_fd_write(fd, val, STATICCONTROL);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_shden);

void fetchdecode_baddr_autoupdate(struct dpu_fetchdecode *fd, u8 layer_mask)
{
	u32 val;

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, STATICCONTROL);
	val &= ~BASEADDRESSAUTOUPDATE_MASK;
	val |= BASEADDRESSAUTOUPDATE(layer_mask);
	dpu_fd_write(fd, val, STATICCONTROL);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_baddr_autoupdate);

void fetchdecode_set_burstlength(struct dpu_fetchdecode *fd, dma_addr_t baddr,
				 bool use_prefetch)
{
	struct dpu_soc *dpu = fd->dpu;
	unsigned int burst_size, burst_length;
	u32 val;

	if (use_prefetch) {
		/*
		 * address TKT343664:
		 * fetch unit base address has to align to burst size
		 */
		burst_size = 1 << (ffs(baddr) - 1);
		burst_size = min(burst_size, 128U);
		burst_length = burst_size / 8;
	} else {
		burst_length = 16;
	}

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, BURSTBUFFERMANAGEMENT);
	val &= ~SETBURSTLENGTH_MASK;
	val |= SETBURSTLENGTH(burst_length);
	dpu_fd_write(fd, val, BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fd->mutex);

	dev_dbg(dpu->dev, "FetchDecode%d burst length is %u\n",
							fd->id, burst_length);
}
EXPORT_SYMBOL_GPL(fetchdecode_set_burstlength);

void fetchdecode_baseaddress(struct dpu_fetchdecode *fd, dma_addr_t paddr)
{
	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, paddr, BASEADDRESS0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_baseaddress);

void fetchdecode_source_bpp(struct dpu_fetchdecode *fd, int bpp)
{
	u32 val;

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, SOURCEBUFFERATTRIBUTES0);
	val &= ~0x3f0000;
	val |= BITSPERPIXEL(bpp);
	dpu_fd_write(fd, val, SOURCEBUFFERATTRIBUTES0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_source_bpp);

void fetchdecode_source_stride(struct dpu_fetchdecode *fd, unsigned int width,
			       int bpp, unsigned int stride,
			       dma_addr_t baddr, bool use_prefetch)
{
	unsigned int burst_size;
	u32 val;

	if (use_prefetch) {
		/*
		 * address TKT343664:
		 * fetch unit base address has to align to burst size
		 */
		burst_size = 1 << (ffs(baddr) - 1);
		burst_size = min(burst_size, 128U);

		stride = width * (bpp >> 3);
		/*
		 * address TKT339017:
		 * fixup for burst size vs stride mismatch
		 */
		stride = round_up(stride, burst_size);
	}

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, SOURCEBUFFERATTRIBUTES0);
	val &= ~0xffff;
	val |= STRIDE(stride);
	dpu_fd_write(fd, val, SOURCEBUFFERATTRIBUTES0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_source_stride);

void fetchdecode_src_buf_dimensions(struct dpu_fetchdecode *fd, unsigned int w,
				    unsigned int h)
{
	u32 val;

	val = LINEWIDTH(w) | LINECOUNT(h);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, SOURCEBUFFERDIMENSION0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_src_buf_dimensions);

void fetchdecode_set_fmt(struct dpu_fetchdecode *fd, u32 fmt)
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

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, CONTROL);
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
	dpu_fd_write(fd, val, CONTROL);

	val = dpu_fd_read(fd, LAYERPROPERTY0);
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
	dpu_fd_write(fd, val, LAYERPROPERTY0);
	mutex_unlock(&fd->mutex);

	for (i = 0; i < ARRAY_SIZE(dpu_pixel_format_matrix); i++) {
		if (dpu_pixel_format_matrix[i].pixel_format == fmt) {
			bits = dpu_pixel_format_matrix[i].bits;
			shift = dpu_pixel_format_matrix[i].shift;

			if (is_planar_yuv) {
				bits &= ~(U_BITS_MASK | V_BITS_MASK);
				shift &= ~(U_SHIFT_MASK | V_SHIFT_MASK);
			}

			mutex_lock(&fd->mutex);
			dpu_fd_write(fd, bits, COLORCOMPONENTBITS0);
			dpu_fd_write(fd, shift, COLORCOMPONENTSHIFT0);
			mutex_unlock(&fd->mutex);
			return;
		}
	}

	WARN_ON(1);
}
EXPORT_SYMBOL_GPL(fetchdecode_set_fmt);

void fetchdecode_layeroffset(struct dpu_fetchdecode *fd, unsigned int x,
			     unsigned int y)
{
	u32 val;

	val = LAYERXOFFSET(x) | LAYERYOFFSET(y);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, LAYEROFFSET0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_layeroffset);

void fetchdecode_clipoffset(struct dpu_fetchdecode *fd, unsigned int x,
			    unsigned int y)
{
	u32 val;

	val = CLIPWINDOWXOFFSET(x) | CLIPWINDOWYOFFSET(y);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, CLIPWINDOWOFFSET0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_clipoffset);

void fetchdecode_source_buffer_enable(struct dpu_fetchdecode *fd)
{
	u32 val;

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, LAYERPROPERTY0);
	val |= SOURCEBUFFERENABLE;
	dpu_fd_write(fd, val, LAYERPROPERTY0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_source_buffer_enable);

void fetchdecode_source_buffer_disable(struct dpu_fetchdecode *fd)
{
	u32 val;

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, LAYERPROPERTY0);
	val &= ~SOURCEBUFFERENABLE;
	dpu_fd_write(fd, val, LAYERPROPERTY0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_source_buffer_disable);

bool fetchdecode_is_enabled(struct dpu_fetchdecode *fd)
{
	u32 val;

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, LAYERPROPERTY0);
	mutex_unlock(&fd->mutex);

	return !!(val & SOURCEBUFFERENABLE);
}
EXPORT_SYMBOL_GPL(fetchdecode_is_enabled);

void fetchdecode_clipdimensions(struct dpu_fetchdecode *fd, unsigned int w,
				unsigned int h)
{
	u32 val;

	val = CLIPWINDOWWIDTH(w) | CLIPWINDOWHEIGHT(h);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, CLIPWINDOWDIMENSIONS0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_clipdimensions);

void fetchdecode_framedimensions(struct dpu_fetchdecode *fd, unsigned int w,
				 unsigned int h)
{
	u32 val;

	val = FRAMEWIDTH(w) | FRAMEHEIGHT(h);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, FRAMEDIMENSIONS);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_framedimensions);

void fetchdecode_rgb_constantcolor(struct dpu_fetchdecode *fd,
					u8 r, u8 g, u8 b, u8 a)
{
	u32 val;

	val = rgb_color(r, g, b, a);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, CONSTANTCOLOR0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_rgb_constantcolor);

void fetchdecode_yuv_constantcolor(struct dpu_fetchdecode *fd, u8 y, u8 u, u8 v)
{
	u32 val;

	val = yuv_color(y, u, v);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, CONSTANTCOLOR0);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_yuv_constantcolor);

void fetchdecode_controltrigger(struct dpu_fetchdecode *fd, bool trigger)
{
	u32 val;

	val = trigger ? SHDTOKGEN : 0;

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, val, CONTROLTRIGGER);
	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(fetchdecode_controltrigger);

int fetchdecode_fetchtype(struct dpu_fetchdecode *fd, fetchtype_t *type)
{
	struct dpu_soc *dpu = fd->dpu;
	u32 val;

	mutex_lock(&fd->mutex);
	val = dpu_fd_read(fd, FETCHTYPE);
	val &= FETCHTYPE_MASK;
	mutex_unlock(&fd->mutex);

	switch (val) {
	case FETCHTYPE__DECODE:
		dev_dbg(dpu->dev, "FetchDecode%d with RL and RLAD decoder\n",
				fd->id);
		break;
	case FETCHTYPE__LAYER:
		dev_dbg(dpu->dev, "FetchDecode%d with fractional "
				"plane(8 layers)\n", fd->id);
		break;
	case FETCHTYPE__WARP:
		dev_dbg(dpu->dev, "FetchDecode%d with arbitrary warping and "
				"fractional plane(8 layers)\n", fd->id);
		break;
	case FETCHTYPE__ECO:
		dev_dbg(dpu->dev, "FetchDecode%d with minimum feature set for "
				"alpha, chroma and coordinate planes\n",
				fd->id);
		break;
	case FETCHTYPE__PERSP:
		dev_dbg(dpu->dev, "FetchDecode%d with affine, perspective and "
				"arbitrary warping\n", fd->id);
		break;
	case FETCHTYPE__ROT:
		dev_dbg(dpu->dev, "FetchDecode%d with affine and arbitrary "
				"warping\n", fd->id);
		break;
	case FETCHTYPE__DECODEL:
		dev_dbg(dpu->dev, "FetchDecode%d with RL and RLAD decoder, "
				"reduced feature set\n", fd->id);
		break;
	case FETCHTYPE__LAYERL:
		dev_dbg(dpu->dev, "FetchDecode%d with fractional "
				"plane(8 layers), reduced feature set\n",
				fd->id);
		break;
	case FETCHTYPE__ROTL:
		dev_dbg(dpu->dev, "FetchDecode%d with affine and arbitrary "
				"warping, reduced feature set\n", fd->id);
		break;
	default:
		dev_warn(dpu->dev, "Invalid fetch type %u for FetchDecode%d\n",
				val, fd->id);
		return -EINVAL;
	}

	*type = val;
	return 0;
}
EXPORT_SYMBOL_GPL(fetchdecode_fetchtype);

shadow_load_req_t fetchdecode_to_shdldreq_t(struct dpu_fetchdecode *fd)
{
	shadow_load_req_t t = 0;

	switch (fd->id) {
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

u32 fetchdecode_get_vproc_mask(struct dpu_fetchdecode *fd)
{
	struct dpu_soc *dpu = fd->dpu;
	const struct dpu_devtype *devtype = dpu->devtype;

	return devtype->version == DPU_V1 ?
			fd_vproc_cap_v1[fd->id] : fd_vproc_cap_v2[fd->id];
}
EXPORT_SYMBOL_GPL(fetchdecode_get_vproc_mask);

struct dpu_fetcheco *fetchdecode_get_fetcheco(struct dpu_fetchdecode *fd)
{
	struct dpu_soc *dpu = fd->dpu;

	switch (fd->id) {
	case 0:
	case 1:
		return dpu->fe_priv[fd->id];
	case 2:
	case 3:
		/* TODO: for DPU v1, add FetchEco2 support */
		return dpu->fe_priv[fd->id - 2];
	default:
		WARN_ON(1);
	}

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(fetchdecode_get_fetcheco);

bool fetchdecode_need_fetcheco(struct dpu_fetchdecode *fd, u32 fmt)
{
	struct dpu_fetcheco *fe = fetchdecode_get_fetcheco(fd);

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

struct dpu_hscaler *fetchdecode_get_hscaler(struct dpu_fetchdecode *fd)
{
	struct dpu_soc *dpu = fd->dpu;

	switch (fd->id) {
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

struct dpu_vscaler *fetchdecode_get_vscaler(struct dpu_fetchdecode *fd)
{
	struct dpu_soc *dpu = fd->dpu;

	switch (fd->id) {
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

unsigned int fetchdecode_get_stream_id(struct dpu_fetchdecode *fd)
{
	return fd->stream_id;
}
EXPORT_SYMBOL_GPL(fetchdecode_get_stream_id);

void fetchdecode_set_stream_id(struct dpu_fetchdecode *fd, unsigned int id)
{
	switch (id) {
	case DPU_PLANE_SRC_TO_DISP_STREAM0:
	case DPU_PLANE_SRC_TO_DISP_STREAM1:
	case DPU_PLANE_SRC_DISABLED:
		fd->stream_id = id;
		break;
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(fetchdecode_set_stream_id);

void
fetchdecode_configure_prefetch(struct dpu_fetchdecode *fd,
			       unsigned int stream_id,
			       unsigned int width, unsigned int height,
			       unsigned int x_offset, unsigned int y_offset,
			       unsigned int stride, u32 format, u64 modifier,
			       unsigned long baddr, unsigned long uv_baddr,
			       bool start, bool aux_start)
{
	if (WARN_ON(!fd || !fd->dprc))
		return;

	dprc_configure(fd->dprc,
			stream_id, width, height, x_offset, y_offset, stride,
			format, modifier, baddr, uv_baddr, start, aux_start);
}
EXPORT_SYMBOL_GPL(fetchdecode_configure_prefetch);

void fetchdecode_enable_prefetch(struct dpu_fetchdecode *fd)
{
	if (WARN_ON(!fd || !fd->dprc))
		return;

	dprc_enable(fd->dprc);
}
EXPORT_SYMBOL_GPL(fetchdecode_enable_prefetch);

void fetchdecode_disable_prefetch(struct dpu_fetchdecode *fd)
{
	if (WARN_ON(!fd || !fd->dprc))
		return;

	dprc_disable(fd->dprc);
}
EXPORT_SYMBOL_GPL(fetchdecode_disable_prefetch);

void fetchdecode_reg_update_prefetch(struct dpu_fetchdecode *fd)
{
	if (WARN_ON(!fd || !fd->dprc))
		return;

	dprc_reg_update(fd->dprc);
}
EXPORT_SYMBOL_GPL(fetchdecode_reg_update_prefetch);

void fetchdecode_prefetch_irq_handle(struct dpu_fetchdecode *fd)
{
	if (WARN_ON(!fd || !fd->dprc))
		return;

	dprc_irq_handle(fd->dprc);
}
EXPORT_SYMBOL_GPL(fetchdecode_prefetch_irq_handle);

void fetchdecode_prefetch_enable_first_frame_irq(struct dpu_fetchdecode *fd)
{
	if (WARN_ON(!fd || !fd->dprc))
		return;

	dprc_enable_ctrl_done_irq(fd->dprc);
}
EXPORT_SYMBOL_GPL(fetchdecode_prefetch_enable_first_frame_irq);

bool fetchdecode_has_prefetch(struct dpu_fetchdecode *fd)
{
	return !!fd->dprc;
}
EXPORT_SYMBOL_GPL(fetchdecode_has_prefetch);

bool fetchdecode_prefetch_format_supported(struct dpu_fetchdecode *fd,
					   u32 format, u64 modifier)
{
	if (WARN_ON(!fd || !fd->dprc))
		return false;

	return dprc_format_supported(fd->dprc, format, modifier);
}
EXPORT_SYMBOL_GPL(fetchdecode_prefetch_format_supported);

bool fetchdecode_prefetch_stride_supported(struct dpu_fetchdecode *fd,
					   unsigned int stride,
					   unsigned int uv_stride,
					   unsigned int width,
					   u32 format)
{
	if (WARN_ON(!fd || !fd->dprc))
		return false;

	return dprc_stride_supported(fd->dprc,
					stride, uv_stride, width, format);
}
EXPORT_SYMBOL_GPL(fetchdecode_prefetch_stride_supported);

bool fetchdecode_prefetch_crop_supported(struct dpu_fetchdecode *fd,
					 u64 modifier, u32 y_offset)
{
	if (WARN_ON(!fd || !fd->dprc))
		return false;

	return dprc_crop_supported(fd->dprc, modifier, y_offset);
}
EXPORT_SYMBOL_GPL(fetchdecode_prefetch_crop_supported);

bool fetchdecode_prefetch_stride_double_check(struct dpu_fetchdecode *fd,
					      unsigned int stride,
					      unsigned int uv_stride,
					      unsigned int width,
					      u32 format,
					      dma_addr_t baseaddr,
					      dma_addr_t uv_baseaddr)
{
	if (WARN_ON(!fd || !fd->dprc))
		return false;

	return dprc_stride_double_check(fd->dprc,
					stride, uv_stride, width, format,
					baseaddr, uv_baseaddr);
}
EXPORT_SYMBOL_GPL(fetchdecode_prefetch_stride_double_check);

void fetchdecode_pin_off(struct dpu_fetchdecode *fd)
{
	fd->pin_off = true;
}
EXPORT_SYMBOL_GPL(fetchdecode_pin_off);

void fetchdecode_unpin_off(struct dpu_fetchdecode *fd)
{
	fd->pin_off = false;
}
EXPORT_SYMBOL_GPL(fetchdecode_unpin_off);

bool fetchdecode_is_pinned_off(struct dpu_fetchdecode *fd)
{
	return fd->pin_off;
}
EXPORT_SYMBOL_GPL(fetchdecode_is_pinned_off);

struct dpu_fetchdecode *dpu_fd_get(struct dpu_soc *dpu, int id)
{
	struct dpu_fetchdecode *fd;
	int i;

	for (i = 0; i < ARRAY_SIZE(fd_ids); i++)
		if (fd_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fd_ids))
		return ERR_PTR(-EINVAL);

	fd = dpu->fd_priv[i];

	mutex_lock(&fd->mutex);

	if (fd->inuse) {
		fd = ERR_PTR(-EBUSY);
		goto out;
	}

	fd->inuse = true;
out:
	mutex_unlock(&fd->mutex);

	return fd;
}
EXPORT_SYMBOL_GPL(dpu_fd_get);

void dpu_fd_put(struct dpu_fetchdecode *fd)
{
	mutex_lock(&fd->mutex);

	fd->inuse = false;

	mutex_unlock(&fd->mutex);
}
EXPORT_SYMBOL_GPL(dpu_fd_put);

void _dpu_fd_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_fetchdecode *fd;
	int i;

	for (i = 0; i < ARRAY_SIZE(fd_ids); i++)
		if (fd_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(fd_ids)))
		return;

	fd = dpu->fd_priv[i];

	fetchdecode_pixengcfg_dynamic_src_sel(fd, FD_SRC_DISABLE);
	fetchdecode_baddr_autoupdate(fd, 0x0);
	fetchdecode_shden(fd, true);

	mutex_lock(&fd->mutex);
	dpu_fd_write(fd, SETNUMBUFFERS(16) | SETBURSTLENGTH(16),
			BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fd->mutex);
}

int dpu_fd_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetchdecode *fd;
	int ret, i;

	fd = devm_kzalloc(dpu->dev, sizeof(*fd), GFP_KERNEL);
	if (!fd)
		return -ENOMEM;

	dpu->fd_priv[id] = fd;

	fd->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!fd->pec_base)
		return -ENOMEM;

	fd->base = devm_ioremap(dpu->dev, base, SZ_1K);
	if (!fd->base)
		return -ENOMEM;

	fd->dpu = dpu;
	fd->id = id;
	for (i = 0; i < ARRAY_SIZE(fd_ids); i++) {
		if (fd_ids[i] == id) {
			fd->shdlreq = fd_shdlreqs[i];
			break;
		}
	}
	mutex_init(&fd->mutex);

	ret = fetchdecode_pixengcfg_dynamic_src_sel(fd, FD_SRC_DISABLE);
	if (ret < 0)
		return ret;

	ret = fetchdecode_fetchtype(fd, &fd->fetchtype);
	if (ret < 0)
		return ret;

	_dpu_fd_init(dpu, id);

	return 0;
}

void fetchdecode_get_dprc(struct dpu_fetchdecode *fd, void *data)
{
	if (WARN_ON(!fd))
		return;

	fd->dprc = data;
}
