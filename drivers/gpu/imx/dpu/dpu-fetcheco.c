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

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include "dpu-prv.h"

#define BASEADDRESS0			0x10
#define SOURCEBUFFERATTRIBUTES0		0x14
#define SOURCEBUFFERDIMENSION0		0x18
#define COLORCOMPONENTBITS0		0x1C
#define COLORCOMPONENTSHIFT0		0x20
#define LAYEROFFSET0			0x24
#define CLIPWINDOWOFFSET0		0x28
#define CLIPWINDOWDIMENSIONS0		0x2C
#define CONSTANTCOLOR0			0x30
#define LAYERPROPERTY0			0x34
#define FRAMEDIMENSIONS			0x38
#define FRAMERESAMPLING			0x3C
#define CONTROL				0x40
#define CONTROLTRIGGER			0x44
#define START				0x48
#define FETCHTYPE			0x4C
#define BURSTBUFFERPROPERTIES		0x50
#define HIDDENSTATUS			0x54

struct dpu_fetcheco {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	bool pin_off;
	struct dpu_soc *dpu;
	/* see DPU_PLANE_SRC_xxx */
	unsigned int stream_id;
};

static inline u32 dpu_pec_fe_read(struct dpu_fetcheco *fe, unsigned int offset)
{
	return readl(fe->pec_base + offset);
}

static inline void dpu_pec_fe_write(struct dpu_fetcheco *fe, u32 value,
				    unsigned int offset)
{
	writel(value, fe->pec_base + offset);
}

static inline u32 dpu_fe_read(struct dpu_fetcheco *fe, unsigned int offset)
{
	return readl(fe->base + offset);
}

static inline void dpu_fe_write(struct dpu_fetcheco *fe, u32 value,
				unsigned int offset)
{
	writel(value, fe->base + offset);
}

void fetcheco_shden(struct dpu_fetcheco *fe, bool enable)
{
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_fe_write(fe, val, STATICCONTROL);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_shden);

void fetcheco_set_burstlength(struct dpu_fetcheco *fe, dma_addr_t baddr,
			      bool use_prefetch)
{
	struct dpu_soc *dpu = fe->dpu;
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

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, BURSTBUFFERMANAGEMENT);
	val &= ~SETBURSTLENGTH_MASK;
	val |= SETBURSTLENGTH(burst_length);
	dpu_fe_write(fe, val, BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fe->mutex);

	dev_dbg(dpu->dev, "FetchEco%d burst length is %u\n",
						fe->id, burst_length);
}
EXPORT_SYMBOL_GPL(fetcheco_set_burstlength);

void fetcheco_baseaddress(struct dpu_fetcheco *fe, dma_addr_t paddr)
{
	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, paddr, BASEADDRESS0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_baseaddress);

void fetcheco_source_bpp(struct dpu_fetcheco *fe, int bpp)
{
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, SOURCEBUFFERATTRIBUTES0);
	val &= ~0x3f0000;
	val |= BITSPERPIXEL(bpp);
	dpu_fe_write(fe, val, SOURCEBUFFERATTRIBUTES0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_source_bpp);

void fetcheco_source_stride(struct dpu_fetcheco *fe, int stride)
{
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, SOURCEBUFFERATTRIBUTES0);
	val &= ~0xffff;
	val |= STRIDE(stride);
	dpu_fe_write(fe, val, SOURCEBUFFERATTRIBUTES0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_source_stride);

void fetcheco_src_buf_dimensions(struct dpu_fetcheco *fe, unsigned int w,
				 unsigned int h, u32 fmt)
{
	int width = dpu_format_plane_width(w, fmt, 1);
	int height = dpu_format_plane_height(h, fmt, 1);
	u32 val;

	switch (fmt) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		break;
	default:
		WARN(1, "Unsupported FetchEco pixel format 0x%08x\n", fmt);
		return;
	}

	val = LINEWIDTH(width) | LINECOUNT(height);

	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, val, SOURCEBUFFERDIMENSION0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_src_buf_dimensions);

void fetcheco_set_fmt(struct dpu_fetcheco *fe, u32 fmt)
{
	u32 val, bits, shift;
	int i, hsub, vsub;
	unsigned int x, y;

	switch (fmt) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		break;
	default:
		WARN(1, "Unsupported FetchEco pixel format 0x%08x\n", fmt);
		return;
	}

	hsub = dpu_format_horz_chroma_subsampling(fmt);
	switch (hsub) {
	case 1:
		x = 0x4;
		break;
	case 2:
		x = 0x2;
		break;
	default:
		WARN_ON(1);
		return;
	}

	vsub = dpu_format_vert_chroma_subsampling(fmt);
	switch (vsub) {
	case 1:
		y = 0x4;
		break;
	case 2:
		y = 0x2;
		break;
	default:
		WARN_ON(1);
		return;
	}

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, FRAMERESAMPLING);
	val &= ~(DELTAX_MASK | DELTAY_MASK);
	val |= DELTAX(x) | DELTAY(y);
	dpu_fe_write(fe, val, FRAMERESAMPLING);

	val = dpu_fe_read(fe, CONTROL);
	val &= ~RASTERMODE_MASK;
	val |= RASTERMODE(RASTERMODE__NORMAL);
	dpu_fe_write(fe, val, CONTROL);
	mutex_unlock(&fe->mutex);

	for (i = 0; i < ARRAY_SIZE(dpu_pixel_format_matrix); i++) {
		if (dpu_pixel_format_matrix[i].pixel_format == fmt) {
			bits = dpu_pixel_format_matrix[i].bits;
			shift = dpu_pixel_format_matrix[i].shift;

			bits &= ~Y_BITS_MASK;
			shift &= ~Y_SHIFT_MASK;

			mutex_lock(&fe->mutex);
			dpu_fe_write(fe, bits, COLORCOMPONENTBITS0);
			dpu_fe_write(fe, shift, COLORCOMPONENTSHIFT0);
			mutex_unlock(&fe->mutex);
			return;
		}
	}

	WARN_ON(1);
}
EXPORT_SYMBOL_GPL(fetcheco_set_fmt);

void fetcheco_layeroffset(struct dpu_fetcheco *fe, unsigned int x,
			  unsigned int y)
{
	u32 val;

	val = LAYERXOFFSET(x) | LAYERYOFFSET(y);

	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, val, LAYEROFFSET0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_layeroffset);

void fetcheco_clipoffset(struct dpu_fetcheco *fe, unsigned int x,
			 unsigned int y)
{
	u32 val;

	val = CLIPWINDOWXOFFSET(x) | CLIPWINDOWYOFFSET(y);

	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, val, CLIPWINDOWOFFSET0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_clipoffset);

void fetcheco_clipdimensions(struct dpu_fetcheco *fe, unsigned int w,
			     unsigned int h)
{
	u32 val;

	val = CLIPWINDOWWIDTH(w) | CLIPWINDOWHEIGHT(h);

	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, val, CLIPWINDOWDIMENSIONS0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_clipdimensions);

void fetcheco_source_buffer_enable(struct dpu_fetcheco *fe)
{
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, LAYERPROPERTY0);
	val |= SOURCEBUFFERENABLE;
	dpu_fe_write(fe, val, LAYERPROPERTY0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_source_buffer_enable);

void fetcheco_source_buffer_disable(struct dpu_fetcheco *fe)
{
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, LAYERPROPERTY0);
	val &= ~SOURCEBUFFERENABLE;
	dpu_fe_write(fe, val, LAYERPROPERTY0);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_source_buffer_disable);

bool fetcheco_is_enabled(struct dpu_fetcheco *fe)
{
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, LAYERPROPERTY0);
	mutex_unlock(&fe->mutex);

	return !!(val & SOURCEBUFFERENABLE);
}
EXPORT_SYMBOL_GPL(fetcheco_is_enabled);

void fetcheco_framedimensions(struct dpu_fetcheco *fe, unsigned int w,
			      unsigned int h)
{
	u32 val;

	val = FRAMEWIDTH(w) | FRAMEHEIGHT(h);

	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, val, FRAMEDIMENSIONS);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_framedimensions);

void fetcheco_frameresampling(struct dpu_fetcheco *fe, unsigned int x,
			      unsigned int y)
{
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, FRAMERESAMPLING);
	val &= ~(DELTAX_MASK | DELTAY_MASK);
	val |= DELTAX(x) | DELTAY(y);
	dpu_fe_write(fe, val, FRAMERESAMPLING);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_frameresampling);

void fetcheco_controltrigger(struct dpu_fetcheco *fe, bool trigger)
{
	u32 val;

	val = trigger ? SHDTOKGEN : 0;

	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, val, CONTROLTRIGGER);
	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_controltrigger);

int fetcheco_fetchtype(struct dpu_fetcheco *fe, fetchtype_t *type)
{
	struct dpu_soc *dpu = fe->dpu;
	u32 val;

	mutex_lock(&fe->mutex);
	val = dpu_fe_read(fe, FETCHTYPE);
	val &= FETCHTYPE_MASK;
	mutex_unlock(&fe->mutex);

	switch (val) {
	case FETCHTYPE__DECODE:
		dev_dbg(dpu->dev, "FetchEco%d with RL and RLAD decoder\n",
				fe->id);
		break;
	case FETCHTYPE__LAYER:
		dev_dbg(dpu->dev, "FetchEco%d with fractional "
				"plane(8 layers)\n", fe->id);
		break;
	case FETCHTYPE__WARP:
		dev_dbg(dpu->dev, "FetchEco%d with arbitrary warping and "
				"fractional plane(8 layers)\n", fe->id);
		break;
	case FETCHTYPE__ECO:
		dev_dbg(dpu->dev, "FetchEco%d with minimum feature set for "
				"alpha, chroma and coordinate planes\n",
				fe->id);
		break;
	case FETCHTYPE__PERSP:
		dev_dbg(dpu->dev, "FetchEco%d with affine, perspective and "
				"arbitrary warping\n", fe->id);
		break;
	case FETCHTYPE__ROT:
		dev_dbg(dpu->dev, "FetchEco%d with affine and arbitrary "
				"warping\n", fe->id);
		break;
	case FETCHTYPE__DECODEL:
		dev_dbg(dpu->dev, "FetchEco%d with RL and RLAD decoder, "
				"reduced feature set\n", fe->id);
		break;
	case FETCHTYPE__LAYERL:
		dev_dbg(dpu->dev, "FetchEco%d with fractional "
				"plane(8 layers), reduced feature set\n",
				fe->id);
		break;
	case FETCHTYPE__ROTL:
		dev_dbg(dpu->dev, "FetchEco%d with affine and arbitrary "
				"warping, reduced feature set\n", fe->id);
		break;
	default:
		dev_warn(dpu->dev, "Invalid fetch type %u for FetchEco%d\n",
				val, fe->id);
		return -EINVAL;
	}

	*type = val;
	return 0;
}
EXPORT_SYMBOL_GPL(fetcheco_fetchtype);

dpu_block_id_t fetcheco_get_block_id(struct dpu_fetcheco *fe)
{
	switch (fe->id) {
	case 0:
		return ID_FETCHECO0;
	case 1:
		return ID_FETCHECO1;
	case 2:
		return ID_FETCHECO2;
	case 9:
		return ID_FETCHECO9;
	default:
		WARN_ON(1);
	}

	return ID_NONE;
}
EXPORT_SYMBOL_GPL(fetcheco_get_block_id);

unsigned int fetcheco_get_stream_id(struct dpu_fetcheco *fe)
{
	return fe->stream_id;
}
EXPORT_SYMBOL_GPL(fetcheco_get_stream_id);

void fetcheco_set_stream_id(struct dpu_fetcheco *fe, unsigned int id)
{
	switch (id) {
	case DPU_PLANE_SRC_TO_DISP_STREAM0:
	case DPU_PLANE_SRC_TO_DISP_STREAM1:
	case DPU_PLANE_SRC_DISABLED:
		fe->stream_id = id;
		break;
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(fetcheco_set_stream_id);

void fetcheco_pin_off(struct dpu_fetcheco *fe)
{
	fe->pin_off = true;
}
EXPORT_SYMBOL_GPL(fetcheco_pin_off);

void fetcheco_unpin_off(struct dpu_fetcheco *fe)
{
	fe->pin_off = false;
}
EXPORT_SYMBOL_GPL(fetcheco_unpin_off);

bool fetcheco_is_pinned_off(struct dpu_fetcheco *fe)
{
	return fe->pin_off;
}
EXPORT_SYMBOL_GPL(fetcheco_is_pinned_off);

struct dpu_fetcheco *dpu_fe_get(struct dpu_soc *dpu, int id)
{
	struct dpu_fetcheco *fe;
	int i;

	for (i = 0; i < ARRAY_SIZE(fe_ids); i++)
		if (fe_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fe_ids))
		return ERR_PTR(-EINVAL);

	fe = dpu->fe_priv[i];

	mutex_lock(&fe->mutex);

	if (fe->inuse) {
		fe = ERR_PTR(-EBUSY);
		goto out;
	}

	fe->inuse = true;
out:
	mutex_unlock(&fe->mutex);

	return fe;
}
EXPORT_SYMBOL_GPL(dpu_fe_get);

void dpu_fe_put(struct dpu_fetcheco *fe)
{
	mutex_lock(&fe->mutex);

	fe->inuse = false;

	mutex_unlock(&fe->mutex);
}
EXPORT_SYMBOL_GPL(dpu_fe_put);

void _dpu_fe_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_fetcheco *fe;
	int i;

	for (i = 0; i < ARRAY_SIZE(fe_ids); i++)
		if (fe_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(fe_ids)))
		return;

	fe = dpu->fe_priv[i];

	fetcheco_shden(fe, true);

	mutex_lock(&fe->mutex);
	dpu_fe_write(fe, SETNUMBUFFERS(16) | SETBURSTLENGTH(16),
			BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fe->mutex);
}

int dpu_fe_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetcheco *fe;
	int i;

	fe = devm_kzalloc(dpu->dev, sizeof(*fe), GFP_KERNEL);
	if (!fe)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(fe_ids); i++)
		if (fe_ids[i] == id)
			break;

	dpu->fe_priv[i] = fe;

	fe->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!fe->pec_base)
		return -ENOMEM;

	fe->base = devm_ioremap(dpu->dev, base, SZ_128);
	if (!fe->base)
		return -ENOMEM;

	fe->dpu = dpu;
	fe->id = id;
	mutex_init(&fe->mutex);

	_dpu_fe_init(dpu, id);

	return 0;
}
