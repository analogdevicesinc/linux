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
#include <video/imx8-prefetch.h>
#include "dpu-prv.h"

#define PIXENGCFG_STATUS		0x8
#define BASEADDRESS(n)			(0x10 + (n) * 0x28)
#define SOURCEBUFFERATTRIBUTES(n)	(0x14 + (n) * 0x28)
#define SOURCEBUFFERDIMENSION(n)	(0x18 + (n) * 0x28)
#define COLORCOMPONENTBITS(n)		(0x1C + (n) * 0x28)
#define COLORCOMPONENTSHIFT(n)		(0x20 + (n) * 0x28)
#define LAYEROFFSET(n)			(0x24 + (n) * 0x28)
#define CLIPWINDOWOFFSET(n)		(0x28 + (n) * 0x28)
#define CLIPWINDOWDIMENSIONS(n)		(0x2C + (n) * 0x28)
#define CONSTANTCOLOR(n)		(0x30 + (n) * 0x28)
#define LAYERPROPERTY(n)		(0x34 + (n) * 0x28)
#define FRAMEDIMENSIONS			0x150
#define FRAMERESAMPLING			0x154
#define CONTROL				0x158
#define TRIGGERENABLE			0x15C
#define SHDLDREQ(lm)			((lm) & 0xFF)
#define CONTROLTRIGGER			0x160
#define START				0x164
#define FETCHTYPE			0x168
#define BURSTBUFFERPROPERTIES		0x16C
#define STATUS				0x170
#define HIDDENSTATUS			0x174

static const shadow_load_req_t fl_shdlreqs[] = {
	SHLDREQID_FETCHLAYER0, SHLDREQID_FETCHLAYER1,
};

struct dpu_fetchlayer {
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

static inline u32 dpu_fl_read(struct dpu_fetchlayer *fl, unsigned int offset)
{
	return readl(fl->base + offset);
}

static inline void dpu_fl_write(struct dpu_fetchlayer *fl, u32 value,
				unsigned int offset)
{
	writel(value, fl->base + offset);
}

static inline u32 rgb_color(u8 r, u8 g, u8 b, u8 a)
{
	return (r << 24) | (g << 16) | (b << 8) | a;
}

static inline u32 yuv_color(u8 y, u8 u, u8 v)
{
	return (y << 24) | (u << 16) | (v << 8);
}

void fetchlayer_shden(struct dpu_fetchlayer *fl, bool enable)
{
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_fl_write(fl, val, STATICCONTROL);
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_shden);

void fetchlayer_baddr_autoupdate(struct dpu_fetchlayer *fl, u8 layer_mask)
{
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, STATICCONTROL);
	val &= ~BASEADDRESSAUTOUPDATE_MASK;
	val |= BASEADDRESSAUTOUPDATE(layer_mask);
	dpu_fl_write(fl, val, STATICCONTROL);
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_baddr_autoupdate);

void fetchlayer_shdldreq_sticky(struct dpu_fetchlayer *fl, u8 layer_mask)
{
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, STATICCONTROL);
	val &= ~SHDLDREQSTICKY_MASK;
	val |= SHDLDREQSTICKY(layer_mask);
	dpu_fl_write(fl, val, STATICCONTROL);
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_shdldreq_sticky);

void fetchlayer_set_burstlength(struct dpu_fetchlayer *fl, dma_addr_t baddr,
				bool use_prefetch)
{
	struct dpu_soc *dpu = fl->dpu;
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

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, BURSTBUFFERMANAGEMENT);
	val &= ~SETBURSTLENGTH_MASK;
	val |= SETBURSTLENGTH(burst_length);
	dpu_fl_write(fl, val, BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fl->mutex);

	dev_dbg(dpu->dev, "FetchLayer%d burst length is %u\n",
							fl->id, burst_length);
}
EXPORT_SYMBOL_GPL(fetchlayer_set_burstlength);

void fetchlayer_baseaddress(struct dpu_fetchlayer *fl, unsigned int index,
			    dma_addr_t paddr)
{
	mutex_lock(&fl->mutex);
	dpu_fl_write(fl, paddr, BASEADDRESS(index));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_baseaddress);

void fetchlayer_source_bpp(struct dpu_fetchlayer *fl, unsigned int index,
			   int bpp)
{
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, SOURCEBUFFERATTRIBUTES(index));
	val &= ~0x3f0000;
	val |= BITSPERPIXEL(bpp);
	dpu_fl_write(fl, val, SOURCEBUFFERATTRIBUTES(index));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_source_bpp);

void fetchlayer_source_stride(struct dpu_fetchlayer *fl, unsigned int index,
			      unsigned int width, int bpp, unsigned int stride,
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

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, SOURCEBUFFERATTRIBUTES(index));
	val &= ~0xffff;
	val |= STRIDE(stride);
	dpu_fl_write(fl, val, SOURCEBUFFERATTRIBUTES(index));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_source_stride);

void fetchlayer_src_buf_dimensions(struct dpu_fetchlayer *fl,
				   unsigned int index, unsigned int w,
				   unsigned int h)
{
	u32 val;

	val = LINEWIDTH(w) | LINECOUNT(h);

	mutex_lock(&fl->mutex);
	dpu_fl_write(fl, val, SOURCEBUFFERDIMENSION(index));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_src_buf_dimensions);

void fetchlayer_set_fmt(struct dpu_fetchlayer *fl, unsigned int index, u32 fmt)
{
	u32 val, bits, shift;
	int i;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, LAYERPROPERTY(index));
	val &= ~YUVCONVERSIONMODE_MASK;
	val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE__OFF);
	dpu_fl_write(fl, val, LAYERPROPERTY(index));
	mutex_unlock(&fl->mutex);

	for (i = 0; i < ARRAY_SIZE(dpu_pixel_format_matrix); i++) {
		if (dpu_pixel_format_matrix[i].pixel_format == fmt) {
			bits = dpu_pixel_format_matrix[i].bits;
			shift = dpu_pixel_format_matrix[i].shift;

			mutex_lock(&fl->mutex);
			dpu_fl_write(fl, bits, COLORCOMPONENTBITS(index));
			dpu_fl_write(fl, shift, COLORCOMPONENTSHIFT(index));
			mutex_unlock(&fl->mutex);
			return;
		}
	}

	WARN_ON(1);
}
EXPORT_SYMBOL_GPL(fetchlayer_set_fmt);

void fetchlayer_source_buffer_enable(struct dpu_fetchlayer *fl,
				     unsigned int index)
{
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, LAYERPROPERTY(index));
	val |= SOURCEBUFFERENABLE;
	dpu_fl_write(fl, val, LAYERPROPERTY(index));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_source_buffer_enable);

void fetchlayer_source_buffer_disable(struct dpu_fetchlayer *fl,
				      unsigned int index)
{
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, LAYERPROPERTY(index));
	val &= ~SOURCEBUFFERENABLE;
	dpu_fl_write(fl, val, LAYERPROPERTY(index));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_source_buffer_disable);

bool fetchlayer_is_enabled(struct dpu_fetchlayer *fl, unsigned int index)
{
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, LAYERPROPERTY(index));
	mutex_unlock(&fl->mutex);

	return !!(val & SOURCEBUFFERENABLE);
}
EXPORT_SYMBOL_GPL(fetchlayer_is_enabled);

void fetchlayer_framedimensions(struct dpu_fetchlayer *fl, unsigned int w,
				unsigned int h)
{
	u32 val;

	val = FRAMEWIDTH(w) | FRAMEHEIGHT(h);

	mutex_lock(&fl->mutex);
	dpu_fl_write(fl, val, FRAMEDIMENSIONS);
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_framedimensions);

void fetchlayer_rgb_constantcolor(struct dpu_fetchlayer *fl,
					u8 r, u8 g, u8 b, u8 a)
{
	u32 val;

	val = rgb_color(r, g, b, a);

	mutex_lock(&fl->mutex);
	dpu_fl_write(fl, val, CONSTANTCOLOR(fl->id));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_rgb_constantcolor);

void fetchlayer_yuv_constantcolor(struct dpu_fetchlayer *fl, u8 y, u8 u, u8 v)
{
	u32 val;

	val = yuv_color(y, u, v);

	mutex_lock(&fl->mutex);
	dpu_fl_write(fl, val, CONSTANTCOLOR(fl->id));
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_yuv_constantcolor);

void fetchlayer_controltrigger(struct dpu_fetchlayer *fl, bool trigger)
{
	u32 val;

	val = trigger ? SHDTOKGEN : 0;

	mutex_lock(&fl->mutex);
	dpu_fl_write(fl, val, CONTROLTRIGGER);
	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(fetchlayer_controltrigger);

int fetchlayer_fetchtype(struct dpu_fetchlayer *fl, fetchtype_t *type)
{
	struct dpu_soc *dpu = fl->dpu;
	u32 val;

	mutex_lock(&fl->mutex);
	val = dpu_fl_read(fl, FETCHTYPE);
	val &= FETCHTYPE_MASK;
	mutex_unlock(&fl->mutex);

	switch (val) {
	case FETCHTYPE__DECODE:
		dev_dbg(dpu->dev, "FetchLayer%d with RL and RLAD decoder\n",
				fl->id);
		break;
	case FETCHTYPE__LAYER:
		dev_dbg(dpu->dev, "FetchLayer%d with fractional "
				"plane(8 layers)\n", fl->id);
		break;
	case FETCHTYPE__WARP:
		dev_dbg(dpu->dev, "FetchLayer%d with arbitrary warping and "
				"fractional plane(8 layers)\n", fl->id);
		break;
	case FETCHTYPE__ECO:
		dev_dbg(dpu->dev, "FetchLayer%d with minimum feature set for "
				"alpha, chroma and coordinate planes\n",
				fl->id);
		break;
	case FETCHTYPE__PERSP:
		dev_dbg(dpu->dev, "FetchLayer%d with affine, perspective and "
				"arbitrary warping\n", fl->id);
		break;
	case FETCHTYPE__ROT:
		dev_dbg(dpu->dev, "FetchLayer%d with affine and arbitrary "
				"warping\n", fl->id);
		break;
	case FETCHTYPE__DECODEL:
		dev_dbg(dpu->dev, "FetchLayer%d with RL and RLAD decoder, "
				"reduced feature set\n", fl->id);
		break;
	case FETCHTYPE__LAYERL:
		dev_dbg(dpu->dev, "FetchLayer%d with fractional "
				"plane(8 layers), reduced feature set\n",
				fl->id);
		break;
	case FETCHTYPE__ROTL:
		dev_dbg(dpu->dev, "FetchLayer%d with affine and arbitrary "
				"warping, reduced feature set\n", fl->id);
		break;
	default:
		dev_warn(dpu->dev, "Invalid fetch type %u for FetchLayer%d\n",
				val, fl->id);
		return -EINVAL;
	}

	*type = val;
	return 0;
}
EXPORT_SYMBOL_GPL(fetchlayer_fetchtype);

unsigned int fetchlayer_get_stream_id(struct dpu_fetchlayer *fl)
{
	return fl->stream_id;
}
EXPORT_SYMBOL_GPL(fetchlayer_get_stream_id);

void fetchlayer_set_stream_id(struct dpu_fetchlayer *fl, unsigned int id)
{
	switch (id) {
	case DPU_PLANE_SRC_TO_DISP_STREAM0:
	case DPU_PLANE_SRC_TO_DISP_STREAM1:
	case DPU_PLANE_SRC_DISABLED:
		fl->stream_id = id;
		break;
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(fetchlayer_set_stream_id);

void
fetchlayer_configure_prefetch(struct dpu_fetchlayer *fl, unsigned int stream_id,
			      unsigned int width, unsigned int height,
			      unsigned int x_offset, unsigned int y_offset,
			      unsigned int stride, u32 format, u64 modifier,
			      unsigned long baddr, bool start)
{
	if (WARN_ON(!fl || !fl->dprc))
		return;

	dprc_configure(fl->dprc,
			stream_id, width, height, x_offset, y_offset, stride,
			format, modifier, baddr, 0, start, false);
}
EXPORT_SYMBOL_GPL(fetchlayer_configure_prefetch);

void fetchlayer_enable_prefetch(struct dpu_fetchlayer *fl)
{
	if (WARN_ON(!fl || !fl->dprc))
		return;

	dprc_enable(fl->dprc);
}
EXPORT_SYMBOL_GPL(fetchlayer_enable_prefetch);

void fetchlayer_disable_prefetch(struct dpu_fetchlayer *fl)
{
	if (WARN_ON(!fl || !fl->dprc))
		return;

	dprc_disable(fl->dprc);
}
EXPORT_SYMBOL_GPL(fetchlayer_disable_prefetch);

void fetchlayer_reg_update_prefetch(struct dpu_fetchlayer *fl)
{
	if (WARN_ON(!fl || !fl->dprc))
		return;

	dprc_reg_update(fl->dprc);
}
EXPORT_SYMBOL_GPL(fetchlayer_reg_update_prefetch);

void fetchlayer_prefetch_first_frame_handle(struct dpu_fetchlayer *fl)
{
	if (WARN_ON(!fl || !fl->dprc))
		return;

	dprc_first_frame_handle(fl->dprc);
}
EXPORT_SYMBOL_GPL(fetchlayer_prefetch_first_frame_handle);

void fetchlayer_prefetch_irq_handle(struct dpu_fetchlayer *fl)
{
	if (WARN_ON(!fl || !fl->dprc))
		return;

	dprc_irq_handle(fl->dprc);
}
EXPORT_SYMBOL_GPL(fetchlayer_prefetch_irq_handle);

void fetchlayer_prefetch_enable_first_frame_irq(struct dpu_fetchlayer *fl)
{
	if (WARN_ON(!fl || !fl->dprc))
		return;

	dprc_enable_ctrl_done_irq(fl->dprc);
}
EXPORT_SYMBOL_GPL(fetchlayer_prefetch_enable_first_frame_irq);

bool fetchlayer_has_prefetch(struct dpu_fetchlayer *fl)
{
	return !!fl->dprc;
}
EXPORT_SYMBOL_GPL(fetchlayer_has_prefetch);

bool fetchlayer_prefetch_format_supported(struct dpu_fetchlayer *fl,
					  u32 format, u64 modifier)
{
	if (WARN_ON(!fl || !fl->dprc))
		return false;

	return dprc_format_supported(fl->dprc, format, modifier);
}
EXPORT_SYMBOL_GPL(fetchlayer_prefetch_format_supported);

bool fetchlayer_prefetch_stride_supported(struct dpu_fetchlayer *fl,
					  unsigned int stride,
					  unsigned int width,
					  u32 format)
{
	if (WARN_ON(!fl || !fl->dprc))
		return false;

	return dprc_stride_supported(fl->dprc, stride, 0, width, format);
}
EXPORT_SYMBOL_GPL(fetchlayer_prefetch_stride_supported);

bool fetchlayer_prefetch_stride_double_check(struct dpu_fetchlayer *fl,
					     unsigned int stride,
					     unsigned int width,
					     u32 format,
					     dma_addr_t baseaddr)
{
	if (WARN_ON(!fl || !fl->dprc))
		return false;

	return dprc_stride_double_check(fl->dprc, stride, 0, width, format,
					baseaddr, 0);
}
EXPORT_SYMBOL_GPL(fetchlayer_prefetch_stride_double_check);

void fetchlayer_pin_off(struct dpu_fetchlayer *fl)
{
	fl->pin_off = true;
}
EXPORT_SYMBOL_GPL(fetchlayer_pin_off);

void fetchlayer_unpin_off(struct dpu_fetchlayer *fl)
{
	fl->pin_off = false;
}
EXPORT_SYMBOL_GPL(fetchlayer_unpin_off);

bool fetchlayer_is_pinned_off(struct dpu_fetchlayer *fl)
{
	return fl->pin_off;
}
EXPORT_SYMBOL_GPL(fetchlayer_is_pinned_off);

struct dpu_fetchlayer *dpu_fl_get(struct dpu_soc *dpu, int id)
{
	struct dpu_fetchlayer *fl;
	int i;

	for (i = 0; i < ARRAY_SIZE(fl_ids); i++)
		if (fl_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fl_ids))
		return ERR_PTR(-EINVAL);

	fl = dpu->fl_priv[i];

	mutex_lock(&fl->mutex);

	if (fl->inuse) {
		fl = ERR_PTR(-EBUSY);
		goto out;
	}

	fl->inuse = true;
out:
	mutex_unlock(&fl->mutex);

	return fl;
}
EXPORT_SYMBOL_GPL(dpu_fl_get);

void dpu_fl_put(struct dpu_fetchlayer *fl)
{
	mutex_lock(&fl->mutex);

	fl->inuse = false;

	mutex_unlock(&fl->mutex);
}
EXPORT_SYMBOL_GPL(dpu_fl_put);

void _dpu_fl_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_fetchlayer *fl;
	int i;

	for (i = 0; i < ARRAY_SIZE(fl_ids); i++)
		if (fl_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(fl_ids)))
		return;

	fl = dpu->fl_priv[i];

	fetchlayer_baddr_autoupdate(fl, 0x0);
	fetchlayer_shden(fl, true);
	fetchlayer_shdldreq_sticky(fl, 0xFF);
	for (i = 0; i < DPU_FRAC_PLANE_LAYER_NUM; i++)
		fetchlayer_source_buffer_disable(fl, i);

	mutex_lock(&fl->mutex);
	dpu_fl_write(fl, SETNUMBUFFERS(16) | SETBURSTLENGTH(16),
			BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fl->mutex);
}

int dpu_fl_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetchlayer *fl;
	int ret, i;

	fl = devm_kzalloc(dpu->dev, sizeof(*fl), GFP_KERNEL);
	if (!fl)
		return -ENOMEM;

	dpu->fl_priv[id] = fl;

	fl->pec_base = devm_ioremap(dpu->dev, base, SZ_16);
	if (!fl->pec_base)
		return -ENOMEM;

	fl->base = devm_ioremap(dpu->dev, base, SZ_512);
	if (!fl->base)
		return -ENOMEM;

	fl->dpu = dpu;
	fl->id = id;
	for (i = 0; i < ARRAY_SIZE(fl_ids); i++) {
		if (fl_ids[i] == id) {
			fl->shdlreq = fl_shdlreqs[i];
			break;
		}
	}
	mutex_init(&fl->mutex);

	ret = fetchlayer_fetchtype(fl, &fl->fetchtype);
	if (ret < 0)
		return ret;

	_dpu_fl_init(dpu, id);

	return 0;
}

void fetchlayer_get_dprc(struct dpu_fetchlayer *fl, void *data)
{
	if (WARN_ON(!fl))
		return;

	fl->dprc = data;
}
