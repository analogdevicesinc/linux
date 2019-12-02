/*
 * Copyright 2018-2019 NXP
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
#include <video/dpu.h>
#include "dpu-prv.h"

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

/* base address has to align to burst size */
unsigned int fetchunit_burst_size_fixup_tkt343664(dma_addr_t baddr)
{
	unsigned int burst_size;

	burst_size = 1 << (ffs(baddr) - 1);
	burst_size = round_up(burst_size, 8);
	burst_size = min(burst_size, 128U);

	return burst_size;
}
EXPORT_SYMBOL_GPL(fetchunit_burst_size_fixup_tkt343664);

/* fixup for burst size vs stride mismatch */
unsigned int
fetchunit_stride_fixup_tkt339017(unsigned int stride, unsigned int burst_size,
				 dma_addr_t baddr, bool nonzero_mod)
{
	if (nonzero_mod)
		stride = round_up(stride + round_up(baddr % 8, 8), burst_size);
	else
		stride = round_up(stride, burst_size);

	return stride;
}
EXPORT_SYMBOL_GPL(fetchunit_stride_fixup_tkt339017);

void fetchunit_get_dprc(struct dpu_fetchunit *fu, void *data)
{
	if (WARN_ON(!fu))
		return;

	fu->dprc = data;
}
EXPORT_SYMBOL_GPL(fetchunit_get_dprc);

void fetchunit_shden(struct dpu_fetchunit *fu, bool enable)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_fu_write(fu, STATICCONTROL, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_shden);

void fetchunit_baddr_autoupdate(struct dpu_fetchunit *fu, u8 layer_mask)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, STATICCONTROL);
	val &= ~BASEADDRESSAUTOUPDATE_MASK;
	val |= BASEADDRESSAUTOUPDATE(layer_mask);
	dpu_fu_write(fu, STATICCONTROL, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_baddr_autoupdate);

void fetchunit_shdldreq_sticky(struct dpu_fetchunit *fu, u8 layer_mask)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, STATICCONTROL);
	val &= ~SHDLDREQSTICKY_MASK;
	val |= SHDLDREQSTICKY(layer_mask);
	dpu_fu_write(fu, STATICCONTROL, val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_shdldreq_sticky);

void fetchunit_set_burstlength(struct dpu_fetchunit *fu,
			       unsigned int x_offset, unsigned int mt_w,
			       int bpp, dma_addr_t baddr, bool use_prefetch)
{
	struct dpu_soc *dpu = fu->dpu;
	unsigned int burst_size, burst_length;
	bool nonzero_mod = !!mt_w;
	u32 val;

	if (use_prefetch) {
		/* consider PRG x offset to calculate buffer address */
		if (nonzero_mod)
			baddr += (x_offset % mt_w) * (bpp / 8);

		burst_size = fetchunit_burst_size_fixup_tkt343664(baddr);
		burst_length = burst_size / 8;
	} else {
		burst_length = 16;
	}

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, BURSTBUFFERMANAGEMENT);
	val &= ~SETBURSTLENGTH_MASK;
	val |= SETBURSTLENGTH(burst_length);
	dpu_fu_write(fu, BURSTBUFFERMANAGEMENT, val);
	mutex_unlock(&fu->mutex);

	dev_dbg(dpu->dev, "%s%d burst length is %u\n",
					fu->name, fu->id, burst_length);
}
EXPORT_SYMBOL_GPL(fetchunit_set_burstlength);

void fetchunit_set_baseaddress(struct dpu_fetchunit *fu, unsigned int width,
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
	dpu_fu_write(fu, BASEADDRESS(fu->sub_id), baddr);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_set_baseaddress);

void fetchunit_set_src_bpp(struct dpu_fetchunit *fu, int bpp)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, SOURCEBUFFERATTRIBUTES(fu->sub_id));
	val &= ~0x3f0000;
	val |= BITSPERPIXEL(bpp);
	dpu_fu_write(fu, SOURCEBUFFERATTRIBUTES(fu->sub_id), val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_set_src_bpp);

/*
 * The arguments width and bpp are valid only when use_prefetch is true.
 * For fetcheco, since the pixel format has to be NV12 or NV21 when
 * use_prefetch is true, we assume width stands for how many UV we have
 * in bytes for one line, while bpp should be 8bits for every U or V component.
 */
void fetchunit_set_src_stride(struct dpu_fetchunit *fu,
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
	val = dpu_fu_read(fu, SOURCEBUFFERATTRIBUTES(fu->sub_id));
	val &= ~0xffff;
	val |= STRIDE(stride);
	dpu_fu_write(fu, SOURCEBUFFERATTRIBUTES(fu->sub_id), val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_set_src_stride);

void fetchunit_set_pixel_blend_mode(struct dpu_fetchunit *fu,
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
	val = dpu_fu_read(fu, LAYERPROPERTY(fu->sub_id));
	val &= ~(PREMULCONSTRGB | ALPHA_ENABLE_MASK | RGB_ENABLE_MASK);
	val |= mode;
	dpu_fu_write(fu, LAYERPROPERTY(fu->sub_id), val);

	val = dpu_fu_read(fu, CONSTANTCOLOR(fu->sub_id));
	val &= ~CONSTANTALPHA_MASK;
	val |= CONSTANTALPHA(alpha >> 8);
	dpu_fu_write(fu, CONSTANTCOLOR(fu->sub_id), val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_set_pixel_blend_mode);

void fetchunit_enable_src_buf(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY(fu->sub_id));
	val |= SOURCEBUFFERENABLE;
	dpu_fu_write(fu, LAYERPROPERTY(fu->sub_id), val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_enable_src_buf);

void fetchunit_disable_src_buf(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY(fu->sub_id));
	val &= ~SOURCEBUFFERENABLE;
	dpu_fu_write(fu, LAYERPROPERTY(fu->sub_id), val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchunit_disable_src_buf);

bool fetchunit_is_enabled(struct dpu_fetchunit *fu)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY(fu->sub_id));
	mutex_unlock(&fu->mutex);

	return !!(val & SOURCEBUFFERENABLE);
}
EXPORT_SYMBOL_GPL(fetchunit_is_enabled);

unsigned int fetchunit_get_stream_id(struct dpu_fetchunit *fu)
{
	if (WARN_ON(!fu))
		return DPU_PLANE_SRC_DISABLED;

	return fu->stream_id;
}
EXPORT_SYMBOL_GPL(fetchunit_get_stream_id);

void fetchunit_set_stream_id(struct dpu_fetchunit *fu, unsigned int id)
{
	if (WARN_ON(!fu))
		return;

	switch (id) {
	case DPU_PLANE_SRC_TO_DISP_STREAM0:
	case DPU_PLANE_SRC_TO_DISP_STREAM1:
	case DPU_PLANE_SRC_DISABLED:
		fu->stream_id = id;
		break;
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(fetchunit_set_stream_id);

bool fetchunit_is_fetchdecode(struct dpu_fetchunit *fu)
{
	if (WARN_ON(!fu))
		return false;

	return fu->type == FU_T_FD;
}
EXPORT_SYMBOL_GPL(fetchunit_is_fetchdecode);

bool fetchunit_is_fetcheco(struct dpu_fetchunit *fu)
{
	if (WARN_ON(!fu))
		return false;

	return fu->type == FU_T_FE;
}
EXPORT_SYMBOL_GPL(fetchunit_is_fetcheco);

bool fetchunit_is_fetchlayer(struct dpu_fetchunit *fu)
{
	if (WARN_ON(!fu))
		return false;

	return fu->type == FU_T_FL;
}
EXPORT_SYMBOL_GPL(fetchunit_is_fetchlayer);

bool fetchunit_is_fetchwarp(struct dpu_fetchunit *fu)
{
	if (WARN_ON(!fu))
		return false;

	return fu->type == FU_T_FW;
}
EXPORT_SYMBOL_GPL(fetchunit_is_fetchwarp);
