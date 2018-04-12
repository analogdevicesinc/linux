/*
 * Copyright 2018 NXP
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
#define WARPCONTROL			0x158
#define ARBSTARTX			0x15c
#define ARBSTARTY			0x160
#define ARBDELTA			0x164
#define FIRPOSITIONS			0x168
#define FIRCOEFFICIENTS			0x16c
#define CONTROL				0x170
#define TRIGGERENABLE			0x174
#define SHDLDREQ(lm)			((lm) & 0xFF)
#define CONTROLTRIGGER			0x178
#define START				0x17c
#define FETCHTYPE			0x180
#define BURSTBUFFERPROPERTIES		0x184
#define STATUS				0x188
#define HIDDENSTATUS			0x18c

struct dpu_fetchwarp {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	bool pin_off;
	struct dpu_soc *dpu;
	fetchtype_t fetchtype;
	/* see DPU_PLANE_SRC_xxx */
	unsigned int stream_id;
	struct dprc *dprc;
};

static inline u32 dpu_fw_read(struct dpu_fetchwarp *fw, unsigned int offset)
{
	return readl(fw->base + offset);
}

static inline void dpu_fw_write(struct dpu_fetchwarp *fw, u32 value,
				unsigned int offset)
{
	writel(value, fw->base + offset);
}

static inline u32 rgb_color(u8 r, u8 g, u8 b, u8 a)
{
	return (r << 24) | (g << 16) | (b << 8) | a;
}

static inline u32 yuv_color(u8 y, u8 u, u8 v)
{
	return (y << 24) | (u << 16) | (v << 8);
}

void fetchwarp_shden(struct dpu_fetchwarp *fw, bool enable)
{
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_fw_write(fw, val, STATICCONTROL);
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_shden);

void fetchwarp_baddr_autoupdate(struct dpu_fetchwarp *fw, u8 layer_mask)
{
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, STATICCONTROL);
	val &= ~BASEADDRESSAUTOUPDATE_MASK;
	val |= BASEADDRESSAUTOUPDATE(layer_mask);
	dpu_fw_write(fw, val, STATICCONTROL);
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_baddr_autoupdate);

void fetchwarp_shdldreq_sticky(struct dpu_fetchwarp *fw, u8 layer_mask)
{
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, STATICCONTROL);
	val &= ~SHDLDREQSTICKY_MASK;
	val |= SHDLDREQSTICKY(layer_mask);
	dpu_fw_write(fw, val, STATICCONTROL);
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_shdldreq_sticky);

void fetchwarp_set_burstlength(struct dpu_fetchwarp *fw, dma_addr_t baddr,
			       bool use_prefetch)
{
	struct dpu_soc *dpu = fw->dpu;
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

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, BURSTBUFFERMANAGEMENT);
	val &= ~SETBURSTLENGTH_MASK;
	val |= SETBURSTLENGTH(burst_length);
	dpu_fw_write(fw, val, BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fw->mutex);

	dev_dbg(dpu->dev, "FetchWarp%d burst length is %u\n",
							fw->id, burst_length);
}
EXPORT_SYMBOL_GPL(fetchwarp_set_burstlength);

void fetchwarp_baseaddress(struct dpu_fetchwarp *fw, unsigned int index,
			   dma_addr_t paddr)
{
	mutex_lock(&fw->mutex);
	dpu_fw_write(fw, paddr, BASEADDRESS(index));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_baseaddress);

void fetchwarp_source_bpp(struct dpu_fetchwarp *fw, unsigned int index,
			  int bpp)
{
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, SOURCEBUFFERATTRIBUTES(index));
	val &= ~0x3f0000;
	val |= BITSPERPIXEL(bpp);
	dpu_fw_write(fw, val, SOURCEBUFFERATTRIBUTES(index));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_source_bpp);

void fetchwarp_source_stride(struct dpu_fetchwarp *fw, unsigned int index,
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

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, SOURCEBUFFERATTRIBUTES(index));
	val &= ~0xffff;
	val |= STRIDE(stride);
	dpu_fw_write(fw, val, SOURCEBUFFERATTRIBUTES(index));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_source_stride);

void fetchwarp_src_buf_dimensions(struct dpu_fetchwarp *fw,
				  unsigned int index, unsigned int w,
				  unsigned int h)
{
	u32 val;

	val = LINEWIDTH(w) | LINECOUNT(h);

	mutex_lock(&fw->mutex);
	dpu_fw_write(fw, val, SOURCEBUFFERDIMENSION(index));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_src_buf_dimensions);

void fetchwarp_set_fmt(struct dpu_fetchwarp *fw, unsigned int index, u32 fmt)
{
	u32 val, bits, shift;
	int i;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, LAYERPROPERTY(index));
	val &= ~YUVCONVERSIONMODE_MASK;
	dpu_fw_write(fw, val, LAYERPROPERTY(index));
	mutex_unlock(&fw->mutex);

	for (i = 0; i < ARRAY_SIZE(dpu_pixel_format_matrix); i++) {
		if (dpu_pixel_format_matrix[i].pixel_format == fmt) {
			bits = dpu_pixel_format_matrix[i].bits;
			shift = dpu_pixel_format_matrix[i].shift;

			mutex_lock(&fw->mutex);
			dpu_fw_write(fw, bits, COLORCOMPONENTBITS(index));
			dpu_fw_write(fw, shift, COLORCOMPONENTSHIFT(index));
			mutex_unlock(&fw->mutex);
			return;
		}
	}

	WARN_ON(1);
}
EXPORT_SYMBOL_GPL(fetchwarp_set_fmt);

void fetchwarp_source_buffer_enable(struct dpu_fetchwarp *fw,
				    unsigned int index)
{
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, LAYERPROPERTY(index));
	val |= SOURCEBUFFERENABLE;
	dpu_fw_write(fw, val, LAYERPROPERTY(index));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_source_buffer_enable);

void fetchwarp_source_buffer_disable(struct dpu_fetchwarp *fw,
				     unsigned int index)
{
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, LAYERPROPERTY(index));
	val &= ~SOURCEBUFFERENABLE;
	dpu_fw_write(fw, val, LAYERPROPERTY(index));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_source_buffer_disable);

bool fetchwarp_is_enabled(struct dpu_fetchwarp *fw, unsigned int index)
{
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, LAYERPROPERTY(index));
	mutex_unlock(&fw->mutex);

	return !!(val & SOURCEBUFFERENABLE);
}
EXPORT_SYMBOL_GPL(fetchwarp_is_enabled);

void fetchwarp_framedimensions(struct dpu_fetchwarp *fw, unsigned int w,
			       unsigned int h)
{
	u32 val;

	val = FRAMEWIDTH(w) | FRAMEHEIGHT(h);

	mutex_lock(&fw->mutex);
	dpu_fw_write(fw, val, FRAMEDIMENSIONS);
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_framedimensions);

void fetchwarp_rgb_constantcolor(struct dpu_fetchwarp *fw,
				 u8 r, u8 g, u8 b, u8 a)
{
	u32 val;

	val = rgb_color(r, g, b, a);

	mutex_lock(&fw->mutex);
	dpu_fw_write(fw, val, CONSTANTCOLOR(fw->id));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_rgb_constantcolor);

void fetchwarp_yuv_constantcolor(struct dpu_fetchwarp *fw, u8 y, u8 u, u8 v)
{
	u32 val;

	val = yuv_color(y, u, v);

	mutex_lock(&fw->mutex);
	dpu_fw_write(fw, val, CONSTANTCOLOR(fw->id));
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_yuv_constantcolor);

void fetchwarp_controltrigger(struct dpu_fetchwarp *fw, bool trigger)
{
	u32 val;

	val = trigger ? SHDTOKGEN : 0;

	mutex_lock(&fw->mutex);
	dpu_fw_write(fw, val, CONTROLTRIGGER);
	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_controltrigger);

int fetchwarp_fetchtype(struct dpu_fetchwarp *fw, fetchtype_t *type)
{
	struct dpu_soc *dpu = fw->dpu;
	u32 val;

	mutex_lock(&fw->mutex);
	val = dpu_fw_read(fw, FETCHTYPE);
	val &= FETCHTYPE_MASK;
	mutex_unlock(&fw->mutex);

	switch (val) {
	case FETCHTYPE__DECODE:
		dev_dbg(dpu->dev, "FetchWarp%d with RL and RLAD decoder\n",
				fw->id);
		break;
	case FETCHTYPE__LAYER:
		dev_dbg(dpu->dev, "FetchWarp%d with fractional "
				"plane(8 layers)\n", fw->id);
		break;
	case FETCHTYPE__WARP:
		dev_dbg(dpu->dev, "FetchWarp%d with arbitrary warping and "
				"fractional plane(8 layers)\n", fw->id);
		break;
	case FETCHTYPE__ECO:
		dev_dbg(dpu->dev, "FetchWarp%d with minimum feature set for "
				"alpha, chroma and coordinate planes\n",
				fw->id);
		break;
	case FETCHTYPE__PERSP:
		dev_dbg(dpu->dev, "FetchWarp%d with affine, perspective and "
				"arbitrary warping\n", fw->id);
		break;
	case FETCHTYPE__ROT:
		dev_dbg(dpu->dev, "FetchWarp%d with affine and arbitrary "
				"warping\n", fw->id);
		break;
	case FETCHTYPE__DECODEL:
		dev_dbg(dpu->dev, "FetchWarp%d with RL and RLAD decoder, "
				"reduced feature set\n", fw->id);
		break;
	case FETCHTYPE__LAYERL:
		dev_dbg(dpu->dev, "FetchWarp%d with fractional "
				"plane(8 layers), reduced feature set\n",
				fw->id);
		break;
	case FETCHTYPE__ROTL:
		dev_dbg(dpu->dev, "FetchWarp%d with affine and arbitrary "
				"warping, reduced feature set\n", fw->id);
		break;
	default:
		dev_warn(dpu->dev, "Invalid fetch type %u for FetchWarp%d\n",
				val, fw->id);
		return -EINVAL;
	}

	*type = val;
	return 0;
}
EXPORT_SYMBOL_GPL(fetchwarp_fetchtype);

unsigned int fetchwarp_get_stream_id(struct dpu_fetchwarp *fw)
{
	return fw->stream_id;
}
EXPORT_SYMBOL_GPL(fetchwarp_get_stream_id);

void fetchwarp_set_stream_id(struct dpu_fetchwarp *fw, unsigned int id)
{
	switch (id) {
	case DPU_PLANE_SRC_TO_DISP_STREAM0:
	case DPU_PLANE_SRC_TO_DISP_STREAM1:
	case DPU_PLANE_SRC_DISABLED:
		fw->stream_id = id;
		break;
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(fetchwarp_set_stream_id);

void
fetchwarp_configure_prefetch(struct dpu_fetchwarp *fw, unsigned int stream_id,
			     unsigned int width, unsigned int height,
			     unsigned int x_offset, unsigned int y_offset,
			     unsigned int stride, u32 format, u64 modifier,
			     unsigned long baddr, bool start)
{
	if (WARN_ON(!fw || !fw->dprc))
		return;

	dprc_configure(fw->dprc,
			stream_id, width, height, x_offset, y_offset, stride,
			format, modifier, baddr, 0, start, false, false);
}
EXPORT_SYMBOL_GPL(fetchwarp_configure_prefetch);

void fetchwarp_enable_prefetch(struct dpu_fetchwarp *fw)
{
	if (WARN_ON(!fw || !fw->dprc))
		return;

	dprc_enable(fw->dprc);
}
EXPORT_SYMBOL_GPL(fetchwarp_enable_prefetch);

void fetchwarp_disable_prefetch(struct dpu_fetchwarp *fw)
{
	if (WARN_ON(!fw || !fw->dprc))
		return;

	dprc_disable(fw->dprc);
}
EXPORT_SYMBOL_GPL(fetchwarp_disable_prefetch);

void fetchwarp_reg_update_prefetch(struct dpu_fetchwarp *fw)
{
	if (WARN_ON(!fw || !fw->dprc))
		return;

	dprc_reg_update(fw->dprc);
}
EXPORT_SYMBOL_GPL(fetchwarp_reg_update_prefetch);

void fetchwarp_prefetch_first_frame_handle(struct dpu_fetchwarp *fw)
{
	if (WARN_ON(!fw || !fw->dprc))
		return;

	dprc_first_frame_handle(fw->dprc);
}
EXPORT_SYMBOL_GPL(fetchwarp_prefetch_first_frame_handle);

void fetchwarp_prefetch_irq_handle(struct dpu_fetchwarp *fw)
{
	if (WARN_ON(!fw || !fw->dprc))
		return;

	dprc_irq_handle(fw->dprc);
}
EXPORT_SYMBOL_GPL(fetchwarp_prefetch_irq_handle);

void fetchwarp_prefetch_enable_first_frame_irq(struct dpu_fetchwarp *fw)
{
	if (WARN_ON(!fw || !fw->dprc))
		return;

	dprc_enable_ctrl_done_irq(fw->dprc);
}
EXPORT_SYMBOL_GPL(fetchwarp_prefetch_enable_first_frame_irq);

bool fetchwarp_has_prefetch(struct dpu_fetchwarp *fw)
{
	return !!fw->dprc;
}
EXPORT_SYMBOL_GPL(fetchwarp_has_prefetch);

bool fetchwarp_prefetch_format_supported(struct dpu_fetchwarp *fw,
					 u32 format, u64 modifier)
{
	if (WARN_ON(!fw || !fw->dprc))
		return false;

	return dprc_format_supported(fw->dprc, format, modifier);
}
EXPORT_SYMBOL_GPL(fetchwarp_prefetch_format_supported);

bool fetchwarp_prefetch_stride_supported(struct dpu_fetchwarp *fw,
					 unsigned int stride,
					 unsigned int width,
					 u32 format)
{
	if (WARN_ON(!fw || !fw->dprc))
		return false;

	return dprc_stride_supported(fw->dprc, stride, 0, width, format);
}
EXPORT_SYMBOL_GPL(fetchwarp_prefetch_stride_supported);

bool fetchwarp_prefetch_stride_double_check(struct dpu_fetchwarp *fw,
					    unsigned int stride,
					    unsigned int width,
					    u32 format,
					    dma_addr_t baseaddr)
{
	if (WARN_ON(!fw || !fw->dprc))
		return false;

	return dprc_stride_double_check(fw->dprc, stride, 0, width, format,
					baseaddr, 0);
}
EXPORT_SYMBOL_GPL(fetchwarp_prefetch_stride_double_check);

void fetchwarp_pin_off(struct dpu_fetchwarp *fw)
{
	fw->pin_off = true;
}
EXPORT_SYMBOL_GPL(fetchwarp_pin_off);

void fetchwarp_unpin_off(struct dpu_fetchwarp *fw)
{
	fw->pin_off = false;
}
EXPORT_SYMBOL_GPL(fetchwarp_unpin_off);

bool fetchwarp_is_pinned_off(struct dpu_fetchwarp *fw)
{
	return fw->pin_off;
}
EXPORT_SYMBOL_GPL(fetchwarp_is_pinned_off);

struct dpu_fetchwarp *dpu_fw_get(struct dpu_soc *dpu, int id)
{
	struct dpu_fetchwarp *fw;
	int i;

	for (i = 0; i < ARRAY_SIZE(fw_ids); i++)
		if (fw_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fw_ids))
		return ERR_PTR(-EINVAL);

	fw = dpu->fw_priv[i];

	mutex_lock(&fw->mutex);

	if (fw->inuse) {
		fw = ERR_PTR(-EBUSY);
		goto out;
	}

	fw->inuse = true;
out:
	mutex_unlock(&fw->mutex);

	return fw;
}
EXPORT_SYMBOL_GPL(dpu_fw_get);

void dpu_fw_put(struct dpu_fetchwarp *fw)
{
	mutex_lock(&fw->mutex);

	fw->inuse = false;

	mutex_unlock(&fw->mutex);
}
EXPORT_SYMBOL_GPL(dpu_fw_put);

void _dpu_fw_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_fetchwarp *fw;
	int i;

	for (i = 0; i < ARRAY_SIZE(fw_ids); i++)
		if (fw_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(fw_ids)))
		return;

	fw = dpu->fw_priv[i];

	fetchwarp_baddr_autoupdate(fw, 0x0);
	fetchwarp_shden(fw, true);
	fetchwarp_shdldreq_sticky(fw, 0xFF);
	for (i = 0; i < DPU_FRAC_PLANE_LAYER_NUM; i++)
		fetchwarp_source_buffer_disable(fw, i);

	mutex_lock(&fw->mutex);
	dpu_fw_write(fw, SETNUMBUFFERS(16) | SETBURSTLENGTH(16),
			BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fw->mutex);
}

int dpu_fw_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetchwarp *fw;
	int i, ret;

	fw = devm_kzalloc(dpu->dev, sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(fw_ids); i++)
		if (fw_ids[i] == id)
			break;

	dpu->fw_priv[i] = fw;

	fw->pec_base = devm_ioremap(dpu->dev, base, SZ_16);
	if (!fw->pec_base)
		return -ENOMEM;

	fw->base = devm_ioremap(dpu->dev, base, SZ_512);
	if (!fw->base)
		return -ENOMEM;

	fw->dpu = dpu;
	fw->id = id;

	mutex_init(&fw->mutex);

	ret = fetchwarp_fetchtype(fw, &fw->fetchtype);
	if (ret < 0)
		return ret;

	_dpu_fw_init(dpu, id);

	return 0;
}

void fetchwarp_get_dprc(struct dpu_fetchwarp *fw, void *data)
{
	if (WARN_ON(!fw))
		return;

	fw->dprc = data;
}
