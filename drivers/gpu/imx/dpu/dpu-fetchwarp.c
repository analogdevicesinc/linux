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

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
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
	struct dpu_fetchunit fu;
	fetchtype_t fetchtype;
};

static void
fetchwarp_set_src_buf_dimensions(struct dpu_fetchunit *fu,
				 unsigned int w, unsigned int h,
				 u32 unused1, bool unused2)
{
	u32 val;

	val = LINEWIDTH(w) | LINECOUNT(h);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, SOURCEBUFFERDIMENSION(fu->sub_id), val);
	mutex_unlock(&fu->mutex);
}

static void fetchwarp_set_fmt(struct dpu_fetchunit *fu,
			      u32 fmt,
			      enum drm_color_encoding color_encoding,
			      enum drm_color_range color_range,
			      bool unused)
{
	u32 val, bits, shift;
	int i, sub_id = fu->sub_id;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, LAYERPROPERTY(sub_id));
	val &= ~YUVCONVERSIONMODE_MASK;
	dpu_fu_write(fu, LAYERPROPERTY(sub_id), val);
	mutex_unlock(&fu->mutex);

	for (i = 0; i < ARRAY_SIZE(dpu_pixel_format_matrix); i++) {
		if (dpu_pixel_format_matrix[i].pixel_format == fmt) {
			bits = dpu_pixel_format_matrix[i].bits;
			shift = dpu_pixel_format_matrix[i].shift;

			mutex_lock(&fu->mutex);
			dpu_fu_write(fu, COLORCOMPONENTBITS(sub_id), bits);
			dpu_fu_write(fu, COLORCOMPONENTSHIFT(sub_id), shift);
			mutex_unlock(&fu->mutex);
			return;
		}
	}

	WARN_ON(1);
}

static void
fetchwarp_set_framedimensions(struct dpu_fetchunit *fu,
			      unsigned int w, unsigned int h, bool unused)
{
	u32 val;

	val = FRAMEWIDTH(w) | FRAMEHEIGHT(h);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, FRAMEDIMENSIONS, val);
	mutex_unlock(&fu->mutex);
}

void fetchwarp_rgb_constantcolor(struct dpu_fetchunit *fu,
				 u8 r, u8 g, u8 b, u8 a)
{
	u32 val;

	val = rgb_color(r, g, b, a);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CONSTANTCOLOR(fu->id), val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_rgb_constantcolor);

void fetchwarp_yuv_constantcolor(struct dpu_fetchunit *fu, u8 y, u8 u, u8 v)
{
	u32 val;

	val = yuv_color(y, u, v);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CONSTANTCOLOR(fu->id), val);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetchwarp_yuv_constantcolor);

static void fetchwarp_set_controltrigger(struct dpu_fetchunit *fu)
{
	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, CONTROLTRIGGER, SHDTOKGEN);
	mutex_unlock(&fu->mutex);
}

int fetchwarp_fetchtype(struct dpu_fetchunit *fu, fetchtype_t *type)
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
		dev_warn(dpu->dev, "Invalid fetch type %u for FetchWarp%d\n",
				val, fu->id);
		return -EINVAL;
	}

	*type = val;
	return 0;
}
EXPORT_SYMBOL_GPL(fetchwarp_fetchtype);

struct dpu_fetchunit *dpu_fw_get(struct dpu_soc *dpu, int id)
{
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(fw_ids); i++)
		if (fw_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fw_ids))
		return ERR_PTR(-EINVAL);

	fu = dpu->fw_priv[i];

	mutex_lock(&fu->mutex);

	if (fu->inuse) {
		mutex_unlock(&fu->mutex);
		return ERR_PTR(-EBUSY);
	}

	fu->inuse = true;

	mutex_unlock(&fu->mutex);

	return fu;
}
EXPORT_SYMBOL_GPL(dpu_fw_get);

void dpu_fw_put(struct dpu_fetchunit *fu)
{
	mutex_lock(&fu->mutex);

	fu->inuse = false;

	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(dpu_fw_put);

static const struct dpu_fetchunit_ops fw_ops = {
	.set_burstlength	= fetchunit_set_burstlength,
	.set_baseaddress	= fetchunit_set_baseaddress,
	.set_src_bpp		= fetchunit_set_src_bpp,
	.set_src_stride		= fetchunit_set_src_stride,
	.set_src_buf_dimensions	= fetchwarp_set_src_buf_dimensions,
	.set_fmt		= fetchwarp_set_fmt,
	.set_pixel_blend_mode	= fetchunit_set_pixel_blend_mode,
	.enable_src_buf		= fetchunit_enable_src_buf,
	.disable_src_buf	= fetchunit_disable_src_buf,
	.is_enabled		= fetchunit_is_enabled,
	.set_framedimensions	= fetchwarp_set_framedimensions,
	.set_controltrigger	= fetchwarp_set_controltrigger,
	.get_stream_id		= fetchunit_get_stream_id,
	.set_stream_id		= fetchunit_set_stream_id,
};

void _dpu_fw_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(fw_ids); i++)
		if (fw_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(fw_ids)))
		return;

	fu = dpu->fw_priv[i];

	fetchunit_baddr_autoupdate(fu, 0x0);
	fetchunit_shden(fu, true);
	fetchunit_shdldreq_sticky(fu, 0xFF);
	fetchunit_disable_src_buf(fu);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, BURSTBUFFERMANAGEMENT,
			SETNUMBUFFERS(16) | SETBURSTLENGTH(16));
	mutex_unlock(&fu->mutex);
}

int dpu_fw_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetchwarp *fw;
	struct dpu_fetchunit *fu;
	int i, ret;

	fw = devm_kzalloc(dpu->dev, sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(fw_ids); i++)
		if (fw_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fw_ids))
		return -EINVAL;

	fu = &fw->fu;
	dpu->fw_priv[i] = fu;

	fu->pec_base = devm_ioremap(dpu->dev, base, SZ_16);
	if (!fu->pec_base)
		return -ENOMEM;

	fu->base = devm_ioremap(dpu->dev, base, SZ_512);
	if (!fu->base)
		return -ENOMEM;

	fu->dpu = dpu;
	fu->id = id;
	fu->sub_id = 0;
	fu->type = FU_T_FW;
	fu->ops = &fw_ops;
	fu->name = "fetchwarp";

	mutex_init(&fu->mutex);

	ret = fetchwarp_fetchtype(fu, &fw->fetchtype);
	if (ret < 0)
		return ret;

	_dpu_fw_init(dpu, id);

	return 0;
}
