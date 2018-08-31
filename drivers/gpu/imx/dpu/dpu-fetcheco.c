/*
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
	struct dpu_fetchunit fu;
};

static void
fetcheco_set_src_buf_dimensions(struct dpu_fetchunit *fu,
				unsigned int w, unsigned int h,
				u32 fmt, bool deinterlace)
{
	int width, height;
	u32 val;

	if (deinterlace) {
		width = w;
		height = h / 2;
	} else {
		width = dpu_format_plane_width(w, fmt, 1);
		height = dpu_format_plane_height(h, fmt, 1);
	}

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

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, val, SOURCEBUFFERDIMENSION0);
	mutex_unlock(&fu->mutex);
}

static void fetcheco_set_fmt(struct dpu_fetchunit *fu, u32 fmt, bool unused)
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

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, FRAMERESAMPLING);
	val &= ~(DELTAX_MASK | DELTAY_MASK);
	val |= DELTAX(x) | DELTAY(y);
	dpu_fu_write(fu, val, FRAMERESAMPLING);

	val = dpu_fu_read(fu, CONTROL);
	val &= ~RASTERMODE_MASK;
	val |= RASTERMODE(RASTERMODE__NORMAL);
	dpu_fu_write(fu, val, CONTROL);
	mutex_unlock(&fu->mutex);

	for (i = 0; i < ARRAY_SIZE(dpu_pixel_format_matrix); i++) {
		if (dpu_pixel_format_matrix[i].pixel_format == fmt) {
			bits = dpu_pixel_format_matrix[i].bits;
			shift = dpu_pixel_format_matrix[i].shift;

			bits &= ~Y_BITS_MASK;
			shift &= ~Y_SHIFT_MASK;

			mutex_lock(&fu->mutex);
			dpu_fu_write(fu, bits, COLORCOMPONENTBITS0);
			dpu_fu_write(fu, shift, COLORCOMPONENTSHIFT0);
			mutex_unlock(&fu->mutex);
			return;
		}
	}

	WARN_ON(1);
}

void fetcheco_layeroffset(struct dpu_fetchunit *fu, unsigned int x,
			  unsigned int y)
{
	u32 val;

	val = LAYERXOFFSET(x) | LAYERYOFFSET(y);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, val, LAYEROFFSET0);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_layeroffset);

void fetcheco_clipoffset(struct dpu_fetchunit *fu, unsigned int x,
			 unsigned int y)
{
	u32 val;

	val = CLIPWINDOWXOFFSET(x) | CLIPWINDOWYOFFSET(y);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, val, CLIPWINDOWOFFSET0);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_clipoffset);

void fetcheco_clipdimensions(struct dpu_fetchunit *fu, unsigned int w,
			     unsigned int h)
{
	u32 val;

	val = CLIPWINDOWWIDTH(w) | CLIPWINDOWHEIGHT(h);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, val, CLIPWINDOWDIMENSIONS0);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_clipdimensions);

static void
fetcheco_set_framedimensions(struct dpu_fetchunit *fu,
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

void fetcheco_frameresampling(struct dpu_fetchunit *fu, unsigned int x,
			      unsigned int y)
{
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, FRAMERESAMPLING);
	val &= ~(DELTAX_MASK | DELTAY_MASK);
	val |= DELTAX(x) | DELTAY(y);
	dpu_fu_write(fu, val, FRAMERESAMPLING);
	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(fetcheco_frameresampling);

static void fetcheco_set_controltrigger(struct dpu_fetchunit *fu)
{
	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, SHDTOKGEN, CONTROLTRIGGER);
	mutex_unlock(&fu->mutex);
}

int fetcheco_fetchtype(struct dpu_fetchunit *fu, fetchtype_t *type)
{
	struct dpu_soc *dpu = fu->dpu;
	u32 val;

	mutex_lock(&fu->mutex);
	val = dpu_fu_read(fu, FETCHTYPE);
	val &= FETCHTYPE_MASK;
	mutex_unlock(&fu->mutex);

	switch (val) {
	case FETCHTYPE__DECODE:
		dev_dbg(dpu->dev, "FetchEco%d with RL and RLAD decoder\n",
				fu->id);
		break;
	case FETCHTYPE__LAYER:
		dev_dbg(dpu->dev, "FetchEco%d with fractional "
				"plane(8 layers)\n", fu->id);
		break;
	case FETCHTYPE__WARP:
		dev_dbg(dpu->dev, "FetchEco%d with arbitrary warping and "
				"fractional plane(8 layers)\n", fu->id);
		break;
	case FETCHTYPE__ECO:
		dev_dbg(dpu->dev, "FetchEco%d with minimum feature set for "
				"alpha, chroma and coordinate planes\n",
				fu->id);
		break;
	case FETCHTYPE__PERSP:
		dev_dbg(dpu->dev, "FetchEco%d with affine, perspective and "
				"arbitrary warping\n", fu->id);
		break;
	case FETCHTYPE__ROT:
		dev_dbg(dpu->dev, "FetchEco%d with affine and arbitrary "
				"warping\n", fu->id);
		break;
	case FETCHTYPE__DECODEL:
		dev_dbg(dpu->dev, "FetchEco%d with RL and RLAD decoder, "
				"reduced feature set\n", fu->id);
		break;
	case FETCHTYPE__LAYERL:
		dev_dbg(dpu->dev, "FetchEco%d with fractional "
				"plane(8 layers), reduced feature set\n",
				fu->id);
		break;
	case FETCHTYPE__ROTL:
		dev_dbg(dpu->dev, "FetchEco%d with affine and arbitrary "
				"warping, reduced feature set\n", fu->id);
		break;
	default:
		dev_warn(dpu->dev, "Invalid fetch type %u for FetchEco%d\n",
				val, fu->id);
		return -EINVAL;
	}

	*type = val;
	return 0;
}
EXPORT_SYMBOL_GPL(fetcheco_fetchtype);

dpu_block_id_t fetcheco_get_block_id(struct dpu_fetchunit *fu)
{
	switch (fu->id) {
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

struct dpu_fetchunit *dpu_fe_get(struct dpu_soc *dpu, int id)
{
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(fe_ids); i++)
		if (fe_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(fe_ids))
		return ERR_PTR(-EINVAL);

	fu = dpu->fe_priv[i];

	mutex_lock(&fu->mutex);

	if (fu->inuse) {
		mutex_unlock(&fu->mutex);
		return ERR_PTR(-EBUSY);
	}

	fu->inuse = true;

	mutex_unlock(&fu->mutex);

	return fu;
}
EXPORT_SYMBOL_GPL(dpu_fe_get);

void dpu_fe_put(struct dpu_fetchunit *fu)
{
	mutex_lock(&fu->mutex);

	fu->inuse = false;

	mutex_unlock(&fu->mutex);
}
EXPORT_SYMBOL_GPL(dpu_fe_put);

static const struct dpu_fetchunit_ops fe_ops = {
	.set_burstlength	= fetchunit_set_burstlength,
	.set_baseaddress	= fetchunit_set_baseaddress,
	.set_src_bpp		= fetchunit_set_src_bpp,
	.set_src_stride		= fetchunit_set_src_stride,
	.set_src_buf_dimensions	= fetcheco_set_src_buf_dimensions,
	.set_fmt		= fetcheco_set_fmt,
	.enable_src_buf		= fetchunit_enable_src_buf,
	.disable_src_buf	= fetchunit_disable_src_buf,
	.is_enabled		= fetchunit_is_enabled,
	.set_framedimensions	= fetcheco_set_framedimensions,
	.set_controltrigger	= fetcheco_set_controltrigger,
	.get_stream_id		= fetchunit_get_stream_id,
	.set_stream_id		= fetchunit_set_stream_id,
	.pin_off		= fetchunit_pin_off,
	.unpin_off		= fetchunit_unpin_off,
	.is_pinned_off		= fetchunit_is_pinned_off,
};

void _dpu_fe_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(fe_ids); i++)
		if (fe_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(fe_ids)))
		return;

	fu = dpu->fe_priv[i];

	fetchunit_shden(fu, true);

	mutex_lock(&fu->mutex);
	dpu_fu_write(fu, SETNUMBUFFERS(16) | SETBURSTLENGTH(16),
			BURSTBUFFERMANAGEMENT);
	mutex_unlock(&fu->mutex);
}

int dpu_fe_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_fetcheco *fe;
	struct dpu_fetchunit *fu;
	int i;

	fe = devm_kzalloc(dpu->dev, sizeof(*fe), GFP_KERNEL);
	if (!fe)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(fe_ids); i++)
		if (fe_ids[i] == id)
			break;

	fu = &fe->fu;
	dpu->fe_priv[i] = fu;

	fu->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!fu->pec_base)
		return -ENOMEM;

	fu->base = devm_ioremap(dpu->dev, base, SZ_128);
	if (!fu->base)
		return -ENOMEM;

	fu->dpu = dpu;
	fu->id = id;
	fu->type = FU_T_FE;
	fu->ops = &fe_ops;
	fu->name = "fetcheco";

	mutex_init(&fu->mutex);

	_dpu_fe_init(dpu, id);

	return 0;
}
