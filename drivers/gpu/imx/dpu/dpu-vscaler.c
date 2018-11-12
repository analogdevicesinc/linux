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

#define PIXENGCFG_DYNAMIC		0x8
#define PIXENGCFG_DYNAMIC_SRC_SEL_MASK	0x3F

#define SETUP1				0xC
#define SCALE_FACTOR_MASK		0xFFFFF
#define SCALE_FACTOR(n)			((n) & 0xFFFFF)
#define SETUP2				0x10
#define SETUP3				0x14
#define SETUP4				0x18
#define SETUP5				0x1C
#define PHASE_OFFSET_MASK		0x1FFFFF
#define PHASE_OFFSET(n)			((n) & 0x1FFFFF)
#define CONTROL				0x20
#define OUTPUT_SIZE_MASK		0x3FFF0000
#define OUTPUT_SIZE(n)			((((n) - 1) << 16) & OUTPUT_SIZE_MASK)
#define FIELD_MODE			0x3000
#define FILTER_MODE			0x100
#define SCALE_MODE			0x10
#define MODE				0x1

static const vs_src_sel_t src_sels[3][6] = {
	{
		VS_SRC_SEL__DISABLE,
		VS_SRC_SEL__EXTSRC4,
		VS_SRC_SEL__FETCHDECODE0,
		VS_SRC_SEL__FETCHDECODE2,
		VS_SRC_SEL__MATRIX4,
		VS_SRC_SEL__HSCALER4,
	}, {
		VS_SRC_SEL__DISABLE,
		VS_SRC_SEL__EXTSRC5,
		VS_SRC_SEL__FETCHDECODE1,
		VS_SRC_SEL__FETCHDECODE3,
		VS_SRC_SEL__MATRIX5,
		VS_SRC_SEL__HSCALER5,
	}, {
		VS_SRC_SEL__DISABLE,
		VS_SRC_SEL__MATRIX9,
		VS_SRC_SEL__HSCALER9,
	},
};

struct dpu_vscaler {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
	/* see DPU_PLANE_SRC_xxx */
	unsigned int stream_id;
};

static inline u32 dpu_pec_vs_read(struct dpu_vscaler *vs,
				  unsigned int offset)
{
	return readl(vs->pec_base + offset);
}

static inline void dpu_pec_vs_write(struct dpu_vscaler *vs, u32 value,
				    unsigned int offset)
{
	writel(value, vs->pec_base + offset);
}

static inline u32 dpu_vs_read(struct dpu_vscaler *vs, unsigned int offset)
{
	return readl(vs->base + offset);
}

static inline void dpu_vs_write(struct dpu_vscaler *vs, u32 value,
				unsigned int offset)
{
	writel(value, vs->base + offset);
}

int vscaler_pixengcfg_dynamic_src_sel(struct dpu_vscaler *vs, vs_src_sel_t src)
{
	struct dpu_soc *dpu = vs->dpu;
	const unsigned int *block_id_map = dpu->devtype->sw2hw_block_id_map;
	const unsigned int vs_id_array[] = {4, 5, 9};
	int i, j;
	u32 val, mapped_src;

	for (i = 0; i < ARRAY_SIZE(vs_id_array); i++)
		if (vs_id_array[i] == vs->id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(vs_id_array)))
		return -EINVAL;

	mutex_lock(&vs->mutex);
	for (j = 0; j < ARRAY_SIZE(src_sels[0]); j++) {
		if (src_sels[i][j] == src) {
			mapped_src = block_id_map ? block_id_map[src] : src;
			if (WARN_ON(mapped_src == NA))
				return -EINVAL;

			val = dpu_pec_vs_read(vs, PIXENGCFG_DYNAMIC);
			val &= ~PIXENGCFG_DYNAMIC_SRC_SEL_MASK;
			val |= mapped_src;
			dpu_pec_vs_write(vs, val, PIXENGCFG_DYNAMIC);
			mutex_unlock(&vs->mutex);
			return 0;
		}
	}
	mutex_unlock(&vs->mutex);

	dev_err(dpu->dev, "Invalid source for VScaler%d\n", vs->id);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(vscaler_pixengcfg_dynamic_src_sel);

void vscaler_pixengcfg_clken(struct dpu_vscaler *vs, pixengcfg_clken_t clken)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_pec_vs_read(vs, PIXENGCFG_DYNAMIC);
	val &= ~CLKEN_MASK;
	val |= clken << CLKEN_MASK_SHIFT;
	dpu_pec_vs_write(vs, val, PIXENGCFG_DYNAMIC);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_pixengcfg_clken);

void vscaler_shden(struct dpu_vscaler *vs, bool enable)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_vs_read(vs, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_vs_write(vs, val, STATICCONTROL);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_shden);

void vscaler_setup1(struct dpu_vscaler *vs, u32 src, u32 dst, bool deinterlace)
{
	struct dpu_soc *dpu = vs->dpu;
	u32 scale_factor;
	u64 tmp64;

	if (deinterlace)
		dst *= 2;

	if (src == dst) {
		scale_factor = 0x80000;
	} else {
		if (src > dst) {
			tmp64 = (u64)((u64)dst * 0x80000);
			do_div(tmp64, src);

		} else {
			tmp64 = (u64)((u64)src * 0x80000);
			do_div(tmp64, dst);
		}
		scale_factor = (u32)tmp64;
	}

	WARN_ON(scale_factor > 0x80000);

	mutex_lock(&vs->mutex);
	dpu_vs_write(vs, SCALE_FACTOR(scale_factor), SETUP1);
	mutex_unlock(&vs->mutex);

	dev_dbg(dpu->dev, "Vscaler%d scale factor 0x%08x\n",
						vs->id, scale_factor);
}
EXPORT_SYMBOL_GPL(vscaler_setup1);

void vscaler_setup2(struct dpu_vscaler *vs, bool deinterlace)
{
	/* 0x20000: +0.25 phase offset for deinterlace */
	u32 phase_offset = deinterlace ? 0x20000 : 0;

	mutex_lock(&vs->mutex);
	dpu_vs_write(vs, PHASE_OFFSET(phase_offset), SETUP2);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_setup2);

void vscaler_setup3(struct dpu_vscaler *vs, bool deinterlace)
{
	/* 0x1e0000: -0.25 phase offset for deinterlace */
	u32 phase_offset = deinterlace ? 0x1e0000 : 0;

	mutex_lock(&vs->mutex);
	dpu_vs_write(vs, PHASE_OFFSET(phase_offset), SETUP3);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_setup3);

void vscaler_setup4(struct dpu_vscaler *vs, u32 phase_offset)
{
	mutex_lock(&vs->mutex);
	dpu_vs_write(vs, PHASE_OFFSET(phase_offset), SETUP4);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_setup4);

void vscaler_setup5(struct dpu_vscaler *vs, u32 phase_offset)
{
	mutex_lock(&vs->mutex);
	dpu_vs_write(vs, PHASE_OFFSET(phase_offset), SETUP5);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_setup5);

void vscaler_output_size(struct dpu_vscaler *vs, u32 line_num)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_vs_read(vs, CONTROL);
	val &= ~OUTPUT_SIZE_MASK;
	val |= OUTPUT_SIZE(line_num);
	dpu_vs_write(vs, val, CONTROL);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_output_size);

void vscaler_field_mode(struct dpu_vscaler *vs, scaler_field_mode_t m)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_vs_read(vs, CONTROL);
	val &= ~FIELD_MODE;
	val |= m;
	dpu_vs_write(vs, val, CONTROL);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_field_mode);

void vscaler_filter_mode(struct dpu_vscaler *vs, scaler_filter_mode_t m)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_vs_read(vs, CONTROL);
	val &= ~FILTER_MODE;
	val |= m;
	dpu_vs_write(vs, val, CONTROL);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_filter_mode);

void vscaler_scale_mode(struct dpu_vscaler *vs, scaler_scale_mode_t m)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_vs_read(vs, CONTROL);
	val &= ~SCALE_MODE;
	val |= m;
	dpu_vs_write(vs, val, CONTROL);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_scale_mode);

void vscaler_mode(struct dpu_vscaler *vs, scaler_mode_t m)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_vs_read(vs, CONTROL);
	val &= ~MODE;
	val |= m;
	dpu_vs_write(vs, val, CONTROL);
	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(vscaler_mode);

bool vscaler_is_enabled(struct dpu_vscaler *vs)
{
	u32 val;

	mutex_lock(&vs->mutex);
	val = dpu_vs_read(vs, CONTROL);
	mutex_unlock(&vs->mutex);

	return (val & MODE) == SCALER_ACTIVE;
}
EXPORT_SYMBOL_GPL(vscaler_is_enabled);

dpu_block_id_t vscaler_get_block_id(struct dpu_vscaler *vs)
{
	switch (vs->id) {
	case 4:
		return ID_VSCALER4;
	case 5:
		return ID_VSCALER5;
	case 9:
		return ID_VSCALER9;
	default:
		WARN_ON(1);
	}

	return ID_NONE;
}
EXPORT_SYMBOL_GPL(vscaler_get_block_id);

unsigned int vscaler_get_stream_id(struct dpu_vscaler *vs)
{
	return vs->stream_id;
}
EXPORT_SYMBOL_GPL(vscaler_get_stream_id);

void vscaler_set_stream_id(struct dpu_vscaler *vs, unsigned int id)
{
	switch (id) {
	case DPU_PLANE_SRC_TO_DISP_STREAM0:
	case DPU_PLANE_SRC_TO_DISP_STREAM1:
	case DPU_PLANE_SRC_DISABLED:
		vs->stream_id = id;
		break;
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(vscaler_set_stream_id);

struct dpu_vscaler *dpu_vs_get(struct dpu_soc *dpu, int id)
{
	struct dpu_vscaler *vs;
	int i;

	for (i = 0; i < ARRAY_SIZE(vs_ids); i++)
		if (vs_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(vs_ids))
		return ERR_PTR(-EINVAL);

	vs = dpu->vs_priv[i];

	mutex_lock(&vs->mutex);

	if (vs->inuse) {
		mutex_unlock(&vs->mutex);
		return ERR_PTR(-EBUSY);
	}

	vs->inuse = true;

	mutex_unlock(&vs->mutex);

	return vs;
}
EXPORT_SYMBOL_GPL(dpu_vs_get);

void dpu_vs_put(struct dpu_vscaler *vs)
{
	mutex_lock(&vs->mutex);

	vs->inuse = false;

	mutex_unlock(&vs->mutex);
}
EXPORT_SYMBOL_GPL(dpu_vs_put);

void _dpu_vs_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_vscaler *vs;
	int i;

	for (i = 0; i < ARRAY_SIZE(vs_ids); i++)
		if (vs_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(vs_ids)))
		return;

	vs = dpu->vs_priv[i];

	vscaler_shden(vs, true);
	vscaler_setup2(vs, false);
	vscaler_setup3(vs, false);
	vscaler_setup4(vs, 0);
	vscaler_setup5(vs, 0);
	vscaler_pixengcfg_dynamic_src_sel(vs, VS_SRC_SEL__DISABLE);
}

int dpu_vs_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_vscaler *vs;
	int i;

	vs = devm_kzalloc(dpu->dev, sizeof(*vs), GFP_KERNEL);
	if (!vs)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(vs_ids); i++)
		if (vs_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(vs_ids))
		return -EINVAL;

	dpu->vs_priv[i] = vs;

	vs->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_8);
	if (!vs->pec_base)
		return -ENOMEM;

	vs->base = devm_ioremap(dpu->dev, base, SZ_1K);
	if (!vs->base)
		return -ENOMEM;

	vs->dpu = dpu;
	vs->id = id;

	mutex_init(&vs->mutex);

	_dpu_vs_init(dpu, id);

	return 0;
}
