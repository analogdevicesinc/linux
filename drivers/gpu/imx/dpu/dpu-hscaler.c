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

#define PIXENGCFG_DYNAMIC		0x8
#define PIXENGCFG_DYNAMIC_SRC_SEL_MASK	0x3F

#define SETUP1				0xC
#define SCALE_FACTOR_MASK		0xFFFFF
#define SCALE_FACTOR(n)			((n) & 0xFFFFF)
#define SETUP2				0x10
#define PHASE_OFFSET_MASK		0x1FFFFF
#define PHASE_OFFSET(n)			((n) & 0x1FFFFF)
#define CONTROL				0x14
#define OUTPUT_SIZE_MASK		0x3FFF0000
#define OUTPUT_SIZE(n)			((((n) - 1) << 16) & OUTPUT_SIZE_MASK)
#define FILTER_MODE			0x100
#define SCALE_MODE			0x10
#define MODE				0x1

static const hs_src_sel_t src_sels[3][6] = {
	{
		HS_SRC_SEL__DISABLE,
		HS_SRC_SEL__EXTSRC4,
		HS_SRC_SEL__FETCHDECODE0,
		HS_SRC_SEL__FETCHDECODE2,
		HS_SRC_SEL__MATRIX4,
		HS_SRC_SEL__VSCALER4,
	}, {
		HS_SRC_SEL__DISABLE,
		HS_SRC_SEL__EXTSRC5,
		HS_SRC_SEL__FETCHDECODE1,
		HS_SRC_SEL__FETCHDECODE3,
		HS_SRC_SEL__MATRIX5,
		HS_SRC_SEL__VSCALER5,
	}, {
		HS_SRC_SEL__DISABLE,
		HS_SRC_SEL__MATRIX9,
		HS_SRC_SEL__VSCALER9,
		HS_SRC_SEL__FILTER9,
	},
};

struct dpu_hscaler {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
	/* see DPU_PLANE_SRC_xxx */
	unsigned int stream_id;
};

static inline u32 dpu_pec_hs_read(struct dpu_hscaler *hs,
				  unsigned int offset)
{
	return readl(hs->pec_base + offset);
}

static inline void dpu_pec_hs_write(struct dpu_hscaler *hs, u32 value,
				    unsigned int offset)
{
	writel(value, hs->pec_base + offset);
}

static inline u32 dpu_hs_read(struct dpu_hscaler *hs, unsigned int offset)
{
	return readl(hs->base + offset);
}

static inline void dpu_hs_write(struct dpu_hscaler *hs, u32 value,
				unsigned int offset)
{
	writel(value, hs->base + offset);
}

int hscaler_pixengcfg_dynamic_src_sel(struct dpu_hscaler *hs, hs_src_sel_t src)
{
	struct dpu_soc *dpu = hs->dpu;
	const unsigned int *block_id_map = dpu->devtype->sw2hw_block_id_map;
	const unsigned int hs_id_array[] = {4, 5, 9};
	int i, j;
	u32 val, mapped_src;

	for (i = 0; i < ARRAY_SIZE(hs_id_array); i++)
		if (hs_id_array[i] == hs->id)
			break;

	if (WARN_ON(i == (ARRAY_SIZE(hs_id_array) + 1)))
		return -EINVAL;

	mutex_lock(&hs->mutex);
	for (j = 0; j < ARRAY_SIZE(src_sels[0]); j++) {
		if (src_sels[i][j] == src) {
			mapped_src = block_id_map ? block_id_map[src] : src;
			if (WARN_ON(mapped_src == NA))
				return -EINVAL;

			val = dpu_pec_hs_read(hs, PIXENGCFG_DYNAMIC);
			val &= ~PIXENGCFG_DYNAMIC_SRC_SEL_MASK;
			val |= mapped_src;
			dpu_pec_hs_write(hs, val, PIXENGCFG_DYNAMIC);
			mutex_unlock(&hs->mutex);
			return 0;
		}
	}
	mutex_unlock(&hs->mutex);

	dev_err(dpu->dev, "Invalid source for HScaler%d\n", hs->id);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(hscaler_pixengcfg_dynamic_src_sel);

void hscaler_pixengcfg_clken(struct dpu_hscaler *hs, pixengcfg_clken_t clken)
{
	u32 val;

	mutex_lock(&hs->mutex);
	val = dpu_pec_hs_read(hs, PIXENGCFG_DYNAMIC);
	val &= ~CLKEN_MASK;
	val |= clken << CLKEN_MASK_SHIFT;
	dpu_pec_hs_write(hs, val, PIXENGCFG_DYNAMIC);
	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(hscaler_pixengcfg_clken);

void hscaler_shden(struct dpu_hscaler *hs, bool enable)
{
	u32 val;

	mutex_lock(&hs->mutex);
	val = dpu_hs_read(hs, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_hs_write(hs, val, STATICCONTROL);
	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(hscaler_shden);

void hscaler_setup1(struct dpu_hscaler *hs, u32 src, u32 dst)
{
	struct dpu_soc *dpu = hs->dpu;
	u32 scale_factor;
	u64 tmp64;

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

	mutex_lock(&hs->mutex);
	dpu_hs_write(hs, SCALE_FACTOR(scale_factor), SETUP1);
	mutex_unlock(&hs->mutex);

	dev_dbg(dpu->dev, "Hscaler%d scale factor 0x%08x\n",
						hs->id, scale_factor);
}
EXPORT_SYMBOL_GPL(hscaler_setup1);

void hscaler_setup2(struct dpu_hscaler *hs, u32 phase_offset)
{
	mutex_lock(&hs->mutex);
	dpu_hs_write(hs, PHASE_OFFSET(phase_offset), SETUP2);
	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(hscaler_setup2);

void hscaler_output_size(struct dpu_hscaler *hs, u32 line_num)
{
	u32 val;

	mutex_lock(&hs->mutex);
	val = dpu_hs_read(hs, CONTROL);
	val &= ~OUTPUT_SIZE_MASK;
	val |= OUTPUT_SIZE(line_num);
	dpu_hs_write(hs, val, CONTROL);
	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(hscaler_output_size);

void hscaler_filter_mode(struct dpu_hscaler *hs, scaler_filter_mode_t m)
{
	u32 val;

	mutex_lock(&hs->mutex);
	val = dpu_hs_read(hs, CONTROL);
	val &= ~FILTER_MODE;
	val |= m;
	dpu_hs_write(hs, val, CONTROL);
	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(hscaler_filter_mode);

void hscaler_scale_mode(struct dpu_hscaler *hs, scaler_scale_mode_t m)
{
	u32 val;

	mutex_lock(&hs->mutex);
	val = dpu_hs_read(hs, CONTROL);
	val &= ~SCALE_MODE;
	val |= m;
	dpu_hs_write(hs, val, CONTROL);
	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(hscaler_scale_mode);

void hscaler_mode(struct dpu_hscaler *hs, scaler_mode_t m)
{
	u32 val;

	mutex_lock(&hs->mutex);
	val = dpu_hs_read(hs, CONTROL);
	val &= ~MODE;
	val |= m;
	dpu_hs_write(hs, val, CONTROL);
	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(hscaler_mode);

bool hscaler_is_enabled(struct dpu_hscaler *hs)
{
	u32 val;

	mutex_lock(&hs->mutex);
	val = dpu_hs_read(hs, CONTROL);
	mutex_unlock(&hs->mutex);

	return (val & MODE) == SCALER_ACTIVE;
}
EXPORT_SYMBOL_GPL(hscaler_is_enabled);

dpu_block_id_t hscaler_get_block_id(struct dpu_hscaler *hs)
{
	switch (hs->id) {
	case 4:
		return ID_HSCALER4;
	case 5:
		return ID_HSCALER5;
	case 9:
		return ID_HSCALER9;
	default:
		WARN_ON(1);
	}

	return ID_NONE;
}
EXPORT_SYMBOL_GPL(hscaler_get_block_id);

unsigned int hscaler_get_stream_id(struct dpu_hscaler *hs)
{
	return hs->stream_id;
}
EXPORT_SYMBOL_GPL(hscaler_get_stream_id);

void hscaler_set_stream_id(struct dpu_hscaler *hs, unsigned int id)
{
	switch (id) {
	case DPU_PLANE_SRC_TO_DISP_STREAM0:
	case DPU_PLANE_SRC_TO_DISP_STREAM1:
	case DPU_PLANE_SRC_DISABLED:
		hs->stream_id = id;
		break;
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(hscaler_set_stream_id);

struct dpu_hscaler *dpu_hs_get(struct dpu_soc *dpu, int id)
{
	struct dpu_hscaler *hs;
	int i;

	for (i = 0; i < ARRAY_SIZE(hs_ids); i++)
		if (hs_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(hs_ids))
		return ERR_PTR(-EINVAL);

	hs = dpu->hs_priv[i];

	mutex_lock(&hs->mutex);

	if (hs->inuse) {
		hs = ERR_PTR(-EBUSY);
		goto out;
	}

	hs->inuse = true;
out:
	mutex_unlock(&hs->mutex);

	return hs;
}
EXPORT_SYMBOL_GPL(dpu_hs_get);

void dpu_hs_put(struct dpu_hscaler *hs)
{
	mutex_lock(&hs->mutex);

	hs->inuse = false;

	mutex_unlock(&hs->mutex);
}
EXPORT_SYMBOL_GPL(dpu_hs_put);

int dpu_hs_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_hscaler *hs;
	int i;

	hs = devm_kzalloc(dpu->dev, sizeof(*hs), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(hs_ids); i++)
		if (hs_ids[i] == id)
			break;

	dpu->hs_priv[i] = hs;

	hs->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_8);
	if (!hs->pec_base)
		return -ENOMEM;

	hs->base = devm_ioremap(dpu->dev, base, SZ_1K);
	if (!hs->base)
		return -ENOMEM;

	hs->dpu = dpu;
	hs->id = id;

	mutex_init(&hs->mutex);

	hscaler_shden(hs, true);
	hscaler_setup2(hs, 0);

	return 0;
}
