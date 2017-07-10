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
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include "dpu-prv.h"

#define PIXENGCFG_DYNAMIC			0x8
#define PIXENGCFG_DYNAMIC_PRIM_SEL_MASK		0x3F
#define PIXENGCFG_DYNAMIC_SEC_SEL_MASK		0x3F00
#define PIXENGCFG_DYNAMIC_SEC_SEL_SHIFT		8

static const lb_prim_sel_t prim_sels[] = {
	LB_PRIM_SEL__DISABLE,
	LB_PRIM_SEL__BLITBLEND9,
	LB_PRIM_SEL__CONSTFRAME0,
	LB_PRIM_SEL__CONSTFRAME1,
	LB_PRIM_SEL__CONSTFRAME4,
	LB_PRIM_SEL__CONSTFRAME5,
	LB_PRIM_SEL__MATRIX4,
	LB_PRIM_SEL__HSCALER4,
	LB_PRIM_SEL__VSCALER4,
	LB_PRIM_SEL__EXTSRC4,
	LB_PRIM_SEL__MATRIX5,
	LB_PRIM_SEL__HSCALER5,
	LB_PRIM_SEL__VSCALER5,
	LB_PRIM_SEL__EXTSRC5,
	LB_PRIM_SEL__LAYERBLEND0,
	LB_PRIM_SEL__LAYERBLEND1,
	LB_PRIM_SEL__LAYERBLEND2,
	LB_PRIM_SEL__LAYERBLEND3,
	LB_PRIM_SEL__LAYERBLEND4,
	LB_PRIM_SEL__LAYERBLEND5,
};

#define PIXENGCFG_STATUS			0xC
#define SHDTOKSEL				(0x3 << 3)
#define SHDTOKSEL_SHIFT				3
#define SHDLDSEL				(0x3 << 1)
#define CONTROL					0xC
#define MODE_MASK				BIT(0)
#define BLENDCONTROL				0x10
#define ALPHA(a)				(((a) & 0xFF) << 16)
#define PRIM_C_BLD_FUNC__ONE_MINUS_SEC_ALPHA	0x5
#define SEC_C_BLD_FUNC__CONST_ALPHA		(0x6 << 4)
#define PRIM_A_BLD_FUNC__ONE_MINUS_SEC_ALPHA	(0x5 << 8)
#define SEC_A_BLD_FUNC__ONE			(0x1 << 12)
#define POSITION				0x14
#define XPOS(x)					((x) & 0x7FFF)
#define YPOS(y)					(((y) & 0x7FFF) << 16)
#define PRIMCONTROLWORD				0x18
#define SECCONTROLWORD				0x1C

#define CONTROLWORD				0x18
#define CURPIXELCNT				0x1C
static u16 get_xval(u32 pixel_cnt)
{
	return pixel_cnt && 0xFF;
}

static u16 get_yval(u32 pixel_cnt)
{
	return pixel_cnt >> 16;
}
#define LASTPIXELCNT				0x20
#define PERFCOUNTER				0x24

struct dpu_layerblend {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
};

static inline u32 dpu_pec_lb_read(struct dpu_layerblend *lb,
				  unsigned int offset)
{
	return readl(lb->pec_base + offset);
}

static inline void dpu_pec_lb_write(struct dpu_layerblend *lb, u32 value,
				    unsigned int offset)
{
	writel(value, lb->pec_base + offset);
}

static inline u32 dpu_lb_read(struct dpu_layerblend *lb, unsigned int offset)
{
	return readl(lb->base + offset);
}

static inline void dpu_lb_write(struct dpu_layerblend *lb, u32 value,
				unsigned int offset)
{
	writel(value, lb->base + offset);
}

int layerblend_pixengcfg_dynamic_prim_sel(struct dpu_layerblend *lb,
					  lb_prim_sel_t prim)
{
	struct dpu_soc *dpu = lb->dpu;
	const unsigned int *block_id_map = dpu->devtype->sw2hw_block_id_map;
	int fixed_sels_num = ARRAY_SIZE(prim_sels) - 6;
	int i;
	u32 val, mapped_prim;

	mutex_lock(&lb->mutex);
	for (i = 0; i < fixed_sels_num + lb->id; i++) {
		if (prim_sels[i] == prim) {
			mapped_prim = block_id_map ? block_id_map[prim] : prim;
			if (WARN_ON(mapped_prim == NA))
				return -EINVAL;

			val = dpu_pec_lb_read(lb, PIXENGCFG_DYNAMIC);
			val &= ~PIXENGCFG_DYNAMIC_PRIM_SEL_MASK;
			val |= mapped_prim;
			dpu_pec_lb_write(lb, val, PIXENGCFG_DYNAMIC);
			mutex_unlock(&lb->mutex);
			return 0;
		}
	}
	mutex_unlock(&lb->mutex);

	dev_err(dpu->dev, "Invalid primary source for LayerBlend%d\n", lb->id);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(layerblend_pixengcfg_dynamic_prim_sel);

void layerblend_pixengcfg_dynamic_sec_sel(struct dpu_layerblend *lb,
					  lb_sec_sel_t sec)
{
	struct dpu_soc *dpu = lb->dpu;
	const unsigned int *block_id_map = dpu->devtype->sw2hw_block_id_map;
	u32 val, mapped_sec;

	mapped_sec = block_id_map ? block_id_map[sec] : sec;
	if (WARN_ON(mapped_sec == NA))
		return;

	mutex_lock(&lb->mutex);
	val = dpu_pec_lb_read(lb, PIXENGCFG_DYNAMIC);
	val &= ~PIXENGCFG_DYNAMIC_SEC_SEL_MASK;
	val |= mapped_sec << PIXENGCFG_DYNAMIC_SEC_SEL_SHIFT;
	dpu_pec_lb_write(lb, val, PIXENGCFG_DYNAMIC);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_pixengcfg_dynamic_sec_sel);

void layerblend_pixengcfg_clken(struct dpu_layerblend *lb,
				pixengcfg_clken_t clken)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_pec_lb_read(lb, PIXENGCFG_DYNAMIC);
	val &= ~CLKEN_MASK;
	val |= clken << CLKEN_MASK_SHIFT;
	dpu_pec_lb_write(lb, val, PIXENGCFG_DYNAMIC);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_pixengcfg_clken);

void layerblend_shden(struct dpu_layerblend *lb, bool enable)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_lb_write(lb, val, STATICCONTROL);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_shden);

void layerblend_shdtoksel(struct dpu_layerblend *lb, lb_shadow_sel_t sel)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, STATICCONTROL);
	val &= ~SHDTOKSEL;
	val |= (sel << SHDTOKSEL_SHIFT);
	dpu_lb_write(lb, val, STATICCONTROL);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_shdtoksel);

void layerblend_shdldsel(struct dpu_layerblend *lb, lb_shadow_sel_t sel)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, STATICCONTROL);
	val &= ~SHDLDSEL;
	val |= sel;
	dpu_lb_write(lb, val, STATICCONTROL);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_shdldsel);

void layerblend_control(struct dpu_layerblend *lb, lb_mode_t mode)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, CONTROL);
	val &= ~MODE_MASK;
	val |= mode;
	dpu_lb_write(lb, val, CONTROL);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_control);

void layerblend_blendcontrol(struct dpu_layerblend *lb)
{
	u32 val;

	val = ALPHA(0xff) |
	      PRIM_C_BLD_FUNC__ONE_MINUS_SEC_ALPHA |
	      SEC_C_BLD_FUNC__CONST_ALPHA |
	      PRIM_A_BLD_FUNC__ONE_MINUS_SEC_ALPHA |
	      SEC_A_BLD_FUNC__ONE;

	mutex_lock(&lb->mutex);
	dpu_lb_write(lb, val, BLENDCONTROL);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_blendcontrol);

void layerblend_position(struct dpu_layerblend *lb, int x, int y)
{
	mutex_lock(&lb->mutex);
	dpu_lb_write(lb, XPOS(x) | YPOS(y), POSITION);
	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(layerblend_position);

u32 layerblend_last_control_word(struct dpu_layerblend *lb)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, CONTROLWORD);
	mutex_unlock(&lb->mutex);

	return val;
}
EXPORT_SYMBOL_GPL(layerblend_last_control_word);

void layerblend_pixel_cnt(struct dpu_layerblend *lb, u16 *x, u16 *y)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, CURPIXELCNT);
	mutex_unlock(&lb->mutex);

	*x = get_xval(val);
	*y = get_yval(val);
}
EXPORT_SYMBOL_GPL(layerblend_pixel_cnt);

void layerblend_last_pixel_cnt(struct dpu_layerblend *lb, u16 *x, u16 *y)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, LASTPIXELCNT);
	mutex_unlock(&lb->mutex);

	*x = get_xval(val);
	*y = get_yval(val);
}
EXPORT_SYMBOL_GPL(layerblend_last_pixel_cnt);

u32 layerblend_perfresult(struct dpu_layerblend *lb)
{
	u32 val;

	mutex_lock(&lb->mutex);
	val = dpu_lb_read(lb, PERFCOUNTER);
	mutex_unlock(&lb->mutex);

	return val;
}
EXPORT_SYMBOL_GPL(layerblend_perfresult);

struct dpu_layerblend *dpu_lb_get(struct dpu_soc *dpu, int id)
{
	struct dpu_layerblend *lb;
	int i;

	for (i = 0; i < ARRAY_SIZE(lb_ids); i++)
		if (lb_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(lb_ids))
		return ERR_PTR(-EINVAL);

	lb = dpu->lb_priv[i];

	mutex_lock(&lb->mutex);

	if (lb->inuse) {
		lb = ERR_PTR(-EBUSY);
		goto out;
	}

	lb->inuse = true;
out:
	mutex_unlock(&lb->mutex);

	return lb;
}
EXPORT_SYMBOL_GPL(dpu_lb_get);

void dpu_lb_put(struct dpu_layerblend *lb)
{
	mutex_lock(&lb->mutex);

	lb->inuse = false;

	mutex_unlock(&lb->mutex);
}
EXPORT_SYMBOL_GPL(dpu_lb_put);

int dpu_lb_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_layerblend *lb;
	int ret;

	lb = devm_kzalloc(dpu->dev, sizeof(*lb), GFP_KERNEL);
	if (!lb)
		return -ENOMEM;

	dpu->lb_priv[id] = lb;

	lb->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!lb->pec_base)
		return -ENOMEM;

	lb->base = devm_ioremap(dpu->dev, base, SZ_32);
	if (!lb->base)
		return -ENOMEM;

	lb->dpu = dpu;
	lb->id = id;
	mutex_init(&lb->mutex);

	ret = layerblend_pixengcfg_dynamic_prim_sel(lb, LB_PRIM_SEL__DISABLE);
	if (ret < 0)
		return ret;

	layerblend_pixengcfg_dynamic_sec_sel(lb, LB_SEC_SEL__DISABLE);
	layerblend_pixengcfg_clken(lb, CLKEN__AUTOMATIC);
	layerblend_shdldsel(lb, BOTH);
	layerblend_shdtoksel(lb, BOTH);
	layerblend_shden(lb, true);

	return 0;
}
