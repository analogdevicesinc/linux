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
#include "dpu-prv.h"

#define PIXENGCFG_STATIC		0x8
#define POWERDOWN			BIT(4)
#define SYNC_MODE			BIT(8)
#define SW_RESET			BIT(11)
#define DIV(n)				(((n) & 0xFF) << 16)
#define DIV_RESET			0x80
#define PIXENGCFG_DYNAMIC		0xC
#define PIXENGCFG_REQUEST		0x10
#define SHDLDREQ(n)			BIT(n)
#define SEL_SHDLDREQ			BIT(0)
#define PIXENGCFG_TRIGGER		0x14
#define SYNC_TRIGGER			BIT(0)
#define TRIGGER_SEQUENCE_COMPLETE	BIT(4)
#define PIXENGCFG_STATUS		0x18
#define SYNC_BUSY			BIT(8)
#define KICK_MODE			BIT(8)
#define PERFCOUNTMODE			BIT(12)
#define CONTROL				0xC
#define GAMMAAPPLYENABLE		BIT(0)
#define SOFTWAREKICK			0x10
#define KICK				BIT(0)
#define STATUS				0x14
#define CNT_ERR_STS			BIT(0)
#define CONTROLWORD			0x18
#define CURPIXELCNT			0x1C
static u16 get_xval(u32 pixel_cnt)
{
	return pixel_cnt && 0xFF;
}

static u16 get_yval(u32 pixel_cnt)
{
	return pixel_cnt >> 16;
}
#define LASTPIXELCNT			0x20
#define PERFCOUNTER			0x24

struct dpu_extdst {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
};

static inline u32 dpu_pec_ed_read(struct dpu_extdst *ed, unsigned int offset)
{
	return readl(ed->pec_base + offset);
}

static inline void dpu_pec_ed_write(struct dpu_extdst *ed, u32 value,
				unsigned int offset)
{
	writel(value, ed->pec_base + offset);
}

static inline u32 dpu_ed_read(struct dpu_extdst *ed, unsigned int offset)
{
	return readl(ed->base + offset);
}

static inline void dpu_ed_write(struct dpu_extdst *ed, u32 value,
				unsigned int offset)
{
	writel(value, ed->base + offset);
}

static inline bool dpu_ed_is_safety_stream(struct dpu_extdst *ed)
{
	if (ed->id == 4 || ed->id == 5)
		return true;

	return false;
}

static inline bool dpu_ed_src_sel_is_extsrc(extdst_src_sel_t src)
{
	if (src == ED_SRC_EXTSRC4 || src == ED_SRC_EXTSRC5)
		return true;

	return false;
}

void extdst_pixengcfg_shden(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_pec_ed_write(ed, val, PIXENGCFG_STATIC);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_shden);

void extdst_pixengcfg_powerdown(struct dpu_extdst *ed, bool powerdown)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (powerdown)
		val |= POWERDOWN;
	else
		val &= ~POWERDOWN;
	dpu_pec_ed_write(ed, val, PIXENGCFG_STATIC);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_powerdown);

void extdst_pixengcfg_sync_mode(struct dpu_extdst *ed, ed_sync_mode_t mode)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (mode == AUTO)
		val |= SYNC_MODE;
	else
		val &= ~SYNC_MODE;
	dpu_pec_ed_write(ed, val, PIXENGCFG_STATIC);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_sync_mode);

void extdst_pixengcfg_reset(struct dpu_extdst *ed, bool reset)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (reset)
		val |= SW_RESET;
	else
		val &= ~SW_RESET;
	dpu_pec_ed_write(ed, val, PIXENGCFG_STATIC);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_reset);

void extdst_pixengcfg_div(struct dpu_extdst *ed, u16 div)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	val &= ~0xFF0000;
	val |= DIV(div);
	dpu_pec_ed_write(ed, val, PIXENGCFG_STATIC);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_div);

int extdst_pixengcfg_src_sel(struct dpu_extdst *ed, extdst_src_sel_t src)
{
	struct dpu_soc *dpu = ed->dpu;
	const unsigned int *block_id_map = dpu->devtype->sw2hw_block_id_map;
	u32 mapped_src;

	mapped_src = block_id_map ? block_id_map[src] : src;
	if (WARN_ON(mapped_src == NA))
		return -EINVAL;

	if (dpu_ed_is_safety_stream(ed) && dpu_ed_src_sel_is_extsrc(src)) {
		dev_err(dpu->dev, "ExtDst%d source cannot be ExtSrc\n", ed->id);
		return -EINVAL;
	}

	mutex_lock(&ed->mutex);
	dpu_pec_ed_write(ed, mapped_src, PIXENGCFG_DYNAMIC);
	mutex_unlock(&ed->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_src_sel);

void extdst_pixengcfg_sel_shdldreq(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_REQUEST);
	val |= SEL_SHDLDREQ;
	dpu_pec_ed_write(ed, val, PIXENGCFG_REQUEST);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_sel_shdldreq);

void extdst_pixengcfg_shdldreq(struct dpu_extdst *ed, u32 req_mask)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_REQUEST);
	val |= req_mask;
	dpu_pec_ed_write(ed, val, PIXENGCFG_REQUEST);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_shdldreq);

void extdst_pixengcfg_sync_trigger(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_pec_ed_write(ed, SYNC_TRIGGER, PIXENGCFG_TRIGGER);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_sync_trigger);

void extdst_pixengcfg_trigger_sequence_complete(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_pec_ed_write(ed, TRIGGER_SEQUENCE_COMPLETE, PIXENGCFG_TRIGGER);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_trigger_sequence_complete);

bool extdst_pixengcfg_is_sync_busy(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATUS);
	mutex_unlock(&ed->mutex);

	return val & SYNC_BUSY;
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_is_sync_busy);

ed_pipeline_status_t extdst_pixengcfg_pipeline_status(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATUS);
	mutex_unlock(&ed->mutex);

	return val & 0x3;
}
EXPORT_SYMBOL_GPL(extdst_pixengcfg_pipeline_status);

void extdst_shden(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_ed_write(ed, val, STATICCONTROL);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_shden);

void extdst_kick_mode(struct dpu_extdst *ed, ed_kick_mode_t mode)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATICCONTROL);
	val &= ~KICK_MODE;
	val |= mode;
	dpu_ed_write(ed, val, STATICCONTROL);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_kick_mode);

void extdst_perfcountmode(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATICCONTROL);
	if (enable)
		val |= PERFCOUNTMODE;
	else
		val &= ~PERFCOUNTMODE;
	dpu_ed_write(ed, val, STATICCONTROL);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_perfcountmode);

void extdst_gamma_apply_enable(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, CONTROL);
	if (enable)
		val |= GAMMAAPPLYENABLE;
	else
		val &= ~GAMMAAPPLYENABLE;
	dpu_ed_write(ed, val, CONTROL);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_gamma_apply_enable);

void extdst_kick(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_ed_write(ed, KICK, SOFTWAREKICK);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_kick);

void extdst_cnt_err_clear(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_ed_write(ed, CNT_ERR_STS, STATUS);
	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(extdst_cnt_err_clear);

bool extdst_cnt_err_status(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATUS);
	mutex_unlock(&ed->mutex);

	return val & CNT_ERR_STS;
}
EXPORT_SYMBOL_GPL(extdst_cnt_err_status);

u32 extdst_last_control_word(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, CONTROLWORD);
	mutex_unlock(&ed->mutex);

	return val;
}
EXPORT_SYMBOL_GPL(extdst_last_control_word);

void extdst_pixel_cnt(struct dpu_extdst *ed, u16 *x, u16 *y)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, CURPIXELCNT);
	mutex_unlock(&ed->mutex);

	*x = get_xval(val);
	*y = get_yval(val);
}
EXPORT_SYMBOL_GPL(extdst_pixel_cnt);

void extdst_last_pixel_cnt(struct dpu_extdst *ed, u16 *x, u16 *y)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, LASTPIXELCNT);
	mutex_unlock(&ed->mutex);

	*x = get_xval(val);
	*y = get_yval(val);
}
EXPORT_SYMBOL_GPL(extdst_last_pixel_cnt);

u32 extdst_perfresult(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, PERFCOUNTER);
	mutex_unlock(&ed->mutex);

	return val;
}
EXPORT_SYMBOL_GPL(extdst_perfresult);

struct dpu_extdst *dpu_ed_get(struct dpu_soc *dpu, int id)
{
	struct dpu_extdst *ed;
	int i;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(ed_ids))
		return ERR_PTR(-EINVAL);

	ed = dpu->ed_priv[i];

	mutex_lock(&ed->mutex);

	if (ed->inuse) {
		mutex_unlock(&ed->mutex);
		return ERR_PTR(-EBUSY);
	}

	ed->inuse = true;

	mutex_unlock(&ed->mutex);

	return ed;
}
EXPORT_SYMBOL_GPL(dpu_ed_get);

void dpu_ed_put(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);

	ed->inuse = false;

	mutex_unlock(&ed->mutex);
}
EXPORT_SYMBOL_GPL(dpu_ed_put);

struct dpu_extdst *dpu_aux_ed_peek(struct dpu_extdst *ed)
{
	unsigned int aux_id = ed->id ^ 1;
	int i;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == aux_id)
			return ed->dpu->ed_priv[i];

	return NULL;
}
EXPORT_SYMBOL_GPL(dpu_aux_ed_peek);

void _dpu_ed_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_extdst *ed;
	int i;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(ed_ids)))
		return;

	ed = dpu->ed_priv[i];

	extdst_pixengcfg_src_sel(ed, ED_SRC_DISABLE);
	extdst_pixengcfg_shden(ed, true);
	extdst_pixengcfg_powerdown(ed, false);
	extdst_pixengcfg_sync_mode(ed, SINGLE);
	extdst_pixengcfg_reset(ed, false);
	extdst_pixengcfg_div(ed, DIV_RESET);
	extdst_shden(ed, true);
	extdst_perfcountmode(ed, false);
	extdst_kick_mode(ed, EXTERNAL);
}

int dpu_ed_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_extdst *ed;
	int ret, i;

	ed = devm_kzalloc(dpu->dev, sizeof(*ed), GFP_KERNEL);
	if (!ed)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == id)
			break;

	dpu->ed_priv[i] = ed;

	ed->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_32);
	if (!ed->pec_base)
		return -ENOMEM;

	ed->base = devm_ioremap(dpu->dev, base, SZ_64);
	if (!ed->base)
		return -ENOMEM;

	ed->dpu = dpu;
	ed->id = id;
	mutex_init(&ed->mutex);

	ret = extdst_pixengcfg_src_sel(ed, ED_SRC_DISABLE);
	if (ret < 0)
		return ret;

	_dpu_ed_init(dpu, id);

	return 0;
}
