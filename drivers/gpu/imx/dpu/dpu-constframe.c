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

static unsigned int safety_stream_cf_color = 0x0;
module_param(safety_stream_cf_color, uint, 0444);
MODULE_PARM_DESC(safety_stream_cf_color,
"Safety stream constframe color in hex(0xRRGGBBAA) [default=0x00000000]");

#define FRAMEDIMENSIONS		0xC
#define WIDTH(w)		(((w) - 1) & 0x3FFF)
#define HEIGHT(h)		((((h) - 1) & 0x3FFF) << 16)
#define CONSTANTCOLOR		0x10
#define RED(r)			(((r) & 0xFF) << 24)
#define GREEN(g)		(((g) & 0xFF) << 16)
#define BLUE(b)			(((b) & 0xFF) << 8)
#define ALPHA(a)		((a) & 0xFF)
#define CONTROLTRIGGER		0x14
#define START			0x18
#define STATUS			0x1C

static const shadow_load_req_t cf_shdlreqs[] = {
	SHLDREQID_CONSTFRAME0, SHLDREQID_CONSTFRAME1,
	SHLDREQID_CONSTFRAME4, SHLDREQID_CONSTFRAME5,
};

struct dpu_constframe {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
	shadow_load_req_t shdlreq;
};

static inline u32 dpu_cf_read(struct dpu_constframe *cf, unsigned int offset)
{
	return readl(cf->base + offset);
}

static inline void dpu_cf_write(struct dpu_constframe *cf, u32 value,
				unsigned int offset)
{
	writel(value, cf->base + offset);
}

void constframe_shden(struct dpu_constframe *cf, bool enable)
{
	u32 val;

	val = enable ? SHDEN : 0;

	mutex_lock(&cf->mutex);
	dpu_cf_write(cf, val, STATICCONTROL);
	mutex_unlock(&cf->mutex);
}
EXPORT_SYMBOL_GPL(constframe_shden);

void constframe_framedimensions(struct dpu_constframe *cf, unsigned int w,
				unsigned int h)
{
	u32 val;

	val = WIDTH(w) | HEIGHT(h);

	mutex_lock(&cf->mutex);
	dpu_cf_write(cf, val, FRAMEDIMENSIONS);
	mutex_unlock(&cf->mutex);
}
EXPORT_SYMBOL_GPL(constframe_framedimensions);

void constframe_framedimensions_copy_prim(struct dpu_constframe *cf)
{
	struct dpu_constframe *prim_cf = NULL;
	unsigned int prim_id;
	int i;
	u32 val;

	if (cf->id != 0 && cf->id != 1) {
		dev_warn(cf->dpu->dev, "ConstFrame%d is not a secondary one\n",
								cf->id);
		return;
	}

	prim_id = cf->id + 4;

	for (i = 0; i < ARRAY_SIZE(cf_ids); i++)
		if (cf_ids[i] == prim_id)
			prim_cf = cf->dpu->cf_priv[i];

	if (!prim_cf) {
		dev_warn(cf->dpu->dev, "cannot find ConstFrame%d's primary peer\n",
								cf->id);
		return;
	}

	mutex_lock(&cf->mutex);
	val = dpu_cf_read(prim_cf, FRAMEDIMENSIONS);
	dpu_cf_write(cf, val, FRAMEDIMENSIONS);
	mutex_unlock(&cf->mutex);
}
EXPORT_SYMBOL_GPL(constframe_framedimensions_copy_prim);

void constframe_constantcolor(struct dpu_constframe *cf, unsigned int r,
			      unsigned int g, unsigned int b, unsigned int a)
{
	u32 val;

	val = RED(r) | GREEN(g) | BLUE(b) | ALPHA(a);

	mutex_lock(&cf->mutex);
	dpu_cf_write(cf, val, CONSTANTCOLOR);
	mutex_unlock(&cf->mutex);
}
EXPORT_SYMBOL_GPL(constframe_constantcolor);

void constframe_controltrigger(struct dpu_constframe *cf, bool trigger)
{
	u32 val;

	val = trigger ? SHDTOKGEN : 0;

	mutex_lock(&cf->mutex);
	dpu_cf_write(cf, val, CONTROLTRIGGER);
	mutex_unlock(&cf->mutex);
}
EXPORT_SYMBOL_GPL(constframe_controltrigger);

shadow_load_req_t constframe_to_shdldreq_t(struct dpu_constframe *cf)
{
	shadow_load_req_t t = 0;

	switch (cf->id) {
	case 0:
		t = SHLDREQID_CONSTFRAME0;
		break;
	case 1:
		t = SHLDREQID_CONSTFRAME1;
		break;
	case 4:
		t = SHLDREQID_CONSTFRAME4;
		break;
	case 5:
		t = SHLDREQID_CONSTFRAME5;
		break;
	}

	return t;
}
EXPORT_SYMBOL_GPL(constframe_to_shdldreq_t);

struct dpu_constframe *dpu_cf_get(struct dpu_soc *dpu, int id)
{
	struct dpu_constframe *cf;
	int i;

	for (i = 0; i < ARRAY_SIZE(cf_ids); i++)
		if (cf_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(cf_ids))
		return ERR_PTR(-EINVAL);

	cf = dpu->cf_priv[i];

	mutex_lock(&cf->mutex);

	if (cf->inuse) {
		mutex_unlock(&cf->mutex);
		return ERR_PTR(-EBUSY);
	}

	cf->inuse = true;

	mutex_unlock(&cf->mutex);

	return cf;
}
EXPORT_SYMBOL_GPL(dpu_cf_get);

void dpu_cf_put(struct dpu_constframe *cf)
{
	mutex_lock(&cf->mutex);

	cf->inuse = false;

	mutex_unlock(&cf->mutex);
}
EXPORT_SYMBOL_GPL(dpu_cf_put);

struct dpu_constframe *dpu_aux_cf_peek(struct dpu_constframe *cf)
{
	unsigned int aux_id = cf->id ^ 1;
	int i;

	for (i = 0; i < ARRAY_SIZE(cf_ids); i++)
		if (cf_ids[i] == aux_id)
			return cf->dpu->cf_priv[i];

	return NULL;
}
EXPORT_SYMBOL_GPL(dpu_aux_cf_peek);

void _dpu_cf_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_constframe *cf;
	int i;

	for (i = 0; i < ARRAY_SIZE(cf_ids); i++)
		if (cf_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(cf_ids)))
		return;

	cf = dpu->cf_priv[i];

	constframe_shden(cf, true);

	if (id == 4 || id == 5) {
		mutex_lock(&cf->mutex);
		dpu_cf_write(cf, safety_stream_cf_color, CONSTANTCOLOR);
		mutex_unlock(&cf->mutex);
	}
}

int dpu_cf_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_constframe *cf;
	int i;

	cf = devm_kzalloc(dpu->dev, sizeof(*cf), GFP_KERNEL);
	if (!cf)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(cf_ids); i++)
		if (cf_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(cf_ids))
		return -EINVAL;

	dpu->cf_priv[i] = cf;

	cf->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!cf->pec_base)
		return -ENOMEM;

	cf->base = devm_ioremap(dpu->dev, base, SZ_32);
	if (!cf->base)
		return -ENOMEM;

	cf->dpu = dpu;
	cf->id = id;
	cf->shdlreq = cf_shdlreqs[i];

	mutex_init(&cf->mutex);

	_dpu_cf_init(dpu, id);

	return 0;
}
