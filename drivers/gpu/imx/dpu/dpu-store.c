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
#include "dpu-prv.h"

#define PIXENGCFG_STATIC		0x8

struct dpu_store {
	void __iomem *pec_base;
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
};

static inline u32 dpu_pec_st_read(struct dpu_store *st, unsigned int offset)
{
	return readl(st->pec_base + offset);
}

static inline void dpu_pec_st_write(struct dpu_store *st, u32 value,
				    unsigned int offset)
{
	writel(value, st->pec_base + offset);
}

void store_pixengcfg_syncmode_fixup(struct dpu_store *st, bool enable)
{
	struct dpu_soc *dpu;
	u32 val;

	if (!st)
		return;

	dpu = st->dpu;

	if (!dpu->devtype->has_syncmode_fixup)
		return;

	mutex_lock(&st->mutex);
	val = dpu_pec_st_read(st, PIXENGCFG_STATIC);
	if (enable)
		val |= BIT(16);
	else
		val &= ~BIT(16);
	dpu_pec_st_write(st, val, PIXENGCFG_STATIC);
	mutex_unlock(&st->mutex);
}
EXPORT_SYMBOL_GPL(store_pixengcfg_syncmode_fixup);

struct dpu_store *dpu_st_get(struct dpu_soc *dpu, int id)
{
	struct dpu_store *st;
	int i;

	for (i = 0; i < ARRAY_SIZE(st_ids); i++)
		if (st_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(st_ids))
		return ERR_PTR(-EINVAL);

	st = dpu->st_priv[i];

	mutex_lock(&st->mutex);

	if (st->inuse) {
		mutex_unlock(&st->mutex);
		return ERR_PTR(-EBUSY);
	}

	st->inuse = true;

	mutex_unlock(&st->mutex);

	return st;
}
EXPORT_SYMBOL_GPL(dpu_st_get);

void dpu_st_put(struct dpu_store *st)
{
	mutex_lock(&st->mutex);

	st->inuse = false;

	mutex_unlock(&st->mutex);
}
EXPORT_SYMBOL_GPL(dpu_st_put);

int dpu_st_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_store *st;
	int i;

	st = devm_kzalloc(dpu->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(st_ids); i++)
		if (st_ids[i] == id)
			break;

	dpu->st_priv[i] = st;

	st->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_32);
	if (!st->pec_base)
		return -ENOMEM;

	st->base = devm_ioremap(dpu->dev, base, SZ_256);
	if (!st->base)
		return -ENOMEM;

	st->dpu = dpu;
	st->id = id;

	mutex_init(&st->mutex);

	return 0;
}
