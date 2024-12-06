// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019,2023 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"

#define STATICCONTROL		0x8

#define FRAMEDIMENSIONS		0xc
#define  WIDTH(w)		(((w) - 1) & 0x3fff)
#define  HEIGHT(h)		((((h) - 1) & 0x3fff) << 16)

#define CONSTANTCOLOR		0x10
#define  RED(r)			(((r) & 0xff) << 24)
#define  GREEN(g)		(((g) & 0xff) << 16)
#define  BLUE(b)		(((b) & 0xff) << 8)
#define  ALPHA(a)		((a) & 0xff)

#define CONTROLTRIGGER		0x14
#define START			0x18
#define STATUS			0x1c

struct dpu95_constframe {
	void __iomem *pec_base;
	void __iomem *base;
	int id;
	unsigned int index;
	enum dpu95_link_id link_id;
	struct dpu95_soc *dpu;
};

static const enum dpu95_link_id dpu95_cf_link_id[] = {
	DPU95_LINK_ID_CONSTFRAME0, DPU95_LINK_ID_CONSTFRAME1,
	DPU95_LINK_ID_CONSTFRAME4, DPU95_LINK_ID_CONSTFRAME5,
};

static inline void dpu95_cf_write(struct dpu95_constframe *cf,
				  unsigned int offset, u32 value)
{
	writel(value, cf->base + offset);
}

static void dpu95_cf_enable_shden(struct dpu95_constframe *cf)
{
	dpu95_cf_write(cf, STATICCONTROL, SHDEN);
}

enum dpu95_link_id dpu95_cf_get_link_id(struct dpu95_constframe *cf)
{
	return cf->link_id;
}

void dpu95_cf_framedimensions(struct dpu95_constframe *cf, unsigned int w,
			      unsigned int h)
{
	dpu95_cf_write(cf, FRAMEDIMENSIONS, WIDTH(w) | HEIGHT(h));
}

void dpu95_cf_constantcolor_black(struct dpu95_constframe *cf)
{
	dpu95_cf_write(cf, CONSTANTCOLOR, 0);
}

static struct dpu95_constframe *dpu95_cf_get(struct dpu95_soc *dpu,
					     unsigned int id)
{
	struct dpu95_constframe *cf;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->cf); i++) {
		cf = dpu->cf[i];
		if (cf->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->cf))
		return ERR_PTR(-EINVAL);

	return cf;
}

/* ConstFrame for safety stream */
struct dpu95_constframe *dpu95_cf_safe_get(struct dpu95_soc *dpu,
					   unsigned int stream_id)
{
	return dpu95_cf_get(dpu, stream_id + DPU95_SAFETY_STREAM_OFFSET);
}

/* ConstFrame for content stream */
struct dpu95_constframe *dpu95_cf_cont_get(struct dpu95_soc *dpu,
					   unsigned int stream_id)
{
	return dpu95_cf_get(dpu, stream_id);
}

void dpu95_cf_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	dpu95_cf_enable_shden(dpu->cf[index]);
}

int dpu95_cf_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_constframe *cf;

	cf = devm_kzalloc(dpu->dev, sizeof(*cf), GFP_KERNEL);
	if (!cf)
		return -ENOMEM;

	dpu->cf[index] = cf;

	cf->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!cf->pec_base)
		return -ENOMEM;

	cf->base = devm_ioremap(dpu->dev, base, SZ_32);
	if (!cf->base)
		return -ENOMEM;

	cf->dpu = dpu;
	cf->id = id;
	cf->index = index;
	cf->link_id = dpu95_cf_link_id[index];

	return 0;
}
