// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019,2023 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"

#define PIXENGCFG_STATIC		0x8
#define  POWERDOWN			BIT(4)
#define  SYNC_MODE			BIT(8)
#define  AUTO				BIT(8)
#define  SINGLE				0
#define  DIV_MASK			0xff0000
#define  DIV(n)				(((n) & 0xff) << 16)
#define  DIV_RESET			0x80

#define PIXENGCFG_DYNAMIC		0xc

#define PIXENGCFG_REQUEST		0x10
#define  SHDLDREQ(n)			BIT(n)
#define  SEL_SHDLDREQ			BIT(0)

#define PIXENGCFG_TRIGGER		0x14
#define  SYNC_TRIGGER			BIT(0)
#define  TRIGGER_SEQUENCE_COMPLETE	BIT(4)

#define PIXENGCFG_STATUS		0x18
#define  SYNC_BUSY			BIT(8)
#define  PERFCOUNTMODE			BIT(12)

#define STATICCONTROL			0x8
#define  KICK_MODE			BIT(8)
#define  EXTERNAL			BIT(8)
#define  SOFTWARE			0
#define  PERFCOUNTMODE			BIT(12)

#define CONTROL				0xc
#define  GAMMAAPPLYENABLE		BIT(0)

#define SOFTWAREKICK			0x10
#define  KICK				BIT(0)

#define STATUS				0x14
#define  CNT_ERR_STS			BIT(0)

#define CONTROLWORD			0x18
#define CURPIXELCNT			0x1c
#define LASTPIXELCNT			0x20
#define PERFCOUNTER			0x24

struct dpu95_extdst {
	void __iomem *pec_base;
	void __iomem *base;
	int id;
	unsigned int index;
	bool inuse;
	struct dpu95_soc *dpu;
};

static const enum dpu95_link_id src_sels[] = {
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_LAYERBLEND6,
	DPU95_LINK_ID_LAYERBLEND5,
	DPU95_LINK_ID_LAYERBLEND4,
	DPU95_LINK_ID_LAYERBLEND3,
	DPU95_LINK_ID_LAYERBLEND2,
	DPU95_LINK_ID_LAYERBLEND1,
};

static inline u32 dpu95_pec_ed_read(struct dpu95_extdst *ed,
				    unsigned int offset)
{
	return readl(ed->pec_base + offset);
}

static inline void dpu95_pec_ed_write(struct dpu95_extdst *ed,
				      unsigned int offset, u32 value)
{
	writel(value, ed->pec_base + offset);
}

static inline void dpu95_pec_ed_write_mask(struct dpu95_extdst *ed,
					   unsigned int offset,
					   u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_pec_ed_read(ed, offset);
	tmp &= ~mask;
	dpu95_pec_ed_write(ed, offset, tmp | value);
}

static inline u32 dpu95_ed_read(struct dpu95_extdst *ed, unsigned int offset)
{
	return readl(ed->base + offset);
}

static inline void dpu95_ed_write(struct dpu95_extdst *ed,
				  unsigned int offset, u32 value)
{
	writel(value, ed->base + offset);
}

static inline void dpu95_ed_write_mask(struct dpu95_extdst *ed,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_ed_read(ed, offset);
	tmp &= ~mask;
	dpu95_ed_write(ed, offset, tmp | value);
}

static void dpu95_ed_pec_enable_shden(struct dpu95_extdst *ed)
{
	dpu95_pec_ed_write_mask(ed, PIXENGCFG_STATIC, SHDEN, SHDEN);
}

void dpu95_ed_pec_poweron(struct dpu95_extdst *ed)
{
	dpu95_pec_ed_write_mask(ed, PIXENGCFG_STATIC, POWERDOWN, 0);
}

static void dpu95_ed_pec_sync_mode_single(struct dpu95_extdst *ed)
{
	dpu95_pec_ed_write_mask(ed, PIXENGCFG_STATIC, SYNC_MODE, SINGLE);
}

static void dpu95_ed_pec_div_reset(struct dpu95_extdst *ed)
{
	dpu95_pec_ed_write_mask(ed, PIXENGCFG_STATIC, DIV_MASK, DIV(DIV_RESET));
}

void dpu95_ed_pec_src_sel(struct dpu95_extdst *ed, enum dpu95_link_id src)
{
	struct dpu95_soc *dpu = ed->dpu;
	int i;

	for (i = 0; i < ARRAY_SIZE(src_sels); i++) {
		if (src_sels[i] == src) {
			dpu95_pec_ed_write(ed, PIXENGCFG_DYNAMIC, src);
			return;
		}
	}

	dev_err(dpu->dev, "invalid source(0x%02x) for ExtDst%u\n", src, ed->id);
}

void dpu95_ed_pec_sync_trigger(struct dpu95_extdst *ed)
{
	dpu95_pec_ed_write(ed, PIXENGCFG_TRIGGER, SYNC_TRIGGER);
}

static void dpu95_ed_enable_shden(struct dpu95_extdst *ed)
{
	dpu95_ed_write_mask(ed, STATICCONTROL, SHDEN, SHDEN);
}
static void dpu95_ed_kick_mode_external(struct dpu95_extdst *ed)
{
	dpu95_ed_write_mask(ed, STATICCONTROL, KICK_MODE, EXTERNAL);
}

static void dpu95_ed_disable_perfcountmode(struct dpu95_extdst *ed)
{
	dpu95_ed_write_mask(ed, STATICCONTROL, PERFCOUNTMODE, 0);
}

static void dpu95_ed_disable_gamma_apply(struct dpu95_extdst *ed)
{
	dpu95_ed_write_mask(ed, CONTROL, GAMMAAPPLYENABLE, 0);
}

static struct dpu95_extdst *dpu95_ed_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_extdst *ed;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->ed); i++) {
		ed = dpu->ed[i];
		if (ed->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->ed))
		return ERR_PTR(-EINVAL);

	return ed;
}

/* ExtDst for safety stream */
struct dpu95_extdst *dpu95_ed_safe_get(struct dpu95_soc *dpu,
				       unsigned int stream_id)
{
	return dpu95_ed_get(dpu, stream_id + DPU95_SAFETY_STREAM_OFFSET);
}

/* ExtDst for content stream */
struct dpu95_extdst *dpu95_ed_cont_get(struct dpu95_soc *dpu,
				       unsigned int stream_id)
{
	return dpu95_ed_get(dpu, stream_id);
}

void dpu95_ed_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_extdst *ed = dpu->ed[index];

	dpu95_ed_pec_src_sel(ed, DPU95_LINK_ID_NONE);
	dpu95_ed_pec_enable_shden(ed);
	dpu95_ed_pec_poweron(ed);
	dpu95_ed_pec_sync_mode_single(ed);
	dpu95_ed_pec_div_reset(ed);
	dpu95_ed_enable_shden(ed);
	dpu95_ed_disable_perfcountmode(ed);
	dpu95_ed_kick_mode_external(ed);
	dpu95_ed_disable_gamma_apply(ed);
}

int dpu95_ed_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_extdst *ed;

	ed = devm_kzalloc(dpu->dev, sizeof(*ed), GFP_KERNEL);
	if (!ed)
		return -ENOMEM;

	dpu->ed[index] = ed;

	ed->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_32);
	if (!ed->pec_base)
		return -ENOMEM;

	ed->base = devm_ioremap(dpu->dev, base, SZ_128);
	if (!ed->base)
		return -ENOMEM;

	ed->dpu = dpu;
	ed->id = id;
	ed->index = index;

	return 0;
}
