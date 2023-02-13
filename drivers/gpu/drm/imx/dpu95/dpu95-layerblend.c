// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019,2023 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#include <drm/drm_blend.h>

#include "dpu95.h"

#define PIXENGCFG_DYNAMIC			0x8
#define  PIXENGCFG_DYNAMIC_PRIM_SEL_MASK	0x3f
#define  PIXENGCFG_DYNAMIC_SEC_SEL_SHIFT	8
#define  PIXENGCFG_DYNAMIC_SEC_SEL_MASK		0x3f00

#define PIXENGCFG_STATUS			0xc

#define STATICCONTROL				0x8
#define  SHDTOKSEL_MASK				0x18
#define  SHDTOKSEL(n)				((n) << 3)
#define  SHDLDSEL_MASK				0x6
#define  SHDLDSEL(n)				((n) << 1)

#define CONTROL					0xc

#define BLENDCONTROL				0x10
#define  ALPHA(a)				(((a) & 0xff) << 16)
#define  PRIM_C_BLD_FUNC__ONE_MINUS_CONST_ALPHA	0x7
#define  PRIM_C_BLD_FUNC__ONE_MINUS_SEC_ALPHA	0x5
#define  PRIM_C_BLD_FUNC__ZERO			0x0
#define  SEC_C_BLD_FUNC__CONST_ALPHA		(0x6 << 4)
#define  SEC_C_BLD_FUNC__SEC_ALPHA		(0x4 << 4)
#define  PRIM_A_BLD_FUNC__ZERO			(0x0 << 8)
#define  SEC_A_BLD_FUNC__ZERO			(0x0 << 12)

#define POSITION				0x14
#define  XPOS(x)				((x) & 0xffff)
#define  YPOS(y)				(((y) & 0xffff) << 16)

#define PRIMCONTROLWORD				0x18
#define SECCONTROLWORD				0x1c

enum dpu95_lb_shadow_sel {
	PRIMARY,	/* background plane */
	SECONDARY,	/* foreground plane */
	BOTH,
};

struct dpu95_layerblend {
	void __iomem *pec_base;
	void __iomem *base;
	unsigned int id;
	unsigned int index;
	enum dpu95_link_id link_id;
	struct dpu95_soc *dpu;
};

static const enum dpu95_link_id dpu95_lb_link_id[] = {
	DPU95_LINK_ID_LAYERBLEND1, DPU95_LINK_ID_LAYERBLEND2,
	DPU95_LINK_ID_LAYERBLEND3, DPU95_LINK_ID_LAYERBLEND4,
	DPU95_LINK_ID_LAYERBLEND5, DPU95_LINK_ID_LAYERBLEND6,
};

static const enum dpu95_link_id prim_sels[] = {
	/* common options */
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_CONSTFRAME0,
	DPU95_LINK_ID_CONSTFRAME1,
	DPU95_LINK_ID_CONSTFRAME4,
	DPU95_LINK_ID_CONSTFRAME5,
	/*
	 * special options:
	 * layerblend(n) has n special options,
	 * from layerblend0 to layerblend(n - 1), e.g.,
	 * layerblend3 has 3 special options -
	 * layerblend0/1/2.
	 */
	DPU95_LINK_ID_LAYERBLEND1,
	DPU95_LINK_ID_LAYERBLEND2,
	DPU95_LINK_ID_LAYERBLEND3,
	DPU95_LINK_ID_LAYERBLEND4,
	DPU95_LINK_ID_LAYERBLEND5,
	DPU95_LINK_ID_LAYERBLEND6,
};

static const enum dpu95_link_id sec_sels[] = {
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_FETCHLAYER0,
	DPU95_LINK_ID_FETCHLAYER1,
	DPU95_LINK_ID_FETCHYUV0,
	DPU95_LINK_ID_FETCHYUV1,
	DPU95_LINK_ID_FETCHYUV2,
	DPU95_LINK_ID_FETCHYUV3,
	DPU95_LINK_ID_HSCALER4,
	DPU95_LINK_ID_MATRIX4,
};

static inline u32 dpu95_pec_lb_read(struct dpu95_layerblend *lb,
				    unsigned int offset)
{
	return readl(lb->pec_base + offset);
}

static inline void dpu95_pec_lb_write(struct dpu95_layerblend *lb,
				      unsigned int offset, u32 value)
{
	writel(value, lb->pec_base + offset);
}

static inline void dpu95_pec_lb_write_mask(struct dpu95_layerblend *lb,
					   unsigned int offset,
					   u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_pec_lb_read(lb, offset);
	tmp &= ~mask;
	dpu95_pec_lb_write(lb, offset, tmp | value);
}

static inline u32 dpu95_lb_read(struct dpu95_layerblend *lb, unsigned int offset)
{
	return readl(lb->base + offset);
}

static inline void dpu95_lb_write(struct dpu95_layerblend *lb,
				  unsigned int offset, u32 value)
{
	writel(value, lb->base + offset);
}

static inline void dpu95_lb_write_mask(struct dpu95_layerblend *lb,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_lb_read(lb, offset);
	tmp &= ~mask;
	dpu95_lb_write(lb, offset, tmp | value);
}

enum dpu95_link_id dpu95_lb_get_link_id(struct dpu95_layerblend *lb)
{
	return lb->link_id;
}

void dpu95_lb_pec_dynamic_prim_sel(struct dpu95_layerblend *lb,
				   enum dpu95_link_id prim)
{
	struct dpu95_soc *dpu = lb->dpu;
	int fixed_sels_num = ARRAY_SIZE(prim_sels) - 6;
	int i;

	for (i = 0; i < fixed_sels_num + lb->id - 1; i++) {
		if (prim_sels[i] == prim) {
			dpu95_pec_lb_write_mask(lb, PIXENGCFG_DYNAMIC,
						PIXENGCFG_DYNAMIC_PRIM_SEL_MASK,
						prim);
			return;
		}
	}

	dev_err(dpu->dev, "LayerBlend%u - invalid primary source 0x%02x\n",
		lb->id, prim);
}

void dpu95_lb_pec_dynamic_sec_sel(struct dpu95_layerblend *lb,
				  enum dpu95_link_id sec)
{
	struct dpu95_soc *dpu = lb->dpu;
	int i;

	for (i = 0; i < ARRAY_SIZE(sec_sels); i++) {
		if (sec_sels[i] == sec) {
			dpu95_pec_lb_write_mask(lb, PIXENGCFG_DYNAMIC,
					PIXENGCFG_DYNAMIC_SEC_SEL_MASK,
					sec << PIXENGCFG_DYNAMIC_SEC_SEL_SHIFT);
			return;
		}
	}

	dev_err(dpu->dev, "LayerBlend%u - invalid secondary source 0x%02x\n",
		lb->id, sec);
}

void dpu95_lb_pec_clken(struct dpu95_layerblend *lb, enum dpu95_pec_clken clken)
{
	dpu95_pec_lb_write_mask(lb, PIXENGCFG_DYNAMIC, CLKEN_MASK, CLKEN(clken));
}

static void dpu95_lb_enable_shden(struct dpu95_layerblend *lb)
{
	dpu95_lb_write_mask(lb, STATICCONTROL, SHDEN, SHDEN);
}

static void dpu95_lb_shdtoksel(struct dpu95_layerblend *lb,
			       enum dpu95_lb_shadow_sel sel)
{
	dpu95_lb_write_mask(lb, STATICCONTROL, SHDTOKSEL_MASK, SHDTOKSEL(sel));
}

static void dpu95_lb_shdldsel(struct dpu95_layerblend *lb,
			      enum dpu95_lb_shadow_sel sel)
{
	dpu95_lb_write_mask(lb, STATICCONTROL, SHDLDSEL_MASK, SHDLDSEL(sel));
}

void dpu95_lb_mode(struct dpu95_layerblend *lb, enum dpu95_lb_mode mode)
{
	dpu95_lb_write_mask(lb, CONTROL, CTRL_MODE_MASK, mode);
}

void dpu95_lb_blendcontrol(struct dpu95_layerblend *lb, unsigned int zpos,
			   unsigned int pixel_blend_mode, u16 alpha)
{
	u32 val = PRIM_A_BLD_FUNC__ZERO | SEC_A_BLD_FUNC__ZERO;

	if (zpos == 0) {
		val |= PRIM_C_BLD_FUNC__ZERO | SEC_C_BLD_FUNC__CONST_ALPHA;
		alpha = DRM_BLEND_ALPHA_OPAQUE;
	} else {
		switch (pixel_blend_mode) {
		case DRM_MODE_BLEND_PIXEL_NONE:
			val |= PRIM_C_BLD_FUNC__ONE_MINUS_CONST_ALPHA |
			       SEC_C_BLD_FUNC__CONST_ALPHA;
			break;
		case DRM_MODE_BLEND_PREMULTI:
			val |= PRIM_C_BLD_FUNC__ONE_MINUS_SEC_ALPHA |
			       SEC_C_BLD_FUNC__CONST_ALPHA;
			break;
		case DRM_MODE_BLEND_COVERAGE:
			val |= PRIM_C_BLD_FUNC__ONE_MINUS_SEC_ALPHA |
			       SEC_C_BLD_FUNC__SEC_ALPHA;
			break;
		}
	}

	val |= ALPHA(alpha >> 8);

	dpu95_lb_write(lb, BLENDCONTROL, val);
}

void dpu95_lb_position(struct dpu95_layerblend *lb, int x, int y)
{
	dpu95_lb_write(lb, POSITION, XPOS(x) | YPOS(y));
}

unsigned int dpu95_lb_get_id(struct dpu95_layerblend *lb)
{
	return lb->id;
}

struct dpu95_layerblend *dpu95_lb_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_layerblend *lb;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->lb); i++) {
		lb = dpu->lb[i];
		if (lb->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->lb))
		return ERR_PTR(-EINVAL);

	return lb;
}

void dpu95_lb_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_layerblend *lb = dpu->lb[index];

	dpu95_lb_pec_dynamic_prim_sel(lb, DPU95_LINK_ID_NONE);
	dpu95_lb_pec_dynamic_sec_sel(lb, DPU95_LINK_ID_NONE);
	dpu95_lb_pec_clken(lb, CLKEN_DISABLE);
	dpu95_lb_shdldsel(lb, BOTH);
	dpu95_lb_shdtoksel(lb, BOTH);
	dpu95_lb_enable_shden(lb);
}

int dpu95_lb_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_layerblend *lb;

	lb = devm_kzalloc(dpu->dev, sizeof(*lb), GFP_KERNEL);
	if (!lb)
		return -ENOMEM;

	dpu->lb[index] = lb;

	lb->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!lb->pec_base)
		return -ENOMEM;

	lb->base = devm_ioremap(dpu->dev, base, SZ_32);
	if (!lb->base)
		return -ENOMEM;

	lb->dpu = dpu;
	lb->id = id;
	lb->index = index;
	lb->link_id = dpu95_lb_link_id[index];

	return 0;
}
