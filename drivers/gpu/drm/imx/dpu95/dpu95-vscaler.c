// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2017-2019,2023 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"

#define PIXENGCFG_DYNAMIC		0x8
#define  PIXENGCFG_DYNAMIC_SRC_SEL_MASK	0x3f

#define STATICCONTROL			0x8
#define SETUP1				0xc
#define SETUP2				0x10
#define CONTROL				0x14

#define DPU95_VSCALER_NO_STREAM_ID	(~0)

struct dpu95_vscaler {
	void __iomem *pec_base;
	void __iomem *base;
	unsigned int id;
	unsigned int index;
	unsigned int stream_id;
	enum dpu95_link_id link_id;
	struct dpu95_soc *dpu;
	const struct dpu95_vscaler_ops *ops;
};

static const enum dpu95_link_id dpu95_vs_link_id[] = {
	DPU95_LINK_ID_VSCALER4, DPU95_LINK_ID_VSCALER9
};

static const enum dpu95_link_id src_sels[2][7] = {
	{
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_FETCHYUV0,
		DPU95_LINK_ID_FETCHYUV1,
		DPU95_LINK_ID_FETCHYUV2,
		DPU95_LINK_ID_FETCHYUV3,
		DPU95_LINK_ID_HSCALER4,
		DPU95_LINK_ID_MATRIX4,
	}, {
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_HSCALER9,
	},
};

static inline u32 dpu95_pec_vs_read(struct dpu95_vscaler *vs,
				    unsigned int offset)
{
	return readl(vs->pec_base + offset);
}

static inline void dpu95_pec_vs_write(struct dpu95_vscaler *vs,
				      unsigned int offset, u32 value)
{
	writel(value, vs->pec_base + offset);
}

static inline void dpu95_pec_vs_write_mask(struct dpu95_vscaler *vs,
					   unsigned int offset,
					   u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_pec_vs_read(vs, offset);
	tmp &= ~mask;
	dpu95_pec_vs_write(vs, offset, tmp | value);
}

static inline u32 dpu95_vs_read(struct dpu95_vscaler *vs, unsigned int offset)
{
	return readl(vs->base + offset);
}

static inline void dpu95_vs_write(struct dpu95_vscaler *vs,
				  unsigned int offset, u32 value)
{
	writel(value, vs->base + offset);
}

static inline void dpu95_vs_write_mask(struct dpu95_vscaler *vs,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_vs_read(vs, offset);
	tmp &= ~mask;
	dpu95_vs_write(vs, offset, tmp | value);
}

enum dpu95_link_id dpu95_vs_get_link_id(struct dpu95_vscaler *vs)
{
	return vs->link_id;
}

void dpu95_vs_pec_dynamic_src_sel(struct dpu95_vscaler *vs,
				  enum dpu95_link_id src)
{
	struct dpu95_soc *dpu = vs->dpu;
	int i;

	for (i = 0; i < ARRAY_SIZE(src_sels[vs->index]); i++) {
		if (src_sels[vs->index][i] == src) {
			dpu95_pec_vs_write_mask(vs, PIXENGCFG_DYNAMIC,
						PIXENGCFG_DYNAMIC_SRC_SEL_MASK,
						src);
			return;
		}
	}

	dev_err(dpu->dev, "VScaler%u - invalid source 0x%02x\n", vs->id, src);
}

void dpu95_vs_pec_clken(struct dpu95_vscaler *vs, enum dpu95_pec_clken clken)
{
	dpu95_pec_vs_write_mask(vs, PIXENGCFG_DYNAMIC, CLKEN_MASK, CLKEN(clken));
}

static void dpu95_vs_enable_shden(struct dpu95_vscaler *vs)
{
	dpu95_vs_write_mask(vs, STATICCONTROL, SHDEN, SHDEN);
}

void dpu95_vs_setup1(struct dpu95_vscaler *vs,
		     unsigned int src_w, unsigned int dst_w, bool deinterlace)
{
	struct dpu95_soc *dpu = vs->dpu;
	u32 scale_factor;
	u64 tmp64;

	if (deinterlace)
		dst_w *= 2;

	if (src_w == dst_w) {
		scale_factor = 0x80000;
	} else {
		if (src_w > dst_w) {
			tmp64 = (u64)((u64)dst_w * 0x80000);
			do_div(tmp64, src_w);

		} else {
			tmp64 = (u64)((u64)src_w * 0x80000);
			do_div(tmp64, dst_w);
		}
		scale_factor = (u32)tmp64;
	}

	if (scale_factor > 0x80000) {
		dev_err(dpu->dev, "VScaler%u - invalid scale factor 0x%08x\n",
			vs->id, scale_factor);
		return;
	}

	dpu95_vs_write(vs, SETUP1, SCALE_FACTOR(scale_factor));

	dev_dbg(dpu->dev, "VScaler%u - scale factor 0x%08x\n",
		vs->id, scale_factor);
}

void dpu95_vs_setup2(struct dpu95_vscaler *vs, bool deinterlace)
{
	/* 0x20000: +0.25 phase offset for deinterlace */
	u32 phase_offset = deinterlace ? 0x20000 : 0;

	dpu95_vs_write(vs, SETUP2, PHASE_OFFSET(phase_offset));
}

void dpu95_vs_output_size(struct dpu95_vscaler *vs, u32 line_num)
{
	dpu95_vs_write_mask(vs, CONTROL, OUTPUT_SIZE_MASK, OUTPUT_SIZE(line_num));
}

void dpu95_vs_filter_mode(struct dpu95_vscaler *vs,
			  enum dpu95_scaler_filter_mode m)
{
	dpu95_vs_write_mask(vs, CONTROL, FILTER_MODE_MASK, FILTER_MODE(m));
}

void dpu95_vs_scale_mode(struct dpu95_vscaler *vs,
			 enum dpu95_scaler_scale_mode m)
{
	dpu95_vs_write_mask(vs, CONTROL, SCALE_MODE_MASK, SCALE_MODE(m));
}

void dpu95_vs_mode(struct dpu95_vscaler *vs, enum dpu95_scaler_mode m)
{
	dpu95_vs_write_mask(vs, CONTROL, CTRL_MODE_MASK, m);
}

unsigned int dpu95_vs_get_id(struct dpu95_vscaler *vs)
{
	return vs->id;
}

struct dpu95_vscaler *dpu95_vs_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_vscaler *vs;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->vs); i++) {
		vs = dpu->vs[i];
		if (vs->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->vs))
		return ERR_PTR(-EINVAL);

	return vs;
}

void dpu95_vs_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_vscaler *vs = dpu->vs[index];

	dpu95_vs_enable_shden(vs);
	dpu95_vs_setup2(vs, false);
	dpu95_vs_pec_dynamic_src_sel(vs, DPU95_LINK_ID_NONE);
}

static bool dpu95_vs_is_enabled(struct dpu95_vscaler *vs)
{
	u32 val = dpu95_vs_read(vs, CONTROL);

	return !!(val & SCALER_ACTIVE);
}

static void
dpu95_vs_set_stream_id(struct dpu95_vscaler *vs, unsigned int stream_id)
{
	struct dpu95_soc *dpu = vs->dpu;

	vs->stream_id = stream_id;

	dev_dbg(dpu->dev, "VScaler%u sets stream id %u\n", vs->id, stream_id);
}

static unsigned int dpu95_vs_get_stream_id(struct dpu95_vscaler *vs)
{
	struct dpu95_soc *dpu = vs->dpu;

	dev_dbg(dpu->dev, "VScaler%u gets stream id %u\n",
		vs->id, vs->stream_id);

	return vs->stream_id;
}

static void dpu95_vs_set_no_stream_id(struct dpu95_vscaler *vs)
{
	struct dpu95_soc *dpu = vs->dpu;

	vs->stream_id = DPU95_VSCALER_NO_STREAM_ID;

	dev_dbg(dpu->dev, "VScaler%u sets no stream id\n", vs->id);
}

static bool dpu95_vs_has_stream_id(struct dpu95_vscaler *vs)
{
	struct dpu95_soc *dpu = vs->dpu;
	bool result = vs->stream_id != DPU95_VSCALER_NO_STREAM_ID;

	if (result)
		dev_dbg(dpu->dev, "VScaler%u has stream id\n", vs->id);
	else
		dev_dbg(dpu->dev, "VScaler%u has no stream id\n", vs->id);

	return result;
}

static const struct dpu95_vscaler_ops dpu95_vs_ops = {
	.is_enabled		= dpu95_vs_is_enabled,
	.set_stream_id		= dpu95_vs_set_stream_id,
	.get_stream_id		= dpu95_vs_get_stream_id,
	.set_no_stream_id	= dpu95_vs_set_no_stream_id,
	.has_stream_id		= dpu95_vs_has_stream_id,
};

const struct dpu95_vscaler_ops *dpu95_vs_get_ops(struct dpu95_vscaler *vs)
{
	return vs->ops;
}

int dpu95_vs_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_vscaler *vs;

	vs = devm_kzalloc(dpu->dev, sizeof(*vs), GFP_KERNEL);
	if (!vs)
		return -ENOMEM;

	dpu->vs[index] = vs;

	vs->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_8);
	if (!vs->pec_base)
		return -ENOMEM;

	vs->base = devm_ioremap(dpu->dev, base, SZ_1K);
	if (!vs->base)
		return -ENOMEM;

	vs->dpu = dpu;
	vs->id = id;
	vs->index = index;
	vs->link_id = dpu95_vs_link_id[index];
	vs->ops = &dpu95_vs_ops;

	return 0;
}
