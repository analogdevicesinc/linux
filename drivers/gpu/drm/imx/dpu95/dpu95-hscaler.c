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

#define DPU95_HSCALER_NO_STREAM_ID	(~0)

struct dpu95_hscaler {
	void __iomem *pec_base;
	void __iomem *base;
	unsigned int id;
	unsigned int index;
	unsigned int stream_id;
	enum dpu95_link_id link_id;
	struct dpu95_soc *dpu;
	const struct dpu95_hscaler_ops *ops;
};

static const enum dpu95_link_id dpu95_hs_link_id[] = {
	DPU95_LINK_ID_HSCALER4, DPU95_LINK_ID_HSCALER9
};

static const enum dpu95_link_id src_sels[2][6] = {
	{
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_FETCHYUV0,
		DPU95_LINK_ID_FETCHYUV1,
		DPU95_LINK_ID_FETCHYUV2,
		DPU95_LINK_ID_FETCHYUV3,
		DPU95_LINK_ID_MATRIX4,
	}, {
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_FILTER9,
	},
};

static inline u32 dpu95_pec_hs_read(struct dpu95_hscaler *hs,
				    unsigned int offset)
{
	return readl(hs->pec_base + offset);
}

static inline void dpu95_pec_hs_write(struct dpu95_hscaler *hs,
				      unsigned int offset, u32 value)
{
	writel(value, hs->pec_base + offset);
}

static inline void dpu95_pec_hs_write_mask(struct dpu95_hscaler *hs,
					   unsigned int offset,
					   u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_pec_hs_read(hs, offset);
	tmp &= ~mask;
	dpu95_pec_hs_write(hs, offset, tmp | value);
}

static inline u32 dpu95_hs_read(struct dpu95_hscaler *hs, unsigned int offset)
{
	return readl(hs->base + offset);
}

static inline void dpu95_hs_write(struct dpu95_hscaler *hs,
				  unsigned int offset, u32 value)
{
	writel(value, hs->base + offset);
}

static inline void dpu95_hs_write_mask(struct dpu95_hscaler *hs,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_hs_read(hs, offset);
	tmp &= ~mask;
	dpu95_hs_write(hs, offset, tmp | value);
}

enum dpu95_link_id dpu95_hs_get_link_id(struct dpu95_hscaler *hs)
{
	return hs->link_id;
}

void dpu95_hs_pec_dynamic_src_sel(struct dpu95_hscaler *hs,
				  enum dpu95_link_id src)
{
	struct dpu95_soc *dpu = hs->dpu;
	int i;

	for (i = 0; i < ARRAY_SIZE(src_sels[hs->index]); i++) {
		if (src_sels[hs->index][i] == src) {
			dpu95_pec_hs_write_mask(hs, PIXENGCFG_DYNAMIC,
						PIXENGCFG_DYNAMIC_SRC_SEL_MASK,
						src);
			return;
		}
	}

	dev_err(dpu->dev, "HScaler%u - invalid source 0x%02x\n", hs->id, src);
}

void dpu95_hs_pec_clken(struct dpu95_hscaler *hs, enum dpu95_pec_clken clken)
{
	dpu95_pec_hs_write_mask(hs, PIXENGCFG_DYNAMIC, CLKEN_MASK, CLKEN(clken));
}

static void dpu95_hs_enable_shden(struct dpu95_hscaler *hs)
{
	dpu95_hs_write_mask(hs, STATICCONTROL, SHDEN, SHDEN);
}

void dpu95_hs_setup1(struct dpu95_hscaler *hs,
		     unsigned int src_w, unsigned int dst_w)
{
	struct dpu95_soc *dpu = hs->dpu;
	u32 scale_factor;
	u64 tmp64;

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
		dev_err(dpu->dev, "HScaler%u - invalid scale factor 0x%08x\n",
			hs->id, scale_factor);
		return;
	}

	dpu95_hs_write(hs, SETUP1, SCALE_FACTOR(scale_factor));

	dev_dbg(dpu->dev, "HScaler%u - scale factor 0x%08x\n",
		hs->id, scale_factor);
}

void dpu95_hs_setup2(struct dpu95_hscaler *hs, u32 phase_offset)
{
	dpu95_hs_write(hs, SETUP2, PHASE_OFFSET(phase_offset));
}

void dpu95_hs_output_size(struct dpu95_hscaler *hs, u32 line_num)
{
	dpu95_hs_write_mask(hs, CONTROL, OUTPUT_SIZE_MASK, OUTPUT_SIZE(line_num));
}

void dpu95_hs_filter_mode(struct dpu95_hscaler *hs,
			  enum dpu95_scaler_filter_mode m)
{
	dpu95_hs_write_mask(hs, CONTROL, FILTER_MODE_MASK, FILTER_MODE(m));
}

void dpu95_hs_scale_mode(struct dpu95_hscaler *hs,
			 enum dpu95_scaler_scale_mode m)
{
	dpu95_hs_write_mask(hs, CONTROL, SCALE_MODE_MASK, SCALE_MODE(m));
}

void dpu95_hs_mode(struct dpu95_hscaler *hs, enum dpu95_scaler_mode m)
{
	dpu95_hs_write_mask(hs, CONTROL, CTRL_MODE_MASK, m);
}

unsigned int dpu95_hs_get_id(struct dpu95_hscaler *hs)
{
	return hs->id;
}

struct dpu95_hscaler *dpu95_hs_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_hscaler *hs;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->hs); i++) {
		hs = dpu->hs[i];
		if (hs->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->hs))
		return ERR_PTR(-EINVAL);

	return hs;
}

void dpu95_hs_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_hscaler *hs = dpu->hs[index];

	dpu95_hs_enable_shden(hs);
	dpu95_hs_setup2(hs, 0);
	dpu95_hs_pec_dynamic_src_sel(hs, DPU95_LINK_ID_NONE);
}

static bool dpu95_hs_is_enabled(struct dpu95_hscaler *hs)
{
	u32 val = dpu95_hs_read(hs, CONTROL);

	return !!(val & SCALER_ACTIVE);
}

static void
dpu95_hs_set_stream_id(struct dpu95_hscaler *hs, unsigned int stream_id)
{
	struct dpu95_soc *dpu = hs->dpu;

	hs->stream_id = stream_id;

	dev_dbg(dpu->dev, "HScaler%u sets stream id %u\n", hs->id, stream_id);
}

static unsigned int dpu95_hs_get_stream_id(struct dpu95_hscaler *hs)
{
	struct dpu95_soc *dpu = hs->dpu;

	dev_dbg(dpu->dev, "HScaler%u gets stream id %u\n",
		hs->id, hs->stream_id);

	return hs->stream_id;
}

static void dpu95_hs_set_no_stream_id(struct dpu95_hscaler *hs)
{
	struct dpu95_soc *dpu = hs->dpu;

	hs->stream_id = DPU95_HSCALER_NO_STREAM_ID;

	dev_dbg(dpu->dev, "HScaler%u sets no stream id\n", hs->id);
}

static bool dpu95_hs_has_stream_id(struct dpu95_hscaler *hs)
{
	struct dpu95_soc *dpu = hs->dpu;
	bool result = hs->stream_id != DPU95_HSCALER_NO_STREAM_ID;

	if (result)
		dev_dbg(dpu->dev, "HScaler%u has stream id\n", hs->id);
	else
		dev_dbg(dpu->dev, "HScaler%u has no stream id\n", hs->id);

	return result;
}

static const struct dpu95_hscaler_ops dpu95_hs_ops = {
	.is_enabled		= dpu95_hs_is_enabled,
	.set_stream_id		= dpu95_hs_set_stream_id,
	.get_stream_id		= dpu95_hs_get_stream_id,
	.set_no_stream_id	= dpu95_hs_set_no_stream_id,
	.has_stream_id		= dpu95_hs_has_stream_id,
};

const struct dpu95_hscaler_ops *dpu95_hs_get_ops(struct dpu95_hscaler *hs)
{
	return hs->ops;
}

int dpu95_hs_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_hscaler *hs;

	hs = devm_kzalloc(dpu->dev, sizeof(*hs), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	dpu->hs[index] = hs;

	hs->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_8);
	if (!hs->pec_base)
		return -ENOMEM;

	hs->base = devm_ioremap(dpu->dev, base, SZ_1K);
	if (!hs->base)
		return -ENOMEM;

	hs->dpu = dpu;
	hs->id = id;
	hs->index = index;
	hs->link_id = dpu95_hs_link_id[index];
	hs->ops = &dpu95_hs_ops;

	return 0;
}
