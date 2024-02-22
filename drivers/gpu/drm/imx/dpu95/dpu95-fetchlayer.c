// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020,2022,2023 NXP
 */

#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"
#include "dpu95-fetchunit.h"

#define SHDLDREQCONTROL		0xc
#define  SHDLDREQSTICKY(lm)	((lm) & 0xff)

static const enum dpu95_link_id dpu95_fl_link_id[] = {
	DPU95_LINK_ID_FETCHLAYER0, DPU95_LINK_ID_FETCHLAYER1,
};

static void dpu95_fl_shdldreq_sticky(struct dpu95_fetchunit *fu, u8 layer_mask)
{
	dpu95_fu_write(fu, SHDLDREQCONTROL, SHDLDREQSTICKY(layer_mask));
}

static void dpu95_fl_set_src_buf_dimensions(struct dpu95_fetchunit *fu,
					    unsigned int w, unsigned int h,
					    const struct drm_format_info *unused1,
					    bool unused2)
{
	dpu95_fu_write(fu, SOURCEBUFFERDIMENSION(fu),
		       LINEWIDTH(w) | LINECOUNT(h));
}

static void dpu95_fl_set_fmt(struct dpu95_fetchunit *fu,
			     const struct drm_format_info *format,
			     enum drm_color_encoding color_encoding,
			     enum drm_color_range color_range,
			     bool unused)
{
	u32 bits = 0, shifts = 0;

	dpu95_fu_set_src_bpp(fu, format->cpp[0] * 8);

	dpu95_fu_write_mask(fu, LAYERPROPERTY(fu), YUVCONVERSIONMODE_MASK,
			    YUVCONVERSIONMODE(YUVCONVERSIONMODE_OFF));

	dpu95_fu_get_pixel_format_bits(fu, format->format, &bits);
	dpu95_fu_get_pixel_format_shifts(fu, format->format, &shifts);

	dpu95_fu_write(fu, COLORCOMPONENTBITS(fu), bits);
	dpu95_fu_write(fu, COLORCOMPONENTSHIFT(fu), shifts);
}

static void
dpu95_fl_set_framedimensions(struct dpu95_fetchunit *fu, unsigned int w,
			     unsigned int h, bool unused)
{
	dpu95_fu_write(fu, FRAMEDIMENSIONS(fu), FRAMEWIDTH(w) | FRAMEHEIGHT(h));
}

static void dpu95_fl_set_ops(struct dpu95_fetchunit *fu)
{
	memcpy(&fu->ops, &dpu95_fu_common_ops, sizeof(dpu95_fu_common_ops));
	fu->ops.set_src_buf_dimensions	= dpu95_fl_set_src_buf_dimensions;
	fu->ops.set_fmt			= dpu95_fl_set_fmt;
	fu->ops.set_framedimensions	= dpu95_fl_set_framedimensions;
}

struct dpu95_fetchunit *dpu95_fl_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->fl); i++) {
		fu = dpu->fl[i];
		if (fu->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->fl))
		return ERR_PTR(-EINVAL);

	return fu;
}

void dpu95_fl_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_fetchunit *fu = dpu->fl[index];

	dpu95_fu_common_hw_init(fu);
	dpu95_fl_shdldreq_sticky(fu, 0xff);
}

int dpu95_fl_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_fetchunit *fu;

	fu = devm_kzalloc(dpu->dev, sizeof(*fu), GFP_KERNEL);
	if (!fu)
		return -ENOMEM;

	dpu->fl[index] = fu;

	fu->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!fu->pec_base)
		return -ENOMEM;

	fu->base = devm_ioremap(dpu->dev, base, SZ_2K);
	if (!fu->base)
		return -ENOMEM;

	fu->dpu = dpu;
	fu->id = id;
	fu->index = index;
	fu->type = type;
	fu->sub_id = 0;
	fu->association_bit = FRAC_PLANE(index);
	fu->link_id = dpu95_fl_link_id[index];
	fu->reg_offset1 = 0x18;
	fu->reg_offset2 = 0x1d8;
	fu->reg_burstbuffermanagement = 0x10;
	fu->reg_burstbufferproperties = 0x1f8;
	snprintf(fu->name, sizeof(fu->name), "FetchLayer%u", id);

	dpu95_fl_set_ops(fu);

	return 0;
}
