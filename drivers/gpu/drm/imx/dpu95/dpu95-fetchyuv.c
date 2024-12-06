// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2022,2023 NXP
 */

#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"
#include "dpu95-fetchunit.h"

#define PIXENGCFG_DYNAMIC	0x8

#define DPU95_FETCHYUV_CAP_MASK	(DPU95_FETCHUNIT_CAP_USE_FETCHECO | \
				 DPU95_FETCHUNIT_CAP_USE_SCALER |   \
				 DPU95_FETCHUNIT_CAP_PACKED_YUV422)

static const enum dpu95_link_id dpu95_fy_link_id[] = {
	DPU95_LINK_ID_FETCHYUV0, DPU95_LINK_ID_FETCHYUV1,
	DPU95_LINK_ID_FETCHYUV2, DPU95_LINK_ID_FETCHYUV3,
};

static const enum dpu95_link_id fy_srcs[4][2] = {
	{
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_FETCHECO0,
	}, {
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_FETCHECO1,
	}, {
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_FETCHECO2,
	}, {
		DPU95_LINK_ID_NONE,
		DPU95_LINK_ID_FETCHECO9,
	},
};

static void dpu95_fy_pec_dynamic_src_sel(struct dpu95_fetchunit *fu,
					 enum dpu95_link_id src)
{
	struct dpu95_soc *dpu = fu->dpu;
	int i;

	for (i = 0; i < ARRAY_SIZE(fy_srcs[fu->index]); i++) {
		if (fy_srcs[fu->index][i] == src) {
			dpu95_pec_fu_write(fu, PIXENGCFG_DYNAMIC, src);
			return;
		}
	}

	dev_err(dpu->dev, "%s - invalid source 0x%02x\n", fu->name, src);
}

static void
dpu95_fy_set_src_buf_dimensions(struct dpu95_fetchunit *fu,
				unsigned int w, unsigned int h,
				const struct drm_format_info *unused,
				bool deinterlace)
{
	if (deinterlace)
		h /= 2;

	dpu95_fu_write(fu, SOURCEBUFFERDIMENSION(fu),
		       LINEWIDTH(w) | LINECOUNT(h));
}

static void dpu95_fy_set_fmt(struct dpu95_fetchunit *fu,
			     const struct drm_format_info *format,
			     enum drm_color_encoding color_encoding,
			     enum drm_color_range color_range,
			     bool deinterlace)
{
	u32 val, bits = 0, shifts = 0;
	bool is_planar_yuv = false, is_rastermode_yuv422 = false;
	bool is_yuv422upsamplingmode_interpolate = false;
	bool is_inputselect_compact = false;
	unsigned int bpp;

	switch (format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		is_rastermode_yuv422 = true;
		is_yuv422upsamplingmode_interpolate = true;
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		if (deinterlace)
			is_yuv422upsamplingmode_interpolate = true;
		is_planar_yuv = true;
		is_rastermode_yuv422 = true;
		is_inputselect_compact = true;
		bpp = format->cpp[0] * 8;
		break;
	default:
		bpp = format->cpp[0] * 8;
		break;
	}

	dpu95_fu_set_src_bpp(fu, bpp);

	val = dpu95_fu_read(fu, CONTROL(fu));
	val &= ~YUV422UPSAMPLINGMODE_MASK;
	val &= ~INPUTSELECT_MASK;
	val &= ~RASTERMODE_MASK;
	if (is_yuv422upsamplingmode_interpolate)
		val |= YUV422UPSAMPLINGMODE(YUV422UPSAMPLINGMODE_INTERPOLATE);
	else
		val |= YUV422UPSAMPLINGMODE(YUV422UPSAMPLINGMODE_REPLICATE);
	if (is_inputselect_compact)
		val |= INPUTSELECT(INPUTSELECT_COMPPACK);
	else
		val |= INPUTSELECT(INPUTSELECT_INACTIVE);
	if (is_rastermode_yuv422)
		val |= RASTERMODE(RASTERMODE_YUV422);
	else
		val |= RASTERMODE(RASTERMODE_NORMAL);
	dpu95_fu_write(fu, CONTROL(fu), val);

	val = dpu95_fu_read(fu, LAYERPROPERTY(fu));
	val &= ~YUVCONVERSIONMODE_MASK;
	if (format->is_yuv) {
		if (color_encoding == DRM_COLOR_YCBCR_BT709)
			val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE_ITU709);
		else if (color_encoding == DRM_COLOR_YCBCR_BT601 &&
			 color_range == DRM_COLOR_YCBCR_FULL_RANGE)
			val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE_ITU601_FR);
		else
			val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE_ITU601);
	} else {
		val |= YUVCONVERSIONMODE(YUVCONVERSIONMODE_OFF);
	}
	dpu95_fu_write(fu, LAYERPROPERTY(fu), val);

	dpu95_fu_get_pixel_format_bits(fu, format->format, &bits);
	dpu95_fu_get_pixel_format_shifts(fu, format->format, &shifts);

	if (is_planar_yuv) {
		bits &= ~(U_BITS_MASK | V_BITS_MASK);
		shifts &= ~(U_SHIFT_MASK | V_SHIFT_MASK);
	}

	dpu95_fu_write(fu, COLORCOMPONENTBITS(fu), bits);
	dpu95_fu_write(fu, COLORCOMPONENTSHIFT(fu), shifts);
}

static void dpu95_fy_set_framedimensions(struct dpu95_fetchunit *fu,
					 unsigned int w, unsigned int h,
					 bool deinterlace)
{
	if (deinterlace)
		h /= 2;

	dpu95_fu_write(fu, FRAMEDIMENSIONS(fu), FRAMEWIDTH(w) | FRAMEHEIGHT(h));
}

static void dpu95_fy_set_ops(struct dpu95_fetchunit *fu)
{
	memcpy(&fu->ops, &dpu95_fu_common_ops, sizeof(dpu95_fu_common_ops));
	fu->ops.set_pec_dynamic_src_sel = dpu95_fy_pec_dynamic_src_sel;
	fu->ops.set_src_buf_dimensions	= dpu95_fy_set_src_buf_dimensions;
	fu->ops.set_fmt			= dpu95_fy_set_fmt;
	fu->ops.set_framedimensions	= dpu95_fy_set_framedimensions;
}

struct dpu95_fetchunit *dpu95_fy_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_fetchunit *fu;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->fy); i++) {
		fu = dpu->fy[i];
		if (fu->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->fy))
		return ERR_PTR(-EINVAL);

	fu->fe = dpu95_fe_get(dpu, id == 3 ? 9 : id);
	if (IS_ERR(fu->fe))
		return ERR_CAST(fu->fe);

	fu->hs = dpu95_hs_get(dpu, fu->type == DPU95_DISP ? 4 : 9);
	if (IS_ERR(fu->hs))
		return ERR_CAST(fu->hs);

	return fu;
}

void dpu95_fy_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_fetchunit *fu = dpu->fy[index];

	fu->ops.set_pec_dynamic_src_sel(fu, DPU95_LINK_ID_NONE);
	dpu95_fu_common_hw_init(fu);
}

int dpu95_fy_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base)
{
	struct dpu95_fetchunit *fu;

	fu = devm_kzalloc(dpu->dev, sizeof(*fu), GFP_KERNEL);
	if (!fu)
		return -ENOMEM;

	dpu->fy[index] = fu;

	fu->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_16);
	if (!fu->pec_base)
		return -ENOMEM;

	fu->base = devm_ioremap(dpu->dev, base, SZ_256);
	if (!fu->base)
		return -ENOMEM;

	fu->dpu = dpu;
	fu->id = id;
	fu->index = index;
	fu->type = type;
	fu->association_bit = id == 3 ? INT_PLANE : VIDEO_PLANE(index);
	fu->link_id = dpu95_fy_link_id[index];
	fu->cap_mask = DPU95_FETCHYUV_CAP_MASK;
	fu->reg_offset1 = 0x28;
	fu->reg_offset2 = 0x60;
	fu->reg_burstbuffermanagement = 0x0c;
	fu->reg_burstbufferproperties = 0x80;

	snprintf(fu->name, sizeof(fu->name), "FetchYUV%u", id);

	dpu95_fy_set_ops(fu);

	return 0;
}
