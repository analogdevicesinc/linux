// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2018-2021,2023 NXP
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include <drm/drm_blend.h>

#include "dpu95-fetchunit.h"

#define DPU95_FETCHUNIT_NO_STREAM_ID	(~0)

struct dpu95_fetchunit_pixel_format {
	u32 pixel_format;
	u32 bits;
	u32 shifts;
};

static const struct dpu95_fetchunit_pixel_format pixel_formats[] = {
	{
		DRM_FORMAT_ARGB8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(16) | G_SHIFT(8)  | B_SHIFT(0)  | A_SHIFT(24),
	}, {
		DRM_FORMAT_XRGB8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(16) | G_SHIFT(8)  | B_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_ABGR8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(0)  | G_SHIFT(8)  | B_SHIFT(16) | A_SHIFT(24),
	}, {
		DRM_FORMAT_XBGR8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(0)  | G_SHIFT(8)  | B_SHIFT(16) | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGBA8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(24) | G_SHIFT(16) | B_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGBX8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(24) | G_SHIFT(16) | B_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_BGRA8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(8)  | G_SHIFT(16) | B_SHIFT(24) | A_SHIFT(0),
	}, {
		DRM_FORMAT_BGRX8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(8)  | G_SHIFT(16) | B_SHIFT(24) | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGB888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(16) | G_SHIFT(8)  | B_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_BGR888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(0)  | G_SHIFT(8)  | B_SHIFT(16) | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGB565,
		R_BITS(5)   | G_BITS(6)   | B_BITS(5)   | A_BITS(0),
		R_SHIFT(11) | G_SHIFT(5)  | B_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_YUYV,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(8)  | V_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_UYVY,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(8)  | U_SHIFT(0)  | V_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV12,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(0)  | V_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV21,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(8)  | V_SHIFT(0)  | A_SHIFT(0),
	},
};

void dpu95_fu_get_pixel_format_bits(struct dpu95_fetchunit *fu,
				    u32 format, u32 *bits)
{
	struct dpu95_soc *dpu = fu->dpu;
	int i;

	for (i = 0; i < ARRAY_SIZE(pixel_formats); i++) {
		if (pixel_formats[i].pixel_format == format) {
			*bits = pixel_formats[i].bits;
			return;
		}
	}

	dev_warn(dpu->dev, "%s - unsupported pixel format 0x%08x\n",
		 fu->name, format);
}

void dpu95_fu_get_pixel_format_shifts(struct dpu95_fetchunit *fu,
				      u32 format, u32 *shifts)
{
	struct dpu95_soc *dpu = fu->dpu;
	int i;

	for (i = 0; i < ARRAY_SIZE(pixel_formats); i++) {
		if (pixel_formats[i].pixel_format == format) {
			*shifts = pixel_formats[i].shifts;
			return;
		}
	}

	dev_warn(dpu->dev, "%s - unsupported pixel format 0x%08x\n",
		 fu->name, format);
}

static bool dpu95_fu_is_enabled(struct dpu95_fetchunit *fu)
{
	u32 val = dpu95_fu_read(fu, LAYERPROPERTY(fu));

	return !!(val & SOURCEBUFFERENABLE);
}

static void dpu95_fu_enable_shden(struct dpu95_fetchunit *fu)
{
	dpu95_fu_write_mask(fu, STATICCONTROL, SHDEN, SHDEN);
}

static void dpu95_fu_set_linemode(struct dpu95_fetchunit *fu,
				  enum dpu95_linemode mode)
{
	dpu95_fu_write_mask(fu, BURSTBUFFERMANAGEMENT(fu), LINEMODE_MASK, mode);
}

static void dpu95_fu_set_numbuffers(struct dpu95_fetchunit *fu,
				    unsigned int num)
{
	dpu95_fu_write_mask(fu, BURSTBUFFERMANAGEMENT(fu), SETNUMBUFFERS_MASK,
			    SETNUMBUFFERS(num));
}

static void
dpu95_fu_set_burstlength(struct dpu95_fetchunit *fu, unsigned int burst_length)
{
	dpu95_fu_write_mask(fu, BURSTBUFFERMANAGEMENT(fu), SETBURSTLENGTH_MASK,
			    SETBURSTLENGTH(burst_length));
}

static void
dpu95_fu_set_maxburstlength(struct dpu95_fetchunit *fu, unsigned int burst_length)
{
	dpu95_fu_write_mask(fu, BURSTBUFFERMANAGEMENT(fu),
			    SETMAXBURSTLENGTH_MASK,
			    SETMAXBURSTLENGTH(burst_length));
}

static void dpu95_fu_combinertimeout_disable(struct dpu95_fetchunit *fu)
{
	dpu95_fu_write_mask(fu, BURSTBUFFERMANAGEMENT(fu),
			    COMBINERTIMEOUT_ENABLE, 0);
}

static void dpu95_fu_combinerlineflush_disable(struct dpu95_fetchunit *fu)
{
	dpu95_fu_write_mask(fu, BURSTBUFFERMANAGEMENT(fu),
			    COMBINERLINEFLUSH_ENABLE, 0);
}

static void dpu95_fu_set_baseaddress(struct dpu95_fetchunit *fu,
				     dma_addr_t baddr)
{
	dpu95_fu_write(fu, BASEADDRESS(fu), lower_32_bits(baddr));
	dpu95_fu_write(fu, BASEADDRESSMSB(fu), upper_32_bits(baddr));
}

void dpu95_fu_set_src_bpp(struct dpu95_fetchunit *fu, unsigned int bpp)
{
	dpu95_fu_write_mask(fu, SOURCEBUFFERATTRIBUTES(fu), BITSPERPIXEL_MASK,
			    BITSPERPIXEL(bpp));
}

static void dpu95_fu_set_src_stride(struct dpu95_fetchunit *fu,
				    unsigned int stride)
{
	dpu95_fu_write_mask(fu, SOURCEBUFFERATTRIBUTES(fu), STRIDE_MASK,
			    STRIDE(stride));
}

static void dpu95_fu_layeroffset(struct dpu95_fetchunit *fu, unsigned int x,
				 unsigned int y)
{
	dpu95_fu_write(fu, LAYEROFFSET(fu), LAYERXOFFSET(x) | LAYERYOFFSET(y));
}

static void dpu95_fu_clipoffset(struct dpu95_fetchunit *fu, unsigned int x,
				unsigned int y)
{
	dpu95_fu_write(fu, CLIPWINDOWOFFSET(fu),
		       CLIPWINDOWXOFFSET(x) | CLIPWINDOWYOFFSET(y));
}

static void dpu95_fu_clipdimensions(struct dpu95_fetchunit *fu, unsigned int w,
				    unsigned int h)
{
	dpu95_fu_write(fu, CLIPWINDOWDIMENSIONS(fu),
		       CLIPWINDOWWIDTH(w) | CLIPWINDOWHEIGHT(h));
}

static void dpu95_fu_set_pixel_blend_mode(struct dpu95_fetchunit *fu,
					  unsigned int pixel_blend_mode,
					  u16 alpha, bool fb_format_has_alpha)
{
	u32 mode = 0;

	if (pixel_blend_mode == DRM_MODE_BLEND_PREMULTI ||
	    pixel_blend_mode == DRM_MODE_BLEND_COVERAGE) {
		mode = ALPHACONSTENABLE;

		if (fb_format_has_alpha)
			mode |= ALPHASRCENABLE;
	}

	dpu95_fu_write_mask(fu, LAYERPROPERTY(fu),
			    PREMULCONSTRGB | ALPHA_ENABLE_MASK | RGB_ENABLE_MASK,
			    mode);

	dpu95_fu_write_mask(fu, CONSTANTCOLOR(fu), CONSTANTALPHA_MASK,
			    CONSTANTALPHA(alpha >> 8));
}

static void dpu95_fu_enable_src_buf(struct dpu95_fetchunit *fu)
{
	struct dpu95_soc *dpu = fu->dpu;

	dpu95_fu_write_mask(fu, LAYERPROPERTY(fu), SOURCEBUFFERENABLE,
			    SOURCEBUFFERENABLE);

	dev_dbg(dpu->dev, "%s enables source buffer in shadow\n", fu->name);
}

static void dpu95_fu_disable_src_buf(struct dpu95_fetchunit *fu)
{
	struct dpu95_soc *dpu = fu->dpu;

	if (fu->ops.set_pec_dynamic_src_sel)
		fu->ops.set_pec_dynamic_src_sel(fu, DPU95_LINK_ID_NONE);

	dpu95_fu_write_mask(fu, LAYERPROPERTY(fu), SOURCEBUFFERENABLE, 0);

	if (fu->fe)
		fu->fe->ops.disable_src_buf(fu->fe);

	if (fu->hs) {
		dpu95_hs_pec_clken(fu->hs, CLKEN_DISABLE);
		dpu95_hs_mode(fu->hs, SCALER_NEUTRAL);
	}

	if (fu->lb) {
		dpu95_lb_pec_clken(fu->lb, CLKEN_DISABLE);
		dpu95_lb_mode(fu->lb, LB_NEUTRAL);
	}

	dev_dbg(dpu->dev, "%s disables source buffer in shadow\n", fu->name);
}

static struct dpu95_fetchunit *dpu95_fu_get_fetcheco(struct dpu95_fetchunit *fu)
{
	return fu->fe;
}

static struct dpu95_hscaler *dpu95_fu_get_hscaler(struct dpu95_fetchunit *fu)
{
	return fu->hs;
}

static void
dpu95_fu_set_layerblend(struct dpu95_fetchunit *fu, struct dpu95_layerblend *lb)
{
	fu->lb = lb;
}

static bool dpu95_fu_is_available(struct dpu95_fetchunit *fu)
{
	return fu->is_available;
}

static void dpu95_fu_set_available(struct dpu95_fetchunit *fu)
{
	fu->is_available = true;
}

static void dpu95_fu_set_inavailable(struct dpu95_fetchunit *fu)
{
	fu->is_available = false;
}

static void
dpu95_fu_set_stream_id(struct dpu95_fetchunit *fu, unsigned int stream_id)
{
	struct dpu95_soc *dpu = fu->dpu;
	int ret;

	ret = regmap_update_bits(dpu->regmap, PLANE_ASSOCIATION,
				 fu->association_bit,
				 stream_id ? fu->association_bit : 0);
	if (ret < 0)
		dev_err(dpu->dev, "%s failed to set association bit: %d\n",
			fu->name, ret);

	fu->stream_id = stream_id;

	dev_dbg(dpu->dev, "%s sets stream id %u\n", fu->name, stream_id);
}

static unsigned int dpu95_fu_get_stream_id(struct dpu95_fetchunit *fu)
{
	struct dpu95_soc *dpu = fu->dpu;

	dev_dbg(dpu->dev, "%s gets stream id %u\n", fu->name, fu->stream_id);

	return fu->stream_id;
}

static void dpu95_fu_set_no_stream_id(struct dpu95_fetchunit *fu)
{
	struct dpu95_soc *dpu = fu->dpu;

	fu->stream_id = DPU95_FETCHUNIT_NO_STREAM_ID;

	dev_dbg(dpu->dev, "%s sets no stream id\n", fu->name);
}

static bool dpu95_fu_has_stream_id(struct dpu95_fetchunit *fu)
{
	struct dpu95_soc *dpu = fu->dpu;
	bool result = fu->stream_id != DPU95_FETCHUNIT_NO_STREAM_ID;

	if (result)
		dev_dbg(dpu->dev, "%s has stream id\n", fu->name);
	else
		dev_dbg(dpu->dev, "%s has no stream id\n", fu->name);

	return result;
}

static enum dpu95_link_id dpu95_fu_get_link_id(struct dpu95_fetchunit *fu)
{
	return fu->link_id;
}

static u32 dpu95_fu_get_cap_mask(struct dpu95_fetchunit *fu)
{
	return fu->cap_mask;
}

static const char *dpu95_fu_get_name(struct dpu95_fetchunit *fu)
{
	return fu->name;
}

const struct dpu95_fetchunit_ops dpu95_fu_common_ops = {
	.is_enabled		= dpu95_fu_is_enabled,
	.set_numbuffers		= dpu95_fu_set_numbuffers,
	.set_burstlength	= dpu95_fu_set_burstlength,
	.set_baseaddress	= dpu95_fu_set_baseaddress,
	.set_src_stride		= dpu95_fu_set_src_stride,
	.set_pixel_blend_mode	= dpu95_fu_set_pixel_blend_mode,
	.enable_src_buf		= dpu95_fu_enable_src_buf,
	.disable_src_buf	= dpu95_fu_disable_src_buf,
	.get_fetcheco		= dpu95_fu_get_fetcheco,
	.get_hscaler		= dpu95_fu_get_hscaler,
	.set_layerblend		= dpu95_fu_set_layerblend,
	.is_available		= dpu95_fu_is_available,
	.set_available		= dpu95_fu_set_available,
	.set_inavailable	= dpu95_fu_set_inavailable,
	.set_stream_id		= dpu95_fu_set_stream_id,
	.get_stream_id		= dpu95_fu_get_stream_id,
	.set_no_stream_id	= dpu95_fu_set_no_stream_id,
	.has_stream_id		= dpu95_fu_has_stream_id,
	.get_link_id		= dpu95_fu_get_link_id,
	.get_cap_mask		= dpu95_fu_get_cap_mask,
	.get_name		= dpu95_fu_get_name,
};

const struct dpu95_fetchunit_ops *dpu95_fu_get_ops(struct dpu95_fetchunit *fu)
{
	return &fu->ops;
}

struct dpu95_fetchunit *dpu95_fu_get_from_list(struct list_head *l)
{
	return container_of(l, struct dpu95_fetchunit, node);
}

void dpu95_fu_add_to_list(struct dpu95_fetchunit *fu, struct list_head *l)
{
	list_add(&fu->node, l);
}

void dpu95_fu_common_hw_init(struct dpu95_fetchunit *fu)
{
	dpu95_fu_enable_shden(fu);
	dpu95_fu_set_linemode(fu, LINEMODE_DISPLAY);
	dpu95_fu_layeroffset(fu, 0x0, 0x0);
	dpu95_fu_clipoffset(fu, 0x0, 0x0);
	dpu95_fu_clipdimensions(fu, 0x0, 0x0);
	dpu95_fu_set_numbuffers(fu, 16);
	dpu95_fu_set_burstlength(fu, 16);
	dpu95_fu_set_maxburstlength(fu, 64);
	dpu95_fu_combinertimeout_disable(fu);
	dpu95_fu_combinerlineflush_disable(fu);
	dpu95_fu_disable_src_buf(fu);
	dpu95_fu_set_no_stream_id(fu);
}
