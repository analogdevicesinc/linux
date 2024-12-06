/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2019,2020,2022,2023 NXP
 */

#ifndef __DPU95_FETCHUNIT_H__
#define __DPU95_FETCHUNIT_H__

#include <linux/io.h>

#include "dpu95.h"

#define REG_OFFSET1			((fu)->reg_offset1)
#define REG_OFFSET2			((fu)->reg_offset2)
#define SUBID_OFFSET			(((fu)->sub_id) * 0x38)

#define STATICCONTROL			0x8

#define BURSTBUFFERMANAGEMENT(fu)	((fu)->reg_burstbuffermanagement)
#define  COMBINERLINEFLUSH_ENABLE	BIT(30)
#define  COMBINERTIMEOUT_ENABLE		BIT(29)
#define  COMBINERTIMEOUT(n)		(((n) & 0xff) << 21)
#define  COMBINERTIMEOUT_MASK		0x1fe00000
#define  SETMAXBURSTLENGTH(n)		((((n) - 1) & 0xff) << 13)
#define  SETMAXBURSTLENGTH_MASK		0x1fe000
#define  SETBURSTLENGTH(n)		(((n) & 0x1f) << 8)
#define  SETBURSTLENGTH_MASK		0x1f00
#define  SETNUMBUFFERS(n)		((n) & 0xff)
#define  SETNUMBUFFERS_MASK		0xff
#define  LINEMODE_MASK			0x80000000
#define  LINEMODE_SHIFT			31
enum dpu95_linemode {
	/*
	 * Mandatory setting for operation in the Display Controller.
	 * Works also for Blit Engine with marginal performance impact.
	 */
	LINEMODE_DISPLAY = 0,
	/* Recommended setting for operation in the Blit Engine. */
	LINEMODE_BLIT = (1 << LINEMODE_SHIFT),
};

#define BASEADDRESS(fu)			(0x00 + SUBID_OFFSET + REG_OFFSET1)
#define BASEADDRESSMSB(fu)		(0x04 + SUBID_OFFSET + REG_OFFSET1)
#define AUTOUPDATEBASEADDRESS(fu)	(0x08 + SUBID_OFFSET + REG_OFFSET1)
#define AUTOUPDATEBASEADDRESS_MSB(fu)	(0x0c + SUBID_OFFSET + REG_OFFSET1)

#define SOURCEBUFFERATTRIBUTES(fu)	(0x10 + SUBID_OFFSET + REG_OFFSET1)
#define  BITSPERPIXEL_MASK		0x3f0000
#define  BITSPERPIXEL(bpp)		(((bpp) & 0x3f) << 16)
#define  STRIDE_MASK			0xffff
#define  STRIDE(n)			(((n) - 1) & 0xffff)

#define SOURCEBUFFERDIMENSION(fu)	(0x14 + SUBID_OFFSET + REG_OFFSET1)
#define  LINEWIDTH(w)			(((w) - 1) & 0x3fff)
#define  LINECOUNT(h)			((((h) - 1) & 0x3fff) << 16)

#define COLORCOMPONENTBITS(fu)		(0x18 + SUBID_OFFSET + REG_OFFSET1)
#define  ITUFORMAT			BIT(31)
#define  R_BITS(n)			(((n) & 0xf) << 24)
#define  G_BITS(n)			(((n) & 0xf) << 16)
#define  B_BITS(n)			(((n) & 0xf) << 8)
#define  A_BITS(n)			((n) & 0xf)
#define  Y_BITS(n)			R_BITS(n)
#define  Y_BITS_MASK			0xf000000
#define  U_BITS(n)			G_BITS(n)
#define  U_BITS_MASK			0xf0000
#define  V_BITS(n)			B_BITS(n)
#define  V_BITS_MASK			0xf00

#define COLORCOMPONENTSHIFT(fu)		(0x1c + SUBID_OFFSET + REG_OFFSET1)
#define  R_SHIFT(n)			(((n) & 0x1f) << 24)
#define  G_SHIFT(n)			(((n) & 0x1f) << 16)
#define  B_SHIFT(n)			(((n) & 0x1f) << 8)
#define  A_SHIFT(n)			((n) & 0x1f)
#define  Y_SHIFT(n)			R_SHIFT(n)
#define  Y_SHIFT_MASK			0x1f000000
#define  U_SHIFT(n)			G_SHIFT(n)
#define  U_SHIFT_MASK			0x1f0000
#define  V_SHIFT(n)			B_SHIFT(n)
#define  V_SHIFT_MASK			0x1f00

#define LAYEROFFSET(fu)			(0x20 + SUBID_OFFSET + REG_OFFSET1)
#define  LAYERXOFFSET(x)		((x) & 0x7fff)
#define  LAYERYOFFSET(y)		(((y) & 0x7fff) << 16)

#define CLIPWINDOWOFFSET(fu)		(0x24 + SUBID_OFFSET + REG_OFFSET1)
#define  CLIPWINDOWXOFFSET(x)		((x) & 0x7fff)
#define  CLIPWINDOWYOFFSET(y)		(((y) & 0x7fff) << 16)

#define CLIPWINDOWDIMENSIONS(fu)	(0x28 + SUBID_OFFSET + REG_OFFSET1)
#define  CLIPWINDOWWIDTH(w)		((w) & 0x3fff)
#define  CLIPWINDOWHEIGHT(h)		(((h) & 0x3fff) << 16)

#define CONSTANTCOLOR(fu)		(0x2c + SUBID_OFFSET + REG_OFFSET1)
#define  CONSTANTALPHA_MASK		0xff
#define  CONSTANTALPHA(n)		((n) & CONSTANTALPHA_MASK)

#define LAYERPROPERTY(fu)		(0x30 + SUBID_OFFSET + REG_OFFSET1)
#define	 PALETTEENABLE			BIT(0)
#define  TILEMODE(n)			(((n) & 0x3) << 4)
#define  TILEMODE_MASK			0x30
enum dpu95_tilemode {
	TILE_FILL_ZERO,
	TILE_FILL_CONSTANT,
	TILE_PAD,
	TILE_PAD_ZERO,
};

#define  ALPHASRCENABLE			BIT(8)
#define  ALPHACONSTENABLE		BIT(9)
#define  ALPHAMASKENABLE		BIT(10)
#define  ALPHATRANSENABLE		BIT(11)
#define  ALPHA_ENABLE_MASK		(ALPHASRCENABLE | ALPHACONSTENABLE | \
					 ALPHAMASKENABLE | ALPHATRANSENABLE)
#define  RGBALPHASRCENABLE		BIT(12)
#define  RGBALPHACONSTENABLE		BIT(13)
#define  RGBALPHAMASKENABLE		BIT(14)
#define  RGBALPHATRANSENABLE		BIT(15)
#define  RGB_ENABLE_MASK		(RGBALPHASRCENABLE |	\
					 RGBALPHACONSTENABLE |	\
					 RGBALPHAMASKENABLE |	\
					 RGBALPHATRANSENABLE)
#define  PREMULCONSTRGB			BIT(16)
enum dpu95_yuvconversionmode {
	YUVCONVERSIONMODE_OFF,
	YUVCONVERSIONMODE_ITU601,
	YUVCONVERSIONMODE_ITU601_FR,
	YUVCONVERSIONMODE_ITU709,
};

#define  YUVCONVERSIONMODE_MASK		0x60000
#define  YUVCONVERSIONMODE(m)		(((m) & 0x3) << 17)
#define  GAMMAREMOVEENABLE		BIT(20)
#define  CLIPWINDOWENABLE		BIT(30)
#define  SOURCEBUFFERENABLE		BIT(31)

#define FRAMEDIMENSIONS(fu)		(0x00 + REG_OFFSET2)
#define  EMPTYFRAME			BIT(31)
#define  FRAMEWIDTH(w)			(((w) - 1) & 0x3fff)
#define  FRAMEHEIGHT(h)			((((h) - 1) & 0x3fff) << 16)

#define FRAMERESAMPLING(fu)		(0x04 + REG_OFFSET2)
#define  DELTAX_MASK			0x3f000
#define  DELTAY_MASK			0xfc0000
#define  DELTAX(x)			(((x) & 0x3f) << 12)
#define  DELTAY(y)			(((y) & 0x3f) << 18)
#define  STARTX_MASK			0x3f
#define  STARTY_MASK			0xfc0
#define  STARTX(x)			((x) & 0x3f)
#define  STARTY(y)			(((y) & 0x3f) << 6)

#define CONTROL(fu)			(0x08 + REG_OFFSET2)
#define  CLIPLAYER_MASK			0x1e0000
#define  CLIPLAYER(n)			(((n) & 0xf) << 17)

#define  CLIPCOLOR_MASK			BIT(16)
#define  CLIPCOLOR(n)			(((n) & 0x1) << 16)
enum dpu95_clipcolor {
	CLIPCOLOR_NULL,
	CLIPCOLOR_LAYER,
};

#define  YUV420UPSAMPLINGMODE_MASK	0x6000
#define  YUV420UPSAMPLINGMODE(m)	(((m) & 0x3) << 13)
enum dpu95_yuv420upsamplingmode {
	YUV420UPSAMPLINGMODE_INACTIVE,
	YUV420UPSAMPLINGMODE_REPLICATE,
	YUV420UPSAMPLINGMODE_INTERPOLATE,
	YUV420UPSAMPLINGMODE_INTERLACED,
};

#define  SECINPUTSELECT_MASK		0x1800
#define  SECINPUTSELECT(s)		(((s) & 0x3) << 11)

#define  PALETTEIDXWIDTH_MASK		0x700
#define  PALETTEIDXWIDTH(n)		(((n) & 0x7) << 8)

#define  RAWPIXEL			BIT(7)
#define  CHROMAHREPLEN			BIT(6)
#define  YUV422UPSAMPLINGMODE_MASK	BIT(5)
#define  YUV422UPSAMPLINGMODE(m)	(((m) & 0x1) << 5)
enum dpu95_yuv422upsamplingmode {
	YUV422UPSAMPLINGMODE_REPLICATE,
	YUV422UPSAMPLINGMODE_INTERPOLATE,
};

#define  INPUTSELECT_MASK		0x18
#define  INPUTSELECT(s)			(((s) & 0x3) << 3)
enum dpu95_inputselect {
	INPUTSELECT_INACTIVE,
	INPUTSELECT_COMPPACK,
	INPUTSELECT_ALPHAMASK,
	INPUTSELECT_COORDINATE,
};

#define  RASTERMODE_MASK		0x7
#define  RASTERMODE(m)			((m) & 0x7)
enum dpu95_rastermode {
	RASTERMODE_NORMAL,
	RASTERMODE_DECODE,
	RASTERMODE_ARBITRARY,
	RASTERMODE_PERSPECTIVE,
	RASTERMODE_YUV422,
	RASTERMODE_AFFINE,
	RASTERMODE_YVU422,
};

#define CONTROLTRIGGER(fu)		(0x0c + REG_OFFSET2)
#define  SHDTOKGEN			BIT(0)

#define START(fu)			(0x10 + REG_OFFSET2)

#define FETCHTYPE(fu)			(0x14 + REG_OFFSET2)
#define  FETCHTYPE_MASK			0xf

#define BURSTBUFFERPROPERTIES(fu)	((fu)->reg_burstbufferproperties)

/* register in blk-ctrl */
#define PLANE_ASSOCIATION		0x20
#define  VIDEO_PLANE(n)			BIT(8 + 2 * (n))
#define  INT_PLANE			BIT(6)
#define  FRAC_PLANE(n)			BIT(2 * (n))

struct dpu95_fetchunit {
	void __iomem *pec_base;
	void __iomem *base;
	char name[12];
	struct list_head node;
	unsigned int reg_offset;
	unsigned int id;
	unsigned int index;
	unsigned int sub_id;	/* for fractional fetch units */
	unsigned int stream_id;
	unsigned int association_bit;
	enum dpu95_unit_type type;
	enum dpu95_link_id link_id;
	u32 cap_mask;
	bool is_available;
	struct dpu95_soc *dpu;
	struct dpu95_fetchunit_ops ops;
	struct dpu95_fetchunit *fe;
	struct dpu95_hscaler *hs;
	struct dpu95_layerblend *lb;
	u32 reg_offset1;
	u32 reg_offset2;
	u32 reg_burstbuffermanagement;
	u32 reg_burstbufferproperties;
};

extern const struct dpu95_fetchunit_ops dpu95_fu_common_ops;

static inline void
dpu95_pec_fu_write(struct dpu95_fetchunit *fu, unsigned int offset, u32 value)
{
	writel(value, fu->pec_base + offset);
}

static inline u32 dpu95_pec_fu_read(struct dpu95_fetchunit *fu,
				    unsigned int offset)
{
	return readl(fu->pec_base + offset);
}

static inline u32 dpu95_fu_read(struct dpu95_fetchunit *fu, unsigned int offset)
{
	return readl(fu->base + offset);
}

static inline void
dpu95_fu_write(struct dpu95_fetchunit *fu, unsigned int offset, u32 value)
{
	writel(value, fu->base + offset);
}

static inline void dpu95_fu_write_mask(struct dpu95_fetchunit *fu,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_fu_read(fu, offset);
	tmp &= ~mask;
	dpu95_fu_write(fu, offset, tmp | value);
}

void dpu95_fu_get_pixel_format_bits(struct dpu95_fetchunit *fu,
				    u32 format, u32 *bits);
void dpu95_fu_get_pixel_format_shifts(struct dpu95_fetchunit *fu,
				      u32 format, u32 *shifts);
void dpu95_fu_set_src_bpp(struct dpu95_fetchunit *fu, unsigned int bpp);
void dpu95_fu_common_hw_init(struct dpu95_fetchunit *fu);

#endif /* __DPU95_FETCHUNIT_H__ */
