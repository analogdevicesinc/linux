/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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
#ifndef __DPU_PRV_H__
#define __DPU_PRV_H__

#include <drm/drm_fourcc.h>

#define NA				0xDEADBEEF	/* not available */

#define STATICCONTROL			0x8
#define BASEADDRESSAUTOUPDATE(lm)	(((lm) & 0xFF) << 16)
#define BASEADDRESSAUTOUPDATE_MASK	(0xFF << 16)
#define SHDEN				BIT(0)
#define BURSTBUFFERMANAGEMENT		0xC
#define SETNUMBUFFERS(n)		((n) & 0xFF)
#define SETBURSTLENGTH(n)		(((n) & 0x1F) << 8)
#define LINEMODE_MASK			0x80000000U
#define LINEMODE_SHIFT			31U
enum linemode {
	/*
	 * Mandatory setting for operation in the Display Controller.
	 * Works also for Blit Engine with marginal performance impact.
	 */
	LINEMODE__DISPLAY = 0,
	/* Recommended setting for operation in the Blit Engine. */
	LINEMODE__BLIT = 1 << LINEMODE_SHIFT,
};

#define BITSPERPIXEL(bpp)		(((bpp) & 0x3F) << 16)
#define STRIDE(n)			(((n) - 1) & 0xFFFF)
#define LINEWIDTH(w)			(((w) - 1) & 0x3FFF)
#define LINECOUNT(h)			((((h) - 1) & 0x3FFF) << 16)
#define ITUFORMAT			BIT(31)
#define R_BITS(n)			(((n) & 0xF) << 24)
#define G_BITS(n)			(((n) & 0xF) << 16)
#define B_BITS(n)			(((n) & 0xF) << 8)
#define A_BITS(n)			((n) & 0xF)
#define R_SHIFT(n)			(((n) & 0x1F) << 24)
#define G_SHIFT(n)			(((n) & 0x1F) << 16)
#define B_SHIFT(n)			(((n) & 0x1F) << 8)
#define A_SHIFT(n)			((n) & 0x1F)
#define LAYERXOFFSET(x)			((x) & 0x7FFF)
#define LAYERYOFFSET(y)			(((y) & 0x7FFF) << 16)
#define CLIPWINDOWXOFFSET(x)		((x) & 0x7FFF)
#define CLIPWINDOWYOFFSET(y)		(((y) & 0x7FFF) << 16)
#define CLIPWINDOWWIDTH(w)		(((w) - 1) & 0x3FFF)
#define CLIPWINDOWHEIGHT(h)		((((h) - 1) & 0x3FFF) << 16)
#define	PALETTEENABLE			BIT(0)
typedef enum {
	TILE_FILL_ZERO,
	TILE_FILL_CONSTANT,
	TILE_PAD,
	TILE_PAD_ZERO,
} tilemode_t;
#define ALPHASRCENABLE			BIT(8)
#define ALPHACONSTENABLE		BIT(9)
#define ALPHAMASKENABLE			BIT(10)
#define ALPHATRANSENABLE		BIT(11)
#define RGBALPHASRCENABLE		BIT(12)
#define RGBALPHACONSTENABLE		BIT(13)
#define RGBALPHAMASKENABLE		BIT(14)
#define RGBALPHATRANSENABLE		BIT(15)
#define PREMULCONSTRGB			BIT(16)
typedef enum {
	YUVCONVERSIONMODE__OFF,
	YUVCONVERSIONMODE__ITU601,
	YUVCONVERSIONMODE__ITU601_FR,
	YUVCONVERSIONMODE__ITU709,
} yuvconversionmode_t;
#define YUVCONVERSIONMODE_MASK		0x60000
#define YUVCONVERSIONMODE(m)		(((m) & 0x3) << 17)
#define GAMMAREMOVEENABLE		BIT(20)
#define CLIPWINDOWENABLE		BIT(30)
#define SOURCEBUFFERENABLE		BIT(31)
#define EMPTYFRAME			BIT(31)
#define FRAMEWIDTH(w)			(((w) - 1) & 0x3FFF)
#define FRAMEHEIGHT(h)			((((h) - 1) & 0x3FFF) << 16)
#define DELTAX_MASK			0x3F000
#define DELTAY_MASK			0xFC0000
#define DELTAX(x)			(((x) & 0x3F) << 12)
#define DELTAY(y)			(((y) & 0x3F) << 18)
#define YUV422UPSAMPLINGMODE_MASK	BIT(5)
#define YUV422UPSAMPLINGMODE(m)		(((m) & 0x1) << 5)
typedef enum {
	YUV422UPSAMPLINGMODE__REPLICATE,
	YUV422UPSAMPLINGMODE__INTERPOLATE,
} yuv422upsamplingmode_t;
#define SHDTOKGEN			BIT(0)
#define FETCHTYPE_MASK			0xF

enum {
	DPU_V1,
	DPU_V2,
};

#define DPU_VPROC_CAP_HSCALER4	BIT(0)
#define DPU_VPROC_CAP_VSCALER4	BIT(1)
#define DPU_VPROC_CAP_HSCALER5	BIT(2)
#define DPU_VPROC_CAP_VSCALER5	BIT(3)

#define DPU_VPROC_CAP_HSCALE	(DPU_VPROC_CAP_HSCALER4 | \
				 DPU_VPROC_CAP_HSCALER5)
#define DPU_VPROC_CAP_VSCALE	(DPU_VPROC_CAP_VSCALER4 | \
				 DPU_VPROC_CAP_VSCALER5)

struct dpu_unit {
	char *name;
	unsigned int num;
	const unsigned int *ids;
	const unsigned long *pec_ofss;	/* PixEngCFG */
	const unsigned long *ofss;
};

struct cm_reg_ofs {
	u32 ipidentifier;
	u32 lockunlock;
	u32 lockstatus;
	u32 userinterruptmask;
	u32 interruptenable;
	u32 interruptpreset;
	u32 interruptclear;
	u32 interruptstatus;
	u32 userinterruptenable;
	u32 userinterruptpreset;
	u32 userinterruptclear;
	u32 userinterruptstatus;
	u32 generalpurpose;
};

struct dpu_devtype {
	const unsigned long cm_ofs;	/* common */
	const struct dpu_unit *cfs;
	const struct dpu_unit *decs;
	const struct dpu_unit *eds;
	const struct dpu_unit *fds;
	const struct dpu_unit *fgs;
	const struct dpu_unit *fls;
	const struct dpu_unit *hss;
	const struct dpu_unit *lbs;
	const struct dpu_unit *tcons;
	const struct dpu_unit *vss;
	const struct cm_reg_ofs *cm_reg_ofs;
	const unsigned int *intsteer_map;
	const unsigned int intsteer_map_size;
	const unsigned long *unused_irq;
	const unsigned int *sw2hw_irq_map;	/* NULL means linear */
	const unsigned int *sw2hw_block_id_map;	/* NULL means linear */
	bool has_capture;
	bool pixel_link_quirks;
	bool pixel_link_nhvsync;	/* HSYNC and VSYNC high active */
	unsigned int version;
};

struct dpu_soc {
	struct device		*dev;
	const struct dpu_devtype	*devtype;
	spinlock_t		lock;

	void __iomem		*cm_reg;

	int			id;
	int			usecount;

	struct regmap		*intsteer_regmap;
	int			intsteer_usecount;
	spinlock_t		intsteer_lock;
	int			irq_cm;		/* irq common */
	int			irq_stream0a;
	int			irq_stream1a;
	int			irq_reserved0;
	int			irq_reserved1;
	int			irq_blit;
	struct irq_domain	*domain;

	struct dpu_constframe	*cf_priv[4];
	struct dpu_disengcfg	*dec_priv[2];
	struct dpu_extdst	*ed_priv[4];
	struct dpu_fetchdecode	*fd_priv[4];
	struct dpu_framegen	*fg_priv[2];
	struct dpu_fetchlayer	*fl_priv[2];
	struct dpu_hscaler	*hs_priv[3];
	struct dpu_layerblend	*lb_priv[7];
	struct dpu_tcon		*tcon_priv[2];
	struct dpu_vscaler	*vs_priv[3];
};

#define DECLARE_DPU_UNIT_INIT_FUNC(block)			\
int dpu_##block##_init(struct dpu_soc *dpu, unsigned int id,	\
			 unsigned long pec_base, unsigned long base)

DECLARE_DPU_UNIT_INIT_FUNC(cf);
DECLARE_DPU_UNIT_INIT_FUNC(dec);
DECLARE_DPU_UNIT_INIT_FUNC(ed);
DECLARE_DPU_UNIT_INIT_FUNC(fd);
DECLARE_DPU_UNIT_INIT_FUNC(fg);
DECLARE_DPU_UNIT_INIT_FUNC(fl);
DECLARE_DPU_UNIT_INIT_FUNC(hs);
DECLARE_DPU_UNIT_INIT_FUNC(lb);
DECLARE_DPU_UNIT_INIT_FUNC(tcon);
DECLARE_DPU_UNIT_INIT_FUNC(vs);

static const unsigned int cf_ids[] = {0, 1, 4, 5};
static const unsigned int dec_ids[] = {0, 1};
static const unsigned int ed_ids[] = {0, 1, 4, 5};
static const unsigned int fd_ids[] = {0, 1, 2, 3};
static const unsigned int fg_ids[] = {0, 1};
static const unsigned int fl_ids[] = {0, 1};
static const unsigned int hs_ids[] = {4, 5, 9};
static const unsigned int lb_ids[] = {0, 1, 2, 3, 4, 5, 6};
static const unsigned int tcon_ids[] = {0, 1};
static const unsigned int vs_ids[] = {4, 5, 9};

struct dpu_pixel_format {
	u32 pixel_format;
	u32 bits;
	u32 shift;
};

static const struct dpu_pixel_format dpu_pixel_format_matrix[] = {
	{
		DRM_FORMAT_XRGB8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(16) | G_SHIFT(8)  | B_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_XBGR8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(0)  | G_SHIFT(8)  | B_SHIFT(16) | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGBX8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(24) | G_SHIFT(16) | B_SHIFT(8)  | A_SHIFT(0),
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
	},
};

#endif				/* __DPU_PRV_H__ */
