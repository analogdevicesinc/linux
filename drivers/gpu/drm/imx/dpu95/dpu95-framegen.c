// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020,2023 NXP
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/sizes.h>

#include "dpu95.h"

#define FGSTCTRL			0x8
#define  FGSYNCMODE_MASK		0xe
#define  FGSYNCMODE(n)			((n) << 1)
#define  FGSHDTOKGENSYNCMODE_MASK	BIT(4)
#define  FGSHDTOKGENSYNCMODE(n)		((n) << 4)

#define HTCFG1				0xc
#define  HTOTAL(n)			((((n) - 1) & 0x3fff) << 16)
#define  HACT(n)			((n) & 0x3fff)

#define HTCFG2				0x10
#define  HSEN				BIT(31)
#define  HSBP(n)			((((n) - 1) & 0x3fff) << 16)
#define  HSYNC(n)			(((n) - 1) & 0x3fff)

#define VTCFG1				0x14
#define  VTOTAL(n)			((((n) - 1) & 0x3fff) << 16)
#define  VACT(n)			((n) & 0x3fff)

#define VTCFG2				0x18
#define  VSEN				BIT(31)
#define  VSBP(n)			((((n) - 1) & 0x3fff) << 16)
#define  VSYNC(n)			(((n) - 1) & 0x3fff)

#define INTCONFIG(n)			(0x1c + 4 * (n))
#define  EN				BIT(31)
#define  ROW(n)				(((n) & 0x3fff) << 16)
#define  COL(n)				((n) & 0x3fff)

#define PKICKCONFIG			0x2c

#define SKICKCONFIG			0x30
#define  SKICKTRIG			BIT(30)
#define  EXTERNAL			BIT(30)

#define SECSTATCONFIG			0x34
#define FGSRCR(n)			(0x38 + ((n) - 1) * 0x4)
#define FGKSDR				0x74

#define PACFG				0x78
#define  STARTX(n)			(((n) + 1) & 0x3fff)
#define  STARTY(n)			(((((n) + 1) & 0x3fff)) << 16)

#define SACFG				0x7c

#define FGINCTRL			0x80
#define FGINCTRLPANIC			0x84
#define  FGDM_MASK			0x7
#define  ENPRIMALPHA			BIT(3)
#define  ENSECALPHA			BIT(4)

#define FGCCR				0x88
#define  CCALPHA(a)			(((a) & 0x3) << 30)
#define  CCRED(r)			(((r) & 0x3ff) << 20)
#define  CCGREEN(g)			(((g) & 0x3ff) << 10)
#define  CCBLUE(b)			((b) & 0x3ff)

#define FGENABLE			0x8c
#define  FGEN				BIT(0)

#define FGSLR				0x90
#define  SHDTOKGEN			BIT(0)

#define FGENSTS				0x94
#define  ENSTS				BIT(0)

#define FGTIMESTAMP			0x98
#define  FRAMEINDEX_SHIFT		14
#define  FRAMEINDEX_MASK		(DPU95_FRAMEGEN_MAX_FRAME_INDEX << \
					 FRAMEINDEX_SHIFT)
#define  LINEINDEX_MASK			0x3fff

#define FGCHSTAT			0x9c
#define  SECSYNCSTAT			BIT(24)
#define  SFIFOEMPTY			BIT(16)
#define  PRIMSYNCSTAT			BIT(8)
#define  PFIFOEMPTY			BIT(0)

#define FGCHSTATCLR			0xa0
#define  CLRSECSTAT			BIT(16)
#define  CLRPRIMSTAT			BIT(0)

#define FGSKEWMON			0xa4
#define FGPFIFOMIN			0xa8
#define FGPFIFOMAX			0xac
#define FGPFIFOFILLCLR			0xb0
#define FGPFIFOTRES			0xb4
#define FGSFIFOMIN			0xb8
#define FGSFIFOMAX			0xbc
#define FGSFIFOFILLCLR			0xc0
#define FGSREPD				0xc4
#define FGSRFTD				0xc8
#define FGSRCSHTOTAL			0xcc
#define FGSRCLOCKDIV			0xd0
#define FGSRl				0xd4

/* registers in blk-ctrl */
#define CLOCK_CTRL			0x0
#define  DSIP_CLK_SEL(n)		(0x3 << (2 * (n)))
#define  CCM				0x0
#define  DSI_PLL(n)			(0x1 << (2 * (n)))
#define  LVDS_PLL_7(n)			(0x2 << (2 * (n)))

struct dpu95_framegen {
	void __iomem *base;
	int id;
	unsigned int index;
	struct dpu95_soc *dpu;
};

static inline u32 dpu95_fg_read(struct dpu95_framegen *fg, unsigned int offset)
{
	return readl(fg->base + offset);
}

static inline void dpu95_fg_write(struct dpu95_framegen *fg,
				  unsigned int offset, u32 value)
{
	writel(value, fg->base + offset);
}

static inline void dpu95_fg_write_mask(struct dpu95_framegen *fg,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_fg_read(fg, offset);
	tmp &= ~mask;
	dpu95_fg_write(fg, offset, tmp | value);
}

static void dpu95_fg_enable_shden(struct dpu95_framegen *fg)
{
	dpu95_fg_write_mask(fg, FGSTCTRL, SHDEN, SHDEN);
}

void dpu95_fg_syncmode(struct dpu95_framegen *fg, enum dpu95_fg_syncmode mode)
{
	dpu95_fg_write_mask(fg, FGSTCTRL, FGSYNCMODE_MASK, FGSYNCMODE(mode));
}

void dpu95_fg_shdtokgen_syncmode(struct dpu95_framegen *fg,
				 enum dpu95_fg_shdtokgensyncmode mode)
{
	dpu95_fg_write_mask(fg, FGSTCTRL, FGSHDTOKGENSYNCMODE_MASK,
			    FGSHDTOKGENSYNCMODE(mode));
}

void dpu95_fg_cfg_videomode(struct dpu95_framegen *fg,
			    struct drm_display_mode *m,
			    bool enc_is_dsi)
{
	struct dpu95_soc *dpu = fg->dpu;
	u32 hact, htotal, hsync, hsbp;
	u32 vact, vtotal, vsync, vsbp;
	u16 vsync_start, vsync_end;
	u32 kick_row, kick_col;
	int ret;

	hact = m->crtc_hdisplay;
	htotal = m->crtc_htotal;
	hsync = m->crtc_hsync_end - m->crtc_hsync_start;
	hsbp = m->crtc_htotal - m->crtc_hsync_start;

	if (enc_is_dsi) {
		/* add one pixel to vfp and reduce one from vbp */
		vsync_start = m->crtc_vsync_start - 1;
		vsync_end = m->crtc_vsync_end - 1;
	} else {
		vsync_start = m->crtc_vsync_start;
		vsync_end = m->crtc_vsync_end;
	}

	vact = m->crtc_vdisplay;
	vtotal = m->crtc_vtotal;
	vsync = vsync_end - vsync_start;
	vsbp = m->crtc_vtotal - vsync_start;

	/* video mode */
	dpu95_fg_write(fg, HTCFG1, HACT(hact)   | HTOTAL(htotal));
	dpu95_fg_write(fg, HTCFG2, HSYNC(hsync) | HSBP(hsbp) | HSEN);
	dpu95_fg_write(fg, VTCFG1, VACT(vact)   | VTOTAL(vtotal));
	dpu95_fg_write(fg, VTCFG2, VSYNC(vsync) | VSBP(vsbp) | VSEN);

	kick_col = hact + 1;
	kick_row = vact;

	/* disable panic mechanism */
	dpu95_fg_write(fg, FGPFIFOTRES, 0);

	/* pkickconfig */
	dpu95_fg_write(fg, PKICKCONFIG, COL(kick_col) | ROW(kick_row) | EN);

	/* primary area position configuration */
	dpu95_fg_write(fg, PACFG, STARTX(0) | STARTY(0));

	/* alpha */
	dpu95_fg_write_mask(fg, FGINCTRL,      ENPRIMALPHA | ENSECALPHA, 0);
	dpu95_fg_write_mask(fg, FGINCTRLPANIC, ENPRIMALPHA | ENSECALPHA, 0);

	/* constant color is green(used in panic mode)  */
	dpu95_fg_write(fg, FGCCR, CCGREEN(0x3ff));

	if (enc_is_dsi)
		clk_set_rate(dpu->clk_pix, m->crtc_clock * 1000);

	ret = regmap_update_bits(dpu->regmap, CLOCK_CTRL, DSIP_CLK_SEL(fg->id),
				 enc_is_dsi ? CCM : LVDS_PLL_7(fg->id));
	if (ret < 0)
		dev_err(dpu->dev, "FrameGen%d failed to set DSIP_CLK_SEL: %d\n",
			fg->id, ret);
}

void dpu95_fg_displaymode(struct dpu95_framegen *fg, enum dpu95_fg_dm mode)
{
	dpu95_fg_write_mask(fg, FGINCTRL, FGDM_MASK, mode);
}

void dpu95_fg_panic_displaymode(struct dpu95_framegen *fg, enum dpu95_fg_dm mode)
{
	dpu95_fg_write_mask(fg, FGINCTRLPANIC, FGDM_MASK, mode);
}

void dpu95_fg_enable(struct dpu95_framegen *fg)
{
	dpu95_fg_write(fg, FGENABLE, FGEN);
}

void dpu95_fg_disable(struct dpu95_framegen *fg)
{
	dpu95_fg_write(fg, FGENABLE, 0);
}

void dpu95_fg_shdtokgen(struct dpu95_framegen *fg)
{
	dpu95_fg_write(fg, FGSLR, SHDTOKGEN);
}

u32 dpu95_fg_get_frame_index(struct dpu95_framegen *fg)
{
	u32 val = dpu95_fg_read(fg, FGTIMESTAMP);

	return (val & FRAMEINDEX_MASK) >> FRAMEINDEX_SHIFT;
}

int dpu95_fg_get_line_index(struct dpu95_framegen *fg)
{
	u32 val = dpu95_fg_read(fg, FGTIMESTAMP);

	return val & LINEINDEX_MASK;
}

bool dpu95_fg_primary_requests_to_read_empty_fifo(struct dpu95_framegen *fg)
{
	u32 val;

	val = dpu95_fg_read(fg, FGCHSTAT);

	return !!(val & PFIFOEMPTY);
}

void dpu95_fg_primary_clear_channel_status(struct dpu95_framegen *fg)
{
	dpu95_fg_write(fg, FGCHSTATCLR, CLRPRIMSTAT);
}

bool dpu95_fg_secondary_requests_to_read_empty_fifo(struct dpu95_framegen *fg)
{
	u32 val;

	val = dpu95_fg_read(fg, FGCHSTAT);

	return !!(val & SFIFOEMPTY);
}

void dpu95_fg_secondary_clear_channel_status(struct dpu95_framegen *fg)
{
	dpu95_fg_write(fg, FGCHSTATCLR, CLRSECSTAT);
}

int dpu95_fg_wait_for_primary_syncup(struct dpu95_framegen *fg)
{
	struct device *dev = fg->dpu->dev;
	u32 val;
	int ret;

	ret = readl_poll_timeout(fg->base + FGCHSTAT, val,
				 val & PRIMSYNCSTAT, 5, 100000);
	if (ret) {
		dev_dbg(dev, "failed to wait for FrameGen%u primary syncup\n",
			fg->id);
		return -ETIMEDOUT;
	}

	dev_dbg(dev, "FrameGen%u primary syncup\n", fg->id);

	return 0;
}

int dpu95_fg_wait_for_secondary_syncup(struct dpu95_framegen *fg)
{
	struct device *dev = fg->dpu->dev;
	u32 val;
	int ret;

	ret = readl_poll_timeout(fg->base + FGCHSTAT, val,
				 val & SECSYNCSTAT, 5, 100000);
	if (ret) {
		dev_dbg(dev, "failed to wait for FrameGen%u secondary syncup\n",
			fg->id);
		return -ETIMEDOUT;
	}

	dev_dbg(dev, "FrameGen%u secondary syncup\n", fg->id);

	return 0;
}

void dpu95_fg_enable_clock(struct dpu95_framegen *fg, bool enc_is_dsi)
{
	if (enc_is_dsi) {
		clk_prepare_enable(fg->dpu->clk_pix);
	} else {
		clk_prepare_enable(fg->dpu->clk_ldb_vco);
		clk_prepare_enable(fg->dpu->clk_ldb);
	}
}

void dpu95_fg_disable_clock(struct dpu95_framegen *fg, bool enc_is_dsi)
{
	if (enc_is_dsi) {
		clk_disable_unprepare(fg->dpu->clk_pix);
	} else {
		clk_disable_unprepare(fg->dpu->clk_ldb);
		clk_disable_unprepare(fg->dpu->clk_ldb_vco);
	}
}

struct dpu95_framegen *dpu95_fg_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_framegen *fg;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->fg); i++) {
		fg = dpu->fg[i];
		if (fg->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->fg))
		return ERR_PTR(-EINVAL);

	return fg;
}

void dpu95_fg_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_framegen *fg = dpu->fg[index];

	dpu95_fg_enable_shden(fg);
	dpu95_fg_syncmode(fg, FG_SYNCMODE_OFF);
}

int dpu95_fg_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long unused, unsigned long base)
{
	struct dpu95_framegen *fg;

	fg = devm_kzalloc(dpu->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	dpu->fg[index] = fg;

	fg->base = devm_ioremap(dpu->dev, base, SZ_256);
	if (!fg->base)
		return -ENOMEM;

	fg->dpu = dpu;
	fg->id = id;
	fg->index = index;

	return 0;
}
