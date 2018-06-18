/*
 * Copyright (C) 2017 NXP
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

#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <drm/drm_fourcc.h>

#include <video/imx-dcss.h>
#include "dcss-prv.h"

#define USE_CTXLD

#define DCSS_DTG_TC_CONTROL_STATUS			0x00
#define   CH3_EN					BIT(0)
#define   CH2_EN					BIT(1)
#define   CH1_EN					BIT(2)
#define   OVL_DATA_MODE					BIT(3)
#define   BLENDER_VIDEO_ALPHA_SEL			BIT(7)
#define   DTG_START					BIT(8)
#define   DBY_MODE_EN					BIT(9)
#define   CH1_ALPHA_SEL					BIT(10)
#define   CSS_PIX_COMP_SWAP_POS				12
#define   CSS_PIX_COMP_SWAP_MASK			GENMASK(14, 12)
#define   DEFAULT_FG_ALPHA_POS				24
#define   DEFAULT_FG_ALPHA_MASK				GENMASK(31, 24)
#define DCSS_DTG_TC_DTG					0x04
#define DCSS_DTG_TC_DISP_TOP				0x08
#define DCSS_DTG_TC_DISP_BOT				0x0C
#define DCSS_DTG_TC_CH1_TOP				0x10
#define DCSS_DTG_TC_CH1_BOT				0x14
#define DCSS_DTG_TC_CH2_TOP				0x18
#define DCSS_DTG_TC_CH2_BOT				0x1C
#define DCSS_DTG_TC_CH3_TOP				0x20
#define DCSS_DTG_TC_CH3_BOT				0x24
#define   TC_X_POS					0
#define   TC_X_MASK					GENMASK(12, 0)
#define   TC_Y_POS					16
#define   TC_Y_MASK					GENMASK(28, 16)
#define DCSS_DTG_TC_CTXLD				0x28
#define   TC_CTXLD_DB_Y_POS				0
#define   TC_CTXLD_DB_Y_MASK				GENMASK(12, 0)
#define   TC_CTXLD_SB_Y_POS				16
#define   TC_CTXLD_SB_Y_MASK				GENMASK(28, 16)
#define DCSS_DTG_TC_CH1_BKRND				0x2C
#define DCSS_DTG_TC_CH2_BKRND				0x30
#define   BKRND_R_Y_COMP_POS				20
#define   BKRND_R_Y_COMP_MASK				GENMASK(29, 20)
#define   BKRND_G_U_COMP_POS				10
#define   BKRND_G_U_COMP_MASK				GENMASK(19, 10)
#define   BKRND_B_V_COMP_POS				0
#define   BKRND_B_V_COMP_MASK				GENMASK(9, 0)
#define DCSS_DTG_BLENDER_DBY_RANGEINV			0x38
#define DCSS_DTG_BLENDER_DBY_RANGEMIN			0x3C
#define DCSS_DTG_BLENDER_DBY_BDP			0x40
#define DCSS_DTG_BLENDER_BKRND_I			0x44
#define DCSS_DTG_BLENDER_BKRND_P			0x48
#define DCSS_DTG_BLENDER_BKRND_T			0x4C
#define DCSS_DTG_LINE0_INT				0x50
#define DCSS_DTG_LINE1_INT				0x54
#define DCSS_DTG_BG_ALPHA_DEFAULT			0x58
#define DCSS_DTG_INT_STATUS				0x5C
#define DCSS_DTG_INT_CONTROL				0x60
#define DCSS_DTG_TC_CH3_BKRND				0x64
#define DCSS_DTG_INT_MASK				0x68
#define   LINE0_IRQ					BIT(0)
#define   LINE1_IRQ					BIT(1)
#define   LINE2_IRQ					BIT(2)
#define   LINE3_IRQ					BIT(3)
#define DCSS_DTG_LINE2_INT				0x6C
#define DCSS_DTG_LINE3_INT				0x70
#define DCSS_DTG_DBY_OL					0x74
#define DCSS_DTG_DBY_BL					0x78
#define DCSS_DTG_DBY_EL					0x7C

static struct dcss_debug_reg dtg_debug_reg[] = {
	DCSS_DBG_REG(DCSS_DTG_TC_CONTROL_STATUS),
	DCSS_DBG_REG(DCSS_DTG_TC_DTG),
	DCSS_DBG_REG(DCSS_DTG_TC_DISP_TOP),
	DCSS_DBG_REG(DCSS_DTG_TC_DISP_BOT),
	DCSS_DBG_REG(DCSS_DTG_TC_CH1_TOP),
	DCSS_DBG_REG(DCSS_DTG_TC_CH1_BOT),
	DCSS_DBG_REG(DCSS_DTG_TC_CH2_TOP),
	DCSS_DBG_REG(DCSS_DTG_TC_CH2_BOT),
	DCSS_DBG_REG(DCSS_DTG_TC_CH3_TOP),
	DCSS_DBG_REG(DCSS_DTG_TC_CH3_BOT),
	DCSS_DBG_REG(DCSS_DTG_TC_CTXLD),
	DCSS_DBG_REG(DCSS_DTG_TC_CH1_BKRND),
	DCSS_DBG_REG(DCSS_DTG_TC_CH2_BKRND),
	DCSS_DBG_REG(DCSS_DTG_BLENDER_DBY_RANGEINV),
	DCSS_DBG_REG(DCSS_DTG_BLENDER_DBY_RANGEMIN),
	DCSS_DBG_REG(DCSS_DTG_BLENDER_DBY_BDP),
	DCSS_DBG_REG(DCSS_DTG_BLENDER_BKRND_I),
	DCSS_DBG_REG(DCSS_DTG_BLENDER_BKRND_P),
	DCSS_DBG_REG(DCSS_DTG_BLENDER_BKRND_T),
	DCSS_DBG_REG(DCSS_DTG_LINE0_INT),
	DCSS_DBG_REG(DCSS_DTG_LINE1_INT),
	DCSS_DBG_REG(DCSS_DTG_BG_ALPHA_DEFAULT),
	DCSS_DBG_REG(DCSS_DTG_INT_STATUS),
	DCSS_DBG_REG(DCSS_DTG_INT_CONTROL),
	DCSS_DBG_REG(DCSS_DTG_TC_CH3_BKRND),
	DCSS_DBG_REG(DCSS_DTG_INT_MASK),
	DCSS_DBG_REG(DCSS_DTG_LINE2_INT),
	DCSS_DBG_REG(DCSS_DTG_LINE3_INT),
	DCSS_DBG_REG(DCSS_DTG_DBY_OL),
	DCSS_DBG_REG(DCSS_DTG_DBY_BL),
	DCSS_DBG_REG(DCSS_DTG_DBY_EL),
};

struct dcss_dtg_priv {
	struct dcss_soc *dcss;
	void __iomem *base_reg;
	u32 base_ofs;

	u32 ctx_id;

	bool in_use;

	u32 dis_ulc_x;
	u32 dis_ulc_y;

	u32 control_status;
	u32 alpha;
	u32 use_global;

	int ctxld_kick_irq;

	/*
	 * This will be passed on by DRM CRTC so that we can signal when DTG has
	 * been successfully stopped. Otherwise, any modesetting while DTG is
	 * still on may result in unpredictable behavior.
	 */
	struct completion *dis_completion;
};

static void dcss_dtg_write(struct dcss_dtg_priv *dtg, u32 val, u32 ofs)
{
	if (!dtg->in_use)
		dcss_writel(val, dtg->base_reg + ofs);
#if defined(USE_CTXLD)
	dcss_ctxld_write(dtg->dcss, dtg->ctx_id, val, dtg->base_ofs + ofs);
#endif
}

#ifdef CONFIG_DEBUG_FS
void dcss_dtg_dump_regs(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	int j;

	seq_puts(s, ">> Dumping DTG:\n");
	for (j = 0; j < ARRAY_SIZE(dtg_debug_reg); j++)
		seq_printf(s, "%-35s(0x%04x) -> 0x%08x\n",
			   dtg_debug_reg[j].name,
			   dtg_debug_reg[j].ofs,
			   dcss_readl(dcss->dtg_priv->base_reg +
				      dtg_debug_reg[j].ofs));
}
#endif

static irqreturn_t dcss_dtg_irq_handler(int irq, void *data)
{
	struct dcss_dtg_priv *dtg = data;
	u32 status;

	status = dcss_readl(dtg->base_reg + DCSS_DTG_INT_STATUS);

	dcss_ctxld_kick(dtg->dcss);

	dcss_writel(status & LINE1_IRQ, dtg->base_reg + DCSS_DTG_INT_CONTROL);

	return IRQ_HANDLED;
}

static int dcss_dtg_irq_config(struct dcss_dtg_priv *dtg)
{
	struct dcss_soc *dcss = dtg->dcss;
	struct platform_device *pdev = to_platform_device(dcss->dev);
	int ret;

	dtg->ctxld_kick_irq = platform_get_irq_byname(pdev, "ctxld_kick");
	if (dtg->ctxld_kick_irq < 0) {
		dev_err(dcss->dev, "dtg: can't get line2 irq number\n");
		return dtg->ctxld_kick_irq;
	}

	ret = devm_request_irq(dcss->dev, dtg->ctxld_kick_irq,
			       dcss_dtg_irq_handler,
			       IRQF_TRIGGER_HIGH,
			       "dcss_ctxld_kick", dtg);
	if (ret) {
		dev_err(dcss->dev, "dtg: irq request failed.\n");
		return ret;
	}

	return 0;
}

int dcss_dtg_init(struct dcss_soc *dcss, unsigned long dtg_base)
{
	struct dcss_dtg_priv *dtg;

	dtg = devm_kzalloc(dcss->dev, sizeof(*dtg), GFP_KERNEL);
	if (!dtg)
		return -ENOMEM;

	dcss->dtg_priv = dtg;
	dtg->dcss = dcss;

	dtg->base_reg = devm_ioremap(dcss->dev, dtg_base, SZ_4K);
	if (!dtg->base_reg) {
		dev_err(dcss->dev, "dtg: unable to remap dtg base\n");
		return -ENOMEM;
	}

	dtg->base_ofs = dtg_base;

#if defined(USE_CTXLD)
	dtg->ctx_id = CTX_DB;
#endif

	dtg->alpha = 255;
	dtg->use_global = 0;

	dtg->control_status |= OVL_DATA_MODE | BLENDER_VIDEO_ALPHA_SEL |
		((dtg->alpha << DEFAULT_FG_ALPHA_POS) & DEFAULT_FG_ALPHA_MASK);

	return dcss_dtg_irq_config(dtg);
}

void dcss_dtg_exit(struct dcss_soc *dcss)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;

	/* stop DTG */
	dcss_writel(DTG_START, dtg->base_reg + DCSS_DTG_TC_CONTROL_STATUS);
}

void dcss_dtg_sync_set(struct dcss_soc *dcss, struct videomode *vm)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;
	u16 dtg_lrc_x, dtg_lrc_y;
	u16 dis_ulc_x, dis_ulc_y;
	u16 dis_lrc_x, dis_lrc_y;
	u32 sb_ctxld_trig, db_ctxld_trig;

	dev_dbg(dcss->dev, "hfront_porch = %d\n", vm->hfront_porch);
	dev_dbg(dcss->dev, "hback_porch = %d\n", vm->hback_porch);
	dev_dbg(dcss->dev, "hsync_len = %d\n", vm->hsync_len);
	dev_dbg(dcss->dev, "hactive = %d\n", vm->hactive);
	dev_dbg(dcss->dev, "vfront_porch = %d\n", vm->vfront_porch);
	dev_dbg(dcss->dev, "vback_porch = %d\n", vm->vback_porch);
	dev_dbg(dcss->dev, "vsync_len = %d\n", vm->vsync_len);
	dev_dbg(dcss->dev, "vactive = %d\n", vm->vactive);

	dtg_lrc_x = vm->hfront_porch + vm->hback_porch + vm->hsync_len +
		    vm->hactive - 1;
	dtg_lrc_y = vm->vfront_porch + vm->vback_porch + vm->vsync_len +
		    vm->vactive - 1;
	dis_ulc_x = vm->hsync_len + vm->hback_porch - 1;
	dis_ulc_y = vm->vsync_len + vm->vfront_porch + vm->vback_porch - 1;
	dis_lrc_x = vm->hsync_len + vm->hback_porch + vm->hactive - 1;
	dis_lrc_y = vm->vsync_len + vm->vfront_porch + vm->vback_porch +
		    vm->vactive - 1;

	clk_disable_unprepare(dcss->pout_clk);
	clk_disable_unprepare(dcss->pdiv_clk);
	clk_set_rate(dcss->pdiv_clk, vm->pixelclock);
	clk_prepare_enable(dcss->pdiv_clk);
	clk_prepare_enable(dcss->pout_clk);

	msleep(500);

	dcss_dtg_write(dtg, ((dtg_lrc_y << TC_Y_POS) | dtg_lrc_x),
		       DCSS_DTG_TC_DTG);
	dcss_dtg_write(dtg, ((dis_ulc_y << TC_Y_POS) | dis_ulc_x),
		       DCSS_DTG_TC_DISP_TOP);
	dcss_dtg_write(dtg, ((dis_lrc_y << TC_Y_POS) | dis_lrc_x),
		       DCSS_DTG_TC_DISP_BOT);

	dtg->dis_ulc_x = dis_ulc_x;
	dtg->dis_ulc_y = dis_ulc_y;

	sb_ctxld_trig = ((0 * dis_lrc_y / 100) << TC_CTXLD_SB_Y_POS) &
							TC_CTXLD_SB_Y_MASK;
	db_ctxld_trig = ((99 * dis_lrc_y / 100) << TC_CTXLD_DB_Y_POS) &
							TC_CTXLD_DB_Y_MASK;

	dcss_dtg_write(dtg, sb_ctxld_trig | db_ctxld_trig, DCSS_DTG_TC_CTXLD);

	/* vblank trigger */
	dcss_dtg_write(dtg, 0, DCSS_DTG_LINE0_INT);

	/* CTXLD trigger */
	dcss_dtg_write(dtg, ((98 * dis_lrc_y) / 100) << 16, DCSS_DTG_LINE1_INT);
}
EXPORT_SYMBOL(dcss_dtg_sync_set);

void dcss_dtg_plane_pos_set(struct dcss_soc *dcss, int ch_num,
			    int px, int py, int pw, int ph)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;
	u16 p_ulc_x, p_ulc_y;
	u16 p_lrc_x, p_lrc_y;

	p_ulc_x = dtg->dis_ulc_x + px;
	p_ulc_y = dtg->dis_ulc_y + py;
	p_lrc_x = p_ulc_x + pw;
	p_lrc_y = p_ulc_y + ph;

	if (!px && !py && !pw && !ph) {
		dcss_dtg_write(dtg, 0, DCSS_DTG_TC_CH1_TOP + 0x8 * ch_num);
		dcss_dtg_write(dtg, 0, DCSS_DTG_TC_CH1_BOT + 0x8 * ch_num);
	} else {
		dcss_dtg_write(dtg, ((p_ulc_y << TC_Y_POS) | p_ulc_x),
			       DCSS_DTG_TC_CH1_TOP + 0x8 * ch_num);
		dcss_dtg_write(dtg, ((p_lrc_y << TC_Y_POS) | p_lrc_x),
			       DCSS_DTG_TC_CH1_BOT + 0x8 * ch_num);
	}
}
EXPORT_SYMBOL(dcss_dtg_plane_pos_set);

static bool dcss_dtg_global_alpha_needed(u32 pix_format)
{
	return pix_format == DRM_FORMAT_XRGB8888    ||
	       pix_format == DRM_FORMAT_XBGR8888    ||
	       pix_format == DRM_FORMAT_RGBX8888    ||
	       pix_format == DRM_FORMAT_BGRX8888    ||
	       pix_format == DRM_FORMAT_XRGB2101010 ||
	       pix_format == DRM_FORMAT_XBGR2101010 ||
	       pix_format == DRM_FORMAT_RGBX1010102 ||
	       pix_format == DRM_FORMAT_BGRX1010102 ||
	       pix_format == DRM_FORMAT_UYVY	    ||
	       pix_format == DRM_FORMAT_VYUY	    ||
	       pix_format == DRM_FORMAT_YUYV	    ||
	       pix_format == DRM_FORMAT_YVYU	    ||
	       pix_format == DRM_FORMAT_NV12	    ||
	       pix_format == DRM_FORMAT_NV21	    ||
	       pix_format == DRM_FORMAT_P010;
}

bool dcss_dtg_global_alpha_changed(struct dcss_soc *dcss, int ch_num,
				   u32 pix_format, int alpha,
				   int use_global_alpha)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;

	if (ch_num)
		return false;

	return alpha != dtg->alpha || use_global_alpha != dtg->use_global;
}
EXPORT_SYMBOL(dcss_dtg_global_alpha_changed);

void dcss_dtg_plane_alpha_set(struct dcss_soc *dcss, int ch_num,
			      u32 pix_format, int alpha, bool use_global_alpha)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;
	u32 alpha_val;

	/* we care about alpha only when channel 0 is concerned */
	if (ch_num)
		return;

	alpha_val = (alpha << DEFAULT_FG_ALPHA_POS) & DEFAULT_FG_ALPHA_MASK;

	/*
	 * Use global alpha if pixel format does not have alpha channel or the
	 * user explicitly chose to use global alpha.
	 */
	if (dcss_dtg_global_alpha_needed(pix_format) || use_global_alpha) {
		dtg->control_status &= ~(CH1_ALPHA_SEL | DEFAULT_FG_ALPHA_MASK);
		dtg->control_status |= alpha_val;
	} else {
		dtg->control_status |= CH1_ALPHA_SEL;
	}

	dtg->alpha = alpha;
	dtg->use_global = use_global_alpha;
}
EXPORT_SYMBOL(dcss_dtg_plane_alpha_set);

void dcss_dtg_css_set(struct dcss_soc *dcss, u32 pix_format)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;

	if (pix_format == DRM_FORMAT_P010) {
		dtg->control_status &= ~CSS_PIX_COMP_SWAP_MASK;
		return;
	}

	dtg->control_status |=
			(0x5 << CSS_PIX_COMP_SWAP_POS) & CSS_PIX_COMP_SWAP_MASK;
}
EXPORT_SYMBOL(dcss_dtg_css_set);

static void dcss_dtg_disable_callback(void *data)
{
	struct dcss_dtg_priv *dtg = data;

	dtg->control_status &= ~DTG_START;

	dcss_writel(dtg->control_status,
		    dtg->base_reg + DCSS_DTG_TC_CONTROL_STATUS);

	dtg->in_use = false;

	complete(dtg->dis_completion);
}

void dcss_dtg_enable(struct dcss_soc *dcss, bool en,
		     struct completion *dis_completion)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;

	if (!en) {
		dcss->dcss_disable_callback = dcss_dtg_disable_callback;
		dtg->dis_completion = dis_completion;
		return;
	}

	dcss->dcss_disable_callback = NULL;
	dtg->dis_completion = NULL;

	dtg->control_status |= DTG_START;

	dcss_dtg_write(dtg, dtg->control_status, DCSS_DTG_TC_CONTROL_STATUS);

	dtg->in_use = true;
}
EXPORT_SYMBOL(dcss_dtg_enable);

bool dcss_dtg_is_enabled(struct dcss_soc *dcss)
{
	return dcss->dtg_priv->in_use;
}
EXPORT_SYMBOL(dcss_dtg_is_enabled);

void dcss_dtg_ch_enable(struct dcss_soc *dcss, int ch_num, bool en)
{
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;
	u32 ch_en_map[] = {CH1_EN, CH2_EN, CH3_EN};
	u32 control_status;

	control_status = dtg->control_status & ~ch_en_map[ch_num];
	control_status |= en ? ch_en_map[ch_num] : 0;

	if (dtg->control_status != control_status)
		dcss_dtg_write(dtg, control_status, DCSS_DTG_TC_CONTROL_STATUS);

	dtg->control_status = control_status;
}
EXPORT_SYMBOL(dcss_dtg_ch_enable);

void dcss_dtg_vblank_irq_enable(struct dcss_soc *dcss, bool en)
{
	void __iomem *reg;
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;
	u32 val = en ? (LINE0_IRQ | LINE1_IRQ) : 0;

	/* need to keep the CTXLD kick interrupt ON if DTRC is used */
	if (!en && (dcss_dtrc_is_running(dcss, 1) ||
		    dcss_dtrc_is_running(dcss, 2)))
		val |= LINE1_IRQ;

	reg = dtg->base_reg + DCSS_DTG_INT_MASK;

	dcss_update(val, LINE0_IRQ | LINE1_IRQ, reg);

	dcss_dpr_irq_enable(dcss, en);
}

void dcss_dtg_vblank_irq_clear(struct dcss_soc *dcss)
{
	void __iomem *reg;
	struct dcss_dtg_priv *dtg = dcss->dtg_priv;

	reg = dtg->base_reg + DCSS_DTG_INT_CONTROL;

	dcss_update(LINE0_IRQ, LINE0_IRQ, reg);
}
