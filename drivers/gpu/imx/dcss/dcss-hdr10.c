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

#include <video/imx-dcss.h>
#include "dcss-prv.h"
#include "dcss-tables.h"

#define USE_CTXLD

#define DCSS_HDR10_A0_LUT		0x0000
#define DCSS_HDR10_A1_LUT		0x1000
#define DCSS_HDR10_A2_LUT		0x2000
/* one CSCA and CSCB for each channel(pipe) */
#define DCSS_HDR10_CSCA_BASE		0x3000
#define DCSS_HDR10_CSCB_BASE		0x3800

/* one CSCO for all channels(pipes) */
#define DCSS_HDR10_CSCO_BASE		0x3000

#define DCSS_HDR10_LUT_CONTROL		(DCSS_HDR10_CSCA_BASE + 0x80)
#define   LUT_ENABLE			BIT(0)
#define   LUT_EN_FOR_ALL_PELS		BIT(1)
#define DCSS_HDR10_FL2FX		(DCSS_HDR10_CSCB_BASE + 0x74)
#define DCSS_HDR10_LTNL			(DCSS_HDR10_CSCO_BASE + 0x74)
#define   LTNL_PASS_THRU		BIT(0)
#define   FIX2FLT_DISABLE		BIT(1)
#define   LTNL_EN_FOR_ALL_PELS		BIT(2)
#define   FIX2FLT_EN_FOR_ALL_PELS	BIT(3)

/* following offsets are relative to CSC(A|B|O)_BASE */
#define DCSS_HDR10_CSC_CONTROL		0x00
#define   CSC_EN			BIT(0)
#define   CSC_ALL_PIX_EN		BIT(1)
#define   CSC_BYPASS			BIT(15)
#define DCSS_HDR10_CSC_H00		0x04
#define DCSS_HDR10_CSC_H10		0x08
#define DCSS_HDR10_CSC_H20		0x0C
#define DCSS_HDR10_CSC_H01		0x10
#define DCSS_HDR10_CSC_H11		0x14
#define DCSS_HDR10_CSC_H21		0x18
#define DCSS_HDR10_CSC_H02		0x1C
#define DCSS_HDR10_CSC_H12		0x20
#define DCSS_HDR10_CSC_H22		0x24
#define   H_COEF_MASK			GENMASK(15, 0)
#define DCSS_HDR10_CSC_IO0		0x28
#define DCSS_HDR10_CSC_IO1		0x2C
#define DCSS_HDR10_CSC_IO2		0x30
#define   PRE_OFFSET_MASK		GENMASK(9, 0)
#define DCSS_HDR10_CSC_IO_MIN0		0x34
#define DCSS_HDR10_CSC_IO_MIN1		0x38
#define DCSS_HDR10_CSC_IO_MIN2		0x3C
#define DCSS_HDR10_CSC_IO_MAX0		0x40
#define DCSS_HDR10_CSC_IO_MAX1		0x44
#define DCSS_HDR10_CSC_IO_MAX2		0x48
#define   IO_CLIP_MASK			GENMASK(9, 0)
#define DCSS_HDR10_CSC_NORM		0x4C
#define   NORM_MASK			GENMASK(4, 0)
#define DCSS_HDR10_CSC_OO0		0x50
#define DCSS_HDR10_CSC_OO1		0x54
#define DCSS_HDR10_CSC_OO2		0x58
#define   POST_OFFSET_MASK		GENMASK(27, 0)
#define DCSS_HDR10_CSC_OMIN0		0x5C
#define DCSS_HDR10_CSC_OMIN1		0x60
#define DCSS_HDR10_CSC_OMIN2		0x64
#define DCSS_HDR10_CSC_OMAX0		0x68
#define DCSS_HDR10_CSC_OMAX1		0x6C
#define DCSS_HDR10_CSC_OMAX2		0x70
#define   POST_CLIP_MASK		GENMASK(9, 0)

#define HDR10_LUT_MAX_ENTRIES		1024
#define HDR10_CSC_MAX_REGS		28

#define OPIPE_CH_NO			3

enum dcss_hdr10_csc {
	HDR10_CSCA,
	HDR10_CSCB,
};

struct dcss_hdr10_ch {
	void __iomem *base_reg;
	u32 base_ofs;

	u32 ctx_id;

	u32 old_in_cs;
	u32 old_out_cs;
};

struct dcss_hdr10_priv {
	struct dcss_soc *dcss;

	struct dcss_hdr10_ch ch[4]; /* 4th channel is, actually, OPIPE */

	u32 opipe_ctx_id;
};

static struct dcss_debug_reg hdr10_debug_reg[] = {
	DCSS_DBG_REG(DCSS_HDR10_CSC_CONTROL),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H00),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H10),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H20),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H01),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H11),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H21),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H02),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H12),
	DCSS_DBG_REG(DCSS_HDR10_CSC_H22),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO0),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO1),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO2),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO_MIN0),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO_MIN1),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO_MIN2),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO_MAX0),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO_MAX1),
	DCSS_DBG_REG(DCSS_HDR10_CSC_IO_MAX2),
	DCSS_DBG_REG(DCSS_HDR10_CSC_NORM),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OO0),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OO1),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OO2),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OMIN0),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OMIN1),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OMIN2),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OMAX0),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OMAX1),
	DCSS_DBG_REG(DCSS_HDR10_CSC_OMAX2),
};

static void dcss_hdr10_write(struct dcss_soc *dcss, u32 ch_num,
			     u32 val, u32 ofs)
{
	struct dcss_hdr10_priv *hdr10 = dcss->hdr10_priv;

#if !defined(USE_CTXLD)
	dcss_writel(val, hdr10->ch[ch_num].base_reg + ofs);
#else
	dcss_ctxld_write(dcss, hdr10->ch[ch_num].ctx_id, val,
			 hdr10->ch[ch_num].base_ofs + ofs);
#endif
}

#ifdef CONFIG_DEBUG_FS
void dcss_hdr10_dump_regs(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	int ch, csc, r;
	int csc_no;

	for (ch = 0; ch < 4; ch++) {
		void __iomem *csc_base = dcss->hdr10_priv->ch[ch].base_reg +
					 DCSS_HDR10_CSCA_BASE;

		if (ch < 3) {
			seq_printf(s, ">> Dumping HDR10 CH %d:\n", ch);
			csc_no = 2;
		} else {
			seq_puts(s, ">> Dumping HDR10 OPIPE:\n");
			csc_no = 1;
		}

		for (csc = 0; csc < csc_no; csc++) {
			csc_base += csc * 0x800;

			if (ch < 3)
				seq_printf(s, "\t>> Dumping CSC%s of CH %d:\n",
					   csc ? "B" : "A", ch);
			else
				seq_puts(s, "\t>> Dumping CSC of OPIPE:\n");

			for (r = 0; r < ARRAY_SIZE(hdr10_debug_reg); r++)
				seq_printf(s, "\t%-35s(0x%04x) -> 0x%08x\n",
					   hdr10_debug_reg[r].name,
					   hdr10_debug_reg[r].ofs,
					   dcss_readl(csc_base +
						      hdr10_debug_reg[r].ofs));
		}
	}
}
#endif

static void dcss_hdr10_csc_fill(struct dcss_soc *dcss, int ch_num,
				enum dcss_hdr10_csc csc_to_use,
				u32 *map)
{
	int i;
	u32 csc_base_ofs[] = {
		DCSS_HDR10_CSCA_BASE + DCSS_HDR10_CSC_H00,
		DCSS_HDR10_CSCB_BASE + DCSS_HDR10_CSC_H00,
	};

	for (i = 0; i < HDR10_CSC_MAX_REGS; i++) {
		u32 reg_ofs = csc_base_ofs[csc_to_use] + i * sizeof(u32);

		dcss_hdr10_write(dcss, ch_num, map[i], reg_ofs);
	}
}

static void dcss_hdr10_lut_fill(struct dcss_soc *dcss, int ch_num,
				int comp, u16 *map)
{
	int i;
	u32 lut_base_ofs;

	lut_base_ofs = DCSS_HDR10_A0_LUT + comp * 0x1000;

	for (i = 0; i < HDR10_LUT_MAX_ENTRIES; i++) {
		u32 reg_ofs = lut_base_ofs + i * sizeof(u32);

		dcss_hdr10_write(dcss, ch_num, map[i], reg_ofs);
	}
}

void dcss_hdr10_cfg(struct dcss_soc *dcss)
{
	struct dcss_hdr10_priv *hdr10 = dcss->hdr10_priv;
	struct dcss_hdr10_ch *ch;
	int i;
	u16 *lut;

	for (i = 0; i < 4; i++) {
		ch = &hdr10->ch[i];

		lut = i < 3 ? dcss_hdr10_comp_lut : dcss_hdr10_opipe;

		dcss_hdr10_lut_fill(dcss, i, 0, lut);
		dcss_hdr10_lut_fill(dcss, i, 1, lut);
		dcss_hdr10_lut_fill(dcss, i, 2, lut);

		ch->old_out_cs = DCSS_COLORSPACE_UNKNOWN;
		ch->old_in_cs = DCSS_COLORSPACE_UNKNOWN;
	}
}

static int dcss_hdr10_ch_init_all(struct dcss_soc *dcss,
				  unsigned long hdr10_base)
{
	struct dcss_hdr10_priv *hdr10 = dcss->hdr10_priv;
	struct dcss_hdr10_ch *ch;
	int i;

	for (i = 0; i < 4; i++) {
		ch = &hdr10->ch[i];

		ch->base_ofs = hdr10_base + i * 0x4000;

		ch->base_reg = devm_ioremap(dcss->dev, ch->base_ofs, SZ_16K);
		if (!ch->base_reg) {
			dev_err(dcss->dev, "hdr10: unable to remap ch base\n");
			return -ENOMEM;
		}

#if defined(USE_CTXLD)
		ch->ctx_id = CTX_SB_HP;
#endif
	}

#ifndef CONFIG_PM
	dcss_hdr10_cfg(dcss);
#endif

	return 0;
}

int dcss_hdr10_init(struct dcss_soc *dcss, unsigned long hdr10_base)
{
	struct dcss_hdr10_priv *hdr10;

	hdr10 = devm_kzalloc(dcss->dev, sizeof(*hdr10), GFP_KERNEL);
	if (!hdr10)
		return -ENOMEM;

	dcss->hdr10_priv = hdr10;
	hdr10->dcss = dcss;

	return dcss_hdr10_ch_init_all(dcss, hdr10_base);
}

void dcss_hdr10_exit(struct dcss_soc *dcss)
{
}

static void dcss_hdr10_csc_en(struct dcss_soc *dcss, int ch_num,
			      enum dcss_hdr10_csc csc_to_en, bool en)
{
	u32 ctrl_reg[] = {
		DCSS_HDR10_CSCA_BASE + DCSS_HDR10_CSC_CONTROL,
		DCSS_HDR10_CSCB_BASE + DCSS_HDR10_CSC_CONTROL,
	};

	dcss_hdr10_write(dcss, ch_num, en ? CSC_EN | CSC_ALL_PIX_EN : 0,
			 ctrl_reg[csc_to_en]);
}

static void dcss_hdr10_cs2rgb_setup(struct dcss_soc *dcss, int ch_num,
				    enum dcss_color_space in_cs)
{
	if (in_cs == DCSS_COLORSPACE_YUV) {
		dcss_hdr10_csc_fill(dcss, ch_num, HDR10_CSCA,
				    dcss_hdr10_yuv2rgb_csca);
		dcss_hdr10_csc_en(dcss, ch_num, HDR10_CSCA, true);
		dcss_hdr10_csc_fill(dcss, ch_num, HDR10_CSCB,
				    dcss_hdr10_yuv2rgb_cscb);
		dcss_hdr10_csc_en(dcss, ch_num, HDR10_CSCB, true);
	} else {
		dcss_hdr10_csc_fill(dcss, ch_num, HDR10_CSCA,
				    dcss_hdr10_rgb2rgb_csca);
		dcss_hdr10_csc_en(dcss, ch_num, HDR10_CSCA, true);
		dcss_hdr10_csc_fill(dcss, ch_num, HDR10_CSCB,
				    dcss_hdr10_rgb2rgb_cscb);
		dcss_hdr10_csc_en(dcss, ch_num, HDR10_CSCB, true);
	}

	dcss_hdr10_write(dcss, ch_num, LUT_ENABLE | LUT_EN_FOR_ALL_PELS,
			 DCSS_HDR10_LUT_CONTROL);
	dcss_hdr10_write(dcss, OPIPE_CH_NO,
			 LTNL_EN_FOR_ALL_PELS | FIX2FLT_EN_FOR_ALL_PELS,
			 DCSS_HDR10_LTNL);
}

void dcss_hdr10_pipe_csc_setup(struct dcss_soc *dcss, int ch_num,
			       enum dcss_color_space in_cs,
			       enum dcss_color_space out_cs)
{
	struct dcss_hdr10_ch *ch = &dcss->hdr10_priv->ch[ch_num];
	bool cs_chgd = (in_cs != ch->old_in_cs) || (out_cs != ch->old_out_cs);

	if (out_cs == DCSS_COLORSPACE_RGB && cs_chgd)
		dcss_hdr10_cs2rgb_setup(dcss, ch_num, in_cs);

	ch->old_in_cs = in_cs;
	ch->old_out_cs = out_cs;
}
EXPORT_SYMBOL(dcss_hdr10_pipe_csc_setup);

