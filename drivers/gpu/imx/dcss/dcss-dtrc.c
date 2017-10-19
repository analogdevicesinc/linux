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
#include <drm/drm_fourcc.h>

#include <video/imx-dcss.h>
#include "dcss-prv.h"

#define USE_CTXLD

#define DTRC_F0_OFS			0x00
#define DTRC_F1_OFS			0x60

#define DCSS_DTRC_DYDSADDR			0x00
#define DCSS_DTRC_DCDSADDR			0x04
#define DCSS_DTRC_DYTSADDR			0x08
#define DCSS_DTRC_DCTSADDR			0x0C
#define DCSS_DTRC_SIZE				0x10
#define   FRAME_WIDTH_POS			0
#define   FRAME_WIDTH_MASK			GENMASK(9, 0)
#define   FRAME_HEIGHT_POS			16
#define   FRAME_HEIGHT_MASK			GENMASK(25, 16)
#define DCSS_DTRC_SYSSA				0x14
#define DCSS_DTRC_SYSEA				0x18
#define DCSS_DTRC_SUVSSA			0x1C
#define DCSS_DTRC_SUVSEA			0x20
#define DCSS_DTRC_CROPORIG			0x24
#define DCSS_DTRC_CROPSIZE			0x28
#define DCSS_DTRC_DCTL				0x2C
#define DCSS_DTRC_DYDSADDR_EXT			0x30
#define DCSS_DTRC_DCDSADDR_EXT			0x34
#define DCSS_DTRC_DYTSADDR_EXT			0x38
#define DCSS_DTRC_DCTSADDR_EXT			0x3C
#define DCSS_DTRC_SYSSA_EXT			0x40
#define DCSS_DTRC_SYSEA_EXT			0x44
#define DCSS_DTRC_SUVSSA_EXT			0x48
#define DCSS_DTRC_SUVSEA_EXT			0x4C

#define DCSS_DTRC_INTEN				0xC0
#define DCSS_DTRC_FDINTR			0xC4
#define DCSS_DTRC_DTCTRL			0xC8
#define   CURRENT_FRAME				BIT(31)
#define   HOT_RESET				BIT(2)
#define DCSS_DTRC_ARIDR				0xCC
#define DCSS_DTRC_DTID2DDR			0xD0
#define DCSS_DTRC_CONFIG			0xD4
#define DCSS_DTRC_VER				0xD8
#define DCSS_DTRC_PFCTRL			0xF0
#define DCSS_DTRC_PFCR				0xF4
#define DCSS_DTRC_TOCR				0xF8

struct dcss_dtrc_ch {
	void __iomem *base_reg;
	u32 base_ofs;

	u32 xres;
	u32 yres;
	u32 pix_format;

	u32 ctx_id;

	bool bypass;
};

struct dcss_dtrc_priv {
	struct dcss_soc *dcss;
	void __iomem *dtrc_reg;

	struct dcss_dtrc_ch ch[2];
};

#ifdef CONFIG_DEBUG_FS
static struct dcss_debug_reg dtrc_frame_debug_reg[] = {
	DCSS_DBG_REG(DCSS_DTRC_DYDSADDR),
	DCSS_DBG_REG(DCSS_DTRC_DCDSADDR),
	DCSS_DBG_REG(DCSS_DTRC_DYTSADDR),
	DCSS_DBG_REG(DCSS_DTRC_DCTSADDR),
	DCSS_DBG_REG(DCSS_DTRC_SIZE),
	DCSS_DBG_REG(DCSS_DTRC_SYSSA),
	DCSS_DBG_REG(DCSS_DTRC_SYSEA),
	DCSS_DBG_REG(DCSS_DTRC_SUVSSA),
	DCSS_DBG_REG(DCSS_DTRC_SUVSEA),
	DCSS_DBG_REG(DCSS_DTRC_CROPORIG),
	DCSS_DBG_REG(DCSS_DTRC_CROPSIZE),
	DCSS_DBG_REG(DCSS_DTRC_DCTL),
	DCSS_DBG_REG(DCSS_DTRC_DYDSADDR_EXT),
	DCSS_DBG_REG(DCSS_DTRC_DCDSADDR_EXT),
	DCSS_DBG_REG(DCSS_DTRC_DYTSADDR_EXT),
	DCSS_DBG_REG(DCSS_DTRC_DCTSADDR_EXT),
	DCSS_DBG_REG(DCSS_DTRC_SYSSA_EXT),
	DCSS_DBG_REG(DCSS_DTRC_SYSEA_EXT),
	DCSS_DBG_REG(DCSS_DTRC_SUVSSA_EXT),
	DCSS_DBG_REG(DCSS_DTRC_SUVSEA_EXT),
};

static struct dcss_debug_reg dtrc_ctrl_debug_reg[] = {
	DCSS_DBG_REG(DCSS_DTRC_INTEN),
	DCSS_DBG_REG(DCSS_DTRC_FDINTR),
	DCSS_DBG_REG(DCSS_DTRC_DTCTRL),
	DCSS_DBG_REG(DCSS_DTRC_ARIDR),
	DCSS_DBG_REG(DCSS_DTRC_DTID2DDR),
	DCSS_DBG_REG(DCSS_DTRC_CONFIG),
	DCSS_DBG_REG(DCSS_DTRC_VER),
	DCSS_DBG_REG(DCSS_DTRC_PFCTRL),
	DCSS_DBG_REG(DCSS_DTRC_PFCR),
	DCSS_DBG_REG(DCSS_DTRC_TOCR),
};

static void dcss_dtrc_dump_frame_regs(struct seq_file *s, void *data,
				      int ch, int frame)
{
	struct dcss_soc *dcss = data;
	int i;

	seq_printf(s, "\t>> Dumping F%d regs:\n", frame);
	for (i = 0; i < ARRAY_SIZE(dtrc_frame_debug_reg); i++)
		seq_printf(s, "\t%-35s(0x%04x) -> 0x%08x\n",
			   dtrc_frame_debug_reg[i].name,
			   dtrc_frame_debug_reg[i].ofs + frame * DTRC_F1_OFS,
			   dcss_readl(dcss->dtrc_priv->ch[ch].base_reg +
				      dtrc_frame_debug_reg[i].ofs +
				      frame * DTRC_F1_OFS));
}

void dcss_dtrc_dump_regs(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	int ch, fr, i;

	for (ch = 0; ch < 2; ch++) {
		seq_printf(s, ">> Dumping DTRC for CH %d:\n", ch + 1);
		for (fr = 0; fr < 2; fr++)
			dcss_dtrc_dump_frame_regs(s, data, ch, fr);

		seq_printf(s, "\t>> Dumping DTRC CTRL regs for CH %d:\n",
			   ch + 1);
		for (i = 0; i < ARRAY_SIZE(dtrc_ctrl_debug_reg); i++)
			seq_printf(s, "\t%-35s(0x%04x) -> 0x%08x\n",
				   dtrc_ctrl_debug_reg[i].name,
				   dtrc_ctrl_debug_reg[i].ofs,
				   dcss_readl(dcss->dtrc_priv->ch[ch].base_reg +
					      dtrc_ctrl_debug_reg[i].ofs));
	}
}
#endif

static int dcss_dtrc_ch_init_all(struct dcss_soc *dcss, unsigned long dtrc_base)
{
	struct dcss_dtrc_priv *dtrc = dcss->dtrc_priv;
	struct dcss_dtrc_ch *ch;
	int i;

	for (i = 0; i < 2; i++) {
		ch = &dtrc->ch[i];

		ch->base_ofs = dtrc_base + i * 0x1000;

		ch->base_reg = devm_ioremap(dcss->dev, ch->base_ofs, SZ_4K);
		if (!ch->base_reg) {
			dev_err(dcss->dev, "dtrc: unable to remap ch base\n");
			return -ENOMEM;
		}
	}

	return 0;
}

static void dcss_dtrc_write(struct dcss_dtrc_priv *dtrc, int ch_num,
			    u32 val, u32 ofs)
{
#if !defined(USE_CTXLD)
	dcss_writel(val, dtrc->ch[ch_num].base_reg + ofs);
#else
	dcss_ctxld_write(dtrc->dcss, dtrc->ch[ch_num].ctx_id,
			 val, dtrc->ch[ch_num].base_ofs + ofs);
#endif
}

int dcss_dtrc_init(struct dcss_soc *dcss, unsigned long dtrc_base)
{
	struct dcss_dtrc_priv *dtrc;

	dtrc = devm_kzalloc(dcss->dev, sizeof(*dtrc), GFP_KERNEL);
	if (!dtrc)
		return -ENOMEM;

	dcss->dtrc_priv = dtrc;
	dtrc->dcss = dcss;

	return dcss_dtrc_ch_init_all(dcss, dtrc_base);
}

void dcss_dtrc_exit(struct dcss_soc *dcss)
{
	struct dcss_dtrc_priv *dtrc = dcss->dtrc_priv;

	/* reset the module to default */
	dcss_writel(HOT_RESET, dtrc->dtrc_reg + DCSS_DTRC_DTCTRL);
}

void dcss_dtrc_bypass(struct dcss_soc *dcss, int ch_num)
{
	struct dcss_dtrc_priv *dtrc = dcss->dtrc_priv;

	if (ch_num == 0)
		return;

	ch_num -= 1;

	if (dtrc->ch[ch_num].bypass)
		return;

	dcss_dtrc_write(dtrc, ch_num, 2, DCSS_DTRC_DTCTRL);
	dcss_dtrc_write(dtrc, ch_num, 0, DCSS_DTRC_DYTSADDR);
	dcss_dtrc_write(dtrc, ch_num, 0, DCSS_DTRC_DCTSADDR);
	dcss_dtrc_write(dtrc, ch_num, 0x0f0e0100, DCSS_DTRC_ARIDR);
	dcss_dtrc_write(dtrc, ch_num, 0x0f0e, DCSS_DTRC_DTID2DDR);

	dtrc->ch[ch_num].bypass = true;
}
EXPORT_SYMBOL(dcss_dtrc_bypass);

void dcss_dtrc_addr_set(struct dcss_soc *dcss, int ch_num, u32 p1_ba, u32 p2_ba)
{
	struct dcss_dtrc_priv *dtrc = dcss->dtrc_priv;
	struct dcss_dtrc_ch *ch = &dtrc->ch[ch_num];
	int curr_frame;

	if (ch_num == 0)
		return;

	ch_num -= 1;

	curr_frame = dcss_readl(dtrc->ch[ch_num].base_reg + DCSS_DTRC_DTCTRL);
	curr_frame = (curr_frame & CURRENT_FRAME) >> 31;

	dcss_dtrc_write(dtrc, ch_num, p1_ba, DCSS_DTRC_DYDSADDR);
	dcss_dtrc_write(dtrc, ch_num, p2_ba, DCSS_DTRC_DCDSADDR);

	dcss_dtrc_write(dtrc, ch_num, p1_ba, DTRC_F1_OFS + DCSS_DTRC_DYDSADDR);
	dcss_dtrc_write(dtrc, ch_num, p2_ba, DTRC_F1_OFS + DCSS_DTRC_DCDSADDR);

	dcss_dtrc_write(dtrc, ch_num, p1_ba, DCSS_DTRC_SYSSA);
	dcss_dtrc_write(dtrc, ch_num, p1_ba + ch->xres * ch->yres,
			DCSS_DTRC_SYSEA);

	dcss_dtrc_write(dtrc, ch_num, p1_ba, DTRC_F1_OFS + DCSS_DTRC_SYSSA);
	dcss_dtrc_write(dtrc, ch_num, p1_ba + ch->xres * ch->yres,
			DTRC_F1_OFS + DCSS_DTRC_SYSEA);

	dcss_dtrc_write(dtrc, ch_num, p2_ba, DCSS_DTRC_SUVSSA);
	dcss_dtrc_write(dtrc, ch_num, p2_ba + ch->xres * ch->yres / 4,
			DCSS_DTRC_SUVSEA);

	dcss_dtrc_write(dtrc, ch_num, p2_ba, DTRC_F1_OFS + DCSS_DTRC_SUVSSA);
	dcss_dtrc_write(dtrc, ch_num, p2_ba + ch->xres * ch->yres / 4,
			DTRC_F1_OFS + DCSS_DTRC_SUVSEA);

	dcss_dtrc_write(dcss->dtrc_priv, ch_num - 1, 0x0f0e0100,
			DCSS_DTRC_ARIDR);
	dcss_dtrc_write(dcss->dtrc_priv, ch_num - 1, 0x0f0e,
			DCSS_DTRC_DTID2DDR);

	dcss_dtrc_write(dtrc, ch_num, 0x50f00108, DCSS_DTRC_DTCTRL);

	/* TODO: hardcoded this for testing purposes. */
	dcss_dtrc_write(dtrc, ch_num, 0x20002,
			(curr_frame ^ 1) * DTRC_F1_OFS + DCSS_DTRC_DCTL);
	dcss_dtrc_write(dtrc, ch_num, 0x20003,
			curr_frame * DTRC_F1_OFS + DCSS_DTRC_DCTL);

	dtrc->ch[ch_num].bypass = false;
}
EXPORT_SYMBOL(dcss_dtrc_addr_set);

void dcss_dtrc_set_res(struct dcss_soc *dcss, int ch_num, u32 xres, u32 yres)
{
	struct dcss_dtrc_priv *dtrc = dcss->dtrc_priv;
	u32 frame_height, frame_width;

	if (ch_num == 0)
		return;

	dtrc->ch[ch_num].xres = xres;
	dtrc->ch[ch_num].yres = yres;

	frame_height = ((yres >> 3) << FRAME_HEIGHT_POS) & FRAME_HEIGHT_MASK;
	frame_width = ((xres >> 3) << FRAME_WIDTH_POS) & FRAME_WIDTH_MASK;

	dcss_dtrc_write(dcss->dtrc_priv, ch_num - 1, frame_height | frame_width,
			DTRC_F0_OFS + DCSS_DTRC_SIZE);
	dcss_dtrc_write(dcss->dtrc_priv, ch_num - 1, frame_height | frame_width,
			DTRC_F1_OFS + DCSS_DTRC_SIZE);
}
EXPORT_SYMBOL(dcss_dtrc_set_res);
