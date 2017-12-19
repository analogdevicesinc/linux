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
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <video/imx-dcss.h>
#include "dcss-prv.h"

#define USE_CTXLD

#define DCSS_RDSRC_CTRL_STATUS			0x00
#define   RDSRC_RD_ERR				BIT(31)
#define   RDSRC_FRAME_COMP			BIT(30)
#define   RDSRC_FIFO_SIZE_POS			16
#define   RDSRC_FIFO_SIZE_MASK			GENMASK(22, 16)
#define   RDSRC_RD_ERR_EN			BIT(15)
#define   RDSRC_FRAME_COMP_EN			BIT(14)
#define   RDSRC_P_SIZE_POS			7
#define   RDSRC_P_SIZE_MASK			GENMASK(9, 7)
#define   RDSRC_T_SIZE_POS			5
#define   RDSRC_T_SIZE_MASK			GENMASK(6, 5)
#define   RDSRC_BPP_POS				2
#define   RDSRC_BPP_MASK			GENMASK(4, 2)
#define   RDSRC_ENABLE				BIT(0)
#define DCSS_RDSRC_BASE_ADDR			0x10
#define DCSS_RDSRC_PITCH			0x14
#define DCSS_RDSRC_WIDTH			0x18
#define DCSS_RDSRC_HEIGHT			0x1C

struct dcss_rdsrc_priv {
	void __iomem *base_reg;
	u32 base_ofs;
	struct dcss_soc *dcss;

	u32 ctx_id;

	u32 buf_addr;

	u32 ctrl_status;
};

#ifdef CONFIG_DEBUG_FS
static struct dcss_debug_reg rdsrc_debug_reg[] = {
	DCSS_DBG_REG(DCSS_RDSRC_CTRL_STATUS),
	DCSS_DBG_REG(DCSS_RDSRC_BASE_ADDR),
	DCSS_DBG_REG(DCSS_RDSRC_PITCH),
	DCSS_DBG_REG(DCSS_RDSRC_WIDTH),
	DCSS_DBG_REG(DCSS_RDSRC_HEIGHT),
};

void dcss_rdsrc_dump_regs(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	int i;

	seq_puts(s, ">> Dumping RD_SRC:\n");
	for (i = 0; i < ARRAY_SIZE(rdsrc_debug_reg); i++) {
		seq_printf(s, "%-35s(0x%04x) -> 0x%08x\n",
			   rdsrc_debug_reg[i].name,
			   rdsrc_debug_reg[i].ofs,
			   dcss_readl(dcss->rdsrc_priv->base_reg +
				      rdsrc_debug_reg[i].ofs));
	}
}
#endif

static void dcss_rdsrc_write(struct dcss_rdsrc_priv *rdsrc, u32 val, u32 ofs)
{
#if !defined(USE_CTXLD)
	dcss_writel(val, rdsrc->base_reg + ofs);
#else
	dcss_ctxld_write(rdsrc->dcss, rdsrc->ctx_id,
			 val, rdsrc->base_ofs + ofs);
#endif
}

int dcss_rdsrc_init(struct dcss_soc *dcss, unsigned long rdsrc_base)
{
	struct dcss_rdsrc_priv *rdsrc;

	rdsrc = devm_kzalloc(dcss->dev, sizeof(*rdsrc), GFP_KERNEL);
	if (!rdsrc)
		return -ENOMEM;

	rdsrc->base_reg = devm_ioremap(dcss->dev, rdsrc_base, SZ_4K);
	if (!rdsrc->base_reg) {
		dev_err(dcss->dev, "rdsrc: unable to remap base\n");
		return -ENOMEM;
	}

	dcss->rdsrc_priv = rdsrc;
	rdsrc->base_ofs = rdsrc_base;
	rdsrc->dcss = dcss;

#if defined(USE_CTXLD)
	rdsrc->ctx_id = CTX_SB_HP;
#endif

	return 0;
}

void dcss_rdsrc_exit(struct dcss_soc *dcss)
{
}

void dcss_rdsrc_setup(struct dcss_soc *dcss, u32 pix_format, u32 dst_xres,
		      u32 dst_yres, u32 base_addr)
{
	struct dcss_rdsrc_priv *rdsrc = dcss->rdsrc_priv;
	u32 buf_size, pitch, bpp;

	/* since the scaler output is YUV444, the RDSRC output has to match */
	bpp = 4;

	rdsrc->ctrl_status = FIFO_512 << RDSRC_FIFO_SIZE_POS;
	rdsrc->ctrl_status |= PSIZE_256 << RDSRC_P_SIZE_POS;
	rdsrc->ctrl_status |= TSIZE_256 << RDSRC_T_SIZE_POS;
	rdsrc->ctrl_status |= BPP_32_10BIT_OUTPUT << RDSRC_BPP_POS;

	buf_size = dst_xres * dst_yres * bpp;
	pitch = dst_xres * bpp;

	rdsrc->buf_addr = base_addr;

	dcss_rdsrc_write(rdsrc, rdsrc->buf_addr, DCSS_RDSRC_BASE_ADDR);
	dcss_rdsrc_write(rdsrc, pitch, DCSS_RDSRC_PITCH);
	dcss_rdsrc_write(rdsrc, dst_xres, DCSS_RDSRC_WIDTH);
	dcss_rdsrc_write(rdsrc, dst_yres, DCSS_RDSRC_HEIGHT);
}

void dcss_rdsrc_enable(struct dcss_soc *dcss, bool en)
{
	struct dcss_rdsrc_priv *rdsrc = dcss->rdsrc_priv;

	/* RDSRC is turned off by setting the width and height to 0 */
	if (!en) {
		dcss_rdsrc_write(rdsrc, 0, DCSS_RDSRC_WIDTH);
		dcss_rdsrc_write(rdsrc, 0, DCSS_RDSRC_HEIGHT);
	}

	dcss_rdsrc_write(rdsrc, rdsrc->ctrl_status, DCSS_RDSRC_CTRL_STATUS);
}
