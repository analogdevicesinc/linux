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
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include <video/imx-dcss.h>
#include "dcss-prv.h"

#define USE_CTXLD

#define DCSS_WRSCL_CTRL_STATUS			0x00
#define   WRSCL_ERR				BIT(31)
#define   WRSCL_ERR_EN				BIT(30)
#define   WRSCL_FRAME_COMP			BIT(29)
#define   WRSCL_FRAME_COMP_EN			BIT(28)
#define   WRSCL_FIFO_SIZE_POS			18
#define   WRSCL_FIFO_SIZE_MASK			GENMAK(24, 18)
#define   WRSCL_P_FREQ_POS			10
#define   WRSCL_P_FREQ_MASK			GENMASK(17, 10)
#define   WRSCL_P_SIZE_POS			7
#define   WRSCL_P_SIZE_MASK			GENMASK(9, 7)
#define   WRSCL_T_SIZE_POS			5
#define   WRSCL_T_SIZE_MASK			GENMASK(6, 5)
#define   WRSCL_BPP_POS				2
#define   WRSCL_BPP_MASK			GENMASK(4, 2)
#define   WRSCL_REPEAT				BIT(1)
#define   WRSCL_ENABLE				BIT(0)
#define DCSS_WRSCL_BASE_ADDR			0x10
#define DCSS_WRSCL_PITCH			0x14

struct dcss_wrscl_priv {
	void __iomem *base_reg;
	u32 base_ofs;
	struct dcss_soc *dcss;

	u32 ctx_id;

	u32 buf_size;
	u32 buf_addr;
	void *buf_vaddr;

	u32 ctrl_status;
};

#ifdef CONFIG_DEBUG_FS
static struct dcss_debug_reg wrscl_debug_reg[] = {
	DCSS_DBG_REG(DCSS_WRSCL_CTRL_STATUS),
	DCSS_DBG_REG(DCSS_WRSCL_BASE_ADDR),
	DCSS_DBG_REG(DCSS_WRSCL_PITCH),
};

void dcss_wrscl_dump_regs(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	int i;

	seq_puts(s, ">> Dumping WR_SCL:\n");
	for (i = 0; i < ARRAY_SIZE(wrscl_debug_reg); i++) {
		seq_printf(s, "%-35s(0x%04x) -> 0x%08x\n",
			   wrscl_debug_reg[i].name,
			   wrscl_debug_reg[i].ofs,
			   dcss_readl(dcss->wrscl_priv->base_reg +
				      wrscl_debug_reg[i].ofs));
	}
}
#endif

static void dcss_wrscl_write(struct dcss_wrscl_priv *wrscl, u32 val, u32 ofs)
{
#if !defined(USE_CTXLD)
	dcss_writel(val, wrscl->base_reg + ofs);
#else
	dcss_ctxld_write(wrscl->dcss, wrscl->ctx_id,
			 val, wrscl->base_ofs + ofs);
#endif
}

int dcss_wrscl_init(struct dcss_soc *dcss, unsigned long wrscl_base)
{
	struct dcss_wrscl_priv *wrscl;

	wrscl = devm_kzalloc(dcss->dev, sizeof(*wrscl), GFP_KERNEL);
	if (!wrscl)
		return -ENOMEM;

	wrscl->base_reg = devm_ioremap(dcss->dev, wrscl_base, SZ_4K);
	if (!wrscl->base_reg) {
		dev_err(dcss->dev, "wrscl: unable to remap base\n");
		return -ENOMEM;
	}

	dcss->wrscl_priv = wrscl;
	wrscl->base_ofs = wrscl_base;
	wrscl->dcss = dcss;

#if defined(USE_CTXLD)
	wrscl->ctx_id = CTX_SB_HP;
#endif

	return 0;
}

void dcss_wrscl_exit(struct dcss_soc *dcss)
{
}

static const u16 dcss_wrscl_psize_map[] = {64, 128, 256, 512, 1024, 2048, 4096};

u32 dcss_wrscl_setup(struct dcss_soc *dcss, u32 pix_format, u32 vrefresh_hz,
		     u32 dst_xres, u32 dst_yres)
{
	struct dcss_wrscl_priv *wrscl = dcss->wrscl_priv;
	u32 pitch, p_size, p_freq, bpp;
	dma_addr_t dma_handle;
	u32 b_clk = clk_get_rate(dcss->axi_clk);

	/* we'd better release the old buffer */
	if (wrscl->buf_addr)
		dmam_free_coherent(dcss->dev, wrscl->buf_size,
				   wrscl->buf_vaddr, wrscl->buf_addr);

	p_size = PSIZE_256;

	/* scaler output is YUV444 */
	bpp = 4;

	/* spread the load over the entire frame */
	p_freq = ((u64)b_clk * dcss_wrscl_psize_map[p_size]) /
		 ((u64)dst_xres * dst_yres * vrefresh_hz * bpp * 8);

	/* choose a slightly smaller p_freq */
	p_freq = p_freq - 3 > 255 ? 255 : p_freq - 3;

	wrscl->ctrl_status = FIFO_512 << WRSCL_FIFO_SIZE_POS;
	wrscl->ctrl_status |= p_size << WRSCL_P_SIZE_POS;
	wrscl->ctrl_status |= TSIZE_256 << WRSCL_T_SIZE_POS;
	wrscl->ctrl_status |= BPP_32_10BIT_OUTPUT << WRSCL_BPP_POS;
	wrscl->ctrl_status |= p_freq << WRSCL_P_FREQ_POS;

	wrscl->buf_size = dst_xres * dst_yres * bpp;
	pitch = dst_xres * bpp;

	wrscl->buf_vaddr = dmam_alloc_coherent(dcss->dev, wrscl->buf_size,
				    &dma_handle, GFP_KERNEL);
	if (!wrscl->buf_vaddr) {
		dev_err(dcss->dev, "wrscl: cannot alloc buf mem\n");
		return 0;
	}

	wrscl->buf_addr = dma_handle;

	dcss_wrscl_write(wrscl, wrscl->buf_addr, DCSS_WRSCL_BASE_ADDR);
	dcss_wrscl_write(wrscl, pitch, DCSS_WRSCL_PITCH);

	return wrscl->buf_addr;
}

void dcss_wrscl_enable(struct dcss_soc *dcss, bool en)
{
	struct dcss_wrscl_priv *wrscl = dcss->wrscl_priv;

	if (en)
		wrscl->ctrl_status |= WRSCL_ENABLE | WRSCL_REPEAT;
	else
		wrscl->ctrl_status &= ~(WRSCL_ENABLE | WRSCL_REPEAT);

	dcss_wrscl_write(wrscl, wrscl->ctrl_status, DCSS_WRSCL_CTRL_STATUS);

	if (!en && wrscl->buf_addr) {
		dmam_free_coherent(dcss->dev, wrscl->buf_size,
				   wrscl->buf_vaddr, wrscl->buf_addr);
		wrscl->buf_addr = 0;
	}
}

