// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/seq_file.h>

#include "dcss-dev.h"

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

struct dcss_wrscl {
	struct device *dev;

	void __iomem *base_reg;
	u32 base_ofs;

	struct dcss_ctxld *ctxld;
	u32 ctx_id;

	u32 buf_size;
	u32 buf_addr;
	void *buf_vaddr;

	struct clk *bclk;

	u32 ctrl_status;
};

static void dcss_wrscl_write(struct dcss_wrscl *wrscl, u32 val, u32 ofs)
{
	dcss_ctxld_write(wrscl->ctxld, wrscl->ctx_id,
			 val, wrscl->base_ofs + ofs);
}

int dcss_wrscl_init(struct dcss_dev *dcss, unsigned long wrscl_base)
{
	struct dcss_wrscl *wrscl;

	wrscl = devm_kzalloc(dcss->dev, sizeof(*wrscl), GFP_KERNEL);
	if (!wrscl)
		return -ENOMEM;

	wrscl->base_reg = devm_ioremap(dcss->dev, wrscl_base, SZ_4K);
	if (!wrscl->base_reg) {
		dev_err(dcss->dev, "wrscl: unable to remap base\n");
		devm_kfree(dcss->dev, wrscl);
		return -ENOMEM;
	}

	dcss->wrscl = wrscl;

	wrscl->dev = dcss->dev;
	wrscl->base_ofs = wrscl_base;
	wrscl->ctxld = dcss->ctxld;
	wrscl->ctx_id = CTX_SB_HP;
	wrscl->bclk = dcss->axi_clk;

	return 0;
}

void dcss_wrscl_exit(struct dcss_wrscl *wrscl)
{
	devm_iounmap(wrscl->dev, wrscl->base_reg);
	devm_kfree(wrscl->dev, wrscl);
}

static const u16 dcss_wrscl_psize_map[] = {64, 128, 256, 512, 1024, 2048, 4096};

u32 dcss_wrscl_setup(struct dcss_wrscl *wrscl, u32 pix_format, u32 vrefresh_hz,
		     u32 dst_xres, u32 dst_yres)
{
	u32 pitch, p_size, p_freq, bpp;
	dma_addr_t dma_handle;
	u32 bclk_rate = clk_get_rate(wrscl->bclk);

	/* we'd better release the old buffer */
	if (wrscl->buf_addr)
		dmam_free_coherent(wrscl->dev, wrscl->buf_size,
				   wrscl->buf_vaddr, wrscl->buf_addr);

	p_size = PSIZE_256;

	/* scaler output is YUV444 */
	bpp = 4;

	/* spread the load over the entire frame */
	p_freq = ((u64)bclk_rate * dcss_wrscl_psize_map[p_size]) /
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

	wrscl->buf_vaddr = dmam_alloc_coherent(wrscl->dev, wrscl->buf_size,
					       &dma_handle, GFP_KERNEL);
	if (!wrscl->buf_vaddr) {
		dev_err(wrscl->dev, "wrscl: cannot alloc buf mem\n");
		return 0;
	}

	wrscl->buf_addr = dma_handle;

	dcss_wrscl_write(wrscl, wrscl->buf_addr, DCSS_WRSCL_BASE_ADDR);
	dcss_wrscl_write(wrscl, pitch, DCSS_WRSCL_PITCH);

	return wrscl->buf_addr;
}

void dcss_wrscl_enable(struct dcss_wrscl *wrscl)
{
	wrscl->ctrl_status |= WRSCL_ENABLE | WRSCL_REPEAT;

	dcss_wrscl_write(wrscl, wrscl->ctrl_status, DCSS_WRSCL_CTRL_STATUS);
}

void dcss_wrscl_disable(struct dcss_wrscl *wrscl)
{
	wrscl->ctrl_status &= ~(WRSCL_ENABLE | WRSCL_REPEAT);

	dcss_wrscl_write(wrscl, wrscl->ctrl_status, DCSS_WRSCL_CTRL_STATUS);

	if (wrscl->buf_addr) {
		dmam_free_coherent(wrscl->dev, wrscl->buf_size,
				   wrscl->buf_vaddr, wrscl->buf_addr);
		wrscl->buf_addr = 0;
	}
}
