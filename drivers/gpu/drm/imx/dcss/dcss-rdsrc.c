// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/seq_file.h>

#include "dcss-dev.h"

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

struct dcss_rdsrc {
	struct device *dev;

	void __iomem *base_reg;
	u32 base_ofs;

	struct dcss_ctxld *ctxld;
	u32 ctx_id;

	u32 buf_addr;

	u32 ctrl_status;
};

static void dcss_rdsrc_write(struct dcss_rdsrc *rdsrc, u32 val, u32 ofs)
{
	dcss_ctxld_write(rdsrc->ctxld, rdsrc->ctx_id, val,
			 rdsrc->base_ofs + ofs);
}

int dcss_rdsrc_init(struct dcss_dev *dcss, unsigned long rdsrc_base)
{
	struct dcss_rdsrc *rdsrc;

	rdsrc = devm_kzalloc(dcss->dev, sizeof(*rdsrc), GFP_KERNEL);
	if (!rdsrc)
		return -ENOMEM;

	rdsrc->base_reg = devm_ioremap(dcss->dev, rdsrc_base, SZ_4K);
	if (!rdsrc->base_reg) {
		dev_err(dcss->dev, "rdsrc: unable to remap base\n");
		devm_kfree(dcss->dev, rdsrc);
		return -ENOMEM;
	}

	dcss->rdsrc = rdsrc;

	rdsrc->dev = dcss->dev;
	rdsrc->base_ofs = rdsrc_base;
	rdsrc->ctxld = dcss->ctxld;
	rdsrc->ctx_id = CTX_SB_HP;

	return 0;
}

void dcss_rdsrc_exit(struct dcss_rdsrc *rdsrc)
{
	devm_iounmap(rdsrc->dev, rdsrc->base_reg);
	devm_kfree(rdsrc->dev, rdsrc);
}

void dcss_rdsrc_setup(struct dcss_rdsrc *rdsrc, u32 pix_format, u32 dst_xres,
		      u32 dst_yres, u32 base_addr)
{
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

void dcss_rdsrc_enable(struct dcss_rdsrc *rdsrc)
{
	dcss_rdsrc_write(rdsrc, rdsrc->ctrl_status, DCSS_RDSRC_CTRL_STATUS);
}

void dcss_rdsrc_disable(struct dcss_rdsrc *rdsrc)
{
	/* RDSRC is turned off by setting the width and height to 0 */
	dcss_rdsrc_write(rdsrc, 0, DCSS_RDSRC_WIDTH);
	dcss_rdsrc_write(rdsrc, 0, DCSS_RDSRC_HEIGHT);

	dcss_rdsrc_write(rdsrc, rdsrc->ctrl_status, DCSS_RDSRC_CTRL_STATUS);
}
