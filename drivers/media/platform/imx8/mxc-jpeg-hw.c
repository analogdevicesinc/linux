/*
 * Copyright 2018 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <media/videobuf2-core.h>
#include "mxc-jpeg-hw.h"

void print_descriptor_info(struct device *dev, struct mxc_jpeg_desc *desc)
{
	dev_info(dev, " MXC JPEG NEXT PTR 0x%x\n", desc->next_descpt_ptr);
	dev_info(dev, " MXC JPEG BUF BASE0 0x%x\n", desc->buf_base0);
	dev_info(dev, " MXC JPEG PITCH %d\n", desc->line_pitch);
	dev_info(dev, " MXC JPEG BUF BASE 0x%x\n", desc->stm_bufbase);
	dev_info(dev, " MXC JPEG BUF SIZE %d\n", desc->stm_bufsize);
	dev_info(dev, " MXC JPEG IMGSIZE %x (%d x %d)\n", desc->imgsize,
		desc->imgsize >> 16, desc->imgsize & 0xFFFF);
	dev_info(dev, " MXC JPEG STM CTRL 0x%x\n", desc->stm_ctrl);
}

void mxc_jpeg_enable_irq(void __iomem *reg, int slot)
{
	writel(0xFFFFFFFF, reg + MXC_SLOT_OFFSET(slot, SLOT_IRQ_EN));
}

void mxc_jpeg_reset(void __iomem *reg)
{
	writel(MXC_ENABLE_DEC, reg);
}

u32 mxc_jpeg_get_offset(void __iomem *reg, int slot)
{
	return readl(reg + MXC_SLOT_OFFSET(slot, SLOT_BUF_PTR));
}

void mxc_jpeg_enc_config(void __iomem *reg, struct mxc_jpeg_desc *cfg_desc,
			 u32 cfg_handle, u32 tbl_handle, u32 jpg_handle)
{
	u32 regval, slot;

	writel(0x1e0, reg + CAST_STATUS0); /* X = Image width, RO reg */
	writel(0x3ff, reg + CAST_STATUS1); /* Y = Image height , RO reg */
	writel(0x4b, reg + CAST_STATUS2); /* HMCU , RO reg */

	slot = mxc_jpeg_get_slot(reg);
	mxc_jpeg_enable_irq(reg, slot);
	writel(MXC_SLOT_EN(slot), reg + GLB_CTRL);

	cfg_desc->next_descpt_ptr = 0;
	cfg_desc->buf_base0 = tbl_handle;
	cfg_desc->buf_base1 = 0;
	cfg_desc->line_pitch = 0x300;
	cfg_desc->stm_bufbase = jpg_handle;
	cfg_desc->stm_bufsize = 0x100000;
	cfg_desc->imgsize = 0x01000100;
	cfg_desc->stm_ctrl = MXC_CONFIG_MOD;

	//print_descriptor_info(cfg_desc);
	writel(cfg_handle, reg + MXC_SLOT_OFFSET(slot, SLOT_NXT_DESCPT_PTR));
	writel(cfg_handle | MXC_NXT_DESCPT_EN, reg +
	       MXC_SLOT_OFFSET(slot, SLOT_NXT_DESCPT_PTR));
	writel(MXC_SLOT_EN(slot) | MXC_ENDIAN_MD | MXC_ENABLE_DEC |
	       MXC_DEC_GO, reg + GLB_CTRL);
	regval = readl(reg + STM_BUFBASE);
	regval = readl(reg + MXC_SLOT_OFFSET(slot, SLOT_BUF_PTR));
}

int mxc_jpeg_enable(void __iomem *reg)
{
	u32 regval;

	writel(MXC_ENABLE_DEC, reg + GLB_CTRL);
	regval = readl(reg);
	return regval;
}

void mxc_jpeg_go(void __iomem *reg)
{
	u32 val;

	val = readl(reg + GLB_CTRL);
	writel(MXC_ENDIAN_MD | MXC_DEC_GO | val, reg + GLB_CTRL);
	writel(MXC_DEC_EXIT_IDLE_MODE, reg + CAST_STATUS13);
	//print_cast_decoder_info(reg);
}

void print_cast_decoder_info(struct device *dev, void __iomem *reg)
{
	int regval;

	regval = readl(reg + MXC_SLOT_OFFSET(0, SLOT_CUR_DESCPT_PTR));
	dev_info(dev, " MXC_JPEG: CUR DESCPT PTR: %x\n", regval);
	regval = readl(reg + CAST_STATUS0);
	dev_info(dev, " MXC_JPEG: CAST_INFO 0: %x\n", regval);
	regval = readl(reg + CAST_STATUS1);
	dev_info(dev, " MXC_JPEG: CAST_INFO 1: %x\n", regval);
	regval = readl(reg + CAST_STATUS2);
	dev_info(dev, " MXC_JPEG: CAST_INFO 2: %x\n", regval);
	regval = readl(reg + CAST_STATUS3);
	dev_info(dev, " MXC_JPEG: CAST_INFO 3: %x\n", regval);
	regval = readl(reg + CAST_STATUS4);
	dev_info(dev, " MXC_JPEG: CAST_INFO 4: %x\n", regval);
	regval = readl(reg + CAST_STATUS5);
	dev_info(dev, " MXC_JPEG: CAST_INFO 5: %x\n", regval);
	regval = readl(reg + CAST_STATUS6);
	dev_info(dev, " MXC_JPEG: CAST_INFO 6: %x\n", regval);
	regval = readl(reg + CAST_STATUS7);
	dev_info(dev, " MXC_JPEG: CAST_INFO 7: %x\n", regval);
	regval = readl(reg + CAST_STATUS8);
	dev_info(dev, " MXC_JPEG: CAST_INFO 8: %x\n", regval);
	regval = readl(reg + CAST_STATUS9);
	dev_info(dev, " MXC_JPEG: CAST_INFO 9: %x\n", regval);
	regval = readl(reg + CAST_STATUS10);
	dev_info(dev, " MXC_JPEG: CAST_INFO 10: %x\n", regval);
	regval = readl(reg + CAST_STATUS11);
	dev_info(dev, " MXC_JPEG: CAST_INFO 11: %x\n", regval);
	regval = readl(reg + CAST_STATUS12);
	dev_info(dev, " MXC_JPEG: CAST_INFO 12: %x\n", regval);
	regval = readl(reg + CAST_STATUS13);
	dev_info(dev, " MXC_JPEG: CAST_INFO 13: %x\n", regval);
}

int mxc_jpeg_get_slot(void __iomem *reg)
{
	int slot_val;
	int i = 0;
	int tmp = MXC_SLOT_EN(0);

	/* currently enabled slots */
	slot_val = readl(reg) & 0xF0;

	for (; tmp != tmp << 4; tmp = tmp << 1) {
		if ((slot_val & tmp) == 0)
			/* first free slot */
			return i;
		++i;
	}
	return -EINVAL;
}

void mxc_jpeg_enable_slot(void __iomem *reg, int slot)
{
	u32 regval;

	regval = readl(reg + GLB_CTRL);
	writel(MXC_SLOT_EN(slot) | regval, reg + GLB_CTRL);
}

void mxc_jpeg_set_addrs(struct mxc_jpeg_desc *desc, u32 buf_base0, u32 bufbase)
{
	desc->buf_base0 = buf_base0;
	desc->buf_base1 = 0x0;
	desc->stm_bufbase = bufbase;
}

int mxc_jpeg_set_params(struct mxc_jpeg_desc *desc,  u32 bufsize,
			 u16 out_pitch, u32 format)
{
	desc->line_pitch = out_pitch;
	desc->stm_bufsize = bufsize;
	switch (format) {
	case V4L2_PIX_FMT_YUV32:
		desc->stm_ctrl |= MXC_JPEG_YUV444 << 3;
		break;
	case V4L2_PIX_FMT_YUYV:
		desc->stm_ctrl |= MXC_JPEG_YUV422 << 3;
		break;
	case V4L2_PIX_FMT_RGB32:
		desc->stm_ctrl |= MXC_JPEG_RGB << 3;
		break;
	default:
		return -1;
	}
	return 0;
}

void mxc_jpeg_set_bufsize(struct mxc_jpeg_desc *desc,  u32 bufsize)
{
	desc->stm_bufsize = bufsize;
}

void mxc_jpeg_set_res(struct mxc_jpeg_desc *desc, u16 w, u16 h)
{
	desc->imgsize = w << 16 | h;
}

void mxc_jpeg_set_desc(u32 desc, void __iomem *reg, int slot)
{
	writel(desc | MXC_NXT_DESCPT_EN,
	       reg + MXC_SLOT_OFFSET(slot, SLOT_NXT_DESCPT_PTR));
}
