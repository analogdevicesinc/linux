/*
 * Copyright (C) 2017 Freescale Semiconductor, Inc. All Rights Reserved.
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

enum mxc_jpeg_image_format {
	MXC_JPEG_YUV420	= 0x0,
	MXC_JPEG_YUV422 = 0x1,
	MXC_JPEG_RGB	= 0x2,
	MXC_JPEG_YUV444	= 0x3,
	MXC_JPEG_Y	= 0x4,
	MXC_JPEG_ARGB	= 0x6,
};

void print_descriptor_info(struct mxc_jpeg_desc *desc)
{
	printk(KERN_DEBUG " MXC JPEG NEXT PTR %x\n", desc->next_descpt_ptr);
	printk(KERN_DEBUG " MXC JPEG BUF BASE0 %x\n", desc->buf_base0);
	printk(KERN_DEBUG " MXC JPEG PITCH %d\n", desc->line_pitch);
	printk(KERN_DEBUG " MXC JPEG BUF BASE %x\n", desc->stm_bufbase);
	printk(KERN_DEBUG " MXC JPEG BUF SIZE %d\n", desc->stm_bufsize);
	printk(KERN_DEBUG " MXC JPEG IMGSIZE %dx%d\n", desc->w, desc->h);
	printk(KERN_DEBUG " MXC JPEG STM CTRL %x\n", desc->stm_ctrl);
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

	writel(0x1e0, reg + CAST_STATUS0);
	writel(0x3ff, reg + CAST_STATUS1);
	writel(0x4b, reg + CAST_STATUS2);

	slot = mxc_jpeg_get_slot(reg);
	mxc_jpeg_enable_irq(reg, slot);
	writel((1 << (slot + 4)), reg);

	cfg_desc->next_descpt_ptr = 0;
	cfg_desc->buf_base0 = tbl_handle;
	cfg_desc->buf_base1 = 0;
	cfg_desc->line_pitch = 0x300;
	cfg_desc->stm_bufbase = jpg_handle;
	cfg_desc->stm_bufsize = 0x100000;
	cfg_desc->w = 0x0100;
	cfg_desc->h = 0x0100;
	cfg_desc->stm_ctrl = MXC_CONFIG_MOD;

	//print_descriptor_info(cfg_desc);
	writel(cfg_handle, reg + MXC_SLOT_OFFSET(slot, SLOT_NXT_DESCPT_PTR));
	writel(cfg_handle | 1, reg +
	       MXC_SLOT_OFFSET(slot, SLOT_NXT_DESCPT_PTR));
	writel((1 << (slot + 4)) | MXC_ENDIAN_MD | MXC_ENABLE_DEC |
	       MXC_DEC_GO, reg);
	regval = readl(reg + STM_BUFBASE);
	regval = readl(reg + MXC_SLOT_OFFSET(slot, SLOT_BUF_PTR));
}

int mxc_jpeg_enable(void __iomem *reg)
{
	u32 regval;

	regval = readl(reg);
	writel(MXC_ENABLE_DEC, reg);
	regval = readl(reg);
	return regval;
}

void mxc_jpeg_go(void __iomem *reg)
{
	u32 val;

	val = readl(reg);
	writel(MXC_ENDIAN_MD | MXC_DEC_GO | val, reg);
	writel(0x4, reg + 0x134);
	//print_cast_decoder_info(reg);
}

void print_cast_decoder_info(void __iomem *reg)
{
	int regval;

	regval = readl(reg + MXC_SLOT_OFFSET(0, SLOT_CUR_DESCPT_PTR));
	printk(KERN_DEBUG " MXC_JPEG: CUR DESCPT PTR: %x\n", regval);
	regval = readl(reg + 0x100);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 1: %x\n", regval);
	regval = readl(reg + 0x104);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 2: %x\n", regval);
	regval = readl(reg + 0x108);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 3: %x\n", regval);
	regval = readl(reg + 0x10c);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 4: %x\n", regval);
	regval = readl(reg + 0x110);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 5: %x\n", regval);
	regval = readl(reg + 0x114);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 6: %x\n", regval);
	regval = readl(reg + 0x118);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 7: %x\n", regval);
	regval = readl(reg + 0x11c);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 8: %x\n", regval);
	regval = readl(reg + 0x120);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 9: %x\n", regval);
	regval = readl(reg + 0x124);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 10: %x\n", regval);
	regval = readl(reg + 0x128);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 11: %x\n", regval);
	regval = readl(reg + 0x12c);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 12: %x\n", regval);
	regval = readl(reg + 0x130);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 13: %x\n", regval);
	regval = readl(reg + 0x134);
	printk(KERN_DEBUG " MXC_JPEG: CAST_INFO 14: %x\n", regval);
}

int mxc_jpeg_get_slot(void __iomem *reg)
{
	int slot_val;
	int i = 0;
	int tmp = MXC_SLOT_EN;

	slot_val = readl(reg) & 0xF0;
	for (; tmp != tmp << 4; tmp = tmp << 1) {
		if ((slot_val & tmp) == 0)
			return i;
		++i;
	}
	return -EINVAL;
}

void mxc_jpeg_enable_slot(void __iomem *reg, int slot)
{
	u32 regval;

	regval = readl(reg);
	writel((1 << (slot + 4)) | regval, reg);
}

void mxc_jpeg_set_addrs(struct mxc_jpeg_desc *desc, u32 buf_base0, u32 bufbase)
{
	desc->buf_base0 = buf_base0;
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

void mxc_jpeg_set_res(struct mxc_jpeg_desc *desc, u16 w, u16 h)
{
	desc->w = w;
	desc->h = h;
}

void mxc_jpeg_set_desc(u32 desc, void __iomem *reg, int slot)
{
	writel(desc | 1, reg + MXC_SLOT_OFFSET(slot, SLOT_NXT_DESCPT_PTR));
}
