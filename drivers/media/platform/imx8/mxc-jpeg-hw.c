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

#define print_wrapper_reg(dev, base_address, reg_offset)\
		internal_print_wrapper_reg(dev, (base_address), #reg_offset,\
					   (reg_offset))
#define internal_print_wrapper_reg(dev, base_address, reg_name, reg_offset) {\
		int val;\
		val = readl((base_address) + (reg_offset));\
		dev_info(dev, "Wrapper reg %s = 0x%x\n", reg_name, val);\
}

void print_descriptor_info(struct device *dev, struct mxc_jpeg_desc *desc)
{
	dev_info(dev, " MXC JPEG NEXT_DESCPT_PTR 0x%x\n",
		desc->next_descpt_ptr);
	dev_info(dev, " MXC JPEG BUF_BASE0 0x%x\n", desc->buf_base0);
	dev_info(dev, " MXC JPEG BUF_BASE1 0x%x\n", desc->buf_base1);
	dev_info(dev, " MXC JPEG LINE_PITCH %d\n", desc->line_pitch);
	dev_info(dev, " MXC JPEG STM_BUFBASE 0x%x\n", desc->stm_bufbase);
	dev_info(dev, " MXC JPEG STM_BUFSIZE %d\n", desc->stm_bufsize);
	dev_info(dev, " MXC JPEG IMGSIZE %x (%d x %d)\n", desc->imgsize,
		desc->imgsize >> 16, desc->imgsize & 0xFFFF);
	dev_info(dev, " MXC JPEG STM_CTRL 0x%x\n", desc->stm_ctrl);
}

void print_cast_decoder_info(struct device *dev, void __iomem *reg)
{
	dev_info(dev, "CAST IP decoder regs:\n");
	print_wrapper_reg(dev, reg, CAST_STATUS0);
	print_wrapper_reg(dev, reg, CAST_STATUS1);
	print_wrapper_reg(dev, reg, CAST_STATUS2);
	print_wrapper_reg(dev, reg, CAST_STATUS3);
	print_wrapper_reg(dev, reg, CAST_STATUS4);
	print_wrapper_reg(dev, reg, CAST_STATUS5);
	print_wrapper_reg(dev, reg, CAST_STATUS6);
	print_wrapper_reg(dev, reg, CAST_STATUS7);
	print_wrapper_reg(dev, reg, CAST_STATUS8);
	print_wrapper_reg(dev, reg, CAST_STATUS9);
	print_wrapper_reg(dev, reg, CAST_STATUS10);
	print_wrapper_reg(dev, reg, CAST_STATUS11);
	print_wrapper_reg(dev, reg, CAST_STATUS12);
	print_wrapper_reg(dev, reg, CAST_STATUS13);
}

void print_cast_encoder_info(struct device *dev, void __iomem *reg)
{
	dev_info(dev, "CAST IP encoder regs:\n");
	print_wrapper_reg(dev, reg, CAST_MODE);
	print_wrapper_reg(dev, reg, CAST_CFG_MODE);
	print_wrapper_reg(dev, reg, CAST_QUALITY);
	print_wrapper_reg(dev, reg, CAST_RSVD);
	print_wrapper_reg(dev, reg, CAST_REC_REGS_SEL);
	print_wrapper_reg(dev, reg, CAST_LUMTH);
	print_wrapper_reg(dev, reg, CAST_CHRTH);
	print_wrapper_reg(dev, reg, CAST_NOMFRSIZE_LO);
	print_wrapper_reg(dev, reg, CAST_NOMFRSIZE_HI);
	print_wrapper_reg(dev, reg, CAST_OFBSIZE_LO);
	print_wrapper_reg(dev, reg, CAST_OFBSIZE_HI);
}

void print_wrapper_info(struct device *dev, void __iomem *reg)
{
	dev_info(dev, "Wrapper regs:\n");
	print_wrapper_reg(dev, reg, GLB_CTRL);
	print_wrapper_reg(dev, reg, COM_STATUS);
	print_wrapper_reg(dev, reg, BUF_BASE0);
	print_wrapper_reg(dev, reg, BUF_BASE1);
	print_wrapper_reg(dev, reg, LINE_PITCH);
	print_wrapper_reg(dev, reg, STM_BUFBASE);
	print_wrapper_reg(dev, reg, STM_BUFSIZE);
	print_wrapper_reg(dev, reg, IMGSIZE);
	print_wrapper_reg(dev, reg, STM_CTRL);
}

void mxc_jpeg_enable_irq(void __iomem *reg, int slot)
{
	writel(0xFFFFFFFF, reg + MXC_SLOT_OFFSET(slot, SLOT_IRQ_EN));
}

void mxc_jpeg_reset(void __iomem *reg)
{
	writel(GLB_CTRL_JPG_EN, reg + GLB_CTRL);
}

void mxc_jpeg_sw_reset(void __iomem *reg)
{
	writel(GLB_CTRL_SFT_RST | GLB_CTRL_JPG_EN, reg + GLB_CTRL);
}

u32 mxc_jpeg_get_offset(void __iomem *reg, int slot)
{
	return readl(reg + MXC_SLOT_OFFSET(slot, SLOT_BUF_PTR));
}

void mxc_jpeg_enc_config(struct device *dev,
			 void __iomem *reg, struct mxc_jpeg_desc *cfg_desc,
			 u32 cfg_handle, u32 cfg_stream_handle, u32 jpg_handle)
{
	u32 slot;

	/*
	 * "Config_Mode" enabled, "Config_Mode auto clear enabled",
	 * "GO" enabled, "GO bit auto clear" enabled
	 */
	writel(0x1e0, reg + CAST_MODE);

	/* all markers and segments */
	writel(0x3ff, reg + CAST_CFG_MODE);

	/* quality factor */
	writel(0x4b, reg + CAST_QUALITY);

	cfg_desc->next_descpt_ptr = 0;
	cfg_desc->buf_base0 = cfg_stream_handle;
	cfg_desc->buf_base1 = 0;
	cfg_desc->line_pitch = 0;
	cfg_desc->stm_bufbase = 0;
	cfg_desc->stm_bufsize = 0x2000;
	cfg_desc->imgsize = 0;
	cfg_desc->stm_ctrl = STM_CTRL_CONFIG_MOD(1) | STM_CTRL_AUTO_START(1);

	slot = 0; /* TODO get slot*/
	writel(GLB_CTRL_SLOT_EN(slot) | GLB_CTRL_L_ENDIAN | GLB_CTRL_JPG_EN,
	       reg + GLB_CTRL);

	mxc_jpeg_enable_irq(reg, slot);

	print_descriptor_info(dev, cfg_desc);
	print_wrapper_info(dev, reg);
	print_cast_encoder_info(dev, reg);

	mxc_jpeg_set_desc(cfg_handle, reg, slot);
	mxc_jpeg_go_auto(reg);
}

void wait_frmdone(struct device *dev, void __iomem *reg)
{
	u32 regval = 0;

	do {
		regval = readl(reg + MXC_SLOT_OFFSET(0, SLOT_STATUS));
	} while (!(regval & SLOTa_STATUS_FRMDONE));

	writel(regval, reg + MXC_SLOT_OFFSET(0, SLOT_STATUS)); /* w1c */

	dev_dbg(dev, "Received FRMDONE\n");
	if (regval & SLOTa_STATUS_ENC_CONFIG_ERR)
		dev_info(dev, "SLOTa_STATUS_ENC_CONFIG_ERR\n");
}

int mxc_jpeg_enable(void __iomem *reg)
{
	u32 regval;

	writel(GLB_CTRL_JPG_EN, reg + GLB_CTRL);
	regval = readl(reg);
	return regval;
}

void mxc_jpeg_go(void __iomem *reg)
{
	u32 val;

	val = readl(reg + GLB_CTRL);
	writel(GLB_CTRL_L_ENDIAN | GLB_CTRL_DEC_GO | val, reg + GLB_CTRL);
	writel(MXC_DEC_EXIT_IDLE_MODE, reg + CAST_STATUS13);
}

void mxc_jpeg_go_auto(void __iomem *reg)
{
	u32 val;

	/* Automatically start the CAST encoder/decoder */
	val = readl(reg + STM_CTRL);
	writel(STM_CTRL_AUTO_START(1) | val, reg + STM_CTRL);

	val = readl(reg + GLB_CTRL);
	writel(GLB_CTRL_L_ENDIAN | GLB_CTRL_DEC_GO | val, reg + GLB_CTRL);
}

int mxc_jpeg_get_slot(void __iomem *reg)
{
	int slot_val;
	int i = 0;
	int tmp = GLB_CTRL_SLOT_EN(0);

	/* currently enabled slots */
	slot_val = readl(reg + GLB_CTRL) & 0xF0;

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
	writel(GLB_CTRL_SLOT_EN(slot) | regval, reg + GLB_CTRL);
}

void mxc_jpeg_set_config_mode(void __iomem *reg, int config_mode)
{
	u32 regval;

	regval = readl(reg + STM_CTRL);
	regval &= ~STM_CTRL_CONFIG_MOD(1);
	writel(STM_CTRL_CONFIG_MOD(config_mode) | regval, reg + STM_CTRL);
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


void mxc_jpeg_set_line_pitch(struct mxc_jpeg_desc *desc, u32 line_pitch)
{
	desc->line_pitch = line_pitch;
}

void mxc_jpeg_set_desc(u32 desc, void __iomem *reg, int slot)
{
	writel(desc | MXC_NXT_DESCPT_EN,
	       reg + MXC_SLOT_OFFSET(slot, SLOT_NXT_DESCPT_PTR));
}

void mxc_jpeg_set_regs_from_desc(struct mxc_jpeg_desc *desc, void __iomem *reg)
{
	writel(desc->buf_base0, reg + BUF_BASE0);
	writel(desc->buf_base1, reg + BUF_BASE1);
	writel(desc->line_pitch, reg + LINE_PITCH);
	writel(desc->stm_bufbase, reg + STM_BUFBASE);
	writel(desc->stm_bufsize, reg + STM_BUFSIZE);
	writel(desc->imgsize, reg + IMGSIZE);
	writel(desc->stm_ctrl, reg + STM_CTRL);
}
