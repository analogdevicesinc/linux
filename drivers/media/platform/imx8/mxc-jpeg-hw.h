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
#ifndef _MXC_JPEG_HW_H
#define _MXC_JPEG_HW_H

/* JPEG Decoder/Encoder Wrapper Register Map */
#define GLB_CTRL			0x0
#define COM_STATUS			0x4
#define BUF_BASE0			0x14
#define BUF_BASE1			0x18
#define LINE_PITCH			0x1C
#define STM_BUFBASE			0x20
#define STM_BUFSIZE			0x24
#define IMGSIZE				0x28
#define STM_CTRL			0x2C

/* CAST JPEG-Decoder Register Map */
#define CAST_STATUS0			0x100
#define CAST_STATUS1			0x104
#define CAST_STATUS2			0x108
#define CAST_STATUS3			0x10c
#define CAST_STATUS4			0x110
#define CAST_STATUS5			0x114
#define CAST_STATUS6			0x118
#define CAST_STATUS7			0x11c
#define CAST_STATUS8			0x120
#define CAST_STATUS9			0x124
#define CAST_STATUS10			0x128
#define CAST_STATUS11			0x12c
#define CAST_STATUS12			0x130
#define CAST_STATUS13			0x134

/* CAST JPEG-Encoder Register Map */
#define CAST_MODE			0x100
#define CAST_CFG_MODE			0x104
#define CAST_QUALITY			0x108
#define CAST_RSVD			0x10c
#define CAST_REC_REGS_SEL		0x110
#define CAST_LUMTH			0x114
#define CAST_CHRTH			0x118
#define CAST_NOMFRSIZE_LO		0x11c
#define CAST_NOMFRSIZE_HI		0x120
#define CAST_OFBSIZE_LO			0x124
#define CAST_OFBSIZE_HI			0x128
/* TODO add more if necessary*/

#define MXC_MAX_SLOTS	1 /* TODO use all 4 slots*/
/* JPEG-Decoder Wrapper Slot Registers 0..3 */
#define SLOT_BASE			0x10000
#define SLOT_STATUS			0x0
#define SLOT_IRQ_EN			0x4
#define SLOT_BUF_PTR			0x8
#define SLOT_CUR_DESCPT_PTR		0xC
#define SLOT_NXT_DESCPT_PTR		0x10
#define MXC_SLOT_OFFSET(slot, offset)	((SLOT_BASE * (slot + 1)) + offset)

/* GLB_CTRL fields */
#define GLB_CTRL_JPG_EN					0x1
#define GLB_CTRL_SFT_RST				(0x1 << 1)
#define GLB_CTRL_DEC_GO					(0x1 << 2)
#define GLB_CTRL_L_ENDIAN				(0x1 << 3)
#define GLB_CTRL_SLOT_EN(slot)				(0x1 << (slot + 4))

/* STM_CTRL fields */
#define STM_CTRL_PIXEL_PRECISION		(0x1 << 2)
#define STM_CTRL_IMAGE_FORMAT(img_fmt)		((img_fmt) << 3)
#define STM_CTRL_BITBUF_PTR_CLR(clr)		((clr) << 7)
#define STM_CTRL_AUTO_START(go)			((go) << 8)
#define STM_CTRL_CONFIG_MOD(mod)		((mod) << 9)

/* SLOTa_STATUS fields TBD */
#define SLOTa_STATUS_FRMDONE			(0x1 << 3)
#define SLOTa_STATUS_ENC_CONFIG_ERR		(0x1 << 8)

/* SLOTa_IRQ_EN fields TBD */

#define MXC_NXT_DESCPT_EN			0x1
#define MXC_DEC_EXIT_IDLE_MODE			0x4

/* JPEG-Decoder Wrapper - STM_CTRL Register Fields */
#define MXC_PIXEL_PRECISION(precision) ((precision)/8 << 2)
enum mxc_jpeg_image_format {
	MXC_JPEG_YUV420 = 0x0, /* 2 Plannar, Y=1st plane UV=2nd plane */
	MXC_JPEG_YUV422 = 0x1, /* 1 Plannar, YUYV sequence */
	MXC_JPEG_RGB	= 0x2, /* RGBRGB packed format */
	MXC_JPEG_YUV444	= 0x3, /* 1 Plannar, YUVYUV sequence */
	MXC_JPEG_GRAY = 0x4, /* Y8 or Y12 or Single Component */
	MXC_JPEG_RESERVED = 0x5,
	MXC_JPEG_ARGB	= 0x6,
};


#include "mxc-jpeg.h"
void print_descriptor_info(struct device *dev, struct mxc_jpeg_desc *desc);
void print_cast_decoder_info(struct device *dev, void __iomem *reg);
void print_cast_encoder_info(struct device *dev, void __iomem *reg);
void print_wrapper_info(struct device *dev, void __iomem *reg);
void mxc_jpeg_reset(void __iomem *reg);
void mxc_jpeg_sw_reset(void __iomem *reg);
int mxc_jpeg_enable(void __iomem *reg);
void wait_frmdone(struct device *dev, void __iomem *reg);
void mxc_jpeg_enc_config(struct device *dev,
			 void __iomem *reg, struct mxc_jpeg_desc *cfg_desc,
			 u32 cfg_handle, u32 tbl_handle, u32 jpg_handle);
void mxc_jpeg_go(void __iomem *reg);
void mxc_jpeg_go_auto(void __iomem *reg);
int mxc_jpeg_get_slot(void __iomem *reg);
u32 mxc_jpeg_get_offset(void __iomem *reg, int slot);
void mxc_jpeg_enable_slot(void __iomem *reg, int slot);
void mxc_jpeg_enable_irq(void __iomem *reg, int slot);
int mxc_jpeg_set_input(void __iomem *reg, u32 in_buf, u32 bufsize);
int mxc_jpeg_set_output(void __iomem *reg, u16 out_pitch, u32 out_buf,
			u16 w, u16 h);
void mxc_jpeg_set_config_mode(void __iomem *reg, int config_mode);
int mxc_jpeg_set_params(struct mxc_jpeg_desc *desc,  u32 bufsize, u16
			 out_pitch, u32 format);
void mxc_jpeg_set_bufsize(struct mxc_jpeg_desc *desc,  u32 bufsize);
void mxc_jpeg_set_res(struct mxc_jpeg_desc *desc, u16 w, u16 h);
void mxc_jpeg_set_line_pitch(struct mxc_jpeg_desc *desc, u32 line_pitch);
void mxc_jpeg_set_desc(u32 desc, void __iomem *reg, int slot);
void mxc_jpeg_set_regs_from_desc(struct mxc_jpeg_desc *desc, void __iomem *reg);
#endif
