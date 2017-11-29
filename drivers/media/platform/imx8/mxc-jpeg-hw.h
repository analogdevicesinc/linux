/*
 * Copyright 2017 NXP
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
#define COM_STATUS			0x4
#define OUT_BUFFER0			0x14
#define OUT_BUFFER1			0x18
#define OUT_PITCH			0x1C
#define STM_BUFBASE			0x20
#define STM_BUFSIZE			0x24
#define IMG_SIZE			0x28
#define STM_CTRL			0x2C
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
#define SLOT_BASE			0x10000
#define SLOT_STATUS			0x0
#define SLOT_IRQ_EN			0x4
#define SLOT_BUF_PTR			0x8
#define SLOT_CUR_DESCPT_PTR		0xC
#define SLOT_NXT_DESCPT_PTR		0x10
#define MXC_SLOT_OFFSET(slot, offset)	((SLOT_BASE * (slot + 1)) + offset)

#define MXC_ENABLE_DEC			(0x1)
#define MXC_RESET_DEC			(0x1 << 1)
#define MXC_DEC_GO			(0x1 << 2)
#define MXC_ENDIAN_MD			(0x1 << 3)
#define MXC_SLOT_EN			(0x1 << 4)
#define MXC_CONFIG_MOD			(0x1 << 9)

#include "mxc-jpeg.h"
void print_descriptor_info(struct mxc_jpeg_desc *desc);
void mxc_jpeg_reset(void __iomem *reg);
int mxc_jpeg_enable(void __iomem *reg);
void mxc_jpeg_enc_config(void __iomem *reg, struct mxc_jpeg_desc *cfg_desc,
			 u32 cfg_handle, u32 tbl_handle, u32 jpg_handle);
void mxc_jpeg_go(void __iomem *reg);
int mxc_jpeg_get_slot(void __iomem *reg);
u32 mxc_jpeg_get_offset(void __iomem *reg, int slot);
void mxc_jpeg_enable_slot(void __iomem *reg, int slot);
void mxc_jpeg_enable_irq(void __iomem *reg, int slot);
int mxc_jpeg_set_input(void __iomem *reg, u32 in_buf, u32 bufsize);
int mxc_jpeg_set_output(void __iomem *reg, u16 out_pitch, u32 out_buf,
			u16 w, u16 h);
void mxc_jpeg_set_addrs(struct mxc_jpeg_desc *desc, u32 src_addr, u32 dst_addr);
int mxc_jpeg_set_params(struct mxc_jpeg_desc *desc,  u32 bufsize, u16
			 out_pitch, u32 format);
void mxc_jpeg_set_res(struct mxc_jpeg_desc *desc, u16 w, u16 h);
void mxc_jpeg_set_desc(u32 desc, void __iomem *reg, int slot);
void print_cast_decoder_info(void __iomem *reg);
#endif
