/*
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#ifndef _UAPI_PXP_DMA
#define _UAPI_PXP_DMA

#include <linux/posix_types.h>
#include <linux/types.h>

#ifndef __KERNEL__
typedef unsigned long dma_addr_t;
typedef unsigned char bool;
#endif

/*  PXP Pixel format definitions */
/*  Four-character-code (FOURCC) */
#define fourcc(a, b, c, d)\
	(((__u32)(a)<<0)|((__u32)(b)<<8)|((__u32)(c)<<16)|((__u32)(d)<<24))

/*!
 * @name PXP Pixel Formats
 *
 * Pixel formats are defined with ASCII FOURCC code. The pixel format codes are
 * the same used by V4L2 API.
 */

/*! @} */
/*! @name RGB Formats */
/*! @{ */
#define PXP_PIX_FMT_RGB332  fourcc('R', 'G', 'B', '1')	/*!<  8  RGB-3-3-2    */
#define PXP_PIX_FMT_RGB444  fourcc('R', '4', '4', '4')	/* 16  xxxxrrrr ggggbbbb */
#define PXP_PIX_FMT_ARGB444 fourcc('A', 'R', '1', '2')	/* 16  aaaarrrr ggggbbbb */
#define PXP_PIX_FMT_RGBA444 fourcc('R', 'A', '1', '2')	/* 16  rrrrgggg bbbbaaaa */
#define PXP_PIX_FMT_XRGB444 fourcc('X', 'R', '1', '2')	/* 16  xxxxrrrr ggggbbbb */
#define PXP_PIX_FMT_RGB555  fourcc('R', 'G', 'B', 'O')	/*!< 16  RGB-5-5-5    */
#define PXP_PIX_FMT_ARGB555 fourcc('A', 'R', '1', '5')	/*!< 16  ARGB-1-5-5-5 */
#define PXP_PIX_FMT_RGBA555 fourcc('R', 'A', '1', '5')	/*!< 16  RGBA-5-5-5-1 */
#define PXP_PIX_FMT_XRGB555 fourcc('X', 'R', '1', '5')	/*!< 16  XRGB-1-5-5-5 */
#define PXP_PIX_FMT_RGB565  fourcc('R', 'G', 'B', 'P')	/*!< 16  RGB-5-6-5    */
#define PXP_PIX_FMT_BGR565  fourcc('B', 'G', 'R', 'P')	/*!< 16  BGR-5-6-5    */
#define PXP_PIX_FMT_RGB666  fourcc('R', 'G', 'B', '6')	/*!< 18  RGB-6-6-6    */
#define PXP_PIX_FMT_BGR666  fourcc('B', 'G', 'R', '6')	/*!< 18  BGR-6-6-6    */
#define PXP_PIX_FMT_BGR24   fourcc('B', 'G', 'R', '3')	/*!< 24  BGR-8-8-8    */
#define PXP_PIX_FMT_RGB24   fourcc('R', 'G', 'B', '3')	/*!< 24  RGB-8-8-8    */
#define PXP_PIX_FMT_XBGR32  fourcc('X', 'B', 'G', 'R')	/*!< 32  XBGR-8-8-8-8 */
#define PXP_PIX_FMT_BGRX32  fourcc('B', 'G', 'R', 'X')	/*!< 32  BGRX-8-8-8-8 */
#define PXP_PIX_FMT_BGRA32  fourcc('B', 'G', 'R', 'A')	/*!< 32  BGRA-8-8-8-8 */
#define PXP_PIX_FMT_XRGB32  fourcc('X', 'R', 'G', 'B')	/*!< 32  XRGB-8-8-8-8 */
#define PXP_PIX_FMT_RGBX32  fourcc('R', 'G', 'B', 'X')	/*!< 32  RGBX-8-8-8-8 */
#define PXP_PIX_FMT_ARGB32  fourcc('A', 'R', 'G', 'B')	/*!< 32  ARGB-8-8-8-8 */
#define PXP_PIX_FMT_RGBA32  fourcc('R', 'G', 'B', 'A')	/*!< 32  RGBA-8-8-8-8 */
#define PXP_PIX_FMT_ABGR32  fourcc('A', 'B', 'G', 'R')	/*!< 32  ABGR-8-8-8-8 */
#define PXP_PIX_FMT_RGB32	PXP_PIX_FMT_XRGB32
/*! @} */
/*! @name YUV Interleaved Formats */
/*! @{ */
#define PXP_PIX_FMT_YUYV    fourcc('Y', 'U', 'Y', 'V')	/*!< 16 YUV 4:2:2 */
#define PXP_PIX_FMT_UYVY    fourcc('U', 'Y', 'V', 'Y')	/*!< 16 YUV 4:2:2 */
#define PXP_PIX_FMT_VYUY    fourcc('V', 'Y', 'U', 'Y')  /*!< 16 YVU 4:2:2 */
#define PXP_PIX_FMT_YVYU    fourcc('Y', 'V', 'Y', 'U')  /*!< 16 YVU 4:2:2 */
#define PXP_PIX_FMT_Y41P    fourcc('Y', '4', '1', 'P')	/*!< 12 YUV 4:1:1 */
#define PXP_PIX_FMT_VUY444  fourcc('V', 'U', 'Y', 'A')	/*!< 32 VUYA 8:8:8 */
#define PXP_PIX_FMT_YUV444  fourcc('A', 'Y', 'U', 'V') /*!< 32 AYUV 8:8:8 */
#define PXP_PIX_FMT_YVU444  fourcc('A', 'Y', 'V', 'U') /*!< 32 AYUV 8:8:8 */
/* two planes -- one Y, one Cb + Cr interleaved  */
#define PXP_PIX_FMT_NV12    fourcc('N', 'V', '1', '2')	/* 12  Y/CbCr 4:2:0  */
#define PXP_PIX_FMT_NV21    fourcc('N', 'V', '2', '1')	/* 12  Y/CbCr 4:2:0  */
#define PXP_PIX_FMT_NV16    fourcc('N', 'V', '1', '6')	/* 12  Y/CbCr 4:2:2  */
#define PXP_PIX_FMT_NV61    fourcc('N', 'V', '6', '1')	/* 12  Y/CbCr 4:2:2  */
/*! @} */
/*! @name YUV Planar Formats */
/*! @{ */
#define PXP_PIX_FMT_GREY    fourcc('G', 'R', 'E', 'Y')	/*!< 8  Greyscale */
#define PXP_PIX_FMT_GY04    fourcc('G', 'Y', '0', '4') /*!< 4  Greyscale */
#define PXP_PIX_FMT_YVU410P fourcc('Y', 'V', 'U', '9')	/*!< 9  YVU 4:1:0 */
#define PXP_PIX_FMT_YUV410P fourcc('Y', 'U', 'V', '9')	/*!< 9  YUV 4:1:0 */
#define PXP_PIX_FMT_YVU420P fourcc('Y', 'V', '1', '2')	/*!< 12 YVU 4:2:0 */
#define PXP_PIX_FMT_YUV420P fourcc('I', '4', '2', '0')	/*!< 12 YUV 4:2:0 */
#define PXP_PIX_FMT_YUV420P2 fourcc('Y', 'U', '1', '2')	/*!< 12 YUV 4:2:0 */
#define PXP_PIX_FMT_YVU422P fourcc('Y', 'V', '1', '6')	/*!< 16 YVU 4:2:2 */
#define PXP_PIX_FMT_YUV422P fourcc('4', '2', '2', 'P')	/*!< 16 YUV 4:2:2 */
/*! @} */

#define PXP_LUT_NONE			0x0
#define PXP_LUT_INVERT			0x1
#define PXP_LUT_BLACK_WHITE		0x2
#define PXP_LUT_USE_CMAP		0x4

/* dithering modes enum */
#define PXP_DITHER_PASS_THROUGH 0
#define PXP_DITHER_FLOYD	1
#define PXP_DITHER_ATKINSON	2
#define PXP_DITHER_ORDERED	3
#define PXP_DITHER_QUANT_ONLY	4

#define NR_PXP_VIRT_CHANNEL	16

#define PXP_IOC_MAGIC  'P'

#define PXP_IOC_GET_CHAN      _IOR(PXP_IOC_MAGIC, 0, struct pxp_mem_desc)
#define PXP_IOC_PUT_CHAN      _IOW(PXP_IOC_MAGIC, 1, struct pxp_mem_desc)
#define PXP_IOC_CONFIG_CHAN   _IOW(PXP_IOC_MAGIC, 2, struct pxp_mem_desc)
#define PXP_IOC_START_CHAN    _IOW(PXP_IOC_MAGIC, 3, struct pxp_mem_desc)
#define PXP_IOC_GET_PHYMEM    _IOWR(PXP_IOC_MAGIC, 4, struct pxp_mem_desc)
#define PXP_IOC_PUT_PHYMEM    _IOW(PXP_IOC_MAGIC, 5, struct pxp_mem_desc)
#define PXP_IOC_WAIT4CMPLT    _IOWR(PXP_IOC_MAGIC, 6, struct pxp_mem_desc)

#define PXP_IOC_FILL_DATA    _IOWR(PXP_IOC_MAGIC, 7, struct pxp_mem_desc)

#define ALPHA_MODE_ROP		0x1
#define ALPHA_MODE_LEGACY	0x2
#define ALPHA_MODE_PORTER_DUFF	0x3

/* Order significant! */
enum pxp_channel_status {
	PXP_CHANNEL_FREE,
	PXP_CHANNEL_INITIALIZED,
	PXP_CHANNEL_READY,
};

enum pxp_working_mode {
	PXP_MODE_LEGACY			= 0x1,
	PXP_MODE_STANDARD		= 0x2,
	PXP_MODE_ADVANCED		= 0x4,
};

enum pxp_buffer_flag {
	PXP_BUF_FLAG_WFE_A_FETCH0	= 0x0001,
	PXP_BUF_FLAG_WFE_A_FETCH1	= 0x0002,
	PXP_BUF_FLAG_WFE_A_STORE0	= 0x0004,
	PXP_BUF_FLAG_WFE_A_STORE1	= 0x0008,
	PXP_BUF_FLAG_WFE_B_FETCH0	= 0x0010,
	PXP_BUF_FLAG_WFE_B_FETCH1	= 0x0020,
	PXP_BUF_FLAG_WFE_B_STORE0	= 0x0040,
	PXP_BUF_FLAG_WFE_B_STORE1	= 0x0080,
	PXP_BUF_FLAG_DITHER_FETCH0	= 0x0100,
	PXP_BUF_FLAG_DITHER_FETCH1	= 0x0200,
	PXP_BUF_FLAG_DITHER_STORE0	= 0x0400,
	PXP_BUF_FLAG_DITHER_STORE1	= 0x0800,
};

enum pxp_engine_ctrl {
	PXP_ENABLE_ROTATE0		= 0x001,
	PXP_ENABLE_ROTATE1		= 0x002,
	PXP_ENABLE_LUT			= 0x004,
	PXP_ENABLE_CSC2			= 0x008,
	PXP_ENABLE_ALPHA_B		= 0x010,
	PXP_ENABLE_INPUT_FETCH_SOTRE	= 0x020,
	PXP_ENABLE_WFE_B		= 0x040,
	PXP_ENABLE_WFE_A		= 0x080,
	PXP_ENABLE_DITHER		= 0x100,
	PXP_ENABLE_PS_AS_OUT		= 0x200,
	PXP_ENABLE_COLLISION_DETECT     = 0x400,
	PXP_ENABLE_HANDSHAKE		= 0x1000,
	PXP_ENABLE_DITHER_BYPASS	= 0x2000,
};

enum pxp_op_type {
	PXP_OP_2D			= 0x001,
	PXP_OP_DITHER			= 0x002,
	PXP_OP_WFE_A			= 0x004,
	PXP_OP_WFE_B			= 0x008,
};

struct rect {
	int top;		/* Upper left coordinate of rectangle */
	int left;
	int width;
	int height;
};

#define ALPHA_MODE_STRAIGHT	0x0
#define ALPHA_MODE_INVERSED	0x1

#define GLOBAL_ALPHA_MODE_ON	0x0
#define GLOBAL_ALPHA_MODE_OFF	0x1
#define GLOBAL_ALPHA_MODE_SCALE	0x2

#define FACTOR_MODE_ONE		0x0
#define FACTOR_MODE_ZERO	0x1
#define FACTOR_MODE_STRAIGHT	0x2
#define FACTOR_MODE_INVERSED	0x3

#define COLOR_MODE_STRAIGHT	0x0
#define COLOR_MODE_MULTIPLY	0x1

struct pxp_alpha {
	unsigned int alpha_mode;
	unsigned int global_alpha_mode;
	unsigned int global_alpha_value;
	unsigned int factor_mode;
	unsigned int color_mode;
};

struct pxp_layer_param {
	unsigned short left;
	unsigned short top;
	unsigned short width;
	unsigned short height;
	unsigned short stride; /* aka pitch */
	unsigned int pixel_fmt;

	unsigned int flag;
	/* layers combining parameters
	 * (these are ignored for S0 and output
	 * layers, and only apply for OL layer)
	 */
	bool combine_enable;
	unsigned int color_key_enable;
	unsigned int color_key;
	bool global_alpha_enable;
	/* global alpha is either override or multiply */
	bool global_override;
	unsigned char global_alpha;
	bool alpha_invert;
	bool local_alpha_enable;
	int comp_mask;

	struct pxp_alpha alpha;
	struct rect crop;

	dma_addr_t paddr;
};

struct pxp_collision_info {
	unsigned int pixel_cnt;
	unsigned int rect_min_x;
	unsigned int rect_min_y;
	unsigned int rect_max_x;
	unsigned int rect_max_y;
	unsigned int victim_luts[2];
};

struct pxp_proc_data {
	/* S0 Transformation Info */
	int scaling;
	int hflip;
	int vflip;
	int rotate;
	int rot_pos;
	int yuv;
	unsigned int alpha_mode;

	/* Source rectangle (srect) defines the sub-rectangle
	 * within S0 to undergo processing.
	 */
	struct rect srect;
	/* Dest rect (drect) defines how to position the processed
	 * source rectangle (after resizing) within the output frame,
	 * whose dimensions are defined in pxp->pxp_conf_state.out_param
	 */
	struct rect drect;

	/* Current S0 configuration */
	unsigned int bgcolor;
	unsigned char fill_en;

	/* Output overlay support */
	int overlay_state;

	/* LUT transformation on Y data */
	int lut_transform;
	unsigned char *lut_map; /* 256 entries */
	bool lut_map_updated; /* Map recently changed */
	bool combine_enable;

	enum pxp_op_type op_type;

	/* LUT cleanup */
	__u64 lut_sels;

	/* the mode pxp's working against */
	enum pxp_working_mode working_mode;
	enum pxp_engine_ctrl engine_enable;

	/* wfe */
/*
 * partial:
 *         0 - full update
 *         1 - partial update
 * alpha_en:
 *         0 - upd is {Y4[3:0],4'b0000} format
 *         1 - upd is {Y4[3:0],3'b000,alpha} format
 * reagl_en:
 *         0 - use normal waveform algorithm
 *         1 - enable reagl/-d waveform algorithm
 * detection_only:
 *         0 - write working buffer
 *         1 - do no write working buffer, detection only
 * lut:
 *         valid value 0-63
 *         set to the lut used for next update
 */
	bool partial_update;
	bool alpha_en;
	bool lut_update;
	bool reagl_en;		/* enable reagl/-d */
	bool reagl_d_en;	/* enable reagl or reagl-d */
	bool detection_only;
	int lut;
	bool lut_cleanup;
	unsigned int lut_status_1;
	unsigned int lut_status_2;

	/* Dithering specific data */
	int dither_mode;
	unsigned int quant_bit;
};

struct pxp_config_data {
	struct pxp_layer_param s0_param;
	struct pxp_layer_param ol_param[1];
	struct pxp_layer_param out_param;
	struct pxp_layer_param wfe_a_fetch_param[2];
	struct pxp_layer_param wfe_a_store_param[2];
	struct pxp_layer_param wfe_b_fetch_param[2];
	struct pxp_layer_param wfe_b_store_param[2];
	struct pxp_layer_param dither_fetch_param[2];
	struct pxp_layer_param dither_store_param[2];
	struct pxp_proc_data proc_data;
	int layer_nr;

	/* Users don't touch */
	int handle;
};

#endif
