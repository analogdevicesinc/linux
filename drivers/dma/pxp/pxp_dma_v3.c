/*
 * Copyright (C) 2010-2016 Freescale Semiconductor, Inc.
 *
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
/*
 * Based on STMP378X PxP driver
 * Copyright 2008-2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dmaengine.h>
#include <linux/pxp_dma.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/of.h>

#include "regs-pxp_v3.h"
#include "reg_bitfields.h"

#ifdef CONFIG_MXC_FPGA_M4_TEST
#include "cm4_image.c"
#define FPGA_TCML_ADDR        0x0C7F8000
#define PINCTRL               0x0C018000
#define PIN_DOUT              0x700
void __iomem *fpga_tcml_base;
void __iomem *pinctrl_base;
#endif


#define PXP_FILL_TIMEOUT	3000
#define busy_wait(cond)							\
	({                                                              \
	unsigned long end_jiffies = jiffies + 				\
			msecs_to_jiffies(PXP_FILL_TIMEOUT);         	\
	bool succeeded = false;                                         \
	do {                                                            \
		if (cond) {                                             \
			succeeded = true;                               \
			break;                                          \
		}                                                       \
		cpu_relax();                                            \
	} while (time_after(end_jiffies, jiffies));                   	\
		succeeded;                                              \
	})

#define	PXP_DOWNSCALE_THRESHOLD		0x4000

#define CONFIG_FB_MXC_EINK_FPGA

/* define all the pxp 2d nodes */
#define PXP_2D_PS		0
#define PXP_2D_AS		1
#define PXP_2D_INPUT_FETCH0	2
#define PXP_2D_INPUT_FETCH1	3
#define PXP_2D_CSC1		4
#define PXP_2D_ROTATION1	5
#define PXP_2D_ALPHA0_S0	6
#define PXP_2D_ALPHA0_S1	7
#define PXP_2D_ALPHA1_S0	8
#define PXP_2D_ALPHA1_S1	9
#define PXP_2D_CSC2		10
#define PXP_2D_LUT		11
#define PXP_2D_ROTATION0	12
#define PXP_2D_OUT		13
#define PXP_2D_INPUT_STORE0	14
#define PXP_2D_INPUT_STORE1	15
#define PXP_2D_NUM		16

#define PXP_2D_ALPHA0_S0_S1	0xaa
#define PXP_2D_ALPHA1_S0_S1	0xbb

#define PXP_2D_MUX_BASE		50
#define PXP_2D_MUX_MUX0		(PXP_2D_MUX_BASE + 0)
#define PXP_2D_MUX_MUX1		(PXP_2D_MUX_BASE + 1)
#define PXP_2D_MUX_MUX2		(PXP_2D_MUX_BASE + 2)
#define PXP_2D_MUX_MUX3		(PXP_2D_MUX_BASE + 3)
#define PXP_2D_MUX_MUX4		(PXP_2D_MUX_BASE + 4)
#define PXP_2D_MUX_MUX5		(PXP_2D_MUX_BASE + 5)
#define PXP_2D_MUX_MUX6		(PXP_2D_MUX_BASE + 6)
#define PXP_2D_MUX_MUX7		(PXP_2D_MUX_BASE + 7)
#define PXP_2D_MUX_MUX8		(PXP_2D_MUX_BASE + 8)
#define PXP_2D_MUX_MUX9		(PXP_2D_MUX_BASE + 9)
#define PXP_2D_MUX_MUX10	(PXP_2D_MUX_BASE + 10)
#define PXP_2D_MUX_MUX11	(PXP_2D_MUX_BASE + 11)
#define PXP_2D_MUX_MUX12	(PXP_2D_MUX_BASE + 12)
#define PXP_2D_MUX_MUX13	(PXP_2D_MUX_BASE + 13)
#define PXP_2D_MUX_MUX14	(PXP_2D_MUX_BASE + 14)
#define PXP_2D_MUX_MUX15	(PXP_2D_MUX_BASE + 15)

/* define pxp 2d node types */
#define PXP_2D_TYPE_INPUT	1
#define PXP_2D_TYPE_ALU		2
#define PXP_2D_TYPE_OUTPUT	3

#define DISTANCE_INFINITY	0xffff
#define NO_PATH_NODE		0xffffffff

#define PXP_MAX_INPUT_NUM	2
#define PXP_MAX_OUTPUT_NUM	2

#define FETCH_NOOP		0x01
#define FETCH_EXPAND		0x02
#define FETCH_SHIFT		0x04

#define STORE_NOOP		0x01
#define STORE_SHIFT		0x02
#define STORE_SHRINK		0x04

#define NEED_YUV_SWAP		0x02

#define IN_NEED_COMPOSITE	(0x01 | IN_NEED_FMT_UNIFIED)
#define IN_NEED_CSC		(0x02 | IN_NEED_FMT_UNIFIED)
#define IN_NEED_SCALE		(0x04 | IN_NEED_FMT_UNIFIED)
#define IN_NEED_ROTATE_FLIP	(0x08 | IN_NEED_FMT_UNIFIED)
#define IN_NEED_FMT_UNIFIED	0x10
#define IN_NEED_SHIFT		0x20
#define IN_NEED_LUT		(0x40 | IN_NEED_UNIFIED)

#define OUT_NEED_SHRINK		0x100
#define OUT_NEED_SHIFT		0x200

#define PXP_ROTATE_0	0
#define PXP_ROTATE_90	1
#define PXP_ROTATE_180	2
#define PXP_ROTATE_270	3

#define PXP_H_FLIP     1
#define PXP_V_FLIP     2

#define PXP_OP_TYPE_2D		0x001
#define PXP_OP_TYPE_DITHER	0x002
#define PXP_OP_TYPE_WFE_A	0x004
#define PXP_OP_TYPE_WFE_B	0x008

/* define store engine output mode */
#define STORE_MODE_NORMAL	1
#define STORE_MODE_BYPASS	2
#define STORE_MODE_DUAL		3
#define STORE_MODE_HANDSHAKE	4

/* define fetch engine input mode */
#define FETCH_MODE_NORMAL	1
#define FETCH_MODE_BYPASS	2
#define FETCH_MODE_HANDSHAKE	3

#define COMMON_FMT_BPP		32

#define R_COMP		0
#define G_COMP		1
#define B_COMP		2
#define A_COMP		3

#define Y_COMP		0
#define U_COMP		1
#define V_COMP		2
#define Y1_COMP		4

static LIST_HEAD(head);
static int timeout_in_ms = 600;
static unsigned int block_size;
static struct kmem_cache *tx_desc_cache;
static struct kmem_cache *edge_node_cache;
static struct pxp_collision_info col_info;
static dma_addr_t paddr;
static bool v3p_flag;
static int alpha_blending_version;
static bool pxp_legacy;

struct pxp_dma {
	struct dma_device dma;
};

enum pxp_alpha_blending_version {
	PXP_ALPHA_BLENDING_NONE	 = 0x0,
	PXP_ALPHA_BLENDING_V1	 = 0x1,
	PXP_ALPHA_BLENDING_V2	 = 0x2,
};

struct pxp_alpha_global {
	unsigned int color_key_enable;
	bool combine_enable;
	bool global_alpha_enable;
	bool global_override;
	bool alpha_invert;
	bool local_alpha_enable;
	unsigned char global_alpha;
	int comp_mask;
};

struct rectangle {
	uint16_t x;
	uint16_t y;
	uint16_t width;
	uint16_t height;
};

struct pxp_alpha_info {
	uint8_t alpha_mode;
	uint8_t rop_type;

	struct pxp_alpha s0_alpha;
	struct pxp_alpha s1_alpha;
};

struct pxp_op_info{
	uint16_t op_type;
	uint16_t rotation;
	uint8_t  flip;
	uint8_t  fill_en;
	uint32_t fill_data;
	uint8_t  alpha_blending;
	struct pxp_alpha_info alpha_info;

	/* Dithering specific data */
	uint32_t dither_mode;
	uint32_t quant_bit;

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
	uint32_t lut_status_1;
	uint32_t lut_status_2;
};

struct pxp_pixmap {
	uint8_t channel_id;
	uint8_t bpp;
	int32_t pitch;
	uint16_t width;
	uint16_t height;
	struct rectangle crop;
	uint32_t rotate;
	uint8_t flip;
	uint32_t format;	/* fourcc pixmap format */
	uint32_t flags;
	bool valid;
	dma_addr_t paddr;
	struct pxp_alpha_global g_alpha;
};

struct pxp_task_info {
	uint8_t input_num;
	uint8_t output_num;
	struct pxp_pixmap input[PXP_MAX_INPUT_NUM];
	struct pxp_pixmap output[PXP_MAX_OUTPUT_NUM];
	struct pxp_op_info op_info;
	uint32_t pxp_2d_flags;
};

struct pxps {
	struct platform_device *pdev;
	struct clk *ipg_clk;
	struct clk *axi_clk;
	void __iomem *base;
	int irq;		/* PXP IRQ to the CPU */

	spinlock_t lock;
	struct mutex clk_mutex;
	int clk_stat;
#define	CLK_STAT_OFF		0
#define	CLK_STAT_ON		1
	int pxp_ongoing;
	int lut_state;

	struct device *dev;
	struct pxp_dma pxp_dma;
	struct pxp_channel channel[NR_PXP_VIRT_CHANNEL];
	struct work_struct work;

	const struct pxp_devdata *devdata;
	struct pxp_task_info task;

	/* describes most recent processing configuration */
	struct pxp_config_data pxp_conf_state;

	/* to turn clock off when pxp is inactive */
	struct timer_list clk_timer;

	/* for pxp config dispatch asynchronously*/
	struct task_struct *dispatch;
	wait_queue_head_t thread_waitq;
	struct completion complete;
};

#define to_pxp_dma(d) container_of(d, struct pxp_dma, dma)
#define to_tx_desc(tx) container_of(tx, struct pxp_tx_desc, txd)
#define to_pxp_channel(d) container_of(d, struct pxp_channel, dma_chan)
#define to_pxp(id) container_of(id, struct pxps, pxp_dma)

#define to_pxp_task_info(op) container_of((op), struct pxp_task_info, op_info)
#define to_pxp_from_task(task) container_of((task), struct pxps, task)

#define PXP_DEF_BUFS	2
#define PXP_MIN_PIX	8

static uint8_t active_bpp(uint8_t bpp)
{
	switch(bpp) {
	case 8:
		return 0x0;
	case 16:
		return 0x1;
	case 32:
		return 0x2;
	case 64:
		return 0x3;
	default:
		return 0xff;
	}
}

static uint8_t rotate_map(uint32_t degree)
{
	switch (degree) {
	case 0:
		return PXP_ROTATE_0;
	case 90:
		return PXP_ROTATE_90;
	case 180:
		return PXP_ROTATE_180;
	case 270:
		return PXP_ROTATE_270;
	default:
		return 0;
	}
}

static uint8_t expand_format(uint32_t format)
{
	switch (format) {
	case PXP_PIX_FMT_RGB565:
	case PXP_PIX_FMT_BGR565:
		return 0x0;
	case PXP_PIX_FMT_RGB555:
		return 0x1;
	case PXP_PIX_FMT_YUYV:
	case PXP_PIX_FMT_YVYU:
		return 0x5;
	case PXP_PIX_FMT_UYVY:
	case PXP_PIX_FMT_VYUY:
		return 0x6;
	case PXP_PIX_FMT_NV16:
		return 0x7;
	default:
		return 0xff;
	}
}

struct color_component {
	uint8_t id;
	uint8_t	offset;
	uint8_t length;
	uint8_t mask;
};

struct color {
	uint32_t format;
	struct color_component comp[4];
};

struct color rgb_colors[] = {
	{
		.format = PXP_PIX_FMT_RGB565,
		.comp = {
				{ .id = B_COMP,	.offset = 0,  .length = 5, .mask = 0x1f, },
				{ .id = G_COMP,	.offset = 5,  .length = 6, .mask = 0x3f, },
				{ .id = R_COMP, .offset = 11, .length = 5, .mask = 0x1f, },
				{ .id = A_COMP,	.offset = 0,  .length = 0, .mask = 0x0,  },
			},
	}, {
		.format  = PXP_PIX_FMT_BGR565,
		.comp = {
				{ .id = R_COMP,	.offset = 0,  .length = 5, .mask = 0x1f, },
				{ .id = G_COMP,	.offset = 5,  .length = 6, .mask = 0x3f, },
				{ .id = B_COMP, .offset = 11, .length = 6, .mask = 0x3f, },
				{ .id = A_COMP,	.offset = 0,  .length = 0, .mask = 0x0,  },
			},
	}, {
		.format = PXP_PIX_FMT_ARGB555,
		.comp = {
				{ .id = B_COMP,	.offset = 0,  .length = 5, .mask = 0x1f, },
				{ .id = G_COMP,	.offset = 5,  .length = 5, .mask = 0x1f, },
				{ .id = R_COMP, .offset = 10, .length = 5, .mask = 0x1f, },
				{ .id = A_COMP,	.offset = 15, .length = 1, .mask = 0x1,  },
			},
	}, {
		.format = PXP_PIX_FMT_XRGB555,
		.comp = {
				{ .id = B_COMP,	.offset = 0,  .length = 5, .mask = 0x1f, },
				{ .id = G_COMP,	.offset = 5,  .length = 5, .mask = 0x1f, },
				{ .id = R_COMP, .offset = 10, .length = 5, .mask = 0x1f, },
				{ .id = A_COMP,	.offset = 15, .length = 1, .mask = 0x1,  },
			},
	}, {
		.format = PXP_PIX_FMT_RGB555,
		.comp = {
				{ .id = B_COMP,	.offset = 0,  .length = 5, .mask = 0x1f, },
				{ .id = G_COMP,	.offset = 5,  .length = 5, .mask = 0x1f, },
				{ .id = R_COMP, .offset = 10, .length = 5, .mask = 0x1f, },
				{ .id = A_COMP,	.offset = 15, .length = 1, .mask = 0x1,  },
			},
	}, {
		.format = PXP_PIX_FMT_RGBA555,
		.comp = {
				{ .id = A_COMP,	.offset = 0,  .length = 1, .mask = 0x1,  },
				{ .id = B_COMP,	.offset = 1,  .length = 5, .mask = 0x1f, },
				{ .id = G_COMP, .offset = 6,  .length = 5, .mask = 0x1f, },
				{ .id = R_COMP,	.offset = 11, .length = 5, .mask = 0x1f, },
			},
	}, {
		.format = PXP_PIX_FMT_ARGB444,
		.comp = {
				{ .id = B_COMP,	.offset = 0,  .length = 4, .mask = 0xf,  },
				{ .id = G_COMP,	.offset = 4,  .length = 4, .mask = 0xf,  },
				{ .id = R_COMP, .offset = 8,  .length = 4, .mask = 0xf,  },
				{ .id = A_COMP,	.offset = 12, .length = 4, .mask = 0xf,  },
			},
	}, {
		.format = PXP_PIX_FMT_XRGB444,
		.comp = {
				{ .id = B_COMP,	.offset = 0,  .length = 4, .mask = 0xf,  },
				{ .id = G_COMP,	.offset = 4,  .length = 4, .mask = 0xf,  },
				{ .id = R_COMP, .offset = 8,  .length = 4, .mask = 0xf,  },
				{ .id = A_COMP,	.offset = 12, .length = 4, .mask = 0xf,  },
			},
	}, {
		.format = PXP_PIX_FMT_RGBA444,
		.comp = {
				{ .id = A_COMP,	.offset = 0,  .length = 4, .mask = 0xf,  },
				{ .id = B_COMP,	.offset = 4,  .length = 4, .mask = 0xf,  },
				{ .id = G_COMP, .offset = 8,  .length = 4, .mask = 0xf,  },
				{ .id = R_COMP,	.offset = 12, .length = 4, .mask = 0xf,  },
			},
	}, {
		.format  = PXP_PIX_FMT_RGB24,
		.comp = {
				{ .id = B_COMP, .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = R_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 0,  .length = 0, .mask = 0x0,  },
			},
	}, {
		.format  = PXP_PIX_FMT_BGR24,
		.comp = {
				{ .id = R_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = B_COMP, .offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 0,  .length = 0, .mask = 0x0,  },
			},
	}, {
		.format  = PXP_PIX_FMT_XRGB32,
		.comp = {
				{ .id = B_COMP, .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = R_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_RGBX32,
		.comp = {
				{ .id = A_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = B_COMP, .offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = R_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_XBGR32,
		.comp = {
				{ .id = R_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = B_COMP, .offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_BGRX32,
		.comp = {
				{ .id = A_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = R_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = B_COMP, .offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_ARGB32,
		.comp = {
				{ .id = B_COMP, .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = R_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_ABGR32,
		.comp = {
				{ .id = R_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = B_COMP, .offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_RGBA32,
		.comp = {
				{ .id = A_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = B_COMP, .offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = R_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_BGRA32,
		.comp = {
				{ .id = A_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = R_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = G_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = B_COMP, .offset = 24, .length = 8, .mask = 0xff, },
			},
	},
};

/* only one plane yuv formats */
struct color yuv_colors[] = {
	{
		.format  = PXP_PIX_FMT_GREY,
		.comp = {
				{ .id = Y_COMP,	 .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = U_COMP,  .offset = 8,  .length = 0, .mask = 0x00, },
				{ .id = V_COMP,	 .offset = 16, .length = 0, .mask = 0x00, },
				{ .id = A_COMP,	 .offset = 24, .length = 0, .mask = 0x00, },
			},
	}, {
		.format  = PXP_PIX_FMT_YUYV,
		.comp = {
				{ .id = V_COMP,	 .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = Y1_COMP, .offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = U_COMP,	 .offset = 16, .length = 8, .mask = 0xff, },
				{ .id = Y_COMP,	 .offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_UYVY,
		.comp = {
				{ .id = Y1_COMP, .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = V_COMP,  .offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = Y_COMP,	 .offset = 16, .length = 8, .mask = 0xff, },
				{ .id = U_COMP,	 .offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_YVYU,
		.comp = {
				{ .id = U_COMP,	 .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = Y1_COMP, .offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = V_COMP,	 .offset = 16, .length = 8, .mask = 0xff, },
				{ .id = Y_COMP,	 .offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_VYUY,
		.comp = {
				{ .id = Y1_COMP, .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = U_COMP,  .offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = Y_COMP,	 .offset = 16, .length = 8, .mask = 0xff, },
				{ .id = V_COMP,	 .offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_YUV444,
		.comp = {
				{ .id = V_COMP, .offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = U_COMP,	.offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = Y_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	}, {
		.format  = PXP_PIX_FMT_YVU444,
		.comp = {
				{ .id = U_COMP,	.offset = 0,  .length = 8, .mask = 0xff, },
				{ .id = V_COMP, .offset = 8,  .length = 8, .mask = 0xff, },
				{ .id = Y_COMP,	.offset = 16, .length = 8, .mask = 0xff, },
				{ .id = A_COMP,	.offset = 24, .length = 8, .mask = 0xff, },
			},
	},
};

/* 4 to 1 mux */
struct mux {
	uint32_t id;
	uint8_t mux_inputs[4];
	uint8_t mux_outputs[2];
};

/* Adjacent list structure */
struct edge_node {
	uint32_t adjvex;
	uint32_t prev_vnode;
	struct edge_node *next;
	uint32_t mux_used;
	struct mux_config muxes;
};

struct vetex_node {
	uint8_t type;
	struct edge_node *first;
};

struct path_node {
	struct list_head node;
	uint32_t id;
	uint32_t distance;
	uint32_t prev_node;
};

static struct vetex_node adj_list[PXP_2D_NUM];
static struct path_node path_table[PXP_2D_NUM][PXP_2D_NUM];

static bool adj_array_v3[PXP_2D_NUM][PXP_2D_NUM] = {
      /* 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 */
	{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0  */
	{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, /* 1  */
	{0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0}, /* 2  */
	{0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1}, /* 3  */
	{0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 4  */
	{0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0}, /* 5  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0}, /* 6  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0}, /* 7  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0}, /* 8  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0}, /* 9  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0}, /* 10 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0}, /* 11 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}, /* 12 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 13 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 14 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 15 */
};


static struct mux muxes_v3[16] = {
	{
		/* mux0 */
		.id = 0,
		.mux_inputs = {PXP_2D_CSC1, PXP_2D_INPUT_FETCH0, PXP_2D_INPUT_FETCH1, 0xff},
		.mux_outputs = {PXP_2D_ROTATION1, 0xff},
	}, {
		/* mux1 */
		.id = 1,
		.mux_inputs = {PXP_2D_INPUT_FETCH0, PXP_2D_ROTATION1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_ALPHA1_S1, PXP_2D_MUX_MUX5},
	}, {
		/* mux2 */
		.id = 2,
		.mux_inputs = {PXP_2D_INPUT_FETCH1, PXP_2D_ROTATION1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_ALPHA1_S0, 0xff},
	}, {
		/* mux3 */
		.id = 3,
		.mux_inputs = {PXP_2D_CSC1, PXP_2D_ROTATION1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_ALPHA0_S0, 0xff},
	}, {
		/* mux4 is not used in ULT1 */
		.id = 4,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux5 */
		.id = 5,
		.mux_inputs = {PXP_2D_MUX_MUX1, PXP_2D_ALPHA1_S0_S1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX7, 0xff},
	}, {
		/* mux6 */
		.id = 6,
		.mux_inputs = {PXP_2D_ALPHA1_S0_S1, PXP_2D_ALPHA0_S0_S1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_CSC2, 0xff},
	}, {
		/* mux7 */
		.id = 7,
		.mux_inputs = {PXP_2D_MUX_MUX5, PXP_2D_CSC2, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX9, PXP_2D_MUX_MUX10},
	}, {
		/* mux8 */
		.id = 8,
		.mux_inputs = {PXP_2D_CSC2, PXP_2D_ALPHA0_S0_S1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX9, PXP_2D_MUX_MUX11},
	}, {
		/* mux9 */
		.id = 9,
		.mux_inputs = {PXP_2D_MUX_MUX7, PXP_2D_MUX_MUX8, 0xff, 0xff},
		.mux_outputs = {PXP_2D_LUT, 0xff},
	}, {
		/* mux10 */
		.id = 10,
		.mux_inputs = {PXP_2D_MUX_MUX7, PXP_2D_LUT, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX12, PXP_2D_MUX_MUX15},
	}, {
		/* mux11 */
		.id = 11,
		.mux_inputs = {PXP_2D_LUT, PXP_2D_MUX_MUX8, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX12, PXP_2D_MUX_MUX14},
	}, {
		/* mux12 */
		.id = 12,
		.mux_inputs = {PXP_2D_MUX_MUX10, PXP_2D_MUX_MUX11, 0xff, 0xff},
		.mux_outputs = {PXP_2D_ROTATION0, 0xff},
	}, {
		/* mux13 */
		.id = 13,
		.mux_inputs = {PXP_2D_INPUT_FETCH1, 0xff, 0xff, 0xff},
		.mux_outputs = {PXP_2D_INPUT_STORE1, 0xff},
	}, {
		/* mux14 */
		.id = 14,
		.mux_inputs = {PXP_2D_ROTATION0, PXP_2D_MUX_MUX11, 0xff, 0xff},
		.mux_outputs = {PXP_2D_OUT, 0xff},
	}, {
		/* mux15 */
		.id = 15,
		.mux_inputs = {PXP_2D_INPUT_FETCH0, PXP_2D_MUX_MUX10, 0xff, 0xff},
		.mux_outputs = {PXP_2D_INPUT_STORE0, 0xff},
	},
};

static bool adj_array_v3p[PXP_2D_NUM][PXP_2D_NUM] = {
      /* 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 */
	{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0  */
	{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, /* 1  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 2  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 3  */
	{0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 4  */
	{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 5  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0}, /* 6  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0}, /* 7  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 8  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 9  */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0}, /* 10 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}, /* 11 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}, /* 12 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 13 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 14 */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, /* 15 */
};

static struct mux muxes_v3p[16] = {
	{
		/* mux0 */
		.id = 0,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux1 */
		.id = 1,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux2 */
		.id = 2,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux3 */
		.id = 3,
		.mux_inputs = {PXP_2D_CSC1, PXP_2D_ROTATION1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_ALPHA0_S0, 0xff},
	}, {
		/* mux4 is not used in ULT1 */
		.id = 4,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux5 */
		.id = 5,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux6 */
		.id = 6,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	},  {
		/* mux7 */
		.id = 7,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux8 */
		.id = 8,
		.mux_inputs = {PXP_2D_CSC2, PXP_2D_ALPHA0_S0_S1, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX9, PXP_2D_MUX_MUX11},
	}, {
		/* mux9 */
		.id = 9,
		.mux_inputs = {0xff, PXP_2D_MUX_MUX8, 0xff, 0xff},
		.mux_outputs = {PXP_2D_LUT, 0xff},
	}, {
		/* mux10 */
		.id = 10,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux11 */
		.id = 11,
		.mux_inputs = {PXP_2D_LUT, PXP_2D_MUX_MUX8, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX12, PXP_2D_ROTATION0},
	}, {
		/* mux12 */
		.id = 12,
		.mux_inputs = {PXP_2D_ROTATION0, PXP_2D_MUX_MUX11, 0xff, 0xff},
		.mux_outputs = {PXP_2D_MUX_MUX14, 0xff},
	}, {
		/* mux13 */
		.id = 13,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	}, {
		/* mux14 */
		.id = 14,
		.mux_inputs = {0xff, PXP_2D_MUX_MUX12, 0xff, 0xff},
		.mux_outputs = {PXP_2D_OUT, 0xff},
	}, {
		/* mux15 */
		.id = 15,
		.mux_inputs = {0xff, 0xff, 0xff, 0xff},
		.mux_outputs = {0xff, 0xff},
	},
};

static void __iomem *pxp_reg_base;

#define pxp_writel(val, reg) writel(val, pxp_reg_base + (reg))

static __attribute__((aligned (1024*4))) unsigned int active_matrix_data_8x8[64]={
   0x06050100, 0x04030207, 0x06050100, 0x04030207,
   0x00040302, 0x07060501, 0x00040302, 0x07060501,
   0x02070605, 0x01000403, 0x02070605, 0x01000403,
   0x05010004, 0x03020706, 0x05010004, 0x03020706,
   0x04030207, 0x06050100, 0x04030207, 0x06050100,
   0x07060501, 0x00040302, 0x07060501, 0x00040302,
   0x01000403, 0x02070605, 0x01000403, 0x02070605,
   0x03020706, 0x05010004, 0x03020706, 0x05010004,
   0x06050100, 0x04030207, 0x06050100, 0x04030207,
   0x00040302, 0x07060501, 0x00040302, 0x07060501,
   0x02070605, 0x01000403, 0x02070605, 0x01000403,
   0x05010004, 0x03020706, 0x05010004, 0x03020706,
   0x04030207, 0x06050100, 0x04030207, 0x06050100,
   0x07060501, 0x00040302, 0x07060501, 0x00040302,
   0x01000403, 0x02070605, 0x01000403, 0x02070605,
   0x03020706, 0x05010004, 0x03020706, 0x05010004
    };

static __attribute__((aligned (1024*4))) unsigned int bit1_dither_data_8x8[64]={

	1,       49*2,    13*2,    61*2,    4*2,     52*2,    16*2,    64*2,
	33*2,    17*2,    45*2,    29*2,    36*2,    20*2,    48*2,    32*2,
	9*2,     57*2,    5*2,     53*2,    12*2,    60*2,    8*2,     56*2,
	41*2,    25*2,    37*2,    21*2,    44*2,    28*2,    40*2,    24*2,
	3*2,     51*2,    15*2,    63*2,    2*2,     50*2,    14*2,    62*2,
	35*2,    19*2,    47*2,    31*2,    34*2,    18*2,    46*2,    30*2,
	11*2,    59*2,    7*2,     55*2,    10*2,    58*2,    6*2,     54*2,
	43*2,    27*2,    39*2,    23*2,    42*2,    26*2,    38*2,    22*2
};

static __attribute__((aligned (1024*4))) unsigned int bit2_dither_data_8x8[64]={

	1,     49,    13,    61,    4,     52,    16,    64,
	33,    17,    45,    29,    36,    20,    48,    32,
	9,     57,    5,     53,    12,    60,    8,     56,
	41,    25,    37,    21,    44,    28,    40,    24,
	3,     51,    15,    63,    2,     50,    14,    62,
	35,    19,    47,    31,    34,    18,    46,    30,
	11,    59,    7,     55,    10,    58,    6,     54,
	43,    27,    39,    23,    42,    26,    38,    22
};

static __attribute__((aligned (1024*4))) unsigned int bit4_dither_data_8x8[64]={

	1,       49/4,    13/4,    61/4,    4/4,     52/4,    16/4,    64/4,
	33/4,    17/4,    45/4,    29/4,    36/4,    20/4,    48/4,    32/4,
	9/4,     57/4,    5/4,     53/4,    12/4,    60/4,    8/4,     56/4,
	41/4,    25/4,    37/4,    21/4,    44/4,    28/4,    40/4,    24/4,
	3/4,     51/4,    15/4,    63/4,    2/4,     50/4,    14/4,    62/4,
	35/4,    19/4,    47/4,    31/4,    34/4,    18/4,    46/4,    30/4,
	11/4,    59/4,    7/4,     55/4,    10/4,    58/4,    6/4,     54/4,
	43/4,    27/4,    39/4,    23/4,    42/4,    26/4,    38/4,    22/4
};

static void pxp_dithering_configure(struct pxps *pxp);
static void pxp_dithering_configure_v3p(struct pxps *pxp);
static void pxp_dithering_process(struct pxps *pxp);
static void pxp_wfe_a_process(struct pxps *pxp);
static void pxp_wfe_a_process_v3p(struct pxps *pxp);
static void pxp_wfe_a_configure(struct pxps *pxp);
static void pxp_wfe_a_configure_v3p(struct pxps *pxp);
static void pxp_wfe_b_process(struct pxps *pxp);
static void pxp_wfe_b_configure(struct pxps *pxp);
static void pxp_lut_status_set(struct pxps *pxp, unsigned int lut);
static void pxp_lut_status_set_v3p(struct pxps *pxp, unsigned int lut);
static void pxp_lut_status_clr(unsigned int lut);
static void pxp_lut_status_clr_v3p(unsigned int lut);
static void pxp_start2(struct pxps *pxp);
static void pxp_data_path_config_v3p(struct pxps *pxp);
static void pxp_soft_reset(struct pxps *pxp);
static void pxp_collision_detection_disable(struct pxps *pxp);
static void pxp_collision_detection_enable(struct pxps *pxp,
					   unsigned int width,
					   unsigned int height);
static void pxp_luts_activate(struct pxps *pxp, u64 lut_status);
static bool pxp_collision_status_report(struct pxps *pxp, struct pxp_collision_info *info);
static void pxp_histogram_status_report(struct pxps *pxp, u32 *hist_status);
static void pxp_histogram_enable(struct pxps *pxp,
				 unsigned int width,
				 unsigned int height);
static void pxp_histogram_disable(struct pxps *pxp);
static void pxp_lut_cleanup_multiple(struct pxps *pxp, u64 lut, bool set);
static void pxp_lut_cleanup_multiple_v3p(struct pxps *pxp, u64 lut, bool set);
static void pxp_luts_deactivate(struct pxps *pxp, u64 lut_status);
static void pxp_set_colorkey(struct pxps *pxp);

enum {
	DITHER0_LUT = 0x0,	/* Select the LUT memory for access */
	DITHER0_ERR0 = 0x1,	/* Select the ERR0 memory for access */
	DITHER0_ERR1 = 0x2,	/* Select the ERR1 memory for access */
	DITHER1_LUT = 0x3,	/* Select the LUT memory for access */
	DITHER2_LUT = 0x4,	/* Select the LUT memory for access */
	ALU_A = 0x5,		/* Select the ALU instr memory for access */
	ALU_B = 0x6,		/* Select the ALU instr memory for access */
	WFE_A = 0x7,		/* Select the WFE_A instr memory for access */
	WFE_B = 0x8,		/* Select the WFE_B instr memory for access */
	RESERVED = 0x15,
};

enum pxp_devtype {
	PXP_V3,
	PXP_V3P,	/* minor changes over V3, use WFE_B to replace WFE_A */
};

#define pxp_is_v3(pxp) (pxp->devdata->version == 30)
#define pxp_is_v3p(pxp) (pxp->devdata->version == 31)

struct pxp_devdata {
	void (*pxp_wfe_a_configure)(struct pxps *pxp);
	void (*pxp_wfe_a_process)(struct pxps *pxp);
	void (*pxp_lut_status_set)(struct pxps *pxp, unsigned int lut);
	void (*pxp_lut_status_clr)(unsigned int lut);
	void (*pxp_dithering_configure)(struct pxps *pxp);
	void (*pxp_lut_cleanup_multiple)(struct pxps *pxp, u64 lut, bool set);
	void (*pxp_data_path_config)(struct pxps *pxp);
	unsigned int version;
};

static const struct pxp_devdata pxp_devdata[] = {
	[PXP_V3] = {
		.pxp_wfe_a_configure = pxp_wfe_a_configure,
		.pxp_wfe_a_process = pxp_wfe_a_process,
		.pxp_lut_status_set = pxp_lut_status_set,
		.pxp_lut_status_clr = pxp_lut_status_clr,
		.pxp_lut_cleanup_multiple = pxp_lut_cleanup_multiple,
		.pxp_dithering_configure = pxp_dithering_configure,
		.pxp_data_path_config = NULL,
		.version = 30,
	},
	[PXP_V3P] = {
		.pxp_wfe_a_configure = pxp_wfe_a_configure_v3p,
		.pxp_wfe_a_process = pxp_wfe_a_process_v3p,
		.pxp_lut_status_set = pxp_lut_status_set_v3p,
		.pxp_lut_status_clr = pxp_lut_status_clr_v3p,
		.pxp_lut_cleanup_multiple = pxp_lut_cleanup_multiple_v3p,
		.pxp_dithering_configure = pxp_dithering_configure_v3p,
		.pxp_data_path_config = pxp_data_path_config_v3p,
		.version = 31,
	},
};

/*
 * PXP common functions
 */
static void dump_pxp_reg(struct pxps *pxp)
{
	dev_dbg(pxp->dev, "PXP_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_CTRL));
	dev_dbg(pxp->dev, "PXP_STAT 0x%x",
		__raw_readl(pxp->base + HW_PXP_STAT));
	dev_dbg(pxp->dev, "PXP_OUT_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_CTRL));
	dev_dbg(pxp->dev, "PXP_OUT_BUF 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_BUF));
	dev_dbg(pxp->dev, "PXP_OUT_BUF2 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_BUF2));
	dev_dbg(pxp->dev, "PXP_OUT_PITCH 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_PITCH));
	dev_dbg(pxp->dev, "PXP_OUT_LRC 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_LRC));
	dev_dbg(pxp->dev, "PXP_OUT_PS_ULC 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_PS_ULC));
	dev_dbg(pxp->dev, "PXP_OUT_PS_LRC 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_PS_LRC));
	dev_dbg(pxp->dev, "PXP_OUT_AS_ULC 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_AS_ULC));
	dev_dbg(pxp->dev, "PXP_OUT_AS_LRC 0x%x",
		__raw_readl(pxp->base + HW_PXP_OUT_AS_LRC));
	dev_dbg(pxp->dev, "PXP_PS_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_CTRL));
	dev_dbg(pxp->dev, "PXP_PS_BUF 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_BUF));
	dev_dbg(pxp->dev, "PXP_PS_UBUF 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_UBUF));
	dev_dbg(pxp->dev, "PXP_PS_VBUF 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_VBUF));
	dev_dbg(pxp->dev, "PXP_PS_PITCH 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_PITCH));
	dev_dbg(pxp->dev, "PXP_PS_BACKGROUND_0 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_BACKGROUND_0));
	dev_dbg(pxp->dev, "PXP_PS_SCALE 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_SCALE));
	dev_dbg(pxp->dev, "PXP_PS_OFFSET 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_OFFSET));
	dev_dbg(pxp->dev, "PXP_PS_CLRKEYLOW_0 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_CLRKEYLOW_0));
	dev_dbg(pxp->dev, "PXP_PS_CLRKEYHIGH 0x%x",
		__raw_readl(pxp->base + HW_PXP_PS_CLRKEYHIGH_0));
	dev_dbg(pxp->dev, "PXP_AS_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_AS_CTRL));
	dev_dbg(pxp->dev, "PXP_AS_BUF 0x%x",
		__raw_readl(pxp->base + HW_PXP_AS_BUF));
	dev_dbg(pxp->dev, "PXP_AS_PITCH 0x%x",
		__raw_readl(pxp->base + HW_PXP_AS_PITCH));
	dev_dbg(pxp->dev, "PXP_AS_CLRKEYLOW 0x%x",
		__raw_readl(pxp->base + HW_PXP_AS_CLRKEYLOW_0));
	dev_dbg(pxp->dev, "PXP_AS_CLRKEYHIGH 0x%x",
		__raw_readl(pxp->base + HW_PXP_AS_CLRKEYHIGH_0));
	dev_dbg(pxp->dev, "PXP_CSC1_COEF0 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC1_COEF0));
	dev_dbg(pxp->dev, "PXP_CSC1_COEF1 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC1_COEF1));
	dev_dbg(pxp->dev, "PXP_CSC1_COEF2 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC1_COEF2));
	dev_dbg(pxp->dev, "PXP_CSC2_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC2_CTRL));
	dev_dbg(pxp->dev, "PXP_CSC2_COEF0 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC2_COEF0));
	dev_dbg(pxp->dev, "PXP_CSC2_COEF1 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC2_COEF1));
	dev_dbg(pxp->dev, "PXP_CSC2_COEF2 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC2_COEF2));
	dev_dbg(pxp->dev, "PXP_CSC2_COEF3 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC2_COEF3));
	dev_dbg(pxp->dev, "PXP_CSC2_COEF4 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC2_COEF4));
	dev_dbg(pxp->dev, "PXP_CSC2_COEF5 0x%x",
		__raw_readl(pxp->base + HW_PXP_CSC2_COEF5));
	dev_dbg(pxp->dev, "PXP_LUT_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_LUT_CTRL));
	dev_dbg(pxp->dev, "PXP_LUT_ADDR 0x%x",
		__raw_readl(pxp->base + HW_PXP_LUT_ADDR));
	dev_dbg(pxp->dev, "PXP_LUT_DATA 0x%x",
		__raw_readl(pxp->base + HW_PXP_LUT_DATA));
	dev_dbg(pxp->dev, "PXP_LUT_EXTMEM 0x%x",
		__raw_readl(pxp->base + HW_PXP_LUT_EXTMEM));
	dev_dbg(pxp->dev, "PXP_CFA 0x%x",
		__raw_readl(pxp->base + HW_PXP_CFA));
	dev_dbg(pxp->dev, "PXP_ALPHA_A_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_ALPHA_A_CTRL));
	dev_dbg(pxp->dev, "PXP_ALPHA_B_CTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_ALPHA_B_CTRL));
	dev_dbg(pxp->dev, "PXP_POWER_REG0 0x%x",
		__raw_readl(pxp->base + HW_PXP_POWER_REG0));
	dev_dbg(pxp->dev, "PXP_NEXT 0x%x",
		__raw_readl(pxp->base + HW_PXP_NEXT));
	dev_dbg(pxp->dev, "PXP_DEBUGCTRL 0x%x",
		__raw_readl(pxp->base + HW_PXP_DEBUGCTRL));
	dev_dbg(pxp->dev, "PXP_DEBUG 0x%x",
		__raw_readl(pxp->base + HW_PXP_DEBUG));
	dev_dbg(pxp->dev, "PXP_VERSION 0x%x",
		__raw_readl(pxp->base + HW_PXP_VERSION));
}

static void dump_pxp_reg2(struct pxps *pxp)
{
#ifdef DEBUG
	int i = 0;

	for (i=0; i< ((0x33C0/0x10) + 1);i++) {
		printk("0x%08x: 0x%08x\n", 0x10*i, __raw_readl(pxp->base + 0x10*i));
	}
#endif
}

static void print_param(struct pxp_layer_param *p, char *s)
{
	pr_debug("%s: t/l/w/h/s %d/%d/%d/%d/%d, addr %x\n", s,
		p->top, p->left, p->width, p->height, p->stride, p->paddr);
}

/* when it is, return yuv plane number */
static uint8_t is_yuv(uint32_t format)
{
	switch (format) {
	case PXP_PIX_FMT_GREY:
	case PXP_PIX_FMT_GY04:
	case PXP_PIX_FMT_YUYV:
	case PXP_PIX_FMT_UYVY:
	case PXP_PIX_FMT_YVYU:
	case PXP_PIX_FMT_VYUY:
	case PXP_PIX_FMT_YUV444:
	case PXP_PIX_FMT_YVU444:
		return 1;
	case PXP_PIX_FMT_NV12:
	case PXP_PIX_FMT_NV21:
	case PXP_PIX_FMT_NV16:
	case PXP_PIX_FMT_NV61:
		return 2;
	case PXP_PIX_FMT_YUV420P:
	case PXP_PIX_FMT_YUV422P:
	case PXP_PIX_FMT_YVU420P:
	case PXP_PIX_FMT_YVU422P:
		return 3;
	default:
		return 0;
	}
}

static u32 get_bpp_from_fmt(u32 pix_fmt)
{
	unsigned int bpp = 0;

	switch (pix_fmt) {
	case PXP_PIX_FMT_GREY:
	case PXP_PIX_FMT_NV16:
	case PXP_PIX_FMT_NV61:
	case PXP_PIX_FMT_NV12:
	case PXP_PIX_FMT_NV21:
	case PXP_PIX_FMT_YUV422P:
	case PXP_PIX_FMT_YVU422P:
	case PXP_PIX_FMT_YUV420P:
	case PXP_PIX_FMT_YVU420P:
		bpp = 8;
		break;
	case PXP_PIX_FMT_RGB555:
	case PXP_PIX_FMT_ARGB555:
	case PXP_PIX_FMT_XRGB555:
	case PXP_PIX_FMT_RGBA555:
	case PXP_PIX_FMT_ARGB444:
	case PXP_PIX_FMT_XRGB444:
	case PXP_PIX_FMT_RGBA444:
	case PXP_PIX_FMT_RGB565:
	case PXP_PIX_FMT_BGR565:
	case PXP_PIX_FMT_YUYV:
	case PXP_PIX_FMT_YVYU:
	case PXP_PIX_FMT_UYVY:
	case PXP_PIX_FMT_VYUY:
		bpp = 16;
		break;
	case PXP_PIX_FMT_RGB24:
	case PXP_PIX_FMT_BGR24:
		bpp = 24;
		break;
	case PXP_PIX_FMT_XRGB32:
	case PXP_PIX_FMT_RGBX32:
	case PXP_PIX_FMT_XBGR32:
	case PXP_PIX_FMT_BGRX32:
	case PXP_PIX_FMT_ARGB32:
	case PXP_PIX_FMT_RGBA32:
	case PXP_PIX_FMT_ABGR32:
	case PXP_PIX_FMT_BGRA32:
	case PXP_PIX_FMT_YUV444:
	case PXP_PIX_FMT_YVU444:
		bpp = 32;
		break;
	default:
		pr_err("%s: pix_fmt unsupport yet: 0x%x\n", __func__, pix_fmt);
		break;
	}

	return bpp;
}

static uint32_t pxp_parse_ps_fmt(uint32_t format)
{
	uint32_t fmt_ctrl;

	switch (format) {
	case PXP_PIX_FMT_XRGB32:
	case PXP_PIX_FMT_ARGB32:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__RGB888;
		break;
	case PXP_PIX_FMT_RGB565:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__RGB565;
		break;
	case PXP_PIX_FMT_RGB555:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__RGB555;
		break;
	case PXP_PIX_FMT_YUV420P:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YUV420;
		break;
	case PXP_PIX_FMT_YVU420P:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YUV420;
		break;
	case PXP_PIX_FMT_GREY:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__Y8;
		break;
	case PXP_PIX_FMT_GY04:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__Y4;
		break;
	case PXP_PIX_FMT_VUY444:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YUV1P444;
		break;
	case PXP_PIX_FMT_YUV422P:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YUV422;
		break;
	case PXP_PIX_FMT_UYVY:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__UYVY1P422;
		break;
	case PXP_PIX_FMT_YUYV:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__UYVY1P422;
		break;
	case PXP_PIX_FMT_VYUY:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__VYUY1P422;
		break;
	case PXP_PIX_FMT_YVYU:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__VYUY1P422;
		break;
	case PXP_PIX_FMT_NV12:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YUV2P420;
		break;
	case PXP_PIX_FMT_NV21:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YVU2P420;
		break;
	case PXP_PIX_FMT_NV16:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YUV2P422;
		break;
	case PXP_PIX_FMT_NV61:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__YVU2P422;
		break;
	case PXP_PIX_FMT_RGBA32:
	case PXP_PIX_FMT_RGBX32:
		fmt_ctrl = BV_PXP_PS_CTRL_FORMAT__RGBA888;
		break;
	default:
		pr_debug("PS doesn't support this format\n");
		fmt_ctrl = 0;
	}

	return fmt_ctrl;
}

static void pxp_set_colorkey(struct pxps *pxp)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_layer_param *s0_params = &pxp_conf->s0_param;
	struct pxp_layer_param *ol_params = &pxp_conf->ol_param[0];

	/* Low and high are set equal. V4L does not allow a chromakey range */
	if (s0_params->color_key_enable == 0 || s0_params->color_key == -1) {
		/* disable color key */
		pxp_writel(0xFFFFFF, HW_PXP_PS_CLRKEYLOW_0);
		pxp_writel(0, HW_PXP_PS_CLRKEYHIGH_0);
	} else {
		pxp_writel(s0_params->color_key, HW_PXP_PS_CLRKEYLOW_0);
		pxp_writel(s0_params->color_key, HW_PXP_PS_CLRKEYHIGH_0);
	}

	if (ol_params->color_key_enable != 0 && ol_params->color_key != -1) {
		pxp_writel(ol_params->color_key, HW_PXP_AS_CLRKEYLOW_0);
		pxp_writel(ol_params->color_key, HW_PXP_AS_CLRKEYHIGH_0);
	} else {
		/* disable color key */
		pxp_writel(0xFFFFFF, HW_PXP_AS_CLRKEYLOW_0);
		pxp_writel(0, HW_PXP_AS_CLRKEYHIGH_0);
	}
}

static uint32_t pxp_parse_as_fmt(uint32_t format)
{
	uint32_t fmt_ctrl;

	switch (format) {
	case PXP_PIX_FMT_BGRA32:
	case PXP_PIX_FMT_ARGB32:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__ARGB8888;
		break;
	case PXP_PIX_FMT_RGBA32:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__RGBA8888;
		break;
	case PXP_PIX_FMT_XRGB32:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__RGB888;
		break;
	case PXP_PIX_FMT_ARGB555:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__ARGB1555;
		break;
	case PXP_PIX_FMT_ARGB444:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__ARGB4444;
		break;
	case PXP_PIX_FMT_RGBA555:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__RGBA5551;
		break;
	case PXP_PIX_FMT_RGBA444:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__RGBA4444;
		break;
	case PXP_PIX_FMT_RGB555:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__RGB555;
		break;
	case PXP_PIX_FMT_RGB444:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__RGB444;
		break;
	case PXP_PIX_FMT_RGB565:
		fmt_ctrl = BV_PXP_AS_CTRL_FORMAT__RGB565;
		break;
	default:
		pr_debug("AS doesn't support this format\n");
		fmt_ctrl = 0xf;
		break;
	}

	return fmt_ctrl;
}

static uint32_t pxp_parse_out_fmt(uint32_t format)
{
	uint32_t fmt_ctrl;

	switch (format) {
	case PXP_PIX_FMT_BGRA32:
	case PXP_PIX_FMT_ARGB32:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__ARGB8888;
		break;
	case PXP_PIX_FMT_XRGB32:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__RGB888;
		break;
	case PXP_PIX_FMT_RGB24:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__RGB888P;
		break;
	case PXP_PIX_FMT_RGB565:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__RGB565;
		break;
	case PXP_PIX_FMT_RGB555:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__RGB555;
		break;
	case PXP_PIX_FMT_GREY:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__Y8;
		break;
	case PXP_PIX_FMT_GY04:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__Y4;
		break;
	case PXP_PIX_FMT_UYVY:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__UYVY1P422;
		break;
	case PXP_PIX_FMT_VYUY:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__VYUY1P422;
		break;
	case PXP_PIX_FMT_NV12:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__YUV2P420;
		break;
	case PXP_PIX_FMT_NV21:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__YVU2P420;
		break;
	case PXP_PIX_FMT_NV16:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__YUV2P422;
		break;
	case PXP_PIX_FMT_NV61:
		fmt_ctrl = BV_PXP_OUT_CTRL_FORMAT__YVU2P422;
		break;
	default:
		pr_debug("OUT doesn't support this format\n");
		fmt_ctrl = 0;
	}

	return fmt_ctrl;
}

static void set_mux(struct mux_config *path_ctrl)
{
	struct mux_config *mux = path_ctrl;

	*(uint32_t *)path_ctrl = 0xFFFFFFFF;

	mux->mux0_sel = 0;
	mux->mux3_sel = 1;
	mux->mux6_sel = 1;
	mux->mux8_sel = 0;
	mux->mux9_sel = 1;
	mux->mux11_sel = 0;
	mux->mux12_sel = 1;
	mux->mux14_sel = 0;
}

static void set_mux_val(struct mux_config *muxes,
			uint32_t mux_id,
			uint32_t mux_val)
{
	BUG_ON(!muxes);
	BUG_ON(mux_id > 15);

	switch (mux_id) {
	case 0:
		muxes->mux0_sel  = mux_val;
		break;
	case 1:
		muxes->mux1_sel  = mux_val;
		break;
	case 2:
		muxes->mux2_sel  = mux_val;
		break;
	case 3:
		muxes->mux3_sel  = mux_val;
		break;
	case 4:
		muxes->mux4_sel  = mux_val;
		break;
	case 5:
		muxes->mux5_sel  = mux_val;
		break;
	case 6:
		muxes->mux6_sel  = mux_val;
		break;
	case 7:
		muxes->mux7_sel  = mux_val;
		break;
	case 8:
		muxes->mux8_sel  = mux_val;
		break;
	case 9:
		muxes->mux9_sel  = mux_val;
		break;
	case 10:
		muxes->mux10_sel = mux_val;
		break;
	case 11:
		muxes->mux11_sel = mux_val;
		break;
	case 12:
		muxes->mux12_sel = mux_val;
		break;
	case 13:
		muxes->mux13_sel = mux_val;
		break;
	case 14:
		muxes->mux14_sel = mux_val;
		break;
	case 15:
		muxes->mux15_sel = mux_val;
		break;
	default:
		break;
	}
}

static uint32_t get_mux_val(struct mux_config *muxes,
			    uint32_t mux_id)
{
	BUG_ON(!muxes);
	BUG_ON(mux_id > 15);

	switch (mux_id) {
	case 0:
		return muxes->mux0_sel;
	case 1:
		return muxes->mux1_sel;
	case 2:
		return muxes->mux2_sel;
	case 3:
		return muxes->mux3_sel;
	case 4:
		return muxes->mux4_sel;
	case 5:
		return muxes->mux5_sel;
	case 6:
		return muxes->mux6_sel;
	case 7:
		return muxes->mux7_sel;
	case 8:
		return muxes->mux8_sel;
	case 9:
		return muxes->mux9_sel;
	case 10:
		return muxes->mux10_sel;
	case 11:
		return muxes->mux11_sel;
	case 12:
		return muxes->mux12_sel;
	case 13:
		return muxes->mux13_sel;
	case 14:
		return muxes->mux14_sel;
	case 15:
		return muxes->mux15_sel;
	default:
		return -EINVAL;
	}
}

static uint32_t pxp_store_ctrl_config(struct pxp_pixmap *out, uint8_t mode,
				      uint8_t fill_en, uint8_t combine_2ch)
{
	struct store_ctrl ctrl;
	uint8_t output_active_bpp;

	memset((void*)&ctrl, 0x0, sizeof(ctrl));

	if (combine_2ch) {
		ctrl.combine_2channel = 1;
		if (out) {
			output_active_bpp = active_bpp(out->bpp);
			ctrl.pack_in_sel  = (output_active_bpp < 0x3) ? 1 : 0;
			ctrl.store_memory_en = 1;
		}
	} else {
		if (fill_en) {
			ctrl.fill_data_en = 1;
			ctrl.wr_num_bytes = 2;
		}
		ctrl.store_memory_en = 1;
	}

	if (out->rotate || out->flip)
		ctrl.block_en = 1;

	ctrl.ch_en = 1;

	return *(uint32_t *)&ctrl;
}

static uint32_t pxp_store_size_config(struct pxp_pixmap *out)
{
	struct store_size size;

	memset((void*)&size, 0x0, sizeof(size));

	size.out_height = out->height - 1;
	size.out_width  = out->width - 1;

	return *(uint32_t *)&size;
}

static uint32_t pxp_store_pitch_config(struct pxp_pixmap *out0,
				       struct pxp_pixmap *out1)
{
	struct store_pitch pitch;

	memset((void*)&pitch, 0x0, sizeof(pitch));

	pitch.ch0_out_pitch = out0->pitch;
	pitch.ch1_out_pitch = out1 ? out1->pitch : 0;

	return *(uint32_t *)&pitch;
}

static struct color *pxp_find_rgb_color(uint32_t format)
{
	int i;

	for (i = 0; i < sizeof(rgb_colors) / sizeof(struct color); i++) {
		if (rgb_colors[i].format == format)
			return &rgb_colors[i];
	}

	return NULL;
}

static struct color_component *pxp_find_comp(struct color *color, uint8_t id)
{
	int i;

	for (i = 0; i < 4; i++) {
		if (id == color->comp[i].id)
			return &color->comp[i];
	}

	return NULL;
}

static struct color *pxp_find_yuv_color(uint32_t format)
{
	int i;

	for (i = 0; i < sizeof(yuv_colors) / sizeof(struct color); i++) {
		if (yuv_colors[i].format == format)
			return &yuv_colors[i];
	}

	return NULL;
}

static uint64_t pxp_store_d_shift_calc(uint32_t in_fmt, uint32_t out_fmt,
				       struct store_d_mask *d_mask)
{
	int i, shift_width, shift_flag, drop = 0;
	struct store_d_shift d_shift;
	struct color *input_color, *output_color;
	struct color_component *input_comp, *output_comp;

	BUG_ON((in_fmt == out_fmt));
	memset((void*)&d_shift, 0x0, sizeof(d_shift));
	memset((void*)d_mask, 0x0, sizeof(*d_mask) * 8);

	if (!is_yuv(in_fmt)) {
		input_color  = pxp_find_rgb_color(in_fmt);
		output_color = pxp_find_rgb_color(out_fmt);
	} else {
		input_color  = pxp_find_yuv_color(in_fmt);
		output_color = pxp_find_yuv_color(out_fmt);
	}

	for (i = 0; i < 4; i++) {
		input_comp  = &input_color->comp[i];
		if (!input_comp->length)
			continue;

		output_comp = pxp_find_comp(output_color, input_comp->id);
		if (!output_comp->length)
			continue;

		/* only rgb format can drop color bits */
		if (input_comp->length > output_comp->length) {
			drop = input_comp->length - output_comp->length;
			input_comp->offset += drop;
		}
		d_mask[i].d_mask_l = output_comp->mask << input_comp->offset;

		shift_width = input_comp->offset - output_comp->offset;
		if (shift_width > 0)
			shift_flag = 0;		/* right shift */
		else if (shift_width < 0) {
			shift_flag = 1;		/* left shift */
			shift_width = -shift_width;
		} else
			shift_width = shift_flag = 0; /* no shift require */

		switch (i) {
		case 0:
			d_shift.d_shift_width0 = shift_width;
			d_shift.d_shift_flag0  = shift_flag;
			break;
		case 1:
			d_shift.d_shift_width1 = shift_width;
			d_shift.d_shift_flag1  = shift_flag;
			break;
		case 2:
			d_shift.d_shift_width2 = shift_width;
			d_shift.d_shift_flag2  = shift_flag;
			break;
		case 3:
			d_shift.d_shift_width3 = shift_width;
			d_shift.d_shift_flag3  = shift_flag;
			break;
		default:
			printk(KERN_ERR "unsupport d shift\n");
			break;
		}

		input_comp->offset -= drop;
	}

	return *(uint64_t *)&d_shift;
}

static uint32_t pxp_store_shift_ctrl_config(struct pxp_pixmap *out,
					    uint8_t shift_bypass)
{
	struct store_shift_ctrl shift_ctrl;

	memset((void*)&shift_ctrl, 0x0, sizeof(shift_ctrl));

	shift_ctrl.output_active_bpp = active_bpp(out->bpp);
	/* Not general data */
	if (!shift_bypass) {
		switch(out->format) {
		case PXP_PIX_FMT_YUYV:
			shift_bypass = 1;
		case PXP_PIX_FMT_YVYU:
			shift_ctrl.out_yuv422_1p_en = 1;
			break;
		case PXP_PIX_FMT_NV16:
			shift_bypass = 1;
		case PXP_PIX_FMT_NV61:
			shift_ctrl.out_yuv422_2p_en = 1;
			break;
		default:
			break;
		}
	}
	shift_ctrl.shift_bypass = shift_bypass;

	return *(uint32_t *)&shift_ctrl;
}

static uint32_t pxp_fetch_ctrl_config(struct pxp_pixmap *in,
				      uint8_t mode)
{
	struct fetch_ctrl ctrl;

	memset((void*)&ctrl, 0x0, sizeof(ctrl));

	if (mode == FETCH_MODE_NORMAL)
		ctrl.bypass_pixel_en = 0;

	if (in->flip == PXP_H_FLIP)
		ctrl.hflip = 1;
	else if (in->flip == PXP_V_FLIP)
		ctrl.vflip = 1;

	ctrl.rotation_angle = rotate_map(in->rotate);

	if (in->rotate || in->flip)
		ctrl.block_en = 1;

	ctrl.ch_en = 1;

	return *(uint32_t *)&ctrl;
}

static uint32_t pxp_fetch_active_size_ulc(struct pxp_pixmap *in)
{
	struct fetch_active_size_ulc size_ulc;

	memset((void*)&size_ulc, 0x0, sizeof(size_ulc));

	size_ulc.active_size_ulc_x = 0;
	size_ulc.active_size_ulc_y = 0;

	return *(uint32_t *)&size_ulc;
}

static uint32_t pxp_fetch_active_size_lrc(struct pxp_pixmap *in)
{
	struct fetch_active_size_lrc size_lrc;

	memset((void*)&size_lrc, 0x0, sizeof(size_lrc));

	size_lrc.active_size_lrc_x = in->crop.width - 1;
	size_lrc.active_size_lrc_y = in->crop.height - 1;

	return *(uint32_t *)&size_lrc;
}

static uint32_t pxp_fetch_pitch_config(struct pxp_pixmap *in0,
				       struct pxp_pixmap *in1)
{
	struct fetch_pitch pitch;

	memset((void*)&pitch, 0x0, sizeof(pitch));

	if (in0)
		pitch.ch0_input_pitch = in0->pitch;
	if (in1)
		pitch.ch1_input_pitch = in1->pitch;

	return *(uint32_t *)&pitch;
}

static uint32_t pxp_fetch_shift_ctrl_config(struct pxp_pixmap *in,
					    uint8_t shift_bypass,
					    uint8_t need_expand)
{
	uint8_t input_expand_format;
	struct fetch_shift_ctrl shift_ctrl;

	memset((void*)&shift_ctrl, 0x0, sizeof(shift_ctrl));

	shift_ctrl.input_active_bpp = active_bpp(in->bpp);
	shift_ctrl.shift_bypass = shift_bypass;

	if (in->bpp == 32)
		need_expand = 0;

	if (need_expand) {
		input_expand_format = expand_format(in->format);

		if (input_expand_format <= 0x7) {
			shift_ctrl.expand_en = 1;
			shift_ctrl.expand_format = input_expand_format;
		}
	}

	return *(uint32_t *)&shift_ctrl;
}

static uint32_t pxp_fetch_shift_calc(uint32_t in_fmt, uint32_t out_fmt,
				     struct fetch_shift_width *shift_width)
{
	int i;
	struct fetch_shift_offset shift_offset;
	struct color *input_color, *output_color;
	struct color_component *input_comp, *output_comp;

	memset((void*)&shift_offset, 0x0, sizeof(shift_offset));
	memset((void*)shift_width, 0x0, sizeof(*shift_width));

	if (!is_yuv(in_fmt)) {
		input_color  = pxp_find_rgb_color(in_fmt);
		output_color = pxp_find_rgb_color(out_fmt);
	} else {
		input_color  = pxp_find_yuv_color(in_fmt);
		output_color = pxp_find_yuv_color(out_fmt);
	}

	for(i = 0; i < 4; i++) {
		output_comp = &output_color->comp[i];
		if (!output_comp->length)
			continue;

		input_comp = pxp_find_comp(input_color, output_comp->id);
		switch (i) {
		case 0:
			shift_offset.offset0 = input_comp->offset;
			shift_width->width0  = input_comp->length;
			break;
		case 1:
			shift_offset.offset1 = input_comp->offset;
			shift_width->width1  = input_comp->length;
			break;
		case 2:
			shift_offset.offset2 = input_comp->offset;
			shift_width->width2  = input_comp->length;
			break;
		case 3:
			shift_offset.offset3 = input_comp->offset;
			shift_width->width3  = input_comp->length;
			break;
		}
	}

	return *(uint32_t *)&shift_offset;
}

static int pxp_start(struct pxps *pxp)
{
	__raw_writel(BM_PXP_CTRL_ENABLE_ROTATE1 | BM_PXP_CTRL_ENABLE |
		BM_PXP_CTRL_ENABLE_CSC2 | BM_PXP_CTRL_ENABLE_LUT |
		BM_PXP_CTRL_ENABLE_PS_AS_OUT | BM_PXP_CTRL_ENABLE_ROTATE0,
			pxp->base + HW_PXP_CTRL_SET);
	dump_pxp_reg(pxp);

	return 0;
}

static bool fmt_ps_support(uint32_t format)
{
	switch (format) {
	case PXP_PIX_FMT_XRGB32:
	case PXP_PIX_FMT_ARGB32:
	case PXP_PIX_FMT_RGB555:
	case PXP_PIX_FMT_XRGB555:
	case PXP_PIX_FMT_ARGB555:
	case PXP_PIX_FMT_RGB444:
	case PXP_PIX_FMT_XRGB444:
	case PXP_PIX_FMT_ARGB444:
	case PXP_PIX_FMT_RGB565:
	case PXP_PIX_FMT_YUV444:
	case PXP_PIX_FMT_UYVY:
	/* need word byte swap */
	case PXP_PIX_FMT_YUYV:
	case PXP_PIX_FMT_VYUY:
	/* need word byte swap */
	case PXP_PIX_FMT_YVYU:
	case PXP_PIX_FMT_GREY:
	case PXP_PIX_FMT_GY04:
	case PXP_PIX_FMT_NV16:
	case PXP_PIX_FMT_NV12:
	case PXP_PIX_FMT_NV61:
	case PXP_PIX_FMT_NV21:
	case PXP_PIX_FMT_YUV422P:
	case PXP_PIX_FMT_YUV420P:
	case PXP_PIX_FMT_YVU420P:
	case PXP_PIX_FMT_RGBA32:
	case PXP_PIX_FMT_RGBX32:
	case PXP_PIX_FMT_RGBA555:
	case PXP_PIX_FMT_RGBA444:
		return true;
	default:
		return false;
	}
}

static bool fmt_as_support(uint32_t format)
{
	switch (format) {
	case PXP_PIX_FMT_ARGB32:
	case PXP_PIX_FMT_RGBA32:
	case PXP_PIX_FMT_XRGB32:
	case PXP_PIX_FMT_BGRA32:
	case PXP_PIX_FMT_ARGB555:
	case PXP_PIX_FMT_ARGB444:
	case PXP_PIX_FMT_RGBA555:
	case PXP_PIX_FMT_RGBA444:
	case PXP_PIX_FMT_RGB555:
	case PXP_PIX_FMT_RGB444:
	case PXP_PIX_FMT_RGB565:
		return true;
	default:
		return false;
	}
}

static bool fmt_out_support(uint32_t format)
{
	switch (format) {
	case PXP_PIX_FMT_ARGB32:
	case PXP_PIX_FMT_XRGB32:
	case PXP_PIX_FMT_BGRA32:
	case PXP_PIX_FMT_RGB24:
	case PXP_PIX_FMT_ARGB555:
	case PXP_PIX_FMT_ARGB444:
	case PXP_PIX_FMT_RGB555:
	case PXP_PIX_FMT_RGB444:
	case PXP_PIX_FMT_RGB565:
	case PXP_PIX_FMT_YUV444:
	case PXP_PIX_FMT_UYVY:
	case PXP_PIX_FMT_VYUY:
	case PXP_PIX_FMT_GREY:
	case PXP_PIX_FMT_GY04:
	case PXP_PIX_FMT_NV16:
	case PXP_PIX_FMT_NV12:
	case PXP_PIX_FMT_NV61:
	case PXP_PIX_FMT_NV21:
		return true;
	default:
		return false;
	}
}

/* common means 'ARGB32/XRGB32/YUV444' */
static uint8_t fmt_fetch_to_common(uint32_t in)
{
	switch (in) {
	case PXP_PIX_FMT_ARGB32:
	case PXP_PIX_FMT_XRGB32:
	case PXP_PIX_FMT_YUV444:
		return FETCH_NOOP;

	case PXP_PIX_FMT_RGB565:
	case PXP_PIX_FMT_RGB555:
	case PXP_PIX_FMT_ARGB555:
	case PXP_PIX_FMT_RGB444:
	case PXP_PIX_FMT_ARGB444:
	case PXP_PIX_FMT_YUYV:
	case PXP_PIX_FMT_UYVY:
	case PXP_PIX_FMT_NV16:
		return FETCH_EXPAND;

	case PXP_PIX_FMT_RGBA32:
	case PXP_PIX_FMT_RGBX32:
	case PXP_PIX_FMT_BGRA32:
	case PXP_PIX_FMT_BGRX32:
	case PXP_PIX_FMT_ABGR32:
	case PXP_PIX_FMT_XBGR32:
	case PXP_PIX_FMT_YVU444:
		return FETCH_SHIFT;

	case PXP_PIX_FMT_BGR565:
	case PXP_PIX_FMT_YVYU:
	case PXP_PIX_FMT_VYUY:
		return FETCH_EXPAND | FETCH_SHIFT;

	default:
		return 0;
	}
}

static uint8_t fmt_store_from_common(uint32_t out)
{
	switch (out) {
	case PXP_PIX_FMT_ARGB32:
	case PXP_PIX_FMT_XRGB32:
	case PXP_PIX_FMT_YUV444:
		return STORE_NOOP;

	case PXP_PIX_FMT_YUYV:
	case PXP_PIX_FMT_NV16:
		return STORE_SHRINK;

	case PXP_PIX_FMT_RGBA32:
	case PXP_PIX_FMT_RGBX32:
	case PXP_PIX_FMT_BGRA32:
	case PXP_PIX_FMT_BGRX32:
	case PXP_PIX_FMT_ABGR32:
	case PXP_PIX_FMT_XBGR32:
	case PXP_PIX_FMT_YVU444:
	case PXP_PIX_FMT_RGB565:
	case PXP_PIX_FMT_RGB555:
	case PXP_PIX_FMT_ARGB555:
	case PXP_PIX_FMT_RGB444:
	case PXP_PIX_FMT_ARGB444:
	case PXP_PIX_FMT_GREY:
		return STORE_SHIFT;

	case PXP_PIX_FMT_YVYU:
	case PXP_PIX_FMT_NV61:
		return STORE_SHIFT | STORE_SHRINK;

	default:
		return 0;
	}
}

static void filter_possible_inputs(struct pxp_pixmap *input,
				   uint32_t *possible)
{
	uint8_t clear = 0xff;
	uint8_t position = 0;

	do {
		position = find_next_bit((unsigned long *)possible, 32, position);
		if (position >= sizeof(uint32_t) * 8)
			break;

		switch (position) {
		case PXP_2D_PS:
			if (!fmt_ps_support(input->format))
				clear = PXP_2D_PS;
			break;
		case PXP_2D_AS:
			if (!fmt_as_support(input->format))
				clear = PXP_2D_AS;
			break;
		case PXP_2D_INPUT_FETCH0:
		case PXP_2D_INPUT_FETCH1:
			if ((is_yuv(input->format) == 3)) {
				clear = position;
				break;
			}
			if ((input->flags & IN_NEED_FMT_UNIFIED) ||
			     is_yuv(input->format) == 2)
				if (!fmt_fetch_to_common(input->format))
					clear = position;
			break;
		default:
			pr_err("invalid input node: %d\n", position);
			clear = position;
			break;
		}

		if (clear != 0xff) {
			clear_bit(clear, (unsigned long*)possible);
			clear = 0xff;
		}

		position++;
	} while (1);
}

static void filter_possible_outputs(struct pxp_pixmap *output,
				    uint32_t *possible)
{
	uint8_t clear = 0xff;
	uint8_t position = 0;

	do {
		position = find_next_bit((unsigned long *)possible, 32, position);
		if (position >= sizeof(uint32_t) * 8)
			break;

		switch (position) {
		case PXP_2D_OUT:
			if (!fmt_out_support(output->format))
				clear = PXP_2D_OUT;
			break;
		case PXP_2D_INPUT_STORE0:
		case PXP_2D_INPUT_STORE1:
			if (output->flags) {
				if (!fmt_store_from_common(output->format))
					clear = position;
			}
			break;
		default:
			pr_err("invalid output node: %d\n", position);
			clear = position;
			break;
		}

		if (clear != 0xff) {
			clear_bit(clear, (unsigned long*)possible);
			clear = 0xff;
		}

		position++;
	} while (1);
}

static uint32_t calc_shortest_path(uint32_t *nodes_used)
{
	uint32_t distance = 0;
	uint32_t from = 0, to = 0, bypass, end;

	do {
		from = find_next_bit((unsigned long *)nodes_used, 32, from);
		if (from >= sizeof(uint32_t) * 8)
			break;

		if (to != 0) {
			if (path_table[to][from].distance == DISTANCE_INFINITY)
				return DISTANCE_INFINITY;

			distance += path_table[to][from].distance;
			/* backtrace */
			end = from;
			while (1) {
				bypass = path_table[to][end].prev_node;
				if (bypass == to)
					break;
				set_bit(bypass, (unsigned long*)nodes_used);
				end = bypass;
			}
		}

		to = find_next_bit((unsigned long *)nodes_used, 32, from + 1);
		if (to >= sizeof(uint32_t) * 8)
			break;

		if (path_table[from][to].distance == DISTANCE_INFINITY)
			return DISTANCE_INFINITY;

		distance += path_table[from][to].distance;
		/* backtrace */
		end = to;
		while (1) {
			bypass = path_table[from][end].prev_node;
			if (bypass == from)
				break;
			set_bit(bypass, (unsigned long*)nodes_used);
			end = bypass;
		}

		from = to + 1;
	} while (1);

	return distance;
}

static uint32_t find_best_path(uint32_t inputs,
			       uint32_t outputs,
			       struct pxp_pixmap *in,
			       uint32_t *nodes_used)
{
	uint32_t outs;
	uint32_t nodes_add, best_nodes_used = 0;
	uint8_t in_pos = 0, out_pos = 0;
	uint32_t nodes_in_path, best_nodes_in_path = 0;
	uint32_t best_distance = DISTANCE_INFINITY, distance;

	do {
		outs = outputs;
		in_pos = find_next_bit((unsigned long *)&inputs, 32, in_pos);
		if (in_pos >= sizeof(uint32_t) * 8)
			break;
		nodes_add = 0;
		set_bit(in_pos, (unsigned long *)&nodes_add);

		switch (in_pos) {
		case PXP_2D_PS:
			if ((in->flags & IN_NEED_CSC) == IN_NEED_CSC) {
				if (is_yuv(in->format))
					set_bit(PXP_2D_CSC1,
						(unsigned long *)&nodes_add);
				else
					set_bit(PXP_2D_CSC2,
						(unsigned long *)&nodes_add);
			}
			if ((in->flags & IN_NEED_ROTATE_FLIP) == IN_NEED_ROTATE_FLIP)
				set_bit(PXP_2D_ROTATION1,
					(unsigned long *)&nodes_add);
			clear_bit(PXP_2D_INPUT_STORE0, (unsigned long *)&outs);
			break;
		case PXP_2D_AS:
			if ((in->flags & IN_NEED_CSC) == IN_NEED_CSC)
				set_bit(PXP_2D_CSC2,
					(unsigned long *)&nodes_add);
			if ((in->flags & IN_NEED_ROTATE_FLIP) == IN_NEED_ROTATE_FLIP)
				set_bit(PXP_2D_ROTATION0,
					(unsigned long *)&nodes_add);
			clear_bit(PXP_2D_INPUT_STORE0, (unsigned long *)&outs);
			break;
		case PXP_2D_INPUT_FETCH0:
		case PXP_2D_INPUT_FETCH1:
			if ((in->flags & IN_NEED_CSC) == IN_NEED_CSC)
				set_bit(PXP_2D_CSC2,
					(unsigned long *)&nodes_add);
			clear_bit(PXP_2D_OUT, (unsigned long *)&outs);
			if ((in->flags & IN_NEED_ROTATE_FLIP) == IN_NEED_ROTATE_FLIP)
				set_bit(PXP_2D_ROTATION1,
					(unsigned long *)&nodes_add);
			break;
		default:
			/* alph0_s0/s1, alpha1_s0/s1 */
			break;
		}

		nodes_add |= *nodes_used;

		do {
			out_pos = find_next_bit((unsigned long *)&outs, 32, out_pos);
			if (out_pos >= sizeof(uint32_t) * 8)
				break;
			set_bit(out_pos, (unsigned long *)&nodes_add);

			switch(out_pos) {
			case PXP_2D_ALPHA0_S0:
			case PXP_2D_ALPHA0_S1:
			case PXP_2D_ALPHA1_S0:
			case PXP_2D_ALPHA1_S1:
				clear_bit(PXP_2D_CSC2, (unsigned long *)&nodes_add);
				clear_bit(PXP_2D_ROTATION0, (unsigned long *)&nodes_add);
				clear_bit(PXP_2D_LUT, (unsigned long *)&nodes_add);
				break;
			default:
				break;
			}

			nodes_in_path = nodes_add;
			distance = calc_shortest_path(&nodes_in_path);
			if (best_distance > distance) {
				best_distance = distance;
				best_nodes_used = nodes_add;
				best_nodes_in_path = nodes_in_path;
			}
			pr_debug("%s: out_pos = %d, nodes_in_path = 0x%x, nodes_add = 0x%x, distance = 0x%x\n",
				 __func__, out_pos, nodes_in_path, nodes_add, distance);

			clear_bit(out_pos, (unsigned long *)&nodes_add);

			out_pos++;
		} while (1);

		in_pos++;
	} while (1);

	*nodes_used = best_nodes_used;

	return best_nodes_in_path;
}

static uint32_t ps_calc_scaling(struct pxp_pixmap *input,
				struct pxp_pixmap *output,
				struct ps_ctrl *ctrl)
{
	struct ps_scale scale;
	uint32_t decx, decy;

	memset((void*)&scale, 0x0, sizeof(scale));

	if (!output->crop.width || !output->crop.height) {
		pr_err("Invalid drect width and height passed in\n");
		return 0;
	}

	if ((input->rotate == 90) || (input->rotate == 270))
		swap(output->crop.width, output->crop.height);

	decx = input->crop.width  / output->crop.width;
	decy = input->crop.height / output->crop.height;

	if (decx > 1) {
		if (decx >= 2 && decx < 4) {
			decx = 2;
			ctrl->decx = 1;
		} else if (decx >= 4 && decx < 8) {
			decx = 4;
			ctrl->decx = 2;
		} else if (decx >= 8) {
			decx = 8;
			ctrl->decx = 3;
		}
		scale.xscale = input->crop.width * 0x1000 /
				(output->crop.width * decx);
	} else {
		if (!is_yuv(input->format) ||
		    (is_yuv(input->format) == is_yuv(output->format)) ||
		    (input->format == PXP_PIX_FMT_GREY) ||
		    (input->format == PXP_PIX_FMT_GY04) ||
		    (input->format == PXP_PIX_FMT_VUY444)) {
			if ((input->crop.width > 1) &&
			    (output->crop.width > 1))
				scale.xscale = (input->crop.width - 1) * 0x1000 /
						(output->crop.width - 1);
			else
				scale.xscale = input->crop.width * 0x1000 /
						output->crop.width;
		} else {
			if ((input->crop.width > 2) &&
			    (output->crop.width > 1))
				scale.xscale = (input->crop.width - 2) * 0x1000 /
						(output->crop.width - 1);
			else
				scale.xscale = input->crop.width * 0x1000 /
						output->crop.width;
		}
	}

	if (decy > 1) {
		if (decy >= 2 && decy < 4) {
			decy = 2;
			ctrl->decy = 1;
		} else if (decy >= 4 && decy < 8) {
			decy = 4;
			ctrl->decy = 2;
		} else if (decy >= 8) {
			decy = 8;
			ctrl->decy = 3;
		}
		scale.yscale = input->crop.height * 0x1000 /
				(output->crop.height * decy);
	} else {
		if ((input->crop.height > 1) && (output->crop.height > 1))
			scale.yscale = (input->crop.height - 1) * 0x1000 /
					(output->crop.height - 1);
		else
			scale.yscale = input->crop.height * 0x1000 /
					output->crop.height;
	}

	return *(uint32_t *)&scale;
}

static int pxp_ps_config(struct pxp_pixmap *input,
			 struct pxp_pixmap *output)
{
	uint32_t offset, U, V;
	struct ps_ctrl ctrl;
	struct coordinate out_ps_ulc, out_ps_lrc;

	memset((void*)&ctrl, 0x0, sizeof(ctrl));

	ctrl.format = pxp_parse_ps_fmt(input->format);

	switch (output->rotate) {
	case 0:
		out_ps_ulc.x = output->crop.x;
		out_ps_ulc.y = output->crop.y;
		out_ps_lrc.x = out_ps_ulc.x + output->crop.width - 1;
		out_ps_lrc.y = out_ps_ulc.y + output->crop.height - 1;
		break;
	case 90:
		out_ps_ulc.x = output->crop.y;
		out_ps_ulc.y = output->width - (output->crop.x + output->crop.width);
		out_ps_lrc.x = out_ps_ulc.x + output->crop.height - 1;
		out_ps_lrc.y = out_ps_ulc.y + output->crop.width - 1;
		break;
	case 180:
		out_ps_ulc.x = output->width - (output->crop.x + output->crop.width);
		out_ps_ulc.y = output->height - (output->crop.y + output->crop.height);
		out_ps_lrc.x = out_ps_ulc.x + output->crop.width - 1;
		out_ps_lrc.y = out_ps_ulc.y + output->crop.height - 1;
		break;
	case 270:
		out_ps_ulc.x = output->height - (output->crop.y + output->crop.height);
		out_ps_ulc.y = output->crop.x;
		out_ps_lrc.x = out_ps_ulc.x + output->crop.height - 1;
		out_ps_lrc.y = out_ps_ulc.y + output->crop.width - 1;
		break;
	default:
		pr_err("PxP only support rotate 0 90 180 270\n");
		return -EINVAL;
		break;
	}

	if ((input->format == PXP_PIX_FMT_YUYV) ||
	    (input->format == PXP_PIX_FMT_YVYU))
		ctrl.wb_swap = 1;

	pxp_writel(ps_calc_scaling(input, output, &ctrl),
		   HW_PXP_PS_SCALE);
	pxp_writel(*(uint32_t *)&ctrl, HW_PXP_PS_CTRL);

	offset = input->crop.y * input->pitch +
		 input->crop.x * (input->bpp >> 3);
	pxp_writel(input->paddr + offset, HW_PXP_PS_BUF);

	switch (is_yuv(input->format)) {
	case 0:		/* RGB */
	case 1:		/* 1 Plane YUV */
		break;
	case 2:		/* NV16,NV61,NV12,NV21 */
		if ((input->format == PXP_PIX_FMT_NV16) ||
		    (input->format == PXP_PIX_FMT_NV61)) {
			U = input->paddr + input->width * input->height;
			pxp_writel(U + offset, HW_PXP_PS_UBUF);
		}
		else {
			U = input->paddr + input->width * input->height;
			pxp_writel(U + (offset >> 1), HW_PXP_PS_UBUF);
		}
		break;
	case 3:		/* YUV422P, YUV420P */
		if (input->format == PXP_PIX_FMT_YUV422P) {
			U = input->paddr + input->width * input->height;
			pxp_writel(U + (offset >> 1), HW_PXP_PS_UBUF);
			V = U + (input->width * input->height >> 1);
			pxp_writel(V + (offset >> 1), HW_PXP_PS_VBUF);
		} else if (input->format == PXP_PIX_FMT_YUV420P) {
			U = input->paddr + input->width * input->height;
			pxp_writel(U + (offset >> 2), HW_PXP_PS_UBUF);
			V = U + (input->width * input->height >> 2);
			pxp_writel(V + (offset >> 2), HW_PXP_PS_VBUF);
		} else if (input->format == PXP_PIX_FMT_YVU420P) {
			U = input->paddr + input->width * input->height;
			V = U + (input->width * input->height >> 2);
			pxp_writel(U + (offset >> 2), HW_PXP_PS_VBUF);
			pxp_writel(V + (offset >> 2), HW_PXP_PS_UBUF);
		}

		break;
	default:
		break;
	}

	pxp_writel(input->pitch, HW_PXP_PS_PITCH);
	pxp_writel(*(uint32_t *)&out_ps_ulc, HW_PXP_OUT_PS_ULC);
	pxp_writel(*(uint32_t *)&out_ps_lrc, HW_PXP_OUT_PS_LRC);

	pxp_writel(BF_PXP_CTRL_ENABLE_PS_AS_OUT(1) |
		   BF_PXP_CTRL_IRQ_ENABLE(1),
		   HW_PXP_CTRL_SET);

	return 0;
}

static int pxp_as_config(struct pxp_pixmap *input,
			 struct pxp_pixmap *output)
{
	uint32_t offset;
	struct as_ctrl ctrl;
	struct coordinate out_as_ulc, out_as_lrc;

	memset((void*)&ctrl, 0x0, sizeof(ctrl));

	ctrl.format = pxp_parse_as_fmt(input->format);

	if (alpha_blending_version == PXP_ALPHA_BLENDING_V1) {
		if (input->format == PXP_PIX_FMT_BGRA32) {
			if (!input->g_alpha.combine_enable) {
				ctrl.alpha_ctrl = BV_PXP_AS_CTRL_ALPHA_CTRL__ROPs;
				ctrl.rop = 0x3;
			}
		}

		if (input->g_alpha.global_alpha_enable) {
			if (input->g_alpha.global_override)
				ctrl.alpha_ctrl = BV_PXP_AS_CTRL_ALPHA_CTRL__Override;
			else
				ctrl.alpha_ctrl = BV_PXP_AS_CTRL_ALPHA_CTRL__Multiply;

			if (input->g_alpha.alpha_invert)
				ctrl.alpha0_invert = 0x1;
		}

		if (input->g_alpha.color_key_enable) {
			ctrl.enable_colorkey = 1;
		}

		ctrl.alpha = input->g_alpha.global_alpha;
	}

	out_as_ulc.x = out_as_ulc.y = 0;
	if (input->g_alpha.combine_enable) {
		out_as_lrc.x = input->width - 1;
		out_as_lrc.y = input->height - 1;
	} else {
		out_as_lrc.x = output->crop.width - 1;
		out_as_lrc.y = output->crop.height - 1;
	}

	offset = input->crop.y * input->pitch +
		 input->crop.x * (input->bpp >> 3);
	pxp_writel(input->paddr + offset, HW_PXP_AS_BUF);

	pxp_writel(input->pitch, HW_PXP_AS_PITCH);
	pxp_writel(*(uint32_t *)&out_as_ulc, HW_PXP_OUT_AS_ULC);
	pxp_writel(*(uint32_t *)&out_as_lrc, HW_PXP_OUT_AS_LRC);

	pxp_writel(*(uint32_t *)&ctrl, HW_PXP_AS_CTRL);
	pxp_writel(BF_PXP_CTRL_ENABLE_PS_AS_OUT(1) |
		   BF_PXP_CTRL_IRQ_ENABLE(1),
		   HW_PXP_CTRL_SET);

	return 0;
}

static uint32_t pxp_fetch_size_config(struct pxp_pixmap *input)
{
	struct fetch_size total_size;

	memset((void*)&total_size, 0x0, sizeof(total_size));

	total_size.input_total_width  = input->width - 1;
	total_size.input_total_height = input->height - 1;

	return *(uint32_t *)&total_size;
}

static int pxp_fetch_config(struct pxp_pixmap *input,
			    uint32_t fetch_index)
{
	uint8_t  shift_bypass = 1, expand_en = 0;
	uint32_t flags, pitch = 0, offset, UV = 0;
	uint32_t in_fmt, out_fmt;
	uint32_t size_ulc, size_lrc;
	uint32_t fetch_ctrl, total_size;
	uint32_t shift_ctrl, shift_offset = 0;
	struct fetch_shift_width shift_width;

	memset((unsigned int *)&shift_width, 0x0, sizeof(shift_width));
	fetch_ctrl = pxp_fetch_ctrl_config(input, FETCH_MODE_NORMAL);
	size_ulc = pxp_fetch_active_size_ulc(input);
	size_lrc = pxp_fetch_active_size_lrc(input);
	total_size = pxp_fetch_size_config(input);

	if (input->flags) {
		flags = fmt_fetch_to_common(input->format);
		shift_bypass = (flags & FETCH_SHIFT) ? 0 : 1;
		expand_en    = (flags & FETCH_EXPAND) ? 1 : 0;

		if (!shift_bypass) {
			if (expand_en) {
				if (is_yuv(input->format)) {
					in_fmt  = PXP_PIX_FMT_YVU444;
					out_fmt = PXP_PIX_FMT_YUV444;
				} else {
					in_fmt  = PXP_PIX_FMT_ABGR32;
					out_fmt = PXP_PIX_FMT_ARGB32;
				}
			} else {
				in_fmt  = input->format;
				out_fmt = is_yuv(input->format) ?
						 PXP_PIX_FMT_YUV444 :
						 PXP_PIX_FMT_ARGB32;
			}

			shift_offset = pxp_fetch_shift_calc(in_fmt, out_fmt,
							    &shift_width);
		}
	}
	shift_ctrl = pxp_fetch_shift_ctrl_config(input, shift_bypass, expand_en);

	offset = input->crop.y * input->pitch +
		 input->crop.x * (input->bpp >> 3);
	if (is_yuv(input->format) == 2)
		UV = input->paddr + input->width * input->height;

	switch (fetch_index) {
	case PXP_2D_INPUT_FETCH0:
		pitch = __raw_readl(pxp_reg_base + HW_PXP_INPUT_FETCH_PITCH);
		pitch |= pxp_fetch_pitch_config(input, NULL);
		pxp_writel(fetch_ctrl, HW_PXP_INPUT_FETCH_CTRL_CH0);
		pxp_writel(size_ulc, HW_PXP_INPUT_FETCH_ACTIVE_SIZE_ULC_CH0);
		pxp_writel(size_lrc, HW_PXP_INPUT_FETCH_ACTIVE_SIZE_LRC_CH0);
		pxp_writel(total_size, HW_PXP_INPUT_FETCH_SIZE_CH0);
		pxp_writel(shift_ctrl, HW_PXP_INPUT_FETCH_SHIFT_CTRL_CH0);
		pxp_writel(input->paddr + offset, HW_PXP_INPUT_FETCH_ADDR_0_CH0);
		if (UV)
			pxp_writel(UV + offset, HW_PXP_INPUT_FETCH_ADDR_1_CH0);
		pxp_writel(shift_ctrl, HW_PXP_INPUT_FETCH_SHIFT_CTRL_CH0);
		if (shift_offset)
			pxp_writel(*(uint32_t *)&shift_offset, HW_PXP_INPUT_FETCH_SHIFT_OFFSET_CH0);
		pxp_writel(*(uint32_t *)&shift_width, HW_PXP_INPUT_FETCH_SHIFT_WIDTH_CH0);
		break;
	case PXP_2D_INPUT_FETCH1:
		pitch = __raw_readl(pxp_reg_base + HW_PXP_INPUT_FETCH_PITCH);
		pitch |= pxp_fetch_pitch_config(NULL, input);
		pxp_writel(fetch_ctrl, HW_PXP_INPUT_FETCH_CTRL_CH1);
		pxp_writel(size_ulc, HW_PXP_INPUT_FETCH_ACTIVE_SIZE_ULC_CH1);
		pxp_writel(size_lrc, HW_PXP_INPUT_FETCH_ACTIVE_SIZE_LRC_CH1);
		pxp_writel(total_size, HW_PXP_INPUT_FETCH_SIZE_CH1);
		pxp_writel(shift_ctrl, HW_PXP_INPUT_FETCH_SHIFT_CTRL_CH1);
		pxp_writel(input->paddr + offset, HW_PXP_INPUT_FETCH_ADDR_0_CH1);
		if (UV)
			pxp_writel(UV + offset, HW_PXP_INPUT_FETCH_ADDR_1_CH1);
		pxp_writel(shift_ctrl, HW_PXP_INPUT_FETCH_SHIFT_CTRL_CH1);
		if (shift_offset)
			pxp_writel(*(uint32_t *)&shift_offset, HW_PXP_INPUT_FETCH_SHIFT_OFFSET_CH1);
		pxp_writel(*(uint32_t *)&shift_width, HW_PXP_INPUT_FETCH_SHIFT_WIDTH_CH1);
		break;
	default:
		break;
	}

	pxp_writel(pitch, HW_PXP_INPUT_FETCH_PITCH);
	pxp_writel(BF_PXP_CTRL_ENABLE_INPUT_FETCH_STORE(1), HW_PXP_CTRL_SET);

	return 0;
}

static int pxp_csc1_config(struct pxp_pixmap *input,
			   bool is_ycbcr)
{
	BUG_ON(!is_yuv(input->format));

	if (!is_ycbcr) {
		/* YUV -> RGB */
		pxp_writel(0x04030000, HW_PXP_CSC1_COEF0);
		pxp_writel(0x01230208, HW_PXP_CSC1_COEF1);
		pxp_writel(0x076b079c, HW_PXP_CSC1_COEF2);

		return 0;
	}

	/* YCbCr -> RGB */
	pxp_writel(0x84ab01f0, HW_PXP_CSC1_COEF0);
	pxp_writel(0x01980204, HW_PXP_CSC1_COEF1);
	pxp_writel(0x0730079c, HW_PXP_CSC1_COEF2);

	return 0;
}

static int pxp_rotation1_config(struct pxp_pixmap *input)
{
	uint8_t rotate;

	if (input->flip == PXP_H_FLIP)
		pxp_writel(BF_PXP_CTRL_HFLIP1(1), HW_PXP_CTRL_SET);
	else if (input->flip == PXP_V_FLIP)
		pxp_writel(BF_PXP_CTRL_VFLIP1(1), HW_PXP_CTRL_SET);

	rotate = rotate_map(input->rotate);
	pxp_writel(BF_PXP_CTRL_ROTATE1(rotate), HW_PXP_CTRL_SET);

	pxp_writel(BF_PXP_CTRL_ENABLE_ROTATE1(1), HW_PXP_CTRL_SET);

	return 0;
}

static int pxp_rotation0_config(struct pxp_pixmap *input)
{
	uint8_t rotate;

	if (input->flip == PXP_H_FLIP)
		pxp_writel(BF_PXP_CTRL_HFLIP0(1), HW_PXP_CTRL_SET);
	else if (input->flip == PXP_V_FLIP)
		pxp_writel(BF_PXP_CTRL_VFLIP0(1), HW_PXP_CTRL_SET);

	rotate = rotate_map(input->rotate);
	pxp_writel(BF_PXP_CTRL_ROTATE0(rotate), HW_PXP_CTRL_SET);

	pxp_writel(BF_PXP_CTRL_ENABLE_ROTATE0(1), HW_PXP_CTRL_SET);

	return 0;
}

static int pxp_csc2_config(struct pxp_pixmap *output)
{
	if (is_yuv(output->format)) {
		/* RGB -> YUV */
		pxp_writel(0x4, HW_PXP_CSC2_CTRL);
		pxp_writel(0x0096004D, HW_PXP_CSC2_COEF0);
		pxp_writel(0x05DA001D, HW_PXP_CSC2_COEF1);
		pxp_writel(0x007005B6, HW_PXP_CSC2_COEF2);
		pxp_writel(0x057C009E, HW_PXP_CSC2_COEF3);
		pxp_writel(0x000005E6, HW_PXP_CSC2_COEF4);
		pxp_writel(0x00000000, HW_PXP_CSC2_COEF5);
	}

	pxp_writel(BF_PXP_CTRL_ENABLE_CSC2(1), HW_PXP_CTRL_SET);

	return 0;
}

static int pxp_out_config(struct pxp_pixmap *output)
{
	uint32_t offset, UV;
	struct out_ctrl ctrl;
	struct coordinate out_lrc;

	memset((void*)&ctrl, 0x0, sizeof(ctrl));

	ctrl.format = pxp_parse_out_fmt(output->format);
	offset = output->crop.y * output->pitch +
		 output->crop.x * (output->bpp >> 3);

	pxp_writel(*(uint32_t *)&ctrl, HW_PXP_OUT_CTRL);

	pxp_writel(output->paddr, HW_PXP_OUT_BUF);
	if (is_yuv(output->format) == 2) {
		UV = output->paddr + output->width * output->height;
		if ((output->format == PXP_PIX_FMT_NV16) ||
		    (output->format == PXP_PIX_FMT_NV61))
			pxp_writel(UV + offset, HW_PXP_OUT_BUF2);
		else
			pxp_writel(UV + (offset >> 1), HW_PXP_OUT_BUF2);
	}

	if (output->rotate == 90 || output->rotate == 270) {
		out_lrc.y = output->width - 1;
		out_lrc.x = output->height - 1;
	} else {
		out_lrc.x = output->width - 1;
		out_lrc.y = output->height - 1;
	}

	pxp_writel(*(uint32_t *)&out_lrc, HW_PXP_OUT_LRC);

	pxp_writel(output->pitch, HW_PXP_OUT_PITCH);

	/* set global alpha if necessary */
	if (output->g_alpha.global_alpha_enable) {
		pxp_writel(output->g_alpha.global_alpha << 24, HW_PXP_OUT_CTRL_SET);
		pxp_writel(BM_PXP_OUT_CTRL_ALPHA_OUTPUT, HW_PXP_OUT_CTRL_SET);
	}

	pxp_writel(BF_PXP_CTRL_ENABLE_PS_AS_OUT(1) |
		   BF_PXP_CTRL_IRQ_ENABLE(1),
		   HW_PXP_CTRL_SET);

	return 0;
}

static int pxp_store_config(struct pxp_pixmap *output,
			    struct pxp_op_info *op)
{
	uint8_t combine_2ch, flags;
	uint32_t in_fmt, out_fmt, offset, UV = 0;
	uint64_t d_shift = 0;
	struct store_d_mask d_mask[8];
	uint32_t store_ctrl, store_size, store_pitch, shift_ctrl;

	memset((void*)d_mask, 0x0, sizeof(*d_mask) * 8);
	combine_2ch = (output->bpp == 64) ? 1 : 0;
	store_ctrl  = pxp_store_ctrl_config(output, STORE_MODE_NORMAL,
					    op->fill_en, combine_2ch);
	store_size  = pxp_store_size_config(output);
	store_pitch = pxp_store_pitch_config(output, NULL);

	pxp_writel(store_ctrl, HW_PXP_INPUT_STORE_CTRL_CH0);

	if (output->flags) {
		flags = fmt_store_from_common(output->format);
		if (flags == STORE_NOOP)
			shift_ctrl = pxp_store_shift_ctrl_config(output, 1);
		else if (flags & STORE_SHIFT) {
			in_fmt = is_yuv(output->format) ? PXP_PIX_FMT_YUV444 :
							  PXP_PIX_FMT_ARGB32;
			out_fmt = (flags & STORE_SHRINK) ? PXP_PIX_FMT_YVU444 :
							   output->format;
			d_shift = pxp_store_d_shift_calc(in_fmt, out_fmt, d_mask);
			shift_ctrl = pxp_store_shift_ctrl_config(output, 0);
		} else
			shift_ctrl = pxp_store_shift_ctrl_config(output, 0);

		if (flags & STORE_SHIFT) {
			pxp_writel((uint32_t)d_shift, HW_PXP_INPUT_STORE_D_SHIFT_L_CH0);
			/* TODO use only 4 masks */
			pxp_writel(d_mask[0].d_mask_l, HW_PXP_INPUT_STORE_D_MASK0_L_CH0);
			pxp_writel(d_mask[0].d_mask_h, HW_PXP_INPUT_STORE_D_MASK0_H_CH0);
			pxp_writel(d_mask[1].d_mask_l, HW_PXP_INPUT_STORE_D_MASK1_L_CH0);
			pxp_writel(d_mask[1].d_mask_h, HW_PXP_INPUT_STORE_D_MASK1_H_CH0);
			pxp_writel(d_mask[2].d_mask_l, HW_PXP_INPUT_STORE_D_MASK2_L_CH0);
			pxp_writel(d_mask[2].d_mask_h, HW_PXP_INPUT_STORE_D_MASK2_H_CH0);
			pxp_writel(d_mask[3].d_mask_l, HW_PXP_INPUT_STORE_D_MASK3_L_CH0);
			pxp_writel(d_mask[3].d_mask_h, HW_PXP_INPUT_STORE_D_MASK3_H_CH0);
		}
	} else
		shift_ctrl = pxp_store_shift_ctrl_config(output, 1);

	pxp_writel(shift_ctrl, HW_PXP_INPUT_STORE_SHIFT_CTRL_CH0);
	pxp_writel(store_size, HW_PXP_INPUT_STORE_SIZE_CH0);
	pxp_writel(store_pitch, HW_PXP_INPUT_STORE_PITCH);
	if (op->fill_en) {
		uint32_t lrc;

		lrc = (output->width - 1) | ((output->height - 1) << 16);
		pxp_writel(op->fill_data, HW_PXP_INPUT_STORE_FILL_DATA_CH0);

		pxp_writel(0x1, HW_PXP_INPUT_FETCH_CTRL_CH0);
		pxp_writel(0, HW_PXP_INPUT_FETCH_ACTIVE_SIZE_ULC_CH0);
		pxp_writel(lrc, HW_PXP_INPUT_FETCH_ACTIVE_SIZE_LRC_CH0);
	}

	offset = output->crop.y * output->pitch +
		 output->crop.x * (output->bpp >> 3);
	if (is_yuv(output->format == 2)) {
		UV = output->paddr + output->width * output->height;
		pxp_writel(UV + offset, HW_PXP_INPUT_STORE_ADDR_1_CH0);
	}
	pxp_writel(output->paddr + offset, HW_PXP_INPUT_STORE_ADDR_0_CH0);

	pxp_writel(BF_PXP_CTRL_ENABLE_INPUT_FETCH_STORE(1), HW_PXP_CTRL_SET);

	return 0;
}

static int pxp_alpha_config(struct pxp_op_info *op,
			    uint8_t alpha_node)
{
	uint32_t as_ctrl;
	struct pxp_alpha_ctrl alpha_ctrl;
	struct pxp_alpha_info *alpha = &op->alpha_info;
	struct pxp_alpha *s0_alpha, *s1_alpha;

	memset((void*)&alpha_ctrl, 0x0, sizeof(alpha_ctrl));

	if (alpha_blending_version != PXP_ALPHA_BLENDING_V1) {
		if (alpha->alpha_mode == ALPHA_MODE_ROP) {
			switch (alpha_node) {
			case PXP_2D_ALPHA0_S0:
				as_ctrl = __raw_readl(pxp_reg_base + HW_PXP_AS_CTRL);
				as_ctrl |= BF_PXP_AS_CTRL_ALPHA_CTRL(BV_PXP_AS_CTRL_ALPHA_CTRL__ROPs);
				as_ctrl |= BF_PXP_AS_CTRL_ROP(alpha->rop_type);
				pxp_writel(as_ctrl, HW_PXP_AS_CTRL);
				break;
			case PXP_2D_ALPHA1_S0:
				pxp_writel(BM_PXP_ALPHA_B_CTRL_1_ROP_ENABLE |
					   BF_PXP_ALPHA_B_CTRL_1_ROP(alpha->rop_type),
					   HW_PXP_ALPHA_B_CTRL_1);
				pxp_writel(BF_PXP_CTRL_ENABLE_ALPHA_B(1), HW_PXP_CTRL_SET);
				break;
			default:
				break;
			}

			return 0;
		}

		s0_alpha = &alpha->s0_alpha;
		s1_alpha = &alpha->s1_alpha;

		alpha_ctrl.poter_duff_enable = 1;

		alpha_ctrl.s0_s1_factor_mode = s1_alpha->factor_mode;
		alpha_ctrl.s0_global_alpha_mode = s0_alpha->global_alpha_mode;
		alpha_ctrl.s0_alpha_mode = s0_alpha->alpha_mode;
		alpha_ctrl.s0_color_mode = s0_alpha->color_mode;

		alpha_ctrl.s1_s0_factor_mode = s0_alpha->factor_mode;
		alpha_ctrl.s1_global_alpha_mode = s1_alpha->global_alpha_mode;
		alpha_ctrl.s1_alpha_mode = s1_alpha->alpha_mode;
		alpha_ctrl.s1_color_mode = s1_alpha->color_mode;

		alpha_ctrl.s0_global_alpha = s0_alpha->global_alpha_value;
		alpha_ctrl.s1_global_alpha = s1_alpha->global_alpha_value;

		switch (alpha_node) {
		case PXP_2D_ALPHA0_S0:
			pxp_writel(*(uint32_t *)&alpha_ctrl, HW_PXP_ALPHA_A_CTRL);
			break;
		case PXP_2D_ALPHA1_S0:
			pxp_writel(*(uint32_t *)&alpha_ctrl, HW_PXP_ALPHA_B_CTRL);
			pxp_writel(BF_PXP_CTRL_ENABLE_ALPHA_B(1), HW_PXP_CTRL_SET);
			break;
		default:
			break;
		}
	}

	return 0;
}

static void pxp_lut_config(struct pxp_op_info *op)
{
	struct pxp_task_info *task = to_pxp_task_info(op);
	struct pxps *pxp = to_pxp_from_task(task);
	struct pxp_proc_data *proc_data = &pxp->pxp_conf_state.proc_data;
	int lut_op = proc_data->lut_transform;
	u32 reg_val;
	int i;
	bool use_cmap = (lut_op & PXP_LUT_USE_CMAP) ? true : false;
	u8 *cmap = proc_data->lut_map;
	u32 entry_src;
	u32 pix_val;
	u8 entry[4];

	/*
	 * If LUT already configured as needed, return...
	 * Unless CMAP is needed and it has been updated.
	 */
	if ((pxp->lut_state == lut_op) &&
		!(use_cmap && proc_data->lut_map_updated))
		return;

	if (lut_op == PXP_LUT_NONE) {
		__raw_writel(BM_PXP_LUT_CTRL_BYPASS,
			     pxp->base + HW_PXP_LUT_CTRL);
	} else if (((lut_op & PXP_LUT_INVERT) != 0)
		&& ((lut_op & PXP_LUT_BLACK_WHITE) != 0)) {
		/* Fill out LUT table with inverted monochromized values */

		/* clear bypass bit, set lookup mode & out mode */
		__raw_writel(BF_PXP_LUT_CTRL_LOOKUP_MODE
				(BV_PXP_LUT_CTRL_LOOKUP_MODE__DIRECT_Y8) |
				BF_PXP_LUT_CTRL_OUT_MODE
				(BV_PXP_LUT_CTRL_OUT_MODE__Y8),
				pxp->base + HW_PXP_LUT_CTRL);

		/* Initialize LUT address to 0 and set NUM_BYTES to 0 */
		__raw_writel(0, pxp->base + HW_PXP_LUT_ADDR);

		/* LUT address pointer auto-increments after each data write */
		for (pix_val = 0; pix_val < 256; pix_val += 4) {
			for (i = 0; i < 4; i++) {
				entry_src = use_cmap ?
					cmap[pix_val + i] : pix_val + i;
				entry[i] = (entry_src < 0x80) ? 0xFF : 0x00;
			}
			reg_val = (entry[3] << 24) | (entry[2] << 16) |
				(entry[1] << 8) | entry[0];
			__raw_writel(reg_val, pxp->base + HW_PXP_LUT_DATA);
		}
	} else if ((lut_op & PXP_LUT_INVERT) != 0) {
		/* Fill out LUT table with 8-bit inverted values */

		/* clear bypass bit, set lookup mode & out mode */
		__raw_writel(BF_PXP_LUT_CTRL_LOOKUP_MODE
				(BV_PXP_LUT_CTRL_LOOKUP_MODE__DIRECT_Y8) |
				BF_PXP_LUT_CTRL_OUT_MODE
				(BV_PXP_LUT_CTRL_OUT_MODE__Y8),
				pxp->base + HW_PXP_LUT_CTRL);

		/* Initialize LUT address to 0 and set NUM_BYTES to 0 */
		__raw_writel(0, pxp->base + HW_PXP_LUT_ADDR);

		/* LUT address pointer auto-increments after each data write */
		for (pix_val = 0; pix_val < 256; pix_val += 4) {
			for (i = 0; i < 4; i++) {
				entry_src = use_cmap ?
					cmap[pix_val + i] : pix_val + i;
				entry[i] = ~entry_src & 0xFF;
			}
			reg_val = (entry[3] << 24) | (entry[2] << 16) |
				(entry[1] << 8) | entry[0];
			__raw_writel(reg_val, pxp->base + HW_PXP_LUT_DATA);
		}
	} else if ((lut_op & PXP_LUT_BLACK_WHITE) != 0) {
		/* Fill out LUT table with 8-bit monochromized values */

		/* clear bypass bit, set lookup mode & out mode */
		__raw_writel(BF_PXP_LUT_CTRL_LOOKUP_MODE
				(BV_PXP_LUT_CTRL_LOOKUP_MODE__DIRECT_Y8) |
				BF_PXP_LUT_CTRL_OUT_MODE
				(BV_PXP_LUT_CTRL_OUT_MODE__Y8),
				pxp->base + HW_PXP_LUT_CTRL);

		/* Initialize LUT address to 0 and set NUM_BYTES to 0 */
		__raw_writel(0, pxp->base + HW_PXP_LUT_ADDR);

		/* LUT address pointer auto-increments after each data write */
		for (pix_val = 0; pix_val < 256; pix_val += 4) {
			for (i = 0; i < 4; i++) {
				entry_src = use_cmap ?
					cmap[pix_val + i] : pix_val + i;
				entry[i] = (entry_src < 0x80) ? 0x00 : 0xFF;
			}
			reg_val = (entry[3] << 24) | (entry[2] << 16) |
				(entry[1] << 8) | entry[0];
			__raw_writel(reg_val, pxp->base + HW_PXP_LUT_DATA);
		}
	} else if (use_cmap) {
		/* Fill out LUT table using colormap values */

		/* clear bypass bit, set lookup mode & out mode */
		__raw_writel(BF_PXP_LUT_CTRL_LOOKUP_MODE
				(BV_PXP_LUT_CTRL_LOOKUP_MODE__DIRECT_Y8) |
				BF_PXP_LUT_CTRL_OUT_MODE
				(BV_PXP_LUT_CTRL_OUT_MODE__Y8),
				pxp->base + HW_PXP_LUT_CTRL);

		/* Initialize LUT address to 0 and set NUM_BYTES to 0 */
		__raw_writel(0, pxp->base + HW_PXP_LUT_ADDR);

		/* LUT address pointer auto-increments after each data write */
		for (pix_val = 0; pix_val < 256; pix_val += 4) {
			for (i = 0; i < 4; i++)
				entry[i] = cmap[pix_val + i];
			reg_val = (entry[3] << 24) | (entry[2] << 16) |
				(entry[1] << 8) | entry[0];
			__raw_writel(reg_val, pxp->base + HW_PXP_LUT_DATA);
		}
	}

	pxp_writel(BM_PXP_CTRL_ENABLE_ROTATE1 | BM_PXP_CTRL_ENABLE_ROTATE0 |
			BM_PXP_CTRL_ENABLE_CSC2 | BM_PXP_CTRL_ENABLE_LUT,
			HW_PXP_CTRL_SET);

	pxp->lut_state = lut_op;
}

static int pxp_2d_task_config(struct pxp_pixmap *input,
			      struct pxp_pixmap *output,
			      struct pxp_op_info *op,
			      uint32_t nodes_used)
{
	uint8_t position = 0;

	do {
		position = find_next_bit((unsigned long *)&nodes_used, 32, position);
		if (position >= sizeof(uint32_t) * 8)
			break;

		switch (position) {
		case PXP_2D_PS:
			pxp_ps_config(input, output);
			break;
		case PXP_2D_AS:
			pxp_as_config(input, output);
			break;
		case PXP_2D_INPUT_FETCH0:
		case PXP_2D_INPUT_FETCH1:
			pxp_fetch_config(input, position);
			break;
		case PXP_2D_CSC1:
			pxp_csc1_config(input, true);
			break;
		case PXP_2D_ROTATION1:
			pxp_rotation1_config(input);
			break;
		case PXP_2D_ALPHA0_S0:
		case PXP_2D_ALPHA1_S0:
			pxp_alpha_config(op, position);
			break;
		case PXP_2D_ALPHA0_S1:
		case PXP_2D_ALPHA1_S1:
			break;
		case PXP_2D_CSC2:
			pxp_csc2_config(output);
			break;
		case PXP_2D_LUT:
			pxp_lut_config(op);
			break;
		case PXP_2D_ROTATION0:
			pxp_rotation0_config(input);
			break;
		case PXP_2D_OUT:
			pxp_out_config(output);
			break;
		case PXP_2D_INPUT_STORE0:
		case PXP_2D_INPUT_STORE1:
			pxp_store_config(output, op);
			break;
		default:
			break;
		}

		position++;
	} while (1);

	return 0;
}

static void mux_config_helper(struct mux_config *path_ctrl,
			      struct edge_node *enode)
{
	uint32_t mux_val, mux_pos = 0;

	if (enode->mux_used) {
		do {
			mux_pos = find_next_bit((unsigned long *)&enode->mux_used,
						32, mux_pos);
			if (mux_pos >= 16)
				break;

			mux_val = get_mux_val(&enode->muxes, mux_pos);
			pr_debug("%s: mux_pos = %d, mux_val = %d\n",
				  __func__, mux_pos, mux_val);
			set_mux_val(path_ctrl, mux_pos, mux_val);

			mux_pos++;
		} while (1);
	}
}

static void pxp_2d_calc_mux(uint32_t nodes, struct mux_config *path_ctrl)
{
	struct edge_node *enode;
	uint8_t from = 0, to = 0;

	do {
		from = find_next_bit((unsigned long *)&nodes, 32, from);
		if (from >= sizeof(uint32_t) * 8)
			break;

		if (to != 0) {
			enode = adj_list[to].first;
			while (enode) {
				if (enode->adjvex == from) {
					mux_config_helper(path_ctrl, enode);
					break;
				}
				enode = enode->next;
			}
		}

		to = find_next_bit((unsigned long *)&nodes, 32, from + 1);
		if (to >= sizeof(uint32_t) * 8)
			break;

		enode = adj_list[from].first;
		while (enode) {
			if (enode->adjvex == to) {
				mux_config_helper(path_ctrl, enode);
				break;
			}
			enode = enode->next;
		}

		from = to + 1;
	} while (1);
}

static int pxp_2d_op_handler(struct pxps *pxp)
{
	struct mux_config path_ctrl0;
	struct pxp_proc_data *proc_data = &pxp->pxp_conf_state.proc_data;
	struct pxp_task_info *task = &pxp->task;
	struct pxp_op_info *op = &task->op_info;
	struct pxp_pixmap *input, *output, *input_s0, *input_s1;
	uint32_t possible_inputs, possible_outputs;
	uint32_t possible_inputs_s0, possible_inputs_s1;
	uint32_t inputs_filter_s0, inputs_filter_s1;
	uint32_t nodes_used = 0, nodes_in_path;
	uint32_t partial_nodes_used = 0;
	uint32_t nodes_used_s0 = 0, nodes_used_s1 = 0;
	uint32_t nodes_in_path_s0, nodes_in_path_s1;
	uint32_t val;

	output = &task->output[0];
	if (!output->pitch)
		return -EINVAL;

	*(unsigned int*)&path_ctrl0 = 0xffffffff;

reparse:
	switch (task->input_num) {
	case 0:
		/* Fill operation: use input store engine */
		if (is_yuv(output->format) > 1)
			return -EINVAL;

		if (output->bpp > 32)
			return -EINVAL;

		nodes_used = 1 << PXP_2D_INPUT_STORE0;
		pxp_2d_task_config(NULL, output, op, nodes_used);
		break;
	case 1:
		/* No Composite */
		possible_inputs  = (1 << PXP_2D_PS) |
				   (1 << PXP_2D_AS) |
				   (1 << PXP_2D_INPUT_FETCH0);
		possible_outputs = (1 << PXP_2D_OUT) |
				   (1 << PXP_2D_INPUT_STORE0);

		input = &task->input[0];
		if (!input->pitch)
			return -EINVAL;

		if (input->rotate || input->flip) {
			input->flags |= IN_NEED_ROTATE_FLIP;
			output->rotate = input->rotate;
			output->flip = input->flip;
		}

		if (!is_yuv(input->format) != !is_yuv(output->format))
			input->flags |= IN_NEED_CSC;
		else if (input->format != output->format)
			input->flags |= IN_NEED_FMT_UNIFIED;

		if ((input->rotate == 90) || (input->rotate == 270)) {
			if ((input->crop.width != output->crop.height) ||
			    (input->crop.height != output->crop.width))
				input->flags |= IN_NEED_SCALE;
		} else {
			if ((input->crop.width != output->crop.width) ||
			    (input->crop.height != output->crop.height))
				input->flags |= IN_NEED_SCALE;
		}

		if (input->flags) {
			/* only ps has scaling function */
			if ((input->flags & IN_NEED_SCALE) == IN_NEED_SCALE)
				possible_inputs = 1 << PXP_2D_PS;
			output->flags |= (output->bpp < 32) ? OUT_NEED_SHRINK :
							      OUT_NEED_SHIFT;
		}

		filter_possible_inputs(input, &possible_inputs);
		filter_possible_outputs(output, &possible_outputs);

		if (!possible_inputs || !possible_outputs) {
			dev_err(&pxp->pdev->dev, "unsupport 2d operation\n");
			return -EINVAL;
		}

		if (proc_data->lut_transform)
			nodes_used |= (1 << PXP_2D_LUT);

		nodes_in_path = find_best_path(possible_inputs,
					       possible_outputs,
					       input, &nodes_used);

		if (nodes_in_path & (1 << PXP_2D_ROTATION1)) {
			clear_bit(PXP_2D_ROTATION1, (unsigned long *)&nodes_in_path);
			set_bit(PXP_2D_ROTATION0, (unsigned long *)&nodes_in_path);
		}

		if (nodes_used & (1 << PXP_2D_ROTATION1)) {
			clear_bit(PXP_2D_ROTATION1, (unsigned long *)&nodes_used);
			set_bit(PXP_2D_ROTATION0, (unsigned long *)&nodes_used);
		}

		pr_debug("%s: nodes_in_path = 0x%x, nodes_used = 0x%x\n",
			  __func__, nodes_in_path, nodes_used);
		if (!nodes_used) {
			dev_err(&pxp->pdev->dev, "unsupport 2d operation\n");
			return -EINVAL;
		}

		/* If use input fetch0, should use
		 * alpha b instead of alpha a */
		if (nodes_in_path & (1 << PXP_2D_ALPHA0_S0)) {
			if (nodes_in_path & (1 << PXP_2D_INPUT_FETCH0)) {
				clear_bit(PXP_2D_ALPHA0_S0,
					  (unsigned long *)&nodes_in_path);
				set_bit(PXP_2D_ALPHA1_S1,
					  (unsigned long *)&nodes_in_path);
			}
		}

		/* In this case input read in
		 * by input fetch engine
		 */
		if ((nodes_in_path & (1 << PXP_2D_ALPHA1_S1)) ||
		    (nodes_in_path & (1 << PXP_2D_ALPHA1_S0))) {
			memcpy(&task->input[1], input, sizeof(*input));
			if (input->rotate == 90 || input->rotate == 270) {
				uint32_t temp;

				input = &task->input[1];
				input->rotate = 0;
				input->flags  = 0;
				temp = input->width;
				input->width  = input->height;
				input->height = temp;
				input->pitch  = input->width * (input->bpp >> 3);
				temp = input->crop.width;
				input->crop.width  = input->crop.height;
				input->crop.height = temp;
			}

			op->alpha_info.alpha_mode = ALPHA_MODE_ROP;
			/* s0 AND s1 */
			op->alpha_info.rop_type = 0x0;
			task->input_num = 2;
			goto reparse;
		}

		pxp_2d_calc_mux(nodes_in_path, &path_ctrl0);
		pr_debug("%s: path_ctrl0 = 0x%x\n",
			 __func__, *(uint32_t *)&path_ctrl0);
		pxp_2d_task_config(input, output, op, nodes_used);

		if (is_yuv(input->format) && is_yuv(output->format)) {
			val = readl(pxp_reg_base + HW_PXP_CSC1_COEF0);
			val |= (BF_PXP_CSC1_COEF0_YCBCR_MODE(1) |
					BF_PXP_CSC1_COEF0_BYPASS(1));
			pxp_writel(val, HW_PXP_CSC1_COEF0);
		}
		break;
	case 2:
		/* Composite */
		input_s0 = &task->input[0];
		input_s1 = &task->input[1];
		if (!input_s0->pitch || !input_s1->pitch)
			return -EINVAL;

		possible_inputs_s0 = (1 << PXP_2D_PS) |
				     (1 << PXP_2D_INPUT_FETCH0) |
				     (1 << PXP_2D_INPUT_FETCH1);
		possible_inputs_s1 = (1 << PXP_2D_AS) |
				     (1 << PXP_2D_INPUT_FETCH0);
		possible_outputs   = (1 << PXP_2D_OUT) |
				     (1 << PXP_2D_INPUT_STORE0);

		if (input_s0->rotate || input_s0->flip) {
			input_s0->flags |= IN_NEED_ROTATE_FLIP;
			output->rotate = input_s0->rotate;
			output->flip = input_s0->flip;
		}
		if (input_s1->rotate || input_s1->flip) {
			input_s1->flags |= IN_NEED_ROTATE_FLIP;
			clear_bit(PXP_2D_AS,
				  (unsigned long *)&possible_inputs_s1);
		}

		if (is_yuv(input_s0->format) && is_yuv(input_s1->format))
			return -EINVAL;

		if (is_yuv(input_s0->format)){
			/* need do yuv -> rgb conversion by csc1 */
			possible_inputs_s0 = 1 << PXP_2D_PS;
			input_s0->flags |= IN_NEED_CSC;
		} else if (is_yuv(input_s1->format)) {
			possible_inputs_s1 = 1 << PXP_2D_PS;
			input_s1->flags |= IN_NEED_CSC;
		}

		filter_possible_inputs(input_s0, &possible_inputs_s0);
		filter_possible_inputs(input_s1, &possible_inputs_s1);

		if (!possible_inputs_s0 || !possible_inputs_s0)
			return -EINVAL;

		filter_possible_outputs(output, &possible_outputs);
		if (!possible_outputs)
			return -EINVAL;

		pr_debug("%s: poss_s0 = 0x%x, poss_s1 = 0x%x, poss_out = 0x%x\n",
			 __func__, possible_inputs_s0, possible_inputs_s1, possible_outputs);

		inputs_filter_s0 = possible_inputs_s0;
		inputs_filter_s1 = possible_inputs_s1;

		/* Using alpha0, possible cases:
		 * 1. PS --> S0, AS --> S1;
		 */
		if (possible_inputs_s1 & (1 << PXP_2D_AS)) {
			clear_bit(PXP_2D_INPUT_FETCH0,
				  (unsigned long *)&possible_inputs_s0);
			clear_bit(PXP_2D_INPUT_FETCH1,
				  (unsigned long *)&possible_inputs_s0);
			clear_bit(PXP_2D_INPUT_STORE0,
				  (unsigned long *)&possible_outputs);

			if (!possible_inputs_s0 || !possible_outputs)
				goto alpha1;

			nodes_in_path_s0 = find_best_path(possible_inputs_s0,
							  1 << PXP_2D_ALPHA0_S0,
							  input_s0,
							  &partial_nodes_used);
			if (!nodes_in_path_s0)
				goto alpha1;

			nodes_used_s0 |= partial_nodes_used;
			partial_nodes_used = 0;

			if (is_yuv(output->format))
				set_bit(PXP_2D_CSC2,
					(unsigned long *)&partial_nodes_used);
			if (output->rotate || output->flip)
				set_bit(PXP_2D_ROTATION0,
					(unsigned long *)&partial_nodes_used);

			nodes_in_path_s0 |= find_best_path(1 << PXP_2D_ALPHA0_S0,
							   possible_outputs,
							   input_s0,
							   &partial_nodes_used);
			if (!(nodes_in_path_s0 & possible_outputs))
				goto alpha1;
			nodes_used_s0 |= partial_nodes_used;

			possible_inputs_s1 = (1 << PXP_2D_AS);
			nodes_in_path_s1 = find_best_path(possible_inputs_s1,
							  1 << PXP_2D_ALPHA0_S1,
							  input_s1,
							  &nodes_used_s1);
			if (!nodes_in_path_s1)
				goto alpha1;

			goto config;
		}
alpha1:
		partial_nodes_used = 0;
		possible_inputs_s0 = inputs_filter_s0;
		possible_inputs_s1 = inputs_filter_s1;

		/* Using alpha1, possible cases:
		 * 1. FETCH1 --> S0, FETCH0 --> S1;
		 */
		clear_bit(PXP_2D_PS,
			  (unsigned long *)&possible_inputs_s0);
		clear_bit(PXP_2D_INPUT_FETCH0,
			  (unsigned long *)&possible_inputs_s0);
		clear_bit(PXP_2D_OUT,
			  (unsigned long *)&possible_outputs);

		if (!possible_inputs_s0 || !possible_outputs)
			return -EINVAL;

		nodes_in_path_s0 = find_best_path(possible_inputs_s0,
						  1 << PXP_2D_ALPHA1_S0,
						  input_s0,
						  &partial_nodes_used);
		pr_debug("%s: nodes_in_path_s0 = 0x%x\n", __func__, nodes_in_path_s0);
		BUG_ON(!nodes_in_path_s0);

		nodes_used_s0 |= partial_nodes_used;
		if ((nodes_used_s0 & (1 << PXP_2D_INPUT_FETCH0)) ||
		    (nodes_used_s0 & (1 << PXP_2D_INPUT_FETCH1)))
			clear_bit(PXP_2D_OUT, (unsigned long *)&possible_outputs);
		else
			clear_bit(PXP_2D_INPUT_STORE0,
				  (unsigned long *)&possible_outputs);
		partial_nodes_used = 0;

		if (is_yuv(output->format))
			set_bit(PXP_2D_CSC2,
				(unsigned long *)&partial_nodes_used);
		if (output->rotate || output->flip)
			set_bit(PXP_2D_ROTATION0,
				(unsigned long *)&partial_nodes_used);

		nodes_in_path_s0 |= find_best_path(1 << PXP_2D_ALPHA1_S0,
						   possible_outputs,
						   input_s0,
						   &partial_nodes_used);
		BUG_ON(!(nodes_in_path_s0 & possible_outputs));
		nodes_used_s0 |= partial_nodes_used;
		pr_debug("%s: nodes_in_path_s0 = 0x%x, nodes_used_s0 = 0x%x\n",
			 __func__, nodes_in_path_s0, nodes_used_s0);

		clear_bit(PXP_2D_AS,
			  (unsigned long *)&possible_inputs_s1);
		BUG_ON(!possible_inputs_s1);

		nodes_in_path_s1 = find_best_path(possible_inputs_s1,
						  1 << PXP_2D_ALPHA1_S1,
						  input_s1,
						  &nodes_used_s1);
		pr_debug("%s: poss_s1 = 0x%x, nodes_used_s1 = 0x%x\n",
			 __func__, possible_inputs_s1, nodes_used_s1);
		BUG_ON(!nodes_in_path_s1);
		/* To workaround an IC bug */
		path_ctrl0.mux4_sel = 0x0;
config:
		if (nodes_in_path_s0 & (1 << PXP_2D_ROTATION1)) {
			clear_bit(PXP_2D_ROTATION1, (unsigned long *)&nodes_in_path_s0);
			set_bit(PXP_2D_ROTATION0, (unsigned long *)&nodes_in_path_s0);
		}

		pr_debug("%s: nodes_in_path_s0 = 0x%x, nodes_used_s0 = 0x%x, nodes_in_path_s1 = 0x%x, nodes_used_s1 = 0x%x\n",
			 __func__, nodes_in_path_s0, nodes_used_s0, nodes_in_path_s1, nodes_used_s1);
		pxp_2d_calc_mux(nodes_in_path_s0, &path_ctrl0);
		pxp_2d_calc_mux(nodes_in_path_s1, &path_ctrl0);

		pr_debug("%s: s0 paddr = 0x%x, s1 paddr = 0x%x, out paddr = 0x%x\n",
			 __func__, input_s0->paddr, input_s1->paddr, output->paddr);

		if (nodes_used_s0 & (1 << PXP_2D_ROTATION1)) {
			clear_bit(PXP_2D_ROTATION1, (unsigned long *)&nodes_used_s0);
			set_bit(PXP_2D_ROTATION0, (unsigned long *)&nodes_used_s0);
		}

		pxp_2d_task_config(input_s0, output, op, nodes_used_s0);
		pxp_2d_task_config(input_s1, output, op, nodes_used_s1);
		break;
	default:
		break;
	}

	__raw_writel(proc_data->bgcolor,
			 pxp->base + HW_PXP_PS_BACKGROUND_0);
	pxp_set_colorkey(pxp);

	if (proc_data->lut_transform && pxp_is_v3(pxp))
		set_mux(&path_ctrl0);

	pr_debug("%s: path_ctrl0 = 0x%x\n",
		 __func__, *(uint32_t *)&path_ctrl0);
	pxp_writel(*(uint32_t *)&path_ctrl0, HW_PXP_DATA_PATH_CTRL0);

	return 0;
}

/**
 * pxp_config() - configure PxP for a processing task
 * @pxps:	PXP context.
 * @pxp_chan:	PXP channel.
 * @return:	0 on success or negative error code on failure.
 */
static int pxp_config(struct pxps *pxp, struct pxp_channel *pxp_chan)
{
	int ret = 0;
	struct pxp_task_info *task = &pxp->task;
	struct pxp_op_info *op = &task->op_info;
	struct pxp_config_data *pxp_conf_data = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf_data->proc_data;

	switch (op->op_type) {
	case PXP_OP_TYPE_2D:
		pxp_writel(0xffffffff, HW_PXP_OUT_AS_ULC);
		pxp_writel(0x0, HW_PXP_OUT_AS_LRC);
		pxp_writel(0xffffffff, HW_PXP_OUT_PS_ULC);
		pxp_writel(0x0, HW_PXP_OUT_PS_LRC);
		pxp_writel(0x0, HW_PXP_INPUT_FETCH_PITCH);
		pxp_writel(0x40000000, HW_PXP_CSC1_COEF0);
		ret = pxp_2d_op_handler(pxp);
		break;
	case PXP_OP_TYPE_DITHER:
		pxp_dithering_process(pxp);
		if (pxp_is_v3p(pxp)) {
			__raw_writel(
				BM_PXP_CTRL_ENABLE         |
				BM_PXP_CTRL_ENABLE_DITHER  |
				BM_PXP_CTRL_ENABLE_CSC2    |
				BM_PXP_CTRL_ENABLE_LUT     |
				BM_PXP_CTRL_ENABLE_ROTATE0 |
				BM_PXP_CTRL_ENABLE_PS_AS_OUT,
				pxp->base + HW_PXP_CTRL_SET);
			return 0;
		}
		break;
	case PXP_OP_TYPE_WFE_A:
		pxp_luts_deactivate(pxp, proc_data->lut_sels);

		if (proc_data->lut_cleanup == 0) {
			/* We should enable histogram in standard mode
			 * in wfe_a processing for waveform mode selection
			 */
			pxp_histogram_enable(pxp, pxp_conf_data->wfe_a_fetch_param[0].width,
					pxp_conf_data->wfe_a_fetch_param[0].height);

			pxp_luts_activate(pxp, (u64)proc_data->lut_status_1 |
					((u64)proc_data->lut_status_2 << 32));

			/* collision detection should be always enable in standard mode */
			pxp_collision_detection_enable(pxp, pxp_conf_data->wfe_a_fetch_param[0].width,
					pxp_conf_data->wfe_a_fetch_param[0].height);
		}

		if (pxp->devdata && pxp->devdata->pxp_wfe_a_configure)
			pxp->devdata->pxp_wfe_a_configure(pxp);
		if (pxp->devdata && pxp->devdata->pxp_wfe_a_process)
			pxp->devdata->pxp_wfe_a_process(pxp);
		break;
	case PXP_OP_TYPE_WFE_B:
		pxp_wfe_b_configure(pxp);
		pxp_wfe_b_process(pxp);
		break;
	default:
		/* Unsupport */
		ret = -EINVAL;
		pr_err("Invalid pxp operation type passed\n");
		break;
	}

	return ret;
}

static void pxp_clk_enable(struct pxps *pxp)
{
	mutex_lock(&pxp->clk_mutex);

	if (pxp->clk_stat == CLK_STAT_ON) {
		mutex_unlock(&pxp->clk_mutex);
		return;
	}

	pm_runtime_get_sync(pxp->dev);

	clk_prepare_enable(pxp->ipg_clk);
	clk_prepare_enable(pxp->axi_clk);
	pxp->clk_stat = CLK_STAT_ON;

	mutex_unlock(&pxp->clk_mutex);
}

static void pxp_clk_disable(struct pxps *pxp)
{
	unsigned long flags;

	mutex_lock(&pxp->clk_mutex);

	if (pxp->clk_stat == CLK_STAT_OFF) {
		mutex_unlock(&pxp->clk_mutex);
		return;
	}

	spin_lock_irqsave(&pxp->lock, flags);
	if ((pxp->pxp_ongoing == 0) && list_empty(&head)) {
		spin_unlock_irqrestore(&pxp->lock, flags);
		clk_disable_unprepare(pxp->ipg_clk);
		clk_disable_unprepare(pxp->axi_clk);
		pxp->clk_stat = CLK_STAT_OFF;
	} else
		spin_unlock_irqrestore(&pxp->lock, flags);

	pm_runtime_put_sync_suspend(pxp->dev);

	mutex_unlock(&pxp->clk_mutex);
}

static inline void clkoff_callback(struct work_struct *w)
{
	struct pxps *pxp = container_of(w, struct pxps, work);

	pxp_clk_disable(pxp);
}

static void pxp_clkoff_timer(unsigned long arg)
{
	struct pxps *pxp = (struct pxps *)arg;

	if ((pxp->pxp_ongoing == 0) && list_empty(&head))
		schedule_work(&pxp->work);
	else
		mod_timer(&pxp->clk_timer,
			  jiffies + msecs_to_jiffies(timeout_in_ms));
}

static struct pxp_tx_desc *pxpdma_first_queued(struct pxp_channel *pxp_chan)
{
	return list_entry(pxp_chan->queue.next, struct pxp_tx_desc, list);
}

static int convert_param_to_pixmap(struct pxp_pixmap *pixmap,
				   struct pxp_layer_param *param)
{
	if (!param->width || !param->height)
		return -EINVAL;

	pixmap->width  = param->width;
	pixmap->height = param->height;
	pixmap->format = param->pixel_fmt;
	pixmap->paddr  = param->paddr;
	pixmap->bpp    = get_bpp_from_fmt(pixmap->format);

	if (pxp_legacy) {
		pixmap->pitch = (param->stride) ? (param->stride * pixmap->bpp >> 3) :
						(param->width * pixmap->bpp >> 3);
	} else {
		if (!param->stride || (param->stride == param->width))
			pixmap->pitch  = param->width * pixmap->bpp >> 3;
		else
			pixmap->pitch = param->stride;
	}

	pixmap->crop.x = param->crop.left;
	pixmap->crop.y = param->crop.top;
	pixmap->crop.width  = param->crop.width;
	pixmap->crop.height = param->crop.height;

	pixmap->g_alpha.color_key_enable = param->color_key_enable;
	pixmap->g_alpha.combine_enable = param->combine_enable;
	pixmap->g_alpha.global_alpha_enable = param->global_alpha_enable;
	pixmap->g_alpha.global_override = param->global_override;
	pixmap->g_alpha.global_alpha = param->global_alpha;
	pixmap->g_alpha.alpha_invert = param->alpha_invert;
	pixmap->g_alpha.local_alpha_enable = param->local_alpha_enable;
	pixmap->g_alpha.comp_mask = param->comp_mask;

	return 0;
}

/* called with pxp_chan->lock held */
static void __pxpdma_dostart(struct pxp_channel *pxp_chan)
{
	struct pxp_dma *pxp_dma = to_pxp_dma(pxp_chan->dma_chan.device);
	struct pxps *pxp = to_pxp(pxp_dma);
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &config_data->proc_data;
	struct pxp_tx_desc *desc;
	struct pxp_tx_desc *child;
	struct pxp_task_info *task = &pxp->task;
	struct pxp_op_info *op = &task->op_info;
	struct pxp_alpha_info *alpha = &op->alpha_info;
	struct pxp_layer_param *param = NULL;
	struct pxp_pixmap *input, *output;
	int i = 0, ret;
	bool combine_enable = false;

	memset(&pxp->pxp_conf_state.s0_param, 0,  sizeof(struct pxp_layer_param));
	memset(&pxp->pxp_conf_state.out_param, 0,  sizeof(struct pxp_layer_param));
	memset(pxp->pxp_conf_state.ol_param, 0,  sizeof(struct pxp_layer_param));
	memset(&pxp->pxp_conf_state.proc_data, 0,  sizeof(struct pxp_proc_data));

	memset(task, 0, sizeof(*task));
	/* S0 */
	desc = list_first_entry(&head, struct pxp_tx_desc, list);
	memcpy(&pxp->pxp_conf_state.s0_param,
	       &desc->layer_param.s0_param, sizeof(struct pxp_layer_param));
	memcpy(&pxp->pxp_conf_state.proc_data,
	       &desc->proc_data, sizeof(struct pxp_proc_data));

	if (proc_data->combine_enable)
		alpha_blending_version = PXP_ALPHA_BLENDING_V2;
	else
		alpha_blending_version = PXP_ALPHA_BLENDING_NONE;

	pxp_legacy = (proc_data->pxp_legacy) ? true : false;

	/* Save PxP configuration */
	list_for_each_entry(child, &desc->tx_list, list) {
		if (i == 0) {	/* Output */
			memcpy(&pxp->pxp_conf_state.out_param,
			       &child->layer_param.out_param,
			       sizeof(struct pxp_layer_param));
		} else if (i == 1) {	/* Overlay */
			memcpy(&pxp->pxp_conf_state.ol_param[i - 1],
			       &child->layer_param.ol_param,
			       sizeof(struct pxp_layer_param));
			if (pxp->pxp_conf_state.ol_param[i - 1].width != 0 &&
				pxp->pxp_conf_state.ol_param[i - 1].height != 0) {
				if (pxp->pxp_conf_state.ol_param[i - 1].combine_enable)
					alpha_blending_version = PXP_ALPHA_BLENDING_V1;
			}
		}

		if (proc_data->engine_enable & PXP_ENABLE_DITHER) {
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_DITHER_FETCH0)
				memcpy(&pxp->pxp_conf_state.dither_fetch_param[0],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_DITHER_FETCH1)
				memcpy(&pxp->pxp_conf_state.dither_fetch_param[1],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_DITHER_STORE0)
				memcpy(&pxp->pxp_conf_state.dither_store_param[0],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_DITHER_STORE1)
				memcpy(&pxp->pxp_conf_state.dither_store_param[1],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			op->op_type = PXP_OP_TYPE_DITHER;
		}

		if (proc_data->engine_enable & PXP_ENABLE_WFE_A) {
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_A_FETCH0)
				memcpy(&pxp->pxp_conf_state.wfe_a_fetch_param[0],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_A_FETCH1)
				memcpy(&pxp->pxp_conf_state.wfe_a_fetch_param[1],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_A_STORE0)
				memcpy(&pxp->pxp_conf_state.wfe_a_store_param[0],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_A_STORE1)
				memcpy(&pxp->pxp_conf_state.wfe_a_store_param[1],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			op->op_type = PXP_OP_TYPE_WFE_A;
		}

		if (proc_data->engine_enable & PXP_ENABLE_WFE_B) {
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_B_FETCH0)
				memcpy(&pxp->pxp_conf_state.wfe_b_fetch_param[0],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_B_FETCH1)
				memcpy(&pxp->pxp_conf_state.wfe_b_fetch_param[1],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_B_STORE0)
				memcpy(&pxp->pxp_conf_state.wfe_b_store_param[0],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			if (child->layer_param.processing_param.flag & PXP_BUF_FLAG_WFE_B_STORE1)
				memcpy(&pxp->pxp_conf_state.wfe_b_store_param[1],
				       &child->layer_param.processing_param,
				       sizeof(struct pxp_layer_param));
			op->op_type = PXP_OP_TYPE_WFE_B;
		}

		i++;
	}

	if (!op->op_type) {
		op->op_type = PXP_OP_TYPE_2D;

		if ((alpha_blending_version == PXP_ALPHA_BLENDING_V1) ||
			(alpha_blending_version == PXP_ALPHA_BLENDING_V2))
			combine_enable = true;

		if (combine_enable)
			task->input_num = 2;
		else if (proc_data->fill_en)
			task->input_num = 0;
		else
			task->input_num = 1;

		output = &task->output[0];
		switch (task->input_num) {
		case 0:
			op->fill_en   = 1;
			op->fill_data = proc_data->bgcolor;
			break;
		case 1:
			param = &pxp->pxp_conf_state.s0_param;
			input = &task->input[0];

			ret = convert_param_to_pixmap(input, param);
			if (ret < 0) {
				param = &pxp->pxp_conf_state.ol_param[0];
				ret = convert_param_to_pixmap(input, param);
				BUG_ON(ret < 0);
			} else {
				input->crop.x = proc_data->srect.left;
				input->crop.y = proc_data->srect.top;
				input->crop.width  = proc_data->srect.width;
				input->crop.height = proc_data->srect.height;
			}

			input->rotate = proc_data->rotate;
			input->flip = (proc_data->hflip) ? PXP_H_FLIP :
				      (proc_data->vflip) ? PXP_V_FLIP : 0;
			break;
		case 2:
			/* s0 */
			param = &pxp->pxp_conf_state.s0_param;
			input = &task->input[0];

			ret = convert_param_to_pixmap(input, param);
			BUG_ON(ret < 0);
			input->crop.x = proc_data->srect.left;
			input->crop.y = proc_data->srect.top;
			input->crop.width  = proc_data->srect.width;
			input->crop.height = proc_data->srect.height;
			alpha->s0_alpha = param->alpha;

			input->rotate = proc_data->rotate;
			input->flip   = (proc_data->hflip) ? PXP_H_FLIP :
					(proc_data->vflip) ? PXP_V_FLIP : 0;

			/* overlay */
			param = &pxp->pxp_conf_state.ol_param[0];
			input = &task->input[1];

			ret = convert_param_to_pixmap(input, param);
			BUG_ON(ret < 0);
			alpha->s1_alpha = param->alpha;
			alpha->alpha_mode = proc_data->alpha_mode;
			break;
		}

		param = &pxp->pxp_conf_state.out_param;
		ret = convert_param_to_pixmap(output, param);
		BUG_ON(ret < 0);

		output->crop.x = proc_data->drect.left;
		output->crop.y = proc_data->drect.top;
		output->crop.width  = proc_data->drect.width;
		output->crop.height = proc_data->drect.height;
	}

	pr_debug("%s:%d S0 w/h %d/%d paddr %08x\n", __func__, __LINE__,
		 pxp->pxp_conf_state.s0_param.width,
		 pxp->pxp_conf_state.s0_param.height,
		 pxp->pxp_conf_state.s0_param.paddr);
	pr_debug("%s:%d S0 crop (top, left)=(%d, %d), (width, height)=(%d, %d)\n",
		__func__, __LINE__,
		pxp->pxp_conf_state.s0_param.crop.top,
		pxp->pxp_conf_state.s0_param.crop.left,
		pxp->pxp_conf_state.s0_param.crop.width,
		pxp->pxp_conf_state.s0_param.crop.height);
	pr_debug("%s:%d OUT w/h %d/%d paddr %08x\n", __func__, __LINE__,
		 pxp->pxp_conf_state.out_param.width,
		 pxp->pxp_conf_state.out_param.height,
		 pxp->pxp_conf_state.out_param.paddr);
}

static int pxpdma_dostart_work(struct pxps *pxp)
{
	int ret;
	struct pxp_channel *pxp_chan = NULL;
	unsigned long flags;
	dma_async_tx_callback callback;
	void *callback_param;
	struct pxp_tx_desc *desc = NULL;
	struct pxp_tx_desc *child, *_child;
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &config_data->proc_data;

	spin_lock_irqsave(&pxp->lock, flags);

	desc = list_entry(head.next, struct pxp_tx_desc, list);
	pxp_chan = to_pxp_channel(desc->txd.chan);

	__pxpdma_dostart(pxp_chan);

	/* Configure PxP */
	ret = pxp_config(pxp, pxp_chan);
	if (ret) {
		callback = desc->txd.callback;
		callback_param = desc->txd.callback_param;

		callback(callback_param);

		/* Unsupport operation */
		list_for_each_entry_safe(child, _child, &desc->tx_list, list) {
			list_del_init(&child->list);
			kmem_cache_free(tx_desc_cache, (void *)child);
		}
		list_del_init(&desc->list);
		kmem_cache_free(tx_desc_cache, (void *)desc);

		spin_unlock_irqrestore(&pxp->lock, flags);
		return -EINVAL;
	}

	if (proc_data->working_mode & PXP_MODE_STANDARD) {
		if(!pxp_is_v3p(pxp) || !(proc_data->engine_enable & PXP_ENABLE_DITHER))
			pxp_start2(pxp);
	} else
		pxp_start(pxp);

	spin_unlock_irqrestore(&pxp->lock, flags);

	return 0;
}

static void pxpdma_dequeue(struct pxp_channel *pxp_chan, struct pxps *pxp)
{
	unsigned long flags;
	struct pxp_tx_desc *desc = NULL;

	do {
		desc = pxpdma_first_queued(pxp_chan);
		spin_lock_irqsave(&pxp->lock, flags);
		list_move_tail(&desc->list, &head);
		spin_unlock_irqrestore(&pxp->lock, flags);
	} while (!list_empty(&pxp_chan->queue));
}

static dma_cookie_t pxp_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct pxp_tx_desc *desc = to_tx_desc(tx);
	struct pxp_channel *pxp_chan = to_pxp_channel(tx->chan);
	dma_cookie_t cookie;

	dev_dbg(&pxp_chan->dma_chan.dev->device, "received TX\n");

	/* pxp_chan->lock can be taken under ichan->lock, but not v.v. */
	spin_lock(&pxp_chan->lock);

	cookie = pxp_chan->dma_chan.cookie;

	if (++cookie < 0)
		cookie = 1;

	/* from dmaengine.h: "last cookie value returned to client" */
	pxp_chan->dma_chan.cookie = cookie;
	tx->cookie = cookie;

	/* Here we add the tx descriptor to our PxP task queue. */
	list_add_tail(&desc->list, &pxp_chan->queue);

	spin_unlock(&pxp_chan->lock);

	dev_dbg(&pxp_chan->dma_chan.dev->device, "done TX\n");

	return cookie;
}

/**
 * pxp_init_channel() - initialize a PXP channel.
 * @pxp_dma:   PXP DMA context.
 * @pchan:  pointer to the channel object.
 * @return      0 on success or negative error code on failure.
 */
static int pxp_init_channel(struct pxp_dma *pxp_dma,
			    struct pxp_channel *pxp_chan)
{
	int ret = 0;

	/*
	 * We are using _virtual_ channel here.
	 * Each channel contains all parameters of corresponding layers
	 * for one transaction; each layer is represented as one descriptor
	 * (i.e., pxp_tx_desc) here.
	 */

	INIT_LIST_HEAD(&pxp_chan->queue);

	return ret;
}

static irqreturn_t pxp_irq(int irq, void *dev_id)
{
	struct pxps *pxp = dev_id;
	struct pxp_channel *pxp_chan;
	struct pxp_tx_desc *desc;
	struct pxp_tx_desc *child, *_child;
	dma_async_tx_callback callback;
	void *callback_param;
	unsigned long flags;
	u32 hist_status;
	int pxp_irq_status = 0;

	dump_pxp_reg(pxp);

	if (__raw_readl(pxp->base + HW_PXP_STAT) & BM_PXP_STAT_IRQ0)
		__raw_writel(BM_PXP_STAT_IRQ0, pxp->base + HW_PXP_STAT_CLR);
	else {
		int irq_clr = 0;

		pxp_irq_status = __raw_readl(pxp->base + HW_PXP_IRQ);
		BUG_ON(!pxp_irq_status);

		if (pxp_irq_status & BM_PXP_IRQ_FIRST_CH0_PREFETCH_IRQ)
			irq_clr |= BM_PXP_IRQ_FIRST_CH0_PREFETCH_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_FIRST_CH1_PREFETCH_IRQ)
			irq_clr |= BM_PXP_IRQ_FIRST_CH1_PREFETCH_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_FIRST_CH0_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_FIRST_CH0_STORE_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_FIRST_CH1_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_FIRST_CH1_STORE_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_FIRST_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_FIRST_STORE_IRQ;

		if (pxp_irq_status & BM_PXP_IRQ_WFE_B_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_WFE_B_STORE_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_WFE_A_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_WFE_A_STORE_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_DITHER_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_DITHER_STORE_IRQ;

		if (pxp_irq_status & BM_PXP_IRQ_WFE_A_CH0_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_WFE_A_CH0_STORE_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_WFE_A_CH1_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_WFE_A_CH1_STORE_IRQ;

		if (pxp_irq_status & BM_PXP_IRQ_WFE_B_CH0_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_WFE_B_CH0_STORE_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_WFE_B_CH1_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_WFE_B_CH1_STORE_IRQ;

		if (pxp_irq_status & BM_PXP_IRQ_DITHER_CH0_PREFETCH_IRQ)
			irq_clr |= BM_PXP_IRQ_DITHER_CH0_PREFETCH_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_DITHER_CH1_PREFETCH_IRQ)
			irq_clr |= BM_PXP_IRQ_DITHER_CH1_PREFETCH_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_DITHER_CH0_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_DITHER_CH0_STORE_IRQ;
		if (pxp_irq_status & BM_PXP_IRQ_DITHER_CH1_STORE_IRQ)
			irq_clr |= BM_PXP_IRQ_DITHER_CH1_STORE_IRQ;
		/*XXX other irqs status clear should be added below */

		__raw_writel(irq_clr, pxp->base + HW_PXP_IRQ_CLR);

		pxp_writel(BM_PXP_CTRL_ENABLE, HW_PXP_CTRL_CLR);
	}
	pxp_collision_status_report(pxp, &col_info);
	pxp_histogram_status_report(pxp, &hist_status);
	/*XXX before a new update operation, we should
	 * always clear all the collision information
	 */
	pxp_collision_detection_disable(pxp);
	pxp_histogram_disable(pxp);

	pxp_writel(0x0, HW_PXP_CTRL);
	pxp_soft_reset(pxp);
	if (pxp->devdata && pxp->devdata->pxp_data_path_config)
		pxp->devdata->pxp_data_path_config(pxp);
	__raw_writel(0xffff, pxp->base + HW_PXP_IRQ_MASK);

	spin_lock_irqsave(&pxp->lock, flags);
	if (list_empty(&head)) {
		pxp->pxp_ongoing = 0;
		spin_unlock_irqrestore(&pxp->lock, flags);
		return IRQ_NONE;
	}

	/* Get descriptor and call callback */
	desc = list_entry(head.next, struct pxp_tx_desc, list);
	pxp_chan = to_pxp_channel(desc->txd.chan);

	pxp_chan->completed = desc->txd.cookie;

	callback = desc->txd.callback;
	callback_param = desc->txd.callback_param;

	/* Send histogram status back to caller */
	desc->hist_status = hist_status;

	if ((desc->txd.flags & DMA_PREP_INTERRUPT) && callback)
		callback(callback_param);

	pxp_chan->status = PXP_CHANNEL_INITIALIZED;

	list_for_each_entry_safe(child, _child, &desc->tx_list, list) {
		list_del_init(&child->list);
		kmem_cache_free(tx_desc_cache, (void *)child);
	}
	list_del_init(&desc->list);
	kmem_cache_free(tx_desc_cache, (void *)desc);

	complete(&pxp->complete);
	pxp->pxp_ongoing = 0;
	mod_timer(&pxp->clk_timer, jiffies + msecs_to_jiffies(timeout_in_ms));

	spin_unlock_irqrestore(&pxp->lock, flags);

	return IRQ_HANDLED;
}

/* allocate/free dma tx descriptor dynamically*/
static struct pxp_tx_desc *pxpdma_desc_alloc(struct pxp_channel *pxp_chan)
{
	struct pxp_tx_desc *desc = NULL;
	struct dma_async_tx_descriptor *txd = NULL;

	desc = kmem_cache_alloc(tx_desc_cache, GFP_KERNEL | __GFP_ZERO);
	if (desc == NULL)
		return NULL;

	INIT_LIST_HEAD(&desc->list);
	INIT_LIST_HEAD(&desc->tx_list);
	txd = &desc->txd;
	dma_async_tx_descriptor_init(txd, &pxp_chan->dma_chan);
	txd->tx_submit = pxp_tx_submit;

	return desc;
}


/* Allocate and initialise a transfer descriptor. */
static struct dma_async_tx_descriptor *pxp_prep_slave_sg(struct dma_chan *chan,
							 struct scatterlist
							 *sgl,
							 unsigned int sg_len,
							 enum
							 dma_transfer_direction
							 direction,
							 unsigned long tx_flags,
							 void *context)
{
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	struct pxp_dma *pxp_dma = to_pxp_dma(chan->device);
	struct pxps *pxp = to_pxp(pxp_dma);
	struct pxp_tx_desc *pos = NULL, *next = NULL;
	struct pxp_tx_desc *desc = NULL;
	struct pxp_tx_desc *first = NULL, *prev = NULL;
	struct scatterlist *sg;
	dma_addr_t phys_addr;
	int i;

	if (direction != DMA_DEV_TO_MEM && direction != DMA_MEM_TO_DEV) {
		dev_err(chan->device->dev, "Invalid DMA direction %d!\n",
			direction);
		return NULL;
	}

	if (unlikely(sg_len < 2))
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		desc = pxpdma_desc_alloc(pxp_chan);
		if (!desc) {
			dev_err(chan->device->dev, "no enough memory to allocate tx descriptor\n");

			if (first) {
				list_for_each_entry_safe(pos, next, &first->tx_list, list) {
					list_del_init(&pos->list);
					kmem_cache_free(tx_desc_cache, (void*)pos);
				}
				list_del_init(&first->list);
				kmem_cache_free(tx_desc_cache, (void*)first);
			}

			return NULL;
		}

		phys_addr = sg_dma_address(sg);

		if (!first) {
			first = desc;

			desc->layer_param.s0_param.paddr = phys_addr;
		} else {
			list_add_tail(&desc->list, &first->tx_list);
			prev->next = desc;
			desc->next = NULL;

			if (i == 1)
				desc->layer_param.out_param.paddr = phys_addr;
			else
				desc->layer_param.ol_param.paddr = phys_addr;
		}

		prev = desc;
	}

	pxp->pxp_conf_state.layer_nr = sg_len;
	first->txd.flags = tx_flags;
	first->len = sg_len;
	pr_debug("%s:%d first %p, first->len %d, flags %08x\n",
		 __func__, __LINE__, first, first->len, first->txd.flags);

	return &first->txd;
}

static void pxp_issue_pending(struct dma_chan *chan)
{
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	struct pxp_dma *pxp_dma = to_pxp_dma(chan->device);
	struct pxps *pxp = to_pxp(pxp_dma);

	spin_lock(&pxp_chan->lock);

	if (list_empty(&pxp_chan->queue)) {
		spin_unlock(&pxp_chan->lock);
		return;
	}

	pxpdma_dequeue(pxp_chan, pxp);
	pxp_chan->status = PXP_CHANNEL_READY;

	spin_unlock(&pxp_chan->lock);

	pxp_clk_enable(pxp);
	wake_up_interruptible(&pxp->thread_waitq);
}

static void __pxp_terminate_all(struct dma_chan *chan)
{
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	pxp_chan->status = PXP_CHANNEL_INITIALIZED;
}

static int pxp_device_terminate_all(struct dma_chan *chan)
{
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);

	spin_lock(&pxp_chan->lock);
	__pxp_terminate_all(chan);
	spin_unlock(&pxp_chan->lock);

	return 0;
}

static int pxp_alloc_chan_resources(struct dma_chan *chan)
{
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	struct pxp_dma *pxp_dma = to_pxp_dma(chan->device);
	int ret;

	/* dmaengine.c now guarantees to only offer free channels */
	BUG_ON(chan->client_count > 1);
	WARN_ON(pxp_chan->status != PXP_CHANNEL_FREE);

	chan->cookie = 1;
	pxp_chan->completed = -ENXIO;

	pr_debug("%s dma_chan.chan_id %d\n", __func__, chan->chan_id);
	ret = pxp_init_channel(pxp_dma, pxp_chan);
	if (ret < 0)
		goto err_chan;

	pxp_chan->status = PXP_CHANNEL_INITIALIZED;

	dev_dbg(&chan->dev->device, "Found channel 0x%x, irq %d\n",
		chan->chan_id, pxp_chan->eof_irq);

	return ret;

err_chan:
	return ret;
}

static void pxp_free_chan_resources(struct dma_chan *chan)
{
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);

	spin_lock(&pxp_chan->lock);

	__pxp_terminate_all(chan);

	pxp_chan->status = PXP_CHANNEL_FREE;

	spin_unlock(&pxp_chan->lock);
}

static enum dma_status pxp_tx_status(struct dma_chan *chan,
				     dma_cookie_t cookie,
				     struct dma_tx_state *txstate)
{
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);

	if (cookie != chan->cookie)
		return DMA_ERROR;

	if (txstate) {
		txstate->last = pxp_chan->completed;
		txstate->used = chan->cookie;
		txstate->residue = 0;
	}
	return DMA_COMPLETE;
}

static void pxp_data_path_config_v3p(struct pxps *pxp)
{
	u32 val = 0;

	__raw_writel(
		BF_PXP_DATA_PATH_CTRL0_MUX15_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX14_SEL(1)|
		BF_PXP_DATA_PATH_CTRL0_MUX13_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX12_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX11_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX10_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX9_SEL(1)|
		BF_PXP_DATA_PATH_CTRL0_MUX8_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX7_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX6_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX5_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX4_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX3_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX2_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX1_SEL(0)|
		BF_PXP_DATA_PATH_CTRL0_MUX0_SEL(0),
		pxp->base + HW_PXP_DATA_PATH_CTRL0);

	/*
	 * MUX17: HIST_B as histogram: 0: output buffer, 1: wfe_store
	 * MUX16: HIST_A as collision: 0: output buffer, 1: wfe_store
	 */
	if (pxp_is_v3(pxp))
		val = BF_PXP_DATA_PATH_CTRL1_MUX17_SEL(1)|
		      BF_PXP_DATA_PATH_CTRL1_MUX16_SEL(0);
	else if (pxp_is_v3p(pxp))
		val = BF_PXP_DATA_PATH_CTRL1_MUX17_SEL(1)|
		      BF_PXP_DATA_PATH_CTRL1_MUX16_SEL(1);
	__raw_writel(val, pxp->base + HW_PXP_DATA_PATH_CTRL1);
}

static void pxp_soft_reset(struct pxps *pxp)
{
	__raw_writel(BM_PXP_CTRL_SFTRST, pxp->base + HW_PXP_CTRL_CLR);
	__raw_writel(BM_PXP_CTRL_CLKGATE, pxp->base + HW_PXP_CTRL_CLR);

	__raw_writel(BM_PXP_CTRL_SFTRST, pxp->base + HW_PXP_CTRL_SET);
	while (!(__raw_readl(pxp->base + HW_PXP_CTRL) & BM_PXP_CTRL_CLKGATE))
		dev_dbg(pxp->dev, "%s: wait for clock gate off", __func__);

	__raw_writel(BM_PXP_CTRL_SFTRST, pxp->base + HW_PXP_CTRL_CLR);
	__raw_writel(BM_PXP_CTRL_CLKGATE, pxp->base + HW_PXP_CTRL_CLR);
}

static void pxp_sram_init(struct pxps *pxp, u32 select,
			u32 buffer_addr, u32 length)
{
	u32 i;

	__raw_writel(
		BF_PXP_INIT_MEM_CTRL_ADDR(0) |
		BF_PXP_INIT_MEM_CTRL_SELECT(select) |
		BF_PXP_INIT_MEM_CTRL_START(1),
		pxp->base + HW_PXP_INIT_MEM_CTRL);

	if ((select == WFE_A) || (select == WFE_B)) {
		for (i = 0; i < length / 2; i++) {
			__raw_writel(*(((u32*)buffer_addr) + 2 * i + 1),
				pxp->base + HW_PXP_INIT_MEM_DATA_HIGH);

			__raw_writel(*(((u32*)buffer_addr) + 2 * i),
				pxp->base + HW_PXP_INIT_MEM_DATA);
		}
	} else {
		for (i = 0; i < length; i++) {
			__raw_writel(*(((u32*) buffer_addr) + i),
				pxp->base + HW_PXP_INIT_MEM_DATA);
		}
	}

	__raw_writel(
		BF_PXP_INIT_MEM_CTRL_ADDR(0) |
		BF_PXP_INIT_MEM_CTRL_SELECT(select) |
		BF_PXP_INIT_MEM_CTRL_START(0),
		pxp->base + HW_PXP_INIT_MEM_CTRL);
}

/*
 * wfe a configuration
 * configure wfe a engine for waveform processing
 * including its fetch and store module
 */
static void pxp_wfe_a_configure(struct pxps *pxp)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;

	/* FETCH */
	__raw_writel(
		BF_PXP_WFA_FETCH_CTRL_BF1_EN(1) |
		BF_PXP_WFA_FETCH_CTRL_BF1_HSK_MODE(0) |
		BF_PXP_WFA_FETCH_CTRL_BF1_BYTES_PP(0) |
		BF_PXP_WFA_FETCH_CTRL_BF1_LINE_MODE(0) |
		BF_PXP_WFA_FETCH_CTRL_BF1_SRAM_IF(0) |
		BF_PXP_WFA_FETCH_CTRL_BF1_BURST_LEN(0) |
		BF_PXP_WFA_FETCH_CTRL_BF1_BYPASS_MODE(0) |
		BF_PXP_WFA_FETCH_CTRL_BF2_EN(1) |
		BF_PXP_WFA_FETCH_CTRL_BF2_HSK_MODE(0) |
		BF_PXP_WFA_FETCH_CTRL_BF2_BYTES_PP(1) |
		BF_PXP_WFA_FETCH_CTRL_BF2_LINE_MODE(0) |
		BF_PXP_WFA_FETCH_CTRL_BF2_SRAM_IF(0) |
		BF_PXP_WFA_FETCH_CTRL_BF2_BURST_LEN(0) |
		BF_PXP_WFA_FETCH_CTRL_BF2_BYPASS_MODE(0),
		pxp->base + HW_PXP_WFA_FETCH_CTRL);

	__raw_writel(
		BF_PXP_WFA_ARRAY_PIXEL0_MASK_SIGN_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL0_MASK_OFFSET_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL0_MASK_SIGN_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL0_MASK_OFFSET_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL0_MASK_BUF_SEL(1) |
		BF_PXP_WFA_ARRAY_PIXEL0_MASK_H_OFS(0) |
		BF_PXP_WFA_ARRAY_PIXEL0_MASK_L_OFS(3),
		pxp->base + HW_PXP_WFA_ARRAY_PIXEL0_MASK);

	 __raw_writel(
		BF_PXP_WFA_ARRAY_PIXEL1_MASK_SIGN_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL1_MASK_OFFSET_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL1_MASK_SIGN_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL1_MASK_OFFSET_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL1_MASK_BUF_SEL(1) |
		BF_PXP_WFA_ARRAY_PIXEL1_MASK_H_OFS(4) |
		BF_PXP_WFA_ARRAY_PIXEL1_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFA_ARRAY_PIXEL1_MASK);

	 __raw_writel(
		BF_PXP_WFA_ARRAY_PIXEL3_MASK_SIGN_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL3_MASK_OFFSET_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL3_MASK_SIGN_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL3_MASK_OFFSET_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL3_MASK_BUF_SEL(1) |
		BF_PXP_WFA_ARRAY_PIXEL3_MASK_H_OFS(8) |
		BF_PXP_WFA_ARRAY_PIXEL3_MASK_L_OFS(9),
		pxp->base + HW_PXP_WFA_ARRAY_PIXEL2_MASK);

	__raw_writel(
		BF_PXP_WFA_ARRAY_PIXEL4_MASK_SIGN_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL4_MASK_OFFSET_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL4_MASK_SIGN_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL4_MASK_OFFSET_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL4_MASK_BUF_SEL(1) |
		BF_PXP_WFA_ARRAY_PIXEL4_MASK_H_OFS(10) |
		BF_PXP_WFA_ARRAY_PIXEL4_MASK_L_OFS(15),
		pxp->base + HW_PXP_WFA_ARRAY_PIXEL3_MASK);

	__raw_writel(
		BF_PXP_WFA_ARRAY_PIXEL2_MASK_SIGN_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL2_MASK_OFFSET_Y(0) |
		BF_PXP_WFA_ARRAY_PIXEL2_MASK_SIGN_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL2_MASK_OFFSET_X(0) |
		BF_PXP_WFA_ARRAY_PIXEL2_MASK_BUF_SEL(0) |
		BF_PXP_WFA_ARRAY_PIXEL2_MASK_H_OFS(4) |
		BF_PXP_WFA_ARRAY_PIXEL2_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFA_ARRAY_PIXEL4_MASK);

	__raw_writel(1, pxp->base + HW_PXP_WFA_ARRAY_REG2);

	/* STORE */
	__raw_writel(
		BF_PXP_WFE_A_STORE_CTRL_CH0_CH_EN(1)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_BLOCK_EN(0)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_BLOCK_16(0)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_HANDSHAKE_EN(0)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_ARRAY_EN(0)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_ARRAY_LINE_NUM(0)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_STORE_BYPASS_EN(0)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_STORE_MEMORY_EN(1)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_PACK_IN_SEL(1)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_FILL_DATA_EN(0)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_WR_NUM_BYTES(8)|
		BF_PXP_WFE_A_STORE_CTRL_CH0_COMBINE_2CHANNEL(1) |
		BF_PXP_WFE_A_STORE_CTRL_CH0_ARBIT_EN(0),
		pxp->base + HW_PXP_WFE_A_STORE_CTRL_CH0);

	__raw_writel(
		 BF_PXP_WFE_A_STORE_CTRL_CH1_CH_EN(1)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_BLOCK_EN(0)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_BLOCK_16(0)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_HANDSHAKE_EN(0)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_ARRAY_EN(0)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_ARRAY_LINE_NUM(0)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_STORE_BYPASS_EN(0)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_STORE_MEMORY_EN(1)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_PACK_IN_SEL(1)|
		 BF_PXP_WFE_A_STORE_CTRL_CH1_WR_NUM_BYTES(16),
		pxp->base + HW_PXP_WFE_A_STORE_CTRL_CH1);

	__raw_writel(
		BF_PXP_WFE_A_STORE_SHIFT_CTRL_CH0_OUTPUT_ACTIVE_BPP(0)|
		BF_PXP_WFE_A_STORE_SHIFT_CTRL_CH0_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_A_STORE_SHIFT_CTRL_CH0_OUT_YUV422_2P_EN(0)|
		BF_PXP_WFE_A_STORE_SHIFT_CTRL_CH0_SHIFT_BYPASS(0),
		pxp->base + HW_PXP_WFE_A_STORE_SHIFT_CTRL_CH0);


	__raw_writel(
		BF_PXP_WFE_A_STORE_SHIFT_CTRL_CH1_OUTPUT_ACTIVE_BPP(1)|
		BF_PXP_WFE_A_STORE_SHIFT_CTRL_CH1_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_A_STORE_SHIFT_CTRL_CH1_OUT_YUV422_2P_EN(0),
		pxp->base + HW_PXP_WFE_A_STORE_SHIFT_CTRL_CH1);

	__raw_writel(BF_PXP_WFE_A_STORE_FILL_DATA_CH0_FILL_DATA_CH0(0),
		pxp->base + HW_PXP_WFE_A_STORE_FILL_DATA_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK0_H_CH0_D_MASK0_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK0_H_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK0_L_CH0_D_MASK0_L_CH0(0xf), /* fetch CP */
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK0_L_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK1_H_CH0_D_MASK1_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK1_H_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK1_L_CH0_D_MASK1_L_CH0(0xf00), /* fetch NP */
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK1_L_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK2_H_CH0_D_MASK2_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK2_H_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK2_L_CH0_D_MASK2_L_CH0(0x00000),
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK2_L_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK3_H_CH0_D_MASK3_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK3_H_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK3_L_CH0_D_MASK3_L_CH0(0x3f000000), /* fetch LUT */
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK3_L_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK4_H_CH0_D_MASK4_H_CH0(0xf),
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK4_H_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_D_MASK4_L_CH0_D_MASK4_L_CH0(0x0), /* fetch Y4 */
		pxp->base + HW_PXP_WFE_A_STORE_D_MASK4_L_CH0);

	__raw_writel(
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH0(32) |
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG0(1) |
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH1(28)|
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG1(1) |
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH2(24)|
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG2(1)|
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH3(18)|
		BF_PXP_WFE_A_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG3(1),
		pxp->base + HW_PXP_WFE_A_STORE_D_SHIFT_L_CH0);

	__raw_writel(
		BF_PXP_WFE_A_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH4(28) |
		BF_PXP_WFE_A_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG4(0) |
		BF_PXP_WFE_A_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH5(0)|
		BF_PXP_WFE_A_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG5(0) |
		BF_PXP_WFE_A_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH6(0)|
		BF_PXP_WFE_A_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG6(0) |
		BF_PXP_WFE_A_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH7(0),
		pxp->base + HW_PXP_WFE_A_STORE_D_SHIFT_H_CH0);

	__raw_writel(
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH0(1)|
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG0(1)|
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH1(1)|
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG1(0)|
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH2(32+6)|
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG2(1)|
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH3(32+6)|
		BF_PXP_WFE_A_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG3(1),
		pxp->base + HW_PXP_WFE_A_STORE_F_SHIFT_L_CH0);

	__raw_writel(
		BF_PXP_WFE_A_STORE_F_MASK_H_CH0_F_MASK4(0)|
		BF_PXP_WFE_A_STORE_F_MASK_H_CH0_F_MASK5(0)|
		BF_PXP_WFE_A_STORE_F_MASK_H_CH0_F_MASK6(0)|
		BF_PXP_WFE_A_STORE_F_MASK_H_CH0_F_MASK7(0),
		pxp->base + HW_PXP_WFE_A_STORE_F_MASK_H_CH0);


	__raw_writel(
		BF_PXP_WFE_A_STORE_F_MASK_L_CH0_F_MASK0(0x1) |
		BF_PXP_WFE_A_STORE_F_MASK_L_CH0_F_MASK1(0x2) |
		BF_PXP_WFE_A_STORE_F_MASK_L_CH0_F_MASK2(0x4) |
		BF_PXP_WFE_A_STORE_F_MASK_L_CH0_F_MASK3(0x8),
		pxp->base + HW_PXP_WFE_A_STORE_F_MASK_L_CH0);

	/* ALU */
	__raw_writel(BF_PXP_ALU_A_INST_ENTRY_ENTRY_ADDR(0),
		pxp->base + HW_PXP_ALU_A_INST_ENTRY);

	__raw_writel(BF_PXP_ALU_A_PARAM_PARAM0(0) |
		BF_PXP_ALU_A_PARAM_PARAM1(0),
		pxp->base + HW_PXP_ALU_A_PARAM);

	__raw_writel(BF_PXP_ALU_A_CONFIG_BUF_ADDR(0),
		pxp->base + HW_PXP_ALU_A_CONFIG);

	__raw_writel(BF_PXP_ALU_A_LUT_CONFIG_MODE(0) |
		BF_PXP_ALU_A_LUT_CONFIG_EN(0),
		pxp->base + HW_PXP_ALU_A_LUT_CONFIG);

	__raw_writel(BF_PXP_ALU_A_LUT_DATA0_LUT_DATA_L(0),
		pxp->base + HW_PXP_ALU_A_LUT_DATA0);

	__raw_writel(BF_PXP_ALU_A_LUT_DATA1_LUT_DATA_H(0),
		pxp->base + HW_PXP_ALU_A_LUT_DATA1);

	__raw_writel(BF_PXP_ALU_A_CTRL_BYPASS    (1) |
		BF_PXP_ALU_A_CTRL_ENABLE    (1) |
		BF_PXP_ALU_A_CTRL_START     (0) |
		BF_PXP_ALU_A_CTRL_SW_RESET  (0),
		pxp->base + HW_PXP_ALU_A_CTRL);

	/* WFE A */
	__raw_writel(0x3F3F0303, pxp->base + HW_PXP_WFE_A_STAGE1_MUX0);
	__raw_writel(0x0C00000C, pxp->base + HW_PXP_WFE_A_STAGE1_MUX1);
	__raw_writel(0x01040000, pxp->base + HW_PXP_WFE_A_STAGE1_MUX2);
	__raw_writel(0x0A0A0904, pxp->base + HW_PXP_WFE_A_STAGE1_MUX3);
	__raw_writel(0x00000B0B, pxp->base + HW_PXP_WFE_A_STAGE1_MUX4);

	__raw_writel(0x1800280E, pxp->base + HW_PXP_WFE_A_STAGE2_MUX0);
	__raw_writel(0x00280E01, pxp->base + HW_PXP_WFE_A_STAGE2_MUX1);
	__raw_writel(0x280E0118, pxp->base + HW_PXP_WFE_A_STAGE2_MUX2);
	__raw_writel(0x00011800, pxp->base + HW_PXP_WFE_A_STAGE2_MUX3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STAGE2_MUX4);
	__raw_writel(0x1800280E, pxp->base + HW_PXP_WFE_A_STAGE2_MUX5);
	__raw_writel(0x00280E01, pxp->base + HW_PXP_WFE_A_STAGE2_MUX6);
	__raw_writel(0x1A0E0118, pxp->base + HW_PXP_WFE_A_STAGE2_MUX7);
	__raw_writel(0x1B012911, pxp->base + HW_PXP_WFE_A_STAGE2_MUX8);
	__raw_writel(0x00002911, pxp->base + HW_PXP_WFE_A_STAGE2_MUX9);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STAGE2_MUX10);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STAGE2_MUX11);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STAGE2_MUX12);

	__raw_writel(0x07060504, pxp->base + HW_PXP_WFE_A_STAGE3_MUX0);
	__raw_writel(0x3F3F3F08, pxp->base + HW_PXP_WFE_A_STAGE3_MUX1);
	__raw_writel(0x03020100, pxp->base + HW_PXP_WFE_A_STAGE3_MUX2);
	__raw_writel(0x3F3F3F3F, pxp->base + HW_PXP_WFE_A_STAGE3_MUX3);

	__raw_writel(0x001F1F1F, pxp->base + HW_PXP_WFE_A_STAGE2_5X6_MASKS_0);
	__raw_writel(0x3f030100, pxp->base + HW_PXP_WFE_A_STAGE2_5X6_ADDR_0);

	__raw_writel(0x00000700, pxp->base + HW_PXP_WFE_A_STG2_5X1_OUT0);
	__raw_writel(0x00007000, pxp->base + HW_PXP_WFE_A_STG2_5X1_OUT1);
	__raw_writel(0x0000A000, pxp->base + HW_PXP_WFE_A_STG2_5X1_OUT2);
	__raw_writel(0x000000C0, pxp->base + HW_PXP_WFE_A_STG2_5X1_OUT3);
	__raw_writel(0x071F1F1F, pxp->base + HW_PXP_WFE_A_STG2_5X1_MASKS);

	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_2);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_3);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_4);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_5);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_6);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT2_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT3_7);

	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_0);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_1);
	__raw_writel(0x04050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_2);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_3);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_4);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_5);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_6);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT0_7);

	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_0);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_1);
	__raw_writel(0x05080808, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_2);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_3);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_4);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_5);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_6);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT1_7);

	__raw_writel(0x07070707, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_0);
	__raw_writel(0x07070707, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_1);
	__raw_writel(0x070C0C0C, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_2);
	__raw_writel(0x07070707, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_3);
	__raw_writel(0X0F0F0F0F, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_4);
	__raw_writel(0X0F0F0F0F, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_5);
	__raw_writel(0X0F0F0F0F, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_6);
	__raw_writel(0X0F0F0F0F, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT2_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG2_5X6_OUT3_7);

	if (pxp->devdata && pxp->devdata->pxp_lut_cleanup_multiple)
		pxp->devdata->pxp_lut_cleanup_multiple(pxp,
					proc_data->lut_sels, 1);
}

static void pxp_wfe_a_configure_v3p(struct pxps *pxp)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;

	/* FETCH */
	__raw_writel(
		BF_PXP_WFB_FETCH_CTRL_BF1_EN(1) |
		BF_PXP_WFB_FETCH_CTRL_BF1_HSK_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_BYTES_PP(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_LINE_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_SRAM_IF(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_BURST_LEN(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_BYPASS_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_EN(1) |
		BF_PXP_WFB_FETCH_CTRL_BF2_HSK_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_BYTES_PP(1) |
		BF_PXP_WFB_FETCH_CTRL_BF2_LINE_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_SRAM_IF(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_BURST_LEN(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_BYPASS_MODE(0),
		pxp->base + HW_PXP_WFB_FETCH_CTRL);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_H_OFS(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_L_OFS(3),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL0_MASK);

	 __raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_H_OFS(4) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL1_MASK);

	 __raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_H_OFS(8) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_L_OFS(9),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL2_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_H_OFS(10) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_L_OFS(15),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL3_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_BUF_SEL(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_H_OFS(4) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL4_MASK);

	__raw_writel(1, pxp->base + HW_PXP_WFB_ARRAY_REG2);

	/* STORE */
	__raw_writel(
		BF_PXP_WFE_B_STORE_CTRL_CH0_CH_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_BLOCK_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_BLOCK_16(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_HANDSHAKE_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARRAY_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARRAY_LINE_NUM(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_STORE_BYPASS_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_STORE_MEMORY_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_PACK_IN_SEL(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_FILL_DATA_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_WR_NUM_BYTES(8)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_COMBINE_2CHANNEL(1) |
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARBIT_EN(0),
		pxp->base + HW_PXP_WFE_B_STORE_CTRL_CH0);

	__raw_writel(
		 BF_PXP_WFE_B_STORE_CTRL_CH1_CH_EN(1)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_BLOCK_EN(0)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_BLOCK_16(0)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_HANDSHAKE_EN(0)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_ARRAY_EN(0)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_ARRAY_LINE_NUM(0)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_STORE_BYPASS_EN(0)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_STORE_MEMORY_EN(1)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_PACK_IN_SEL(1)|
		 BF_PXP_WFE_B_STORE_CTRL_CH1_WR_NUM_BYTES(16),
		pxp->base + HW_PXP_WFE_B_STORE_CTRL_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUTPUT_ACTIVE_BPP(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_2P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_SHIFT_BYPASS(0),
		pxp->base + HW_PXP_WFE_B_STORE_SHIFT_CTRL_CH0);


	__raw_writel(
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH1_OUTPUT_ACTIVE_BPP(1)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH1_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH1_OUT_YUV422_2P_EN(0),
		pxp->base + HW_PXP_WFE_B_STORE_SHIFT_CTRL_CH1);

	__raw_writel(BF_PXP_WFE_B_STORE_FILL_DATA_CH0_FILL_DATA_CH0(0),
		pxp->base + HW_PXP_WFE_B_STORE_FILL_DATA_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK0_H_CH0_D_MASK0_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK0_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK0_L_CH0_D_MASK0_L_CH0(0xf), /* fetch CP */
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK0_L_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK1_H_CH0_D_MASK1_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK1_L_CH0_D_MASK1_L_CH0(0xf00), /* fetch NP */
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_L_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK2_H_CH0_D_MASK2_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK2_L_CH0_D_MASK2_L_CH0(0x00000),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_L_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK3_H_CH0_D_MASK3_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK3_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK3_L_CH0_D_MASK3_L_CH0(0x3f000000), /* fetch LUT */
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK3_L_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK4_H_CH0_D_MASK4_H_CH0(0xf),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK4_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK4_L_CH0_D_MASK4_L_CH0(0x0), /* fetch Y4 */
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK4_L_CH0);

	__raw_writel(0x0, pxp->base + HW_PXP_WFE_B_STORE_D_MASK5_H_CH0);
	__raw_writel(0x0, pxp->base + HW_PXP_WFE_B_STORE_D_MASK5_L_CH0);
	__raw_writel(0x0, pxp->base + HW_PXP_WFE_B_STORE_D_MASK6_H_CH0);
	__raw_writel(0x0, pxp->base + HW_PXP_WFE_B_STORE_D_MASK6_L_CH0);
	__raw_writel(0x0, pxp->base + HW_PXP_WFE_B_STORE_D_MASK7_H_CH0);
	__raw_writel(0x0, pxp->base + HW_PXP_WFE_B_STORE_D_MASK7_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH0(32) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG0(1) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH1(28)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG1(1) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH2(24)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG2(1)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH3(18)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG3(1),
		pxp->base + HW_PXP_WFE_B_STORE_D_SHIFT_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH4(28) |
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG4(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH5(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG5(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH6(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG6(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH7(0),
		pxp->base + HW_PXP_WFE_B_STORE_D_SHIFT_H_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH0(1)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG0(1)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH1(1)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG1(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH2(32+6)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG2(1)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH3(32+6)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG3(1),
		pxp->base + HW_PXP_WFE_B_STORE_F_SHIFT_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_WIDTH4(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_FLAG4(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_WIDTH5(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_FLAG5(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_WIDTH6(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_FLAG6(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_WIDTH7(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_H_CH0_F_SHIFT_FLAG7(0),
		pxp->base + HW_PXP_WFE_B_STORE_F_SHIFT_H_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK0(0x1) |
		BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK1(0x2) |
		BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK2(0x4) |
		BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK3(0x8),
		pxp->base + HW_PXP_WFE_B_STORE_F_MASK_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK4(0x0)|
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK5(0x0)|
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK6(0x0)|
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK7(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_F_MASK_H_CH0);


	/* ALU */
	__raw_writel(BF_PXP_ALU_B_INST_ENTRY_ENTRY_ADDR(0),
		pxp->base + HW_PXP_ALU_B_INST_ENTRY);

	__raw_writel(BF_PXP_ALU_B_PARAM_PARAM0(0) |
		BF_PXP_ALU_B_PARAM_PARAM1(0),
		pxp->base + HW_PXP_ALU_B_PARAM);

	__raw_writel(BF_PXP_ALU_B_CONFIG_BUF_ADDR(0),
		pxp->base + HW_PXP_ALU_B_CONFIG);

	__raw_writel(BF_PXP_ALU_B_LUT_CONFIG_MODE(0) |
		BF_PXP_ALU_B_LUT_CONFIG_EN(0),
		pxp->base + HW_PXP_ALU_B_LUT_CONFIG);

	__raw_writel(BF_PXP_ALU_B_LUT_DATA0_LUT_DATA_L(0),
		pxp->base + HW_PXP_ALU_B_LUT_DATA0);

	__raw_writel(BF_PXP_ALU_B_LUT_DATA1_LUT_DATA_H(0),
		pxp->base + HW_PXP_ALU_B_LUT_DATA1);

	__raw_writel(BF_PXP_ALU_B_CTRL_BYPASS    (1) |
		BF_PXP_ALU_B_CTRL_ENABLE    (1) |
		BF_PXP_ALU_B_CTRL_START     (0) |
		BF_PXP_ALU_B_CTRL_SW_RESET  (0),
		pxp->base + HW_PXP_ALU_B_CTRL);

	/* WFE A */
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX2);
	__raw_writel(0x03000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX3);
	__raw_writel(0x00000003, pxp->base + HW_PXP_WFE_B_STAGE1_MUX4);
	__raw_writel(0x04000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX5);
	__raw_writel(0x0A090401, pxp->base + HW_PXP_WFE_B_STAGE1_MUX6);
	__raw_writel(0x000B0B0A, pxp->base + HW_PXP_WFE_B_STAGE1_MUX7);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX8);

	__raw_writel(0x1901290C, pxp->base + HW_PXP_WFE_B_STAGE2_MUX0);
	__raw_writel(0x01290C02, pxp->base + HW_PXP_WFE_B_STAGE2_MUX1);
	__raw_writel(0x290C0219, pxp->base + HW_PXP_WFE_B_STAGE2_MUX2);
	__raw_writel(0x00021901, pxp->base + HW_PXP_WFE_B_STAGE2_MUX3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STAGE2_MUX4);
	__raw_writel(0x1901290C, pxp->base + HW_PXP_WFE_B_STAGE2_MUX5);
	__raw_writel(0x01290C02, pxp->base + HW_PXP_WFE_B_STAGE2_MUX6);
	__raw_writel(0x1B0C0219, pxp->base + HW_PXP_WFE_B_STAGE2_MUX7);
	__raw_writel(0x1C022A0F, pxp->base + HW_PXP_WFE_B_STAGE2_MUX8);
	__raw_writel(0x02002A0F, pxp->base + HW_PXP_WFE_B_STAGE2_MUX9);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STAGE2_MUX10);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STAGE2_MUX11);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STAGE2_MUX12);

	__raw_writel(0x2a123a1d, pxp->base + HW_PXP_WFE_B_STAGE3_MUX0);
	__raw_writel(0x00000013, pxp->base + HW_PXP_WFE_B_STAGE3_MUX1);
	__raw_writel(0x2a123a1d, pxp->base + HW_PXP_WFE_B_STAGE3_MUX2);
	__raw_writel(0x00000013, pxp->base + HW_PXP_WFE_B_STAGE3_MUX3);
	__raw_writel(0x3b202c1d, pxp->base + HW_PXP_WFE_B_STAGE3_MUX4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE3_MUX5);
	__raw_writel(0x003b202d, pxp->base + HW_PXP_WFE_B_STAGE3_MUX6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE3_MUX7);
	__raw_writel(0x07060504, pxp->base + HW_PXP_WFE_B_STAGE3_MUX8);
	__raw_writel(0x00000008, pxp->base + HW_PXP_WFE_B_STAGE3_MUX9);
	__raw_writel(0x03020100, pxp->base + HW_PXP_WFE_B_STAGE3_MUX10);

	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_7);

	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_7);

	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_5X8_MASKS_0);

	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X1_OUT0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X1_MASKS);

	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_2);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_3);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_4);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_5);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_6);
	__raw_writel(0xFFFFFFFF, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_7);

	__raw_writel(0x00000700, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT0);
	__raw_writel(0x00007000, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT1);
	__raw_writel(0x0000A000, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT2);
	__raw_writel(0x000000C0, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT3);
	__raw_writel(0x070F1F1F, pxp->base + HW_PXP_WFE_B_STG2_5X1_MASKS);

	__raw_writel(0x001F1F1F, pxp->base + HW_PXP_WFE_B_STAGE2_5X6_MASKS_0);
	__raw_writel(0x3f232120, pxp->base + HW_PXP_WFE_B_STAGE2_5X6_ADDR_0);

	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_0);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_1);
	__raw_writel(0x04050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_2);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_3);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_4);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_5);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_6);
	__raw_writel(0x04040404, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_7);

	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_0);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_1);
	__raw_writel(0x05080808, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_2);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_3);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_4);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_5);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_6);
	__raw_writel(0x05050505, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_7);

	__raw_writel(0x07070707, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_0);
	__raw_writel(0x07070707, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_1);
	__raw_writel(0x070C0C0C, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_2);
	__raw_writel(0x07070707, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_3);
	__raw_writel(0x0F0F0F0F, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_4);
	__raw_writel(0x0F0F0F0F, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_5);
	__raw_writel(0x0F0F0F0F, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_6);
	__raw_writel(0x0F0F0F0F, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT2_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT3_7);

	__raw_writel(0x070F1F1F, pxp->base + HW_PXP_WFE_B_STG3_F8X1_MASKS);

	__raw_writel(0x00000700, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_7);

	__raw_writel(0x00007000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_7);

	__raw_writel(0x0000A000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT2_7);

	__raw_writel(0x000000C0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT3_7);

	if (pxp->devdata && pxp->devdata->pxp_lut_cleanup_multiple)
		pxp->devdata->pxp_lut_cleanup_multiple(pxp,
					proc_data->lut_sels, 1);
}

/*
 *  wfe a processing
 * use wfe a to process an update
 * x,y,width,height:
 *         coordinate and size of the update region
 * wb:
 *         working buffer, 16bpp
 * upd:
 *         update buffer, in Y4 with or without alpha, 8bpp
 * twb:
 *         temp working buffer, 16bpp
 *         only used when reagl_en is 1
 * y4c:
 *         y4c buffer, {Y4[3:0],3'b000,collision}, 8bpp
 * lut:
 *         valid value 0-63
 *         set to the lut used for next update
 * partial:
 *         0 - full update
 *         1 - partial update
 * reagl_en:
 *         0 - use normal waveform algorithm
 *         1 - enable reagl/-d waveform algorithm
 * detection_only:
 *         0 - write working buffer
 *         1 - do no write working buffer, detection only
 * alpha_en:
 *         0 - upd is {Y4[3:0],4'b0000} format
 *         1 - upd is {Y4[3:0],3'b000,alpha} format
 */
static void pxp_wfe_a_process(struct pxps *pxp)
{
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &config_data->proc_data;
	struct pxp_layer_param *fetch_ch0 = &config_data->wfe_a_fetch_param[0];
	struct pxp_layer_param *fetch_ch1 = &config_data->wfe_a_fetch_param[1];
	struct pxp_layer_param *store_ch0 = &config_data->wfe_a_store_param[0];
	struct pxp_layer_param *store_ch1 = &config_data->wfe_a_store_param[1];
	int v;

	if (fetch_ch0->width != fetch_ch1->width ||
		fetch_ch0->height != fetch_ch1->height) {
		dev_err(pxp->dev, "width/height should be same for two fetch "
				"channels\n");
	}

	print_param(fetch_ch0, "wfe_a fetch_ch0");
	print_param(fetch_ch1, "wfe_a fetch_ch1");
	print_param(store_ch0, "wfe_a store_ch0");
	print_param(store_ch1, "wfe_a store_ch1");

	/* Fetch */
	__raw_writel(fetch_ch0->paddr, pxp->base + HW_PXP_WFA_FETCH_BUF1_ADDR);

	__raw_writel(BF_PXP_WFA_FETCH_BUF1_CORD_YCORD(fetch_ch0->top) |
		BF_PXP_WFA_FETCH_BUF1_CORD_XCORD(fetch_ch0->left),
		pxp->base + HW_PXP_WFA_FETCH_BUF1_CORD);

	__raw_writel(fetch_ch0->stride, pxp->base + HW_PXP_WFA_FETCH_BUF1_PITCH);

	__raw_writel(BF_PXP_WFA_FETCH_BUF1_SIZE_BUF_HEIGHT(fetch_ch0->height - 1) |
		BF_PXP_WFA_FETCH_BUF1_SIZE_BUF_WIDTH(fetch_ch0->width - 1),
		pxp->base + HW_PXP_WFA_FETCH_BUF1_SIZE);

	__raw_writel(fetch_ch1->paddr, pxp->base + HW_PXP_WFA_FETCH_BUF2_ADDR);

	__raw_writel(BF_PXP_WFA_FETCH_BUF2_CORD_YCORD(fetch_ch1->top) |
		BF_PXP_WFA_FETCH_BUF2_CORD_XCORD(fetch_ch1->left),
		pxp->base + HW_PXP_WFA_FETCH_BUF2_CORD);

	__raw_writel(fetch_ch1->stride * 2, pxp->base + HW_PXP_WFA_FETCH_BUF2_PITCH);

	__raw_writel(BF_PXP_WFA_FETCH_BUF2_SIZE_BUF_HEIGHT(fetch_ch1->height - 1) |
		BF_PXP_WFA_FETCH_BUF2_SIZE_BUF_WIDTH(fetch_ch1->width - 1),
		pxp->base + HW_PXP_WFA_FETCH_BUF2_SIZE);

	/* Store */
	__raw_writel(BF_PXP_WFE_A_STORE_SIZE_CH0_OUT_WIDTH(store_ch0->width - 1) |
		BF_PXP_WFE_A_STORE_SIZE_CH0_OUT_HEIGHT(store_ch0->height - 1),
		pxp->base + HW_PXP_WFE_A_STORE_SIZE_CH0);


	__raw_writel(BF_PXP_WFE_A_STORE_SIZE_CH1_OUT_WIDTH(store_ch1->width - 1) |
		BF_PXP_WFE_A_STORE_SIZE_CH1_OUT_HEIGHT(store_ch1->height - 1),
		pxp->base + HW_PXP_WFE_A_STORE_SIZE_CH1);

	__raw_writel(BF_PXP_WFE_A_STORE_PITCH_CH0_OUT_PITCH(store_ch0->stride) |
		BF_PXP_WFE_A_STORE_PITCH_CH1_OUT_PITCH(store_ch1->stride * 2),
		pxp->base + HW_PXP_WFE_A_STORE_PITCH);

	__raw_writel(BF_PXP_WFE_A_STORE_ADDR_0_CH0_OUT_BASE_ADDR0(store_ch0->paddr),
		pxp->base + HW_PXP_WFE_A_STORE_ADDR_0_CH0);
	__raw_writel(BF_PXP_WFE_A_STORE_ADDR_1_CH0_OUT_BASE_ADDR1(0),
		pxp->base + HW_PXP_WFE_A_STORE_ADDR_1_CH0);

	__raw_writel(BF_PXP_WFE_A_STORE_ADDR_0_CH1_OUT_BASE_ADDR0(
		store_ch1->paddr + (store_ch1->left + store_ch1->top *
		store_ch1->stride) * 2),
		pxp->base + HW_PXP_WFE_A_STORE_ADDR_0_CH1);

	__raw_writel(BF_PXP_WFE_A_STORE_ADDR_1_CH1_OUT_BASE_ADDR1(0),
		pxp->base + HW_PXP_WFE_A_STORE_ADDR_1_CH1);

	/* ALU */
	__raw_writel(BF_PXP_ALU_A_BUF_SIZE_BUF_WIDTH(fetch_ch0->width) |
	        BF_PXP_ALU_A_BUF_SIZE_BUF_HEIGHT(fetch_ch0->height),
		pxp->base + HW_PXP_ALU_A_BUF_SIZE);

	/* WFE */
	__raw_writel(BF_PXP_WFE_A_DIMENSIONS_WIDTH(fetch_ch0->width) |
		BF_PXP_WFE_A_DIMENSIONS_HEIGHT(fetch_ch0->height),
		pxp->base + HW_PXP_WFE_A_DIMENSIONS);

	/* Here it should be fetch_ch1 */
	__raw_writel(BF_PXP_WFE_A_OFFSET_X_OFFSET(fetch_ch1->left) |
		BF_PXP_WFE_A_OFFSET_Y_OFFSET(fetch_ch1->top),
		pxp->base + HW_PXP_WFE_A_OFFSET);

	__raw_writel((proc_data->lut & 0x000000FF) | 0x00000F00,
			pxp->base + HW_PXP_WFE_A_SW_DATA_REGS);
	__raw_writel((proc_data->partial_update | (proc_data->reagl_en << 1)),
			pxp->base + HW_PXP_WFE_A_SW_FLAG_REGS);

	__raw_writel(
		BF_PXP_WFE_A_CTRL_ENABLE(1) |
		BF_PXP_WFE_A_CTRL_SW_RESET(1),
		pxp->base + HW_PXP_WFE_A_CTRL);

       if (proc_data->alpha_en) {
		__raw_writel(BF_PXP_WFA_ARRAY_FLAG0_MASK_SIGN_Y(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_OFFSET_Y(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_SIGN_X(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_OFFSET_X(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_BUF_SEL(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_H_OFS(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_L_OFS(0),
			pxp->base + HW_PXP_WFA_ARRAY_FLAG0_MASK);
        } else {
		__raw_writel(BF_PXP_WFA_ARRAY_FLAG0_MASK_SIGN_Y(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_OFFSET_Y(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_SIGN_X(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_OFFSET_X(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_BUF_SEL(2) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_H_OFS(0) |
			BF_PXP_WFA_ARRAY_FLAG0_MASK_L_OFS(0),
			pxp->base + HW_PXP_WFA_ARRAY_FLAG0_MASK);
        }

	/* disable CH1 when only doing detection */
	v = __raw_readl(pxp->base + HW_PXP_WFE_A_STORE_CTRL_CH1);
	if (proc_data->detection_only) {
		v &= ~BF_PXP_WFE_A_STORE_CTRL_CH1_CH_EN(1);
		printk(KERN_EMERG "%s: detection only happens\n", __func__);
	} else
		v |= BF_PXP_WFE_A_STORE_CTRL_CH1_CH_EN(1);
	__raw_writel(v, pxp->base + HW_PXP_WFE_A_STORE_CTRL_CH1);
}

static void pxp_wfe_a_process_v3p(struct pxps *pxp)
{
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &config_data->proc_data;
	struct pxp_layer_param *fetch_ch0 = &config_data->wfe_a_fetch_param[0];
	struct pxp_layer_param *fetch_ch1 = &config_data->wfe_a_fetch_param[1];
	struct pxp_layer_param *store_ch0 = &config_data->wfe_a_store_param[0];
	struct pxp_layer_param *store_ch1 = &config_data->wfe_a_store_param[1];
	int v;

	if (fetch_ch0->width != fetch_ch1->width ||
		fetch_ch0->height != fetch_ch1->height) {
		dev_err(pxp->dev, "width/height should be same for two fetch "
				"channels\n");
	}

	print_param(fetch_ch0, "wfe_a fetch_ch0");
	print_param(fetch_ch1, "wfe_a fetch_ch1");
	print_param(store_ch0, "wfe_a store_ch0");
	print_param(store_ch1, "wfe_a store_ch1");

	/* Fetch */
	__raw_writel(fetch_ch0->paddr, pxp->base + HW_PXP_WFB_FETCH_BUF1_ADDR);

	__raw_writel(BF_PXP_WFB_FETCH_BUF1_CORD_YCORD(fetch_ch0->top) |
		BF_PXP_WFB_FETCH_BUF1_CORD_XCORD(fetch_ch0->left),
		pxp->base + HW_PXP_WFB_FETCH_BUF1_CORD);

	__raw_writel(fetch_ch0->stride, pxp->base + HW_PXP_WFB_FETCH_BUF1_PITCH);

	__raw_writel(BF_PXP_WFB_FETCH_BUF1_SIZE_BUF_HEIGHT(fetch_ch0->height - 1) |
		BF_PXP_WFB_FETCH_BUF1_SIZE_BUF_WIDTH(fetch_ch0->width - 1),
		pxp->base + HW_PXP_WFB_FETCH_BUF1_SIZE);

	__raw_writel(fetch_ch1->paddr, pxp->base + HW_PXP_WFB_FETCH_BUF2_ADDR);

	__raw_writel(BF_PXP_WFB_FETCH_BUF2_CORD_YCORD(fetch_ch1->top) |
		BF_PXP_WFB_FETCH_BUF2_CORD_XCORD(fetch_ch1->left),
		pxp->base + HW_PXP_WFB_FETCH_BUF2_CORD);

	__raw_writel(fetch_ch1->stride * 2, pxp->base + HW_PXP_WFB_FETCH_BUF2_PITCH);

	__raw_writel(BF_PXP_WFB_FETCH_BUF2_SIZE_BUF_HEIGHT(fetch_ch1->height - 1) |
		BF_PXP_WFB_FETCH_BUF2_SIZE_BUF_WIDTH(fetch_ch1->width - 1),
		pxp->base + HW_PXP_WFB_FETCH_BUF2_SIZE);

	/* Store */
	__raw_writel(BF_PXP_WFE_B_STORE_SIZE_CH0_OUT_WIDTH(store_ch0->width - 1) |
		BF_PXP_WFE_B_STORE_SIZE_CH0_OUT_HEIGHT(store_ch0->height - 1),
		pxp->base + HW_PXP_WFE_B_STORE_SIZE_CH0);


	__raw_writel(BF_PXP_WFE_B_STORE_SIZE_CH1_OUT_WIDTH(store_ch1->width - 1) |
		BF_PXP_WFE_B_STORE_SIZE_CH1_OUT_HEIGHT(store_ch1->height - 1),
		pxp->base + HW_PXP_WFE_B_STORE_SIZE_CH1);

	__raw_writel(BF_PXP_WFE_B_STORE_PITCH_CH0_OUT_PITCH(store_ch0->stride) |
		BF_PXP_WFE_B_STORE_PITCH_CH1_OUT_PITCH(store_ch1->stride * 2),
		pxp->base + HW_PXP_WFE_B_STORE_PITCH);

	__raw_writel(BF_PXP_WFE_B_STORE_ADDR_0_CH0_OUT_BASE_ADDR0(store_ch0->paddr),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_0_CH0);
	__raw_writel(BF_PXP_WFE_B_STORE_ADDR_1_CH0_OUT_BASE_ADDR1(0),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_1_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_ADDR_0_CH1_OUT_BASE_ADDR0(
		store_ch1->paddr + (store_ch1->left + store_ch1->top *
		store_ch1->stride) * 2),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_0_CH1);

	__raw_writel(BF_PXP_WFE_B_STORE_ADDR_1_CH1_OUT_BASE_ADDR1(0),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_1_CH1);

	/* ALU */
	__raw_writel(BF_PXP_ALU_B_BUF_SIZE_BUF_WIDTH(fetch_ch0->width) |
	        BF_PXP_ALU_B_BUF_SIZE_BUF_HEIGHT(fetch_ch0->height),
		pxp->base + HW_PXP_ALU_B_BUF_SIZE);

	/* WFE */
	__raw_writel(BF_PXP_WFE_B_DIMENSIONS_WIDTH(fetch_ch0->width) |
		BF_PXP_WFE_B_DIMENSIONS_HEIGHT(fetch_ch0->height),
		pxp->base + HW_PXP_WFE_B_DIMENSIONS);

	/* Here it should be fetch_ch1 */
	__raw_writel(BF_PXP_WFE_B_OFFSET_X_OFFSET(fetch_ch1->left) |
		BF_PXP_WFE_B_OFFSET_Y_OFFSET(fetch_ch1->top),
		pxp->base + HW_PXP_WFE_B_OFFSET);

	__raw_writel((proc_data->lut & 0x000000FF) | 0x00000F00,
			pxp->base + HW_PXP_WFE_B_SW_DATA_REGS);
	__raw_writel((proc_data->partial_update | (proc_data->reagl_en << 1)),
			pxp->base + HW_PXP_WFE_B_SW_FLAG_REGS);

	__raw_writel(
		BF_PXP_WFE_B_CTRL_ENABLE(1) |
		BF_PXP_WFE_B_CTRL_SW_RESET(1),
		pxp->base + HW_PXP_WFE_B_CTRL);

       if (proc_data->alpha_en) {
		__raw_writel(BF_PXP_WFB_ARRAY_FLAG0_MASK_SIGN_Y(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_OFFSET_Y(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_SIGN_X(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_OFFSET_X(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_BUF_SEL(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_H_OFS(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_L_OFS(0),
			pxp->base + HW_PXP_WFB_ARRAY_FLAG0_MASK);
        } else {
		__raw_writel(BF_PXP_WFB_ARRAY_FLAG0_MASK_SIGN_Y(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_OFFSET_Y(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_SIGN_X(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_OFFSET_X(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_BUF_SEL(2) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_H_OFS(0) |
			BF_PXP_WFB_ARRAY_FLAG0_MASK_L_OFS(0),
			pxp->base + HW_PXP_WFB_ARRAY_FLAG0_MASK);
        }

	/* disable CH1 when only doing detection */
	v = __raw_readl(pxp->base + HW_PXP_WFE_B_STORE_CTRL_CH1);
	if (proc_data->detection_only) {
		v &= ~BF_PXP_WFE_B_STORE_CTRL_CH1_CH_EN(1);
		printk(KERN_EMERG "%s: detection only happens\n", __func__);
	} else
		v |= BF_PXP_WFE_B_STORE_CTRL_CH1_CH_EN(1);
	__raw_writel(v, pxp->base + HW_PXP_WFE_B_STORE_CTRL_CH1);
}

/*
 * wfe b configuration
 *
 * configure wfe b engnine for reagl/-d waveform processing
 */
static void pxp_wfe_b_configure(struct pxps *pxp)
{
	/* Fetch */
	__raw_writel(
		BF_PXP_WFB_FETCH_CTRL_BF1_EN(1) |
		BF_PXP_WFB_FETCH_CTRL_BF1_HSK_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_BYTES_PP(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_LINE_MODE(1) |
		BF_PXP_WFB_FETCH_CTRL_BF1_SRAM_IF(1) |
		BF_PXP_WFB_FETCH_CTRL_BF1_BURST_LEN(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_BYPASS_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF1_BORDER_MODE(1) |
		BF_PXP_WFB_FETCH_CTRL_BF2_EN(1) |
		BF_PXP_WFB_FETCH_CTRL_BF2_HSK_MODE(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_BYTES_PP(1) |
		BF_PXP_WFB_FETCH_CTRL_BF2_LINE_MODE(1) |
		BF_PXP_WFB_FETCH_CTRL_BF2_SRAM_IF(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_BURST_LEN(0) |
		BF_PXP_WFB_FETCH_CTRL_BF2_BORDER_MODE(1) |
		BF_PXP_WFB_FETCH_CTRL_BF2_BYPASS_MODE(0),
		pxp->base + HW_PXP_WFB_FETCH_CTRL);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_H_OFS(0) |
		BF_PXP_WFB_ARRAY_PIXEL0_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL0_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_H_OFS(10) |
		BF_PXP_WFB_ARRAY_PIXEL1_MASK_L_OFS(15),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL1_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_BUF_SEL(0) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_H_OFS(2) |
		BF_PXP_WFB_ARRAY_PIXEL2_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL2_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_BUF_SEL(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_H_OFS(0) |
		BF_PXP_WFB_ARRAY_PIXEL3_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL3_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_SIGN_X(1) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_OFFSET_X(1) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_BUF_SEL(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_H_OFS(0) |
		BF_PXP_WFB_ARRAY_PIXEL4_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL4_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL5_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL5_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL5_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL5_MASK_OFFSET_X(1) |
		BF_PXP_WFB_ARRAY_PIXEL5_MASK_BUF_SEL(0) |
		BF_PXP_WFB_ARRAY_PIXEL5_MASK_H_OFS(0) |
		BF_PXP_WFB_ARRAY_PIXEL5_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL5_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL6_MASK_SIGN_Y(1) |
		BF_PXP_WFB_ARRAY_PIXEL6_MASK_OFFSET_Y(1) |
		BF_PXP_WFB_ARRAY_PIXEL6_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL6_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL6_MASK_BUF_SEL(0) |
		BF_PXP_WFB_ARRAY_PIXEL6_MASK_H_OFS(0) |
		BF_PXP_WFB_ARRAY_PIXEL6_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL6_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_PIXEL7_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_PIXEL7_MASK_OFFSET_Y(1) |
		BF_PXP_WFB_ARRAY_PIXEL7_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL7_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_PIXEL7_MASK_BUF_SEL(0) |
		BF_PXP_WFB_ARRAY_PIXEL7_MASK_H_OFS(0) |
		BF_PXP_WFB_ARRAY_PIXEL7_MASK_L_OFS(7),
		pxp->base + HW_PXP_WFB_ARRAY_PIXEL7_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG0_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG0_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG0_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG0_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_FLAG0_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG0_MASK_H_OFS(8) |
		BF_PXP_WFB_ARRAY_FLAG0_MASK_L_OFS(8),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG0_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG1_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG1_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG1_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG1_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_FLAG1_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG1_MASK_H_OFS(9) |
		BF_PXP_WFB_ARRAY_FLAG1_MASK_L_OFS(9),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG1_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG2_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG2_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG2_MASK_SIGN_X(1) |
		BF_PXP_WFB_ARRAY_FLAG2_MASK_OFFSET_X(1) |
		BF_PXP_WFB_ARRAY_FLAG2_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG2_MASK_H_OFS(8) |
		BF_PXP_WFB_ARRAY_FLAG2_MASK_L_OFS(8),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG2_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG3_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG3_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG3_MASK_SIGN_X(1) |
		BF_PXP_WFB_ARRAY_FLAG3_MASK_OFFSET_X(1) |
		BF_PXP_WFB_ARRAY_FLAG3_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG3_MASK_H_OFS(9) |
		BF_PXP_WFB_ARRAY_FLAG3_MASK_L_OFS(9),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG3_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG4_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG4_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG4_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG4_MASK_OFFSET_X(1) |
		BF_PXP_WFB_ARRAY_FLAG4_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG4_MASK_H_OFS(8) |
		BF_PXP_WFB_ARRAY_FLAG4_MASK_L_OFS(8),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG4_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG5_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG5_MASK_OFFSET_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG5_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG5_MASK_OFFSET_X(1) |
		BF_PXP_WFB_ARRAY_FLAG5_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG5_MASK_H_OFS(9) |
		BF_PXP_WFB_ARRAY_FLAG5_MASK_L_OFS(9),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG5_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG6_MASK_SIGN_Y(1) |
		BF_PXP_WFB_ARRAY_FLAG6_MASK_OFFSET_Y(1) |
		BF_PXP_WFB_ARRAY_FLAG6_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG6_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_FLAG6_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG6_MASK_H_OFS(8) |
		BF_PXP_WFB_ARRAY_FLAG6_MASK_L_OFS(8),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG6_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG7_MASK_SIGN_Y(1) |
		BF_PXP_WFB_ARRAY_FLAG7_MASK_OFFSET_Y(1) |
		BF_PXP_WFB_ARRAY_FLAG7_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG7_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_FLAG7_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG7_MASK_H_OFS(9) |
		BF_PXP_WFB_ARRAY_FLAG7_MASK_L_OFS(9),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG7_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG8_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG8_MASK_OFFSET_Y(1) |
		BF_PXP_WFB_ARRAY_FLAG8_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG8_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_FLAG8_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG8_MASK_H_OFS(8) |
		BF_PXP_WFB_ARRAY_FLAG8_MASK_L_OFS(8),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG8_MASK);

	__raw_writel(
		BF_PXP_WFB_ARRAY_FLAG9_MASK_SIGN_Y(0) |
		BF_PXP_WFB_ARRAY_FLAG9_MASK_OFFSET_Y(1) |
		BF_PXP_WFB_ARRAY_FLAG9_MASK_SIGN_X(0) |
		BF_PXP_WFB_ARRAY_FLAG9_MASK_OFFSET_X(0) |
		BF_PXP_WFB_ARRAY_FLAG9_MASK_BUF_SEL(1) |
		BF_PXP_WFB_ARRAY_FLAG9_MASK_H_OFS(9) |
		BF_PXP_WFB_ARRAY_FLAG9_MASK_L_OFS(9),
		pxp->base + HW_PXP_WFB_ARRAY_FLAG9_MASK);

	pxp_sram_init(pxp, WFE_B, (u32)active_matrix_data_8x8, 64);

	/* Store */
	__raw_writel(
		BF_PXP_WFE_B_STORE_CTRL_CH0_CH_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_BLOCK_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_BLOCK_16(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_HANDSHAKE_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARRAY_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARRAY_LINE_NUM(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_STORE_BYPASS_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_STORE_MEMORY_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_PACK_IN_SEL(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_FILL_DATA_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_WR_NUM_BYTES(32)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_COMBINE_2CHANNEL(1) |
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARBIT_EN(0),
		pxp->base + HW_PXP_WFE_B_STORE_CTRL_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_CTRL_CH1_CH_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_BLOCK_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_BLOCK_16(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_HANDSHAKE_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_ARRAY_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_ARRAY_LINE_NUM(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_STORE_BYPASS_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_STORE_MEMORY_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_PACK_IN_SEL(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_WR_NUM_BYTES(32),
		pxp->base + HW_PXP_WFE_B_STORE_CTRL_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUTPUT_ACTIVE_BPP(1)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_2P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_SHIFT_BYPASS(0),
		pxp->base + HW_PXP_WFE_B_STORE_SHIFT_CTRL_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUTPUT_ACTIVE_BPP(1)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_2P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_SHIFT_BYPASS(0),
		pxp->base + HW_PXP_WFE_B_STORE_SHIFT_CTRL_CH1);

	__raw_writel(BF_PXP_WFE_B_STORE_ADDR_1_CH0_OUT_BASE_ADDR1(0),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_1_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_ADDR_0_CH1_OUT_BASE_ADDR0(0),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_0_CH1);

	__raw_writel(BF_PXP_WFE_B_STORE_ADDR_1_CH1_OUT_BASE_ADDR1(0),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_1_CH1);

	__raw_writel(BF_PXP_WFE_B_STORE_FILL_DATA_CH0_FILL_DATA_CH0(0),
		pxp->base + HW_PXP_WFE_B_STORE_FILL_DATA_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK0_H_CH0_D_MASK0_H_CH0(0x00000000),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK0_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK0_L_CH0_D_MASK0_L_CH0(0xff),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK0_L_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK1_H_CH0_D_MASK1_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK1_L_CH0_D_MASK1_L_CH0(0x3f00),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_L_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK2_H_CH0_D_MASK2_H_CH0(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_H_CH0);

	__raw_writel(BF_PXP_WFE_B_STORE_D_MASK2_L_CH0_D_MASK2_L_CH0(0x0),
		pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH4(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG4(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH5(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG5(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH6(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG6(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_WIDTH7(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_H_CH0_D_SHIFT_FLAG7(0),
		pxp->base + HW_PXP_WFE_B_STORE_D_SHIFT_H_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH0(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG0(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH1(2)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG1(1) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH2(6)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG2(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH3(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG3(0),
		pxp->base + HW_PXP_WFE_B_STORE_D_SHIFT_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH0(8)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG0(1)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH1(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG1(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH2(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG2(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH3(0)|
		BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG3(0),
		pxp->base + HW_PXP_WFE_B_STORE_F_SHIFT_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK4(0)|
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK5(0)|
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK6(0)|
		BF_PXP_WFE_B_STORE_F_MASK_H_CH0_F_MASK7(0),
		pxp->base + HW_PXP_WFE_B_STORE_F_MASK_H_CH0);

	/* ALU */
	__raw_writel(BF_PXP_ALU_B_INST_ENTRY_ENTRY_ADDR(0),
		pxp->base + HW_PXP_ALU_B_INST_ENTRY);

	__raw_writel(BF_PXP_ALU_B_PARAM_PARAM0(0) |
		BF_PXP_ALU_B_PARAM_PARAM1(0),
		pxp->base + HW_PXP_ALU_B_PARAM);

	__raw_writel(BF_PXP_ALU_B_CONFIG_BUF_ADDR(0),
		pxp->base + HW_PXP_ALU_B_CONFIG);

	__raw_writel(BF_PXP_ALU_B_LUT_CONFIG_MODE(0) |
		BF_PXP_ALU_B_LUT_CONFIG_EN(0),
		pxp->base + HW_PXP_ALU_B_LUT_CONFIG);

	__raw_writel(BF_PXP_ALU_B_LUT_DATA0_LUT_DATA_L(0),
		pxp->base + HW_PXP_ALU_B_LUT_DATA0);

	__raw_writel(BF_PXP_ALU_B_LUT_DATA1_LUT_DATA_H(0),
		pxp->base + HW_PXP_ALU_B_LUT_DATA1);

	__raw_writel(
		BF_PXP_ALU_B_CTRL_BYPASS    (1) |
		BF_PXP_ALU_B_CTRL_ENABLE    (1) |
		BF_PXP_ALU_B_CTRL_START     (0) |
		BF_PXP_ALU_B_CTRL_SW_RESET  (0),
		pxp->base + HW_PXP_ALU_B_CTRL);

	/* WFE */
	__raw_writel(0x00000402, pxp->base + HW_PXP_WFE_B_SW_DATA_REGS);

	__raw_writel(0x02040608, pxp->base + HW_PXP_WFE_B_STAGE1_MUX0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX4);
	__raw_writel(0x03000000, pxp->base + HW_PXP_WFE_B_STAGE1_MUX5);
	__raw_writel(0x050A040A, pxp->base + HW_PXP_WFE_B_STAGE1_MUX6);
	__raw_writel(0x070A060A, pxp->base + HW_PXP_WFE_B_STAGE1_MUX7);
	__raw_writel(0x0000000A, pxp->base + HW_PXP_WFE_B_STAGE1_MUX8);

	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX0);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX1);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX2);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX4);
	__raw_writel(0x1C1E2022, pxp->base + HW_PXP_WFE_B_STAGE2_MUX5);
	__raw_writel(0x1215181A, pxp->base + HW_PXP_WFE_B_STAGE2_MUX6);
	__raw_writel(0x00000C0F, pxp->base + HW_PXP_WFE_B_STAGE2_MUX7);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX8);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX9);
	__raw_writel(0x01000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX10);
	__raw_writel(0x000C010B, pxp->base + HW_PXP_WFE_B_STAGE2_MUX11);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE2_MUX12);

	__raw_writel(0x09000C01, pxp->base + HW_PXP_WFE_B_STAGE3_MUX0);
	__raw_writel(0x003A2A1D, pxp->base + HW_PXP_WFE_B_STAGE3_MUX1);
	__raw_writel(0x09000C01, pxp->base + HW_PXP_WFE_B_STAGE3_MUX2);
	__raw_writel(0x003A2A1D, pxp->base + HW_PXP_WFE_B_STAGE3_MUX3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE3_MUX4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE3_MUX5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE3_MUX6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STAGE3_MUX7);
	__raw_writel(0x07060504, pxp->base + HW_PXP_WFE_B_STAGE3_MUX8);
	__raw_writel(0x00000008, pxp->base + HW_PXP_WFE_B_STAGE3_MUX9);
	__raw_writel(0x00001211, pxp->base + HW_PXP_WFE_B_STAGE3_MUX10);

	__raw_writel(0x02010100, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_0);
	__raw_writel(0x03020201, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_1);
	__raw_writel(0x03020201, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_2);
	__raw_writel(0x04030302, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT0_7);

	__raw_writel(0x02010100, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_0);
	__raw_writel(0x03020201, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_1);
	__raw_writel(0x03020201, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_2);
	__raw_writel(0x04030302, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_3);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_4);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_5);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_6);
	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X8_OUT1_7);

	__raw_writel(0x0000000F, pxp->base + HW_PXP_WFE_B_STAGE1_5X8_MASKS_0);

	__raw_writel(0x00000000, pxp->base + HW_PXP_WFE_B_STG1_5X1_OUT0);
	__raw_writel(0x0000000F, pxp->base + HW_PXP_WFE_B_STG1_5X1_MASKS);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT2_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT3_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT4_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STAGE2_5X6_MASKS_0);
	__raw_writel(0x3F3F3F3F, pxp->base + HW_PXP_WFE_B_STAGE2_5X6_ADDR_0);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT0_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_0);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X6_OUT1_7);

	__raw_writel(0x00008000, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT0);
	__raw_writel(0x0000FFFE, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT2);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG2_5X1_OUT3);
	__raw_writel(0x00000F0F, pxp->base + HW_PXP_WFE_B_STG2_5X1_MASKS);

	__raw_writel(0x00007F7F, pxp->base + HW_PXP_WFE_B_STG3_F8X1_MASKS);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_0);
	__raw_writel(0x00FF00FF, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_2);
	__raw_writel(0x000000FF, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT0_7);

	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_0);
	__raw_writel(0xFF3FFF3F, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_1);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_2);
	__raw_writel(0xFFFFFF1F, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_3);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_4);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_5);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_6);
	__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG3_F8X1_OUT1_7);

	__raw_writel(
		BF_PXP_WFE_B_CTRL_ENABLE(1) |
		BF_PXP_WFE_B_CTRL_SW_RESET(1),
		pxp->base + HW_PXP_WFE_B_CTRL);
}

/* wfe b processing
 * use wfe b to process an update
 * call this function only after pxp_wfe_a_processing
 * x,y,width,height:
 *         coordinate and size of the update region
 * twb:
 *         temp working buffer, 16bpp
 *         only used when reagl_en is 1
 * wb:
 *         working buffer, 16bpp
 * lut:
 *         lut buffer, 8bpp
 * lut_update:
 *         0 - wfe_b is used for reagl/reagl-d operation
 *         1 - wfe_b is used for lut update operation
 * reagl_d_en:
 *         0 - use reagl waveform algorithm
 *         1 - use reagl/-d waveform algorithm
 */
static void pxp_wfe_b_process(struct pxps *pxp)
{
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &config_data->proc_data;
	struct pxp_layer_param *fetch_ch0 = &config_data->wfe_b_fetch_param[0];
	struct pxp_layer_param *fetch_ch1 = &config_data->wfe_b_fetch_param[1];
	struct pxp_layer_param *store_ch0 = &config_data->wfe_b_store_param[0];
	struct pxp_layer_param *store_ch1 = &config_data->wfe_b_store_param[1];
	static int comp_mask;
	/* Fetch */

	print_param(fetch_ch0, "wfe_b fetch_ch0");
	print_param(fetch_ch1, "wfe_b fetch_ch1");
	print_param(store_ch0, "wfe_b store_ch0");
	print_param(store_ch1, "wfe_b store_ch1");

	__raw_writel(fetch_ch0->paddr, pxp->base + HW_PXP_WFB_FETCH_BUF1_ADDR);

	__raw_writel(
		BF_PXP_WFB_FETCH_BUF1_CORD_YCORD(fetch_ch0->top) |
		BF_PXP_WFB_FETCH_BUF1_CORD_XCORD(fetch_ch0->left),
		pxp->base + HW_PXP_WFB_FETCH_BUF1_CORD);

	__raw_writel(fetch_ch0->stride,
		pxp->base + HW_PXP_WFB_FETCH_BUF1_PITCH);

	__raw_writel(
		BF_PXP_WFB_FETCH_BUF1_SIZE_BUF_HEIGHT(fetch_ch0->height-1) |
		BF_PXP_WFB_FETCH_BUF1_SIZE_BUF_WIDTH(fetch_ch0->width-1),
		pxp->base + HW_PXP_WFB_FETCH_BUF1_SIZE);

	__raw_writel(fetch_ch1->paddr, pxp->base + HW_PXP_WFB_FETCH_BUF2_ADDR);

	__raw_writel(fetch_ch1->stride * 2,
			pxp->base + HW_PXP_WFB_FETCH_BUF2_PITCH);

	__raw_writel(
		BF_PXP_WFB_FETCH_BUF2_CORD_YCORD(fetch_ch1->top) |
		BF_PXP_WFB_FETCH_BUF2_CORD_XCORD(fetch_ch1->left),
		pxp->base + HW_PXP_WFB_FETCH_BUF2_CORD);

	__raw_writel(
		BF_PXP_WFB_FETCH_BUF2_SIZE_BUF_HEIGHT(fetch_ch1->height-1) |
		BF_PXP_WFB_FETCH_BUF2_SIZE_BUF_WIDTH(fetch_ch1->width-1),
		pxp->base + HW_PXP_WFB_FETCH_BUF2_SIZE);

	if (!proc_data->lut_update) {
		__raw_writel(
			BF_PXP_WFB_FETCH_CTRL_BF1_EN(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_HSK_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BYTES_PP(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_LINE_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_SRAM_IF(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BURST_LEN(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BYPASS_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BORDER_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_EN(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_HSK_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BYTES_PP(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_LINE_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_SRAM_IF(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BURST_LEN(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BORDER_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BYPASS_MODE(0),
			pxp->base + HW_PXP_WFB_FETCH_CTRL);
	} else {
		__raw_writel(
			BF_PXP_WFB_FETCH_CTRL_BF1_EN(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_HSK_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BYTES_PP(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_LINE_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_SRAM_IF(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BURST_LEN(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BORDER_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BYPASS_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_EN(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_HSK_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BYTES_PP(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_LINE_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_SRAM_IF(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BURST_LEN(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BORDER_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BYPASS_MODE(0),
			pxp->base + HW_PXP_WFB_FETCH_CTRL);
	}

#ifdef	CONFIG_REAGLD_ALGO_CHECK
	__raw_writel(
			(__raw_readl(pxp->base + HW_PXP_WFE_B_SW_DATA_REGS) & 0x0000FFFF) | ((fetch_ch0->comp_mask&0x000000FF)<<16),
			pxp->base + HW_PXP_WFE_B_SW_DATA_REGS);
#else
	__raw_writel(
			(__raw_readl(pxp->base + HW_PXP_WFE_B_SW_DATA_REGS) & 0x0000FFFF) | ((comp_mask&0x000000FF)<<16),
			pxp->base + HW_PXP_WFE_B_SW_DATA_REGS);

	/* comp_mask only need to be updated upon REAGL-D, 0,1,...7, 0,1,...  */
	if (proc_data->reagl_d_en) {
		comp_mask++;
		if (comp_mask>7)
			comp_mask = 0;
	}
#endif

	/* Store */
	__raw_writel(
		BF_PXP_WFE_B_STORE_SIZE_CH0_OUT_WIDTH(store_ch0->width-1)|
		BF_PXP_WFE_B_STORE_SIZE_CH0_OUT_HEIGHT(store_ch0->height-1),
		pxp->base + HW_PXP_WFE_B_STORE_SIZE_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SIZE_CH1_OUT_WIDTH(store_ch1->width-1)|
		BF_PXP_WFE_B_STORE_SIZE_CH1_OUT_HEIGHT(store_ch1->height-1),
		pxp->base + HW_PXP_WFE_B_STORE_SIZE_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_PITCH_CH0_OUT_PITCH(store_ch0->stride * 2)|
		BF_PXP_WFE_B_STORE_PITCH_CH1_OUT_PITCH(store_ch1->stride * 2),
		pxp->base + HW_PXP_WFE_B_STORE_PITCH);

	__raw_writel(
		BF_PXP_WFE_B_STORE_ADDR_0_CH0_OUT_BASE_ADDR0(store_ch0->paddr
			+ (store_ch0->left + store_ch0->top * store_ch0->stride) * 2),
		pxp->base + HW_PXP_WFE_B_STORE_ADDR_0_CH0);

	if (proc_data->lut_update) {
		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK1_H_CH0_D_MASK1_H_CH0(0x0),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_H_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK1_L_CH0_D_MASK1_L_CH0(0x0),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_L_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK2_H_CH0_D_MASK2_H_CH0(0x0),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_H_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK2_L_CH0_D_MASK2_L_CH0(0x3f0000),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_L_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK0(0x30)|
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK1(0)|
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK2(0)|
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK3(0),
			pxp->base + HW_PXP_WFE_B_STORE_F_MASK_L_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH0(4)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG0(1)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH1(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG1(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH2(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG2(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH3(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG3(0),
			pxp->base + HW_PXP_WFE_B_STORE_F_SHIFT_L_CH0);
	} else {
		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK1_H_CH0_D_MASK1_H_CH0(0x0),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_H_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK1_L_CH0_D_MASK1_L_CH0(0x3f00),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK1_L_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK2_H_CH0_D_MASK2_H_CH0(0x0),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_H_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_D_MASK2_L_CH0_D_MASK2_L_CH0(0x0),
			pxp->base + HW_PXP_WFE_B_STORE_D_MASK2_L_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK0(3)|
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK1(0)|
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK2(0)|
			BF_PXP_WFE_B_STORE_F_MASK_L_CH0_F_MASK3(0),
			pxp->base + HW_PXP_WFE_B_STORE_F_MASK_L_CH0);

		__raw_writel(
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH0(8)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG0(1)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH1(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG1(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH2(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG2(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_WIDTH3(0)|
			BF_PXP_WFE_B_STORE_F_SHIFT_L_CH0_F_SHIFT_FLAG3(0),
			pxp->base + HW_PXP_WFE_B_STORE_F_SHIFT_L_CH0);
	}

	/* ALU */
	__raw_writel(
		BF_PXP_ALU_B_BUF_SIZE_BUF_WIDTH(fetch_ch0->width) |
		BF_PXP_ALU_B_BUF_SIZE_BUF_HEIGHT(fetch_ch0->height),
		pxp->base + HW_PXP_ALU_B_BUF_SIZE);

	/* WFE */
	__raw_writel(
		BF_PXP_WFE_B_DIMENSIONS_WIDTH(fetch_ch0->width) |
		BF_PXP_WFE_B_DIMENSIONS_HEIGHT(fetch_ch0->height),
		pxp->base + HW_PXP_WFE_B_DIMENSIONS);

	__raw_writel(	/*TODO check*/
		BF_PXP_WFE_B_OFFSET_X_OFFSET(fetch_ch0->left) |
		BF_PXP_WFE_B_OFFSET_Y_OFFSET(fetch_ch0->top),
		pxp->base + HW_PXP_WFE_B_OFFSET);

	__raw_writel(proc_data->reagl_d_en, pxp->base + HW_PXP_WFE_B_SW_FLAG_REGS);
}

void pxp_fill(
        u32 bpp,
        u32 value,
        u32 width,
        u32 height,
        u32 output_buffer,
        u32 output_pitch)
{
	u32 active_bpp;
	u32 pitch;

	if (bpp == 8) {
		active_bpp = 0;
		pitch = output_pitch;
	} else if(bpp == 16) {
		active_bpp = 1;
		pitch = output_pitch * 2;
	} else {
		active_bpp = 2;
		pitch = output_pitch * 4;
	}

	__raw_writel(
		BF_PXP_WFE_B_STORE_CTRL_CH0_CH_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_BLOCK_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_BLOCK_16(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_HANDSHAKE_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARRAY_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARRAY_LINE_NUM(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_STORE_BYPASS_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_STORE_MEMORY_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_PACK_IN_SEL(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_FILL_DATA_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_WR_NUM_BYTES(32)|
		BF_PXP_WFE_B_STORE_CTRL_CH0_COMBINE_2CHANNEL(0) |
		BF_PXP_WFE_B_STORE_CTRL_CH0_ARBIT_EN(0),
		pxp_reg_base + HW_PXP_WFE_B_STORE_CTRL_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_CTRL_CH1_CH_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_BLOCK_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_BLOCK_16(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_HANDSHAKE_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_ARRAY_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_ARRAY_LINE_NUM(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_STORE_BYPASS_EN(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_STORE_MEMORY_EN(1)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_PACK_IN_SEL(0)|
		BF_PXP_WFE_B_STORE_CTRL_CH1_WR_NUM_BYTES(16),
		pxp_reg_base + HW_PXP_WFE_B_STORE_CTRL_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SIZE_CH0_OUT_WIDTH(width-1)|
		BF_PXP_WFE_B_STORE_SIZE_CH0_OUT_HEIGHT(height-1),
		pxp_reg_base + HW_PXP_WFE_B_STORE_SIZE_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SIZE_CH1_OUT_WIDTH(width-1)|
		BF_PXP_WFE_B_STORE_SIZE_CH1_OUT_HEIGHT(height-1),
		pxp_reg_base + HW_PXP_WFE_B_STORE_SIZE_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_PITCH_CH0_OUT_PITCH(pitch)|
		BF_PXP_WFE_B_STORE_PITCH_CH1_OUT_PITCH(pitch),
		pxp_reg_base + HW_PXP_WFE_B_STORE_PITCH);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUTPUT_ACTIVE_BPP(active_bpp)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_OUT_YUV422_2P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH0_SHIFT_BYPASS(1),
		pxp_reg_base + HW_PXP_WFE_B_STORE_SHIFT_CTRL_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH1_OUTPUT_ACTIVE_BPP(active_bpp)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH1_OUT_YUV422_1P_EN(0)|
		BF_PXP_WFE_B_STORE_SHIFT_CTRL_CH1_OUT_YUV422_2P_EN(0),
		pxp_reg_base + HW_PXP_WFE_B_STORE_SHIFT_CTRL_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_ADDR_0_CH0_OUT_BASE_ADDR0(output_buffer),
		pxp_reg_base + HW_PXP_WFE_B_STORE_ADDR_0_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_ADDR_1_CH0_OUT_BASE_ADDR1(0),
		pxp_reg_base + HW_PXP_WFE_B_STORE_ADDR_1_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_ADDR_0_CH1_OUT_BASE_ADDR0(output_buffer),
		pxp_reg_base + HW_PXP_WFE_B_STORE_ADDR_0_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_ADDR_1_CH1_OUT_BASE_ADDR1(0),
		pxp_reg_base + HW_PXP_WFE_B_STORE_ADDR_1_CH1);

	__raw_writel(
		BF_PXP_WFE_B_STORE_FILL_DATA_CH0_FILL_DATA_CH0(value),
		pxp_reg_base + HW_PXP_WFE_B_STORE_FILL_DATA_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_MASK0_H_CH0_D_MASK0_H_CH0(0x00000000),
		pxp_reg_base + HW_PXP_WFE_B_STORE_D_MASK0_H_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_MASK0_L_CH0_D_MASK0_L_CH0(0x000000ff),
		pxp_reg_base + HW_PXP_WFE_B_STORE_D_MASK0_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_MASK1_H_CH0_D_MASK1_H_CH0(0x00000000),
		pxp_reg_base + HW_PXP_WFE_B_STORE_D_MASK1_H_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_MASK1_L_CH0_D_MASK1_L_CH0(0x000000ff),
		pxp_reg_base + HW_PXP_WFE_B_STORE_D_MASK1_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_MASK2_H_CH0_D_MASK2_H_CH0(0x00000000),
		pxp_reg_base + HW_PXP_WFE_B_STORE_D_MASK2_H_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_MASK2_L_CH0_D_MASK2_L_CH0(0x000000ff),
		pxp_reg_base + HW_PXP_WFE_B_STORE_D_MASK2_L_CH0);

	__raw_writel(
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH0(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG0(0) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH1(32)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG1(1) |
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH2(40)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG2(1)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH3(0)|
		BF_PXP_WFE_B_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG3(0),
		pxp_reg_base +  HW_PXP_WFE_B_STORE_D_SHIFT_L_CH0);

	__raw_writel(
		BF_PXP_CTRL2_ENABLE                   (1) |
		BF_PXP_CTRL2_ROTATE0                  (0) |
		BF_PXP_CTRL2_HFLIP0                   (0) |
		BF_PXP_CTRL2_VFLIP0                   (0) |
		BF_PXP_CTRL2_ROTATE1                  (0) |
		BF_PXP_CTRL2_HFLIP1                   (0) |
		BF_PXP_CTRL2_VFLIP1                   (0) |
		BF_PXP_CTRL2_ENABLE_DITHER            (0) |
		BF_PXP_CTRL2_ENABLE_WFE_A             (0) |
		BF_PXP_CTRL2_ENABLE_WFE_B             (1) |
		BF_PXP_CTRL2_ENABLE_INPUT_FETCH_STORE (0) |
		BF_PXP_CTRL2_ENABLE_ALPHA_B           (0) |
		BF_PXP_CTRL2_BLOCK_SIZE               (0) |
		BF_PXP_CTRL2_ENABLE_CSC2              (0) |
		BF_PXP_CTRL2_ENABLE_LUT               (0) |
		BF_PXP_CTRL2_ENABLE_ROTATE0           (0) |
		BF_PXP_CTRL2_ENABLE_ROTATE1           (0),
		pxp_reg_base + HW_PXP_CTRL2);

	if (busy_wait(BM_PXP_IRQ_WFE_B_CH0_STORE_IRQ &
			__raw_readl(pxp_reg_base + HW_PXP_IRQ)) == false)
		printk("%s: wait for completion timeout\n", __func__);
}
EXPORT_SYMBOL(pxp_fill);

static void pxp_lut_cleanup_multiple(struct pxps *pxp, u64 lut, bool set)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;

	if (proc_data->lut_cleanup == 1) {
		if (set) {
			__raw_writel((u32)lut, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_0 + 0x4);
			__raw_writel((u32)(lut>>32), pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_1 + 0x4);
		} else {
			pxp_luts_deactivate(pxp, lut);
			__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_0);
			__raw_writel(0, pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT1_1);
		}
	}
}

static void pxp_lut_cleanup_multiple_v3p(struct pxps *pxp, u64 lut, bool set)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;

	if (proc_data->lut_cleanup == 1) {
		if (set) {
			__raw_writel((u32)lut, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_0 + 0x4);
			__raw_writel((u32)(lut>>32), pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_1 + 0x4);
		} else {
			pxp_luts_deactivate(pxp, lut);
			__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_0);
			__raw_writel(0, pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT1_1);
		}
	}
}

#ifdef CONFIG_MXC_FPGA_M4_TEST
void m4_process(void)
{
	__raw_writel(0x7, pinctrl_base + PIN_DOUT);	/* M4 Start */

	while (!(__raw_readl(pxp_reg_base + HW_PXP_HANDSHAKE_CPU_STORE) & BM_PXP_HANDSHAKE_CPU_STORE_SW0_B0_READY));

	__raw_writel(0x3, pinctrl_base + PIN_DOUT);	/* M4 Stop */


}
#else
void m4_process(void) {}
#endif
EXPORT_SYMBOL(m4_process);

static void pxp_lut_status_set(struct pxps *pxp, unsigned int lut)
{
	if(lut<32)
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_A_STG1_8X1_OUT0_0) | (1 << lut),
				pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_0);
	else {
		lut = lut -32;
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_A_STG1_8X1_OUT0_1) | (1 << lut),
				pxp->base + HW_PXP_WFE_A_STG1_8X1_OUT0_1);
	}
}

static void pxp_lut_status_set_v3p(struct pxps *pxp, unsigned int lut)
{
	if(lut<32)
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_B_STG1_8X1_OUT0_0) | (1 << lut),
				pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_0);
	else {
		lut = lut -32;
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_B_STG1_8X1_OUT0_1) | (1 << lut),
				pxp->base + HW_PXP_WFE_B_STG1_8X1_OUT0_1);
	}
}

static void pxp_luts_activate(struct pxps *pxp, u64 lut_status)
{
	int i = 0;

	if (!lut_status)
		return;

	for (i = 0; i < 64; i++) {
		if (lut_status & (1ULL << i))
			if (pxp->devdata && pxp->devdata->pxp_lut_status_set)
				pxp->devdata->pxp_lut_status_set(pxp, i);
	}
}

static void pxp_lut_status_clr(unsigned int lut)
{
	if(lut<32)
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_A_STG1_8X1_OUT0_0) & (~(1 << lut)),
				pxp_reg_base + HW_PXP_WFE_A_STG1_8X1_OUT0_0);
	else
	{
		lut = lut -32;
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_A_STG1_8X1_OUT0_1) & (~(1 << lut)),
				pxp_reg_base + HW_PXP_WFE_A_STG1_8X1_OUT0_1);
	}
}

static void pxp_lut_status_clr_v3p(unsigned int lut)
{
	if(lut<32)
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_B_STG1_8X1_OUT0_0) & (~(1 << lut)),
				pxp_reg_base + HW_PXP_WFE_B_STG1_8X1_OUT0_0);
	else
	{
		lut = lut -32;
		__raw_writel(
				__raw_readl(pxp_reg_base + HW_PXP_WFE_B_STG1_8X1_OUT0_1) & (~(1 << lut)),
				pxp_reg_base + HW_PXP_WFE_B_STG1_8X1_OUT0_1);
	}
}

/* this function should be called in the epdc
 * driver explicitly when some epdc lut becomes
 * idle. So it should be exported.
 */
static void pxp_luts_deactivate(struct pxps *pxp, u64 lut_status)
{
	int i = 0;

	if (!lut_status)
		return;

	for (i = 0; i < 64; i++) {
		if (lut_status & (1ULL << i))
			if (pxp->devdata && pxp->devdata->pxp_lut_status_clr)
				pxp->devdata->pxp_lut_status_clr(i);
	}
}

/* use histogram_B engine to calculate histogram status */
static void pxp_histogram_enable(struct pxps *pxp,
				 unsigned int width,
				 unsigned int height)
{
	__raw_writel(
			BF_PXP_HIST_B_BUF_SIZE_HEIGHT(height)|
			BF_PXP_HIST_B_BUF_SIZE_WIDTH(width),
			pxp->base + HW_PXP_HIST_B_BUF_SIZE);

	__raw_writel(
			BF_PXP_HIST_B_MASK_MASK_EN(1)|
			BF_PXP_HIST_B_MASK_MASK_MODE(0)|
			BF_PXP_HIST_B_MASK_MASK_OFFSET(64)|
			BF_PXP_HIST_B_MASK_MASK_WIDTH(0)|
			BF_PXP_HIST_B_MASK_MASK_VALUE0(1) |
			BF_PXP_HIST_B_MASK_MASK_VALUE1(0),
			pxp->base + HW_PXP_HIST_B_MASK);

	__raw_writel(
			BF_PXP_HIST_B_CTRL_PIXEL_WIDTH(3)|
			BF_PXP_HIST_B_CTRL_PIXEL_OFFSET(8)|
			BF_PXP_HIST_B_CTRL_CLEAR(0)|
			BF_PXP_HIST_B_CTRL_ENABLE(1),
			pxp->base + HW_PXP_HIST_B_CTRL);
}

static void pxp_histogram_status_report(struct pxps *pxp, u32 *hist_status)
{
	BUG_ON(!hist_status);

	*hist_status = (__raw_readl(pxp->base + HW_PXP_HIST_B_CTRL) & BM_PXP_HIST_B_CTRL_STATUS)
			>> BP_PXP_HIST_B_CTRL_STATUS;
	dev_dbg(pxp->dev, "%d pixels are used to calculate histogram status %d\n",
			__raw_readl(pxp->base + HW_PXP_HIST_B_TOTAL_PIXEL), *hist_status);
}

static void pxp_histogram_disable(struct pxps *pxp)
{
	__raw_writel(
			BF_PXP_HIST_B_CTRL_PIXEL_WIDTH(3)|
			BF_PXP_HIST_B_CTRL_PIXEL_OFFSET(4)|
			BF_PXP_HIST_B_CTRL_CLEAR(1)|
			BF_PXP_HIST_B_CTRL_ENABLE(0),
			pxp->base + HW_PXP_HIST_B_CTRL);
}

/* the collision detection function will be
 * called by epdc driver when required
 */
static void pxp_collision_detection_enable(struct pxps *pxp,
					   unsigned int width,
					   unsigned int height)
{
	__raw_writel(
			BF_PXP_HIST_A_BUF_SIZE_HEIGHT(height)|
			BF_PXP_HIST_A_BUF_SIZE_WIDTH(width),
			pxp_reg_base + HW_PXP_HIST_A_BUF_SIZE);

	__raw_writel(
			BF_PXP_HIST_A_MASK_MASK_EN(1)|
			BF_PXP_HIST_A_MASK_MASK_MODE(0)|
			BF_PXP_HIST_A_MASK_MASK_OFFSET(65)|
			BF_PXP_HIST_A_MASK_MASK_WIDTH(0)|
			BF_PXP_HIST_A_MASK_MASK_VALUE0(1) |
			BF_PXP_HIST_A_MASK_MASK_VALUE1(0),
			pxp_reg_base + HW_PXP_HIST_A_MASK);

	__raw_writel(
			BF_PXP_HIST_A_CTRL_PIXEL_WIDTH(6)|
			BF_PXP_HIST_A_CTRL_PIXEL_OFFSET(24)|
			BF_PXP_HIST_A_CTRL_CLEAR(0)|
			BF_PXP_HIST_A_CTRL_ENABLE(1),
			pxp_reg_base + HW_PXP_HIST_A_CTRL);
}

static void pxp_collision_detection_disable(struct pxps *pxp)
{
	__raw_writel(
			BF_PXP_HIST_A_CTRL_PIXEL_WIDTH(6)|
			BF_PXP_HIST_A_CTRL_PIXEL_OFFSET(24)|
			BF_PXP_HIST_A_CTRL_CLEAR(1)|
			BF_PXP_HIST_A_CTRL_ENABLE(0),
			pxp_reg_base + HW_PXP_HIST_A_CTRL);
}

/* this function can be called in the epdc callback
 * function in the pxp_irq() to let the epdc know
 * the collision information for the previous working
 * buffer update.
 */
static bool pxp_collision_status_report(struct pxps *pxp, struct pxp_collision_info *info)
{
	unsigned int count;

	BUG_ON(!info);
	memset(info, 0x0, sizeof(*info));

	info->pixel_cnt = count = __raw_readl(pxp->base + HW_PXP_HIST_A_TOTAL_PIXEL);
	if (!count)
		return false;

	dev_dbg(pxp->dev, "%s: pixel_cnt = %d\n", __func__, info->pixel_cnt);
	info->rect_min_x = __raw_readl(pxp->base + HW_PXP_HIST_A_ACTIVE_AREA_X) & 0xffff;
	dev_dbg(pxp->dev, "%s: rect_min_x = %d\n", __func__, info->rect_min_x);
	info->rect_max_x = (__raw_readl(pxp->base + HW_PXP_HIST_A_ACTIVE_AREA_X) >> 16) & 0xffff;
	dev_dbg(pxp->dev, "%s: rect_max_x = %d\n", __func__, info->rect_max_x);
	info->rect_min_y = __raw_readl(pxp->base + HW_PXP_HIST_A_ACTIVE_AREA_Y) & 0xffff;
	dev_dbg(pxp->dev, "%s: rect_min_y = %d\n", __func__, info->rect_min_y);
	info->rect_max_y = (__raw_readl(pxp->base + HW_PXP_HIST_A_ACTIVE_AREA_Y) >> 16) & 0xffff;
	dev_dbg(pxp->dev, "%s: rect_max_y = %d\n", __func__, info->rect_max_y);

	info->victim_luts[0] = __raw_readl(pxp->base + HW_PXP_HIST_A_RAW_STAT0);
	dev_dbg(pxp->dev, "%s: victim_luts[0] = 0x%x\n", __func__, info->victim_luts[0]);
	info->victim_luts[1] = __raw_readl(pxp->base + HW_PXP_HIST_A_RAW_STAT1);
	dev_dbg(pxp->dev, "%s: victim_luts[1] = 0x%x\n", __func__, info->victim_luts[1]);

	return true;
}

void pxp_get_collision_info(struct pxp_collision_info *info)
{
	BUG_ON(!info);

	memcpy(info, &col_info, sizeof(struct pxp_collision_info));
}
EXPORT_SYMBOL(pxp_get_collision_info);

static void dither_prefetch_config(struct pxps *pxp)
{
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_layer_param *fetch_ch0 = &config_data->dither_fetch_param[0];
	struct pxp_layer_param *fetch_ch1 = &config_data->dither_fetch_param[1];

	print_param(fetch_ch0, "dither fetch_ch0");
	print_param(fetch_ch1, "dither fetch_ch1");
	__raw_writel(
			BF_PXP_DITHER_FETCH_CTRL_CH0_CH_EN(1) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_BLOCK_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_BLOCK_16(0)|
			BF_PXP_DITHER_FETCH_CTRL_CH0_HANDSHAKE_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_BYPASS_PIXEL_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_HIGH_BYTE(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_HFLIP(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_VFLIP(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_ROTATION_ANGLE(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_RD_NUM_BYTES(32) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_HANDSHAKE_SCAN_LINE_NUM(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_ARBIT_EN(0),
			pxp->base + HW_PXP_DITHER_FETCH_CTRL_CH0);

	__raw_writel(
			BF_PXP_DITHER_FETCH_CTRL_CH1_CH_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_BLOCK_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_BLOCK_16(0)|
			BF_PXP_DITHER_FETCH_CTRL_CH1_HANDSHAKE_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_BYPASS_PIXEL_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_HFLIP(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_VFLIP(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_ROTATION_ANGLE(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_RD_NUM_BYTES(2) |
			BF_PXP_DITHER_FETCH_CTRL_CH1_HANDSHAKE_SCAN_LINE_NUM(0),
			pxp->base + HW_PXP_DITHER_FETCH_CTRL_CH1);

	__raw_writel(
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_ULC_CH0_ACTIVE_SIZE_ULC_X(0) |
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_ULC_CH0_ACTIVE_SIZE_ULC_Y(0),
			pxp->base + HW_PXP_DITHER_FETCH_ACTIVE_SIZE_ULC_CH0);
	__raw_writel(
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_LRC_CH0_ACTIVE_SIZE_LRC_X(fetch_ch0->width - 1) |
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_LRC_CH0_ACTIVE_SIZE_LRC_Y(fetch_ch0->height - 1),
			pxp->base + HW_PXP_DITHER_FETCH_ACTIVE_SIZE_LRC_CH0);

	__raw_writel(
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_ULC_CH1_ACTIVE_SIZE_ULC_X(0) |
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_ULC_CH1_ACTIVE_SIZE_ULC_Y(0),
			pxp->base + HW_PXP_DITHER_FETCH_ACTIVE_SIZE_ULC_CH1);
	__raw_writel(
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_LRC_CH1_ACTIVE_SIZE_LRC_X(fetch_ch1->width - 1) |
			BF_PXP_DITHER_FETCH_ACTIVE_SIZE_LRC_CH1_ACTIVE_SIZE_LRC_Y(fetch_ch1->height - 1),
			pxp->base + HW_PXP_DITHER_FETCH_ACTIVE_SIZE_LRC_CH1);
	__raw_writel(
			BF_PXP_DITHER_FETCH_SIZE_CH0_INPUT_TOTAL_WIDTH(fetch_ch0->width - 1) |
			BF_PXP_DITHER_FETCH_SIZE_CH0_INPUT_TOTAL_HEIGHT(fetch_ch0->height - 1),
			pxp->base + HW_PXP_DITHER_FETCH_SIZE_CH0);

	__raw_writel(
			BF_PXP_DITHER_FETCH_SIZE_CH1_INPUT_TOTAL_WIDTH(fetch_ch1->width - 1) |
			BF_PXP_DITHER_FETCH_SIZE_CH1_INPUT_TOTAL_HEIGHT(fetch_ch1->height - 1),
			pxp->base + HW_PXP_DITHER_FETCH_SIZE_CH1);

	__raw_writel(
			BF_PXP_DITHER_FETCH_PITCH_CH0_INPUT_PITCH(fetch_ch0->stride) |
			BF_PXP_DITHER_FETCH_PITCH_CH1_INPUT_PITCH(fetch_ch1->stride),
			pxp->base + HW_PXP_DITHER_FETCH_PITCH);

	__raw_writel(
			BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH0_INPUT_ACTIVE_BPP(0) |
			BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH0_EXPAND_FORMAT(0) |
			BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH0_EXPAND_EN(0) |
			BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH0_SHIFT_BYPASS(1),
			pxp->base + HW_PXP_DITHER_FETCH_SHIFT_CTRL_CH0);

	__raw_writel(
                        BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH1_INPUT_ACTIVE_BPP(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH1_EXPAND_FORMAT(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH1_EXPAND_EN(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_CTRL_CH1_SHIFT_BYPASS(1),
			pxp->base + HW_PXP_DITHER_FETCH_SHIFT_CTRL_CH1);

	__raw_writel(
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH0_OFFSET0(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH0_OFFSET1(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH0_OFFSET2(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH0_OFFSET3(0),
			pxp->base + HW_PXP_DITHER_FETCH_SHIFT_OFFSET_CH0);

	__raw_writel(
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH1_OFFSET0(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH1_OFFSET1(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH1_OFFSET2(0) |
                        BF_PXP_DITHER_FETCH_SHIFT_OFFSET_CH1_OFFSET3(0),
			pxp->base + HW_PXP_DITHER_FETCH_SHIFT_OFFSET_CH1);

	__raw_writel(
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH0_WIDTH0(7) |
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH0_WIDTH1(7) |
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH0_WIDTH2(7) |
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH0_WIDTH3(7),
			pxp->base + HW_PXP_DITHER_FETCH_SHIFT_WIDTH_CH0);

	__raw_writel(
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH1_WIDTH0(7) |
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH1_WIDTH1(7) |
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH1_WIDTH2(7) |
                        BF_PXP_DITHER_FETCH_SHIFT_WIDTH_CH1_WIDTH3(7),
			pxp->base + HW_PXP_DITHER_FETCH_SHIFT_WIDTH_CH1);

	__raw_writel(
			BF_PXP_DITHER_FETCH_ADDR_0_CH0_INPUT_BASE_ADDR0(fetch_ch0->paddr),
			pxp->base + HW_PXP_DITHER_FETCH_ADDR_0_CH0);

	__raw_writel(
			BF_PXP_DITHER_FETCH_ADDR_1_CH0_INPUT_BASE_ADDR1(0),
			pxp->base + HW_PXP_DITHER_FETCH_ADDR_1_CH0);

	__raw_writel(
			BF_PXP_DITHER_FETCH_ADDR_0_CH1_INPUT_BASE_ADDR0(fetch_ch1->paddr),
			pxp->base + HW_PXP_DITHER_FETCH_ADDR_0_CH1);

	__raw_writel(
			BF_PXP_DITHER_FETCH_ADDR_1_CH1_INPUT_BASE_ADDR1(0),
			pxp->base + HW_PXP_DITHER_FETCH_ADDR_1_CH1);
}

static void dither_store_config(struct pxps *pxp)
{
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_layer_param *store_ch0 = &config_data->dither_store_param[0];
	struct pxp_layer_param *store_ch1 = &config_data->dither_store_param[1];

	print_param(store_ch0, "dither store_ch0");
	print_param(store_ch1, "dither store_ch1");

	__raw_writel(
			BF_PXP_DITHER_STORE_CTRL_CH0_CH_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_BLOCK_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_BLOCK_16(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_HANDSHAKE_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_ARRAY_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_ARRAY_LINE_NUM(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_STORE_BYPASS_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_STORE_MEMORY_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_PACK_IN_SEL(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_FILL_DATA_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_WR_NUM_BYTES(32)|
			BF_PXP_DITHER_STORE_CTRL_CH0_COMBINE_2CHANNEL(0) |
			BF_PXP_DITHER_STORE_CTRL_CH0_ARBIT_EN(0),
			pxp->base + HW_PXP_DITHER_STORE_CTRL_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_CTRL_CH1_CH_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_BLOCK_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_BLOCK_16(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_HANDSHAKE_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_ARRAY_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_ARRAY_LINE_NUM(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_STORE_BYPASS_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_STORE_MEMORY_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH1_PACK_IN_SEL(0)|
			BF_PXP_DITHER_STORE_CTRL_CH1_WR_NUM_BYTES(32),
			pxp->base + HW_PXP_DITHER_STORE_CTRL_CH1);

	__raw_writel(
			BF_PXP_DITHER_STORE_SIZE_CH0_OUT_WIDTH(store_ch0->width - 1) |
			BF_PXP_DITHER_STORE_SIZE_CH0_OUT_HEIGHT(store_ch0->height - 1),
			pxp->base + HW_PXP_DITHER_STORE_SIZE_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_SIZE_CH1_OUT_WIDTH(store_ch1->width - 1) |
			BF_PXP_DITHER_STORE_SIZE_CH1_OUT_HEIGHT(store_ch1->height - 1),
			pxp->base + HW_PXP_DITHER_STORE_SIZE_CH1);

	__raw_writel(
			BF_PXP_DITHER_STORE_PITCH_CH0_OUT_PITCH(store_ch0->stride) |
			BF_PXP_DITHER_STORE_PITCH_CH1_OUT_PITCH(store_ch1->stride),
			pxp->base + HW_PXP_DITHER_STORE_PITCH);

	__raw_writel(
			BF_PXP_DITHER_STORE_SHIFT_CTRL_CH0_OUTPUT_ACTIVE_BPP(0)|
			BF_PXP_DITHER_STORE_SHIFT_CTRL_CH0_OUT_YUV422_1P_EN(0)|
			BF_PXP_DITHER_STORE_SHIFT_CTRL_CH0_OUT_YUV422_2P_EN(0)|
			BF_PXP_DITHER_STORE_SHIFT_CTRL_CH0_SHIFT_BYPASS(1),
			pxp->base + HW_PXP_DITHER_STORE_SHIFT_CTRL_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_SHIFT_CTRL_CH1_OUTPUT_ACTIVE_BPP(0)|
			BF_PXP_DITHER_STORE_SHIFT_CTRL_CH1_OUT_YUV422_1P_EN(0)|
			BF_PXP_DITHER_STORE_SHIFT_CTRL_CH1_OUT_YUV422_2P_EN(0),
			pxp->base + HW_PXP_DITHER_STORE_SHIFT_CTRL_CH1);

	__raw_writel(
			BF_PXP_DITHER_STORE_ADDR_0_CH0_OUT_BASE_ADDR0(store_ch0->paddr),
			pxp->base + HW_PXP_DITHER_STORE_ADDR_0_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_ADDR_1_CH0_OUT_BASE_ADDR1(0),
			pxp->base + HW_PXP_DITHER_STORE_ADDR_1_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_ADDR_0_CH1_OUT_BASE_ADDR0(store_ch1->paddr),
			pxp->base + HW_PXP_DITHER_STORE_ADDR_0_CH1);

	__raw_writel(
			BF_PXP_DITHER_STORE_ADDR_1_CH1_OUT_BASE_ADDR1(0),
			pxp->base + HW_PXP_DITHER_STORE_ADDR_1_CH1);

	__raw_writel(
			BF_PXP_DITHER_STORE_FILL_DATA_CH0_FILL_DATA_CH0(0),
			pxp->base + HW_PXP_DITHER_STORE_FILL_DATA_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_D_MASK0_H_CH0_D_MASK0_H_CH0(0xffffff),
			pxp->base + HW_PXP_DITHER_STORE_D_MASK0_H_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_D_MASK0_L_CH0_D_MASK0_L_CH0(0x0),
			pxp->base + HW_PXP_DITHER_STORE_D_MASK0_L_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_D_MASK1_H_CH0_D_MASK1_H_CH0(0x0),
			pxp->base + HW_PXP_DITHER_STORE_D_MASK1_H_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_D_MASK1_L_CH0_D_MASK1_L_CH0(0xff),
			pxp->base + HW_PXP_DITHER_STORE_D_MASK1_L_CH0);

	__raw_writel(
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH0(32) |
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG0(0) |
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH1(32)|
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG1(1) |
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH2(0)|
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG2(0)|
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_WIDTH3(0)|
			BF_PXP_DITHER_STORE_D_SHIFT_L_CH0_D_SHIFT_FLAG3(0),
			pxp->base + HW_PXP_DITHER_STORE_D_SHIFT_L_CH0);
}

static void pxp_set_final_lut_data(struct pxps *pxp)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;

	if(proc_data->quant_bit < 2) {
		pxp_sram_init(pxp, DITHER0_LUT, (u32)bit1_dither_data_8x8, 64);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA0(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA1(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA2(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA3(0x0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA0);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA4(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA5(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA6(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA7(0x0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA1);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA8(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA9(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA10(0xf0)|
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA11(0xf0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA2);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA12(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA13(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA14(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA15(0xf0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA3);
	} else if(proc_data->quant_bit < 4) {
		pxp_sram_init(pxp, DITHER0_LUT, (u32)bit2_dither_data_8x8, 64);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA0(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA1(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA2(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA3(0x0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA0);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA4(0x50) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA5(0x50) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA6(0x50) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA7(0x50),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA1);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA8(0xa0) |
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA9(0xa0) |
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA10(0xa0)|
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA11(0xa0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA2);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA12(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA13(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA14(0xf0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA15(0xf0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA3);
	} else {
		pxp_sram_init(pxp, DITHER0_LUT, (u32)bit4_dither_data_8x8, 64);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA0(0x0) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA1(0x10) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA2(0x20) |
				BF_PXP_DITHER_FINAL_LUT_DATA0_DATA3(0x30),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA0);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA4(0x40) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA5(0x50) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA6(0x60) |
				BF_PXP_DITHER_FINAL_LUT_DATA1_DATA7(0x70),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA1);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA8(0x80) |
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA9(0x90) |
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA10(0xa0)|
				BF_PXP_DITHER_FINAL_LUT_DATA2_DATA11(0xb0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA2);

		__raw_writel(
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA12(0xc0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA13(0xd0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA14(0xe0) |
				BF_PXP_DITHER_FINAL_LUT_DATA3_DATA15(0xf0),
				pxp->base + HW_PXP_DITHER_FINAL_LUT_DATA3);
	}
}

static void pxp_dithering_process(struct pxps *pxp)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	u32 val = 0;

	if (pxp->devdata && pxp->devdata->pxp_dithering_configure)
		pxp->devdata->pxp_dithering_configure(pxp);

	if (pxp_is_v3(pxp))
		val = BF_PXP_DITHER_CTRL_ENABLE0            (1) |
		      BF_PXP_DITHER_CTRL_ENABLE1            (0) |
		      BF_PXP_DITHER_CTRL_ENABLE2            (0) |
		      BF_PXP_DITHER_CTRL_DITHER_MODE2       (0) |
		      BF_PXP_DITHER_CTRL_DITHER_MODE1       (0) |
		      BF_PXP_DITHER_CTRL_DITHER_MODE0(proc_data->dither_mode) |
		      BF_PXP_DITHER_CTRL_LUT_MODE           (0) |
		      BF_PXP_DITHER_CTRL_IDX_MATRIX0_SIZE   (1) |
		      BF_PXP_DITHER_CTRL_IDX_MATRIX1_SIZE   (0) |
		      BF_PXP_DITHER_CTRL_IDX_MATRIX2_SIZE   (0) |
		      BF_PXP_DITHER_CTRL_BUSY2              (0) |
		      BF_PXP_DITHER_CTRL_BUSY1              (0) |
		      BF_PXP_DITHER_CTRL_BUSY0              (0);
	else if (pxp_is_v3p(pxp)) {
		if (proc_data->dither_mode != 0 &&
			proc_data->dither_mode != 3) {
			dev_err(pxp->dev, "Not supported dithering mode. "
					"Forced to be Orderred mode!\n");
			proc_data->dither_mode = 3;
		}

		val = BF_PXP_DITHER_CTRL_ENABLE0            (1) |
		      BF_PXP_DITHER_CTRL_ENABLE1            (1) |
		      BF_PXP_DITHER_CTRL_ENABLE2            (1) |
		      BF_PXP_DITHER_CTRL_DITHER_MODE2(proc_data->dither_mode) |
		      BF_PXP_DITHER_CTRL_DITHER_MODE1(proc_data->dither_mode) |
		      BF_PXP_DITHER_CTRL_DITHER_MODE0(proc_data->dither_mode) |
		      BF_PXP_DITHER_CTRL_LUT_MODE           (0) |
		      BF_PXP_DITHER_CTRL_IDX_MATRIX0_SIZE   (1) |
		      BF_PXP_DITHER_CTRL_IDX_MATRIX1_SIZE   (1) |
		      BF_PXP_DITHER_CTRL_IDX_MATRIX2_SIZE   (1) |
		      BF_PXP_DITHER_CTRL_FINAL_LUT_ENABLE   (0) |
		      BF_PXP_DITHER_CTRL_BUSY2              (0) |
		      BF_PXP_DITHER_CTRL_BUSY1              (0) |
		      BF_PXP_DITHER_CTRL_BUSY0              (0);
	}
	__raw_writel(val, pxp->base + HW_PXP_DITHER_CTRL);

	switch(proc_data->dither_mode) {
		case PXP_DITHER_PASS_THROUGH:
			/* no more settings required */
			break;
		case PXP_DITHER_FLOYD:
		case PXP_DITHER_ATKINSON:
		case PXP_DITHER_ORDERED:
			if(!proc_data->quant_bit || proc_data->quant_bit > 7) {
				dev_err(pxp->dev, "unsupported quantization bit number!\n");
				return;
			}
			__raw_writel(
					BF_PXP_DITHER_CTRL_FINAL_LUT_ENABLE(1) |
					BF_PXP_DITHER_CTRL_NUM_QUANT_BIT(proc_data->quant_bit),
					pxp->base + HW_PXP_DITHER_CTRL_SET);
			pxp_set_final_lut_data(pxp);

			break;
		case PXP_DITHER_QUANT_ONLY:
			if(!proc_data->quant_bit || proc_data->quant_bit > 7) {
				dev_err(pxp->dev, "unsupported quantization bit number!\n");
				return;
			}
			__raw_writel(
					BF_PXP_DITHER_CTRL_NUM_QUANT_BIT(proc_data->quant_bit),
					pxp->base + HW_PXP_DITHER_CTRL_SET);
			break;
		default:
			/* unknown mode */
			dev_err(pxp->dev, "unknown dithering mode passed!\n");
			__raw_writel(0x0, pxp->base + HW_PXP_DITHER_CTRL);
			return;
	}
}

static void pxp_dithering_configure(struct pxps *pxp)
{
	dither_prefetch_config(pxp);
	dither_store_config(pxp);
}

static void pxp_dithering_configure_v3p(struct pxps *pxp)
{
	struct pxp_config_data *config_data = &pxp->pxp_conf_state;
	struct pxp_layer_param *fetch_ch0 = &config_data->dither_fetch_param[0];
	struct pxp_layer_param *store_ch0 = &config_data->dither_store_param[0];

	__raw_writel(BF_PXP_CTRL_BLOCK_SIZE(BV_PXP_CTRL_BLOCK_SIZE__8X8) |
			BF_PXP_CTRL_ROTATE0(BV_PXP_CTRL_ROTATE0__ROT_0) |
			BM_PXP_CTRL_IRQ_ENABLE,
			pxp->base + HW_PXP_CTRL);

	__raw_writel(BF_PXP_PS_CTRL_DECX(BV_PXP_PS_CTRL_DECX__DISABLE) |
			BF_PXP_PS_CTRL_DECY(BV_PXP_PS_CTRL_DECY__DISABLE) |
			BF_PXP_PS_CTRL_FORMAT(BV_PXP_PS_CTRL_FORMAT__Y8),
			pxp->base + HW_PXP_PS_CTRL);

	__raw_writel(BF_PXP_OUT_CTRL_FORMAT(BV_PXP_OUT_CTRL_FORMAT__Y8),
			pxp->base + HW_PXP_OUT_CTRL);

	__raw_writel(BF_PXP_PS_SCALE_YSCALE(4096) |
			BF_PXP_PS_SCALE_XSCALE(4096),
			pxp->base + HW_PXP_PS_SCALE);

	__raw_writel(store_ch0->paddr, pxp->base + HW_PXP_OUT_BUF);

	__raw_writel(store_ch0->stride, pxp->base + HW_PXP_OUT_PITCH);

	__raw_writel(BF_PXP_OUT_LRC_X(store_ch0->width - 1) |
			BF_PXP_OUT_LRC_Y(store_ch0->height - 1),
			pxp->base + HW_PXP_OUT_LRC);

	__raw_writel(BF_PXP_OUT_AS_ULC_X(1) |
			BF_PXP_OUT_AS_ULC_Y(1),
			pxp->base + HW_PXP_OUT_AS_ULC);

	__raw_writel(BF_PXP_OUT_AS_LRC_X(0) |
			BF_PXP_OUT_AS_LRC_Y(0),
			pxp->base + HW_PXP_OUT_AS_LRC);

	__raw_writel(BF_PXP_OUT_PS_ULC_X(0) |
			BF_PXP_OUT_PS_ULC_Y(0),
			pxp->base + HW_PXP_OUT_PS_ULC);

	__raw_writel(BF_PXP_OUT_PS_LRC_X(fetch_ch0->width - 1) |
			BF_PXP_OUT_PS_LRC_Y(fetch_ch0->height - 1),
			pxp->base + HW_PXP_OUT_PS_LRC);

	__raw_writel(fetch_ch0->paddr, pxp->base + HW_PXP_PS_BUF);

	__raw_writel(fetch_ch0->stride, pxp->base + HW_PXP_PS_PITCH);

	__raw_writel(0x40000000, pxp->base + HW_PXP_CSC1_COEF0);

	__raw_writel(BF_PXP_DITHER_STORE_SIZE_CH0_OUT_WIDTH(store_ch0->width-1)|
		BF_PXP_DITHER_STORE_SIZE_CH0_OUT_HEIGHT(store_ch0->height-1),
		pxp->base + HW_PXP_DITHER_STORE_SIZE_CH0);

	__raw_writel(BF_PXP_DATA_PATH_CTRL0_MUX14_SEL(1),
			pxp->base + HW_PXP_DATA_PATH_CTRL0_CLR);
}

static void pxp_start2(struct pxps *pxp)
{
	struct pxp_config_data *pxp_conf = &pxp->pxp_conf_state;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	int dither_wfe_a_handshake = 0;
	int wfe_a_b_handshake = 0;
	int count = 0;

	int wfe_a_enable = ((proc_data->engine_enable & PXP_ENABLE_WFE_A) == PXP_ENABLE_WFE_A);
	int wfe_b_enable = ((proc_data->engine_enable & PXP_ENABLE_WFE_B) == PXP_ENABLE_WFE_B);
	int dither_enable = ((proc_data->engine_enable & PXP_ENABLE_DITHER) == PXP_ENABLE_DITHER);
	int handshake = ((proc_data->engine_enable & PXP_ENABLE_HANDSHAKE) == PXP_ENABLE_HANDSHAKE);
	int dither_bypass = ((proc_data->engine_enable & PXP_ENABLE_DITHER_BYPASS) == PXP_ENABLE_DITHER_BYPASS);
	u32 val = 0;

	if (dither_enable)
		count++;
	if (wfe_a_enable)
		count++;
	if (wfe_b_enable)
		count++;

	if (count == 0)
		return;
	if (handshake && (count == 1)) {
		dev_warn(pxp->dev, "Warning: Can not use handshake mode when "
				"only one sub-block is enabled!\n");
		handshake = 0;
	}

	if (handshake && wfe_b_enable && (wfe_a_enable == 0)) {
		dev_err(pxp->dev, "WFE_B only works when WFE_A is enabled!\n");
		return;
	}

	if (handshake && dither_enable && wfe_a_enable)
		dither_wfe_a_handshake = 1;
	if (handshake && wfe_a_enable && wfe_b_enable)
		wfe_a_b_handshake = 1;

	dev_dbg(pxp->dev, "handshake %d, dither_wfe_a_handshake %d, "
				"wfe_a_b_handshake %d, dither_bypass %d\n",
				handshake,
				dither_wfe_a_handshake,
				wfe_a_b_handshake,
				dither_bypass);

	if (handshake) {
		/* for handshake, we only enable the last completion INT */
		if (wfe_b_enable)
			__raw_writel(0x8000, pxp->base + HW_PXP_IRQ_MASK);
		else if (wfe_a_enable)
			__raw_writel(0x4000, pxp->base + HW_PXP_IRQ_MASK);

		/* Dither fetch */
		__raw_writel(
			BF_PXP_DITHER_FETCH_CTRL_CH0_CH_EN(1) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_BLOCK_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_BLOCK_16(0)|
			BF_PXP_DITHER_FETCH_CTRL_CH0_HANDSHAKE_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_BYPASS_PIXEL_EN(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_HIGH_BYTE(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_HFLIP(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_VFLIP(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_ROTATION_ANGLE(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_RD_NUM_BYTES(32) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_HANDSHAKE_SCAN_LINE_NUM(0) |
			BF_PXP_DITHER_FETCH_CTRL_CH0_ARBIT_EN(0),
			pxp->base + HW_PXP_DITHER_FETCH_CTRL_CH0);

		if (dither_bypass) {
			/* Dither store */
			__raw_writel(
			BF_PXP_DITHER_STORE_CTRL_CH0_CH_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_BLOCK_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_BLOCK_16(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_HANDSHAKE_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_ARRAY_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_ARRAY_LINE_NUM(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_STORE_BYPASS_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_STORE_MEMORY_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_PACK_IN_SEL(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_FILL_DATA_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_WR_NUM_BYTES(32)|
			BF_PXP_DITHER_STORE_CTRL_CH0_COMBINE_2CHANNEL(0) |
			BF_PXP_DITHER_STORE_CTRL_CH0_ARBIT_EN(0),
			pxp->base + HW_PXP_DITHER_STORE_CTRL_CH0);

			/* WFE_A fetch */
			__raw_writel(
			BF_PXP_WFA_FETCH_CTRL_BF1_EN(1) |
			BF_PXP_WFA_FETCH_CTRL_BF1_HSK_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_BYTES_PP(2) |
			BF_PXP_WFA_FETCH_CTRL_BF1_LINE_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_SRAM_IF(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_BURST_LEN(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_BYPASS_MODE(1) |
			BF_PXP_WFA_FETCH_CTRL_BF2_EN(1) |
			BF_PXP_WFA_FETCH_CTRL_BF2_HSK_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_BYTES_PP(1) |
			BF_PXP_WFA_FETCH_CTRL_BF2_LINE_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_SRAM_IF(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_BURST_LEN(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_BYPASS_MODE(0),
			pxp->base + HW_PXP_WFA_FETCH_CTRL);

		} else if (dither_wfe_a_handshake) {
			/* Dither store */
			__raw_writel(
			BF_PXP_DITHER_STORE_CTRL_CH0_CH_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_BLOCK_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_BLOCK_16(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_HANDSHAKE_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_ARRAY_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_ARRAY_LINE_NUM(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_STORE_BYPASS_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_STORE_MEMORY_EN(1)|
			BF_PXP_DITHER_STORE_CTRL_CH0_PACK_IN_SEL(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_FILL_DATA_EN(0)|
			BF_PXP_DITHER_STORE_CTRL_CH0_WR_NUM_BYTES(32)|
			BF_PXP_DITHER_STORE_CTRL_CH0_COMBINE_2CHANNEL(0) |
			BF_PXP_DITHER_STORE_CTRL_CH0_ARBIT_EN(0),
			pxp->base + HW_PXP_DITHER_STORE_CTRL_CH0);

			/* WFE_A fetch */
			__raw_writel(
			BF_PXP_WFA_FETCH_CTRL_BF1_EN(1) |
			BF_PXP_WFA_FETCH_CTRL_BF1_HSK_MODE(1) |
			BF_PXP_WFA_FETCH_CTRL_BF1_BYTES_PP(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_LINE_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_SRAM_IF(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_BURST_LEN(0) |
			BF_PXP_WFA_FETCH_CTRL_BF1_BYPASS_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_EN(1) |
			BF_PXP_WFA_FETCH_CTRL_BF2_HSK_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_BYTES_PP(1) |
			BF_PXP_WFA_FETCH_CTRL_BF2_LINE_MODE(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_SRAM_IF(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_BURST_LEN(0) |
			BF_PXP_WFA_FETCH_CTRL_BF2_BYPASS_MODE(0),
			pxp->base + HW_PXP_WFA_FETCH_CTRL);
		}

		if (wfe_a_b_handshake) {
			/* WFE_A Store */
			__raw_writel(
			BF_PXP_WFE_A_STORE_CTRL_CH1_CH_EN(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_BLOCK_EN(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_BLOCK_16(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_HANDSHAKE_EN(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_ARRAY_EN(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_ARRAY_LINE_NUM(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_STORE_BYPASS_EN(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_STORE_MEMORY_EN(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_PACK_IN_SEL(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_WR_NUM_BYTES(16),
			pxp->base + HW_PXP_WFE_A_STORE_CTRL_CH1);

			/* WFE_B fetch */
			__raw_writel(
			BF_PXP_WFB_FETCH_CTRL_BF1_EN(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_HSK_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BYTES_PP(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_LINE_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_SRAM_IF(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BURST_LEN(0) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BORDER_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF1_BYPASS_MODE(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_EN(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_HSK_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BYTES_PP(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_LINE_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_SRAM_IF(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BURST_LEN(0) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BORDER_MODE(1) |
			BF_PXP_WFB_FETCH_CTRL_BF2_BYPASS_MODE(0),
			pxp->base + HW_PXP_WFB_FETCH_CTRL);
		} else {
			/* WFE_A Store */
			__raw_writel(
			BF_PXP_WFE_A_STORE_CTRL_CH1_CH_EN(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_BLOCK_EN(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_BLOCK_16(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_HANDSHAKE_EN(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_ARRAY_EN(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_ARRAY_LINE_NUM(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_STORE_BYPASS_EN(0)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_STORE_MEMORY_EN(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_PACK_IN_SEL(1)|
			BF_PXP_WFE_A_STORE_CTRL_CH1_WR_NUM_BYTES(16),
			pxp->base + HW_PXP_WFE_A_STORE_CTRL_CH1);
		}

		if (pxp_is_v3(pxp))
			val = BF_PXP_CTRL_ENABLE_WFE_A(wfe_a_enable) |
				BF_PXP_CTRL_ENABLE_WFE_B(wfe_b_enable);
		else if (pxp_is_v3p(pxp))
			val = BF_PXP_CTRL_ENABLE_WFE_B(wfe_a_enable |
				wfe_b_enable);

		/* trigger operation */
		__raw_writel(
		BF_PXP_CTRL_ENABLE(1) |
		BF_PXP_CTRL_IRQ_ENABLE(0) |
		BF_PXP_CTRL_NEXT_IRQ_ENABLE(0) |
		BF_PXP_CTRL_LUT_DMA_IRQ_ENABLE(0) |
		BF_PXP_CTRL_ENABLE_LCD0_HANDSHAKE(1) |
		BF_PXP_CTRL_HANDSHAKE_ABORT_SKIP(1) |
		BF_PXP_CTRL_ROTATE0(0) |
		BF_PXP_CTRL_HFLIP0(0) |
		BF_PXP_CTRL_VFLIP0(0) |
		BF_PXP_CTRL_ROTATE1(0) |
		BF_PXP_CTRL_HFLIP1(0) |
		BF_PXP_CTRL_VFLIP1(0) |
		BF_PXP_CTRL_ENABLE_PS_AS_OUT(0) |
		BF_PXP_CTRL_ENABLE_DITHER(dither_enable) |
		BF_PXP_CTRL_ENABLE_INPUT_FETCH_STORE(0) |
		BF_PXP_CTRL_ENABLE_ALPHA_B(0) |
		BF_PXP_CTRL_BLOCK_SIZE(1) |
		BF_PXP_CTRL_ENABLE_CSC2(0) |
		BF_PXP_CTRL_ENABLE_LUT(1) |
		BF_PXP_CTRL_ENABLE_ROTATE0(0) |
		BF_PXP_CTRL_ENABLE_ROTATE1(0) |
		BF_PXP_CTRL_EN_REPEAT(0) |
		val,
		pxp->base + HW_PXP_CTRL);

		return;
	}

	if (pxp_is_v3(pxp))
		val = BF_PXP_CTRL_ENABLE_WFE_A(wfe_a_enable) |
		      BF_PXP_CTRL_ENABLE_WFE_B(wfe_b_enable) |
		      BF_PXP_CTRL_ENABLE_INPUT_FETCH_STORE(0) |
		      BF_PXP_CTRL_ENABLE_ALPHA_B(0);
	else if (pxp_is_v3p(pxp))
		val = BF_PXP_CTRL_ENABLE_WFE_B(wfe_a_enable |
			wfe_b_enable);

	__raw_writel(
			BF_PXP_CTRL_ENABLE(1) |
			BF_PXP_CTRL_IRQ_ENABLE(0) |
			BF_PXP_CTRL_NEXT_IRQ_ENABLE(0) |
			BF_PXP_CTRL_LUT_DMA_IRQ_ENABLE(0) |
			BF_PXP_CTRL_ENABLE_LCD0_HANDSHAKE(0) |
			BF_PXP_CTRL_ROTATE0(0) |
			BF_PXP_CTRL_HFLIP0(0) |
			BF_PXP_CTRL_VFLIP0(0) |
			BF_PXP_CTRL_ROTATE1(0) |
			BF_PXP_CTRL_HFLIP1(0) |
			BF_PXP_CTRL_VFLIP1(0) |
			BF_PXP_CTRL_ENABLE_PS_AS_OUT(0) |
			BF_PXP_CTRL_ENABLE_DITHER(dither_enable) |
			BF_PXP_CTRL_BLOCK_SIZE(0) |
			BF_PXP_CTRL_ENABLE_CSC2(0) |
			BF_PXP_CTRL_ENABLE_LUT(0) |
			BF_PXP_CTRL_ENABLE_ROTATE0(0) |
			BF_PXP_CTRL_ENABLE_ROTATE1(0) |
			BF_PXP_CTRL_EN_REPEAT(0) |
			val,
			pxp->base + HW_PXP_CTRL);

	if (pxp_is_v3(pxp))
		val = BF_PXP_CTRL2_ENABLE_WFE_A             (0) |
		      BF_PXP_CTRL2_ENABLE_WFE_B             (0) |
		      BF_PXP_CTRL2_ENABLE_INPUT_FETCH_STORE (0) |
		      BF_PXP_CTRL2_ENABLE_ALPHA_B           (0);
	else if (pxp_is_v3p(pxp))
		val = BF_PXP_CTRL2_ENABLE_WFE_B(0);

	__raw_writel(
			BF_PXP_CTRL2_ENABLE                   (0) |
			BF_PXP_CTRL2_ROTATE0                  (0) |
			BF_PXP_CTRL2_HFLIP0                   (0) |
			BF_PXP_CTRL2_VFLIP0                   (0) |
			BF_PXP_CTRL2_ROTATE1                  (0) |
			BF_PXP_CTRL2_HFLIP1                   (0) |
			BF_PXP_CTRL2_VFLIP1                   (0) |
			BF_PXP_CTRL2_ENABLE_DITHER            (0) |
			BF_PXP_CTRL2_BLOCK_SIZE               (0) |
			BF_PXP_CTRL2_ENABLE_CSC2              (0) |
			BF_PXP_CTRL2_ENABLE_LUT               (0) |
			BF_PXP_CTRL2_ENABLE_ROTATE0           (0) |
			BF_PXP_CTRL2_ENABLE_ROTATE1           (0),
			pxp->base + HW_PXP_CTRL2);

	dump_pxp_reg2(pxp);
}

static int pxp_dma_init(struct pxps *pxp)
{
	struct pxp_dma *pxp_dma = &pxp->pxp_dma;
	struct dma_device *dma = &pxp_dma->dma;
	int i;

	dma_cap_set(DMA_SLAVE, dma->cap_mask);
	dma_cap_set(DMA_PRIVATE, dma->cap_mask);

	/* Compulsory common fields */
	dma->dev = pxp->dev;
	dma->device_alloc_chan_resources = pxp_alloc_chan_resources;
	dma->device_free_chan_resources = pxp_free_chan_resources;
	dma->device_tx_status = pxp_tx_status;
	dma->device_issue_pending = pxp_issue_pending;

	/* Compulsory for DMA_SLAVE fields */
	dma->device_prep_slave_sg = pxp_prep_slave_sg;
	dma->device_terminate_all = pxp_device_terminate_all;

	/* Initialize PxP Channels */
	INIT_LIST_HEAD(&dma->channels);
	for (i = 0; i < NR_PXP_VIRT_CHANNEL; i++) {
		struct pxp_channel *pxp_chan = pxp->channel + i;
		struct dma_chan *dma_chan = &pxp_chan->dma_chan;

		spin_lock_init(&pxp_chan->lock);

		/* Only one EOF IRQ for PxP, shared by all channels */
		pxp_chan->eof_irq = pxp->irq;
		pxp_chan->status = PXP_CHANNEL_FREE;
		pxp_chan->completed = -ENXIO;
		snprintf(pxp_chan->eof_name, sizeof(pxp_chan->eof_name),
			 "PXP EOF %d", i);

		dma_chan->device = &pxp_dma->dma;
		dma_chan->cookie = 1;
		dma_chan->chan_id = i;
		list_add_tail(&dma_chan->device_node, &dma->channels);
	}

	return dma_async_device_register(&pxp_dma->dma);
}

static ssize_t clk_off_timeout_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", timeout_in_ms);
}

static ssize_t clk_off_timeout_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int val;
	if (sscanf(buf, "%d", &val) > 0) {
		timeout_in_ms = val;
		return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(clk_off_timeout, 0644, clk_off_timeout_show,
		   clk_off_timeout_store);

static ssize_t block_size_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	return sprintf(buf, "%d\n", block_size);
}

static ssize_t block_size_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	char **last = NULL;

	block_size = simple_strtoul(buf, last, 0);
	if (block_size > 1)
		block_size = 1;

	return count;
}
static DEVICE_ATTR(block_size, S_IWUSR | S_IRUGO,
		   block_size_show, block_size_store);

static struct platform_device_id imx_pxpdma_devtype[] = {
	{
		.name = "imx7d-pxp-dma",
		.driver_data = PXP_V3,
	}, {
		.name = "imx6ull-pxp-dma",
		.driver_data = PXP_V3P,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, imx_pxpdma_devtype);

static const struct of_device_id imx_pxpdma_dt_ids[] = {
	{ .compatible = "fsl,imx7d-pxp-dma", .data = &imx_pxpdma_devtype[0], },
	{ .compatible = "fsl,imx6ull-pxp-dma", .data = &imx_pxpdma_devtype[1], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_pxpdma_dt_ids);

static int has_pending_task(struct pxps *pxp, struct pxp_channel *task)
{
	int found;
	unsigned long flags;

	spin_lock_irqsave(&pxp->lock, flags);
	found = !list_empty(&head);
	spin_unlock_irqrestore(&pxp->lock, flags);

	return found;
}

static int pxp_dispatch_thread(void *argv)
{
	struct pxps *pxp = (struct pxps *)argv;
	struct pxp_channel *pending = NULL;
	unsigned long flags;

	set_freezable();

	while (!kthread_should_stop()) {
		int ret;
		ret = wait_event_freezable(pxp->thread_waitq,
					has_pending_task(pxp, pending) ||
					kthread_should_stop());
		if (ret < 0)
			continue;

		if (kthread_should_stop())
			break;

		spin_lock_irqsave(&pxp->lock, flags);
		pxp->pxp_ongoing = 1;
		spin_unlock_irqrestore(&pxp->lock, flags);
		init_completion(&pxp->complete);
		ret = pxpdma_dostart_work(pxp);
		if (ret) {
			pxp->pxp_ongoing = 0;
			continue;
		}
		ret = wait_for_completion_timeout(&pxp->complete, 2 * HZ);
		if (ret == 0) {
			printk(KERN_EMERG "%s: task is timeout\n\n", __func__);
			break;
		}
		if (pxp->devdata && pxp->devdata->pxp_lut_cleanup_multiple)
			pxp->devdata->pxp_lut_cleanup_multiple(pxp, 0, 0);
	}

	return 0;
}

static int pxp_init_interrupt(struct platform_device *pdev)
{
	int legacy_irq, std_irq, err;
	struct pxps *pxp = platform_get_drvdata(pdev);

	legacy_irq = platform_get_irq(pdev, 0);
	if (legacy_irq < 0) {
		dev_err(&pdev->dev, "failed to get pxp legacy irq: %d\n",
			legacy_irq);
		return legacy_irq;
	}

	std_irq = platform_get_irq(pdev, 1);
	if (std_irq < 0) {
		dev_err(&pdev->dev, "failed to get pxp standard irq: %d\n",
			std_irq);
		return std_irq;
	}

	err = devm_request_irq(&pdev->dev, legacy_irq, pxp_irq, 0,
				"pxp-dmaengine-legacy", pxp);
	if (err) {
		dev_err(&pdev->dev, "Request pxp legacy irq failed: %d\n", err);
		return err;
	}

	err = devm_request_irq(&pdev->dev, std_irq, pxp_irq, 0,
				"pxp-dmaengine-std", pxp);
	if (err) {
		dev_err(&pdev->dev, "Request pxp standard irq failed: %d\n", err);
		return err;
	}

	pxp->irq = legacy_irq;

	/* enable all the possible irq raised by PXP */
	__raw_writel(0xffff, pxp->base + HW_PXP_IRQ_MASK);

	return 0;
}

static int pxp_create_attrs(struct platform_device *pdev)
{
	int ret = 0;

	if ((ret = device_create_file(&pdev->dev, &dev_attr_clk_off_timeout))) {
		dev_err(&pdev->dev,
			"Unable to create file from clk_off_timeout\n");
		return ret;
	}

	if ((ret = device_create_file(&pdev->dev, &dev_attr_block_size))) {
		device_remove_file(&pdev->dev, &dev_attr_clk_off_timeout);

		dev_err(&pdev->dev,
			"Unable to create file from block_size\n");
		return ret;
	}

	return 0;
}

static void pxp_remove_attrs(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_clk_off_timeout);
	device_remove_file(&pdev->dev, &dev_attr_block_size);
}

static void pxp_init_timer(struct pxps *pxp)
{
	INIT_WORK(&pxp->work, clkoff_callback);

	init_timer(&pxp->clk_timer);
	pxp->clk_timer.function = pxp_clkoff_timer;
	pxp->clk_timer.data = (unsigned long)pxp;
}

static bool is_mux_node(uint32_t node_id)
{
	if ((node_id < PXP_2D_MUX_MUX0) ||
	    (node_id > PXP_2D_MUX_MUX15))
		return false;

	return true;
}

static bool search_mux_chain(uint32_t mux_id,
			     struct edge_node *enode)
{
	bool found = false;
	uint32_t i, j, next_mux = 0;
	uint32_t output;
	struct mux *muxes;

	muxes = (v3p_flag) ? muxes_v3p : muxes_v3;

	for (i = 0; i < 2; i++) {
		output = muxes[mux_id].mux_outputs[i];
		if (output == 0xff)
			break;

		if ((output == enode->adjvex)) {
			/* found */
			found = true;
			break;
		} else if (is_mux_node(output)) {
			next_mux = output - PXP_2D_MUX_BASE;
			found = search_mux_chain(next_mux, enode);

			if (found) {
				for (j = 0; j < 4; j++) {
					if (muxes[next_mux].mux_inputs[j] ==
					    (mux_id + PXP_2D_MUX_BASE))
						break;
				}

				set_bit(next_mux, (unsigned long *)&enode->mux_used);
				set_mux_val(&enode->muxes, next_mux, j);
				break;
			}
		}
	}

	return found;
}

static void enode_mux_config(unsigned int vnode_id,
			     struct edge_node *enode)
{
	uint32_t i, j;
	bool via_mux = false, need_search = false;
	struct mux *muxes;

	BUG_ON(vnode_id >= PXP_2D_NUM);
	BUG_ON(enode->adjvex >= PXP_2D_NUM);

	muxes = (v3p_flag) ? muxes_v3p : muxes_v3;

	for (i = 0; i < 16; i++) {
		for (j = 0; j < 4; j++) {
			if (muxes[i].mux_inputs[j] == 0xff)
				break;

			if (muxes[i].mux_inputs[j] == vnode_id)
				need_search = true;
			else if (muxes[i].mux_inputs[j] == PXP_2D_ALPHA0_S0_S1) {
				if ((vnode_id == PXP_2D_ALPHA0_S0) ||
				    (vnode_id == PXP_2D_ALPHA0_S1))
					need_search = true;
			} else if (muxes[i].mux_inputs[j] == PXP_2D_ALPHA1_S0_S1) {
				if ((vnode_id == PXP_2D_ALPHA1_S0) ||
				    (vnode_id == PXP_2D_ALPHA1_S1))
					need_search = true;
			}

			if (need_search) {
				via_mux = search_mux_chain(i, enode);
				need_search = false;
				break;
			}
		}

		if (via_mux) {
			set_bit(i, (unsigned long *)&enode->mux_used);
			set_mux_val(&enode->muxes, i, j);
			break;
		}
	}
}

static int pxp_create_initial_graph(struct platform_device *pdev)
{
	int i, j, first;
	static bool (*adj_array)[PXP_2D_NUM];
	struct edge_node *enode, *curr = NULL;

	adj_array = (v3p_flag) ? adj_array_v3p : adj_array_v3;

	for (i = 0; i < PXP_2D_NUM; i++) {
		switch (i) {
		case PXP_2D_PS:
		case PXP_2D_AS:
		case PXP_2D_INPUT_FETCH0:
		case PXP_2D_INPUT_FETCH1:
			adj_list[i].type = PXP_2D_TYPE_INPUT;
			break;
		case PXP_2D_OUT:
		case PXP_2D_INPUT_STORE0:
		case PXP_2D_INPUT_STORE1:
			adj_list[i].type = PXP_2D_TYPE_OUTPUT;
			break;
		default:
			adj_list[i].type = PXP_2D_TYPE_ALU;
			break;
		}

		first = -1;

		for (j = 0; j < PXP_2D_NUM; j++) {
			if (adj_array[i][j]) {
				enode = kmem_cache_alloc(edge_node_cache,
							 GFP_KERNEL | __GFP_ZERO);
				if (!enode) {
					dev_err(&pdev->dev, "allocate edge node failed\n");
					return -ENOMEM;
				}
				enode->adjvex = j;
				enode->prev_vnode = i;

				if (unlikely(first == -1)) {
					first = j;
					adj_list[i].first = enode;
				} else
					curr->next = enode;

				curr = enode;
				enode_mux_config(i, enode);
				dev_dbg(&pdev->dev, "(%d -> %d): mux_used 0x%x, mux_config 0x%x\n\n",
					 i, j, enode->mux_used, *(unsigned int*)&enode->muxes);
			}
		}
	}

	return 0;
}

/* Calculate the shortest paths start via
 * 'from' node to other nodes
 */
static void pxp_find_shortest_path(unsigned int from)
{
	int i;
	struct edge_node *enode;
	struct path_node *pnode, *adjnode;
	struct list_head queue;

	INIT_LIST_HEAD(&queue);
	list_add_tail(&path_table[from][from].node, &queue);

	while(!list_empty(&queue)) {
		pnode = list_entry(queue.next, struct path_node, node);
		enode = adj_list[pnode->id].first;
		while (enode) {
			adjnode = &path_table[from][enode->adjvex];

			if (adjnode->distance == DISTANCE_INFINITY) {
				adjnode->distance  = pnode->distance + 1;
				adjnode->prev_node = pnode->id;
				list_add_tail(&adjnode->node, &queue);
			}

			enode = enode->next;
		}
		list_del_init(&pnode->node);
	}

	for (i = 0; i < PXP_2D_NUM; i++)
		pr_debug("From %u: to %d (id = %d, distance = 0x%x, prev_node = %d\n",
			 from, i, path_table[from][i].id, path_table[from][i].distance,
			 path_table[from][i].prev_node);
}

static int pxp_gen_shortest_paths(struct platform_device *pdev)
{
	int i, j;

	for (i = 0; i < PXP_2D_NUM; i++) {
		for (j = 0; j < PXP_2D_NUM; j++) {
			path_table[i][j].id = j;
			path_table[i][j].distance = DISTANCE_INFINITY;
			path_table[i][j].prev_node = NO_PATH_NODE;
			INIT_LIST_HEAD(&path_table[i][j].node);
		}

		path_table[i][i].distance = 0;

		pxp_find_shortest_path(i);
	}

	return 0;
}

#ifdef	CONFIG_MXC_FPGA_M4_TEST
static void pxp_config_m4(struct platform_device *pdev)
{
	fpga_tcml_base = ioremap(FPGA_TCML_ADDR, SZ_32K);
	if (fpga_tcml_base == NULL) {
		dev_err(&pdev->dev,
			"get fpga_tcml_base error.\n");
		goto exit;
	}
	pinctrl_base = ioremap(PINCTRL, SZ_4K);
	if (pinctrl_base == NULL) {
		dev_err(&pdev->dev,
			"get fpga_tcml_base error.\n");
		goto exit;
	}

	__raw_writel(0xC0000000, pinctrl_base + 0x08);
	__raw_writel(0x3, pinctrl_base + PIN_DOUT);
	int i;
	for (i = 0; i < 1024 * 32 / 4; i++) {
		*(((unsigned int *)(fpga_tcml_base)) + i) = cm4_image[i];
	}
}
#endif

static int pxp_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(imx_pxpdma_dt_ids, &pdev->dev);
	struct pxps *pxp;
	struct resource *res;
	int err = 0;

	if (of_id)
		pdev->id_entry = of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pxp = devm_kzalloc(&pdev->dev, sizeof(*pxp), GFP_KERNEL);
	if (!pxp) {
		dev_err(&pdev->dev, "failed to allocate control object\n");
		err = -ENOMEM;
		goto exit;
	}

	pxp->dev = &pdev->dev;

	platform_set_drvdata(pdev, pxp);

	spin_lock_init(&pxp->lock);
	mutex_init(&pxp->clk_mutex);

	pxp->base = devm_ioremap_resource(&pdev->dev, res);
	if (pxp->base == NULL) {
		dev_err(&pdev->dev, "Couldn't ioremap regs\n");
		err = -ENODEV;
		goto exit;
	}
	pxp_reg_base = pxp->base;

	pxp->pdev = pdev;
	pxp->devdata = &pxp_devdata[pdev->id_entry->driver_data];

	v3p_flag = (pxp_is_v3p(pxp)) ? true : false;

	pxp->ipg_clk = devm_clk_get(&pdev->dev, "pxp_ipg");
	pxp->axi_clk = devm_clk_get(&pdev->dev, "pxp_axi");

	if (IS_ERR(pxp->ipg_clk) || IS_ERR(pxp->axi_clk)) {
		dev_err(&pdev->dev, "pxp clocks invalid\n");
		err = -EINVAL;
		goto exit;
	}

	pxp_soft_reset(pxp);
	pxp_writel(0x0, HW_PXP_CTRL);
	/* Initialize DMA engine */
	err = pxp_dma_init(pxp);
	if (err < 0)
		goto exit;

	pxp_clk_enable(pxp);
	pxp_soft_reset(pxp);

	/* Initialize PXP Interrupt */
	err = pxp_init_interrupt(pdev);
	if (err < 0)
		goto exit;

	if (pxp->devdata && pxp->devdata->pxp_data_path_config)
		pxp->devdata->pxp_data_path_config(pxp);

	dump_pxp_reg(pxp);
	pxp_clk_disable(pxp);

	pxp_init_timer(pxp);

	init_waitqueue_head(&pxp->thread_waitq);
	/* allocate a kernel thread to dispatch pxp conf */
	pxp->dispatch = kthread_run(pxp_dispatch_thread, pxp, "pxp_dispatch");
	if (IS_ERR(pxp->dispatch)) {
		err = PTR_ERR(pxp->dispatch);
		goto exit;
	}
	tx_desc_cache = kmem_cache_create("tx_desc", sizeof(struct pxp_tx_desc),
					  0, SLAB_HWCACHE_ALIGN, NULL);
	if (!tx_desc_cache) {
		err = -ENOMEM;
		goto exit;
	}

	edge_node_cache = kmem_cache_create("edge_node", sizeof(struct edge_node),
					    0, SLAB_HWCACHE_ALIGN, NULL);
	if (!edge_node_cache) {
		err = -ENOMEM;
		kmem_cache_destroy(tx_desc_cache);
		goto exit;
	}

	err = pxp_create_attrs(pdev);
	if (err) {
		kmem_cache_destroy(tx_desc_cache);
		kmem_cache_destroy(edge_node_cache);
		goto exit;
	}

	if ((err = pxp_create_initial_graph(pdev))) {
		kmem_cache_destroy(tx_desc_cache);
		kmem_cache_destroy(edge_node_cache);
		goto exit;
	}

	pxp_gen_shortest_paths(pdev);

#ifdef	CONFIG_MXC_FPGA_M4_TEST
	pxp_config_m4(pdev);
#endif
	register_pxp_device();
	pm_runtime_enable(pxp->dev);

	dma_alloc_coherent(NULL, PAGE_ALIGN(1920 * 1088 * 4),
			   &paddr, GFP_KERNEL);

exit:
	if (err)
		dev_err(&pdev->dev, "Exiting (unsuccessfully) pxp_probe()\n");
	return err;
}

static int pxp_remove(struct platform_device *pdev)
{
	struct pxps *pxp = platform_get_drvdata(pdev);

	unregister_pxp_device();
	kmem_cache_destroy(tx_desc_cache);
	kmem_cache_destroy(edge_node_cache);
	kthread_stop(pxp->dispatch);
	cancel_work_sync(&pxp->work);
	del_timer_sync(&pxp->clk_timer);
	clk_disable_unprepare(pxp->ipg_clk);
	clk_disable_unprepare(pxp->axi_clk);
	pxp_remove_attrs(pdev);
	dma_async_device_unregister(&(pxp->pxp_dma.dma));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pxp_suspend(struct device *dev)
{
	struct pxps *pxp = dev_get_drvdata(dev);

	pxp_clk_enable(pxp);
	while (__raw_readl(pxp->base + HW_PXP_CTRL) & BM_PXP_CTRL_ENABLE)
		;

	__raw_writel(BM_PXP_CTRL_SFTRST, pxp->base + HW_PXP_CTRL);
	pxp_clk_disable(pxp);

	return 0;
}

static int pxp_resume(struct device *dev)
{
	struct pxps *pxp = dev_get_drvdata(dev);

	pxp_clk_enable(pxp);
	/* Pull PxP out of reset */
	pxp_soft_reset(pxp);
	if (pxp->devdata && pxp->devdata->pxp_data_path_config)
		pxp->devdata->pxp_data_path_config(pxp);
	/* enable all the possible irq raised by PXP */
	__raw_writel(0xffff, pxp->base + HW_PXP_IRQ_MASK);
	pxp_clk_disable(pxp);

	return 0;
}
#else
#define	pxp_suspend	NULL
#define	pxp_resume	NULL
#endif

#ifdef CONFIG_PM
static int pxp_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pxp busfreq high release.\n");

	return 0;
}

static int pxp_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pxp busfreq high request.\n");

	return 0;
}
#else
#define	pxp_runtime_suspend	NULL
#define	pxp_runtime_resume	NULL
#endif

static const struct dev_pm_ops pxp_pm_ops = {
	SET_RUNTIME_PM_OPS(pxp_runtime_suspend, pxp_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pxp_suspend, pxp_resume)
};

static struct platform_driver pxp_driver = {
	.driver = {
			.name = "imx-pxp-v3",
			.of_match_table = of_match_ptr(imx_pxpdma_dt_ids),
			.pm = &pxp_pm_ops,
		   },
	.probe = pxp_probe,
	.remove = pxp_remove,
};

static int __init pxp_init(void)
{
        return platform_driver_register(&pxp_driver);
}
late_initcall(pxp_init);

static void __exit pxp_exit(void)
{
        platform_driver_unregister(&pxp_driver);
}
module_exit(pxp_exit);


MODULE_DESCRIPTION("i.MX PxP driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
