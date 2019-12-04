/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 NXP Semiconductor
 */

#ifndef __MXC_ISI_CORE_H__
#define __MXC_ISI_CORE_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sys_soc.h>

#include "imx8-common.h"

#define MXC_ISI_DRIVER_NAME	"mxc-isi"
#define MXC_ISI_CAPTURE		"mxc-isi-cap"
#define MXC_ISI_M2M		"mxc-isi-m2m"
#define MXC_MAX_PLANES		3

struct mxc_isi_dev;

enum mxc_isi_out_fmt {
	MXC_ISI_OUT_FMT_RGBA32	= 0x0,
	MXC_ISI_OUT_FMT_ABGR32,
	MXC_ISI_OUT_FMT_ARGB32,
	MXC_ISI_OUT_FMT_RGBX32,
	MXC_ISI_OUT_FMT_XBGR32,
	MXC_ISI_OUT_FMT_XRGB32,
	MXC_ISI_OUT_FMT_RGB32P,
	MXC_ISI_OUT_FMT_BGR32P,
	MXC_ISI_OUT_FMT_A2BGR10,
	MXC_ISI_OUT_FMT_A2RGB10,
	MXC_ISI_OUT_FMT_RGB565,
	MXC_ISI_OUT_FMT_RAW8,
	MXC_ISI_OUT_FMT_RAW10,
	MXC_ISI_OUT_FMT_RAW10P,
	MXC_ISI_OUT_FMT_RAW12,
	MXC_ISI_OUT_FMT_RAW16,
	MXC_ISI_OUT_FMT_YUV444_1P8P,
	MXC_ISI_OUT_FMT_YUV444_2P8P,
	MXC_ISI_OUT_FMT_YUV444_3P8P,
	MXC_ISI_OUT_FMT_YUV444_1P8,
	MXC_ISI_OUT_FMT_YUV444_1P10,
	MXC_ISI_OUT_FMT_YUV444_2P10,
	MXC_ISI_OUT_FMT_YUV444_3P10,
	MXC_ISI_OUT_FMT_YUV444_1P10P = 0x18,
	MXC_ISI_OUT_FMT_YUV444_2P10P,
	MXC_ISI_OUT_FMT_YUV444_3P10P,
	MXC_ISI_OUT_FMT_YUV444_1P12 = 0x1C,
	MXC_ISI_OUT_FMT_YUV444_2P12,
	MXC_ISI_OUT_FMT_YUV444_3P12,
	MXC_ISI_OUT_FMT_YUV422_1P8P = 0x20,
	MXC_ISI_OUT_FMT_YUV422_2P8P,
	MXC_ISI_OUT_FMT_YUV422_3P8P,
	MXC_ISI_OUT_FMT_YUV422_1P10 = 0x24,
	MXC_ISI_OUT_FMT_YUV422_2P10,
	MXC_ISI_OUT_FMT_YUV422_3P10,
	MXC_ISI_OUT_FMT_YUV422_1P10P = 0x28,
	MXC_ISI_OUT_FMT_YUV422_2P10P,
	MXC_ISI_OUT_FMT_YUV422_3P10P,
	MXC_ISI_OUT_FMT_YUV422_1P12 = 0x2C,
	MXC_ISI_OUT_FMT_YUV422_2P12,
	MXC_ISI_OUT_FMT_YUV422_3P12,
	MXC_ISI_OUT_FMT_YUV420_2P8P = 0x31,
	MXC_ISI_OUT_FMT_YUV420_3P8P,
	MXC_ISI_OUT_FMT_YUV420_2P10 = 0x35,
	MXC_ISI_OUT_FMT_YUV420_3P10,
	MXC_ISI_OUT_FMT_YUV420_2P10P = 0x39,
	MXC_ISI_OUT_FMT_YUV420_3P10P,
	MXC_ISI_OUT_FMT_YUV420_2P12 = 0x3D,
	MXC_ISI_OUT_FMT_YUV420_3P12,
};

enum mxc_isi_in_fmt {
	MXC_ISI_IN_FMT_BGR8P	= 0x0,
};

enum mxc_isi_m2m_in_fmt {
	MXC_ISI_M2M_IN_FMT_BGR8P	= 0x0,
	MXC_ISI_M2M_IN_FMT_RGB8P,
	MXC_ISI_M2M_IN_FMT_XRGB8,
	MXC_ISI_M2M_IN_FMT_RGBX8,
	MXC_ISI_M2M_IN_FMT_XBGR8,
	MXC_ISI_M2M_IN_FMT_RGB565,
	MXC_ISI_M2M_IN_FMT_A2BGR10,
	MXC_ISI_M2M_IN_FMT_A2RGB10,
	MXC_ISI_M2M_IN_FMT_YUV444_1P8P,
	MXC_ISI_M2M_IN_FMT_YUV444_1P10,
	MXC_ISI_M2M_IN_FMT_YUV444_1P10P,
	MXC_ISI_M2M_IN_FMT_YUV444_1P12,
	MXC_ISI_M2M_IN_FMT_YUV444_1P8,
	MXC_ISI_M2M_IN_FMT_YUV422_1P8P,
	MXC_ISI_M2M_IN_FMT_YUV422_1P10,
	MXC_ISI_M2M_IN_FMT_YUV422_1P10P,
};

struct mxc_isi_fmt {
	char	*name;
	u32	mbus_code;
	u32	fourcc;
	u32	color;
	u16	memplanes;
	u16	colplanes;
	u8	colorspace;
	u8	depth[MXC_MAX_PLANES];
	u16	mdataplanes;
	u16	flags;
};

struct mxc_isi_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *alpha;
	struct v4l2_ctrl *num_cap_buf;
	struct v4l2_ctrl *num_out_buf;
	bool ready;
};

/**
 * struct addr -  physical address set for DMA
 * @y:	 luminance plane physical address
 * @cb:	 Cb plane physical address
 * @cr:	 Cr plane physical address
 */
struct frame_addr {
	u32	y;
	u32	cb;
	u32	cr;
};

/**
 * struct mxc_isi_frame - source/target frame properties
 * o_width:	 original image width from sensor
 * o_height: original image height from sensor
 * c_width:	 crop image width set by g_selection
 * c_height: crop image height set by g_selection
 * h_off:	crop horizontal pixel offset
 * v_off:	crop vertical pixel offset
 * width:	out image pixel width
 * height:	out image pixel weight
 * bytesperline: bytesperline value for each plane
 * paddr:	image frame buffer physical addresses
 * fmt:	color format pointer
 */
struct mxc_isi_frame {
	u32	o_width;
	u32	o_height;
	u32	c_width;
	u32	c_height;
	u32	h_off;
	u32	v_off;
	u32	width;
	u32	height;
	unsigned int	sizeimage[MXC_MAX_PLANES];
	unsigned int	bytesperline[MXC_MAX_PLANES];
	struct mxc_isi_fmt	*fmt;
};

struct mxc_isi_roi_alpha {
	u8 alpha;
	struct v4l2_rect rect;
};

struct mxc_isi_buffer {
	struct vb2_v4l2_buffer  v4l2_buf;
	struct list_head	list;
	struct frame_addr	paddr;
	enum mxc_isi_buf_id	id;
	bool discard;
};

struct mxc_isi_m2m_dev {
	struct platform_device	*pdev;

	struct video_device vdev;
	struct v4l2_device  v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct v4l2_fh      fh;
	struct v4l2_pix_format_mplane pix;

	struct list_head	out_active;
	struct mxc_isi_ctrls	ctrls;

	struct mxc_isi_frame src_f;
	struct mxc_isi_frame dst_f;

	struct mutex lock;
	spinlock_t   slock;

	unsigned int aborting;
	unsigned int frame_count;

	u32 req_cap_buf_num;
	u32 req_out_buf_num;

	u8 id;
};

struct mxc_isi_ctx {
	struct mxc_isi_m2m_dev *isi_m2m;
	struct v4l2_fh	    fh;
};

struct mxc_isi_dev_ops {
	int (*clk_get)(struct mxc_isi_dev *mxc_isi);
	int (*clk_enable)(struct mxc_isi_dev *mxc_isi);
	void (*clk_disable)(struct mxc_isi_dev *mxc_isi);
};

struct mxc_isi_cap_dev {
	struct v4l2_subdev  sd;
	struct video_device vdev;
	struct v4l2_fh      fh;
	struct vb2_queue    vb2_q;
	struct v4l2_pix_format_mplane pix;

	struct mxc_isi_dev     *mxc_isi;
	struct platform_device *pdev;
	struct mxc_isi_ctrls   ctrls;
	struct mxc_isi_buffer  buf_discard[2];

	struct media_pad cap_pad;
	struct media_pad sd_pads[MXC_ISI_SD_PADS_NUM];

	struct list_head out_pending;
	struct list_head out_active;
	struct list_head out_discard;

	struct mxc_isi_frame src_f;
	struct mxc_isi_frame dst_f;

	u32 frame_count;
	u32 id;

	struct mutex lock;
	spinlock_t   slock;

	/* dirty buffer */
	size_t     discard_size[MXC_MAX_PLANES];
	void       *discard_buffer[MXC_MAX_PLANES];
	dma_addr_t discard_buffer_dma[MXC_MAX_PLANES];
};

struct mxc_isi_dev {
	/* Pointer to isi capture child device driver data */
	struct mxc_isi_cap_dev *isi_cap;

	/* Pointer to isi m2m child device driver data */
	struct mxc_isi_m2m_dev *isi_m2m;

	struct platform_device *pdev;

	/* clk for imx8qxp/qm platform */
	struct clk *clk;

	/* clks for imx8mn platform */
	struct clk *clk_disp_axi;
	struct clk *clk_disp_apb;
	struct clk *clk_root_disp_axi;
	struct clk *clk_root_disp_apb;

	const struct mxc_isi_dev_ops *ops;

	struct reset_control *soft_resetn;
	struct reset_control *clk_enable;

	struct mutex lock;
	spinlock_t   slock;

	void __iomem *regs;

	u8 chain_buf;
	u8 alpha;
	bool m2m_enabled;
	bool buf_active_reverse;

	/* manage share ISI channel resource */
	atomic_t usage_count;

	/* scale factor */
	u32 xfactor;
	u32 yfactor;
	u32 pre_dec_x;
	u32 pre_dec_y;

	u32 status;

	u32 interface[MAX_PORTS];
	int id;

	unsigned int hflip:1;
	unsigned int vflip:1;
	unsigned int cscen:1;
	unsigned int scale:1;
	unsigned int alphaen:1;
	unsigned int crop:1;
	unsigned int deinterlace:1;
	unsigned int is_streaming:1;
};

static inline void set_frame_bounds(struct mxc_isi_frame *f,
				    u32 width, u32 height)
{
	f->o_width  = width;
	f->o_height = height;
	f->c_width  = width;
	f->c_height = height;
	f->width  = width;
	f->height = height;
}

static inline void set_frame_out(struct mxc_isi_frame *f,
				 u32 width, u32 height)
{
	f->c_width  = width;
	f->c_height = height;
	f->width  = width;
	f->height = height;
}

static inline void set_frame_crop(struct mxc_isi_frame *f,
				  u32 left, u32 top, u32 width, u32 height)
{
	f->h_off = left;
	f->v_off = top;
	f->c_width  = width;
	f->c_height = height;
}

#if defined(CONFIG_IMX8_ISI_CORE)
struct mxc_isi_dev *mxc_isi_get_hostdata(struct platform_device *pdev);
struct device *mxc_isi_dev_get_parent(struct platform_device *pdev);
#else
static inline struct mxc_isi_dev *mxc_isi_get_hostdata(struct platform_device *pdev) {}
static inline struct struct device *mxc_isi_dev_get_parent(struct platform_device *pdev) {}
#endif

#endif /* __MXC_ISI_CORE_H__ */
