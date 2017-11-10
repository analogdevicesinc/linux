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

#ifndef MXC_ISI_CORE_H_
#define MXC_ISI_CORE_H_

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#define MXC_ISI_DRIVER_NAME	"mxc-isi"
#define MXC_ISI_MAX_DEVS	8

#define ISI_OF_NODE_NAME	"isi"

#define MXC_ISI_SD_PAD_SINK_MIPI0_VC0	0
#define MXC_ISI_SD_PAD_SINK_MIPI0_VC1	1
#define MXC_ISI_SD_PAD_SINK_MIPI0_VC2	2
#define MXC_ISI_SD_PAD_SINK_MIPI0_VC3	3
#define MXC_ISI_SD_PAD_SINK_MIPI1_VC0	4
#define MXC_ISI_SD_PAD_SINK_MIPI1_VC1	5
#define MXC_ISI_SD_PAD_SINK_MIPI1_VC2	6
#define MXC_ISI_SD_PAD_SINK_MIPI1_VC3	7
#if 0
#define MXC_ISI_SD_PAD_SINK_MIPI_CSI0	0
#define MXC_ISI_SD_PAD_SINK_MIPI_CSI1	1
#endif
#define MXC_ISI_SD_PAD_SINK_DC0			8
#define MXC_ISI_SD_PAD_SINK_DC1			9
#define MXC_ISI_SD_PAD_SINK_HDMI		10
#define MXC_ISI_SD_PAD_SINK_MEM			11
#define MXC_ISI_SD_PAD_SOURCE_MEM		12
#define MXC_ISI_SD_PAD_SOURCE_DC0		13
#define MXC_ISI_SD_PAD_SOURCE_DC1		14
#define MXC_ISI_SD_PADS_NUM				15

#define MXC_MAX_PLANES		3

enum isi_input_interface {
	ISI_INPUT_INTERFACE_DC0 = 0,
	ISI_INPUT_INTERFACE_DC1,
	ISI_INPUT_INTERFACE_MIPI0_CSI2,
	ISI_INPUT_INTERFACE_MIPI1_CSI2,
	ISI_INPUT_INTERFACE_HDMI,
	ISI_INPUT_INTERFACE_MEM,
	ISI_INPUT_INTERFACE_MAX,
};

enum isi_input_sub_interface {
	ISI_INPUT_SUB_INTERFACE_VC0 = 0,
	ISI_INPUT_SUB_INTERFACE_VC1,
	ISI_INPUT_SUB_INTERFACE_VC2,
	ISI_INPUT_SUB_INTERFACE_VC3,
};

enum isi_output_interface {
	ISI_OUTPUT_INTERFACE_DC0 = 0,
	ISI_OUTPUT_INTERFACE_DC1,
	ISI_OUTPUT_INTERFACE_MEM,
	ISI_OUTPUT_INTERFACE_MAX,
};

enum {
	IN_PORT,
	SUB_IN_PORT,
	OUT_PORT,
	MAX_PORTS,
};

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

enum mxc_isi_power_state {
	MXC_ISI_PM_SUSPENDED = 0x01,
	MXC_ISI_PM_POWERED = 0x02,
};

struct mxc_isi_fmt {
	char	*name;
	u32 mbus_code;
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
	u32 c_width;
	u32 c_height;
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
	struct vb2_v4l2_buffer v4l2_buf;
	struct list_head	list;
	struct frame_addr	paddr;
};

struct mxc_isi_m2m_dev {
	struct video_device		vdev;
	struct v4l2_m2m_dev		*m2m_dev;
	struct v4l2_fh			fh;

	struct mxc_isi_frame	src_f;
	struct mxc_isi_frame	dst_f;
};

struct mxc_isi_cap_dev {
	struct v4l2_subdev		sd;
	struct video_device		vdev;
	struct v4l2_fh			fh;
	struct media_pad		cap_pad;
	struct media_pad		sd_pads[MXC_ISI_SD_PADS_NUM];
	struct vb2_queue		vb2_q;
	struct list_head		out_pending;
	struct list_head		out_active;

	struct mxc_isi_frame	src_f;
	struct mxc_isi_frame	dst_f;
	u32						frame_count;

	u32 buf_index;

};

struct mxc_isi_dev {
	spinlock_t				slock;
	struct mutex			lock;
	wait_queue_head_t		irq_queue;

	int						id;
	void __iomem			*regs;
	unsigned long			state;

	struct platform_device		*pdev;
	struct v4l2_device			*v4l2_dev;
	struct mxc_isi_m2m_dev		m2m;
	struct mxc_isi_cap_dev		isi_cap;
	struct clk		*clk;

	u32 interface[MAX_PORTS];
	u32 flags;

	/* scale factor */
	u32	xfactor;
	u32	yfactor;
	u32	pre_dec_x;
	u32	pre_dec_y;

	unsigned int		hflip:1;
	unsigned int		vflip:1;

	unsigned int		cscen:1;
	unsigned int		scale:1;
	unsigned int		alphaen:1;
	unsigned int		crop:1;
	unsigned int		deinterlace:1;

	struct mxc_isi_ctrls ctrls;
	u8			alpha;		/* goable alpha */
	struct mxc_isi_roi_alpha alpha_roi[5];		/* ROI alpha */
};

static inline void set_frame_bounds(struct mxc_isi_frame *f, u32 width, u32 height)
{
	f->o_width  = width;
	f->o_height = height;
	f->c_width  = width;
	f->c_height = height;
	f->width  = width;
	f->height = height;
}

static inline void set_frame_out(struct mxc_isi_frame *f, u32 width, u32 height)
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

#endif
