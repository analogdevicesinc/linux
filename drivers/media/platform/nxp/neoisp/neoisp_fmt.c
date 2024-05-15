// SPDX-License-Identifier: GPL-2.0+
/*
 * NEOISP supported formats definition
 *
 * Copyright 2023-2024 NXP
 * Author: Aymen Sghaier (aymen.sghaier@nxp.com)
 *
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/nxp_neoisp.h>

#include "neoisp.h"
#include "neoisp_regs.h"

const struct v4l2_frmsize_stepwise neoisp_frmsize_stepwise = {
	.min_width = NEOISP_MIN_W,
	.min_height = NEOISP_MIN_H,
	.max_width = NEOISP_MAX_W,
	.max_height = NEOISP_MAX_H,
	.step_width = 1UL << NEOISP_ALIGN_W,
	.step_height = 1UL << NEOISP_ALIGN_H,
};

const struct neoisp_fmt_s formats_vcap[NEOISP_FMT_VCAP_COUNT] = {
	{
		.fourcc = V4L2_PIX_FMT_BGR24,     /* 24-bit BGR 8-8-8 */
		.align = 32,
		.bit_depth = 24,
		.num_planes = 1,
		.is_rgb = 1,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_RGB24,     /* 24-bit RGB 8-8-8 */
		.align = 32,
		.bit_depth = 24,
		.num_planes = 1,
		.is_rgb = 1,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_XBGR32,    /* 32-bit BGRX 8-8-8-8 */
		.align = 32,
		.bit_depth = 32,
		.num_planes = 1,
		.is_rgb = 1,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_RGBX32,    /* 32-bit RGBX 8-8-8-8 */
		.align = 32,
		.bit_depth = 32,
		.num_planes = 1,
		.is_rgb = 1,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_GREY,      /* 8-bit Greyscale */
		.align = 32,
		.bit_depth = 8,
		.num_planes = 1,
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,      /* 12-bit Y/CbCr 4:2:0 */
		.align = 32,
		.bit_depth = 8,
		.num_planes = 2,
		.pl_divisors = {1, 2},
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_NV21,      /* 12-bit Y/CrCb 4:2:0 */
		.align = 32,
		.bit_depth = 8,
		.num_planes = 2,
		.pl_divisors = {1, 2},
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_NV16,      /* 16-bit Y/CbCr 4:2:2 */
		.align = 32,
		.bit_depth = 8,
		.num_planes = 2,
		.pl_divisors = {1, 1},
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,      /* 16-bit Y/CrCb 4:2:2 */
		.align = 32,
		.bit_depth = 8,
		.num_planes = 2,
		.pl_divisors = {1, 1},
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,     /* 16-bit YUV 4:2:2 */
		.align = 32,
		.bit_depth = 16,
		.num_planes = 1,
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_YUV24,    /* 24-bit YUV 4:4:4 8-8-8 */
		.align = 32,
		.bit_depth = 24,
		.num_planes = 1,
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_YUVX32,     /* 32-bit YUVX 4:4:4 */
		.align = 32,
		.bit_depth = 32,
		.num_planes = 1,
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_VUYX32,     /* 32-bit VUYX 4:4:4 */
		.align = 32,
		.bit_depth = 32,
		.num_planes = 1,
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_YUYV,     /* 16-bit YUYV 4:2:2 */
		.align = 32,
		.bit_depth = 16,
		.num_planes = 1,
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}, {
		.fourcc = V4L2_PIX_FMT_VYUY,     /* 16-bit VYUY 4:2:2 */
		.align = 32,
		.bit_depth = 16,
		.num_planes = 1,
		.is_rgb = 0,
		.type = NEOISP_FMT_VIDEO_CAPTURE
	}
};

const struct neoisp_fmt_s formats_vout[NEOISP_FMT_VOUT_COUNT] = {
	{
		.fourcc = V4L2_PIX_FMT_SRGGB8,    /* 8-bit Bayer RGRG/GBGB */
		.align = 32,
		.bit_depth = 8,
		.ibpp = 6,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB10,   /* 10-bit Bayer RGRG/GBGB */
		.align = 32,
		.bit_depth = 10,
		.ibpp = 4,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB10P,  /* 10-bit Bayer RGRG/GBGB packed */
		.align = 32,
		.bit_depth = 10,
		.ibpp = 5, /* FIXME this may be wrong for 10bits packed format ISP */
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB12,   /* 12-bit Bayer RGRG/GBGB */
		.align = 32,
		.bit_depth = 12,
		.ibpp = 0,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB14,   /* 14-bit Bayer RGRG/GBGB */
		.align = 32,
		.bit_depth = 14,
		.ibpp = 1,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB16,   /* 16-bit Bayer RGRG/GBGB */
		.align = 32,
		.bit_depth = 16,
		.ibpp = 2,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.align = 32,
		.bit_depth = 8,
		.ibpp = 6,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR10,
		.align = 32,
		.bit_depth = 10,
		.ibpp = 4,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR10P,
		.align = 32,
		.bit_depth = 10,
		.ibpp = 5,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR12,
		.align = 32,
		.bit_depth = 12,
		.ibpp = 0,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR14,
		.align = 32,
		.bit_depth = 14,
		.ibpp = 1,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR16,
		.align = 32,
		.bit_depth = 16,
		.ibpp = 2,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.align = 32,
		.bit_depth = 8,
		.ibpp = 6,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG10,
		.align = 32,
		.bit_depth = 10,
		.ibpp = 4,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG10P,
		.align = 32,
		.bit_depth = 10,
		.ibpp = 5,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG12,
		.align = 32,
		.bit_depth = 12,
		.ibpp = 0,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG14,
		.align = 32,
		.bit_depth = 14,
		.ibpp = 1,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG16,
		.align = 32,
		.bit_depth = 16,
		.ibpp = 2,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.align = 32,
		.bit_depth = 8,
		.ibpp = 6,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG10,
		.align = 32,
		.bit_depth = 10,
		.ibpp = 4,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG10P,
		.align = 32,
		.bit_depth = 10,
		.ibpp = 5,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG12,
		.align = 32,
		.bit_depth = 12,
		.ibpp = 0,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG14,
		.align = 32,
		.bit_depth = 14,
		.ibpp = 1,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG16,
		.align = 32,
		.bit_depth = 16,
		.ibpp = 2,
		.num_planes = 1,
		.type = NEOISP_FMT_VIDEO_OUTPUT
	}
};

/* META OUTPUT */
const struct neoisp_fmt_s formats_mout[NEOISP_FMT_MOUT_COUNT] = {
	{
		.fourcc = V4L2_META_FMT_NEO_ISP_PARAMS, /* NXP neo isp 3A parameters */
		.align = 32,
		.bit_depth = 8,
		.num_planes = 1,
		.type = NEOISP_FMT_META_OUTPUT
	}
};

/* META CAPTURE */
const struct neoisp_fmt_s formats_mcap[NEOISP_FMT_MCAP_COUNT] = {
	{
		.fourcc = V4L2_META_FMT_NEO_ISP_STATS, /* NXP neo isp 3A Statistics */
		.align = 32,
		.bit_depth = 8,
		.num_planes = 1,
		.type = NEOISP_FMT_META_CAPTURE
	}
};
