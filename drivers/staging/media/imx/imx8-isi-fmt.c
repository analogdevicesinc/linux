// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP
 *
 */

#include "imx8-isi-core.h"

struct mxc_isi_fmt mxc_isi_out_formats[] = {
	{
		.name		= "RGB565",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RGB565,
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 3,
		.mbus_code  = MEDIA_BUS_FMT_RGB565_1X16,
	}, {
		.name		= "RGB24",
		.fourcc		= V4L2_PIX_FMT_RGB24,
		.depth		= { 24 },
		.color		= MXC_ISI_OUT_FMT_BGR32P,
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 2,
		.mbus_code  = MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.name		= "BGR24",
		.fourcc		= V4L2_PIX_FMT_BGR24,
		.depth		= { 24 },
		.color		= MXC_ISI_OUT_FMT_RGB32P,
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 2,
		.mbus_code  = MEDIA_BUS_FMT_BGR888_1X24,
	}, {
		.name		= "YUYV-16",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_YUV422_1P8P,
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 3,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.name		= "YUV32 (X-Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_YUV444_1P8,
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 2,
		.mbus_code	= MEDIA_BUS_FMT_AYUV8_1X32,
	}, {
		.name		= "NV12 (YUYV)",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= { 8, 8 },
		.color		= MXC_ISI_OUT_FMT_YUV420_2P8P,
		.memplanes	= 1,
		.colplanes	= 2,
		.align		= 4,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.name		= "NV12M (YUYV)",
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.depth		= { 8, 8 },
		.color		= MXC_ISI_OUT_FMT_YUV420_2P8P,
		.memplanes	= 2,
		.colplanes	= 2,
		.align		= 4,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.name		= "YUV444M (Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUV444M,
		.depth		= { 8, 8, 8 },
		.color		= MXC_ISI_OUT_FMT_YUV444_3P8P,
		.memplanes	= 3,
		.colplanes	= 3,
		.align		= 4,
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,
	}, {
		.name		= "xBGR32",
		.fourcc		= V4L2_PIX_FMT_XBGR32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_XRGB32,
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 2,
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.name		= "ABGR32",
		.fourcc		= V4L2_PIX_FMT_ABGR32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_ARGB32,
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 2,
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
	}
};

size_t mxc_isi_out_formats_size = ARRAY_SIZE(mxc_isi_out_formats);
