/*
 *    VSI v4l2 media config manager.
 *
 *    Copyright (c) 2019, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License, version 2, as
 *    published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License version 2 for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 at the following locations:
 *    https://opensource.org/licenses/gpl-2.0.php
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include "vsi-v4l2-priv.h"

static struct vsi_v4l2_dev_info vsi_v4l2_hwconfig = {0};

/*copy from daemon header*/
enum VCEncLevel {
	VCENC_HEVC_LEVEL_1 = 30,
	VCENC_HEVC_LEVEL_2 = 60,
	VCENC_HEVC_LEVEL_2_1 = 63,
	VCENC_HEVC_LEVEL_3 = 90,
	VCENC_HEVC_LEVEL_3_1 = 93,
	VCENC_HEVC_LEVEL_4 = 120,
	VCENC_HEVC_LEVEL_4_1 = 123,
	VCENC_HEVC_LEVEL_5 = 150,
	VCENC_HEVC_LEVEL_5_1 = 153,
	VCENC_HEVC_LEVEL_5_2 = 156,
	VCENC_HEVC_LEVEL_6 = 180,
	VCENC_HEVC_LEVEL_6_1 = 183,
	VCENC_HEVC_LEVEL_6_2 = 186,

	/* H264 Defination*/
	VCENC_H264_LEVEL_1 = 10,
	VCENC_H264_LEVEL_1_b = 99,
	VCENC_H264_LEVEL_1_1 = 11,
	VCENC_H264_LEVEL_1_2 = 12,
	VCENC_H264_LEVEL_1_3 = 13,
	VCENC_H264_LEVEL_2 = 20,
	VCENC_H264_LEVEL_2_1 = 21,
	VCENC_H264_LEVEL_2_2 = 22,
	VCENC_H264_LEVEL_3 = 30,
	VCENC_H264_LEVEL_3_1 = 31,
	VCENC_H264_LEVEL_3_2 = 32,
	VCENC_H264_LEVEL_4 = 40,
	VCENC_H264_LEVEL_4_1 = 41,
	VCENC_H264_LEVEL_4_2 = 42,
	VCENC_H264_LEVEL_5 = 50,
	VCENC_H264_LEVEL_5_1 = 51,
	VCENC_H264_LEVEL_5_2 = 52,
};

static s32 leveltbl_hevc[][3] = {
	{VCENC_HEVC_LEVEL_1,	V4L2_MPEG_VIDEO_HEVC_LEVEL_1,  35000},
	{VCENC_HEVC_LEVEL_2,	V4L2_MPEG_VIDEO_HEVC_LEVEL_2,    1500000},
	{VCENC_HEVC_LEVEL_2_1,	V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1,  3000000},
	{VCENC_HEVC_LEVEL_3,	V4L2_MPEG_VIDEO_HEVC_LEVEL_3,    6000000},
	{VCENC_HEVC_LEVEL_3_1,	V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1,  10000000},
	{VCENC_HEVC_LEVEL_4,	V4L2_MPEG_VIDEO_HEVC_LEVEL_4,    12000000},
	{VCENC_HEVC_LEVEL_4_1,	V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1,  20000000},
	{VCENC_HEVC_LEVEL_5,	V4L2_MPEG_VIDEO_HEVC_LEVEL_5,    25000000},
	{VCENC_HEVC_LEVEL_5_1,	V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1,  40000000},
	{VCENC_HEVC_LEVEL_5_2,	V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2,  60000000},
	{VCENC_HEVC_LEVEL_6,	V4L2_MPEG_VIDEO_HEVC_LEVEL_6,    60000000},
	{VCENC_HEVC_LEVEL_6_1,	V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1,  120000000},
	{VCENC_HEVC_LEVEL_6_2,	V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2,  240000000},
};
static s32 leveltbl_h264[][3] = {
	{VCENC_H264_LEVEL_1,	V4L2_MPEG_VIDEO_H264_LEVEL_1_0, 175000},
	{VCENC_H264_LEVEL_1_b,	V4L2_MPEG_VIDEO_H264_LEVEL_1B,   350000},
	{VCENC_H264_LEVEL_1_1,	V4L2_MPEG_VIDEO_H264_LEVEL_1_1,  500000},
	{VCENC_H264_LEVEL_1_2,	V4L2_MPEG_VIDEO_H264_LEVEL_1_2,  1000000},
	{VCENC_H264_LEVEL_1_3,	V4L2_MPEG_VIDEO_H264_LEVEL_1_3,  2000000},
	{VCENC_H264_LEVEL_2,	V4L2_MPEG_VIDEO_H264_LEVEL_2_0,  2000000},
	{VCENC_H264_LEVEL_2_1,	V4L2_MPEG_VIDEO_H264_LEVEL_2_1,  4000000},
	{VCENC_H264_LEVEL_2_2,	V4L2_MPEG_VIDEO_H264_LEVEL_2_2,  4000000},
	{VCENC_H264_LEVEL_3,	V4L2_MPEG_VIDEO_H264_LEVEL_3_0,  10000000},
	{VCENC_H264_LEVEL_3_1,	V4L2_MPEG_VIDEO_H264_LEVEL_3_1,  14000000},
	{VCENC_H264_LEVEL_3_2,	V4L2_MPEG_VIDEO_H264_LEVEL_3_2,  20000000},
	{VCENC_H264_LEVEL_4,	V4L2_MPEG_VIDEO_H264_LEVEL_4_0,  25000000},
	{VCENC_H264_LEVEL_4_1,	V4L2_MPEG_VIDEO_H264_LEVEL_4_1,  62500000},
	{VCENC_H264_LEVEL_4_2,	V4L2_MPEG_VIDEO_H264_LEVEL_4_2,  62500000},
	{VCENC_H264_LEVEL_5,	V4L2_MPEG_VIDEO_H264_LEVEL_5_0,  135000000},
	{VCENC_H264_LEVEL_5_1,	V4L2_MPEG_VIDEO_H264_LEVEL_5_1,  240000000},
	{VCENC_H264_LEVEL_5_2,	V4L2_MPEG_VIDEO_H264_LEVEL_5_2,  240000000},
};

static const u8 colorprimaries[] = {
	V4L2_COLORSPACE_LAST,
	V4L2_COLORSPACE_REC709,        /*Rec. ITU-R BT.709-6*/
	0,
	0,
	V4L2_COLORSPACE_470_SYSTEM_M, /*Rec. ITU-R BT.470-6 System M*/
	V4L2_COLORSPACE_470_SYSTEM_BG,/*Rec. ITU-R BT.470-6 System B, G*/
	V4L2_COLORSPACE_SMPTE170M,    /*SMPTE170M*/
	V4L2_COLORSPACE_SMPTE240M,    /*SMPTE240M*/
	V4L2_COLORSPACE_GENERIC_FILM, /*Generic film*/
	V4L2_COLORSPACE_BT2020,       /*Rec. ITU-R BT.2020-2*/
	V4L2_COLORSPACE_ST428         /*SMPTE ST 428-1*/
};

static const u8 colortransfers[] = {
	V4L2_XFER_FUNC_LAST,
	V4L2_XFER_FUNC_709,      /*Rec. ITU-R BT.709-6*/
	0,
	0,
	V4L2_XFER_FUNC_GAMMA22,  /*Rec. ITU-R BT.470-6 System M*/
	V4L2_XFER_FUNC_GAMMA28,  /*Rec. ITU-R BT.470-6 System B, G*/
	V4L2_XFER_FUNC_709,      /*SMPTE170M*/
	V4L2_XFER_FUNC_SMPTE240M,/*SMPTE240M*/
	V4L2_XFER_FUNC_NONE,     /*Linear transfer characteristics*/
	0,
	0,
	V4L2_XFER_FUNC_XVYCC,    /*IEC 61966-2-4*/
	V4L2_XFER_FUNC_BT1361,   /*Rec. ITU-R BT.1361-0 extended colour gamut*/
	V4L2_XFER_FUNC_SRGB,     /*IEC 61966-2-1 sRGB or sYCC*/
	V4L2_XFER_FUNC_709,      /*Rec. ITU-R BT.2020-2 (10 bit system)*/
	V4L2_XFER_FUNC_709,      /*Rec. ITU-R BT.2020-2 (12 bit system)*/
	V4L2_XFER_FUNC_SMPTE2084,/*SMPTE ST 2084*/
	V4L2_XFER_FUNC_ST428,    /*SMPTE ST 428-1*/
	V4L2_XFER_FUNC_HLG       /*Rec. ITU-R BT.2100-0 hybrid log-gamma (HLG)*/
};

static const u8 colormatrixcoefs[] = {
	V4L2_YCBCR_ENC_LAST,
	V4L2_YCBCR_ENC_709,             /*Rec. ITU-R BT.709-6*/
	0,
	0,
	V4L2_YCBCR_ENC_BT470_6M,        /*Title 47 Code of Federal Regulations*/
	V4L2_YCBCR_ENC_601,             /*Rec. ITU-R BT.601-7 625*/
	V4L2_YCBCR_ENC_601,             /*Rec. ITU-R BT.601-7 525*/
	V4L2_YCBCR_ENC_SMPTE240M,       /*SMPTE240M*/
	0,
	V4L2_YCBCR_ENC_BT2020,          /*Rec. ITU-R BT.2020-2*/
	V4L2_YCBCR_ENC_BT2020_CONST_LUM /*Rec. ITU-R BT.2020-2 constant*/
};

static int enc_isRGBformat(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_RGBA32:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_BGR565:
	case V4L2_PIX_FMT_RGBX32:
		return 1;
	default:
		return 0;
	}
}

static int enc_setvui(struct v4l2_format *v4l2fmt, struct v4l2_daemon_enc_params *encparams)
{
	u32 colorspace, quantization, transfer, matrixcoeff;
	int i;

	quantization = v4l2fmt->fmt.pix_mp.quantization;
	colorspace = v4l2fmt->fmt.pix_mp.colorspace;
	transfer = v4l2fmt->fmt.pix_mp.xfer_func;
	matrixcoeff = v4l2fmt->fmt.pix_mp.ycbcr_enc;

	encparams->specific.enc_h26x_cmd.vuiColorDescripPresentFlag = 1;
	encparams->specific.enc_h26x_cmd.vuiVideoSignalTypePresentFlag = 1;
	if (quantization == V4L2_QUANTIZATION_LIM_RANGE ||
		quantization == V4L2_QUANTIZATION_DEFAULT)
		encparams->specific.enc_h26x_cmd.videoRange = 0;
	else
		encparams->specific.enc_h26x_cmd.videoRange = 1;
	encparams->specific.enc_h26x_cmd.vuiColorPrimaries = 0;
	if (colorspace == V4L2_COLORSPACE_SRGB)	//SRGB is duplicated with REC709
		encparams->specific.enc_h26x_cmd.vuiColorPrimaries = 1;
	else {
		for (i = 0; i < ARRAY_SIZE(colorprimaries); i++) {
			if (colorprimaries[i] == colorspace) {
				encparams->specific.enc_h26x_cmd.vuiColorPrimaries = i;
				break;
			}
		}
	}
	encparams->specific.enc_h26x_cmd.vuiTransferCharacteristics = 0;
	for (i = 0; i < ARRAY_SIZE(colortransfers); i++) {
		if (colortransfers[i] == transfer) {
			encparams->specific.enc_h26x_cmd.vuiTransferCharacteristics = i;
			break;
		}
	}
	encparams->specific.enc_h26x_cmd.vuiMatrixCoefficients = 0;
	for (i = 0; i < ARRAY_SIZE(colormatrixcoefs); i++) {
		if (colormatrixcoefs[i] == matrixcoeff) {
			encparams->specific.enc_h26x_cmd.vuiMatrixCoefficients = i;
			break;
		}
	}
	v4l2_klog(LOGLVL_CONFIG, "%s %x from %d:%d:%d:%d to %d:%d:%d:%d",
		__func__, v4l2fmt->fmt.pix_mp.pixelformat, colorspace, transfer, matrixcoeff, quantization,
		encparams->specific.enc_h26x_cmd.vuiColorPrimaries,
		encparams->specific.enc_h26x_cmd.vuiTransferCharacteristics,
		encparams->specific.enc_h26x_cmd.vuiMatrixCoefficients,
		encparams->specific.enc_h26x_cmd.videoRange);
	if (binputqueue(v4l2fmt->type) &&  enc_isRGBformat(v4l2fmt->fmt.pix_mp.pixelformat)) {
		if (encparams->specific.enc_h26x_cmd.videoRange == 0)
			encparams->general.colorConversion = VCENC_RGBTOYUV_BT601_FULL_RANGE;
		/*full/limit range is reversed here for ctrl sw reverse it. May turn back to right setting when it's fixed*/
		switch (colorspace) {
		case V4L2_COLORSPACE_REC709:
			if (encparams->specific.enc_h26x_cmd.videoRange == 0)
				encparams->general.colorConversion = VCENC_RGBTOYUV_BT709_FULL_RANGE;
			else
				encparams->general.colorConversion = VCENC_RGBTOYUV_BT709;
			break;
		case V4L2_COLORSPACE_JPEG:
			if (encparams->specific.enc_h26x_cmd.videoRange == 0)
				encparams->general.colorConversion = VCENC_RGBTOYUV_BT601_FULL_RANGE;
			else
				encparams->general.colorConversion = VCENC_RGBTOYUV_BT601;
			break;
		case V4L2_COLORSPACE_BT2020:
			encparams->general.colorConversion = VCENC_RGBTOYUV_BT2020;
			break;
		default:
			break;
		}
	}
	return 0;
}

void vsi_dec_getvui(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	struct v4l2_daemon_dec_info *dec_info = &ctx->mediacfg.decparams.dec_info.dec_info;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	if (test_bit(CTX_FLAG_SRCCHANGED_BIT, &ctx->flag))
		pix->quantization = (dec_info->video_range == 0 ?  V4L2_QUANTIZATION_LIM_RANGE :
								   V4L2_QUANTIZATION_FULL_RANGE);
	else
		pix->quantization = ctx->mediacfg.quantization;
	if (dec_info->colour_description_present_flag) {
		if (dec_info->colour_primaries < ARRAY_SIZE(colorprimaries))
			pix->colorspace = colorprimaries[dec_info->colour_primaries];
		if (dec_info->transfer_characteristics < ARRAY_SIZE(colortransfers))
			pix->xfer_func = colortransfers[dec_info->transfer_characteristics];
		if (dec_info->matrix_coefficients < ARRAY_SIZE(colormatrixcoefs))
			pix->ycbcr_enc = colormatrixcoefs[dec_info->matrix_coefficients];
	} else {
		pix->colorspace = ctx->mediacfg.colorspace;
		pix->xfer_func = ctx->mediacfg.xfer_func;
		pix->ycbcr_enc = ctx->mediacfg.ycbcr_enc;
	}

	v4l2_klog(LOGLVL_CONFIG, "%s:%x:%d:%d:%d:%d",
		  __func__, pix->pixelformat, pix->colorspace, pix->xfer_func,
		  pix->ycbcr_enc, pix->quantization);
}

void vsi_dec_updatevui(struct v4l2_daemon_dec_info *src, struct v4l2_daemon_dec_info *dst)
{
	v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d:%d", __func__,
		src->colour_primaries, src->transfer_characteristics, src->matrix_coefficients);
	dst->colour_description_present_flag = 1;
	dst->colour_primaries = src->colour_primaries;
	dst->transfer_characteristics = src->transfer_characteristics;
	dst->matrix_coefficients = src->matrix_coefficients;
}

static void vsi_enum_decfsize(struct v4l2_frmsizeenum *f, u32 pixel_format)
{
	if (vsi_v4l2_hwconfig.max_dec_resolution > 1920) {
		switch (pixel_format) {
		case V4L2_PIX_FMT_HEVC:
		case V4L2_PIX_FMT_VP9:
			f->stepwise.min_width = 144;
			f->stepwise.max_width = 4096;
			f->stepwise.min_height = 144;
			f->stepwise.max_height = 2304;
			break;
		case V4L2_PIX_FMT_H264:
		case V4L2_PIX_FMT_VP8:
		case V4L2_PIX_FMT_VC1_ANNEX_G:
		case V4L2_PIX_FMT_VC1_ANNEX_L:
			f->stepwise.min_width = 48;
			f->stepwise.max_width = 4096;
			f->stepwise.min_height = 48;
			f->stepwise.max_height = 4096;
			break;
		case V4L2_PIX_FMT_MPEG4:
		case V4L2_PIX_FMT_XVID:
		case V4L2_PIX_FMT_MPEG2:
		case V4L2_PIX_FMT_MPEG1:
		case V4L2_PIX_FMT_H263:
			f->stepwise.min_width = 48;
			f->stepwise.max_width = 1920;
			f->stepwise.min_height = 48;
			f->stepwise.max_height = 1088;
			break;
		case V4L2_PIX_FMT_JPEG:
			f->stepwise.min_width = 48;
			f->stepwise.max_width = 16368;
			f->stepwise.min_height = 48;
			f->stepwise.max_height = 16368;
			break;
		default:
			f->stepwise.min_width = 48;
			f->stepwise.max_width = 1920;
			f->stepwise.min_height = 48;
			f->stepwise.max_height = 1088;
			break;
		}
	} else {
		switch (pixel_format) {
		case V4L2_PIX_FMT_HEVC:
		case V4L2_PIX_FMT_VP9:
			f->stepwise.min_width = 144;
			f->stepwise.max_width = 1920;
			f->stepwise.min_height = 144;
			f->stepwise.max_height = 1088;
			break;
		case V4L2_PIX_FMT_H264:
		case V4L2_PIX_FMT_VP8:
			f->stepwise.min_width = 48;
			f->stepwise.max_width = 1920;
			f->stepwise.min_height = 48;
			f->stepwise.max_height = 4096;
			break;
		default:
			f->stepwise.min_width = 144;
			f->stepwise.max_width = 1920;
			f->stepwise.min_height = 144;
			f->stepwise.max_height = 1088;
			break;
		}
	}

	if (pixel_format == V4L2_PIX_FMT_HEVC || pixel_format == V4L2_PIX_FMT_VP9) {
		f->stepwise.step_width = 8;
		f->stepwise.step_height = 8;
	} else {
		f->stepwise.step_width = 16;
		f->stepwise.step_height = 16;
	}
}

int vsi_get_Level(struct vsi_v4l2_ctx *ctx, int mediatype, int dir, int level)
{
	int i, size;
	int (*table)[3];

	if (mediatype == 0) {
		table = leveltbl_h264;
		size = ARRAY_SIZE(leveltbl_h264);
	} else {
		table = leveltbl_hevc;
		size = ARRAY_SIZE(leveltbl_hevc);
	}
	for (i = 0; i < size; i++) {
		if (dir == 0 && level == table[i][0]) {
			v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d:%d", __func__, dir, level, table[i][1]);
			return table[i][1];
		}
		if (dir == 1 && level == table[i][1]) {
			//ctx->mediacfg.encparams.specific.enc_h26x_cmd.cpbSize = table[i][2];
			v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d:%d", __func__, dir, level, table[i][0]);
			return table[i][0];
		}
	}
	if (dir == 0 && level == DEFAULTLEVEL)
		return (mediatype == 0 ? V4L2_MPEG_VIDEO_H264_LEVEL_1_0 :
			V4L2_MPEG_VIDEO_HEVC_LEVEL_1);
	return -EINVAL;
}

static struct vsi_video_fmt vsi_raw_fmt[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.enc_fmt = VCENC_YUV420_SEMIPLANAR,
		.dec_fmt = VSI_V4L2_DEC_PIX_FMT_NV12,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.enc_fmt = VCENC_YUV420_SEMIPLANAR,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 2,
		.comp_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_GREY,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DEC_PIX_FMT_400,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "411 semi planar",
		.fourcc = V4L2_PIX_FMT_411SP,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DEC_PIX_FMT_411SP,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DEC_PIX_FMT_422SP,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV24,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DEC_PIX_FMT_444SP,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.enc_fmt = VCENC_YUV420_PLANAR,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 3,
		.comp_planes = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.enc_fmt = VCENC_YUV420_PLANAR,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21M,
		.enc_fmt = VCENC_YUV420_SEMIPLANAR_VU,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 2,
		.comp_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.enc_fmt = VCENC_YUV420_SEMIPLANAR_VU,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.enc_fmt = VCENC_YUV422_INTERLEAVED_YUYV,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGB565,
		.enc_fmt = VCENC_RGB565,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "BGR16",
		.fourcc = V4L2_PIX_FMT_BGR565,
		.enc_fmt = VCENC_BGR565,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGB555,
		.enc_fmt = VCENC_RGB555,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGBA32,
		.enc_fmt = VCENC_BGR888,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_BGR32,
		.enc_fmt = VCENC_RGB888,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_ABGR32,
		.enc_fmt = VCENC_RGB888,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGBX32,
		.enc_fmt = VCENC_BGR888,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "VSI DTRC",
		.fourcc = V4L2_PIX_FMT_DTRC,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DECOUT_DTRC,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "P010",
		.fourcc = V4L2_PIX_FMT_P010,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DECOUT_P010,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "NV12 10Bit",
		.fourcc = V4L2_PIX_FMT_NV12X,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DECOUT_NV12_10BIT,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "DTRC 10Bit",
		.fourcc = V4L2_PIX_FMT_TILEX,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DECOUT_DTRC_10BIT,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "VSI DTRC compressed",
		.fourcc = V4L2_PIX_FMT_RFC,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DECOUT_RFC,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
	{
		.name = "VSI DTRC 10 bit compressed",
		.fourcc = V4L2_PIX_FMT_RFCX,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = VSI_V4L2_DECOUT_RFC_10BIT,
		.flag = 0,
		.num_planes = 1,
		.comp_planes = 1,
	},
};

static struct vsi_video_fmt vsi_coded_fmt[] = {
	{
		.fourcc = V4L2_PIX_FMT_HEVC,
		.enc_fmt = V4L2_DAEMON_CODEC_ENC_HEVC,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_HEVC,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(0),
	},
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.enc_fmt = V4L2_DAEMON_CODEC_ENC_H264,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_H264,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(1),
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_JPEG,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(2),
	},
	{
		.fourcc = V4L2_PIX_FMT_VP8,
		.enc_fmt = V4L2_DAEMON_CODEC_ENC_VP8,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_VP8,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(3),
	},
	{
		.fourcc = V4L2_PIX_FMT_VP9,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_VP9,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(4),
	},
	{
		.name = "av1",
		.fourcc = V4L2_PIX_FMT_AV1,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(5),
	},
	{
		.fourcc = V4L2_PIX_FMT_MPEG2,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_MPEG2,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(6),
	},
	{
		.fourcc = V4L2_PIX_FMT_MPEG4,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_MPEG4,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(7),
	},
	{
		.fourcc = V4L2_PIX_FMT_H263,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_H263,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(8),
	},
	{
		.fourcc = V4L2_PIX_FMT_VC1_ANNEX_G,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_VC1_G,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(9),
	},
	{
		.fourcc = V4L2_PIX_FMT_VC1_ANNEX_L,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_VC1_L,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(10),
	},
	{
		.name = "rv30",
		.fourcc = V4L2_PIX_FMT_RV30,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_RV,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(11),
	},
	{
		.name = "rv40",
		.fourcc = V4L2_PIX_FMT_RV40,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_RV,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(11),
	},
	{
		.name = "avs",
		.fourcc = V4L2_PIX_FMT_AVS,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_AVS2,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(12),
	},
	{
		.fourcc = V4L2_PIX_FMT_XVID,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_XVID,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(13),
	},
	{
		.fourcc = V4L2_PIX_FMT_MPEG1,
		.enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE,
		.dec_fmt = V4L2_DAEMON_CODEC_DEC_MPEG2,
		.flag = (V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED),
		.num_planes = 1,
		.comp_planes = 1,
		.mask = BIT(6),
	},
};

static int istiledfmt(int pixelformat)
{
	switch (pixelformat) {
	case VSI_V4L2_DECOUT_DTRC:
	case VSI_V4L2_DECOUT_DTRC_10BIT:
	case VSI_V4L2_DECOUT_RFC:
	case VSI_V4L2_DECOUT_RFC_10BIT:
		return 1;
	default:
		return 0;
	}
}

static int isJpegOnlyFmt(int outfmt)
{
	switch (outfmt) {
	case VSI_V4L2_DEC_PIX_FMT_400:
	case VSI_V4L2_DEC_PIX_FMT_411SP:
	case VSI_V4L2_DEC_PIX_FMT_422SP:
	case VSI_V4L2_DEC_PIX_FMT_444SP:
		return 1;
	default:
		return 0;
	}
}

void vsi_enum_encfsize(struct v4l2_frmsizeenum *f, u32 pixel_format)
{
	switch (pixel_format) {
	case V4L2_PIX_FMT_HEVC:
		f->stepwise.min_width = 136;
		f->stepwise.max_width = 1920;
		f->stepwise.step_width = 2;
		f->stepwise.min_height = 136;
		f->stepwise.max_height = 8192;
		f->stepwise.step_height = 2;
		break;
	case V4L2_PIX_FMT_H264:
		if (vsi_v4l2_hwconfig.enc_isH1) {
			f->stepwise.min_width = 144;
			f->stepwise.max_width = 1920;
			f->stepwise.step_width = 4;
			f->stepwise.min_height = 96;
			f->stepwise.max_height = 4080;
			f->stepwise.step_height = 4;
		} else {
			f->stepwise.min_width = 144;
			f->stepwise.max_width = 1920;
			f->stepwise.step_width = 2;
			f->stepwise.min_height = 144;
			f->stepwise.max_height = 8192;
			f->stepwise.step_height = 2;
		}
		break;
	case V4L2_PIX_FMT_VP8:
		f->stepwise.min_width = 144;
		f->stepwise.max_width = 1920;
		f->stepwise.step_width = 4;
		f->stepwise.min_height = 96;
		f->stepwise.max_height = 4080;
		f->stepwise.step_height = 4;
		break;
	default:
		if (vsi_v4l2_hwconfig.enc_isH1) {
			f->stepwise.min_width = 144;
			f->stepwise.max_width = 1920;
			f->stepwise.step_width = 4;
			f->stepwise.min_height = 96;
			f->stepwise.max_height = 4080;
			f->stepwise.step_height = 4;
		} else {
			f->stepwise.min_width = 144;
			f->stepwise.max_width = 1920;
			f->stepwise.step_width = 2;
			f->stepwise.min_height = 144;
			f->stepwise.max_height = 8192;
			f->stepwise.step_height = 2;
		}
		break;
	}
	f->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	v4l2_klog(LOGLVL_CONFIG, "%s:%x->%d:%d:%d:%d:%d:%d",
		__func__, pixel_format,
		f->stepwise.min_width, f->stepwise.max_width, f->stepwise.step_width,
		f->stepwise.min_height, f->stepwise.max_height, f->stepwise.step_height);
}

int vsi_set_profile(struct vsi_v4l2_ctx *ctx, int type, int profile)
{
	static int h264list[] = {
		VCENC_H264_BASE_PROFILE,
		VCENC_H264_BASE_PROFILE,
		VCENC_H264_MAIN_PROFILE,
		VCENC_H264_BASE_PROFILE,
		VCENC_H264_HIGH_PROFILE,
		VCENC_H264_HIGH_10_PROFILE,
		VCENC_H264_HIGH_10_PROFILE,
		VCENC_H264_HIGH_10_PROFILE,
		VCENC_H264_HIGH_10_PROFILE,
		VCENC_H264_HIGH_10_PROFILE,
		VCENC_H264_HIGH_10_PROFILE,
		VCENC_H264_BASE_PROFILE,
		VCENC_H264_BASE_PROFILE,
		VCENC_H264_HIGH_PROFILE,
		VCENC_H264_HIGH_PROFILE,
		VCENC_H264_HIGH_PROFILE,
		VCENC_H264_HIGH_PROFILE,
	};
	static int hevclist[] = {
		VCENC_HEVC_MAIN_PROFILE,
		VCENC_HEVC_MAIN_STILL_PICTURE_PROFILE,
		VCENC_HEVC_MAIN_10_PROFILE,
	};
	static int vp9list[] = {
		VCENC_VP9_MAIN_PROFILE,
		VCENC_VP9_MSRGB_PROFILE,
		VCENC_VP9_HIGH_PROFILE,
		VCENC_VP9_HSRGB_PROFILE,
	};

	v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d", __func__, type, profile);
	switch (type) {
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		if (profile >= ARRAY_SIZE(h264list))
			return -EINVAL;
		ctx->mediacfg.profile_h264 = h264list[profile];
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		if (profile >= ARRAY_SIZE(hevclist))
			return -EINVAL;
		ctx->mediacfg.profile_hevc = hevclist[profile];
		return 0;
	case V4L2_CID_MPEG_VIDEO_VP8_PROFILE:
		ctx->mediacfg.profile_vp8 = profile;
		return 0;
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
		if (profile >= ARRAY_SIZE(vp9list))
			return -EINVAL;
		ctx->mediacfg.profile_vp9 = vp9list[profile];
		return 0;
	default:
		return -EINVAL;
	}
}

int vsi_get_profile(struct vsi_v4l2_ctx *ctx, int type)
{
	v4l2_klog(LOGLVL_CONFIG, "%s:%d", __func__, type);
	switch (type) {
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		if (ctx->mediacfg.profile_h264 == VCENC_H264_BASE_PROFILE)
			return V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
		else if (ctx->mediacfg.profile_h264 == VCENC_H264_MAIN_PROFILE)
			return V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
		else if (ctx->mediacfg.profile_h264 == VCENC_H264_HIGH_PROFILE)
			return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
		else
			return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10;
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		if (ctx->mediacfg.profile_hevc == VCENC_HEVC_MAIN_PROFILE)
			return V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
		else if (ctx->mediacfg.profile_hevc == VCENC_HEVC_MAIN_STILL_PICTURE_PROFILE)
			return V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE;
		else
			return V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10;
	case V4L2_CID_MPEG_VIDEO_VP8_PROFILE:
		return ctx->mediacfg.profile_vp8;
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
		if (ctx->mediacfg.profile_vp9 == VCENC_VP9_MAIN_PROFILE)
			return V4L2_MPEG_VIDEO_VP9_PROFILE_0;
		else if (ctx->mediacfg.profile_vp9 == VCENC_VP9_MSRGB_PROFILE)
			return V4L2_MPEG_VIDEO_VP9_PROFILE_1;
		else if (ctx->mediacfg.profile_vp9 == VCENC_VP9_HIGH_PROFILE)
			return V4L2_MPEG_VIDEO_VP9_PROFILE_2;
		else
			return V4L2_MPEG_VIDEO_VP9_PROFILE_3;
	default:
		return -EINVAL;
	}

}

struct vsi_video_fmt *vsi_find_format(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	u32 fourcc;
	int i;
	int braw = brawfmt(ctx->flag, fmt->type);
	struct vsi_video_fmt *retfmt = NULL;

	if (isencoder(ctx))
		fourcc = fmt->fmt.pix_mp.pixelformat;
	else
		fourcc = fmt->fmt.pix.pixelformat;
	if (braw) {
		for (i = 0; i < ARRAY_SIZE(vsi_raw_fmt); i++) {
			if (vsi_raw_fmt[i].fourcc == fourcc) {
				if (isencoder(ctx) && vsi_raw_fmt[i].enc_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
					retfmt = &vsi_raw_fmt[i];
				if (isdecoder(ctx) && vsi_raw_fmt[i].dec_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
					retfmt = &vsi_raw_fmt[i];
				break;
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(vsi_coded_fmt); i++) {
			if (vsi_coded_fmt[i].fourcc == fourcc) {
				if (isencoder(ctx) && vsi_coded_fmt[i].enc_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
					retfmt = &vsi_coded_fmt[i];
				if (isdecoder(ctx) && vsi_coded_fmt[i].dec_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
					retfmt = &vsi_coded_fmt[i];
				break;
			}
		}
	}
	return retfmt;
}

struct vsi_video_fmt *vsi_enum_dec_format(int idx, int braw, struct vsi_v4l2_ctx *ctx)
{
	u32 inputformat = ctx->mediacfg.decparams.dec_info.io_buffer.inputFormat;
	int i = 0, k =  -1, outfmt;

	if (braw == 1) {
		for (; i < ARRAY_SIZE(vsi_raw_fmt); i++) {
			outfmt = vsi_raw_fmt[i].dec_fmt;
			if (outfmt == V4L2_DAEMON_CODEC_UNKNOW_TYPE)
				continue;
			if (istiledfmt(outfmt)) {
				if (inputformat != V4L2_DAEMON_CODEC_DEC_HEVC &&
					inputformat != V4L2_DAEMON_CODEC_DEC_VP9)
					continue;
				if ((outfmt == VSI_V4L2_DECOUT_DTRC ||
					outfmt == VSI_V4L2_DECOUT_DTRC_10BIT) &&
					vsi_v4l2_hwconfig.max_dec_resolution > 1920)
					continue;
				if ((outfmt == VSI_V4L2_DECOUT_RFC ||
					outfmt == VSI_V4L2_DECOUT_RFC_10BIT) &&
					vsi_v4l2_hwconfig.max_dec_resolution <= 1920)
					continue;
			}
			if (test_bit(CTX_FLAG_SRCCHANGED_BIT, &ctx->flag)) {
				if (inputformat == V4L2_DAEMON_CODEC_DEC_JPEG &&
					outfmt != ctx->mediacfg.decparams.dec_info.dec_info.src_pix_fmt)
					continue;
				if ((outfmt == VSI_V4L2_DECOUT_NV12_10BIT ||
					outfmt == VSI_V4L2_DECOUT_P010) &&
					ctx->mediacfg.decparams.dec_info.dec_info.bit_depth < 10)
					continue;
				if ((outfmt == VSI_V4L2_DECOUT_DTRC_10BIT ||
					outfmt == VSI_V4L2_DECOUT_RFC_10BIT) &&
					ctx->mediacfg.decparams.dec_info.dec_info.bit_depth != 10)
					continue;
				if ((outfmt == VSI_V4L2_DECOUT_DTRC ||
					outfmt == VSI_V4L2_DECOUT_RFC) &&
					ctx->mediacfg.decparams.dec_info.dec_info.bit_depth != 8)
					continue;
			}
			k++;

			if (k == idx) {
				v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d=%x", __func__, idx, braw, vsi_raw_fmt[i].fourcc);
				return &vsi_raw_fmt[i];
			}
		}
	} else {
		for (; i < ARRAY_SIZE(vsi_coded_fmt); i++) {
			if (vsi_coded_fmt[i].dec_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
				k++;
			if (k == idx) {
				v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d=%x", __func__, idx, braw, vsi_coded_fmt[i].fourcc);
				return &vsi_coded_fmt[i];
			}
		}
	}
	return NULL;
}

struct vsi_video_fmt *vsi_enum_encformat(int idx, int braw)
{
	int i = 0, k =  -1;

	if (braw == 1) {
		for (; i < ARRAY_SIZE(vsi_raw_fmt); i++) {
			if (vsi_raw_fmt[i].enc_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
				k++;
			if (k == idx) {
				v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d=%x", __func__, idx, braw, vsi_raw_fmt[i].fourcc);
				return &vsi_raw_fmt[i];
			}
		}
	} else {
		for (; i < ARRAY_SIZE(vsi_coded_fmt); i++) {
			if (vsi_coded_fmt[i].enc_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
				k++;
			if (k == idx) {
				v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d=%x", __func__, idx, braw, vsi_coded_fmt[i].fourcc);
				return &vsi_coded_fmt[i];
			}
		}
	}
	return NULL;
}

static void vsi_set_default_parameter_enc(
	struct v4l2_daemon_enc_params *enc_params,
	enum v4l2_daemon_codec_fmt fmt)
{
	/*general*/
	enc_params->general.outputRateNumer = 30;
	enc_params->general.outputRateDenom = 1;
	enc_params->general.inputRateNumer = 30;
	enc_params->general.inputRateDenom = 1;
	enc_params->general.outputRateNumer = 30;
	enc_params->general.outputRateDenom = 1;
	enc_params->general.lastPic = 100;
	enc_params->general.inputFormat = VCENC_FMT_INVALID;
	enc_params->general.bitPerSecond = 1000000;
	enc_params->general.colorConversion = -1;
	enc_params->general.codecFormat = V4L2_DAEMON_CODEC_ENC_HEVC;

	enc_params->specific.enc_h26x_cmd.byteStream = 1;
	enc_params->specific.enc_h26x_cmd.profile = -1;
	enc_params->specific.enc_h26x_cmd.tier = -1;
	enc_params->specific.enc_h26x_cmd.avclevel = DEFAULTLEVEL;
	enc_params->specific.enc_h26x_cmd.hevclevel = DEFAULTLEVEL;
	enc_params->specific.enc_h26x_cmd.intraAreaTop = 1;
	enc_params->specific.enc_h26x_cmd.intraAreaLeft = 1;
	enc_params->specific.enc_h26x_cmd.intraAreaBottom = 1;
	enc_params->specific.enc_h26x_cmd.intraAreaRight = -1;
	enc_params->specific.enc_h26x_cmd.pcm_loop_filter_disabled_flag = 1;
	/* Rate control parameters */
	enc_params->specific.enc_h26x_cmd.hrdConformance = -1;
	enc_params->specific.enc_h26x_cmd.cpbSize = -1;	//let daemon decides
	enc_params->specific.enc_h26x_cmd.intraPicRate = DEFAULT_INTRA_PIC_RATE;
	enc_params->specific.enc_h26x_cmd.qpHdr = DEFAULT_QP;
	enc_params->specific.enc_h26x_cmd.qpHdrI_h26x = -1;
	enc_params->specific.enc_h26x_cmd.qpHdrP_h26x = -1;
	enc_params->specific.enc_h26x_cmd.qpHdrI_vpx = -1;
	enc_params->specific.enc_h26x_cmd.qpHdrP_vpx = -1;
	enc_params->specific.enc_h26x_cmd.qpMax_h26x = 51;
	enc_params->specific.enc_h26x_cmd.qpMax_vpx = 127;
	enc_params->specific.enc_h26x_cmd.qpMaxI = 51;
	enc_params->specific.enc_h26x_cmd.bitVarRangeI = 10000;
	enc_params->specific.enc_h26x_cmd.bitVarRangeP = 10000;
	enc_params->specific.enc_h26x_cmd.bitVarRangeB = 10000;
	enc_params->specific.enc_h26x_cmd.u32StaticSceneIbitPercent = 80;
	enc_params->specific.enc_h26x_cmd.tolMovingBitRate = 2000;
	enc_params->specific.enc_h26x_cmd.monitorFrames = -1;
	enc_params->specific.enc_h26x_cmd.picRc = -1;	//for VBR and CBR, 0 only for CQP
	enc_params->specific.enc_h26x_cmd.ctbRc = -1;
	enc_params->specific.enc_h26x_cmd.blockRCSize = -1;
	enc_params->specific.enc_h26x_cmd.rcQpDeltaRange = -1;
	enc_params->specific.enc_h26x_cmd.rcBaseMBComplexity = -1;
	enc_params->specific.enc_h26x_cmd.picQpDeltaMin = -1;
	enc_params->specific.enc_h26x_cmd.picQpDeltaMax = -1;
	enc_params->specific.enc_h26x_cmd.ctbRcRowQpStep = -1;
	enc_params->specific.enc_h26x_cmd.tolCtbRcInter = -1;
	enc_params->specific.enc_h26x_cmd.tolCtbRcIntra = -1;
	enc_params->specific.enc_h26x_cmd.bitrateWindow = 150;
	enc_params->specific.enc_h26x_cmd.bFrameQpDelta = -1;
	enc_params->specific.enc_h26x_cmd.enableSao = 1;
	enc_params->specific.enc_h26x_cmd.tc_Offset = -2;
	enc_params->specific.enc_h26x_cmd.beta_Offset = 5;
	enc_params->specific.enc_h26x_cmd.ssim = 1;
	enc_params->specific.enc_h26x_cmd.userData = (kaddr_t)NULL;
	enc_params->specific.enc_h26x_cmd.gopSize = DEFAULT_GOP_SIZE;
	enc_params->specific.enc_h26x_cmd.gopCfg = (kaddr_t)NULL;
	enc_params->specific.enc_h26x_cmd.outReconFrame = 1;
	enc_params->specific.enc_h26x_cmd.ltrInterval = -1;
	enc_params->specific.enc_h26x_cmd.bitDepthLuma = 8;
	enc_params->specific.enc_h26x_cmd.bitDepthChroma = 8;
	enc_params->specific.enc_h26x_cmd.rdoLevel = 3;
	enc_params->specific.enc_h26x_cmd.constCb = -1;
	enc_params->specific.enc_h26x_cmd.constCr = -1;
	/* HDR10 */
	enc_params->specific.enc_h26x_cmd.hdr10_primary = 9;
	enc_params->specific.enc_h26x_cmd.hdr10_matrix = 9;
	enc_params->specific.enc_h26x_cmd.vui_timing_info_enable = 1;
	enc_params->specific.enc_h26x_cmd.log2MaxPicOrderCntLsb = 16;
	enc_params->specific.enc_h26x_cmd.log2MaxFrameNum = 12;
	enc_params->specific.enc_h26x_cmd.cuInfoVersion = 2;
	enc_params->specific.enc_h26x_cmd.parallelCoreNum = 1;
	enc_params->specific.enc_h26x_cmd.idrHdr = 1;
}

static struct vsi_video_fmt *vsi_find_first_fmt(bool is_enc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vsi_coded_fmt); i++) {
		if (is_enc && vsi_coded_fmt[i].enc_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
			return &vsi_coded_fmt[i];
		if (!is_enc && vsi_coded_fmt[i].dec_fmt != V4L2_DAEMON_CODEC_UNKNOW_TYPE)
			return &vsi_coded_fmt[i];
	}

	return NULL;
}

static void vsiv4l2_initfmt(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_format f;
	struct vsi_v4l2_mediacfg *cfg = &ctx->mediacfg;

	if (isencoder(ctx)) {
		memset(&f, 0, sizeof(f));
		f.fmt.pix_mp.width = VSI_DEFAULT_WIDTH;
		f.fmt.pix_mp.height = VSI_DEFAULT_HEIGHT;
		f.fmt.pix_mp.pixelformat = cfg->outfmt_fourcc;
		f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		vsiv4l2_setfmt(ctx, &f);

		memset(&f, 0, sizeof(f));
		f.fmt.pix_mp.width = VSI_DEFAULT_WIDTH;
		f.fmt.pix_mp.height = VSI_DEFAULT_HEIGHT;
		f.fmt.pix_mp.pixelformat = cfg->infmt_fourcc;
		f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		vsiv4l2_setfmt(ctx, &f);
	} else {
		memset(&f, 0, sizeof(f));
		f.fmt.pix.width = VSI_DEFAULT_WIDTH;
		f.fmt.pix.height = VSI_DEFAULT_HEIGHT;
		f.fmt.pix.pixelformat = cfg->infmt_fourcc;
		f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		vsiv4l2_setfmt(ctx, &f);

		memset(&f, 0, sizeof(f));
		f.fmt.pix.width = VSI_DEFAULT_WIDTH;
		f.fmt.pix.height = VSI_DEFAULT_HEIGHT;
		f.fmt.pix.pixelformat = cfg->outfmt_fourcc;
		f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vsiv4l2_setfmt(ctx, &f);
	}
}

void vsiv4l2_initcfg(struct vsi_v4l2_ctx *ctxp)
{
	struct vsi_v4l2_mediacfg *ctx = &ctxp->mediacfg;
	struct vsi_video_fmt *vfmt = NULL;
	int enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE;
	int dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE;
	u32 enc_fmt_fourcc = 0;
	u32 dec_fmt_fourcc = 0;

	v4l2_klog(LOGLVL_CONFIG, "%llx:%s default enc_fmt:%d, dec_fmt:%d\n",
		  ctxp->ctxid, __func__, enc_fmt, dec_fmt);

	if (isencoder(ctxp)) {
		vfmt = vsi_find_first_fmt(1);
		if (!vfmt)
			return;
		enc_fmt = vfmt->enc_fmt;
		enc_fmt_fourcc = vfmt->fourcc;
		vsi_set_default_parameter_enc(&ctx->encparams, enc_fmt);

		ctx->srcplanes = 2;
		ctx->infmt_fourcc = V4L2_PIX_FMT_NV12M;
		ctx->outfmt_fourcc = enc_fmt_fourcc;
		ctx->flags = vfmt->flag;
	} else {
		vfmt = vsi_find_first_fmt(0);
		if (!vfmt)
			return;
		dec_fmt = vfmt->dec_fmt;
		dec_fmt_fourcc = vfmt->fourcc;
		ctx->decparams.dec_info.io_buffer.inputFormat = dec_fmt;
		ctx->decparams.dec_info.io_buffer.outBufFormat = VSI_V4L2_DEC_PIX_FMT_NV12;
		ctx->decparams.dec_info.io_buffer.outputPixelDepth = DEFAULT_PIXELDEPTH;
		ctx->src_pixeldepth = DEFAULT_PIXELDEPTH;
		ctx->decparams.dec_info.dec_info.bit_depth = DEFAULT_PIXELDEPTH;
		ctx->srcplanes = 1;
		ctx->infmt_fourcc = dec_fmt_fourcc;
		ctx->outfmt_fourcc = V4L2_PIX_FMT_NV12;
		ctx->flags = vfmt->flag;
	}
	ctx->dstplanes = 1;
	ctx->profile_hevc = VCENC_HEVC_MAIN_PROFILE;
	ctx->profile_h264 = VCENC_H264_BASE_PROFILE;
	ctx->profile_vp9 = VCENC_VP9_MAIN_PROFILE;
	ctx->encparams.specific.enc_h26x_cmd.gopSize = DEFAULT_GOP_SIZE;

	ctx->encparams.general.inputFormat = VCENC_FMT_INVALID;
	ctx->field = V4L2_FIELD_NONE;
	ctx->colorspace = V4L2_COLORSPACE_REC709;
	ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	ctx->minbuf_4capture = 1;
	ctx->minbuf_4output = 1;

	ctx->capparam.capability = V4L2_CAP_TIMEPERFRAME;
	ctx->capparam.readbuffers = 0;
	ctx->capparam.timeperframe.numerator = 1;
	ctx->capparam.timeperframe.denominator = 25;

	ctx->outputparam.capability = V4L2_CAP_TIMEPERFRAME;
	ctx->outputparam.writebuffers = 0;
	ctx->outputparam.timeperframe.numerator = 1;
	ctx->outputparam.timeperframe.denominator = 25;

	ctx->encparams.general.inputRateNumer = 1;
	ctx->encparams.general.inputRateDenom = 25;

	ctx->encparams.general.outputRateNumer = 1;
	ctx->encparams.general.outputRateDenom = 25;

	ctx->multislice_mode = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE;	//0
	vsiv4l2_initfmt(ctxp);
}

static int get_fmtprofile(struct vsi_v4l2_mediacfg *pcfg)
{
	int codecFormat = pcfg->encparams.general.codecFormat;

	switch (codecFormat) {
	case V4L2_DAEMON_CODEC_ENC_HEVC:
		return pcfg->profile_hevc;
	case V4L2_DAEMON_CODEC_ENC_H264:
		return pcfg->profile_h264;
	case V4L2_DAEMON_CODEC_ENC_VP8:
		return pcfg->profile_vp8;
	case V4L2_DAEMON_CODEC_ENC_VP9:
		return pcfg->profile_vp9;
	default:
		return -EINVAL;
	}
}

static int vsi_calc_table_size(int pixelformat, int width, int height)
{
	int luma_table_size = 0;
	int chroma_table_size = 0;
	int width_in_cbs;
	int height_in_cbs;

	if (pixelformat != V4L2_PIX_FMT_RFC && pixelformat != V4L2_PIX_FMT_RFCX)
		return 0;

	/*luma table size*/
	width_in_cbs = ALIGN(DIV_ROUND_UP(width, 8), 16);
	height_in_cbs = DIV_ROUND_UP(height, 8);
	luma_table_size = ALIGN(width_in_cbs * height_in_cbs, 16);

	/*chroma table size*/
	width_in_cbs = ALIGN(DIV_ROUND_UP(width, 16), 16);
	height_in_cbs = DIV_ROUND_UP(height / 2, 4);
	chroma_table_size = ALIGN(width_in_cbs * height_in_cbs, 16);

	return luma_table_size + chroma_table_size;
}

static void verifyPlanesize(u32 psize[], int braw, int pixelformat, int width, int height, int bdecoder)
{
	int basesize = width * height;
	int chromausize = 0;
	int chromavsize = 0;
	int padsize = 0;
	int tablesize = 0;
	int mvssize = 0;

	switch (pixelformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV12X:
	case V4L2_PIX_FMT_DTRC:
	case V4L2_PIX_FMT_P010:
	case V4L2_PIX_FMT_TILEX:
	case V4L2_PIX_FMT_411SP:
		chromausize = basesize / 2;
		if (bdecoder)
			padsize = basesize / 4 + 32;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YUV420M:
		chromausize = basesize / 4;
		chromavsize = basesize / 4;
		if (bdecoder)
			padsize = basesize / 4 + 32;
		break;
	case V4L2_PIX_FMT_NV16:
		chromausize = basesize;
		break;
	case V4L2_PIX_FMT_NV24:
		chromausize = basesize * 2;
		break;
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_YUYV:
		break;
	case V4L2_PIX_FMT_RFC:
	case V4L2_PIX_FMT_RFCX:
		chromausize = basesize / 2;
		if (bdecoder)
			padsize = basesize / 4 + 32;
		mvssize = DIV_ROUND_UP(width, 64) * DIV_ROUND_UP(height, 64) * 64 * 16;
		padsize = max_t(int, padsize, mvssize);
		tablesize = vsi_calc_table_size(pixelformat, width, height);
		break;
	default:
		break;
	}

	//for coded format we support 1 plane only
	//except certain header the CR data can be any small
	//so just make it page aligned.
	if (!braw && psize[0])
		psize[0] = clamp_val(psize[0], SZ_128K, SZ_8M);
	else if (!braw)
		psize[0] = clamp_val(basesize, SZ_128K, SZ_8M);
	else
		psize[0] = basesize;
	psize[1] = chromausize;
	psize[2] = chromavsize;
	psize[3] = padsize + tablesize;

	v4l2_klog(LOGLVL_CONFIG, "%s:%c%c%c%c:%dx%d:%d:%d:%d:%d",
		  __func__,
		  pixelformat, pixelformat >> 8, pixelformat >> 16, pixelformat >> 24,
		  width, height,
		  psize[0], psize[1], psize[2], psize[3]);
}

struct vsi_video_fmt *vsi_get_fmt_by_fourcc(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vsi_raw_fmt); i++) {
		if (fourcc == vsi_raw_fmt[i].fourcc)
			return &vsi_raw_fmt[i];
	}
	for (i = 0; i < ARRAY_SIZE(vsi_coded_fmt); i++) {
		if (fourcc == vsi_coded_fmt[i].fourcc)
			return &vsi_coded_fmt[i];
	}

	return NULL;
}

static u32 get_plane_size(u32 psize[], u32 planeno, u32 num_planes)
{
	int i;
	u32 size = 0;

	if (planeno >= num_planes)
		return 0;
	if (planeno < num_planes - 1)
		return psize[planeno];
	for (i = planeno; i < 4; i++)
		size += psize[i];
	return size;
}

static int is_doublesizefmt(int fmt)
{
	if (fmt == VCENC_YUV422_INTERLEAVED_YUYV ||
		fmt == VCENC_RGB565 ||
		fmt == VCENC_BGR565 ||
		fmt == VCENC_RGB555)
		return 1;
	return 0;
}

static int is_quadsizefmt(int fmt)
{
	if (fmt == VCENC_RGB888 ||
		fmt == VCENC_BGR888)
		return 1;
	return 0;
}

int vsiv4l2_enc_getalign(u32 srcfmt, u32 dstfmt, int width)
{
	int bytesperline = width;

	switch (dstfmt) {
	case V4L2_DAEMON_CODEC_ENC_HEVC:
		bytesperline = ALIGN(bytesperline, 8);
		break;
	case V4L2_DAEMON_CODEC_ENC_H264:
	case V4L2_DAEMON_CODEC_ENC_VP8:
	default:
		if (vsi_v4l2_hwconfig.enc_isH1) {
			if (is_doublesizefmt(srcfmt))
				bytesperline = ALIGN(bytesperline, 32);
			else if (is_quadsizefmt(srcfmt))
				bytesperline = ALIGN(bytesperline, 64);
			else
				bytesperline = ALIGN(bytesperline, 16);
		} else
			bytesperline = ALIGN(bytesperline, 16);
		break;
	}
	return bytesperline;
}

static int vsiv4l2_setfmt_enc(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
	struct v4l2_pix_format_mplane *pixmp = &fmt->fmt.pix_mp;
	struct vsi_video_fmt *targetfmt;
	int ret = 0;

	ret = vsiv4l2_verifyfmt(ctx, fmt, 0);
	if (ret != 0)
		return ret;
	targetfmt = vsi_find_format(ctx, fmt);
	if (binputqueue(fmt->type)) {
		pcfg->width_src = pixmp->width;
		pcfg->height_src = pixmp->height;
		pcfg->encparams.general.horOffsetSrc = 0;
		pcfg->encparams.general.verOffsetSrc = 0;
		pcfg->encparams.general.width = pixmp->width;
		pcfg->encparams.general.height = pixmp->height;
		pcfg->encparams.general.inputFormat = targetfmt->enc_fmt;
		pcfg->infmt_fourcc = pixmp->pixelformat;
		pcfg->bytesperline = pixmp->plane_fmt[0].bytesperline;
	} else {
		pcfg->encparams.general.width = pixmp->width;
		pcfg->encparams.general.height = pixmp->height;
		pcfg->encparams.general.codecFormat = targetfmt->enc_fmt;
		pcfg->outfmt_fourcc = pixmp->pixelformat;
		pcfg->encparams.specific.enc_h26x_cmd.profile = get_fmtprofile(pcfg);
	}

	if (binputqueue(fmt->type)) {
		if (is_doublesizefmt(targetfmt->enc_fmt))
			pcfg->encparams.general.lumWidthSrc = pcfg->bytesperline/2;
		else if (is_quadsizefmt(targetfmt->enc_fmt))
			pcfg->encparams.general.lumWidthSrc = pcfg->bytesperline/4;
		else
			pcfg->encparams.general.lumWidthSrc = pcfg->bytesperline;

		pcfg->encparams.general.lumHeightSrc = pixmp->height;
	}

	if (binputqueue(fmt->type))
		pcfg->srcplanes = pixmp->num_planes;
	else
		pcfg->dstplanes = pixmp->num_planes;
	pcfg->field = pixmp->field;
	pcfg->colorspace = pixmp->colorspace;
	pcfg->flags = pixmp->flags;
	pcfg->quantization = pixmp->quantization;
	pcfg->xfer_func = pixmp->xfer_func;
	pcfg->ycbcr_enc = pixmp->ycbcr_enc;
	enc_setvui(fmt, &pcfg->encparams);

	v4l2_klog(LOGLVL_CONFIG, "%s type:%d, planes:%d, res:%dx%d, bytesperline:%d, sizeimage:%d,%d,%d\n",
		__func__, fmt->type, pixmp->num_planes, pixmp->width,
		pixmp->height, pixmp->plane_fmt[0].bytesperline,
		pixmp->plane_fmt[0].sizeimage, pixmp->plane_fmt[1].sizeimage,
		pixmp->plane_fmt[2].sizeimage);

	return ret;
}

static void vsiv4l2_convertpixel2MB(
	struct v4l2_rect *src,
	int fmt,
	s32 *left,
	s32 *top,
	s32 *right,
	s32 *bottom)
{
	int align = (fmt == V4L2_DAEMON_CODEC_ENC_HEVC ? 64:16);
	u32 r, b;

	*left = src->left / align;
	*top = src->top / align;
	r = src->left + src->width;
	b = src->top + src->height;
	*right = (r + align - 1) / align;
	*bottom = (b + align - 1) / align;
}

void vsi_convertROI(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_enc_roi_params *proi = &ctx->mediacfg.roiinfo;
	int fmt = ctx->mediacfg.encparams.general.codecFormat;
	int i, num;
	struct v4l2_daemon_enc_h26x_cmd *penccfg = &ctx->mediacfg.encparams.specific.enc_h26x_cmd;

	if (vsi_v4l2_hwconfig.encformat == 0)
		return;
	num = (vsi_v4l2_hwconfig.enc_isH1 ? VSI_V4L2_MAX_ROI_REGIONS_H1 : VSI_V4L2_MAX_ROI_REGIONS);
	if (proi->num_roi_regions < num)
		num = proi->num_roi_regions;

	for (i = 0; i < num; i++) {
		penccfg->roiAreaEnable[i] = proi->roi_params[i].enable;
		vsiv4l2_convertpixel2MB(&proi->roi_params[i].rect, fmt, &penccfg->roiAreaLeft[i],
			&penccfg->roiAreaTop[i], &penccfg->roiAreaRight[i], &penccfg->roiAreaBottom[i]);
		penccfg->roiDeltaQp[i] = proi->roi_params[i].qp_delta;
		penccfg->roiQp[i] = -1;
	}
	/*disable left ones*/
	for (; i < VSI_V4L2_MAX_ROI_REGIONS; i++) {
		penccfg->roiAreaEnable[i] = penccfg->roiAreaTop[i] = penccfg->roiAreaLeft[i] =
			penccfg->roiAreaBottom[i] = penccfg->roiAreaRight[i] = 0;
	}
}

void vsi_convertIPCM(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_enc_ipcm_params *ipcm = &ctx->mediacfg.ipcminfo;
	int fmt = ctx->mediacfg.encparams.general.codecFormat;
	int i, num;
	struct v4l2_daemon_enc_h26x_cmd *penccfg = &ctx->mediacfg.encparams.specific.enc_h26x_cmd;

	if (vsi_v4l2_hwconfig.encformat == 0 || vsi_v4l2_hwconfig.enc_isH1)
		return;
	num = (ipcm->num_ipcm_regions > VSI_V4L2_MAX_IPCM_REGIONS ?
			VSI_V4L2_MAX_IPCM_REGIONS : ipcm->num_ipcm_regions);
	for (i = 0; i < num; i++) {
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.pcm_loop_filter_disabled_flag = 0;
		penccfg->ipcmAreaEnable[i] = ipcm->ipcm_params[i].enable;
		vsiv4l2_convertpixel2MB(&ipcm->ipcm_params[i].rect, fmt, &penccfg->ipcmAreaLeft[i],
			&penccfg->ipcmAreaTop[i], &penccfg->ipcmAreaRight[i], &penccfg->ipcmAreaBottom[i]);
	}
	/*disable left ones*/
	for (; i < VSI_V4L2_MAX_IPCM_REGIONS; i++) {
		penccfg->ipcmAreaEnable[i] = penccfg->ipcmAreaTop[i] = penccfg->ipcmAreaLeft[i] =
			penccfg->ipcmAreaBottom[i] = penccfg->ipcmAreaRight[i] = 0;
	}
}

int vsiv4l2_setROI(struct vsi_v4l2_ctx *ctx, void *params)
{
	int i;
	struct v4l2_enc_roi_params *proi = (struct v4l2_enc_roi_params *)params;

	ctx->mediacfg.roiinfo = *proi;
	v4l2_klog(LOGLVL_CONFIG, "%s:%d", __func__, proi->num_roi_regions);
	for (i = 0; i < proi->num_roi_regions; i++) {
		v4l2_klog(LOGLVL_CONFIG, "%d:%d:%d:%d:%d:%d", proi->roi_params[i].enable,
			proi->roi_params[i].qp_delta, proi->roi_params[i].rect.left,
			proi->roi_params[i].rect.top, proi->roi_params[i].rect.width, proi->roi_params[i].rect.height);
	}
	return 0;
}

int vsiv4l2_getROIcount(void)
{
	if (vsi_v4l2_hwconfig.encformat == 0)
		return 0;
	if (vsi_v4l2_hwconfig.enc_isH1)
		return VSI_V4L2_MAX_ROI_REGIONS_H1;
	return VSI_V4L2_MAX_ROI_REGIONS;
}

int vsiv4l2_setIPCM(struct vsi_v4l2_ctx *ctx, void *params)
{
	int i;
	struct v4l2_enc_ipcm_params *ipcm = (struct v4l2_enc_ipcm_params *)params;

	ctx->mediacfg.ipcminfo = *ipcm;
	v4l2_klog(LOGLVL_CONFIG, "%s:%d", __func__, ipcm->num_ipcm_regions);
	for (i = 0; i < ipcm->num_ipcm_regions; i++) {
		v4l2_klog(LOGLVL_CONFIG, "ipcm %d:%d:%d:%d:%d", ipcm->ipcm_params[i].enable,
			ipcm->ipcm_params[i].rect.left, ipcm->ipcm_params[i].rect.top,
			ipcm->ipcm_params[i].rect.width, ipcm->ipcm_params[i].rect.height);
	}
	return 0;
}

int vsiv4l2_getIPCMcount(void)
{
	if (vsi_v4l2_hwconfig.encformat == 0 || vsi_v4l2_hwconfig.enc_isH1)
		return 0;
	return VSI_V4L2_MAX_IPCM_REGIONS;
}

static int vsiv4l2_decidepixeldepth(int pixelformat, int origdepth)
{
	switch (pixelformat) {
	case VSI_V4L2_DECOUT_P010:
		return 16;
	case VSI_V4L2_DEC_PIX_FMT_NV12:
	case VSI_V4L2_DEC_PIX_FMT_400:
	case VSI_V4L2_DEC_PIX_FMT_411SP:
	case VSI_V4L2_DEC_PIX_FMT_422SP:
	case VSI_V4L2_DEC_PIX_FMT_444SP:
		return 8;
	case VSI_V4L2_DECOUT_NV12_10BIT:
		return 10;
	case VSI_V4L2_DECOUT_DTRC_10BIT:
	case VSI_V4L2_DECOUT_RFC_10BIT:
	case VSI_V4L2_DECOUT_DTRC:
	case VSI_V4L2_DECOUT_RFC:
	default:
		return origdepth;
	}
}

static int vsiv4l2_setfmt_dec(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	struct v4l2_pix_format pix_ori = fmt->fmt.pix;
	struct vsi_video_fmt *targetfmt;
	int ret = 0;

	ret = vsiv4l2_verifyfmt(ctx, fmt, 1);
	if (ret != 0)
		return ret;
	targetfmt = vsi_find_format(ctx, fmt);
	if (binputqueue(fmt->type)) {
		pcfg->decparams.dec_info.io_buffer.srcwidth = pix->width;
		pcfg->decparams.dec_info.io_buffer.srcheight = pix->height;
		pcfg->decparams.dec_info.io_buffer.inputFormat = targetfmt->dec_fmt;
		pcfg->infmt_fourcc = pix->pixelformat;
	} else {
		pcfg->outfmt_fourcc = pix->pixelformat;
		pcfg->decparams.dec_info.io_buffer.outputPixelDepth =
				vsiv4l2_decidepixeldepth(targetfmt->dec_fmt, pcfg->src_pixeldepth);

		if (!test_bit(CTX_FLAG_SRCCHANGED_BIT, &ctx->flag)) {
			struct v4l2_frmsizeenum fmsize;

			pcfg->decparams.dec_info.io_buffer.output_width = pix->width;
			pcfg->decparams.dec_info.io_buffer.output_height = pix->height;
			pcfg->decparams.dec_info.io_buffer.outBufFormat = targetfmt->dec_fmt;
			pcfg->bytesperline = pix->bytesperline;

			vsi_enum_decfsize(&fmsize, pcfg->infmt_fourcc);
			pcfg->decparams.dec_info.dec_info.visible_rect.left = 0;
			pcfg->decparams.dec_info.dec_info.visible_rect.top = 0;
			pcfg->decparams.dec_info.dec_info.visible_rect.width =
				clamp(pix_ori.width, fmsize.stepwise.min_width, fmsize.stepwise.max_width);
			pcfg->decparams.dec_info.dec_info.visible_rect.height =
				clamp(pix_ori.height, fmsize.stepwise.min_height, fmsize.stepwise.max_height);
		} else {
			//dtrc is only for HEVC and VP9
			if (istiledfmt(targetfmt->dec_fmt)
				&& pcfg->decparams.dec_info.io_buffer.inputFormat != V4L2_DAEMON_CODEC_DEC_VP9
				&& pcfg->decparams.dec_info.io_buffer.inputFormat != V4L2_DAEMON_CODEC_DEC_HEVC)
				return -EINVAL;
			if (pcfg->decparams.dec_info.io_buffer.output_width)
				pix->width = pcfg->decparams.dec_info.io_buffer.output_width;
			if (pcfg->decparams.dec_info.io_buffer.output_height)
				pix->height = pcfg->decparams.dec_info.io_buffer.output_height;
			pcfg->decparams.dec_info.io_buffer.outBufFormat = targetfmt->dec_fmt;
			pcfg->bytesperline = pix->width * pcfg->decparams.dec_info.io_buffer.outputPixelDepth / 8;
			pcfg->bytesperline = ALIGN(pcfg->bytesperline, 16);
			pix->bytesperline = pcfg->bytesperline;
		}
	}
	ret = vsiv4l2_verifyfmt(ctx, fmt, 0);

	pcfg->field = pix->field;
	pcfg->colorspace = pix->colorspace;
	pcfg->flags = pix->flags;
	pcfg->quantization = pix->quantization;
	pcfg->xfer_func = pix->xfer_func;
	pcfg->ycbcr_enc = pix->ycbcr_enc;

	v4l2_klog(LOGLVL_CONFIG, "%s type:%d, res:%dx%d, bytesperline:%d, sizeimage:%d\n",
		__func__, fmt->type, pix->width, pix->height, pix->bytesperline,
		pix->sizeimage);
	return ret;
}

int vsiv4l2_setfmt(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	if (isencoder(ctx))
		return vsiv4l2_setfmt_enc(ctx, fmt);
	else
		return vsiv4l2_setfmt_dec(ctx, fmt);
}

static u32 vsiv4l2_default_bytesperline(u32 fourcc, u32 width)
{
	u32 bytesperline = width;

	switch (fourcc) {
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGBX32:
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_RGBA32:
		bytesperline = width * 4;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_BGR565:
	case V4L2_PIX_FMT_RGB555:
		bytesperline = width * 2;
		break;
	default:
		bytesperline = width;
		break;
	}

	return bytesperline;
}

static int vsiv4l2_verifyfmt_enc(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt, int try_only)
{
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
	struct v4l2_pix_format_mplane *pixmp = &fmt->fmt.pix_mp;
	struct v4l2_frmsizeenum fmsize;
	struct vsi_video_fmt *vfmt = NULL;
	int braw;
	u32 bytesperline;
	u32 try_size[VB2_MAX_PLANES];
	u32 *psize = (binputqueue(fmt->type) ? pcfg->sizeimagesrc : pcfg->sizeimagedst);
	int i;

	if (try_only)
		psize = try_size;

	if (!isvalidtype(fmt->type, ctx->flag)) {
		if (V4L2_TYPE_IS_OUTPUT(fmt->type))
			fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		else
			fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	}

	braw = brawfmt(ctx->flag, fmt->type);
	vfmt = vsi_find_format(ctx, fmt);
	if (!vfmt) {
		vfmt = vsi_enum_encformat(0, braw);
		if (!vfmt)
			return -EINVAL;
		pixmp->pixelformat = vfmt->fourcc;
	}

	pixmp->num_planes = vfmt->num_planes ? vfmt->num_planes : 1;
	if (binputqueue(fmt->type))
		fmsize.pixel_format = pcfg->outfmt_fourcc;
	else
		fmsize.pixel_format = pixmp->pixelformat;
	vsi_enum_encfsize(&fmsize, fmsize.pixel_format);

	pixmp->width = clamp(pixmp->width, fmsize.stepwise.min_width, fmsize.stepwise.max_width);
	pixmp->height = clamp(pixmp->height, fmsize.stepwise.min_height, fmsize.stepwise.max_height);

	if (vsi_v4l2_hwconfig.enc_isH1) {
		if (pixmp->width & 0x3)
			pixmp->width = ALIGN(pixmp->width, 4);
	} else {
		if (pixmp->width & 0x1)
			pixmp->width = ALIGN(pixmp->width, 2);
	}
	if (pixmp->height & 0x1)
		pixmp->height = ALIGN(pixmp->height, 2);

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	bytesperline = max_t(int, pixmp->plane_fmt[0].bytesperline,
			     vsiv4l2_default_bytesperline(pixmp->pixelformat, pixmp->width));
	if (braw) {
		bytesperline = vsiv4l2_enc_getalign(vfmt->enc_fmt,
			pcfg->encparams.general.codecFormat, bytesperline);
	}

	if (pixmp->pixelformat == V4L2_PIX_FMT_YUV420M || pixmp->pixelformat == V4L2_PIX_FMT_YUV420) {
		pixmp->plane_fmt[0].bytesperline = bytesperline;
		pixmp->plane_fmt[1].bytesperline = bytesperline / 2;
		pixmp->plane_fmt[2].bytesperline = bytesperline / 2;
	} else {
		for (i = 0; i < pixmp->num_planes; i++)
			pixmp->plane_fmt[i].bytesperline = bytesperline;
	}

	for (i = 0; i < pixmp->num_planes; i++)
		psize[i] = pixmp->plane_fmt[i].sizeimage;
	verifyPlanesize(psize, braw, pixmp->pixelformat, bytesperline, pixmp->height, 0);
	for (i = 0; i < pixmp->num_planes; i++)
		pixmp->plane_fmt[i].sizeimage = get_plane_size(psize, i, pixmp->num_planes);

	return 0;
}

static int vsiv4l2_verifyfmt_dec(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt, int try_only)
{
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	struct v4l2_frmsizeenum fmsize;
	struct vsi_video_fmt *vfmt = NULL;
	u32 bytesperline;
	int braw;
	u32 try_size[VB2_MAX_PLANES];
	u32 *psize = (binputqueue(fmt->type) ? pcfg->sizeimagesrc : pcfg->sizeimagedst);

	if (try_only)
		psize = try_size;

	if (!isvalidtype(fmt->type, ctx->flag)) {
		if (V4L2_TYPE_IS_OUTPUT(fmt->type))
			fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		else
			fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}

	braw = brawfmt(ctx->flag, fmt->type);
	vfmt = vsi_find_format(ctx, fmt);
	if (!vfmt) {
		vfmt = vsi_enum_dec_format(0, braw, ctx);
		if (!vfmt)
			return -EINVAL;
		pix->pixelformat = vfmt->fourcc;
	}
	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;

	if (binputqueue(fmt->type))
		fmsize.pixel_format = pix->pixelformat;
	else
		fmsize.pixel_format = pcfg->infmt_fourcc;

	vsi_enum_decfsize(&fmsize, fmsize.pixel_format);
	if (V4L2_TYPE_IS_CAPTURE(fmt->type)) {
		pix->width = ALIGN(pix->width, fmsize.stepwise.step_width);
		pix->height = ALIGN(pix->height, fmsize.stepwise.step_height);
	}
	pix->width = clamp(pix->width, fmsize.stepwise.min_width, fmsize.stepwise.max_width);
	pix->height = clamp(pix->height, fmsize.stepwise.min_height, fmsize.stepwise.max_height);

	if (braw) {
		if (!test_bit(CTX_FLAG_SRCCHANGED_BIT, &ctx->flag)) {
			bytesperline = ALIGN(vsiv4l2_default_bytesperline(pix->pixelformat, pix->width), 16);
		} else {
			int outputPixelDepth = vsiv4l2_decidepixeldepth(vfmt->dec_fmt, pcfg->src_pixeldepth);

			bytesperline = ALIGN(pix->width * outputPixelDepth / 8, 16);
		}
	} else {
		bytesperline = vsiv4l2_default_bytesperline(pix->pixelformat, pix->width);
	}
	pix->bytesperline = bytesperline;

	psize[0] = pix->sizeimage;
	verifyPlanesize(psize, braw, pix->pixelformat, bytesperline, pix->height, 1);
	pix->sizeimage = get_plane_size(psize, 0, 1);

	return 0;
}

int vsiv4l2_verifyfmt(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt, int try_only)
{
	if (isencoder(ctx))
		return vsiv4l2_verifyfmt_enc(ctx, fmt, try_only);
	else
		return vsiv4l2_verifyfmt_dec(ctx, fmt, try_only);
}

int vsiv4l2_verifycrop(struct v4l2_selection *s)
{
	int update = 0, align, mask;
	struct v4l2_rect rect = s->r;
	int right = rect.left + rect.width;
	int bottom = rect.top + rect.height;

	if (vsi_v4l2_hwconfig.enc_isH1) {
		mask = 3;
		align = 4;
	} else {
		mask = 1;
		align = 2;
	}
	if ((rect.left & mask) || (right & mask) || (rect.top & 0x1) || (bottom & 0x1))
		update = 1;
	if (!update)
		return 0;
	if (s->flags == (V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE))
		return -ERANGE;
	if (s->flags & V4L2_SEL_FLAG_LE) {
		rect.left = ALIGN(rect.left, align);
		if (right & mask)
			right = ALIGN(right, align) - align;
		rect.top = ALIGN(rect.top, 2);
		if (bottom & 0x1)
			bottom = ALIGN(bottom, 2) - 2;
		rect.width = right - rect.left;
		rect.height = bottom - rect.top;
		s->r = rect;
		return 0;
	}
	/*enlarge is previleged*/
	if (rect.left & mask)
		rect.left = ALIGN(rect.left, align) - align;
	right = ALIGN(right, align);
	if (rect.top & 2)
		rect.top = ALIGN(rect.top, 2) - 2;
	bottom = ALIGN(bottom, 2);
	rect.width = right - rect.left;
	rect.height = bottom - rect.top;
	s->r = rect;
	v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d:%d:%d", __func__,
		rect.top, rect.left, rect.width, rect.height);
	return 0;
}

static u32 find_local_dec_format(s32 fmt, int braw)
{
	int i;

	if (braw) {
		for (i = 0; i < ARRAY_SIZE(vsi_raw_fmt); i++) {
			if (vsi_raw_fmt[i].dec_fmt == fmt)
				return vsi_raw_fmt[i].fourcc;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(vsi_coded_fmt); i++) {
			if (vsi_coded_fmt[i].dec_fmt == fmt)
				return vsi_coded_fmt[i].fourcc;
		}
	}
	return -1;
}

static int vsiv4l2_getfmt_enc(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
	struct v4l2_pix_format_mplane *pixmp = &fmt->fmt.pix_mp;
	int i;
	int *psize = (binputqueue(fmt->type) ? pcfg->sizeimagesrc : pcfg->sizeimagedst);

	if (binputqueue(fmt->type)) {
		pixmp->width = pcfg->width_src;
		pixmp->height = pcfg->height_src;
		pixmp->pixelformat = pcfg->infmt_fourcc;
	} else {
		pixmp->width = pcfg->encparams.general.width;
		pixmp->height = pcfg->encparams.general.height;
		pixmp->pixelformat = pcfg->outfmt_fourcc;
	}
	pixmp->field = pcfg->field;
	if (binputqueue(fmt->type))
		pixmp->num_planes = pcfg->srcplanes;
	else
		pixmp->num_planes = pcfg->dstplanes;
	if (pixmp->num_planes == 0)
		pixmp->num_planes = 1;
	for (i = 0; i < pixmp->num_planes; i++)
		pixmp->plane_fmt[i].sizeimage = get_plane_size(psize, i, pixmp->num_planes);
	if (pixmp->pixelformat == V4L2_PIX_FMT_YUV420M || pixmp->pixelformat == V4L2_PIX_FMT_YUV420) {
		pixmp->plane_fmt[0].bytesperline = pcfg->bytesperline;
		pixmp->plane_fmt[1].bytesperline =
			pixmp->plane_fmt[2].bytesperline = pcfg->bytesperline/2;
		i = 3;
	} else {
		for (i = 0; i < pixmp->num_planes; i++)
			pixmp->plane_fmt[i].bytesperline = pcfg->bytesperline;
	}

	for (; i < VIDEO_MAX_PLANES; i++) {
		pixmp->plane_fmt[i].bytesperline = 0;
		pixmp->plane_fmt[i].sizeimage = 0;
	}

	pixmp->colorspace = pcfg->colorspace;
	pixmp->flags = pcfg->flags;
	pixmp->quantization = pcfg->quantization;
	pixmp->xfer_func = pcfg->xfer_func;
	pixmp->ycbcr_enc = pcfg->ycbcr_enc;

	v4l2_klog(LOGLVL_CONFIG, "%s type:%d, planes:%d, res:%dx%d, bytesperline:%d, sizeimage:%d,%d,%d\n",
		__func__, fmt->type, pixmp->num_planes, pixmp->width,
		pixmp->height, pixmp->plane_fmt[0].bytesperline,
		pixmp->plane_fmt[0].sizeimage, pixmp->plane_fmt[1].sizeimage,
		pixmp->plane_fmt[2].sizeimage);
	return 0;
}

static int vsiv4l2_getfmt_dec(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	int *psize = (binputqueue(fmt->type) ? pcfg->sizeimagesrc : pcfg->sizeimagedst);

	if (binputqueue(fmt->type)) {
		pix->width = pcfg->decparams.dec_info.io_buffer.srcwidth;
		pix->height = pcfg->decparams.dec_info.io_buffer.srcheight;
		pix->pixelformat = pcfg->infmt_fourcc;
		pix->bytesperline = pix->width;
	} else {
		pix->width = pcfg->decparams.dec_info.io_buffer.output_width;
		pix->height = pcfg->decparams.dec_info.io_buffer.output_height;
		pix->bytesperline = pcfg->bytesperline;
		pix->pixelformat = pcfg->outfmt_fourcc;
	}
	pix->field = pcfg->field;
	pix->sizeimage = get_plane_size(psize, 0, 1);
	pix->flags = pcfg->flags;

	vsi_dec_getvui(ctx, fmt);

	v4l2_klog(LOGLVL_CONFIG, "%s type:%d, res:%dx%d, bytesperline:%d, sizeimage:%d\n",
		__func__, fmt->type, pix->width, pix->height, pix->bytesperline,
		pix->sizeimage);
	return 0;
}

int vsiv4l2_getfmt(struct vsi_v4l2_ctx *ctx, struct v4l2_format *fmt)
{
	if (isencoder(ctx))
		return vsiv4l2_getfmt_enc(ctx, fmt);
	else
		return vsiv4l2_getfmt_dec(ctx, fmt);
}

void vsi_v4l2_update_decfmt(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_format fmt;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vsiv4l2_getfmt(ctx, &fmt);

	if (ctx->mediacfg.decparams.dec_info.dec_info.bit_depth == 10) {
		if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_NV12X &&
			fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_P010 &&
			fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_TILEX &&
			fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RFCX) {
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12X;
		}
	}
	if (isJpegOnlyFmt(ctx->mediacfg.decparams.dec_info.dec_info.src_pix_fmt)) {
		fmt.fmt.pix.pixelformat = find_local_dec_format(ctx->mediacfg.decparams.dec_info.dec_info.src_pix_fmt, 1);
	}
	vsiv4l2_setfmt(ctx, &fmt);
}

int vsiv4l2_buffer_config(
	struct vsi_v4l2_ctx *ctx,
	struct vb2_queue *vq,
	unsigned int *nbuffers,
	unsigned int *nplanes,
	unsigned int sizes[]
)
{
	int i, planes;
	int *psize;
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;

	if (binputqueue(vq->type)) {
		planes = pcfg->srcplanes;
		psize = pcfg->sizeimagesrc;
	} else {
		planes = pcfg->dstplanes;
		psize = pcfg->sizeimagedst;
	}
	if (*nplanes) {
		if (*nplanes != planes)
			return -EINVAL;
		for (i = 0; i < planes; i++) {
			if (sizes[i] < psize[i])
				return -EINVAL;
		}
	}
	*nplanes = planes;
	for (i = 0; i < planes; i++)
		sizes[i] = get_plane_size(psize, i, planes);

	if (isencoder(ctx)) {
		if (*nbuffers + vq->num_buffers < ctx->mediacfg.encparams.specific.enc_h26x_cmd.gopSize)
			*nbuffers = ctx->mediacfg.encparams.specific.enc_h26x_cmd.gopSize - vq->num_buffers;
	} else {
		if (!binputqueue(vq->type) && *nbuffers + vq->num_buffers < ctx->mediacfg.minbuf_4capture)
			*nbuffers = ctx->mediacfg.minbuf_4capture - vq->num_buffers;
	}

	v4l2_klog(LOGLVL_BRIEF, "%llx:%d::%s:%d:%d:%d:%d:%d", ctx->ctxid, vq->type, __func__,
		*nbuffers, *nplanes, sizes[0], sizes[1], sizes[2]);
	return 0;
}

void vsiv4l2_set_hwinfo(struct vsi_v4l2_dev_info *hwinfo)
{
	int i, j;

	vsi_v4l2_hwconfig = *hwinfo;
	v4l2_klog(LOGLVL_BRIEF, "%s::%d:%d:%llx:%llx", __func__,
		hwinfo->enc_isH1, hwinfo->max_dec_resolution, hwinfo->encformat, hwinfo->decformat);
	for (i = 0; i < ARRAY_SIZE(vsi_coded_fmt); i++) {
		if ((vsi_coded_fmt[i].mask & hwinfo->encformat) == 0)
			vsi_coded_fmt[i].enc_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE;
		if ((vsi_coded_fmt[i].mask & hwinfo->decformat) == 0) {
			//disable all jpg only output fmt
			if (vsi_coded_fmt[i].dec_fmt == V4L2_DAEMON_CODEC_DEC_JPEG) {
				for (j = 0; j < ARRAY_SIZE(vsi_raw_fmt); j++) {
					if (isJpegOnlyFmt(vsi_raw_fmt[j].dec_fmt))
						vsi_raw_fmt[j].dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE;
				}
			}
			vsi_coded_fmt[i].dec_fmt = V4L2_DAEMON_CODEC_UNKNOW_TYPE;
		}
	}
}

struct vsi_v4l2_dev_info *vsiv4l2_get_hwinfo(void)
{
	return &vsi_v4l2_hwconfig;
}

void vsi_v4l2_update_ctrlcfg(struct v4l2_ctrl_config *cfg)
{
	switch (cfg->id) {
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		if (vsi_v4l2_hwconfig.max_dec_resolution > 1920)
			cfg->max = V4L2_MPEG_VIDEO_H264_LEVEL_5_2;
		else
			cfg->max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		if (vsi_v4l2_hwconfig.max_dec_resolution > 1920)
			cfg->max = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2;
		else
			cfg->max = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1;
		break;
	default:
		break;
	}
}


