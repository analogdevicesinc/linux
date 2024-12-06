/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright 2018-2020 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef _UAPI__LINUX_IMX_VPU_H
#define _UAPI__LINUX_IMX_VPU_H

#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>

/*imx v4l2 controls & extension controls*/

//ctrls & extension ctrls definitions
#define V4L2_CID_NON_FRAME		(V4L2_CID_USER_IMX_BASE)
#define V4L2_CID_DIS_REORDER		(V4L2_CID_USER_IMX_BASE + 1)
#define V4L2_CID_ROI_COUNT		(V4L2_CID_USER_IMX_BASE + 2)
#define V4L2_CID_ROI			(V4L2_CID_USER_IMX_BASE + 3)
#define V4L2_CID_IPCM_COUNT		(V4L2_CID_USER_IMX_BASE + 4)
#define V4L2_CID_IPCM			(V4L2_CID_USER_IMX_BASE + 5)
#define V4L2_CID_HDR10META		(V4L2_CID_USER_IMX_BASE + 6)
#define V4L2_CID_SECUREMODE		(V4L2_CID_USER_IMX_BASE + 7)
#define V4L2_CID_SC_ENABLE		(V4L2_CID_USER_IMX_BASE + 8)

#define V4L2_MAX_ROI_REGIONS		8
struct v4l2_enc_roi_param {
	struct v4l2_rect rect;
	__u32 enable;
	__s32 qp_delta;
	__u32 reserved[2];
};

struct v4l2_enc_roi_params {
	__u32 num_roi_regions;
	struct v4l2_enc_roi_param roi_params[V4L2_MAX_ROI_REGIONS];
	__u32 config_store;
	__u32 reserved[2];
};

#define V4L2_MAX_IPCM_REGIONS		2
struct v4l2_enc_ipcm_param {
	struct v4l2_rect rect;
	__u32 enable;
	__u32 reserved[2];
};
struct v4l2_enc_ipcm_params {
	__u32 num_ipcm_regions;
	struct v4l2_enc_ipcm_param ipcm_params[V4L2_MAX_IPCM_REGIONS];
	__u32 config_store;
	__u32 reserved[2];
};

struct v4l2_hdr10_meta {
	__u32 hasHdr10Meta;
	__u32 redPrimary[2];
	__u32 greenPrimary[2];
	__u32 bluePrimary[2];
	__u32 whitePoint[2];
	__u32 maxMasteringLuminance;
	__u32 minMasteringLuminance;
	__u32 maxContentLightLevel;
	__u32 maxFrameAverageLightLevel;
};

/*imx v4l2 command*/
#define V4L2_DEC_CMD_IMX_BASE		(0x08000000)
#define V4L2_DEC_CMD_RESET		(V4L2_DEC_CMD_IMX_BASE + 1)

/*imx v4l2 event*/
//error happened in dec/enc
#define V4L2_EVENT_CODEC_ERROR		(V4L2_EVENT_PRIVATE_START + 1)
//frame loss in dec/enc
#define V4L2_EVENT_SKIP					(V4L2_EVENT_PRIVATE_START + 2)
//crop area change in dec, not reso change
#define V4L2_EVENT_CROPCHANGE			(V4L2_EVENT_PRIVATE_START + 3)
//some options can't be handled by codec, so might be ignored or updated. But codec could go on.
#define V4L2_EVENT_INVALID_OPTION		(V4L2_EVENT_PRIVATE_START + 4)

/*imx v4l2 warning msg, attached with event V4L2_EVENT_INVALID_OPTION*/
enum {
	UNKONW_WARNING = -1,		//not known warning type
	RIOREGION_NOTALLOW,		//(part of)roi region can not work with media setting and be ignored by enc
	IPCMREGION_NOTALLOW,		//(part of)ipcm region can not work with media setting and be ignored by enc
	LEVEL_UPDATED,				//current level cant't work with media setting and be updated by enc
};

/* imx v4l2 formats */
/*raw formats*/
#define V4L2_PIX_FMT_BGR565		v4l2_fourcc('B', 'G', 'R', 'P') /* 16  BGR-5-6-5     */
#define V4L2_PIX_FMT_NV12X			v4l2_fourcc('N', 'V', 'X', '2') /* Y/CbCr 4:2:0 for 10bit  */
#define V4L2_PIX_FMT_DTRC			v4l2_fourcc('D', 'T', 'R', 'C') /* 8bit tile output, uncompressed */
#define V4L2_PIX_FMT_P010			v4l2_fourcc('P', '0', '1', '0')	/*ms p010, data stored in upper 10 bits of 16 */
#define V4L2_PIX_FMT_TILEX			v4l2_fourcc('D', 'T', 'R', 'X') /* 10 bit tile output, uncompressed */
#define V4L2_PIX_FMT_RFC			v4l2_fourcc('R', 'F', 'C', '0') /* 8bit tile output, with rfc*/
#define V4L2_PIX_FMT_RFCX			v4l2_fourcc('R', 'F', 'C', 'X') /* 10 bit tile output, with rfc */
#define V4L2_PIX_FMT_411SP			v4l2_fourcc('4', '1', 'S', 'P') /* YUV 411 Semi planar */

/*codec format*/
#define V4L2_PIX_FMT_AV1			v4l2_fourcc('A', 'V', '1', '0')	/* av1 */
#define V4L2_PIX_FMT_AVS			v4l2_fourcc('A', 'V', 'S', '0')	/* avs */
/*codec formats*/
#endif	//#ifndef _UAPI__LINUX_IMX_VPU_H

