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

/*!
 * @file insert_startcode.h
 *
 */
#ifndef __INSERT_STARTCODE_H__
#define __INSERT_STARTCODE_H__

#include "vpu_b0.h"
#include "mediasys_types.h"
// Startcode insertion types for VC1
#define VC1_SCODE_NEW_SEQUENCE 0x31
#define VC1_SCODE_NEW_PICTURE 0x32
#define VC1_SCODE_NEW_SLICE 0x33
#define RCV_V2_FRAMESIZE_FLAGS (0xFF000000)
#define RCV_HEADER_LEN          24
#define RCV_CODEC_VERSION       (0x5 << 24) //FOURCC_WMV3_WMV
#define RCV_NUM_FRAMES          0xFF
#define RCV_SET_HDR_EXT     0x80000000
#define VC1_IS_NOT_NAL(id)      ((id & 0x00FFFFFF) != 0x00010000)
#define VC1_MAX_FRM_HEADER_SIZE 32
#define VC1_MAX_SEQ_HEADER_SIZE 256

u_int32 insert_scode_4_pic(struct vpu_ctx *ctx, u_int8 *dst, u_int8 *src, u_int32 vdec_std, u_int32 uPayloadSize);
u_int32 insert_scode_4_seq(struct vpu_ctx *ctx, u_int8 *src, u_int8 *dst, u_int32 vdec_std, u_int32 uPayloadSize);

#endif
