/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2017 NXP
 *
 ******************************************************************************
 *
 * This file was auto-generated. Do not edit it manually.
 *
 ******************************************************************************
 *
 * hdcp2.h
 *
 ******************************************************************************
 */

#ifndef HDCP2_H
#define HDCP2_H
#include "mailBox.h"
/* HDCP 2 registers
 * and general use function for HDCP2 (transmitter and receiver)
 * Author - yehonatan levin - cadence */
/**
 *  \file
 *  \brief HDCP 2 registers
 * and general use function for HDCP2 (trasmiter and reciever)
 */

/* HDCP2 register list */

#ifdef DP_TX
#define HDCP2_RTX                       0x69000
#define HDCP2_TX_CAPS                   0x69008
#define HDCP2_CERT_RX                   0x6900B
#define HDCP2_RRX                       0x69215
#define HDCP2_RX_CAPS                   0x6921D
#define HDCP2_EKPUB_KM                  0x69220
#define HDCP2_EKH_KM_WR                 0x692A0
#define HDCP2_M                         0x692B0
#define HDCP2_H_TAG                     0x692C0
#define HDCP2_EKH_KM_RD                 0x692E0
#define HDCP2_RN                        0x692F0
#define HDCP2_L_TAG                     0x692F8
#define HDCP2_EDKEY_KS                  0x69318
#define HDCP2_RIV                       0x69328
#define HDCP2_RX_INFO                   0x69330
#define HDCP2_SEQ_NUM_V                 0x69332
#define HDCP2_V_TAG                     0x69335
#define HDCP2_RECEIVER_ID_LIST          0x69345
#define HDCP2_V                         0x693E0
#define HDCP2_SEQ_NUM_M                 0x693F0
#define HDCP2_K                         0x693F3
#define HDCP2_STREAM_ID_TYPE            0x693F5
#define HDCP2_M_TAG                     0x69473
#define HDCP2_RXSTATUS                  0x69493
#define HDCP2_RSVD                      0x69494
#define HDCP2_DBG                       0x69518

#else /* HDMI */
#define HDCP2_HDCP14        0x0
#define HDCP2_RSVD1         0x44
#define HDCP2_HDCP2_VERSION 0x50
#define HDCP2_RSVD2         0x51
#define HDCP2_WRITE_MESSAGE 0x60
#define HDCP2_RSVD3         0x61
#define HDCP2_RXSTATUS      0x70
#define HDCP2_RSVD4         0x72
#define HDCP2_READ_MESSAGE  0x80
#define HDCP2_RSVD5         0x81
#define HDCP2_DBG           0xC0
#endif

/* HDCP2 commands */
#define HDCP2_CMD_AKE_INIT                          2
#define HDCP2_CMD_AKE_SEND_CERT                     3
#define HDCP2_CMD_AKE_NO_STORED_KM                  4
#define HDCP2_CMD_AKE_STORED_KM                     5
#define HDCP2_CMD_AKE_SEND_H_PRIME                  7
#define HDCP2_CMD_AKE_SEND_PAIRING_INFO             8
#define HDCP2_CMD_LC_INIT                           9
#define HDCP2_CMD_LC_SEND_L_PRIME                   10
#define HDCP2_SKE_SEND_EKS                          11
#define HDCP2_REPEATER_AUTH_SEND_RECEIVER_ID_LIST   12
#define HDCP2_REPEATER_AUTH_SEND_ACK                15
#define HDCP2_REPEATER_AUTH_STREAM_MANAGE           16
#define HDCP2_REPEATER_AUTH_STREAM_READY            17

/* values */
#define HDCP2_VAL_HDCP2_VERSION_SUPPORTED 2

#define LC_128_LEN 16
extern u8 pHdcpLc128[LC_128_LEN];

typedef enum {
	HDCP2_NOT_FINISHED = 0x11,
	HDCP2_FINISHED
} HDCP_2_REC_RES;

/* command structs */
/* AKE INIT */
typedef struct {
	u8 version;
	u8 transmitter_capability_mask[2];
} S_HDCP2_TXCAPS;

typedef struct {
	u8 rtx[8];
	S_HDCP2_TXCAPS txcaps;
} S_HDCP2_CMD_AKE_INIT;

/* AKE_SEND_CERT */

typedef struct {
	u8 cert_rx[522];
	u8 r_rx[8];
	u8 rxcaps[3];
} S_HDCP2_CMD_AKE_SEND_CERT;

/* AKE_NO_STORED_KM */

typedef struct {
	u8 ekpub_km[128];
} S_HDCP2_CMD_AKE_NO_STORED_KM;

/* AKE_STORED_KM */

typedef struct {
	u8 ekh_km[16];
	u8 m[16];
} S_HDCP2_CMD_AKE_STORED_KM;

/* AKE_SEND_H_PRIME */

typedef struct {
	u8 h[32];
} S_HDCP2_CMD_AKE_SEND_H_PRIME;

/* AKE_SEND_PAIRING_INFO */

typedef struct {
	u8 Ekh_Km[16];
} S_HDCP2_CMD_AKE_SEND_PAIRING_INFO;

/* LC_Init */

typedef struct {
	u8 rn[8];
} S_HDCP2_CMD_LC_Init;

/* LC_Send_L_Prime */

typedef struct {
	u8 l[32];
} S_HDCP2_CMD_LC_Send_L_Prime;

/* LC_Send_Eks */

typedef struct {
	u8 Edkey_Ks[16];
	u8 Riv[8];
} S_HDCP2_CMD_SKE_Send_Eks;

/* REPEATER_AUTH_SEND_RECEIVER_ID_LIST */

typedef struct {
	u8 RxInfo[2];
	u8 seq_num_V[3];
	u8 V[16];		/* max device count * 5 */
} S_HDCP2_CMD_REPEATER_AUTH_SEND_RECEIVER_ID_LIST;

/* HDCP2_RxInfo bits */
typedef struct {
	u16 HDCP1_DEVICE_DOWNSTREAM:1;
	u16 HDCP2_0_REPEATER_DOWNSTREAM:1;
	u16 MAX_CASCADE_EXCEEDED:1;
	u16 MAX_DEVS_EXCEEDED:1;
	u16 DEVICE_COUNT:5;
	u16 DEPTH:3;
} S_HDCP2_RX_INFO_BITS;

typedef union {
	S_HDCP2_RX_INFO_BITS bits;
	u16 value16Bit;
} U_HDCP2_RX_INFO;

/* REPEATER_AUTH_SEND_ACK */

typedef struct {
	u8 v[16];
} S_HDCP2_CMD_REPEATER_AUTH_SEND_ACK;

/* REPEATER_AUTH_STREAM_MANAGE */

typedef struct {
	u8 seq_num_m[3];
	u8 k[2];
	u8 streamId_Type[2];	/* should be k*2 by spec??? */
} S_HDCP2_CMD_REPEATER_AUTH_STREAM_MANAGE;

/* REPEATER_AUTH_STREAM_READY */

typedef struct {
	u8 m[32];
} S_HDCP2_CMD_REPEATER_AUTH_STREAM_READY;

/* HDCP2_RXSTATUS bits */
#ifdef DP_TX
typedef struct {

	u8 READY:1;
	u8 H_AVAILABLE:1;
	u8 PAIRING_AVAILABLE:1;
	u8 REAUTH_REQ:1;
	u8 LINK_INTEGRITY_FAILURE:1;
	u8 RSVD:3;
} S_HDCP2_RX_STATUS_BITS;
#else
typedef struct {
	u16 Message_Size:10;
	u16 READY:1;
	u16 REAUTH_REQ:1;
	u16 RSVD:4;
} S_HDCP2_RX_STATUS_BITS;

#endif

typedef union {
	S_HDCP2_RX_STATUS_BITS bits;
	u16 value16Bit;
} U_HDCP2_RX_STATUS;

/* HDCP ports mail box messages */
typedef enum {
	HDCP_GENERAL_SET_LC_128 = 0,
	HDCP_SET_SEED,
} HDCP_GENERAL_MSG;

/**
 *  \brief get command length for specific command
 *
 *  \param [in] offset offset of the command
 *  \return Return_Description
 *
 */
u32 hdcp2_commandLen(u32 offset);
/**
 *  \brief message length for specific message
 *
 *  \param [in] msg the message
 *  \return the size of this message
 *
 */
u32 hdcp2_MsgcommandLen(u8 msg);

#endif
