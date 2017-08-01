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
 * hdcp_tran.h
 *
 ******************************************************************************
 */

#ifndef HDCP_TRAN_H
#define HDCP_TRAN_H

#include "hdcp.h"

/**
 *  \file
 *  \brief general HDCP2 transmitter function and data structures
 */

/* supported HDCP TX ports */
typedef enum {
	HDCP_TX_PORT_0,
	HDCP_TX_PORT_1,
	HDCP_TX_PORT_2,
	HDCP_TX_PORT_3,
	HDCP_TX_NUM_OF_PORTS,
} HDCP_TX_PORT;

/* HDCP TX ports working mode (HDCP 2.2 or 1.4) */
typedef enum {
	HDCP_TX_2,		/* lock only with HDCP2 */
	HDCP_TX_1,		/* lock only with HDCP1 */
	HDCP_TX_BOTH,   /* lock on HDCP2 or 1 depend on other side */
} HDCP_TX_MODE;

/* HDCP TX ports stream type (relevant if receiver is repeater) */
typedef enum {
	HDCP_CONTENT_TYPE_0,	/* May be transmitted by The HDCP Repeater to all HDCP Devices. */
	HDCP_CONTENT_TYPE_1,	/* Must not be transmitted by the HDCP Repeater to HDCP 1.x-compliant Devices and HDCP 2.0-compliant Repeaters */
} HDCP_TX_CONTENT_STREAM_TYPE;

/* HDCP TX ports stream packet status */
typedef enum {
	HDCP_CONTENT_NEED_TO_SEND,
	HDCP_CONTENT_SENT_AND_WAIT_FOR_RESPOND,
	HDCP_CONTENT_SENT_AND_FAILED,
	HDCP_CONTENT_SUCCESS,
} HDCP_TX_CONTENT_STREAM_STATUS;

/* HDCP TX ports working mode (HDCP 2.2 or 1.4) */
typedef enum {
	HDCP_TX_NOT_ACTIVE,
	HDCP_TX_NOT_CONFIGURED,
	HDCP_TX_START,
	HDCP_TX_WAIT_FOR_RX_TYPE,
	HDCP_TX_ACTION
} HDCP_TX_STATE;

typedef union {
	struct {
		HDCP_TX_MODE port_supported_modes:2;
		HDCP_TX_MODE port_cur_mode:2;
		HDCP_TX_STATE port_state:4;
		HDCP_TX_CONTENT_STREAM_TYPE contentType:1;
		HDCP_TX_CONTENT_STREAM_STATUS content_status:2;
		unsigned char statusWasUpdated:1;
		unsigned char errorWasUpdated:1;
		unsigned char ENABLE_1_1_FEATURES:1;
		unsigned char ENABLE_1_1_FEATURES_onCurrentConnection:1;
		unsigned char hdmi_mode:1;
		unsigned char irq:1;
	} fields;
	unsigned int bits;
} U_HDCP_TRANS_PORT_DATA_STATUS;

/* struct holding data needed to the HDCP transmitter */
#define TX_PORT_STATUS_EXTRA_DATA 3
typedef struct {

	U_HDCP_TRANS_PORT_DATA_STATUS status;
	unsigned short port_status;
	unsigned char port_status_extraData[TX_PORT_STATUS_EXTRA_DATA];
	unsigned int seq_num_M;	/* for sending content stream manage */
	unsigned int seq_num_V;
	unsigned char rxTxBuffer[RX_TX_HDCP_TRANS_MAX_BUFFER];

	unsigned char recieverIdListCommand[(128 * 5) + 4];
	unsigned int recieverIdListSize;
	MB_TYPE mailBoxType;

} S_HDCP_TRANS_PORT_DATA;

typedef struct {
	S_HDCP_TRANS_PORT_DATA port[HDCP_TX_NUM_OF_PORTS];
} S_HDCP_TRANS_DATA;

/* HDCP TX ports working mode (HDCP 2.2 or 1.4) */

/**
 *
 *  \brief transmitter supported API (via mail box)
 */
typedef enum {
	HDCP_TX_CONFIGURATION,	/*!< use this command to set HDCP transmitter type and wake it up (or stop ), 1 byte with following bits :  Bit (0-1)= 0 - support only HDCP 2, 1 - support only HDCP 1, 2 - support both HDCP, Bit 2 - active (to activate port set to 1), to stop port set to 0 */
	HDCP2_TX_SET_PUBLIC_KEY_PARAMS,	/*!< use it to set public key for the HDCP2.x transmitter(HDCP2.x), Modulus n - 384 bytes,  E - 3 bytes */
	HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS,	/*!< use this command to enforce  the random parameters (for debug only!), instead of the random data generated by the embedded RNG.Use this command after HDCP_TX_SET_PUBLIC_KEY_PARAMS command. Relevant to (HDCP2.x), data is : KM 16 bytes,RN 8 bytes,KS 16 bytes,RIV 8 bytes,RTX 8 bytes */
	HDCP2_TX_RESPOND_KM,	/*!< If km is stored, return all parameters, else there is no extra data(HDCP2.x), data is : Receiver ID (5 bytes),m (16 bytes),Km (16 bytes),Ekh(Km)(16 bytes) */
	HDCP1_TX_SEND_KEYS,	/*!< send keys needed for HDCP 1, data is :  AKSV (5 bytes), ksv (7*40 = 280 bytes) */
	HDCP1_TX_SEND_RANDOM_AN,	/*!< set AN, use it for debug purpose, if not use, it will be random number, data is (8 bytes) */
	HDCP_TX_STATUS_CHANGE,	/*!< Will be called in port status change event by cadence HDCP IP, Status for the port:Bit 0 - AUTHENTICATED (1 - link is authenticated), Bit 1 - receiver is REPEATER (1 for repeater, 0 not),Bit 2 - 0 for HDCP1, 1 for HDCP2,     */
	HDCP2_TX_IS_KM_STORED,	/*!< controller check if KM is stored by host(HDCP2.x), data is : Receiver ID (5 bytes) */
	HDCP2_TX_STORE_KM,	/*!< controller ask host to store KM, host may store it on non-volatile memory for faster authentication(HDCP2.x), data is : Receiver ID (5 bytes),m (16 bytes),Km(16 bytes),Ekh(Km),(16 bytes) */
	HDCP_TX_IS_RECEIVER_ID_VALID,	/*!< controller check if receivers ID are not in revocation list, input is->first byte for number of receivers, then list of receivers (5 bytes each) */
	HDCP_TX_RESPOND_RECEIVER_ID_VALID,	/*!< If receivers ID are valid return 1, otherwise return 0, same for HDCP1,HDCP2 */
	HDCP_TX_TEST_KEYS,	/*!< compare HDCP keys with facsimile key */
	HDCP2_TX_SET_KM_KEY_PARAMS,	/*!< This Command is used to load customer defined Key for km-key encryption into the HDCP2.x transmitter controller. */
} HDCP_TRNAS_MAIL_BOX_MSG;
	  /** @} *///

/* HDCP_TX_CONFIGURATION */

/* bits 0-1 HDCP_TX_MODE */
#define HDCP_TX_CONFIGURATION_MODE_OFFSET                       0
#define HDCP_TX_CONFIGURATION_MODE_LEN                          2
#define HDCP_TX_CONFIGURATION_RUN                               2
#define HDCP_TX_CONFIGURATION_RPTR_CONTENT_STREAM_MNGR_OFFSET   3
#define HDCP_TX_CONFIGURATION_RPTR                              4
#define HDCP_TX_ENABLE_1_1_FEATURES                             5
#define HDCP_TX_SECOND_LINK                                     6
#define HDCP_TX_HDMI_MODE                                       7

#define HDCP_TX_CONFIGURATION_ENABLE_KM_KEY_MASK (1 << 4)

/* HDCP_TX_STATUS_CHANGE bits */
#define HDCP_STATUS_AUTHENTICATED           0
#define HDCP_STATUS_REPEATER                1
#define HDCP_STATUS_HDCP2                   2
#define HDCP_STATUS_STRAM_MANAGE_SUCCESS    4
#define HDCP_STATUS_ERROR_TYPE              5
#define HDCP_STATUS_1_1_FEATURES            9

/**
 *
 *  \brief different error types for HDCP_TX_STATUS_CHANGE
 */
typedef enum {
	HDCP_TRAN_ERR_No_error,
	HDCP_TRAN_ERR_HPD_is_down,
	HDCP_TRAN_ERR_SRM_failure,
	HDCP_TRAN_ERR_signature_verification,
	HDCP_TRAN_ERR_h_tag_diff_h,
	HDCP_TRAN_ERR_v_tag_diff_v,
	HDCP_TRAN_ERR_locality_check,
	HDCP_TRAN_ERR_Ddc,
	HDCP_TRAN_ERR_REAUTH_REQ,
	HDCP_TRAN_ERR_topology,
	HDCP_TRAN_ERR_HDCP_RSVD1,
	HDCP_TRAN_ERR_HDMI_capability,
	HDCP_TRAN_ERR_RI,
	HDCP_TRAN_ERR_watchDog_expired,
} HDCP_TRNAS_ERR_TYPE;

/* HDCP2_TX_SET_PUBLIC_KEY_PARAMS */
#define DLP_MODULUS_N 384
#define DLP_E 3
typedef struct {
	unsigned char N[DLP_MODULUS_N];
	unsigned char E[DLP_E];
} S_HDCP_TRANS_PUBLIC_KEY_PARAMS;

/* HDCP2_TX_SET_KM_KEY_PARAMS */
#define DLP_KM_KEY 16
typedef struct {
	unsigned char KM_KEY[DLP_KM_KEY];
} S_HDCP_TRANS_KM_KEY_PARAMS;

/* HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS */
#define DEBUG_RANDOM_NUMBERS_KM_LEN 16
#define DEBUG_RANDOM_NUMBERS_RN_LEN 8
#define DEBUG_RANDOM_NUMBERS_KS_LEN 16
#define DEBUG_RANDOM_NUMBERS_RIV_LEN 8
#define DEBUG_RANDOM_NUMBERS_RTX_LEN 8

typedef struct {
	unsigned char KM[DEBUG_RANDOM_NUMBERS_KM_LEN];
	unsigned char RN[DEBUG_RANDOM_NUMBERS_RN_LEN];
	unsigned char KS[DEBUG_RANDOM_NUMBERS_KS_LEN];
	unsigned char RIV[DEBUG_RANDOM_NUMBERS_RIV_LEN];
	unsigned char RTX[DEBUG_RANDOM_NUMBERS_RTX_LEN];
} S_HDCP_TRANS_DEBUG_RANDOM_NUMBERS;

#define HDCP_PAIRING_M_LEN 16
#define HDCP_PAIRING_M_EKH 16
#define HDCP_PAIRING_R_ID 5

typedef struct {
	unsigned char Receiver_ID[HDCP_PAIRING_R_ID];
	unsigned char m[HDCP_PAIRING_M_LEN];
	unsigned char KM[DEBUG_RANDOM_NUMBERS_KM_LEN];
	unsigned char EKH[HDCP_PAIRING_M_EKH];
} S_HDCP_TRANS_PAIRING_DATA;

typedef struct {
	unsigned char Receiver_ID[HDCP_PAIRING_R_ID];
} S_HDCP_TRANS_REVOCATION_LIST;

/* HDCP1_TX_SEND_KEYS */
#define AKSV_SIZE (5)
#define HDCPT1_KSV_SIZE (7 * 40)

typedef struct {
	unsigned char AKSV[AKSV_SIZE];
	unsigned char KSV[HDCPT1_KSV_SIZE];
} S_HDCP_TX_MAIL_BOX_CMD_HDCP1_TX_SEND_KEYS;

#define AN_SIZE (8)

/* HDCP2_TX_SESSION_KEY */
#define HDCP_SESSION_KEY_LEN 16

#endif //HDCP_TRAN_H
