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
 * general_handler.h
 *
 ******************************************************************************
 */

#ifndef GENERAL_HANDLER_H
#define GENERAL_HANDLER_H

/**
 *  \file
 *  \brief general handler, checks available messages, receives it from mailbox, handles requests and sends response to the host
 */
#define DP_TX_MAIL_HANDLER_REQUEST_BUFFER_LEN 256

/**
 *  \brief opcode defines host->controller
 */
#define GENERAL_MAIN_CONTROL            0x01
#define GENERAL_TEST_ECHO               0x02
#define GENERAL_BUS_SETTINGS            0x03
#define GENERAL_TEST_ACCESS             0x04

#define GENERAL_WRITE_REGISTER          0x05
#define GENERAL_WRITE_FIELD             0x06
#define GENERAL_READ_REGISTER           0x07

#define GENERAL_TEST_TRNG_SIMPLE        0xF0

#define GENERAL_MAIN_CONTROL_SET_ACTIVE_BIT         0
#define GENERAL_MAIN_CONTROL_SET_ALT_CIPHER_ADDR    1
#define GENERAL_MAIN_CONTROL_SET_FAST_HDCP_DELAYS   2

#define GENERAL_BUS_SETTINGS_DPCD_BUS_BIT           0
#define GENERAL_BUS_SETTINGS_DPCD_BUS_LOCK_BIT      1
#define GENERAL_BUS_SETTINGS_HDCP_BUS_BIT           2
#define GENERAL_BUS_SETTINGS_HDCP_BUS_LOCK_BIT      3
#define GENERAL_BUS_SETTINGS_CAPB_OWNER_BIT         4
#define GENERAL_BUS_SETTINGS_CAPB_OWNER_LOCK_BIT    5

/**
 *  \brief opcode defines controller->host
 */

#define GENERAL_MAIN_CONTROL_RESP        0x01
#define GENERAL_TEST_ECHO_RESP           0x02
#define GENERAL_BUS_SETTINGS_RESP        0x03

#define GENERAL_READ_REGISTER_RESP       0x07

#define GENERAL_BUS_SETTINGS_RESP_DPCD_BUS_BIT      0
#define GENERAL_BUS_SETTINGS_RESP_HDCP_BUS_BIT      1
#define GENERAL_BUS_SETTINGS_RESP_CAPB_OWNER_BIT    2

#define GENERAL_BUS_SETTINGS_RESP_SUCCESS           0
#define GENERAL_BUS_SETTINGS_RESP_LOCK_ERROR        1

typedef struct {
	unsigned char dpcd_locked;
	unsigned char hdcp_locked;
	unsigned char capb_locked;
	unsigned char active_mode;
} S_GENERAL_HANDLER_DATA;

/**
 *  \brief event id sent to the host
 */
typedef enum {
	EVENT_ID_DPTX_HPD = 0,
	EVENT_ID_HDMI_TX_HPD = 0,
	EVENT_ID_HDMI_RX_5V = 0,

	EVENT_ID_DPTX_TRAINING = 1,
	EVENT_ID_HDMI_RX_SCDC_CHANGE = 1,

	EVENT_ID_RESERVE0 = 2,
	EVENT_ID_RESERVE1 = 3,

	EVENT_ID_HDCPTX_STATUS = 4,
	EVENT_ID_HDCPRX_STATUS = 4,

	EVENT_ID_HDCPTX_IS_KM_STORED = 5,
	EVENT_ID_HDCPTX_STORE_KM = 6,
	EVENT_ID_HDCPTX_IS_RECEIVER_ID_VALID = 7,
	EVENT_ID_HDMITX_READ_REQUEST = 8,
} EVENT_ID;

/**
 * \brief convert bank id and register number to address and write to ptr
 */

#define select_reg_old(bank, reg_no, ptr) \
{ \
	ptr = 0; \
	if ((bank == 0x22) || (bank == 0x20) || (bank == 0x0b) || (bank == 0x09) || (bank == 0x0A)) \
		ptr = (u32 *)(bank << 8 | reg_no); \
}

#define select_reg(bank, reg_no, ptr) \
do { \
	ptr = (u32 *)(bank << 8 | reg_no); \
} while (0)

#define select_reg4(pmsb, p2, p3, plsb, ptr) \
do { \
	ptr = (u32 *)((pmsb << 24) | (p2 << 16) | (p3 << 8) | (plsb << 0)); \
} while (0)

#define EVENTS_DPTX_CNT 2
#define EVENTS_HDCPTX_CNT 4

void general_handler_set_active_mode(void);
void general_handler_set_standby_mode(void);

/**
 *  \brief request sending en event to the host
 *  \param [in] eventId
 *  \param [in] eventCode
 */

#endif /* GENERAL_HANDLER_H */
