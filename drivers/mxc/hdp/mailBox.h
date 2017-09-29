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
 * mailBox.h
 *
 ******************************************************************************
 */

#ifndef MAIL_BOX_H
#define MAIL_BOX_H

#define MAIL_BOX_MAX_SIZE 600
#define MAIL_BOX_MAX_TX_SIZE 160

 /**
 *  \file mailBox.h
 *  \brief Implementation mail box communication channel between IP and external host
 */

#define MB_MODULE_ID_DP_TX          0x01
#define MB_MODULE_ID_DP_RX          0x02
#define MB_MODULE_ID_HDMI_TX        0x03
#define MB_MODULE_ID_HDMI_RX        0x04
#define MB_MODULE_ID_MHL_TX         0x05
#define MB_MODULE_ID_MHL_RX         0x06
#define MB_MODULE_ID_HDCP_TX        0x07
#define MB_MODULE_ID_HDCP_RX        0x08
#define MB_MODULE_ID_HDCP_GENERAL   0x09
#define MB_MODULE_ID_GENERAL        0x0A

typedef enum {
	MB_TYPE_REGULAR,
	MB_TYPE_SECURE,
	MB_TYPE_COUNT,
} MB_TYPE;

typedef enum {
	MB_SUCCESS,
	MB_BUSY,
	MB_NO_MEMORY
} MB_RET;

typedef enum {
	MB_TO_HOST,
	MB_TO_CONTROLLER,
} MB_IDX;

typedef enum {
	MB_STATE_EMPTY,
	MB_STATE_WAIT_MODULE_ID,
	MB_STATE_WAIT_SIZE_MSB,
	MB_STATE_WAIT_SIZE_LSB,
	MB_STATE_READ_DATA,
	MB_STATE_MSG_READY,
} MB_RX_STATE;

#define MB_OPCODE_ID 0
#define MB_MODULE_ID 1
#define MB_SIZE_MSB_ID 2
#define MB_SIZE_LSB_ID 3
#define MB_DATA_ID 4

typedef struct {
	MB_RX_STATE rxState;
	u32 rx_data_idx;
	u32 rx_final_msgSize;
	u8 rxBuff[MAIL_BOX_MAX_SIZE];
	u8 txBuff[MAIL_BOX_MAX_TX_SIZE];
	u32 txTotal;
	u32 txCur;
} S_MAIL_BOX_PORT_DATA;

typedef struct {
	S_MAIL_BOX_PORT_DATA portData;
	u8 portsTxBusy;		//bit for each port (0-7)
} S_MAIL_BOX_DATA;

#endif //MAIL_BOX_H
