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
 * Copyright 2017-2018 NXP
 *
 ******************************************************************************
 *
 * This file was auto-generated. Do not edit it manually.
 *
 ******************************************************************************
 *
 * opcodes.h
 *
 ******************************************************************************
 */

#ifndef OPCODES_H_
#define OPCODES_H_

#define DP_TX_MAIL_HANDLER_H
#define DP_TX_MAIL_HANDLER_REQUEST_BUFFER_LEN 256
#define DPTX_SET_POWER_MNG              0x00
#define DPTX_SET_HOST_CAPABILITIES      0x01
#define DPTX_GET_EDID                   0x02
#define DPTX_READ_DPCD                  0x03
#define DPTX_WRITE_DPCD                 0x04
#define DPTX_ENABLE_EVENT               0x05
#define DPTX_WRITE_REGISTER             0x06
#define DPTX_READ_REGISTER              0x07
#define DPTX_WRITE_FIELD                0x08
#define DPTX_TRAINING_CONTROL           0x09
#define DPTX_READ_EVENT                 0x0A
#define DPTX_READ_LINK_STAT                0x0B
#define DPTX_SET_VIDEO                    0x0C
#define DPTX_SET_AUDIO                    0x0D
#define DPTX_GET_LAST_AUX_STAUS            0x0E
#define DPTX_SET_LINK_BREAK_POINT        0x0F
#define DPTX_FORCE_LANES                0x10
#define DPTX_HPD_STATE                             0x11
#define DPTX_EDP_RATE_TRAINING          0x12
#define DPTX_DBG_SET                               0xF0
#define DP_TX_OPCODE_READ_I2C_REQUEST              0xA5
#define DP_TX_OPCODE_WRITE_I2C_REQUEST             0xA6
#define DP_TX_OPCODE_MESSAGE_FILTER                0xA7
#define DPTX_EDID_RESP                             0x02
#define DPTX_DPCD_READ_RESP                        0x03
#define DPTX_DPCD_WRITE_RESP                       0x04
#define DPTX_READ_EVENT_RESP                       0x0A
#define DPTX_READ_REGISTER_RESP                    0x07
#define DP_TX_OPCODE_MESSAGE                       0x10
#define DP_TX_OPCODE_READ_I2C_RESPONSE             0x50
#define DP_TX_OPCODE_WRITE_I2C_RESPONSE            0x60
#define DP_TX_OPCODE_LOOPBACK_TEST                 0xFE
#define DP_TX_OPCODE_BIT_TEST                      0xFF
#define DP_TX_EVENT_ENABLE_HPD_BIT                 0x00
#define DP_TX_EVENT_ENABLE_TRAINING_BIT            0x01
#define DP_TX_EVENT_CODE_HPD_HIGH                  0x01
#define DP_TX_EVENT_CODE_HPD_LOW                   0x02
#define DP_TX_EVENT_CODE_HPD_PULSE                 0x04
#define DP_TX_EVENT_CODE_HPD_STATE_HIGH            0x08
#define DP_TX_EVENT_CODE_HPD_STATE_LOW             0x00
#define DP_TX_EVENT_CODE_TRAINING_FULL_STARTED     0x01
#define DP_TX_EVENT_CODE_TRAINING_FAST_STARTED     0x02
#define DP_TX_EVENT_CODE_TRAINING_FINISHED_CR      0x04
#define DP_TX_EVENT_CODE_TRAINING_FINISHED_EQ      0x08
#define DP_TX_EVENT_CODE_TRAINING_FINISHED_FAST    0x10
#define DP_TX_EVENT_CODE_TRAINING_FAILED_CR        0x20
#define DP_TX_EVENT_CODE_TRAINING_FAILED_EQ        0x40
#define DP_TX_EVENT_CODE_TRAINING_FAILED_FAST      0x80
#define MB_MODULE_ID_DP_TX                         0x01
#define MB_MODULE_ID_DP_RX                         0x02
#define MB_MODULE_ID_HDMI_TX                       0x03
#define MB_MODULE_ID_HDMI_RX                       0x04
#define MB_MODULE_ID_MHL_TX                        0x05
#define MB_MODULE_ID_MHL_RX                        0x06
#define MB_MODULE_ID_HDCP_TX                       0x07
#define MB_MODULE_ID_HDCP_RX                       0x08
#define MB_MODULE_ID_HDCP_GENERAL                  0x09
#define MB_MODULE_ID_GENERAL                       0x0A
#define MB_MODULE_ID                               1

#endif
