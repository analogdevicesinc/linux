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
 * dptx_stream.h
 *
 ******************************************************************************
 */

#ifndef DPTX_STREAM_H_
#define DPTX_STREAM_H_

/* register MSA_HORIZONTAL_0 */
#define MSA_HORIZONTAL_0 32
#define F_PCK_STUFF_HTOTAL(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_PCK_STUFF_HTOTAL_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_PCK_STUFF_HSTART(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_PCK_STUFF_HSTART_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register MSA_HORIZONTAL_1 */
#define MSA_HORIZONTAL_1 33
#define F_PCK_STUFF_HSYNCWIDTH(x) (((x) & ((1 << 15) - 1)) << 0)
#define F_PCK_STUFF_HSYNCWIDTH_RD(x) (((x) & (((1 << 15) - 1) << 0)) >> 0)
#define F_PCK_STUFF_HSYNCPOLARITY(x) (((x) & ((1 << 1) - 1)) << 15)
#define F_PCK_STUFF_HSYNCPOLARITY_RD(x) (((x) & (((1 << 1) - 1) << 15)) >> 15)
#define F_PCK_STUFF_HWIDTH(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_PCK_STUFF_HWIDTH_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register MSA_VERTICAL_0 */
#define MSA_VERTICAL_0 34
#define F_PCK_STUFF_VTOTAL(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_PCK_STUFF_VTOTAL_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_PCK_STUFF_VSTART(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_PCK_STUFF_VSTART_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register MSA_VERTICAL_1 */
#define MSA_VERTICAL_1 35
#define F_PCK_STUFF_VSYNCWIDTH(x) (((x) & ((1 << 15) - 1)) << 0)
#define F_PCK_STUFF_VSYNCWIDTH_RD(x) (((x) & (((1 << 15) - 1) << 0)) >> 0)
#define F_PCK_STUFF_VSYNCPOLARITY(x) (((x) & ((1 << 1) - 1)) << 15)
#define F_PCK_STUFF_VSYNCPOLARITY_RD(x) (((x) & (((1 << 1) - 1) << 15)) >> 15)
#define F_PCK_STUFF_VHEIGHT(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_PCK_STUFF_VHEIGHT_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register MSA_MISC */
#define MSA_MISC 36
#define F_MSA_MISC0(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_MSA_MISC0_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_MSA_MISC1(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_MSA_MISC1_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_MSA_MISC1_INV(x) (((x) & ((1 << 1) - 1)) << 16)
#define F_MSA_MISC1_INV_RD(x) (((x) & (((1 << 1) - 1) << 16)) >> 16)
#define F_MSA_IN_MID_INTERLACE_EN(x) (((x) & ((1 << 1) - 1)) << 17)
#define F_MSA_IN_MID_INTERLACE_EN_RD(x) (((x) & (((1 << 1) - 1) << 17)) >> 17)

/* register STREAM_CONFIG */
#define STREAM_CONFIG 37
#define F_STREAM_NUM(x) (((x) & ((1 << 6) - 1)) << 0)
#define F_STREAM_NUM_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)

/* register AUDIO_PACK_STATUS */
#define AUDIO_PACK_STATUS 38
#define F_AP_FIFO_EMPTY(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_AP_FIFO_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_AP_FIFO_FULL(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_AP_FIFO_FULL_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_AP_AIF_FSM_CURR_ST(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_AP_AIF_FSM_CURR_ST_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_AP_SDP_TRANSFER_FSM_CURR_ST(x) (((x) & ((1 << 3) - 1)) << 3)
#define F_AP_SDP_TRANSFER_FSM_CURR_ST_RD(x) (((x) & (((1 << 3) - 1) << 3)) >> 3)
#define F_AP_FIFO_RD_FSM_CURR_ST(x) (((x) & ((1 << 2) - 1)) << 6)
#define F_AP_FIFO_RD_FSM_CURR_ST_RD(x) (((x) & (((1 << 2) - 1) << 6)) >> 6)
#define F_AP_FIFO_WR_FSM_CURR_ST(x) (((x) & ((1 << 2) - 1)) << 8)
#define F_AP_FIFO_WR_FSM_CURR_ST_RD(x) (((x) & (((1 << 2) - 1) << 8)) >> 8)
#define F_AP_PARITY_FSM_CURRENT_STATE(x) (((x) & ((1 << 3) - 1)) << 10)
#define F_AP_PARITY_FSM_CURRENT_STATE_RD(x) (((x) & (((1 << 3) - 1) << 10)) >> 10)
#define F_AUDIO_TS_VERSION(x) (((x) & ((1 << 6) - 1)) << 16)
#define F_AUDIO_TS_VERSION_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)

/* register VIF_STATUS */
#define VIF_STATUS 39
#define F_VIF_FIFO_EMPTY(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_VIF_FIFO_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_VIF_FIFO_FULL(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_VIF_FIFO_FULL_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_VIF_WR_CTRL_STATE(x) (((x) & ((1 << 6) - 1)) << 2)
#define F_VIF_WR_CTRL_STATE_RD(x) (((x) & (((1 << 6) - 1) << 2)) >> 2)
#define F_VIF_RD_CTRL_STATE(x) (((x) & ((1 << 20) - 1)) << 8)
#define F_VIF_RD_CTRL_STATE_RD(x) (((x) & (((1 << 20) - 1) << 8)) >> 8)

/* register PCK_STUFF_STATUS_0 */
#define PCK_STUFF_STATUS_0 40
#define F_NO_VIDEO_GEN_STATE(x) (((x) & ((1 << 5) - 1)) << 0)
#define F_NO_VIDEO_GEN_STATE_RD(x) (((x) & (((1 << 5) - 1) << 0)) >> 0)
#define F_SST_VIDEO_GEN_STATE(x) (((x) & ((1 << 7) - 1)) << 8)
#define F_SST_VIDEO_GEN_STATE_RD(x) (((x) & (((1 << 7) - 1) << 8)) >> 8)
#define F_MST_VIDEO_GEN_STATE(x) (((x) & ((1 << 6) - 1)) << 16)
#define F_MST_VIDEO_GEN_STATE_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
#define F_MSA_GEN_STATE(x) (((x) & ((1 << 7) - 1)) << 24)
#define F_MSA_GEN_STATE_RD(x) (((x) & (((1 << 7) - 1) << 24)) >> 24)

/* register PCK_STUFF_STATUS_1 */
#define PCK_STUFF_STATUS_1 41
#define F_SST_SS_GEN_STATE(x) (((x) & ((1 << 6) - 1)) << 0)
#define F_SST_SS_GEN_STATE_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
#define F_MST_SS_GEN_STATE(x) (((x) & ((1 << 6) - 1)) << 8)
#define F_MST_SS_GEN_STATE_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)

/* register INFO_PACK_STATUS */
#define INFO_PACK_STATUS 42
#define F_INFO_PACK_FIFO_FULL(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_INFO_PACK_FIFO_FULL_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_INFO_PACK_FIFO_EMPTY(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_INFO_PACK_FIFO_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_IP_PARITY_FSM_CURRENT_STATE(x) (((x) & ((1 << 3) - 1)) << 2)
#define F_IP_PARITY_FSM_CURRENT_STATE_RD(x) (((x) & (((1 << 3) - 1) << 2)) >> 2)
#define F_IP_FIFO_WR_FSM_CURRENT_STATE(x) (((x) & ((1 << 3) - 1)) << 5)
#define F_IP_FIFO_WR_FSM_CURRENT_STATE_RD(x) (((x) & (((1 << 3) - 1) << 5)) >> 5)
#define F_IP_FIFO_RD_FSM_CURRENT_STATE(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_IP_FIFO_RD_FSM_CURRENT_STATE_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_IP_SEND_DATA_FSM_CURRENT_STATE(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_IP_SEND_DATA_FSM_CURRENT_STATE_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_IN_VBID(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_IN_VBID_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register RATE_GOVERNOR_STATUS */
#define RATE_GOVERNOR_STATUS 43
#define F_RATE_GOVERNOR_FSM_STATE(x) (((x) & ((1 << 3) - 1)) << 0)
#define F_RATE_GOVERNOR_FSM_STATE_RD(x) (((x) & (((1 << 3) - 1) << 0)) >> 0)
#define F_CFG_TU_VS_DIFF(x) (((x) & ((1 << 2) - 1)) << 8)
#define F_CFG_TU_VS_DIFF_RD(x) (((x) & (((1 << 2) - 1) << 8)) >> 8)
#define F_CFG_HSYNC_DELAY(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_CFG_HSYNC_DELAY_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
#define F_CFG_EN_HSYNC_DELAY(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_CFG_EN_HSYNC_DELAY_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register DP_HORIZONTAL */
#define DP_HORIZONTAL 44
#define F_HSYNCWIDTH(x) (((x) & ((1 << 15) - 1)) << 0)
#define F_HSYNCWIDTH_RD(x) (((x) & (((1 << 15) - 1) << 0)) >> 0)
#define F_HWIDTH(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_HWIDTH_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register DP_VERTICAL_0 */
#define DP_VERTICAL_0 45
#define F_VHEIGHT(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VHEIGHT_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_VSTART(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_VSTART_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register DP_VERTICAL_1 */
#define DP_VERTICAL_1 46
#define F_VTOTAL(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VTOTAL_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_VTOTAL_EVEN(x) (((x) & ((1 << 1) - 1)) << 16)
#define F_VTOTAL_EVEN_RD(x) (((x) & (((1 << 1) - 1) << 16)) >> 16)

/* register DP_BLOCK_SDP */
#define DP_BLOCK_SDP 47
#define F_BLOCK_SDP_BS(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_BLOCK_SDP_BS_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_BLOCK_SDP_BE(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_BLOCK_SDP_BE_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_NO_VIDEO_BLOCK_SDP(x) (((x) & ((1 << 12) - 1)) << 16)
#define F_NO_VIDEO_BLOCK_SDP_RD(x) (((x) & (((1 << 12) - 1) << 16)) >> 16)

#endif //DPTX_STREAM
