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
 * Copyright 2018 NXP
 *
 ******************************************************************************
 *
 * This file was auto-generated. Do not edit it manually.
 *
 ******************************************************************************
 *
 * sink_mhl_hd.h
 *
 ******************************************************************************
 */

#ifndef SINK_MHL_HD_H_
#define SINK_MHL_HD_H_

/* register TMDS_ALIGN_CTRL */
#define TMDS_ALIGN_CTRL 0
#define F_CHAR_MATCH_CNT_THR(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_CHAR_MATCH_CNT_THR_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHAR_HAM_DIS_TOL(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHAR_HAM_DIS_TOL_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_LOCK_ALIGN(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_LOCK_ALIGN_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_ALIGN_WD_THR(x) (((x) & ((1 << 3) - 1)) << 9)
#define F_ALIGN_WD_THR_RD(x) (((x) & (((1 << 3) - 1) << 9)) >> 9)
#define F_ALIGN_ANY_CHAR(x) (((x) & ((1 << 1) - 1)) << 12)
#define F_ALIGN_ANY_CHAR_RD(x) (((x) & (((1 << 1) - 1) << 12)) >> 12)

/* register TMDS_DEC_CTRL */
#define TMDS_DEC_CTRL 1
#define F_DECODER_ERR_CORR_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_DECODER_ERR_CORR_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_TMDS_DECODER_SW_RST(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_TMDS_DECODER_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_TMDS_FORCE_HDMI_MODE(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_TMDS_FORCE_HDMI_MODE_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_TMDS_SW_HDMI_MODE(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_TMDS_SW_HDMI_MODE_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register TMDS_DEC_ST */
#define TMDS_DEC_ST 2
#define F_DEFRAMER_HDMI_MODE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_DEFRAMER_HDMI_MODE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_CHAR_HAM_ERR(x) (((x) & ((1 << 3) - 1)) << 2)
#define F_CHAR_HAM_ERR_RD(x) (((x) & (((1 << 3) - 1) << 2)) >> 2)
#define F_CHAR_ALIGNED(x) (((x) & ((1 << 3) - 1)) << 5)
#define F_CHAR_ALIGNED_RD(x) (((x) & (((1 << 3) - 1) << 5)) >> 5)
#define F_CTRL_ALIGN_ERR(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_CTRL_ALIGN_ERR_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_CTRL_ALIGNED(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_CTRL_ALIGNED_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)
#define F_TMDS_CHAR_CH0_WIN(x) (((x) & ((1 << 4) - 1)) << 10)
#define F_TMDS_CHAR_CH0_WIN_RD(x) (((x) & (((1 << 4) - 1) << 10)) >> 10)
#define F_TMDS_CHAR_CH1_WIN(x) (((x) & ((1 << 4) - 1)) << 14)
#define F_TMDS_CHAR_CH1_WIN_RD(x) (((x) & (((1 << 4) - 1) << 14)) >> 14)
#define F_TMDS_CHAR_CH2_WIN(x) (((x) & ((1 << 4) - 1)) << 18)
#define F_TMDS_CHAR_CH2_WIN_RD(x) (((x) & (((1 << 4) - 1) << 18)) >> 18)

/* register TMDS_CH0_ERR_CNT */
#define TMDS_CH0_ERR_CNT 3
#define F_DEFRAMER_CH0_ERROR_CNT(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_DEFRAMER_CH0_ERROR_CNT_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register TMDS_CH1_ERR_CNT */
#define TMDS_CH1_ERR_CNT 4
#define F_DEFRAMER_CH1_ERROR_CNT(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_DEFRAMER_CH1_ERROR_CNT_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register TMDS_CH2_ERR_CNT */
#define TMDS_CH2_ERR_CNT 5
#define F_DEFRAMER_CH2_ERROR_CNT(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_DEFRAMER_CH2_ERROR_CNT_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register HDCP_FIFO_STAT */
#define HDCP_FIFO_STAT 6
#define F_HDCP_DOUBLE_FIFO_REMPTY(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_HDCP_DOUBLE_FIFO_REMPTY_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_HDCP_DOUBLE_FIFO_WFULL(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_HDCP_DOUBLE_FIFO_WFULL_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_HDCP_DOUBLE_FIFO_UNDERRUN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_HDCP_DOUBLE_FIFO_UNDERRUN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_HDCP_DOUBLE_FIFO_OVERRUN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_HDCP_DOUBLE_FIFO_OVERRUN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_HDCP22_DATA_FIFO_UNDERRUN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_HDCP22_DATA_FIFO_UNDERRUN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_HDCP22_DATA_FIFO_OVERRUN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_HDCP22_DATA_FIFO_OVERRUN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_HDCP22_DATA_FIFO_REMPTY(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_HDCP22_DATA_FIFO_REMPTY_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_HDCP22_DATA_FIFO_WFULL(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_HDCP22_DATA_FIFO_WFULL_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
#define F_HDCP_DELAY_FIFO_EMPTY(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_HDCP_DELAY_FIFO_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_HDCP_DELAY_FIFO_FULL(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_HDCP_DELAY_FIFO_FULL_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)
#define F_HDCP14_DATA_FIFO_UNDERRUN(x) (((x) & ((1 << 2) - 1)) << 10)
#define F_HDCP14_DATA_FIFO_UNDERRUN_RD(x) (((x) & (((1 << 2) - 1) << 10)) >> 10)
#define F_HDCP14_DATA_FIFO_OVERRUN(x) (((x) & ((1 << 2) - 1)) << 12)
#define F_HDCP14_DATA_FIFO_OVERRUN_RD(x) (((x) & (((1 << 2) - 1) << 12)) >> 12)
#define F_HDCP14_DATA_FIFO_REMPTY(x) (((x) & ((1 << 2) - 1)) << 14)
#define F_HDCP14_DATA_FIFO_REMPTY_RD(x) (((x) & (((1 << 2) - 1) << 14)) >> 14)
#define F_HDCP14_DATA_FIFO_WFULL(x) (((x) & ((1 << 2) - 1)) << 16)
#define F_HDCP14_DATA_FIFO_WFULL_RD(x) (((x) & (((1 << 2) - 1) << 16)) >> 16)

/* register HDCP_DELAY_FIFO_CTRL */
#define HDCP_DELAY_FIFO_CTRL 7
#define F_HDCP_DELAY_FIFO_AFULL_THR(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_HDCP_DELAY_FIFO_AFULL_THR_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_HDCP_DELAY_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_HDCP_DELAY_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_HDCP_DOUBLE_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_HDCP_DOUBLE_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_HDCP14_DATA_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_HDCP14_DATA_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_HDCP_CTRL_SW_RST(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_HDCP_CTRL_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register HDCP_CTRL */
#define HDCP_CTRL 8
#define F_HDCP_ENABLE_1P1_FEATURES(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_HDCP_ENABLE_1P1_FEATURES_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_HDCP_SELECT(x) (((x) & ((1 << 2) - 1)) << 1)
#define F_HDCP_SELECT_RD(x) (((x) & (((1 << 2) - 1) << 1)) >> 1)
#define F_FORCE_VSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_FORCE_VSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_FORCE_HSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_FORCE_HSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SW_VSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SW_VSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_SW_HSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_SW_HSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)

/* register HDCP_STAT */
#define HDCP_STAT 9
#define F_HDCP_ESS_STATE(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_HDCP_ESS_STATE_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_SINK_HD_CIPHER_HSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SINK_HD_CIPHER_HSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SINK_HD_CIPHER_VSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SINK_HD_CIPHER_VSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)

/* register PKT_CTRL */
#define PKT_CTRL 10
#define F_SINK_AVMUTE_SET(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_AVMUTE_SET_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_AVMUTE_CLR(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_AVMUTE_CLR_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_ERR_CORR_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_ERR_CORR_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)

/* register PKT_STAT_0 */
#define PKT_STAT_0 11
#define F_SINK_AVMUTE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_AVMUTE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_ACR_CTS(x) (((x) & ((1 << 20) - 1)) << 1)
#define F_SINK_ACR_CTS_RD(x) (((x) & (((1 << 20) - 1) << 1)) >> 1)
#define F_PKT_DEC_GCP_CD(x) (((x) & ((1 << 4) - 1)) << 21)
#define F_PKT_DEC_GCP_CD_RD(x) (((x) & (((1 << 4) - 1) << 21)) >> 21)
#define F_PKT_DEC_GCP_PP(x) (((x) & ((1 << 4) - 1)) << 25)
#define F_PKT_DEC_GCP_PP_RD(x) (((x) & (((1 << 4) - 1) << 25)) >> 25)

/* register PKT_STAT_1 */
#define PKT_STAT_1 12
#define F_SINK_ACR_N(x) (((x) & ((1 << 20) - 1)) << 0)
#define F_SINK_ACR_N_RD(x) (((x) & (((1 << 20) - 1) << 0)) >> 0)

/* register PKT_ERR_CNT_HEADER */
#define PKT_ERR_CNT_HEADER 13
#define F_PKT_HEADER_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_PKT_HEADER_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_PKT_HEADER_CORR_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_PKT_HEADER_CORR_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)

/* register PKT_ERR_CNT_01 */
#define PKT_ERR_CNT_01 14
#define F_PKT_SUBPKT0_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_PKT_SUBPKT0_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_PKT_SUBPKT0_CORR_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_PKT_SUBPKT0_CORR_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_PKT_SUBPKT1_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_PKT_SUBPKT1_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
#define F_PKT_SUBPKT1_CORR_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_PKT_SUBPKT1_CORR_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register PKT_ERR_CNT_23 */
#define PKT_ERR_CNT_23 15
#define F_PKT_SUBPKT2_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_PKT_SUBPKT2_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_PKT_SUBPKT2_CORR_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_PKT_SUBPKT2_CORR_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_PKT_SUBPKT3_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_PKT_SUBPKT3_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
#define F_PKT_SUBPKT3_CORR_ERR_CNT(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_PKT_SUBPKT3_CORR_ERR_CNT_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register TMDS_SCR_CTRL */
#define TMDS_SCR_CTRL 16
#define F_SCRAMBLER_MODE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SCRAMBLER_MODE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SCRAMBLER_SW_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SCRAMBLER_SW_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SCRAMBLER_CTRL_SW_RST(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SCRAMBLER_CTRL_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)

/* register TMDS_SCR_CNT_INT_CTRL */
#define TMDS_SCR_CNT_INT_CTRL 17
#define F_SCRAMBLER_SSCP_LINE_DET_THR(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_SCRAMBLER_SSCP_LINE_DET_THR_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_SCRAMBLER_CTRL_LINE_DET_THR(x) (((x) & ((1 << 24) - 1)) << 8)
#define F_SCRAMBLER_CTRL_LINE_DET_THR_RD(x) (((x) & (((1 << 24) - 1) << 8)) >> 8)

/* register TMDS_SCR_VALID_CTRL */
#define TMDS_SCR_VALID_CTRL 18
#define F_SCRAMBLER_SSCP_LINE_VALID_THR(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_SCRAMBLER_SSCP_LINE_VALID_THR_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_SCRAMBLER_CTRL_LINE_VALID_THR(x) (((x) & ((1 << 24) - 1)) << 8)
#define F_SCRAMBLER_CTRL_LINE_VALID_THR_RD(x) (((x) & (((1 << 24) - 1) << 8)) >> 8)

/* register TMDS_SCR_CNT_INT_ST */
#define TMDS_SCR_CNT_INT_ST 19
#define F_SCRAMBLER_SSCP_LINE_CNT(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_SCRAMBLER_SSCP_LINE_CNT_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_SCRAMBLER_CTRL_LINE_CNT(x) (((x) & ((1 << 24) - 1)) << 8)
#define F_SCRAMBLER_CTRL_LINE_CNT_RD(x) (((x) & (((1 << 24) - 1) << 8)) >> 8)

/* register TMDS_MHL_HD_INT_MASK */
#define TMDS_MHL_HD_INT_MASK 20
#define F_TMDS_MHL_HD_MASK(x) (((x) & ((1 << 9) - 1)) << 0)
#define F_TMDS_MHL_HD_MASK_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)

/* register TMDS_MHL_HD_INT_STAT */
#define TMDS_MHL_HD_INT_STAT 21
#define F_TMDS_MHL_HD_STATUS(x) (((x) & ((1 << 9) - 1)) << 0)
#define F_TMDS_MHL_HD_STATUS_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT0_CTRL */
#define TMDS_STAT_CNT0_CTRL 22
#define F_STATUS_CNT0_INC_SEL(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_STATUS_CNT0_INC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_STATUS_CNT0_DEC_SEL(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_STATUS_CNT0_DEC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_STATUS_CNT0_CLR_SEL(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_STATUS_CNT0_CLR_SEL_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_STATUS_CNT0_START_SEL(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_STATUS_CNT0_START_SEL_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_STATUS_CNT0_STOP_SEL(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_STATUS_CNT0_STOP_SEL_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)

/* register TMDS_STAT_CNT0_THR */
#define TMDS_STAT_CNT0_THR 23
#define F_STATUS_CNT0_THR(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT0_THR_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT0_VAL */
#define TMDS_STAT_CNT0_VAL 24
#define F_STATUS_CNT0_VAL(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT0_VAL_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT1_CTRL */
#define TMDS_STAT_CNT1_CTRL 25
#define F_STATUS_CNT1_INC_SEL(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_STATUS_CNT1_INC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_STATUS_CNT1_DEC_SEL(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_STATUS_CNT1_DEC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_STATUS_CNT1_CLR_SEL(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_STATUS_CNT1_CLR_SEL_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_STATUS_CNT1_START_SEL(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_STATUS_CNT1_START_SEL_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_STATUS_CNT1_STOP_SEL(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_STATUS_CNT1_STOP_SEL_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)

/* register TMDS_STAT_CNT1_THR */
#define TMDS_STAT_CNT1_THR 26
#define F_STATUS_CNT1_THR(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT1_THR_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT1_VAL */
#define TMDS_STAT_CNT1_VAL 27
#define F_STATUS_CNT1_VAL(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT1_VAL_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT2_CTRL */
#define TMDS_STAT_CNT2_CTRL 28
#define F_STATUS_CNT2_INC_SEL(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_STATUS_CNT2_INC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_STATUS_CNT2_DEC_SEL(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_STATUS_CNT2_DEC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_STATUS_CNT2_CLR_SEL(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_STATUS_CNT2_CLR_SEL_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_STATUS_CNT2_START_SEL(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_STATUS_CNT2_START_SEL_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_STATUS_CNT2_STOP_SEL(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_STATUS_CNT2_STOP_SEL_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)

/* register TMDS_STAT_CNT2_THR */
#define TMDS_STAT_CNT2_THR 29
#define F_STATUS_CNT2_THR(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT2_THR_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT2_VAL */
#define TMDS_STAT_CNT2_VAL 30
#define F_STATUS_CNT2_VAL(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT2_VAL_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT3_CTRL */
#define TMDS_STAT_CNT3_CTRL 31
#define F_STATUS_CNT3_INC_SEL(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_STATUS_CNT3_INC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_STATUS_CNT3_DEC_SEL(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_STATUS_CNT3_DEC_SEL_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_STATUS_CNT3_CLR_SEL(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_STATUS_CNT3_CLR_SEL_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_STATUS_CNT3_START_SEL(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_STATUS_CNT3_START_SEL_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_STATUS_CNT3_STOP_SEL(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_STATUS_CNT3_STOP_SEL_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)

/* register TMDS_STAT_CNT3_THR */
#define TMDS_STAT_CNT3_THR 32
#define F_STATUS_CNT3_THR(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT3_THR_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_STAT_CNT3_VAL */
#define TMDS_STAT_CNT3_VAL 33
#define F_STATUS_CNT3_VAL(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_STATUS_CNT3_VAL_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register TMDS_POLARITY_STAT */
#define TMDS_POLARITY_STAT 34
#define F_TMDS_HSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_TMDS_HSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_TMDS_VSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_TMDS_VSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register TMDS_MHL_HD_ERR_INT_MASK */
#define TMDS_MHL_HD_ERR_INT_MASK 35
#define F_TMDS_MHL_HD_ERR_MASK(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TMDS_MHL_HD_ERR_MASK_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TMDS_MHL_HD_ERR_INT_STAT */
#define TMDS_MHL_HD_ERR_INT_STAT 36
#define F_TMDS_MHL_HD_ERR_STATUS(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TMDS_MHL_HD_ERR_STATUS_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register MHL_HD_INT_MASK */
#define MHL_HD_INT_MASK 37
#define F_MHL_HD_INT_MASK(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_MHL_HD_INT_MASK_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)

/* register MHL_HD_INT_STAT */
#define MHL_HD_INT_STAT 38
#define F_MHL_HD_INT_STATUS(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_MHL_HD_INT_STATUS_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)

/* register PKT_ACR_CTS_CTRL */
#define PKT_ACR_CTS_CTRL 39
#define F_SW_FORCE_ACR_CTS(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SW_FORCE_ACR_CTS_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_PKT_DEC_ACR_CTS(x) (((x) & ((1 << 20) - 1)) << 1)
#define F_PKT_DEC_ACR_CTS_RD(x) (((x) & (((1 << 20) - 1) << 1)) >> 1)

/* register PKT_ACR_N_CTRL */
#define PKT_ACR_N_CTRL 40
#define F_SW_FORCE_ACR_N(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SW_FORCE_ACR_N_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_PKT_DEC_ACR_N(x) (((x) & ((1 << 20) - 1)) << 1)
#define F_PKT_DEC_ACR_N_RD(x) (((x) & (((1 << 20) - 1) << 1)) >> 1)

/* register PKT_AVI_DATA_LOW */
#define PKT_AVI_DATA_LOW 41
#define F_PKT_DEC_AVI_DATA_LOW(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_DEC_AVI_DATA_LOW_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_AVI_DATA_HIGH */
#define PKT_AVI_DATA_HIGH 42
#define F_PKT_DEC_AVI_DATA_HIGH(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_PKT_DEC_AVI_DATA_HIGH_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

#endif /* SINK_MHL_HD */

