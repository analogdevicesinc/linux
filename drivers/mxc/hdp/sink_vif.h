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
 * sink_vif.h
 *
 ******************************************************************************
 */
#ifndef SINK_VIF_H_
#define SINK_VIF_H_

/* register VIDEO_UNPACK_CFG */
#define VIDEO_UNPACK_CFG 0
#define F_SW_CD_PHASE(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SW_CD_PHASE_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_FORCE_SW_CD(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_FORCE_SW_CD_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SW_CD_COLOR_DEPTH(x) (((x) & ((1 << 4) - 1)) << 5)
#define F_SW_CD_COLOR_DEPTH_RD(x) (((x) & (((1 << 4) - 1) << 5)) >> 5)
#define F_FORCE_SW_PHASE(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_FORCE_SW_PHASE_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)
#define F_VIDEO_PIXEL_ENCODING(x) (((x) & ((1 << 2) - 1)) << 10)
#define F_VIDEO_PIXEL_ENCODING_RD(x) (((x) & (((1 << 2) - 1) << 10)) >> 10)

/* register VIDEO_UNPACK_CTRL */
#define VIDEO_UNPACK_CTRL 1
#define F_SW_CD_FSM_CLR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SW_CD_FSM_CLR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_CD_ENABLE(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_CD_ENABLE_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_CD_FIFO_AEMPTY_TH(x) (((x) & ((1 << 5) - 1)) << 2)
#define F_CD_FIFO_AEMPTY_TH_RD(x) (((x) & (((1 << 5) - 1) << 2)) >> 2)
#define F_FSM_ERROR_ENABLE(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_FSM_ERROR_ENABLE_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register VIDEO_UNPACK_STAT */
#define VIDEO_UNPACK_STAT 2
#define F_CD_FIFO_OVERRUN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_CD_FIFO_OVERRUN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_CD_FIFO_UNDERRUN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_CD_FIFO_UNDERRUN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_CD_PHASE(x) (((x) & ((1 << 4) - 1)) << 2)
#define F_CD_PHASE_RD(x) (((x) & (((1 << 4) - 1) << 2)) >> 2)
#define F_CD_COLOR_DEPTH(x) (((x) & ((1 << 4) - 1)) << 6)
#define F_CD_COLOR_DEPTH_RD(x) (((x) & (((1 << 4) - 1) << 6)) >> 6)
#define F_CD_LAST_PHASE(x) (((x) & ((1 << 4) - 1)) << 10)
#define F_CD_LAST_PHASE_RD(x) (((x) & (((1 << 4) - 1) << 10)) >> 10)
#define F_CD_FIFO_EMPTY(x) (((x) & ((1 << 1) - 1)) << 14)
#define F_CD_FIFO_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 14)) >> 14)
#define F_CD_FIFO_FULL(x) (((x) & ((1 << 1) - 1)) << 15)
#define F_CD_FIFO_FULL_RD(x) (((x) & (((1 << 1) - 1) << 15)) >> 15)
#define F_CD_STATE(x) (((x) & ((1 << 5) - 1)) << 16)
#define F_CD_STATE_RD(x) (((x) & (((1 << 5) - 1) << 16)) >> 16)

/* register VANLYZ_CTRL */
#define VANLYZ_CTRL 4
#define F_VANLYZ_START(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_VANLYZ_START_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_VANLYZ_RESET(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_VANLYZ_RESET_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_VANLYZ_FRAMES_CHECK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_VANLYZ_FRAMES_CHECK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_VANLYZ_FORMAT_FINDER_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_VANLYZ_FORMAT_FINDER_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register VANLYZ_FRAMES_TO_CHECK */
#define VANLYZ_FRAMES_TO_CHECK 5
#define F_VANLYZ_FRAMES_TO_CHECK(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_VANLYZ_FRAMES_TO_CHECK_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register VANLYZ_CFG_0 */
#define VANLYZ_CFG_0 6
#define F_VANLYZ_HSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_VANLYZ_HSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_VANLYZ_VSYNC_POLARITY(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_VANLYZ_VSYNC_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_VANLYZ_BITWIDTH(x) (((x) & ((1 << 2) - 1)) << 2)
#define F_VANLYZ_BITWIDTH_RD(x) (((x) & (((1 << 2) - 1) << 2)) >> 2)

/* register VANLYZ_CFG_1 */
#define VANLYZ_CFG_1 7
#define F_VANLYZ_FRONT_PORCH(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_FRONT_PORCH_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_VANLYZ_BACK_PORCH(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_VANLYZ_BACK_PORCH_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register VANLYZ_CFG_2 */
#define VANLYZ_CFG_2 8
#define F_VANLYZ_ACTIVE_SLOT(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_ACTIVE_SLOT_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_VANLYZ_FRAME_LINES(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_VANLYZ_FRAME_LINES_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register VANLYZ_CFG_3 */
#define VANLYZ_CFG_3 9
#define F_VANLYZ_LINE_WIDTH(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_LINE_WIDTH_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register VANLYZ_CFG_4 */
#define VANLYZ_CFG_4 10
#define F_VANLYZ_NUM_CLK_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_VANLYZ_NUM_CLK_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_VANLYZ_VSYNC_LINES(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_VANLYZ_VSYNC_LINES_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register VANLYZ_CFG_5 */
#define VANLYZ_CFG_5 11
#define F_VANLYZ_3D_MODE(x) (((x) & ((1 << 3) - 1)) << 0)
#define F_VANLYZ_3D_MODE_RD(x) (((x) & (((1 << 3) - 1) << 0)) >> 0)
#define F_VANLYZ_EOF_LINES(x) (((x) & ((1 << 8) - 1)) << 3)
#define F_VANLYZ_EOF_LINES_RD(x) (((x) & (((1 << 8) - 1) << 3)) >> 3)
#define F_VANLYZ_SOF_LINES(x) (((x) & ((1 << 8) - 1)) << 11)
#define F_VANLYZ_SOF_LINES_RD(x) (((x) & (((1 << 8) - 1) << 11)) >> 11)

/* register VANLYZ_CLK_METER_REF_CYC */
#define VANLYZ_CLK_METER_REF_CYC 12
#define F_VANLYZ_CLK_METER_REF_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_VANLYZ_CLK_METER_REF_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

/* register VANLYZ_CLK_METER_MEAS_TOLRNCE */
#define VANLYZ_CLK_METER_MEAS_TOLRNCE 13
#define F_VANLYZ_CLK_METER_MEAS_TOLRNCE(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_CLK_METER_MEAS_TOLRNCE_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register VANLYZ_FORMAT_NUM */
#define VANLYZ_FORMAT_NUM 14
#define F_VANLYZ_FORMAT1_NUM(x) (((x) & ((1 << 6) - 1)) << 0)
#define F_VANLYZ_FORMAT1_NUM_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
#define F_VANLYZ_FORMAT2_NUM(x) (((x) & ((1 << 6) - 1)) << 8)
#define F_VANLYZ_FORMAT2_NUM_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)

/* register VANLYZ_FAILURES */
#define VANLYZ_FAILURES 15
#define F_VANLYZ_FAILURES(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_FAILURES_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register VANLYZ_ST_0 */
#define VANLYZ_ST_0 16
#define F_VANLYZ_STATUS_HP(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_VANLYZ_STATUS_HP_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_VANLYZ_STATUS_VP(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_VANLYZ_STATUS_VP_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register VANLYZ_ST_1 */
#define VANLYZ_ST_1 17
#define F_VANLYZ_STATUS_FRAME_LINES(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_STATUS_FRAME_LINES_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_VANLYZ_STATUS_FP(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_VANLYZ_STATUS_FP_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register VANLYZ_ST_2 */
#define VANLYZ_ST_2 18
#define F_VANLYZ_STATUS_BP(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_STATUS_BP_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_VANLYZ_STATUS_AS(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_VANLYZ_STATUS_AS_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register VANLYZ_STATUS_3 */
#define VANLYZ_STATUS_3 19
#define F_VANLYZ_STATUS_LINE_WIDTH(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_STATUS_LINE_WIDTH_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register VANLYZ_STATUS_4 */
#define VANLYZ_STATUS_4 20
#define F_VANLYZ_STATUS_VSYNC_LINES(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_VANLYZ_STATUS_VSYNC_LINES_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_VANLYZ_STATUS_EOF_LINES(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_VANLYZ_STATUS_EOF_LINES_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_VANLYZ_STATUS_SOF_LINES(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_VANLYZ_STATUS_SOF_LINES_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)

/* register VANLYZ_STATUS_5 */
#define VANLYZ_STATUS_5 21
#define F_VANLYZ_FORMAT_FINDER_ADD(x) (((x) & ((1 << 6) - 1)) << 0)
#define F_VANLYZ_FORMAT_FINDER_ADD_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
#define F_VANLYZ_FORMAT_FINDER_ACTIVE(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_VANLYZ_FORMAT_FINDER_ACTIVE_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_VANLYZ_CNT_LINES(x) (((x) & ((1 << 18) - 1)) << 7)
#define F_VANLYZ_CNT_LINES_RD(x) (((x) & (((1 << 18) - 1) << 7)) >> 7)

/* register VANLYZ_STATUS_6 */
#define VANLYZ_STATUS_6 22
#define F_VANLYZ_CNT_VIDEO(x) (((x) & ((1 << 18) - 1)) << 0)
#define F_VANLYZ_CNT_VIDEO_RD(x) (((x) & (((1 << 18) - 1) << 0)) >> 0)

/* register VANLYZ_STATUS_7 */
#define VANLYZ_STATUS_7 23
#define F_VANLYZ_VIDEO_SIZE(x) (((x) & ((1 << 18) - 1)) << 0)
#define F_VANLYZ_VIDEO_SIZE_RD(x) (((x) & (((1 << 18) - 1) << 0)) >> 0)

/* register VANLYZ_PIC_CFG_0 */
#define VANLYZ_PIC_CFG_0 24
#define F_VANLYZ_PIC_R(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_PIC_R_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_VANLYZ_PIC_G(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_VANLYZ_PIC_G_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register VANLYZ_PIC_CFG_1 */
#define VANLYZ_PIC_CFG_1 25
#define F_VANLYZ_PIC_B(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_VANLYZ_PIC_B_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register VIF_MHL_HD_ERR_INT_MASK */
#define VIF_MHL_HD_ERR_INT_MASK 26
#define F_VIF_MHL_HD_ERR_MASK(x) (((x) & ((1 << 20) - 1)) << 0)
#define F_VIF_MHL_HD_ERR_MASK_RD(x) (((x) & (((1 << 20) - 1) << 0)) >> 0)

/* register VIF_MHL_HD_ERR_INT_STAT */
#define VIF_MHL_HD_ERR_INT_STAT 27
#define F_VIF_MHL_HD_ERR_STATUS(x) (((x) & ((1 << 20) - 1)) << 0)
#define F_VIF_MHL_HD_ERR_STATUS_RD(x) (((x) & (((1 << 20) - 1) << 0)) >> 0)

/* register VIF_IP_DETECT_CTRL */
#define VIF_IP_DETECT_CTRL 28
#define F_READ_DTCT_ERR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_READ_DTCT_ERR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_IP_DTCT_WIN(x) (((x) & ((1 << 12) - 1)) << 1)
#define F_IP_DTCT_WIN_RD(x) (((x) & (((1 << 12) - 1) << 1)) >> 1)
#define F_IP_DTCT_EN(x) (((x) & ((1 << 1) - 1)) << 13)
#define F_IP_DTCT_EN_RD(x) (((x) & (((1 << 1) - 1) << 13)) >> 13)

/* register VIF_IP_DETECT_ST1 */
#define VIF_IP_DETECT_ST1 29
#define F_IP_DTCT_HSYNC2VSYNC_F1(x) (((x) & ((1 << 16) - 1)) << 0)
#define F_IP_DTCT_HSYNC2VSYNC_F1_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
#define F_IP_DTCT_HSYNC2VSYNC_F2(x) (((x) & ((1 << 16) - 1)) << 16)
#define F_IP_DTCT_HSYNC2VSYNC_F2_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register VIF_IP_DETECT_ST2 */
#define VIF_IP_DETECT_ST2 30
#define F_IP_STATE(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_IP_STATE_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)
#define F_IP_DTCT_ERR(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_IP_DTCT_ERR_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_IP_DTCT_HJITTER(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_IP_DTCT_HJITTER_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_IP_DTCT_VJITTER(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_IP_DTCT_VJITTER_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_IP_DTCT_IP(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_IP_DTCT_IP_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_IP_DTCT_FIELD(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_IP_DTCT_FIELD_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)

#endif /* SINK_VIF */
