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
 ******************************************************************************
 *
 * This file was auto-generated. Do not edit it manually.
 *
 ******************************************************************************
 *
 * mhl_hdtx_top.h
 *
 ******************************************************************************
 */

#ifndef MHL_HDTX_TOP_H_
# define MHL_HDTX_TOP_H_

/* register SCHEDULER_H_SIZE */
# define SCHEDULER_H_SIZE 0
# define F_H_BLANK_SIZE(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_H_BLANK_SIZE_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_H_ACTIVE_SIZE(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_H_ACTIVE_SIZE_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register SCHEDULER_V_SIZE */
# define SCHEDULER_V_SIZE 1
# define F_V_BLANK_SIZE(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_V_BLANK_SIZE_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_V_ACTIVE_SIZE(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_V_ACTIVE_SIZE_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register SCHEDULER_KEEP_OUT */
# define SCHEDULER_KEEP_OUT 2
# define F_HKEEP_OUT(x) (((x) & ((1 << 9) - 1)) << 0)
# define F_HKEEP_OUT_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)
# define F_VKEEP_OUT_START(x) (((x) & ((1 << 11) - 1)) << 9)
# define F_VKEEP_OUT_START_RD(x) (((x) & (((1 << 11) - 1) << 9)) >> 9)
# define F_VKEEP_OUT_ZONE(x) (((x) & ((1 << 8) - 1)) << 20)
# define F_VKEEP_OUT_ZONE_RD(x) (((x) & (((1 << 8) - 1) << 20)) >> 20)

/* register HDTX_SIGNAL_FRONT_WIDTH */
# define HDTX_SIGNAL_FRONT_WIDTH 3
# define F_HFRONT(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_HFRONT_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_VFRONT(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_VFRONT_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register HDTX_SIGNAL_SYNC_WIDTH */
# define HDTX_SIGNAL_SYNC_WIDTH 4
# define F_HSYNC(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_HSYNC_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_VSYNC(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_VSYNC_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register HDTX_SIGNAL_BACK_WIDTH */
# define HDTX_SIGNAL_BACK_WIDTH 5
# define F_HBACK(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_HBACK_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_VBACK(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_VBACK_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register HDTX_CONTROLLER */
# define HDTX_CONTROLLER 6
# define F_HDMI_MODE(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_HDMI_MODE_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)
# define F_VIF_DATA_WIDTH(x) (((x) & ((1 << 2) - 1)) << 2)
# define F_VIF_DATA_WIDTH_RD(x) (((x) & (((1 << 2) - 1) << 2)) >> 2)
# define F_AUTO_MODE(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_AUTO_MODE_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
# define F_IL_PROG(x) (((x) & ((1 << 2) - 1)) << 5)
# define F_IL_PROG_RD(x) (((x) & (((1 << 2) - 1) << 5)) >> 5)
# define F_PIC_3D(x) (((x) & ((1 << 4) - 1)) << 7)
# define F_PIC_3D_RD(x) (((x) & (((1 << 4) - 1) << 7)) >> 7)
# define F_BCH_EN(x) (((x) & ((1 << 1) - 1)) << 11)
# define F_BCH_EN_RD(x) (((x) & (((1 << 1) - 1) << 11)) >> 11)
# define F_GCP_EN(x) (((x) & ((1 << 1) - 1)) << 12)
# define F_GCP_EN_RD(x) (((x) & (((1 << 1) - 1) << 12)) >> 12)
# define F_SET_AVMUTE(x) (((x) & ((1 << 1) - 1)) << 13)
# define F_SET_AVMUTE_RD(x) (((x) & (((1 << 1) - 1) << 13)) >> 13)
# define F_CLEAR_AVMUTE(x) (((x) & ((1 << 1) - 1)) << 14)
# define F_CLEAR_AVMUTE_RD(x) (((x) & (((1 << 1) - 1) << 14)) >> 14)
# define F_DATA_EN(x) (((x) & ((1 << 1) - 1)) << 15)
# define F_DATA_EN_RD(x) (((x) & (((1 << 1) - 1) << 15)) >> 15)
# define F_HDMI_ENCODING(x) (((x) & ((1 << 2) - 1)) << 16)
# define F_HDMI_ENCODING_RD(x) (((x) & (((1 << 2) - 1) << 16)) >> 16)
# define F_HDMI2_PREAMBLE_EN(x) (((x) & ((1 << 1) - 1)) << 18)
# define F_HDMI2_PREAMBLE_EN_RD(x) (((x) & (((1 << 1) - 1) << 18)) >> 18)
# define F_HDMI2_CTRL_IL_MODE(x) (((x) & ((1 << 1) - 1)) << 19)
# define F_HDMI2_CTRL_IL_MODE_RD(x) (((x) & (((1 << 1) - 1) << 19)) >> 19)

/* register HDTX_HDCP */
# define HDTX_HDCP 7
# define F_HDTX_HDCP_SELECT(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_HDTX_HDCP_SELECT_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)
# define F_ENC_BIT(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_ENC_BIT_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_HDCP_ENABLE_1P1_FEATURES(x) (((x) & ((1 << 1) - 1)) << 3)
# define F_HDCP_ENABLE_1P1_FEATURES_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
# define F_HDCP_DELAY_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_HDCP_DELAY_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
# define F_HDCP_DELAY_FIFO_SW_START(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_HDCP_DELAY_FIFO_SW_START_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
# define F_HDCP_DOUBLE_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 6)
# define F_HDCP_DOUBLE_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
# define F_HDCP_SINGLE_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 7)
# define F_HDCP_SINGLE_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
# define F_HDCP_DELAY_FIFO_AFULL_THR(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_HDCP_DELAY_FIFO_AFULL_THR_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_HDCP_CTRL_SW_RST(x) (((x) & ((1 << 1) - 1)) << 12)
# define F_HDCP_CTRL_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 12)) >> 12)
# define F_HDCP_CTRL_IL_MODE(x) (((x) & ((1 << 1) - 1)) << 13)
# define F_HDCP_CTRL_IL_MODE_RD(x) (((x) & (((1 << 1) - 1) << 13)) >> 13)

/* register HDTX_HPD */
# define HDTX_HPD 8
# define F_HPD_VALID_WIDTH(x) (((x) & ((1 << 12) - 1)) << 0)
# define F_HPD_VALID_WIDTH_RD(x) (((x) & (((1 << 12) - 1) << 0)) >> 0)
# define F_HPD_GLITCH_WIDTH(x) (((x) & ((1 << 8) - 1)) << 12)
# define F_HPD_GLITCH_WIDTH_RD(x) (((x) & (((1 << 8) - 1) << 12)) >> 12)

/* register HDTX_CLOCK_REG_0 */
# define HDTX_CLOCK_REG_0 9
# define F_DATA_REGISTER_VAL_0(x) (((x) & ((1 << 20) - 1)) << 0)
# define F_DATA_REGISTER_VAL_0_RD(x) (((x) & (((1 << 20) - 1) << 0)) >> 0)

/* register HDTX_CLOCK_REG_1 */
# define HDTX_CLOCK_REG_1 10
# define F_DATA_REGISTER_VAL_1(x) (((x) & ((1 << 20) - 1)) << 0)
# define F_DATA_REGISTER_VAL_1_RD(x) (((x) & (((1 << 20) - 1) << 0)) >> 0)

/* register HPD_PLUG_IN */
# define HPD_PLUG_IN 11
# define F_FILTER_HPD(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_FILTER_HPD_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register HDCP_IN */
# define HDCP_IN 12
# define F_HDCP_ESS_STATE(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_HDCP_ESS_STATE_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_HDCP_DOUBLE_FIFO_WFULL(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_HDCP_DOUBLE_FIFO_WFULL_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
# define F_HDCP_DOUBLE_FIFO_REMPTY(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_HDCP_DOUBLE_FIFO_REMPTY_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
# define F_HDCP_DOUBLE_FIFO_OVERRUN(x) (((x) & ((1 << 1) - 1)) << 6)
# define F_HDCP_DOUBLE_FIFO_OVERRUN_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
# define F_HDCP_DOUBLE_FIFO_UNDERRUN(x) (((x) & ((1 << 1) - 1)) << 7)
# define F_HDCP_DOUBLE_FIFO_UNDERRUN_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
# define F_HDCP_DELAY_FIFO_EMPTY(x) (((x) & ((1 << 1) - 1)) << 8)
# define F_HDCP_DELAY_FIFO_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
# define F_HDCP_DELAY_FIFO_FULL(x) (((x) & ((1 << 1) - 1)) << 9)
# define F_HDCP_DELAY_FIFO_FULL_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)
# define F_HDCP_SINGLE_FIFO_WFULL(x) (((x) & ((1 << 2) - 1)) << 10)
# define F_HDCP_SINGLE_FIFO_WFULL_RD(x) (((x) & (((1 << 2) - 1) << 10)) >> 10)
# define F_HDCP_SINGLE_FIFO_REMPTY(x) (((x) & ((1 << 2) - 1)) << 12)
# define F_HDCP_SINGLE_FIFO_REMPTY_RD(x) (((x) & (((1 << 2) - 1) << 12)) >> 12)
# define F_HDCP_SINGLE_FIFO_OVERRUN(x) (((x) & ((1 << 2) - 1)) << 14)
# define F_HDCP_SINGLE_FIFO_OVERRUN_RD(x) (((x) & (((1 << 2) - 1) << 14)) >> 14)
# define F_HDCP_SINGLE_FIFO_UNDERRUN(x) (((x) & ((1 << 2) - 1)) << 16)
# define F_HDCP_SINGLE_FIFO_UNDERRUN_RD(x) (((x) & (((1 << 2) - 1) << 16)) >> 16)

/* register GCP_FORCE_COLOR_DEPTH_CODING */
# define GCP_FORCE_COLOR_DEPTH_CODING 13
# define F_COLOR_DEPTH_VAL(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_COLOR_DEPTH_VAL_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_COLOR_DEPTH_FORCE(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_COLOR_DEPTH_FORCE_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
# define F_DEFAULT_PHASE_VAL(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_DEFAULT_PHASE_VAL_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)

/* register SSCP_POSITIONING */
# define SSCP_POSITIONING 14
# define F_SSCP_ROW_VAL(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_SSCP_ROW_VAL_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_SSCP_COL_VAL(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_SSCP_COL_VAL_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register HDCP_WIN_OF_OPP_POSITION */
# define HDCP_WIN_OF_OPP_POSITION 15
# define F_HDCP_WIN_OF_OPP_START(x) (((x) & ((1 << 10) - 1)) << 0)
# define F_HDCP_WIN_OF_OPP_START_RD(x) (((x) & (((1 << 10) - 1) << 0)) >> 0)
# define F_HDCP_WIN_OF_OPP_SIZE(x) (((x) & ((1 << 6) - 1)) << 10)
# define F_HDCP_WIN_OF_OPP_SIZE_RD(x) (((x) & (((1 << 6) - 1) << 10)) >> 10)

#endif //MHL_HDTX_TOP
