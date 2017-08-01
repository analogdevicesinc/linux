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
 * dptx_framer.h
 *
 ******************************************************************************
 */

#ifndef DPTX_FRAMER_H_
# define DPTX_FRAMER_H_

/* register DP_FRAMER_GLOBAL_CONFIG */
# define DP_FRAMER_GLOBAL_CONFIG 0
# define F_NUM_LANES(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_NUM_LANES_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)
# define F_MST_SST(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_MST_SST_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_GLOBAL_EN(x) (((x) & ((1 << 1) - 1)) << 3)
# define F_GLOBAL_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
# define F_RG_EN(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_RG_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
# define F_NO_VIDEO(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_NO_VIDEO_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
# define F_ENC_RST_DIS(x) (((x) & ((1 << 1) - 1)) << 6)
# define F_ENC_RST_DIS_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
# define F_WR_VHSYNC_FALL(x) (((x) & ((1 << 1) - 1)) << 7)
# define F_WR_VHSYNC_FALL_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register DP_SW_RESET */
# define DP_SW_RESET 1
# define F_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register DP_FRAMER_TU */
# define DP_FRAMER_TU 2
# define F_TU_VALID_SYMBOLS(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_TU_VALID_SYMBOLS_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_TU_SIZE(x) (((x) & ((1 << 7) - 1)) << 8)
# define F_TU_SIZE_RD(x) (((x) & (((1 << 7) - 1) << 8)) >> 8)
# define F_TU_CNT_RST_EN(x) (((x) & ((1 << 1) - 1)) << 15)
# define F_TU_CNT_RST_EN_RD(x) (((x) & (((1 << 1) - 1) << 15)) >> 15)
# define F_BS_SR_REPLACE_POSITION(x) (((x) & ((1 << 9) - 1)) << 16)
# define F_BS_SR_REPLACE_POSITION_RD(x) (((x) & (((1 << 9) - 1) << 16)) >> 16)

/* register DP_FRAMER_PXL_REPR */
# define DP_FRAMER_PXL_REPR 3
# define F_COLOR_DEPTH(x) (((x) & ((1 << 5) - 1)) << 0)
# define F_COLOR_DEPTH_RD(x) (((x) & (((1 << 5) - 1) << 0)) >> 0)
# define F_PXL_ENC_FORMAT(x) (((x) & ((1 << 5) - 1)) << 8)
# define F_PXL_ENC_FORMAT_RD(x) (((x) & (((1 << 5) - 1) << 8)) >> 8)

/* register DP_FRAMER_SP */
# define DP_FRAMER_SP 4
# define F_VSP(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_VSP_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_HSP(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_HSP_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_INTERLACE_EN(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_INTERLACE_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_FRAMER_3D_EN(x) (((x) & ((1 << 1) - 1)) << 3)
# define F_FRAMER_3D_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
# define F_STACKED_3D_EN(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_STACKED_3D_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)

/* register AUDIO_PACK_CONTROL */
# define AUDIO_PACK_CONTROL 5
# define F_MST_SDP_ID(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_MST_SDP_ID_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
# define F_AUDIO_PACK_EN(x) (((x) & ((1 << 1) - 1)) << 8)
# define F_AUDIO_PACK_EN_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
# define F_MONO(x) (((x) & ((1 << 1) - 1)) << 9)
# define F_MONO_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)

/* register DP_VC_TABLE_0 */
# define DP_VC_TABLE_0 6
# define F_VC_TABLE_0(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_0_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_1(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_1_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_2(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_2_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_3(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_3_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_1 */
# define DP_VC_TABLE_1 7
# define F_VC_TABLE_4(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_4_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_5(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_5_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_6(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_6_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_7(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_7_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_2 */
# define DP_VC_TABLE_2 8
# define F_VC_TABLE_8(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_8_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_9(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_9_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_10(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_10_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_11(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_11_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_3 */
# define DP_VC_TABLE_3 9
# define F_VC_TABLE_12(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_12_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_13(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_13_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_14(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_14_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_15(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_15_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_4 */
# define DP_VC_TABLE_4 10
# define F_VC_TABLE_16(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_16_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_17(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_17_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_18(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_18_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_19(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_19_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_5 */
# define DP_VC_TABLE_5 11
# define F_VC_TABLE_20(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_20_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_21(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_21_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_22(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_22_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_23(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_23_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_6 */
# define DP_VC_TABLE_6 12
# define F_VC_TABLE_24(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_24_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_25(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_25_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_26(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_26_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_27(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_27_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_7 */
# define DP_VC_TABLE_7 13
# define F_VC_TABLE_28(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_28_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_29(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_29_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_30(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_30_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_31(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_31_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_8 */
# define DP_VC_TABLE_8 14
# define F_VC_TABLE_32(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_32_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_33(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_33_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_34(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_34_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_35(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_35_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_9 */
# define DP_VC_TABLE_9 15
# define F_VC_TABLE_36(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_VC_TABLE_36_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
# define F_VC_TABLE_37(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_37_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_38(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_38_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_39(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_39_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_10 */
# define DP_VC_TABLE_10 16
# define F_VC_TABLE_40(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_40_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_41(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_41_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_42(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_42_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_43(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_43_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_11 */
# define DP_VC_TABLE_11 17
# define F_VC_TABLE_44(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_44_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_45(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_45_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_46(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_46_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_47(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_47_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_12 */
# define DP_VC_TABLE_12 18
# define F_VC_TABLE_48(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_48_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_49(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_49_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_50(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_50_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_51(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_51_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_13 */
# define DP_VC_TABLE_13 19
# define F_VC_TABLE_52(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_52_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_53(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_53_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_54(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_54_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_55(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_55_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VC_TABLE_14 */
# define DP_VC_TABLE_14 20
# define F_VC_TABLE_56(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_VC_TABLE_56_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_57(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_57_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_58(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_58_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_59(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_59_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register LINE_THRESH */
# define LINE_THRESH 21
# define F_CFG_ACTIVE_LINE_TRESH(x) (((x) & ((1 << 6) - 1)) << 0)
# define F_CFG_ACTIVE_LINE_TRESH_RD(x) (((x) & (((1 << 6) - 1) << 0)) >> 0)
# define F_VC_TABLE_61(x) (((x) & ((1 << 6) - 1)) << 8)
# define F_VC_TABLE_61_RD(x) (((x) & (((1 << 6) - 1) << 8)) >> 8)
# define F_VC_TABLE_62(x) (((x) & ((1 << 6) - 1)) << 16)
# define F_VC_TABLE_62_RD(x) (((x) & (((1 << 6) - 1) << 16)) >> 16)
# define F_VC_TABLE_63(x) (((x) & ((1 << 6) - 1)) << 24)
# define F_VC_TABLE_63_RD(x) (((x) & (((1 << 6) - 1) << 24)) >> 24)

/* register DP_VB_ID */
# define DP_VB_ID 22
# define F_VB_ID(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_VB_ID_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register DP_MTPH_LVP_CONTROL */
# define DP_MTPH_LVP_CONTROL 23
# define F_MTPH_LVP_EN(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_MTPH_LVP_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register DP_MTPH_SYMBOL_VALUES */
# define DP_MTPH_SYMBOL_VALUES 24
# define F_MPTH_LVP_SYM(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_MPTH_LVP_SYM_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
# define F_MTPH_ECF_SYM(x) (((x) & ((1 << 8) - 1)) << 8)
# define F_MTPH_ECF_SYM_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
# define F_MTPH_MTPH_SYM(x) (((x) & ((1 << 8) - 1)) << 16)
# define F_MTPH_MTPH_SYM_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)

/* register DP_MTPH_ECF_CONTROL */
# define DP_MTPH_ECF_CONTROL 25
# define F_MPTH_ECF_EN(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_MPTH_ECF_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_MTPH_ACT_EN(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_MTPH_ACT_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register DP_FIELDSEQ_3D */
# define DP_FIELDSEQ_3D 26
# define F_FIELD_SEQ_START(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_FIELD_SEQ_START_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_FIELD_SEQ_END(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_FIELD_SEQ_END_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register DP_MTPH_STATUS */
# define DP_MTPH_STATUS 27
# define F_MTP_ACT_CNT_CTRL_CURR_STATE(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_MTP_ACT_CNT_CTRL_CURR_STATE_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
# define F_MTP_ECF_CNT_CTRL_CURR_STATE(x) (((x) & ((1 << 8) - 1)) << 8)
# define F_MTP_ECF_CNT_CTRL_CURR_STATE_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
# define F_MTPH_ACT_STATUS(x) (((x) & ((1 << 1) - 1)) << 16)
# define F_MTPH_ACT_STATUS_RD(x) (((x) & (((1 << 1) - 1) << 16)) >> 16)
# define F_MTPH_ECF_STATUS(x) (((x) & ((1 << 1) - 1)) << 17)
# define F_MTPH_ECF_STATUS_RD(x) (((x) & (((1 << 1) - 1) << 17)) >> 17)
# define F_MTPH_LVP_STATUS(x) (((x) & ((1 << 1) - 1)) << 18)
# define F_MTPH_LVP_STATUS_RD(x) (((x) & (((1 << 1) - 1) << 18)) >> 18)

/* register DP_INTERRUPT_SOURCE */
# define DP_INTERRUPT_SOURCE 28
# define F_PSLVERR(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_PSLVERR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_MTPH_ACT_EN_CLEAR(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_MTPH_ACT_EN_CLEAR_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_MTPH_LVP_EN_CLEAR(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_MTPH_LVP_EN_CLEAR_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_MTPH_ECF_EN_CLEAR(x) (((x) & ((1 << 1) - 1)) << 3)
# define F_MTPH_ECF_EN_CLEAR_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register DP_INTERRUPT_MASK */
# define DP_INTERRUPT_MASK 29
# define F_PSLVERR_MASK(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_PSLVERR_MASK_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_MTPH_ACT_EN_CLEAR_MASK(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_MTPH_ACT_EN_CLEAR_MASK_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_MTPH_LVP_EN_CLEAR_MASK(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_MTPH_LVP_EN_CLEAR_MASK_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_MTPH_ECF_EN_CLEAR_MASK(x) (((x) & ((1 << 1) - 1)) << 3)
# define F_MTPH_ECF_EN_CLEAR_MASK_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register DP_FRONT_BACK_PORCH */
# define DP_FRONT_BACK_PORCH 30
# define F_BACK_PORCH(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_BACK_PORCH_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)
# define F_FRONT_PORCH(x) (((x) & ((1 << 16) - 1)) << 16)
# define F_FRONT_PORCH_RD(x) (((x) & (((1 << 16) - 1) << 16)) >> 16)

/* register DP_BYTE_COUNT */
# define DP_BYTE_COUNT 31
# define F_BYTE_COUNT(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_BYTE_COUNT_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

#endif //DPTX_FRAMER
