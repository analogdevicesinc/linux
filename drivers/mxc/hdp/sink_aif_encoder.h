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
 * sink_aif_encoder.h
 *
 ******************************************************************************
 */

#ifndef SINK_AIF_ENCODER_H_
#define SINK_AIF_ENCODER_H_

/* register AUDIO_SINK_CNTL */
#define AUDIO_SINK_CNTL 0
#define F_SINK_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_I2S_ENC_START(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_I2S_ENC_START_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register AUDIO_SINK_STTS */
#define AUDIO_SINK_STTS 1
#define F_CH_INDX_ERR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_CH_INDX_ERR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register AUDIO_SINK_CNFG */
#define AUDIO_SINK_CNFG 2
#define F_ENC_LOW_INDEX_MSB(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_ENC_LOW_INDEX_MSB_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_AUDIO_CH_NUM(x) (((x) & ((1 << 5) - 1)) << 1)
#define F_SINK_AUDIO_CH_NUM_RD(x) (((x) & (((1 << 5) - 1) << 1)) >> 1)
#define F_ENC_SAMPLE_JUST(x) (((x) & ((1 << 2) - 1)) << 6)
#define F_ENC_SAMPLE_JUST_RD(x) (((x) & (((1 << 2) - 1) << 6)) >> 6)
#define F_ENC_SMPL_WIDTH(x) (((x) & ((1 << 2) - 1)) << 8)
#define F_ENC_SMPL_WIDTH_RD(x) (((x) & (((1 << 2) - 1) << 8)) >> 8)
#define F_I2S_ENC_WL_SIZE(x) (((x) & ((1 << 2) - 1)) << 10)
#define F_I2S_ENC_WL_SIZE_RD(x) (((x) & (((1 << 2) - 1) << 10)) >> 10)
#define F_CNTL_SMPL_ONLY_EN(x) (((x) & ((1 << 1) - 1)) << 12)
#define F_CNTL_SMPL_ONLY_EN_RD(x) (((x) & (((1 << 1) - 1) << 12)) >> 12)
#define F_SINK_AIF_TYPE_SMPL(x) (((x) & ((1 << 4) - 1)) << 13)
#define F_SINK_AIF_TYPE_SMPL_RD(x) (((x) & (((1 << 4) - 1) << 13)) >> 13)
#define F_CNTL_TYPE_OVRD(x) (((x) & ((1 << 4) - 1)) << 17)
#define F_CNTL_TYPE_OVRD_RD(x) (((x) & (((1 << 4) - 1) << 17)) >> 17)
#define F_CNTL_TYPE_OVRD_EN(x) (((x) & ((1 << 1) - 1)) << 21)
#define F_CNTL_TYPE_OVRD_EN_RD(x) (((x) & (((1 << 1) - 1) << 21)) >> 21)
#define F_I2S_ENC_PORT_EN(x) (((x) & ((1 << 4) - 1)) << 22)
#define F_I2S_ENC_PORT_EN_RD(x) (((x) & (((1 << 4) - 1) << 22)) >> 22)
#define F_WS_POLARITY(x) (((x) & ((1 << 1) - 1)) << 26)
#define F_WS_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 26)) >> 26)

/* register FIFO_CNTL_ADDR */
#define FIFO_CNTL_ADDR 3
#define F_CFG_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_CFG_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_CFG_INDEX_SYNC_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_CFG_INDEX_SYNC_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_CFG_FIFO_DIR(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_CFG_FIFO_DIR_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_CFG_DIS_PORT3(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_CFG_DIS_PORT3_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register FIFO_STTS_ADDR */
#define FIFO_STTS_ADDR 4
#define F_ST_WFULL_REG(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_ST_WFULL_REG_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_ST_REMPTY_REG(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_ST_REMPTY_REG_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_ST_OVERRUN_REG(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_ST_OVERRUN_REG_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_ST_UNDERRUN_REG(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_ST_UNDERRUN_REG_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SINK_COM_CH_STTS_BITS */
#define SINK_COM_CH_STTS_BITS 5
#define F_BYTE0(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_BYTE0_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_CATEGORY_CODE(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_CATEGORY_CODE_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_SAMPLING_FREQ(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_SAMPLING_FREQ_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_CLOCK_ACCURACY(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_CLOCK_ACCURACY_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
#define F_ORIGINAL_SAMP_FREQ(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_ORIGINAL_SAMP_FREQ_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register SINK_STTS_BIT_CH01 */
#define SINK_STTS_BIT_CH01 6
#define F_SOURCE_NUM_CH0(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH0_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH0(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH0_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH0(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH0_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH1(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH1_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH1(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH1_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH1(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH1_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH23 */
#define SINK_STTS_BIT_CH23 7
#define F_SOURCE_NUM_CH2(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH2_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH2(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH2_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH2(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH2_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH3(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH3_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH3(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH3_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH3(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH3_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH45 */
#define SINK_STTS_BIT_CH45 8
#define F_SOURCE_NUM_CH4(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH4_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH4(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH4_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH4(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH4_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH5(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH5_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH5(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH5_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH5(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH5_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH67 */
#define SINK_STTS_BIT_CH67 9
#define F_SOURCE_NUM_CH6(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH6_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH6(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH6_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH6(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH6_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH7(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH7_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH7(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH7_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH7(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH7_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH89 */
#define SINK_STTS_BIT_CH89 10
#define F_SOURCE_NUM_CH8(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH8_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH8(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH8_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH8(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH8_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH9(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH9_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH9(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH9_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH9(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH9_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH1011 */
#define SINK_STTS_BIT_CH1011 11
#define F_SOURCE_NUM_CH10(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH10_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH10(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH10_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH10(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH10_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH11(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH11_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH11(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH11_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH11(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH11_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH1213 */
#define SINK_STTS_BIT_CH1213 12
#define F_SOURCE_NUM_CH12(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH12_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH12(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH12_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH12(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH12_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH13(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH13_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH13(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH13_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH13(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH13_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH1415 */
#define SINK_STTS_BIT_CH1415 13
#define F_SOURCE_NUM_CH14(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH14_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH14(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH14_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH14(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH14_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH15(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH15_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH15(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH15_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH15(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH15_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH1617 */
#define SINK_STTS_BIT_CH1617 14
#define F_SOURCE_NUM_CH16(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH16_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH16(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH16_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH16(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH16_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH17(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH17_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH17(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH17_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH17(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH17_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH1819 */
#define SINK_STTS_BIT_CH1819 15
#define F_SOURCE_NUM_CH18(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH18_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH18(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH18_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH18(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH18_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH19(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH19_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH19(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH19_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH19(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH19_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH2021 */
#define SINK_STTS_BIT_CH2021 16
#define F_SOURCE_NUM_CH20(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH20_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH20(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH20_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH20(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH20_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH21(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH21_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH21(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH21_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH21(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH21_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH2223 */
#define SINK_STTS_BIT_CH2223 17
#define F_SOURCE_NUM_CH22(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH22_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH22(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH22_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH22(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH22_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH23(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH23_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH23(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH23_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH23(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH23_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH2425 */
#define SINK_STTS_BIT_CH2425 18
#define F_SOURCE_NUM_CH24(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH24_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH24(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH24_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH24(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH24_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH25(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH25_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH25(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH25_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH25(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH25_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH2627 */
#define SINK_STTS_BIT_CH2627 19
#define F_SOURCE_NUM_CH26(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH26_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH26(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH26_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH26(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH26_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH27(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH27_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH27(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH27_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH27(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH27_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH2829 */
#define SINK_STTS_BIT_CH2829 20
#define F_SOURCE_NUM_CH28(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH28_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH28(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH28_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH28(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH28_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH29(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH29_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH29(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH29_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH29(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH29_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

/* register SINK_STTS_BIT_CH3031 */
#define SINK_STTS_BIT_CH3031 21
#define F_SOURCE_NUM_CH30(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_SOURCE_NUM_CH30_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_CHANNEL_NUM_CH30(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_CHANNEL_NUM_CH30_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
#define F_WORD_LENGTH_CH30(x) (((x) & ((1 << 4) - 1)) << 8)
#define F_WORD_LENGTH_CH30_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
#define F_SOURCE_NUM_CH31(x) (((x) & ((1 << 4) - 1)) << 12)
#define F_SOURCE_NUM_CH31_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
#define F_CHANNEL_NUM_CH31(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CHANNEL_NUM_CH31_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_WORD_LENGTH_CH31(x) (((x) & ((1 << 4) - 1)) << 20)
#define F_WORD_LENGTH_CH31_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)

#endif /* SINK_AIF_ENCODER */

