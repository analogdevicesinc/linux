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
 * source_aif_decoder.h
 *
 ******************************************************************************
 */

#ifndef SOURCE_AIF_DECODER_H_
# define SOURCE_AIF_DECODER_H_

/* register AUDIO_SRC_CNTL */
# define AUDIO_SRC_CNTL 0
# define F_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_I2S_DEC_START(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_I2S_DEC_START_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_I2S_BLOCK_START_FORCE(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_I2S_BLOCK_START_FORCE_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_SPDIF_TS_EN(x) (((x) & ((1 << 1) - 1)) << 3)
# define F_SPDIF_TS_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
# define F_I2S_TS_EN(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_I2S_TS_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
# define F_VALID_BITS_FORCE(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_VALID_BITS_FORCE_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
# define F_VALID_ALL(x) (((x) & ((1 << 1) - 1)) << 6)
# define F_VALID_ALL_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)

/* register AUDIO_SRC_CNFG */
# define AUDIO_SRC_CNFG 1
# define F_LOW_INDEX_MSB(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_LOW_INDEX_MSB_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_WS_POLARITY(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_WS_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_AUDIO_CH_NUM(x) (((x) & ((1 << 5) - 1)) << 2)
# define F_AUDIO_CH_NUM_RD(x) (((x) & (((1 << 5) - 1) << 2)) >> 2)
# define F_AUDIO_SAMPLE_JUST(x) (((x) & ((1 << 2) - 1)) << 7)
# define F_AUDIO_SAMPLE_JUST_RD(x) (((x) & (((1 << 2) - 1) << 7)) >> 7)
# define F_AUDIO_SAMPLE_WIDTH(x) (((x) & ((1 << 2) - 1)) << 9)
# define F_AUDIO_SAMPLE_WIDTH_RD(x) (((x) & (((1 << 2) - 1) << 9)) >> 9)
# define F_TRANS_SMPL_WIDTH(x) (((x) & ((1 << 2) - 1)) << 11)
# define F_TRANS_SMPL_WIDTH_RD(x) (((x) & (((1 << 2) - 1) << 11)) >> 11)
# define F_AUDIO_CHANNEL_TYPE(x) (((x) & ((1 << 4) - 1)) << 13)
# define F_AUDIO_CHANNEL_TYPE_RD(x) (((x) & (((1 << 4) - 1) << 13)) >> 13)
# define F_I2S_DEC_PORT_EN(x) (((x) & ((1 << 4) - 1)) << 17)
# define F_I2S_DEC_PORT_EN_RD(x) (((x) & (((1 << 4) - 1) << 17)) >> 17)

/* register COM_CH_STTS_BITS */
# define COM_CH_STTS_BITS 2
# define F_BYTE0(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_BYTE0_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
# define F_CATEGORY_CODE(x) (((x) & ((1 << 8) - 1)) << 8)
# define F_CATEGORY_CODE_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
# define F_SAMPLING_FREQ(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_SAMPLING_FREQ_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_CLOCK_ACCURACY(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_CLOCK_ACCURACY_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_ORIGINAL_SAMP_FREQ(x) (((x) & ((1 << 4) - 1)) << 24)
# define F_ORIGINAL_SAMP_FREQ_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register STTS_BIT_CH01 */
# define STTS_BIT_CH01 3
# define F_SOURCE_NUM_CH0(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH0_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH0(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH0_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH0(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH0_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH1(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH1_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH1(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH1_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH1(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH1_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS1_0(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS1_0_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH23 */
# define STTS_BIT_CH23 4
# define F_SOURCE_NUM_CH2(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH2_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH2(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH2_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH2(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH2_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH3(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH3_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH3(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH3_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH3(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH3_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS3_2(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS3_2_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH45 */
# define STTS_BIT_CH45 5
# define F_SOURCE_NUM_CH4(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH4_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH4(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH4_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH4(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH4_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH5(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH5_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH5(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH5_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH5(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH5_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS5_4(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS5_4_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH67 */
# define STTS_BIT_CH67 6
# define F_SOURCE_NUM_CH6(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH6_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH6(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH6_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH6(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH6_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH7(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH7_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH7(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH7_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH7(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH7_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS7_6(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS7_6_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH89 */
# define STTS_BIT_CH89 7
# define F_SOURCE_NUM_CH8(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH8_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH8(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH8_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH8(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH8_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH9(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH9_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH9(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH9_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH9(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH9_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS9_8(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS9_8_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH1011 */
# define STTS_BIT_CH1011 8
# define F_SOURCE_NUM_CH10(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH10_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH10(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH10_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH10(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH10_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH11(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH11_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH11(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH11_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH11(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH11_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS11_10(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS11_10_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH1213 */
# define STTS_BIT_CH1213 9
# define F_SOURCE_NUM_CH12(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH12_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH12(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH12_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH12(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH12_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH13(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH13_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH13(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH13_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH13(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH13_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS13_12(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS13_12_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH1415 */
# define STTS_BIT_CH1415 10
# define F_SOURCE_NUM_CH14(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH14_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH14(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH14_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH14(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH14_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH15(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH15_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH15(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH15_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH15(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH15_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS15_14(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS15_14_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH1617 */
# define STTS_BIT_CH1617 11
# define F_SOURCE_NUM_CH16(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH16_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH16(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH16_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH16(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH16_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH17(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH17_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH17(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH17_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH17(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH17_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS17_16(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS17_16_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH1819 */
# define STTS_BIT_CH1819 12
# define F_SOURCE_NUM_CH18(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH18_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH18(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH18_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH18(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH18_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH19(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH19_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH19(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH19_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH19(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH19_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS19_18(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS19_18_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH2021 */
# define STTS_BIT_CH2021 13
# define F_SOURCE_NUM_CH20(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH20_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH20(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH20_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH20(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH20_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH21(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH21_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH21(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH21_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH21(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH21_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS21_20(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS21_20_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH2223 */
# define STTS_BIT_CH2223 14
# define F_SOURCE_NUM_CH22(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH22_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH22(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH22_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH22(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH22_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH23(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH23_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH23(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH23_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH23(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH23_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS23_22(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS23_22_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH2425 */
# define STTS_BIT_CH2425 15
# define F_SOURCE_NUM_CH24(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH24_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH24(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH24_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH24(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH24_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH25(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH25_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH25(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH25_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH25(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH25_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS25_24(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS25_24_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH2627 */
# define STTS_BIT_CH2627 16
# define F_SOURCE_NUM_CH26(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH26_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH26(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH26_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH26(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH26_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH27(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH27_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH27(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH27_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH27(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH27_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS27_26(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS27_26_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH2829 */
# define STTS_BIT_CH2829 17
# define F_SOURCE_NUM_CH28(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH28_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH28(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH28_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH28(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH28_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH29(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH29_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH29(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH29_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH29(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH29_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS29_28(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS29_28_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register STTS_BIT_CH3031 */
# define STTS_BIT_CH3031 18
# define F_SOURCE_NUM_CH30(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_NUM_CH30_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_CHANNEL_NUM_CH30(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_CHANNEL_NUM_CH30_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_WORD_LENGTH_CH30(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_WORD_LENGTH_CH30_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_NUM_CH31(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_NUM_CH31_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_CHANNEL_NUM_CH31(x) (((x) & ((1 << 4) - 1)) << 16)
# define F_CHANNEL_NUM_CH31_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
# define F_WORD_LENGTH_CH31(x) (((x) & ((1 << 4) - 1)) << 20)
# define F_WORD_LENGTH_CH31_RD(x) (((x) & (((1 << 4) - 1) << 20)) >> 20)
# define F_VALID_BITS31_30(x) (((x) & ((1 << 2) - 1)) << 24)
# define F_VALID_BITS31_30_RD(x) (((x) & (((1 << 2) - 1) << 24)) >> 24)

/* register SPDIF_CTRL_ADDR */
# define SPDIF_CTRL_ADDR 19
# define F_SPDIF_JITTER_AVG_WIN(x) (((x) & ((1 << 3) - 1)) << 0)
# define F_SPDIF_JITTER_AVG_WIN_RD(x) (((x) & (((1 << 3) - 1) << 0)) >> 0)
# define F_SPDIF_JITTER_THRSH(x) (((x) & ((1 << 8) - 1)) << 3)
# define F_SPDIF_JITTER_THRSH_RD(x) (((x) & (((1 << 8) - 1) << 3)) >> 3)
# define F_SPDIF_FIFO_MID_RANGE(x) (((x) & ((1 << 8) - 1)) << 11)
# define F_SPDIF_FIFO_MID_RANGE_RD(x) (((x) & (((1 << 8) - 1) << 11)) >> 11)
# define F_SPDIF_JITTER_BYPASS(x) (((x) & ((1 << 1) - 1)) << 19)
# define F_SPDIF_JITTER_BYPASS_RD(x) (((x) & (((1 << 1) - 1) << 19)) >> 19)
# define F_SPDIF_AVG_SEL(x) (((x) & ((1 << 1) - 1)) << 20)
# define F_SPDIF_AVG_SEL_RD(x) (((x) & (((1 << 1) - 1) << 20)) >> 20)
# define F_SPDIF_ENABLE(x) (((x) & ((1 << 1) - 1)) << 21)
# define F_SPDIF_ENABLE_RD(x) (((x) & (((1 << 1) - 1) << 21)) >> 21)
# define F_SPDIF_JITTER_STATUS(x) (((x) & ((1 << 4) - 1)) << 22)
# define F_SPDIF_JITTER_STATUS_RD(x) (((x) & (((1 << 4) - 1) << 22)) >> 22)

/* register SPDIF_CH1_CS_3100_ADDR */
# define SPDIF_CH1_CS_3100_ADDR 20
# define F_SPDIF_CH1_ST_STTS_BITS3100(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH1_ST_STTS_BITS3100_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH1_CS_6332_ADDR */
# define SPDIF_CH1_CS_6332_ADDR 21
# define F_SPDIF_CH1_ST_STTS_BITS6332(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH1_ST_STTS_BITS6332_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH1_CS_9564_ADDR */
# define SPDIF_CH1_CS_9564_ADDR 22
# define F_SPDIF_CH1_ST_STTS_BITS9564(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH1_ST_STTS_BITS9564_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH1_CS_12796_ADDR */
# define SPDIF_CH1_CS_12796_ADDR 23
# define F_SPDIF_CH1_ST_STTS_BITS12796(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH1_ST_STTS_BITS12796_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH1_CS_159128_ADDR */
# define SPDIF_CH1_CS_159128_ADDR 24
# define F_SPDIF_CH1_ST_STTS_BITS159128(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH1_ST_STTS_BITS159128_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH1_CS_191160_ADDR */
# define SPDIF_CH1_CS_191160_ADDR 25
# define F_SPDIF_CH1_ST_STTS_BITS191160(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH1_ST_STTS_BITS191160_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH2_CS_3100_ADDR */
# define SPDIF_CH2_CS_3100_ADDR 26
# define F_SPDIF_CH2_ST_STTS_BITS3100(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH2_ST_STTS_BITS3100_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH2_CS_6332_ADDR */
# define SPDIF_CH2_CS_6332_ADDR 27
# define F_SPDIF_CH2_ST_STTS_BITS6332(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH2_ST_STTS_BITS6332_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH2_CS_9564_ADDR */
# define SPDIF_CH2_CS_9564_ADDR 28
# define F_SPDIF_CH2_ST_STTS_BITS9564(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH2_ST_STTS_BITS9564_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH2_CS_12796_ADDR */
# define SPDIF_CH2_CS_12796_ADDR 29
# define F_SPDIF_CH2_ST_STTS_BITS12796(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH2_ST_STTS_BITS12796_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH2_CS_159128_ADDR */
# define SPDIF_CH2_CS_159128_ADDR 30
# define F_SPDIF_CH2_ST_STTS_BITS159128(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH2_ST_STTS_BITS159128_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register SPDIF_CH2_CS_191160_ADDR */
# define SPDIF_CH2_CS_191160_ADDR 31
# define F_SPDIF_CH2_ST_STTS_BITS191160(x) (((x) & ((1 << 32) - 1)) << 0)
# define F_SPDIF_CH2_ST_STTS_BITS191160_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

#endif //SOURCE_AIF_DECODER
