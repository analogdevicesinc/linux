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
 * source_phy.h
 *
 ******************************************************************************
 */

#ifndef SOURCE_PHY_H_
# define SOURCE_PHY_H_

/* register SHIFT_PATTERN_IN_3_0 */
# define SHIFT_PATTERN_IN_3_0 0
# define F_SOURCE_PHY_SHIFT_PATTERN0(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SOURCE_PHY_SHIFT_PATTERN0_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
# define F_SOURCE_PHY_SHIFT_PATTERN1(x) (((x) & ((1 << 8) - 1)) << 8)
# define F_SOURCE_PHY_SHIFT_PATTERN1_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
# define F_SOURCE_PHY_SHIFT_PATTERN2(x) (((x) & ((1 << 8) - 1)) << 16)
# define F_SOURCE_PHY_SHIFT_PATTERN2_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
# define F_SOURCE_PHY_SHIFT_PATTERN3(x) (((x) & ((1 << 8) - 1)) << 24)
# define F_SOURCE_PHY_SHIFT_PATTERN3_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register SHIFT_PATTERN_IN_4_7 */
# define SHIFT_PATTERN_IN_4_7 1
# define F_SOURCE_PHY_SHIFT_PATTERN4(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SOURCE_PHY_SHIFT_PATTERN4_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
# define F_SOURCE_PHY_SHIFT_PATTERN5(x) (((x) & ((1 << 8) - 1)) << 8)
# define F_SOURCE_PHY_SHIFT_PATTERN5_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
# define F_SOURCE_PHY_SHIFT_PATTERN6(x) (((x) & ((1 << 8) - 1)) << 16)
# define F_SOURCE_PHY_SHIFT_PATTERN6_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
# define F_SOURCE_PHY_SHIFT_PATTERN7(x) (((x) & ((1 << 8) - 1)) << 24)
# define F_SOURCE_PHY_SHIFT_PATTERN7_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register SHIFT_PATTERN_IN9_8 */
# define SHIFT_PATTERN_IN9_8 2
# define F_SOURCE_PHY_SHIFT_PATTERN8(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SOURCE_PHY_SHIFT_PATTERN8_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
# define F_SOURCE_PHY_SHIFT_PATTERN9(x) (((x) & ((1 << 8) - 1)) << 8)
# define F_SOURCE_PHY_SHIFT_PATTERN9_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
# define F_SOURCE_PHY_SHIFT_LOAD(x) (((x) & ((1 << 1) - 1)) << 16)
# define F_SOURCE_PHY_SHIFT_LOAD_RD(x) (((x) & (((1 << 1) - 1) << 16)) >> 16)
# define F_SOURCE_PHY_SHIFT_EN(x) (((x) & ((1 << 1) - 1)) << 17)
# define F_SOURCE_PHY_SHIFT_EN_RD(x) (((x) & (((1 << 1) - 1) << 17)) >> 17)
# define F_SOURCE_PHY_SHIFT_REPETITION(x) (((x) & ((1 << 3) - 1)) << 18)
# define F_SOURCE_PHY_SHIFT_REPETITION_RD(x) (((x) & (((1 << 3) - 1) << 18)) >> 18)

/* register PRBS_CNTRL */
# define PRBS_CNTRL 3
# define F_SOURCE_PHY_PRBS0_MODE(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_SOURCE_PHY_PRBS0_MODE_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)
# define F_SOURCE_PHY_PRBS0_OUT_MODE(x) (((x) & ((1 << 2) - 1)) << 2)
# define F_SOURCE_PHY_PRBS0_OUT_MODE_RD(x) (((x) & (((1 << 2) - 1) << 2)) >> 2)
# define F_SOURCE_PHY_PRBS1_MODE(x) (((x) & ((1 << 2) - 1)) << 4)
# define F_SOURCE_PHY_PRBS1_MODE_RD(x) (((x) & (((1 << 2) - 1) << 4)) >> 4)
# define F_SOURCE_PHY_PRBS1_OUT_MODE(x) (((x) & ((1 << 2) - 1)) << 6)
# define F_SOURCE_PHY_PRBS1_OUT_MODE_RD(x) (((x) & (((1 << 2) - 1) << 6)) >> 6)
# define F_SOURCE_PHY_PRBS2_MODE(x) (((x) & ((1 << 2) - 1)) << 8)
# define F_SOURCE_PHY_PRBS2_MODE_RD(x) (((x) & (((1 << 2) - 1) << 8)) >> 8)
# define F_SOURCE_PHY_PRBS2_OUT_MODE(x) (((x) & ((1 << 2) - 1)) << 10)
# define F_SOURCE_PHY_PRBS2_OUT_MODE_RD(x) (((x) & (((1 << 2) - 1) << 10)) >> 10)
# define F_SOURCE_PHY_PRBS3_MODE(x) (((x) & ((1 << 2) - 1)) << 12)
# define F_SOURCE_PHY_PRBS3_MODE_RD(x) (((x) & (((1 << 2) - 1) << 12)) >> 12)
# define F_SOURCE_PHY_PRBS3_OUT_MODE(x) (((x) & ((1 << 2) - 1)) << 14)
# define F_SOURCE_PHY_PRBS3_OUT_MODE_RD(x) (((x) & (((1 << 2) - 1) << 14)) >> 14)

/* register PRBS_ERR_INSERTION */
# define PRBS_ERR_INSERTION 4
# define F_ADD_ERROR0(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_ADD_ERROR0_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_NUMBER_OF_ERRORS0(x) (((x) & ((1 << 5) - 1)) << 1)
# define F_NUMBER_OF_ERRORS0_RD(x) (((x) & (((1 << 5) - 1) << 1)) >> 1)
# define F_ADD_ERROR1(x) (((x) & ((1 << 1) - 1)) << 6)
# define F_ADD_ERROR1_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
# define F_NUMBER_OF_ERRORS1(x) (((x) & ((1 << 5) - 1)) << 7)
# define F_NUMBER_OF_ERRORS1_RD(x) (((x) & (((1 << 5) - 1) << 7)) >> 7)
# define F_ADD_ERROR2(x) (((x) & ((1 << 1) - 1)) << 12)
# define F_ADD_ERROR2_RD(x) (((x) & (((1 << 1) - 1) << 12)) >> 12)
# define F_NUMBER_OF_ERRORS2(x) (((x) & ((1 << 5) - 1)) << 13)
# define F_NUMBER_OF_ERRORS2_RD(x) (((x) & (((1 << 5) - 1) << 13)) >> 13)
# define F_ADD_ERROR3(x) (((x) & ((1 << 1) - 1)) << 18)
# define F_ADD_ERROR3_RD(x) (((x) & (((1 << 1) - 1) << 18)) >> 18)
# define F_NUMBER_OF_ERRORS3(x) (((x) & ((1 << 5) - 1)) << 19)
# define F_NUMBER_OF_ERRORS3_RD(x) (((x) & (((1 << 5) - 1) << 19)) >> 19)

/* register LANES_CONFIG */
# define LANES_CONFIG 5
# define F_SOURCE_PHY_LANE0_SWAP(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_SOURCE_PHY_LANE0_SWAP_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)
# define F_SOURCE_PHY_LANE1_SWAP(x) (((x) & ((1 << 2) - 1)) << 2)
# define F_SOURCE_PHY_LANE1_SWAP_RD(x) (((x) & (((1 << 2) - 1) << 2)) >> 2)
# define F_SOURCE_PHY_LANE2_SWAP(x) (((x) & ((1 << 2) - 1)) << 4)
# define F_SOURCE_PHY_LANE2_SWAP_RD(x) (((x) & (((1 << 2) - 1) << 4)) >> 4)
# define F_SOURCE_PHY_LANE3_SWAP(x) (((x) & ((1 << 2) - 1)) << 6)
# define F_SOURCE_PHY_LANE3_SWAP_RD(x) (((x) & (((1 << 2) - 1) << 6)) >> 6)
# define F_SOURCE_PHY_LANE0_LSB_MSB(x) (((x) & ((1 << 1) - 1)) << 8)
# define F_SOURCE_PHY_LANE0_LSB_MSB_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
# define F_SOURCE_PHY_LANE1_LSB_MSB(x) (((x) & ((1 << 1) - 1)) << 9)
# define F_SOURCE_PHY_LANE1_LSB_MSB_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)
# define F_SOURCE_PHY_LANE2_LSB_MSB(x) (((x) & ((1 << 1) - 1)) << 10)
# define F_SOURCE_PHY_LANE2_LSB_MSB_RD(x) (((x) & (((1 << 1) - 1) << 10)) >> 10)
# define F_SOURCE_PHY_LANE3_LSB_MSB(x) (((x) & ((1 << 1) - 1)) << 11)
# define F_SOURCE_PHY_LANE3_LSB_MSB_RD(x) (((x) & (((1 << 1) - 1) << 11)) >> 11)
# define F_SOURCE_PHY_AUX_SPARE(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_PHY_AUX_SPARE_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)
# define F_SOURCE_PHY_LANE0_POLARITY(x) (((x) & ((1 << 1) - 1)) << 16)
# define F_SOURCE_PHY_LANE0_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 16)) >> 16)
# define F_SOURCE_PHY_LANE1_POLARITY(x) (((x) & ((1 << 1) - 1)) << 17)
# define F_SOURCE_PHY_LANE1_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 17)) >> 17)
# define F_SOURCE_PHY_LANE2_POLARITY(x) (((x) & ((1 << 1) - 1)) << 18)
# define F_SOURCE_PHY_LANE2_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 18)) >> 18)
# define F_SOURCE_PHY_LANE3_POLARITY(x) (((x) & ((1 << 1) - 1)) << 19)
# define F_SOURCE_PHY_LANE3_POLARITY_RD(x) (((x) & (((1 << 1) - 1) << 19)) >> 19)
# define F_SOURCE_PHY_DATA_DEL_EN(x) (((x) & ((1 << 1) - 1)) << 20)
# define F_SOURCE_PHY_DATA_DEL_EN_RD(x) (((x) & (((1 << 1) - 1) << 20)) >> 20)
# define F_SOURCE_PHY_COMB_BYPASS(x) (((x) & ((1 << 1) - 1)) << 21)
# define F_SOURCE_PHY_COMB_BYPASS_RD(x) (((x) & (((1 << 1) - 1) << 21)) >> 21)
# define F_SOURCE_PHY_20_10(x) (((x) & ((1 << 1) - 1)) << 22)
# define F_SOURCE_PHY_20_10_RD(x) (((x) & (((1 << 1) - 1) << 22)) >> 22)

/* register PHY_DATA_SEL */
# define PHY_DATA_SEL 6
# define F_SOURCE_PHY_DATA_SEL(x) (((x) & ((1 << 3) - 1)) << 0)
# define F_SOURCE_PHY_DATA_SEL_RD(x) (((x) & (((1 << 3) - 1) << 0)) >> 0)
# define F_SOURCE_PHY_MHDP_SEL(x) (((x) & ((1 << 2) - 1)) << 3)
# define F_SOURCE_PHY_MHDP_SEL_RD(x) (((x) & (((1 << 2) - 1) << 3)) >> 3)

/* register LANES_DEL_VAL */
# define LANES_DEL_VAL 7
# define F_SOURCE_PHY_LANE0_DEL_VAL(x) (((x) & ((1 << 4) - 1)) << 0)
# define F_SOURCE_PHY_LANE0_DEL_VAL_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
# define F_SOURCE_PHY_LANE1_DEL_VAL(x) (((x) & ((1 << 4) - 1)) << 4)
# define F_SOURCE_PHY_LANE1_DEL_VAL_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)
# define F_SOURCE_PHY_LANE2_DEL_VAL(x) (((x) & ((1 << 4) - 1)) << 8)
# define F_SOURCE_PHY_LANE2_DEL_VAL_RD(x) (((x) & (((1 << 4) - 1) << 8)) >> 8)
# define F_SOURCE_PHY_LANE3_DEL_VAL(x) (((x) & ((1 << 4) - 1)) << 12)
# define F_SOURCE_PHY_LANE3_DEL_VAL_RD(x) (((x) & (((1 << 4) - 1) << 12)) >> 12)

#endif //SOURCE_PHY
