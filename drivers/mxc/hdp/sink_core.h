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
 * sink_core.h
 *
 ******************************************************************************
 */

#ifndef SINK_CORE_H_
#define SINK_CORE_H_

/* register SCDC_INT_MSK */
#define SCDC_INT_MSK 0
#define F_SCDC_INT_MASK(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_SCDC_INT_MASK_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SCDC_INT_STS */
#define SCDC_INT_STS 1
#define F_SCDC_INT_STATUS(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_SCDC_INT_STATUS_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register PHY_IF_LANE0_CFG */
#define PHY_IF_LANE0_CFG 2
#define F_SINK_PHY_DATA0_DATA_REVERSE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_PHY_DATA0_DATA_REVERSE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_PHY_DATA0_CHAR_REVERSE(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_PHY_DATA0_CHAR_REVERSE_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SINK_PHY_CHAR0_SWAP(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SINK_PHY_CHAR0_SWAP_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SINK_PHY_LANE0(x) (((x) & ((1 << 2) - 1)) << 3)
#define F_SINK_PHY_LANE0_RD(x) (((x) & (((1 << 2) - 1) << 3)) >> 3)

/* register PHY_IF_LANE1_CFG */
#define PHY_IF_LANE1_CFG 3
#define F_SINK_PHY_DATA1_DATA_REVERSE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_PHY_DATA1_DATA_REVERSE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_PHY_DATA1_CHAR_REVERSE(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_PHY_DATA1_CHAR_REVERSE_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SINK_PHY_CHAR1_SWAP(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SINK_PHY_CHAR1_SWAP_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SINK_PHY_LANE1(x) (((x) & ((1 << 2) - 1)) << 3)
#define F_SINK_PHY_LANE1_RD(x) (((x) & (((1 << 2) - 1) << 3)) >> 3)

/* register PHY_IF_LANE2_CFG */
#define PHY_IF_LANE2_CFG 4
#define F_SINK_PHY_DATA2_DATA_REVERSE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_PHY_DATA2_DATA_REVERSE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_PHY_DATA2_CHAR_REVERSE(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_PHY_DATA2_CHAR_REVERSE_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SINK_PHY_CHAR2_SWAP(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SINK_PHY_CHAR2_SWAP_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SINK_PHY_LANE2(x) (((x) & ((1 << 2) - 1)) << 3)
#define F_SINK_PHY_LANE2_RD(x) (((x) & (((1 << 2) - 1) << 3)) >> 3)

/* register TOP_ST */
#define TOP_ST 5
#define F_SINK_5V(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_5V_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register TOP_CFG */
#define TOP_CFG 6
#define F_SINK_HPD(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_HPD_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SCDC_FILTER_CFG */
#define SCDC_FILTER_CFG 7
#define F_SCDC_FILTER_BYPASS(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SCDC_FILTER_BYPASS_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SCDC_FILTER_VALID_WIDTH(x) (((x) & ((1 << 12) - 1)) << 1)
#define F_SCDC_FILTER_VALID_WIDTH_RD(x) (((x) & (((1 << 12) - 1) << 1)) >> 1)
#define F_SCDC_FILTER_GLITCH_WIDTH(x) (((x) & ((1 << 8) - 1)) << 13)
#define F_SCDC_FILTER_GLITCH_WIDTH_RD(x) (((x) & (((1 << 8) - 1) << 13)) >> 13)

#endif /* SINK_CORE */

