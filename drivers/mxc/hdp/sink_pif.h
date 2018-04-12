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
 * sink_pif.h
 *
 ******************************************************************************
 */

#ifndef SINK_PIF_H_
#define SINK_PIF_H_

/* register PKT_INFO_TYPE_CFG1 */
#define PKT_INFO_TYPE_CFG1 0
#define F_INFO_TYPE1(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_INFO_TYPE1_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_INFO_TYPE2(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_INFO_TYPE2_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_INFO_TYPE3(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_INFO_TYPE3_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
#define F_INFO_TYPE4(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_INFO_TYPE4_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register PKT_INFO_TYPE_CFG2 */
#define PKT_INFO_TYPE_CFG2 1
#define F_INFO_TYPE5(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_INFO_TYPE5_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_INFO_TYPE6(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_INFO_TYPE6_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_INFO_TYPE7(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_INFO_TYPE7_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
#define F_INFO_TYPE8(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_INFO_TYPE8_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register PKT_INFO_TYPE_CFG3 */
#define PKT_INFO_TYPE_CFG3 2
#define F_INFO_TYPE9(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_INFO_TYPE9_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_INFO_TYPE10(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_INFO_TYPE10_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_INFO_TYPE11(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_INFO_TYPE11_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
#define F_INFO_TYPE12(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_INFO_TYPE12_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register PKT_INFO_TYPE_CFG4 */
#define PKT_INFO_TYPE_CFG4 3
#define F_INFO_TYPE13(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_INFO_TYPE13_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_INFO_TYPE14(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_INFO_TYPE14_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_INFO_TYPE15(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_INFO_TYPE15_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)
#define F_INFO_TYPE16(x) (((x) & ((1 << 8) - 1)) << 24)
#define F_INFO_TYPE16_RD(x) (((x) & (((1 << 8) - 1) << 24)) >> 24)

/* register PKT_INFO_CTRL */
#define PKT_INFO_CTRL 4
#define F_PACKET_RDN_WR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_PACKET_RDN_WR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_PACKET_NUM(x) (((x) & ((1 << 4) - 1)) << 1)
#define F_PACKET_NUM_RD(x) (((x) & (((1 << 4) - 1) << 1)) >> 1)

/* register PKT_INFO_HEADER */
#define PKT_INFO_HEADER 7
#define F_PKT_MEMORY_HEADER(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_HEADER_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INFO_DATA1 */
#define PKT_INFO_DATA1 8
#define F_PKT_MEMORY_DATA1(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_DATA1_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INFO_DATA2 */
#define PKT_INFO_DATA2 9
#define F_PKT_MEMORY_DATA2(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_DATA2_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INFO_DATA3 */
#define PKT_INFO_DATA3 10
#define F_PKT_MEMORY_DATA3(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_DATA3_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INFO_DATA4 */
#define PKT_INFO_DATA4 11
#define F_PKT_MEMORY_DATA4(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_DATA4_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INFO_DATA5 */
#define PKT_INFO_DATA5 12
#define F_PKT_MEMORY_DATA5(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_DATA5_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INFO_DATA6 */
#define PKT_INFO_DATA6 13
#define F_PKT_MEMORY_DATA6(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_DATA6_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INFO_DATA7 */
#define PKT_INFO_DATA7 14
#define F_PKT_MEMORY_DATA7(x) (((x) & ((1 << 32) - 1)) << 0)
#define F_PKT_MEMORY_DATA7_RD(x) (((x) & (((1 << 32) - 1) << 0)) >> 0)

/* register PKT_INT_STATUS */
#define PKT_INT_STATUS 15
#define F_PKT_RDY_STATUS(x) (((x) & ((1 << 17) - 1)) << 0)
#define F_PKT_RDY_STATUS_RD(x) (((x) & (((1 << 17) - 1) << 0)) >> 0)

/* register PKT_INT_MASK */
#define PKT_INT_MASK 16
#define F_PKT_RDY_MASK(x) (((x) & ((1 << 17) - 1)) << 0)
#define F_PKT_RDY_MASK_RD(x) (((x) & (((1 << 17) - 1) << 0)) >> 0)

/* register PKT_TRANS_CTRL */
#define PKT_TRANS_CTRL 17
#define F_PKT_TRANS_MASK_ERR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_PKT_TRANS_MASK_ERR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

#endif /* SINK_PIF */
