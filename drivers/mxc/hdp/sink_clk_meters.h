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
 * sink_clk_meters.h
 *
 ******************************************************************************
 */

#ifndef SINK_CLK_METERS_H_
#define SINK_CLK_METERS_H_

/* register CLK_METER_REF_CFG */
#define CLK_METER_REF_CFG 0
#define F_REF_CYCLES_SINK_REF_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_REF_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_REF_CLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_REF_CLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_PHY_CHAR_CFG */
#define CLK_METER_PHY_CHAR_CFG 1
#define F_REF_CYCLES_SINK_PHY_CHAR_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_PHY_CHAR_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_PHY_CHAR_CLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_PHY_CHAR_CLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_PHY_DATA_CFG */
#define CLK_METER_PHY_DATA_CFG 2
#define F_REF_CYCLES_SINK_PHY_DATA_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_PHY_DATA_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_PHY_DATA_CLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_PHY_DATA_CLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_PHY_PIXEL_CFG */
#define CLK_METER_PHY_PIXEL_CFG 3
#define F_REF_CYCLES_SINK_PHY_PIXEL_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_PHY_PIXEL_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_PHY_PIXEL_CLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_PHY_PIXEL_CLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_SPDIF_CFG */
#define CLK_METER_SPDIF_CFG 4
#define F_REF_CYCLES_SINK_SPDIF_MCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_SPDIF_MCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_SPDIF_MCLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_SPDIF_MCLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_I2S_CFG */
#define CLK_METER_I2S_CFG 5
#define F_REF_CYCLES_SINK_I2S_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_I2S_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_I2S_CLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_I2S_CLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_PCLK_CFG */
#define CLK_METER_PCLK_CFG 6
#define F_REF_CYCLES_SINK_PCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_PCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_PCLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_PCLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_SCLK_CFG */
#define CLK_METER_SCLK_CFG 7
#define F_REF_CYCLES_SINK_SCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_SCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_SCLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_SCLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_CCLK_CFG */
#define CLK_METER_CCLK_CFG 8
#define F_REF_CYCLES_SINK_CCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_CCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_CCLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_CCLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_TRNG_CFG */
#define CLK_METER_TRNG_CFG 9
#define F_REF_CYCLES_SINK_TRNG_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_REF_CYCLES_SINK_TRNG_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_TOLERANCE_SINK_TRNG_CLK(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_MEAS_TOLERANCE_SINK_TRNG_CLK_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CLK_METER_REF_ST */
#define CLK_METER_REF_ST 10
#define F_MEAS_CYCLES_SINK_REF_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_REF_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_REF_CLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_REF_CLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_PHY_CHAR_ST */
#define CLK_METER_PHY_CHAR_ST 11
#define F_MEAS_CYCLES_SINK_PHY_CHAR_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_PHY_CHAR_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_PHY_CHAR_CLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_PHY_CHAR_CLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_PHY_DATA_ST */
#define CLK_METER_PHY_DATA_ST 12
#define F_MEAS_CYCLES_SINK_PHY_DATA_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_PHY_DATA_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_PHY_DATA_CLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_PHY_DATA_CLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_PHY_PIXEL_ST */
#define CLK_METER_PHY_PIXEL_ST 13
#define F_MEAS_CYCLES_SINK_PHY_PIXEL_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_PHY_PIXEL_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_PHY_PIXEL_CLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_PHY_PIXEL_CLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_SPDIF_ST */
#define CLK_METER_SPDIF_ST 14
#define F_MEAS_CYCLES_SINK_SPDIF_MCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_SPDIF_MCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_SPDIF_MCLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_SPDIF_MCLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_I2S_ST */
#define CLK_METER_I2S_ST 15
#define F_MEAS_CYCLES_SINK_I2S_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_I2S_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_I2S_CLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_I2S_CLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_PCLK_ST */
#define CLK_METER_PCLK_ST 16
#define F_MEAS_CYCLES_SINK_PCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_PCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_PCLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_PCLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_SCLK_ST */
#define CLK_METER_SCLK_ST 17
#define F_MEAS_CYCLES_SINK_SCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_SCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_SCLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_SCLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_CCLK_ST */
#define CLK_METER_CCLK_ST 18
#define F_MEAS_CYCLES_SINK_CCLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_CCLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_CCLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_CCLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_TRNG_ST */
#define CLK_METER_TRNG_ST 19
#define F_MEAS_CYCLES_SINK_TRNG_CLK(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_MEAS_CYCLES_SINK_TRNG_CLK_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_MEAS_STABLE_SINK_TRNG_CLK(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_MEAS_STABLE_SINK_TRNG_CLK_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CLK_METER_INT_MASK */
#define CLK_METER_INT_MASK 20
#define F_MEAS_MASK_SINK_REF_CLK(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_MEAS_MASK_SINK_REF_CLK_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_MEAS_MASK_SINK_PHY_CHAR_CLK(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_MEAS_MASK_SINK_PHY_CHAR_CLK_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_MEAS_MASK_SINK_PHY_DATA_CLK(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_MEAS_MASK_SINK_PHY_DATA_CLK_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_MEAS_MASK_SINK_PHY_PIXEL_CLK(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_MEAS_MASK_SINK_PHY_PIXEL_CLK_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_MEAS_MASK_SINK_SPDIF_MCLK(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_MEAS_MASK_SINK_SPDIF_MCLK_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_MEAS_MASK_SINK_I2S_CLK(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_MEAS_MASK_SINK_I2S_CLK_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_MEAS_MASK_SINK_PCLK(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_MEAS_MASK_SINK_PCLK_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_MEAS_MASK_SINK_SCLK(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_MEAS_MASK_SINK_SCLK_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
#define F_MEAS_MASK_SINK_CCLK(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_MEAS_MASK_SINK_CCLK_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_MEAS_MASK_SINK_TRNG_CLK(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_MEAS_MASK_SINK_TRNG_CLK_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)

/* register CLK_METER_INT_ST */
#define CLK_METER_INT_ST 21
#define F_MEAS_STATUS_SINK_REF_CLK(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_MEAS_STATUS_SINK_REF_CLK_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_MEAS_STATUS_SINK_PHY_CHAR_CLK(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_MEAS_STATUS_SINK_PHY_CHAR_CLK_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_MEAS_STATUS_SINK_PHY_DATA_CLK(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_MEAS_STATUS_SINK_PHY_DATA_CLK_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_MEAS_STATUS_SINK_PHY_PIXEL_CLK(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_MEAS_STATUS_SINK_PHY_PIXEL_CLK_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_MEAS_STATUS_SINK_SPDIF_MCLK(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_MEAS_STATUS_SINK_SPDIF_MCLK_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_MEAS_STATUS_SINK_I2S_CLK(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_MEAS_STATUS_SINK_I2S_CLK_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_MEAS_STATUS_SINK_PCLK(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_MEAS_STATUS_SINK_PCLK_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_MEAS_STATUS_SINK_SCLK(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_MEAS_STATUS_SINK_SCLK_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
#define F_MEAS_STATUS_SINK_CCLK(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_MEAS_STATUS_SINK_CCLK_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_MEAS_STATUS_SINK_TRNG_CLK(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_MEAS_STATUS_SINK_TRNG_CLK_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)

#endif /* SINK_CLK_METERS */

