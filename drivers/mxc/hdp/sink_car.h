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
 * sink_car.h
 *
 ******************************************************************************
 */

#ifndef SINK_CAR_H_
#define SINK_CAR_H_

/* register SINK_MHL_HD_CAR */
#define SINK_MHL_HD_CAR 0
#define F_SINK_MHL_HD_VIF_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_MHL_HD_VIF_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_MHL_HD_VIF_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_MHL_HD_VIF_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SINK_MHL_HD_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SINK_MHL_HD_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SINK_MHL_HD_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SINK_MHL_HD_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_SINK_MHL_HD_PHY_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SINK_MHL_HD_PHY_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SINK_MHL_HD_PHY_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SINK_MHL_HD_PHY_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)

/* register SINK_CEC_CAR */
#define SINK_CEC_CAR 1
#define F_SINK_CEC_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_CEC_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_CEC_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_CEC_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SINK_SCDC_CAR */
#define SINK_SCDC_CAR 2
#define F_SCDC_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SCDC_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SCDC_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SCDC_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SCDC_RSTN_EN(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_SCDC_RSTN_EN_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_SCDC_CLK_EN(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_SCDC_CLK_EN_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)

/* register SINK_PKT_CAR */
#define SINK_PKT_CAR 3
#define F_SINK_PKT_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_PKT_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_PKT_SYS_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_PKT_SYS_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SINK_PKT_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SINK_PKT_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SINK_PKT_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SINK_PKT_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SINK_AIF_CAR */
#define SINK_AIF_CAR 4
#define F_SINK_AIF_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_AIF_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_AIF_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_AIF_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SINK_AIF_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SINK_AIF_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SINK_AIF_SYS_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SINK_AIF_SYS_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_SINK_ACR_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SINK_ACR_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SINK_ACR_CHAR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SINK_ACR_CHAR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)

/* register SINK_CIPHER_CAR */
#define SINK_CIPHER_CAR 5
#define F_SINK_CIPHER_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_CIPHER_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_CIPHER_CHAR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_CIPHER_CHAR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SINK_CIPHER_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SINK_CIPHER_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SINK_CIPHER_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SINK_CIPHER_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SINK_CRYPTO_CAR */
#define SINK_CRYPTO_CAR 6
#define F_SINK_CRYPTO_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_CRYPTO_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_CRYPTO_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_CRYPTO_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SINK_VIF_CAR */
#define SINK_VIF_CAR 7
#define F_SINK_VIF_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_VIF_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_VIF_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_VIF_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SINK_AUDIO_CAR */
#define SINK_AUDIO_CAR 8
#define F_ACR_REF_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_ACR_REF_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_ACR_REF_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_ACR_REF_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SPDIF_MCLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SPDIF_MCLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SPDIF_MCLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SPDIF_MCLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_I2S_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_I2S_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_I2S_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_I2S_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)

/* register SINK_PHY_CAR */
#define SINK_PHY_CAR 9
#define F_SINK_XT_PCLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_XT_PCLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_XT_PRESETN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_XT_PRESETN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SINK_DEBUG_CAR */
#define SINK_DEBUG_CAR 10
#define F_SINK_CLOCK_METER_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SINK_CLOCK_METER_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SINK_CLOCK_METER_SYS_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SINK_CLOCK_METER_SYS_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

#endif /* SINK_CAR */

