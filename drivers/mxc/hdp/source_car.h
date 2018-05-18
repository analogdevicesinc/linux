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
 * source_car.h
 *
 ******************************************************************************
 */

#ifndef SOURCE_CAR_H_
#define SOURCE_CAR_H_

/* register SOURCE_HDTX_CAR */
#define SOURCE_HDTX_CAR 0
#define F_SOURCE_HDTX_PIXEL_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_HDTX_PIXEL_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_HDTX_PIXEL_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_HDTX_PIXEL_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_HDTX_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_HDTX_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_HDTX_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_HDTX_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_SOURCE_HDTX_PHY_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SOURCE_HDTX_PHY_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SOURCE_HDTX_PHY_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SOURCE_HDTX_PHY_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_SOURCE_HDTX_PHY_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_SOURCE_HDTX_PHY_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_SOURCE_HDTX_PHY_CHAR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_SOURCE_HDTX_PHY_CHAR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register SOURCE_DPTX_CAR */
#define SOURCE_DPTX_CAR 1
#define F_SOURCE_CFG_DPTX_VIF_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CFG_DPTX_VIF_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CFG_DPTX_VIF_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CFG_DPTX_VIF_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_DPTX_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_DPTX_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_DPTX_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_DPTX_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_SOURCE_AUX_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SOURCE_AUX_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SOURCE_AUX_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SOURCE_AUX_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_SOUrcE_DPTX_PHY_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_SOUrcE_DPTX_PHY_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_SOUrcE_DPTX_PHY_CHAR_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_SOUrcE_DPTX_PHY_CHAR_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
#define F_SOUrcE_DPTX_PHY_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_SOUrcE_DPTX_PHY_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_SOUrcE_DPTX_PHY_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_SOUrcE_DPTX_PHY_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)
#define F_SOUrcE_DPTX_FRMR_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 10)
#define F_SOUrcE_DPTX_FRMR_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 10)) >> 10)
#define F_SOUrcE_DPTX_FRMR_DATA_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 11)
#define F_SOUrcE_DPTX_FRMR_DATA_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 11)) >> 11)

/* register SOURCE_PHY_CAR */
#define SOURCE_PHY_CAR 2
#define F_SOURCE_PHY_DATA_OUT_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_PHY_DATA_OUT_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_PHY_DATA_OUT_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_PHY_DATA_OUT_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_PHY_CHAR_OUT_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_PHY_CHAR_OUT_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_PHY_CHAR_OUT_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_PHY_CHAR_OUT_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SOURCE_CEC_CAR */
#define SOURCE_CEC_CAR 3
#define F_SOURCE_CEC_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CEC_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CEC_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CEC_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SOURCE_CBUS_CAR */
#define SOURCE_CBUS_CAR 4
#define F_SOURCE_CBUS_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CBUS_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CBUS_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CBUS_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SOURCE_PKT_CAR */
#define SOURCE_PKT_CAR 6
#define F_SOURCE_PKT_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_PKT_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_PKT_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_PKT_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_PKT_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_PKT_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_PKT_SYS_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_PKT_SYS_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SOURCE_AIF_CAR */
#define SOURCE_AIF_CAR 7
#define F_SOURCE_AIF_PKT_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_AIF_PKT_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_AIF_PKT_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_AIF_PKT_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_AIF_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_AIF_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_AIF_SYS_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_AIF_SYS_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_SOURCE_SPDIF_CDR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SOURCE_SPDIF_CDR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SOURCE_SPDIF_CDR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SOURCE_SPDIF_CDR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_SOURCE_SPDIF_MCLK_EN(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_SOURCE_SPDIF_MCLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_SOURCE_SPDIF_MCLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_SOURCE_SPDIF_MCLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register SOURCE_CIPHER_CAR */
#define SOURCE_CIPHER_CAR 8
#define F_SOURCE_CIPHER_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CIPHER_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CIPHER_CHAR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CIPHER_CHAR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_CIPHER_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_CIPHER_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SOURCE_CRYPTO_CAR */
#define SOURCE_CRYPTO_CAR 9
#define F_SOURCE_CRYPTO_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CRYPTO_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CRYPTO_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CRYPTO_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

#endif /* SOURCE_CAR */
