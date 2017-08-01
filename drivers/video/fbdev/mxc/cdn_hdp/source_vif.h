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
 * source_vif.h
 *
 ******************************************************************************
 */

#ifndef SOURCE_VIF_H_
# define SOURCE_VIF_H_

/* register BND_HSYNC2VSYNC */
# define BND_HSYNC2VSYNC 0
# define F_IP_DTCT_WIN(x) (((x) & ((1 << 12) - 1)) << 0)
# define F_IP_DTCT_WIN_RD(x) (((x) & (((1 << 12) - 1) << 0)) >> 0)
# define F_IP_DET_EN(x) (((x) & ((1 << 1) - 1)) << 12)
# define F_IP_DET_EN_RD(x) (((x) & (((1 << 1) - 1) << 12)) >> 12)
# define F_IP_VIF_BYPASS(x) (((x) & ((1 << 1) - 1)) << 13)
# define F_IP_VIF_BYPASS_RD(x) (((x) & (((1 << 1) - 1) << 13)) >> 13)

/* register HSYNC2VSYNC_F1_L1 */
# define HSYNC2VSYNC_F1_L1 1
# define F_IP_DTCT_HSYNC2VSYNC_F1(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_IP_DTCT_HSYNC2VSYNC_F1_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register HSYNC2VSYNC_F2_L1 */
# define HSYNC2VSYNC_F2_L1 2
# define F_IP_DTCT_HSYNC2VSYNC_F2(x) (((x) & ((1 << 16) - 1)) << 0)
# define F_IP_DTCT_HSYNC2VSYNC_F2_RD(x) (((x) & (((1 << 16) - 1) << 0)) >> 0)

/* register HSYNC2VSYNC_STATUS */
# define HSYNC2VSYNC_STATUS 3
# define F_IP_DTCT_ERR(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_IP_DTCT_ERR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_IP_DCT_IP(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_IP_DCT_IP_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_IP_DTCT_VJITTER(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_IP_DTCT_VJITTER_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_IP_DTCT_HJITTER(x) (((x) & ((1 << 1) - 1)) << 3)
# define F_IP_DTCT_HJITTER_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register HSYNC2VSYNC_POL_CTRL */
# define HSYNC2VSYNC_POL_CTRL 4
# define F_VPOL(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_VPOL_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_HPOL(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_HPOL_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_VIF_AUTO_MODE(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_VIF_AUTO_MODE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

#endif //SOURCE_VIF
