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
 * aif_pckt2smp.h
 *
 ******************************************************************************
 */

#ifndef AIF_PCKT2SMP_H_
# define AIF_PCKT2SMP_H_

/* register PKT2SMPL_CNTL */
# define PKT2SMPL_CNTL 0
# define F_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_PKT2SMPL_EN(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_PKT2SMPL_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_NUM_OF_I2S_PORTS(x) (((x) & ((1 << 2) - 1)) << 2)
# define F_NUM_OF_I2S_PORTS_RD(x) (((x) & (((1 << 2) - 1) << 2)) >> 2)
# define F_AIF_ERR_MASK(x) (((x) & ((1 << 1) - 1)) << 4)
# define F_AIF_ERR_MASK_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
# define F_AIF_OVERFLOW_MASK(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_AIF_OVERFLOW_MASK_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
# define F_AIF_UNDERFLOW_MASK(x) (((x) & ((1 << 1) - 1)) << 6)
# define F_AIF_UNDERFLOW_MASK_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
# define F_CFG_FORCE_SP(x) (((x) & ((1 << 1) - 1)) << 7)
# define F_CFG_FORCE_SP_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register ACR_CFG */
# define ACR_CFG 1
# define F_ACR_STOP_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_ACR_STOP_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_ACR_FIFO_STATUS_DIS(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_ACR_FIFO_STATUS_DIS_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_ACR_DIS_USE_NEDGE(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_ACR_DIS_USE_NEDGE_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
# define F_ACR_MASTER_CLK_FX_MODE(x) (((x) & ((1 << 2) - 1)) << 3)
# define F_ACR_MASTER_CLK_FX_MODE_RD(x) (((x) & (((1 << 2) - 1) << 3)) >> 3)
# define F_ACR_SW_RESET(x) (((x) & ((1 << 1) - 1)) << 5)
# define F_ACR_SW_RESET_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)

/* register SPDIF_CFG */
# define SPDIF_CFG 2
# define F_SPDIF_SELECTED(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_SPDIF_SELECTED_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register AUDIO_FIFO_PTR_CFG0 */
# define AUDIO_FIFO_PTR_CFG0 3
# define F_AUDIO_FIFO_PTR_EMPTY_LOW(x) (((x) & ((1 << 9) - 1)) << 0)
# define F_AUDIO_FIFO_PTR_EMPTY_LOW_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)
# define F_AUDIO_FIFO_PTR_EMPTY_HIGH(x) (((x) & ((1 << 9) - 1)) << 9)
# define F_AUDIO_FIFO_PTR_EMPTY_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 9)) >> 9)
# define F_AUDIO_FIFO_PTR_LOW_LOW(x) (((x) & ((1 << 9) - 1)) << 18)
# define F_AUDIO_FIFO_PTR_LOW_LOW_RD(x) (((x) & (((1 << 9) - 1) << 18)) >> 18)

/* register AUDIO_FIFO_PTR_CFG1 */
# define AUDIO_FIFO_PTR_CFG1 4
# define F_AUDIO_FIFO_PTR_LOW_HIGH(x) (((x) & ((1 << 9) - 1)) << 0)
# define F_AUDIO_FIFO_PTR_LOW_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)
# define F_AUDIO_FIFO_PTR_BELOW_LOW(x) (((x) & ((1 << 9) - 1)) << 9)
# define F_AUDIO_FIFO_PTR_BELOW_LOW_RD(x) (((x) & (((1 << 9) - 1) << 9)) >> 9)
# define F_AUDIO_FIFO_PTR_BELOW_HIGH(x) (((x) & ((1 << 9) - 1)) << 18)
# define F_AUDIO_FIFO_PTR_BELOW_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 18)) >> 18)

/* register AUDIO_FIFO_PTR_CFG2 */
# define AUDIO_FIFO_PTR_CFG2 5
# define F_AUDIO_FIFO_PTR_NOMINAL_LOW(x) (((x) & ((1 << 9) - 1)) << 0)
# define F_AUDIO_FIFO_PTR_NOMINAL_LOW_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)
# define F_AUDIO_FIFO_PTR_NOMINAL_HIGH(x) (((x) & ((1 << 9) - 1)) << 9)
# define F_AUDIO_FIFO_PTR_NOMINAL_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 9)) >> 9)
# define F_AUDIO_FIFO_PTR_ABOVE_LOW(x) (((x) & ((1 << 9) - 1)) << 18)
# define F_AUDIO_FIFO_PTR_ABOVE_LOW_RD(x) (((x) & (((1 << 9) - 1) << 18)) >> 18)

/* register AUDIO_FIFO_PTR_CFG3 */
# define AUDIO_FIFO_PTR_CFG3 6
# define F_AUDIO_FIFO_PTR_ABOVE_HIGH(x) (((x) & ((1 << 9) - 1)) << 0)
# define F_AUDIO_FIFO_PTR_ABOVE_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)
# define F_AUDIO_FIFO_PTR_HIGH_LOW(x) (((x) & ((1 << 9) - 1)) << 9)
# define F_AUDIO_FIFO_PTR_HIGH_LOW_RD(x) (((x) & (((1 << 9) - 1) << 9)) >> 9)
# define F_AUDIO_FIFO_PTR_HIGH_HIGH(x) (((x) & ((1 << 9) - 1)) << 18)
# define F_AUDIO_FIFO_PTR_HIGH_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 18)) >> 18)

/* register AUDIO_FIFO_PTR_CFG4 */
# define AUDIO_FIFO_PTR_CFG4 7
# define F_AUDIO_FIFO_PTR_FULL_LOW(x) (((x) & ((1 << 9) - 1)) << 0)
# define F_AUDIO_FIFO_PTR_FULL_LOW_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)
# define F_AUDIO_FIFO_PTR_FULL_HIGH(x) (((x) & ((1 << 9) - 1)) << 9)
# define F_AUDIO_FIFO_PTR_FULL_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 9)) >> 9)

/* register AUDIO_FIFO_PTR_CFG5 */
# define AUDIO_FIFO_PTR_CFG5 8
# define F_AUDIO_FIFO_PTR_IDLE_LOW(x) (((x) & ((1 << 9) - 1)) << 0)
# define F_AUDIO_FIFO_PTR_IDLE_LOW_RD(x) (((x) & (((1 << 9) - 1) << 0)) >> 0)
# define F_AUDIO_FIFO_PTR_IDLE_HIGH(x) (((x) & ((1 << 9) - 1)) << 9)
# define F_AUDIO_FIFO_PTR_IDLE_HIGH_RD(x) (((x) & (((1 << 9) - 1) << 9)) >> 9)

/* register AIF_INT_STTS */
# define AIF_INT_STTS 9
# define F_AIF_ERR_STATUS(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_AIF_ERR_STATUS_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_AIF_OVERFLOW_STATUS(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_AIF_OVERFLOW_STATUS_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_AIF_UNDERFLOW_STATUS(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_AIF_UNDERFLOW_STATUS_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)

/* register AIF_ACR_N_ST */
# define AIF_ACR_N_ST 10
# define F_ACR_N(x) (((x) & ((1 << 20) - 1)) << 0)
# define F_ACR_N_RD(x) (((x) & (((1 << 20) - 1) << 0)) >> 0)

/* register AIF_ACR_CTS_ST */
# define AIF_ACR_CTS_ST 11
# define F_ACR_CTS(x) (((x) & ((1 << 20) - 1)) << 0)
# define F_ACR_CTS_RD(x) (((x) & (((1 << 20) - 1) << 0)) >> 0)

/* register AIF_ACR_N_OFST_CFG */
# define AIF_ACR_N_OFST_CFG 12
# define F_ACR_N_OFFSET(x) (((x) & ((1 << 24) - 1)) << 0)
# define F_ACR_N_OFFSET_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

/* register AIF_ACR_CTS_OFST_CFG */
# define AIF_ACR_CTS_OFST_CFG 13
# define F_ACR_CTS_OFFSET(x) (((x) & ((1 << 24) - 1)) << 0)
# define F_ACR_CTS_OFFSET_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

#endif
