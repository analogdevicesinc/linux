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
 * apb_cfg.h
 *
 ******************************************************************************
 */

#ifndef APB_CFG_H_
# define APB_CFG_H_

/* register APB_CTRL */
# define APB_CTRL 0
# define F_APB_XT_RESET(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_APB_XT_RESET_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_APB_DRAM_PATH(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_APB_DRAM_PATH_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
# define F_APB_IRAM_PATH(x) (((x) & ((1 << 1) - 1)) << 2)
# define F_APB_IRAM_PATH_RD(x) (((x) & (((1 << 1) 1) << 2)) >> 2)

/* register XT_INT_CTRL */
# define XT_INT_CTRL 1
# define F_XT_INT_POLARITY(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_XT_INT_POLARITY_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)

/* register MAILBOX_FULL_ADDR */
# define MAILBOX_FULL_ADDR 2
# define F_MAILBOX_FULL(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_MAILBOX_FULL_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register MAILBOX_EMPTY_ADDR */
# define MAILBOX_EMPTY_ADDR 3
# define F_MAILBOX_EMPTY(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_MAILBOX_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register MAILBOX0_WR_DATA */
# define MAILBOX0_WR_DATA 4
# define F_MAILBOX0_WR_DATA(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_MAILBOX0_WR_DATA_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register MAILBOX0_RD_DATA */
# define MAILBOX0_RD_DATA 5
# define F_MAILBOX0_RD_DATA(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_MAILBOX0_RD_DATA_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register KEEP_ALIVE */
# define KEEP_ALIVE 6
# define F_KEEP_ALIVE_CNT(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_KEEP_ALIVE_CNT_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register VER_L */
# define VER_L 7
# define F_VER_LSB(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_VER_LSB_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register VER_H */
# define VER_H 8
# define F_VER_MSB(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_VER_MSB_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register VER_LIB_L_ADDR */
# define VER_LIB_L_ADDR 9
# define F_SW_LIB_VER_L(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_LIB_VER_L_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register VER_LIB_H_ADDR */
# define VER_LIB_H_ADDR 10
# define F_SW_LIB_VER_H(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_LIB_VER_H_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SW_DEBUG_L */
# define SW_DEBUG_L 11
# define F_SW_DEBUG_7_0(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_DEBUG_7_0_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SW_DEBUG_H */
# define SW_DEBUG_H 12
# define F_SW_DEBUG_15_8(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_DEBUG_15_8_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register MAILBOX_INT_MASK */
# define MAILBOX_INT_MASK 13
# define F_MAILBOX_INT_MASK(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_MAILBOX_INT_MASK_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)

/* register MAILBOX_INT_STATUS */
# define MAILBOX_INT_STATUS 14
# define F_MAILBOX_INT_STATUS(x) (((x) & ((1 << 2) - 1)) << 0)
# define F_MAILBOX_INT_STATUS_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)

/* register SW_CLK_L */
# define SW_CLK_L 15
# define F_SW_CLOCK_VAL_L(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_CLOCK_VAL_L_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SW_CLK_H */
# define SW_CLK_H 16
# define F_SW_CLOCK_VAL_H(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_CLOCK_VAL_H_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SW_EVENTS0 */
# define SW_EVENTS0 17
# define F_SW_EVENTS7_0(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_EVENTS7_0_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SW_EVENTS1 */
# define SW_EVENTS1 18
# define F_SW_EVENTS15_8(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_EVENTS15_8_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SW_EVENTS2 */
# define SW_EVENTS2 19
# define F_SW_EVENTS23_16(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_EVENTS23_16_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register SW_EVENTS3 */
# define SW_EVENTS3 20
# define F_SW_EVENTS31_24(x) (((x) & ((1 << 8) - 1)) << 0)
# define F_SW_EVENTS31_24_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register XT_OCD_CTRL */
# define XT_OCD_CTRL 24
# define F_XT_DRESET(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_XT_DRESET_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
# define F_XT_OCDHALTONRESET(x) (((x) & ((1 << 1) - 1)) << 1)
# define F_XT_OCDHALTONRESET_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register XT_OCD_CTRL_RO */
# define XT_OCD_CTRL_RO 25
# define F_XT_XOCDMODE(x) (((x) & ((1 << 1) - 1)) << 0)
# define F_XT_XOCDMODE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register APB_INT_MASK */
# define APB_INT_MASK 27
# define F_APB_INTR_MASK(x) (((x) & ((1 << 3) - 1)) << 0)
# define F_APB_INTR_MASK_RD(x) (((x) & (((1 << 3) - 1) << 0)) >> 0)

/* register APB_STATUS_MASK */
# define APB_STATUS_MASK 28
# define F_APB_INTR_STATUS(x) (((x) & ((1 << 3) - 1)) << 0)
# define F_APB_INTR_STATUS_RD(x) (((x) & (((1 << 3) - 1) << 0)) >> 0)

#endif
