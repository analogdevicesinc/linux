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
 * API_AFE_ss28fdsoi_hdmirx.h
 *
 ******************************************************************************
 */

#ifndef API_AFE_SS28FDSOI_HDMIRX_H_
#define API_AFE_SS28FDSOI_HDMIRX_H_

#include <linux/io.h>
#include "../../../../mxc/hdp/all.h"

#define REFCLK_FREQ_KHZ                        24000
#define LINK_WRITE                             0x2000
#define LINK_ID                                0x0

#define CMN_PLLSM0_PLLEN_TMR_ADDR              0x0029
#define CMN_PLLSM0_PLLPRE_TMR_ADDR             0x002A
#define CMN_PLLSM0_PLLVREF_TMR_ADDR            0x002B
#define CMN_PLLSM0_PLLLOCK_TMR_ADDR            0x002C
#define CMN_PLLSM0_USER_DEF_CTRL_ADDR          0x002F
#define CMN_PLL0_VCOCAL_CTRL_ADDR              0x0080
#define CMN_PLL0_VCOCAL_OVRD_ADDR              0x0083
#define CMN_PLL0_LOCK_REFCNT_START_ADDR        0x0090
#define CMN_PLL0_LOCK_PLLCNT_START_ADDR        0x0092
#define CMN_PLL0_DIV2SEL_OSR_CTRL_ADDR         0x009B
#define CMN_ICAL_CTRL_ADDR                     0x00C0
#define CMN_ICAL_OVRD_ADDR                     0x00C1
#define CMN_RXCAL_CTRL_ADDR                    0x00D0
#define CMN_RXCAL_OVRD_ADDR                    0x00D1
#define CMN_RXCAL_INIT_TMR_ADDR                0x00D4
#define CMN_TXPUCAL_OVRD_ADDR                  0x00E1
#define CMN_TXPDCAL_OVRD_ADDR                  0x00F1
#define CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR      0x01A0
#define CMN_CMSMT_REF_CLK_TMR_VALUE_ADDR       0x01A2
#define CMN_CMSMT_TEST_CLK_CNT_VALUE_ADDR      0x01A3
#define CMN_DIAG_PLL0_FBH_OVRD_ADDR            0x01C0
#define CMN_DIAG_PLL0_FBL_OVRD_ADDR            0x01C1
#define CMN_DIAG_PLL0_TEST_MODE_ADDR           0x01C4
#define CMN_DIAG_PLL0_INCLK_CTRL_ADDR          0x01CA
#define CMN_DIAG_PLL0_V2I_TUNE_ADDR            0x01C5
#define CMN_DIAG_PLL0_CP_TUNE_ADDR             0x01C6
#define CMN_DIAG_PLL0_PTATIS_TUNE1_ADDR        0x01C8
#define CMN_DIAG_PLL0_PTATIS_TUNE2_ADDR        0x01C9
#define XCVR_PSM_CAL_TMR_ADDR                  0x4002
#define XCVR_PSM_A0IN_TMR_ADDR                 0x4003
#define XCVR_DIAG_RX_LANE_CAL_RST_TMR_ADDR     0x40EA
#define TX_ANA_CTRL_REG_1_ADDR                 0x5020
#define TX_ANA_CTRL_REG_2_ADDR                 0x5021
#define TX_DIG_CTRL_REG_1_ADDR                 0x5023
#define TX_DIG_CTRL_REG_2_ADDR                 0x5024
#define TXDA_CYA_AUXDA_CYA_ADDR                0x5025
#define TX_ANA_CTRL_REG_3_ADDR                 0x5026
#define TX_ANA_CTRL_REG_4_ADDR                 0x5027
#define TX_ANA_CTRL_REG_5_ADDR                 0x5029
#define RX_PSC_A0_ADDR                         0x8000
#define RX_IQPI_ILL_CAL_OVRD_ADDR              0x8023
#define RX_EPI_ILL_CAL_OVRD_ADDR               0x8033
#define RX_SLC_CTRL_ADDR                       0x80E0
#define RX_REE_PERGCSM_EQENM_PH1_ADDR          0x8179
#define RX_REE_PERGCSM_EQENM_PH2_ADDR          0x817A
#define RX_REE_VGA_GAIN_OVRD_ADDR              0x81AD
#define RX_REE_SMGM_CTRL1_ADDR                 0x81BD
#define RX_DIAG_ILL_IQ_TRIM0_ADDR              0x81C1
#define RX_DIAG_ILL_E_TRIM0_ADDR               0x81C2
#define RX_DIAG_ILL_IQE_TRIM2_ADDR             0x81C5
#define RX_DIAG_DFE_CTRL2_ADDR                 0x81D4
#define RX_DIAG_SC2C_DELAY_ADDR                0x81E1
#define RX_DIAG_SMPLR_OSR_ADDR                 0x81E2
#define RX_CLK_SLICER_CAL_OVRD_ADDR            0x8621
#define RX_CLK_SLICER_CAL_INIT_TMR_ADDR        0x8624
#define PHY_MODE_CTL_ADDR                      0xC008
#define PHY_PMA_CMN_CTRL1_ADDR                 0xC800
#define PHY_PMA_CMN_CTRL2_ADDR                 0xC801
#define PHY_PMA_SSM_STATE_ADDR                 0xC802
#define PHY_PMA_PLL_SM_STATE_ADDR              0xC803
#define PHY_PMA_XCVR_CTRL_ADDR                 0xCC00

typedef enum {
  TMDS_BIT_CLOCK_RATIO_1_10 = 10,
  TMDS_BIT_CLOCK_RATIO_1_40 = 40
} tmds_bit_clock_ratio_t;

typedef enum {
  PIXEL_ENCODING_RGB    = 0,
  PIXEL_ENCODING_YUV422 = 1,
  PIXEL_ENCODING_YUV444 = 2,
  PIXEL_ENCODING_YUV420 = 3,
} pixel_encoding_t;

void speedup_config(state_struct *state);
void arc_config(state_struct *state);
void pma_config(state_struct *state);
u8 pma_cmn_ready(state_struct *state);
u8 pma_rx_clk_signal_detect(state_struct *state);
u32 pma_rx_clk_freq_detect(state_struct *state);
void pre_data_rate_change(state_struct *state);
u8 pma_pll_config(state_struct *state, u32, clk_ratio_t, tmds_bit_clock_ratio_t, unsigned char);
clk_ratio_t clk_ratio_detect(state_struct *state, u32, u32, u8, pixel_encoding_t, tmds_bit_clock_ratio_t);
void phy_status(state_struct *state);

#endif
