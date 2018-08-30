/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE SOFTWARE IS PROVIDED "AS IS",
 * WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 *
 * API_AFE_mcu2_dp.c
 *
 ******************************************************************************
 */
#include <linux/delay.h>
#include "API_AFE_mcu2_dp.h"
#include "../../../../mxc/hdp/all.h"

/* values of TX_TXCC_MGNFS_MULT_000 register for [voltage_swing][pre_emphasis]
 * 0xFF means, that the combination is forbidden.
static u16 mgnfsValues[4][4] = {{0x2B, 0x19, 0x0E, 0x02},
						{0x21, 0x10, 0x01, 0xFF},
						{0x18, 0x02, 0xFF, 0xFF},
						{0x04, 0xFF, 0xFF, 0xFF}};

* values of TX_TXCC_CPOST_MULT_00 register for [voltage_swing][pre_emphasis]
* 0xFF means, that the combination is forbidden.
static u16 cpostValues[4][4] = {{0x00, 0x14, 0x21, 0x29},
						{0x00, 0x15, 0x20, 0xFF},
						{0x00, 0x15, 0xFF, 0xFF},
						{0x00, 0xFF, 0xFF, 0xFF}};
*/

static void afe_write_reg(state_struct *state,
			  ENUM_AFE_LINK_RATE link_rate,
			  unsigned int addr,
			  unsigned int val1_6,
			  unsigned int val2_1,
			  unsigned int val2_4,
			  unsigned int val2_7,
			  unsigned int val3_2,
			  unsigned int val4_3,
			  unsigned int val5_4)
{
	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
		Afe_write(state, addr, val1_6); break;
	case AFE_LINK_RATE_2_1:
		Afe_write(state, addr, val2_1); break;
	case AFE_LINK_RATE_2_4:
		Afe_write(state, addr, val2_4); break;
	case AFE_LINK_RATE_2_7:
		Afe_write(state, addr, val2_7); break;
	case AFE_LINK_RATE_3_2:
		Afe_write(state, addr, val3_2); break;
	case AFE_LINK_RATE_4_3:
		Afe_write(state, addr, val4_3); break;
	case AFE_LINK_RATE_5_4:
		Afe_write(state, addr, val5_4); break;
	default:
		break;
	}
}

static void phy_cfg_24mhz(state_struct *state, int num_lanes)
{
	int k;

	for (k = 0; k < num_lanes; k++) {
		/* Afe_write (XCVR_DIAG_LANE_FCM_EN_TO | (k << 9), 0x01e0); */
		Afe_write(state, XCVR_DIAG_LANE_FCM_EN_MGN_TMR | (k << 9),
			  0x0090);
		Afe_write(state, TX_RCVDET_EN_TMR | (k << 9), 0x0960);
		Afe_write(state, TX_RCVDET_ST_TMR | (k << 9), 0x0030);
	}
}

static void phy_cfg_27mhz(state_struct *state, int num_lanes)
{
	int k;

	Afe_write(state, CMN_SSM_BIAS_TMR, 0x0087);
	Afe_write(state, CMN_PLLSM0_PLLEN_TMR, 0x001B);
	Afe_write(state, CMN_PLLSM0_PLLPRE_TMR, 0x0036);
	Afe_write(state, CMN_PLLSM0_PLLVREF_TMR, 0x001B);
	Afe_write(state, CMN_PLLSM0_PLLLOCK_TMR, 0x006C);
	Afe_write(state, CMN_ICAL_INIT_TMR, 0x0044);
	Afe_write(state, CMN_ICAL_ITER_TMR, 0x0006);
	Afe_write(state, CMN_ICAL_ADJ_INIT_TMR, 0x0022);
	Afe_write(state, CMN_ICAL_ADJ_ITER_TMR, 0x0006);
	Afe_write(state, CMN_TXPUCAL_INIT_TMR, 0x0022);
	Afe_write(state, CMN_TXPUCAL_ITER_TMR, 0x0006);
	Afe_write(state, CMN_TXPU_ADJ_INIT_TMR, 0x0022);
	Afe_write(state, CMN_TXPU_ADJ_ITER_TMR, 0x0006);
	Afe_write(state, CMN_TXPDCAL_INIT_TMR, 0x0022);
	Afe_write(state, CMN_TXPDCAL_ITER_TMR, 0x0006);
	Afe_write(state, CMN_TXPD_ADJ_INIT_TMR, 0x0022);
	Afe_write(state, CMN_TXPD_ADJ_ITER_TMR, 0x0006);
	Afe_write(state, CMN_RXCAL_INIT_TMR, 0x0022);
	Afe_write(state, CMN_RXCAL_ITER_TMR, 0x0006);
	Afe_write(state, CMN_RX_ADJ_INIT_TMR, 0x0022);
	Afe_write(state, CMN_RX_ADJ_ITER_TMR, 0x0006);

	for (k = 0; k < num_lanes; k++) {
		Afe_write(state, XCVR_PSM_CAL_TMR  | (k << 9), 0x016D);
		Afe_write(state, XCVR_PSM_A0IN_TMR | (k << 9), 0x016D);
		Afe_write(state, XCVR_DIAG_LANE_FCM_EN_MGN_TMR | (k << 9),
			  0x00A2);
		Afe_write(state, TX_DIAG_BGREF_PREDRV_DELAY    | (k << 9),
			  0x0097);
		Afe_write(state, TX_RCVDET_EN_TMR | (k << 9), 0x0A8C);
		Afe_write(state, TX_RCVDET_ST_TMR | (k << 9), 0x0036);
	}
}

static void phy_cfg_dp_pll0_24mhz(state_struct *state,
				  int num_lanes,
				  ENUM_AFE_LINK_RATE link_rate)
{
	int k;
	unsigned short rdata;

	rdata = Afe_read(state, PHY_HDP_CLK_CTL);
	rdata = rdata & 0x00FF;
	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_1:
	case AFE_LINK_RATE_2_4:
	case AFE_LINK_RATE_2_7:
		rdata = rdata | 0x2400;
		break;
	case AFE_LINK_RATE_3_2:
	case AFE_LINK_RATE_4_3:
	case AFE_LINK_RATE_5_4:
		rdata = rdata | 0x1200;
		break;
	case AFE_LINK_RATE_8_1: /* Not used in MCU2 */
	default:
		pr_info("Warning. Unsupported Link Rate!\n");
		break;
	}
	Afe_write(state, PHY_HDP_CLK_CTL, rdata);
	rdata = Afe_read(state, CMN_DIAG_HSCLK_SEL);
	rdata = rdata & 0xFFCC;
	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_1:
	case AFE_LINK_RATE_2_4:
	case AFE_LINK_RATE_2_7:
		rdata = rdata | 0x0011;
		break;
	case AFE_LINK_RATE_3_2:
	case AFE_LINK_RATE_4_3:
	case AFE_LINK_RATE_5_4:
		rdata = rdata | 0x0000;
		break;
	default:
		break;
	}
	Afe_write(state, CMN_DIAG_HSCLK_SEL, rdata);
	for (k = 0; k < num_lanes; k = k + 1) {
		rdata = Afe_read(state, (XCVR_DIAG_HSCLK_SEL | (k << 9)));
		rdata = rdata & 0xCFFF;
		switch (link_rate) {
		case AFE_LINK_RATE_1_6:
		case AFE_LINK_RATE_2_1:
		case AFE_LINK_RATE_2_4:
		case AFE_LINK_RATE_2_7:
			rdata = rdata | 0x1000;
			break;
		case AFE_LINK_RATE_3_2:
		case AFE_LINK_RATE_4_3:
		case AFE_LINK_RATE_5_4:
			rdata = rdata | 0x0000;
			break;
		default:
			break;
		}
		Afe_write(state, (XCVR_DIAG_HSCLK_SEL | (k << 9)), rdata);
	}
	/* Gbps       1.62    2.16    2.43    2.7     3.24    4.32    5.4 */
	Afe_write(state, CMN_PLL0_VCOCAL_INIT_TMR, 0x00F0);
	Afe_write(state, CMN_PLL0_VCOCAL_ITER_TMR, 0x0018);
	afe_write_reg(state, link_rate, CMN_PLL0_VCOCAL_START,
		      0x30B9, 0x3087, 0x3096, 0x30B4, 0x30B9, 0x3087, 0x30B4);
	afe_write_reg(state, link_rate, CMN_PLL0_INTDIV,
		      0x0086, 0x00B3, 0x00CA, 0x00E0, 0x0086, 0x00B3, 0x00E0);
	afe_write_reg(state, link_rate, CMN_PLL0_FRACDIV,
		      0xF915, 0xF6C7, 0x75A1, 0xF479, 0xF915, 0xF6C7, 0xF479);
	afe_write_reg(state, link_rate, CMN_PLL0_HIGH_THR,
		      0x0022, 0x002D, 0x0033, 0x0038, 0x0022, 0x002D, 0x0038);
#ifdef SSC_ON_INIT
	/* Following register writes enable SSC on PHY's initialization. */
	afe_write_reg(state, link_rate, CMN_PLL0_SS_CTRL1,
		      0x0140, 0x01AB, 0x01E0, 0x0204, 0x0140, 0x01AB, 0x0204);
	Afe_write(state, CMN_PLL0_SS_CTRL2, 0x7F03);
#endif
	Afe_write(state, CMN_PLL0_DSM_DIAG, 0x0020);
	afe_write_reg(state, link_rate, CMN_PLLSM0_USER_DEF_CTRL,
		      0x0000, 0x1000, 0x1000, 0x1000, 0x0000, 0x1000, 0x1000);
	Afe_write(state, CMN_DIAG_PLL0_OVRD, 0x0000);
	Afe_write(state, CMN_DIAG_PLL0_FBH_OVRD, 0x0000);
	Afe_write(state, CMN_DIAG_PLL0_FBL_OVRD, 0x0000);
	afe_write_reg(state, link_rate, CMN_DIAG_PLL0_V2I_TUNE,
		      0x0006, 0x0007, 0x0007, 0x0007, 0x0006, 0x0007, 0x0007);
	Afe_write(state, CMN_DIAG_PLL0_CP_TUNE,                 0x0045);
	Afe_write(state, CMN_DIAG_PLL0_LF_PROG,                 0x0008);
	afe_write_reg(state, link_rate, CMN_DIAG_PLL0_PTATIS_TUNE1,
		      0x0100, 0x0001, 0x0001, 0x0001, 0x0100, 0x0001, 0x0001);
	afe_write_reg(state, link_rate, CMN_DIAG_PLL0_PTATIS_TUNE2,
		      0x0007, 0x0001, 0x0001, 0x0001, 0x0007, 0x0001, 0x0001);
	for (k = 0; k < num_lanes; k = k + 1) {
		rdata = Afe_read(state, (XCVR_DIAG_PLLDRC_CTRL | (k << 9)));
		rdata = rdata & 0x8FFF;
		switch (link_rate) {
		case AFE_LINK_RATE_1_6:
		case AFE_LINK_RATE_2_1:
		case AFE_LINK_RATE_2_4:
		case AFE_LINK_RATE_2_7:
			rdata = rdata | 0x2000;
			break;
		case AFE_LINK_RATE_3_2:
		case AFE_LINK_RATE_4_3:
		case AFE_LINK_RATE_5_4:
			rdata = rdata | 0x1000;
			break;
		default:
			break;
		}
		Afe_write(state, (XCVR_DIAG_PLLDRC_CTRL | (k << 9)), rdata);
	}
}

/* Valid for 27 MHz only */
static void phy_cfg_dp_pll0_27mhz(state_struct *state,
				  int num_lanes,
				  ENUM_AFE_LINK_RATE link_rate)
{
	int k;
	unsigned short rdata;

	rdata = Afe_read(state, PHY_HDP_CLK_CTL);
	rdata = rdata & 0x00FF;
	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_1:
	case AFE_LINK_RATE_2_4:
	case AFE_LINK_RATE_2_7:
		rdata = rdata | 0x2400;
		break;
	case AFE_LINK_RATE_3_2:
	case AFE_LINK_RATE_4_3:
	case AFE_LINK_RATE_5_4:
		rdata = rdata | 0x1200;
		break;
	case AFE_LINK_RATE_8_1: /* Not supported MCU1 or MCU2 */
	default:
		pr_info("Warning. Unsupported Link Rate!\n");
		break;
	}
	Afe_write(state, PHY_HDP_CLK_CTL, rdata);
	rdata = Afe_read(state, CMN_DIAG_HSCLK_SEL);
	rdata = rdata & 0xFFCC;
	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_1:
	case AFE_LINK_RATE_2_4:
	case AFE_LINK_RATE_2_7:
		rdata = rdata | 0x0011;
		break;
	case AFE_LINK_RATE_3_2:
	case AFE_LINK_RATE_4_3:
	case AFE_LINK_RATE_5_4:
		rdata = rdata | 0x0000;
		break;
	default:
		break;
	}
	Afe_write(state, CMN_DIAG_HSCLK_SEL, rdata);
	for (k = 0; k < num_lanes; k = k + 1) {
		rdata = Afe_read(state, (XCVR_DIAG_HSCLK_SEL | (k << 9)));
		rdata = rdata & 0xCFFF;
		switch (link_rate) {
		case AFE_LINK_RATE_1_6:
		case AFE_LINK_RATE_2_1:
		case AFE_LINK_RATE_2_4:
		case AFE_LINK_RATE_2_7:
			rdata = rdata | 0x1000;
			break;
		case AFE_LINK_RATE_3_2:
		case AFE_LINK_RATE_4_3:
		case AFE_LINK_RATE_5_4:
			rdata = rdata | 0x0000;
			break;
		default:
			break;
		}
		Afe_write(state, (XCVR_DIAG_HSCLK_SEL | (k << 9)), rdata);
	}
	/* Gbps       1.62    2.16    2.43    2.7     3.24    4.32    5.4 */
	Afe_write(state, CMN_PLL0_VCOCAL_INIT_TMR, 0x010E);
	Afe_write(state, CMN_PLL0_VCOCAL_ITER_TMR, 0x001B);
	afe_write_reg(state, link_rate, CMN_PLL0_VCOCAL_START,
		      0x30B9, 0x3087, 0x3096, 0x30B4, 0x30B9, 0x3087, 0x30B4);
	afe_write_reg(state, link_rate, CMN_PLL0_INTDIV,
		      0x0077, 0x009F, 0x00B3, 0x00C7, 0x0077, 0x009F, 0x00C7);
	afe_write_reg(state, link_rate, CMN_PLL0_FRACDIV,
		      0xF9DA, 0xF7CD, 0xF6C7, 0xF5C1, 0xF9DA, 0xF7CD, 0xF5C1);
	afe_write_reg(state, link_rate, CMN_PLL0_HIGH_THR,
		      0x001E, 0x0028, 0x002D, 0x0032, 0x001E, 0x0028, 0x0032);
#ifdef SSC_ON_INIT
	/* Following register writes enable SSC on PHY's initialization. */
	afe_write_reg(state, link_rate, CMN_PLL0_SS_CTRL1,
		      0x0152, 0x01C2, 0x01FB, 0x0233, 0x0152, 0x01C2, 0x0233);
	Afe_write(state, CMN_PLL0_SS_CTRL2, 0x6B04);
#endif
	Afe_write(state, CMN_PLL0_DSM_DIAG, 0x0020);
	afe_write_reg(state, link_rate, CMN_PLLSM0_USER_DEF_CTRL,
		      0x0000, 0x1000, 0x1000, 0x1000, 0x0000, 0x1000, 0x1000);
	Afe_write(state, CMN_DIAG_PLL0_OVRD, 0x0000);
	Afe_write(state, CMN_DIAG_PLL0_FBH_OVRD, 0x0000);
	Afe_write(state, CMN_DIAG_PLL0_FBL_OVRD, 0x0000);
	afe_write_reg(state, link_rate, CMN_DIAG_PLL0_V2I_TUNE,
		      0x0006, 0x0007, 0x0007, 0x0007, 0x0006, 0x0007, 0x0007);
	afe_write_reg(state, link_rate, CMN_DIAG_PLL0_CP_TUNE,
		      0x0043, 0x0043, 0x0043, 0x0042, 0x0043, 0x0043, 0x0042);
	Afe_write(state, CMN_DIAG_PLL0_LF_PROG,                 0x0008);
	afe_write_reg(state, link_rate, CMN_DIAG_PLL0_PTATIS_TUNE1,
		      0x0100, 0x0001, 0x0001, 0x0001, 0x0100, 0x0001, 0x0001);
	afe_write_reg(state, link_rate, CMN_DIAG_PLL0_PTATIS_TUNE2,
		      0x0007, 0x0001, 0x0001, 0x0001, 0x0007, 0x0001, 0x0001);
	for (k = 0; k < num_lanes; k = k + 1) {
		rdata = Afe_read(state, (XCVR_DIAG_PLLDRC_CTRL | (k << 9)));
		rdata = rdata & 0x8FFF;
		switch (link_rate) {
		case AFE_LINK_RATE_1_6:
		case AFE_LINK_RATE_2_1:
		case AFE_LINK_RATE_2_4:
		case AFE_LINK_RATE_2_7:
			rdata = rdata | 0x2000;
			break;
		case AFE_LINK_RATE_3_2:
		case AFE_LINK_RATE_4_3:
		case AFE_LINK_RATE_5_4:
			rdata = rdata | 0x1000;
			break;
		default:
			break;
		}
		Afe_write(state, (XCVR_DIAG_PLLDRC_CTRL | (k << 9)), rdata);
	}
}

static void phy_cfg_dp_ln(state_struct *state, int num_lanes)
{
	int k;
	unsigned short rdata;

	for (k = 0; k < num_lanes; k = k + 1) {
		Afe_write(state, (XCVR_PSM_RCTRL | (k << 9)),  0xBEFC);
		if (state->edp == 0) {
			Afe_write(state, (TX_PSC_A0 | (k << 9)), 0x6799);
			Afe_write(state, (TX_PSC_A1 | (k << 9)), 0x6798);
			Afe_write(state, (TX_PSC_A2 | (k << 9)), 0x0098);
			Afe_write(state, (TX_PSC_A3 | (k << 9)), 0x0098);
		} else {
			Afe_write(state, (TX_PSC_A0 | (k << 9)), 0x279B);
			Afe_write(state, (TX_PSC_A1 | (k << 9)), 0x2798);
			Afe_write(state, (TX_PSC_A2 | (k << 9)), 0x0098);
			Afe_write(state, (TX_PSC_A3 | (k << 9)), 0x0098);

			rdata = Afe_read(state, TX_DIAG_TX_DRV | (k << 9));
			/* keep bits related to programmable boost */
			rdata &= 0x0600;
			rdata |= 0x00C0;
			Afe_write(state, (TX_DIAG_TX_DRV | (k << 9)), rdata);
		}
		rdata = Afe_read(state, RX_PSC_CAL | (k << 9));
		rdata = rdata & 0xFFBB;
		Afe_write(state, (RX_PSC_CAL | (k << 9)), rdata);
		rdata = Afe_read(state, RX_PSC_A0  | (k << 9));
		rdata = rdata & 0xFFBB;
		Afe_write(state, (RX_PSC_A0  | (k << 9)), rdata);
	}
}

static void aux_cfg_t28hpc(state_struct *state)
{
#ifdef DEBUG
	unsigned short rdata;
#endif
	Afe_write(state, TX_DIG_CTRL_REG_2, 36);

	Afe_write(state, TX_ANA_CTRL_REG_2, 0x0100);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_2, 0x0300);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_3, 0x0000);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_1, 0x2008);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_1, 0x2018);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_1, 0xA018);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_2, 0x030C);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_5, 0x0000);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_4, 0x1001);
	cdn_usleep(150);
	Afe_write(state, TX_ANA_CTRL_REG_1, 0xA098);
	cdn_usleep(5000);
	Afe_write(state, TX_ANA_CTRL_REG_1, 0xA198);
	cdn_usleep(5000);
	Afe_write(state, TX_ANA_CTRL_REG_2, 0x030d);
	cdn_usleep(5000);
	Afe_write(state, TX_ANA_CTRL_REG_2, 0x030f);
	cdn_usleep(5000);

#ifdef DEBUG
	rdata = Afe_read(state, TX_ANA_CTRL_REG_1);
	pr_info("TX_ANA_CTRL_REG_1 %x)\n", rdata);
	rdata = Afe_read(state, TX_ANA_CTRL_REG_2);
	pr_info("TX_ANA_CTRL_REG_2 %x)\n", rdata);
	rdata = Afe_read(state, TX_ANA_CTRL_REG_3);
	pr_info("TX_ANA_CTRL_REG_3 %x)\n", rdata);
	rdata = Afe_read(state, TX_ANA_CTRL_REG_4);
	pr_info("TX_ANA_CTRL_REG_4 %x)\n", rdata);
	rdata = Afe_read(state, TX_ANA_CTRL_REG_5);
	pr_info("TX_ANA_CTRL_REG_5 %x)\n", rdata);
#endif
}

void afe_init_t28hpc(state_struct *state,
		     int num_lanes,
		     ENUM_AFE_LINK_RATE link_rate)
{
	u16 val;
	const int phy_reset_workaround = 0;

	const REFCLK_FREQ refclk = REFCLK_27MHZ;

	if (AFE_check_rate_supported(link_rate) == 0) {
		pr_err("afe_init_t28hpc(): Selected link rate not supported: 0x%x\n",
		       link_rate);
		return;
	}

	if (phy_reset_workaround) {
		int k;
		uint32_t reg_val;
		/* enable PHY isolation mode only for CMN */
		/* register PHY_PMA_ISOLATION_CTRL */
		Afe_write(state, 0xC81F, 0xD000);

		/* set cmn_pll0_clk_datart1_div/cmn_pll0_clk_datart0_div
		 * dividers
		 */
		/* register PHY_PMA_ISO_PLL_CTRL1 */
		reg_val = Afe_read(state, 0xC812);
		reg_val &= 0xFF00;
		reg_val |= 0x0012;
		Afe_write(state, 0xC812, reg_val);

		/* assert PHY reset from isolation register */
		/* register PHY_ISO_CMN_CTRL */
		Afe_write(state, 0xC010, 0x0000);

		/* assert PMA CMN reset */
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0000);

		for (k = 0; k < num_lanes; k++) {
			/* register XCVR_DIAG_BIDI_CTRL */
			Afe_write(state, 0x40E8 | (k << 9), 0x00FF);
		}
	}

	val = Afe_read(state, PHY_PMA_CMN_CTRL1);
	val = val & 0xFFF7;
	val = val | 0x0008;
	Afe_write(state, PHY_PMA_CMN_CTRL1, val);

	Afe_write(state, CMN_DIAG_PLL0_TEST_MODE, 0x0022);
	Afe_write(state, CMN_PSM_CLK_CTRL, 0x0016);

	if (refclk == REFCLK_24MHZ) {
		phy_cfg_24mhz(state, num_lanes);
		phy_cfg_dp_pll0_24mhz(state, num_lanes, link_rate);
	} else if (refclk == REFCLK_27MHZ) {
		phy_cfg_27mhz(state, num_lanes);
		phy_cfg_dp_pll0_27mhz(state, num_lanes, link_rate);
	} else
		pr_info("AFE_init() *E: Incorrect value of the refclk: %0d\n",
			refclk);

	val = Afe_read(state, PHY_PMA_CMN_CTRL1);
	val = val & 0xFF8F;
	/* for single ended reference clock on the cmn_ref_clk_int pin:
	 * PHY_PMA_CMN_CTRL1[6:4]=3'b011
	 * val |= 0x0030;
	 */

	/* for differential clock on the refclk_p and refclk_m off chip pins:
	 * PHY_PMA_CMN_CTRL1[6:4]=3'b000
	 * val = val | 0x0030;
	 */
	val = val | 0x0000; /* select external reference */
	Afe_write(state, PHY_PMA_CMN_CTRL1, val);

	/* for differential clock on the refclk_p and refclk_m off chip pins:
	 * CMN_DIAG_ACYA[8]=1'b1
	 */
	Afe_write(state, CMN_DIAG_ACYA /*0x01FF*/, 0x0100);

	if (phy_reset_workaround) {
		int k;
		/* Deassert PHY reset*/
		/* register PHY_ISO_CMN_CTRL */
		Afe_write(state, 0xC010, 0x0001);
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0003);
		for (k = 0; k < num_lanes; k++) {
			/* register XCVR_PSM_RCTRL */
			Afe_write(state, 0x4001 | (k << 9), 0xFEFC);
		}
		/* Assert cmn_macro_pwr_en */
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0013);

		/* wait for cmn_macro_pwr_en_ack */
		/* PHY_PMA_ISO_CMN_CTRL */
		while (!(Afe_read(state, 0xC810) & (1 << 5)))
			;

		/* wait for cmn_ready */
		/* PHY_PMA_CMN_CTRL1 */
		while (!(Afe_read(state, 0xC800) & (1 << 0)))
			;
	}

	if (state->edp != 0)
		Afe_write(state, CMN_DIAG_CAL_CTRL, 0x0001);

	phy_cfg_dp_ln(state, num_lanes);

	/* Configure PHY in A2 Mode */
	Afe_write(state, PHY_HDP_MODE_CTRL, 0x0004);
}

void afe_power_t28hpc(state_struct *state,
		      int num_lanes,
		      ENUM_AFE_LINK_RATE link_rate)
{
	unsigned short val;
	int i = 0;

	if (AFE_check_rate_supported(link_rate) == 0) {
		pr_err("%s() *E: Selected link rate not supported: 0x%x\n",
		       __func__, link_rate);
		return;
	}

	Afe_write(state, TX_DIAG_ACYA_0, 1);
	Afe_write(state, TX_DIAG_ACYA_1, 1);
	Afe_write(state, TX_DIAG_ACYA_2, 1);
	Afe_write(state, TX_DIAG_ACYA_3, 1);

	Afe_write(state, TXDA_CYA_AUXDA_CYA, 1);

	/* Wait for A2 ACK (PHY_HDP_MODE_CTL [6] = 1’b1) */
	do {
		val = Afe_read(state, PHY_HDP_MODE_CTRL);
		val = val >> 6;
		if (i++ % 10000 == 0)
			pr_info("Wait for A2 ACK\n");
	} while ((val & 1) ==  0);

	/* Configure PHY in A0 mode (PHY must be in the A0 power
	 * state in order to transmit data)
	 */
	Afe_write(state, PHY_HDP_MODE_CTRL, 0x0101);
	/* Wait for A2 ACK (PHY_HDP_MODE_CTL [4] = 1’b1) */
	do {
		val = Afe_read(state, PHY_HDP_MODE_CTRL);
		val = val >> 4;
		if (i++ % 10000 == 0)
			pr_info("Wait for A2 ACK again\n");
	} while ((val & 1) ==  0);

	aux_cfg_t28hpc(state);

}
