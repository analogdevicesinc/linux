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
 * API_AFE_ss28fdsoi_hdmirx.c
 *
 ******************************************************************************
 */

#include "API_AFE_ss28fdsoi_hdmirx.h"

static inline void write16(state_struct *state, u32 addr, u16 val)
{
	Afe_write(state, addr, val);
}

static inline void multi_write16(state_struct *state, u32 addr,
				 u16 val)
{
	u16 addr_tmp = addr;

	if ((addr & 0x1E00) == LINK_ID << 9) {
		addr_tmp |= LINK_WRITE;
		Afe_write(state, addr_tmp, val);
	} else {
		Afe_write(state, addr_tmp, val);
	}
}

static inline u16 read16(state_struct *state, u32 addr)
{
	u16 reg_val;

	reg_val = Afe_read(state, addr);
	return reg_val;
}

u16 inside_i(u16 value, u16 left_sharp_corner,
		  u16 right_sharp_corner)
{
	if (value < left_sharp_corner)
		return 0;
	if (value > right_sharp_corner)
		return 0;
	return 1;
}

u16 inside_f(u32 value, u32 left_sharp_corner, u32 right_sharp_corner)
{
	if (value < left_sharp_corner)
		return 0;
	if (value > right_sharp_corner)
		return 0;
	return 1;
}

void arc_config(state_struct *state)
{
	u16 reg_val;

	write16(state, TX_DIG_CTRL_REG_2_ADDR, 0x0024);

	reg_val = read16(state, TX_ANA_CTRL_REG_1_ADDR);
	reg_val |= 0x2000;
	write16(state, TX_ANA_CTRL_REG_1_ADDR, reg_val);

	write16(state, TX_ANA_CTRL_REG_2_ADDR, 0x0100);
	write16(state, TX_ANA_CTRL_REG_2_ADDR, 0x0300);
	write16(state, TX_ANA_CTRL_REG_3_ADDR, 0x0000);
	write16(state, TX_ANA_CTRL_REG_1_ADDR, 0x2008);
	write16(state, TX_ANA_CTRL_REG_1_ADDR, 0x2018);
	write16(state, TX_ANA_CTRL_REG_1_ADDR, 0x2098);
	write16(state, TX_ANA_CTRL_REG_2_ADDR, 0x030C);
	write16(state, TX_ANA_CTRL_REG_5_ADDR, 0x0000);
	write16(state, TX_ANA_CTRL_REG_4_ADDR, 0x4001);
	write16(state, TX_ANA_CTRL_REG_1_ADDR, 0x2198);
	write16(state, TX_ANA_CTRL_REG_2_ADDR, 0x030D);
	write16(state, TX_ANA_CTRL_REG_2_ADDR, 0x030F);
}

void pma_config(state_struct *state)
{
	int i;
	u16 reg_val = 0;
	pr_info("pma_config() Configuring PMA\n");

	write16(state, CMN_CMSMT_REF_CLK_TMR_VALUE_ADDR, 0x0801);
	write16(state, RX_CLK_SLICER_CAL_INIT_TMR_ADDR, 0x00FF);
	write16(state, CMN_RXCAL_INIT_TMR_ADDR, 0x003F);
	write16(state, CMN_DIAG_PLL0_TEST_MODE_ADDR, 0x0022);
	multi_write16(state, XCVR_PSM_CAL_TMR_ADDR, 0x0160);

	/* Drives the rx_differential_invert PMA input for the associated lane */
	for (i = 0; i < 3; i++) {
		reg_val = 0x0c61;
		write16(state, (PHY_PMA_XCVR_CTRL_ADDR | (i << 6)), reg_val);
	}
}

void pre_data_rate_change(state_struct *state)
{
	u16 reg_val;

	pr_info("pre_data_rate_change() Set the A3 power mode\n");
	reg_val = read16(state, PHY_MODE_CTL_ADDR);
	reg_val &= 0xFFF0;
	reg_val |= 0x0008;
	write16(state, PHY_MODE_CTL_ADDR, reg_val);

	msleep(20);

	/* Disable PLL */
	pr_info("pre_data_rate_change() Disable PLL0\n");
	reg_val = read16(state, PHY_MODE_CTL_ADDR);
	reg_val &= 0xEFFF;
	write16(state, PHY_MODE_CTL_ADDR, reg_val);

	msleep(20);
}

u8 pma_cmn_ready(state_struct *state)
{
	u32 i;

	for (i = 0; i < 20; i++) {
		if (read16(state, PHY_PMA_CMN_CTRL1_ADDR) & (1 << 0))
			break;
		msleep(10);
	}
	if (i == 20)
		return -1;
	return 0;
}

u8 pma_rx_clk_signal_detect(state_struct *state)
{
	u32 i;

	for (i = 0; i < 20; i++) {
		if (read16(state, PHY_MODE_CTL_ADDR) & (1 << 8))
			break;
		msleep(10);
	}
	if (i == 20)
		return -1;
	return 0;
}

u32 pma_rx_clk_freq_detect(state_struct *state)
{
	u16 reg_val;
	u32 rx_clk_freq;
	u32 i;

	pr_info("pma_rx_clk_freq_detect() Starting Rx clock frequency detection...\n");

	/* Start frequency detection: */
	write16(state, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x8000);

	/* Wait for pma_rx_clk_freq_detect_done */
	for (i = 0; i < 20; i++) {
		if (read16(state, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR) & (1 << 14))
			break;
		msleep(10);
	}
	if (i == 20)
		return -1;

	/* Read the measured value */
	reg_val = read16(state, CMN_CMSMT_TEST_CLK_CNT_VALUE_ADDR);

	/* Calculate TMDS clock frequency */
	rx_clk_freq = reg_val * REFCLK_FREQ_KHZ / 2048;

	/* Turn off frequency measurement: */
	write16(state, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x0000);
	pr_info("pma_rx_clk_freq_detect() Starting Rx clock frequency detection... DONE (TMDS clock freq: %d kHz)\n",
	     rx_clk_freq);
	return rx_clk_freq;
}

void pma_pll_config(state_struct *state,
		    u32 rx_clk_freq,
		    clk_ratio_t clk_ratio,
		    tmds_bit_clock_ratio_t tmds_bit_clk_ratio,
		    unsigned char data_rate_change)
{
	int i, loop;
	u16 reg_val;
	u64 vco_freq_khz;

	reg_field_t cmnda_pll0_ip_div;
	reg_field_t cmnda_pll0_hs_sym_div_sel;
	reg_field_t cmn_pll0_fb_div_high_ovrd_en;
	reg_field_t cmnda_pll0_fb_div_high_out;
	reg_field_t cmn_pll0_fb_div_low_ovrd_en;
	reg_field_t cmnda_pll0_fb_div_low_out;
	reg_field_t cmn_pll_clk_osr;
	reg_field_t cmn_pll_clk_div2_ratio;
	reg_field_t cmn_pll_clk_div2_sel;
	reg_field_t rx_diag_smplr_osr;
	reg_field_t rx_psc_a0;
	reg_field_t rx_ree_pergcsm_eqenm_ph1;
	reg_field_t rx_ree_pergcsm_eqenm_ph2;
	reg_field_t vga_gain_accum_override_en;
	reg_field_t vga_gain_accum_override;
	reg_field_t vga_gain_tgt_adj_override_en;
	reg_field_t vga_gain_tgt_adj_override;
	reg_field_t ree_gen_sm_en_usb;
	reg_field_t ree_gen_sm_en_periodic;
	reg_field_t ana_en_epath_gen_ctrl_sm_usb;
	reg_field_t ana_en_epath_gen_ctrl_sm_periodic;
	reg_field_t rxda_eq_range_sel;
	reg_field_t rxda_vga_sa_range_sel;
	reg_field_t vco_ring_select;
	reg_field_t cmnda_pll0_v2i_prog;
	reg_field_t cmnda_pll0_coarse_prog;
	reg_field_t cmnda_pll0_cp_gain;
	reg_field_t cmnda_pll0_const_ndac_cntrl;
	reg_field_t cmnda_pll0_const_pmos_cntrl;
	reg_field_t cmnda_pll0_ptat_ndac_cntrl;
	reg_field_t rxda_pi_iq_bias_trim;
	reg_field_t rxda_pi_iq_pload_bias_trim;
	reg_field_t rxda_pi_iq_pload_trim;
	reg_field_t rxda_pi_e_bias_trim;
	reg_field_t rxda_pi_e_pload_bias_trim;
	reg_field_t rxda_pi_e_pload_trim;
	reg_field_t rxda_pi_range_sel;
	reg_field_t rxda_pi_cal_cm_trim;
	reg_field_t xcvr_pll_en;
	reg_field_t xcvr_link_reset_n;
	reg_field_t xcvr_power_state_req;
	reg_field_t xcvr_power_state_ack;
	reg_field_t iso_pma_cmn_pll0_clk_datart1_div;
	reg_field_t iso_pma_cmn_pll0_clk_datart0_div;
	reg_field_t iso_pma_cmn_pll0_clk_en;

	/* Set fields' labels */
	cmnda_pll0_ip_div.label = "cmnda_pll0_ip_div";
	cmnda_pll0_hs_sym_div_sel.label = "cmnda_pll0_hs_sym_div_sel";
	cmn_pll0_fb_div_high_ovrd_en.label = "cmn_pll0_fb_div_high_ovrd_en";
	cmnda_pll0_fb_div_high_out.label = "cmnda_pll0_fb_div_high_out";
	cmn_pll0_fb_div_low_ovrd_en.label = "cmn_pll0_fb_div_low_ovrd_en";
	cmnda_pll0_fb_div_low_out.label = "cmnda_pll0_fb_div_low_out";
	cmn_pll_clk_osr.label = "cmn_pll_clk_osr";
	cmn_pll_clk_div2_ratio.label = "cmn_pll_clk_div2_ratio";
	cmn_pll_clk_div2_sel.label = "cmn_pll_clk_div2_sel";
	rx_diag_smplr_osr.label = "rx_diag_smplr_osr";
	rx_psc_a0.label = "rx_psc_a0";
	rx_ree_pergcsm_eqenm_ph1.label = "rx_ree_pergcsm_eqenm_ph1";
	rx_ree_pergcsm_eqenm_ph2.label = "rx_ree_pergcsm_eqenm_ph2";
	vga_gain_accum_override_en.label = "vga_gain_accum_override_en";
	vga_gain_accum_override.label = "vga_gain_accum_override";
	vga_gain_tgt_adj_override_en.label = "vga_gain_tgt_adj_override_en";
	vga_gain_tgt_adj_override.label = "vga_gain_tgt_adj_override";
	ree_gen_sm_en_usb.label = "ree_gen_sm_en_usb";
	ree_gen_sm_en_periodic.label = "ree_gen_sm_en_periodic";
	ana_en_epath_gen_ctrl_sm_usb.label = "ana_en_epath_gen_ctrl_sm_usb";
	ana_en_epath_gen_ctrl_sm_periodic.label =
	    "ana_en_epath_gen_ctrl_sm_periodic";
	rxda_eq_range_sel.label = "rxda_eq_range_sel";
	rxda_vga_sa_range_sel.label = "rxda_vga_sa_range_sel";
	vco_ring_select.label = "vco_ring_select";
	cmnda_pll0_v2i_prog.label = "cmnda_pll0_v2i_prog";
	cmnda_pll0_coarse_prog.label = "cmnda_pll0_coarse_prog";
	cmnda_pll0_cp_gain.label = "cmnda_pll0_cp_gain";
	cmnda_pll0_const_ndac_cntrl.label = "cmnda_pll0_const_ndac_cntrl";
	cmnda_pll0_const_pmos_cntrl.label = "cmnda_pll0_const_pmos_cntrl";
	cmnda_pll0_ptat_ndac_cntrl.label = "cmnda_pll0_ptat_ndac_cntrl";
	rxda_pi_iq_bias_trim.label = "rxda_pi_iq_bias_trim";
	rxda_pi_iq_pload_bias_trim.label = "rxda_pi_iq_pload_bias_trim";
	rxda_pi_iq_pload_trim.label = "rxda_pi_iq_pload_trim";
	rxda_pi_e_bias_trim.label = "rxda_pi_e_bias_trim";
	rxda_pi_e_pload_bias_trim.label = "rxda_pi_e_pload_bias_trim";
	rxda_pi_e_pload_trim.label = "rxda_pi_e_pload_trim";
	rxda_pi_range_sel.label = "rxda_pi_range_sel";
	rxda_pi_cal_cm_trim.label = "rxda_pi_cal_cm_trim";
	xcvr_pll_en.label = "xcvr_pll_en";
	xcvr_link_reset_n.label = "xcvr_link_reset_n";
	xcvr_power_state_req.label = "xcvr_power_state_req";
	xcvr_power_state_ack.label = "xcvr_power_state_ack";
	iso_pma_cmn_pll0_clk_datart1_div.label = "iso_pma_cmn_pll0_clk_datart1_div";
	iso_pma_cmn_pll0_clk_datart0_div.label = "iso_pma_cmn_pll0_clk_datart0_div";
	iso_pma_cmn_pll0_clk_en.label = "iso_pma_cmn_pll0_clk_en";

	/* Set field position in a target register */
	cmnda_pll0_ip_div.msb = 7;
	cmnda_pll0_ip_div.lsb = 0;
	cmnda_pll0_hs_sym_div_sel.msb = 9;
	cmnda_pll0_hs_sym_div_sel.lsb = 8;
	cmn_pll0_fb_div_high_ovrd_en.msb = 15;
	cmn_pll0_fb_div_high_ovrd_en.lsb = 15;
	cmnda_pll0_fb_div_high_out.msb = 9;
	cmnda_pll0_fb_div_high_out.lsb = 0;
	cmn_pll0_fb_div_low_ovrd_en.msb = 15;
	cmn_pll0_fb_div_low_ovrd_en.lsb = 15;
	cmnda_pll0_fb_div_low_out.msb = 9;
	cmnda_pll0_fb_div_low_out.lsb = 0;
	cmn_pll_clk_osr.msb = 2;
	cmn_pll_clk_osr.lsb = 0;
	cmn_pll_clk_div2_ratio.msb = 6;
	cmn_pll_clk_div2_ratio.lsb = 4;
	cmn_pll_clk_div2_sel.msb = 9;
	cmn_pll_clk_div2_sel.lsb = 8;
	rx_diag_smplr_osr.msb = 2;
	rx_diag_smplr_osr.lsb = 0;
	rx_psc_a0.msb = 15;
	rx_psc_a0.lsb = 0;
	rx_ree_pergcsm_eqenm_ph1.msb = 15;
	rx_ree_pergcsm_eqenm_ph1.lsb = 0;
	rx_ree_pergcsm_eqenm_ph2.msb = 15;
	rx_ree_pergcsm_eqenm_ph2.lsb = 0;
	vga_gain_accum_override_en.msb = 7;
	vga_gain_accum_override_en.lsb = 7;
	vga_gain_accum_override.msb = 4;
	vga_gain_accum_override.lsb = 0;
	vga_gain_tgt_adj_override_en.msb = 15;
	vga_gain_tgt_adj_override_en.lsb = 15;
	vga_gain_tgt_adj_override.msb = 12;
	vga_gain_tgt_adj_override.lsb = 8;
	ree_gen_sm_en_usb.msb = 3;
	ree_gen_sm_en_usb.lsb = 0;
	ree_gen_sm_en_periodic.msb = 11;
	ree_gen_sm_en_periodic.lsb = 8;
	ana_en_epath_gen_ctrl_sm_usb.msb = 7;
	ana_en_epath_gen_ctrl_sm_usb.lsb = 4;
	ana_en_epath_gen_ctrl_sm_periodic.msb = 15;
	ana_en_epath_gen_ctrl_sm_periodic.lsb = 12;
	rxda_eq_range_sel.msb = 2;
	rxda_eq_range_sel.lsb = 0;
	rxda_vga_sa_range_sel.msb = 6;
	rxda_vga_sa_range_sel.lsb = 4;
	vco_ring_select.msb = 12;
	vco_ring_select.lsb = 12;
	cmnda_pll0_v2i_prog.msb = 5;
	cmnda_pll0_v2i_prog.lsb = 4;
	cmnda_pll0_coarse_prog.msb = 2;
	cmnda_pll0_coarse_prog.lsb = 0;
	cmnda_pll0_cp_gain.msb = 8;
	cmnda_pll0_cp_gain.lsb = 0;
	cmnda_pll0_const_ndac_cntrl.msb = 11;
	cmnda_pll0_const_ndac_cntrl.lsb = 8;
	cmnda_pll0_const_pmos_cntrl.msb = 7;
	cmnda_pll0_const_pmos_cntrl.lsb = 0;
	cmnda_pll0_ptat_ndac_cntrl.msb = 5;
	cmnda_pll0_ptat_ndac_cntrl.lsb = 0;
	rxda_pi_iq_bias_trim.msb = 14;
	rxda_pi_iq_bias_trim.lsb = 12;
	rxda_pi_iq_pload_bias_trim.msb = 10;
	rxda_pi_iq_pload_bias_trim.lsb = 8;
	rxda_pi_iq_pload_trim.msb = 7;
	rxda_pi_iq_pload_trim.lsb = 0;
	rxda_pi_e_bias_trim.msb = 14;
	rxda_pi_e_bias_trim.lsb = 12;
	rxda_pi_e_pload_bias_trim.msb = 10;
	rxda_pi_e_pload_bias_trim.lsb = 8;
	rxda_pi_e_pload_trim.msb = 7;
	rxda_pi_e_pload_trim.lsb = 0;
	rxda_pi_range_sel.msb = 11;
	rxda_pi_range_sel.lsb = 8;
	rxda_pi_cal_cm_trim.msb = 7;
	rxda_pi_cal_cm_trim.lsb = 0;
	xcvr_pll_en.msb = 12;
	xcvr_pll_en.lsb = 12;
	xcvr_link_reset_n.msb = 13;
	xcvr_link_reset_n.lsb = 13;
	xcvr_power_state_req.msb = 3;
	xcvr_power_state_req.lsb = 0;
	xcvr_power_state_ack.msb = 7;
	xcvr_power_state_ack.lsb = 4;
	iso_pma_cmn_pll0_clk_datart1_div.msb = 14;
	iso_pma_cmn_pll0_clk_datart1_div.lsb = 11;
	iso_pma_cmn_pll0_clk_datart0_div.msb = 10;
	iso_pma_cmn_pll0_clk_datart0_div.lsb = 7;
	iso_pma_cmn_pll0_clk_en.msb = 5;
	iso_pma_cmn_pll0_clk_en.lsb = 5;

	pr_info("pma_pll_config() Configuring PLL0 ...\n");

	if (tmds_bit_clk_ratio == TMDS_BIT_CLOCK_RATIO_1_10) {
		if (inside_f(rx_clk_freq, 18750, 37500)) {
			set_field_value(&cmnda_pll0_ip_div, 0x1);
			set_field_value(&cmnda_pll0_hs_sym_div_sel, 0x0);
			set_field_value(&cmn_pll0_fb_div_high_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_high_out, 0x1E);
			set_field_value(&cmn_pll0_fb_div_low_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_low_out, 0x7E);
			set_field_value(&rx_diag_smplr_osr, 0x4);
			set_field_value(&rx_psc_a0, 0x8BF5);
			set_field_value(&rx_ree_pergcsm_eqenm_ph1, 0x0080);
			set_field_value(&rx_ree_pergcsm_eqenm_ph2, 0x0080);
			set_field_value(&vga_gain_accum_override_en, 0x1);
			set_field_value(&vga_gain_accum_override, 0x1A);
			set_field_value(&vga_gain_tgt_adj_override_en, 0x0);
			set_field_value(&vga_gain_tgt_adj_override, 0x00);
			set_field_value(&ree_gen_sm_en_usb, 0x1);
			set_field_value(&ree_gen_sm_en_periodic, 0x1);
			set_field_value(&ana_en_epath_gen_ctrl_sm_usb, 0x0);
			set_field_value(&ana_en_epath_gen_ctrl_sm_periodic,
					0x0);
			set_field_value(&rxda_eq_range_sel, 0x1);
			set_field_value(&rxda_vga_sa_range_sel, 0x2);
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmn_pll_clk_osr, 0x4);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x0);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmn_pll_clk_osr, 0x4);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x4);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmn_pll_clk_osr, 0x4);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x5);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmn_pll_clk_osr, 0x4);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x3);
				break;
			case CLK_RATIO_1_2:
				set_field_value(&cmn_pll_clk_osr, 0x4);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_5_8:
				set_field_value(&cmn_pll_clk_osr, 0x4);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x1);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_4:
				set_field_value(&cmn_pll_clk_osr, 0x4);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x2);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			}
		} else if (inside_f(rx_clk_freq, 37500, 75000)) {
			set_field_value(&cmnda_pll0_ip_div, 0x1);
			set_field_value(&cmnda_pll0_hs_sym_div_sel, 0x0);
			set_field_value(&cmn_pll0_fb_div_high_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_high_out, 0x0E);
			set_field_value(&cmn_pll0_fb_div_low_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_low_out, 0x3E);
			set_field_value(&rx_diag_smplr_osr, 0x3);
			set_field_value(&rx_psc_a0, 0x8BF5);
			set_field_value(&rx_ree_pergcsm_eqenm_ph1, 0x0080);
			set_field_value(&rx_ree_pergcsm_eqenm_ph2, 0x0080);
			set_field_value(&vga_gain_accum_override_en, 0x1);
			set_field_value(&vga_gain_accum_override, 0x1A);
			set_field_value(&vga_gain_tgt_adj_override_en, 0x0);
			set_field_value(&vga_gain_tgt_adj_override, 0x00);
			set_field_value(&ree_gen_sm_en_usb, 0x1);
			set_field_value(&ree_gen_sm_en_periodic, 0x1);
			set_field_value(&ana_en_epath_gen_ctrl_sm_usb, 0x0);
			set_field_value(&ana_en_epath_gen_ctrl_sm_periodic,
					0x0);
			set_field_value(&rxda_eq_range_sel, 0x1);
			set_field_value(&rxda_vga_sa_range_sel, 0x2);
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmn_pll_clk_osr, 0x3);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x0);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmn_pll_clk_osr, 0x3);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x4);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmn_pll_clk_osr, 0x3);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x5);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmn_pll_clk_osr, 0x3);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x3);
				break;
			case CLK_RATIO_1_2:
				set_field_value(&cmn_pll_clk_osr, 0x3);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_5_8:
				set_field_value(&cmn_pll_clk_osr, 0x3);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x1);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_4:
				set_field_value(&cmn_pll_clk_osr, 0x3);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x2);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			}
		} else if (inside_f(rx_clk_freq, 75000, 150000)) {
			set_field_value(&cmnda_pll0_ip_div, 0x1);
			set_field_value(&cmnda_pll0_hs_sym_div_sel, 0x0);
			set_field_value(&cmn_pll0_fb_div_high_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_high_out, 0x0A);
			set_field_value(&cmn_pll0_fb_div_low_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_low_out, 0x1A);
			set_field_value(&rx_diag_smplr_osr, 0x2);
			set_field_value(&rx_psc_a0, 0x8BF5);
			set_field_value(&rx_ree_pergcsm_eqenm_ph1, 0x0080);
			set_field_value(&rx_ree_pergcsm_eqenm_ph2, 0x0080);
			set_field_value(&vga_gain_accum_override_en, 0x1);
			set_field_value(&vga_gain_accum_override, 0x1A);
			set_field_value(&vga_gain_tgt_adj_override_en, 0x0);
			set_field_value(&vga_gain_tgt_adj_override, 0x00);
			set_field_value(&ree_gen_sm_en_usb, 0x1);
			set_field_value(&ree_gen_sm_en_periodic, 0x1);
			set_field_value(&ana_en_epath_gen_ctrl_sm_usb, 0x0);
			set_field_value(&ana_en_epath_gen_ctrl_sm_periodic,
					0x0);
			set_field_value(&rxda_eq_range_sel, 0x1);
			set_field_value(&rxda_vga_sa_range_sel, 0x2);
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmn_pll_clk_osr, 0x2);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x0);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmn_pll_clk_osr, 0x2);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x4);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmn_pll_clk_osr, 0x2);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x5);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmn_pll_clk_osr, 0x2);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x3);
				break;
			case CLK_RATIO_1_2:
				set_field_value(&cmn_pll_clk_osr, 0x2);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_5_8:
				set_field_value(&cmn_pll_clk_osr, 0x2);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x1);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_4:
				set_field_value(&cmn_pll_clk_osr, 0x2);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x2);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			}
		} else if (inside_f(rx_clk_freq, 150000, 300000)) {
			set_field_value(&cmnda_pll0_ip_div, 0x2);
			set_field_value(&cmnda_pll0_hs_sym_div_sel, 0x0);
			set_field_value(&cmn_pll0_fb_div_high_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_high_out, 0x0A);
			set_field_value(&cmn_pll0_fb_div_low_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_low_out, 0x1A);
			set_field_value(&rx_diag_smplr_osr, 0x1);
			set_field_value(&rx_psc_a0, 0x8BF5);
			set_field_value(&rx_ree_pergcsm_eqenm_ph1, 0x0080);
			set_field_value(&rx_ree_pergcsm_eqenm_ph2, 0x0080);
			set_field_value(&vga_gain_accum_override_en, 0x1);
			set_field_value(&vga_gain_accum_override, 0x1A);
			set_field_value(&vga_gain_tgt_adj_override_en, 0x0);
			set_field_value(&vga_gain_tgt_adj_override, 0x00);
			set_field_value(&ree_gen_sm_en_usb, 0x1);
			set_field_value(&ree_gen_sm_en_periodic, 0x1);
			set_field_value(&ana_en_epath_gen_ctrl_sm_usb, 0x0);
			set_field_value(&ana_en_epath_gen_ctrl_sm_periodic,
					0x0);
			set_field_value(&rxda_eq_range_sel, 0x2);
			set_field_value(&rxda_vga_sa_range_sel, 0x3);
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmn_pll_clk_osr, 0x1);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x0);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmn_pll_clk_osr, 0x1);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x4);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmn_pll_clk_osr, 0x1);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x5);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmn_pll_clk_osr, 0x1);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x3);
				break;
			case CLK_RATIO_1_2:
				set_field_value(&cmn_pll_clk_osr, 0x1);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_5_8:
				set_field_value(&cmn_pll_clk_osr, 0x1);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x1);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_4:
				set_field_value(&cmn_pll_clk_osr, 0x1);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x2);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			}
		} else if (inside_f(rx_clk_freq, 300000, 340000)) {
			set_field_value(&cmnda_pll0_ip_div, 0x3);
			set_field_value(&cmnda_pll0_hs_sym_div_sel, 0x0);
			set_field_value(&cmn_pll0_fb_div_high_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_high_out, 0x0A);
			set_field_value(&cmn_pll0_fb_div_low_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_low_out, 0x10);
			set_field_value(&rx_diag_smplr_osr, 0x0);
			set_field_value(&rx_psc_a0, 0x8BF5);
			set_field_value(&rx_ree_pergcsm_eqenm_ph1, 0x0080);
			set_field_value(&rx_ree_pergcsm_eqenm_ph2, 0x0080);
			set_field_value(&vga_gain_accum_override_en, 0x1);
			set_field_value(&vga_gain_accum_override, 0x1A);
			set_field_value(&vga_gain_tgt_adj_override_en, 0x0);
			set_field_value(&vga_gain_tgt_adj_override, 0x00);
			set_field_value(&ree_gen_sm_en_usb, 0x1);
			set_field_value(&ree_gen_sm_en_periodic, 0x1);
			set_field_value(&ana_en_epath_gen_ctrl_sm_usb, 0x0);
			set_field_value(&ana_en_epath_gen_ctrl_sm_periodic,
					0x0);
			set_field_value(&rxda_eq_range_sel, 0x3);
			set_field_value(&rxda_vga_sa_range_sel, 0x3);
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmn_pll_clk_osr, 0x0);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x0);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmn_pll_clk_osr, 0x0);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x4);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmn_pll_clk_osr, 0x0);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x5);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmn_pll_clk_osr, 0x0);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x3);
				break;
			case CLK_RATIO_1_2:
				set_field_value(&cmn_pll_clk_osr, 0x0);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_5_8:
				set_field_value(&cmn_pll_clk_osr, 0x0);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x1);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_4:
				set_field_value(&cmn_pll_clk_osr, 0x0);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x2);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			}
		} else
			pr_err("TMDS clock frequency (%d KHz) is out of range\n", rx_clk_freq);

	} else {		/* TMDS_BIT_CLOCK_RATIO_1_40 */
		if (inside_f(rx_clk_freq, 85000, 150000)) {
			set_field_value(&cmnda_pll0_ip_div, 0x1);
			set_field_value(&cmnda_pll0_hs_sym_div_sel, 0x0);
			set_field_value(&cmn_pll0_fb_div_high_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_high_out, 0x0A);
			set_field_value(&cmn_pll0_fb_div_low_ovrd_en, 0x1);
			set_field_value(&cmnda_pll0_fb_div_low_out, 0x1A);
			set_field_value(&rx_diag_smplr_osr, 0x0);
			set_field_value(&rx_psc_a0, 0x8BFD);
			set_field_value(&rx_ree_pergcsm_eqenm_ph1, 0x019F);
			set_field_value(&rx_ree_pergcsm_eqenm_ph2, 0x019F);
			set_field_value(&vga_gain_accum_override_en, 0x0);
			set_field_value(&vga_gain_accum_override, 0x01);
			set_field_value(&vga_gain_tgt_adj_override_en, 0x0);
			set_field_value(&vga_gain_tgt_adj_override, 0x1F);
			set_field_value(&ree_gen_sm_en_usb, 0x1);
			set_field_value(&ree_gen_sm_en_periodic, 0x1);
			set_field_value(&ana_en_epath_gen_ctrl_sm_usb, 0x0);
			set_field_value(&ana_en_epath_gen_ctrl_sm_periodic, 0x1);
			set_field_value(&rxda_eq_range_sel, 0x3);
			set_field_value(&rxda_vga_sa_range_sel, 0x3);
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmn_pll_clk_osr, 0x00);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x0);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmn_pll_clk_osr, 0x00);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x4);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmn_pll_clk_osr, 0x00);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x5);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmn_pll_clk_osr, 0x00);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x3);
				break;
			case CLK_RATIO_1_2:
				set_field_value(&cmn_pll_clk_osr, 0x00);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x0);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_5_8:
				set_field_value(&cmn_pll_clk_osr, 0x00);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x1);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			case CLK_RATIO_3_4:
				set_field_value(&cmn_pll_clk_osr, 0x00);
				set_field_value(&cmn_pll_clk_div2_ratio, 0x2);
				set_field_value(&cmn_pll_clk_div2_sel, 0x1);
				break;
			}
		} else
			pr_err("pma_pll_config() *E: TMDS clock frequency (%d kHz) is out of range\n",
			     rx_clk_freq);
	}

	vco_freq_khz =
	    (2048 * (u64) rx_clk_freq * (1 << cmn_pll_clk_osr.value) * tmds_bit_clk_ratio) / 2047;

	pr_info("VCO frequency (refclk: %d kHz, TMDS clk: %d kHz, OSR: %0d, tmds_bit_clk_ratio: %d) equals %llu kHz\n",
	     REFCLK_FREQ_KHZ, rx_clk_freq, 1 << cmn_pll_clk_osr.value,
	     tmds_bit_clk_ratio, vco_freq_khz);

	if (inside_f(vco_freq_khz, 3000000, 3400000 - 1000)) {
		set_field_value(&vco_ring_select, 0x0);
		set_field_value(&cmnda_pll0_v2i_prog, 0x0);
		set_field_value(&cmnda_pll0_coarse_prog, 0x3);
		switch (cmnda_pll0_fb_div_low_out.value) {
		case 0x7E:
			set_field_value(&cmnda_pll0_cp_gain, 0x78);
			break;
		case 0x3E:
			set_field_value(&cmnda_pll0_cp_gain, 0x78);
			break;
		case 0x10:
			set_field_value(&cmnda_pll0_cp_gain, 0x58);
			break;
		default:
			set_field_value(&cmnda_pll0_cp_gain, 0x78);
		}
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x04);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x0D);
	} else if (inside_f(vco_freq_khz, 3400000, 3687000 - 1000)) {
		set_field_value(&vco_ring_select, 0x1);
		set_field_value(&cmnda_pll0_v2i_prog, 0x1);
		set_field_value(&cmnda_pll0_coarse_prog, 0x7);
		switch (cmnda_pll0_fb_div_low_out.value) {
		case 0x7E:
			set_field_value(&cmnda_pll0_cp_gain, 0x68);
			break;
		case 0x3E:
			set_field_value(&cmnda_pll0_cp_gain, 0x68);
			break;
		case 0x10:
			set_field_value(&cmnda_pll0_cp_gain, 0x59);
			break;
		default:
			set_field_value(&cmnda_pll0_cp_gain, 0x68);
		}
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x8E);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x2F);
	} else if (inside_f(vco_freq_khz, 3687000, 3999000 - 1000)) {
		set_field_value(&vco_ring_select, 0x1);
		set_field_value(&cmnda_pll0_v2i_prog, 0x1);
		set_field_value(&cmnda_pll0_coarse_prog, 0x7);
		set_field_value(&cmnda_pll0_cp_gain, 0x64);
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x8E);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x2F);
	} else if (inside_f(vco_freq_khz, 3999000, 4337000 - 1000)) {
		set_field_value(&vco_ring_select, 0x1);
		set_field_value(&cmnda_pll0_v2i_prog, 0x1);
		set_field_value(&cmnda_pll0_coarse_prog, 0x7);
		set_field_value(&cmnda_pll0_cp_gain, 0x56);
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x8E);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x2F);
	} else if (inside_f(vco_freq_khz, 4337000, 4703000 - 1000)) {
		set_field_value(&vco_ring_select, 0x1);
		set_field_value(&cmnda_pll0_v2i_prog, 0x1);
		set_field_value(&cmnda_pll0_coarse_prog, 0x7);
		set_field_value(&cmnda_pll0_cp_gain, 0x58);
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x8E);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x2F);
	} else if (inside_f(vco_freq_khz, 4703000, 5101000 - 1000)) {
		set_field_value(&vco_ring_select, 0x1);
		set_field_value(&cmnda_pll0_v2i_prog, 0x1);
		set_field_value(&cmnda_pll0_coarse_prog, 0x7);
		set_field_value(&cmnda_pll0_cp_gain, 0x54);
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x04);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x0D);
	} else if (inside_f(vco_freq_khz, 5101000, 5532000 - 1000)) {
		set_field_value(&vco_ring_select, 0x1);
		set_field_value(&cmnda_pll0_v2i_prog, 0x1);
		set_field_value(&cmnda_pll0_coarse_prog, 0x7);
		set_field_value(&cmnda_pll0_cp_gain, 0x49);
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x04);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x0D);
	} else if (inside_f(vco_freq_khz, 5532000, 6000000 - 1000)) {
		set_field_value(&vco_ring_select, 0x1);
		set_field_value(&cmnda_pll0_v2i_prog, 0x1);
		set_field_value(&cmnda_pll0_coarse_prog, 0x7);
		set_field_value(&cmnda_pll0_cp_gain, 0x3E);
		set_field_value(&cmnda_pll0_const_ndac_cntrl, 0x0);
		set_field_value(&cmnda_pll0_const_pmos_cntrl, 0x04);
		set_field_value(&cmnda_pll0_ptat_ndac_cntrl, 0x0D);
	} else
		pr_err("%s VCO frequency (%llu KHz) is out of range\n", __func__, vco_freq_khz);

	if (inside_f(vco_freq_khz, 3000000, 4000000)) {
		set_field_value(&rxda_pi_iq_bias_trim, 0x5);
		set_field_value(&rxda_pi_iq_pload_bias_trim, 0x2);
		set_field_value(&rxda_pi_iq_pload_trim, 0x3F);
		set_field_value(&rxda_pi_e_bias_trim, 0x5);
		set_field_value(&rxda_pi_e_pload_bias_trim, 0x2);
		set_field_value(&rxda_pi_e_pload_trim, 0x3F);
		set_field_value(&rxda_pi_range_sel, 0x2);
		set_field_value(&rxda_pi_cal_cm_trim, 0x00);
	} else if (inside_f(vco_freq_khz, 4000000, 6000000)) {
		set_field_value(&rxda_pi_iq_bias_trim, 0x5);
		set_field_value(&rxda_pi_iq_pload_bias_trim, 0x4);
		set_field_value(&rxda_pi_iq_pload_trim, 0x3F);
		set_field_value(&rxda_pi_e_bias_trim, 0x5);
		set_field_value(&rxda_pi_e_pload_bias_trim, 0x4);
		set_field_value(&rxda_pi_e_pload_trim, 0x3F);
		set_field_value(&rxda_pi_range_sel, 0x3);
		set_field_value(&rxda_pi_cal_cm_trim, 0x00);
	}
	set_field_value(&xcvr_pll_en, 0x1);
	set_field_value(&xcvr_link_reset_n, 0x0);
	set_field_value(&xcvr_power_state_req, 0x0);
	set_field_value(&iso_pma_cmn_pll0_clk_datart1_div, 0x1);
	set_field_value(&iso_pma_cmn_pll0_clk_datart0_div, 0x2);
	set_field_value(&iso_pma_cmn_pll0_clk_en, 0x1);

	/*******************************************************
	* Register setting
	********************************************************/

	/* CMN_DIAG_PLL0_INCLK_CTRL */
	reg_val = set_reg_value(cmnda_pll0_ip_div);
	reg_val |= set_reg_value(cmnda_pll0_hs_sym_div_sel);
	write16(state, CMN_DIAG_PLL0_INCLK_CTRL_ADDR, reg_val);

	/* CMN_DIAG_PLL0_FBH_OVRD */
	reg_val = set_reg_value(cmn_pll0_fb_div_high_ovrd_en);
	reg_val |= set_reg_value(cmnda_pll0_fb_div_high_out);
	write16(state, CMN_DIAG_PLL0_FBH_OVRD_ADDR, reg_val);

	/* CMN_DIAG_PLL0_FBL_OVRD */
	reg_val = set_reg_value(cmn_pll0_fb_div_low_ovrd_en);
	reg_val |= set_reg_value(cmnda_pll0_fb_div_low_out);
	write16(state, CMN_DIAG_PLL0_FBL_OVRD_ADDR, reg_val);

	/* CMN_PLL0_DIV2SEL_OSR_CTRL */
	reg_val = set_reg_value(cmn_pll_clk_osr);
	reg_val |= set_reg_value(cmn_pll_clk_div2_ratio);
	reg_val |= set_reg_value(cmn_pll_clk_div2_sel);
	write16(state, CMN_PLL0_DIV2SEL_OSR_CTRL_ADDR, reg_val);

	/* RX_DIAG_SMPLR_OSR */
	reg_val = set_reg_value(rx_diag_smplr_osr);
	multi_write16(state, RX_DIAG_SMPLR_OSR_ADDR, reg_val);

	/* RX_PSC_A0 */
	reg_val = set_reg_value(rx_psc_a0);
	multi_write16(state, RX_PSC_A0_ADDR, reg_val);

	/* RX_REE_PERGCSM_EQENM_PH1 */
	reg_val = set_reg_value(rx_ree_pergcsm_eqenm_ph1);
	multi_write16(state, RX_REE_PERGCSM_EQENM_PH1_ADDR, reg_val);

	/* RX_REE_PERGCSM_EQENM_PH1 */
	reg_val = set_reg_value(rx_ree_pergcsm_eqenm_ph2);
	multi_write16(state, RX_REE_PERGCSM_EQENM_PH2_ADDR, reg_val);

	/* RX_REE_VGA_GAIN_OVRD */
	reg_val = set_reg_value(vga_gain_accum_override_en);
	reg_val |= set_reg_value(vga_gain_accum_override);
	reg_val |= set_reg_value(vga_gain_tgt_adj_override_en);
	reg_val |= set_reg_value(vga_gain_tgt_adj_override);
	multi_write16(state, RX_REE_VGA_GAIN_OVRD_ADDR, reg_val);

	/* RX_REE_SMGM_CTRL1 */
	reg_val = set_reg_value(ree_gen_sm_en_usb);
	reg_val |= set_reg_value(ree_gen_sm_en_periodic);
	reg_val |= set_reg_value(ana_en_epath_gen_ctrl_sm_usb);
	reg_val |= set_reg_value(ana_en_epath_gen_ctrl_sm_periodic);
	multi_write16(state, RX_REE_SMGM_CTRL1_ADDR, reg_val);

	/* RX_DIAG_DFE_CTRL2 */
	reg_val = set_reg_value(rxda_eq_range_sel);
	reg_val |= set_reg_value(rxda_vga_sa_range_sel);
	multi_write16(state, RX_DIAG_DFE_CTRL2_ADDR, reg_val);

	/* CMN_PLLSM0_USER_DEF_CTRL */
	reg_val = set_reg_value(vco_ring_select);
	write16(state, CMN_PLLSM0_USER_DEF_CTRL_ADDR, reg_val);

	/* CMN_DIAG_PLL0_V2I_TUNE */
	reg_val = set_reg_value(cmnda_pll0_v2i_prog);
	reg_val |= set_reg_value(cmnda_pll0_coarse_prog);
	write16(state, CMN_DIAG_PLL0_V2I_TUNE_ADDR, reg_val);

	/* CMN_DIAG_PLL0_CP_TUNE */
	reg_val = set_reg_value(cmnda_pll0_cp_gain);
	write16(state, CMN_DIAG_PLL0_CP_TUNE_ADDR, reg_val);

	/* CMN_DIAG_PLL0_PTATIS_TUNE1 */
	reg_val = set_reg_value(cmnda_pll0_const_ndac_cntrl);
	reg_val |= set_reg_value(cmnda_pll0_const_pmos_cntrl);
	write16(state, CMN_DIAG_PLL0_PTATIS_TUNE1_ADDR, reg_val);

	/* CMN_DIAG_PLL0_PTATIS_TUNE2 */
	reg_val = set_reg_value(cmnda_pll0_ptat_ndac_cntrl);
	write16(state, CMN_DIAG_PLL0_PTATIS_TUNE2_ADDR, reg_val);

	/* RX_DIAG_ILL_IQ_TRIM0 */
	reg_val = set_reg_value(rxda_pi_iq_bias_trim);
	reg_val |= set_reg_value(rxda_pi_iq_pload_bias_trim);
	reg_val |= set_reg_value(rxda_pi_iq_pload_trim);
	write16(state, RX_DIAG_ILL_IQ_TRIM0_ADDR, reg_val);

	/* RX_DIAG_ILL_E_TRIM0 */
	reg_val = set_reg_value(rxda_pi_e_bias_trim);
	reg_val |= set_reg_value(rxda_pi_e_pload_bias_trim);
	reg_val |= set_reg_value(rxda_pi_e_pload_trim);
	write16(state, RX_DIAG_ILL_E_TRIM0_ADDR, reg_val);

	/* RX_DIAG_ILL_IQE_TRIM2 */
	reg_val = set_reg_value(rxda_pi_range_sel);
	reg_val |= set_reg_value(rxda_pi_cal_cm_trim);
	write16(state, RX_DIAG_ILL_IQE_TRIM2_ADDR, reg_val);

	/* Enable PLL */
	/* PHY_MODE_CTL */
	reg_val = set_reg_value(xcvr_pll_en);
	reg_val |= set_reg_value(xcvr_link_reset_n);
	reg_val |= set_reg_value(xcvr_power_state_req);
	write16(state, PHY_MODE_CTL_ADDR, reg_val);

	/* Wait for PLL0 ready: */
	/* PHY_PMA_CMN_CTRL2 */
	for (i = 0; i < 20; i++) {
		if (read16(state, PHY_PMA_CMN_CTRL2_ADDR) & (1 << 0))
			break;
		msleep(10);
	}
	if (i == 20)
		pr_info("pma_pll_ready failed\n");

	/* Turn on output clocks: */
	/* PHY_PMA_CMN_CTRL2 */
	reg_val = set_reg_value(iso_pma_cmn_pll0_clk_datart1_div);
	reg_val |= set_reg_value(iso_pma_cmn_pll0_clk_datart0_div);
	reg_val |= set_reg_value(iso_pma_cmn_pll0_clk_en);
	write16(state, PHY_PMA_CMN_CTRL2_ADDR, reg_val);

	if (data_rate_change) {
		pr_info("pma_pll_config() Disable Rx Eq Training\n");
		for (i = 0; i < 3; i++) {
			reg_val =
			    read16(state, PHY_PMA_XCVR_CTRL_ADDR | (i << 6));
			reg_val &= 0xFFEF;
			write16(state, PHY_PMA_XCVR_CTRL_ADDR | (i << 6), reg_val);
		}
	}
	/* Get current power state: */
	/* PHY_MODE_CTL */
	reg_val = read16(state, PHY_MODE_CTL_ADDR);
	reg_val &= 0x00F0;
	pr_info("pma_pll_config() Current power state: 0x%02X\n", (reg_val >> 4));

	/* Deassert link reset: */
	/* PHY_MODE_CTL */
	pr_info("pma_pll_config() Deassert link reset\n");
	set_field_value(&xcvr_link_reset_n, 0x1);
	reg_val |= set_reg_value(xcvr_pll_en);
	reg_val |= set_reg_value(xcvr_link_reset_n);
	write16(state, PHY_MODE_CTL_ADDR, reg_val);

	/* Wait for xcvr_psm_ready for all the lanes */
	loop = 0;
	do {
		reg_val = (1 << 13);
		for (i = 0; i < 3; i++) {
			reg_val &= read16(state, PHY_PMA_XCVR_CTRL_ADDR | (i << 6)) & (1 << 13);
			pr_info("pma_pll_config() xcvr_psm_ready(%0d): 0x%0X\n", i, reg_val >> 13);
		}
		msleep(10);
		loop++;
	} while (!reg_val && loop < 20);
	/* Timeout */
	if (loop == 20)
		pr_err("pma_pll_config() Waiting for xcvr_psm_ready... failed\n");

	/* Set A0 power state: */
	/* PHY_MODE_CTL */
	set_field_value(&xcvr_power_state_req, 0x1);
	reg_val = set_reg_value(xcvr_pll_en);
	reg_val |= set_reg_value(xcvr_link_reset_n);
	reg_val |= set_reg_value(xcvr_power_state_req);
	write16(state, PHY_MODE_CTL_ADDR, reg_val);
	pr_info("pma_pll_config() Requested A0 power mode\n");

	/* Wait for A0 power mode acknowledged: */
	/* PHY_MODE_CTL */
	set_field_value(&xcvr_power_state_ack, 0x1);

	for (i = 0; i < 20; i++) {
		if (((read16(state, PHY_MODE_CTL_ADDR) & 0x00F0) == set_reg_value(xcvr_power_state_ack)))
			break;
		msleep(10);
	}
	if (i == 20)
		pr_err("Waiting for A0 power mode acknowledged failed\n");

	if (data_rate_change) {
		pr_info("pma_pll_config() Enable Rx Eq Training\n");
		for (i = 0; i < 3; i++) {
			reg_val =
			    read16(state, PHY_PMA_XCVR_CTRL_ADDR | (i << 6));
			reg_val |= 0x0010;
			write16(state, PHY_PMA_XCVR_CTRL_ADDR | (i << 6),
				reg_val);
		}
	}
}

clk_ratio_t clk_ratio_detect(state_struct *state,
				u32 rx_clk_freq,	/* khz */
				u32 pxl_clk_freq,	/* khz */
				u8 vic,
				pixel_encoding_t pixel_encoding,
				tmds_bit_clock_ratio_t tmds_bit_clk_ratio)
{
	clk_ratio_t clk_ratio_detected = CLK_RATIO_1_1;

	u64 tmds_freq_nominal_1_1, tmds_freq_nominal_1_1_min,
	    tmds_freq_nominal_1_1_max;
	u64 tmds_freq_nominal_5_4, tmds_freq_nominal_5_4_min,
	    tmds_freq_nominal_5_4_max;
	u64 tmds_freq_nominal_3_2, tmds_freq_nominal_3_2_min,
	    tmds_freq_nominal_3_2_max;
	u64 tmds_freq_nominal_2_1, tmds_freq_nominal_2_1_min,
	    tmds_freq_nominal_2_1_max;
	u64 tmds_freq_nominal_1_2, tmds_freq_nominal_1_2_min,
	    tmds_freq_nominal_1_2_max;
	u64 tmds_freq_nominal_5_8, tmds_freq_nominal_5_8_min,
	    tmds_freq_nominal_5_8_max;
	u64 tmds_freq_nominal_3_4, tmds_freq_nominal_3_4_min,
	    tmds_freq_nominal_3_4_max;
	u64 min, max;

	/* Check the TMDS/pixel clock ratio. */
	pr_info("VIC %0d, pixel encoding: %0d, TMDS bit clock ratio: %0d and TMDS clk %d KHz\n",
	     vic, pixel_encoding, tmds_bit_clk_ratio, rx_clk_freq);

	tmds_freq_nominal_1_1 = pxl_clk_freq;

	min = 990;
	max = 1010;

	tmds_freq_nominal_5_4 = tmds_freq_nominal_1_1;
	tmds_freq_nominal_3_2 = tmds_freq_nominal_1_1;
	tmds_freq_nominal_2_1 = tmds_freq_nominal_1_1;
	tmds_freq_nominal_1_2 = tmds_freq_nominal_1_1;
	tmds_freq_nominal_5_8 = tmds_freq_nominal_1_1;
	tmds_freq_nominal_3_4 = tmds_freq_nominal_1_1;

	/* Exclude some of the clock ratios based on pixel excoding */
	switch (pixel_encoding) {
	case PIXEL_ENCODING_YUV422:
		tmds_freq_nominal_5_4 = 0;
		tmds_freq_nominal_3_2 = 0;
		tmds_freq_nominal_2_1 = 0;
		tmds_freq_nominal_1_2 = 0;
		tmds_freq_nominal_5_8 = 0;
		tmds_freq_nominal_3_4 = 0;
		break;
	case PIXEL_ENCODING_YUV420:
		tmds_freq_nominal_5_4 = 0;
		tmds_freq_nominal_3_2 = 0;
		tmds_freq_nominal_2_1 = 0;
		break;
	default:		/* RGB/YUV444 */
		tmds_freq_nominal_1_2 = 0;
		tmds_freq_nominal_5_8 = 0;
		tmds_freq_nominal_3_4 = 0;
	}

	tmds_freq_nominal_1_1_min =
	    min * tmds_freq_nominal_1_1 * 10 * 1 / (tmds_bit_clk_ratio * 1000 * 1);
	tmds_freq_nominal_1_1_max =
	    max * tmds_freq_nominal_1_1 * 10 * 1 / (tmds_bit_clk_ratio * 1000 * 1);
	tmds_freq_nominal_5_4_min =
	    min * tmds_freq_nominal_5_4 * 10 * 5 / (tmds_bit_clk_ratio * 1000 * 4);
	tmds_freq_nominal_5_4_max =
	    max * tmds_freq_nominal_5_4 * 10 * 5 / (tmds_bit_clk_ratio * 1000 * 4);
	tmds_freq_nominal_3_2_min =
	    min * tmds_freq_nominal_3_2 * 10 * 3 / (tmds_bit_clk_ratio * 1000 * 2);
	tmds_freq_nominal_3_2_max =
	    max * tmds_freq_nominal_3_2 * 10 * 3 / (tmds_bit_clk_ratio * 1000 * 2);
	tmds_freq_nominal_2_1_min =
	    min * tmds_freq_nominal_2_1 * 10 * 2 / (tmds_bit_clk_ratio * 1000 * 1);
	tmds_freq_nominal_2_1_max =
	    max * tmds_freq_nominal_2_1 * 10 * 2 / (tmds_bit_clk_ratio * 1000 * 1);
	tmds_freq_nominal_1_2_min =
	    min * tmds_freq_nominal_1_2 * 10 * 1 / (tmds_bit_clk_ratio * 1000 * 2);
	tmds_freq_nominal_1_2_max =
	    max * tmds_freq_nominal_1_2 * 10 * 1 / (tmds_bit_clk_ratio * 1000 * 2);
	tmds_freq_nominal_5_8_min =
	    min * tmds_freq_nominal_5_8 * 10 * 5 / (tmds_bit_clk_ratio * 1000 * 8);
	tmds_freq_nominal_5_8_max =
	    max * tmds_freq_nominal_5_8 * 10 * 5 / (tmds_bit_clk_ratio * 1000 * 8);
	tmds_freq_nominal_3_4_min =
	    min * tmds_freq_nominal_3_4 * 10 * 3 / (tmds_bit_clk_ratio * 1000 * 4);
	tmds_freq_nominal_3_4_max =
	    max * tmds_freq_nominal_3_4 * 10 * 3 / (tmds_bit_clk_ratio * 1000 * 4);

	if (rx_clk_freq > tmds_freq_nominal_1_1_min
	    && rx_clk_freq < tmds_freq_nominal_1_1_max) {
		clk_ratio_detected = CLK_RATIO_1_1;
		pr_info("Detected TMDS/pixel clock ratio of 1:1\n");
	} else if (rx_clk_freq > tmds_freq_nominal_5_4_min
		   && rx_clk_freq < tmds_freq_nominal_5_4_max) {
		clk_ratio_detected = CLK_RATIO_5_4;
		pr_info("Detected TMDS/pixel clock ratio of 5:4\n");
	} else if (rx_clk_freq > tmds_freq_nominal_3_2_min
		   && rx_clk_freq < tmds_freq_nominal_3_2_max) {
		clk_ratio_detected = CLK_RATIO_3_2;
		pr_info("Detected TMDS/pixel clock ratio of 3:2\n");
	} else if (rx_clk_freq > tmds_freq_nominal_2_1_min
		   && rx_clk_freq < tmds_freq_nominal_2_1_max) {
		clk_ratio_detected = CLK_RATIO_2_1;
		pr_info("Detected TMDS/pixel clock ratio of 2:1\n");
	} else if (rx_clk_freq > tmds_freq_nominal_1_2_min
		   && rx_clk_freq < tmds_freq_nominal_1_2_max) {
		clk_ratio_detected = CLK_RATIO_1_2;
		pr_info("Detected TMDS/pixel clock ratio of 1:2\n");
	} else if (rx_clk_freq > tmds_freq_nominal_5_8_min
		   && rx_clk_freq < tmds_freq_nominal_5_8_max) {
		clk_ratio_detected = CLK_RATIO_5_8;
		pr_info("Detected TMDS/pixel clock ratio of 5:8\n");
	} else if (rx_clk_freq > tmds_freq_nominal_3_4_min
		   && rx_clk_freq < tmds_freq_nominal_3_4_max) {
		clk_ratio_detected = CLK_RATIO_3_4;
		pr_info("Detected TMDS/pixel clock ratio of 3:4\n");
	} else {
		pr_err("Failed to detected TMDS/pixel clock ratio\n");
		pr_err("VIC: %02d and TMDS clock of %d KHz\n", vic, rx_clk_freq);
	}

	return clk_ratio_detected;
}
