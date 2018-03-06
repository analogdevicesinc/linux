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
 * Copyright 2017-2018 NXP
 *
 ******************************************************************************
 *
 * API_AFE_t28hpc_hdmitx.c
 *
 ******************************************************************************
 */

#include <drm/drmP.h>
#include "API_AFE_t28hpc_hdmitx.h"
#include "imx-hdp.h"

static char inside(u32 value, u32 left_sharp_corner,
		  u32 right_sharp_corner)
{
	if (value < left_sharp_corner)
		return false;
	if (value > right_sharp_corner)
		return false;
	return true;
}

int phy_cfg_t28hpc(state_struct *state, int num_lanes, struct drm_display_mode *mode, int bpp,
		VIC_PXL_ENCODING_FORMAT format, bool pixel_clk_from_phy)
{

	const int phy_reset_workaround = 1;
	unsigned int vco_freq;
	unsigned char k;
	uint32_t reg_val;
	uint32_t pixel_freq_khz = mode->clock;
	uint32_t character_clock_ratio_num = 1;
	uint32_t character_clock_ratio_den = 1;
	uint32_t character_freq_khz;
	const unsigned int refclk_freq_khz = 27000;
	unsigned int ftemp, ftemp2;

	clk_ratio_t clk_ratio = 0;
	reg_field_t cmnda_pll0_hs_sym_div_sel;
	reg_field_t cmnda_pll0_ip_div;
	reg_field_t cmnda_pll0_fb_div_low;
	reg_field_t cmnda_pll0_fb_div_high;
	reg_field_t cmn_ref_clk_dig_div;
	reg_field_t divider_scaler;
	reg_field_t cmnda_hs_clk_0_sel;
	reg_field_t cmnda_hs_clk_1_sel;
	reg_field_t tx_subrate;
	reg_field_t voltage_to_current_coarse;
	reg_field_t voltage_to_current;
	reg_field_t ndac_ctrl;
	reg_field_t pmos_ctrl;
	reg_field_t ptat_ndac_ctrl;
	reg_field_t charge_pump_gain;
	reg_field_t vco_ring_select;
	reg_field_t pll_feedback_divider_total;
	reg_field_t cmnda_pll0_pxdiv_high;
	reg_field_t cmnda_pll0_pxdiv_low;
	reg_field_t coarse_code;
	reg_field_t v2i_code;
	reg_field_t vco_cal_code;

	cmnda_pll0_fb_div_high.value = 0x00A;
	ftemp = pixel_freq_khz;

	DRM_INFO("mode:%dx%dp%d, pixel clock %u kHz\n", mode->hdisplay, mode->vdisplay, mode->vrefresh, ftemp);

	/* Set field position */
	cmnda_pll0_hs_sym_div_sel.msb = 9;
	cmnda_pll0_hs_sym_div_sel.lsb = 8;
	cmnda_pll0_ip_div.msb = 7;
	cmnda_pll0_ip_div.lsb = 0;
	cmnda_pll0_fb_div_low.msb = 9;
	cmnda_pll0_fb_div_low.lsb = 0;
	cmnda_pll0_fb_div_high.msb = 9;
	cmnda_pll0_fb_div_high.lsb = 0;
	cmn_ref_clk_dig_div.msb = 13;
	cmn_ref_clk_dig_div.lsb = 12;
	divider_scaler.msb = 14;
	divider_scaler.lsb = 12;
	cmnda_hs_clk_0_sel.msb = 1;
	cmnda_hs_clk_0_sel.lsb = 0;
	cmnda_hs_clk_1_sel.msb = 1;
	cmnda_hs_clk_1_sel.lsb = 0;
	tx_subrate.msb = 2;
	tx_subrate.lsb = 0;
	voltage_to_current_coarse.msb = 2;
	voltage_to_current_coarse.lsb = 0;
	voltage_to_current.msb = 5;
	voltage_to_current.lsb = 4;
	ndac_ctrl.msb = 11;
	ndac_ctrl.lsb = 8;
	pmos_ctrl.msb = 7;
	pmos_ctrl.lsb = 0;
	ptat_ndac_ctrl.msb = 5;
	ptat_ndac_ctrl.lsb = 0;
	charge_pump_gain.msb = 8;
	charge_pump_gain.lsb = 0;
	vco_ring_select.msb = 12;
	vco_ring_select.lsb = 12;
	pll_feedback_divider_total.msb = 9;
	pll_feedback_divider_total.lsb = 0;
	cmnda_pll0_pxdiv_high.msb = 9;
	cmnda_pll0_pxdiv_high.lsb = 0;
	cmnda_pll0_pxdiv_low.msb = 9;
	cmnda_pll0_pxdiv_low.lsb = 0;
	coarse_code.msb = 7;
	coarse_code.lsb = 0;
	v2i_code.msb = 3;
	v2i_code.lsb = 0;
	vco_cal_code.msb = 8;
	vco_cal_code.lsb = 0;

	if (phy_reset_workaround) {
		/* register PHY_PMA_ISOLATION_CTRL */
		Afe_write(state, 0xC81F, 0xD000);	/*  enable PHY isolation mode only for CMN */
		// register PHY_PMA_ISO_PLL_CTRL1
		reg_val = Afe_read(state, 0xC812);
		reg_val &= 0xFF00;
		reg_val |= 0x0012;
		Afe_write(state, 0xC812, reg_val);	/* set cmn_pll0_clk_datart1_div/cmn_pll0_clk_datart0_div dividers */
		/* register PHY_ISO_CMN_CTRL */
		Afe_write(state, 0xC010, 0x0000);	/* assert PHY reset from isolation register */
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0000);	/* assert PMA CMN reset */
		/* register XCVR_DIAG_BIDI_CTRL */
		for (k = 0; k < num_lanes; k++) {
			Afe_write(state, 0x40E8 | (k << 9), 0x00FF);
		}
	}
	/*---------------------------------------------------------------
	 * Describing Task phy_cfg_hdp
	 * --------------------------------------------------------------*/
	/* register PHY_PMA_CMN_CTRL1 */
	reg_val = Afe_read(state, 0xC800);
	reg_val &= 0xFFF7;
	reg_val |= 0x0008;
	Afe_write(state, 0xC800, reg_val);

	/* register CMN_DIAG_PLL0_TEST_MODE */
	Afe_write(state, 0x01C4, 0x0020);
	/* register CMN_PSM_CLK_CTRL */
	Afe_write(state, 0x0061, 0x0016);

	switch (format) {
	case YCBCR_4_2_2:
		clk_ratio = CLK_RATIO_1_1;
		character_clock_ratio_num = 1;
		character_clock_ratio_den = 1;
		break;
	case YCBCR_4_2_0:
		switch (bpp) {
		case 8:
			clk_ratio = CLK_RATIO_1_2;
			character_clock_ratio_num = 1;
			character_clock_ratio_den = 2;
			break;
		case 10:
			clk_ratio = CLK_RATIO_5_8;
			character_clock_ratio_num = 5;
			character_clock_ratio_den = 8;
			break;
		case 12:
			clk_ratio = CLK_RATIO_3_4;
			character_clock_ratio_num = 3;
			character_clock_ratio_den = 4;
			break;
		case 16:
			clk_ratio = CLK_RATIO_1_1;
			character_clock_ratio_num = 1;
			character_clock_ratio_den = 1;
			break;
		default:
			DRM_WARN("Invalid ColorDepth\n");
		}
		break;

	default:
		switch (bpp) {
			/* Assume RGB */
		case 10:
			clk_ratio = CLK_RATIO_5_4;
			character_clock_ratio_num = 5;
			character_clock_ratio_den = 4;
			break;
		case 12:
			clk_ratio = CLK_RATIO_3_2;
			character_clock_ratio_num = 3;
			character_clock_ratio_den = 2;
			break;
		case 16:
			clk_ratio = CLK_RATIO_2_1;
			character_clock_ratio_num = 2;
			character_clock_ratio_den = 1;
			break;
		default:
			clk_ratio = CLK_RATIO_1_1;
			character_clock_ratio_num = 1;
			character_clock_ratio_den = 1;
		}
	}

	character_freq_khz = pixel_freq_khz *
	    character_clock_ratio_num / character_clock_ratio_den;
	ftemp = pixel_freq_khz;
	ftemp2 = character_freq_khz;
	DRM_INFO
	    ("Pixel clock frequency: %u kHz, character clock frequency: %u, color depth is %0d-bit.\n",
	     ftemp, ftemp2, bpp);
	if (pixel_clk_from_phy == 0) {

		/* --------------------------------------------------------------
		 * Describing Task phy_cfg_hdmi_pll0_0pt5736 (Clock is input)
		 * --------------------------------------------------------------*/

		/* register CMN_PLL0_VCOCAL_INIT_TMR */
		Afe_write(state, 0x0084, 0x0064);
		/* register CMN_PLL0_VCOCAL_ITER_TMR */
		Afe_write(state, 0x0085, 0x000A);
		/* register PHY_HDP_CLK_CTL */
		reg_val = Afe_read(state, 0xC009);
		reg_val &= 0x00FF;
		reg_val |= 0x1200;
		Afe_write(state, 0xC009, reg_val);

		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			if (inside(pixel_freq_khz, 340000, 600000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x3C);
				set_field_value(&cmnda_pll0_fb_div_low, 0x24A);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x06);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&pll_feedback_divider_total,
						600);
			} else if (inside(pixel_freq_khz, 170000, 340000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x22);
				set_field_value(&cmnda_pll0_fb_div_low, 0x146);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x07);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						340);
			} else if (inside(pixel_freq_khz, 85000, 170000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				set_field_value(&cmnda_pll0_ip_div, 0x11);
				set_field_value(&cmnda_pll0_fb_div_low, 0x146);
				set_field_value(&cmn_ref_clk_dig_div, 0x00);
				set_field_value(&divider_scaler, 0x07);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						340);
			} else if (inside(pixel_freq_khz, 42500, 85000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				set_field_value(&cmnda_pll0_ip_div, 0x08);
				set_field_value(&cmnda_pll0_fb_div_low, 0x132);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						320);
			} else if (inside(pixel_freq_khz, 25000, 42500)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				set_field_value(&cmnda_pll0_ip_div, 0x05);
				set_field_value(&cmnda_pll0_fb_div_low, 0x182);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						400);
			} else {
				ftemp = pixel_freq_khz;
				DRM_WARN
				    ("Pixel clock frequency (%u) is outside of the supported range\n",
				     ftemp);
			}
			break;

		case CLK_RATIO_5_4:
			if (inside(pixel_freq_khz, 272000, 480000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x30);
				set_field_value(&cmnda_pll0_fb_div_low, 0x24A);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x05);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&pll_feedback_divider_total,
						600);
			} else if (inside(pixel_freq_khz, 136000, 272000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x1A);
				set_field_value(&cmnda_pll0_fb_div_low, 0x137);
				set_field_value(&cmn_ref_clk_dig_div, 0x02);
				set_field_value(&divider_scaler, 0x04);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						325);
			} else if (inside(pixel_freq_khz, 68000, 136000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				set_field_value(&cmnda_pll0_ip_div, 0x0D);
				set_field_value(&cmnda_pll0_fb_div_low, 0x137);
				set_field_value(&cmn_ref_clk_dig_div, 0x02);
				set_field_value(&divider_scaler, 0x02);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						325);
			} else if (inside(pixel_freq_khz, 34000, 68000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				set_field_value(&cmnda_pll0_ip_div, 0x06);
				set_field_value(&cmnda_pll0_fb_div_low, 0x11E);
				set_field_value(&cmn_ref_clk_dig_div, 0x02);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						300);
			} else if (inside(pixel_freq_khz, 25000, 34000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				set_field_value(&cmnda_pll0_ip_div, 0x04);
				set_field_value(&cmnda_pll0_fb_div_low, 0x182);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						400);
			} else {
				ftemp = pixel_freq_khz;
				DRM_ERROR
				    ("Pixel clock frequency (%u) is outside of the supported range\n",
				     ftemp);
			}
			break;
		case CLK_RATIO_3_2:
			if (inside(pixel_freq_khz, 226000, 400000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x28);
				set_field_value(&cmnda_pll0_fb_div_low, 0x24A);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x04);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&pll_feedback_divider_total,
						600);
			} else if (inside(pixel_freq_khz, 113000, 226000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x16);
				set_field_value(&cmnda_pll0_fb_div_low, 0x13C);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x05);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						330);
			} else if (inside(pixel_freq_khz, 56000, 113000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				set_field_value(&cmnda_pll0_ip_div, 0x0B);
				set_field_value(&cmnda_pll0_fb_div_low, 0x13C);
				set_field_value(&cmn_ref_clk_dig_div, 0x00);
				set_field_value(&divider_scaler, 0x05);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						330);
			} else if (inside(pixel_freq_khz, 28000, 56000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				set_field_value(&cmnda_pll0_ip_div, 0x06);
				set_field_value(&cmnda_pll0_fb_div_low, 0x15A);
				set_field_value(&cmn_ref_clk_dig_div, 0x02);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						360);
			} else if (inside(pixel_freq_khz, 25000, 28000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmnda_pll0_fb_div_low, 0x15A);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						360);
			} else {
				ftemp = pixel_freq_khz;
				DRM_ERROR
				    ("Pixel clock frequency (%u) is outside of the supported range\n",
				     ftemp);
			}
			break;
		case CLK_RATIO_2_1:
			if (inside(pixel_freq_khz, 170000, 300000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x22);
				set_field_value(&cmnda_pll0_fb_div_low, 0x29A);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x06);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&pll_feedback_divider_total,
						680);
			} else if (inside(pixel_freq_khz, 85000, 170000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x11);
				set_field_value(&cmnda_pll0_fb_div_low, 0x146);
				set_field_value(&cmn_ref_clk_dig_div, 0x00);
				set_field_value(&divider_scaler, 0x07);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						340);
			} else if (inside(pixel_freq_khz, 42500, 85000)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				set_field_value(&cmnda_pll0_ip_div, 0x08);
				set_field_value(&cmnda_pll0_fb_div_low, 0x132);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						320);
			} else if (inside(pixel_freq_khz, 25000, 42500)) {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				set_field_value(&cmnda_pll0_ip_div, 0x05);
				set_field_value(&cmnda_pll0_fb_div_low, 0x182);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&pll_feedback_divider_total,
						400);
			} else {
				ftemp = pixel_freq_khz;
				DRM_ERROR
				    ("Pixel clock frequency (%u) is outside of the supported range\n",
				     ftemp);
			}
			break;
		case CLK_RATIO_1_2:
			if (!(inside(pixel_freq_khz, 594000, 594000))) {
				ftemp = pixel_freq_khz;
				DRM_ERROR
				    ("Pixel clock frequency (%u) is outside of the supported range\n",
				     ftemp);
			} else {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				set_field_value(&cmnda_pll0_ip_div, 0x3C);
				set_field_value(&cmnda_pll0_fb_div_low, 0x24A);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x06);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&pll_feedback_divider_total,
						600);
			}
			break;
		case CLK_RATIO_5_8:
			if (!(inside(pixel_freq_khz, 594000, 594000))) {
				ftemp = pixel_freq_khz;
				DRM_ERROR
				    ("Pixel clock frequency (%u) is outside of the supported range\n",
				     ftemp);
			} else {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x3C);
				set_field_value(&cmnda_pll0_fb_div_low, 0x169);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x06);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&pll_feedback_divider_total,
						375);
			}
			break;
		case CLK_RATIO_3_4:
			if (!(inside(pixel_freq_khz, 594000, 594000))) {
				ftemp = pixel_freq_khz;
				DRM_ERROR
				    ("Pixel clock frequency (%u) is outside of the supported range\n",
				     ftemp);
			} else {
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				set_field_value(&cmnda_pll0_ip_div, 0x3C);
				set_field_value(&cmnda_pll0_fb_div_low, 0x1B4);
				set_field_value(&cmn_ref_clk_dig_div, 0x03);
				set_field_value(&divider_scaler, 0x06);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&pll_feedback_divider_total,
						450);
			}
			break;
		}
		vco_freq =
		    pixel_freq_khz * pll_feedback_divider_total.value /
		    cmnda_pll0_ip_div.value;
		ftemp = vco_freq;
		DRM_INFO("VCO frequency is %u kHz\n", ftemp);

		if (inside(vco_freq, 1700000, 2000000)) {
			set_field_value(&voltage_to_current_coarse, 0x04);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x09);
			set_field_value(&ptat_ndac_ctrl, 0x09);
			switch (pll_feedback_divider_total.value) {
			case 300:
				set_field_value(&charge_pump_gain, 0x82);
				break;
			case 320:
				set_field_value(&charge_pump_gain, 0x83);
				break;
			case 325:
				set_field_value(&charge_pump_gain, 0x83);
				break;
			case 330:
				set_field_value(&charge_pump_gain, 0x84);
				break;
			case 340:
				set_field_value(&charge_pump_gain, 0x84);
				break;
			case 360:
				set_field_value(&charge_pump_gain, 0x86);
				break;
			case 400:
				set_field_value(&charge_pump_gain, 0xA2);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else if (inside(vco_freq, 2000000, 2400000)) {
			set_field_value(&voltage_to_current_coarse, 0x04);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x09);
			set_field_value(&ptat_ndac_ctrl, 0x09);
			switch (pll_feedback_divider_total.value) {
			case 300:
				set_field_value(&charge_pump_gain, 0x47);
				break;
			case 320:
				set_field_value(&charge_pump_gain, 0x4B);
				break;
			case 325:
				set_field_value(&charge_pump_gain, 0x4C);
				break;
			case 330:
				set_field_value(&charge_pump_gain, 0x80);
				break;
			case 340:
				set_field_value(&charge_pump_gain, 0x81);
				break;
			case 360:
				set_field_value(&charge_pump_gain, 0x82);
				break;
			case 400:
				set_field_value(&charge_pump_gain, 0x84);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else if (inside(vco_freq, 2400000, 2800000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			switch (pll_feedback_divider_total.value) {
			case 300:
				set_field_value(&charge_pump_gain, 0x43);
				break;
			case 320:
				set_field_value(&charge_pump_gain, 0x45);
				break;
			case 325:
				set_field_value(&charge_pump_gain, 0x45);
				break;
			case 330:
				set_field_value(&charge_pump_gain, 0x45);
				break;
			case 340:
				set_field_value(&charge_pump_gain, 0x86);
				break;
			case 360:
				set_field_value(&charge_pump_gain, 0x4A);
				break;
			case 400:
				set_field_value(&charge_pump_gain, 0x81);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else if (inside(vco_freq, 2800000, 3400000)) {
			set_field_value(&voltage_to_current_coarse, 0x06);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			switch (pll_feedback_divider_total.value) {
			case 300:
				set_field_value(&charge_pump_gain, 0x3D);
				break;
			case 320:
				set_field_value(&charge_pump_gain, 0x41);
				break;
			case 325:
				set_field_value(&charge_pump_gain, 0x41);
				break;
			case 330:
				set_field_value(&charge_pump_gain, 0x41);
				break;
			case 340:
				set_field_value(&charge_pump_gain, 0x42);
				break;
			case 360:
				set_field_value(&charge_pump_gain, 0x43);
				break;
			case 400:
				set_field_value(&charge_pump_gain, 0x46);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else if (inside(vco_freq, 3400000, 3900000)) {
			set_field_value(&voltage_to_current_coarse, 0x04);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x07);
			set_field_value(&ptat_ndac_ctrl, 0x0F);
			switch (pll_feedback_divider_total.value) {
			case 375:
				set_field_value(&charge_pump_gain, 0x41);
				break;
			case 600:
				set_field_value(&charge_pump_gain, 0x82);
				break;
			case 680:
				set_field_value(&charge_pump_gain, 0x85);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else if (inside(vco_freq, 3900000, 4500000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x07);
			set_field_value(&ptat_ndac_ctrl, 0x0F);
			switch (pll_feedback_divider_total.value) {
			case 450:
				set_field_value(&charge_pump_gain, 0x41);
				break;
			case 600:
				set_field_value(&charge_pump_gain, 0x4B);
				break;
			case 680:
				set_field_value(&charge_pump_gain, 0x82);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else if (inside(vco_freq, 4500000, 5200000)) {
			set_field_value(&voltage_to_current_coarse, 0x06);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			switch (pll_feedback_divider_total.value) {
			case 600:
				set_field_value(&charge_pump_gain, 0x45);
				break;
			case 680:
				set_field_value(&charge_pump_gain, 0x4A);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else if (inside(vco_freq, 5200000, 6000000)) {
			set_field_value(&voltage_to_current_coarse, 0x07);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			switch (pll_feedback_divider_total.value) {
			case 600:
				set_field_value(&charge_pump_gain, 0x42);
				break;
			case 680:
				set_field_value(&charge_pump_gain, 0x45);
				break;
			default:
				DRM_WARN
				    ("pll_feedback_divider_total (%0d) is outside of the supported range for vco_freq equal %u\n",
				     pll_feedback_divider_total.value, ftemp);
			}
		} else
			DRM_WARN
			    ("VCO frequency %u kHz is outside of the supported range\n",
			     ftemp);

		/* register CMN_DIAG_PLL0_INCLK_CTRL */
		reg_val = set_reg_value(cmnda_pll0_hs_sym_div_sel);
		reg_val |= set_reg_value(cmnda_pll0_ip_div);
		Afe_write(state, 0x01CA, reg_val);
		/* register CMN_DIAG_PLL0_FBL_OVRD */
		reg_val = set_reg_value(cmnda_pll0_fb_div_low);
		reg_val |= (1 << 15);
		Afe_write(state, 0x01C1, reg_val);
		/* register PHY_PMA_CMN_CTRL1 */
		reg_val = Afe_read(state, 0xC800);
		reg_val &= 0xCFFF;
		reg_val |= set_reg_value(cmn_ref_clk_dig_div);
		Afe_write(state, 0xC800, reg_val);
		/* register CMN_CDIAG_REFCLK_CTRL */
		reg_val = Afe_read(state, 0x0062);
		reg_val &= 0x8FFF;
		reg_val |= set_reg_value(divider_scaler);
		reg_val |= 0x00C0;
		Afe_write(state, 0x0062, reg_val);
		/* register CMN_DIAG_HSCLK_SEL */
		reg_val = Afe_read(state, 0x01E0);
		reg_val &= 0xFF00;
		reg_val |= (cmnda_hs_clk_0_sel.value >> 1) << 0;
		reg_val |= (cmnda_hs_clk_1_sel.value >> 1) << 4;
		Afe_write(state, 0x01E0, reg_val);

		/* register XCVR_DIAG_HSCLK_SEL */
		for (k = 0; k < num_lanes; k++) {
			reg_val = Afe_read(state, 0x40E1 | (k << 9));
			reg_val &= 0xCFFF;
			reg_val |= (cmnda_hs_clk_0_sel.value >> 1) << 12;
			Afe_write(state, 0x40E1 | (k << 9), reg_val);
		}

		/* register TX_DIAG_TX_CTRL */
		for (k = 0; k < num_lanes; k++) {
			reg_val = Afe_read(state, 0x41E0 | (k << 9));
			reg_val &= 0xFF3F;
			reg_val |= (tx_subrate.value >> 1) << 6;
			Afe_write(state, 0x41E0 | (k << 9), reg_val);
		}

		/* register CMN_PLLSM0_USER_DEF_CTRL */
		reg_val = set_reg_value(vco_ring_select);
		Afe_write(state, 0x002F, reg_val);
		/* register CMN_DIAG_PLL0_OVRD */
		Afe_write(state, 0x01C2, 0x0000);
		/* register CMN_DIAG_PLL0_FBH_OVRD */
		reg_val = set_reg_value(cmnda_pll0_fb_div_high);
		reg_val |= (1 << 15);
		Afe_write(state, 0x01C0, reg_val);
		/* register CMN_DIAG_PLL0_V2I_TUNE */
		reg_val = set_reg_value(voltage_to_current_coarse);
		reg_val |= set_reg_value(voltage_to_current);
		Afe_write(state, 0x01C5, reg_val);
		/* register CMN_DIAG_PLL0_PTATIS_TUNE1 */
		reg_val = set_reg_value(pmos_ctrl);
		reg_val |= set_reg_value(ndac_ctrl);
		Afe_write(state, 0x01C8, reg_val);
		/* register CMN_DIAG_PLL0_PTATIS_TUNE2 */
		reg_val = set_reg_value(ptat_ndac_ctrl);
		Afe_write(state, 0x01C9, reg_val);
		/* register CMN_DIAG_PLL0_CP_TUNE */
		reg_val = set_reg_value(charge_pump_gain);
		Afe_write(state, 0x01C6, reg_val);
		/* register CMN_DIAG_PLL0_LF_PROG */
		Afe_write(state, 0x01C7, 0x0008);

		/* register XCVR_DIAG_PLLDRC_CTRL */
		for (k = 0; k < num_lanes; k++) {
			reg_val = Afe_read(state, 0x40E0 | (k << 9));
			reg_val &= 0xBFFF;
			Afe_write(state, 0x40E0 | (k << 9), reg_val);
		}

	} else {
		/* Describing task phy_cfg_hdmi_pll0_0pt099_ver2 (Clock is OUTPUT) */
		if (inside(pixel_freq_khz, 27000, 27000)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						240);
				set_field_value(&cmnda_pll0_fb_div_low, 0xBC);
				set_field_value(&cmnda_pll0_fb_div_high, 0x30);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x26);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x26);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						300);
				set_field_value(&cmnda_pll0_fb_div_low, 0x0EC);
				set_field_value(&cmnda_pll0_fb_div_high, 0x03C);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x030);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x030);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						360);
				set_field_value(&cmnda_pll0_fb_div_low, 0x11C);
				set_field_value(&cmnda_pll0_fb_div_high, 0x048);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x03A);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x03A);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						240);
				set_field_value(&cmnda_pll0_fb_div_low, 0x0BC);
				set_field_value(&cmnda_pll0_fb_div_high, 0x030);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x026);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x026);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				break;
			default:
				break;
			}
		} else if (inside(pixel_freq_khz, 54000, 54000)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						480);
				set_field_value(&cmnda_pll0_fb_div_low, 0x17C);
				set_field_value(&cmnda_pll0_fb_div_high, 0x060);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x026);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x026);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmnda_pll0_ip_div, 0x04);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						400);
				set_field_value(&cmnda_pll0_fb_div_low, 0x13C);
				set_field_value(&cmnda_pll0_fb_div_high, 0x050);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x017);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x017);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmnda_pll0_ip_div, 0x04);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						480);
				set_field_value(&cmnda_pll0_fb_div_low, 0x17C);
				set_field_value(&cmnda_pll0_fb_div_high, 0x060);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x01C);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x01C);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						240);
				set_field_value(&cmnda_pll0_fb_div_low, 0x0bc);
				set_field_value(&cmnda_pll0_fb_div_high, 0x030);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x012);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x012);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			default:
				break;
			}
		} else if (inside(pixel_freq_khz, 74250, 74250)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						660);
				set_field_value(&cmnda_pll0_fb_div_low, 0x20c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x084);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x026);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x026);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x03);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmnda_pll0_ip_div, 0x04);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						550);
				set_field_value(&cmnda_pll0_fb_div_low, 0x1b4);
				set_field_value(&cmnda_pll0_fb_div_high, 0x06e);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x017);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x017);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x04);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmnda_pll0_ip_div, 0x04);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						660);
				set_field_value(&cmnda_pll0_fb_div_low, 0x20c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x084);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x01c);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x01c);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						330);
				set_field_value(&cmnda_pll0_fb_div_low, 0x104);
				set_field_value(&cmnda_pll0_fb_div_high, 0x042);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x012);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x012);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			default:
				break;
			}
		} else if (inside(pixel_freq_khz, 99000, 99000)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						440);
				set_field_value(&cmnda_pll0_fb_div_low, 0x15c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x058);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x012);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x012);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						275);
				set_field_value(&cmnda_pll0_fb_div_low, 0x0d8);
				set_field_value(&cmnda_pll0_fb_div_high, 0x037);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x00b);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x00a);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						330);
				set_field_value(&cmnda_pll0_fb_div_low, 0x104);
				set_field_value(&cmnda_pll0_fb_div_high, 0x042);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x00d);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x00d);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						440);
				set_field_value(&cmnda_pll0_fb_div_low, 0x15c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x058);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x012);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x012);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			default:
				break;
			}
		} else if (inside(pixel_freq_khz, 148500, 148500)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						660);
				set_field_value(&cmnda_pll0_fb_div_low, 0x20c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x084);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x012);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x012);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x02);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmnda_pll0_ip_div, 0x04);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						550);
				set_field_value(&cmnda_pll0_fb_div_low, 0x1b4);
				set_field_value(&cmnda_pll0_fb_div_high, 0x06e);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x00b);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x00a);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						495);
				set_field_value(&cmnda_pll0_fb_div_low, 0x188);
				set_field_value(&cmnda_pll0_fb_div_high, 0x063);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x00d);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x00d);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						660);
				set_field_value(&cmnda_pll0_fb_div_low, 0x20c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x084);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x012);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x012);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x02);
				set_field_value(&cmnda_hs_clk_1_sel, 0x02);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			default:
				break;
			}
		} else if (inside(pixel_freq_khz, 198000, 198000)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						220);
				set_field_value(&cmnda_pll0_fb_div_low, 0x0ac);
				set_field_value(&cmnda_pll0_fb_div_high, 0x02c);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x003);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x003);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			case CLK_RATIO_5_4:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						550);
				set_field_value(&cmnda_pll0_fb_div_low, 0x1b4);
				set_field_value(&cmnda_pll0_fb_div_high, 0x06e);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x00b);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x00a);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						330);
				set_field_value(&cmnda_pll0_fb_div_low, 0x104);
				set_field_value(&cmnda_pll0_fb_div_high, 0x042);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x006);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x005);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						440);
				set_field_value(&cmnda_pll0_fb_div_low, 0x15c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x058);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x008);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x008);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			default:
				break;
			}
		} else if (inside(pixel_freq_khz, 297000, 297000)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						330);
				set_field_value(&cmnda_pll0_fb_div_low, 0x104);
				set_field_value(&cmnda_pll0_fb_div_high, 0x042);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x003);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x003);
				set_field_value(&vco_ring_select, 0x00);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			case CLK_RATIO_3_2:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						495);
				set_field_value(&cmnda_pll0_fb_div_low, 0x188);
				set_field_value(&cmnda_pll0_fb_div_high, 0x063);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x006);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x005);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			case CLK_RATIO_2_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						660);
				set_field_value(&cmnda_pll0_fb_div_low, 0x20c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x084);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x008);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x008);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			default:
				ftemp = pixel_freq_khz;
				DRM_WARN
				    ("This pixel clock frequency (%u kHz) is not supported with this (%0d-bit) color depth.\n",
				     ftemp, bpp);
			}
		} else if (inside(pixel_freq_khz, 594000, 594000)) {
			switch (clk_ratio) {
			case CLK_RATIO_1_1:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						660);
				set_field_value(&cmnda_pll0_fb_div_low, 0x20c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x084);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x003);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x003);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			case CLK_RATIO_1_2:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						660);
				set_field_value(&cmnda_pll0_fb_div_low, 0x20c);
				set_field_value(&cmnda_pll0_fb_div_high, 0x084);
				set_field_value(&cmnda_pll0_pxdiv_low, 0x003);
				set_field_value(&cmnda_pll0_pxdiv_high, 0x003);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x02);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x01);
				break;
			case CLK_RATIO_5_8:
				set_field_value(&cmnda_pll0_ip_div, 0x04);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						550);
				set_field_value(&cmnda_pll0_fb_div_low, 0x1b4);
				set_field_value(&cmnda_pll0_fb_div_high, 0x06e);
				/* does not matter - pixel clock delivered to controller from SoC */
				set_field_value(&cmnda_pll0_pxdiv_low, 0x003);
				/* does not matter - pixel clock delivered to controller from SoC */
				set_field_value(&cmnda_pll0_pxdiv_high, 0x003);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			case CLK_RATIO_3_4:
				set_field_value(&cmnda_pll0_ip_div, 0x03);
				set_field_value(&cmn_ref_clk_dig_div, 0x01);
				set_field_value(&divider_scaler, 0x01);
				set_field_value(&pll_feedback_divider_total,
						495);
				set_field_value(&cmnda_pll0_fb_div_low, 0x188);
				set_field_value(&cmnda_pll0_fb_div_high, 0x063);
				/* does not matter - pixel clock delivered to controller from SoC */
				set_field_value(&cmnda_pll0_pxdiv_low, 0x003);
				/* does not matter - pixel clock delivered to controller from SoC */
				set_field_value(&cmnda_pll0_pxdiv_high, 0x003);
				set_field_value(&vco_ring_select, 0x01);
				set_field_value(&cmnda_hs_clk_0_sel, 0x01);
				set_field_value(&cmnda_hs_clk_1_sel, 0x01);
				set_field_value(&tx_subrate, 0x01);
				set_field_value(&cmnda_pll0_hs_sym_div_sel,
						0x00);
				break;
			default:
				DRM_WARN
				    ("This pixel clock frequency (%d KHz) is not supported with this (%0d-bit) color depth.\n",
				     pixel_freq_khz, bpp);
			}
		} else {
			ftemp = pixel_freq_khz;
			DRM_WARN
			    ("This pixel clock frequency (%u kHz) is not supported.\n",
			     ftemp);
		}

		vco_freq =
		    refclk_freq_khz * pll_feedback_divider_total.value /
		    cmnda_pll0_ip_div.value;
		ftemp = vco_freq;
		DRM_INFO("VCO frequency is %u kHz\n", ftemp);

		if (inside(vco_freq, 1980000, 1980000)) {
			set_field_value(&voltage_to_current_coarse, 0x04);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x09);
			set_field_value(&ptat_ndac_ctrl, 0x09);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 160);
			set_field_value(&v2i_code, 5);
			set_field_value(&vco_cal_code, 183);
		} else if (inside(vco_freq, 2160000, 2160000)) {
			set_field_value(&voltage_to_current_coarse, 0x04);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x09);
			set_field_value(&ptat_ndac_ctrl, 0x09);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 166);
			set_field_value(&v2i_code, 6);
			set_field_value(&vco_cal_code, 208);
		} else if (inside(vco_freq, 2475000, 2475000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 167);
			set_field_value(&v2i_code, 6);
			set_field_value(&vco_cal_code, 209);
		} else if (inside(vco_freq, 2700000, 2700000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			switch (pll_feedback_divider_total.value) {
			case 300:
				set_field_value(&charge_pump_gain, 0x042);
				break;
			case 400:
				set_field_value(&charge_pump_gain, 0x04c);
				break;
			}
			set_field_value(&coarse_code, 188);
			set_field_value(&v2i_code, 6);
			set_field_value(&vco_cal_code, 225);
		} else if (inside(vco_freq, 2970000, 2970000)) {
			set_field_value(&voltage_to_current_coarse, 0x06);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 183);
			set_field_value(&v2i_code, 6);
			set_field_value(&vco_cal_code, 225);
		} else if (inside(vco_freq, 3240000, 3240000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			switch (pll_feedback_divider_total.value) {
			case 360:
				set_field_value(&charge_pump_gain, 0x042);
				break;
			case 480:
				set_field_value(&charge_pump_gain, 0x04c);
				break;
			}
			set_field_value(&coarse_code, 203);
			set_field_value(&v2i_code, 7);
			set_field_value(&vco_cal_code, 256);
		} else if (inside(vco_freq, 3712500, 3712500)) {
			set_field_value(&voltage_to_current_coarse, 0x04);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x07);
			set_field_value(&ptat_ndac_ctrl, 0x0F);
			set_field_value(&charge_pump_gain, 0x04c);
			set_field_value(&coarse_code, 212);
			set_field_value(&v2i_code, 7);
			set_field_value(&vco_cal_code, 257);
		} else if (inside(vco_freq, 3960000, 3960000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x07);
			set_field_value(&ptat_ndac_ctrl, 0x0F);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 184);
			set_field_value(&v2i_code, 6);
			set_field_value(&vco_cal_code, 226);
		} else if (inside(vco_freq, 4320000, 4320000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x07);
			set_field_value(&ptat_ndac_ctrl, 0x0F);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 205);
			set_field_value(&v2i_code, 7);
			set_field_value(&vco_cal_code, 258);
		} else if (inside(vco_freq, 4455000, 4455000)) {
			set_field_value(&voltage_to_current_coarse, 0x05);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x00);
			set_field_value(&pmos_ctrl, 0x07);
			set_field_value(&ptat_ndac_ctrl, 0x0F);
			switch (pll_feedback_divider_total.value) {
			case 495:
				set_field_value(&charge_pump_gain, 0x042);
				break;
			case 660:
				set_field_value(&charge_pump_gain, 0x04c);
				break;
			}
			set_field_value(&coarse_code, 219);
			set_field_value(&v2i_code, 7);
			set_field_value(&vco_cal_code, 272);
		} else if (inside(vco_freq, 4950000, 4950000)) {
			set_field_value(&voltage_to_current_coarse, 0x06);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 213);
			set_field_value(&v2i_code, 7);
			set_field_value(&vco_cal_code, 258);
		} else if (inside(vco_freq, 5940000, 5940000)) {
			set_field_value(&voltage_to_current_coarse, 0x07);
			set_field_value(&voltage_to_current, 0x03);
			set_field_value(&ndac_ctrl, 0x01);
			set_field_value(&pmos_ctrl, 0x00);
			set_field_value(&ptat_ndac_ctrl, 0x07);
			set_field_value(&charge_pump_gain, 0x042);
			set_field_value(&coarse_code, 244);
			set_field_value(&v2i_code, 8);
			set_field_value(&vco_cal_code, 292);
		} else {
			ftemp = vco_freq;
			DRM_WARN("Current vco_freq (%u kHz) is not supported.\n",
			       ftemp);
		}

		/* register CMN_PLL0_VCOCAL_INIT_TMR */
		Afe_write(state, 0x0084, 0x0064);
		/* register CMN_PLL0_VCOCAL_ITER_TMR */
		Afe_write(state, 0x0085, 0x000A);
		/* register PHY_HDP_CLK_CTL */
		reg_val = Afe_read(state, 0xC009);
		reg_val &= 0x00FF;
		reg_val |= 0x2 << 8;
		reg_val |= 0x1 << 12;
		Afe_write(state, 0xC009, reg_val);
		/* register CMN_DIAG_PLL0_INCLK_CTRL */
		reg_val = set_reg_value(cmnda_pll0_ip_div);
		reg_val |= set_reg_value(cmnda_pll0_hs_sym_div_sel);
		Afe_write(state, 0x01CA, reg_val);
		/* register CMN_DIAG_PLL0_FBH_OVRD */
		reg_val = set_reg_value(cmnda_pll0_fb_div_high);
		reg_val |= (1 << 15);
		Afe_write(state, 0x01C0, reg_val);
		/* register CMN_DIAG_PLL0_FBL_OVRD */
		reg_val = set_reg_value(cmnda_pll0_fb_div_low);
		reg_val |= (1 << 15);
		Afe_write(state, 0x01C1, reg_val);
		/* register CMN_DIAG_PLL0_PXL_DIVL */
		reg_val = set_reg_value(cmnda_pll0_pxdiv_low);
		Afe_write(state, 0x01CC, reg_val);
		/* register CMN_DIAG_PLL0_PXL_DIVH */
		reg_val = set_reg_value(cmnda_pll0_pxdiv_high);
		reg_val |= (1 << 15);
		Afe_write(state, 0x01CB, reg_val);

		/* register TX_DIAG_TX_CTRL */
		for (k = 0; k < num_lanes; k++) {
			reg_val = Afe_read(state, 0x41E0 | (k << 9));
			reg_val &= 0xFF3F;
			reg_val |= (tx_subrate.value >> 1) << 6;
			Afe_write(state, 0x41E0 | (k << 9), reg_val);
		}

		/* register PHY_PMA_CMN_CTRL1 */
		reg_val = Afe_read(state, 0xC800);
		reg_val &= 0xCFFF;
		reg_val |= set_reg_value(cmn_ref_clk_dig_div);
		Afe_write(state, 0xC800, reg_val);
		/* register CMN_CDIAG_REFCLK_CTRL */
		reg_val = Afe_read(state, 0x0062);
		reg_val &= 0x8FFF;
		reg_val |= set_reg_value(divider_scaler);
		reg_val |= 0x00C0;
		Afe_write(state, 0x0062, reg_val);
		/* register CMN_DIAG_HSCLK_SEL */
		reg_val = Afe_read(state, 0x01E0);
		reg_val &= 0xFF00;
		reg_val |= (cmnda_hs_clk_0_sel.value >> 1) << 0;
		reg_val |= (cmnda_hs_clk_1_sel.value >> 1) << 4;
		Afe_write(state, 0x01E0, reg_val);
		/* register CMN_PLLSM0_USER_DEF_CTRL */
		reg_val = set_reg_value(vco_ring_select);
		Afe_write(state, 0x002F, reg_val);

		/* register XCVR_DIAG_HSCLK_SEL */
		for (k = 0; k < num_lanes; k++) {
			reg_val = Afe_read(state, 0x40E1 | (k << 9));
			reg_val &= 0xCFFF;
			reg_val |= (cmnda_hs_clk_0_sel.value >> 1) << 12;
			Afe_write(state, 0x40E1 | (k << 9), reg_val);
		}

		/* register CMN_DIAG_PLL0_OVRD */
		Afe_write(state, 0x01C2, 0x0000);
		/* register CMN_DIAG_PLL0_V2I_TUNE */
		reg_val = set_reg_value(voltage_to_current_coarse);
		reg_val |= set_reg_value(voltage_to_current);
		Afe_write(state, 0x01C5, reg_val);
		/* register CMN_DIAG_PLL0_PTATIS_TUNE1 */
		reg_val = set_reg_value(pmos_ctrl);
		reg_val |= set_reg_value(ndac_ctrl);
		Afe_write(state, 0x01C8, reg_val);
		/* register CMN_DIAG_PLL0_PTATIS_TUNE2 */
		reg_val = set_reg_value(ptat_ndac_ctrl);
		Afe_write(state, 0x01C9, reg_val);
		/* register CMN_PLL0_VCOCAL_START */
		reg_val = Afe_read(state, 0x0081);
		reg_val &= 0xFE00;
		reg_val |= set_reg_value(vco_cal_code);
		Afe_write(state, 0x0081, reg_val);
		/* register CMN_DIAG_PLL0_CP_TUNE */
		reg_val = set_reg_value(charge_pump_gain);
		Afe_write(state, 0x01C6, reg_val);
		/* register CMN_DIAG_PLL0_LF_PROG */
		Afe_write(state, 0x01C7, 0x0008);

		/* register XCVR_DIAG_PLLDRC_CTRL */
		for (k = 0; k < num_lanes; k++) {
			reg_val = Afe_read(state, 0x40E0 | (k << 9));
			reg_val &= 0xBFFF;
			Afe_write(state, 0x40E0 | (k << 9), reg_val);
		}
	}

	/* Back to task phy_cfg_hdp */

	/* register PHY_PMA_CMN_CTRL1 */
	reg_val = Afe_read(state, 0xC800);
	reg_val &= 0xFF8F;
	/* for differential clock on the refclk_p and refclk_m
	 * off chip pins: PHY_PMA_CMN_CTRL1[6:4]=3'b000 */
	reg_val |= 0x0000;
	Afe_write(state, 0xC800, reg_val);

	/* register CMN_DIAG_ACYA */
	Afe_write(state, 0x01FF, 0x0100);

	if (phy_reset_workaround) {
		/* register PHY_ISO_CMN_CTRL */
		Afe_write(state, 0xC010, 0x0001);	// Deassert PHY reset
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0003);
		for (k = 0; k < num_lanes; k++) {
			/* register XCVR_PSM_RCTRL */
			Afe_write(state, 0x4001 | (k << 9), 0xFEFC);
		}
		/* register PHY_PMA_ISO_CMN_CTRL
		 * Assert cmn_macro_pwr_en*/
		Afe_write(state, 0xC810, 0x0013);

		/* PHY_PMA_ISO_CMN_CTRL
		 * wait for cmn_macro_pwr_en_ack*/
		while (!(Afe_read(state, 0xC810) & (1 << 5)))
			;

		/* PHY_PMA_CMN_CTRL1 wait for cmn_ready */
		while (!(Afe_read(state, 0xC800) & (1 << 0)))
			;
	} else {
		for (k = 0; k < num_lanes; k++) {
			/* register XCVR_PSM_RCTRL */
			Afe_write(state, 0x4001 | (k << 9), 0xBEFC);
		}
	}
	for (k = 0; k < num_lanes; k++) {
		/* register TX_PSC_A0 */
		Afe_write(state, 0x4100 | (k << 9), 0x6791);
		/* register TX_PSC_A1 */
		Afe_write(state, 0x4101 | (k << 9), 0x6790);
		/* register TX_PSC_A2 */
		Afe_write(state, 0x4102 | (k << 9), 0x0090);
		/* register TX_PSC_A3 */
		Afe_write(state, 0x4103 | (k << 9), 0x0090);
		/* register RX_PSC_CAL */
		reg_val = Afe_read(state, 0x8006 | (k << 9));
		reg_val &= 0xFFBB;
		Afe_write(state, 0x8006 | (k << 9), reg_val);
		reg_val = Afe_read(state, 0x8000 | (k << 9));
		reg_val &= 0xFFBB;
		Afe_write(state, 0x8000 | (k << 9), reg_val);
	}

	/* End of task phy_cfg_hdp */
	/* register PHY_HDP_MODE_CTL */
	Afe_write(state, 0xC008, 0x0004);

	return character_freq_khz;

}

#define __ARC_CONFIG__

int hdmi_tx_t28hpc_power_config_seq(state_struct *state, int num_lanes)
{
	unsigned char k;

	/* Configure the power state.
	 * register TX_DIAG_ACYA */
	for (k = 0; k < num_lanes; k++) {
		/* register XCVR_PSM_CAL_TMR */
		Afe_write(state, 0x41FF | (k << 9), 0x0001);
	}

	/* register PHY_DP_MODE_CTL */
	while (!(Afe_read(state, 0xC008) & (1 << 6)))
		;

#ifdef __ARC_CONFIG__
	imx_arc_power_up(state);
	imx_arc_calibrate(state);
	imx_arc_config(state);
#endif

	/* PHY_DP_MODE_CTL */
	Afe_write(state, 0xC008, (((0x0F << num_lanes) & 0x0F) << 12) | 0x0101);

	/* PHY_DP_MODE_CTL */
	while (!(Afe_read(state, 0xC008) & (1 << 4)))
		;

	return 0;
}
