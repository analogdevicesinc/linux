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
#include "imx-hdp.h"
#include "API_AFE_t28hpc_hdmitx.h"
#include "t28hpc_hdmitx_table.h"

/* check pixel clock rate in
 * Table 8. HDMI TX pixel clock */
int pixel_clock_range_t28hpc(struct drm_display_mode *mode)
{
	int i, row, rate;

	row = T28HPC_HDMITX_CLOCK_CONTROL_TABLE_ROWS_PIXEL_OUT;
	for (i = 0; i < row; i++) {
		   rate = t28hpc_hdmitx_clock_control_table_pixel_out[i][T8_PIXEL_CLK_FREQ_KHZ];
		   if (rate == mode->clock)
			   return 1;
	}
	return 0;
}

int phy_cfg_hdp_t28hpc(state_struct *state,
				int num_lanes,
				struct drm_display_mode *mode,
				int bpp,
				VIC_PXL_ENCODING_FORMAT format,
				bool pixel_clk_from_phy)
{
	const int phy_reset_workaround = 1;
	u32 vco_freq_khz;
	unsigned char k;
	unsigned char row;
	u32 feedback_factor;
	u32 reg_val;
	u32 pll_feedback_divider_total;
	int pixel_freq_khz = mode->clock;
	u32 character_clock_ratio_num = 1;
	u32 character_clock_ratio_den = 1;
	u32 character_freq_khz;
	const u32 refclk_freq_khz = 27000;
	clk_ratio_t clk_ratio = CLK_RATIO_1_1;

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

	reg_field_t cmnda_pll0_pxdiv_high;
	reg_field_t cmnda_pll0_pxdiv_low;
	reg_field_t coarse_code;
	reg_field_t v2i_code;
	reg_field_t vco_cal_code;

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
		switch (bpp) {	/* Assume RGB */
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

	/* Determine a relevant feedback factor as used in the
	 * t28hpc_hdmitx_clock_control_table table */
	switch (clk_ratio) {
	case CLK_RATIO_1_1:
		feedback_factor = 1000;
		break;
	case CLK_RATIO_5_4:
		feedback_factor = 1250;
		break;
	case CLK_RATIO_3_2:
		feedback_factor = 1500;
		break;
	case CLK_RATIO_2_1:
		feedback_factor = 2000;
		break;
	case CLK_RATIO_1_2:
		feedback_factor = 500;
		break;
	case CLK_RATIO_5_8:
		feedback_factor = 625;
		break;
	case CLK_RATIO_3_4:
		feedback_factor = 750;
		break;
	}

	character_freq_khz =
	    pixel_freq_khz * character_clock_ratio_num /
	    character_clock_ratio_den;
	DRM_INFO
	    ("Pixel clock frequency: %d KHz, character clock frequency: %d, color depth is %0d-bit.\n",
	     pixel_freq_khz, character_freq_khz, bpp);

	if (pixel_clk_from_phy == 0) {

		/* Get right row from the t28hpc_hdmitx_clock_control_table_pixel_in table.
		 * Check if 'pixel_freq_mhz' falls inside
		 * the <PIXEL_CLK_FREQ_MHZ_MIN, PIXEL_CLK_FREQ_MHZ_MAX> range.
		 * Consider only the rows with FEEDBACK_FACTOR column matching feedback_factor. */
		row =
		    get_table_row((const u32 *)&t28hpc_hdmitx_clock_control_table_pixel_in,
				  T28HPC_HDMITX_CLOCK_CONTROL_TABLE_ROWS_PIXEL_IN,
				  T28HPC_HDMITX_CLOCK_CONTROL_TABLE_COLS_PIXEL_IN,
				  pixel_freq_khz, T6_PIXEL_CLK_FREQ_KHZ_MIN,
				  T6_PIXEL_CLK_FREQ_KHZ_MAX, T6_FEEDBACK_FACTOR,
				  feedback_factor);

		/* Check if row was found */
		if (row + 1) {
			DRM_INFO
			    ("Pixel clock frequency (%d KHz) is supported in this color depth (%0d-bit). Settings found in row %0d\n",
			     pixel_freq_khz, bpp, row);
		} else {
			DRM_INFO
			    ("Pixel clock frequency (%d KHz) not supported for this color depth (%0d-bit)\n",
			     pixel_freq_khz, bpp);
			return 0;
		}

		/* Extract particular values from the
		 * t28hpc_hdmitx_clock_control_table_pixel_in table */
		set_field_value(&cmnda_pll0_ip_div,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_CMNDA_PLL0_IP_DIV]);
		set_field_value(&cmn_ref_clk_dig_div,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_CMN_REF_CLK_DIG_DIV]);
		set_field_value(&divider_scaler,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_REF_CLK_DIVIDER_SCALER]);
		set_field_value(&cmnda_pll0_fb_div_low,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_CMNDA_PLL0_FB_DIV_LOW]);
		set_field_value(&cmnda_pll0_fb_div_high,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_CMNDA_PLL0_FB_DIV_HIGH]);
		set_field_value(&vco_ring_select,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_VCO_RING_SELECT]);
		set_field_value(&cmnda_hs_clk_0_sel,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_CMNDA_HS_CLK_0_SEL]);
		set_field_value(&cmnda_hs_clk_1_sel,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_CMNDA_HS_CLK_1_SEL]);
		set_field_value(&tx_subrate,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_HSCLK_DIV_TX_SUB_RATE]);
		set_field_value(&cmnda_pll0_hs_sym_div_sel,
				t28hpc_hdmitx_clock_control_table_pixel_in[row]
				[T6_CMNDA_PLL0_HS_SYM_DIV_SEL]);

		/* Display parameters (informative message) */
		DRM_DEBUG("set_field_value() cmnda_pll0_ip_div        : 0x%02X\n",
		       cmnda_pll0_ip_div.value);
		DRM_DEBUG("set_field_value() cmn_ref_clk_dig_div      : 0x%X\n",
		       cmn_ref_clk_dig_div.value);
		DRM_DEBUG("set_field_value() divider_scaler           : 0x%X\n",
		       divider_scaler.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_fb_div_low    : 0x%03X\n",
		       cmnda_pll0_fb_div_low.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_fb_div_high   : 0x%03X\n",
		       cmnda_pll0_fb_div_high.value);
		DRM_DEBUG("set_field_value() vco_ring_select          : %0d\n",
		       vco_ring_select.value);
		DRM_DEBUG("set_field_value() cmnda_hs_clk_0_sel       : %0d\n",
		       cmnda_hs_clk_0_sel.value);
		DRM_DEBUG("set_field_value() cmnda_hs_clk_1_sel       : %0d\n",
		       cmnda_hs_clk_1_sel.value);
		DRM_DEBUG("set_field_value() tx_subrate               : %0d\n",
		       tx_subrate.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_hs_sym_div_sel: 0x%X\n",
		       cmnda_pll0_hs_sym_div_sel.value);

		pll_feedback_divider_total =
		    cmnda_pll0_fb_div_low.value + cmnda_pll0_fb_div_high.value + 4;
		vco_freq_khz =
		    pixel_freq_khz * pll_feedback_divider_total /
		    cmnda_pll0_ip_div.value;
		DRM_INFO("VCO frequency is %d\n", vco_freq_khz);

		/* Get right row from the t28hpc_hdmitx_pll_tuning_table_pixel_in table.
		 * Check if 'vco_freq_khz' falls inside the
		 * <PLL_VCO_FREQ_KHZ_MIN, PLL_VCO_FREQ_KHZ_MAX> range.
		 * Consider only the rows with PLL_FEEDBACK_DIV_TOTAL
		 * column matching pll_feedback_divider_total. */
		row =
		    get_table_row((const u32 *)&t28hpc_hdmitx_pll_tuning_table_pixel_in,
				  T28HPC_HDMITX_PLL_TUNING_TABLE_ROWS_PIXEL_IN,
				  T28HPC_HDMITX_PLL_TUNING_TABLE_COLS_PIXEL_IN,
				  vco_freq_khz, T7_PLL_VCO_FREQ_KHZ_MIN,
				  T7_PLL_VCO_FREQ_KHZ_MAX,
				  T7_PLL_FEEDBACK_DIV_TOTAL,
				  pll_feedback_divider_total);

		if (row + 1) {
			DRM_INFO
			    ("VCO frequency (%d KHz) is supported. Settings found in row %0d\n",
			     vco_freq_khz, row);
		} else {
			DRM_INFO("VCO frequency (%d KHz) not supported\n",
			       vco_freq_khz);
			return 0;
		}

		/* Extract particular values from
		 * the t28hpc_hdmitx_pll_tuning_table_pixel_in table */
		set_field_value(&voltage_to_current_coarse,
				t28hpc_hdmitx_pll_tuning_table_pixel_in[row]
				[T7_VOLTAGE_TO_CURRENT_COARSE]);
		set_field_value(&voltage_to_current,
				t28hpc_hdmitx_pll_tuning_table_pixel_in[row]
				[T7_VOLTAGE_TO_CURRENT]);
		set_field_value(&ndac_ctrl,
				t28hpc_hdmitx_pll_tuning_table_pixel_in[row]
				[T7_NDAC_CTRL]);
		set_field_value(&pmos_ctrl,
				t28hpc_hdmitx_pll_tuning_table_pixel_in[row]
				[T7_PMOS_CTRL]);
		set_field_value(&ptat_ndac_ctrl,
				t28hpc_hdmitx_pll_tuning_table_pixel_in[row]
				[T7_PTAT_NDAC_CTRL]);
		set_field_value(&charge_pump_gain,
				t28hpc_hdmitx_pll_tuning_table_pixel_in[row]
				[T7_CHARGE_PUMP_GAIN]);

		/* Display parameters (informative message) */
		DRM_DEBUG("set_field_value() voltage_to_current_coarse : 0x%X\n",
		       voltage_to_current_coarse.value);
		DRM_DEBUG("set_field_value() voltage_to_current        : 0x%X\n",
		       voltage_to_current.value);
		DRM_DEBUG("set_field_value() ndac_ctrl                 : 0x%X\n",
		       ndac_ctrl.value);
		DRM_DEBUG("set_field_value() pmos_ctrl                 : 0x%02X\n",
		       pmos_ctrl.value);
		DRM_DEBUG("set_field_value() ptat_ndac_ctrl            : 0x%02X\n",
		       ptat_ndac_ctrl.value);
		DRM_DEBUG("set_field_value() charge_pump_gain          : 0x%03X\n",
		       charge_pump_gain.value);

	} else {
		/* pixel_clk_from_phy == 1 */

		/* Get right row from the t28hpc_hdmitx_clock_control_table_pixel_out table.
		 * Check if 'pixel_freq_khz' value matches the PIXEL_CLK_FREQ_MHZ column.
		 * Consider only the rows with FEEDBACK_FACTOR column matching feedback_factor. */
		row =
		    get_table_row((const u32 *)&t28hpc_hdmitx_clock_control_table_pixel_out,
				  T28HPC_HDMITX_CLOCK_CONTROL_TABLE_ROWS_PIXEL_OUT,
				  T28HPC_HDMITX_CLOCK_CONTROL_TABLE_COLS_PIXEL_OUT,
				  pixel_freq_khz, T8_PIXEL_CLK_FREQ_KHZ,
				  T8_PIXEL_CLK_FREQ_KHZ, T8_FEEDBACK_FACTOR,
				  feedback_factor);

		/* Check if row was found */
		if (row + 1) {
			DRM_INFO
			    ("Pixel clock frequency (%d KHz) is supported in this color depth (%0d-bit). Settings found in row %0d\n",
			     pixel_freq_khz, bpp, row);
		} else {
			DRM_INFO
			    ("Pixel clock frequency (%d KHz) not supported for this color depth (%0d-bit)\n",
			     pixel_freq_khz, bpp);
			return 0;
		}

		/* Extract particular values from
		 * the t28hpc_hdmitx_clock_control_table_pixel_out table */
		set_field_value(&cmnda_pll0_ip_div,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_PLL0_IP_DIV]);
		set_field_value(&cmn_ref_clk_dig_div,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMN_REF_CLK_DIG_DIV]);
		set_field_value(&divider_scaler,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_REF_CLK_DIVIDER_SCALER]);
		set_field_value(&cmnda_pll0_fb_div_low,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_PLL0_FB_DIV_LOW]);
		set_field_value(&cmnda_pll0_fb_div_high,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_PLL0_FB_DIV_HIGH]);
		set_field_value(&cmnda_pll0_pxdiv_low,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_PLL0_PXDIV_LOW]);
		set_field_value(&cmnda_pll0_pxdiv_high,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_PLL0_PXDIV_HIGH]);
		set_field_value(&vco_ring_select,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_VCO_RING_SELECT]);
		set_field_value(&cmnda_hs_clk_0_sel,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_HS_CLK_0_SEL]);
		set_field_value(&cmnda_hs_clk_1_sel,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_HS_CLK_1_SEL]);
		set_field_value(&tx_subrate,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_HSCLK_DIV_TX_SUB_RATE]);
		set_field_value(&cmnda_pll0_hs_sym_div_sel,
				t28hpc_hdmitx_clock_control_table_pixel_out[row]
				[T8_CMNDA_PLL0_HS_SYM_DIV_SEL]);

		/* Display parameters (informative message) */
		DRM_DEBUG("set_field_value() cmnda_pll0_ip_div        : 0x%02X\n",
		       cmnda_pll0_ip_div.value);
		DRM_DEBUG("set_field_value() cmn_ref_clk_dig_div      : 0x%X\n",
		       cmn_ref_clk_dig_div.value);
		DRM_DEBUG("set_field_value() divider_scaler           : 0x%X\n",
		       divider_scaler.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_fb_div_low    : 0x%03X\n",
		       cmnda_pll0_fb_div_low.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_fb_div_high   : 0x%03X\n",
		       cmnda_pll0_fb_div_high.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_pxdiv_low     : 0x%03X\n",
		       cmnda_pll0_pxdiv_low.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_pxdiv_high    : 0x%03X\n",
		       cmnda_pll0_pxdiv_high.value);
		DRM_DEBUG("set_field_value() vco_ring_select          : %0d\n",
		       vco_ring_select.value);
		DRM_DEBUG("set_field_value() cmnda_hs_clk_0_sel       : %0d\n",
		       cmnda_hs_clk_0_sel.value);
		DRM_DEBUG("set_field_value() cmnda_hs_clk_1_sel       : %0d\n",
		       cmnda_hs_clk_1_sel.value);
		DRM_DEBUG("set_field_value() tx_subrate               : %0d\n",
		       tx_subrate.value);
		DRM_DEBUG("set_field_value() cmnda_pll0_hs_sym_div_sel: 0x%X\n",
		       cmnda_pll0_hs_sym_div_sel.value);

		pll_feedback_divider_total =
		    cmnda_pll0_fb_div_low.value + cmnda_pll0_fb_div_high.value + 4;
		vco_freq_khz =
		    refclk_freq_khz * pll_feedback_divider_total / cmnda_pll0_ip_div.value;

		DRM_INFO("VCO frequency is %d\n", vco_freq_khz);

		/* Get right row from the t28hpc_hdmitx_pll_tuning_table table_pixel_out.
		 * Check if 'vco_freq_khz' falls inside
		 * the <PLL_VCO_FREQ_KH_MIN, PLL_VCO_FREQ_KHZ_MAX> range.
		 * Consider only the rows with PLL_FEEDBACK_DIV_TOTAL
		 * column matching pll_feedback_divider_total. */
		row =
		    get_table_row((const u32 *)&t28hpc_hdmitx_pll_tuning_table_pixel_out,
				  T28HPC_HDMITX_PLL_TUNING_TABLE_ROWS_PIXEL_OUT,
				  T28HPC_HDMITX_PLL_TUNING_TABLE_COLS_PIXEL_OUT,
				  vco_freq_khz, T9_PLL_VCO_FREQ_KHZ_MIN,
				  T9_PLL_VCO_FREQ_KHZ_MAX,
				  T9_PLL_FEEDBACK_DIV_TOTAL,
				  pll_feedback_divider_total);

		if (row + 1) {
			DRM_INFO
			    ("VCO frequency (%d KHz) is supported. Settings found in row %0d\n",
			     vco_freq_khz, row);
		} else {
			DRM_INFO("VCO frequency (%d KHz) not supported\n",
			       vco_freq_khz);
			return 0;
		}

		/* Extract particular values from
		 * the t28hpc_hdmitx_pll_tuning_table_pixel_out table. */
		set_field_value(&voltage_to_current_coarse,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_VOLTAGE_TO_CURRENT_COARSE]);
		set_field_value(&voltage_to_current,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_VOLTAGE_TO_CURRENT]);
		set_field_value(&ndac_ctrl,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_NDAC_CTRL]);
		set_field_value(&pmos_ctrl,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_PMOS_CTRL]);
		set_field_value(&ptat_ndac_ctrl,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_PTAT_NDAC_CTRL]);
		set_field_value(&charge_pump_gain,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_CHARGE_PUMP_GAIN]);
		set_field_value(&coarse_code,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_COARSE_CODE]);
		set_field_value(&v2i_code,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_V2I_CODE]);
		set_field_value(&vco_cal_code,
				t28hpc_hdmitx_pll_tuning_table_pixel_out[row]
				[T9_VCO_CAL_CODE]);

		/* Display parameters (informative message) */
		DRM_DEBUG("set_field_value() voltage_to_current_coarse : 0x%X\n",
		       voltage_to_current_coarse.value);
		DRM_DEBUG("set_field_value() voltage_to_current        : 0x%X\n",
		       voltage_to_current.value);
		DRM_DEBUG("set_field_value() ndac_ctrl                 : 0x%X\n",
		       ndac_ctrl.value);
		DRM_DEBUG("set_field_value() pmos_ctrl                 : 0x%02X\n",
		       pmos_ctrl.value);
		DRM_DEBUG("set_field_value() ptat_ndac_ctrl            : 0x%02X\n",
		       ptat_ndac_ctrl.value);
		DRM_DEBUG("set_field_value() charge_pump_gain          : 0x%03X\n",
		       charge_pump_gain.value);
		DRM_DEBUG("set_field_value() coarse_code               : %0d\n",
		       coarse_code.value);
		DRM_DEBUG("set_field_value() v2i_code                  : %0d\n",
		       v2i_code.value);
		DRM_DEBUG("set_field_value() vco_cal_code              : %0d\n",
		       vco_cal_code.value);
	}

	if (phy_reset_workaround) {
		/* register PHY_PMA_ISOLATION_CTRL */
		/* enable PHY isolation mode only for CMN */
		Afe_write(state, 0xC81F, 0xD000);
		/* register PHY_PMA_ISO_PLL_CTRL1 */
		reg_val = Afe_read(state, 0xC812);
		reg_val &= 0xFF00;
		reg_val |= 0x0012;
		/* set cmn_pll0_clk_datart1_div/cmn_pll0_clk_datart0_div dividers */
		Afe_write(state, 0xC812, reg_val);
		/* register PHY_ISO_CMN_CTRL */
		/* assert PHY reset from isolation register */
		Afe_write(state, 0xC010, 0x0000);
		/* register PHY_PMA_ISO_CMN_CTRL */
		/* assert PMA CMN reset */
		Afe_write(state, 0xC810, 0x0000);
		/* register XCVR_DIAG_BIDI_CTRL */
		for (k = 0; k < num_lanes; k++) {
			Afe_write(state, 0x40E8 | (k << 9), 0x00FF);
		}
	}
	/* Describing Task phy_cfg_hdp */

	/* register PHY_PMA_CMN_CTRL1 */
	reg_val = Afe_read(state, 0xC800);
	reg_val &= 0xFFF7;
	reg_val |= 0x0008;
	Afe_write(state, 0xC800, reg_val);

	/* register CMN_DIAG_PLL0_TEST_MODE */
	Afe_write(state, 0x01C4, 0x0020);
	/* register CMN_PSM_CLK_CTRL */
	Afe_write(state, 0x0061, 0x0016);

	/* Describing Task phy_cfg_hdmi_pll0_0pt5736 */

	/* register CMN_PLL0_VCOCAL_INIT_TMR */
	Afe_write(state, 0x0084, 0x0064);
	/* register CMN_PLL0_VCOCAL_ITER_TMR */
	Afe_write(state, 0x0085, 0x000A);
	/*register PHY_HDP_CLK_CTL */
	reg_val = Afe_read(state, 0xC009);
	reg_val &= 0x00FF;
	reg_val |= 0x1200;
	Afe_write(state, 0xC009, reg_val);
	/* register CMN_DIAG_PLL0_INCLK_CTRL */
	reg_val = set_reg_value(cmnda_pll0_hs_sym_div_sel);
	reg_val |= set_reg_value(cmnda_pll0_ip_div);
	Afe_write(state, 0x01CA, reg_val);
	/* register CMN_DIAG_PLL0_FBH_OVRD */
	reg_val = set_reg_value(cmnda_pll0_fb_div_high);
	reg_val |= (1 << 15);
	Afe_write(state, 0x01C0, reg_val);
	/* register CMN_DIAG_PLL0_FBL_OVRD */
	reg_val = set_reg_value(cmnda_pll0_fb_div_low);
	reg_val |= (1 << 15);
	Afe_write(state, 0x01C1, reg_val);
	/*register PHY_PMA_CMN_CTRL1 */
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
	/*register CMN_DIAG_HSCLK_SEL */
	reg_val = Afe_read(state, 0x01E0);
	reg_val &= 0xFF00;
	reg_val |= (cmnda_hs_clk_0_sel.value >> 1) << 0;
	reg_val |= (cmnda_hs_clk_1_sel.value >> 1) << 4;
	Afe_write(state, 0x01E0, reg_val);
	/*register XCVR_DIAG_HSCLK_SEL */
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
	if (pixel_clk_from_phy == 1) {
		/* register CMN_DIAG_PLL0_PXL_DIVL */
		reg_val = set_reg_value(cmnda_pll0_pxdiv_low);
		Afe_write(state, 0x01CC, reg_val);
		/* register CMN_DIAG_PLL0_PXL_DIVH */
		reg_val = set_reg_value(cmnda_pll0_pxdiv_high);
		reg_val |= (1 << 15);
		Afe_write(state, 0x01CB, reg_val);

		/* register CMN_PLL0_VCOCAL_START */
		reg_val = Afe_read(state, 0x0081);
		reg_val &= 0xFE00;
		reg_val |= set_reg_value(vco_cal_code);
		Afe_write(state, 0x0081, reg_val);
	}

	/* Back to task phy_cfg_hdp */

	/* register PHY_PMA_CMN_CTRL1 */
	reg_val = Afe_read(state, 0xC800);
	reg_val &= 0xFF8F;
	/* for single ended reference clock
	 * on the cmn_ref_clk_int pin: PHY_PMA_CMN_CTRL1[6:4]=3'b011 */
	/* for differential clock on the refclk_p and
	 * refclk_m off chip pins: PHY_PMA_CMN_CTRL1[6:4]=3'b000 */
	reg_val |= 0x0000;
	Afe_write(state, 0xC800, reg_val);

	/*register CMN_DIAG_ACYA */
	/* for differential clock on the refclk_p and
	 * refclk_m off chip pins: CMN_DIAG_ACYA[8]=1'b1 */
	Afe_write(state, 0x01FF, 0x0100);

	if (phy_reset_workaround) {
		/* register PHY_ISO_CMN_CTRL */
		/* Deassert PHY reset */
		Afe_write(state, 0xC010, 0x0001);
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0003);
		for (k = 0; k < num_lanes; k++) {
			/* register XCVR_PSM_RCTRL */
			Afe_write(state, 0x4001 | (k << 9), 0xFEFC);
		}
		/* register PHY_PMA_ISO_CMN_CTRL */
		/* Assert cmn_macro_pwr_en */
		Afe_write(state, 0xC810, 0x0013);

		/* PHY_PMA_ISO_CMN_CTRL */
		/* wait for cmn_macro_pwr_en_ack */
		while (!(Afe_read(state, 0xC810) & (1 << 5)));

		/* PHY_PMA_CMN_CTRL1 */
		/* wait for cmn_ready */
		while (!(Afe_read(state, 0xC800) & (1 << 0)));
	} else {
		for (k = 0; k < num_lanes; k++)
			/* register XCVR_PSM_RCTRL */
			Afe_write(state, 0x4001 | (k << 9), 0xBEFC);
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
		/* register RX_PSC_A0 */
		reg_val = Afe_read(state, 0x8000 | (k << 9));
		reg_val &= 0xFFBB;
		Afe_write(state, 0x8000 | (k << 9), reg_val);
	}

	/* register PHY_HDP_MODE_CTL */
	Afe_write(state, 0xC008, 0x0004);

	return character_freq_khz;
}

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
	while (!(Afe_read(state, 0xC008) & (1 << 6)));

	imx_arc_power_up(state);
	imx_arc_calibrate(state);
	imx_arc_config(state);

	/* PHY_DP_MODE_CTL */
	Afe_write(state, 0xC008, (((0x0F << num_lanes) & 0x0F) << 12) | 0x0101);

	/* PHY_DP_MODE_CTL */
	while (!(Afe_read(state, 0xC008) & (1 << 4)));

	return 0;
}
