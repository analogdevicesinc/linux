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
 * API_AFE_ss28fdsoi_kiran_hdmitx.c
 *
 ******************************************************************************
 */

#include <drm/drmP.h>
#include <linux/io.h>
#include "API_AFE_ss28fdsoi_kiran_hdmitx.h"
#include "ss28fdsoi_hdmitx_table.h"

static int inside(u32 value, u32 left_sharp_corner, u32 right_sharp_corner)
{
	if (value < left_sharp_corner)
		return 0;
	if (value > right_sharp_corner)
		return 0;
	return 1;
}

int get_table_row_match_column(const u32 *array, u32 table_rows,
			       u32 table_cols, u32 start_row,
			       u32 column_to_search,
			       u32 value_to_search_in_column)
{
	u32 idx_cols, idx_rows;
	u32 value;
	for (idx_rows = start_row; idx_rows < table_rows; idx_rows++) {
		for (idx_cols = 0; idx_cols < table_cols; idx_cols++) {
			if (idx_cols == column_to_search) {
				value =
				    *((array + idx_rows * table_cols) +
				      idx_cols);
				if (value == value_to_search_in_column) {
					return idx_rows;
				}
			}
		}
	}
	return -1;
}

int get_table_row(const u32 *array, u32 table_rows,
		  u32 table_cols, u32 variable_in_range,
		  u32 range_min_column, u32 range_max_column,
		  u32 column_to_search, u32 column_value)
{
	u32 i = 0;
	while (1) {
		i = get_table_row_match_column(array, table_rows, table_cols, i,
					       column_to_search, column_value);
		if (i + 1) {
			if (inside(variable_in_range,
				   *((array + i * table_cols) +
				     range_min_column),
				   *((array + i * table_cols) +
				     range_max_column))) {
				break;
			}
			i++;
		} else {
			break;
		}
	}
	return i;
}

int phy_cfg_hdp_ss28fdsoi(state_struct *state, int num_lanes,
			  VIC_MODES vicMode, int bpp,
			  VIC_PXL_ENCODING_FORMAT format)
{
	const int phy_reset_workaround = 0;
	u32 vco_freq_khz;
	unsigned char i;
	u32 row, feedback_factor;
	uint32_t reg_val;
	int pixel_freq_khz = vic_table[vicMode][PIXEL_FREQ_KHZ];
	uint32_t character_clock_ratio_num = 1;
	uint32_t character_clock_ratio_den = 1;
	int character_freq_khz;
	u32 ftemp;
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
	reg_field_t vco_ring_select;
	reg_field_t pll_feedback_divider_total;
	reg_field_t voltage_to_current_coarse;
	reg_field_t voltage_to_current;
	reg_field_t ndac_ctrl;
	reg_field_t pmos_ctrl;
	reg_field_t ptat_ndac_ctrl;
	reg_field_t charge_pump_gain;

	/* Set field position in a target register */
	cmnda_pll0_fb_div_high.value = 0x00A;
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
	vco_ring_select.msb = 12;
	vco_ring_select.lsb = 12;
	pll_feedback_divider_total.msb = 9;
	pll_feedback_divider_total.lsb = 0;
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

	DRM_INFO
	    ("phy_cfg_hdp() num_lanes: %0d, vicMode: %0d, color depth: %0d-bit, encoding: %0d\n",
	     num_lanes, vicMode, bpp, format);

	/* register PHY_PMA_ISOLATION_CTRL
	 * enable PHY isolation mode only for CMN */
	if (phy_reset_workaround) {
		Afe_write(state, 0xC81F, 0xD000);
		reg_val = Afe_read(state, 0xC812);
		reg_val &= 0xFF00;
		reg_val |= 0x0012;
		/* set cmn_pll0_clk_datart1_div/cmn_pll0_clk_datart0_div dividers */
		Afe_write(state, 0xC812, reg_val);
		/* register PHY_ISO_CMN_CTRL */
		Afe_write(state, 0xC010, 0x0000);	/* assert PHY reset from isolation register */
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0000);	/* assert PMA CMN reset */
		/* register XCVR_DIAG_BIDI_CTRL */
		for (i = 0; i < num_lanes; i++) {
			Afe_write(state, 0x40E8 | (i << 9), 0x00FF);
		}
	} else {
		/*--------------------------------------------------------------
		 * Describing Task phy_cfg_hdp
		 * ------------------------------------------------------------*/
		/* register PHY_PMA_CMN_CTRL1 */
		for (i = 0; i < num_lanes; i++)
			Afe_write(state, 0x40E8 | (i << 9), 0x007F);
	}
	/* register CMN_DIAG_PLL0_TEST_MODE */
	Afe_write(state, 0x01C4, 0x0022);
	/* register CMN_PSM_CLK_CTRL */
	Afe_write(state, 0x0061, 0x0016);

	/* Determine the TMDS/PIXEL clock ratio */
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
			pr_err("Invalid ColorDepth\n");
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

	/* Determine a relevant feedback factor as used
	 * in the ss28fdsoi_hdmitx_clock_control_table table */
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

	/* Get right row from the ss28fdsoi_hdmitx_clock_control_table table.
	 * Check if 'pixel_freq_khz' falls inside the
	 * <PIXEL_CLK_FREQ_KHZ_MIN, PIXEL_CLK_FREQ_KHZ_MAX> range.
	 * Consider only the rows with FEEDBACK_FACTOR column matching feedback_factor. */
	row =
	    get_table_row((const u32 *)&ss28fdsoi_hdmitx_clock_control_table,
			  SS28FDSOI_HDMITX_CLOCK_CONTROL_TABLE_ROWS,
			  SS28FDSOI_HDMITX_CLOCK_CONTROL_TABLE_COLS,
			  pixel_freq_khz, PIXEL_CLK_FREQ_KHZ_MIN,
			  PIXEL_CLK_FREQ_KHZ_MAX, FEEDBACK_FACTOR,
			  feedback_factor);

	/* Check if row was found */
	ftemp = pixel_freq_khz;
	if (row + 1) {
		DRM_INFO
		    ("Pixel clock frequency (%u kHz) is supported in this color depth (%0d-bit). Settings found in row %0d\n",
		     ftemp, bpp, row);
	} else {
		DRM_INFO
		    ("Pixel clock frequency (%u kHz) not supported for this color depth (%0d-bit), row=%d\n",
		     ftemp, bpp, row);
	}
	character_freq_khz =
	    pixel_freq_khz * character_clock_ratio_num /
	    character_clock_ratio_den;
	ftemp = character_freq_khz;
	DRM_INFO("Character clock frequency: %u kHz.\n", ftemp);

	/* Extract particular values from the ss28fdsoi_hdmitx_clock_control_table table */
	set_field_value(&cmnda_pll0_hs_sym_div_sel,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[CMNDA_PLL0_HS_SYM_DIV_SEL]);
	set_field_value(&cmnda_pll0_ip_div,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[CMNDA_PLL0_IP_DIV]);
	set_field_value(&cmnda_pll0_fb_div_low,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[CMNDA_PLL0_FB_DIV_LOW]);
	set_field_value(&cmnda_pll0_fb_div_high,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[CMNDA_PLL0_FB_DIV_HIGH]);
	set_field_value(&cmn_ref_clk_dig_div,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[CMN_REF_CLK_DIG_DIV]);
	set_field_value(&divider_scaler,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[REF_CLK_DIVIDER_SCALER]);
	set_field_value(&cmnda_hs_clk_0_sel,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[CMNDA_HS_CLK_0_SEL]);
	set_field_value(&cmnda_hs_clk_1_sel,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[CMNDA_HS_CLK_1_SEL]);
	set_field_value(&tx_subrate, ss28fdsoi_hdmitx_clock_control_table[row]
			[HSCLK_DIV_TX_SUB_RATE]);
	set_field_value(&vco_ring_select,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[VCO_RING_SELECT]);
	set_field_value(&pll_feedback_divider_total,
			ss28fdsoi_hdmitx_clock_control_table[row]
			[PLL_FB_DIV_TOTAL]);

	/* Display parameters (informative message) */
	DRM_DEBUG("set_field_value() cmnda_pll0_hs_sym_div_sel : 0x%X\n",
		cmnda_pll0_hs_sym_div_sel.value);
	DRM_DEBUG("set_field_value() cmnda_pll0_ip_div         : 0x%02X\n",
		cmnda_pll0_ip_div.value);
	DRM_DEBUG("set_field_value() cmnda_pll0_fb_div_low     : 0x%03X\n",
		cmnda_pll0_fb_div_low.value);
	DRM_DEBUG("set_field_value() cmnda_pll0_fb_div_high    : 0x%03X\n",
		cmnda_pll0_fb_div_high.value);
	DRM_DEBUG("set_field_value() cmn_ref_clk_dig_div       : 0x%X\n",
		cmn_ref_clk_dig_div.value);
	DRM_DEBUG("set_field_value() divider_scaler            : 0x%X\n",
		divider_scaler.value);
	DRM_DEBUG("set_field_value() cmnda_hs_clk_0_sel        : %0d\n",
		cmnda_hs_clk_0_sel.value);
	DRM_DEBUG("set_field_value() cmnda_hs_clk_1_sel        : %0d\n",
		cmnda_hs_clk_1_sel.value);
	DRM_DEBUG("set_field_value() tx_subrate                : %0d\n",
		tx_subrate.value);
	DRM_DEBUG("set_field_value() vco_ring_select           : %0d\n",
		vco_ring_select.value);
	DRM_DEBUG("set_field_value() pll_feedback_divider_total: %0d\n",
		pll_feedback_divider_total.value);

	vco_freq_khz =
	    pixel_freq_khz * pll_feedback_divider_total.value /
	    cmnda_pll0_ip_div.value;

	/* Get right row from the ss28fdsoi_hdmitx_pll_tuning_table table.
	 * Check if 'vco_freq_mhz' falls inside the
	 * <PLL_VCO_FREQ_MHZ_MIN, PLL_VCO_FREQ_MHZ_MAX> range.
	 * Consider only the rows with PLL_FEEDBACK_DIV_TOTAL.
	 * column matching pll_feedback_divider_total. */
	row =
	    get_table_row((const u32 *)&ss28fdsoi_hdmitx_pll_tuning_table,
			  SS28FDSOI_HDMITX_PLL_TUNING_TABLE_ROWS,
			  SS28FDSOI_HDMITX_PLL_TUNING_TABLE_COLS, vco_freq_khz,
			  PLL_VCO_FREQ_KHZ_MIN, PLL_VCO_FREQ_KHZ_MAX,
			  PLL_FEEDBACK_DIV_TOTAL,
			  pll_feedback_divider_total.value);
	ftemp = vco_freq_khz;
	if (row + 1) {
		DRM_INFO
		    ("VCO frequency (%u kHz) is supported. Settings found in row %0d\n",
		     ftemp, row);
	} else {
		DRM_INFO("VCO frequency (%u kHz) not supported\n", ftemp);
	}

	/* Extract particular values from the ss28fdsoi_hdmitx_pll_tuning_table table */
	set_field_value(&voltage_to_current_coarse,
			ss28fdsoi_hdmitx_pll_tuning_table[row]
			[VOLTAGE_TO_CURRENT_COARSE]);
	set_field_value(&voltage_to_current,
			ss28fdsoi_hdmitx_pll_tuning_table[row]
			[VOLTAGE_TO_CURRENT]);
	set_field_value(&ndac_ctrl,
			ss28fdsoi_hdmitx_pll_tuning_table[row][NDAC_CTRL]);
	set_field_value(&pmos_ctrl,
			ss28fdsoi_hdmitx_pll_tuning_table[row][PMOS_CTRL]);
	set_field_value(&ptat_ndac_ctrl,
			ss28fdsoi_hdmitx_pll_tuning_table[row][PTAT_NDAC_CTRL]);
	set_field_value(&charge_pump_gain,
			ss28fdsoi_hdmitx_pll_tuning_table[row]
			[CHARGE_PUMP_GAIN]);

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

	/* ---------------------------------------------------------------
	 * Describing Task phy_cfg_hdmi_pll0_0pt5736
	 *---------------------------------------------------------------*/

	Afe_write(state, 0x0081, 0x30A0);
	/* register CMN_PLL0_VCOCAL_INIT_TMR */
	Afe_write(state, 0x0084, 0x0064);
	/* register CMN_PLL0_VCOCAL_ITER_TMR */
	Afe_write(state, 0x0085, 0x000A);
	/* register PHY_HDP_CLK_CTL */
	reg_val = Afe_read(state, 0xC009);
	reg_val &= 0x00FF;
	reg_val |= 0x1200;
	Afe_write(state, 0xC009, reg_val);
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
	reg_val = set_reg_value(divider_scaler);
	Afe_write(state, 0x0062, reg_val);
	/* register CMN_DIAG_HSCLK_SEL */
	reg_val = Afe_read(state, 0x01E0);
	reg_val &= 0xFF00;
	reg_val |= (cmnda_hs_clk_0_sel.value >> 1) << 0;
	reg_val |= (cmnda_hs_clk_1_sel.value >> 1) << 4;
	Afe_write(state, 0x01E0, reg_val);

	/* register XCVR_DIAG_HSCLK_SEL */
	for (i = 0; i < num_lanes; i++) {
		reg_val = Afe_read(state, 0x40E1 | (i << 9));
		reg_val &= 0xCFFF;
		reg_val |= (cmnda_hs_clk_0_sel.value >> 1) << 12;
		Afe_write(state, 0x40E1 | (i << 9), reg_val);
	}

	/* register TX_DIAG_TX_CTRL */
	for (i = 0; i < num_lanes; i++) {
		reg_val = Afe_read(state, 0x41E0 | (i << 9));
		reg_val &= 0xFF3F;
		reg_val |= (tx_subrate.value >> 1) << 6;
		Afe_write(state, 0x41E0 | (i << 9), reg_val);
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
	for (i = 0; i < num_lanes; i++) {
		reg_val = Afe_read(state, 0x40E0 | (i << 9));
		reg_val &= 0xBFFF;
		Afe_write(state, 0x40E0 | (i << 9), reg_val);
	}

	/* register PHY_PMA_CMN_CTRL1 */
	reg_val = Afe_read(state, 0xC800);
	reg_val &= 0xFF8F;
	reg_val |= 0x0030;
	Afe_write(state, 0xC800, reg_val);

	/*--------------------------------------------------------------------
	*--------------------Back to task phy_cfg_hdp------------------------
	*--------------------------------------------------------------------*/

	if (phy_reset_workaround) {
		/* register PHY_ISO_CMN_CTRL */
		Afe_write(state, 0xC010, 0x0001);	// Deassert PHY reset */
		for (i = 0; i < num_lanes; i++) {
			/* register TX_DIAG_ACYA */
			Afe_write(state, 0x41FF | (i << 9), 0x0001);
		}
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0003);
		for (i = 0; i < num_lanes; i++) {
			/* register XCVR_PSM_RCTRL */
			Afe_write(state, 0x4001 | (i << 9), 0xFEFC);
		}
		/* register PHY_PMA_ISO_CMN_CTRL */
		Afe_write(state, 0xC810, 0x0013);	/* Assert cmn_macro_pwr_en */

		/* PHY_PMA_ISO_CMN_CTRL */
		while (!(Afe_read(state, 0xC810) & (1 << 5))) ;	/* wait for cmn_macro_pwr_en_ack */

		/* PHY_PMA_CMN_CTRL1 */
		while (!(Afe_read(state, 0xC800) & (1 << 0))) ;	/* wait for cmn_ready */
	} else {
		for (i = 0; i < num_lanes; i++) {
			Afe_write(state, 0x41FF | (i << 9), 0x0001);
			/* register XCVR_PSM_RCTRL */
			Afe_write(state, 0x4001 | (i << 9), 0xBEFC);
		}
	}
	for (i = 0; i < num_lanes; i++) {
		/* register TX_PSC_A0 */
		Afe_write(state, 0x4100 | (i << 9), 0x6791);
		/* register TX_PSC_A1 */
		Afe_write(state, 0x4101 | (i << 9), 0x6790);
		/* register TX_PSC_A2 */
		Afe_write(state, 0x4102 | (i << 9), 0x0090);
		/* register TX_PSC_A3 */
		Afe_write(state, 0x4103 | (i << 9), 0x0090);
	}

	/* register PHY_HDP_MODE_CTL */
	Afe_write(state, 0xC008, 0x0004);
	return character_freq_khz;

}

int hdmi_tx_kiran_power_configuration_seq(state_struct *state, int num_lanes)
{
	/* Configure the power state. */

	/* PHY_DP_MODE_CTL */
	while (!(Afe_read(state, 0xC008) & (1 << 6))) ;

#ifdef __ARC_CONFIG__
	arc_power_up(state);
	arc_calibrate(state);
	arc_config(state);
#endif

	/* PHY_DP_MODE_CTL */
	Afe_write(state, 0xC008, (((0x0F << num_lanes) & 0x0F) << 12) | 0x0001);

	/* PHY_DP_MODE_CTL */
	while (!(Afe_read(state, 0xC008) & (1 << 4))) ;
	return 0;
}
