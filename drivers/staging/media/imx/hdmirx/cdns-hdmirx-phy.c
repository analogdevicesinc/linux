/*
 * Copyright 2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/ktime.h>
#include "cdns-hdmirx-phy.h"

#define PMA_REF_CLK_TMR_VALUE_DEF 0x0801
#define PMA_CMN_READY_TIMEOUT_MS 5
#define PMA_RX_CLK_SIGNAL_DETECT_TIMEOUT_MS 200
#define PMA_RX_CLK_FREQ_DETECT_TIMEOUT_MS 5
#define PMA_RX_CLK_FREQ_DETECT_MIN_THRESH 24000
#define PMA_RX_CLK_FREQ_DETECT_MAX_THRESH 340000
#define PMA_POWER_CHNG_TIMEOUT_MS 100
#define TMDS_STABLE_DETECT_COUNT_THRESHOLD 3
#define TMDS_STABLE_DETECT_TIMEOUT_MS 500

static int pma_power_state_chng(struct cdns_hdmirx_device *hdmirx, u8 power_state);

typedef struct {
	u32 value;
	u8 lsb;
	u8 msb;
	u8 *label;
} reg_field_t;

static void set_field_value(reg_field_t *reg_field, u32 value)
{
	u8 length;
	u32 max_value;
	u32 trunc_val;
	length = (reg_field->msb - reg_field->lsb + 1);

    max_value = (1 << length) - 1;
    if (value > max_value) {
		trunc_val = value;
		trunc_val &= (1 << length) - 1;
		pr_err("set_field_value() Error! Specified value (0x%0X)\
				exceeds field capacity - it will by truncated to\
				0x%0X (%0d-bit field - max value: %0d dec)\n",
				value, trunc_val, length, max_value);
	} else
		reg_field->value = value;
}

static int set_reg_value(reg_field_t reg_field)
{
    return reg_field.value << reg_field.lsb;
}

static int cdns_hdmirx_phy_reg_write(struct cdns_hdmirx_device *hdmirx, u32 addr, u32 val)
{
    return cdns_hdmirx_reg_write(hdmirx, ADDR_PHY_AFE + (addr << 2), val);
}

static int cdns_hdmirx_phy_reg_read(struct cdns_hdmirx_device *hdmirx, u32 addr)
{
    return cdns_hdmirx_reg_read(hdmirx, ADDR_PHY_AFE + (addr << 2));
}

static inline void write16(struct cdns_hdmirx_device *hdmirx, u32 addr, u16 val)
{
	cdns_hdmirx_phy_reg_write(hdmirx, addr, val);
}

static inline void multi_write16(struct cdns_hdmirx_device *hdmirx, u32 addr, u16 val)
{
	u16 addr_tmp = addr;

	if ((addr & 0x1E00) == LINK_ID << 9) {
		addr_tmp |= LINK_WRITE;
		cdns_hdmirx_phy_reg_write(hdmirx, addr_tmp, val);
	} else {
		cdns_hdmirx_phy_reg_write(hdmirx, addr_tmp, val);
	}
}

static inline u16 read16(struct cdns_hdmirx_device *hdmirx, u32 addr)
{
	return cdns_hdmirx_phy_reg_read(hdmirx, addr);
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

void arc_config(struct cdns_hdmirx_device *hdmirx)
{
	u16 reg_val;

	write16(hdmirx, TXDA_CYA_AUXDA_CYA_ADDR, 0x0001);
	udelay(1);
	write16(hdmirx, TX_DIG_CTRL_REG_1_ADDR, 0x3);
	udelay(1);
	write16(hdmirx, TX_DIG_CTRL_REG_2_ADDR, 0x0024);
	udelay(1);

	reg_val = read16(hdmirx, TX_ANA_CTRL_REG_1_ADDR);
	reg_val |= 0x2000;
	write16(hdmirx, TX_ANA_CTRL_REG_1_ADDR, reg_val);
	udelay(1);

	write16(hdmirx, TX_ANA_CTRL_REG_2_ADDR, 0x0100);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_2_ADDR, 0x0300);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_3_ADDR, 0x0000);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_1_ADDR, 0x2008);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_1_ADDR, 0x2018);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_1_ADDR, 0x2098);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_2_ADDR, 0x030C);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_5_ADDR, 0x0010);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_4_ADDR, 0x4001);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_1_ADDR, 0x2198);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_2_ADDR, 0x030D);
	udelay(1);
	write16(hdmirx, TX_ANA_CTRL_REG_2_ADDR, 0x030F);
}

void pma_config(struct cdns_hdmirx_device *hdmirx)
{
	int i;

	u16 const RX_CLK_SLICER_CAL_TUNE_VAL = 0x0008;
	u16 const RX_CLK_TERM_CTRL_VAL = 0x0001;
	u16 const RX_CLK_SLICER_CAL_INIT_TMR_VAL = 0x00FF;
	u16 const RX_CLK_SLICER_CAL_ITER_TMR_VAL = 0x00FF;

	dev_dbg(&hdmirx->pdev->dev, "pma_config() Configuring PMA\n");

	write16(hdmirx, CMN_CMSMT_REF_CLK_TMR_VALUE_ADDR, PMA_REF_CLK_TMR_VALUE_DEF);

	dev_dbg(&hdmirx->pdev->dev, "Changing RX_CLK_TERM_CTRL from 0x%.4X to 0x%.4X\n",
		read16(hdmirx, RX_CLK_TERM_CTRL_ADDR),
		RX_CLK_TERM_CTRL_VAL);
	write16(hdmirx, RX_CLK_TERM_CTRL_ADDR, RX_CLK_TERM_CTRL_VAL);

	dev_dbg(&hdmirx->pdev->dev, "Changing RX_CLK_SLICER_CAL_TUNE_ADDR from 0x%.4X to 0x%.4X\n",
		read16(hdmirx, RX_CLK_SLICER_CAL_TUNE_ADDR),
		RX_CLK_SLICER_CAL_TUNE_VAL);
	write16(hdmirx, RX_CLK_SLICER_CAL_TUNE_ADDR, RX_CLK_SLICER_CAL_TUNE_VAL);

	dev_dbg(&hdmirx->pdev->dev, "Changing RX_CLK_SLICER_CAL_INIT_TMR from 0x%.4X to 0x%.4X\n",
		read16(hdmirx, RX_CLK_SLICER_CAL_INIT_TMR_ADDR),
		RX_CLK_SLICER_CAL_INIT_TMR_VAL);
	write16(hdmirx, RX_CLK_SLICER_CAL_INIT_TMR_ADDR,
		RX_CLK_SLICER_CAL_INIT_TMR_VAL);

	dev_dbg(&hdmirx->pdev->dev, "Changing RX_CLK_SLICER_CAL_ITER_TMR from 0x%.4X to 0x%.4X\n",
		read16(hdmirx, RX_CLK_SLICER_CAL_ITER_TMR_ADDR),
		RX_CLK_SLICER_CAL_ITER_TMR_VAL);
	write16(hdmirx, RX_CLK_SLICER_CAL_ITER_TMR_ADDR,
		RX_CLK_SLICER_CAL_ITER_TMR_VAL);

	write16(hdmirx, CMN_RXCAL_INIT_TMR_ADDR, 0x003F);
	write16(hdmirx, CMN_DIAG_PLL0_TEST_MODE_ADDR, 0x0022);
	multi_write16(hdmirx, XCVR_PSM_CAL_TMR_ADDR, 0x0160);

	/* Drives the rx_differential_invert PMA input for the selected lane */
	for (i = 0; i < 3; i++) {
		u16 const reg_val = 0x0c61;
		write16(hdmirx, (PHY_PMA_XCVR_CTRL_ADDR | (i << 6)), reg_val);
	}
}

void pre_data_rate_change(struct cdns_hdmirx_device *hdmirx)
{
	u16 reg_val;
	int i;
	int ret_val;
	const ktime_t timeout = ktime_timeout_ms(1500);

	/* Turn off frequency measurement: */
	write16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x0000);

	/* Request A3 power hdmirx */
	/* Can only really do this if PHY already in A0 hdmirx, all other
	   times are meaningless and can cause issues if PLL not running...
	*/
	reg_val = read16(hdmirx, PHY_MODE_CTL_ADDR);
	if ((reg_val & 0x00F0) == 0x0010) {
		ret_val = pma_power_state_chng(hdmirx, 0x8);
		dev_dbg(&hdmirx->pdev->dev, "Shutting down PLL\n");
		write16(hdmirx, PHY_MODE_CTL_ADDR, reg_val & 0xEF00);
		do {
			if (ktime_after(ktime_get(), timeout)) {
				dev_dbg(&hdmirx->pdev->dev, "Timed out PHY_PMA_CMN_CTRL2_ADDR: 0x%04X\n", reg_val);
				dev_dbg(&hdmirx->pdev->dev, "Setting PHY reset\n");
				imx8qm_hdmi_phy_reset(hdmirx, 0);
				break;
			}
			reg_val = read16(hdmirx, PHY_PMA_CMN_CTRL2_ADDR);
		} while ((reg_val & 0x0004) == 0x0000);
	} else
		dev_dbg(&hdmirx->pdev->dev, "Skipping A3 power change since PHY_MODE_CTL: 0x%04X\n", reg_val);


	/* Clear power hdmirx and set transceiver resets active */
	write16(hdmirx, PHY_MODE_CTL_ADDR, 0x0000);

	/* Setting PMA Transceiver Control for each lane to default */
	for (i = 0; i < 3; i++) {
		reg_val = 0x0C61;
		write16(hdmirx, PHY_PMA_XCVR_CTRL_ADDR | (i << 6), reg_val);
	}

	/* De-assert PHY reset in case we applied it earlier due to timeout...*/
	imx8qm_hdmi_phy_reset(hdmirx, 1);
}

int pma_cmn_ready(struct cdns_hdmirx_device *hdmirx)
{
	const ktime_t timeout = ktime_timeout_ms(PMA_CMN_READY_TIMEOUT_MS);

	do {
		udelay(10);
		if (read16(hdmirx, PHY_PMA_CMN_CTRL1_ADDR) & (1 << 0))
			return 0;
	} while (ktime_before(ktime_get(), timeout));

	dev_warn(&hdmirx->pdev->dev, "%s timeout\n", __func__);

	return -1;
}

int phy_in_reset(struct cdns_hdmirx_device *hdmirx)
{
	if (read16(hdmirx, PHY_PMA_CMN_CTRL1_ADDR) & (1 << 0))
		return 0;
	else
		return 1;
}

int pma_rx_clk_signal_detect(struct cdns_hdmirx_device *hdmirx)
{
	const ktime_t timeout =
		ktime_timeout_ms(PMA_RX_CLK_SIGNAL_DETECT_TIMEOUT_MS);

	do {
		if (read16(hdmirx, PHY_MODE_CTL_ADDR) & (1 << 8))
			return 0;
	} while (ktime_before(ktime_get(), timeout));

	return -1;
}

/* This is raw version returning whether signal is currently
   detected or not whereas above will wait for signal...
*/
bool pma_rx_clk_sig_detected(struct cdns_hdmirx_device *hdmirx)
{
	if (read16(hdmirx, PHY_MODE_CTL_ADDR) & (1 << 8))
		return true;
	else
		return false;
}

static int pma_rx_clk_freq_detect(struct cdns_hdmirx_device *hdmirx)
{
	u16 reg_val;
	u32 rx_clk_freq;
	ktime_t timeout;

	/* Turn off measurement in case not already off... */
	write16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x0000);

	/* Start by triggering a frequency measurement timeout to
	   check that at is somewhat working..
	   We will reset this back to PMA_REF_CLK_TMR_VALUE_DEF later..*/
	write16(hdmirx, CMN_CMSMT_REF_CLK_TMR_VALUE_ADDR, 0x0001);

	/* Start frequency detection: */
	write16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x8000);
	udelay(5);
	/* Should be done by now */
	if (!(read16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR) & (1 << 14))) {
		dev_err(&hdmirx->pdev->dev, "%s() freq detect -> initial timeout check failed\n", __func__);
		goto meas_error;
	}
	/* Check that measured value is 0 */
	reg_val = read16(hdmirx, CMN_CMSMT_TEST_CLK_CNT_VALUE_ADDR);
	if (reg_val != 0) {
		dev_err(&hdmirx->pdev->dev, "%s() freq detect -> initial check expected 0 got 0x%04X\n", __func__, reg_val);
		goto meas_error;
	}
	/* Turn off measurement */
	write16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x0000);
	udelay(5);
	/* Zero check of status */
	reg_val = read16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR);
	if (reg_val != 0) {
		dev_err(&hdmirx->pdev->dev, "%s() freq detect -> initial check ctrl expected 0 got 0x%04X\n", __func__, reg_val);
		goto meas_error;
	}
	/* Reset count value back to default */
	write16(hdmirx, CMN_CMSMT_REF_CLK_TMR_VALUE_ADDR, PMA_REF_CLK_TMR_VALUE_DEF);

	/* Check if signal actually exists before starting measurement...*/
	if (!pma_rx_clk_sig_detected(hdmirx))
		goto meas_error;

	/* Start proper frequency detection: */
	timeout = ktime_timeout_ms(PMA_RX_CLK_FREQ_DETECT_TIMEOUT_MS);
	write16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x8000);

	/* Wait for pma_rx_clk_freq_detect_done */
	while (!(read16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR) & (1 << 14))) {
		if (ktime_after(ktime_get(), timeout)) {
			dev_err(&hdmirx->pdev->dev, "%s() freq detect -> timeout\n", __func__);
			goto meas_error;
		}
		if (!pma_rx_clk_sig_detected(hdmirx)) {
			dev_err(&hdmirx->pdev->dev, "%s() freq detect -> signal lost during measurement\n", __func__);
			goto meas_error;
		}
	}

	/* Read the measured value */
	reg_val = read16(hdmirx, CMN_CMSMT_TEST_CLK_CNT_VALUE_ADDR);

	/* Calculate TMDS clock frequency */
	rx_clk_freq = reg_val * REFCLK_FREQ_KHZ / 2048;

	/* Turn off frequency measurement: */
	write16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x0000);
	udelay(5);

	return rx_clk_freq;

meas_error:
	write16(hdmirx, CMN_CMSMT_CLK_FREQ_MSMT_CTRL_ADDR, 0x0000);
	udelay(5);
	return -1;
}

int cdns_hdmirx_get_stable_tmds(struct cdns_hdmirx_device *hdmirx)
{
	const ktime_t timeout = ktime_timeout_ms(TMDS_STABLE_DETECT_TIMEOUT_MS);
	char i = 0;
	int tmds = -1;

	dev_dbg(&hdmirx->pdev->dev, "MEASURING TMDS\n");
	do {
		int val;

		if (hdmirx->tmdsmon_state == 2) {
			dev_dbg(&hdmirx->pdev->dev, "TMDS Monitor cleanup in progress...\n");
			return -1;
		}

		val = pma_rx_clk_freq_detect(hdmirx);
		if ((tmds < (val + 50)) && (tmds > (val - 50))) {
			i++;
		} else if (tmds != val) {
			i = 0;
			tmds = val;
		}

		if (i == TMDS_STABLE_DETECT_COUNT_THRESHOLD) {
			dev_dbg(&hdmirx->pdev->dev, "DONE MEASURING TMDS, got %d\n", tmds);
			if (((tmds < PMA_RX_CLK_FREQ_DETECT_MIN_THRESH) && (tmds >= 0)) ||
			     (tmds > PMA_RX_CLK_FREQ_DETECT_MAX_THRESH)) {
				dev_dbg(&hdmirx->pdev->dev, "hdmirx_get_stable_tmds() measured value not valid %d, will try again after 100ms\n", tmds);
				i = 0;
				tmds = 0;
				mdelay(100);	/* Give source some settling time */
			} else
				return tmds; /* Can be -1 */
		}

		udelay(10);

	} while (ktime_before(ktime_get(), timeout));

	dev_warn(&hdmirx->pdev->dev, "hdmirx_get_stable_tmds() timeout\n");
	return -1;
}

int get_rescal_code(struct cdns_hdmirx_device *hdmirx)
{
	u16 reg_val;
	reg_val = read16(hdmirx, 0xD0);
	dev_dbg(&hdmirx->pdev->dev, "Done with CMN_RXCAL_CTRL: 0x%04X\n", reg_val);
	return (reg_val & 0xFF);
}

int set_rescal_code(struct cdns_hdmirx_device *hdmirx, u8 rescal_code)
{
	write16(hdmirx, 0x1EC, 0x3F4);
	write16(hdmirx, 0xD1, 0x8000 | rescal_code);
	return 0;
}
int reset_rescal_code(struct cdns_hdmirx_device *hdmirx)
{
	write16(hdmirx, 0x1EC, 0x7F4);
	write16(hdmirx, 0xD1, 0x0);
	return 0;
}

int set_slicer_tune_val(struct cdns_hdmirx_device *hdmirx, u8 tune_val)
{
	write16(hdmirx, 0x8623, tune_val);
	return 0;
}

int set_sigdet_refcnt_adj(struct cdns_hdmirx_device *hdmirx, u32 sigdet_refcnt_adj)
{
	u32 temp_val;
	temp_val = sigdet_refcnt_adj;
	write16(hdmirx, 0x8600, temp_val & 0xff);
	temp_val = temp_val >> 8;
	write16(hdmirx, 0x8601, temp_val & 0xff);
	temp_val = temp_val >> 8;
	write16(hdmirx, 0x8602, temp_val & 0xff);
	temp_val = temp_val >> 8;
	write16(hdmirx, 0x8603, temp_val & 0xff);
	return 0;
}

int pma_power_state_chng(struct cdns_hdmirx_device *hdmirx, u8 power_state)
{
	u16 reg_val;
	reg_field_t xcvr_power_state_req;
	reg_field_t xcvr_power_state_ack;
	ktime_t timeout;

	xcvr_power_state_req.label = "xcvr_power_state_req";
	xcvr_power_state_ack.label = "xcvr_power_state_ack";
	xcvr_power_state_req.msb = 3;
	xcvr_power_state_req.lsb = 0;
	xcvr_power_state_ack.msb = 7;
	xcvr_power_state_ack.lsb = 4;
	set_field_value(&xcvr_power_state_req, power_state);
	set_field_value(&xcvr_power_state_ack, power_state);

	/* Get current power state: */
	/* PHY_MODE_CTL */
	reg_val = read16(hdmirx, PHY_MODE_CTL_ADDR);
	dev_dbg(&hdmirx->pdev->dev, "pma_power_state_chng() PHY_MODE_CTL: 0x%04X\n", reg_val);

	reg_val &= 0xFFF0;
	reg_val |= set_reg_value(xcvr_power_state_req);

	write16(hdmirx, PHY_MODE_CTL_ADDR, reg_val);
	dev_dbg(&hdmirx->pdev->dev, "pma_power_state_chng() Requested power mode 0x%02X\n", power_state);

	/* Wait for power mode acknowledged: */
	/* PHY_MODE_CTL */
	timeout = ktime_timeout_ms(PMA_POWER_CHNG_TIMEOUT_MS);
	do {
		udelay(5);
		reg_val = read16(hdmirx, PHY_MODE_CTL_ADDR);
		if ((reg_val & 0x00F0) == set_reg_value(xcvr_power_state_ack)) {
			write16(hdmirx, PHY_MODE_CTL_ADDR, reg_val & 0xFFF0);
			dev_dbg(&hdmirx->pdev->dev, "pma_power_state_chng() Done with PHY_MODE_CTL: 0x%04X\n", reg_val);
			return 0;
		}
	} while (ktime_before(ktime_get(), timeout));

	dev_warn(&hdmirx->pdev->dev, "pma_power_state_chng() Timed out with PHY_MODE_CTL: 0x%04X\n", reg_val);
	write16(hdmirx, PHY_MODE_CTL_ADDR, reg_val & 0xFFF0);
	return -1;
}

int pma_pll_config(struct cdns_hdmirx_device *hdmirx,
		    u32 rx_clk_freq,
		    u32 clk_ratio,
		    u32 tmds_bit_clk_ratio,
		    u8 data_rate_change)
{
	int i, loop;
	u16 reg_val;
	u64 vco_freq_khz;
	const ktime_t timeout = ktime_timeout_ms(1000);

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

	dev_dbg(&hdmirx->pdev->dev, "pma_pll_config() Configuring PLL0 ...\n");

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
		} else {
			dev_err(&hdmirx->pdev->dev, "TMDS clock frequency (%d KHz) is out of range\n", rx_clk_freq);
			return -1;
		}

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
		} else {
			dev_err(&hdmirx->pdev->dev, "pma_pll_config() *E: TMDS clock frequency (%d kHz) is out of range\n",
			     rx_clk_freq);
			return -1;
		}
	}

	vco_freq_khz =
	    (2048 * (u64) rx_clk_freq * (1 << cmn_pll_clk_osr.value) * tmds_bit_clk_ratio) / 2047;

	dev_info(&hdmirx->pdev->dev, "VCO frequency (refclk: %d kHz, TMDS clk: %d kHz, OSR: %0d, tmds_bit_clk_ratio: %d) equals %llu kHz\n",
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
	} else {
		dev_err(&hdmirx->pdev->dev, "%s VCO frequency (%llu KHz) is out of range\n", __func__, vco_freq_khz);
		return -1;
	}

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
	write16(hdmirx, CMN_DIAG_PLL0_INCLK_CTRL_ADDR, reg_val);

	/* CMN_DIAG_PLL0_FBH_OVRD */
	reg_val = set_reg_value(cmn_pll0_fb_div_high_ovrd_en);
	reg_val |= set_reg_value(cmnda_pll0_fb_div_high_out);
	write16(hdmirx, CMN_DIAG_PLL0_FBH_OVRD_ADDR, reg_val);

	/* CMN_DIAG_PLL0_FBL_OVRD */
	reg_val = set_reg_value(cmn_pll0_fb_div_low_ovrd_en);
	reg_val |= set_reg_value(cmnda_pll0_fb_div_low_out);
	write16(hdmirx, CMN_DIAG_PLL0_FBL_OVRD_ADDR, reg_val);

	/* CMN_PLL0_DIV2SEL_OSR_CTRL */
	reg_val = set_reg_value(cmn_pll_clk_osr);
	reg_val |= set_reg_value(cmn_pll_clk_div2_ratio);
	reg_val |= set_reg_value(cmn_pll_clk_div2_sel);
	write16(hdmirx, CMN_PLL0_DIV2SEL_OSR_CTRL_ADDR, reg_val);

	/* RX_DIAG_SMPLR_OSR */
	reg_val = set_reg_value(rx_diag_smplr_osr);
	multi_write16(hdmirx, RX_DIAG_SMPLR_OSR_ADDR, reg_val);

	/* RX_PSC_A0 */
	reg_val = set_reg_value(rx_psc_a0);
	multi_write16(hdmirx, RX_PSC_A0_ADDR, reg_val);

	/* RX_REE_PERGCSM_EQENM_PH1 */
	reg_val = set_reg_value(rx_ree_pergcsm_eqenm_ph1);
	multi_write16(hdmirx, RX_REE_PERGCSM_EQENM_PH1_ADDR, reg_val);

	/* RX_REE_PERGCSM_EQENM_PH1 */
	reg_val = set_reg_value(rx_ree_pergcsm_eqenm_ph2);
	multi_write16(hdmirx, RX_REE_PERGCSM_EQENM_PH2_ADDR, reg_val);

	/* RX_REE_VGA_GAIN_OVRD */
	reg_val = set_reg_value(vga_gain_accum_override_en);
	reg_val |= set_reg_value(vga_gain_accum_override);
	reg_val |= set_reg_value(vga_gain_tgt_adj_override_en);
	reg_val |= set_reg_value(vga_gain_tgt_adj_override);
	multi_write16(hdmirx, RX_REE_VGA_GAIN_OVRD_ADDR, reg_val);

	/* RX_REE_SMGM_CTRL1 */
	reg_val = set_reg_value(ree_gen_sm_en_usb);
	reg_val |= set_reg_value(ree_gen_sm_en_periodic);
	reg_val |= set_reg_value(ana_en_epath_gen_ctrl_sm_usb);
	reg_val |= set_reg_value(ana_en_epath_gen_ctrl_sm_periodic);
	multi_write16(hdmirx, RX_REE_SMGM_CTRL1_ADDR, reg_val);

	/* RX_DIAG_DFE_CTRL2 */
	reg_val = set_reg_value(rxda_eq_range_sel);
	reg_val |= set_reg_value(rxda_vga_sa_range_sel);
	multi_write16(hdmirx, RX_DIAG_DFE_CTRL2_ADDR, reg_val);

	/* CMN_PLLSM0_USER_DEF_CTRL */
	reg_val = set_reg_value(vco_ring_select);
	write16(hdmirx, CMN_PLLSM0_USER_DEF_CTRL_ADDR, reg_val);

	/* CMN_DIAG_PLL0_V2I_TUNE */
	reg_val = set_reg_value(cmnda_pll0_v2i_prog);
	reg_val |= set_reg_value(cmnda_pll0_coarse_prog);
	write16(hdmirx, CMN_DIAG_PLL0_V2I_TUNE_ADDR, reg_val);

	/* CMN_DIAG_PLL0_CP_TUNE */
	reg_val = set_reg_value(cmnda_pll0_cp_gain);
	write16(hdmirx, CMN_DIAG_PLL0_CP_TUNE_ADDR, reg_val);

	/* CMN_DIAG_PLL0_PTATIS_TUNE1 */
	reg_val = set_reg_value(cmnda_pll0_const_ndac_cntrl);
	reg_val |= set_reg_value(cmnda_pll0_const_pmos_cntrl);
	write16(hdmirx, CMN_DIAG_PLL0_PTATIS_TUNE1_ADDR, reg_val);

	/* CMN_DIAG_PLL0_PTATIS_TUNE2 */
	reg_val = set_reg_value(cmnda_pll0_ptat_ndac_cntrl);
	write16(hdmirx, CMN_DIAG_PLL0_PTATIS_TUNE2_ADDR, reg_val);

	/* RX_DIAG_ILL_IQ_TRIM0 */
	reg_val = set_reg_value(rxda_pi_iq_bias_trim);
	reg_val |= set_reg_value(rxda_pi_iq_pload_bias_trim);
	reg_val |= set_reg_value(rxda_pi_iq_pload_trim);
	write16(hdmirx, RX_DIAG_ILL_IQ_TRIM0_ADDR, reg_val);

	/* RX_DIAG_ILL_E_TRIM0 */
	reg_val = set_reg_value(rxda_pi_e_bias_trim);
	reg_val |= set_reg_value(rxda_pi_e_pload_bias_trim);
	reg_val |= set_reg_value(rxda_pi_e_pload_trim);
	write16(hdmirx, RX_DIAG_ILL_E_TRIM0_ADDR, reg_val);

	/* RX_DIAG_ILL_IQE_TRIM2 */
	reg_val = set_reg_value(rxda_pi_range_sel);
	reg_val |= set_reg_value(rxda_pi_cal_cm_trim);
	write16(hdmirx, RX_DIAG_ILL_IQE_TRIM2_ADDR, reg_val);

	/* Enable PLL */
	/* PHY_MODE_CTL */
	reg_val = set_reg_value(xcvr_pll_en);
	reg_val |= set_reg_value(xcvr_link_reset_n);
	reg_val |= set_reg_value(xcvr_power_state_req);
	write16(hdmirx, PHY_MODE_CTL_ADDR, reg_val);

	/* Wait for PLL0 ready: */
	/* PHY_PMA_CMN_CTRL2 */
	do {
		if (ktime_after(ktime_get(), timeout))
			goto timeout_err;
		udelay(10);
	} while ((read16(hdmirx, PHY_PMA_CMN_CTRL2_ADDR) & (1 << 0)) == 0);

	/* Turn on output clocks: */
	/* PHY_PMA_CMN_CTRL2 */
	reg_val = set_reg_value(iso_pma_cmn_pll0_clk_datart1_div);
	reg_val |= set_reg_value(iso_pma_cmn_pll0_clk_datart0_div);
	reg_val |= set_reg_value(iso_pma_cmn_pll0_clk_en);
	write16(hdmirx, PHY_PMA_CMN_CTRL2_ADDR, reg_val);

	dev_dbg(&hdmirx->pdev->dev, "pma_pll_config() Disable Rx Eq Training\n");
	for (i = 0; i < 3; i++) {
		reg_val =
		    read16(hdmirx, PHY_PMA_XCVR_CTRL_ADDR | (i << 6));
		reg_val &= 0xFFEF;
		write16(hdmirx, PHY_PMA_XCVR_CTRL_ADDR | (i << 6), reg_val);
	}

	/* Get current power state: */
	/* PHY_MODE_CTL */
	reg_val = read16(hdmirx, PHY_MODE_CTL_ADDR);
	dev_dbg(&hdmirx->pdev->dev, "pma_pll_config() PHY_MODE_CTL: 0x%04X\n", reg_val);

	reg_val &= 0x00F0;
	dev_dbg(&hdmirx->pdev->dev, "pma_pll_config() Current power state: 0x%02X\n", (reg_val >> 4));

	/* Deassert link reset: */
	/* PHY_MODE_CTL */
	dev_dbg(&hdmirx->pdev->dev, "pma_pll_config() Deassert link reset\n");
	set_field_value(&xcvr_link_reset_n, 0x1);
	reg_val |= set_reg_value(xcvr_pll_en);
	reg_val |= set_reg_value(xcvr_link_reset_n);
	write16(hdmirx, PHY_MODE_CTL_ADDR, reg_val);

	/* Wait for xcvr_psm_ready for all the lanes */
	loop = 0;
	do {
		reg_val = (1 << 13);
		if (ktime_after(ktime_get(), timeout))
			goto timeout_err;

		for (i = 0; i < 3; i++) {
			reg_val &= read16(hdmirx, PHY_PMA_XCVR_CTRL_ADDR | (i << 6)) & (1 << 13);
			dev_dbg(&hdmirx->pdev->dev, "pma_pll_config() xcvr_psm_ready(%0d): 0x%0X\n", i, reg_val >> 13);
		}
	} while (!reg_val && loop < 20);

	/* Set A0 power state: */
	/* PHY_MODE_CTL */
	if (pma_power_state_chng(hdmirx, 0x1))
		goto timeout_err;

	dev_dbg(&hdmirx->pdev->dev, "pma_pll_config() Enable Rx Eq Training\n");
	for (i = 0; i < 3; i++) {
		reg_val =
		    read16(hdmirx, PHY_PMA_XCVR_CTRL_ADDR | (i << 6));
		reg_val |= 0x0010;
		write16(hdmirx, PHY_PMA_XCVR_CTRL_ADDR | (i << 6), reg_val);
	}

	return 0;

timeout_err:
	dev_err(&hdmirx->pdev->dev, "pma_pll_config() Timeout error!\n");
	return -1;

}

u32 clk_ratio_detect(struct cdns_hdmirx_device *hdmirx,
				u32 rx_clk_freq,	/* khz */
				u32 pxl_clk_freq,	/* khz */
				u8 vic,
				u32 pixel_encoding,
				u32 tmds_bit_clk_ratio)
{
	u32 clk_ratio_detected = CLK_RATIO_1_1;

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
	dev_info(&hdmirx->pdev->dev, "VIC %0d, pixel encoding: %0d, TMDS bit clock ratio: %0d and TMDS clk %d KHz\n",
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
	    && rx_clk_freq < tmds_freq_nominal_1_1_max)
		clk_ratio_detected = CLK_RATIO_1_1;
	else if (rx_clk_freq > tmds_freq_nominal_5_4_min
		   && rx_clk_freq < tmds_freq_nominal_5_4_max)
		clk_ratio_detected = CLK_RATIO_5_4;
	else if (rx_clk_freq > tmds_freq_nominal_3_2_min
		   && rx_clk_freq < tmds_freq_nominal_3_2_max)
		clk_ratio_detected = CLK_RATIO_3_2;
	else if (rx_clk_freq > tmds_freq_nominal_2_1_min
		   && rx_clk_freq < tmds_freq_nominal_2_1_max)
		clk_ratio_detected = CLK_RATIO_2_1;
	else if (rx_clk_freq > tmds_freq_nominal_1_2_min
		   && rx_clk_freq < tmds_freq_nominal_1_2_max)
		clk_ratio_detected = CLK_RATIO_1_2;
	else if (rx_clk_freq > tmds_freq_nominal_5_8_min
		   && rx_clk_freq < tmds_freq_nominal_5_8_max)
		clk_ratio_detected = CLK_RATIO_5_8;
	else if (rx_clk_freq > tmds_freq_nominal_3_4_min
		   && rx_clk_freq < tmds_freq_nominal_3_4_max)
		clk_ratio_detected = CLK_RATIO_3_4;
	else {
		dev_err(&hdmirx->pdev->dev, "Unable to detected TMDS/pixel clock ratio - using default\n");
		dev_err(&hdmirx->pdev->dev, "VIC: %02d and TMDS clock of %d KHz\n", vic, rx_clk_freq);
	}
	dev_dbg(&hdmirx->pdev->dev, "Detected TMDS/pixel clock ratio of %d\n", clk_ratio_detected);

	return clk_ratio_detected;
}
