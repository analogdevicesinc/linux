/*
 * Copyright 2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _CDNS_HDMIRX_PHY_H_
#define _CDNS_HDMIRX_PHY_H_

#include <linux/io.h>
#include "cdns-mhdp-hdmirx.h"

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
#define RX_CLK_SLICER_CAL_TUNE_ADDR            0x8623
#define RX_CLK_SLICER_CAL_INIT_TMR_ADDR        0x8624
#define RX_CLK_SLICER_CAL_ITER_TMR_ADDR        0x8625
#define RX_CLK_TERM_CTRL_ADDR                  0x8660
#define PHY_MODE_CTL_ADDR                      0xC008
#define PHY_PMA_CMN_CTRL1_ADDR                 0xC800
#define PHY_PMA_CMN_CTRL2_ADDR                 0xC801
#define PHY_PMA_SSM_STATE_ADDR                 0xC802
#define PHY_PMA_PLL_SM_STATE_ADDR              0xC803
#define PHY_PMA_XCVR_CTRL_ADDR                 0xCC00

enum {
  TMDS_BIT_CLOCK_RATIO_1_10 = 10,
  TMDS_BIT_CLOCK_RATIO_1_40 = 40
};

void arc_config(struct cdns_hdmirx_device *hdmirx);
void pma_config(struct cdns_hdmirx_device *hdmirx);
int pma_cmn_ready(struct cdns_hdmirx_device *hdmirx);
int phy_in_reset(struct cdns_hdmirx_device *hdmirx);
int pma_rx_clk_signal_detect(struct cdns_hdmirx_device *hdmirx);
bool pma_rx_clk_sig_detected(struct cdns_hdmirx_device *hdmirx);
void pre_data_rate_change(struct cdns_hdmirx_device *hdmirx);
int pma_pll_config(struct cdns_hdmirx_device *hdmirx, u32, u32, u32, u8);
u32 clk_ratio_detect(struct cdns_hdmirx_device *hdmirx, u32, u32, u8, u32, u32);
int get_rescal_code(struct cdns_hdmirx_device *hdmirx);
int set_rescal_code(struct cdns_hdmirx_device *hdmirx, u8 rescal_code);
int reset_rescal_code(struct cdns_hdmirx_device *hdmirx);
int set_slicer_tune_val(struct cdns_hdmirx_device *hdmirx, u8 tune_val);
int set_sigdet_refcnt_adj(struct cdns_hdmirx_device *hdmirx, u32 sigdet_refcnt_adj);
#endif
