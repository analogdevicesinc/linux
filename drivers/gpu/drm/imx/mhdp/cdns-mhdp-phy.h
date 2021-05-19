/*
 * Copyright 2019-2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _CDN_DP_PHY_H
#define _CDN_DP_PHY_H

#include <drm/bridge/cdns-mhdp.h>

#define CMN_SSM_BIAS_TMR                0x0022
#define CMN_PLLSM0_PLLEN_TMR            0x0029
#define CMN_PLLSM0_PLLPRE_TMR           0x002A
#define CMN_PLLSM0_PLLVREF_TMR          0x002B
#define CMN_PLLSM0_PLLLOCK_TMR          0x002C
#define CMN_PLLSM0_USER_DEF_CTRL        0x002F
#define CMN_PSM_CLK_CTRL                0x0061
#define CMN_CDIAG_REFCLK_CTRL           0x0062
#define CMN_PLL0_VCOCAL_START           0x0081
#define CMN_PLL0_VCOCAL_INIT_TMR        0x0084
#define CMN_PLL0_VCOCAL_ITER_TMR        0x0085
#define CMN_PLL0_INTDIV                 0x0094
#define CMN_PLL0_FRACDIV                0x0095
#define CMN_PLL0_HIGH_THR               0x0096
#define CMN_PLL0_DSM_DIAG               0x0097
#define CMN_PLL0_SS_CTRL1               0x0098
#define CMN_PLL0_SS_CTRL2               0x0099
#define CMN_ICAL_INIT_TMR               0x00C4
#define CMN_ICAL_ITER_TMR               0x00C5
#define CMN_RXCAL_INIT_TMR              0x00D4
#define CMN_RXCAL_ITER_TMR              0x00D5
#define CMN_TXPUCAL_CTRL                0x00E0
#define CMN_TXPUCAL_INIT_TMR            0x00E4
#define CMN_TXPUCAL_ITER_TMR            0x00E5
#define CMN_TXPDCAL_CTRL                0x00F0
#define CMN_TXPDCAL_INIT_TMR            0x00F4
#define CMN_TXPDCAL_ITER_TMR            0x00F5
#define CMN_ICAL_ADJ_INIT_TMR           0x0102
#define CMN_ICAL_ADJ_ITER_TMR           0x0103
#define CMN_RX_ADJ_INIT_TMR             0x0106
#define CMN_RX_ADJ_ITER_TMR             0x0107
#define CMN_TXPU_ADJ_CTRL               0x0108
#define CMN_TXPU_ADJ_INIT_TMR           0x010A
#define CMN_TXPU_ADJ_ITER_TMR           0x010B
#define CMN_TXPD_ADJ_CTRL               0x010c
#define CMN_TXPD_ADJ_INIT_TMR           0x010E
#define CMN_TXPD_ADJ_ITER_TMR           0x010F
#define CMN_DIAG_PLL0_FBH_OVRD          0x01C0
#define CMN_DIAG_PLL0_FBL_OVRD          0x01C1
#define CMN_DIAG_PLL0_OVRD              0x01C2
#define CMN_DIAG_PLL0_TEST_MODE         0x01C4
#define CMN_DIAG_PLL0_V2I_TUNE          0x01C5
#define CMN_DIAG_PLL0_CP_TUNE           0x01C6
#define CMN_DIAG_PLL0_LF_PROG           0x01C7
#define CMN_DIAG_PLL0_PTATIS_TUNE1      0x01C8
#define CMN_DIAG_PLL0_PTATIS_TUNE2      0x01C9
#define CMN_DIAG_PLL0_INCLK_CTRL        0x01CA
#define CMN_DIAG_PLL0_PXL_DIVH          0x01CB
#define CMN_DIAG_PLL0_PXL_DIVL          0x01CC
#define CMN_DIAG_HSCLK_SEL              0x01E0
#define CMN_DIAG_PER_CAL_ADJ            0x01EC
#define CMN_DIAG_CAL_CTRL               0x01ED
#define CMN_DIAG_ACYA                   0x01FF
#define XCVR_PSM_RCTRL                  0x4001
#define XCVR_PSM_CAL_TMR                0x4002
#define XCVR_PSM_A0IN_TMR               0x4003
#define TX_TXCC_CAL_SCLR_MULT_0         0x4047
#define TX_TXCC_CPOST_MULT_00_0         0x404C
#define TX_TXCC_MGNFS_MULT_000_0        0x4050
#define XCVR_DIAG_PLLDRC_CTRL           0x40E0
#define XCVR_DIAG_PLLDRC_CTRL           0x40E0
#define XCVR_DIAG_HSCLK_SEL             0x40E1
#define XCVR_DIAG_BIDI_CTRL             0x40E8
#define XCVR_DIAG_LANE_FCM_EN_MGN_TMR   0x40F2
#define XCVR_DIAG_LANE_FCM_EN_MGN       0x40F2
#define TX_PSC_A0                       0x4100
#define TX_PSC_A1                       0x4101
#define TX_PSC_A2                       0x4102
#define TX_PSC_A3                       0x4103
#define TX_RCVDET_CTRL                  0x4120
#define TX_RCVDET_EN_TMR                0x4122
#define TX_RCVDET_EN_TMR                0x4122
#define TX_RCVDET_ST_TMR                0x4123
#define TX_RCVDET_ST_TMR                0x4123
#define TX_BIST_CTRL                    0x4140
#define TX_BIST_UDDWR                   0x4141
#define TX_DIAG_TX_CTRL                 0x41E0
#define TX_DIAG_TX_DRV                  0x41E1
#define TX_DIAG_BGREF_PREDRV_DELAY      0x41E7
#define TX_DIAG_BGREF_PREDRV_DELAY      0x41E7
#define XCVR_PSM_RCTRL_1                0x4201
#define TX_TXCC_CAL_SCLR_MULT_1         0x4247
#define TX_TXCC_CPOST_MULT_00_1         0x424C
#define TX_TXCC_MGNFS_MULT_000_1        0x4250
#define XCVR_DIAG_PLLDRC_CTRL_1         0x42E0
#define XCVR_DIAG_HSCLK_SEL_1           0x42E1
#define XCVR_DIAG_LANE_FCM_EN_MGN_TMR_1 0x42F2
#define TX_RCVDET_EN_TMR_1              0x4322
#define TX_RCVDET_ST_TMR_1              0x4323
#define TX_DIAG_ACYA_0                  0x41FF
#define TX_DIAG_ACYA_1                  0x43FF
#define TX_DIAG_ACYA_2                  0x45FF
#define TX_DIAG_ACYA_3                  0x47FF
#define TX_ANA_CTRL_REG_1               0x5020
#define TX_ANA_CTRL_REG_2               0x5021
#define TXDA_COEFF_CALC                 0x5022
#define TX_DIG_CTRL_REG_1               0x5023
#define TX_DIG_CTRL_REG_2               0x5024
#define TXDA_CYA_AUXDA_CYA              0x5025
#define TX_ANA_CTRL_REG_3               0x5026
#define TX_ANA_CTRL_REG_4               0x5027
#define TX_ANA_CTRL_REG_5               0x5029
#define RX_PSC_A0                       0x8000
#define RX_PSC_CAL                      0x8006
#define PMA_LANE_CFG                    0xC000
#define PIPE_CMN_CTRL1                  0xC001
#define PIPE_CMN_CTRL2                  0xC002
#define PIPE_COM_LOCK_CFG1              0xC003
#define PIPE_COM_LOCK_CFG2              0xC004
#define PIPE_RCV_DET_INH                0xC005
#define PHY_HDP_MODE_CTRL               0xC008
#define PHY_HDP_CLK_CTL                 0xC009
#define STS                             0xC00F
#define PHY_ISO_CMN_CTRL                0xC010
#define PHY_ISO_CMN_CTRL                0xC010
#define PHY_HDP_TX_CTL_L0               0xC408
#define PHY_DP_TX_CTL                   0xC408
#define PHY_HDP_TX_CTL_L1               0xC448
#define PHY_HDP_TX_CTL_L2               0xC488
#define PHY_HDP_TX_CTL_L3               0xC4C8
#define PHY_PMA_CMN_CTRL1               0xC800
#define PMA_CMN_CTRL1                   0xC800
#define PHY_PMA_ISO_CMN_CTRL            0xC810
#define PHY_PMA_ISO_PLL_CTRL1           0xC812
#define PHY_PMA_ISOLATION_CTRL          0xC81F
#define PHY_ISOLATION_CTRL              0xC81F
#define PHY_PMA_ISO_XCVR_CTRL           0xCC11
#define PHY_PMA_ISO_LINK_MODE           0xCC12
#define PHY_PMA_ISO_PWRST_CTRL          0xCC13
#define PHY_PMA_ISO_TX_DATA_LO          0xCC14
#define PHY_PMA_ISO_TX_DATA_HI          0xCC15
#define PHY_PMA_ISO_RX_DATA_LO          0xCC16
#define PHY_PMA_ISO_RX_DATA_HI          0xCC17

int cdns_dp_phy_set_imx8mq(struct cdns_mhdp_device *hdp);
int cdns_dp_phy_set_imx8qm(struct cdns_mhdp_device *hdp);
int cdns_dp_phy_shutdown(struct cdns_mhdp_device *mhdp);
bool cdns_hdmi_phy_video_valid_imx8mq(struct cdns_mhdp_device *hdp);
bool cdns_hdmi_phy_video_valid_imx8qm(struct cdns_mhdp_device *hdp);
int cdns_hdmi_phy_set_imx8mq(struct cdns_mhdp_device *hdp);
int cdns_hdmi_phy_set_imx8qm(struct cdns_mhdp_device *hdp);
int cdns_hdmi_phy_shutdown(struct cdns_mhdp_device *mhdp);
int cdns_hdmi_phy_power_up(struct cdns_mhdp_device *mhdp);
#endif /* _CDNS_MHDP_PHY_H */
