/**
 * cdns3-nxp-reg-def.h - nxp wrap layer register definition
 *
 * Copyright 2017 NXP
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DRIVERS_USB_CDNS3_NXP_H
#define __DRIVERS_USB_CDNS3_NXP_H

#define USB3_CORE_CTRL1    0x00
#define USB3_CORE_CTRL2    0x04
#define USB3_INT_REG       0x08
#define USB3_CORE_STATUS   0x0c
#define XHCI_DEBUG_LINK_ST 0x10
#define XHCI_DEBUG_BUS     0x14
#define USB3_SSPHY_CTRL1   0x40
#define USB3_SSPHY_CTRL2   0x44
#define USB3_SSPHY_STATUS  0x4c
#define USB2_PHY_CTRL1     0x50
#define USB2_PHY_CTRL2     0x54
#define USB2_PHY_STATUS    0x5c

/* Register bits definition */

/* USB3_CORE_CTRL1 */
#define PWR_SW_RESET	(1 << 31)
#define APB_SW_RESET	(1 << 30)
#define AXI_SW_RESET	(1 << 29)
#define RW_SW_RESET	(1 << 28)
#define PHY_SW_RESET	(1 << 27)
#define PHYAHB_SW_RESET	(1 << 26)
#define ALL_SW_RESET	(PWR_SW_RESET | APB_SW_RESET | AXI_SW_RESET | \
		RW_SW_RESET | PHY_SW_RESET | PHYAHB_SW_RESET)
#define OC_DISABLE	(1 << 9)
#define MODE_STRAP_MASK	(0x7)
#define DEV_MODE	(1 << 2)
#define HOST_MODE	(1 << 1)
#define OTG_MODE	(1 << 0)

/* USB3_INT_REG */
#define HOST_INT1_EN (1 << 0) /* HOST INT b7:0 */
#define DEV_INT_EN (3 << 8) /* DEV INT b9:8 */

/* USB3_CORE_STATUS */
#define DEV_POWER_ON_READY	(1 << 13)
#define HOST_POWER_ON_READY	(1 << 12)


/* PHY register definition */
#define PHY_PMA_CMN_CTRL1			(0xC800 * 4)
#define TB_ADDR_CMN_DIAG_HSCLK_SEL		(0x01e0 * 4)
#define TB_ADDR_CMN_PLL0_VCOCAL_INIT_TMR	(0x0084 * 4)
#define TB_ADDR_CMN_PLL0_VCOCAL_ITER_TMR	(0x0085 * 4)
#define TB_ADDR_CMN_PLL0_INTDIV	                (0x0094 * 4)
#define TB_ADDR_CMN_PLL0_FRACDIV		(0x0095 * 4)
#define TB_ADDR_CMN_PLL0_HIGH_THR		(0x0096 * 4)
#define TB_ADDR_CMN_PLL0_SS_CTRL1		(0x0098 * 4)
#define TB_ADDR_CMN_PLL0_SS_CTRL2		(0x0099 * 4)
#define TB_ADDR_CMN_PLL0_DSM_DIAG		(0x0097 * 4)
#define TB_ADDR_CMN_DIAG_PLL0_OVRD		(0x01c2 * 4)
#define TB_ADDR_CMN_DIAG_PLL0_FBH_OVRD		(0x01c0 * 4)
#define TB_ADDR_CMN_DIAG_PLL0_FBL_OVRD		(0x01c1 * 4)
#define TB_ADDR_CMN_DIAG_PLL0_V2I_TUNE          (0x01C5 * 4)
#define TB_ADDR_CMN_DIAG_PLL0_CP_TUNE           (0x01C6 * 4)
#define TB_ADDR_CMN_DIAG_PLL0_LF_PROG           (0x01C7 * 4)
#define TB_ADDR_CMN_DIAG_PLL0_TEST_MODE		(0x01c4 * 4)
#define TB_ADDR_CMN_PSM_CLK_CTRL		(0x0061 * 4)
#define TB_ADDR_XCVR_DIAG_RX_LANE_CAL_RST_TMR	(0x40ea * 4)
#define TB_ADDR_XCVR_PSM_RCTRL	                (0x4001 * 4)
#define TB_ADDR_TX_PSC_A0		        (0x4100 * 4)
#define TB_ADDR_TX_PSC_A1		        (0x4101 * 4)
#define TB_ADDR_TX_PSC_A2		        (0x4102 * 4)
#define TB_ADDR_TX_PSC_A3		        (0x4103 * 4)
#define TB_ADDR_TX_DIAG_ECTRL_OVRD		(0x41f5 * 4)
#define TB_ADDR_TX_PSC_CAL		        (0x4106 * 4)
#define TB_ADDR_TX_PSC_RDY		        (0x4107 * 4)
#define TB_ADDR_RX_PSC_A0	                (0x8000 * 4)
#define TB_ADDR_RX_PSC_A1	                (0x8001 * 4)
#define TB_ADDR_RX_PSC_A2	                (0x8002 * 4)
#define TB_ADDR_RX_PSC_A3	                (0x8003 * 4)
#define TB_ADDR_RX_PSC_CAL	                (0x8006 * 4)
#define TB_ADDR_RX_PSC_RDY	                (0x8007 * 4)
#define TB_ADDR_TX_TXCC_MGNLS_MULT_000		(0x4058 * 4)
#define TB_ADDR_TX_DIAG_BGREF_PREDRV_DELAY	(0x41e7 * 4)
#define TB_ADDR_RX_SLC_CU_ITER_TMR		(0x80e3 * 4)
#define TB_ADDR_RX_SIGDET_HL_FILT_TMR		(0x8090 * 4)
#define TB_ADDR_RX_SAMP_DAC_CTRL		(0x8058 * 4)
#define TB_ADDR_RX_DIAG_SIGDET_TUNE		(0x81dc * 4)
#define TB_ADDR_RX_DIAG_LFPSDET_TUNE2		(0x81df * 4)
#define TB_ADDR_RX_DIAG_BS_TM	                (0x81f5 * 4)
#define TB_ADDR_RX_DIAG_DFE_CTRL1		(0x81d3 * 4)
#define TB_ADDR_RX_DIAG_ILL_IQE_TRIM4		(0x81c7 * 4)
#define TB_ADDR_RX_DIAG_ILL_E_TRIM0		(0x81c2 * 4)
#define TB_ADDR_RX_DIAG_ILL_IQ_TRIM0		(0x81c1 * 4)
#define TB_ADDR_RX_DIAG_ILL_IQE_TRIM6		(0x81c9 * 4)
#define TB_ADDR_RX_DIAG_RXFE_TM3		(0x81f8 * 4)
#define TB_ADDR_RX_DIAG_RXFE_TM4		(0x81f9 * 4)
#define TB_ADDR_RX_DIAG_LFPSDET_TUNE		(0x81dd * 4)
#define TB_ADDR_RX_DIAG_DFE_CTRL3		(0x81d5 * 4)
#define TB_ADDR_RX_DIAG_SC2C_DELAY		(0x81e1 * 4)
#define TB_ADDR_RX_REE_VGA_GAIN_NODFE		(0x81bf * 4)
#define TB_ADDR_XCVR_PSM_CAL_TMR		(0x4002 * 4)
#define TB_ADDR_XCVR_PSM_A0BYP_TMR		(0x4004 * 4)
#define TB_ADDR_XCVR_PSM_A0IN_TMR		(0x4003 * 4)
#define TB_ADDR_XCVR_PSM_A1IN_TMR		(0x4005 * 4)
#define TB_ADDR_XCVR_PSM_A2IN_TMR		(0x4006 * 4)
#define TB_ADDR_XCVR_PSM_A3IN_TMR		(0x4007 * 4)
#define TB_ADDR_XCVR_PSM_A4IN_TMR		(0x4008 * 4)
#define TB_ADDR_XCVR_PSM_A5IN_TMR		(0x4009 * 4)
#define TB_ADDR_XCVR_PSM_A0OUT_TMR		(0x400a * 4)
#define TB_ADDR_XCVR_PSM_A1OUT_TMR		(0x400b * 4)
#define TB_ADDR_XCVR_PSM_A2OUT_TMR		(0x400c * 4)
#define TB_ADDR_XCVR_PSM_A3OUT_TMR		(0x400d * 4)
#define TB_ADDR_XCVR_PSM_A4OUT_TMR		(0x400e * 4)
#define TB_ADDR_XCVR_PSM_A5OUT_TMR		(0x400f * 4)
#define TB_ADDR_TX_RCVDET_EN_TMR	        (0x4122 * 4)
#define TB_ADDR_TX_RCVDET_ST_TMR	        (0x4123 * 4)
#define TB_ADDR_XCVR_DIAG_LANE_FCM_EN_MGN_TMR	(0x40f2 * 4)

#endif /* __DRIVERS_USB_CDNS3_NXP_H */
