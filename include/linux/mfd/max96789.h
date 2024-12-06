// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2023 NXP
 */

#ifndef _MAX96789_H_
#define _MAX96789_H_

/* DEV */
#define MAX96789_DEV_ADDR					  0x0000
#define   CFG_BLOCK						  BIT(0)
#define   DEV_ADDR_MASK						  GENMASK(7, 1)
#define   DEV_ADDR_SHIFT					  1
#define MAX96789_DEV_CH_CTRL					  0x0001
#define   TX_RATE_MASK						  GENMASK(3, 2)
#define   TX_RATE_SHIFT						  2
#define   DIS_REM_CC						  BIT(4)
#define   DIS_LOCAL_CC						  BIT(5)
#define   IIC_1_EN						  BIT(6)
#define   IIC_2_EN						  BIT(7)
#define MAX96789_DEV_PIPE_EN					  0x0002
#define   AUD_TX_EN_X						  BIT(2)
#define   AUD_TX_EN_Y						  BIT(3)
#define   VID_TX_EN_X						  BIT(4)
#define   VID_TX_EN_Y						  BIT(5)
#define   VID_TX_EN_Z						  BIT(6)
#define   VID_TX_EN_U						  BIT(7)
#define MAX96789_DEV_AUD_UART_CFG				  0x0003
#define   RCLKSEL_MASK						  GENMASK(1, 0)
#define   RCLKSEL_SHIFT						  0
#define   AUD_RX_A_SRC						  BIT(3)
#define   UART_1_EN						  BIT(4)
#define   UART_2_EN						  BIT(5)
#define   AUD_RX_B_SRC						  BIT(6)
#define MAX96789_DEV_LINK					  0x0004
#define   AUD_TX_SRC_X						  BIT(0)
#define   AUD_TX_SRC_Y						  BIT(1)
#define   LINK_EN_A						  BIT(4)
#define   LINK_EN_B						  BIT(5)
#define   GMSL2_A						  BIT(6)
#define   GMSL2_B						  BIT(7)
#define MAX96789_DEV_FAULT_MON					  0x0005
#define   PU_LF0						  BIT(0)
#define   PU_LF1						  BIT(1)
#define   PU_LF2						  BIT(2)
#define   PU_LF3						  BIT(3)
#define   LOCK_CFG						  BIT(5)
#define   ERRB_EN						  BIT(6)
#define   LOCK_EN						  BIT(7)
#define MAX96789_DEV_RCLK					  0x0006
#define   RCLKEN						  BIT(5)
#define MAX96789_DEV_ID						  0x000d
#define MAX96789_DEV_REV					  0x000e
#define   DEV_REV_MASK						  GENMASK(3, 0)
#define   DEV_REV_SHIFT						  0
#define MAX96789_DEV_VIDEO_CAP					  0x000f
#define   HDCP_CPBL						  BIT(0)
#define   SPLTR_CPBL_N						  BIT(1)
#define   DUAL_CPBL_N						  BIT(2)
#define   DV_CPBL_N						  BIT(3)
#define   SPEED_CPBL_MASK					  GENMASK(5, 4)
#define   SPEED_CPBL_SHIFT					  4
#define   ASYM_DV_DIS						  BIT(6)
#define MAX96789_DEV_LF_STATUS1					  0x0026
#define   LF_0_MASK						  GENMASK(2, 0)
#define   LF_0_SHIFT						  0
#define   LF_1_MASK						  GENMASK(6, 4)
#define   LF_1_SHIFT						  4
#define MAX96789_DEV_LF_STATUS2					  0x0027
#define   LF_2_MASK						  GENMASK(2, 0)
#define   LF_2_SHIFT						  0
#define   LF_3_MASK						  GENMASK(6, 4)
#define   LF_3_SHIFT						  4

/* TCTRL */
#define MAX96789_TCTRL_PWR0					  0x0008
#define   CMP_STATUS_MASK					  GENMASK(4, 0)
#define   CMP_STATUS_SHIFT					  0
#define   VDDBAD_STATUS_MASK					  GENMASK(7, 5)
#define   VDDBAD_STATUS_SHIFT					  5
#define MAX96789_TCTRL_PWR4					  0x000c
#define   WAKE_EN_A						  BIT(4)
#define   WAKE_EN_B						  BIT(5)
#define   DIS_LOCAL_WAKE					  BIT(6)
#define MAX96789_TCTRL_CTRL0					  0x0010
#define   LINK_CFG_MASK						  GENMASK(1, 0)
#define   LINK_CFG_SHIFT					  0
#define   LINK_CFG_DUAL_LINK					  0
#define   LINK_CFG_GMSL_A					  1
#define   LINK_CFG_GMSL_B					  2
#define   LINK_CFG_SPLITTER					  3
#define   REG_ENABLE						  BIT(2)
#define   SLEEP							  BIT(3)
#define   AUTO_LINK						  BIT(4)
#define   RESET_ONESHOT						  BIT(5)
#define   RESET_LINK						  BIT(6)
#define   RESET_ALL						  BIT(7)
#define MAX96789_TCTRL_CTRL1					  0x0011
#define   CXTP_A						  BIT(0)
#define   CXTP_B						  BIT(2)
#define MAX96789_TCTRL_CTRL2					  0x0012
#define   REG_MNL						  BIT(4)
#define MAX96789_TCTRL_CTRL3					  0x0013
#define   CMU_LOCKED						  BIT(1)
#define   ERROR							  BIT(2)
#define   LOCKED						  BIT(3)
#define MAX96789_TCTRL_INTR0					  0x0018
#define   DEC_ERR_THR_MASK					  GENMASK(2, 0)
#define   DEC_ERR_THR_SHIFT					  0
#define   AUTO_ERR_RST_EN					  BIT(3)
#define MAX96789_TCTRL_INTR1					  0x0019
#define   AUTO_CNT_RST_EN					  BIT(3)
#define   PKT_CNT_EXP_MASK					  GENMASK(7, 4)
#define   PKT_CNT_EXP_SHIFT					  4
#define MAX96789_TCTRL_INTR2					  0x001a
#define   DEC_ERR_OEN_A						  BIT(0)
#define   DEC_ERR_OEN_B						  BIT(1)
#define   IDLE_ERR_OEN						  BIT(2)
#define   LFLT_INT_OEN						  BIT(3)
#define   MEM_INT_ERR_OEN					  BIT(4)
#define   REM_ERR_OEN						  BIT(5)
#define MAX96789_TCTRL_INTR3					  0x001b
#define   DEC_ERR_FLAG_A					  BIT(0)
#define   DEC_ERR_FLAG_B					  BIT(1)
#define   IDLE_ERR_FLAG						  BIT(2)
#define   LFLT_INT						  BIT(3)
#define   MEM_INT_ERR_FLAG					  BIT(4)
#define   REM_ERR_FLAG						  BIT(5)
#define MAX96789_TCTRL_INTR4					  0x001c
#define   WM_ERR_OEN						  BIT(0)
#define   PKT_CNT_OEN						  BIT(1)
#define   RT_CNT_OEN						  BIT(2)
#define   MAX_RT_OEN						  BIT(3)
#define   HDCP_INT_OEN						  BIT(4)
#define   VDD_OV_OEN						  BIT(5)
#define   EOM_ERR_OEN_A						  BIT(6)
#define   EOM_ERR_OEN_B						  BIT(7)
#define MAX96789_TCTRL_INTR5					  0x001d
#define   WM_ERR_FLAG						  BIT(0)
#define   PKT_CNT_FLAG						  BIT(1)
#define   RT_CNT_FLAG						  BIT(2)
#define   MAX_RT_FLAG						  BIT(3)
#define   HDCP_INT_FLAG						  BIT(4)
#define   VDD_OV_FLAG						  BIT(5)
#define   EOM_ERR_FLAG_A					  BIT(6)
#define   EOM_ERR_FLAG_B					  BIT(7)
#define MAX96789_TCTRL_INTR6					  0x001e
#define   MIPI_ERR_OEN						  BIT(0)
#define   APRBS_A_ERR_OEN					  BIT(1)
#define   APRBS_B_ERR_OEN					  BIT(2)
#define   LOCK_A_OEN						  BIT(3)
#define   LOCK_B_OEN						  BIT(4)
#define   VDDBAD_INT_OEN					  BIT(5)
#define   VDDCMP_INT_OEN					  BIT(7)
#define MAX96789_TCTRL_INTR7					  0x001f
#define   MIPI_ERR_FLAG						  BIT(0)
#define   APRBS_A_ERR_FLAG					  BIT(1)
#define   APRBS_B_ERR_FLAG					  BIT(2)
#define   LOCK_A						  BIT(3)
#define   LOCK_B						  BIT(4)
#define   VDDBAD_INT_FLAG					  BIT(5)
#define   VDDCMP_INT_FLAG					  BIT(7)
#define MAX96789_TCTRL_INTR8					  0x0020
#define   ERR_TX_ID_MASK					  GENMASK(4, 0)
#define   ERR_TX_ID_SHIFT					  0
#define   ERR_TX_EN						  BIT(7)
#define MAX96789_TCTRL_INTR9					  0x0021
#define   ERR_RX_ID_MASK					  GENMASK(4, 0)
#define   ERR_RX_ID_SHIFT					  0
#define   ERR_RX_EN						  BIT(7)
#define MAX96789_TCTRL_DEC_ERR_A				  0x0022
#define MAX96789_TCTRL_DEC_ERR_B				  0x0023
#define MAX96789_TCTRL_IDLE_ERR_A				  0x0024
#define MAX96789_TCTRL_IDLE_ERR_B				  0x0025
#define MAX96789_TCTRL_PKT_CNT					  0x003d

/* GMSL */
#define MAX96789_GMSL_TX1					  0x0029
#define   ERRG_EN_A						  BIT(4)
#define   ERRG_EN_B						  BIT(5)
#define   LINK_PRBS_GEN						  BIT(7)
#define MAX96789_GMSL_TX2					  0x002a
#define   ERRG_PER						  BIT(0)
#define   ERRG_BURST_MASK					  GENMASK(3, 1)
#define   ERRG_BURST_SHIFT					  1
#define   ERRG_RATE_MASK					  GENMASK(5, 4)
#define   ERRG_RATE_SHIFT					  4
#define   ERRG_CNT_MASK						  GENMASK(7, 6)
#define   ERRG_CNT_SHIFT					  6
#define MAX96789_GMSL_TX3					  0x002b
#define   TIMEOUT_MASK						  GENMASK(2, 0)
#define   TIMEOUT_SHIFT						  0
#define MAX96789_GMSL_RX0					  0x002c
#define   PKT_CNT_SEL_MASK					  GENMASK(3, 0)
#define   PKT_CNT_SEL_SHIFT					  0
#define   PKT_CNT_LBW_MASK					  GENMASK(7, 6)
#define   PKT_CNT_LBW_SHIFT					  6
#define MAX96789_GMSL_GPIOA					  0x0030
#define   GPIO_FWD_CDLY_MASK					  GENMASK(5, 0)
#define   GPIO_FWD_CDLY_SHIFT					  0
#define   GPIO_RX_FAST_BIDIR_EN					  BIT(7)
#define MAX96789_GMSL_GPIOB					  0x0031
#define   GPIO_REV_CDLY_MASK					  GENMASK(5, 0)
#define   GPIO_REV_CDLY_SHIFT					  0
#define   GPIO_TX_WNDW_MASK					  GENMASK(7, 6)
#define   GPIO_TX_WNDW_SHIFT					  6

/* CC */
#define MAX96789_CC_I2C_0					  0x0040
#define   SLV_TO_MASK						  GENMASK(2, 0)
#define   SLV_TO_SHIFT						  0
#define   SLV_SH_MASK						  GENMASK(5, 4)
#define   SLV_SH_SHIFT						  4
#define MAX96789_CC_I2C_1					  0x0041
#define   MST_TO_MASK						  GENMASK(2, 0)
#define   MST_TO_SHIFT						  0
#define   MST_BT_MASK						  GENMASK(6, 4)
#define   MST_BT_SHIFT						  4
#define MAX96789_CC_I2C_2					  0x0042
#define   SRC_A_MASK						  GENMASK(7, 1)
#define   SRC_A_SHIFT						  1
#define MAX96789_CC_I2C_3					  0x0043
#define   DST_A_MASK						  GENMASK(7, 1)
#define   DST_A_SHIFT						  1
#define MAX96789_CC_I2C_4					  0x0044
#define   SRC_B_MASK						  GENMASK(7, 1)
#define   SRC_B_SHIFT						  1
#define MAX96789_CC_I2C_5					  0x0045
#define   DST_B_MASK						  GENMASK(7, 1)
#define   DST_B_SHIFT						  1
#define MAX96789_CC_I2C_6					  0x0046
#define   I2C_SRC_CNT_MASK					  GENMASK(2, 0)
#define   I2C_SRC_CNT_SHIFT					  0
#define   I2C_AUTO_CFG						  BIT(3)
#define MAX96789_CC_I2C_7					  0x0047
#define   REM_ACK_RECVED					  BIT(0)
#define   REM_ACK_ACKED						  BIT(1)
#define   UART_TX_OVERFLOW					  BIT(6)
#define   UART_RX_OVERFLOW					  BIT(7)
#define MAX96789_CC_UART_0					  0x0048
#define   BYPASS_EN						  BIT(0)
#define   BYPASS_TO_MASK					  GENMASK(2, 1)
#define   BYPASS_TO_SHIFT					  1
#define   BYPASS_DIS_PAR					  BIT(3)
#define   LOC_MS_EN						  BIT(4)
#define   REM_MS_EN						  BIT(5)
#define   ARB_TO_LEN_MASK					  GENMASK(7, 6)
#define   ARB_TO_LEN_SHIFT					  6
#define MAX96789_CC_BITLEN_LSB					  0x0049
#define MAX96789_CC_UART_2					  0x004a
#define   BITLEN_MSB_MASK					  GENMASK(5, 0)
#define   BITLEN_MSB_SHIFT					  0
#define   OUT_DELAY_MASK					  GENMASK(7, 6)
#define   OUT_DELAY_SHIFT					  6
#define MAX96789_CC_I2C_PT_0					  0x004c
#define   SLV_TO_PT_MASK					  GENMASK(2, 0)
#define   SLV_TO_PT_SHIFT					  0
#define   SLV_SH_PT_MASK					  GENMASK(5, 4)
#define   SLV_SH_PT_SHIFT					  4
#define MAX96789_CC_I2C_PT_1					  0x004d
#define   MST_TO_PT_MASK					  GENMASK(2, 0)
#define   MST_TO_PT_SHIFT					  0
#define   PT_SWAP						  BIT(3)
#define   MST_BT_PT_MASK					  GENMASK(6, 4)
#define   MST_BT_PT_SHIFT					  4
#define MAX96789_CC_I2C_PT_2					  0x004e
#define   REM_ACK_RECVED_1					  BIT(0)
#define   REM_ACK_ACKED_1					  BIT(1)
#define   XOVER_EN_1						  BIT(3)
#define   REM_ACK_RECVED_2					  BIT(4)
#define   REM_ACK_ACKED_2					  BIT(5)
#define   XOVER_EN_2						  BIT(7)
#define MAX96789_CC_UART_PT_0					  0x004f
#define   UART_TX_OVERFLOW_1					  BIT(0)
#define   UART_RX_OVERFLOW_1					  BIT(1)
#define   DIS_PAR_1						  BIT(2)
#define   BITLEN_MAN_CFG_1					  BIT(3)
#define   UART_TX_OVERFLOW_2					  BIT(4)
#define   UART_RX_OVERFLOW_2					  BIT(5)
#define   DIS_PAR_2						  BIT(6)
#define   BITLEN_MAN_CFG_2					  BIT(7)

/* CFGV_VIDEO: pipelines X -> 0, Y -> 1, Z -> 2, U -> 3 */
#define MAX96789_CFGV_VIDEO_TX0(p)				  (0x0050 + (p) * 4)
#define   VID_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   VID_PRIO_CFG_SHIFT					  0
#define   VID_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   VID_PRIO_VAL_SHIFT					  2
#define   VID_TX_CRC_EN						  BIT(7)
#define MAX96789_CFGV_VIDEO_TX1(p)				  (0x0051 + (p) * 4)
#define   VID_BW_VAL_MASK					  GENMASK(5, 0)
#define   VID_BW_VAL_SHIFT					  0
#define   VID_BW_MULT_MASK					  GENMASK(7, 6)
#define   VID_BW_MULT_SHIFT					  6
#define MAX96789_CFGV_VIDEO_TX3(p)				  (0x0053 + (p) * 4)
#define   VID_TX_STR_SEL_MASK					  GENMASK(1, 0)
#define   VID_TX_STR_SEL_SHIFT					  0
#define   VID_TX_SPLT_MASK_A					  BIT(4)
#define   VID_TX_SPLT_MASK_B					  BIT(5)

/* CFGA_AUDIO: channels X -> 0, Y -> 1 */
#define MAX96789_CFGA_AUDIO_TR0(ch)				  (0x0068 + (ch) * 8)
#define   AUD_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   AUD_PRIO_CFG_SHIFT					  0
#define   AUD_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   AUD_PRIO_VAL_SHIFT					  2
#define   AUD_RX_CRC_EN						  BIT(6)
#define   AUD_TX_CRC_EN						  BIT(7)
#define MAX96789_CFGA_AUDIO_TR1(ch)				  (0x0069 + (ch) * 8)
#define   AUD_BW_VAL_MASK					  GENMASK(5, 0)
#define   AUD_BW_VAL_SHIFT					  0
#define   AUD_BW_MULT_MASK					  GENMASK(7, 6)
#define   AUD_BW_MULT_SHIFT					  6
#define MAX96789_CFGA_AUDIO_TR3(ch)				  (0x006b + (ch) * 8)
#define   AUD_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   AUD_TX_SRC_ID_SHIFT					  0
#define   AUD_TX_SPLT_MASK_A					  BIT(4)
#define   AUD_TX_SPLT_MASK_B					  BIT(5)
#define MAX96789_CFGA_AUDIO_RX_SRC_SEL(ch)			  (0x006c + (ch) * 8)
#define MAX96789_CFGA_AUDIO_ARQ0(ch)				  (0x006d + (ch) * 8)
#define   AUD_DIS_DBL_ACK_RETX					  BIT(2)
#define   AUD_ARQ_EN						  BIT(3)
#define   AUD_ACK_SRC_ID					  BIT(4)
#define   AUD_MATCH_SRC_ID					  BIT(5)
#define   AUD_ACK_CNT						  BIT(6)
#define   AUD_ARQ_AUTO_CFG					  BIT(7)
#define MAX96789_CFGA_AUDIO_ARQ1(ch)				  (0x006e + (ch) * 8)
#define   AUD_RT_CNT_OEN					  BIT(0)
#define   AUD_MAX_RT_ERR_OEN					  BIT(1)
#define   AUD_MAX_RT_MASK					  GENMASK(6, 4)
#define   AUD_MAX_RT_SHIFT					  4
#define MAX96789_CFGA_AUDIO_ARQ2(ch)				  (0x006f + (ch) * 8)
#define   AUD_RT_CNT_MASK					  GENMASK(6, 0)
#define   AUD_RT_CNT_SHIFT					  0
#define   AUD_MAX_RT_ERR					  BIT(7)

/* CFGI_INFOFR */
#define MAX96789_CFGI_INFOFR_TR0				  0x0078
#define   INFOFR_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   INFOFR_PRIO_CFG_SHIFT					  0
#define   INFOFR_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   INFOFR_PRIO_VAL_SHIFT					  2
#define   INFOFR_RX_CRC_EN					  BIT(6)
#define   INFOFR_TX_CRC_EN					  BIT(7)
#define MAX96789_CFGI_INFOFR_TR1				  0x0079
#define   INFOFR_BW_VAL_MASK					  GENMASK(5, 0)
#define   INFOFR_BW_VAL_SHIFT					  0
#define   INFOFR_BW_MULT_MASK					  GENMASK(7, 6)
#define   INFOFR_BW_MULT_SHIFT					  6
#define MAX96789_CFGI_INFOFR_TR3				  0x007b
#define   INFOFR_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   INFOFR_TX_SRC_ID_SHIFT				  0
#define   INFOFR_TX_SPLT_MASK_A					  BIT(4)
#define   INFOFR_TX_SPLT_MASK_B					  BIT(5)
#define MAX96789_CFGI_INFOFR_RX_SRC_SEL				  0x007c

/* CFGL_SPI */
#define MAX96789_CFGL_SPI_TR0					  0x0080
#define   SPI_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   SPI_PRIO_CFG_SHIFT					  0
#define   SPI_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   SPI_PRIO_VAL_SHIFT					  2
#define   SPI_RX_CRC_EN						  BIT(6)
#define   SPI_TX_CRC_EN						  BIT(7)
#define MAX96789_CFGL_SPI_TR1					  0x0081
#define   SPI_BW_VAL_MASK					  GENMASK(5, 0)
#define   SPI_BW_VAL_SHIFT					  0
#define   SPI_BW_MULT_MASK					  GENMASK(7, 6)
#define   SPI_BW_MULT_SHIFT					  6
#define MAX96789_CFGL_SPI_TR3					  0x0083
#define   SPI_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   SPI_TX_SRC_ID_SHIFT					  0
#define   SPI_TX_SPLT_MASK_A					  BIT(4)
#define   SPI_TX_SPLT_MASK_B					  BIT(5)
#define MAX96789_CFGL_SPI_RX_SRC_SEL				  0x0084
#define MAX96789_CFGL_SPI_ARQ0					  0x0085
#define   SPI_DIS_DBL_ACK_RETX					  BIT(2)
#define   SPI_ARQ_EN						  BIT(3)
#define   SPI_ACK_SRC_ID					  BIT(4)
#define   SPI_MATCH_SRC_ID					  BIT(5)
#define   SPI_ACK_CNT						  BIT(6)
#define   SPI_ARQ_AUTO_CFG					  BIT(7)
#define MAX96789_CFGL_SPI_ARQ1					  0x0086
#define   SPI_RT_CNT_OEN					  BIT(0)
#define   SPI_MAX_RT_ERR_OEN					  BIT(1)
#define   SPI_MAX_RT_MASK					  GENMASK(6, 4)
#define   SPI_MAX_RT_SHIFT					  4
#define MAX96789_CFGL_SPI_ARQ2					  0x0087
#define   SPI_RT_CNT_MASK					  GENMASK(6, 0)
#define   SPI_RT_CNT_SHIFT					  0
#define   SPI_MAX_RT_ERR					  BIT(7)

/* CFGC_CC */
#define MAX96789_CFGC_CC_TR0					  0x0088
#define   CC_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   CC_PRIO_CFG_SHIFT					  0
#define   CC_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   CC_PRIO_VAL_SHIFT					  2
#define MAX96789_CFGC_CC_TR1					  0x0089
#define   CC_BW_VAL_MASK					  GENMASK(5, 0)
#define   CC_BW_VAL_SHIFT					  0
#define   CC_BW_MULT_MASK					  GENMASK(7, 6)
#define   CC_BW_MULT_SHIFT					  6
#define MAX96789_CFGC_CC_TR3					  0x008b
#define   CC_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   CC_TX_SRC_ID_SHIFT					  0
#define   CC_TX_SPLT_MASK_A					  BIT(4)
#define   CC_TX_SPLT_MASK_B					  BIT(5)
#define MAX96789_CFGC_CC_RX_SRC_SEL				  0x008c
#define MAX96789_CFGC_CC_ARQ0					  0x008d
#define   CC_DIS_DBL_ACK_RETX					  BIT(2)
#define   CC_ACK_SRC_ID						  BIT(4)
#define   CC_MATCH_SRC_ID					  BIT(5)
#define   CC_ACK_CNT						  BIT(6)
#define   CC_ARQ_AUTO_CFG					  BIT(7)
#define MAX96789_CFGC_CC_ARQ1					  0x008e
#define   CC_RT_CNT_OEN						  BIT(0)
#define   CC_MAX_RT_ERR_OEN					  BIT(1)
#define   CC_MAX_RT_MASK					  GENMASK(6, 4)
#define   CC_MAX_RT_SHIFT					  4
#define MAX96789_CFGC_CC_ARQ2					  0x008f
#define   CC_RT_CNT_MASK					  GENMASK(6, 0)
#define   CC_RT_CNT_SHIFT					  0
#define   CC_MAX_RT_ERR						  BIT(7)

/* CFGL_GPIO */
#define MAX96789_CFGL_GPIO_TR0					  0x0090
#define   GPIO_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   GPIO_PRIO_CFG_SHIFT					  0
#define   GPIO_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   GPIO_PRIO_VAL_SHIFT					  2
#define   GPIO_RX_CRC_EN					  BIT(6)
#define   GPIO_TX_CRC_EN					  BIT(7)
#define MAX96789_CFGL_GPIO_TR1					  0x0091
#define   GPIO_BW_VAL_MASK					  GENMASK(5, 0)
#define   GPIO_BW_VAL_SHIFT					  0
#define   GPIO_BW_MULT_MASK					  GENMASK(7, 6)
#define   GPIO_BW_MULT_SHIFT					  6
#define MAX96789_CFGL_GPIO_TR3					  0x0093
#define   GPIO_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   GPIO_TX_SRC_ID_SHIFT					  0
#define   GPIO_TX_SPLT_MASK_A					  BIT(4)
#define   GPIO_TX_SPLT_MASK_B					  BIT(5)
#define MAX96789_CFGL_GPIO_RX_SRC_SEL				  0x0094
#define MAX96789_CFGL_GPIO_ARQ0					  0x0095
#define   GPIO_DIS_DBL_ACK_RETX					  BIT(2)
#define   GPIO_ARQ_EN						  BIT(3)
#define   GPIO_ACK_SRC_ID					  BIT(4)
#define   GPIO_MATCH_SRC_ID					  BIT(5)
#define   GPIO_ACK_CNT						  BIT(6)
#define   GPIO_ARQ_AUTO_CFG					  BIT(7)
#define MAX96789_CFGL_GPIO_ARQ1					  0x0096
#define   GPIO_RT_CNT_OEN					  BIT(0)
#define   GPIO_MAX_RT_ERR_OEN					  BIT(1)
#define   GPIO_MAX_RT_MASK					  GENMASK(6, 4)
#define   GPIO_MAX_RT_SHIFT					  4
#define MAX96789_CFGL_GPIO_ARQ2					  0x0097
#define   GPIO_RT_CNT_MASK					  GENMASK(6, 0)
#define   GPIO_RT_CNT_SHIFT					  0
#define   GPIO_MAX_RT_ERR					  BIT(7)

/* CFGL_IIC: pass-through I2C: X -> 0, Y -> 1 */
#define MAX96789_CFGL_IIC_TR0(ch)				  (0x00a0 + (ch) * 8)
#define   PT_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   PT_PRIO_CFG_SHIFT					  0
#define   PT_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   PT_PRIO_VAL_SHIFT					  2
#define   PT_RX_CRC_EN						  BIT(6)
#define   PT_TX_CRC_EN						  BIT(7)
#define MAX96789_CFGL_IIC_TR1(ch)				  (0x00a1 + (ch) * 8)
#define   PT_BW_VAL_MASK					  GENMASK(5, 0)
#define   PT_BW_VAL_SHIFT					  0
#define   PT_BW_MULT_MASK					  GENMASK(7, 6)
#define   PT_BW_MULT_SHIFT					  6
#define MAX96789_CFGL_IIC_TR3(ch)				  (0x00a3 + (ch) * 8)
#define   PT_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   PT_TX_SRC_ID_SHIFT					  0
#define   PT_TX_SPLT_MASK_A					  BIT(4)
#define   PT_TX_SPLT_MASK_B					  BIT(5)
#define MAX96789_CFGL_IIC_RX_SRC_SEL(ch)			  (0x00a4 + (ch) * 8)
#define MAX96789_CFGL_IIC_ARQ0(ch)				  (0x00a5 + (ch) * 8)
#define   PT_DIS_DBL_ACK_RETX					  BIT(2)
#define   PT_ARQ_EN						  BIT(3)
#define   PT_ACK_SRC_ID						  BIT(4)
#define   PT_MATCH_SRC_ID					  BIT(5)
#define   PT_ACK_CNT						  BIT(6)
#define   PT_ARQ_AUTO_CFG					  BIT(7)
#define MAX96789_CFGL_IIC_ARQ1(ch)				  (0x00a6 + (ch) * 8)
#define   PT_RT_CNT_OEN						  BIT(0)
#define   PT_MAX_RT_ERR_OEN					  BIT(1)
#define   PT_MAX_RT_MASK					  GENMASK(6, 4)
#define   PT_MAX_RT_SHIFT					  4
#define MAX96789_CFGL_IIC_ARQ2(ch)				  (0x00a7 + (ch) * 8)
#define   PT_RT_CNT_MASK					  GENMASK(6, 0)
#define   PT_RT_CNT_SHIFT					  0
#define   PT_MAX_RT_ERR						  BIT(7)

/* VID_TX: pipelines X -> 0, Y -> 1, Z -> 2, U ->3 */
#define MAX96789_VID_TX_VIDEO_TX0(p)				  (0x0100 + (p) * 8)
#define   AUTO_BPP						  BIT(3)
#define   ENC_MODE_MASK						  GENMASK(5, 4)
#define   ENC_MODE_SHIFT					  4
#define   LINE_CRC_EN						  BIT(6)
#define   LINE_CRC_SEL						  BIT(7)
#define MAX96789_VID_TX_VIDEO_TX1(p)				  (0x0101 + (p) * 8)
#define   BPP_MASK						  GENMASK(5, 0)
#define   BPP_SHIFT						  0
#define MAX96789_VID_TX_VIDEO_TX2(p)				  (0x0102 + (p) * 8)
#define   LIM_HEART						  BIT(2)
#define   FIFO_WARN						  BIT(4)
#define   OVERFLOW						  BIT(5)
#define   DRIFT_ERR						  BIT(6)
#define   PCLKDET						  BIT(7)
#define MAX96789_VID_TX_VIDEO_TX6(p)				  (0x0106 + (p) * 8)
#define   MASK_VIDEO_DE						  BIT(6)

/* AUD_TX: A -> 0, B -> 1 */
#define MAX96789_AUD_TX_AUDIO_TX0(ch)				  (0x0120 + (ch) * 0x10)
#define   AUD_SINK_SRC						  BIT(0)
#define   FORCE_AUD						  BIT(1)
#define   INV_WS						  BIT(2)
#define   INV_SCK						  BIT(3)
#define   I2S_CFG_MASK						  GENMASK(5, 4)
#define   I2S_CFG_SHIFT						  4
#define MAX96789_AUD_TX_AUDIO_TX1(ch)				  (0x0121 + (ch) * 0x10)
#define   AUD_INF_PER						  BIT(2)
#define   AUD_DRIFT_DET_EN					  BIT(3)
#define   AUD_STR_TX_MASK					  GENMASK(5, 4)
#define   AUD_STR_TX_SHIFT					  4
#define   AUD_PRIO_MASK						  GENMASK(7, 6)
#define   AUD_PRIO_SHIFT					  6
#define MAX96789_AUD_TX_AUDIO_TX5(ch)				  (0x0125 + (ch) * 0x10)
#define   ACLKDET						  BIT(4)
#define   AUD_OVERFLOW						  BIT(5)
#define   AUD_FIFO_WARN						  BIT(6)
#define   AUD_DRIFT_ERR						  BIT(7)
#define MAX96789_AUD_TX_AUDIO_TX7(ch)				  0x0127
#define   PRBSEN_AUD						  BIT(6)
#define   PRBS_SEL						  BIT(7)
#define MAX96789_AUD_TX_AUDIO_TX8(ch)				  (0x0128 + (ch) * 0x10)
#define   PRBS_WS_GEN						  BIT(6)
#define   PRBS_WS_LEN						  BIT(7)

/* AUD_RX: A -> 0, B -> 1 */
#define MAX96789_AUD_RX_AUDIO_RX1(ch)				  (0x0140 + (ch) * 0x11)
#define   AUD_EN_RX						  BIT(0)
#define   INV_WS_RX						  BIT(2)
#define   INV_SCK_RX						  BIT(3)
#define   AUD_RX_SINK_SRC					  BIT(7)
#define MAX96789_AUD_RX_APRBS_ERR(ch)				  (0x0143 + (ch) * 0x11)
#define MAX96789_AUD_RX_AUDIO_RX7(ch)				  (0x0146 + (ch) * 0x11)
#define   AUD_STRM_MASK						  GENMASK(3, 2)
#define   AUD_STRM_SHIFT					  2
#define   APRBS_CHK_EN						  BIT(4)
#define MAX96789_AUD_RX_AUDIO_RX9(ch)				  (0x0148 + (ch) * 0x11)
#define   APRBS_VALID						  BIT(4)
#define   AUD_PKT_DET						  BIT(5)
#define   AUD_LOCK						  BIT(6)
#define   AUD_BLK_LEN_ERR					  BIT(7)

/* SPI */
#define MAX96789_SPI_0						  0x0170
#define   SPI_EN						  BIT(0)
#define   MST_SLVN						  BIT(1)
#define   SPI_CC_EN						  BIT(2)
#define   SPI_IGNR_ID						  BIT(3)
#define   SPI_CC_TRG_ID_MASK					  GENMASK(5, 4)
#define   SPI_CC_TRG_ID_SHIFT					  4
#define   SPI_LOC_ID_MASK					  GENMASK(7, 6)
#define   SPI_LOC_ID_SHIFT					  6
#define MAX96789_SPI_1						  0x0171
#define   SPI_BASE_PRIO_MASK					  GENMASK(1, 0)
#define   SPI_BASE_PRIO_SHIFT					  0
#define   SPI_LOC_N_MASK					  GENMASK(7, 2)
#define   SPI_LOC_N_SHIFT					  2
#define MAX96789_SPI_2						  0x0172
#define   SPIM_SS1_ACT_H					  BIT(0)
#define   SPIM_SS2_ACT_H					  BIT(1)
#define   SPI_MOD3						  BIT(2)
#define   SPI_MOD3_F						  BIT(3)
#define   FULL_SCK_SETUP					  BIT(4)
#define   REQ_HOLD_OFF_MASK					  GENMASK(7, 5)
#define   REQ_HOLD_OFF_SHIFT					  5
#define MAX96789_SPI_SPIM_SS_DLY_CLKS				  0x0173
#define MAX96789_SPI_SPIM_SCK_LO_CLKS				  0x0174
#define MAX96789_SPI_SPIM_SCK_HI_CLKS				  0x0175
#define MAX96789_SPI_6						  0x0176
#define   RWN_IO_EN						  BIT(0)
#define   BNE_IO_EN						  BIT(1)
#define   SS_IO_EN_1						  BIT(2)
#define   SS_IO_EN_2						  BIT(3)
#define   SPIS_RWN						  BIT(4)
#define   BNE							  BIT(5)
#define MAX96789_SPI_7						  0x0177
#define   SPIS_BYTE_CNT_MASK					  GENMASK(4, 0)
#define   SPIS_BYTE_CNT_SHIFT					  0
#define   SPI_TX_OVRFLW						  BIT(6)
#define   SPI_RX_OVRFLW						  BIT(7)
#define MAX96789_SPI_REQ_HOLD_OFF_TO				  0x0178

/* WM */
#define MAX96789_WM_0						  0x0190
#define   WM_EN							  BIT(0)
#define   WM_DET_MASK						  GENMASK(3, 2)
#define   WM_DET_SHIFT						  2
#define   WM_MODE_MASK						  GENMASK(6, 4)
#define   WM_MODE_SHIFT						  4
#define   WM_LEN						  BIT(7)
#define MAX96789_WM_2						  0x0192
#define   WM_NPFILT_MASK					  GENMASK(1, 0)
#define   WM_NPFILT_SHIFT					  0
#define   VSYNCPOL						  BIT(2)
#define   HSYNCPOL						  BIT(3)
#define MAX96789_WM_3						  0x0193
#define   WM_TH_MASK						  GENMASK(6, 0)
#define   WM_TH_SHIFT						  0
#define MAX96789_WM_4						  0x0194
#define   WM_MASKMODE_MASK					  GENMASK(1, 0)
#define   WM_MASKMODE_SHIFT					  0
#define   WM_COLORADJ						  BIT(3)
#define MAX96789_WM_5						  0x0195
#define   WM_ERROR						  BIT(0)
#define   WM_DETOUT						  BIT(1)
#define MAX96789_WM_TIMER					  0x0196
#define MAX96789_WM_WREN_L					  0x01ae
#define MAX96789_WM_WREN_H					  0x01af

/* VTX: pipelines X -> 0, Y -> 1, Z -> 2, U -> 3 */
#define MAX96789_VTX_CROSS_0(p)					  (0x01b0 + (p) * 0x43)
#define   CROSS0_MASK						  GENMASK(4, 0)
#define   CROSS0_SHIFT						  0
#define   CROSS0_F						  BIT(5)
#define   CROSS0_I						  BIT(6)
#define MAX96789_VTX_CROSS_1(p)					  (0x01b1 + (p) * 0x43)
#define   CROSS1_MASK						  GENMASK(4, 0)
#define   CROSS1_SHIFT						  0
#define   CROSS1_F						  BIT(5)
#define   CROSS1_I						  BIT(6)
#define MAX96789_VTX_CROSS_2(p)					  (0x01b2 + (p) * 0x43)
#define   CROSS2_MASK						  GENMASK(4, 0)
#define   CROSS2_SHIFT						  0
#define   CROSS2_F						  BIT(5)
#define   CROSS2_I						  BIT(6)
#define MAX96789_VTX_CROSS_3(p)					  (0x01b3 + (p) * 0x43)
#define   CROSS3_MASK						  GENMASK(4, 0)
#define   CROSS3_SHIFT						  0
#define   CROSS3_F						  BIT(5)
#define   CROSS3_I						  BIT(6)
#define MAX96789_VTX_CROSS_4(p)					  (0x01b4 + (p) * 0x43)
#define   CROSS4_MASK						  GENMASK(4, 0)
#define   CROSS4_SHIFT						  0
#define   CROSS4_F						  BIT(5)
#define   CROSS4_I						  BIT(6)
#define MAX96789_VTX_CROSS_5(p)					  (0x01b5 + (p) * 0x43)
#define   CROSS5_MASK						  GENMASK(4, 0)
#define   CROSS5_SHIFT						  0
#define   CROSS5_F						  BIT(5)
#define   CROSS5_I						  BIT(6)
#define MAX96789_VTX_CROSS_6(p)					  (0x01b6 + (p) * 0x43)
#define   CROSS6_MASK						  GENMASK(4, 0)
#define   CROSS6_SHIFT						  0
#define   CROSS6_F						  BIT(5)
#define   CROSS6_I						  BIT(6)
#define MAX96789_VTX_CROSS_7(p)					  (0x01b7 + (p) * 0x43)
#define   CROSS7_MASK						  GENMASK(4, 0)
#define   CROSS7_SHIFT						  0
#define   CROSS7_F						  BIT(5)
#define   CROSS7_I						  BIT(6)
#define MAX96789_VTX_CROSS_8(p)					  (0x01b8 + (p) * 0x43)
#define   CROSS8_MASK						  GENMASK(4, 0)
#define   CROSS8_SHIFT						  0
#define   CROSS8_F						  BIT(5)
#define   CROSS8_I						  BIT(6)
#define MAX96789_VTX_CROSS_9(p)					  (0x01b9 + (p) * 0x43)
#define   CROSS9_MASK						  GENMASK(4, 0)
#define   CROSS9_SHIFT						  0
#define   CROSS9_F						  BIT(5)
#define   CROSS9_I						  BIT(6)
#define MAX96789_VTX_CROSS_10(p)				  (0x01ba + (p) * 0x43)
#define   CROSS10_MASK						  GENMASK(4, 0)
#define   CROSS10_SHIFT						  0
#define   CROSS10_F						  BIT(5)
#define   CROSS10_I						  BIT(6)
#define MAX96789_VTX_CROSS_11(p)				  (0x01bb + (p) * 0x43)
#define   CROSS11_MASK						  GENMASK(4, 0)
#define   CROSS11_SHIFT						  0
#define   CROSS11_F						  BIT(5)
#define   CROSS11_I						  BIT(6)
#define MAX96789_VTX_CROSS_12(p)				  (0x01bc + (p) * 0x43)
#define   CROSS12_MASK						  GENMASK(4, 0)
#define   CROSS12_SHIFT						  0
#define   CROSS12_F						  BIT(5)
#define   CROSS12_I						  BIT(6)
#define MAX96789_VTX_CROSS_13(p)				  (0x01bd + (p) * 0x43)
#define   CROSS13_MASK						  GENMASK(4, 0)
#define   CROSS13_SHIFT						  0
#define   CROSS13_F						  BIT(5)
#define   CROSS13_I						  BIT(6)
#define MAX96789_VTX_CROSS_14(p)				  (0x01be + (p) * 0x43)
#define   CROSS14_MASK						  GENMASK(4, 0)
#define   CROSS14_SHIFT						  0
#define   CROSS14_F						  BIT(5)
#define   CROSS14_I						  BIT(6)
#define MAX96789_VTX_CROSS_15(p)				  (0x01bf + (p) * 0x43)
#define   CROSS15_MASK						  GENMASK(4, 0)
#define   CROSS15_SHIFT						  0
#define   CROSS15_F						  BIT(5)
#define   CROSS15_I						  BIT(6)
#define MAX96789_VTX_CROSS_16(p)				  (0x01c0 + (p) * 0x43)
#define   CROSS16_MASK						  GENMASK(4, 0)
#define   CROSS16_SHIFT						  0
#define   CROSS16_F						  BIT(5)
#define   CROSS16_I						  BIT(6)
#define MAX96789_VTX_CROSS_17(p)				  (0x01c1 + (p) * 0x43)
#define   CROSS17_MASK						  GENMASK(4, 0)
#define   CROSS17_SHIFT						  0
#define   CROSS17_F						  BIT(5)
#define   CROSS17_I						  BIT(6)
#define MAX96789_VTX_CROSS_18(p)				  (0x01c2 + (p) * 0x43)
#define   CROSS18_MASK						  GENMASK(4, 0)
#define   CROSS18_SHIFT						  0
#define   CROSS18_F						  BIT(5)
#define   CROSS18_I						  BIT(6)
#define MAX96789_VTX_CROSS_19(p)				  (0x01c3 + (p) * 0x43)
#define   CROSS19_MASK						  GENMASK(4, 0)
#define   CROSS19_SHIFT						  0
#define   CROSS19_F						  BIT(5)
#define   CROSS19_I						  BIT(6)
#define MAX96789_VTX_CROSS_20(p)				  (0x01c4 + (p) * 0x43)
#define   CROSS20_MASK						  GENMASK(4, 0)
#define   CROSS20_SHIFT						  0
#define   CROSS20_F						  BIT(5)
#define   CROSS20_I						  BIT(6)
#define MAX96789_VTX_CROSS_21(p)				  (0x01c5 + (p) * 0x43)
#define   CROSS21_MASK						  GENMASK(4, 0)
#define   CROSS21_SHIFT						  0
#define   CROSS21_F						  BIT(5)
#define   CROSS21_I						  BIT(6)
#define MAX96789_VTX_CROSS_22(p)				  (0x01c6 + (p) * 0x43)
#define   CROSS22_MASK						  GENMASK(4, 0)
#define   CROSS22_SHIFT						  0
#define   CROSS22_F						  BIT(5)
#define   CROSS22_I						  BIT(6)
#define MAX96789_VTX_CROSS_23(p)				  (0x01c7 + (p) * 0x43)
#define   CROSS23_MASK						  GENMASK(4, 0)
#define   CROSS23_SHIFT						  0
#define   CROSS23_F						  BIT(5)
#define   CROSS23_I						  BIT(6)
#define MAX96789_VTX_VTX0(p)					  (0x01c8 + (p) * 0x43)
#define   VTG_MODE_MASK						  GENMASK(1, 0)
#define   VTG_MODE_SHIFT					  0
#define   DE_INV						  BIT(2)
#define   HS_INV						  BIT(3)
#define   VS_INV						  BIT(4)
#define   GEN_DE						  BIT(5)
#define   GEN_HS						  BIT(6)
#define   GEN_VS						  BIT(7)
#define MAX96789_VTX_VTX1(p)					  (0x01c9 + (p) * 0x43)
#define   VS_TRIG						  BIT(0)
#define   PCLKDET_VTX						  BIT(5)
#define MAX96789_VTX_VS_DLY_2(p)				  (0x01ca + (p) * 0x43)
#define MAX96789_VTX_VS_DLY_1(p)				  (0x01cb + (p) * 0x43)
#define MAX96789_VTX_VS_DLY_0(p)				  (0x01cc + (p) * 0x43)
#define MAX96789_VTX_VS_HIGH_2(p)				  (0x01cd + (p) * 0x43)
#define MAX96789_VTX_VS_HIGH_1(p)				  (0x01ce + (p) * 0x43)
#define MAX96789_VTX_VS_HIGH_0(p)				  (0x01cf + (p) * 0x43)
#define MAX96789_VTX_VS_LOW_2(p)				  (0x01d0 + (p) * 0x43)
#define MAX96789_VTX_VS_LOW_1(p)				  (0x01d1 + (p) * 0x43)
#define MAX96789_VTX_VS_LOW_0(p)				  (0x01d2 + (p) * 0x43)
#define MAX96789_VTX_V2H_2(p)					  (0x01d3 + (p) * 0x43)
#define MAX96789_VTX_V2H_1(p)					  (0x01d4 + (p) * 0x43)
#define MAX96789_VTX_V2H_0(p)					  (0x01d5 + (p) * 0x43)
#define MAX96789_VTX_HS_HIGH_1(p)				  (0x01d6 + (p) * 0x43)
#define MAX96789_VTX_HS_HIGH_0(p)				  (0x01d7 + (p) * 0x43)
#define MAX96789_VTX_HS_LOW_1(p)				  (0x01d8 + (p) * 0x43)
#define MAX96789_VTX_HS_LOW_0(p)				  (0x01d9 + (p) * 0x43)
#define MAX96789_VTX_HS_CNT_1(p)				  (0x01da + (p) * 0x43)
#define MAX96789_VTX_HS_CNT_0(p)				  (0x01db + (p) * 0x43)
#define MAX96789_VTX_V2D_2(p)					  (0x01dc + (p) * 0x43)
#define MAX96789_VTX_V2D_1(p)					  (0x01dd + (p) * 0x43)
#define MAX96789_VTX_V2D_0(p)					  (0x01de + (p) * 0x43)
#define MAX96789_VTX_DE_HIGH_1(p)				  (0x01df + (p) * 0x43)
#define MAX96789_VTX_DE_HIGH_0(p)				  (0x01e0 + (p) * 0x43)
#define MAX96789_VTX_DE_LOW_1(p)				  (0x01e1 + (p) * 0x43)
#define MAX96789_VTX_DE_LOW_0(p)				  (0x01e2 + (p) * 0x43)
#define MAX96789_VTX_DE_CNT_1(p)				  (0x01e3 + (p) * 0x43)
#define MAX96789_VTX_DE_CNT_0(p)				  (0x01e4 + (p) * 0x43)
#define MAX96789_VTX_VTX29(p)					  (0x01e5 + (p) * 0x43)
#define   PATGEN_MODE_MASK					  GENMASK(1, 0)
#define   PATGEN_MODE_SHIFT					  0
#define   GRAD_MODE						  BIT(2)
#define   VID_PRBS_EN						  BIT(7)
#define MAX96789_VTX_GRAD_INC(p)				  (0x01e6 + (p) * 0x43)
#define MAX96789_VTX_CHKR_A_L(p)				  (0x01e7 + (p) * 0x43)
#define MAX96789_VTX_CHKR_A_M(p)				  (0x01e8 + (p) * 0x43)
#define MAX96789_VTX_CHKR_A_H(p)				  (0x01e9 + (p) * 0x43)
#define MAX96789_VTX_CHKR_B_L(p)				  (0x01ea + (p) * 0x43)
#define MAX96789_VTX_CHKR_B_M(p)				  (0x01eb + (p) * 0x43)
#define MAX96789_VTX_CHKR_B_H(p)				  (0x01ec + (p) * 0x43)
#define MAX96789_VTX_CHKR_RPT_A(p)				  (0x01ed + (p) * 0x43)
#define MAX96789_VTX_CHKR_RPT_B(p)				  (0x01ee + (p) * 0x43)
#define MAX96789_VTX_CHKR_ALT(p)				  (0x01ef + (p) * 0x43)
#define MAX96789_VTX_VTX40(p)					  (0x01f0 + (p) * 0x43)
#define   CROSSHS_MASK						  GENMASK(4, 0)
#define   CROSSHS_SHIFT						  0
#define   CROSSHS_F						  BIT(5)
#define   CROSSHS_I						  BIT(6)
#define   DIS_COLOR_CROSSBAR					  BIT(7)
#define MAX96789_VTX_VTX41(p)					  (0x01f1 + (p) * 0x43)
#define   CROSSVS_MASK						  GENMASK(4, 0)
#define   CROSSVS_SHIFT						  0
#define   CROSSVS_F						  BIT(5)
#define   CROSSVS_I						  BIT(6)
#define MAX96789_VTX_VTX42(p)					  (0x01f2 + (p) * 0x43)
#define   CROSSDE_MASK						  GENMASK(4, 0)
#define   CROSSDE_SHIFT						  0
#define   CROSSDE_F						  BIT(5)
#define   CROSSDE_I						  BIT(6)

/* GPIO: 0 <= n < 21 */
#define MAX96789_GPIO_A(n)					  (0x02be + (n) * 3)
#define   GPIO_OUT_DIS						  BIT(0)
#define   GPIO_TX_EN						  BIT(1)
#define   GPIO_RX_EN						  BIT(2)
#define   GPIO_IN						  BIT(3)
#define   GPIO_OUT						  BIT(4)
#define   TX_COMP_EN						  BIT(5)
#define   RES_CFG						  BIT(7)
#define MAX96789_GPIO_B(n)					  (0x02bf + (n) * 3)
#define   GPIO_TX_ID_MASK					  GENMASK(4, 0)
#define   GPIO_TX_ID_SHIFT					  0
#define   OUT_TYPE						  BIT(5)
#define   PULL_UPDN_SEL_MASK					  GENMASK(7, 6)
#define   PULL_UPDN_SEL_SHIFT					  6
#define MAX96789_GPIO_C(n)					  (0x02c0 + (n) * 3)
#define   GPIO_RX_ID_MASK					  GENMASK(4, 0)
#define   GPIO_RX_ID_SHIFT					  0
#define   GPIO_IO_RX_EN						  BIT(5)
#define   OVR_RES_CFG						  BIT(7)

/* CMU */
#define MAX96789_CMU_CMU4					  0x0304
#define   D_SPEED_MASK						  GENMASK(1, 0)
#define   D_SPEED_SHIFT						  0
#define   C_SPEED_MASK						  GENMASK(3, 2)
#define   C_SPEED_SHIFT						  2
#define   B_SPEED_MASK						  GENMASK(5, 4)
#define   B_SPEED_SHIFT						  4
#define   A_SPEED_MASK						  GENMASK(7, 6)
#define   A_SPEED_SHIFT						  6

/* FRONTTOP */
#define MAX96789_FRONTTOP_PORT_SEL				  0x0308
#define   CLK_SELX						  BIT(0)
#define   CLK_SELY						  BIT(1)
#define   CLK_SELZ						  BIT(2)
#define   CLK_SELU						  BIT(3)
#define   START_PORTA						  BIT(4)
#define   START_PORTB						  BIT(5)
#define   ENABLE_LINE_INFO					  BIT(6)
#define MAX96789_FRONTTOP_VIDEO_PIPE_START			  0x0311
#define   START_PORTAX						  BIT(0)
#define   START_PORTAY						  BIT(1)
#define   START_PORTAZ						  BIT(2)
#define   START_PORTAU						  BIT(3)
#define   START_PORTBX						  BIT(4)
#define   START_PORTBY						  BIT(5)
#define   START_PORTBZ						  BIT(6)
#define   START_PORTBU						  BIT(7)
/* pipe: X -> 0, Y -> 1, Z -> 2, U -> 3 */
#define MAX96789_FRONTTOP_PIPE_SW_OVR(p)			  (0x031c + (p) * 0x01)
#define   SOFT_BPP_MASK						  GENMASK(4, 0)
#define   SOFT_BPP_SHIFT					  0
#define   SOFT_BPP_EN						  BIT(5)
#define   SOFT_DT_EN						  BIT(7)
#define MAX96789_FRONTTOP_PIPE_SW_OVR_VAL(p)			  (0x0321 + (p) * 0x01)
#define   SOFT_DT_MASK						  GENMASK(5, 0)
#define   SOFT_DT_SHIFT						  0
#define MAX96789_FRONTTOP_GANGED_CFG				  0x0325
#define   GANGED_MODE_EN					  BIT(0)
#define   GANGED_FIRST_A_OR_B					  BIT(1)
#define   GANGED_DE_OUTPUT_POL					  BIT(2)
#define   GANGED_DE_INPUT_POL					  BIT(3)
#define   GANGED_HS_DE_SWAP					  BIT(4)
#define   GANGED_EMPTY						  BIT(5)
#define   GANGED_FULL						  BIT(6)
#define   FORCE_START_MIPI_FRONTTOP				  BIT(7)
#define MAX96789_FRONTTOP_CROSS_CFG				  0x0326
#define   CROSS_X_MASK						  GENMASK(1, 0)
#define   CROSS_X_SHIFT						  0
#define   CROSS_Y_MASK						  GENMASK(3, 2)
#define   CROSS_Y_SHIFT						  2
#define   CROSS_Z_MASK						  GENMASK(5, 4)
#define   CROSS_Z_SHIFT						  4
#define   CROSS_U_MASK						  GENMASK(7, 6)
#define   CROSS_U_SHIFT						  6

/* DUALVIEW: A -> 0, B -> 1 */
#define MAX96789_DUALVIEW_DV0(d)				  (0x032a + (d) * 3)
#define   DV_EN							  BIT(0)
#define   DV_SPL						  BIT(1)
#define   DV_CONV						  BIT(2)
#define   LINE_ALT						  BIT(5)
#define   DV_SWP_AB						  BIT(6)
#define   DV_LOCK						  BIT(7)
#define MAX96789_DUALVIEW_DV_PPL_L(d)				  (0x032b + (d) * 3)
#define MAX96789_DUALVIEW_DV2(d)				  (0x032c + (d) * 3)
#define   DV_PPL_H_MASK						  GENMASK(4, 0)
#define   DV_PPL_H_SHIFT					  0
#define   DV_MEM_CRC_ERR					  BIT(5)

/* MIPI_RX */
#define MAX96789_MIPI_RX_PHY_CFG				  0x0330
#define   PHY_CONFIG_MASK					  GENMASK(2, 0)
#define   PHY_CONFIG_SHIFT					  0
#define   PHY_CONFIG_ONLY_PORT_A_EN				  (4 << PHY_CONFIG_SHIFT)
#define   PHY_CONFIG_ONLY_PORT_B_EN				  (5 << PHY_CONFIG_SHIFT)
#define   PHY_CONFIG_BOTH_PORTS_EN				  (6 << PHY_CONFIG_SHIFT)
#define   MIPI_RX_RESET						  BIT(3)
#define MAX96789_MIPI_RX_LANES_NUM				  0x0331
#define   CTRL0_NUM_LANES_MASK					  GENMASK(1, 0)
#define   CTRL0_NUM_LANES_SHIFT					  0
#define   CTRL1_NUM_LANES_MASK					  GENMASK(5, 4)
#define   CTRL1_NUM_LANES_SHIFT					  4
#define MAX96789_MIPI_RX_LANE_MAP(p)				  (0x0332 + (p) * 0x01)
#define   PHY_0_2_LANE_MAP_MASK					  GENMASK(3, 0)
#define   PHY_0_2_LANE_MAP_SHIFT				  0
#define   PHY_1_3_LANE_MAP_MASK					  GENMASK(7, 4)
#define   PHY_1_3_LANE_MAP_SHIFT				  4
#define MAX96789_MIPI_RX_POL_MAP(p)				  (0x0334 + (p) * 0x01)
#define   PHY_0_2_POL_MAP_MASK					  GENMASK(2, 0)
#define   PHY_0_2_POL_MAP_SHIFT					  0
#define   PHY_1_3_POL_MAP_MASK					  GENMASK(6, 4)
#define   PHY_1_3_POL_MAP_SHIFT					  4
#define MAX96789_MIPI_RX_TIMINGS				  0x0338
#define   T_CLK_SETTLE_MASK					  GENMASK(1, 0)
#define   T_CLK_SETTLE_SHIFT					  0
#define   T_CLK_MISS_MASK					  GENMASK(3, 2)
#define   T_CLK_MISS_SHIFT					  2
#define   T_HS_SETTLE_MASK					  GENMASK(5, 4)
#define   T_HS_SETTLE_SHIFT					  4
#define MAX96789_MIPI_RX_PHY0_LP_ERR				  0x0339
#define   PHY0_LP_ERR_MASK					  GENMASK(4, 0)
#define   PHY0_LP_ERR_SHIFT					  0
#define MAX96789_MIPI_RX_PHY0_HS_ERR				  0x033a
#define MAX96789_MIPI_RX_PHY1_LP_ERR				  0x033b
#define   PHY1_LP_ERR_MASK					  GENMASK(4, 0)
#define   PHY1_LP_ERR_SHIFT					  0
#define MAX96789_MIPI_RX_PHY1_HS_ERR				  0x033c
#define MAX96789_MIPI_RX_PHY2_LP_ERR				  0x033d
#define   PHY2_LP_ERR_MASK					  GENMASK(4, 0)
#define   PHY2_LP_ERR_SHIFT					  0
#define MAX96789_MIPI_RX_PHY2_HS_ERR				  0x033e
#define MAX96789_MIPI_RX_PHY3_LP_ERR				  0x033f
#define   PHY3_LP_ERR_MASK					  GENMASK(4, 0)
#define   PHY3_LP_ERR_SHIFT					  0
#define MAX96789_MIPI_RX_PHY3_HS_ERR				  0x0340

/* MIPI_DSI: port A -> 0, B -> 1 */
#define MAX96789_MIPI_DSI_CFG(p)				  (0x0380 + (p) * 0x10)
#define   AUTODETECT_LINE_LENGTH				  BIT(3)
#define   VSYNC_POLARITY					  BIT(4)
#define   HSYNC_POLARITY					  BIT(5)
#define   VIDEO_MODE_MASK					  GENMASK(7, 6)
#define   VIDEO_MODE_SHIFT					  6
#define MAX96789_MIPI_DSI_CFG_LINE_LENGTH_L(p)			  (0x0381 + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_LINE_LENGTH_H(p)			  (0x0382 + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_HSYNC_WIDTH_L(p)			  (0x0385 + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_VSYNC_WIDTH_L(p)			  (0x0386 + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_SYNC(p)				  (0x0387 + (p) * 0x10)
#define   HSYNC_WIDTH_H_MASK					  GENMASK(3, 0)
#define   HSYNC_WIDTH_H_SHIFT					  0
#define   VSYNC_WIDTH_H_MASK					  GENMASK(7, 4)
#define   VSYNC_WIDTH_H_SHIFT					  4
#define MAX96789_MIPI_DSI_CFG_VC(p)				  (0x0388 + (p) * 0x10)
#define   USE_NULL_PKT_BLLP					  BIT(2)
#define   VC_MASK						  GENMASK(7, 6)
#define   VC_SHIFT						  6
#define MAX96789_MIPI_DSI_CFG_ERR_CHK(p)			  (0x0389 + (p) * 0x10)
#define   DSI_REPRESSED_AFTER_UNRECOVERABLE_ECC_ERROR		  BIT(0)
#define   CRC_ERROR_ASSERTS_INVALID_TX_LENGTH_ERROR		  BIT(2)
#define   DISABLE_EOTP						  BIT(4)
#define   DISABLE_VC_CHECK					  BIT(5)
#define MAX96789_MIPI_DSI_CFG_HRX_TO_COUNT_L(p)			  (0x038a + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_HRX_TO_COUNT_M(p)			  (0x038b + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_HRX_TO_COUNT_H(p)			  (0x038c + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_LTX_P_TO_COUNT_L(p)		  (0x038d + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_LTX_P_TO_COUNT_M(p)		  (0x038e + (p) * 0x10)
#define MAX96789_MIPI_DSI_CFG_LTX_P_TO_COUNT_H(p)		  (0x038f + (p) * 0x10)

#define MAX96789_MIPI_DSI_CTRL_STATUS(p)			  (0x03a0 + (p) * 0x02)

#define MAX96789_MIPI_DSI_SKEW(p)				  (0x03a4 + (p) * 0x0c)
#define   DESKEW_EN						  BIT(0)
#define   DE_SKEW_SEL						  BIT(1)
#define   SKEW_FIFO_SIZE_MASK					  GENMASK(7, 6)
#define   SKEW_FIFO_SIZE_SHIFT					  6
#define MAX96789_MIPI_DSI_VFP_L(p)				  (0x03a5 + (p) * 0x0c)
#define MAX96789_MIPI_DSI_VP(p)					  (0x03a6 + (p) * 0x0c)
#define   VFP_H_MASK						  GENMASK(3, 0)
#define   VFP_H_SHIFT						  0
#define   VBP_L_MASK						  GENMASK(7, 4)
#define   VBP_L_SHIFT						  4
#define MAX96789_MIPI_DSI_VBP_H(p)				  (0x03a7 + (p) * 0x0c)
#define MAX96789_MIPI_DSI_VACTIVE_L(p)				  (0x03a8 + (p) * 0x0c)
#define MAX96789_MIPI_DSI_VACTIVE_H(p)				  (0x03a9 + (p) * 0x0c)
#define   VACTIVE_H_MASK					  GENMASK(3, 0)
#define   VACTIVE_H_SHIFT					  0
#define MAX96789_MIPI_DSI_HFP_L(p)				  (0x03aa + (p) * 0x0c)
#define MAX96789_MIPI_DSI_HP(p)					  (0x03ab + (p) * 0x0c)
#define   HFP_H_MASK						  GENMASK(3, 0)
#define   HFP_H_SHIFT						  0
#define   HBP_L_MASK						  GENMASK(7, 4)
#define   HBP_L_SHIFT						  4
#define MAX96789_MIPI_DSI_HBP_H(p)				  (0x03ac + (p) * 0x0c)
#define MAX96789_MIPI_DSI_HACTIVE_L(p)				  (0x03ad + (p) * 0x0c)
#define MAX96789_MIPI_DSI_HACTIVE_H(p)				  (0x03ae + (p) * 0x0c)
#define   HACTIVE_H_MASK					  GENMASK(4, 0)
#define   HACTIVE_H_SHIFT					  0

/* GMSL1: A -> 0, B -> 1 */
#define MAX96789_GMSL1_SS(l)					  (0x0402 + (l) * 0x200)
#define   SSEN							  BIT(5)
#define MAX96789_GMSL1_LINK_CFG(l)				  (0x0404 + (l) * 0x200)
#define   FWDCCEN						  BIT(0)
#define   REVCCEN						  BIT(1)
#define   AUDIO_EN						  BIT(4)
#define   PRBSEN						  BIT(5)
#define   CLINKEN						  BIT(6)
#define   SEREN							  BIT(7)
#define MAX96789_GMSL1_PRBS(l)					  (0x0405 + (l) * 0x200)
#define   PRBS_LEN_MASK						  GENMASK(5, 4)
#define   PRBS_LEN_SHIFT					  4
#define MAX96789_GMSL1_7(l)					  (0x0407 + (l) * 0x200)
#define   PXL_CRC						  BIT(0)
#define   HVEN							  BIT(2)
#define   DRS							  BIT(3)
#define   BWS							  BIT(5)
#define   HIBW							  BIT(6)
#define   DBL							  BIT(7)
#define MAX96789_GMSL1_D(l)					  (0x040d + (l) * 0x200)
#define   LOC_CC_EN						  BIT(0)
#define   I2C_LOC_ACK						  BIT(7)
#define MAX96789_GMSL1_F(l)					  (0x040f + (l) * 0x200)
#define   SET_GPO						  BIT(0)
#define   GPO_RX_EN						  BIT(2)
#define   CNTL_IN_EN_MASK					  GENMASK(7, 3)
#define   CNTL_IN_EN_SHIFT					  3
#define MAX96789_GMSL1_11(l)					  (0x0411 + (l) * 0x200)
#define   ERRG_EN_G1						  BIT(0)
#define   ERRG_PER2						  BIT(1)
#define   ERRG_CNT2_MASK					  GENMASK(3, 2)
#define   ERRG_CNT2_SHIFT					  2
#define   ERRG_TYPE2_MASK					  GENMASK(5, 4)
#define   ERRG_TYPE2_SHIFT					  4
#define   ERRG_RATE2_MASK					  GENMASK(7, 6)
#define   ERRG_RATE2_SHIFT					  6
#define MAX96789_GMSL1_13(l)					  (0x0413 + (l) * 0x200)
#define   ALLOW_PKTCC						  BIT(6)
#define   NO_REM_MST						  BIT(7)
#define MAX96789_GMSL1_14(l)					  (0x0414 + (l) * 0x200)
#define   REM_CCLOCK						  BIT(0)
#define   CC_WBLOCK						  BIT(1)
#define   REV_BIT_LOCK						  BIT(2)
#define   CC_WBLOCK_LOST					  BIT(3)
#define   I2C_TIMED_OUT2					  BIT(4)
#define MAX96789_GMSL1_15(l)					  (0x0415 + (l) * 0x200)
#define   SEL_RGB888						  BIT(0)
#define   SEL_VESA						  BIT(1)
#define MAX96789_GMSL1_16(l)					  (0x0416 + (l) * 0x200)
#define   GMSL_MAX_RT_ERR					  BIT(6)
#define MAX96789_GMSL1_CC_I2C_RETR_CNT(l)			  (0x041c + (l) * 0x200)
#define MAX96789_GMSL1_CC_CRC_ERRCNT(l)				  (0x041d + (l) * 0x200)
#define MAX96789_GMSL1_39(l)					  (0x0438 + (l) * 0x200)
#define   CROSS24_MASK						  GENMASK(4, 0)
#define   CROSS24_SHIFT						  0
#define   CROSS24_F						  BIT(5)
#define   CROSS24_I						  BIT(6)
#define MAX96789_GMSL1_3A(l)					  (0x0439 + (l) * 0x200)
#define   CROSS25_MASK						  GENMASK(4, 0)
#define   CROSS25_SHIFT						  0
#define   CROSS25_F						  BIT(5)
#define   CROSS25_I						  BIT(6)
#define MAX96789_GMSL1_3B(l)					  (0x043a + (l) * 0x200)
#define   CROSS26_MASK						  GENMASK(4, 0)
#define   CROSS26_SHIFT						  0
#define   CROSS26_F						  BIT(5)
#define   CROSS26_I						  BIT(6)
#define MAX96789_GMSL1_3C(l)					  (0x043b + (l) * 0x200)
#define   CROSS27_MASK						  GENMASK(4, 0)
#define   CROSS27_SHIFT						  0
#define   CROSS27_F						  BIT(5)
#define   CROSS27_I						  BIT(6)
#define MAX96789_GMSL1_3D(l)					  (0x043c + (l) * 0x200)
#define   CROSS28_MASK						  GENMASK(4, 0)
#define   CROSS28_SHIFT						  0
#define   CROSS28_F						  BIT(5)
#define   CROSS28_I						  BIT(6)
#define MAX96789_GMSL1_3E(l)					  (0x043d + (l) * 0x200)
#define   CROSS29_MASK						  GENMASK(4, 0)
#define   CROSS29_SHIFT						  0
#define   CROSS29_F						  BIT(5)
#define   CROSS29_I						  BIT(6)
#define MAX96789_GMSL1_3F(l)					  (0x043e + (l) * 0x200)
#define   CROSS30_MASK						  GENMASK(4, 0)
#define   CROSS30_SHIFT						  0
#define   CROSS30_F						  BIT(5)
#define   CROSS30_I						  BIT(6)
#define MAX96789_GMSL1_42(l)					  (0x0442 + (l) * 0x200)
#define   GPO_EN						  BIT(0)
#define   GPI_RT_EN						  BIT(1)
#define   GPI_COMP_EN						  BIT(2)
#define   I2C_RT_EN						  BIT(3)
#define   MAX_RT_EN						  BIT(4)
#define MAX96789_GMSL1_4D(l)					  (0x044d + (l) * 0x200)
#define   HIGHIMM						  BIT(7)
#define MAX96789_GMSL1_66(l)					  (0x0466 + (l) * 0x200)
#define   DIS_DE						  BIT(3)
#define   REV_FAST						  BIT(4)
#define   PRBS_TYPE						  BIT(5)
#define MAX96789_GMSL1_67(l)					  (0x0467 + (l) * 0x200)
#define   DBL_ALIGN_TO_MASK					  GENMASK(2, 0)
#define   DBL_ALIGN_TO_SHIFT					  0
#define   DE_RGB888_DBL						  BIT(4)
#define   AUTO_CLINK						  BIT(5)
#define   CNTL_TRIG_MASK					  GENMASK(7, 6)
#define   CNTL_TRIG_SHIFT					  6
#define MAX96789_GMSL1_68(l)					  (0x0468 + (l) * 0x200)
#define   CC_CRC_LENGTH_MASK					  GENMASK(1, 0)
#define   CC_CRC_LENGTH_SHIFT					  0
#define MAX96789_GMSL1_96(l)					  (0x0496 + (l) * 0x200)
#define   BYPASS_HVD_ALIGN					  BIT(3)
#define MAX96789_GMSL1_99(l)					  (0x0499 + (l) * 0x200)
#define   ALIGN_INFO						  BIT(2)
#define   ALIGN_VS_MODE						  BIT(3)
#define   REV_FILT_EN						  BIT(4)
#define MAX96789_GMSL1_9A(l)					  (0x049a + (l) * 0x200)
#define   PKTCC_EN						  BIT(3)
#define MAX96789_GMSL1_C8(l)					  (0x04c8 + (l) * 0x200)
#define   I2C_ACK_ACKED						  BIT(4)
#define   I2C_ACK_RECVED					  BIT(5)
#define   ALIGNING						  BIT(6)

/* MISC */
#define MAX96789_MISC_BITLEN_PT_1_L				  0x0548
#define MAX96789_MISC_UART_PT_1					  0x0549
#define   BITLEN_PT_1_H_MASK					  GENMASK(5, 0)
#define   BITLEN_PT_1_H_SHIFT					  0
#define MAX96789_MISC_BITLEN_PT_2_L				  0x054a
#define MAX96789_MISC_UART_PT_3					  0x054b
#define   BITLEN_PT_2_H_MASK					  GENMASK(5, 0)
#define   BITLEN_PT_2_H_SHIFT					  0
#define MAX96789_MISC_I2C_PT_4					  0x0550
#define   SRC_A_1_MASK						  GENMASK(7, 1)
#define   SRC_A_1_SHIFT						  1
#define MAX96789_MISC_I2C_PT_5					  0x0551
#define   DST_A_1_MASK						  GENMASK(7, 1)
#define   DST_A_1_SHIFT						  1
#define MAX96789_MISC_I2C_PT_6					  0x0552
#define   SRC_B_1_MASK						  GENMASK(7, 1)
#define   SRC_B_1_SHIFT						  1
#define MAX96789_MISC_I2C_PT_7					  0x0553
#define   DST_B_1_MASK						  GENMASK(7, 1)
#define   DST_B_1_SHIFT						  1
#define MAX96789_MISC_I2C_PT_8					  0x0554
#define   SRC_A_2_MASK						  GENMASK(7, 1)
#define   SRC_A_2_SHIFT						  1
#define MAX96789_MISC_I2C_PT_9					  0x0555
#define   DST_A_2_MASK						  GENMASK(7, 1)
#define   DST_A_2_SHIFT						  1
#define MAX96789_MISC_I2C_PT_10					  0x0556
#define   SRC_B_2_MASK						  GENMASK(7, 1)
#define   SRC_B_2_SHIFT						  1
#define MAX96789_MISC_I2C_PT_11					  0x0557
#define   DST_B_2_MASK						  GENMASK(7, 1)
#define   DST_B_2_SHIFT						  1
#define MAX96789_MISC_CSI_SSPLL_FB_FRACTION_IN_L		  0x055b
#define MAX96789_MISC_SSPLL_PREDEF_EN				  0x055c
#define   CSI_SSPLL_FB_FRACTION_IN_H_MASK			  GENMASK(3, 0)
#define   CSI_SSPLL_FB_FRACTION_IN_H_SHIFT			  0
#define   CSI_SSPLL_FB_FRACTION_PREDEF_EN			  BIT(4)
#define   SSPLL_SEL_BCLK_A_MASK					  GENMASK(7, 5)
#define   SSPLL_SEL_BCLK_A_SHIFT				  5
#define MAX96789_MISC_HS_VS_X					  0x055d
#define   HS_POL_X						  BIT(0)
#define   VS_POL_X						  BIT(1)
#define   HS_DET_X						  BIT(4)
#define   VS_DET_X						  BIT(5)
#define   DE_DET_X						  BIT(6)
#define MAX96789_MISC_HS_VS_Y					  0x055e
#define   HS_POL_Y						  BIT(0)
#define   VS_POL_Y						  BIT(1)
#define   HS_DET_Y						  BIT(4)
#define   VS_DET_Y						  BIT(5)
#define   DE_DET_Y						  BIT(6)
#define MAX96789_MISC_HS_VS_Z					  0x055f
#define   HS_POL_Z						  BIT(0)
#define   VS_POL_Z						  BIT(1)
#define   HS_DET_Z						  BIT(4)
#define   VS_DET_Z						  BIT(5)
#define   DE_DET_Z						  BIT(6)
#define MAX96789_MISC_HS_VS_U					  0x056a
#define   HS_POL_U						  BIT(0)
#define   VS_POL_U						  BIT(1)
#define   HS_DET_U						  BIT(4)
#define   VS_DET_U						  BIT(5)
#define   DE_DET_U						  BIT(6)
#define MAX96789_MISC_CSI_SSPLLB_FB_FRACTION_IN_L		  0x056c
#define MAX96789_MISC_SSPLLB_PREDEF_EN				  0x056d
#define   CSI_SSPLLB_FB_FRACTION_IN_H_MASK			  GENMASK(3, 0)
#define   CSI_SSPLLB_FB_FRACTION_IN_H_SHIFT			  0
#define   CSI_SSPLLB_FB_FRACTION_PREDEF_EN			  BIT(4)
#define   SSPLL_SEL_BCLK_B_MASK					  GENMASK(7, 5)
#define   SSPLL_SEL_BCLK_B_SHIFT				  5
#define MAX96789_MISC_SSPLL_DIV					  0x056e
#define   SSPLL_ODIV_EXP_MASK					  GENMASK(2, 0)
#define   SSPLL_ODIV_EXP_SHIFT					  0
#define   SSPLL_INDIV_MASK					  GENMASK(7, 3)
#define   SSPLL_INDIV_SHIFT					  3
#define MAX96789_MISC_SSPLL_FB_DIV_L				  0x056f
#define MAX96789_MISC_SSPLL_DIV3				  0x0570
#define   SSPLL_FORCE_DIV					  BIT(6)
#define   SSPLL_FB_DIV_H					  BIT(7)
#define MAX96789_MISC_SSPLL_DIV4				  0x0571
#define   SSPLLB_ODIV_EXP_MASK					  GENMASK(2, 0)
#define   SSPLLB_ODIV_EXP_SHIFT					  0
#define   SSPLLB_INDIV_MASK					  GENMASK(7, 3)
#define   SSPLLB_INDIV_SHIFT					  3
#define MAX96789_MISC_SSPLLB_FB_DIV_L				  0x0572
#define MAX96789_MISC_SSPLL_DIV6				  0x0573
#define   SSPLLB_FORCE_DIV					  BIT(6)
#define   SSPLLB_FB_DIV_H					  BIT(7)
#define MAX96789_MISC_G1_I2C_0					  0x0575
#define   G1_SLV_TO_MASK					  GENMASK(2, 0)
#define   G1_SLV_TO_SHIFT					  0
#define   G1_SLV_SH_MASK					  GENMASK(5, 4)
#define   G1_SLV_SH_SHIFT					  4
#define MAX96789_MISC_G1_I2C_2					  0x0577
#define   G1_SRC_A_MASK						  GENMASK(7, 1)
#define   G1_SRC_A_SHIFT					  1
#define MAX96789_MISC_G1_I2C_3					  0x0578
#define   G1_DST_A_MASK						  GENMASK(7, 1)
#define   G1_DST_A_SHIFT					  1
#define MAX96789_MISC_G1_I2C_4					  0x0579
#define   G1_SRC_B_MASK						  GENMASK(7, 1)
#define   G1_SRC_B_SHIFT					  1
#define MAX96789_MISC_G1_I2C_5					  0x057a
#define   G1_DST_B_MASK						  GENMASK(7, 1)
#define   G1_DST_B_SHIFT					  1
#define MAX96789_MISC_PM_OV_STAT				  0x057d
#define   OV_LEVEL_MASK						  GENMASK(1, 0)
#define   OV_LEVEL_SHIFT					  0

/* ASYM_DV */
#define MAX96789_ASYM_DV_M_A_L					  0x0700
#define MAX96789_ASYM_DV_M_A_M					  0x0701
#define MAX96789_ASYM_DV_M_A_H					  0x0702
#define MAX96789_ASYM_DV_M_B_L					  0x0703
#define MAX96789_ASYM_DV_M_B_M					  0x0704
#define MAX96789_ASYM_DV_M_B_H					  0x0705
#define MAX96789_ASYM_DV_N_L					  0x0706
#define MAX96789_ASYM_DV_N_M					  0x0707
#define MAX96789_ASYM_DV_N_H					  0x0708
#define MAX96789_ASYM_DV_X_OFF_WR_A_L				  0x0709
#define MAX96789_ASYM_DV_X_OFF_WR_A_H				  0x070a
#define MAX96789_ASYM_DV_X_MAX_A_L				  0x070b
#define MAX96789_ASYM_DV_X_MAX_A_H				  0x070c
#define MAX96789_ASYM_DV_Y_MAX_A_L				  0x070d
#define MAX96789_ASYM_DV_Y_MAX_A_H				  0x070e
#define MAX96789_ASYM_DV_X_OFF_WR_B_L				  0x070f
#define MAX96789_ASYM_DV_X_OFF_WR_B_H				  0x0710
#define MAX96789_ASYM_DV_X_MAX_B_L				  0x0711
#define MAX96789_ASYM_DV_X_MAX_B_H				  0x0712
#define MAX96789_ASYM_DV_Y_MAX_B_L				  0x0713
#define MAX96789_ASYM_DV_Y_MAX_B_H				  0x0714
#define MAX96789_ASYM_DV_FIFOLEVEL_A_L				  0x0715
#define MAX96789_ASYM_DV_ASYM22					  0x0716
#define   FIFOLEVEL_A_H_MASK					  GENMASK(4, 0)
#define   FIFOLEVEL_A_H_SHIFT					  0
#define   DE_INV_A						  BIT(5)
#define   HS_INV_A						  BIT(6)
#define   VS_INV_A						  BIT(7)
#define MAX96789_ASYM_DV_FIFOLEVEL_B_L				  0x0717
#define MAX96789_ASYM_DV_ASYM24					  0x0718
#define   FIFOLEVEL_B_H_MASK					  GENMASK(4, 0)
#define   FIFOLEVEL_B_H_SHIFT					  0
#define   DE_INV_B						  BIT(5)
#define   HS_INV_B						  BIT(6)
#define   VS_INV_B						  BIT(7)
#define MAX96789_ASYM_DV_ASYM25					  0x0719
#define   ASYM_START						  BIT(0)
#define   ASYM_EN_A						  BIT(1)
#define   ASYM_EN_B						  BIT(2)
#define   ASYM_WR_B_MUX						  BIT(3)
#define   LUT_TEMPLATE_SEL_B_MASK				  GENMASK(7, 4)
#define   LUT_TEMPLATE_SEL_B_SHIFT				  4
#define MAX96789_ASYM_DV_ASYM26					  0x071a
#define   ASYM_SF_VS_INV					  BIT(3)
#define   LUT_TEMPLATE_SEL_A_MASK				  GENMASK(7, 4)
#define   LUT_TEMPLATE_SEL_A_SHIFT				  4
#define MAX96789_ASYM_DV_ASYM27					  0x071b
#define   ASYM_OVERFLOW_B					  BIT(0)
#define   ASYM_OVERFLOW_A					  BIT(1)
#define   ASYM_UNDERFLOW_B					  BIT(2)
#define   ASYM_UNDERFLOW_A					  BIT(3)

/* ASYM_READ: dual-view settings: A -> 0, B -> 1 */
#define MAX96789_ASYM_READ_VS_DLY_0(v)				  (0x0720 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_DLY_1(v)				  (0x0721 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_DLY_2(v)				  (0x0722 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_HIGH_0(v)				  (0x0723 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_HIGH_1(v)				  (0x0724 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_HIGH_2(v)				  (0x0725 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_LOW_0(v)				  (0x0726 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_LOW_1(v)				  (0x0727 + (v) * 0x30)
#define MAX96789_ASYM_READ_VS_LOW_2(v)				  (0x0728 + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_DLY_0(v)				  (0x0729 + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_DLY_1(v)				  (0x072a + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_DLY_2(v)				  (0x072b + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_HIGH_0(v)				  (0x072c + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_HIGH_1(v)				  (0x072d + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_LOW_0(v)				  (0x072e + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_LOW_1(v)				  (0x072f + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_CNT_0(v)				  (0x0730 + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_CNT_1(v)				  (0x0731 + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_LLOW_0(v)				  (0x0732 + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_LLOW_1(v)				  (0x0733 + (v) * 0x30)
#define MAX96789_ASYM_READ_HS_LLOW_2(v)				  (0x0734 + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_DLY_0(v)				  (0x0735 + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_DLY_1(v)				  (0x0736 + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_DLY_2(v)				  (0x0737 + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_HIGH_0(v)				  (0x0738 + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_HIGH_1(v)				  (0x0739 + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_LOW_0(v)				  (0x073a + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_LOW_1(v)				  (0x073b + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_CNT_0(v)				  (0x073c + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_CNT_1(v)				  (0x073d + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_LLOW_0(v)				  (0x073e + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_LLOW_1(v)				  (0x073f + (v) * 0x30)
#define MAX96789_ASYM_READ_DE_LLOW_2(v)				  (0x0740 + (v) * 0x30)

/* ASYM_DV_LUT */
#define MAX96789_ASYM_SCREEN_B_LUT_BASE				  0x0800
#define MAX96789_ASYM_SCREEN_A_LUT_BASE				  0x1000
#define   ASYM_SCREEN_LUT_SIZE					  270

/* SPI_CC_WR */
#define MAX96789_SPI_CC_WR					  0x1300

/* SPI_CC_RD */
#define MAX96789_SPI_CC_RD					  0x1380

/* RLMS: links A -> 0, B -> 1 */
#define MAX96789_RLMS_3(l)					  (0x1403 + (l) * 0x100)
#define   ADAPTEN						  BIT(7)
#define MAX96789_RLMS_4(l)					  (0x1404 + (l) * 0x100)
#define   EOM_EN						  BIT(0)
#define   EOM_PER_MODE						  BIT(1)
#define   EOM_CHK_THR_MASK					  GENMASK(3, 2)
#define   EOM_CHK_THR_SHIFT					  2
#define   EOM_CHK_AMOUNT_MASK					  GENMASK(7, 4)
#define   EOM_CHK_AMOUNT_SHIFT					  4
#define MAX96789_RLMS_5(l)					  (0x1405 + (l) * 0x100)
#define   EOM_MIN_THR_MASK					  GENMASK(6, 0)
#define   EOM_MIN_THR_SHIFT					  0
#define   EOM_MAN_TRG_REQ					  BIT(7)
#define MAX96789_RLMS_6(l)					  (0x1406 + (l) * 0x100)
#define   EOM_RST_THR_MASK					  GENMASK(6, 0)
#define   EOM_RST_THR_SHIFT					  0
#define   EOM_PV_MODE						  BIT(7)
#define MAX96789_RLMS_7(l)					  (0x1407 + (l) * 0x100)
#define   EOM_MASK						  GENMASK(6, 0)
#define   EOM_SHIFT						  0
#define   EOM_DONE						  BIT(7)
#define MAX96789_RLMS_EYEMONPERCNTL(l)				  (0x1434 + (l) * 0x100)
#define MAX96789_RLMS_EYEMONPERCNTH(l)				  (0x1435 + (l) * 0x100)
#define MAX96789_RLMS_37(l)					  (0x1437 + (l) * 0x100)
#define   EYEMONDPOL						  BIT(0)
#define   EYEMONPH						  BIT(1)
#define   EYEMONSTART						  BIT(2)
#define   EYEMONCNTCLR						  BIT(3)
#define   EYEMONDONE						  BIT(4)
#define MAX96789_RLMS_EYEMONERRCNTL(l)				  (0x1438 + (l) * 0x100)
#define MAX96789_RLMS_EYEMONERRCNTH(l)				  (0x1439 + (l) * 0x100)
#define MAX96789_RLMS_EYEMONVALCNTL(l)				  (0x143a + (l) * 0x100)
#define MAX96789_RLMS_EYEMONVALCNTH(l)				  (0x143b + (l) * 0x100)
#define MAX96789_RLMS_3D(l)					  (0x143d + (l) * 0x100)
#define   ERRCHPHTOGEN						  BIT(0)
#define   ERRCHPH_MASK						  GENMASK(7, 1)
#define   ERRCHPH_SHIFT						  1
#define MAX96789_RLMS_3E(l)					  (0x143e + (l) * 0x100)
#define   ERRCHPHSEC_MASK					  GENMASK(6, 0)
#define   ERRCHPHSEC_SHIFT					  0
#define   ERRCHPHSECTA						  BIT(7)
#define MAX96789_RLMS_3F(l)					  (0x143f + (l) * 0x100)
#define   ERRCHPHPRI_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRI_SHIFT					  0
#define   ERRCHPHPRITA						  BIT(7)
#define MAX96789_RLMS_49(l)					  (0x1449 + (l) * 0x100)
#define   ERRCHPWRUP						  BIT(2)
#define MAX96789_RLMS_58(l)					  (0x1458 + (l) * 0x100)
#define   ERRCHVTH1_MASK					  GENMASK(6, 0)
#define   ERRCHVTH1_SHIFT					  0
#define MAX96789_RLMS_59(l)					  (0x1459 + (l) * 0x100)
#define   ERRCHVTH0_MASK					  GENMASK(6, 0)
#define   ERRCHVTH0_SHIFT					  0
#define MAX96789_RLMS_64(l)					  (0x1464 + (l) * 0x100)
#define   TXSSCMODE_MASK					  GENMASK(1, 0)
#define   TXSSCMODE_SHIFT					  0
#define MAX96789_RLMS_70(l)					  (0x1470 + (l) * 0x100)
#define   TXSSCFRQCTRL_MASK					  GENMASK(6, 0)
#define   TXSSCFRQCTRL_SHIFT					  0
#define MAX96789_RLMS_71(l)					  (0x1471 + (l) * 0x100)
#define   TXSSCEN						  BIT(0)
#define   TXSSCCENSPRST_MASK					  GENMASK(6, 1)
#define   TXSSCCENSPRST_SHIFT					  1
#define MAX96789_RLMS_TXSSCPRESCLL(l)				  (0x1472 + (l) * 0x100)
#define MAX96789_RLMS_73(l)					  (0x1473 + (l) * 0x100)
#define   TXSSCPRESCLH_MASK					  GENMASK(2, 0)
#define   TXSSCPRESCLH_SHIFT					  0
#define MAX96789_RLMS_TXSSCPHL(l)				  (0x1474 + (l) * 0x100)
#define MAX96789_RLMS_75(l)					  (0x1475 + (l) * 0x100)
#define   TXSSCPHH_MASK						  GENMASK(6, 0)
#define   TXSSCPHH_SHIFT					  0
#define MAX96789_RLMS_76(l)					  (0x1476 + (l) * 0x100)
#define   TXSSCPHQUAD_MASK					  GENMASK(1, 0)
#define   TXSSCPHQUAD_SHIFT					  0
#define MAX96789_RLMS_95(l)					  (0x1495 + (l) * 0x100)
#define   TXAMPLMAN_MASK					  GENMASK(5, 0)
#define   TXAMPLMAN_SHIFT					  0
#define   TXAMPLMANEN						  BIT(7)
#define MAX96789_RLMS_A4(l)					  (0x14a4 + (l) * 0x100)
#define   AEQ_PER_MASK						  GENMASK(5, 0)
#define   AEQ_PER_SHIFT						  0
#define   AEQ_PER_MULT_MASK					  GENMASK(7, 6)
#define   AEQ_PER_MULT_SHIFT					  6
#define MAX96789_RLMS_AC(l)					  (0x14ac + (l) * 0x100)
#define   ERRCHPHSECFR3G_MASK					  GENMASK(6, 0)
#define   ERRCHPHSECFR3G_SHIFT					  0
#define   ERRCHPHSECTAFR3G					  BIT(7)
#define MAX96789_RLMS_AD(l)					  (0x14ad + (l) * 0x100)
#define   ERRCHPHPRIFR3G_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRIFR3G_SHIFT					  0
#define   ERRCHPHPRITAFR3G					  BIT(7)
#define MAX96789_RLMS_B6(l)					  (0x14b6 + (l) * 0x100)
#define   ERRCHPHSECSRG1875_MASK				  GENMASK(6, 0)
#define   ERRCHPHSECSRG1875_SHIFT				  0
#define   ERRCHPHSECTASRG1875					  BIT(7)
#define MAX96789_RLMS_B7(l)					  (0x14b7 + (l) * 0x100)
#define   ERRCHPHPRISRG1875_MASK				  GENMASK(6, 0)
#define   ERRCHPHPRISRG1875_SHIFT				  0
#define   ERRCHPHPRITASRG1875					  BIT(7)

/* DPLL_AUD_SSPLL: A -> 0, B -> 1 */
#define MAX96789_DPLL_AUD_SSPLL_DPLL_3(ch)			  (0x1b03 + (ch) * 0x100)
#define   CONFIG_SPREAD_BIT_RATIO_MASK				  GENMASK(2, 0)
#define   CONFIG_SPREAD_BIT_RATIO_SHIFT				  0
#define   CONFIG_FORCE_ENABLE_SS				  BIT(3)

extern const struct regmap_config max96789_regmap_cfg;

enum max96789_dev_id {
	ID_MAX96789 = 0x80,
};

enum max96789_dsi_port_id {
	DSI_PORT_A,
	DSI_PORT_B,
	DSI_MAX_PORTS,
};

enum max96789_gmsl_link_id {
	GMSL_LINK_A,
	GMSL_LINK_B,
	GMSL_MAX_LINKS,
};

enum max96789_gmsl_link_type {
	LINK_TYPE_GMSL1,
	LINK_TYPE_GMSL2,
};

enum max96789_gmsl2_link_speed {
	GMSL2_SPEED_3G = 1,
	GMSL2_SPEED_6G = 2,
};

enum max96789_video_pipe_id {
	VIDEO_PIPE_X,
	VIDEO_PIPE_Y,
	VIDEO_PIPE_Z,
	VIDEO_PIPE_U,
	VIDEO_MAX_PIPES,
};

struct max96789 {
	struct regmap *regmap;
	struct device *dev;

	unsigned int id;
	unsigned int rev;

	unsigned int gmsl_link_mask;
	unsigned int gmsl_links_used;
	enum max96789_gmsl_link_type gmsl_links_types[GMSL_MAX_LINKS];
	enum max96789_gmsl2_link_speed gmsl2_link_speed;
	bool gmsl2_dual_link;
	bool gmsl_links_need_reset;
	bool link_setup_finished;
};

int max96789_dev_init(struct max96789 *max96789, ulong expected_dev_id);

#endif /* _MAX96789_H_ */
