// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2023 NXP
 */

#ifndef _MAX96752_REGS_H_
#define _MAX96752_REGS_H_

/* DEV */
#define MAX96752_DEV_REG0					  0x0000
#define   CFG_BLOCK						  BIT(0)
#define   DEV_ADDR_MASK						  GENMASK(7, 1)
#define   DEV_ADDR_SHIFT					  1
#define MAX96752_DEV_CH_CTRL					  0x0001
#define   RX_RATE_MASK						  GENMASK(1, 0)
#define   RX_RATE_SHIFT						  0
#define   TX_RATE_MASK						  GENMASK(3, 2)
#define   TX_RATE_SHIFT						  2
#define   IIC_1_EN						  BIT(4)
#define   IIC_2_EN						  BIT(5)
#define   LVDS_HALFSW						  BIT(7)
#define MAX96752_DEV_REG2					  0x0002
#define   AUD_TX_EN						  BIT(2)
#define   DIS_REM_CC						  BIT(4)
#define   DIS_LOCAL_CC						  BIT(5)
#define   VID_EN						  BIT(6)
#define   LOCK_CFG						  BIT(7)
#define MAX96752_DEV_REG3					  0x0003
#define   VIDEO_LOCK						  BIT(0)
#define   UART_1_EN						  BIT(2)
#define   UART_2_EN						  BIT(3)
#define   I2CSEL						  BIT(4)
#define MAX96752_DEV_ID						  0x000d
#define MAX96752_DEV_REV					  0x000e
#define   DEV_REV_MASK						  GENMASK(3, 0)
#define   DEV_REV_SHIFT						  0
#define MAX96752_DEV_VIDEO_CAP					  0x000f
#define   HDCP_CPBL						  BIT(0)
#define   SPLTR_CPBL						  BIT(1)
#define   DUAL_CPBL						  BIT(2)
#define   SPEED_CPBL_MASK					  GENMASK(5, 4)
#define   SPEED_CPBL_SHIFT					  4
#define MAX96752_DEV_PIN_DRV_EN_0				  0x0038
#define MAX96752_DEV_PIN_DRV_EN_1				  0x0039
#define MAX96752_DEV_IO_CHK2					  0x003a
#define   PIN_DRV_EN_2_MASK					  GENMASK(1, 0)
#define   PIN_DRV_EN_2_SHIFT					  0
#define   PIN_DRV_SEL						  BIT(7)
#define MAX96752_DEV_PIN_IN_0					  0x003b
#define MAX96752_DEV_PIN_IN_1					  0x003c

/* TCTRL */
#define MAX96752_TCTRL_PWR0					  0x0008
#define   CMP_STATUS_MASK					  GENMASK(4, 0)
#define   CMP_STATUS_SHIFT					  0
#define   VDDBAD_STATUS_MASK					  GENMASK(7, 5)
#define   VDDBAD_STATUS_SHIFT					  5
#define MAX96752_TCTRL_PWR1					  0x0009
#define   OVERTEMP						  BIT(7)
#define MAX96752_TCTRL_PWR4					  0x000c
#define   WAKE_EN_A						  BIT(4)
#define   WAKE_EN_B						  BIT(5)
#define   DIS_LOCAL_WAKE					  BIT(6)
#define MAX96752_TCTRL_CTRL0					  0x0010
#define   LINK_CFG_MASK						  GENMASK(1, 0)
#define   LINK_CFG_SHIFT					  0
#define   LINK_CFG_DUAL_LINK					  0
#define   LINK_CFG_GMSL_A					  1
#define   LINK_CFG_GMSL_B					  2
#define   LINK_CFG_SPLITTER					  3
#define   SLEEP							  BIT(3)
#define   AUTO_LINK						  BIT(4)
#define   RESET_ONESHOT						  BIT(5)
#define   RESET_LINK						  BIT(6)
#define   RESET_ALL						  BIT(7)
#define MAX96752_TCTRL_CTRL1					  0x0011
#define   CXTP_A						  BIT(0)
#define   BACK_COMP_SPLTR					  BIT(3)
#define   CXTP_B						  BIT(4)
#define MAX96752_TCTRL_CTRL3					  0x0013
#define   CMU_LOCKED						  BIT(1)
#define   ERROR							  BIT(2)
#define   LOCKED						  BIT(3)
#define   LINK_MODE_MASK					  GENMASK(5, 4)
#define   LINK_MODE_SHIFT					  4
#define MAX96752_TCTRL_INTR0					  0x0018
#define   DEC_ERR_THR_MASK					  GENMASK(2, 0)
#define   DEC_ERR_THR_SHIFT					  0
#define   AUTO_ERR_RST_EN					  BIT(3)
#define MAX96752_TCTRL_INTR1					  0x0019
#define   PKT_CNT_THR_MASK					  GENMASK(2, 0)
#define   PKT_CNT_THR_SHIFT					  0
#define   AUTO_CNT_RST_EN					  BIT(3)
#define   PKT_CNT_EXP_MASK					  GENMASK(7, 4)
#define   PKT_CNT_EXP_SHIFT					  4
#define MAX96752_TCTRL_INTR2					  0x001a
#define   DEC_ERR_OEN_A						  BIT(0)
#define   DEC_ERR_OEN_B						  BIT(1)
#define   IDLE_ERR_OEN						  BIT(2)
#define   REM_ERR_OEN						  BIT(5)
#define MAX96752_TCTRL_INTR3					  0x001b
#define   DEC_ERR_FLAG_A					  BIT(0)
#define   DEC_ERR_FLAG_B					  BIT(1)
#define   IDLE_ERR_FLAG						  BIT(2)
#define   REM_ERR_FLAG						  BIT(5)
#define MAX96752_TCTRL_INTR4					  0x001c
#define   WM_ERR_OEN						  BIT(0)
#define   PKT_CNT_OEN						  BIT(1)
#define   RT_CNT_OEN						  BIT(2)
#define   MAX_RT_OEN						  BIT(3)
#define   HDCP_INT_OEN						  BIT(4)
#define   EOM_ERR_OEN_A						  BIT(6)
#define MAX96752_TCTRL_INTR5					  0x001d
#define   WM_ERR_FLAG						  BIT(0)
#define   PKT_CNT_FLAG						  BIT(1)
#define   RT_CNT_FLAG						  BIT(2)
#define   MAX_RT_FLAG						  BIT(3)
#define   HDCP_INT_FLAG						  BIT(4)
#define   EOM_ERR_FLAG_A					  BIT(6)
#define   EOM_ERR_FLAG_B					  BIT(7)
#define MAX96752_TCTRL_INTR6					  0x001e
#define   VID_PXL_CRC_ERR_OEN					  BIT(0)
#define   APRBS_ERR_OEN						  BIT(1)
#define   VPRBS_ERR_OEN						  BIT(2)
#define   LCRC_ERR_OEN						  BIT(3)
#define   VDD_OV_OEN						  BIT(4)
#define   VDDBAD_INT_OEN					  BIT(5)
#define   VDDCMP_INT_OEN					  BIT(7)
#define MAX96752_TCTRL_INTR7					  0x001f
#define   VID_PXL_CRC_ERR_FLAG					  BIT(0)
#define   APRBS_ERR_FLAG					  BIT(1)
#define   VPRBS_ERR_FLAG					  BIT(2)
#define   LCRC_ERR_FLAG						  BIT(3)
#define   VDD_OV_FLAG						  BIT(4)
#define   VDDBAD_INT_FLAG					  BIT(5)
#define   VDDCMP_INT_FLAG					  BIT(7)
#define MAX96752_TCTRL_INTR8					  0x0020
#define   ERR_TX_ID_MASK					  GENMASK(4, 0)
#define   ERR_TX_ID_SHIFT					  0
#define   ERR_TX_EN						  BIT(7)
#define MAX96752_TCTRL_INTR9					  0x0021
#define   ERR_RX_ID_MASK					  GENMASK(4, 0)
#define   ERR_RX_ID_SHIFT					  0
#define   ERR_RX_EN						  BIT(7)
#define MAX96752_TCTRL_DEC_ERR_A				  0x0022
#define MAX96752_TCTRL_IDLE_ERR					  0x0024
#define MAX96752_TCTRL_PKT_CNT					  0x0025
#define MAX96752_TCTRL_VID_PXL_CRC_ERR				  0x0026

/* GMSL */
#define MAX96752_GMSL_TX1					  0x0029
#define   ERRG_EN_A						  BIT(4)
#define   ERRG_EN_B						  BIT(5)
#define   LINK_PRBS_GEN						  BIT(7)
#define MAX96752_GMSL_TX2					  0x002a
#define   ERRG_PER						  BIT(0)
#define   ERRG_BURST_MASK					  GENMASK(3, 1)
#define   ERRG_BURST_SHIFT					  1
#define   ERRG_RATE_MASK					  GENMASK(5, 4)
#define   ERRG_RATE_SHIFT					  4
#define   ERRG_CNT_MASK						  GENMASK(7, 6)
#define   ERRG_CNT_SHIFT					  6
#define MAX96752_GMSL_TX3					  0x002b
#define   TIMEOUT_MASK						  GENMASK(2, 0)
#define   TIMEOUT_SHIFT						  0
#define MAX96752_GMSL_RX0					  0x002c
#define   PKT_CNT_SEL_MASK					  GENMASK(3, 0)
#define   PKT_CNT_SEL_SHIFT					  0
#define   PKT_CNT_LBW_MASK					  GENMASK(7, 6)
#define   PKT_CNT_LBW_SHIFT					  6
#define MAX96752_GMSL_GPIOA					  0x0030
#define   GPIO_FWD_CDLY_MASK					  GENMASK(5, 0)
#define   GPIO_FWD_CDLY_SHIFT					  0
#define   GPIO_RX_FAST_BIDIR_EN					  BIT(7)
#define MAX96752_GMSL_GPIOB					  0x0031
#define   GPIO_REV_CDLY_MASK					  GENMASK(5, 0)
#define   GPIO_REV_CDLY_SHIFT					  0
#define   GPIO_TX_WNDW_MASK					  GENMASK(7, 6)
#define   GPIO_TX_WNDW_SHIFT					  6

/* CC */
#define MAX96752_CC_I2C_0					  0x0040
#define   SLV_TO_MASK						  GENMASK(2, 0)
#define   SLV_TO_SHIFT						  0
#define   SLV_SH_MASK						  GENMASK(5, 4)
#define   SLV_SH_SHIFT						  4
#define MAX96752_CC_I2C_1					  0x0041
#define   MST_TO_MASK						  GENMASK(2, 0)
#define   MST_TO_SHIFT						  0
#define   MST_BT_MASK						  GENMASK(6, 4)
#define   MST_BT_SHIFT						  4
#define MAX96752_CC_I2C_2					  0x0042
#define   SRC_A_MASK						  GENMASK(7, 1)
#define   SRC_A_SHIFT						  1
#define MAX96752_CC_I2C_3					  0x0043
#define   DST_A_MASK						  GENMASK(7, 1)
#define   DST_A_SHIFT						  1
#define MAX96752_CC_I2C_4					  0x0044
#define   SRC_B_MASK						  GENMASK(7, 1)
#define   SRC_B_SHIFT						  1
#define MAX96752_CC_I2C_5					  0x0045
#define   DST_B_MASK						  GENMASK(7, 1)
#define   DST_B_SHIFT						  1
#define MAX96752_CC_I2C_6					  0x0046
#define   I2C_SRC_CNT_MASK					  GENMASK(2, 0)
#define   I2C_SRC_CNT_SHIFT					  0
#define   I2C_AUTO_CFG						  BIT(3)
#define MAX96752_CC_I2C_7					  0x0047
#define   REM_ACK_RECVED					  BIT(0)
#define   REM_ACK_ACKED						  BIT(1)
#define   I2C_TIMED_OUT						  BIT(2)
#define   UART_TX_OVERFLOW					  BIT(6)
#define   UART_RX_OVERFLOW					  BIT(7)
#define MAX96752_CC_UART_0					  0x0048
#define   BYPASS_EN						  BIT(0)
#define   BYPASS_TO_MASK					  GENMASK(2, 1)
#define   BYPASS_TO_SHIFT					  1
#define   BYPASS_DIS_PAR					  BIT(3)
#define   LOC_MS_EN						  BIT(4)
#define   REM_MS_EN						  BIT(5)
#define   ARB_TO_LEN_MASK					  GENMASK(7, 6)
#define   ARB_TO_LEN_SHIFT					  6
#define MAX96752_CC_BITLEN_LSB					  0x0049
#define MAX96752_CC_UART_2					  0x004a
#define   BITLEN_MSB_MASK					  GENMASK(5, 0)
#define   BITLEN_MSB_SHIFT					  0
#define   OUT_DELAY_MASK					  GENMASK(7, 6)
#define   OUT_DELAY_SHIFT					  6
#define MAX96752_CC_I2C_PT_0					  0x004c
#define   SLV_TO_PT_MASK					  GENMASK(2, 0)
#define   SLV_TO_PT_SHIFT					  0
#define   SLV_SH_PT_MASK					  GENMASK(5, 4)
#define   SLV_SH_PT_SHIFT					  4
#define MAX96752_CC_I2C_PT_1					  0x004d
#define   MST_TO_PT_MASK					  GENMASK(2, 0)
#define   MST_TO_PT_SHIFT					  0
#define   MST_BT_PT_MASK					  GENMASK(6, 4)
#define   MST_BT_PT_SHIFT					  4
#define   MST_DBL_PT						  BIT(7)
#define MAX96752_CC_I2C_PT_2					  0x004e
#define   REM_ACK_RECVED_1					  BIT(0)
#define   REM_ACK_ACKED_1					  BIT(1)
#define   I2C_TIMED_OUT_1					  BIT(2)
#define   XOVER_EN_1						  BIT(3)
#define   REM_ACK_RECVED_2					  BIT(4)
#define   REM_ACK_ACKED_2					  BIT(5)
#define   I2C_TIMED_OUT_2					  BIT(6)
#define   XOVER_EN_2						  BIT(7)
#define MAX96752_CC_UART_PT_0					  0x004f
#define   UART_TX_OVERFLOW_1					  BIT(0)
#define   UART_RX_OVERFLOW_1					  BIT(1)
#define   DIS_PAR_1						  BIT(2)
#define   BITLEN_MAN_CFG_1					  BIT(3)
#define   UART_TX_OVERFLOW_2					  BIT(4)
#define   UART_RX_OVERFLOW_2					  BIT(5)
#define   DIS_PAR_2						  BIT(6)
#define   BITLEN_MAN_CFG_2					  BIT(7)

/* CFGH_VIDEO */
#define MAX96752_CFGH_VIDEO_RX0					  0x0050
#define   STR_SEL_MASK						  GENMASK(1, 0)
#define   STR_SEL_SHIFT						  0
#define   RX_CRC_EN						  BIT(7)

/* CFGL_AUDIO */
#define MAX96752_CFGL_AUDIO_TR0					  0x0058
#define   AUD_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   AUD_PRIO_CFG_SHIFT					  0
#define   AUD_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   AUD_PRIO_VAL_SHIFT					  2
#define   AUD_RX_CRC_EN						  BIT(6)
#define   AUD_TX_CRC_EN						  BIT(7)
#define MAX96752_CFGL_AUDIO_TR1					  0x0059
#define   AUD_BW_VAL_MASK					  GENMASK(5, 0)
#define   AUD_BW_VAL_SHIFT					  0
#define   AUD_BW_MULT_MASK					  GENMASK(7, 6)
#define   AUD_BW_MULT_SHIFT					  6
#define MAX96752_CFGL_AUDIO_TR3					  0x005b
#define   AUD_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   AUD_TX_SRC_ID_SHIFT					  0
#define   AUD_TX_SPLT_MASK_A					  BIT(4)
#define   AUD_TX_SPLT_MASK_B					  BIT(5)
#define MAX96752_CFGL_AUDIO_RX_SRC_SEL				  0x005c
#define MAX96752_CFGL_AUDIO_ARQ0				  0x005d
#define   AUD_ACK_SRC_ID					  BIT(4)
#define   AUD_MATCH_SRC_ID					  BIT(5)
#define   AUD_ACK_CNT						  BIT(6)
#define   AUD_ARQ_AUTO_CFG					  BIT(7)
#define MAX96752_CFGL_AUDIO_ARQ1				  0x005e
#define   AUD_RT_CNT_OEN					  BIT(0)
#define   AUD_MAX_RT_ERR_OEN					  BIT(1)
#define   AUD_MAX_RT_MASK					  GENMASK(6, 4)
#define   AUD_MAX_RT_SHIFT					  4
#define MAX96752_CFGL_AUDIO_ARQ2				  0x005f
#define   AUD_RT_CNT_MASK					  GENMASK(6, 0)
#define   AUD_RT_CNT_SHIFT					  0
#define   AUD_MAX_RT_ERR					  BIT(7)

/* CFGI_INFOFR */
#define MAX96752_CFGI_INFOFR_TR0				  0x0060
#define   INFOFR_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   INFOFR_PRIO_CFG_SHIFT					  0
#define   INFOFR_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   INFOFR_PRIO_VAL_SHIFT					  2
#define   INFOFR_RX_CRC_EN					  BIT(6)
#define   INFOFR_TX_CRC_EN					  BIT(7)
#define MAX96752_CFGI_INFOFR_TR1				  0x0061
#define   INFOFR_BW_VAL_MASK					  GENMASK(5, 0)
#define   INFOFR_BW_VAL_SHIFT					  0
#define   INFOFR_BW_MULT_MASK					  GENMASK(7, 6)
#define   INFOFR_BW_MULT_SHIFT					  6
#define MAX96752_CFGI_INFOFR_TR3				  0x0063
#define   INFOFR_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   INFOFR_TX_SRC_ID_SHIFT				  0
#define   INFOFR_TX_SPLT_MASK_A					  BIT(4)
#define   INFOFR_TX_SPLT_MASK_B					  BIT(5)
#define MAX96752_CFGI_INFOFR_RX_SRC_SEL				  0x0064

/* CFGL_SPI */
#define MAX96752_CFGL_SPI_TR0					  0x0068
#define   SPI_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   SPI_PRIO_CFG_SHIFT					  0
#define   SPI_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   SPI_PRIO_VAL_SHIFT					  2
#define   SPI_RX_CRC_EN						  BIT(6)
#define   SPI_TX_CRC_EN						  BIT(7)
#define MAX96752_CFGL_SPI_TR1					  0x0069
#define   SPI_BW_VAL_MASK					  GENMASK(5, 0)
#define   SPI_BW_VAL_SHIFT					  0
#define   SPI_BW_MULT_MASK					  GENMASK(7, 6)
#define   SPI_BW_MULT_SHIFT					  6
#define MAX96752_CFGL_SPI_TR3					  0x006b
#define   SPI_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   SPI_TX_SRC_ID_SHIFT					  0
#define   SPI_TX_SPLT_MASK_A					  BIT(4)
#define   SPI_TX_SPLT_MASK_B					  BIT(5)
#define MAX96752_CFGL_SPI_RX_SRC_SEL				  0x006c
#define MAX96752_CFGL_SPI_ARQ0					  0x006d
#define   SPI_ACK_SRC_ID					  BIT(4)
#define   SPI_MATCH_SRC_ID					  BIT(5)
#define   SPI_ACK_CNT						  BIT(6)
#define   SPI_ARQ_AUTO_CFG					  BIT(7)
#define MAX96752_CFGL_SPI_ARQ1					  0x006e
#define   SPI_RT_CNT_OEN					  BIT(0)
#define   SPI_MAX_RT_ERR_OEN					  BIT(1)
#define   SPI_MAX_RT_MASK					  GENMASK(6, 4)
#define   SPI_MAX_RT_SHIFT					  4
#define MAX96752_CFGL_SPI_ARQ2					  0x006f
#define   SPI_RT_CNT_MASK					  GENMASK(6, 0)
#define   SPI_RT_CNT_SHIFT					  0
#define   SPI_MAX_RT_ERR					  BIT(7)

/* CFGC_CC */
#define MAX96752_CFGC_CC_TR0					  0x0070
#define   CC_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   CC_PRIO_CFG_SHIFT					  0
#define   CC_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   CC_PRIO_VAL_SHIFT					  2
#define   CC_RX_CRC_EN						  BIT(6)
#define   CC_TX_CRC_EN						  BIT(7)
#define MAX96752_CFGC_CC_TR1					  0x0071
#define   CC_BW_VAL_MASK					  GENMASK(5, 0)
#define   CC_BW_VAL_SHIFT					  0
#define   CC_BW_MULT_MASK					  GENMASK(7, 6)
#define   CC_BW_MULT_SHIFT					  6
#define MAX96752_CFGC_CC_TR3					  0x0073
#define   CC_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   CC_TX_SRC_ID_SHIFT					  0
#define   CC_TX_SPLT_MASK_A					  BIT(4)
#define   CC_TX_SPLT_MASK_B					  BIT(5)
#define MAX96752_CFGC_CC_RX_SRC_SEL				  0x0074
#define MAX96752_CFGC_CC_ARQ0					  0x0075
#define   CC_ACK_SRC_ID						  BIT(4)
#define   CC_MATCH_SRC_ID					  BIT(5)
#define   CC_ACK_CNT						  BIT(6)
#define   CC_ARQ_AUTO_CFG					  BIT(7)
#define MAX96752_CFGC_CC_ARQ1					  0x0076
#define   CC_RT_CNT_OEN						  BIT(0)
#define   CC_MAX_RT_ERR_OEN					  BIT(1)
#define   CC_MAX_RT_MASK					  GENMASK(6, 4)
#define   CC_MAX_RT_SHIFT					  4
#define MAX96752_CFGC_CC_ARQ2					  0x0077
#define   CC_RT_CNT_MASK					  GENMASK(6, 0)
#define   CC_RT_CNT_SHIFT					  0
#define   CC_MAX_RT_ERR						  BIT(7)

/* CFGL_GPIO */
#define MAX96752_CFGL_GPIO_TR0					  0x0078
#define   GPIO_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   GPIO_PRIO_CFG_SHIFT					  0
#define   GPIO_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   GPIO_PRIO_VAL_SHIFT					  2
#define   GPIO_RX_CRC_EN					  BIT(6)
#define   GPIO_TX_CRC_EN					  BIT(7)
#define MAX96752_CFGL_GPIO_TR1					  0x0079
#define   GPIO_BW_VAL_MASK					  GENMASK(5, 0)
#define   GPIO_BW_VAL_SHIFT					  0
#define   GPIO_BW_MULT_MASK					  GENMASK(7, 6)
#define   GPIO_BW_MULT_SHIFT					  6
#define MAX96752_CFGL_GPIO_TR3					  0x007b
#define   GPIO_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   GPIO_TX_SRC_ID_SHIFT					  0
#define   GPIO_TX_SPLT_MASK_A					  BIT(4)
#define   GPIO_TX_SPLT_MASK_B					  BIT(5)
#define MAX96752_CFGL_GPIO_RX_SRC_SEL				  0x007c
#define MAX96752_CFGL_GPIO_ARQ0					  0x007d
#define   GPIO_ACK_SRC_ID					  BIT(4)
#define   GPIO_MATCH_SRC_ID					  BIT(5)
#define   GPIO_ACK_CNT						  BIT(6)
#define   GPIO_ARQ_AUTO_CFG					  BIT(7)
#define MAX96752_CFGL_GPIO_ARQ1					  0x007e
#define   GPIO_RT_CNT_OEN					  BIT(0)
#define   GPIO_MAX_RT_ERR_OEN					  BIT(1)
#define   GPIO_MAX_RT_MASK					  GENMASK(6, 4)
#define   GPIO_MAX_RT_SHIFT					  4
#define MAX96752_CFGL_GPIO_ARQ2					  0x007f
#define   GPIO_RT_CNT_MASK					  GENMASK(6, 0)
#define   GPIO_RT_CNT_SHIFT					  0
#define   GPIO_MAX_RT_ERR					  BIT(7)

/* CFGL_IIC: pass-through I2C: X -> 0, Y -> 1 */
#define MAX96752_CFGL_IIC_TR0(ch)				  (0x00a0 + (ch) * 8)
#define   PT_PRIO_CFG_MASK					  GENMASK(1, 0)
#define   PT_PRIO_CFG_SHIFT					  0
#define   PT_PRIO_VAL_MASK					  GENMASK(3, 2)
#define   PT_PRIO_VAL_SHIFT					  2
#define   PT_RX_CRC_EN						  BIT(6)
#define   PT_TX_CRC_EN						  BIT(7)
#define MAX96752_CFGL_IIC_TR1(ch)				  (0x00a1 + (ch) * 8)
#define   PT_BW_VAL_MASK					  GENMASK(5, 0)
#define   PT_BW_VAL_SHIFT					  0
#define   PT_BW_MULT_MASK					  GENMASK(7, 6)
#define   PT_BW_MULT_SHIFT					  6
#define MAX96752_CFGL_IIC_TR3(ch)				  (0x00a3 + (ch) * 8)
#define   PT_TX_SRC_ID_MASK					  GENMASK(2, 0)
#define   PT_TX_SRC_ID_SHIFT					  0
#define   PT_TX_SPLT_MASK_A					  BIT(4)
#define   PT_TX_SPLT_MASK_B					  BIT(5)
#define MAX96752_CFGL_IIC_RX_SRC_SEL(ch)			  (0x00a4 + (ch) * 8)
#define MAX96752_CFGL_IIC_ARQ0(ch)				  (0x00a5 + (ch) * 8)
#define   PT_ACK_SRC_ID						  BIT(4)
#define   PT_MATCH_SRC_ID					  BIT(5)
#define   PT_ACK_CNT						  BIT(6)
#define   PT_ARQ_AUTO_CFG					  BIT(7)
#define MAX96752_CFGL_IIC_ARQ1(ch)				  (0x00a6 * (ch) * 8)
#define   PT_RT_CNT_OEN						  BIT(0)
#define   PT_MAX_RT_ERR_OEN					  BIT(1)
#define   PT_MAX_RT_MASK					  GENMASK(6, 4)
#define   PT_MAX_RT_SHIFT					  4
#define MAX96752_CFGL_IIC_ARQ2(ch)				  0x00a7
#define   PT_RT_CNT_MASK					  GENMASK(6, 0)
#define   PT_RT_CNT_SHIFT					  0
#define   PT_MAX_RT_ERR						  BIT(7)

/* VID_RX */
#define MAX96752_VID_RX0					  0x0100
#define   LINE_CRC_EN						  BIT(1)
#define   LCRC_ERR						  BIT(7)
#define MAX96752_VID_RX3					  0x0103
#define   HTRACKEN						  BIT(0)
#define   VTRACKEN						  BIT(1)
#define   DTRACKEN						  BIT(2)
#define   HLOCKED						  BIT(3)
#define   VLOCKED						  BIT(4)
#define   DLOCKED						  BIT(5)
#define   HD_TR_MODE						  BIT(6)
#define MAX96752_VID_RX_OVERFLW_LIM_L				  0x0104
#define MAX96752_VID_RX_UNDERFLW_LIM_L				  0x0105
#define MAX96752_VID_RX6					  0x0106
#define   VID_OVERFLW_LIM_H					  BIT(0)
#define   VID_UNDERFLW_LIM_H					  BIT(1)
#define   VID_SAR_INC_H_MASK					  GENMASK(7, 5)
#define   VID_SAR_INC_H_SHIFT					  5
#define MAX96752_VID_RX_SAR_INC_L				  0x0107
#define MAX96752_VID_RX8					  0x0108
#define   VID_SEQ_ERR						  BIT(4)
#define   VID_PKT_DET						  BIT(5)
#define   VID_LOCK						  BIT(6)
#define   VID_BLK_LEN_ERR					  BIT(7)
#define MAX96752_VID_RX10					  0x010a
#define   MASK_VIDEO_DE						  BIT(6)
#define MAX96752_VID_RX_CLOCKGEN				  0x0111
#define   CLK_FREEZE						  BIT(0)
#define   CLK_FREEZE_CONS					  BIT(1)
#define   CLK_FREEZE_RNG_MASK					  GENMASK(3, 2)
#define   CLK_FREEZE_RNG_SHIFT					  2

/* AUD_TX */
#define MAX96752_AUD_TX0					  0x0120
#define   AUD_SINK_SRC						  BIT(0)
#define   FORCE_AUD						  BIT(1)
#define   INV_WS						  BIT(2)
#define   INV_SCK						  BIT(3)
#define   I2S_CFG_MASK						  GENMASK(5, 4)
#define   I2S_CFG_SHIFT						  4
#define   I2S_TDM_CFG_MASK					  GENMASK(7, 6)
#define   I2S_TDM_CFG_SHIFT					  6
#define MAX96752_AUD_TX1					  0x0121
#define   AUD_INF_PER						  BIT(2)
#define   AUD_DRIFT_DET_EN					  BIT(3)
#define   AUD_STR_TX_MASK					  GENMASK(5, 4)
#define   AUD_STR_TX_SHIFT					  4
#define   AUD_PRIO_MASK						  GENMASK(7, 6)
#define   AUD_PRIO_SHIFT					  6
#define MAX96752_AUD_TX5					  0x0125
#define   ACLKDET						  BIT(4)
#define   AUD_OVERFLOW						  BIT(5)
#define   AUD_FIFO_WARN						  BIT(6)
#define   AUD_DRIFT_ERR						  BIT(7)
#define MAX96752_AUD_TX7					  0x0127
#define   PRBSEN_AUD						  BIT(6)
#define   PRBS_SEL						  BIT(7)
#define MAX96752_AUD_TX8					  0x0128
#define   PRBS_WS_GEN						  BIT(6)
#define   PRBS_WS_LEN						  BIT(7)

/* AUD_RX */
#define MAX96752_AUD_RX1					  0x0140
#define   AUD_EN_RX						  BIT(0)
#define   INV_WS_RX						  BIT(2)
#define   INV_SCK_RX						  BIT(3)
#define   AUD_RX_SINK_SRC					  BIT(7)
#define MAX96752_AUD_RX_APRBS_ERR				  0x0143
#define MAX96752_AUD_RX7					  0x0146
#define   AUD_STRM_MASK						  GENMASK(3, 2)
#define   AUD_STRM_SHIFT					  2
#define   APRBS_CHK_EN						  BIT(4)
#define MAX96752_AUD_RX9					  0x0148
#define   APRBS_VALID						  BIT(4)
#define   AUD_PKT_DET						  BIT(5)
#define   AUD_LOCK						  BIT(6)
#define   AUD_BLK_LEN_ERR					  BIT(7)
#define MAX96752_AUD_RX_INFO_RX4				  0x014d
#define   INFO_AUD_DEPTH_MASK					  GENMASK(6, 0)
#define   INFO_AUD_DEPTH_SHIFT					  0

/* SPI */
#define MAX96752_SPI_0						  0x0160
#define   SPI_EN						  BIT(0)
#define   MST_SLVN						  BIT(1)
#define   SPI_CC_EN						  BIT(2)
#define   SPI_IGNR_ID						  BIT(3)
#define   SPI_CC_TRG_ID_MASK					  GENMASK(5, 4)
#define   SPI_CC_TRG_ID_SHIFT					  4
#define   SPI_LOC_ID_MASK					  GENMASK(7, 6)
#define   SPI_LOC_ID_SHIFT					  6
#define MAX96752_SPI_1						  0x0161
#define   SPI_BASE_PRIO_MASK					  GENMASK(1, 0)
#define   SPI_BASE_PRIO_SHIFT					  0
#define   SPI_LOC_N_MASK					  GENMASK(7, 2)
#define   SPI_LOC_N_SHIFT					  2
#define MAX96752_SPI_2						  0x0162
#define   SPIM_SS1_ACT_H					  BIT(0)
#define   SPIM_SS2_ACT_H					  BIT(1)
#define   SPI_MOD3						  BIT(2)
#define   SPI_MOD3_F						  BIT(3)
#define   FULL_SCK_SETUP					  BIT(4)
#define   REQ_HOLD_OFF_MASK					  GENMASK(7, 5)
#define   REQ_HOLD_OFF_SHIFT					  5
#define MAX96752_SPI_SPIM_SS_DLY_CLKS				  0x0163
#define MAX96752_SPI_SPIM_SCK_LO_CLKS				  0x0164
#define MAX96752_SPI_SPIM_SCK_HI_CLKS				  0x0165
#define MAX96752_SPI_SPI_6					  0x0166
#define   RWN_IO_EN						  BIT(0)
#define   BNE_IO_EN						  BIT(1)
#define   SS_IO_EN_1						  BIT(2)
#define   SS_IO_EN_2						  BIT(3)
#define   SPIS_RWN						  BIT(4)
#define   BNE							  BIT(5)
#define MAX96752_SPI_7						  0x0167
#define   SPIS_BYTE_CNT_MASK					  GENMASK(4, 0)
#define   SPIS_BYTE_CNT_SHIFT					  0
#define   SPI_TX_OVRFLW						  BIT(6)
#define   SPI_RX_OVRFLW						  BIT(7)
#define MAX96752_SPI_REQ_HOLD_OFF_TO				  0x0168

/* WM */
#define MAX96752_WM_0						  0x0188
#define   WM_EN							  BIT(0)
#define   WME_EN						  BIT(1)
#define   WM_DET_MASK						  GENMASK(3, 2)
#define   WM_DET_SHIFT						  2
#define   WM_MODE_MASK						  GENMASK(6, 4)
#define   WM_MODE_SHIFT						  4
#define   WM_LEN						  BIT(7)
#define MAX96752_WM_2						  0x018a
#define   WM_NPFILT_MASK					  GENMASK(1, 0)
#define   WM_NPFILT_SHIFT					  0
#define   VSYNCPOL						  BIT(2)
#define   HSYNCPOL						  BIT(3)
#define MAX96752_WM_3						  0x018b
#define   WM_TH_MASK						  GENMASK(6, 0)
#define   WM_TH_SHIFT						  0
#define MAX96752_WM_4						  0x018c
#define   WM_MASKMODE_MASK					  GENMASK(1, 0)
#define   WM_MASKMODE_SHIFT					  0
#define MAX96752_WM_5						  0x018d
#define   WM_ERROR						  BIT(0)
#define   WM_DETOUT						  BIT(1)
#define MAX96752_WM_TIMER					  0x018e
#define MAX96752_WM_WREN_L					  0x01ae
#define MAX96752_WM_WREN_H					  0x01af

/* VRX */
#define MAX96752_VRX_CROSS_0					  0x01b0
#define   CROSS0_MASK						  GENMASK(4, 0)
#define   CROSS0_SHIFT						  0
#define   CROSS0_F						  BIT(5)
#define   CROSS0_I						  BIT(6)
#define MAX96752_VRX_CROSS_1					  0x01b1
#define   CROSS1_MASK						  GENMASK(4, 0)
#define   CROSS1_SHIFT						  0
#define   CROSS1_F						  BIT(5)
#define   CROSS1_I						  BIT(6)
#define MAX96752_VRX_CROSS_2					  0x01b2
#define   CROSS2_MASK						  GENMASK(4, 0)
#define   CROSS2_SHIFT						  0
#define   CROSS2_F						  BIT(5)
#define   CROSS2_I						  BIT(6)
#define MAX96752_VRX_CROSS_3					  0x01b3
#define   CROSS3_MASK						  GENMASK(4, 0)
#define   CROSS3_SHIFT						  0
#define   CROSS3_F						  BIT(5)
#define   CROSS3_I						  BIT(6)
#define MAX96752_VRX_CROSS_4					  0x01b4
#define   CROSS4_MASK						  GENMASK(4, 0)
#define   CROSS4_SHIFT						  0
#define   CROSS4_F						  BIT(5)
#define   CROSS4_I						  BIT(6)
#define MAX96752_VRX_CROSS_5					  0x01b5
#define   CROSS5_MASK						  GENMASK(4, 0)
#define   CROSS5_SHIFT						  0
#define   CROSS5_F						  BIT(5)
#define   CROSS5_I						  BIT(6)
#define MAX96752_VRX_CROSS_6					  0x01b6
#define   CROSS6_MASK						  GENMASK(4, 0)
#define   CROSS6_SHIFT						  0
#define   CROSS6_F						  BIT(5)
#define   CROSS6_I						  BIT(6)
#define MAX96752_VRX_CROSS_7					  0x01b7
#define   CROSS7_MASK						  GENMASK(4, 0)
#define   CROSS7_SHIFT						  0
#define   CROSS7_F						  BIT(5)
#define   CROSS7_I						  BIT(6)
#define MAX96752_VRX_CROSS_8					  0x01b8
#define   CROSS8_MASK						  GENMASK(4, 0)
#define   CROSS8_SHIFT						  0
#define   CROSS8_F						  BIT(5)
#define   CROSS8_I						  BIT(6)
#define MAX96752_VRX_CROSS_9					  0x01b9
#define   CROSS9_MASK						  GENMASK(4, 0)
#define   CROSS9_SHIFT						  0
#define   CROSS9_F						  BIT(5)
#define   CROSS9_I						  BIT(6)
#define MAX96752_VRX_CROSS_10					  0x01ba
#define   CROSS10_MASK						  GENMASK(4, 0)
#define   CROSS10_SHIFT						  0
#define   CROSS10_F						  BIT(5)
#define   CROSS10_I						  BIT(6)
#define MAX96752_VRX_CROSS_11					  0x01bb
#define   CROSS11_MASK						  GENMASK(4, 0)
#define   CROSS11_SHIFT						  0
#define   CROSS11_F						  BIT(5)
#define   CROSS11_I						  BIT(6)
#define MAX96752_VRX_CROSS_12					  0x01bc
#define   CROSS12_MASK						  GENMASK(4, 0)
#define   CROSS12_SHIFT						  0
#define   CROSS12_F						  BIT(5)
#define   CROSS12_I						  BIT(6)
#define MAX96752_VRX_CROSS_13					  0x01bd
#define   CROSS13_MASK						  GENMASK(4, 0)
#define   CROSS13_SHIFT						  0
#define   CROSS13_F						  BIT(5)
#define   CROSS13_I						  BIT(6)
#define MAX96752_VRX_CROSS_14					  0x01be
#define   CROSS14_MASK						  GENMASK(4, 0)
#define   CROSS14_SHIFT						  0
#define   CROSS14_F						  BIT(5)
#define   CROSS14_I						  BIT(6)
#define MAX96752_VRX_CROSS_15					  0x01bf
#define   CROSS15_MASK						  GENMASK(4, 0)
#define   CROSS15_SHIFT						  0
#define   CROSS15_F						  BIT(5)
#define   CROSS15_I						  BIT(6)
#define MAX96752_VRX_CROSS_16					  0x01c0
#define   CROSS16_MASK						  GENMASK(4, 0)
#define   CROSS16_SHIFT						  0
#define   CROSS16_F						  BIT(5)
#define   CROSS16_I						  BIT(6)
#define MAX96752_VRX_CROSS_17					  0x01c1
#define   CROSS17_MASK						  GENMASK(4, 0)
#define   CROSS17_SHIFT						  0
#define   CROSS17_F						  BIT(5)
#define   CROSS17_I						  BIT(6)
#define MAX96752_VRX_CROSS_18					  0x01c2
#define   CROSS18_MASK						  GENMASK(4, 0)
#define   CROSS18_SHIFT						  0
#define   CROSS18_F						  BIT(5)
#define   CROSS18_I						  BIT(6)
#define MAX96752_VRX_CROSS_19					  0x01c3
#define   CROSS19_MASK						  GENMASK(4, 0)
#define   CROSS19_SHIFT						  0
#define   CROSS19_F						  BIT(5)
#define   CROSS19_I						  BIT(6)
#define MAX96752_VRX_CROSS_20					  0x01c4
#define   CROSS20_MASK						  GENMASK(4, 0)
#define   CROSS20_SHIFT						  0
#define   CROSS20_F						  BIT(5)
#define   CROSS20_I						  BIT(6)
#define MAX96752_VRX_CROSS_21					  0x01c5
#define   CROSS21_MASK						  GENMASK(4, 0)
#define   CROSS21_SHIFT						  0
#define   CROSS21_F						  BIT(5)
#define   CROSS21_I						  BIT(6)
#define MAX96752_VRX_CROSS_22					  0x01c6
#define   CROSS22_MASK						  GENMASK(4, 0)
#define   CROSS22_SHIFT						  0
#define   CROSS22_F						  BIT(5)
#define   CROSS22_I						  BIT(6)
#define MAX96752_VRX_CROSS_23					  0x01c7
#define   CROSS23_MASK						  GENMASK(4, 0)
#define   CROSS23_SHIFT						  0
#define   CROSS23_F						  BIT(5)
#define   CROSS23_I						  BIT(6)
#define MAX96752_VRX_CROSS_24					  0x01c8
#define   CROSS24_MASK						  GENMASK(4, 0)
#define   CROSS24_SHIFT						  0
#define   CROSS24_F						  BIT(5)
#define   CROSS24_I						  BIT(6)
#define MAX96752_VRX_CROSS_25					  0x01c9
#define   CROSS25_MASK						  GENMASK(4, 0)
#define   CROSS25_SHIFT						  0
#define   CROSS25_F						  BIT(5)
#define   CROSS25_I						  BIT(6)
#define MAX96752_VRX_CROSS_26					  0x01ca
#define   CROSS26_MASK						  GENMASK(4, 0)
#define   CROSS26_SHIFT						  0
#define   CROSS26_F						  BIT(5)
#define   CROSS26_I						  BIT(6)
#define MAX96752_VRX_CROSS_27					  0x01cb
#define   CROSS27_MASK						  GENMASK(4, 0)
#define   CROSS27_SHIFT						  0
#define   CROSS27_F						  BIT(5)
#define   CROSS27_I						  BIT(6)
#define MAX96752_VRX_VPRBS_ERR					  0x01cc
#define MAX96752_VRX_OLDI0					  0x01cd
#define   LUT_A_EN						  BIT(0)
#define   LUT_B_EN						  BIT(1)
#define   LUT_C_EN						  BIT(2)
#define   HS_OUT_EN						  BIT(3)
#define   VPRBS_CHK_EN						  BIT(4)
#define   VPRBS_FAIL						  BIT(5)
#define MAX96752_VRX_OLDI1					  0x01ce
#define   OLDI_SPL_POL						  BIT(0)
#define   OLDI_SPL_MODE_MASK					  GENMASK(2, 1)
#define   OLDI_SPL_MODE_SHIFT					  1
#define   OLDI_SPL_EN						  BIT(3)
#define   OLDI_SWAP_AB						  BIT(4)
#define   OLDI_4TH_LANE						  BIT(5)
#define   OLDI_FORMAT						  BIT(6)
#define   OLDI_OUTSEL						  BIT(7)
#define MAX96752_VRX_OLDI2					  0x01cf
#define   SSEN							  BIT(0)
#define   OLDI_DUP						  BIT(1)
#define   VS_OUT_EN						  BIT(4)
#define   PD_LVDS_A						  BIT(6)
#define   PD_LVDS_B						  BIT(7)
#define MAX96752_VRX_OLDI3					  0x01d0
#define   LANE_SEL_A0_MASK					  GENMASK(2, 0)
#define   LANE_SEL_A0_SHIFT					  0
#define   LANE_INV_A0						  BIT(3)
#define   LANE_SEL_B0_MASK					  GENMASK(6, 4)
#define   LANE_SEL_B0_SHIFT					  4
#define   LANE_INV_B0						  BIT(7)
#define MAX96752_VRX_OLDI4					  0x01d1
#define   LANE_SEL_A1_MASK					  GENMASK(2, 0)
#define   LANE_SEL_A1_SHIFT					  0
#define   LANE_INV_A1						  BIT(3)
#define   LANE_SEL_B1_MASK					  GENMASK(6, 4)
#define   LANE_SEL_B1_SHIFT					  4
#define   LANE_INV_B1						  BIT(7)
#define MAX96752_VRX_OLDI5					  0x01d2
#define   LANE_SEL_A2_MASK					  GENMASK(2, 0)
#define   LANE_SEL_A2_SHIFT					  0
#define   LANE_INV_A2						  BIT(3)
#define   LANE_SEL_B2_MASK					  GENMASK(6, 4)
#define   LANE_SEL_B2_SHIFT					  4
#define   LANE_INV_B2						  BIT(7)
#define MAX96752_VRX_OLDI6					  0x01d3
#define   LANE_SEL_A3_MASK					  GENMASK(2, 0)
#define   LANE_SEL_A3_SHIFT					  0
#define   LANE_INV_A3						  BIT(3)
#define   LANE_SEL_B3_MASK					  GENMASK(6, 4)
#define   LANE_SEL_B3_SHIFT					  4
#define   LANE_INV_B3						  BIT(7)
#define MAX96752_VRX_OLDI7					  0x01d4
#define   LANE_SEL_ACLK_MASK					  GENMASK(2, 0)
#define   LANE_SEL_ACLK_SHIFT					  0
#define   LANE_INV_ACLK						  BIT(3)
#define   LANE_SEL_BCLK_MASK					  GENMASK(6, 4)
#define   LANE_SEL_BCLK_SHIFT					  4
#define   LANE_INV_BCLK						  BIT(7)
#define MAX96752_VRX_VRX46					  0x01ea
#define   DUAL_OLDI_AUTO_RST_ALIGN				  BIT(0)
#define   DUAL_OLDI_AUTO_RST_ALIGNED				  BIT(1)

/* GPIO: 0 <= n < 16 */
#define MAX96752_GPIO_A(n)					  (0x0200 + (n) * 3)
#define   GPIO_OUT_DIS						  BIT(0)
#define   GPIO_TX_EN						  BIT(1)
#define   GPIO_RX_EN						  BIT(2)
#define   GPIO_IN						  BIT(3)
#define   GPIO_OUT						  BIT(4)
#define   TX_COMP_EN						  BIT(5)
#define   RES_CFG						  BIT(7)
#define MAX96752_GPIO_B(n)					  (0x0201 + (n) * 3)
#define   GPIO_TX_ID_MASK					  GENMASK(4, 0)
#define   GPIO_TX_ID_SHIFT					  0
#define   OUT_TYPE						  BIT(5)
#define   PULL_UPDN_SEL_MASK					  GENMASK(7, 6)
#define   PULL_UPDN_SEL_SHIFT					  6
#define MAX96752_GPIO_C(n)					  (0x0202 + (n) * 3)
#define   GPIO_RX_ID_MASK					  GENMASK(4, 0)
#define   GPIO_RX_ID_SHIFT					  0
#define   OVR_RES_CFG						  BIT(7)

/* CMU */
#define MAX96752_CMU_CMU4					  0x0244
#define   GPIO_SPEED_A_MASK					  GENMASK(1, 0)
#define   GPIO_SPEED_A_SHIFT					  0
#define   GPIO_SPEED_B_MASK					  GENMASK(3, 2)
#define   GPIO_SPEED_B_SHIFT					  2

/* MISC */
#define MAX96752_MISC_BITLEN_PT_1_L				  0x024a
#define MAX96752_MISC_UART_PT_1					  0x024b
#define   BITLEN_PT_1_H_MASK					  GENMASK(5, 0)
#define   BITLEN_PT_1_H_SHIFT					  0
#define MAX96752_MISC_BITLEN_PT_2_L				  0x024c
#define MAX96752_MISC_UART_PT_3					  0x024d
#define   BITLEN_PT_2_H_MASK					  GENMASK(5, 0)
#define   BITLEN_PT_2_H_SHIFT					  0
#define MAX96752_MISC_I2C_PT_4					  0x0252
#define   SRC_A_1_MASK						  GENMASK(7, 1)
#define   SRC_A_1_SHIFT						  1
#define MAX96752_MISC_I2C_PT_5					  0x0253
#define   DST_A_1_MASK						  GENMASK(7, 1)
#define   DST_A_1_SHIFT						  1
#define MAX96752_MISC_I2C_PT_6					  0x0254
#define   SRC_B_1_MASK						  GENMASK(7, 1)
#define   SRC_B_1_SHIFT						  1
#define MAX96752_MISC_I2C_PT_7					  0x0255
#define   DST_B_1_MASK						  GENMASK(7, 1)
#define   DST_B_1_SHIFT						  1
#define MAX96752_MISC_I2C_PT_8					  0x0256
#define   SRC_A_2_MASK						  GENMASK(7, 1)
#define   SRC_A_2_SHIFT						  1
#define MAX96752_MISC_I2C_PT_9					  0x0257
#define   DST_A_2_MASK						  GENMASK(7, 1)
#define   DST_A_2_SHIFT						  1
#define MAX96752_MISC_I2C_PT_10					  0x0258
#define   SRC_B_2_MASK						  GENMASK(7, 1)
#define   SRC_B_2_SHIFT						  1
#define MAX96752_MISC_I2C_PT_11					  0x0259
#define   DST_B_2_MASK						  GENMASK(7, 1)
#define   DST_B_2_SHIFT						  1
#define MAX96752_MISC_PM_OV_STAT				  0x025d
#define   OV_LEVEL_MASK						  GENMASK(1, 0)
#define   OV_LEVEL_SHIFT					  0
#define MAX96752_MISC_LOSS_INT_OEN				  0x02d2
#define   LOSS_OF_VIDEO_LOCK_OEN				  BIT(0)
#define   LOSS_OF_LOCK_OEN					  BIT(1)
#define MAX96752_MISC_LOSS_INT_FLAG				  0x02d3
#define   LOSS_OF_VIDEO_LOCK_FLAG				  BIT(0)
#define   LOSS_OF_LOCK_FLAG					  BIT(1)

/* RLMS: links A -> 0, B -> 1 */
#define MAX96752_RLMS_3(l)					  (0x0403 + (l) * 0x100)
#define   ADAPTEN						  BIT(7)
#define MAX96752_RLMS_4(l)					  (0x0404 + (l) * 0x100)
#define   EOM_EN						  BIT(0)
#define   RSVD_EOM_PER_MODE					  BIT(1)
#define   EOM_CHK_THR_MASK					  GENMASK(3, 2)
#define   EOM_CHK_THR_SHIFT					  2
#define   RSVD_EOM_CHK_AMOUNT_MASK				  GENMASK(7, 4)
#define   RSVD_EOM_CHK_AMOUNT_SHIFT				  4
#define MAX96752_RLMS_5(l)					  (0x0405 + (l) * 0x100)
#define   EOM_MIN_THR_MASK					  GENMASK(6, 0)
#define   EOM_MIN_THR_SHIFT					  0
#define   EOM_MAN_TRG_REQ					  BIT(7)
#define MAX96752_RLMS_6(l)					  (0x0406 + (l) * 0x100)
#define   EOM_RST_THR_MASK					  GENMASK(6, 0)
#define   EOM_RST_THR_SHIFT					  0
#define   EOM_PV_MODE						  BIT(7)
#define MAX96752_RLMS_7(l)					  (0x0407 + (l) * 0x100)
#define   EOM_MASK						  GENMASK(6, 0)
#define   EOM_SHIFT						  0
#define   EOM_DONE						  BIT(7)
#define MAX96752_RLMS_EYEMONPERCNTL(l)				  (0x0434 + (l) * 0x100)
#define MAX96752_RLMS_EYEMONPERCNTH(l)				  (0x0435 + (l) * 0x100)
#define MAX96752_RLMS_37(l)					  (0x0437 + (l) * 0x100)
#define   EYEMONDPOL						  BIT(0)
#define   EYEMONPH						  BIT(1)
#define   EYEMONSTART						  BIT(2)
#define   EYEMONCNTCLR						  BIT(3)
#define   EYEMONDONE						  BIT(4)
#define MAX96752_RLMS_EYEMONERRCNTL(l)				  (0x0438 + (l) * 0x100)
#define MAX96752_RLMS_EYEMONERRCNTH(l)				  (0x0439 + (l) * 0x100)
#define MAX96752_RLMS_EYEMONVALCNTL(l)				  (0x043a + (l) * 0x100)
#define MAX96752_RLMS_EYEMONVALCNTH(l)				  (0x043b + (l) * 0x100)
#define MAX96752_RLMS_3D(l)					  (0x043d + (l) * 0x100)
#define   ERRCHPHTOGEN						  BIT(0)
#define   ERRCHPH_MASK						  GENMASK(7, 1)
#define   ERRCHPH_SHIFT						  1
#define MAX96752_RLMS_3E(l)					  (0x043e + (l) * 0x100)
#define   ERRCHPHSEC_MASK					  GENMASK(6, 0)
#define   ERRCHPHSEC_SHIFT					  0
#define   ERRCHPHSECTA						  BIT(7)
#define MAX96752_RLMS_3F(l)					  (0x043f + (l) * 0x100)
#define   ERRCHPHPRI_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRI_SHIFT					  0
#define   ERRCHPHPRITA						  BIT(7)
#define MAX96752_RLMS_49(l)					  (0x0449 + (l) * 0x100)
#define   ERRCHPWRUP						  BIT(2)
#define MAX96752_RLMS_58(l)					  (0x0458 + (l) * 0x100)
#define   ERRCHVTH1_MASK					  GENMASK(6, 0)
#define   ERRCHVTH1_SHIFT					  0
#define MAX96752_RLMS_59(l)					  (0x0459 + (l) * 0x100)
#define   ERRCHVTH0_MASK					  GENMASK(6, 0)
#define   ERRCHVTH0_SHIFT					  0
#define MAX96752_RLMS_64(l)					  (0x0464 + (l) * 0x100)
#define   TXSSCMODE_MASK					  GENMASK(1, 0)
#define   TXSSCMODE_SHIFT					  0
#define MAX96752_RLMS_70(l)					  (0x0470 + (l) * 0x100)
#define   TXSSCFRQCTRL_MASK					  GENMASK(6, 0)
#define   TXSSCFRQCTRL_SHIFT					  0
#define MAX96752_RLMS_71(l)					  (0x0471 + (l) * 0x100)
#define   TXSSCEN						  BIT(0)
#define   TXSSCCENSPRST_MASK					  GENMASK(6, 1)
#define   TXSSCCENSPRST_SHIFT					  1
#define MAX96752_RLMS_TXSSCPRESCLL(l)				  (0x0472 + (l) * 0x100)
#define MAX96752_RLMS_73(l)					  (0x0473 + (l) * 0x100)
#define   TXSSCPRESCLH_MASK					  GENMASK(2, 0)
#define   TXSSCPRESCLH_SHIFT					  0
#define MAX96752_RLMS_TXSSCPHL(l)				  (0x0474 + (l) * 0x100)
#define MAX96752_RLMS_75(l)					  (0x0475 + (l) * 0x100)
#define   TXSSCPHH_MASK						  GENMASK(6, 0)
#define   TXSSCPHH_SHIFT					  0
#define MAX96752_RLMS_76(l)					  (0x0476 + (l) * 0x100)
#define   TXSSCPHQUAD_MASK					  GENMASK(1, 0)
#define   TXSSCPHQUAD_SHIFT					  0
#define MAX96752_RLMS_95(l)					  (0x0495 + (l) * 0x100)
#define   TXAMPLMAN_MASK					  GENMASK(5, 0)
#define   TXAMPLMAN_SHIFT					  0
#define   TXAMPLMANEN						  BIT(7)
#define MAX96752_RLMS_A4(l)					  (0x04a4 + (l) * 0x100)
#define   RSVD_AEQ_PER_MASK					  GENMASK(5, 0)
#define   RSVD_AEQ_PER_SHIFT					  0
#define   RSVD_AEQ_PER_MULT_MASK				  GENMASK(7, 6)
#define   RSVD_AEQ_PER_MULT_SHIFT				  6
#define MAX96752_RLMS_AC(l)					  (0x04ac + (l) * 0x100)
#define   ERRCHPHSECFR3G_MASK					  GENMASK(6, 0)
#define   ERRCHPHSECFR3G_SHIFT					  0
#define   ERRCHPHSECTAFR3G					  BIT(7)
#define MAX96752_RLMS_AD(l)					  (0x04ad + (l) * 0x100)
#define   ERRCHPHPRIFR3G_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRIFR3G_SHIFT					  0
#define   ERRCHPHPRITAFR3G					  BIT(7)
#define MAX96752_RLMS_AE(l)					  (0x04ae + (l) * 0x100)
#define   ERRCHPHSECFR1G5_MASK					  GENMASK(6, 0)
#define   ERRCHPHSECFR1G5_SHIFT					  0
#define   ERRCHPHSECTAFR1G5					  BIT(7)
#define MAX96752_RLMS_AF(l)					  (0x04af + (l) * 0x100)
#define   ERRCHPHPRIFR1G5_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRIFR1G5_SHIFT					  0
#define   ERRCHPHPRITAFR1G5					  BIT(7)
#define MAX96752_RLMS_B0(l)					  (0x04b0 + (l) * 0x100)
#define   ERRCHPHSECSR1G5_MASK					  GENMASK(6, 0)
#define   ERRCHPHSECSR1G5_SHIFT					  0
#define   ERRCHPHSECTASR1G5					  BIT(7)
#define MAX96752_RLMS_B1(l)					  (0x04b1 + (l) * 0x100)
#define   ERRCHPHPRISR1G5_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRISR1G5_SHIFT					  0
#define   ERRCHPHPRITASR1G5					  BIT(7)
#define MAX96752_RLMS_B2(l)					  (0x04b2 + (l) * 0x100)
#define   ERRCHPHSECSRG75_MASK					  GENMASK(6, 0)
#define   ERRCHPHSECSRG75_SHIFT					  0
#define   ERRCHPHSECTASRG75					  BIT(7)
#define MAX96752_RLMS_B3(l)					  (0x04b3 + (l) * 0x100)
#define   ERRCHPHPRISRG75_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRISRG75_SHIFT					  0
#define   ERRCHPHPRITASRG75					  BIT(7)
#define MAX96752_RLMS_B4(l)					  (0x04b4 + (l) * 0x100)
#define   ERRCHPHSECSRG375_MASK					  GENMASK(6, 0)
#define   ERRCHPHSECSRG375_SHIFT				  0
#define   ERRCHPHSECTASRG375					  BIT(7)
#define MAX96752_RLMS_B5(l)					  (0x04b5 + (l) * 0x100)
#define   ERRCHPHPRISRG375_MASK					  GENMASK(6, 0)
#define   ERRCHPHPRISRG375_SHIFT				  0
#define   ERRCHPHPRITASRG375					  BIT(7)
#define MAX96752_RLMS_B6(l)					  (0x04b6 + (l) * 0x100)
#define   ERRCHPHSECSRG1875_MASK				  GENMASK(6, 0)
#define   ERRCHPHSECSRG1875_SHIFT				  0
#define   ERRCHPHSECTASRG1875					  BIT(7)
#define MAX96752_RLMS_B7(l)					  (0x04b7 + (l) * 0x100)
#define   ERRCHPHPRISRG1875_MASK				  GENMASK(6, 0)
#define   ERRCHPHPRISRG1875_SHIFT				  0
#define   ERRCHPHPRITASRG1875					  BIT(7)

/* DPLL_AUD */
#define MAX96752_DPLL_AUD_DPLL_0				  0x0b00
#define   AUD_CONFIG_SOFT_RST_N					  BIT(0)
#define MAX96752_DPLL_AUD_DPLL_3				  0x0b03
#define   AUD_CONFIG_SPREAD_BIT_RATIO_MASK			  GENMASK(2, 0)
#define   AUD_CONFIG_SPREAD_BIT_RATIO_SHIFT			  0

/* DPLL_OLDI */
#define MAX96752_DPLL_OLDI_DPLL_0				  0x0d00
#define   OLDI_CONFIG_SOFT_RST_N				  BIT(0)
#define MAX96752_DPLL_OLDI_DPLL_3				  0x0d03
#define   OLDI_CONFIG_SPREAD_BIT_RATIO_MASK			  GENMASK(2, 0)
#define   OLDI_CONFIG_SPREAD_BIT_RATIO_SHIFT			  0

/* COLOR_LUTs */
#define MAX96752_COLOR_A_LUT					  0x1000
#define MAX96752_COLOR_B_LUT					  0x1100
#define MAX96752_COLOR_C_LUT					  0x1200
#define   COLOR_LUT_SIZE					  256

extern const struct regmap_config max96752_regmap_cfg;

enum max96752_dev_id {
	ID_MAX96752 = 0x82,
};

enum max96752_gmsl2_link_speed {
	GMSL2_SPEED_3G = 1,
	GMSL2_SPEED_6G = 2,
};

struct max96752 {
	struct regmap *regmap;
	struct device *dev;

	unsigned int id;
	unsigned int rev;

	enum max96752_gmsl2_link_speed gmsl2_link_speed;
	bool gmsl2_dual_link;

	int link_id;
	bool link_setup_finished;
};

int max96752_dev_init(struct max96752 *max96752);
int max96752_dev_check(struct max96752 *max96752, ulong expected_dev_id);

#endif /* _MAX96752_REGS_H_ */
