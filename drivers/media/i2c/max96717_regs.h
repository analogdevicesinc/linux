// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 NXP
 */

#ifndef _MAX96717_REGS_H_
#define _MAX96717_REGS_H_

/* DEV */
#define MAX96717_DEV_REG0				  0x0000
#define   CFG_BLOCK					  BIT(0)
#define   DEV_ADDR_MASK					  GENMASK(7, 1)
#define   DEV_ADDR_SHIFT				  1
#define MAX96717_DEV_REG1				  0x0001
#define   RX_RATE_MASK					  GENMASK(1, 0)
#define   RX_RATE_SHIFT					  0
#define   TX_RATE_MASK					  GENMASK(3, 2)
#define   TX_RATE_SHIFT					  2
#define   DIS_REM_CC					  BIT(4)
#define   DIS_LOCAL_CC					  BIT(5)
#define   IIC_1_EN					  BIT(6)
#define   IIC_2_EN					  BIT(7)
#define MAX96717_DEV_REG2				  0x0002
#define   VID_TX_EN_Z					  BIT(6)
#define MAX96717_DEV_REG3				  0x0003
#define   RCLKSEL_MASK					  GENMASK(1, 0)
#define   RCLKSEL_SHIFT					  0
#define   RCLK_ALT					  BIT(2)
#define   UART_1_EN					  BIT(4)
#define   UART_2_EN					  BIT(5)
#define MAX96717_DEV_REG4				  0x0004
#define   XTAL_PU					  BIT(0)
#define   CC_CRC_MSGCNTR_OVR				  BIT(2)
#define   CC_CRC_EN					  BIT(3)
#define   CC_MSGCNTR_EN					  BIT(4)
#define MAX96717_DEV_REG5				  0x0005
#define   PU_LF0					  BIT(0)
#define   PU_LF1					  BIT(1)
#define   ALT_ERRB_EN					  BIT(4)
#define   ALT_LOCK_EN					  BIT(5)
#define   ERRB_EN					  BIT(6)
#define   LOCK_EN					  BIT(7)
#define MAX96717_DEV_REG6				  0x0006
#define   I2CSEL					  BIT(4)
#define   RCLKEN					  BIT(5)
#define MAX96717_DEV_REG13				  0x000d
#define MAX96717_DEV_REG14				  0x000e
#define   DEV_REV_MASK					  GENMASK(3, 0)
#define   DEV_REV_SHIFT					  0
#define MAX96717_DEV_REG26				  0x0026
#define   LF_0_MASK					  GENMASK(2, 0)
#define   LF_0_SHIFT					  0
#define   LF_1_MASK					  GENMASK(6, 4)
#define   LF_1_SHIFT					  4

/* TCTRL */
#define MAX96717_TCTRL_PWR0				  0x0008
#define   CMP_STATUS_MASK				  GENMASK(4, 0)
#define   CMP_STATUS_SHIFT				  0
#define   VDDBAD_STATUS_MASK				  GENMASK(7, 5)
#define   VDDBAD_STATUS_SHIFT				  5
#define MAX96717_TCTRL_PWR4				  0x000c
#define   WAKE_EN_A					  BIT(4)
#define   DIS_LOCAL_WAKE				  BIT(6)
#define MAX96717_TCTRL_CTRL0				  0x0010
#define   SLEEP						  BIT(3)
#define   RESET_ONESHOT					  BIT(5)
#define   RESET_LINK					  BIT(6)
#define   RESET_ALL					  BIT(7)
#define MAX96717_TCTRL_CTRL1				  0x0011
#define   CXTP_A					  BIT(0)
#define MAX96717_TCTRL_CTRL2				  0x0012
#define   LDO_BYPASS					  BIT(4)
#define MAX96717_TCTRL_CTRL3				  0x0013
#define   CMU_LOCKED					  BIT(1)
#define   ERROR						  BIT(2)
#define   LOCKED					  BIT(3)
#define MAX96717_TCTRL_INTR0				  0x0018
#define   DEC_ERR_THR_MASK				  GENMASK(2, 0)
#define   DEC_ERR_THR_SHIFT				  0
#define   AUTO_ERR_RST_EN				  BIT(3)
#define MAX96717_TCTRL_INTR1				  0x0019
#define   AUTO_CNT_RST_EN				  BIT(3)
#define   PKT_CNT_EXP_MASK				  GENMASK(7, 4)
#define   PKT_CNT_EXP_SHIFT				  4
#define MAX96717_TCTRL_INTR2				  0x001a
#define   DEC_ERR_OEN_A					  BIT(0)
#define   IDLE_ERR_OEN					  BIT(2)
#define   LFLT_INT_OEN					  BIT(3)
#define   REM_ERR_OEN					  BIT(5)
#define   REFGEN_UNLOCKED_OEN				  BIT(7)
#define MAX96717_TCTRL_INTR3				  0x001b
#define   DEC_ERR_FLAG_A				  BIT(0)
#define   IDLE_ERR_FLAG					  BIT(2)
#define   LFLT_INT					  BIT(3)
#define   REM_ERR_FLAG					  BIT(5)
#define   REFGEN_UNLOCKED				  BIT(7)
#define MAX96717_TCTRL_INTR4				  0x001c
#define   PKT_CNT_OEN					  BIT(1)
#define   RT_CNT_OEN					  BIT(2)
#define   MAX_RT_OEN					  BIT(3)
#define   VDD18_OV_OEN					  BIT(4)
#define   VDD_OV_OEN					  BIT(5)
#define   EOM_ERR_OEN_A					  BIT(6)
#define   VREG_OV_OEN					  BIT(7)
#define MAX96717_TCTRL_INTR5				  0x001d
#define   PKT_CNT_FLAG					  BIT(1)
#define   RT_CNT_FLAG					  BIT(2)
#define   MAX_RT_FLAG					  BIT(3)
#define   VDD18_OV_FLAG					  BIT(4)
#define   VDD_OV_FLAG					  BIT(5)
#define   EOM_ERR_FLAG_A				  BIT(6)
#define   VREG_OV_FLAG					  BIT(7)
#define MAX96717_TCTRL_INTR6				  0x001e
#define   MIPI_ERR_OEN					  BIT(0)
#define   ADC_INT_OEN					  BIT(2)
#define   RTTN_CRC_ERR_OEN				  BIT(3)
#define   EFUSE_CRC_ERR_OEN				  BIT(4)
#define   VDDBAD_INT_OEN				  BIT(5)
#define   PORZ_INT_OEN					  BIT(6)
#define   VDDCMP_INT_OEN				  BIT(7)
#define MAX96717_TCTRL_INTR7				  0x001f
#define   MIPI_ERR_FLAG					  BIT(0)
#define   ADC_INT_FLAG					  BIT(2)
#define   RTTN_CRC_INT					  BIT(3)
#define   EFUSE_CRC_ERR					  BIT(4)
#define   VDDBAD_INT_FLAG				  BIT(5)
#define   PORZ_INT_FLAG					  BIT(6)
#define   VDDCMP_INT_FLAG				  BIT(7)
#define MAX96717_TCTRL_INTR8				  0x0020
#define   ERR_TX_ID_MASK				  GENMASK(4, 0)
#define   ERR_TX_ID_SHIFT				  0
#define   ERR_TX_EN					  BIT(7)
#define MAX96717_TCTRL_INTR9				  0x0021
#define   ERR_RX_ID_MASK				  GENMASK(4, 0)
#define   ERR_RX_ID_SHIFT				  0
#define   ERR_RX_EN					  BIT(7)
#define MAX96717_TCTRL_DEC_ERR_A			  0x0022
#define MAX96717_TCTRL_IDLE_ERR				  0x0024
#define MAX96717_TCTRL_PKT_CNT				  0x0025

/* GMSL */
#define MAX96717_GMSL_TX0				  0x0028
#define   TX_FEC_EN					  BIT(1)
#define MAX96717_GMSL_TX1				  0x0029
#define   DIS_ENC					  BIT(0)
#define   DIS_SCR					  BIT(1)
#define   TX_FEC_CRC_EN					  BIT(3)
#define   ERRG_EN_A					  BIT(4)
#define   LINK_PRBS_GEN					  BIT(7)
#define MAX96717_GMSL_TX2				  0x002a
#define   ERRG_PER					  BIT(0)
#define   ERRG_BURST_MASK				  GENMASK(3, 1)
#define   ERRG_BURST_SHIFT				  1
#define   ERRG_RATE_MASK				  GENMASK(5, 4)
#define   ERRG_RATE_SHIFT				  4
#define   ERRG_CNT_MASK					  GENMASK(7, 6)
#define   ERRG_CNT_SHIFT				  6
#define MAX96717_GMSL_TX3				  0x002b
#define   TX_FEC_ACTIVE					  BIT(5)
#define MAX96717_GMSL_RX0				  0x002c
#define   PKT_CNT_SEL_MASK				  GENMASK(3, 0)
#define   PKT_CNT_SEL_SHIFT				  0
#define   PKT_CNT_LBW_MASK				  GENMASK(7, 6)
#define   PKT_CNT_LBW_SHIFT				  6
#define MAX96717_GMSL_RX1				  0x002d
#define   LINK_PRBS_CHK					  BIT(7)
#define MAX96717_GMSL_GPIOA				  0x0030
#define   GPIO_FWD_CDLY_MASK				  GENMASK(5, 0)
#define   GPIO_FWD_CDLY_SHIFT				  0
#define MAX96717_GMSL_GPIOB				  0x0031
#define   GPIO_REV_CDLY_MASK				  GENMASK(5, 0)
#define   GPIO_REV_CDLY_SHIFT				  0

/* CC */
#define MAX96717_CC_I2C_0				  0x0040
#define   SLV_TO_MASK					  GENMASK(2, 0)
#define   SLV_TO_SHIFT					  0
#define   SLV_SH_MASK					  GENMASK(5, 4)
#define   SLV_SH_SHIFT					  4
#define MAX96717_CC_I2C_1				  0x0041
#define   MST_TO_MASK					  GENMASK(2, 0)
#define   MST_TO_SHIFT					  0
#define   MST_BT_MASK					  GENMASK(6, 4)
#define   MST_BT_SHIFT					  4
#define MAX96717_CC_I2C_2				  0x0042
#define   SRC_A_MASK					  GENMASK(7, 1)
#define   SRC_A_SHIFT					  1
#define MAX96717_CC_I2C_3				  0x0043
#define   DST_A_MASK					  GENMASK(7, 1)
#define   DST_A_SHIFT					  1
#define MAX96717_CC_I2C_4				  0x0044
#define   SRC_B_MASK					  GENMASK(7, 1)
#define   SRC_B_SHIFT					  1
#define MAX96717_CC_I2C_5				  0x0045
#define   DST_B_MASK					  GENMASK(7, 1)
#define   DST_B_SHIFT					  1
#define MAX96717_CC_UART_0				  0x0048
#define   BYPASS_EN					  BIT(0)
#define   BYPASS_TO_MASK				  GENMASK(2, 1)
#define   BYPASS_TO_SHIFT				  1
#define   BYPASS_DIS_PAR				  BIT(3)
#define   LOC_MS_EN					  BIT(4)
#define   REM_MS_EN					  BIT(5)
#define MAX96717_CC_I2C_PT_0				  0x004c
#define   SLV_TO_PT_MASK				  GENMASK(2, 0)
#define   SLV_TO_PT_SHIFT				  0
#define   SLV_SH_PT_MASK				  GENMASK(5, 4)
#define   SLV_SH_PT_SHIFT				  4
#define MAX96717_CC_I2C_PT_1				  0x004d
#define   MST_TO_PT_MASK				  GENMASK(2, 0)
#define   MST_TO_PT_SHIFT				  0
#define   MST_BT_PT_MASK				  GENMASK(6, 4)
#define   MST_BT_PT_SHIFT				  4
#define MAX96717_CC_UART_PT_0				  0x004f
#define   DIS_PAR_1					  BIT(2)
#define   BITLEN_MAN_CFG_1				  BIT(3)
#define   DIS_PAR_2					  BIT(6)
#define   BITLEN_MAN_CFG_2				  BIT(7)

/* CFGV_VIDEO_Z */
#define MAX96717_CFGV_VIDEO_Z_TX0			  0x0058
#define   VIDEO_Z_TX_CRC_EN				  BIT(7)
#define MAX96717_CFGV_VIDEO_Z_TX3			  0x005b
#define   TX_STR_SEL_MASK				  GENMASK(1, 0)
#define   TX_STR_SEL_SHIFT				  0

/* CFGI_INFOFR */
#define MAX96717_CFGI_INFOFR_TR0			  0x0078
#define   RX_CRC_EN					  BIT(6)
#define   TX_CRC_EN					  BIT(7)
#define MAX96717_CFGI_INFOFR_TR3			  0x007b
#define   TX_SRC_ID_MASK				  GENMASK(2, 0)
#define   TX_SRC_ID_SHIFT				  0
#define MAX96717_CFGI_INFOFR_RX_SRC_SEL			  0x007c

/* CFGL_SPI */
#define MAX96717_CFGL_SPI_TR0				  0x0080
#define   SPI_RX_CRC_EN					  BIT(6)
#define   SPI_TX_CRC_EN					  BIT(7)
#define MAX96717_CFGL_SPI_TR3				  0x0083
#define   SPI_TX_SRC_ID_MASK				  GENMASK(2, 0)
#define   SPI_TX_SRC_ID_SHIFT				  0
#define MAX96717_CFGL_SPI_RX_SRC_SEL			  0x0084
#define MAX96717_CFGL_SPI_ARQ0				  0x0085
#define   SPI_DIS_DBL_ACK_RETX				  BIT(2)
#define   SPI_ARQ0_EN					  BIT(3)
#define MAX96717_CFGL_SPI_ARQ1				  0x0086
#define   SPI_RT_CNT_OEN				  BIT(0)
#define   SPI_MAX_RT_ERR_OEN				  BIT(1)
#define MAX96717_CFGL_SPI_ARQ2				  0x0087
#define   SPI_RT_CNT_MASK				  GENMASK(6, 0)
#define   SPI_RT_CNT_SHIFT				  0
#define   SPI_MAX_RT_ERR				  BIT(7)

/* CFGL_GPIO */
#define MAX96717_CFGL_GPIO_TR0				  0x0090
#define   GPIO_RX_CRC_EN				  BIT(6)
#define   GPIO_TX_CRC_EN				  BIT(7)
#define MAX96717_CFGL_GPIO_TR3				  0x0093
#define   GPIO_TX_SRC_ID_MASK				  GENMASK(2, 0)
#define   GPIO_TX_SRC_ID_SHIFT				  0
#define MAX96717_CFGL_GPIO_RX_SRC_SEL			  0x0094
#define MAX96717_CFGL_GPIO_ARQ0				  0x0095
#define   GPIO_DIS_DBL_ACK_RETX				  BIT(2)
#define   GPIO_ARQ0_EN					  BIT(3)
#define MAX96717_CFGL_GPIO_ARQ1				  0x0096
#define   GPIO_RT_CNT_OEN				  BIT(0)
#define   GPIO_MAX_RT_ERR_OEN				  BIT(1)
#define MAX96717_CFGL_GPIO_ARQ2				  0x0097
#define   GPIO_RT_CNT_MASK				  GENMASK(6, 0)
#define   GPIO_RT_CNT_SHIFT				  0
#define   GPIO_MAX_RT_ERR				  BIT(7)

/* CFGL_IIC: 0 <= ch < 2 */
#define MAX96717_CFGL_IIC_TR0(ch)			  (0x00a0 + (ch) * 0x08)
#define   IIC_RX_CRC_EN					  BIT(6)
#define   IIC_TX_CRC_EN					  BIT(7)
#define MAX96717_CFGL_IIC_TR3(ch)			  (0x00a3 + (ch) * 0x08)
#define   IIC_TX_SRC_ID_MASK				  GENMASK(2, 0)
#define   IIC_TX_SRC_ID_SHIFT				  0
#define MAX96717_CFGL_IIC_RX_SRC_SEL(ch)		  (0x00a4 + (ch) * 0x08)
#define MAX96717_CFGL_IIC_ARQ0(ch)			  (0x00a5 + (ch) * 0x08)
#define   IIC_DIS_DBL_ACK_RETX				  BIT(2)
#define   IIC_ARQ0_EN					  BIT(3)
#define MAX96717_CFGL_IIC_ARQ1(ch)			  (0x00a6 + (ch) * 0x08)
#define   IIC_RT_CNT_OEN				  BIT(0)
#define   IIC_MAX_RT_ERR_OEN				  BIT(1)
#define MAX96717_CFGL_IIC_ARQ2(ch)			  (0x00a7 + (ch) * 0x08)
#define   IIC_RT_CNT_MASK				  GENMASK(6, 0)
#define   IIC_RT_CNT_SHIFT				  0
#define   IIC_MAX_RT_ERR				  BIT(7)

/* VID_TX_Z */
#define MAX96717_VID_TX_Z_VIDEO_TX0			  0x0110
#define   CLKDET_BYP					  BIT(2)
#define   AUTO_BPP					  BIT(3)
#define   ENC_MODE_MASK					  GENMASK(5, 4)
#define   ENC_MODE_SHIFT				  4
#define   LINE_CRC_EN					  BIT(6)
#define   LINE_CRC_SEL					  BIT(7)
#define MAX96717_VID_TX_Z_VIDEO_TX1			  0x0111
#define   BPP_MASK					  GENMASK(5, 0)
#define   BPP_SHIFT					  0
#define MAX96717_VID_TX_Z_VIDEO_TX2			  0x0112
#define   LIM_HEART					  BIT(2)
#define   FIFO_WARN					  BIT(4)
#define   OVERFLOW					  BIT(5)
#define   DRIFT_ERR					  BIT(6)
#define   PCLKDET					  BIT(7)

/* SPI */
#define MAX96717_SPI_0					  0x0170
#define   SPI_EN					  BIT(0)
#define   MST_SLVN					  BIT(1)
#define   SPI_CC_EN					  BIT(2)
#define   SPI_IGNR_ID					  BIT(3)
#define   SPI_CC_TRG_ID_MASK				  GENMASK(5, 4)
#define   SPI_CC_TRG_ID_SHIFT				  4
#define   SPI_LOC_ID_MASK				  GENMASK(7, 6)
#define   SPI_LOC_ID_SHIFT				  6
#define MAX96717_SPI_1					  0x0171
#define   SPI_BASE_PRIO_MASK				  GENMASK(1, 0)
#define   SPI_BASE_PRIO_SHIFT				  0
#define   SPI_LOC_N_MASK				  GENMASK(7, 2)
#define   SPI_LOC_N_SHIFT				  2
#define MAX96717_SPI_2					  0x0172
#define   SPIM_SS1_ACT_H				  BIT(0)
#define   SPIM_SS2_ACT_H				  BIT(1)
#define   SPI_MOD3					  BIT(2)
#define   SPI_MOD3_F					  BIT(3)
#define   FULL_SCK_SETUP				  BIT(4)
#define   REQ_HOLD_OFF_MASK				  GENMASK(7, 5)
#define   REQ_HOLD_OFF_SHIFT				  5
#define MAX96717_SPI_SPIM_SS_DLY_CLKS			  0x0173
#define MAX96717_SPI_SPIM_SCK_LO_CLKS			  0x0174
#define MAX96717_SPI_SPIM_SCK_HI_CLKS			  0x0175
#define MAX96717_SPI_6					  0x0176
#define   RWN_IO_EN					  BIT(0)
#define   BNE_IO_EN					  BIT(1)
#define   SS_IO_EN_1					  BIT(2)
#define   SS_IO_EN_2					  BIT(3)
#define   SPIS_RWN					  BIT(4)
#define   BNE						  BIT(5)
#define MAX96717_SPI_7					  0x0177
#define   SPIS_BYTE_CNT_MASK				  GENMASK(4, 0)
#define   SPIS_BYTE_CNT_SHIFT				  0
#define   SPI_TX_OVRFLW					  BIT(6)
#define   SPI_RX_OVRFLW					  BIT(7)
#define MAX96717_SPI_REQ_HOLD_OFF_TO			  0x0178

/* VTX_Z */
#define MAX96717_VTX_Z_CROSS_0				  0x0236
#define   CROSS0_MASK					  GENMASK(4, 0)
#define   CROSS0_SHIFT					  0
#define   CROSS0_F					  BIT(5)
#define   CROSS0_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_1				  0x0237
#define   CROSS1_MASK					  GENMASK(4, 0)
#define   CROSS1_SHIFT					  0
#define   CROSS1_F					  BIT(5)
#define   CROSS1_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_2				  0x0238
#define   CROSS2_MASK					  GENMASK(4, 0)
#define   CROSS2_SHIFT					  0
#define   CROSS2_F					  BIT(5)
#define   CROSS2_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_3				  0x0239
#define   CROSS3_MASK					  GENMASK(4, 0)
#define   CROSS3_SHIFT					  0
#define   CROSS3_F					  BIT(5)
#define   CROSS3_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_4				  0x023a
#define   CROSS4_MASK					  GENMASK(4, 0)
#define   CROSS4_SHIFT					  0
#define   CROSS4_F					  BIT(5)
#define   CROSS4_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_5				  0x023b
#define   CROSS5_MASK					  GENMASK(4, 0)
#define   CROSS5_SHIFT					  0
#define   CROSS5_F					  BIT(5)
#define   CROSS5_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_6				  0x023c
#define   CROSS6_MASK					  GENMASK(4, 0)
#define   CROSS6_SHIFT					  0
#define   CROSS6_F					  BIT(5)
#define   CROSS6_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_7				  0x023d
#define   CROSS7_MASK					  GENMASK(4, 0)
#define   CROSS7_SHIFT					  0
#define   CROSS7_F					  BIT(5)
#define   CROSS7_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_8				  0x023e
#define   CROSS8_MASK					  GENMASK(4, 0)
#define   CROSS8_SHIFT					  0
#define   CROSS8_F					  BIT(5)
#define   CROSS8_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_9				  0x023f
#define   CROSS9_MASK					  GENMASK(4, 0)
#define   CROSS9_SHIFT					  0
#define   CROSS9_F					  BIT(5)
#define   CROSS9_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_10				  0x0240
#define   CROSS10_MASK					  GENMASK(4, 0)
#define   CROSS10_SHIFT					  0
#define   CROSS10_F					  BIT(5)
#define   CROSS10_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_11				  0x0241
#define   CROSS11_MASK					  GENMASK(4, 0)
#define   CROSS11_SHIFT					  0
#define   CROSS11_F					  BIT(5)
#define   CROSS11_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_12				  0x0242
#define   CROSS12_MASK					  GENMASK(4, 0)
#define   CROSS12_SHIFT					  0
#define   CROSS12_F					  BIT(5)
#define   CROSS12_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_13				  0x0243
#define   CROSS13_MASK					  GENMASK(4, 0)
#define   CROSS13_SHIFT					  0
#define   CROSS13_F					  BIT(5)
#define   CROSS13_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_14				  0x0244
#define   CROSS14_MASK					  GENMASK(4, 0)
#define   CROSS14_SHIFT					  0
#define   CROSS14_F					  BIT(5)
#define   CROSS14_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_15				  0x0245
#define   CROSS15_MASK					  GENMASK(4, 0)
#define   CROSS15_SHIFT					  0
#define   CROSS15_F					  BIT(5)
#define   CROSS15_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_16				  0x0246
#define   CROSS16_MASK					  GENMASK(4, 0)
#define   CROSS16_SHIFT					  0
#define   CROSS16_F					  BIT(5)
#define   CROSS16_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_17				  0x0247
#define   CROSS17_MASK					  GENMASK(4, 0)
#define   CROSS17_SHIFT					  0
#define   CROSS17_F					  BIT(5)
#define   CROSS17_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_18				  0x0248
#define   CROSS18_MASK					  GENMASK(4, 0)
#define   CROSS18_SHIFT					  0
#define   CROSS18_F					  BIT(5)
#define   CROSS18_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_19				  0x0249
#define   CROSS19_MASK					  GENMASK(4, 0)
#define   CROSS19_SHIFT					  0
#define   CROSS19_F					  BIT(5)
#define   CROSS19_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_20				  0x024a
#define   CROSS20_MASK					  GENMASK(4, 0)
#define   CROSS20_SHIFT					  0
#define   CROSS20_F					  BIT(5)
#define   CROSS20_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_21				  0x024b
#define   CROSS21_MASK					  GENMASK(4, 0)
#define   CROSS21_SHIFT					  0
#define   CROSS21_F					  BIT(5)
#define   CROSS21_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_22				  0x024c
#define   CROSS22_MASK					  GENMASK(4, 0)
#define   CROSS22_SHIFT					  0
#define   CROSS22_F					  BIT(5)
#define   CROSS22_I					  BIT(6)
#define MAX96717_VTX_Z_CROSS_23				  0x024d
#define   CROSS23_MASK					  GENMASK(4, 0)
#define   CROSS23_SHIFT					  0
#define   CROSS23_F					  BIT(5)
#define   CROSS23_I					  BIT(6)
#define MAX96717_VTX_Z_VTX0				  0x024e
#define   VTG_MODE_MASK					  GENMASK(1, 0)
#define   VTG_MODE_SHIFT				  0
#define   DE_INV					  BIT(2)
#define   HS_INV					  BIT(3)
#define   VS_INV					  BIT(4)
#define   GEN_DE					  BIT(5)
#define   GEN_HS					  BIT(6)
#define   GEN_VS					  BIT(7)
#define MAX96717_VTX_Z_VTX1				  0x024f
#define   VS_TRIG					  BIT(0)
#define   PATGEN_CLK_SRC_MASK				  GENMASK(3, 1)
#define   PATGEN_CLK_SRC_SHIFT				  1
#define   PCLKDET_VTX					  BIT(5)
#define MAX96717_VTX_Z_VS_DLY_2				  0x0250
#define MAX96717_VTX_Z_VS_DLY_1				  0x0251
#define MAX96717_VTX_Z_VS_DLY_0				  0x0252
#define MAX96717_VTX_Z_VS_HIGH_2			  0x0253
#define MAX96717_VTX_Z_VS_HIGH_1			  0x0254
#define MAX96717_VTX_Z_VS_HIGH_0			  0x0255
#define MAX96717_VTX_Z_VS_LOW_2				  0x0256
#define MAX96717_VTX_Z_VS_LOW_1				  0x0257
#define MAX96717_VTX_Z_VS_LOW_0				  0x0258
#define MAX96717_VTX_Z_V2H_2				  0x0259
#define MAX96717_VTX_Z_V2H_1				  0x025a
#define MAX96717_VTX_Z_V2H_0				  0x025b
#define MAX96717_VTX_Z_HS_HIGH_1			  0x025c
#define MAX96717_VTX_Z_HS_HIGH_0			  0x025d
#define MAX96717_VTX_Z_HS_LOW_1				  0x025e
#define MAX96717_VTX_Z_HS_LOW_0				  0x025f
#define MAX96717_VTX_Z_HS_CNT_1				  0x0260
#define MAX96717_VTX_Z_HS_CNT_0				  0x0261
#define MAX96717_VTX_Z_V2D_2				  0x0262
#define MAX96717_VTX_Z_V2D_1				  0x0263
#define MAX96717_VTX_Z_V2D_0				  0x0264
#define MAX96717_VTX_Z_DE_HIGH_1			  0x0265
#define MAX96717_VTX_Z_DE_HIGH_0			  0x0266
#define MAX96717_VTX_Z_DE_LOW_1				  0x0267
#define MAX96717_VTX_Z_DE_LOW_0				  0x0268
#define MAX96717_VTX_Z_DE_CNT_1				  0x0269
#define MAX96717_VTX_Z_DE_CNT_0				  0x026a
#define MAX96717_VTX_Z_VTX29				  0x026b
#define   PATGEN_MODE_MASK				  GENMASK(1, 0)
#define   PATGEN_MODE_SHIFT				  0
#define   GRAD_MODE					  BIT(2)
#define   VPRBS_FAIL					  BIT(5)
#define   VID_PRBS_EN					  BIT(7)
#define MAX96717_VTX_Z_GRAD_INC				  0x026c
#define MAX96717_VTX_Z_CHKR_A_L				  0x026d
#define MAX96717_VTX_Z_CHKR_A_M				  0x026e
#define MAX96717_VTX_Z_CHKR_A_H				  0x026f
#define MAX96717_VTX_Z_CHKR_B_L				  0x0270
#define MAX96717_VTX_Z_CHKR_B_M				  0x0271
#define MAX96717_VTX_Z_CHKR_B_H				  0x0272
#define MAX96717_VTX_Z_CHKR_RPT_A			  0x0273
#define MAX96717_VTX_Z_CHKR_RPT_B			  0x0274
#define MAX96717_VTX_Z_CHKR_ALT				  0x0275
#define MAX96717_VTX_Z_VTX40				  0x0276
#define   CROSSHS_MASK					  GENMASK(4, 0)
#define   CROSSHS_SHIFT					  0
#define   CROSSHS_F					  BIT(5)
#define   CROSSHS_I					  BIT(6)
#define MAX96717_VTX_Z_VTX41				  0x0277
#define   CROSSVS_MASK					  GENMASK(4, 0)
#define   CROSSVS_SHIFT					  0
#define   CROSSVS_F					  BIT(5)
#define   CROSSVS_I					  BIT(6)
#define MAX96717_VTX_Z_VTX42				  0x0278
#define   CROSSDE_MASK					  GENMASK(4, 0)
#define   CROSSDE_SHIFT					  0
#define   CROSSDE_F					  BIT(5)
#define   CROSSDE_I					  BIT(6)

/* GPIO: 0 <= gpio < 11 */
#define MAX96717_GPIO_A(gpio)				  (0x02be + (gpio) * 0x03)
#define   GPIO_OUT_DIS					  BIT(0)
#define   GPIO_TX_EN					  BIT(1)
#define   GPIO_RX_EN					  BIT(2)
#define   GPIO_IN					  BIT(3)
#define   GPIO_OUT					  BIT(4)
#define   TX_COMP_EN					  BIT(5)
#define   RES_CFG					  BIT(7)
#define MAX96717_GPIO_B(gpio)				  (0x02bf + (gpio) * 0x03)
#define   GPIO_TX_ID_MASK				  GENMASK(4, 0)
#define   GPIO_TX_ID_SHIFT				  0
#define   OUT_TYPE					  BIT(5)
#define   OUT_TYPE_PUSH_PULL				  BIT(5)
#define   OUT_TYPE_OPEN_DRAIN				  0
#define   PULL_UPDN_SEL_MASK				  GENMASK(7, 6)
#define   PULL_UPDN_SEL_SHIFT				  6
#define   GPIO_NO_PULL					  0
#define   GPIO_PULL_UP					  1
#define   GPIO_PULL_DOWN				  2
#define MAX96717_GPIO_C(gpio)				  (0x02c0 + (gpio) * 0x03)
#define   GPIO_RX_ID_MASK				  GENMASK(4, 0)
#define   GPIO_RX_ID_SHIFT				  0
#define   OVR_RES_CFG					  BIT(7)

/* CMU */
#define MAX96717_CMU_CMU2				  0x0302
#define   PFDDIV_RSHORT_MASK				  GENMASK(6, 4)
#define   PFDDIV_RSHORT_SHIFT				  4
#define   PFDDIV_VREG_1V0				  0
#define   PFDDIV_VREG_1V1				  1
#define   PFDDIV_VREG_0V875				  2
#define   PFDDIV_VREG_0V94				  3

/* FRONTTOP */
#define MAX96717_FRONTTOP_0				  0x0308
#define   START_PORTB					  BIT(5)
#define   ENABLE_LINE_INFO				  BIT(6)
#define MAX96717_FRONTTOP_VC_SELZ_L			  0x030d
#define MAX96717_FRONTTOP_VC_SELZ_H			  0x030e
#define MAX96717_FRONTTOP_9				  0x0311
#define   START_PORTBZ					  BIT(6)
#define MAX96717_FRONTTOP_10				  0x0312
#define   BPP8DBLZ					  BIT(2)
#define MAX96717_FRONTTOP_11				  0x0313
#define   BPP10DBLZ					  BIT(2)
#define   BPP12DBLZ					  BIT(6)
#define MAX96717_FRONTTOP_16				  0x0318
#define   MEM_DT1_SELZ_MASK				  GENMASK(6, 0)
#define   MEM_DT1_SELZ_SHIFT				  0
#define MAX96717_FRONTTOP_17				  0x0319
#define   MEM_DT2_SELZ_MASK				  GENMASK(6, 0)
#define   MEM_DT2_SELZ_SHIFT				  0
#define MAX96717_FRONTTOP_22				  0x031e
#define   SOFT_BPPZ_MASK				  GENMASK(4, 0)
#define   SOFT_BPPZ_SHIFT				  0
#define   SOFT_BPPZ_EN					  BIT(5)
#define   SOFT_VCZ_EN					  BIT(6)
#define   SOFT_DTZ_EN					  BIT(7)
#define MAX96717_FRONTTOP_24				  0x0320
#define   SOFT_VCZ_MASK					  GENMASK(5, 4)
#define   SOFT_VCZ_SHIFT				  4
#define MAX96717_FRONTTOP_27				  0x0323
#define   SOFT_DTZ_MASK					  GENMASK(5, 0)
#define   SOFT_DTZ_SHIFT				  0
#define MAX96717_FRONTTOP_29				  0x0325
#define   FORCE_START_MIPI_FRONTTOP			  BIT(7)

/* MIPI_RX */
#define MAX96717_MIPI_RX_0				  0x0330
#define   MIPI_RX_RESET					  BIT(3)
#define   CTRL1_VC_MAP_EN				  BIT(5)
#define   MIPI_NONCONTCLK_EN				  BIT(6)
#define MAX96717_MIPI_RX_1				  0x0331
#define   CTRL1_NUM_LANES_MASK				  GENMASK(5, 4)
#define   CTRL1_NUM_LANES_SHIFT				  4
#define   CTRL1_DESKEWEN				  BIT(6)
#define   CTRL1_VCX_EN					  BIT(7)
#define MAX96717_MIPI_RX_2				  0x0332
#define   PHY1_LANE_MAP_MASK				  GENMASK(7, 4)
#define   PHY1_LANE_MAP_SHIFT				  4
#define MAX96717_MIPI_RX_3				  0x0333
#define   PHY2_LANE_MAP_MASK				  GENMASK(3, 0)
#define   PHY2_LANE_MAP_SHIFT				  0
#define MAX96717_MIPI_RX_4				  0x0334
#define   PHY1_POL_MAP_MASK				  GENMASK(6, 4)
#define   PHY1_POL_MAP_SHIFT				  4
#define MAX96717_MIPI_RX_5				  0x0335
#define   PHY2_POL_MAP_MASK				  GENMASK(2, 0)
#define   PHY2_POL_MAP_SHIFT				  0
#define MAX96717_MIPI_RX_7				  0x0337
#define MAX96717_MIPI_RX_8				  0x0338
#define   T_CLK_SETTLE_MASK				  GENMASK(1, 0)
#define   T_CLK_SETTLE_SHIFT				  0
#define   T_HS_SETTLE_MASK				  GENMASK(5, 4)
#define   T_HS_SETTLE_SHIFT				  4
#define MAX96717_MIPI_RX_11				  0x033b
#define   PHY1_LP_ERR_MASK				  GENMASK(4, 0)
#define   PHY1_LP_ERR_SHIFT				  0
#define MAX96717_MIPI_RX_PHY1_HS_ERR			  0x033c
#define MAX96717_MIPI_RX_13				  0x033d
#define   PHY2_LP_ERR_MASK				  GENMASK(4, 0)
#define   PHY2_LP_ERR_SHIFT				  0
#define MAX96717_MIPI_RX_PHY2_HS_ERR			  0x033e
#define MAX96717_MIPI_RX_CTRL1_CSI_ERR_L		  0x0343
#define MAX96717_MIPI_RX_20				  0x0344
#define   CTRL1_CSI_ERR_H_MASK				  GENMASK(2, 0)
#define   CTRL1_CSI_ERR_H_SHIFT				  0
#define MAX96717_MIPI_RX_21				  0x0345
#define   CTRL1_VC_MAP0_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP0_SHIFT				  4
#define MAX96717_MIPI_RX_22				  0x0346
#define   CTRL1_VC_MAP1_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP1_SHIFT				  4
#define MAX96717_MIPI_RX_23				  0x0347
#define   CTRL1_VC_MAP2_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP2_SHIFT				  4
#define MAX96717_MIPI_RX_60				  0x036c
#define   CTRL1_VC_MAP3_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP3_SHIFT				  4
#define MAX96717_MIPI_RX_61				  0x036d
#define   CTRL1_VC_MAP4_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP4_SHIFT				  4
#define MAX96717_MIPI_RX_62				  0x036e
#define   CTRL1_VC_MAP5_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP5_SHIFT				  4
#define MAX96717_MIPI_RX_63				  0x036f
#define   CTRL1_VC_MAP6_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP6_SHIFT				  4

/* MIPI_RX_EXT */
#define MAX96717_MIPI_RX_EXT_00				  0x0377
#define   CTRL1_VC_MAP7_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP7_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_0				  0x0378
#define   CTRL1_VC_MAP8_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP8_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_1				  0x0379
#define   CTRL1_VC_MAP9_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP9_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_2				  0x037a
#define   CTRL1_VC_MAP10_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP10_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_3				  0x037b
#define   CTRL1_VC_MAP11_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP11_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_4				  0x037c
#define   CTRL1_VC_MAP12_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP12_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_5				  0x037d
#define   CTRL1_VC_MAP13_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP13_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_6				  0x037e
#define   CTRL1_VC_MAP14_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP14_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_7				  0x037f
#define   CTRL1_VC_MAP15_MASK				  GENMASK(7, 4)
#define   CTRL1_VC_MAP15_SHIFT				  4
#define MAX96717_MIPI_RX_EXT_8				  0x0380
#define   TUN_FIFO_OVERFLOW				  BIT(0)
#define MAX96717_MIPI_RX_EXT_RSVD			  0x0381
#define MAX96717_MIPI_RX_EXT_11				  0x0383
#define   TUN_MODE					  BIT(7)
#define MAX96717_MIPI_RX_EXT_PHY1_PKT_CNT		  0x038d
#define MAX96717_MIPI_RX_EXT_CSI1_PKT_CNT		  0x038e
#define MAX96717_MIPI_RX_EXT_TUN_PKT_CNT		  0x038f
#define MAX96717_MIPI_RX_EXT_PHY_CLK_CNT		  0x0390

/* FRONTTOP_EXT */
#define MAX96717_FRONTTOP_EXT_MEM_DT3_SELZ		  0x03c8
#define MAX96717_FRONTTOP_EXT_MEM_DT4_SELZ		  0x03c9
#define MAX96717_FRONTTOP_EXT_MEM_DT5_SELZ		  0x03ca
#define MAX96717_FRONTTOP_EXT_MEM_DT6_SELZ		  0x03cb
#define MAX96717_FRONTTOP_EXT_17			  0x03d1
#define   MEM_DT3_SELZ_EN				  BIT(0)
#define   MEM_DT4_SELZ_EN				  BIT(1)
#define   MEM_DT5_SELZ_EN				  BIT(2)
#define   MEM_DT6_SELZ_EN				  BIT(3)

/* MIPI_RX_EXT2 */
#define MAX96717_MIPI_RX_EXT2_A				  0x03dc
#define   MEM_DT7_SELZ_MASK				  GENMASK(6, 0)
#define   MEM_DT7_SELZ_SHIFT				  0
#define MAX96717_MIPI_RX_EXT2_B				  0x03dd
#define   MEM_DT8_SELZ_MASK				  GENMASK(6, 0)
#define   MEM_DT8_SELZ_SHIFT				  0

/* REF_VTG */
#define MAX96717_REF_VTG_VTX0				  0x03e0
#define   REF_VTG_GEN_VS				  BIT(0)
#define   REF_VTG_VS_INV				  BIT(1)
#define   REF_VTG_GEN_HS				  BIT(2)
#define   REF_VTG_HS_INV				  BIT(3)
#define   REF_VTG_MODE_MASK				  GENMASK(5, 4)
#define   REF_VTG_MODE_SHIFT				  4
#define   REF_VTG_VS_TRIG				  BIT(6)
#define MAX96717_REF_VTG_VS_HIGH_2			  0x03e1
#define MAX96717_REF_VTG_VS_HIGH_1			  0x03e2
#define MAX96717_REF_VTG_VS_HIGH_0			  0x03e3
#define MAX96717_REF_VTG_VS_LOW_2			  0x03e4
#define MAX96717_REF_VTG_VS_LOW_1			  0x03e5
#define MAX96717_REF_VTG_VS_LOW_0			  0x03e6
#define MAX96717_REF_VTG_V2H_2				  0x03e7
#define MAX96717_REF_VTG_V2H_1				  0x03e8
#define MAX96717_REF_VTG_V2H_0				  0x03e9
#define MAX96717_REF_VTG_HS_HIGH_1			  0x03ea
#define MAX96717_REF_VTG_HS_HIGH_0			  0x03eb
#define MAX96717_REF_VTG_HS_LOW_1			  0x03ec
#define MAX96717_REF_VTG_HS_LOW_0			  0x03ed
#define MAX96717_REF_VTG_HS_CNT_1			  0x03ee
#define MAX96717_REF_VTG_HS_CNT_0			  0x03ef
#define MAX96717_REF_VTG_0				  0x03f0
#define   REFGEN_EN					  BIT(0)
#define   REFGEN_RST					  BIT(1)
#define   REFGEN_PREDEF_FREQ_ALT			  BIT(3)
#define   REFGEN_PREDEF_FREQ_MASK			  GENMASK(5, 4)
#define   REFGEN_PREDEF_FREQ_SHIFT			  4
#define   REFGEN_PREDEF_EN				  BIT(6)
#define   REFGEN_LOCKED					  BIT(7)
#define MAX96717_REF_VTG_1				  0x03f1
#define   PCLKEN					  BIT(0)
#define   PCLK_GPIO_MASK				  GENMASK(5, 1)
#define   PCLK_GPIO_SHIFT				  1
#define   RCLKEN_Y					  BIT(7)
#define MAX96717_REF_VTG_2				  0x03f2
#define   HSEN						  BIT(0)
#define   HS_GPIO_MASK					  GENMASK(5, 1)
#define   HS_GPIO_SHIFT					  1
#define MAX96717_REF_VTG_3				  0x03f3
#define   VSEN						  BIT(0)
#define   VS_GPIO_MASK					  GENMASK(5, 1)
#define   VS_GPIO_SHIFT					  1
#define MAX96717_REF_VTG_REFGEN_FB_FRACT_L		  0x03f4
#define MAX96717_REF_VTG_5				  0x03f5
#define   REFGEN_FB_FRACT_H_MASK			  GENMASK(3, 0)
#define   REFGEN_FB_FRACT_H_SHIFT			  0
#define MAX96717_REF_VTG_VS_DLY_2			  0x03f6
#define MAX96717_REF_VTG_VS_DLY_1			  0x03f7
#define MAX96717_REF_VTG_VS_DLY_0			  0x03f8
#define MAX96717_REF_VTG_9				  0x03f9
#define   REF_VTG_TRIG_ID_MASK				  GENMASK(4, 0)
#define   REF_VTG_TRIG_ID_SHIFT				  0
#define   REF_VTG_TRIG_EN				  BIT(7)

/* AFE */
#define MAX96717_AFE_ADC_CTRL_0				  0x0500
#define   CPU_ADC_START					  BIT(0)
#define   ADC_PU					  BIT(1)
#define   BUF_PU					  BIT(2)
#define   ADC_REFBUF_PU					  BIT(3)
#define   ADC_CHGPUMP_PU				  BIT(4)
#define   BUF_BYPASS					  BIT(7)
#define MAX96717_AFE_ADC_CTRL_1				  0x0501
#define   ADC_SCALE					  BIT(1)
#define   ADC_REFSEL					  BIT(2)
#define   ADC_CLK_EN					  BIT(3)
#define   ADC_CHSEL_MASK				  GENMASK(7, 4)
#define   ADC_CHSEL_SHIFT				  4
#define MAX96717_AFE_ADC_CTRL_2				  0x0502
#define   INMUX_EN					  BIT(0)
#define   ADC_XREF					  BIT(1)
#define   ADC_DIV_MASK					  GENMASK(3, 2)
#define   ADC_DIV_SHIFT					  2
#define MAX96717_AFE_ADC_DATA_L				  0x0508
#define MAX96717_AFE_ADC_DATA1				  0x0509
#define   ADC_DATA_H_MASK				  GENMASK(1, 0)
#define   ADC_DATA_H_SHIFT				  0
#define MAX96717_AFE_ADC_INTRIE0			  0x050c
#define   ADC_DONE_IE					  BIT(0)
#define   ADC_REF_READY_IE				  BIT(1)
#define   ADC_HI_LIMIT_IE				  BIT(2)
#define   ADC_LO_LIMIT_IE				  BIT(3)
#define   ADC_TMON_CAL_OOD_IE				  BIT(5)
#define   ADC_OVERRANGE_IE				  BIT(6)
#define   ADC_CALDONE_IE				  BIT(7)
#define MAX96717_AFE_ADC_INTRIE1			  0x050d
#define   CH0_HI_LIMIT_IE				  BIT(0)
#define   CH1_HI_LIMIT_IE				  BIT(1)
#define   CH2_HI_LIMIT_IE				  BIT(2)
#define   CH3_HI_LIMIT_IE				  BIT(3)
#define   CH4_HI_LIMIT_IE				  BIT(4)
#define   CH5_HI_LIMIT_IE				  BIT(5)
#define   CH6_HI_LIMIT_IE				  BIT(6)
#define   CH7_HI_LIMIT_IE				  BIT(7)
#define MAX96717_AFE_ADC_INTRIE2			  0x050e
#define   CH0_LO_LIMIT_IE				  BIT(0)
#define   CH1_LO_LIMIT_IE				  BIT(1)
#define   CH2_LO_LIMIT_IE				  BIT(2)
#define   CH3_LO_LIMIT_IE				  BIT(3)
#define   CH4_LO_LIMIT_IE				  BIT(4)
#define   CH5_LO_LIMIT_IE				  BIT(5)
#define   CH6_LO_LIMIT_IE				  BIT(6)
#define   CH7_LO_LIMIT_IE				  BIT(7)
#define MAX96717_AFE_ADC_INTRIE3			  0x050f
#define   TMON_ERR_IE					  BIT(1)
#define   REFLIMSCL3_IE					  BIT(3)
#define   REFLIMSCL2_IE					  BIT(4)
#define   REFLIMSCL1_IE					  BIT(5)
#define   REFLIM_IE					  BIT(6)
#define MAX96717_AFE_ADC_INTR0				  0x0510
#define   ADC_DONE_IF					  BIT(0)
#define   ADC_REF_READY_IF				  BIT(1)
#define   ADC_HI_LIMIT_IF				  BIT(2)
#define   ADC_LO_LIMIT_IF				  BIT(3)
#define   ADC_TMON_CAL_OOD_IF				  BIT(5)
#define   ADC_OVERRANGE_IF				  BIT(6)
#define   ADC_CALDONE_IF				  BIT(7)
#define MAX96717_AFE_ADC_INTR1				  0x0511
#define   CH0_HI_LIMIT_IF				  BIT(0)
#define   CH1_HI_LIMIT_IF				  BIT(1)
#define   CH2_HI_LIMIT_IF				  BIT(2)
#define   CH3_HI_LIMIT_IF				  BIT(3)
#define   CH4_HI_LIMIT_IF				  BIT(4)
#define   CH5_HI_LIMIT_IF				  BIT(5)
#define   CH6_HI_LIMIT_IF				  BIT(6)
#define   CH7_HI_LIMIT_IF				  BIT(7)
#define MAX96717_AFE_ADC_INTR2				  0x0512
#define   CH0_LO_LIMIT_IF				  BIT(0)
#define   CH1_LO_LIMIT_IF				  BIT(1)
#define   CH2_LO_LIMIT_IF				  BIT(2)
#define   CH3_LO_LIMIT_IF				  BIT(3)
#define   CH4_LO_LIMIT_IF				  BIT(4)
#define   CH5_LO_LIMIT_IF				  BIT(5)
#define   CH6_LO_LIMIT_IF				  BIT(6)
#define   CH7_LO_LIMIT_IF				  BIT(7)
#define MAX96717_AFE_ADC_INTR3				  0x0513
#define   TMON_ERR_IF					  BIT(1)
#define   REFLIMSCL3_IF					  BIT(3)
#define   REFLIMSCL2_IF					  BIT(4)
#define   REFLIMSCL1_IF					  BIT(5)
#define   REFLIM_IF					  BIT(6)
#define MAX96717_AFE_CHLOLIMIT_L0			  0x0514
#define MAX96717_AFE_ADC_LIMIT0_1			  0x0515
#define   CHLOLIMIT_H0_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H0_SHIFT				  0
#define   CHHILIMIT_L0_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L0_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT0_2			  0x0516
#define   CHHILIMIT_H0_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H0_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT0_3			  0x0517
#define   CH_SEL0_MASK					  GENMASK(3, 0)
#define   CH_SEL0_SHIFT					  0
#define   DIV_SEL0_MASK					  GENMASK(5, 4)
#define   DIV_SEL0_SHIFT				  4
#define MAX96717_AFE_CHLOLIMIT_L1			  0x0518
#define MAX96717_AFE_ADC_LIMIT1_1			  0x0519
#define   CHLOLIMIT_H1_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H1_SHIFT				  0
#define   CHHILIMIT_L1_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L1_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT1_2			  0x051a
#define   CHHILIMIT_H1_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H1_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT1_3			  0x051b
#define   CH_SEL1_MASK					  GENMASK(3, 0)
#define   CH_SEL1_SHIFT					  0
#define   DIV_SEL1_MASK					  GENMASK(5, 4)
#define   DIV_SEL1_SHIFT				  4
#define MAX96717_AFE_CHLOLIMIT_L2			  0x051c
#define MAX96717_AFE_ADC_LIMIT2_1			  0x051d
#define   CHLOLIMIT_H2_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H2_SHIFT				  0
#define   CHHILIMIT_L2_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L2_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT2_2			  0x051e
#define   CHHILIMIT_H2_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H2_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT2_3			  0x051f
#define   CH_SEL2_MASK					  GENMASK(3, 0)
#define   CH_SEL2_SHIFT					  0
#define   DIV_SEL2_MASK					  GENMASK(5, 4)
#define   DIV_SEL2_SHIFT				  4
#define MAX96717_AFE_CHLOLIMIT_L3			  0x0520
#define MAX96717_AFE_ADC_LIMIT3_1			  0x0521
#define   CHLOLIMIT_H3_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H3_SHIFT				  0
#define   CHHILIMIT_L3_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L3_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT3_2			  0x0522
#define   CHHILIMIT_H3_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H3_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT3_3			  0x0523
#define   CH_SEL3_MASK					  GENMASK(3, 0)
#define   CH_SEL3_SHIFT					  0
#define   DIV_SEL3_MASK					  GENMASK(5, 4)
#define   DIV_SEL3_SHIFT				  4
#define MAX96717_AFE_CHLOLIMIT_L4			  0x0524
#define MAX96717_AFE_ADC_LIMIT4_1			  0x0525
#define   CHLOLIMIT_H4_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H4_SHIFT				  0
#define   CHHILIMIT_L4_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L4_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT4_2			  0x0526
#define   CHHILIMIT_H4_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H4_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT4_3			  0x0527
#define   CH_SEL4_MASK					  GENMASK(3, 0)
#define   CH_SEL4_SHIFT					  0
#define   DIV_SEL4_MASK					  GENMASK(5, 4)
#define   DIV_SEL4_SHIFT				  4
#define MAX96717_AFE_CHLOLIMIT_L5			  0x0528
#define MAX96717_AFE_ADC_LIMIT5_1			  0x0529
#define   CHLOLIMIT_H5_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H5_SHIFT				  0
#define   CHHILIMIT_L5_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L5_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT5_2			  0x052a
#define   CHHILIMIT_H5_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H5_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT5_3			  0x052b
#define   CH_SEL5_MASK					  GENMASK(3, 0)
#define   CH_SEL5_SHIFT					  0
#define   DIV_SEL5_MASK					  GENMASK(5, 4)
#define   DIV_SEL5_SHIFT				  4
#define MAX96717_AFE_CHLOLIMIT_L6			  0x052c
#define MAX96717_AFE_ADC_LIMIT6_1			  0x052d
#define   CHLOLIMIT_H6_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H6_SHIFT				  0
#define   CHHILIMIT_L6_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L6_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT6_2			  0x052e
#define   CHHILIMIT_H6_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H6_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT6_3			  0x052f
#define   CH_SEL6_MASK					  GENMASK(3, 0)
#define   CH_SEL6_SHIFT					  0
#define   DIV_SEL6_MASK					  GENMASK(5, 4)
#define   DIV_SEL6_SHIFT				  4
#define MAX96717_AFE_CHLOLIMIT_L7			  0x0530
#define MAX96717_AFE_ADC_LIMIT7_1			  0x0531
#define   CHLOLIMIT_H7_MASK				  GENMASK(1, 0)
#define   CHLOLIMIT_H7_SHIFT				  0
#define   CHHILIMIT_L7_MASK				  GENMASK(7, 4)
#define   CHHILIMIT_L7_SHIFT				  4
#define MAX96717_AFE_ADC_LIMIT7_2			  0x0532
#define   CHHILIMIT_H7_MASK				  GENMASK(5, 0)
#define   CHHILIMIT_H7_SHIFT				  0
#define MAX96717_AFE_ADC_LIMIT7_3			  0x0533
#define   CH_SEL7_MASK					  GENMASK(3, 0)
#define   CH_SEL7_SHIFT					  0
#define   DIV_SEL7_MASK					  GENMASK(5, 4)
#define   DIV_SEL7_SHIFT				  4
#define MAX96717_AFE_ADC_RR_CTRL0			  0x0534
#define   ADC_RR_RUN					  BIT(0)
#define MAX96717_AFE_ADC_CTRL_4				  0x053e
#define   ADC_PIN_EN_MASK				  GENMASK(2, 0)
#define   ADC_PIN_EN_SHIFT				  0

/* MISC */
#define MAX96717_MISC_BITLEN_PT_1_L			  0x0548
#define MAX96717_MISC_UART_PT_1				  0x0549
#define   BITLEN_PT_1_H_MASK				  GENMASK(5, 0)
#define   BITLEN_PT_1_H_SHIFT				  0
#define MAX96717_MISC_BITLEN_PT_2_L			  0x054a
#define MAX96717_MISC_UART_PT_3				  0x054b
#define   BITLEN_PT_2_H_MASK				  GENMASK(5, 0)
#define   BITLEN_PT_2_H_SHIFT				  0
#define MAX96717_MISC_I2C_PT_4				  0x0550
#define   SRC_A_1_MASK					  GENMASK(7, 1)
#define   SRC_A_1_SHIFT					  1
#define MAX96717_MISC_I2C_PT_5				  0x0551
#define   DST_A_1_MASK					  GENMASK(7, 1)
#define   DST_A_1_SHIFT					  1
#define MAX96717_MISC_I2C_PT_6				  0x0552
#define   SRC_B_1_MASK					  GENMASK(7, 1)
#define   SRC_B_1_SHIFT					  1
#define MAX96717_MISC_I2C_PT_7				  0x0553
#define   DST_B_1_MASK					  GENMASK(7, 1)
#define   DST_B_1_SHIFT					  1
#define MAX96717_MISC_I2C_PT_8				  0x0554
#define   SRC_A_2_MASK					  GENMASK(7, 1)
#define   SRC_A_2_SHIFT					  1
#define MAX96717_MISC_I2C_PT_9				  0x0555
#define   DST_A_2_MASK					  GENMASK(7, 1)
#define   DST_A_2_SHIFT					  1
#define MAX96717_MISC_I2C_PT_10				  0x0556
#define   SRC_B_2_MASK					  GENMASK(7, 1)
#define   SRC_B_2_SHIFT					  1
#define MAX96717_MISC_I2C_PT_11				  0x0557
#define   DST_B_2_MASK					  GENMASK(7, 1)
#define   DST_B_2_SHIFT					  1
#define MAX96717_MISC_HS_VS_Z				  0x055f
#define   HS_POL_Z					  BIT(0)
#define   VS_POL_Z					  BIT(1)
#define   HS_DET_Z					  BIT(4)
#define   VS_DET_Z					  BIT(5)
#define   DE_DET_Z					  BIT(6)
#define MAX96717_MISC_UNLOCK_KEY			  0x056e
#define MAX96717_MISC_PIO_SLEW_0			  0x056f
#define   PIO00_SLEW_MASK				  GENMASK(1, 0)
#define   PIO00_SLEW_SHIFT				  0
#define   PIO01_SLEW_MASK				  GENMASK(3, 2)
#define   PIO01_SLEW_SHIFT				  2
#define   PIO02_SLEW_MASK				  GENMASK(5, 4)
#define   PIO02_SLEW_SHIFT				  4
#define MAX96717_MISC_PIO_SLEW_1			  0x0570
#define   PIO05_SLEW_MASK				  GENMASK(3, 2)
#define   PIO05_SLEW_SHIFT				  2
#define   PIO06_SLEW_MASK				  GENMASK(5, 4)
#define   PIO06_SLEW_SHIFT				  4
#define MAX96717_MISC_PIO_SLEW_2			  0x0571
#define   PIO010_SLEW_MASK				  GENMASK(5, 4)
#define   PIO010_SLEW_SHIFT				  4
#define   PIO011_SLEW_MASK				  GENMASK(7, 6)
#define   PIO011_SLEW_SHIFT				  6

/* MIPI_RX_EXT3 */
#define MAX96717_MIPI_RX_EXT3_CTRL1_FS_CNT_L		  0x0584
#define MAX96717_MIPI_RX_EXT3_CTRL1_FS_CNT_H		  0x0585
#define MAX96717_MIPI_RX_EXT3_CTRL1_FE_CNT_L		  0x0586
#define MAX96717_MIPI_RX_EXT3_CTRL1_FE_CNT_H		  0x0587
#define MAX96717_MIPI_RX_EXT3_EXT8			  0x0588
#define   CTRL1_FS_VC_SEL_MASK				  GENMASK(3, 0)
#define   CTRL1_FS_VC_SEL_SHIFT				  0

/* SPI_CC_WR */
#define MAX96717_SPI_CC_WR_SPI_CC_WR_			  0x1300

/* SPI_CC_RD */
#define MAX96717_SPI_CC_RD_SPI_CC_RD_			  0x1380

/* RLMS */
#define MAX96717_RLMS_4					  0x1404
#define   EOM_EN					  BIT(0)
#define   EOM_PER_MODE					  BIT(1)
#define   EOM_CHK_THR_MASK				  GENMASK(3, 2)
#define   EOM_CHK_THR_SHIFT				  2
#define   EOM_CHK_AMOUNT_MASK				  GENMASK(7, 4)
#define   EOM_CHK_AMOUNT_SHIFT				  4
#define MAX96717_RLMS_5					  0x1405
#define   EOM_MIN_THR_MASK				  GENMASK(6, 0)
#define   EOM_MIN_THR_SHIFT				  0
#define   EOM_MAN_TRG_REQ				  BIT(7)
#define MAX96717_RLMS_A_RLMS6				  0x1406
#define   EOM_PV_MODE					  BIT(7)
#define MAX96717_RLMS_7					  0x1407
#define   EOM_MASK					  GENMASK(6, 0)
#define   EOM_SHIFT					  0
#define   EOM_DONE					  BIT(7)
#define MAX96717_RLMS_17				  0x1417
#define   AGCEN						  BIT(0)
#define   BSTEN						  BIT(1)
#define   BSTENOV					  BIT(2)
#define   DFE1EN					  BIT(3)
#define   DFE2EN					  BIT(4)
#define   DFE3EN					  BIT(5)
#define   DFE4EN					  BIT(6)
#define   DFE5EN					  BIT(7)
#define MAX96717_RLMS_AGCMUL				  0x141c
#define MAX96717_RLMS_RLMS1D				  0x141d
#define   AGCMUH_MASK					  GENMASK(5, 0)
#define   AGCMUH_SHIFT					  0
#define MAX96717_RLMS_AGCINIT				  0x141f
#define MAX96717_RLMS_32				  0x1432
#define   OSNMODE					  BIT(7)
#define MAX96717_RLMS_EYEMONVALCNTL			  0x143a
#define MAX96717_RLMS_EYEMONVALCNTH			  0x143b
#define MAX96717_RLMS_64				  0x1464
#define   TXSSCMODE_MASK				  GENMASK(1, 0)
#define   TXSSCMODE_SHIFT				  0
#define MAX96717_RLMS_70				  0x1470
#define   TXSSCFRQCTRL_MASK				  GENMASK(6, 0)
#define   TXSSCFRQCTRL_SHIFT				  0
#define MAX96717_RLMS_71				  0x1471
#define   TXSSCEN					  BIT(0)
#define   TXSSCCENSPRST_MASK				  GENMASK(6, 1)
#define   TXSSCCENSPRST_SHIFT				  1
#define MAX96717_RLMS_TXSSCPRESCLL			  0x1472
#define MAX96717_RLMS_73				  0x1473
#define   TXSSCPRESCLH_MASK				  GENMASK(2, 0)
#define   TXSSCPRESCLH_SHIFT				  0
#define MAX96717_RLMS_TXSSCPHL				  0x1474
#define MAX96717_RLMS_75				  0x1475
#define   TXSSCPHH_MASK					  GENMASK(6, 0)
#define   TXSSCPHH_SHIFT				  0
#define MAX96717_RLMS_A_RLMS76				  0x1476
#define   TXSSCPHQUAD_MASK				  GENMASK(1, 0)
#define   TXSSCPHQUAD_SHIFT				  0
#define MAX96717_RLMS_A8				  0x14a8
#define   FW_PHY_RSTB					  BIT(5)
#define   FW_PHY_PU_TX					  BIT(6)
#define   FW_PHY_CTRL					  BIT(7)
#define MAX96717_RLMS_A9				  0x14a9
#define   FW_RXD_EN					  BIT(3)
#define   FW_TXD_EN					  BIT(4)
#define   FW_TXD_SQUELCH				  BIT(5)
#define   FW_REPCAL_RSTB				  BIT(7)
#define MAX96717_RLMS_AA				  0x14aa
#define   ROR_CLK_DET					  BIT(5)
#define MAX96717_RLMS_CE				  0x14ce
#define   ENFFE						  BIT(0)
#define   ENMINUS_MAN					  BIT(3)
#define   ENMINUS_REG					  BIT(4)

/* DPLL_REF */
#define MAX96717_DPLL_REF_0				  0x1a00
#define   CONFIG_SOFT_RST_N				  BIT(0)
#define MAX96717_DPLL_REF_3				  0x1a03
#define   CONFIG_SPREAD_BIT_RATIO_MASK			  GENMASK(2, 0)
#define   CONFIG_SPREAD_BIT_RATIO_SHIFT			  0
#define   CONFIG_USE_INTERNAL_DIVIDER_VALUES		  BIT(4)
#define   CONFIG_SEL_CLOCK_OUT_USE_EXTERNAL		  BIT(7)
#define MAX96717_DPLL_REF_7				  0x1a07
#define   CONFIG_DIV_IN_MASK				  GENMASK(6, 2)
#define   CONFIG_DIV_IN_SHIFT				  2
#define   CONFIG_DIV_FB_L				  BIT(7)
#define MAX96717_DPLL_REF_CONFIG_DIV_FB_H		  0x1a08
#define MAX96717_DPLL_REF_9				  0x1a09
#define   CONFIG_DIV_FB_EXP_MASK			  GENMASK(2, 0)
#define   CONFIG_DIV_FB_EXP_SHIFT			  0
#define   CONFIG_DIV_OUT_L_MASK				  GENMASK(7, 3)
#define   CONFIG_DIV_OUT_L_SHIFT			  3
#define MAX96717_DPLL_REF_10				  0x1a0a
#define   CONFIG_DIV_OUT_H_MASK				  GENMASK(3, 0)
#define   CONFIG_DIV_OUT_H_SHIFT			  0
#define   CONFIG_DIV_OUT_EXP_MASK			  GENMASK(6, 4)
#define   CONFIG_DIV_OUT_EXP_SHIFT			  4
#define   CONFIG_ALLOW_COARSE_CHANGE			  BIT(7)

/* EFUSE */
#define MAX96717_EFUSE_SERIAL_NUMBER_0			  0x1c50
#define MAX96717_EFUSE_SERIAL_NUMBER_1			  0x1c51
#define MAX96717_EFUSE_SERIAL_NUMBER_2			  0x1c52
#define MAX96717_EFUSE_SERIAL_NUMBER_3			  0x1c53
#define MAX96717_EFUSE_SERIAL_NUMBER_4			  0x1c54
#define MAX96717_EFUSE_SERIAL_NUMBER_5			  0x1c55
#define MAX96717_EFUSE_SERIAL_NUMBER_6			  0x1c56
#define MAX96717_EFUSE_SERIAL_NUMBER_7			  0x1c57
#define MAX96717_EFUSE_SERIAL_NUMBER_8			  0x1c58
#define MAX96717_EFUSE_SERIAL_NUMBER_9			  0x1c59
#define MAX96717_EFUSE_SERIAL_NUMBER_10			  0x1c5a
#define MAX96717_EFUSE_SERIAL_NUMBER_11			  0x1c5b
#define MAX96717_EFUSE_SERIAL_NUMBER_12			  0x1c5c
#define MAX96717_EFUSE_SERIAL_NUMBER_13			  0x1c5d
#define MAX96717_EFUSE_SERIAL_NUMBER_14			  0x1c5e
#define MAX96717_EFUSE_SERIAL_NUMBER_15			  0x1c5f
#define MAX96717_EFUSE_SERIAL_NUMBER_16			  0x1c60
#define MAX96717_EFUSE_SERIAL_NUMBER_17			  0x1c61
#define MAX96717_EFUSE_SERIAL_NUMBER_18			  0x1c62
#define MAX96717_EFUSE_SERIAL_NUMBER_19			  0x1c63
#define MAX96717_EFUSE_SERIAL_NUMBER_20			  0x1c64
#define MAX96717_EFUSE_SERIAL_NUMBER_21			  0x1c65
#define MAX96717_EFUSE_SERIAL_NUMBER_22			  0x1c66
#define MAX96717_EFUSE_SERIAL_NUMBER_23			  0x1c67

/* FUNC_SAFE */
#define MAX96717_FUNC_SAFE_REGCRC0			  0x1d00
#define   RESET_CRC					  BIT(0)
#define   CHECK_CRC					  BIT(1)
#define   PERIODIC_COMPUTE				  BIT(2)
#define   I2C_WR_COMPUTE				  BIT(3)
#define   GEN_ROLLING_CRC				  BIT(4)
#define MAX96717_FUNC_SAFE_CRC_PERIOD			  0x1d01
#define MAX96717_FUNC_SAFE_REGCRC_LSB			  0x1d02
#define MAX96717_FUNC_SAFE_REGCRC_MSB			  0x1d03
#define MAX96717_FUNC_SAFE_I2C_UART_CRC0		  0x1d08
#define   RESET_MSGCNTR					  BIT(0)
#define MAX96717_FUNC_SAFE_I2C_UART_CRC1		  0x1d09
#define   RESET_CRC_ERR_CNT				  BIT(0)
#define   RESET_MSGCNTR_ERR_CNT				  BIT(1)
#define MAX96717_FUNC_SAFE_I2C_UART_CRC2		  0x1d0a
#define MAX96717_FUNC_SAFE_I2C_UART_CRC3		  0x1d0b
#define MAX96717_FUNC_SAFE_I2C_UART_CRC4		  0x1d0c
#define MAX96717_FUNC_SAFE_FS_INTR0			  0x1d12
#define   REG_CRC_ERR_OEN				  BIT(0)
#define   MEM_ECC_ERR1_OEN				  BIT(4)
#define   MEM_ECC_ERR2_OEN				  BIT(5)
#define   I2C_UART_CRC_ERR_OEN				  BIT(6)
#define   I2C_UART_MSGCNTR_ERR_OEN			  BIT(7)
#define MAX96717_FUNC_SAFE_FS_INTR1			  0x1d13
#define   REG_CRC_ERR_FLAG				  BIT(0)
#define   MEM_ECC_ERR1_INT				  BIT(4)
#define   MEM_ECC_ERR2_INT				  BIT(5)
#define   I2C_UART_CRC_ERR_INT				  BIT(6)
#define   I2C_UART_MSGCNTR_ERR_INT			  BIT(7)
#define MAX96717_FUNC_SAFE_MEM_ECC0			  0x1d14
#define   RESET_MEM_ECC_ERR1_CNT			  BIT(0)
#define   RESET_MEM_ECC_ERR2_CNT			  BIT(1)
#define MAX96717_FUNC_SAFE_REG_POST0			  0x1d20
#define   POST_LBIST_PASSED				  BIT(5)
#define   POST_MBIST_PASSED				  BIT(6)
#define   POST_DONE					  BIT(7)
#define MAX96717_FUNC_SAFE_REGADCBIST0			  0x1d28
#define   RUN_TMON_CAL					  BIT(0)
#define   RUN_ACCURACY					  BIT(2)
#define   MUXVER_EN					  BIT(4)
#define   RR_ACCURACY					  BIT(7)
#define MAX96717_FUNC_SAFE_REFLIM			  0x1d31
#define MAX96717_FUNC_SAFE_REFLIMSCL1			  0x1d32
#define MAX96717_FUNC_SAFE_REFLIMSCL2			  0x1d33
#define MAX96717_FUNC_SAFE_REFLIMSCL3			  0x1d34
#define MAX96717_FUNC_SAFE_TLIMIT			  0x1d35
#define MAX96717_FUNC_SAFE_MUXV_CTRL			  0x1d37
#define MAX96717_FUNC_SAFE_TMONCAL_OOD_WAIT_B2		  0x1d3a
#define MAX96717_FUNC_SAFE_T_EST_OUT_B0			  0x1d3b
#define MAX96717_FUNC_SAFE_REGADCBIST14			  0x1d3c
#define   ALT_T_EST_OUT_B1_MASK				  GENMASK(1, 0)
#define   ALT_T_EST_OUT_B1_SHIFT			  0
#define   T_EST_OUT_B1_MASK				  GENMASK(7, 6)
#define   T_EST_OUT_B1_SHIFT				  6
#define MAX96717_FUNC_SAFE_ALT_T_EST_OUT_B0		  0x1d3d
#define MAX96717_FUNC_SAFE_CC_RTTN_ERR			  0x1d5f
#define   INJECT_RTTN_CRC_ERR				  BIT(0)
#define   INJECT_EFUSE_CRC_ERR				  BIT(1)
#define   RESET_EFUSE_CRC_ERR				  BIT(2)

#endif /* _MAX96717_REGS_H_ */
