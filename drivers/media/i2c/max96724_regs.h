// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 NXP
 */

#ifndef _MAX96724_REGS_H_
#define _MAX96724_REGS_H_

/* DEV */
#define MAX96724_DEV_REG0				  0x0000
#define   CFG_BLOCK					  BIT(0)
#define   DEV_ADDR_MASK					  GENMASK(7, 1)
#define   DEV_ADDR_SHIFT				  1
#define MAX96724_DEV_REG1				  0x0001
#define   DIS_LOC_CC_MASK				  GENMASK(5, 4)
#define   DIS_LOC_CC_SHIFT				  4
#define MAX96724_DEV_REG3				  0x0003
#define   DIS_REM_CC_A_MASK				  GENMASK(1, 0)
#define   DIS_REM_CC_A_SHIFT				  0
#define   DIS_REM_CC_B_MASK				  GENMASK(3, 2)
#define   DIS_REM_CC_B_SHIFT				  2
#define   DIS_REM_CC_C_MASK				  GENMASK(5, 4)
#define   DIS_REM_CC_C_SHIFT				  4
#define   DIS_REM_CC_D_MASK				  GENMASK(7, 6)
#define   DIS_REM_CC_D_SHIFT				  6
#define MAX96724_DEV_REG4				  0x0004
#define   VID_EN_0					  BIT(0)
#define   VID_EN_1					  BIT(1)
#define   VID_EN_2					  BIT(2)
#define   VID_EN_3					  BIT(3)
#define MAX96724_DEV_REG5				  0x0005
#define   ERRB_MST_RST					  BIT(3)
#define   ERRB_LOCK_OEN					  BIT(4)
#define   LOCK_CFG					  BIT(5)
#define   ERRB_EN					  BIT(6)
#define   LOCK_EN					  BIT(7)
#define MAX96724_DEV_REG6				  0x0006
#define   LINK_EN_A					  BIT(0)
#define   LINK_EN_B					  BIT(1)
#define   LINK_EN_C					  BIT(2)
#define   LINK_EN_D					  BIT(3)
#define   GMSL2_A					  BIT(4)
#define   GMSL2_B					  BIT(5)
#define   GMSL2_C					  BIT(6)
#define   GMSL2_D					  BIT(7)
#define MAX96724_DEV_REG7				  0x0007
#define   CC_CROSSOVER_SEL_MASK				  GENMASK(7, 4)
#define   CC_CROSSOVER_SEL_SHIFT			  4
#define MAX96724_DEV_CTRL12				  0x000a
#define   LOCKED_B					  BIT(3)
#define MAX96724_DEV_CTRL13				  0x000b
#define   LOCKED_C					  BIT(3)
#define MAX96724_DEV_CTRL14				  0x000c
#define   LOCKED_D					  BIT(3)
#define MAX96724_DEV_REG13				  0x000d
#define MAX96724_DEV_REG26				  0x0010
#define   RX_RATE_PHYA_MASK				  GENMASK(1, 0)
#define   RX_RATE_PHYA_SHIFT				  0
#define   TX_RATE_PHYA_MASK				  GENMASK(3, 2)
#define   TX_RATE_PHYA_SHIFT				  2
#define   RX_RATE_PHYB_MASK				  GENMASK(5, 4)
#define   RX_RATE_PHYB_SHIFT				  4
#define   TX_RATE_PHYB_MASK				  GENMASK(7, 6)
#define   TX_RATE_PHYB_SHIFT				  6
#define MAX96724_DEV_REG27				  0x0011
#define   RX_RATE_PHYC_MASK				  GENMASK(1, 0)
#define   RX_RATE_PHYC_SHIFT				  0
#define   TX_RATE_PHYC_MASK				  GENMASK(3, 2)
#define   TX_RATE_PHYC_SHIFT				  2
#define   RX_RATE_PHYD_MASK				  GENMASK(5, 4)
#define   RX_RATE_PHYD_SHIFT				  4
#define   TX_RATE_PHYD_MASK				  GENMASK(7, 6)
#define   TX_RATE_PHYD_SHIFT				  6

/* TOP_CTRL */
#define MAX96724_TOP_CTRL_PWR0				  0x0012
#define   CMP_STATUS_MASK				  GENMASK(4, 0)
#define   CMP_STATUS_SHIFT				  0
#define   VDDBAD_STATUS_MASK				  GENMASK(7, 5)
#define   VDDBAD_STATUS_SHIFT				  5
#define MAX96724_TOP_CTRL_PWR1				  0x0013
#define   RESET_ALL					  BIT(6)
#define MAX96724_TOP_CTRL_CTRL1				  0x0018
#define   RESET_ONESHOT_A				  BIT(0)
#define   RESET_ONESHOT_B				  BIT(1)
#define   RESET_ONESHOT_C				  BIT(2)
#define   RESET_ONESHOT_D				  BIT(3)
#define   RESET_LINK_A					  BIT(4)
#define   RESET_LINK_B					  BIT(5)
#define   RESET_LINK_C					  BIT(6)
#define   RESET_LINK_D					  BIT(7)
#define MAX96724_TOP_CTRL_CTRL3				  0x001a
#define   LOCK_PIN					  BIT(0)
#define   CMU_LOCKED					  BIT(1)
#define   ERROR						  BIT(2)
#define   LOCKED_A					  BIT(3)
#define MAX96724_TOP_CTRL_CTRL11			  0x0022
#define   CXTP_A					  BIT(0)
#define   CXTP_B					  BIT(2)
#define   CXTP_C					  BIT(4)
#define   CXTP_D					  BIT(6)
#define MAX96724_TOP_CTRL_INTR2				  0x0025
#define   DEC_ERR_OEN_A					  BIT(0)
#define   DEC_ERR_OEN_B					  BIT(1)
#define   DEC_ERR_OEN_C					  BIT(2)
#define   DEC_ERR_OEN_D					  BIT(3)
#define MAX96724_TOP_CTRL_INTR3				  0x0026
#define   DEC_ERR_FLAG_A				  BIT(0)
#define   DEC_ERR_FLAG_B				  BIT(1)
#define   DEC_ERR_FLAG_C				  BIT(2)
#define   DEC_ERR_FLAG_D				  BIT(3)
#define MAX96724_TOP_CTRL_INTR4				  0x0027
#define   LFLT_INT_OEN					  BIT(2)
#define   EOM_ERR_OEN_A					  BIT(4)
#define   EOM_ERR_OEN_B					  BIT(5)
#define   EOM_ERR_OEN_C					  BIT(6)
#define   EOM_ERR_OEN_D					  BIT(7)
#define MAX96724_TOP_CTRL_INTR5				  0x0028
#define   LFLT_INT					  BIT(2)
#define   EOM_ERR_FLAG_A				  BIT(4)
#define   EOM_ERR_FLAG_B				  BIT(5)
#define   EOM_ERR_FLAG_C				  BIT(6)
#define   EOM_ERR_FLAG_D				  BIT(7)
#define MAX96724_TOP_CTRL_INTR6				  0x0029
#define   FSYNC_ERR_OEN					  BIT(0)
#define   REM_ERR_OEN					  BIT(1)
#define   VPRBS_ERR_OEN					  BIT(2)
#define   LCRC_ERR_OEN					  BIT(3)
#define   G1_A_ERR_OEN					  BIT(4)
#define   G1_B_ERR_OEN					  BIT(5)
#define   G1_C_ERR_OEN					  BIT(6)
#define   G1_D_ERR_OEN					  BIT(7)
#define MAX96724_TOP_CTRL_INTR7				  0x002a
#define   FSYNC_ERR_FLAG				  BIT(0)
#define   REM_ERR_FLAG					  BIT(1)
#define   VPRBS_ERR_FLAG				  BIT(2)
#define   LCRC_ERR_FLAG					  BIT(3)
#define   G1_A_ERR_FLAG					  BIT(4)
#define   G1_B_ERR_FLAG					  BIT(5)
#define   G1_C_ERR_FLAG					  BIT(6)
#define   G1_D_ERR_FLAG					  BIT(7)
#define MAX96724_TOP_CTRL_INTR8				  0x002b
#define   IDLE_ERR_OEN_A				  BIT(0)
#define   IDLE_ERR_OEN_B				  BIT(1)
#define   IDLE_ERR_OEN_C				  BIT(2)
#define   IDLE_ERR_OEN_D				  BIT(3)
#define MAX96724_TOP_CTRL_INTR9				  0x002c
#define   IDLE_ERR_FLAG_A				  BIT(0)
#define   IDLE_ERR_FLAG_B				  BIT(1)
#define   IDLE_ERR_FLAG_C				  BIT(2)
#define   IDLE_ERR_FLAG_D				  BIT(3)
#define MAX96724_TOP_CTRL_INTR10			  0x002d
#define   MAX_RT_OEN_A					  BIT(0)
#define   MAX_RT_OEN_B					  BIT(1)
#define   MAX_RT_OEN_C					  BIT(2)
#define   MAX_RT_OEN_D					  BIT(3)
#define   RT_CNT_OEN_A					  BIT(4)
#define   RT_CNT_OEN_B					  BIT(5)
#define   RT_CNT_OEN_C					  BIT(6)
#define   RT_CNT_OEN_D					  BIT(7)
#define MAX96724_TOP_CTRL_INTR11			  0x002e
#define   MAX_RT_FLAG_A					  BIT(0)
#define   MAX_RT_FLAG_B					  BIT(1)
#define   MAX_RT_FLAG_C					  BIT(2)
#define   MAX_RT_FLAG_D					  BIT(3)
#define   RT_CNT_FLAG_A					  BIT(4)
#define   RT_CNT_FLAG_B					  BIT(5)
#define   RT_CNT_FLAG_C					  BIT(6)
#define   RT_CNT_FLAG_D					  BIT(7)
#define MAX96724_TOP_CTRL_INTR12			  0x002f
#define   ERR_TX_ID_MASK				  GENMASK(4, 0)
#define   ERR_TX_ID_SHIFT				  0
#define   ERR_TX_EN					  BIT(7)
#define MAX96724_TOP_CTRL_INTR13			  0x0030
#define   ERR_RX_ID_A_MASK				  GENMASK(4, 0)
#define   ERR_RX_ID_A_SHIFT				  0
#define   ERR_RX_RECVED_A				  BIT(6)
#define   ERR_RX_EN_A					  BIT(7)
#define MAX96724_TOP_CTRL_INTR14			  0x0031
#define   ERR_RX_ID_B_MASK				  GENMASK(4, 0)
#define   ERR_RX_ID_B_SHIFT				  0
#define   ERR_RX_RECVED_B				  BIT(6)
#define   ERR_RX_EN_B					  BIT(7)
#define MAX96724_TOP_CTRL_INTR15			  0x0032
#define   ERR_RX_ID_C_MASK				  GENMASK(4, 0)
#define   ERR_RX_ID_C_SHIFT				  0
#define   ERR_RX_RECVED_C				  BIT(6)
#define   ERR_RX_EN_C					  BIT(7)
#define MAX96724_TOP_CTRL_INTR16			  0x0033
#define   ERR_RX_ID_D_MASK				  GENMASK(4, 0)
#define   ERR_RX_ID_D_SHIFT				  0
#define   ERR_RX_RECVED_D				  BIT(6)
#define   ERR_RX_EN_D					  BIT(7)
#define MAX96724_TOP_CTRL_DEC_ERR_A			  0x0035
#define MAX96724_TOP_CTRL_DEC_ERR_B			  0x0036
#define MAX96724_TOP_CTRL_DEC_ERR_C			  0x0037
#define MAX96724_TOP_CTRL_DEC_ERR_D			  0x0038
#define MAX96724_TOP_CTRL_IDLE_ERR_A			  0x0039
#define MAX96724_TOP_CTRL_IDLE_ERR_B			  0x003a
#define MAX96724_TOP_CTRL_IDLE_ERR_C			  0x003b
#define MAX96724_TOP_CTRL_IDLE_ERR_D			  0x003c
#define MAX96724_TOP_CTRL_VID_PXL_CRC_ERR_VIDEOMASK_OEN	  0x0044
#define   VID_PXL_CRC_ERR_OEN_A				  BIT(0)
#define   VID_PXL_CRC_ERR_OEN_B				  BIT(1)
#define   VID_PXL_CRC_ERR_OEN_C				  BIT(2)
#define   VID_PXL_CRC_ERR_OEN_D				  BIT(3)
#define   VIDEO_MASKED_0_OEN				  BIT(4)
#define   VIDEO_MASKED_1_OEN				  BIT(5)
#define   VIDEO_MASKED_2_OEN				  BIT(6)
#define   VIDEO_MASKED_3_OEN				  BIT(7)
#define MAX96724_TOP_CTRL_VID_PXL_CRC_VIDEOMASK_INT_FLAG  0x0045
#define   VID_PXL_CRC_ERR_A				  BIT(0)
#define   VID_PXL_CRC_ERR_B				  BIT(1)
#define   VID_PXL_CRC_ERR_C				  BIT(2)
#define   VID_PXL_CRC_ERR_D				  BIT(3)
#define   VIDEO_MASKED_0_FLAG				  BIT(4)
#define   VIDEO_MASKED_1_FLAG				  BIT(5)
#define   VIDEO_MASKED_2_FLAG				  BIT(6)
#define   VIDEO_MASKED_3_FLAG				  BIT(7)
#define MAX96724_TOP_CTRL_PWR_STATUS_OEN		  0x0048
#define   VDDBAD_INT_OEN				  BIT(7)
#define MAX96724_TOP_CTRL_PWR_STATUS_OV_FLAG		  0x0049
#define   CMP_STATUS_VDD18_OV				  BIT(0)
#define   CMP_STATUS_VDDIO_OV				  BIT(1)
#define   CMP_STATUS_VDD12_OV				  BIT(2)
#define   CMP_STATUS_VDD_OV				  BIT(3)
#define   VDDBAD_INT_FLAG				  BIT(7)
#define MAX96724_TOP_CTRL_VDDCMP_MASK			  0x004a
#define   VDDCMP_MASK_MASK				  GENMASK(4, 0)
#define   VDDCMP_MASK_SHIFT				  0
#define   CMP_VTERM_MASK				  BIT(5)
#define   VDDCMP_INT_OEN				  BIT(7)
#define MAX96724_TOP_CTRL_VDDCMP_STATUS_FLAG		  0x004b
#define   CMP_VTERM_STATUS				  BIT(5)
#define   VDDCMP_INT_FLAG				  BIT(7)
#define MAX96724_TOP_CTRL_DEV_REV			  0x004c
#define   DEV_REV_MASK					  GENMASK(3, 0)
#define   DEV_REV_SHIFT					  0
#define MAX96724_TOP_CTRL_EFUSE_CTRL			  0x004d
#define   EFUSE_CRC_ERR_OEN				  BIT(4)
#define   EFUSE_CRC_ERR_RST				  BIT(5)
#define   EFUSE_CRC_ERR_RST_OS				  BIT(6)
#define MAX96724_TOP_CTRL_EFUSE_CRC_ERR			  0x004e
#define   EFUSE_CRC_ERR					  BIT(4)

/* CFGH_VIDEO_CRC */
#define MAX96724_CFGH_VIDEO_CRC_RX_CRC_EN_A_B		  0x0060
#define MAX96724_CFGH_VIDEO_CRC_RX_CRC_EN_C_D		  0x0061

/* CFGI_INFOFR: 0 <= link < 4 */
#define MAX96724_CFGI_INFOFR_TR0(link)			  (0x0070 + (link) * 0x04)
#define   PRIO_VAL_MASK					  GENMASK(3, 2)
#define   PRIO_VAL_SHIFT				  2
#define   RX_CRC_EN					  BIT(6)
#define   TX_CRC_EN					  BIT(7)
#define MAX96724_CFGI_INFOFR_TR1(link)			  (0x0071 + (link) * 0x04)
#define   BW_VAL_MASK					  GENMASK(5, 0)
#define   BW_VAL_SHIFT					  0
#define   BW_MULT_MASK					  GENMASK(7, 6)
#define   BW_MULT_SHIFT					  6
#define MAX96724_CFGI_INFOFR_TR2(link)			  (0x0072 + (link) * 0x04)
#define   TX_SRC_ID_MASK				  GENMASK(2, 0)
#define   TX_SRC_ID_SHIFT				  0
#define MAX96724_CFGI_INFOFR_RX_SRC_SEL(link)		  (0x0073 + (link) * 0x04)

/* CFGL_GPIO: 0 <= link < 4 */
#define MAX96724_CFGL_GPIO_TR0(link)			  (0x00a0 + (link) * 0x08)
#define   GPIO_PRIO_VAL_MASK				  GENMASK(3, 2)
#define   GPIO_PRIO_VAL_SHIFT				  2
#define   GPIO_RX_CRC_EN				  BIT(6)
#define   GPIO_TX_CRC_EN				  BIT(7)
#define MAX96724_CFGL_GPIO_TR1(link)			  (0x00a1 + (link) * 0x08)
#define   GPIO_BW_VAL_MASK				  GENMASK(5, 0)
#define   GPIO_BW_VAL_SHIFT				  0
#define   GPIO_BW_MULT_MASK				  GENMASK(7, 6)
#define   GPIO_BW_MULT_SHIFT				  6
#define MAX96724_CFGL_GPIO_TR3(link)			  (0x00a3 + (link) * 0x08)
#define   GPIO_TX_SRC_ID_MASK				  GENMASK(2, 0)
#define   GPIO_TX_SRC_ID_SHIFT				  0
#define MAX96724_CFGL_GPIO_RX_SRC_SEL(link)		  (0x00a4 + (link) * 0x08)
#define MAX96724_CFGL_GPIO_ARQ1(link)			  (0x00a6 + (link) * 0x08)
#define   RT_CNT_OEN					  BIT(0)
#define   MAX_RT_ERR_OEN				  BIT(1)
#define   MAX_RT_MASK					  GENMASK(6, 4)
#define   MAX_RT_SHIFT					  4
#define MAX96724_CFGL_GPIO_ARQ2(link)			  (0x00a7 + (link) * 0x08)
#define   RT_CNT_MASK					  GENMASK(6, 0)
#define   RT_CNT_SHIFT					  0
#define   MAX_RT_ERR					  BIT(7)

/* CC */
#define MAX96724_CC_I2C_7				  0x00c7
#define   I2C_INTREG_SLV_0_TO_MASK			  GENMASK(2, 0)
#define   I2C_INTREG_SLV_0_TO_SHIFT			  0
#define   I2C_REGSLV_0_TIMED_OUT			  BIT(3)
#define   I2C_INTREG_SLV_1_TO_MASK			  GENMASK(6, 4)
#define   I2C_INTREG_SLV_1_TO_SHIFT			  4
#define   I2C_REGSLV_1_TIMED_OUT			  BIT(7)

/* LINE_FAULT */
#define MAX96724_LINE_FAULT_REG0			  0x00e0
#define   PU_LF0					  BIT(0)
#define   PU_LF1					  BIT(1)
#define   PU_LF2					  BIT(2)
#define   PU_LF3					  BIT(3)
#define MAX96724_LINE_FAULT_REG1			  0x00e1
#define   LF_0_MASK					  GENMASK(2, 0)
#define   LF_0_SHIFT					  0
#define   LF_1_MASK					  GENMASK(6, 4)
#define   LF_1_SHIFT					  4
#define MAX96724_LINE_FAULT_REG2			  0x00e2
#define   LF_2_MASK					  GENMASK(2, 0)
#define   LF_2_SHIFT					  0
#define   LF_3_MASK					  GENMASK(6, 4)
#define   LF_3_SHIFT					  4
#define MAX96724_LINE_FAULT_REG5			  0x00e5
#define   LFLT_INT_FLAG_MASK				  GENMASK(3, 0)
#define   LFLT_INT_FLAG_SHIFT				  0
#define MAX96724_LINE_FAULT_REG6			  0x00e6
#define   MASK_LF0					  BIT(0)
#define   MASK_LF1					  BIT(1)
#define   MASK_LF2					  BIT(2)
#define   MASK_LF3					  BIT(3)

/* VIDEO_PIPE_SEL */
#define MAX96724_VIDEO_PIPE_SEL_VIDEO_PIPE_SEL_0	  0x00f0
#define   VIDEO_PIPE_SEL_0_MASK				  GENMASK(3, 0)
#define   VIDEO_PIPE_SEL_0_SHIFT			  0
#define   VIDEO_PIPE_SEL_1_MASK				  GENMASK(7, 4)
#define   VIDEO_PIPE_SEL_1_SHIFT			  4
#define MAX96724_VIDEO_PIPE_SEL_VIDEO_PIPE_SEL_1	  0x00f1
#define   VIDEO_PIPE_SEL_2_MASK				  GENMASK(3, 0)
#define   VIDEO_PIPE_SEL_2_SHIFT			  0
#define   VIDEO_PIPE_SEL_3_MASK				  GENMASK(7, 4)
#define   VIDEO_PIPE_SEL_3_SHIFT			  4
#define MAX96724_VIDEO_PIPE_SEL_VIDEO_PIPE_EN		  0x00f4
#define   VIDEO_PIPE_EN_MASK				  GENMASK(3, 0)
#define   VIDEO_PIPE_EN_SHIFT				  0
#define   STREAM_SEL_ALL				  BIT(4)

/* HVD_GPIO_CTRL */
#define MAX96724_HVD_GPIO_CTRL_HVD_GPIO_CTRL_EN		  0x00fa
#define   HVD_OUT_EN_MASK				  GENMASK(3, 0)
#define   HVD_OUT_EN_SHIFT				  0
#define MAX96724_HVD_GPIO_CTRL_HVD_GPIO_CTRL_HS		  0x00fb
#define   HVD_HS_SEL0_MASK				  GENMASK(1, 0)
#define   HVD_HS_SEL0_SHIFT				  0
#define   HVD_HS_SEL1_MASK				  GENMASK(3, 2)
#define   HVD_HS_SEL1_SHIFT				  2
#define   HVD_HS_SEL2_MASK				  GENMASK(5, 4)
#define   HVD_HS_SEL2_SHIFT				  4
#define   HVD_HS_SEL3_MASK				  GENMASK(7, 6)
#define   HVD_HS_SEL3_SHIFT				  6
#define MAX96724_HVD_GPIO_CTRL_HVD_GPIO_CTRL_VS		  0x00fc
#define   HVD_VS_SEL0_MASK				  GENMASK(1, 0)
#define   HVD_VS_SEL0_SHIFT				  0
#define   HVD_VS_SEL1_MASK				  GENMASK(3, 2)
#define   HVD_VS_SEL1_SHIFT				  2
#define   HVD_VS_SEL2_MASK				  GENMASK(5, 4)
#define   HVD_VS_SEL2_SHIFT				  4
#define   HVD_VS_SEL3_MASK				  GENMASK(7, 6)
#define   HVD_VS_SEL3_SHIFT				  6
#define MAX96724_HVD_GPIO_CTRL_HVD_GPIO_CTRL_DE		  0x00fd
#define   HVD_DE_SEL0_MASK				  GENMASK(1, 0)
#define   HVD_DE_SEL0_SHIFT				  0
#define   HVD_DE_SEL1_MASK				  GENMASK(3, 2)
#define   HVD_DE_SEL1_SHIFT				  2
#define   HVD_DE_SEL2_MASK				  GENMASK(5, 4)
#define   HVD_DE_SEL2_SHIFT				  4
#define   HVD_DE_SEL3_MASK				  GENMASK(7, 6)
#define   HVD_DE_SEL3_SHIFT				  6
#define MAX96724_HVD_GPIO_CTRL_HVD_GPIO_CTRL_SEL	  0x00fe
#define   HVD_OUT_SEL0_MASK				  GENMASK(1, 0)
#define   HVD_OUT_SEL0_SHIFT				  0
#define   HVD_OUT_SEL1_MASK				  GENMASK(3, 2)
#define   HVD_OUT_SEL1_SHIFT				  2
#define   HVD_OUT_SEL2_MASK				  GENMASK(5, 4)
#define   HVD_OUT_SEL2_SHIFT				  4
#define   HVD_OUT_SEL3_MASK				  GENMASK(7, 6)
#define   HVD_OUT_SEL3_SHIFT				  6
#define MAX96724_HVD_GPIO_CTRL_HVD_GPIO_CTRL_ST		  0x00ff
#define   HVD_ST_SEL0_MASK				  GENMASK(1, 0)
#define   HVD_ST_SEL0_SHIFT				  0
#define   HVD_ST_SEL1_MASK				  GENMASK(3, 2)
#define   HVD_ST_SEL1_SHIFT				  2
#define   HVD_ST_SEL2_MASK				  GENMASK(5, 4)
#define   HVD_ST_SEL2_SHIFT				  4
#define   HVD_ST_SEL3_MASK				  GENMASK(7, 6)
#define   HVD_ST_SEL3_SHIFT				  6

/* VID_RX: 0 <= link < 4 */
#define MAX96724_VID_RX_VIDEO_RX0(link)			  (0x0100 + (link) * 0x12)
#define   DIS_PKT_DET					  BIT(0)
#define   LINE_CRC_EN					  BIT(1)
#define   SEQ_MISS_EN					  BIT(4)
#define   LCRC_ERR					  BIT(7)
#define MAX96724_VID_RX_VIDEO_RX6(link)			  (0x0106 + (link) * 0x12)
#define   LIM_HEART					  BIT(3)
#define   VID_SEQ_ERR_OEN				  BIT(4)
#define MAX96724_VID_RX_VIDEO_RX8(link)			  (0x0108 + (link) * 0x12)
#define   VID_SEQ_ERR					  BIT(4)
#define   VID_PKT_DET					  BIT(5)
#define   VID_LOCK					  BIT(6)

/* VID_RX_PKT_DET */
#define MAX96724_VID_RX_PKT_DET_LIM_HEART_TIMEOUT_0	  0x0160
#define   LIM_HEART_TIMEOUT_0_MASK			  GENMASK(6, 0)
#define   LIM_HEART_TIMEOUT_0_SHIFT			  0
#define MAX96724_VID_RX_PKT_DET_LIM_HEART_TIMEOUT_1	  0x0161
#define   LIM_HEART_TIMEOUT_1_MASK			  GENMASK(6, 0)
#define   LIM_HEART_TIMEOUT_1_SHIFT			  0
#define MAX96724_VID_RX_PKT_DET_LIM_HEART_TIMEOUT_2	  0x0162
#define   LIM_HEART_TIMEOUT_2_MASK			  GENMASK(6, 0)
#define   LIM_HEART_TIMEOUT_2_SHIFT			  0
#define MAX96724_VID_RX_PKT_DET_LIM_HEART_TIMEOUT_3	  0x0163
#define   LIM_HEART_TIMEOUT_3_MASK			  GENMASK(6, 0)
#define   LIM_HEART_TIMEOUT_3_SHIFT			  0

/* VRX: 0 <= link < 4 */
#define MAX96724_VRX_CROSS_HS(link)			  (0x01d8 + (link) * 0x20)
#define   CROSS_HS_MASK					  GENMASK(4, 0)
#define   CROSS_HS_SHIFT				  0
#define   CROSS_HS_F					  BIT(5)
#define   CROSS_HS_I					  BIT(6)
#define MAX96724_VRX_CROSS_VS(link)			  (0x01d9 + (link) * 0x20)
#define   CROSS_VS_MASK					  GENMASK(4, 0)
#define   CROSS_VS_SHIFT				  0
#define   CROSS_VS_F					  BIT(5)
#define   CROSS_VS_I					  BIT(6)
#define MAX96724_VRX_CROSS_DE(link)			  (0x01da + (link) * 0x20)
#define   CROSS_DE_MASK					  GENMASK(4, 0)
#define   CROSS_DE_SHIFT				  0
#define   CROSS_DE_F					  BIT(5)
#define   CROSS_DE_I					  BIT(6)
#define MAX96724_VRX_VPRBS_ERR(link)			  (0x01db + (link) * 0x20)
#define MAX96724_VRX_VPRBS(link)			  (0x01dc + (link) * 0x20)
#define   VIDEO_LOCK					  BIT(0)
#define   DIS_GLITCH_FILT				  BIT(1)
#define   VPRBS9_GENCHK_EN				  BIT(2)
#define   VPRBS7_GENCHK_EN				  BIT(3)
#define   VPRBS24_GENCHK_EN				  BIT(4)
#define   VPRBS_FAIL					  BIT(5)
#define   VPRBS_CHECK					  BIT(6)
#define   PATGEN_CLK_SRC				  BIT(7)

/* GPIO_AGGR0 */
#define MAX96724_GPIO_AGGR0_POLARITY_A_L		  0x02e0
#define MAX96724_GPIO_AGGR0_POLARITY_B_L		  0x02e1
#define MAX96724_GPIO_AGGR0_POLARITY_C_L		  0x02e2
#define MAX96724_GPIO_AGGR0_POLARITY_D_L		  0x02e3
#define MAX96724_GPIO_AGGR0_POLARITY_AB_H		  0x02e4
#define   POLARITY_A_H_MASK				  GENMASK(2, 0)
#define   POLARITY_A_H_SHIFT				  0
#define   POLARITY_B_H_MASK				  GENMASK(6, 4)
#define   POLARITY_B_H_SHIFT				  4
#define MAX96724_GPIO_AGGR0_POLARITY_CD_H		  0x02e5
#define   POLARITY_C_H_MASK				  GENMASK(2, 0)
#define   POLARITY_C_H_SHIFT				  0
#define   POLARITY_D_H_MASK				  GENMASK(6, 4)
#define   POLARITY_D_H_SHIFT				  4
#define MAX96724_GPIO_AGGR0_ENABLE_A_L			  0x02e6
#define MAX96724_GPIO_AGGR0_ENABLE_B_L			  0x02e7
#define MAX96724_GPIO_AGGR0_ENABLE_C_L			  0x02e8
#define MAX96724_GPIO_AGGR0_ENABLE_D_L			  0x02e9
#define MAX96724_GPIO_AGGR0_ENABLE_AB_H			  0x02ea
#define   ENABLE_A_H_MASK				  GENMASK(2, 0)
#define   ENABLE_A_H_SHIFT				  0
#define   ENABLE_B_H_MASK				  GENMASK(6, 4)
#define   ENABLE_B_H_SHIFT				  4
#define MAX96724_GPIO_AGGR0_ENABLE_CD_H			  0x02eb
#define   ENABLE_C_H_MASK				  GENMASK(2, 0)
#define   ENABLE_C_H_SHIFT				  0
#define   ENABLE_D_H_MASK				  GENMASK(6, 4)
#define   ENABLE_D_H_SHIFT				  4
#define MAX96724_GPIO_AGGR0_READ_A_L			  0x02ec
#define MAX96724_GPIO_AGGR0_READ_B_L			  0x02ed
#define MAX96724_GPIO_AGGR0_READ_C_L			  0x02ee
#define MAX96724_GPIO_AGGR0_READ_D_L			  0x02ef
#define MAX96724_GPIO_AGGR0_READ_AB_H			  0x02f0
#define   READ_A_H_MASK					  GENMASK(2, 0)
#define   READ_A_H_SHIFT				  0
#define   READ_B_H_MASK					  GENMASK(6, 4)
#define   READ_B_H_SHIFT				  4
#define MAX96724_GPIO_AGGR0_READ_CD_H			  0x02f1
#define   READ_C_H_MASK					  GENMASK(2, 0)
#define   READ_C_H_SHIFT				  0
#define   READ_D_H_MASK					  GENMASK(6, 4)
#define   READ_D_H_SHIFT				  4
#define MAX96724_GPIO_AGGR0_OUTPUT			  0x02f2
#define   READ_FLAG					  BIT(0)
#define   DESTINATION					  BIT(1)
#define   OUTPUT_ENABLE					  BIT(2)
#define   OUTPUT_INVERT					  BIT(3)

/* GPIO_A: 0 <= gpio < 11 */
#define MAX96724_GPIO_A_A(gpio)				  (0x0300 + (gpio) * 0x03)
#define   GPIO_OUT_DIS					  BIT(0)
#define   GPIO_TX_EN_A					  BIT(1)
#define   GPIO_RX_EN_A					  BIT(2)
#define   GPIO_IN					  BIT(3)
#define   GPIO_OUT					  BIT(4)
#define   TX_COMP_EN_A					  BIT(5)
#define   RES_CFG					  BIT(7)
#define MAX96724_GPIO_A_B(gpio)				  (0x0301 + (gpio) * 0x03)
#define   GPIO_TX_ID_A_MASK				  GENMASK(4, 0)
#define   GPIO_TX_ID_A_SHIFT				  0
#define   OUT_TYPE					  BIT(5)
#define   PULL_UPDN_SEL_MASK				  GENMASK(7, 6)
#define   PULL_UPDN_SEL_SHIFT				  6
#define MAX96724_GPIO_A_C(gpio)				  (0x0302 + (gpio) * 0x03)
#define   GPIO_RX_ID_A_MASK				  GENMASK(4, 0)
#define   GPIO_RX_ID_A_SHIFT				  0
#define   GPIO_RECVED					  BIT(6)
#define   OVR_RES_CFG					  BIT(7)

/* GPIO_B: 0 <= gpio < 11 */
#define MAX96724_GPIO_B_B(gpio)				  (0x0337 + (gpio) * 0x03)
#define   GPIO_TX_ID_B_MASK				  GENMASK(4, 0)
#define   GPIO_TX_ID_B_SHIFT				  0
#define   GPIO_TX_EN_B					  BIT(5)
#define   TX_COMP_EN_B					  BIT(6)
#define MAX96724_GPIO_B_C(gpio)				  (0x0338 + (gpio) * 0x03)
#define   GPIO_RX_ID_B_MASK				  GENMASK(4, 0)
#define   GPIO_RX_ID_B_SHIFT				  0
#define   GPIO_RX_EN_B					  BIT(5)
#define   GPIO_RECVED_B					  BIT(6)

/* GPIO_C: 0 <= gpio < 11 */
#define MAX96724_GPIO_C_B(gpio)				  (0x036d + (gpio) * 0x03)
#define   GPIO_TX_ID_C_MASK				  GENMASK(4, 0)
#define   GPIO_TX_ID_C_SHIFT				  0
#define   GPIO_TX_EN_C					  BIT(5)
#define   TX_COMP_EN_C					  BIT(6)
#define MAX96724_GPIO_C_C(gpio)				  (0x036e + (gpio) * 0x03)
#define   GPIO_RX_ID_C_MASK				  GENMASK(4, 0)
#define   GPIO_RX_ID_C_SHIFT				  0
#define   GPIO_RX_EN_C					  BIT(5)
#define   GPIO_RECVED_C					  BIT(6)

/* GPIO_D: 0 <= gpio < 11 */
#define MAX96724_GPIO0_D_B(gpio)			  (0x03a4 + (gpio) * 0x03)
#define   GPIO_TX_ID_D_MASK				  GENMASK(4, 0)
#define   GPIO_TX_ID_D_SHIFT				  0
#define   GPIO_TX_EN_D					  BIT(5)
#define   TX_COMP_EN_D					  BIT(6)
#define MAX96724_GPIO0_D_C(gpio)			  (0x03a5 + (gpio) * 0x03)
#define   GPIO_RX_ID_D_MASK				  GENMASK(4, 0)
#define   GPIO_RX_ID_D_SHIFT				  0
#define   GPIO_RX_EN_D					  BIT(5)
#define   GPIO_RECVED_D					  BIT(6)

/* BACKTOP0 */
#define MAX96724_BACKTOP0_1				  0x0400
#define   CSIPLL0_LOCK					  BIT(4)
#define   CSIPLL1_LOCK					  BIT(5)
#define   CSIPLL2_LOCK					  BIT(6)
#define   CSIPLL3_LOCK					  BIT(7)
#define MAX96724_BACKTOP0_VS_VC0_L			  0x0401
#define MAX96724_BACKTOP0_VS_VC0_H			  0x0402
#define MAX96724_BACKTOP0_VS_VC1_L			  0x0403
#define MAX96724_BACKTOP0_VS_VC1_H			  0x0404
#define MAX96724_BACKTOP0_VS_VC2_L			  0x0405
#define MAX96724_BACKTOP0_VS_VC2_H			  0x0406
#define MAX96724_BACKTOP0_VS_VC3_L			  0x0407
#define MAX96724_BACKTOP0_VS_VC3_H			  0x0408
#define MAX96724_BACKTOP0_10				  0x0409
#define   DE_SEL0					  BIT(4)
#define   DE_SEL1					  BIT(5)
#define   DE_SEL2					  BIT(6)
#define   DE_SEL3					  BIT(7)
#define MAX96724_BACKTOP0_11				  0x040a
#define   LMO_0						  BIT(0)
#define   LMO_1						  BIT(1)
#define   LMO_2						  BIT(2)
#define   LMO_3						  BIT(3)
#define   CMD_OVERFLOW0					  BIT(4)
#define   CMD_OVERFLOW1					  BIT(5)
#define   CMD_OVERFLOW2					  BIT(6)
#define   CMD_OVERFLOW3					  BIT(7)
#define MAX96724_BACKTOP0_12				  0x040b
#define   CSI_OUT_EN					  BIT(1)
#define   SOFT_BPP_0_MASK				  GENMASK(7, 3)
#define   SOFT_BPP_0_SHIFT				  3
#define MAX96724_BACKTOP0_13				  0x040c
#define   SOFT_VC_0_MASK				  GENMASK(3, 0)
#define   SOFT_VC_0_SHIFT				  0
#define   SOFT_VC_1_MASK				  GENMASK(7, 4)
#define   SOFT_VC_1_SHIFT				  4
#define MAX96724_BACKTOP0_14				  0x040d
#define   SOFT_VC_2_MASK				  GENMASK(3, 0)
#define   SOFT_VC_2_SHIFT				  0
#define   SOFT_VC_3_MASK				  GENMASK(7, 4)
#define   SOFT_VC_3_SHIFT				  4
#define MAX96724_BACKTOP0_15				  0x040e
#define   SOFT_DT_0_MASK				  GENMASK(5, 0)
#define   SOFT_DT_0_SHIFT				  0
#define   SOFT_DT_1_H_MASK				  GENMASK(7, 6)
#define   SOFT_DT_1_H_SHIFT				  6
#define MAX96724_BACKTOP0_16				  0x040f
#define   SOFT_DT_1_L_MASK				  GENMASK(3, 0)
#define   SOFT_DT_1_L_SHIFT				  0
#define   SOFT_DT_2_H_MASK				  GENMASK(7, 4)
#define   SOFT_DT_2_H_SHIFT				  4
#define MAX96724_BACKTOP0_17				  0x0410
#define   SOFT_DT_2_L_MASK				  GENMASK(1, 0)
#define   SOFT_DT_2_L_SHIFT				  0
#define   SOFT_DT_3_MASK				  GENMASK(7, 2)
#define   SOFT_DT_3_SHIFT				  2
#define MAX96724_BACKTOP0_18				  0x0411
#define   SOFT_BPP_1_MASK				  GENMASK(4, 0)
#define   SOFT_BPP_1_SHIFT				  0
#define   SOFT_BPP_2_H_MASK				  GENMASK(7, 5)
#define   SOFT_BPP_2_H_SHIFT				  5
#define MAX96724_BACKTOP0_19				  0x0412
#define   SOFT_BPP_2_L_MASK				  GENMASK(1, 0)
#define   SOFT_BPP_2_L_SHIFT				  0
#define   SOFT_BPP_3_MASK				  GENMASK(6, 2)
#define   SOFT_BPP_3_SHIFT				  2
#define MAX96724_BACKTOP0_PHY0_CSI_TX_DPLL_L		  0x0413
#define MAX96724_BACKTOP0_21				  0x0414
#define   PHY0_CSI_TX_DPLL_H_MASK			  GENMASK(3, 0)
#define   PHY0_CSI_TX_DPLL_H_SHIFT			  0
#define   BPP8DBL0					  BIT(4)
#define   BPP8DBL1					  BIT(5)
#define   BPP8DBL2					  BIT(6)
#define   BPP8DBL3					  BIT(7)
#define MAX96724_BACKTOP0_22				  0x0415
#define   PHY0_CSI_TX_DPLL_PREDEF_FREQ_MASK		  GENMASK(4, 0)
#define   PHY0_CSI_TX_DPLL_PREDEF_FREQ_SHIFT		  0
#define   PHY0_CSI_TX_DPLL_FB_FRACTION_PREDEF_EN	  BIT(5)
#define   OVERRIDE_BPP_VC_DT_0				  BIT(6)
#define   OVERRIDE_BPP_VC_DT_1				  BIT(7)
#define MAX96724_BACKTOP0_PHY1_CSI_TX_DPLL_L		  0x0416
#define MAX96724_BACKTOP0_24				  0x0417
#define   PHY1_CSI_TX_DPLL_FB_FRACTION_IN_H_MASK	  GENMASK(3, 0)
#define   PHY1_CSI_TX_DPLL_FB_FRACTION_IN_H_SHIFT	  0
#define   BPP8DBL0_MODE					  BIT(4)
#define   BPP8DBL1_MODE					  BIT(5)
#define   BPP8DBL2_MODE					  BIT(6)
#define   BPP8DBL3_MODE					  BIT(7)
#define MAX96724_BACKTOP0_25				  0x0418
#define   PHY1_CSI_TX_DPLL_PREDEF_FREQ_MASK		  GENMASK(4, 0)
#define   PHY1_CSI_TX_DPLL_PREDEF_FREQ_SHIFT		  0
#define   PHY1_CSI_TX_DPLL_FB_FRACTION_PREDEF_EN	  BIT(5)
#define   OVERRIDE_BPP_VC_DT_2				  BIT(6)
#define   OVERRIDE_BPP_VC_DT_3				  BIT(7)
#define MAX96724_BACKTOP0_PHY2_CSI_TX_DPLL_L		  0x0419
#define MAX96724_BACKTOP0_27				  0x041a
#define   PHY2_CSI_TX_DPLL_FB_FRACTION_IN_H_MASK	  GENMASK(3, 0)
#define   PHY2_CSI_TX_DPLL_FB_FRACTION_IN_H_SHIFT	  0
#define   YUV_8_10_MUX_MODE0				  BIT(4)
#define   YUV_8_10_MUX_MODE1				  BIT(5)
#define   YUV_8_10_MUX_MODE2				  BIT(6)
#define   YUV_8_10_MUX_MODE3				  BIT(7)
#define MAX96724_BACKTOP0_28				  0x041b
#define   PHY2_CSI_TX_DPLL_PREDEF_FREQ_MASK		  GENMASK(4, 0)
#define   PHY2_CSI_TX_DPLL_PREDEF_FREQ_SHIFT		  0
#define   PHY2_CSI_TX_DPLL_FB_FRACTION_PREDEF_EN	  BIT(5)
#define MAX96724_BACKTOP0_PHY3_CSI_TX_DPLL_L		  0x041c
#define MAX96724_BACKTOP0_30				  0x041d
#define   PHY3_CSI_TX_DPLL_FB_FRACTION_IN_H_MASK	  GENMASK(3, 0)
#define   PHY3_CSI_TX_DPLL_FB_FRACTION_IN_H_SHIFT	  0
#define   BPP10DBL3					  BIT(4)
#define   BPP10DBL3_MODE				  BIT(5)
#define MAX96724_BACKTOP0_31				  0x041e
#define   PHY3_CSI_TX_DPLL_PREDEF_FREQ_MASK		  GENMASK(4, 0)
#define   PHY3_CSI_TX_DPLL_PREDEF_FREQ_SHIFT		  0
#define   PHY3_CSI_TX_DPLL_FB_FRACTION_PREDEF_EN	  BIT(5)
#define   BPP10DBL2					  BIT(6)
#define   BPP10DBL2_MODE				  BIT(7)
#define MAX96724_BACKTOP0_32				  0x041f
#define   BPP12DBL0					  BIT(0)
#define   BPP12DBL1					  BIT(1)
#define   BPP12DBL2					  BIT(2)
#define   BPP12DBL3					  BIT(3)
#define   BPP10DBL0					  BIT(4)
#define   BPP10DBL0_MODE				  BIT(5)
#define   BPP10DBL1					  BIT(6)
#define   BPP10DBL1_MODE				  BIT(7)

/* BACKTOP_1 */
#define MAX96724_BACKTOP1_1				  0x0420
#define   ERRB_PKT_EN_MASK				  GENMASK(7, 4)
#define   ERRB_PKT_EN_SHIFT				  4
#define MAX96724_BACKTOP1_2				  0x0421
#define   ERRB_PKT_INSERT_MODE_1_MASK			  GENMASK(1, 0)
#define   ERRB_PKT_INSERT_MODE_1_SHIFT			  0
#define   ERRB_PKT_INSERT_MODE_2_MASK			  GENMASK(3, 2)
#define   ERRB_PKT_INSERT_MODE_2_SHIFT			  2
#define   ERRB_PKT_INSERT_MODE_3_MASK			  GENMASK(5, 4)
#define   ERRB_PKT_INSERT_MODE_3_SHIFT			  4
#define   ERRB_PKT_INSERT_MODE_4_MASK			  GENMASK(7, 6)
#define   ERRB_PKT_INSERT_MODE_4_SHIFT			  6
#define MAX96724_BACKTOP1_3				  0x0422
#define   ERRB_PKT_EDGE_SEL_1				  BIT(0)
#define   ERRB_PKT_EDGE_SEL_2				  BIT(2)
#define   ERRB_PKT_EDGE_SEL_3				  BIT(4)
#define   ERRB_PKT_EDGE_SEL_4				  BIT(6)
#define MAX96724_BACKTOP1_4				  0x0423
#define   ERRB_PKT_DT_1_MASK				  GENMASK(5, 0)
#define   ERRB_PKT_DT_1_SHIFT				  0
#define   ERRB_PKT_DBL_MODE_1				  BIT(7)
#define MAX96724_BACKTOP1_5				  0x0424
#define   ERRB_PKT_DT_2_MASK				  GENMASK(5, 0)
#define   ERRB_PKT_DT_2_SHIFT				  0
#define   ERRB_PKT_DBL_MODE_2				  BIT(7)
#define MAX96724_BACKTOP1_6				  0x0425
#define   ERRB_PKT_DT_3_MASK				  GENMASK(5, 0)
#define   ERRB_PKT_DT_3_SHIFT				  0
#define   ERRB_PKT_DBL_MODE_3				  BIT(7)
#define MAX96724_BACKTOP1_7				  0x0426
#define   ERRB_PKT_DT_4_MASK				  GENMASK(5, 0)
#define   ERRB_PKT_DT_4_SHIFT				  0
#define   ERRB_PKT_DBL_MODE_4				  BIT(7)
#define MAX96724_BACKTOP1_8				  0x0427
#define   ERRB_PKT_VC_OVRD_1_MASK			  GENMASK(4, 0)
#define   ERRB_PKT_VC_OVRD_1_SHIFT			  0
#define   ERRB_PKT_VC_OVRD_EN_1				  BIT(7)
#define MAX96724_BACKTOP1_9				  0x0428
#define   ERRB_PKT_VC_OVRD_2_MASK			  GENMASK(4, 0)
#define   ERRB_PKT_VC_OVRD_2_SHIFT			  0
#define   ERRB_PKT_VC_OVRD_EN_2				  BIT(7)
#define MAX96724_BACKTOP1_10				  0x0429
#define   ERRB_PKT_VC_OVRD_3_MASK			  GENMASK(4, 0)
#define   ERRB_PKT_VC_OVRD_3_SHIFT			  0
#define   ERRB_PKT_VC_OVRD_EN_3				  BIT(7)
#define MAX96724_BACKTOP1_11				  0x042a
#define   ERRB_PKT_VC_OVRD_4_MASK			  GENMASK(4, 0)
#define   ERRB_PKT_VC_OVRD_4_SHIFT			  0
#define   ERRB_PKT_VC_OVRD_EN_4				  BIT(7)
#define MAX96724_BACKTOP1_12				  0x042b
#define   ERRB_PKT_VC_1_MASK				  GENMASK(4, 0)
#define   ERRB_PKT_VC_1_SHIFT				  0
#define MAX96724_BACKTOP1_13				  0x042c
#define   ERRB_PKT_VC_2_MASK				  GENMASK(4, 0)
#define   ERRB_PKT_VC_2_SHIFT				  0
#define MAX96724_BACKTOP1_14				  0x042d
#define   ERRB_PKT_VC_3_MASK				  GENMASK(4, 0)
#define   ERRB_PKT_VC_3_SHIFT				  0
#define MAX96724_BACKTOP1_15				  0x042e
#define   ERRB_PKT_VC_4_MASK				  GENMASK(4, 0)
#define   ERRB_PKT_VC_4_SHIFT				  0
#define MAX96724_BACKTOP1_22				  0x0435
#define   N_VS_BLOCK_MASK				  GENMASK(3, 0)
#define   N_VS_BLOCK_SHIFT				  0
#define MAX96724_BACKTOP1_23				  0x0436
#define   DIS_VS0					  BIT(0)
#define   DIS_VS1					  BIT(1)
#define   DIS_VS2					  BIT(2)
#define   DIS_VS3					  BIT(3)
#define MAX96724_BACKTOP1_24				  0x0437
#define   ERRB_PKT_WC_OVRD_EN_1				  BIT(4)
#define   ERRB_PKT_WC_OVRD_EN_2				  BIT(5)
#define   ERRB_PKT_WC_OVRD_EN_3				  BIT(6)
#define   ERRB_PKT_WC_OVRD_EN_4				  BIT(7)
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_1_H		  0x0438
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_1_L		  0x0439
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_2_H		  0x043a
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_2_L		  0x043b
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_3_H		  0x043c
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_3_L		  0x043d
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_4_H		  0x043e
#define MAX96724_BACKTOP_1_ERRB_PKT_WC_4_L		  0x043f
#define MAX96724_BACKTOP1_33				  0x0440
#define   FIFO_EMPTY_0					  BIT(0)
#define   FIFO_EMPTY_1					  BIT(1)
#define   FIFO_EMPTY_2					  BIT(2)
#define   FIFO_EMPTY_3					  BIT(3)
#define MAX96724_BACKTOP1_HDR_ERR_1			  0x0442
#define   TUN_HDR_ECC_FLAG_0				  BIT(0)
#define   TUN_HDR_ECC_ERR_FLAG_0			  BIT(1)
#define   TUN_HDR_CRC_ERR_0_FLAG_0			  BIT(2)
#define   TUN_HDR_CRC_ERR_1_FLAG_0			  BIT(3)
#define   TUN_HDR_CRC_ERR_2_FLAG_0			  BIT(4)
#define   TUN_HDR_CRC_ERR_3_FLAG_0			  BIT(5)
#define   TUN_HDR_ERR_FLAG_0				  BIT(6)
#define   TUN_HDR_ERR_FLAG_0_OEN			  BIT(7)
#define MAX96724_BACKTOP1_HDR_ERR_2			  0x0443
#define   TUN_HDR_ECC_FLAG_1				  BIT(0)
#define   TUN_HDR_ECC_ERR_FLAG_1			  BIT(1)
#define   TUN_HDR_CRC_ERR_0_FLAG_1			  BIT(2)
#define   TUN_HDR_CRC_ERR_1_FLAG_1			  BIT(3)
#define   TUN_HDR_CRC_ERR_2_FLAG_1			  BIT(4)
#define   TUN_HDR_CRC_ERR_3_FLAG_1			  BIT(5)
#define   TUN_HDR_ERR_FLAG_1				  BIT(6)
#define   TUN_HDR_ERR_FLAG_1_OEN			  BIT(7)
#define MAX96724_BACKTOP1_HDR_ERR_3			  0x0444
#define   TUN_HDR_ECC_FLAG_2				  BIT(0)
#define   TUN_HDR_ECC_ERR_FLAG_2			  BIT(1)
#define   TUN_HDR_CRC_ERR_0_FLAG_2			  BIT(2)
#define   TUN_HDR_CRC_ERR_1_FLAG_2			  BIT(3)
#define   TUN_HDR_CRC_ERR_2_FLAG_2			  BIT(4)
#define   TUN_HDR_CRC_ERR_3_FLAG_2			  BIT(5)
#define   TUN_HDR_ERR_FLAG_2				  BIT(6)
#define   TUN_HDR_ERR_FLAG_2_OEN			  BIT(7)
#define MAX96724_BACKTOP1_HDR_ERR_4			  0x0445
#define   TUN_HDR_ECC_FLAG_3				  BIT(0)
#define   TUN_HDR_ECC_ERR_FLAG_3			  BIT(1)
#define   TUN_HDR_CRC_ERR_0_FLAG_3			  BIT(2)
#define   TUN_HDR_CRC_ERR_1_FLAG_3			  BIT(3)
#define   TUN_HDR_CRC_ERR_2_FLAG_3			  BIT(4)
#define   TUN_HDR_CRC_ERR_3_FLAG_3			  BIT(5)
#define   TUN_HDR_ERR_FLAG_3				  BIT(6)
#define   TUN_HDR_ERR_FLAG_3_OEN			  BIT(7)
#define MAX96724_BACKTOP1_39				  0x0446
#define   BKTP0_VM_TIMEOUT_OVRD				  BIT(0)
#define   BKTP1_VM_TIMEOUT_OVRD				  BIT(1)
#define   BKTP2_VM_TIMEOUT_OVRD				  BIT(2)
#define   BKTP3_VM_TIMEOUT_OVRD				  BIT(3)
#define   BKTP0_LINE_LEN_OVRD				  BIT(4)
#define   BKTP1_LINE_LEN_OVRD				  BIT(5)
#define   BKTP2_LINE_LEN_OVRD				  BIT(6)
#define   BKTP3_LINE_LEN_OVRD				  BIT(7)
#define MAX96724_BACKTOP_1_BKTP0_LINE_LEN_H		  0x0447
#define MAX96724_BACKTOP_1_BKTP0_LINE_LEN_L		  0x0448
#define MAX96724_BACKTOP_1_BKTP1_LINE_LEN_H		  0x0449
#define MAX96724_BACKTOP_1_BKTP1_LINE_LEN_L		  0x044a
#define MAX96724_BACKTOP_1_BKTP2_LINE_LEN_H		  0x044b
#define MAX96724_BACKTOP_1_BKTP2_LINE_LEN_L		  0x044c
#define MAX96724_BACKTOP_1_BKTP3_LINE_LEN_H		  0x044d
#define MAX96724_BACKTOP_1_BKTP3_LINE_LEN_L		  0x044e
#define MAX96724_BACKTOP1_48				  0x044f
#define   BKTP1_VM_TIMEOUT_DIV_MASK			  GENMASK(1, 0)
#define   BKTP1_VM_TIMEOUT_DIV_SHIFT			  0
#define   BKTP2_VM_TIMEOUT_DIV_MASK			  GENMASK(3, 2)
#define   BKTP2_VM_TIMEOUT_DIV_SHIFT			  2
#define   BKTP3_VM_TIMEOUT_DIV_MASK			  GENMASK(5, 4)
#define   BKTP3_VM_TIMEOUT_DIV_SHIFT			  4
#define   BKTP4_VM_TIMEOUT_DIV_MASK			  GENMASK(7, 6)
#define   BKTP4_VM_TIMEOUT_DIV_SHIFT			  6
#define MAX96724_BACKTOP1_EMBED0			  0x0450
#define   EMBED_FL_NUM_BKTP0_MASK			  GENMASK(1, 0)
#define   EMBED_FL_NUM_BKTP0_SHIFT			  0
#define   EMBED_FL_EN_BKTP0				  BIT(3)
#define   EMBED_LL_NUM_BKTP0_MASK			  GENMASK(5, 4)
#define   EMBED_LL_NUM_BKTP0_SHIFT			  4
#define   EMBED_LL_EN_BKTP0				  BIT(7)
#define MAX96724_BACKTOP1_EMBED1			  0x0451
#define   EMBED_FL_NUM_BKTP1_MASK			  GENMASK(1, 0)
#define   EMBED_FL_NUM_BKTP1_SHIFT			  0
#define   EMBED_FL_EN_BKTP1				  BIT(3)
#define   EMBED_LL_NUM_BKTP1_MASK			  GENMASK(5, 4)
#define   EMBED_LL_NUM_BKTP1_SHIFT			  4
#define   EMBED_LL_EN_BKTP1				  BIT(7)
#define MAX96724_BACKTOP1_EMBED2			  0x0452
#define   EMBED_FL_NUM_BKTP2_MASK			  GENMASK(1, 0)
#define   EMBED_FL_NUM_BKTP2_SHIFT			  0
#define   EMBED_FL_EN_BKTP2				  BIT(3)
#define   EMBED_LL_NUM_BKTP2_MASK			  GENMASK(5, 4)
#define   EMBED_LL_NUM_BKTP2_SHIFT			  4
#define   EMBED_LL_EN_BKTP2				  BIT(7)
#define MAX96724_BACKTOP1_EMBED3			  0x0453
#define   EMBED_FL_NUM_BKTP3_MASK			  GENMASK(1, 0)
#define   EMBED_FL_NUM_BKTP3_SHIFT			  0
#define   EMBED_FL_EN_BKTP3				  BIT(3)
#define   EMBED_LL_NUM_BKTP3_MASK			  GENMASK(5, 4)
#define   EMBED_LL_NUM_BKTP3_SHIFT			  4
#define   EMBED_LL_EN_BKTP3				  BIT(7)
#define MAX96724_BACKTOP_1_CMD_LMO_ERRB_EN		  0x0454
#define   LMO_0_ERRB_OEN				  BIT(0)
#define   LMO_1_ERRB_OEN				  BIT(1)
#define   LMO_2_ERRB_OEN				  BIT(2)
#define   LMO_3_ERRB_OEN				  BIT(3)
#define   CMD_OVFL_0_ERRB_OEN				  BIT(4)
#define   CMD_OVFL_1_ERRB_OEN				  BIT(5)
#define   CMD_OVFL_2_ERRB_OEN				  BIT(6)
#define   CMD_OVFL_3_ERRB_OEN				  BIT(7)
#define MAX96724_BACKTOP_1_DPLL_ERRB_OEN		  0x0455
#define   CSI_DPLL0_ERRB_OEN				  BIT(0)
#define   CSI_DPLL1_ERRB_OEN				  BIT(1)
#define   CSI_DPLL2_ERRB_OEN				  BIT(2)
#define   CSI_DPLL3_ERRB_OEN				  BIT(3)
#define   CSIPLL0_LOL_STICKY_FLAG			  BIT(4)
#define   CSIPLL1_LOL_STICKY_FLAG			  BIT(5)
#define   CSIPLL2_LOL_STICKY_FLAG			  BIT(6)
#define   CSIPLL3_LOL_STICKY_FLAG			  BIT(7)
#define MAX96724_BACKTOP1_OVERRIDE_BPP_DT		  0x0456
#define   OVERRIDE_BPP_DT_0				  BIT(0)
#define   OVERRIDE_BPP_DT_1				  BIT(1)
#define   OVERRIDE_BPP_DT_2				  BIT(2)
#define   OVERRIDE_BPP_DT_3				  BIT(3)
#define   OVERRIDE_VC_0					  BIT(4)
#define   OVERRIDE_VC_1					  BIT(5)
#define   OVERRIDE_VC_2					  BIT(6)
#define   OVERRIDE_VC_3					  BIT(7)
#define MAX96724_BACKTOP1_OVERRIDE_VC			  0x0457
#define   OVERRIDE_VC_BITS_2_AND_3			  BIT(0)
#define MAX96724_BACKTOP1_SRAM_LCRC_ERR			  0x0458
#define   SRAM_LCRC_ERR_0				  BIT(0)
#define   SRAM_LCRC_ERR_1				  BIT(1)
#define   SRAM_LCRC_ERR_2				  BIT(2)
#define   SRAM_LCRC_ERR_3				  BIT(3)
#define   SRAM_LCRC_ERR_OEN_0				  BIT(4)
#define   SRAM_LCRC_ERR_OEN_1				  BIT(5)
#define   SRAM_LCRC_ERR_OEN_2				  BIT(6)
#define   SRAM_LCRC_ERR_OEN_3				  BIT(7)
#define MAX96724_BACKTOP1_SRAM_LCRC_EN			  0x0459
#define   SRAM_LCRC_PIXEL_CHK_DIS_0			  BIT(0)
#define   SRAM_LCRC_PIXEL_CHK_DIS_1			  BIT(1)
#define   SRAM_LCRC_PIXEL_CHK_DIS_2			  BIT(2)
#define   SRAM_LCRC_PIXEL_CHK_DIS_3			  BIT(3)
#define   SRAM_LCRC_TUN_CHK_DIS_0			  BIT(4)
#define   SRAM_LCRC_TUN_CHK_DIS_1			  BIT(5)
#define   SRAM_LCRC_TUN_CHK_DIS_2			  BIT(6)
#define   SRAM_LCRC_TUN_CHK_DIS_3			  BIT(7)
#define MAX96724_BACKTOP1_SRAM_LCRC_RESET		  0x045a
#define   SRAM_LCRC_MATCH_RESET_0			  BIT(0)
#define   SRAM_LCRC_MATCH_RESET_1			  BIT(1)
#define   SRAM_LCRC_MATCH_RESET_2			  BIT(2)
#define   SRAM_LCRC_MATCH_RESET_3			  BIT(3)
#define   INIT_SRAM_LCRC_ERR_DIS_0			  BIT(4)
#define   INIT_SRAM_LCRC_ERR_DIS_1			  BIT(5)
#define   INIT_SRAM_LCRC_ERR_DIS_2			  BIT(6)
#define   INIT_SRAM_LCRC_ERR_DIS_3			  BIT(7)

/* ERR_INJ */
#define MAX96724_ERR_INJ_BKTOP_1			  0x0480
#define   SRAM_LCRC_ERR_INJ_DIS_0			  BIT(0)
#define   SRAM_LCRC_ERR_INJ_DIS_1			  BIT(1)
#define   SRAM_LCRC_ERR_INJ_DIS_2			  BIT(2)
#define   SRAM_LCRC_ERR_INJ_DIS_3			  BIT(3)
#define MAX96724_ERR_INJ_MEM_1BIT			  0x0481
#define   MEM_ERR_INJ_1BIT_BKTP1			  BIT(0)
#define   MEM_ERR_INJ_1BIT_BKTP2			  BIT(1)
#define   MEM_ERR_INJ_1BIT_BKTP3			  BIT(2)
#define   MEM_ERR_INJ_1BIT_BKTP4			  BIT(3)
#define MAX96724_ERR_INJ_MEM_2BIT			  0x0482
#define   MEM_ERR_INJ_2BIT_BKTP1			  BIT(0)
#define   MEM_ERR_INJ_2BIT_BKTP2			  BIT(1)
#define   MEM_ERR_INJ_2BIT_BKTP3			  BIT(2)
#define   MEM_ERR_INJ_2BIT_BKTP4			  BIT(3)
#define MAX96724_ERR_INJ_MEM_WORD_LOC_EN		  0x0483
#define   MEM_ERR_INJ_WORD_LOC_1_EN			  BIT(0)
#define   MEM_ERR_INJ_WORD_LOC_2_EN			  BIT(1)
#define MAX96724_ERR_INJ_MEM_WORD_LOC_1			  0x0484
#define MAX96724_ERR_INJ_MEM_WORD_LOC_2			  0x0485
#define MAX96724_ERR_INJ_MEM_PKT_NUM			  0x0486
#define   MEM_ERR_INJ_PKT_NUM_MASK			  GENMASK(3, 0)
#define   MEM_ERR_INJ_PKT_NUM_SHIFT			  0
#define MAX96724_ERR_INJ_MEM_BIT1_LOC			  0x0487
#define   MEM_ERR_INJ_BIT1_LOC_MASK			  GENMASK(4, 0)
#define   MEM_ERR_INJ_BIT1_LOC_SHIFT			  0
#define MAX96724_ERR_INJ_MEM_BIT2_LOC			  0x0488
#define   MEM_ERR_INJ_BIT2_LOC_MASK			  GENMASK(4, 0)
#define   MEM_ERR_INJ_BIT2_LOC_SHIFT			  0

/* FSYNC */
#define MAX96724_FSYNC_0				  0x04a0
#define   FSYNC_METH_MASK				  GENMASK(1, 0)
#define   FSYNC_METH_SHIFT				  0
#define   FSYNC_MODE_MASK				  GENMASK(3, 2)
#define   FSYNC_MODE_SHIFT				  2
#define   EN_VS_GEN					  BIT(4)
#define   FSYNC_OUT_PIN					  BIT(5)
#define MAX96724_FSYNC_1				  0x04a1
#define   FSYNC_PER_DIV_MASK				  GENMASK(3, 0)
#define   FSYNC_PER_DIV_SHIFT				  0
#define MAX96724_FSYNC_2				  0x04a2
#define   K_VAL_MASK					  GENMASK(3, 0)
#define   K_VAL_SHIFT					  0
#define   K_VAL_SIGN					  BIT(4)
#define   MST_LINK_SEL_MASK				  GENMASK(7, 5)
#define   MST_LINK_SEL_SHIFT				  5
#define MAX96724_FSYNC_P_VAL_L				  0x04a3
#define MAX96724_FSYNC_4				  0x04a4
#define   P_VAL_H_MASK					  GENMASK(4, 0)
#define   P_VAL_H_SHIFT					  0
#define   P_VAL_SIGN					  BIT(5)
#define MAX96724_FSYNC_PERIOD_L				  0x04a5
#define MAX96724_FSYNC_PERIOD_M				  0x04a6
#define MAX96724_FSYNC_PERIOD_H				  0x04a7
#define MAX96724_FSYNC_FRM_DIFF_ERR_THR_L		  0x04a8
#define MAX96724_FSYNC_9				  0x04a9
#define   FRM_DIFF_ERR_THR_H_MASK			  GENMASK(4, 0)
#define   FRM_DIFF_ERR_THR_H_SHIFT			  0
#define MAX96724_FSYNC_OVLP_WINDOW_L			  0x04aa
#define MAX96724_FSYNC_11				  0x04ab
#define   OVLP_WINDOW_H_MASK				  GENMASK(4, 0)
#define   OVLP_WINDOW_H_SHIFT				  0
#define   EN_FSIN_LAST					  BIT(7)
#define MAX96724_FSYNC_15				  0x04af
#define   FS_LINK_0					  BIT(0)
#define   FS_LINK_1					  BIT(1)
#define   FS_LINK_2					  BIT(2)
#define   FS_LINK_3					  BIT(3)
#define   AUTO_FS_LINKS					  BIT(4)
#define   FS_USE_XTAL					  BIT(6)
#define   FS_GPIO_TYPE					  BIT(7)
#define MAX96724_FSYNC_ERR_CNT				  0x04b0
#define MAX96724_FSYNC_17				  0x04b1
#define   FSYNC_ERR_THR_MASK				  GENMASK(2, 0)
#define   FSYNC_ERR_THR_SHIFT				  0
#define   FSYNC_TX_ID_MASK				  GENMASK(7, 3)
#define   FSYNC_TX_ID_SHIFT				  3
#define MAX96724_FSYNC_CALC_FRM_LEN_L			  0x04b2
#define MAX96724_FSYNC_CALC_FRM_LEN_M			  0x04b3
#define MAX96724_FSYNC_CALC_FRM_LEN_H			  0x04b4
#define MAX96724_FSYNC_FRM_DIFF_L			  0x04b5
#define MAX96724_FSYNC_22				  0x04b6
#define   FRM_DIFF_H_MASK				  GENMASK(5, 0)
#define   FRM_DIFF_H_SHIFT				  0
#define   FSYNC_LOCKED					  BIT(6)
#define   FSYNC_LOSS_OF_LOCK				  BIT(7)
#define MAX96724_FSYNC_23				  0x04b7
#define   FSYNC_RST_MODE				  BIT(5)

/* CFGC_CC0: 0 <= cc < 2, 0 <= link < 4 */
#define MAX96724_CFGC_CC_TR0(cc, link)			  (0x0500 + (cc) * 0x60 + (link) * 0x10)
#define   CC_PRIO_CFG_MASK				  GENMASK(1, 0)
#define   CC_PRIO_CFG_SHIFT				  0
#define   CC_PRIO_VAL_MASK				  GENMASK(3, 2)
#define   CC_PRIO_VAL_SHIFT				  2
#define   CC_RX_CRC_EN					  BIT(6)
#define   CC_TX_CRC_EN					  BIT(7)
#define MAX96724_CFGC_CC_TR1(cc, link)			  (0x0501 + (cc) * 0x60 + (link) * 0x10)
#define   CC_BW_VAL_MASK				  GENMASK(5, 0)
#define   CC_BW_VAL_SHIFT				  0
#define   CC_BW_MULT_MASK				  GENMASK(7, 6)
#define   CC_BW_MULT_SHIFT				  6
#define MAX96724_CFGC_CC_TR3(cc, link)			  (0x0503 + (cc) * 0x60 + (link) * 0x10)
#define   CC_TX_SRC_ID_MASK				  GENMASK(2, 0)
#define   CC_TX_SRC_ID_SHIFT				  0
#define MAX96724_CFGC_CC_RX_SRC_SEL(cc, link)		  (0x0504 + (cc) * 0x60 + (link) * 0x10)
#define MAX96724_CFGC_CC_ARQ1(cc, link)			  (0x0506 + (cc) * 0x60 + (link) * 0x10)
#define   CC_RT_CNT_OEN					  BIT(0)
#define   CC_MAX_RT_ERR_OEN				  BIT(1)
#define   CC_MAX_RT_MASK				  GENMASK(6, 4)
#define   CC_MAX_RT_SHIFT				  4
#define MAX96724_CFGC_CC_ARQ2(cc, link)			  (0x0507 + (cc) * 0x60 + (link) * 0x10)
#define   CC_RT_CNT_MASK				  GENMASK(6, 0)
#define   CC_RT_CNT_SHIFT				  0
#define   CC_MAX_RT_ERR					  BIT(7)

/* CC_G2P0: 0 <= link < 4 */
#define MAX96724_CC_G2P0_I2C_0(link)			  (0x0640 + (link) * 0x10)
#define   SLV_TO_P0_A_MASK				  GENMASK(2, 0)
#define   SLV_TO_P0_A_SHIFT				  0
#define   SLV_SH_P0_A_MASK				  GENMASK(5, 4)
#define   SLV_SH_P0_A_SHIFT				  4
#define MAX96724_CC_G2P0_I2C_1(link)			  (0x0641 + (link) * 0x10)
#define   MST_TO_P0_A_MASK				  GENMASK(2, 0)
#define   MST_TO_P0_A_SHIFT				  0
#define   MST_BT_P0_A_MASK				  GENMASK(6, 4)
#define   MST_BT_P0_A_SHIFT				  4
#define MAX96724_CC_G2P0_I2C_2(link)			  (0x0642 + (link) * 0x10)
#define   SRC_A_P0_A_MASK				  GENMASK(7, 1)
#define   SRC_A_P0_A_SHIFT				  1
#define MAX96724_CC_G2P0_I2C_3(link)			  (0x0643 + (link) * 0x10)
#define   DST_A_P0_A_MASK				  GENMASK(7, 1)
#define   DST_A_P0_A_SHIFT				  1
#define MAX96724_CC_G2P0_I2C_4(link)			  (0x0644 + (link) * 0x10)
#define   SRC_B_P0_A_MASK				  GENMASK(7, 1)
#define   SRC_B_P0_A_SHIFT				  1
#define MAX96724_CC_G2P0_I2C_5(link)			  (0x0645 + (link) * 0x10)
#define   DST_B_P0_A_MASK				  GENMASK(7, 1)
#define   DST_B_P0_A_SHIFT				  1

/* CC_G2P1: 0 <= link < 4 */
#define MAX96724_CC_G2P1_I2C_0(link)			  (0x0680 + (link) * 0x10)
#define   SLV_TO_P1_A_MASK				  GENMASK(2, 0)
#define   SLV_TO_P1_A_SHIFT				  0
#define   SLV_SH_P1_A_MASK				  GENMASK(5, 4)
#define   SLV_SH_P1_A_SHIFT				  4
#define   I2C_HSM_P1					  BIT(6)
#define MAX96724_CC_G2P1_I2C_1(link)			  (0x0681 + (link) * 0x10)
#define   MST_TO_P1_A_MASK				  GENMASK(2, 0)
#define   MST_TO_P1_A_SHIFT				  0
#define   MST_BT_P1_A_MASK				  GENMASK(6, 4)
#define   MST_BT_P1_A_SHIFT				  4
#define MAX96724_CC_G2P1_I2C_2(link)			  (0x0682 + (link) * 0x10)
#define   SRC_A_P1_A_MASK				  GENMASK(7, 1)
#define   SRC_A_P1_A_SHIFT				  1
#define MAX96724_CC_G2P1_I2C_3(link)			  (0x0683 + (link) * 0x10)
#define   DST_A_P1_A_MASK				  GENMASK(7, 1)
#define   DST_A_P1_A_SHIFT				  1
#define MAX96724_CC_G2P1_I2C_4(link)			  (0x0684 + (link) * 0x10)
#define   SRC_B_P1_A_MASK				  GENMASK(7, 1)
#define   SRC_B_P1_A_SHIFT				  1
#define MAX96724_CC_G2P1_I2C_5(link)			  (0x0685 + (link) * 0x10)
#define   DST_B_P1_A_MASK				  GENMASK(7, 1)
#define   DST_B_P1_A_SHIFT				  1

/* PROFILE_CTRL */
#define MAX96724_PROFILE_CTRL_MIPI_SEL			  0x06e1
#define   PROFILE_MIPI_SEL_MASK				  GENMASK(5, 0)
#define   PROFILE_MIPI_SEL_SHIFT			  0
#define MAX96724_PROFILE_CTRL_GMSL_1_0			  0x06ea
#define   PROFILE_GMSL_0_MASK				  GENMASK(2, 0)
#define   PROFILE_GMSL_0_SHIFT				  0
#define   PROFILE_GMSL_1_MASK				  GENMASK(6, 4)
#define   PROFILE_GMSL_1_SHIFT				  4
#define MAX96724_PROFILE_CTRL_GMSL_3_2			  0x06eb
#define   PROFILE_GMSL_2_MASK				  GENMASK(2, 0)
#define   PROFILE_GMSL_2_SHIFT				  0
#define   PROFILE_GMSL_3_MASK				  GENMASK(6, 4)
#define   PROFILE_GMSL_3_SHIFT				  4

/* MIPI_TX_EXT: 0 <= ctrl < 4 */
#define MAX96724_MIPI_TX_EXT_0(ctrl)			  (0x0800 + (ctrl) * 0x10)
#define   MAP_DST_0_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_0_H_SHIFT				  2
#define   MAP_SRC_0_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_0_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_1(ctrl)			  (0x0801 + (ctrl) * 0x10)
#define   MAP_DST_1_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_1_H_SHIFT				  2
#define   MAP_SRC_1_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_1_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_2(ctrl)			  (0x0802 + (ctrl) * 0x10)
#define   MAP_DST_2_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_2_H_SHIFT				  2
#define   MAP_SRC_2_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_2_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_3(ctrl)			  (0x0803 + (ctrl) * 0x10)
#define   MAP_DST_3_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_3_H_SHIFT				  2
#define   MAP_SRC_3_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_3_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_4(ctrl)			  (0x0804 + (ctrl) * 0x10)
#define   MAP_DST_4_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_4_H_SHIFT				  2
#define   MAP_SRC_4_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_4_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_5(ctrl)			  (0x0805 + (ctrl) * 0x10)
#define   MAP_DST_5_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_5_H_SHIFT				  2
#define   MAP_SRC_5_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_5_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_6(ctrl)			  (0x0806 + (ctrl) * 0x10)
#define   MAP_DST_6_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_6_H_SHIFT				  2
#define   MAP_SRC_6_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_6_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_7(ctrl)			  (0x0807 + (ctrl) * 0x10)
#define   MAP_DST_7_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_7_H_SHIFT				  2
#define   MAP_SRC_7_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_7_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_8(ctrl)			  (0x0808 + (ctrl) * 0x10)
#define   MAP_DST_8_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_8_H_SHIFT				  2
#define   MAP_SRC_8_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_8_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_9(ctrl)			  (0x0809 + (ctrl) * 0x10)
#define   MAP_DST_9_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_9_H_SHIFT				  2
#define   MAP_SRC_9_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_9_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_10(ctrl)			  (0x080a + (ctrl) * 0x10)
#define   MAP_DST_10_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_10_H_SHIFT				  2
#define   MAP_SRC_10_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_10_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_11(ctrl)			  (0x080b + (ctrl) * 0x10)
#define   MAP_DST_11_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_11_H_SHIFT				  2
#define   MAP_SRC_11_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_11_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_12(ctrl)			  (0x080c + (ctrl) * 0x10)
#define   MAP_DST_12_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_12_H_SHIFT				  2
#define   MAP_SRC_12_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_12_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_13(ctrl)			  (0x080d + (ctrl) * 0x10)
#define   MAP_DST_13_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_13_H_SHIFT				  2
#define   MAP_SRC_13_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_13_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_14(ctrl)			  (0x080e + (ctrl) * 0x10)
#define   MAP_DST_14_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_14_H_SHIFT				  2
#define   MAP_SRC_14_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_14_H_SHIFT				  5
#define MAX96724_MIPI_TX_EXT_15(ctrl)			  (0x080f + (ctrl) * 0x10)
#define   MAP_DST_15_H_MASK				  GENMASK(4, 2)
#define   MAP_DST_15_H_SHIFT				  2
#define   MAP_SRC_15_H_MASK				  GENMASK(7, 5)
#define   MAP_SRC_15_H_SHIFT				  5

/* MIPI_PHY */
#define MAX96724_MIPI_PHY_0				  0x08a0
#define   PHY_4X2					  BIT(0)
#define   PHY_2X4					  BIT(2)
#define   PHY_1X4A_22					  BIT(3)
#define   PHY_1X4B_22					  BIT(4)
#define   FORCE_CLK0_EN					  BIT(5)
#define   FORCE_CLK3_EN					  BIT(6)
#define   FORCE_CSI_OUT_EN				  BIT(7)
#define MAX96724_MIPI_PHY_1				  0x08a1
#define   T_CLK_PRZERO_MASK				  GENMASK(1, 0)
#define   T_CLK_PRZERO_SHIFT				  0
#define   T_CLK_TRAIL_MASK				  GENMASK(3, 2)
#define   T_CLK_TRAIL_SHIFT				  2
#define   T_HS_PREP_MASK				  GENMASK(5, 4)
#define   T_HS_PREP_SHIFT				  4
#define   T_HS_PRZERO_MASK				  GENMASK(7, 6)
#define   T_HS_PRZERO_SHIFT				  6
#define MAX96724_MIPI_PHY_2				  0x08a2
#define   T_HS_TRAIL_MASK				  GENMASK(1, 0)
#define   T_HS_TRAIL_SHIFT				  0
#define   T_LPX_MASK					  GENMASK(3, 2)
#define   T_LPX_SHIFT					  2
#define   PHY_STDBY_N_MASK				  GENMASK(7, 4)
#define   PHY_STDBY_N_SHIFT				  4
#define MAX96724_MIPI_PHY_3				  0x08a3
#define   PHY0_LANE_MAP_MASK				  GENMASK(3, 0)
#define   PHY0_LANE_MAP_SHIFT				  0
#define   PHY1_LANE_MAP_MASK				  GENMASK(7, 4)
#define   PHY1_LANE_MAP_SHIFT				  4
#define MAX96724_MIPI_PHY_4				  0x08a4
#define   PHY2_LANE_MAP_MASK				  GENMASK(3, 0)
#define   PHY2_LANE_MAP_SHIFT				  0
#define   PHY3_LANE_MAP_MASK				  GENMASK(7, 4)
#define   PHY3_LANE_MAP_SHIFT				  4
#define MAX96724_MIPI_PHY_5				  0x08a5
#define   PHY0_POL_MAP_MASK				  GENMASK(2, 0)
#define   PHY0_POL_MAP_SHIFT				  0
#define   PHY1_POL_MAP_MASK				  GENMASK(5, 3)
#define   PHY1_POL_MAP_SHIFT				  3
#define   T_CLK_PREP_MASK				  GENMASK(7, 6)
#define   T_CLK_PREP_SHIFT				  6
#define MAX96724_MIPI_PHY_6				  0x08a6
#define   PHY2_POL_MAP_MASK				  GENMASK(2, 0)
#define   PHY2_POL_MAP_SHIFT				  0
#define   PHY3_POL_MAP_MASK				  GENMASK(5, 3)
#define   PHY3_POL_MAP_SHIFT				  3
#define MAX96724_MIPI_PHY_8				  0x08a8
#define   T_LPXESC_MASK					  GENMASK(7, 5)
#define   T_LPXESC_SHIFT				  5
#define MAX96724_MIPI_PHY_9				  0x08a9
#define   PHY_CP0_MASK					  GENMASK(7, 3)
#define   PHY_CP0_SHIFT					  3
#define MAX96724_MIPI_PHY_10				  0x08aa
#define   PHY_CP1_MASK					  GENMASK(7, 3)
#define   PHY_CP1_SHIFT					  3
#define MAX96724_MIPI_PHY_11				  0x08ab
#define   PHY_CP_ERR_MASK				  GENMASK(7, 4)
#define   PHY_CP_ERR_SHIFT				  4
#define MAX96724_MIPI_PHY_13				  0x08ad
#define   T_T3_PREBEGIN_MASK				  GENMASK(5, 0)
#define   T_T3_PREBEGIN_SHIFT				  0
#define MAX96724_MIPI_PHY_14				  0x08ae
#define   T_T3_PREP_MASK				  GENMASK(1, 0)
#define   T_T3_PREP_SHIFT				  0
#define   T_T3_POST_MASK				  GENMASK(6, 2)
#define   T_T3_POST_SHIFT				  2
#define MAX96724_MIPI_PHY_16				  0x08b0
#define   TUN_ECC_CORR_ERR_OEN				  BIT(3)
#define   TUN_ECC_UNCORR_ERR_OEN			  BIT(4)
#define   TUN_DATA_CRC_ERR_OEN				  BIT(5)
#define   TUN_CONV_DATA_CRC_ERR_OEN			  BIT(6)
#define MAX96724_MIPI_PHY_17				  0x08b1
#define   TUN_ECC_CORR_ERR				  BIT(3)
#define   TUN_ECC_UNCORR_ERR				  BIT(4)
#define   TUN_DATA_CRC_ERR				  BIT(5)
#define   TUN_CONV_DATA_CRC_ERR				  BIT(6)
#define MAX96724_MIPI_PHY_18				  0x08b2
#define   CSIPLL0_PLLORANGEL				  BIT(0)
#define   CSIPLL0_PLLORANGEH				  BIT(1)
#define   CSIPLL1_PLLORANGEL				  BIT(2)
#define   CSIPLL1_PLLORANGEH				  BIT(3)
#define   CSIPLL2_PLLORANGEL				  BIT(4)
#define   CSIPLL2_PLLORANGEH				  BIT(5)
#define   CSIPLL3_PLLORANGEL				  BIT(6)
#define   CSIPLL3_PLLORANGEH				  BIT(7)
#define MAX96724_MIPI_PHY_19				  0x08b3
#define   CSIPLL0_PLLORANGEL_FLAG			  BIT(0)
#define   CSIPLL0_PLLORANGEH_FLAG			  BIT(1)
#define   CSIPLL1_PLLORANGEL_FLAG			  BIT(2)
#define   CSIPLL1_PLLORANGEH_FLAG			  BIT(3)
#define   CSIPLL2_PLLORANGEL_FLAG			  BIT(4)
#define   CSIPLL2_PLLORANGEH_FLAG			  BIT(5)
#define   CSIPLL3_PLLORANGEL_FLAG			  BIT(6)
#define   CSIPLL3_PLLORANGEH_FLAG			  BIT(7)
#define MAX96724_MIPI_PHY_20				  0x08b4
#define   CSIPLL0_PLLORANGEL_OEN			  BIT(0)
#define   CSIPLL0_PLLORANGEH_OEN			  BIT(1)
#define   CSIPLL1_PLLORANGEL_OEN			  BIT(2)
#define   CSIPLL1_PLLORANGEH_OEN			  BIT(3)
#define   CSIPLL2_PLLORANGEL_OEN			  BIT(4)
#define   CSIPLL2_PLLORANGEH_OEN			  BIT(5)
#define   CSIPLL3_PLLORANGEL_OEN			  BIT(6)
#define   CSIPLL3_PLLORANGEH_OEN			  BIT(7)
#define MAX96724_MIPI_PHY_MIPI_PRBS_0			  0x08c0
#define   MIPI_PRBS_EN_P0_LN0_MASK			  GENMASK(1, 0)
#define   MIPI_PRBS_EN_P0_LN0_SHIFT			  0
#define   MIPI_PRBS_EN_P0_LN1_MASK			  GENMASK(3, 2)
#define   MIPI_PRBS_EN_P0_LN1_SHIFT			  2
#define   MIPI_PRBS_EN_P1_LN0_MASK			  GENMASK(5, 4)
#define   MIPI_PRBS_EN_P1_LN0_SHIFT			  4
#define   MIPI_PRBS_EN_P1_LN1_MASK			  GENMASK(7, 6)
#define   MIPI_PRBS_EN_P1_LN1_SHIFT			  6
#define MAX96724_MIPI_PHY_MIPI_PRBS_1			  0x08c1
#define   MIPI_PRBS_EN_P2_LN0_MASK			  GENMASK(1, 0)
#define   MIPI_PRBS_EN_P2_LN0_SHIFT			  0
#define   MIPI_PRBS_EN_P2_LN1_MASK			  GENMASK(3, 2)
#define   MIPI_PRBS_EN_P2_LN1_SHIFT			  2
#define   MIPI_PRBS_EN_P3_LN0_MASK			  GENMASK(5, 4)
#define   MIPI_PRBS_EN_P3_LN0_SHIFT			  4
#define   MIPI_PRBS_EN_P3_LN1_MASK			  GENMASK(7, 6)
#define   MIPI_PRBS_EN_P3_LN1_SHIFT			  6
#define MAX96724_MIPI_PHY_MIPI_PRBS_2			  0x08c2
#define   MIPI_CUST_SEED_EN_P0_LN0			  BIT(0)
#define   MIPI_CUST_SEED_EN_P0_LN1			  BIT(1)
#define   MIPI_CUST_SEED_EN_P1_LN0			  BIT(2)
#define   MIPI_CUST_SEED_EN_P1_LN1			  BIT(3)
#define   MIPI_CUST_SEED_EN_P2_LN0			  BIT(4)
#define   MIPI_CUST_SEED_EN_P2_LN1			  BIT(5)
#define   MIPI_CUST_SEED_EN_P3_LN0			  BIT(6)
#define   MIPI_CUST_SEED_EN_P3_LN1			  BIT(7)
#define MAX96724_MIPI_PHY_MIPI_PRBS_3			  0x08c3
#define   MIPI_CUSTOM_SEED_2_MASK			  GENMASK(1, 0)
#define   MIPI_CUSTOM_SEED_2_SHIFT			  0
#define MAX96724_MIPI_PHY_MIPI_CUSTOM_SEED_1		  0x08c4
#define MAX96724_MIPI_PHY_MIPI_CUSTOM_SEED_0		  0x08c5
#define MAX96724_MIPI_PHY_21				  0x08c6
#define   AUTO_MASK_EN_MASK				  GENMASK(3, 0)
#define   AUTO_MASK_EN_SHIFT				  0
#define   FORCE_VIDEO_MASK_MASK				  GENMASK(7, 4)
#define   FORCE_VIDEO_MASK_SHIFT			  4
#define MAX96724_MIPI_PHY_22				  0x08c7
#define   VIDEO_MASK_RESTART_EN_MASK			  GENMASK(3, 0)
#define   VIDEO_MASK_RESTART_EN_SHIFT			  0
#define   VIDEO_MASK_LATCH_RESET			  BIT(7)
#define MAX96724_MIPI_PHY_24				  0x08c9
#define   RST_MIPITX_LOC_MASK				  GENMASK(3, 0)
#define   RST_MIPITX_LOC_SHIFT				  0
#define MAX96724_MIPI_PHY_MIPI_CTRL_SEL			  0x08ca
#define   MIPI_CTRL_SEL_0_MASK				  GENMASK(1, 0)
#define   MIPI_CTRL_SEL_0_SHIFT				  0
#define   MIPI_CTRL_SEL_1_MASK				  GENMASK(3, 2)
#define   MIPI_CTRL_SEL_1_SHIFT				  2
#define   MIPI_CTRL_SEL_2_MASK				  GENMASK(5, 4)
#define   MIPI_CTRL_SEL_2_SHIFT				  4
#define   MIPI_CTRL_SEL_3_MASK				  GENMASK(7, 6)
#define   MIPI_CTRL_SEL_3_SHIFT				  6
#define MAX96724_MIPI_PHY_25				  0x08d0
#define   CSI2_TX0_PKT_CNT_MASK				  GENMASK(3, 0)
#define   CSI2_TX0_PKT_CNT_SHIFT			  0
#define   CSI2_TX1_PKT_CNT_MASK				  GENMASK(7, 4)
#define   CSI2_TX1_PKT_CNT_SHIFT			  4
#define MAX96724_MIPI_PHY_26				  0x08d1
#define   CSI2_TX2_PKT_CNT_MASK				  GENMASK(3, 0)
#define   CSI2_TX2_PKT_CNT_SHIFT			  0
#define   CSI2_TX3_PKT_CNT_MASK				  GENMASK(7, 4)
#define   CSI2_TX3_PKT_CNT_SHIFT			  4
#define MAX96724_MIPI_PHY_27				  0x08d2
#define   PHY0_PKT_CNT_MASK				  GENMASK(3, 0)
#define   PHY0_PKT_CNT_SHIFT				  0
#define   PHY1_PKT_CNT_MASK				  GENMASK(7, 4)
#define   PHY1_PKT_CNT_SHIFT				  4
#define MAX96724_MIPI_PHY_28				  0x08d3
#define   PHY2_PKT_CNT_MASK				  GENMASK(3, 0)
#define   PHY2_PKT_CNT_SHIFT				  0
#define   PHY3_PKT_CNT_MASK				  GENMASK(7, 4)
#define   PHY3_PKT_CNT_SHIFT				  4
#define MAX96724_MIPI_PHY_CP_ERR_OE			  0x08d4
#define   PHY_CP0_OV_ERR_OEN				  BIT(4)
#define   PHY_CP0_UF_ERR_OEN				  BIT(5)
#define   PHY_CP1_OV_ERR_OEN				  BIT(6)
#define   PHY_CP1_UF_ERR_OEN				  BIT(7)
#define MAX96724_MIPI_PHY_FLAGS				  0x08d5
#define   DESKEW_START_OVERLAP_FLAG_0			  BIT(0)
#define   DESKEW_START_OVERLAP_FLAG_1			  BIT(1)
#define   DESKEW_START_OVERLAP_FLAG_2			  BIT(2)
#define   DESKEW_START_OVERLAP_FLAG_3			  BIT(3)
#define MAX96724_MIPI_PHY_OEN				  0x08d6
#define   DESKEW_START_OVERLAP_OEN_0			  BIT(0)
#define   DESKEW_START_OVERLAP_OEN_1			  BIT(1)
#define   DESKEW_START_OVERLAP_OEN_2			  BIT(2)
#define   DESKEW_START_OVERLAP_OEN_3			  BIT(3)
#define MAX96724_MIPI_PHY_ERR_PKT_0			  0x08d8
#define   ERR_PKT_DT_0_MASK				  GENMASK(5, 0)
#define   ERR_PKT_DT_0_SHIFT				  0
#define   ERR_PKT_EN_0					  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_1			  0x08d9
#define   ERR_PKT_DT_1_MASK				  GENMASK(5, 0)
#define   ERR_PKT_DT_1_SHIFT				  0
#define   ERR_PKT_EN_1					  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_2			  0x08da
#define   ERR_PKT_DT_2_MASK				  GENMASK(5, 0)
#define   ERR_PKT_DT_2_SHIFT				  0
#define   ERR_PKT_EN_2					  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_3			  0x08db
#define   ERR_PKT_DT_3_MASK				  GENMASK(5, 0)
#define   ERR_PKT_DT_3_SHIFT				  0
#define   ERR_PKT_EN_3					  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_4			  0x08dc
#define   ERR_PKT_VC_OVRD_0_MASK			  GENMASK(4, 0)
#define   ERR_PKT_VC_OVRD_0_SHIFT			  0
#define   ERR_PKT_VC_OVRD_EN_0				  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_5			  0x08dd
#define   ERR_PKT_VC_OVRD_1_MASK			  GENMASK(4, 0)
#define   ERR_PKT_VC_OVRD_1_SHIFT			  0
#define   ERR_PKT_VC_OVRD_EN_1				  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_6			  0x08de
#define   ERR_PKT_VC_OVRD_2_MASK			  GENMASK(4, 0)
#define   ERR_PKT_VC_OVRD_2_SHIFT			  0
#define   ERR_PKT_VC_OVRD_EN_2				  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_7			  0x08e0
#define   ERR_PKT_VC_OVRD_3_MASK			  GENMASK(4, 0)
#define   ERR_PKT_VC_OVRD_3_SHIFT			  0
#define   ERR_PKT_VC_OVRD_EN_3				  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_8			  0x08e1
#define   ERR_PKT_VC_0_MASK				  GENMASK(4, 0)
#define   ERR_PKT_VC_0_SHIFT				  0
#define MAX96724_MIPI_PHY_ERR_PKT_9			  0x08e2
#define   ERR_PKT_VC_1_MASK				  GENMASK(4, 0)
#define   ERR_PKT_VC_1_SHIFT				  0
#define MAX96724_MIPI_PHY_ERR_PKT_10			  0x08e3
#define   ERR_PKT_VC_2_MASK				  GENMASK(4, 0)
#define   ERR_PKT_VC_2_SHIFT				  0
#define MAX96724_MIPI_PHY_ERR_PKT_11			  0x08e4
#define   ERR_PKT_VC_3_MASK				  GENMASK(4, 0)
#define   ERR_PKT_VC_3_SHIFT				  0
#define MAX96724_MIPI_PHY_ERR_PKT_12			  0x08e5
#define   ERR_PKT_WC_OVRD_EN_0				  BIT(4)
#define   ERR_PKT_WC_OVRD_EN_1				  BIT(5)
#define   ERR_PKT_WC_OVRD_EN_2				  BIT(6)
#define   ERR_PKT_WC_OVRD_EN_3				  BIT(7)
#define MAX96724_MIPI_PHY_ERR_PKT_WC_0_H		  0x08e6
#define MAX96724_MIPI_PHY_ERR_PKT_WC_0_L		  0x08e7
#define MAX96724_MIPI_PHY_ERR_PKT_WC_1_H		  0x08e8
#define MAX96724_MIPI_PHY_ERR_PKT_WC_1_L		  0x08e9
#define MAX96724_MIPI_PHY_ERR_PKT_WC_2_H		  0x08ea
#define MAX96724_MIPI_PHY_ERR_PKT_WC_2_L		  0x08eb
#define MAX96724_MIPI_PHY_ERR_PKT_WC_3_H		  0x08ec
#define MAX96724_MIPI_PHY_ERR_PKT_WC_3_L		  0x08ed

/* MIPI_TX: 0 <= pipe < 4 */
#define MAX96724_MIPI_TX_MODE(pipe)			  (0x0901 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_STATUS(pipe)			  (0x0902 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_DESKEW_INIT(pipe)		  (0x0903 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_DESKEW_PER(pipe)		  (0x0904 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_CSI2_T_PRE(pipe)		  (0x0905 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_CSI2_T_POST(pipe)		  (0x0906 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_CSI2_TX_GAP(pipe)		  (0x0907 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_CSI2_TWAKEUP_L(pipe)		  (0x0908 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_CSI2_TWAKEUP_M(pipe)		  (0x0909 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_10(pipe)			  (0x090a + (pipe) * 0x40)
#define   CSI2_TWAKEUP_H_MASK				  GENMASK(2, 0)
#define   CSI2_TWAKEUP_H_SHIFT				  0
#define   CSI2_VCX_EN					  BIT(4)
#define   CSI2_CPHY_EN					  BIT(5)
#define   CSI2_LANE_CNT_MASK				  GENMASK(7, 6)
#define   CSI2_LANE_CNT_SHIFT				  6
#define MAX96724_MIPI_TX_MAP_EN_L(pipe)			  (0x090b + (pipe) * 0x40)
#define MAX96724_MIPI_TX_MAP_EN_H(pipe)			  (0x090c + (pipe) * 0x40)

/* 0 <= map_no < 16 */
#define MAX96724_MIPI_TX_MAP_SRC(pipe, map_no)		  (0x090d + (pipe) * 0x40 + (map_no) * 0x02)
#define MAX96724_MIPI_TX_MAP_DST(pipe, map_no)		  (0x090e + (pipe) * 0x40 + (map_no) * 0x02)

#define MAX96724_MIPI_TX_45(pipe)			  (0x092d + (pipe) * 0x40)
#define   MAP_DPHY_DEST_0_MASK				  GENMASK(1, 0)
#define   MAP_DPHY_DEST_0_SHIFT				  0
#define   MAP_DPHY_DEST_1_MASK				  GENMASK(3, 2)
#define   MAP_DPHY_DEST_1_SHIFT				  2
#define   MAP_DPHY_DEST_2_MASK				  GENMASK(5, 4)
#define   MAP_DPHY_DEST_2_SHIFT				  4
#define   MAP_DPHY_DEST_3_MASK				  GENMASK(7, 6)
#define   MAP_DPHY_DEST_3_SHIFT				  6
#define MAX96724_MIPI_TX_46(pipe)			  (0x092e + (pipe) * 0x40)
#define   MAP_DPHY_DEST_4_MASK				  GENMASK(1, 0)
#define   MAP_DPHY_DEST_4_SHIFT				  0
#define   MAP_DPHY_DEST_5_MASK				  GENMASK(3, 2)
#define   MAP_DPHY_DEST_5_SHIFT				  2
#define   MAP_DPHY_DEST_6_MASK				  GENMASK(5, 4)
#define   MAP_DPHY_DEST_6_SHIFT				  4
#define   MAP_DPHY_DEST_7_MASK				  GENMASK(7, 6)
#define   MAP_DPHY_DEST_7_SHIFT				  6
#define MAX96724_MIPI_TX_47(pipe)			  (0x092f + (pipe) * 0x40)
#define   MAP_DPHY_DEST_8_MASK				  GENMASK(1, 0)
#define   MAP_DPHY_DEST_8_SHIFT				  0
#define   MAP_DPHY_DEST_9_MASK				  GENMASK(3, 2)
#define   MAP_DPHY_DEST_9_SHIFT				  2
#define   MAP_DPHY_DEST_10_MASK				  GENMASK(5, 4)
#define   MAP_DPHY_DEST_10_SHIFT			  4
#define   MAP_DPHY_DEST_11_MASK				  GENMASK(7, 6)
#define   MAP_DPHY_DEST_11_SHIFT			  6
#define MAX96724_MIPI_TX_48(pipe)			  (0x0930 + (pipe) * 0x40)
#define   MAP_DPHY_DEST_12_MASK				  GENMASK(1, 0)
#define   MAP_DPHY_DEST_12_SHIFT			  0
#define   MAP_DPHY_DEST_13_MASK				  GENMASK(3, 2)
#define   MAP_DPHY_DEST_13_SHIFT			  2
#define   MAP_DPHY_DEST_14_MASK				  GENMASK(5, 4)
#define   MAP_DPHY_DEST_14_SHIFT			  4
#define   MAP_DPHY_DEST_15_MASK				  GENMASK(7, 6)
#define   MAP_DPHY_DEST_15_SHIFT			  6
#define MAX96724_MIPI_TX_MAP_CON(pipe)			  (0x0931 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_SKEW_PER_SEL(pipe)		  (0x0932 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_51(pipe)			  (0x0933 + (pipe) * 0x40)
#define   ALT_MEM_MAP12					  BIT(0)
#define   ALT_MEM_MAP8					  BIT(1)
#define   ALT_MEM_MAP10					  BIT(2)
#define   MODE_DT					  BIT(3)
#define   ALT2_MEM_MAP8					  BIT(4)
#define MAX96724_MIPI_TX_52(pipe)			  (0x0934 + (pipe) * 0x40)
#define   VIDEO_MASKED_MASK				  GENMASK(3, 0)
#define   VIDEO_MASKED_SHIFT				  0
#define   VIDEO_MASKED_LATCHED_MASK			  GENMASK(7, 4)
#define   VIDEO_MASKED_LATCHED_SHIFT			  4
#define MAX96724_MIPI_TX_54(pipe)			  (0x0936 + (pipe) * 0x40)
#define   TUN_EN					  BIT(0)
#define   DESKEW_TUN_SRC_MASK				  GENMASK(2, 1)
#define   DESKEW_TUN_SRC_SHIFT				  1
#define   TUN_SER_LANE_NUM_MASK				  GENMASK(4, 3)
#define   TUN_SER_LANE_NUM_SHIFT			  3
#define   DESKEW_TUN_MASK				  GENMASK(6, 5)
#define   DESKEW_TUN_SHIFT				  5
#define   TUN_NO_CORR					  BIT(7)
#define MAX96724_MIPI_TX_PKT_START_ADDR(pipe)		  (0x0938 + (pipe) * 0x40)
#define MAX96724_MIPI_TX_57(pipe)			  (0x0939 + (pipe) * 0x40)
#define   TUN_DPHY_TO_CPHY_CONV_OVRD			  BIT(1)
#define   TUN_DPHY_TO_CPHY_CONV				  BIT(2)
#define   TUN_DEST_MASK					  GENMASK(5, 4)
#define   TUN_DEST_SHIFT				  4
#define   DIS_AUTO_TUN_DET				  BIT(6)
#define   DIS_AUTO_SER_LANE_DET				  BIT(7)
#define MAX96724_MIPI_TX_ERR_INJ_B1(pipe)		  (0x093a + (pipe) * 0x40)
#define   DCPHY_CONV_ERR_INJ_B1_SITE_MASK		  GENMASK(4, 0)
#define   DCPHY_CONV_ERR_INJ_B1_SITE_SHIFT		  0
#define   DCPHY_CONV_ERR_INJ_B1_EN			  BIT(7)
#define MAX96724_MIPI_TX_ERR_INJ_B2(pipe)		  (0x093b + (pipe) * 0x40)
#define   DCPHY_CONV_ERR_INJ_B2_SITE_MASK		  GENMASK(4, 0)
#define   DCPHY_CONV_ERR_INJ_B2_SITE_SHIFT		  0
#define   DCPHY_CONV_ERR_INJ_B2_EN			  BIT(7)
#define MAX96724_MIPI_TX_ERRB_DESKEW_ORDER(pipe)	  (0x093c + (pipe) * 0x40)
#define   DESKEW_BEFORE_VS_PKT_MODE			  BIT(1)
#define   DESKEW_AFTER_ERRB_PKT_MODE			  BIT(2)
#define   DESKEW_BEFORE_ERRB_PKT_MODE			  BIT(3)

/* GMSL1: 0 <= link < 4 */
#define MAX96724_GMSL1_4(link)				  (0x0b04 + (link) * 0x100)
#define   FWDCCEN					  BIT(0)
#define   REVCCEN					  BIT(1)
#define   CC_PORT_SEL					  BIT(3)
#define   PRBSEN					  BIT(5)
#define MAX96724_GMSL1_5(link)				  (0x0b05 + (link) * 0x100)
#define   EQTUNE_MASK					  GENMASK(3, 0)
#define   EQTUNE_SHIFT					  0
#define   EN_EQ						  BIT(4)
#define   HVTR_MODE					  BIT(5)
#define   NO_REM_MST					  BIT(6)
#define MAX96724_GMSL1_6(link)				  (0x0b06 + (link) * 0x100)
#define   HV_SRC_MASK					  GENMASK(2, 0)
#define   HV_SRC_SHIFT					  0
#define   GPI_RT_EN					  BIT(3)
#define   GPI_COMP_EN					  BIT(4)
#define   I2C_RT_EN					  BIT(5)
#define   MAX_RT_EN					  BIT(6)
#define   HIGHIMM					  BIT(7)
#define MAX96724_GMSL1_7(link)				  (0x0b07 + (link) * 0x100)
#define   PXL_CRC					  BIT(0)
#define   HVEN						  BIT(2)
#define   HIBW						  BIT(3)
#define   BWS						  BIT(5)
#define   DRS						  BIT(6)
#define   DBL						  BIT(7)
#define MAX96724_GMSL1_8(link)				  (0x0b08 + (link) * 0x100)
#define   CC_CRC_LENGTH_MASK				  GENMASK(1, 0)
#define   CC_CRC_LENGTH_SHIFT				  0
#define   PKTCC_EN					  BIT(2)
#define   EN_FSYNC_TX					  BIT(4)
#define   GPI_EN					  BIT(5)
#define   GPI_SEL_MASK					  GENMASK(7, 6)
#define   GPI_SEL_SHIFT					  6
#define MAX96724_GMSL1_D(link)				  (0x0b0d + (link) * 0x100)
#define   HS_TRACK_FSYNC				  BIT(2)
#define   I2C_LOC_ACK					  BIT(7)
#define MAX96724_GMSL1_DET_THR(link)			  (0x0b0e + (link) * 0x100)
#define MAX96724_GMSL1_F(link)				  (0x0b0f + (link) * 0x100)
#define   PRBS_TYPE					  BIT(0)
#define   DE_EN						  BIT(3)
#define   EN_VS_FILT					  BIT(4)
#define   EN_HS_FILT					  BIT(5)
#define   EN_DE_FILT					  BIT(6)
#define MAX96724_GMSL1_10(link)				  (0x0b10 + (link) * 0x100)
#define   RCEG_EN					  BIT(0)
#define   RCEG_ERR_NUM_MASK				  GENMASK(4, 1)
#define   RCEG_ERR_NUM_SHIFT				  1
#define   RCEG_BOUND					  BIT(5)
#define   RCEG_TYPE_MASK				  GENMASK(7, 6)
#define   RCEG_TYPE_SHIFT				  6
#define MAX96724_GMSL1_11(link)				  (0x0b11 + (link) * 0x100)
#define   RCEG_LO_BST_LEN_MASK				  GENMASK(1, 0)
#define   RCEG_LO_BST_LEN_SHIFT				  0
#define   RCEG_LO_BST_PRB_MASK				  GENMASK(3, 2)
#define   RCEG_LO_BST_PRB_SHIFT				  2
#define   RCEG_ERR_RATE_MASK				  GENMASK(7, 4)
#define   RCEG_ERR_RATE_SHIFT				  4
#define MAX96724_GMSL1_12(link)				  (0x0b12 + (link) * 0x100)
#define   RCEG_ERR_PER_EN				  BIT(0)
#define   MAX_RT_ERR_EN					  BIT(1)
#define   LINE_CRC_EN_GMSL1				  BIT(3)
#define   LINE_CRC_LOC_MASK				  GENMASK(5, 4)
#define   LINE_CRC_LOC_SHIFT				  4
#define   CC_CRC_ERR_EN					  BIT(6)
#define   UNDERBST_DET_EN				  BIT(7)
#define MAX96724_GMSL1_13(link)				  (0x0b13 + (link) * 0x100)
#define   EOM_MIN_THR_G1_MASK				  GENMASK(4, 0)
#define   EOM_MIN_THR_G1_SHIFT				  0
#define   EOM_MAN_TRG_REQ_G1				  BIT(5)
#define   EOM_PER_MODE_G1				  BIT(6)
#define   EOM_EN_G1					  BIT(7)
#define MAX96724_GMSL1_14(link)				  (0x0b14 + (link) * 0x100)
#define   EOM_PER_THR_MASK				  GENMASK(4, 0)
#define   EOM_PER_THR_SHIFT				  0
#define   AEQ_MAN_TRG_REQ				  BIT(5)
#define   AEQ_PER_MODE					  BIT(6)
#define   AEQ_EN					  BIT(7)
#define MAX96724_GMSL1_DET_ERR(link)			  (0x0b15 + (link) * 0x100)
#define MAX96724_GMSL1_PRBS_ERR(link)			  (0x0b16 + (link) * 0x100)
#define MAX96724_GMSL1_17(link)				  (0x0b17 + (link) * 0x100)
#define   MAX_RT_ERR_GPI				  BIT(3)
#define   GPI_IN					  BIT(4)
#define   PRBS_OK					  BIT(5)
#define   MAX_RT_ERR_I2C				  BIT(6)
#define MAX96724_GMSL1_CC_RETR_CNT(link)		  (0x0b18 + (link) * 0x100)
#define MAX96724_GMSL1_CC_CRC_ERRCNT(link)		  (0x0b19 + (link) * 0x100)
#define MAX96724_GMSL1_RCEG_ERR_CNT(link)		  (0x0b1a + (link) * 0x100)
#define MAX96724_GMSL1_GMSL1_1B(link)			  (0x0b1b + (link) * 0x100)
#define   LINE_CRC_ERR					  BIT(2)
#define MAX96724_GMSL1_1C(link)				  (0x0b1c + (link) * 0x100)
#define   EOM_EYE_WIDTH_MASK				  GENMASK(5, 0)
#define   EOM_EYE_WIDTH_SHIFT				  0
#define MAX96724_GMSL1_1D(link)				  (0x0b1d + (link) * 0x100)
#define   AEQ_BST_MASK					  GENMASK(3, 0)
#define   AEQ_BST_SHIFT					  0
#define   UNDERBOOST_DET				  BIT(4)
#define MAX96724_GMSL1_CRC_VALUE_0(link)		  (0x0b20 + (link) * 0x100)
#define MAX96724_GMSL1_CRC_VALUE_1(link)		  (0x0b21 + (link) * 0x100)
#define MAX96724_GMSL1_CRC_VALUE_2(link)		  (0x0b22 + (link) * 0x100)
#define MAX96724_GMSL1_CRC_VALUE_3(link)		  (0x0b23 + (link) * 0x100)
#define MAX96724_GMSL1_96(link)				  (0x0b96 + (link) * 0x100)
#define   DBL_ALIGN_TO					  BIT(0)
#define   CONV_GMSL1_EN					  BIT(1)
#define   CONV_GMSL1_DATATYPE_MASK			  GENMASK(7, 3)
#define   CONV_GMSL1_DATATYPE_SHIFT			  3
#define MAX96724_GMSL1_CB(link)				  (0x0bcb + (link) * 0x100)
#define   LOCKED_G1					  BIT(0)

/* GMSL: 0 <= link < 4 */
#define MAX96724_GMSL_TX1(link)				  (0x1001 + (link) * 0x10)
#define   ERRG_EN					  BIT(4)
#define MAX96724_GMSL_TX2(link)				  (0x1002 + (link) * 0x10)
#define   ERRG_PER					  BIT(0)
#define   ERRG_BURST_MASK				  GENMASK(3, 1)
#define   ERRG_BURST_SHIFT				  1
#define   ERRG_RATE_MASK				  GENMASK(5, 4)
#define   ERRG_RATE_SHIFT				  4
#define   ERRG_CNT_MASK					  GENMASK(7, 6)
#define   ERRG_CNT_SHIFT				  6
#define MAX96724_GMSL_TX3(link)				  (0x1003 + (link) * 0x10)
#define   TIMEOUT_MASK					  GENMASK(2, 0)
#define   TIMEOUT_SHIFT					  0
#define MAX96724_GMSL_RX0(link)				  (0x1004 + (link) * 0x10)
#define   PKT_CNT_SEL_MASK				  GENMASK(3, 0)
#define   PKT_CNT_SEL_SHIFT				  0
#define   PKT_CNT_LBW_MASK				  GENMASK(7, 6)
#define   PKT_CNT_LBW_SHIFT				  6
#define MAX96724_GMSL_GPIOA(link)			  (0x1008 + (link) * 0x10)
#define   GPIO_FWD_CDLY_MASK				  GENMASK(5, 0)
#define   GPIO_FWD_CDLY_SHIFT				  0
#define   GPIO_TX_CASC					  BIT(6)
#define MAX96724_GMSL_GPIOB(link)			  (0x1009 + (link) * 0x10)
#define   GPIO_REV_CDLY_MASK				  GENMASK(5, 0)
#define   GPIO_REV_CDLY_SHIFT				  0
#define   GPIO_TX_WNDW_MASK				  GENMASK(7, 6)
#define   GPIO_TX_WNDW_SHIFT				  6

/* VRX_PATGEN */
#define MAX96724_VRX_PATGEN_PATGEN_0			  0x1050
#define   VTG_MODE_MASK					  GENMASK(1, 0)
#define   VTG_MODE_SHIFT				  0
#define   DE_INV					  BIT(2)
#define   HS_INV					  BIT(3)
#define   VS_INV					  BIT(4)
#define   GEN_DE					  BIT(5)
#define   GEN_HS					  BIT(6)
#define   GEN_VS					  BIT(7)
#define MAX96724_VRX_PATGEN_PATGEN_1			  0x1051
#define   VS_TRIG					  BIT(0)
#define   PATGEN_MODE_MASK				  GENMASK(5, 4)
#define   PATGEN_MODE_SHIFT				  4
#define   GRAD_MODE					  BIT(7)
#define MAX96724_VRX_PATGEN_VS_DLY_2			  0x1052
#define MAX96724_VRX_PATGEN_VS_DLY_1			  0x1053
#define MAX96724_VRX_PATGEN_VS_DLY_0			  0x1054
#define MAX96724_VRX_PATGEN_VS_HIGH_2			  0x1055
#define MAX96724_VRX_PATGEN_VS_HIGH_1			  0x1056
#define MAX96724_VRX_PATGEN_VS_HIGH_0			  0x1057
#define MAX96724_VRX_PATGEN_VS_LOW_2			  0x1058
#define MAX96724_VRX_PATGEN_VS_LOW_1			  0x1059
#define MAX96724_VRX_PATGEN_VS_LOW_0			  0x105a
#define MAX96724_VRX_PATGEN_V2H_2			  0x105b
#define MAX96724_VRX_PATGEN_V2H_1			  0x105c
#define MAX96724_VRX_PATGEN_V2H_0			  0x105d
#define MAX96724_VRX_PATGEN_HS_HIGH_1			  0x105e
#define MAX96724_VRX_PATGEN_HS_HIGH_0			  0x105f
#define MAX96724_VRX_PATGEN_HS_LOW_1			  0x1060
#define MAX96724_VRX_PATGEN_HS_LOW_0			  0x1061
#define MAX96724_VRX_PATGEN_HS_CNT_1			  0x1062
#define MAX96724_VRX_PATGEN_HS_CNT_0			  0x1063
#define MAX96724_VRX_PATGEN_V2D_2			  0x1064
#define MAX96724_VRX_PATGEN_V2D_1			  0x1065
#define MAX96724_VRX_PATGEN_V2D_0			  0x1066
#define MAX96724_VRX_PATGEN_DE_HIGH_1			  0x1067
#define MAX96724_VRX_PATGEN_DE_HIGH_0			  0x1068
#define MAX96724_VRX_PATGEN_DE_LOW_1			  0x1069
#define MAX96724_VRX_PATGEN_DE_LOW_0			  0x106a
#define MAX96724_VRX_PATGEN_DE_CNT_1			  0x106b
#define MAX96724_VRX_PATGEN_DE_CNT_0			  0x106c
#define MAX96724_VRX_PATGEN_GRAD_INCR			  0x106d
#define MAX96724_VRX_PATGEN_CHKR_COLOR_A_L		  0x106e
#define MAX96724_VRX_PATGEN_CHKR_COLOR_A_M		  0x106f
#define MAX96724_VRX_PATGEN_CHKR_COLOR_A_H		  0x1070
#define MAX96724_VRX_PATGEN_CHKR_COLOR_B_L		  0x1071
#define MAX96724_VRX_PATGEN_CHKR_COLOR_B_M		  0x1072
#define MAX96724_VRX_PATGEN_CHKR_COLOR_B_H		  0x1073
#define MAX96724_VRX_PATGEN_CHKR_RPT_A			  0x1074
#define MAX96724_VRX_PATGEN_CHKR_RPT_B			  0x1075
#define MAX96724_VRX_PATGEN_CHKR_ALT			  0x1076

/* TEST_CTRL */
#define MAX96724_TEST_CTRL_DP_ORSTB_CTL			  0x1191
#define   DP_RST_CC					  BIT(0)
#define   DP_RST_FS					  BIT(1)
#define   DP_RST_VP					  BIT(2)
#define   DP_RST_MIPI					  BIT(3)
#define   DP_RST_MIPI2					  BIT(4)
#define   DP_RST_STABLE					  BIT(5)
#define   DP_RST_MIPI3					  BIT(6)
#define   DPLL_AUTO_RST					  BIT(7)

/* VID_PXL_CRC_ERR */
#define MAX96724_VID_PXL_CRC_ERR_AX			  0x11d0
#define MAX96724_VID_PXL_CRC_ERR_AY			  0x11d1
#define MAX96724_VID_PXL_CRC_ERR_AZ			  0x11d2
#define MAX96724_VID_PXL_CRC_ERR_AU			  0x11e0
#define MAX96724_VID_PXL_CRC_ERR_BX			  0x11e1
#define MAX96724_VID_PXL_CRC_ERR_BY			  0x11e2
#define MAX96724_VID_PXL_CRC_ERR_BZ			  0x11e3
#define MAX96724_VID_PXL_CRC_ERR_BU			  0x11e4
#define MAX96724_VID_PXL_CRC_ERR_CX			  0x11e5
#define MAX96724_VID_PXL_CRC_ERR_CY			  0x11e6
#define MAX96724_VID_PXL_CRC_ERR_CZ			  0x11e7
#define MAX96724_VID_PXL_CRC_ERR_CU			  0x11e8
#define MAX96724_VID_PXL_CRC_ERR_DX			  0x11e9
#define MAX96724_VID_PXL_CRC_ERR_DY			  0x11ea
#define MAX96724_VID_PXL_CRC_ERR_DZ			  0x11eb
#define MAX96724_VID_PXL_CRC_ERR_DU			  0x11ec

/* VID_HVD_DET */
#define MAX96724_VID_HVD_DET_DE_DET			  0x11f0
#define   DE_DET_0					  BIT(0)
#define   DE_DET_1					  BIT(1)
#define   DE_DET_2					  BIT(2)
#define   DE_DET_3					  BIT(3)
#define MAX96724_VID_HVD_DET_HS_DET			  0x11f1
#define   HS_DET_0					  BIT(0)
#define   HS_DET_1					  BIT(1)
#define   HS_DET_2					  BIT(2)
#define   HS_DET_3					  BIT(3)
#define MAX96724_VID_HVD_DET_VS_DET			  0x11f2
#define   VS_DET_0					  BIT(0)
#define   VS_DET_1					  BIT(1)
#define   VS_DET_2					  BIT(2)
#define   VS_DET_3					  BIT(3)
#define MAX96724_VID_HVD_DET_HS_POL			  0x11f3
#define   HS_POL_0					  BIT(0)
#define   HS_POL_1					  BIT(1)
#define   HS_POL_2					  BIT(2)
#define   HS_POL_3					  BIT(3)
#define MAX96724_VID_HVD_DET_VS_POL			  0x11f4
#define   VS_POL_0					  BIT(0)
#define   VS_POL_1					  BIT(1)
#define   VS_POL_2					  BIT(2)
#define   VS_POL_3					  BIT(3)
#define MAX96724_VID_HVD_DET_HVD_CNT_CTRL		  0x11f9
#define   HVD_CNT_EN_0					  BIT(0)
#define   HVD_CNT_EN_1					  BIT(1)
#define   HVD_CNT_EN_2					  BIT(2)
#define   HVD_CNT_EN_3					  BIT(3)
#define   HVD_CNT_RST_0					  BIT(4)
#define   HVD_CNT_RST_1					  BIT(5)
#define   HVD_CNT_RST_2					  BIT(6)
#define   HVD_CNT_RST_3					  BIT(7)
#define MAX96724_VID_HVD_DET_HVD_CNT_OS			  0x11fa
#define   HVD_CNT_OS_EN_0				  BIT(0)
#define   HVD_CNT_OS_EN_1				  BIT(1)
#define   HVD_CNT_OS_EN_2				  BIT(2)
#define   HVD_CNT_OS_EN_3				  BIT(3)
#define MAX96724_VID_HVD_DET_VS_CNT_WNDW_0_MSB		  0x1200
#define   VS_CNT_WINDOW_0_MSB_MASK			  GENMASK(1, 0)
#define   VS_CNT_WINDOW_0_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_VS_CNT_WINDOW_0_LSB	  0x1201
#define MAX96724_VID_HVD_DET_VS_CNT_0_CMP		  0x1202
#define   VS_CNT_0_CMP_MASK				  GENMASK(5, 0)
#define   VS_CNT_0_CMP_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_0_CMP_MSB		  0x1203
#define   HS_CNT_0_CMP_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_0_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_HS_CNT_0_CMP_LSB		  0x1204
#define MAX96724_VID_HVD_DET_DE_CNT_0_CMP_MSB		  0x1205
#define   DE_CNT_0_CMP_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_0_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_DE_CNT_0_CMP_LSB		  0x1206
#define MAX96724_VID_HVD_DET_VS_CNT_0			  0x1207
#define   VS_CNT_0_MASK					  GENMASK(5, 0)
#define   VS_CNT_0_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_0_MSB		  0x1208
#define   HS_CNT_0_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_0_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_0_LSB		  0x1209
#define MAX96724_VID_HVD_DET_DE_CNT_0_MSB		  0x120a
#define   DE_CNT_0_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_0_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_DE_CNT_0_LSB		  0x120b
#define MAX96724_VID_HVD_DET_VRX_0_CMP_ERR_OEN		  0x120c
#define   DE_CNT_0_CMP_ERR_OEN				  BIT(5)
#define   HS_CNT_0_CMP_ERR_OEN				  BIT(6)
#define   VS_CNT_0_CMP_ERR_OEN				  BIT(7)
#define MAX96724_VID_HVD_DET_VRX_0_CMP_ERR_FLAG		  0x120d
#define   DE_CNT_0_CMP_ERR_FLAG				  BIT(5)
#define   HS_CNT_0_CMP_ERR_FLAG				  BIT(6)
#define   VS_CNT_0_CMP_ERR_FLAG				  BIT(7)
#define MAX96724_VID_HVD_DET_VS_CNT_WNDW_1_MSB		  0x1210
#define   VS_CNT_WINDOW_1_MSB_MASK			  GENMASK(1, 0)
#define   VS_CNT_WINDOW_1_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_VS_CNT_WINDOW_1_LSB	  0x1211
#define MAX96724_VID_HVD_DET_VS_CNT_1_CMP		  0x1212
#define   VS_CNT_1_CMP_MASK				  GENMASK(5, 0)
#define   VS_CNT_1_CMP_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_1_CMP_MSB		  0x1213
#define   HS_CNT_1_CMP_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_1_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_HS_CNT_1_CMP_LSB		  0x1214
#define MAX96724_VID_HVD_DET_DE_CNT_1_CMP_MSB		  0x1215
#define   DE_CNT_1_CMP_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_1_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_DE_CNT_1_CMP_LSB		  0x1216
#define MAX96724_VID_HVD_DET_VS_CNT_1			  0x1217
#define   VS_CNT_1_MASK					  GENMASK(5, 0)
#define   VS_CNT_1_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_1_MSB		  0x1218
#define   HS_CNT_1_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_1_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_1_LSB		  0x1219
#define MAX96724_VID_HVD_DET_DE_CNT_1_MSB		  0x121a
#define   DE_CNT_1_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_1_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_DE_CNT_1_LSB		  0x121b
#define MAX96724_VID_HVD_DET_VRX_1_CMP_ERR_OEN		  0x121c
#define   DE_CNT_1_CMP_ERR_OEN				  BIT(5)
#define   HS_CNT_1_CMP_ERR_OEN				  BIT(6)
#define   VS_CNT_1_CMP_ERR_OEN				  BIT(7)
#define MAX96724_VID_HVD_DET_VRX_1_CMP_ERR_FLAG		  0x121d
#define   DE_CNT_1_CMP_ERR_FLAG				  BIT(5)
#define   HS_CNT_1_CMP_ERR_FLAG				  BIT(6)
#define   VS_CNT_1_CMP_ERR_FLAG				  BIT(7)
#define MAX96724_VID_HVD_DET_VS_CNT_WNDW_2_MSB		  0x1220
#define   VS_CNT_WINDOW_2_MSB_MASK			  GENMASK(1, 0)
#define   VS_CNT_WINDOW_2_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_VS_CNT_WINDOW_2_LSB	  0x1221
#define MAX96724_VID_HVD_DET_VS_CNT_2_CMP		  0x1222
#define   VS_CNT_2_CMP_MASK				  GENMASK(5, 0)
#define   VS_CNT_2_CMP_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_2_CMP_MSB		  0x1223
#define   HS_CNT_2_CMP_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_2_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_HS_CNT_2_CMP_LSB		  0x1224
#define MAX96724_VID_HVD_DET_DE_CNT_2_CMP_MSB		  0x1225
#define   DE_CNT_2_CMP_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_2_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_DE_CNT_2_CMP_LSB		  0x1226
#define MAX96724_VID_HVD_DET_VS_CNT_2			  0x1227
#define   VS_CNT_2_MASK					  GENMASK(5, 0)
#define   VS_CNT_2_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_2_MSB		  0x1228
#define   HS_CNT_2_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_2_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_2_LSB		  0x1229
#define MAX96724_VID_HVD_DET_DE_CNT_2_MSB		  0x122a
#define   DE_CNT_2_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_2_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_DE_CNT_2_LSB		  0x122b
#define MAX96724_VID_HVD_DET_VRX_2_CMP_ERR_OEN		  0x122c
#define   DE_CNT_2_CMP_ERR_OEN				  BIT(5)
#define   HS_CNT_2_CMP_ERR_OEN				  BIT(6)
#define   VS_CNT_2_CMP_ERR_OEN				  BIT(7)
#define MAX96724_VID_HVD_DET_VRX_2_CMP_ERR_FLAG		  0x122d
#define   DE_CNT_2_CMP_ERR_FLAG				  BIT(5)
#define   HS_CNT_2_CMP_ERR_FLAG				  BIT(6)
#define   VS_CNT_2_CMP_ERR_FLAG				  BIT(7)
#define MAX96724_VID_HVD_DET_VS_CNT_WNDW_3_MSB		  0x1230
#define   VS_CNT_WINDOW_3_MSB_MASK			  GENMASK(1, 0)
#define   VS_CNT_WINDOW_3_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_VS_CNT_WINDOW_3_LSB	  0x1231
#define MAX96724_VID_HVD_DET_VS_CNT_3_CMP		  0x1232
#define   VS_CNT_3_CMP_MASK				  GENMASK(5, 0)
#define   VS_CNT_3_CMP_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_3_CMP_MSB		  0x1233
#define   HS_CNT_3_CMP_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_3_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_HS_CNT_3_CMP_LSB		  0x1234
#define MAX96724_VID_HVD_DET_DE_CNT_3_CMP_MSB		  0x1235
#define   DE_CNT_3_CMP_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_3_CMP_MSB_SHIFT			  0
#define MAX96724_VID_HVD_DET_DE_CNT_3_CMP_LSB		  0x1236
#define MAX96724_VID_HVD_DET_VS_CNT_3			  0x1237
#define   VS_CNT_3_MASK					  GENMASK(5, 0)
#define   VS_CNT_3_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_3_MSB		  0x1238
#define   HS_CNT_3_MSB_MASK				  GENMASK(3, 0)
#define   HS_CNT_3_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_HS_CNT_3_LSB		  0x1239
#define MAX96724_VID_HVD_DET_DE_CNT_3_MSB		  0x123a
#define   DE_CNT_3_MSB_MASK				  GENMASK(3, 0)
#define   DE_CNT_3_MSB_SHIFT				  0
#define MAX96724_VID_HVD_DET_DE_CNT_3_LSB		  0x123b
#define MAX96724_VID_HVD_DET_VRX_3_CMP_ERR_OEN		  0x123c
#define   DE_CNT_3_CMP_ERR_OEN				  BIT(5)
#define   HS_CNT_3_CMP_ERR_OEN				  BIT(6)
#define   VS_CNT_3_CMP_ERR_OEN				  BIT(7)
#define MAX96724_VID_HVD_DET_VRX_3_CMP_ERR_FLAG		  0x123d
#define   DE_CNT_3_CMP_ERR_FLAG				  BIT(5)
#define   HS_CNT_3_CMP_ERR_FLAG				  BIT(6)
#define   VS_CNT_3_CMP_ERR_FLAG				  BIT(7)

/* TUN_DET */
#define MAX96724_TUN_DET_MODE_DET			  0x1260
#define   BACKTOP1_TUN_DET				  BIT(0)
#define   BACKTOP2_TUN_DET				  BIT(1)
#define   BACKTOP3_TUN_DET				  BIT(2)
#define   BACKTOP4_TUN_DET				  BIT(3)
#define   CPHY_MODE_OVRD_EN				  BIT(7)
#define MAX96724_TUN_DET_CPHY_DET			  0x1261
#define   BACKTOP1_CPHY_MODE_DET			  BIT(0)
#define   BACKTOP2_CPHY_MODE_DET			  BIT(1)
#define   BACKTOP3_CPHY_MODE_DET			  BIT(2)
#define   BACKTOP4_CPHY_MODE_DET			  BIT(3)
#define   BACKTOP1_CPHY_MODE_OVRD			  BIT(4)
#define   BACKTOP2_CPHY_MODE_OVRD			  BIT(5)
#define   BACKTOP3_CPHY_MODE_OVRD			  BIT(6)
#define   BACKTOP4_CPHY_MODE_OVRD			  BIT(7)

#endif /* _MAX96724_REGS_H_ */
