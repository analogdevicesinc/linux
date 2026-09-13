// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * MAX2035x register definitions
 */

#ifndef __LINUX_MFD_MAX2035X_REGISTERS_H__
#define __LINUX_MFD_MAX2035X_REGISTERS_H__

/* ---------------------------------------------------------- */
/* ------------------------ MAX20355 ------------------------ */
/* ---------------------------------------------------------- */

/* --- REVISION_ID (0x00) --- */
#define MAX20355_REG_REVISION_ID                    0x00

#define MAX2035_REVISION_ID_MASK                    GENMASK(7, 0)
#define MAX2035_REVISION_ID_SHIFT                   0

/* --- STATUS0 (0x01) --- */
#define MAX20355_REG_STATUS0                        0x01

#define MAX20355_STATUS0_ITF_RDY_STS_BIT            BIT(7)
#define MAX20355_STATUS0_ITF_RDY_STS_SHIFT          7
#define MAX20355_STATUS0_CH1_CON_STS_BIT            BIT(6)
#define MAX20355_STATUS0_CH1_CON_STS_SHIFT          6
#define MAX20355_STATUS0_CH2_CON_STS_BIT            BIT(5)
#define MAX20355_STATUS0_CH2_CON_STS_SHIFT          5
#define MAX20355_STATUS0_CH1_IDL_STS_BIT            BIT(4)
#define MAX20355_STATUS0_CH1_IDL_STS_SHIFT          4
#define MAX20355_STATUS0_CH2_IDL_STS_BIT            BIT(3)
#define MAX20355_STATUS0_CH2_IDL_STS_SHIFT          3
#define MAX20355_STATUS0_PLC2_MOI_DET_BIT           BIT(1)
#define MAX20355_STATUS0_PLC2_MOI_DET_SHIFT         1
#define MAX20355_STATUS0_PLC1_MOI_DET_BIT           BIT(0)
#define MAX20355_STATUS0_PLC1_MOI_DET_SHIFT         0

/* --- STATUS1 (0x02) --- */
#define MAX20355_REG_STATUS1                        0x02

#define MAX20355_STATUS1_MASK                       GENMASK(7, 0)
#define MAX20355_STATUS1_SHIFT                      0

/* --- STATUS2 (0x03) --- */
#define MAX20355_REG_STATUS2                        0x03

#define MAX20355_STATUS2_MASK                       GENMASK(7, 0)
#define MAX20355_STATUS2_SHIFT                      0

/* --- STATUS3 (0x04) --- */
#define MAX20355_REG_STATUS3                        0x04

#define MAX20355_STATUS3_SYS_ERR_INT_BIT            BIT(7)
#define MAX20355_STATUS3_SYS_ERR_INT_SHIFT          7
#define MAX20355_STATUS3_BB_FAULT_BIT               BIT(6)
#define MAX20355_STATUS3_BB_FAULT_SHIFT             6
#define MAX20355_STATUS3_THM_FLT_STS_BIT            BIT(5)
#define MAX20355_STATUS3_THM_FLT_STS_SHIFT          5

/* --- INT0 (0x05) --- */
#define MAX20355_REG_INT0                           0x05

#define MAX20355_INT0_ITF_RDY_STS_BIT               BIT(7)
#define MAX20355_INT0_ITF_RDY_STS_SHIFT             7
#define MAX20355_INT0_CH1_CON_BIT                   BIT(6)
#define MAX20355_INT0_CH1_CON_SHIFT                 6
#define MAX20355_INT0_CH2_CON_BIT                   BIT(5)
#define MAX20355_INT0_CH2_CON_SHIFT                 5
#define MAX20355_INT0_CH1_IDL_BIT                   BIT(4)
#define MAX20355_INT0_CH1_IDL_SHIFT                 4
#define MAX20355_INT0_CH2_IDL_BIT                   BIT(3)
#define MAX20355_INT0_CH2_IDL_SHIFT                 3
#define MAX20355_INT0_MOI_DNE_BIT                   BIT(2)
#define MAX20355_INT0_MOI_DNE_SHIFT                 2
#define MAX20355_INT0_PLC2_MOI_DET_BIT              BIT(1)
#define MAX20355_INT0_PLC2_MOI_DET_SHIFT            1
#define MAX20355_INT0_PLC1_MOI_DET_BIT              BIT(0)
#define MAX20355_INT0_PLC1_MOI_DET_SHIFT            0

/* --- INT1 (0x06) --- */
#define MAX20355_REG_INT1                           0x06

#define MAX20355_INT1_SYS_ERR_BIT                   BIT(7)
#define MAX20355_INT1_SYS_ERR_SHIFT                 7
#define MAX20355_INT1_BB_FAULT_BIT                  BIT(6)
#define MAX20355_INT1_BB_FAULT_SHIFT                6
#define MAX20355_INT1_THM_FLT_BIT                   BIT(5)
#define MAX20355_INT1_THM_FLT_SHIFT                 5
#define MAX20355_INT1_PLC_NEW_DAT_BIT               BIT(4)
#define MAX20355_INT1_PLC_NEW_DAT_SHIFT             4
#define MAX20355_INT1_PLC2_CMD_DNE_BIT              BIT(3)
#define MAX20355_INT1_PLC2_CMD_DNE_SHIFT            3
#define MAX20355_INT1_PLC1_CMD_DNE_BIT              BIT(2)
#define MAX20355_INT1_PLC1_CMD_DNE_SHIFT            2
#define MAX20355_INT1_PLC2_CMD_ERR_BIT              BIT(1)
#define MAX20355_INT1_PLC2_CMD_ERR_SHIFT            1
#define MAX20355_INT1_PLC1_CMD_ERR_BIT              BIT(0)
#define MAX20355_INT1_PLC1_CMD_ERR_SHIFT            0

/* --- INT2 (0x07) --- */
#define MAX20355_REG_INT2                           0x07

#define MAX20355_INT2_MOI_DET_BIT                   BIT(3)
#define MAX20355_INT2_MOI_DET_SHIFT                 3
#define MAX20355_INT2_RES_DET_ABR_BIT               BIT(2)
#define MAX20355_INT2_RES_DET_ABR_SHIFT             2
#define MAX20355_INT2_RES_DET_OPN_BIT               BIT(1)
#define MAX20355_INT2_RES_DET_OPN_SHIFT             1
#define MAX20355_INT2_RES_DET_GND_BIT               BIT(0)
#define MAX20355_INT2_RES_DET_GND_SHIFT             0

/* --- INT3 (0x08) --- */
#define MAX20355_REG_INT3                           0x08

#define MAX20355_INT3_URT_TMO_FLT2_BIT              BIT(5)
#define MAX20355_INT3_URT_TMO_FLT2_SHIFT            5
#define MAX20355_INT3_URT_MODFAIL2_BIT              BIT(4)
#define MAX20355_INT3_URT_MODFAIL2_SHIFT            4
#define MAX20355_INT3_URT_MODDONE2_BIT              BIT(3)
#define MAX20355_INT3_URT_MODDONE2_SHIFT            3
#define MAX20355_INT3_URT_TMO_FLT1_BIT              BIT(2)
#define MAX20355_INT3_URT_TMO_FLT1_SHIFT            2
#define MAX20355_INT3_URT_MODFAIL1_BIT              BIT(1)
#define MAX20355_INT3_URT_MODFAIL1_SHIFT            1
#define MAX20355_INT3_URT_MODDONE1_BIT              BIT(0)
#define MAX20355_INT3_URT_MODDONE1_SHIFT            0

/* --- INTMASK0 (0x09) --- */
#define MAX20355_REG_INTMASK0                       0x09

#define MAX20355_INTMASK0_ITF_RDY_STS_INTM_BIT      BIT(7)
#define MAX20355_INTMASK0_ITF_RDY_STS_INTM_SHIFT    7
#define MAX20355_INTMASK0_CH1_CON_INTM_BIT          BIT(6)
#define MAX20355_INTMASK0_CH1_CON_INTM_SHIFT        6
#define MAX20355_INTMASK0_CH2_CON_INTM_BIT          BIT(5)
#define MAX20355_INTMASK0_CH2_CON_INTM_SHIFT        5
#define MAX20355_INTMASK0_CH1_IDL_INTM_BIT          BIT(4)
#define MAX20355_INTMASK0_CH1_IDL_INTM_SHIFT        4
#define MAX20355_INTMASK0_CH2_IDL_INTM_BIT          BIT(3)
#define MAX20355_INTMASK0_CH2_IDL_INTM_SHIFT        3
#define MAX20355_INTMASK0_MOI_DNE_INTM_BIT          BIT(2)
#define MAX20355_INTMASK0_MOI_DNE_INTM_SHIFT        2
#define MAX20355_INTMASK0_PLC2_MOI_DET_INTM_BIT     BIT(1)
#define MAX20355_INTMASK0_PLC2_MOI_DET_INTM_SHIFT   1
#define MAX20355_INTMASK0_PLC1_MOI_DET_INTM_BIT     BIT(0)
#define MAX20355_INTMASK0_PLC1_MOI_DET_INTM_SHIFT   0

/* --- INTMASK1 (0x0A) --- */
#define MAX20355_REG_INTMASK1                       0x0A

#define MAX20355_INTMASK1_SYS_ERR_INTM_BIT          BIT(7)
#define MAX20355_INTMASK1_SYS_ERR_INTM_SHIFT        7
#define MAX20355_INTMASK1_BB_FAULTM_BIT             BIT(6)
#define MAX20355_INTMASK1_BB_FAULTM_SHIFT           6
#define MAX20355_INTMASK1_THM_FLT_INTM_BIT          BIT(5)
#define MAX20355_INTMASK1_THM_FLT_INTM_SHIFT        5
#define MAX20355_INTMASK1_PLC_NEW_DATM_BIT          BIT(4)
#define MAX20355_INTMASK1_PLC_NEW_DATM_SHIFT        4
#define MAX20355_INTMASK1_PLC2_CMD_DNEM_BIT         BIT(3)
#define MAX20355_INTMASK1_PLC2_CMD_DNEM_SHIFT       3
#define MAX20355_INTMASK1_PLC1_CMD_DNEM_BIT         BIT(2)
#define MAX20355_INTMASK1_PLC1_CMD_DNEM_SHIFT       2
#define MAX20355_INTMASK1_PLC2_CMD_ERRM_BIT         BIT(1)
#define MAX20355_INTMASK1_PLC2_CMD_ERRM_SHIFT       1
#define MAX20355_INTMASK1_PLC1_CMD_ERRM_BIT         BIT(0)
#define MAX20355_INTMASK1_PLC1_CMD_ERRM_SHIFT       0

/* --- INTMASK2 (0x0B) --- */
#define MAX20355_REG_INTMASK2                       0x0B

#define MAX20355_INTMASK2_MOI_DETM_BIT              BIT(3)
#define MAX20355_INTMASK2_MOI_DETM_SHIFT            3
#define MAX20355_INTMASK2_RES_DET_ABRM_BIT          BIT(2)
#define MAX20355_INTMASK2_RES_DET_ABRM_SHIFT        2
#define MAX20355_INTMASK2_RES_DET_OPNM_BIT          BIT(1)
#define MAX20355_INTMASK2_RES_DET_OPNM_SHIFT        1
#define MAX20355_INTMASK2_RES_DET_GNDM_BIT          BIT(0)
#define MAX20355_INTMASK2_RES_DET_GNDM_SHIFT        0

/* --- INTMASK3 (0x0C) --- */
#define MAX20355_REG_INTMASK3                       0x0C

#define MAX20355_INTMASK3_URT_TMO_FLT2M_BIT         BIT(5)
#define MAX20355_INTMASK3_URT_TMO_FLT2M_SHIFT       5
#define MAX20355_INTMASK3_URT_MODFAIL2M_BIT         BIT(4)
#define MAX20355_INTMASK3_URT_MODFAIL2M_SHIFT       4
#define MAX20355_INTMASK3_URT_MODDONE2M_BIT         BIT(3)
#define MAX20355_INTMASK3_URT_MODDONE2M_SHIFT       3
#define MAX20355_INTMASK3_URT_TMO_FLT1M_BIT         BIT(2)
#define MAX20355_INTMASK3_URT_TMO_FLT1M_SHIFT       2
#define MAX20355_INTMASK3_URT_MODFAIL1M_BIT         BIT(1)
#define MAX20355_INTMASK3_URT_MODFAIL1M_SHIFT       1
#define MAX20355_INTMASK3_URT_MODDONE1M_BIT         BIT(0)
#define MAX20355_INTMASK3_URT_MODDONE1M_SHIFT       0

/* --- SYSTEM_REG0 (0x1A) --- */
#define MAX20355_REG_SYSTEM_REG0                    0x1A

#define MAX20355_SYSTEM_OFF_CMD_INP_BIT             BIT(7)
#define MAX20355_SYSTEM_OFF_CMD_INP_SHIFT           7
#define MAX20355_SYSTEM_SOFT_RESET_BIT              BIT(6)
#define MAX20355_SYSTEM_SOFT_RESET_SHIFT            6
#define MAX20355_SYSTEM_STAY_ON_BIT                 BIT(1)
#define MAX20355_SYSTEM_STAY_ON_SHIFT               1
#define MAX20355_SYSTEM_ENB_OTP_ENA_BIT             BIT(0)
#define MAX20355_SYSTEM_ENB_OTP_ENA_SHIFT           0

/* --- BOT_CMD (0x1B) --- */
#define MAX20355_REG_BOT_CMD                        0x1B

#define MAX20355_BOTCMD_FG_RESET_BIT                BIT(7)
#define MAX20355_BOTCMD_FG_RESET_SHIFT              7
#define MAX20355_BOTCMD_FG_ENA_BYP_BIT              BIT(6)
#define MAX20355_BOTCMD_FG_ENA_BYP_SHIFT            6
#define MAX20355_BOTCMD_FG_ENA_VAL_BIT              BIT(5)
#define MAX20355_BOTCMD_FG_ENA_VAL_SHIFT            5

/* --- UART_CTR0 (0x20) --- */
#define MAX20355_REG_UART_CTR0                      0x20

#define MAX20355_UART_CTR_URT_AUTO_EN2_BIT          BIT(3)
#define MAX20355_UART_CTR_URT_AUTO_EN2_SHIFT        3
#define MAX20355_UART_CTR_URT_AUTO_EN1_BIT          BIT(2)
#define MAX20355_UART_CTR_URT_AUTO_EN1_SHIFT        2
#define MAX20355_UART_CTR_TX_TMO_TIM_MASK           GENMASK(1, 0)
#define MAX20355_UART_CTR_TX_TMO_TIM_SHIFT          0

/* --- UART_CTRx (0x21/0x22) --- */
#define MAX20355_REG_UART_CTR1                      0x21
#define MAX20355_REG_UART_CTR2                      0x22

#define MAX20355_UART_CTR_PLC_URT_DIS_BIT           BIT(7)
#define MAX20355_UART_CTR_PLC_URT_DIS_SHIFT         7
#define MAX20355_UART_CTR_TMO_TMR_ENA_BIT           BIT(6)
#define MAX20355_UART_CTR_TMO_TMR_ENA_SHIFT         6
#define MAX20355_UART_CTR_I2C_URT_MOD_BIT           BIT(5)
#define MAX20355_UART_CTR_I2C_URT_MOD_SHIFT         5
#define MAX20355_UART_CTR_I2C_URT_ENA_BIT           BIT(4)
#define MAX20355_UART_CTR_I2C_URT_ENA_SHIFT         4
#define MAX20355_UART_CTR_I2C_URT_ABR_BIT           BIT(3)
#define MAX20355_UART_CTR_I2C_URT_ABR_SHIFT         3
#define MAX20355_UART_CTR_I2C_URT_SWC_BIT           BIT(2)
#define MAX20355_UART_CTR_I2C_URT_SWC_SHIFT         2
#define MAX20355_UART_CTR_I2C_TX_SWC_BIT            BIT(1)
#define MAX20355_UART_CTR_I2C_TX_SWC_SHIFT          1
#define MAX20355_UART_CTR_I2C_RX_SWC_BIT            BIT(0)
#define MAX20355_UART_CTR_I2C_RX_SWC_SHIFT          0

/* --- SYSTEM_CONFIG0 (0x30) --- */
#define MAX20355_REG_SYSTEM_CONFIG0                 0x30

#define MAX20355_SYS_CFG0_LOW_PWR_ENA_BIT           BIT(6)
#define MAX20355_SYS_CFG0_LOW_PWR_ENA_SHIFT         6

/* --- PLC_CONFIG0 (0x31) --- */
#define MAX20355_REG_PLC_CONFIG0                    0x31

#define MAX20355_PLC_CFG0_PLCSNKSEL_MASK            GENMASK(7, 6)
#define MAX20355_PLC_CFG0_PLCSNKSEL_SHIFT           6
#define MAX20355_PLC_CFG0_PLCTHRSEL_MASK            GENMASK(1, 0)
#define MAX20355_PLC_CFG0_PLCTHRSEL_SHIFT           0

/* --- PLC_CONFIG1 (0x32) --- */
#define MAX20355_REG_PLC_CONFIG1                    0x32

#define MAX20355_PLC_CFG1_PLC2_IPROG_MASK           GENMASK(5, 3)
#define MAX20355_PLC_CFG1_PLC2_IPROG_SHIFT          3
#define MAX20355_PLC_CFG1_PLC1_IPROG_MASK           GENMASK(2, 0)
#define MAX20355_PLC_CFG1_PLC1_IPROG_SHIFT          0

/* --- PLC_CONFIG2 (0x33) --- */
#define MAX20355_REG_PLC_CONFIG2                    0x33

#define MAX20355_PLC_CFG2_PL2_CHN_ENA_BIT           BIT(7)
#define MAX20355_PLC_CFG2_PL2_CHN_ENA_SHIFT         7
#define MAX20355_PLC_CFG2_PL1_CHN_ENA_BIT           BIT(6)
#define MAX20355_PLC_CFG2_PL1_CHN_ENA_SHIFT         6
#define MAX20355_PLC_CFG2_PL2_RES_REQ_BIT           BIT(5)
#define MAX20355_PLC_CFG2_PL2_RES_REQ_SHIFT         5
#define MAX20355_PLC_CFG2_PL1_RES_REQ_BIT           BIT(4)
#define MAX20355_PLC_CFG2_PL1_RES_REQ_SHIFT         4
#define MAX20355_PLC_CFG2_PNG_RTY_NUM_MASK          GENMASK(3, 1)
#define MAX20355_PLC_CFG2_PNG_RTY_NUM_SHIFT         1
#define MAX20355_PLC_CFG2_PLC_DOUBLE_BIT            BIT(0)
#define MAX20355_PLC_CFG2_PLC_DOUBLE_SHIFT          0

/* --- PLC_CONFIG3 (0x34) --- */
#define MAX20355_REG_PLC_CONFIG3                    0x34

#define MAX20355_PLC_CFG3_PLC_RRT_TMR_MASK          GENMASK(7, 0)
#define MAX20355_PLC_CFG3_PLC_RRT_TMR_SHIFT         0

/* --- PLC_CONFIG4 (0x35) --- */
#define MAX20355_REG_PLC_CONFIG4                    0x35

#define MAX20355_PLC_CFG4_PLC_CONFIG_MASK           GENMASK(7, 0)
#define MAX20355_PLC_CFG4_PLC_CONFIG_SHIFT          0

/* --- PLC_CONFIG5 (0x36) --- */
#define MAX20355_REG_PLC_CONFIG5                    0x36

#define MAX20355_PLC_CFG5_PLC_IS_FULL_BIT           BIT(7)
#define MAX20355_PLC_CFG5_PLC_IS_FULL_SHIFT         7
#define MAX20355_PLC_CFG5_RAM_IS_FULL_BIT           BIT(6)
#define MAX20355_PLC_CFG5_RAM_IS_FULL_SHIFT         6
#define MAX20355_PLC_CFG5_CONT_STREAM_BIT           BIT(0)
#define MAX20355_PLC_CFG5_CONT_STREAM_SHIFT         0

/* --- PLC_CONFIG6 (0x37) --- */
#define MAX20355_REG_PLC_CONFIG6                    0x37

#define MAX20355_PLC_CFG6_SWP_PLC_RAM_BIT           BIT(7)
#define MAX20355_PLC_CFG6_SWP_PLC_RAM_SHIFT         7
#define MAX20355_PLC_CFG6_NO_UART_MDE_BIT           BIT(5)
#define MAX20355_PLC_CFG6_NO_UART_MDE_SHIFT         5
#define MAX20355_PLC_CFG6_NO_IDLE_MDE_BIT           BIT(4)
#define MAX20355_PLC_CFG6_NO_IDLE_MDE_SHIFT         4
#define MAX20355_PLC_CFG6_DAT_MAX_RTY_MASK          GENMASK(2, 0)
#define MAX20355_PLC_CFG6_DAT_MAX_RTY_SHIFT         0

/* --- PLC_ARGx (0x38/0x3A) --- */
#define MAX20355_REG_PLC_ARG1                       0x38
#define MAX20355_REG_PLC_ARG2                       0x3A

#define MAX20355_PLC_ARG_CMD_MASK                   GENMASK(7, 0)
#define MAX20355_PLC_ARG_CMD_SHIFT                  0

/* --- PLC_CMDx (0x39/0x3B) --- */
#define MAX20355_REG_PLC_CMD1                       0x39
#define MAX20355_REG_PLC_CMD2                       0x3B

#define MAX20355_PLC_CMD_RUN_TRG_BIT                BIT(7)
#define MAX20355_PLC_CMD_RUN_TRG_SHIFT              7
#define MAX20355_PLC_CMD_COMMAND_MASK               GENMASK(6, 0)
#define MAX20355_PLC_CMD_COMMAND_SHIFT              0

/* --- PLC_RX (0x3C) --- */
#define MAX20355_REG_PLC_RX                         0x3C

#define MAX20355_PLC_RX_CH_SEL_BIT                  BIT(7)
#define MAX20355_PLC_RX_CH_SEL_SHIFT                7
#define MAX20355_PLC_RX_BYTES_MASK                  GENMASK(6, 0)
#define MAX20355_PLC_RX_BYTES_SHIFT                 0

/* --- PLC_FIFO (0x3D) --- */
#define MAX20355_REG_PLC_FIFO                       0x3D

#define MAX20355_PLC_FIFO_PL2_MASTER_BIT            BIT(3)
#define MAX20355_PLC_FIFO_PL2_MASTER_SHIFT          3
#define MAX20355_PLC_FIFO_PL2_SLAVE_BIT             BIT(2)
#define MAX20355_PLC_FIFO_PL2_SLAVE_SHIFT           2
#define MAX20355_PLC_FIFO_PL1_MASTER_BIT            BIT(1)
#define MAX20355_PLC_FIFO_PL1_MASTER_SHIFT          1
#define MAX20355_PLC_FIFO_PL1_SLAVE_BIT             BIT(0)
#define MAX20355_PLC_FIFO_PL1_SLAVE_SHIFT           0

/* --- BB_UP_DOWN (0x40) --- */
#define MAX20355_REG_BB_UP_DOWN                     0x40

#define MAX20355_BB_UP_DOWN_UPDOWN_MASK             GENMASK(7, 5)
#define MAX20355_BB_UP_DOWN_UPDOWN_SHIFT            5
#define MAX20355_BB_UP_DOWN_I2C_FRC_BIT             BIT(4)
#define MAX20355_BB_UP_DOWN_I2C_FRC_SHIFT           4
#define MAX20355_BB_UP_DOWN_ALG_MAX_BIT             BIT(1)
#define MAX20355_BB_UP_DOWN_ALG_MAX_SHIFT           1
#define MAX20355_BB_UP_DOWN_ALG_MIN_BIT             BIT(0)
#define MAX20355_BB_UP_DOWN_ALG_MIN_SHIFT           0

/* --- BB_VOLT_DEF (0x41) --- */
#define MAX20355_REG_BB_VOLT_DEF                    0x41

#define MAX20355_BB_VOLT_DEF_MASK                   GENMASK(7, 0)
#define MAX20355_BB_VOLT_DEF_SHIFT                  0

/* --- BB_VLT_TRAN (0x42) --- */
#define MAX20355_REG_BB_VLT_TRAN                    0x42

#define MAX20355_BB_VLT_TRAN_MASK                   GENMASK(7, 0)
#define MAX20355_BB_VLT_TRAN_SHIFT                  0

/* --- BB_RMP_CFG3 (0x46) --- */
#define MAX20355_REG_BB_RMP_CFG3                    0x46

#define MAX20355_BB_RMP_CFG3_MIN_OVL_MASK           GENMASK(2, 0)
#define MAX20355_BB_RMP_CFG3_MIN_OVL_SHIFT          0

/* --- BB_ANA_CFG1 (0x47) --- */
#define MAX20355_REG_BB_ANA_CFG1                    0x47

#define MAX20355_BB_ANA_CFG1_PSV_DCHG_BIT           BIT(7)
#define MAX20355_BB_ANA_CFG1_PSV_DCHG_SHIFT         7
#define MAX20355_BB_ANA_CFG1_ACT_DCHG_BIT           BIT(6)
#define MAX20355_BB_ANA_CFG1_ACT_DCHG_SHIFT         6

/* --- BB_ANA_CFG2 (0x48) --- */
#define MAX20355_REG_BB_ANA_CFG2                    0x48

#define MAX20355_BB_ANA_CFG2_FRC_EN_BIT             BIT(6)
#define MAX20355_BB_ANA_CFG2_FRC_EN_SHIFT           6
#define MAX20355_BB_ANA_CFG2_LOW_BW_BIT             BIT(3)
#define MAX20355_BB_ANA_CFG2_LOW_BW_SHIFT           3
#define MAX20355_BB_ANA_CFG2_L2UH_BIT               BIT(2)
#define MAX20355_BB_ANA_CFG2_L2UH_SHIFT             2
#define MAX20355_BB_ANA_CFG2_ZCCM_EN_BIT            BIT(1)
#define MAX20355_BB_ANA_CFG2_ZCCM_EN_SHIFT          1

/* --- BB_ALG1 (0x4A) --- */
#define MAX20355_REG_BB_ALG1                        0x4A

#define MAX20355_BB_ALG1_AVG_T_DELTA_MASK           GENMASK(7, 0)
#define MAX20355_BB_ALG1_AVG_T_DELTA_SHIFT          0

/* --- BB_MULTI (0x4B) --- */
#define MAX20355_REG_BB_MULTI                       0x4B

#define MAX20355_BB_MULTI_UP_DWN_DIS_BIT            BIT(2)
#define MAX20355_BB_MULTI_UP_DWN_DIS_SHIFT          2
#define MAX20355_BB_MULTI_MULTI_UP_BIT              BIT(1)
#define MAX20355_BB_MULTI_MULTI_UP_SHIFT            1
#define MAX20355_BB_MULTI_MULTI_DW_BIT              BIT(0)
#define MAX20355_BB_MULTI_MULTI_DW_SHIFT            0

/* --- GPIOx (0x58/0x59/0x5A/0x5B) --- */
#define MAX20355_REG_GPIO1                          0x58
#define MAX20355_REG_GPIO2                          0x59
#define MAX20355_REG_GPIO3                          0x5A
#define MAX20355_REG_GPIO4                          0x5B

#define MAX20355_GPIO_PLCCTR_BIT                    BIT(3)
#define MAX20355_GPIO_PLCCTR_SHIFT                  3
#define MAX20355_GPIO_ENRES_BIT                     BIT(2)
#define MAX20355_GPIO_ENRES_SHIFT                   2
#define MAX20355_GPIO_ENPUP_BIT                     BIT(1)
#define MAX20355_GPIO_ENPUP_SHIFT                   1
#define MAX20355_GPIO_DOUT_BIT                      BIT(0)
#define MAX20355_GPIO_DOUT_SHIFT                    0

/* --- GPIO_RDB1 (0x5C) --- */
#define MAX20355_REG_GPIO_RDB1                      0x5C

#define MAX20355_GPIO_RDB1_INP4_BIT                 BIT(7)
#define MAX20355_GPIO_RDB1_INP4_SHIFT               7
#define MAX20355_GPIO_RDB1_INP3_BIT                 BIT(6)
#define MAX20355_GPIO_RDB1_INP3_SHIFT               6
#define MAX20355_GPIO_RDB1_INP2_BIT                 BIT(5)
#define MAX20355_GPIO_RDB1_INP2_SHIFT               5
#define MAX20355_GPIO_RDB1_INP1_BIT                 BIT(4)
#define MAX20355_GPIO_RDB1_INP1_SHIFT               4
#define MAX20355_GPIO_RDB1_CMOS_EN_BIT              BIT(0)
#define MAX20355_GPIO_RDB1_CMOS_EN_SHIFT            0

/* --- GPIO_RDB2 (0x5D) --- */
#define MAX20355_REG_GPIO_RDB2                      0x5D

#define MAX20355_GPIO_RDB2_PLC2_GPIO4_BIT           BIT(7)
#define MAX20355_GPIO_RDB2_PLC2_GPIO4_SHIFT         7
#define MAX20355_GPIO_RDB2_PLC2_GPIO3_BIT           BIT(6)
#define MAX20355_GPIO_RDB2_PLC2_GPIO3_SHIFT         6
#define MAX20355_GPIO_RDB2_PLC2_GPIO2_BIT           BIT(5)
#define MAX20355_GPIO_RDB2_PLC2_GPIO2_SHIFT         5
#define MAX20355_GPIO_RDB2_PLC2_GPIO1_BIT           BIT(4)
#define MAX20355_GPIO_RDB2_PLC2_GPIO1_SHIFT         4
#define MAX20355_GPIO_RDB2_PLC1_GPIO4_BIT           BIT(3)
#define MAX20355_GPIO_RDB2_PLC1_GPIO4_SHIFT         3
#define MAX20355_GPIO_RDB2_PLC1_GPIO3_BIT           BIT(2)
#define MAX20355_GPIO_RDB2_PLC1_GPIO3_SHIFT         2
#define MAX20355_GPIO_RDB2_PLC1_GPIO2_BIT           BIT(1)
#define MAX20355_GPIO_RDB2_PLC1_GPIO2_SHIFT         1
#define MAX20355_GPIO_RDB2_PLC1_GPIO1_BIT           BIT(0)
#define MAX20355_GPIO_RDB2_PLC1_GPIO1_SHIFT         0

/* --- SOC_BYTE_1 (0x60) --- */
#define MAX20355_REG_SOC_BYTE_1                     0x60

#define MAX20355_SOC_BYTE_1_MASK                    GENMASK(7, 0)
#define MAX20355_SOC_BYTE_1_SHIFT                   0

/* --- SOC_BYTE_0 (0x61) --- */
#define MAX20355_REG_SOC_BYTE_0                     0x61

#define MAX20355_SOC_BYTE_0_MASK                    GENMASK(7, 0)
#define MAX20355_SOC_BYTE_0_SHIFT                   0

/* --- VCELL_BYTE_1 (0x62) --- */
#define MAX20355_REG_VCELL_BYTE_1                   0x62

#define MAX20355_VCELL_BYTE_1_MASK                  GENMASK(7, 0)
#define MAX20355_VCELL_BYTE_1_SHIFT                 0

/* --- VCELL_BYTE_0 (0x63) --- */
#define MAX20355_REG_VCELL_BYTE_0                   0x63

#define MAX20355_VCELL_BYTE_0_MASK                  GENMASK(7, 0)
#define MAX20355_VCELL_BYTE_0_SHIFT                 0

/* --- TTE_BYTE_1 (0x64) --- */
#define MAX20355_REG_TTE_BYTE_1                     0x64

#define MAX20355_TTE_BYTE_1_MASK                    GENMASK(7, 0)
#define MAX20355_TTE_BYTE_1_SHIFT                   0

/* --- TTE_BYTE_0 (0x65) --- */
#define MAX20355_REG_TTE_BYTE_0                     0x65

#define MAX20355_TTE_BYTE_0_MASK                    GENMASK(7, 0)
#define MAX20355_TTE_BYTE_0_SHIFT                   0

/* --- AVGVCELL_BYTE_1 (0x66) --- */
#define MAX20355_REG_AVGVCELL_BYTE_1                0x66

#define MAX20355_AVGVCELL_BYTE_1_MASK               GENMASK(7, 0)
#define MAX20355_AVGVCELL_BYTE_1_SHIFT              0

/* --- AVGVCELL_BYTE_0 (0x67) --- */
#define MAX20355_REG_AVGVCELL_BYTE_0                0x67

#define MAX20355_AVGVCELL_BYTE_0_MASK               GENMASK(7, 0)
#define MAX20355_AVGVCELL_BYTE_0_SHIFT              0

/* --- TTF_BYTE_1 (0x68) --- */
#define MAX20355_REG_TTF_BYTE_1                     0x68

#define MAX20355_TTF_BYTE_1_MASK                    GENMASK(7, 0)
#define MAX20355_TTF_BYTE_1_SHIFT                   0

/* --- TTF_BYTE_0 (0x69) --- */
#define MAX20355_REG_TTF_BYTE_0                     0x69

#define MAX20355_TTF_BYTE_0_MASK                    GENMASK(7, 0)
#define MAX20355_TTF_BYTE_0_SHIFT                   0

/* --- READY_REG (0x6A) --- */
#define MAX20355_REG_READY_REG                      0x6A

#define MAX20355_READY_DOP_RDY_SIG_BIT              BIT(7)
#define MAX20355_READY_DOP_RDY_SIG_SHIFT            7
#define MAX20355_READY_DOP_I2C_ENA_BIT              BIT(5)
#define MAX20355_READY_DOP_I2C_ENA_SHIFT            5
#define MAX20355_READY_TTF_RDY_BIT                  BIT(4)
#define MAX20355_READY_TTF_RDY_SHIFT                4
#define MAX20355_READY_AVGVCELL_RDY_BIT             BIT(3)
#define MAX20355_READY_AVGVCELL_RDY_SHIFT           3
#define MAX20355_READY_TTE_RDY_BIT                  BIT(2)
#define MAX20355_READY_TTE_RDY_SHIFT                2
#define MAX20355_READY_VCELL_RDY_BIT                BIT(1)
#define MAX20355_READY_VCELL_RDY_SHIFT              1
#define MAX20355_READY_SOC_RDY_BIT                  BIT(0)
#define MAX20355_READY_SOC_RDY_SHIFT                0

/* --- FG_RDY_1/3 (0x6B/0x6D) --- */
#define MAX20355_REG_FG_RDY_1                       0x6B
#define MAX20355_REG_FG_RDY_3                       0x6D

#define MAX20355_FG_RDY_SLV_AVG_MASK                GENMASK(7, 0)
#define MAX20355_FG_RDY_SLV_AVG_SHIFT               0

/* --- FG_RDY_2/4 (0x6C/0x6E) --- */
#define MAX20355_REG_FG_RDY_2                       0x6C
#define MAX20355_REG_FG_RDY_4                       0x6E

#define MAX20355_FG_RDY_SLV_SOC_MASK                GENMASK(7, 0)
#define MAX20355_FG_RDY_SLV_SOC_SHIFT               0

/* --- FG_RDY_5 (0x6F) --- */
#define MAX20355_REG_FG_RDY_5                       0x6F

#define MAX20355_FG_RDY_SLV2_CHG_DNE_BIT            BIT(7)
#define MAX20355_FG_RDY_SLV2_CHG_DNE_SHIFT          7
#define MAX20355_FG_RDY_SLV1_CHG_DNE_BIT            BIT(6)
#define MAX20355_FG_RDY_SLV1_CHG_DNE_SHIFT          6
#define MAX20355_FG_RDY_SLV_SOCRDY2_BIT             BIT(3)
#define MAX20355_FG_RDY_SLV_SOCRDY2_SHIFT           3
#define MAX20355_FG_RDY_SLV_AVGRDY2_BIT             BIT(2)
#define MAX20355_FG_RDY_SLV_AVGRDY2_SHIFT           2
#define MAX20355_FG_RDY_SLV_SOCRDY1_BIT             BIT(1)
#define MAX20355_FG_RDY_SLV_SOCRDY1_SHIFT           1
#define MAX20355_FG_RDY_SLV_AVGRDY1_BIT             BIT(0)
#define MAX20355_FG_RDY_SLV_AVGRDY1_SHIFT           0

/* --- ADC_CTRL1 (0x70) --- */
#define MAX20355_REG_ADC_CTRL1                      0x70

#define MAX20355_ADC_CTRL1_ADCGNDTRH_MASK           GENMASK(6, 3)
#define MAX20355_ADC_CTRL1_ADCGNDTRH_SHIFT          3
#define MAX20355_ADC_CTRL1_RESDETRTY_MASK           GENMASK(2, 0)
#define MAX20355_ADC_CTRL1_RESDETRTY_SHIFT          0

/* --- ADC_CTRL2 (0x71) --- */
#define MAX20355_REG_ADC_CTRL2                      0x71

#define MAX20355_ADC_CTRL2_ADCRNG_MASK              GENMASK(5, 0)
#define MAX20355_ADC_CTRL2_ADCRNG_SHIFT             0

/* --- ADC_CTRL3 (0x72) --- */
#define MAX20355_REG_ADC_CTRL3                      0x72

#define MAX20355_ADC_CTRL3_ADCAVGNUM_MASK           GENMASK(2, 0)
#define MAX20355_ADC_CTRL3_ADCAVGNUM_SHIFT          0

/* --- ADC_CTRL4 (0x73) --- */
#define MAX20355_REG_ADC_CTRL4                      0x73

#define MAX20355_ADC_CTRL4_ADCNOISECTR_MASK         GENMASK(5, 0)
#define MAX20355_ADC_CTRL4_ADCNOISECTR_SHIFT        0

/* --- MOI_DET_REG1 (0x74) --- */
#define MAX20355_REG_MOI_DET_REG1                   0x74

#define MAX20355_MOI_DET_REG1_RACCDET_MIP_MASK      GENMASK(1, 0)
#define MAX20355_MOI_DET_REG1_RACCDET_MIP_SHIFT     0

/* --- MOI_DET_REG2 (0x75) --- */
#define MAX20355_REG_MOI_DET_REG2                   0x75

#define MAX20355_MOI_DET_REG2_RACCDETMTHR_MASK      GENMASK(7, 0)
#define MAX20355_MOI_DET_REG2_RACCDETMTHR_SHIFT     0

/* --- MOI_DET_REG3 (0x76) --- */
#define MAX20355_REG_MOI_DET_REG3                   0x76

#define MAX20355_MOI_DET_REG3_MOI_DET_AUT2_BIT      BIT(7)
#define MAX20355_MOI_DET_REG3_MOI_DET_AUT2_SHIFT    7
#define MAX20355_MOI_DET_REG3_MOI_DET_AUT1_BIT      BIT(6)
#define MAX20355_MOI_DET_REG3_MOI_DET_AUT1_SHIFT    6
#define MAX20355_MOI_DET_REG3_MOI_MAN_PL2_BIT       BIT(5)
#define MAX20355_MOI_DET_REG3_MOI_MAN_PL2_SHIFT     5
#define MAX20355_MOI_DET_REG3_MOI_MAN_PL1_BIT       BIT(4)
#define MAX20355_MOI_DET_REG3_MOI_MAN_PL1_SHIFT     4
#define MAX20355_MOI_DET_REG3_MOI_MAN_RTY2_BIT      BIT(3)
#define MAX20355_MOI_DET_REG3_MOI_MAN_RTY2_SHIFT    3
#define MAX20355_MOI_DET_REG3_MOI_MAN_RTY1_BIT      BIT(2)
#define MAX20355_MOI_DET_REG3_MOI_MAN_RTY1_SHIFT    2
#define MAX20355_MOI_DET_REG3_MOI_AUT_RTY2_BIT      BIT(1)
#define MAX20355_MOI_DET_REG3_MOI_AUT_RTY2_SHIFT    1
#define MAX20355_MOI_DET_REG3_MOI_AUT_RTY1_BIT      BIT(0)
#define MAX20355_MOI_DET_REG3_MOI_AUT_RTY1_SHIFT    0

/* --- IP_RES_REG (0x78) --- */
#define MAX20355_REG_IP_RES_REG                     0x78

#define MAX20355_IP_RES_REG_IP_RES_DET_MASK         GENMASK(1, 0)
#define MAX20355_IP_RES_REG_IP_RES_DET_SHIFT        0

/* --- ADC_VAL1 (0x79) --- */
#define MAX20355_REG_ADC_VAL1                       0x79

#define MAX20355_ADC_VAL1_ADCAVG_MASK               GENMASK(7, 0)
#define MAX20355_ADC_VAL1_ADCAVG_SHIFT              0

/* --- ADC_VAL2 (0x7A) --- */
#define MAX20355_REG_ADC_VAL2                       0x7A

#define MAX20355_ADC_VAL2_ADCMAX_MASK               GENMASK(7, 0)
#define MAX20355_ADC_VAL2_ADCMAX_SHIFT              0

/* --- ADC_VAL3 (0x7B) --- */
#define MAX20355_REG_ADC_VAL3                       0x7B

#define MAX20355_ADC_VAL2_ADCMIN_MASK               GENMASK(7, 0)
#define MAX20355_ADC_VAL2_ADCMIN_SHIFT              0

#define MAX20355_REG_MAX                            MAX20355_REG_ADC_VAL3

/* ---------------------------------------------------------- */
/* ------------------------ MAX20357 ------------------------ */
/* ---------------------------------------------------------- */

/* --- REVISION_ID (0x00) --- */
#define MAX20357_REG_REVISION_ID                    0x00

#define MAX20357_REVISION_ID_MASK                   GENMASK(7, 0)
#define MAX20357_REVISION_ID_SHIFT                  0

/* --- Status0 (0x01) --- */
#define MAX20357_REG_STATUS0                        0x01

#define MAX20357_STATUS0_CHN_CON_STS_BIT            BIT(7)
#define MAX20357_STATUS0_CHN_CON_STS_SHIFT          7
#define MAX20357_STATUS0_CHN_WTY_STS_BIT            BIT(6)
#define MAX20357_STATUS0_CHN_WTY_STS_SHIFT          6
#define MAX20357_STATUS0_CHN_IDL_STS_BIT            BIT(5)
#define MAX20357_STATUS0_CHN_IDL_STS_SHIFT          5
#define MAX20357_STATUS0_SRT_XFER_BIT               BIT(4)
#define MAX20357_STATUS0_SRT_XFER_SHIFT             4
#define MAX20357_STATUS0_LNG_XFER_BIT               BIT(3)
#define MAX20357_STATUS0_LNG_XFER_SHIFT             3
#define MAX20357_STATUS0_THMSTAT_MASK               GENMASK(2, 0)
#define MAX20357_STATUS0_THMSTAT_SHIFT              0

/* --- Status1 (0x02) --- */
#define MAX20357_REG_STATUS1                        0x02

#define MAX20357_STATUS1_JEITA_IS_REG_BIT           BIT(7)
#define MAX20357_STATUS1_JEITA_IS_REG_SHIFT         7
#define MAX20357_STATUS1_CHG_RESTA_B_BIT            BIT(6)
#define MAX20357_STATUS1_CHG_RESTA_B_SHIFT          6
#define MAX20357_STATUS1_CHG_THRM_REG_BIT           BIT(5)
#define MAX20357_STATUS1_CHG_THRM_REG_SHIFT         5
#define MAX20357_STATUS1_CC1_TMO_BIT                BIT(4)
#define MAX20357_STATUS1_CC1_TMO_SHIFT              4
#define MAX20357_STATUS1_CHGSTAT_MASK               GENMASK(3, 0)
#define MAX20357_STATUS1_CHGSTAT_SHIFT              0

/* --- Status2 (0x03) --- */
#define MAX20357_REG_STATUS2                        0x03

#define MAX20357_STATUS2_PLC_SUMACT_BIT             BIT(7)
#define MAX20357_STATUS2_PLC_SUMACT_SHIFT           7
#define MAX20357_STATUS2_PLC_SUMCURR_MASK           GENMASK(6, 1)
#define MAX20357_STATUS2_PLC_SUMCURR_SHIFT          1
#define MAX20357_STATUS2_DEAD_FOUND_STS_BIT         BIT(0)
#define MAX20357_STATUS2_DEAD_FOUND_STS_SHIFT       0

/* --- Status3 (0x04) --- */
#define MAX20357_REG_STATUS3                        0x04

#define MAX20357_STATUS3_SYSMINREG_BIT              BIT(7)
#define MAX20357_STATUS3_SYSMINREG_SHIFT            7
#define MAX20357_STATUS3_CHGREV_BIT                 BIT(6)
#define MAX20357_STATUS3_CHGREV_SHIFT               6
#define MAX20357_STATUS3_CHGVOLTMODE_BIT            BIT(5)
#define MAX20357_STATUS3_CHGVOLTMODE_SHIFT          5
#define MAX20357_STATUS3_CHGVOLTSTP_BIT             BIT(4)
#define MAX20357_STATUS3_CHGVOLTSTP_SHIFT           4
#define MAX20357_STATUS3_CHGGMD_BIT                 BIT(3)
#define MAX20357_STATUS3_CHGGMD_SHIFT               3
#define MAX20357_STATUS3_LDOGMD_BIT                 BIT(2)
#define MAX20357_STATUS3_LDOGMD_SHIFT               2
#define MAX20357_STATUS3_PLCOk_BIT                  BIT(1)
#define MAX20357_STATUS3_PLCOk_SHIFT                1
#define MAX20357_STATUS3_SYSREV_BIT                 BIT(0)
#define MAX20357_STATUS3_SYSREV_SHIFT               0

/* --- Status4 (0x05) --- */
#define MAX20357_REG_STATUS4                        0x05

#define MAX20357_STATUS4_PLC_STATUS_MASK            GENMASK(7, 0)
#define MAX20357_STATUS4_PLC_STATUS_SHIFT           0

/* --- Status5 (0x06) --- */
#define MAX20357_REG_STATUS5                        0x06

#define MAX20357_STATUS5_ITF_RDY_STS_BIT            BIT(7)
#define MAX20357_STATUS5_ITF_RDY_STS_SHIFT          7
#define MAX20357_STATUS5_BATUVLOB_BIT               BIT(6)
#define MAX20357_STATUS5_BATUVLOB_SHIFT             6
#define MAX20357_STATUS5_DEAD_BATT_BIT              BIT(5)
#define MAX20357_STATUS5_DEAD_BATT_SHIFT            5
#define MAX20357_STATUS5_PLC_MOI_DET_BIT            BIT(4)
#define MAX20357_STATUS5_PLC_MOI_DET_SHIFT          4
#define MAX20357_STATUS5_BOTCODE_LTC_MASK           GENMASK(3, 0)
#define MAX20357_STATUS5_BOTCODE_LTC_SHIFT          0

/* --- Status6 (0x07) --- */
#define MAX20357_REG_STATUS6                        0x07

#define MAX20357_STATUS6_URT_SWC_OPN_BIT            BIT(4)
#define MAX20357_STATUS6_URT_SWC_OPN_SHIFT          4
#define MAX20357_STATUS6_CHG_PRQ_INP_BIT            BIT(3)
#define MAX20357_STATUS6_CHG_PRQ_INP_SHIFT          3
#define MAX20357_STATUS6_SWC_OFF_MOD_MASK           GENMASK(2, 0)
#define MAX20357_STATUS6_SWC_OFF_MOD_SHIFT          0

/* --- Int0 (0x08) --- */
#define MAX20357_REG_INT0                           0x08

#define MAX20357_INT0_PLC_SUMACT_BIT                BIT(7)
#define MAX20357_INT0_PLC_SUMACT_SHIFT              7
#define MAX20357_INT0_PLC_SUMCURR_BIT               BIT(6)
#define MAX20357_INT0_PLC_SUMCURR_SHIFT             6
#define MAX20357_INT0_CHG_THRM_REG_BIT              BIT(5)
#define MAX20357_INT0_CHG_THRM_REG_SHIFT            5
#define MAX20357_INT0_CC1_TMO_BIT                   BIT(4)
#define MAX20357_INT0_CC1_TMO_SHIFT                 4
#define MAX20357_INT0_CHGSTAT_BIT                   BIT(3)
#define MAX20357_INT0_CHGSTAT_SHIFT                 3
#define MAX20357_INT0_SYSMINREG_BIT                 BIT(2)
#define MAX20357_INT0_SYSMINREG_SHIFT               2
#define MAX20357_INT0_CHG_RESTA_B_BIT               BIT(1)
#define MAX20357_INT0_CHG_RESTA_B_SHIFT             1
#define MAX20357_INT0_THMSTAT_BIT                   BIT(0)
#define MAX20357_INT0_THMSTAT_SHIFT                 0

/* --- Int1 (0x09) --- */
#define MAX20357_REG_INT1                           0x09

#define MAX20357_INT1_JEITA_IS_REG_BIT              BIT(7)
#define MAX20357_INT1_JEITA_IS_REG_SHIFT            7
#define MAX20357_INT1_CHG_REV_BIT                   BIT(6)
#define MAX20357_INT1_CHG_REV_SHIFT                 6
#define MAX20357_INT1_CHG_VOLT_MODE_BIT             BIT(5)
#define MAX20357_INT1_CHG_VOLT_MODE_SHIFT           5
#define MAX20357_INT1_CHG_VOLT_STP_BIT              BIT(4)
#define MAX20357_INT1_CHG_VOLT_STP_SHIFT            4
#define MAX20357_INT1_CHG_GMD_BIT                   BIT(3)
#define MAX20357_INT1_CHG_GMD_SHIFT                 3
#define MAX20357_INT1_LDO_GMD_BIT                   BIT(2)
#define MAX20357_INT1_LDO_GMD_SHIFT                 2
#define MAX20357_INT1_PLCOk_BIT                     BIT(1)
#define MAX20357_INT1_PLCOk_SHIFT                   1
#define MAX20357_INT1_SYSREV_BIT                    BIT(0)
#define MAX20357_INT1_SYSREV_SHIFT                  0

/* --- Int2 (0x0A) --- */
#define MAX20357_REG_INT2                           0x0A

#define MAX20357_INT2_CHN_CON_BIT                   BIT(7)
#define MAX20357_INT2_CHN_CON_SHIFT                 7
#define MAX20357_INT2_CHN_WTY_BIT                   BIT(6)
#define MAX20357_INT2_CHN_WTY_SHIFT                 6
#define MAX20357_INT2_CHN_IDL_BIT                   BIT(5)
#define MAX20357_INT2_CHN_IDL_SHIFT                 5
#define MAX20357_INT2_SRT_XFER_RISE_BIT             BIT(4)
#define MAX20357_INT2_SRT_XFER_RISE_SHIFT           4
#define MAX20357_INT2_SRT_XFER_FALL_BIT             BIT(3)
#define MAX20357_INT2_SRT_XFER_FALL_SHIFT           3
#define MAX20357_INT2_PLC_NEW_DAT_BIT               BIT(2)
#define MAX20357_INT2_PLC_NEW_DAT_SHIFT             2
#define MAX20357_INT2_PLC_CMD_DNE_BIT               BIT(1)
#define MAX20357_INT2_PLC_CMD_DNE_SHIFT             1
#define MAX20357_INT2_PLC_CMD_ERR_BIT               BIT(0)
#define MAX20357_INT2_PLC_CMD_ERR_SHIFT             0

/* --- Int3 (0x0B) --- */
#define MAX20357_REG_INT3                           0x0B

#define MAX20357_INT3_LNG_XFER_BIT                  BIT(7)
#define MAX20357_INT3_LNG_XFER_SHIFT                7
#define MAX20357_INT3_BATUVLOB_BIT                  BIT(6)
#define MAX20357_INT3_BATUVLOB_SHIFT                6
#define MAX20357_INT3_MOI_DNE_BIT                   BIT(5)
#define MAX20357_INT3_MOI_DNE_SHIFT                 5
#define MAX20357_INT3_PLC_MOI_DET_BIT               BIT(4)
#define MAX20357_INT3_PLC_MOI_DET_SHIFT             4
#define MAX20357_INT3_MOI_DET_BIT                   BIT(3)
#define MAX20357_INT3_MOI_DET_SHIFT                 3
#define MAX20357_INT3_RES_DET_ABR_BIT               BIT(2)
#define MAX20357_INT3_RES_DET_ABR_SHIFT             2
#define MAX20357_INT3_RES_DET_OPN_BIT               BIT(1)
#define MAX20357_INT3_RES_DET_OPN_SHIFT             1
#define MAX20357_INT3_RES_DET_GND_BIT               BIT(0)
#define MAX20357_INT3_RES_DET_GND_SHIFT             0

/* --- Int4 (0x0C) --- */
#define MAX20357_REG_INT4                           0x0C

#define MAX20357_INT4_URT_TMO_FLT_BIT               BIT(7)
#define MAX20357_INT4_URT_TMO_FLT_SHIFT             7
#define MAX20357_INT4_URT_MODFAIL_BIT               BIT(6)
#define MAX20357_INT4_URT_MODFAIL_SHIFT             6
#define MAX20357_INT4_URT_MODDONE_BIT               BIT(5)
#define MAX20357_INT4_URT_MODDONE_SHIFT             5
#define MAX20357_INT4_URT_SWC_OPN_BIT               BIT(4)
#define MAX20357_INT4_URT_SWC_OPN_SHIFT             4
#define MAX20357_INT4_DEAD_FOUND_BIT                BIT(3)
#define MAX20357_INT4_DEAD_FOUND_SHIFT              3
#define MAX20357_INT4_SWC_OFF_MOD_BIT               BIT(1)
#define MAX20357_INT4_SWC_OFF_MOD_SHIFT             1
#define MAX20357_INT4_CHG_PRQ_INP_BIT               BIT(0)
#define MAX20357_INT4_CHG_PRQ_INP_SHIFT             0

/* --- Int5 (0x0D) --- */
#define MAX20357_REG_INT5                           0x0D

#define MAX20357_INT5_ITF_RDY_STS_BIT               BIT(7)
#define MAX20357_INT5_ITF_RDY_STS_SHIFT             7
#define MAX20357_INT5_WD_ITR_CLR_BIT                BIT(2)
#define MAX20357_INT5_WD_ITR_CLR_SHIFT              2

/* --- IntMask0 (0x0E) --- */
#define MAX20357_REG_INTMASK0                       0x0E

#define MAX20357_INTMASK0_PLC_SUMACT_BIT            BIT(7)
#define MAX20357_INTMASK0_PLC_SUMACT_SHIFT          7
#define MAX20357_INTMASK0_PLC_SUMCURR_BIT           BIT(6)
#define MAX20357_INTMASK0_PLC_SUMCURR_SHIFT         6
#define MAX20357_INTMASK0_CHG_THRM_REG_BIT          BIT(5)
#define MAX20357_INTMASK0_CHG_THRM_REG_SHIFT        5
#define MAX20357_INTMASK0_CC1_TMO_BIT               BIT(4)
#define MAX20357_INTMASK0_CC1_TMO_SHIFT             4
#define MAX20357_INTMASK0_CHGSTAT_BIT               BIT(3)
#define MAX20357_INTMASK0_CHGSTAT_SHIFT             3
#define MAX20357_INTMASK0_SYSMINREG_BIT             BIT(2)
#define MAX20357_INTMASK0_SYSMINREG_SHIFT           2
#define MAX20357_INTMASK0_CHG_RESTA_B_BIT           BIT(1)
#define MAX20357_INTMASK0_CHG_RESTA_B_SHIFT         1
#define MAX20357_INTMASK0_THMSTAT_BIT               BIT(0)
#define MAX20357_INTMASK0_THMSTAT_SHIFT             0

/* --- IntMask1 (0x0F) --- */
#define MAX20357_REG_INTMASK1                       0x0F

#define MAX20357_INTMASK1_JEITA_IS_REG_BIT          BIT(7)
#define MAX20357_INTMASK1_JEITA_IS_REG_SHIFT        7
#define MAX20357_INTMASK1_CHG_REV_BIT               BIT(6)
#define MAX20357_INTMASK1_CHG_REV_SHIFT             6
#define MAX20357_INTMASK1_CHG_VOLT_MODE_BIT         BIT(5)
#define MAX20357_INTMASK1_CHG_VOLT_MODE_SHIFT       5
#define MAX20357_INTMASK1_CHG_VOLT_STP_BIT          BIT(4)
#define MAX20357_INTMASK1_CHG_VOLT_STP_SHIFT        4
#define MAX20357_INTMASK1_CHG_GMD_BIT               BIT(3)
#define MAX20357_INTMASK1_CHG_GMD_SHIFT             3
#define MAX20357_INTMASK1_LDO_GMD_BIT               BIT(2)
#define MAX20357_INTMASK1_LDO_GMD_SHIFT             2
#define MAX20357_INTMASK1_PLCOK_BIT                 BIT(1)
#define MAX20357_INTMASK1_PLCOK_SHIFT               1
#define MAX20357_INTMASK1_SYSREV_BIT                BIT(0)
#define MAX20357_INTMASK1_SYSREV_SHIFT              0

/* --- IntMask2 (0x10) --- */
#define MAX20357_REG_INTMASK2                       0x10

#define MAX20357_INTMASK2_CHN_CON_BIT               BIT(7)
#define MAX20357_INTMASK2_CHN_CON_SHIFT             7
#define MAX20357_INTMASK2_CHN_WTY_BIT               BIT(6)
#define MAX20357_INTMASK2_CHN_WTY_SHIFT             6
#define MAX20357_INTMASK2_CHN_IDL_BIT               BIT(5)
#define MAX20357_INTMASK2_CHN_IDL_SHIFT             5
#define MAX20357_INTMASK2_SRT_XFER_RISE_BIT         BIT(4)
#define MAX20357_INTMASK2_SRT_XFER_RISE_SHIFT       4
#define MAX20357_INTMASK2_SRT_XFER_FALL_BIT         BIT(3)
#define MAX20357_INTMASK2_SRT_XFER_FALL_SHIFT       3
#define MAX20357_INTMASK2_PLC_NEW_DAT_BIT           BIT(2)
#define MAX20357_INTMASK2_PLC_NEW_DAT_SHIFT         2
#define MAX20357_INTMASK2_PLC_CMD_DNE_BIT           BIT(1)
#define MAX20357_INTMASK2_PLC_CMD_DNE_SHIFT         1
#define MAX20357_INTMASK2_PLC_CMD_ERR_BIT           BIT(0)
#define MAX20357_INTMASK2_PLC_CMD_ERR_SHIFT         0

/* --- IntMask3 (0x11) --- */
#define MAX20357_REG_INTMASK3                       0x11

#define MAX20357_INTMASK3_LNG_XFER_BIT              BIT(7)
#define MAX20357_INTMASK3_LNG_XFER_SHIFT            7
#define MAX20357_INTMASK3_BATUVLOB_BIT              BIT(6)
#define MAX20357_INTMASK3_BATUVLOB_SHIFT            6
#define MAX20357_INTMASK3_MOI_DNE_BIT               BIT(5)
#define MAX20357_INTMASK3_MOI_DNE_SHIFT             5
#define MAX20357_INTMASK3_PLC_MOI_DET_BIT           BIT(4)
#define MAX20357_INTMASK3_PLC_MOI_DET_SHIFT         4
#define MAX20357_INTMASK3_MOI_DET_BIT               BIT(3)
#define MAX20357_INTMASK3_MOI_DET_SHIFT             3
#define MAX20357_INTMASK3_RES_DET_ABR_BIT           BIT(2)
#define MAX20357_INTMASK3_RES_DET_ABR_SHIFT         2
#define MAX20357_INTMASK3_RES_DET_OPN_BIT           BIT(1)
#define MAX20357_INTMASK3_RES_DET_OPN_SHIFT         1
#define MAX20357_INTMASK3_RES_DET_GND_BIT           BIT(0)
#define MAX20357_INTMASK3_RES_DET_GND_SHIFT         0

/* --- IntMask4 (0x12) --- */
#define MAX20357_REG_INTMASK4                       0x12

#define MAX20357_INTMASK4_URT_TMO_FLT_BIT           BIT(7)
#define MAX20357_INTMASK4_URT_TMO_FLT_SHIFT         7
#define MAX20357_INTMASK4_URT_MODFAIL_BIT           BIT(6)
#define MAX20357_INTMASK4_URT_MODFAIL_SHIFT         6
#define MAX20357_INTMASK4_URT_MODDONE_BIT           BIT(5)
#define MAX20357_INTMASK4_URT_MODDONE_SHIFT         5
#define MAX20357_INTMASK4_URT_SWC_OPN_BIT           BIT(4)
#define MAX20357_INTMASK4_URT_SWC_OPN_SHIFT         4
#define MAX20357_INTMASK4_DEAD_FOUND_BIT            BIT(3)
#define MAX20357_INTMASK4_DEAD_FOUND_SHIFT          3
#define MAX20357_INTMASK4_SWC_OFF_MOD_BIT           BIT(1)
#define MAX20357_INTMASK4_SWC_OFF_MOD_SHIFT         1
#define MAX20357_INTMASK4_CHG_PRQ_INP_BIT           BIT(0)
#define MAX20357_INTMASK4_CHG_PRQ_INP_SHIFT         0

/* --- IntMask5 (0x13) --- */
#define MAX20357_REG_INTMASK5                       0x13

#define MAX20357_INTMASK5_ITF_RDY_STS_BIT           BIT(7)
#define MAX20357_INTMASK5_ITF_RDY_STS_SHIFT         7
#define MAX20357_INTMASK5_WD_ITR_CLR_BIT            BIT(2)
#define MAX20357_INTMASK5_WD_ITR_CLR_SHIFT          2

/* --- SYSTEM_REG0 (0x1A) --- */
#define MAX20357_REG_SYSTEM_REG0                    0x1A

#define MAX20357_SYSTEM_OFF_CMD_INP_BIT             BIT(7)
#define MAX20357_SYSTEM_OFF_CMD_INP_SHIFT           7
#define MAX20357_SYSTEM_SOFT_RESET_BIT              BIT(6)
#define MAX20357_SYSTEM_SOFT_RESET_SHIFT            6
#define MAX20357_SYSTEM_HARD_RESET_BIT              BIT(5)
#define MAX20357_SYSTEM_HARD_RESET_SHIFT            5
#define MAX20357_SYSTEM_SFT_HRD_RST_BIT             BIT(4)
#define MAX20357_SYSTEM_SFT_HRD_RST_SHIFT           4
#define MAX20357_SYSTEM_SEAL_I2C_CMD_BIT            BIT(3)
#define MAX20357_SYSTEM_SEAL_I2C_CMD_SHIFT          3
#define MAX20357_SYSTEM_STAY_ON_BIT                 BIT(1)
#define MAX20357_SYSTEM_STAY_ON_SHIFT               1
#define MAX20357_SYSTEM_ENB_OTP_ENA_BIT             BIT(0)
#define MAX20357_SYSTEM_ENB_OTP_ENA_SHIFT           0

/* --- BOT_CMD (0x1B) --- */
#define MAX20357_REG_BOT_CMD                        0x1B

#define MAX20357_BOTCMD_FG_RESET_BIT                BIT(7)
#define MAX20357_BOTCMD_FG_RESET_SHIFT              7
#define MAX20357_BOTCMD_FG_ENA_BYP_BIT              BIT(6)
#define MAX20357_BOTCMD_FG_ENA_BYP_SHIFT            6
#define MAX20357_BOTCMD_FG_ENA_VAL_BIT              BIT(5)
#define MAX20357_BOTCMD_FG_ENA_VAL_SHIFT            5
#define MAX20357_BOTCMD_JTA_HRS_ENA_BIT             BIT(0)
#define MAX20357_BOTCMD_JTA_HRS_ENA_SHIFT           0

/* --- BOT_RDB (0x1C) --- */
#define MAX20357_REG_BOT_RDB                        0x1C

#define MAX20357_BOTRDB_RESET_MODE_MASK             GENMASK(7, 5)
#define MAX20357_BOTRDB_RESET_MODE_SHIFT            5

/* --- UART_Ctr0 (0x20) --- */
#define MAX20357_REG_UART_CTR0                      0x20

#define MAX20357_UART_CTR0_URT_AUTO_EN_BIT          BIT(2)
#define MAX20357_UART_CTR0_URT_AUTO_EN_SHIFT        2
#define MAX20357_UART_CTR0_RXS_TMO_TIM_MASK         GENMASK(1, 0)
#define MAX20357_UART_CTR0_RXS_TMO_TIM_SHIFT        0

/* --- UART_Ctr1 (0x21) --- */
#define MAX20357_REG_UART_CTR1                      0x21

#define MAX20357_UART_CTR1_TMO_TMR_ENA_BIT          BIT(6)
#define MAX20357_UART_CTR1_TMO_TMR_ENA_SHIFT        6
#define MAX20357_UART_CTR1_I2C_URT_MOD_BIT          BIT(5)
#define MAX20357_UART_CTR1_I2C_URT_MOD_SHIFT        5
#define MAX20357_UART_CTR1_I2C_URT_ENA_BIT          BIT(4)
#define MAX20357_UART_CTR1_I2C_URT_ENA_SHIFT        4
#define MAX20357_UART_CTR1_I2C_URT_ABR_BIT          BIT(3)
#define MAX20357_UART_CTR1_I2C_URT_ABR_SHIFT        3
#define MAX20357_UART_CTR1_I2C_URT_SWC_BIT          BIT(2)
#define MAX20357_UART_CTR1_I2C_URT_SWC_SHIFT        2
#define MAX20357_UART_CTR1_I2C_TX_SWC_BIT           BIT(1)
#define MAX20357_UART_CTR1_I2C_TX_SWC_SHIFT         1
#define MAX20357_UART_CTR1_I2C_RX_SWC_BIT           BIT(0)
#define MAX20357_UART_CTR1_I2C_RX_SWC_SHIFT         0

/* --- SYSTEM_CONFIG0 (0x30) --- */
#define MAX20357_REG_SYSTEM_CONFIG0                 0x30

#define MAX20357_SYSTEM_CONFIG0_CHG_WKP_EN_BIT      BIT(7)
#define MAX20357_SYSTEM_CONFIG0_CHG_WKP_EN_SHIFT    7
#define MAX20357_SYSTEM_CONFIG0_LOW_PWR_EN_BIT      BIT(6)
#define MAX20357_SYSTEM_CONFIG0_LOW_PWR_EN_SHIFT    6
#define MAX20357_SYSTEM_CONFIG0_ICHG_X2_BIT         BIT(4)
#define MAX20357_SYSTEM_CONFIG0_ICHG_X2_SHIFT       4
#define MAX20357_SYSTEM_CONFIG0_CHG_RES_EN_BIT      BIT(3)
#define MAX20357_SYSTEM_CONFIG0_CHG_RES_EN_SHIFT    3
#define MAX20357_SYSTEM_CONFIG0_I2C_LDO_EN_BIT      BIT(2)
#define MAX20357_SYSTEM_CONFIG0_I2C_LDO_EN_SHIFT    2
#define MAX20357_SYSTEM_CONFIG0_SYSUV_SEL_MASK      GENMASK(1, 0)
#define MAX20357_SYSTEM_CONFIG0_SYSUV_SEL_SHIFT     0

/* --- PLC_CONFIG0 (0x31) --- */
#define MAX20357_REG_PLC_CONFIG0                    0x31

#define MAX20357_PLC_CFG0_PLCTHRSEL_MASK            GENMASK(7, 6)
#define MAX20357_PLC_CFG0_PLCTHRSEL_SHIFT           6
#define MAX20357_PLC_CFG0_PLCSNKSEL_MASK            GENMASK(1, 0)
#define MAX20357_PLC_CFG0_PLCSNKSEL_SHIFT           0

/* --- PLC_CONFIG1 (0x32) --- */
#define MAX20357_REG_PLC_CONFIG1                    0x32

#define MAX20357_PLC_CFG1_OTP_SUM_REV_BIT   	    BIT(7)
#define MAX20357_PLC_CFG1_OTP_SUM_REV_SHIFT 	    7
#define MAX20357_PLC_CFG1_FRC_I2C_SUM_BIT   	    BIT(6)
#define MAX20357_PLC_CFG1_FRC_I2C_SUM_SHIFT 	    6
#define MAX20357_PLC_CFG1_PLCCURR_MASK      	    GENMASK(5, 0)
#define MAX20357_PLC_CFG1_PLCCURR_SHIFT     	    0

/* --- PLC_CONFIG2 (0x33) --- */
#define MAX20357_REG_PLC_CONFIG2                    0x33

#define MAX20357_PLC_CFG2_PLC_DSC_OTP_BIT           BIT(7)
#define MAX20357_PLC_CFG2_PLC_DSC_OTP_SHIFT         7
#define MAX20357_PLC_CFG2_PLC_DROP_MASK             GENMASK(6, 4)
#define MAX20357_PLC_CFG2_PLC_DROP_SHIFT            4
#define MAX20357_PLC_CFG2_PLC_HLD_MASK              GENMASK(3, 2)
#define MAX20357_PLC_CFG2_PLC_HLD_SHIFT             2
#define MAX20357_PLC_CFG2_PLC_HREF_MASK             GENMASK(1, 0)
#define MAX20357_PLC_CFG2_PLC_HREF_SHIFT            0

/* --- PLC_CONFIG3 (0x34) --- */
#define MAX20357_REG_PLC_CONFIG3                    0x34

#define MAX20357_PLC_CFG3_PLC_MONITOR_BIT           BIT(7)
#define MAX20357_PLC_CFG3_PLC_MONITOR_SHIFT         7
#define MAX20357_PLC_CFG3_PLC_CON_STS_BIT           BIT(6)
#define MAX20357_PLC_CFG3_PLC_CON_STS_SHIFT         6
#define MAX20357_PLC_CFG3_PLC_CONFIG_MASK           GENMASK(5, 3)
#define MAX20357_PLC_CFG3_PLC_CONFIG_SHIFT          3
#define MAX20357_PLC_CFG3_DAT_MAX_RTY_MASK          GENMASK(2, 0)
#define MAX20357_PLC_CFG3_DAT_MAX_RTY_SHIFT         0

/* --- PLC_CONFIG4 (0x35) --- */
#define MAX20357_REG_PLC_CONFIG4                    0x35

#define MAX20357_PLC_CFG4_PLC_FSM_ENA_BIT           BIT(7)
#define MAX20357_PLC_CFG4_PLC_FSM_ENA_SHIFT         7
#define MAX20357_PLC_CFG4_RAM_IS_FULL_BIT           BIT(6)
#define MAX20357_PLC_CFG4_RAM_IS_FULL_SHIFT         6
#define MAX20357_PLC_CFG4_FIFO_MASTER_BIT           BIT(5)
#define MAX20357_PLC_CFG4_FIFO_MASTER_SHIFT         5
#define MAX20357_PLC_CFG4_FIFO_SLAVE_BIT            BIT(4)
#define MAX20357_PLC_CFG4_FIFO_SLAVE_SHIFT          4
#define MAX20357_PLC_CFG4_PLC_IS_FULL_BIT           BIT(3)
#define MAX20357_PLC_CFG4_PLC_IS_FULL_SHIFT         3
#define MAX20357_PLC_CFG4_PNG_TIMEOUT_BIT           BIT(2)
#define MAX20357_PLC_CFG4_PNG_TIMEOUT_SHIFT         2
#define MAX20357_PLC_CFG4_PLC_RES_REQ_BIT           BIT(1)
#define MAX20357_PLC_CFG4_PLC_RES_REQ_SHIFT         1
#define MAX20357_PLC_CFG4_CONT_STREAM_BIT           BIT(0)
#define MAX20357_PLC_CFG4_CONT_STREAM_SHIFT         0

/* --- PLC_CONFIG5 (0x36) --- */
#define MAX20357_REG_PLC_CONFIG5                    0x36

#define MAX20357_PLC_CFG5_NO_FIFO_SLAVE_BIT         BIT(7)
#define MAX20357_PLC_CFG5_NO_FIFO_SLAVE_SHIFT       7
#define MAX20357_PLC_CFG5_NO_SEAL_MDE_BIT           BIT(6)
#define MAX20357_PLC_CFG5_NO_SEAL_MDE_SHIFT         6
#define MAX20357_PLC_CFG5_NO_UART_MDE_BIT           BIT(5)
#define MAX20357_PLC_CFG5_NO_UART_MDE_SHIFT         5
#define MAX20357_PLC_CFG5_NO_IDLE_MDE_BIT           BIT(4)
#define MAX20357_PLC_CFG5_NO_IDLE_MDE_SHIFT         4
#define MAX20357_PLC_CFG5_SWP_PLC_RAM_BIT           BIT(0)
#define MAX20357_PLC_CFG5_SWP_PLC_RAM_SHIFT         0

/* --- PLC_ARG (0x37) --- */
#define MAX20357_REG_PLC_ARG                        0x37

#define MAX20357_PLC_ARG_CMD_ARG_MASK               GENMASK(7, 0)
#define MAX20357_PLC_ARG_CMD_ARG_SHIFT              0

/* --- PLC_CMD (0x38) --- */
#define MAX20357_REG_PLC_CMD                        0x38

#define MAX20357_PLC_CMD_RUN_TRG_BIT                BIT(7)
#define MAX20357_PLC_CMD_RUN_TRG_SHIFT              7
#define MAX20357_PLC_CMD_COMMAND_MASK               GENMASK(6, 0)
#define MAX20357_PLC_CMD_COMMAND_SHIFT              0

/* --- PLC_RX (0x39) --- */
#define MAX20357_REG_PLC_RX                         0x39

#define MAX20357_PLC_RX_BYTES_MASK                  GENMASK(6, 0)
#define MAX20357_PLC_RX_BYTES_SHIFT                 0

/* --- Master_feed_back0 (0x3A) --- */
#define MAX20357_REG_MASTER_FEEDBACK0               0x3A

#define MAX20357_MST_SOC_VAL_MASK                   GENMASK(7, 0)
#define MAX20357_MST_SOC_VAL_SHIFT                  0

/* --- Master_feed_back1 (0x3B) --- */
#define MAX20357_REG_MASTER_FEEDBACK1               0x3B

#define MAX20357_MST_FGREADY_MASK                   BIT(7)
#define MAX20357_MST_FGREADY_SHIFT                  7

/* --- WATCHDOG0 (0x3C) --- */
#define MAX20357_REG_WATCHDOG0                      0x3C

#define MAX20357_WATCHDOG0_WD_RST_TYPE_MASK         GENMASK(3, 2)
#define MAX20357_WATCHDOG0_WD_RST_TYPE_SHIFT        2
#define MAX20357_WATCHDOG0_WD_EOC_SEL_MASK          GENMASK(1, 0)
#define MAX20357_WATCHDOG0_WD_EOC_SEL_SHIFT         0

/* --- SYS_MIN0 (0x3D) --- */
#define MAX20357_REG_SYS_MIN0                       0x3D

#define MAX20357_SYS_MIN0_PP_DRP_MASK               GENMASK(7, 5)
#define MAX20357_SYS_MIN0_PP_DRP_SHIFT              5
#define MAX20357_SYS_MIN0_SYSMIN_FORCE_BIT          BIT(4)
#define MAX20357_SYS_MIN0_SYSMIN_FORCE_SHIFT        4
#define MAX20357_SYS_MIN0_SYSMIN_MASK               GENMASK(3, 0)
#define MAX20357_SYS_MIN0_SYSMIN_SHIFT              0

/* --- ILimCtrlChg (0x40) --- */
#define MAX20357_REG_ILIMCTRL_CHG                   0x40

#define MAX20357_ILIMCTRL_CHG_CHGTHRMLIM_MASK       GENMASK(3, 0)
#define MAX20357_ILIMCTRL_CHG_CHGTHRMLIM_SHIFT      0

/* --- ChgCur0 (0x41) --- */
#define MAX20357_REG_CHG_CUR0                       0x41

#define MAX20357_CHG_CUR0_CC1IFCHG_MASK             GENMASK(6, 0)
#define MAX20357_CHG_CUR0_CC1IFCHG_SHIFT            0

/* --- ChgCur1 (0x42) --- */
#define MAX20357_REG_CHG_CUR1                       0x42

#define MAX20357_CHG_CUR1_CC2IFCHG_MASK             GENMASK(6, 0)
#define MAX20357_CHG_CUR1_CC2IFCHG_SHIFT            0

/* --- ChgCntl0 (0x43) --- */
#define MAX20357_REG_CHG_CNTL0                      0x43

#define MAX20357_CHG_CNTL0_CHG_EN_BIT               BIT(7)
#define MAX20357_CHG_CNTL0_CHG_EN_SHIFT             7
#define MAX20357_CHG_CNTL0_CHG_AUTOSTOP_BIT         BIT(6)
#define MAX20357_CHG_CNTL0_CHG_AUTOSTOP_SHIFT       6
#define MAX20357_CHG_CNTL0_CHG_AUTORESTA_BIT        BIT(5)
#define MAX20357_CHG_CNTL0_CHG_AUTORESTA_SHIFT      5
#define MAX20357_CHG_CNTL0_CC1_ROOM_ONLY_BIT        BIT(2)
#define MAX20357_CHG_CNTL0_CC1_ROOM_ONLY_SHIFT      2
#define MAX20357_CHG_CNTL0_CC1_TMO_LIMIT_BIT        BIT(1)
#define MAX20357_CHG_CNTL0_CC1_TMO_LIMIT_SHIFT      1
#define MAX20357_CHG_CNTL0_CC1_ENABLE_BIT           BIT(0)
#define MAX20357_CHG_CNTL0_CC1_ENABLE_SHIFT         0

/* --- ChgCntl1 (0x44) --- */
#define MAX20357_REG_CHG_CNTL1                      0x44

#define MAX20357_CHG_CNTL1_BATRECHG_MASK            GENMASK(7, 6)
#define MAX20357_CHG_CNTL1_BATRECHG_SHIFT           6
#define MAX20357_CHG_CNTL1_BATREG_MASK              GENMASK(5, 0)
#define MAX20357_CHG_CNTL1_BATREG_SHIFT             0

/* --- ChgCntl2 (0x45) --- */
#define MAX20357_REG_CHG_CNTL2                      0x45

#define MAX20357_CHG_CNTL2_VPCHG_MASK               GENMASK(6, 4)
#define MAX20357_CHG_CNTL2_VPCHG_SHIFT              4
#define MAX20357_CHG_CNTL2_IPCHG_MASK               GENMASK(3, 2)
#define MAX20357_CHG_CNTL2_IPCHG_SHIFT              2
#define MAX20357_CHG_CNTL2_ICHGDONE_MASK            GENMASK(1, 0)
#define MAX20357_CHG_CNTL2_ICHGDONE_SHIFT           0

/* --- ChgTmr (0x46) --- */
#define MAX20357_REG_CHG_TMR                        0x46

#define MAX20357_CHG_TMR_MTCHGTMR_MASK              GENMASK(7, 6)
#define MAX20357_CHG_TMR_MTCHGTMR_SHIFT             6
#define MAX20357_CHG_TMR_PCHGTMR_MASK               GENMASK(5, 4)
#define MAX20357_CHG_TMR_PCHGTMR_SHIFT              4
#define MAX20357_CHG_TMR_CC1FCHGTMR_MASK            GENMASK(3, 2)
#define MAX20357_CHG_TMR_CC1FCHGTMR_SHIFT           2
#define MAX20357_CHG_TMR_CHGTMR_MASK                GENMASK(1, 0)
#define MAX20357_CHG_TMR_CHGTMR_SHIFT               0

/* --- ChgCfg0 (0x47) --- */
#define MAX20357_REG_CHG_CFG0                       0x47

#define MAX20357_CHG_CFG0_CHGSTEPHYS_MASK           GENMASK(6, 4)
#define MAX20357_CHG_CFG0_CHGSTEPHYS_SHIFT          4
#define MAX20357_CHG_CFG0_CHGSTEPRISE_MASK          GENMASK(3, 0)
#define MAX20357_CHG_CFG0_CHGSTEPRISE_SHIFT         0

/* --- ThmCfg0 (0x48) --- */
#define MAX20357_REG_THM_CFG0                       0x48

#define MAX20357_THM_CFG0_CHGCOOL_CC1FCHG_MASK      GENMASK(7, 5)
#define MAX20357_THM_CFG0_CHGCOOL_CC1FCHG_SHIFT     5
#define MAX20357_THM_CFG0_CHGCOOL_BATREG_MASK       GENMASK(4, 3)
#define MAX20357_THM_CFG0_CHGCOOL_BATREG_SHIFT      3
#define MAX20357_THM_CFG0_CHGCOOL_CC2IFCHG_MASK     GENMASK(2, 0)
#define MAX20357_THM_CFG0_CHGCOOL_CC2IFCHG_SHIFT    0

/* --- ThmCfg1 (0x49) --- */
#define MAX20357_REG_THM_CFG1                       0x49

#define MAX20357_THM_CFG1_CHGROOM_CC1FCHG_MASK      GENMASK(7, 5)
#define MAX20357_THM_CFG1_CHGROOM_CC1FCHG_SHIFT     5
#define MAX20357_THM_CFG1_CHGROOM_BATREG_MASK       GENMASK(4, 3)
#define MAX20357_THM_CFG1_CHGROOM_BATREG_SHIFT      3
#define MAX20357_THM_CFG1_CHGROOM_CC2IFCHG_MASK     GENMASK(2, 0)
#define MAX20357_THM_CFG1_CHGROOM_CC2IFCHG_SHIFT    0

/* --- ThmCfg2 (0x4A) --- */
#define MAX20357_REG_THM_CFG2                       0x4A

#define MAX20357_THM_CFG2_CHGWARM_CC1FCHG_MASK      GENMASK(7, 5)
#define MAX20357_THM_CFG2_CHGWARM_CC1FCHG_SHIFT     5
#define MAX20357_THM_CFG2_CHGWARM_BATREG_MASK       GENMASK(4, 3)
#define MAX20357_THM_CFG2_CHGWARM_BATREG_SHIFT      3
#define MAX20357_THM_CFG2_CHGWARM_CC2IFCHG_MASK     GENMASK(2, 0)
#define MAX20357_THM_CFG2_CHGWARM_CC2IFCHG_SHIFT    0

/* --- ThmCfg3 (0x4B) --- */
#define MAX20357_REG_THM_CFG3                       0x4B

#define MAX20357_THM_CFG3_CHGT1_THR_DEF_MASK        GENMASK(5, 3)
#define MAX20357_THM_CFG3_CHGT1_THR_DEF_SHIFT       3
#define MAX20357_THM_CFG3_CHGT1_THR_CC1_MASK        GENMASK(2, 0)
#define MAX20357_THM_CFG3_CHGT1_THR_CC1_SHIFT       0

/* --- ThmCfg4 (0x4C) --- */
#define MAX20357_REG_THM_CFG4                       0x4C

#define MAX20357_THM_CFG4_CHGT2_THR_DEF_MASK        GENMASK(5, 3)
#define MAX20357_THM_CFG4_CHGT2_THR_DEF_SHIFT       3
#define MAX20357_THM_CFG4_CHGT2_THR_CC1_MASK        GENMASK(2, 0)
#define MAX20357_THM_CFG4_CHGT2_THR_CC1_SHIFT       0

/* --- ThmCfg5 (0x4D) --- */
#define MAX20357_REG_THM_CFG5                       0x4D

#define MAX20357_THM_CFG5_CHGT3_THR_DEF_MASK        GENMASK(5, 3)
#define MAX20357_THM_CFG5_CHGT3_THR_DEF_SHIFT       3
#define MAX20357_THM_CFG5_CHGT3_THR_CC1_MASK        GENMASK(2, 0)
#define MAX20357_THM_CFG5_CHGT3_THR_CC1_SHIFT       0

/* --- ThmCfg6 (0x4E) --- */
#define MAX20357_REG_THM_CFG6                       0x4E

#define MAX20357_THM_CFG6_CHGT4_THR_DEF_MASK        GENMASK(5, 3)
#define MAX20357_THM_CFG6_CHGT4_THR_DEF_SHIFT       3
#define MAX20357_THM_CFG6_CHGT4_THR_CC1_MASK        GENMASK(2, 0)
#define MAX20357_THM_CFG6_CHGT4_THR_CC1_SHIFT       0

/* --- ThmCfg7 (0x4F) --- */
#define MAX20357_REG_THM_CFG7                       0x4F

#define MAX20357_THM_CFG7_JTA_EOC_SEL_MASK          GENMASK(7, 6)
#define MAX20357_THM_CFG7_JTA_EOC_SEL_SHIFT         6
#define MAX20357_THM_CFG7_THMPUSEL_BIT              BIT(3)
#define MAX20357_THM_CFG7_THMPUSEL_SHIFT            3
#define MAX20357_THM_CFG7_THMEN_MASK                GENMASK(2, 0)
#define MAX20357_THM_CFG7_THMEN_SHIFT               0

/* --- ChgCtr1 (0x50) --- */
#define MAX20357_REG_CHG_CTR1                       0x50

#define MAX20357_CHG_CTR1_I2C_CRF_ENA_BIT           BIT(7)
#define MAX20357_CHG_CTR1_I2C_CRF_ENA_SHIFT         7
#define MAX20357_CHG_CTR1_CHG_CC_TRK_BIT            BIT(6)
#define MAX20357_CHG_CTR1_CHG_CC_TRK_SHIFT          6
#define MAX20357_CHG_CTR1_CHGSTS_FCMRG_BIT          BIT(5)
#define MAX20357_CHG_CTR1_CHGSTS_FCMRG_SHIFT        5
#define MAX20357_CHG_CTR1_VLT_DNE_ENA_BIT           BIT(4)
#define MAX20357_CHG_CTR1_VLT_DNE_ENA_SHIFT         4
#define MAX20357_CHG_CTR1_LNG_XFER_OPT_BIT          BIT(2)
#define MAX20357_CHG_CTR1_LNG_XFER_OPT_SHIFT        2

/* --- ChgCtr2 (0x51) --- */
#define MAX20357_REG_CHG_CTR2                       0x51

#define MAX20357_CHG_CTR2_BATT_UVLO_ENA_BIT         BIT(7)
#define MAX20357_CHG_CTR2_BATT_UVLO_ENA_SHIFT       7
#define MAX20357_CHG_CTR2_BATT_PULLDOWN_BIT         BIT(6)
#define MAX20357_CHG_CTR2_BATT_PULLDOWN_SHIFT       6
#define MAX20357_CHG_CTR2_FRC_PCHG_BIT              BIT(5)
#define MAX20357_CHG_CTR2_FRC_PCHG_SHIFT            5
#define MAX20357_CHG_CTR2_VLT_CTR_PLC_MASK          GENMASK(1, 0)
#define MAX20357_CHG_CTR2_VLT_CTR_PLC_SHIFT         0

/* --- GPIOx (0x58/0x59/0x5A/0x5B) --- */
#define MAX20357_REG_GPIO1                          0x58
#define MAX20357_REG_GPIO2                          0x59
#define MAX20357_REG_GPIO3                          0x5A
#define MAX20357_REG_GPIO4                          0x5B

#define MAX20357_GPIO_CMPE_BIT                      BIT(4)
#define MAX20357_GPIO_CMPE_SHIFT                    4
#define MAX20357_GPIO_PLCCTR_BIT                    BIT(3)
#define MAX20357_GPIO_PLCCTR_SHIFT                  3
#define MAX20357_GPIO_ENRES_BIT                     BIT(2)
#define MAX20357_GPIO_ENRES_SHIFT                   2
#define MAX20357_GPIO_ENPUP_BIT                     BIT(1)
#define MAX20357_GPIO_ENPUP_SHIFT                   1
#define MAX20357_GPIO_DOUT_BIT                      BIT(0)
#define MAX20357_GPIO_DOUT_SHIFT                    0

/* --- GPIO_RDB1 (0x5C) --- */
#define MAX20357_REG_GPIO_RDB1                      0x5C

#define MAX20357_GPIO_RDB1_DAINP4_BIT               BIT(7)
#define MAX20357_GPIO_RDB1_DAINP4_SHIFT             7
#define MAX20357_GPIO_RDB1_DAINP3_BIT               BIT(6)
#define MAX20357_GPIO_RDB1_DAINP3_SHIFT             6
#define MAX20357_GPIO_RDB1_DAINP2_BIT               BIT(5)
#define MAX20357_GPIO_RDB1_DAINP2_SHIFT             5
#define MAX20357_GPIO_RDB1_DAINP1_BIT               BIT(4)
#define MAX20357_GPIO_RDB1_DAINP1_SHIFT             4
#define MAX20357_GPIO_RDB1_CMOS_EN_BIT              BIT(0)
#define MAX20357_GPIO_RDB1_CMOS_EN_SHIFT            0

/* --- GPIO_RDB2 (0x5D) --- */
#define MAX20357_REG_GPIO_RDB2                      0x5D

#define MAX20357_GPIO_RDB2_MST_GPIO4_BIT            BIT(3)
#define MAX20357_GPIO_RDB2_MST_GPIO4_SHIFT          3
#define MAX20357_GPIO_RDB2_MST_GPIO3_BIT            BIT(2)
#define MAX20357_GPIO_RDB2_MST_GPIO3_SHIFT          2
#define MAX20357_GPIO_RDB2_MST_GPIO2_BIT            BIT(1)
#define MAX20357_GPIO_RDB2_MST_GPIO2_SHIFT          1
#define MAX20357_GPIO_RDB2_MST_GPIO1_BIT            BIT(0)
#define MAX20357_GPIO_RDB2_MST_GPIO1_SHIFT          0

/* --- AVGVCELL_BYTE_1 (0x60) --- */
#define MAX20357_REG_AVGVCELL_BYTE_1                0x60

#define MAX20357_AVGVCELL_BYTE_1_MASK               GENMASK(7, 0)
#define MAX20357_AVGVCELL_BYTE_1_SHIFT              0

/* --- AVGVCELL_BYTE_0 (0x61) --- */
#define MAX20357_REG_AVGVCELL_BYTE_0                0x61

#define MAX20357_AVGVCELL_BYTE_0_MASK               GENMASK(7, 0)
#define MAX20357_AVGVCELL_BYTE_0_SHIFT              0

/* --- VCELL_BYTE_1 (0x62) --- */
#define MAX20357_REG_VCELL_BYTE_1                   0x62

#define MAX20357_VCELL_BYTE_1_MASK                  GENMASK(7, 0)
#define MAX20357_VCELL_BYTE_1_SHIFT                 0

/* --- VCELL_BYTE_0 (0x63) --- */
#define MAX20357_REG_VCELL_BYTE_0                   0x63

#define MAX20357_VCELL_BYTE_0_MASK                  GENMASK(7, 0)
#define MAX20357_VCELL_BYTE_0_SHIFT                 0

/* --- TTE_BYTE_1 (0x64) --- */
#define MAX20357_REG_TTE_BYTE_1                     0x64

#define MAX20357_TTE_BYTE_1_MASK                    GENMASK(7, 0)
#define MAX20357_TTE_BYTE_1_SHIFT                   0

/* --- TTE_BYTE_0 (0x65) --- */
#define MAX20357_REG_TTE_BYTE_0                     0x65

#define MAX20357_TTE_BYTE_0_MASK                    GENMASK(7, 0)
#define MAX20357_TTE_BYTE_0_SHIFT                   0

/* --- SOC_BYTE_1 (0x66) --- */
#define MAX20357_REG_SOC_BYTE_1                     0x66

#define MAX20357_SOC_BYTE_1_MASK                    GENMASK(7, 0)
#define MAX20357_SOC_BYTE_1_SHIFT                   0

/* --- SOC_BYTE_0 (0x67) --- */
#define MAX20357_REG_SOC_BYTE_0                     0x67

#define MAX20357_SOC_BYTE_0_MASK                    GENMASK(7, 0)
#define MAX20357_SOC_BYTE_0_SHIFT                   0

/* --- TTF_BYTE_1 (0x68) --- */
#define MAX20357_REG_TTF_BYTE_1                     0x68

#define MAX20357_TTF_BYTE_1_MASK                    GENMASK(7, 0)
#define MAX20357_TTF_BYTE_1_SHIFT                   0

/* --- TTF_BYTE_0 (0x69) --- */
#define MAX20357_REG_TTF_BYTE_0                     0x69

#define MAX20357_TTF_BYTE_0_MASK                    GENMASK(7, 0)
#define MAX20357_TTF_BYTE_0_SHIFT                   0

/* --- READY_REG (0x6A) --- */
#define MAX20357_REG_READY_REG                      0x6A

#define MAX20357_READY_DOP_RDY_SIG_BIT              BIT(7)
#define MAX20357_READY_DOP_RDY_SIG_SHIFT            7
#define MAX20357_READY_DOP_I2C_ENA_BIT              BIT(5)
#define MAX20357_READY_DOP_I2C_ENA_SHIFT            5
#define MAX20357_READY_TTF_RDY_BIT                  BIT(4)
#define MAX20357_READY_TTF_RDY_SHIFT                4
#define MAX20357_READY_AVGVCELL_RDY_BIT             BIT(3)
#define MAX20357_READY_AVGVCELL_RDY_SHIFT           3
#define MAX20357_READY_TTE_RDY_BIT                  BIT(2)
#define MAX20357_READY_TTE_RDY_SHIFT                2
#define MAX20357_READY_VCELL_RDY_BIT                BIT(1)
#define MAX20357_READY_VCELL_RDY_SHIFT              1
#define MAX20357_READY_SOC_RDY_BIT                  BIT(0)
#define MAX20357_READY_SOC_RDY_SHIFT                0

/* --- ADC_CTRL1 (0x70) --- */
#define MAX20357_REG_ADC_CTRL1                      0x70

#define MAX20357_ADC_CTRL1_ADCGNDTRH_MASK           GENMASK(6, 3)
#define MAX20357_ADC_CTRL1_ADCGNDTRH_SHIFT          3
#define MAX20357_ADC_CTRL1_RESDETRTY_MASK           GENMASK(2, 0)
#define MAX20357_ADC_CTRL1_RESDETRTY_SHIFT          0

/* --- ADC_CTRL2 (0x71) --- */
#define MAX20357_REG_ADC_CTRL2                      0x71

#define MAX20357_ADC_CTRL2_ADCRNG_MASK              GENMASK(5, 0)
#define MAX20357_ADC_CTRL2_ADCRNG_SHIFT             0

/* --- ADC_CTRL3 (0x72) --- */
#define MAX20357_REG_ADC_CTRL3                      0x72

#define MAX20357_ADC_CTRL3_ADCAVGNUM_MASK           GENMASK(2, 0)
#define MAX20357_ADC_CTRL3_ADCAVGNUM_SHIFT          0

/* --- ADC_CTRL4 (0x73) --- */
#define MAX20357_REG_ADC_CTRL4                      0x73

#define MAX20357_ADC_CTRL4_ADCNOISECTR_MASK         GENMASK(5, 0)
#define MAX20357_ADC_CTRL4_ADCNOISECTR_SHIFT        0

/* --- MOI_DET_REG1 (0x74) --- */
#define MAX20357_REG_MOI_DET_REG1                   0x74

#define MAX20357_MOI_DET_REG1_RACCDET_MIP_MASK      GENMASK(1, 0)
#define MAX20357_MOI_DET_REG1_RACCDET_MIP_SHIFT     0

/* --- MOI_DET_REG2 (0x75) --- */
#define MAX20357_REG_MOI_DET_REG2                   0x75

#define MAX20357_MOI_DET_REG2_RACCDETMTHR_MASK      GENMASK(7, 0)
#define MAX20357_MOI_DET_REG2_RACCDETMTHR_SHIFT     0

/* --- MOI_DET_REG3 (0x76) --- */
#define MAX20357_REG_MOI_DET_REG3                   0x76

#define MAX20357_MOI_DET_REG3_MOI_DET_AUT_BIT       BIT(6)
#define MAX20357_MOI_DET_REG3_MOI_DET_AUT_SHIFT     6
#define MAX20357_MOI_DET_REG3_MOI_MAN_PL_BIT        BIT(4)
#define MAX20357_MOI_DET_REG3_MOI_MAN_PL_SHIFT      4
#define MAX20357_MOI_DET_REG3_MOI_MAN_RTY_BIT       BIT(2)
#define MAX20357_MOI_DET_REG3_MOI_MAN_RTY_SHIFT     2
#define MAX20357_MOI_DET_REG3_MOI_AUT_RTY_BIT       BIT(0)
#define MAX20357_MOI_DET_REG3_MOI_AUT_RTY_SHIFT     0

/* --- IP_RES_REG (0x77) --- */
#define MAX20357_REG_IP_RES_REG                     0x77

#define MAX20357_IP_RES_REG_IP_RES_DET_MASK         GENMASK(1, 0)
#define MAX20357_IP_RES_REG_IP_RES_DET_SHIFT        0

/* --- ADC_VAL1 (0x78) --- */
#define MAX20357_REG_ADC_VAL1                       0x78

#define MAX20357_ADC_VAL1_ADC_AVG_MASK              GENMASK(7, 0)
#define MAX20357_ADC_VAL1_ADC_AVG_SHIFT             0

/* --- ADC_VAL2 (0x79) --- */
#define MAX20357_REG_ADC_VAL2                       0x79

#define MAX20357_ADC_VAL2_ADC_MAX_MASK              GENMASK(7, 0)
#define MAX20357_ADC_VAL2_ADC_MAX_SHIFT             0

/* --- ADC_VAL3 (0x7A) --- */
#define MAX20357_REG_ADC_VAL3                       0x7A

#define MAX20357_ADC_VAL3_ADC_MIN_MASK              GENMASK(7, 0)
#define MAX20357_ADC_VAL3_ADC_MIN_SHIFT             0

#define MAX20357_REG_MAX                            MAX20357_REG_ADC_VAL3

/* ---------------------------------------------------------- */
#define MAX2035X_REG_MAX(type) \
    ((type) == MAX20355 ? MAX20355_REG_MAX : MAX20357_REG_MAX)


/* ---------------------------------------------------------- */
/* ------------------------ FUELGAUGE ----------------------- */
/* ---------------------------------------------------------- */

#define MAX2035X_FG_REG_STATUS                      0x00

#define MAX2035X_FG_STATUS_POR                      BIT(1)
#define MAX2035X_FG_STATUS_POR_SHIFT                1

#define MAX2035X_FG_REG_REPCAP                      0x05
#define MAX2035X_FG_REG_REPSOC                      0x06
#define MAX2035X_FG_REG_VCELL                       0x09
#define MAX2035X_FG_REG_MIXCAP                      0x0F
#define MAX2035X_FG_REG_FULLCAPREP                  0x10
#define MAX2035X_FG_REG_TTE                         0x11
#define MAX2035X_FG_REG_QRTABLE00                   0x12
#define MAX2035X_FG_REG_FULLSOCTHR                  0x13
#define MAX2035X_FG_REG_CYCLES                      0x17
#define MAX2035X_FG_REG_DESIGNCAP                   0x18
#define MAX2035X_FG_REG_AVGVCELL                    0x19
#define MAX2035X_FG_REG_CONFIG                      0x1D
#define MAX2035X_FG_REG_ICHGTERM                    0x1E
#define MAX2035X_FG_REG_AVCAP                       0x1F
#define MAX2035X_FG_REG_TTF                         0x20
#define MAX2035X_FG_REG_QRTABLE10                   0x22
#define MAX2035X_FG_REG_FULLCAPNOM                  0x23
#define MAX2035X_FG_REG_LEARNCFG                    0x28
#define MAX2035X_FG_REG_RELAXCFG                    0x2A
#define MAX2035X_FG_REG_MISCCFG                     0x2B
#define MAX2035X_FG_REG_TGAIN                       0x2C
#define MAX2035X_FG_REG_TOFF                        0x2D
#define MAX2035X_FG_REG_QRTABLE20                   0x32
#define MAX2035X_FG_REG_RCOMP0                      0x38
#define MAX2035X_FG_REG_VEMPTY                      0x3A

#define MAX2035X_FG_REG_FSTAT                       0x3D

#define MAX2035X_FG_FSTAT_DNR                       BIT(0)
#define MAX2035X_FG_FSTAT_DNR_SHIFT                 0

#define MAX2035X_FG_REG_QRTABLE30                   0x42
#define MAX2035X_FG_REG_DQACC                       0x45
#define MAX2035X_FG_REG_DPACC                       0x46
#define MAX2035X_FG_REG_SOFT_WAKEUP                 0x60
#define MAX2035X_FG_REG_TABLE_UNLOCK1               0x62
#define MAX2035X_FG_REG_TABLE_UNLOCK2               0x63
#define MAX2035X_FG_REG_RCOMPSEG                    0xAF
#define MAX2035X_FG_REG_TEMPCO                      0xB8
#define MAX2035X_FG_REG_CURVE                       0xB9
#define MAX2035X_FG_REG_HIBCFG                      0xBA
#define MAX2035X_FG_REG_CONFIG2                     0xBB
#define MAX2035X_FG_REG_MODELCFG                    0xDB
#define MAX2035X_FG_REG_VFSOC                       0xFF

#endif /* __LINUX_MFD_MAX2035X_REGISTERS_H__ */
