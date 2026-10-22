/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * MAX2035x register definitions
 */

#ifndef __LINUX_MFD_MAX2035X_REGISTERS_H__
#define __LINUX_MFD_MAX2035X_REGISTERS_H__

/* --- REVISION_ID (0x00) --- */
#define MAX20355_REG_REVISION_ID                    0x00

/* --- STATUS0 (0x01) --- */
#define MAX20355_REG_STATUS0                        0x01

#define MAX20355_STATUS0_ITF_RDY_STS_BIT            BIT(7)
#define MAX20355_STATUS0_CH1_CON_STS_BIT            BIT(6)
#define MAX20355_STATUS0_CH2_CON_STS_BIT            BIT(5)
#define MAX20355_STATUS0_CH1_IDL_STS_BIT            BIT(4)
#define MAX20355_STATUS0_CH2_IDL_STS_BIT            BIT(3)
#define MAX20355_STATUS0_PLC2_MOI_DET_BIT           BIT(1)
#define MAX20355_STATUS0_PLC2_MOI_DET_SHIFT         1
#define MAX20355_STATUS0_PLC1_MOI_DET_BIT           BIT(0)

/* --- STATUS1 (0x02) --- */
#define MAX20355_REG_STATUS1                        0x02

/* --- STATUS2 (0x03) --- */
#define MAX20355_REG_STATUS2                        0x03

/* --- INT0 (0x05) --- */
#define MAX20355_REG_INT0                           0x05

#define MAX20355_INT0_ITF_RDY_STS_BIT               BIT(7)
#define MAX20355_INT0_CH1_CON_BIT                   BIT(6)
#define MAX20355_INT0_CH2_CON_BIT                   BIT(5)
#define MAX20355_INT0_CH1_IDL_BIT                   BIT(4)
#define MAX20355_INT0_CH2_IDL_BIT                   BIT(3)
#define MAX20355_INT0_MOI_DNE_BIT                   BIT(2)
#define MAX20355_INT0_PLC2_MOI_DET_BIT              BIT(1)
#define MAX20355_INT0_PLC1_MOI_DET_BIT              BIT(0)

#define MAX20355_INT1_SYS_ERR_BIT                   BIT(7)
#define MAX20355_INT1_BB_FAULT_BIT                  BIT(6)
#define MAX20355_INT1_THM_FLT_BIT                   BIT(5)
#define MAX20355_INT1_PLC_NEW_DAT_BIT               BIT(4)
#define MAX20355_INT1_PLC2_CMD_DNE_BIT              BIT(3)
#define MAX20355_INT1_PLC1_CMD_DNE_BIT              BIT(2)
#define MAX20355_INT1_PLC2_CMD_ERR_BIT              BIT(1)
#define MAX20355_INT1_PLC1_CMD_ERR_BIT              BIT(0)

#define MAX20355_INT2_MOI_DET_BIT                   BIT(3)
#define MAX20355_INT2_RES_DET_ABR_BIT               BIT(2)
#define MAX20355_INT2_RES_DET_OPN_BIT               BIT(1)
#define MAX20355_INT2_RES_DET_GND_BIT               BIT(0)

#define MAX20355_INT3_URT_TMO_FLT2_BIT              BIT(5)
#define MAX20355_INT3_URT_MODFAIL2_BIT              BIT(4)
#define MAX20355_INT3_URT_MODDONE2_BIT              BIT(3)
#define MAX20355_INT3_URT_TMO_FLT1_BIT              BIT(2)
#define MAX20355_INT3_URT_MODFAIL1_BIT              BIT(1)
#define MAX20355_INT3_URT_MODDONE1_BIT              BIT(0)

/* --- INTMASK0 (0x09) --- */
#define MAX20355_REG_INTMASK0                       0x09

/* --- SYSTEM_REG0 (0x1A) --- */
#define MAX20355_REG_SYSTEM_REG0                    0x1A

#define MAX20355_SYSTEM_OFF_CMD_INP_BIT             BIT(7)
#define MAX20355_SYSTEM_SOFT_RESET_BIT              BIT(6)

/* --- UART_CTR0 (0x20) --- */
#define MAX20355_REG_UART_CTR0                      0x20

#define MAX20355_UART_CTR_URT_AUTO_EN2_BIT          BIT(3)
#define MAX20355_UART_CTR_URT_AUTO_EN1_BIT          BIT(2)

/* --- UART_CTRx (0x21/0x22) --- */
#define MAX20355_REG_UART_CTR1                      0x21
#define MAX20355_REG_UART_CTR2                      0x22

#define MAX20355_UART_CTR_TMO_TMR_ENA_BIT           BIT(6)
#define MAX20355_UART_CTR_I2C_URT_MOD_BIT           BIT(5)
#define MAX20355_UART_CTR_I2C_URT_SWC_BIT           BIT(2)

/* --- PLC_CONFIG2 (0x33) --- */
#define MAX20355_REG_PLC_CONFIG2                    0x33

#define MAX20355_PLC_CFG2_PL2_CHN_ENA_BIT           BIT(7)
#define MAX20355_PLC_CFG2_PL1_CHN_ENA_BIT           BIT(6)
#define MAX20355_PLC_CFG2_PL2_RES_REQ_BIT           BIT(5)
#define MAX20355_PLC_CFG2_PL1_RES_REQ_BIT           BIT(4)

/* --- PLC_CONFIG5 (0x36) --- */
#define MAX20355_REG_PLC_CONFIG5                    0x36

#define MAX20355_PLC_CFG5_RAM_IS_FULL_BIT           BIT(6)

/* --- PLC_ARGx (0x38/0x3A) --- */
#define MAX20355_REG_PLC_ARG1                       0x38
#define MAX20355_REG_PLC_ARG2                       0x3A

/* --- PLC_CMDx (0x39/0x3B) --- */
#define MAX20355_REG_PLC_CMD1                       0x39
#define MAX20355_REG_PLC_CMD2                       0x3B

#define MAX20355_PLC_CMD_RUN_TRG_BIT                BIT(7)

/* --- PLC_FIFO (0x3D) --- */
#define MAX20355_REG_PLC_FIFO                       0x3D

#define MAX20355_PLC_FIFO_PL2_MASTER_BIT            BIT(3)
#define MAX20355_PLC_FIFO_PL2_SLAVE_BIT             BIT(2)
#define MAX20355_PLC_FIFO_PL1_MASTER_BIT            BIT(1)
#define MAX20355_PLC_FIFO_PL1_SLAVE_BIT             BIT(0)

/* --- BB_VOLT_DEF (0x41) --- */
#define MAX20355_REG_BB_VOLT_DEF                    0x41

#define MAX20355_BB_VOLT_DEF_MASK                   GENMASK(7, 0)

/* --- GPIOx (0x58/0x59/0x5A/0x5B) --- */
#define MAX20355_REG_GPIO1                          0x58

#define MAX20355_GPIO_PLCCTR_BIT                    BIT(3)

/* --- SOC_BYTE_1 (0x60) --- */
#define MAX20355_REG_SOC_BYTE_1                     0x60

/* --- SOC_BYTE_0 (0x61) --- */
#define MAX20355_REG_SOC_BYTE_0                     0x61

/* --- VCELL_BYTE_1 (0x62) --- */
#define MAX20355_REG_VCELL_BYTE_1                   0x62

/* --- VCELL_BYTE_0 (0x63) --- */
#define MAX20355_REG_VCELL_BYTE_0                   0x63

/* --- TTE_BYTE_1 (0x64) --- */
#define MAX20355_REG_TTE_BYTE_1                     0x64

/* --- TTE_BYTE_0 (0x65) --- */
#define MAX20355_REG_TTE_BYTE_0                     0x65

/* --- AVGVCELL_BYTE_1 (0x66) --- */
#define MAX20355_REG_AVGVCELL_BYTE_1                0x66

/* --- AVGVCELL_BYTE_0 (0x67) --- */
#define MAX20355_REG_AVGVCELL_BYTE_0                0x67

/* --- TTF_BYTE_1 (0x68) --- */
#define MAX20355_REG_TTF_BYTE_1                     0x68

/* --- TTF_BYTE_0 (0x69) --- */
#define MAX20355_REG_TTF_BYTE_0                     0x69

/* --- READY_REG (0x6A) --- */
#define MAX20355_REG_READY_REG                      0x6A

/* --- FG_RDY_5 (0x6F) --- */
#define MAX20355_REG_FG_RDY_5                       0x6F

#define MAX20355_FG_RDY_SLV2_CHG_DNE_BIT            BIT(7)
#define MAX20355_FG_RDY_SLV1_CHG_DNE_BIT            BIT(6)

/* --- ADC_VAL3 (0x7B) --- */
#define MAX20355_REG_ADC_VAL3                       0x7B

#define MAX20355_REG_MAX                            MAX20355_REG_ADC_VAL3

/* --- REVISION_ID (0x00) --- */
#define MAX20357_REG_REVISION_ID                    0x00

/* --- Status0 (0x01) --- */
#define MAX20357_REG_STATUS0                        0x01

#define MAX20357_STATUS0_CHN_CON_STS_BIT            BIT(7)
#define MAX20357_STATUS0_CHN_IDL_STS_BIT            BIT(5)

/* --- Status1 (0x02) --- */
#define MAX20357_REG_STATUS1                        0x02

#define MAX20357_STATUS1_CHGSTAT_MASK               GENMASK(3, 0)

/* --- Status2 (0x03) --- */
#define MAX20357_REG_STATUS2                        0x03

#define MAX20357_STATUS2_DEAD_FOUND_STS_BIT         BIT(0)

/* --- Status4 (0x05) --- */
#define MAX20357_REG_STATUS4                        0x05

/* --- Status5 (0x06) --- */
#define MAX20357_REG_STATUS5                        0x06

#define MAX20357_STATUS5_ITF_RDY_STS_BIT            BIT(7)
#define MAX20357_STATUS5_PLC_MOI_DET_BIT            BIT(4)
#define MAX20357_STATUS5_PLC_MOI_DET_SHIFT          4

/* --- Int0 (0x08) --- */
#define MAX20357_REG_INT0                           0x08

#define MAX20357_INT0_PLC_SUMACT_BIT                BIT(7)
#define MAX20357_INT0_PLC_SUMCURR_BIT               BIT(6)
#define MAX20357_INT0_CHG_THRM_REG_BIT              BIT(5)
#define MAX20357_INT0_CC1_TMO_BIT                   BIT(4)
#define MAX20357_INT0_CHGSTAT_BIT                   BIT(3)
#define MAX20357_INT0_SYSMINREG_BIT                 BIT(2)
#define MAX20357_INT0_CHG_RESTA_B_BIT               BIT(1)
#define MAX20357_INT0_THMSTAT_BIT                   BIT(0)

#define MAX20357_INT1_JEITA_IS_REG_BIT              BIT(7)
#define MAX20357_INT1_CHG_REV_BIT                   BIT(6)
#define MAX20357_INT1_CHG_VOLT_MODE_BIT             BIT(5)
#define MAX20357_INT1_CHG_VOLT_STP_BIT              BIT(4)
#define MAX20357_INT1_CHG_GMD_BIT                   BIT(3)
#define MAX20357_INT1_LDO_GMD_BIT                   BIT(2)
#define MAX20357_INT1_PLCOk_BIT                     BIT(1)
#define MAX20357_INT1_SYSREV_BIT                    BIT(0)

#define MAX20357_INT2_CHN_CON_BIT                   BIT(7)
#define MAX20357_INT2_CHN_WTY_BIT                   BIT(6)
#define MAX20357_INT2_CHN_IDL_BIT                   BIT(5)
#define MAX20357_INT2_SRT_XFER_RISE_BIT             BIT(4)
#define MAX20357_INT2_SRT_XFER_FALL_BIT             BIT(3)
#define MAX20357_INT2_PLC_NEW_DAT_BIT               BIT(2)
#define MAX20357_INT2_PLC_CMD_DNE_BIT               BIT(1)
#define MAX20357_INT2_PLC_CMD_ERR_BIT               BIT(0)

#define MAX20357_INT3_LNG_XFER_BIT                  BIT(7)
#define MAX20357_INT3_BATUVLOB_BIT                  BIT(6)
#define MAX20357_INT3_MOI_DNE_BIT                   BIT(5)
#define MAX20357_INT3_PLC_MOI_DET_BIT               BIT(4)
#define MAX20357_INT3_MOI_DET_BIT                   BIT(3)
#define MAX20357_INT3_RES_DET_ABR_BIT               BIT(2)
#define MAX20357_INT3_RES_DET_OPN_BIT               BIT(1)
#define MAX20357_INT3_RES_DET_GND_BIT               BIT(0)

#define MAX20357_INT4_URT_TMO_FLT_BIT               BIT(7)
#define MAX20357_INT4_URT_MODFAIL_BIT               BIT(6)
#define MAX20357_INT4_URT_MODDONE_BIT               BIT(5)
#define MAX20357_INT4_URT_SWC_OPN_BIT               BIT(4)
#define MAX20357_INT4_DEAD_FOUND_BIT                BIT(3)
#define MAX20357_INT4_SWC_OFF_MOD_BIT               BIT(1)
#define MAX20357_INT4_CHG_PRQ_INP_BIT               BIT(0)

#define MAX20357_INT5_ITF_RDY_STS_BIT               BIT(7)
#define MAX20357_INT5_WD_ITR_CLR_BIT                BIT(2)

/* --- IntMask0 (0x0E) --- */
#define MAX20357_REG_INTMASK0                       0x0E

/* --- SYSTEM_REG0 (0x1A) --- */
#define MAX20357_REG_SYSTEM_REG0                    0x1A

#define MAX20357_SYSTEM_OFF_CMD_INP_BIT             BIT(7)
#define MAX20357_SYSTEM_SOFT_RESET_BIT              BIT(6)
#define MAX20357_SYSTEM_HARD_RESET_BIT              BIT(5)
#define MAX20357_SYSTEM_SEAL_I2C_CMD_BIT            BIT(3)

/* --- UART_Ctr0 (0x20) --- */
#define MAX20357_REG_UART_CTR0                      0x20

#define MAX20357_UART_CTR0_URT_AUTO_EN_BIT          BIT(2)

/* --- UART_Ctr1 (0x21) --- */
#define MAX20357_REG_UART_CTR1                      0x21

#define MAX20357_UART_CTR1_TMO_TMR_ENA_BIT          BIT(6)
#define MAX20357_UART_CTR1_I2C_URT_MOD_BIT          BIT(5)
#define MAX20357_UART_CTR1_I2C_URT_ENA_BIT          BIT(4)
#define MAX20357_UART_CTR1_I2C_URT_SWC_BIT          BIT(2)
#define MAX20357_UART_CTR1_I2C_TX_SWC_BIT           BIT(1)
#define MAX20357_UART_CTR1_I2C_RX_SWC_BIT           BIT(0)

/* --- PLC_CONFIG4 (0x35) --- */
#define MAX20357_REG_PLC_CONFIG4                    0x35

#define MAX20357_PLC_CFG4_PLC_FSM_ENA_BIT           BIT(7)
#define MAX20357_PLC_CFG4_RAM_IS_FULL_BIT           BIT(6)
#define MAX20357_PLC_CFG4_FIFO_MASTER_BIT           BIT(5)
#define MAX20357_PLC_CFG4_FIFO_SLAVE_BIT            BIT(4)
#define MAX20357_PLC_CFG4_PLC_RES_REQ_BIT           BIT(1)

/* --- PLC_CONFIG5 (0x36) --- */
#define MAX20357_REG_PLC_CONFIG5                    0x36

#define MAX20357_PLC_CFG5_NO_UART_MDE_BIT           BIT(5)

/* --- PLC_ARG (0x37) --- */
#define MAX20357_REG_PLC_ARG                        0x37

/* --- PLC_CMD (0x38) --- */
#define MAX20357_REG_PLC_CMD                        0x38

#define MAX20357_PLC_CMD_RUN_TRG_BIT                BIT(7)

/* --- ChgCur0 (0x41) --- */
#define MAX20357_REG_CHG_CUR0                       0x41

#define MAX20357_CHG_CUR0_CC1IFCHG_MASK             GENMASK(6, 0)

/* --- ChgCntl0 (0x43) --- */
#define MAX20357_REG_CHG_CNTL0                      0x43

#define MAX20357_CHG_CNTL0_CHG_EN_BIT               BIT(7)
#define MAX20357_CHG_CNTL0_CHG_AUTOSTOP_BIT         BIT(6)
#define MAX20357_CHG_CNTL0_CHG_AUTORESTA_BIT        BIT(5)
#define MAX20357_CHG_CNTL0_CC1_ENABLE_BIT           BIT(0)

/* --- ChgCntl1 (0x44) --- */
#define MAX20357_REG_CHG_CNTL1                      0x44

#define MAX20357_CHG_CNTL1_BATREG_MASK              GENMASK(5, 0)

/* --- ThmCfg7 (0x4F) --- */
#define MAX20357_REG_THM_CFG7                       0x4F

#define MAX20357_THM_CFG7_THMEN_MASK                GENMASK(2, 0)

/* --- ChgCtr1 (0x50) --- */
#define MAX20357_REG_CHG_CTR1                       0x50

#define MAX20357_CHG_CTR1_CHG_CC_TRK_BIT            BIT(6)

/* --- GPIOx (0x58/0x59/0x5A/0x5B) --- */
#define MAX20357_REG_GPIO1                          0x58

#define MAX20357_GPIO_PLCCTR_BIT                    BIT(3)

/* --- AVGVCELL_BYTE_1 (0x60) --- */
#define MAX20357_REG_AVGVCELL_BYTE_1                0x60

/* --- AVGVCELL_BYTE_0 (0x61) --- */
#define MAX20357_REG_AVGVCELL_BYTE_0                0x61

/* --- VCELL_BYTE_1 (0x62) --- */
#define MAX20357_REG_VCELL_BYTE_1                   0x62

/* --- VCELL_BYTE_0 (0x63) --- */
#define MAX20357_REG_VCELL_BYTE_0                   0x63

/* --- TTE_BYTE_1 (0x64) --- */
#define MAX20357_REG_TTE_BYTE_1                     0x64

/* --- TTE_BYTE_0 (0x65) --- */
#define MAX20357_REG_TTE_BYTE_0                     0x65

/* --- SOC_BYTE_1 (0x66) --- */
#define MAX20357_REG_SOC_BYTE_1                     0x66

/* --- SOC_BYTE_0 (0x67) --- */
#define MAX20357_REG_SOC_BYTE_0                     0x67

/* --- TTF_BYTE_1 (0x68) --- */
#define MAX20357_REG_TTF_BYTE_1                     0x68

/* --- TTF_BYTE_0 (0x69) --- */
#define MAX20357_REG_TTF_BYTE_0                     0x69

/* --- READY_REG (0x6A) --- */
#define MAX20357_REG_READY_REG                      0x6A

/* --- ADC_VAL3 (0x7A) --- */
#define MAX20357_REG_ADC_VAL3                       0x7A

#define MAX20357_REG_MAX                            MAX20357_REG_ADC_VAL3

#define MAX2035X_FG_REG_STATUS                      0x00

#define MAX2035X_FG_STATUS_POR                      BIT(1)

#define MAX2035X_FG_REG_REPCAP                      0x05
#define MAX2035X_FG_REG_REPSOC                      0x06
#define MAX2035X_FG_REG_MIXCAP                      0x0F
#define MAX2035X_FG_REG_FULLCAPREP                  0x10
#define MAX2035X_FG_REG_TTE                         0x11
#define MAX2035X_FG_REG_QRTABLE00                   0x12
#define MAX2035X_FG_REG_FULLSOCTHR                  0x13
#define MAX2035X_FG_REG_CYCLES                      0x17
#define MAX2035X_FG_REG_DESIGNCAP                   0x18
#define MAX2035X_FG_REG_CONFIG                      0x1D
#define MAX2035X_FG_REG_ICHGTERM                    0x1E
#define MAX2035X_FG_REG_AVCAP                       0x1F
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

#define MAX2035X_FG_REG_QRTABLE30                   0x42
#define MAX2035X_FG_REG_DQACC                       0x45
#define MAX2035X_FG_REG_DPACC                       0x46
#define MAX2035X_FG_REG_SOFT_WAKEUP                 0x60
#define MAX2035X_FG_REG_TABLE_UNLOCK1               0x62
#define MAX2035X_FG_REG_TABLE_UNLOCK2               0x63
#define MAX2035X_FG_REG_RCOMPSEG                    0xAF
#define MAX2035X_FG_REG_TEMPCO                      0x39
#define MAX2035X_FG_REG_CURVE                       0xB9
#define MAX2035X_FG_REG_HIBCFG                      0xBA
#define MAX2035X_FG_REG_CONFIG2                     0xBB
#define MAX2035X_FG_REG_VFSOC                       0xFF

#endif /* __LINUX_MFD_MAX2035X_REGISTERS_H__ */
