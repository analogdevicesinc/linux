/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This header provides macros for MAXIM MAX77840 device bindings.
 *
 * Copyright (C) 2020 Maxim Integrated. All rights reserved.
 *
 * Author:
 *	Maxim LDD <opensource@maximintegrated.com>
 */

#ifndef _DT_BINDINGS_MFD_MAX77840_H
#define _DT_BINDINGS_MFD_MAX77840_H

/* MAX77840 TOP */
#define TOP_INTSRCMASK       0x23
#define TOP_SYSINTMASK       0x26
#define TOP_SAFEOUTCTRL      0xc6

/* MAX77840 PMIC Charger */
#define PMIC_CHG_INT_MASK    0xb1
#define PMIC_CHG_CNFG_00     0xb7
#define PMIC_CHG_CNFG_01     0xb8
#define PMIC_CHG_CNFG_02     0xb9
#define PMIC_CHG_CNFG_03     0xba
#define PMIC_CHG_CNFG_04     0xbb
#define PMIC_CHG_CNFG_05     0xbc
#define PMIC_CHG_CNFG_06     0xbd
#define PMIC_CHG_CNFG_07     0xbe
#define PMIC_CHG_CNFG_08     0xbf
#define PMIC_CHG_CNFG_09     0xc0
#define PMIC_CHG_CNFG_10     0xc1
#define PMIC_CHG_CNFG_11     0xc2
#define PMIC_CHG_CNFG_12     0xc3

/* MAX77840 FG */
#define FG_STATUS            0x00
#define FG_VALRT_Th          0x01
#define FG_TALRT_Th          0x02
#define FG_SALRT_Th          0x03
#define FG_AtRate            0x04

#define FG_QRTable00         0x12
#define FG_FullSOCThr        0x13
#define FG_RFAST             0x15

#define FG_DesignCap         0x18

#define FG_CONFIG            0x1d
#define FG_RemCapAv          0x1e

#define FG_QRTable10         0x22

#define FG_AIN               0x27
#define FG_LearnCFG          0x28
#define FG_FilterCFG         0x29
#define FG_RelaxCFG          0x2a
#define FG_MiscCFG           0x2b
#define FG_TGAIN             0x2c
#define FG_TOFF              0x2d
#define FG_CGAIN             0x2e
#define FG_COFF              0x2f
#define FG_QRTable20         0x32

#define FG_FullCapRep        0x35
#define FG_Iave_empty        0x36
#define FG_RCOMP0            0x38
#define FG_TempCo            0x39
#define FG_V_EMPTY           0x3a
#define FG_TaskPeriod        0x3c
#define FG_SHDNTIMER         0x3f
#define FG_DischargeTH       0x40
#define FG_QRTable30         0x42
#define FG_dQ_acc            0x45
#define FG_dP_acc            0x46
#define FG_ConvgCFG          0x49

#define FG_CMD               0x60
#define FG_TUL1              0x62
#define FG_TUL2              0x63
#define FG_OTUL1             0x6b
#define FG_OTUL2             0x6c

#define FG_TALRT_Th2         0xb2
#define FG_CV_MixCap         0xb6
#define FG_CURVE             0xb9
#define FG_HibCFG            0xba
#define FG_Config2           0xbb
#define FG_ChargeSgtate0     0xd1
#define FG_ChargeSgtate1     0xd2
#define FG_ChargeSgtate2     0xd3
#define FG_ChargeSgtate3     0xd4
#define FG_ChargeSgtate4     0xd5
#define FG_ChargeSgtate5     0xd6
#define FG_ChargeSgtate6     0xd7
#define FG_ChargeSgtate7     0xd8
#define FG_JEITA_Volt        0xd9
#define FG_JEITA_Curr        0xda
#define FG_SmartChgCfg       0xdb

#endif //_DT_BINDINGS_MFD_MAX77840_H
