/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Maxim Integrated Products, Inc.
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Author : Analog Devices <joan.na@analog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77840_CHARGER_H__
#define __MAX77840_CHARGER_H__

#define __TEST_DEVICE_NODE__

/* Register map */
#define REG_CHG_INT                0xB0
#define REG_CHG_INT_MASK           0xB1
#define BIT_BYP                    BIT(0)
#define BIT_BAT2SOC                BIT(1)
#define BIT_BATP                   BIT(2)
#define BIT_BAT                    BIT(3)
#define BIT_CHG                    BIT(4)
#define BIT_TOPOFF                 BIT(5)
#define BIT_CHGIN                  BIT(6)
#define BIT_AICL                   BIT(7)

#define REG_CHG_INT_OK             0xB2
#define BIT_BYP_OK                 BIT(0)
#define BIT_BAT2SOC_OK             BIT(1)
#define BIT_BATP_OK                BIT(2)
#define BIT_BAT_OK                 BIT(3)
#define BIT_CHG_OK                 BIT(4)
#define BIT_TOPOFF_OK              BIT(5)
#define BIT_CHGIN_OK               BIT(6)
#define BIT_AICL_CHGINI_OK         BIT(7)

#define REG_CHG_DTLS_00            0xB3
#define BIT_BATP_DTLS              BIT(0)
#define BIT_OVPDRV_DTLS            BIT(1)
#define BIT_VBUSDET_DTLS           BIT(2)
#define BIT_CHGIN_DTLS             BITS(6, 5)

#define REG_CHG_DTLS_01            0xB4
#define BIT_CHG_DTLS               BITS(3, 0)
#define BIT_BAT_DTLS               BITS(6, 4)
#define BIT_TREG                   BIT(7)

#define REG_CHG_DTLS_02            0xB5
#define BIT_BYP_DTLS               BITS(2, 0)
#define BIT_OTGILIM                BIT(0)
#define BIT_BSTILIM                BIT(1)
#define BIT_BCKNEGILIM             BIT(2)
#define BIT_AICL_DTLS              BIT(3)
#define BIT_CHGINI_DTLS            BIT(4)

#define REG_CHG_CNFG_00            0xB7
#define BIT_MODE                   BITS(3, 0)
#define BIT_MODE_CHARGER           BIT(0)
#define BIT_MODE_OTG               BIT(1)
#define BIT_MODE_BUCK              BIT(2)
#define BIT_MODE_BOOST             BIT(3)
#define BIT_WDTEN                  BIT(4)
#define BIT_SPREAD                 BIT(5)
#define BIT_DISIBS                 BIT(6)
#define BIT_DIS_CD_CTRL            BIT(7)

#define REG_CHG_CNFG_01            0xB8
#define BIT_FCHGTIME               BITS(2, 0)
#define BIT_FSW                    BIT(3)
#define BIT_CHG_RSTRT              BITS(5, 4)
#define BIT_LSEL                   BIT(6)
#define BIT_PQEN                   BIT(7)

#define REG_CHG_CNFG_02            0xB9
#define BIT_CHG_CC                 BITS(5, 0)
#define BIT_OTG_ILIM               BITS(7, 6)

#define REG_CHG_CNFG_03            0xBA
#define BIT_TO_ITH                 BITS(2, 0)
#define BIT_TO_TIME                BITS(5, 3)
#define BIT_ILIM                   BITS(7, 6)

#define REG_CHG_CNFG_04            0xBB
#define BIT_CHG_CV_PRM             BITS(5, 0)
#define BIT_MINVSYS                BITS(7, 6)

#define REG_CHG_CNFG_06            0xBD
#define BIT_WDTCLR                 BITS(1, 0)
#define BIT_CHGPROT                BITS(3, 2)
#define BIT_MAXOTG_EN              BIT(4)
#define BIT_OTG_DC                 BIT(5)
#define BIT_LEDEN                  BIT(7)

#define REG_CHG_CNFG_07            0xBE
#define BIT_DIS_QBATOFF            BIT(2)
#define BIT_REGTEMP                BITS(6, 3)
#define BIT_WD_QBATOFF             BIT(7)

#define REG_CHG_CNFG_09            0xC0
#define BIT_CHGIN_ILIM             BITS(6, 0)
#define BIT_OVPDRV_CTL             BIT(7)

#define REG_CHG_CNFG_10            0xC1
#define BIT_DISSKIP                BIT(0)
#define BIT_TODEB_EN               BIT(1)
#define BIT_TODEN                  BITS(3, 2)

#define REG_CHG_CNFG_11            0xC2
#define BIT_VBYPSET                BITS(6, 0)

#define REG_CHG_CNFG_12            0xC3
#define BIT_B2SOVRC                BITS(3, 0)
#define BIT_VCHGIN_REG             BITS(5, 4)
#define BIT_CHGINSEL               BIT(6)
#define BIT_CHG_LPM                BIT(7)

enum {
	CHGIN_DTLS_UVLO,
	CHGIN_DTLS_INVALID_01,
	CHGIN_DTLS_OVLO,
	CHGIN_DTLS_VALID,
};

enum {
	CHG_DTLS_PREQUAL,
	CHG_DTLS_FASTCHARGE_CC,
	CHG_DTLS_FASTCHARGE_CV,
	CHG_DTLS_TOPOFF,
	CHG_DTLS_DONE,
	CHG_DTLS_RESEVRED_05,
	CHG_DTLS_OFF_TIMER_FAULT,
	CHG_DTLS_OFF_SUSPEND,
	CHG_DTLS_OFF_INPUT_INVALID,
	CHG_DTLS_RESERVED_09,
	CHG_DTLS_OFF_JUCTION_TEMP,
	CHG_DTLS_OFF_WDT_EXPIRED,
};

enum {
	BAT_DTLS_NO_BATTERY,
	BAT_DTLS_RESERVED_01,
	BAT_DTLS_TIMER_FAULT,
	BAT_DTLS_OKAY,
	BAT_DTLS_OKAY_LOW,
	BAT_DTLS_OVERVOLTAGE,
	BAT_DTLS_OVERCURRENT,
	BAT_DTLS_RESERVED_07,
};

enum {
	CHG_INT_BYP_I,
	CHG_INT_BAT2SOC_I,
	CHG_INT_BATP_I,
	CHG_INT_BAT_I,
	CHG_INT_CHG_I,
	CHG_INT_TOPOFF_I,
	CHG_INT_CHGIN_I,
	CHG_INT_AICL_CHGINI_I,
};

struct max77840_charger_platform_data {
	int fast_charge_timer;
	int fast_charge_current;
	int topoff_current;
	int topoff_timer;
	int restart_threshold;
	int termination_voltage;
	int input_current_limit;
};

#endif /* !__MAX77840_CHARGER_H__ */
