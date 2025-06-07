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

#ifndef __MAX77840_MFD_H__
#define __MAX77840_MFD_H__

#define __TEST_DEVICE_NODE__

#include <linux/bits.h>
#include <linux/bitops.h>

/* MAX77840 Top Devices */
#define MAX77840_NAME			"max77840"

/* MAX77840 PMIC Devices */
#define MAX77840_REGULATOR_NAME         "max77840-regulator"
#define MAX77840_CHARGER_NAME           "max77840-charger"
#define MAX77840_CHARGER_DETECT_NAME    "max77840-charger-detect"
#define MAX77840_FUELGAUGE_NAME         "max77840-fuelgauge"

/* Register map */
/* Interrupt register & mask bit */
#define REG_INTSRC             0x22
#define REG_INTSRCMASK         0x23
#define BIT_CHGR_INT           BIT(0)
#define BIT_SYS_INT            BIT(1)
#define BIT_FG_INT             BIT(2)
#define BIT_CHGDET_INT         BIT(3)
#define BIT_B2SOVRC_INT        BIT(5)

#define REG_SYSINTSRC          0x24
#define REG_SYSINTMASK         0x26
#define BIT_T120C_INT          BIT(0)
#define BIT_T140C_INT          BIT(1)
#define BIT_LOWSYS_INT         BIT(3)
#define BIT_SYSUVLO_INT        BIT(4)
#define BIT_SYSOVLO_INT        BIT(5)
#define BIT_TSHDN_INT          BIT(6)

#define REG_CHGDET_INT         0x01
#define REG_CHGDET_INT_MASK    0x03
#define BIT_CHGDET_CHGTYPE_I   BIT(0)
#define BIT_CHGDET_CHGDETRUN_I BIT(1)
#define BIT_CHGDET_DCDTMR_I    BIT(2)
#define BIT_CHGDET_DXOVP_I     BIT(3)
#define BIT_CHGDET_VDNMON_I    BIT(4)

#define REG_CHARGER_INT        0xB0
#define REG_CHARGER_INT_MASK   0xB1
#define BIT_CHG_BYP_I          BIT(0)
#define BIT_CHG_BAT2SOC_I      BIT(1)
#define BIT_CHG_BATP_I         BIT(2)
#define BIT_CHG_BAT_I          BIT(3)
#define BIT_CHG_CHG_I          BIT(4)
#define BIT_CHG_TOPOFF_I       BIT(5)
#define BIT_CHG_CHGIN_I        BIT(6)
#define BIT_CHG_AICL_I         BIT(7)

/* Chip Interrupts */
enum {
	MAX77840_CHGR_INT = 0,
	MAX77840_SYS_INT,
	MAX77840_FG_INT,
	MAX77840_CHGDET_INT,
	MAX77840_B2SOVRC_INT,

	MAX77840_SYS_IRQ_START,
	MAX77840_SYS_IRQ_T120C = MAX77840_SYS_IRQ_START,
	MAX77840_SYS_IRQ_T140C,
	MAX77840_SYS_IRQ_LOWSYS,
	MAX77840_SYS_IRQ_UVLO,
	MAX77840_SYS_IRQ_OVLO,
	MAX77840_SYS_IRQ_TSHDN,

	MAX77840_CHGDET_IRQ_START,
	MAX77840_CHGDET_IRQ_CHGTYPE_I = MAX77840_CHGDET_IRQ_START,
	MAX77840_CHGDET_IRQ_CHGDETRUN_I,
	MAX77840_CHGDET_IRQ_DCDTMR_I,
	MAX77840_CHGDET_IRQ_DXOVP_I,
	MAX77840_CHGDET_IRQ_VDCNMON_I,

	MAX77840_CHG_IRQ_START,
	MAX77840_CHG_IRQ_BYP_I = MAX77840_CHG_IRQ_START,
	MAX77840_CHG_IRQ_BAT2SOC_I,
	MAX77840_CHG_IRQ_BATP_I,
	MAX77840_CHG_IRQ_BAT_I,
	MAX77840_CHG_IRQ_CHG_I,
	MAX77840_CHG_IRQ_WCIN_I,
	MAX77840_CHG_IRQ_CHGIN_I,
	MAX77840_CHG_IRQ_AICL_I,

	MAX77840_NUM_OF_INTS,
};

enum{
	SYS_IRQ_T120C = 0,
	SYS_IRQ_T140C,
	SYS_IRQ_LOWSYS,
	SYS_IRQ_UVLO,
	SYS_IRQ_OVLO,
	SYS_IRQ_TSHDN,

	CHGDET_IRQ_CHGTYPE_I = 0,
	CHGDET_IRQ_CHGDETRUN_I,
	CHGDET_IRQ_DCDTMR_I,
	CHGDET_IRQ_DxOVP_I,
	CHGDET_IRQ_VDCNMON_I,

	CHG_IRQ_BYP_I = 0,
	CHG_IRQ_BAT2SOC_I,
	CHG_IRQ_BATP_I,
	CHG_IRQ_BAT_I,
	CHG_IRQ_CHG_I,
	CHG_IRQ_TOPOFF_I,
	CHG_IRQ_CHGIN_I,
	CHG_IRQ_AICL_I,

	FG_IRQ_ALERT = 0,

	B2SOVRC_IRQ_ALERT = 0,

};

/*******************************************************************************
 * Useful Macros
 ******************************************************************************/
#undef  BIT_RSVD
#define BIT_RSVD  0

#undef  BITS
#define BITS(_end, _start) GENMASK((_end), (_start))

/*******************************************************************************
 * Sub Modules Support
 ******************************************************************************/
enum {
	MAX77840_DEV_REGULATOR = 0,
	MAX77840_DEV_CHARGER,
	MAX77840_DEV_CHARGER_DETECT,
	MAX77840_DEV_FUELGAUGE,
	MAX77840_DEV_NUM_OF_DEVICES,
};

struct max77840_dev {
	void                        *pdata;
	struct mutex                lock; /* mutext */
	struct device               *dev;

	int                         irq;
	int                         irq_gpio;

	struct regmap_irq_chip_data *irqc_intsrc;
	struct regmap_irq_chip_data *irqc_sys;
	struct regmap_irq_chip_data *irqc_chg;
	struct regmap_irq_chip_data *irqc_chgdet;

	struct i2c_client           *pmic;       /* 0xCC , CLOGIC/SAFELDOS */
	struct i2c_client           *chg;        /* 0xD2, CHARGER */
	struct i2c_client           *chg_det;    /* 0x4A, CHARGER DETECT */
	struct i2c_client           *fuel;       /* 0x6C, FUEL GAUGE */

	struct regmap               *regmap_pmic;    /* CLOGIC/SAFELDOS */
	struct regmap               *regmap_chg;     /* CHARGER */
	struct regmap               *regmap_chg_det; /* CHARGER DETECT */
	struct regmap               *regmap_fuel;    /* FUEL GAUGE */
};

/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max77840_read(struct regmap *regmap, u8 addr, u8 *val);
int max77840_write(struct regmap *regmap, u8 addr, u8 val);
int max77840_fg_read(struct regmap *regmap, u8 addr, u16 *val);
int max77840_fg_write(struct regmap *regmap, u8 addr, u16 val);
int max77840_bulk_read(struct regmap *regmap, u8 addr, u8 *dst, u16 len);
int max77840_bulk_write(struct regmap *regmap, u8 addr, const u8 *src, u16 len);

/*******************************************************************************
 * Interrupt
 ******************************************************************************/
int max77840_irq_init(struct max77840_dev *max77840);
void max77840_irq_exit(struct max77840_dev *max77840);
int max77840_irq_resume(struct max77840_dev *max77840);

int max77840_map_irq(struct max77840_dev *max77840, int irq);

#endif /* !__MAX77840_MFD_H__ */
