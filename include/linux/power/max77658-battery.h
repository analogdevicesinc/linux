/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __MAX77658_BATTERY_H_
#define __MAX77658_BATTERY_H_

#define MAX77658_IALRTTH_RESOLUTION	8567
#define MAX77658_FG_DELAY		1000
#define MAX77658_BATTERY_FULL		100
#define MAX77658_BATTERY_LOW		40
#define MAX77658_BATTERY_CRITICAL	10

/* Register map */
/* Status and Configuration Registers */
#define REG_STATUS			0x00
#define REG_VALRTTH			0x01
#define REG_TALRTTH			0x02
#define REG_SALRTTH			0x03
#define REG_CONFIG			0x1D
#define REG_DEVNAME			0x21
#define REG_VEMPTY			0x3A
#define REG_AVGPOWER			0xB3
#define REG_IALRTTH			0xB4
#define REG_CONFIG2			0xBB

/* Measurement Registers */
#define REG_TEMP			0x08
#define REG_VCELL			0x09
#define REG_CURRENT			0x0A
#define REG_AVGCURRENT			0x0B
#define REG_AVGVCELL			0x19
#define REG_MAXMINTEMP			0x1A
#define REG_MAXMINVOLT			0x1B
#define REG_MAXMINCURR			0x1C

/* Output Registers */
#define REG_REPSOC			0x06
#define REG_TTE				0x11
#define REG_TTF				0x20

/* Bit Fields */
/* Status Register */
#define BIT_STATUS_BR			BIT(15)
#define BIT_STATUS_SMX			BIT(14)
#define BIT_STATUS_TMX			BIT(13)
#define BIT_STATUS_VMX			BIT(12)
#define BIT_STATUS_BI			BIT(11)
#define BIT_STATUS_SMN			BIT(10)
#define BIT_STATUS_TMN			BIT(9)
#define BIT_STATUS_VMN			BIT(8)
#define BIT_STATUS_dSOCi		BIT(7)
#define BIT_STATUS_POR			BIT(2)

/* Config Register */
#define	BIT_CONFIG_AEN			BIT(2)

/* SAlrtTh Register */
#define BIT_SALRTTH_SMAX		BITS(7, 0)
#define BIT_SALRTTH_SMIN		BITS(15, 8)

/* TAlrtTh Register */
#define BIT_TALRTTH_TMAX		BITS(7, 0)
#define BIT_TALRTTH_TMIN		BITS(15, 8)

/* MaxMinTemp Register */
#define BIT_MAXMINTEMP_MAX		BITS(7, 0)
#define BIT_MAXMINTEMP_MIN		BITS(15, 8)

struct max77658_fg_dev {
	struct device *dev;
	struct max77658_dev *max77658;
	struct regmap *regmap;

	struct delayed_work work;
	struct power_supply	*battery;
	struct power_supply_desc psy_batt_d;

	/* mutex */
	struct mutex lock;

	/* battery voltage */
	int vcell;
	/* State Of Charge */
	int soc;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;

	int lasttime_vcell;
	int lasttime_soc;

	int volt_min; /* in mV */
	int volt_max; /* in mV */
	int temp_min; /* in DegreC */
	int temp_max; /* in DegreeC */
	int soc_max;  /* in percent */
	int soc_min;  /* in percent */
	int curr_max; /* in mA */
	int curr_min; /* in mA */
};

#endif // __MAX77658_BATTERY_H_
