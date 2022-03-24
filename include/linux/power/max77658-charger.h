/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __MAX77658_CHARGER_H_
#define __MAX77658_CHARGER_H_

#define IRQ_WORK_DELAY		0

#define STATUS_MAP(_chg_dtls, _health, _status, _charge_type)		\
	[CHG_DTLS_##_chg_dtls] = {					\
		.health = POWER_SUPPLY_HEALTH_##_health,		\
		.status = POWER_SUPPLY_STATUS_##_status,		\
		.charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type,	\
	}

/* Register map */
/* Charger registers */
#define REG_STAT_CHG_A			0x02
#define REG_STAT_CHG_B			0x03
#define REG_CNFG_CHG_A			0x20
#define REG_CNFG_CHG_B			0x21
#define REG_CNFG_CHG_C			0x22
#define REG_CNFG_CHG_D			0x23
#define REG_CNFG_CHG_E			0x24
#define REG_CNFG_CHG_F			0x25
#define REG_CNFG_CHG_G			0x26
#define REG_CNFG_CHG_H			0x27
#define REG_CNFG_CHG_I			0x28

/* Bit Fields */
/* Status Charger A */
#define BIT_STAT_A_VCHGIN_MIN_STAT	BIT(6)
#define BIT_STAT_A_ICHGIN_LIM_STAT	BIT(5)
#define BIT_STAT_A_VSYSY_MIN_STAT	BIT(5)
#define BIT_STAT_A_TJ_REG_STAT		BIT(3)
#define BITS_STAT_A_THM_DTLS		BITS(2, 0)

/* Status Charger B */
#define BITS_STAT_B_CHG_DTLS		BITS(7, 4)
#define BITS_STAT_B_CHGIN_DTSL		BITS(3, 2)
#define BIT_STAT_B_CHG			BIT(1)

/* Charger Config B */
#define BITS_CNFG_B_ICHGIN_LIM		BITS(4, 2)
#define BIT_CNFG_B_CHG_EN		BIT(0)

/* Charger Config C */
#define BITS_CNFG_C_TOPOFFTIMER		BITS(2, 0)

/* Charger Config E */
#define BITS_CNFG_E_CC			BITS(7, 2)
#define BITS_CNFG_E_TFASTCHG		BITS(1, 0)

/* Charger Config G */
#define BITS_CNFG_G_CHG_CV		BITS(7, 2)

enum {
	CHG_DTLS_OFF,
	CHG_DTLS_PREQUAL,
	CHG_DTLS_FASTCHARGE_CC,
	CHG_DTLS_JEITA_FASTCHARGE_CC,
	CHG_DTLS_FASTCHARGE_CV,
	CHG_DTLS_JEITA_FASTCHARGE_CV,
	CHG_DTLS_TOPOFF,
	CHG_DTLS_JEITA_TOPOFF,
	CHG_DTLS_DONE,
	CHG_DTLS_JEITA_DONE,
	CHG_DTLS_OFF_TIMER_FAULT,
	CHG_DTLS_OFF_CHARGE_TIMER_FAULT,
	CHG_DTLS_OFF_BATTERY_TEMP_FAULT,
	CHG_DTLS_RESERVED_13,
};

/* Chip Interrupts */
enum{
	CHG_IRQ_THM_I = 0,
	CHG_IRQ_CHG_I,
	CHG_IRQ_CHGIN_I,
	CHG_IRQ_TJ_REG_I,
	CHG_IRQ_CHGIN_CTRL_I,
	CHG_IRQ_SYS_CTRL_I,
	CHG_IRQ_SYS_CNFG_I,

	CHG_IRQ_MAX,
};

struct max77658_charger_dev {
	struct device *dev;
	struct max77658_dev *max77658;
	struct regmap *regmap;

	struct power_supply *psy_chg;
	struct power_supply_desc psy_chg_d;

	int irq;
	int irq_arr[CHG_IRQ_MAX];
	int irq_mask;

	struct delayed_work irq_work;
	/* mutex */
	struct mutex lock;

	int present;
	int health;
	int status;
	int charge_type;

	int fast_charge_current_ua;
	int fast_charge_timer_hr;
	int input_current_limit_ma;
	int topoff_timer_min;
};

#endif // __MAX77658_CHARGER_H_
