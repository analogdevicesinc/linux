// SPDX-License-Identifier: GPL-2.0
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

#define DEBUG

#define log_level	1

#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/power_supply.h>
#include <linux/mfd/max77840.h>
#include <linux/power/max77840-charger.h>
#ifdef __TEST_DEVICE_NODE__
#include <linux/string.h>
#include <linux/sysfs.h>
#endif

#define M2SH	__ffs

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

/* detail register bit description */

#define SIOP_INPUT_LIMIT_CURRENT                1200
#define SIOP_CHARGING_LIMIT_CURRENT             1000
#define SLOW_CHARGING_CURRENT_STANDARD          400

#define IRQ_WORK_DELAY              0
static char *chg_supplied_to[] = {
	"max77840-charger",
};

struct max77840_charger_data {
	struct device                  *dev;
	struct max77840_dev            *max77840;
	struct regmap                  *regmap;
	struct power_supply            *psy_chg;
	struct power_supply_desc       psy_chg_d;

	int                            byp_irq;
	int                            chgin_irq;
	int                            aicl_irq;
	int                            chg_irq;

	int                            irq;
	int                            irq_mask;
	int                            details_0;
	int                            details_1;
	int                            details_2;
	spinlock_t                     irq_lock; /* spin lock */
	struct delayed_work            irq_work;

	/* mutex */
	struct mutex                   lock;

	int                            present;
	int                            health;
	int                            status;
	int                            charge_type;

	struct max77840_charger_platform_data *pdata;
};

static inline struct power_supply *get_power_supply_by_name(char *name)
{
	if (!name)
		return (struct power_supply  *)NULL;
	else
		return power_supply_get_by_name(name);
}

static enum power_supply_property max77840_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static inline int GET_TO_ITH(int x)
{
	if (x < 4)
		return x * 25 + 100;
	else
		return x * 50;
}

static inline int SET_TO_ITH(int x)
{
	if (x < 100)
		return 0x00;
	else if (x < 200)
		return (x - 100) / 25;
	else if (x < 350)
		return x / 50;
	else
		return 0x07;
}

static inline int max77840_ffs(unsigned int x)
{
	return x ? __ffs(x) : 0;
}

static void max77840_charger_initialize(struct max77840_charger_data *charger);

/* charger API function */
static int max77840_charger_unlock(struct max77840_charger_data *charger)
{
	int rc;

	rc = regmap_update_bits(charger->regmap, REG_CHG_CNFG_06, BIT_CHGPROT, BIT_CHGPROT);
	if (rc < 0) {
		pr_err("%s: failed to unlock [%d]\n", __func__, rc);
		goto out;
	}

out:
	return rc;
}

static bool max77840_charger_present_input(struct max77840_charger_data *charger)
{
	u8 chg_int_ok = 0;
	int rc;

	rc = max77840_read(charger->regmap, REG_CHG_INT_OK, &chg_int_ok);
	if (rc < 0)
		return false;

	if ((chg_int_ok & BIT_CHGIN_OK) == BIT_CHGIN_OK)
		return true;
	/* check whether charging or not in the UVLO condition */
	if (((charger->details_0 & BIT_CHGIN_DTLS) == 0) &&
	    (((charger->details_1 & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CC) ||
	     ((charger->details_1 & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CV))) {
		return true;
	} else {
		return false;
	}
}

static int max77840_charger_get_input_current(struct max77840_charger_data *charger)
{
	u8 reg_data = 0;
	int steps[3] = { 0, 33, 67 };	/* mA */
	int get_current, quotient, remainder;

	pr_info("max77840 charger get input current");

	max77840_read(charger->regmap, REG_CHG_CNFG_09, &reg_data);

	quotient = reg_data / 3;
	remainder = reg_data % 3;

	if ((reg_data & BIT_CHGIN_ILIM) < 3)
		get_current = 100;	/* 100mA */
	else if ((reg_data & BIT_CHGIN_ILIM) > 0x78)
		get_current = 4000;	/* 4000mA */
	else
		get_current = quotient * 100 + steps[remainder];

	return get_current;
}

static int max77840_charger_set_input_current(struct max77840_charger_data
					      *charger,
					      int input_current)
{
	int quotient, remainder;
	u8 reg_data = 0;

	/* unit mA */
	if (!input_current) {
		reg_data = 0;
	} else {
		quotient = input_current / 100;
		remainder = input_current % 100;

		if (remainder >= 67)
			reg_data |= (quotient * 3) + 2;
		else if (remainder >= 33)
			reg_data |= (quotient * 3) + 1;
		else if (remainder < 33)
			reg_data |= quotient * 3;
	}
	pr_info("%s: reg_data(0x%02x), charging current(%d)\n",
		__func__, reg_data, input_current);

	return regmap_update_bits(charger->regmap, REG_CHG_CNFG_09,
			BIT_CHGIN_ILIM, reg_data);
}

static int max77840_charger_get_charge_current(struct max77840_charger_data *charger)
{
	struct regmap *regmap = charger->regmap;

	u8 reg_data = 0;
	int get_current;

	max77840_read(regmap, REG_CHG_CNFG_02, &reg_data);

	if ((reg_data & BIT_CHG_CC) < 2)
		get_current = 100;	/* 100mA */
	else
		get_current = (reg_data & BIT_CHG_CC) * 50;

	pr_info("%s: reg_data(0x%02x), get_current(%d)\n",
		__func__, reg_data, get_current);

	return get_current;
}

static int max77840_charger_set_charge_current(struct max77840_charger_data *charger,
					       int fast_charging_current)
{
	int curr_step = 50;
	u8 reg_data = 0;
	int rc;

	/* unit mA */
	if (!fast_charging_current) {
		rc = regmap_update_bits(charger->regmap, REG_CHG_CNFG_02, BIT_CHG_CC, 0);

	} else {
		reg_data = (fast_charging_current / curr_step);
		rc = regmap_update_bits(charger->regmap, REG_CHG_CNFG_02, BIT_CHG_CC, reg_data);
	}
	pr_info("%s: reg_data(0x%02x), charging current(%d)\n",
		__func__, reg_data, fast_charging_current);

	return rc;
}

static int max77840_charger_set_topoff_current(struct max77840_charger_data *charger,
					       int termination_current,
					       int termination_time)
{
	u8 reg_data;

	/* termination_current (mA) */
	reg_data = SET_TO_ITH(termination_current);

	/* termination_time (min) */
	reg_data |= ((termination_time / 10) << M2SH(BIT_TO_TIME));
	pr_info("%s: reg_data(0x%02x), topoff(%d), time(%d)\n",
		__func__, reg_data, termination_current, termination_time);
	return regmap_update_bits(charger->regmap, REG_CHG_CNFG_03,
				  BIT_TO_ITH | BIT_TO_TIME, reg_data);
}

static int max77840_charger_set_enable(struct max77840_charger_data *charger, int en)
{
	return regmap_update_bits(charger->regmap,
		REG_CHG_CNFG_00, BIT_MODE_CHARGER, !!en);
}

static int max77840_charger_exit_dev(struct max77840_charger_data *charger)
{
	struct max77840_charger_platform_data *pdata = charger->pdata;
	int rc;

	rc = max77840_charger_set_enable(charger, false);
	if (rc < 0) {
		pr_err("CHG_CNFG_00 write error [%d]\n", rc);
		return rc;
	}

	rc = max77840_charger_set_charge_current(charger, pdata->fast_charge_current);

	return rc;
}

static int max77840_charger_init_dev(struct max77840_charger_data *charger)
{
	int rc;

	/* charger enable */
	rc = max77840_charger_set_enable(charger, true);

	return rc;
}

static void max77840_charger_initialize(struct max77840_charger_data *charger)
{
	struct max77840_charger_platform_data *pdata = charger->pdata;
	int rc;
	u8 val, temp_val;

	/* interrupt mask - if you want to enable some bits, you should clear them */
	val  = 0;
	val |= BIT_AICL;
	val |= BIT_CHGIN;
	//val |= BIT_TOPOFF;
	//val |= BIT_CHG;
	val |= BIT_BAT;
	val |= BIT_BATP;
	val |= BIT_BAT2SOC;
	val |= BIT_BYP;

	rc = max77840_write(charger->regmap, REG_CHG_INT_MASK, val);
	if (rc < 0) {
		pr_err("CHG_INT_MASK write error [%d]\n", rc);
		goto out;
	}

	pr_info("%s: CHG_INT_MASK is set to 0x%x\n", __func__, val);

	/* unlock charger register */
	rc = max77840_charger_unlock(charger);
	if (rc < 0)
		goto out;

	/* charge current (mA) */
	rc = max77840_charger_set_charge_current(charger, pdata->fast_charge_current);
	if (rc < 0)
		goto out;

	/* input current limit (mA) */
	rc = max77840_charger_set_input_current(charger, pdata->input_current_limit);
	if (rc < 0)
		goto out;

	/* topoff current(mA) and topoff timer(min) */
	rc = max77840_charger_set_topoff_current(charger,
						 pdata->topoff_current,
						 pdata->topoff_timer);
	if (rc < 0)
		goto out;

	/* charge restart threshold(mV) and fast-charge timer(hr) */
	val = pdata->restart_threshold < 200 ?
		(int)(pdata->restart_threshold - 100) / 50 : 0x03;

	temp_val = pdata->fast_charge_timer == 0 ? 0x00 :
		pdata->fast_charge_timer < 4 ? 0x01 :
		pdata->fast_charge_timer < 16 ?
		(int)DIV_ROUND_UP(pdata->fast_charge_timer - 4, 2) + 1 : 0x00;

	val = val << M2SH(BIT_CHG_RSTRT) | temp_val << M2SH(BIT_FCHGTIME);

	rc = regmap_update_bits(charger->regmap, REG_CHG_CNFG_01,
				(BIT_CHG_RSTRT | BIT_FCHGTIME), val);
	if (rc < 0)
		goto out;

	/* charge termination voltage (mV) */
	val = pdata->termination_voltage < 3650 ? 0x00 :
		pdata->termination_voltage <= 4325 ?
		(int)DIV_ROUND_UP(pdata->termination_voltage - 3650, 25) :
		pdata->termination_voltage <= 4340 ? 0x1C :
		pdata->termination_voltage <= 4700 ?
		(int)DIV_ROUND_UP(pdata->termination_voltage - 3650, 25) + 1 : 0x2B;
	rc = regmap_update_bits(charger->regmap,
				REG_CHG_CNFG_04,
				BIT_CHG_CV_PRM,
				val << M2SH(BIT_CHG_CV_PRM));
	if (rc < 0)
		goto out;

out:
	return;
}

struct max77840_charger_status_map {
	int health, status, charge_type;
};

static struct max77840_charger_status_map max77840_charger_status_map[] = {
#define STATUS_MAP(_chg_dtls, _health, _status, _charge_type) \
	[CHG_DTLS_##_chg_dtls] = {\
		.health = POWER_SUPPLY_HEALTH_##_health,\
		.status = POWER_SUPPLY_STATUS_##_status,\
		.charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type,\
	}
	/* chg_details_xx, health, status, charge_type */
	STATUS_MAP(PREQUAL, GOOD, CHARGING, TRICKLE),
	STATUS_MAP(FASTCHARGE_CC, GOOD, CHARGING, FAST),
	STATUS_MAP(FASTCHARGE_CV, GOOD, CHARGING, FAST),
	STATUS_MAP(TOPOFF, GOOD, CHARGING, FAST),
	STATUS_MAP(DONE, GOOD, FULL, NONE),
	STATUS_MAP(OFF_TIMER_FAULT, SAFETY_TIMER_EXPIRE, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_SUSPEND, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_INPUT_INVALID, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_JUCTION_TEMP, UNKNOWN, NOT_CHARGING, UNKNOWN),
	STATUS_MAP(OFF_WDT_EXPIRED, WATCHDOG_TIMER_EXPIRE, NOT_CHARGING, UNKNOWN),
};

static int max77840_charger_update(struct max77840_charger_data *charger)
{
	int rc;
	u8 chg_details[3];
	u8 chg_dtls;

	charger->health      = POWER_SUPPLY_HEALTH_UNKNOWN;
	charger->status      = POWER_SUPPLY_STATUS_UNKNOWN;
	charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	rc = max77840_bulk_read(charger->regmap, REG_CHG_DTLS_00, chg_details, 3);
	if (rc < 0) {
		pr_err("CHG_DETAILS read error [%d]\n", rc);
		goto out;
	}

	pr_info("%s: chg_details 00=0x%x, 01=0x%x, 02=0x%x\n",
		__func__, chg_details[0], chg_details[1], chg_details[2]);

	charger->present = max77840_charger_present_input(charger);
	if (unlikely(!charger->present)) {
		/* no charger present */
		charger->health      = POWER_SUPPLY_HEALTH_UNKNOWN;
		charger->status      = POWER_SUPPLY_STATUS_DISCHARGING;
		charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto out;
	}

	chg_dtls = chg_details[1] & BIT_CHG_DTLS;

	charger->health = max77840_charger_status_map[chg_dtls].health;
	charger->status = max77840_charger_status_map[chg_dtls].status;
	charger->charge_type = max77840_charger_status_map[chg_dtls].charge_type;

	if (likely(charger->health != POWER_SUPPLY_HEALTH_UNKNOWN))
		goto out;

	/* override health by TREG */
	if ((chg_details[1] & BIT_TREG) != 0)
		charger->health = POWER_SUPPLY_HEALTH_OVERHEAT;

out:
	pr_info("%s: PRESENT %d HEALTH %d STATUS %d CHARGE_TYPE %d\n",
		__func__,
		charger->present, charger->health,
		charger->status, charger->charge_type);
	return rc;
}

static int max77840_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max77840_charger_data *charger = power_supply_get_drvdata(psy);
	int rc = 0;

	rc = max77840_charger_update(charger);
	if (rc < 0)
		goto out;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->present;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = charger->health;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = charger->status;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = charger->charge_type;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max77840_charger_get_charge_current(charger);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = max77840_charger_get_input_current(charger);
		break;

	default:
		rc = -EINVAL;
		goto out;
	}

out:
	pr_info("%s: <get_property> psp %d val %d [%d]\n",
		__func__, psp, val->intval, rc);
	return rc;
}

static int max77840_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct max77840_charger_data *charger = power_supply_get_drvdata(psy);
	int rc = 0;

	__lock(charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		rc = max77840_charger_set_enable(charger, val->intval);
		if (rc < 0)
			goto out;
			/* apply charge current */
		rc = max77840_charger_set_charge_current(charger,
							 charger->pdata->fast_charge_current);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* val->intval - uA */
		rc = max77840_charger_set_charge_current(charger, val->intval / 1000); /* mA */
		if (rc < 0)
			goto out;
		charger->pdata->fast_charge_current = val->intval / 1000; /* mA */
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77840_charger_set_input_current(charger, val->intval / 1000); /* mA */
		if (rc < 0)
			goto out;
		charger->pdata->input_current_limit = val->intval / 1000; /* mA */
		break;

	default:
		rc = -EINVAL;
		goto out;
	}

out:
	pr_info("%s: <set_property> psp %d val %d [%d]\n",
		__func__, psp, val->intval, rc);
	__unlock(charger);
	return rc;
}

/* interrupt handler and workqueue */
static void max77840_do_irq(struct max77840_charger_data *charger, int irq)
{
	u8 val, chg_details[3];
	bool chg_input, bat2soc_state;
	bool chgini_mode, aicl_mode, topoff_state;

	chg_details[0] = charger->details_0;
	chg_details[1] = charger->details_1;
	chg_details[2] = charger->details_2;

	switch (irq) {
	case CHG_INT_AICL_CHGINI_I:
		aicl_mode = (chg_details[2] & BIT_AICL_DTLS) ? false : true;
		chgini_mode = (chg_details[2] & BIT_CHGINI_DTLS) ? false : true;
		pr_debug("CHG_INT_AICL_CHGINI: AICL_DTLS %s\n",
			 aicl_mode ? "In AICL mode" : "Not in AICL mode");
		pr_debug("CHG_INT_AICL_CHGINI: CHGINI_DTLS %s\n",
			 chgini_mode ? "In CHGINI mode" : "Not in CHGINI mode");
		break;

	case CHG_INT_CHGIN_I:
		chg_input = max77840_charger_present_input(charger);

		if (chg_input) {
			/* charger insert */
			max77840_charger_init_dev(charger);
		} else {
			/* charger remove */
			max77840_charger_exit_dev(charger);
		}
		pr_debug("CHG_INT_CHGIN: Charger input %s\n",
			 chg_input ? "inserted" : "removed");
		break;

	case CHG_INT_TOPOFF_I:
		max77840_read(charger->regmap, REG_CHG_INT_OK, &val);
		topoff_state = (val & BIT_TOPOFF_OK) ? false : true;
		pr_debug("CHG_INT_TOPOFF: TOPOFF %s\n",
			 topoff_state ? "Not in TOPOFF state" : "in TOPOFF state");
		break;

	case CHG_INT_CHG_I:
		/* do insert code here */
		val = (chg_details[1] & BIT_CHG_DTLS) >> max77840_ffs(BIT_CHG_DTLS);
		pr_debug("CHG_INT_CHG: chg_dtls = %02Xh\n", val);
		break;

	case CHG_INT_BAT_I:
		/* do insert code here */
		val = (chg_details[1] & BIT_BAT_DTLS) >> max77840_ffs(BIT_BAT_DTLS);
		pr_debug("CHG_INT_BAT: bat_dtls = %02Xh\n", val);
		break;

	case CHG_INT_BATP_I:
		/* do insert code here */
		val = (chg_details[0] & BIT_BATP_DTLS) >> max77840_ffs(BIT_BATP_DTLS);
		pr_debug("CHG_INT_BATP: battery %s\n",
			 val ? "no presence" : "presence");
		break;

	case CHG_INT_BAT2SOC_I:
		/* do insert code here */
		max77840_read(charger->regmap, REG_CHG_INT_OK, &val);
		bat2soc_state = (val & BIT_BAT2SOC_OK) ? false : true;
		pr_debug("CHG_INT_BAT2SOC: battery to SYS %s\n",
			 val ? "has hit overcurrent limit" : "is okay");
		break;

	case CHG_INT_BYP_I:
		/* do insert code here */
		val = (chg_details[2] & BIT_BYP_DTLS) >> max77840_ffs(BIT_BYP_DTLS);
		pr_debug("CHG_INT_BYP: byp_dtls = %02Xh\n", val);
		break;

	default:
		break;
	}
	/* notify psy changed */
	power_supply_changed(charger->psy_chg);
}

static void max77840_charger_irq_work(struct work_struct *work)
{
	struct max77840_charger_data *me =
		container_of(work, struct max77840_charger_data, irq_work.work);
	u8 irq;

	__lock(me);

	irq = me->irq - me->byp_irq;

	max77840_do_irq(me, irq);

	__unlock(me);
}

static irqreturn_t max77840_charger_chgin_isr(int irq, void *data)
{
	struct max77840_charger_data *me = data;
	u8 reg_details[3];

	me->irq = irq;

	max77840_bulk_read(me->regmap, REG_CHG_DTLS_00, reg_details, 3);
	pr_info("%s: chg_dtls[0]=0x%x, [1]=0x%x, [2]=0x%x\n",
		__func__, reg_details[0], reg_details[1], reg_details[2]);

	me->details_0 = reg_details[0];
	me->details_1 = reg_details[1];
	me->details_2 = reg_details[2];

	schedule_delayed_work(&me->irq_work, IRQ_WORK_DELAY);
	return IRQ_HANDLED;
}

static irqreturn_t max77840_charger_aicl_isr(int irq, void *data)
{
	struct max77840_charger_data *me = data;
	u8 reg_details[3];

	me->irq = irq;

	max77840_bulk_read(me->regmap, REG_CHG_DTLS_00, reg_details, 3);
	pr_info("%s: chg_dtls[0]=0x%x, [1]=0x%x, [2]=0x%x\n",
		__func__, reg_details[0], reg_details[1], reg_details[2]);

	me->details_0 = reg_details[0];
	me->details_1 = reg_details[1];
	me->details_2 = reg_details[2];

	schedule_delayed_work(&me->irq_work, IRQ_WORK_DELAY);
	return IRQ_HANDLED;
}

static irqreturn_t max77840_charger_chg_isr(int irq, void *data)
{
	struct max77840_charger_data *me = data;
	u8 reg_details[3];

	me->irq = irq;

	max77840_bulk_read(me->regmap, REG_CHG_DTLS_00, reg_details, 3);
	pr_info("%s: chg_dtls[0]=0x%x, [1]=0x%x, [2]=0x%x\n",
		__func__, reg_details[0], reg_details[1], reg_details[2]);

	me->details_0 = reg_details[0];
	me->details_1 = reg_details[1];
	me->details_2 = reg_details[2];

	schedule_delayed_work(&me->irq_work, IRQ_WORK_DELAY);
	return IRQ_HANDLED;
}

#ifdef __TEST_DEVICE_NODE__
static struct max77840_charger_data *test_if;
static ssize_t max77840_test_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int ret, reg;
	unsigned long val;
	u8 rval;
	char *args[3];
	char *sep = ",\t";
	char *buff = (char  *)buf;
	struct regmap *if_regmap = test_if->regmap;

	args[0] = strsep(&buff, sep);
	if (!args[0]) {
		dev_err(test_if->dev, "Command has wrong format.\n");
		return -2;
	}

	// register read
	args[1] = strsep(&buff, sep);
	if (strncmp("read", args[0], 4) == 0) {
		ret = kstrtoul(args[1], 0, &val);
		reg = (int)val;
		ret = max77840_read(if_regmap, reg, &rval);
		if (ret < 0)
			dev_err(test_if->dev, "failed to read i2c: %d @ function %s\n",
				ret, __func__);
		else
			dev_err(test_if->dev, "read [0x%x] = 0x%x\n", reg, rval);
		return size;
	}
	// register write
	else if (strncmp("write", args[0], 5) == 0) {
		ret = kstrtoul(args[1], 0, &val);
		reg = (int)val;
		args[2] = strsep(&buff, sep);
		ret = kstrtoul(args[2], 0, &val);
		rval = (int)val;
		ret = max77840_write(if_regmap, reg, rval);
		if (ret < 0)
			dev_err(test_if->dev, "failed to write i2c: %d @ function %s\n",
				ret, __func__);
		else
			dev_err(test_if->dev, "write [0x%x] = 0x%x\n", reg, rval);
		return size;
	}
	dev_err(test_if->dev, "Command not supported.\n");

	return 0;
}

static struct device_attribute max77840_attribute = {
		.attr = {
			.name = "max77840_test_node",
			.mode = 0x0666,
			},
		.show = NULL,
		.store = max77840_test_store,
};

static int max77840_test_node(struct max77840_charger_data *charger)
{
	int ret;

	ret = sysfs_create_file(&charger->psy_chg->dev.kobj, &max77840_attribute.attr);
	test_if = charger;
	return ret;
}
#else
static int max77840_test_node(struct max77840_charger_data *charger)
{
	return 0;
}
#endif

#ifdef CONFIG_OF
static int max77840_charger_parse_dt(struct max77840_charger_data *charger)
{
	struct device *dev = charger->dev;
	struct device_node *np = of_find_node_by_name(NULL, "charger");
	struct max77840_charger_platform_data *pdata;

	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata))
		return -ENOMEM;

	pdata->fast_charge_timer = 4;	/* disable */
	ret |= of_property_read_u32(np, "fast_charge_timer", &pdata->fast_charge_timer);
	pr_debug("property:FCHGTIME %uhour\n", pdata->fast_charge_timer);

	pdata->fast_charge_current = 500;	/* 500mA */
	ret |= of_property_read_u32(np, "fast_charge_current", &pdata->fast_charge_current);
	pr_debug("property:CHG_CC %umA\n", pdata->fast_charge_current);

	pdata->termination_voltage = 4350;	/* 4350mV */
	ret |= of_property_read_u32(np, "charge_termination_voltage", &pdata->termination_voltage);
	pr_debug("property:CHG_CV_PRM %umV\n", pdata->termination_voltage);

	pdata->topoff_timer = 30;	/* 0 min */
	ret |= of_property_read_u32(np, "topoff_timer", &pdata->topoff_timer);
	pr_debug("property:TOPOFF_TIME %umin\n", pdata->topoff_timer);

	pdata->topoff_current = 150;	/* 200mA */
	ret |= of_property_read_u32(np, "topoff_current", &pdata->topoff_current);
	pr_debug("property:TOPOFF_ITH %umA\n", pdata->topoff_current);

	pdata->restart_threshold = 150;	/* 150mV */
	ret |= of_property_read_u32(np, "restart_threshold", &pdata->restart_threshold);
	pr_debug("property:CHG_RSTRT %umV\n", pdata->restart_threshold);

	pdata->input_current_limit = 500; /* 500mA */
	ret |= of_property_read_u32(np, "input_current_limit", &pdata->input_current_limit);
	pr_debug("property:INPUT_CURRENT_LIMIT %umA\n", pdata->input_current_limit);

	if (ret < 0)
		return ret;
	charger->pdata = pdata;
	return 0;
}
#endif

static int max77840_charger_probe(struct platform_device *pdev)
{
	struct max77840_dev *max77840 = dev_get_drvdata(pdev->dev.parent);
	struct max77840_charger_platform_data *pdata;
	struct max77840_charger_data *charger;
	int ret = 0;
	u8 val = 0;
	struct power_supply_config charger_cfg;

	pr_err("%s: Max77840 Charger Driver Loading\n", __func__);

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("%s: out of memory\n", __func__);
		pdata = ERR_PTR(-ENOMEM);
		return -ENOMEM;
	}

	mutex_init(&charger->lock);

	charger->dev = &pdev->dev;
	charger->regmap = max77840->regmap_chg;
	charger->pdata = pdata;

#if defined(CONFIG_OF)
	ret = max77840_charger_parse_dt(charger);
	if (ret < 0)
		pr_err("%s not found charger dt! ret[%d]\n", __func__, ret);
#else
	pdata = dev_get_platdata(&pdev->dev);
#endif

	platform_set_drvdata(pdev, charger);
	charger->psy_chg_d.name		= "max77840-charger";
	charger->psy_chg_d.type		= POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg_d.get_property	= max77840_charger_get_property;
	charger->psy_chg_d.set_property	= max77840_charger_set_property;
	charger->psy_chg_d.properties	= max77840_charger_props;
	charger->psy_chg_d.num_properties = ARRAY_SIZE(max77840_charger_props);
	charger_cfg.drv_data = charger;
	charger_cfg.supplied_to = chg_supplied_to;
	charger_cfg.of_node = max77840->dev->of_node;
	charger_cfg.num_supplicants = ARRAY_SIZE(chg_supplied_to);

	max77840_charger_initialize(charger);

	charger->psy_chg = devm_power_supply_register(max77840->dev,
						      &charger->psy_chg_d,
						      &charger_cfg);
	if (IS_ERR(charger->psy_chg)) {
		pr_err("Couldn't register psy_chg rc=%ld\n", PTR_ERR(charger->psy_chg));
		goto err_power_supply_register;
	}

	INIT_DELAYED_WORK(&charger->irq_work, max77840_charger_irq_work);

	charger->byp_irq = regmap_irq_get_virq(max77840->irqc_chg, CHG_IRQ_BYP_I);
	charger->chgin_irq = regmap_irq_get_virq(max77840->irqc_chg, CHG_IRQ_CHGIN_I);
	charger->aicl_irq = regmap_irq_get_virq(max77840->irqc_chg, CHG_IRQ_AICL_I);
	charger->chg_irq = regmap_irq_get_virq(max77840->irqc_chg, CHG_IRQ_CHG_I);

	ret = request_threaded_irq(charger->chgin_irq, NULL,
				   max77840_charger_chgin_isr,
				   IRQF_ONESHOT | IRQF_TRIGGER_FALLING, "charger-chgin", charger);
	if (ret) {
		pr_err("%s: Failed to Request CHGIN IRQ\n", __func__);
		goto err_chgin_irq;
	}

	ret = request_threaded_irq(charger->aicl_irq, NULL,
				   max77840_charger_aicl_isr,
				   IRQF_ONESHOT | IRQF_TRIGGER_FALLING, "charger-aicl", charger);
	if (ret) {
		pr_err("%s: Failed to Request AICL IRQ\n", __func__);
		goto err_aicl_irq;
	}

	ret = request_threaded_irq(charger->chg_irq, NULL,
				   max77840_charger_chg_isr,
				   IRQF_ONESHOT  | IRQF_TRIGGER_FALLING, "charger-chg", charger);
	if (ret) {
		pr_err("%s: Failed to Request CHG IRQ\n", __func__);
		goto err_chg_irq;
	}

	/* clear IRQ */
	max77840_read(charger->regmap, REG_CHG_INT, &val);
	pr_err("%s: REG_CHG_INT %Xh\n", __func__, val);

	max77840_read(charger->regmap, REG_CHG_INT_MASK, &val);

	pr_info("%s: Max77840 Charger Driver Loaded\n", __func__);
	max77840_test_node(charger);
	return 0;

err_chg_irq:
	free_irq(charger->aicl_irq, NULL);
err_aicl_irq:
	free_irq(charger->chgin_irq, NULL);
err_chgin_irq:
	power_supply_unregister(charger->psy_chg);
err_power_supply_register:
	kfree(charger);

	return ret;
}

static int max77840_charger_remove(struct platform_device *pdev)
{
	struct max77840_charger_data *charger =
		platform_get_drvdata(pdev);

	free_irq(charger->chgin_irq, NULL);
	free_irq(charger->aicl_irq, NULL);
	power_supply_unregister(charger->psy_chg);

	kfree(charger);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77840_charger_suspend(struct device *dev)
{
	return 0;
}

static int max77840_charger_resume(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77840_charger_pm_ops, max77840_charger_suspend,
		max77840_charger_resume);

#ifdef CONFIG_OF
static const struct of_device_id max77840_charger_dt_ids[] = {
	{ .compatible = "maxim,max77840-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77840_charger_dt_ids);
#endif

static struct platform_driver max77840_charger_driver = {
	.driver = {
		.name = MAX77840_CHARGER_NAME,
		.pm = &max77840_charger_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = max77840_charger_dt_ids,
#endif
	},
	.probe = max77840_charger_probe,
	.remove = max77840_charger_remove,
};

static int __init max77840_charger_init(void)
{
	return platform_driver_register(&max77840_charger_driver);
}

static void __exit max77840_charger_exit(void)
{
	platform_driver_unregister(&max77840_charger_driver);
}

module_init(max77840_charger_init);
module_exit(max77840_charger_exit);

MODULE_DESCRIPTION("MAX77840 Charger Driver");
MODULE_AUTHOR("joan.na@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
