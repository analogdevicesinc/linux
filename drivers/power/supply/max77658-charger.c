// SPDX-License-Identifier: GPL-2.0-or-later

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/max77658.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/max77658-charger.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/version.h>

static char *chg_supplied_to[] = {
	MAX77658_CHARGER_NAME,
};

#define M2SH __CONST_FFS

static int max77658_set_input_current_limit(struct max77658_charger_dev *charger,
					    int input_current)
{
	int curr_step = 95;
	u8 reg_data = 0;

	/* unit mA */
	if (input_current)
		reg_data = input_current <= 475 ?
				(5 - (int)DIV_ROUND_UP(input_current, curr_step)) : 0x04;
	else
		reg_data = 0;

	reg_data = reg_data << M2SH(BITS_CNFG_B_ICHGIN_LIM);

	return regmap_update_bits(charger->regmap, REG_CNFG_CHG_B,
				  BITS_CNFG_B_ICHGIN_LIM, reg_data);
}

static int max77658_get_input_current_limit(struct max77658_charger_dev *charger,
					    int *get_current)
{
	unsigned int reg_data = 0;
	int ret;

	ret = regmap_read(charger->regmap, REG_CNFG_CHG_B, &reg_data);
	if (ret)
		return ret;

	reg_data = (reg_data & BITS_CNFG_B_ICHGIN_LIM) >> M2SH(BITS_CNFG_B_ICHGIN_LIM);
	*get_current = reg_data * 95; // 95 mA

	return 0;
}

static int max77658_set_charge_current(struct max77658_charger_dev *charger,
				       int fast_charge_current_ua)
{
	int curr_step = 7500;
	int max_value = 300000;
	u8 reg_data = 0;
	int ret;

	if (fast_charge_current_ua) {
		if (fast_charge_current_ua < curr_step)
			reg_data = 0x00;
		else if (fast_charge_current_ua > max_value)
			reg_data = 0x27;
		else
			reg_data = (fast_charge_current_ua / curr_step) - 1;

		reg_data = reg_data << M2SH(BITS_CNFG_E_CC);
		ret = regmap_update_bits(charger->regmap,
					 REG_CNFG_CHG_E,
					 BITS_CNFG_E_CC,
					 reg_data);
	} else {
		ret = regmap_update_bits(charger->regmap,
					 REG_CNFG_CHG_E,
					 BITS_CNFG_E_CC,
					 0);
	}

	if (ret)
		dev_err(charger->dev, "Error in writing register CNFG_CHG_E\n");

	return ret;
}

static int max77658_get_charge_current(struct max77658_charger_dev *charger,
				       int *get_current)
{
	struct regmap *regmap = charger->regmap;

	unsigned int reg_data = 0;
	int curr_step = 7500;
	int max_value = 300000;
	int ret;

	ret = regmap_read(regmap, REG_CNFG_CHG_E, &reg_data);
	if (ret)
		return ret;

	reg_data = (reg_data & BITS_CNFG_E_CC) >> M2SH(BITS_CNFG_E_CC);
	if (reg_data > 27)
		*get_current = max_value; /* 300000uA */
	else
		*get_current = reg_data * curr_step; /* 7500 uA */

	return 0;
}

static int max77658_set_topoff_timer(struct max77658_charger_dev *charger,
				     int termination_time)
{
	u8 reg_data;

	if (termination_time > 35)
		termination_time = 35;

	/* termination_time (min) */
	reg_data = ((termination_time / 5) << M2SH(BITS_CNFG_C_TOPOFFTIMER));

	return regmap_update_bits(charger->regmap,
				  REG_CNFG_CHG_C,
				  BITS_CNFG_C_TOPOFFTIMER,
				  reg_data);
}

static int max77658_charger_initialize(struct max77658_charger_dev *charger)
{
	int ret;
	unsigned int val;

	/*
	 * Interrupt mask
	 * To enable interrupt, set the corresponding bit to '0'
	 */
	val  = 0;
	val |= (BIT_INT_SYS_CNFG | BIT_INT_SYS_CTRL | BIT_INT_CHGIN_CTRL |
			BIT_INT_TJ_REG | BIT_INT_CHGIN | BIT_INT_CHG | BIT_INT_THM);

	ret = regmap_write(charger->regmap, REG_INT_M_CHG, val);
	if (ret)
		return dev_err_probe(charger->dev, ret,
				     "Error in writing register INT_M_CHG\n");

	/* charge current (uA) */
	ret = max77658_set_charge_current(charger, charger->fast_charge_current_ua);
	if (ret)
		return dev_err_probe(charger->dev, ret,
				     "Error in writing register CNFG_CHG_C\n");

	if (charger->fast_charge_timer_hr == 0)
		val = 0x00;
	else if (charger->fast_charge_timer_hr < 3)
		val = 0x01;
	else if (charger->fast_charge_timer_hr <= 7)
		val = (int)DIV_ROUND_UP(charger->fast_charge_timer_hr - 1, 2);
	else
		val = 0x00;

	ret = regmap_update_bits(charger->regmap,
				 REG_CNFG_CHG_E,
				 BITS_CNFG_E_TFASTCHG,
				 val);
	if (ret)
		return dev_err_probe(charger->dev, ret,
				     "Error in writing register CNFG_CHG_E\n");

	/* input current limit (mA) */
	ret = max77658_set_input_current_limit(charger, charger->input_current_limit_ma);
	if (ret)
		return dev_err_probe(charger->dev, ret,
				     "Error in writing register CNFG_CHG_B\n");

	/* topoff timer(min) */
	return max77658_set_topoff_timer(charger, charger->topoff_timer_min);
}

static int max77658_charger_parse_dt(struct max77658_charger_dev *charger)
{
	struct device *dev = charger->dev;
	struct device_node *np = of_find_node_by_name(NULL, "charger");
	int ret;

	/* Charger */
	if (!np)
		return dev_err_probe(dev, -ENODEV, 
				     "Failed to find device_node\n");

	ret = of_property_read_u32(np, "fast-charge-timer",
				   &charger->fast_charge_timer_hr);
	if (ret)
		charger->fast_charge_timer_hr = 0;

	ret = of_property_read_u32(np, "fast-charge-current",
				   &charger->fast_charge_current_ua);
	if (ret)
		charger->fast_charge_current_ua = 15000;

	ret = of_property_read_u32(np, "topoff-timer",
				   &charger->topoff_timer_min);
	if (ret)
		charger->topoff_timer_min = 30;

	ret = of_property_read_u32(np, "input-current-limit",
				   &charger->input_current_limit_ma);
	if (ret)
		charger->input_current_limit_ma = 475;

	return 0;
}

struct max77658_charger_status_map {
	int health;
	int status;
	int charge_type;
};

static struct max77658_charger_status_map max77658_charger_status_map[] = {
	/* chg_details_xx, health, status, charge_type */
	STATUS_MAP(OFF, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(PREQUAL, GOOD, CHARGING, TRICKLE),
	STATUS_MAP(FASTCHARGE_CC, GOOD, CHARGING, FAST),
	STATUS_MAP(JEITA_FASTCHARGE_CC, GOOD, CHARGING, FAST), // Jeita CC
	STATUS_MAP(FASTCHARGE_CV, GOOD, CHARGING, FAST),
	STATUS_MAP(JEITA_FASTCHARGE_CV, GOOD, CHARGING, FAST), // Jeita CV
	STATUS_MAP(TOPOFF, GOOD, CHARGING, FAST),
	STATUS_MAP(JEITA_TOPOFF, GOOD, CHARGING, FAST), // Jeita TOPOFF
	STATUS_MAP(DONE, GOOD, FULL, NONE),
	STATUS_MAP(JEITA_DONE, GOOD, FULL, NONE), // Jeita Done
	STATUS_MAP(OFF_TIMER_FAULT, SAFETY_TIMER_EXPIRE, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_CHARGE_TIMER_FAULT, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_BATTERY_TEMP_FAULT, HOT, NOT_CHARGING, NONE),
};

static int max77658_charger_update(struct max77658_charger_dev *charger)
{
	unsigned int stat_chg_b, chg_dtls;
	int ret;

	charger->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	charger->status = POWER_SUPPLY_STATUS_UNKNOWN;
	charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	ret = regmap_read(charger->regmap, REG_STAT_CHG_B, &stat_chg_b);
	if (ret)
		return ret;

	charger->present = stat_chg_b & BIT_STAT_B_CHG;
	if (!charger->present) {
		/* no charger present */
		charger->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		charger->status = POWER_SUPPLY_STATUS_DISCHARGING;
		charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		return 0;
	}

	chg_dtls = (stat_chg_b & BITS_STAT_B_CHG_DTLS) >> M2SH(BITS_STAT_B_CHG_DTLS);

	charger->health = max77658_charger_status_map[chg_dtls].health;
	charger->status = max77658_charger_status_map[chg_dtls].status;
	charger->charge_type = max77658_charger_status_map[chg_dtls].charge_type;

	return 0;
}

static enum power_supply_property max77658_charger_battery_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX
};

static int max77658_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		return 0;
	}

	return ret;
}

static int max77658_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{

	struct max77658_charger_dev *charger = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&charger->lock);

	ret = max77658_charger_update(charger);
	if (ret)
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
		ret = max77658_get_charge_current(charger, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77658_get_input_current_limit(charger, &val->intval);
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&charger->lock);

	return ret;
}

static int max77658_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct max77658_charger_dev *charger = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&charger->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_update_bits(charger->regmap,
					 REG_CNFG_CHG_B,
					 BIT_CNFG_B_CHG_EN,
					 !!val->intval);
		if (ret)
			goto out;

		ret = max77658_set_charge_current(charger,
					charger->fast_charge_current_ua);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* val->intval - uA */
		ret = max77658_set_charge_current(charger, val->intval);
		if (ret)
			goto out;

		charger->fast_charge_current_ua = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		/* val->intval - uA */
		ret = max77658_set_input_current_limit(charger, val->intval);
		if (ret)
			goto out;

		charger->input_current_limit_ma = val->intval;
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&charger->lock);

	return ret;
}

/* interrupt handler and workqueue */
static void max77658_do_irq(struct max77658_charger_dev *charger, int irq)
{
	unsigned int val, stat_chg_b;
	int chg_present, vchgin, ichgin;
	int ret;

	switch (irq) {
	case CHG_IRQ_THM_I:
		ret = regmap_read(charger->regmap, REG_STAT_CHG_A, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_A\n");
			return;
		}

		val = (val & BITS_STAT_A_THM_DTLS) >> M2SH(BITS_STAT_A_THM_DTLS);
		dev_dbg(charger->dev, "CHG_INT_THM: thm_dtls = %02Xh\n", val);
		break;

	case CHG_IRQ_CHG_I:
		ret = regmap_read(charger->regmap, REG_STAT_CHG_B, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_B\n");
			return;
		}

		val = (val & BITS_STAT_B_CHG_DTLS) >> FFS(BITS_STAT_B_CHG_DTLS);
		dev_dbg(charger->dev, "CHG_INT_CHG: chg_dtls = %02Xh\n", val);
		break;

	case CHG_IRQ_CHGIN_I:
		ret = regmap_read(charger->regmap, REG_STAT_CHG_B, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_B\n");
			return;
		}

		val = (val & BITS_STAT_B_CHG_DTLS) >> FFS(BITS_STAT_B_CHG_DTLS);
		dev_dbg(charger->dev, "CHG_INT_CHG: chg_dtls = %02Xh\n", val);

		ret = regmap_read(charger->regmap, REG_STAT_CHG_B, &stat_chg_b);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_B\n");
			return;
		}

		chg_present = stat_chg_b & BIT_STAT_B_CHG;

		if (chg_present) {
			/* charger insert */
			regmap_update_bits(charger->regmap,
					   REG_CNFG_CHG_B,
					   BIT_CNFG_B_CHG_EN,
					   true);
		} else {
			/* charger remove */
			regmap_update_bits(charger->regmap,
					   REG_CNFG_CHG_B,
					   BIT_CNFG_B_CHG_EN,
					   false);

			max77658_set_charge_current(charger, charger->fast_charge_current_ua);
		}
		dev_dbg(charger->dev,
			"CHG_INT_CHGIN: Charger input %s\n",
			chg_present ? "inserted" : "removed");
		break;

	case CHG_IRQ_TJ_REG_I:
		ret = regmap_read(charger->regmap, REG_STAT_CHG_A, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_A\n");
			return;
		}

		val = (val & BIT_STAT_A_TJ_REG_STAT) >> M2SH(BIT_STAT_A_TJ_REG_STAT);
		dev_dbg(charger->dev, "CHG_INT_TJ_REG: tj_reg_stat = %02Xh\n", val);
		dev_dbg(charger->dev,
			"CHG_INT_TJ_REG: Die temperature %s Tj-reg\n",
			val ? "has exceeded" : "has not exceeded");
		break;

	case CHG_IRQ_CHGIN_CTRL_I:
		ret = regmap_read(charger->regmap, REG_STAT_CHG_A, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_A\n");
			return;
		}

		vchgin = (val & BIT_STAT_A_VCHGIN_MIN_STAT) >>
					M2SH(BIT_STAT_A_VCHGIN_MIN_STAT);
		dev_dbg(charger->dev,
			"CHG_INT_CHGIN: VCHGIN_MIN_STAT %s\n",
			vchgin ? "has changed" : "has not changed");

		ichgin = (val & BIT_STAT_A_ICHGIN_LIM_STAT) >>
					M2SH(BIT_STAT_A_ICHGIN_LIM_STAT);
		dev_dbg(charger->dev,
			"CHG_INT_CHGIN: ICHGIN_LIM_STAT %s\n",
			ichgin ? "has changed" : "has not changed");
		break;

	case CHG_IRQ_SYS_CTRL_I:
		ret = regmap_read(charger->regmap, REG_STAT_CHG_A, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_A\n");
			return;
		}

		val = (val & BIT_STAT_A_VSYSY_MIN_STAT) >>
					M2SH(BIT_STAT_A_VSYSY_MIN_STAT);
		dev_dbg(charger->dev,
			"CHG_INT_SYS_CTRL: The minimum system voltage regulation loop %s\n",
			val ? "has engaged" : "has not engaged");
		break;

	case CHG_IRQ_SYS_CNFG_I:
		ret = regmap_read(charger->regmap, REG_CNFG_CHG_G, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read CNFG_CHG_G\n");
			return;
		}

		val = (val & BITS_CNFG_G_CHG_CV) >> M2SH(BITS_CNFG_G_CHG_CV);
		dev_dbg(charger->dev, "CHG_INT_SYS_CNFG: CHG_VC = %02Xh\n", val);
		break;

	default:
		break;
	}

	/* notify psy changed */
	power_supply_changed(charger->psy_chg);

	return;
}

static irqreturn_t max77658_charger_isr(int irq, void *data)
{
	struct max77658_charger_dev *charger = data;

	charger->irq = irq;
	max77658_charger_update(charger);
	schedule_delayed_work(&charger->irq_work, IRQ_WORK_DELAY);

	return IRQ_HANDLED;
}

static void max77658_charger_irq_work(struct work_struct *work)
{
	struct max77658_charger_dev *charger =
		container_of(work, struct max77658_charger_dev, irq_work.work);

	mutex_lock(&charger->lock);
	max77658_do_irq(charger, charger->irq - charger->irq_arr[0]);
	mutex_unlock(&charger->lock);
}

static struct max77658_irq_desc {
	const char *name;
} max77658_irq_descs[] = {
	{ "charger-thm" },
	{ "charger-chg" },
	{ "charger-chgin" },
	{ "charger-tj-reg" },
	{ "charger-chgin-ctrl" },
	{ "charger-sys-ctrl" },
	{ "charger-sys-cnfg" },
};

static int max77658_charger_probe(struct platform_device *pdev)
{
	struct max77658_dev *max77658 = dev_get_drvdata(pdev->dev.parent);
	struct max77658_charger_dev *charger;
	int ret = 0;
	int i;
	struct power_supply_config charger_cfg = {};

	charger = devm_kzalloc(&pdev->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return dev_err_probe(&pdev->dev, -ENOMEM,
				     "Memory allocation failed\n");

	mutex_init(&charger->lock);

	charger->dev = &pdev->dev;
	charger->regmap = max77658->regmap_pmic;

	ret = max77658_charger_parse_dt(charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Not found charger dt!\n");

	platform_set_drvdata(pdev, charger);
	charger->psy_chg_d.name = MAX77658_CHARGER_NAME;
	charger->psy_chg_d.type	= POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg_d.get_property	= max77658_charger_get_property;
	charger->psy_chg_d.set_property	= max77658_charger_set_property;
	charger->psy_chg_d.properties = max77658_charger_battery_props;
	charger->psy_chg_d.property_is_writeable = max77658_property_is_writeable;
	charger->psy_chg_d.num_properties =
		ARRAY_SIZE(max77658_charger_battery_props);
	charger_cfg.drv_data = charger;
	charger_cfg.supplied_to = chg_supplied_to;
	charger_cfg.of_node = pdev->dev.of_node;
	charger_cfg.num_supplicants = ARRAY_SIZE(chg_supplied_to);

	charger->psy_chg = devm_power_supply_register(&pdev->dev,
						      &charger->psy_chg_d,
						      &charger_cfg);
	if (IS_ERR(charger->psy_chg))
		return dev_err_probe(&pdev->dev, PTR_ERR(charger->psy_chg),
				     "Failed to register power supply\n");

	ret = max77658_charger_initialize(charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to initialize charger\n");

	INIT_DELAYED_WORK(&charger->irq_work, max77658_charger_irq_work);

	for (i = 0; i < CHG_IRQ_MAX; i++) {
		charger->irq_arr[i] = regmap_irq_get_virq(max77658->irqc_chg, i);

		if (charger->irq_arr[i] < 0) {
			dev_err(&pdev->dev, "Failed to get virq for CHG_IRQ%d\n", i);
			continue;
		}

		ret = devm_request_threaded_irq(&pdev->dev, charger->irq_arr[i],
						NULL, max77658_charger_isr,
						IRQF_TRIGGER_FALLING,
						max77658_irq_descs[i].name, charger);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "Failed to request irq: %d\n",
					     charger->irq_arr[i]);
	}

	return 0;
}

static int max77658_charger_remove(struct platform_device *pdev)
{
	struct max77658_charger_dev *charger = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&charger->irq_work);

	return 0;
}

static const struct platform_device_id max77658_charger_id[] = {
	{ MAX77658_CHARGER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, max77658_charger_id);

static struct platform_driver max77658_charger_driver = {
	.driver = {
		.name = MAX77658_CHARGER_NAME,
	},
	.probe = max77658_charger_probe,
	.remove = max77658_charger_remove,
	.id_table = max77658_charger_id,
};

module_platform_driver(max77658_charger_driver);

MODULE_DESCRIPTION("MAX77658 Charger Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com ");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
