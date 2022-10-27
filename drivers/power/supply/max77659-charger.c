// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77659.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define MAX77659_IRQ_WORK_DELAY 0
#define MAX77659_CHARGER_CURRENT_MAX 300000
#define MAX77659_CHARGER_CURRENT_MIN 7500
#define MAX77659_CHARGER_CURRENT_STEP 7500
#define MAX77659_CHARGER_TOPOFF_TIMER_STEP 5

#define MAX77659_REG_STAT_CHG_A 0x02
#define MAX77659_REG_STAT_CHG_B 0x03
#define MAX77659_REG_CNFG_CHG_A 0x20
#define MAX77659_REG_CNFG_CHG_B 0x21
#define MAX77659_REG_CNFG_CHG_C 0x22
#define MAX77659_REG_CNFG_CHG_D 0x23
#define MAX77659_REG_CNFG_CHG_E 0x24
#define MAX77659_REG_CNFG_CHG_F 0x25
#define MAX77659_REG_CNFG_CHG_G 0x26
#define MAX77659_REG_CNFG_CHG_H 0x27
#define MAX77659_REG_CNFG_CHG_I 0x28

#define MAX77659_BIT_STAT_A_VSYSY_MIN_STAT	BIT(4)
#define MAX77659_BIT_STAT_A_TJ_REG_STAT		BIT(3)
#define MAX77659_BIT_STAT_A_THM_DTLS		GENMASK(2, 0)
#define MAX77659_BIT_STAT_B_CHG_DTLS		GENMASK(7, 4)
#define MAX77659_BIT_STAT_B_CHGIN_DTSL		GENMASK(3, 2)
#define MAX77659_BIT_STAT_B_CHG			BIT(1)
#define MAX77659_BIT_CNFG_B_CHG_EN		BIT(0)
#define MAX77659_BIT_CNFG_C_TOPOFFTIMER		GENMASK(2, 0)
#define MAX77659_BIT_CNFG_E_CC			GENMASK(7, 2)
#define MAX77659_BIT_CNFG_E_TFASTCHG		GENMASK(1, 0)
#define MAX77659_BIT_CNFG_G_CHG_CV		GENMASK(7, 2)

enum {
	MAX77659_CHG_DTLS_OFF,
	MAX77659_CHG_DTLS_PREQUAL,
	MAX77659_CHG_DTLS_FASTCHARGE_CC,
	MAX77659_CHG_DTLS_JEITA_FASTCHARGE_CC,
	MAX77659_CHG_DTLS_FASTCHARGE_CV,
	MAX77659_CHG_DTLS_JEITA_FASTCHARGE_CV,
	MAX77659_CHG_DTLS_TOPOFF,
	MAX77659_CHG_DTLS_JEITA_TOPOFF,
	MAX77659_CHG_DTLS_DONE,
	MAX77659_CHG_DTLS_JEITA_DONE,
	MAX77659_CHG_DTLS_OFF_TIMER_FAULT,
	MAX77659_CHG_DTLS_OFF_CHARGE_TIMER_FAULT,
	MAX77659_CHG_DTLS_OFF_BATTERY_TEMP_FAULT,
	MAX77659_CHG_DTLS_RESERVED_13,
};

enum {
	MAX77659_CHG_IRQ_THM_I,
	MAX77659_CHG_IRQ_CHG_I,
	MAX77659_CHG_IRQ_CHGIN_I,
	MAX77659_CHG_IRQ_TJ_REG_I,
	MAX77659_CHG_IRQ_SYS_CTRL_I,

	MAX77659_CHG_IRQ_MAX,
};

struct max77659_charger_dev {
	struct device *dev;
	struct max77659_dev *max77659;
	struct regmap *regmap;

	struct power_supply *psy_chg;
	struct power_supply_desc psy_chg_d;

	int irq;
	int irq_arr[MAX77659_CHG_IRQ_MAX];
	int irq_mask;

	struct delayed_work irq_work;
	/* mutex for charger operations*/
	struct mutex lock;

	int present;
	int health;
	int status;
	int charge_type;

	unsigned int fast_charge_current_ua;
	unsigned int fast_charge_timer_hr;
	unsigned int topoff_timer_min;
};

static char *chg_supplied_to[] = {
	MAX77659_CHARGER_NAME,
};

static int max77659_set_charge_current(struct max77659_charger_dev *charger,
				       int fast_charge_current_ua)
{
	unsigned int reg_data;

	fast_charge_current_ua = clamp_val(fast_charge_current_ua, MAX77659_CHARGER_CURRENT_MIN,
					   MAX77659_CHARGER_CURRENT_MAX);
	reg_data = fast_charge_current_ua / MAX77659_CHARGER_CURRENT_STEP - 1;
	reg_data = FIELD_PREP(MAX77659_BIT_CNFG_E_CC, reg_data);

	return regmap_update_bits(charger->regmap, MAX77659_REG_CNFG_CHG_E,
				  MAX77659_BIT_CNFG_E_CC, reg_data);
}

static int max77659_get_charge_current(struct max77659_charger_dev *charger,
				       int *get_current)
{
	struct regmap *regmap = charger->regmap;
	unsigned int reg_data, current_val;
	int ret;

	ret = regmap_read(regmap, MAX77659_REG_CNFG_CHG_E, &reg_data);
	if (ret)
		return ret;

	reg_data = FIELD_GET(MAX77659_BIT_CNFG_E_CC, reg_data);
	current_val = (reg_data + 1) * MAX77659_CHARGER_CURRENT_STEP;

	*get_current = clamp_val(current_val, MAX77659_CHARGER_CURRENT_MIN,
				 MAX77659_CHARGER_CURRENT_MAX);

	return 0;
}

static int max77659_set_topoff_timer(struct max77659_charger_dev *charger, int termination_time_min)
{
	unsigned int reg_data;

	termination_time_min = clamp_val(termination_time_min, 0, MAX77659_BIT_CNFG_C_TOPOFFTIMER
					 * MAX77659_CHARGER_TOPOFF_TIMER_STEP);
	reg_data = FIELD_PREP(MAX77659_BIT_CNFG_C_TOPOFFTIMER,
			      termination_time_min / MAX77659_CHARGER_TOPOFF_TIMER_STEP);

	return regmap_update_bits(charger->regmap, MAX77659_REG_CNFG_CHG_C,
				  MAX77659_BIT_CNFG_C_TOPOFFTIMER, reg_data);
}

static int max77659_charger_initialize(struct max77659_charger_dev *charger)
{
	int ret;
	unsigned int val;

	val = (MAX77659_BIT_INT_SYS_CTRL | MAX77659_BIT_INT_TJ_REG | MAX77659_BIT_INT_CHGIN |
	       MAX77659_BIT_INT_CHG | MAX77659_BIT_INT_THM);

	ret = regmap_write(charger->regmap, MAX77659_REG_INT_M_CHG, val);
	if (ret)
		return dev_err_probe(charger->dev, ret, "Error in writing register INT_M_CHG\n");

	ret = max77659_set_charge_current(charger, charger->fast_charge_current_ua);
	if (ret)
		return dev_err_probe(charger->dev, ret, "Error in writing register CNFG_CHG_C\n");

	if (charger->fast_charge_timer_hr == 0 || charger->fast_charge_timer_hr > 7)
		val = 0x00;
	else if (charger->fast_charge_timer_hr < 3)
		val = 0x01;
	else
		val = (int)DIV_ROUND_UP(charger->fast_charge_timer_hr - 1, 2);

	ret = regmap_update_bits(charger->regmap, MAX77659_REG_CNFG_CHG_E,
				 MAX77659_BIT_CNFG_E_TFASTCHG, val);
	if (ret)
		return dev_err_probe(charger->dev, ret, "Error in writing register CNFG_CHG_E\n");

	return max77659_set_topoff_timer(charger, charger->topoff_timer_min);
}

static void max77659_charger_parse_dt(struct max77659_charger_dev *charger)
{
	int ret;

	ret = device_property_read_u32(charger->dev, "fast-charge-timer",
				       &charger->fast_charge_timer_hr);
	if (ret)
		dev_dbg(charger->dev, "Could not read fast-charge-timer DT property\n");

	ret = device_property_read_u32(charger->dev, "fast-charge-current",
				       &charger->fast_charge_current_ua);
	if (ret)
		dev_dbg(charger->dev, "Could not read fast-charge-current DT property\n");

	ret = device_property_read_u32(charger->dev, "topoff-timer", &charger->topoff_timer_min);
	if (ret)
		dev_dbg(charger->dev, "Could not read topoff-timer DT property\n");
}

struct max77659_charger_status_map {
	int health;
	int status;
	int charge_type;
};

#define STATUS_MAP(_MAX77659_CHG_DTLS, _health, _status, _charge_type) \
	[MAX77659_CHG_DTLS_##_MAX77659_CHG_DTLS] = {\
		.health = POWER_SUPPLY_HEALTH_##_health,\
		.status = POWER_SUPPLY_STATUS_##_status,\
		.charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type,\
	}

static struct max77659_charger_status_map max77659_charger_status_map[] = {
	/* chg_details_xx, health, status, charge_type */
	STATUS_MAP(OFF, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(PREQUAL, GOOD, CHARGING, TRICKLE),
	STATUS_MAP(FASTCHARGE_CC, GOOD, CHARGING, FAST),
	STATUS_MAP(JEITA_FASTCHARGE_CC, GOOD, CHARGING, FAST),
	STATUS_MAP(FASTCHARGE_CV, GOOD, CHARGING, FAST),
	STATUS_MAP(JEITA_FASTCHARGE_CV, GOOD, CHARGING, FAST),
	STATUS_MAP(TOPOFF, GOOD, CHARGING, FAST),
	STATUS_MAP(JEITA_TOPOFF, GOOD, CHARGING, FAST),
	STATUS_MAP(DONE, GOOD, FULL, NONE),
	STATUS_MAP(JEITA_DONE, GOOD, FULL, NONE),
	STATUS_MAP(OFF_TIMER_FAULT, SAFETY_TIMER_EXPIRE, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_CHARGE_TIMER_FAULT, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_BATTERY_TEMP_FAULT, HOT, NOT_CHARGING, NONE),
};

static int max77659_charger_update(struct max77659_charger_dev *charger)
{
	unsigned int stat_chg_b, chg_dtls;
	int ret;

	ret = regmap_read(charger->regmap, MAX77659_REG_STAT_CHG_B, &stat_chg_b);
	if (ret) {
		charger->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		charger->status = POWER_SUPPLY_STATUS_UNKNOWN;
		charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		return ret;
	}

	charger->present = stat_chg_b & MAX77659_BIT_STAT_B_CHG;
	if (!charger->present) {
		charger->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		charger->status = POWER_SUPPLY_STATUS_DISCHARGING;
		charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		return 0;
	}

	chg_dtls = FIELD_GET(MAX77659_BIT_STAT_B_CHG_DTLS, stat_chg_b);

	charger->health = max77659_charger_status_map[chg_dtls].health;
	charger->status = max77659_charger_status_map[chg_dtls].status;
	charger->charge_type = max77659_charger_status_map[chg_dtls].charge_type;

	return 0;
}

static enum power_supply_property max77659_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW
};

static int max77659_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return 1;
	default:
		return 0;
	}
}

static int max77659_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max77659_charger_dev *charger = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&charger->lock);

	ret = max77659_charger_update(charger);
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
		ret = max77659_get_charge_current(charger, &val->intval);
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&charger->lock);

	return ret;
}

static int max77659_charger_set_property(struct power_supply *psy, enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct max77659_charger_dev *charger = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&charger->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_update_bits(charger->regmap, MAX77659_REG_CNFG_CHG_B,
					 MAX77659_BIT_CNFG_B_CHG_EN, !!val->intval);
		if (ret)
			goto out;

		ret = max77659_set_charge_current(charger, charger->fast_charge_current_ua);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* val->intval - uA */
		ret = max77659_set_charge_current(charger, val->intval);
		if (ret)
			goto out;

		charger->fast_charge_current_ua = val->intval;
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&charger->lock);

	return ret;
}

static void max77659_charger_irq_handler(struct max77659_charger_dev *charger, int irq)
{
	unsigned int val, stat_chg_b;
	int chg_present;
	int ret;

	switch (irq) {
	case MAX77659_CHG_IRQ_THM_I:
		ret = regmap_read(charger->regmap, MAX77659_REG_STAT_CHG_A, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_A\n");
			return;
		}

		val = FIELD_GET(MAX77659_BIT_STAT_A_THM_DTLS, val);
		dev_dbg(charger->dev, "CHG_INT_THM: thm_dtls = %02Xh\n", val);
		break;

	case MAX77659_CHG_IRQ_CHG_I:
		ret = regmap_read(charger->regmap, MAX77659_REG_STAT_CHG_B, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_B\n");
			return;
		}

		val = FIELD_GET(MAX77659_BIT_STAT_B_CHG_DTLS, val);
		dev_dbg(charger->dev, "CHG_INT_CHG: MAX77659_CHG_DTLS = %02Xh\n", val);
		break;

	case MAX77659_CHG_IRQ_CHGIN_I:
		ret = regmap_read(charger->regmap, MAX77659_REG_STAT_CHG_B, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_B\n");
			return;
		}

		val = FIELD_GET(MAX77659_BIT_STAT_B_CHG_DTLS, val);
		dev_dbg(charger->dev, "CHG_INT_CHG: MAX77659_CHG_DTLS = %02Xh\n", val);

		ret = regmap_read(charger->regmap, MAX77659_REG_STAT_CHG_B, &stat_chg_b);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_B\n");
			return;
		}

		chg_present = stat_chg_b & MAX77659_BIT_STAT_B_CHG;

		regmap_update_bits(charger->regmap, MAX77659_REG_CNFG_CHG_B,
				   MAX77659_BIT_CNFG_B_CHG_EN, !!chg_present);
		if (!chg_present)
			max77659_set_charge_current(charger, charger->fast_charge_current_ua);

		dev_dbg(charger->dev, "CHG_INT_CHGIN: Charger input %s\n",
			chg_present ? "inserted" : "removed");
		break;

	case MAX77659_CHG_IRQ_TJ_REG_I:
		ret = regmap_read(charger->regmap, MAX77659_REG_STAT_CHG_A, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_A\n");
			return;
		}

		val = FIELD_GET(MAX77659_BIT_STAT_A_TJ_REG_STAT, val);
		dev_dbg(charger->dev, "CHG_INT_TJ_REG: tj_reg_stat = %02Xh\n", val);
		dev_dbg(charger->dev, "CHG_INT_TJ_REG: Die temperature %s Tj-reg\n",
			val ? "has exceeded" : "has not exceeded");
		break;

	case MAX77659_CHG_IRQ_SYS_CTRL_I:
		ret = regmap_read(charger->regmap, MAX77659_REG_STAT_CHG_A, &val);
		if (ret) {
			dev_err(charger->dev, "Failed to read STAT_CHG_A\n");
			return;
		}

		val = FIELD_GET(MAX77659_BIT_STAT_A_VSYSY_MIN_STAT, val);
		dev_dbg(charger->dev,
			"CHG_INT_SYS_CTRL: The minimum system voltage regulation loop %s\n",
			 val ? "has engaged" : "has not engaged");
		break;

	default:
		break;
	}

	power_supply_changed(charger->psy_chg);
}

static irqreturn_t max77659_charger_isr(int irq, void *data)
{
	struct max77659_charger_dev *charger = data;

	charger->irq = irq;
	max77659_charger_update(charger);
	max77659_charger_irq_handler(charger, charger->irq - charger->irq_arr[0]);

	return IRQ_HANDLED;
}

static const char * const max77659_irq_descs[] = {
	"charger-thm",
	"charger-chg",
	"charger-chgin",
	"charger-tj-reg",
	"charger-sys-ctrl",
};

static int max77659_charger_probe(struct platform_device *pdev)
{
	struct max77659_dev *max77659 = dev_get_drvdata(pdev->dev.parent);
	struct max77659_charger_dev *charger;
	struct device *dev = &pdev->dev;
	int i, ret;
	struct power_supply_config charger_cfg = {};

	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	mutex_init(&charger->lock);

	charger->dev = dev;
	charger->regmap = max77659->regmap;
	charger->fast_charge_current_ua = 15000;
	charger->topoff_timer_min = 30;

	max77659_charger_parse_dt(charger);

	platform_set_drvdata(pdev, charger);
	charger->psy_chg_d.name = MAX77659_CHARGER_NAME;
	charger->psy_chg_d.type = POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg_d.get_property	= max77659_charger_get_property;
	charger->psy_chg_d.set_property	= max77659_charger_set_property;
	charger->psy_chg_d.properties = max77659_charger_props;
	charger->psy_chg_d.property_is_writeable = max77659_property_is_writeable;
	charger->psy_chg_d.num_properties = ARRAY_SIZE(max77659_charger_props);
	charger_cfg.drv_data = charger;
	charger_cfg.supplied_to = chg_supplied_to;
	charger_cfg.of_node = dev->of_node;
	charger_cfg.num_supplicants = ARRAY_SIZE(chg_supplied_to);

	ret = max77659_charger_initialize(charger);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize charger\n");

	for (i = 0; i < MAX77659_CHG_IRQ_MAX; i++) {
		charger->irq_arr[i] = regmap_irq_get_virq(max77659->irqc_chg, i);

		if (charger->irq_arr[i] < 0) {
			return dev_err_probe(dev, -EINVAL,
					     "Invalid IRQ for MAX77659_CHG_IRQ %d\n", i);
		}

		ret = devm_request_threaded_irq(dev, charger->irq_arr[i],
						NULL, max77659_charger_isr, IRQF_TRIGGER_FALLING,
						max77659_irq_descs[i], charger);
		if (ret) {
			return dev_err_probe(dev, ret, "Failed to request irq: %d\n",
					     charger->irq_arr[i]);
		}
	}

	charger->psy_chg = devm_power_supply_register(dev,
						      &charger->psy_chg_d, &charger_cfg);
	if (IS_ERR(charger->psy_chg)) {
		return dev_err_probe(dev, PTR_ERR(charger->psy_chg),
				     "Failed to register power supply\n");
	}

	return 0;
}

static const struct of_device_id max77659_charger_of_id[] = {
	{ .compatible = "adi,max77659-charger" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max77659_charger_of_id);

static const struct platform_device_id max77659_charger_id[] = {
	{ MAX77659_CHARGER_NAME, 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, max77659_charger_id);

static struct platform_driver max77659_charger_driver = {
	.driver = {
		.name = MAX77659_CHARGER_NAME,
		.of_match_table = of_match_ptr(max77659_charger_of_id),
	},
	.probe = max77659_charger_probe,
	.id_table = max77659_charger_id,
};

module_platform_driver(max77659_charger_driver);

MODULE_DESCRIPTION("MAX77659 Charger Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com, Zeynep.Arslanbenzer@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
