/*
 * pf1550_charger.c - regulator driver for the PF1550
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Robin Gong <yibin.gong@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/mfd/pf1550.h>

#define PF1550_CHARGER_NAME		"pf1550-charger"
#define PF1550_DEFAULT_CONSTANT_VOLT	4200000
#define PF1550_DEFAULT_MIN_SYSTEM_VOLT	3500000
#define PF1550_DEFAULT_THERMAL_TEMP	75

static const char *pf1550_charger_model		= "PF1550";
static const char *pf1550_charger_manufacturer	= "Freescale";

struct pf1550_charger {
	struct device *dev;
	struct pf1550_dev *pf1550;
	struct power_supply *charger;
	struct power_supply_desc psy_desc;
	int irq;
	struct delayed_work irq_work;
	struct mutex mutex;

	u32 constant_volt;
	u32 min_system_volt;
	u32 thermal_regulation_temp;
};

static struct pf1550_irq_info pf1550_charger_irqs[] = {
	{ PF1550_CHARG_IRQ_BAT2SOCI,		"BAT2SOC" },
	{ PF1550_CHARG_IRQ_BATI,		"BAT" },
	{ PF1550_CHARG_IRQ_CHGI,		"CHG" },
	{ PF1550_CHARG_IRQ_VBUSI,		"VBUS" },
	{ PF1550_CHARG_IRQ_DPMI,		"DPM" },
	{ PF1550_CHARG_IRQ_THMI,		"THM" },
};

static int pf1550_get_charger_state(struct regmap *regmap, int *val)
{
	int ret;
	unsigned int data;

	ret = regmap_read(regmap, PF1550_CHARG_REG_CHG_SNS, &data);
	if (ret < 0)
		return ret;

	data &= PF1550_CHG_SNS_MASK;

	switch (data) {
	case PF1550_CHG_PRECHARGE:
	case PF1550_CHG_CONSTANT_CURRENT:
	case PF1550_CHG_CONSTANT_VOL:
	case PF1550_CHG_EOC:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case PF1550_CHG_DONE:
		*val = POWER_SUPPLY_STATUS_FULL;
		break;
	case PF1550_CHG_TIMER_FAULT:
	case PF1550_CHG_SUSPEND:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case PF1550_CHG_OFF_INV:
	case PF1550_CHG_OFF_TEMP:
	case PF1550_CHG_LINEAR_ONLY:
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return 0;
}

static int pf1550_get_charge_type(struct regmap *regmap, int *val)
{
	int ret;
	unsigned int data;

	ret = regmap_read(regmap, PF1550_CHARG_REG_CHG_SNS, &data);
	if (ret < 0)
		return ret;

	data &= PF1550_CHG_SNS_MASK;

	switch (data) {
	case PF1550_CHG_SNS_MASK:
		*val = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case PF1550_CHG_CONSTANT_CURRENT:
	case PF1550_CHG_CONSTANT_VOL:
	case PF1550_CHG_EOC:
		*val = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case PF1550_CHG_DONE:
	case PF1550_CHG_TIMER_FAULT:
	case PF1550_CHG_SUSPEND:
	case PF1550_CHG_OFF_INV:
	case PF1550_CHG_BAT_OVER:
	case PF1550_CHG_OFF_TEMP:
	case PF1550_CHG_LINEAR_ONLY:
		*val = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		*val = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	return 0;
}

/*
 * Supported health statuses:
 *  - POWER_SUPPLY_HEALTH_DEAD
 *  - POWER_SUPPLY_HEALTH_GOOD
 *  - POWER_SUPPLY_HEALTH_OVERVOLTAGE
 *  - POWER_SUPPLY_HEALTH_UNKNOWN
 */
static int pf1550_get_battery_health(struct regmap *regmap, int *val)
{
	int ret;
	unsigned int data;

	ret = regmap_read(regmap, PF1550_CHARG_REG_BATT_SNS, &data);
	if (ret < 0)
		return ret;

	data &= PF1550_BAT_SNS_MASK;

	switch (data) {
	case PF1550_BAT_NO_DETECT:
		*val = POWER_SUPPLY_HEALTH_DEAD;
		break;
	case PF1550_BAT_NO_VBUS:
	case PF1550_BAT_LOW_THAN_PRECHARG:
	case PF1550_BAT_CHARG_FAIL:
	case PF1550_BAT_HIGH_THAN_PRECHARG:
		*val = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case PF1550_BAT_OVER_VOL:
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	default:
		*val = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	}

	return 0;
}

static int pf1550_get_present(struct regmap *regmap, int *val)
{
	unsigned int data;
	int ret;

	ret = regmap_read(regmap, PF1550_CHARG_REG_BATT_SNS, &data);
	if (ret < 0)
		return ret;

	data &= PF1550_BAT_SNS_MASK;
	*val = (data == PF1550_BAT_NO_DETECT) ? 0 : 1;

	return 0;
}

static int pf1550_get_online(struct regmap *regmap, int *val)
{
	unsigned int data;
	int ret;

	ret = regmap_read(regmap, PF1550_CHARG_REG_VBUS_SNS, &data);
	if (ret < 0)
		return ret;

	*val = (data & PF1550_VBUS_VALID) ? 1 : 0;

	return 0;
}

static void pf1550_chg_bat_isr(struct pf1550_charger *chg)
{
	unsigned int data;

	if (regmap_read(chg->pf1550->regmap, PF1550_CHARG_REG_BATT_SNS, &data)) {
		dev_err(chg->dev, "Read BATT_SNS error.\n");
		return;
	}

	switch (data & PF1550_BAT_SNS_MASK) {
	case PF1550_BAT_NO_VBUS:
		dev_dbg(chg->dev, "No valid VBUS input.\n");
		break;
	case PF1550_BAT_LOW_THAN_PRECHARG:
		dev_dbg(chg->dev, "VBAT < VPRECHG.LB.\n");
		break;
	case PF1550_BAT_CHARG_FAIL:
		dev_dbg(chg->dev, "Battery charging failed.\n");
		break;
	case PF1550_BAT_HIGH_THAN_PRECHARG:
		dev_dbg(chg->dev, "VBAT > VPRECHG.LB.\n");
		break;
	case PF1550_BAT_OVER_VOL:
		dev_dbg(chg->dev, "VBAT > VBATOV.\n");
		break;
	case PF1550_BAT_NO_DETECT:
		dev_dbg(chg->dev, "Battery not detected.\n");
		break;
	default:
		dev_err(chg->dev, "Unknown value read:%x\n",
			data & PF1550_CHG_SNS_MASK);
	}
}

static void pf1550_chg_chg_isr(struct pf1550_charger *chg)
{
	unsigned int data;

	if (regmap_read(chg->pf1550->regmap, PF1550_CHARG_REG_CHG_SNS, &data)) {
		dev_err(chg->dev, "Read CHG_SNS error.\n");
		return;
	}

	switch (data & PF1550_CHG_SNS_MASK) {
	case PF1550_CHG_PRECHARGE:
		dev_dbg(chg->dev, "In pre-charger mode.\n");
		break;
	case PF1550_CHG_CONSTANT_CURRENT:
		dev_dbg(chg->dev, "In fast-charge constant current mode.\n");
		break;
	case PF1550_CHG_CONSTANT_VOL:
		dev_dbg(chg->dev, "In fast-charge constant voltage mode.\n");
		break;
	case PF1550_CHG_EOC:
		dev_dbg(chg->dev, "In EOC mode.\n");
		break;
	case PF1550_CHG_DONE:
		dev_dbg(chg->dev, "In DONE mode.\n");
		break;
	case PF1550_CHG_TIMER_FAULT:
		dev_info(chg->dev, "In timer fault mode.\n");
		break;
	case PF1550_CHG_SUSPEND:
		dev_info(chg->dev, "In thermistor suspend mode.\n");
		break;
	case PF1550_CHG_OFF_INV:
		dev_info(chg->dev, "Input invalid, charger off.\n");
		break;
	case PF1550_CHG_BAT_OVER:
		dev_info(chg->dev, "Battery over-voltage.\n");
		break;
	case PF1550_CHG_OFF_TEMP:
		dev_info(chg->dev, "Temp high, charger off.\n");
		break;
	case PF1550_CHG_LINEAR_ONLY:
		dev_dbg(chg->dev, "In Linear mode, not charging.\n");
		break;
	default:
		dev_err(chg->dev, "Unknown value read:%x\n",
			data & PF1550_CHG_SNS_MASK);
	}
}

static void pf1550_chg_vbus_isr(struct pf1550_charger *chg)
{
	enum power_supply_type old_type;
	unsigned int data;

	if (regmap_read(chg->pf1550->regmap, PF1550_CHARG_REG_VBUS_SNS, &data)) {
		dev_err(chg->dev, "Read VBUS_SNS error.\n");
		return;
	}

	old_type = chg->psy_desc.type;

	if (data & PF1550_VBUS_UVLO) {
		chg->psy_desc.type = POWER_SUPPLY_TYPE_BATTERY;
		dev_dbg(chg->dev, "VBUS deattached.\n");
	}
	if (data & PF1550_VBUS_IN2SYS)
		dev_dbg(chg->dev, "VBUS_IN2SYS_SNS.\n");
	if (data & PF1550_VBUS_OVLO)
		dev_dbg(chg->dev, "VBUS_OVLO_SNS.\n");
	if (data & PF1550_VBUS_VALID) {
		chg->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
		dev_dbg(chg->dev, "VBUS attached.\n");
	}

	if (old_type != chg->psy_desc.type)
		power_supply_changed(chg->charger);
}

static irqreturn_t pf1550_charger_irq_handler(int irq, void *data)
{
	struct pf1550_charger *chg = data;

	chg->irq = irq;
	schedule_delayed_work(&chg->irq_work,  msecs_to_jiffies(10));

	return IRQ_HANDLED;
}

static void pf1550_charger_irq_work(struct work_struct *work)
{
	struct pf1550_charger *chg = container_of(to_delayed_work(work),
						  struct pf1550_charger,
						  irq_work);
	int i, irq_type = -1;

	if (!chg->charger)
		return;

	mutex_lock(&chg->mutex);

	for (i = 0; i < ARRAY_SIZE(pf1550_charger_irqs); i++)
		if (chg->irq == pf1550_charger_irqs[i].virq)
			irq_type = pf1550_charger_irqs[i].irq;

	switch (irq_type) {
	case PF1550_CHARG_IRQ_BAT2SOCI:
		dev_info(chg->dev, "BAT to SYS Overcurrent interrupt.\n");
		break;
	case PF1550_CHARG_IRQ_BATI:
		pf1550_chg_bat_isr(chg);
		break;
	case PF1550_CHARG_IRQ_CHGI:
		pf1550_chg_chg_isr(chg);
		break;
	case PF1550_CHARG_IRQ_VBUSI:
		pf1550_chg_vbus_isr(chg);
		break;
	case PF1550_CHARG_IRQ_DPMI:
		dev_info(chg->dev, "DPM interrupt.\n");
		break;
	case PF1550_CHARG_IRQ_THMI:
		dev_info(chg->dev, "Thermal interrupt.\n");
		break;
	default:
		dev_err(chg->dev, "unknown interrupt occurred.\n");
	}

	mutex_unlock(&chg->mutex);
}

static enum power_supply_property pf1550_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int pf1550_charger_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct pf1550_charger *chg = power_supply_get_drvdata(psy);
	struct regmap *regmap = chg->pf1550->regmap;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = pf1550_get_charger_state(regmap, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = pf1550_get_charge_type(regmap, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = pf1550_get_battery_health(regmap, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = pf1550_get_present(regmap, &val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = pf1550_get_online(regmap, &val->intval);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pf1550_charger_model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = pf1550_charger_manufacturer;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int pf1550_set_constant_volt(struct pf1550_charger *chg,
		unsigned int uvolt)
{
	unsigned int data;

	if (uvolt >= 3500000 && uvolt <= 4440000)
		data = 8 + (uvolt - 3500000) / 20000;
	else {
		dev_err(chg->dev, "Wrong value for constant voltage\n");
		return -EINVAL;
	}

	dev_dbg(chg->dev, "Charging constant voltage: %u (0x%x)\n", uvolt,
			data);

	return regmap_update_bits(chg->pf1550->regmap,
			PF1550_CHARG_REG_BATT_REG,
			PF1550_CHARG_REG_BATT_REG_CHGCV_MASK, data);
}

static int pf1550_set_min_system_volt(struct pf1550_charger *chg,
		unsigned int uvolt)
{
	unsigned int data;

	switch (uvolt) {
	case 3500000:
		data = 0x0;
		break;
	case 3700000:
		data = 0x1;
		break;
	case 4300000:
		data = 0x2;
		break;
	default:
		dev_err(chg->dev, "Wrong value for minimum system voltage\n");
		return -EINVAL;
	}

	data <<= PF1550_CHARG_REG_BATT_REG_VMINSYS_SHIFT;

	dev_dbg(chg->dev, "Minimum system regulation voltage: %u (0x%x)\n",
			uvolt, data);

	return regmap_update_bits(chg->pf1550->regmap,
			PF1550_CHARG_REG_BATT_REG,
			PF1550_CHARG_REG_BATT_REG_VMINSYS_MASK, data);
}

static int pf1550_set_thermal_regulation_temp(struct pf1550_charger *chg,
		unsigned int cels)
{
	unsigned int data;

	switch (cels) {
	case 60:
		data = 0x0;
		break;
	case 75:
		data = 0x1;
		break;
	case 90:
		data = 0x2;
		break;
	case 105:
		data = 0x3;
		break;
	default:
		dev_err(chg->dev, "Wrong value for thermal temperature\n");
		return -EINVAL;
	}

	data <<= PF1550_CHARG_REG_THM_REG_CNFG_REGTEMP_SHIFT;

	dev_dbg(chg->dev, "Thermal regulation loop temperature: %u (0x%x)\n",
			cels, data);

	return regmap_update_bits(chg->pf1550->regmap,
			PF1550_CHARG_REG_THM_REG_CNFG,
			PF1550_CHARG_REG_THM_REG_CNFG_REGTEMP_MASK, data);
}

/*
 * Sets charger registers to proper and safe default values.
 */
static int pf1550_reg_init(struct pf1550_charger *chg)
{
	int ret;
	unsigned int data;

	/* Unmask charger interrupt */
	ret =  regmap_write(chg->pf1550->regmap, PF1550_CHARG_REG_CHG_INT_MASK,
				0x11);
	if (ret) {
		dev_err(chg->dev, "Error unmask charger interrupt: %d\n", ret);
		return ret;
	}

	ret = regmap_read(chg->pf1550->regmap, PF1550_CHARG_REG_VBUS_SNS,
				&data);
	if (ret) {
		dev_err(chg->dev, "Read charg vbus_sns error: %d\n", ret);
		return ret;
	}

	if (data & PF1550_VBUS_VALID)
		chg->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;

	ret = pf1550_set_constant_volt(chg, chg->constant_volt);
	if (ret)
		return ret;

	ret = pf1550_set_min_system_volt(chg, chg->min_system_volt);
	if (ret)
		return ret;

	ret = pf1550_set_thermal_regulation_temp(chg,
			chg->thermal_regulation_temp);
	if (ret)
		return ret;

	return 0;
}

static int pf1550_dt_init(struct device *dev, struct pf1550_charger *chg)
{
	struct device_node *np = dev->of_node;

	if (!np) {
		dev_err(dev, "no charger OF node\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "fsl,constant-microvolt",
			&chg->constant_volt))
		chg->constant_volt = PF1550_DEFAULT_CONSTANT_VOLT;

	if (of_property_read_u32(np, "fsl,min-system-microvolt",
			&chg->min_system_volt))
		chg->min_system_volt = PF1550_DEFAULT_MIN_SYSTEM_VOLT;

	if (of_property_read_u32(np, "fsl,thermal-regulation",
			&chg->thermal_regulation_temp))
		chg->thermal_regulation_temp = PF1550_DEFAULT_THERMAL_TEMP;

	return 0;
}

static int pf1550_charger_probe(struct platform_device *pdev)
{
	struct pf1550_charger *chg;
	struct power_supply_config psy_cfg = {};
	struct pf1550_dev *pf1550 = dev_get_drvdata(pdev->dev.parent);
	int i, ret;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->pf1550 = pf1550;

	platform_set_drvdata(pdev, chg);

	ret = pf1550_dt_init(&pdev->dev, chg);
	if (ret)
		return ret;

	mutex_init(&chg->mutex);

	INIT_DELAYED_WORK(&chg->irq_work, pf1550_charger_irq_work);

	for (i = 0; i < ARRAY_SIZE(pf1550_charger_irqs); i++) {
		struct pf1550_irq_info *charger_irq =
						&pf1550_charger_irqs[i];
		unsigned int virq = 0;

		virq = regmap_irq_get_virq(pf1550->irq_data_charger,
					charger_irq->irq);
		if (!virq)
			return -EINVAL;

		charger_irq->virq = virq;

		ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
					pf1550_charger_irq_handler,
					IRQF_NO_SUSPEND,
					charger_irq->name, chg);
		if (ret) {
			dev_err(&pdev->dev,
				"failed: irq request (IRQ: %d, error :%d)\n",
				charger_irq->irq, ret);
			return ret;
		}
	}

	psy_cfg.drv_data = chg;

	chg->psy_desc.name = PF1550_CHARGER_NAME;
	chg->psy_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	chg->psy_desc.get_property = pf1550_charger_get_property;
	chg->psy_desc.properties = pf1550_charger_props;
	chg->psy_desc.num_properties = ARRAY_SIZE(pf1550_charger_props);

	chg->charger = power_supply_register(&pdev->dev, &chg->psy_desc,
						&psy_cfg);
	if (IS_ERR(chg->charger)) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		ret = PTR_ERR(chg->charger);
		return ret;
	}

	ret = pf1550_reg_init(chg);

	return ret;
}

static int pf1550_charger_remove(struct platform_device *pdev)
{
	struct pf1550_charger *chg = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&chg->irq_work);
	power_supply_unregister(chg->charger);

	return 0;
}

static const struct platform_device_id pf1550_charger_id[] = {
	{ "pf1550-charger", 0, },
	{ }
};
MODULE_DEVICE_TABLE(platform, pf1550_charger_id);

static struct platform_driver pf1550_charger_driver = {
	.driver = {
		.name	= "pf1550-charger",
	},
	.probe		= pf1550_charger_probe,
	.remove		= pf1550_charger_remove,
	.id_table	= pf1550_charger_id,
};
module_platform_driver(pf1550_charger_driver);

MODULE_AUTHOR("Robin Gong <yibin.gong@freescale.com>");
MODULE_DESCRIPTION("PF1550 charger driver");
MODULE_LICENSE("GPL");
