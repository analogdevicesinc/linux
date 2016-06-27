/*
 * pf1550.c - regulator driver for the PF1550
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
 *
 * This driver is based on pfuze100-regulator.c
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/mfd/pf1550.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/platform_device.h>

struct pf1550_desc {
	struct regulator_desc desc;
	unsigned char stby_reg;
	unsigned char stby_mask;
};

struct pf1550_regulator_info {
	struct device *dev;
	struct pf1550_dev *pf1550;
	int irq;
};

static struct pf1550_irq_info pf1550_regulator_irqs[] = {
	{ PF1550_PMIC_IRQ_SW1_LS,		"sw1-lowside" },
	{ PF1550_PMIC_IRQ_SW2_LS,		"sw2-lowside" },
	{ PF1550_PMIC_IRQ_SW3_LS,		"sw3-lowside" },

	{ PF1550_PMIC_IRQ_SW1_HS,		"sw1-highside" },
	{ PF1550_PMIC_IRQ_SW2_HS,		"sw2-highside" },
	{ PF1550_PMIC_IRQ_SW3_HS,		"sw3-highside" },

	{ PF1550_PMIC_IRQ_LDO1_FAULT,		"ldo1-fault" },
	{ PF1550_PMIC_IRQ_LDO2_FAULT,		"ldo2-fault" },
	{ PF1550_PMIC_IRQ_LDO3_FAULT,		"ldo3-fault" },

	{ PF1550_PMIC_IRQ_TEMP_110,		"temp-110" },
	{ PF1550_PMIC_IRQ_TEMP_125,		"temp-125" },
};

static const int pf1550_sw12_volts[] = {
	1100000, 1200000, 1350000, 1500000, 1800000, 2500000, 3000000, 3300000,
};

static const int pf1550_ldo13_volts[] = {
	750000, 800000, 850000, 900000, 950000, 1000000, 1050000, 1100000,
	1150000, 1200000, 1250000, 1300000, 1350000, 1400000, 1450000, 1500000,
	1800000, 1900000, 2000000, 2100000, 2200000, 2300000, 2400000, 2500000,
	2600000, 2700000, 2800000, 2900000, 3000000, 3100000, 3200000, 3300000,
};

static int pf1550_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	int id = rdev_get_id(rdev);
	unsigned int ramp_bits;
	int ret;

	if (id < PF1550_VREFDDR) {
		ramp_delay = 6250 / ramp_delay;
		ramp_bits = ramp_delay >> 1;
		ret = regmap_update_bits(rdev->regmap,
					 rdev->desc->vsel_reg + 4,
					 0x10, ramp_bits << 4);
		if (ret < 0)
			dev_err(&rdev->dev, "ramp failed, err %d\n", ret);
	} else
		ret = -EACCES;

	return ret;
}

static struct regulator_ops pf1550_sw_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = pf1550_set_ramp_delay,
};

static struct regulator_ops pf1550_ldo1_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_ascend,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops pf1550_ldo2_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops pf1550_fixed_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

#define PF_VREF(_chip, _name, voltage)	{	\
	.desc = {	\
		.name = #_name,	\
		.of_match = of_match_ptr(#_name),	\
		.regulators_node = of_match_ptr("regulators"),	\
		.n_voltages = 1,	\
		.ops = &pf1550_fixed_ops,	\
		.type = REGULATOR_VOLTAGE,	\
		.id = _chip ## _ ## _name,	\
		.owner = THIS_MODULE,	\
		.min_uV = (voltage),	\
		.enable_reg = _chip ## _PMIC_REG_ ## _name ## _CTRL, \
		.enable_mask = 0x1,	\
	},	\
	.stby_reg = _chip ## _PMIC_REG_ ## _name ## _CTRL, \
	.stby_mask = 0x2,	\
}

#define PF_SW1(_chip, _name, mask, voltages)	{	\
	.desc = {	\
		.name = #_name,	\
		.of_match = of_match_ptr(#_name),	\
		.regulators_node = of_match_ptr("regulators"),	\
		.n_voltages = ARRAY_SIZE(voltages),	\
		.ops = &pf1550_sw_ops,	\
		.type = REGULATOR_VOLTAGE,	\
		.id = _chip ## _ ## _name,	\
		.owner = THIS_MODULE,	\
		.volt_table = voltages,	\
		.vsel_reg = _chip ## _PMIC_REG_ ## _name ## _VOLT, \
		.vsel_mask = (mask),	\
	},	\
	.stby_reg = _chip ## _PMIC_REG_ ## _name ## _STBY_VOLT,	\
	.stby_mask = (mask),	\
}

#define PF_SW3(_chip, _name, min, max, mask, step)	{	\
	.desc = {	\
		.name = #_name,	\
		.of_match = of_match_ptr(#_name),	\
		.regulators_node = of_match_ptr("regulators"),	\
		.n_voltages = ((max) - (min)) / (step) + 1,	\
		.ops = &pf1550_sw_ops,	\
		.type = REGULATOR_VOLTAGE,	\
		.id = _chip ## _ ## _name,	\
		.owner = THIS_MODULE,	\
		.min_uV = (min),	\
		.uV_step = (step),	\
		.vsel_reg = _chip ## _PMIC_REG_ ## _name ## _VOLT, \
		.vsel_mask = (mask),	\
	},	\
	.stby_reg = _chip ## _PMIC_REG_ ## _name ## _STBY_VOLT,	\
	.stby_mask = (mask),	\
}

#define PF_LDO1(_chip, _name, mask, voltages)	{	\
	.desc = {	\
		.name = #_name,	\
		.of_match = of_match_ptr(#_name),	\
		.regulators_node = of_match_ptr("regulators"),	\
		.n_voltages = ARRAY_SIZE(voltages),	\
		.ops = &pf1550_ldo1_ops,	\
		.type = REGULATOR_VOLTAGE,	\
		.id = _chip ## _ ## _name,	\
		.owner = THIS_MODULE,	\
		.volt_table = voltages, \
		.vsel_reg = _chip ## _PMIC_REG_ ## _name ## _VOLT, \
		.vsel_mask = (mask),	\
		.enable_reg = _chip ## _PMIC_REG_ ## _name ## _CTRL, \
		.enable_mask = 0x1,	\
	},	\
	.stby_reg = _chip ## _PMIC_REG_ ## _name ## _CTRL, \
	.stby_mask = 0x2,	\
}

#define PF_LDO2(_chip, _name, mask, min, max, step)	{	\
	.desc = {	\
		.name = #_name,	\
		.of_match = of_match_ptr(#_name),	\
		.regulators_node = of_match_ptr("regulators"),	\
		.n_voltages = ((max) - (min)) / (step) + 1,	\
		.ops = &pf1550_ldo2_ops,	\
		.type = REGULATOR_VOLTAGE,	\
		.id = _chip ## _ ## _name,	\
		.owner = THIS_MODULE,	\
		.min_uV = (min),	\
		.uV_step = (step),	\
		.vsel_reg = _chip ## _PMIC_REG_ ## _name ## _VOLT, \
		.vsel_mask = (mask),	\
		.enable_reg = _chip ## _PMIC_REG_ ## _name ## _CTRL, \
		.enable_mask = 0x1,	\
	},	\
	.stby_reg = _chip ## _PMIC_REG_ ## _name ## _CTRL, \
	.stby_mask = 0x2,	\
}

static const struct pf1550_desc pf1550_regulators[] = {
	PF_SW3(PF1550, SW1, 600000, 1387500, 0x3f, 12500),
	PF_SW3(PF1550, SW2, 600000, 1387500, 0x3f, 12500),
	PF_SW3(PF1550, SW3, 1800000, 3300000, 0xf, 100000),
	PF_VREF(PF1550, VREFDDR, 1200000),
	PF_LDO1(PF1550, LDO1, 0x1f, pf1550_ldo13_volts),
	PF_LDO2(PF1550, LDO2, 0xf, 1800000, 3300000, 100000),
	PF_LDO1(PF1550, LDO3, 0x1f, pf1550_ldo13_volts),
};

static irqreturn_t pf1550_regulator_irq_handler(int irq, void *data)
{
	struct pf1550_regulator_info *info = data;
	int i, irq_type = -1;

	info->irq = irq;

	for (i = 0; i < ARRAY_SIZE(pf1550_regulator_irqs); i++)
		if (info->irq == pf1550_regulator_irqs[i].virq)
			irq_type = pf1550_regulator_irqs[i].irq;

	switch (irq_type) {
	case PF1550_PMIC_IRQ_SW1_LS:
	case PF1550_PMIC_IRQ_SW2_LS:
	case PF1550_PMIC_IRQ_SW3_LS:
		dev_info(info->dev, "lowside interrupt trigged! irq_type=%d\n",
				irq_type);
		break;
	case PF1550_PMIC_IRQ_SW1_HS:
	case PF1550_PMIC_IRQ_SW2_HS:
	case PF1550_PMIC_IRQ_SW3_HS:
		dev_info(info->dev, "highside interrupt triggered! irq_type=%d\n",
				irq_type);
		break;
	case PF1550_PMIC_IRQ_LDO1_FAULT:
	case PF1550_PMIC_IRQ_LDO2_FAULT:
	case PF1550_PMIC_IRQ_LDO3_FAULT:
		dev_info(info->dev, "ldo fault triggered! irq_type=%d\n",
				irq_type);
		break;
	case PF1550_PMIC_IRQ_TEMP_110:
	case PF1550_PMIC_IRQ_TEMP_125:
		dev_info(info->dev, "thermal exception triggered! irq_type=%d\n",
				irq_type);
		break;
	default:
		dev_err(info->dev, "regulator interrupt: irq %d occurred\n",
				irq_type);
	}

	return IRQ_HANDLED;
}

static int pf1550_regulator_probe(struct platform_device *pdev)
{
	struct pf1550_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct pf1550_regulator_info *info;
	int i, ret = 0;
	struct regulator_config config = { };

	info = devm_kzalloc(&pdev->dev, sizeof(struct pf1550_regulator_info),
				   GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	config.dev = iodev->dev;
	config.regmap = iodev->regmap;

	for (i = 0; i < ARRAY_SIZE(pf1550_regulators); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(&pdev->dev,
					&pf1550_regulators[i].desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"Failed to initialize regulator-%d\n", i);
			return PTR_ERR(rdev);
		}
	}

	info->dev = &pdev->dev;
	info->pf1550 = iodev;

	platform_set_drvdata(pdev, info);

	for (i = 0; i < ARRAY_SIZE(pf1550_regulator_irqs); i++) {
		struct pf1550_irq_info *regulator_irq =
						&pf1550_regulator_irqs[i];
		unsigned int virq = 0;

		virq = regmap_irq_get_virq(iodev->irq_data_regulator,
					regulator_irq->irq);

		if (!virq)
			return -EINVAL;
		regulator_irq->virq = virq;

		ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
					pf1550_regulator_irq_handler,
					IRQF_NO_SUSPEND,
					regulator_irq->name, info);
		if (ret) {
			dev_err(&pdev->dev,
				"failed: irq request (IRQ: %d, error :%d)\n",
				regulator_irq->irq, ret);
			return ret;
		}
	}

	/* unmask all exception interrupts for regulators */
	regmap_write(info->pf1550->regmap, PF1550_PMIC_REG_SW_INT_MASK0, 0);
	regmap_write(info->pf1550->regmap, PF1550_PMIC_REG_SW_INT_MASK1, 0);
	regmap_write(info->pf1550->regmap, PF1550_PMIC_REG_LDO_INT_MASK0, 0);
	regmap_write(info->pf1550->regmap, PF1550_PMIC_REG_TEMP_INT_MASK0, 0);

	return 0;
}

static const struct platform_device_id pf1550_regulator_id[] = {
	{"pf1550-regulator", PF1550},
	{},
};

MODULE_DEVICE_TABLE(platform, pf1550_regulator_id);

static struct platform_driver pf1550_regulator_driver = {
	.driver = {
		   .name = "pf1550-regulator",
		   },
	.probe = pf1550_regulator_probe,
	.id_table = pf1550_regulator_id,
};

module_platform_driver(pf1550_regulator_driver);

MODULE_DESCRIPTION("Freescale PF1550 regulator driver");
MODULE_AUTHOR("Robin Gong <yibin.gong@freescale.com>");
MODULE_LICENSE("GPL");
