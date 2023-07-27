// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 BayLibre SAS
// Author: Bartosz Golaszewski <bgolaszewski@baylibre.com>
//
// Regulator driver for MAXIM 77650/77651 charger/power-supply.

#include <linux/of.h>
#include <linux/mfd/max77650.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MAX77650_REGULATOR_EN_CTRL_MASK		GENMASK(2, 0)
#define MAX77650_REGULATOR_EN_CTRL_BITS(_reg) \
		((_reg) & MAX77650_REGULATOR_EN_CTRL_MASK)
#define MAX77650_REGULATOR_ENABLED		GENMASK(2, 1)
#define MAX77650_REGULATOR_DISABLED		BIT(2)

#define MAX77650_REGULATOR_V_LDO_MASK		GENMASK(6, 0)
#define MAX77650_REGULATOR_V_SBB_MASK		GENMASK(5, 0)
#define MAX77651_REGULATOR_V_SBB1_MASK		GENMASK(5, 2)
#define MAX77651_REGULATOR_V_SBB1_RANGE_MASK	GENMASK(1, 0)

#define MAX77650_REGULATOR_AD_MASK		BIT(3)
#define MAX77650_REGULATOR_AD_DISABLED		0x00
#define MAX77650_REGULATOR_AD_ENABLED		BIT(3)

#define MAX77650_REGULATOR_CURR_LIM_MASK	GENMASK(7, 6)

#define MAX77658_REGULATOR_V_SBB_MASK		GENMASK(7, 0)
#define MAX77658_REGULATOR_CURR_LIM_MASK	GENMASK(5, 4)

#define MAX77654_REGULATOR_V_SBB_MASK		GENMASK(6, 0)

#define MAX77659_REGULATOR_CURR_LIM_SBB0_MASK	GENMASK(1, 0)
#define MAX77659_REGULATOR_CURR_LIM_SBB1_MASK	GENMASK(3, 2)
#define MAX77659_REGULATOR_CURR_LIM_SBB2_MASK	GENMASK(5, 4)

enum {
	MAX77643_REGULATOR_ID_LDO0 = 0,
	MAX77643_REGULATOR_ID_SBB0,
	MAX77643_REGULATOR_ID_SBB1,
	MAX77643_REGULATOR_ID_SBB2,
	MAX77643_REGULATOR_NUM_REGULATORS,
};

enum {
	MAX77650_REGULATOR_ID_LDO = 0,
	MAX77650_REGULATOR_ID_SBB0,
	MAX77650_REGULATOR_ID_SBB1,
	MAX77650_REGULATOR_ID_SBB2,
	MAX77650_REGULATOR_NUM_REGULATORS,
};

enum {
	MAX77654_REGULATOR_ID_LDO0 = 0,
	MAX77654_REGULATOR_ID_LDO1,
	MAX77654_REGULATOR_ID_SBB0,
	MAX77654_REGULATOR_ID_SBB1,
	MAX77654_REGULATOR_ID_SBB2,
	MAX77654_REGULATOR_NUM_REGULATORS,
};

enum {
	MAX77658_REGULATOR_ID_LDO0 = 0,
	MAX77658_REGULATOR_ID_LDO1,
	MAX77658_REGULATOR_ID_SBB0,
	MAX77658_REGULATOR_ID_SBB1,
	MAX77658_REGULATOR_ID_SBB2,
	MAX77658_REGULATOR_NUM_REGULATORS,
};

enum {
	MAX77659_REGULATOR_ID_LDO0 = 0,
	MAX77659_REGULATOR_ID_SBB0,
	MAX77659_REGULATOR_ID_SBB1,
	MAX77659_REGULATOR_ID_SBB2,
	MAX77659_REGULATOR_NUM_REGULATORS,
};

static const struct linear_range max77650_ldo_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(1350000, 0x00, 0x7f, 12500),
};

static const struct linear_range max77654_ldo_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x7f, 25000),
};

static const struct linear_range max77658_ldo_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0x7f, 25000),
};

static const struct linear_range max77650_sbb0_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x3f, 25000),
};

static const struct linear_range max77650_sbb1_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x3f, 12500),
};

static const unsigned int max77651_sbb1_volt_range_sel[] = {
	0x0, 0x1, 0x2, 0x3
};

static const struct linear_range max77651_sbb1_volt_ranges[] = {
	/* range index 0 */
	REGULATOR_LINEAR_RANGE(2400000, 0x00, 0x0f, 50000),
	/* range index 1 */
	REGULATOR_LINEAR_RANGE(3200000, 0x00, 0x0f, 50000),
	/* range index 2 */
	REGULATOR_LINEAR_RANGE(4000000, 0x00, 0x0f, 50000),
	/* range index 3 */
	REGULATOR_LINEAR_RANGE(4800000, 0x00, 0x09, 50000),
};

static const struct linear_range max77650_sbb2_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x3f, 50000),
};

static const struct linear_range max77651_sbb2_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(2400000, 0x00, 0x3f, 50000),
};

static const struct linear_range max77654_sbb_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x5e, 50000),
};

static const struct linear_range max77658_sbb_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0xc8, 25000),
};

static const struct linear_range max77659_sbb_volt_ranges[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0x2f, 25000),
	REGULATOR_LINEAR_RANGE(1700000, 0x30, 0x7c, 50000),
};

static const unsigned int max77650_current_limit_table[] = {
	1000000, 866000, 707000, 500000,
};

static const unsigned int max77658_current_limit_table[] = {
	1000000, 750000, 500000, 333000,
};

static const struct regulator_ops max77650_regulator_LDO_ops = {
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_ascend,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_active_discharge	= regulator_set_active_discharge_regmap,
};

static const struct regulator_ops max77650_regulator_SBB_ops = {
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_ascend,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.get_current_limit	= regulator_get_current_limit_regmap,
	.set_current_limit	= regulator_set_current_limit_regmap,
	.set_active_discharge	= regulator_set_active_discharge_regmap,
};

#define MAX77650_REGULATOR_DESC_SBB(_name, _family, _id, _lr, _vsel_mask,\
				    _csel_reg, _csel_mask, _curr_table)	\
{									\
	.name			= _name,				\
	.id			= _family##_REGULATOR_ID_##_id,		\
	.of_match		= of_match_ptr(_name),			\
	.regulators_node	= of_match_ptr("regulators"),		\
	.ops			= &max77650_regulator_SBB_ops,		\
	.linear_ranges		= _lr,					\
	.n_linear_ranges	= ARRAY_SIZE(_lr),			\
	.vsel_mask		= _vsel_mask,				\
	.vsel_reg		= _family##_REG_CNFG_##_id##_A,		\
	.enable_reg		= _family##_REG_CNFG_##_id##_B,		\
	.enable_mask		= MAX77650_REGULATOR_EN_CTRL_MASK,	\
	.enable_val		= MAX77650_REGULATOR_ENABLED,		\
	.disable_val		= MAX77650_REGULATOR_DISABLED,		\
	.active_discharge_off	= MAX77650_REGULATOR_AD_DISABLED,	\
	.active_discharge_on	= MAX77650_REGULATOR_AD_ENABLED,	\
	.active_discharge_mask	= MAX77650_REGULATOR_AD_MASK,		\
	.active_discharge_reg	= _family##_REG_CNFG_##_id##_B,		\
	.type			= REGULATOR_VOLTAGE,			\
	.owner			= THIS_MODULE,				\
	.csel_reg		= (_csel_reg),				\
	.csel_mask		= (_csel_mask),				\
	.curr_table		= _curr_table,				\
	.n_current_limits	=  ARRAY_SIZE(_curr_table),		\
}

#define MAX77650_REGULATOR_DESC_LDO(_name, _family, _id, _lr)		\
{									\
	.name			= _name,				\
	.id			= _family##_REGULATOR_ID_##_id,		\
	.of_match		= of_match_ptr(_name),			\
	.regulators_node	= of_match_ptr("regulators"),		\
	.ops			= &max77650_regulator_LDO_ops,		\
	.type			= REGULATOR_VOLTAGE,			\
	.owner			= THIS_MODULE,				\
	.linear_ranges		= _lr,					\
	.n_linear_ranges	= ARRAY_SIZE(_lr),			\
	.vsel_reg		= _family##_REG_CNFG_##_id##_A,		\
	.vsel_mask		= MAX77650_REGULATOR_V_LDO_MASK,	\
	.enable_reg		= _family##_REG_CNFG_##_id##_B,		\
	.enable_mask		= MAX77650_REGULATOR_EN_CTRL_MASK,	\
	.enable_val		= MAX77650_REGULATOR_ENABLED,		\
	.disable_val		= MAX77650_REGULATOR_DISABLED,		\
	.active_discharge_off	= MAX77650_REGULATOR_AD_DISABLED,	\
	.active_discharge_on	= MAX77650_REGULATOR_AD_ENABLED,	\
	.active_discharge_mask	= MAX77650_REGULATOR_AD_MASK,		\
	.active_discharge_reg	= _family##_REG_CNFG_##_id##_B,		\
}

static const struct regulator_desc max77643_regulator_desc[] = {
	MAX77650_REGULATOR_DESC_LDO("ldo0", MAX77643, LDO0,
				    max77658_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_SBB("sbb0", MAX77643, SBB0,
				    max77658_sbb_volt_ranges,
				    MAX77658_REGULATOR_V_SBB_MASK,
				    MAX77643_REG_CNFG_SBB0_B,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb1", MAX77643, SBB1,
				    max77658_sbb_volt_ranges,
				    MAX77658_REGULATOR_V_SBB_MASK,
				    MAX77643_REG_CNFG_SBB1_B,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb2", MAX77643, SBB2,
				    max77658_sbb_volt_ranges,
				    MAX77658_REGULATOR_V_SBB_MASK,
				    MAX77643_REG_CNFG_SBB2_B,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
};

static const struct regulator_desc max77650_regulator_desc[] = {
	MAX77650_REGULATOR_DESC_LDO("ldo0", MAX77650, LDO,
				    max77650_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_SBB("sbb0", MAX77650, SBB0,
				    max77650_sbb0_volt_ranges,
				    MAX77650_REGULATOR_V_SBB_MASK,
				    MAX77650_REG_CNFG_SBB0_A,
				    MAX77650_REGULATOR_CURR_LIM_MASK,
				    max77650_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb1", MAX77650, SBB1,
				    max77650_sbb1_volt_ranges,
				    MAX77650_REGULATOR_V_SBB_MASK,
				    MAX77650_REG_CNFG_SBB1_A,
				    MAX77650_REGULATOR_CURR_LIM_MASK,
				    max77650_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb2", MAX77650, SBB2,
				    max77650_sbb2_volt_ranges,
				    MAX77650_REGULATOR_V_SBB_MASK,
				    MAX77650_REG_CNFG_SBB2_A,
				    MAX77650_REGULATOR_CURR_LIM_MASK,
				    max77650_current_limit_table),
};

static const struct regulator_desc max77651_regulator_desc[] = {
	MAX77650_REGULATOR_DESC_LDO("ldo0", MAX77650, LDO,
				    max77650_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_SBB("sbb0", MAX77650, SBB0,
				    max77650_sbb0_volt_ranges,
				    MAX77650_REGULATOR_V_SBB_MASK,
				    MAX77650_REG_CNFG_SBB0_A,
				    MAX77650_REGULATOR_CURR_LIM_MASK,
				    max77650_current_limit_table),
	{
		.name			= "sbb1",
		.of_match		= of_match_ptr("sbb1"),
		.regulators_node	= of_match_ptr("regulators"),
		.id			= MAX77650_REGULATOR_ID_SBB1,
		.ops			= &max77650_regulator_SBB_ops,
		.linear_range_selectors	= max77651_sbb1_volt_range_sel,
		.linear_ranges		= max77651_sbb1_volt_ranges,
		.n_linear_ranges	= ARRAY_SIZE(max77651_sbb1_volt_ranges),
		.n_voltages		= 58,
		.vsel_step		= 1,
		.vsel_range_mask	= MAX77651_REGULATOR_V_SBB1_RANGE_MASK,
		.vsel_range_reg		= MAX77650_REG_CNFG_SBB1_A,
		.vsel_mask		= MAX77651_REGULATOR_V_SBB1_MASK,
		.vsel_reg		= MAX77650_REG_CNFG_SBB1_A,
		.active_discharge_off	= MAX77650_REGULATOR_AD_DISABLED,
		.active_discharge_on	= MAX77650_REGULATOR_AD_ENABLED,
		.active_discharge_mask	= MAX77650_REGULATOR_AD_MASK,
		.active_discharge_reg	= MAX77650_REG_CNFG_SBB1_B,
		.enable_time		= 100,
		.type			= REGULATOR_VOLTAGE,
		.owner			= THIS_MODULE,
		.csel_reg		= MAX77650_REG_CNFG_SBB1_A,
		.csel_mask		= MAX77650_REGULATOR_CURR_LIM_MASK,
		.curr_table		= max77650_current_limit_table,
		.n_current_limits = ARRAY_SIZE(max77650_current_limit_table),
	},
	MAX77650_REGULATOR_DESC_SBB("sbb2", MAX77650, SBB2,
				    max77651_sbb2_volt_ranges,
				    MAX77650_REGULATOR_V_SBB_MASK,
				    MAX77650_REG_CNFG_SBB2_A,
				    MAX77650_REGULATOR_CURR_LIM_MASK,
				    max77650_current_limit_table),
};

static const struct regulator_desc max77654_regulator_desc[] = {
	MAX77650_REGULATOR_DESC_LDO("ldo0", MAX77654, LDO0,
				    max77654_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_LDO("ldo1", MAX77654, LDO1,
				    max77654_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_SBB("sbb0", MAX77654, SBB0,
				    max77654_sbb_volt_ranges,
				    MAX77654_REGULATOR_V_SBB_MASK,
				    MAX77654_REG_CNFG_SBB0_A,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb1", MAX77654, SBB1,
				    max77654_sbb_volt_ranges,
				    MAX77654_REGULATOR_V_SBB_MASK,
				    MAX77654_REG_CNFG_SBB1_A,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb2", MAX77654, SBB2,
				    max77654_sbb_volt_ranges,
				    MAX77654_REGULATOR_V_SBB_MASK,
				    MAX77654_REG_CNFG_SBB2_A,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
};

static const struct regulator_desc max77658_regulator_desc[] = {
	MAX77650_REGULATOR_DESC_LDO("ldo0", MAX77658, LDO0,
				    max77658_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_LDO("ldo1", MAX77658, LDO1,
				    max77658_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_SBB("sbb0", MAX77658, SBB0,
				    max77658_sbb_volt_ranges,
				    MAX77658_REGULATOR_V_SBB_MASK,
				    MAX77658_REG_CNFG_SBB0_A,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb1", MAX77658, SBB1,
				    max77658_sbb_volt_ranges,
				    MAX77658_REGULATOR_V_SBB_MASK,
				    MAX77658_REG_CNFG_SBB1_A,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb2", MAX77658, SBB2,
				    max77658_sbb_volt_ranges,
				    MAX77658_REGULATOR_V_SBB_MASK,
				    MAX77658_REG_CNFG_SBB2_A,
				    MAX77658_REGULATOR_CURR_LIM_MASK,
				    max77658_current_limit_table),
};

static const struct regulator_desc max77659_regulator_desc[] = {
	MAX77650_REGULATOR_DESC_LDO("ldo0", MAX77659, LDO0,
				    max77658_ldo_volt_ranges),
	MAX77650_REGULATOR_DESC_SBB("sbb0", MAX77659, SBB0,
				    max77659_sbb_volt_ranges,
				    MAX77654_REGULATOR_V_SBB_MASK,
				    MAX77659_REG_CNFG_SBB_TOP_B,
				    MAX77659_REGULATOR_CURR_LIM_SBB0_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb1", MAX77659, SBB1,
				    max77659_sbb_volt_ranges,
				    MAX77654_REGULATOR_V_SBB_MASK,
				    MAX77659_REG_CNFG_SBB_TOP_B,
				    MAX77659_REGULATOR_CURR_LIM_SBB1_MASK,
				    max77658_current_limit_table),
	MAX77650_REGULATOR_DESC_SBB("sbb2", MAX77659, SBB2,
				    max77659_sbb_volt_ranges,
				    MAX77654_REGULATOR_V_SBB_MASK,
				    MAX77659_REG_CNFG_SBB_TOP_B,
				    MAX77659_REGULATOR_CURR_LIM_SBB2_MASK,
				    max77658_current_limit_table),
};

static int max77650_regulator_probe(struct platform_device *pdev)
{
	struct max77650_dev *max77650 = dev_get_drvdata(pdev->dev.parent);
	const struct regulator_desc *regulators;
	struct regulator_config config = { };
	struct device *dev, *parent;
	struct regulator_dev *rdev;
	int n_regulators = 0;
	struct regmap *map;
	unsigned int val;
	int i, rv;

	dev = &pdev->dev;
	parent = dev->parent;

	if (!dev->of_node)
		dev->of_node = parent->of_node;

	switch (max77650->id) {
	case ID_MAX77643:
		regulators = max77643_regulator_desc;
		n_regulators = ARRAY_SIZE(max77643_regulator_desc);
		break;
	case ID_MAX77650:
		map = dev_get_regmap(parent, NULL);
		if (!map)
			return -ENODEV;

		rv = regmap_read(map, MAX77650_REG_CID, &val);
		if (rv)
			return rv;

		switch (MAX77650_CID_BITS(val)) {
		case MAX77650_CID_77650A:
		case MAX77650_CID_77650C:
			regulators = max77650_regulator_desc;
			n_regulators = ARRAY_SIZE(max77650_regulator_desc);
			break;
		case MAX77650_CID_77651A:
		case MAX77650_CID_77651B:
			regulators = max77651_regulator_desc;
			n_regulators = ARRAY_SIZE(max77651_regulator_desc);
			break;
		default:
			return -ENODEV;
		}
		break;
	case ID_MAX77659:
		regulators = max77659_regulator_desc;
		n_regulators = ARRAY_SIZE(max77659_regulator_desc);
		break;
	case ID_MAX77654:
		regulators = max77654_regulator_desc;
		n_regulators = ARRAY_SIZE(max77654_regulator_desc);
		break;
	case ID_MAX77658:
		regulators = max77658_regulator_desc;
		n_regulators = ARRAY_SIZE(max77658_regulator_desc);
		break;
	default:
		return -EINVAL;
	}

	config.dev = parent;

	for (i = 0; i < n_regulators; i++) {
		rdev = devm_regulator_register(dev, &regulators[i], &config);
		if (IS_ERR(rdev)) {
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "Unable to register regulator\n");
		}
	}

	return 0;
}

static const struct of_device_id max77650_regulator_of_match[] = {
	{ .compatible = "adi,max77643-regulator" },
	{ .compatible = "maxim,max77650-regulator" },
	{ .compatible = "adi,max77654-regulator" },
	{ .compatible = "adi,max77658-regulator" },
	{ .compatible = "adi,max77659-regulator" },
	{ /* sentinel */  }
};
MODULE_DEVICE_TABLE(of, max77650_regulator_of_match);

static struct platform_driver max77650_regulator_driver = {
	.driver = {
		.name = "max77650-regulator",
		.of_match_table = max77650_regulator_of_match,
	},
	.probe = max77650_regulator_probe,
};
module_platform_driver(max77650_regulator_driver);

MODULE_DESCRIPTION("MAXIM 77650/77651 regulator driver");
MODULE_AUTHOR("Bartosz Golaszewski <bgolaszewski@baylibre.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:max77650-regulator");
