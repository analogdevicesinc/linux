// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 Analog Devices, Inc.
 * ADI regulator driver for the MAX77643/54/58/59
 */

#include <linux/mfd/max77658.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define MAX77658_LDO_VOLT_REG_MAX	0x7F
#define MAX77658_LDO_VOLT_N_RANGE	0x80
#define MAX77658_LDO_VOLT_STEP		25000
#define MAX77658_LDO_VOLT_BASE		500000
#define MAX77654_LDO_VOLT_BASE		800000

#define MAX77658_REG_CNFG_LDO0_A	0x48
#define MAX77658_REG_CNFG_LDO0_B	0x49

#define MAX77654_REG_CNFG_LDO0_A	0x38
#define MAX77654_REG_CNFG_LDO0_B	0x39

#define MAX77658_REG_CNFG_LDO1_A	0x4A
#define MAX77658_REG_CNFG_LDO1_B	0x4B

#define MAX77654_REG_CNFG_LDO1_A	0x3A
#define MAX77654_REG_CNFG_LDO1_B	0x3B

#define MAX77658_BITS_CONFIG_LDOX_A_TV_LDO	GENMASK(6, 0)
#define MAX77658_BITS_CONFIG_LDOX_B_EN_LDO	GENMASK(2, 0)

/*
 * 0.500 to 3.675V (25mV step)
 */
static const struct linear_range MAX77658_LDO_volts[] = {
	REGULATOR_LINEAR_RANGE(MAX77658_LDO_VOLT_BASE, 0x00,
			       MAX77658_LDO_VOLT_REG_MAX,
			       MAX77658_LDO_VOLT_STEP),
};

/*
 * 0.800 to 3.975V (25mV step)
 */
static const struct linear_range MAX77654_LDO_volts[] = {
	REGULATOR_LINEAR_RANGE(MAX77654_LDO_VOLT_BASE, 0x00,
			       MAX77658_LDO_VOLT_REG_MAX,
			       MAX77658_LDO_VOLT_STEP),
};

static const struct regulator_ops max77658_LDO_ops = {
	.list_voltage	 = regulator_list_voltage_linear_range,
	.map_voltage	 = regulator_map_voltage_ascend,
	.is_enabled	 = regulator_is_enabled_regmap,
	.enable		 = regulator_enable_regmap,
	.disable	 = regulator_disable_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

#define REGULATOR_DESC_LDO(num, volts, vsel_r, vsel_m, enable_r, enable_m) { \
	.name			= "LDO"#num,				\
	.id			= num,					\
	.of_match		= of_match_ptr("LDO"#num),		\
	.regulators_node	= of_match_ptr("regulators"),		\
	.ops			= &max77658_LDO_ops,			\
	.type			= REGULATOR_VOLTAGE,			\
	.owner			= THIS_MODULE,				\
	.linear_ranges		= volts,				\
	.n_linear_ranges	= ARRAY_SIZE(volts),			\
	.vsel_reg		= vsel_r,				\
	.vsel_mask		= vsel_m,				\
	.enable_reg		= enable_r,				\
	.enable_mask		= enable_m,				\
	.enable_val		= 0x06,					\
	.disable_val		= 0x04,					\
}

static const struct regulator_desc max77643_59_ldo_desc[] = {
	REGULATOR_DESC_LDO(0, MAX77658_LDO_volts, MAX77654_REG_CNFG_LDO0_A,
			   MAX77658_BITS_CONFIG_LDOX_A_TV_LDO,
			   MAX77654_REG_CNFG_LDO0_B,
			   MAX77658_BITS_CONFIG_LDOX_B_EN_LDO),
};

static const struct regulator_desc max77654_ldo_desc[] = {
	REGULATOR_DESC_LDO(0, MAX77654_LDO_volts, MAX77654_REG_CNFG_LDO0_A,
			   MAX77658_BITS_CONFIG_LDOX_A_TV_LDO,
			   MAX77654_REG_CNFG_LDO0_B,
			   MAX77658_BITS_CONFIG_LDOX_B_EN_LDO),
	REGULATOR_DESC_LDO(1, MAX77654_LDO_volts, MAX77654_REG_CNFG_LDO1_A,
			   MAX77658_BITS_CONFIG_LDOX_A_TV_LDO,
			   MAX77654_REG_CNFG_LDO1_B,
			   MAX77658_BITS_CONFIG_LDOX_B_EN_LDO),
};

static const struct regulator_desc max77658_ldo_desc[] = {
	REGULATOR_DESC_LDO(0, MAX77658_LDO_volts, MAX77658_REG_CNFG_LDO0_A,
			   MAX77658_BITS_CONFIG_LDOX_A_TV_LDO,
			   MAX77658_REG_CNFG_LDO0_B,
			   MAX77658_BITS_CONFIG_LDOX_B_EN_LDO),
	REGULATOR_DESC_LDO(1, MAX77658_LDO_volts, MAX77658_REG_CNFG_LDO1_A,
			   MAX77658_BITS_CONFIG_LDOX_A_TV_LDO,
			   MAX77658_REG_CNFG_LDO1_B,
			   MAX77658_BITS_CONFIG_LDOX_B_EN_LDO),
};

static int max77658_regulator_probe(struct platform_device *pdev)
{
	struct max77658_dev *max77658 = dev_get_drvdata(pdev->dev.parent);
	const struct regulator_desc *regulators;
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	int n_regulators = 0;
	int i;

	switch (max77658->id) {
	case ID_MAX77643:
	case ID_MAX77659:
		regulators = max77643_59_ldo_desc;
		n_regulators = ARRAY_SIZE(max77643_59_ldo_desc);
		break;
	case ID_MAX77654:
		regulators = max77654_ldo_desc;
		n_regulators = ARRAY_SIZE(max77654_ldo_desc);
		break;
	case ID_MAX77658:
		regulators = max77658_ldo_desc;
		n_regulators = ARRAY_SIZE(max77658_ldo_desc);
		break;
	default:
		return -EINVAL;
	}

	config.dev = pdev->dev.parent;

	for (i = 0; i < n_regulators; i++) {
		rdev = devm_regulator_register(&pdev->dev, &regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "Failed to register %s regulator\n",
				regulators[i].name);
			return PTR_ERR(rdev);
		}
	}

	return 0;
}

static const struct platform_device_id max77658_regulator_id[] = {
	{ "max77658-regulator" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, max77658_regulator_id);

static struct platform_driver max77658_regulator_driver = {
	.driver = {
		.name = "max77658-regulator",
	},
	.probe = max77658_regulator_probe,
	.id_table = max77658_regulator_id,
};

module_platform_driver(max77658_regulator_driver);

MODULE_DESCRIPTION("MAX77658 Regulator Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com, Zeynep.Arslanbenzer@analog.com");
MODULE_LICENSE("GPL");
