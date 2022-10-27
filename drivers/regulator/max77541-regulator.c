// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 Analog Devices, Inc.
 * ADI Regulator driver for the MAX77540 and MAX77541
 *
 */

#include <linux/i2c.h>
#include <linux/mfd/max77541.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

static const struct regulator_ops max77541_buck_ops = {
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
	.list_voltage		= regulator_list_voltage_pickable_linear_range,
	.get_voltage_sel	= regulator_get_voltage_sel_pickable_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_pickable_regmap,
};

static const struct linear_range max77540_buck_ranges[] = {
	/* Ranges when VOLT_SEL bits are 0x00 */
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0x8B, 5000),
	REGULATOR_LINEAR_RANGE(1200000, 0x8C, 0xFF, 0),
	/* Ranges when VOLT_SEL bits are 0x40 */
	REGULATOR_LINEAR_RANGE(1200000, 0x00, 0x8B, 10000),
	REGULATOR_LINEAR_RANGE(2400000, 0x8C, 0xFF, 0),
	/* Ranges when VOLT_SEL bits are  0x80 */
	REGULATOR_LINEAR_RANGE(2000000, 0x00, 0x9F, 20000),
	REGULATOR_LINEAR_RANGE(5200000, 0xA0, 0xFF, 0),
};

static const struct linear_range max77541_buck_ranges[] = {
	/* Ranges when VOLT_SEL bits are 0x00 */
	REGULATOR_LINEAR_RANGE(300000, 0x00, 0xB3, 5000),
	REGULATOR_LINEAR_RANGE(1200000, 0xB4, 0xFF, 0),
	/* Ranges when VOLT_SEL bits are 0x40 */
	REGULATOR_LINEAR_RANGE(1200000, 0x00, 0x8B, 10000),
	REGULATOR_LINEAR_RANGE(2400000, 0x8C, 0xFF, 0),
	/* Ranges when VOLT_SEL bits are  0x80 */
	REGULATOR_LINEAR_RANGE(2000000, 0x00, 0x9F, 20000),
	REGULATOR_LINEAR_RANGE(5200000, 0xA0, 0xFF, 0),
};

static const unsigned int max77541_buck_volt_range_sel[] = {
	0x00, 0x00, 0x40, 0x40, 0x80, 0x80
};

#define MAX77540_BUCK(_id, _ops)					\
	{	.id = MAX77541_BUCK ## _id,				\
		.name = "BUCK"#_id,					\
		.of_match = "BUCK"#_id,					\
		.regulators_node = "regulators",			\
		.enable_reg = MAX77541_REG_EN_CTRL,			\
		.enable_mask = MAX77541_BIT_M ## _id ## _EN,		\
		.ops = &(_ops),						\
		.type = REGULATOR_VOLTAGE,				\
		.linear_ranges = max77540_buck_ranges,			\
		.n_linear_ranges = ARRAY_SIZE(max77540_buck_ranges),	\
		.vsel_reg = MAX77541_REG_M ## _id ## _VOUT,		\
		.vsel_mask = MAX77541_BITS_MX_VOUT,			\
		.vsel_range_reg = MAX77541_REG_M ## _id ## _CFG1,	\
		.vsel_range_mask = MAX77541_BITS_MX_CFG1_RNG,		\
		.linear_range_selectors = max77541_buck_volt_range_sel, \
		.owner = THIS_MODULE,					\
	}

#define MAX77541_BUCK(_id, _ops)					\
	{	.id = MAX77541_BUCK ## _id,				\
		.name = "BUCK"#_id,					\
		.of_match = "BUCK"#_id,					\
		.regulators_node = "regulators",			\
		.enable_reg = MAX77541_REG_EN_CTRL,			\
		.enable_mask = MAX77541_BIT_M ## _id ## _EN,		\
		.ops = &(_ops),						\
		.type = REGULATOR_VOLTAGE,				\
		.linear_ranges = max77541_buck_ranges,			\
		.n_linear_ranges = ARRAY_SIZE(max77541_buck_ranges),	\
		.vsel_reg = MAX77541_REG_M ## _id ## _VOUT,		\
		.vsel_mask = MAX77541_BITS_MX_VOUT,			\
		.vsel_range_reg = MAX77541_REG_M ## _id ## _CFG1,	\
		.vsel_range_mask = MAX77541_BITS_MX_CFG1_RNG,		\
		.linear_range_selectors = max77541_buck_volt_range_sel, \
		.owner = THIS_MODULE,					\
	}

static const struct regulator_desc max77540_regulators_desc[] = {
	MAX77540_BUCK(1, max77541_buck_ops),
	MAX77540_BUCK(2, max77541_buck_ops)
};

static const struct regulator_desc max77541_regulators_desc[] = {
	MAX77541_BUCK(1, max77541_buck_ops),
	MAX77541_BUCK(2, max77541_buck_ops)
};

struct max77541_regulator_data {
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};

struct max77541_regulator_dev {
	struct device *dev;
	struct max77541_dev *max77541;
	struct max77541_regulator_data *rdata[MAX77541_MAX_REGULATORS];
};

static struct of_regulator_match max77541_regulator_matches[] = {
	[0]           = { .name = "buck1" },
	[1]           = { .name = "buck2" },
};

static int max77541_regulator_parse_dt(struct max77541_regulator_dev *regulator)
{
	struct device *dev = regulator->dev;
	struct device_node *node;
	struct max77541_regulator_data *rdata;
	int max77541_regulator_matches_len = ARRAY_SIZE(max77541_regulator_matches);
	int max77541_regulator_number;
	int i;

	node = of_find_node_by_name(regulator->max77541->dev->of_node, "regulators");
	if (!node)
		return dev_err_probe(dev, -ENODEV, "Unable to find regulator node\n");

	max77541_regulator_number = of_regulator_match(dev, node, max77541_regulator_matches,
						       max77541_regulator_matches_len);
	of_node_put(node);

	for (i = 0; i < max77541_regulator_number; i++) {
		rdata = devm_kzalloc(dev, sizeof(*rdata), GFP_KERNEL);
		if (!rdata)
			return -ENOMEM;

		regulator->rdata[i] = rdata;
		regulator->rdata[i]->initdata	= max77541_regulator_matches[i].init_data;
		regulator->rdata[i]->of_node	= max77541_regulator_matches[i].of_node;
	}

	return 0;
}

static int max77541_regulator_probe(struct platform_device *pdev)
{
	struct max77541_dev *max77541 = dev_get_drvdata(pdev->dev.parent);
	struct max77541_regulator_dev *regulator;
	struct regulator_dev *rdev;
	struct regulator_config config = {};
	int ret;
	int i;

	regulator = devm_kzalloc(&pdev->dev, sizeof(*regulator), GFP_KERNEL);
	if (!regulator)
		return -ENOMEM;

	regulator->dev = &pdev->dev;
	regulator->max77541 = max77541;
	ret = max77541_regulator_parse_dt(regulator);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, regulator);

	for (i = 0; i < MAX77541_MAX_REGULATORS; i++) {
		config.dev = &pdev->dev;
		config.driver_data = regulator;
		config.init_data = regulator->rdata[i]->initdata;
		config.of_node = regulator->rdata[i]->of_node;
		config.regmap = regulator->max77541->regmap;
		switch (regulator->max77541->type) {
		case MAX77540:
			rdev = devm_regulator_register(&pdev->dev,
						       &max77540_regulators_desc[i], &config);
			if (IS_ERR(rdev))
				return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
							"Failed to register regulator\n");
			break;
		case MAX77541:
			rdev = devm_regulator_register(&pdev->dev,
						       &max77541_regulators_desc[i], &config);
			if (IS_ERR(rdev))
				return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
						"Failed to register regulator\n");
			break;
		default:
			return -EINVAL;
		}
	}

	return ret;
}

static const struct platform_device_id max77541_regulator_platform_id[] = {
	{ "max77540-regulator", MAX77540 },
	{ "max77541-regulator", MAX77541 },
	{  /* sentinel */  }
};
MODULE_DEVICE_TABLE(platform, max77541_regulator_platform_id);

static const struct of_device_id max77541_regulator_of_id[] = {
	{
		.compatible = "adi,max77540-regulator",
		.data = (void *)MAX77540,
	},
	{
		.compatible = "adi,max77541-regulator",
		.data = (void *)MAX77541,
	},
	{ /* sentinel */  }
};
MODULE_DEVICE_TABLE(of, max77541_regulator_of_id);

static struct platform_driver max77541_regulator_driver = {
	.driver = {
		.name = "max77541-regulator",
		.of_match_table = max77541_regulator_of_id,
	},
	.probe = max77541_regulator_probe,
	.id_table = max77541_regulator_platform_id,
};

module_platform_driver(max77541_regulator_driver);

MODULE_AUTHOR("Okan Sahin <Okan.Sahin@analog.com>");
MODULE_DESCRIPTION("MAX77540/MAX77541 regulator driver");
MODULE_LICENSE("GPL");
