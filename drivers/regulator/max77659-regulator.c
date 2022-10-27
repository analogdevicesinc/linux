// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/mfd/max77659.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define MAX77659_LDO_VOLT_REG_MAX 0x7F
#define MAX77659_LDO_VOLT_N_RANGE 0x80
#define MAX77659_LDO_VOLT_STEP 25000
#define MAX77659_LDO_VOLT_BASE 500000

#define MAX77659_REG_CNFG_LDO0_A 0x38
#define MAX77659_REG_CNFG_LDO0_B 0x39

#define MAX77659_BITS_CONFIG_LDO0_A_TV_LDO GENMASK(6, 0)
#define MAX77659_BITS_CONFIG_LDO0_B_EN_LDO GENMASK(2, 0)

struct max77659_regulator_data {
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};

struct max77659_regulator_dev {
	struct device *dev;
	struct max77659_dev *max77659;
	struct regulator_dev *rdev;
	struct max77659_regulator_data *rdata;
};

/*
 * 0.500 to 3.675V (25mV step)
 */
static const struct linear_range MAX77659_LDO_volts[] = {
	REGULATOR_LINEAR_RANGE(MAX77659_LDO_VOLT_BASE, 0x00, MAX77659_LDO_VOLT_REG_MAX,
			       MAX77659_LDO_VOLT_STEP),
};

static struct regulator_ops max77659_LDO_ops = {
	.list_voltage	 = regulator_list_voltage_linear_range,
	.map_voltage	 = regulator_map_voltage_ascend,
	.is_enabled	 = regulator_is_enabled_regmap,
	.enable		 = regulator_enable_regmap,
	.disable	 = regulator_disable_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static struct regulator_desc max77659_LDO_desc = {
	.name		 = "LDO",
	.id		 = 0,
	.ops		 = &max77659_LDO_ops,
	.type		 = REGULATOR_VOLTAGE,
	.owner		 = THIS_MODULE,
	.n_linear_ranges = MAX77659_LDO_VOLT_N_RANGE,
	.linear_ranges	 = MAX77659_LDO_volts,
	.vsel_reg	 = MAX77659_REG_CNFG_LDO0_A,
	.vsel_mask	 = MAX77659_BITS_CONFIG_LDO0_A_TV_LDO,
	.enable_reg	 = MAX77659_REG_CNFG_LDO0_B,
	.enable_mask	 = MAX77659_BITS_CONFIG_LDO0_B_EN_LDO,
	.enable_val	 = 0x06,
	.disable_val	 = 0x04,
};

static int max77659_regulator_parse_dt(struct max77659_regulator_dev *regulator)
{
	struct device *dev = regulator->dev;
	struct device_node *np;
	struct max77659_regulator_data *rdata;

	rdata = devm_kzalloc(dev, sizeof(*rdata), GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	np = of_find_node_by_name(NULL, "regulator");
	if (!np)
		return dev_err_probe(dev, -EINVAL, "Unable to find regulator node\n");

	regulator->rdata = rdata;
	rdata->initdata = of_get_regulator_init_data(dev, np, &max77659_LDO_desc);
	rdata->of_node = np;

	of_node_put(np);

	return 0;
}

static int max77659_regulator_probe(struct platform_device *pdev)
{
	struct max77659_dev *max77659 = dev_get_drvdata(pdev->dev.parent);
	struct max77659_regulator_dev *regulator;
	struct regmap *regmap;
	struct regulator_config config = {};
	int ret;

	regulator = devm_kzalloc(&pdev->dev, sizeof(*regulator), GFP_KERNEL);
	if (!regulator)
		return -ENOMEM;

	regulator->dev = &pdev->dev;
	regulator->max77659 = max77659;

	ret = max77659_regulator_parse_dt(regulator);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to parse DT\n");

	regulator->rdev = devm_kzalloc(&pdev->dev, sizeof(struct regulator_dev *), GFP_KERNEL);
	if (!regulator->rdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, regulator);
	regmap = regulator->max77659->regmap;

	config.dev = &pdev->dev;
	config.driver_data = regulator;
	config.init_data = regulator->rdata->initdata;
	config.of_node = regulator->rdata->of_node;
	config.regmap = regmap;
	regulator->rdev = devm_regulator_register(&pdev->dev, &max77659_LDO_desc, &config);

	if (IS_ERR(regulator->rdev)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(regulator->rdev),
				     "Failed to register regulator %s\n", max77659_LDO_desc.name);
	}

	return 0;
}

static const struct platform_device_id max77659_regulator_id[] = {
	{ MAX77659_REGULATOR_NAME, 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, max77659_regulator_id);

static struct platform_driver max77659_regulator_driver = {
	.driver = {
		.name = MAX77659_REGULATOR_NAME,
	},
	.probe = max77659_regulator_probe,
	.id_table = max77659_regulator_id,
};

module_platform_driver(max77659_regulator_driver);

MODULE_DESCRIPTION("max77659 Regulator Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com, Zeynep.Arslanbenzer@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
