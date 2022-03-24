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
#include <linux/power/max77658-regulator.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/version.h>

static unsigned int max77658_safeout_volt_table[0x80];

static void initialize_safeout_volt_table(void)
{
	int i;
	int volt_step = 25;
	int base = 500000;

	for (i = 0; i < 0x80; i++)
		max77658_safeout_volt_table[i] = base + (i * volt_step);
}

static const struct regulator_ops max77658_safeout_ops = {
	.list_voltage		= regulator_list_voltage_table,
	.map_voltage		= regulator_map_voltage_ascend,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

#define REGULATOR_DESC_SFO(num, vsel_m, enable_m) {			   \
	.name			= "SAFEOUT"#num,			   \
	.id			= MAX77658_SAFEOUT##num,		   \
	.ops			= &max77658_safeout_ops,		   \
	.type			= REGULATOR_VOLTAGE,			   \
	.owner			= THIS_MODULE,				   \
	.n_voltages		= ARRAY_SIZE(max77658_safeout_volt_table), \
	.volt_table		= max77658_safeout_volt_table,		   \
	.vsel_reg		= REG_CNFG_LDO##num##_A,		   \
	.vsel_mask		= (vsel_m),				   \
	.enable_reg		= REG_CNFG_LDO##num##_B,		   \
	.enable_mask	        = (enable_m),				   \
	.enable_val		= 0x06,					   \
	.disable_val	        = 0x04,					   \
}

static struct regulator_desc max77658_safeout_desc[] = {
	REGULATOR_DESC_SFO(0, BITS_CONFIG_LDOx_A_TV_LDO, BITS_CONFIG_LDOx_B_EN_LDO),
	REGULATOR_DESC_SFO(1, BITS_CONFIG_LDOx_A_TV_LDO, BITS_CONFIG_LDOx_B_EN_LDO),
};

static int max77658_regulator_parse_dt(struct max77658_regulator_dev *regulator)
{
	struct device *dev = regulator->dev;
	struct device_node *np = of_find_node_by_name(NULL, "regulator");
	struct device_node *reg_np;
	struct max77658_regulator_data *rdata;
	int i;

	if (!np)
		return dev_err_probe(dev, -EINVAL, "Unable to find regulator node\n");

	regulator->num_regulators = of_get_child_count(np);

	rdata = devm_kzalloc(dev, sizeof(*rdata) * regulator->num_regulators, GFP_KERNEL);

	if (!rdata)
		return dev_err_probe(dev, -ENOMEM, "Failed to allocate memory\n");

	regulator->regulators = rdata;
	for_each_child_of_node(np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(max77658_safeout_desc); i++)
			if (!of_node_cmp(reg_np->name, max77658_safeout_desc[i].name))
				break;

		if (i == ARRAY_SIZE(max77658_safeout_desc)) {
			dev_warn(regulator->dev,
				 "don't know how to configure regulator %s\n",
				 reg_np->name);

			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(dev,
							     reg_np,
							     max77658_safeout_desc);
		rdata->of_node = reg_np;
		rdata++;
	}
	of_node_put(np);

	return 0;
}

static int max77658_regulator_probe(struct platform_device *pdev)
{
	struct max77658_dev *max77658 = dev_get_drvdata(pdev->dev.parent);
	struct max77658_regulator_dev *regulator;
	struct regmap *regmap;
	struct regulator_config config = {};
	int i, ret;

	regulator = devm_kzalloc(&pdev->dev,
				 sizeof(struct max77658_regulator_dev),
				 GFP_KERNEL);
	if (!regulator)
		return dev_err_probe(&pdev->dev, -ENOMEM,
				     "Failed to allocate memory\n");

	regulator->dev = &pdev->dev;
	regulator->max77658 = max77658;

	ret = max77658_regulator_parse_dt(regulator);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to parse DT\n");

	regulator->rdev = devm_kzalloc(&pdev->dev,
				sizeof(struct regulator_dev *) * regulator->num_regulators,
				GFP_KERNEL);
	if (!regulator->rdev)
		return dev_err_probe(&pdev->dev, -ENOMEM,
				     "Failed to allocate memory for regulator devices\n");

	platform_set_drvdata(pdev, regulator);
	regmap = regulator->max77658->regmap_pmic;

	initialize_safeout_volt_table();

	for (i = 0; i < regulator->num_regulators; i++) {
		int id = regulator->regulators[i].id;

		config.dev = &pdev->dev;
		config.driver_data = regulator;
		config.init_data = regulator->regulators[i].initdata;
		config.of_node = regulator->regulators[i].of_node;
		config.regmap = regmap;

		regulator->rdev[i] = devm_regulator_register(&pdev->dev,
							    &max77658_safeout_desc[id],
							    &config);

		if (IS_ERR(regulator->rdev[i]))
			return dev_err_probe(&pdev->dev, PTR_ERR(regulator->rdev[i]),
					     "Failed to register regulator %s\n",
					      max77658_safeout_desc[id].name);
	}
	return 0;
}

static const struct platform_device_id max77658_regulator_id[] = {
	{ MAX77658_REGULATOR_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, max77658_regulator_id);

static struct platform_driver max77658_regulator_driver = {
	.driver = {
		.name = MAX77658_REGULATOR_NAME,
	},
	.probe = max77658_regulator_probe,
	.id_table = max77658_regulator_id,
};

module_platform_driver(max77658_regulator_driver);

MODULE_DESCRIPTION("MAX77658 Regulator Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com ");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
