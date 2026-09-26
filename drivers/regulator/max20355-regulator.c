// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX20355 buck-boost regulator driver
 *
 * The MAX20355 provides a 3.3W buck-boost converter that powers the PLC
 * line. DVS (Dynamic Voltage Scaling) is handled autonomously by the
 * hardware's PLC algorithm; this driver exposes the voltage setting
 * for monitoring and manual override.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regmap.h>

#include "../mfd/maxim/max2035x/max2035x.h"
#include "../mfd/maxim/max2035x/max2035x_registers.h"


/*
 * Buck-boost output voltage range: 3.0V to 5.5V
 * BB_VOLT_DEF (0x41) is an 8-bit register, LSB ≈ 9.766mV
 * Voltage = 3000mV + BB_VOLT_DEF * 9766µV
 */
static const struct linear_range max20355_bb_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0, 255, 9766),
};

static const struct regulator_ops max20355_bb_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

static const struct regulator_desc max20355_bb_desc = {
	.name = "buck-boost",
	.of_match = "buck-boost",
	.regulators_node = "regulators",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &max20355_bb_ops,
	.vsel_reg = MAX20355_REG_BB_VOLT_DEF,
	.vsel_mask = MAX20355_BB_VOLT_DEF_MASK,
	.linear_ranges = max20355_bb_ranges,
	.n_linear_ranges = ARRAY_SIZE(max20355_bb_ranges),
};

static int max20355_regulator_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = {};
	struct regulator_dev *rdev;

	config.dev = chip->dev;
	config.regmap = chip->regmap;

	rdev = devm_regulator_register(&pdev->dev, &max20355_bb_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
				     "Failed to register regulator\n");

	dev_info(&pdev->dev, "MAX20355 buck-boost regulator probed\n");

	return 0;
}

static const struct platform_device_id max20355_regulator_id[] = {
	{ "max20355-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max20355_regulator_id);

static struct platform_driver max20355_regulator_driver = {
	.driver = {
		.name = "max20355-regulator",
	},
	.probe = max20355_regulator_probe,
	.id_table = max20355_regulator_id,
};
module_platform_driver(max20355_regulator_driver);

MODULE_DESCRIPTION("Analog Devices MAX20355 buck-boost regulator driver");
MODULE_AUTHOR("Judy Na <judy.na@analog.com>");
MODULE_LICENSE("GPL");
