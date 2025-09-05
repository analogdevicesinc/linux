// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2025 Analog Devices, Inc.
 * ADI regulator driver for MAX77675.
 */

#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>

#include "max77675-regulator.h"

struct max77675_regulator {
	struct device *dev;           // Device pointer for logging and PM
	struct regmap *regmap;        // Regmap for register access
	struct regulator_dev *rdev[MAX77675_NUM_REGULATORS];
	int id;                      // SBB channel ID (0~3)

	bool use_12_5mv;             // 12.5mV step mode flag, read from HW
	unsigned int uV_step;         // voltage step size in microvolts
	bool active_discharge_enabled; // Active discharge enable status (optional)
	unsigned int drive_strength;  // Drive strength setting (optional)

	struct mutex lock;            // Mutex for register access synchronization
};

/**
 * Set the Manual Reset Time (MRT).
 *
 * Configures the manual reset time by updating the corresponding bits in the CNFG_GLBL_A register.
 *
 * @param regmap Pointer to the regmap instance.
 * @param mrt Desired MRT value (enum `max77675_mrt`).
 *
 * @return 0 on success, negative error on failure.
 */
static int max77675_set_mrt(struct max77675_regulator *maxreg, enum max77675_mrt mrt)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_MRT_MASK,
				  (mrt << MAX77675_MRT_SHIFT));
}

/**
 * Enable or disable the internal pull-up resistor on the nEN pin.
 *
 * Updates the pull-up resistor setting on the nEN pin in the CNFG_GLBL_A register.
 *
 * @param regmap Pointer to the regmap instance.
 * @param disable true to disable, false to enable pull-up resistor.
 *
 * @return 0 on success, negative error on failure.
 */
static int max77675_set_pullup_disable(struct max77675_regulator *maxreg, enum max77675_pu_dis dis)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_PU_DIS_BIT,
				  dis << MAX77675_PU_DIS_SHIFT);
}

/**
 * Enable or disable the Bias Low Power Mode (LPM).
 *
 * Configures the Bias Low Power Mode (LPM) to save power by adjusting the CNFG_GLBL_A register.
 *
 * @param regmap Pointer to the regmap instance.
 * @param enable true to enable LPM, false to disable.
 *
 * @return 0 on success, negative error on failure.
 */
static int max77675_set_bias_lpm(struct max77675_regulator *maxreg, enum max77675_bias_lpm en)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_BIAS_LPM_BIT,
				  en << MAX77675_BIAS_LPM_SHIFT);
}

/**
 * Enable or disable the SIMO internal channel.
 *
 * Updates the SIMO internal channel setting in the CNFG_GLBL_A register.
 *
 * @param regmap Pointer to the regmap instance.
 * @param disable true to disable the SIMO channel, false to enable.
 *
 * @return 0 on success, negative error on failure.
 */
static int max77675_set_simo_int_ch_dis(struct max77675_regulator *maxreg,
					enum max77675_simo_int_ch_dis dis)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_SIMO_CH_DIS_BIT,
				  dis << MAX77675_SIMO_CH_DIS_SHIFT);
}

/**
 * Set the nEN mode.
 *
 * Configures the nEN mode in the CNFG_GLBL_A register.
 *
 * @param regmap Pointer to the regmap instance.
 * @param mode Desired nEN mode (enum `max77675_nen_mode`).
 *
 * @return 0 on success, negative error on failure.
 */
static int max77675_set_enable_mode(struct max77675_regulator *maxreg, enum max77675_en_mode mode)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_NEN_MODE_MASK,
				  (mode << MAX77675_NEN_MODE_SHIFT));
}

/**
 * Configure the debounce timer for the nEN pin.
 *
 * Sets the debounce timer to either 30ms or 100us in the CNFG_GLBL_A register.
 *
 * @param regmap Pointer to the regmap instance.
 * @param enable_30ms true for 30ms, false for 100us debounce time.
 *
 * @return 0 on success, negative error on failure.
 */
static int max77675_set_debounce_timer(struct max77675_regulator *maxreg,
				       enum max77675_deb_en timer)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_DBEN_N_EN_BIT,
				  timer << MAX77675_DBEN_N_EN_SHIFT);
}

/**
 * Set the software control state.
 *
 * Configures the software control state in the CNFG_SBB_TOP_A register.
 *
 * @param regmap Pointer to the regmap instance.
 * @param ctrl Desired control state (enum `max77675_sft_ctrl`).
 *
 * @return 0 on success, negative error on failure.
 */
static int max77675_set_software_ctrl(struct max77675_regulator *maxreg,
				      enum max77675_sft_ctrl ctrl)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_B,
				  MAX77675_SFT_CTRL_MASK,
				  ctrl << MAX77675_SFT_CTRL_SHIFT);
}

/**
 * Enable the regulator by setting enable bits to ON (0b110).
 */
static int max77675_regulator_enable(struct regulator_dev *rdev)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	unsigned int val;
	int ret;

	dev_info(maxreg->dev, "Enable called for regulator: %s (reg=0x%02x, mask=0x%02x)\n",
		 desc->name, desc->enable_reg, desc->enable_mask);

	ret = regmap_read(maxreg->regmap, desc->enable_reg, &val);
	if (ret) {
		dev_err(maxreg->dev, "Failed to read enable_reg (0x%02x): %d\n",
			desc->enable_reg, ret);
		return ret;
	}

	dev_info(maxreg->dev, "Original enable_reg value: 0x%02x\n", val);

	val &= ~desc->enable_mask;
	val |= (MAX77675_ENABLE_ON << __ffs(desc->enable_mask));

	dev_info(maxreg->dev, "Modified enable_reg value: 0x%02x\n", val);

	ret = regmap_write(maxreg->regmap, desc->enable_reg, val);
	if (ret) {
		dev_err(maxreg->dev, "Failed to write enable_reg (0x%02x): %d\n",
			desc->enable_reg, ret);
	} else {
		dev_info(maxreg->dev, "Successfully enabled regulator: %s\n", desc->name);
	}

	return ret;
}

/**
 * Disable the regulator by setting enable bits to OFF (0b000).
 */
static int max77675_regulator_disable(struct regulator_dev *rdev)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	unsigned int val;
	int ret;

	ret = regmap_read(maxreg->regmap, desc->enable_reg, &val);
	if (ret)
		return ret;

	val &= ~desc->enable_mask;
	val |= (MAX77675_ENABLE_OFF << __ffs(desc->enable_mask));

	return regmap_write(maxreg->regmap, desc->enable_reg, val);
}

/**
 * Check if the regulator is enabled by reading enable bits.
 *
 * Return: 1 if enabled, 0 if disabled, negative error code on failure.
 */
static int max77675_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	unsigned int val;
	int ret;
	unsigned int enable_bits;

	ret = regmap_read(maxreg->regmap, desc->enable_reg, &val);
	if (ret)
		return ret;

	enable_bits = (val & desc->enable_mask) >> __ffs(desc->enable_mask);

	return (enable_bits == MAX77675_ENABLE_ON) ? 1 : 0;
}

/**
 * List the voltage for a given selector.
 *
 * This function calculates the voltage in microvolts (uV) corresponding to the given
 * selector value using the step size (uV_step) and minimum voltage (min_uV) from
 * the regulator's descriptor.
 *
 * @param rdev Pointer to the regulator device structure.
 * @param selector The selector value representing a specific voltage step.
 *
 * @return The calculated voltage in microvolts (uV).
 */
static int max77675_list_voltage(struct regulator_dev *rdev, unsigned int selector)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	unsigned int voltage;

	// Use uV_step from regulator_desc
	unsigned int uV_step = rdev->desc->uV_step;

	// Calculate the voltage in microvolts based on the selector value
	voltage = selector * uV_step + rdev->desc->min_uV;

	dev_info(maxreg->dev, "[%s] list voltage : %duV (%d x %duV + %duV)\n",
		 rdev->desc->name, voltage, selector, uV_step, rdev->desc->min_uV);

	return voltage;
}

static int max77675_set_voltage(struct regulator_dev *rdev, int min_uV,
				int max_uV, unsigned int *selector)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	unsigned int selector_value;
	unsigned int uV_step, min_reg_uV, max_reg_uV;
	int ret;

	uV_step = rdev->desc->uV_step;
	min_reg_uV = rdev->desc->min_uV;
	max_reg_uV = min_reg_uV + uV_step * (rdev->desc->n_voltages - 1);

	dev_info(maxreg->dev, "[%s] Requested voltage range: %duV - %duV\n",
		 rdev->desc->name, min_uV, max_uV);

	dev_info(maxreg->dev, "[%s] Supported voltage range: %duV - %duV (step: %duV)\n",
		 rdev->desc->name, min_reg_uV, max_reg_uV, uV_step);

	// Check if voltage is within valid range
	if (min_uV < min_reg_uV || max_uV > max_reg_uV) {
		dev_err(maxreg->dev, "[%s] Voltage out of range: min_uV = %d, max_uV = %d\n",
			rdev->desc->name, min_uV, max_uV);
		return -EINVAL;
	}

	// Calculate selector
	selector_value = (min_uV - min_reg_uV) / uV_step;
	if (selector_value > rdev->desc->n_voltages - 1) {
		dev_warn(maxreg->dev, "[%s] Voltage selector clipped to max value\n",
			 rdev->desc->name);
		selector_value = rdev->desc->n_voltages - 1;
	}

	// Output selector debug info
	dev_info(maxreg->dev, "[%s] Calculated selector = %u, sets voltage to %duV\n",
		 rdev->desc->name, selector_value, min_reg_uV + selector_value * uV_step);

	// Assign value to voltage index
	*selector = selector_value;

	// Write to vsel_reg
	ret = regmap_write(maxreg->regmap, rdev->desc->vsel_reg, selector_value);
	if (ret < 0) {
		dev_err(maxreg->dev, "[%s] Failed to write selector 0x%x to vsel_reg (0x%x)\n",
			rdev->desc->name, selector_value, rdev->desc->vsel_reg);
		return ret;
	}

	dev_info(maxreg->dev, "[%s] Voltage successfully set\n", rdev->desc->name);
	return 0;
}

/**
 * Get the current voltage of the specified regulator.
 *
 * This function retrieves the current voltage setting for the regulator by reading
 * the voltage select register and calculating the corresponding voltage using the
 * step size from `uV_step`.
 *
 * @param rdev Pointer to the regulator device structure.
 *
 * @return The current voltage in microvolts (uV), or a negative error code on failure.
 */
static int max77675_get_voltage(struct regulator_dev *rdev)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	unsigned int selector_value;   // selector_value is the raw value read from the register
	int ret;
	unsigned int uV_step;
	unsigned int voltage;

	// Use uV_step from regulator_desc
	uV_step = rdev->desc->uV_step;

	// Read the current voltage selector value from the voltage select register
	ret = regmap_read(maxreg->regmap, rdev->desc->vsel_reg, &selector_value);
	if (ret < 0) {
		dev_err(maxreg->dev, "Failed to read voltage from vsel_reg\n");
		return ret;
	}

	// Calculate the voltage in microvolts based on the selector value and step size
	voltage = selector_value * uV_step + rdev->desc->min_uV;

	dev_info(maxreg->dev, "[%s] voltage : %duV (%d x %duV + %duV)\n",
		 rdev->desc->name, voltage, selector_value, uV_step, rdev->desc->min_uV);

	return voltage;
}

/**
 * Set the Active Discharge (ADE) for the given regulator.
 *
 * This function sets or clears the Active Discharge bit in the corresponding
 * SBBx configuration register to enable or disable the active discharge function.
 *
 * @param rdev Regulator device structure pointer.
 * @param enable Boolean flag to enable (true) or disable (false) Active Discharge.
 *
 * @return 0 on success, negative error code on failure.
 */
static int max77675_set_active_discharge(struct regulator_dev *rdev, bool enable)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	unsigned int reg_val, reg_addr, bit_mask;
	int ret;

	// Determine the register address and bitmask based on the regulator
	switch (maxreg->id) {
	case 0:
		reg_addr = MAX77675_REG_CNFG_SBB_TOP_B;
		bit_mask = MAX77675_ADE_SBB0_BIT;
		break;
	case 1:
		reg_addr = MAX77675_REG_CNFG_SBB_TOP_B;
		bit_mask = MAX77675_ADE_SBB1_BIT;
		break;
	case 2:
		reg_addr = MAX77675_REG_CNFG_SBB_TOP_B;
		bit_mask = MAX77675_ADE_SBB2_BIT;
		break;
	case 3:
		reg_addr = MAX77675_REG_CNFG_SBB_TOP_B;
		bit_mask = MAX77675_ADE_SBB3_BIT;
		break;
	default:
		pr_err("Invalid regulator id: %d\n", maxreg->id);
		return -EINVAL;
	}

	// Read the current register value
	ret = regmap_read(maxreg->regmap, reg_addr, &reg_val);
	if (ret < 0) {
		pr_err("Failed to read register 0x%02x for regulator %d\n", reg_addr, maxreg->id);
		return ret;
	}

	// Set or clear the ADE bit based on the enable flag
	if (enable)
		reg_val |= bit_mask;  // Enable Active Discharge
	else
		reg_val &= ~bit_mask; // Disable Active Discharge

	// Write back the updated register value
	ret = regmap_write(maxreg->regmap, reg_addr, reg_val);
	if (ret < 0) {
		pr_err("Failed to write register 0x%02x for regulator %d\n", reg_addr, maxreg->id);
		return ret;
	}

	return 0;
}

static const struct regulator_ops max77675_regulator_ops = {
	.list_voltage = max77675_list_voltage,
	.enable = max77675_regulator_enable,
	.disable = max77675_regulator_disable,
	.is_enabled = max77675_regulator_is_enabled,
	.set_voltage = max77675_set_voltage,
	.get_voltage = max77675_get_voltage,
	.set_active_discharge = max77675_set_active_discharge,
};

static const struct regulator_desc max77675_regulators[MAX77675_NUM_REGULATORS] = {
	{
		.name = "sbb0",
		.of_match = "sbb0",
		.regulators_node = "regulators",
		.id = 0,
		.ops = &max77675_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 201,
		.min_uV = 500000,
		.uV_step = 25000,
		.vsel_reg = MAX77675_REG_CNFG_SBB0_A,
		.vsel_mask = 0xFF,
		.enable_reg = MAX77675_REG_CNFG_SBB0_B,
		.enable_mask = 0x07,
		.enable_val = 0x06,
		.disable_val = 0x04,
	},
	{
		.name = "sbb1",
		.of_match = "sbb1",
		.regulators_node = "regulators",
		.id = 1,
		.ops = &max77675_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 201,
		.min_uV = 500000,
		.uV_step = 25000,
		.vsel_reg = MAX77675_REG_CNFG_SBB1_A,
		.vsel_mask = 0xFF,
		.enable_reg = MAX77675_REG_CNFG_SBB1_B,
		.enable_mask = 0x07,
		.enable_val = 0x06,
		.disable_val = 0x04,
	},
	{
		.name = "sbb2",
		.of_match = "sbb2",
		.regulators_node = "regulators",
		.id = 2,
		.ops = &max77675_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 201,
		.min_uV = 500000,
		.uV_step = 25000,
		.vsel_reg = MAX77675_REG_CNFG_SBB2_A,
		.vsel_mask = 0xFF,
		.enable_reg = MAX77675_REG_CNFG_SBB2_B,
		.enable_mask = 0x07,
		.enable_val = 0x06,
		.disable_val = 0x04,
	},
	{
		.name = "sbb3",
		.of_match = "sbb3",
		.regulators_node = "regulators",
		.id = 3,
		.ops = &max77675_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 201,
		.min_uV = 500000,
		.uV_step = 25000,
		.vsel_reg = MAX77675_REG_CNFG_SBB3_A,
		.vsel_mask = 0xFF,
		.enable_reg = MAX77675_REG_CNFG_SBB3_B,
		.enable_mask = 0x07,
		.enable_val = 0x06,
		.disable_val = 0x04,
	},
};

static const struct regmap_config max77675_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
	.cache_type = REGCACHE_RBTREE,
};

static int max77675_regulator_probe(struct i2c_client *client)
{
	struct max77675_regulator *maxreg;
	struct regulator_config config = {};
	struct device_node *regulators_np;
	int i, ret;

	maxreg = devm_kzalloc(&client->dev, sizeof(*maxreg), GFP_KERNEL);
	if (!maxreg)
		return -ENOMEM;

	maxreg->dev = &client->dev;

	maxreg->regmap = devm_regmap_init_i2c(client, &max77675_regmap_config);
	if (IS_ERR(maxreg->regmap)) {
		dev_err(&client->dev, "Failed to init regmap\n");
		return PTR_ERR(maxreg->regmap);
	}

	regulators_np = of_get_child_by_name(client->dev.of_node, "regulators");
	if (!regulators_np) {
		dev_err(&client->dev, "No 'regulators' subnode found in DT\n");
		return -EINVAL;
	}

	config.dev = &client->dev;
	config.regmap = maxreg->regmap;
	config.driver_data = maxreg;

	for (i = 0; i < MAX77675_NUM_REGULATORS; i++) {
		const struct regulator_desc *desc = &max77675_regulators[i];

		config.of_node = of_get_child_by_name(regulators_np, desc->name);
		if (!config.of_node) {
			dev_warn(&client->dev, "No DT node for regulator %s\n", desc->name);
			continue;
		}

		maxreg->rdev[i] = devm_regulator_register(&client->dev, desc, &config);
		if (IS_ERR(maxreg->rdev[i])) {
			ret = PTR_ERR(maxreg->rdev[i]);
			dev_err(&client->dev,
				"Failed to register regulator %d (%s): %d\n",
				i, desc->name, ret);
			return ret;
		}
	}

	i2c_set_clientdata(client, maxreg);

	max77675_set_mrt(maxreg, MRT_4S);
	max77675_set_pullup_disable(maxreg, PU_EN);
	max77675_set_bias_lpm(maxreg, BIAS_NORMAL);
	max77675_set_simo_int_ch_dis(maxreg, SIMO_INT_EN);
	max77675_set_enable_mode(maxreg, EN_PUSH_BUTTON);
	max77675_set_debounce_timer(maxreg, DEBOUNCE_100US);
	max77675_set_software_ctrl(maxreg, SFT_NO_ACTION);

	dev_info(&client->dev, "MAX77675 regulators registered\n");
	return 0;
}

static void max77675_regulator_remove(struct i2c_client *client)
{
	struct max77675_regulator *maxreg = i2c_get_clientdata(client);

	dev_info(maxreg->dev, "MAX77675 regulators removed\n");
}

static const struct i2c_device_id max77675_i2c_id[] = {
	{ "max77675", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77675_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id max77675_of_match[] = {
	{ .compatible = "maxim,max77675", },
	{ }
};
MODULE_DEVICE_TABLE(of, max77675_of_match);
#endif

static struct i2c_driver max77675_regulator_driver = {
	.driver = {
		.name = "max77675",
		.of_match_table = of_match_ptr(max77675_of_match),
	},
	.probe = max77675_regulator_probe,
	.remove = max77675_regulator_remove,
	.id_table = max77675_i2c_id,
};

module_i2c_driver(max77675_regulator_driver);

MODULE_DESCRIPTION("MAX77675 Regulator Driver");
MODULE_AUTHOR("joan.na@analog.com");
MODULE_LICENSE("GPL");
