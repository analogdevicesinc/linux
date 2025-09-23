// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2025 Analog Devices, Inc.
 * ADI regulator driver for MAX77675.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include <linux/bitfield.h>

#include "max77675-regulator.h"

struct max77675_regulator_pdata {
	u8 fps_slot;
	u8 dvs_slew_rate;
};

struct max77675_config {
	u8 drv_slew_rate;
	u8 latency_mode;
	u8 drv_sbb_strength;
	u8 manual_reset_time;
	u8 en_pullup_disable;
	u8 bias_low_power_request;
	u8 simo_int_ldo_always_on;
	u8 en_mode;
	u8 en_debounce_time;
};

struct max77675_regulator {
	struct device *dev;           // Device pointer for logging and PM
	struct regmap *regmap;        // Regmap for register access
	struct max77675_config config;
	struct max77675_regulator_pdata pdata[MAX77675_NUM_REGULATORS];
};

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
	val |= desc->enable_val;

	dev_info(maxreg->dev, "Modified enable_reg value: 0x%02x\n", val);

	ret = regmap_write(maxreg->regmap, desc->enable_reg, val);
	if (ret)
		dev_err(maxreg->dev, "Failed to write enable_reg (0x%02x): %d\n",
			desc->enable_reg, ret);
	else
		dev_info(maxreg->dev, "Successfully enabled regulator: %s\n", desc->name);

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
	val |= desc->disable_val;

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
	unsigned int voltage;

	// Use uV_step from regulator_desc
	unsigned int uV_step = rdev->desc->uV_step;

	// Calculate the voltage in microvolts based on the selector value
	voltage = selector * uV_step + rdev->desc->min_uV;

	return voltage;
}

/**
 * Set latency mode.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param enable true to enable latency mode, false to disable.
 */
static int max77675_set_latency_mode(struct max77675_regulator *maxreg, bool enable)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_SBB_TOP_B,
				  MAX77675_LAT_MODE_BIT,
				  FIELD_PREP(MAX77675_LAT_MODE_BIT, enable));
}

/**
 * Set DVS slew rate mode.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param enable true to use DVS-controlled slew rate, false for fixed 2mV/us.
 */
static int max77675_set_dvs_slew_rate(struct max77675_regulator *maxreg, bool enable)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_SBB_TOP_B,
				  MAX77675_DVS_SLEW_BIT,
				  FIELD_PREP(MAX77675_DVS_SLEW_BIT, enable));
}

/**
 * Set drive strength.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param strength 2-bit drive strength value (0-3).
 *
 * @return 0 on success, negative error code on failure.
 */
static int max77675_set_drv_sbb_strength(struct max77675_regulator *maxreg, u8 strength)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_SBB_TOP_A,
				  MAX77675_DRV_SBB_MASK,
				  FIELD_PREP(MAX77675_DRV_SBB_MASK, strength));
}

/**
 * Set manual reset time (MRT) for EN pin.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param mrt 2-bit value (0x0: 4s, 0x1: 8s, 0x2: 12s, 0x3: 16s)
 */
static int max77675_set_manual_reset_time(struct max77675_regulator *maxreg, u8 mrt)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_MRT_MASK,
				  FIELD_PREP(MAX77675_MRT_MASK, mrt));
}

/**
 * Enable or disable internal pull-up resistor on EN pin.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param disable true to disable pull-up, false to enable
 */
static int max77675_set_en_pullup_disable(struct max77675_regulator *maxreg, bool disable)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_PU_DIS_BIT,
				  FIELD_PREP(MAX77675_PU_DIS_BIT, disable));
}

/**
 * Request main bias to enter low-power mode.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param enable true to request low-power mode, false for normal
 */
static int max77675_set_bias_low_power_request(struct max77675_regulator *maxreg, bool enable)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_BIAS_LPM_BIT,
				  FIELD_PREP(MAX77675_BIAS_LPM_BIT, enable));
}

/**
 * Force SIMO internal LDO to always supply 1.8V.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param enable true to always supply 1.8V, false for normal operation
 */
static int max77675_set_simo_int_ldo_always_on(struct max77675_regulator *maxreg, bool enable)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_SIMO_CH_DIS_BIT,
				  FIELD_PREP(MAX77675_SIMO_CH_DIS_BIT, enable));
}

/**
 * Set EN pin mode.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param mode 2-bit value: 0x0 (push-button), 0x1 (slide-switch), 0x2 (logic)
 */
static int max77675_set_en_mode(struct max77675_regulator *maxreg, u8 mode)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_EN_MODE_MASK,
				  FIELD_PREP(MAX77675_EN_MODE_MASK, mode));
}

/**
 * Set debounce time for EN pin.
 *
 * @param maxreg Pointer to max77675 device structure.
 * @param debounce_30ms true for 30ms, false for 100us
 */
static int max77675_set_en_debounce_time(struct max77675_regulator *maxreg, bool debounce_30ms)
{
	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_GLBL_A,
				  MAX77675_DBEN_EN_BIT,
				  FIELD_PREP(MAX77675_DBEN_EN_BIT, debounce_30ms));
}

static int max77675_regulator_get_fps_src(struct max77675_regulator *maxreg, int id)
{
	unsigned int reg_addr;
	unsigned int val;
	int ret;

	switch (id) {
	case 0:
		reg_addr = MAX77675_REG_CNFG_SBB0_B;
		break;
	case 1:
		reg_addr = MAX77675_REG_CNFG_SBB1_B;
		break;
	case 2:
		reg_addr = MAX77675_REG_CNFG_SBB2_B;
		break;
	case 3:
		reg_addr = MAX77675_REG_CNFG_SBB3_B;
		break;
	default:
		dev_err(maxreg->dev, "Invalid regulator id: %d\n", id);
		return -EINVAL;
	}

	ret = regmap_read(maxreg->regmap, reg_addr, &val);
	if (ret < 0) {
		dev_err(maxreg->dev, "Failed to read FPS source (reg 0x%02x): %d\n",
			reg_addr, ret);
		return ret;
	}

	return val & MAX77675_EN_SBB_MASK;
}

static int max77675_regulator_set_fps_src(struct max77675_regulator *maxreg, int id, u8 fps_src)
{
	unsigned int reg_addr;
	int ret;

	switch (id) {
	case 0:
		reg_addr = MAX77675_REG_CNFG_SBB0_B;
		break;
	case 1:
		reg_addr = MAX77675_REG_CNFG_SBB1_B;
		break;
	case 2:
		reg_addr = MAX77675_REG_CNFG_SBB2_B;
		break;
	case 3:
		reg_addr = MAX77675_REG_CNFG_SBB3_B;
		break;
	default:
		dev_err(maxreg->dev, "Invalid regulator id: %d\n", id);
		return -EINVAL;
	}

	ret = regmap_update_bits(maxreg->regmap, reg_addr,
				 MAX77675_EN_SBB_MASK, fps_src);
	if (ret < 0) {
		dev_err(maxreg->dev, "Failed to set FPS source (reg 0x%02x): %d\n",
			reg_addr, ret);
		return ret;
	}

	return 0;
}

/**
 * max77675_set_sbb_slew_rate - Set the slew rate for a specific SBB regulator channel
 *
 * @maxreg: Pointer to the max77675 regulator structure
 * @id: Regulator channel ID (ID_SBB0 ~ ID_SBB3)
 * @val: Slew rate value (0 = 2mV/us, 1 = use DVS_SLEW)
 *
 * This function configures the slew rate control source for the specified SBB channel by
 * updating the corresponding bits in the CNFG_SBB_TOP_B register.
 *
 * Return: 0 on success, negative error code on failure (e.g., invalid channel ID).
 */
static int max77675_set_sbb_slew_rate(struct max77675_regulator *maxreg, int id, u8 val)
{
	u8 mask, value;

	switch (id) {
	case ID_SBB0:
		mask = MAX77675_SR_SBB0_BIT;
		value = FIELD_PREP(MAX77675_SR_SBB0_BIT, val);
		break;

	case ID_SBB1:
		mask = MAX77675_SR_SBB1_BIT;
		value = FIELD_PREP(MAX77675_SR_SBB1_BIT, val);
		break;

	case ID_SBB2:
		mask = MAX77675_SR_SBB2_BIT;
		value = FIELD_PREP(MAX77675_SR_SBB2_BIT, val);
		break;

	case ID_SBB3:
		mask = MAX77675_SR_SBB3_BIT;
		value = FIELD_PREP(MAX77675_SR_SBB3_BIT, val);
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(maxreg->regmap, MAX77675_REG_CNFG_SBB_TOP_B, mask, val);
}

static int max77675_init_regulator(struct max77675_regulator *maxreg, int id)
{
	struct max77675_regulator_pdata *rpdata = &maxreg->pdata[id];
	int ret;

	if (rpdata->fps_slot == MAX77675_FPS_DEF) {
		ret = max77675_regulator_get_fps_src(maxreg, id);
		if (ret < 0) {
			dev_err(maxreg->dev, "Failed to read FPS source for ID %d\n", id);
			return ret;
		}
		rpdata->fps_slot = ret;
	}

	ret = max77675_regulator_set_fps_src(maxreg, id, rpdata->fps_slot);
	if (ret)
		dev_warn(maxreg->dev, "Failed to set FPS source for ID %d\n", id);

	ret = max77675_set_sbb_slew_rate(maxreg, id, rpdata->dvs_slew_rate);
	if (ret)
		dev_warn(maxreg->dev, "Failed to set slew rate for ID %d\n", id);

	dev_dbg(maxreg->dev, "Regulator ID %d: FPS slot = %u, slew rate = %u\n",
		id, rpdata->fps_slot, rpdata->dvs_slew_rate);

	return 0;
}

static int max77675_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	struct max77675_regulator *maxreg = config->driver_data;
	struct max77675_regulator_pdata *rpdata = &maxreg->pdata[desc->id];
	u32 pval;
	int ret;

	/* Parse FPS slot from DT */
	ret = of_property_read_u32(np, "maxim,fps-slot", &pval);
	rpdata->fps_slot = (!ret) ? (u8)pval : MAX77675_FPS_DEF;

	/* Parse slew rate control source */
	ret = of_property_read_u32(np, "maxim,slew-rate-use-drv", &pval);
	rpdata->dvs_slew_rate = (!ret) ? (u8)pval : 0;

	dev_dbg(maxreg->dev, "Regulator %s: parsed fps_slot=%u, dvs_slew_rate=%u\n",
		desc->name, rpdata->fps_slot, rpdata->dvs_slew_rate);

	/* Apply parsed configuration */
	return max77675_init_regulator(maxreg, desc->id);
}

static int max77675_get_error_flags(struct regulator_dev *rdev, unsigned int *flags)
{
	struct max77675_regulator *maxreg = rdev_get_drvdata(rdev);
	unsigned int int_flags;
	int id = rdev_get_id(rdev);
	int ret;

	ret = regmap_read(maxreg->regmap, MAX77675_REG_INT_GLBL, &int_flags);
	if (ret) {
		dev_err(maxreg->dev, "Failed to read INT_GLBL: %d\n", ret);
		return ret;
	}

	*flags = 0;

	switch (id) {
	case 0:
		if (!(int_flags & MAX77675_INT_SBB0_F_BIT))
			*flags |= REGULATOR_ERROR_FAIL;
		break;
	case 1:
		if (!(int_flags & MAX77675_INT_SBB1_F_BIT))
			*flags |= REGULATOR_ERROR_FAIL;
		break;
	case 2:
		if (!(int_flags & MAX77675_INT_SBB2_F_BIT))
			*flags |= REGULATOR_ERROR_FAIL;
		break;
	case 3:
		if (!(int_flags & MAX77675_INT_SBB3_F_BIT))
			*flags |= REGULATOR_ERROR_FAIL;
		break;
	default:
		dev_warn(maxreg->dev, "Unsupported regulator ID: %d\n", id);
		break;
	}

	if (!(int_flags & MAX77675_INT_TJAL1_R_BIT) || !(int_flags & MAX77675_INT_TJAL2_R_BIT))
		*flags |= REGULATOR_ERROR_OVER_TEMP;

	dev_dbg(maxreg->dev, "%s: flags = 0x%x\n", __func__, *flags);

	return 0;
}

static const struct regulator_ops max77675_regulator_ops = {
	.list_voltage         = max77675_list_voltage,
	.enable               = max77675_regulator_enable,
	.disable              = max77675_regulator_disable,
	.is_enabled           = max77675_regulator_is_enabled,
	.list_voltage         = regulator_list_voltage_linear,
	.map_voltage          = regulator_map_voltage_linear,
	.set_voltage_sel      = regulator_set_voltage_sel_regmap,
	.get_voltage_sel      = regulator_get_voltage_sel_regmap,
	.set_active_discharge = regulator_set_active_discharge_regmap,
	.get_error_flags      = max77675_get_error_flags,
};

static struct regulator_desc max77675_regulators[MAX77675_NUM_REGULATORS] = {
	{
		.name                  = "sbb0",
		.of_match              = of_match_ptr("sbb0"),
		.regulators_node       = of_match_ptr("regulators"),
		.of_parse_cb           = max77675_of_parse_cb,
		.id                    = ID_SBB0,
		.ops                   = &max77675_regulator_ops,
		.type                  = REGULATOR_VOLTAGE,
		.owner                 = THIS_MODULE,
		.n_voltages            = MAX77675_NUM_LEVELS_25MV,
		.min_uV                = MAX77675_MIN_UV,
		.uV_step               = MAX77675_STEP_25MV,
		.vsel_reg              = MAX77675_REG_CNFG_SBB0_A,
		.vsel_mask             = MAX77675_TV_SBB0_MASK,
		.enable_reg            = MAX77675_REG_CNFG_SBB0_B,
		.enable_mask           = MAX77675_EN_SBB0_MASK,
		.enable_val            = MAX77675_ENABLE_ON,
		.disable_val           = MAX77675_ENABLE_OFF,
		.active_discharge_off  = MAX77675_REGULATOR_AD_OFF,
		.active_discharge_on   = MAX77675_REGULATOR_AD_ON,
		.active_discharge_mask = MAX77675_REGULATOR_AD_MASK,
		.active_discharge_reg  = MAX77675_REG_CNFG_SBB0_B,
	},
	{
		.name                  = "sbb1",
		.of_match              = of_match_ptr("sbb1"),
		.regulators_node       = of_match_ptr("regulators"),
		.of_parse_cb           = max77675_of_parse_cb,
		.id                    = ID_SBB1,
		.ops                   = &max77675_regulator_ops,
		.type                  = REGULATOR_VOLTAGE,
		.owner                 = THIS_MODULE,
		.n_voltages            = MAX77675_NUM_LEVELS_25MV,
		.min_uV                = MAX77675_MIN_UV,
		.uV_step               = MAX77675_STEP_25MV,
		.vsel_reg              = MAX77675_REG_CNFG_SBB1_A,
		.vsel_mask             = MAX77675_TV_SBB1_MASK,
		.enable_reg            = MAX77675_REG_CNFG_SBB1_B,
		.enable_mask           = MAX77675_EN_SBB1_MASK,
		.enable_val            = MAX77675_ENABLE_ON,
		.disable_val           = MAX77675_ENABLE_OFF,
		.active_discharge_off  = MAX77675_REGULATOR_AD_OFF,
		.active_discharge_on   = MAX77675_REGULATOR_AD_ON,
		.active_discharge_mask = MAX77675_REGULATOR_AD_MASK,
		.active_discharge_reg  = MAX77675_REG_CNFG_SBB1_B,
	},
	{
		.name                  = "sbb2",
		.of_match              = of_match_ptr("sbb2"),
		.regulators_node       = of_match_ptr("regulators"),
		.of_parse_cb           = max77675_of_parse_cb,
		.id                    = ID_SBB2,
		.ops                   = &max77675_regulator_ops,
		.type                  = REGULATOR_VOLTAGE,
		.owner                 = THIS_MODULE,
		.n_voltages            = MAX77675_NUM_LEVELS_25MV,
		.min_uV                = MAX77675_MIN_UV,
		.uV_step               = MAX77675_STEP_25MV,
		.vsel_reg              = MAX77675_REG_CNFG_SBB2_A,
		.vsel_mask             = MAX77675_TV_SBB2_MASK,
		.enable_reg            = MAX77675_REG_CNFG_SBB2_B,
		.enable_mask           = MAX77675_EN_SBB2_MASK,
		.enable_val            = MAX77675_ENABLE_ON,
		.disable_val           = MAX77675_ENABLE_OFF,
		.active_discharge_off  = MAX77675_REGULATOR_AD_OFF,
		.active_discharge_on   = MAX77675_REGULATOR_AD_ON,
		.active_discharge_mask = MAX77675_REGULATOR_AD_MASK,
		.active_discharge_reg  = MAX77675_REG_CNFG_SBB2_B,
	},
	{
		.name                  = "sbb3",
		.of_match              = of_match_ptr("sbb3"),
		.regulators_node       = of_match_ptr("regulators"),
		.of_parse_cb           = max77675_of_parse_cb,
		.id                    = ID_SBB3,
		.ops                   = &max77675_regulator_ops,
		.type                  = REGULATOR_VOLTAGE,
		.owner                 = THIS_MODULE,
		.n_voltages            = MAX77675_NUM_LEVELS_25MV,
		.min_uV                = MAX77675_MIN_UV,
		.uV_step               = MAX77675_STEP_25MV,
		.vsel_reg              = MAX77675_REG_CNFG_SBB3_A,
		.vsel_mask             = MAX77675_TV_SBB3_MASK,
		.enable_reg            = MAX77675_REG_CNFG_SBB3_B,
		.enable_mask           = MAX77675_EN_SBB3_MASK,
		.enable_val            = MAX77675_ENABLE_ON,
		.disable_val           = MAX77675_ENABLE_OFF,
		.active_discharge_off  = MAX77675_REGULATOR_AD_OFF,
		.active_discharge_on   = MAX77675_REGULATOR_AD_ON,
		.active_discharge_mask = MAX77675_REGULATOR_AD_MASK,
		.active_discharge_reg  = MAX77675_REG_CNFG_SBB3_B,
	},
};

static const struct regmap_config max77675_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77675_MAX_REGISTER,
	.cache_type = REGCACHE_RBTREE,
};

static int max77675_apply_config(struct max77675_regulator *maxreg,
				 const struct max77675_config *config)
{
	int ret;

	ret = max77675_set_dvs_slew_rate(maxreg, config->drv_slew_rate);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set DVS slew rate: %d\n", ret);
		return ret;
	}

	ret = max77675_set_latency_mode(maxreg, config->latency_mode);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set latency mode: %d\n", ret);
		return ret;
	}

	ret = max77675_set_drv_sbb_strength(maxreg, config->drv_sbb_strength);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set drive strength: %d\n", ret);
		return ret;
	}

	ret = max77675_set_manual_reset_time(maxreg, config->manual_reset_time);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set manual reset time: %d\n", ret);
		return ret;
	}

	ret = max77675_set_en_pullup_disable(maxreg, config->en_pullup_disable);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set EN pull-up disable: %d\n", ret);
		return ret;
	}

	ret = max77675_set_bias_low_power_request(maxreg, config->bias_low_power_request);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set bias low-power request: %d\n", ret);
		return ret;
	}

	ret = max77675_set_simo_int_ldo_always_on(maxreg, config->simo_int_ldo_always_on);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set SIMO internal LDO always-on: %d\n", ret);
		return ret;
	}

	ret = max77675_set_en_mode(maxreg, config->en_mode);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set EN mode: %d\n", ret);
		return ret;
	}

	ret = max77675_set_en_debounce_time(maxreg, config->en_debounce_time);
	if (ret) {
		dev_err(maxreg->dev, "Failed to set EN debounce time: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77675_parse_config(struct device_node *np, struct max77675_config *config)
{
	int ret;
	u32 val;

	/* drv slew rate */
	ret = of_property_read_u32(np, "maxim,dvs-slew-rate", &val);
	if (ret < 0)
		val = 0;
	config->drv_slew_rate = val ? 1 : 0;

	/* latency mode */
	ret = of_property_read_u32(np, "maxim,latency-mode", &val);
	if (ret < 0)
		val = 0;
	config->latency_mode = val ? 1 : 0;

	/* drive strength */
	ret = of_property_read_u32(np, "maxim,drv-sbb-strength", &val);
	if (ret < 0)
		val = 0;
	config->drv_sbb_strength = val & 0x3;

	/* Manual reset time for EN pin */
	ret = of_property_read_u32(np, "maxim,manual-reset-time", &val);
	if (ret < 0)
		val = 0;
	config->manual_reset_time = val & 0x3;

	/* Disable internal pull-up resistor on EN pin */
	ret = of_property_read_u32(np, "maxim,en-pullup-disable", &val);
	if (ret < 0)
		val = 0;
	config->en_pullup_disable = val ? 1 : 0;

	/* Request low-power mode for main bias */
	ret = of_property_read_u32(np, "maxim,bias-low-power-request", &val);
	if (ret < 0)
		val = 0;
	config->bias_low_power_request = val ? 1 : 0;

	/* Force internal LDO to always supply 1.8V */
	ret = of_property_read_u32(np, "maxim,simo-int-ldo-always-on", &val);
	if (ret < 0)
		val = 0;
	config->simo_int_ldo_always_on = val ? 1 : 0;

	/* EN pin mode: push-button, slide-switch, or logic */
	ret = of_property_read_u32(np, "maxim,en-mode", &val);
	if (ret < 0)
		val = 0;
	config->en_mode = val & 0x3;

	/* Debounce time for EN pin */
	ret = of_property_read_u32(np, "maxim,en-debounce-time", &val);
	if (ret < 0)
		val = 0;
	config->en_debounce_time = val ? 1 : 0;

	/* Debug print all parsed values */
	pr_info("MAX77675 config parsed:\n"
		 "  dvs_slew_rate: %u\n"
		 "  latency_mode: %u\n"
		 "  drv_sbb_strength: %u\n"
		 "  manual_reset_time: %u\n"
		 "  en_pullup_disable: %u\n"
		 "  bias_low_power_request: %u\n"
		 "  simo_int_ldo_always_on: %u\n"
		 "  en_mode: %u\n"
		 "  en_debounce_time: %u\n",
		 config->drv_slew_rate,
		 config->latency_mode,
		 config->drv_sbb_strength,
		 config->manual_reset_time,
		 config->en_pullup_disable,
		 config->bias_low_power_request,
		 config->simo_int_ldo_always_on,
		 config->en_mode,
		 config->en_debounce_time);

	return 0;
}

static int max77675_init_event(struct max77675_regulator *maxreg)
{
	unsigned int ercflag, int_glbl;
	int ret;

	ret = regmap_read(maxreg->regmap, MAX77675_REG_ERCF_GLBL, &ercflag);
	if (ret) {
		dev_err(maxreg->dev, "Failed to read CID register: %d\n", ret);
		return ret;
	}

	ret = regmap_read(maxreg->regmap, MAX77675_REG_INT_GLBL, &int_glbl);
	if (ret) {
		dev_err(maxreg->dev, "Failed to read INT_GLBL register: %d\n", ret);
		return ret;
	}

	if (ercflag & MAX77675_SFT_CRST_F_BIT)
		dev_info(maxreg->dev, "Software Cold Reset Flag is set\n");

	if (ercflag & MAX77675_SFT_OFF_F_BIT)
		dev_info(maxreg->dev, "Software Off Flag is set\n");

	if (ercflag & MAX77675_MRST_BIT)
		dev_info(maxreg->dev, "Manual Reset Timer Flag is set\n");

	if (ercflag & MAX77675_UVLO_BIT)
		dev_info(maxreg->dev, "Undervoltage Lockout Flag is set\n");

	if (ercflag & MAX77675_OVLO_BIT)
		dev_info(maxreg->dev, "Overvoltage Lockout Flag is set\n");

	if (ercflag & MAX77675_TOVLD_BIT)
		dev_info(maxreg->dev, "Thermal Overload Flag is set\n");

	if (int_glbl & MAX77675_INT_SBB3_F_BIT)
		dev_info(maxreg->dev, "SBB3 Channel Fault Interrupt occurred\n");

	if (int_glbl & MAX77675_INT_SBB2_F_BIT)
		dev_info(maxreg->dev, "SBB2 Channel Fault Interrupt occurred\n");

	if (int_glbl & MAX77675_INT_SBB1_F_BIT)
		dev_info(maxreg->dev, "SBB1 Channel Fault Interrupt occurred\n");

	if (int_glbl & MAX77675_INT_SBB0_F_BIT)
		dev_info(maxreg->dev, "SBB0 Channel Fault Interrupt occurred\n");

	if (int_glbl & MAX77675_INT_TJAL2_R_BIT)
		dev_info(maxreg->dev, "Thermal Alarm 2 Rising Interrupt occurred\n");

	if (int_glbl & MAX77675_INT_TJAL1_R_BIT)
		dev_info(maxreg->dev, "Thermal Alarm 1 Rising Interrupt occurred\n");

	if (int_glbl & MAX77675_INT_EN_R_BIT)
		dev_info(maxreg->dev, "nEN Rising Edge Interrupt occurred\n");

	if (int_glbl & MAX77675_INT_EN_F_BIT)
		dev_info(maxreg->dev, "nEN Falling Edge Interrupt occurred\n");

	return 0;
}

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
		dev_err(maxreg->dev, "Failed to init regmap\n");
		return PTR_ERR(maxreg->regmap);
	}

	ret = max77675_init_event(maxreg);
	if (ret) {
		dev_err(maxreg->dev, "Failed to init event flag for regulator\n");
		return ret;
	}

	// global config parsing
	ret = max77675_parse_config(client->dev.of_node, &maxreg->config);
	if (ret)
		dev_warn(maxreg->dev, "Failed to parse global config, using defaults\n");

	ret = max77675_apply_config(maxreg, &maxreg->config);
	if (ret)
		dev_warn(maxreg->dev, "Failed to apply config: %d\n", ret);

	config.dev = &client->dev;
	config.regmap = maxreg->regmap;
	config.driver_data = maxreg;

	regulators_np = of_get_child_by_name(client->dev.of_node, "regulators");
	if (!regulators_np) {
		dev_err(maxreg->dev, "No 'regulators' subnode found in DT\n");
		return -EINVAL;
	}

	for (i = 0; i < MAX77675_NUM_REGULATORS; i++) {
		const struct regulator_desc *desc = &max77675_regulators[i];
		struct regulator_dev *rdev;
		struct device_node *child_np;

		child_np = of_get_child_by_name(regulators_np, desc->name);
		if (!child_np) {
			dev_warn(maxreg->dev, "No DT node for regulator %s\n", desc->name);
			continue;
		}

		config.of_node = child_np;

		rdev = devm_regulator_register(&client->dev, desc, &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(maxreg->dev,
				"Failed to register regulator %d (%s): %d\n",
				i, desc->name, ret);
			of_node_put(child_np);
			return ret;
		}
		of_node_put(child_np);
	}

	i2c_set_clientdata(client, maxreg);

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
MODULE_AUTHOR("Joan Na <joan.na@analog.com>");
MODULE_LICENSE("GPL");
