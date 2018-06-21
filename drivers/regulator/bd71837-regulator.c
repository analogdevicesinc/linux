/*
 * @file bd71837-regulator.c ROHM BD71837MWV regulator driver
 *
 * @author: cpham2403@gmail.com
 * Copyright 2017.
 *
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mfd/bd71837.h>
#include <linux/regulator/of_regulator.h>

#define BD71837_DVS_BUCK_NUM		4	/* Buck 1/2/3/4 support DVS */
#define BD71837_DVS_RUN_IDLE_SUSP	3
#define BD71837_DVS_RUN_IDLE		2
#define BD71837_DVS_RUN				1

struct bd71837_buck_dvs {
	u32 voltage[BD71837_DVS_RUN_IDLE_SUSP];
};

/** @brief bd71837 regulator type */
struct bd71837_pmic {
	struct regulator_desc descs[BD71837_REGULATOR_CNT];	/**< regulator description to system */
	struct bd71837 *mfd;									/**< parent device */
	struct device *dev;										/**< regulator kernel device */
	struct regulator_dev *rdev[BD71837_REGULATOR_CNT];		/**< regulator device of system */
	struct bd71837_buck_dvs buck_dvs[BD71837_DVS_BUCK_NUM];			/**< buck1/2 dvs */
	int	reg_index;
};

/*
 * BUCK1/2/3/4
 * BUCK1RAMPRATE[1:0] BUCK1 DVS ramp rate setting
 * 00: 10.00mV/usec 10mV 1uS
 * 01: 5.00mV/usec	10mV 2uS
 * 10: 2.50mV/usec	10mV 4uS
 * 11: 1.25mV/usec	10mV 8uS
 */
static int bd71837_buck1234_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	struct bd71837_pmic *pmic = rdev_get_drvdata(rdev);
	struct bd71837 *mfd = pmic->mfd;
	int id = rdev->desc->id;
	unsigned int ramp_value = BUCK1_RAMPRATE_10P00MV;

	dev_dbg(pmic->dev, "Buck[%d] Set Ramp = %d\n", id + 1, ramp_delay);
	switch (ramp_delay) {
	case 1 ... 1250:
		ramp_value = BUCK1_RAMPRATE_1P25MV;
		break;
	case 1251 ... 2500:
		ramp_value = BUCK1_RAMPRATE_2P50MV;
		break;
	case 2501 ... 5000:
		ramp_value = BUCK1_RAMPRATE_5P00MV;
		break;
	case 5001 ... 10000:
		ramp_value = BUCK1_RAMPRATE_10P00MV;
		break;
	default:
		ramp_value = BUCK1_RAMPRATE_10P00MV;
		dev_err(pmic->dev, "%s: ramp_delay: %d not supported, setting 10000mV//us\n",
			rdev->desc->name, ramp_delay);
	}

	return regmap_update_bits(mfd->regmap, BD71837_REG_BUCK1_CTRL + id,
			BUCK1_RAMPRATE_MASK, ramp_value << 6);
}

static struct regulator_ops bd71837_ldo_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops bd71837_fixed_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

static struct regulator_ops bd71837_buck_regulator_ops = {
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static struct regulator_ops bd71837_buck1234_regulator_ops = {
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd71837_buck1234_set_ramp_delay,
};

/*
 * BUCK1/2/3/4
 * 0.70 to 1.30V (10mV step)
 */
static const struct regulator_linear_range bd71837_buck1234_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(700000,  0x00, 0x3C, 10000),
	REGULATOR_LINEAR_RANGE(1300000,  0x3D, 0x3F, 0),
};

/*
 * BUCK5
 * 0.9V to 1.35V ()
 */
static const struct regulator_linear_range bd71837_buck5_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(700000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(1050000, 0x04, 0x05, 50000),
	REGULATOR_LINEAR_RANGE(1200000, 0x06, 0x07, 150000),
};

/*
 * BUCK6
 * 3.0V to 3.3V (step 100mV)
 */
static const struct regulator_linear_range bd71837_buck6_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
};

/*
 * BUCK7
 * 1.605V to 1.995V ()
 * 000 = 1.605V
 * 001 = 1.695V
 * 010 = 1.755V
 * 011 = 1.8V (Initial)
 * 100 = 1.845V
 * 101 = 1.905V
 * 110 = 1.95V
 * 111 = 1.995V
 */
static const struct regulator_linear_range bd71837_buck7_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1605000, 0x00, 0x01, 90000),
	REGULATOR_LINEAR_RANGE(1755000, 0x02, 0x03, 45000),
	REGULATOR_LINEAR_RANGE(1845000, 0x04, 0x05, 60000),
	REGULATOR_LINEAR_RANGE(1950000, 0x06, 0x07, 45000),
};

/*
 * BUCK8
 * 0.8V to 1.40V (step 10mV)
 */
static const struct regulator_linear_range bd71837_buck8_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x3C, 10000),
	REGULATOR_LINEAR_RANGE(1400000,  0x3D, 0x3F, 0),
};

/*
 * LDO1
 * 3.0 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
};

/*
 * LDO3
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo3_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

/*
 * LDO4
 * 0.9 to 1.8V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo4_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(900000, 0x00, 0x09, 100000),
	REGULATOR_LINEAR_RANGE(1800000, 0x0A, 0x0F, 0),
};

/*
 * LDO5
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo5_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000,  0x00, 0x0F, 100000),
};

/*
 * LDO6
 * 0.9 to 1.8V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo6_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(900000,  0x00, 0x09, 100000),
	REGULATOR_LINEAR_RANGE(1800000, 0x0A, 0x0F, 0),
};

/*
 * LDO7
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo7_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

static const struct regulator_desc bd71837_regulators[] = {
	{
		.name = "BUCK1",
		.id = BD71837_BUCK1,
		.ops = &bd71837_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK1_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck1234_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK1_VOLT_RUN,
		.vsel_mask = BUCK1_RUN_MASK,
		.enable_reg = BD71837_REG_BUCK1_CTRL,
		.enable_mask = BUCK1_SEL|BUCK1_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK2",
		.id = BD71837_BUCK2,
		.ops = &bd71837_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK2_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck1234_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK2_VOLT_RUN,
		.vsel_mask = BUCK2_RUN_MASK,
		.enable_reg = BD71837_REG_BUCK2_CTRL,
		.enable_mask = BUCK2_SEL|BUCK2_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK3",
		.id = BD71837_BUCK3,
		.ops = &bd71837_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK3_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck1234_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK3_VOLT_RUN,
		.vsel_mask = BUCK3_RUN_MASK,
		.enable_reg = BD71837_REG_BUCK3_CTRL,
		.enable_mask = BUCK3_SEL|BUCK3_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK4",
		.id = BD71837_BUCK4,
		.ops = &bd71837_buck1234_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK4_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck1234_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck1234_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK4_VOLT_RUN,
		.vsel_mask = BUCK4_RUN_MASK,
		.enable_reg = BD71837_REG_BUCK4_CTRL,
		.enable_mask = BUCK4_SEL|BUCK4_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK5",
		.id = BD71837_BUCK5,
		.ops = &bd71837_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK5_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck5_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck5_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK5_VOLT,
		.vsel_mask = BUCK5_MASK,
		.enable_reg = BD71837_REG_BUCK5_CTRL,
		.enable_mask = BUCK5_SEL|BUCK5_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK6",
		.id = BD71837_BUCK6,
		.ops = &bd71837_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK6_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck6_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck6_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK6_VOLT,
		.vsel_mask = BUCK6_MASK,
		.enable_reg = BD71837_REG_BUCK6_CTRL,
		.enable_mask = BUCK6_SEL|BUCK6_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK7",
		.id = BD71837_BUCK7,
		.ops = &bd71837_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK7_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck7_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck7_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK7_VOLT,
		.vsel_mask = BUCK7_MASK,
		.enable_reg = BD71837_REG_BUCK7_CTRL,
		.enable_mask = BUCK7_SEL|BUCK7_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK8",
		.id = BD71837_BUCK8,
		.ops = &bd71837_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_BUCK8_VOLTAGE_NUM,
		.linear_ranges = bd71837_buck8_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_buck8_voltage_ranges),
		.vsel_reg = BD71837_REG_BUCK8_VOLT,
		.vsel_mask = BUCK8_MASK,
		.enable_reg = BD71837_REG_BUCK8_CTRL,
		.enable_mask = BUCK8_SEL|BUCK8_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO1",
		.id = BD71837_LDO1,
		.ops = &bd71837_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_LDO1_VOLTAGE_NUM,
		.linear_ranges = bd71837_ldo1_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_ldo1_voltage_ranges),
		.vsel_reg = BD71837_REG_LDO1_VOLT,
		.vsel_mask = LDO1_MASK,
		.enable_reg = BD71837_REG_LDO1_VOLT,
		.enable_mask = LDO1_EN,
		.owner = THIS_MODULE,
	},
	/*
	 * LDO2 0.9V
	 * Fixed voltage
	 */
	{
		.name = "LDO2",
		.id = BD71837_LDO2,
		.ops = &bd71837_fixed_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_LDO2_VOLTAGE_NUM,
		.min_uV = 900000,
		.enable_reg = BD71837_REG_LDO2_VOLT,
		.enable_mask = LDO2_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO3",
		.id = BD71837_LDO3,
		.ops = &bd71837_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_LDO3_VOLTAGE_NUM,
		.linear_ranges = bd71837_ldo3_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_ldo3_voltage_ranges),
		.vsel_reg = BD71837_REG_LDO3_VOLT,
		.vsel_mask = LDO3_MASK,
		.enable_reg = BD71837_REG_LDO3_VOLT,
		.enable_mask = LDO3_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO4",
		.id = BD71837_LDO4,
		.ops = &bd71837_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_LDO4_VOLTAGE_NUM,
		.linear_ranges = bd71837_ldo4_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_ldo4_voltage_ranges),
		.vsel_reg = BD71837_REG_LDO4_VOLT,
		.vsel_mask = LDO4_MASK,
		.enable_reg = BD71837_REG_LDO4_VOLT,
		.enable_mask = LDO4_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO5",
		.id = BD71837_LDO5,
		.ops = &bd71837_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_LDO5_VOLTAGE_NUM,
		.linear_ranges = bd71837_ldo5_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_ldo5_voltage_ranges),
		.vsel_reg = BD71837_REG_LDO5_VOLT,
		.vsel_mask = LDO5_MASK,
		.enable_reg = BD71837_REG_LDO5_VOLT,
		.enable_mask = LDO5_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO6",
		.id = BD71837_LDO6,
		.ops = &bd71837_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_LDO6_VOLTAGE_NUM,
		.linear_ranges = bd71837_ldo6_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_ldo6_voltage_ranges),
		.vsel_reg = BD71837_REG_LDO6_VOLT,
		.vsel_mask = LDO6_MASK,
		.enable_reg = BD71837_REG_LDO6_VOLT,
		.enable_mask = LDO6_EN,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO7",
		.id = BD71837_LDO7,
		.ops = &bd71837_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71837_LDO7_VOLTAGE_NUM,
		.linear_ranges = bd71837_ldo7_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71837_ldo7_voltage_ranges),
		.vsel_reg = BD71837_REG_LDO7_VOLT,
		.vsel_mask = LDO7_MASK,
		.enable_reg = BD71837_REG_LDO7_VOLT,
		.enable_mask = LDO7_EN,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_OF

static struct of_regulator_match bd71837_matches[] = {
	{ .name = "buck1",	},
	{ .name = "buck2",	},
	{ .name = "buck3",	},
	{ .name = "buck4",	},
	{ .name = "buck5",	},
	{ .name = "buck6",	},
	{ .name = "buck7",	},
	{ .name = "buck8",	},
	{ .name = "ldo1",	},
	{ .name = "ldo2",	},
	{ .name = "ldo3",	},
	{ .name = "ldo4",	},
	{ .name = "ldo5",	},
	{ .name = "ldo6",	},
	{ .name = "ldo7",	},
};

/**@brief parse bd71837 regulator device tree
 * @param pdev platform device of bd71837 regulator
 * @param bd71837_reg_matches return regualtor matches
 * @retval 0 parse success
 * @retval NULL parse fail
 */
static int bd71837_parse_dt_reg_data(
		struct platform_device *pdev,
		struct of_regulator_match **reg_matches)
{
	// struct bd71837 *bd71837 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np, *regulators;
	struct of_regulator_match *matches;
	int ret, count;

	np = of_node_get(pdev->dev.parent->of_node);
	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&pdev->dev, "regulator node not found\n");
		return -EINVAL;
	}

	count = ARRAY_SIZE(bd71837_matches);
	matches = bd71837_matches;

	ret = of_regulator_match(&pdev->dev, regulators, matches, count);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return ret;
	}

	*reg_matches = matches;

	return 0;
}
#else
static inline int bd71837_parse_dt_reg_data(
			struct platform_device *pdev,
			struct of_regulator_match **reg_matches)
{
	*reg_matches = NULL;
	return 0;
}
#endif

/** @brief retrive out32k output value */
static ssize_t show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd71837_pmic *pmic = dev_get_drvdata(dev);
	int o;

	o = bd71837_reg_read(pmic->mfd, BD71837_REG_OUT32K);
	o = (o & OUT32K_EN) != 0;

	return sprintf(buf, "%d\n", o);
}

/** @brief set o output value */
static ssize_t set_value(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd71837_pmic *pmic = dev_get_drvdata(dev);
	int o, r;

	if (sscanf(buf, "%d", &o) < 1) {
		return -EINVAL;
	}

	if (o != 0) {
		o = OUT32K_EN;
	}
	r = bd71837_update_bits(pmic->mfd, BD71837_REG_OUT32K, OUT32K_EN, o);
	if (r < 0) {
		return r;
	}
	return count;
}

/** @brief list all supported values */
static ssize_t available_values(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0 1 \n");
}

/** @brief directly set raw value to chip register, format: 'register value' */
static ssize_t bd71837_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct bd71837_pmic *pmic = dev_get_drvdata(dev);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pmic->reg_index = -1;
		dev_err(pmic->dev, "registers set: <reg> <value>\n");
		return count;
	}

	if (ret == 1 && reg < BD71837_MAX_REGISTER) {
		pmic->reg_index = reg;
		dev_info(pmic->dev, "registers set: reg=0x%x\n", reg);
		return count;
	}

	if (reg > BD71837_MAX_REGISTER) {
		dev_err(pmic->dev, "reg=%d out of Max=%d\n", reg, BD71837_MAX_REGISTER);
		return -EINVAL;
	}
	dev_info(pmic->dev, "registers set: reg=0x%x, val=0x%x\n", reg, val);
	ret = bd71837_reg_write(pmic->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd71837_sysfs_print_reg(struct bd71837_pmic *pmic,
				       u8 reg,
				       char *buf)
{
	int ret = bd71837_reg_read(pmic->mfd, reg);
	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd71837_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct bd71837_pmic *pmic = dev_get_drvdata(dev);
	ssize_t ret = 0;
	int i;

	dev_info(pmic->dev, "register: index[0x%x]\n", pmic->reg_index);
	if (pmic->reg_index >= 0) {
		ret += bd71837_sysfs_print_reg(pmic, pmic->reg_index, buf + ret);
	} else {
		for (i = 0; i < BD71837_MAX_REGISTER; i++) {
			ret += bd71837_sysfs_print_reg(pmic, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(out32k_value, S_IWUSR | S_IRUGO, show_value, set_value);
static DEVICE_ATTR(available_value, S_IWUSR | S_IRUGO, available_values, NULL);
static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd71837_sysfs_show_registers, bd71837_sysfs_set_registers);

/** @brief device sysfs attribute table, about o */
static struct attribute *clk_attributes[] = {
	&dev_attr_out32k_value.attr,
	&dev_attr_available_value.attr,
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group clk_attr_group = {
	.attrs	= clk_attributes,
};

/*----------------------------------------------------------------------*/
#ifdef CONFIG_OF
/** @brief buck1/2 dvs enable/voltage from device tree
 * @param pdev platfrom device pointer
 * @param buck_dvs pointer
 * @return void
 */
static void of_bd71837_buck_dvs(struct platform_device *pdev, struct bd71837_buck_dvs *buck_dvs)
{
	struct device_node *pmic_np;

	pmic_np = of_node_get(pdev->dev.parent->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return;
	}

	if (of_get_property(pmic_np, "bd71837,pmic-buck1-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71837,pmic-buck1-dvs-voltage",
							&buck_dvs[0].voltage[0], BD71837_DVS_RUN_IDLE_SUSP)) {
			dev_err(&pdev->dev, "buck1 voltages not specified\n");
		}
	}

	if (of_get_property(pmic_np, "bd71837,pmic-buck2-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71837,pmic-buck2-dvs-voltage",
						&buck_dvs[1].voltage[0], BD71837_DVS_RUN_IDLE)) {
			dev_err(&pdev->dev, "buck2 voltages not specified\n");
		}
	}

	if (of_get_property(pmic_np, "bd71837,pmic-buck3-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71837,pmic-buck3-dvs-voltage",
						&buck_dvs[2].voltage[0], BD71837_DVS_RUN)) {
			dev_err(&pdev->dev, "buck3 voltages not specified\n");
		}
	}
	if (of_get_property(pmic_np, "bd71837,pmic-buck4-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71837,pmic-buck4-dvs-voltage",
						&buck_dvs[3].voltage[0], BD71837_DVS_RUN)) {
			dev_err(&pdev->dev, "buck4 voltages not specified\n");
		}
	}
}
#else
static void of_bd71837_buck_dvs(struct platform_device *pdev, struct bd71837_buck_dvs *buck_dvs)
{
	buck_dvs[0].voltage[0] = BUCK1_RUN_DEFAULT;
	buck_dvs[0].voltage[1] = BUCK1_IDLE_DEFAULT;
	buck_dvs[0].voltage[2] = BUCK1_SUSP_DEFAULT;
	buck_dvs[1].voltage[0] = BUCK2_RUN_DEFAULT;
	buck_dvs[1].voltage[1] = BUCK2_IDLE_DEFAULT;
	buck_dvs[1].voltage[2] = 0; /* Not supported */
	buck_dvs[2].voltage[0] = BUCK3_RUN_DEFAULT;
	buck_dvs[2].voltage[1] = 0; /* Not supported */
	buck_dvs[2].voltage[2] = 0; /* Not supported */
	buck_dvs[3].voltage[0] = BUCK4_RUN_DEFAULT;
	buck_dvs[3].voltage[1] = 0; /* Not supported */
	buck_dvs[3].voltage[2] = 0; /* Not supported */
}
#endif

static int bd71837_buck1234_dvs_init(struct bd71837_pmic *pmic)
{
	struct bd71837 *bd71837 = pmic->mfd;
	struct bd71837_buck_dvs *buck_dvs = &pmic->buck_dvs[0];
	int i, ret, val, selector = 0;
	u8 reg_run, reg_idle, reg_susp;
	u8 reg_run_msk, reg_idle_msk, reg_susp_msk;

	for (i = 0; i < BD71837_DVS_BUCK_NUM; i++, buck_dvs++) {
		switch (i) {
		case 0:
		default:
			reg_run = BD71837_REG_BUCK1_VOLT_RUN;
			reg_run_msk = BUCK1_RUN_MASK;
			reg_idle = BD71837_REG_BUCK1_VOLT_IDLE;
			reg_idle_msk = BUCK1_IDLE_MASK;
			reg_susp = BD71837_REG_BUCK1_VOLT_SUSP;
			reg_susp_msk = BUCK1_SUSP_MASK;
			break;
		case 1:
			reg_run = BD71837_REG_BUCK2_VOLT_RUN;
			reg_run_msk = BUCK2_RUN_MASK;
			reg_idle = BD71837_REG_BUCK2_VOLT_IDLE;
			reg_idle_msk = BUCK2_IDLE_MASK;
			reg_susp = 0;
			break;
		case 2:
			reg_run = BD71837_REG_BUCK3_VOLT_RUN;
			reg_run_msk = BUCK3_RUN_MASK;
			reg_idle = 0;
			reg_susp = 0;
			break;
		case 3:
			reg_run = BD71837_REG_BUCK4_VOLT_RUN;
			reg_run_msk = BUCK4_RUN_MASK;
			reg_idle = 0;
			reg_susp = 0;
			break;
		}

		dev_dbg(pmic->dev, "Buck%d: DVS Run-Idle-Susp[%d - %d - %d].\n", i+1, buck_dvs->voltage[0], buck_dvs->voltage[1], buck_dvs->voltage[2]);
		if (reg_run > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[0], buck_dvs->voltage[0]);
			if (selector < 0) {
				dev_dbg(pmic->dev, "%s(): not found selector for Run voltage [%d]\n", __func__, buck_dvs->voltage[0]);
			} else {
				val = (selector & reg_run_msk);
				ret = bd71837_reg_write(bd71837, reg_run, val);
				if (ret < 0)
					return ret;
			}
		}
		if (reg_idle > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[1], buck_dvs->voltage[1]);
			if (selector < 0) {
				dev_dbg(pmic->dev, "%s(): not found selector for Idle voltage [%d]\n", __func__, buck_dvs->voltage[1]);
			} else {
				val = (selector & reg_idle_msk);
				ret = bd71837_reg_write(bd71837, reg_idle, val);
				if (ret < 0)
					return ret;
			}
		}
		if (reg_susp > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[2], buck_dvs->voltage[2]);
			if (selector < 0) {
				dev_dbg(pmic->dev, "%s(): not found selector for Susp voltage [%d]\n", __func__, buck_dvs->voltage[2]);
			} else {
				val = (selector & reg_susp_msk);
				ret = bd71837_reg_write(bd71837, reg_susp, val);
				if (ret < 0)
					return ret;
			}
		}
	}
	return 0;
}

/**@brief bd71837 pmic interrupt
 * @param irq system irq
 * @param pwrsys bd71837 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd71837_pmic_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71837 *mfd = dev_get_drvdata(dev->parent);
	// struct bd71837_power *pwr = dev_get_drvdata(dev);
	int reg;

	bd71837_debug(BD71837_DBG0, "bd71837_pmic_interrupt() in.\n");

	reg = bd71837_reg_read(mfd, BD71837_REG_IRQ);
	if (reg < 0)
		return IRQ_NONE;

	if (reg & IRQ_SWRST) {
		bd71837_debug(BD71837_DBG0, "IRQ_SWRST\n");
	}
	if (reg & IRQ_PWRON_S) {
		bd71837_debug(BD71837_DBG0, "IRQ_PWRON_S\n");
	}
	if (reg & IRQ_PWRON_L) {
		bd71837_debug(BD71837_DBG0, "IRQ_PWRON_L\n");
	}
	if (reg & IRQ_PWRON) {
		bd71837_debug(BD71837_DBG0, "IRQ_PWRON\n");
	}
	if (reg & IRQ_WDOG) {
		bd71837_debug(BD71837_DBG0, "IRQ_WDOG\n");
	}
	if (reg & IRQ_ON_REQ) {
		bd71837_debug(BD71837_DBG0, "IRQ_ON_REQ\n");
	}
	if (reg & IRQ_STBY_REQ) {
		bd71837_debug(BD71837_DBG0, "IRQ_STBY_REQ\n");
	}

	reg = bd71837_reg_write(mfd, BD71837_REG_IRQ, reg);
	if (reg < 0)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

/**@brief probe bd71837 regulator device
 @param pdev bd71837 regulator platform device
 @retval 0 success
 @retval negative fail
*/
static int bd71837_probe(struct platform_device *pdev)
{
	struct bd71837_pmic *pmic;
	struct bd71837_board *pdata;
	struct regulator_config config = {};
	struct bd71837 *bd71837 = dev_get_drvdata(pdev->dev.parent);
	struct of_regulator_match *matches = NULL;
	int i = 0, err, irq = 0, ret = 0;

	pmic = kzalloc(sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(&pdev->dev, "Memory allocation failed for pmic\n");
		return -ENOMEM;
	}

	memcpy(pmic->descs, bd71837_regulators,	sizeof(pmic->descs));

	pmic->dev = &pdev->dev;
	pmic->mfd = bd71837;
	platform_set_drvdata(pdev, pmic);
	pdata = dev_get_platdata(bd71837->dev);

	if (!pdata && bd71837->dev->of_node) {
		bd71837_parse_dt_reg_data(pdev,	&matches);
		if (matches == NULL) {
			dev_err(&pdev->dev, "Platform data not found\n");
			return -EINVAL;
		}
	}

	/* Get buck dvs parameters */
	of_bd71837_buck_dvs(pdev, &pmic->buck_dvs[0]);

	/* Register LOCK release */
	err = bd71837_reg_write(bd71837, BD71837_REG_REGLOCK, 0x0);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to write LOCK register(%d)\n", err);
		goto err;
	}

	for (i = 0; i < BD71837_REGULATOR_CNT; i++) {
		struct regulator_init_data *init_data;
		struct regulator_desc *desc;
		struct regulator_dev *rdev;

		desc = &pmic->descs[i];
		desc->name = bd71837_matches[i].name;

		if (pdata) {
			init_data = pdata->init_data[i];
		} else {
			init_data = matches[i].init_data;
		}

		config.dev = pmic->dev;
		config.init_data = init_data;
		config.driver_data = pmic;
		config.regmap = bd71837->regmap;
		config.of_node = matches[i].of_node;
		dev_info(config.dev, "regulator register name '%s'\n", desc->name);

		rdev = regulator_register(desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(bd71837->dev,
				"failed to register %s regulator\n",
				desc->name);
			err = PTR_ERR(rdev);
			goto err;
		}
		pmic->rdev[i] = rdev;
	}

	/* Init sysfs registers */
	pmic->reg_index = -1;

	err = sysfs_create_group(&pdev->dev.kobj, &clk_attr_group);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to create attribute group: %d\n", err);
		goto err;
	}

	/* Init Buck1/2/3/4 dvs */
	err = bd71837_buck1234_dvs_init(pmic);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to buck12 dvs: %d\n", err);
		goto err;
	}

	/* Add Interrupt */
	irq  = platform_get_irq(pdev, 0); // get irq number
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			bd71837_pmic_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* Un-mask IRQ Interrupt */
	ret = bd71837_reg_write(bd71837, BD71837_REG_MIRQ, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Write Un-mask 'BD71837_REG_MIRQ': failed!\n");
		ret = -EIO;
		goto err;
	}

	return 0;

err:
	while (--i >= 0)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return err;
}

/**@brief remove bd71837 regulator device
 @param pdev bd71837 regulator platform device
 @return 0
*/
static int __exit bd71837_remove(struct platform_device *pdev)
{
	struct bd71837_pmic *pmic = platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &clk_attr_group);

	for (i = 0; i < BD71837_REGULATOR_CNT; i++)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return 0;
}

static struct platform_driver bd71837_driver = {
	.driver = {
		.name = "bd71837-pmic",
		.owner = THIS_MODULE,
	},
	.probe = bd71837_probe,
	.remove = bd71837_remove,
};

/**@brief module initialize function */
static int __init bd71837_init(void)
{
	return platform_driver_register(&bd71837_driver);
}
subsys_initcall(bd71837_init);

/**@brief module deinitialize function */
static void __exit bd71837_cleanup(void)
{
	platform_driver_unregister(&bd71837_driver);
}
module_exit(bd71837_cleanup);

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("BD71837 voltage regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd71837-pmic");

/*-------------------------------------------------------*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define PROCFS_NAME 		"bd71837"
#define BD71837_REV			"BD71837 Driver: Rev002\n"

#define BD71837_BUF_SIZE	1024
static char procfs_buffer[BD71837_BUF_SIZE];
/**
 * This function is called then the /proc file is read
 *
 */
static int onetime;
static ssize_t bd71837_proc_read (struct file *file, char __user *buffer, size_t count, loff_t *data)
{
	int ret = 0, error = 0;
	if (onetime == 0) {
		onetime = 1;
		memset(procfs_buffer, 0, BD71837_BUF_SIZE);
		sprintf(procfs_buffer, "%s", BD71837_REV);
		ret = strlen(procfs_buffer);
		error = copy_to_user(buffer, procfs_buffer, strlen(procfs_buffer));
	} else {
		//Clear for next time
		onetime = 0;
	}
	return (error != 0) ? 0 : ret;
}

static ssize_t bd71837_proc_write (struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	sscanf(buffer, "0x%x", &bd71837_debug_mask);
	printk("bd71837: bd71837_debug_mask=0x%08x\n", bd71837_debug_mask);
	return count;
}

static const struct file_operations bd71837_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= bd71837_proc_read,
	.write		= bd71837_proc_write,
};

/**
 *This function is called when the module is loaded
 *
 */
int bd71837_revision_init(void)
{
	struct proc_dir_entry *bd71837_proc_entry;

	/* create the /proc/bd71837 */
	bd71837_proc_entry = proc_create(PROCFS_NAME, 0644, NULL, &bd71837_proc_fops);
	if (bd71837_proc_entry == NULL) {
		printk("Error: Could not initialize /proc/%s\n", PROCFS_NAME);
		return -ENOMEM;
	}

	return 0;
}
module_init(bd71837_revision_init);
/*-------------------------------------------------------*/
