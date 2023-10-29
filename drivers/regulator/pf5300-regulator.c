// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 NXP.
 * NXP PF5300 pmic driver
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/pf5300.h>

struct pf5300_dvs_config {
	unsigned int run_reg;
	unsigned int run_mask;
	unsigned int standby_reg;
	unsigned int standby_mask;
};

struct pf5300_regulator_desc {
	struct regulator_desc desc;
	const struct pf5300_dvs_config dvs;
};

struct pf5300 {
	struct device *dev;
	struct regmap *regmap;
	enum pf5300_chip_type type;
	unsigned int rcnt;
	unsigned short addr;
	bool crc_en;
};

static const struct regmap_range pf5300_range = {
	.range_min = PF5300_REG_INT_STATUS1,
	.range_max = PF5300_REG_FLT_CNT2,
};

static const struct regmap_access_table pf5300_volatile_regs = {
	.yes_ranges = &pf5300_range,
	.n_yes_ranges = 1,
};

static const struct regmap_config pf5300_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &pf5300_volatile_regs,
	.max_register = PF5300_MAX_REGISTER - 1,
	.cache_type = REGCACHE_RBTREE,
};

static uint8_t crc8_j1850(uint8_t *data, uint8_t length)
{
	uint8_t t_crc;
	uint8_t i, j;

	t_crc = 0xFF;
	for (i = 0; i < length; i++) {
		t_crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if ((t_crc & 0x80) != 0) {
				t_crc <<= 1;
				t_crc ^= 0x1D;
			} else {
				t_crc <<= 1;
			}
		}
	}
	return t_crc;
}

static int pf5300_pmic_read(struct pf5300 *pf5300, unsigned int reg,
			    unsigned int *val)
{
	u8 crcBuf[3];
	u8 data[2], crc;
	int ret;

	if (reg < PF5300_MAX_REGISTER) {
		ret = regmap_raw_read(pf5300->regmap, reg, data,
				      pf5300->crc_en ? 2U : 1U);
		if (ret)
			return ret;

		*val = data[0];
		if (pf5300->crc_en) {
			/* Get CRC */
			crcBuf[0] = pf5300->addr << 1U | 0x1U;
			crcBuf[1] = reg;
			crcBuf[2] = data[0];
			crc = crc8_j1850(crcBuf, 3U);

			if (crc != data[1])
				return -EINVAL;
		} else {
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return ret;
}

static int pf5300_pmic_write(struct pf5300 *pf5300, unsigned int reg,
			     unsigned int val, uint8_t mask)
{
	uint8_t crcBuf[3];
	uint8_t data[2];
	unsigned int rxBuf;
	int ret;

	/* If not updating entire register, perform a read-mod-write */
	data[0] = val;

	if (mask != 0xFFU) {
		/* Read data */
		ret = pf5300_pmic_read(pf5300, reg, &rxBuf);
		if (ret) {
			dev_err(pf5300->dev, "Read reg=%0x error!\n", reg);
			return ret;
		}
		data[0] = (val & mask) | (rxBuf & (~mask));
	}

	if (reg < PF5300_MAX_REGISTER) {
		if (pf5300->crc_en) {
			/* Get CRC */
			crcBuf[0] = pf5300->addr << 1U;
			crcBuf[1] = reg;
			crcBuf[2] = data[0];
			data[1] = crc8_j1850(crcBuf, 3U);
		}
		/* Write data */
		ret = regmap_raw_write(pf5300->regmap, reg, data,
				       pf5300->crc_en ? 2U : 1U);
		if (ret) {
			dev_err(pf5300->dev, "Write reg=%0x error!\n", reg);
			return ret;
		}
	}

	return ret;
}

/**
 * pf5300_regulator_enable_regmap for regmap users
 *
 * @rdev: regulator to operate on
 *
 * Regulators that use regmap for their register I/O can set the
 * enable_reg and enable_mask fields in their descriptor and then use
 * this as their enable() operation, saving some code.
 */
static int pf5300_regulator_enable_regmap(struct regulator_dev *rdev)
{
	unsigned int val;
	struct pf5300 *pf5300 = dev_get_drvdata(rdev->dev.parent);

	if (rdev->desc->enable_is_inverted) {
		val = rdev->desc->disable_val;
	} else {
		val = rdev->desc->enable_val;
		if (!val)
			val = rdev->desc->enable_mask;
	}

	return pf5300_pmic_write(pf5300, rdev->desc->enable_reg, val,
				 rdev->desc->enable_mask);
}

/**
 * pf5300_regulator_disable_regmap for regmap users
 *
 * @rdev: regulator to operate on
 *
 * Regulators that use regmap for their register I/O can set the
 * enable_reg and enable_mask fields in their descriptor and then use
 * this as their disable() operation, saving some code.
 */
static int pf5300_regulator_disable_regmap(struct regulator_dev *rdev)
{
	unsigned int val;

	struct pf5300 *pf5300 = dev_get_drvdata(rdev->dev.parent);

	if (rdev->desc->enable_is_inverted) {
		val = rdev->desc->enable_val;
		if (!val)
			val = rdev->desc->enable_mask;
	} else {
		val = rdev->desc->disable_val;
	}

	return pf5300_pmic_write(pf5300, rdev->desc->enable_reg, val,
				 rdev->desc->enable_mask);
}

/**
 * pf5300_regulator_is_enabled_regmap for regmap users
 *
 * @rdev: regulator to operate on
 *
 * Regulators that use regmap for their register I/O can set the
 * enable_reg and enable_mask fields in their descriptor and then use
 * this as their is_enabled operation, saving some code.
 */
int pf5300_regulator_is_enabled_regmap(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;
	struct pf5300 *pf5300 = dev_get_drvdata(rdev->dev.parent);

	ret = pf5300_pmic_read(pf5300, rdev->desc->enable_reg, &val);
	if (ret != 0)
		return ret;

	val &= rdev->desc->enable_mask;

	if (rdev->desc->enable_is_inverted) {
		if (rdev->desc->enable_val)
			return val != rdev->desc->enable_val;
		return val == 0;
	} else {
		if (rdev->desc->enable_val)
			return val == rdev->desc->enable_val;
		return val != 0;
	}
}

/**
 * pf5300_regulator_set_voltage_sel_regmap for regmap users
 *
 * @rdev: regulator to operate on
 * @sel: Selector to set
 *
 * Regulators that use regmap for their register I/O can set the
 * vsel_reg and vsel_mask fields in their descriptor and then use this
 * as their set_voltage_vsel operation, saving some code.
 */
static int pf5300_regulator_set_voltage_sel_regmap(struct regulator_dev *rdev,
					    unsigned int sel)
{
	int ret;
	struct pf5300 *pf5300 = dev_get_drvdata(rdev->dev.parent);

	sel <<= ffs(rdev->desc->vsel_mask) - 1;
	ret = pf5300_pmic_write(pf5300, rdev->desc->vsel_reg, sel,
				rdev->desc->vsel_mask);
	if (ret)
		return ret;

	if (rdev->desc->apply_bit)
		ret = pf5300_pmic_write(pf5300, rdev->desc->apply_reg,
					rdev->desc->apply_bit,
					rdev->desc->apply_bit);
	return ret;
}

static int find_closest_bigger(unsigned int target, const unsigned int *table,
			       unsigned int num_sel, unsigned int *sel)
{
	unsigned int s, tmp, max, maxsel = 0;
	bool found = false;

	max = table[0];

	for (s = 0; s < num_sel; s++) {
		if (table[s] > max) {
			max = table[s];
			maxsel = s;
		}
		if (table[s] >= target) {
			if (!found || table[s] - target < tmp - target) {
				tmp = table[s];
				*sel = s;
				found = true;
				if (tmp == target)
					break;
			}
		}
	}

	if (!found) {
		*sel = maxsel;
		return -EINVAL;
	}

	return 0;
}

/**
 * pf5300_regulator_set_ramp_delay_regmap
 *
 * @rdev: regulator to operate on
 *
 * Regulators that use regmap for their register I/O can set the ramp_reg
 * and ramp_mask fields in their descriptor and then use this as their
 * set_ramp_delay operation, saving some code.
 */
int pf5300_regulator_set_ramp_delay_regmap(struct regulator_dev *rdev,
					   int ramp_delay)
{
	int ret;
	unsigned int sel;
	struct pf5300 *pf5300 = dev_get_drvdata(rdev->dev.parent);

	if (WARN_ON(!rdev->desc->n_ramp_values || !rdev->desc->ramp_delay_table))
		return -EINVAL;

	ret = find_closest_bigger(ramp_delay, rdev->desc->ramp_delay_table,
				  rdev->desc->n_ramp_values, &sel);

	if (ret) {
		dev_warn(rdev_get_dev(rdev),
			 "Can't set ramp-delay %u, setting %u\n", ramp_delay,
			 rdev->desc->ramp_delay_table[sel]);
	}

	sel <<= ffs(rdev->desc->ramp_mask) - 1;

	return pf5300_pmic_write(pf5300, rdev->desc->ramp_reg, sel,
				 rdev->desc->ramp_mask);
}

/**
 * pf5300_regulator_get_voltage_sel_regmap for regmap users
 *
 * @rdev: regulator to operate on
 *
 * Regulators that use regmap for their register I/O can set the
 * vsel_reg and vsel_mask fields in their descriptor and then use this
 * as their get_voltage_vsel operation, saving some code.
 */
int pf5300_regulator_get_voltage_sel_regmap(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;
	struct pf5300 *pf5300 = dev_get_drvdata(rdev->dev.parent);

	ret = pf5300_pmic_read(pf5300, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	val &= rdev->desc->vsel_mask;
	val >>= ffs(rdev->desc->vsel_mask) - 1;

	return val;
}

static const struct regulator_ops pf5300_dvs_sw_regulator_ops = {
	.enable = pf5300_regulator_enable_regmap,
	.disable = pf5300_regulator_disable_regmap,
	.is_enabled = pf5300_regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = pf5300_regulator_set_voltage_sel_regmap,
	.get_voltage_sel = pf5300_regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay	= pf5300_regulator_set_ramp_delay_regmap,
};

/*
 * SW1 0.5V to 1.2V
 * 0.5V to 1.2V (5mV step)
 */
static const struct linear_range pf5300_dvs_sw1_volts[] = {
	REGULATOR_LINEAR_RANGE(500000,   0x00, 0x8C, 5000),
	REGULATOR_LINEAR_RANGE(0,        0x8D, 0xFF, 0),
};

/*
 * SW1
 * SW1_DVS[1:0] SW1 slew ramp rate setting
 * 00: 8mV/8usec
 * 01: 8mV/4usec
 * 10: 8mV/2usec
 * 11: 8mV/1usec
 */
static const unsigned int pf5300_dvs_sw_ramp_table[] = {
	1000, 2000, 4000, 8000
};

/* SW1 dvs 0.5v to 1.35v
 * SW2-5 dvs 0.3v to 1.35v
 */
static int sw_set_dvs(const struct regulator_desc *desc,
		      struct device_node *np, struct regmap *regmap,
		      char *prop, unsigned int reg, unsigned int mask)
{
	int ret, i;
	uint32_t uv;

	ret = of_property_read_u32(np, prop, &uv);
	if (ret == -EINVAL)
		return 0;
	else if (ret)
		return ret;

	for (i = 0; i < desc->n_voltages; i++) {
		ret = regulator_desc_list_voltage_linear_range(desc, i);
		if (ret < 0)
			continue;
		if (ret == uv) {
			i <<= ffs(desc->vsel_mask) - 1;
			ret = regmap_update_bits(regmap, reg, mask, i);
			break;
		}
	}

	return ret;
}

static int pf5300_set_dvs_levels(struct device_node *np,
				 const struct regulator_desc *desc,
				 struct regulator_config *cfg)
{
	struct pf5300_regulator_desc *data = container_of(desc,
					struct pf5300_regulator_desc, desc);
	const struct pf5300_dvs_config *dvs = &data->dvs;
	unsigned int reg, mask;
	char *prop;
	int i, ret = 0;

	for (i = 0; i < PF5300_DVS_LEVEL_MAX; i++) {
		switch (i) {
		case PF5300_DVS_LEVEL_RUN:
			prop = "nxp,dvs-run-voltage";
			reg = dvs->run_reg;
			mask = dvs->run_mask;
			break;
		case PF5300_DVS_LEVEL_STANDBY:
			prop = "nxp,dvs-standby-voltage";
			reg = dvs->standby_reg;
			mask = dvs->standby_mask;
			break;
		default:
			return -EINVAL;
		}

		ret = sw_set_dvs(desc, np, cfg->regmap, prop, reg, mask);
		if (ret)
			break;
	}

	return ret;
}

static const struct pf5300_regulator_desc pf5300_regulators[] = {
	{
		.desc = {
			.name = "sw1",
			.of_match = of_match_ptr("SW1"),
			.regulators_node = of_match_ptr("regulators"),
			.id = PF5300_SW1,
			.ops = &pf5300_dvs_sw_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = PF5300_SW1_VOLTAGE_NUM,
			.linear_ranges = pf5300_dvs_sw1_volts,
			.n_linear_ranges = ARRAY_SIZE(pf5300_dvs_sw1_volts),
			.vsel_reg = PF5300_REG_SW1_VOLT,
			.vsel_mask = SW1_VOLT_MASK,
			.enable_reg = PF5300_REG_SW1_CTRL1,
			.enable_mask = SW1_MODE_MASK,
			.enable_val = SW_MODE_PWM,
			.ramp_reg = PF5300_REG_SW1_CTRL1,
			.ramp_mask = SW1_RAMP_MASK,
			.ramp_delay_table = pf5300_dvs_sw_ramp_table,
			.n_ramp_values = ARRAY_SIZE(pf5300_dvs_sw_ramp_table),
			.owner = THIS_MODULE,
			.of_parse_cb = pf5300_set_dvs_levels,
		},
		.dvs = {
			.run_reg = PF5300_REG_SW1_VOLT,
			.run_mask = SW1_VOLT_MASK,
			.standby_reg = PF5300_REG_SW1_STBY_VOLT,
			.standby_mask = SW1_STBY_VOLT_MASK,
		},
	},
};

static int pf5300_i2c_probe(struct i2c_client *i2c)
{
	enum pf5300_chip_type type = (unsigned int)(uintptr_t)
				      of_device_get_match_data(&i2c->dev);
	const struct pf5300_regulator_desc	*regulator_desc;
	struct regulator_config config = { };
	struct pf5300 *pf5300;
	unsigned int val, device_id, device_fam, i;
	int ret;
	struct device_node *np = i2c->dev.of_node;

	pf5300 = devm_kzalloc(&i2c->dev, sizeof(struct pf5300), GFP_KERNEL);
	if (!pf5300)
		return -ENOMEM;

	switch (type) {
	case PF5300_TYPE_PF5300:
	case PF5300_TYPE_PF5301:
	case PF5300_TYPE_PF5302:
		regulator_desc = pf5300_regulators;
		pf5300->rcnt = ARRAY_SIZE(pf5300_regulators);
		break;
	default:
		dev_err(&i2c->dev, "Unknown device type");
		return -EINVAL;
	}

	if (of_property_read_bool(np, "i2c-crc-enable"))
		pf5300->crc_en = true;
	pf5300->type = type;
	pf5300->dev = &i2c->dev;

	dev_set_drvdata(&i2c->dev, pf5300);

	pf5300->regmap = devm_regmap_init_i2c(i2c, &pf5300_regmap_config);
	if (IS_ERR(pf5300->regmap)) {
		dev_err(&i2c->dev, "regmap initialization failed\n");
		return PTR_ERR(pf5300->regmap);
	}

	ret = pf5300_pmic_read(pf5300, PF5300_REG_DEV_ID, &val);
	if (ret) {
		dev_err(&i2c->dev, "Read device id error\n");
		return ret;
	}

	device_id = val & 0x0F;
	device_fam = (val & 0xF0) >> 4;

	/* Check your board and dts for match the right pmic */
	if (device_fam == 0x05 &&
	    ((device_id != 0x03 && type == PF5300_TYPE_PF5300) ||
	     (device_id != 0x04 && type == PF5300_TYPE_PF5301) ||
	     (device_id != 0x05 && type == PF5300_TYPE_PF5302))) {
		dev_err(&i2c->dev, "Device id(%x) mismatched\n",
			device_id);
		return -EINVAL;
	}

	for (i = 0; i < pf5300->rcnt; i++) {
		const struct regulator_desc *desc;
		struct regulator_dev *rdev;
		const struct pf5300_regulator_desc *r;

		r = &regulator_desc[i];
		desc = &r->desc;

		config.regmap = pf5300->regmap;
		config.dev = pf5300->dev;

		rdev = devm_regulator_register(pf5300->dev, desc, &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(pf5300->dev,
				"Failed to register regulator(%s): %d\n",
				desc->name, ret);
			return ret;
		}
	}

	dev_err(&i2c->dev, "%s probed.\n",
		type == PF5300_TYPE_PF5300 ? "pf5300" :
		(type == PF5300_TYPE_PF5301 ? "pf5301" : "pf5302"));

	return 0;
}

static const struct of_device_id pf5300_of_match[] = {
	{
		.compatible = "nxp,pf5300",
		.data = (void *)PF5300_TYPE_PF5300,
	},
	{
		.compatible = "nxp,pf5301",
		.data = (void *)PF5300_TYPE_PF5301,
	},
	{
		.compatible = "nxp,pf5302",
		.data = (void *)PF5300_TYPE_PF5302,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pf5300_of_match);

static struct i2c_driver pf5300_i2c_driver = {
	.driver = {
		.name = "nxp-pf5300",
		.of_match_table = pf5300_of_match,
	},
	.probe = pf5300_i2c_probe,
};

module_i2c_driver(pf5300_i2c_driver);

MODULE_AUTHOR("Joy Zou <joy.zou@nxp.com>");
MODULE_DESCRIPTION("NXP PF5300 Power Management IC driver");
MODULE_LICENSE("GPL");
