// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2025 Analog Devices, Inc.
 * ADI regulator driver for MAX77533.
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

/* Register Addresses */
#define MAX77533_REG_CFG		0x00
#define MAX77533_REG_VOUT		0x01

/* CFG Register Bits */
#define MAX77533_BIT_EN			BIT(0)
#define MAX77533_BIT_EN_LOGIC		BIT(1)
#define MAX77533_BIT_MODE		BIT(2)
#define MAX77533_BIT_IPEAK		BIT(3)
#define MAX77533_BIT_ADEN		BIT(6)

#define MAX77533_BITS_SFT_STRT		GENMASK(5, 4)

/* VOUT Register Bits */
#define MAX77533_BITS_VOUT		GENMASK(6, 0)

struct max77533_regulator {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_dev *rdev;
	struct gpio_desc *pok_gpio;
	int pok_irq;
};

static const struct regmap_config max77533_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x01,
};

static const unsigned int max77533_current_limit_table[] = {
	500000, 2000000
};

static irqreturn_t max77533_pok_irq_handler(int irq, void *data)
{
	struct max77533_regulator *max77533 = data;
	int pok_state;

	pok_state = gpiod_get_value_cansleep(max77533->pok_gpio);
	if (pok_state < 0)
		return IRQ_HANDLED;

	if (pok_state)
		regulator_notifier_call_chain(max77533->rdev,
					      REGULATOR_EVENT_VOLTAGE_CHANGE,
					      NULL);
	else
		regulator_notifier_call_chain(max77533->rdev,
					      REGULATOR_EVENT_UNDER_VOLTAGE,
					      NULL);

	return IRQ_HANDLED;
}

static int max77533_get_status(struct regulator_dev *rdev)
{
	struct max77533_regulator *max77533 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(max77533->regmap, MAX77533_REG_CFG, &val);
	if (ret)
		return REGULATOR_STATUS_ERROR;

	if (!(val & MAX77533_BIT_EN))
		return REGULATOR_STATUS_OFF;

	if (max77533->pok_gpio) {
		int pok = gpiod_get_value_cansleep(max77533->pok_gpio);

		if (pok < 0)
			return REGULATOR_STATUS_ERROR;
		if (pok)
			return REGULATOR_STATUS_ON;
		return REGULATOR_STATUS_ERROR;
	}

	return REGULATOR_STATUS_ON;
}

static int max77533_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct max77533_regulator *max77533 = rdev_get_drvdata(rdev);
	unsigned int val;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		val = MAX77533_BIT_MODE;
		break;
	case REGULATOR_MODE_NORMAL:
		val = 0;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(max77533->regmap, MAX77533_REG_CFG,
				  MAX77533_BIT_MODE, val);
}

static unsigned int max77533_get_mode(struct regulator_dev *rdev)
{
	struct max77533_regulator *max77533 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(max77533->regmap, MAX77533_REG_CFG, &val);
	if (ret)
		return REGULATOR_MODE_NORMAL;

	if (val & MAX77533_BIT_MODE)
		return REGULATOR_MODE_FAST;

	return REGULATOR_MODE_NORMAL;
}

static const struct regulator_ops max77533_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.get_current_limit	= regulator_get_current_limit_regmap,
	.set_current_limit	= regulator_set_current_limit_regmap,
	.set_active_discharge	= regulator_set_active_discharge_regmap,
	.set_soft_start		= regulator_set_soft_start_regmap,
	.set_mode		= max77533_set_mode,
	.get_mode		= max77533_get_mode,
	.get_status		= max77533_get_status,
};

static const struct regulator_desc max77533_desc = {
	.name			= "max77533",
	.ops			= &max77533_ops,
	.type			= REGULATOR_VOLTAGE,
	.owner			= THIS_MODULE,
	.min_uV			= 800000,
	.uV_step		= 50000,
	.n_voltages		= 85,
	.vsel_reg		= MAX77533_REG_VOUT,
	.vsel_mask		= MAX77533_BITS_VOUT,
	.enable_reg		= MAX77533_REG_CFG,
	.enable_mask		= MAX77533_BIT_EN,
	.active_discharge_reg	= MAX77533_REG_CFG,
	.active_discharge_mask	= MAX77533_BIT_ADEN,
	.active_discharge_off	= 0,
	.active_discharge_on	= MAX77533_BIT_ADEN,
	.soft_start_reg		= MAX77533_REG_CFG,
	.soft_start_mask	= MAX77533_BITS_SFT_STRT,
	.csel_reg		= MAX77533_REG_CFG,
	.csel_mask		= MAX77533_BIT_IPEAK,
	.curr_table		= max77533_current_limit_table,
	.n_current_limits	= ARRAY_SIZE(max77533_current_limit_table),
};

/* sysfs attributes */
static ssize_t registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct max77533_regulator *max77533 = dev_get_drvdata(dev);
	unsigned int cfg, vout;
	int ret;

	ret = regmap_read(max77533->regmap, MAX77533_REG_CFG, &cfg);
	if (ret)
		return ret;

	ret = regmap_read(max77533->regmap, MAX77533_REG_VOUT, &vout);
	if (ret)
		return ret;

	return sysfs_emit(buf, "CFG(0x00)=0x%02x VOUT(0x01)=0x%02x\n", cfg, vout);
}

static ssize_t registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct max77533_regulator *max77533 = dev_get_drvdata(dev);
	unsigned int reg, val;
	int ret;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret != 2)
		return -EINVAL;

	if (reg > MAX77533_REG_VOUT || val > 0xFF)
		return -EINVAL;

	ret = regmap_write(max77533->regmap, reg, val);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(registers);

static ssize_t pok_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct max77533_regulator *max77533 = dev_get_drvdata(dev);
	int pok_state;

	if (!max77533->pok_gpio)
		return sysfs_emit(buf, "N/A\n");

	pok_state = gpiod_get_value_cansleep(max77533->pok_gpio);
	if (pok_state < 0)
		return pok_state;

	return sysfs_emit(buf, "%d\n", pok_state);
}
static DEVICE_ATTR_RO(pok);

static struct attribute *max77533_attrs[] = {
	&dev_attr_registers.attr,
	&dev_attr_pok.attr,
	NULL,
};
ATTRIBUTE_GROUPS(max77533);

static int max77533_parse_dt(struct device *dev, struct max77533_regulator *max77533)
{
	struct device_node *np = dev->of_node;
	u32 val;
	int ret;

	if (!np)
		return 0;

	ret = of_property_read_u32(np, "adi,peak-current-limit-microamp", &val);
	if (!ret) {
		unsigned int sel = (val > 1000000) ? 1 : 0;

		regmap_update_bits(max77533->regmap, MAX77533_REG_CFG,
				   MAX77533_BIT_IPEAK,
				   sel ? MAX77533_BIT_IPEAK : 0);
	}

	ret = of_property_read_u32(np, "adi,soft-start-us", &val);
	if (!ret) {
		unsigned int sel;

		if (val <= 1000)
			sel = 0;
		else if (val <= 2000)
			sel = 1;
		else if (val <= 4000)
			sel = 2;
		else
			sel = 3;

		regmap_update_bits(max77533->regmap, MAX77533_REG_CFG,
				   MAX77533_BITS_SFT_STRT,
				   FIELD_PREP(MAX77533_BITS_SFT_STRT, sel));
	}

	if (of_property_read_bool(np, "adi,fpwm-mode"))
		regmap_update_bits(max77533->regmap, MAX77533_REG_CFG,
				   MAX77533_BIT_MODE, MAX77533_BIT_MODE);

	if (of_property_read_bool(np, "adi,en-logic-and"))
		regmap_update_bits(max77533->regmap, MAX77533_REG_CFG,
				   MAX77533_BIT_EN_LOGIC, MAX77533_BIT_EN_LOGIC);

	return 0;
}

static int max77533_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max77533_regulator *max77533;
	struct regulator_config config = {};
	int ret;

	max77533 = devm_kzalloc(dev, sizeof(*max77533), GFP_KERNEL);
	if (!max77533)
		return -ENOMEM;

	max77533->dev = dev;
	i2c_set_clientdata(client, max77533);

	max77533->regmap = devm_regmap_init_i2c(client, &max77533_regmap_config);
	if (IS_ERR(max77533->regmap))
		return dev_err_probe(dev, PTR_ERR(max77533->regmap),
				     "Failed to init regmap\n");

	ret = max77533_parse_dt(dev, max77533);
	if (ret)
		return ret;

	max77533->pok_gpio = devm_gpiod_get_optional(dev, "pok", GPIOD_IN);
	if (IS_ERR(max77533->pok_gpio))
		return dev_err_probe(dev, PTR_ERR(max77533->pok_gpio),
				     "Failed to get POK GPIO\n");

	config.dev = dev;
	config.driver_data = max77533;
	config.of_node = dev->of_node;
	config.regmap = max77533->regmap;

	max77533->rdev = devm_regulator_register(dev, &max77533_desc, &config);
	if (IS_ERR(max77533->rdev))
		return dev_err_probe(dev, PTR_ERR(max77533->rdev),
				     "Failed to register regulator\n");

	if (max77533->pok_gpio) {
		max77533->pok_irq = gpiod_to_irq(max77533->pok_gpio);
		if (max77533->pok_irq > 0) {
			ret = devm_request_threaded_irq(dev, max77533->pok_irq,
							NULL,
							max77533_pok_irq_handler,
							IRQF_ONESHOT |
							IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING,
							"max77533-pok", max77533);
			if (ret)
				dev_warn(dev, "Failed to request POK IRQ: %d\n",
					 ret);
		}
	}

	return 0;
}

static const struct of_device_id max77533_of_match[] = {
	{ .compatible = "adi,max77533" },
	{ .compatible = "maxim,max77533" },
	{ .compatible = "maxim,max77533a" },
	{ .compatible = "maxim,max77533b-12" },
	{ .compatible = "maxim,max77533b-18" },
	{ .compatible = "maxim,max77533b-33" },
	{ .compatible = "maxim,max77533q" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77533_of_match);

static const struct i2c_device_id max77533_id[] = {
	{ "max77533" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77533_id);

static struct i2c_driver max77533_driver = {
	.driver = {
		.name = "max77533",
		.of_match_table = max77533_of_match,
		.dev_groups = max77533_groups,
	},
	.probe = max77533_probe,
	.id_table = max77533_id,
};
module_i2c_driver(max77533_driver);

MODULE_AUTHOR("Joan Na <joan.na@analog.com>");
MODULE_DESCRIPTION("MAX77533 regulator driver");
MODULE_LICENSE("GPL");
