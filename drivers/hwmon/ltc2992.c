// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 * LTC2992 - Dual Wide Range Power Monitor
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define LTC2992_DSENSE1		0x14
#define LTC2992_SENSE1		0x1E
#define LTC2992_G1		0x28
#define LTC2992_G2		0x5A
#define LTC2992_G3		0x64
#define LTC2992_G4		0x6E

#define LTC2992_DSENSE(x)	(LTC2992_DSENSE1 + ((x) * 0x32))
#define LTC2992_SENSE(x)	(LTC2992_SENSE1 + ((x) * 0x32))

#define LTC2992_IADC_MAX_UV		51200
#define LTC2992_VADC_MAX_MILLIV		102400
#define LTC2992_VADC_GPIO_MAX_MILLIV	2048
#define LTC2992_ADCS_SCALE		65536

struct ltc2992_supply {
	u32				r_sense_uohm;
};

struct ltc2992_state {
	struct i2c_client		*client;
	struct regmap			*regmap;
	struct ltc2992_supply		supplies[2];
};

static const u8 ltc2992_gpio_addr_map[] = {
	LTC2992_G1, LTC2992_G2, LTC2992_G3, LTC2992_G4
};

static int ltc2992_read_reg(struct ltc2992_state *st, u8 addr, const u8 reg_len, u32 *val)
{
	u8 regvals[4];
	int ret;
	int i;

	ret = regmap_bulk_read(st->regmap, addr, regvals, reg_len);
	if (ret < 0)
		return ret;

	*val = 0;
	for (i = 0; i < reg_len; i++)
		*val |= regvals[reg_len - i - 1] << (i * 8);

	return 0;
}

static umode_t ltc2992_is_visible(const void *data, enum hwmon_sensor_types type, u32 attr,
				  int channel)
{
	const struct ltc2992_state *st = data;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			if (st->supplies[channel].r_sense_uohm)
				return 0444;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int ltc2992_read(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel,
			long *val)
{
	struct ltc2992_state *st = dev_get_drvdata(dev);
	int ret = -EOPNOTSUPP;
	u64 value = 0;
	u32 reg = 0;
	u8 nr_gpio;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			/* get voltage of GPIOs */
			if (channel > 1) {
				nr_gpio = channel - 2;
				ret = ltc2992_read_reg(st, ltc2992_gpio_addr_map[nr_gpio], 2, &reg);
				if (ret < 0)
					return ret;

				value = mul_u64_u32_div(reg, LTC2992_VADC_GPIO_MAX_MILLIV,
							LTC2992_ADCS_SCALE);
				break;
			}

			/* get voltage of SENSE+ */
			ret = ltc2992_read_reg(st, LTC2992_SENSE(channel), 2, &reg);
			if (ret < 0)
				return ret;

			value = mul_u64_u32_div(reg, LTC2992_VADC_MAX_MILLIV, LTC2992_ADCS_SCALE);
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			ret = ltc2992_read_reg(st, LTC2992_DSENSE(channel), 2, &reg);
			if (ret < 0)
				return ret;

			value = reg * LTC2992_IADC_MAX_UV * 1000;
			value = div_u64(value,
					LTC2992_ADCS_SCALE * st->supplies[channel].r_sense_uohm);

			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	*val = value;

	return ret;
}

static int ltc2992_write(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel,
			 long val)
{
	return 0;
}

static const struct hwmon_ops ltc2992_hwmon_ops = {
	.is_visible = ltc2992_is_visible,
	.read = ltc2992_read,
	.write = ltc2992_write,
};

static const struct hwmon_channel_info *ltc2992_info[] = {
	HWMON_CHANNEL_INFO(in, HWMON_I_INPUT, HWMON_I_INPUT,
			   HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT),
	HWMON_CHANNEL_INFO(curr, HWMON_C_INPUT, HWMON_C_INPUT),
	NULL
};

static const struct hwmon_chip_info ltc2992_chip_info = {
	.ops = &ltc2992_hwmon_ops,
	.info = ltc2992_info,
};

static const struct regmap_config ltc2992_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xE8,
};

static int ltc2992_parse_dt(struct ltc2992_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	u32 addr;
	u32 val;
	int ret;

	fwnode = dev_fwnode(&st->client->dev);

	fwnode_for_each_available_child_node(fwnode, child) {
		ret = fwnode_property_read_u32(child, "reg", &addr);
		if (ret < 0)
			return ret;

		if (addr > 1)
			return -EINVAL;

		ret = fwnode_property_read_u32(child, "shunt-resistor-micro-ohms", &val);
		if (!ret)
			st->supplies[addr].r_sense_uohm = val;
	}

	return 0;
}

static int ltc2992_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *hwmon_dev;
	struct ltc2992_state *st;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;
	st->regmap = devm_regmap_init_i2c(client, &ltc2992_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = ltc2992_parse_dt(st);
	if (ret < 0)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(&client->dev, client->name, st,
							 &ltc2992_chip_info, NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct of_device_id ltc2992_of_match[] = {
	{ .compatible = "adi,ltc2992" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2992_of_match);

static const struct i2c_device_id ltc2992_i2c_id[] = {
	{"ltc2992", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ltc2992_i2c_id);

static struct i2c_driver ltc2992_i2c_driver = {
	.driver = {
		.name = "ltc2992",
		.of_match_table = ltc2992_of_match,
	},
	.probe    = ltc2992_i2c_probe,
	.id_table = ltc2992_i2c_id,
};

module_i2c_driver(ltc2992_i2c_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Hwmon driver for Linear Technology 2992");
MODULE_LICENSE("Dual BSD/GPL");
