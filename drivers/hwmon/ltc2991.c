// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Analog Devices, Inc.
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
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
#include <linux/regulator/consumer.h>

#define LTC2991_STATUS_LOW		0x00
#define LTC2991_CH_EN_TRIGGER		0x01
#define LTC2991_V1_V4_CTRL		0x06
#define LTC2991_V5_V8_CTRL		0x07
#define LTC2991_PWM_TH_LSB_T_INT	0x08
#define LTC2991_PWM_TH_MSB		0x09
#define LTC2991_CHANNEL_V_MSB(x)	(0x0A + ((x) * 2))
#define LTC2991_CHANNEL_T_MSB(x)	(0x0A + ((x) * 4))
#define LTC2991_CHANNEL_C_MSB(x)	(0x0C + ((x) * 4))
#define LTC2991_T_INT_MSB		0x1A
#define LTC2991_VCC_MSB			0x1C

#define LTC2991_V7_V8_EN		BIT(7)
#define LTC2991_V5_V6_EN		BIT(6)
#define LTC2991_V3_V4_EN		BIT(5)
#define LTC2991_V1_V2_EN		BIT(4)
#define LTC2991_T_INT_VCC_EN		BIT(3)

#define LTC2991_V3_V4_FILT_EN		BIT(7)
#define LTC2991_V3_V4_TEMP_EN		BIT(5)
#define LTC2991_V3_V4_DIFF_EN		BIT(4)
#define LTC2991_V1_V2_FILT_EN		BIT(3)
#define LTC2991_V1_V2_TEMP_EN		BIT(1)
#define LTC2991_V1_V2_DIFF_EN		BIT(0)

#define LTC2991_V7_V8_FILT_EN		BIT(7)
#define LTC2991_V7_V8_TEMP_EN		BIT(5)
#define LTC2991_V7_V8_DIFF_EN		BIT(4)
#define LTC2991_V5_V6_FILT_EN		BIT(7)
#define LTC2991_V5_V6_TEMP_EN		BIT(5)
#define LTC2991_V5_V6_DIFF_EN		BIT(4)

#define LTC2991_REPEAT_ACQ_EN		BIT(4)
#define LTC2991_T_INT_FILT_EN		BIT(3)

#define LTC2991_MAX_CHANNEL		4
#define LTC2991_T_INT_CH_NR		4
#define LTC2991_VCC_CH_NR		0

static const char *const label_voltages[] = {
	"vcc",
	"voltage1",
	"voltage2",
	"voltage3",
	"voltage4",
	"voltage5",
	"voltage6",
	"voltage7",
	"voltage8"
};

static const char *const label_temp[] = {
	"t1",
	"t2",
	"t3",
	"t4",
	"t_int"
};

static const char *const label_curr[] = {
	"v1-v2",
	"v3-v4",
	"v5-v6",
	"v7-v8"
};

struct ltc2991_state {
	struct i2c_client	*client;
	struct regmap		*regmap;
	u32			r_sense_mohm[LTC2991_MAX_CHANNEL];
	bool			temp_en[LTC2991_MAX_CHANNEL];
};

static int ltc2991_read_reg(struct ltc2991_state *st, u8 addr, u8 reg_len,
			    int *val)
{
	u8 regvals[2];
	int ret;
	int i;

	ret = regmap_bulk_read(st->regmap, addr, regvals, reg_len);
	if (ret)
		return ret;

	*val = 0;
	for (i = 0; i < reg_len; i++)
		*val |= regvals[reg_len - i - 1] << (i * 8);

	return 0;
}

static int ltc2991_get_voltage(struct ltc2991_state *st, u32 reg, long *val)
{
	int reg_val, ret, offset = 0;

	ret = ltc2991_read_reg(st, reg, 2, &reg_val);
	if (ret)
		return ret;

	if (reg == LTC2991_VCC_MSB)
		/* Vcc 2.5V offset */
		offset = 2500;

	/* Vx, 305.18uV/LSB */
	*val = DIV_ROUND_CLOSEST(sign_extend32(reg_val, 14) * 30518,
				 1000 * 100) + offset;

	return 0;
}

static int ltc2991_read_in(struct device *dev, u32 attr, int channel, long *val)
{
	struct ltc2991_state *st = dev_get_drvdata(dev);
	u32 reg;

	switch (attr) {
	case hwmon_in_input:
		if (channel == LTC2991_VCC_CH_NR)
			reg = LTC2991_VCC_MSB;
		else
			reg = LTC2991_CHANNEL_V_MSB(channel - 1);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ltc2991_get_voltage(st, reg, val);
}

static int ltc2991_get_curr(struct ltc2991_state *st, u32 reg, int channel,
			    long *val)
{
	int reg_val, ret;

	ret = ltc2991_read_reg(st, reg, 2, &reg_val);
	if (ret)
		return ret;

	/* Vx-Vy, 19.075uV/LSB */
	*val = DIV_ROUND_CLOSEST(sign_extend32(reg_val, 14) * 19075,
				 1000) / st->r_sense_mohm[channel];

	return 0;
}

static int ltc2991_read_curr(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct ltc2991_state *st = dev_get_drvdata(dev);
	u32 reg;

	switch (attr) {
	case hwmon_curr_input:
		reg = LTC2991_CHANNEL_C_MSB(channel);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ltc2991_get_curr(st, reg, channel, val);
}

static int ltc2991_get_temp(struct ltc2991_state *st, u32 reg, int channel,
			    long *val)
{
	int reg_val, ret;

	ret = ltc2991_read_reg(st, reg, 2, &reg_val);
	if (ret)
		return ret;

	/* Temp LSB = 0.0625 Degrees */
	*val = DIV_ROUND_CLOSEST(sign_extend32(reg_val, 12) * 1000, 16);

	return 0;
}

static int ltc2991_read_temp(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct ltc2991_state *st = dev_get_drvdata(dev);
	u32 reg;

	switch (attr) {
	case hwmon_temp_input:
		if (channel == LTC2991_T_INT_CH_NR)
			reg = LTC2991_T_INT_MSB;
		else
			reg = LTC2991_CHANNEL_T_MSB(channel);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ltc2991_get_temp(st, reg, channel, val);
}

static int ltc2991_read(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel,
			long *val)
{
	switch (type) {
	case hwmon_in:
		return ltc2991_read_in(dev, attr, channel, val);
	case hwmon_curr:
		return ltc2991_read_curr(dev, attr, channel, val);
	case hwmon_temp:
		return ltc2991_read_temp(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t ltc2991_is_visible(const void *data, enum hwmon_sensor_types type, u32 attr,
				  int channel)
{
	const struct ltc2991_state *st = data;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_label:
			return 0444;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
		case hwmon_curr_label:
			if (st->r_sense_mohm[channel])
				return 0444;
			break;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_label:
			if (st->temp_en[channel] || channel == LTC2991_T_INT_CH_NR)
				return 0444;
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int ltc2991_read_string(struct device *dev, enum hwmon_sensor_types type, u32 attr,
			       int channel, const char **str)
{
	switch (type) {
	case hwmon_temp:
		*str = label_temp[channel];
		break;
	case hwmon_curr:
		*str = label_curr[channel];
		break;
	case hwmon_in:
		*str = label_voltages[channel];
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct hwmon_ops ltc2991_hwmon_ops = {
	.is_visible = ltc2991_is_visible,
	.read = ltc2991_read,
	.read_string = ltc2991_read_string,
};

static const struct hwmon_channel_info *ltc2991_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL
			   ),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL
			   ),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL
			   ),
	NULL
};

static const struct hwmon_chip_info ltc2991_chip_info = {
	.ops = &ltc2991_hwmon_ops,
	.info = ltc2991_info,
};

static const struct regmap_config ltc2991_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x1D,
};

static void ltc2991_regulator_disable(void *regulator)
{
	regulator_disable(regulator);
}

static int ltc2991_init(struct ltc2991_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	struct regulator *regulator;
	int ret;
	u32 val, addr;
	u8 v5_v8_reg_data = 0, v1_v4_reg_data = 0;

	regulator = devm_regulator_get_optional(&st->client->dev, "vcc");
	if (IS_ERR(regulator)) {
		if (PTR_ERR(regulator) != -ENODEV)
			return dev_err_probe(&st->client->dev, PTR_ERR(regulator),
					     "Failed to get specified VCC supply\n");
		regulator = NULL;
	} else {
		ret = regulator_enable(regulator);
		if (ret)
			return dev_err_probe(&st->client->dev, ret,
					     "Failed to enable VCC regulator\n");

		ret = devm_add_action_or_reset(&st->client->dev,
					       ltc2991_regulator_disable, regulator);
		if (ret)
			return dev_err_probe(&st->client->dev, ret,
					     "Failed to add regulator disable\n");
	}

	fwnode = dev_fwnode(&st->client->dev);

	fwnode_for_each_available_child_node(fwnode, child) {
		ret = fwnode_property_read_u32(child, "reg", &addr);
		if (ret < 0) {
			fwnode_handle_put(child);
			return ret;
		}

		if (addr > 3) {
			fwnode_handle_put(child);
			return -EINVAL;
		}

		ret = fwnode_property_read_u32(child, "shunt-resistor-mili-ohms", &val);
		if (!ret) {
			st->r_sense_mohm[addr] = val;
			switch (addr) {
			case 0:
				v1_v4_reg_data |= LTC2991_V1_V2_DIFF_EN;
				break;
			case 1:
				v1_v4_reg_data |= LTC2991_V3_V4_DIFF_EN;
				break;
			case 2:
				v5_v8_reg_data |= LTC2991_V5_V6_DIFF_EN;
				break;
			case 3:
				v5_v8_reg_data |= LTC2991_V7_V8_DIFF_EN;
				break;
			default:
				break;
			}
		}

		ret = fwnode_property_read_bool(child, "temperature-enable");
		if (ret) {
			st->temp_en[addr] = ret;
			switch (addr) {
			case 0:
				v1_v4_reg_data |= LTC2991_V1_V2_TEMP_EN;
				break;
			case 1:
				v1_v4_reg_data |= LTC2991_V3_V4_TEMP_EN;
				break;
			case 2:
				v5_v8_reg_data |= LTC2991_V5_V6_TEMP_EN;
				break;
			case 3:
				v5_v8_reg_data |= LTC2991_V7_V8_TEMP_EN;
				break;
			default:
				break;
			}
		}
	}

	/* Setup V5-V8 Control register */
	ret = regmap_write(st->regmap, LTC2991_V5_V8_CTRL, v5_v8_reg_data);
	if (ret)
		return dev_err_probe(&st->client->dev, ret,
				     "Error: Failed to set V5-V8 CTRL reg.\n");

	/* Setup V1-V4 Control register */
	ret = regmap_write(st->regmap, LTC2991_V1_V4_CTRL, v1_v4_reg_data);
	if (ret)
		return dev_err_probe(&st->client->dev, ret,
				     "Error: Failed to set V1-V4 CTRL reg.\n");

	/* Setup continuous mode */
	ret = regmap_write(st->regmap, LTC2991_PWM_TH_LSB_T_INT,
			   LTC2991_REPEAT_ACQ_EN);
	if (ret)
		return dev_err_probe(&st->client->dev, ret,
				     "Error: Failed to set contiuous mode.\n");

	/* Enable all channels and trigger conversions */
	ret = regmap_write(st->regmap, LTC2991_CH_EN_TRIGGER,
			   LTC2991_V7_V8_EN | LTC2991_V5_V6_EN |
			   LTC2991_V3_V4_EN | LTC2991_V1_V2_EN |
			   LTC2991_T_INT_VCC_EN);
	if (ret)
		return dev_err_probe(&st->client->dev, ret,
				     "Error: Failed to enable conversions.\n");

	return 0;
}

static int ltc2991_i2c_probe(struct i2c_client *client)
{
	int ret;
	struct device *hwmon_dev;
	struct ltc2991_state *st;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;
	st->regmap = devm_regmap_init_i2c(client, &ltc2991_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = ltc2991_init(st);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(&client->dev,
							 client->name, st,
							 &ltc2991_chip_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct of_device_id ltc2991_of_match[] = {
	{ .compatible = "adi,ltc2991" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2991_of_match);

static const struct i2c_device_id ltc2991_i2c_id[] = {
	{ "ltc2991", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ltc2991_i2c_id);

static struct i2c_driver ltc2991_i2c_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "ltc2991",
		.of_match_table = ltc2991_of_match,
	},
	.probe_new = ltc2991_i2c_probe,
	.id_table = ltc2991_i2c_id,
};

module_i2c_driver(ltc2991_i2c_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC2991 HWMON Driver");
MODULE_LICENSE("GPL");
