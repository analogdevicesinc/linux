// SPDX-License-Identifier: GPL-2.0
/*
 * Regulator driver for Analog Devices ADP5055
 *
 * Copyright (C) 2025 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

/*
 * ADP5055 Register Map.
 */
#define ADP5055_CTRL123         0xD1
#define ADP5055_CTRL_MODE1      0xD3
#define ADP5055_CTRL_MODE2      0xD4
#define ADP5055_DLY0            0xD5
#define ADP5055_DLY1            0xD6
#define ADP5055_DLY2            0xD7
#define ADP5055_VID0            0xD8
#define ADP5055_VID1            0xD9
#define ADP5055_VID2            0xDA
#define ADP5055_DVS_LIM0        0xDC
#define ADP5055_DVS_LIM1        0xDD
#define ADP5055_DVS_LIM2        0xDE
#define ADP5055_FT_CFG          0xDF
#define ADP5055_PG_CFG          0xE0

/*
 * ADP5055 Field Masks.
 */
#define	ADP5055_MASK_EN_MODE		BIT(0)
#define	ADP5055_MASK_OCP_BLANKING	BIT(7)
#define	ADP5055_MASK_PSM321		GENMASK(6, 4)
#define	ADP5055_MASK_DIS		GENMASK(2, 0)
#define	ADP5055_MASK_DIS_DLY		GENMASK(6, 4)
#define	ADP5055_MASK_EN_DLY		GENMASK(2, 0)
#define	ADP5055_MASK_DVS_LIM_UPPER	GENMASK(7, 4)
#define	ADP5055_MASK_DVS_LIM_LOWER	GENMASK(3, 0)
#define	ADP5055_MASK_FAST_TRANSIENT3	GENMASK(5, 4)
#define	ADP5055_MASK_FAST_TRANSIENT2	GENMASK(3, 2)
#define	ADP5055_MASK_FAST_TRANSIENT1	GENMASK(1, 0)
#define	ADP5055_MASK_DLY_PWRGD		BIT(4)
#define	ADP5055_MASK_PWRGD321		GENMASK(2, 0)

#define	ADP5055_MAX_VOUT		790500
#define	ADP5055_MIN_VOUT		408000

#define ADP5055_NUM_CH			3

struct adp5055 {
	struct regmap *regmap;
	struct gpio_desc *hw_en_gpio[ADP5055_NUM_CH];
	bool enable_mode_sw;
};

static const unsigned int adp5055_disable_delay_ch_vals[] = {
	0,
	2,
	4,
	6,
	8,
	10,
	12,
	14,
};

static const unsigned int adp5055_enable_delay_ch_vals[] = {
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
};

static const char * const adp5055_fast_transient_ch_vals[] = {
	"none",
	"3G_1.5%",
	"5G_1.5%",
	"5G_2.5%",
};

static int adp5055_get_prop_index(const u32 *table, size_t table_size,
				  u32 value)
{
	int i;

	for (i = 0; i < table_size; i++)
		if (table[i] == value)
			return i;

	return -EINVAL;
}

static const struct regmap_range adp5055_reg_ranges[] = {
	regmap_reg_range(0xD1, 0xE0),
};

static const struct regmap_access_table adp5055_write_ranges_table = {
	.yes_ranges	= adp5055_reg_ranges,
	.n_yes_ranges	= ARRAY_SIZE(adp5055_reg_ranges),
};

static const struct regmap_access_table adp5055_read_ranges_table = {
	.yes_ranges	= adp5055_reg_ranges,
	.n_yes_ranges	= ARRAY_SIZE(adp5055_reg_ranges),
};

static const struct regmap_config adp5055_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
	.wr_table = &adp5055_write_ranges_table,
	.rd_table = &adp5055_read_ranges_table,
};

static const struct linear_range adp5055_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(408000, 0, 255, 1500),
};

static int adp5055_parse_fw(struct device *dev, struct  adp5055 *adp5055)
{
	int i, ret;
	struct regmap *regmap = adp5055->regmap;
	int val;

	bool ocp_blanking;
	bool power_saving_mode_ch[ADP5055_NUM_CH];
	bool output_discharge_function_ch[ADP5055_NUM_CH];
	u32 disable_delay_ch[ADP5055_NUM_CH];
	u32 enable_delay_ch[ADP5055_NUM_CH];
	u32 dvs_limit_upper_ch[ADP5055_NUM_CH];
	u32 dvs_limit_lower_ch[ADP5055_NUM_CH];
	u32 fast_transient_ch[ADP5055_NUM_CH];
	bool delay_power_good;
	bool mask_power_good_ch[ADP5055_NUM_CH];

	for (i = 0; i < ADP5055_NUM_CH; i++)
		fast_transient_ch[i] = 3;

	adp5055->enable_mode_sw = device_property_read_bool(dev, "adi,enable-mode-sw");

	ocp_blanking = device_property_read_bool(dev, "adi,ocp-blanking");

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &i);
		if (ret || i >= ADP5055_NUM_CH)
			return dev_err_probe(dev, ret, "Failed to read reg value of child node");

		power_saving_mode_ch[i] = fwnode_property_read_bool(child,
						   "adi,power-saving-mode-ch");

		output_discharge_function_ch[i] = fwnode_property_read_bool(child,
			"adi,output-discharge-function-ch");

		ret = fwnode_property_read_u32(child, "adi,disable-delay-ch",
					   &disable_delay_ch[i]);
		if (!ret) {
			ret = adp5055_get_prop_index(adp5055_disable_delay_ch_vals,
							ARRAY_SIZE(adp5055_disable_delay_ch_vals),
							disable_delay_ch[i]);
			if (ret < 0)
				return dev_err_probe(dev, ret, "Failed to initialize disable-delay-ch. Using default.");
			disable_delay_ch[i] = ret;
		}

		ret = fwnode_property_read_u32(child, "adi,enable-delay-ch",
					   &enable_delay_ch[i]);
		if (!ret) {
			ret = adp5055_get_prop_index(adp5055_disable_delay_ch_vals,
							ARRAY_SIZE(adp5055_disable_delay_ch_vals),
							enable_delay_ch[i]);
			if (ret < 0)
				return dev_err_probe(dev, ret, "Failed to initialize enable-delay-ch. Using default.");
			enable_delay_ch[i] = ret;
		}

		ret = fwnode_property_read_u32(child, "adi,dvs-limit-upper-ch",
					&dvs_limit_upper_ch[i]);
		if (ret)
			return ret;

		ret = fwnode_property_read_u32(child, "adi,dvs-limit-lower-ch",
					&dvs_limit_lower_ch[i]);
		if (ret)
			return ret;

		ret = fwnode_property_match_string(child, "adi,fast-transient-ch",
							*adp5055_fast_transient_ch_vals);
		if (ret < 0)
			fast_transient_ch[i] = 0;
		else
			fast_transient_ch[i] = ret;

		mask_power_good_ch[i] = fwnode_property_read_bool(child,
						 "adi,mask-power-good-ch");

		val = FIELD_PREP(ADP5055_MASK_DIS_DLY, disable_delay_ch[i]);
		val |= FIELD_PREP(ADP5055_MASK_EN_DLY, enable_delay_ch[i]);
		ret = regmap_write(regmap, ADP5055_DLY0 + i, val);
		if (ret)
			return ret;

		val = FIELD_PREP(ADP5055_MASK_DVS_LIM_UPPER, dvs_limit_upper_ch[i]);
		val |= FIELD_PREP(ADP5055_MASK_DVS_LIM_LOWER, dvs_limit_lower_ch[i]);
		ret = regmap_write(regmap, ADP5055_DVS_LIM0 + i, val);
		if (ret)
			return ret;

		i++;
	};

	delay_power_good = device_property_read_bool(dev,
				    "adi,delay-power-good");

	val = FIELD_PREP(ADP5055_MASK_EN_MODE, adp5055->enable_mode_sw);
	ret = regmap_write(regmap, ADP5055_CTRL_MODE1, val);
	if (ret)
		return ret;

	val = FIELD_PREP(ADP5055_MASK_OCP_BLANKING, ocp_blanking);
	val |= FIELD_PREP(ADP5055_MASK_PSM321, power_saving_mode_ch[0]);
	val |= FIELD_PREP(ADP5055_MASK_DIS, output_discharge_function_ch[1]);
	ret = regmap_write(regmap, ADP5055_CTRL_MODE2, val);
	if (ret)
		return ret;

	val = FIELD_PREP(ADP5055_MASK_FAST_TRANSIENT1, fast_transient_ch[0]);
	val |= FIELD_PREP(ADP5055_MASK_FAST_TRANSIENT2, fast_transient_ch[1]);
	val |= FIELD_PREP(ADP5055_MASK_FAST_TRANSIENT3, fast_transient_ch[2]);
	ret = regmap_write(regmap, ADP5055_FT_CFG, val);
	if (ret)
		return ret;

	val = FIELD_PREP(ADP5055_MASK_DLY_PWRGD, delay_power_good);
	val |= FIELD_PREP(ADP5055_MASK_PWRGD321, mask_power_good_ch[0]);
	ret = regmap_write(regmap, ADP5055_PG_CFG, val);
	if (ret)
		return ret;

	// Request and configure GPIOs
	adp5055->hw_en_gpio[0] = devm_gpiod_get_optional(dev, "adi,hw_en_0",
				 GPIOD_OUT_HIGH);
	if (IS_ERR(adp5055->hw_en_gpio[0])) {
		return dev_err_probe(dev, PTR_ERR(adp5055->hw_en_gpio[0]),
				     "Failed to get hw_en_0 GPIO\n");
	}
	adp5055->hw_en_gpio[1] = devm_gpiod_get_optional(dev, "adi,hw_en_1",
				 GPIOD_OUT_HIGH);
	if (IS_ERR(adp5055->hw_en_gpio[1])) {
		return dev_err_probe(dev, PTR_ERR(adp5055->hw_en_gpio[1]),
				     "Failed to get hw_en_1 GPIO\n");
	}
	adp5055->hw_en_gpio[2] = devm_gpiod_get_optional(dev, "adi,hw_en_2",
				 GPIOD_OUT_HIGH);
	if (IS_ERR(adp5055->hw_en_gpio[2])) {
		return dev_err_probe(dev, PTR_ERR(adp5055->hw_en_gpio[2]),
				     "Failed to get hw_en_2 GPIO\n");
	}

	gpiod_set_value_cansleep(adp5055->hw_en_gpio[0], 0);
	gpiod_set_value_cansleep(adp5055->hw_en_gpio[1], 0);
	gpiod_set_value_cansleep(adp5055->hw_en_gpio[2], 0);

	return 0;
}

static int adp5055_is_enabled(struct regulator_dev *dev)
{
	struct adp5055 *adp5055 = rdev_get_drvdata(dev);
	int id, ret;
	int mask;
	int val_sw, val_hw;

	id = rdev_get_id(dev);
	mask = 1 << id;
	ret = regmap_read(adp5055->regmap, ADP5055_CTRL_MODE1, &val_sw);
	val_hw = gpiod_get_value_cansleep(adp5055->hw_en_gpio[id]);

	if (adp5055->enable_mode_sw)
		return (val_sw & mask) != 0;
	else
		return val_hw;
};

static int adp5055_en_func(struct regulator_dev *dev, int en_val)
{
	struct adp5055 *adp5055 = rdev_get_drvdata(dev);
	int id, ret;
	int mask;

	id = rdev_get_id(dev);
	mask = 1 << id;

	if (adp5055->enable_mode_sw) {
		ret = regmap_update_bits(adp5055->regmap, ADP5055_CTRL_MODE1, mask, en_val);
		if (ret)
			return ret;
	} else {
		gpiod_set_value_cansleep(adp5055->hw_en_gpio[id], en_val);
	}
	return 0;
}

static int adp5055_enable(struct regulator_dev *dev)
{
	return adp5055_en_func(dev, 1);
}

static int adp5055_disable(struct regulator_dev *dev)
{
	return adp5055_en_func(dev, 0);
}

static const struct regulator_ops adp5055_ops = {
	.list_voltage			= regulator_list_voltage_linear_range,
	.map_voltage			= regulator_map_voltage_linear_range,
	.set_voltage_sel		= regulator_set_voltage_sel_regmap,
	.get_voltage_sel		= regulator_get_voltage_sel_regmap,
	.enable				= adp5055_enable,
	.disable			= adp5055_disable,
	.is_enabled			= adp5055_is_enabled,
};

#define ADP5055_REG_(_name, _id, _ch, _ops)					\
	[_id] = {							\
		.name			= _name,			\
		.ops			= _ops,				\
		.linear_ranges		= adp5055_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(adp5055_voltage_ranges), \
		.vsel_reg		= ADP5055_VID##_ch,		\
		.vsel_mask		= GENMASK(7, 0),		\
		.enable_reg		= ADP5055_CTRL123,		\
		.enable_mask		= BIT(_ch),			\
		.owner			= THIS_MODULE,			\
	}

#define ADP5055_REG(_name, _id, _ch) \
	ADP5055_REG_(_name, _id, _ch, &adp5055_ops)

static const struct regulator_desc adp5055_regulators[] = {
	ADP5055_REG("DCDC1", 0, 0),
	ADP5055_REG("DCDC2", 1, 1),
	ADP5055_REG("DCDC3", 2, 2),
};

static const struct of_device_id adp5055_dt_ids[] = {
	{ .compatible = "adi,adp5055"},
	{ }
};
MODULE_DEVICE_TABLE(of, adp5055_dt_ids);

static int adp5055_probe(struct i2c_client *client)
{
	struct regulator_init_data *init_data;
	struct device *dev = &client->dev;
	struct adp5055 *adp5055;
	int i, ret;

	init_data = of_get_regulator_init_data(dev, client->dev.of_node,
					       &adp5055_regulators[0]);
	if (!init_data)
		return -EINVAL;

	adp5055 = devm_kzalloc(dev, sizeof(struct adp5055), GFP_KERNEL);
	if (!adp5055)
		return -ENOMEM;

	adp5055->regmap = devm_regmap_init_i2c(client, &adp5055_regmap_config);
	if (IS_ERR(adp5055->regmap)) {
		ret = PTR_ERR(adp5055->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	ret = adp5055_parse_fw(dev, adp5055);
	if (ret < 0)
		return ret;

	for (i = 0; i < ADP5055_NUM_CH; i++) {
		const struct regulator_desc *desc = &adp5055_regulators[i];
		struct regulator_config config = { };
		struct regulator_dev *rdev;

		config.dev = dev;
		config.driver_data = adp5055;
		config.regmap = adp5055->regmap;
		config.init_data = init_data;

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(dev, "Failed to register %s\n", desc->name);
			return PTR_ERR(rdev);
		}
	}

	return 0;
}

static const struct of_device_id adp5055_of_match[] = {
	{ .compatible = "adi,adp5055", },
	{ }
};
MODULE_DEVICE_TABLE(of, adp5055_of_match);

static const struct i2c_device_id adp5055_ids[] = {
	{ .name = "adp5055"},
	{ },
};
MODULE_DEVICE_TABLE(i2c, adp5055_ids);

static struct i2c_driver adp5055_driver = {
	.driver	= {
		.name	= "adp5055",
		.of_match_table = adp5055_of_match,
	},
	.probe		= adp5055_probe,
	.id_table	= adp5055_ids,
};
module_i2c_driver(adp5055_driver);

MODULE_DESCRIPTION("ADP5055 Voltage Regulator Driver");
MODULE_AUTHOR("Alexis Czezar Torreno <alexisczezar.torreno@analog.com>");
MODULE_LICENSE("GPL");
