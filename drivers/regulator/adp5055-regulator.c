// SPDX-License-Identifier: GPL-2.0
/*
 * Regulator driver for Analog Devices ADP5055
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

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
#define	ADP5055_MASK_EN0		BIT(0)
#define	ADP5055_MASK_EN1		BIT(1)
#define	ADP5055_MASK_EN2		BIT(2)
#define	ADP5055_MASK_EN_MODE		GENMASK(1, 0)
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
	u32 enable_mode_code;
	bool ocp_blanking;
	u32 power_saving_mode_ch321_code;
	u32 output_discharge_function_ch321_code;
	u32 disable_delay_code_ch123[ADP5055_NUM_CH];
	u32 enable_delay_code_ch123[ADP5055_NUM_CH];
	u32 dvs_limit_upper_code_ch123[ADP5055_NUM_CH];
	u32 dvs_limit_lower_code_ch123[ADP5055_NUM_CH];
	u32 fast_transient_code_ch123[ADP5055_NUM_CH];
	bool delay_power_good;
	u32 mask_power_good_ch321_code;
};

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
	int val1, val2, val3;

	adp5055->enable_mode_code = 0;
	adp5055->ocp_blanking = 0;
	adp5055->power_saving_mode_ch321_code = 0;
	adp5055->output_discharge_function_ch321_code = 7;
	for (i = 0; i < ADP5055_NUM_CH; i++) {
		adp5055->disable_delay_code_ch123[i] = 0;
		adp5055->enable_delay_code_ch123[i] = 0;
		adp5055->dvs_limit_upper_code_ch123[i] = 0;
		adp5055->dvs_limit_lower_code_ch123[i] = 0;
		adp5055->fast_transient_code_ch123[i] = 3;
	}
	adp5055->delay_power_good = 1;
	adp5055->mask_power_good_ch321_code = 0;

	device_property_read_u32(dev, "adi,enable-mode-code",
				 &adp5055->enable_mode_code);
	adp5055->ocp_blanking = device_property_read_bool(dev, "adi,ocp-blanking");
	device_property_read_u32(dev, "adi,power-saving-mode-ch321-code",
				 &adp5055->power_saving_mode_ch321_code);
	device_property_read_u32(dev, "adi,output-discharge-function-ch321-code",
				 &adp5055->output_discharge_function_ch321_code);
	device_property_read_u32_array(dev, "adi,disable-delay-code-ch123",
				       &adp5055->disable_delay_code_ch123[0], ADP5055_NUM_CH);
	device_property_read_u32_array(dev, "adi,enable-delay-code-ch123",
				       &adp5055->enable_delay_code_ch123[0], ADP5055_NUM_CH);
	device_property_read_u32_array(dev, "adi,dvs-limit-upper-code-ch123",
				       &adp5055->dvs_limit_upper_code_ch123[0], ADP5055_NUM_CH);
	device_property_read_u32_array(dev, "adi,dvs-limit-lower-code-ch123",
				       &adp5055->dvs_limit_lower_code_ch123[0], ADP5055_NUM_CH);
	device_property_read_u32_array(dev, "adi,fast-transient-code-ch123",
				       &adp5055->fast_transient_code_ch123[0], ADP5055_NUM_CH);
	adp5055->delay_power_good = device_property_read_bool(dev,
				    "adi,delay-power-good");
	device_property_read_u32(dev, "adi,mask-power-good-ch321-code",
				 &adp5055->mask_power_good_ch321_code);

	val1 = FIELD_PREP(ADP5055_MASK_EN_MODE, adp5055->enable_mode_code);
	ret = regmap_write(regmap, ADP5055_CTRL_MODE1, val1);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_OCP_BLANKING, adp5055->ocp_blanking);
	val2 = FIELD_PREP(ADP5055_MASK_PSM321, adp5055->power_saving_mode_ch321_code);
	val3 = FIELD_PREP(ADP5055_MASK_DIS,
			  adp5055->output_discharge_function_ch321_code);
	ret = regmap_write(regmap, ADP5055_CTRL_MODE2, val1 | val2 | val3);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_DIS_DLY, adp5055->disable_delay_code_ch123[0]);
	val2 = FIELD_PREP(ADP5055_MASK_EN_DLY, adp5055->enable_delay_code_ch123[0]);
	ret = regmap_write(regmap, ADP5055_DLY0, val1 | val2);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_DIS_DLY, adp5055->disable_delay_code_ch123[1]);
	val2 = FIELD_PREP(ADP5055_MASK_EN_DLY, adp5055->enable_delay_code_ch123[1]);
	ret = regmap_write(regmap, ADP5055_DLY1, val1 | val2);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_DIS_DLY, adp5055->disable_delay_code_ch123[2]);
	val2 = FIELD_PREP(ADP5055_MASK_EN_DLY, adp5055->enable_delay_code_ch123[2]);
	ret = regmap_write(regmap, ADP5055_DLY2, val1 | val2);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_DVS_LIM_UPPER,
			  adp5055->dvs_limit_upper_code_ch123[0]);
	val2 = FIELD_PREP(ADP5055_MASK_DVS_LIM_LOWER,
			  adp5055->dvs_limit_lower_code_ch123[0]);
	ret = regmap_write(regmap, ADP5055_DVS_LIM0, val1 | val2);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_DVS_LIM_UPPER,
			  adp5055->dvs_limit_upper_code_ch123[1]);
	val2 = FIELD_PREP(ADP5055_MASK_DVS_LIM_LOWER,
			  adp5055->dvs_limit_lower_code_ch123[1]);
	ret = regmap_write(regmap, ADP5055_DVS_LIM1, val1 | val2);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_DVS_LIM_UPPER,
			  adp5055->dvs_limit_upper_code_ch123[2]);
	val2 = FIELD_PREP(ADP5055_MASK_DVS_LIM_LOWER,
			  adp5055->dvs_limit_lower_code_ch123[2]);
	ret = regmap_write(regmap, ADP5055_DVS_LIM2, val1 | val2);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_FAST_TRANSIENT1,
			  adp5055->fast_transient_code_ch123[0]);
	val2 = FIELD_PREP(ADP5055_MASK_FAST_TRANSIENT2,
			  adp5055->fast_transient_code_ch123[1]);
	val3 = FIELD_PREP(ADP5055_MASK_FAST_TRANSIENT3,
			  adp5055->fast_transient_code_ch123[2]);
	ret = regmap_write(regmap, ADP5055_FT_CFG, val1 | val2 | val3);
	if (ret)
		return ret;

	val1 = FIELD_PREP(ADP5055_MASK_DLY_PWRGD, adp5055->delay_power_good);
	val2 = FIELD_PREP(ADP5055_MASK_PWRGD321, adp5055->mask_power_good_ch321_code);
	ret = regmap_write(regmap, ADP5055_PG_CFG, val1 | val2);
	if (ret)
		return ret;

	return 0;
}

static const struct regulator_ops adp5055_ops = {
	.list_voltage			= regulator_list_voltage_linear_range,
	.map_voltage			= regulator_map_voltage_linear_range,
	.set_voltage_sel		= regulator_set_voltage_sel_regmap,
	.get_voltage_sel		= regulator_get_voltage_sel_regmap,
	.enable				= regulator_enable_regmap,
	.disable			= regulator_disable_regmap,
	.is_enabled			= regulator_is_enabled_regmap,
};

#define ADP5055_REG_(_name, _id, _ops)					\
	[_id] = {							\
		.name			= _name,			\
		.ops			= _ops,				\
		.linear_ranges		= adp5055_voltage_ranges,	\
		.n_linear_ranges	= ARRAY_SIZE(adp5055_voltage_ranges), \
		.vsel_reg		= ADP5055_VID##_id,		\
		.vsel_mask		= GENMASK(7, 0),		\
		.enable_reg		= ADP5055_CTRL123,		\
		.enable_mask		= ADP5055_MASK_EN##_id,		\
		.owner			= THIS_MODULE,			\
	}

#define ADP5055_REG(_name, _id) \
	ADP5055_REG_(_name, _id, &adp5055_ops)

static const struct regulator_desc adp5055_regulators[] = {
	ADP5055_REG("DCDC1", 0),
	ADP5055_REG("DCDC2", 1),
	ADP5055_REG("DCDC3", 2),
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
	const struct regmap_config *regmap_config;
	int i, ret;

	regmap_config = &adp5055_regmap_config;

	init_data = of_get_regulator_init_data(dev, client->dev.of_node,
					       &adp5055_regulators[0]);
	if (!init_data)
		return -EINVAL;

	adp5055 = devm_kzalloc(dev, sizeof(struct adp5055), GFP_KERNEL);
	if (!adp5055)
		return -ENOMEM;

	adp5055->regmap = devm_regmap_init_i2c(client, regmap_config);
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

	i2c_set_clientdata(client, adp5055);

	return 0;
}

static const struct i2c_device_id adp5055_ids[] = {
	{ .name = "adp5055"},
	{ },
};
MODULE_DEVICE_TABLE(i2c, adp5055_ids);

static struct i2c_driver adp5055_driver = {
	.driver	= {
		.name	= "adp5055",
	},
	.probe		= adp5055_probe,
	.id_table	= adp5055_ids,
};

module_i2c_driver(adp5055_driver);

MODULE_DESCRIPTION("ADP5055 Voltage Regulator Driver");
MODULE_AUTHOR("Alexis Czezar Torreno <alexisczezar.torreno@analog.com>");
MODULE_LICENSE("GPL");
