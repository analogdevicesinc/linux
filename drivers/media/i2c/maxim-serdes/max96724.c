// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX96724 Quad GMSL2 Deserializer Driver
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/regmap.h>

#include "max_des.h"

#define MAX96724_NAME "max96724"
#define MAX96724_GPIO_NUM 9
#define MAX96724_DPLL_FREQ 2500
#define MAX96724_PHYS_NUM 4

struct max96724_priv {
	struct max_des_priv des_priv;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct pinctrl_dev *pctldev;

	struct pinctrl_desc pctldesc;
	struct gpio_chip gc;
};

#define des_to_priv(des) container_of(des, struct max96724_priv, des_priv)

static int max96724_read(struct max96724_priv *priv, int reg)
{
	int ret, val;

	ret = regmap_read(priv->regmap, reg, &val);
	dev_dbg(priv->dev, "read %d 0x%x = 0x%02x\n", ret, reg, val);
	if (ret) {
		dev_err(priv->dev, "read 0x%04x failed\n", reg);
		return ret;
	}

	return val;
}

static int max96724_write(struct max96724_priv *priv, unsigned int reg, u8 val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	dev_dbg(priv->dev, "write %d 0x%x = 0x%02x\n", ret, reg, val);
	if (ret)
		dev_err(priv->dev, "write 0x%04x failed\n", reg);

	return ret;
}

static int max96724_update_bits(struct max96724_priv *priv, unsigned int reg,
				u8 mask, u8 val)
{
	int ret;

	ret = regmap_update_bits(priv->regmap, reg, mask, val);
	dev_dbg(priv->dev, "update %d 0x%x 0x%02x = 0x%02x\n", ret, reg, mask,
		val);
	if (ret)
		dev_err(priv->dev, "update 0x%04x failed\n", reg);

	return ret;
}

static unsigned int max96724_field_get(unsigned int val, unsigned int mask)
{
	return (val & mask) >> __ffs(mask);
}

static unsigned int max96724_field_prep(unsigned int val, unsigned int mask)
{
	return (val << __ffs(mask)) & mask;
}

static int max96724_wait_for_device(struct max96724_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 100; i++) {
		ret = max96724_read(priv, 0x0);
		if (ret >= 0)
			return 0;

		msleep(10);

		dev_dbg(priv->dev, "Retry %u waiting for deserializer: %d\n", i,
			ret);
	}

	dev_err(priv->dev, "Deserializer not found. Err: %d\n", ret);

	return ret;
}

#define MAX96724_PIN(n) \
	PINCTRL_PIN(n, "mfp" __stringify(n))

static const struct pinctrl_pin_desc max96724_pins[] = {
	MAX96724_PIN(0),
	MAX96724_PIN(1),
	MAX96724_PIN(2),
	MAX96724_PIN(3),
	MAX96724_PIN(4),
	MAX96724_PIN(5),
	MAX96724_PIN(6),
	MAX96724_PIN(7),
	MAX96724_PIN(8),
};

#define MAX96724_GROUP_PINS(name, ...) \
	static const unsigned int name ## _pins[] = { __VA_ARGS__ }

MAX96724_GROUP_PINS(mfp0, 0);
MAX96724_GROUP_PINS(mfp1, 1);
MAX96724_GROUP_PINS(mfp2, 2);
MAX96724_GROUP_PINS(mfp3, 3);
MAX96724_GROUP_PINS(mfp4, 4);
MAX96724_GROUP_PINS(mfp5, 5);
MAX96724_GROUP_PINS(mfp6, 6);
MAX96724_GROUP_PINS(mfp7, 7);
MAX96724_GROUP_PINS(mfp8, 8);

#define MAX96724_GROUP(name) \
	PINCTRL_PINGROUP(__stringify(name), name ## _pins, ARRAY_SIZE(name ## _pins))

static const struct pingroup max96724_ctrl_groups[] = {
	MAX96724_GROUP(mfp0),
	MAX96724_GROUP(mfp1),
	MAX96724_GROUP(mfp2),
	MAX96724_GROUP(mfp3),
	MAX96724_GROUP(mfp4),
	MAX96724_GROUP(mfp5),
	MAX96724_GROUP(mfp6),
	MAX96724_GROUP(mfp7),
	MAX96724_GROUP(mfp8),
};

#define MAX96724_FUNC_GROUPS(name, ...) \
	static const char * const name ## _groups[] = { __VA_ARGS__ }

MAX96724_FUNC_GROUPS(gpio, "mfp0", "mfp1", "mfp2", "mfp3", "mfp4", "mfp5",
			   "mfp6", "mfp7", "mfp8");

enum max96724_func {
	max96724_func_gpio,
};

#define MAX96724_FUNC(name)						\
	[max96724_func_ ## name] =					\
		PINCTRL_PINFUNCTION(__stringify(name), name ## _groups,	\
				    ARRAY_SIZE(name ## _groups))

static const struct pinfunction max96724_functions[] = {
	MAX96724_FUNC(gpio),
};

enum max96724_pinctrl_params {
	MAX96724_PINCTRL_PULL_STRENGTH_WEAK = PIN_CONFIG_END + 1,
	MAX96724_PINCTRL_JITTER_COMPENSATION_EN,
	MAX96724_PINCTRL_GMSL_TX_EN_A,
	MAX96724_PINCTRL_GMSL_RX_EN_A,
	MAX96724_PINCTRL_GMSL_TX_ID_A,
	MAX96724_PINCTRL_GMSL_RX_ID_A,
	MAX96724_PINCTRL_GMSL_TX_EN_B,
	MAX96724_PINCTRL_GMSL_RX_EN_B,
	MAX96724_PINCTRL_GMSL_TX_ID_B,
	MAX96724_PINCTRL_GMSL_RX_ID_B,
	MAX96724_PINCTRL_GMSL_TX_EN_C,
	MAX96724_PINCTRL_GMSL_RX_EN_C,
	MAX96724_PINCTRL_GMSL_TX_ID_C,
	MAX96724_PINCTRL_GMSL_RX_ID_C,
	MAX96724_PINCTRL_GMSL_TX_EN_D,
	MAX96724_PINCTRL_GMSL_RX_EN_D,
	MAX96724_PINCTRL_GMSL_TX_ID_D,
	MAX96724_PINCTRL_GMSL_RX_ID_D,
	MAX96724_PINCTRL_INPUT_VALUE,
};

static const struct pinconf_generic_params max96724_cfg_params[] = {
	{ "maxim,pull-strength-weak", MAX96724_PINCTRL_PULL_STRENGTH_WEAK, 0 },
	{ "maxim,jitter-compensation", MAX96724_PINCTRL_JITTER_COMPENSATION_EN, 0 },
	{ "maxim,gmsl-tx-a", MAX96724_PINCTRL_GMSL_TX_EN_A, 0 },
	{ "maxim,gmsl-rx-a", MAX96724_PINCTRL_GMSL_RX_EN_A, 0 },
	{ "maxim,gmsl-tx-id-a", MAX96724_PINCTRL_GMSL_TX_ID_A, 0 },
	{ "maxim,gmsl-rx-id-a", MAX96724_PINCTRL_GMSL_RX_ID_A, 0 },
	{ "maxim,gmsl-tx-b", MAX96724_PINCTRL_GMSL_TX_EN_B, 0 },
	{ "maxim,gmsl-rx-b", MAX96724_PINCTRL_GMSL_RX_EN_B, 0 },
	{ "maxim,gmsl-tx-id-b", MAX96724_PINCTRL_GMSL_TX_ID_B, 0 },
	{ "maxim,gmsl-rx-id-b", MAX96724_PINCTRL_GMSL_RX_ID_B, 0 },
	{ "maxim,gmsl-tx-c", MAX96724_PINCTRL_GMSL_TX_EN_C, 0 },
	{ "maxim,gmsl-rx-c", MAX96724_PINCTRL_GMSL_RX_EN_C, 0 },
	{ "maxim,gmsl-tx-id-c", MAX96724_PINCTRL_GMSL_TX_ID_C, 0 },
	{ "maxim,gmsl-rx-id-c", MAX96724_PINCTRL_GMSL_RX_ID_C, 0 },
	{ "maxim,gmsl-tx-d", MAX96724_PINCTRL_GMSL_TX_EN_D, 0 },
	{ "maxim,gmsl-rx-d", MAX96724_PINCTRL_GMSL_RX_EN_D, 0 },
	{ "maxim,gmsl-tx-id-d", MAX96724_PINCTRL_GMSL_TX_ID_D, 0 },
	{ "maxim,gmsl-rx-id-d", MAX96724_PINCTRL_GMSL_RX_ID_D, 0 },
};

static int max96724_ctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(max96724_ctrl_groups);
}

static const char *max96724_ctrl_get_group_name(struct pinctrl_dev *pctldev,
						unsigned selector)
{
	return max96724_ctrl_groups[selector].name;
}

static int max96724_ctrl_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
					const unsigned **pins, unsigned *num_pins)
{
	*pins = (unsigned *) max96724_ctrl_groups[selector].pins;
	*num_pins = max96724_ctrl_groups[selector].npins;

	return 0;
}

static int max96724_get_pin_config_reg(unsigned int offset, u32 param,
				       unsigned int *reg, unsigned int *mask,
				       unsigned int *val)
{
	*reg = 0x300 + offset * 0x3;

	switch (param) {
	case PIN_CONFIG_OUTPUT_ENABLE:
		*mask = BIT(0);
		*val = 0b0;
		return 0;
	case PIN_CONFIG_INPUT_ENABLE:
		*mask = BIT(0);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_EN_A:
		*mask = BIT(1);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_A:
		*mask = BIT(2);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_EN_B:
		*reg += 0x37;
		if (offset > 2)
			*reg+=1;
		if (offset > 7)
			*reg+=1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_B:
		*reg += 0x38;
		if (offset > 2)
			*reg+=1;
		if (offset > 7)
			*reg+=1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_EN_C:
		*reg += 0x6d;
		if (offset > 0)
			*reg+=1;
		if (offset > 5)
			*reg+=1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_C:
		*reg += 0x6e;
		if (offset > 0)
			*reg+=1;
		if (offset > 5)
			*reg+=1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_EN_D:
		*reg += 0xa4;
		if (offset > 3)
			*reg+=1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_D:
		*reg += 0xa5;
		if (offset > 3)
			*reg+=1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_INPUT_VALUE:
		*mask = BIT(3);
		*val = 0b1;
		return 0;
	case PIN_CONFIG_OUTPUT:
		*mask = BIT(4);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_JITTER_COMPENSATION_EN:
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_PULL_STRENGTH_WEAK:
		*mask = BIT(7);
		*val = 0b0;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_ID_A:
		*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_ID_B:
		*reg += 0x37;
		if (offset > 2)
			*reg+=1;
		if (offset > 7)
			*reg+=1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_ID_C:
		*reg += 0x6d;
		if (offset > 0)
			*reg+=1;
		if (offset > 5)
			*reg+=1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_ID_D:
		*reg += 0xa4;
		if (offset > 3)
			*reg+=1;
		*mask = GENMASK(4, 0);
		return 0;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		*reg += 1;
		*mask = BIT(5);
		*val = 0b0;
		return 0;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case PIN_CONFIG_BIAS_DISABLE:
		*reg += 1;
		*mask = GENMASK(7, 6);
		*val = 0b00;
		return 0;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		*reg += 1;
		*mask = GENMASK(7, 6);
		*val = 0b10;
		return 0;
	case PIN_CONFIG_BIAS_PULL_UP:
		*reg += 1;
		*mask = GENMASK(7, 6);
		*val = 0b01;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_A:
		*reg += 2;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_B:
		*reg += 0x38;
		if (offset > 2)
			*reg+=1;
		if (offset > 7)
			*reg+=1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_C:
		*reg += 0x6e;
		if (offset > 0)
			*reg+=1;
		if (offset > 5)
			*reg+=1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_D:
		*reg += 0xa5;
		if (offset > 3)
			*reg+=1;
		*mask = GENMASK(4, 0);
		return 0;
	default:
		return -ENOTSUPP;
	}
}

static int max96724_conf_pin_config_get(struct pinctrl_dev *pctldev,
					unsigned int offset,
					unsigned long *config)
{
	struct max96724_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	u32 param = pinconf_to_config_param(*config);
	unsigned int reg, mask, val;
	int ret;

	ret = max96724_get_pin_config_reg(offset, param, &reg, &mask, &val);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		ret = max96724_read(priv, reg);
		if (ret < 0)
			return ret;

		val = max96724_field_get(ret, mask) == val;
		if (!val)
			return -EINVAL;

		break;
	case MAX96724_PINCTRL_JITTER_COMPENSATION_EN:
	case MAX96724_PINCTRL_PULL_STRENGTH_WEAK:
	case MAX96724_PINCTRL_GMSL_TX_EN_A:
	case MAX96724_PINCTRL_GMSL_TX_EN_B:
	case MAX96724_PINCTRL_GMSL_TX_EN_C:
	case MAX96724_PINCTRL_GMSL_TX_EN_D:
	case MAX96724_PINCTRL_GMSL_RX_EN_A:
	case MAX96724_PINCTRL_GMSL_RX_EN_B:
	case MAX96724_PINCTRL_GMSL_RX_EN_C:
	case MAX96724_PINCTRL_GMSL_RX_EN_D:
	case MAX96724_PINCTRL_INPUT_VALUE:
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		ret = max96724_read(priv, reg);
		if (ret < 0)
			return ret;

		val = max96724_field_get(ret, mask) == val;
		break;
	case MAX96724_PINCTRL_GMSL_TX_ID_A:
	case MAX96724_PINCTRL_GMSL_TX_ID_B:
	case MAX96724_PINCTRL_GMSL_TX_ID_C:
	case MAX96724_PINCTRL_GMSL_TX_ID_D:
	case MAX96724_PINCTRL_GMSL_RX_ID_A:
	case MAX96724_PINCTRL_GMSL_RX_ID_B:
	case MAX96724_PINCTRL_GMSL_RX_ID_C:
	case MAX96724_PINCTRL_GMSL_RX_ID_D:
		ret = max96724_read(priv, reg);
		if (ret < 0)
			return ret;

		val = max96724_field_get(val, mask);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, val);

	return 0;
}

static int max96724_conf_pin_config_set_one(struct max96724_priv *priv,
					    unsigned int offset,
					    unsigned long config)
{
	u32 param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	unsigned int reg, mask, val;
	int ret;

	ret = max96724_get_pin_config_reg(offset, param, &reg, &mask, &val);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		val = max96724_field_prep(val, mask);

		ret = max96724_update_bits(priv, reg, mask, val);
		break;
	case MAX96724_PINCTRL_JITTER_COMPENSATION_EN:
	case MAX96724_PINCTRL_PULL_STRENGTH_WEAK:
	case MAX96724_PINCTRL_GMSL_TX_EN_A:
	case MAX96724_PINCTRL_GMSL_TX_EN_B:
	case MAX96724_PINCTRL_GMSL_TX_EN_C:
	case MAX96724_PINCTRL_GMSL_TX_EN_D:
	case MAX96724_PINCTRL_GMSL_RX_EN_A:
	case MAX96724_PINCTRL_GMSL_RX_EN_B:
	case MAX96724_PINCTRL_GMSL_RX_EN_C:
	case MAX96724_PINCTRL_GMSL_RX_EN_D:
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		val = max96724_field_prep(arg ? val : ~val, mask);

		ret = max96724_update_bits(priv, reg, mask, val);
		break;
	case MAX96724_PINCTRL_GMSL_TX_ID_A:
	case MAX96724_PINCTRL_GMSL_TX_ID_B:
	case MAX96724_PINCTRL_GMSL_TX_ID_C:
	case MAX96724_PINCTRL_GMSL_TX_ID_D:
	case MAX96724_PINCTRL_GMSL_RX_ID_A:
	case MAX96724_PINCTRL_GMSL_RX_ID_B:
	case MAX96724_PINCTRL_GMSL_RX_ID_C:
	case MAX96724_PINCTRL_GMSL_RX_ID_D:
		val = max96724_field_prep(arg, mask);

		ret = max96724_update_bits(priv, reg, mask, val);
		break;
	default:
		return -ENOTSUPP;
	}

	if (param == PIN_CONFIG_OUTPUT) {
		config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 1);
		ret = max96724_conf_pin_config_set_one(priv, offset, config);
		if (ret)
			return ret;
	}

	/* Enable for all links if jitter compensation is enabled*/
	if (param == MAX96724_PINCTRL_JITTER_COMPENSATION_EN) {
		max96724_get_pin_config_reg(offset,
					    MAX96724_PINCTRL_GMSL_TX_ID_B, &reg,
					    &mask, &val);
		max96724_update_bits(priv, reg, BIT(6), BIT(6));
		max96724_get_pin_config_reg(offset,
					    MAX96724_PINCTRL_GMSL_TX_ID_C, &reg,
					    &mask, &val);
		max96724_update_bits(priv, reg, BIT(6), BIT(6));
		max96724_get_pin_config_reg(offset,
					    MAX96724_PINCTRL_GMSL_TX_ID_D, &reg,
					    &mask, &val);
		max96724_update_bits(priv, reg, BIT(6), BIT(6));
	}

	return ret;
}

static int max96724_conf_pin_config_set(struct pinctrl_dev *pctldev,
					unsigned int offset,
					unsigned long *configs,
					unsigned int num_configs)
{
	struct max96724_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	int ret;

	while (num_configs--) {
		unsigned long config = *configs;

		ret = max96724_conf_pin_config_set_one(priv, offset, config);
		if (ret)
			return ret;

		configs++;
	}

	return 0;
}

static int max96724_mux_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(max96724_functions);
}

static const char *max96724_mux_get_function_name(struct pinctrl_dev *pctldev,
						  unsigned selector)
{
	return max96724_functions[selector].name;
}

static int max96724_mux_get_groups(struct pinctrl_dev *pctldev,
				   unsigned selector,
				   const char * const **groups,
				   unsigned * const num_groups)
{
	*groups = max96724_functions[selector].groups;
	*num_groups = max96724_functions[selector].ngroups;

	return 0;
}

static int max96724_mux_set(struct pinctrl_dev *pctldev, unsigned selector,
			    unsigned group)
{
	return 0;
}

static int max96724_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 0);
	struct max96724_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96724_conf_pin_config_get(priv->pctldev, offset, &config);
	if (ret)
		return ret;

	return pinconf_to_config_argument(config) ? GPIO_LINE_DIRECTION_OUT
						  : GPIO_LINE_DIRECTION_IN;
}

static int max96724_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1);
	struct max96724_priv *priv = gpiochip_get_data(gc);

	return max96724_conf_pin_config_set_one(priv, offset, config);
}

static int max96724_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
					  int value)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);
	struct max96724_priv *priv = gpiochip_get_data(gc);

	return max96724_conf_pin_config_set_one(priv, offset, config);
}

static int max96724_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(MAX96724_PINCTRL_INPUT_VALUE, 0);
	struct max96724_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96724_conf_pin_config_get(priv->pctldev, offset, &config);
	if (ret)
		return ret;

	return pinconf_to_config_argument(config);
}

static void max96724_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);
	struct max96724_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96724_conf_pin_config_set_one(priv, offset, config);
	if (ret)
		dev_err(priv->dev, "Failed to set GPIO %u output value, err: %d\n",
			offset, ret);
}

static int max96724_gpio_add_pin_ranges(struct gpio_chip *gc)
{
	struct device_node *np = dev_of_node(gc->parent);
	struct pinctrl_dev *pctldev = of_pinctrl_get(np);

	if (!pctldev)
		return 0;

	return gpiochip_add_pin_range(gc, pinctrl_dev_get_devname(pctldev), 0, 0,
				      gc->ngpio);
}

static int max96724_reset(struct max96724_priv *priv)
{
	int ret;

	ret = max96724_wait_for_device(priv);
	if (ret)
		return ret;

	ret = max96724_update_bits(priv, 0x13, 0x40, 0x40);
	if (ret)
		return ret;

	msleep(10);

	ret = max96724_wait_for_device(priv);
	if (ret)
		return ret;

	return 0;
}

static int max96724_mipi_enable(struct max_des_priv *des_priv, bool enable)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	int ret;

	if (enable) {
		ret = max96724_update_bits(priv, 0x40b, 0x02, 0x02);
		if (ret)
			return ret;

		ret = max96724_update_bits(priv, 0x8a0, 0x80, 0x80);
		if (ret)
			return ret;
	} else {
		ret = max96724_update_bits(priv, 0x8a0, 0x80, 0x00);
		if (ret)
			return ret;

		ret = max96724_update_bits(priv, 0x40b, 0x02, 0x00);
		if (ret)
			return ret;
	}

	return 0;
}

static const unsigned int max96724_lane_configs[][MAX96724_PHYS_NUM] = {
	{ 2, 2, 2, 2 }, { 0, 0, 0, 0 }, { 0, 4, 4, 0 },
	{ 0, 4, 2, 2 }, { 2, 2, 4, 0 },
};

static int max96724_init_lane_config(struct max96724_priv *priv)
{
	unsigned int num_lane_configs = ARRAY_SIZE(max96724_lane_configs);
	struct max_des_priv *des_priv = &priv->des_priv;
	struct max_des_phy *phy;
	unsigned int i, j;
	int ret;

	for (i = 0; i < num_lane_configs; i++) {
		bool matching = true;

		for (j = 0; j < des_priv->ops->num_phys; j++) {
			phy = max_des_phy_by_id(des_priv, j);

			if (phy->enabled &&
			    phy->mipi.num_data_lanes !=
				    max96724_lane_configs[i][j]) {
				matching = false;
				break;
			}
		}

		if (matching)
			break;
	}

	if (i == num_lane_configs || i == 1) {
		dev_err(priv->dev,
			"Invalid lane configuration. Num data lines: %d\n",
			phy->mipi.num_data_lanes);

		return -EINVAL;
	}

	/*
	 * TODO: add a fake 2-lane mode that uses the 4-lane config
	 * and disables the other 2 lanes.
	 */
	/* Select 2x4 or 4x2 mode. */
	ret = max96724_update_bits(priv, 0x8a0, 0x1f, BIT(i));
	if (ret)
		return ret;

	return 0;
}

static int max96724_init(struct max_des_priv *des_priv)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	int ret;

	/* Disable all PHYs. */
	ret = max96724_update_bits(priv, 0x8a2, GENMASK(7, 4), 0x00);
	if (ret)
		return ret;

	/* Disable automatic stream select. */
	ret = max96724_update_bits(priv, 0xf4, BIT(4), 0x00);
	if (ret)
		return ret;

	/* Disable all pipes. */
	ret = max96724_update_bits(priv, 0xf4, GENMASK(3, 0), 0x00);
	if (ret)
		return ret;

	ret = max96724_init_lane_config(priv);
	if (ret)
		return ret;

	return 0;
}

static int max96724_init_phy(struct max_des_priv *des_priv,
			     struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	unsigned int num_data_lanes = phy->mipi.num_data_lanes;
	unsigned int reg, val, shift, mask, clk_bit;
	unsigned int index = phy->index;
	unsigned int i;
	int ret;

	/* Configure a lane count. */
	/* TODO: Add support CPHY mode. */
	ret = max96724_update_bits(priv, 0x90a + 0x40 * index, GENMASK(7, 6),
				   FIELD_PREP(GENMASK(7, 6),
					      num_data_lanes - 1));
	if (ret)
		return ret;

	if (num_data_lanes == 4) {
		mask = 0xff;
		val = 0xe4;
		shift = 0;
	} else {
		mask = 0xf;
		val = 0x4;
		shift = 4 * (index % 2);
	}

	reg = 0x8a3 + index / 2;

	/* Configure lane mapping. */
	/* TODO: Add support for lane swapping. */
	ret = max96724_update_bits(priv, reg, mask << shift, val << shift);
	if (ret)
		return ret;

	if (num_data_lanes == 4) {
		mask = 0x3f;
		clk_bit = 5;
		shift = 0;
	} else {
		mask = 0x7;
		clk_bit = 2;
		shift = 4 * (index % 2);
	}

	reg = 0x8a5 + index / 2;

	/* Configure lane polarity. */
	val = 0;
	for (i = 0; i < num_data_lanes + 1; i++)
		if (phy->mipi.lane_polarities[i])
			val |= BIT(i == 0 ? clk_bit : i < 3 ? i - 1 : i);
	ret = max96724_update_bits(priv, reg, mask << shift, val << shift);
	if (ret)
		return ret;

	/* Put DPLL block into reset. */
	ret = max96724_update_bits(priv, 0x1c00 + 0x100 * index, BIT(0), 0x00);
	if (ret)
		return ret;

	/* Set DPLL frequency. */
	reg = 0x415 + 0x3 * index;
	ret = max96724_update_bits(priv, reg, GENMASK(4, 0),
				   MAX96724_DPLL_FREQ / 100);
	if (ret)
		return ret;

	/* Enable DPLL frequency. */
	ret = max96724_update_bits(priv, reg, BIT(5), BIT(5));
	if (ret)
		return ret;

	/* Pull DPLL block out of reset. */
	ret = max96724_update_bits(priv, 0x1c00 + 0x100 * index, BIT(0), 0x01);
	if (ret)
		return ret;

	/* Disable initial deskew. */
	ret = max96724_write(priv, 0x903 + 0x40 * index, 0x07);
	if (ret)
		return ret;

	/* Disable periodic deskew. */
	ret = max96724_write(priv, 0x904 + 0x40 * index, 0x01);
	if (ret)
		return ret;

	/* Set alternate memory map modes. */
	val = phy->alt_mem_map12 ? BIT(0) : 0;
	val |= phy->alt_mem_map8 ? BIT(1) : 0;
	val |= phy->alt_mem_map10 ? BIT(2) : 0;
	ret = max96724_update_bits(priv, 0x933 + 0x40 * index, GENMASK(2, 0),
				   val);

	/* Enable PHY. */
	mask = BIT(index + 4);
	ret = max96724_update_bits(priv, 0x8a2, mask, mask);
	if (ret)
		return ret;

	return 0;
}

static int max96724_init_pipe_remap(struct max96724_priv *priv,
				    struct max_des_pipe *pipe,
				    struct max_des_dt_vc_remap *remap,
				    unsigned int i)
{
	unsigned int index = pipe->index;
	unsigned int reg, val, shift, mask;
	int ret;

	/* Set source Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	reg = 0x90d + 0x40 * index + i * 2;
	ret = max96724_write(priv, reg,
			     MAX_DES_DT_VC(remap->from_dt, remap->from_vc));
	if (ret)
		return ret;

	/* Set destination Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	reg = 0x90e + 0x40 * index + i * 2;
	ret = max96724_write(priv, reg,
			     MAX_DES_DT_VC(remap->to_dt, remap->to_vc));
	if (ret)
		return ret;

	/* Set destination PHY. */
	reg = 0x92d + 0x40 * index + i / 4;
	shift = (i % 4) * 2;
	mask = 0x3 << shift;
	val = (remap->phy & 0x3) << shift;
	ret = max96724_update_bits(priv, reg, mask, val);
	if (ret)
		return ret;

	/* Enable remap. */
	reg = 0x90b + 0x40 * index + i / 8;
	val = BIT(i % 8);
	ret = max96724_update_bits(priv, reg, val, val);
	if (ret)
		return ret;

	return 0;
}

static int max96724_update_pipe_remaps(struct max_des_priv *des_priv,
				       struct max_des_pipe *pipe)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	unsigned int i;
	int ret;

	for (i = 0; i < pipe->num_remaps; i++) {
		struct max_des_dt_vc_remap *remap = &pipe->remaps[i];

		ret = max96724_init_pipe_remap(priv, pipe, remap, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int max96724_init_pipe(struct max_des_priv *des_priv,
			      struct max_des_pipe *pipe)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	unsigned int index = pipe->index;
	unsigned int reg, shift;
	int ret;

	/* Set destination PHY. */
	shift = index * 2;
	ret = max96724_update_bits(priv, 0x8ca, GENMASK(1, 0) << shift,
				   pipe->phy_id << shift);
	if (ret)
		return ret;

	shift = 4;
	reg = 0x939 + 0x40 * index;
	ret = max96724_update_bits(priv, reg, GENMASK(1, 0) << shift,
				   pipe->phy_id << shift);
	if (ret)
		return ret;

	/* Enable pipe. */
	ret = max96724_update_bits(priv, 0xf4, BIT(index), BIT(index));
	if (ret)
		return ret;

	/* Set source stream. */
	reg = 0xf0 + index / 2;
	shift = 4 * (index % 2);
	ret = max96724_update_bits(priv, reg, GENMASK(1, 0) << shift,
			 0 << shift);
	if (ret)
		return ret;

	/* Set source link. */
	shift += 2;
	ret = max96724_update_bits(priv, reg, GENMASK(1, 0) << shift,
				   pipe->link_id << shift);
	if (ret)
		return ret;

	return 0;
}

static int max96724_select_links(struct max_des_priv *des_priv,
				 unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	int ret;

	ret = max96724_update_bits(priv, 0x6, GENMASK(3, 0), mask);
	if (ret)
		return ret;

	msleep(60);

	return 0;
}

static struct pinctrl_ops max96724_ctrl_ops = {
	.get_groups_count = max96724_ctrl_get_groups_count,
	.get_group_name = max96724_ctrl_get_group_name,
	.get_group_pins = max96724_ctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinconf_ops max96724_conf_ops = {
	.pin_config_get = max96724_conf_pin_config_get,
	.pin_config_set = max96724_conf_pin_config_set,
	.is_generic = true,
};

static const struct pinmux_ops max96724_mux_ops = {
	.get_functions_count = max96724_mux_get_functions_count,
	.get_function_name = max96724_mux_get_function_name,
	.get_function_groups = max96724_mux_get_groups,
	.set_mux = max96724_mux_set,
	.strict = true,
};

static const struct max_des_ops max96724_ops = {
	.num_phys = 4,
	.num_pipes = 4,
	.num_links = 4,
	.supports_pipe_link_remap = true,
	.mipi_enable = max96724_mipi_enable,
	.init = max96724_init,
	.init_phy = max96724_init_phy,
	.init_pipe = max96724_init_pipe,
	.update_pipe_remaps = max96724_update_pipe_remaps,
	.select_links = max96724_select_links,
};

static int max96724_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96724_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &max_des_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->des_priv.dev = dev;
	priv->des_priv.client = client;
	priv->des_priv.regmap = priv->regmap;
	priv->des_priv.ops = &max96724_ops;

	ret = max96724_reset(priv);
	if (ret)
		return ret;

	priv->pctldesc = (struct pinctrl_desc){
		.owner = THIS_MODULE,
		.name = MAX96724_NAME,
		.pins = max96724_pins,
		.npins = ARRAY_SIZE(max96724_pins),
		.pctlops = &max96724_ctrl_ops,
		.confops = &max96724_conf_ops,
		.pmxops = &max96724_mux_ops,
		.custom_params = max96724_cfg_params,
		.num_custom_params = ARRAY_SIZE(max96724_cfg_params),
	};

	ret = devm_pinctrl_register_and_init(dev, &priv->pctldesc, priv,
					     &priv->pctldev);
	if (ret)
		return ret;

	ret = pinctrl_enable(priv->pctldev);
	if (ret)
		return ret;

	priv->gc = (struct gpio_chip){
		.owner = THIS_MODULE,
		.label = MAX96724_NAME,
		.base = -1,
		.ngpio = MAX96724_GPIO_NUM,
		.parent = dev,
		.can_sleep = true,
		.request = gpiochip_generic_request,
		.free = gpiochip_generic_free,
		.set_config = gpiochip_generic_config,
		.get_direction = max96724_gpio_get_direction,
		.direction_input = max96724_gpio_direction_input,
		.direction_output = max96724_gpio_direction_output,
		.get = max96724_gpio_get,
		.set = max96724_gpio_set,
		.add_pin_ranges = max96724_gpio_add_pin_ranges,
	};

	ret = devm_gpiochip_add_data(dev, &priv->gc, priv);
	if (ret)
		return ret;

	return max_des_probe(&priv->des_priv);
}

static void max96724_remove(struct i2c_client *client)
{
	struct max96724_priv *priv = i2c_get_clientdata(client);

	max_des_remove(&priv->des_priv);
}

static const struct of_device_id max96724_of_table[] = {
	{ .compatible = "maxim,max96724" },
	{},
};
MODULE_DEVICE_TABLE(of, max96724_of_table);

static struct i2c_driver max96724_i2c_driver = {
	.driver	= {
		.name = MAX96724_NAME,
		.of_match_table	= of_match_ptr(max96724_of_table),
	},
	.probe_new = max96724_probe,
	.remove = max96724_remove,
};

module_i2c_driver(max96724_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX96724 Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
