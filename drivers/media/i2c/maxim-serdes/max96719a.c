// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX96719A GMSL2 Serializer Driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/regmap.h>

#include "max_ser.h"

#define MAX96719A_REG0				0x0

#define MAX96719A_RCLKOUT_CTRL			0x14
#define MAX96719A_RCLKOUT_CTRL_RCLKOUT_EN	BIT(5)

#define MAX96719A_WAKE_CTRL			0x101
#define MAX96719A_WAKE_CTRL_WAKE_EN		BIT(2)

#define MAX96719A_MIPI_RX13			0xa018
#define MAX96719A_MIPI_RX14			0xa019
#define MAX96719A_CSI_RX_TUN_PKT_CNT		0xa126

#define MAX96719A_VC_REMAP_CTRL			0xa127
#define MAX96719A_VC_REMAP_CTRL_CUST_EN		GENMASK(7, 4)

#define MAX96719A_VC_REMAP_CUST0(x)		0xa129
#define MAX96719A_VC_REMAP_CUST0_RCVD_VC0	GENMASK(3, 0)
#define MAX96719A_VC_REMAP_CUST0_MAPPED_VC0	GENMASK(7, 4)

#define MAX96719A_VIDEO_TX_0			0x2003
#define MAX96719A_VIDEO_TX_0_VIDEO_ENABLE	BIT(0)

#define MAX96719A_VIDEO_TX2			0x2005
#define MAX96719A_VIDEO_TX2_PCLKDET		BIT(1)

#define MAX96719A_GPIO_CFG0_X(x)		(0x302b + (x) * 0xa)
#define MAX96719A_GPIO_CFG0_X_GPIO_OUT_DIS	BIT(0)
#define MAX96719A_GPIO_CFG0_X_GPIO_TX_EN	BIT(1)
#define MAX96719A_GPIO_CFG0_X_GPIO_RX_EN	BIT(2)
#define MAX96719A_GPIO_CFG0_X_GPIO_OUT		BIT(3)
#define MAX96719A_GPIO_CFG0_X_PULL_UPDN_SEL	GENMASK(6, 5)
#define MAX96719A_GPIO_CFG0_X_PULL_UPDN_SEL_PU	0b01
#define MAX96719A_GPIO_CFG0_X_PULL_UPDN_SEL_PD	0b10

#define MAX96719A_GPIO_RXID_X(x)		(0x302d + (x) * 0xa)
#define MAX96719A_GPIO_RXID_X_GMSL_RX_ID	GENMASK(4, 0)

#define MAX96719A_GPIO_TXID_X(x)		(0x302e + (x) * 0xa)
#define MAX96719A_GPIO_TXID_X_GMSL_TX_ID	GENMASK(4, 0)

#define MAX96719A_GPIO_STAT0_X(x)		(0x3030 + (x) * 0xa)
#define MAX96719A_GPIO_STAT0_X_GPIO_IN		BIT(0)

#define MAX96719A_GPIO_SLEW_CFG_X(x)		(0x3032 + (x) * 0xa)
#define MAX96719A_GPIO_SLEW_CFG_X_GPIO_SLEW	BIT(0)

#define MAX96719A_SRC_ADDR_0(x)			(0x351a + (x) * 0x2)
#define MAX96719A_SRC_ADDR_0_SRC_ADDR		GENMASK(7, 1)

#define MAX96719A_DST_ADDR_0(x)			(0x351b + (x) * 0x2)
#define MAX96719A_DST_ADDR_0_DST_ADDR		GENMASK(7, 1)

#define MAX96719A_PDM_TX_CTRL			0x3704
#define MAX96719A_PDM_TX_CTRL_PDM_EN		BIT(0)

#define MAX96719A_MIPI_RX0			0xa005
#define MAX96719A_MIPI_RX0_LANE_COUNT		GENMASK(6, 5)

#define MAX96719A_DEFAULT_CLKOUT_RATE		25000000UL

#define MAX96719A_STREAM_ID			2

#define MAX96719A_NAME				"max96719a"
#define MAX96719A_PINCTRL_NAME			MAX96719A_NAME "-pinctrl"
#define MAX96719A_GPIOCHIP_NAME			MAX96719A_NAME "-gpiochip"
#define MAX96719A_GPIO_NUM			7

#define field_get(mask, val) (((val) & (mask)) >> __ffs(mask))
#define field_prep(mask, val) (((val) << __ffs(mask)) & (mask))

struct max96719a_priv {
	struct max_ser ser;
	struct pinctrl_desc pctldesc;
	struct gpio_chip gc;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct pinctrl_dev *pctldev;

	struct clk_hw clk_hw;
};

#define ser_to_priv(_ser) \
	container_of(_ser, struct max96719a_priv, ser)

static inline struct max96719a_priv *clk_hw_to_priv(struct clk_hw *hw)
{
	return container_of(hw, struct max96719a_priv, clk_hw);
}

static const struct regmap_config max96719a_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xa148,
};

static int max96719a_wait_for_device(struct max96719a_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 10; i++) {
		unsigned int val;

		ret = regmap_read(priv->regmap, MAX96719A_REG0, &val);
		if (!ret && val)
			return 0;

		msleep(100);

		dev_err(priv->dev, "Retry %u waiting for serializer: %d\n", i, ret);
	}

	return ret;
}

#define MAX96719A_PIN(n) \
	PINCTRL_PIN(n, "gpio" __stringify(n))

static const struct pinctrl_pin_desc max96719a_pins[] = {
	MAX96719A_PIN(0),
	MAX96719A_PIN(1),
	MAX96719A_PIN(2),
	MAX96719A_PIN(3),
	MAX96719A_PIN(4),
	MAX96719A_PIN(5),
	MAX96719A_PIN(6),
};

#define MAX96719A_GROUP_PINS(name, ...) \
	static const unsigned int name ## _pins[] = { __VA_ARGS__ }

MAX96719A_GROUP_PINS(gpio0, 0);
MAX96719A_GROUP_PINS(gpio1, 1);
MAX96719A_GROUP_PINS(gpio2, 2);
MAX96719A_GROUP_PINS(gpio3, 3);
MAX96719A_GROUP_PINS(gpio4, 4);
MAX96719A_GROUP_PINS(gpio5, 5);
MAX96719A_GROUP_PINS(gpio6, 6);

#define MAX96719A_GROUP(name) \
	PINCTRL_PINGROUP(__stringify(name), name ## _pins, ARRAY_SIZE(name ## _pins))

static const struct pingroup max96719a_ctrl_groups[] = {
	MAX96719A_GROUP(gpio0),
	MAX96719A_GROUP(gpio1),
	MAX96719A_GROUP(gpio2),
	MAX96719A_GROUP(gpio3),
	MAX96719A_GROUP(gpio4),
	MAX96719A_GROUP(gpio5),
	MAX96719A_GROUP(gpio6),
};

#define MAX96719A_FUNC_GROUPS(name, ...) \
	static const char * const name ## _groups[] = { __VA_ARGS__ }

MAX96719A_FUNC_GROUPS(gpio, "gpio0", "gpio1", "gpio2", "gpio3", "gpio4",
		      "gpio5", "gpio6");

enum max96719a_func {
	max96719a_func_gpio,
};

#define MAX96719A_FUNC(name)						\
	[max96719a_func_ ## name] =					\
		PINCTRL_PINFUNCTION(__stringify(name), name ## _groups,	\
				    ARRAY_SIZE(name ## _groups))

static const struct pinfunction max96719a_functions[] = {
	MAX96719A_FUNC(gpio),
};

#define MAX96719A_PINCTRL_X(x)				(PIN_CONFIG_END + (x))
#define MAX96719A_PINCTRL_GMSL_TX_EN			MAX96719A_PINCTRL_X(0)
#define MAX96719A_PINCTRL_GMSL_RX_EN			MAX96719A_PINCTRL_X(1)
#define MAX96719A_PINCTRL_GMSL_TX_ID			MAX96719A_PINCTRL_X(2)
#define MAX96719A_PINCTRL_GMSL_RX_ID			MAX96719A_PINCTRL_X(3)
#define MAX96719A_PINCTRL_INPUT_VALUE			MAX96719A_PINCTRL_X(4)

static const struct pinconf_generic_params max96719a_cfg_params[] = {
	{ "maxim,gmsl-tx", MAX96719A_PINCTRL_GMSL_TX_EN, 0 },
	{ "maxim,gmsl-rx", MAX96719A_PINCTRL_GMSL_RX_EN, 0 },
	{ "maxim,gmsl-tx-id", MAX96719A_PINCTRL_GMSL_TX_ID, 0 },
	{ "maxim,gmsl-rx-id", MAX96719A_PINCTRL_GMSL_RX_ID, 0 },
};

static int max96719a_ctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(max96719a_ctrl_groups);
}

static const char *max96719a_ctrl_get_group_name(struct pinctrl_dev *pctldev,
						 unsigned int selector)
{
	return max96719a_ctrl_groups[selector].name;
}

static int max96719a_ctrl_get_group_pins(struct pinctrl_dev *pctldev,
					 unsigned int selector,
					 const unsigned int **pins,
					 unsigned int *num_pins)
{
	*pins = (unsigned int *)max96719a_ctrl_groups[selector].pins;
	*num_pins = max96719a_ctrl_groups[selector].npins;

	return 0;
}

static int max96719a_get_pin_config_reg(unsigned int offset, u32 param,
				        unsigned int *reg, unsigned int *mask,
				        unsigned int *val)
{
	*reg = MAX96719A_GPIO_CFG0_X(offset);

	switch (param) {
	case PIN_CONFIG_OUTPUT_ENABLE:
		*mask = MAX96719A_GPIO_CFG0_X_GPIO_OUT_DIS;
		*val = 0b0;
		return 0;
	case PIN_CONFIG_INPUT_ENABLE:
		*mask = MAX96719A_GPIO_CFG0_X_GPIO_OUT_DIS;
		*val = 0b1;
		return 0;
	case MAX96719A_PINCTRL_GMSL_TX_EN:
		*mask = MAX96719A_GPIO_CFG0_X_GPIO_TX_EN;
		*val = 0b1;
		return 0;
	case MAX96719A_PINCTRL_GMSL_RX_EN:
		*mask = MAX96719A_GPIO_CFG0_X_GPIO_RX_EN;
		*val = 0b1;
		return 0;
	case PIN_CONFIG_OUTPUT:
		*mask = MAX96719A_GPIO_CFG0_X_GPIO_OUT;
		*val = 0b1;
		return 0;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		*mask = MAX96719A_GPIO_CFG0_X_PULL_UPDN_SEL;
		*val = MAX96719A_GPIO_CFG0_X_PULL_UPDN_SEL_PD;
		return 0;
	case PIN_CONFIG_BIAS_PULL_UP:
		*mask = MAX96719A_GPIO_CFG0_X_PULL_UPDN_SEL;
		*val = MAX96719A_GPIO_CFG0_X_PULL_UPDN_SEL_PU;
		return 0;
	}

	switch (param) {
	case MAX96719A_PINCTRL_GMSL_RX_ID:
		*reg = MAX96719A_GPIO_RXID_X(offset);
		*mask = MAX96719A_GPIO_RXID_X_GMSL_RX_ID;
		return 0;
	case MAX96719A_PINCTRL_GMSL_TX_ID:
		*reg = MAX96719A_GPIO_TXID_X(offset);
		*mask = MAX96719A_GPIO_TXID_X_GMSL_TX_ID;
		return 0;
	case MAX96719A_PINCTRL_INPUT_VALUE:
		*reg = MAX96719A_GPIO_STAT0_X(offset);
		*mask = MAX96719A_GPIO_STAT0_X_GPIO_IN;
		return 0;
	case PIN_CONFIG_SLEW_RATE:
		*reg = MAX96719A_GPIO_SLEW_CFG_X(offset);
		*mask = MAX96719A_GPIO_SLEW_CFG_X_GPIO_SLEW;
		return 0;
	default:
		return -ENOTSUPP;
	}
}

static int max96719a_conf_pin_config_get(struct pinctrl_dev *pctldev,
					 unsigned int offset,
					 unsigned long *config)
{
	struct max96719a_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	u32 param = pinconf_to_config_param(*config);
	unsigned int reg, mask, val, en_val;
	int ret;

	ret = max96719a_get_pin_config_reg(offset, param, &reg, &mask, &en_val);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
	case MAX96719A_PINCTRL_GMSL_TX_EN:
	case MAX96719A_PINCTRL_GMSL_RX_EN:
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_INPUT_ENABLE:
		ret = regmap_read(priv->regmap, reg, &val);
		if (ret)
			return ret;

		val = field_get(mask, val) == en_val;
		if (!val)
			return -EINVAL;

		break;
	case MAX96719A_PINCTRL_INPUT_VALUE:
	case PIN_CONFIG_OUTPUT:
		ret = regmap_read(priv->regmap, reg, &val);
		if (ret)
			return ret;

		val = field_get(mask, val) == en_val;
		break;
	case MAX96719A_PINCTRL_GMSL_TX_ID:
	case MAX96719A_PINCTRL_GMSL_RX_ID:
	case PIN_CONFIG_SLEW_RATE:
		ret = regmap_read(priv->regmap, reg, &val);
		if (ret)
			return ret;

		val = field_get(mask, val);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, val);

	return 0;
}

static int max96719a_conf_pin_config_set_one(struct max96719a_priv *priv,
					     unsigned int offset,
					     unsigned long config)
{
	u32 param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	unsigned int reg, mask, val, en_val;
	int ret;

	ret = max96719a_get_pin_config_reg(offset, param, &reg, &mask, &en_val);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		val = field_prep(mask, en_val);

		ret = regmap_update_bits(priv->regmap, reg, mask, val);
		break;
	case MAX96719A_PINCTRL_GMSL_TX_EN:
	case MAX96719A_PINCTRL_GMSL_RX_EN:
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		val = field_prep(mask, arg ? en_val : ~en_val);

		ret = regmap_update_bits(priv->regmap, reg, mask, val);
		break;
	case MAX96719A_PINCTRL_GMSL_TX_ID:
	case MAX96719A_PINCTRL_GMSL_RX_ID:
	case PIN_CONFIG_SLEW_RATE:
		val = field_prep(mask, arg);

		ret = regmap_update_bits(priv->regmap, reg, mask, val);
		break;
	default:
		return -ENOTSUPP;
	}

	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_OUTPUT:
		config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 1);
		return max96719a_conf_pin_config_set_one(priv, offset, config);
	case PIN_CONFIG_OUTPUT_ENABLE:
		config = pinconf_to_config_packed(MAX96719A_PINCTRL_GMSL_RX_EN, 0);
		return max96719a_conf_pin_config_set_one(priv, offset, config);
	default:
		break;
	}

	return 0;
}

static int max96719a_conf_pin_config_set(struct pinctrl_dev *pctldev,
					 unsigned int offset,
					 unsigned long *configs,
					 unsigned int num_configs)
{
	struct max96719a_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	int ret;

	while (num_configs--) {
		unsigned long config = *configs;

		ret = max96719a_conf_pin_config_set_one(priv, offset, config);
		if (ret)
			return ret;

		configs++;
	}

	return 0;
}

static int max96719a_mux_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(max96719a_functions);
}

static const char *max96719a_mux_get_function_name(struct pinctrl_dev *pctldev,
						   unsigned int selector)
{
	return max96719a_functions[selector].name;
}

static int max96719a_mux_get_groups(struct pinctrl_dev *pctldev,
				    unsigned int selector,
				    const char * const **groups,
				    unsigned int * const num_groups)
{
	*groups = max96719a_functions[selector].groups;
	*num_groups = max96719a_functions[selector].ngroups;

	return 0;
}

static int max96719a_mux_set(struct pinctrl_dev *pctldev, unsigned int selector,
			     unsigned int group)
{
	struct max96719a_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	switch (selector) {
	case max96719a_func_gpio:
		switch (group) {
		case 0:
			return regmap_clear_bits(priv->regmap, MAX96719A_WAKE_CTRL,
						 MAX96719A_WAKE_CTRL_WAKE_EN);
		case 2:
		case 3:
			return regmap_clear_bits(priv->regmap, MAX96719A_PDM_TX_CTRL,
						 MAX96719A_PDM_TX_CTRL_PDM_EN);
		}
		break;
	}

	return 0;
}

static int max96719a_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 0);
	struct max96719a_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96719a_conf_pin_config_get(priv->pctldev, offset, &config);
	if (ret)
		return ret;

	return pinconf_to_config_argument(config) ? GPIO_LINE_DIRECTION_OUT
						  : GPIO_LINE_DIRECTION_IN;
}

static int max96719a_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1);
	struct max96719a_priv *priv = gpiochip_get_data(gc);

	return max96719a_conf_pin_config_set_one(priv, offset, config);
}

static int max96719a_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
					   int value)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);
	struct max96719a_priv *priv = gpiochip_get_data(gc);

	return max96719a_conf_pin_config_set_one(priv, offset, config);
}

static int max96719a_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(MAX96719A_PINCTRL_INPUT_VALUE, 0);
	struct max96719a_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96719a_conf_pin_config_get(priv->pctldev, offset, &config);
	if (ret)
		return ret;

	return pinconf_to_config_argument(config);
}

static void max96719a_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);
	struct max96719a_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96719a_conf_pin_config_set_one(priv, offset, config);
	if (ret)
		dev_err(priv->dev, "Failed to set GPIO %u output value, err: %d\n",
			offset, ret);
}

static int max96719a_set_pipe_enable(struct max_ser *ser,
				     struct max_ser_pipe *pipe, bool enable)
{
	struct max96719a_priv *priv = ser_to_priv(ser);

	return regmap_assign_bits(priv->regmap, MAX96719A_VIDEO_TX_0,
				  MAX96719A_VIDEO_TX_0_VIDEO_ENABLE, enable);
}

static unsigned int max96719a_get_pipe_stream_id(struct max_ser *ser,
						 struct max_ser_pipe *pipe)
{
	/*
	 * MAX96719A only has one pipe, which has a HW id of 2, which is what
	 * its stream id is hardcoded to, too. Ignore the received stream_id
	 * and set it to the hardcoded value.
	 */
	return MAX96719A_STREAM_ID;
}

static int max96719a_set_pipe_vc_remap(struct max_ser *ser, struct max_ser_pipe *pipe,
				       unsigned int i, struct max_vc_remap *vc_remap)
{
	struct max96719a_priv *priv = ser_to_priv(ser);

	return regmap_write(priv->regmap, MAX96719A_VC_REMAP_CUST0(i),
			    FIELD_PREP(MAX96719A_VC_REMAP_CUST0_RCVD_VC0, vc_remap->src) |
			    FIELD_PREP(MAX96719A_VC_REMAP_CUST0_MAPPED_VC0, vc_remap->dst));
}

static int max96719a_set_pipe_vc_remaps_enable(struct max_ser *ser,
					       struct max_ser_pipe *pipe,
					       unsigned int mask)
{
	struct max96719a_priv *priv = ser_to_priv(ser);

	return regmap_update_bits(priv->regmap, MAX96719A_VC_REMAP_CTRL,
				  MAX96719A_VC_REMAP_CTRL_CUST_EN,
				  FIELD_PREP(MAX96719A_VC_REMAP_CTRL_CUST_EN, mask));
}

static int max96719a_reg_read(struct max_ser *ser, unsigned int reg,
			      unsigned int *val)
{
	struct max96719a_priv *priv = ser_to_priv(ser);

	return regmap_read(priv->regmap, reg, val);
}

static int max96719a_reg_write(struct max_ser *ser, unsigned int reg,
			       unsigned int val)
{
	struct max96719a_priv *priv = ser_to_priv(ser);

	return regmap_write(priv->regmap, reg, val);
}

static int max96719a_log_status(struct max_ser *ser, const char *name)
{
	struct max96719a_priv *priv = ser_to_priv(ser);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, MAX96719A_CSI_RX_TUN_PKT_CNT, &val);
	if (ret)
		return ret;

	pr_info("%s: tun_pkt_cnt: %u\n", name, val);

	ret = regmap_read(priv->regmap, MAX96719A_MIPI_RX13, &val);
	if (ret)
		return ret;

	pr_info("%s: \tphy_pkt_cnt: %u\n", name, val);

	ret = regmap_read(priv->regmap, MAX96719A_MIPI_RX14, &val);
	if (ret)
		return ret;

	pr_info("%s: \tphy_clk_cnt: %u\n", name, val);

	ret = regmap_read(priv->regmap, MAX96719A_VIDEO_TX2, &val);
	if (ret)
		return ret;

	pr_info("%s: \tpclkdet: %u\n", name, !!(val & MAX96719A_VIDEO_TX2_PCLKDET));

	return 0;
}

static int max96719a_init_phy(struct max_ser *ser,
			      struct max_ser_phy *phy)
{
	struct max96719a_priv *priv = ser_to_priv(ser);
	unsigned int num_data_lanes = phy->mipi.num_data_lanes;
	unsigned int i;

	if (num_data_lanes == 3 || num_data_lanes == 1) {
		dev_err(priv->dev, "Unsupported number of data lanes\n");
		return -EINVAL;
	}

	for (i = 0; i < num_data_lanes; i++) {
		if (phy->mipi.data_lanes[i] != i + 1) {
			dev_err(priv->dev, "Unsupported lane remapping\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < num_data_lanes + 1; i++) {
		if (phy->mipi.lane_polarities[i]) {
			dev_err(priv->dev, "Unsupported lane polarity\n");
			return -EINVAL;
		}
	}

	return regmap_update_bits(priv->regmap, MAX96719A_MIPI_RX0,
				  MAX96719A_MIPI_RX0_LANE_COUNT,
				  FIELD_PREP(MAX96719A_MIPI_RX0_LANE_COUNT,
					     num_data_lanes - 1));
}

static int max96719a_set_i2c_xlate(struct max_ser *ser, unsigned int i,
				   struct max_i2c_xlate *xlate)
{
	struct max96719a_priv *priv = ser_to_priv(ser);
	int ret;

	ret = regmap_update_bits(priv->regmap, MAX96719A_SRC_ADDR_0(i),
				 MAX96719A_SRC_ADDR_0_SRC_ADDR,
				 FIELD_PREP(MAX96719A_SRC_ADDR_0_SRC_ADDR, xlate->src));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, MAX96719A_DST_ADDR_0(i),
				 MAX96719A_DST_ADDR_0_DST_ADDR,
				 FIELD_PREP(MAX96719A_DST_ADDR_0_DST_ADDR, xlate->dst));
	if (ret)
		return ret;

	return 0;
}

static const struct pinctrl_ops max96719a_ctrl_ops = {
	.get_groups_count = max96719a_ctrl_get_groups_count,
	.get_group_name = max96719a_ctrl_get_group_name,
	.get_group_pins = max96719a_ctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinconf_ops max96719a_conf_ops = {
	.pin_config_get = max96719a_conf_pin_config_get,
	.pin_config_set = max96719a_conf_pin_config_set,
	.is_generic = true,
};

static const struct pinmux_ops max96719a_mux_ops = {
	.get_functions_count = max96719a_mux_get_functions_count,
	.get_function_name = max96719a_mux_get_function_name,
	.get_function_groups = max96719a_mux_get_groups,
	.set_mux = max96719a_mux_set,
};

static const struct max_phys_config max96719a_phys_configs[] = {
	{ { 4 } },
};

static const struct max_ser_ops max96719a_ops = {
	.num_i2c_xlates = 7,
	.num_pipes = 1,
	.num_phys = 1,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max96719a_phys_configs),
		.configs = max96719a_phys_configs,
	},
	.reg_read = max96719a_reg_read,
	.reg_write = max96719a_reg_write,
	.log_status = max96719a_log_status,
	.set_i2c_xlate = max96719a_set_i2c_xlate,
	.init_phy = max96719a_init_phy,
	.set_pipe_enable = max96719a_set_pipe_enable,
	.get_pipe_stream_id = max96719a_get_pipe_stream_id,
	.set_pipe_vc_remap = max96719a_set_pipe_vc_remap,
	.set_pipe_vc_remaps_enable = max96719a_set_pipe_vc_remaps_enable,
};

static unsigned long max96719a_clk_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	return MAX96719A_DEFAULT_CLKOUT_RATE;
}

static long max96719a_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	return MAX96719A_DEFAULT_CLKOUT_RATE;
}

static int max96719a_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	return 0;
}

static int max96719a_clk_prepare(struct clk_hw *hw)
{
	struct max96719a_priv *priv = clk_hw_to_priv(hw);

	return regmap_set_bits(priv->regmap, MAX96719A_RCLKOUT_CTRL,
			       MAX96719A_RCLKOUT_CTRL_RCLKOUT_EN);
}

static void max96719a_clk_unprepare(struct clk_hw *hw)
{
	struct max96719a_priv *priv = clk_hw_to_priv(hw);

	regmap_clear_bits(priv->regmap, MAX96719A_RCLKOUT_CTRL,
			  MAX96719A_RCLKOUT_CTRL_RCLKOUT_EN);
}

static const struct clk_ops max96719a_clk_ops = {
	.prepare     = max96719a_clk_prepare,
	.unprepare   = max96719a_clk_unprepare,
	.set_rate    = max96719a_clk_set_rate,
	.recalc_rate = max96719a_clk_recalc_rate,
	.round_rate  = max96719a_clk_round_rate,
};

static int max96719a_register_clkout(struct max96719a_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct clk_init_data init = { .ops = &max96719a_clk_ops };
	int ret;

	init.name = kasprintf(GFP_KERNEL, "max96719a.%s.clk_out", dev_name(dev));
	if (!init.name)
		return -ENOMEM;

	priv->clk_hw.init = &init;

	ret = devm_clk_hw_register(dev, &priv->clk_hw);
	kfree(init.name);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot register clock HW\n");

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					  &priv->clk_hw);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Cannot add OF clock provider\n");

	return 0;
}

static int max96719a_gpiochip_probe(struct max96719a_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	priv->pctldesc = (struct pinctrl_desc) {
		.owner = THIS_MODULE,
		.name = MAX96719A_PINCTRL_NAME,
		.pins = max96719a_pins,
		.npins = ARRAY_SIZE(max96719a_pins),
		.pctlops = &max96719a_ctrl_ops,
		.confops = &max96719a_conf_ops,
		.pmxops = &max96719a_mux_ops,
		.custom_params = max96719a_cfg_params,
		.num_custom_params = ARRAY_SIZE(max96719a_cfg_params),
	};

	ret = devm_pinctrl_register_and_init(dev, &priv->pctldesc, priv, &priv->pctldev);
	if (ret)
		return ret;

	ret = pinctrl_enable(priv->pctldev);
	if (ret)
		return ret;

	priv->gc = (struct gpio_chip) {
		.owner = THIS_MODULE,
		.label = MAX96719A_GPIOCHIP_NAME,
		.base = -1,
		.ngpio = MAX96719A_GPIO_NUM,
		.parent = dev,
		.can_sleep = true,
		.request = gpiochip_generic_request,
		.free = gpiochip_generic_free,
		.set_config = gpiochip_generic_config,
		.get_direction = max96719a_gpio_get_direction,
		.direction_input = max96719a_gpio_direction_input,
		.direction_output = max96719a_gpio_direction_output,
		.get = max96719a_gpio_get,
		.set = max96719a_gpio_set,
	};

	return devm_gpiochip_add_data(dev, &priv->gc, priv);
}

static int max96719a_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96719a_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &max96719a_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->ser.ops = &max96719a_ops;

	ret = max96719a_wait_for_device(priv);
	if (ret)
		return ret;

	ret = max96719a_gpiochip_probe(priv);
	if (ret)
		return ret;

	ret = max96719a_register_clkout(priv);
	if (ret)
		return ret;

	return max_ser_probe(client, &priv->ser);
}

static void max96719a_remove(struct i2c_client *client)
{
	struct max96719a_priv *priv = i2c_get_clientdata(client);

	max_ser_remove(&priv->ser);
}

static const struct of_device_id max96719a_of_ids[] = {
	{ .compatible = "maxim,max96719a" },
	{ }
};
MODULE_DEVICE_TABLE(of, max96719a_of_ids);

static struct i2c_driver max96719a_i2c_driver = {
	.driver	= {
		.name = MAX96719A_NAME,
		.of_match_table = max96719a_of_ids,
	},
	.probe = max96719a_probe,
	.remove = max96719a_remove,
};

module_i2c_driver(max96719a_i2c_driver);

MODULE_DESCRIPTION("MAX96719A GMSL2 Serializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
