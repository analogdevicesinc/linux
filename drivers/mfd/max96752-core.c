// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core MFD support for Analog Devices MAX96752 deserializer
 *
 * Copyright 2023 NXP
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/mfd/maxim_serdes.h>
#include <linux/mfd/max96752.h>

static const struct mfd_cell max96752_mfd_cells[] = {
	{
		.name = "max96752-lvds",
		.of_compatible = "maxim,max96752-lvds",
	},
};

static bool max96752_writable_register(struct device *dev,
				       unsigned int reg)
{
	switch (reg) {
	case MAX96752_DEV_ID ... MAX96752_DEV_VIDEO_CAP:
	case MAX96752_DEV_PIN_IN_0 ... MAX96752_DEV_PIN_IN_1:
	case MAX96752_TCTRL_PWR0 ... MAX96752_TCTRL_PWR1:
	case MAX96752_TCTRL_CTRL3:
	case MAX96752_TCTRL_INTR3:
	case MAX96752_TCTRL_INTR5:
	case MAX96752_TCTRL_INTR7:
	case MAX96752_CC_I2C_7:
	case MAX96752_CC_BITLEN_LSB:
	case MAX96752_VID_RX8:
	case MAX96752_AUD_TX5:
	case MAX96752_AUD_RX9 ... MAX96752_AUD_RX_INFO_RX4:
	case MAX96752_SPI_7:
	case MAX96752_WM_5:
	case MAX96752_RLMS_7(0):
	case MAX96752_RLMS_7(1):
	case MAX96752_RLMS_EYEMONERRCNTL(0) ... MAX96752_RLMS_EYEMONVALCNTH(0):
	case MAX96752_RLMS_EYEMONERRCNTL(1) ... MAX96752_RLMS_EYEMONVALCNTH(1):
		return false;

	default:
		return true;
	}
}

static bool max96752_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX96752_DEV_REG3:
	case MAX96752_TCTRL_PWR0 ... MAX96752_TCTRL_PWR1:
	case MAX96752_TCTRL_CTRL3:
	case MAX96752_TCTRL_INTR3:
	case MAX96752_TCTRL_INTR5:
	case MAX96752_TCTRL_INTR7:
	case MAX96752_CC_I2C_7:
	case MAX96752_CC_BITLEN_LSB ... MAX96752_CC_UART_2:
	case MAX96752_CC_I2C_PT_2:
	case MAX96752_VID_RX3:
	case MAX96752_VID_RX8:
	case MAX96752_AUD_TX5:
	case MAX96752_AUD_RX9 ... MAX96752_AUD_RX_INFO_RX4:
	case MAX96752_SPI_7:
	case MAX96752_WM_5:
	case MAX96752_VRX_OLDI0:
	case MAX96752_RLMS_7(0):
	case MAX96752_RLMS_37(0) ... MAX96752_RLMS_EYEMONVALCNTH(0):
	case MAX96752_RLMS_7(1):
	case MAX96752_RLMS_37(1) ... MAX96752_RLMS_EYEMONVALCNTH(1):
	case MAX96752_GPIO_A(0):
	case MAX96752_GPIO_A(1):
	case MAX96752_GPIO_A(2):
	case MAX96752_GPIO_A(3):
	case MAX96752_GPIO_A(4):
	case MAX96752_GPIO_A(5):
	case MAX96752_GPIO_A(6):
	case MAX96752_GPIO_A(7):
	case MAX96752_GPIO_A(8):
	case MAX96752_GPIO_A(9):
	case MAX96752_GPIO_A(10):
	case MAX96752_GPIO_A(11):
	case MAX96752_GPIO_A(12):
	case MAX96752_GPIO_A(13):
	case MAX96752_GPIO_A(14):
	case MAX96752_GPIO_A(15):
		return true;

	default:
		return false;
	}
}

const struct regmap_config max96752_regmap_cfg = {
	.name = "max96752",
	.reg_bits = 16,
	.val_bits = 8,

	.max_register = MAX96752_COLOR_C_LUT + COLOR_LUT_SIZE,
	.writeable_reg = max96752_writable_register,
	.volatile_reg = max96752_volatile_reg,

	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(max96752_regmap_cfg);

static int max96752_get_dt_gmsl_links_params(struct max96752 *max96752)
{
	struct device *dev = max96752->dev;
	int ret;
	int val;

	max96752->gmsl2_dual_link = of_property_present(dev->of_node, "maxim,gmsl2-dual-link");

	ret = of_property_read_s32(dev->of_node, "maxim,gmsl2-link-speed", &val);
	if (ret < 0) {
		dev_warn(dev, "GMSL2 link speed not set in DT, using pin configuration\n");

		ret = regmap_read(max96752->regmap, MAX96752_DEV_CH_CTRL, &val);
		if (ret)
			return -EIO;

		max96752->gmsl2_link_speed = val & TX_RATE_MASK;

		return 0;
	}

	if (!ret && val != 3 && val != 6) {
		dev_err(dev, "wrong GMSL speed set in DT\n");
		return -EINVAL;
	}

	max96752->gmsl2_link_speed = val / 3;

	return 0;
}

int max96752_set_link_params(struct max96752 *max96752)
{
	struct device *dev = max96752->dev;
	struct regmap *regmap = max96752->regmap;
	int link_id = max96752->link_id;
	unsigned int reg_val;

	/* setting GMSL2 link speed */
	regmap_update_bits(regmap, MAX96752_DEV_CH_CTRL, RX_RATE_MASK,
			   max96752->gmsl2_link_speed << RX_RATE_SHIFT);

	/* activating dual-link */
	if (max96752->gmsl2_dual_link) {
		regmap_read(regmap, MAX96752_DEV_VIDEO_CAP, &reg_val);
		if (!(reg_val & DUAL_CPBL)) {
			dev_dbg(dev, "configuring dual-link mode\n");

			/* Set link configuration. */
			regmap_update_bits(regmap, MAX96752_TCTRL_CTRL0,
					   LINK_CFG_MASK | AUTO_LINK, LINK_CFG_DUAL_LINK);
		}
	}

	/* set the TX_SRC_ID for all channels */
	regmap_update_bits(regmap, MAX96752_CFGL_AUDIO_TR3, AUD_TX_SRC_ID_MASK, link_id);
	regmap_update_bits(regmap, MAX96752_CFGI_INFOFR_TR3, INFOFR_TX_SRC_ID_MASK, link_id);
	regmap_update_bits(regmap, MAX96752_CFGL_SPI_TR3, SPI_TX_SRC_ID_MASK, link_id);
	regmap_update_bits(regmap, MAX96752_CFGC_CC_TR3, CC_TX_SRC_ID_MASK, link_id);
	regmap_update_bits(regmap, MAX96752_CFGL_GPIO_TR3, GPIO_TX_SRC_ID_MASK, link_id);
	regmap_update_bits(regmap, MAX96752_CFGL_IIC_TR3(0), PT_TX_SRC_ID_MASK, link_id);
	regmap_update_bits(regmap, MAX96752_CFGL_IIC_TR3(1), PT_TX_SRC_ID_MASK, link_id);

	return 0;
}

static void max96752_set_gpio_pull_resistor(struct max96752 *max96752, unsigned int gpio_pin,
					    unsigned int pull_type, unsigned int pull_strength)
{
	regmap_update_bits(max96752->regmap, MAX96752_GPIO_B(gpio_pin), PULL_UPDN_SEL_MASK,
			   pull_type << PULL_UPDN_SEL_SHIFT);
	regmap_update_bits(max96752->regmap, MAX96752_GPIO_A(gpio_pin), RES_CFG,
			   pull_strength ? RES_CFG : 0);
}

static void max96752_set_gpio_direction(struct max96752 *max96752, unsigned int gpio_pin,
					unsigned int direction)
{
	regmap_update_bits(max96752->regmap, MAX96752_GPIO_A(gpio_pin), GPIO_OUT_DIS, direction);
}

static void max96752_set_gpio_gmsl_tx(struct max96752 *max96752, unsigned int gpio_pin, bool en)
{
	regmap_update_bits(max96752->regmap, MAX96752_GPIO_A(gpio_pin), GPIO_TX_EN,
			   en ? GPIO_TX_EN : 0);
}

static void max96752_set_gpio_gmsl_rx(struct max96752 *max96752, unsigned int gpio_pin, bool en)
{
	regmap_update_bits(max96752->regmap, MAX96752_GPIO_A(gpio_pin), GPIO_RX_EN,
			   en ? GPIO_RX_EN : 0);
}

static void max96752_set_gpio_rx_id(struct max96752 *max96752, unsigned int local_gpio_pin,
				    unsigned int remote_gpio_pin)
{
	regmap_update_bits(max96752->regmap, MAX96752_GPIO_C(local_gpio_pin), GPIO_RX_ID_MASK,
			   remote_gpio_pin << GPIO_RX_ID_SHIFT);
}

static void max96752_config_standalone_gpio(struct max96752 *max96752, unsigned int gpio_pin,
					    unsigned int *pin_configs)
{
	struct device *dev = max96752->dev;

	dev_dbg(dev, "config standalone gpio %d: dir = %d, pull_type = %d, strength = %d\n",
		gpio_pin, pin_configs[0], pin_configs[1], pin_configs[2]);

	max96752_set_gpio_direction(max96752, gpio_pin, pin_configs[0]);
	max96752_set_gpio_pull_resistor(max96752, gpio_pin, pin_configs[1], pin_configs[2]);
	max96752_set_gpio_gmsl_tx(max96752, gpio_pin, false);
	max96752_set_gpio_gmsl_rx(max96752, gpio_pin, false);
}

static void max96752_config_tunneled_gpio(struct max96752 *max96752, unsigned int local_gpio_pin,
					  unsigned int *pin_configs, unsigned int remote_gpio_pin)
{
	struct device *dev = max96752->dev;

	dev_dbg(dev, "config tunneled gpio %d: dir = %d, pull_type = %d, strength = %d\n",
		local_gpio_pin, pin_configs[0], pin_configs[1], pin_configs[2]);

	max96752_set_gpio_direction(max96752, local_gpio_pin, pin_configs[0]);
	max96752_set_gpio_pull_resistor(max96752, local_gpio_pin, pin_configs[1], pin_configs[2]);
	max96752_set_gpio_gmsl_tx(max96752, local_gpio_pin, pin_configs[0] ? true : false);
	max96752_set_gpio_gmsl_rx(max96752, local_gpio_pin, pin_configs[0] ? false : true);

	if (!pin_configs[0])
		max96752_set_gpio_rx_id(max96752, local_gpio_pin, remote_gpio_pin);
}

static int max96752_set_gpio_pin(struct max96752 *max96752, struct device_node *gpios_node,
				 struct device_node *pin_node)
{
	struct device *dev = max96752->dev;
	unsigned int local_gpio_pin, remote_gpio_pin;
	unsigned int pin_configs[3];
	int ret;
	struct device_node *ep, *remote_pin_node;

	ret = of_property_read_u32(pin_node, "reg", &local_gpio_pin);
	if (ret)
		return ret;

	ret = of_property_read_u32_array(pin_node, "maxim,pin_config", pin_configs, 3);
	if (ret) {
		dev_err(dev, "Bad gpio pin configuration.\n");
		return ret;
	}

	ep = of_graph_get_endpoint_by_regs(gpios_node, local_gpio_pin, 0);
	if (!ep) {
		max96752_config_standalone_gpio(max96752, local_gpio_pin, pin_configs);
		return 0;
	}

	remote_pin_node = of_graph_get_remote_port(ep);
	if (!remote_pin_node) {
		dev_err(dev, "Couldn't find corresponding remote gpio pin\n");
		of_node_put(ep);
		return -ENODEV;
	}

	of_node_put(ep);

	ret = of_property_read_u32(remote_pin_node, "reg", &remote_gpio_pin);
	if (ret) {
		of_node_put(remote_pin_node);
		return ret;
	}

	of_node_put(remote_pin_node);

	max96752_config_tunneled_gpio(max96752, local_gpio_pin, pin_configs, remote_gpio_pin);

	return 0;
}

static int max96752_set_gpios(struct max96752 *max96752)
{
	struct device *dev = max96752->dev;
	struct device_node *dev_node = dev->of_node;
	struct device_node *gpios_node, *child;
	int ret;

	/* look for a gpio-config node that contains gpios' pin configurations */
	gpios_node = of_get_child_by_name(dev_node, "gpio-config");
	if (!gpios_node) {
		dev_info(dev, "No gpios DT node found, using GPIOs defaults.\n");
		return 0;
	}

	for_each_child_of_node(gpios_node, child) {
		ret = max96752_set_gpio_pin(max96752, gpios_node, child);
		if (ret) {
			of_node_put(gpios_node);
			of_node_put(child);
			return ret;
		}
	}

	of_node_put(gpios_node);

	return 0;
}

int max96752_dev_check(struct max96752 *max96752, ulong expected_dev_id)
{
	struct device *dev = max96752->dev;
	int ret;

	ret = regmap_read(max96752->regmap, MAX96752_DEV_ID, &max96752->id);
	if (ret) {
		dev_err(dev, "Failed to read device id: %d\n", ret);
		return ret;
	}

	if (max96752->id != expected_dev_id) {
		dev_err(dev, "Wrong Maxim deserializer detected: id 0x%x\n",
			max96752->id);
		return -ENODEV;
	}

	ret = regmap_read(max96752->regmap, MAX96752_DEV_REV, &max96752->rev);
	if (ret) {
		dev_err(dev, "Failed to read device revision: %d\n", ret);
		return ret;
	}

	max96752->rev &= DEV_REV_MASK;

	dev_info(dev, "Found Maxim deserializer with id 0x%x and rev 0x%x\n",
		 max96752->id, max96752->rev);

	return 0;
}
EXPORT_SYMBOL(max96752_dev_check);

int max96752_dev_init(struct max96752 *max96752)
{
	struct device *dev = max96752->dev;
	int ret;

	dev_set_drvdata(dev, max96752);

	ret = max96752_get_dt_gmsl_links_params(max96752);
	if (ret)
		return ret;

	ret = max96752_set_link_params(max96752);
	if (ret)
		return ret;

	/* disable audio receiver by default */
	regmap_update_bits(max96752->regmap, MAX96752_AUD_RX1, AUD_EN_RX, 0);

	ret = max96752_set_gpios(max96752);
	if (ret)
		return ret;

	return devm_mfd_add_devices(max96752->dev, PLATFORM_DEVID_AUTO, max96752_mfd_cells,
				    ARRAY_SIZE(max96752_mfd_cells),
				    NULL, 0, NULL);
}
EXPORT_SYMBOL_GPL(max96752_dev_init);

MODULE_DESCRIPTION("MAX96752 core MFD driver");
MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@oss.nxp.com>");
MODULE_LICENSE("GPL v2");
