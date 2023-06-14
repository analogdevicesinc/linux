// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core MFD support for Analog Devices MAX96789 MIPI-DSI serializer
 *
 * Copyright 2023 NXP
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/mfd/maxim_serdes.h>
#include <linux/mfd/max96789.h>

static const struct mfd_cell max96789_mfd_cells[] = {
	{
		.name = "max96789-dsi0",
		.of_compatible = "maxim,max96789-dsi",
		.of_reg = 0,
		.use_of_reg = true,
	},
	{
		.name = "max96789-dsi1",
		.of_compatible = "maxim,max96789-dsi",
		.of_reg = 1,
		.use_of_reg = true,
	},
};

static bool max96789_writable_register(struct device *dev,
				       unsigned int reg)
{
	switch (reg) {
	case MAX96789_DEV_ID ... MAX96789_DEV_VIDEO_CAP:
	case MAX96789_DEV_LF_STATUS1 ... MAX96789_DEV_LF_STATUS2:
	case MAX96789_TCTRL_PWR0:
	case MAX96789_TCTRL_CTRL3:
	case MAX96789_TCTRL_INTR3:
	case MAX96789_TCTRL_INTR5:
	case MAX96789_TCTRL_INTR7:
	case MAX96789_CC_I2C_7:
	case MAX96789_CC_BITLEN_LSB:
	case MAX96789_AUD_TX_AUDIO_TX5(0):
	case MAX96789_AUD_TX_AUDIO_TX5(1):
	case MAX96789_AUD_RX_AUDIO_RX9(0):
	case MAX96789_AUD_RX_AUDIO_RX9(1):
	case MAX96789_SPI_7:
	case MAX96789_WM_5:
	case MAX96789_DUALVIEW_DV_PPL_L(0) ... MAX96789_DUALVIEW_DV2(0):
	case MAX96789_DUALVIEW_DV_PPL_L(1) ... MAX96789_DUALVIEW_DV2(1):
	case MAX96789_GMSL1_14(0):
	case MAX96789_GMSL1_14(1):
	case MAX96789_GMSL1_16(0):
	case MAX96789_GMSL1_16(1):
	case MAX96789_GMSL1_CC_I2C_RETR_CNT(0) ... MAX96789_GMSL1_CC_CRC_ERRCNT(0):
	case MAX96789_GMSL1_CC_I2C_RETR_CNT(1) ... MAX96789_GMSL1_CC_CRC_ERRCNT(1):
	case MAX96789_GMSL1_C8(0):
	case MAX96789_GMSL1_C8(1):
	case MAX96789_MISC_HS_VS_X ... MAX96789_MISC_HS_VS_U:
	case MAX96789_ASYM_DV_ASYM27:
	case MAX96789_RLMS_7(0):
	case MAX96789_RLMS_7(1):
	case MAX96789_RLMS_EYEMONERRCNTL(0) ... MAX96789_RLMS_EYEMONVALCNTH(0):
	case MAX96789_RLMS_EYEMONERRCNTL(1) ... MAX96789_RLMS_EYEMONVALCNTH(1):
		return false;

	default:
		return true;
	}
}

static bool max96789_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX96789_TCTRL_CTRL3:
	case MAX96789_TCTRL_INTR3:
	case MAX96789_TCTRL_INTR5:
	case MAX96789_TCTRL_INTR7:
	case MAX96789_MIPI_DSI_CTRL_STATUS(DSI_PORT_A):
	case MAX96789_MIPI_DSI_CTRL_STATUS(DSI_PORT_B):
	case MAX96789_VID_TX_VIDEO_TX2(VIDEO_PIPE_X):
	case MAX96789_VID_TX_VIDEO_TX2(VIDEO_PIPE_Y):
	case MAX96789_VID_TX_VIDEO_TX2(VIDEO_PIPE_Z):
	case MAX96789_VID_TX_VIDEO_TX2(VIDEO_PIPE_U):
	case MAX96789_MISC_HS_VS_X:
	case MAX96789_MISC_HS_VS_Y:
	case MAX96789_MISC_HS_VS_Z:
	case MAX96789_MISC_HS_VS_U:
	case MAX96789_MIPI_RX_PHY0_LP_ERR:
	case MAX96789_MIPI_RX_PHY0_HS_ERR:
	case MAX96789_MIPI_RX_PHY1_LP_ERR:
	case MAX96789_MIPI_RX_PHY1_HS_ERR:
	case MAX96789_MIPI_RX_PHY2_LP_ERR:
	case MAX96789_MIPI_RX_PHY2_HS_ERR:
	case MAX96789_MIPI_RX_PHY3_LP_ERR:
	case MAX96789_MIPI_RX_PHY3_HS_ERR:
	case MAX96789_GPIO_A(0):
	case MAX96789_GPIO_A(1):
	case MAX96789_GPIO_A(2):
	case MAX96789_GPIO_A(3):
	case MAX96789_GPIO_A(4):
	case MAX96789_GPIO_A(5):
	case MAX96789_GPIO_A(6):
	case MAX96789_GPIO_A(7):
	case MAX96789_GPIO_A(8):
	case MAX96789_GPIO_A(9):
	case MAX96789_GPIO_A(10):
	case MAX96789_GPIO_A(11):
	case MAX96789_GPIO_A(12):
	case MAX96789_GPIO_A(13):
	case MAX96789_GPIO_A(14):
	case MAX96789_GPIO_A(15):
	case MAX96789_GPIO_A(16):
	case MAX96789_GPIO_A(17):
	case MAX96789_GPIO_A(18):
	case MAX96789_GPIO_A(19):
	case MAX96789_GPIO_A(20):
		return true;

	default:
		return false;
	}
}

const struct regmap_config max96789_regmap_cfg = {
	.name = "max96789",
	.reg_bits = 16,
	.val_bits = 8,

	.max_register = MAX96789_DPLL_AUD_SSPLL_DPLL_3(1),
	.writeable_reg = max96789_writable_register,
	.volatile_reg = max96789_volatile_reg,

	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(max96789_regmap_cfg);

static int max96789_get_dt_gmsl_links_params(struct max96789 *max96789)
{
	struct device *dev = max96789->dev;
	unsigned int link_types[GMSL_MAX_LINKS];
	int ret;
	int val;

	ret = of_property_read_u32_array(dev->of_node, "maxim,gmsl-links-types",
					 link_types, GMSL_MAX_LINKS);
	if (ret < 0) {
		dev_warn(dev, "GMSL links types not set in DT, defaulting to pin configuration\n");

		ret = regmap_read(max96789->regmap, MAX96789_DEV_LINK, &val);
		if (ret)
			return -EIO;

		max96789->gmsl_links_types[GMSL_LINK_A] = !!(val & GMSL2_A);
		max96789->gmsl_links_types[GMSL_LINK_B] = !!(val & GMSL2_B);
	} else {
		max96789->gmsl_links_types[GMSL_LINK_A] = (link_types[GMSL_LINK_A] & 0x3) - 1;
		max96789->gmsl_links_types[GMSL_LINK_B] = (link_types[GMSL_LINK_B] & 0x3) - 1;
	}

	max96789->gmsl2_dual_link = of_property_present(dev->of_node, "maxim,gmsl2-dual-link");
	if (max96789->gmsl2_dual_link &&
	    (max96789->gmsl_links_types[GMSL_LINK_A] == LINK_TYPE_GMSL1 ||
	     max96789->gmsl_links_types[GMSL_LINK_B] == LINK_TYPE_GMSL1)) {
		dev_warn(dev, "ignoring dual-link DT setting because at least one link is GMSL1\n");

		max96789->gmsl2_dual_link = false;
	}

	ret = of_property_read_s32(dev->of_node, "maxim,gmsl2-link-speed", &val);
	if (ret < 0) {
		dev_warn(dev, "GMSL2 link speed not set in DT, using pin configuration\n");

		ret = regmap_read(max96789->regmap, MAX96789_DEV_CH_CTRL, &val);
		if (ret)
			return -EIO;

		max96789->gmsl2_link_speed = val & TX_RATE_MASK;

		return 0;
	}

	if (!ret && val != 3 && val != 6) {
		dev_err(dev, "wrong GMSL speed set in DT\n");
		return -EINVAL;
	}

	max96789->gmsl2_link_speed = val / 3;

	return 0;
}

static int max96789_set_link_params(struct max96789 *max96789)
{
	struct device *dev = max96789->dev;
	unsigned int reg_val;

	max96789->gmsl_links_need_reset = true;

	/* setting link types */
	reg_val = max96789->gmsl_links_types[GMSL_LINK_A] == LINK_TYPE_GMSL2 ? GMSL2_A : 0;
	reg_val |= max96789->gmsl_links_types[GMSL_LINK_B] == LINK_TYPE_GMSL2 ? GMSL2_B : 0;
	regmap_update_bits(max96789->regmap, MAX96789_DEV_LINK, GMSL2_A | GMSL2_B, reg_val);

	/* activating dual-link */
	if (!max96789->gmsl2_dual_link)
		return 0;

	regmap_read(max96789->regmap, MAX96789_DEV_VIDEO_CAP, &reg_val);
	if (!(reg_val & DUAL_CPBL_N)) {
		dev_dbg(dev, "configuring dual-link mode\n");

		reg_val = LINK_EN_A | LINK_EN_B | GMSL2_A | GMSL2_B;
		regmap_update_bits(max96789->regmap, MAX96789_DEV_LINK, reg_val, reg_val);

		/*
		 * Set link configuration but do not reset the link just yet, this will be
		 * done by the remote deserializer device once it probes.
		 */
		regmap_update_bits(max96789->regmap, MAX96789_TCTRL_CTRL0,
				   LINK_CFG_MASK | AUTO_LINK, LINK_CFG_DUAL_LINK);
	}

	return 0;
}

static void max96789_set_gpio_pull_resistor(struct max96789 *max96789, unsigned int gpio_pin,
					    unsigned int pull_type, unsigned int pull_strength)
{
	regmap_update_bits(max96789->regmap, MAX96789_GPIO_B(gpio_pin), PULL_UPDN_SEL_MASK,
			   pull_type << PULL_UPDN_SEL_SHIFT);
	regmap_update_bits(max96789->regmap, MAX96789_GPIO_A(gpio_pin), RES_CFG,
			   pull_strength ? RES_CFG : 0);
}

static void max96789_set_gpio_direction(struct max96789 *max96789, unsigned int gpio_pin,
					unsigned int direction)
{
	regmap_update_bits(max96789->regmap, MAX96789_GPIO_A(gpio_pin), GPIO_OUT_DIS, direction);
}

static void max96789_set_gpio_gmsl_tx(struct max96789 *max96789, unsigned int gpio_pin, bool en)
{
	regmap_update_bits(max96789->regmap, MAX96789_GPIO_A(gpio_pin), GPIO_TX_EN,
			   en ? GPIO_TX_EN : 0);
}

static void max96789_set_gpio_gmsl_rx(struct max96789 *max96789, unsigned int gpio_pin, bool en)
{
	regmap_update_bits(max96789->regmap, MAX96789_GPIO_A(gpio_pin), GPIO_RX_EN,
			   en ? GPIO_RX_EN : 0);
}

static void max96789_set_gpio_rx_id(struct max96789 *max96789, unsigned int local_gpio_pin,
				    unsigned int remote_gpio_pin)
{
	regmap_update_bits(max96789->regmap, MAX96789_GPIO_C(local_gpio_pin), GPIO_RX_ID_MASK,
			   remote_gpio_pin << GPIO_RX_ID_SHIFT);
}

static void max96789_config_standalone_gpio(struct max96789 *max96789, unsigned int gpio_pin,
					    unsigned int *pin_configs)
{
	struct device *dev = max96789->dev;

	dev_dbg(dev, "config standalone gpio %d: dir = %d, pull_type = %d, strength = %d\n",
		gpio_pin, pin_configs[0], pin_configs[1], pin_configs[2]);

	max96789_set_gpio_direction(max96789, gpio_pin, pin_configs[0]);
	max96789_set_gpio_pull_resistor(max96789, gpio_pin, pin_configs[1], pin_configs[2]);
	max96789_set_gpio_gmsl_tx(max96789, gpio_pin, false);
	max96789_set_gpio_gmsl_rx(max96789, gpio_pin, false);
}

static void max96789_config_tunneled_gpio(struct max96789 *max96789, unsigned int local_gpio_pin,
					  unsigned int *pin_configs, unsigned int remote_gpio_pin)
{
	struct device *dev = max96789->dev;

	dev_dbg(dev, "config tunneled gpio %d: dir = %d, pull_type = %d, strength = %d\n",
		local_gpio_pin, pin_configs[0], pin_configs[1], pin_configs[2]);

	max96789_set_gpio_direction(max96789, local_gpio_pin, pin_configs[0]);
	max96789_set_gpio_pull_resistor(max96789, local_gpio_pin, pin_configs[1], pin_configs[2]);
	max96789_set_gpio_gmsl_tx(max96789, local_gpio_pin, pin_configs[0] ? true : false);
	max96789_set_gpio_gmsl_rx(max96789, local_gpio_pin, pin_configs[0] ? false : true);

	if (!pin_configs[0])
		max96789_set_gpio_rx_id(max96789, local_gpio_pin, remote_gpio_pin);
}

static int max96789_set_gpio_pin(struct max96789 *max96789, struct device_node *gpios_node,
				 struct device_node *pin_node)
{
	struct device *dev = max96789->dev;
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
		max96789_config_standalone_gpio(max96789, local_gpio_pin, pin_configs);
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

	max96789_config_tunneled_gpio(max96789, local_gpio_pin, pin_configs, remote_gpio_pin);

	return 0;
}

static int max96789_set_gpios(struct max96789 *max96789)
{
	struct device *dev = max96789->dev;
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
		ret = max96789_set_gpio_pin(max96789, gpios_node, child);
		if (ret) {
			of_node_put(gpios_node);
			of_node_put(child);
			return ret;
		}
	}

	of_node_put(gpios_node);
	return 0;
}

static void max96789_finish_link_setup(void *data)
{
	struct max96789 *max96789 = data;
	int ret;
	unsigned int reg_val;

	if (max96789->gmsl_links_types[GMSL_LINK_A] == LINK_TYPE_GMSL2 ||
	    max96789->gmsl_links_types[GMSL_LINK_B] == LINK_TYPE_GMSL2)
		regmap_update_bits(max96789->regmap, MAX96789_DEV_CH_CTRL, TX_RATE_MASK,
				   max96789->gmsl2_link_speed << TX_RATE_SHIFT);

	if (max96789->gmsl_links_used == 2 || max96789->gmsl2_dual_link)
		regmap_update_bits(max96789->regmap, MAX96789_DEV_LINK,
				   LINK_EN_B | LINK_EN_A, LINK_EN_B | LINK_EN_A);

	if (max96789->gmsl_links_used == 2)
		regmap_update_bits(max96789->regmap, MAX96789_TCTRL_CTRL0,
				   LINK_CFG_MASK | AUTO_LINK | RESET_ONESHOT,
				   LINK_CFG_SPLITTER | RESET_ONESHOT);
	else if (max96789->gmsl2_dual_link)
		regmap_update_bits(max96789->regmap, MAX96789_TCTRL_CTRL0,
				   LINK_CFG_MASK | AUTO_LINK | RESET_ONESHOT,
				   LINK_CFG_DUAL_LINK | RESET_ONESHOT);

	ret = regmap_read_poll_timeout(max96789->regmap, MAX96789_TCTRL_INTR7, reg_val,
				       reg_val & (LOCK_B | LOCK_A), 500, 300000);
	if (ret < 0) {
		dev_err(max96789->dev, "%s: GMSL links not locked, MAX96789_TCTRL_INTR7 = 0x%02x\n",
			__func__, reg_val);
		return;
	}

	max96789->link_setup_finished = true;
}

int max96789_dev_init(struct max96789 *max96789, ulong expected_dev_id)
{
	struct device *dev = max96789->dev;
	int ret;
	unsigned int reg_val;

	dev_set_drvdata(dev, max96789);

	ret = devm_regulator_get_enable_optional(dev, "v12p0");
	if (ret < 0 && ret != -ENODEV)
		return ret;

	/* power-up completes in approx 2ms, according to specifications */
	usleep_range(2000, 2500);

	ret = regmap_read(max96789->regmap, MAX96789_DEV_ID, &max96789->id);
	if (ret) {
		dev_err(dev, "Failed to read device id: %d\n", ret);
		return ret;
	}

	if (max96789->id != expected_dev_id) {
		dev_err(dev, "Wrong Maxim serializer detected: id 0x%x\n",
			max96789->id);
		return -ENODEV;
	}

	ret = regmap_read(max96789->regmap, MAX96789_DEV_REV, &max96789->rev);
	if (ret) {
		dev_err(dev, "Failed to read device revision: %d\n", ret);
		return ret;
	}

	max96789->rev &= DEV_REV_MASK;

	dev_info(dev, "Found Maxim serializer with id 0x%x and rev 0x%x\n",
		 max96789->id, max96789->rev);

	/* Documentation states that:
	 *
	 * "The following register writes must be made to ensure proper
	 * serializer operation. Without these writes, the operation of the
	 * device as specified in the datasheet cannot be guaranteed."
	 *
	 * However, there's no further documentation of that register or its
	 * fields. So, the next command will use hard-coded values.
	 */
	ret = regmap_update_bits(max96789->regmap, 0x302, 0x07, 0x10);
	if (ret) {
		dev_err(dev, "Cannot increase voltage to clock system: %d\n", ret);
		return ret;
	}

	ret = max96789_get_dt_gmsl_links_params(max96789);
	if (ret)
		return ret;

	/* wait for at least one GMSL link to lock with default settings*/
	ret = regmap_read_poll_timeout(max96789->regmap, MAX96789_TCTRL_CTRL3, reg_val,
				       reg_val & LOCKED, 500, 300000);
	if (ret < 0) {
		dev_err(max96789->dev, "%s: no GMSL link locked, MAX96789_TCTRL_CTRL3 = 0x%02x\n",
			__func__, reg_val);
		return -ENODEV;
	}

	ret = max96789_set_link_params(max96789);
	if (ret)
		return ret;

	ret = max96789_set_gpios(max96789);
	if (ret)
		return ret;

	maxim_serdes_chain_register_local(dev, max96789->gmsl_links_used,
					  max96789_finish_link_setup, max96789);

	devm_mfd_add_devices(max96789->dev, PLATFORM_DEVID_AUTO, max96789_mfd_cells,
			     ARRAY_SIZE(max96789_mfd_cells),
			     NULL, 0, NULL);
	return 0;
}
EXPORT_SYMBOL_GPL(max96789_dev_init);

MODULE_DESCRIPTION("MAX96789 core MFD driver");
MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@oss.nxp.com>");
MODULE_LICENSE("GPL v2");
