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

#define MAX96719A_SRC_ADDR_0(x)			(0x351a + (x) * 0x2)
#define MAX96719A_SRC_ADDR_0_SRC_ADDR		GENMASK(7, 1)

#define MAX96719A_DST_ADDR_0(x)			(0x351b + (x) * 0x2)
#define MAX96719A_DST_ADDR_0_DST_ADDR		GENMASK(7, 1)

#define MAX96719A_MIPI_RX0			0xa005
#define MAX96719A_MIPI_RX0_LANE_COUNT		GENMASK(6, 5)

#define MAX96719A_DEFAULT_CLKOUT_RATE		25000000UL

#define MAX96719A_STREAM_ID			2

#define MAX96719A_NAME				"max96719a"

struct max96719a_priv {
	struct max_ser ser;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

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
