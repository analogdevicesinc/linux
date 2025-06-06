// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX96726A Quad GMSL2 Deserializer Driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include "max_des.h"

#define MAX96726A_REG0				0x0

#define MAX96726A_REG7				0x7
#define MAX96726A_REG7_LINK_EN			GENMASK(3, 0)
#define MAX96726A_REG7_GMSL3_X(x)		BIT((x) + 4)

#define MAX96726A_REG16				0x10
#define MAX96726A_REG16_RX_RATE(x)		(GENMASK(1, 0) << ((x) * 2))
#define MAX96726A_REG16_RX_RATE_3Gbps		0b01
#define MAX96726A_REG16_RX_RATE_6Gbps		0b10
#define MAX96726A_REG16_RX_RATE_12Gbps		0b11

#define MAX96726A_PWR6				0x19
#define MAX96726A_PWR6_RESET_ALL		BIT(6)

#define MAX96726A_CTRL1				0x1e
#define MAX96726A_CTRL1_RESET_ONESHOT		GENMASK(3, 0)

#define MAX96726A_VIDEO_PIPE_SEL(p)		(0xf0 + (p) / 2)
#define MAX96726A_VIDEO_PIPE_SEL_STREAM(p)	(GENMASK(1, 0) << (4 * ((p) % 2)))

#define MAX96726A_VIDEO_PIPE_EN			0xf4
#define MAX96726A_VIDEO_PIPE_EN_MASK(p)		BIT(p)

#define MAX96726A_STREAM_SEL_ALL		0xf5
#define MAX96726A_STREAM_SEL_ALL_MASK		BIT(0)

#define MAX96726A_VPRBS_FLAGS(p)		(0x110 + (p) * 0x1c)
#define MAX96726A_VPRBS_FLAGS_VIDEO_LOCK	BIT(0)

#define MAX96726A_CONFIG_0(p)			(0x420 + (p) * 0x18)
#define MAX96726A_CONFIG_0_CON_DEST		GENMASK(1, 0)

#define MAX96726A_VC_REMAP_MODES(p)		(0x426 + (p) * 0x18)
#define MAX96726A_OVERRIDE_VC_SRC_DST_MODE	BIT(5)
#define MAX96726A_OVERRIDE_VC_PIPE_ID_MODE	BIT(6)
#define MAX96726A_OVERRIDE_VC_LINK_ID_MODE	BIT(7)

#define MAX96726A_VC_REMAP_0_SRC(p, x)		(0x427 + (p) * 0x18 + (x) * 0x2)
#define MAX96726A_VC_REMAP_0_SRC_MASK		GENMASK(4, 0)

#define MAX96726A_VC_REMAP_0_DST(p, x)		(0x428 + (p) * 0x18 + (x) * 0x2)
#define MAX96726A_VC_REMAP_0_DST_MASK		GENMASK(4, 0)

#define MAX96726A_MIPI_PHY0			0x8b0
#define MAX96726A_MIPI_PHY0_PHY_STDBY_N(x)	(GENMASK(1, 0) << ((x) * 2))

#define MAX96726A_MIPI_PHY49			0x8c4
#define MAX96726A_MIPI_PHY49_CSI2_TX_PKT_CNT(x)	(GENMASK(3, 0) << (4 * ((x) == 0 ? 1 : 0)))

#define MAX96726A_MIPI_PHY50(x)			(0x8c5 + (x))

#define MAX96726A_MIPI_DPLL_CONFIG_5(x)		(0x98f + (x) * 0x3)
#define MAX96726A_DPLL_PREDEF_FREQ		GENMASK(5, 0)
#define MAX96726A_DPLL_PREDEF_FREQ_EN		BIT(7)

#define MAX96726A_MIPI_TX3(x)			(0xa02 + (x) * 0x40)
#define MAX96726A_MIPI_TX3_DESKEW_INIT_2X32K	FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX96726A_MIPI_TX3_DESKEW_INIT_AUTO	BIT(7)

#define MAX96726A_MIPI_TX4(x)			(0xa03 + (x) * 0x40)
#define MAX96726A_MIPI_TX4_DESKEW_PER_2K	FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX96726A_MIPI_TX4_DESKEW_PER_AUTO	BIT(7)

#define MAX96726A_MIPI_TX10(x)			(0xa06 + (x) * 0x40)
#define MAX96726A_MIPI_TX10_CSI2_LANE_CNT	GENMASK(7, 6)
#define MAX96726A_MIPI_TX10_CSI2_CPHY_EN	BIT(5)

#define MAX96726A_FEC_RX_CTRL(x)		(0x10b0 + (x) * 0x20)
#define MAX96726A_FEC_RX_CTRL_RX_FEC_EN		BIT(0)

#define field_get(mask, val) (((val) & (mask)) >> __ffs(mask))
#define field_prep(mask, val) (((val) << __ffs(mask)) & (mask))

#define MAX96726A_PHYS_NUM		2

static const struct regmap_config max96726a_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x13a9,
};

struct max96726a_priv {
	struct max_des des;
	const struct max96726a_chip_info *info;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct gpio_desc *gpiod_pwdn;
};

struct max96726a_chip_info {
	unsigned int versions;
};

#define des_to_priv(_des) \
	container_of(_des, struct max96726a_priv, des)

static int max96726a_wait_for_device(struct max96726a_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 10; i++) {
		unsigned int val;

		ret = regmap_read(priv->regmap, MAX96726A_REG0, &val);
		if (!ret && val)
			return 0;

		msleep(100);

		dev_err(priv->dev, "Retry %u waiting for deserializer: %d\n", i, ret);
	}

	return ret;
}

static int max96726a_reset(struct max96726a_priv *priv)
{
	int ret;

	ret = max96726a_wait_for_device(priv);
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, MAX96726A_PWR6,
			      MAX96726A_PWR6_RESET_ALL);
	if (ret)
		return ret;

	msleep(100);

	ret = max96726a_wait_for_device(priv);
	if (ret)
		return ret;

	return 0;
}

static int max96726a_reg_read(struct max_des *des, unsigned int reg,
			      unsigned int *val)
{
	struct max96726a_priv *priv = des_to_priv(des);

	return regmap_read(priv->regmap, reg, val);
}

static int max96726a_reg_write(struct max_des *des, unsigned int reg,
			       unsigned int val)
{
	struct max96726a_priv *priv = des_to_priv(des);

	return regmap_write(priv->regmap, reg, val);
}

static int max9626a_log_pipe_status(struct max_des *des,
				    struct max_des_pipe *pipe, const char *name)
{
	struct max96726a_priv *priv = des_to_priv(des);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, MAX96726A_VPRBS_FLAGS(pipe->index), &val);
	if (ret)
		return ret;

	pr_info("%s: \tvideo_lock: %u\n", name,
		!!(val & MAX96726A_VPRBS_FLAGS_VIDEO_LOCK));

	return 0;
}

static int max96726a_log_phy_status(struct max_des *des,
				    struct max_des_phy *phy, const char *name)
{
	struct max96726a_priv *priv = des_to_priv(des);
	unsigned int index = phy->index;
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, MAX96726A_MIPI_PHY49, &val);
	if (ret)
		return ret;

	pr_info("%s: \tcsi2_pkt_cnt: %lu\n", name,
		field_get(MAX96726A_MIPI_PHY49_CSI2_TX_PKT_CNT(index), val));

	ret = regmap_read(priv->regmap, MAX96726A_MIPI_PHY50(index), &val);
	if (ret)
		return ret;

	pr_info("%s: \tphy_pkt_cnt: %u\n", name, val);

	return 0;
}

static int max96726a_init(struct max_des *des)
{
	struct max96726a_priv *priv = des_to_priv(des);
	int ret;

	/* Enable stream autoselect. */
	ret = regmap_set_bits(priv->regmap, MAX96726A_STREAM_SEL_ALL,
			      MAX96726A_STREAM_SEL_ALL_MASK);
	if (ret)
		return ret;

	return 0;
}

static int max96726a_init_phy(struct max_des *des, struct max_des_phy *phy)
{
	struct max96726a_priv *priv = des_to_priv(des);
	bool is_cphy = phy->bus_type == V4L2_MBUS_CSI2_CPHY;
	unsigned int num_data_lanes = phy->mipi.num_data_lanes;
	unsigned int dpll_freq = phy->link_frequency * 2;
	unsigned int num_hw_data_lanes;
	unsigned int index = phy->index;
	unsigned int i;
	int ret;

	num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

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

	/* Configure a lane count. */
	ret = regmap_update_bits(priv->regmap, MAX96726A_MIPI_TX10(index),
				 MAX96726A_MIPI_TX10_CSI2_LANE_CNT,
				 FIELD_PREP(MAX96726A_MIPI_TX10_CSI2_LANE_CNT,
					    num_data_lanes - 1));
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96726A_MIPI_TX10(index),
				 MAX96726A_MIPI_TX10_CSI2_CPHY_EN, is_cphy);
	if (ret)
		return ret;

	/* Set DPLL frequency. */
	ret = regmap_update_bits(priv->regmap, MAX96726A_MIPI_DPLL_CONFIG_5(index),
				 MAX96726A_DPLL_PREDEF_FREQ,
				 FIELD_PREP(MAX96726A_DPLL_PREDEF_FREQ,
					    div_u64(dpll_freq, 100000000)));
	if (ret)
		return ret;

	/* Enable DPLL frequency. */
	ret = regmap_clear_bits(priv->regmap, MAX96726A_MIPI_DPLL_CONFIG_5(index),
				MAX96726A_DPLL_PREDEF_FREQ_EN);
	if (ret)
		return ret;

	if (dpll_freq > 1500000000ull) {
		/* Enable initial deskew with 2 x 32k UI. */
		ret = regmap_write(priv->regmap, MAX96726A_MIPI_TX3(index),
				   MAX96726A_MIPI_TX3_DESKEW_INIT_AUTO |
				   MAX96726A_MIPI_TX3_DESKEW_INIT_2X32K);
		if (ret)
			return ret;

		/* Enable periodic deskew with 2 x 1k UI.. */
		ret = regmap_write(priv->regmap, MAX96726A_MIPI_TX4(index),
				   MAX96726A_MIPI_TX4_DESKEW_PER_AUTO |
				   MAX96726A_MIPI_TX4_DESKEW_PER_2K);
		if (ret)
			return ret;
	} else {
		/* Disable initial deskew. */
		ret = regmap_write(priv->regmap, MAX96726A_MIPI_TX3(index), 0x0);
		if (ret)
			return ret;

		/* Disable periodic deskew. */
		ret = regmap_write(priv->regmap, MAX96726A_MIPI_TX4(index), 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int max96726a_set_phy_active(struct max_des *des, struct max_des_phy *phy,
				    bool enable)
{
	struct max96726a_priv *priv = des_to_priv(des);

	return regmap_assign_bits(priv->regmap, MAX96726A_MIPI_PHY0,
				  MAX96726A_MIPI_PHY0_PHY_STDBY_N(phy->index), enable);
}

static int max96726a_set_pipe_vc_remap(struct max_des *des,
				       struct max_des_pipe *pipe,
				       unsigned int i,
				       struct max_vc_remap *vc_remap)
{
	struct max96726a_priv *priv = des_to_priv(des);
	int ret;

	ret = regmap_update_bits(priv->regmap, MAX96726A_VC_REMAP_0_SRC(pipe->index, i),
				 MAX96726A_VC_REMAP_0_SRC_MASK,
				 FIELD_PREP(MAX96726A_VC_REMAP_0_SRC_MASK, vc_remap->src));

	return regmap_update_bits(priv->regmap, MAX96726A_VC_REMAP_0_DST(pipe->index, i),
				  MAX96726A_VC_REMAP_0_DST_MASK,
				  FIELD_PREP(MAX96726A_VC_REMAP_0_DST_MASK, vc_remap->dst));
}

static int max96726a_set_pipe_vc_remaps_enable(struct max_des *des,
					       struct max_des_pipe *pipe,
					       unsigned int mask)
{
	struct max96726a_priv *priv = des_to_priv(des);

	return regmap_update_bits(priv->regmap, MAX96726A_VC_REMAP_MODES(pipe->index),
				  MAX96726A_OVERRIDE_VC_SRC_DST_MODE |
				  MAX96726A_OVERRIDE_VC_PIPE_ID_MODE |
				  MAX96726A_OVERRIDE_VC_LINK_ID_MODE,
				  FIELD_PREP(MAX96726A_OVERRIDE_VC_SRC_DST_MODE, !!mask));
}

static int max96726a_set_pipe_enable(struct max_des *des, struct max_des_pipe *pipe,
				     bool enable)
{
	struct max96726a_priv *priv = des_to_priv(des);

	return regmap_assign_bits(priv->regmap, MAX96726A_VIDEO_PIPE_EN,
				  MAX96726A_VIDEO_PIPE_EN_MASK(pipe->index), enable);
}

static int max96726a_set_pipe_stream_id(struct max_des *des, struct max_des_pipe *pipe,
				        unsigned int stream_id)
{
	struct max96726a_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;

	return regmap_update_bits(priv->regmap, MAX96726A_VIDEO_PIPE_SEL(index),
				  MAX96726A_VIDEO_PIPE_SEL_STREAM(index),
				  field_prep(MAX96726A_VIDEO_PIPE_SEL_STREAM(index),
					     stream_id));
}

static int max96726a_set_pipe_tunnel_phy(struct max_des *des,
					 struct max_des_pipe *pipe,
					 struct max_des_phy *phy)
{
	struct max96726a_priv *priv = des_to_priv(des);
	unsigned int phy_id = phy->index == 0 ? 1 : 2;

	return regmap_update_bits(priv->regmap, MAX96726A_CONFIG_0(pipe->index),
				  MAX96726A_CONFIG_0_CON_DEST,
				  FIELD_PREP(MAX96726A_CONFIG_0_CON_DEST, phy_id));
}

static int max96726a_select_links(struct max_des *des, unsigned int mask)
{
	struct max96726a_priv *priv = des_to_priv(des);
	int ret;

	ret = regmap_update_bits(priv->regmap, MAX96726A_REG7,
				 MAX96726A_REG7_LINK_EN,
				 FIELD_PREP(MAX96726A_REG7_LINK_EN, mask));
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, MAX96726A_CTRL1,
			      MAX96726A_CTRL1_RESET_ONESHOT);
	if (ret)
		return ret;

	msleep(200);

	return 0;
}

static int max96726a_set_link_version(struct max_des *des,
				      struct max_des_link *link,
				      enum max_gmsl_version version)
{
	struct max96726a_priv *priv = des_to_priv(des);
	unsigned int index = link->index;
	bool gmsl3_en = version == MAX_GMSL_3;
	unsigned int mask, val;
	int ret;

	if (version == MAX_GMSL_3)
		val = MAX96726A_REG16_RX_RATE_12Gbps;
	else if (version == MAX_GMSL_2_6Gbps)
		val = MAX96726A_REG16_RX_RATE_6Gbps;
	else
		val = MAX96726A_REG16_RX_RATE_3Gbps;

	mask = MAX96726A_REG16_RX_RATE(index);
	ret = regmap_update_bits(priv->regmap, MAX96726A_REG16, mask,
				 field_prep(mask, val));
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96726A_FEC_RX_CTRL(index),
				 MAX96726A_FEC_RX_CTRL_RX_FEC_EN, gmsl3_en);
	if (ret)
		return ret;

	return regmap_assign_bits(priv->regmap, MAX96726A_REG7,
				  MAX96726A_REG7_GMSL3_X(index), gmsl3_en);
}

static const struct max_phys_config max96726a_phys_configs[] = {
	{ { 4, 4 } },
};

static const struct max_des_ops max96726a_ops = {
	.versions = BIT(MAX_GMSL_2_3Gbps) |
		    BIT(MAX_GMSL_2_6Gbps) |
		    BIT(MAX_GMSL_3),
	.modes = BIT(MAX_GMSL_TUNNEL_MODE),
	.use_atr = true,
	.num_pipes = 8,
	.num_phys = 2,
	.num_links = 4,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max96726a_phys_configs),
		.configs = max96726a_phys_configs,
	},
	.reg_read = max96726a_reg_read,
	.reg_write = max96726a_reg_write,
	.log_pipe_status = max9626a_log_pipe_status,
	.log_phy_status = max96726a_log_phy_status,
	.init = max96726a_init,
	.init_phy = max96726a_init_phy,
	.set_phy_active = max96726a_set_phy_active,
	.set_pipe_enable = max96726a_set_pipe_enable,
	.set_pipe_stream_id = max96726a_set_pipe_stream_id,
	.set_pipe_vc_remap = max96726a_set_pipe_vc_remap,
	.set_pipe_vc_remaps_enable = max96726a_set_pipe_vc_remaps_enable,
	.set_pipe_tunnel_phy = max96726a_set_pipe_tunnel_phy,
	.select_links = max96726a_select_links,
	.set_link_version = max96726a_set_link_version,
};

static int max96726a_probe(struct i2c_client *client)
{
	struct regmap_config i2c_regmap = max96726a_i2c_regmap;
	struct device *dev = &client->dev;
	struct max96726a_priv *priv;
	struct max_des_ops *ops;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ops = devm_kzalloc(dev, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	priv->info = device_get_match_data(dev);
	if (!priv->info) {
		dev_err(dev, "Failed to get match data\n");
		return -ENODEV;
	}

	priv->dev = dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->gpiod_pwdn = devm_gpiod_get_optional(&client->dev, "powerdown",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpiod_pwdn))
		return PTR_ERR(priv->gpiod_pwdn);

	if (priv->gpiod_pwdn) {
		/* PWDN must be held for 1us for reset */
		udelay(1);

		gpiod_set_value_cansleep(priv->gpiod_pwdn, 0);
		/* Maximum power-up time (tLOCK) 4ms */
		usleep_range(4000, 5000);
	}

	*ops = max96726a_ops;

	ops->versions = priv->info->versions;
	priv->des.ops = ops;

	ret = max96726a_reset(priv);
	if (ret)
		return ret;

	return max_des_probe(client, &priv->des);
}

static void max96726a_remove(struct i2c_client *client)
{
	struct max96726a_priv *priv = i2c_get_clientdata(client);

	max_des_remove(&priv->des);

	gpiod_set_value_cansleep(priv->gpiod_pwdn, 1);
}

static const struct max96726a_chip_info max96726a_info = {
	.versions = BIT(MAX_GMSL_2_3Gbps) |
		    BIT(MAX_GMSL_2_6Gbps) |
		    BIT(MAX_GMSL_3),
};

static const struct max96726a_chip_info max96726b_info = {
	.versions = BIT(MAX_GMSL_2_3Gbps) |
		    BIT(MAX_GMSL_2_6Gbps),
};

static const struct of_device_id max96726a_of_table[] = {
	{ .compatible = "maxim,max96726a", .data = &max96726a_info },
	{ .compatible = "maxim,max96726b", .data = &max96726b_info },
	{ },
};
MODULE_DEVICE_TABLE(of, max96726a_of_table);

static struct i2c_driver max96726a_i2c_driver = {
	.driver	= {
		.name = "max96726a",
		.of_match_table	= of_match_ptr(max96726a_of_table),
	},
	.probe = max96726a_probe,
	.remove = max96726a_remove,
};

module_i2c_driver(max96726a_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX96726A Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
