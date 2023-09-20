// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX96724 Quad GMSL2 Deserializer Driver
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include "max_des.h"

#define MAX96724_PHYS_NUM		4

struct max96724_priv {
	struct max_des_priv des_priv;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
};

#define des_to_priv(des) \
	container_of(des, struct max96724_priv, des_priv)

static int max96724_read(struct max96724_priv *priv, int reg)
{
	int ret, val;

	ret = regmap_read(priv->regmap, reg, &val);
	dev_err(priv->dev, "read %d 0x%x = 0x%02x\n", ret, reg, val);
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
	dev_err(priv->dev, "write %d 0x%x = 0x%02x\n", ret, reg, val);
	if (ret)
		dev_err(priv->dev, "write 0x%04x failed\n", reg);

	return ret;
}

static int max96724_update_bits(struct max96724_priv *priv, unsigned int reg,
			        u8 mask, u8 val)
{
	int ret;

	ret = regmap_update_bits(priv->regmap, reg, mask, val);
	dev_err(priv->dev, "update %d 0x%x 0x%02x = 0x%02x\n", ret, reg, mask, val);
	if (ret)
		dev_err(priv->dev, "update 0x%04x failed\n", reg);

	return ret;
}

static int max96724_wait_for_device(struct max96724_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 10; i++) {
		ret = max96724_read(priv, 0x0);
		if (ret >= 0)
			return 0;

		msleep(100);

		dev_err(priv->dev, "Retry %u waiting for deserializer: %d\n", i, ret);
	}

	return ret;
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

static int max96724_log_pipe_status(struct max_des_priv *des_priv,
				    struct max_des_pipe *pipe, const char *name)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	unsigned int index = pipe->index;
	unsigned int reg, mask;
	int ret;

	reg = 0x1dc + index * 0x20;
	mask = BIT(0);
	ret = max96724_read(priv, reg);
	if (ret < 0)
		return ret;

	ret = ret & mask;
	pr_info("%s: \tvideo_lock: %u\n", name, ret);

	return 0;
}

static int max96724_log_phy_status(struct max_des_priv *des_priv,
				   struct max_des_phy *phy, const char *name)
{
	struct max96724_priv *priv = des_to_priv(des_priv);
	unsigned int index = phy->index;
	unsigned int reg, mask, shift;
	int ret;

	reg = 0x8d0 + index / 2;
	shift = 4 * (index % 2);
	mask = GENMASK(3, 0);
	ret = max96724_read(priv, reg);
	if (ret < 0)
		return ret;

	ret = (ret >> shift) & mask;
	pr_info("%s: \tcsi2_pkt_cnt: %u\n", name, ret);

	reg += 2;
	ret = max96724_read(priv, reg);
	if (ret < 0)
		return ret;

	ret = (ret >> shift) & mask;
	pr_info("%s: \tphy_pkt_cnt: %u\n", name, ret);

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

struct max96724_lane_config {
	unsigned int lanes[MAX96724_PHYS_NUM];
	unsigned int clock_lane[MAX96724_PHYS_NUM];
	unsigned int bit;
};

static const struct max96724_lane_config max96724_lane_configs[] = {

	/*
	 * PHY 1 can be in 4-lane mode (combining lanes of PHY 0 and PHY 1)
	 * but only use the data lanes of PHY0, while continuing to use the
	 * clock lane of PHY 1.
	 */
	{ { 0, 2, 4, 0 }, { 0, 3, 0, 0 }, BIT(2) },
	{ { 0, 2, 2, 2 }, { 0, 3, 0, 0 }, BIT(3) },

	{ { 2, 2, 2, 2 }, { 0, 0, 0, 0 }, BIT(0) },
	{ { 0, 4, 4, 0 }, { 0, 0, 0, 0 }, BIT(2) },
	{ { 0, 4, 2, 2 }, { 0, 0, 0, 0 }, BIT(3) },
	{ { 2, 2, 4, 0 }, { 0, 0, 0, 0 }, BIT(4) },
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

			if (!phy->enabled)
				continue;

			if (phy->mipi.num_data_lanes == max96724_lane_configs[i].lanes[j] &&
			    phy->mipi.clock_lane == max96724_lane_configs[i].clock_lane[j])
				continue;

			matching = false;
			break;
		}

		if (matching)
			break;
	}

	if (i == num_lane_configs) {
		dev_err(priv->dev, "Invalid lane configuration\n");
		return -EINVAL;
	}

	ret = max96724_update_bits(priv, 0x8a0, 0x1f,
				   max96724_lane_configs[i].bit);
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

	ret = max96724_update_bits(priv, 0xf4, BIT(4),
				   des_priv->pipe_stream_autoselect
				   ? BIT(4) : 0x00);
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
	unsigned int num_hw_data_lanes;
	unsigned int reg, val, shift, mask, clk_bit;
	unsigned int index = phy->index;
	unsigned int i;
	int ret;

	/* Configure a lane count. */
	/* TODO: Add support CPHY mode. */
	if (index == 1 && phy->mipi.clock_lane == 3 &&
	    phy->mipi.num_data_lanes == 2)
		num_hw_data_lanes = 4;
	else
		num_hw_data_lanes = phy->mipi.num_data_lanes;

	reg = 0x90a + 0x40 * index;
	shift = 6;
	mask = GENMASK(1, 0);
	val = num_data_lanes - 1;
	ret = max96724_update_bits(priv, reg, mask << shift, val << shift);
	if (ret)
		return ret;

	/* Configure lane mapping. */
	/* TODO: Add support for lane swapping. */
	if (num_hw_data_lanes == 4) {
		mask = 0xff;
		val = 0xe4;
		shift = 0;
	} else {
		mask = 0xf;
		val = 0x4;
		shift = 4 * (index % 2);
	}

	reg = 0x8a3 + index / 2;

	ret = max96724_update_bits(priv, reg, mask << shift, val << shift);
	if (ret)
		return ret;

	/* Configure lane polarity. */
	if (num_hw_data_lanes == 4) {
		mask = 0x3f;
		clk_bit = 5;
		shift = 0;
	} else {
		mask = 0x7;
		clk_bit = 2;
		shift = 4 * (index % 2);
	}

	reg = 0x8a5 + index / 2;

	val = 0;
	for (i = 0; i < num_data_lanes + 1; i++)
		if (phy->mipi.lane_polarities[i])
			val |= BIT(i == 0 ? clk_bit : i < 3 ? i - 1 : i);
	ret = max96724_update_bits(priv, reg, mask << shift, val << shift);
	if (ret)
		return ret;

	if (phy->link_frequency > 1500000000ull) {
		/* Enable initial deskew with 2 x 32k UI. */
		ret = max96724_write(priv, 0x903 + 0x40 * index, 0x81);
		if (ret)
			return ret;

		/* Disable periodic deskew with 2 x 1k UI.. */
		ret = max96724_write(priv, 0x904 + 0x40 * index, 0x81);
		if (ret)
			return ret;
	} else {
		/* Disable initial deskew. */
		ret = max96724_write(priv, 0x903 + 0x40 * index, 0x07);
		if (ret)
			return ret;

		/* Disable periodic deskew. */
		ret = max96724_write(priv, 0x904 + 0x40 * index, 0x01);
		if (ret)
			return ret;
	}

	/* Put DPLL block into reset. */
	ret = max96724_update_bits(priv, 0x1c00 + 0x100 * index, BIT(0), 0x00);
	if (ret)
		return ret;

	/* Set DPLL frequency. */
	reg = 0x415 + 0x3 * index;
	ret = max96724_update_bits(priv, reg, GENMASK(4, 0),
				   div_u64(phy->link_frequency, 100000000));
	if (ret)
		return ret;

	/* Enable DPLL frequency. */
	ret = max96724_update_bits(priv, reg, BIT(5), BIT(5));
	if (ret)
		return ret;

	/* Pull DPLL block out of reset. */
	reg = 0x1c00 + 0x100 * index;
	ret = max96724_update_bits(priv, reg, BIT(0), 0x01);
	if (ret)
		return ret;

	/* Set alternate memory map modes. */
	val  = phy->alt_mem_map12 ? BIT(0) : 0;
	val |= phy->alt_mem_map8 ? BIT(1) : 0;
	val |= phy->alt_mem_map10 ? BIT(2) : 0;
	val |= phy->alt2_mem_map8 ? BIT(4) : 0;
	reg = 0x933 + 0x40 * index;
	ret = max96724_update_bits(priv, reg, GENMASK(2, 0), val);
	if (ret)
		return ret;

	/* Enable PHY. */
	shift = 4;
	if (num_hw_data_lanes == 4)
		/* PHY 1 -> bits [1:0] */
		/* PHY 2 -> bits [3:2] */
		mask = 0x3 << ((index / 2) * 2 + shift);
	else
		mask = 0x1 << (index + shift);

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
	unsigned int reg, shift, mask;
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

	if (!des_priv->pipe_stream_autoselect) {
		/* Set source stream. */
		reg = 0xf0 + index / 2;
		shift = 4 * (index % 2);
		ret = max96724_update_bits(priv, reg, GENMASK(1, 0) << shift,
					   pipe->stream_id << shift);
		if (ret)
			return ret;
	}

	/* Set source link. */
	shift += 2;
	ret = max96724_update_bits(priv, reg, GENMASK(1, 0) << shift,
				   pipe->link_id << shift);
	if (ret)
		return ret;

	/* Set 8bit double mode. */
	mask = BIT(index) << 4;
	ret = max96724_update_bits(priv, 0x414, mask, pipe->dbl8 ? mask : 0);
	if (ret)
		return ret;

	mask = BIT(index) << 4;
	ret = max96724_update_bits(priv, 0x417, mask, pipe->dbl8mode ? mask : 0);
	if (ret)
		return ret;

	/* Set 10bit double mode. */
	if (index == 3) {
		reg = 0x41d;
		mask = BIT(4);
	} else if (index == 2) {
		reg = 0x41e;
		mask = BIT(6);
	} else if (index == 1) {
		reg = 0x41f;
		mask = BIT(6);
	} else {
		reg = 0x41f;
		mask = BIT(4);
	}

	ret = max96724_update_bits(priv, reg,
				   mask | (mask << 1),
				   (pipe->dbl10 ? mask : 0) |
				   (pipe->dbl10mode ? (mask << 1) : 0));
	if (ret)
		return ret;

	/* Set 12bit double mode. */
	mask = BIT(index);
	ret = max96724_update_bits(priv, 0x41f, mask, pipe->dbl12 ? mask : 0);
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

static const struct max_des_ops max96724_ops = {
	.num_phys = 4,
	.num_pipes = 4,
	.num_links = 4,
	.supports_pipe_link_remap = true,
	.supports_pipe_stream_autoselect = true,
	.supports_tunnel_mode = true,
	.log_pipe_status = max96724_log_pipe_status,
	.log_phy_status = max96724_log_phy_status,
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

	return max_des_probe(&priv->des_priv);
}

static int max96724_remove(struct i2c_client *client)
{
	struct max96724_priv *priv = i2c_get_clientdata(client);

	return max_des_remove(&priv->des_priv);
}

static const struct of_device_id max96724_of_table[] = {
	{ .compatible = "maxim,max96724" },
	{ },
};
MODULE_DEVICE_TABLE(of, max96724_of_table);

static struct i2c_driver max96724_i2c_driver = {
	.driver	= {
		.name = "max96724",
		.of_match_table	= of_match_ptr(max96724_of_table),
	},
	.probe_new = max96724_probe,
	.remove = max96724_remove,
};

module_i2c_driver(max96724_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX96724 Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
