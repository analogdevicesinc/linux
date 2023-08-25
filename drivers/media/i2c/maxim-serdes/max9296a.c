// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX9296A Quad GMSL2 Deserializer Driver
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include "max_des.h"

#define MAX9296A_DPLL_FREQ		2500

struct max9296a_priv {
	struct max_des_priv des_priv;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
};

#define des_to_priv(des) \
	container_of(des, struct max9296a_priv, des_priv)

static int max9296a_read(struct max9296a_priv *priv, int reg)
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

static int max9296a_write(struct max9296a_priv *priv, unsigned int reg, u8 val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	dev_dbg(priv->dev, "write %d 0x%x = 0x%02x\n", ret, reg, val);
	if (ret)
		dev_err(priv->dev, "write 0x%04x failed\n", reg);

	return ret;
}

static int max9296a_update_bits(struct max9296a_priv *priv, unsigned int reg,
			        u8 mask, u8 val)
{
	int ret;

	ret = regmap_update_bits(priv->regmap, reg, mask, val);
	dev_dbg(priv->dev, "update %d 0x%x 0x%02x = 0x%02x\n", ret, reg, mask, val);
	if (ret)
		dev_err(priv->dev, "update 0x%04x failed\n", reg);

	return ret;
}

static int max9296a_wait_for_device(struct max9296a_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 100; i++) {
		ret = max9296a_read(priv, 0x0);
		if (ret >= 0)
			return 0;

		msleep(10);

		dev_dbg(priv->dev, "Retry %u waiting for deserializer: %d\n", i, ret);
	}

	dev_err(priv->dev, "Deserializer not found. Err: %d\n", ret);

	return ret;
}

static int max9296a_reset(struct max9296a_priv *priv)
{
	int ret;

	ret = max9296a_wait_for_device(priv);
	if (ret)
		return ret;

	ret = max9296a_update_bits(priv, 0x10, 0x80, 0x80);
	if (ret)
		return ret;

	ret = max9296a_wait_for_device(priv);
	if (ret)
		return ret;

	return 0;
}

static int max9296a_mipi_enable(struct max_des_priv *des_priv, bool enable)
{
	struct max9296a_priv *priv = des_to_priv(des_priv);
	int ret;

	if (enable) {
		ret = max9296a_update_bits(priv, 0x313, 0x02, 0x02);
		if (ret)
			return ret;

		ret = max9296a_update_bits(priv, 0x330, 0x80, 0x80);
		if (ret)
			return ret;
	} else {
		ret = max9296a_update_bits(priv, 0x330, 0x80, 0x00);
		if (ret)
			return ret;

		ret = max9296a_update_bits(priv, 0x313, 0x02, 0x00);
		if (ret)
			return ret;
	}

	return 0;
}

static int max9296a_init(struct max_des_priv *des_priv)
{
	struct max9296a_priv *priv = des_to_priv(des_priv);
	int ret;

	/* Disable all PHYs. */
	ret = max9296a_update_bits(priv, 0x332, GENMASK(7, 4), 0x00);
	if (ret)
		return ret;

	/* Disable all pipes. */
	ret = max9296a_update_bits(priv, 0x2, GENMASK(7, 4), 0x00);
	if (ret)
		return ret;

	/* Disable link auto-select. */
	ret = max9296a_update_bits(priv, 0x10, BIT(4), 0);
	if (ret)
		return ret;

	return 0;
}

static int max9296a_init_phy(struct max_des_priv *des_priv,
			     struct max_des_phy *phy)
{
	struct max9296a_priv *priv = des_to_priv(des_priv);
	unsigned int num_data_lanes = phy->mipi.num_data_lanes;
	unsigned int reg, val, shift, mask, clk_bit;
	unsigned int index;
	unsigned int i;
	int ret;

	/*
	 * MAX9296A has four PHYs, but does not support single-PHY configurations,
	 * only double-PHY configurations, even when only using two lanes.
	 * Some registers are indexed for 4 PHYs, and some are indexed for only 2 PHYs.
	 */
	index = phy->index * 2;

	/* Configure a lane count. */
	/* TODO: Add support CPHY mode. */
	ret = max9296a_update_bits(priv, 0x44a + 0x40 * index / 2, GENMASK(7, 6),
				   FIELD_PREP(GENMASK(7, 6), num_data_lanes - 1));
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

	reg = 0x333 + index / 2;

	/* Configure lane mapping. */
	/* TODO: Add support for lane swapping. */
	ret = max9296a_update_bits(priv, reg, mask << shift, val << shift);
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

	reg = 0x335 + index / 2;

	/* Configure lane polarity. */
	val = 0;
	for (i = 0; i < num_data_lanes + 1; i++)
		if (phy->mipi.lane_polarities[i])
			val |= BIT(i == 0 ? clk_bit : i < 3 ? i - 1 : i);
	ret = max9296a_update_bits(priv, reg, mask << shift, val << shift);
	if (ret)
		return ret;

	/* Put DPLL block into reset. */
	ret = max9296a_update_bits(priv, 0x1c00 + 0x100 * index, BIT(0), 0x00);
	if (ret)
		return ret;

	/* Set DPLL frequency. */
	reg = 0x31d + 0x3 * index;
	ret = max9296a_update_bits(priv, reg, GENMASK(4, 0),
				   MAX9296A_DPLL_FREQ / 100);
	if (ret)
		return ret;

	/* Enable DPLL frequency. */
	ret = max9296a_update_bits(priv, reg, BIT(5), BIT(5));
	if (ret)
		return ret;

	/* Pull DPLL block out of reset. */
	ret = max9296a_update_bits(priv, 0x1c00 + 0x100 * index, BIT(0), 0x01);
	if (ret)
		return ret;

	/* Disable initial deskew. */
	ret = max9296a_write(priv, 0x443 + 0x40 * index / 2, 0x07);
	if (ret)
		return ret;

	/* Disable periodic deskew. */
	ret = max9296a_write(priv, 0x444 + 0x40 * index / 2, 0x01);
	if (ret)
		return ret;

	/* Set alternate memory map modes. */
	val  = phy->alt_mem_map12 ? BIT(0) : 0;
	val |= phy->alt_mem_map8 ? BIT(1) : 0;
	val |= phy->alt_mem_map10 ? BIT(2) : 0;
	ret = max9296a_update_bits(priv, 0x433 + 0x40 * index, GENMASK(2, 0), val);

	/* Enable PHY. */
	mask = GENMASK(1, 0) << (index + 4);
	ret = max9296a_update_bits(priv, 0x332, mask, mask);
	if (ret)
		return ret;

	return 0;
}

static int max9296a_init_pipe_remap(struct max9296a_priv *priv,
				    struct max_des_pipe *pipe,
				    struct max_des_dt_vc_remap *remap,
				    unsigned int i)
{
	unsigned int index = pipe->index;
	unsigned int reg, val, shift, mask;
	unsigned int phy_id = remap->phy * 2;
	int ret;

	/* Set source Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	reg = 0x40d + 0x40 * index + i * 2;
	ret = max9296a_write(priv, reg,
			     MAX_DES_DT_VC(remap->from_dt, remap->from_vc));
	if (ret)
		return ret;

	/* Set destination Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	reg = 0x40e + 0x40 * index + i * 2;
	ret = max9296a_write(priv, reg,
			     MAX_DES_DT_VC(remap->to_dt, remap->to_vc));
	if (ret)
		return ret;

	/* Set destination PHY. */
	reg = 0x42d + 0x40 * index + i / 4;
	shift = (i % 4) * 2;
	mask = 0x3 << shift;
	val = (phy_id & 0x3) << shift;
	ret = max9296a_update_bits(priv, reg, mask, val);
	if (ret)
		return ret;

	/* Enable remap. */
	reg = 0x40b + 0x40 * index + i / 8;
	val = BIT(i % 8);
	ret = max9296a_update_bits(priv, reg, val, val);
	if (ret)
		return ret;

	return 0;
}

static int max9296a_update_pipe_remaps(struct max_des_priv *des_priv,
				     struct max_des_pipe *pipe)
{
	struct max9296a_priv *priv = des_to_priv(des_priv);
	unsigned int i;
	int ret;

	for (i = 0; i < pipe->num_remaps; i++) {
		struct max_des_dt_vc_remap *remap = &pipe->remaps[i];

		ret = max9296a_init_pipe_remap(priv, pipe, remap, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int max9296a_init_pipe(struct max_des_priv *des_priv,
			      struct max_des_pipe *pipe)
{
	struct max9296a_priv *priv = des_to_priv(des_priv);
	unsigned int index = pipe->index;
	unsigned int reg, mask;
	int ret;

	/* Enable pipe. */
	mask = BIT(index + 4);
	ret = max9296a_update_bits(priv, 0x2, mask, mask);
	if (ret)
		return ret;

	/* Set source stream. */
	reg = 0x50 + index;
	ret = max9296a_update_bits(priv, reg, GENMASK(1, 0), pipe->stream_id);
	if (ret)
		return ret;

	return 0;
}

static int max9296a_select_links(struct max_des_priv *des_priv,
				 unsigned int mask)
{
	struct max9296a_priv *priv = des_to_priv(des_priv);

	if (!mask) {
		dev_err(priv->dev, "Disable all links unsupported\n");
		return -EINVAL;
	}

	return max9296a_update_bits(priv, 0x10,
				    BIT(5) | GENMASK(1, 0),
				    BIT(5) | FIELD_PREP(GENMASK(1, 0), mask));
}

static const struct max_des_ops max9296a_ops = {
	.num_phys = 2,
	.num_pipes = 4,
	.num_links = 2,
	.fix_tx_ids = true,
	.mipi_enable = max9296a_mipi_enable,
	.init = max9296a_init,
	.init_phy = max9296a_init_phy,
	.init_pipe = max9296a_init_pipe,
	.update_pipe_remaps = max9296a_update_pipe_remaps,
	.select_links = max9296a_select_links,
};

static int max9296a_probe(struct i2c_client *client)
{
	struct max9296a_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &client->dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &max_des_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->des_priv.dev = &client->dev;
	priv->des_priv.client = client;
	priv->des_priv.regmap = priv->regmap;
	priv->des_priv.ops = &max9296a_ops;

	ret = max9296a_reset(priv);
	if (ret)
		return ret;

	return max_des_probe(&priv->des_priv);
}

static void max9296a_remove(struct i2c_client *client)
{
	struct max9296a_priv *priv = i2c_get_clientdata(client);

	max_des_remove(&priv->des_priv);
}

static const struct of_device_id max9296a_of_table[] = {
	{ .compatible = "maxim,max9296a" },
	{ },
};
MODULE_DEVICE_TABLE(of, max9296a_of_table);

static struct i2c_driver max9296a_i2c_driver = {
	.driver	= {
		.name = "max9296a",
		.of_match_table	= of_match_ptr(max9296a_of_table),
	},
	.probe_new = max9296a_probe,
	.remove = max9296a_remove,
};

module_i2c_driver(max9296a_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX9296A Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
