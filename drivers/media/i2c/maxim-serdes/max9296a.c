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

/* TODO: backport fixes from MAX96724. */

#define MAX9296A_DPLL_FREQ		2500
#define MAX9296A_PIPES_NUM		4

struct max9296a_priv {
	struct max_des_priv des_priv;
	const struct max9296a_chip_info *info;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
};

struct max9296a_chip_info {
	unsigned int num_pipes;
	unsigned int pipe_hw_ids[MAX9296A_PIPES_NUM];
	unsigned int num_phys;
	unsigned int num_links;
	bool phy0_first_lanes_on_master_phy;
};

#define des_to_priv(des) \
	container_of(des, struct max9296a_priv, des_priv)

static int max9296a_read(struct max9296a_priv *priv, int reg)
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

static int max9296a_write(struct max9296a_priv *priv, unsigned int reg, u8 val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	dev_err(priv->dev, "write %d 0x%x = 0x%02x\n", ret, reg, val);
	if (ret)
		dev_err(priv->dev, "write 0x%04x failed\n", reg);

	return ret;
}

static int max9296a_update_bits(struct max9296a_priv *priv, unsigned int reg,
			        u8 mask, u8 val)
{
	int ret;

	ret = regmap_update_bits(priv->regmap, reg, mask, val);
	dev_err(priv->dev, "update %d 0x%x 0x%02x = 0x%02x\n", ret, reg, mask, val);
	if (ret)
		dev_err(priv->dev, "update 0x%04x failed\n", reg);

	return ret;
}

static int max9296a_wait_for_device(struct max9296a_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 10; i++) {
		ret = max9296a_read(priv, 0x0);
		if (ret >= 0)
			return 0;

		msleep(100);

		dev_err(priv->dev, "Retry %u waiting for deserializer: %d\n", i, ret);
	}

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

static unsigned int max9296a_pipe_id(struct max9296a_priv *priv,
				     struct max_des_pipe *pipe)
{
	return priv->info->pipe_hw_ids[pipe->index];
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
	unsigned int master_phy, slave_phy;
	unsigned int master_shift, slave_shift;
	unsigned int reg, val, mask;
	unsigned int clk_bit, lane_0_bit, lane_2_bit;
	unsigned int used_data_lanes = 0;
	unsigned int i;
	int ret;

	/*
	 * MAX9296A has four PHYs, but does not support single-PHY configurations,
	 * only double-PHY configurations, even when only using two lanes.
	 * For PHY 0 + PHY 1, PHY 1 is the master PHY.
	 * For PHY 2 + PHY 3, PHY 2 is the master PHY.
	 * Clock is always on the master PHY.
	 * For first pair of PHYs, first lanes are on the master PHY.
	 * For second pair of PHYs, first lanes are on the master PHY too.
	 *
	 * PHY 0 + 1
	 * CLK = PHY 1
	 * PHY1 Lane 0 = D0
	 * PHY1 Lane 1 = D1
	 * PHY0 Lane 0 = D2
	 * PHY0 Lane 1 = D3
	 *
	 * PHY 2 + 3
	 * CLK = PHY 2
	 * PHY2 Lane 0 = D0
	 * PHY2 Lane 1 = D1
	 * PHY3 Lane 0 = D2
	 * PHY3 Lane 1 = D3
	 */
	if (phy->index == 0) {
		master_phy = 1;
		slave_phy = 0;
	} else if (phy->index == 1) {
		master_phy = 2;
		slave_phy = 3;
	} else {
		return -EINVAL;
	}

	/* Configure a lane count. */
	/* TODO: Add support CPHY mode. */
	ret = max9296a_update_bits(priv, 0x44a + 0x40 * phy->index, GENMASK(7, 6),
				   FIELD_PREP(GENMASK(7, 6), num_data_lanes - 1));
	if (ret)
		return ret;

	/* Configure lane mapping. */
	/*
	 * The lane of each PHY can be mapped to physical lanes 0, 1, 2,
	 * and 3. This mapping is exclusive, multiple lanes, even if unused
	 * cannot be mapped to the same physical lane.
	 * Each lane mapping is represented as two bits.
	 */
	reg = 0x333 + phy->index;

	master_shift = (master_phy % 2) * 4;
	slave_shift = (slave_phy % 2) * 4;

	if (phy->index == 0 && priv->info->phy0_first_lanes_on_master_phy) {
		lane_0_bit = master_shift;
		lane_2_bit = slave_shift;
	} else {
		lane_0_bit = slave_shift;
		lane_2_bit = master_shift;
	}

	val = 0;
	for (i = 0; i < 4 ; i++) {
		unsigned int shift;
		unsigned int map;

		if (i < num_data_lanes) {
			if (phy->mipi.data_lanes[i] < 1)
				return -EINVAL;

			map = phy->mipi.data_lanes[i] - 1;
		} else {
			map = ffz(used_data_lanes);
		}

		if (i < 2)
			shift = lane_0_bit;
		else
			shift = lane_2_bit;

		shift += (i % 2) * 2;

		val |= map << shift;

		used_data_lanes |= BIT(map);
	}

	ret = max9296a_update_bits(priv, reg, 0xff, val);
	if (ret)
		return ret;

	/*
	 * Configure lane polarity.
	 * PHY 0 and 1 are on register 0x335.
	 * PHY 1 and 2 are on register 0x336.
	 * Each PHY has 3 bits of polarity configuration.
	 * 0th bit is for lane 0.
	 * 1st bit is for lane 1.
	 * 2nd bit is for clock lane.
	 */
	reg = 0x335 + phy->index;

	master_shift = (master_phy % 2) * 3;
	slave_shift = (slave_phy % 2) * 3;
	clk_bit = master_shift + 2;

	if (phy->index == 0 && priv->info->phy0_first_lanes_on_master_phy) {
		lane_0_bit = master_shift;
		lane_2_bit = slave_shift;
	} else {
		lane_0_bit = slave_shift;
		lane_2_bit = master_shift;
	}

	val = 0;

	if (phy->mipi.lane_polarities[0])
		val |= BIT(clk_bit);

	for (i = 0; i < num_data_lanes; i++) {
		unsigned int shift;
		unsigned int map;

		if (!phy->mipi.lane_polarities[i + 1])
			continue;

		map = i;

		if (map < 2)
			shift = lane_0_bit;
		else
			shift = lane_2_bit;

		shift += map % 2;

		val |= BIT(shift);
	}

	ret = max9296a_update_bits(priv, reg, 0x3f, val);
	if (ret)
		return ret;

	/* Put DPLL block into reset. */
	ret = max9296a_update_bits(priv, 0x1c00 + 0x100 * master_phy, BIT(0), 0x00);
	if (ret)
		return ret;

	/* Set DPLL frequency. */
	reg = 0x31d + 0x3 * master_phy;
	ret = max9296a_update_bits(priv, reg, GENMASK(4, 0),
				   MAX9296A_DPLL_FREQ / 100);
	if (ret)
		return ret;

	/* Enable DPLL frequency. */
	ret = max9296a_update_bits(priv, reg, BIT(5), BIT(5));
	if (ret)
		return ret;

	/* Pull DPLL block out of reset. */
	ret = max9296a_update_bits(priv, 0x1c00 + 0x100 * master_phy, BIT(0), 0x01);
	if (ret)
		return ret;


	if (phy->link_frequency > 1500000000ull) {
		/* Enable initial deskew with 2 x 32k UI. */
		ret = max9296a_write(priv, 0x443 + 0x40 * phy->index, 0x81);
		if (ret)
			return ret;

		/* Enable periodic deskew with 2 x 1k UI.. */
		ret = max9296a_write(priv, 0x444 + 0x40 * phy->index, 0x81);
		if (ret)
			return ret;
	} else {
		/* Disable initial deskew. */
		ret = max9296a_write(priv, 0x443 + 0x40 * phy->index, 0x07);
		if (ret)
			return ret;

		/* Disable periodic deskew. */
		ret = max9296a_write(priv, 0x444 + 0x40 * phy->index, 0x01);
		if (ret)
			return ret;
	}

	/* Set alternate memory map modes. */
	val  = phy->alt_mem_map12 ? BIT(0) : 0;
	val |= phy->alt_mem_map8 ? BIT(1) : 0;
	val |= phy->alt_mem_map10 ? BIT(2) : 0;
	ret = max9296a_update_bits(priv, 0x433 + 0x40 * master_phy, GENMASK(2, 0), val);

	/* Enable PHY. */
	mask = (BIT(master_phy) | BIT(slave_phy)) << 4;
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
	unsigned int index = max9296a_pipe_id(priv, pipe);
	unsigned int reg, val, shift, mask;
	unsigned int phy_id;
	int ret;

	if (remap->phy == 0)
		phy_id = 1;
	else if (remap->phy == 1)
		phy_id = 2;

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
	unsigned int index = max9296a_pipe_id(priv, pipe);
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
	struct device *dev = &client->dev;
	struct max9296a_priv *priv;
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

	priv->regmap = devm_regmap_init_i2c(client, &max_des_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	*ops = max9296a_ops;

	ops->num_phys = priv->info->num_phys;
	ops->num_pipes = priv->info->num_pipes;
	ops->num_links = priv->info->num_links;

	priv->des_priv.dev = dev;
	priv->des_priv.client = client;
	priv->des_priv.regmap = priv->regmap;
	priv->des_priv.ops = ops;

	ret = max9296a_reset(priv);
	if (ret)
		return ret;

	return max_des_probe(&priv->des_priv);
}

static int max9296a_remove(struct i2c_client *client)
{
	struct max9296a_priv *priv = i2c_get_clientdata(client);

	return max_des_remove(&priv->des_priv);
}

static const struct max9296a_chip_info max9296a_info = {
	.phy0_first_lanes_on_master_phy = true,
	.num_pipes = 4,
	.pipe_hw_ids = { 0, 1, 2, 3 },
	.num_phys = 2,
	.num_links = 2,
};

static const struct of_device_id max9296a_of_table[] = {
	{ .compatible = "maxim,max9296a", .data = &max9296a_info },
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
