// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX9296A Quad GMSL2 Deserializer Driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include "max_des.h"

#define MAX9296A_REG0				0x0

#define MAX9296A_REG1				0x1
#define MAX9296A_REG1_RX_RATE_A			GENMASK(1, 0)
#define MAX9296A_REG1_RX_RATE_3Gbps		0b01
#define MAX9296A_REG1_RX_RATE_6Gbps		0b10
#define MAX9296A_REG1_RX_RATE_12Gbps		0b11

#define MAX9296A_REG2				0x2
#define MAX9296A_REG2_VID_EN(p)			BIT((p) + 4)

#define MAX9296A_REG4				0x4
#define MAX9296A_REG4_GMSL3_X(x)		BIT((x) + 6)
#define MAX9296A_REG4_RX_RATE_B			GENMASK(1, 0)

#define MAX9296A_REG6				0x6
#define MAX9296A_REG6_GMSL2_X(x)		BIT((x) + 6)

#define MAX9296A_CTRL0				0x10
#define MAX9296A_CTRL0_LINK_CFG			GENMASK(1, 0)
#define MAX9296A_CTRL0_AUTO_LINK		BIT(4)
#define MAX9296A_CTRL0_RESET_ONESHOT		BIT(5)
#define MAX9296A_CTRL0_RESET_ALL		BIT(7)

#define MAX9296A_CTRL2				0x12
#define MAX9296A_CTRL2_RESET_ONESHOT_B		BIT(5)

#define MAX9296A_MIPI_TX0(x)			(0x28 + (x) * 0x5000)
#define MAX9296A_MIPI_TX0_RX_FEC_EN		BIT(1)

#define MAX9296A_RX50(p)			(0x50 + (p))
#define MAX9296A_RX50_STR_SEL			GENMASK(1, 0)

#define MAX9296A_VIDEO_PIPE_EN			0x160
#define MAX9296A_VIDEO_PIPE_EN_MASK(p)		BIT(p)

#define MAX9296A_VIDEO_PIPE_SEL			0x161
#define MAX9296A_VIDEO_PIPE_SEL_STREAM(p)	(GENMASK(1, 0) << ((p) * 3))

#define MAX9296A_VPRBS(p)			(0x1dc + (p) * 0x20)
#define MAX9296A_VPRBS_VIDEO_LOCK		BIT(0)

#define MAX9296A_BACKTOP12			0x313
#define MAX9296A_BACKTOP12_CSI_OUT_EN		BIT(1)

#define MAX9296A_BACKTOP21			0x31c
#define MAX9296A_BACKTOP21_BPP8DBL(p)		BIT(4 + (p))

#define MAX9296A_BACKTOP22(x)			(0x31d + (x) * 0x3)
#define MAX9296A_BACKTOP22_PHY_CSI_TX_DPLL	GENMASK(4, 0)
#define MAX9296A_BACKTOP22_PHY_CSI_TX_DPLL_EN	BIT(5)

#define MAX9296A_BACKTOP24			0x31f
#define MAX9296A_BACKTOP24_BPP8DBL_MODE(p)	BIT(4 + (p))

#define MAX9296A_BACKTOP32			0x327
#define MAX9296A_BACKTOP32_BPP10DBL(p)		BIT(p)
#define MAX9296A_BACKTOP32_BPP10DBL_MODE(p)	BIT(4 + (p))

#define MAX9296A_BACKTOP33			0x328
#define MAX9296A_BACKTOP32_BPP12DBL(p)		BIT(p)

#define MAX9296A_MIPI_PHY2			0x332
#define MAX9296A_MIPI_PHY2_PHY_STDBY_N(x)	(GENMASK(5, 4) << ((x) * 2))

#define MAX9296A_MIPI_PHY3(x)			(0x333 + (x))
#define MAX9296A_MIPI_PHY3_PHY_LANE_MAP_4	GENMASK(7, 0)

#define MAX9296A_MIPI_PHY5(x)			(0x335 + (x))
#define MAX9296A_MIPI_PHY5_PHY_POL_MAP_0_1	GENMASK(1, 0)
#define MAX9296A_MIPI_PHY5_PHY_POL_MAP_2_3	GENMASK(4, 3)
#define MAX9296A_MIPI_PHY5_PHY_POL_MAP_CLK(x)	((x) == 0 ? BIT(5) : BIT(2))

#define MAX9296A_MIPI_PHY18			(0x342)
#define MAX9296A_MIPI_PHY18_CSI2_TX_PKT_CNT(x)	(GENMASK(3, 0) << (4 * (x)))

#define MAX9296A_MIPI_PHY20			(0x342)
#define MAX9296A_MIPI_PHY20_PHY_PKT_CNT(x)	(GENMASK(3, 0) << (4 * (x)))

#define MAX9296A_MIPI_TX3(x)			(0x403 + (x) * 0x40)
#define MAX9296A_MIPI_TX3_DESKEW_INIT_8X32K	FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX9296A_MIPI_TX3_DESKEW_INIT_AUTO	BIT(7)

#define MAX9296A_MIPI_TX4(x)			(0x404 + (x) * 0x40)
#define MAX9296A_MIPI_TX4_DESKEW_PER_2K		FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX9296A_MIPI_TX4_DESKEW_PER_AUTO	BIT(7)

#define MAX9296A_MIPI_TX10(x)			(0x40a + (x) * 0x40)
#define MAX9296A_MIPI_TX10_CSI2_LANE_CNT	GENMASK(7, 6)
#define MAX9296A_MIPI_TX10_CSI2_CPHY_EN		BIT(5)

#define MAX9296A_MIPI_TX11(p)			(0x40b + (p) * 0x40)
#define MAX9296A_MIPI_TX12(p)			(0x40c + (p) * 0x40)

#define MAX9296A_MIPI_TX13(p, x)		(0x40d + (p) * 0x40 + (x) * 0x2)
#define MAX9296A_MIPI_TX13_MAP_SRC_DT		GENMASK(5, 0)
#define MAX9296A_MIPI_TX13_MAP_SRC_VC		GENMASK(7, 6)

#define MAX9296A_MIPI_TX14(p, x)		(0x40e + (p) * 0x40 + (x) * 0x2)
#define MAX9296A_MIPI_TX14_MAP_DST_DT		GENMASK(5, 0)
#define MAX9296A_MIPI_TX14_MAP_DST_VC		GENMASK(7, 6)

#define MAX9296A_MIPI_TX45(p, x)		(0x42d + (p) * 0x40 + (x) / 4)
#define MAX9296A_MIPI_TX45_MAP_DPHY_DEST(x)	(GENMASK(1, 0) << (2 * ((x) % 4)))

#define MAX9296A_MIPI_TX51(x)			(0x433 + (x) * 0x40)
#define MAX9296A_MIPI_TX51_ALT_MEM_MAP_12	BIT(0)
#define MAX9296A_MIPI_TX51_ALT_MEM_MAP_8	BIT(1)
#define MAX9296A_MIPI_TX51_ALT_MEM_MAP_10	BIT(2)
#define MAX9296A_MIPI_TX51_ALT2_MEM_MAP_8	BIT(4)

#define MAX9296A_MIPI_TX52(x)			(0x434 +  (x) * 0x40)
#define MAX9296A_MIPI_TX52_TUN_DEST		BIT(1)
#define MAX9296A_MIPI_TX52_TUN_EN		BIT(0)

#define MAX9296A_GMSL1_EN			0xf00
#define MAX9296A_GMSL1_EN_LINK_EN		GENMASK(1, 0)

#define MAX9296A_RLMS3E(x)			(0x143e + (x) * 0x100)
#define MAX9296A_RLMS3F(x)			(0x143f + (x) * 0x100)
#define MAX9296A_RLMS49(x)			(0x1449 + (x) * 0x100)
#define MAX9296A_RLMS7E(x)			(0x147e + (x) * 0x100)
#define MAX9296A_RLMS7F(x)			(0x147f + (x) * 0x100)
#define MAX9296A_RLMSA3(x)			(0x14a3 + (x) * 0x100)
#define MAX9296A_RLMSA5(x)			(0x14a5 + (x) * 0x100)
#define MAX9296A_RLMSD8(x)			(0x14d8 + (x) * 0x100)

#define MAX9296A_DPLL_0(x)			(0x1c00 + (x) * 0x100)
#define MAX9296A_DPLL_0_CONFIG_SOFT_RST_N	BIT(0)

#define field_get(mask, val) (((val) & (mask)) >> __ffs(mask))
#define field_prep(mask, val) (((val) << __ffs(mask)) & (mask))

#define MAX9296A_PIPES_NUM		4
#define MAX9296A_PHYS_NUM		2

static const struct regmap_config max9296a_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
};

struct max9296a_priv {
	struct max_des des;
	const struct max9296a_chip_info *info;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct gpio_desc *gpiod_pwdn;
};

struct max9296a_chip_info {
	unsigned int max_register;
	enum max_gmsl_version versions;
	unsigned int num_pipes;
	unsigned int pipe_hw_ids[MAX9296A_PIPES_NUM];
	unsigned int phy_hw_ids[MAX9296A_PIPES_NUM];
	unsigned int num_phys;
	unsigned int num_links;
	struct max_phys_configs phys_configs;
	bool use_atr;
	bool has_per_link_reset;
	bool phy0_lanes_0_1_on_second_phy;
	bool polarity_on_physical_lanes;
	bool needs_single_link_version;
	bool needs_unique_stream_id;
	bool supports_cphy;
	bool supports_phy_log;
	bool adjust_rlms;
	bool fix_tx_ids;

	int (*set_pipe_stream_id)(struct max_des *des, struct max_des_pipe *pipe,
				  unsigned int stream_id);
	int (*set_pipe_enable)(struct max_des *des, struct max_des_pipe *pipe,
			       bool enable);
	int (*set_pipe_phy)(struct max_des *des, struct max_des_pipe *pipe,
			    struct max_des_phy *phy);
	int (*set_pipe_tunnel_enable)(struct max_des *des, struct max_des_pipe *pipe,
				      bool enable);
};

#define des_to_priv(_des) \
	container_of(_des, struct max9296a_priv, des)

static int max9296a_wait_for_device(struct max9296a_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 10; i++) {
		unsigned int val;

		ret = regmap_read(priv->regmap, MAX9296A_REG0, &val);
		if (!ret && val)
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

	ret = regmap_set_bits(priv->regmap, MAX9296A_CTRL0,
			      MAX9296A_CTRL0_RESET_ALL);
	if (ret)
		return ret;

	msleep(100);

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

static unsigned int max9296a_phy_id(struct max9296a_priv *priv,
				    struct max_des_phy *phy)
{
	return priv->info->phy_hw_ids[phy->index];
}

static int max9296a_reg_read(struct max_des *des, unsigned int reg,
			     unsigned int *val)
{
	struct max9296a_priv *priv = des_to_priv(des);

	return regmap_read(priv->regmap, reg, val);
}

static int max9296a_reg_write(struct max_des *des, unsigned int reg,
			      unsigned int val)
{
	struct max9296a_priv *priv = des_to_priv(des);

	return regmap_write(priv->regmap, reg, val);
}

static int max9626a_log_pipe_status(struct max_des *des,
				    struct max_des_pipe *pipe, const char *name)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, MAX9296A_VPRBS(index), &val);
	if (ret)
		return ret;

	pr_info("%s: \tvideo_lock: %u\n", name,
		!!(val & MAX9296A_VPRBS_VIDEO_LOCK));

	return 0;
}

static int max9296a_log_phy_status(struct max_des *des,
				   struct max_des_phy *phy, const char *name)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = phy->index;
	unsigned int val;
	int ret;

	if (!priv->info->supports_phy_log)
		return 0;

	ret = regmap_read(priv->regmap, MAX9296A_MIPI_PHY18, &val);
	if (ret)
		return ret;

	pr_info("%s: \tcsi2_pkt_cnt: %lu\n", name,
		field_get(MAX9296A_MIPI_PHY18_CSI2_TX_PKT_CNT(index), val));

	ret = regmap_read(priv->regmap, MAX9296A_MIPI_PHY20, &val);
	if (ret)
		return ret;

	pr_info("%s: \tphy_pkt_cnt: %lu\n", name,
		field_get(MAX9296A_MIPI_PHY20_PHY_PKT_CNT(index), val));

	return 0;
}

static int max9296a_set_enable(struct max_des *des, bool enable)
{
	struct max9296a_priv *priv = des_to_priv(des);

	return regmap_assign_bits(priv->regmap, MAX9296A_BACKTOP12,
				  MAX9296A_BACKTOP12_CSI_OUT_EN, enable);
}

static int max9296a_init_phy(struct max_des *des, struct max_des_phy *phy)
{
	struct max9296a_priv *priv = des_to_priv(des);
	bool is_cphy = phy->bus_type == V4L2_MBUS_CSI2_CPHY;
	unsigned int num_data_lanes = phy->mipi.num_data_lanes;
	unsigned int dpll_freq = phy->link_frequency * 2;
	unsigned int num_hw_data_lanes;
	unsigned int hw_index = max9296a_phy_id(priv, phy);
	unsigned int index = phy->index;
	unsigned int used_data_lanes = 0;
	unsigned int val;
	unsigned int i;
	int ret;

	if (is_cphy && !priv->info->supports_cphy) {
		dev_err(priv->dev, "CPHY not supported\n");
		return -EINVAL;
	}

	num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

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
	 *
	 * MAX96714 only has two PHYs which cannot support single-PHY configurations.
	 * Clock is always on the master PHY, first lanes are on PHY 0, even if
	 * PHY 1 is the master PHY.
	 *
	 * PHY 0 + 1
	 * CLK = PHY 1
	 * PHY0 Lane 0 = D0
	 * PHY0 Lane 1 = D1
	 * PHY1 Lane 0 = D2
	 * PHY1 Lane 1 = D3
	 */

	/* Configure a lane count. */
	ret = regmap_update_bits(priv->regmap, MAX9296A_MIPI_TX10(hw_index),
				 MAX9296A_MIPI_TX10_CSI2_LANE_CNT,
				 FIELD_PREP(MAX9296A_MIPI_TX10_CSI2_LANE_CNT,
					    num_data_lanes - 1));
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX10(hw_index),
				 MAX9296A_MIPI_TX10_CSI2_CPHY_EN, is_cphy);
	if (ret)
		return ret;

	/* Configure lane mapping. */
	/*
	 * The lane of each PHY can be mapped to physical lanes 0, 1, 2, and 3.
	 * This mapping is exclusive, multiple lanes, even if unused cannot be
	 * mapped to the same physical lane.
	 * Each lane mapping is represented as two bits.
	 */
	val = 0;
	for (i = 0; i < num_hw_data_lanes ; i++) {
		unsigned int map;

		if (i < num_data_lanes)
			map = phy->mipi.data_lanes[i] - 1;
		else
			map = ffz(used_data_lanes);

		val |= map << (i * 2);
		used_data_lanes |= BIT(map);
	}

	if (phy->index == 0 && priv->info->phy0_lanes_0_1_on_second_phy)
		val = ((val & 0xf) << 4) | ((val >> 4) & 0xf);

	ret = regmap_update_bits(priv->regmap, MAX9296A_MIPI_PHY3(index),
				 MAX9296A_MIPI_PHY3_PHY_LANE_MAP_4,
				 FIELD_PREP(MAX9296A_MIPI_PHY3_PHY_LANE_MAP_4, val));
	if (ret)
		return ret;

	/*
	 * Configure lane polarity.
	 *
	 * PHY 0 and 1 are on register 0x335.
	 * PHY 2 and 3 are on register 0x336.
	 *
	 * Each PHY has 3 bits of polarity configuration.
	 *
	 * On MAX9296A, each bit represents the lane polarity of logical lanes.
	 * Each of these lanes can be mapped to any physical lane.
	 * 0th bit is for lane 0.
	 * 1st bit is for lane 1.
	 * 2nd bit is for clock lane.
	 *
	 * On MAX96714, each bit represents the lane polarity of physical lanes.
	 * 0th bit for physical lane 0.
	 * 1st bit for physical lane 1.
	 * 2nd bit for clock lane of PHY 0, the slave PHY, which is unused.
	 *
	 * 3rd bit for physical lane 2.
	 * 4th bit for physical lane 3.
	 * 5th bit for clock lane of PHY 1, the master PHY.
	 */

	val = 0;
	for (i = 0; i < num_data_lanes; i++) {
		unsigned int map;

		if (!phy->mipi.lane_polarities[i + 1])
			continue;

		/*
		 * The numbers inside the data_lanes array specify the hardware
		 * lane each logical lane maps to.
		 * If polarity is set for the physical lanes, retrieve the
		 * physical lane matching the logical lane from data_lanes.
		 * Otherwise, when polarity is set for the logical lanes
		 * the index of the polarity can be used.
		 */

		if (priv->info->polarity_on_physical_lanes)
			map = phy->mipi.data_lanes[i] - 1;
		else
			map = i;

		val |= BIT(map);
	}

	if (phy->index == 0 && priv->info->phy0_lanes_0_1_on_second_phy)
		val = ((val & 0x3) << 2) | ((val >> 2) & 0x3);

	ret = regmap_update_bits(priv->regmap, MAX9296A_MIPI_PHY5(index),
				 MAX9296A_MIPI_PHY5_PHY_POL_MAP_0_1 |
				 MAX9296A_MIPI_PHY5_PHY_POL_MAP_2_3,
				 FIELD_PREP(MAX9296A_MIPI_PHY5_PHY_POL_MAP_0_1, val) |
				 FIELD_PREP(MAX9296A_MIPI_PHY5_PHY_POL_MAP_2_3, val >> 2));
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_MIPI_PHY5(index),
				 MAX9296A_MIPI_PHY5_PHY_POL_MAP_CLK(index),
				 phy->mipi.lane_polarities[0]);
	if (ret)
		return ret;

	/* Put DPLL block into reset. */
	ret = regmap_clear_bits(priv->regmap, MAX9296A_DPLL_0(hw_index),
				MAX9296A_DPLL_0_CONFIG_SOFT_RST_N);
	if (ret)
		return ret;

	/* Set DPLL frequency. */
	ret = regmap_update_bits(priv->regmap, MAX9296A_BACKTOP22(index),
				 MAX9296A_BACKTOP22_PHY_CSI_TX_DPLL,
				 FIELD_PREP(MAX9296A_BACKTOP22_PHY_CSI_TX_DPLL,
					    div_u64(dpll_freq, 100000000)));
	if (ret)
		return ret;

	/* Enable DPLL frequency. */
	ret = regmap_set_bits(priv->regmap, MAX9296A_BACKTOP22(index),
			      MAX9296A_BACKTOP22_PHY_CSI_TX_DPLL_EN);
	if (ret)
		return ret;

	/* Pull DPLL block out of reset. */
	ret = regmap_set_bits(priv->regmap, MAX9296A_DPLL_0(hw_index),
			      MAX9296A_DPLL_0_CONFIG_SOFT_RST_N);
	if (ret)
		return ret;

	if (dpll_freq > 1500000000ull) {
		/* Enable initial deskew with 2 x 32k UI. */
		ret = regmap_write(priv->regmap, MAX9296A_MIPI_TX3(hw_index),
				   MAX9296A_MIPI_TX3_DESKEW_INIT_AUTO |
				   MAX9296A_MIPI_TX3_DESKEW_INIT_8X32K);
		if (ret)
			return ret;

		/* Enable periodic deskew with 2 x 1k UI.. */
		ret = regmap_write(priv->regmap, MAX9296A_MIPI_TX4(hw_index),
				   MAX9296A_MIPI_TX4_DESKEW_PER_AUTO |
				   MAX9296A_MIPI_TX4_DESKEW_PER_2K);
		if (ret)
			return ret;
	} else {
		/* Disable initial deskew. */
		ret = regmap_write(priv->regmap, MAX9296A_MIPI_TX3(hw_index), 0x0);
		if (ret)
			return ret;

		/* Disable periodic deskew. */
		ret = regmap_write(priv->regmap, MAX9296A_MIPI_TX4(hw_index), 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int max9296a_set_phy_mode(struct max_des *des, struct max_des_phy *phy,
				 struct max_des_phy_mode *mode)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int phy_id = max9296a_phy_id(priv, phy);
	int ret;

	/* Set alternate memory map modes. */
	ret = regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX51(phy_id),
				 MAX9296A_MIPI_TX51_ALT_MEM_MAP_12,
				 mode->alt_mem_map12);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX51(phy_id),
				 MAX9296A_MIPI_TX51_ALT_MEM_MAP_8,
				 mode->alt_mem_map8);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX51(phy_id),
				 MAX9296A_MIPI_TX51_ALT_MEM_MAP_10,
				 mode->alt_mem_map10);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX51(phy_id),
				 MAX9296A_MIPI_TX51_ALT2_MEM_MAP_8,
				 mode->alt2_mem_map8);
	if (ret)
		return ret;

	return 0;
}

static int max9296a_set_phy_active(struct max_des *des, struct max_des_phy *phy,
				   bool enable)
{
	struct max9296a_priv *priv = des_to_priv(des);

	return regmap_assign_bits(priv->regmap, MAX9296A_MIPI_PHY2,
				  MAX9296A_MIPI_PHY2_PHY_STDBY_N(phy->index), enable);
}

static int max9296a_set_pipe_remap(struct max_des *des,
				   struct max_des_pipe *pipe,
				   unsigned int i,
				   struct max_des_remap *remap)
{
	struct max9296a_priv *priv = des_to_priv(des);
	struct max_des_phy *phy = &des->phys[remap->phy];
	unsigned int phy_id = max9296a_phy_id(priv, phy);
	unsigned int index = max9296a_pipe_id(priv, pipe);
	int ret;

	/* Set source Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	ret = regmap_write(priv->regmap, MAX9296A_MIPI_TX13(index, i),
			   FIELD_PREP(MAX9296A_MIPI_TX13_MAP_SRC_DT,
				      remap->from_dt) |
			   FIELD_PREP(MAX9296A_MIPI_TX13_MAP_SRC_VC,
				      remap->from_vc));
	if (ret)
		return ret;

	/* Set destination Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	ret = regmap_write(priv->regmap, MAX9296A_MIPI_TX14(index, i),
			   FIELD_PREP(MAX9296A_MIPI_TX14_MAP_DST_DT,
				      remap->to_dt) |
			   FIELD_PREP(MAX9296A_MIPI_TX14_MAP_DST_VC,
				      remap->to_vc));
	if (ret)
		return ret;

	/* Set destination PHY. */
	return regmap_update_bits(priv->regmap, MAX9296A_MIPI_TX45(index, i),
				  MAX9296A_MIPI_TX45_MAP_DPHY_DEST(i),
				  field_prep(MAX9296A_MIPI_TX45_MAP_DPHY_DEST(i),
					     phy_id));
}

static int max9296a_set_pipe_remaps_enable(struct max_des *des,
					   struct max_des_pipe *pipe,
					   unsigned int mask)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);
	int ret;

	ret = regmap_write(priv->regmap, MAX9296A_MIPI_TX11(index), mask);
	if (ret)
		return ret;

	return regmap_write(priv->regmap, MAX9296A_MIPI_TX12(index), mask >> 8);
}

static int max9296a_set_pipe_enable(struct max_des *des, struct max_des_pipe *pipe,
				    bool enable)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);

	return regmap_assign_bits(priv->regmap, MAX9296A_REG2,
				  MAX9296A_REG2_VID_EN(index), enable);
}

static int max96714_set_pipe_enable(struct max_des *des, struct max_des_pipe *pipe,
				    bool enable)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);

	return regmap_assign_bits(priv->regmap, MAX9296A_VIDEO_PIPE_EN,
				  MAX9296A_VIDEO_PIPE_EN_MASK(index - 1), enable);
}

static int max96714_set_pipe_tunnel_enable(struct max_des *des,
					   struct max_des_pipe *pipe, bool enable)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);

	return regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX52(index),
				  MAX9296A_MIPI_TX52_TUN_EN, enable);
}

static int max9296a_set_pipe_stream_id(struct max_des *des, struct max_des_pipe *pipe,
				       unsigned int stream_id)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);

	return regmap_update_bits(priv->regmap, MAX9296A_RX50(index), MAX9296A_RX50_STR_SEL,
				  FIELD_PREP(MAX9296A_RX50_STR_SEL, pipe->stream_id));
}

static int max96714_set_pipe_stream_id(struct max_des *des, struct max_des_pipe *pipe,
				       unsigned int stream_id)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;

	return regmap_update_bits(priv->regmap, MAX9296A_VIDEO_PIPE_SEL,
				  MAX9296A_VIDEO_PIPE_SEL_STREAM(index),
				  field_prep(MAX9296A_VIDEO_PIPE_SEL_STREAM(index),
					     stream_id));
}

static int max96716a_set_pipe_phy(struct max_des *des,
				  struct max_des_pipe *pipe,
				  struct max_des_phy *phy)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);

	return regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX52(index),
				  MAX9296A_MIPI_TX52_TUN_DEST, phy->index);
}

static int max9296a_set_pipe_mode(struct max_des *des,
				  struct max_des_pipe *pipe,
				  struct max_des_pipe_mode *mode)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = max9296a_pipe_id(priv, pipe);
	int ret;

	/* Set 8bit double mode. */
	ret = regmap_assign_bits(priv->regmap, MAX9296A_BACKTOP21,
				 MAX9296A_BACKTOP21_BPP8DBL(index), mode->dbl8);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_BACKTOP24,
				 MAX9296A_BACKTOP24_BPP8DBL_MODE(index),
				 mode->dbl8mode);
	if (ret)
		return ret;

	/* Set 10bit double mode. */
	ret = regmap_assign_bits(priv->regmap, MAX9296A_BACKTOP32,
				 MAX9296A_BACKTOP32_BPP10DBL(index), mode->dbl10);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_BACKTOP32,
				 MAX9296A_BACKTOP32_BPP10DBL_MODE(index),
				 mode->dbl10mode);
	if (ret)
		return ret;

	/* Set 12bit double mode. */
	/* TODO: check support for double mode on MAX96714. */
	return regmap_assign_bits(priv->regmap, MAX9296A_BACKTOP33,
				  MAX9296A_BACKTOP32_BPP12DBL(index), mode->dbl12);
}

static int max9296a_reset_link(struct max9296a_priv *priv, unsigned int index)
{
	unsigned int reg, mask;

	if (index == 0) {
		reg = MAX9296A_CTRL0;
		mask = MAX9296A_CTRL0_RESET_ONESHOT;
	} else {
		reg = MAX9296A_CTRL2;
		mask = MAX9296A_CTRL2_RESET_ONESHOT_B;
	}

	return regmap_set_bits(priv->regmap, reg, mask);
}

static int max9296a_init_link_rlms(struct max9296a_priv *priv,
				   struct max_des_link *link)
{
	unsigned int index = link->index;
	int ret;

	/*
	 * These settings are described as required on datasheet page 53
	 * for MAX96714.
	 */

	ret = regmap_write(priv->regmap, MAX9296A_RLMS3E(index), 0xfd);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX9296A_RLMS3F(index), 0x3d);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX9296A_RLMS49(index), 0xf5);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX9296A_RLMS7E(index), 0xa8);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX9296A_RLMS7F(index), 0x68);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX9296A_RLMSA3(index), 0x30);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX9296A_RLMSA5(index), 0x70);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX9296A_RLMSD8(index), 0x07);
	if (ret)
		return ret;

	return max9296a_reset_link(priv, link->index);
}

static int max9296a_init_link(struct max_des *des, struct max_des_link *link)
{
	struct max9296a_priv *priv = des_to_priv(des);
	int ret;

	if (priv->info->adjust_rlms) {
		ret = max9296a_init_link_rlms(priv, link);
		if (ret)
			return ret;
	}

	return 0;
}

static int max9296a_select_links(struct max_des *des, unsigned int mask)
{
	struct max9296a_priv *priv = des_to_priv(des);
	int ret;

	if (priv->info->num_links == 1)
		return 0;

	if (!mask) {
		dev_err(priv->dev, "Disable all links unsupported\n");
		return -EINVAL;
	}

	ret = regmap_update_bits(priv->regmap, MAX9296A_GMSL1_EN,
				 MAX9296A_GMSL1_EN_LINK_EN,
				 FIELD_PREP(MAX9296A_GMSL1_EN_LINK_EN, mask));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, MAX9296A_CTRL0,
				 MAX9296A_CTRL0_AUTO_LINK |
				 MAX9296A_CTRL0_LINK_CFG |
				 MAX9296A_CTRL0_RESET_ONESHOT,
				 FIELD_PREP(MAX9296A_CTRL0_LINK_CFG, mask) |
				 FIELD_PREP(MAX9296A_CTRL0_RESET_ONESHOT, 1));
	if (ret)
		return ret;

	if (priv->info->has_per_link_reset) {
		ret = max9296a_reset_link(priv, 1);
		if (ret)
			return ret;
	}

	msleep(200);

	return 0;
}

static int max9296a_set_link_version(struct max_des *des,
				     struct max_des_link *link,
				     enum max_gmsl_version version)
{
	struct max9296a_priv *priv = des_to_priv(des);
	unsigned int index = link->index;
	bool gmsl3_en = version == MAX_GMSL_3;
	unsigned int reg, mask, val;
	int ret;

	if (priv->info->needs_single_link_version)
		index = 0;

	if (index == 0) {
		reg = MAX9296A_REG1;
		mask = MAX9296A_REG1_RX_RATE_A;
	} else {
		reg = MAX9296A_REG4;
		mask = MAX9296A_REG4_RX_RATE_B;
	}

	if (version == MAX_GMSL_3)
		val = MAX9296A_REG1_RX_RATE_12Gbps;
	else if (version == MAX_GMSL_2_6Gbps)
		val = MAX9296A_REG1_RX_RATE_6Gbps;
	else
		val = MAX9296A_REG1_RX_RATE_3Gbps;

	ret = regmap_update_bits(priv->regmap, reg, mask, field_prep(mask, val));
	if (ret)
		return ret;

	if (!(priv->info->versions & BIT(MAX_GMSL_3)))
		return 0;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_MIPI_TX0(index),
				 MAX9296A_MIPI_TX0_RX_FEC_EN, gmsl3_en);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX9296A_REG6,
				 MAX9296A_REG6_GMSL2_X(index), !gmsl3_en);
	if (ret)
		return ret;

	return regmap_assign_bits(priv->regmap, MAX9296A_REG4,
				  MAX9296A_REG4_GMSL3_X(index), gmsl3_en);
}

static const struct max_des_ops max9296a_ops = {
	.num_remaps_per_pipe = 16,
	.reg_read = max9296a_reg_read,
	.reg_write = max9296a_reg_write,
	.log_pipe_status = max9626a_log_pipe_status,
	.log_phy_status = max9296a_log_phy_status,
	.set_enable = max9296a_set_enable,
	.init_phy = max9296a_init_phy,
	.set_phy_mode = max9296a_set_phy_mode,
	.set_phy_active = max9296a_set_phy_active,
	.set_pipe_remap = max9296a_set_pipe_remap,
	.set_pipe_remaps_enable = max9296a_set_pipe_remaps_enable,
	.set_pipe_mode = max9296a_set_pipe_mode,
	.init_link = max9296a_init_link,
	.select_links = max9296a_select_links,
	.set_link_version = max9296a_set_link_version,
};

static int max9296a_probe(struct i2c_client *client)
{
	struct regmap_config i2c_regmap = max9296a_i2c_regmap;
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

	i2c_regmap.max_register = priv->info->max_register;
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

	*ops = max9296a_ops;

	ops->versions = priv->info->versions;
	ops->needs_single_link_version = priv->info->needs_single_link_version;
	ops->needs_unique_stream_id = priv->info->needs_unique_stream_id;
	ops->fix_tx_ids = priv->info->fix_tx_ids;
	ops->num_phys = priv->info->num_phys;
	ops->num_pipes = priv->info->num_pipes;
	ops->num_links = priv->info->num_links;
	ops->phys_configs = priv->info->phys_configs;
	ops->set_pipe_enable = priv->info->set_pipe_enable;
	ops->set_pipe_stream_id = priv->info->set_pipe_stream_id;
	ops->set_pipe_phy = priv->info->set_pipe_phy;
	ops->set_pipe_tunnel_enable = priv->info->set_pipe_tunnel_enable;
	ops->use_atr = priv->info->use_atr;
	priv->des.ops = ops;

	ret = max9296a_reset(priv);
	if (ret)
		return ret;

	return max_des_probe(client, &priv->des);
}

static void max9296a_remove(struct i2c_client *client)
{
	struct max9296a_priv *priv = i2c_get_clientdata(client);

	max_des_remove(&priv->des);

	gpiod_set_value_cansleep(priv->gpiod_pwdn, 1);
}

static const struct max_phys_config max9296a_phys_configs[] = {
	{ { 4, 4 } },
};

static const struct max_phys_config max96714_phys_configs[] = {
	{ { 4 } },
};

static const struct max9296a_chip_info max9296a_info = {
	.max_register = 0x1f00,
	.versions = BIT(MAX_GMSL_2_3Gbps) |
		    BIT(MAX_GMSL_2_6Gbps),
	.set_pipe_stream_id = max9296a_set_pipe_stream_id,
	.set_pipe_enable = max9296a_set_pipe_enable,
	.needs_single_link_version = true,
	.needs_unique_stream_id = true,
	.use_atr = true,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max9296a_phys_configs),
		.configs = max9296a_phys_configs,
	},
	.phy0_lanes_0_1_on_second_phy = true,
	.fix_tx_ids = true,
	.num_pipes = 4,
	.pipe_hw_ids = { 0, 1, 2, 3 },
	.num_phys = 2,
	.phy_hw_ids = { 1, 2 },
	.num_links = 2,
};

static const struct max9296a_chip_info max96714_info = {
	.max_register = 0x5011,
	.versions = BIT(MAX_GMSL_2_3Gbps) |
		    BIT(MAX_GMSL_2_6Gbps),
	.set_pipe_stream_id = max96714_set_pipe_stream_id,
	.set_pipe_enable = max96714_set_pipe_enable,
	.set_pipe_tunnel_enable = max96714_set_pipe_tunnel_enable,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max96714_phys_configs),
		.configs = max96714_phys_configs,
	},
	.polarity_on_physical_lanes = true,
	.supports_phy_log = true,
	.adjust_rlms = true,
	.num_pipes = 1,
	.pipe_hw_ids = { 1 },
	.num_phys = 1,
	.phy_hw_ids = { 1 },
	.num_links = 1,
};

static const struct max9296a_chip_info max96714f_info = {
	.max_register = 0x5011,
	.versions = BIT(MAX_GMSL_2_3Gbps),
	.set_pipe_stream_id = max96714_set_pipe_stream_id,
	.set_pipe_enable = max96714_set_pipe_enable,
	.set_pipe_tunnel_enable = max96714_set_pipe_tunnel_enable,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max96714_phys_configs),
		.configs = max96714_phys_configs,
	},
	.polarity_on_physical_lanes = true,
	.supports_phy_log = true,
	.adjust_rlms = true,
	.num_pipes = 1,
	.pipe_hw_ids = { 1 },
	.num_phys = 1,
	.phy_hw_ids = { 1 },
	.num_links = 1,
};

static const struct max9296a_chip_info max96716a_info = {
	.max_register = 0x52d6,
	.versions = BIT(MAX_GMSL_2_3Gbps) |
		    BIT(MAX_GMSL_2_6Gbps),
	.set_pipe_stream_id = max96714_set_pipe_stream_id,
	.set_pipe_enable = max96714_set_pipe_enable,
	.set_pipe_phy = max96716a_set_pipe_phy,
	.set_pipe_tunnel_enable = max96714_set_pipe_tunnel_enable,
	.use_atr = true,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max9296a_phys_configs),
		.configs = max9296a_phys_configs,
	},
	.has_per_link_reset = true,
	.phy0_lanes_0_1_on_second_phy = true,
	.supports_cphy = true,
	.supports_phy_log = true,
	.num_pipes = 2,
	.pipe_hw_ids = { 1, 2 },
	.num_phys = 2,
	.phy_hw_ids = { 1, 2 },
	.num_links = 2,
};

static const struct max9296a_chip_info max96792a_info = {
	.max_register = 0x52d6,
	.versions = BIT(MAX_GMSL_2_3Gbps) |
		    BIT(MAX_GMSL_2_6Gbps) |
		    BIT(MAX_GMSL_3),
	.set_pipe_stream_id = max96714_set_pipe_stream_id,
	.set_pipe_enable = max96714_set_pipe_enable,
	.set_pipe_phy = max96716a_set_pipe_phy,
	.set_pipe_tunnel_enable = max96714_set_pipe_tunnel_enable,
	.use_atr = true,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max9296a_phys_configs),
		.configs = max9296a_phys_configs,
	},
	.has_per_link_reset = true,
	.phy0_lanes_0_1_on_second_phy = true,
	.supports_cphy = true,
	.supports_phy_log = true,
	.num_pipes = 2,
	.pipe_hw_ids = { 1, 2 },
	.num_phys = 2,
	.phy_hw_ids = { 1, 2 },
	.num_links = 2,
};

static const struct of_device_id max9296a_of_table[] = {
	{ .compatible = "maxim,max9296a", .data = &max9296a_info },
	{ .compatible = "maxim,max96714", .data = &max96714_info },
	{ .compatible = "maxim,max96714f", .data = &max96714f_info },
	{ .compatible = "maxim,max96714r", .data = &max96714f_info },
	{ .compatible = "maxim,max96716a", .data = &max96716a_info },
	{ .compatible = "maxim,max96792a", .data = &max96792a_info },
	{ },
};
MODULE_DEVICE_TABLE(of, max9296a_of_table);

static struct i2c_driver max9296a_i2c_driver = {
	.driver	= {
		.name = "max9296a",
		.of_match_table	= of_match_ptr(max9296a_of_table),
	},
	.probe = max9296a_probe,
	.remove = max9296a_remove,
};

module_i2c_driver(max9296a_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX9296A Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
