// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2020 NXP
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define LVDS_CTRL		0x128
#define SPARE_IN(n)		(((n) & 0x7) << 25)
#define SPARE_IN_MASK		0xe000000
#define TEST_RANDOM_NUM_EN	BIT(24)
#define TEST_MUX_SRC(n)		(((n) & 0x3) << 22)
#define TEST_MUX_SRC_MASK	0xc00000
#define TEST_EN			BIT(21)
#define TEST_DIV4_EN		BIT(20)
#define VBG_ADJ(n)		(((n) & 0x7) << 17)
#define VBG_ADJ_MASK		0xe0000
#define SLEW_ADJ(n)		(((n) & 0x7) << 14)
#define SLEW_ADJ_MASK		0x1c000
#define CC_ADJ(n)		(((n) & 0x7) << 11)
#define CC_ADJ_MASK		0x3800
#define CM_ADJ(n)		(((n) & 0x7) << 8)
#define CM_ADJ_MASK		0x700
#define PRE_EMPH_ADJ(n)		(((n) & 0x7) << 5)
#define PRE_EMPH_ADJ_MASK	0xe0
#define PRE_EMPH_EN		BIT(4)
#define HS_EN			BIT(3)
#define BG_EN			BIT(2)
#define CH_EN(id)		BIT(id)

struct imx8mp_lvds_phy {
	struct phy *phy;
	unsigned int id;
};

struct imx8mp_lvds_phy_priv {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
	struct clk *apb_clk;
	struct imx8mp_lvds_phy *phys[2];
};

static inline unsigned int phy_read(struct phy *phy, unsigned int reg)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	unsigned int val;

	regmap_read(priv->regmap, reg, &val);

	return val;
}

static inline void
phy_write(struct phy *phy, unsigned int reg, unsigned int value)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);

	regmap_write(priv->regmap, reg, value);
}

static int imx8mp_lvds_phy_init(struct phy *phy)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);

	clk_prepare_enable(priv->apb_clk);

	mutex_lock(&priv->lock);
	phy_write(phy, LVDS_CTRL,
			CC_ADJ(0x2) | PRE_EMPH_EN | PRE_EMPH_ADJ(0x3));
	mutex_unlock(&priv->lock);

	clk_disable_unprepare(priv->apb_clk);

	return 0;
}

static int imx8mp_lvds_phy_power_on(struct phy *phy)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	struct imx8mp_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	unsigned int id = lvds_phy->id;
	unsigned int val;
	bool bg_en;

	clk_prepare_enable(priv->apb_clk);

	mutex_lock(&priv->lock);
	val = phy_read(phy, LVDS_CTRL);
	bg_en = !!(val & BG_EN);
	val |= BG_EN;
	phy_write(phy, LVDS_CTRL, val);
	mutex_unlock(&priv->lock);

	/* Wait 15us to make sure the bandgap to be stable. */
	if (!bg_en)
		usleep_range(15, 20);

	mutex_lock(&priv->lock);
	val = phy_read(phy, LVDS_CTRL);
	val |= CH_EN(id);
	phy_write(phy, LVDS_CTRL, val);
	mutex_unlock(&priv->lock);

	clk_disable_unprepare(priv->apb_clk);

	/* Wait 5us to ensure the phy be settling. */
	usleep_range(5, 10);

	return 0;
}

static int imx8mp_lvds_phy_power_off(struct phy *phy)
{
	struct imx8mp_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	struct imx8mp_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	unsigned int id = lvds_phy->id;
	unsigned int val;

	clk_prepare_enable(priv->apb_clk);

	mutex_lock(&priv->lock);
	val = phy_read(phy, LVDS_CTRL);
	val &= ~BG_EN;
	phy_write(phy, LVDS_CTRL, val);

	val = phy_read(phy, LVDS_CTRL);
	val &= ~CH_EN(id);
	phy_write(phy, LVDS_CTRL, val);
	mutex_unlock(&priv->lock);

	clk_disable_unprepare(priv->apb_clk);

	return 0;
}

static const struct phy_ops imx8mp_lvds_phy_ops = {
	.init = imx8mp_lvds_phy_init,
	.power_on = imx8mp_lvds_phy_power_on,
	.power_off = imx8mp_lvds_phy_power_off,
	.owner = THIS_MODULE,
};

static int imx8mp_lvds_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct phy_provider *phy_provider;
	struct imx8mp_lvds_phy_priv *priv;
	struct imx8mp_lvds_phy *lvds_phy;
	struct phy *phy;
	u32 phy_id;
	int ret;

	if (!np)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = syscon_regmap_lookup_by_phandle(np, "gpr");
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "failed to get regmap\n");
		return PTR_ERR(priv->regmap);
	}

	priv->dev = dev;

	priv->apb_clk = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->apb_clk)) {
		dev_err(dev, "cannot get apb clock\n");
		return PTR_ERR(priv->apb_clk);
	}

	mutex_init(&priv->lock);
	dev_set_drvdata(dev, priv);

	pm_runtime_enable(dev);

	for_each_available_child_of_node(np, child) {
		if (of_property_read_u32(child, "reg", &phy_id)) {
			dev_err(dev, "missing reg property in node %s\n",
				child->name);
			ret = -EINVAL;
			goto put_child;
		}

		if (phy_id >= ARRAY_SIZE(priv->phys)) {
			dev_err(dev, "invalid reg in node %s\n", child->name);
			ret = -EINVAL;
			goto put_child;
		}

		if (priv->phys[phy_id]) {
			dev_err(dev, "duplicated phy id: %u\n", phy_id);
			ret = -EINVAL;
			goto put_child;
		}

		lvds_phy = devm_kzalloc(dev, sizeof(*lvds_phy), GFP_KERNEL);
		if (!lvds_phy) {
			ret = -ENOMEM;
			goto put_child;
		}

		phy = devm_phy_create(dev, child, &imx8mp_lvds_phy_ops);
		if (IS_ERR(phy)) {
			dev_err(dev, "failed to create phy\n");
			ret = PTR_ERR(phy);
			goto put_child;
		}

		lvds_phy->phy = phy;
		lvds_phy->id = phy_id;
		priv->phys[phy_id] = lvds_phy;

		phy_set_drvdata(phy, lvds_phy);
	}

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		pm_runtime_disable(dev);
		return PTR_ERR(phy_provider);
	}

	return 0;

put_child:
	of_node_put(child);
	pm_runtime_disable(dev);
	return ret;
}

static int imx8mp_lvds_phy_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id imx8mp_lvds_phy_of_match[] = {
	{ .compatible = "fsl,imx8mp-lvds-phy" },
	{}
};
MODULE_DEVICE_TABLE(of, imx8mp_lvds_phy_of_match);

static struct platform_driver imx8mp_lvds_phy_driver = {
	.probe	= imx8mp_lvds_phy_probe,
	.remove = imx8mp_lvds_phy_remove,
	.driver = {
		.name = "imx8mp-lvds-phy",
		.of_match_table	= imx8mp_lvds_phy_of_match,
	}
};
module_platform_driver(imx8mp_lvds_phy_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX8MP LVDS PHY driver");
MODULE_LICENSE("GPL v2");
