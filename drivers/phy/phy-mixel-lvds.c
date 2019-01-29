/*
 * Copyright 2017-2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mixel-lvds.h>
#include <linux/platform_device.h>

#define SET		0x4
#define CLR		0x8
#define TOG		0xc

#define PHY_CTRL	0x0
#define M(n)		(((n) & 0x3) << 17)
#define M_MASK		0x60000
#define CCM(n)		(((n) & 0x7) << 14)
#define CCM_MASK	0x1c000
#define CA(n)		(((n) & 0x7) << 11)
#define CA_MASK		0x3800
#define TST(n)		(((n) & 0x3f) << 5)
#define TST_MASK	0x7e0
#define CH_EN(id)	BIT(3 + (id))
#define NB		BIT(2)
#define RFB		BIT(1)
#define PD		BIT(0)

#define PHY_STATUS	0x10
#define LOCK		BIT(0)

#define PHY_SS_CTRL	0x20
#define CH_HSYNC_M(id)	BIT(0 + ((id) * 2))
#define CH_VSYNC_M(id)	BIT(1 + ((id) * 2))
#define CH_PHSYNC(id)	BIT(0 + ((id) * 2))
#define CH_PVSYNC(id)	BIT(1 + ((id) * 2))

struct mixel_lvds_phy {
	struct phy *phy;
	unsigned int id;
};

struct mixel_lvds_phy_priv {
	struct device *dev;
	void __iomem *base;
	struct mutex lock;
	struct clk *phy_clk;
	struct mixel_lvds_phy *phys[2];
};

static inline u32 phy_read(struct phy *phy, unsigned int reg)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);

	return readl(priv->base + reg);
}

static inline void phy_write(struct phy *phy, unsigned int reg, u32 value)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);

	writel(value, priv->base + reg);
}

void mixel_phy_lvds_set_phy_speed(struct phy *phy, unsigned long phy_clk_rate)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	u32 val;

	/* assuming NB is zero - 7bits per channel */
	clk_prepare_enable(priv->phy_clk);
	mutex_lock(&priv->lock);
	val = phy_read(phy, PHY_CTRL);
	val &= ~M_MASK;
	if (phy_clk_rate < 44000000)
		val |= M(0x2);
	else if (phy_clk_rate < 90000000)
		val |= M(0x1);
	else
		val |= M(0x0);
	phy_write(phy, PHY_CTRL, val);
	mutex_unlock(&priv->lock);
	clk_disable_unprepare(priv->phy_clk);

	clk_set_rate(priv->phy_clk, phy_clk_rate);
}
EXPORT_SYMBOL_GPL(mixel_phy_lvds_set_phy_speed);

void mixel_phy_lvds_set_hsync_pol(struct phy *phy, bool active_high)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	unsigned int id = lvds_phy->id;
	u32 val;

	clk_prepare_enable(priv->phy_clk);
	mutex_lock(&priv->lock);
	val = phy_read(phy, PHY_SS_CTRL);
	val &= ~CH_HSYNC_M(id);
	if (active_high)
		val |= CH_PHSYNC(id);
	phy_write(phy, PHY_SS_CTRL, val);
	mutex_unlock(&priv->lock);
	clk_disable_unprepare(priv->phy_clk);
}
EXPORT_SYMBOL_GPL(mixel_phy_lvds_set_hsync_pol);

void mixel_phy_lvds_set_vsync_pol(struct phy *phy, bool active_high)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	unsigned int id = lvds_phy->id;
	u32 val;

	clk_prepare_enable(priv->phy_clk);
	mutex_lock(&priv->lock);
	val = phy_read(phy, PHY_SS_CTRL);
	val &= ~CH_VSYNC_M(id);
	if (active_high)
		val |= CH_PVSYNC(id);
	phy_write(phy, PHY_SS_CTRL, val);
	mutex_unlock(&priv->lock);
	clk_disable_unprepare(priv->phy_clk);
}
EXPORT_SYMBOL_GPL(mixel_phy_lvds_set_vsync_pol);

static int mixel_lvds_phy_init(struct phy *phy)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	u32 val;

	clk_prepare_enable(priv->phy_clk);
	mutex_lock(&priv->lock);
	val = phy_read(phy, PHY_CTRL);
	val &= ~(M_MASK | CCM_MASK | CA_MASK | TST_MASK | NB | PD);
	val |= (M(0x0) | CCM(0x5) | CA(0x4) | TST(0x25) | RFB);
	phy_write(phy, PHY_CTRL, val);
	mutex_unlock(&priv->lock);
	clk_disable_unprepare(priv->phy_clk);

	return 0;
}

static int mixel_lvds_phy_power_on(struct phy *phy)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	unsigned int id = lvds_phy->id;

	clk_prepare_enable(priv->phy_clk);

	mutex_lock(&priv->lock);
	phy_write(phy, PHY_CTRL + SET, CH_EN(id));
	mutex_unlock(&priv->lock);

	usleep_range(500, 1000);

	return 0;
}

static int mixel_lvds_phy_power_off(struct phy *phy)
{
	struct mixel_lvds_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	unsigned int id = lvds_phy->id;

	mutex_lock(&priv->lock);
	phy_write(phy, PHY_CTRL + CLR, CH_EN(id));
	mutex_unlock(&priv->lock);

	clk_disable_unprepare(priv->phy_clk);

	return 0;
}

static const struct phy_ops mixel_lvds_phy_ops = {
	.init = mixel_lvds_phy_init,
	.power_on = mixel_lvds_phy_power_on,
	.power_off = mixel_lvds_phy_power_off,
	.owner = THIS_MODULE,
};

static int mixel_lvds_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct resource *res;
	struct phy_provider *phy_provider;
	struct mixel_lvds_phy_priv *priv;
	struct mixel_lvds_phy *lvds_phy;
	struct phy *phy;
	u32 phy_id;
	int ret;

	if (!np)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_ioremap(dev, res->start, SZ_256);
	if (!priv->base)
		return -ENOMEM;

	priv->dev = dev;

	priv->phy_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(priv->phy_clk)) {
		dev_err(dev, "cannot get phy clock\n");
		return PTR_ERR(priv->phy_clk);
	}

	mutex_init(&priv->lock);
	dev_set_drvdata(dev, priv);

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

		phy = devm_phy_create(dev, child, &mixel_lvds_phy_ops);
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

	return PTR_ERR_OR_ZERO(phy_provider);

put_child:
	of_node_put(child);
	return ret;
}

static const struct of_device_id mixel_lvds_phy_of_match[] = {
	{ .compatible = "mixel,lvds-phy" },
	{}
};
MODULE_DEVICE_TABLE(of, mixel_lvds_phy_of_match);

static struct platform_driver mixel_lvds_phy_driver = {
	.probe	= mixel_lvds_phy_probe,
	.driver = {
		.name = "mixel-lvds-phy",
		.of_match_table	= mixel_lvds_phy_of_match,
	}
};
module_platform_driver(mixel_lvds_phy_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("Mixel LVDS PHY driver");
MODULE_LICENSE("GPL v2");
