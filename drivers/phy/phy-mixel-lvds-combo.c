/*
 * Copyright 2017-2020 NXP
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
#include <linux/phy/phy-mixel-lvds-combo.h>
#include <linux/platform_device.h>

/* Control and Status Registers(CSR) */
#define PHY_CTRL	0x00
#define CCM(n)		(((n) & 0x7) << 5)
#define CCM_MASK	0xe0
#define CA(n)		(((n) & 0x7) << 2)
#define CA_MASK		0x1c
#define RFB		BIT(1)
#define LVDS_EN		BIT(0)

#define SS		0x20
#define CH_HSYNC_M(id)	BIT(0 + ((id) * 2))
#define CH_VSYNC_M(id)	BIT(1 + ((id) * 2))
#define CH_PHSYNC(id)	BIT(0 + ((id) * 2))
#define CH_PVSYNC(id)	BIT(1 + ((id) * 2))

#define ULPS		0x30
#define ULPS_MASK	0x1f
#define LANE(n)		BIT(n)

#define DPI		0x40
#define COLOR_CODE_MASK	0x7
#define BIT16_CFG1	0x0
#define BIT16_CFG2	0x1
#define BIT16_CFG3	0x2
#define BIT18_CFG1	0x3
#define BIT18_CFG2	0x4
#define BIT24		0x5

/* controller registers */
#define PD_TX		0x300
#define PD_PLL		0x31c
#define CO		0x32c
#define CO_DIV(n)	(ffs(n) - 1)

#define MIN_PLL_VCO_FREQ	640000000
#define MAX_PLL_VCO_FREQ	1500000000

struct mixel_lvds_phy {
	struct device *dev;
	void __iomem *csr_base;
	void __iomem *ctrl_base;
	struct mutex lock;
	struct phy *phy;
	struct clk *phy_clk;
};

static inline u32 phy_csr_read(struct phy *phy, unsigned int reg)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);

	return readl(lvds_phy->csr_base + reg);
}

static inline void phy_csr_write(struct phy *phy, unsigned int reg, u32 value)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);

	writel(value, lvds_phy->csr_base + reg);
}

static inline u32 phy_ctrl_read(struct phy *phy, unsigned int reg)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);

	return readl(lvds_phy->ctrl_base + reg);
}

static inline void phy_ctrl_write(struct phy *phy, unsigned int reg, u32 value)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);

	writel(value, lvds_phy->ctrl_base + reg);
}

void mixel_phy_combo_lvds_set_phy_speed(struct phy *phy,
					unsigned long phy_clk_rate)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	struct device *dev = lvds_phy->dev;
	unsigned long serial_clk = 7 * phy_clk_rate;
	unsigned long fvco = serial_clk;
	int div = 1;

	/*
	 * Choose an appropriate divider to meet the requirement of
	 * PLL VCO frequency range.
	 *
	 *  ---------  640MHz ~ 1500MHz   ------------      --------------
	 * | PLL VCO | ----------------> | CO divider | -> | serial clock |
	 *  ---------                     ------------      --------------
	 *                                 1/2/4/8 div     7 * phy_clk_rate
	 */
	if (fvco < MIN_PLL_VCO_FREQ) {
		do {
			div *= 2;
			fvco = serial_clk * div;
		} while (fvco < MIN_PLL_VCO_FREQ);

		if (div > 8)
			div = 8;
	}

	/* final fvco */
	fvco = serial_clk * div;
	if (fvco < MIN_PLL_VCO_FREQ || fvco > MAX_PLL_VCO_FREQ)
		dev_warn(dev, "PLL VCO frequency %lu is out of range\n", fvco);

	phy_pm_runtime_get_sync(phy);

	clk_prepare_enable(lvds_phy->phy_clk);
	mutex_lock(&lvds_phy->lock);
	phy_ctrl_write(phy, CO, CO_DIV(div));
	mutex_unlock(&lvds_phy->lock);
	clk_disable_unprepare(lvds_phy->phy_clk);

	clk_set_rate(lvds_phy->phy_clk, phy_clk_rate);

	phy_pm_runtime_put(phy);
}
EXPORT_SYMBOL_GPL(mixel_phy_combo_lvds_set_phy_speed);

void mixel_phy_combo_lvds_set_hsync_pol(struct phy *phy, bool active_high)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	u32 val;

	phy_pm_runtime_get_sync(phy);

	clk_prepare_enable(lvds_phy->phy_clk);
	mutex_lock(&lvds_phy->lock);
	val = phy_csr_read(phy, SS);
	val &= ~(CH_HSYNC_M(0) | CH_HSYNC_M(1));
	if (active_high)
		val |= (CH_PHSYNC(0) | CH_PHSYNC(1));
	phy_csr_write(phy, SS, val);
	mutex_unlock(&lvds_phy->lock);
	clk_disable_unprepare(lvds_phy->phy_clk);

	phy_pm_runtime_put(phy);
}
EXPORT_SYMBOL_GPL(mixel_phy_combo_lvds_set_hsync_pol);

void mixel_phy_combo_lvds_set_vsync_pol(struct phy *phy, bool active_high)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	u32 val;

	phy_pm_runtime_get_sync(phy);

	clk_prepare_enable(lvds_phy->phy_clk);
	mutex_lock(&lvds_phy->lock);
	val = phy_csr_read(phy, SS);
	val &= ~(CH_VSYNC_M(0) | CH_VSYNC_M(1));
	if (active_high)
		val |= (CH_PVSYNC(0) | CH_PVSYNC(1));
	phy_csr_write(phy, SS, val);
	mutex_unlock(&lvds_phy->lock);
	clk_disable_unprepare(lvds_phy->phy_clk);

	phy_pm_runtime_put(phy);
}
EXPORT_SYMBOL_GPL(mixel_phy_combo_lvds_set_vsync_pol);

static int mixel_lvds_combo_phy_init(struct phy *phy)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	u32 val;

	clk_prepare_enable(lvds_phy->phy_clk);
	mutex_lock(&lvds_phy->lock);
	val = phy_csr_read(phy, PHY_CTRL);
	val &= ~(CCM_MASK | CA_MASK);
	val |= (CCM(0x5) | CA(0x4) | RFB);
	phy_csr_write(phy, PHY_CTRL, val);

	val = phy_csr_read(phy, DPI);
	val &= ~COLOR_CODE_MASK;
	val |= BIT24;
	phy_csr_write(phy, DPI, val);
	mutex_unlock(&lvds_phy->lock);
	clk_disable_unprepare(lvds_phy->phy_clk);

	return 0;
}

static int mixel_lvds_combo_phy_power_on(struct phy *phy)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	u32 val;

	clk_prepare_enable(lvds_phy->phy_clk);
	mutex_lock(&lvds_phy->lock);
	phy_ctrl_write(phy, PD_PLL, 0);
	phy_ctrl_write(phy, PD_TX, 0);

	val = phy_csr_read(phy, ULPS);
	val &= ~ULPS_MASK;
	phy_csr_write(phy, ULPS, val);

	val = phy_csr_read(phy, PHY_CTRL);
	val |= LVDS_EN;
	phy_csr_write(phy, PHY_CTRL, val);
	mutex_unlock(&lvds_phy->lock);

	usleep_range(500, 1000);

	return 0;
}

static int mixel_lvds_combo_phy_power_off(struct phy *phy)
{
	struct mixel_lvds_phy *lvds_phy = phy_get_drvdata(phy);
	u32 val;

	mutex_lock(&lvds_phy->lock);
	val = phy_csr_read(phy, PHY_CTRL);
	val &= ~LVDS_EN;
	phy_csr_write(phy, PHY_CTRL, val);

	val = phy_csr_read(phy, ULPS);
	val |= ULPS_MASK;
	phy_csr_write(phy, ULPS, val);

	phy_ctrl_write(phy, PD_TX, 1);
	phy_ctrl_write(phy, PD_PLL, 1);
	mutex_unlock(&lvds_phy->lock);
	clk_disable_unprepare(lvds_phy->phy_clk);

	return 0;
}

static const struct phy_ops mixel_lvds_combo_phy_ops = {
	.init = mixel_lvds_combo_phy_init,
	.power_on = mixel_lvds_combo_phy_power_on,
	.power_off = mixel_lvds_combo_phy_power_off,
	.owner = THIS_MODULE,
};

static int mixel_lvds_combo_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct phy_provider *phy_provider;
	struct mixel_lvds_phy *lvds_phy;

	lvds_phy = devm_kzalloc(dev, sizeof(*lvds_phy), GFP_KERNEL);
	if (!lvds_phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	lvds_phy->csr_base = devm_ioremap(dev, res->start, SZ_256);
	if (!lvds_phy->csr_base)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -ENODEV;

	lvds_phy->ctrl_base = devm_ioremap(dev, res->start, SZ_4K);
	if (!lvds_phy->ctrl_base)
		return -ENOMEM;

	lvds_phy->phy_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(lvds_phy->phy_clk)) {
		dev_err(dev, "cannot get phy clock\n");
		return PTR_ERR(lvds_phy->phy_clk);
	}

	lvds_phy->dev = dev;
	mutex_init(&lvds_phy->lock);

	pm_runtime_enable(dev);

	lvds_phy->phy = devm_phy_create(dev, NULL, &mixel_lvds_combo_phy_ops);
	if (IS_ERR(lvds_phy->phy)) {
		dev_err(dev, "failed to create phy\n");
		pm_runtime_disable(dev);
		return PTR_ERR(lvds_phy->phy);
	}

	phy_set_drvdata(lvds_phy->phy, lvds_phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		pm_runtime_disable(dev);
		return PTR_ERR(phy_provider);
	}

	return 0;
}

static int mixel_lvds_combo_phy_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id mixel_lvds_combo_phy_of_match[] = {
	{ .compatible = "mixel,lvds-combo-phy" },
	{}
};
MODULE_DEVICE_TABLE(of, mixel_lvds_combo_phy_of_match);

static struct platform_driver mixel_lvds_combo_phy_driver = {
	.probe	= mixel_lvds_combo_phy_probe,
	.remove	= mixel_lvds_combo_phy_remove,
	.driver = {
		.name = "mixel-lvds-combo-phy",
		.of_match_table	= mixel_lvds_combo_phy_of_match,
	}
};
module_platform_driver(mixel_lvds_combo_phy_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("Mixel LVDS combo PHY driver");
MODULE_LICENSE("GPL v2");
