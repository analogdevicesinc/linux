// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

#include "stmmac.h"
#include "stmmac_platform.h"

#define EMAC_1G_CG_ENABLE                       BIT(0)
#define EMAC_1G_YODA_MASK                       GENMASK(19, 3)

#define ETH1G_DEVCLK_MASK                       GENMASK(13, 6)
#define ETH1G_DEVCLK_DIV_FUND                   BIT(6)
#define ETH1G_DEVCLK_DIV_KILLCLK                0       /* BIT(7) */
#define ETH1G_DEVCLK_DIV_MCS_RESET              0       /* BIT(8) */
#define ETH1G_DEVCLK_DIV_RATIO                  0       /* Bits 9-10 */
#define ETH1G_DEVCLK_DIV_RB                     BIT(11)
#define ETH1G_DEVCLK_BUFFER_ENABLE              BIT(12)
#define ETH1G_DEVCLK_BUFFER_TERM_ENABLE         BIT(13)
#define ETH1G_DEVCLK_DEFAULT_VAL                ETH1G_DEVCLK_DIV_FUND |         \
	ETH1G_DEVCLK_DIV_KILLCLK |      \
	ETH1G_DEVCLK_DIV_MCS_RESET |    \
	ETH1G_DEVCLK_DIV_RATIO |        \
	ETH1G_DEVCLK_DIV_RB |           \
	ETH1G_DEVCLK_BUFFER_ENABLE

#define ETH1G_REFCLK_MASK                       BIT(17)
#define ETH1G_REFCLK_REFPATH_PD                 0 /* BIT(17) */
#define ETH1G_REFCLK_DEFAULT_VAL                ETH1G_REFCLK_REFPATH_PD

#define BASE_CLK_SPEED_50MHZ                    50
#define BASE_CLK_SPEED_125MHZ                   125
#define BASE_CLK_SPEED_250MHZ                   250

struct adrv906x_priv_data {
	struct stmmac_priv *stm_priv;
	unsigned int base_clk_speed;
	void __iomem *clk_div_base;
};

static void adrv906x_dwmac_mac_speed(void *priv, unsigned int speed)
{
	struct adrv906x_priv_data *sam_priv = (struct adrv906x_priv_data *)priv;
	u32 reg;

	// Disable clock
	reg = ioread32(sam_priv->clk_div_base);
	reg |= EMAC_1G_CG_ENABLE;
	iowrite32(reg, sam_priv->clk_div_base);

	// Set PHY iface (RGMII | RMII) and clock divider
	switch (sam_priv->stm_priv->plat->phy_interface) {
	case PHY_INTERFACE_MODE_RMII:
		if (sam_priv->base_clk_speed == BASE_CLK_SPEED_125MHZ) {
			dev_err(sam_priv->stm_priv->device,
				"phy mode RMII - invalid clock speed");
			return;
		}

		reg &= ~EMAC_1G_YODA_MASK;
		reg |= 4 << 3;
		if (speed == SPEED_10) {
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_50MHZ)
				reg |= (19 << 6);
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_250MHZ)
				reg |= (99 << 6) + (4 << 13);
		}
		if (speed == SPEED_100) {
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_50MHZ)
				reg |= (1 << 6);
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_250MHZ)
				reg |= (9 << 6) + (4 << 13);
		}
		if (speed == SPEED_1000) {
			dev_err(sam_priv->stm_priv->device,
				"phy mode RMII - 1G not supported");
			return;
		}
		break;
	case PHY_INTERFACE_MODE_RGMII:
		reg &= ~EMAC_1G_YODA_MASK;
		reg |= 1 << 3;
		if (speed == SPEED_10) {
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_50MHZ)
				reg |= (19 << 13);
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_125MHZ)
				reg |= (49 << 13);
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_250MHZ)
				reg |= (99 << 13);
		}
		if (speed == SPEED_100) {
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_50MHZ)
				reg |= (1 << 13);
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_125MHZ)
				reg |= (4 << 13);
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_250MHZ)
				reg |= (9 << 13);
		}
		if (speed == SPEED_1000) {
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_50MHZ) {
				dev_err(sam_priv->stm_priv->device,
					"phy mode RGMII - invalid clock speed");
				return;
			}

			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_125MHZ)
				reg |= 0;
			if (sam_priv->base_clk_speed == BASE_CLK_SPEED_250MHZ)
				reg |= (1 << 13);
		}
		break;
	default:
		dev_err(sam_priv->stm_priv->device,
			"phy mode not supported");
		return;
	}
	iowrite32(reg, sam_priv->clk_div_base);

	// Re-enable clock
	reg &= ~EMAC_1G_CG_ENABLE;
	iowrite32(reg, sam_priv->clk_div_base);
}

void adrv906x_dwmac_link_update_info(struct net_device *ndev)
{
	struct stmmac_priv *stm_priv = netdev_priv(ndev);
	struct phy_device *phydev = ndev->phydev;

	if (!phydev->link) {
		stm_priv->speed = 0;
		netdev_info(ndev, "%s: link down", ndev->name);
		return;
	}

	netdev_info(ndev, "%s: link up, speed %u Mb/s, %s duplex",
		    ndev->name, phydev->speed,
		    phydev->duplex ? "full" : "half");
}

static void adrv906x_clk_buffer_enable(void __iomem *clk_ctrl_base, bool term_en)
{
	u32 val;

	val = ioread32(clk_ctrl_base);
	val &= ~ETH1G_DEVCLK_MASK;
	val |= ETH1G_DEVCLK_DEFAULT_VAL;
	if (term_en)
		val |= ETH1G_DEVCLK_BUFFER_TERM_ENABLE;
	iowrite32(val, clk_ctrl_base);

	val = ioread32(clk_ctrl_base + 0x04);
	val &= ~ETH1G_REFCLK_MASK;
	val |= ETH1G_REFCLK_DEFAULT_VAL;
	iowrite32(val, clk_ctrl_base + 0x04);
}

static int dwmac_adrv906x_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct adrv906x_priv_data *sam_priv;
	struct device *dev = &pdev->dev;
	struct device_node *clk_div_np;
	struct net_device *ndev;
	void __iomem *clk_ctrl_base;
	u32 addr, len;
	bool term_en;
	int ret;

	sam_priv = devm_kzalloc(dev, sizeof(*sam_priv), GFP_KERNEL);
	if (!sam_priv)
		return -ENOMEM;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided");
			return -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	clk_div_np = of_get_child_by_name(pdev->dev.of_node, "clock_divider");
	if (!clk_div_np) {
		dev_err(&pdev->dev, "clock divider could not be detected");
		return -EINVAL;
	}

	of_property_read_u32(clk_div_np, "base-clk-speed", &sam_priv->base_clk_speed);
	dev_info(&pdev->dev, "base clock speed %d", sam_priv->base_clk_speed);

	of_property_read_u32_index(clk_div_np, "reg", 0, &addr);
	of_property_read_u32_index(clk_div_np, "reg", 1, &len);
	sam_priv->clk_div_base = devm_ioremap(&pdev->dev, addr, len);

	of_property_read_u32_index(clk_div_np, "ctrl_reg", 0, &addr);
	of_property_read_u32_index(clk_div_np, "ctrl_reg", 1, &len);
	clk_ctrl_base = devm_ioremap(&pdev->dev, addr, len);

	term_en = of_property_read_bool(clk_div_np, "adi,term_en");

	adrv906x_clk_buffer_enable(clk_ctrl_base, term_en);

	plat_dat->bsp_priv = sam_priv;
	plat_dat->fix_mac_speed = adrv906x_dwmac_mac_speed;

	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			goto err_remove_config_dt;
	}

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_exit;

	ndev = platform_get_drvdata(pdev);
	sam_priv->stm_priv = netdev_priv(ndev);

	return 0;

err_exit:
	if (plat_dat->exit)
		plat_dat->exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	if (pdev->dev.of_node)
		stmmac_remove_config_dt(pdev, plat_dat);

	devm_kfree(dev, sam_priv);
	return ret;
}

static const struct of_device_id dwmac_adrv906x_match[] = {
	{ .compatible = "adi,adrv906x-dwmac", },
	{ .compatible = "snps,dwmac-5.10a",   },
	{ },
};
MODULE_DEVICE_TABLE(of, dwmac_adrv906x_match);

static struct platform_driver dwmac_adrv906x_driver = {
	.probe			= dwmac_adrv906x_probe,
	.remove			= stmmac_pltfr_remove,
	.driver			= {
		.name		= "adrv906x1geth",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(dwmac_adrv906x_match),
	},
};
module_platform_driver(dwmac_adrv906x_driver);

MODULE_DESCRIPTION("ADRV906X 1G dwmac driver");
MODULE_LICENSE("GPL v2");
