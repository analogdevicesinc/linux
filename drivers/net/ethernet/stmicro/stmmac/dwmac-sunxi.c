// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * dwmac-sunxi.c - Allwinner sunxi DWMAC specific glue layer
 *
 * Copyright (C) 2013 Chen-Yu Tsai
 *
 * Chen-Yu Tsai  <wens@csie.org>
 */

#include <linux/stmmac.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/of_net.h>
#include <linux/regulator/consumer.h>

#include "stmmac_platform.h"

struct sunxi_priv_data {
	phy_interface_t interface;
	int clk_enabled;
	struct clk *tx_clk;
	struct regulator *regulator;
};

#define SUN7I_GMAC_GMII_RGMII_RATE	125000000
#define SUN7I_GMAC_MII_RATE		25000000

static int sun7i_gmac_init(struct platform_device *pdev, void *priv)
{
	struct sunxi_priv_data *gmac = priv;
	int ret = 0;

	if (gmac->regulator) {
		ret = regulator_enable(gmac->regulator);
		if (ret)
			return ret;
	}

	/* Set GMAC interface port mode
	 *
	 * The GMAC TX clock lines are configured by setting the clock
	 * rate, which then uses the auto-reparenting feature of the
	 * clock driver, and enabling/disabling the clock.
	 */
	if (phy_interface_mode_is_rgmii(gmac->interface)) {
		clk_set_rate(gmac->tx_clk, SUN7I_GMAC_GMII_RGMII_RATE);
		clk_prepare_enable(gmac->tx_clk);
		gmac->clk_enabled = 1;
	} else {
		clk_set_rate(gmac->tx_clk, SUN7I_GMAC_MII_RATE);
		ret = clk_prepare(gmac->tx_clk);
		if (ret && gmac->regulator)
			regulator_disable(gmac->regulator);
	}

	return ret;
}

static void sun7i_gmac_exit(struct platform_device *pdev, void *priv)
{
	struct sunxi_priv_data *gmac = priv;

	if (gmac->clk_enabled) {
		clk_disable(gmac->tx_clk);
		gmac->clk_enabled = 0;
	}
	clk_unprepare(gmac->tx_clk);

	if (gmac->regulator)
		regulator_disable(gmac->regulator);
}

static int sun7i_set_clk_tx_rate(void *bsp_priv, struct clk *clk_tx_i,
				 phy_interface_t interface, int speed)
{
	struct sunxi_priv_data *gmac = bsp_priv;

	if (interface == PHY_INTERFACE_MODE_GMII) {
		if (gmac->clk_enabled) {
			clk_disable(gmac->tx_clk);
			gmac->clk_enabled = 0;
		}
		clk_unprepare(gmac->tx_clk);

		if (speed == 1000) {
			clk_set_rate(gmac->tx_clk, SUN7I_GMAC_GMII_RGMII_RATE);
			clk_prepare_enable(gmac->tx_clk);
			gmac->clk_enabled = 1;
		} else {
			clk_set_rate(gmac->tx_clk, SUN7I_GMAC_MII_RATE);
			clk_prepare(gmac->tx_clk);
		}
	}
	return 0;
}

static int sun7i_gmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct sunxi_priv_data *gmac;
	struct device *dev = &pdev->dev;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	gmac = devm_kzalloc(dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac)
		return -ENOMEM;

	gmac->interface = plat_dat->phy_interface;

	gmac->tx_clk = devm_clk_get(dev, "allwinner_gmac_tx");
	if (IS_ERR(gmac->tx_clk)) {
		dev_err(dev, "could not get tx clock\n");
		return PTR_ERR(gmac->tx_clk);
	}

	/* Optional regulator for PHY */
	gmac->regulator = devm_regulator_get_optional(dev, "phy");
	if (IS_ERR(gmac->regulator)) {
		if (PTR_ERR(gmac->regulator) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "no regulator found\n");
		gmac->regulator = NULL;
	}

	/* platform data specifying hardware features and callbacks.
	 * hardware features were copied from Allwinner drivers. */
	plat_dat->tx_coe = 1;
	plat_dat->core_type = DWMAC_CORE_GMAC;
	plat_dat->bsp_priv = gmac;
	plat_dat->init = sun7i_gmac_init;
	plat_dat->exit = sun7i_gmac_exit;
	plat_dat->set_clk_tx_rate = sun7i_set_clk_tx_rate;
	plat_dat->tx_fifo_size = 4096;
	plat_dat->rx_fifo_size = 16384;

	return devm_stmmac_pltfr_probe(pdev, plat_dat, &stmmac_res);
}

static const struct of_device_id sun7i_dwmac_match[] = {
	{ .compatible = "allwinner,sun7i-a20-gmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun7i_dwmac_match);

static struct platform_driver sun7i_dwmac_driver = {
	.probe  = sun7i_gmac_probe,
	.driver = {
		.name           = "sun7i-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = sun7i_dwmac_match,
	},
};
module_platform_driver(sun7i_dwmac_driver);

MODULE_AUTHOR("Chen-Yu Tsai <wens@csie.org>");
MODULE_DESCRIPTION("Allwinner sunxi DWMAC specific glue layer");
MODULE_LICENSE("GPL");
