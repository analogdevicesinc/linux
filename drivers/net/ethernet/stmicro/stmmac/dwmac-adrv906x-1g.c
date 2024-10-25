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
#define EMAC_1G_OSC_CLK_DIV_MASK                GENMASK(19, 13)
#define EMAC_1G_CLK_DIV_MASK                    GENMASK(12, 6)
#define EMAC_1G_PHY_INTF_SEL_I_MASK             GENMASK(5, 3)
#define EMAC_1G_OSC_CLK_DIV_OFF                 13
#define EMAC_1G_CLK_DIV_OFF                     6
#define EMAC_1G_PHY_INTF_SEL_I_OFF              3
#define EMAC_1G_PHY_INTF_SEL_I_RMII             4
#define EMAC_1G_PHY_INTF_SEL_I_RGMII            1

#define ETH1G_DEVCLK_MASK                       GENMASK(13, 6)
#define ETH1G_DEVCLK_DIV_FUND                   BIT(6)
#define ETH1G_DEVCLK_DIV_KILLCLK                0       /* BIT(7) */
#define ETH1G_DEVCLK_DIV_MCS_RESET              0       /* BIT(8) */
#define ETH1G_DEVCLK_DIV_RATIO                  0       /* Bits 9-10 */
#define ETH1G_DEVCLK_DIV_RB                     BIT(11)
#define ETH1G_DEVCLK_BUFFER_ENABLE              BIT(12)
#define ETH1G_DEVCLK_BUFFER_TERM_ENABLE         BIT(13)
#define ETH1G_DEVCLK_DEFAULT_VAL                (ETH1G_DEVCLK_DIV_FUND |       \
						 ETH1G_DEVCLK_DIV_KILLCLK |    \
						 ETH1G_DEVCLK_DIV_MCS_RESET |  \
						 ETH1G_DEVCLK_DIV_RATIO |      \
						 ETH1G_DEVCLK_DIV_RB |         \
						 ETH1G_DEVCLK_BUFFER_ENABLE)

#define ETH1G_REFCLK_MASK                       BIT(17)
#define ETH1G_REFCLK_REFPATH_PD                 0 /* BIT(17) */
#define ETH1G_REFCLK_DEFAULT_VAL                ETH1G_REFCLK_REFPATH_PD

#define HZ_TO_MHZ(freq)                         (freq * 1000 * 1000)
#define CLK_2_5MHZ                              HZ_TO_MHZ(2.5)
#define CLK_25MHZ                               HZ_TO_MHZ(25)
#define CLK_50MHZ                               HZ_TO_MHZ(50)
#define CLK_125MHZ                              HZ_TO_MHZ(125)

struct adrv906x_priv_data {
	struct stmmac_priv *stm_priv;
	uint32_t base_clk_speed;
	void __iomem *clk_div_base;
	phy_interface_t phy_interface;
};

static char *macaddr;
module_param(macaddr, charp, 0644);
MODULE_PARM_DESC(macaddr, "set dev mac addresse via kernel module parameter");

static int adrv906x_dwmac_set_clk_dividers(void *priv, unsigned int speed, bool force_reconfig)
{
	struct adrv906x_priv_data *adrv_priv = (struct adrv906x_priv_data *)priv;
	ulong rate;
	u32 reg;
	uint32_t osc_div;
	uint32_t rmii_div;

	/* Required clock freq depends on the link speed */
	switch (speed) {
	case SPEED_10:   rate = CLK_2_5MHZ; break;
	case SPEED_100:  rate = CLK_25MHZ; break;
	case SPEED_1000: rate = CLK_125MHZ; break;
	default: pr_err("Invalid link speed"); return -1;
	}

	/* Sanity checks */
	if (((adrv_priv->base_clk_speed) % rate) != 0) {
		pr_err("Unable to get MAC clock");
		return -1;
	}

	if ((adrv_priv->phy_interface == PHY_INTERFACE_MODE_RMII) &&
	    ((adrv_priv->base_clk_speed % CLK_50MHZ) != 0)) {
		pr_err("Unable to get RMII PHY clock (50 MHz)");
		return -1;
	}

	if ((adrv_priv->phy_interface == PHY_INTERFACE_MODE_RMII) &&
	    (speed == SPEED_1000)) {
		pr_err("RMII does not support 1000 Mbs");
		return -1;
	}

	if ((adrv_priv->phy_interface != PHY_INTERFACE_MODE_RMII) &&
	    (adrv_priv->phy_interface != PHY_INTERFACE_MODE_RGMII)) {
		pr_err("MAC-PHY Interface (%d) not supported", adrv_priv->phy_interface);
		return -1;
	}

	/* Compute clock dividers */
	if (adrv_priv->phy_interface == PHY_INTERFACE_MODE_RMII) {
		/* input_clk   _|-> OSC_CLK_DIV (50 MHz) ------> PHY core clock (REF_CLK)
		 * (ie 250MHz)  |-> RMII_CLK_DIV (2.5|25 MHz) -> Tx_clk and Rx_clk to GMAC
		 *                  (based on PHY link 10|100)
		 */
		osc_div = (adrv_priv->base_clk_speed / CLK_50MHZ) - 1;
		rmii_div = ((adrv_priv->base_clk_speed) / (rate)) - 1;
	} else if (adrv_priv->phy_interface == PHY_INTERFACE_MODE_RGMII) {
		/*
		 * input_clk     -> OSC_CLK_DIV (2.5|25|125 MHz) -> Tx_clk to PHY and Tx_clk to GMAC
		 * (ie 250MHz)      (based on PHY link 10|100|1000)
		 *
		 * Note: Rx_clk to GMAC IP is provided by the external PHY
		 * Note: RMII_CLK_DIV does not apply
		 * Note: PHY core clock is provided by a crystal circuitry
		 */
		osc_div = ((adrv_priv->base_clk_speed) / rate) - 1;
		rmii_div = 0;
	}

	/* Do not reconfigure clock dividers in RMII if they have not changed
	 *
	 * Workaround: Adrv906x clocks divider block require to stop all the
	 * clocks before updating their value (even though only one of them
	 * needs to be updated). For RMII, one of these clocks is the PHY core
	 * clock (REF_CLK). Linux general driver flow does not expect that PHY
	 * core clock is interrupted at any time. This leads to a wrong behaviour
	 * (link up and down all the time). Flow:
	 * - MAC detects link is up (reading PHY register)
	 * - MAC reconfigure clocks (according to Link Speed)
	 * - Stopping the clock in the previous step results in a link down
	 *   indication from the PHY chip (and a new autonegotiation process)
	 * - MAC sets the link down again
	 * - A bit later, autoneg was complete and link is up again and the
	 *   story repeats once and again.
	 *
	 * The fix is not to reconfigure clocks if their value remains the same.
	 * This prevents the endless loop in the second iteration.
	 *
	 * Note: PHY core clock in RGMII is not generated by Adrv906x, so this
	 *       issue does not apply.
	 * Note: During initalization, reconfiguration is forced. This enables
	 *       the 50 MHz PHY core clock (RMII case)
	 */
	if (!force_reconfig && adrv_priv->phy_interface == PHY_INTERFACE_MODE_RMII) {
		uint32_t curr_osc_div;
		uint32_t curr_rmii_div;

		reg = ioread32(adrv_priv->clk_div_base);
		curr_osc_div = (reg & EMAC_1G_OSC_CLK_DIV_MASK) >> EMAC_1G_OSC_CLK_DIV_OFF;
		curr_rmii_div = (reg & EMAC_1G_CLK_DIV_MASK) >> EMAC_1G_CLK_DIV_OFF;

		if ((osc_div == curr_osc_div) && (rmii_div == curr_rmii_div))
			return 0;
	}

	/* Disable clock */
	reg = ioread32(adrv_priv->clk_div_base);
	reg |= EMAC_1G_CG_ENABLE;
	iowrite32(reg, adrv_priv->clk_div_base);

	/* Set clock divider */
	if (adrv_priv->phy_interface == PHY_INTERFACE_MODE_RMII) {
		reg &= ~EMAC_1G_PHY_INTF_SEL_I_MASK;
		reg |= EMAC_1G_PHY_INTF_SEL_I_RMII << EMAC_1G_PHY_INTF_SEL_I_OFF;
	} else if (adrv_priv->phy_interface == PHY_INTERFACE_MODE_RGMII) {
		reg &= ~EMAC_1G_PHY_INTF_SEL_I_MASK;
		reg |= EMAC_1G_PHY_INTF_SEL_I_RGMII << EMAC_1G_PHY_INTF_SEL_I_OFF;
	}
	reg &= ~(EMAC_1G_OSC_CLK_DIV_MASK | EMAC_1G_CLK_DIV_MASK);
	reg |= (osc_div << EMAC_1G_OSC_CLK_DIV_OFF) |
	       (rmii_div << EMAC_1G_CLK_DIV_OFF);
	iowrite32(reg, adrv_priv->clk_div_base);

	/* Re-enable clock */
	reg &= ~EMAC_1G_CG_ENABLE;
	iowrite32(reg, adrv_priv->clk_div_base);

	return 0;
}

static void adrv906x_dwmac_fix_mac_speed(void *priv, unsigned int speed)
{
	adrv906x_dwmac_set_clk_dividers(priv, speed, false);
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
	struct adrv906x_priv_data *adrv_priv;
	struct device *dev = &pdev->dev;
	struct device_node *clk_div_np;
	struct net_device *ndev;
	struct sockaddr sock_addr;
	void __iomem *clk_ctrl_base;
	u32 addr, len;
	bool term_en;
	int ret;

	adrv_priv = devm_kzalloc(dev, sizeof(*adrv_priv), GFP_KERNEL);
	if (!adrv_priv)
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

	if (macaddr) {
		memset(sock_addr.sa_data, 0, sizeof(sock_addr.sa_data));
		mac_pton(macaddr, sock_addr.sa_data);
		stmmac_res.mac = sock_addr.sa_data;
	}

	clk_div_np = of_get_child_by_name(pdev->dev.of_node, "clock_divider");
	if (!clk_div_np) {
		dev_err(&pdev->dev, "clock divider could not be detected");
		return -EINVAL;
	}

	adrv_priv->base_clk_speed = clk_get_rate(plat_dat->stmmac_clk);
	dev_info(&pdev->dev, "base clock speed %d MHz", adrv_priv->base_clk_speed / (HZ_TO_MHZ(1)));

	of_property_read_u32_index(clk_div_np, "reg", 0, &addr);
	of_property_read_u32_index(clk_div_np, "reg", 1, &len);
	adrv_priv->clk_div_base = devm_ioremap(&pdev->dev, addr, len);

	of_property_read_u32_index(clk_div_np, "ctrl_reg", 0, &addr);
	of_property_read_u32_index(clk_div_np, "ctrl_reg", 1, &len);
	clk_ctrl_base = devm_ioremap(&pdev->dev, addr, len);

	term_en = of_property_read_bool(clk_div_np, "adi,term_en");

	adrv906x_clk_buffer_enable(clk_ctrl_base, term_en);

	plat_dat->bsp_priv = adrv_priv;
	plat_dat->fix_mac_speed = adrv906x_dwmac_fix_mac_speed;

	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			goto err_remove_config_dt;
	}


	adrv_priv->phy_interface = plat_dat->phy_interface;
	/* Configure clock distribution (depends on the phy interface type).
	 * Enable Link-speed-related clocks to arbitrary value
	 * Enable PHY core clock to 50 MHz (only for RMII)
	 */
	ret = adrv906x_dwmac_set_clk_dividers(adrv_priv, SPEED_100, true);
	if (ret)
		goto err_remove_config_dt;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_exit;

	ndev = platform_get_drvdata(pdev);
	adrv_priv->stm_priv = netdev_priv(ndev);

	return 0;

err_exit:
	if (plat_dat->exit)
		plat_dat->exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	if (pdev->dev.of_node)
		stmmac_remove_config_dt(pdev, plat_dat);

	devm_kfree(dev, adrv_priv);
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
