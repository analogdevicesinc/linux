/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "pfe_mod.h"

struct ls1012a_pfe_platform_data pfe_platform_data;

static int pfe_get_gemac_if_proprties(struct device_node *parent, int port, int
					if_cnt,
					struct ls1012a_pfe_platform_data
					*pdata)
{
	struct device_node *gem = NULL, *phy = NULL;
	int size;
	int ii = 0, phy_id = 0;
	const u32 *addr;
	const void *mac_addr;

	for (ii = 0; ii < if_cnt; ii++) {
		gem = of_get_next_child(parent, gem);
		if (!gem)
			goto err;
		addr = of_get_property(gem, "reg", &size);
		if (addr && (be32_to_cpup(addr) == port))
			break;
	}

	if (ii >= if_cnt) {
		pr_err("%s:%d Failed to find interface = %d\n",
		       __func__, __LINE__, if_cnt);
		goto err;
	}

	pdata->ls1012a_eth_pdata[port].gem_id = port;

	mac_addr = of_get_mac_address(gem);

	if (mac_addr) {
		memcpy(pdata->ls1012a_eth_pdata[port].mac_addr, mac_addr,
		       ETH_ALEN);
	}

	pdata->ls1012a_eth_pdata[port].mii_config = of_get_phy_mode(gem);

	if ((pdata->ls1012a_eth_pdata[port].mii_config) < 0)
		pr_err("%s:%d Incorrect Phy mode....\n", __func__,
		       __LINE__);

	addr = of_get_property(gem, "fsl,gemac-bus-id", &size);
	if (!addr)
		pr_err("%s:%d Invalid gemac-bus-id....\n", __func__,
		       __LINE__);
	else
		pdata->ls1012a_eth_pdata[port].bus_id = be32_to_cpup(addr);

	addr = of_get_property(gem, "fsl,gemac-phy-id", &size);
	if (!addr) {
		pr_err("%s:%d Invalid gemac-phy-id....\n", __func__,
		       __LINE__);
	} else {
		phy_id = be32_to_cpup(addr);
		pdata->ls1012a_eth_pdata[port].phy_id = phy_id;
		pdata->ls1012a_mdio_pdata[0].phy_mask &= ~(1 << phy_id);
	}

	addr = of_get_property(gem, "fsl,mdio-mux-val", &size);
	if (!addr) {
		pr_err("%s: Invalid mdio-mux-val....\n", __func__);
	} else {
		phy_id = be32_to_cpup(addr);
		pdata->ls1012a_eth_pdata[port].mdio_muxval = phy_id;
	}

	if (pdata->ls1012a_eth_pdata[port].phy_id < 32)
		pfe->mdio_muxval[pdata->ls1012a_eth_pdata[port].phy_id] =
			 pdata->ls1012a_eth_pdata[port].mdio_muxval;

	addr = of_get_property(gem, "fsl,pfe-phy-if-flags", &size);
	if (!addr)
		pr_err("%s:%d Invalid pfe-phy-if-flags....\n",
		       __func__, __LINE__);
	else
		pdata->ls1012a_eth_pdata[port].phy_flags = be32_to_cpup(addr);

	/* If PHY is enabled, read mdio properties */
	if (pdata->ls1012a_eth_pdata[port].phy_flags & GEMAC_NO_PHY)
		goto done;

	phy = of_get_next_child(gem, NULL);

	addr = of_get_property(phy, "reg", &size);

	if (!addr)
		pr_err("%s:%d Invalid phy enable flag....\n",
		       __func__, __LINE__);
	else
		pdata->ls1012a_mdio_pdata[port].enabled = be32_to_cpup(addr);

	pdata->ls1012a_mdio_pdata[port].irq[0] = PHY_POLL;

done:

	return 0;

err:
	return -1;
}

/*
 *
 * pfe_platform_probe -
 *
 *
 */
static int pfe_platform_probe(struct platform_device *pdev)
{
	struct resource res;
	int ii, rc, interface_count = 0, size = 0;
	const u32 *prop;
	struct device_node  *np;
	struct clk *pfe_clk;

	np = pdev->dev.of_node;

	if (!np) {
		pr_err("Invalid device node\n");
		return -EINVAL;
	}

	pfe = kzalloc(sizeof(*pfe), GFP_KERNEL);
	if (!pfe) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(pdev, pfe);

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));

	if (of_address_to_resource(np, 1, &res)) {
		rc = -ENOMEM;
		pr_err("failed to get ddr resource\n");
		goto err_ddr;
	}

	pfe->ddr_phys_baseaddr = res.start;
	pfe->ddr_size = resource_size(&res);

	pfe->ddr_baseaddr = phys_to_virt(res.start);
	if (!pfe->ddr_baseaddr) {
		pr_err("ioremap() ddr failed\n");
		rc = -ENOMEM;
		goto err_ddr;
	}

	pfe->scfg =
		syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						"fsl,pfe-scfg");
	if (IS_ERR(pfe->scfg)) {
		dev_err(&pdev->dev, "No syscfg phandle specified\n");
		return PTR_ERR(pfe->scfg);
	}

	pfe->cbus_baseaddr = of_iomap(np, 0);
	if (!pfe->cbus_baseaddr) {
		rc = -ENOMEM;
		pr_err("failed to get axi resource\n");
		goto err_axi;
	}

	pfe->hif_irq = platform_get_irq(pdev, 0);
	if (pfe->hif_irq < 0) {
		pr_err("platform_get_irq for hif failed\n");
		rc = pfe->hif_irq;
		goto err_hif_irq;
	}

	pfe->wol_irq = platform_get_irq(pdev, 2);
	if (pfe->wol_irq < 0) {
		pr_err("platform_get_irq for WoL failed\n");
		rc = pfe->wol_irq;
		goto err_hif_irq;
	}

	/* Read interface count */
	prop = of_get_property(np, "fsl,pfe-num-interfaces", &size);
	if (!prop) {
		pr_err("Failed to read number of interfaces\n");
		rc = -ENXIO;
		goto err_prop;
	}

	interface_count = be32_to_cpup(prop);
	if (interface_count <= 0) {
		pr_err("No ethernet interface count : %d\n",
		       interface_count);
		rc = -ENXIO;
		goto err_prop;
	}

	pfe_platform_data.ls1012a_mdio_pdata[0].phy_mask = 0xffffffff;

	for (ii = 0; ii < interface_count; ii++) {
		pfe_get_gemac_if_proprties(np, ii, interface_count,
					   &pfe_platform_data);
	}

	pfe->dev = &pdev->dev;

	pfe->dev->platform_data = &pfe_platform_data;

	/* declare WoL capabilities */
	device_init_wakeup(&pdev->dev, true);

	/* find the clocks */
	pfe_clk = devm_clk_get(pfe->dev, "pfe");
	if (IS_ERR(pfe_clk))
		return PTR_ERR(pfe_clk);

	/* PFE clock is (platform clock / 2) */
	/* save sys_clk value as KHz */
	pfe->ctrl.sys_clk = clk_get_rate(pfe_clk) / (2 * 1000);

	rc = pfe_probe(pfe);
	if (rc < 0)
		goto err_probe;

	return 0;

err_probe:
err_prop:
err_hif_irq:
	iounmap(pfe->cbus_baseaddr);

err_axi:
	iounmap(pfe->ddr_baseaddr);

err_ddr:
	platform_set_drvdata(pdev, NULL);

	kfree(pfe);

err_alloc:
	return rc;
}

/*
 * pfe_platform_remove -
 */
static int pfe_platform_remove(struct platform_device *pdev)
{
	struct pfe *pfe = platform_get_drvdata(pdev);
	int rc;

	pr_info("%s\n", __func__);

	rc = pfe_remove(pfe);

	iounmap(pfe->cbus_baseaddr);
	iounmap(pfe->ddr_baseaddr);

	platform_set_drvdata(pdev, NULL);

	kfree(pfe);

	return rc;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
int pfe_platform_suspend(struct device *dev)
{
	struct pfe *pfe = platform_get_drvdata(to_platform_device(dev));
	struct net_device *netdev;
	int i;

	pfe->wake = 0;

	for (i = 0; i < (NUM_GEMAC_SUPPORT); i++) {
		netdev = pfe->eth.eth_priv[i]->ndev;

		netif_device_detach(netdev);

		if (netif_running(netdev))
			if (pfe_eth_suspend(netdev))
				pfe->wake = 1;
	}

	/* Shutdown PFE only if we're not waking up the system */
	if (!pfe->wake) {
#if defined(LS1012A_PFE_RESET_WA)
		pfe_hif_rx_idle(&pfe->hif);
#endif
		pfe_ctrl_suspend(&pfe->ctrl);
		pfe_firmware_exit(pfe);

		pfe_hif_exit(pfe);
		pfe_hif_lib_exit(pfe);

		pfe_hw_exit(pfe);
	}

	return 0;
}

static int pfe_platform_resume(struct device *dev)
{
	struct pfe *pfe = platform_get_drvdata(to_platform_device(dev));
	struct net_device *netdev;
	int i;

	if (!pfe->wake) {
		pfe_hw_init(pfe, 1);
		pfe_hif_lib_init(pfe);
		pfe_hif_init(pfe);
#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
		util_enable();
#endif
		tmu_enable(0xf);
		class_enable();
		pfe_ctrl_resume(&pfe->ctrl);
	}

	for (i = 0; i < (NUM_GEMAC_SUPPORT); i++) {
		netdev = pfe->eth.eth_priv[i]->ndev;

		if (pfe->eth.eth_priv[i]->mii_bus)
			pfe_eth_mdio_reset(pfe->eth.eth_priv[i]->mii_bus);

		if (netif_running(netdev))
			pfe_eth_resume(netdev);

		netif_device_attach(netdev);
	}
	return 0;
}
#else
#define pfe_platform_suspend NULL
#define pfe_platform_resume NULL
#endif

static const struct dev_pm_ops pfe_platform_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pfe_platform_suspend, pfe_platform_resume)
};
#endif

static const struct of_device_id pfe_match[] = {
	{
		.compatible = "fsl,pfe",
	},
	{},
};
MODULE_DEVICE_TABLE(of, pfe_match);

static struct platform_driver pfe_platform_driver = {
	.probe = pfe_platform_probe,
	.remove = pfe_platform_remove,
	.driver = {
		.name = "pfe",
		.of_match_table = pfe_match,
#ifdef CONFIG_PM
		.pm = &pfe_platform_pm_ops,
#endif
	},
};

module_platform_driver(pfe_platform_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PFE Ethernet driver");
MODULE_AUTHOR("NXP DNCPE");
