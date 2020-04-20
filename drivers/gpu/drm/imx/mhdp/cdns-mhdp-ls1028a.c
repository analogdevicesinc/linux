// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 *
 */
#include <linux/clk.h>
#include <drm/drm_vblank.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include "cdns-mhdp-imx.h"

static const struct of_device_id scfg_device_ids[] = {
	{ .compatible = "fsl,ls1028a-scfg", },
	{}
};

static void ls1028a_phy_reset(u8 reset)
{
	struct device_node *scfg_node;
	void __iomem *scfg_base = NULL;

	scfg_node = of_find_matching_node(NULL, scfg_device_ids);
	if (scfg_node)
		scfg_base = of_iomap(scfg_node, 0);

	iowrite32(reset, scfg_base + 0x230);
}

int ls1028a_clocks_init(struct imx_mhdp_device *imx_mhdp)
{
	struct device *dev = imx_mhdp->mhdp.dev;
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	clks->clk_core = devm_clk_get(dev, "clk_core");
	if (IS_ERR(clks->clk_core)) {
		dev_warn(dev, "failed to get hdp core clk\n");
		return PTR_ERR(clks->clk_core);
	}

	clks->clk_pxl = devm_clk_get(dev, "clk_pxl");
	if (IS_ERR(clks->clk_pxl)) {
		dev_warn(dev, "failed to get pxl clk\n");
		return PTR_ERR(clks->clk_pxl);
	}

	return true;
}

static int ls1028a_pixel_clk_enable(struct imx_mhdp_device *imx_mhdp)
{
	struct imx_hdp_clks *clks = &imx_mhdp->clks;
	struct device *dev = imx_mhdp->mhdp.dev;
	int ret;

	ret = clk_prepare_enable(clks->clk_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk pxl error\n", __func__);
		return ret;
	}

	return ret;
}

static void ls1028a_pixel_clk_disable(struct imx_mhdp_device *imx_mhdp)
{
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	clk_disable_unprepare(clks->clk_pxl);
}

static void ls1028a_pixel_clk_set_rate(struct imx_mhdp_device *imx_mhdp,
				       u32 pclock)
{
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	clk_set_rate(clks->clk_pxl, pclock);
}

int cdns_mhdp_power_on_ls1028a(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp = container_of
				(mhdp, struct imx_mhdp_device, mhdp);

	/* clock init and  rate set */
	ls1028a_clocks_init(imx_mhdp);

	ls1028a_pixel_clk_enable(imx_mhdp);

	/* Init pixel clock with 148.5MHz before FW init */
	ls1028a_pixel_clk_set_rate(imx_mhdp, 148500000);

	ls1028a_phy_reset(1);

	return 0;
}

void cdns_mhdp_pclk_rate_ls1028a(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp = container_of
				(mhdp, struct imx_mhdp_device, mhdp);

	/* set pixel clock before video mode setup */
	ls1028a_pixel_clk_disable(imx_mhdp);

	ls1028a_pixel_clk_set_rate(imx_mhdp, imx_mhdp->mhdp.mode.clock * 1000);

	ls1028a_pixel_clk_enable(imx_mhdp);
}
