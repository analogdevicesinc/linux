/*
 * Samsung MIPI DSI Host Controller on IMX
 *
 * Copyright 2018 NXP
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <drm/bridge/sec_mipi_dsim.h>
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include <drm/drm_modeset_helper_vtables.h>

#include "imx-drm.h"

#define DRIVER_NAME "imx_sec_dsim_drv"

/* Dispmix Control & GPR Registers */
#define DISPLAY_MIX_SFT_RSTN_CSR		0X00
   #define MIPI_DSI_I_PRESETn_SFT_EN		BIT(5)
#define DISPLAY_MIX_CLK_EN_CSR			0x04
   #define MIPI_DSI_PCLK_SFT_EN			BIT(8)
   #define MIPI_DSI_CLKREF_SFT_EN		BIT(9)
#define GPR_MIPI_RESET_DIV			0x08
   /* Clock & Data lanes reset: Active Low */
   #define GPR_MIPI_S_RESETN			BIT(16)
   #define GPR_MIPI_M_RESETN			BIT(17)

struct imx_sec_dsim_device {
	struct device *dev;
	struct drm_encoder encoder;
	struct regmap *gpr;
};

#define enc_to_dsim(enc) container_of(enc, struct imx_sec_dsim_device, encoder)

static struct imx_sec_dsim_device *dsim_dev;

static void disp_mix_dsim_soft_reset_release(struct regmap *gpr, bool release)
{
	if (release)
		/* release dsi blk reset */
		regmap_update_bits(gpr, DISPLAY_MIX_SFT_RSTN_CSR,
				   MIPI_DSI_I_PRESETn_SFT_EN,
				   MIPI_DSI_I_PRESETn_SFT_EN);
	else
		regmap_update_bits(gpr, DISPLAY_MIX_SFT_RSTN_CSR,
				   MIPI_DSI_I_PRESETn_SFT_EN,
				   0x0);
}

static void disp_mix_dsim_clks_enable(struct regmap *gpr, bool enable)
{
	if (enable)
		regmap_update_bits(gpr, DISPLAY_MIX_CLK_EN_CSR,
				   MIPI_DSI_PCLK_SFT_EN | MIPI_DSI_CLKREF_SFT_EN,
				   MIPI_DSI_PCLK_SFT_EN | MIPI_DSI_CLKREF_SFT_EN);
	else
		regmap_update_bits(gpr, DISPLAY_MIX_CLK_EN_CSR,
				   MIPI_DSI_PCLK_SFT_EN | MIPI_DSI_CLKREF_SFT_EN,
				   0x0);
}

static void imx_sec_dsim_lanes_reset(struct regmap *gpr, bool reset)
{
	if (!reset)
		/* release lanes reset */
		regmap_update_bits(gpr, GPR_MIPI_RESET_DIV,
				   GPR_MIPI_S_RESETN | GPR_MIPI_M_RESETN,
				   GPR_MIPI_S_RESETN | GPR_MIPI_M_RESETN);
	else
		/* reset lanes */
		regmap_update_bits(gpr, GPR_MIPI_RESET_DIV,
				   GPR_MIPI_S_RESETN | GPR_MIPI_M_RESETN,
				   0x0);
}

static void imx_sec_dsim_encoder_helper_enable(struct drm_encoder *encoder)
{
	struct imx_sec_dsim_device *dsim_dev = enc_to_dsim(encoder);

	pm_runtime_get_sync(dsim_dev->dev);

	imx_sec_dsim_lanes_reset(dsim_dev->gpr, false);
}

static void imx_sec_dsim_encoder_helper_disable(struct drm_encoder *encoder)
{
	struct imx_sec_dsim_device *dsim_dev = enc_to_dsim(encoder);

	imx_sec_dsim_lanes_reset(dsim_dev->gpr, true);

	pm_runtime_put_sync(dsim_dev->dev);
}

static int imx_sec_dsim_encoder_helper_atomic_check(struct drm_encoder *encoder,
						    struct drm_crtc_state *crtc_state,
						    struct drm_connector_state *conn_state)
{
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);

	/* sec dsim can only accept active hight DE */
	imx_crtc_state->bus_flags |= DRM_BUS_FLAG_DE_HIGH;

	/* For the dotclock polarity, default is neg edge;
	 * and in the dsim spec, there is no explict words
	 * to illustrate the dotclock polarity requirement.
	 */
	imx_crtc_state->bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;

	imx_crtc_state->bus_format = adjusted_mode->private_flags & 0xffff;

	return 0;
}

static const struct drm_encoder_helper_funcs imx_sec_dsim_encoder_helper_funcs = {
	.enable  = imx_sec_dsim_encoder_helper_enable,
	.disable = imx_sec_dsim_encoder_helper_disable,
	.atomic_check = imx_sec_dsim_encoder_helper_atomic_check,
};

static const struct drm_encoder_funcs imx_sec_dsim_encoder_funcs = {
	.destroy = imx_drm_encoder_destroy,
};

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.mode_valid	= NULL,
};

static const struct of_device_id imx_sec_dsim_dt_ids[] = {
	{
		.compatible = "fsl,imx8mm-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_sec_dsim_dt_ids);

static int imx_sec_dsim_bind(struct device *dev, struct device *master,
			     void *data)
{
	int ret, irq;
	struct resource *res;
	struct drm_device *drm_dev = data;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id = of_match_device(imx_sec_dsim_dt_ids,
							   dev);
	const struct sec_mipi_dsim_plat_data *pdata = of_id->data;
	struct drm_encoder *encoder;

	dev_dbg(dev, "%s: dsim bind begin\n", __func__);
	dsim_dev = devm_kzalloc(dev, sizeof(*dsim_dev), GFP_KERNEL);
	if (!dsim_dev) {
		dev_err(dev, "Unable to allocate 'dsim_dev'\n");
		return -ENOMEM;
	}

	dsim_dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	dsim_dev->gpr = syscon_regmap_lookup_by_phandle(np, "dsi-gpr");
	if (IS_ERR(dsim_dev->gpr))
		return PTR_ERR(dsim_dev->gpr);

	encoder = &dsim_dev->encoder;
	ret = imx_drm_encoder_parse_of(drm_dev, encoder, np);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &imx_sec_dsim_encoder_helper_funcs);

	ret = drm_encoder_init(drm_dev, encoder,
			       &imx_sec_dsim_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, dev_name(dev));
	if (ret)
		return ret;

	/* bind sec dsim bridge */
	ret = sec_mipi_dsim_bind(dev, master, data, encoder, res, irq, pdata);
	if (ret) {
		dev_err(dev, "failed to bind sec dsim bridge: %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	pm_runtime_enable(dev);

	/* Pull dsim out of reset */
	disp_mix_dsim_soft_reset_release(dsim_dev->gpr, true);
	disp_mix_dsim_clks_enable(dsim_dev->gpr, true);
	imx_sec_dsim_lanes_reset(dsim_dev->gpr, false);

	dev_dbg(dev, "%s: dsim bind end\n", __func__);

	return 0;
}

static void imx_sec_dsim_unbind(struct device *dev, struct device *master,
				void *data)
{
	pm_runtime_disable(dev);

	sec_mipi_dsim_unbind(dev, master, data);

	drm_encoder_cleanup(&dsim_dev->encoder);
}

static const struct component_ops imx_sec_dsim_ops = {
	.bind	= imx_sec_dsim_bind,
	.unbind	= imx_sec_dsim_unbind,
};

static int imx_sec_dsim_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s: dsim probe begin\n", __func__);

	return component_add(&pdev->dev, &imx_sec_dsim_ops);
}

static int imx_sec_dsim_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx_sec_dsim_ops);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int imx_sec_dsim_suspend(struct device *dev)
{
	return 0;
}

static int imx_sec_dsim_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM
static int imx_sec_dsim_runtime_suspend(struct device *dev)
{
	sec_mipi_dsim_suspend(dev);

	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int imx_sec_dsim_runtime_resume(struct device *dev)
{
	request_bus_freq(BUS_FREQ_HIGH);

	sec_mipi_dsim_resume(dev);

	return 0;
}
#endif

static const struct dev_pm_ops imx_sec_dsim_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx_sec_dsim_suspend,
				imx_sec_dsim_resume)
	SET_RUNTIME_PM_OPS(imx_sec_dsim_runtime_suspend,
			   imx_sec_dsim_runtime_resume,
			   NULL)
};

struct platform_driver imx_sec_dsim_driver = {
	.probe    = imx_sec_dsim_probe,
	.remove   = imx_sec_dsim_remove,
	.driver   = {
		.name = DRIVER_NAME,
		.of_match_table = imx_sec_dsim_dt_ids,
		.pm = &imx_sec_dsim_pm_ops,
	},
};

module_platform_driver(imx_sec_dsim_driver);

MODULE_DESCRIPTION("NXP i.MX MIPI DSI Host Controller driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
