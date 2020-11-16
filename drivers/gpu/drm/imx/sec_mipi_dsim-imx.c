/*
 * Samsung MIPI DSI Host Controller on IMX
 *
 * Copyright 2018-2020 NXP
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
#include <linux/reset.h>
#include <drm/bridge/sec_mipi_dsim.h>
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"
#include "sec_mipi_dphy_ln14lpp.h"
#include "sec_mipi_pll_1432x.h"

#define DRIVER_NAME "imx_sec_dsim_drv"

/* fixed phy ref clk rate */
#define PHY_REF_CLK		12000

struct imx_sec_dsim_device {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct clk *clk_cfg;
	struct clk *clk_pllref;
	struct drm_encoder encoder;

	struct reset_control *soft_resetn;
	struct reset_control *clk_enable;
	struct reset_control *mipi_reset;

	atomic_t rpm_suspended;
};

#define enc_to_dsim(enc) container_of(enc, struct imx_sec_dsim_device, encoder)

static struct imx_sec_dsim_device *dsim_dev;

#if CONFIG_PM
static int imx_sec_dsim_runtime_suspend(struct device *dev);
static int imx_sec_dsim_runtime_resume(struct device *dev);
#else
static int imx_sec_dsim_runtime_suspend(struct device *dev)
{
	return 0;
}
static int imx_sec_dsim_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static int sec_dsim_rstc_reset(struct reset_control *rstc, bool assert)
{
	int ret;

	if (!rstc)
		return 0;

	ret = assert ? reset_control_assert(rstc)	:
		       reset_control_deassert(rstc);

	return ret;
}

static void imx_sec_dsim_encoder_helper_enable(struct drm_encoder *encoder)
{
	int ret;
	struct imx_sec_dsim_device *dsim_dev = enc_to_dsim(encoder);

	pm_runtime_get_sync(dsim_dev->dev);

	ret = sec_dsim_rstc_reset(dsim_dev->mipi_reset, false);
	if (ret)
		dev_err(dsim_dev->dev, "deassert mipi_reset failed\n");
}

static void imx_sec_dsim_encoder_helper_disable(struct drm_encoder *encoder)
{
	int ret;
	struct imx_sec_dsim_device *dsim_dev = enc_to_dsim(encoder);

	ret = sec_dsim_rstc_reset(dsim_dev->mipi_reset, true);
	if (ret)
		dev_err(dsim_dev->dev, "deassert mipi_reset failed\n");

	pm_runtime_put_sync(dsim_dev->dev);
}

static int imx_sec_dsim_encoder_helper_atomic_check(struct drm_encoder *encoder,
						    struct drm_crtc_state *crtc_state,
						    struct drm_connector_state *conn_state)
{
	int ret;
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct drm_bridge *bridge = drm_bridge_chain_get_first_bridge(encoder);
	struct drm_bridge_state *bridge_state;
	struct drm_bus_cfg *input_bus_cfg;

	/* check pll out */
	ret = sec_mipi_dsim_check_pll_out(bridge->driver_private,
					  adjusted_mode);
	if (ret)
		return ret;

	bridge_state = drm_atomic_get_new_bridge_state(crtc_state->state,
						       bridge);

	if (WARN_ON(!bridge_state))
		return -ENODEV;

	input_bus_cfg = &bridge_state->input_bus_cfg;

	imx_crtc_state->bus_format = input_bus_cfg->format;
	imx_crtc_state->bus_flags  = input_bus_cfg->flags;

	return 0;
}

static const struct drm_encoder_helper_funcs imx_sec_dsim_encoder_helper_funcs = {
	.enable  = imx_sec_dsim_encoder_helper_enable,
	.disable = imx_sec_dsim_encoder_helper_disable,
	.atomic_check = imx_sec_dsim_encoder_helper_atomic_check,
};

static int sec_dsim_determine_pll_ref_rate(u32 *rate, u32 min, u32 max)
{
	int ret;
	struct device *dev = dsim_dev->dev;
	u32 req_rate = PHY_REF_CLK;
	unsigned long get_rate;

	ret = of_property_read_u32(dev->of_node, "pref-rate", &req_rate);
	if (!ret) {
		if (req_rate != clamp(req_rate, min, max)) {
			dev_warn(dev, "invalid requested PLL ref clock rate : %u\n", req_rate);
			req_rate = PHY_REF_CLK;
			dev_warn(dev, "use default clock rate : %u\n", req_rate);
		}
	}

set_rate:
	ret = clk_set_rate(dsim_dev->clk_pllref, ((unsigned long)req_rate) * 1000);
	if (ret)
		return ret;

	get_rate = clk_get_rate(dsim_dev->clk_pllref);
	if (!get_rate)
		return -EINVAL;

	/* PLL ref clock rate should be set precisely */
	if (get_rate != req_rate * 1000) {
		/* default clock rate should can be set precisely */
		if (WARN_ON(unlikely(req_rate == PHY_REF_CLK)))
			return -EINVAL;

		dev_warn(dev, "request rate %u cannot be satisfied\n", req_rate);
		req_rate = PHY_REF_CLK;
		dev_warn(dev, "use default clock rate : %u\n", req_rate);

		goto set_rate;
	}

	*rate = req_rate;

	return 0;
}

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.dphy_pll	= &pll_1432x,
	.dphy_timing	= dphy_timing_ln14lpp_v1p2,
	.num_dphy_timing = ARRAY_SIZE(dphy_timing_ln14lpp_v1p2),
	.dphy_timing_cmp = dphy_timing_default_cmp,
	.mode_valid	= NULL,
	.determine_pll_ref_rate = sec_dsim_determine_pll_ref_rate,
};

static const struct of_device_id imx_sec_dsim_dt_ids[] = {
	{
		.compatible = "fsl,imx8mm-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{
		.compatible = "fsl,imx8mn-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{
		.compatible = "fsl,imx8mp-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_sec_dsim_dt_ids);

static int sec_dsim_of_parse_resets(struct imx_sec_dsim_device *dsim)
{
	int ret;
	struct device *dev = dsim->dev;
	struct device_node *np = dev->of_node;
	struct device_node *parent, *child;
	struct of_phandle_args args;
	struct reset_control *rstc;
	const char *compat;
	uint32_t len, rstc_num = 0;

	/* TODO: bypass resets for imx8mp platform */
	compat = of_get_property(np, "compatible", NULL);
	if (unlikely(!compat))
		return -ENODEV;

	len = strlen(compat);
	if (!of_compat_cmp(compat, "fsl,imx8mp-mipi-dsim", len))
		return 0;

	ret = of_parse_phandle_with_args(np, "resets", "#reset-cells",
					 0, &args);
	if (ret)
		return ret;

	parent = args.np;
	for_each_child_of_node(parent, child) {
		compat = of_get_property(child, "compatible", NULL);
		if (!compat)
			continue;

		rstc = of_reset_control_array_get(child, false, false, true);
		if (IS_ERR(rstc))
			continue;

		len = strlen(compat);
		if (!of_compat_cmp("dsi,soft-resetn", compat, len)) {
			dsim->soft_resetn = rstc;
			rstc_num++;
		} else if (!of_compat_cmp("dsi,clk-enable", compat, len)) {
			dsim->clk_enable = rstc;
			rstc_num++;
		} else if (!of_compat_cmp("dsi,mipi-reset", compat, len)) {
			dsim->mipi_reset = rstc;
			rstc_num++;
		} else
			dev_warn(dev, "invalid dsim reset node: %s\n", compat);
	}

	if (!rstc_num) {
		dev_err(dev, "no invalid reset control exists\n");
		return -EINVAL;
	}

	return 0;
}

static void sec_dsim_of_put_resets(struct imx_sec_dsim_device *dsim)
{
	if (dsim->soft_resetn)
		reset_control_put(dsim->soft_resetn);

	if (dsim->clk_enable)
		reset_control_put(dsim->clk_enable);

	if (dsim->mipi_reset)
		reset_control_put(dsim->mipi_reset);
}

static int imx_sec_dsim_bind(struct device *dev, struct device *master,
			     void *data)
{
	int ret;
	struct drm_device *drm_dev = data;
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id = of_match_device(imx_sec_dsim_dt_ids,
							   dev);
	const struct sec_mipi_dsim_plat_data *pdata;
	struct drm_encoder *encoder;

	dev_dbg(dev, "%s: dsim bind begin\n", __func__);

	if (!of_id)
		return -ENODEV;
	pdata = of_id->data;

	encoder = &dsim_dev->encoder;
	ret = imx_drm_encoder_parse_of(drm_dev, encoder, np);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &imx_sec_dsim_encoder_helper_funcs);

	ret = drm_simple_encoder_init(drm_dev, encoder, DRM_MODE_ENCODER_DSI);
	if (ret)
		return ret;

	/* bind sec dsim bridge */
	ret = sec_mipi_dsim_bind(dev, master, data, encoder,
				 dsim_dev->base, dsim_dev->irq, pdata);
	if (ret) {
		dev_err(dev, "failed to bind sec dsim bridge: %d\n", ret);
		drm_encoder_cleanup(encoder);

		/* If no panel or bridge connected, just return 0
		 * to make component core to believe it is bound
		 * successfully to allow other components can be
		 * bound continuously, since in component core,
		 * it follows 'one fails, all fail'. It is useful
		 * when there exists multiple heads display.
		 */
		if (ret == -ENODEV)
			return 0;

		return ret;
	}

	dev_dbg(dev, "%s: dsim bind end\n", __func__);

	return 0;
}

static void imx_sec_dsim_unbind(struct device *dev, struct device *master,
				void *data)
{
	if (!dsim_dev->encoder.dev)
		return;

	drm_encoder_cleanup(&dsim_dev->encoder);

	sec_mipi_dsim_unbind(dev, master, data);
}

static const struct component_ops imx_sec_dsim_ops = {
	.bind	= imx_sec_dsim_bind,
	.unbind	= imx_sec_dsim_unbind,
};

static int imx_sec_dsim_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "%s: dsim probe begin\n", __func__);

	dsim_dev = devm_kzalloc(dev, sizeof(*dsim_dev), GFP_KERNEL);
	if (!dsim_dev) {
		dev_err(dev, "Unable to allocate 'dsim_dev'\n");
		return -ENOMEM;
	}
	dsim_dev->dev = dev;

	dsim_dev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsim_dev->base))
		return PTR_ERR(dsim_dev->base);

	dsim_dev->irq = platform_get_irq(pdev, 0);
	if (dsim_dev->irq < 0)
		return -ENODEV;

	dsim_dev->clk_cfg = devm_clk_get(dev, "cfg");
	if (IS_ERR(dsim_dev->clk_cfg))
		return PTR_ERR(dsim_dev->clk_cfg);

	dsim_dev->clk_pllref = devm_clk_get(dev, "pll-ref");
	if (IS_ERR(dsim_dev->clk_pllref))
		return PTR_ERR(dsim_dev->clk_pllref);

	ret = sec_dsim_of_parse_resets(dsim_dev);
	if (ret)
		return ret;

	atomic_set(&dsim_dev->rpm_suspended, 1);

	pm_runtime_enable(dev);

	return component_add(dev, &imx_sec_dsim_ops);
}

static int imx_sec_dsim_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx_sec_dsim_ops);
	pm_runtime_disable(&pdev->dev);
	sec_dsim_of_put_resets(dsim_dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int imx_sec_dsim_suspend(struct device *dev)
{
	return imx_sec_dsim_runtime_suspend(dev);
}

static int imx_sec_dsim_resume(struct device *dev)
{
	return imx_sec_dsim_runtime_resume(dev);
}
#endif

#ifdef CONFIG_PM
static int imx_sec_dsim_runtime_suspend(struct device *dev)
{
	/* check sec dsim is bound or not */
	if (unlikely(!dsim_dev->encoder.dev))
		return 0;

	if (atomic_inc_return(&dsim_dev->rpm_suspended) > 1)
		return 0;

	sec_mipi_dsim_suspend(dev);

	clk_disable_unprepare(dsim_dev->clk_cfg);
	clk_disable_unprepare(dsim_dev->clk_pllref);

	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int imx_sec_dsim_runtime_resume(struct device *dev)
{
	int ret;

	/* check sec dsim is bound or not */
	if (unlikely(!dsim_dev->encoder.dev))
		return 0;

	if (unlikely(!atomic_read(&dsim_dev->rpm_suspended))) {
		dev_warn(dsim_dev->dev,
			 "Unbalanced %s!\n", __func__);
		return 0;
	}

	if (!atomic_dec_and_test(&dsim_dev->rpm_suspended))
		return 0;

	request_bus_freq(BUS_FREQ_HIGH);

	ret = clk_prepare_enable(dsim_dev->clk_pllref);
	if (WARN_ON(unlikely(ret)))
		return ret;

	ret = clk_prepare_enable(dsim_dev->clk_cfg);
	if (WARN_ON(unlikely(ret)))
		return ret;

	ret = sec_dsim_rstc_reset(dsim_dev->soft_resetn, false);
	if (ret) {
		dev_err(dev, "deassert soft_resetn failed\n");
		return ret;
	}

	ret = sec_dsim_rstc_reset(dsim_dev->clk_enable, true);
	if (ret) {
		dev_err(dev, "assert clk_enable failed\n");
		return ret;
	}

	ret = sec_dsim_rstc_reset(dsim_dev->mipi_reset, false);
	if (ret) {
		dev_err(dev, "deassert mipi_reset failed\n");
		return ret;
	}

	sec_mipi_dsim_resume(dev);

	return 0;
}
#endif

static const struct dev_pm_ops imx_sec_dsim_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(imx_sec_dsim_suspend,
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
