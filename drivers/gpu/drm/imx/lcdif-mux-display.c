/*
 * Copyright 2020 NXP
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
#include <linux/component.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"

#define LCDIF_MUX_MODE_MASK	0x3

enum lcdif_mux_mode {
	LCDIF_MUX_MODE_LCDIF,
	LCDIF_MUX_MODE_PL_RGB888,
	LCDIF_MUX_MODE_PL_RGB666,
	LCDIF_MUX_MODE_PL_RGB565,
};

struct imx_lcdif_mux_display {
	struct drm_encoder encoder;
	struct device *dev;
	struct regmap *regmap;
	struct clk *clk_bypass_div;
	struct clk *clk_pixel;
	struct drm_bridge *bridge;
	u32 bus_format;
	enum lcdif_mux_mode mux_mode;
};

static inline struct imx_lcdif_mux_display *enc_to_lmuxd(struct drm_encoder *e)
{
	return container_of(e, struct imx_lcdif_mux_display, encoder);
}

static void imx_lmuxd_encoder_enable(struct drm_encoder *encoder)
{
	struct imx_lcdif_mux_display *lmuxd = enc_to_lmuxd(encoder);

	clk_prepare_enable(lmuxd->clk_pixel);
}

static void imx_lmuxd_encoder_disable(struct drm_encoder *encoder)
{
	struct imx_lcdif_mux_display *lmuxd = enc_to_lmuxd(encoder);

	clk_disable_unprepare(lmuxd->clk_pixel);
}

static void
imx_lmuxd_encoder_atomic_mode_set(struct drm_encoder *encoder,
				  struct drm_crtc_state *crtc_state,
				  struct drm_connector_state *conn_state)
{
	struct imx_lcdif_mux_display *lmuxd = enc_to_lmuxd(encoder);
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	unsigned long pixel_clk = mode->clock * 1000;

	clk_set_rate(lmuxd->clk_bypass_div, pixel_clk);
	clk_set_rate(lmuxd->clk_pixel, pixel_clk);

	regmap_update_bits(lmuxd->regmap,
			   0x0, LCDIF_MUX_MODE_MASK, lmuxd->mux_mode);
}

static int
imx_lmuxd_encoder_atomic_check(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
	struct imx_lcdif_mux_display *lmuxd = enc_to_lmuxd(encoder);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct drm_display_info *di = &conn_state->connector->display_info;
	u32 bus_format;

	imx_crtc_state->bus_flags = di->bus_flags;

	if (!lmuxd->bus_format)
		lmuxd->bus_format = di->bus_formats[0];

	switch (lmuxd->bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		bus_format = MEDIA_BUS_FMT_RGB565_1X30_PADLO;
		lmuxd->mux_mode = LCDIF_MUX_MODE_PL_RGB565;
		break;
	case MEDIA_BUS_FMT_RGB666_1X18:
		bus_format = MEDIA_BUS_FMT_RGB666_1X30_PADLO;
		lmuxd->mux_mode = LCDIF_MUX_MODE_PL_RGB666;
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		bus_format = MEDIA_BUS_FMT_RGB888_1X30_PADLO;
		lmuxd->mux_mode = LCDIF_MUX_MODE_PL_RGB888;
		break;
	default:
		return -EINVAL;
	}

	imx_crtc_state->bus_format = bus_format;

	return 0;
}

static const struct drm_encoder_helper_funcs imx_lmuxd_encoder_helper_funcs = {
	.enable = imx_lmuxd_encoder_enable,
	.disable = imx_lmuxd_encoder_disable,
	.atomic_mode_set = imx_lmuxd_encoder_atomic_mode_set,
	.atomic_check = imx_lmuxd_encoder_atomic_check,
};

static int imx_lmuxd_register(struct drm_device *drm,
	struct imx_lcdif_mux_display *lmuxd)
{
	struct drm_encoder *encoder = &lmuxd->encoder;
	int ret;

	ret = imx_drm_encoder_parse_of(drm, encoder, lmuxd->dev->of_node);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &imx_lmuxd_encoder_helper_funcs);
	drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_DPI);

	ret = drm_bridge_attach(encoder, lmuxd->bridge, NULL, 0);
	if (ret < 0) {
		dev_err(lmuxd->dev, "failed to attach bridge: %d\n", ret);
		return ret;
	}

	return 0;
}

static int imx_lmuxd_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct device_node *np = dev->of_node;
	struct imx_lcdif_mux_display *lmuxd = dev_get_drvdata(dev);
	struct drm_panel *panel;
	const char *fmt;
	u32 bus_format = 0;
	int ret;

	lmuxd->regmap =
		syscon_regmap_lookup_by_phandle(np, "fsl,lcdif-mux-regs");
	if (IS_ERR(lmuxd->regmap)) {
		dev_err(dev, "failed to get lcdif mux regmap\n");
		return PTR_ERR(lmuxd->regmap);
	}

	lmuxd->clk_bypass_div = devm_clk_get(dev, "bypass_div");
	if (IS_ERR(lmuxd->clk_bypass_div))
		return PTR_ERR(lmuxd->clk_bypass_div);

	lmuxd->clk_pixel = devm_clk_get(dev, "pixel");
	if (IS_ERR(lmuxd->clk_bypass_div))
		return PTR_ERR(lmuxd->clk_bypass_div);

	ret = of_property_read_string(np, "fsl,interface-pix-fmt", &fmt);
	if (!ret) {
		if (!strcmp(fmt, "rgb565"))
			bus_format = MEDIA_BUS_FMT_RGB565_1X16;
		else if (!strcmp(fmt, "rgb666"))
			bus_format = MEDIA_BUS_FMT_RGB666_1X18;
		else if (!strcmp(fmt, "rgb888"))
			bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	}
	lmuxd->bus_format = bus_format;

	/* port@1 is the output port */
	ret = drm_of_find_panel_or_bridge(np, 1, 0, &panel, &lmuxd->bridge);
	if (ret)
		return ret;

	if (panel) {
		lmuxd->bridge = devm_drm_panel_bridge_add(dev, panel);
		if (IS_ERR(lmuxd->bridge)) {
			ret = PTR_ERR(lmuxd->bridge);
			dev_err(dev, "failed to add panel bridge %d\n", ret);
			return ret;
		}
	}

	lmuxd->dev = dev;

	return imx_lmuxd_register(drm, lmuxd);
}

static void imx_lmuxd_unbind(struct device *dev, struct device *master,
	void *data)
{
}

static const struct component_ops imx_lmuxd_ops = {
	.bind	= imx_lmuxd_bind,
	.unbind	= imx_lmuxd_unbind,
};

static int imx_lmuxd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx_lcdif_mux_display *lmuxd;

	lmuxd = devm_kzalloc(dev, sizeof(*lmuxd), GFP_KERNEL);
	if (!lmuxd)
		return -ENOMEM;

	dev_set_drvdata(dev, lmuxd);

	return component_add(dev, &imx_lmuxd_ops);
}

static int imx_lmuxd_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx_lmuxd_ops);

	return 0;
}

static const struct of_device_id imx_lmuxd_dt_ids[] = {
	{ .compatible = "fsl,imx-lcdif-mux-display", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_lmuxd_dt_ids);

static struct platform_driver imx_lmuxd_driver = {
	.probe		= imx_lmuxd_probe,
	.remove		= imx_lmuxd_remove,
	.driver		= {
		.of_match_table = imx_lmuxd_dt_ids,
		.name	= "imx-lcdif-mux-display",
	},
};

module_platform_driver(imx_lmuxd_driver);

MODULE_DESCRIPTION("i.MX LCDIF mux display driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-lcdif-mux-display");
