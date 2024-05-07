// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2022 NXP
 */

#include <linux/component.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"

#define DISPLAY_MUX_CTRL	0x60
#define  PARALLEL_DISP_FORMAT	0x700

enum imx93_pdf_format {
	RGB888_TO_RGB888 = 0x0,
	RGB888_TO_RGB666 = 0x1 << 8,
	RGB565_TO_RGB565 = 0x2 << 8,
};

struct imx93_pdf {
	struct drm_encoder encoder;
	struct device *dev;
	struct regmap *regmap;
	struct drm_bridge *bridge;
	enum imx93_pdf_format format;
	u32 bus_format;
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_state;
	struct pinctrl_state *gpio_state;
};

static inline struct imx93_pdf *enc_to_pdf(struct drm_encoder *e)
{
	return container_of(e, struct imx93_pdf, encoder);
}

static void imx93_pdf_encoder_enable(struct drm_encoder *encoder)
{
	struct imx93_pdf *pdf = enc_to_pdf(encoder);

	regmap_update_bits(pdf->regmap, DISPLAY_MUX_CTRL, PARALLEL_DISP_FORMAT,
			   pdf->format);

	pinctrl_select_state(pdf->pinctrl, pdf->default_state);
}

static void imx93_pdf_encoder_disable(struct drm_encoder *encoder)
{
	struct imx93_pdf *pdf = enc_to_pdf(encoder);

	if (pdf->gpio_state)
		pinctrl_select_state(pdf->pinctrl, pdf->gpio_state);
}

static int
imx93_pdf_encoder_atomic_check(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
	struct imx93_pdf *pdf = enc_to_pdf(encoder);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct drm_display_info *di = &conn_state->connector->display_info;
	u32 bus_format;

	if (!pdf->bus_format)
		pdf->bus_format = di->bus_formats[0];

	switch (pdf->bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		pdf->format = RGB565_TO_RGB565;
		bus_format = pdf->bus_format;
		break;
	case MEDIA_BUS_FMT_RGB666_1X18:
		pdf->format = RGB888_TO_RGB666;
		bus_format = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		pdf->format = RGB888_TO_RGB888;
		bus_format = pdf->bus_format;
		break;
	default:
		dev_dbg(pdf->dev, "invalid bus format 0x%x\n", pdf->bus_format);
		return -EINVAL;
	}

	imx_crtc_state->bus_format = bus_format;
	imx_crtc_state->bus_flags = di->bus_flags;

	return 0;
}

static const struct drm_encoder_helper_funcs imx93_pdf_encoder_helper_funcs = {
	.atomic_check = imx93_pdf_encoder_atomic_check,
	.enable = imx93_pdf_encoder_enable,
	.disable = imx93_pdf_encoder_disable,
};

static int imx93_pdf_register(struct drm_device *drm, struct imx93_pdf *pdf)
{
	struct drm_encoder *encoder = &pdf->encoder;
	int ret;

	ret = imx_drm_encoder_parse_of(drm, encoder, pdf->dev->of_node);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &imx93_pdf_encoder_helper_funcs);
	drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_DPI);

	ret = drm_bridge_attach(encoder, pdf->bridge, NULL, 0);
	if (ret < 0) {
		dev_err(pdf->dev, "failed to attach bridge: %d\n", ret);
		return ret;
	}

	return 0;
}

static int imx93_pdf_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct device_node *np = dev->of_node;
	struct imx93_pdf *pdf = dev_get_drvdata(dev);
	struct drm_panel *panel;
	const char *fmt;
	u32 bus_format = 0;
	int ret;

	pdf->regmap = syscon_node_to_regmap(np->parent);
	if (IS_ERR(pdf->regmap))
		return dev_err_probe(dev, PTR_ERR(pdf->regmap),
				     "failed to get regmap\n");

	pdf->dev = dev;

	ret = of_property_read_string(np, "fsl,interface-pix-fmt", &fmt);
	if (!ret) {
		if (!strcmp(fmt, "rgb565"))
			bus_format = MEDIA_BUS_FMT_RGB565_1X16;
		else if (!strcmp(fmt, "rgb666"))
			bus_format = MEDIA_BUS_FMT_RGB666_1X18;
		else if (!strcmp(fmt, "rgb888"))
			bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	}
	pdf->bus_format = bus_format;

	pdf->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pdf->pinctrl))
		return dev_err_probe(dev, PTR_ERR(pdf->pinctrl),
				     "failed to get pinctrl\n");

	pdf->default_state = pinctrl_lookup_state(pdf->pinctrl, "default");
	if (IS_ERR(pdf->default_state)) {
		ret = PTR_ERR(pdf->default_state);
		dev_err(dev, "failed to find default pinctrl state: %d\n", ret);
		return ret;
	}

	pdf->gpio_state = pinctrl_lookup_state(pdf->pinctrl, "gpio");
	if (IS_ERR(pdf->gpio_state)) {
		ret = PTR_ERR(pdf->gpio_state);
		dev_dbg(dev, "failed to find gpio pinctrl state: %d\n", ret);
		pdf->gpio_state = NULL;
	}

	if (pdf->gpio_state)
		pinctrl_select_state(pdf->pinctrl, pdf->gpio_state);

	/* port@1 is the output port */
	ret = drm_of_find_panel_or_bridge(np, 1, 0, &panel, &pdf->bridge);
	if (ret)
		return ret;

	if (panel) {
		pdf->bridge = devm_drm_panel_bridge_add(dev, panel);
		if (IS_ERR(pdf->bridge)) {
			ret = PTR_ERR(pdf->bridge);
			dev_err(dev, "failed to add panel bridge %d\n", ret);
			return ret;
		}
	}

	return imx93_pdf_register(drm, pdf);
}

static const struct component_ops imx93_pdf_ops = {
	.bind = imx93_pdf_bind,
};

static int imx93_pdf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx93_pdf *pdf;

	pdf = devm_kzalloc(dev, sizeof(*pdf), GFP_KERNEL);
	if (!pdf)
		return -ENOMEM;

	dev_set_drvdata(dev, pdf);

	return component_add(dev, &imx93_pdf_ops);
}

static int imx93_pdf_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx93_pdf_ops);

	return 0;
}

static const struct of_device_id imx93_pdf_dt_ids[] = {
	{ .compatible = "fsl,imx93-parallel-display-format", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx93_pdf_dt_ids);

static struct platform_driver imx93_pdf_driver = {
	.probe		= imx93_pdf_probe,
	.remove		= imx93_pdf_remove,
	.driver		= {
		.of_match_table = imx93_pdf_dt_ids,
		.name	= "imx93-parallel-display-format",
	},
};

module_platform_driver(imx93_pdf_driver);

MODULE_DESCRIPTION("i.MX93 parallel display format driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx93-parallel-display-format");
