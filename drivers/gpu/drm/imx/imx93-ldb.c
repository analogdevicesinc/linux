// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"

#define DRIVER_NAME "imx93-ldb"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_REG_CH0_FIFO_RESET		(1 << 11)
#define LDB_REG_ASYNC_FIFO_EN		(1 << 24)
#define LDB_FIFO_THRESHOLD		(4 << 25)

struct imx93_ldb;

struct imx93_ldb_channel {
	struct ldb_channel base;
	struct imx93_ldb *imx93_ldb;

	struct drm_encoder encoder;

	struct phy *phy;
	bool phy_is_on;

	u32 bus_flags;
};

static inline struct imx93_ldb_channel *
enc_to_imx93_ldb_ch(struct drm_encoder *e)
{
	return container_of(e, struct imx93_ldb_channel, encoder);
}

struct imx93_ldb {
	struct ldb base;
	struct imx93_ldb_channel channel;
	struct clk *clk_root;
};

static void imx93_ldb_encoder_enable(struct drm_encoder *encoder)
{
	struct imx93_ldb_channel *imx93_ldb_ch = enc_to_imx93_ldb_ch(encoder);
	struct imx93_ldb *imx93_ldb = imx93_ldb_ch->imx93_ldb;
	struct ldb *ldb = &imx93_ldb->base;

	clk_prepare_enable(imx93_ldb->clk_root);

	ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
	ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI0;

	phy_power_on(imx93_ldb_ch->phy);
	imx93_ldb_ch->phy_is_on = true;
}

static void
imx93_ldb_encoder_atomic_mode_set(struct drm_encoder *encoder,
				  struct drm_crtc_state *crtc_state,
				  struct drm_connector_state *connector_state)
{
	struct imx93_ldb_channel *imx93_ldb_ch = enc_to_imx93_ldb_ch(encoder);
	struct imx93_ldb *imx93_ldb = imx93_ldb_ch->imx93_ldb;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	unsigned long serial_clk;

	serial_clk = mode->clock * 7000UL;
	clk_set_rate(imx93_ldb->clk_root, serial_clk);
}

static void imx93_ldb_encoder_disable(struct drm_encoder *encoder)
{
	struct imx93_ldb_channel *imx93_ldb_ch = enc_to_imx93_ldb_ch(encoder);
	struct imx93_ldb *imx93_ldb = imx93_ldb_ch->imx93_ldb;

	phy_power_off(imx93_ldb_ch->phy);
	imx93_ldb_ch->phy_is_on = false;

	clk_disable_unprepare(imx93_ldb->clk_root);
}

static int
imx93_ldb_encoder_atomic_check(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx93_ldb_channel *imx93_ldb_ch = enc_to_imx93_ldb_ch(encoder);
	struct ldb_channel *ldb_ch = &imx93_ldb_ch->base;
	struct drm_bridge_state *bridge_state = NULL;
	struct drm_bridge *bridge;

	bridge = drm_bridge_chain_get_first_bridge(encoder);
	bridge_state = drm_atomic_get_new_bridge_state(crtc_state->state, bridge);

	if (!ldb_ch->bus_format)
		ldb_ch->bus_format = bridge_state->output_bus_cfg.format;

	imx_crtc_state->bus_flags = bridge_state->input_bus_cfg.flags;

	switch (ldb_ch->bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB666_1X18;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum
drm_mode_status imx93_ldb_mode_valid(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode)
{
	struct imx93_ldb_channel *imx93_ldb_ch = enc_to_imx93_ldb_ch(encoder);
	struct imx93_ldb *imx93_ldb = imx93_ldb_ch->imx93_ldb;
	unsigned long serial_rate;

	if (mode->clock > 80000)
		return MODE_CLOCK_HIGH;

	if (imx93_ldb_ch->base.panel)
		return MODE_OK;

	serial_rate = mode->clock * 7000UL;

	if (serial_rate == clk_round_rate(imx93_ldb->clk_root, serial_rate))
		return MODE_OK;

	return MODE_BAD;
}

static const struct drm_encoder_helper_funcs imx93_ldb_encoder_helper_funcs = {
	.atomic_mode_set = imx93_ldb_encoder_atomic_mode_set,
	.enable = imx93_ldb_encoder_enable,
	.disable = imx93_ldb_encoder_disable,
	.atomic_check = imx93_ldb_encoder_atomic_check,
	.mode_valid = imx93_ldb_mode_valid,
};

static const struct of_device_id imx93_ldb_dt_ids[] = {
	{ .compatible = "fsl,imx93-ldb", },
	{ }
};
MODULE_DEVICE_TABLE(of, imx93_ldb_dt_ids);

static int
imx93_ldb_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct imx93_ldb *imx93_ldb = dev_get_drvdata(dev);
	struct ldb *ldb = &imx93_ldb->base;
	struct imx93_ldb_channel *imx93_ldb_ch = &imx93_ldb->channel;
	struct ldb_channel *ldb_ch = &imx93_ldb_ch->base;
	struct drm_encoder *encoder[1];
	int ret;
	int i;

	ret = of_get_child_count(np);
	if (ret > 1) {
		dev_err(dev, "invalid child/LVDS channel count: %d\n", ret);
		return -EINVAL;
	}

	ldb->dev = dev;
	ldb->ctrl_reg = 0x20;
	ldb->output_port = 1;

	imx93_ldb_ch->imx93_ldb = imx93_ldb;
	ldb->channel[0] = ldb_ch;

	imx93_ldb->clk_root = devm_clk_get(dev, "ldb");
	if (IS_ERR(imx93_ldb->clk_root))
		return PTR_ERR(imx93_ldb->clk_root);

	*encoder = &imx93_ldb_ch->encoder;
	drm_encoder_helper_add(*encoder, &imx93_ldb_encoder_helper_funcs);
	drm_simple_encoder_init(drm, *encoder, DRM_MODE_ENCODER_LVDS);

	pm_runtime_enable(dev);

	ret = ldb_bind(ldb, encoder);
	if (ret)
		goto disable_pm_runtime;

	child = of_get_next_child(np, NULL);
	if (!child) {
		dev_err(dev, "no child node for LVDS channel\n");
		ret = -ENODEV;
		goto disable_pm_runtime;
	}

	ret = of_property_read_u32(child, "reg", &i);
	if (ret || i != 0) {
		dev_err(dev, "invalid LVDS channel number\n");
		ret = -EINVAL;
		goto free_child;
	}

	if (!of_device_is_available(child)) {
		dev_info(dev, "LVDS channel is not available\n");
		goto free_child;
	}

	imx93_ldb_ch->phy = devm_of_phy_get(dev, child, "ldb_phy");
	if (IS_ERR(imx93_ldb_ch->phy)) {
		ret = PTR_ERR(imx93_ldb_ch->phy);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get LVDS channel phy: %d\n",
				ret);
		goto free_child;
	}

	ret = phy_init(imx93_ldb_ch->phy);
	if (ret < 0) {
		dev_err(dev, "failed to initialize LVDS channel phy: %d\n",
			ret);
		goto free_child;
	}

	if (!ldb_ch->is_valid) {
		drm_encoder_cleanup(*encoder);
		goto free_child;
	}

	ret = imx_drm_encoder_parse_of(drm, *encoder, child);
	if (ret)
		goto free_child;

	return 0;

free_child:
	of_node_put(child);
disable_pm_runtime:
	pm_runtime_disable(dev);

	return ret;
}

static void imx93_ldb_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct imx93_ldb *imx93_ldb = dev_get_drvdata(dev);
	struct imx93_ldb_channel *imx93_ldb_ch = &imx93_ldb->channel;

	if (imx93_ldb_ch->phy_is_on)
		phy_power_off(imx93_ldb_ch->phy);

	phy_exit(imx93_ldb_ch->phy);

	pm_runtime_disable(dev);
}

static const struct component_ops imx93_ldb_ops = {
	.bind	= imx93_ldb_bind,
	.unbind	= imx93_ldb_unbind,
};

static int imx93_ldb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx93_ldb *imx93_ldb;

	imx93_ldb = devm_kzalloc(dev, sizeof(*imx93_ldb), GFP_KERNEL);
	if (!imx93_ldb)
		return -ENOMEM;

	dev_set_drvdata(dev, imx93_ldb);

	return component_add(dev, &imx93_ldb_ops);
}

static void imx93_ldb_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx93_ldb_ops);
}

#ifdef CONFIG_PM_SLEEP
static int imx93_ldb_suspend(struct device *dev)
{
	struct imx93_ldb *imx93_ldb = dev_get_drvdata(dev);

	if (!imx93_ldb)
		return 0;

	phy_exit(imx93_ldb->channel.phy);

	return 0;
}

static int imx93_ldb_resume(struct device *dev)
{
	struct imx93_ldb *imx93_ldb = dev_get_drvdata(dev);

	if (!imx93_ldb)
		return 0;

	phy_init(imx93_ldb->channel.phy);

	return 0;
}
#endif

static const struct dev_pm_ops imx93_ldb_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(imx93_ldb_suspend, imx93_ldb_resume)
};

static struct platform_driver imx93_ldb_driver = {
	.probe		= imx93_ldb_probe,
	.remove		= imx93_ldb_remove,
	.driver		= {
		.of_match_table = imx93_ldb_dt_ids,
		.name	= DRIVER_NAME,
		.pm	= &imx93_ldb_pm_ops,
	},
};

module_platform_driver(imx93_ldb_driver);

MODULE_DESCRIPTION("i.MX93 LVDS driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
