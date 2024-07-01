// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020,2022 NXP
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

#define DRIVER_NAME "imx8mp-ldb"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_REG_CH0_FIFO_RESET		(1 << 11)
#define LDB_REG_CH1_FIFO_RESET		(1 << 12)
#define LDB_REG_ASYNC_FIFO_EN		(1 << 24)
#define LDB_FIFO_THRESHOLD		(4 << 25)

struct imx8mp_ldb;

struct imx8mp_ldb_channel {
	struct ldb_channel base;
	struct imx8mp_ldb *imx8mp_ldb;

	struct drm_encoder encoder;

	struct phy *phy;
	bool phy_is_on;

	u32 bus_flags;
};

static inline struct imx8mp_ldb_channel *
enc_to_imx8mp_ldb_ch(struct drm_encoder *e)
{
	return container_of(e, struct imx8mp_ldb_channel, encoder);
}

struct imx8mp_ldb {
	struct ldb base;
	struct imx8mp_ldb_channel channel[LDB_CH_NUM];
	struct clk *clk_root;
};

static void imx8mp_ldb_encoder_enable(struct drm_encoder *encoder)
{
	struct imx8mp_ldb_channel *imx8mp_ldb_ch =
						enc_to_imx8mp_ldb_ch(encoder);
	struct imx8mp_ldb *imx8mp_ldb = imx8mp_ldb_ch->imx8mp_ldb;
	struct ldb *ldb = &imx8mp_ldb->base;

	clk_prepare_enable(imx8mp_ldb->clk_root);

	if (imx8mp_ldb_ch == &imx8mp_ldb->channel[0] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
		ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI0;
	}
	if (imx8mp_ldb_ch == &imx8mp_ldb->channel[1] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;
		ldb->ldb_ctrl |= ldb->dual ?
				LDB_CH1_MODE_EN_TO_DI0 : LDB_CH1_MODE_EN_TO_DI1;
	}

	if (ldb->dual) {
		phy_power_on(imx8mp_ldb->channel[0].phy);
		phy_power_on(imx8mp_ldb->channel[1].phy);

		imx8mp_ldb->channel[0].phy_is_on = true;
		imx8mp_ldb->channel[1].phy_is_on = true;
	} else {
		phy_power_on(imx8mp_ldb_ch->phy);

		imx8mp_ldb_ch->phy_is_on = true;
	}
}

static void
imx8mp_ldb_encoder_atomic_mode_set(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *connector_state)
{
	struct imx8mp_ldb_channel *imx8mp_ldb_ch =
						enc_to_imx8mp_ldb_ch(encoder);
	struct imx8mp_ldb *imx8mp_ldb = imx8mp_ldb_ch->imx8mp_ldb;
	struct ldb *ldb = &imx8mp_ldb->base;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	unsigned long serial_clk;

	if (mode->clock > 160000) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 160 MHz pixel clock\n", __func__);
	}
	if (mode->clock > 80000 && !ldb->dual) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 80 MHz pixel clock\n", __func__);
	}

	serial_clk = mode->clock * (ldb->dual ? 3500UL : 7000UL);
	clk_set_rate(imx8mp_ldb->clk_root, serial_clk);
}

static void imx8mp_ldb_encoder_disable(struct drm_encoder *encoder)
{
	struct imx8mp_ldb_channel *imx8mp_ldb_ch =
						enc_to_imx8mp_ldb_ch(encoder);
	struct imx8mp_ldb *imx8mp_ldb = imx8mp_ldb_ch->imx8mp_ldb;
	struct ldb *ldb = &imx8mp_ldb->base;

	if (ldb->dual) {
		phy_power_off(imx8mp_ldb->channel[0].phy);
		phy_power_off(imx8mp_ldb->channel[1].phy);

		imx8mp_ldb->channel[0].phy_is_on = false;
		imx8mp_ldb->channel[1].phy_is_on = false;
	} else {
		phy_power_off(imx8mp_ldb_ch->phy);

		imx8mp_ldb_ch->phy_is_on = false;
	}

	clk_disable_unprepare(imx8mp_ldb->clk_root);
}

static int
imx8mp_ldb_encoder_atomic_check(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx8mp_ldb_channel *imx8mp_ldb_ch =
						enc_to_imx8mp_ldb_ch(encoder);
	struct ldb_channel *ldb_ch = &imx8mp_ldb_ch->base;
	struct imx8mp_ldb *imx8mp_ldb = imx8mp_ldb_ch->imx8mp_ldb;
	struct ldb *ldb = &imx8mp_ldb->base;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
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

	/*
	 * Due to limited video PLL frequency points on i.MX8mp,
	 * we do mode fixup here in case any mode is unsupported.
	 */
	if (ldb->dual)
		mode->clock = mode->clock > 100000 ? 148500 : 74250;
	else
		mode->clock = 74250;

	return 0;
}

static enum drm_mode_status
imx8mp_ldb_encoder_mode_valid(struct drm_encoder *encoder,
			      const struct drm_display_mode *mode)
{
	struct imx8mp_ldb_channel *imx8mp_ldb_ch =
						enc_to_imx8mp_ldb_ch(encoder);
	struct ldb_channel *ldb_ch = &imx8mp_ldb_ch->base;
	struct imx8mp_ldb *imx8mp_ldb = imx8mp_ldb_ch->imx8mp_ldb;
	struct ldb *ldb = &imx8mp_ldb->base;

	/* it should be okay with a panel */
	if (ldb_ch->panel)
		return MODE_OK;

	/*
	 * Due to limited video PLL frequency points on i.MX8mp,
	 * we do mode valid check here.
	 */
	if (ldb->dual && mode->clock != 74250 && mode->clock != 148500)
		return MODE_NOCLOCK;

	if (!ldb->dual && mode->clock != 74250)
		return MODE_NOCLOCK;

	return MODE_OK;
}

static const struct drm_encoder_helper_funcs imx8mp_ldb_encoder_helper_funcs = {
	.atomic_mode_set = imx8mp_ldb_encoder_atomic_mode_set,
	.enable = imx8mp_ldb_encoder_enable,
	.disable = imx8mp_ldb_encoder_disable,
	.atomic_check = imx8mp_ldb_encoder_atomic_check,
	.mode_valid = imx8mp_ldb_encoder_mode_valid,
};

static const struct of_device_id imx8mp_ldb_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-ldb", },
	{ }
};
MODULE_DEVICE_TABLE(of, imx8mp_ldb_dt_ids);

static int
imx8mp_ldb_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb;
	struct ldb_channel *ldb_ch;
	struct drm_encoder *encoder[LDB_CH_NUM];
	int ret;
	int i;

	ldb = &imx8mp_ldb->base;
	ldb->dev = dev;
	ldb->ctrl_reg = 0x5c,
	ldb->output_port = 1;

	for (i = 0; i < LDB_CH_NUM; i++) {
		imx8mp_ldb->channel[i].imx8mp_ldb = imx8mp_ldb;
		ldb->channel[i] = &imx8mp_ldb->channel[i].base;
	}

	imx8mp_ldb->clk_root = devm_clk_get(dev, "ldb");
	if (IS_ERR(imx8mp_ldb->clk_root))
		return PTR_ERR(imx8mp_ldb->clk_root);

	for_each_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0 || i > 1)
			return -EINVAL;

		if (!of_device_is_available(child))
			continue;

		encoder[i] = &imx8mp_ldb->channel[i].encoder;

		drm_encoder_helper_add(encoder[i],
				      &imx8mp_ldb_encoder_helper_funcs);
		drm_simple_encoder_init(drm, encoder[i], DRM_MODE_ENCODER_LVDS);
	}

	pm_runtime_enable(dev);

	ret = ldb_bind(ldb, encoder);
	if (ret)
		goto disable_pm_runtime;

	for_each_child_of_node(np, child) {
		struct imx8mp_ldb_channel *imx8mp_ldb_ch;
		bool auxiliary_ch = false;

		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0 || i > 1) {
			ret = -EINVAL;
			goto free_child;
		}

		if (ldb->dual && i > 0) {
			auxiliary_ch = true;
			imx8mp_ldb_ch = &imx8mp_ldb->channel[i];
			goto get_phy;
		}

		if (!of_device_is_available(child))
			continue;

		imx8mp_ldb_ch = &imx8mp_ldb->channel[i];
get_phy:
		imx8mp_ldb_ch->phy = devm_of_phy_get(dev, child, "ldb_phy");
		if (IS_ERR(imx8mp_ldb_ch->phy)) {
			ret = PTR_ERR(imx8mp_ldb_ch->phy);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "can't get channel%d phy: %d\n",
					i, ret);
			goto free_child;
		}

		ret = phy_init(imx8mp_ldb_ch->phy);
		if (ret < 0) {
			dev_err(dev, "failed to initialize channel%d phy: %d\n",
				i, ret);
			goto free_child;
		}

		if (auxiliary_ch)
			continue;
	}

	for (i = 0; i < LDB_CH_NUM; i++) {
		ldb_ch = &imx8mp_ldb->channel[i].base;

		if (!ldb_ch->is_valid)
			continue;

		ret = imx_drm_encoder_parse_of(drm, encoder[i], ldb_ch->child);
		if (ret)
			goto disable_pm_runtime;
	}

	return 0;

free_child:
	of_node_put(child);
disable_pm_runtime:
	pm_runtime_disable(dev);

	return ret;
}

static void imx8mp_ldb_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < LDB_CH_NUM; i++) {
		struct imx8mp_ldb_channel *imx8mp_ldb_ch =
						&imx8mp_ldb->channel[i];

		if (imx8mp_ldb_ch->phy_is_on)
			phy_power_off(imx8mp_ldb_ch->phy);

		phy_exit(imx8mp_ldb_ch->phy);
	}

	pm_runtime_disable(dev);
}

static const struct component_ops imx8mp_ldb_ops = {
	.bind	= imx8mp_ldb_bind,
	.unbind	= imx8mp_ldb_unbind,
};

static int imx8mp_ldb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx8mp_ldb *imx8mp_ldb;

	imx8mp_ldb = devm_kzalloc(dev, sizeof(*imx8mp_ldb), GFP_KERNEL);
	if (!imx8mp_ldb)
		return -ENOMEM;

	dev_set_drvdata(dev, imx8mp_ldb);

	return component_add(dev, &imx8mp_ldb_ops);
}

static void imx8mp_ldb_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx8mp_ldb_ops);
}

#ifdef CONFIG_PM_SLEEP
static int imx8mp_ldb_suspend(struct device *dev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(dev);
	int i;

	if (imx8mp_ldb == NULL)
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++)
		phy_exit(imx8mp_ldb->channel[i].phy);

	return 0;
}

static int imx8mp_ldb_resume(struct device *dev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(dev);
	int i;

	if (imx8mp_ldb == NULL)
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++)
		phy_init(imx8mp_ldb->channel[i].phy);

	return 0;
}
#endif

static const struct dev_pm_ops imx8mp_ldb_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(imx8mp_ldb_suspend, imx8mp_ldb_resume)
};

static struct platform_driver imx8mp_ldb_driver = {
	.probe		= imx8mp_ldb_probe,
	.remove		= imx8mp_ldb_remove,
	.driver		= {
		.of_match_table = imx8mp_ldb_dt_ids,
		.name	= DRIVER_NAME,
		.pm	= &imx8mp_ldb_pm_ops,
	},
};

module_platform_driver(imx8mp_ldb_driver);

MODULE_DESCRIPTION("i.MX8MP LVDS driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
