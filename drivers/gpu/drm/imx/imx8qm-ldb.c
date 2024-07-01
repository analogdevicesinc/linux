// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020,2022 NXP
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/firmware/imx/sci.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mixel-lvds.h>
#include <linux/platform_device.h>

#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"

#define DRIVER_NAME "imx8qm-ldb"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_BIT_MAP_CH0_JEIDA		(1 << 6)
#define LDB_BIT_MAP_CH1_JEIDA		(1 << 8)
#define LDB_CH0_10BIT_EN		(1 << 22)
#define LDB_CH1_10BIT_EN		(1 << 23)
#define LDB_CH0_DATA_WIDTH_24BIT	(1 << 24)
#define LDB_CH1_DATA_WIDTH_24BIT	(1 << 26)
#define LDB_CH0_DATA_WIDTH_30BIT	(2 << 24)
#define LDB_CH1_DATA_WIDTH_30BIT	(2 << 26)

struct imx8qm_ldb;

struct imx8qm_ldb_channel {
	struct ldb_channel base;
	struct imx8qm_ldb *imx8qm_ldb;

	struct drm_encoder encoder;

	struct phy *phy;
	bool phy_is_on;

	u32 bus_flags;
};

static inline struct imx8qm_ldb_channel *
enc_to_imx8qm_ldb_ch(struct drm_encoder *e)
{
	return container_of(e, struct imx8qm_ldb_channel, encoder);
}

struct imx8qm_ldb {
	struct ldb base;
	struct imx8qm_ldb_channel channel[LDB_CH_NUM];
	struct clk *clk_pixel;
	struct clk *clk_bypass;
	struct imx_sc_ipc *handle;

	int id;
};

static void
imx8qm_ldb_ch_set_bus_format(struct imx8qm_ldb_channel *imx8qm_ldb_ch,
			     u32 bus_format)
{
	struct imx8qm_ldb *imx8qm_ldb = imx8qm_ldb_ch->imx8qm_ldb;
	struct ldb *ldb = &imx8qm_ldb->base;
	struct ldb_channel *ldb_ch = &imx8qm_ldb_ch->base;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_DATA_WIDTH_24BIT;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_DATA_WIDTH_24BIT;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_DATA_WIDTH_24BIT;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_DATA_WIDTH_24BIT;
		break;
	case MEDIA_BUS_FMT_RGB101010_1X7X5_SPWG:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_10BIT_EN |
					 LDB_CH0_DATA_WIDTH_30BIT;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_10BIT_EN |
					 LDB_CH1_DATA_WIDTH_30BIT;
		break;
	case MEDIA_BUS_FMT_RGB101010_1X7X5_JEIDA:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_10BIT_EN |
					 LDB_CH0_DATA_WIDTH_30BIT |
					 LDB_BIT_MAP_CH0_JEIDA;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_10BIT_EN |
					 LDB_CH1_DATA_WIDTH_30BIT |
					 LDB_BIT_MAP_CH1_JEIDA;
		break;
	}
}

static void imx8qm_ldb_pxlink_set_mst_valid(struct imx8qm_ldb *imx8qm_ldb,
					    int dc_id, bool enable)
{
	u32 rsc = dc_id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;

	imx_sc_misc_set_control(imx8qm_ldb->handle,
				rsc, IMX_SC_C_PXL_LINK_MST2_VLD, enable);
}

static void imx8qm_ldb_pxlink_set_sync_ctrl(struct imx8qm_ldb *imx8qm_ldb,
					    int dc_id, bool enable)
{
	u32 rsc = dc_id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;

	imx_sc_misc_set_control(imx8qm_ldb->handle,
				rsc, IMX_SC_C_SYNC_CTRL1, enable);
}

static void imx8qm_ldb_encoder_enable(struct drm_encoder *encoder)
{
	struct imx8qm_ldb_channel *imx8qm_ldb_ch =
						enc_to_imx8qm_ldb_ch(encoder);
	struct imx8qm_ldb *imx8qm_ldb = imx8qm_ldb_ch->imx8qm_ldb;
	struct ldb *ldb = &imx8qm_ldb->base;

	clk_prepare_enable(imx8qm_ldb->clk_pixel);
	clk_prepare_enable(imx8qm_ldb->clk_bypass);

	if (imx8qm_ldb_ch == &imx8qm_ldb->channel[0] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
		ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI0;
	}
	if (imx8qm_ldb_ch == &imx8qm_ldb->channel[1] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;
		ldb->ldb_ctrl |= ldb->dual ?
				LDB_CH1_MODE_EN_TO_DI0 : LDB_CH1_MODE_EN_TO_DI1;
	}

	if (ldb->dual) {
		phy_power_on(imx8qm_ldb->channel[0].phy);
		phy_power_on(imx8qm_ldb->channel[1].phy);

		imx8qm_ldb->channel[0].phy_is_on = true;
		imx8qm_ldb->channel[1].phy_is_on = true;
	} else {
		phy_power_on(imx8qm_ldb_ch->phy);

		imx8qm_ldb_ch->phy_is_on = true;
	}

	imx8qm_ldb_pxlink_set_mst_valid(imx8qm_ldb, imx8qm_ldb->id, true);
	imx8qm_ldb_pxlink_set_sync_ctrl(imx8qm_ldb, imx8qm_ldb->id, true);
}

static void
imx8qm_ldb_encoder_atomic_mode_set(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *connector_state)
{
	struct imx8qm_ldb_channel *imx8qm_ldb_ch =
						enc_to_imx8qm_ldb_ch(encoder);
	struct imx8qm_ldb *imx8qm_ldb = imx8qm_ldb_ch->imx8qm_ldb;
	struct ldb_channel *ldb_ch = &imx8qm_ldb_ch->base;
	struct ldb *ldb = &imx8qm_ldb->base;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	unsigned long di_clk = mode->clock * 1000;

	if (mode->clock > 300000) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 300 MHz pixel clock\n", __func__);
	}
	if (mode->clock > 150000 && !ldb->dual) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 150 MHz pixel clock\n", __func__);
	}

	clk_set_rate(imx8qm_ldb->clk_bypass, di_clk);
	clk_set_rate(imx8qm_ldb->clk_pixel, di_clk);

	if (ldb->dual) {
		mixel_phy_lvds_set_phy_speed(imx8qm_ldb->channel[0].phy,
								di_clk / 2);
		mixel_phy_lvds_set_phy_speed(imx8qm_ldb->channel[1].phy,
								di_clk / 2);
	} else {
		mixel_phy_lvds_set_phy_speed(imx8qm_ldb_ch->phy, di_clk);
	}

	if (ldb->dual) {
		/* VSYNC */
		if (mode->flags & DRM_MODE_FLAG_NVSYNC) {
			mixel_phy_lvds_set_vsync_pol(imx8qm_ldb->channel[0].phy,
						     false);
			mixel_phy_lvds_set_vsync_pol(imx8qm_ldb->channel[1].phy,
						     false);
		} else if (mode->flags & DRM_MODE_FLAG_PVSYNC) {
			mixel_phy_lvds_set_vsync_pol(imx8qm_ldb->channel[0].phy,
						     true);
			mixel_phy_lvds_set_vsync_pol(imx8qm_ldb->channel[1].phy,
						     true);
		}
		/* HSYNC */
		if (mode->flags & DRM_MODE_FLAG_NHSYNC) {
			mixel_phy_lvds_set_hsync_pol(imx8qm_ldb->channel[0].phy,
						     false);
			mixel_phy_lvds_set_hsync_pol(imx8qm_ldb->channel[1].phy,
						     false);
		} else if (mode->flags & DRM_MODE_FLAG_PHSYNC) {
			mixel_phy_lvds_set_hsync_pol(imx8qm_ldb->channel[0].phy,
						     true);
			mixel_phy_lvds_set_hsync_pol(imx8qm_ldb->channel[1].phy,
						     true);
		}
	} else {
		/* VSYNC */
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			mixel_phy_lvds_set_vsync_pol(imx8qm_ldb_ch->phy, false);
		else if (mode->flags & DRM_MODE_FLAG_PVSYNC)
			mixel_phy_lvds_set_vsync_pol(imx8qm_ldb_ch->phy, true);
		/* HSYNC */
		if (mode->flags & DRM_MODE_FLAG_NHSYNC)
			mixel_phy_lvds_set_hsync_pol(imx8qm_ldb_ch->phy, false);
		else if (mode->flags & DRM_MODE_FLAG_PHSYNC)
			mixel_phy_lvds_set_hsync_pol(imx8qm_ldb_ch->phy, true);
	}

	if (!ldb_ch->bus_format) {
		struct drm_connector *connector = connector_state->connector;
		struct drm_display_info *di = &connector->display_info;

		if (di->num_bus_formats)
			ldb_ch->bus_format = di->bus_formats[0];
	}
	imx8qm_ldb_ch_set_bus_format(imx8qm_ldb_ch, ldb_ch->bus_format);
}

static void imx8qm_ldb_encoder_disable(struct drm_encoder *encoder)
{
	struct imx8qm_ldb_channel *imx8qm_ldb_ch =
						enc_to_imx8qm_ldb_ch(encoder);
	struct imx8qm_ldb *imx8qm_ldb = imx8qm_ldb_ch->imx8qm_ldb;
	struct ldb *ldb = &imx8qm_ldb->base;

	imx8qm_ldb_pxlink_set_mst_valid(imx8qm_ldb, imx8qm_ldb->id, false);
	imx8qm_ldb_pxlink_set_sync_ctrl(imx8qm_ldb, imx8qm_ldb->id, false);

	if (ldb->dual) {
		phy_power_off(imx8qm_ldb->channel[0].phy);
		phy_power_off(imx8qm_ldb->channel[1].phy);

		imx8qm_ldb->channel[0].phy_is_on = false;
		imx8qm_ldb->channel[1].phy_is_on = false;
	} else {
		phy_power_off(imx8qm_ldb_ch->phy);

		imx8qm_ldb_ch->phy_is_on = false;
	}

	clk_disable_unprepare(imx8qm_ldb->clk_bypass);
	clk_disable_unprepare(imx8qm_ldb->clk_pixel);
}

static int
imx8qm_ldb_encoder_atomic_check(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx8qm_ldb_channel *imx8qm_ldb_ch =
						enc_to_imx8qm_ldb_ch(encoder);
	struct ldb_channel *ldb_ch = &imx8qm_ldb_ch->base;
	struct drm_display_info *di = &conn_state->connector->display_info;
	u32 bus_format = ldb_ch->bus_format;

	/* Bus format description in DT overrides connector display info. */
	if (!bus_format && di->num_bus_formats) {
		bus_format = di->bus_formats[0];
		imx_crtc_state->bus_flags = di->bus_flags;
	} else {
		bus_format = ldb_ch->bus_format;
		imx_crtc_state->bus_flags = imx8qm_ldb_ch->bus_flags;
	}
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB666_1X30_PADLO;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB888_1X30_PADLO;
		break;
	case MEDIA_BUS_FMT_RGB101010_1X7X5_SPWG:
	case MEDIA_BUS_FMT_RGB101010_1X7X5_JEIDA:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB101010_1X30;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct drm_encoder_helper_funcs imx8qm_ldb_encoder_helper_funcs = {
	.atomic_mode_set = imx8qm_ldb_encoder_atomic_mode_set,
	.enable = imx8qm_ldb_encoder_enable,
	.disable = imx8qm_ldb_encoder_disable,
	.atomic_check = imx8qm_ldb_encoder_atomic_check,
};

static const struct of_device_id imx8qm_ldb_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-ldb", },
	{ }
};
MODULE_DEVICE_TABLE(of, imx8qm_ldb_dt_ids);

static int
imx8qm_ldb_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct imx8qm_ldb *imx8qm_ldb = dev_get_drvdata(dev);
	struct ldb *ldb;
	struct ldb_channel *ldb_ch;
	struct drm_encoder *encoder[LDB_CH_NUM];
	int ret;
	int i;

	ldb = &imx8qm_ldb->base;
	ldb->dev = dev;
	ldb->ctrl_reg = 0xe0;
	ldb->output_port = 1;

	for (i = 0; i < LDB_CH_NUM; i++) {
		imx8qm_ldb->channel[i].imx8qm_ldb = imx8qm_ldb;
		ldb->channel[i] = &imx8qm_ldb->channel[i].base;
	}

	ret = imx_scu_get_handle(&imx8qm_ldb->handle);
	if (ret) {
		dev_err(dev, "failed to get scu ipc handle %d\n", ret);
		return ret;
	}

	imx8qm_ldb->id = of_alias_get_id(np, "ldb");

	imx8qm_ldb->clk_pixel = devm_clk_get(dev, "pixel");
	if (IS_ERR(imx8qm_ldb->clk_pixel))
		return PTR_ERR(imx8qm_ldb->clk_pixel);

	imx8qm_ldb->clk_bypass = devm_clk_get(dev, "bypass");
	if (IS_ERR(imx8qm_ldb->clk_bypass))
		return PTR_ERR(imx8qm_ldb->clk_bypass);

	for_each_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0 || i > 1)
			return -EINVAL;

		if (!of_device_is_available(child))
			continue;

		encoder[i] = &imx8qm_ldb->channel[i].encoder;

		drm_encoder_helper_add(encoder[i],
				      &imx8qm_ldb_encoder_helper_funcs);
		drm_simple_encoder_init(drm, encoder[i], DRM_MODE_ENCODER_LVDS);
	}

	pm_runtime_enable(dev);

	ret = ldb_bind(ldb, encoder);
	if (ret)
		goto disable_pm_runtime;

	for_each_child_of_node(np, child) {
		struct imx8qm_ldb_channel *imx8qm_ldb_ch;
		bool auxiliary_ch = false;

		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0 || i > 1) {
			ret = -EINVAL;
			goto free_child;
		}

		if (ldb->dual && i > 0) {
			auxiliary_ch = true;
			imx8qm_ldb_ch = &imx8qm_ldb->channel[i];
			goto get_phy;
		}

		if (!of_device_is_available(child))
			continue;

		imx8qm_ldb_ch = &imx8qm_ldb->channel[i];
get_phy:
		imx8qm_ldb_ch->phy = devm_of_phy_get(dev, child, "ldb_phy");
		if (IS_ERR(imx8qm_ldb_ch->phy)) {
			ret = PTR_ERR(imx8qm_ldb_ch->phy);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "can't get channel%d phy: %d\n",
					i, ret);
			goto free_child;
		}

		ret = phy_init(imx8qm_ldb_ch->phy);
		if (ret < 0) {
			dev_err(dev, "failed to initialize channel%d phy: %d\n",
				i, ret);
			goto free_child;
		}

		if (auxiliary_ch)
			continue;
	}

	for (i = 0; i < LDB_CH_NUM; i++) {
		ldb_ch = &imx8qm_ldb->channel[i].base;

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

static void imx8qm_ldb_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct imx8qm_ldb *imx8qm_ldb = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < LDB_CH_NUM; i++) {
		struct imx8qm_ldb_channel *imx8qm_ldb_ch =
						&imx8qm_ldb->channel[i];

		if (imx8qm_ldb_ch->phy_is_on)
			phy_power_off(imx8qm_ldb_ch->phy);

		phy_exit(imx8qm_ldb_ch->phy);
	}

	pm_runtime_disable(dev);
}

static const struct component_ops imx8qm_ldb_ops = {
	.bind	= imx8qm_ldb_bind,
	.unbind	= imx8qm_ldb_unbind,
};

static int imx8qm_ldb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx8qm_ldb *imx8qm_ldb;

	imx8qm_ldb = devm_kzalloc(dev, sizeof(*imx8qm_ldb), GFP_KERNEL);
	if (!imx8qm_ldb)
		return -ENOMEM;

	dev_set_drvdata(dev, imx8qm_ldb);

	return component_add(dev, &imx8qm_ldb_ops);
}

static void imx8qm_ldb_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx8qm_ldb_ops);
}

#ifdef CONFIG_PM_SLEEP
static int imx8qm_ldb_suspend(struct device *dev)
{
	struct imx8qm_ldb *imx8qm_ldb = dev_get_drvdata(dev);
	int i;

	if (imx8qm_ldb == NULL)
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++)
		phy_exit(imx8qm_ldb->channel[i].phy);

	return 0;
}

static int imx8qm_ldb_resume(struct device *dev)
{
	struct imx8qm_ldb *imx8qm_ldb = dev_get_drvdata(dev);
	int i;

	if (imx8qm_ldb == NULL)
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++)
		phy_init(imx8qm_ldb->channel[i].phy);

	return 0;
}
#endif

static const struct dev_pm_ops imx8qm_ldb_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(imx8qm_ldb_suspend, imx8qm_ldb_resume)
};

static struct platform_driver imx8qm_ldb_driver = {
	.probe		= imx8qm_ldb_probe,
	.remove		= imx8qm_ldb_remove,
	.driver		= {
		.of_match_table = imx8qm_ldb_dt_ids,
		.name	= DRIVER_NAME,
		.pm	= &imx8qm_ldb_pm_ops,
	},
};

module_platform_driver(imx8qm_ldb_driver);

MODULE_DESCRIPTION("i.MX8QM LVDS driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
