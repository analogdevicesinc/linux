// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020,2022 NXP
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/firmware/imx/sci.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mixel-lvds-combo.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>

#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"

#define DRIVER_NAME "imx8qxp-ldb"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_BIT_MAP_CH0_JEIDA		(1 << 6)
#define LDB_BIT_MAP_CH1_JEIDA		(1 << 8)
#define LDB_DI0_VS_POL_ACT_LOW		(1 << 9)
#define LDB_DI1_VS_POL_ACT_LOW		(1 << 10)
#define LDB_CH0_10BIT_EN		(1 << 22)
#define LDB_CH1_10BIT_EN		(1 << 23)
#define LDB_CH0_DATA_WIDTH_24BIT	(1 << 24)
#define LDB_CH1_DATA_WIDTH_24BIT	(1 << 26)
#define LDB_CH0_DATA_WIDTH_30BIT	(2 << 24)
#define LDB_CH1_DATA_WIDTH_30BIT	(2 << 26)
#define LDB_CH_SEL			(1 << 28)

struct imx8qxp_ldb;

struct imx8qxp_ldb_channel {
	struct ldb_channel base;
	struct imx8qxp_ldb *imx8qxp_ldb;

	struct drm_encoder encoder;

	struct phy *phy;
	struct phy *aux_phy;
	bool phy_is_on;

	u32 bus_flags;
};

static inline struct imx8qxp_ldb_channel *
enc_to_imx8qxp_ldb_ch(struct drm_encoder *e)
{
	return container_of(e, struct imx8qxp_ldb_channel, encoder);
}

struct imx8qxp_ldb {
	struct ldb base;
	struct regmap *aux_regmap;
	struct imx8qxp_ldb_channel channel[LDB_CH_NUM];
	struct clk *clk_pixel;
	struct clk *clk_bypass;
	struct clk *clk_aux_pixel;
	struct clk *clk_aux_bypass;
	struct imx_sc_ipc *handle;

	struct device *pd_main_dev;
	struct device *pd_aux_dev;
	struct device_link *pd_main_link;
	struct device_link *pd_aux_link;

	int id;
};

static void
imx8qxp_ldb_ch_set_bus_format(struct imx8qxp_ldb_channel *imx8qxp_ldb_ch,
			      u32 bus_format)
{
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb *ldb = &imx8qxp_ldb->base;
	struct ldb_channel *ldb_ch = &imx8qxp_ldb_ch->base;

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

static void imx8qxp_ldb_pxlink_enable(struct imx8qxp_ldb *imx8qxp_ldb,
				      int stream_id, bool enable)
{
	u8 ctrl = stream_id ?
		IMX_SC_C_PXL_LINK_MST2_ENB : IMX_SC_C_PXL_LINK_MST1_ENB;

	imx_sc_misc_set_control(imx8qxp_ldb->handle,
				IMX_SC_R_DC_0, ctrl, enable);
}

static void imx8qxp_ldb_pxlink_set_mst_valid(struct imx8qxp_ldb *imx8qxp_ldb,
					     int stream_id, bool enable)
{
	u8 ctrl = stream_id ?
		IMX_SC_C_PXL_LINK_MST2_VLD : IMX_SC_C_PXL_LINK_MST1_VLD;

	imx_sc_misc_set_control(imx8qxp_ldb->handle,
					IMX_SC_R_DC_0, ctrl, enable);
}

static void imx8qxp_ldb_pxlink_set_sync_ctrl(struct imx8qxp_ldb *imx8qxp_ldb,
					     int stream_id, bool enable)
{
	u8 ctrl = stream_id ? IMX_SC_C_SYNC_CTRL1 : IMX_SC_C_SYNC_CTRL0;

	imx_sc_misc_set_control(imx8qxp_ldb->handle,
					IMX_SC_R_DC_0, ctrl, enable);
}

static void imx8qxp_ldb_encoder_enable(struct drm_encoder *encoder)
{
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb *ldb = &imx8qxp_ldb->base;

	clk_prepare_enable(imx8qxp_ldb->clk_pixel);
	clk_prepare_enable(imx8qxp_ldb->clk_bypass);

	if (ldb->dual) {
		clk_prepare_enable(imx8qxp_ldb->clk_aux_pixel);
		clk_prepare_enable(imx8qxp_ldb->clk_aux_bypass);
	}

	/*
	 * LDB frontend doesn't know if the auxiliary LDB is used or not.
	 * Enable pixel link after dual or single LDB clocks are enabled
	 * so that the dual LDBs are synchronized.
	 */
	imx8qxp_ldb_pxlink_enable(imx8qxp_ldb, imx8qxp_ldb->id, true);

	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[0] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
		ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI0;
	}
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[1] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;
		ldb->ldb_ctrl |= ldb->dual ?
				LDB_CH1_MODE_EN_TO_DI0 : LDB_CH1_MODE_EN_TO_DI1;
	}

	pm_runtime_get_sync(ldb->dev);

	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
	if (ldb->dual)
		regmap_write(imx8qxp_ldb->aux_regmap, ldb->ctrl_reg,
						ldb->ldb_ctrl | LDB_CH_SEL);

	if (ldb->dual) {
		phy_power_on(imx8qxp_ldb->channel[0].phy);
		phy_power_on(imx8qxp_ldb->channel[0].aux_phy);

		imx8qxp_ldb->channel[0].phy_is_on = true;
	} else {
		phy_power_on(imx8qxp_ldb_ch->phy);

		imx8qxp_ldb_ch->phy_is_on = true;
	}

	imx8qxp_ldb_pxlink_set_mst_valid(imx8qxp_ldb, imx8qxp_ldb->id, true);
	imx8qxp_ldb_pxlink_set_sync_ctrl(imx8qxp_ldb, imx8qxp_ldb->id, true);
}

static void
imx8qxp_ldb_encoder_atomic_mode_set(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *connector_state)
{
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb_channel *ldb_ch = &imx8qxp_ldb_ch->base;
	struct ldb *ldb = &imx8qxp_ldb->base;
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

	clk_set_rate(imx8qxp_ldb->clk_bypass, di_clk);
	clk_set_rate(imx8qxp_ldb->clk_pixel, di_clk);

	if (ldb->dual) {
		clk_set_rate(imx8qxp_ldb->clk_aux_bypass, di_clk);
		clk_set_rate(imx8qxp_ldb->clk_aux_pixel, di_clk);
	}

	if (ldb->dual) {
		mixel_phy_combo_lvds_set_phy_speed(imx8qxp_ldb->channel[0].phy,
								di_clk / 2);
		mixel_phy_combo_lvds_set_phy_speed(imx8qxp_ldb->channel[0].aux_phy,
								di_clk / 2);
	} else {
		mixel_phy_combo_lvds_set_phy_speed(imx8qxp_ldb_ch->phy, di_clk);
	}

	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[0])
		ldb->ldb_ctrl &= ~LDB_CH_SEL;
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[1])
		ldb->ldb_ctrl |= LDB_CH_SEL;

	/* FIXME - assumes straight connections DI0 --> CH0, DI1 --> CH1 */
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[0] || ldb->dual) {
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI0_VS_POL_ACT_LOW;
		else if (mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI0_VS_POL_ACT_LOW;
	}
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[1] || ldb->dual) {
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI1_VS_POL_ACT_LOW;
		else if (mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI1_VS_POL_ACT_LOW;
	}

	pm_runtime_get_sync(ldb->dev);

	/* settle vsync polarity and channel selection down early */
	if (ldb->dual) {
		regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
		regmap_write(imx8qxp_ldb->aux_regmap, ldb->ctrl_reg,
						ldb->ldb_ctrl | LDB_CH_SEL);
	}

	pm_runtime_put(ldb->dev);

	if (ldb->dual) {
		/* VSYNC */
		if (mode->flags & DRM_MODE_FLAG_NVSYNC) {
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].phy, false);
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, false);
		} else {
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].phy, true);
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, true);
		}
		/* HSYNC */
		if (mode->flags & DRM_MODE_FLAG_NHSYNC) {
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].phy, false);
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, false);
		} else {
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].phy, true);
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, true);
		}
	} else {
		/* VSYNC */
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			mixel_phy_combo_lvds_set_vsync_pol(imx8qxp_ldb_ch->phy,
							   false);
		else if (mode->flags & DRM_MODE_FLAG_PVSYNC)
			mixel_phy_combo_lvds_set_vsync_pol(imx8qxp_ldb_ch->phy,
							   true);
		/* HSYNC */
		if (mode->flags & DRM_MODE_FLAG_NHSYNC)
			mixel_phy_combo_lvds_set_hsync_pol(imx8qxp_ldb_ch->phy,
							   false);
		else if (mode->flags & DRM_MODE_FLAG_PHSYNC)
			mixel_phy_combo_lvds_set_hsync_pol(imx8qxp_ldb_ch->phy,
							   true);
	}

	if (!ldb_ch->bus_format) {
		struct drm_connector *connector = connector_state->connector;
		struct drm_display_info *di = &connector->display_info;

		if (di->num_bus_formats)
			ldb_ch->bus_format = di->bus_formats[0];
	}
	imx8qxp_ldb_ch_set_bus_format(imx8qxp_ldb_ch, ldb_ch->bus_format);
}

static void imx8qxp_ldb_encoder_disable(struct drm_encoder *encoder)
{
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb *ldb = &imx8qxp_ldb->base;

	imx8qxp_ldb_pxlink_set_mst_valid(imx8qxp_ldb, imx8qxp_ldb->id, false);
	imx8qxp_ldb_pxlink_set_sync_ctrl(imx8qxp_ldb, imx8qxp_ldb->id, false);

	if (ldb->dual) {
		phy_power_off(imx8qxp_ldb->channel[0].phy);
		phy_power_off(imx8qxp_ldb->channel[0].aux_phy);

		imx8qxp_ldb->channel[0].phy_is_on = false;
	} else {
		phy_power_off(imx8qxp_ldb_ch->phy);

		imx8qxp_ldb_ch->phy_is_on = false;
	}

	if (ldb->dual)
		regmap_write(imx8qxp_ldb->aux_regmap,
					ldb->ctrl_reg, ldb->ldb_ctrl);

	pm_runtime_put(ldb->dev);

	clk_disable_unprepare(imx8qxp_ldb->clk_bypass);
	clk_disable_unprepare(imx8qxp_ldb->clk_pixel);

	if (ldb->dual) {
		clk_disable_unprepare(imx8qxp_ldb->clk_aux_bypass);
		clk_disable_unprepare(imx8qxp_ldb->clk_aux_pixel);
	}

	imx8qxp_ldb_pxlink_enable(imx8qxp_ldb, imx8qxp_ldb->id, false);
}

static int
imx8qxp_ldb_encoder_atomic_check(struct drm_encoder *encoder,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct ldb_channel *ldb_ch = &imx8qxp_ldb_ch->base;
	struct drm_display_info *di = &conn_state->connector->display_info;
	u32 bus_format = ldb_ch->bus_format;

	/* Bus format description in DT overrides connector display info. */
	if (!bus_format && di->num_bus_formats) {
		bus_format = di->bus_formats[0];
		imx_crtc_state->bus_flags = di->bus_flags;
	} else {
		bus_format = ldb_ch->bus_format;
		imx_crtc_state->bus_flags = imx8qxp_ldb_ch->bus_flags;
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

static const struct drm_encoder_helper_funcs
imx8qxp_ldb_encoder_helper_funcs = {
	.atomic_mode_set = imx8qxp_ldb_encoder_atomic_mode_set,
	.enable = imx8qxp_ldb_encoder_enable,
	.disable = imx8qxp_ldb_encoder_disable,
	.atomic_check = imx8qxp_ldb_encoder_atomic_check,
};

static const struct of_device_id imx8qxp_ldb_dt_ids[] = {
	{ .compatible = "fsl,imx8qxp-ldb", },
	{ }
};
MODULE_DEVICE_TABLE(of, imx8qxp_ldb_dt_ids);

static void imx8qxp_ldb_detach_pm_domains(struct imx8qxp_ldb *imx8qxp_ldb)
{
	if (imx8qxp_ldb->pd_aux_link && !IS_ERR(imx8qxp_ldb->pd_aux_link))
		device_link_del(imx8qxp_ldb->pd_aux_link);
	if (imx8qxp_ldb->pd_aux_dev && !IS_ERR(imx8qxp_ldb->pd_aux_dev))
		dev_pm_domain_detach(imx8qxp_ldb->pd_aux_dev, true);

	if (imx8qxp_ldb->pd_main_link && !IS_ERR(imx8qxp_ldb->pd_main_link))
		device_link_del(imx8qxp_ldb->pd_main_link);
	if (imx8qxp_ldb->pd_main_dev && !IS_ERR(imx8qxp_ldb->pd_main_dev))
		dev_pm_domain_detach(imx8qxp_ldb->pd_main_dev, true);

	imx8qxp_ldb->pd_aux_dev = NULL;
	imx8qxp_ldb->pd_aux_link = NULL;
	imx8qxp_ldb->pd_main_dev = NULL;
	imx8qxp_ldb->pd_main_link = NULL;
}

static int
imx8qxp_ldb_attach_pm_domains(struct imx8qxp_ldb *imx8qxp_ldb, bool dual)
{
	struct ldb *ldb = &imx8qxp_ldb->base;
	struct device *dev = ldb->dev;
	u32 flags = DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE;
	int ret = 0;

	imx8qxp_ldb->pd_main_dev = dev_pm_domain_attach_by_name(dev, "main");
	if (IS_ERR(imx8qxp_ldb->pd_main_dev)) {
		ret = PTR_ERR(imx8qxp_ldb->pd_main_dev);
		dev_err(dev, "Failed to attach main pd dev: %d\n", ret);
		goto fail;
	}
	imx8qxp_ldb->pd_main_link = device_link_add(dev,
					imx8qxp_ldb->pd_main_dev, flags);
	if (IS_ERR(imx8qxp_ldb->pd_main_link)) {
		ret = PTR_ERR(imx8qxp_ldb->pd_main_link);
		dev_err(dev, "Failed to add device link to main pd dev: %d\n",
			ret);
		goto fail;
	}

	if (!dual)
		goto out;

	imx8qxp_ldb->pd_aux_dev = dev_pm_domain_attach_by_name(dev, "aux");
	if (IS_ERR(imx8qxp_ldb->pd_aux_dev)) {
		ret = PTR_ERR(imx8qxp_ldb->pd_aux_dev);
		dev_err(dev, "Failed to attach aux pd dev: %d\n", ret);
		goto fail;
	}
	imx8qxp_ldb->pd_aux_link = device_link_add(dev,
					imx8qxp_ldb->pd_aux_dev, flags);
	if (IS_ERR(imx8qxp_ldb->pd_aux_link)) {
		ret = PTR_ERR(imx8qxp_ldb->pd_aux_link);
		dev_err(dev, "Failed to add device link to aux pd dev: %d\n",
			ret);
		goto fail;
	}

out:
	return ret;
fail:
	imx8qxp_ldb_detach_pm_domains(imx8qxp_ldb);
	return ret;
}

static int imx8qxp_ldb_init_sc_misc(int ldb_id, bool dual)
{
	struct imx_sc_ipc *handle;
	u32 rsc;
	bool is_aux = false;
	int ret = 0;

	imx_scu_get_handle(&handle);

again:
	rsc = ldb_id ? IMX_SC_R_MIPI_1 : IMX_SC_R_MIPI_0;

	ret |= imx_sc_misc_set_control(handle,
				       rsc, IMX_SC_C_MODE, 1);
	ret |= imx_sc_misc_set_control(handle,
				       rsc, IMX_SC_C_DUAL_MODE, is_aux);
	ret |= imx_sc_misc_set_control(handle,
				       rsc, IMX_SC_C_PXL_LINK_SEL, is_aux);

	if (dual && !is_aux) {
		ldb_id ^= 1;
		is_aux = true;
		goto again;
	}

	return ret;
}

static struct phy *imx8qxp_ldb_get_aux_phy(struct device_node *auxldb_np)
{
	struct device_node *child;
	struct phy *phy = NULL;
	int ret, i;

	for_each_child_of_node(auxldb_np, child) {
		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0) {
			of_node_put(child);
			return ERR_PTR(-ENODEV);
		}

		if (i != 1)
			continue;

		phy = of_phy_get(child, "ldb_phy");
	}

	of_node_put(child);

	return phy;
}

static int
imx8qxp_ldb_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct device_node *np = dev->of_node;
	struct device_node *auxldb_np = NULL, *child;
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb;
	struct ldb_channel *ldb_ch;
	struct drm_encoder *encoder[LDB_CH_NUM];
	bool dual;
	int ret;
	int i;

	ldb = &imx8qxp_ldb->base;
	ldb->dev = dev;
	ldb->ctrl_reg = 0xe0;
	ldb->output_port = 1;

	for (i = 0; i < LDB_CH_NUM; i++) {
		imx8qxp_ldb->channel[i].imx8qxp_ldb = imx8qxp_ldb;
		ldb->channel[i] = &imx8qxp_ldb->channel[i].base;
	}

	ret = imx_scu_get_handle(&imx8qxp_ldb->handle);
	if (ret) {
		dev_err(dev, "failed to get scu ipc handle %d\n", ret);
		return ret;
	}

	imx8qxp_ldb->id = of_alias_get_id(np, "ldb");

	for_each_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0 || i > 1)
			return -EINVAL;

		if (!of_device_is_available(child))
			continue;

		encoder[i] = &imx8qxp_ldb->channel[i].encoder;

		drm_encoder_helper_add(encoder[i],
				      &imx8qxp_ldb_encoder_helper_funcs);
		drm_simple_encoder_init(drm, encoder[i], DRM_MODE_ENCODER_LVDS);
	}

	dual = of_property_read_bool(np, "fsl,dual-channel");

	ret = imx8qxp_ldb_attach_pm_domains(imx8qxp_ldb, dual);
	if (ret) {
		dev_err(dev, "failed to attach pm domains %d\n", ret);
		return ret;
	}

	pm_runtime_enable(dev);

	ret = ldb_bind(ldb, encoder);
	if (ret)
		goto disable_pm_runtime;

	ret = imx8qxp_ldb_init_sc_misc(imx8qxp_ldb->id, ldb->dual);
	if (ret) {
		dev_err(dev, "failed to initialize sc misc %d\n", ret);
		goto disable_pm_runtime;
	}

	imx8qxp_ldb->clk_pixel = devm_clk_get(dev, "pixel");
	if (IS_ERR(imx8qxp_ldb->clk_pixel)) {
		ret = PTR_ERR(imx8qxp_ldb->clk_pixel);
		goto disable_pm_runtime;
	}

	imx8qxp_ldb->clk_bypass = devm_clk_get(dev, "bypass");
	if (IS_ERR(imx8qxp_ldb->clk_bypass)) {
		ret = PTR_ERR(imx8qxp_ldb->clk_bypass);
		goto disable_pm_runtime;
	}

	if (ldb->dual) {
		imx8qxp_ldb->clk_aux_pixel = devm_clk_get(dev, "aux_pixel");
		if (IS_ERR(imx8qxp_ldb->clk_aux_pixel)) {
			ret = PTR_ERR(imx8qxp_ldb->clk_aux_pixel);
			goto disable_pm_runtime;
		}

		imx8qxp_ldb->clk_aux_bypass = devm_clk_get(dev, "aux_bypass");
		if (IS_ERR(imx8qxp_ldb->clk_aux_bypass)) {
			ret = PTR_ERR(imx8qxp_ldb->clk_aux_bypass);
			goto disable_pm_runtime;
		}

		auxldb_np = of_parse_phandle(np, "fsl,auxldb", 0);
		if (!auxldb_np) {
			dev_err(dev,
				"failed to find aux LDB node in device tree\n");
			ret = -ENODEV;
			goto disable_pm_runtime;
		}

		if (of_device_is_available(auxldb_np)) {
			dev_err(dev, "aux LDB node is already in use\n");
			of_node_put(auxldb_np);
			ret = -ENODEV;
			goto disable_pm_runtime;
		}

		imx8qxp_ldb->aux_regmap =
			syscon_regmap_lookup_by_phandle(auxldb_np, "gpr");
		if (IS_ERR(imx8qxp_ldb->aux_regmap)) {
			ret = PTR_ERR(imx8qxp_ldb->aux_regmap);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to get aux regmap\n");
			of_node_put(auxldb_np);
			goto disable_pm_runtime;
		}

		pm_runtime_get_sync(dev);
		regmap_write(imx8qxp_ldb->aux_regmap, ldb->ctrl_reg, 0);
		pm_runtime_put(dev);
	}

	for_each_child_of_node(np, child) {
		struct imx8qxp_ldb_channel *imx8qxp_ldb_ch;

		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0 || i > 1) {
			ret = -EINVAL;
			goto free_child;
		}

		if (!of_device_is_available(child))
			continue;

		imx8qxp_ldb_ch = &imx8qxp_ldb->channel[i];

		imx8qxp_ldb_ch->phy = devm_of_phy_get(dev, child, "ldb_phy");
		if (IS_ERR(imx8qxp_ldb_ch->phy)) {
			ret = PTR_ERR(imx8qxp_ldb_ch->phy);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "can't get channel%d phy: %d\n",
					i, ret);
			goto free_child;
		}

		ret = phy_init(imx8qxp_ldb_ch->phy);
		if (ret < 0) {
			dev_err(dev, "failed to initialize channel%d phy: %d\n",
				i, ret);
			goto free_child;
		}

		if (ldb->dual) {
			imx8qxp_ldb_ch->aux_phy =
					imx8qxp_ldb_get_aux_phy(auxldb_np);
			if (IS_ERR(imx8qxp_ldb_ch->aux_phy)) {
				ret = PTR_ERR(imx8qxp_ldb_ch->aux_phy);
				if (ret != -EPROBE_DEFER)
					dev_err(dev,
						"can't get channel0 aux phy: %d\n",
						ret);
				goto free_child;
			}

			ret = phy_init(imx8qxp_ldb_ch->aux_phy);
			if (ret < 0) {
				dev_err(dev,
					"failed to initialize channel0 aux phy: %d\n",
					ret);
				goto free_child;
			}
		}
	}

	if (ldb->dual)
		of_node_put(auxldb_np);

	for (i = 0; i < LDB_CH_NUM; i++) {
		ldb_ch = &imx8qxp_ldb->channel[i].base;

		if (!ldb_ch->is_valid)
			continue;

		ret = imx_drm_encoder_parse_of(drm, encoder[i], ldb_ch->child);
		if (ret)
			goto disable_pm_runtime;
	}

	return 0;

free_child:
	of_node_put(child);
	if (ldb->dual)
		of_node_put(auxldb_np);
disable_pm_runtime:
	pm_runtime_disable(dev);
	imx8qxp_ldb_detach_pm_domains(imx8qxp_ldb);

	return ret;
}

static void imx8qxp_ldb_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb = &imx8qxp_ldb->base;
	int i;

	for (i = 0; i < LDB_CH_NUM; i++) {
		struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						&imx8qxp_ldb->channel[i];

		if (imx8qxp_ldb_ch->phy_is_on) {
			phy_power_off(imx8qxp_ldb_ch->phy);
			if (ldb->dual)
				phy_power_off(imx8qxp_ldb_ch->aux_phy);
		}

		phy_exit(imx8qxp_ldb_ch->phy);
		if (ldb->dual && i == 0)
			phy_exit(imx8qxp_ldb_ch->aux_phy);
	}

	imx8qxp_ldb_detach_pm_domains(imx8qxp_ldb);
}

static const struct component_ops imx8qxp_ldb_ops = {
	.bind	= imx8qxp_ldb_bind,
	.unbind	= imx8qxp_ldb_unbind,
};

static int imx8qxp_ldb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx8qxp_ldb *imx8qxp_ldb;

	imx8qxp_ldb = devm_kzalloc(dev, sizeof(*imx8qxp_ldb), GFP_KERNEL);
	if (!imx8qxp_ldb)
		return -ENOMEM;

	dev_set_drvdata(dev, imx8qxp_ldb);

	return component_add(dev, &imx8qxp_ldb_ops);
}

static void imx8qxp_ldb_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx8qxp_ldb_ops);
}

#ifdef CONFIG_PM_SLEEP
static int imx8qxp_ldb_suspend(struct device *dev)
{
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb = &imx8qxp_ldb->base;
	int i;

	if (imx8qxp_ldb == NULL)
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++) {
		phy_exit(imx8qxp_ldb->channel[i].phy);

		if (ldb->dual && i == 0)
			phy_exit(imx8qxp_ldb->channel[i].aux_phy);
	}

	return 0;
}

static int imx8qxp_ldb_resume(struct device *dev)
{
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb = &imx8qxp_ldb->base;
	int i;

	if (imx8qxp_ldb == NULL)
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++) {
		phy_init(imx8qxp_ldb->channel[i].phy);

		if (ldb->dual && i == 0)
			phy_init(imx8qxp_ldb->channel[i].aux_phy);
	}

	imx8qxp_ldb_init_sc_misc(imx8qxp_ldb->id, ldb->dual);

	return 0;
}
#endif

static const struct dev_pm_ops imx8qxp_ldb_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(imx8qxp_ldb_suspend, imx8qxp_ldb_resume)
};

static struct platform_driver imx8qxp_ldb_driver = {
	.probe		= imx8qxp_ldb_probe,
	.remove		= imx8qxp_ldb_remove,
	.driver		= {
		.of_match_table = imx8qxp_ldb_dt_ids,
		.name	= DRIVER_NAME,
		.pm	= &imx8qxp_ldb_pm_ops,
	},
};

module_platform_driver(imx8qxp_ldb_driver);

MODULE_DESCRIPTION("i.MX8QXP LVDS driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
