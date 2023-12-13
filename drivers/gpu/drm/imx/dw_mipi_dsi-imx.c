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
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/pm_runtime.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>

#include "imx-drm.h"

#define enc_to_dsi(enc)	container_of(enc, struct dw_mipi_dsi_imx, encoder)

struct dw_mipi_dsi_imx {
	struct device *dev;
	struct drm_encoder encoder;
	void __iomem *base;

	struct clk *byte_clk;
	struct clk *pixel_clk;

	struct phy *phy;
	union phy_configure_opts phy_cfg;

	u32 lanes;
	u32 format;

	struct dw_mipi_dsi *dmd;
	struct dw_mipi_dsi_plat_data pdata;
};

static enum drm_mode_status
__dw_mipi_dsi_imx_mode_valid(void *priv_data,
			     const struct drm_display_mode *mode,
			     union phy_configure_opts *phy_cfg)
{
	struct dw_mipi_dsi_imx *dsi = priv_data;
	struct device *dev = dsi->dev;
	struct drm_bridge *bridge;
	int bpp;
	int ret;

	bridge = dw_mipi_dsi_get_bridge(dsi->dmd);

	/* Get the last bridge */
	while (drm_bridge_get_next_bridge(bridge))
		bridge = drm_bridge_get_next_bridge(bridge);

	if ((bridge->ops & DRM_BRIDGE_OP_DETECT) &&
	    (bridge->ops & DRM_BRIDGE_OP_EDID)) {
		unsigned long pixel_clock_rate = mode->clock * 1000;
		unsigned long rounded_rate;

		/* Allow +/-0.5% pixel clock rate deviation */
		rounded_rate = clk_round_rate(dsi->pixel_clk, pixel_clock_rate);
		if (rounded_rate < pixel_clock_rate * 995 / 1000 ||
		    rounded_rate > pixel_clock_rate * 1005 / 1000) {
			DRM_DEV_DEBUG(dev, "failed to round clock for mode " DRM_MODE_FMT "\n",
				      DRM_MODE_ARG(mode));
			return MODE_NOCLOCK;
		}
	}

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	if (bpp < 0) {
		DRM_DEV_DEBUG(dev, "failed to get bpp for pixel format %d\n",
			      dsi->format);
		return MODE_ERROR;
	}

	ret = phy_mipi_dphy_get_default_config(mode->clock * MSEC_PER_SEC,
					       bpp, dsi->lanes,
					       &phy_cfg->mipi_dphy);
	if (ret < 0) {
		DRM_DEV_DEBUG(dev, "failed to get default phy cfg %d\n", ret);
		return MODE_ERROR;
	}

	ret = phy_validate(dsi->phy, PHY_MODE_MIPI_DPHY, 0, phy_cfg);
	if (ret < 0) {
		DRM_DEV_DEBUG(dev, "failed to validate phy cfg %d\n", ret);
		return MODE_ERROR;
	}

	return MODE_OK;
}

static enum drm_mode_status
dw_mipi_dsi_imx_mode_valid(void *priv_data,
			   const struct drm_display_mode *mode,
			   unsigned long mode_flags,
			   u32 lanes, u32 format)
{
	union phy_configure_opts phy_cfg;

	return __dw_mipi_dsi_imx_mode_valid(priv_data, mode, &phy_cfg);
}

static bool dw_mipi_dsi_mode_fixup(void *priv_data,
				   const struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct dw_mipi_dsi_imx *dsi = priv_data;
	unsigned long pixel_clock_rate;
	unsigned long rounded_rate;

	pixel_clock_rate = mode->clock * 1000;
	rounded_rate = clk_round_rate(dsi->pixel_clk, pixel_clock_rate);

	memcpy(adjusted_mode, mode, sizeof(*mode));
	adjusted_mode->clock = rounded_rate / 1000;

	DRM_DEV_DEBUG(dsi->dev, "adj clock %d for mode " DRM_MODE_FMT "\n",
		      adjusted_mode->clock, DRM_MODE_ARG(mode));

	return true;
}

static int dw_mipi_dsi_imx_phy_init(void *priv_data)
{
	struct dw_mipi_dsi_imx *dsi = priv_data;
	int ret;

	ret = phy_set_mode(dsi->phy, PHY_MODE_MIPI_DPHY);
	if (ret) {
		DRM_DEV_ERROR(dsi->dev, "failed to set phy mode: %d\n", ret);
		return ret;
	}

	ret = phy_init(dsi->phy);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "failed to init phy: %d\n", ret);
		return ret;
	}

	ret = phy_configure(dsi->phy, &dsi->phy_cfg);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "failed to configure phy: %d\n", ret);
		goto uninit_phy;
	}

	ret = phy_power_on(dsi->phy);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "failed to power on phy: %d\n", ret);
		goto uninit_phy;
	}

	return ret;

uninit_phy:
	phy_exit(dsi->phy);
	return ret;
}

static void dw_mipi_dsi_imx_phy_power_off(void *priv_data)
{
	struct dw_mipi_dsi_imx *dsi = priv_data;
	int ret;

	ret = phy_power_off(dsi->phy);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "failed to power off phy: %d\n", ret);

	ret = phy_exit(dsi->phy);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "failed to exit phy: %d\n", ret);
}

static int
dw_mipi_dsi_get_lane_mbps(void *priv_data, const struct drm_display_mode *mode,
			  unsigned long mode_flags, u32 lanes, u32 format,
			  unsigned int *lane_mbps)
{
	struct dw_mipi_dsi_imx *dsi = priv_data;
	struct device *dev = dsi->dev;
	unsigned long mpclk = DIV_ROUND_UP(mode->clock, MSEC_PER_SEC);
	int bpp;
	int ret;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0) {
		DRM_DEV_ERROR(dev, "failed to get bpp for pixel format %d\n",
			      format);
		return bpp;
	}

	*lane_mbps = mpclk * (bpp / lanes);

	ret = phy_mipi_dphy_get_default_config(mode->clock * MSEC_PER_SEC,
					       bpp, lanes,
					       &dsi->phy_cfg.mipi_dphy);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to get default phy cfg %d\n", ret);
		return ret;
	}

	return 0;
}

struct hstt {
	unsigned int maxfreq;
	struct dw_mipi_dsi_dphy_timing timing;
};

#define HSTT(_maxfreq, _c_lp2hs, _c_hs2lp, _d_lp2hs, _d_hs2lp)	\
{								\
	.maxfreq = (_maxfreq),					\
	.timing = {						\
		.clk_lp2hs = (_c_lp2hs),			\
		.clk_hs2lp = (_c_hs2lp),			\
		.data_lp2hs = (_d_lp2hs),			\
		.data_hs2lp = (_d_hs2lp),			\
	}							\
}

/* Table A-4 High-Speed Transition Times */
struct hstt hstt_table[] = {
	HSTT(80,    21,  17,  15, 10),
	HSTT(90,    23,  17,  16, 10),
	HSTT(100,   22,  17,  16, 10),
	HSTT(110,   25,  18,  17, 11),
	HSTT(120,   26,  20,  18, 11),
	HSTT(130,   27,  19,  19, 11),
	HSTT(140,   27,  19,  19, 11),
	HSTT(150,   28,  20,  20, 12),
	HSTT(160,   30,  21,  22, 13),
	HSTT(170,   30,  21,  23, 13),
	HSTT(180,   31,  21,  23, 13),
	HSTT(190,   32,  22,  24, 13),
	HSTT(205,   35,  22,  25, 13),
	HSTT(220,   37,  26,  27, 15),
	HSTT(235,   38,  28,  27, 16),
	HSTT(250,   41,  29,  30, 17),
	HSTT(275,   43,  29,  32, 18),
	HSTT(300,   45,  32,  35, 19),
	HSTT(325,   48,  33,  36, 18),
	HSTT(350,   51,  35,  40, 20),
	HSTT(400,   59,  37,  44, 21),
	HSTT(450,   65,  40,  49, 23),
	HSTT(500,   71,  41,  54, 24),
	HSTT(550,   77,  44,  57, 26),
	HSTT(600,   82,  46,  64, 27),
	HSTT(650,   87,  48,  67, 28),
	HSTT(700,   94,  52,  71, 29),
	HSTT(750,   99,  52,  75, 31),
	HSTT(800,  105,  55,  82, 32),
	HSTT(850,  110,  58,  85, 32),
	HSTT(900,  115,  58,  88, 35),
	HSTT(950,  120,  62,  93, 36),
	HSTT(1000, 128,  63,  99, 38),
	HSTT(1050, 132,  65, 102, 38),
	HSTT(1100, 138,  67, 106, 39),
	HSTT(1150, 146,  69, 112, 42),
	HSTT(1200, 151,  71, 117, 43),
	HSTT(1250, 153,  74, 120, 45),
	HSTT(1300, 160,  73, 124, 46),
	HSTT(1350, 165,  76, 130, 47),
	HSTT(1400, 172,  78, 134, 49),
	HSTT(1450, 177,  80, 138, 49),
	HSTT(1500, 183,  81, 143, 52),
	HSTT(1550, 191,  84, 147, 52),
	HSTT(1600, 194,  85, 152, 52),
	HSTT(1650, 201,  86, 155, 53),
	HSTT(1700, 208,  88, 161, 53),
	HSTT(1750, 212,  89, 165, 53),
	HSTT(1800, 220,  90, 171, 54),
	HSTT(1850, 223,  92, 175, 54),
	HSTT(1900, 231,  91, 180, 55),
	HSTT(1950, 236,  95, 185, 56),
	HSTT(2000, 243,  97, 190, 56),
	HSTT(2050, 248,  99, 194, 58),
	HSTT(2100, 252, 100, 199, 59),
	HSTT(2150, 259, 102, 204, 61),
	HSTT(2200, 266, 105, 210, 62),
	HSTT(2250, 269, 109, 213, 63),
	HSTT(2300, 272, 109, 217, 65),
	HSTT(2350, 281, 112, 225, 66),
	HSTT(2400, 283, 115, 226, 66),
	HSTT(2450, 282, 115, 226, 67),
	HSTT(2500, 281, 118, 227, 67),
};

static int
dw_mipi_dsi_phy_get_timing(void *priv_data, unsigned int lane_mbps,
			   struct dw_mipi_dsi_dphy_timing *timing)
{
	struct dw_mipi_dsi_imx *dsi = priv_data;
	int i;

	for (i = 0; i < ARRAY_SIZE(hstt_table); i++)
		if (lane_mbps <= hstt_table[i].maxfreq)
			break;

	if (i == ARRAY_SIZE(hstt_table))
		i--;

	*timing = hstt_table[i].timing;

	DRM_DEV_DEBUG(dsi->dev, "get phy timing for %u <= %u (lane_mbps)\n",
		      lane_mbps, hstt_table[i].maxfreq);

	return 0;
}

static const struct dw_mipi_dsi_phy_ops dw_mipi_dsi_imx_phy_ops = {
	.init = dw_mipi_dsi_imx_phy_init,
	.power_off = dw_mipi_dsi_imx_phy_power_off,
	.get_lane_mbps = dw_mipi_dsi_get_lane_mbps,
	.get_timing = dw_mipi_dsi_phy_get_timing,
};

static int dw_mipi_dsi_imx_host_attach(void *priv_data,
				       struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi_imx *dsi = priv_data;

	dsi->lanes = device->lanes;
	dsi->format = device->format;

	return 0;
}

static const struct dw_mipi_dsi_host_ops dw_mipi_dsi_imx_host_ops = {
	.attach = dw_mipi_dsi_imx_host_attach,
};

static int
dw_mipi_dsi_encoder_atomic_check(struct drm_encoder *encoder,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *s = to_imx_crtc_state(crtc_state);
	struct dw_mipi_dsi_imx *dsi = enc_to_dsi(encoder);
	struct device *dev = dsi->dev;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		s->bus_format = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	case MIPI_DSI_FMT_RGB666:
		s->bus_format = MEDIA_BUS_FMT_RGB666_1X18;
		break;
	case MIPI_DSI_FMT_RGB565:
		s->bus_format = MEDIA_BUS_FMT_RGB565_1X16;
		break;
	default:
		DRM_DEV_ERROR(dev, "unsupported DSI format: 0x%x\n",
			      dsi->format);
		return -EINVAL;
	}

	s->bus_flags = DRM_BUS_FLAG_DE_HIGH;

	/* Force mode_changed to true, when CRTC state is changed to active. */
	if (crtc_state->active_changed && crtc_state->active)
		crtc_state->mode_changed = true;

	return 0;
}

static void
dw_mipi_dsi_encoder_mode_set(struct drm_encoder *encoder,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adjusted_mode)
{
	struct dw_mipi_dsi_imx *dsi = enc_to_dsi(encoder);

	pm_runtime_get_sync(dsi->dev);
}

static void dw_mipi_dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct dw_mipi_dsi_imx *dsi = enc_to_dsi(encoder);

	pm_runtime_put(dsi->dev);
}

static const struct drm_encoder_helper_funcs
dw_mipi_dsi_encoder_helper_funcs = {
	.atomic_check = dw_mipi_dsi_encoder_atomic_check,
	.mode_set = dw_mipi_dsi_encoder_mode_set,
	.disable = dw_mipi_dsi_encoder_disable,
};

static int imx_dsi_drm_create_encoder(struct dw_mipi_dsi_imx *dsi,
				      struct drm_device *drm_dev)
{
	struct device *dev = dsi->dev;
	struct drm_encoder *encoder = &dsi->encoder;
	int ret;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
							     dev->of_node);

	ret = drm_simple_encoder_init(drm_dev, encoder, DRM_MODE_ENCODER_DSI);
	if (ret) {
		DRM_DEV_ERROR(dev,
			      "failed to initialize encoder with drm: %d\n",
			      ret);
		return ret;
	}

	drm_encoder_helper_add(encoder, &dw_mipi_dsi_encoder_helper_funcs);

	return 0;
}

static int
dw_mipi_dsi_imx_bind(struct device *dev, struct device *master, void *data)
{
	struct dw_mipi_dsi_imx *dsi = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_bridge *bridge;
	int ret;

	ret = imx_dsi_drm_create_encoder(dsi, drm_dev);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to create drm encoder\n");
		return ret;
	}

	bridge = of_drm_find_bridge(dev->of_node);
	if (!bridge) {
		DRM_DEV_ERROR(dev, "failed to find drm bridge\n");
		return -ENODEV;
	}

	ret = drm_bridge_attach(&dsi->encoder, bridge, NULL, 0);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to attach bridge: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct component_ops dw_mipi_dsi_imx_ops = {
	.bind = dw_mipi_dsi_imx_bind,
};

static int dw_mipi_dsi_imx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_mipi_dsi_imx *dsi;
	struct resource *res;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->base)) {
		DRM_DEV_ERROR(dev, "failed to get dsi registers\n");
		return PTR_ERR(dsi->base);
	}

	dsi->byte_clk = devm_clk_get(dev, "byte");
	if (IS_ERR(dsi->byte_clk)) {
		ret = PTR_ERR(dsi->byte_clk);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "failed to get byte clk: %d\n", ret);
		return ret;
	}

	dsi->pixel_clk = devm_clk_get(dev, "pixel");
	if (IS_ERR(dsi->pixel_clk)) {
		ret = PTR_ERR(dsi->pixel_clk);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "failed to get pixel clk: %d\n", ret);
		return ret;
	}

	dsi->phy = devm_phy_get(dev, "dphy");
	if (IS_ERR(dsi->phy)) {
		ret = PTR_ERR(dsi->phy);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "failed to get phy: %d\n", ret);
		return ret;
	}

	dsi->dev = dev;
	dsi->pdata.base = dsi->base;
	dsi->pdata.max_data_lanes = 4;
	dsi->pdata.mode_valid = dw_mipi_dsi_imx_mode_valid;
	dsi->pdata.mode_fixup = dw_mipi_dsi_mode_fixup;
	dsi->pdata.phy_ops = &dw_mipi_dsi_imx_phy_ops;
	dsi->pdata.host_ops = &dw_mipi_dsi_imx_host_ops;
	dsi->pdata.priv_data = dsi;
	platform_set_drvdata(pdev, dsi);

	dsi->dmd = dw_mipi_dsi_probe(pdev, &dsi->pdata);
	if (IS_ERR(dsi->dmd)) {
		ret = PTR_ERR(dsi->dmd);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev,
				      "failed to probe dw_mipi_dsi: %d\n", ret);
		return ret;
	}

	ret = component_add(dev, &dw_mipi_dsi_imx_ops);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to register component: %d\n", ret);
		dw_mipi_dsi_remove(dsi->dmd);
	}

	return ret;
}

static int dw_mipi_dsi_imx_remove(struct platform_device *pdev)
{
	struct dw_mipi_dsi_imx *dsi = platform_get_drvdata(pdev);

	component_del(dsi->dev, &dw_mipi_dsi_imx_ops);

	dw_mipi_dsi_remove(dsi->dmd);

	clk_disable_unprepare(dsi->byte_clk);

	return 0;
}

#ifdef CONFIG_PM
static int dw_mipi_dsi_imx_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_mipi_dsi_imx *dsi = platform_get_drvdata(pdev);

	clk_disable_unprepare(dsi->byte_clk);

	return 0;
}

static int dw_mipi_dsi_imx_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_mipi_dsi_imx *dsi = platform_get_drvdata(pdev);
	int ret;

	ret = clk_prepare_enable(dsi->byte_clk);
	if (ret)
		DRM_DEV_ERROR(dev, "failed to enable byte_clk: %d\n", ret);

	return ret;
}

static const struct dev_pm_ops dw_mipi_dsi_imx_pm_ops = {
	SET_RUNTIME_PM_OPS(dw_mipi_dsi_imx_runtime_suspend,
			   dw_mipi_dsi_imx_runtime_resume, NULL)
};
#endif

static const struct of_device_id dw_mipi_dsi_imx_dt_ids[] = {
	{ .compatible = "fsl,imx93-mipi-dsi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw_mipi_dsi_imx_dt_ids);

struct platform_driver dw_mipi_dsi_imx_driver = {
	.probe	= dw_mipi_dsi_imx_probe,
	.remove	= dw_mipi_dsi_imx_remove,
	.driver	= {
		.of_match_table = dw_mipi_dsi_imx_dt_ids,
		.name = "dw-mipi-dsi-imx",
		.pm = &dw_mipi_dsi_imx_pm_ops,
	},
};

module_platform_driver(dw_mipi_dsi_imx_driver);

MODULE_DESCRIPTION("Freescale i.MX93 Synopsys DesignWare MIPI DSI Host Controller driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dw-mipi-dsi-imx");
