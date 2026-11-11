// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36850 DriverIC panels driver
 *
 * Copyright (c) 2022, 2023 Jianhua Lu <lujianhua000@gmail.com>
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct panel_info {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	enum drm_panel_orientation orientation;

	struct gpio_desc *reset_gpio;
	struct regulator *vddio;
	struct regulator *avdd;
	struct regulator *avee;
};

struct panel_desc {
	unsigned int bpc;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *mode;
	int (*init_sequence)(struct panel_info *pinfo);

	bool is_dual_dsi;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int truly_wqhd_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi0 = pinfo->dsi[0];
	struct mipi_dsi_device *dsi1 = pinfo->dsi[1];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = NULL };

	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xff, 0x10);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xfb, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1,
					  MIPI_DCS_SET_ADDRESS_MODE, 0x00);
	mipi_dsi_dual(mipi_dsi_dcs_set_tear_on_multi, &dsi_ctx, dsi0, dsi1,
		      MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dual(mipi_dsi_dcs_set_tear_scanline_multi, &dsi_ctx, dsi0, dsi1,
		      1000);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1,
					  MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
					  0xff);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1,
					  MIPI_DCS_WRITE_CONTROL_DISPLAY,
					  0x2c);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1,
					  MIPI_DCS_WRITE_POWER_SAVE, 0x01);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1,
					  MIPI_DCS_EXIT_INVERT_MODE);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dual_dcs_write_seq_multi(&dsi_ctx, dsi0, dsi1, 0xbb, 0x10);
	mipi_dsi_dual(mipi_dsi_dcs_exit_sleep_mode_multi, &dsi_ctx, dsi0, dsi1);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dual(mipi_dsi_dcs_set_display_on_multi, &dsi_ctx, dsi0, dsi1);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode truly_wqhd_mode = {
	.clock = (720 + 120 + 20 + 140) * 2 * (2560 + 8 + 4 + 20) * 60 / 1000,
	.hdisplay = 720 * 2,
	.hsync_start = (720 + 120) * 2,
	.hsync_end = (720 + 120 + 20) * 2,
	.htotal = (720 + 120 + 20 + 140) * 2,
	.vdisplay = 2560,
	.vsync_start = 2560 + 8,
	.vsync_end = 2560 + 8 + 4,
	.vtotal = 2560 + 8 + 4 + 20,
};

static const struct panel_desc truly_wqhd_desc = {
	.mode = &truly_wqhd_mode,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = truly_wqhd_init_sequence,
	.is_dual_dsi = true,
};

static void nt36850_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	msleep(50);
}

static int nt36850_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	ret = regulator_enable(pinfo->vddio);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddio regulator: %d\n", ret);
		return ret;
	}

	msleep(20);

	ret = regulator_enable(pinfo->avdd);
	if (ret) {
		dev_err(panel->dev, "failed to enable avdd regulator: %d\n", ret);
		goto err_vddio;
	}

	ret = regulator_enable(pinfo->avee);
	if (ret) {
		dev_err(panel->dev, "failed to enable avee regulator: %d\n", ret);
		goto err_avdd;
	}

	msleep(20);

	nt36850_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		goto err_avee;
	}

	return 0;

err_avee:
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_disable(pinfo->avee);
err_avdd:
	regulator_disable(pinfo->avdd);
err_vddio:
	regulator_disable(pinfo->vddio);

	return ret;
}

static int nt36850_num_dsi_hosts(struct panel_info *pinfo)
{
	return pinfo->desc->is_dual_dsi ? 2 : 1;
}

static int nt36850_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct mipi_dsi_multi_context dsi_ctx = { };
	int i;

	for (i = 0; i < nt36850_num_dsi_hosts(pinfo); i++)
		pinfo->dsi[i]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	for (i = 0; i < nt36850_num_dsi_hosts(pinfo); i++) {
		dsi_ctx.dsi = pinfo->dsi[i];

		mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	}
	mipi_dsi_msleep(&dsi_ctx, 120);

	for (i = 0; i < nt36850_num_dsi_hosts(pinfo); i++) {
		dsi_ctx.dsi = pinfo->dsi[i];

		mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	}
	mipi_dsi_msleep(&dsi_ctx, 120);

	for (i = 0; i < nt36850_num_dsi_hosts(pinfo); i++)
		pinfo->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;

	return dsi_ctx.accum_err;
}

static int nt36850_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_disable(pinfo->avee);
	regulator_disable(pinfo->avdd);
	regulator_disable(pinfo->vddio);

	return 0;
}

static int nt36850_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);

	connector->display_info.bpc = pinfo->desc->bpc;

	return drm_connector_helper_get_modes_fixed(connector, pinfo->desc->mode);
}

static enum drm_panel_orientation nt36850_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

static const struct drm_panel_funcs nt36850_panel_funcs = {
	.disable = nt36850_disable,
	.prepare = nt36850_prepare,
	.unprepare = nt36850_unprepare,
	.get_modes = nt36850_get_modes,
	.get_orientation = nt36850_get_orientation,
};

static int nt36850_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct panel_info *pinfo;
	int i, ret;

	pinfo = devm_drm_panel_alloc(dev, struct panel_info, panel,
				     &nt36850_panel_funcs,
				     DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(pinfo))
		return PTR_ERR(pinfo);

	pinfo->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(pinfo->vddio))
		return dev_err_probe(dev, PTR_ERR(pinfo->vddio), "failed to get vddio regulator\n");

	pinfo->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(pinfo->avdd))
		return dev_err_probe(dev, PTR_ERR(pinfo->avdd), "failed to get avdd regulator\n");

	pinfo->avee = devm_regulator_get(dev, "avee");
	if (IS_ERR(pinfo->avee))
		return dev_err_probe(dev, PTR_ERR(pinfo->avee), "failed to get avee regulator\n");

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");

	pinfo->desc = of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	/* If the panel is dual dsi, register DSI1 */
	if (pinfo->desc->is_dual_dsi) {
		struct mipi_dsi_device_info dsi_info = {
			.type = "dsi-secondary",
		};

		dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi1) {
			dev_err(dev, "cannot get secondary DSI node.\n");
			return -ENODEV;
		}

		dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
		of_node_put(dsi1);
		if (!dsi1_host)
			return dev_err_probe(dev, -EPROBE_DEFER, "cannot get secondary DSI host\n");

		pinfo->dsi[1] = devm_mipi_dsi_device_register_full(dev, dsi1_host, &dsi_info);
		if (IS_ERR(pinfo->dsi[1])) {
			dev_err(dev, "cannot get secondary DSI device\n");
			return PTR_ERR(pinfo->dsi[1]);
		}
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	ret = drm_of_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	pinfo->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&pinfo->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	ret = devm_drm_panel_add(&dsi->dev, &pinfo->panel);
	if (ret)
		return dev_err_probe(dev, ret, "can't add the panel\n");

	for (i = 0; i < nt36850_num_dsi_hosts(pinfo); i++) {
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;

		ret = devm_mipi_dsi_attach(dev, pinfo->dsi[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot attach to DSI%d host.\n", i);
	}

	return 0;
}

static const struct of_device_id nt36850_of_match[] = {
	{
		.compatible = "truly,wqhd60-nt36850",
		.data = &truly_wqhd_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, nt36850_of_match);

static struct mipi_dsi_driver nt36850_driver = {
	.probe = nt36850_probe,
	.driver = {
		.name = "panel-novatek-nt36850",
		.of_match_table = nt36850_of_match,
	},
};
module_mipi_dsi_driver(nt36850_driver);

MODULE_AUTHOR("Dmitry Baryshkov <dmitry.baryshkov@oss.qualcomm.com>");
MODULE_DESCRIPTION("DRM driver for Novatek NT36850 based MIPI DSI panels");
MODULE_LICENSE("GPL");
