// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Amarula Solutions
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

struct feiyang {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;

	struct regulator	*dvdd;
	struct regulator	*avdd;
	struct gpio_desc	*reset;
};

static inline struct feiyang *panel_to_feiyang(struct drm_panel *panel)
{
	return container_of(panel, struct feiyang, panel);
}

static void feiyang_init(struct mipi_dsi_multi_context *dsi_ctx)
{
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x80, 0x58);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x81, 0x47);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x82, 0xd4);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x83, 0x88);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x84, 0xa9);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x85, 0xc3);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x86, 0x82);
}

static int feiyang_prepare(struct drm_panel *panel)
{
	struct feiyang *ctx = panel_to_feiyang(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	ret = regulator_enable(ctx->dvdd);
	if (ret)
		return ret;

	/* T1 (dvdd start + dvdd rise) 0 < T1 <= 10ms */
	msleep(10);

	ret = regulator_enable(ctx->avdd);
	if (ret)
		goto disable_dvdd;

	/* T3 (dvdd rise + avdd start + avdd rise) T3 >= 20ms */
	msleep(20);

	gpiod_set_value(ctx->reset, 0);

	/*
	 * T5 + T6 (avdd rise + video & logic signal rise)
	 * T5 >= 10ms, 0 < T6 <= 10ms
	 */
	msleep(20);

	gpiod_set_value(ctx->reset, 1);

	/* T12 (video & logic signal rise + backlight rise) T12 >= 200ms */
	msleep(200);

	feiyang_init(&dsi_ctx);
	ret = dsi_ctx.accum_err;
	if (ret < 0)
		goto assert_reset;

	return 0;

assert_reset:
	gpiod_set_value(ctx->reset, 0);
	regulator_disable(ctx->avdd);

	/* Preserve the AVDD-to-DVDD shutdown delay from unprepare(). */
	msleep(10);

disable_dvdd:
	regulator_disable(ctx->dvdd);
	return ret;
}

static int feiyang_enable(struct drm_panel *panel)
{
	struct feiyang *ctx = panel_to_feiyang(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* T12 (video & logic signal rise + backlight rise) T12 >= 200ms */
	msleep(200);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int feiyang_disable(struct drm_panel *panel)
{
	struct feiyang *ctx = panel_to_feiyang(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);

	/*
	 * Returning an error would leave panel->enabled set, causing the
	 * next enable to be skipped even after unprepare powers off the panel.
	 * The helper above already logs any command failure.
	 */
	return 0;
}

static int feiyang_unprepare(struct drm_panel *panel)
{
	struct feiyang *ctx = panel_to_feiyang(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);

	/* Attempt to enter sleep mode even if display off failed. */
	dsi_ctx.accum_err = 0;
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);

	/* T13 (backlight fall + video & logic signal fall) T13 >= 200ms */
	msleep(200);

	gpiod_set_value(ctx->reset, 0);

	regulator_disable(ctx->avdd);

	/* T11 (dvdd rise to fall) 0 < T11 <= 10ms  */
	msleep(10);

	regulator_disable(ctx->dvdd);

	return 0;
}

static const struct drm_display_mode feiyang_default_mode = {
	.clock		= 55000,

	.hdisplay	= 1024,
	.hsync_start	= 1024 + 310,
	.hsync_end	= 1024 + 310 + 20,
	.htotal		= 1024 + 310 + 20 + 90,

	.vdisplay	= 600,
	.vsync_start	= 600 + 12,
	.vsync_end	= 600 + 12 + 2,
	.vtotal		= 600 + 12 + 2 + 21,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int feiyang_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct feiyang *ctx = panel_to_feiyang(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &feiyang_default_mode);
	if (!mode) {
		dev_err(&ctx->dsi->dev, "failed to add mode %ux%u@%u\n",
			feiyang_default_mode.hdisplay,
			feiyang_default_mode.vdisplay,
			drm_mode_vrefresh(&feiyang_default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs feiyang_funcs = {
	.disable = feiyang_disable,
	.unprepare = feiyang_unprepare,
	.prepare = feiyang_prepare,
	.enable = feiyang_enable,
	.get_modes = feiyang_get_modes,
};

static int feiyang_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct feiyang *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(&dsi->dev, struct feiyang, panel,
				   &feiyang_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;

	ctx->dvdd = devm_regulator_get(&dsi->dev, "dvdd");
	if (IS_ERR(ctx->dvdd))
		return dev_err_probe(&dsi->dev, PTR_ERR(ctx->dvdd),
				     "Couldn't get dvdd regulator\n");

	ctx->avdd = devm_regulator_get(&dsi->dev, "avdd");
	if (IS_ERR(ctx->avdd))
		return dev_err_probe(&dsi->dev, PTR_ERR(ctx->avdd),
				     "Couldn't get avdd regulator\n");

	ctx->reset = devm_gpiod_get_optional(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(&dsi->dev, PTR_ERR(ctx->reset),
				     "Couldn't get our reset GPIO\n");

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	ret = devm_drm_panel_add(&dsi->dev, &ctx->panel);
	if (ret)
		return ret;

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = 4;

	ret = devm_mipi_dsi_attach(&dsi->dev, dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct of_device_id feiyang_of_match[] = {
	{ .compatible = "feiyang,fy07024di26a30d", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, feiyang_of_match);

static struct mipi_dsi_driver feiyang_driver = {
	.probe = feiyang_dsi_probe,
	.driver = {
		.name = "feiyang-fy07024di26a30d",
		.of_match_table = feiyang_of_match,
	},
};
module_mipi_dsi_driver(feiyang_driver);

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Feiyang FY07024DI26A30-D MIPI-DSI LCD panel");
MODULE_LICENSE("GPL");
