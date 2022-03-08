// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2021,2022 NXP
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/* User Define command set */
#define UD_SETADDRESSMODE	0x36 /* Set address mode */
#define UD_SETSEQUENCE		0xB0 /* Set sequence */
#define UD_SETPOWER		0xB1 /* Set power */
#define UD_SETDISP		0xB2 /* Set display related register */
#define UD_SETCYC		0xB4 /* Set display waveform cycles */
#define UD_SETVCOM		0xB6 /* Set VCOM voltage */
#define UD_SETTE		0xB7 /* Set internal TE function */
#define UD_SETSENSOR		0xB8 /* Set temperature sensor */
#define UD_SETEXTC		0xB9 /* Set extension command */
#define UD_SETMIPI		0xBA /* Set MIPI control */
#define UD_SETOTP		0xBB /* Set OTP */
#define UD_SETREGBANK		0xBD /* Set register bank */
#define UD_SETDGCLUT		0xC1 /* Set DGC LUT */
#define UD_SETID		0xC3 /* Set ID */
#define UD_SETDDB		0xC4 /* Set DDB */
#define UD_SETCABC		0xC9 /* Set CABC control */
#define UD_SETCABCGAIN		0xCA
#define UD_SETPANEL		0xCC
#define UD_SETOFFSET		0xD2
#define UD_SETGIP0		0xD3 /* Set GIP Option0 */
#define UD_SETGIP1		0xD5 /* Set GIP Option1 */
#define UD_SETGIP2		0xD6 /* Set GIP Option2 */
#define UD_SETGPO		0xD9
#define UD_SETSCALING		0xDD
#define UD_SETIDLE		0xDF
#define UD_SETGAMMA		0xE0 /* Set gamma curve related setting */
#define UD_SETCHEMODE_DYN	0xE4
#define UD_SETCHE		0xE5
#define UD_SETCESEL		0xE6 /* Enable color enhance */
#define UD_SET_SP_CMD		0xE9
#define UD_SETREADINDEX		0xFE /* Set SPI Read Index */
#define UD_GETSPIREAD		0xFF /* SPI Read Command Data */

struct hx8394f {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 66000,
	.hdisplay = 720,
	.hsync_start = 720 + 52,
	.hsync_end = 720 + 52 + 10,
	.htotal = 720 + 52 + 10 + 52,
	.vdisplay = 1280,
	.vsync_start = 1280 + 16,
	.vsync_end = 1280 + 16 + 7,
	.vtotal = 1280 + 16 + 7 + 16,
	.flags = 0,
	.width_mm = 68,
	.height_mm = 122,
};

static inline struct hx8394f *panel_to_hx8394f(struct drm_panel *panel)
{
	return container_of(panel, struct hx8394f, panel);
}

static void hx8394f_dcs_write_buf(struct hx8394f *ctx, const void *data,
				  size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int err;

	err = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (err < 0)
		dev_err_ratelimited(ctx->dev, "MIPI DSI DCS write buffer failed: %d\n", err);
}

#define dcs_write_seq(ctx, seq...)				\
({								\
	static const u8 d[] = { seq };				\
								\
	hx8394f_dcs_write_buf(ctx, d, ARRAY_SIZE(d));		\
})

static void hx8394f_init_sequence(struct hx8394f *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	u8 mipi_data[] = {UD_SETMIPI, 0x60, 0x03, 0x68, 0x6B, 0xB2, 0xC0};

	dcs_write_seq(ctx, UD_SETADDRESSMODE, 0x02);

	dcs_write_seq(ctx, UD_SETEXTC, 0xFF, 0x83, 0x94);

	/* SETMIPI */
	mipi_data[1] = 0x60 | (dsi->lanes - 1);
	hx8394f_dcs_write_buf(ctx, mipi_data, ARRAY_SIZE(mipi_data));

	dcs_write_seq(ctx, UD_SETPOWER, 0x48, 0x12, 0x72, 0x09, 0x32, 0x54,
		      0x71, 0x71, 0x57, 0x47);

	dcs_write_seq(ctx, UD_SETDISP, 0x00, 0x80, 0x64, 0x15, 0x0E, 0x11);

	dcs_write_seq(ctx, UD_SETCYC, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01,
		      0x0C, 0x86, 0x75, 0x00, 0x3F, 0x73, 0x74, 0x73, 0x74,
		      0x73, 0x74, 0x01, 0x0C, 0x86);

	dcs_write_seq(ctx, UD_SETGIP0, 0x00, 0x00, 0x07, 0x07, 0x40, 0x07, 0x0C,
		      0x00, 0x08, 0x10, 0x08, 0x00, 0x08, 0x54, 0x15, 0x0A,
		      0x05, 0x0A, 0x02, 0x15, 0x06, 0x05, 0x06, 0x47, 0x44,
		      0x0A, 0x0A, 0x4B, 0x10, 0x07, 0x07, 0x0C, 0x40);

	dcs_write_seq(ctx, UD_SETGIP1, 0x1C, 0x1C, 0x1D, 0x1D, 0x00, 0x01, 0x02,
		      0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
		      0x24, 0x25, 0x18, 0x18, 0x26, 0x27, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x20, 0x21, 0x18, 0x18, 0x18,
		      0x18);

	dcs_write_seq(ctx, UD_SETGIP2, 0x1C, 0x1C, 0x1D, 0x1D, 0x07, 0x06, 0x05,
		      0x04, 0x03, 0x02, 0x01, 0x00, 0x0B, 0x0A, 0x09, 0x08,
		      0x21, 0x20, 0x18, 0x18, 0x27, 0x26, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x25, 0x24, 0x18, 0x18, 0x18,
		      0x18);

	dcs_write_seq(ctx, UD_SETVCOM, 0x92, 0x92);

	dcs_write_seq(ctx, UD_SETGAMMA, 0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21,
		      0x24, 0x22, 0x47, 0x56, 0x65, 0x66, 0x6E, 0x82, 0x88,
		      0x8B, 0x9A, 0x9D, 0x98, 0xA8, 0xB9, 0x5D, 0x5C, 0x61,
		      0x66, 0x6A, 0x6F, 0x7F, 0x7F, 0x00, 0x0A, 0x15, 0x1B,
		      0x1E, 0x21, 0x24, 0x22, 0x47, 0x56, 0x65, 0x65, 0x6E,
		      0x81, 0x87, 0x8B, 0x98, 0x9D, 0x99, 0xA8, 0xBA, 0x5D,
		      0x5D, 0x62, 0x67, 0x6B, 0x72, 0x7F, 0x7F);
	dcs_write_seq(ctx, 0xC0, 0x1F, 0x31);
	dcs_write_seq(ctx, UD_SETPANEL, 0x03);
	dcs_write_seq(ctx, 0xD4, 0x02);
	dcs_write_seq(ctx, UD_SETREGBANK, 0x02);
	dcs_write_seq(ctx, 0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		      0xFF, 0xFF, 0xFF, 0xFF);
	dcs_write_seq(ctx, UD_SETREGBANK, 0x00);
	dcs_write_seq(ctx, UD_SETREGBANK, 0x01);
	dcs_write_seq(ctx, UD_SETPOWER, 0x00);
	dcs_write_seq(ctx, UD_SETREGBANK, 0x00);
	dcs_write_seq(ctx, 0xBF, 0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01);
	dcs_write_seq(ctx, 0xC6, 0xED);
}

static int hx8394f_disable(struct drm_panel *panel)
{
	struct hx8394f *ctx = panel_to_hx8394f(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->enabled)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		dev_warn(panel->dev, "failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		dev_warn(panel->dev, "failed to enter sleep mode: %d\n", ret);

	msleep(120);

	ctx->enabled = false;

	return 0;
}

static int hx8394f_unprepare(struct drm_panel *panel)
{
	struct hx8394f *ctx = panel_to_hx8394f(panel);
	int ret;

	if (!ctx->prepared)
		return 0;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
	}

	gpiod_set_value_cansleep(ctx->enable_gpio, 0);

	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to disable supplies: %d\n", ret);
		return ret;
	}

	ctx->prepared = false;

	return 0;
}

static int hx8394f_prepare(struct drm_panel *panel)
{
	struct hx8394f *ctx = panel_to_hx8394f(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret) {
		dev_err(ctx->dev, "failed to enable supplies: %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(ctx->enable_gpio, 1);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(55);
	}

	ctx->prepared = true;

	return 0;
}

static int hx8394f_enable(struct drm_panel *panel)
{
	struct hx8394f *ctx = panel_to_hx8394f(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->enabled)
		return 0;

	hx8394f_init_sequence(ctx);

	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to set tear ON (%d)\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	msleep(50);

	ctx->enabled = true;

	return 0;
}

static int hx8394f_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs hx8394f_drm_funcs = {
	.disable = hx8394f_disable,
	.unprepare = hx8394f_unprepare,
	.prepare = hx8394f_prepare,
	.enable = hx8394f_enable,
	.get_modes = hx8394f_get_modes,
};

static int hx8394f_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct hx8394f *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio)) {
		ret = PTR_ERR(ctx->enable_gpio);
		dev_err(dev, "failed to get enable GPIO: %d\n", ret);
		return ret;
	}

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		dev_err(dev, "failed to get reset GPIO: %d\n", ret);
		return ret;
	}

	ctx->supplies[0].supply = "vcc";
	ctx->supplies[1].supply = "iovcc";
	ret = devm_regulator_bulk_get(dev, 2, ctx->supplies);
	if (ret < 0)
		return ret;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	ret = of_property_read_u32(dev->of_node, "himax,dsi-lanes",
				   &dsi->lanes);
	if (ret) {
		dev_err(dev, "failed to get himax,dsi-lanes property: %d\n",
			ret);
		return ret;
	}

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &hx8394f_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach() failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void hx8394f_remove(struct mipi_dsi_device *dsi)
{
	struct hx8394f *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id hx8394f_of_match[] = {
	{ .compatible = "rocktech,hx8394f" },
	{ }
};
MODULE_DEVICE_TABLE(of, hx8394f_of_match);

static struct mipi_dsi_driver hx8394f_driver = {
	.probe = hx8394f_probe,
	.remove = hx8394f_remove,
	.driver = {
		.name = "panel-rocktech-hx8394f",
		.of_match_table = hx8394f_of_match,
	},
};
module_mipi_dsi_driver(hx8394f_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("DRM Driver for Rocktech Himax8394f MIPI DSI panel");
MODULE_LICENSE("GPL v2");
