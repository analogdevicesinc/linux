// SPDX-License-Identifier: GPL-2.0
/*
 * Raydium RM692C9 MIPI-DSI panel driver
 *
 * Copyright 2023,2024 NXP
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

static const u32 rad_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 rad_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE;

struct rad_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 148500,
	.hdisplay = 1080,
	.hsync_start = 1080 + 10,
	.hsync_end = 1080 + 10 + 4,
	.htotal = 1080 + 10 + 4 + 10,
	.vdisplay = 2340,
	.vsync_start = 2340 + 10,
	.vsync_end = 2340 + 10 + 4,
	.vtotal = 2340 + 10 + 4 + 10,
	.width_mm = 70,
	.height_mm = 150,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static inline struct rad_panel *to_rad_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rad_panel, panel);
}

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return COL_FMT_16BPP;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return COL_FMT_18BPP;
	case MIPI_DSI_FMT_RGB888:
		return COL_FMT_24BPP;
	default:
		return COL_FMT_24BPP; /* for backward compatibility */
	}
};

static int rad_panel_prepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	int ret;

	if (rad->prepared)
		return 0;

	ret = regulator_bulk_enable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	/* At lest 10ms needed between power-on and reset-out as RM specifies */
	usleep_range(10000, 12000);

	if (rad->reset) {
		gpiod_set_value_cansleep(rad->reset, 0);
		/*
		 * 50ms delay after reset-out, as per manufacturer initalization
		 * sequence.
		 */
		msleep(50);
	}

	rad->prepared = true;

	return 0;
}

static int rad_panel_unprepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	int ret;

	if (!rad->prepared)
		return 0;

	/*
	 * Right after asserting the reset, we need to release it, so that the
	 * touch driver can have an active connection with the touch controller
	 * even after the display is turned off.
	 */
	if (rad->reset) {
		gpiod_set_value_cansleep(rad->reset, 1);
		usleep_range(15000, 17000);
		gpiod_set_value_cansleep(rad->reset, 0);
	}

	ret = regulator_bulk_disable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	rad->prepared = false;

	return 0;
}

static int rad_panel_enable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;

	if (rad->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}

	usleep_range(120000, 125000);

	ret = mipi_dsi_generic_write(dsi, (u8[]){ WRMAUCCTR, 0x72 }, 2);
	if (ret < 0)
		goto fail;

	/* Use 8-bit brightness mode */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0x4D, 0x65 }, 2);
	if (ret < 0)
		goto fail;

	/* Select User Command Set table (CMD1) */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ WRMAUCCTR, 0x00 }, 2);
	if (ret < 0)
		goto fail;

	/* Set DSI mode */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xC2, 0x0B }, 2);
	if (ret < 0) {
		dev_err(dev, "Failed to set DSI mode (%d)\n", ret);
		goto fail;
	}
	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear ON (%d)\n", ret);
		goto fail;
	}
	/* Set tear scanline */
	ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0x380);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear scanline (%d)\n", ret);
		goto fail;
	}
	/* Set pixel format */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
	dev_dbg(dev, "Interface color format set to 0x%x\n", color_format);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}
	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	usleep_range(5000, 7000);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	backlight_enable(rad->backlight);

	rad->enabled = true;

	return 0;
fail:
	gpiod_set_value_cansleep(rad->reset, 1);

	return ret;
}

static int rad_panel_disable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!rad->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	backlight_disable(rad->backlight);

	usleep_range(10000, 12000);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	rad->enabled = false;

	return 0;
}

static int rad_panel_get_modes(struct drm_panel *panel,
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
	connector->display_info.bus_flags = rad_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 rad_bus_formats,
					 ARRAY_SIZE(rad_bus_formats));
	return 1;
}

static int rad_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	u8 payload[2] = { 0x00, bl->props.brightness & 0xff };
	int ret;

	if (!rad->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				 payload, sizeof(payload));
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops rad_bl_ops = {
	.update_status = rad_bl_update_status,
};

static const struct drm_panel_funcs rad_panel_funcs = {
	.prepare = rad_panel_prepare,
	.unprepare = rad_panel_unprepare,
	.enable = rad_panel_enable,
	.disable = rad_panel_disable,
	.get_modes = rad_panel_get_modes,
};

static const char * const rad_supply_names[] = {
	"v3p3",
	"v1p8",
};

static int rad_init_regulators(struct rad_panel *rad)
{
	struct device *dev = &rad->dsi->dev;
	int i;

	rad->num_supplies = ARRAY_SIZE(rad_supply_names);
	rad->supplies = devm_kcalloc(dev, rad->num_supplies,
				     sizeof(*rad->supplies), GFP_KERNEL);
	if (!rad->supplies)
		return -ENOMEM;

	for (i = 0; i < rad->num_supplies; i++)
		rad->supplies[i].supply = rad_supply_names[i];

	return devm_regulator_bulk_get(dev, rad->num_supplies, rad->supplies);
};

static const struct of_device_id rad_of_match[] = {
	{ .compatible = "raydium,rm692c9", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rad_of_match);

static int rad_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rad_panel *panel;
	struct backlight_properties bl_props;
	int ret;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE |
			  MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	panel->reset = devm_gpiod_get_optional(dev, "reset",
					       GPIOD_OUT_LOW |
					       GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(panel->reset)) {
		ret = PTR_ERR(panel->reset);
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}
	gpiod_set_value_cansleep(panel->reset, 1);

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(dev, dev_name(dev),
							  dev, dsi, &rad_bl_ops,
							  &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	ret = rad_init_regulators(panel);
	if (ret)
		return ret;

	drm_panel_init(&panel->panel, dev, &rad_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);

	return ret;
}

static void rad_panel_remove(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&rad->panel);
}

static void rad_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);

	rad_panel_disable(&rad->panel);
	rad_panel_unprepare(&rad->panel);
}

static struct mipi_dsi_driver rad_panel_driver = {
	.driver = {
		.name = "panel-raydium-rm692c9",
		.of_match_table = rad_of_match,
	},
	.probe = rad_panel_probe,
	.remove = rad_panel_remove,
	.shutdown = rad_panel_shutdown,
};
module_mipi_dsi_driver(rad_panel_driver);

MODULE_AUTHOR("Praveen Adi <praveen.adi@nxp.com>");
MODULE_DESCRIPTION("DRM Driver for Raydium RM692C9 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
