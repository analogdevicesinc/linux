/*
 * i.MX drm driver - Raydium MIPI-DSI panel driver
 *
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#define CMD_TABLE_LEN 2
typedef u8 cmd_set_table[CMD_TABLE_LEN];

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

/* Manufacturer Command Set pages (CMD2) */
static const cmd_set_table manufacturer_cmd_set[] = {
	{0xFE, 0x0B},
	{0x28, 0x40},
	{0x29, 0x4F},
	{0xFE, 0x0E},
	{0x4B, 0x00},
	{0x4C, 0x0F},
	{0x4D, 0x20},
	{0x4E, 0x40},
	{0x4F, 0x60},
	{0x50, 0xA0},
	{0x51, 0xC0},
	{0x52, 0xE0},
	{0x53, 0xFF},
	{0xFE, 0x0D},
	{0x18, 0x08},
	{0x42, 0x00},
	{0x08, 0x41},
	{0x46, 0x02},
	{0x72, 0x09},
	{0xFE, 0x0A},
	{0x24, 0x17},
	{0x04, 0x07},
	{0x1A, 0x0C},
	{0x0F, 0x44},
	{0xFE, 0x04},
	{0x00, 0x0C},
	{0x05, 0x08},
	{0x06, 0x08},
	{0x08, 0x08},
	{0x09, 0x08},
	{0x0A, 0xE6},
	{0x0B, 0x8C},
	{0x1A, 0x12},
	{0x1E, 0xE0},
	{0x29, 0x93},
	{0x2A, 0x93},
	{0x2F, 0x02},
	{0x31, 0x02},
	{0x33, 0x05},
	{0x37, 0x2D},
	{0x38, 0x2D},
	{0x3A, 0x1E},
	{0x3B, 0x1E},
	{0x3D, 0x27},
	{0x3F, 0x80},
	{0x40, 0x40},
	{0x41, 0xE0},
	{0x4F, 0x2F},
	{0x50, 0x1E},
	{0xFE, 0x06},
	{0x00, 0xCC},
	{0x05, 0x05},
	{0x07, 0xA2},
	{0x08, 0xCC},
	{0x0D, 0x03},
	{0x0F, 0xA2},
	{0x32, 0xCC},
	{0x37, 0x05},
	{0x39, 0x83},
	{0x3A, 0xCC},
	{0x41, 0x04},
	{0x43, 0x83},
	{0x44, 0xCC},
	{0x49, 0x05},
	{0x4B, 0xA2},
	{0x4C, 0xCC},
	{0x51, 0x03},
	{0x53, 0xA2},
	{0x75, 0xCC},
	{0x7A, 0x03},
	{0x7C, 0x83},
	{0x7D, 0xCC},
	{0x82, 0x02},
	{0x84, 0x83},
	{0x85, 0xEC},
	{0x86, 0x0F},
	{0x87, 0xFF},
	{0x88, 0x00},
	{0x8A, 0x02},
	{0x8C, 0xA2},
	{0x8D, 0xEA},
	{0x8E, 0x01},
	{0x8F, 0xE8},
	{0xFE, 0x06},
	{0x90, 0x0A},
	{0x92, 0x06},
	{0x93, 0xA0},
	{0x94, 0xA8},
	{0x95, 0xEC},
	{0x96, 0x0F},
	{0x97, 0xFF},
	{0x98, 0x00},
	{0x9A, 0x02},
	{0x9C, 0xA2},
	{0xAC, 0x04},
	{0xFE, 0x06},
	{0xB1, 0x12},
	{0xB2, 0x17},
	{0xB3, 0x17},
	{0xB4, 0x17},
	{0xB5, 0x17},
	{0xB6, 0x11},
	{0xB7, 0x08},
	{0xB8, 0x09},
	{0xB9, 0x06},
	{0xBA, 0x07},
	{0xBB, 0x17},
	{0xBC, 0x17},
	{0xBD, 0x17},
	{0xBE, 0x17},
	{0xBF, 0x17},
	{0xC0, 0x17},
	{0xC1, 0x17},
	{0xC2, 0x17},
	{0xC3, 0x17},
	{0xC4, 0x0F},
	{0xC5, 0x0E},
	{0xC6, 0x00},
	{0xC7, 0x01},
	{0xC8, 0x10},
	{0xFE, 0x06},
	{0x95, 0xEC},
	{0x8D, 0xEE},
	{0x44, 0xEC},
	{0x4C, 0xEC},
	{0x32, 0xEC},
	{0x3A, 0xEC},
	{0x7D, 0xEC},
	{0x75, 0xEC},
	{0x00, 0xEC},
	{0x08, 0xEC},
	{0x85, 0xEC},
	{0xA6, 0x21},
	{0xA7, 0x05},
	{0xA9, 0x06},
	{0x82, 0x06},
	{0x41, 0x06},
	{0x7A, 0x07},
	{0x37, 0x07},
	{0x05, 0x06},
	{0x49, 0x06},
	{0x0D, 0x04},
	{0x51, 0x04},
};

struct rad_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct backlight_device *backlight;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

static inline struct rad_panel *to_rad_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rad_panel, base);
}

static int rad_panel_push_cmd_list(struct mipi_dsi_device *dsi)
{
	size_t i;
	const u8 *cmd;
	size_t count = sizeof(manufacturer_cmd_set) / CMD_TABLE_LEN;
	int ret = 0;

	for (i = 0; i < count ; i++) {
		cmd = manufacturer_cmd_set[i];
		ret = mipi_dsi_generic_write(dsi, cmd, CMD_TABLE_LEN);
		if (ret < 0)
			return ret;
	}

	return ret;
};

static int rad_panel_prepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (rad->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	if (rad->reset != NULL) {
		gpiod_set_value(rad->reset, 1);
		msleep(100);
		gpiod_set_value(rad->reset, 0);
		msleep(100);
		gpiod_set_value(rad->reset, 1);
		msleep(100);
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = rad_panel_push_cmd_list(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	}

	/* Select User Command Set table (CMD1) */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ WRMAUCCTR, 0x00 }, 2);
	if (ret < 0)
		goto fail;

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}

	msleep(100);

	/* Set DSI mode */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xC2, 0x0B }, 2);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set DSI mode (%d)\n", ret);
		goto fail;
	}
	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set tear ON (%d)\n", ret);
		goto fail;
	}
	/* Set tear scanline */
	ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0x380);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set tear scanline (%d)\n", ret);
		goto fail;
	}
	/* Set pixel format to RGB888 */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, 0x77);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}
	/* Set display brightness */
	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x20);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display brightness (%d)\n",
			      ret);
		goto fail;
	}
	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	rad->prepared = true;

	return 0;

fail:
	if (rad->reset != NULL)
		gpiod_set_value(rad->reset, 0);

	return ret;
}

static int rad_panel_unprepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!rad->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to set display OFF (%d)\n", ret);

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);

	usleep_range(10000, 15000);

	if (rad->reset != NULL) {
		gpiod_set_value(rad->reset, 0);
		usleep_range(10000, 15000);
	}

	rad->prepared = false;

	return 0;
}

static int rad_panel_enable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct device *dev = &rad->dsi->dev;

	if (rad->enabled)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	rad->backlight->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(rad->backlight);

	rad->enabled = true;

	return 0;
}

static int rad_panel_disable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct device *dev = &rad->dsi->dev;

	if (!rad->enabled)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	rad->backlight->props.power = FB_BLANK_POWERDOWN;
	backlight_update_status(rad->backlight);

	rad->enabled = false;

	return 0;
}

static int rad_panel_get_modes(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct device *dev = &rad->dsi->dev;
	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&rad->vm, mode);
	mode->width_mm = rad->width_mm;
	mode->height_mm = rad->height_mm;
	connector->display_info.width_mm = rad->width_mm;
	connector->display_info.height_mm = rad->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	if (rad->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (rad->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (rad->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (rad->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	drm_mode_probed_add(panel->connector, mode);

	return 1;
}

static int rad_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (!rad->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int rad_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret = 0;

	if (!rad->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "New brightness: %d\n", bl->props.brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops rad_bl_ops = {
	.update_status = rad_bl_update_status,
	.get_brightness = rad_bl_get_brightness,
};

static const struct drm_panel_funcs rad_panel_funcs = {
	.prepare = rad_panel_prepare,
	.unprepare = rad_panel_unprepare,
	.enable = rad_panel_enable,
	.disable = rad_panel_disable,
	.get_modes = rad_panel_get_modes,
};

/*
 * The clock might range from 66MHz (30Hz refresh rate)
 * to 132MHz (60Hz refresh rate)
 */
static const struct display_timing rad_default_timing = {
	.pixelclock = { 66000000, 120000000, 132000000 },
	.hactive = { 1080, 1080, 1080 },
	.hfront_porch = { 20, 20, 20 },
	.hsync_len = { 2, 2, 2 },
	.hback_porch = { 34, 34, 34 },
	.vactive = { 1920, 1920, 1920 },
	.vfront_porch = { 10, 10, 10 },
	.vsync_len = { 2, 2, 2 },
	.vback_porch = { 4, 4, 4 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
		 DISPLAY_FLAGS_VSYNC_LOW |
		 DISPLAY_FLAGS_DE_LOW |
		 DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int rad_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct rad_panel *panel;
	struct backlight_properties bl_props;
	int ret;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	ret = of_get_videomode(np, &panel->vm, 0);
	if (ret < 0)
		videomode_from_timing(&rad_default_timing, &panel->vm);

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(panel->reset))
		panel->reset = NULL;
	else
		gpiod_set_value(panel->reset, 0);


	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(
				dev, dev_name(dev),
				dev, dsi,
				&rad_bl_ops, &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&panel->base);
	panel->base.funcs = &rad_panel_funcs;
	panel->base.dev = dev;

	ret = drm_panel_add(&panel->base);

	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&panel->base);

	return ret;
}

static int rad_panel_remove(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = rad_panel_disable(&rad->base);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to disable panel (%d)\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			ret);

	drm_panel_detach(&rad->base);

	if (rad->base.dev)
		drm_panel_remove(&rad->base);

	return 0;
}

static void rad_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);

	rad_panel_disable(&rad->base);
}

static const struct of_device_id rad_of_match[] = {
	{ .compatible = "raydium,rm67191", },
	{ }
};
MODULE_DEVICE_TABLE(of, rad_of_match);

static struct mipi_dsi_driver rad_panel_driver = {
	.driver = {
		.name = "panel-raydium-rm67191",
		.of_match_table = rad_of_match,
	},
	.probe = rad_panel_probe,
	.remove = rad_panel_remove,
	.shutdown = rad_panel_shutdown,
};
module_mipi_dsi_driver(rad_panel_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("Raydium RM67191");
MODULE_LICENSE("GPL v2");
