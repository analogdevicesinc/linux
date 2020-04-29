// SPDX-License-Identifier: GPL-2.0
/*
 * WKS 101WX001-WCT parallel LCD panel driver
 *
 * Copyright 2020 NXP
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

struct wks_panel {
	struct drm_panel panel;
	struct gpio_desc *bl_ctr;
	struct regulator *vcc;

	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 71100,
	.hdisplay = 1280,
	.hsync_start = 1280 + 70,
	.hsync_end = 1280 + 70 + 10,
	.htotal = 1280 + 70 + 10 + 80,
	.vdisplay = 800,
	.vsync_start = 800 + 10,
	.vsync_end = 800 + 10 + 3,
	.vtotal = 800 + 10 + 3 + 10,
	.width_mm = 217,
	.height_mm = 135,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static const u32 wks_bus_formats[] = {
	MEDIA_BUS_FMT_RGB666_1X18,
};

static const u32 wks_bus_flags = DRM_BUS_FLAG_DE_HIGH |
				 DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;

static inline struct wks_panel *to_wks_panel(struct drm_panel *panel)
{
	return container_of(panel, struct wks_panel, panel);
}

static int wks_panel_prepare(struct drm_panel *panel)
{
	struct wks_panel *p = to_wks_panel(panel);
	int err;

	if (p->prepared)
		return 0;

	err = regulator_enable(p->vcc);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable vcc: %d\n", err);
		return err;
	}

	p->prepared = true;

	return 0;
}

static int wks_panel_enable(struct drm_panel *panel)
{
	struct wks_panel *p = to_wks_panel(panel);

	if (p->enabled)
		return 0;

	gpiod_set_value_cansleep(p->bl_ctr, 1);

	p->enabled = true;

	return 0;
}


static int wks_panel_disable(struct drm_panel *panel)
{
	struct wks_panel *p = to_wks_panel(panel);

	if (!p->enabled)
		return 0;

	gpiod_set_value_cansleep(p->bl_ctr, 0);

	p->enabled = false;

	return 0;
}

static int wks_panel_unprepare(struct drm_panel *panel)
{
	struct wks_panel *p = to_wks_panel(panel);

	if (!p->prepared)
		return 0;

	regulator_disable(p->vcc);

	p->prepared = false;

	return 0;
}

static int wks_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		DRM_DEV_ERROR(panel->dev, "failed to add mode %ux%ux@%d\n",
			      default_mode.hdisplay, default_mode.vdisplay,
			      drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	DRM_DEV_DEBUG_DRIVER(panel->dev, "Mode flags: 0x%08X", mode->flags);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = wks_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 wks_bus_formats,
					 ARRAY_SIZE(wks_bus_formats));
	return 1;

}

static const struct drm_panel_funcs wks_panel_funcs = {
	.prepare = wks_panel_prepare,
	.enable = wks_panel_enable,
	.disable = wks_panel_disable,
	.unprepare = wks_panel_unprepare,
	.get_modes = wks_panel_get_modes,
};

static int wks_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wks_panel *panel;
	int err;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(panel->vcc))
		return PTR_ERR(panel->vcc);

	panel->bl_ctr = devm_gpiod_get(dev, "blctr", GPIOD_OUT_LOW);
	if (IS_ERR(panel->bl_ctr)) {
		err = PTR_ERR(panel->bl_ctr);
		dev_err(dev, "Failed to get blctr gpio (%d)\n", err);
		return err;
	}

	drm_panel_init(&panel->panel,
		       dev,
		       &wks_panel_funcs,
		       DRM_MODE_CONNECTOR_DPI);

	drm_panel_add(&panel->panel);
	dev_set_drvdata(dev, panel);

	return 0;
}

static void wks_panel_remove(struct platform_device *pdev)
{
	struct wks_panel *p = dev_get_drvdata(&pdev->dev);

	drm_panel_remove(&p->panel);

	wks_panel_disable(&p->panel);
}

static void wks_panel_shutdown(struct platform_device *pdev)
{
	struct wks_panel *p = dev_get_drvdata(&pdev->dev);

	wks_panel_disable(&p->panel);
}

static const struct of_device_id wks_of_match[] = {
	{ .compatible = "wks,101wx001", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wks_of_match);

static struct platform_driver wks_panel_platform_driver = {
	.driver = {
		.name = "panel-wks-101wx001",
		.of_match_table = wks_of_match,
	},
	.probe = wks_panel_probe,
	.remove = wks_panel_remove,
	.shutdown = wks_panel_shutdown,
};
module_platform_driver(wks_panel_platform_driver);

MODULE_AUTHOR("Marco Franchi <marco.franchi@nxp.com>");
MODULE_DESCRIPTION("Seiko 43WVF1G panel driver");
MODULE_LICENSE("GPL v2");
