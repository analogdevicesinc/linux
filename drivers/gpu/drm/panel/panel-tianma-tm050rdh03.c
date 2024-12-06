// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 NXP
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct tm050rdh03_panel {
	struct drm_panel base;
	struct regulator *vdd;
	struct gpio_desc *stbyb_gpio;	/* standby */
};

static inline struct tm050rdh03_panel *
to_tm050rdh03_panel(struct drm_panel *panel)
{
	return container_of(panel, struct tm050rdh03_panel, base);
}

static int tm050rdh03_disable(struct drm_panel *panel)
{
	struct tm050rdh03_panel *p = to_tm050rdh03_panel(panel);

	gpiod_set_value_cansleep(p->stbyb_gpio, 1);

	/* Wait 8 VSYNCs after the panel is in standby mode. */
	fsleep(140);

	return 0;
}

static int tm050rdh03_unprepare(struct drm_panel *panel)
{
	struct tm050rdh03_panel *p = to_tm050rdh03_panel(panel);

	regulator_disable(p->vdd);

	return 0;
}

static int tm050rdh03_prepare(struct drm_panel *panel)
{
	struct tm050rdh03_panel *p = to_tm050rdh03_panel(panel);
	int ret;

	ret = regulator_enable(p->vdd);
	if (ret < 0) {
		dev_err(panel->dev, "failed to enable regulator: %d\n", ret);
		return ret;
	}

	/* Wait for a while before entering normal operation mode. */
	fsleep(10000);

	gpiod_set_value_cansleep(p->stbyb_gpio, 0);

	return ret;
}

static int tm050rdh03_enable(struct drm_panel *panel)
{
	/* Wait 10 VSYNCs before enabling backlight. */
	fsleep(170000);

	return 0;
}

static const struct drm_display_mode tm050rdh03_mode = {
	.clock = 30000,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 48,
	.htotal = 800 + 40 + 48 + 40,
	.vdisplay = 480,
	.vsync_start = 480 + 13,
	.vsync_end = 480 + 13 + 3,
	.vtotal = 480 + 13 + 3 + 29,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm = 108,
	.height_mm = 65,
};

static int tm050rdh03_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &tm050rdh03_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = tm050rdh03_mode.width_mm;
	connector->display_info.height_mm = tm050rdh03_mode.height_mm;
	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_HIGH |
					    DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE |
					    DRM_BUS_FLAG_PIXDATA_SAMPLE_NEGEDGE;

	return 1;
}

static const struct drm_panel_funcs tm050rdh03_funcs = {
	.disable = tm050rdh03_disable,
	.unprepare = tm050rdh03_unprepare,
	.prepare = tm050rdh03_prepare,
	.enable = tm050rdh03_enable,
	.get_modes = tm050rdh03_get_modes,
};

static int tm050rdh03_probe(struct platform_device *pdev)
{
	struct tm050rdh03_panel *p;
	int ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	platform_set_drvdata(pdev, p);

	p->vdd = devm_regulator_get_optional(&pdev->dev, "power");
	if (IS_ERR(p->vdd))
		return dev_err_probe(&pdev->dev, PTR_ERR(p->vdd),
				     "failed to get regulator\n");

	p->stbyb_gpio = devm_gpiod_get_optional(&pdev->dev, "standby",
						GPIOD_ASIS);
	if (IS_ERR(p->stbyb_gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(p->stbyb_gpio),
				     "failed to get standby gpio\n");

	drm_panel_init(&p->base, &pdev->dev, &tm050rdh03_funcs,
		       DRM_MODE_CONNECTOR_DPI);

	ret = drm_panel_of_backlight(&p->base);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to find backlight\n");

	drm_panel_add(&p->base);

	return 0;
}

static void tm050rdh03_remove(struct platform_device *pdev)
{
	struct tm050rdh03_panel *p = platform_get_drvdata(pdev);

	drm_panel_remove(&p->base);
	drm_panel_disable(&p->base);
	drm_panel_unprepare(&p->base);
}

static const struct of_device_id tm050rdh03_of_match[] = {
	{ .compatible = "tianma,tm050rdh03", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tm050rdh03_of_match);

static struct platform_driver tm050rdh03_driver = {
	.probe		= tm050rdh03_probe,
	.remove_new	= tm050rdh03_remove,
	.driver		= {
		.name = "panel-tianma-tm050rdh03",
		.of_match_table = tm050rdh03_of_match,
	},
};
module_platform_driver(tm050rdh03_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("Tianma TM050RDH03 Panel Driver");
MODULE_LICENSE("GPL");
