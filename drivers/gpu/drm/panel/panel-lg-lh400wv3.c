// SPDX-License-Identifier: GPL-2.0-only

#include <linux/array_size.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

static const struct regulator_bulk_data lg_lh400wv3_supplies[] = {
	{ .supply = "avci" }, { .supply = "iovcc" },
};

struct lg_lh400wv3 {
	struct drm_panel panel;
	struct mipi_dbi *dbi;

	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
};

static inline struct lg_lh400wv3 *to_lg_lh400wv3(struct drm_panel *panel)
{
	return container_of(panel, struct lg_lh400wv3, panel);
}

#define panel_dbi_command(priv, cmd, seq...) \
({ \
	const u8 d[] = { seq }; \
	struct drm_panel *panel = &(priv)->panel; \
	struct device *dev = panel->dev; \
	int ret; \
	ret = mipi_dbi_command_stackbuf((priv)->dbi, cmd, d, ARRAY_SIZE(d)); \
	if (ret) \
		dev_err_ratelimited(dev, "error %d when sending command %#02x\n", ret, cmd); \
	ret; \
})

static int lg_lh400wv3_prepare(struct drm_panel *panel)
{
	struct lg_lh400wv3 *priv = to_lg_lh400wv3(panel);
	struct device *dev = panel->dev;
	int ret;

	priv->dbi = dev_get_drvdata(dev->parent);
	if (!priv->dbi)
		return -ENODEV;

	gpiod_set_value_cansleep(priv->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(lg_lh400wv3_supplies), priv->supplies);
	if (ret) {
		dev_err(dev, "failed to enable power supplies: %d\n", ret);
		return ret;
	}

	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(priv->reset_gpio, 0);

	usleep_range(10000, 11000);

	return 0;
};

static int lg_lh400wv3_enable(struct drm_panel *panel)
{
	struct lg_lh400wv3 *priv = to_lg_lh400wv3(panel);

	panel_dbi_command(priv, MIPI_DCS_EXIT_INVERT_MODE);
	panel_dbi_command(priv, MIPI_DCS_SET_TEAR_ON);

	panel_dbi_command(priv, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_24BIT |
			  MIPI_DCS_PIXEL_FMT_24BIT << 4);

	panel_dbi_command(priv, 0xb2, 0x00, 0xc8);
	panel_dbi_command(priv, 0xb3, 0x00);
	panel_dbi_command(priv, 0xb4, 0x04);
	panel_dbi_command(priv, 0xb5, 0x42, 0x10, 0x10, 0x00, 0x20);
	panel_dbi_command(priv, 0xb6, 0x0b, 0x0f, 0x3c, 0x13, 0x13, 0xe8);
	panel_dbi_command(priv, 0xb7, 0x4c, 0x06, 0x0c, 0x00, 0x00);

	panel_dbi_command(priv, 0xc0, 0x01, 0x11);
	panel_dbi_command(priv, 0xc3, 0x07, 0x03, 0x04, 0x04, 0x04);
	panel_dbi_command(priv, 0xc4, 0x12, 0x24, 0x18, 0x18, 0x02, 0x49);
	panel_dbi_command(priv, 0xc5, 0x65);
	panel_dbi_command(priv, 0xc6, 0x41, 0x63);

	panel_dbi_command(priv, 0xd0, 0x00, 0x46, 0x74, 0x32, 0x1d, 0x03, 0x51, 0x15, 0x04);
	panel_dbi_command(priv, 0xd1, 0x00, 0x46, 0x74, 0x32, 0x1d, 0x03, 0x51, 0x15, 0x04);
	panel_dbi_command(priv, 0xd2, 0x00, 0x46, 0x74, 0x32, 0x1f, 0x03, 0x51, 0x15, 0x04);
	panel_dbi_command(priv, 0xd3, 0x00, 0x46, 0x74, 0x32, 0x1f, 0x03, 0x51, 0x15, 0x04);
	panel_dbi_command(priv, 0xd4, 0x01, 0x46, 0x74, 0x25, 0x00, 0x03, 0x51, 0x15, 0x04);
	panel_dbi_command(priv, 0xd5, 0x01, 0x46, 0x74, 0x25, 0x00, 0x03, 0x51, 0x15, 0x04);

	panel_dbi_command(priv, MIPI_DCS_SET_COLUMN_ADDRESS, 0x00, 0x00, 0x01, 0xdf);
	panel_dbi_command(priv, MIPI_DCS_SET_PAGE_ADDRESS, 0x00, 0x00, 0x03, 0x1f);

	panel_dbi_command(priv, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(120);

	panel_dbi_command(priv, MIPI_DCS_WRITE_MEMORY_START);
	panel_dbi_command(priv, MIPI_DCS_SET_DISPLAY_ON);

	return 0;
}

static int lg_lh400wv3_disable(struct drm_panel *panel)
{
	struct lg_lh400wv3 *priv = to_lg_lh400wv3(panel);

	panel_dbi_command(priv, MIPI_DCS_SET_DISPLAY_OFF);
	panel_dbi_command(priv, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(150);

	panel_dbi_command(priv, 0xc1, 0x01);
	usleep_range(10000, 11000);

	return 0;
};

static int lg_lh400wv3_unprepare(struct drm_panel *panel)
{
	struct lg_lh400wv3 *priv = to_lg_lh400wv3(panel);

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(lg_lh400wv3_supplies), priv->supplies);

	msleep(50);

	return 0;
}

static const struct drm_display_mode lg_lh400wv3_mode = {
	.clock = (480 + 10 + 10 + 10) * (800 + 4 + 4 + 4) * 60 / 1000,
	.hdisplay = 480,
	.hsync_start = 480 + 10,
	.hsync_end = 480 + 10 + 10,
	.htotal = 480 + 10 + 10 + 10,
	.vdisplay = 800,
	.vsync_start = 800 + 4,
	.vsync_end = 800 + 4 + 4,
	.vtotal = 800 + 4 + 4 + 4,
	.width_mm = 52,
	.height_mm = 86,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int lg_lh400wv3_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	drm_display_info_set_bus_formats(&connector->display_info,
					 &bus_format, 1);

	connector->display_info.bus_flags = DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE |
					    DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE |
					    DRM_BUS_FLAG_DE_HIGH |
					    DRM_BUS_FLAG_DATA_LSB_TO_MSB;

	return drm_connector_helper_get_modes_fixed(connector, &lg_lh400wv3_mode);
}

static const struct drm_panel_funcs lg_lh400wv3_panel_funcs = {
	.prepare = lg_lh400wv3_prepare,
	.enable = lg_lh400wv3_enable,
	.disable = lg_lh400wv3_disable,
	.unprepare = lg_lh400wv3_unprepare,
	.get_modes = lg_lh400wv3_get_modes,
};

static int lg_lh400wv3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lg_lh400wv3 *priv;
	int ret;

	priv = devm_drm_panel_alloc(dev, struct lg_lh400wv3, panel,
				    &lg_lh400wv3_panel_funcs, DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(lg_lh400wv3_supplies),
					    lg_lh400wv3_supplies, &priv->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get supplies\n");

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "Failed to get reset gpio\n");

	ret = drm_panel_of_backlight(&priv->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	ret = devm_drm_panel_add(dev, &priv->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add panel\n");

	platform_set_drvdata(pdev, priv);

	return 0;
}

static const struct of_device_id lg_lh400wv3_of_match[] = {
	{ .compatible = "lg,lh400wv3-sd04" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lg_lh400wv3_of_match);

static struct platform_driver lg_lh400wv3_driver = {
	.driver = {
		.name = "panel-lg-lh400wv3",
		.of_match_table = lg_lh400wv3_of_match,
	},
	.probe = lg_lh400wv3_probe,
};
module_platform_driver(lg_lh400wv3_driver);

MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("LG LH400WV3-SD04 MIPI DBI Type B panel driver");
MODULE_LICENSE("GPL");
