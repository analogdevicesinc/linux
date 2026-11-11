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

static const struct regulator_bulk_data hitachi_tx10d07vm0baa_supplies[] = {
	{ .supply = "avci" }, { .supply = "iovcc" },
};

struct hitachi_tx10d07vm0baa {
	struct drm_panel panel;
	struct mipi_dbi *dbi;

	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
};

static inline struct hitachi_tx10d07vm0baa *to_hitachi_tx10d07vm0baa(struct drm_panel *panel)
{
	return container_of(panel, struct hitachi_tx10d07vm0baa, panel);
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

static int hitachi_tx10d07vm0baa_prepare(struct drm_panel *panel)
{
	struct hitachi_tx10d07vm0baa *priv = to_hitachi_tx10d07vm0baa(panel);
	struct device *dev = panel->dev;
	int ret;

	priv->dbi = dev_get_drvdata(dev->parent);
	if (!priv->dbi)
		return -ENODEV;

	gpiod_set_value_cansleep(priv->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(hitachi_tx10d07vm0baa_supplies), priv->supplies);
	if (ret) {
		dev_err(dev, "failed to enable power supplies: %d\n", ret);
		return ret;
	}

	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(priv->reset_gpio, 0);

	usleep_range(10000, 11000);

	return 0;
};

static int hitachi_tx10d07vm0baa_enable(struct drm_panel *panel)
{
	struct hitachi_tx10d07vm0baa *priv = to_hitachi_tx10d07vm0baa(panel);

	panel_dbi_command(priv, MIPI_DCS_SET_PARTIAL_ROWS, 0x00, 0x00, 0x03, 0x1f);
	panel_dbi_command(priv, MIPI_DCS_SET_SCROLL_AREA, 0x00, 0x00, 0x03, 0x20,
			  0x00, 0x00);

	panel_dbi_command(priv, MIPI_DCS_SET_ADDRESS_MODE, 0x0a);
	panel_dbi_command(priv, MIPI_DCS_SET_SCROLL_START, 0x00, 0x00);

	panel_dbi_command(priv, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_24BIT);
	panel_dbi_command(priv, MIPI_DCS_SET_TEAR_SCANLINE, 0x00, 0x00);

	panel_dbi_command(priv, 0x71, 0x00);		/* Ex_Vsync_en */

	panel_dbi_command(priv, 0xb2, 0x00);		/* VCSEL */
	panel_dbi_command(priv, 0xb4, 0xaa);		/* setvgmpm */
	panel_dbi_command(priv, 0xb5, 0x33);		/* rbias1 */
	panel_dbi_command(priv, 0xb6, 0x03);		/* rbias2 */

	panel_dbi_command(priv, 0xb7, 0x1a, 0x33, 0x03, 0x03,
			  0x03, 0x00, 0x00, 0x01, 0x02, 0x00,
			  0x00, 0x04, 0x00, 0x01, 0x01, 0x01);	/* set_ddvdhp */
	panel_dbi_command(priv, 0xb8, 0x1c, 0x53, 0x03, 0x03,
			  0x00, 0x01, 0x02, 0x00, 0x00, 0x04,
			  0x00, 0x01, 0x01);			/* set_ddvdhm */

	panel_dbi_command(priv, 0xb9, 0x0a, 0x01, 0x01, 0x00,
			  0x00, 0x00, 0x02, 0x00, 0x02, 0x01);	/* set_vgh */
	panel_dbi_command(priv, 0xba, 0x0f, 0x01, 0x01, 0x00,
			  0x00, 0x00, 0x02, 0x00, 0x02, 0x01);	/* set_vgl */
	panel_dbi_command(priv, 0xbb, 0x00, 0x00, 0x00, 0x00,
			  0x01, 0x02, 0x01);			/* set_vcl */

	panel_dbi_command(priv, 0xc1, 0x01);		/* number of lines */
	panel_dbi_command(priv, 0xc2, 0x08);		/* number of fp lines */
	panel_dbi_command(priv, 0xc3, 0x04);		/* gateset(1) */
	panel_dbi_command(priv, 0xc4, 0x4c);		/* 1h period */
	panel_dbi_command(priv, 0xc5, 0x03);		/* source precharge */
	panel_dbi_command(priv, 0xc6, 0xc4, 0x04);	/* source precharge timing */
	panel_dbi_command(priv, 0xc7, 0x00);		/* source level */
	panel_dbi_command(priv, 0xc8, 0x02);		/* number of bp lines */
	panel_dbi_command(priv, 0xc9, 0x10);		/* gateset(2) */
	panel_dbi_command(priv, 0xca, 0x04, 0x04);	/* gateset(3) */
	panel_dbi_command(priv, 0xcb, 0x03);		/* gateset(4) */
	panel_dbi_command(priv, 0xcc, 0x12);		/* gateset(5) */
	panel_dbi_command(priv, 0xcd, 0x12);		/* gateset(6) */
	panel_dbi_command(priv, 0xce, 0x30);		/* gateset(7) */
	panel_dbi_command(priv, 0xcf, 0x30);		/* gateset(8) */
	panel_dbi_command(priv, 0xd0, 0x40);		/* gateset(9) */
	panel_dbi_command(priv, 0xd1, 0x22);		/* flhw */
	panel_dbi_command(priv, 0xd2, 0x22);		/* vckhw */
	panel_dbi_command(priv, 0xd3, 0x04);		/* flt */
	panel_dbi_command(priv, 0xd4, 0x14);		/* tctrl */
	panel_dbi_command(priv, 0xd6, 0x02);		/* dotinv */
	panel_dbi_command(priv, 0xd7, 0x00);		/* on/off sequence period */

	panel_dbi_command(priv, 0xd8, 0x01, 0x05, 0x06, 0x0d,
			  0x18, 0x09, 0x22, 0x23, 0x00);		/* ponseqa */
	panel_dbi_command(priv, 0xd9, 0x24, 0x01);			/* ponseqb */
	panel_dbi_command(priv, 0xde, 0x09, 0x0f, 0x21, 0x12,
			  0x04);					/* ponseqc */

	panel_dbi_command(priv, 0xdf, 0x02, 0x06, 0x06, 0x06,
			  0x06, 0x00);					/* pofseqa */
	panel_dbi_command(priv, 0xe0, 0x01);				/* pofseqb */

	panel_dbi_command(priv, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0xff);
	panel_dbi_command(priv, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x40);

	panel_dbi_command(priv, 0xe2, 0x00, 0x00);		/* cabc pwm */
	panel_dbi_command(priv, 0xe3, 0x03);			/* cabc */
	panel_dbi_command(priv, 0xe4, 0x66, 0x7b, 0x90, 0xa5,
			  0xbb, 0xc7, 0xe1, 0xe5);		/* cabc brightness */
	panel_dbi_command(priv, 0xe5, 0xc5, 0xc5, 0xc9, 0xc9,
			  0xd1, 0xe1, 0xf1, 0xfe);		/* cabc brightness */
	panel_dbi_command(priv, 0xe7, 0x2a);			/* cabc */
	panel_dbi_command(priv, 0xe8, 0x00);			/* brt_rev */
	panel_dbi_command(priv, 0xe9, 0x00);			/* tefreq */

	panel_dbi_command(priv, 0xea, 0x01);			/* high speed ram */

	panel_dbi_command(priv, 0xeb, 0x00, 0x33, 0x0e, 0x15,
			  0xb7, 0x78, 0x88, 0x0f);		/* gamma setting r pos */
	panel_dbi_command(priv, 0xec, 0x00, 0x33, 0x0e, 0x15,
			  0xb7, 0x78, 0x88, 0x0f);		/* gamma setting r neg */
	panel_dbi_command(priv, 0xed, 0x00, 0x33, 0x0e, 0x15,
			  0xb7, 0x78, 0x88, 0x0f);		/* gamma setting g pos */
	panel_dbi_command(priv, 0xee, 0x00, 0x33, 0x0e, 0x15,
			  0xb7, 0x78, 0x88, 0x0f);		/* gamma setting g neg */
	panel_dbi_command(priv, 0xef, 0x00, 0x33, 0x0e, 0x15,
			  0xb7, 0x78, 0x88, 0x0f);		/* gamma setting b pos */
	panel_dbi_command(priv, 0xf0, 0x00, 0x33, 0x0e, 0x15,
			  0xb7, 0x78, 0x88, 0x0f);		/* gamma setting b neg */

	panel_dbi_command(priv, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(110);

	panel_dbi_command(priv, MIPI_DCS_SET_DISPLAY_ON);

	return 0;
}

static int hitachi_tx10d07vm0baa_disable(struct drm_panel *panel)
{
	struct hitachi_tx10d07vm0baa *priv = to_hitachi_tx10d07vm0baa(panel);

	panel_dbi_command(priv, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(35);

	panel_dbi_command(priv, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(50);

	return 0;
}

static int hitachi_tx10d07vm0baa_unprepare(struct drm_panel *panel)
{
	struct hitachi_tx10d07vm0baa *priv = to_hitachi_tx10d07vm0baa(panel);

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(hitachi_tx10d07vm0baa_supplies), priv->supplies);

	msleep(50);

	return 0;
}

static const struct drm_display_mode hitachi_tx10d07vm0baa_mode = {
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

static int hitachi_tx10d07vm0baa_get_modes(struct drm_panel *panel,
					   struct drm_connector *connector)
{
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	drm_display_info_set_bus_formats(&connector->display_info,
					 &bus_format, 1);

	connector->display_info.bus_flags = DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE |
					    DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE |
					    DRM_BUS_FLAG_DE_HIGH |
					    DRM_BUS_FLAG_DATA_LSB_TO_MSB;

	return drm_connector_helper_get_modes_fixed(connector, &hitachi_tx10d07vm0baa_mode);
}

static const struct drm_panel_funcs hitachi_tx10d07vm0baa_panel_funcs = {
	.prepare = hitachi_tx10d07vm0baa_prepare,
	.enable = hitachi_tx10d07vm0baa_enable,
	.disable = hitachi_tx10d07vm0baa_disable,
	.unprepare = hitachi_tx10d07vm0baa_unprepare,
	.get_modes = hitachi_tx10d07vm0baa_get_modes,
};

static int hitachi_tx10d07vm0baa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hitachi_tx10d07vm0baa *priv;
	int ret;

	priv = devm_drm_panel_alloc(dev, struct hitachi_tx10d07vm0baa, panel,
				    &hitachi_tx10d07vm0baa_panel_funcs, DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(hitachi_tx10d07vm0baa_supplies),
					    hitachi_tx10d07vm0baa_supplies, &priv->supplies);
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

static const struct of_device_id hitachi_tx10d07vm0baa_of_match[] = {
	{ .compatible = "hit,tx10d07vm0baa" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hitachi_tx10d07vm0baa_of_match);

static struct platform_driver hitachi_tx10d07vm0baa_driver = {
	.driver = {
		.name = "panel-hitachi-tx10d07vm0baa",
		.of_match_table = hitachi_tx10d07vm0baa_of_match,
	},
	.probe = hitachi_tx10d07vm0baa_probe,
};
module_platform_driver(hitachi_tx10d07vm0baa_driver);

MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("HITACHI TX10D07VM0BAA MIPI DBI Type B panel driver");
MODULE_LICENSE("GPL");
