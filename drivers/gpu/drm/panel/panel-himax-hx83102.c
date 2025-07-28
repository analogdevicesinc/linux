// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for panels based on Himax HX83102 controller, such as:
 *
 * - Starry 10.51" WUXGA MIPI-DSI panel
 *
 * Based on drivers/gpu/drm/panel/panel-himax-hx8394.c
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

/* Manufacturer specific DSI commands */
#define HX83102_SETPOWER	0xb1
#define HX83102_SETDISP		0xb2
#define HX83102_SETCYC		0xb4
#define HX83102_SETEXTC		0xb9
#define HX83102_SETMIPI		0xba
#define HX83102_SETVDC		0xbc
#define HX83102_SETBANK		0xbd
#define HX83102_UNKNOWN_BE	0xbe
#define HX83102_SETPTBA		0xbf
#define HX83102_SETSTBA		0xc0
#define HX83102_SETTCON		0xc7
#define HX83102_SETRAMDMY	0xc8
#define HX83102_SETPWM		0xc9
#define HX83102_SETCLOCK	0xcb
#define HX83102_SETPANEL	0xcc
#define HX83102_SETCASCADE	0xd0
#define HX83102_SETPCTRL	0xd1
#define HX83102_UNKNOWN_D2	0xd2
#define HX83102_SETGIP0		0xd3
#define HX83102_SETGIP1		0xd5
#define HX83102_SETGIP2		0xd6
#define HX83102_SETGIP3		0xd8
#define HX83102_SETGMA		0xe0
#define HX83102_UNKNOWN_E1	0xe1
#define HX83102_SETTP1		0xe7
#define HX83102_SETSPCCMD	0xe9

struct hx83102 {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	const struct hx83102_panel_desc *desc;

	enum drm_panel_orientation orientation;
	struct regulator *pp1800;
	struct regulator *avee;
	struct regulator *avdd;
	struct gpio_desc *enable_gpio;
};

struct hx83102_panel_desc {
	const struct drm_display_mode *modes;

	/**
	 * @width_mm: width of the panel's active display area
	 * @height_mm: height of the panel's active display area
	 */
	struct {
		unsigned int width_mm;
		unsigned int height_mm;
	} size;

	int (*init)(struct hx83102 *ctx);
};

static inline struct hx83102 *panel_to_hx83102(struct drm_panel *panel)
{
	return container_of(panel, struct hx83102, base);
}

static void hx83102_enable_extended_cmds(struct mipi_dsi_multi_context *dsi_ctx, bool enable)
{
	if (enable)
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, HX83102_SETEXTC, 0x83, 0x10, 0x21, 0x55, 0x00);
	else
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, HX83102_SETEXTC, 0x00, 0x00, 0x00);
}

static int starry_himax83102_j02_init(struct hx83102 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	hx83102_enable_extended_cmds(&dsi_ctx, true);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPOWER, 0x2c, 0xb5, 0xb5, 0x31, 0xf1,
				     0x31, 0xd7, 0x2f, 0x36, 0x36, 0x36, 0x36, 0x1a, 0x8b, 0x11,
				     0x65, 0x00, 0x88, 0xfa, 0xff, 0xff, 0x8f, 0xff, 0x08, 0x74,
				     0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETDISP, 0x00, 0x47, 0xb0, 0x80, 0x00,
				     0x12, 0x72, 0x3c, 0xa3, 0x03, 0x03, 0x00, 0x00, 0x88, 0xf5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCYC, 0x76, 0x76, 0x76, 0x76, 0x76,
				     0x76, 0x63, 0x5c, 0x63, 0x5c, 0x01, 0x9e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xcd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETVDC, 0x1b, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_BE, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPTBA, 0xfc, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSTBA, 0x36, 0x36, 0x22, 0x11, 0x22,
				     0xa0, 0x61, 0x08, 0xf5, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTCON, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETRAMDMY, 0x97);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPWM, 0x00, 0x1e, 0x13, 0x88, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCLOCK, 0x08, 0x13, 0x07, 0x00, 0x0f, 0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPANEL, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCASCADE, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPCTRL, 0x37, 0x06, 0x00, 0x02, 0x04, 0x0c,
				     0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_D2, 0x1f, 0x11, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x08, 0x00, 0x08, 0x37, 0x47, 0x34, 0x3b, 0x12, 0x12, 0x03, 0x03,
				     0x32, 0x10, 0x10, 0x00, 0x10, 0x32, 0x10, 0x08, 0x00, 0x08, 0x32,
				     0x17, 0x94, 0x07, 0x94, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP1, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				     0x18, 0x18, 0x18, 0x18, 0x19, 0x19, 0x40, 0x40, 0x1a, 0x1a, 0x1b,
				     0x1b, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x20, 0x21,
				     0x28, 0x29, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				     0x18, 0x18, 0x18, 0x18, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP2, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				     0x18, 0x18, 0x18, 0x18, 0x40, 0x40, 0x19, 0x19, 0x1a, 0x1a, 0x1b,
				     0x1b, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x29, 0x28,
				     0x21, 0x20, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				     0x18, 0x18, 0x18, 0x18, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xaa, 0xba, 0xea, 0xaa, 0xaa, 0xa0,
				     0xaa, 0xba, 0xea, 0xaa, 0xaa, 0xa0, 0xaa, 0xba, 0xea, 0xaa, 0xaa,
				     0xa0, 0xaa, 0xba, 0xea, 0xaa, 0xaa, 0xa0, 0xaa, 0xba, 0xea, 0xaa,
				     0xaa, 0xa0, 0xaa, 0xba, 0xea, 0xaa, 0xaa, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGMA, 0x00, 0x09, 0x14, 0x1e, 0x26, 0x48,
				     0x61, 0x67, 0x6c, 0x67, 0x7d, 0x7f, 0x80, 0x8b, 0x87, 0x8f, 0x98,
				     0xab, 0xab, 0x55, 0x5c, 0x68, 0x73, 0x00, 0x09, 0x14, 0x1e, 0x26,
				     0x48, 0x61, 0x67, 0x6c, 0x67, 0x7d, 0x7f, 0x80, 0x8b, 0x87, 0x8f,
				     0x98, 0xab, 0xab, 0x55, 0x5c, 0x68, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0x0e, 0x10, 0x10, 0x21, 0x2b, 0x9a,
				     0x02, 0x54, 0x9a, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x12, 0x05,
				     0x02, 0x02, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPOWER, 0x01, 0xbf, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCLOCK, 0x86);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_D2, 0x3c, 0xfa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP0, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x80, 0x0c, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0x02, 0x00, 0x28, 0x01, 0x7e, 0x0f,
				     0x7e, 0x10, 0xa0, 0x00, 0x00, 0x20, 0x40, 0x50, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xff, 0xff, 0xbf, 0xfe, 0xaa, 0xa0,
				     0xff, 0xff, 0xbf, 0xfe, 0xaa, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0xfe, 0x04, 0xfe, 0x04, 0xfe, 0x04,
				     0x03, 0x03, 0x03, 0x26, 0x00, 0x26, 0x81, 0x02, 0x40, 0x00, 0x20,
				     0x9e, 0x04, 0x03, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCYC, 0x03, 0xff, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0x00, 0x2a, 0xaa, 0xa8, 0x00, 0x00,
				     0x00, 0x2a, 0xaa, 0xa8, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xfc, 0x00,
				     0x00, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x2a, 0xaa, 0xa8,
				     0x00, 0x00, 0x00, 0x2a, 0xaa, 0xa8, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x4f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x00);

	return dsi_ctx.accum_err;
};

static int boe_nv110wum_init(struct hx83102 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	msleep(60);

	hx83102_enable_extended_cmds(&dsi_ctx, true);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPOWER, 0x2c, 0xaf, 0xaf, 0x2b, 0xeb, 0x42,
				     0xe1, 0x4d, 0x36, 0x36, 0x36, 0x36, 0x1a, 0x8b, 0x11, 0x65, 0x00,
				     0x88, 0xfa, 0xff, 0xff, 0x8f, 0xff, 0x08, 0x9a, 0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETDISP, 0x00, 0x47, 0xb0, 0x80, 0x00, 0x12,
				     0x71, 0x3c, 0xa3, 0x11, 0x00, 0x00, 0x00, 0x88, 0xf5, 0x22, 0x8f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCYC, 0x49, 0x49, 0x32, 0x32, 0x14, 0x32,
				     0x84, 0x6e, 0x84, 0x6e, 0x01, 0x9c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xcd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETVDC, 0x1b, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_BE, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPTBA, 0xfc, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSTBA, 0x36, 0x36, 0x22, 0x00, 0x00, 0xa0,
				     0x61, 0x08, 0xf5, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTCON, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETRAMDMY, 0x97);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPWM, 0x00, 0x1e, 0x30, 0xd4, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCLOCK, 0x08, 0x13, 0x07, 0x00, 0x0f, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPANEL, 0x02, 0x03, 0x44);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCASCADE, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPCTRL, 0x37, 0x06, 0x00, 0x02, 0x04, 0x0c, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_D2, 0x1f, 0x11, 0x1f, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x04,
				     0x08, 0x04, 0x08, 0x37, 0x37, 0x64, 0x4b, 0x11, 0x11, 0x03, 0x03, 0x32,
				     0x10, 0x0e, 0x00, 0x0e, 0x32, 0x10, 0x0a, 0x00, 0x0a, 0x32, 0x17, 0x98,
				     0x07, 0x98, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP1, 0x18, 0x18, 0x18, 0x18, 0x1e, 0x1e,
				     0x1e, 0x1e, 0x1f, 0x1f, 0x1f, 0x1f, 0x24, 0x24, 0x24, 0x24, 0x07, 0x06,
				     0x07, 0x06, 0x05, 0x04, 0x05, 0x04, 0x03, 0x02, 0x03, 0x02, 0x01, 0x00,
				     0x01, 0x00, 0x21, 0x20, 0x21, 0x20, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				     0x18, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xaf, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0,
				     0xaf, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGMA, 0x00, 0x05, 0x0d, 0x14, 0x1b, 0x2c,
				     0x44, 0x49, 0x51, 0x4c, 0x67, 0x6c, 0x71, 0x80, 0x7d, 0x84, 0x8d, 0xa0,
				     0xa0, 0x4f, 0x58, 0x64, 0x73, 0x00, 0x05, 0x0d, 0x14, 0x1b, 0x2c, 0x44,
				     0x49, 0x51, 0x4c, 0x67, 0x6c, 0x71, 0x80, 0x7d, 0x84, 0x8d, 0xa0, 0xa0,
				     0x4f, 0x58, 0x64, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0x07, 0x10, 0x10, 0x1a, 0x26, 0x9e,
				     0x00, 0x53, 0x9b, 0x14, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_E1, 0x11, 0x00, 0x00, 0x89, 0x30, 0x80,
				     0x07, 0x80, 0x02, 0x58, 0x00, 0x14, 0x02, 0x58, 0x02, 0x58, 0x02, 0x00,
				     0x02, 0x2c, 0x00, 0x20, 0x02, 0x02, 0x00, 0x08, 0x00, 0x0c, 0x05, 0x0e,
				     0x04, 0x94, 0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b,
				     0x0b, 0x33, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xff, 0xff, 0xff, 0xff, 0xfa, 0xa0,
				     0xff, 0xff, 0xff, 0xff, 0xfa, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPOWER, 0x01, 0xbf, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCLOCK, 0x86);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_D2, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP0, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xd1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_E1, 0xf6, 0x2b, 0x34, 0x2b, 0x74, 0x3b,
				     0x74, 0x6b, 0x74);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0x02, 0x00, 0x2b, 0x01, 0x7e, 0x0f,
				     0x7e, 0x10, 0xa0, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCYC, 0x02, 0x00, 0xbb, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xff, 0xaf, 0xff, 0xff, 0xfa, 0xa0,
				     0xff, 0xaf, 0xff, 0xff, 0xfa, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01,
				     0x00, 0x00, 0x00, 0x23, 0x00, 0x23, 0x81, 0x02, 0x40, 0x00, 0x20, 0x65,
				     0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xaa, 0xaf, 0xaa, 0xaa, 0xa0, 0x00,
				     0xaa, 0xaf, 0xaa, 0xaa, 0xa0, 0x00, 0xaa, 0xaf, 0xaa, 0xaa, 0xa0, 0x00,
				     0xaa, 0xaf, 0xaa, 0xaa, 0xa0, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCYC, 0x03, 0xff, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_E1, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x4f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x00);
	hx83102_enable_extended_cmds(&dsi_ctx, false);

	mipi_dsi_msleep(&dsi_ctx, 50);

	return dsi_ctx.accum_err;
};

static int ivo_t109nw41_init(struct hx83102 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	msleep(60);

	hx83102_enable_extended_cmds(&dsi_ctx, true);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPOWER, 0x2c, 0xed, 0xed, 0x27, 0xe7, 0x52,
				     0xf5, 0x39, 0x36, 0x36, 0x36, 0x36, 0x32, 0x8b, 0x11, 0x65, 0x00, 0x88,
				     0xfa, 0xff, 0xff, 0x8f, 0xff, 0x08, 0xd6, 0x33);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETDISP, 0x00, 0x47, 0xb0, 0x80, 0x00, 0x12,
				     0x71, 0x3c, 0xa3, 0x22, 0x20, 0x00, 0x00, 0x88, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCYC, 0x35, 0x35, 0x43, 0x43, 0x35, 0x35,
				     0x30, 0x7a, 0x30, 0x7a, 0x01, 0x9d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xcd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETVDC, 0x1b, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_BE, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPTBA, 0xfc, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSTBA, 0x34, 0x34, 0x22, 0x11, 0x22, 0xa0,
				     0x31, 0x08, 0xf5, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTCON, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xd3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTCON, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETRAMDMY, 0x97);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPWM, 0x00, 0x1e, 0x13, 0x88, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCLOCK, 0x08, 0x13, 0x07, 0x00, 0x0f, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPANEL, 0x02, 0x03, 0x44);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCASCADE, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPCTRL, 0x07, 0x06, 0x00, 0x02, 0x04, 0x2c,
				     0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x08,
				     0x08, 0x08, 0x08, 0x37, 0x07, 0x64, 0x7c, 0x11, 0x11, 0x03, 0x03, 0x32,
				     0x10, 0x0e, 0x00, 0x0e, 0x32, 0x17, 0x97, 0x07, 0x97, 0x32, 0x00, 0x02,
				     0x00, 0x02, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP1, 0x25, 0x24, 0x25, 0x24, 0x18, 0x18,
				     0x18, 0x18, 0x07, 0x06, 0x07, 0x06, 0x05, 0x04, 0x05, 0x04, 0x03, 0x02,
				     0x03, 0x02, 0x01, 0x00, 0x01, 0x00, 0x1e, 0x1e, 0x1e, 0x1e, 0x1f, 0x1f,
				     0x1f, 0x1f, 0x21, 0x20, 0x21, 0x20, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				     0x18, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0,
				     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGMA, 0x00, 0x07, 0x10, 0x17, 0x1c, 0x33,
				     0x48, 0x50, 0x57, 0x50, 0x68, 0x6e, 0x71, 0x7f, 0x81, 0x8a, 0x8e, 0x9b,
				     0x9c, 0x4d, 0x56, 0x5d, 0x73, 0x00, 0x07, 0x10, 0x17, 0x1c, 0x33, 0x48,
				     0x50, 0x57, 0x50, 0x68, 0x6e, 0x71, 0x7f, 0x81, 0x8a, 0x8e, 0x9b, 0x9c,
				     0x4d, 0x56, 0x5d, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0x07, 0x10, 0x10, 0x1a, 0x26, 0x9e,
				     0x00, 0x4f, 0xa0, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x12, 0x0a, 0x02,
				     0x02, 0x00, 0x33, 0x02, 0x04, 0x18, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPOWER, 0x01, 0x7f, 0x11, 0xfd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCLOCK, 0x86);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP0, 0x00, 0x00, 0x04, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0,
				     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0x02, 0x00, 0x2b, 0x01, 0x7e, 0x0f,
				     0x7e, 0x10, 0xa0, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETPTBA, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCLOCK, 0x03, 0x07, 0x00, 0x10, 0x79);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETGIP3, 0xff, 0xff, 0xff, 0xff, 0xfa, 0xa0,
				     0xff, 0xff, 0xff, 0xff, 0xfa, 0xa0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETTP1, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01,
				     0x00, 0x00, 0x00, 0x23, 0x00, 0x23, 0x81, 0x02, 0x40, 0x00, 0x20, 0x6e,
				     0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0,
				     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0, 0xff, 0xff, 0xff, 0xff, 0xfa, 0xa0,
				     0xff, 0xff, 0xff, 0xff, 0xfa, 0xa0, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0,
				     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETCYC, 0x03, 0xff, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_E1, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_UNKNOWN_D2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0xc5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETMIPI, 0x4f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETSPCCMD, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, HX83102_SETBANK, 0x00);
	hx83102_enable_extended_cmds(&dsi_ctx, false);

	mipi_dsi_msleep(&dsi_ctx, 60);

	return dsi_ctx.accum_err;
};

static const struct drm_display_mode starry_mode = {
	.clock = 162680,
	.hdisplay = 1200,
	.hsync_start = 1200 + 60,
	.hsync_end = 1200 + 60 + 20,
	.htotal = 1200 + 60 + 20 + 40,
	.vdisplay = 1920,
	.vsync_start = 1920 + 116,
	.vsync_end = 1920 + 116 + 8,
	.vtotal = 1920 + 116 + 8 + 12,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct hx83102_panel_desc starry_desc = {
	.modes = &starry_mode,
	.size = {
		.width_mm = 141,
		.height_mm = 226,
	},
	.init = starry_himax83102_j02_init,
};

static const struct drm_display_mode boe_tv110wum_default_mode = {
	.clock = 167700,
	.hdisplay = 1200,
	.hsync_start = 1200 + 75,
	.hsync_end = 1200 + 75 + 20,
	.htotal = 1200 + 75 + 20 + 65,
	.vdisplay = 1920,
	.vsync_start = 1920 + 115,
	.vsync_end = 1920 + 115 + 8,
	.vtotal = 1920 + 115 + 8 + 12,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct hx83102_panel_desc boe_nv110wum_desc = {
	.modes = &boe_tv110wum_default_mode,
	.size = {
		.width_mm = 147,
		.height_mm = 235,
	},
	.init = boe_nv110wum_init,
};

static const struct drm_display_mode ivo_t109nw41_default_mode = {
	.clock = 167700,
	.hdisplay = 1200,
	.hsync_start = 1200 + 75,
	.hsync_end = 1200 + 75 + 20,
	.htotal = 1200 + 75 + 20 + 65,
	.vdisplay = 1920,
	.vsync_start = 1920 + 115,
	.vsync_end = 1920 + 115 + 8,
	.vtotal = 1920 + 115 + 8 + 12,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct hx83102_panel_desc ivo_t109nw41_desc = {
	.modes = &ivo_t109nw41_default_mode,
	.size = {
		.width_mm = 147,
		.height_mm = 235,
	},
	.init = ivo_t109nw41_init,
};

static int hx83102_enable(struct drm_panel *panel)
{
	msleep(130);
	return 0;
}

static int hx83102_disable(struct drm_panel *panel)
{
	struct hx83102 *ctx = panel_to_hx83102(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 150);

	return dsi_ctx.accum_err;
}

static int hx83102_unprepare(struct drm_panel *panel)
{
	struct hx83102 *ctx = panel_to_hx83102(panel);

	gpiod_set_value(ctx->enable_gpio, 0);
	usleep_range(1000, 2000);
	regulator_disable(ctx->avee);
	regulator_disable(ctx->avdd);
	usleep_range(5000, 7000);
	regulator_disable(ctx->pp1800);

	return 0;
}

static int hx83102_prepare(struct drm_panel *panel)
{
	struct hx83102 *ctx = panel_to_hx83102(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	gpiod_set_value(ctx->enable_gpio, 0);
	usleep_range(1000, 1500);

	dsi_ctx.accum_err = regulator_enable(ctx->pp1800);
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	usleep_range(3000, 5000);

	dsi_ctx.accum_err = regulator_enable(ctx->avdd);
	if (dsi_ctx.accum_err)
		goto poweroff1v8;
	dsi_ctx.accum_err = regulator_enable(ctx->avee);
	if (dsi_ctx.accum_err)
		goto poweroffavdd;

	usleep_range(10000, 11000);

	mipi_dsi_dcs_nop_multi(&dsi_ctx);
	if (dsi_ctx.accum_err)
		goto poweroff;

	usleep_range(1000, 2000);

	gpiod_set_value(ctx->enable_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value(ctx->enable_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value(ctx->enable_gpio, 1);
	usleep_range(6000, 10000);

	dsi_ctx.accum_err = ctx->desc->init(ctx);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	if (dsi_ctx.accum_err)
		goto poweroff;

	return 0;

poweroff:
	gpiod_set_value(ctx->enable_gpio, 0);
	regulator_disable(ctx->avee);
poweroffavdd:
	regulator_disable(ctx->avdd);
poweroff1v8:
	usleep_range(5000, 7000);
	regulator_disable(ctx->pp1800);

	return dsi_ctx.accum_err;
}

static int hx83102_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct hx83102 *ctx = panel_to_hx83102(panel);
	const struct drm_display_mode *m = ctx->desc->modes;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, m);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = ctx->desc->size.width_mm;
	connector->display_info.height_mm = ctx->desc->size.height_mm;
	connector->display_info.bpc = 8;

	return 1;
}

static enum drm_panel_orientation hx83102_get_orientation(struct drm_panel *panel)
{
	struct hx83102 *ctx = panel_to_hx83102(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs hx83102_drm_funcs = {
	.disable   = hx83102_disable,
	.unprepare = hx83102_unprepare,
	.prepare   = hx83102_prepare,
	.enable    = hx83102_enable,
	.get_modes = hx83102_get_modes,
	.get_orientation = hx83102_get_orientation,
};

static int hx83102_panel_add(struct hx83102 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	int err;

	ctx->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(ctx->avdd))
		return PTR_ERR(ctx->avdd);

	ctx->avee = devm_regulator_get(dev, "avee");
	if (IS_ERR(ctx->avee))
		return PTR_ERR(ctx->avee);

	ctx->pp1800 = devm_regulator_get(dev, "pp1800");
	if (IS_ERR(ctx->pp1800))
		return PTR_ERR(ctx->pp1800);

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->enable_gpio), "Cannot get enable GPIO\n");

	ctx->base.prepare_prev_first = true;

	drm_panel_init(&ctx->base, dev, &hx83102_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	err = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (err < 0)
		return dev_err_probe(dev, err, "failed to get orientation\n");

	err = drm_panel_of_backlight(&ctx->base);
	if (err)
		return err;

	ctx->base.funcs = &hx83102_drm_funcs;
	ctx->base.dev = &ctx->dsi->dev;

	drm_panel_add(&ctx->base);

	return 0;
}

static int hx83102_probe(struct mipi_dsi_device *dsi)
{
	struct hx83102 *ctx;
	int ret;
	const struct hx83102_panel_desc *desc;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
					  MIPI_DSI_MODE_LPM;
	ctx->desc = desc;
	ctx->dsi = dsi;
	ret = hx83102_panel_add(ctx);
	if (ret < 0)
		return ret;

	mipi_dsi_set_drvdata(dsi, ctx);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&ctx->base);

	return ret;
}

static void hx83102_remove(struct mipi_dsi_device *dsi)
{
	struct hx83102 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	if (ctx->base.dev)
		drm_panel_remove(&ctx->base);
}

static const struct of_device_id hx83102_of_match[] = {
	{ .compatible = "boe,nv110wum-l60",
	.data = &boe_nv110wum_desc
	},
	{ .compatible = "ivo,t109nw41",
	  .data = &ivo_t109nw41_desc
	},
	{ .compatible = "starry,himax83102-j02",
	  .data = &starry_desc
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hx83102_of_match);

static struct mipi_dsi_driver hx83102_driver = {
	.probe	= hx83102_probe,
	.remove = hx83102_remove,
	.driver = {
		.name = "panel-himax-hx83102",
		.of_match_table = hx83102_of_match,
	},
};
module_mipi_dsi_driver(hx83102_driver);

MODULE_AUTHOR("Cong Yang <yangcong5@huaqin.corp-partner.google.com>");
MODULE_DESCRIPTION("DRM driver for Himax HX83102 based MIPI DSI panels");
MODULE_LICENSE("GPL");
