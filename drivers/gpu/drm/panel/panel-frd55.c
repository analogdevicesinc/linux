/*
 * Copyright (C) Emcraft 2018
 *
 * Authors: Dmitry Konyshev <probables@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <video/mipi_display.h>

#define DRV_NAME "frd55"

struct frd55 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	bool prepared;
	bool enabled;
};

#define HDISP 1080
#define VDISP 1920

static const struct drm_display_mode default_mode = {
	.clock = 125000,
	.hdisplay = HDISP,
	.hsync_start = HDISP*2 + 12,
	.hsync_end = HDISP*2 + 12 + 2,
	.htotal = HDISP*2 + 12 + 2 + 18,
	.vdisplay = VDISP,
	.vsync_start = VDISP + 4,
	.vsync_end = VDISP + 4 + 2,
	.vtotal = VDISP + 4 + 2 + 7,
	.vrefresh = 50,
	.flags = 0,
	.width_mm = 68,
	.height_mm = 120,
};

static inline struct frd55 *panel_to_frd55(struct drm_panel *panel)
{
	return container_of(panel, struct frd55, panel);
}

static void frd55_dcs_write_buf(struct frd55 *ctx, const void *data,
				   size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (mipi_dsi_generic_write(dsi, data, len) < 0)
		DRM_WARN("mipi dsi dcs write buffer failed\n");
}

#define dcs_write_seq(ctx, seq...)			\
({							\
	static const u8 d[] = { seq };			\
	frd55_dcs_write_buf(ctx, d, ARRAY_SIZE(d));	\
})

static int frd55_init_sequence(struct frd55 *ctx)
{
	dcs_write_seq(ctx, 0xFF, 0x01);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xFF, 0x02);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xFF, 0x03);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xFF, 0x04);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xFF, 0x05);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xD7, 0x31);
	dcs_write_seq(ctx, 0xD8, 0x7E);
	mdelay(100);
	dcs_write_seq(ctx, 0xFF, 0x00);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xBA, 0x03);
	dcs_write_seq(ctx, 0x36, 0x00);
	dcs_write_seq(ctx, 0xB0, 0x00);
	dcs_write_seq(ctx, 0xD3, 0x08);
	dcs_write_seq(ctx, 0xD4, 0x0E);
	dcs_write_seq(ctx, 0xD5, 0x0F);
	dcs_write_seq(ctx, 0xD6, 0x48);
	dcs_write_seq(ctx, 0xD7, 0x00);
	dcs_write_seq(ctx, 0xD9, 0x00);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xFF, 0xEE);
	dcs_write_seq(ctx, 0x40, 0x00);
	dcs_write_seq(ctx, 0x41, 0x00);
	dcs_write_seq(ctx, 0x42, 0x00);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0xFF, 0x01);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0x01, 0x55);
	dcs_write_seq(ctx, 0x04, 0x0C);
	dcs_write_seq(ctx, 0x05, 0x3A);
	dcs_write_seq(ctx, 0x06, 0x50);
	dcs_write_seq(ctx, 0x07, 0xD0);
	dcs_write_seq(ctx, 0x0A, 0x0F);
	dcs_write_seq(ctx, 0x0C, 0x06);
	dcs_write_seq(ctx, 0x0D, 0x7F);
	dcs_write_seq(ctx, 0x0E, 0x7F);
	dcs_write_seq(ctx, 0x0F, 0x74);
	dcs_write_seq(ctx, 0x10, 0x63);
	dcs_write_seq(ctx, 0x11, 0x3C);
	dcs_write_seq(ctx, 0x12, 0x5C);
	dcs_write_seq(ctx, 0x13, 0x5A);
	dcs_write_seq(ctx, 0x14, 0x5A);
	dcs_write_seq(ctx, 0x15, 0x60);
	dcs_write_seq(ctx, 0x16, 0x15);
	dcs_write_seq(ctx, 0x17, 0x15);
	dcs_write_seq(ctx, 0x23, 0x00);
	dcs_write_seq(ctx, 0x24, 0x00);
	dcs_write_seq(ctx, 0x25, 0x00);
	dcs_write_seq(ctx, 0x26, 0x00);
	dcs_write_seq(ctx, 0x27, 0x00);
	dcs_write_seq(ctx, 0x28, 0x00);
	dcs_write_seq(ctx, 0x44, 0x00);
	dcs_write_seq(ctx, 0x45, 0x00);
	dcs_write_seq(ctx, 0x46, 0x00);
	dcs_write_seq(ctx, 0x5B, 0xCA);
	dcs_write_seq(ctx, 0x5C, 0x00);
	dcs_write_seq(ctx, 0x5D, 0x00);
	dcs_write_seq(ctx, 0x5E, 0x2D);
	dcs_write_seq(ctx, 0x5F, 0x1B);
	dcs_write_seq(ctx, 0x60, 0xD5);
	dcs_write_seq(ctx, 0x61, 0xF7);
	dcs_write_seq(ctx, 0x6C, 0xAB);
	dcs_write_seq(ctx, 0x6D, 0x44);
	dcs_write_seq(ctx, 0x6E, 0x80);
	dcs_write_seq(ctx, 0xFF, 0x05);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0x00, 0x3F);
	dcs_write_seq(ctx, 0x01, 0x3F);
	dcs_write_seq(ctx, 0x02, 0x3F);
	dcs_write_seq(ctx, 0x03, 0x3F);
	dcs_write_seq(ctx, 0x04, 0x38);
	dcs_write_seq(ctx, 0x05, 0x3F);
	dcs_write_seq(ctx, 0x06, 0x3F);
	dcs_write_seq(ctx, 0x07, 0x19);
	dcs_write_seq(ctx, 0x08, 0x1D);
	dcs_write_seq(ctx, 0x09, 0x3F);
	dcs_write_seq(ctx, 0x0A, 0x3F);
	dcs_write_seq(ctx, 0x0B, 0x1B);
	dcs_write_seq(ctx, 0x0C, 0x17);
	dcs_write_seq(ctx, 0x0D, 0x3F);
	dcs_write_seq(ctx, 0x0E, 0x3F);
	dcs_write_seq(ctx, 0x0F, 0x08);
	dcs_write_seq(ctx, 0x10, 0x3F);
	dcs_write_seq(ctx, 0x11, 0x10);
	dcs_write_seq(ctx, 0x12, 0x3F);
	dcs_write_seq(ctx, 0x13, 0x3F);
	dcs_write_seq(ctx, 0x14, 0x3F);
	dcs_write_seq(ctx, 0x15, 0x3F);
	dcs_write_seq(ctx, 0x16, 0x3F);
	dcs_write_seq(ctx, 0x17, 0x3F);
	dcs_write_seq(ctx, 0x18, 0x38);
	dcs_write_seq(ctx, 0x19, 0x18);
	dcs_write_seq(ctx, 0x1a, 0x1c);
	dcs_write_seq(ctx, 0x1b, 0x3F);
	dcs_write_seq(ctx, 0x1C, 0x3F);
	dcs_write_seq(ctx, 0x1D, 0x1A);
	dcs_write_seq(ctx, 0x1E, 0x16);
	dcs_write_seq(ctx, 0x1F, 0x3F);
	dcs_write_seq(ctx, 0x20, 0x3F);
	dcs_write_seq(ctx, 0x21, 0x3F);
	dcs_write_seq(ctx, 0x22, 0x3F);
	dcs_write_seq(ctx, 0x23, 0x06);
	dcs_write_seq(ctx, 0x24, 0x3F);
	dcs_write_seq(ctx, 0x25, 0x0E);
	dcs_write_seq(ctx, 0x26, 0x3F);
	dcs_write_seq(ctx, 0x27, 0x3F);
	dcs_write_seq(ctx, 0x54, 0x06);
	dcs_write_seq(ctx, 0x55, 0x05);
	dcs_write_seq(ctx, 0x56, 0x04);
	dcs_write_seq(ctx, 0x58, 0x03);
	dcs_write_seq(ctx, 0x59, 0x1B);
	dcs_write_seq(ctx, 0x5A, 0x1B);
	dcs_write_seq(ctx, 0x5B, 0x01);
	dcs_write_seq(ctx, 0x5C, 0x32);
	dcs_write_seq(ctx, 0x5E, 0x18);
	dcs_write_seq(ctx, 0x5F, 0x20);
	dcs_write_seq(ctx, 0x60, 0x2B);
	dcs_write_seq(ctx, 0x61, 0x2C);
	dcs_write_seq(ctx, 0x62, 0x18);
	dcs_write_seq(ctx, 0x63, 0x01);
	dcs_write_seq(ctx, 0x64, 0x32);
	dcs_write_seq(ctx, 0x65, 0x00);
	dcs_write_seq(ctx, 0x66, 0x44);
	dcs_write_seq(ctx, 0x67, 0x11);
	dcs_write_seq(ctx, 0x68, 0x01);
	dcs_write_seq(ctx, 0x69, 0x01);
	dcs_write_seq(ctx, 0x6A, 0x04);
	dcs_write_seq(ctx, 0x6B, 0x2C);
	dcs_write_seq(ctx, 0x6C, 0x08);
	dcs_write_seq(ctx, 0x6D, 0x08);
	dcs_write_seq(ctx, 0x78, 0x00);
	dcs_write_seq(ctx, 0x79, 0x00);
	dcs_write_seq(ctx, 0x7E, 0x00);
	dcs_write_seq(ctx, 0x7F, 0x00);
	dcs_write_seq(ctx, 0x80, 0x00);
	dcs_write_seq(ctx, 0x81, 0x00);
	dcs_write_seq(ctx, 0x8D, 0x00);
	dcs_write_seq(ctx, 0x8E, 0x00);
	dcs_write_seq(ctx, 0x8F, 0xC0);
	dcs_write_seq(ctx, 0x90, 0x73);
	dcs_write_seq(ctx, 0x91, 0x10);
	dcs_write_seq(ctx, 0x92, 0x07);
	dcs_write_seq(ctx, 0x96, 0x11);
	dcs_write_seq(ctx, 0x97, 0x14);
	dcs_write_seq(ctx, 0x98, 0x00);
	dcs_write_seq(ctx, 0x99, 0x00);
	dcs_write_seq(ctx, 0x9A, 0x00);
	dcs_write_seq(ctx, 0x9B, 0x61);
	dcs_write_seq(ctx, 0x9C, 0x15);
	dcs_write_seq(ctx, 0x9D, 0x30);
	dcs_write_seq(ctx, 0x9F, 0x0F);
	dcs_write_seq(ctx, 0xA2, 0xB0);
	dcs_write_seq(ctx, 0xA7, 0x0A);
	dcs_write_seq(ctx, 0xA9, 0x00);
	dcs_write_seq(ctx, 0xAA, 0x70);
	dcs_write_seq(ctx, 0xAB, 0xDA);
	dcs_write_seq(ctx, 0xAC, 0xFF);
	dcs_write_seq(ctx, 0xAE, 0xF4);
	dcs_write_seq(ctx, 0xAF, 0x40);
	dcs_write_seq(ctx, 0xB0, 0x7F);
	dcs_write_seq(ctx, 0xB1, 0x16);
	dcs_write_seq(ctx, 0xB2, 0x53);
	dcs_write_seq(ctx, 0xB3, 0x00);
	dcs_write_seq(ctx, 0xB4, 0x2A);
	dcs_write_seq(ctx, 0xB5, 0x3A);
	dcs_write_seq(ctx, 0xB6, 0xF0);
	dcs_write_seq(ctx, 0xBC, 0x85);
	dcs_write_seq(ctx, 0xBD, 0xF4);
	dcs_write_seq(ctx, 0xBE, 0x33);
	dcs_write_seq(ctx, 0xBF, 0x13);
	dcs_write_seq(ctx, 0xC0, 0x77);
	dcs_write_seq(ctx, 0xC1, 0x77);
	dcs_write_seq(ctx, 0xC2, 0x77);
	dcs_write_seq(ctx, 0xC3, 0x77);
	dcs_write_seq(ctx, 0xC4, 0x77);
	dcs_write_seq(ctx, 0xC5, 0x77);
	dcs_write_seq(ctx, 0xC6, 0x77);
	dcs_write_seq(ctx, 0xC7, 0x77);
	dcs_write_seq(ctx, 0xC8, 0xAA);
	dcs_write_seq(ctx, 0xC9, 0x2A);
	dcs_write_seq(ctx, 0xCA, 0x00);
	dcs_write_seq(ctx, 0xCB, 0xAA);
	dcs_write_seq(ctx, 0xCC, 0x92);
	dcs_write_seq(ctx, 0xCD, 0x00);
	dcs_write_seq(ctx, 0xCE, 0x18);
	dcs_write_seq(ctx, 0xCF, 0x88);
	dcs_write_seq(ctx, 0xD0, 0xAA);
	dcs_write_seq(ctx, 0xD1, 0x00);
	dcs_write_seq(ctx, 0xD2, 0x00);
	dcs_write_seq(ctx, 0xD3, 0x00);
	dcs_write_seq(ctx, 0xD6, 0x02);
	dcs_write_seq(ctx, 0xED, 0x00);
	dcs_write_seq(ctx, 0xEE, 0x00);
	dcs_write_seq(ctx, 0xEF, 0x70);
	dcs_write_seq(ctx, 0xFA, 0x03);
	dcs_write_seq(ctx, 0xFF, 0x00);
	dcs_write_seq(ctx, 0xFF, 0x01);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0x75, 0x00);
	dcs_write_seq(ctx, 0x76, 0x00);
	dcs_write_seq(ctx, 0x77, 0x00);
	dcs_write_seq(ctx, 0x78, 0x2C);
	dcs_write_seq(ctx, 0x79, 0x00);
	dcs_write_seq(ctx, 0x7A, 0x4F);
	dcs_write_seq(ctx, 0x7B, 0x00);
	dcs_write_seq(ctx, 0x7C, 0x69);
	dcs_write_seq(ctx, 0x7D, 0x00);
	dcs_write_seq(ctx, 0x7E, 0x7F);
	dcs_write_seq(ctx, 0x7F, 0x00);
	dcs_write_seq(ctx, 0x80, 0x92);
	dcs_write_seq(ctx, 0x81, 0x00);
	dcs_write_seq(ctx, 0x82, 0xA3);
	dcs_write_seq(ctx, 0x83, 0x00);
	dcs_write_seq(ctx, 0x84, 0xB3);
	dcs_write_seq(ctx, 0x85, 0x00);
	dcs_write_seq(ctx, 0x86, 0xC1);
	dcs_write_seq(ctx, 0x87, 0x00);
	dcs_write_seq(ctx, 0x88, 0xF3);
	dcs_write_seq(ctx, 0x89, 0x01);
	dcs_write_seq(ctx, 0x8A, 0x1B);
	dcs_write_seq(ctx, 0x8B, 0x01);
	dcs_write_seq(ctx, 0x8C, 0x5A);
	dcs_write_seq(ctx, 0x8D, 0x01);
	dcs_write_seq(ctx, 0x8E, 0x8B);
	dcs_write_seq(ctx, 0x8F, 0x01);
	dcs_write_seq(ctx, 0x90, 0xD9);
	dcs_write_seq(ctx, 0x91, 0x02);
	dcs_write_seq(ctx, 0x92, 0x16);
	dcs_write_seq(ctx, 0x93, 0x02);
	dcs_write_seq(ctx, 0x94, 0x18);
	dcs_write_seq(ctx, 0x95, 0x02);
	dcs_write_seq(ctx, 0x96, 0x4E);
	dcs_write_seq(ctx, 0x97, 0x02);
	dcs_write_seq(ctx, 0x98, 0x88);
	dcs_write_seq(ctx, 0x99, 0x02);
	dcs_write_seq(ctx, 0x9A, 0xAC);
	dcs_write_seq(ctx, 0x9B, 0x02);
	dcs_write_seq(ctx, 0x9C, 0xDD);
	dcs_write_seq(ctx, 0x9D, 0x03);
	dcs_write_seq(ctx, 0x9E, 0x01);
	dcs_write_seq(ctx, 0x9F, 0x03);
	dcs_write_seq(ctx, 0xA0, 0x2E);
	dcs_write_seq(ctx, 0xA2, 0x03);
	dcs_write_seq(ctx, 0xA3, 0x3C);
	dcs_write_seq(ctx, 0xA4, 0x03);
	dcs_write_seq(ctx, 0xA5, 0x4C);
	dcs_write_seq(ctx, 0xA6, 0x03);
	dcs_write_seq(ctx, 0xA7, 0x5D);
	dcs_write_seq(ctx, 0xA9, 0x03);
	dcs_write_seq(ctx, 0xAA, 0x70);
	dcs_write_seq(ctx, 0xAB, 0x03);
	dcs_write_seq(ctx, 0xAC, 0x88);
	dcs_write_seq(ctx, 0xAD, 0x03);
	dcs_write_seq(ctx, 0xAE, 0xA8);
	dcs_write_seq(ctx, 0xAF, 0x03);
	dcs_write_seq(ctx, 0xB0, 0xC8);
	dcs_write_seq(ctx, 0xB1, 0x03);
	dcs_write_seq(ctx, 0xB2, 0xFF);
	dcs_write_seq(ctx, 0xB3, 0x00);
	dcs_write_seq(ctx, 0xB4, 0x00);
	dcs_write_seq(ctx, 0xB5, 0x00);
	dcs_write_seq(ctx, 0xB6, 0x2C);
	dcs_write_seq(ctx, 0xB7, 0x00);
	dcs_write_seq(ctx, 0xB8, 0x4F);
	dcs_write_seq(ctx, 0xB9, 0x00);
	dcs_write_seq(ctx, 0xBA, 0x69);
	dcs_write_seq(ctx, 0xBB, 0x00);
	dcs_write_seq(ctx, 0xBC, 0x7F);
	dcs_write_seq(ctx, 0xBD, 0x00);
	dcs_write_seq(ctx, 0xBE, 0x92);
	dcs_write_seq(ctx, 0xBF, 0x00);
	dcs_write_seq(ctx, 0xC0, 0xA3);
	dcs_write_seq(ctx, 0xC1, 0x00);
	dcs_write_seq(ctx, 0xC2, 0xB3);
	dcs_write_seq(ctx, 0xC3, 0x00);
	dcs_write_seq(ctx, 0xC4, 0xC1);
	dcs_write_seq(ctx, 0xC5, 0x00);
	dcs_write_seq(ctx, 0xC6, 0xF3);
	dcs_write_seq(ctx, 0xC7, 0x01);
	dcs_write_seq(ctx, 0xC8, 0x1B);
	dcs_write_seq(ctx, 0xC9, 0x01);
	dcs_write_seq(ctx, 0xCA, 0x5A);
	dcs_write_seq(ctx, 0xCB, 0x01);
	dcs_write_seq(ctx, 0xCC, 0x8B);
	dcs_write_seq(ctx, 0xCD, 0x01);
	dcs_write_seq(ctx, 0xCE, 0xD9);
	dcs_write_seq(ctx, 0xCF, 0x02);
	dcs_write_seq(ctx, 0xD0, 0x16);
	dcs_write_seq(ctx, 0xD1, 0x02);
	dcs_write_seq(ctx, 0xD2, 0x18);
	dcs_write_seq(ctx, 0xD3, 0x02);
	dcs_write_seq(ctx, 0xD4, 0x4E);
	dcs_write_seq(ctx, 0xD5, 0x02);
	dcs_write_seq(ctx, 0xD6, 0x88);
	dcs_write_seq(ctx, 0xD7, 0x02);
	dcs_write_seq(ctx, 0xD8, 0xAC);
	dcs_write_seq(ctx, 0xD9, 0x02);
	dcs_write_seq(ctx, 0xDA, 0xDD);
	dcs_write_seq(ctx, 0xDB, 0x03);
	dcs_write_seq(ctx, 0xDC, 0x01);
	dcs_write_seq(ctx, 0xDD, 0x03);
	dcs_write_seq(ctx, 0xDE, 0x2E);
	dcs_write_seq(ctx, 0xDF, 0x03);
	dcs_write_seq(ctx, 0xE0, 0x3C);
	dcs_write_seq(ctx, 0xE1, 0x03);
	dcs_write_seq(ctx, 0xE2, 0x4C);
	dcs_write_seq(ctx, 0xE3, 0x03);
	dcs_write_seq(ctx, 0xE4, 0x5D);
	dcs_write_seq(ctx, 0xE5, 0x03);
	dcs_write_seq(ctx, 0xE6, 0x70);
	dcs_write_seq(ctx, 0xE7, 0x03);
	dcs_write_seq(ctx, 0xE8, 0x88);
	dcs_write_seq(ctx, 0xE9, 0x03);
	dcs_write_seq(ctx, 0xEA, 0xA8);
	dcs_write_seq(ctx, 0xEB, 0x03);
	dcs_write_seq(ctx, 0xEC, 0xC8);
	dcs_write_seq(ctx, 0xED, 0x03);
	dcs_write_seq(ctx, 0xEE, 0xFF);
	dcs_write_seq(ctx, 0xEF, 0x00);
	dcs_write_seq(ctx, 0xF0, 0x00);
	dcs_write_seq(ctx, 0xF1, 0x00);
	dcs_write_seq(ctx, 0xF2, 0x2C);
	dcs_write_seq(ctx, 0xF3, 0x00);
	dcs_write_seq(ctx, 0xF4, 0x4F);
	dcs_write_seq(ctx, 0xF5, 0x00);
	dcs_write_seq(ctx, 0xF6, 0x69);
	dcs_write_seq(ctx, 0xF7, 0x00);
	dcs_write_seq(ctx, 0xF8, 0x7F);
	dcs_write_seq(ctx, 0xF9, 0x00);
	dcs_write_seq(ctx, 0xFA, 0x92);
	dcs_write_seq(ctx, 0xFF, 0x02);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0x00, 0x00);
	dcs_write_seq(ctx, 0x01, 0xA3);
	dcs_write_seq(ctx, 0x02, 0x00);
	dcs_write_seq(ctx, 0x03, 0xB3);
	dcs_write_seq(ctx, 0x04, 0x00);
	dcs_write_seq(ctx, 0x05, 0xC1);
	dcs_write_seq(ctx, 0x06, 0x00);
	dcs_write_seq(ctx, 0x07, 0xF3);
	dcs_write_seq(ctx, 0x08, 0x01);
	dcs_write_seq(ctx, 0x09, 0x1B);
	dcs_write_seq(ctx, 0x0A, 0x01);
	dcs_write_seq(ctx, 0x0B, 0x5A);
	dcs_write_seq(ctx, 0x0C, 0x01);
	dcs_write_seq(ctx, 0x0D, 0x8B);
	dcs_write_seq(ctx, 0x0E, 0x01);
	dcs_write_seq(ctx, 0x0F, 0xD9);
	dcs_write_seq(ctx, 0x10, 0x02);
	dcs_write_seq(ctx, 0x11, 0x16);
	dcs_write_seq(ctx, 0x12, 0x02);
	dcs_write_seq(ctx, 0x13, 0x18);
	dcs_write_seq(ctx, 0x14, 0x02);
	dcs_write_seq(ctx, 0x15, 0x4E);
	dcs_write_seq(ctx, 0x16, 0x02);
	dcs_write_seq(ctx, 0x17, 0x88);
	dcs_write_seq(ctx, 0x18, 0x02);
	dcs_write_seq(ctx, 0x19, 0xAC);
	dcs_write_seq(ctx, 0x1A, 0x02);
	dcs_write_seq(ctx, 0x1B, 0xDD);
	dcs_write_seq(ctx, 0x1C, 0x03);
	dcs_write_seq(ctx, 0x1D, 0x01);
	dcs_write_seq(ctx, 0x1E, 0x03);
	dcs_write_seq(ctx, 0x1F, 0x2E);
	dcs_write_seq(ctx, 0x20, 0x03);
	dcs_write_seq(ctx, 0x21, 0x3C);
	dcs_write_seq(ctx, 0x22, 0x03);
	dcs_write_seq(ctx, 0x23, 0x4C);
	dcs_write_seq(ctx, 0x24, 0x03);
	dcs_write_seq(ctx, 0x25, 0x5D);
	dcs_write_seq(ctx, 0x26, 0x03);
	dcs_write_seq(ctx, 0x27, 0x70);
	dcs_write_seq(ctx, 0x28, 0x03);
	dcs_write_seq(ctx, 0x29, 0x88);
	dcs_write_seq(ctx, 0x2A, 0x03);
	dcs_write_seq(ctx, 0x2B, 0xA8);
	dcs_write_seq(ctx, 0x2D, 0x03);
	dcs_write_seq(ctx, 0x2F, 0xC8);
	dcs_write_seq(ctx, 0x30, 0x03);
	dcs_write_seq(ctx, 0x31, 0xFF);
	dcs_write_seq(ctx, 0x32, 0x00);
	dcs_write_seq(ctx, 0x33, 0x00);
	dcs_write_seq(ctx, 0x34, 0x00);
	dcs_write_seq(ctx, 0x35, 0x2C);
	dcs_write_seq(ctx, 0x36, 0x00);
	dcs_write_seq(ctx, 0x37, 0x4F);
	dcs_write_seq(ctx, 0x38, 0x00);
	dcs_write_seq(ctx, 0x39, 0x69);
	dcs_write_seq(ctx, 0x3A, 0x00);
	dcs_write_seq(ctx, 0x3B, 0x7F);
	dcs_write_seq(ctx, 0x3D, 0x00);
	dcs_write_seq(ctx, 0x3F, 0x92);
	dcs_write_seq(ctx, 0x40, 0x00);
	dcs_write_seq(ctx, 0x41, 0xA3);
	dcs_write_seq(ctx, 0x42, 0x00);
	dcs_write_seq(ctx, 0x43, 0xB3);
	dcs_write_seq(ctx, 0x44, 0x00);
	dcs_write_seq(ctx, 0x45, 0xC1);
	dcs_write_seq(ctx, 0x46, 0x00);
	dcs_write_seq(ctx, 0x47, 0xF3);
	dcs_write_seq(ctx, 0x48, 0x01);
	dcs_write_seq(ctx, 0x49, 0x1B);
	dcs_write_seq(ctx, 0x4A, 0x01);
	dcs_write_seq(ctx, 0x4B, 0x5A);
	dcs_write_seq(ctx, 0x4C, 0x01);
	dcs_write_seq(ctx, 0x4D, 0x8B);
	dcs_write_seq(ctx, 0x4E, 0x01);
	dcs_write_seq(ctx, 0x4F, 0xD9);
	dcs_write_seq(ctx, 0x50, 0x02);
	dcs_write_seq(ctx, 0x51, 0x16);
	dcs_write_seq(ctx, 0x52, 0x02);
	dcs_write_seq(ctx, 0x53, 0x18);
	dcs_write_seq(ctx, 0x54, 0x02);
	dcs_write_seq(ctx, 0x55, 0x4E);
	dcs_write_seq(ctx, 0x56, 0x02);
	dcs_write_seq(ctx, 0x58, 0x88);
	dcs_write_seq(ctx, 0x59, 0x02);
	dcs_write_seq(ctx, 0x5A, 0xAC);
	dcs_write_seq(ctx, 0x5B, 0x02);
	dcs_write_seq(ctx, 0x5C, 0xDD);
	dcs_write_seq(ctx, 0x5D, 0x03);
	dcs_write_seq(ctx, 0x5E, 0x01);
	dcs_write_seq(ctx, 0x5F, 0x03);
	dcs_write_seq(ctx, 0x60, 0x2E);
	dcs_write_seq(ctx, 0x61, 0x03);
	dcs_write_seq(ctx, 0x62, 0x3C);
	dcs_write_seq(ctx, 0x63, 0x03);
	dcs_write_seq(ctx, 0x64, 0x4C);
	dcs_write_seq(ctx, 0x65, 0x03);
	dcs_write_seq(ctx, 0x66, 0x5D);
	dcs_write_seq(ctx, 0x67, 0x03);
	dcs_write_seq(ctx, 0x68, 0x70);
	dcs_write_seq(ctx, 0x69, 0x03);
	dcs_write_seq(ctx, 0x6A, 0x88);
	dcs_write_seq(ctx, 0x6B, 0x03);
	dcs_write_seq(ctx, 0x6C, 0xA8);
	dcs_write_seq(ctx, 0x6D, 0x03);
	dcs_write_seq(ctx, 0x6E, 0xC8);
	dcs_write_seq(ctx, 0x6F, 0x03);
	dcs_write_seq(ctx, 0x70, 0xFF);
	dcs_write_seq(ctx, 0x71, 0x00);
	dcs_write_seq(ctx, 0x72, 0x00);
	dcs_write_seq(ctx, 0x73, 0x00);
	dcs_write_seq(ctx, 0x74, 0x2C);
	dcs_write_seq(ctx, 0x75, 0x00);
	dcs_write_seq(ctx, 0x76, 0x4F);
	dcs_write_seq(ctx, 0x77, 0x00);
	dcs_write_seq(ctx, 0x78, 0x69);
	dcs_write_seq(ctx, 0x79, 0x00);
	dcs_write_seq(ctx, 0x7A, 0x7F);
	dcs_write_seq(ctx, 0x7B, 0x00);
	dcs_write_seq(ctx, 0x7C, 0x92);
	dcs_write_seq(ctx, 0x7D, 0x00);
	dcs_write_seq(ctx, 0x7E, 0xA3);
	dcs_write_seq(ctx, 0x7F, 0x00);
	dcs_write_seq(ctx, 0x80, 0xB3);
	dcs_write_seq(ctx, 0x81, 0x00);
	dcs_write_seq(ctx, 0x82, 0xC1);
	dcs_write_seq(ctx, 0x83, 0x00);
	dcs_write_seq(ctx, 0x84, 0xF3);
	dcs_write_seq(ctx, 0x85, 0x01);
	dcs_write_seq(ctx, 0x86, 0x1B);
	dcs_write_seq(ctx, 0x87, 0x01);
	dcs_write_seq(ctx, 0x88, 0x5A);
	dcs_write_seq(ctx, 0x89, 0x01);
	dcs_write_seq(ctx, 0x8A, 0x8B);
	dcs_write_seq(ctx, 0x8B, 0x01);
	dcs_write_seq(ctx, 0x8C, 0xD9);
	dcs_write_seq(ctx, 0x8D, 0x02);
	dcs_write_seq(ctx, 0x8E, 0x16);
	dcs_write_seq(ctx, 0x8F, 0x02);
	dcs_write_seq(ctx, 0x90, 0x18);
	dcs_write_seq(ctx, 0x91, 0x02);
	dcs_write_seq(ctx, 0x92, 0x4E);
	dcs_write_seq(ctx, 0x93, 0x02);
	dcs_write_seq(ctx, 0x94, 0x88);
	dcs_write_seq(ctx, 0x95, 0x02);
	dcs_write_seq(ctx, 0x96, 0xAC);
	dcs_write_seq(ctx, 0x97, 0x02);
	dcs_write_seq(ctx, 0x98, 0xDD);
	dcs_write_seq(ctx, 0x99, 0x03);
	dcs_write_seq(ctx, 0x9A, 0x01);
	dcs_write_seq(ctx, 0x9B, 0x03);
	dcs_write_seq(ctx, 0x9C, 0x2E);
	dcs_write_seq(ctx, 0x9D, 0x03);
	dcs_write_seq(ctx, 0x9E, 0x3C);
	dcs_write_seq(ctx, 0x9F, 0x03);
	dcs_write_seq(ctx, 0xA0, 0x4C);
	dcs_write_seq(ctx, 0xA2, 0x03);
	dcs_write_seq(ctx, 0xA3, 0x5D);
	dcs_write_seq(ctx, 0xA4, 0x03);
	dcs_write_seq(ctx, 0xA5, 0x70);
	dcs_write_seq(ctx, 0xA6, 0x03);
	dcs_write_seq(ctx, 0xA7, 0x88);
	dcs_write_seq(ctx, 0xA9, 0x03);
	dcs_write_seq(ctx, 0xAA, 0xA8);
	dcs_write_seq(ctx, 0xAB, 0x03);
	dcs_write_seq(ctx, 0xAC, 0xC8);
	dcs_write_seq(ctx, 0xAD, 0x03);
	dcs_write_seq(ctx, 0xAE, 0xFF);
	dcs_write_seq(ctx, 0xAF, 0x00);
	dcs_write_seq(ctx, 0xB0, 0x00);
	dcs_write_seq(ctx, 0xB1, 0x00);
	dcs_write_seq(ctx, 0xB2, 0x2C);
	dcs_write_seq(ctx, 0xB3, 0x00);
	dcs_write_seq(ctx, 0xB4, 0x4F);
	dcs_write_seq(ctx, 0xB5, 0x00);
	dcs_write_seq(ctx, 0xB6, 0x69);
	dcs_write_seq(ctx, 0xB7, 0x00);
	dcs_write_seq(ctx, 0xB8, 0x7F);
	dcs_write_seq(ctx, 0xB9, 0x00);
	dcs_write_seq(ctx, 0xBA, 0x92);
	dcs_write_seq(ctx, 0xBB, 0x00);
	dcs_write_seq(ctx, 0xBC, 0xA3);
	dcs_write_seq(ctx, 0xBD, 0x00);
	dcs_write_seq(ctx, 0xBE, 0xB3);
	dcs_write_seq(ctx, 0xBF, 0x00);
	dcs_write_seq(ctx, 0xC0, 0xC1);
	dcs_write_seq(ctx, 0xC1, 0x00);
	dcs_write_seq(ctx, 0xC2, 0xF3);
	dcs_write_seq(ctx, 0xC3, 0x01);
	dcs_write_seq(ctx, 0xC4, 0x1B);
	dcs_write_seq(ctx, 0xC5, 0x01);
	dcs_write_seq(ctx, 0xC6, 0x5A);
	dcs_write_seq(ctx, 0xC7, 0x01);
	dcs_write_seq(ctx, 0xC8, 0x8B);
	dcs_write_seq(ctx, 0xC9, 0x01);
	dcs_write_seq(ctx, 0xCA, 0xD9);
	dcs_write_seq(ctx, 0xCB, 0x02);
	dcs_write_seq(ctx, 0xCC, 0x16);
	dcs_write_seq(ctx, 0xCD, 0x02);
	dcs_write_seq(ctx, 0xCE, 0x18);
	dcs_write_seq(ctx, 0xCF, 0x02);
	dcs_write_seq(ctx, 0xD0, 0x4E);
	dcs_write_seq(ctx, 0xD1, 0x02);
	dcs_write_seq(ctx, 0xD2, 0x88);
	dcs_write_seq(ctx, 0xD3, 0x02);
	dcs_write_seq(ctx, 0xD4, 0xAC);
	dcs_write_seq(ctx, 0xD5, 0x02);
	dcs_write_seq(ctx, 0xD6, 0xDD);
	dcs_write_seq(ctx, 0xD7, 0x03);
	dcs_write_seq(ctx, 0xD8, 0x01);
	dcs_write_seq(ctx, 0xD9, 0x03);
	dcs_write_seq(ctx, 0xDA, 0x2E);
	dcs_write_seq(ctx, 0xDB, 0x03);
	dcs_write_seq(ctx, 0xDC, 0x3C);
	dcs_write_seq(ctx, 0xDD, 0x03);
	dcs_write_seq(ctx, 0xDE, 0x4C);
	dcs_write_seq(ctx, 0xDF, 0x03);
	dcs_write_seq(ctx, 0xE0, 0x5D);
	dcs_write_seq(ctx, 0xE1, 0x03);
	dcs_write_seq(ctx, 0xE2, 0x70);
	dcs_write_seq(ctx, 0xE3, 0x03);
	dcs_write_seq(ctx, 0xE4, 0x88);
	dcs_write_seq(ctx, 0xE5, 0x03);
	dcs_write_seq(ctx, 0xE6, 0xA8);
	dcs_write_seq(ctx, 0xE7, 0x03);
	dcs_write_seq(ctx, 0xE8, 0xC8);
	dcs_write_seq(ctx, 0xE9, 0x03);
	dcs_write_seq(ctx, 0xEA, 0xFF);
	dcs_write_seq(ctx, 0xFF, 0x00);
	dcs_write_seq(ctx, 0xFB, 0x01);
	dcs_write_seq(ctx, 0x11, 0x00);
	mdelay(120);
	dcs_write_seq(ctx, 0x29, 0x00);
	mdelay(20);

	return 0;
}

static int frd55_disable(struct drm_panel *panel)
{
	struct frd55 *ctx = panel_to_frd55(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->enabled)
		return 0; /* This is not an issue so we return 0 here */

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(120);

	ctx->enabled = false;

	return 0;
}

static int frd55_unprepare(struct drm_panel *panel)
{
	struct frd55 *ctx = panel_to_frd55(panel);

	if (!ctx->prepared)
		return 0;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
	}

	ctx->prepared = false;

	return 0;
}

static int frd55_prepare(struct drm_panel *panel)
{
	struct frd55 *ctx = panel_to_frd55(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(100);
	}

	ret = frd55_init_sequence(ctx);
	if (ret)
		return ret;

	ctx->prepared = true;

	return 0;
}

static int frd55_enable(struct drm_panel *panel)
{
	struct frd55 *ctx = panel_to_frd55(panel);

	ctx->enabled = true;

	return 0;
}

static int frd55_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	int ret;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux@%u\n",
			  default_mode.hdisplay, default_mode.vdisplay,
			  default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	panel->connector->display_info.width_mm = mode->width_mm;
	panel->connector->display_info.height_mm = mode->height_mm;

	ret = drm_display_info_set_bus_formats(&panel->connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	drm_mode_probed_add(panel->connector, mode);

	return 1;
}

static const struct drm_panel_funcs frd55_drm_funcs = {
	.disable   = frd55_disable,
	.unprepare = frd55_unprepare,
	.prepare   = frd55_prepare,
	.enable    = frd55_enable,
	.get_modes = frd55_get_modes,
};

static int frd55_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct frd55 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpio\n");
		return PTR_ERR(ctx->reset_gpio);
	}

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &frd55_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach failed. Is host ready?\n");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	DRM_INFO(DRV_NAME "_panel %ux%u@%u %ubpp dsi %udl - ready\n",
		 default_mode.hdisplay, default_mode.vdisplay,
		 default_mode.vrefresh,
		 mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes);

	return 0;
}

static int frd55_remove(struct mipi_dsi_device *dsi)
{
	struct frd55 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id frd55_of_match[] = {
	{ .compatible = "emcraft,frd55" },
	{ }
};
MODULE_DEVICE_TABLE(of, frd55_of_match);

static struct mipi_dsi_driver frd55_driver = {
	.probe  = frd55_probe,
	.remove = frd55_remove,
	.driver = {
		.name = DRV_NAME "_panel",
		.of_match_table = frd55_of_match,
	},
};
module_mipi_dsi_driver(frd55_driver);

MODULE_AUTHOR("Dmitry Konyshev <probables@emcraft.com>");
MODULE_DESCRIPTION("DRM driver for FRD55 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
