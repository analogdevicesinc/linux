// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics SA 2017
 *
 * Authors: Philippe Cornu <philippe.cornu@st.com>
 *          Yannick Fertre <yannick.fertre@st.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/*** Manufacturer Command Set ***/
#define MCS_CMD_MODE_SW		0xfe /* CMD Mode Switch */
#define MCS_CMD1_UCS		0x00 /* User Command Set (UCS = CMD1) */
#define MCS_CMD2_P0		0x01 /* Manufacture Command Set Page0 (CMD2 P0) */
#define MCS_CMD2_P1		0x02 /* Manufacture Command Set Page1 (CMD2 P1) */
#define MCS_CMD2_P2		0x03 /* Manufacture Command Set Page2 (CMD2 P2) */
#define MCS_CMD2_P3		0x04 /* Manufacture Command Set Page3 (CMD2 P3) */

/* CMD2 P0 commands (Display Options and Power) */
#define MCS_STBCTR		0x12 /* TE1 Output Setting Zig-Zag Connection */
#define MCS_SGOPCTR		0x16 /* Source Bias Current */
#define MCS_SDCTR		0x1a /* Source Output Delay Time */
#define MCS_INVCTR		0x1b /* Inversion Type */
#define MCS_EXT_PWR_IC		0x24 /* External PWR IC Control */
#define MCS_SETAVDD		0x27 /* PFM Control for AVDD Output */
#define MCS_SETAVEE		0x29 /* PFM Control for AVEE Output */
#define MCS_BT2CTR		0x2b /* DDVDL Charge Pump Control */
#define MCS_BT3CTR		0x2f /* VGH Charge Pump Control */
#define MCS_BT4CTR		0x34 /* VGL Charge Pump Control */
#define MCS_VCMCTR		0x46 /* VCOM Output Level Control */
#define MCS_SETVGN		0x52 /* VG M/S N Control */
#define MCS_SETVGP		0x54 /* VG M/S P Control */
#define MCS_SW_CTRL		0x5f /* Interface Control for PFM and MIPI */

/* CMD2 P2 commands (GOA Timing Control) - no description in datasheet */
#define GOA_VSTV1		0x00
#define GOA_VSTV2		0x07
#define GOA_VCLK1		0x0e
#define GOA_VCLK2		0x17
#define GOA_VCLK_OPT1		0x20
#define GOA_BICLK1		0x2a
#define GOA_BICLK2		0x37
#define GOA_BICLK3		0x44
#define GOA_BICLK4		0x4f
#define GOA_BICLK_OPT1		0x5b
#define GOA_BICLK_OPT2		0x60
#define MCS_GOA_GPO1		0x6d
#define MCS_GOA_GPO2		0x71
#define MCS_GOA_EQ		0x74
#define MCS_GOA_CLK_GALLON	0x7c
#define MCS_GOA_FS_SEL0		0x7e
#define MCS_GOA_FS_SEL1		0x87
#define MCS_GOA_FS_SEL2		0x91
#define MCS_GOA_FS_SEL3		0x9b
#define MCS_GOA_BS_SEL0		0xac
#define MCS_GOA_BS_SEL1		0xb5
#define MCS_GOA_BS_SEL2		0xbf
#define MCS_GOA_BS_SEL3		0xc9
#define MCS_GOA_BS_SEL4		0xd3

/* CMD2 P3 commands (Gamma) */
#define MCS_GAMMA_VP		0x60 /* Gamma VP1~VP16 */
#define MCS_GAMMA_VN		0x70 /* Gamma VN1~VN16 */

struct rm68200 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *supply;
};

static const struct drm_display_mode default_mode = {
	.clock = 54000,
	.hdisplay = 720,
	.hsync_start = 720 + 48,
	.hsync_end = 720 + 48 + 9,
	.htotal = 720 + 48 + 9 + 48,
	.vdisplay = 1280,
	.vsync_start = 1280 + 12,
	.vsync_end = 1280 + 12 + 5,
	.vtotal = 1280 + 12 + 5 + 12,
	.flags = 0,
	.width_mm = 68,
	.height_mm = 122,
};

static inline struct rm68200 *panel_to_rm68200(struct drm_panel *panel)
{
	return container_of(panel, struct rm68200, panel);
}

/*
 * This panel is not able to auto-increment all cmd addresses so for some of
 * them, we need to send them one by one...
 *
 * Use arr instead of d to avoid shadowing the local d[] declared by
 * mipi_dsi_dcs_write_var_seq_multi().
 */
#define dcs_write_cmd_seq(dsi_ctx, cmd, seq...)			\
	do {								\
		static const u8 arr[] = { seq };			\
		unsigned int i;						\
									\
		for (i = 0; i < ARRAY_SIZE(arr); i++)		\
			mipi_dsi_dcs_write_var_seq_multi(dsi_ctx,	\
							(cmd) + i, arr[i]); \
	} while (0)

static void rm68200_init_sequence(struct mipi_dsi_multi_context *dsi_ctx)
{
	/* Enter CMD2 with page 0 */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_CMD_MODE_SW, MCS_CMD2_P0);
	dcs_write_cmd_seq(dsi_ctx, MCS_EXT_PWR_IC, 0xc0, 0x53, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_BT2CTR, 0xe5);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_SETAVDD, 0x0a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_SETAVEE, 0x0a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_SGOPCTR, 0x52);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_BT3CTR, 0x53);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_BT4CTR, 0x5a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_INVCTR, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_STBCTR, 0x0a);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_SDCTR, 0x06);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_VCMCTR, 0x56);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_SETVGN, 0xa0, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_SETVGP, 0xa0, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_SW_CTRL, 0x11); /* 2 data lanes, see doc */

	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_CMD_MODE_SW, MCS_CMD2_P2);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, GOA_VSTV1, 0x05);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x02, 0x0b);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x03, 0x0f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x04, 0x7d, 0x00, 0x50);
	dcs_write_cmd_seq(dsi_ctx, GOA_VSTV2, 0x05, 0x16, 0x0d, 0x11, 0x7d, 0x00,
			  0x50);
	dcs_write_cmd_seq(dsi_ctx, GOA_VCLK1, 0x07, 0x08, 0x01, 0x02, 0x00, 0x7d,
			  0x00, 0x85, 0x08);
	dcs_write_cmd_seq(dsi_ctx, GOA_VCLK2, 0x03, 0x04, 0x05, 0x06, 0x00, 0x7d,
			  0x00, 0x85, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, GOA_VCLK_OPT1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00);
	dcs_write_cmd_seq(dsi_ctx, GOA_BICLK1, 0x07, 0x08);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2d, 0x01);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x2f, 0x02, 0x00, 0x40, 0x05, 0x08, 0x54, 0x7d,
				     0x00);
	dcs_write_cmd_seq(dsi_ctx, GOA_BICLK2, 0x03, 0x04, 0x05, 0x06, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3d, 0x40);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x3f, 0x05, 0x08, 0x54, 0x7d, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, GOA_BICLK3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, GOA_BICLK4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x58, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, GOA_BICLK_OPT1, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, GOA_BICLK_OPT2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_GOA_GPO1, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_GOA_GPO2, 0x00, 0x20, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_GOA_EQ, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
				     0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_GOA_CLK_GALLON, 0x00, 0x00);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_FS_SEL0, 0xbf, 0x02, 0x06, 0x14, 0x10,
			  0x16, 0x12, 0x08, 0x3f);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_FS_SEL1, 0x3f, 0x3f, 0x3f, 0x3f, 0x0c,
			  0x0a, 0x0e, 0x3f, 0x3f, 0x00);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_FS_SEL2, 0x04, 0x3f, 0x3f, 0x3f, 0x3f,
			  0x05, 0x01, 0x3f, 0x3f, 0x0f);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_FS_SEL3, 0x0b, 0x0d, 0x3f, 0x3f, 0x3f,
			  0x3f);
	dcs_write_cmd_seq(dsi_ctx, 0xa2, 0x3f, 0x09, 0x13, 0x17, 0x11, 0x15);
	dcs_write_cmd_seq(dsi_ctx, 0xa9, 0x07, 0x03, 0x3f);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_BS_SEL0, 0x3f, 0x05, 0x01, 0x17, 0x13,
			  0x15, 0x11, 0x0f, 0x3f);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_BS_SEL1, 0x3f, 0x3f, 0x3f, 0x3f, 0x0b,
			  0x0d, 0x09, 0x3f, 0x3f, 0x07);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_BS_SEL2, 0x03, 0x3f, 0x3f, 0x3f, 0x3f,
			  0x02, 0x06, 0x3f, 0x3f, 0x08);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_BS_SEL3, 0x0c, 0x0a, 0x3f, 0x3f, 0x3f,
			  0x3f, 0x3f, 0x0e, 0x10, 0x14);
	dcs_write_cmd_seq(dsi_ctx, MCS_GOA_BS_SEL4, 0x12, 0x16, 0x00, 0x04, 0x3f);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xdc, 0x02);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xde, 0x12);

	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_CMD_MODE_SW, 0x0e); /* No documentation */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x01, 0x75);

	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_CMD_MODE_SW, MCS_CMD2_P3);
	dcs_write_cmd_seq(dsi_ctx, MCS_GAMMA_VP, 0x00, 0x0c, 0x12, 0x0e, 0x06,
			  0x12, 0x0e, 0x0b, 0x15, 0x0b, 0x10, 0x07, 0x0f,
			  0x12, 0x0c, 0x00);
	dcs_write_cmd_seq(dsi_ctx, MCS_GAMMA_VN, 0x00, 0x0c, 0x12, 0x0e, 0x06,
			  0x12, 0x0e, 0x0b, 0x15, 0x0b, 0x10, 0x07, 0x0f,
			  0x12, 0x0c, 0x00);

	/* Exit CMD2 */
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, MCS_CMD_MODE_SW, MCS_CMD1_UCS);
}

static int rm68200_unprepare(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);

	/* Attempt to enter sleep mode even if display off failed. */
	dsi_ctx.accum_err = 0;
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);

	msleep(120);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
	}

	regulator_disable(ctx->supply);

	return 0;
}

static int rm68200_prepare(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable supply: %d\n", ret);
		return ret;
	}

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(100);
	}

	rm68200_init_sequence(&dsi_ctx);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 125);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	ret = dsi_ctx.accum_err;
	if (ret)
		goto assert_reset;

	mipi_dsi_msleep(&dsi_ctx, 20);

	return 0;

assert_reset:
	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
	}
	/* Balance the regulator enable count on prepare failure. */
	regulator_disable(ctx->supply);
	return ret;
}

static int rm68200_get_modes(struct drm_panel *panel,
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

static const struct drm_panel_funcs rm68200_drm_funcs = {
	.unprepare = rm68200_unprepare,
	.prepare = rm68200_prepare,
	.get_modes = rm68200_get_modes,
};

static int rm68200_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rm68200 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct rm68200, panel,
				   &rm68200_drm_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		dev_err(dev, "cannot get reset GPIO: %d\n", ret);
		return ret;
	}

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "cannot get regulator: %d\n", ret);
		return ret;
	}

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	ret = devm_drm_panel_add(dev, &ctx->panel);
	if (ret)
		return ret;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach() failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id raydium_rm68200_of_match[] = {
	{ .compatible = "raydium,rm68200" },
	{ }
};
MODULE_DEVICE_TABLE(of, raydium_rm68200_of_match);

static struct mipi_dsi_driver raydium_rm68200_driver = {
	.probe = rm68200_probe,
	.driver = {
		.name = "panel-raydium-rm68200",
		.of_match_table = raydium_rm68200_of_match,
	},
};
module_mipi_dsi_driver(raydium_rm68200_driver);

MODULE_AUTHOR("Philippe Cornu <philippe.cornu@st.com>");
MODULE_AUTHOR("Yannick Fertre <yannick.fertre@st.com>");
MODULE_DESCRIPTION("DRM Driver for Raydium RM68200 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
