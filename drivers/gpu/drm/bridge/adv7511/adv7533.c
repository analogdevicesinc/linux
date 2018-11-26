// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 */

#include <linux/of_graph.h>

#include "adv7511.h"

static const struct reg_sequence adv7533_fixed_registers[] = {
	{ 0x16, 0x20 },
	{ 0x9a, 0xe0 },
	{ 0xba, 0x70 },
	{ 0xde, 0x82 },
	{ 0xe4, 0x40 },
	{ 0xe5, 0x80 },
};

static const struct reg_sequence adv7533_cec_fixed_registers[] = {
	{ 0x15, 0xd0 },
	{ 0x17, 0xd0 },
	{ 0x24, 0x20 },
	{ 0x57, 0x11 },
	{ 0x05, 0xc8 },
};

static void adv7511_dsi_config_timing_gen(struct adv7511 *adv)
{
	struct drm_display_mode *mode = &adv->curr_mode;
	unsigned int hsw, hfp, hbp, vsw, vfp, vbp;

	hsw = mode->hsync_end - mode->hsync_start;
	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	vsw = mode->vsync_end - mode->vsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	/* 03-01 Enable Internal Timing Generator */
	regmap_write(adv->regmap_cec, 0x27, 0xcb);

	/* 03-08 Timing Configuration */

	/* horizontal porch params */
	regmap_write(adv->regmap_cec, 0x28, mode->htotal >> 4);
	regmap_write(adv->regmap_cec, 0x29, (mode->htotal << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x2a, hsw >> 4);
	regmap_write(adv->regmap_cec, 0x2b, (hsw << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x2c, hfp >> 4);
	regmap_write(adv->regmap_cec, 0x2d, (hfp << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x2e, hbp >> 4);
	regmap_write(adv->regmap_cec, 0x2f, (hbp << 4) & 0xff);

	/* vertical porch params */
	regmap_write(adv->regmap_cec, 0x30, mode->vtotal >> 4);
	regmap_write(adv->regmap_cec, 0x31, (mode->vtotal << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x32, vsw >> 4);
	regmap_write(adv->regmap_cec, 0x33, (vsw << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x34, vfp >> 4);
	regmap_write(adv->regmap_cec, 0x35, (vfp << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x36, vbp >> 4);
	regmap_write(adv->regmap_cec, 0x37, (vbp << 4) & 0xff);

	/* 03-03 Reset Internal Timing Generator */
	regmap_write(adv->regmap_cec, 0x27, 0xcb);
	regmap_write(adv->regmap_cec, 0x27, 0x8b);
	regmap_write(adv->regmap_cec, 0x27, 0xcb);

}

void adv7533_dsi_power_on(struct adv7511 *adv)
{
	struct mipi_dsi_device *dsi = adv->dsi;
	struct drm_display_mode *mode = &adv->curr_mode;
	u8 clock_div_by_lanes[] = { 6, 4, 3 };	/* 2, 3, 4 lanes */

	/* Gate DSI LP Oscillator */
	regmap_update_bits(adv->regmap_cec, 0x03, 0x02, 0x00);

	/* 01-03 Initialisation (Fixed) Registers */
	regmap_register_patch(adv->regmap_cec, adv7533_cec_fixed_registers,
			      ARRAY_SIZE(adv7533_cec_fixed_registers));

	/* 02-04 DSI Lanes */
	regmap_write(adv->regmap_cec, 0x1c, dsi->lanes << 4);

	/* 02-05 DSI Pixel Clock Divider */
	regmap_write(adv->regmap_cec, 0x16,
		     clock_div_by_lanes[dsi->lanes - 2] << 3);

	if (adv->use_timing_gen)
		adv7511_dsi_config_timing_gen(adv);
	else
		regmap_write(adv->regmap_cec, 0x27, 0x0b);

	/* 04-01 HDMI Output */
	regmap_write(adv->regmap, 0xaf, 0x16);

	/* 09-03 AVI Infoframe - RGB - 16-9 Aspect Ratio */
	regmap_write(adv->regmap, ADV7511_REG_AVI_INFOFRAME(0), 0x10);
	if (FORMAT_RATIO(mode->hdisplay, mode->vdisplay) == RATIO_16_9)
		regmap_write(adv->regmap, ADV7511_REG_AVI_INFOFRAME(1), 0x28);
	else if (FORMAT_RATIO(mode->hdisplay, mode->vdisplay) == RATIO_4_3)
		regmap_write(adv->regmap, ADV7511_REG_AVI_INFOFRAME(1), 0x18);

	/* 04-04 GC Packet Enable */
	regmap_write(adv->regmap, ADV7511_REG_PACKET_ENABLE0, 0x80);

	/* 04-06 GC Colour Depth - 24 Bit */
	regmap_write(adv->regmap, 0x4c, 0x04);

	/* 04-09 Down Dither Output Colour Depth - 8 Bit (default) */
	regmap_write(adv->regmap, 0x49, 0x00);

	/* 07-01 CEC Power Mode - Always Active */
	regmap_write(adv->regmap_cec, 0xbe, 0x3d);

	/* 04-03 HDMI Output Enable  */
	regmap_write(adv->regmap_cec, 0x03, 0x89);
	/* disable test mode */
	regmap_write(adv->regmap_cec, 0x55, 0x00);

}

void adv7533_dsi_power_off(struct adv7511 *adv)
{
	/* disable hdmi */
	regmap_write(adv->regmap_cec, 0x03, 0x0b);
	/* disable internal timing generator */
	regmap_write(adv->regmap_cec, 0x27, 0x0b);
}

enum drm_mode_status adv7533_mode_valid(struct adv7511 *adv,
					const struct drm_display_mode *mode)
{
	struct mipi_dsi_device *dsi = adv->dsi;
	u8 bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	/* Check max clock for either 7533 or 7535 */
	if (mode->clock > adv->info->max_mode_clock_khz)
		return MODE_CLOCK_HIGH;

	/* Check max clock for each lane */
	if (mode->clock * bpp > adv->info->max_lane_freq_khz * adv->num_dsi_lanes)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

int adv7533_patch_registers(struct adv7511 *adv)
{
	return regmap_register_patch(adv->regmap,
				     adv7533_fixed_registers,
				     ARRAY_SIZE(adv7533_fixed_registers));
}

int adv7533_patch_cec_registers(struct adv7511 *adv)
{
	return regmap_register_patch(adv->regmap_cec,
				    adv7533_cec_fixed_registers,
				    ARRAY_SIZE(adv7533_cec_fixed_registers));
}

int adv7533_attach_dsi(struct adv7511 *adv)
{
	struct device *dev = &adv->i2c_main->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	const struct mipi_dsi_device_info info = { .type = "adv7533",
						   .channel = adv->channel_id,
						   .node = NULL,
						 };

	host = of_find_mipi_dsi_host_by_node(adv->host_node);
	if (!host)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "failed to find dsi host\n");

	dsi = devm_mipi_dsi_device_register_full(dev, host, &info);
	if (IS_ERR(dsi))
		return dev_err_probe(dev, PTR_ERR(dsi),
				     "failed to create dsi device\n");

	adv->dsi = dsi;

	dsi->lanes = adv->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to attach dsi to host\n");

	return 0;
}

int adv7533_parse_dt(struct device_node *np, struct adv7511 *adv)
{
	struct device *dev = &adv->i2c_main->dev;
	u32 num_lanes = 0, channel_id = 0;

	of_property_read_u32(np, "adi,dsi-channel", &channel_id);
	of_property_read_u32(np, "adi,dsi-lanes", &num_lanes);

	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}

	if (channel_id > 3) {
		dev_err(dev, "Invalid dsi-channel: %d\n", channel_id);
		return -EINVAL;
	}

	adv->num_dsi_lanes = num_lanes;
	adv->channel_id = channel_id;

	adv->host_node = of_graph_get_remote_node(np, 0, 0);
	if (!adv->host_node)
		return -ENODEV;

	of_node_put(adv->host_node);

	adv->use_timing_gen = !of_property_read_bool(np,
						"adi,disable-timing-generator");

	of_property_read_u32(np, "adi,addr-cec", &adv->addr_cec);
	of_property_read_u32(np, "adi,addr-edid", &adv->addr_edid);
	of_property_read_u32(np, "adi,addr-pkt", &adv->addr_pkt);

	/* TODO: Check if these need to be parsed by DT or not */
	adv->rgb = true;
	adv->embedded_sync = false;

	return 0;
}
