/*
 * Copyright 2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <linux/hdmi.h>
#include <sound/hdmi-codec.h>

#include "cdns-mhdp-hdmirx.h"

static int get_audio_infoframe(struct cdns_hdmirx_device *hdmirx, unsigned int *chan,
			       u16 timeout_ms)
{
	u8 buffer[32];
	int ret;

	ret = infoframe_poll(hdmirx, HDMI_INFOFRAME_TYPE_AUDIO,
			     buffer, timeout_ms);
	if (ret == 0)
		*chan = (buffer[HDMI_INFOFRAME_HEADER_SIZE+1] & 0x07) + 1;

	pr_info("AUDIOIF: ch:%d\n", *chan);

	return ret;
}

static u32 TMDS_rate_table[7] = {
25200, 27000, 54000, 74250, 148500, 297000, 594000,
};

static u32 N_table_32k[8] = {
/*25200, 27000, 54000, 74250, 148500, 297000, 594000,*/
4096, 4096, 4096, 4096, 4096, 3072, 3072, 4096,
};

static u32 N_table_44k[8] = {
6272, 6272, 6272, 6272, 6272, 4704, 9408, 6272,
};

static u32 N_table_48k[8] = {
6144, 6144, 6144, 6144, 6144, 5120, 6144, 6144,
};

static int select_rate(u32 pclk, u32 N)
{
	int i = 0;
	int rate = 0;

	for (i = 0; i < 7; i++) {
		if (pclk == TMDS_rate_table[i])
			break;
	}

	if (i == 7)
		DRM_WARN("pclkc %d is not supported!\n", pclk);

	if (N_table_32k[i] == N)
		rate = 32000;

	if (N_table_44k[i] == N)
		rate = 44100;

	if (N_table_44k[i] * 2 == N)
		rate = 44100 * 2;

	if (N_table_44k[i] * 4 == N)
		rate = 44100 * 4;

	if (N_table_48k[i] == N)
		rate = 48000;

	if (N_table_48k[i] * 2 == N)
		rate = 48000 * 2;

	if (N_table_48k[i] * 4 == N)
		rate = 48000 * 4;

	return rate;
}

static int hdmirx_audio(struct cdns_hdmirx_device *hdmirx)
{
	u32 regread;
	u32 rate;
	u32 chan = 2;
	int status;
	int ret;

	if (hdmirx->initialized != true)
		return -EINVAL;

	ret = get_audio_infoframe(hdmirx, &chan, 100);
	if (ret)
		return ret;

	status = cdns_hdmirx_audioautoconfig(hdmirx, chan, chan/2, 0, 32, 32);
	if (status)
		return -EINVAL;

	regread = cdns_hdmirx_bus_read(hdmirx, AIF_ACR_N_ST);

	rate = select_rate(hdmirx->timings->timings.bt.pixelclock/1000, regread);

	hdmirx->channels = chan;
	hdmirx->sample_rate = rate;

	return 0;
}

/*
 * HDMI audio codec callbacks
 */
static int hdmirx_audio_hw_params(struct device *dev, void *data,
				    struct hdmi_codec_daifmt *daifmt,
				    struct hdmi_codec_params *params)
{
	return 0;
}

static void hdmirx_audio_shutdown(struct device *dev, void *data)
{
	pm_runtime_put_sync(dev);
}

static int hdmirx_audio_startup(struct device *dev, void *data)
{
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);
	int ret;

	pm_runtime_get_sync(dev);
	ret = hdmirx_audio(hdmirx);
	return ret;
}

static int hdmirx_audio_get_eld(struct device *dev, void *data, uint8_t *buf, size_t len)
{
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);

	if (len < 8)
		return -EINVAL;

	memcpy(buf, &hdmirx->sample_rate, 4);
	memcpy(buf + 4, &hdmirx->channels, 4);

	return 0;
}

static const struct hdmi_codec_ops hdmirx_audio_codec_ops = {
	.hw_params = hdmirx_audio_hw_params,
	.audio_shutdown = hdmirx_audio_shutdown,
	.audio_startup = hdmirx_audio_startup,
	.get_eld = hdmirx_audio_get_eld,
};

void cdns_hdmirx_register_audio_driver(struct device *dev)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &hdmirx_audio_codec_ops,
		.max_i2s_channels = 8,
		.i2s = 1,
	};
	struct platform_device *pdev;

	pdev = platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
					     2, &codec_data,
					     sizeof(codec_data));
	if (IS_ERR(pdev))
		return;

	dev_err(dev, "%s driver bound to HDMI\n", HDMI_CODEC_DRV_NAME);
}
