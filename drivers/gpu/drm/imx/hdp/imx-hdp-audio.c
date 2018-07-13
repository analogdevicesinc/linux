/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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
#include <sound/hdmi-codec.h>

#include "imx-hdp.h"
#include "imx-hdmi.h"
#include "imx-dp.h"
#include "../imx-drm.h"

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

static int select_N_index(u32 pclk)
{
	int i = 0;

	for (i = 0; i < 7; i++) {
		if (pclk == TMDS_rate_table[i])
			break;
	}

	if (i == 7)
		DRM_WARN("pclkc %d is not supported!\n", pclk);

	return i;
}

static void imx_hdmi_audio_avi_set(state_struct *state,
						u32 channels)
{
	struct hdmi_audio_infoframe frame;
	u8 buf[32];
	int ret;

	hdmi_audio_infoframe_init(&frame);

	frame.channels = channels;
	frame.coding_type = HDMI_AUDIO_CODING_TYPE_STREAM;

	ret = hdmi_audio_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_ERROR("failed to pack audio infoframe: %d\n", ret);
		return;
	}

	buf[0] = 0;

	CDN_API_InfoframeSet(state, 1, sizeof(buf),
				    (u32 *)buf, HDMI_INFOFRAME_TYPE_AUDIO);
}

static u32 imx_hdp_audio(struct imx_hdp *hdmi, AUDIO_TYPE type, u32 sample_rate, u32 channels, u32 width)
{
	AUDIO_FREQ  freq;
	AUDIO_WIDTH bits;
	int ncts_n;
	state_struct *state = &hdmi->state;
	int idx_n = select_N_index(hdmi->video.cur_mode.clock);

	switch (sample_rate) {
	case 32000:
		freq = AUDIO_FREQ_32;
		ncts_n = N_table_32k[idx_n];
		break;
	case 44100:
		freq = AUDIO_FREQ_44_1;
		ncts_n = N_table_44k[idx_n];
		break;
	case 48000:
		freq = AUDIO_FREQ_48;
		ncts_n = N_table_48k[idx_n];
		break;
	case 88200:
		freq = AUDIO_FREQ_88_2;
		ncts_n = N_table_44k[idx_n] * 2;
		break;
	case 96000:
		freq = AUDIO_FREQ_96;
		ncts_n = N_table_48k[idx_n] * 2;
		break;
	case 176400:
		freq = AUDIO_FREQ_176_4;
		ncts_n = N_table_44k[idx_n] * 4;
		break;
	case 192000:
		freq = AUDIO_FREQ_192;
		ncts_n = N_table_48k[idx_n] * 4;
		break;
	default:
		return -EINVAL;
	}

	switch (width) {
	case 16:
		bits = AUDIO_WIDTH_16;
		break;
	case 24:
		bits = AUDIO_WIDTH_24;
		break;
	case 32:
		bits = AUDIO_WIDTH_32;
		break;
	default:
		return -EINVAL;
	}


	CDN_API_AudioOff_blocking(state, type);
	CDN_API_AudioAutoConfig_blocking(state,
				type,
				channels,
				freq,
				0,
				bits,
				hdmi->audio_type,
				ncts_n,
				AUDIO_MUTE_MODE_UNMUTE);

	if (hdmi->audio_type == CDN_HDMITX_TYPHOON ||
			hdmi->audio_type == CDN_HDMITX_KIRAN)
		imx_hdmi_audio_avi_set(state, channels);
	return 0;
}

/*
 * HDMI audio codec callbacks
 */
static int imx_hdp_audio_hw_params(struct device *dev, void *data,
				    struct hdmi_codec_daifmt *daifmt,
				    struct hdmi_codec_params *params)
{
	struct imx_hdp *hdmi = dev_get_drvdata(dev);
	unsigned int chan = params->cea.channels;

	dev_dbg(hdmi->dev, "%s: %u Hz, %d bit, %d channels\n", __func__,
		params->sample_rate, params->sample_width, chan);

	if (!hdmi->bridge.encoder)
		return -ENODEV;

	if (daifmt->fmt == HDMI_I2S)
		imx_hdp_audio(hdmi,
				AUDIO_TYPE_I2S,
				params->sample_rate,
				chan,
				params->sample_width);
	else if (daifmt->fmt == HDMI_SPDIF)
		imx_hdp_audio(hdmi,
				AUDIO_TYPE_SPIDIF_EXTERNAL,
				params->sample_rate,
				chan,
				params->sample_width);
	else
		return -EINVAL;

	return 0;
}

static void imx_hdp_audio_shutdown(struct device *dev, void *data)
{
	struct imx_hdp *hdmi = dev_get_drvdata(dev);
	state_struct *state = &hdmi->state;

	CDN_API_InfoframeRemovePacket(state, 0x1, 0x84);
}

static int imx_hdp_audio_get_eld(struct device *dev, void *data, uint8_t *buf, size_t len)
{
	struct imx_hdp *hdmi = dev_get_drvdata(dev);

	memcpy(buf, hdmi->connector.eld, min(sizeof(hdmi->connector.eld), len));

	return 0;
}

static const struct hdmi_codec_ops imx_hdp_audio_codec_ops = {
	.hw_params = imx_hdp_audio_hw_params,
	.audio_shutdown = imx_hdp_audio_shutdown,
	.get_eld = imx_hdp_audio_get_eld,
};

void imx_hdp_register_audio_driver(struct device *dev)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &imx_hdp_audio_codec_ops,
		.max_i2s_channels = 8,
		.i2s = 1,
	};
	struct platform_device *pdev;

	pdev = platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
					     1, &codec_data,
					     sizeof(codec_data));
	if (IS_ERR(pdev))
		return;

	DRM_INFO("%s driver bound to HDMI\n", HDMI_CODEC_DRV_NAME);
}


