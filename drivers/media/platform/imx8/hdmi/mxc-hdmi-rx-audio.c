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

#include "mxc-hdmi-rx.h"
#include "../../../../mxc/hdp/API_Audio.h"
#include "../../../../mxc/hdp/API_HDMI_RX_Audio.h"
#include "../../../../mxc/hdp/sink_pif.h"
#include "../../../../mxc/hdp/aif_pckt2smp.h"


static int get_audio_infoframe(state_struct *state, unsigned int *chan)
{

	unsigned int regread;
	int ret = 0;
	int times = 0;

	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_TYPE_CFG1 << 2), F_INFO_TYPE1(0x84));

	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INT_MASK << 2), 0x1FFFE);

	do {
		cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2), &regread);
		udelay(100);
		times++;
	} while (!(regread & (1 << 0)) && times < 5000);

	if (times == 5000) {
		ret = -EINVAL;
		return ret;
	}

	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_CTRL << 2), F_PACKET_RDN_WR(0x0) | F_PACKET_NUM(0x0));

	times = 0;

	do {
		cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2), &regread);
		udelay(10);
		times++;
	} while (!(regread & (1 << 16)) && times < 100);

	if (times == 100) {
		ret = -EINVAL;
		return ret;
	}

	cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INFO_DATA1 << 2), &regread);

	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_TYPE_CFG1 << 2), F_INFO_TYPE1(0x00));

	*chan = ((regread & 0x700) >> 8) + 1;

	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_HEADER << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_DATA1 << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_DATA2 << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_DATA3 << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_DATA4 << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_DATA5 << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_DATA6 << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_DATA7 << 2), 0);

	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_CTRL << 2), F_PACKET_RDN_WR(0x1) | F_PACKET_NUM(0x0));

	times = 0;
	do {
		cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2), &regread);
		udelay(10);
		times++;
	} while (!(regread & (1 << 16)) && times < 100);

	if (times == 100) {
		ret = -EINVAL;
		return ret;
	}

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



static int mxc_hdmi_rx_audio(struct mxc_hdmi_rx_dev *hdmi)
{
	state_struct *state = &hdmi->state;
	u32 regread;
	u32 rate;
	u32 chan = 2;
	CDN_API_STATUS status;
	int ret;

	ret = get_audio_infoframe(state, &chan);
	if (ret)
		return ret;

	status = CDN_API_RX_AudioAutoConfig(state, chan, chan/2, 0, 32, 32);
	if (status != CDN_OK)
		return -EINVAL;

	if (cdn_apb_read(state, ADDR_AIF_ENCODER + (AIF_ACR_N_ST << 2), &regread))
		return -EINVAL;

	rate = select_rate(hdmi->timings->timings.bt.pixelclock/1000, regread);

	hdmi->channels = chan;
	hdmi->sample_rate = rate;

	return 0;
}

/*
 * HDMI audio codec callbacks
 */
static int mxc_hdmi_rx_audio_hw_params(struct device *dev, void *data,
				    struct hdmi_codec_daifmt *daifmt,
				    struct hdmi_codec_params *params)
{
	return 0;
}

static void mxc_hdmi_rx_audio_shutdown(struct device *dev, void *data)
{
	pm_runtime_put_sync(dev);
}

static int mxc_hdmi_rx_audio_startup(struct device *dev, void *data)
{
	struct mxc_hdmi_rx_dev *hdmi = dev_get_drvdata(dev);
	int ret;

	pm_runtime_get_sync(dev);
	ret = mxc_hdmi_rx_audio(hdmi);

	return ret;
}

static int mxc_hdmi_rx_audio_get_eld(struct device *dev, void *data, uint8_t *buf, size_t len)
{
	struct mxc_hdmi_rx_dev *hdmi = dev_get_drvdata(dev);

	if (len < 8)
		return -EINVAL;

	memcpy(buf, &hdmi->sample_rate, 4);
	memcpy(buf + 4, &hdmi->channels, 4);

	return 0;
}

static const struct hdmi_codec_ops mxc_hdmi_rx_audio_codec_ops = {
	.hw_params = mxc_hdmi_rx_audio_hw_params,
	.audio_shutdown = mxc_hdmi_rx_audio_shutdown,
	.audio_startup = mxc_hdmi_rx_audio_startup,
	.get_eld = mxc_hdmi_rx_audio_get_eld,
};

void mxc_hdmi_rx_register_audio_driver(struct device *dev)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &mxc_hdmi_rx_audio_codec_ops,
		.max_i2s_channels = 8,
		.i2s = 1,
	};
	struct platform_device *pdev;

	pdev = platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
					     2, &codec_data,
					     sizeof(codec_data));
	if (IS_ERR(pdev))
		return;

	DRM_INFO("%s driver bound to HDMI\n", HDMI_CODEC_DRV_NAME);
}


