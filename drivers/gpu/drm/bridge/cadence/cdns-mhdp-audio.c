// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Chris Zhong <zyw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/reset.h>
#include <drm/bridge/cdns-mhdp.h>
#include <sound/hdmi-codec.h>
#include <drm/drm_of.h>
#include <drm/drm_vblank.h>
#include <drm/drm_print.h>

#define CDNS_DP_SPDIF_CLK		200000000

static u32 TMDS_rate_table[7] = {
	25200, 27000, 54000, 74250, 148500, 297000, 594000,
};

static u32 N_table_32k[7] = {
/* 25200/27000/54000/74250/148500/297000/594000 */
	4096, 4096, 4096, 4096, 4096, 3072, 3072,
};

static u32 N_table_44k[7] = {
	6272, 6272, 6272, 6272, 6272, 4704, 9408,
};

static u32 N_table_48k[7] = {
	6144, 6144, 6144, 6144, 6144, 5120, 6144,
};

static int select_N_index(u32 pclk)
{
	int num = sizeof(TMDS_rate_table)/sizeof(int);
	int i = 0;

	for (i = 0; i < num ; i++)
		if (pclk == TMDS_rate_table[i])
			break;

	if (i == num) {
		DRM_WARN("pclkc %d is not supported!\n", pclk);
		return num-1;
	}

	return i;
}

static void hdmi_audio_avi_set(struct cdns_mhdp_device *mhdp,
						u32 channels)
{
	struct hdmi_audio_infoframe frame;
	u8 buf[32];
	int ret;

	hdmi_audio_infoframe_init(&frame);

	frame.channels = channels;
	frame.coding_type = HDMI_AUDIO_CODING_TYPE_STREAM;

	if (channels == 2)
		frame.channel_allocation = 0;
	else if (channels == 4)
		frame.channel_allocation = 0x3;
	else if (channels == 6)
		frame.channel_allocation = 0xB;
	else if (channels == 8)
		frame.channel_allocation = 0x13;

	ret = hdmi_audio_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_ERROR("failed to pack audio infoframe: %d\n", ret);
		return;
	}

	buf[0] = 0;

	cdns_mhdp_infoframe_set(mhdp, 1, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_AUDIO);
}

int cdns_mhdp_audio_stop(struct cdns_mhdp_device *mhdp,
			 struct audio_info *audio)
{
	int ret;

	if (audio->connector_type == DRM_MODE_CONNECTOR_DisplayPort) {
		ret = cdns_mhdp_reg_write(mhdp, AUDIO_PACK_CONTROL, 0);
		if (ret) {
			DRM_DEV_ERROR(mhdp->dev, "audio stop failed: %d\n", ret);
			return ret;
		}
	}

	cdns_mhdp_bus_write(0, mhdp, SPDIF_CTRL_ADDR);

	/* clearn the audio config and reset */
	cdns_mhdp_bus_write(0, mhdp, AUDIO_SRC_CNTL);
	cdns_mhdp_bus_write(0, mhdp, AUDIO_SRC_CNFG);
	cdns_mhdp_bus_write(AUDIO_SW_RST, mhdp, AUDIO_SRC_CNTL);
	cdns_mhdp_bus_write(0, mhdp, AUDIO_SRC_CNTL);

	/* reset smpl2pckt component  */
	cdns_mhdp_bus_write(0, mhdp, SMPL2PKT_CNTL);
	cdns_mhdp_bus_write(AUDIO_SW_RST, mhdp, SMPL2PKT_CNTL);
	cdns_mhdp_bus_write(0, mhdp, SMPL2PKT_CNTL);

	/* reset FIFO */
	cdns_mhdp_bus_write(AUDIO_SW_RST, mhdp, FIFO_CNTL);
	cdns_mhdp_bus_write(0, mhdp, FIFO_CNTL);

	if (audio->format == AFMT_SPDIF_INT)
		clk_disable_unprepare(mhdp->spdif_clk);

	return 0;
}
EXPORT_SYMBOL(cdns_mhdp_audio_stop);

int cdns_mhdp_audio_mute(struct cdns_mhdp_device *mhdp, bool enable)
{
	struct audio_info *audio = &mhdp->audio_info;
	int ret = true;

	if (audio->connector_type == DRM_MODE_CONNECTOR_DisplayPort) {
		ret = cdns_mhdp_reg_write_bit(mhdp, DP_VB_ID, 4, 1, enable);
		if (ret)
			DRM_DEV_ERROR(mhdp->dev, "audio mute failed: %d\n", ret);
	}

	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_audio_mute);

static void cdns_mhdp_audio_config_i2s(struct cdns_mhdp_device *mhdp,
				       struct audio_info *audio)
{
	int sub_pckt_num = 1, i2s_port_en_val = 0xf, i;
	int idx = select_N_index(mhdp->mode.clock);
	int numofchannels = audio->channels;
	u32 val, ncts;
	u32 disable_port3 = 0;
	u32 audio_type = 0x2; /* L-PCM */
	u32 transmission_type = 0; /* not required for L-PCM */

	if (numofchannels == 2) {
		if (mhdp->dp.num_lanes == 1)
			sub_pckt_num = 2;
		else
			sub_pckt_num = 4;

		i2s_port_en_val = 1;
	} else if (numofchannels == 4) {
		i2s_port_en_val = 3;
	} else if (numofchannels == 6) {
		numofchannels = 8;
		disable_port3 = 1;
	} else if ((numofchannels == 8) && (audio->non_pcm)) {
		audio_type = 0x9;         /* HBR packet type */
		transmission_type = 0x9;  /* HBR packet type */
	}

	cdns_mhdp_bus_write(0x0, mhdp, SPDIF_CTRL_ADDR);

	val = SYNC_WR_TO_CH_ZERO;
	val |= disable_port3 << 4;
	cdns_mhdp_bus_write(val, mhdp, FIFO_CNTL);

	val = MAX_NUM_CH(numofchannels);
	val |= NUM_OF_I2S_PORTS(numofchannels);
	val |= audio_type << 7;
	val |= CFG_SUB_PCKT_NUM(sub_pckt_num);
	cdns_mhdp_bus_write(val, mhdp, SMPL2PKT_CNFG);

	if (audio->sample_width == 16)
		val = 0;
	else if (audio->sample_width == 24)
		val = 1 << 9;
	else
		val = 2 << 9;

	val |= AUDIO_CH_NUM(numofchannels);
	val |= I2S_DEC_PORT_EN(i2s_port_en_val);
	val |= TRANS_SMPL_WIDTH_32;
	val |= transmission_type << 13;
	cdns_mhdp_bus_write(val, mhdp, AUDIO_SRC_CNFG);

	for (i = 0; i < (numofchannels + 1) / 2; i++) {
		if (audio->sample_width == 16)
			val = (0x02 << 8) | (0x02 << 20);
		else if (audio->sample_width == 24)
			val = (0x0b << 8) | (0x0b << 20);

		val |= ((2 * i) << 4) | ((2 * i + 1) << 16);
		cdns_mhdp_bus_write(val, mhdp, STTS_BIT_CH(i));
	}

	switch (audio->sample_rate) {
	case 32000:
		val = SAMPLING_FREQ(3) |
		      ORIGINAL_SAMP_FREQ(0xc);
		ncts = N_table_32k[idx];
		break;
	case 44100:
		val = SAMPLING_FREQ(0) |
		      ORIGINAL_SAMP_FREQ(0xf);
		ncts = N_table_44k[idx];
		break;
	case 48000:
		val = SAMPLING_FREQ(2) |
		      ORIGINAL_SAMP_FREQ(0xd);
		ncts = N_table_48k[idx];
		break;
	case 88200:
		val = SAMPLING_FREQ(8) |
		      ORIGINAL_SAMP_FREQ(0x7);
		ncts = N_table_44k[idx] * 2;
		break;
	case 96000:
		val = SAMPLING_FREQ(0xa) |
		      ORIGINAL_SAMP_FREQ(5);
		ncts = N_table_48k[idx] * 2;
		break;
	case 176400:
		val = SAMPLING_FREQ(0xc) |
		      ORIGINAL_SAMP_FREQ(3);
		ncts = N_table_44k[idx] * 4;
		break;
	case 192000:
	default:
		val = SAMPLING_FREQ(0xe) |
		      ORIGINAL_SAMP_FREQ(1);
		ncts = N_table_48k[idx] * 4;
		break;
	}
	val |= 4;
	cdns_mhdp_bus_write(val, mhdp, COM_CH_STTS_BITS);

	if (audio->connector_type == DRM_MODE_CONNECTOR_HDMIA)
		cdns_mhdp_reg_write(mhdp, CM_I2S_CTRL, ncts | 0x4000000);

	cdns_mhdp_bus_write(SMPL2PKT_EN, mhdp, SMPL2PKT_CNTL);
	cdns_mhdp_bus_write(I2S_DEC_START, mhdp, AUDIO_SRC_CNTL);
}

static void cdns_mhdp_audio_config_spdif(struct cdns_mhdp_device *mhdp)
{
	u32 val;

	cdns_mhdp_bus_write(SYNC_WR_TO_CH_ZERO, mhdp, FIFO_CNTL);

	val = MAX_NUM_CH(2) | AUDIO_TYPE_LPCM | CFG_SUB_PCKT_NUM(4);
	cdns_mhdp_bus_write(val, mhdp, SMPL2PKT_CNFG);
	cdns_mhdp_bus_write(SMPL2PKT_EN, mhdp, SMPL2PKT_CNTL);

	val = SPDIF_ENABLE | SPDIF_AVG_SEL | SPDIF_JITTER_BYPASS;
	cdns_mhdp_bus_write(val, mhdp, SPDIF_CTRL_ADDR);

	clk_prepare_enable(mhdp->spdif_clk);
	clk_set_rate(mhdp->spdif_clk, CDNS_DP_SPDIF_CLK);
}

int cdns_mhdp_audio_config(struct cdns_mhdp_device *mhdp,
			   struct audio_info *audio)
{
	int ret;

	/* reset the spdif clk before config */
	if (audio->format == AFMT_SPDIF_INT) {
		reset_control_assert(mhdp->spdif_rst);
		reset_control_deassert(mhdp->spdif_rst);
	}

	if (audio->connector_type == DRM_MODE_CONNECTOR_DisplayPort) {
		ret = cdns_mhdp_reg_write(mhdp, CM_LANE_CTRL, LANE_REF_CYC);
		if (ret)
			goto err_audio_config;

		ret = cdns_mhdp_reg_write(mhdp, CM_CTRL, 0);
		if (ret)
			goto err_audio_config;
	} else {
		/* HDMI Mode */
		ret = cdns_mhdp_reg_write(mhdp, CM_CTRL, 8);
		if (ret)
			goto err_audio_config;
	}

	if (audio->format == AFMT_I2S)
		cdns_mhdp_audio_config_i2s(mhdp, audio);
	else if (audio->format == AFMT_SPDIF_INT)
		cdns_mhdp_audio_config_spdif(mhdp);

	if (audio->connector_type == DRM_MODE_CONNECTOR_DisplayPort)
		ret = cdns_mhdp_reg_write(mhdp, AUDIO_PACK_CONTROL, AUDIO_PACK_EN);

	if (audio->connector_type == DRM_MODE_CONNECTOR_HDMIA)
		hdmi_audio_avi_set(mhdp, audio->channels);

err_audio_config:
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "audio config failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_audio_config);

static int audio_hw_params(struct device *dev,  void *data,
				  struct hdmi_codec_daifmt *daifmt,
				  struct hdmi_codec_params *params)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	struct audio_info audio = {
		.sample_width = params->sample_width,
		.sample_rate = params->sample_rate,
		.channels = params->channels,
		.connector_type = mhdp->connector.base.connector_type,
	};
	int ret;

	switch (daifmt->fmt) {
	case HDMI_I2S:
		audio.format = AFMT_I2S;
		break;
	case HDMI_SPDIF:
		audio.format = AFMT_SPDIF_EXT;
		break;
	default:
		DRM_DEV_ERROR(dev, "Invalid format %d\n", daifmt->fmt);
		ret = -EINVAL;
		goto out;
	}

	audio.non_pcm = params->iec.status[0] & IEC958_AES0_NONAUDIO;

	ret = cdns_mhdp_audio_config(mhdp, &audio);
	if (!ret)
		mhdp->audio_info = audio;

out:
	return ret;
}

static void audio_shutdown(struct device *dev, void *data)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	int ret;

	ret = cdns_mhdp_audio_stop(mhdp, &mhdp->audio_info);
	if (!ret)
		mhdp->audio_info.format = AFMT_UNUSED;
}

static int audio_digital_mute(struct device *dev, void *data,
				     bool enable)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	int ret;

	ret = cdns_mhdp_audio_mute(mhdp, enable);

	return ret;
}

static int audio_get_eld(struct device *dev, void *data,
				u8 *buf, size_t len)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	memcpy(buf, mhdp->connector.base.eld,
	       min(sizeof(mhdp->connector.base.eld), len));

	return 0;
}

static int audio_hook_plugged_cb(struct device *dev, void *data,
				 hdmi_codec_plugged_cb fn,
				 struct device *codec_dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	return cdns_hdmi_set_plugged_cb(mhdp, fn, codec_dev);
}

static const struct hdmi_codec_ops audio_codec_ops = {
	.hw_params = audio_hw_params,
	.audio_shutdown = audio_shutdown,
	.digital_mute = audio_digital_mute,
	.get_eld = audio_get_eld,
	.hook_plugged_cb = audio_hook_plugged_cb,
};

int cdns_mhdp_register_audio_driver(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	struct hdmi_codec_pdata codec_data = {
		.i2s = 1,
		.spdif = 1,
		.ops = &audio_codec_ops,
		.max_i2s_channels = 8,
	};

	mhdp->audio_pdev = platform_device_register_data(
			      dev, HDMI_CODEC_DRV_NAME, 1,
			      &codec_data, sizeof(codec_data));

	return PTR_ERR_OR_ZERO(mhdp->audio_pdev);
}

void cdns_mhdp_unregister_audio_driver(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	platform_device_unregister(mhdp->audio_pdev);
}
