/*
 *
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 */
#include <linux/module.h>
#include <linux/string.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "../fsl/fsl_rpmsg_i2s.h"

#define WM8960_LINVOL		0x0
#define WM8960_RINVOL		0x1
#define WM8960_LOUT1		0x2
#define WM8960_ROUT1		0x3
#define WM8960_LDAC		0xa
#define WM8960_RDAC		0xb
#define WM8960_LADC		0x15
#define WM8960_RADC		0x16
#define WM8960_LOUT2		0x28
#define WM8960_ROUT2		0x29

static const DECLARE_TLV_DB_SCALE(adc_tlv, -9750, 50, 1);
static const DECLARE_TLV_DB_SCALE(inpga_tlv, -1725, 75, 0);
static const DECLARE_TLV_DB_SCALE(dac_tlv, -12750, 50, 1);
static const DECLARE_TLV_DB_SCALE(out_tlv, -12100, 100, 1);

static struct snd_kcontrol_new rpmsg_wm8960_ctrls[] = {
SOC_DOUBLE_R_TLV("Capture Volume", WM8960_LINVOL, WM8960_RINVOL,
							0, 63, 0, inpga_tlv),
SOC_DOUBLE_R_TLV("Playback Volume", WM8960_LDAC, WM8960_RDAC,
							0, 255, 0, dac_tlv),
SOC_DOUBLE_R_TLV("Headphone Playback Volume", WM8960_LOUT1, WM8960_ROUT1,
		 0, 127, 0, out_tlv),
SOC_DOUBLE_R_TLV("Speaker Playback Volume", WM8960_LOUT2, WM8960_ROUT2,
		 0, 127, 0, out_tlv),
SOC_DOUBLE_R_TLV("ADC PCM Capture Volume", WM8960_LADC, WM8960_RADC,
	0, 255, 0, adc_tlv),
};

#define RPMSG_RATES (SNDRV_PCM_RATE_8000_48000 |\
		SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define RPMSG_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver rpmsg_wm8960_codec_dai = {
	.name = "rpmsg-wm8960-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = RPMSG_RATES,
		.formats = RPMSG_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = RPMSG_RATES,
		.formats = RPMSG_FORMATS,
	},
};

static unsigned int rpmsg_wm8960_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = snd_soc_codec_get_drvdata(codec);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[RPMSG_AUDIO_I2C];
	int err, reg_val;

	mutex_lock(&i2s_info->i2c_lock);
	rpmsg->param.buffer_addr = reg;
	rpmsg->header.cmd = GET_CODEC_VALUE;
	err = i2s_info->send_message(rpmsg, i2s_info);
	reg_val = rpmsg->param.buffer_size;
	mutex_unlock(&i2s_info->i2c_lock);
	if (err)
		return 0;

	return reg_val;
}

static int rpmsg_wm8960_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = snd_soc_codec_get_drvdata(codec);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[RPMSG_AUDIO_I2C];
	int err;

	mutex_lock(&i2s_info->i2c_lock);
	rpmsg->param.buffer_addr = reg;
	rpmsg->param.buffer_size = val;
	rpmsg->header.cmd = SET_CODEC_VALUE;
	err = i2s_info->send_message(rpmsg, i2s_info);
	mutex_unlock(&i2s_info->i2c_lock);
	if (err)
		return err;

	return 0;
}

static int rpmsg_wm8960_probe(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, WM8960_LINVOL, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_RINVOL, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_LADC, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_RADC, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_LDAC, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_RDAC, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_LOUT1, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_ROUT1, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_LOUT2, 0x100, 0x100);
	snd_soc_update_bits(codec, WM8960_ROUT2, 0x100, 0x100);

	return 0;
}

static struct snd_soc_codec_driver rpmsg_wm8960_codec = {
	.probe = rpmsg_wm8960_probe,
	.read = rpmsg_wm8960_read,
	.write = rpmsg_wm8960_write,
	.component_driver = {
		.controls		= rpmsg_wm8960_ctrls,
		.num_controls		= ARRAY_SIZE(rpmsg_wm8960_ctrls),
	},
};

static int rpmsg_wm8960_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(pdev->dev.parent);
	int ret;

	ret = snd_soc_register_codec(dev,
					&rpmsg_wm8960_codec,
					&rpmsg_wm8960_codec_dai,
					1);
	if (ret) {
		dev_err(dev, "%s: snd_soc_register_codec() failed (%d)\n",
			__func__, ret);
		return ret;
	}

	dev_set_drvdata(dev, rpmsg_i2s);

	return 0;
}

static int rpmsg_wm8960_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver rpmsg_wm8960_codec_driver = {
	.driver = {
		.name = RPMSG_CODEC_DRV_NAME,
	},
	.probe = rpmsg_wm8960_codec_probe,
	.remove = rpmsg_wm8960_codec_remove,
};

module_platform_driver(rpmsg_wm8960_codec_driver);

MODULE_DESCRIPTION("rpmsg wm8960 Codec Driver");
MODULE_LICENSE("GPL");
