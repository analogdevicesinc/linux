// SPDX-License-Identifier: (GPL-2.0+
//
// DSP machine driver
//
// Copyright (c) 2012-2013 by Tensilica Inc. ALL RIGHTS RESERVED.
// Copyright 2018 NXP

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>

struct imx_dsp_audio_data {
	struct snd_soc_dai_link dai[2];
	struct snd_soc_card card;
};

static int imx_dsp_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;
	u32 dai_format = SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_LEFT_J |
			SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
					24576000, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(rtd->dev, "failed to set codec sysclk: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret) {
		dev_err(rtd->dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops imx_dsp_ops_be = {
	.hw_params = imx_dsp_hw_params,
};

static int be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params) {

	struct snd_interval *rate;
	struct snd_interval *channels;
	struct snd_mask *mask;

	rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	rate->max = rate->min = 48000;

	channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	channels->max = channels->min = 2;

	mask = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	snd_mask_none(mask);
	snd_mask_set(mask, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static const struct snd_soc_dapm_route imx_dsp_audio_map[] = {
	{"Playback",  NULL, "Compress Playback"},/* dai route for be and fe */
	{"Playback",  NULL, "Playback"},
};

static int imx_dsp_audio_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np=NULL, *codec_np=NULL, *platform_np=NULL;
	struct platform_device *cpu_pdev;
	struct imx_dsp_audio_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find rpmsg platform device\n");
		ret = -EINVAL;
		goto fail;
	}

        codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
        if (!codec_np) {
                dev_err(&pdev->dev, "phandle missing or invalid\n");
                ret = -EINVAL;
                goto fail;
        }

        platform_np = of_parse_phandle(pdev->dev.of_node, "audio-platform", 0);
        if (!platform_np) {
                dev_err(&pdev->dev, "platform missing or invalid\n");
                ret = -EINVAL;
                goto fail;
        }

	data->dai[0].name = "dsp hifi fe";
	data->dai[0].stream_name = "dsp hifi fe";
	data->dai[0].codec_dai_name = "snd-soc-dummy-dai";
	data->dai[0].codec_name = "snd-soc-dummy";
	data->dai[0].cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai[0].cpu_of_node = cpu_np;
	data->dai[0].platform_of_node = platform_np;
	data->dai[0].playback_only = true;
	data->dai[0].capture_only = false;
	data->dai[0].dpcm_playback = 1;
	data->dai[0].dpcm_capture = 0;
	data->dai[0].dynamic = 1,
	data->dai[0].ignore_pmdown_time = 1,
	data->dai[0].dai_fmt = SND_SOC_DAIFMT_LEFT_J |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	data->dai[1].name = "dsp hifi be";
	data->dai[1].stream_name = "dsp hifi be";
	data->dai[1].codec_dai_name = "cs42888";
	data->dai[1].codec_of_node = codec_np;
	data->dai[1].cpu_dai_name = "snd-soc-dummy-dai";
	data->dai[1].cpu_name = "snd-soc-dummy";
	data->dai[1].platform_name = "snd-soc-dummy";
	data->dai[1].playback_only = true;
	data->dai[1].capture_only = false;
	data->dai[1].dpcm_playback = 1;
	data->dai[1].dpcm_capture = 0;
	data->dai[1].no_pcm = 1,
	data->dai[1].ignore_pmdown_time = 1,
	data->dai[1].dai_fmt = SND_SOC_DAIFMT_LEFT_J |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;
	data->dai[1].ops = &imx_dsp_ops_be;
	data->dai[1].be_hw_params_fixup = be_hw_params_fixup;

	data->card.dapm_routes = imx_dsp_audio_map;
	data->card.num_dapm_routes = ARRAY_SIZE(imx_dsp_audio_map);
	data->card.num_links = 2;
	data->card.dai_link = data->dai;

	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);
	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);
	if (platform_np)
		of_node_put(platform_np);
	return ret;
}

static const struct of_device_id imx_dsp_audio_dt_ids[] = {
	{ .compatible = "fsl,imx-dsp-audio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_dsp_audio_dt_ids);

static struct platform_driver imx_dsp_audio_driver = {
	.driver = {
		.name = "imx-dsp-audio",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_dsp_audio_dt_ids,
	},
	.probe = imx_dsp_audio_probe,
};
module_platform_driver(imx_dsp_audio_driver);

MODULE_LICENSE("GPL v2");
