/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

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
#include "../../../drivers/mxc/hdp/API_Audio.h"
#include "../../../drivers/gpu/drm/imx/hdp/imx-hdp.h"
#include "../../../drivers/video/fbdev/mxc/API_AFE_t28hpc_hdmitx.h"

#define SUPPORT_RATE_NUM 10
#define SUPPORT_CHANNEL_NUM 10

struct imx_cdnhdmi_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	int protocol;
};

static int imx_cdnhdmi_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	static struct snd_pcm_hw_constraint_list constraint_rates;
	static struct snd_pcm_hw_constraint_list constraint_channels;
	static u32 support_rates[SUPPORT_RATE_NUM];
	static u32 support_channels[SUPPORT_CHANNEL_NUM];
	int ret;

	support_rates[0] = 48000;
	support_rates[1] = 96000;
	support_rates[2] = 32000;
	support_rates[3] = 192000;
	constraint_rates.list = support_rates;
	constraint_rates.count = 4;

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
						&constraint_rates);
	if (ret)
		return ret;

	support_channels[0] = 2;
	support_channels[1] = 4;
	support_channels[2] = 8;
	constraint_channels.list = support_channels;
	constraint_channels.count = 3;

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
						&constraint_channels);
	if (ret)
		return ret;

	return 0;
}

static int imx_cdnhdmi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	unsigned int sample_rate = params_rate(params);
	unsigned int channels = params_channels(params);
	unsigned int width = params_physical_width(params);
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2, 32);
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	if (data->protocol == 1)
		imx_hdp_audio(AUDIO_TYPE_I2S, sample_rate, channels, width);
	else
		imx_hdmi_audio(AUDIO_TYPE_I2S, sample_rate, channels, width);

	return 0;
}

static struct snd_soc_ops imx_cdnhdmi_ops = {
	.startup = imx_cdnhdmi_startup,
	.hw_params = imx_cdnhdmi_hw_params,
};

static int imx_cdnhdmi_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *cdnhdmi_np = NULL;
	struct platform_device *cpu_pdev;
	struct imx_cdnhdmi_data *data;
	int ret;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	of_property_read_u32(pdev->dev.of_node, "protocol",
					&data->protocol);

	data->dai.name = "imx8 hdmi";
	data->dai.stream_name = "imx8 hdmi";
	data->dai.codec_dai_name = "snd-soc-dummy-dai";
	data->dai.codec_name = "snd-soc-dummy";
	data->dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platform_of_node = cpu_np;
	data->dai.ops = &imx_cdnhdmi_ops;
	data->dai.playback_only = true;
	data->dai.capture_only = false;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;

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
	if (cdnhdmi_np)
		of_node_put(cdnhdmi_np);
	return ret;
}

static const struct of_device_id imx_cdnhdmi_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-cdnhdmi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_cdnhdmi_dt_ids);

static struct platform_driver imx_cdnhdmi_driver = {
	.driver = {
		.name = "imx-cdnhdmi",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_cdnhdmi_dt_ids,
	},
	.probe = imx_cdnhdmi_probe,
};
module_platform_driver(imx_cdnhdmi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX hdmi audio ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-cdnhdmi");
