/*
 *  Copyright (C) 2012-2013, Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include "../codecs/adau17x1.h"

static const struct snd_soc_dapm_widget zed_adau1761_widgets[] = {
	SND_SOC_DAPM_SPK("Line Out", NULL),
	SND_SOC_DAPM_HP("Headphone Out", NULL),
	SND_SOC_DAPM_MIC("Mic In", NULL),
	SND_SOC_DAPM_MIC("Line In", NULL),
};

static const struct snd_soc_dapm_route zed_adau1761_routes[] = {
	{ "Line Out", NULL, "LOUT" },
	{ "Line Out", NULL, "ROUT" },
	{ "Headphone Out", NULL, "LHP" },
	{ "Headphone Out", NULL, "RHP" },
	{ "Mic In", NULL, "MICBIAS" },
	{ "LINN", NULL, "Mic In" },
	{ "RINN", NULL, "Mic In" },
	{ "LAUX", NULL, "Line In" },
	{ "RAUX", NULL, "Line In" },
};

static int zed_adau1761_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_rate;
	int ret;

	switch (params_rate(params)) {
	case 48000:
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 96000:
		pll_rate = 48000 * 1024;
		break;
	case 44100:
	case 7350:
	case 11025:
	case 14700:
	case 22050:
	case 29400:
	case 88200:
		pll_rate = 44100 * 1024;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_pll(codec_dai, ADAU17X1_PLL,
			ADAU17X1_PLL_SRC_MCLK, 12288000, pll_rate);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, ADAU17X1_CLK_SRC_PLL, pll_rate,
			SND_SOC_CLOCK_IN);

	return ret;
}

static struct snd_soc_ops zed_adau1761_ops = {
	.hw_params = zed_adau1761_hw_params,
};

static struct snd_soc_dai_link zed_adau1761_dai_link = {
	.name = "adau1761",
	.stream_name = "adau1761",
	.codec_dai_name = "adau-hifi",
	.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	.ops = &zed_adau1761_ops,
};

static struct snd_soc_card zed_adau1761_card = {
	.name = "ZED ADAU1761",
	.owner = THIS_MODULE,
	.dai_link = &zed_adau1761_dai_link,
	.num_links = 1,
	.dapm_widgets = zed_adau1761_widgets,
	.num_dapm_widgets = ARRAY_SIZE(zed_adau1761_widgets),
	.dapm_routes = zed_adau1761_routes,
	.num_dapm_routes = ARRAY_SIZE(zed_adau1761_routes),
	.fully_routed = true,
};

static int zed_adau1761_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &zed_adau1761_card;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;

	card->dev = &pdev->dev;

	zed_adau1761_dai_link.codec_of_node = of_parse_phandle(of_node, "audio-codec", 0);
	zed_adau1761_dai_link.cpu_of_node = of_parse_phandle(of_node, "cpu-dai", 0);
	zed_adau1761_dai_link.platform_of_node = zed_adau1761_dai_link.cpu_of_node;

	if (!zed_adau1761_dai_link.codec_of_node ||
		!zed_adau1761_dai_link.cpu_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int zed_adau1761_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id zed_adau1761_of_match[] = {
	{ .compatible = "digilent,zed-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, zed_adau1761_of_match);

static struct platform_driver zed_adau1761_card_driver = {
	.driver = {
		.name = "zed-adau1761-snd",
		.owner = THIS_MODULE,
		.of_match_table = zed_adau1761_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = zed_adau1761_probe,
	.remove = zed_adau1761_remove,
};
module_platform_driver(zed_adau1761_card_driver);

MODULE_DESCRIPTION("ASoC ZED board ADAU1761 driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:zed-adau1761-snd");
