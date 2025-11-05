// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices ASoC Machine driver for sc5xx
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 * Author: Scott Jiang <Scott.Jiang.Linux@gmail.com>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "../codecs/adau1372.h"
#include "../codecs/adau1962.h"
#include "../codecs/adau1977.h"
#include "../codecs/adau17x1.h"

static const struct snd_soc_dapm_widget sc5xx_adau1761_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("In 1", NULL),
	SND_SOC_DAPM_LINE("In 2", NULL),
	SND_SOC_DAPM_LINE("In 3-4", NULL),

	SND_SOC_DAPM_LINE("Diff Out L", NULL),
	SND_SOC_DAPM_LINE("Diff Out R", NULL),
	SND_SOC_DAPM_LINE("Stereo Out", NULL),
	SND_SOC_DAPM_HP("Capless HP Out", NULL),
};

static const struct snd_soc_dapm_route sc5xx_adau1761_dapm_routes[] = {
	{ "LAUX", NULL, "In 3-4" },
	{ "RAUX", NULL, "In 3-4" },
	{ "LINP", NULL, "In 1" },
	{ "LINN", NULL, "In 1"},
	{ "RINP", NULL, "In 2" },
	{ "RINN", NULL, "In 2" },

	{ "In 1", NULL, "MICBIAS" },
	{ "In 2", NULL, "MICBIAS" },

	{ "Capless HP Out", NULL, "LHP" },
	{ "Capless HP Out", NULL, "RHP" },
	{ "Diff Out L", NULL, "LOUT" },
	{ "Diff Out R", NULL, "ROUT" },
	{ "Stereo Out", NULL, "LOUT" },
	{ "Stereo Out", NULL, "ROUT" },
};

static int sc5xx_adau1372_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int fmt, rx_mask = 0;
	unsigned int slot_width = 0;
	int ret, slots = 0;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 1: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 16;
		rx_mask = 0x1;
		break;
	case 4: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 4;
		rx_mask = 0xf;
		break;
	case 8: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 8;
		rx_mask = 0xff;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		slot_width = 16;
		break;
	case 24:
	case 32:
		slot_width = 32;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_runtime_set_dai_fmt(rtd, fmt);
	if (ret)
		return ret;

	return snd_soc_dai_set_tdm_slot(codec_dai, 0, rx_mask,
				 slots, slot_width);

}

static const struct snd_soc_ops adau1372_ops = {
	.hw_params = sc5xx_adau1372_hw_params,
};

static int __maybe_unused sc5xx_adau1372_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int sc5xx_adau1962_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int fmt, rx_mask = 0;
	unsigned int slot_width = 0;
	int ret, slots = 0;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 1: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 16;
		rx_mask = 0x1;
		break;
	case 4: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 4;
		rx_mask = 0xf;
		break;
	case 8: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 8;
		rx_mask = 0xff;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		slot_width = 16;
		break;
	case 24:
	case 32:
		slot_width = 32;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_runtime_set_dai_fmt(rtd, fmt);
	if (ret)
		return ret;

	return snd_soc_dai_set_tdm_slot(codec_dai, 0, rx_mask,
				 slots, slot_width);

}

static const struct snd_soc_ops adau1962_ops = {
	.hw_params = sc5xx_adau1962_hw_params,
};

static int __maybe_unused sc5xx_adau1962_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *component = dai->component;
	int ret =  snd_soc_component_set_sysclk(component, ADAU1962_SYSCLK,
			ADAU1962_SYSCLK_SRC_MCLK, 24576000, SND_SOC_CLOCK_IN);
	return ret;
}

static int sc5xx_adau1979_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int ret, slots = 0;
	unsigned int slot_width = 0;
	unsigned int fmt, rx_mask = 0;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 1: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 16;
		rx_mask = 0x1;
		break;
	case 4: /* TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		slots = 4;
		rx_mask = 0xf;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		slot_width = 16;
		break;
	case 24:
		slot_width = 24;
		break;
	case 32:
		slot_width = 32;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_runtime_set_dai_fmt(rtd, fmt);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, rx_mask,
					slots, slot_width);
	return ret;
}

static int sam_adau1761_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int fmt, rx_mask = 0;
	unsigned int slot_width = 0;
	int ret, slots = 0;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16:
		slot_width = 16;
		break;
	case 24:
	case 32:
		slot_width = 32;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_runtime_set_dai_fmt(rtd, fmt);
	if (ret)
		return ret;

	return snd_soc_dai_set_tdm_slot(codec_dai, 0, rx_mask,
				 slots, slot_width);
}

static int sc5xx_adau1761_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int pll_rate;
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

	ret = snd_soc_dai_set_sysclk(codec_dai, ADAU17X1_CLK_SRC_PLL, 12288000,
			SND_SOC_CLOCK_IN);
	if (ret) {
		pr_err("%s error, ret:%d\n", __func__, ret);
		return ret;
	}

	return sam_adau1761_hw_params(substream, params);
}

static const struct snd_soc_ops sc5xx_adau1761_ops = {
	.hw_params = sc5xx_adau1761_hw_params,
};

static const struct snd_soc_ops adau1979_ops = {
	.hw_params = sc5xx_adau1979_hw_params,
};

static int __maybe_unused sc5xx_adau1979_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *component = dai->component;

	return snd_soc_component_set_sysclk(component, ADAU1977_SYSCLK,
			ADAU1977_SYSCLK_SRC_MCLK, 24576000, SND_SOC_CLOCK_IN);
}

#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1372)
static struct snd_soc_dai_link_component adau1372_codec_component[] = {
	{
		.name = NULL,
		.of_node = NULL,
		.dai_name = "adau1372",
	},
};
#endif

static struct snd_soc_dai_link_component adau1962_codec_component[] = {
	{
		.name = NULL,
		.of_node = NULL,
		.dai_name = "adau1962-hifi",
	},
};

#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1979)
static struct snd_soc_dai_link_component adau1979_codec_component[] = {
	{
		.name = NULL,
		.of_node = NULL,
		.dai_name = "adau1977-hifi",
	},
};
#endif

#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1761)
static struct snd_soc_dai_link_component adau1961_codec_component[] = {
	{
		.name = NULL,
		.of_node = NULL,
		.dai_name = "adau1961-hifi",
	},
};
#endif

static struct snd_soc_dai_link_component sc5xx_platform_component[] = {
	{
		.name = "sc5xx-pcm-audio",
		.of_node = NULL,
		.dai_name = NULL,
	},
};

static struct snd_soc_dai_link_component sc5xx_cpu_component[] = {
	{
		.name = NULL,
		.of_node = NULL,
		.dai_name = NULL,
	},
};

/* Digital audio interface glue - connect codec <--> CPU */
static struct snd_soc_dai_link sc5xx_snd_soc_dai_links[] = {
#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1372)
	{
		.name = "adau1372",
		.stream_name = "ADAU1372",
		.cpus = sc5xx_cpu_component,
		.num_cpus = ARRAY_SIZE(sc5xx_cpu_component),
		.codecs = adau1372_codec_component,
		.num_codecs = ARRAY_SIZE(adau1372_codec_component),
		.platforms = sc5xx_platform_component,
		.num_platforms = ARRAY_SIZE(sc5xx_platform_component),
		.init = sc5xx_adau1372_init,
		.ops = &adau1372_ops,
	},
#endif
#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1962)
	{
		.name = "adau1962",
		.stream_name = "ADAU1962",
		.cpus = sc5xx_cpu_component,
		.num_cpus = ARRAY_SIZE(sc5xx_cpu_component),
		.codecs = adau1962_codec_component,
		.num_codecs = ARRAY_SIZE(adau1962_codec_component),
		.platforms = sc5xx_platform_component,
		.num_platforms = ARRAY_SIZE(sc5xx_platform_component),
		.init = sc5xx_adau1962_init,
		.ops = &adau1962_ops,
	},
#endif
#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1979)
	{
		.name = "adau1979",
		.stream_name = "ADAU1979",
		.cpus = sc5xx_cpu_component,
		.num_cpus = ARRAY_SIZE(sc5xx_cpu_component),
		.codecs = adau1979_codec_component,
		.num_codecs = ARRAY_SIZE(adau1979_codec_component),
		.platforms = sc5xx_platform_component,
		.num_platforms = ARRAY_SIZE(sc5xx_platform_component),
		.init = sc5xx_adau1979_init,
		.ops = &adau1979_ops,
	},
#endif
#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1761)
	{
		.name = "adau1761",
		.stream_name = "adau1761",
		.cpus = sc5xx_cpu_component,
		.num_cpus = ARRAY_SIZE(sc5xx_cpu_component),
		.codecs = adau1961_codec_component,
		.num_codecs = ARRAY_SIZE(adau1961_codec_component),
		.platforms = sc5xx_platform_component,
		.num_platforms = ARRAY_SIZE(sc5xx_platform_component),
		.ops = &sc5xx_adau1761_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
	},
#endif
};

/* ADI sc5xx audio machine driver */
static struct snd_soc_card sc5xx_snd_soc_card = {
	.name = "sc5xx-asoc-card",
	.owner = THIS_MODULE,
	.dai_link = sc5xx_snd_soc_dai_links,
	.num_links = ARRAY_SIZE(sc5xx_snd_soc_dai_links),
#ifdef CONFIG_SND_SC5XX_ADAU1761
	.dapm_widgets = sc5xx_adau1761_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sc5xx_adau1761_dapm_widgets),
	.dapm_routes = sc5xx_adau1761_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(sc5xx_adau1761_dapm_routes),
	.fully_routed = true,
#endif
};

static int sc5xx_snd_soc_probe(struct platform_device *pdev)
{
	int id = 0;
	int ret = 0;

	sc5xx_snd_soc_card.dev = &pdev->dev;

	sc5xx_cpu_component->of_node = of_parse_phandle(pdev->dev.of_node, "adi,cpu-dai", 0);

#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1372)
	sc5xx_snd_soc_dai_links[id++].codecs[0].of_node = of_parse_phandle(pdev->dev.of_node,
									"adi,codec", 0);
#endif
#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1962)
	sc5xx_snd_soc_dai_links[id++].codecs[0].of_node = of_parse_phandle(pdev->dev.of_node,
									"adi,codec", 0);
#endif
#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1979)
	sc5xx_snd_soc_dai_links[id++].codecs[0].of_node = of_parse_phandle(pdev->dev.of_node,
									"adi,codec", 1);
#endif
#if IS_ENABLED(CONFIG_SND_SC5XX_ADAU1761)
	sc5xx_snd_soc_dai_links[id++].codecs[0].of_node = of_parse_phandle(pdev->dev.of_node,
									"adi,codec", 0);
#endif

	ret = devm_snd_soc_register_card(&pdev->dev, &sc5xx_snd_soc_card);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id sc5xx_snd_soc_of_match[] = {
	{ .compatible = "adi,sc5xx-asoc-card" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc5xx_snd_soc_of_match);
#endif

static struct platform_driver sc5xx_snd_soc_driver = {
	.driver = {
		.name = "snd-sc5xx",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(sc5xx_snd_soc_of_match),
	},
	.probe = sc5xx_snd_soc_probe,
};
module_platform_driver(sc5xx_snd_soc_driver);

MODULE_DESCRIPTION("ASoC Machine driver for ADI SC5xx based boards");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
