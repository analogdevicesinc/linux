// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices digital audio interface driver
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include <sound/sc5xx-dai.h>

#include "sc5xx-sport.h"

struct sport_params param;

static int sc5xx_dai_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(cpu_dai);
	struct device *dev = &sport->pdev->dev;
	int ret = 0;

	param.spctl &= ~(SPORT_CTL_OPMODE | SPORT_CTL_CKRE | SPORT_CTL_FSR
			| SPORT_CTL_LFS | SPORT_CTL_LAFS);
	param.spmctl &= ~(SPORT_MCTL_MCE);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		param.spctl |= SPORT_CTL_OPMODE | SPORT_CTL_CKRE
			| SPORT_CTL_LFS;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		param.spctl |= SPORT_CTL_FSR;
		param.spmctl |= SPORT_MCTL_MCE | SPORT_MCTL_MCPDE
			| (0x10 & SPORT_MCTL_MFD);
		param.spcs0 = 0xff;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		param.spctl |= SPORT_CTL_OPMODE | SPORT_CTL_LFS
			| SPORT_CTL_LAFS;
		break;
	default:
		dev_err(dev, "%s: Unknown DAI format type\n", __func__);
		ret = -EINVAL;
		break;
	}

	param.spctl &= ~(SPORT_CTL_ICLK | SPORT_CTL_IFS);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		ret = -ENOTSUPP;
		break;
	default:
		dev_err(dev, "%s: Unknown DAI master type\n", __func__);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sc5xx_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &sport->pdev->dev;
	int ret = 0;

	param.spctl &= ~SPORT_CTL_SLEN;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		param.spctl |= 0x70;
		sport->wdsize = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		param.spctl |= 0xf0;
		sport->wdsize = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		param.spctl |= 0x170;
		sport->wdsize = 3;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		param.spctl |= 0x1f0;
		sport->wdsize = 4;
		break;
	}

	/* set window size in SPORT_MCTL register */
	param.spmctl &= ~SPORT_MCTL_WSIZE;
	if (param.spmctl && SPORT_MCTL_MCE)
		param.spmctl |= (((params_channels(params) - 1) << 8)
					& SPORT_MCTL_WSIZE);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sport->tx_hw_params = *params;
		ret = sport_set_tx_params(sport, &param);
		if (ret) {
			dev_err(dev, "SPORT tx is busy!\n");
			return ret;
		}
	} else {
		sport->rx_hw_params = *params;
		ret = sport_set_rx_params(sport, &param);
		if (ret) {
			dev_err(dev, "SPORT rx is busy!\n");
			return ret;
		}
	}
	return 0;
}

#ifdef CONFIG_PM
static int sc5xx_dai_suspend(struct snd_soc_component *component)
{
	struct sport_device *sport = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai;
	int stream;

	for_each_component_dais(component, dai) {
		for_each_pcm_streams(stream) {
			if (snd_soc_dai_stream_active(dai, stream)) {
				if (stream == SNDRV_PCM_STREAM_CAPTURE)
					sport_rx_stop(sport);
				else if (stream == SNDRV_PCM_STREAM_PLAYBACK)
					sport_tx_stop(sport);
			}
		}
	}

	return 0;
}

static int sc5xx_dai_resume(struct snd_soc_component *component)
{
	struct sport_device *sport = snd_soc_component_get_drvdata(component);
	struct device *dev = &sport->pdev->dev;
	int ret;

	ret = sport_set_tx_params(sport, &param);
	if (ret) {
		dev_err(dev, "SPORT tx is busy!\n");
		return ret;
	}
	ret = sport_set_rx_params(sport, &param);
	if (ret) {
		dev_err(dev, "SPORT rx is busy!\n");
		return ret;
	}

	return 0;
}
#else
#define sc5xx_dai_suspend NULL
#define sc5xx_dai_resume NULL
#endif

#define SC5XX_DAI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | \
		SNDRV_PCM_RATE_192000)

#define SC5XX_DAI_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops sc5xx_i2s_dai_ops = {
	.hw_params	= sc5xx_dai_hw_params,
	.set_fmt	= sc5xx_dai_set_dai_fmt,
};

static struct snd_soc_dai_driver sc5xx_i2s_dai = {
	.name = "sc5xx",
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
		.rates = SC5XX_DAI_RATES,
		.formats = SC5XX_DAI_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 4,
		.rates = SC5XX_DAI_RATES,
		.formats = SC5XX_DAI_FORMATS,
	},
	.ops = &sc5xx_i2s_dai_ops,
};

static const struct snd_soc_component_driver sc5xx_dai_component = {
	.name		= "sc5xx-i2s",
	.suspend = sc5xx_dai_suspend,
	.resume = sc5xx_dai_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id sc5xx_audio_of_match[] = {
	{
		.compatible = "adi,sc5xx-i2s-dai",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sc5xx_audio_of_match);
#endif

static int sc5xx_dai_probe(struct platform_device *pdev)
{
	struct sport_device *sport;
	struct device *dev = &pdev->dev;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, "sclk");
	if (IS_ERR(clk)) {
		dev_err(dev, "Missing clock node `sclk` for i2s\n");
		return PTR_ERR(clk);
	}

	sport = sport_create(pdev);
	if (IS_ERR(sport))
		return PTR_ERR(sport);

	sport->clk = clk;
	clk_prepare_enable(clk);

	/* register with the ASoC layers */
	ret = devm_snd_soc_register_component(dev, &sc5xx_dai_component,
					 &sc5xx_i2s_dai, 1);
	if (ret)
		goto cleanup;

	platform_set_drvdata(pdev, sport);
	return 0;

cleanup:
	sport_delete(sport);
	clk_disable_unprepare(clk);
	return ret;
}

static void sc5xx_dai_remove(struct platform_device *pdev)
{
	struct sport_device *sport = platform_get_drvdata(pdev);

	sport_delete(sport);
	clk_disable_unprepare(sport->clk);
}

static struct platform_driver sc5xx_i2s_dai_driver = {
	.probe  = sc5xx_dai_probe,
	.remove = sc5xx_dai_remove,
	.driver = {
		.name = "sc5xx-i2s-dai",
		.of_match_table = of_match_ptr(sc5xx_audio_of_match),
	},
};

module_platform_driver(sc5xx_i2s_dai_driver);

MODULE_DESCRIPTION("Analog Devices SC5XX I2S DAI driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
