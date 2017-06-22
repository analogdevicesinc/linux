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
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>

struct imx_priv {
	struct platform_device *pdev;
	struct snd_soc_card card;
	struct clk *codec_clk;
	unsigned int clk_frequency;
};

static const struct snd_soc_dapm_widget imx_wm8524_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	unsigned int fmt;
	int ret = 0;

	fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2,
					params_physical_width(params));
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_hifi_hw_params,
};

static struct snd_soc_dai_link imx_wm8524_dai[] = {
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.codec_dai_name = "wm8524-hifi",
		.ops = &imx_hifi_ops,
	},
};

static int imx_wm8524_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd = list_first_entry(
		&card->rtd_list, struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	priv->clk_frequency = clk_get_rate(priv->codec_clk);

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, priv->clk_frequency,
							SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_wm8524_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np = NULL;
	struct platform_device *cpu_pdev;
	struct imx_priv *priv;
	struct platform_device *codec_pdev = NULL;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_pdev = of_find_device_by_node(codec_np);
	if (!codec_pdev || !codec_pdev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	priv->codec_clk = devm_clk_get(&codec_pdev->dev, "mclk");
	if (IS_ERR(priv->codec_clk)) {
		ret = PTR_ERR(priv->codec_clk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	priv->card.dai_link = imx_wm8524_dai;

	imx_wm8524_dai[0].codec_of_node	= codec_np;
	imx_wm8524_dai[0].cpu_dai_name = dev_name(&cpu_pdev->dev);
	imx_wm8524_dai[0].platform_of_node = cpu_np;
	imx_wm8524_dai[0].playback_only	= 1;

	priv->card.late_probe = imx_wm8524_late_probe;
	priv->card.num_links = 1;
	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;
	priv->card.dapm_widgets = imx_wm8524_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8524_dapm_widgets);

	ret = snd_soc_of_parse_card_name(&priv->card, "model");
	if (ret)
		goto fail;

	ret = snd_soc_of_parse_audio_routing(&priv->card, "audio-routing");
	if (ret)
		goto fail;

	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	ret = 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static const struct of_device_id imx_wm8524_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8524", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8524_dt_ids);

static struct platform_driver imx_wm8524_driver = {
	.driver = {
		.name = "imx-wm8524",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8524_dt_ids,
	},
	.probe = imx_wm8524_probe,
};
module_platform_driver(imx_wm8524_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8524 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8524");
