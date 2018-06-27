/* i.MX AK4458 audio support
 *
 * Copyright 2017 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>

#include "../codecs/ak4497.h"
#include "fsl_sai.h"

struct imx_ak4497_data {
	struct snd_soc_card card;
	bool one2one_ratio;
};

static struct snd_soc_dapm_widget imx_ak4497_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out", NULL),
};

static const struct imx_ak4497_fs_mul {
	unsigned int min;
	unsigned int max;
	unsigned int mul;
} fs_mul[] = {
	/**
	 * Table 7      - mapping multiplier and speed mode
	 * Tables 8 & 9 - mapping speed mode and LRCK fs
	 */
	{ .min = 8000,   .max = 32000,  .mul = 1024 }, /* Normal, <= 32kHz */
	{ .min = 44100,  .max = 48000,  .mul = 512  }, /* Normal */
	{ .min = 88200,  .max = 96000,  .mul = 256  }, /* Double */
	{ .min = 176400, .max = 192000, .mul = 128  }, /* Quad */
	{ .min = 352800, .max = 384000, .mul = 2*64 }, /* Oct */
	{ .min = 705600, .max = 768000, .mul = 2*32 }, /* Hex */
};

static bool imx_ak4497_is_dsd(struct snd_pcm_hw_params *params)
{
	snd_pcm_format_t format = params_format(params);

	switch (format) {
	case SNDRV_PCM_FORMAT_DSD_U8:
	case SNDRV_PCM_FORMAT_DSD_U16_LE:
	case SNDRV_PCM_FORMAT_DSD_U16_BE:
	case SNDRV_PCM_FORMAT_DSD_U32_LE:
	case SNDRV_PCM_FORMAT_DSD_U32_BE:
		return true;
	default:
		return false;
	}
}

static unsigned long imx_ak4497_compute_freq(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	unsigned int rate = params_rate(params);
	int i;

	/* Find the appropriate MCLK freq */
	for (i = 0; i < ARRAY_SIZE(fs_mul); i++) {
		if (rate >= fs_mul[i].min && rate <= fs_mul[i].max)
			return rate * fs_mul[i].mul;
	}

	/* Let DAI manage MCLK frequency */
	return 0;
}

static int imx_aif_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	struct imx_ak4497_data *priv = snd_soc_card_get_drvdata(card);
	unsigned int channels = params_channels(params);
	unsigned int fmt = SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS;
	unsigned long freq = imx_ak4497_compute_freq(substream, params);
	bool is_dsd = imx_ak4497_is_dsd(params);
	int ret;

	fmt |= (is_dsd ? SND_SOC_DAIFMT_PDM : SND_SOC_DAIFMT_I2S);

	if (is_dsd && freq > 22579200 && priv->one2one_ratio)
		freq = 22579200;

	ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1, freq,
					SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(dev, "failed to set cpu dai mclk1 rate(%lu): %d\n",
			freq, ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	if (is_dsd)
		ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       0x1, 0x1,
				       1, params_width(params));
	else
		ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       BIT(channels) - 1, BIT(channels) - 1,
				       2, params_physical_width(params));
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	return ret;
}

static const u32 support_rates[] = {
	8000, 11025, 16000, 22050,
	32000, 44100, 48000, 88200,
	96000, 176400, 192000, 352800,
	384000, 705600, 768000,
};

static int imx_aif_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;
	static struct snd_pcm_hw_constraint_list constraint_rates;

	constraint_rates.list = support_rates;
	constraint_rates.count = ARRAY_SIZE(support_rates);

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
						&constraint_rates);
	if (ret)
		return ret;

	return ret;
}

static struct snd_soc_ops imx_aif_ops = {
	.startup   = imx_aif_startup,
	.hw_params = imx_aif_hw_params,
};

static struct snd_soc_dai_link imx_ak4497_dai = {
	.name = "ak4497",
	.stream_name = "Audio",
	.codec_dai_name = "ak4497-aif",
	.ops = &imx_aif_ops,
	.playback_only = 1,
};

static int imx_ak4497_probe(struct platform_device *pdev)
{
	struct imx_ak4497_data *priv;
	struct device_node *cpu_np, *codec_np = NULL;
	struct platform_device *cpu_pdev;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "audio dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "audio codec phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	imx_ak4497_dai.codec_of_node = codec_np;
	imx_ak4497_dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	imx_ak4497_dai.platform_of_node = cpu_np;
	imx_ak4497_dai.playback_only = 1;

	priv->card.dai_link = &imx_ak4497_dai;
	priv->card.num_links = 1;
	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;
	priv->card.dapm_widgets = imx_ak4497_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(imx_ak4497_dapm_widgets);
	priv->one2one_ratio = !of_device_is_compatible(pdev->dev.of_node,
					"fsl,imx-audio-ak4497-mq");

	ret = snd_soc_of_parse_card_name(&priv->card, "model");
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

static const struct of_device_id imx_ak4497_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-ak4497", },
	{ .compatible = "fsl,imx-audio-ak4497-mq", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx_ak4497_dt_ids);

static struct platform_driver imx_ak4497_driver = {
	.driver = {
		.name = "imx-ak4497",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_ak4497_dt_ids,
	},
	.probe = imx_ak4497_probe,
};
module_platform_driver(imx_ak4497_driver);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@nxp.com>");
MODULE_DESCRIPTION("Freescale i.MX AK4497 ASoC machine driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ak4497");
