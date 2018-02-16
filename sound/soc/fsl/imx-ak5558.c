/* i.MX AK5558 audio support
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
#include <sound/pcm.h>
#include <sound/soc-dapm.h>

#include "fsl_sai.h"
#include "../codecs/ak5558.h"


struct imx_ak5558_data {
	struct snd_soc_card card;
	bool tdm_mode;
	unsigned long freq;
	unsigned long slots;
	unsigned long slot_width;
};

/*
 * imx_ack5558_fs_mul - sampling frequency multiplier
 *
 * min <= fs <= max, MCLK = mul * LRCK
 */
struct imx_ak5558_fs_mul {
	unsigned int min;
	unsigned int max;
	unsigned int mul;
};

/*
 * Auto MCLK selection based on LRCK for Normal Mode
 * (Table 4 from datasheet)
 */
static const struct imx_ak5558_fs_mul fs_mul[] = {
	{ .min = 8000,   .max = 32000,  .mul = 1024 },
	{ .min = 48000,  .max = 48000,  .mul = 512  },
	{ .min = 96000,  .max = 96000,  .mul = 256  },
	{ .min = 192000, .max = 192000, .mul = 128  },
	{ .min = 384000, .max = 384000, .mul = 64  },
	{ .min = 768000, .max = 768000, .mul = 32   },
};

static struct snd_soc_dapm_widget imx_ak5558_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const u32 ak5558_rates[] = {
	8000, 16000, 32000,
	48000, 96000, 192000,
	384000, 768000,
};

static const u32 ak5558_tdm_rates[] = {
	8000, 16000, 32000,
	48000, 96000, 192000,
};

static const u32 ak5558_channels[] = {
	1, 2, 4, 6, 8,
};

static unsigned long ak5558_get_mclk_rate(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_ak5558_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int rate = params_rate(params);
	unsigned int freq = data->freq;
	int i;

	for (i = 0; i < ARRAY_SIZE(fs_mul); i++) {
		if (rate < fs_mul[i].min || rate > fs_mul[i].max)
			continue;
		freq = rate * fs_mul[i].mul;
		break;
	}

	return freq;
}

static int imx_aif_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	struct imx_ak5558_data *data = snd_soc_card_get_drvdata(card);
	unsigned int channels = params_channels(params);
	unsigned long mclk_freq;
	unsigned int fmt;
	int ret;

	if (data->tdm_mode)
		fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;
	else
		fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

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

	if (data->tdm_mode) {
		ret = snd_soc_dai_set_tdm_slot(cpu_dai,
			       BIT(channels) - 1, BIT(channels) - 1,
			       8, 32);
		if (ret) {
			dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_tdm_slot(codec_dai,
			       BIT(channels) - 1, BIT(channels) - 1,
			       8, 32);
		if (ret) {
			dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
			return ret;
		}
	} else {
		ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       BIT(channels) - 1, BIT(channels) - 1,
				       2, params_physical_width(params));
		if (ret) {
			dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
			return ret;
		}
	}

	mclk_freq = ak5558_get_mclk_rate(substream, params);
	ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1, mclk_freq,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0)
		dev_err(dev, "failed to set cpu_dai mclk1 rate %lu\n",
			mclk_freq);

	return ret;
}

static int imx_aif_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_ak5558_data *data = snd_soc_card_get_drvdata(card);

	static struct snd_pcm_hw_constraint_list constraint_rates;
	static struct snd_pcm_hw_constraint_list constraint_channels;
	int ret;

	if (data->tdm_mode) {
		constraint_rates.list = ak5558_tdm_rates;
		constraint_rates.count = ARRAY_SIZE(ak5558_tdm_rates);
	} else {
		constraint_rates.list = ak5558_rates;
		constraint_rates.count = ARRAY_SIZE(ak5558_rates);
	}

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
						&constraint_rates);
	if (ret)
		return ret;

	constraint_channels.list = ak5558_channels;
	constraint_channels.count = ARRAY_SIZE(ak5558_channels);
	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
							&constraint_channels);
	if (ret)
		return ret;

	return 0;
}

static struct snd_soc_ops imx_aif_ops = {
	.hw_params = imx_aif_hw_params,
	.startup = imx_aif_startup,
};

static struct snd_soc_dai_link imx_ak5558_dai = {
	.name = "ak5558",
	.stream_name = "Audio",
	.codec_dai_name = "ak5558-aif",
	.ops = &imx_aif_ops,
	.capture_only = 1,
};

static int imx_ak5558_probe(struct platform_device *pdev)
{
	struct imx_ak5558_data *priv;
	struct device_node *cpu_np, *codec_np = NULL;
	struct platform_device *cpu_pdev;
	struct clk *mclk;
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

	if (of_find_property(pdev->dev.of_node, "fsl,tdm", NULL))
		priv->tdm_mode = true;

	imx_ak5558_dai.codec_of_node = codec_np;
	imx_ak5558_dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	imx_ak5558_dai.platform_of_node = cpu_np;
	imx_ak5558_dai.capture_only = 1;

	priv->card.dai_link = &imx_ak5558_dai;
	priv->card.num_links = 1;
	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;
	priv->card.dapm_widgets = imx_ak5558_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(imx_ak5558_dapm_widgets);

	mclk = devm_clk_get(&cpu_pdev->dev, "mclk1");
	if (IS_ERR(mclk)) {
		ret = PTR_ERR(mclk);
		dev_err(&pdev->dev, "failed to get DAI mclk1: %d\n", ret);
		return -EINVAL;
	}

	priv->freq = clk_get_rate(mclk);

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

static const struct of_device_id imx_ak5558_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-ak5558", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx_ak5558_dt_ids);

static struct platform_driver imx_ak5558_driver = {
	.driver = {
		.name = "imx-ak5558",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_ak5558_dt_ids,
	},
	.probe = imx_ak5558_probe,
};
module_platform_driver(imx_ak5558_driver);

MODULE_AUTHOR("Mihai Serban <mihai.serban@nxp.com>");
MODULE_DESCRIPTION("Freescale i.MX AK5558 ASoC machine driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ak5558");
