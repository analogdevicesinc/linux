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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>

#include "fsl_sai.h"
#include "fsl_dsd.h"

struct imx_ak4458_data {
	struct snd_soc_card card;
	int num_codec_conf;
	struct snd_soc_codec_conf *codec_conf;
	bool tdm_mode;
	int pdn_gpio;
	unsigned int slots;
	unsigned int slot_width;
	bool one2one_ratio;
};

static struct snd_soc_dapm_widget imx_ak4458_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out", NULL),
};

/**
 * Tables 3 & 4 - mapping LRCK fs and frame width
 */
static const struct imx_ak4458_fs_map {
	unsigned int rmin;
	unsigned int rmax;
	unsigned int wmin;
	unsigned int wmax;
} fs_map[] = {
	/* Normal, < 32kHz */
	{ .rmin = 8000,   .rmax = 24000,  .wmin = 1024, .wmax = 1024, },
	/* Normal, 32kHz */
	{ .rmin = 32000,  .rmax = 32000,  .wmin = 256,  .wmax = 1024, },
	/* Normal */
	{ .rmin = 44100,  .rmax = 48000,  .wmin = 256,  .wmax = 768,  },
	/* Double */
	{ .rmin = 88200,  .rmax = 96000,  .wmin = 256,  .wmax = 512,  },
	/* Quad */
	{ .rmin = 176400, .rmax = 192000, .wmin = 128,  .wmax = 256,  },
	/* Oct */
	{ .rmin = 352800, .rmax = 384000, .wmin = 32,   .wmax = 128,  },
	/* Hex */
	{ .rmin = 705600, .rmax = 768000, .wmin = 16,   .wmax = 64,   },
};

static const struct imx_ak4458_fs_mul {
	unsigned int min;
	unsigned int max;
	unsigned int mul;
} fs_mul_tdm[] = {
	/*
	 * Table 13	- Audio Interface Format
	 * For TDM mode, MCLK should is set to
	 * obtained from 2 * slots * slot_width
	 */
	{ .min = 128,	.max = 128,	.mul = 256  }, /* TDM128 */
	{ .min = 256,	.max = 256,	.mul = 512  }, /* TDM256 */
	{ .min = 512,	.max = 512,	.mul = 1024  }, /* TDM512 */
};

static const u32 ak4458_rates[] = {
	8000, 11025, 16000, 22050,
	32000, 44100, 48000, 88200,
	96000, 176400, 192000, 352800,
	384000, 705600, 768000,
};

static const u32 ak4458_rates_tdm[] = {
	8000, 16000, 32000,
	48000, 96000,
};

static const u32 ak4458_channels[] = {
	1, 2, 4, 6, 8, 10, 12, 14, 16,
};

static const u32 ak4458_channels_tdm[] = {
	1, 2, 3, 4, 5, 6, 7, 8,
};

static unsigned long ak4458_get_mclk_rate(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_ak4458_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int rate = params_rate(params);
	unsigned int width = data->slots * data->slot_width;
	int i, mode;

	if (data->tdm_mode) {
		/* can be 128, 256 or 512 */
		mode = data->slots * data->slot_width;

		for (i = 0; i < ARRAY_SIZE(fs_mul_tdm); i++) {
			/* min = max = slots * slots_width */
			if (mode != fs_mul_tdm[i].min)
				continue;
			return rate * fs_mul_tdm[i].mul;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(fs_map); i++) {
			if (rate >= fs_map[i].rmin && rate <= fs_map[i].rmax) {
				width = max(width, fs_map[i].wmin);
				width = min(width, fs_map[i].wmax);

				/* Adjust SAI bclk:mclk ratio */
				width *= data->one2one_ratio ? 1 : 2;

				return rate * width;
			}
		}
	}

	/* Let DAI manage clk frequency by default */
	return 0;
}

static int imx_aif_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	struct imx_ak4458_data *data = snd_soc_card_get_drvdata(card);
	unsigned int channels = params_channels(params);
	unsigned int fmt = SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS;
	unsigned long mclk_freq;
	bool is_dsd = fsl_is_dsd(params);
	int ret, i;

	if (is_dsd) {
		channels = 1;
		data->slots = 1;
		data->slot_width = params_width(params);
		fmt |= SND_SOC_DAIFMT_PDM;
	} else if (data->tdm_mode) {
		data->slots = 8;
		data->slot_width = 32;
		fmt |= SND_SOC_DAIFMT_DSP_B;
	} else {
		data->slots = 2;
		data->slot_width = params_physical_width(params);
		fmt |= SND_SOC_DAIFMT_I2S;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       BIT(channels) - 1, BIT(channels) - 1,
				       data->slots, data->slot_width);
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	for (i = 0; i < rtd->num_codecs; i++) {
		struct snd_soc_dai *codec_dai = rtd->codec_dais[i];

		ret = snd_soc_dai_set_fmt(codec_dai, fmt);
		if (ret) {
			dev_err(dev, "failed to set codec dai[%d] fmt: %d\n",
					i, ret);
			return ret;
		}
		ret = snd_soc_dai_set_tdm_slot(codec_dai,
				       BIT(channels) - 1, BIT(channels) - 1,
				       data->slots, data->slot_width);
		if (ret) {
			dev_err(dev, "failed to set codec dai[%d] tdm slot: %d\n",
					i, ret);
			return ret;
		}
	}

	/* set MCLK freq */
	mclk_freq = ak4458_get_mclk_rate(substream, params);
	if (is_dsd)
		mclk_freq = 22579200;
	ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1, mclk_freq,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0)
		dev_err(dev, "failed to set cpui dai mclk1 rate (%lu): %d\n",
			mclk_freq, ret);
	return ret;
}

static int imx_aif_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_ak4458_data *data = snd_soc_card_get_drvdata(card);
	static struct snd_pcm_hw_constraint_list constraint_rates;
	static struct snd_pcm_hw_constraint_list constraint_channels;
	int ret;

	if (data->tdm_mode) {
		constraint_channels.list = ak4458_channels_tdm;
		constraint_channels.count = ARRAY_SIZE(ak4458_channels_tdm);
		constraint_rates.list = ak4458_rates_tdm;
		constraint_rates.count = ARRAY_SIZE(ak4458_rates_tdm);
	} else {
		constraint_channels.list = ak4458_channels;
		constraint_channels.count = ARRAY_SIZE(ak4458_channels);
		constraint_rates.list = ak4458_rates;
		constraint_rates.count = ARRAY_SIZE(ak4458_rates);
	}

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
						&constraint_channels);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
						&constraint_rates);
	if (ret)
		return ret;

	return 0;
}

static struct snd_soc_ops imx_aif_ops = {
	.hw_params = imx_aif_hw_params,
	.startup = imx_aif_startup,
};

static struct snd_soc_dai_link_component ak4458_codecs[] = {
	{
		/* Playback */
		.dai_name = "ak4458-aif",
	},
	{
		/* Capture */
		.dai_name = "ak4458-aif",
	},
};

static struct snd_soc_dai_link imx_ak4458_dai = {
	.name = "ak4458",
	.stream_name = "Audio",
	.codecs = ak4458_codecs,
	.num_codecs = 2,
	.ignore_pmdown_time = 1,
	.ops = &imx_aif_ops,
	.playback_only = 1,
};

static int imx_ak4458_probe(struct platform_device *pdev)
{
	struct imx_ak4458_data *priv;
	struct device_node *cpu_np, *codec_np_0 = NULL, *codec_np_1 = NULL;
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

	codec_np_0 = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np_0) {
		dev_err(&pdev->dev, "audio codec phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np_1 = of_parse_phandle(pdev->dev.of_node, "audio-codec", 1);
	if (!codec_np_1) {
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

	priv->num_codec_conf = 2;
	priv->codec_conf = devm_kzalloc(&pdev->dev,
		priv->num_codec_conf * sizeof(struct snd_soc_codec_conf),
		GFP_KERNEL);
	if (!priv->codec_conf) {
		ret = -ENOMEM;
		goto fail;
	}

	priv->codec_conf[0].name_prefix = "0";
	priv->codec_conf[0].of_node = codec_np_0;
	priv->codec_conf[1].name_prefix = "1";
	priv->codec_conf[1].of_node = codec_np_1;

	ak4458_codecs[0].of_node = codec_np_0;
	ak4458_codecs[1].of_node = codec_np_1;

	imx_ak4458_dai.cpu_dai_name  = dev_name(&cpu_pdev->dev);
	imx_ak4458_dai.platform_of_node = cpu_np;

	priv->card.num_links = 1;
	priv->card.dai_link = &imx_ak4458_dai;
	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;
	priv->card.dapm_widgets = imx_ak4458_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(imx_ak4458_dapm_widgets);
	priv->card.codec_conf = priv->codec_conf;
	priv->card.num_configs = priv->num_codec_conf;
	priv->one2one_ratio = !of_device_is_compatible(pdev->dev.of_node,
					"fsl,imx-audio-ak4458-mq");

	priv->pdn_gpio = of_get_named_gpio(pdev->dev.of_node, "ak4458,pdn-gpio", 0);
	if (gpio_is_valid(priv->pdn_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, priv->pdn_gpio,
				GPIOF_OUT_INIT_LOW, "ak4458,pdn");
		if (ret) {
			dev_err(&pdev->dev, "unable to get pdn gpio\n");
			goto fail;
		}

		gpio_set_value_cansleep(priv->pdn_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(priv->pdn_gpio, 1);
		usleep_range(1000, 2000);
	}

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
	if (codec_np_0)
		of_node_put(codec_np_0);
	if (codec_np_1)
		of_node_put(codec_np_1);

	return ret;
}

static const struct of_device_id imx_ak4458_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-ak4458", },
	{ .compatible = "fsl,imx-audio-ak4458-mq", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx_ak4458_dt_ids);

static struct platform_driver imx_ak4458_driver = {
	.driver = {
		.name = "imx-ak4458",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_ak4458_dt_ids,
	},
	.probe = imx_ak4458_probe,
};
module_platform_driver(imx_ak4458_driver);

MODULE_AUTHOR("Mihai Serban <mihai.serban@nxp.com>");
MODULE_DESCRIPTION("Freescale i.MX AK4458 ASoC machine driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ak4458");
