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
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>

struct imx_ak4458_data {
	struct snd_soc_card card;
	int num_codec_conf;
	struct snd_soc_codec_conf *codec_conf;
};

static struct snd_soc_dapm_widget imx_ak4458_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out", NULL),
};

static int imx_aif_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	unsigned int channels = params_channels(params);
	unsigned int fmt;
	int ret;
	int i;

	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	for (i = 0; i < rtd->num_codecs; i++) {
		struct snd_soc_dai *codec_dai = rtd->codec_dais[i];

		ret = snd_soc_dai_set_fmt(codec_dai, fmt);
		if (ret) {
			dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
			return ret;
		}
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       BIT(channels) - 1, BIT(channels) - 1,
				       2, 32);
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops imx_aif_ops = {
	.hw_params = imx_aif_hw_params,
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

	priv->num_codec_conf = 2;
	priv->codec_conf = devm_kzalloc(&pdev->dev,
		priv->num_codec_conf * sizeof(struct snd_soc_codec_conf),
		GFP_KERNEL);
	if (!priv->codec_conf)
		return -ENOMEM;


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
