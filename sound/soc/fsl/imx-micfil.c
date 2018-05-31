/*
 * Copyright 2018 NXP
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
#include <linux/pinctrl/consumer.h>
#include "fsl_micfil.h"

#define RX 0
#define TX 1

struct imx_micfil_data {
	char name[32];
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
};

static int imx_micfil_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	static struct snd_pcm_hw_constraint_list constraint_rates;
	int ret;
	static u32 support_rates[] = {11025, 16000, 22050,
				      32000, 44100, 48000,};

	constraint_rates.list = support_rates;
	constraint_rates.count = ARRAY_SIZE(support_rates);

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraint_rates);
	if (ret)
		return ret;

	return 0;
}

static int imx_micfil_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct device *dev = rtd->card->dev;
	unsigned int fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;
	int ret, dir;

	/* For playback the XTOR is slave, and for record is master */
	fmt |= tx ? SND_SOC_DAIFMT_CBS_CFS : SND_SOC_DAIFMT_CBM_CFM;
	dir = tx ? SND_SOC_CLOCK_OUT : SND_SOC_CLOCK_IN;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	/* Specific configurations of DAIs starts from here */
	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, 0,
				     0, dir);
	if (ret) {
		dev_err(dev,
			"%s: failed to set cpu sysclk: %d\n", __func__,
			ret);
		return ret;
	}

	return 0;
}

struct snd_soc_ops imx_micfil_ops = {
	.startup = imx_micfil_startup,
	.hw_params = imx_micfil_hw_params,
};

static int imx_micfil_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np;
	struct platform_device *cpu_pdev;
	struct imx_micfil_data *data;
	int ret;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	strncpy(data->name, cpu_np->name, sizeof(data->name) - 1);

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find MICFIL platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data->dai.name = "micfil hifi";
	data->dai.stream_name = "micfil hifi";
	data->dai.codec_dai_name = "snd-soc-dummy-dai";
	data->dai.codec_name = "snd-soc-dummy";
	data->dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platform_of_node = cpu_np;
	data->dai.playback_only = false;
	data->dai.ops = &imx_micfil_ops;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
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
	return ret;
}

static const struct of_device_id imx_micfil_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-micfil", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_micfil_dt_ids);

static struct platform_driver imx_micfil_driver = {
	.driver = {
		.name = "imx-micfil",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_micfil_dt_ids,
	},
	.probe = imx_micfil_probe,
};
module_platform_driver(imx_micfil_driver);

MODULE_AUTHOR("Cosmin-Gabriel Samoila <cosmin.samoila@nxp.com>");
MODULE_DESCRIPTION("NXP Micfil ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-micfil");
