/*
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
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/pm_runtime.h>
#include "fsl_sai.h"
#include "fsl_amix.h"

struct imx_amix {
	struct platform_device *pdev;
	struct snd_soc_card card;
	struct platform_device *amix_pdev;
	struct platform_device *out_pdev;
	struct clk *cpu_mclk;
	int num_dai;
	struct snd_soc_dai_link *dai;
	int num_dai_conf;
	struct snd_soc_codec_conf *dai_conf;
	int num_dapm_routes;
	struct snd_soc_dapm_route *dapm_routes;
};

static const u32 imx_amix_rates[] = {
	8000, 12000, 16000, 24000, 32000, 48000, 64000, 96000,
};

static const struct snd_pcm_hw_constraint_list imx_amix_rate_constraints = {
	.count = ARRAY_SIZE(imx_amix_rates),
	.list = imx_amix_rates,
};

static int imx_amix_fe_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_amix *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = rtd->card->dev;
	unsigned long clk_rate = clk_get_rate(priv->cpu_mclk);
	int ret;

	if (clk_rate % 24576000 == 0) {
		ret = snd_pcm_hw_constraint_list(runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &imx_amix_rate_constraints);
		if (ret)
			return ret;
	} else
		dev_warn(dev, "mclk may be not supported %lu\n", clk_rate);

	ret = snd_pcm_hw_constraint_minmax(runtime,
			SNDRV_PCM_HW_PARAM_CHANNELS, 1, 8);
	if (ret)
		return ret;

	return snd_pcm_hw_constraint_mask64(runtime,
			SNDRV_PCM_HW_PARAM_FORMAT, FSL_AMIX_FORMATS);
}

static int imx_amix_fe_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *be_rtd;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	be_rtd = snd_soc_get_pcm_runtime(rtd->card, "HiFi-AMIX-BE");

	return snd_soc_dai_digital_mute(be_rtd->cpu_dai, 0, substream->stream);
}

static void imx_amix_fe_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *be_rtd;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	be_rtd = snd_soc_get_pcm_runtime(rtd->card, "HiFi-AMIX-BE");

	snd_soc_dai_digital_mute(be_rtd->cpu_dai, 1, substream->stream);
}

static int imx_amix_fe_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF;
	u32 channels = params_channels(params);
	int ret, dir;

	/* For playback the AMIX is slave, and for record is master */
	fmt |= tx ? SND_SOC_DAIFMT_CBS_CFS : SND_SOC_DAIFMT_CBM_CFM;
	dir  = tx ? SND_SOC_CLOCK_OUT : SND_SOC_CLOCK_IN;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	/* Specific configurations of DAIs starts from here */
	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, FSL_SAI_CLK_MAST1, 0, dir);
	if (ret) {
		dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	/*
	 * Per datasheet, AMIX expects 8 slots and 32 bits
	 * for every slot in TDM mode.
	 */
	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, BIT(channels) - 1,
				 BIT(channels) - 1, 8, 32);
	if (ret)
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);

	return ret;
}

static int imx_amix_be_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->card->dev;
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF;
	int ret;

	/* For playback the AMIX is slave, and for record is master */
	fmt |= tx ? SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS;

	/* set AMIX DAI configuration */
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, fmt);
	if (ret)
		dev_err(dev, "failed to set AMIX DAI fmt: %d\n", ret);

	return ret;
}

static struct snd_soc_ops imx_amix_fe_ops = {
	.startup = imx_amix_fe_startup,
	.prepare = imx_amix_fe_prepare,
	.shutdown = imx_amix_fe_shutdown,
	.hw_params = imx_amix_fe_hw_params,
};

static struct snd_soc_ops imx_amix_be_ops = {
	.hw_params = imx_amix_be_hw_params,
};

static int imx_amix_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *amix_np = NULL, *out_cpu_np = NULL;
	struct platform_device *amix_pdev = NULL;
	struct platform_device *cpu_pdev;
	struct of_phandle_args args;
	struct imx_amix *priv;
	int i, num_dai, ret;
	const char *fe_name_pref = "HiFi-AMIX-FE-";
	char *dai_name;

	num_dai = of_count_phandle_with_args(np, "dais", NULL);
	if (num_dai != FSL_AMIX_MAX_DAIS) {
		dev_err(&pdev->dev, "Need 2 dais to be provided for %s\n",
			np->full_name);
		return -EINVAL;
	}

	amix_np = of_parse_phandle(np, "amix-controller", 0);
	if (!amix_np) {
		dev_err(&pdev->dev, "Missing amix-controller phandle at %s\n",
			np->full_name);
		return -EINVAL;
	}

	amix_pdev = of_find_device_by_node(amix_np);
	if (!amix_pdev) {
		dev_err(&pdev->dev, "Missing AMIX platform device for %s\n",
			np->full_name);
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* the dais + backend */
	priv->num_dai = num_dai + 1;
	priv->dai = devm_kzalloc(&pdev->dev,
			priv->num_dai * sizeof(struct snd_soc_dai_link),
			GFP_KERNEL);
	if (!priv->dai)
		return -ENOMEM;

	priv->num_dai_conf = num_dai;
	priv->dai_conf = devm_kzalloc(&pdev->dev,
			priv->num_dai_conf * sizeof(struct snd_soc_codec_conf),
			GFP_KERNEL);
	if (!priv->dai_conf)
		return -ENOMEM;

	/* 2 additional routes needed for Capture */
	priv->num_dapm_routes = num_dai + 2;
	priv->dapm_routes = devm_kzalloc(&pdev->dev,
			priv->num_dapm_routes * sizeof(struct snd_soc_dapm_route),
			GFP_KERNEL);
	if (!priv->dapm_routes)
		return -ENOMEM;

	for (i = 0; i < num_dai; i++) {
		ret = of_parse_phandle_with_args(np, "dais", NULL, i, &args);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"of_parse_phandle_with_args failed (%d)\n", ret);
			return ret;
		}

		if (i == 0)
			out_cpu_np = args.np;

		cpu_pdev = of_find_device_by_node(args.np);
		if (!cpu_pdev) {
			dev_err(&pdev->dev, "failed to find SAI platform device\n");
			return -EINVAL;
		}

		dai_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s%s",
				fe_name_pref, args.np->full_name + 1);

		dev_info(&pdev->dev, "DAI FE name:%s\n", dai_name);

		priv->dai[i].name = dai_name;
		priv->dai[i].stream_name = "HiFi-AMIX-FE";
		priv->dai[i].codec_dai_name = "snd-soc-dummy-dai";
		priv->dai[i].codec_name = "snd-soc-dummy";
		priv->dai[i].cpu_of_node = args.np;
		priv->dai[i].cpu_dai_name = dev_name(&cpu_pdev->dev);
		priv->dai[i].platform_of_node = args.np;
		priv->dai[i].dynamic = 1;
		priv->dai[i].dpcm_playback = 1;
		priv->dai[i].dpcm_capture = (i == 0 ? 1 : 0);
		priv->dai[i].ignore_pmdown_time = 1;
		priv->dai[i].ops = &imx_amix_fe_ops;

		priv->dai_conf[i].of_node = args.np;
		priv->dai_conf[i].name_prefix = dai_name;

		priv->dapm_routes[i].source = devm_kasprintf(&pdev->dev,
			GFP_KERNEL, "%s %s", dai_name, "CPU-Playback");
		priv->dapm_routes[i].sink = "AMIX-Playback";

		if (i == 0) {
			priv->dapm_routes[num_dai].source = "AMIX-Capture";
			priv->dapm_routes[num_dai].sink =
				devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s %s",
				dai_name, "CPU-Capture");
			priv->dapm_routes[num_dai+1].source = "AMIX-Playback";
			priv->dapm_routes[num_dai+1].sink = "AMIX-Capture";
		}
	}

	cpu_pdev = of_find_device_by_node(out_cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		return -EINVAL;
	}
	priv->cpu_mclk = devm_clk_get(&cpu_pdev->dev, "mclk1");
	if (IS_ERR(priv->cpu_mclk)) {
		ret = PTR_ERR(priv->cpu_mclk);
		dev_err(&cpu_pdev->dev, "failed to get DAI mclk1: %d\n", ret);
		return -EINVAL;
	}

	/* Add AMIX Backend */
	priv->dai[num_dai].name = "HiFi-AMIX-BE";
	priv->dai[num_dai].stream_name = "HiFi-AMIX-BE";
	priv->dai[num_dai].codec_dai_name = "snd-soc-dummy-dai";
	priv->dai[num_dai].codec_name = "snd-soc-dummy";
	priv->dai[num_dai].cpu_of_node = amix_np;
	priv->dai[num_dai].platform_name = "snd-soc-dummy";
	priv->dai[num_dai].no_pcm = 1;
	priv->dai[num_dai].dpcm_playback = 1;
	priv->dai[num_dai].dpcm_capture = 1;
	priv->dai[num_dai].ignore_pmdown_time = 1;
	priv->dai[num_dai].ops = &imx_amix_be_ops;

	priv->amix_pdev = amix_pdev;
	priv->out_pdev  = cpu_pdev;

	priv->card.dai_link = priv->dai;
	priv->card.num_links = priv->num_dai;
	priv->card.codec_conf = priv->dai_conf;
	priv->card.num_configs = priv->num_dai_conf;
	priv->card.dapm_routes = priv->dapm_routes;
	priv->card.num_dapm_routes = priv->num_dapm_routes;
	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;

	ret = snd_soc_of_parse_card_name(&priv->card, "model");
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_of_parse_card_name failed (%d)\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, &priv->card);
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		return ret;
	}

	return ret;
}

static const struct of_device_id imx_amix_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-amix", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_amix_dt_ids);

static struct platform_driver imx_amix_driver = {
	.probe = imx_amix_probe,
	.driver = {
		.name = "imx-amix",
		.of_match_table = imx_amix_dt_ids,
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(imx_amix_driver);

MODULE_DESCRIPTION("NXP AMIX ASoC machine driver");
MODULE_AUTHOR("Viorel Suman <viorel.suman@nxp.com>");
MODULE_ALIAS("platform:imx-amix");
MODULE_LICENSE("GPL v2");
