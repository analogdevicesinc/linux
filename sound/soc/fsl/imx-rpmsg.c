/*
 * Copyright (C) 2017 NXP
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
#include "fsl_rpmsg_i2s.h"

struct imx_rpmsg_data {
	struct snd_soc_dai_link dai[1];
	struct snd_soc_card card;
};

static int imx_rpmsg_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np;
	struct platform_device *cpu_pdev;
	struct imx_rpmsg_data *data;
	struct fsl_rpmsg_i2s         *rpmsg_i2s;
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

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find rpmsg platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	rpmsg_i2s = platform_get_drvdata(cpu_pdev);

	data->dai[0].name = "rpmsg hifi";
	data->dai[0].stream_name = "rpmsg hifi";
	if (rpmsg_i2s->codec_wm8960) {
		data->dai[0].codec_dai_name = "rpmsg-wm8960-hifi";
		data->dai[0].codec_name = "rpmsg-audio-codec-wm8960";
	} else {
		data->dai[0].codec_dai_name = "snd-soc-dummy-dai";
		data->dai[0].codec_name = "snd-soc-dummy";
	}
	data->dai[0].cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai[0].platform_of_node = cpu_np;
	data->dai[0].playback_only = true;
	data->dai[0].capture_only = true;
	data->dai[0].dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;
	data->card.num_links = 1;
	data->card.dai_link = data->dai;

	if (of_property_read_bool(pdev->dev.of_node, "rpmsg-out"))
		data->dai[0].capture_only = false;

	if (of_property_read_bool(pdev->dev.of_node, "rpmsg-in"))
		data->dai[0].playback_only = false;

	if (data->dai[0].playback_only && data->dai[0].capture_only) {
		dev_err(&pdev->dev, "no enabled rpmsg DAI link\n");
		ret = -EINVAL;
		goto fail;
	}

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

static const struct of_device_id imx_rpmsg_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-rpmsg", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_dt_ids);

static struct platform_driver imx_rpmsg_driver = {
	.driver = {
		.name = "imx-audio-rpmsg",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_rpmsg_dt_ids,
	},
	.probe = imx_rpmsg_probe,
};
module_platform_driver(imx_rpmsg_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX rpmsg audio ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-rpmsg");
