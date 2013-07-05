/*
 *  Copyright (C) 2012, Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

static const struct snd_soc_dapm_widget adv7511_hdmi_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route adv7511_hdmi_dapm_routes[] = {
	{ "Speaker", NULL, "TMDS" },
};

static struct snd_soc_dai_link hdmi_dai_link = {
	.name = "HDMI",
	.stream_name = "HDMI",
/*	.cpu_dai_name = "75c00000.axi-spdif-tx",
	.platform_name = "xilinx_pcm_audio.2",
	.codec_name = adv7511_codec_name,*/
	.codec_dai_name = "adv7511",
	.dai_fmt = SND_SOC_DAIFMT_SPDIF |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card hdmi_card = {
	.name = "HDMI monitor",
	.owner = THIS_MODULE,
	.dai_link = &hdmi_dai_link,
	.num_links = 1,
	.dapm_widgets = adv7511_hdmi_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(adv7511_hdmi_dapm_widgets),
	.dapm_routes = adv7511_hdmi_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(adv7511_hdmi_dapm_routes),
};

static int adv7511_hdmi_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &hdmi_card;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;

	card->dev = &pdev->dev;

	hdmi_dai_link.codec_of_node = of_parse_phandle(of_node, "audio-codec", 0);
	hdmi_dai_link.cpu_of_node = of_parse_phandle(of_node, "cpu-dai", 0);
	hdmi_dai_link.platform_of_node = hdmi_dai_link.cpu_of_node;

	if (!hdmi_dai_link.codec_of_node || !hdmi_dai_link.cpu_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int adv7511_hdmi_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id adv7511_hdmi_of_match[] = {
	{ .compatible = "adv7511-hdmi-snd", },
	{},
};
MODULE_DEVICE_TABLE(of, adv7511_hdmi_of_match);

static struct platform_driver hdmi_card_driver = {
	.driver = {
		.name = "adv7511-hdmi-snd",
		.owner = THIS_MODULE,
		.of_match_table = adv7511_hdmi_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = adv7511_hdmi_probe,
	.remove = adv7511_hdmi_remove,
};
module_platform_driver(hdmi_card_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("ADV7511 HDMI sound driver");
MODULE_LICENSE("GPL");
