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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

static struct snd_soc_dai_link hdmi_dai_link = {
	.name = "HDMI",
	.stream_name = "HDMI",
/*	.cpu_dai_name = "75c00000.axi-spdif-tx",
	.platform_name = "xilinx_pcm_audio.2",*/
	.codec_name = "adv7511.2-0039",
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
};

static int __devinit adv7511_hdmi_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &hdmi_card;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;

	card->dev = &pdev->dev;

/*	hdmi_dai_link.codec_of_node = of_parse_phandle(of_node, "audio-codec", 0);*/
	hdmi_dai_link.cpu_dai_of_node = of_parse_phandle(of_node, "cpu-dai", 0);
	hdmi_dai_link.platform_of_node = of_parse_phandle(of_node, "pcm", 0);

	if (/*!hdmi_dai_link.codec_of_node || */!hdmi_dai_link.cpu_dai_of_node ||
		!hdmi_dai_link.platform_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int __devexit adv7511_hdmi_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id adv7511_hdmi_of_match[] __devinitconst = {
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
	.remove = __devexit_p(adv7511_hdmi_remove),
};
module_platform_driver(hdmi_card_driver);
