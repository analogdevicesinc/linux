// SPDX-License-Identifier: GPL-2.0
/*
 * ASoC Driver for HiFiBerry ADC
 *
 * Author:	Joerg Schambacher <joerg@hifiberry.com>
 *		Copyright 2024
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/tlv.h>

#include "../codecs/pcm186x.h"
#include "hifiberry_adc_controls.h"

static bool leds_off;

static int pcm1863_add_controls(struct snd_soc_component *component)
{
	snd_soc_add_component_controls(component,
			pcm1863_snd_controls_card,
			ARRAY_SIZE(pcm1863_snd_controls_card));
	return 0;
}

static int snd_rpi_hifiberry_adc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *adc = codec_dai->component;
	int ret;

	ret = pcm1863_add_controls(adc);
	if (ret < 0)
		dev_warn(rtd->dev, "Failed to add pcm1863 controls: %d\n",
		ret);

	codec_dai->driver->capture.rates =
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000;

	/* set GPIO2 to output, GPIO3 input */
	snd_soc_component_write(adc, PCM186X_GPIO3_2_CTRL, 0x00);
	snd_soc_component_write(adc, PCM186X_GPIO3_2_DIR_CTRL, 0x04);
	if (leds_off)
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x00);
	else
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x40);

	return 0;
}

static int snd_rpi_hifiberry_adc_hw_params(
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int channels = params_channels(params);
	int width =  snd_pcm_format_width(params_format(params));

	/* Using powers of 2 allows for an integer clock divisor */
	width = width <= 16 ? 16 : 32;

	ret = snd_soc_dai_set_bclk_ratio(snd_soc_rtd_to_cpu(rtd, 0), channels * width);
	return ret;
}

/* machine stream operations */
static const struct snd_soc_ops snd_rpi_hifiberry_adc_ops = {
	.hw_params = snd_rpi_hifiberry_adc_hw_params,
};

SND_SOC_DAILINK_DEFS(hifi,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2708-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("pcm186x.1-004a", "pcm1863-aif")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2708-i2s.0")));

static struct snd_soc_dai_link snd_rpi_hifiberry_adc_dai[] = {
{
	.name		= "HiFiBerry ADC",
	.stream_name	= "HiFiBerry ADC HiFi",
	.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
	.ops		= &snd_rpi_hifiberry_adc_ops,
	.init		= snd_rpi_hifiberry_adc_init,
	SND_SOC_DAILINK_REG(hifi),
},
};

/* audio machine driver */
static struct snd_soc_card snd_rpi_hifiberry_adc = {
	.name         = "snd_rpi_hifiberry_adc",
	.driver_name  = "HifiberryAdc",
	.owner        = THIS_MODULE,
	.dai_link     = snd_rpi_hifiberry_adc_dai,
	.num_links    = ARRAY_SIZE(snd_rpi_hifiberry_adc_dai),
};

static int snd_rpi_hifiberry_adc_probe(struct platform_device *pdev)
{
	int ret = 0, i = 0;
	struct snd_soc_card *card = &snd_rpi_hifiberry_adc;

	snd_rpi_hifiberry_adc.dev = &pdev->dev;
	if (pdev->dev.of_node) {
		struct device_node *i2s_node;
		struct snd_soc_dai_link *dai;

		dai = &snd_rpi_hifiberry_adc_dai[0];
		i2s_node = of_parse_phandle(pdev->dev.of_node,
			"i2s-controller", 0);
		if (i2s_node) {
			for (i = 0; i < card->num_links; i++) {
				dai->cpus->dai_name = NULL;
				dai->cpus->of_node = i2s_node;
				dai->platforms->name = NULL;
				dai->platforms->of_node = i2s_node;
			}
		}
	}
	leds_off = of_property_read_bool(pdev->dev.of_node,
					"hifiberry-adc,leds_off");
	ret = snd_soc_register_card(&snd_rpi_hifiberry_adc);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev,
			"snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static const struct of_device_id snd_rpi_hifiberry_adc_of_match[] = {
	{ .compatible = "hifiberry,hifiberry-adc", },
	{},
};

MODULE_DEVICE_TABLE(of, snd_rpi_hifiberry_adc_of_match);

static struct platform_driver snd_rpi_hifiberry_adc_driver = {
	.driver = {
		.name   = "snd-rpi-hifiberry-adc",
		.owner  = THIS_MODULE,
		.of_match_table = snd_rpi_hifiberry_adc_of_match,
	},
	.probe          = snd_rpi_hifiberry_adc_probe,
};

module_platform_driver(snd_rpi_hifiberry_adc_driver);

MODULE_AUTHOR("Joerg Schambacher <joerg@hifiberry.com>");
MODULE_DESCRIPTION("ASoC Driver for HiFiBerry ADC");
MODULE_LICENSE("GPL");
