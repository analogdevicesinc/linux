// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2020 Intel Corporation

/*
 *  sof_sdw_rt5682 - Helpers to handle RT5682 from generic machine driver
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/soundwire/sdw.h>
#include <linux/soundwire/sdw_type.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <sound/soc-acpi.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include "sof_sdw_common.h"

static const struct snd_soc_dapm_route rt5682_map[] = {
	/*Headphones*/
	{ "Headphone", NULL, "rt5682 HPOL" },
	{ "Headphone", NULL, "rt5682 HPOR" },
	{ "rt5682 IN1P", NULL, "Headset Mic" },
};

static struct snd_soc_jack_pin rt5682_jack_pins[] = {
	{
		.pin    = "Headphone",
		.mask   = SND_JACK_HEADPHONE,
	},
	{
		.pin    = "Headset Mic",
		.mask   = SND_JACK_MICROPHONE,
	},
};

static const char * const jack_codecs[] = {
	"rt5682"
};

int rt5682_rtd_init(struct snd_soc_pcm_runtime *rtd, struct snd_soc_dai *dai)
{
	struct snd_soc_card *card = rtd->card;
	struct mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai;
	struct snd_soc_component *component;
	struct snd_soc_jack *jack;
	int ret;

	codec_dai = get_codec_dai_by_name(rtd, jack_codecs, ARRAY_SIZE(jack_codecs));
	if (!codec_dai)
		return -EINVAL;

	component = codec_dai->component;
	card->components = devm_kasprintf(card->dev, GFP_KERNEL,
					  "%s hs:rt5682",
					  card->components);
	if (!card->components)
		return -ENOMEM;

	ret = snd_soc_dapm_add_routes(&card->dapm, rt5682_map,
				      ARRAY_SIZE(rt5682_map));

	if (ret) {
		dev_err(card->dev, "rt5682 map addition failed: %d\n", ret);
		return ret;
	}

	ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2 |
					 SND_JACK_BTN_3,
					 &ctx->sdw_headset,
					 rt5682_jack_pins,
					 ARRAY_SIZE(rt5682_jack_pins));
	if (ret) {
		dev_err(rtd->card->dev, "Headset Jack creation failed: %d\n",
			ret);
		return ret;
	}

	jack = &ctx->sdw_headset;

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	ret = snd_soc_component_set_jack(component, jack, NULL);

	if (ret)
		dev_err(rtd->card->dev, "Headset Jack call-back failed: %d\n",
			ret);

	return ret;
}
MODULE_IMPORT_NS(SND_SOC_INTEL_SOF_BOARD_HELPERS);
