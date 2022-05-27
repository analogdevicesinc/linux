// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <sound/asoundef.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

struct ad242x_private {
	struct a2b_node		*node;
	unsigned int		inv_fmt;
	bool			pdm[2];
	bool			pdm_highpass;
};

static const struct snd_soc_dapm_widget ad242x_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("RX0",  NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("RX1",  NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX1", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route ad242x_dapm_routes[] = {
	{ "DAI0 Playback", NULL, "RX0" },
	{ "TX0", NULL, "DAI0 Capture"  },
	{ "DAI1 Playback", NULL, "RX1" },
	{ "TX1", NULL, "DAI1 Capture"  },
};

static int ad242x_set_dai_fmt(struct snd_soc_dai *dai,
			      unsigned int format)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);
	int ret;

	/* set DAI format */
	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		priv->pdm[dai->id] = false;
		break;
	case SND_SOC_DAIFMT_PDM:
		priv->pdm[dai->id] = true;
		break;
	default:
		dev_err(component->dev, "unsupported dai format\n");
		return -EINVAL;
	}

	/*
	 * Setting clock inversion is only supported globally for both DAIs,
	 * so we require the settings for DAI1 to match those for DAI0.
	 */
	if (dai->id == 0) {
		switch (format & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			priv->inv_fmt = 0;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			priv->inv_fmt = A2B_I2SCTL_RXBCLKINV;
			break;
		case SND_SOC_DAIFMT_NB_IF:
		case SND_SOC_DAIFMT_IB_IF:
			dev_err(component->dev, "unsupported inversion mask\n");
			return -EINVAL;
		}

		ret = regmap_update_bits(priv->node->regmap, A2B_I2SCTL,
					A2B_I2SCTL_RXBCLKINV,
					priv->inv_fmt);
		if (ret < 0)
			return ret;
	} else {
		if (format & SND_SOC_DAIFMT_INV_MASK) {
			if(priv->inv_fmt != A2B_I2SCTL_RXBCLKINV) {
				dev_err(component->dev,
					"inversion masks for both dai must be identical\n");
				return -EINVAL;
			}
		}
	}

	if (a2b_node_is_main(priv->node) &&
	   ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)) {
		dev_err(component->dev, "master node must be clock slave\n");
		return -EINVAL;
	}

	if (!a2b_node_is_main(priv->node) &&
	   ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBM_CFM)) {
		dev_err(component->dev, "slave node must be clock master\n");
		return -EINVAL;
	}

	return 0;
}

static int ad242x_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);
	struct a2b_tdm_config *tdm_config = &priv->node->tdm_config;
	struct clk *sync_clk = a2b_node_get_sync_clk(priv->node);
	unsigned int sff_rate = clk_get_rate(sync_clk);
	unsigned int rate = params_rate(params);
	unsigned int val, mask;
	int ret;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (tdm_config->tdm_slot_size != 16)
			return -EINVAL;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		if (tdm_config->tdm_slot_size != 32)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (priv->pdm[dai->id]) {
		if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
			return -EINVAL;

		if (dai->id == 0) {
			val = A2B_PDMCTL_PDM0EN;
			mask = A2B_PDMCTL_PDM0EN | A2B_PDMCTL_PDM0SLOTS;
		} else {
			val = A2B_PDMCTL_PDM1EN;
			mask = A2B_PDMCTL_PDM1EN | A2B_PDMCTL_PDM1SLOTS;
		}

		switch (params_channels(params)) {
		case 1:
			break;
		case 2:
			val = mask;
			break;
		default:
			return -EINVAL;
		}

		mask |= A2B_PDMCTL_HPFEN;
		if (priv->pdm_highpass)
			val |= A2B_PDMCTL_HPFEN;

		ret = regmap_update_bits(priv->node->regmap, A2B_PDMCTL,
					 mask, val);
		if (ret < 0)
			return ret;
	} else {
		if (params_channels(params) > tdm_config->tdm_channels)
			return -EINVAL;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (dai->id == 0)
				mask = A2B_I2SCTL_RX0EN;
			else
				mask = A2B_I2SCTL_RX1EN;
		} else {
			if (dai->id == 0)
				mask = A2B_I2SCTL_TX0EN;
			else
				mask = A2B_I2SCTL_TX1EN;
		}

		ret = regmap_update_bits(priv->node->regmap, A2B_I2SCTL,
					 mask, mask);
		if (ret < 0)
			return ret;
	}

	if (!a2b_node_is_main(priv->node)) {
		val = 0;

		if (rate == sff_rate / 2)
			val = A2B_I2SRATE_I2SRATE(1);
		else if (rate == sff_rate / 4)
			val = A2B_I2SRATE_I2SRATE(2);
		else if (rate == sff_rate * 2)
			val = A2B_I2SRATE_I2SRATE(5);
		else if (rate == sff_rate * 4)
			val = A2B_I2SRATE_I2SRATE(6);
		else if (rate != sff_rate)
			return -EINVAL;

		ret = regmap_write(priv->node->regmap, A2B_I2SRATE, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct snd_soc_dai_ops ad242x_dai_ops = {
	.hw_params	= ad242x_hw_params,
	.set_fmt	= ad242x_set_dai_fmt,
};

#define AD242X_RATES (					\
	SNDRV_PCM_RATE_22050  | SNDRV_PCM_RATE_32000  |	\
	SNDRV_PCM_RATE_44100  | SNDRV_PCM_RATE_48000  |	\
	SNDRV_PCM_RATE_88200  | SNDRV_PCM_RATE_96000  |	\
	SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)
#define AD242X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver ad242x_dai[] = {
	{
		.name = "ad242x-dai0",
		.playback = {
			.stream_name	= "DAI0 Playback",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
		},
		.capture = {
			.stream_name	= "DAI0 Capture",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
		},
		.ops = &ad242x_dai_ops,
	},
	{
		.name = "ad242x-dai1",
		.playback = {
			.stream_name	= "DAI1 Playback",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
		},
		.capture = {
			.stream_name	= "DAI1 Capture",
			.channels_min	= 1,
			.channels_max	= 32,
			.rates		= AD242X_RATES,
			.formats	= AD242X_FORMATS,
		},
		.ops = &ad242x_dai_ops,
	},
};

static int ad242x_soc_probe(struct snd_soc_component *component)
{
	struct ad242x_private *priv = snd_soc_component_get_drvdata(component);

	component->regmap = priv->node->regmap;

	return 0;
}

static const struct snd_soc_component_driver soc_component_device_ad242x = {
	.probe			= ad242x_soc_probe,
	.dapm_widgets		= ad242x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ad242x_dapm_widgets),
	.dapm_routes		= ad242x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ad242x_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int ad242x_codec_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad242x_private *priv;

	if (!dev->of_node)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->node = dev_to_a2b_node(dev->parent);
	platform_set_drvdata(pdev, priv);

	priv->pdm_highpass = of_property_read_bool(dev->of_node,
						   "adi,pdm-highpass-filter");

	return devm_snd_soc_register_component(dev,
					       &soc_component_device_ad242x,
					       ad242x_dai,
					       ARRAY_SIZE(ad242x_dai));
}

static const struct of_device_id ad242x_of_match[] = {
	{ .compatible = "adi,ad2428w-codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, ad242x_of_match);

static struct platform_driver ad242x_platform_driver = {
	.driver  = {
		.name   = "ad242x-codec",
		.of_match_table = ad242x_of_match,
	},
	.probe  = ad242x_codec_platform_probe,
};

module_platform_driver(ad242x_platform_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("AD242X ALSA SoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ad242x-codec");
