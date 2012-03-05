#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "adv7511.h"

static const DECLARE_TLV_DB_SCALE(adv7511_levelshift_tlv, 0, 100, 0);

static const struct snd_kcontrol_new adv7511_controls[] = {
	SOC_SINGLE("Master Playback Switch", 0, 0, 0, 0),
	SOC_SINGLE_TLV("Master Playback Volume", 0, 0, 45, 0,
		adv7511_levelshift_tlv),
};

static const struct snd_soc_dapm_widget adv7511_dapm_widgets[] = {
};

static const struct snd_soc_dapm_route adv7511_routes[] = {
	{ "TMDS", NULL, "DAI IN" },
};

static void adv7511_calc_cts_n(unsigned int f_tmds, unsigned int fs,
	unsigned int *cts, unsigned int *n)
{
	switch (fs) {
	case 32000:
		*n = 4096;
		break;
	case 44100:
		*n = 6272;
		break;
	case 48000:
		*n = 6144;
		break;
	}

	*cts = (f_tmds * *n) / (128 * fs);
}

static int adv7511_update_cts_n(struct adv7511 *adv7511)
{
	unsigned int cts = 0;
	unsigned int n = 0;

	adv7511_calc_cts_n(adv7511->f_tmds, adv7511->f_audio, &cts, &n);

	regmap_write(adv7511->regmap, ADV7511_REG_N0, (n >> 16) & 0xf);
	regmap_write(adv7511->regmap, ADV7511_REG_N1, (n >> 8) & 0xff);
	regmap_write(adv7511->regmap, ADV7511_REG_N2, n & 0xff);

	regmap_write(adv7511->regmap, ADV7511_REG_CTS_MANUAL0, (n >> 16) & 0xf);
	regmap_write(adv7511->regmap, ADV7511_REG_CTS_MANUAL1, (n >> 8) & 0xff);
	regmap_write(adv7511->regmap, ADV7511_REG_CTS_MANUAL2, n & 0xff);

	return 0;
}

static int adv7511_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct adv7511 *adv7511 = snd_soc_codec_get_drvdata(codec);
	unsigned int rate;
	unsigned int len;

	switch (params_rate(params)) {
	case 32000:
		rate = ADV7511_SAMPLE_FREQ_32000;
		break;
	case 44100:
		rate = ADV7511_SAMPLE_FREQ_44100;
		break;
	case 48000:
		rate = ADV7511_SAMPLE_FREQ_48000;
		break;
	case 88200:
		rate = ADV7511_SAMPLE_FREQ_88200;
		break;
	case 96000:
		rate = ADV7511_SAMPLE_FREQ_96000;
		break;
	case 176400:
		rate = ADV7511_SAMPLE_FREQ_176400;
		break;
	case 192000:
		rate = ADV7511_SAMPLE_FREQ_192000;
		break;
	default:
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		len = ADV7511_I2S_SAMPLE_LEN_16;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		len = ADV7511_I2S_SAMPLE_LEN_18;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		len = ADV7511_I2S_SAMPLE_LEN_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		len = ADV7511_I2S_SAMPLE_LEN_24;
		break;
	default:
		return -EINVAL;
	}

	adv7511->f_audio = params_rate(params);

	adv7511_update_cts_n(adv7511);

	regmap_update_bits(adv7511->regmap, 0x14, 0x0f, len);
	regmap_update_bits(adv7511->regmap, 0x15, 0xf0, rate << 4);

	return 0;
}

#define ADV7511_AUDIO_SOURCE_I2S 0
#define ADV7511_AUDIO_SOURCE_SPDIF 1

#define ADV7511_I2S_FORMAT_I2S 0
#define ADV7511_I2S_FORMAT_RIGHT_J 1
#define ADV7511_I2S_FORMAT_LEFT_J 2

static int adv7511_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adv7511 *adv7511 = snd_soc_codec_get_drvdata(codec);
	unsigned int audio_source, i2s_format = 0;
	unsigned int invert_clock;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		audio_source = ADV7511_AUDIO_SOURCE_I2S;
		i2s_format = ADV7511_I2S_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		audio_source = ADV7511_AUDIO_SOURCE_I2S;
		i2s_format = ADV7511_I2S_FORMAT_RIGHT_J;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		audio_source = ADV7511_AUDIO_SOURCE_I2S;
		i2s_format = ADV7511_I2S_FORMAT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_SPDIF:
		audio_source = ADV7511_AUDIO_SOURCE_SPDIF;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		invert_clock = 0;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		invert_clock = 1;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(adv7511->regmap, 0x0a, 0x70, audio_source << 4);
	regmap_update_bits(adv7511->regmap, 0x0b, BIT(6), invert_clock << 6);
	regmap_update_bits(adv7511->regmap, 0x0c, 0x03, i2s_format);

	adv7511->audio_source = audio_source;

	return 0;
}

static int adv7511_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct adv7511 *adv7511 = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		switch (adv7511->audio_source) {
		case ADV7511_AUDIO_SOURCE_I2S:
			break;
		case ADV7511_AUDIO_SOURCE_SPDIF:
			regmap_update_bits(adv7511->regmap, 0x0b, BIT(7), BIT(7));
			break;
		}
		break;
	case SND_SOC_BIAS_PREPARE:
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			adv7511_packet_enable(adv7511, ADV7511_PACKET_ENABLE_AUDIO_SAMPLE);
			adv7511_packet_enable(adv7511, ADV7511_PACKET_ENABLE_AUDIO_INFOFRAME);
			adv7511_packet_enable(adv7511, ADV7511_PACKET_ENABLE_N_CTS);
		} else {
			adv7511_packet_disable(adv7511, ADV7511_PACKET_ENABLE_AUDIO_SAMPLE);
			adv7511_packet_disable(adv7511, ADV7511_PACKET_ENABLE_AUDIO_INFOFRAME);
			adv7511_packet_disable(adv7511, ADV7511_PACKET_ENABLE_N_CTS);
		}
		break;
	case SND_SOC_BIAS_STANDBY:
		regmap_update_bits(adv7511->regmap, 0x0b, BIT(7), 0);
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define ADV7511_RATES (SNDRV_PCM_RATE_32000 |\
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |\
		SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)

#define ADV7511_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE |\
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dai_ops adv7511_dai_ops = {
	.hw_params	= adv7511_hw_params,
	/*.set_sysclk	= adv7511_set_dai_sysclk,*/
	.set_fmt	= adv7511_set_dai_fmt,
};

static struct snd_soc_dai_driver adv7511_dai = {
	.name = "adv7511",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ADV7511_RATES,
		.formats = ADV7511_FORMATS,
	},
	.ops = &adv7511_dai_ops,
};

static int adv7511_suspend(struct snd_soc_codec *codec)
{
	adv7511_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int adv7511_resume(struct snd_soc_codec *codec)
{
	adv7511_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int adv7511_probe(struct snd_soc_codec *codec)
{
	struct adv7511 *adv7511 = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = adv7511->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 0, 0, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	adv7511_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int adv7511_remove(struct snd_soc_codec *codec)
{
	adv7511_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver adv7511_codec_driver = {
	.probe		    = adv7511_probe,
	.remove		    = adv7511_remove,
	.suspend	    = adv7511_suspend,
	.resume		    = adv7511_resume,
	.set_bias_level	    = adv7511_set_bias_level,

/*
	.controls	    = adv7511_controls,
	.num_controls	    = ARRAY_SIZE(adv7511_controls),*/
	.dapm_widgets	    = adv7511_dapm_widgets,
	.num_dapm_widgets   = ARRAY_SIZE(adv7511_dapm_widgets),
	.dapm_routes	    = adv7511_routes,
	.num_dapm_routes    = ARRAY_SIZE(adv7511_routes),
};

int adv7511_audio_init(struct device *dev)
{
    return snd_soc_register_codec(dev, &adv7511_codec_driver,
		&adv7511_dai, 1);
}

void adv7511_audio_exit(struct device *dev)
{
	snd_soc_unregister_codec(dev);
}
