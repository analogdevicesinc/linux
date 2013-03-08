/*
 * SSM4567 amplifier audio driver
 *
 * Copyright 2013-2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "ssm4567.h"

#define SSM4567_REG_POWER		0x00
#define SSM4567_REG_SENSE_AMP		0x01
#define SSM4567_REG_DAC			0x02
#define SSM4567_REG_VOLUME		0x03
#define SSM4567_REG_SAI_CTRL1		0x04
#define SSM4567_REG_SAI_CTRL2		0x05
#define SSM4567_REG_PLACEMENT(x) (0x6 + (x))
#define SSM4567_REG_VBAT		0x0c
#define SSM4567_REG_LIMITER_CTRL1	0x0d
#define SSM4567_REG_LIMITER_CTRL2	0x0e
#define SSM4567_REG_LIMITER_CTRL3	0x0f
#define SSM4567_REG_STATUS1		0x10
#define SSM4567_REG_STATUS2		0x11
#define SSM4567_REG_FAULT		0x12
#define SSM4567_REG_PDM_CTRL		0x13
#define SSM4567_REG_CLOCK_CTRL		0x14
#define SSM4567_REG_BOOST_CTRL1		0x15
#define SSM4567_REG_BOOST_CTRL2		0x16
#define SSM4567_REG_SOFT_RESET		0xff

#define SSM4567_POWER_APWDN_EN		BIT(7)
#define SSM4567_POWER_BSNS_PWDN		BIT(6)
#define SSM4567_POWER_VSNS_PWDN		BIT(5)
#define SSM4567_POWER_ISNS_PWDN		BIT(4)
#define SSM4567_POWER_BOOTST_PWDN	BIT(3)
#define SSM4567_POWER_AMP_PWDN		BIT(2)
#define SSM4567_POWER_VBAT_ONLY		BIT(1)
#define SSM4567_POWER_SPWDN		BIT(0)

#define SSM4567_SAI_CTRL1_BCLK			BIT(6)
#define SSM4567_SAI_CTRL1_TDM_BLCKS_MASK	(0x3 << 4)
#define SSM4567_SAI_CTRL1_TDM_BLCKS_32		(0x0 << 4)
#define SSM4567_SAI_CTRL1_TDM_BLCKS_48		(0x1 << 4)
#define SSM4567_SAI_CTRL1_TDM_BLCKS_64		(0x2 << 4)
#define SSM4567_SAI_CTRL1_FSYNC			BIT(3)
#define SSM4567_SAI_CTRL1_LJ			BIT(2)
#define SSM4567_SAI_CTRL1_TDM			BIT(1)
#define SSM4567_SAI_CTRL1_PDM			BIT(0)

#define SSM4567_SAI_CTRL2_TDM_SLOT_MASK		0x7
#define SSM4567_SAI_CTRL2_TDM_SLOT(x)		(x)

#define SSM4567_DAC_MUTE		BIT(6)
#define SSM4567_DAC_FS_MASK		0x07
#define SSM4567_DAC_FS_8000		0x00
#define SSM4567_DAC_FS_16000		0x01
#define SSM4567_DAC_FS_32000		0x02
#define SSM4567_DAC_FS_64000		0x03
#define SSM4567_DAC_FS_128000		0x04

struct ssm4567 {
	struct regmap *regmap;
	bool tdmc_mode;
};

static const uint8_t ssm4567_register_defaults[] = {
	0x81, 0x09, 0x32, 0x40, 0x00, 0x08, 0x01, 0x20,
	0x32, 0x07, 0x07, 0x07, 0x00, 0xa4, 0x73, 0x00,
	0x00, 0x00, 0x30, 0x40, 0x11, 0x02, 0x00,
};

static const DECLARE_TLV_DB_MINMAX_MUTE(ssm4567_vol_tlv, -7125, 2400);

static const char * const ssm4567_limiter_mode_text[] = {
	"Disable Limiter",
	"Enable Limiter",
	"Mute output if VBAT low",
	"Enable Limier if VBAT low",
};

static const char * const ssm4567_limiter_release_rate_text[] = {
	"3200 ms/dB", "1600 ms/dB", "1200 ms/dB", "800 ms/dB",
};

static const char * const ssm4567_limiter_attack_rate_text[] = {
	"120 us/dB", "60 us/dB", "30 us/dB", "20 us/dB",
};

static const char * const ssm4567_limiter_attack_th_text[] = {
	"6.6Vp", "6.4Vp", "6.2Vp", "6.0Vp", "5.8Vp", "5.6Vp", "5.4Vp", "5.2Vp",
	"5.0Vp", "4.8Vp", "4.6Vp", "4.4Vp", "4.2Vp", "4.0Vp", "3.8Vp", "3.6Vp",
};

static const SOC_ENUM_SINGLE_DECL(ssm4567_limiter_mode_enum,
	SSM4567_REG_LIMITER_CTRL1, 0, ssm4567_limiter_mode_text);
static const SOC_ENUM_SINGLE_DECL(ssm4567_limiter_release_rate_enum,
	SSM4567_REG_LIMITER_CTRL2, 6, ssm4567_limiter_release_rate_text);
static const SOC_ENUM_SINGLE_DECL(ssm4567_limiter_attack_rate_enum,
	SSM4567_REG_LIMITER_CTRL2, 4, ssm4567_limiter_attack_rate_text);
static const SOC_ENUM_SINGLE_DECL(ssm4567_limiter_attack_th_enum,
	SSM4567_REG_LIMITER_CTRL2, 0, ssm4567_limiter_attack_th_text);

static const struct snd_kcontrol_new ssm4567_snd_controls[] = {
	SOC_SINGLE("DAC High Pass Filter Switch", SSM4567_REG_DAC, 5, 1, 0),
	SOC_SINGLE("DAC Low Power Switch", SSM4567_REG_DAC, 4, 1, 0),
	SOC_SINGLE_TLV("Master Playback Volume", SSM4567_REG_VOLUME, 0, 0xff, 1,
			ssm4567_vol_tlv),
	SOC_SINGLE("Low-EMI Switch", SSM4567_REG_SENSE_AMP, 2, 1, 0),

	SOC_ENUM("Limiter Mode", ssm4567_limiter_mode_enum),
	SOC_ENUM("Limiter Attack Rate", ssm4567_limiter_attack_rate_enum),
	SOC_ENUM("Limiter Release Rate", ssm4567_limiter_release_rate_enum),
	SOC_ENUM("Limiter Attack Threshold", ssm4567_limiter_attack_th_enum),
};

static const struct snd_kcontrol_new ssm4567_amplifier_boost_control =
	SOC_DAPM_SINGLE("Switch", SSM4567_REG_POWER, 1, 1, 1);

static const struct snd_soc_dapm_widget ssm4567_dapm_widgets[] = {
	SND_SOC_DAPM_SWITCH("Amplifier Boost", SSM4567_REG_POWER, 3, 1,
		&ssm4567_amplifier_boost_control),
	SND_SOC_DAPM_DAC("DAC", NULL, SSM4567_REG_POWER, 2, 1),

	SND_SOC_DAPM_SIGGEN("Sense"),

	SND_SOC_DAPM_PGA("Current Sense", SSM4567_REG_POWER, 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Voltage Sense", SSM4567_REG_POWER, 5, 1, NULL, 0),
	SND_SOC_DAPM_PGA("VBAT Sense", SSM4567_REG_POWER, 6, 1, NULL, 0),

	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route ssm4567_routes[] = {
	{ "OUT", NULL, "DAC" },
	{ "OUT", NULL, "Amplifier Boost" },
	{ "Amplifier Boost", "Switch", "DAC" },

	{ "DAC", NULL, "Playback" },

	{ "Current Sense", NULL, "Sense" },
	{ "Voltage Sense", NULL, "Sense" },
	{ "VBAT Sense", NULL, "Sense" },
	{ "Capture", NULL, "Current Sense" },
	{ "Capture", NULL, "Voltage Sense" },
	{ "Capture", NULL, "VBAT Sense" },
};

static int ssm4567_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct ssm4567 *ssm4567 = snd_soc_dai_get_drvdata(dai);
	unsigned int rate = params_rate(params);
	unsigned int fs;

	if (rate >= 8000 && rate <= 12000)
		fs = SSM4567_DAC_FS_8000;
	else if (rate >= 16000 && rate <= 24000)
		fs = SSM4567_DAC_FS_16000;
	else if (rate >= 32000 && rate <= 48000)
		fs = SSM4567_DAC_FS_32000;
	else if (rate >= 64000 && rate <= 96000)
		fs = SSM4567_DAC_FS_64000;
	else if (rate >= 128000 && rate <= 192000)
		fs = SSM4567_DAC_FS_128000;
	else
		return -EINVAL;

	return regmap_update_bits(ssm4567->regmap, SSM4567_REG_DAC,
			SSM4567_DAC_FS_MASK, fs);
}

static int ssm4567_mute(struct snd_soc_dai *dai, int mute)
{
	struct ssm4567 *ssm4567 = snd_soc_dai_get_drvdata(dai);
	unsigned int val;

	if (mute)
		val = SSM4567_DAC_MUTE;
	else
		val = 0;

	return regmap_update_bits(ssm4567->regmap, SSM4567_REG_DAC,
			SSM4567_DAC_MUTE, val);
}

static int ssm4567_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct ssm4567 *ssm4567 = snd_soc_dai_get_drvdata(dai);
	unsigned int ctrl1 = 0;
	bool invert_fclk;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* In tdmc mode the DAI format is fixed */
	if (ssm4567->tdmc_mode) {
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;
		default:
			return -EINVAL;
		}

		switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_DSP_A:
			break;
		default:
			return -EINVAL;
		}

		return 0;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		invert_fclk = false;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1 |= SSM4567_SAI_CTRL1_BCLK;
		invert_fclk = false;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		ctrl1 |= SSM4567_SAI_CTRL1_FSYNC;
		invert_fclk = true;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		ctrl1 |= SSM4567_SAI_CTRL1_BCLK;
		invert_fclk = true;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1 |= SSM4567_SAI_CTRL1_LJ;
		invert_fclk = !invert_fclk;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1 |= SSM4567_SAI_CTRL1_TDM;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl1 |= SSM4567_SAI_CTRL1_TDM | SSM4567_SAI_CTRL1_LJ;
		break;
	case SND_SOC_DAIFMT_PDM:
		ctrl1 |= SSM4567_SAI_CTRL1_PDM;
		break;
	default:
		return -EINVAL;
	}

	if (invert_fclk)
		ctrl1 |= SSM4567_SAI_CTRL1_FSYNC;

	return regmap_write(ssm4567->regmap, SSM4567_REG_SAI_CTRL1, ctrl1);
}

static int ssm4567_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	struct ssm4567 *ssm4567 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = regmap_update_bits(ssm4567->regmap,
				SSM4567_REG_POWER, SSM4567_POWER_SPWDN, 0);
		}
		break;
	case SND_SOC_BIAS_OFF:
		ret = regmap_update_bits(ssm4567->regmap, SSM4567_REG_POWER,
			SSM4567_POWER_SPWDN, SSM4567_POWER_SPWDN);
		break;
	}

	if (ret)
		return ret;

	codec->dapm.bias_level = level;
	return 0;
}

static int ssm4567_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int width)
{
	struct ssm4567 *ssm4567 = snd_soc_dai_get_drvdata(dai);
	unsigned int blcks;
	int slot;
	int ret;

	if (tx_mask == 0)
		return -EINVAL;

	if (rx_mask && rx_mask != tx_mask)
		return -EINVAL;

	slot = __ffs(tx_mask);
	if (tx_mask != BIT(slot))
		return -EINVAL;

	switch (width) {
	case 32:
		blcks = SSM4567_SAI_CTRL1_TDM_BLCKS_32;
		break;
	case 48:
		blcks = SSM4567_SAI_CTRL1_TDM_BLCKS_48;
		break;
	case 64:
		blcks = SSM4567_SAI_CTRL1_TDM_BLCKS_64;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(ssm4567->regmap, SSM4567_REG_SAI_CTRL2,
		SSM4567_SAI_CTRL2_TDM_SLOT_MASK,
		SSM4567_SAI_CTRL2_TDM_SLOT(slot));
	if (ret)
		return ret;

	return regmap_update_bits(ssm4567->regmap, SSM4567_REG_SAI_CTRL1,
		SSM4567_SAI_CTRL1_TDM_BLCKS_MASK, blcks);
}

static const struct snd_soc_dai_ops ssm4567_dai_ops = {
	.hw_params	= ssm4567_hw_params,
	.digital_mute	= ssm4567_mute,
	.set_fmt	= ssm4567_set_dai_fmt,
	.set_tdm_slot	= ssm4567_set_tdm_slot,
};

static struct snd_soc_dai_driver ssm4567_dai = {
	.name = "ssm4567-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S32,
		.sig_bits = 24,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S32,
	},
	.ops = &ssm4567_dai_ops,
};

static int ssm4567_codec_probe(struct snd_soc_codec *codec)
{
	struct ssm4567 *ssm4567 = snd_soc_codec_get_drvdata(codec);

	codec->control_data = ssm4567->regmap;
	return snd_soc_codec_set_cache_io(codec, 0, 0, SND_SOC_REGMAP);
}

static struct snd_soc_codec_driver ssm4567_codec_driver = {
	.probe = ssm4567_codec_probe,
	.set_bias_level = ssm4567_set_bias_level,
	.idle_bias_off = true,

	.controls = ssm4567_snd_controls,
	.num_controls = ARRAY_SIZE(ssm4567_snd_controls),
	.dapm_widgets = ssm4567_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ssm4567_dapm_widgets),
	.dapm_routes = ssm4567_routes,
	.num_dapm_routes = ARRAY_SIZE(ssm4567_routes),
};

static bool ssm4567_register_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SSM4567_REG_SOFT_RESET:
	case SSM4567_REG_STATUS1:
	case SSM4567_REG_STATUS2:
	case SSM4567_REG_FAULT:
		return true;
	default:
		return false;
	}
}

const struct regmap_config ssm4567_regmap_config = {
	.val_bits = 8,
	.reg_bits = 8,

	.max_register = SSM4567_REG_BOOST_CTRL2,
	.volatile_reg = ssm4567_register_volatile,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults_raw = ssm4567_register_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(ssm4567_register_defaults),
};
EXPORT_SYMBOL_GPL(ssm4567_regmap_config);

int ssm4567_probe(struct device *dev, bool tdmc_mode, struct regmap *regmap)
{
	struct ssm4567 *ssm4567;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ssm4567 = devm_kzalloc(dev, sizeof(*ssm4567), GFP_KERNEL);
	if (ssm4567 == NULL)
		return -ENOMEM;

	ssm4567->regmap = regmap;
	ssm4567->tdmc_mode = tdmc_mode;
	dev_set_drvdata(dev, ssm4567);

	regmap_write(ssm4567->regmap, SSM4567_REG_SOFT_RESET, 0x00);

	return snd_soc_register_codec(dev, &ssm4567_codec_driver,
		&ssm4567_dai, 1);
}
EXPORT_SYMBOL_GPL(ssm4567_probe);

MODULE_DESCRIPTION("ASoC SSM4567 driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
