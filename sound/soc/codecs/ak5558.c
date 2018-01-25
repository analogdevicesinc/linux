/*
 * ak5558.c  --  audio driver for AK5558 ADC
 *
 * Copyright (C) 2015 Asahi Kasei Microdevices Corporation
 * Copyright 2017 NXP
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>

#include "ak5558.h"

#define AK5558_SLAVE_CKS_AUTO

/* enable debug */
/* #define AK5558_DEBUG */

/* AK5558 Codec Private Data */
struct ak5558_priv {
	struct snd_soc_codec codec;
	struct regmap *regmap;
	struct i2c_client *i2c;
	int fs;		/* Sampling Frequency */
	int rclk;	/* Master Clock */
	int pdn_gpio;	/* Power on / Reset GPIO */
};

/* ak5558 register cache & default register settings */
static const struct reg_default ak5558_reg[] = {
	{ 0x0, 0xFF },	/*	0x00	AK5558_00_POWER_MANAGEMENT1	*/
	{ 0x1, 0x01 },	/*	0x01	AK5558_01_POWER_MANAGEMENT2	*/
	{ 0x2, 0x01 },	/*	0x02	AK5558_02_CONTROL1		*/
	{ 0x3, 0x00 },	/*	0x03	AK5558_03_CONTROL2		*/
	{ 0x4, 0x00 },	/*	0x04	AK5558_04_CONTROL3		*/
	{ 0x5, 0x00 }	/*	0x05	AK5558_05_DSD			*/
};

static const char * const mono_texts[] = {
	"8 Slot", "2 Slot", "4 Slot", "1 Slot",
};

static const struct soc_enum ak5558_mono_enum[] = {
	SOC_ENUM_SINGLE(AK5558_01_POWER_MANAGEMENT2, 1,
			ARRAY_SIZE(mono_texts), mono_texts)
};

static const char * const tdm_texts[] = {
	"Off", "TDM128",  "TDM256", "TDM512",
};

static const char * const digfil_texts[] = {
	"Sharp Roll-Off", "Show Roll-Off",
	"Short Delay Sharp Roll-Off", "Short Delay Show Roll-Off",
};

static const struct soc_enum ak5558_adcset_enum[] = {
	SOC_ENUM_SINGLE(AK5558_03_CONTROL2, 5,
			ARRAY_SIZE(tdm_texts), tdm_texts),
	SOC_ENUM_SINGLE(AK5558_04_CONTROL3, 0,
			ARRAY_SIZE(digfil_texts), digfil_texts),
};

static const char * const dsdon_texts[] = {
	"PCM", "DSD",
};

static const char * const dsdsel_texts[] = {
	"64fs", "128fs", "256fs"
};

static const char * const dckb_texts[] = {
	"Falling", "Rising",
};

static const char * const dcks_texts[] = {
	"512fs", "768fs",
};

static const struct soc_enum ak5558_dsdset_enum[] = {
	SOC_ENUM_SINGLE(AK5558_04_CONTROL3, 7,
			ARRAY_SIZE(dsdon_texts), dsdon_texts),
	SOC_ENUM_SINGLE(AK5558_05_DSD, 0,
			ARRAY_SIZE(dsdsel_texts), dsdsel_texts),
	SOC_ENUM_SINGLE(AK5558_05_DSD, 2, ARRAY_SIZE(dckb_texts), dckb_texts),
	SOC_ENUM_SINGLE(AK5558_05_DSD, 5, ARRAY_SIZE(dcks_texts), dcks_texts),
};

#ifdef AK5558_DEBUG
static const char * const test_reg_select[]   = {
	"read AK5558 Reg 00:05",
};

static const struct soc_enum ak5558_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo;

static int get_test_reg(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value  *ucontrol)
{
	ucontrol->value.enumerated.item[0] = nTestRegNo;

	return 0;
}

static int set_test_reg(struct snd_kcontrol  *kcontrol,
			struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	u32 currMode = ucontrol->value.enumerated.item[0];
	int i, value;
	int regs, rege;

	nTestRegNo = currMode;

	regs = 0x00;
	rege = 0x05;

	for (i = regs; i <= rege; i++) {
		value = snd_soc_read(codec, i);
		pr_info("***AK5558 Addr,Reg=(%x, %x)\n", i, value);
	}

	return 0;
}

#endif

static const struct snd_kcontrol_new ak5558_snd_controls[] = {
	SOC_ENUM("AK5558 Monaural Mode", ak5558_mono_enum[0]),
	SOC_ENUM("AK5558 TDM mode", ak5558_adcset_enum[0]),
	SOC_ENUM("AK5558 Digital Filter", ak5558_adcset_enum[1]),

	SOC_ENUM("AK5558 DSD Mode", ak5558_dsdset_enum[0]),
	SOC_ENUM("AK5558 Frequency of DCLK", ak5558_dsdset_enum[1]),
	SOC_ENUM("AK5558 Polarity of DCLK", ak5558_dsdset_enum[2]),
	SOC_ENUM("AK5558 Master Clock Frequency at DSD Mode",
		 ak5558_dsdset_enum[3]),

	SOC_SINGLE("AK5558 DSD Phase Modulation", AK5558_05_DSD, 3, 1, 0),

#ifdef AK5558_DEBUG
	SOC_ENUM_EXT("AK5558 Reg Read", ak5558_enum[0],
		     get_test_reg, set_test_reg),
#endif

};

static const char * const ak5558_channel_select_texts[] = {"Off", "On"};

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel1_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel1_mux_control =
	SOC_DAPM_ENUM("Ch1 Switch", ak5558_channel1_mux_enum);

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel2_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel2_mux_control =
	SOC_DAPM_ENUM("Ch2 Switch", ak5558_channel2_mux_enum);

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel3_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel3_mux_control =
	SOC_DAPM_ENUM("Ch3 Switch", ak5558_channel3_mux_enum);

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel4_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel4_mux_control =
	SOC_DAPM_ENUM("Ch4 Switch", ak5558_channel4_mux_enum);

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel5_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel5_mux_control =
	SOC_DAPM_ENUM("Ch5 Switch", ak5558_channel5_mux_enum);

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel6_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel6_mux_control =
	SOC_DAPM_ENUM("Ch6 Switch", ak5558_channel6_mux_enum);

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel7_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel7_mux_control =
	SOC_DAPM_ENUM("Ch7 Switch", ak5558_channel7_mux_enum);

static SOC_ENUM_SINGLE_VIRT_DECL(ak5558_channel8_mux_enum,
				 ak5558_channel_select_texts);

static const struct snd_kcontrol_new ak5558_channel8_mux_control =
	SOC_DAPM_ENUM("Ch8 Switch", ak5558_channel8_mux_enum);

static const struct snd_soc_dapm_widget ak5558_dapm_widgets[] = {

	/* Analog Input */
	SND_SOC_DAPM_INPUT("AIN1"),
	SND_SOC_DAPM_INPUT("AIN2"),
	SND_SOC_DAPM_INPUT("AIN3"),
	SND_SOC_DAPM_INPUT("AIN4"),
	SND_SOC_DAPM_INPUT("AIN5"),
	SND_SOC_DAPM_INPUT("AIN6"),
	SND_SOC_DAPM_INPUT("AIN7"),
	SND_SOC_DAPM_INPUT("AIN8"),

	SND_SOC_DAPM_MUX("AK5558 Ch1 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel1_mux_control),
	SND_SOC_DAPM_MUX("AK5558 Ch2 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel2_mux_control),
	SND_SOC_DAPM_MUX("AK5558 Ch3 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel3_mux_control),
	SND_SOC_DAPM_MUX("AK5558 Ch4 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel4_mux_control),
	SND_SOC_DAPM_MUX("AK5558 Ch5 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel5_mux_control),
	SND_SOC_DAPM_MUX("AK5558 Ch6 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel6_mux_control),
	SND_SOC_DAPM_MUX("AK5558 Ch7 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel7_mux_control),
	SND_SOC_DAPM_MUX("AK5558 Ch8 Enable", SND_SOC_NOPM, 0, 0,
			 &ak5558_channel8_mux_control),

	SND_SOC_DAPM_ADC("ADC Ch1", NULL, AK5558_00_POWER_MANAGEMENT1, 0, 0),
	SND_SOC_DAPM_ADC("ADC Ch2", NULL, AK5558_00_POWER_MANAGEMENT1, 1, 0),
	SND_SOC_DAPM_ADC("ADC Ch3", NULL, AK5558_00_POWER_MANAGEMENT1, 2, 0),
	SND_SOC_DAPM_ADC("ADC Ch4", NULL, AK5558_00_POWER_MANAGEMENT1, 3, 0),
	SND_SOC_DAPM_ADC("ADC Ch5", NULL, AK5558_00_POWER_MANAGEMENT1, 4, 0),
	SND_SOC_DAPM_ADC("ADC Ch6", NULL, AK5558_00_POWER_MANAGEMENT1, 5, 0),
	SND_SOC_DAPM_ADC("ADC Ch7", NULL, AK5558_00_POWER_MANAGEMENT1, 6, 0),
	SND_SOC_DAPM_ADC("ADC Ch8", NULL, AK5558_00_POWER_MANAGEMENT1, 7, 0),

	SND_SOC_DAPM_AIF_OUT("SDTO", "Capture", 0, SND_SOC_NOPM, 0, 0),

};

static int ak5558_init_reg(struct snd_soc_codec *codec);

static const struct snd_soc_dapm_route ak5558_intercon[] = {

	{"AK5558 Ch1 Enable", "On", "AIN1"},
	{"ADC Ch1", NULL, "AK5558 Ch1 Enable"},
	{"SDTO", NULL, "ADC Ch1"},

	{"AK5558 Ch2 Enable", "On", "AIN2"},
	{"ADC Ch2", NULL, "AK5558 Ch2 Enable"},
	{"SDTO", NULL, "ADC Ch2"},

	{"AK5558 Ch3 Enable", "On", "AIN3"},
	{"ADC Ch3", NULL, "AK5558 Ch3 Enable"},
	{"SDTO", NULL, "ADC Ch3"},

	{"AK5558 Ch4 Enable", "On", "AIN4"},
	{"ADC Ch4", NULL, "AK5558 Ch4 Enable"},
	{"SDTO", NULL, "ADC Ch4"},

	{"AK5558 Ch5 Enable", "On", "AIN5"},
	{"ADC Ch5", NULL, "AK5558 Ch5 Enable"},
	{"SDTO", NULL, "ADC Ch5"},

	{"AK5558 Ch6 Enable", "On", "AIN6"},
	{"ADC Ch6", NULL, "AK5558 Ch6 Enable"},
	{"SDTO", NULL, "ADC Ch6"},

	{"AK5558 Ch7 Enable", "On", "AIN7"},
	{"ADC Ch7", NULL, "AK5558 Ch7 Enable"},
	{"SDTO", NULL, "ADC Ch7"},

	{"AK5558 Ch8 Enable", "On", "AIN8"},
	{"ADC Ch8", NULL, "AK5558 Ch8 Enable"},
	{"SDTO", NULL, "ADC Ch8"},

};

static int ak5558_set_mcki(struct snd_soc_codec *codec, int fs, int rclk)
{
	u8  mode;
#ifndef AK5558_SLAVE_CKS_AUTO
	int mcki_rate;
#endif

	dev_dbg(codec->dev, "%s fs=%d rclk=%d\n", __func__, fs, rclk);

	mode = snd_soc_read(codec, AK5558_02_CONTROL1);
	mode &= ~AK5558_CKS;

#ifdef AK5558_SLAVE_CKS_AUTO
	mode |= AK5558_CKS_AUTO;
#else
	if (fs != 0 && rclk != 0) {
		if (rclk % fs)
			return -EINVAL;
		mcki_rate = rclk / fs;

		if (fs > 400000) {
			switch (mcki_rate) {
			case 32:
				mode |= AK5558_CKS_32FS_768KHZ;
				break;
			case 48:
				mode |= AK5558_CKS_48FS_768KHZ;
				break;
			case 64:
				mode |= AK5558_CKS_64FS_768KHZ;
				break;
			default:
				return -EINVAL;
			}
		} else if (fs > 200000) {
			switch (mcki_rate) {
			case 64:
				mode |= AK5558_CKS_64FS_384KHZ;
				break;
			case 96:
				mode |= AK5558_CKS_96FS_384KHZ;
				break;
			default:
				return -EINVAL;
			}
		} else if (fs > 108000) {
			switch (mcki_rate) {
			case 128:
				mode |= AK5558_CKS_128FS_192KHZ;
				break;
			case 192:
				mode |= AK5558_CKS_192FS_192KHZ;
				break;
			default:
				return -EINVAL;
			}
		} else if (fs > 54000) {
			switch (mcki_rate) {
			case 256:
				mode |= AK5558_CKS_256FS_96KHZ;
				break;
			case 384:
				mode |= AK5558_CKS_384FS_96KHZ;
				break;
			default:
				return -EINVAL;
			}
		} else {
			switch (mcki_rate) {
			case 256:
				mode |= AK5558_CKS_256FS_48KHZ;
				break;
			case 384:
				mode |= AK5558_CKS_384FS_48KHZ;
				break;
			case 512:
				mode |= AK5558_CKS_512FS_48KHZ;
				break;
			case 768:
				mode |= AK5558_CKS_768FS_48KHZ;
				break;
			case 1024:
				if (fs > 32000)
					return -EINVAL;
				mode |= AK5558_CKS_1024FS_16KHZ;
				break;
			default:
				return -EINVAL;
			}
		}
	}
#endif

	snd_soc_update_bits(codec, AK5558_02_CONTROL1, AK5558_CKS,  mode);

	return 0;
}

static int ak5558_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak5558_priv *ak5558 = snd_soc_codec_get_drvdata(codec);
	u8 bits;

	dev_dbg(dai->dev, "%s(%d)\n", __func__, __LINE__);

	/* set master/slave audio interface */
	bits = snd_soc_read(codec, AK5558_02_CONTROL1);
	bits &= ~AK5558_BITS;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S24_LE:
		bits |= AK5558_DIF_24BIT_MODE;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		bits |= AK5558_DIF_32BIT_MODE;
		break;
	default:
		return -EINVAL;
	}

	ak5558->fs = params_rate(params);
	snd_soc_update_bits(codec, AK5558_02_CONTROL1, AK5558_BITS, bits);

	ak5558_set_mcki(codec, ak5558->fs, ak5558->rclk);

	return 0;
}

static int ak5558_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				 unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak5558_priv *ak5558 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(dai->dev, "%s(%d)\n", __func__, __LINE__);

	ak5558->rclk = freq;
	ak5558_set_mcki(codec, ak5558->fs, ak5558->rclk);

	return 0;
}

static int ak5558_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

	struct snd_soc_codec *codec = dai->codec;
	u8 format;

	dev_dbg(dai->dev, "%s(%d)\n", __func__, __LINE__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
#ifdef AK5558_SLAVE_CKS_AUTO
		break;
#else
		return -EINVAL;
#endif
	case SND_SOC_DAIFMT_CBS_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
	default:
		dev_err(codec->dev, "Clock mode unsupported");
		return -EINVAL;
	}

	/* set master/slave audio interface */
	format = snd_soc_read(codec, AK5558_02_CONTROL1);
	format &= ~AK5558_DIF;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= AK5558_DIF_I2S_MODE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= AK5558_DIF_MSB_MODE;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec,
				  AK5558_02_CONTROL1, AK5558_DIF, format);

	return 0;
}

static int ak5558_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak5558_priv *ak5558 = snd_soc_codec_get_drvdata(codec);
	int ndt;

	if (mute) {
		ndt = 0;
		if (ak5558->fs != 0)
			ndt = 583000 / ak5558->fs;
		if (ndt < 5)
			ndt = 5;
		msleep(ndt);
	}

	return 0;
}

static int ak5558_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{

	dev_dbg(codec->dev, "%s bias level=%d\n", __func__, (int)level);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		ak5558_init_reg(codec);
		break;
	}
	return 0;
}

#define AK5558_RATES SNDRV_PCM_RATE_8000_192000

#define AK5558_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

static const unsigned int ak5558_rates[] = {
	8000, 11025,  16000, 22050,
	32000, 44100, 48000, 88200,
	96000, 176400, 192000, 352800,
	384000, 705600, 768000, 1411200,
	2822400,
};

static const struct snd_pcm_hw_constraint_list ak5558_rate_constraints = {
	.count = ARRAY_SIZE(ak5558_rates),
	.list = ak5558_rates,
};

static int ak5558_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai) {
		int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &ak5558_rate_constraints);

	return ret;
}

static struct snd_soc_dai_ops ak5558_dai_ops = {
	.startup        = ak5558_startup,
	.hw_params	= ak5558_hw_params,
	.set_sysclk	= ak5558_set_dai_sysclk,
	.set_fmt	= ak5558_set_dai_fmt,
	.digital_mute	= ak5558_set_dai_mute,
};

static struct snd_soc_dai_driver ak5558_dai = {
	.name = "ak5558-aif",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = AK5558_FORMATS,
	},
	.ops = &ak5558_dai_ops,
};

static int ak5558_init_reg(struct snd_soc_codec *codec)
{
	int  ret;
	struct ak5558_priv *ak5558 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	usleep_range(10000, 11000);
	if (gpio_is_valid(ak5558->pdn_gpio)) {
		gpio_set_value_cansleep(ak5558->pdn_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(ak5558->pdn_gpio, 1);
		usleep_range(1000, 2000);
	}

	ret = snd_soc_write(codec, AK5558_00_POWER_MANAGEMENT1, 0x0);
	if (ret < 0)
		return ret;

	ret = snd_soc_update_bits(codec, AK5558_02_CONTROL1, AK5558_CKS,
				  AK5558_CKS_AUTO);
	if (ret < 0)
		return ret;

	return 0;
}

static int ak5558_probe(struct snd_soc_codec *codec)
{
	struct ak5558_priv *ak5558 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	ret = ak5558_init_reg(codec);

	ak5558->fs = 48000;
	ak5558->rclk = 0;

	return ret;
}

static int ak5558_remove(struct snd_soc_codec *codec)
{
	struct ak5558_priv *ak5558 = snd_soc_codec_get_drvdata(codec);

	ak5558_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if (gpio_is_valid(ak5558->pdn_gpio)) {
		gpio_set_value_cansleep(ak5558->pdn_gpio, 0);
		usleep_range(1000, 2000);
	}

	return 0;
}

#ifdef CONFIG_PM
static int ak5558_runtime_suspend(struct device *dev)
{
	struct ak5558_priv *ak5558 = dev_get_drvdata(dev);

	regcache_cache_only(ak5558->regmap, true);

	if (gpio_is_valid(ak5558->pdn_gpio)) {
		gpio_set_value_cansleep(ak5558->pdn_gpio, 0);
		usleep_range(1000, 2000);
	}

	return 0;
}

static int ak5558_runtime_resume(struct device *dev)
{
	struct ak5558_priv *ak5558 = dev_get_drvdata(dev);

	if (gpio_is_valid(ak5558->pdn_gpio)) {
		gpio_set_value_cansleep(ak5558->pdn_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(ak5558->pdn_gpio, 1);
		usleep_range(1000, 2000);

	}

	regcache_cache_only(ak5558->regmap, false);
	regcache_mark_dirty(ak5558->regmap);

	return regcache_sync(ak5558->regmap);
}
#endif

const struct dev_pm_ops ak5558_pm = {
	SET_RUNTIME_PM_OPS(ak5558_runtime_suspend, ak5558_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

struct snd_soc_codec_driver soc_codec_dev_ak5558 = {
	.probe = ak5558_probe,
	.remove = ak5558_remove,
	.idle_bias_off = true,
	.set_bias_level = ak5558_set_bias_level,

	.component_driver = {
		.controls = ak5558_snd_controls,
		.num_controls = ARRAY_SIZE(ak5558_snd_controls),
		.dapm_widgets = ak5558_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(ak5558_dapm_widgets),
		.dapm_routes = ak5558_intercon,
		.num_dapm_routes = ARRAY_SIZE(ak5558_intercon),
	},
};

static const struct regmap_config ak5558_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK5558_05_DSD,
	.reg_defaults = ak5558_reg,
	.num_reg_defaults = ARRAY_SIZE(ak5558_reg),
	.cache_type = REGCACHE_RBTREE,
};

static int ak5558_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct device_node *np = i2c->dev.of_node;
	struct ak5558_priv *ak5558;
	int ret = 0;

	dev_err(&i2c->dev, "%s(%d)\n", __func__, __LINE__);

	ak5558 = devm_kzalloc(&i2c->dev, sizeof(struct ak5558_priv),
			      GFP_KERNEL);
	if (ak5558 == NULL)
		return -ENOMEM;

	ak5558->regmap = devm_regmap_init_i2c(i2c, &ak5558_regmap);
	if (IS_ERR(ak5558->regmap))
		return PTR_ERR(ak5558->regmap);

	i2c_set_clientdata(i2c, ak5558);
	ak5558->i2c = i2c;

	ak5558->pdn_gpio = of_get_named_gpio(np, "ak5558,pdn-gpio", 0);
	if (gpio_is_valid(ak5558->pdn_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, ak5558->pdn_gpio,
				GPIOF_OUT_INIT_LOW, "ak5558,pdn");
		if (ret) {
			dev_err(&i2c->dev, "unable to get pdn gpio\n");
			return ret;
		}
	}

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_ak5558,
				     &ak5558_dai, 1);
	if (ret)
		return ret;

	pm_runtime_enable(&i2c->dev);

	return 0;
}

static int ak5558_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	pm_runtime_disable(&client->dev);


	return 0;
}

static const struct of_device_id ak5558_i2c_dt_ids[] = {
	{ .compatible = "asahi-kasei,ak5558"},
	{ }
};

static const struct i2c_device_id ak5558_i2c_id[] = {
	{ "ak5558", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak5558_i2c_id);

static struct i2c_driver ak5558_i2c_driver = {
	.driver = {
		.name = "ak5558",
		.of_match_table = of_match_ptr(ak5558_i2c_dt_ids),
		.pm = &ak5558_pm,
	},
	.probe = ak5558_i2c_probe,
	.remove = ak5558_i2c_remove,
	.id_table = ak5558_i2c_id,
};

module_i2c_driver(ak5558_i2c_driver);

MODULE_AUTHOR("Junichi Wakasugi <wakasugi.jb@om.asahi-kasei.co.jp>");
MODULE_AUTHOR("Mihai Serban <mihai.serban@nxp.com>");
MODULE_DESCRIPTION("ASoC AK5558 ADC driver");
MODULE_LICENSE("GPL");
