/*
 * Cirrus Logic CS42448/CS42888 Audio CODEC Digital Audio Interface (DAI) driver
 *
 * Copyright (C) 2014-2016 Freescale Semiconductor, Inc.
 *
 * Author: Nicolin Chen <Guangyu.Chen@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "rpmsg_cs42xx8.h"
#include "../fsl/fsl_rpmsg_i2s.h"

#define CS42XX8_NUM_SUPPLIES 4
static const char *const cs42xx8_supply_names[CS42XX8_NUM_SUPPLIES] = {
	"VA",
	"VD",
	"VLS",
	"VLC",
};

#define CS42XX8_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_LE)

/* codec private data */
struct rpmsg_cs42xx8_priv {
	struct regulator_bulk_data supplies[CS42XX8_NUM_SUPPLIES];
	struct cs42xx8_driver_data *drvdata;
	struct regmap *regmap;
	struct clk *clk;

	bool slave_mode[2];
	unsigned long sysclk;
	u32 tx_channels;
	int rate[2];
	int reset_gpio;
	int audioindex;
	struct fsl_rpmsg_i2s *rpmsg_i2s;
};

/* -127.5dB to 0dB with step of 0.5dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -12750, 50, 1);
/* -64dB to 24dB with step of 0.5dB */
static const DECLARE_TLV_DB_SCALE(adc_tlv, -6400, 50, 0);

static const char *const cs42xx8_adc_single[] = { "Differential", "Single-Ended" };
static const char *const cs42xx8_szc[] = { "Immediate Change", "Zero Cross",
					"Soft Ramp", "Soft Ramp on Zero Cross" };

static const struct soc_enum adc1_single_enum =
	SOC_ENUM_SINGLE(CS42XX8_ADCCTL, 4, 2, cs42xx8_adc_single);
static const struct soc_enum adc2_single_enum =
	SOC_ENUM_SINGLE(CS42XX8_ADCCTL, 3, 2, cs42xx8_adc_single);
static const struct soc_enum adc3_single_enum =
	SOC_ENUM_SINGLE(CS42XX8_ADCCTL, 2, 2, cs42xx8_adc_single);
static const struct soc_enum dac_szc_enum =
	SOC_ENUM_SINGLE(CS42XX8_TXCTL, 5, 4, cs42xx8_szc);
static const struct soc_enum adc_szc_enum =
	SOC_ENUM_SINGLE(CS42XX8_TXCTL, 0, 4, cs42xx8_szc);

static const struct snd_kcontrol_new cs42xx8_snd_controls[] = {
	SOC_DOUBLE_R_TLV("DAC1 Playback Volume", CS42XX8_VOLAOUT1,
			 CS42XX8_VOLAOUT2, 0, 0xff, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("DAC2 Playback Volume", CS42XX8_VOLAOUT3,
			 CS42XX8_VOLAOUT4, 0, 0xff, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("DAC3 Playback Volume", CS42XX8_VOLAOUT5,
			 CS42XX8_VOLAOUT6, 0, 0xff, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("DAC4 Playback Volume", CS42XX8_VOLAOUT7,
			 CS42XX8_VOLAOUT8, 0, 0xff, 1, dac_tlv),
	SOC_DOUBLE_R_S_TLV("ADC1 Capture Volume", CS42XX8_VOLAIN1,
			   CS42XX8_VOLAIN2, 0, -0x80, 0x30, 7, 0, adc_tlv),
	SOC_DOUBLE_R_S_TLV("ADC2 Capture Volume", CS42XX8_VOLAIN3,
			   CS42XX8_VOLAIN4, 0, -0x80, 0x30, 7, 0, adc_tlv),
	SOC_DOUBLE("DAC1 Invert Switch", CS42XX8_DACINV, 0, 1, 1, 0),
	SOC_DOUBLE("DAC2 Invert Switch", CS42XX8_DACINV, 2, 3, 1, 0),
	SOC_DOUBLE("DAC3 Invert Switch", CS42XX8_DACINV, 4, 5, 1, 0),
	SOC_DOUBLE("DAC4 Invert Switch", CS42XX8_DACINV, 6, 7, 1, 0),
	SOC_DOUBLE("ADC1 Invert Switch", CS42XX8_ADCINV, 0, 1, 1, 0),
	SOC_DOUBLE("ADC2 Invert Switch", CS42XX8_ADCINV, 2, 3, 1, 0),
	SOC_SINGLE("ADC High-Pass Filter Switch", CS42XX8_ADCCTL, 7, 1, 1),
	SOC_SINGLE("DAC De-emphasis Switch", CS42XX8_ADCCTL, 5, 1, 0),
	SOC_ENUM("ADC1 Single Ended Mode Switch", adc1_single_enum),
	SOC_ENUM("ADC2 Single Ended Mode Switch", adc2_single_enum),
	SOC_SINGLE("DAC Single Volume Control Switch", CS42XX8_TXCTL, 7, 1, 0),
	SOC_ENUM("DAC Soft Ramp & Zero Cross Control Switch", dac_szc_enum),
	SOC_SINGLE("DAC Auto Mute Switch", CS42XX8_TXCTL, 4, 1, 0),
	SOC_SINGLE("Mute ADC Serial Port Switch", CS42XX8_TXCTL, 3, 1, 0),
	SOC_SINGLE("ADC Single Volume Control Switch", CS42XX8_TXCTL, 2, 1, 0),
	SOC_ENUM("ADC Soft Ramp & Zero Cross Control Switch", adc_szc_enum),
};

static const struct snd_kcontrol_new cs42xx8_adc3_snd_controls[] = {
	SOC_DOUBLE_R_S_TLV("ADC3 Capture Volume", CS42XX8_VOLAIN5,
			   CS42XX8_VOLAIN6, 0, -0x80, 0x30, 7, 0, adc_tlv),
	SOC_DOUBLE("ADC3 Invert Switch", CS42XX8_ADCINV, 4, 5, 1, 0),
	SOC_ENUM("ADC3 Single Ended Mode Switch", adc3_single_enum),
};

static const struct snd_soc_dapm_widget cs42xx8_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC1", "Playback", CS42XX8_PWRCTL, 1, 1),
	SND_SOC_DAPM_DAC("DAC2", "Playback", CS42XX8_PWRCTL, 2, 1),
	SND_SOC_DAPM_DAC("DAC3", "Playback", CS42XX8_PWRCTL, 3, 1),
	SND_SOC_DAPM_DAC("DAC4", "Playback", CS42XX8_PWRCTL, 4, 1),

	SND_SOC_DAPM_OUTPUT("AOUT1L"),
	SND_SOC_DAPM_OUTPUT("AOUT1R"),
	SND_SOC_DAPM_OUTPUT("AOUT2L"),
	SND_SOC_DAPM_OUTPUT("AOUT2R"),
	SND_SOC_DAPM_OUTPUT("AOUT3L"),
	SND_SOC_DAPM_OUTPUT("AOUT3R"),
	SND_SOC_DAPM_OUTPUT("AOUT4L"),
	SND_SOC_DAPM_OUTPUT("AOUT4R"),

	SND_SOC_DAPM_ADC("ADC1", "Capture", CS42XX8_PWRCTL, 5, 1),
	SND_SOC_DAPM_ADC("ADC2", "Capture", CS42XX8_PWRCTL, 6, 1),

	SND_SOC_DAPM_INPUT("AIN1L"),
	SND_SOC_DAPM_INPUT("AIN1R"),
	SND_SOC_DAPM_INPUT("AIN2L"),
	SND_SOC_DAPM_INPUT("AIN2R"),

};

static const struct snd_soc_dapm_widget cs42xx8_adc3_dapm_widgets[] = {
	SND_SOC_DAPM_ADC("ADC3", "Capture", CS42XX8_PWRCTL, 7, 1),

	SND_SOC_DAPM_INPUT("AIN3L"),
	SND_SOC_DAPM_INPUT("AIN3R"),
};

static const struct snd_soc_dapm_route cs42xx8_dapm_routes[] = {
	/* Playback */
	{ "AOUT1L", NULL, "DAC1" },
	{ "AOUT1R", NULL, "DAC1" },

	{ "AOUT2L", NULL, "DAC2" },
	{ "AOUT2R", NULL, "DAC2" },

	{ "AOUT3L", NULL, "DAC3" },
	{ "AOUT3R", NULL, "DAC3" },

	{ "AOUT4L", NULL, "DAC4" },
	{ "AOUT4R", NULL, "DAC4" },

	/* Capture */
	{ "ADC1", NULL, "AIN1L" },
	{ "ADC1", NULL, "AIN1R" },

	{ "ADC2", NULL, "AIN2L" },
	{ "ADC2", NULL, "AIN2R" },
};

static const struct snd_soc_dapm_route cs42xx8_adc3_dapm_routes[] = {
	/* Capture */
	{ "ADC3", NULL, "AIN3L" },
	{ "ADC3", NULL, "AIN3R" },
};

struct cs42xx8_ratios {
	unsigned int mfreq;
	unsigned int min_mclk;
	unsigned int max_mclk;
	unsigned int ratio[3];
};

static const struct cs42xx8_ratios cs42xx8_ratios[] = {
	{ 0, 1029000, 12800000, {256, 128, 64} },
	{ 2, 1536000, 19200000, {384, 192, 96} },
	{ 4, 2048000, 25600000, {512, 256, 128} },
	{ 6, 3072000, 38400000, {768, 384, 192} },
	{ 8, 4096000, 51200000, {1024, 512, 256} },
};

static int cs42xx8_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rpmsg_cs42xx8_priv *cs42xx8 = snd_soc_codec_get_drvdata(codec);

	cs42xx8->sysclk = freq;

	return 0;
}

static int cs42xx8_set_dai_fmt(struct snd_soc_dai *codec_dai,
			       unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rpmsg_cs42xx8_priv *cs42xx8 = snd_soc_codec_get_drvdata(codec);
	u32 val;

	/* Set DAI format */
	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J:
		val = CS42XX8_INTF_DAC_DIF_LEFTJ | CS42XX8_INTF_ADC_DIF_LEFTJ;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = CS42XX8_INTF_DAC_DIF_I2S | CS42XX8_INTF_ADC_DIF_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val = CS42XX8_INTF_DAC_DIF_RIGHTJ | CS42XX8_INTF_ADC_DIF_RIGHTJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		val = CS42XX8_INTF_DAC_DIF_TDM | CS42XX8_INTF_ADC_DIF_TDM;
		break;
	default:
		dev_err(codec->dev, "unsupported dai format\n");
		return -EINVAL;
	}

	regmap_update_bits(cs42xx8->regmap, CS42XX8_INTF,
			   CS42XX8_INTF_DAC_DIF_MASK |
			   CS42XX8_INTF_ADC_DIF_MASK, val);

	if (cs42xx8->slave_mode[0] == cs42xx8->slave_mode[1]) {
		/* Set master/slave audio interface */
		switch (format & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS:
			cs42xx8->slave_mode[0] = true;
			cs42xx8->slave_mode[1] = true;
			break;
		case SND_SOC_DAIFMT_CBM_CFM:
			cs42xx8->slave_mode[0] = false;
			cs42xx8->slave_mode[1] = false;
			break;
		default:
			dev_err(codec->dev, "unsupported master/slave mode\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int cs42xx8_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rpmsg_cs42xx8_priv *cs42xx8 = snd_soc_codec_get_drvdata(codec);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 ratio[2];
	u32 rate[2];
	u32 fm[2];
	u32 i, val, mask;
	bool condition1, condition2;

	if (tx)
		cs42xx8->tx_channels = params_channels(params);

	rate[tx]  = params_rate(params);
	rate[!tx] = cs42xx8->rate[!tx];

	ratio[tx] = rate[tx] > 0 ? cs42xx8->sysclk / rate[tx] : 0;
	ratio[!tx] = rate[!tx] > 0 ? cs42xx8->sysclk / rate[!tx] : 0;

	for (i = 0; i < 2; i++) {
		if (cs42xx8->slave_mode[i]) {
			fm[i] = CS42XX8_FM_AUTO;
		} else {
			if (rate[i] < 50000)
				fm[i] = CS42XX8_FM_SINGLE;
			else if (rate[i] > 50000 && rate[i] < 100000)
				fm[i] = CS42XX8_FM_DOUBLE;
			else if (rate[i] > 100000 && rate[i] < 200000)
				fm[i] = CS42XX8_FM_QUAD;
			else {
				dev_err(codec->dev,
				"unsupported sample rate or rate combine\n");
				return -EINVAL;
			}
		}
	}

	for (i = 0; i < ARRAY_SIZE(cs42xx8_ratios); i++) {
		condition1 = ((fm[tx] == CS42XX8_FM_AUTO) ?
			(cs42xx8_ratios[i].ratio[0] == ratio[tx] ||
			cs42xx8_ratios[i].ratio[1] == ratio[tx] ||
			cs42xx8_ratios[i].ratio[2] == ratio[tx]) :
			(cs42xx8_ratios[i].ratio[fm[tx]] == ratio[tx])) &&
			cs42xx8->sysclk >= cs42xx8_ratios[i].min_mclk &&
			cs42xx8->sysclk <= cs42xx8_ratios[i].max_mclk;

		if (ratio[tx] <= 0)
			condition1 = true;

		condition2 = ((fm[!tx] == CS42XX8_FM_AUTO) ?
			(cs42xx8_ratios[i].ratio[0] == ratio[!tx] ||
			cs42xx8_ratios[i].ratio[1] == ratio[!tx] ||
			cs42xx8_ratios[i].ratio[2] == ratio[!tx]) :
			(cs42xx8_ratios[i].ratio[fm[!tx]] == ratio[!tx]));

		if (ratio[!tx] <= 0)
			condition2 = true;

		if (condition1 && condition2)
			break;
	}

	if (i == ARRAY_SIZE(cs42xx8_ratios)) {
		dev_err(codec->dev, "unsupported sysclk ratio\n");
		return -EINVAL;
	}

	cs42xx8->rate[tx] = params_rate(params);

	mask = CS42XX8_FUNCMOD_MFREQ_MASK;
	val = cs42xx8_ratios[i].mfreq;

	regmap_update_bits(cs42xx8->regmap, CS42XX8_FUNCMOD,
			   CS42XX8_FUNCMOD_xC_FM_MASK(tx) | mask,
			   CS42XX8_FUNCMOD_xC_FM(tx, fm[tx]) | val);

	return 0;
}

static int cs42xx8_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct rpmsg_cs42xx8_priv *cs42xx8 = snd_soc_codec_get_drvdata(codec);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	cs42xx8->rate[tx] = 0;

	regmap_update_bits(cs42xx8->regmap, CS42XX8_FUNCMOD,
			   CS42XX8_FUNCMOD_xC_FM_MASK(tx),
			   CS42XX8_FUNCMOD_xC_FM(tx, CS42XX8_FM_AUTO));
	return 0;
}

static int cs42xx8_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rpmsg_cs42xx8_priv *cs42xx8 = snd_soc_codec_get_drvdata(codec);
	u8 dac_unmute = cs42xx8->tx_channels ?
			~((0x1 << cs42xx8->tx_channels) - 1) : 0;

	regmap_write(cs42xx8->regmap, CS42XX8_DACMUTE,
		     mute ? CS42XX8_DACMUTE_ALL : dac_unmute);

	return 0;
}

static const struct snd_soc_dai_ops cs42xx8_dai_ops = {
	.set_fmt	= cs42xx8_set_dai_fmt,
	.set_sysclk	= cs42xx8_set_dai_sysclk,
	.hw_params	= cs42xx8_hw_params,
	.hw_free	= cs42xx8_hw_free,
	.digital_mute	= cs42xx8_digital_mute,
};

static struct snd_soc_dai_driver cs42xx8_dai = {
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = CS42XX8_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = CS42XX8_FORMATS,
	},
	.ops = &cs42xx8_dai_ops,
};

static const struct reg_default cs42xx8_reg[] = {
	{ 0x02, 0x00 },   /* Power Control */
	{ 0x03, 0xF0 },   /* Functional Mode */
	{ 0x04, 0x46 },   /* Interface Formats */
	{ 0x05, 0x00 },   /* ADC Control & DAC De-Emphasis */
	{ 0x06, 0x10 },   /* Transition Control */
	{ 0x07, 0x00 },   /* DAC Channel Mute */
	{ 0x08, 0x00 },   /* Volume Control AOUT1 */
	{ 0x09, 0x00 },   /* Volume Control AOUT2 */
	{ 0x0a, 0x00 },   /* Volume Control AOUT3 */
	{ 0x0b, 0x00 },   /* Volume Control AOUT4 */
	{ 0x0c, 0x00 },   /* Volume Control AOUT5 */
	{ 0x0d, 0x00 },   /* Volume Control AOUT6 */
	{ 0x0e, 0x00 },   /* Volume Control AOUT7 */
	{ 0x0f, 0x00 },   /* Volume Control AOUT8 */
	{ 0x10, 0x00 },   /* DAC Channel Invert */
	{ 0x11, 0x00 },   /* Volume Control AIN1 */
	{ 0x12, 0x00 },   /* Volume Control AIN2 */
	{ 0x13, 0x00 },   /* Volume Control AIN3 */
	{ 0x14, 0x00 },   /* Volume Control AIN4 */
	{ 0x15, 0x00 },   /* Volume Control AIN5 */
	{ 0x16, 0x00 },   /* Volume Control AIN6 */
	{ 0x17, 0x00 },   /* ADC Channel Invert */
	{ 0x18, 0x00 },   /* Status Control */
	{ 0x1a, 0x00 },   /* Status Mask */
	{ 0x1b, 0x00 },   /* MUTEC Pin Control */
};

static bool cs42xx8_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS42XX8_STATUS:
		return true;
	default:
		return false;
	}
}

static bool cs42xx8_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS42XX8_CHIPID:
	case CS42XX8_STATUS:
		return false;
	default:
		return true;
	}
}

static int rpmsg_cs42xx8_read(void *context, unsigned int reg, unsigned int *val)
{
	struct rpmsg_cs42xx8_priv *cs42xx8 = context;
	struct fsl_rpmsg_i2s *rpmsg_i2s = cs42xx8->rpmsg_i2s;
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->rpmsg[GET_CODEC_VALUE].send_msg;
	int err, reg_val;

	mutex_lock(&i2s_info->i2c_lock);
	rpmsg->param.audioindex = cs42xx8->audioindex;
	rpmsg->param.buffer_addr = reg;
	rpmsg->header.cmd = GET_CODEC_VALUE;
	err = i2s_info->send_message(&i2s_info->rpmsg[GET_CODEC_VALUE], i2s_info);
	reg_val = i2s_info->rpmsg[GET_CODEC_VALUE].recv_msg.param.reg_data;
	mutex_unlock(&i2s_info->i2c_lock);
	if (err)
		return -EIO;

	*val = reg_val;
	return 0;
}

static int rpmsg_cs42xx8_write(void *context, unsigned int reg, unsigned int val)
{
	struct rpmsg_cs42xx8_priv *cs42xx8 = context;
	struct fsl_rpmsg_i2s *rpmsg_i2s = cs42xx8->rpmsg_i2s;
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->rpmsg[SET_CODEC_VALUE].send_msg;
	int err;

	mutex_lock(&i2s_info->i2c_lock);
	rpmsg->param.audioindex = cs42xx8->audioindex;
	rpmsg->param.buffer_addr = reg;
	rpmsg->param.buffer_size = val;
	rpmsg->header.cmd = SET_CODEC_VALUE;
	err = i2s_info->send_message(&i2s_info->rpmsg[SET_CODEC_VALUE], i2s_info);
	mutex_unlock(&i2s_info->i2c_lock);
	if (err)
		return -EIO;

	return 0;
}

static struct regmap_config rpmsg_cs42xx8_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = CS42XX8_LASTREG,
	.reg_defaults = cs42xx8_reg,
	.num_reg_defaults = ARRAY_SIZE(cs42xx8_reg),
	.volatile_reg = cs42xx8_volatile_register,
	.writeable_reg = cs42xx8_writeable_register,
	.cache_type = REGCACHE_RBTREE,

	.reg_read = rpmsg_cs42xx8_read,
	.reg_write = rpmsg_cs42xx8_write,
};

static int cs42xx8_codec_probe(struct snd_soc_codec *codec)
{
	struct rpmsg_cs42xx8_priv *cs42xx8 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	switch (cs42xx8->drvdata->num_adcs) {
	case 3:
		snd_soc_add_codec_controls(codec, cs42xx8_adc3_snd_controls,
					ARRAY_SIZE(cs42xx8_adc3_snd_controls));
		snd_soc_dapm_new_controls(dapm, cs42xx8_adc3_dapm_widgets,
					ARRAY_SIZE(cs42xx8_adc3_dapm_widgets));
		snd_soc_dapm_add_routes(dapm, cs42xx8_adc3_dapm_routes,
					ARRAY_SIZE(cs42xx8_adc3_dapm_routes));
		break;
	default:
		break;
	}

	/* Mute all DAC channels */
	regmap_write(cs42xx8->regmap, CS42XX8_DACMUTE, CS42XX8_DACMUTE_ALL);
	regmap_update_bits(cs42xx8->regmap, CS42XX8_PWRCTL,
			CS42XX8_PWRCTL_PDN_MASK, 0);
	return 0;
}

static const struct snd_soc_codec_driver cs42xx8_driver = {
	.probe = cs42xx8_codec_probe,
	.idle_bias_off = true,

	.component_driver = {
		.controls		= cs42xx8_snd_controls,
		.num_controls		= ARRAY_SIZE(cs42xx8_snd_controls),
		.dapm_widgets		= cs42xx8_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(cs42xx8_dapm_widgets),
		.dapm_routes		= cs42xx8_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(cs42xx8_dapm_routes),
	},
};

static int rpmsg_cs42xx8_codec_probe(struct platform_device *pdev)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(pdev->dev.parent);
	struct fsl_rpmsg_codec *pdata = pdev->dev.platform_data;
	struct rpmsg_cs42xx8_priv *cs42xx8;
	struct device *dev = &pdev->dev;
	int ret, val, i;

	cs42xx8 = devm_kzalloc(&pdev->dev, sizeof(*cs42xx8), GFP_KERNEL);
	if (cs42xx8 == NULL)
		return -ENOMEM;

	cs42xx8->regmap = devm_regmap_init(&pdev->dev, NULL,
				cs42xx8, &rpmsg_cs42xx8_regmap_config);
	if (IS_ERR(cs42xx8->regmap))
		return PTR_ERR(cs42xx8->regmap);

	dev_set_drvdata(&pdev->dev, cs42xx8);

	cs42xx8->drvdata = devm_kzalloc(&pdev->dev,
			sizeof(struct cs42xx8_driver_data), GFP_KERNEL);
	if (!cs42xx8->drvdata)
		return -ENOMEM;

	cs42xx8->rpmsg_i2s = rpmsg_i2s;

	memcpy(cs42xx8->drvdata->name, pdata->name, 32);
	cs42xx8->drvdata->num_adcs = pdata->num_adcs;
	cs42xx8->audioindex = pdata->audioindex;
	cs42xx8->reset_gpio = of_get_named_gpio(pdev->dev.parent->of_node,
						"reset-gpio", 0);
	if (gpio_is_valid(cs42xx8->reset_gpio)) {
		ret = devm_gpio_request_one(dev, cs42xx8->reset_gpio,
				GPIOF_OUT_INIT_LOW, "cs42xx8 reset");
		if (ret) {
			dev_err(dev, "unable to get reset gpio\n");
			return ret;
		}
		gpio_set_value_cansleep(cs42xx8->reset_gpio, 1);
	}

	cs42xx8->clk = devm_clk_get(pdev->dev.parent, "mclk");
	if (IS_ERR(cs42xx8->clk)) {
		dev_err(dev, "failed to get the clock: %ld\n",
				PTR_ERR(cs42xx8->clk));
		return -EINVAL;
	}

	cs42xx8->sysclk = clk_get_rate(cs42xx8->clk);

	if (of_property_read_bool(pdev->dev.parent->of_node, "fsl,txm-rxs")) {
		/* 0 --  rx,  1 -- tx */
		cs42xx8->slave_mode[0] = true;
		cs42xx8->slave_mode[1] = false;
	}

	if (of_property_read_bool(pdev->dev.parent->of_node, "fsl,txs-rxm")) {
		/* 0 --  rx,  1 -- tx */
		cs42xx8->slave_mode[0] = false;
		cs42xx8->slave_mode[1] = true;
	}

	for (i = 0; i < ARRAY_SIZE(cs42xx8->supplies); i++)
		cs42xx8->supplies[i].supply = cs42xx8_supply_names[i];

	ret = devm_regulator_bulk_get(pdev->dev.parent,
			ARRAY_SIZE(cs42xx8->supplies), cs42xx8->supplies);
	if (ret) {
		dev_err(dev, "failed to request supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(cs42xx8->supplies),
				    cs42xx8->supplies);
	if (ret) {
		dev_err(dev, "failed to enable supplies: %d\n", ret);
		return ret;
	}

	/* Make sure hardware reset done */
	usleep_range(5000, 10000);

	/*
	 * We haven't marked the chip revision as volatile due to
	 * sharing a register with the right input volume; explicitly
	 * bypass the cache to read it.
	 */
	regcache_cache_bypass(cs42xx8->regmap, true);

	/* Validate the chip ID */
	ret = regmap_read(cs42xx8->regmap, CS42XX8_CHIPID, &val);
	if (ret < 0) {
		dev_err(dev, "failed to get device ID, ret = %d", ret);
		goto err_enable;
	}

	/* The top four bits of the chip ID should be 0000 */
	if (((val & CS42XX8_CHIPID_CHIP_ID_MASK) >> 4) != 0x00) {
		dev_err(dev, "unmatched chip ID: %d\n",
			(val & CS42XX8_CHIPID_CHIP_ID_MASK) >> 4);
		ret = -EINVAL;
		goto err_enable;
	}

	dev_info(dev, "found device, revision %X\n",
			val & CS42XX8_CHIPID_REV_ID_MASK);

	regcache_cache_bypass(cs42xx8->regmap, false);

	cs42xx8_dai.name = cs42xx8->drvdata->name;

	/* Each adc supports stereo input */
	cs42xx8_dai.capture.channels_max = cs42xx8->drvdata->num_adcs * 2;

	pm_runtime_enable(dev);
	pm_request_idle(dev);

	ret = snd_soc_register_codec(dev, &cs42xx8_driver, &cs42xx8_dai, 1);
	if (ret) {
		dev_err(dev, "failed to register codec:%d\n", ret);
		goto err_enable;
	}

	regcache_cache_only(cs42xx8->regmap, true);

err_enable:
	regulator_bulk_disable(ARRAY_SIZE(cs42xx8->supplies),
			       cs42xx8->supplies);

	return ret;
}

#ifdef CONFIG_PM
static int cs42xx8_runtime_resume(struct device *dev)
{
	struct rpmsg_cs42xx8_priv *cs42xx8 = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(cs42xx8->clk);
	if (ret) {
		dev_err(dev, "failed to enable mclk: %d\n", ret);
		return ret;
	}

	if (gpio_is_valid(cs42xx8->reset_gpio)) {
		gpio_set_value_cansleep(cs42xx8->reset_gpio, 0);
		gpio_set_value_cansleep(cs42xx8->reset_gpio, 1);
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(cs42xx8->supplies),
				    cs42xx8->supplies);
	if (ret) {
		dev_err(dev, "failed to enable supplies: %d\n", ret);
		goto err_clk;
	}

	regmap_update_bits(cs42xx8->regmap, CS42XX8_PWRCTL,
				CS42XX8_PWRCTL_PDN_MASK, 1);
	/* Make sure hardware reset done */
	usleep_range(5000, 10000);

	regmap_update_bits(cs42xx8->regmap, CS42XX8_PWRCTL,
				CS42XX8_PWRCTL_PDN_MASK, 0);

	regcache_cache_only(cs42xx8->regmap, false);

	ret = regcache_sync(cs42xx8->regmap);
	if (ret) {
		dev_err(dev, "failed to sync regmap: %d\n", ret);
		goto err_bulk;
	}

	return 0;

err_bulk:
	regulator_bulk_disable(ARRAY_SIZE(cs42xx8->supplies),
			       cs42xx8->supplies);
err_clk:
	clk_disable_unprepare(cs42xx8->clk);

	return ret;
}

static int cs42xx8_runtime_suspend(struct device *dev)
{
	struct rpmsg_cs42xx8_priv *cs42xx8 = dev_get_drvdata(dev);

	regcache_cache_only(cs42xx8->regmap, true);

	regulator_bulk_disable(ARRAY_SIZE(cs42xx8->supplies),
			       cs42xx8->supplies);

	clk_disable_unprepare(cs42xx8->clk);

	return 0;
}
#endif

const struct dev_pm_ops rpmsg_cs42xx8_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(cs42xx8_runtime_suspend, cs42xx8_runtime_resume, NULL)
};

static int rpmsg_cs42xx8_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver rpmsg_cs42xx8_codec_driver = {
	.driver = {
		.name = RPMSG_CODEC_DRV_NAME_CS42888,
		.pm = &rpmsg_cs42xx8_pm,
	},
	.probe = rpmsg_cs42xx8_codec_probe,
	.remove = rpmsg_cs42xx8_codec_remove,
};

module_platform_driver(rpmsg_cs42xx8_codec_driver);

MODULE_DESCRIPTION("Cirrus Logic CS42448/CS42888 ALSA SoC Codec Driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
