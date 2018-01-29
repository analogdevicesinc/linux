/*
 * ak4497.c  --  audio driver for AK4497
 *
 * Copyright (C) 2016 Asahi Kasei Microdevices Corporation
 * Copyright (C) 2017, NXP
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <linux/pm_runtime.h>

#include "ak4497.h"

//#define AK4497_DEBUG   //used at debug mode

/* AK4497 Codec Private Data */
struct ak4497_priv {
	struct i2c_client *i2c;
	struct regmap *regmap;
	int fs1;	/* Sampling Frequency */
	int nBickFreq;	/* 0: 48fs for 24bit,  1: 64fs or more for 32bit */
	int nDSDSel;
	int nTdmSds;
	int pdn_gpio;
	int mute_gpio;
	int fmt;
};

/* ak4497 register cache & default register settings */
static const struct reg_default ak4497_reg[] = {
	{ AK4497_00_CONTROL1, 0x0C},
	{ AK4497_01_CONTROL2, 0x22},
	{ AK4497_02_CONTROL3, 0x00},
	{ AK4497_03_LCHATT, 0xFF},
	{ AK4497_04_RCHATT, 0xFF},
	{ AK4497_05_CONTROL4, 0x00},
	{ AK4497_06_DSD1, 0x00},
	{ AK4497_07_CONTROL5, 0x00},
	{ AK4497_08_SOUNDCONTROL, 0x00},
	{ AK4497_09_DSD2, 0x00},
	{ AK4497_0A_CONTROL7, 0x04},
	{ AK4497_0B_CONTROL8, 0x00},
	{ AK4497_0C_RESERVED, 0x00},
	{ AK4497_0D_RESERVED, 0x00},
	{ AK4497_0E_RESERVED, 0x00},
	{ AK4497_0F_RESERVED, 0x00},
	{ AK4497_10_RESERVED, 0x00},
	{ AK4497_11_RESERVED, 0x00},
	{ AK4497_12_RESERVED, 0x00},
	{ AK4497_13_RESERVED, 0x00},
	{ AK4497_14_RESERVED, 0x00},
	{ AK4497_15_DFSREAD, 0x00},
};

/* Volume control:
 * from -127 to 0 dB in 0.5 dB steps (mute instead of -127.5 dB)
 */
static DECLARE_TLV_DB_SCALE(latt_tlv, -12750, 50, 0);
static DECLARE_TLV_DB_SCALE(ratt_tlv, -12750, 50, 0);

static const char * const ak4497_ecs_select_texts[] = {"768kHz", "384kHz"};

static const char * const ak4497_dem_select_texts[] = {
	"44.1kHz", "OFF", "48kHz", "32kHz"};
static const char * const ak4497_dzfm_select_texts[] = {
	"Separated", "ANDed"};

static const char * const ak4497_sellr_select_texts[] = {
	"Rch", "Lch"};
static const char * const ak4497_dckb_select_texts[] = {
	"Falling", "Rising"};
static const char * const ak4497_dcks_select_texts[] = {
	"512fs", "768fs"};

static const char * const ak4497_dsdd_select_texts[] = {
	"Normal", "Volume Bypass"};

static const char * const ak4497_sc_select_texts[] = {
	"Setting 1", "Setting 2", "Setting 3"};
static const char * const ak4497_dsdf_select_texts[] = {
	"50kHz", "150kHz"};
static const char * const ak4497_dsd_input_path_select[] = {
	"16_17_19pin", "3_4_5pin"};
static const char * const ak4497_ats_select_texts[] = {
	"4080/fs", "2040/fs", "510/fs", "255/fs"};

static const struct soc_enum ak4497_dac_enum[] = {
	SOC_ENUM_SINGLE(AK4497_00_CONTROL1, 5,
			ARRAY_SIZE(ak4497_ecs_select_texts),
			ak4497_ecs_select_texts),
	SOC_ENUM_SINGLE(AK4497_01_CONTROL2, 1,
			ARRAY_SIZE(ak4497_dem_select_texts),
			ak4497_dem_select_texts),
	SOC_ENUM_SINGLE(AK4497_01_CONTROL2, 6,
			ARRAY_SIZE(ak4497_dzfm_select_texts),
			ak4497_dzfm_select_texts),
	SOC_ENUM_SINGLE(AK4497_02_CONTROL3, 1,
			ARRAY_SIZE(ak4497_sellr_select_texts),
			ak4497_sellr_select_texts),
	SOC_ENUM_SINGLE(AK4497_02_CONTROL3, 4,
			ARRAY_SIZE(ak4497_dckb_select_texts),
			ak4497_dckb_select_texts),
	SOC_ENUM_SINGLE(AK4497_02_CONTROL3, 5,
			ARRAY_SIZE(ak4497_dcks_select_texts),
			ak4497_dcks_select_texts),
	SOC_ENUM_SINGLE(AK4497_06_DSD1, 1,
			ARRAY_SIZE(ak4497_dsdd_select_texts),
			ak4497_dsdd_select_texts),
	SOC_ENUM_SINGLE(AK4497_08_SOUNDCONTROL, 0,
			ARRAY_SIZE(ak4497_sc_select_texts),
			ak4497_sc_select_texts),
	SOC_ENUM_SINGLE(AK4497_09_DSD2, 1,
			ARRAY_SIZE(ak4497_dsdf_select_texts),
			ak4497_dsdf_select_texts),
	SOC_ENUM_SINGLE(AK4497_09_DSD2, 2,
			ARRAY_SIZE(ak4497_dsd_input_path_select),
			ak4497_dsd_input_path_select),
	SOC_ENUM_SINGLE(AK4497_0B_CONTROL8, 6,
			ARRAY_SIZE(ak4497_ats_select_texts),
			ak4497_ats_select_texts),
};

static const char * const ak4497_dsdsel_select_texts[] = {
	"64fs", "128fs", "256fs", "512fs"};
static const char * const ak4497_bickfreq_select[] = {"48fs", "64fs"};

static const char * const ak4497_tdm_sds_select[] = {
	"L1R1", "TDM128_L1R1", "TDM128_L2R2",
	"TDM256_L1R1", "TDM256_L2R2",  "TDM256_L3R3", "TDM256_L4R4",
	"TDM512_L1R1", "TDM512_L2R2",  "TDM512_L3R3", "TDM512_L4R4",
	"TDM512_L5R5", "TDM512_L6R6",  "TDM512_L7R7", "TDM512_L8R8",
};

static const char * const ak4497_adfs_select[] = {
	"Normal Speed Mode", "Double Speed Mode", "Quad Speed Mode",
	"Quad Speed Mode", "Oct Speed Mode", "Hex Speed Mode", "Oct Speed Mode",
	"Hex Speed Mode"
};

static const struct soc_enum ak4497_dac_enum2[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4497_dsdsel_select_texts),
			    ak4497_dsdsel_select_texts),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4497_bickfreq_select),
			    ak4497_bickfreq_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4497_tdm_sds_select),
			    ak4497_tdm_sds_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4497_adfs_select),
			    ak4497_adfs_select)
};

static int ak4497_get_dsdsel(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4497->nDSDSel;

	return 0;
}

static int ak4497_set_dsdsel(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);

	ak4497->nDSDSel = ucontrol->value.enumerated.item[0];

	if (ak4497->nDSDSel == 0) { /* 2.8224MHz */
		snd_soc_update_bits(codec, AK4497_06_DSD1, 0x01, 0x00);
		snd_soc_update_bits(codec, AK4497_09_DSD2, 0x01, 0x00);
	} else if (ak4497->nDSDSel == 1) { /* 5.6448MHz */
		snd_soc_update_bits(codec, AK4497_06_DSD1, 0x01, 0x01);
		snd_soc_update_bits(codec, AK4497_09_DSD2, 0x01, 0x00);
	} else { /* 11.2896MHz */
		snd_soc_update_bits(codec, AK4497_06_DSD1, 0x01, 0x00);
		snd_soc_update_bits(codec, AK4497_09_DSD2, 0x01, 0x01);
	}

	return 0;
}

static int ak4497_get_bickfs(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4497->nBickFreq;

	return 0;
}

static int ak4497_set_bickfs(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);

	ak4497->nBickFreq = ucontrol->value.enumerated.item[0];

	return 0;
}

static int ak4497_get_tdmsds(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4497->nTdmSds;

	return 0;
}

static int ak4497_set_tdmsds(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);
	int regA, regB;

	ak4497->nTdmSds = ucontrol->value.enumerated.item[0];

	if (ak4497->nTdmSds == 0)
		regB = 0; /* SDS0 bit = 0 */
	else
		regB = (1 & (ak4497->nTdmSds - 1)); /* SDS0 bit = 1 */

	switch (ak4497->nTdmSds) {
	case 0:
		regA = 0; /* Normal */
		break;
	case 1:
	case 2:
		regA = 4; /* TDM128 TDM1-0bits = 1 */
		break;
	case 3:
	case 4:
		regA = 8;  /* TDM128 TDM1-0bits = 2 */
		break;
	case 5:
	case 6:
		regA = 9;  /* TDM128 TDM1-0bits = 2 */
		break;
	case 7:
	case 8:
		regA = 0xC;  /* TDM128 TDM1-0bits = 3 */
		break;
	case 9:
	case 10:
		regA = 0xD;  /* TDM128 TDM1-0bits = 3 */
		break;
	case 11:
	case 12:
		regA = 0xE;  /* TDM128 TDM1-0bits = 3 */
		break;
	case 13:
	case 14:
		regA = 0xF;  /* TDM128 TDM1-0bits = 3 */
		break;
	default:
		regA = 0;
		regB = 0;
		break;
	}

	regA <<= 4;
	regB <<= 4;

	snd_soc_update_bits(codec, AK4497_0A_CONTROL7, 0xF0, regA);
	snd_soc_update_bits(codec, AK4497_0B_CONTROL8, 0x10, regB);

	return 0;
}

static int ak4497_get_adfs(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	int nADFSbit;

	nADFSbit = snd_soc_read(codec, AK4497_15_DFSREAD);
	nADFSbit &= 0x7;

	ucontrol->value.enumerated.item[0] = nADFSbit;

	return 0;
}

static int ak4497_set_adfs(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("AK4497 : ADFS is read only\n");

	return 0;
}

static const char * const gain_control_texts[] = {
	"2.8_2.8Vpp", "2.8_2.5Vpp", "2.5_2.5Vpp", "3.75_3.75Vpp", "3.75_2.5Vpp"
};

static const unsigned int gain_control_values[] = {
	0, 1, 2, 4, 5
};

static const struct soc_enum ak4497_gain_control_enum =
	SOC_VALUE_ENUM_SINGLE(AK4497_07_CONTROL5, 1, 7,
			      ARRAY_SIZE(gain_control_texts),
			      gain_control_texts,
			      gain_control_values);

#ifdef AK4497_DEBUG

static const char * const test_reg_select[] = {
	"read AK4497 Reg 00:0B",
	"read AK4497 Reg 15"
};

static const struct soc_enum ak4497_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo;

static int get_test_reg(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value  *ucontrol)
{
	/* Get the current output routing */
	ucontrol->value.enumerated.item[0] = nTestRegNo;

	return 0;
}

static int set_test_reg(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	u32 currMode = ucontrol->value.enumerated.item[0];
	int i, value;
	int regs, rege;

	nTestRegNo = currMode;

	if (nTestRegNo == 0) {
		regs = 0x00;
		rege = 0x0B;
	} else {
		regs = 0x15;
		rege = 0x15;
	}

	for (i = regs; i <= rege; i++) {
		value = snd_soc_read(codec, i);
		pr_debug("***AK4497 Addr,Reg=(%x, %x)\n", i, value);
	}

	return 0;
}
#endif

static const struct snd_kcontrol_new ak4497_snd_controls[] = {
	SOC_SINGLE_TLV("AK4497 Lch Digital Volume",
			AK4497_03_LCHATT, 0, 0xFF, 0, latt_tlv),
	SOC_SINGLE_TLV("AK4497 Rch Digital Volume",
			AK4497_04_RCHATT, 0, 0xFF, 0, ratt_tlv),

	SOC_ENUM("AK4497 EX DF I/F clock", ak4497_dac_enum[0]),
	SOC_ENUM("AK4497 De-emphasis Response", ak4497_dac_enum[1]),
	SOC_ENUM("AK4497 Data Zero Detect Mode", ak4497_dac_enum[2]),
	SOC_ENUM("AK4497 Data Selection at Mono Mode", ak4497_dac_enum[3]),

	SOC_ENUM("AK4497 Polarity of DCLK", ak4497_dac_enum[4]),
	SOC_ENUM("AK4497 DCKL Frequency", ak4497_dac_enum[5]),

	SOC_ENUM("AK4497 DDSD Play Back Path", ak4497_dac_enum[6]),
	SOC_ENUM("AK4497 Sound control", ak4497_dac_enum[7]),
	SOC_ENUM("AK4497 Cut Off of DSD Filter", ak4497_dac_enum[8]),

	SOC_ENUM_EXT("AK4497 DSD Data Stream", ak4497_dac_enum2[0],
		     ak4497_get_dsdsel, ak4497_set_dsdsel),
	SOC_ENUM_EXT("AK4497 BICK Frequency Select", ak4497_dac_enum2[1],
		     ak4497_get_bickfs, ak4497_set_bickfs),
	SOC_ENUM_EXT("AK4497 TDM Data Select", ak4497_dac_enum2[2],
		     ak4497_get_tdmsds, ak4497_set_tdmsds),

	SOC_SINGLE("AK4497 External Digital Filter", AK4497_00_CONTROL1,
		   6, 1, 0),
	SOC_SINGLE("AK4497 MCLK Frequency Auto Setting", AK4497_00_CONTROL1,
		   7, 1, 0),
	SOC_SINGLE("AK4497 MCLK FS Auto Detect", AK4497_00_CONTROL1, 4, 1, 0),

	SOC_SINGLE("AK4497 Soft Mute Control", AK4497_01_CONTROL2, 0, 1, 0),
	SOC_SINGLE("AK4497 Short delay filter", AK4497_01_CONTROL2, 5, 1, 0),
	SOC_SINGLE("AK4497 Data Zero Detect Enable", AK4497_01_CONTROL2,
		   7, 1, 0),
	SOC_SINGLE("AK4497 Slow Roll-off Filter", AK4497_02_CONTROL3, 0, 1, 0),
	SOC_SINGLE("AK4497 Invering Enable of DZF", AK4497_02_CONTROL3,
		   4, 1, 0),
	SOC_SINGLE("AK4497 Mono Mode", AK4497_02_CONTROL3, 3, 1, 0),
	SOC_SINGLE("AK4497 Super Slow Roll-off Filter", AK4497_05_CONTROL4,
		   0, 1, 0),
	SOC_SINGLE("AK4497 AOUTR Phase Inverting", AK4497_05_CONTROL4,
		   6, 1, 0),
	SOC_SINGLE("AK4497 AOUTL Phase Inverting", AK4497_05_CONTROL4,
		   7, 1, 0),
	SOC_SINGLE("AK4497 DSD Mute Release", AK4497_06_DSD1, 3, 1, 0),
	SOC_SINGLE("AK4497 DSD Mute Control Hold", AK4497_06_DSD1, 4, 1, 0),
	SOC_SINGLE("AK4497 DSDR is detected", AK4497_06_DSD1, 5, 1, 0),
	SOC_SINGLE("AK4497 DSDL is detected", AK4497_06_DSD1, 6, 1, 0),
	SOC_SINGLE("AK4497 DSD Data Mute", AK4497_06_DSD1, 7, 1, 0),
	SOC_SINGLE("AK4497 Synchronization Control", AK4497_07_CONTROL5,
		   0, 1, 0),

	SOC_ENUM("AK4497 Output Level", ak4497_gain_control_enum),
	SOC_SINGLE("AK4497 High Sonud Quality Mode", AK4497_08_SOUNDCONTROL,
		   2, 1, 0),
	SOC_SINGLE("AK4497 Heavy Load Mode", AK4497_08_SOUNDCONTROL, 3, 1, 0),
	SOC_ENUM("AK4497 DSD Data Input Pin", ak4497_dac_enum[9]),
	SOC_SINGLE("AK4497 Daisy Chain", AK4497_0B_CONTROL8, 1, 1, 0),
	SOC_ENUM("AK4497 ATT Transit Time", ak4497_dac_enum[10]),

	SOC_ENUM_EXT("AK4497 Read FS Auto Detect Mode", ak4497_dac_enum2[3],
		     ak4497_get_adfs, ak4497_set_adfs),

#ifdef AK4497_DEBUG
	SOC_ENUM_EXT("Reg Read", ak4497_enum[0], get_test_reg, set_test_reg),
#endif

};

static const char * const ak4497_dac_enable_texts[] = {"Off", "On"};

static SOC_ENUM_SINGLE_VIRT_DECL(ak4497_dac_enable_enum,
				 ak4497_dac_enable_texts);

static const struct snd_kcontrol_new ak4497_dac_enable_control =
	SOC_DAPM_ENUM("DAC Switch", ak4497_dac_enable_enum);

/* ak4497 dapm widgets */
static const struct snd_soc_dapm_widget ak4497_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AK4497 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DAC("AK4497 DAC", NULL, AK4497_0A_CONTROL7, 2, 0),

	SND_SOC_DAPM_MUX("AK4497 DAC Enable", SND_SOC_NOPM,
			 0, 0, &ak4497_dac_enable_control),

	SND_SOC_DAPM_OUTPUT("AK4497 AOUT"),

};

static const struct snd_soc_dapm_route ak4497_intercon[] = {
	{"AK4497 DAC", NULL, "AK4497 SDTI"},
	{"AK4497 DAC Enable", "On", "AK4497 DAC"},
	{"AK4497 AOUT", NULL, "AK4497 DAC Enable"},
};

static int ak4497_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);
	snd_pcm_format_t pcm_format = params_format(params);

	u8 format;
	u8 dfs;
	u8 dfs2;
	int nfs1;
	bool is_dsd = false;

	if (pcm_format == SNDRV_PCM_FORMAT_DSD_U8 ||
		pcm_format == SNDRV_PCM_FORMAT_DSD_U16_LE ||
		pcm_format == SNDRV_PCM_FORMAT_DSD_U16_BE ||
		pcm_format == SNDRV_PCM_FORMAT_DSD_U32_LE ||
		pcm_format == SNDRV_PCM_FORMAT_DSD_U32_BE)
		is_dsd = true;

	nfs1 = params_rate(params);
	ak4497->fs1 = nfs1;

	dfs = snd_soc_read(codec, AK4497_01_CONTROL2);
	dfs &= ~AK4497_DFS;

	dfs2 = snd_soc_read(codec, AK4497_05_CONTROL4);
	dfs2 &= ~AK4497_DFS2;

	if (!is_dsd) {
		switch (nfs1) {
		case 8000:
		case 11025:
		case 16000:
		case 22050:
		case 32000:
		case 44100:
		case 48000:
			dfs |= AK4497_DFS_48KHZ;
			dfs2 |= AK4497_DFS2_48KHZ;
			break;
		case 88200:
		case 96000:
			dfs |= AK4497_DFS_96KHZ;
			dfs2 |= AK4497_DFS2_48KHZ;
			break;

		case 176400:
		case 192000:
			dfs |= AK4497_DFS_192KHZ;
			dfs2 |= AK4497_DFS2_48KHZ;
			break;
		case 384000:
			dfs |= AK4497_DFS_384KHZ;
			dfs2 |= AK4497_DFS2_384KHZ;
			break;
		case 768000:
			dfs |= AK4497_DFS_768KHZ;
			dfs2 |= AK4497_DFS2_384KHZ;
			break;
		default:
			return -EINVAL;
		}
	}

	snd_soc_write(codec, AK4497_01_CONTROL2, dfs);
	snd_soc_write(codec, AK4497_05_CONTROL4, dfs2);

	format = snd_soc_read(codec, AK4497_00_CONTROL1);
	format &= ~AK4497_DIF;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (ak4497->fmt == SND_SOC_DAIFMT_I2S)
			format |= AK4497_DIF_24BIT_I2S;
		else
			format |= AK4497_DIF_16BIT_LSB;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		if (ak4497->fmt == SND_SOC_DAIFMT_I2S)
			format |= AK4497_DIF_32BIT_I2S;
		else if (ak4497->fmt == SND_SOC_DAIFMT_LEFT_J)
			format |= AK4497_DIF_32BIT_MSB;
		else if (ak4497->fmt == SND_SOC_DAIFMT_RIGHT_J)
			format |= AK4497_DIF_32BIT_LSB;
		else
			return -EINVAL;
		break;
	case SNDRV_PCM_FORMAT_DSD_U8:
	case SNDRV_PCM_FORMAT_DSD_U16_LE:
	case SNDRV_PCM_FORMAT_DSD_U32_LE:
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, AK4497_00_CONTROL1, format);

	return 0;
}

static int ak4497_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				 unsigned int freq, int dir)
{
//	struct snd_soc_codec *codec = dai->codec;

	return 0;
}

static int ak4497_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);
	u8 format;
	u8 format2;

	/* set master/slave audio interface */
	format = snd_soc_read(codec, AK4497_00_CONTROL1);
	format &= ~AK4497_DIF;

	format2 = snd_soc_read(codec, AK4497_02_CONTROL3);
	format2 &= ~AK4497_DIF_DSD;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
	case SND_SOC_DAIFMT_CBS_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
	default:
		dev_err(codec->dev, "Clock mode unsupported");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ak4497->fmt = SND_SOC_DAIFMT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ak4497->fmt = SND_SOC_DAIFMT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ak4497->fmt = SND_SOC_DAIFMT_RIGHT_J;
		break;
	case SND_SOC_DAIFMT_PDM:
		format2 |= AK4497_DIF_DSD_MODE;
		break;
	default:
		return -EINVAL;
	}

	/* set format */
	snd_soc_write(codec, AK4497_00_CONTROL1, format);
	snd_soc_write(codec, AK4497_02_CONTROL3, format2);

	return 0;
}

static bool ak4497_volatile(struct device *dev, unsigned int reg)
{
	int ret;

#ifdef AK4497_DEBUG
	ret = 1;
#else
	switch (reg) {
	case AK4497_15_DFSREAD:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
#endif
	return ret;
}

static int ak4497_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		/* RSTN bit = 1 */
		snd_soc_update_bits(codec, AK4497_00_CONTROL1, 0x01, 0x01);
		break;
	case SND_SOC_BIAS_OFF:
		/* RSTN bit = 0 */
		snd_soc_update_bits(codec, AK4497_00_CONTROL1, 0x01, 0x00);
		break;
	}

	return 0;
}

static int ak4497_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);
	int nfs, ndt;

	nfs = ak4497->fs1;

	if (mute) { /* SMUTE: 1 , MUTE */
		snd_soc_update_bits(codec, AK4497_01_CONTROL2, 0x01, 0x01);
		ndt = 7424000 / nfs;
		mdelay(ndt);

		/* External Mute ON */
		if (ak4497->mute_gpio != -1)
			gpio_set_value(ak4497->mute_gpio, 1);
	} else { /* SMUTE: 0, NORMAL operation */

		/* External Mute OFF */
		if (ak4497->mute_gpio != -1)
			gpio_set_value(ak4497->mute_gpio, 0);
		snd_soc_update_bits(codec, AK4497_01_CONTROL2, 0x01, 0x00);
	}

	return 0;
}

#define AK4497_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		      SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		      SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
		      SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
		      SNDRV_PCM_RATE_192000)

#define AK4497_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE  |\
			 SNDRV_PCM_FMTBIT_DSD_U8 |\
			 SNDRV_PCM_FMTBIT_DSD_U16_LE |\
			 SNDRV_PCM_FMTBIT_DSD_U32_LE)

static const unsigned int ak4497_rates[] = {
	8000, 11025,  16000, 22050,
	32000, 44100, 48000, 88200,
	96000, 176400, 192000, 352800,
	384000, 705600, 768000, 1411200,
	2822400,
};

static const struct snd_pcm_hw_constraint_list ak4497_rate_constraints = {
	.count = ARRAY_SIZE(ak4497_rates),
	.list = ak4497_rates,
};

static int ak4497_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai) {
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &ak4497_rate_constraints);

	return ret;
}

static struct snd_soc_dai_ops ak4497_dai_ops = {
	.startup        = ak4497_startup,
	.hw_params	= ak4497_hw_params,
	.set_sysclk	= ak4497_set_dai_sysclk,
	.set_fmt	= ak4497_set_dai_fmt,
	.digital_mute   = ak4497_set_dai_mute,
};

struct snd_soc_dai_driver ak4497_dai[] = {
	{
		.name = "ak4497-aif",
		.playback = {
		       .stream_name = "Playback",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = SNDRV_PCM_RATE_KNOT,
		       .formats = AK4497_FORMATS,
		},
		.ops = &ak4497_dai_ops,
	},
};

static void ak4497_init_reg(struct snd_soc_codec *codec)
{
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);

	if (ak4497->mute_gpio > 0)
		gpio_set_value(ak4497->mute_gpio, 1); /* External Mute ON */

	if (ak4497->pdn_gpio > 0) {
		gpio_set_value(ak4497->pdn_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value(ak4497->pdn_gpio, 1);
		usleep_range(1000, 2000);
	}

	/* ak4497_set_bias_level(codec, SND_SOC_BIAS_STANDBY); */

	/* SYNCE bit = 1 */
	snd_soc_update_bits(codec, AK4497_07_CONTROL5, 0x01, 0x01);

	/*  HLOAD bit = 1, SC2 bit = 1 */
	snd_soc_update_bits(codec, AK4497_08_SOUNDCONTROL, 0x0F, 0x0C);
}

static int ak4497_parse_dt(struct ak4497_priv *ak4497)
{
	struct device *dev;
	struct device_node *np;

	dev = &(ak4497->i2c->dev);
	np = dev->of_node;

	ak4497->pdn_gpio = -1;
	ak4497->mute_gpio = -1;

	if (!np)
		return -1;

	ak4497->pdn_gpio = of_get_named_gpio(np, "ak4497,pdn-gpio", 0);
	if (ak4497->pdn_gpio < 0)
		ak4497->pdn_gpio = -1;

	if (!gpio_is_valid(ak4497->pdn_gpio)) {
		dev_err(dev, "ak4497 pdn pin(%u) is invalid\n",
		       ak4497->pdn_gpio);
		ak4497->pdn_gpio = -1;
	}

	ak4497->mute_gpio = of_get_named_gpio(np, "ak4497,mute-gpio", 0);
	if (ak4497->mute_gpio < 0)
		ak4497->mute_gpio = -1;

	if (!gpio_is_valid(ak4497->mute_gpio)) {
		dev_err(dev, "ak4497 mute_gpio(%u) is invalid\n",
		       ak4497->mute_gpio);
		ak4497->mute_gpio = -1;
	}

	return 0;
}

static int ak4497_probe(struct snd_soc_codec *codec)
{
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	ret = ak4497_parse_dt(ak4497);
	if (ak4497->pdn_gpio > 0) {
		ret = gpio_request(ak4497->pdn_gpio, "ak4497 pdn");
		gpio_direction_output(ak4497->pdn_gpio, 0);
	}
	if (ak4497->mute_gpio > 0) {
		ret = gpio_request(ak4497->mute_gpio, "ak4497 mute");
		gpio_direction_output(ak4497->mute_gpio, 0);
	}

	ak4497_init_reg(codec);

	ak4497->fs1 = 48000;
	ak4497->nBickFreq = 1;
	ak4497->nDSDSel = 0;
	ak4497->nTdmSds = 0;

	return ret;
}

static int ak4497_remove(struct snd_soc_codec *codec)
{
	struct ak4497_priv *ak4497 = snd_soc_codec_get_drvdata(codec);

	ak4497_set_bias_level(codec, SND_SOC_BIAS_OFF);
	if (ak4497->pdn_gpio > 0) {
		gpio_set_value(ak4497->pdn_gpio, 0);
		gpio_free(ak4497->pdn_gpio);
	}
	if (ak4497->mute_gpio > 0)
		gpio_free(ak4497->mute_gpio);

	return 0;
}

#ifdef CONFIG_PM
static int ak4497_runtime_suspend(struct device *dev)
{
	struct ak4497_priv *ak4497 = dev_get_drvdata(dev);

	regcache_cache_only(ak4497->regmap, true);

	if (ak4497->pdn_gpio > 0) {
		gpio_set_value(ak4497->pdn_gpio, 0);
		usleep_range(1000, 2000);
	}

	if (ak4497->mute_gpio > 0)
		gpio_free(ak4497->mute_gpio);

	return 0;
}

static int ak4497_runtime_resume(struct device *dev)
{
	struct ak4497_priv *ak4497 = dev_get_drvdata(dev);

	if (ak4497->mute_gpio > 0)
		gpio_set_value(ak4497->mute_gpio, 1); /* External Mute ON */

	if (ak4497->pdn_gpio > 0) {
		gpio_set_value(ak4497->pdn_gpio, 1);
		usleep_range(1000, 2000);
	}

	regcache_cache_only(ak4497->regmap, false);
	regcache_mark_dirty(ak4497->regmap);

	return regcache_sync(ak4497->regmap);
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops ak4497_pm = {
	SET_RUNTIME_PM_OPS(ak4497_runtime_suspend, ak4497_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

struct snd_soc_codec_driver soc_codec_dev_ak4497 = {
	.probe = ak4497_probe,
	.remove = ak4497_remove,

	.idle_bias_off = true,
	.set_bias_level = ak4497_set_bias_level,

	.component_driver = {
		.controls = ak4497_snd_controls,
		.num_controls = ARRAY_SIZE(ak4497_snd_controls),
		.dapm_widgets = ak4497_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(ak4497_dapm_widgets),
		.dapm_routes = ak4497_intercon,
		.num_dapm_routes = ARRAY_SIZE(ak4497_intercon),
	},
};

static const struct regmap_config ak4497_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK4497_MAX_REGISTERS,
	.volatile_reg = ak4497_volatile,

	.reg_defaults = ak4497_reg,
	.num_reg_defaults = ARRAY_SIZE(ak4497_reg),
	.cache_type = REGCACHE_RBTREE,
};

static int ak4497_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct ak4497_priv *ak4497;
	int ret = 0;

	ak4497 = devm_kzalloc(&i2c->dev,
			      sizeof(struct ak4497_priv), GFP_KERNEL);
	if (ak4497 == NULL)
		return -ENOMEM;

	ak4497->regmap = devm_regmap_init_i2c(i2c, &ak4497_regmap);
	if (IS_ERR(ak4497->regmap))
		return PTR_ERR(ak4497->regmap);

	i2c_set_clientdata(i2c, ak4497);
	ak4497->i2c = i2c;

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_ak4497,
				     &ak4497_dai[0], ARRAY_SIZE(ak4497_dai));
	if (ret < 0) {
		kfree(ak4497);
		return ret;
	}

	pm_runtime_enable(&i2c->dev);

	return 0;
}

static int ak4497_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	pm_runtime_disable(&client->dev);

	return 0;
}

static const struct of_device_id ak4497_i2c_dt_ids[] = {
	{ .compatible = "asahi-kasei,ak4497"},
	{ }
};

static const struct i2c_device_id ak4497_i2c_id[] = {
	{ "ak4497", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4497_i2c_id);

static struct i2c_driver ak4497_i2c_driver = {
	.driver = {
		.name = "ak4497",
		.of_match_table = of_match_ptr(ak4497_i2c_dt_ids),
		.pm = &ak4497_pm,
	},
	.probe = ak4497_i2c_probe,
	.remove = ak4497_i2c_remove,
	.id_table = ak4497_i2c_id,
};

module_i2c_driver(ak4497_i2c_driver);

MODULE_AUTHOR("Junichi Wakasugi <wakasugi.jb@om.asahi-kasei.co.jp>");
MODULE_AUTHOR("Daniel Baluta <daniel.baluta@nxp.com>");
MODULE_DESCRIPTION("ASoC ak4497 codec driver");
MODULE_LICENSE("GPL");
