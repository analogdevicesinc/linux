/*
 * ak4458.c  --  audio driver for AK4458 DAC
 *
 * Copyright (C) 2016 Asahi Kasei Microdevices Corporation
 * Copyright 2017 NXP
 *
 * Authors:
 * Tsuyoshi Mutsuro
 * Junichi Wakasugi <wakasugi.jb@om.asahi-kasei.co.jp>
 * Mihai Serban <mihai.serban@nxp.com>
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
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "ak4458.h"

/* AK4458 Codec Private Data */
struct ak4458_priv {
	struct device *dev;
	struct regmap *regmap;
	int pdn_gpio;
	int mute_gpio;
	int sds;	/* SDS2-0 bits */
	int digfil;	/* SSLOW, SD, SLOW bits */
	int fs;		/* sampling rate */
	int lr[4];	/* (MONO, INVL, INVR, SELLR) x4ch */
};

static const struct reg_default ak4458_reg_defaults[] = {
	{ 0x00, 0x0C },	/*	0x00	AK4458_00_CONTROL1	*/
	{ 0x01, 0x22 },	/*	0x01	AK4458_01_CONTROL2	*/
	{ 0x02, 0x00 },	/*	0x02	AK4458_02_CONTROL3	*/
	{ 0x03, 0xFF },	/*	0x03	AK4458_03_LCHATT	*/
	{ 0x04, 0xFF },	/*	0x04	AK4458_04_RCHATT	*/
	{ 0x05, 0x00 },	/*	0x05	AK4458_05_CONTROL4	*/
	{ 0x06, 0x00 },	/*	0x06	AK4458_06_DSD1		*/
	{ 0x07, 0x03 },	/*	0x07	AK4458_07_CONTROL5	*/
	{ 0x08, 0x00 },	/*	0x08	AK4458_08_SOUND_CONTROL	*/
	{ 0x09, 0x00 },	/*	0x09	AK4458_09_DSD2		*/
	{ 0x0A, 0x0D },	/*	0x0A	AK4458_0A_CONTROL6	*/
	{ 0x0B, 0x0C },	/*	0x0B	AK4458_0B_CONTROL7	*/
	{ 0x0C, 0x00 },	/*	0x0C	AK4458_0C_CONTROL8	*/
	{ 0x0D, 0x00 },	/*	0x0D	AK4458_0D_CONTROL9	*/
	{ 0x0E, 0x50 },	/*	0x0E	AK4458_0E_CONTROL10	*/
	{ 0x0F, 0xFF },	/*	0x0F	AK4458_0F_L2CHATT	*/
	{ 0x10, 0xFF },	/*	0x10	AK4458_10_R2CHATT	*/
	{ 0x11, 0xFF },	/*	0x11	AK4458_11_L3CHATT	*/
	{ 0x12, 0xFF },	/*	0x12	AK4458_12_R3CHATT	*/
	{ 0x13, 0xFF },	/*	0x13	AK4458_13_L4CHATT	*/
	{ 0x14, 0xFF },	/*	0x14	AK4458_14_R4CHATT	*/
};

static const struct regmap_range ak4458_spi_non_readable_reg_ranges[] = {
	regmap_reg_range(AK4458_00_CONTROL1, AK4458_14_R4CHATT),
};

static const struct regmap_access_table ak4458_spi_readable_regs = {
	.no_ranges = ak4458_spi_non_readable_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(ak4458_spi_non_readable_reg_ranges),
};

/*
 * Volume control:
 * from -127 to 0 dB in 0.5 dB steps (mute instead of -127.5 dB)
 */
static DECLARE_TLV_DB_SCALE(latt_tlv, -12750, 50, 1);
static DECLARE_TLV_DB_SCALE(ratt_tlv, -12750, 50, 1);

/*
 * DEM1 bit DEM0 bit Mode
 * 0 0 44.1kHz
 * 0 1 OFF (default)
 * 1 0 48kHz
 * 1 1 32kHz
 */
static const char * const ak4458_dem_select_texts[] = {
	"44.1kHz", "OFF", "48kHz", "32kHz"
};

/*
 * SSLOW, SD, SLOW bits Digital Filter Setting
 * 0, 0, 0 : Sharp Roll-Off Filter
 * 0, 0, 1 : Slow Roll-Off Filter
 * 0, 1, 0 : Short delay Sharp Roll-Off Filter
 * 0, 1, 1 : Short delay Slow Roll-Off Filter
 * 1, *, * : Super Slow Roll-Off Filter
 */
static const char * const ak4458_digfil_select_texts[] = {
	"Sharp Roll-Off Filter",
	"Slow Roll-Off Filter",
	"Short delay Sharp Roll-Off Filter",
	"Short delay Slow Roll-Off Filter",
	"Super Slow Roll-Off Filter"
};

/*
 * DZFB: Inverting Enable of DZF
 * 0: DZF goes H at Zero Detection
 * 1: DZF goes L at Zero Detection
 */
static const char * const ak4458_dzfb_select_texts[] = {"H", "L"};

/*
 * SC1-0 bits: Sound Mode Setting
 * 0 0 : Sound Mode 0
 * 0 1 : Sound Mode 1
 * 1 0 : Sound Mode 2
 * 1 1 : Reserved
 */
static const char * const ak4458_sc_select_texts[] = {
	"Sound Mode 0", "Sound Mode 1", "Sound Mode 2"
};

/*
 * SDS2-0 bits: Output Data Select
 * Refer to Data Sheet
 */
static const char * const ak4458_sds_select_texts[] = {
	"Setting 0", "Setting 1", "Setting 2", "Setting 3",
	"Setting 4", "Setting 5", "Setting 6", "Setting 7",
};

/*
 * TDM1-0 bits: TDM Mode Setting
 * 0 0 : Normal Mode
 * 0 1 : TDM128 Mode
 * 1 0 : TDM256 Mode
 * 1 1 : TDM512 Mode
 */
static const char * const ak4458_tdm_select_texts[] = {
	"Normal Mode", "TDM128 Mode", "TDM256 Mode", "TDM512 Mode"
};

/* FIR2-0 bits: FIR Filter Mode Setting */
static const char * const ak4458_fir_select_texts[] = {
	"Mode 0", "Mode 1", "Mode 2", "Mode 3",
	"Mode 4", "Mode 5", "Mode 6", "Mode 7",
};

/* Mono and SELLR bit Setting (1~4) */
static const char * const ak4458_dac_LR_select_texts[] = {
	"Lch In, Rch In",
	"Lch In, Rch In Invert",
	"Lch In Invert, Rch In",
	"Lch In Invert, Rch In Invert",
	"Rch In, Lch In",
	"Rch In, Lch In Invert",
	"Rch In Invert, Lch In",
	"Rch In Invert, Lch In Invert",
	"Lch In, Lch In",
	"Lch In, Lch In Invert",
	"Lch In Invert, Lch In",
	"Lch In Invert, Lch In Invert",
	"Rch In, Rch In",
	"Rch In, Rch In Invert",
	"Rch In Invert, Rch In",
	"Rch In Invert, Rch In Invert",
};

/* ATS1-0 bits Attenuation Speed */
static const char * const ak4458_ats_select_texts[] = {
	"4080/fs", "2040/fs", "510/fs", "255/fs",
};

/* DIF2 bit Audio Interface Format Setting(BICK fs) */
static const char * const ak4458_dif_select_texts[] = {"32fs,48fs", "64fs",};

static int get_DAC1_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4458->lr[0];

	return 0;
};

static int get_DAC2_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4458->lr[1];

	return 0;
};

static int get_DAC3_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4458->lr[2];

	return 0;
};

static int get_DAC4_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4458->lr[3];

	return 0;
};

static int get_digfil(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4458->digfil;

	return 0;
};

static int get_sds(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4458->sds;

	return 0;
};

static int set_digfil(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	int reg_01, reg_02, reg_05, num;

	num = ucontrol->value.enumerated.item[0];
	if (num > 4)
		return -EINVAL;

	ak4458->digfil = num;

	/* write SD bit */
	reg_01 = snd_soc_read(codec, AK4458_01_CONTROL2);
	reg_01 &= ~AK4458_SD_MASK;

	reg_01 |= ((ak4458->digfil & 0x02) << 4);
	snd_soc_write(codec, AK4458_01_CONTROL2, reg_01);

	/* write SLOW bit */
	reg_02 = snd_soc_read(codec, AK4458_02_CONTROL3);
	reg_02 &= ~AK4458_SLOW_MASK;

	reg_02 |= (ak4458->digfil & 0x01);
	snd_soc_write(codec, AK4458_02_CONTROL3, reg_02);

	/* write SSLOW bit */
	reg_05 = snd_soc_read(codec, AK4458_05_CONTROL4);
	reg_05 &= ~AK4458_SSLOW_MASK;

	reg_05 |= ((ak4458->digfil & 0x04) >> 2);
	snd_soc_write(codec, AK4458_05_CONTROL4, reg_05);

	return 0;
};

static int set_sds(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	int reg_0b, reg_0a;

	if (ucontrol->value.enumerated.item[0] > 7)
		return -EINVAL;

	ak4458->sds = ucontrol->value.enumerated.item[0];

	/* write SDS0 bit */
	reg_0b = snd_soc_read(codec, AK4458_0B_CONTROL7);
	reg_0b &= ~AK4458_SDS0__MASK;

	reg_0b |= ((ak4458->sds & 0x01) << 4);
	snd_soc_write(codec, AK4458_0B_CONTROL7, reg_0b);

	/* write SDS1,2 bits */
	reg_0a = snd_soc_read(codec, AK4458_0A_CONTROL6);
	reg_0a &= ~AK4458_SDS12_MASK;

	reg_0a |= ((ak4458->sds & 0x02) << 4);
	reg_0a |= ((ak4458->sds & 0x04) << 2);
	snd_soc_write(codec, AK4458_0A_CONTROL6, reg_0a);

	return 0;

};

static int set_DAC1_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	int reg_02, reg_05;

	if (ucontrol->value.enumerated.item[0] > 15)
		return -EINVAL;

	ak4458->lr[0] = ucontrol->value.enumerated.item[0];

	/* write MONO1 and SELLR1 bits */
	reg_02 = snd_soc_read(codec, AK4458_02_CONTROL3);
	reg_02 &= ~AK4458_DAC1_LR_MASK;


	reg_02 |= (ak4458->lr[0] & 0x08) << 0;
	reg_02 |= (ak4458->lr[0] & 0x04) >> 1;
	snd_soc_write(codec, AK4458_02_CONTROL3, reg_02);

	/* write INVL1 and INVR1 bits */
	reg_05 = snd_soc_read(codec, AK4458_05_CONTROL4);
	reg_05 &= ~AK4458_DAC1_INV_MASK;

	reg_05 |= (ak4458->lr[0] & 0x02) << 6;
	reg_05 |= (ak4458->lr[0] & 0x01) << 6;
	snd_soc_write(codec, AK4458_05_CONTROL4, reg_05);

	return 0;

};

static int set_DAC2_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	int reg_0D, reg_05;

	if (ucontrol->value.enumerated.item[0] > 15)
		return -EINVAL;


	ak4458->lr[1] = ucontrol->value.enumerated.item[0];

	/* write MONO2 bit */
	reg_0D = snd_soc_read(codec, AK4458_0D_CONTROL9);
	reg_0D &= ~AK4458_DAC2_MASK1;

	reg_0D |= (ak4458->lr[1] & 0x08) << 2;
	snd_soc_write(codec, AK4458_0D_CONTROL9, reg_0D);

	/* write SELLR2 and INVL1 and INVR1 bits */
	reg_05 = snd_soc_read(codec, AK4458_05_CONTROL4);
	reg_05 &= ~AK4458_DAC2_MASK2;

	reg_05 |= (ak4458->lr[1] & 0x04) << 1;
	reg_05 |= (ak4458->lr[1] & 0x02) << 4;
	reg_05 |= (ak4458->lr[1] & 0x01) << 4;
	snd_soc_write(codec, AK4458_05_CONTROL4, reg_05);

	return 0;

};

static int set_DAC3_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	int reg_0C, reg_0D;

	if (ucontrol->value.enumerated.item[0] > 15)
		return -EINVAL;


	ak4458->lr[2] = ucontrol->value.enumerated.item[0];

	/* write MONO3 and SELLR3 bits */
	reg_0D = snd_soc_read(codec, AK4458_0D_CONTROL9);
	reg_0D &= ~AK4458_DAC3_LR_MASK;


	reg_0D |= (ak4458->lr[2] & 0x08) << 3;
	reg_0D |= (ak4458->lr[2] & 0x04) << 0;
	snd_soc_write(codec, AK4458_0D_CONTROL9, reg_0D);

	/* write INVL3 and INVR3 bits */
	reg_0C = snd_soc_read(codec, AK4458_0C_CONTROL8);
	reg_0C &= ~AK4458_DAC3_INV_MASK;

	reg_0C |= (ak4458->lr[2] & 0x02) << 3;
	reg_0C |= (ak4458->lr[2] & 0x01) << 5;
	snd_soc_write(codec, AK4458_0C_CONTROL8, reg_0C);

	return 0;

};

static int set_DAC4_LR(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	int reg_0C, reg_0D;

	if (ucontrol->value.enumerated.item[0] > 15)
		return -EINVAL;


	ak4458->lr[3] = ucontrol->value.enumerated.item[0];

	/* write MONO4 and SELLR4 bits */
	reg_0D = snd_soc_read(codec, AK4458_0D_CONTROL9);
	reg_0D &= ~AK4458_DAC4_LR_MASK;


	reg_0D |= (ak4458->lr[3] & 0x08) << 4;
	reg_0D |= (ak4458->lr[3] & 0x04) << 1;
	snd_soc_write(codec, AK4458_0D_CONTROL9, reg_0D);

	/* write INVL4 and INVR4 bits */
	reg_0C = snd_soc_read(codec, AK4458_0C_CONTROL8);
	reg_0C &= ~AK4458_DAC4_INV_MASK;

	reg_0C |= (ak4458->lr[3] & 0x02) << 5;
	reg_0C |= (ak4458->lr[3] & 0x01) << 7;
	snd_soc_write(codec, AK4458_0C_CONTROL8, reg_0C);

	return 0;

};

static const struct soc_enum ak4458_dac_enum[] = {
/*0*/	SOC_ENUM_SINGLE(AK4458_01_CONTROL2, 1,
			ARRAY_SIZE(ak4458_dem_select_texts),
			ak4458_dem_select_texts),
/*1*/	SOC_ENUM_SINGLE(AK4458_0A_CONTROL6, 0,
			ARRAY_SIZE(ak4458_dem_select_texts),
			ak4458_dem_select_texts),
/*2*/	SOC_ENUM_SINGLE(AK4458_0E_CONTROL10, 4,
			ARRAY_SIZE(ak4458_dem_select_texts),
			ak4458_dem_select_texts),
/*3*/	SOC_ENUM_SINGLE(AK4458_0E_CONTROL10, 6,
			ARRAY_SIZE(ak4458_dem_select_texts),
			ak4458_dem_select_texts),
/*4*/	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4458_digfil_select_texts),
			    ak4458_digfil_select_texts),
/*5*/	SOC_ENUM_SINGLE(AK4458_02_CONTROL3, 2,
			ARRAY_SIZE(ak4458_dzfb_select_texts),
			ak4458_dzfb_select_texts),
/*6*/	SOC_ENUM_SINGLE(AK4458_08_SOUND_CONTROL, 0,
			ARRAY_SIZE(ak4458_sc_select_texts),
			ak4458_sc_select_texts),
/*7*/	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4458_sds_select_texts),
			    ak4458_sds_select_texts),
/*8*/	SOC_ENUM_SINGLE(AK4458_0C_CONTROL8, 0,
			ARRAY_SIZE(ak4458_fir_select_texts),
			ak4458_fir_select_texts),
/*9*/	SOC_ENUM_SINGLE(AK4458_0A_CONTROL6, 6,
			ARRAY_SIZE(ak4458_tdm_select_texts),
			ak4458_tdm_select_texts),
/*10*/  SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4458_dac_LR_select_texts),
			    ak4458_dac_LR_select_texts),
/*11*/	SOC_ENUM_SINGLE(AK4458_0B_CONTROL7, 6,
			ARRAY_SIZE(ak4458_ats_select_texts),
			ak4458_ats_select_texts),
/*12*/	SOC_ENUM_SINGLE(AK4458_00_CONTROL1, 3,
			ARRAY_SIZE(ak4458_dif_select_texts),
			ak4458_dif_select_texts),
};

static const struct snd_kcontrol_new ak4458_snd_controls[] = {
	SOC_SINGLE_TLV("AK4458 L1ch Digital Volume",
		       AK4458_03_LCHATT, 0/*shift*/, 0xFF/*max value*/,
		       0/*invert*/, latt_tlv),
	SOC_SINGLE_TLV("AK4458 R1ch Digital Volume",
		       AK4458_04_RCHATT, 0, 0xFF, 0, ratt_tlv),
	SOC_SINGLE_TLV("AK4458 L2ch Digital Volume",
		       AK4458_0F_L2CHATT, 0/*shift*/, 0xFF/*max value*/,
		       0/*invert*/, latt_tlv),
	SOC_SINGLE_TLV("AK4458 R2ch Digital Volume",
		       AK4458_10_R2CHATT, 0, 0xFF, 0, ratt_tlv),
	SOC_SINGLE_TLV("AK4458 L3ch Digital Volume",
		       AK4458_11_L3CHATT, 0/*shift*/, 0xFF/*max value*/,
		       0/*invert*/, latt_tlv),
	SOC_SINGLE_TLV("AK4458 R3ch Digital Volume",
		       AK4458_12_R3CHATT, 0, 0xFF, 0, ratt_tlv),
	SOC_SINGLE_TLV("AK4458 L4ch Digital Volume",
		       AK4458_13_L4CHATT, 0/*shift*/, 0xFF/*max value*/,
		       0/*invert*/, latt_tlv),
	SOC_SINGLE_TLV("AK4458 R4ch Digital Volume",
		       AK4458_14_R4CHATT, 0, 0xFF, 0, ratt_tlv),

	SOC_ENUM("AK4458 De-emphasis Response DAC1", ak4458_dac_enum[0]),
	SOC_ENUM("AK4458 De-emphasis Response DAC2", ak4458_dac_enum[1]),
	SOC_ENUM("AK4458 De-emphasis Response DAC3", ak4458_dac_enum[2]),
	SOC_ENUM("AK4458 De-emphasis Response DAC4", ak4458_dac_enum[3]),
	SOC_ENUM_EXT("AK4458 Digital Filter Setting", ak4458_dac_enum[4],
		     get_digfil, set_digfil),
	SOC_ENUM("AK4458 Inverting Enable of DZFB", ak4458_dac_enum[5]),
	SOC_ENUM("AK4458 Sound Mode", ak4458_dac_enum[6]),
	SOC_ENUM_EXT("AK4458 SDS Setting", ak4458_dac_enum[7],
		     get_sds, set_sds),
	SOC_ENUM("AK4458 FIR Filter Mode Setting", ak4458_dac_enum[8]),
	SOC_ENUM("AK4458 TDM Mode Setting", ak4458_dac_enum[9]),
	SOC_ENUM_EXT("AK4458 DAC1 LRch Setting", ak4458_dac_enum[10],
		     get_DAC1_LR, set_DAC1_LR),
	SOC_ENUM_EXT("AK4458 DAC2 LRch Setting", ak4458_dac_enum[10],
		     get_DAC2_LR, set_DAC2_LR),
	SOC_ENUM_EXT("AK4458 DAC3 LRch Setting", ak4458_dac_enum[10],
		     get_DAC3_LR, set_DAC3_LR),
	SOC_ENUM_EXT("AK4458 DAC4 LRch Setting", ak4458_dac_enum[10],
		     get_DAC4_LR, set_DAC4_LR),
	SOC_ENUM("AK4458 Attenuation transition Time Setting",
		 ak4458_dac_enum[11]),
	SOC_ENUM("AK4458 BICK fs Setting", ak4458_dac_enum[12]),
};

static const char * const ak4458_dac_select_texts[] = { "OFF", "ON" };

static const struct soc_enum ak4458_dac_mux_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(ak4458_dac_select_texts),
			ak4458_dac_select_texts);
static const struct snd_kcontrol_new ak4458_dac1_mux_control =
	SOC_DAPM_ENUM("DAC1 Switch", ak4458_dac_mux_enum);
static const struct snd_kcontrol_new ak4458_dac2_mux_control =
	SOC_DAPM_ENUM("DAC2 Switch", ak4458_dac_mux_enum);
static const struct snd_kcontrol_new ak4458_dac3_mux_control =
	SOC_DAPM_ENUM("DAC3 Switch", ak4458_dac_mux_enum);
static const struct snd_kcontrol_new ak4458_dac4_mux_control =
	SOC_DAPM_ENUM("DAC4 Switch", ak4458_dac_mux_enum);

/* ak4458 dapm widgets */
static const struct snd_soc_dapm_widget ak4458_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("AK4458 DAC1", NULL, AK4458_0A_CONTROL6, 2, 0),/*pw*/
	SND_SOC_DAPM_AIF_IN("AK4458 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("AK4458 AOUTA"),

	SND_SOC_DAPM_DAC("AK4458 DAC2", NULL, AK4458_0A_CONTROL6, 3, 0),/*pw*/
	SND_SOC_DAPM_OUTPUT("AK4458 AOUTB"),

	SND_SOC_DAPM_DAC("AK4458 DAC3", NULL, AK4458_0B_CONTROL7, 2, 0),/*pw*/
	SND_SOC_DAPM_OUTPUT("AK4458 AOUTC"),

	SND_SOC_DAPM_DAC("AK4458 DAC4", NULL, AK4458_0B_CONTROL7, 3, 0),/*pw*/
	SND_SOC_DAPM_OUTPUT("AK4458 AOUTD"),

	SND_SOC_DAPM_MUX("DAC1 to AOUTA", SND_SOC_NOPM, 0, 0,
			      &ak4458_dac1_mux_control),/*nopm*/
	SND_SOC_DAPM_MUX("DAC2 to AOUTB", SND_SOC_NOPM, 0, 0,
			      &ak4458_dac2_mux_control),/*nopm*/
	SND_SOC_DAPM_MUX("DAC3 to AOUTC", SND_SOC_NOPM, 0, 0,
			      &ak4458_dac3_mux_control),/*nopm*/
	SND_SOC_DAPM_MUX("DAC4 to AOUTD", SND_SOC_NOPM, 0, 0,
			      &ak4458_dac4_mux_control),/*nopm*/
};

static const struct snd_soc_dapm_route ak4458_intercon[] = {
	{"DAC1 to AOUTA",	"ON",	"AK4458 SDTI"},
	{"AK4458 DAC1",		NULL, "DAC1 to AOUTA"},
	{"AK4458 AOUTA",	NULL, "AK4458 DAC1"},

	{"DAC2 to AOUTB",	"ON",	"AK4458 SDTI"},
	{"AK4458 DAC2",		NULL, "DAC2 to AOUTB"},
	{"AK4458 AOUTB",	NULL, "AK4458 DAC2"},

	{"DAC3 to AOUTC",	"ON",	"AK4458 SDTI"},
	{"AK4458 DAC3",		NULL,	"DAC3 to AOUTC"},
	{"AK4458 AOUTC",	NULL,	"AK4458 DAC3"},

	{"DAC4 to AOUTD",	"ON",	"AK4458 SDTI"},
	{"AK4458 DAC4",		NULL,	"DAC4 to AOUTD"},
	{"AK4458 AOUTD",	NULL,   "AK4458 DAC4"},

};

static int ak4458_rstn_control(struct snd_soc_codec *codec, int bit)
{

	u8 rstn;

	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	rstn = snd_soc_read(codec, AK4458_00_CONTROL1);
	rstn &= ~AK4458_RSTN_MASK;

	if (bit)
		rstn |= AK4458_RSTN;

	snd_soc_write(codec, AK4458_00_CONTROL1, rstn);

	return 0;
}

static int ak4458_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

#ifdef AK4458_ACKS_USE_MANUAL_MODE
	u8 dfs1, dfs2;
#endif
	int nfs1;

	dev_dbg(dai->dev, "%s(%d)\n", __func__, __LINE__);

	nfs1 = params_rate(params);
	ak4458->fs = nfs1;

#ifdef AK4458_ACKS_USE_MANUAL_MODE
	dfs1 = snd_soc_read(codec, AK4458_01_CONTROL2);
	dfs1 &= ~AK4458_DFS01_MASK;

	dfs2 = snd_soc_read(codec, AK4458_05_CONTROL4);
	dfs2 &= ~AK4458_DFS2__MASK;

	switch (nfs1) {
	case 32000:
	case 44100:
	case 48000:
		dfs1 |= AK4458_DFS01_48KHZ;
		dfs2 |= AK4458_DFS2__48KHZ;
		break;
	case 88200:
	case 96000:
		dfs1 |= AK4458_DFS01_96KHZ;
		dfs2 |= AK4458_DFS2__96KHZ;
		break;
	case 176400:
	case 192000:
		dfs1 |= AK4458_DFS01_192KHZ;
		dfs2 |= AK4458_DFS2__192KHZ;
		break;
	case 384000:
		dfs1 |= AK4458_DFS01_384KHZ;
		dfs2 |= AK4458_DFS2__384KHZ;
		break;
	case 768000:
		dfs1 |= AK4458_DFS01_768KHZ;
		dfs2 |= AK4458_DFS2__768KHZ;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, AK4458_01_CONTROL2, dfs1);
	snd_soc_write(codec, AK4458_05_CONTROL4, dfs2);

	ak4458_rstn_control(codec, 0);
	ak4458_rstn_control(codec, 1);

#else
	snd_soc_update_bits(codec, AK4458_00_CONTROL1, 0x80, 0x80);
#endif

	return 0;
}

static int ak4458_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				 unsigned int freq, int dir)
{
	dev_dbg(dai->dev, "%s(%d)\n", __func__, __LINE__);

	return 0;
}

static int ak4458_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 format;

	/* set master/slave audio interface */
	format = snd_soc_read(codec, AK4458_00_CONTROL1);

	dev_dbg(dai->dev, "%s(%d) addr 00H = %02X\n",
		 __func__, __LINE__, format);

	format &= ~AK4458_DIF_MASK;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS: /* Slave Mode */
		break;
	case SND_SOC_DAIFMT_CBM_CFM: /* Master Mode is not supported */
	case SND_SOC_DAIFMT_CBS_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
	default:
		dev_err(codec->dev, "Master mode unsupported\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= AK4458_DIF_I2S_LOW_FS_MODE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= AK4458_DIF_MSB_LOW_FS_MODE;
		break;
	default:
		dev_err(codec->dev, "Audio format 0x%02X unsupported\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	/* set format */
	dev_dbg(dai->dev, "%s(%d) addr 00H = %02X\n",
		__func__, __LINE__, format);
	snd_soc_write(codec, AK4458_00_CONTROL1, format);

	ak4458_rstn_control(codec, 0);
	ak4458_rstn_control(codec, 1);

	return 0;
}

static int ak4458_trigger(struct snd_pcm_substream *substream, int cmd,
			  struct snd_soc_dai *codec_dai)
{
	int ret = 0;

	dev_dbg(codec_dai->dev, "%s(%d)\n", __func__, __LINE__);
	return ret;
}


static int ak4458_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}

	return 0;
}

static const int att_speed[] = { 4080, 2040, 510, 255 };

static int ak4458_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);
	int nfs, ndt, ret, reg;
	int ats;

	nfs = ak4458->fs;

	reg = snd_soc_read(codec, AK4458_0B_CONTROL7);
	ats = (reg & 0xC0) >> 6;

	dev_dbg(dai->dev, "%s mute[%s] nfs[%d]\n", __func__,
		 mute ? "ON":"OFF", nfs);

	ndt = att_speed[ats] / (nfs / 1000);

	if (mute) { /* SMUTE: 1 , MUTE */
		ret = snd_soc_update_bits(codec, AK4458_01_CONTROL2,  0x01, 1);
		mdelay(ndt);
		dev_dbg(dai->dev, "%s(%d) mdelay(%d ms)\n",
			__func__, __LINE__, ndt);

		if (gpio_is_valid(ak4458->mute_gpio))
			gpio_set_value_cansleep(ak4458->mute_gpio, 1);

		dev_dbg(dai->dev, "%s External Mute = ON\n", __func__);
	} else { /* SMUTE: 0 ,NORMAL operation */
		if (gpio_is_valid(ak4458->mute_gpio))
			gpio_set_value_cansleep(ak4458->mute_gpio, 0);

		dev_dbg(dai->dev, "%s External Mute = OFF\n", __func__);
		ret = snd_soc_update_bits(codec, AK4458_01_CONTROL2, 0x01, 0);
		mdelay(ndt);
		dev_dbg(dai->dev, "%s(%d) mdelay(%d ms)\n",
			__func__, __LINE__, ndt);
	}
	dev_dbg(dai->dev, "%s(%d) ret[%d]\n", __func__, __LINE__, ret);

	return 0;
}

#define AK4458_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
	SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
	SNDRV_PCM_RATE_192000)
	/* | SNDRV_PCM_RATE_384000 | SNDRV_PCM_RATE_768000 */

#define AK4458_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)


static struct snd_soc_dai_ops ak4458_dai_ops = {
	.hw_params	= ak4458_hw_params,
	.set_sysclk	= ak4458_set_dai_sysclk,
	.set_fmt	= ak4458_set_dai_fmt,
	.trigger = ak4458_trigger,
	.digital_mute = ak4458_set_dai_mute,
};

static struct snd_soc_dai_driver ak4458_dai = {
	.name = "ak4458-aif",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = AK4458_RATES,
		.formats = AK4458_FORMATS,
	},
	.ops = &ak4458_dai_ops,
};

static void ak4458_init_reg(struct snd_soc_codec *codec)
{
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	/* External Mute ON */
	if (gpio_is_valid(ak4458->mute_gpio))
		gpio_set_value_cansleep(ak4458->mute_gpio, 1);

	if (gpio_is_valid(ak4458->pdn_gpio)) {
		gpio_set_value_cansleep(ak4458->pdn_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(ak4458->pdn_gpio, 1);
		usleep_range(1000, 2000);
	}

	ak4458_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

#ifndef AK4458_ACKS_USE_MANUAL_MODE
	snd_soc_update_bits(codec, AK4458_00_CONTROL1,
			    0x80, 0x80);   /* ACKS bit = 1; 10000000 */
	dev_dbg(codec->dev, "%s ACKS bit = 1\n", __func__);
#endif

	ak4458_rstn_control(codec, 0);
	ak4458_rstn_control(codec, 1);
}

static int ak4458_codec_probe(struct snd_soc_codec *codec)
{
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	ak4458_init_reg(codec);

	ak4458->fs = 48000;

	return 0;
}

static int ak4458_codec_remove(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	ak4458_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ak4458_suspend(struct snd_soc_codec *codec)
{
	struct ak4458_priv *ak4458 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s(%d)\n", __func__, __LINE__);

	ak4458_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if (gpio_is_valid(ak4458->pdn_gpio))
		gpio_set_value_cansleep(ak4458->pdn_gpio, 0);

	return 0;
}

static int ak4458_resume(struct snd_soc_codec *codec)
{
	ak4458_init_reg(codec);

	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4458 = {
	.probe = ak4458_codec_probe,
	.remove = ak4458_codec_remove,
	.suspend =	ak4458_suspend,
	.resume =	ak4458_resume,
	.set_bias_level = ak4458_set_bias_level,

	.component_driver = {
		.controls = ak4458_snd_controls,
		.num_controls = ARRAY_SIZE(ak4458_snd_controls),
		.dapm_widgets = ak4458_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(ak4458_dapm_widgets),
		.dapm_routes = ak4458_intercon,
		.num_dapm_routes = ARRAY_SIZE(ak4458_intercon),
	},
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4458);

const struct regmap_config ak4458_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK4458_14_R4CHATT,
	.reg_defaults = ak4458_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4458_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(ak4458_i2c_regmap_config);

const struct regmap_config ak4458_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK4458_14_R4CHATT,
	.rd_table = &ak4458_spi_readable_regs,
	.reg_defaults = ak4458_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4458_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(ak4458_spi_regmap_config);

int ak4458_probe(struct device *dev, struct regmap *regmap)
{
	struct ak4458_priv *ak4458;
	struct device_node *np = dev->of_node;
	int ret;

	ak4458 = devm_kzalloc(dev, sizeof(*ak4458), GFP_KERNEL);
	if (!ak4458)
		return -ENOMEM;

	dev_set_drvdata(dev, ak4458);

	ak4458->dev = dev;
	ak4458->regmap = regmap;

	ak4458->pdn_gpio = of_get_named_gpio(np, "ak4458,pdn-gpio", 0);
	if (gpio_is_valid(ak4458->pdn_gpio)) {
		ret = devm_gpio_request_one(dev, ak4458->pdn_gpio,
				GPIOF_OUT_INIT_LOW, "ak4458,pdn");
		if (ret) {
			dev_err(dev, "unable to get pdn gpio\n");
			return ret;
		}
	}

	ak4458->mute_gpio = of_get_named_gpio(np, "ak4458,mute_gpio", 0);
	if (gpio_is_valid(ak4458->mute_gpio)) {
		ret = devm_gpio_request_one(dev, ak4458->pdn_gpio,
				GPIOF_OUT_INIT_LOW, "ak4458,mute");
		if (ret) {
			dev_err(dev, "unable to get mute gpio\n");
			return ret;
		}
	}

	ret = snd_soc_register_codec(dev, &soc_codec_dev_ak4458,
				     &ak4458_dai, 1);
	if (ret < 0) {
		dev_err(dev, "Failed to register CODEC: %d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ak4458_probe);

void ak4458_remove(struct device *dev)
{
	snd_soc_unregister_codec(dev);
}
EXPORT_SYMBOL_GPL(ak4458_remove);

#if IS_ENABLED(CONFIG_PM)
static int ak4458_runtime_resume(struct device *dev)
{
	struct ak4458_priv *ak4458 = dev_get_drvdata(dev);

	regcache_sync(ak4458->regmap);

	/* TODO Power up*/

	return 0;
}

static int ak4458_runtime_suspend(struct device *dev)
{
	/* TODO Power down */

	return 0;
}
#endif

const struct dev_pm_ops ak4458_pm = {
	SET_RUNTIME_PM_OPS(ak4458_runtime_suspend, ak4458_runtime_resume, NULL)
};
EXPORT_SYMBOL_GPL(ak4458_pm);

MODULE_AUTHOR("Junichi Wakasugi <wakasugi.jb@om.asahi-kasei.co.jp>");
MODULE_AUTHOR("Mihai Serban <mihai.serban@nxp.com>");
MODULE_DESCRIPTION("ASoC AK4458 DAC driver");
MODULE_LICENSE("GPL");
