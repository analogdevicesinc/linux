// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for ADAU1860 codec
 *
 * Copyright 2022 Analog Devices Inc.
 * Author: Bogdan Togorean <bogdan.togorean@analog.com>
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/adau1860.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "adau17x1.h"
#include "adau1860.h"

#define DRV_NAME "adau1860-codec"

#define ADAU1860_FIRMWARE "adau1860.bin"

static const DECLARE_TLV_DB_MINMAX_MUTE(adau1860_adc_tlv, -7125, 2400);
static const DECLARE_TLV_DB_MINMAX_MUTE(adau1860_dac_tlv, -7125, 2400);
static const DECLARE_TLV_DB_SCALE(adau1860_pga_tlv, 0, 75, 0);

static const unsigned int adau1860_bias_select_extreme_values[] = {
	0,
	2,
	3,
};

static const char *const adau1860_bias_select_extreme_text[] = {
	"Normal operation",
	"Enhanced performance",
	"Extreme Power saving",
};

static const char *const adau1860_bias_select_text[] = {
	"Normal operation",
	"Enhanced performance",
};

static SOC_ENUM_SINGLE_DECL(adau1860_playback_bias_enum, ADAU1860_PB_CTRL, 2,
			    adau1860_bias_select_text);

static const char *const adau1860_adc_hpf_select_text[] = {
	"Off",
	"1Hz",
	"4Hz",
	"8Hz",
};

const struct soc_enum adau1860_adc_hpf_enum[] = {
	SOC_ENUM_SINGLE(ADAU1860_ADC_CTRL4, 0,
			ARRAY_SIZE(adau1860_adc_hpf_select_text),
			adau1860_adc_hpf_select_text),
	SOC_ENUM_SINGLE(ADAU1860_ADC_CTRL4, 2,
			ARRAY_SIZE(adau1860_adc_hpf_select_text),
			adau1860_adc_hpf_select_text),
	SOC_ENUM_SINGLE(ADAU1860_ADC_CTRL4, 4,
			ARRAY_SIZE(adau1860_adc_hpf_select_text),
			adau1860_adc_hpf_select_text),
};

const struct soc_enum adau1860_adc_bias_enum[] = {
	SOC_VALUE_ENUM_SINGLE(ADAU1860_ADC_CTRL2, 0, 0x7,
			      ARRAY_SIZE(adau1860_bias_select_extreme_text),
			      adau1860_bias_select_extreme_text,
			      adau1860_bias_select_extreme_values),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_ADC_CTRL2, 4, 0x7,
			      ARRAY_SIZE(adau1860_bias_select_extreme_text),
			      adau1860_bias_select_extreme_text,
			      adau1860_bias_select_extreme_values),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_ADC_CTRL3, 0, 0x7,
			      ARRAY_SIZE(adau1860_bias_select_extreme_text),
			      adau1860_bias_select_extreme_text,
			      adau1860_bias_select_extreme_values),
};

const struct soc_enum adau1860_pga_bias_enum[] = {
	SOC_ENUM_SINGLE(ADAU1860_PGA_CTRL2, 4,
			ARRAY_SIZE(adau1860_bias_select_text),
			adau1860_bias_select_text),
	SOC_ENUM_SINGLE(ADAU1860_PGA_CTRL2, 5,
			ARRAY_SIZE(adau1860_bias_select_text),
			adau1860_bias_select_text),
	SOC_ENUM_SINGLE(ADAU1860_PGA_CTRL2, 6,
			ARRAY_SIZE(adau1860_bias_select_text),
			adau1860_bias_select_text),
};

static const struct snd_kcontrol_new adau1860_pga_controls[] = {
	SOC_SINGLE_TLV("PGA0 Capture Volume", ADAU1860_PGA0_CTRL2, 0, 0x20, 0,
		       adau1860_pga_tlv),
	SOC_SINGLE_TLV("PGA1 Capture Volume", ADAU1860_PGA1_CTRL2, 0, 0x20, 0,
		       adau1860_pga_tlv),
	SOC_SINGLE_TLV("PGA2 Capture Volume", ADAU1860_PGA2_CTRL2, 0, 0x20, 0,
		       adau1860_pga_tlv),
	SOC_ENUM("PGA0 Bias Capture Switch", adau1860_pga_bias_enum[0]),
	SOC_ENUM("PGA1 Bias Capture Switch", adau1860_pga_bias_enum[1]),
	SOC_ENUM("PGA2 Bias Capture Switch", adau1860_pga_bias_enum[2]),
};

static const struct snd_kcontrol_new adau1860_single_mode_controls[] = {
	SOC_SINGLE_TLV("ADC0 Capture Volume", ADAU1860_ADC0_VOL, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("ADC0 Capture Switch", ADAU1860_ADC_MUTES, 0, 1, 1),
	SOC_SINGLE_TLV("ADC1 Capture Volume", ADAU1860_ADC1_VOL, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("ADC1 Capture Switch", ADAU1860_ADC_MUTES, 1, 1, 1),
	SOC_SINGLE_TLV("ADC2 Capture Volume", ADAU1860_ADC2_VOL, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("ADC2 Capture Switch", ADAU1860_ADC_MUTES, 2, 1, 1),
	SOC_ENUM("ADC0 Bias Capture Switch", adau1860_adc_bias_enum[0]),
	SOC_ENUM("ADC1 Bias Capture Switch", adau1860_adc_bias_enum[1]),
	SOC_ENUM("ADC2 Bias Capture Switch", adau1860_adc_bias_enum[2]),
	SOC_ENUM("ADC0 HPF Capture Switch", adau1860_adc_hpf_enum[0]),
	SOC_ENUM("ADC1 HPF Capture Switch", adau1860_adc_hpf_enum[1]),
	SOC_ENUM("ADC2 HPF Capture Switch", adau1860_adc_hpf_enum[2]),
};

static const struct snd_kcontrol_new adau1860_controls[] = {
	SOC_SINGLE_TLV("DMIC0 Capture Volume", ADAU1860_DMIC_VOL0, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC0 Capture Switch", ADAU1860_DMIC_MUTES, 0, 1, 1),
	SOC_SINGLE_TLV("DMIC1 Capture Volume", ADAU1860_DMIC_VOL1, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC1 Capture Switch", ADAU1860_DMIC_MUTES, 1, 1, 1),
	SOC_SINGLE_TLV("DMIC2 Capture Volume", ADAU1860_DMIC_VOL2, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC2 Capture Switch", ADAU1860_DMIC_MUTES, 2, 1, 1),
	SOC_SINGLE_TLV("DMIC3 Capture Volume", ADAU1860_DMIC_VOL3, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC3 Capture Switch", ADAU1860_DMIC_MUTES, 3, 1, 1),
	SOC_SINGLE_TLV("DMIC4 Capture Volume", ADAU1860_DMIC_VOL4, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC4 Capture Switch", ADAU1860_DMIC_MUTES, 4, 1, 1),
	SOC_SINGLE_TLV("DMIC5 Capture Volume", ADAU1860_DMIC_VOL5, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC5 Capture Switch", ADAU1860_DMIC_MUTES, 5, 1, 1),
	SOC_SINGLE_TLV("DMIC6 Capture Volume", ADAU1860_DMIC_VOL6, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC6 Capture Switch", ADAU1860_DMIC_MUTES, 6, 1, 1),
	SOC_SINGLE_TLV("DMIC7 Capture Volume", ADAU1860_DMIC_VOL7, 0, 0xff, 1,
		       adau1860_adc_tlv),
	SOC_SINGLE("DMIC7 Capture Switch", ADAU1860_DMIC_MUTES, 7, 1, 1),
	SOC_SINGLE_TLV("DAC0 Playback Volume", ADAU1860_DAC_VOL0, 0, 0xff, 1,
		       adau1860_dac_tlv),
	SOC_SINGLE("DAC0 Playback Switch", ADAU1860_DAC_CTRL2, 6, 1, 1),
};

static const unsigned int adau1860_spt_mux_val[] = {
	0,  1,	2,  3,	4,  5,	6,  7,	8,  9,	10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
	30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
	45, 46, 47, 48, 49, 50, 51, 55, 56, 57, 58, 63
};

static const char *const adau1860_spt_mux_text[] = {
	"FDSP Ch0",	"FDSP Ch1",	"FDSP Ch2",	"FDSP Ch3",
	"FDSP Ch4",	"FDSP Ch5",	"FDSP Ch6",	"FDSP Ch7",
	"FDSP Ch8",	"FDSP Ch9",	"FDSP Ch10",	"FDSP Ch11",
	"FDSP Ch12",	"FDSP Ch13",	"FDSP Ch14",	"FDSP Ch15",
	"TDSP Ch0",	"TDSP Ch1",	"TDSP Ch2",	"TDSP Ch3",
	"TDSP Ch4",	"TDSP Ch5",	"TDSP Ch6",	"TDSP Ch7",
	"TDSP Ch8",	"TDSP Ch9",	"TDSP Ch10",	"TDSP Ch11",
	"TDSP Ch12",	"TDSP Ch13",	"TDSP Ch14",	"TDSP Ch15",
	"OUT ASRC Ch0", "OUT ASRC Ch1", "OUT ASRC Ch2", "OUT ASRC Ch3",
	"ADC0",		"ADC1",		"ADC2",		"DMIC0",
	"DMIC1",	"DMIC2",	"DMIC3",	"FDEC Ch0",
	"FDEC Ch1",	"FDEC Ch2",	"FDEC Ch3",	"FDEC Ch4",
	"FDEC Ch5",	"FDEC Ch6",	"FDEC Ch7",	"EQ0",
	"DMIC4",	"DMIC5",	"DMIC6",	"DMIC7",
	"Not Used",
};

const struct soc_enum adau1860_spt0_mux_enums[] = {
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(0), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(1), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(2), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(3), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(4), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(5), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(6), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(7), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(8), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(9), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(10), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(11), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(12), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(13), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(14), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(15), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
};

const struct soc_enum adau1860_spt1_mux_enums[] = {
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(0), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(1), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(2), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(3), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(4), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(5), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(6), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(7), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(8), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(9), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(10), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(11), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(12), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(13), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(14), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(15), 0, 0x3f,
			      ARRAY_SIZE(adau1860_spt_mux_text),
			      adau1860_spt_mux_text, adau1860_spt_mux_val),
};

static const struct snd_kcontrol_new adau1860_spt0_out_controls[] = {
	SOC_DAPM_ENUM("SPT0 Out Slot 0", adau1860_spt0_mux_enums[0]),
	SOC_DAPM_ENUM("SPT0 Out Slot 1", adau1860_spt0_mux_enums[1]),
	SOC_DAPM_ENUM("SPT0 Out Slot 2", adau1860_spt0_mux_enums[2]),
	SOC_DAPM_ENUM("SPT0 Out Slot 3", adau1860_spt0_mux_enums[3]),
	SOC_DAPM_ENUM("SPT0 Out Slot 4", adau1860_spt0_mux_enums[4]),
	SOC_DAPM_ENUM("SPT0 Out Slot 5", adau1860_spt0_mux_enums[5]),
	SOC_DAPM_ENUM("SPT0 Out Slot 6", adau1860_spt0_mux_enums[6]),
	SOC_DAPM_ENUM("SPT0 Out Slot 7", adau1860_spt0_mux_enums[7]),
	SOC_DAPM_ENUM("SPT0 Out Slot 8", adau1860_spt0_mux_enums[8]),
	SOC_DAPM_ENUM("SPT0 Out Slot 9", adau1860_spt0_mux_enums[9]),
	SOC_DAPM_ENUM("SPT0 Out Slot 10", adau1860_spt0_mux_enums[10]),
	SOC_DAPM_ENUM("SPT0 Out Slot 11", adau1860_spt0_mux_enums[11]),
	SOC_DAPM_ENUM("SPT0 Out Slot 12", adau1860_spt0_mux_enums[12]),
	SOC_DAPM_ENUM("SPT0 Out Slot 13", adau1860_spt0_mux_enums[13]),
	SOC_DAPM_ENUM("SPT0 Out Slot 14", adau1860_spt0_mux_enums[14]),
	SOC_DAPM_ENUM("SPT0 Out Slot 15", adau1860_spt0_mux_enums[15]),
};

static const struct snd_kcontrol_new adau1860_spt1_out_controls[] = {
	SOC_DAPM_ENUM("SPT1 Out Slot 0", adau1860_spt1_mux_enums[0]),
	SOC_DAPM_ENUM("SPT1 Out Slot 1", adau1860_spt1_mux_enums[1]),
	SOC_DAPM_ENUM("SPT1 Out Slot 2", adau1860_spt1_mux_enums[2]),
	SOC_DAPM_ENUM("SPT1 Out Slot 3", adau1860_spt1_mux_enums[3]),
	SOC_DAPM_ENUM("SPT1 Out Slot 4", adau1860_spt1_mux_enums[4]),
	SOC_DAPM_ENUM("SPT1 Out Slot 5", adau1860_spt1_mux_enums[5]),
	SOC_DAPM_ENUM("SPT1 Out Slot 6", adau1860_spt1_mux_enums[6]),
	SOC_DAPM_ENUM("SPT1 Out Slot 7", adau1860_spt1_mux_enums[7]),
	SOC_DAPM_ENUM("SPT1 Out Slot 8", adau1860_spt1_mux_enums[8]),
	SOC_DAPM_ENUM("SPT1 Out Slot 9", adau1860_spt1_mux_enums[9]),
	SOC_DAPM_ENUM("SPT1 Out Slot 10", adau1860_spt1_mux_enums[10]),
	SOC_DAPM_ENUM("SPT1 Out Slot 11", adau1860_spt1_mux_enums[11]),
	SOC_DAPM_ENUM("SPT1 Out Slot 12", adau1860_spt1_mux_enums[12]),
	SOC_DAPM_ENUM("SPT1 Out Slot 13", adau1860_spt1_mux_enums[13]),
	SOC_DAPM_ENUM("SPT1 Out Slot 14", adau1860_spt1_mux_enums[14]),
	SOC_DAPM_ENUM("SPT1 Out Slot 15", adau1860_spt1_mux_enums[15]),
};

static const char *const adau1860_dac_mux_text[] = {
	"SPT0 Ch0",    "SPT0 Ch1",    "SPT0 Ch2",    "SPT0 Ch3",  "SPT0 Ch4",
	"SPT0 Ch5",    "SPT0 Ch6",    "SPT0 Ch7",    "SPT0 Ch8",  "SPT0 Ch9",
	"SPT0 Ch10",   "SPT0 Ch11",   "SPT0 Ch12",   "SPT0 Ch13", "SPT0 Ch14",
	"SPT0 Ch15",   "SPT1 Ch0",    "SPT1 Ch1",    "SPT1 Ch2",  "SPT1 Ch3",
	"SPT1 Ch4",    "SPT1 Ch5",    "SPT1 Ch6",    "SPT1 Ch7",  "SPT1 Ch8",
	"SPT1 Ch9",    "SPT1 Ch10",   "SPT1 Ch11",   "SPT1 Ch12", "SPT1 Ch13",
	"SPT1 Ch14",   "SPT1 Ch15",   "FDSP Ch0",    "FDSP Ch1",  "FDSP Ch2",
	"FDSP Ch3",    "FDSP Ch4",    "FDSP Ch5",    "FDSP Ch6",  "FDSP Ch7",
	"FDSP Ch8",    "FDSP Ch9",    "FDSP Ch10",   "FDSP Ch11", "FDSP Ch12",
	"FDSP Ch13",   "FDSP Ch14",   "FDSP Ch15",   "TDSP Ch0",  "TDSP Ch1",
	"TDSP Ch2",    "TDSP Ch3",    "TDSP Ch4",    "TDSP Ch5",  "TDSP Ch6",
	"TDSP Ch7",    "TDSP Ch8",    "TDSP Ch9",    "TDSP Ch10", "TDSP Ch11",
	"TDSP Ch12",   "TDSP Ch13",   "TDSP Ch14",   "TDSP Ch15", "IN ASRC Ch0",
	"IN ASRC Ch1", "IN ASRC Ch2", "IN ASRC Ch3", "ADC0",	  "ADC1",
	"ADC2",	       "DMIC0",	      "DMIC1",	     "DMIC2",	  "DMIC3",
	"EQ0",	       "FINT Ch0",    "FINT Ch1",    "FINT Ch2",  "FINT Ch3",
	"FINT Ch4",    "FINT Ch5",    "FINT Ch6",    "FINT Ch7",  "DMIC4",
	"DMIC5",       "DMIC6",	      "DMIC7",
};

static SOC_ENUM_SINGLE_DECL(adau1860_dac_mux_enum, ADAU1860_DAC_ROUTE0, 0,
			    adau1860_dac_mux_text);

static const struct snd_kcontrol_new adau1860_dac_mux_control =
	SOC_DAPM_ENUM("DAC Source", adau1860_dac_mux_enum);

static const char *const adau1860_input_mux_text[] = {
	"ADC",
	"PGA",
};

const struct soc_enum adau1860_input_mux_enums[] = {
	SOC_ENUM_SINGLE(ADAU1860_ADC_CTRL7, 4,
			ARRAY_SIZE(adau1860_input_mux_text),
			adau1860_input_mux_text),
	SOC_ENUM_SINGLE(ADAU1860_ADC_CTRL7, 5,
			ARRAY_SIZE(adau1860_input_mux_text),
			adau1860_input_mux_text),
	SOC_ENUM_SINGLE(ADAU1860_ADC_CTRL7, 6,
			ARRAY_SIZE(adau1860_input_mux_text),
			adau1860_input_mux_text),
};
static const struct snd_kcontrol_new adau1860_input_mux_controls[] = {
	SOC_DAPM_ENUM("ADC0 Input Select", adau1860_input_mux_enums[0]),
	SOC_DAPM_ENUM("ADC1 Input Select", adau1860_input_mux_enums[1]),
	SOC_DAPM_ENUM("ADC2 Input Select", adau1860_input_mux_enums[2]),
};

static const struct snd_soc_dapm_widget adau1860_dapm_widgets[] = {
	SND_SOC_DAPM_REGULATOR_SUPPLY("HPVDD", 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("IOVDD", 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("DVDD", 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("AVDD", 0, 0),

	SND_SOC_DAPM_SUPPLY("Headphone", ADAU1860_HPLDO_CTRL, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("MASTER_BLOCK_EN", ADAU1860_CHIP_PWR, 2, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("ADP", ADAU1860_CHIP_PWR, 0, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SOC", ADAU1860_CHIP_PWR, 1, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("PLL", ADAU1860_PLL_PGA_PWR, 0, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SYSCLK", SND_SOC_NOPM, 0, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC_CLK", ADAU1860_SAI_CLK_PWR, 4, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC_CLK1", ADAU1860_SAI_CLK_PWR, 5, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SPT0_OUT_CLK", ADAU1860_SAI_CLK_PWR, 1, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SPT0_IN_CLK", ADAU1860_SAI_CLK_PWR, 0, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SPT1_OUT_CLK", ADAU1860_SAI_CLK_PWR, 3, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SPT1_IN_CLK", ADAU1860_SAI_CLK_PWR, 2, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("DAC MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_dac_mux_control),

	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT0 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[0]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT1 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[1]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT2 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[2]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT3 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[3]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT4 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[4]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT5 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[5]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT6 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[6]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT7 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[7]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT8 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[8]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT9 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[9]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT10 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[10]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT11 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[11]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT12 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[12]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT13 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[13]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT14 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[14]),
	SND_SOC_DAPM_MUX("SPT0_OUT_SLOT15 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt0_out_controls[15]),

	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT0 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[0]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT1 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[1]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT2 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[2]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT3 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[3]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT4 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[4]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT5 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[5]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT6 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[6]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT7 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[7]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT8 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[8]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT9 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[9]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT10 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[10]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT11 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[11]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT12 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[12]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT13 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[13]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT14 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[14]),
	SND_SOC_DAPM_MUX("SPT1_OUT_SLOT15 DATA MUX", SND_SOC_NOPM, 0, 0,
			 &adau1860_spt1_out_controls[15]),

	SND_SOC_DAPM_AIF_OUT("SPT0_OUT", "SAI0 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SPT1_OUT", "SAI1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SPT0_IN", "SAI0 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SPT1_IN", "SAI1 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_INPUT("AINP0"),
	SND_SOC_DAPM_INPUT("AINN0"),
	SND_SOC_DAPM_INPUT("AINP1"),
	SND_SOC_DAPM_INPUT("AINN1"),
	SND_SOC_DAPM_INPUT("AINP2"),
	SND_SOC_DAPM_INPUT("AINN2"),

	SND_SOC_DAPM_INPUT("DMIC01"),
	SND_SOC_DAPM_INPUT("DMIC23"),

	SND_SOC_DAPM_PGA("PGA0", ADAU1860_PLL_PGA_PWR, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA1", ADAU1860_PLL_PGA_PWR, 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA2", ADAU1860_PLL_PGA_PWR, 6, 0, NULL, 0),

	SND_SOC_DAPM_ADC("ADC0", NULL, ADAU1860_ADC_DAC_HP_PWR, 0, 0),
	SND_SOC_DAPM_ADC("ADC1", NULL, ADAU1860_ADC_DAC_HP_PWR, 1, 0),
	SND_SOC_DAPM_ADC("ADC2", NULL, ADAU1860_ADC_DAC_HP_PWR, 2, 0),
	SND_SOC_DAPM_ADC("DMIC0", NULL, ADAU1860_DMIC_PWR, 0, 0),
	SND_SOC_DAPM_ADC("DMIC1", NULL, ADAU1860_DMIC_PWR, 1, 0),
	SND_SOC_DAPM_ADC("DMIC2", NULL, ADAU1860_DMIC_PWR, 2, 0),
	SND_SOC_DAPM_ADC("DMIC3", NULL, ADAU1860_DMIC_PWR, 3, 0),
	SND_SOC_DAPM_ADC("DMIC4", NULL, ADAU1860_DMIC_PWR, 4, 0),
	SND_SOC_DAPM_ADC("DMIC5", NULL, ADAU1860_DMIC_PWR, 5, 0),
	SND_SOC_DAPM_ADC("DMIC6", NULL, ADAU1860_DMIC_PWR, 6, 0),
	SND_SOC_DAPM_ADC("DMIC7", NULL, ADAU1860_DMIC_PWR, 7, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, ADAU1860_ADC_DAC_HP_PWR, 4, 0),

	SND_SOC_DAPM_OUTPUT("HPOUTP"),
	SND_SOC_DAPM_OUTPUT("HPOUTN"),
};

static int adau1860_dapm_sysclk_check(struct snd_soc_dapm_widget *source,
				      struct snd_soc_dapm_widget *sink);

#define ADAU1860_SPT_OUT_ROUTES(name)                                          \
	{ name, "ADC0", "ADC0" }, { name, "ADC1", "ADC1" },                    \
	{ name, "ADC2", "ADC2" }, { name, "DMIC0", "DMIC0" },                  \
	{ name, "DMIC1", "DMIC1" }, { name, "DMIC2", "DMIC2" },                \
	{ name, "DMIC3", "DMIC3" }, { name, "DMIC4", "DMIC4" },                \
	{ name, "DMIC5", "DMIC5" }, { name, "DMIC6", "DMIC6" },                \
	{ name, "DMIC7", "DMIC7" }

#define ADAU1860_DAC_ROUTES(name)                                              \
	{ name, "SPT0 Ch0", "SPT0_IN" }, { name, "SPT0 Ch1", "SPT0_IN" },      \
	{ name, "SPT0 Ch2", "SPT0_IN" }, { name, "SPT0 Ch3", "SPT0_IN" },      \
	{ name, "SPT0 Ch4", "SPT0_IN" }, { name, "SPT0 Ch5", "SPT0_IN" },      \
	{ name, "SPT0 Ch6", "SPT0_IN" }, { name, "SPT0 Ch7", "SPT0_IN" },      \
	{ name, "SPT0 Ch8", "SPT0_IN" }, { name, "SPT0 Ch9", "SPT0_IN" },      \
	{ name, "SPT0 Ch10", "SPT0_IN" }, { name, "SPT0 Ch11", "SPT0_IN" },    \
	{ name, "SPT0 Ch12", "SPT0_IN" }, { name, "SPT0 Ch12", "SPT0_IN" },    \
	{ name, "SPT0 Ch13", "SPT0_IN" }, { name, "SPT0 Ch14", "SPT0_IN" },    \
	{ name, "SPT0 Ch15", "SPT0_IN" }, { name, "SPT1 Ch0", "SPT1_IN" },     \
	{ name, "SPT1 Ch1", "SPT1_IN" }, { name, "SPT1 Ch2", "SPT1_IN" },      \
	{ name, "SPT1 Ch3", "SPT1_IN" }, { name, "SPT1 Ch4", "SPT1_IN" },      \
	{ name, "SPT1 Ch5", "SPT1_IN" }, { name, "SPT1 Ch6", "SPT1_IN" },      \
	{ name, "SPT1 Ch7", "SPT1_IN" }, { name, "SPT1 Ch8", "SPT1_IN" },      \
	{ name, "SPT1 Ch9", "SPT1_IN" }, { name, "SPT1 Ch10", "SPT1_IN" },     \
	{ name, "SPT1 Ch11", "SPT1_IN" }, { name, "SPT1 Ch12", "SPT1_IN" },    \
	{ name, "SPT1 Ch12", "SPT1_IN" }, { name, "SPT1 Ch13", "SPT1_IN" },    \
	{ name, "SPT1 Ch14", "SPT1_IN" }, { name, "SPT1 Ch15", "SPT1_IN" },    \
	{ name, "ADC0", "ADC0" }, { name, "ADC1", "ADC1" },                    \
	{ name, "ADC2", "ADC2" }, { name, "DMIC0", "DMIC0" },                  \
	{ name, "DMIC1", "DMIC1" }, { name, "DMIC2", "DMIC2" },                \
	{ name, "DMIC3", "DMIC3" }, { name, "DMIC4", "DMIC4" },                \
	{ name, "DMIC5", "DMIC5" }, { name, "DMIC6", "DMIC6" },                \
	{ name, "DMIC7", "DMIC7" }

static const struct snd_soc_dapm_route adau1860_dapm_routes[] = {
	/* Clock Paths */
	{ "SYSCLK", NULL, "PLL", adau1860_dapm_sysclk_check },
	{ "SYSCLK", NULL, "MASTER_BLOCK_EN" },
	{ "DMIC_CLK", NULL, "SYSCLK" },
	{ "DMIC_CLK1", NULL, "SYSCLK" },
	{ "ADP", NULL, "SYSCLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC1", NULL, "DMIC_CLK" },
	{ "DMIC2", NULL, "DMIC_CLK" },
	{ "DMIC3", NULL, "DMIC_CLK" },
	{ "DMIC4", NULL, "DMIC_CLK1" },
	{ "DMIC5", NULL, "DMIC_CLK1" },
	{ "DMIC6", NULL, "DMIC_CLK1" },
	{ "DMIC7", NULL, "DMIC_CLK1" },
	{ "SPT0_IN", NULL, "SPT0_IN_CLK" },
	{ "SPT1_IN", NULL, "SPT1_IN_CLK" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_CLK" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_CLK" },

	/* DMIC Paths */
	{ "DMIC0", NULL, "DMIC01" },
	{ "DMIC1", NULL, "DMIC01" },
	{ "DMIC2", NULL, "DMIC23" },
	{ "DMIC3", NULL, "DMIC23" },

	/* ADC Input Paths */
	{ "ADC0", NULL, "AINP0" },
	{ "ADC0", NULL, "AINN0" },
	{ "ADC1", NULL, "AINP1" },
	{ "ADC1", NULL, "AINN1" },
	{ "ADC2", NULL, "AINP2" },
	{ "ADC2", NULL, "AINN2" },
	{ "PGA0", NULL, "AINP0" },
	{ "PGA0", NULL, "AINN0" },
	{ "PGA1", NULL, "AINP1" },
	{ "PGA1", NULL, "AINN1" },
	{ "PGA2", NULL, "AINP2" },
	{ "PGA2", NULL, "AINN2" },

	/* Digital Paths */
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT0 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT1 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT2 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT3 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT4 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT5 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT6 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT7 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT8 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT9 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT10 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT11 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT12 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT13 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT14 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT0_OUT_SLOT15 DATA MUX"),
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT0 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT1 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT2 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT3 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT4 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT5 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT6 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT7 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT8 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT9 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT10 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT11 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT12 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT13 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT14 DATA MUX" },
	{ "SPT0_OUT", NULL, "SPT0_OUT_SLOT15 DATA MUX" },

	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT0 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT1 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT2 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT3 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT4 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT5 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT6 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT7 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT8 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT9 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT10 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT11 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT12 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT13 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT14 DATA MUX"),
	ADAU1860_SPT_OUT_ROUTES("SPT1_OUT_SLOT15 DATA MUX"),
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT0 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT1 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT2 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT3 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT4 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT5 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT6 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT7 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT8 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT9 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT10 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT11 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT12 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT13 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT14 DATA MUX" },
	{ "SPT1_OUT", NULL, "SPT1_OUT_SLOT15 DATA MUX" },

	/* Audio Data Paths */
	{ "ADC0", NULL, "ADP" },
	{ "ADC1", NULL, "ADP" },
	{ "ADC2", NULL, "ADP" },
	{ "DMIC01", NULL, "ADP" },
	{ "DMIC23", NULL, "ADP" },
	{ "DAC", NULL, "ADP" },

	/* Playback Paths */
	ADAU1860_DAC_ROUTES("DAC MUX"),
	{ "DAC", NULL, "DAC MUX" },
	{ "HPOUTP", NULL, "DAC" },
	{ "HPOUTN", NULL, "DAC" },
};

static int adau1860_dapm_sysclk_check(struct snd_soc_dapm_widget *source,
				      struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(source->dapm);
	struct adau18x0 *adau = snd_soc_component_get_drvdata(component);
	const char *clk;

	switch (adau->pll_src) {
	case ADAU18X0_PLL_SRC_MCLKIN:
		clk = "XTAL";
		break;
	default:
		return 0;
	}

	return strcmp(source->name, clk) == 0;
}

static void adau1860_set_power(struct snd_soc_component *component, bool enable)
{
	struct adau18x0 *adau1860 = snd_soc_component_get_drvdata(component);

	if (adau1860->enabled == enable)
		return;

	dev_dbg(component->dev, "%s param: %d", __func__, enable);

	if (enable) {
		clk_prepare_enable(adau1860->mclk);
		if (adau1860->pd_gpio)
			gpiod_set_value(adau1860->pd_gpio, 1);

		regcache_cache_only(adau1860->regmap, false);
		regcache_sync(adau1860->regmap);
	} else {
		if (adau1860->pd_gpio) {
			/*
			 * This will turn everything off and reset the register
			 * map. No need to do any register writes to manually
			 * turn things off.
			 */
			gpiod_set_value(adau1860->pd_gpio, 0);
			regcache_mark_dirty(adau1860->regmap);
		} else {
			regmap_update_bits(adau1860->regmap, ADAU1860_CHIP_PWR,
					   ADAU1860_MASTER_BLOCK_EN_MSK |
						   ADAU1860_PWR_MODE_MSK,
					   0);
		}
		clk_disable_unprepare(adau1860->mclk);
		regcache_cache_only(adau1860->regmap, true);
	}

	adau1860->enabled = enable;
}

static int adau1860_set_bias_level(struct snd_soc_component *component,
				   enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(component->dev, "%s: SND_SOC_BIAS_STANDBY", __func__);
		adau1860_set_power(component, true);
		break;
	case SND_SOC_BIAS_OFF:
		dev_dbg(component->dev, "%s: SND_SOC_BIAS_OFF", __func__);
		adau1860_set_power(component, false);
		break;
	}
	return 0;
}

static int adau1860_component_probe(struct snd_soc_component *component)
{
	struct adau1860_pdata *pdata = component->dev->platform_data;
	struct adau18x0 *adau = snd_soc_component_get_drvdata(component);

	int ret;

	if (pdata && pdata->input_differential) {
		ret = snd_soc_add_component_controls(
			component, adau1860_pga_controls,
			ARRAY_SIZE(adau1860_pga_controls));
		if (ret)
			return ret;
	} else {
		ret = snd_soc_add_component_controls(
			component, adau1860_single_mode_controls,
			ARRAY_SIZE(adau1860_single_mode_controls));
		if (ret)
			return ret;
	}

	/* ADAU1860 -> load TDSP FW */
	if (adau->type == ADAU1860) {
		dev_dbg(component->dev, "Full LARK device. Loading TDSP FW");
	}

	dev_dbg(component->dev, "LARK device. Loading FDSP FW");

	dev_dbg(component->dev, "LARK device. Loading EQ FW");

	return 0;
}

int adau1860_resume(struct snd_soc_component *component)
{
	struct adau18x0 *adau = snd_soc_component_get_drvdata(component);

	if (adau->switch_mode)
		adau->switch_mode(component->dev);

	regcache_sync(adau->regmap);

	return 0;
}

static const int adau1860_bclk_rates[] = {
	0, 3072000, 6144000, 12288000, 24576000,
};

static const int adau1860_lrclk_rates[] = {
	0, 48000, 96000, 192000, 12000, 24000, 384000, 768000, 8000, 16000,
};

static int adau1860_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct adau18x0 *adau = snd_soc_component_get_drvdata(component);
	int dai_fmt = adau->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	int dai_master = adau->dai_fmt & SND_SOC_DAIFMT_MASTER_MASK;
	int base = dai->driver->base;
	int bclk_target = snd_soc_params_to_bclk(params);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	int i, bclk = 0, lrclk = 0;

	dev_dbg(dai->dev, "%s: DAI id: %d", __func__, dai->id);
	dev_dbg(dai->dev, "lrck is %dHz\n", rate);
	dev_dbg(dai->dev, "bclk is %dHz\n", bclk_target);
	dev_dbg(dai->dev, "width is %d\n", params_width(params));
	dev_dbg(dai->dev, "frame size is %d\n",
		snd_soc_params_to_frame_size(params));
	dev_dbg(dai->dev, "Channels number %d\n", channels);

	if (dai_fmt == SND_SOC_DAIFMT_RIGHT_J) {
		//Do something
	}

	if (dai_master == SND_SOC_DAIFMT_CBM_CFM) {
		for (i = 1; i < ARRAY_SIZE(adau1860_bclk_rates); i++) {
			if (adau1860_bclk_rates[i] == bclk_target) {
				bclk = i;
				break;
			}
		}

		if (i == ARRAY_SIZE(adau1860_bclk_rates)) {
			dev_err(dai->dev, "Unsupported BCLK rate %dHz\n",
				bclk_target);
			return -EINVAL;
		}

		for (i = 1; i < ARRAY_SIZE(adau1860_lrclk_rates); i++) {
			if (adau1860_lrclk_rates[i] == rate) {
				lrclk = i;
				break;
			}
		}

		if (i == ARRAY_SIZE(adau1860_lrclk_rates)) {
			dev_err(dai->dev, "Unsupported sample rate %dHz\n",
				rate);
			return -EINVAL;
		}
	}

	regmap_update_bits(adau->regmap, base + ADAU1860_SPT_CTRL2_OFFS,
			   ADAU1860_SPT_BCLK_SRC_MSK, bclk);
	regmap_update_bits(adau->regmap, base + ADAU1860_SPT_CTRL3_OFFS,
			   ADAU1860_SPT_LRCLK_SRC_MSK, lrclk);
	regmap_write(adau->regmap, ADAU1860_CLK_CTRL(13), 0x90);

	return 0;
}

static int adau1860_set_channel_map(struct snd_soc_dai *dai,
				    unsigned int tx_num, unsigned int *tx_slot,
				    unsigned int rx_num, unsigned int *rx_slot)
{
	dev_dbg(dai->dev,
		"Set Channel Map tx_num: %d, tx_slot[tx_num] : %d, rx_num : %d, rx_slot[rx_num]: %d",
		tx_num, tx_slot[tx_num], rx_num, rx_slot[rx_num]);
	return 0;
}

static int adau1860_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				   unsigned int freq, int dir)
{
	dev_dbg(dai->dev, "Set SYSCLK DAI id: %d clk_id: %d freq : %d dir : %d", dai->id, clk_id,
		freq, dir);
	return 0;
}

static int adau1860_set_component_sysclk(struct snd_soc_component *component,
					 int clk_id, int source, unsigned int freq,
					 int dir)
{
	dev_dbg(component->dev, "Set component SYSCLK clk_id: %d, freq : %d, dir : %d", clk_id, 
		freq, dir);
	return 0;
}

static int adau1860_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_component *component = dai->component;
	int base = dai->driver->base;
	unsigned int reg;
	int ret;

	if (tristate)
		reg = ADAU1860_SPT_TRI_STATE;
	else
		reg = 0;

	ret = snd_soc_component_update_bits(component,
					    base + ADAU1860_SPT_CTRL1_OFFS,
					    ADAU1860_SPT_TRI_STATE, reg);
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int adau1860_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct adau18x0 *adau = snd_soc_component_get_drvdata(component);
	uint8_t lrclk = 0, bclk = 0, ctrl1 = 0;
	int base = dai->driver->base;

	dev_dbg(dai->dev, "%s: Format: %d DAI id: %d", __func__, fmt, dai->id);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl1 |= ADAU1860_SPT_FORMAT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1 |= ADAU1860_SPT_FORMAT_TDM;
		break;
	default:
		dev_err(dai->dev, "Unsupported format on DAI %d\n", dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		dev_dbg(dai->dev, "%s: Codec is frame & bit clk slave",
			__func__);
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		dev_dbg(dai->dev, "%s: Codec is frame & bit clk master",
			__func__);
		break;
	default:
		dev_err(dai->dev, "Unsupported master mode on DAI %d\n",
			dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		bclk = ADAU1860_SPT_BCLK_POL;
		lrclk = ADAU1860_SPT_LRCLK_POL;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		bclk = ADAU1860_SPT_BCLK_POL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		lrclk = ADAU1860_SPT_LRCLK_POL;
		break;
	default:
		dev_err(dai->dev, "Unsupported invert mode on DAI %d\n",
			dai->id);
		return -EINVAL;
	}

	adau->dai_fmt = fmt;

	regmap_update_bits(adau->regmap, base + ADAU1860_SPT_CTRL1_OFFS,
			   ADAU1860_SPT_DATA_FMT_MSK, ctrl1);
	regmap_update_bits(adau->regmap, base + ADAU1860_SPT_CTRL2_OFFS,
			   ADAU1860_SPT_BCLK_POL | ADAU1860_SPT_BCLK_SRC_MSK,
			   bclk);
	regmap_update_bits(adau->regmap, base + ADAU1860_SPT_CTRL3_OFFS,
			   ADAU1860_SPT_LRCLK_POL | ADAU1860_SPT_LRCLK_SRC_MSK,
			   lrclk);

	return 0;
}

static int adau1860_set_dai_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int freq_in, unsigned int freq_out)
{
	dev_dbg(dai->dev, "Set PLL source: %d, freq_in : %d, freq_out : %d",
		source, freq_in, freq_out);
	if (freq_in < 8000000 || freq_in > 27000000)
		return -EINVAL;

	if (freq_out != 24576000 && freq_out != 49152000 &&
	    freq_out != 73728000 && freq_out != 98304000)
		return -EINVAL;

	return 0;
}

static int adau1860_set_dai_tdm_slot(struct snd_soc_dai *dai,
				     unsigned int tx_mask, unsigned int rx_mask,
				     int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct adau18x0 *adau = snd_soc_component_get_drvdata(component);
	int base = dai->driver->base;
	uint8_t ctrl1 = 0;

	dev_dbg(dai->dev,
		"DAI id: %d, tx_mask: %d, rx_mask: %d, slots: %d, slot_width: %d",
		dai->id, tx_mask, rx_mask, slots, slot_width);

	switch (slot_width) {
	case 32:
		break;
	case 24:
		ctrl1 = FIELD_PREP(ADAU1860_SPT_SLOT_WIDTH_MSK,
				   ADAU1860_SPT_SLOT_WIDTH_24);
		break;
	case 16:
		ctrl1 = FIELD_PREP(ADAU1860_SPT_SLOT_WIDTH_MSK,
				   ADAU1860_SPT_SLOT_WIDTH_16);
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(adau->regmap, base + ADAU1860_SPT_CTRL1_OFFS,
			   ADAU1860_SPT_SLOT_WIDTH_MSK, ctrl1);

	return 0;
}

static int adau1860_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s: DAI id: %d", __func__, dai->id);
	return 0;
}

static const struct snd_soc_component_driver adau1860_component_driver = {
	.probe = adau1860_component_probe,
	.resume = adau1860_resume,
	.name = DRV_NAME,
	.set_bias_level = adau1860_set_bias_level,
	.set_sysclk = adau1860_set_component_sysclk,
	.controls = adau1860_controls,
	.num_controls = ARRAY_SIZE(adau1860_controls),
	.dapm_widgets = adau1860_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(adau1860_dapm_widgets),
	.dapm_routes = adau1860_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(adau1860_dapm_routes),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
	.non_legacy_dai_naming = 1,
};

const struct snd_soc_dai_ops adau1860_dai_ops = {
	.hw_params = adau1860_hw_params,
	.set_channel_map = adau1860_set_channel_map,
	.set_sysclk = adau1860_set_dai_sysclk,
	.set_tristate = adau1860_set_tristate,
	.set_fmt = adau1860_set_dai_fmt,
	.set_pll = adau1860_set_dai_pll,
	.set_tdm_slot = adau1860_set_dai_tdm_slot,
	.startup = adau1860_startup,
};

#define ADAU1860_FORMATS                                                       \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_LE |                   \
	 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver adau1860_dai[] = {
	{
		.name = "adau1860-sai0",
		.id = 0,
		.base = ADAU1860_SPT0_CTRL1,
		.playback = {
			.stream_name = "SAI0 Playback",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = ADAU1860_FORMATS,
		},
		.capture = {
			.stream_name = "SAI0 Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = ADAU1860_FORMATS,
		},
		.ops = &adau1860_dai_ops,
	},
	{
		.name = "adau1860-sai1",
		.id = 1,
		.base = ADAU1860_SPT1_CTRL1,
		.playback = {
			.stream_name = "SAI1 Playback",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = ADAU1860_FORMATS,
		},
		.capture = {
			.stream_name = "SAI1 Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = ADAU1860_FORMATS,
		},
		.ops = &adau1860_dai_ops,
	},
};

int adau1860_probe(struct device *dev, struct regmap *regmap,
		   enum adau1860_type type,
		   void (*switch_mode)(struct device *dev))
{
	struct adau18x0 *adau;
	uint16_t val;
	int ret;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	adau = devm_kzalloc(dev, sizeof(*adau), GFP_KERNEL);
	if (!adau)
		return -ENOMEM;

	adau->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(adau->mclk)) {
		if (PTR_ERR(adau->mclk) != -ENOENT)
			return PTR_ERR(adau->mclk);
		/* Clock is optional (for the driver) */
		adau->mclk = NULL;
	} else if (adau->mclk) {
		adau->pll_src = ADAU18X0_PLL_SRC_MCLKIN;
	}

	adau->pd_gpio =
		devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_LOW);
	if (IS_ERR(adau->pd_gpio))
		return PTR_ERR(adau->pd_gpio);

	adau->regmap = regmap;
	adau->switch_mode = switch_mode;
	adau->type = type;

	ret = regmap_bulk_read(adau->regmap, ADAU1860_DEVICE_ID1, &val, 2);
	dev_dbg(dev, "Read Device ID: %x", val);
	if (val != 0x1860) {
		dev_err(dev, "Wrong Device ID: %x", val);
		return 0;
	}

	dev_set_drvdata(dev, adau);

	return devm_snd_soc_register_component(dev, &adau1860_component_driver,
					       adau1860_dai,
					       ARRAY_SIZE(adau1860_dai));
}
EXPORT_SYMBOL_GPL(adau1860_probe);

static const struct regmap_range adau1860_rd_ranges[] = {
	regmap_reg_range(ADAU1860_VENDOR_ID, ADAU1860_MP_MCLKO_RATE),
	regmap_reg_range(ADAU1866_STATUS(1), ADAU1866_STATUS(9)),
};

static const struct regmap_range adau1860_wr_ranges[] = {
	regmap_reg_range(ADAU1860_ADC_DAC_HP_PWR, ADAU1860_MP_MCLKO_RATE),
};

static const struct regmap_range adau1860_no_ranges[] = {
	regmap_reg_range(0, 0x4000bfff),
	regmap_reg_range(0x4000c176, 0x4000c1ff),
	regmap_reg_range(0x4000c201, 0x4000c3ff),
	regmap_reg_range(0x4000c439, 0x4000cc03),
	regmap_reg_range(0x4000cc05, 0x4000cc11),
};

static const struct regmap_access_table adau1860_wr_table = {
	.yes_ranges = adau1860_wr_ranges,
	.n_yes_ranges = ARRAY_SIZE(adau1860_wr_ranges),
	.no_ranges = adau1860_no_ranges,
	.n_no_ranges = ARRAY_SIZE(adau1860_no_ranges),
};

static const struct regmap_access_table adau1860_rd_table = {
	.yes_ranges = adau1860_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(adau1860_rd_ranges),
	.no_ranges = adau1860_no_ranges,
	.n_no_ranges = ARRAY_SIZE(adau1860_no_ranges),
};

static const struct regmap_range adau1860_volatile_ranges[] = {
	regmap_reg_range(ADAU1860_CHIP_PWR, ADAU1860_CLK_CTRL(15)),
};

static const struct regmap_access_table adau1860_volatile_table = {
	.yes_ranges = adau1860_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(adau1860_volatile_ranges),
};

const struct regmap_config adau1860_regmap_config = {
	.val_bits = 8,
	.reg_bits = 32,
	.rd_table = &adau1860_rd_table,
	.wr_table = &adau1860_wr_table,
	.max_register = 0x4000cc12,
	.volatile_table = &adau1860_volatile_table,
	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(adau1860_regmap_config);

MODULE_DESCRIPTION("ASoC ADAU1860 CODEC driver");
MODULE_AUTHOR("Bogdan Togorean <bogdan.togorean@analog.com>");
MODULE_LICENSE("GPL");
