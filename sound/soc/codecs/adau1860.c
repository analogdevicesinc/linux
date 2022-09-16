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

static const struct reg_default adau1860_reg_defaults[] = {
};

static const DECLARE_TLV_DB_MINMAX_MUTE(adau1860_adc_tlv, -7125, 2400);
static const DECLARE_TLV_DB_MINMAX_MUTE(adau1860_dac_tlv, -7125, 2400);
static const DECLARE_TLV_DB_SCALE(adau1860_pga_tlv, 0, 75, 0);

static const unsigned int adau1860_bias_select_extreme_values[] = {
	0, 2, 3,
};

static const char * const adau1860_bias_select_extreme_text[] = {
	"Normal operation", "Enhanced performance", "Extreme Power saving",
};

static const char * const adau1860_bias_select_text[] = {
	"Normal operation", "Enhanced performance",
};

static SOC_ENUM_SINGLE_DECL(adau1860_playback_bias_enum,
		ADAU1860_PB_CTRL, 2, adau1860_bias_select_text);

static const char * const adau1860_adc_hpf_select_text[] = {
	"Off", "1Hz", "4Hz", "8Hz",
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

static const struct snd_kcontrol_new adau1860_differential_mode_controls[] = {
	SOC_SINGLE_TLV("PGA0 Capture Gain", ADAU1860_PGA0_CTRL2,
		0, 0x20, 0, adau1860_pga_tlv),
	SOC_SINGLE_TLV("PGA1 Capture Gain", ADAU1860_PGA1_CTRL2,
		0, 0x20, 0, adau1860_pga_tlv),
	SOC_SINGLE_TLV("PGA2 Capture Gain", ADAU1860_PGA2_CTRL2,
		0, 0x20, 0, adau1860_pga_tlv),
	SOC_ENUM("PGA0 Capture Bias Select", adau1860_pga_bias_enum[0]),
	SOC_ENUM("PGA1 Capture Bias Select", adau1860_pga_bias_enum[1]),
	SOC_ENUM("PGA2 Capture Bias Select", adau1860_pga_bias_enum[2]),
};

static const struct snd_kcontrol_new adau1860_single_mode_controls[] = {
	SOC_SINGLE_TLV("ADC0 Capture Volume", ADAU1860_ADC0_VOL,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("ADC1 Capture Volume", ADAU1860_ADC1_VOL,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("ADC2 Capture Volume", ADAU1860_ADC2_VOL,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_ENUM("ADC0 Bias Select", adau1860_adc_bias_enum[0]),
	SOC_ENUM("ADC1 Bias Select", adau1860_adc_bias_enum[1]),
	SOC_ENUM("ADC2 Bias Select", adau1860_adc_bias_enum[2]),
	SOC_ENUM("ADC0 HPF Select", adau1860_adc_hpf_enum[0]),
	SOC_ENUM("ADC1 HPF Select", adau1860_adc_hpf_enum[1]),
	SOC_ENUM("ADC2 HPF Select", adau1860_adc_hpf_enum[2]),
};

static const struct snd_kcontrol_new adau1860_controls[] = {
	SOC_SINGLE_TLV("DMIC0 Capture Volume", ADAU1860_DMIC_VOL0,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DMIC1 Capture Volume", ADAU1860_DMIC_VOL1,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DMIC2 Capture Volume", ADAU1860_DMIC_VOL2,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DMIC3 Capture Volume", ADAU1860_DMIC_VOL3,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DMIC4 Capture Volume", ADAU1860_DMIC_VOL4,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DMIC5 Capture Volume", ADAU1860_DMIC_VOL5,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DMIC6 Capture Volume", ADAU1860_DMIC_VOL6,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DMIC7 Capture Volume", ADAU1860_DMIC_VOL7,
		0, 0xff, 1, adau1860_adc_tlv),
	SOC_SINGLE_TLV("DAC0 Playback Volume", ADAU1860_DAC_VOL0,
		0, 0xff, 1, adau1860_dac_tlv),
};

static const unsigned int adau1860_spt_mux_val[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 63,
};

static const char * const adau1860_spt_mux_text[] = {
	"FDSP Ch0",
	"FDSP Ch1",
	"FDSP Ch2",
	"FDSP Ch3",
	"FDSP Ch4",
	"FDSP Ch5",
	"FDSP Ch6",
	"FDSP Ch7",
	"FDSP Ch8",
	"FDSP Ch9",
	"FDSP Ch10",
	"FDSP Ch11",
	"FDSP Ch12",
	"FDSP Ch13",
	"FDSP Ch14",
	"FDSP Ch15",
	"TDSP Ch0",
	"TDSP Ch1",
	"TDSP Ch2",
	"TDSP Ch3",
	"TDSP Ch4",
	"TDSP Ch5",
	"TDSP Ch6",
	"TDSP Ch7",
	"TDSP Ch8",
	"TDSP Ch9",
	"TDSP Ch10",
	"TDSP Ch11",
	"TDSP Ch12",
	"TDSP Ch13",
	"TDSP Ch14",
	"TDSP Ch15",
	"OUT ASRC Ch0",
	"OUT ASRC Ch1",
	"OUT ASRC Ch2",
	"OUT ASRC Ch3",
	"ADC0",
	"ADC1",
	"ADC2",
	"DMIC0",
	"DMCI1",
	"DMIC2",
	"DMIC3",
	"FDEC Ch0",
	"FDEC Ch1",
	"FDEC Ch2",
	"FDEC Ch3",
	"FDEC Ch4",
	"FDEC Ch5",
	"FDEC Ch6",
	"FDEC Ch7",
	"EQ0",
	"DMIC4",
	"DMCI5",
	"DMIC6",
	"DMIC7",
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
			adau1860_spt_mux_text,adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT0_ROUTE(7), 0, 0x3f,
			ARRAY_SIZE(adau1860_spt_mux_text),
			adau1860_spt_mux_text,adau1860_spt_mux_val),
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
			adau1860_spt_mux_text,adau1860_spt_mux_val),
	SOC_VALUE_ENUM_SINGLE(ADAU1860_SPT1_ROUTE(7), 0, 0x3f,
			ARRAY_SIZE(adau1860_spt_mux_text),
			adau1860_spt_mux_text,adau1860_spt_mux_val),
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
	SOC_ENUM("SPT0 Out Slot 0", adau1860_spt0_mux_enums[0]),
	SOC_ENUM("SPT0 Out Slot 1", adau1860_spt0_mux_enums[1]),
	SOC_ENUM("SPT0 Out Slot 2", adau1860_spt0_mux_enums[2]),
	SOC_ENUM("SPT0 Out Slot 3", adau1860_spt0_mux_enums[3]),
	SOC_ENUM("SPT0 Out Slot 4", adau1860_spt0_mux_enums[4]),
	SOC_ENUM("SPT0 Out Slot 5", adau1860_spt0_mux_enums[5]),
	SOC_ENUM("SPT0 Out Slot 6", adau1860_spt0_mux_enums[6]),
	SOC_ENUM("SPT0 Out Slot 7", adau1860_spt0_mux_enums[7]),
	SOC_ENUM("SPT0 Out Slot 8", adau1860_spt0_mux_enums[8]),
	SOC_ENUM("SPT0 Out Slot 9", adau1860_spt0_mux_enums[9]),
	SOC_ENUM("SPT0 Out Slot 10", adau1860_spt0_mux_enums[10]),
	SOC_ENUM("SPT0 Out Slot 11", adau1860_spt0_mux_enums[11]),
	SOC_ENUM("SPT0 Out Slot 12", adau1860_spt0_mux_enums[12]),
	SOC_ENUM("SPT0 Out Slot 13", adau1860_spt0_mux_enums[13]),
	SOC_ENUM("SPT0 Out Slot 14", adau1860_spt0_mux_enums[14]),
	SOC_ENUM("SPT0 Out Slot 15", adau1860_spt0_mux_enums[15]),
};

static const struct snd_kcontrol_new adau1860_spt1_out_controls[] = {
	SOC_ENUM("SPT1 Out Slot 0", adau1860_spt0_mux_enums[0]),
	SOC_ENUM("SPT1 Out Slot 1", adau1860_spt0_mux_enums[1]),
	SOC_ENUM("SPT1 Out Slot 2", adau1860_spt0_mux_enums[2]),
	SOC_ENUM("SPT1 Out Slot 3", adau1860_spt0_mux_enums[3]),
	SOC_ENUM("SPT1 Out Slot 4", adau1860_spt0_mux_enums[4]),
	SOC_ENUM("SPT1 Out Slot 5", adau1860_spt0_mux_enums[5]),
	SOC_ENUM("SPT1 Out Slot 6", adau1860_spt0_mux_enums[6]),
	SOC_ENUM("SPT1 Out Slot 7", adau1860_spt0_mux_enums[7]),
	SOC_ENUM("SPT1 Out Slot 8", adau1860_spt0_mux_enums[8]),
	SOC_ENUM("SPT1 Out Slot 9", adau1860_spt0_mux_enums[9]),
	SOC_ENUM("SPT1 Out Slot 10", adau1860_spt0_mux_enums[10]),
	SOC_ENUM("SPT1 Out Slot 11", adau1860_spt0_mux_enums[11]),
	SOC_ENUM("SPT1 Out Slot 12", adau1860_spt0_mux_enums[12]),
	SOC_ENUM("SPT1 Out Slot 13", adau1860_spt0_mux_enums[13]),
	SOC_ENUM("SPT1 Out Slot 14", adau1860_spt0_mux_enums[14]),
	SOC_ENUM("SPT1 Out Slot 15", adau1860_spt0_mux_enums[15]),
};

static const char * const adau1860_dac_mux_text[] = {
	"SPT0 Ch0",
	"SPT0 Ch1",
	"SPT0 Ch2",
	"SPT0 Ch3",
	"SPT0 Ch4",
	"SPT0 Ch5",
	"SPT0 Ch6",
	"SPT0 Ch7",
	"SPT0 Ch8",
	"SPT0 Ch9",
	"SPT0 Ch10",
	"SPT0 Ch11",
	"SPT0 Ch12",
	"SPT0 Ch13",
	"SPT0 Ch14",
	"SPT0 Ch15",
	"SPT1 Ch0",
	"SPT1 Ch1",
	"SPT1 Ch2",
	"SPT1 Ch3",
	"SPT1 Ch4",
	"SPT1 Ch5",
	"SPT1 Ch6",
	"SPT1 Ch7",
	"SPT1 Ch8",
	"SPT1 Ch9",
	"SPT1 Ch10",
	"SPT1 Ch11",
	"SPT1 Ch12",
	"SPT1 Ch13",
	"SPT1 Ch14",
	"SPT1 Ch15",
	"FDSP Ch0",
	"FDSP Ch1",
	"FDSP Ch2",
	"FDSP Ch3",
	"FDSP Ch4",
	"FDSP Ch5",
	"FDSP Ch6",
	"FDSP Ch7",
	"FDSP Ch8",
	"FDSP Ch9",
	"FDSP Ch10",
	"FDSP Ch11",
	"FDSP Ch12",
	"FDSP Ch13",
	"FDSP Ch14",
	"FDSP Ch15",
	"TDSP Ch0",
	"TDSP Ch1",
	"TDSP Ch2",
	"TDSP Ch3",
	"TDSP Ch4",
	"TDSP Ch5",
	"TDSP Ch6",
	"TDSP Ch7",
	"TDSP Ch8",
	"TDSP Ch9",
	"TDSP Ch10",
	"TDSP Ch11",
	"TDSP Ch12",
	"TDSP Ch13",
	"TDSP Ch14",
	"TDSP Ch15",
	"IN ASRC Ch0",
	"IN ASRC Ch1",
	"IN ASRC Ch2",
	"IN ASRC Ch3",
	"ADC0",
	"ADC1",
	"ADC2",
	"DMIC0",
	"DMCI1",
	"DMIC2",
	"DMIC3",
	"EQ0",
	"FINT Ch0",
	"FINT Ch1",
	"FINT Ch2",
	"FINT Ch3",
	"FINT Ch4",
	"FINT Ch5",
	"FINT Ch6",
	"FINT Ch7",
	"DMIC4",
	"DMCI5",
	"DMIC6",
	"DMIC7",
};

static SOC_ENUM_SINGLE_DECL(adau1860_dac_mux_enum, ADAU1860_DAC_ROUTE0,
			    0, adau1860_dac_mux_text);

static const struct snd_kcontrol_new adau1860_dac_mux_control =
	SOC_DAPM_ENUM("DAC Source", adau1860_dac_mux_enum);

static const char * const adau1860_input_mux_text[] = {
	"ADC", "PGA",
};

static SOC_ENUM_SINGLE_DECL(adau1860_input_mux_enum,
	ADAU1860_ADC_CTRL7, 2, adau1860_input_mux_text);

static const struct snd_kcontrol_new adau1860_input_mux_control =
	SOC_DAPM_ENUM("Input Select", adau1860_input_mux_enum);

static const struct snd_soc_dapm_widget adau1860_dapm_widgets[] = {
	SND_SOC_DAPM_REGULATOR_SUPPLY("HPVDD", 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("IOVDD", 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("DVDD", 0, 0),
	SND_SOC_DAPM_REGULATOR_SUPPLY("AVDD", 0, 0),

	SND_SOC_DAPM_SUPPLY("Headphone", ADAU1860_HPLDO_CTRL,
		0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("PLL", ADAU1860_PLL_PGA_PWR, 0, 0, NULL,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("XTAL", ADAU1860_PLL_PGA_PWR, 1, 0, NULL,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("SYSCLK", SND_SOC_NOPM, 0, 0, NULL,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC_CLK", ADAU1860_SAI_CLK_PWR, 4, 0, NULL,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DMIC_CLK1", ADAU1860_SAI_CLK_PWR, 5, 0, NULL,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("DMIC Decimator Mux", SND_SOC_NOPM, 0, 0,
		&adau1860_input_mux_control),
	SND_SOC_DAPM_MUX("ADC Decimator Mux", SND_SOC_NOPM, 0, 0,
		&adau1860_input_mux_control),
	SND_SOC_DAPM_MUX("DAC Mux", SND_SOC_NOPM, 0, 0,
			 &adau1860_dac_mux_control),

	SND_SOC_DAPM_AIF_OUT("SPT0_OUT", NULL, 0, ADAU1860_SAI_CLK_PWR, 1, 0),
	SND_SOC_DAPM_AIF_IN("SPT0_IN", NULL, 0, ADAU1860_SAI_CLK_PWR, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SPT1_OUT", NULL, 0, ADAU1860_SAI_CLK_PWR, 3, 0),
	SND_SOC_DAPM_AIF_IN("SPT1_IN", NULL, 0, ADAU1860_SAI_CLK_PWR, 2, 0),

	SND_SOC_DAPM_INPUT("AINP0"),
	SND_SOC_DAPM_INPUT("AINN0"),
	SND_SOC_DAPM_INPUT("AINP1"),
	SND_SOC_DAPM_INPUT("AINN1"),
	SND_SOC_DAPM_INPUT("AINP2"),
	SND_SOC_DAPM_INPUT("AINN2"),

	SND_SOC_DAPM_INPUT("DMIC01"),
	SND_SOC_DAPM_INPUT("DMIC23"),

	SND_SOC_DAPM_ADC("ADC0", "Capture", ADAU1860_ADC_DAC_HP_PWR, 0, 0),
	SND_SOC_DAPM_ADC("ADC1", "Capture", ADAU1860_ADC_DAC_HP_PWR, 1, 0),
	SND_SOC_DAPM_ADC("ADC2", "Capture", ADAU1860_ADC_DAC_HP_PWR, 2, 0),
	SND_SOC_DAPM_ADC("DMIC0", "Capture", ADAU1860_DMIC_PWR, 0, 0),
	SND_SOC_DAPM_ADC("DMIC1", "Capture", ADAU1860_DMIC_PWR, 1, 0),
	SND_SOC_DAPM_ADC("DMIC2", "Capture", ADAU1860_DMIC_PWR, 2, 0),
	SND_SOC_DAPM_ADC("DMIC3", "Capture", ADAU1860_DMIC_PWR, 3, 0),
	SND_SOC_DAPM_ADC("DMIC4", "Capture", ADAU1860_DMIC_PWR, 4, 0),
	SND_SOC_DAPM_ADC("DMIC5", "Capture", ADAU1860_DMIC_PWR, 5, 0),
	SND_SOC_DAPM_ADC("DMIC6", "Capture", ADAU1860_DMIC_PWR, 6, 0),
	SND_SOC_DAPM_ADC("DMIC7", "Capture", ADAU1860_DMIC_PWR, 7, 0),
	SND_SOC_DAPM_DAC("DAC", "Playback", ADAU1860_ADC_DAC_HP_PWR, 4, 0),

	SND_SOC_DAPM_OUTPUT("HPOUTP"),
	SND_SOC_DAPM_OUTPUT("HPOUTN"),
};

static const struct snd_soc_dapm_route adau1860_dapm_routes[] = {
	/* Clock Paths */
	{ "DMIC_CLK", NULL, "SYSCLK" },

	/* DMIC Paths */
	{ "DMIC Decimator Mux", NULL, "DMIC0" },
	{ "DMIC Decimator Mux", NULL, "DMIC1" },
	{ "DMIC Decimator Mux", NULL, "DMIC2" },
	{ "DMIC Decimator Mux", NULL, "DMIC3" },
	{ "DMIC Decimator Mux", NULL, "DMIC4" },
	{ "DMIC Decimator Mux", NULL, "DMIC5" },
	{ "DMIC Decimator Mux", NULL, "DMIC6" },
	{ "DMIC Decimator Mux", NULL, "DMIC7" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },
	{ "DMIC0", NULL, "DMIC_CLK" },

	/* ADC Input Paths */
	{ "ADC0", NULL, "AINP0" },
	{ "ADC0", NULL, "AINN0" },
	{ "ADC1", NULL, "AINP1" },
	{ "ADC1", NULL, "AINN1" },
	{ "ADC2", NULL, "AINP2" },
	{ "ADC2", NULL, "AINN2" },
	{ "ADC Decimator Mux", NULL, "ADC0" },
	{ "ADC Decimator Mux", NULL, "ADC1" },
	{ "ADC Decimator Mux", NULL, "ADC2" },

	/* Digital Paths */
	{ "DAC Mux", NULL, "SPT0_IN" },
	{ "DAC Mux", NULL, "SPT1_IN" },
	{ "SPT0_OUT", NULL, "DMIC Decimator Mux" },
	{ "SPT0_OUT", NULL, "ADC Decimator Mux" },
	{ "SPT1_OUT", NULL, "DMIC Decimator Mux" },
	{ "SPT1_OUT", NULL, "ADC Decimator Mux" },
	{ "SPT0_IN", NULL, "SAI0 Playback" },
	{ "SPT1_IN", NULL, "SAI1 Playback" },
	{ "SAI0 Capture", NULL, "SPT0_OUT" },
	{ "SAI1 Capture", NULL, "SPT0_OUT" },

	/* Playback Paths */
	{ "DAC", NULL, "DAC Mux" },
	{ "HPOUTP", NULL, "DAC" },
	{ "HPOUTN", NULL, "DAC" },
};

static void adau1860_set_power(struct adau18x0 *adau1860, bool enable)
{
	if (adau1860->enabled == enable)
		return;

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
		}
		clk_disable_unprepare(adau1860->mclk);
		regcache_cache_only(adau1860->regmap, true);
	}

	adau1860->enabled = enable;
}

static int adau1860_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	struct adau18x0 *adau1860 = snd_soc_component_get_drvdata(component);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(component->dev, "%s: SND_SOC_BIAS_STANDBY", __func__);
		adau1860_set_power(adau1860, true);
		break;
	case SND_SOC_BIAS_OFF:
		dev_dbg(component->dev, "%s: SND_SOC_BIAS_OFF", __func__);
		adau1860_set_power(adau1860, false);
		break;
	}
	return 0;
}

static int adau1860_component_probe(struct snd_soc_component *component)
{
	struct adau1860_pdata *pdata = component->dev->platform_data;
	int ret;

	if (pdata && pdata->input_differential) {
		ret = snd_soc_add_component_controls(component,
			adau1860_differential_mode_controls,
			ARRAY_SIZE(adau1860_differential_mode_controls));
		if (ret)
			return ret;
	} else {
		ret = snd_soc_add_component_controls(component,
			adau1860_single_mode_controls,
			ARRAY_SIZE(adau1860_single_mode_controls));
		if (ret)
			return ret;
	}

	ret = snd_soc_add_component_controls(component,
		adau1860_spt1_out_controls, ARRAY_SIZE(adau1860_spt1_out_controls));

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

static int adau1860_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s", __func__);
	return 0;
}

static int adau1860_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	dev_dbg(dai->dev, "Set SYSCLK clk_id: %d, freq : %d, dir : %d", clk_id, freq, dir);
	return 0;
}

static int adau1860_set_dai_fmt(struct snd_soc_dai *dai,
		unsigned int fmt)
{
	int lrclk, bclk;

	lrclk = 0;
	bclk = 0;

	dev_dbg(dai->dev, "%s", __func__);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		break;
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	default:
		dev_err(dai->dev, "Unsupported format on DAI %d\n",
			dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
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
		break;
	case SND_SOC_DAIFMT_IB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	default:
		dev_err(dai->dev, "Unsupported invert mode on DAI %d\n",
			dai->id);
		return -EINVAL;
	}

	return 0;
}

static int adau1860_set_dai_pll(struct snd_soc_dai *dai, int pll_id,
	int source, unsigned int freq_in, unsigned int freq_out)
{
	dev_dbg(dai->dev, "Set PLL source: %d, freq_in : %d, freq_out : %d", source, freq_in, freq_out);
	if (freq_in < 8000000 || freq_in > 27000000)
		return -EINVAL;

	if (freq_out != 24576000 && freq_out != 49152000 &&
	    freq_out != 73728000 && freq_out != 98304000)
		return -EINVAL;
	
	return 0;
}

static int adau1860_set_dai_tdm_slot(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	dev_dbg(dai->dev, "DAI id: %d, tx_mask: %d, rx_mask: %d, slots: %d, slot_width: %d", 
		dai->id, tx_mask, rx_mask, slots, slot_width);
	return 0;
}

static int adau1860_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s", __func__);
	return 0;
}

static const struct snd_soc_component_driver adau1860_component_driver = {
	.probe			= adau1860_component_probe,
	.resume			= adau1860_resume,
	.name			= DRV_NAME,
	.set_bias_level		= adau1860_set_bias_level,
	.controls		= adau1860_controls,
	.num_controls		= ARRAY_SIZE(adau1860_controls),
	.dapm_widgets		= adau1860_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(adau1860_dapm_widgets),
	.dapm_routes		= adau1860_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(adau1860_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

const struct snd_soc_dai_ops adau1860_dai_ops = {
	.hw_params	= adau1860_hw_params,
	.set_sysclk	= adau1860_set_dai_sysclk,
	.set_fmt	= adau1860_set_dai_fmt,
	.set_pll	= adau1860_set_dai_pll,
	.set_tdm_slot	= adau1860_set_dai_tdm_slot,
	.startup	= adau1860_startup,
};

#define ADAU1860_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
	SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver adau1860_dai[] = {
	{
		.name = "adau1860-sai0",
		.id = 0,
		.playback = {
			.stream_name = "SAI0 Playback",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = ADAU1860_FORMATS,
		},
		.capture = {
			.stream_name = "SAI0 Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = ADAU1860_FORMATS,
		},
		.ops = &adau1860_dai_ops,
	},
	{
		.name = "adau1860-sai1",
		.id = 1,
		.playback = {
			.stream_name = "SAI1 Playback",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = ADAU1860_FORMATS,
		},
		.capture = {
			.stream_name = "SAI1 Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = ADAU1860_FORMATS,
		},
		.ops = &adau1860_dai_ops,
	},
};

int adau1860_probe(struct device *dev, struct regmap *regmap,
	enum adau1860_type type, void (*switch_mode)(struct device *dev))
{
	struct adau18x0 *adau;
	unsigned int val;
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
		adau->clk_src = ADAU18X0_CLK_SRC_EXT_CLK;
	}

	adau->pd_gpio = devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_LOW);
	if (IS_ERR(adau->pd_gpio))
		return PTR_ERR(adau->pd_gpio);

	adau->regmap = regmap;
	adau->switch_mode = switch_mode;
	adau->type = type;

	ret = regmap_bulk_read(adau->regmap, ADAU1860_VENDOR_ID, &val, 4);
	dev_dbg(dev, "Read Device ID: %x", val);

	dev_set_drvdata(dev, adau);

	return devm_snd_soc_register_component(dev, &adau1860_component_driver,
					       adau1860_dai, ARRAY_SIZE(adau1860_dai));
}
EXPORT_SYMBOL_GPL(adau1860_probe);

static const struct regmap_range adau1860_rw_ranges[] = {
	regmap_reg_range(0x4000c000, 0x4000cc12),
};

static const struct regmap_range adau1860_no_ranges[] = {
	regmap_reg_range(0, 0x4000bfff),
	regmap_reg_range(0x4000c176, 0x4000c1ff),
	regmap_reg_range(0x4000c201, 0x4000c3ff),
	regmap_reg_range(0x4000c439, 0x4000cc03),
	regmap_reg_range(0x4000cc05, 0x4000cc11),
};

static const struct regmap_access_table adau1860_rw_table = {
	.yes_ranges = adau1860_rw_ranges,
	.n_yes_ranges = ARRAY_SIZE(adau1860_rw_ranges),
	.no_ranges = adau1860_no_ranges,
	.n_no_ranges = ARRAY_SIZE(adau1860_no_ranges),
};

const struct regmap_config adau1860_regmap_config = {
	.val_bits = 8,
	.reg_bits = 32,
	.rd_table = &adau1860_rw_table,
	.wr_table = &adau1860_rw_table,
	.max_register = 0x4000cc12,
	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(adau1860_regmap_config);

MODULE_DESCRIPTION("ASoC ADAU1860 CODEC driver");
MODULE_AUTHOR("Bogdan Togorean <bogdan.togorean@analog.com>");
MODULE_LICENSE("GPL");
