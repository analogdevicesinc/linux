/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ALSA mixer/Kcontrol definitions common to HiFiBerry ADCs
 *
 * used by	DAC+ADC Pro (hifiberry_dacplusadcpro.c),
 *		ADC (hifiberry_adc.c)
 *
 * Author:	Joerg Schambacher <joerg@hifiberry.com>
 *		Copyright 2024
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

static const unsigned int pcm186x_adc_input_channel_sel_value[] = {
	0x00, 0x01, 0x02, 0x03, 0x10
};

static const char * const pcm186x_adcl_input_channel_sel_text[] = {
	"No Select",
	"VINL1[SE]",					/* Default for ADCL */
	"VINL2[SE]",
	"VINL2[SE] + VINL1[SE]",
	"{VIN1P, VIN1M}[DIFF]"
};

static const char * const pcm186x_adcr_input_channel_sel_text[] = {
	"No Select",
	"VINR1[SE]",					/* Default for ADCR */
	"VINR2[SE]",
	"VINR2[SE] + VINR1[SE]",
	"{VIN2P, VIN2M}[DIFF]"
};

static const struct soc_enum pcm186x_adc_input_channel_sel[] = {
	SOC_VALUE_ENUM_SINGLE(PCM186X_ADC1_INPUT_SEL_L, 0,
			      PCM186X_ADC_INPUT_SEL_MASK,
			      ARRAY_SIZE(pcm186x_adcl_input_channel_sel_text),
			      pcm186x_adcl_input_channel_sel_text,
			      pcm186x_adc_input_channel_sel_value),
	SOC_VALUE_ENUM_SINGLE(PCM186X_ADC1_INPUT_SEL_R, 0,
			      PCM186X_ADC_INPUT_SEL_MASK,
			      ARRAY_SIZE(pcm186x_adcr_input_channel_sel_text),
			      pcm186x_adcr_input_channel_sel_text,
			      pcm186x_adc_input_channel_sel_value),
};

static const unsigned int pcm186x_mic_bias_sel_value[] = {
	0x00, 0x01, 0x11
};

static const char * const pcm186x_mic_bias_sel_text[] = {
	"Mic Bias off",
	"Mic Bias on",
	"Mic Bias with Bypass Resistor"
};

static const struct soc_enum pcm186x_mic_bias_sel[] = {
	SOC_VALUE_ENUM_SINGLE(PCM186X_MIC_BIAS_CTRL, 0,
			      GENMASK(4, 0),
			      ARRAY_SIZE(pcm186x_mic_bias_sel_text),
			      pcm186x_mic_bias_sel_text,
			      pcm186x_mic_bias_sel_value),
};

static const unsigned int pcm186x_gain_sel_value[] = {
	0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
	0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50
};

static const char * const pcm186x_gain_sel_text[] = {
	"-12.0dB", "-11.5dB", "-11.0dB", "-10.5dB", "-10.0dB", "-9.5dB",
	"-9.0dB", "-8.5dB", "-8.0dB", "-7.5dB", "-7.0dB", "-6.5dB",
	"-6.0dB", "-5.5dB", "-5.0dB", "-4.5dB", "-4.0dB", "-3.5dB",
	"-3.0dB", "-2.5dB", "-2.0dB", "-1.5dB", "-1.0dB", "-0.5dB",
	"0.0dB", "0.5dB", "1.0dB", "1.5dB", "2.0dB", "2.5dB",
	"3.0dB", "3.5dB", "4.0dB", "4.5dB", "5.0dB", "5.5dB",
	"6.0dB", "6.5dB", "7.0dB", "7.5dB", "8.0dB", "8.5dB",
	"9.0dB", "9.5dB", "10.0dB", "10.5dB", "11.0dB", "11.5dB",
	"12.0dB", "12.5dB", "13.0dB", "13.5dB", "14.0dB", "14.5dB",
	"15.0dB", "15.5dB", "16.0dB", "16.5dB", "17.0dB", "17.5dB",
	"18.0dB", "18.5dB", "19.0dB", "19.5dB", "20.0dB", "20.5dB",
	"21.0dB", "21.5dB", "22.0dB", "22.5dB", "23.0dB", "23.5dB",
	"24.0dB", "24.5dB", "25.0dB", "25.5dB", "26.0dB", "26.5dB",
	"27.0dB", "27.5dB", "28.0dB", "28.5dB", "29.0dB", "29.5dB",
	"30.0dB", "30.5dB", "31.0dB", "31.5dB", "32.0dB", "32.5dB",
	"33.0dB", "33.5dB", "34.0dB", "34.5dB", "35.0dB", "35.5dB",
	"36.0dB", "36.5dB", "37.0dB", "37.5dB", "38.0dB", "38.5dB",
	"39.0dB", "39.5dB", "40.0dB"};

static const struct soc_enum pcm186x_gain_sel[] = {
	SOC_VALUE_ENUM_SINGLE(PCM186X_PGA_VAL_CH1_L, 0,
			      0xff,
			      ARRAY_SIZE(pcm186x_gain_sel_text),
			      pcm186x_gain_sel_text,
			      pcm186x_gain_sel_value),
	SOC_VALUE_ENUM_SINGLE(PCM186X_PGA_VAL_CH1_R, 0,
			      0xff,
			      ARRAY_SIZE(pcm186x_gain_sel_text),
			      pcm186x_gain_sel_text,
			      pcm186x_gain_sel_value),
};

static const struct snd_kcontrol_new pcm1863_snd_controls_card[] = {
	SOC_ENUM("ADC Left Input", pcm186x_adc_input_channel_sel[0]),
	SOC_ENUM("ADC Right Input", pcm186x_adc_input_channel_sel[1]),
	SOC_ENUM("ADC Mic Bias", pcm186x_mic_bias_sel),
	SOC_ENUM("PGA Gain Left", pcm186x_gain_sel[0]),
	SOC_ENUM("PGA Gain Right", pcm186x_gain_sel[1]),
};
