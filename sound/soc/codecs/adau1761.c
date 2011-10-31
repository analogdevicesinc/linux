/*
 * Driver for ADAU1761/ADAU1461/ADAU1761/ADAU1961 codec
 *
 * Copyright 2011-2013 Analog Devices Inc.
 * Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/platform_data/adau17x1.h>

#include "adau17x1.h"

#define ADAU1761_DIGMIC_JACKDETECT	0x4008
#define ADAU1761_REC_MIXER_LEFT0	0x400a
#define ADAU1761_REC_MIXER_LEFT1	0x400b
#define ADAU1761_REC_MIXER_RIGHT0	0x400c
#define ADAU1761_REC_MIXER_RIGHT1	0x400d
#define ADAU1761_LEFT_DIFF_INPUT_VOL	0x400e
#define ADAU1761_RIGHT_DIFF_INPUT_VOL	0x400f
#define ADAU1761_PLAY_LR_MIXER_LEFT	0x4020
#define ADAU1761_PLAY_MIXER_LEFT0	0x401c
#define ADAU1761_PLAY_MIXER_LEFT1	0x401d
#define ADAU1761_PLAY_MIXER_RIGHT0	0x401e
#define ADAU1761_PLAY_MIXER_RIGHT1	0x401f
#define ADAU1761_PLAY_LR_MIXER_RIGHT	0x4021
#define ADAU1761_PLAY_MIXER_MONO	0x4022
#define ADAU1761_PLAY_HP_LEFT_VOL	0x4023
#define ADAU1761_PLAY_HP_RIGHT_VOL	0x4024
#define ADAU1761_PLAY_LINE_LEFT_VOL	0x4025
#define ADAU1761_PLAY_LINE_RIGHT_VOL	0x4026
#define ADAU1761_PLAY_MONO_OUTPUT_VOL	0x4027
#define ADAU1761_POP_CLICK_SUPPRESS	0x4028
#define ADAU1761_JACK_DETECT_PIN	0x4031
#define ADAU1761_DEJITTER		0x4036
#define ADAU1761_CLK_ENABLE0		0x40f9
#define ADAU1761_CLK_ENABLE1		0x40fa

#define ADAU1761_DIGMIC_JACKDETECT_ACTIVE_LOW	BIT(0)
#define ADAU1761_DIGMIC_JACKDETECT_DIGMIC	BIT(5)

#define ADAU1761_DIFF_INPUT_VOL_LDEN		0x01

#define ADAU1761_FIRMWARE "adau1761.bin"

static const struct reg_default adau1761_reg_defaults[] = {
	{ ADAU1761_DEJITTER,			0x03 },
	{ ADAU1761_DIGMIC_JACKDETECT,		0x00 },
	{ ADAU1761_REC_MIXER_LEFT0,		0x00 },
	{ ADAU1761_REC_MIXER_LEFT1,		0x00 },
	{ ADAU1761_REC_MIXER_RIGHT0,		0x00 },
	{ ADAU1761_REC_MIXER_RIGHT1,		0x00 },
	{ ADAU1761_LEFT_DIFF_INPUT_VOL,		0x00 },
	{ ADAU1761_RIGHT_DIFF_INPUT_VOL,	0x00 },
	{ ADAU1761_PLAY_LR_MIXER_LEFT,		0x00 },
	{ ADAU1761_PLAY_MIXER_LEFT0,		0x00 },
	{ ADAU1761_PLAY_MIXER_LEFT1,		0x00 },
	{ ADAU1761_PLAY_MIXER_RIGHT0,		0x00 },
	{ ADAU1761_PLAY_MIXER_RIGHT1,		0x00 },
	{ ADAU1761_PLAY_LR_MIXER_RIGHT,		0x00 },
	{ ADAU1761_PLAY_MIXER_MONO,		0x00 },
	{ ADAU1761_PLAY_HP_LEFT_VOL,		0x00 },
	{ ADAU1761_PLAY_HP_RIGHT_VOL,		0x00 },
	{ ADAU1761_PLAY_LINE_LEFT_VOL,		0x00 },
	{ ADAU1761_PLAY_LINE_RIGHT_VOL,		0x00 },
	{ ADAU1761_PLAY_MONO_OUTPUT_VOL,	0x00 },
	{ ADAU1761_POP_CLICK_SUPPRESS,		0x00 },
	{ ADAU1761_JACK_DETECT_PIN,		0x00 },
	{ ADAU1761_CLK_ENABLE0,			0x00 },
	{ ADAU1761_CLK_ENABLE1,			0x00 },
	{ ADAU17X1_CLOCK_CONTROL,		0x00 },
	{ ADAU17X1_PLL_CONTROL,			0x00 },
	{ ADAU17X1_REC_POWER_MGMT,		0x00 },
	{ ADAU17X1_MICBIAS,			0x00 },
	{ ADAU17X1_SERIAL_PORT0,		0x00 },
	{ ADAU17X1_SERIAL_PORT1,		0x00 },
	{ ADAU17X1_CONVERTER0,			0x00 },
	{ ADAU17X1_CONVERTER1,			0x00 },
	{ ADAU17X1_LEFT_INPUT_DIGITAL_VOL,	0x00 },
	{ ADAU17X1_RIGHT_INPUT_DIGITAL_VOL,	0x00 },
	{ ADAU17X1_ADC_CONTROL,			0x00 },
	{ ADAU17X1_PLAY_POWER_MGMT,		0x00 },
	{ ADAU17X1_DAC_CONTROL0,		0x00 },
	{ ADAU17X1_DAC_CONTROL1,		0x00 },
	{ ADAU17X1_DAC_CONTROL2,		0x00 },
	{ ADAU17X1_SERIAL_PORT_PAD,		0x00 },
	{ ADAU17X1_CONTROL_PORT_PAD0,		0x00 },
	{ ADAU17X1_CONTROL_PORT_PAD1,		0x00 },
	{ ADAU17X1_DSP_SAMPLING_RATE,		0x01 },
	{ ADAU17X1_SERIAL_INPUT_ROUTE,		0x00 },
	{ ADAU17X1_SERIAL_OUTPUT_ROUTE,		0x00 },
	{ ADAU17X1_DSP_ENABLE,			0x00 },
	{ ADAU17X1_DSP_RUN,			0x00 },
	{ ADAU17X1_SERIAL_SAMPLING_RATE,	0x00 },
};

static const DECLARE_TLV_DB_SCALE(adau1761_sing_in_tlv, -1500, 300, 1);
static const DECLARE_TLV_DB_SCALE(adau1761_diff_in_tlv, -1200, 75, 0);
static const DECLARE_TLV_DB_SCALE(adau1761_out_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(adau1761_sidetone_tlv, -1800, 300, 1);
static const DECLARE_TLV_DB_SCALE(adau1761_boost_tlv, -600, 600, 1);
static const DECLARE_TLV_DB_SCALE(adau1761_pga_boost_tlv, -2000, 2000, 1);

static const unsigned int adau1761_bias_select_values[] = {
	0, 2, 3,
};

static const char * const adau1761_bias_select_text[] = {
	"Normal operation", "Enhanced performance", "Power saving",
};

static const char * const adau1761_bias_select_extreme_text[] = {
	"Normal operation", "Extreme power saving", "Enhanced performance",
	"Power saving",
};

static const SOC_ENUM_SINGLE_DECL(adau1761_adc_bias_enum,
		ADAU17X1_REC_POWER_MGMT, 3, adau1761_bias_select_extreme_text);
static const SOC_ENUM_SINGLE_DECL(adau1761_hp_bias_enum,
		ADAU17X1_PLAY_POWER_MGMT, 6, adau1761_bias_select_extreme_text);
static const SOC_ENUM_SINGLE_DECL(adau1761_dac_bias_enum,
		ADAU17X1_PLAY_POWER_MGMT, 4, adau1761_bias_select_extreme_text);
static const SOC_VALUE_ENUM_SINGLE_DECL(adau1761_playback_bias_enum,
		ADAU17X1_PLAY_POWER_MGMT, 2, 0x3, adau1761_bias_select_text,
		adau1761_bias_select_values);
static const SOC_VALUE_ENUM_SINGLE_DECL(adau1761_capture_bias_enum,
		ADAU17X1_REC_POWER_MGMT, 1, 0x3, adau1761_bias_select_text,
		adau1761_bias_select_values);

static const struct snd_kcontrol_new adau1761_jack_detect_controls[] = {
	SOC_SINGLE("Jack Detect Switch", 4, 1, 0, ADAU1761_DIGMIC_JACKDETECT),
};

static const struct snd_kcontrol_new adau1761_differential_mode_controls[] = {
	SOC_DOUBLE_R_TLV("Capture Volume", ADAU1761_LEFT_DIFF_INPUT_VOL,
		ADAU1761_RIGHT_DIFF_INPUT_VOL, 2, 0x3f, 0,
		adau1761_diff_in_tlv),
	SOC_DOUBLE_R("Capture Switch", ADAU1761_LEFT_DIFF_INPUT_VOL,
		ADAU1761_RIGHT_DIFF_INPUT_VOL, 1, 1, 0),

	SOC_DOUBLE_R_TLV("PGA Boost Capture Volume", ADAU1761_REC_MIXER_LEFT1,
		ADAU1761_REC_MIXER_RIGHT1, 3, 2, 0, adau1761_pga_boost_tlv),
};

static const struct snd_kcontrol_new adau1761_single_mode_controls[] = {
	SOC_SINGLE_TLV("Input 1 Capture Volume", ADAU1761_REC_MIXER_LEFT0,
		4, 7, 0, adau1761_sing_in_tlv),
	SOC_SINGLE_TLV("Input 2 Capture Volume", ADAU1761_REC_MIXER_LEFT0,
		1, 7, 0, adau1761_sing_in_tlv),
	SOC_SINGLE_TLV("Input 3 Capture Volume", ADAU1761_REC_MIXER_RIGHT0,
		4, 7, 0, adau1761_sing_in_tlv),
	SOC_SINGLE_TLV("Input 4 Capture Volume", ADAU1761_REC_MIXER_RIGHT0,
		1, 7, 0, adau1761_sing_in_tlv),
};

static const struct snd_kcontrol_new adau1761_controls[] = {
	SOC_DOUBLE_R_TLV("Aux Capture Volume", ADAU1761_REC_MIXER_LEFT1,
		ADAU1761_REC_MIXER_RIGHT1, 0, 7, 0, adau1761_sing_in_tlv),

	SOC_DOUBLE_R_TLV("Headphone Playback Volume", ADAU1761_PLAY_HP_LEFT_VOL,
		ADAU1761_PLAY_HP_RIGHT_VOL, 2, 0x3f, 0, adau1761_out_tlv),
	SOC_DOUBLE_R("Headphone Playback Switch", ADAU1761_PLAY_HP_LEFT_VOL,
		ADAU1761_PLAY_HP_RIGHT_VOL, 1, 1, 0),
	SOC_DOUBLE_R_TLV("Lineout Playback Volume", ADAU1761_PLAY_LINE_LEFT_VOL,
		ADAU1761_PLAY_LINE_RIGHT_VOL, 2, 0x3f, 0, adau1761_out_tlv),
	SOC_DOUBLE_R("Lineout Playback Switch", ADAU1761_PLAY_LINE_LEFT_VOL,
		ADAU1761_PLAY_LINE_RIGHT_VOL, 1, 1, 0),

	SOC_ENUM("ADC Bias", adau1761_adc_bias_enum),
	SOC_ENUM("DAC Bias", adau1761_dac_bias_enum),
	SOC_VALUE_ENUM("Capture Bias", adau1761_capture_bias_enum),
	SOC_VALUE_ENUM("Playback Bias", adau1761_playback_bias_enum),
	SOC_ENUM("Headphone Bias", adau1761_hp_bias_enum),
};

static const struct snd_kcontrol_new adau1761_mono_controls[] = {
	SOC_SINGLE_TLV("Mono Playback Volume", ADAU1761_PLAY_MONO_OUTPUT_VOL,
		2, 0x3f, 0, adau1761_out_tlv),
	SOC_SINGLE("Mono Playback Switch", ADAU1761_PLAY_MONO_OUTPUT_VOL,
		1, 1, 0),
};

static const struct snd_kcontrol_new adau1761_left_mixer_controls[] = {
	SOC_DAPM_SINGLE_VIRT("Left DAC Switch", 0, 1),
	SOC_DAPM_SINGLE_VIRT("Right DAC Switch", 1, 1),
	SOC_DAPM_SINGLE_TLV("Aux Bypass Volume",
		ADAU1761_PLAY_MIXER_LEFT0, 1, 8, 0, adau1761_sidetone_tlv),
	SOC_DAPM_SINGLE_TLV("Right Bypass Volume",
		ADAU1761_PLAY_MIXER_LEFT1, 4, 8, 0, adau1761_sidetone_tlv),
	SOC_DAPM_SINGLE_TLV("Left Bypass Volume",
		ADAU1761_PLAY_MIXER_LEFT1, 0, 8, 0, adau1761_sidetone_tlv),
};

static const struct snd_kcontrol_new adau1761_right_mixer_controls[] = {
	SOC_DAPM_SINGLE_VIRT("Left DAC Switch", 0, 1),
	SOC_DAPM_SINGLE_VIRT("Right DAC Switch", 1, 1),
	SOC_DAPM_SINGLE_TLV("Aux Bypass Volume",
		ADAU1761_PLAY_MIXER_RIGHT0, 1, 8, 0, adau1761_sidetone_tlv),
	SOC_DAPM_SINGLE_TLV("Right Bypass Volume",
		ADAU1761_PLAY_MIXER_RIGHT1, 4, 8, 0, adau1761_sidetone_tlv),
	SOC_DAPM_SINGLE_TLV("Left Bypass Volume",
		ADAU1761_PLAY_MIXER_RIGHT1, 0, 8, 0, adau1761_sidetone_tlv),
};

static const struct snd_kcontrol_new adau1761_left_lr_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("Left Volume",
		ADAU1761_PLAY_LR_MIXER_LEFT, 1, 2, 0, adau1761_boost_tlv),
	SOC_DAPM_SINGLE_TLV("Right Volume",
		ADAU1761_PLAY_LR_MIXER_LEFT, 3, 2, 0, adau1761_boost_tlv),
};

static const struct snd_kcontrol_new adau1761_right_lr_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("Left Volume",
		ADAU1761_PLAY_LR_MIXER_RIGHT, 1, 2, 0, adau1761_boost_tlv),
	SOC_DAPM_SINGLE_TLV("Right Volume",
		ADAU1761_PLAY_LR_MIXER_RIGHT, 3, 2, 0, adau1761_boost_tlv),
};

static const char * const adau1761_input_mux_text[] = {
	"ADC", "DMIC",
};

static const SOC_ENUM_SINGLE_DECL(adau1761_input_mux_enum,
	ADAU17X1_ADC_CONTROL, 2, adau1761_input_mux_text);

static const struct snd_kcontrol_new adau1761_input_mux_control =
	SOC_DAPM_ENUM("Input Select", adau1761_input_mux_enum);

static int adau1761_dejitter_fixup(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct adau *adau = snd_soc_codec_get_drvdata(w->codec);

	/* After any power changes have been made the dejitter circuit
	 * has to be reinitialized. */
	regmap_write(adau->regmap, ADAU1761_DEJITTER, 0);
	if (!adau->master)
		regmap_write(adau->regmap, ADAU1761_DEJITTER, 3);

	return 0;
}

static const struct snd_soc_dapm_widget adau1x61_dapm_widgets[] = {
	SND_SOC_DAPM_MIXER("Left Input Mixer", ADAU1761_REC_MIXER_LEFT0, 0, 0,
		NULL, 0),
	SND_SOC_DAPM_MIXER("Right Input Mixer", ADAU1761_REC_MIXER_RIGHT0, 0, 0,
		NULL, 0),

	/* To avoid clicks and pops the DAC outputs need to be muted when DACs
	 * are disabled. This is why we insert these extra widgets here. The
	 * virtual DAC switches of the playback mixers control whether they get
	 * enabled when the DACs are active. */
	SND_SOC_DAPM_MIXER("Left Playback Mixer Left DAC Mute",
		ADAU1761_PLAY_MIXER_LEFT0, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Left Playback Mixer Right DAC Mute",
		ADAU1761_PLAY_MIXER_LEFT0, 6, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Right Playback Mixer Left DAC Mute",
		ADAU1761_PLAY_MIXER_RIGHT0, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Right Playback Mixer Right DAC Mute",
		ADAU1761_PLAY_MIXER_RIGHT0, 6, 0, NULL, 0),

	SOC_MIXER_ARRAY("Left Playback Mixer", ADAU1761_PLAY_MIXER_LEFT0,
		0, 0, adau1761_left_mixer_controls),
	SOC_MIXER_ARRAY("Right Playback Mixer", ADAU1761_PLAY_MIXER_RIGHT0,
		0, 0, adau1761_right_mixer_controls),
	SOC_MIXER_ARRAY("Left LR Playback Mixer", ADAU1761_PLAY_LR_MIXER_LEFT,
		0, 0, adau1761_left_lr_mixer_controls),
	SOC_MIXER_ARRAY("Right LR Playback Mixer", ADAU1761_PLAY_LR_MIXER_RIGHT,
		0, 0, adau1761_right_lr_mixer_controls),

	SND_SOC_DAPM_SUPPLY("Headphone", ADAU1761_PLAY_HP_LEFT_VOL,
		0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY_S("SYSCLK", 2, SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_POST("Dejitter fixup", adau1761_dejitter_fixup),

	SND_SOC_DAPM_INPUT("LAUX"),
	SND_SOC_DAPM_INPUT("RAUX"),
	SND_SOC_DAPM_INPUT("LINP"),
	SND_SOC_DAPM_INPUT("LINN"),
	SND_SOC_DAPM_INPUT("RINP"),
	SND_SOC_DAPM_INPUT("RINN"),

	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
	SND_SOC_DAPM_OUTPUT("LHP"),
	SND_SOC_DAPM_OUTPUT("RHP"),
};

static const struct snd_soc_dapm_widget adau1761_mono_dapm_widgets[] = {
	SND_SOC_DAPM_MIXER("Mono Playback Mixer", ADAU1761_PLAY_MIXER_MONO,
		0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("MONOOUT"),
};

static const struct snd_soc_dapm_widget adau1761_capless_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("Headphone VGND", 1, ADAU1761_PLAY_MIXER_MONO,
		0, 0, NULL, 0),
};

static const struct snd_soc_dapm_route adau1x61_dapm_routes[] = {
	{ "Left Input Mixer", NULL, "LINP" },
	{ "Left Input Mixer", NULL, "LINN" },
	{ "Left Input Mixer", NULL, "LAUX" },

	{ "Right Input Mixer", NULL, "RINP" },
	{ "Right Input Mixer", NULL, "RINN" },
	{ "Right Input Mixer", NULL, "RAUX" },

	{ "Left Playback Mixer", NULL, "Left Playback Enable"},
	{ "Right Playback Mixer", NULL, "Right Playback Enable"},
	{ "Left LR Playback Mixer", NULL, "Left Playback Enable"},
	{ "Right LR Playback Mixer", NULL, "Right Playback Enable"},

	{ "Left Playback Mixer Left DAC Mute", NULL, "Left DAC" },
	{ "Left Playback Mixer Right DAC Mute", NULL, "Right DAC" },
	{ "Right Playback Mixer Left DAC Mute", NULL, "Left DAC" },
	{ "Right Playback Mixer Right DAC Mute", NULL, "Right DAC" },

	{ "Left Playback Mixer", "Left DAC Switch",
		"Left Playback Mixer Left DAC Mute" },
	{ "Left Playback Mixer", "Right DAC Switch",
		"Left Playback Mixer Right DAC Mute" },

	{ "Right Playback Mixer", "Left DAC Switch",
		"Right Playback Mixer Left DAC Mute" },
	{ "Right Playback Mixer", "Right DAC Switch",
		"Right Playback Mixer Right DAC Mute" },

	{ "Left LR Playback Mixer", "Left Volume", "Left Playback Mixer" },
	{ "Left LR Playback Mixer", "Right Volume", "Right Playback Mixer" },

	{ "Right LR Playback Mixer", "Left Volume", "Left Playback Mixer" },
	{ "Right LR Playback Mixer", "Right Volume", "Right Playback Mixer" },

	{ "Left ADC", NULL, "Left Input Mixer" },
	{ "Right ADC", NULL, "Right Input Mixer" },

	{ "LHP", NULL, "Left Playback Mixer" },
	{ "RHP", NULL, "Right Playback Mixer" },

	{ "LHP", NULL, "Headphone" },
	{ "RHP", NULL, "Headphone" },

	{ "LOUT", NULL, "Left LR Playback Mixer" },
	{ "ROUT", NULL, "Right LR Playback Mixer" },

	{ "Left Playback Mixer", "Aux Bypass Volume", "LAUX" },
	{ "Left Playback Mixer", "Left Bypass Volume", "Left Input Mixer" },
	{ "Left Playback Mixer", "Right Bypass Volume", "Right Input Mixer" },
	{ "Right Playback Mixer", "Aux Bypass Volume", "RAUX" },
	{ "Right Playback Mixer", "Left Bypass Volume", "Left Input Mixer" },
	{ "Right Playback Mixer", "Right Bypass Volume", "Right Input Mixer" },
};

static const struct snd_soc_dapm_route adau1761_mono_dapm_routes[] = {
	{ "Mono Playback Mixer", NULL, "Left Playback Mixer" },
	{ "Mono Playback Mixer", NULL, "Right Playback Mixer" },

	{ "MONOOUT", NULL, "Mono Playback Mixer" },
};

static const struct snd_soc_dapm_route adau1761_capless_dapm_routes[] = {
	{ "Headphone", NULL, "Headphone VGND" },
};

static const struct snd_soc_dapm_widget adau1761_dmic_widgets[] = {
	SND_SOC_DAPM_MUX("Input Select", SND_SOC_NOPM, 0, 0,
		&adau1761_input_mux_control),

	SND_SOC_DAPM_INPUT("DMIC"),
};

static const struct snd_soc_dapm_route adau1761_dmic_routes[] = {
	{ "Input Select", "ADC", "Left ADC" },
	{ "Input Select", "ADC", "Right ADC" },
	{ "Input Select", "DMIC", "DMIC" },

	{ "DMIC", NULL, "Left Dec Filter Enable" },
	{ "DMIC", NULL, "Right Dec Filter Enable" },
};

/*
 * We have to handle 4 different cases, which all require different routes and
 * widgets:
 *	- DSP, DMIC
 *	- DSP, no DMIC
 *	- no DSP, DMIC
 *	- no DSP, no DMIC
 */

DECLARE_ADAU17X1_AIFOUT_MUX(dmic, "ADC/DMIC");
DECLARE_ADAU17X1_AIFOUT_MUX(no_dmic, "ADC");

static const struct snd_soc_dapm_route adau1761_dmic_no_dsp_routes[] = {
	{ "AIFOUT", NULL, "Input Select" },
};

static const struct snd_soc_dapm_route adau1761_dmic_dsp_routes[] = {
	{ "DSP", NULL, "Input Select" },
	{ "AIFOUT Capture Mux", "ADC/DMIC", "Input Select" },
};

static const struct snd_soc_dapm_route adau1761_no_dmic_no_dsp_routes[] = {
	{ "AIFOUT", NULL, "Left ADC" },
	{ "AIFOUT", NULL, "Right ADC" },
};

static const struct snd_soc_dapm_route adau1761_no_dmic_dsp_routes[] = {
	{ "DSP", NULL, "Left ADC" },
	{ "DSP", NULL, "Right ADC" },
	{ "AIFOUT Capture Mux", "ADC", "Left ADC" },
	{ "AIFOUT Capture Mux", "ADC", "Right ADC" },
};

static const struct snd_soc_dapm_widget adau1761_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Serial Port Clock", ADAU1761_CLK_ENABLE0,
		0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Serial Input Routing Clock", ADAU1761_CLK_ENABLE0,
		1, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Serial Output Routing Clock", ADAU1761_CLK_ENABLE0,
		3, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Decimator Resync Clock", ADAU1761_CLK_ENABLE0,
		4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Interpolator Resync Clock", ADAU1761_CLK_ENABLE0,
		2, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Slew Clock", ADAU1761_CLK_ENABLE0, 6, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY_S("Digital Clock 0", 1, ADAU1761_CLK_ENABLE1,
		0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("Digital Clock 1", 1, ADAU1761_CLK_ENABLE1,
		1, 0, NULL, 0),
};

static const struct snd_soc_dapm_route adau1761_dapm_routes[] = {
	{ "Left ADC", NULL, "Digital Clock 0", },
	{ "Right ADC", NULL, "Digital Clock 0", },
	{ "Left DAC", NULL, "Digital Clock 0", },
	{ "Right DAC", NULL, "Digital Clock 0", },

	{ "AIFCLK", NULL, "Digital Clock 1" },

	{ "AIFIN", NULL, "Serial Port Clock" },
	{ "AIFOUT", NULL, "Serial Port Clock" },
	{ "AIFIN", NULL, "Serial Input Routing Clock" },
	{ "AIFOUT", NULL, "Serial Output Routing Clock" },

	{ "AIFIN", NULL, "Decimator Resync Clock" },
	{ "AIFOUT", NULL, "Interpolator Resync Clock" },

	{ "DSP", NULL, "Decimator Resync Clock" },
	{ "DSP", NULL, "Interpolator Resync Clock" },
	{ "DSP", NULL, "Digital Clock 0" },

	{ "Slew Clock", NULL, "Digital Clock 0" },
	{ "Right Playback Mixer", NULL, "Slew Clock" },
	{ "Left Playback Mixer", NULL, "Slew Clock" },

	{ "Digital Clock 0", NULL, "SYSCLK" },
	{ "Digital Clock 1", NULL, "SYSCLK" },

	{ "AIFOUT", NULL, "Decimator Resync Clock" },
};

static int adau1761_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct adau *adau = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		regmap_update_bits(adau->regmap, ADAU17X1_CLOCK_CONTROL,
			ADAU17X1_CLOCK_CONTROL_SYSCLK_EN,
			ADAU17X1_CLOCK_CONTROL_SYSCLK_EN);
		break;
	case SND_SOC_BIAS_OFF:
		regmap_update_bits(adau->regmap, ADAU17X1_CLOCK_CONTROL,
			ADAU17X1_CLOCK_CONTROL_SYSCLK_EN, 0);
		break;

	}
	codec->dapm.bias_level = level;
	return 0;
}

static enum adau1761_output_mode adau1761_get_lineout_mode(
	struct snd_soc_codec *codec)
{
	struct adau1761_platform_data *pdata = codec->dev->platform_data;

	if (pdata)
		return pdata->lineout_mode;

	return ADAU1761_OUTPUT_MODE_LINE;
}

static int adau1761_setup_digmic_jackdetect(struct snd_soc_codec *codec)
{
	struct adau1761_platform_data *pdata = codec->dev->platform_data;
	struct adau *adau = snd_soc_codec_get_drvdata(codec);
	enum adau1761_digmic_jackdet_pin_mode mode;
	const struct snd_soc_dapm_widget *widgets = NULL;
	const struct snd_soc_dapm_route *routes = NULL;
	unsigned int num_widgets = 0;
	unsigned int num_routes = 0;
	unsigned int val = 0;
	int ret;

	if (pdata)
		mode = pdata->digmic_jackdetect_pin_mode;
	else
		mode = ADAU1761_DIGMIC_JACKDET_PIN_MODE_NONE;
	mode = ADAU1761_DIGMIC_JACKDET_PIN_MODE_DIGMIC;

	switch (mode) {
	case ADAU1761_DIGMIC_JACKDET_PIN_MODE_JACKDETECT:
		switch (pdata->jackdetect_debounce_time) {
		case ADAU1761_JACKDETECT_DEBOUNCE_5MS:
		case ADAU1761_JACKDETECT_DEBOUNCE_10MS:
		case ADAU1761_JACKDETECT_DEBOUNCE_20MS:
		case ADAU1761_JACKDETECT_DEBOUNCE_40MS:
			val |= pdata->jackdetect_debounce_time << 6;
			break;
		default:
			return -EINVAL;
		}
		if (pdata->jackdetect_active_low)
			val |= ADAU1761_DIGMIC_JACKDETECT_ACTIVE_LOW;

		ret = snd_soc_add_codec_controls(codec,
			adau1761_jack_detect_controls,
			ARRAY_SIZE(adau1761_jack_detect_controls));
		if (ret)
			return ret;
	case ADAU1761_DIGMIC_JACKDET_PIN_MODE_NONE: /* fallthrough */
		if (adau17x1_has_dsp(adau)) {
			routes = adau1761_no_dmic_dsp_routes;
			num_routes = ARRAY_SIZE(adau1761_no_dmic_dsp_routes);
			ret = snd_soc_dapm_new_controls(&codec->dapm,
				adau17x1_no_dmic_dsp_widgets,
				ARRAY_SIZE(adau17x1_no_dmic_dsp_widgets));
			if (ret)
				return ret;

		} else {
			routes = adau1761_no_dmic_no_dsp_routes;
			num_routes = ARRAY_SIZE(adau1761_no_dmic_no_dsp_routes);
		}
		break;
	case ADAU1761_DIGMIC_JACKDET_PIN_MODE_DIGMIC:
		ret = snd_soc_dapm_new_controls(&codec->dapm,
			adau1761_dmic_widgets,
			ARRAY_SIZE(adau1761_dmic_widgets));
		if (ret)
			return ret;

		if (adau17x1_has_dsp(adau)) {
			routes = adau1761_dmic_dsp_routes;
			num_routes = ARRAY_SIZE(adau1761_dmic_dsp_routes);
			widgets = adau17x1_dmic_dsp_widgets;
			num_widgets = ARRAY_SIZE(adau17x1_dmic_dsp_widgets);
		} else {
			routes = adau1761_dmic_no_dsp_routes;
			num_routes = ARRAY_SIZE(adau1761_dmic_no_dsp_routes);
		}

		ret = snd_soc_dapm_new_controls(&codec->dapm, widgets,
			num_widgets);
		if (ret)
			return ret;

		ret = snd_soc_dapm_add_routes(&codec->dapm,
			adau1761_dmic_routes,
			ARRAY_SIZE(adau1761_dmic_routes));
		if (ret)
			return ret;

		val |= ADAU1761_DIGMIC_JACKDETECT_DIGMIC;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dapm_add_routes(&codec->dapm, routes, num_routes);
	if (ret)
		return ret;

	regmap_write(adau->regmap, ADAU1761_DIGMIC_JACKDETECT, val);

	return 0;
}

static int adau1761_setup_headphone_mode(struct snd_soc_codec *codec)
{
	struct adau *adau = snd_soc_codec_get_drvdata(codec);
	struct adau1761_platform_data *pdata = codec->dev->platform_data;
	enum adau1761_output_mode mode;
	int ret;

	if (pdata)
		mode = pdata->headphone_mode;
	else
		mode = ADAU1761_OUTPUT_MODE_HEADPHONE;

	switch (mode) {
	case ADAU1761_OUTPUT_MODE_LINE:
		break;
	case ADAU1761_OUTPUT_MODE_HEADPHONE_CAPLESS:
		regmap_update_bits(adau->regmap, ADAU1761_PLAY_MONO_OUTPUT_VOL,
			3, 3);
	case ADAU1761_OUTPUT_MODE_HEADPHONE: /* fallthrough */
		regmap_update_bits(adau->regmap, ADAU1761_PLAY_HP_RIGHT_VOL,
			1, 1);
		break;
	default:
		return -EINVAL;
	}

	if (mode == ADAU1761_OUTPUT_MODE_HEADPHONE_CAPLESS) {
		ret = snd_soc_dapm_new_controls(&codec->dapm,
			adau1761_capless_dapm_widgets,
			ARRAY_SIZE(adau1761_capless_dapm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(&codec->dapm,
			adau1761_capless_dapm_routes,
			ARRAY_SIZE(adau1761_capless_dapm_routes));
	} else {
		ret = snd_soc_dapm_new_controls(&codec->dapm,
			adau1761_mono_dapm_widgets,
			ARRAY_SIZE(adau1761_mono_dapm_widgets));
		if (ret)
			return ret;
		ret = snd_soc_dapm_add_routes(&codec->dapm,
			adau1761_mono_dapm_routes,
			ARRAY_SIZE(adau1761_mono_dapm_routes));
	}

	return ret;
}

static bool adau1761_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADAU1761_DIGMIC_JACKDETECT:
	case ADAU1761_REC_MIXER_LEFT0:
	case ADAU1761_REC_MIXER_LEFT1:
	case ADAU1761_REC_MIXER_RIGHT0:
	case ADAU1761_REC_MIXER_RIGHT1:
	case ADAU1761_LEFT_DIFF_INPUT_VOL:
	case ADAU1761_RIGHT_DIFF_INPUT_VOL:
	case ADAU1761_PLAY_LR_MIXER_LEFT:
	case ADAU1761_PLAY_MIXER_LEFT0:
	case ADAU1761_PLAY_MIXER_LEFT1:
	case ADAU1761_PLAY_MIXER_RIGHT0:
	case ADAU1761_PLAY_MIXER_RIGHT1:
	case ADAU1761_PLAY_LR_MIXER_RIGHT:
	case ADAU1761_PLAY_MIXER_MONO:
	case ADAU1761_PLAY_HP_LEFT_VOL:
	case ADAU1761_PLAY_HP_RIGHT_VOL:
	case ADAU1761_PLAY_LINE_LEFT_VOL:
	case ADAU1761_PLAY_LINE_RIGHT_VOL:
	case ADAU1761_PLAY_MONO_OUTPUT_VOL:
	case ADAU1761_POP_CLICK_SUPPRESS:
	case ADAU1761_JACK_DETECT_PIN:
	case ADAU1761_DEJITTER:
	case ADAU1761_CLK_ENABLE0:
	case ADAU1761_CLK_ENABLE1:
		return true;
	default:
		break;
	}

	return adau17x1_readable_register(dev, reg);
}

static int adau1761_probe(struct snd_soc_codec *codec)
{
	struct adau1761_platform_data *pdata = codec->dev->platform_data;
	struct adau *adau = snd_soc_codec_get_drvdata(codec);
	int ret;

	ret = adau17x1_probe(codec);
	if (ret < 0)
		return ret;

	if (pdata && pdata->input_differential) {
		regmap_update_bits(adau->regmap, ADAU1761_LEFT_DIFF_INPUT_VOL,
			ADAU1761_DIFF_INPUT_VOL_LDEN,
			ADAU1761_DIFF_INPUT_VOL_LDEN);
		regmap_update_bits(adau->regmap, ADAU1761_RIGHT_DIFF_INPUT_VOL,
			ADAU1761_DIFF_INPUT_VOL_LDEN,
			ADAU1761_DIFF_INPUT_VOL_LDEN);
		ret = snd_soc_add_codec_controls(codec,
			adau1761_differential_mode_controls,
			ARRAY_SIZE(adau1761_differential_mode_controls));
		if (ret)
			return ret;
	} else {
		ret = snd_soc_add_codec_controls(codec,
			adau1761_single_mode_controls,
			ARRAY_SIZE(adau1761_single_mode_controls));
		if (ret)
			return ret;
	}

	switch (adau1761_get_lineout_mode(codec)) {
	case ADAU1761_OUTPUT_MODE_LINE:
		break;
	case ADAU1761_OUTPUT_MODE_HEADPHONE:
		regmap_update_bits(adau->regmap, ADAU1761_PLAY_LINE_LEFT_VOL,
			1, 1);
		regmap_update_bits(adau->regmap, ADAU1761_PLAY_LINE_RIGHT_VOL,
			1, 1);
		break;
	default:
		return -EINVAL;
	}

	ret = adau1761_setup_headphone_mode(codec);
	if (ret)
		return ret;

	ret = adau1761_setup_digmic_jackdetect(codec);
	if (ret)
		return ret;

	ret = adau17x1_add_routes(codec);
	if (ret < 0)
		return ret;

	if (adau->type == ADAU1761) {
		ret = snd_soc_dapm_new_controls(&codec->dapm,
			adau1761_dapm_widgets,
			ARRAY_SIZE(adau1761_dapm_widgets));
		if (ret)
			return ret;

		ret = snd_soc_dapm_add_routes(&codec->dapm,
			adau1761_dapm_routes,
			ARRAY_SIZE(adau1761_dapm_routes));
		if (ret)
			return ret;

		ret = adau17x1_load_firmware(adau, codec->dev,
			ADAU1761_FIRMWARE);
		if (ret)
			dev_warn(codec->dev, "Failed to firmware\n");
	}

	return 0;
}

static struct snd_soc_codec_driver adau1761_codec_driver = {
	.probe			= adau1761_probe,
	.remove			= adau17x1_suspend,
	.suspend		= adau17x1_suspend,
	.resume			= adau17x1_resume,
	.set_bias_level		= adau1761_set_bias_level,

	.controls		= adau1761_controls,
	.num_controls		= ARRAY_SIZE(adau1761_controls),
	.dapm_widgets		= adau1x61_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(adau1x61_dapm_widgets),
	.dapm_routes		= adau1x61_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(adau1x61_dapm_routes),
};

#define ADAU1761_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
	SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver adau1361_dai_driver = {
	.name = "adau-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ADAU1761_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ADAU1761_FORMATS,
	},
	.ops = &adau17x1_dai_ops,
};

static struct snd_soc_dai_driver adau1761_dai_driver = {
	.name = "adau-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ADAU1761_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ADAU1761_FORMATS,
	},
	.ops = &adau17x1_dai_ops,
};

static int adau1761_bus_probe(struct device *dev,
	struct regmap *regmap, enum adau17x1_type type,
	enum snd_soc_control_type control_type)
{
	struct snd_soc_dai_driver *dai_drv;
	int ret;

	ret = adau17x1_bus_probe(dev, regmap, type, control_type);
	if (ret)
		return ret;

	if (type == ADAU1361)
		dai_drv = &adau1361_dai_driver;
	else
		dai_drv = &adau1761_dai_driver;

	return snd_soc_register_codec(dev, &adau1761_codec_driver, dai_drv, 1);
}

#if IS_ENABLED(CONFIG_SPI_MASTER)

static const struct regmap_config adau1761_spi_regmap_config = {
	.val_bits		= 8,
	.reg_bits		= 24,
	.read_flag_mask		= 0x01,
	.max_register		= 0x40fa,
	.reg_defaults		= adau1761_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(adau1761_reg_defaults),
	.readable_reg		= adau1761_readable_register,
	.volatile_reg		= adau17x1_volatile_register,
	.cache_type		= REGCACHE_RBTREE,
};

static int adau1761_spi_probe(struct spi_device *spi)
{
	enum adau17x1_type type = spi_get_device_id(spi)->driver_data;
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &adau1761_spi_regmap_config);

	return adau1761_bus_probe(&spi->dev, regmap, type, SND_SOC_SPI);
}

static int adau1761_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	return 0;
}

static const struct spi_device_id adau1761_spi_id[] = {
	{ "adau1361", ADAU1361 },
	{ "adau1461", ADAU1761 },
	{ "adau1761", ADAU1761 },
	{ "adau1961", ADAU1361 },
	{ }
};
MODULE_DEVICE_TABLE(spi, adau1761_spi_id);

static struct spi_driver adau1761_spi_driver = {
	.driver = {
		.name	= "adau1761",
		.owner	= THIS_MODULE,
	},
	.probe		= adau1761_spi_probe,
	.remove		= adau1761_spi_remove,
	.id_table	= adau1761_spi_id,
};

static int __init adau1761_spi_register_driver(void)
{
	return spi_register_driver(&adau1761_spi_driver);
}

static void adau1761_spi_unregister_driver(void)
{
	spi_unregister_driver(&adau1761_spi_driver);
}

#else
static int adau1761_spi_register_driver(void) { return 0; }
static void adau1761_spi_unregister_driver(void) {}
#endif

#if IS_ENABLED(CONFIG_I2C)

static const struct regmap_config adau1761_i2c_regmap_config = {
	.val_bits		= 8,
	.reg_bits		= 16,
	.max_register		= 0x40fa,
	.reg_defaults		= adau1761_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(adau1761_reg_defaults),
	.readable_reg		= adau1761_readable_register,
	.volatile_reg		= adau17x1_volatile_register,
	.cache_type		= REGCACHE_RBTREE,
};

static int adau1761_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	enum adau17x1_type type = id->driver_data;
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &adau1761_i2c_regmap_config);

	return adau1761_bus_probe(&client->dev, regmap, type, SND_SOC_I2C);
}

static int adau1761_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id adau1761_i2c_id[] = {
	{ "adau1361", ADAU1361 },
	{ "adau1461", ADAU1761 },
	{ "adau1761", ADAU1761 },
	{ "adau1961", ADAU1361 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1761_i2c_id);

static struct i2c_driver adau1761_i2c_driver = {
	.driver = {
		.name = "adau1761",
		.owner = THIS_MODULE,
	},
	.probe = adau1761_i2c_probe,
	.remove = adau1761_i2c_remove,
	.id_table = adau1761_i2c_id,
};

static int __init adau1761_i2c_register_driver(void)
{
	return i2c_add_driver(&adau1761_i2c_driver);
}

static void __exit adau1761_i2c_unregister_driver(void)
{
	i2c_del_driver(&adau1761_i2c_driver);
}

#else
static int adau1761_i2c_register_driver(void) { return 0; }
static void adau1761_i2c_unregister_driver(void) {}
#endif

static int __init adau1761_init(void)
{
	int ret;

	ret = adau1761_spi_register_driver();
	if (ret)
		return ret;

	ret = adau1761_i2c_register_driver();
	if (ret)
		adau1761_spi_unregister_driver();

	return ret;
}
module_init(adau1761_init);

static void __exit adau1761_exit(void)
{
	adau1761_i2c_unregister_driver();
	adau1761_spi_unregister_driver();
}
module_exit(adau1761_exit);

MODULE_DESCRIPTION("ASoC ADAU1361/ADAU1461/ADAU1761/ADAU1961 CODEC driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
