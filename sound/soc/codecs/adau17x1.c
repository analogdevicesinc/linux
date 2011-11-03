/*
 * Common code for ADAU1X61 and ADAU1X81 codecs
 *
 * Copyright 2011-2013 Analog Devices Inc.
 * Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/gcd.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include "sigmadsp.h"
#include "adau17x1.h"

static const char * const adau17x1_capture_mixer_boost_text[] = {
	"Normal operation", "Boost Level 1", "Boost Level 2", "Boost Level 3",
};

static const SOC_ENUM_SINGLE_DECL(adau17x1_capture_boost_enum,
	ADAU17X1_REC_POWER_MGMT, 5, adau17x1_capture_mixer_boost_text);

static const char * const adau17x1_mic_bias_mode_text[] = {
	"Normal operation", "High performance",
};

static SOC_ENUM_SINGLE_DECL(adau17x1_mic_bias_mode_enum,
	ADAU17X1_MICBIAS, 3, adau17x1_mic_bias_mode_text);

static const char * const adau17x1_mono_stereo_text[] = {
	"Stereo",
	"Mono Left Channel (L+R)",
	"Mono Right Channel (L+R)",
	"Mono (L+R)",
};

static const SOC_ENUM_SINGLE_DECL(adau17x1_dac_mode_enum,
	ADAU17X1_DAC_CONTROL0, 6, adau17x1_mono_stereo_text);

static const DECLARE_TLV_DB_MINMAX(adau17x1_digital_tlv, -9563, 0);

static const struct snd_kcontrol_new adau17x1_controls[] = {
	SOC_DOUBLE_R_TLV("Digital Capture Volume",
		ADAU17X1_LEFT_INPUT_DIGITAL_VOL,
		ADAU17X1_RIGHT_INPUT_DIGITAL_VOL,
		0, 0xff, 1, adau17x1_digital_tlv),
	SOC_DOUBLE_R_TLV("Digital Playback Volume", ADAU17X1_DAC_CONTROL1,
		ADAU17X1_DAC_CONTROL2, 0, 0xff, 1, adau17x1_digital_tlv),

	SOC_SINGLE("ADC High Pass Filter Switch", ADAU17X1_ADC_CONTROL,
		5, 1, 0),
	SOC_SINGLE("Playback De-emphasis Switch", ADAU17X1_DAC_CONTROL0,
		2, 1, 0),

	SOC_ENUM("Capture Boost", adau17x1_capture_boost_enum),

	SOC_ENUM("Mic Bias Mode", adau17x1_mic_bias_mode_enum),

	SOC_ENUM("DAC Mono Stereo", adau17x1_dac_mode_enum),
};

static int adau17x1_pll_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct adau *adau = snd_soc_codec_get_drvdata(w->codec);
	int ret;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		adau->pll_regs[5] = 1;
	} else {
		adau->pll_regs[5] = 0;
		/* Bypass the PLL when disabled, otherwise registers will become
		 * inaccessible. */
		regmap_update_bits(adau->regmap, ADAU17X1_CLOCK_CONTROL,
			ADAU17X1_CLOCK_CONTROL_CORECLK_SRC_PLL, 0);
	}

	/* The PLL register is 6 bytes long and can only be written at once. */
	ret = regmap_raw_write(adau->regmap, ADAU17X1_PLL_CONTROL,
			adau->pll_regs, ARRAY_SIZE(adau->pll_regs));

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		mdelay(5);
		regmap_update_bits(adau->regmap, ADAU17X1_CLOCK_CONTROL,
			ADAU17X1_CLOCK_CONTROL_CORECLK_SRC_PLL,
			ADAU17X1_CLOCK_CONTROL_CORECLK_SRC_PLL);
	}

	return 0;
}

static const struct snd_soc_dapm_widget adau17x1_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("PLL", 3, SND_SOC_NOPM, 0, 0, adau17x1_pll_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("AIFCLK", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("MICBIAS", ADAU17X1_MICBIAS, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Left Playback Enable", ADAU17X1_PLAY_POWER_MGMT,
		0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Right Playback Enable", ADAU17X1_PLAY_POWER_MGMT,
		1, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Left Dec Filter Enable", ADAU17X1_ADC_CONTROL,
		0, 0, NULL,  0),
	SND_SOC_DAPM_SUPPLY("Right Dec Filter Enable", ADAU17X1_ADC_CONTROL,
		1, 0, NULL,  0),

	SND_SOC_DAPM_DAC("Left DAC", NULL, ADAU17X1_DAC_CONTROL0, 0, 0),
	SND_SOC_DAPM_DAC("Right DAC", NULL, ADAU17X1_DAC_CONTROL0, 1, 0),
	SND_SOC_DAPM_ADC("Left ADC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("Right ADC", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_OUT("AIFOUT", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFIN", "Playback", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route adau17x1_dapm_routes[] = {
	{ "Left ADC", NULL, "SYSCLK" },
	{ "Right ADC", NULL, "SYSCLK" },
	{ "Left DAC", NULL, "SYSCLK" },
	{ "Right DAC", NULL, "SYSCLK" },
	{ "AIFOUT", NULL, "SYSCLK" },
	{ "AIFIN", NULL, "SYSCLK" },

	{ "Left ADC", NULL, "Left Dec Filter Enable" },
	{ "Right ADC", NULL, "Right Dec Filter Enable" },

	{ "AIFOUT", NULL, "AIFCLK" },
	{ "AIFIN", NULL, "AIFCLK" },
};

static const struct snd_soc_dapm_route adau17x1_dapm_pll_route = {
	"SYSCLK", NULL, "PLL",
};

static const char * const adau17x1_dac_mux_text[] = {
	"AIFIN",
	"DSP",
};

int adau17x1_dsp_mux_enum_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *w = wlist->widgets[0];
	struct adau *adau = snd_soc_codec_get_drvdata(w->codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, change;
	struct snd_soc_dapm_update update;

	if (ucontrol->value.enumerated.item[0] >= e->max)
		return -EINVAL;

	switch (ucontrol->value.enumerated.item[0]) {
	case 0:
		switch (e->reg) {
		case ADAU17X1_SERIAL_INPUT_ROUTE:
			val = (adau->tdm_dac_slot * 2) + 1;
			break;
		case ADAU17X1_SERIAL_OUTPUT_ROUTE:
			val = (adau->tdm_dac_slot * 2) + 1;
			break;
		default:
			val = 0;
			break;
		}
		break;
	default:
		val = 0;
		break;
	}

	change = snd_soc_test_bits(w->codec, e->reg, 0xff, val);
	if (change) {
		update.kcontrol = kcontrol;
		update.widget = w;
		update.reg = e->reg;
		update.mask = 0xff;
		update.val = val;
		w->dapm->update = &update;

		snd_soc_dapm_mux_update_power(w, kcontrol,
				ucontrol->value.enumerated.item[0], e);

		w->dapm->update = NULL;
	}

	return change;
}
EXPORT_SYMBOL_GPL(adau17x1_dsp_mux_enum_put);

int adau17x1_dsp_mux_enum_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;

	val = snd_soc_read(widget->codec, e->reg);
	ucontrol->value.enumerated.item[0] = val == 0;

	return 0;
}
EXPORT_SYMBOL_GPL(adau17x1_dsp_mux_enum_get);

static const SOC_ENUM_SINGLE_DECL(adau17x1_dac_mux_enum,
	ADAU17X1_SERIAL_INPUT_ROUTE, 0,
	adau17x1_dac_mux_text);

static const struct snd_kcontrol_new adau17x1_dac_mux =
	ADAU17X1_DSP_MUX_ENUM("DAC Playback Mux", adau17x1_dac_mux_enum);

static const struct snd_soc_dapm_widget adau17x1_dsp_dapm_widgets[] = {
	SND_SOC_DAPM_PGA("DSP", ADAU17X1_DSP_RUN, 0, 0, NULL, 0),
	SND_SOC_DAPM_SIGGEN("DSP Siggen"),

	SND_SOC_DAPM_VIRT_MUX("DAC Playback Mux", SND_SOC_NOPM, 0, 0,
		&adau17x1_dac_mux),

};

static const struct snd_soc_dapm_route adau17x1_dsp_dapm_routes[] = {
	{ "DAC Playback Mux", "DSP", "DSP" },
	{ "DAC Playback Mux", "AIFIN", "AIFIN" },

	{ "Left DAC", NULL, "DAC Playback Mux" },
	{ "Right DAC", NULL, "DAC Playback Mux" },

	{ "AIFOUT Capture Mux", "DSP", "DSP" },

	{ "AIFOUT", NULL, "AIFOUT Capture Mux" },

	{ "DSP", NULL, "DSP Siggen" },
};

static const struct snd_soc_dapm_route adau17x1_no_dsp_dapm_routes[] = {
	{ "Left DAC", NULL, "AIFIN" },
	{ "Right DAC", NULL, "AIFIN" },
};

static void adau17x1_check_aifclk(struct snd_soc_codec *codec)
{
	struct adau *adau = snd_soc_codec_get_drvdata(codec);

	/* If we are in master mode we need to generate bit- and frameclock,
	 * regardless of whether there is an active path or not */
	if (codec->active && adau->master)
		snd_soc_dapm_force_enable_pin(&codec->dapm, "AIFCLK");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "AIFCLK");
	snd_soc_dapm_sync(&codec->dapm);
}

bool adau17x1_has_dsp(struct adau *adau)
{
	switch (adau->type) {
	case ADAU1761:
	case ADAU1381:
	case ADAU1781:
		return true;
	default:
		return false;
	}
}
EXPORT_SYMBOL_GPL(adau17x1_has_dsp);

static int adau17x1_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct adau *adau = snd_soc_codec_get_drvdata(codec);
	unsigned int val, div, dsp_div;
	unsigned int freq;

	if (adau->clk_src == ADAU17X1_CLK_SRC_PLL)
		freq = adau->pll_freq;
	else
		freq = adau->sysclk / adau->sysclk_div;

	if (freq % params_rate(params) != 0)
		return -EINVAL;

	switch (freq / params_rate(params)) {
	case 1024: /* fs */
		div = 0;
		dsp_div = 1;
		break;
	case 6144: /* fs / 6 */
		div = 1;
		dsp_div = 6;
		break;
	case 4096: /* fs / 4 */
		div = 2;
		dsp_div = 5;
		break;
	case 3072: /* fs / 3 */
		div = 3;
		dsp_div = 4;
		break;
	case 2048: /* fs / 2 */
		div = 4;
		dsp_div = 3;
		break;
	case 1536: /* fs / 1.5 */
		div = 5;
		dsp_div = 2;
		break;
	case 512: /* fs / 0.5 */
		div = 6;
		dsp_div = 0;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(adau->regmap, ADAU17X1_CONVERTER0, 7, div);
	if (adau17x1_has_dsp(adau)) {
		regmap_write(adau->regmap, ADAU17X1_SERIAL_SAMPLING_RATE, div);
		regmap_write(adau->regmap, ADAU17X1_DSP_SAMPLING_RATE, dsp_div);
	}

	adau17x1_check_aifclk(codec);

	if (adau->dai_fmt != SND_SOC_DAIFMT_RIGHT_J)
		return 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		val = ADAU17X1_SERIAL_PORT1_DELAY16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val = ADAU17X1_SERIAL_PORT1_DELAY8;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val = ADAU17X1_SERIAL_PORT1_DELAY0;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(adau->regmap, ADAU17X1_SERIAL_PORT1,
			ADAU17X1_SERIAL_PORT1_DELAY_MASK, val);
}

static int adau17x1_dai_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	adau17x1_check_aifclk(dai->codec);

	return 0;
}

static void adau17x1_dai_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	adau17x1_check_aifclk(dai->codec);
}

static int adau17x1_set_dai_pll(struct snd_soc_dai *dai, int pll_id,
	int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = dai->codec;
	struct adau *adau = snd_soc_codec_get_drvdata(codec);
	unsigned int div;
	unsigned int r, n, m, i, j;

	if (freq_in < 8000000 || freq_in > 27000000)
		return -EINVAL;

	if (!freq_out) {
		r = 0;
		n = 0;
		m = 0;
		div = 0;
	} else {
		if (freq_out % freq_in != 0) {
			div = DIV_ROUND_UP(freq_in, 13500000);
			freq_in /= div;
			r = freq_out / freq_in;
			i = freq_out % freq_in;
			j = gcd(i, freq_in);
			n = i / j;
			m = freq_in / j;
			div--;
		} else {
			r = freq_out / freq_in;
			n = 0;
			m = 0;
			div = 0;
		}
		if (n > 0xffff || m > 0xffff || div > 3 || r > 8 || r < 2)
			return -EINVAL;
	}

	adau->pll_regs[0] = m >> 8;
	adau->pll_regs[1] = m & 0xff;
	adau->pll_regs[2] = n >> 8;
	adau->pll_regs[3] = n & 0xff;
	adau->pll_regs[4] = (r << 3) | (div << 1);
	if (m != 0)
		adau->pll_regs[4] |= 1; /* Fractional mode */
	adau->pll_regs[5] = 0;

	adau->pll_freq = freq_out;

	return 0;
}

static int adau17x1_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct adau *adau = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_dapm_context *dapm = &dai->codec->dapm;

	switch (clk_id) {
	case ADAU17X1_CLK_SRC_MCLK:
	case ADAU17X1_CLK_SRC_PLL:
		break;
	default:
		return -EINVAL;
	}

	adau->sysclk = freq;

	if (adau->clk_src != clk_id) {
		if (clk_id == ADAU17X1_CLK_SRC_PLL) {
			snd_soc_dapm_add_routes(dapm,
				&adau17x1_dapm_pll_route, 1);
		} else {
			snd_soc_dapm_del_routes(dapm,
				&adau17x1_dapm_pll_route, 1);
		}
	}

	adau->clk_src = clk_id;

	return 0;
}

static int adau17x1_set_dai_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct adau *adau = snd_soc_codec_get_drvdata(dai->codec);

	switch (div) {
	case 1:
	case 2:
	case 3:
	case 4:
		break;
	default:
		return -EINVAL;
	}

	adau->sysclk_div = div;

	return regmap_update_bits(adau->regmap, ADAU17X1_CLOCK_CONTROL,
		ADAU17X1_CLOCK_CONTROL_INFREQ_MASK, (div - 1) << 1);
}

static int adau17x1_set_dai_fmt(struct snd_soc_dai *dai,
		unsigned int fmt)
{
	struct adau *adau = snd_soc_codec_get_drvdata(dai->codec);
	unsigned int ctrl0, ctrl1;
	int lrclk_pol;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		ctrl0 = ADAU17X1_SERIAL_PORT0_MASTER;
		adau->master = true;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		ctrl0 = 0;
		adau->master = false;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		lrclk_pol = 0;
		ctrl1 = ADAU17X1_SERIAL_PORT1_DELAY1;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
		lrclk_pol = 1;
		ctrl1 = ADAU17X1_SERIAL_PORT1_DELAY0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		lrclk_pol = 1;
		ctrl0 |= ADAU17X1_SERIAL_PORT0_PULSE_MODE;
		ctrl1 = ADAU17X1_SERIAL_PORT1_DELAY1;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		lrclk_pol = 1;
		ctrl0 |= ADAU17X1_SERIAL_PORT0_PULSE_MODE;
		ctrl1 = ADAU17X1_SERIAL_PORT1_DELAY0;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl0 |= ADAU17X1_SERIAL_PORT0_BCLK_POL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		lrclk_pol = !lrclk_pol;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		ctrl0 |= ADAU17X1_SERIAL_PORT0_BCLK_POL;
		lrclk_pol = !lrclk_pol;
		break;
	default:
		return -EINVAL;
	}

	if (lrclk_pol)
		ctrl0 |= ADAU17X1_SERIAL_PORT0_LRCLK_POL;

	regmap_write(adau->regmap, ADAU17X1_SERIAL_PORT0, ctrl0);
	regmap_write(adau->regmap, ADAU17X1_SERIAL_PORT1, ctrl1);

	adau->dai_fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	return 0;
}

static int adau17x1_set_dai_tdm_slot(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct adau *adau = snd_soc_codec_get_drvdata(dai->codec);
	unsigned int ser_ctrl0, ser_ctrl1;
	unsigned int conv_ctrl0, conv_ctrl1;

	/* I2S mode */
	if (slots == 0) {
		slots = 2;
		rx_mask = 3;
		tx_mask = 3;
		slot_width = 32;
	}

	switch (slots) {
	case 2:
		ser_ctrl0 = ADUA_SERIAL_PORT0_STEREO;
		break;
	case 4:
		ser_ctrl0 = ADUA_SERIAL_PORT0_TDM4;
		break;
	case 8:
		if (adau->type == ADAU1361)
			return -EINVAL;

		ser_ctrl0 = ADUA_SERIAL_PORT0_TDM8;
		break;
	default:
		return -EINVAL;
	}

	switch (slot_width * slots) {
	case 32:
		if (adau->type == ADAU1761)
			return -EINVAL;

		ser_ctrl1 = ADUA_SERIAL_PORT1_BCLK32;
		break;
	case 64:
		ser_ctrl1 = ADUA_SERIAL_PORT1_BCLK64;
		break;
	case 48:
		ser_ctrl1 = ADUA_SERIAL_PORT1_BCLK48;
		break;
	case 128:
		ser_ctrl1 = ADUA_SERIAL_PORT1_BCLK128;
		break;
	case 256:
		if (adau->type == ADAU1361)
			return -EINVAL;

		ser_ctrl1 = ADUA_SERIAL_PORT1_BCLK256;
		break;
	default:
		return -EINVAL;
	}

	switch (rx_mask) {
	case 0x03:
		conv_ctrl1 = ADAU17X1_CONVERTER1_ADC_PAIR(1);
		adau->tdm_adc_slot = 0;
		break;
	case 0x0c:
		conv_ctrl1 = ADAU17X1_CONVERTER1_ADC_PAIR(2);
		adau->tdm_adc_slot = 1;
		break;
	case 0x30:
		conv_ctrl1 = ADAU17X1_CONVERTER1_ADC_PAIR(3);
		adau->tdm_adc_slot = 2;
		break;
	case 0xc0:
		conv_ctrl1 = ADAU17X1_CONVERTER1_ADC_PAIR(4);
		adau->tdm_adc_slot = 3;
		break;
	default:
		return -EINVAL;
	}

	switch (tx_mask) {
	case 0x03:
		conv_ctrl0 = ADAU17X1_CONVERTER0_DAC_PAIR(1);
		adau->tdm_dac_slot = 0;
		break;
	case 0x0c:
		conv_ctrl0 = ADAU17X1_CONVERTER0_DAC_PAIR(2);
		adau->tdm_dac_slot = 1;
		break;
	case 0x30:
		conv_ctrl0 = ADAU17X1_CONVERTER0_DAC_PAIR(3);
		adau->tdm_dac_slot = 2;
		break;
	case 0xc0:
		conv_ctrl0 = ADAU17X1_CONVERTER0_DAC_PAIR(4);
		adau->tdm_dac_slot = 3;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(adau->regmap, ADAU17X1_CONVERTER0,
		ADAU17X1_CONVERTER0_DAC_PAIR_MASK, conv_ctrl0);
	regmap_update_bits(adau->regmap, ADAU17X1_CONVERTER1,
		ADAU17X1_CONVERTER1_ADC_PAIR_MASK, conv_ctrl1);
	regmap_update_bits(adau->regmap, ADAU17X1_SERIAL_PORT0,
		ADAU17X1_SERIAL_PORT0_TDM_MASK, ser_ctrl0);
	regmap_update_bits(adau->regmap, ADAU17X1_SERIAL_PORT1,
		ADAU17X1_SERIAL_PORT1_BCLK_MASK, ser_ctrl1);

	if (adau17x1_has_dsp(adau)) {
		if (adau->dsp_playback_bypass) {
			regmap_write(adau->regmap, ADAU17X1_SERIAL_INPUT_ROUTE,
				(adau->tdm_dac_slot * 2) + 1);
		}
		if (adau->dsp_capture_bypass) {
			regmap_write(adau->regmap, ADAU17X1_SERIAL_OUTPUT_ROUTE,
				(adau->tdm_adc_slot * 2) + 1);
		}
	}

	return 0;
}

const struct snd_soc_dai_ops adau17x1_dai_ops = {
	.hw_params	= adau17x1_hw_params,
	.set_sysclk	= adau17x1_set_dai_sysclk,
	.set_fmt	= adau17x1_set_dai_fmt,
	.set_pll	= adau17x1_set_dai_pll,
	.set_clkdiv	= adau17x1_set_dai_clkdiv,
	.set_tdm_slot	= adau17x1_set_dai_tdm_slot,
	.startup	= adau17x1_dai_startup,
	.shutdown	= adau17x1_dai_shutdown,
};
EXPORT_SYMBOL_GPL(adau17x1_dai_ops);

int adau17x1_set_micbias_voltage(struct snd_soc_codec *codec,
	enum adau17x1_micbias_voltage micbias)
{
	struct adau *adau = snd_soc_codec_get_drvdata(codec);

	switch (micbias) {
	case ADAU17X1_MICBIAS_0_90_AVDD:
	case ADAU17X1_MICBIAS_0_65_AVDD:
		break;
	default:
		return -EINVAL;
	}

	regmap_write(adau->regmap, ADAU17X1_MICBIAS, micbias << 2);

	return 0;
}
EXPORT_SYMBOL_GPL(adau17x1_set_micbias_voltage);

bool adau17x1_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADAU17X1_CLOCK_CONTROL:
	case ADAU17X1_PLL_CONTROL:
	case ADAU17X1_REC_POWER_MGMT:
	case ADAU17X1_MICBIAS:
	case ADAU17X1_SERIAL_PORT0:
	case ADAU17X1_SERIAL_PORT1:
	case ADAU17X1_CONVERTER0:
	case ADAU17X1_CONVERTER1:
	case ADAU17X1_LEFT_INPUT_DIGITAL_VOL:
	case ADAU17X1_RIGHT_INPUT_DIGITAL_VOL:
	case ADAU17X1_ADC_CONTROL:
	case ADAU17X1_PLAY_POWER_MGMT:
	case ADAU17X1_DAC_CONTROL0:
	case ADAU17X1_DAC_CONTROL1:
	case ADAU17X1_DAC_CONTROL2:
	case ADAU17X1_SERIAL_PORT_PAD:
	case ADAU17X1_CONTROL_PORT_PAD0:
	case ADAU17X1_CONTROL_PORT_PAD1:
	case ADAU17X1_DSP_SAMPLING_RATE:
	case ADAU17X1_SERIAL_INPUT_ROUTE:
	case ADAU17X1_SERIAL_OUTPUT_ROUTE:
	case ADAU17X1_DSP_ENABLE:
	case ADAU17X1_DSP_RUN:
	case ADAU17X1_SERIAL_SAMPLING_RATE:
		return true;
	default:
		break;
	}
	return false;
}
EXPORT_SYMBOL_GPL(adau17x1_readable_register);

bool adau17x1_volatile_register(struct device *dev, unsigned int reg)
{
	/* SigmaDSP parameter and program memory */
	if (reg < 0x4000)
		return true;

	switch (reg) {
	/* The PLL register is 6 bytes long */
	case ADAU17X1_PLL_CONTROL:
	case ADAU17X1_PLL_CONTROL + 1:
	case ADAU17X1_PLL_CONTROL + 2:
	case ADAU17X1_PLL_CONTROL + 3:
	case ADAU17X1_PLL_CONTROL + 4:
	case ADAU17X1_PLL_CONTROL + 5:
		return true;
	default:
		break;
	}

	return false;
}
EXPORT_SYMBOL_GPL(adau17x1_volatile_register);

int adau17x1_load_firmware(struct adau *adau, struct device *dev,
	const char *firmware)
{
	int ret;
	int dspsr;

	ret = regmap_read(adau->regmap, ADAU17X1_DSP_SAMPLING_RATE, &dspsr);
	if (ret)
		return ret;

	regmap_write(adau->regmap, ADAU17X1_DSP_ENABLE, 1);
	regmap_write(adau->regmap, ADAU17X1_DSP_SAMPLING_RATE, 0xf);

	ret = process_sigma_firmware_regmap(dev, adau->regmap, firmware);
	if (ret) {
		regmap_write(adau->regmap, ADAU17X1_DSP_ENABLE, 0);
		return ret;
	}
	regmap_write(adau->regmap, ADAU17X1_DSP_SAMPLING_RATE, dspsr);

	return 0;
}
EXPORT_SYMBOL_GPL(adau17x1_load_firmware);

int adau17x1_probe(struct snd_soc_codec *codec)
{
	struct adau *adau = snd_soc_codec_get_drvdata(codec);
	int ret;

	ret = snd_soc_codec_set_cache_io(codec, 0, 0, SND_SOC_REGMAP);
	if (ret)
		return ret;

	codec->driver->set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	ret = snd_soc_add_codec_controls(codec, adau17x1_controls,
		ARRAY_SIZE(adau17x1_controls));
	if (ret)
		return ret;
	ret = snd_soc_dapm_new_controls(&codec->dapm, adau17x1_dapm_widgets,
		ARRAY_SIZE(adau17x1_dapm_widgets));
	if (ret)
		return ret;

	if (adau17x1_has_dsp(adau)) {
		ret = snd_soc_dapm_new_controls(&codec->dapm,
			adau17x1_dsp_dapm_widgets,
			ARRAY_SIZE(adau17x1_dsp_dapm_widgets));
	}
	return ret;
}
EXPORT_SYMBOL_GPL(adau17x1_probe);

int adau17x1_add_routes(struct snd_soc_codec *codec)
{
	struct adau *adau = snd_soc_codec_get_drvdata(codec);
	int ret;

	ret = snd_soc_dapm_add_routes(&codec->dapm, adau17x1_dapm_routes,
		ARRAY_SIZE(adau17x1_dapm_routes));
	if (ret)
		return ret;

	if (adau17x1_has_dsp(adau)) {
		ret = snd_soc_dapm_add_routes(&codec->dapm,
			adau17x1_dsp_dapm_routes,
			ARRAY_SIZE(adau17x1_dsp_dapm_routes));
	} else {
		ret = snd_soc_dapm_add_routes(&codec->dapm,
			adau17x1_no_dsp_dapm_routes,
			ARRAY_SIZE(adau17x1_no_dsp_dapm_routes));
	}
	return ret;
}
EXPORT_SYMBOL_GPL(adau17x1_add_routes);

#if IS_ENABLED(CONFIG_SPI_MASTER)
static void adau17x1_spi_mode(struct device *dev)
{
	/* To get the device into SPI mode CLATCH has to be pulled low three
	 * times. Do this by issuing three dummy reads. */
	spi_w8r8(to_spi_device(dev), 0x00);
	spi_w8r8(to_spi_device(dev), 0x00);
	spi_w8r8(to_spi_device(dev), 0x00);
}
#else
static inline void adau17x1_spi_mode(struct device *dev) {}
#endif

int adau17x1_suspend(struct snd_soc_codec *codec)
{
	codec->driver->set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}
EXPORT_SYMBOL_GPL(adau17x1_suspend);

int adau17x1_resume(struct snd_soc_codec *codec)
{
	struct adau *adau = snd_soc_codec_get_drvdata(codec);

	if (adau->control_type == SND_SOC_SPI)
		adau17x1_spi_mode(codec->dev);

	codec->driver->set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	regcache_sync(adau->regmap);

	return 0;
}
EXPORT_SYMBOL_GPL(adau17x1_resume);

int adau17x1_bus_probe(struct device *dev, struct regmap *regmap,
	enum adau17x1_type type, enum snd_soc_control_type control_type)
{
	struct adau *adau;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	adau = devm_kzalloc(dev, sizeof(*adau), GFP_KERNEL);
	if (!adau)
		return -ENOMEM;

	adau->regmap = regmap;
	adau->control_type = control_type;
	adau->type = type;
	adau->sysclk_div = 1;

	dev_set_drvdata(dev, adau);

	if (control_type == SND_SOC_SPI)
		adau17x1_spi_mode(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(adau17x1_bus_probe);

MODULE_DESCRIPTION("ASoC ADAU1X61/ADAU1X81 common code");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
