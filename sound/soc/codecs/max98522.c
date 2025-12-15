// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * max98522.c  --  MAX98522 ALSA Soc Audio driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
 *
 */

#include <linux/acpi.h>
#include <linux/cdev.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "max98522.h"

static struct reg_default max98522_reg_defaults[] = {
	{MAX98522_SOFTWARE_RESET, 0x00},
	{MAX98522_SUPPLY_AND_OTP_INT_RAW, 0x00},
	{MAX98522_POWER_UP_AND_DOWN_INT_RAW, 0x00},
	{MAX98522_CLK_AND_DATA_INT_RAW, 0x00},
	{MAX98522_AMP_OUTPUT_INT_RAW, 0x00},
	{MAX98522_AMP_A_THERMAL_WARN_INT_RAW, 0x00},
	{MAX98522_AMP_B_THERMAL_WARN_INT_RAW, 0x00},
	{MAX98522_BPE_AND_DHT_INT_RAW, 0x00},
	{MAX98522_SUPPLY_AND_OTP_INT_STATE, 0x00},
	{MAX98522_POWER_UP_AND_DOWN_INT_STATE, 0x07},
	{MAX98522_CLK_AND_DATA_INT_STATE, 0x00},
	{MAX98522_AMP_OUTPUT_FAULT_INT_STATE, 0x34},
	{MAX98522_AMP_A_THERMAL_WARN_INT_STATE, 0x00},
	{MAX98522_AMP_B_THERMAL_WARN_INT_STATE, 0x00},
	{MAX98522_BPE_AND_DHT_INT_STATE, 0x00},
	{MAX98522_SUPPLY_AND_OTP_INT_FLAG, 0x00},
	{MAX98522_POWER_UP_AND_DOWN_INT_FLAG, 0x00},
	{MAX98522_CLK_AND_DATA_INT_FLAG, 0x00},
	{MAX98522_AMP_OUTPUT_FAULT_INT_FLAG, 0x00},
	{MAX98522_AMP_A_THERMAL_INT_FLAG, 0x00},
	{MAX98522_AMP_B_THERMAL_INT_FLAG, 0x00},
	{MAX98522_BPE_AND_DHT_INT_FLAG, 0x00},
	{MAX98522_SUPPLY_AND_OTP_INT_EN, 0x01},
	{MAX98522_POWER_UP_AND_DOWN_INT_EN, 0x00},
	{MAX98522_CLK_AND_DATA_INT_EN, 0x00},
	{MAX98522_AMP_OUTPUT_FAULT_INT_EN, 0x00},
	{MAX98522_AMP_A_THERMAL_INT_EN, 0x00},
	{MAX98522_AMP_B_THERMAL_INT_EN, 0x00},
	{MAX98522_BPE_AND_DHT_INT_EN, 0x00},
	{MAX98522_SUPPLY_AND_OTP_INT_CLR, 0x00},
	{MAX98522_POWER_UP_AND_DOWN_INT_CLR, 0x00},
	{MAX98522_CLK_AND_DATA_INT_CLR, 0x00},
	{MAX98522_AMP_OUTPUT_FAULT_INT_CLR, 0x00},
	{MAX98522_AMP_A_THERMAL_INT_CLR, 0x00},
	{MAX98522_AMP_B_THERMAL_INT_CLR, 0x00},
	{MAX98522_BPE_AND_DHT_INT_CLR, 0x00},
	{MAX98522_DRIVE_STRENGTH_CTRL, 0x5D},
	{MAX98522_IRQ_CTRL, 0x00},
	{MAX98522_CLK_MONITOR_CTRL, 0x00},
	{MAX98522_SPK_MONITOR_THRESH, 0x03},
	{MAX98522_ENABLE_CTRLS, 0x03},
	{MAX98522_PCM_MODE_CFG, 0x84},
	{MAX98522_PCM_CLK_SETUP, 0x04},
	{MAX98522_PCM_SR_SET_1, 0x58},
	{MAX98522_PCM_TX_CTRL_12, 0x00},
	{MAX98522_PCM_TX_CTRL_07, 0x00},
	{MAX98522_PCM_TX_CTRL_08, 0x00},
	{MAX98522_PCM_TX_CTRL_09, 0x00},
	{MAX98522_PCM_TX_CTRL_10, 0x00},
	{MAX98522_PCM_TX_CTRL_11, 0x00},
	{MAX98522_PCM_TX_CTRL_06, 0x00},
	{MAX98522_PCM_TX_CTRL_01, 0x00},
	{MAX98522_PCM_TX_CTRL_02, 0x01},
	{MAX98522_PCM_TX_CTRL_03, 0x00},
	{MAX98522_PCM_TX_CTRL_04, 0x01},
	{MAX98522_PCM_TX_CTRL_05, 0x00},
	{MAX98522_PCM_TX_CTRL_13, 0x00},
	{MAX98522_PCM_TX_HIZ_CTRL_01, 0xFF},
	{MAX98522_PCM_TX_HIZ_CTRL_02, 0xFF},
	{MAX98522_PCM_TX_HIZ_CTRL_03, 0xFF},
	{MAX98522_PCM_TX_HIZ_CTRL_04, 0xFF},
	{MAX98522_PCM_TX_HIZ_CTRL_05, 0xFF},
	{MAX98522_PCM_TX_HIZ_CTRL_06, 0xFF},
	{MAX98522_PCM_TX_HIZ_CTRL_07, 0xFF},
	{MAX98522_PCM_TX_HIZ_CTRL_08, 0xFF},
	{MAX98522_PCM_INTERFACE_MODE, 0x00},
	{MAX98522_PCM_RX_SOURCE_01, 0x00},
	{MAX98522_PCM_RX_SOURCE_02, 0x10},
	{MAX98522_PCM_TX_SOURCE_ENABLES_1, 0x03},
	{MAX98522_PCM_TX_SOURCE_ENABLES_2, 0x03},
	{MAX98522_PCM_RX_ENABLES, 0x01},
	{MAX98522_PCM_TX_ENABLES, 0x01},
	{MAX98522_TONE_GEN_AND_DC_CFG, 0x04},
	{MAX98522_TONE_GEN_DC_LEVEL_1, 0x00},
	{MAX98522_TONE_GEN_DC_LEVEL_2, 0x00},
	{MAX98522_TONE_GEN_DC_LEVEL_3, 0x00},
	{MAX98522_TONE_GEN_CLK_CTRL, 0x00},
	{MAX98522_TONE_GEN_ENABLE, 0x00},
	{MAX98522_NOISE_GATE_MODE_CTRL, 0x32},
	{MAX98522_NOISE_GATE_MODE_EN, 0x00},
	{MAX98522_AMP_A_VOL_LVL_CTRL, 0x00},
	{MAX98522_AMP_B_VOL_LVL_CTRL, 0x00},
	{MAX98522_AMP_VOL_UPDATE_CTRL, 0x00},
	{MAX98522_AMP_A_VOL_RAMP_CTRL, 0x00},
	{MAX98522_AMP_A_PATH_GAIN, 0x0C},
	{MAX98522_AMP_A_DSP_CFG, 0x01},
	{MAX98522_AMP_A_CLIP_GAIN, 0x00},
	{MAX98522_AMP_A_CLASS_DG_THRESH, 0x0F},
	{MAX98522_AMP_A_SPK_MODE, 0x01},
	{MAX98522_AMP_A_SWITCH_FREQ, 0x00},
	{MAX98522_AMP_A_SPK_EDGE_HV, 0x0A},
	{MAX98522_AMP_A_SPK_EDGE_LV, 0xAA},
	{MAX98522_AMP_A_DAC_GAIN_AND_CURRENT_CFG, 0x03},
	{MAX98522_AMP_B_VOL_RAMP_CTRL, 0x00},
	{MAX98522_AMP_B_PATH_GAIN, 0x0C},
	{MAX98522_AMP_B_DSP_CFG, 0x01},
	{MAX98522_AMP_B_CLIP_GAIN, 0x00},
	{MAX98522_AMP_B_CLASS_DG_THRESH, 0x0F},
	{MAX98522_AMP_B_SPK_MODE, 0x01},
	{MAX98522_AMP_B_SWITCH_FREQ, 0x00},
	{MAX98522_AMP_B_SPK_EDGE_HV, 0x0A},
	{MAX98522_AMP_B_SPK_EDGE_LV, 0xAA},
	{MAX98522_AMP_B_DAC_GAIN_AND_CURRENT_CFG, 0x03},
	{MAX98522_SSM_CFG, 0x09},
	{MAX98522_SPK_PHASE_CTRL, 0x02},
	{MAX98522_AMP_EN, 0x03},
	{MAX98522_IV_SENSE_PATH_DSP_CFG, 0x0F},
	{MAX98522_IV_SENSE_PATH_ENABLES, 0x33},
	{MAX98522_MEAS_ADC_SAMPLE_RATE, 0xF0},
	{MAX98522_MEAS_ADC_OPTIMAL_MODE, 0x00},
	{MAX98522_MEAS_ADC_READBACK_CTRL_1, 0x00},
	{MAX98522_MEAS_ADC_READBACK_CTRL_2, 0x00},
	{MAX98522_MEAS_ADC_DITHER_CONTROL, 0x00},
	{MAX98522_MEAS_ADC_PVDD_CFG, 0x00},
	{MAX98522_MEAS_ADC_VBAT_CFG, 0x00},
	{MAX98522_MEAS_ADC_AMP_A_THERMAL_CFG, 0x00},
	{MAX98522_MEAS_ADC_AMP_B_THERMAL_CFG, 0x00},
	{MAX98522_MEAS_ADC_PVDD_READBACK_MSB, 0xCF},
	{MAX98522_MEAS_ADC_PVDD_READBACK_LSB, 0x00},
	{MAX98522_MEAS_ADC_VBAT_READBACK_MSB, 0xBB},
	{MAX98522_MEAS_ADC_VBAT_READBACK_LSB, 0x01},
	{MAX98522_MEAS_ADC_AMP_A_THERMAL_READBACK_MSB, 0x8C},
	{MAX98522_MEAS_ADC_AMP_A_THERMAL_READBACK_LSB, 0x01},
	{MAX98522_MEAS_ADC_AMP_B_THERMAL_READBACK_MSB, 0x8C},
	{MAX98522_MEAS_ADC_AMP_B_THERMAL_READBACK_LSB, 0x01},
	{MAX98522_MEAS_ADC_LOWEST_PVDD_READBACK_MSB, 0x88},
	{MAX98522_MEAS_ADC_LOWEST_PVDD_READBACK_LSB, 0x00},
	{MAX98522_MEAS_ADC_LOWEST_VBAT_READBACK_MSB, 0x99},
	{MAX98522_MEAS_ADC_LOWEST_VBAT_READBACK_LSB, 0x01},
	{MAX98522_MEAS_ADC_AMP_A_HIGHEST_THERMAL_READBACK_MSB, 0x8F},
	{MAX98522_MEAS_ADC_AMP_A_HIGHEST_THERMAL_READBACK_LSB, 0x01},
	{MAX98522_MEAS_ADC_AMP_B_HIGHEST_THERMAL_READBACK_MSB, 0x8F},
	{MAX98522_MEAS_ADC_AMP_B_HIGHEST_THERMAL_READBACK_LSB, 0x00},
	{MAX98522_MEAS_ADC_CFG, 0x03},
	{MAX98522_WARN_THRESH_1, 0x3C},
	{MAX98522_WARN_THRESH_2, 0x3C},
	{MAX98522_THERMAL_SHUTDOWN_THRESH, 0x48},
	{MAX98522_THERMAL_HYSTERESIS, 0x02},
	{MAX98522_THERMAL_FOLDBACK_SETTINGS, 0xC5},
	{MAX98522_THERMAL_FOLDBACK_ENABLE, 0x00},
	{MAX98522_BPE_STATE, 0x00},
	{MAX98522_BPE_L2_THRESH_MSB, 0x00},
	{MAX98522_BPE_L2_THRESH_LSB, 0x00},
	{MAX98522_BPE_L1_THRESH_MSB, 0x00},
	{MAX98522_BPE_L1_THRESH_LSB, 0x00},
	{MAX98522_BPE_L0_THRESH_MSB, 0x00},
	{MAX98522_BPE_L0_THRESH_LSB, 0x00},
	{MAX98522_BPE_L2_DWELL_AND_HOLD_TIME, 0x00},
	{MAX98522_BPE_L1_DWELL_AND_HOLD_TIME, 0x00},
	{MAX98522_BPE_L0_HOLD_TIME, 0x00},
	{MAX98522_BPE_L2_ATTACK_AND_RELEASE_STEP, 0x00},
	{MAX98522_BPE_L1_ATTACK_AND_RELEASE_STEP, 0x00},
	{MAX98522_BPE_L0_ATTACK_AND_RELEASE_STEP, 0x00},
	{MAX98522_BPE_L2_MAX_GAIN_ATTEN, 0x00},
	{MAX98522_BPE_L1_MAX_GAIN_ATTEN, 0x00},
	{MAX98522_BPE_L0_MAX_GAIN_ATTEN, 0x00},
	{MAX98522_BPE_L2_GAIN_ATTACK_AND_RELEASE_RATES, 0x00},
	{MAX98522_BPE_L1_GAIN_ATTACK_AND_RELEASE_RATES, 0x00},
	{MAX98522_BPE_L0_GAIN_ATTACK_AND_RELEASE_RATES, 0x00},
	{MAX98522_BPE_L2_LIMITER_CFG, 0x00},
	{MAX98522_BPE_L1_LIMITER_CFG, 0x00},
	{MAX98522_BPE_L0_LIMITER_CFG, 0x00},
	{MAX98522_BPE_L2_LIMITER_ATTACK_AND_RELEASE_RATES, 0x00},
	{MAX98522_BPE_L1_LIMITER_ATTACK_AND_RELEASE_RATES, 0x00},
	{MAX98522_BPE_L0_LIMITER_ATTACK_AND_RELEASE_RATES, 0x00},
	{MAX98522_BPE_THRESH_HYSTERESIS, 0x00},
	{MAX98522_BPE_INFINITE_HOLD_CLR, 0x00},
	{MAX98522_BPE_SUPPLY_SOURCE, 0x00},
	{MAX98522_BPE_LOWEST_STATE, 0x00},
	{MAX98522_BPE_LOWEST_GAIN, 0x00},
	{MAX98522_BPE_LOWEST_LIMITER, 0x00},
	{MAX98522_BPE_ENABLE, 0x00},
	{MAX98522_DHT_CFG_1, 0x00},
	{MAX98522_LIMITER_CFG_1, 0x08},
	{MAX98522_LIMITER_CFG_2, 0x00},
	{MAX98522_DHT_CFG_2, 0x14},
	{MAX98522_DHT_CFG_3, 0x02},
	{MAX98522_DHT_CFG_4, 0x04},
	{MAX98522_DHT_SUPPLY_HYSTERESIS_CFG, 0x07},
	{MAX98522_DRC_OFFSET_CFG, 0x00},
	{MAX98522_DHT_ENABLE, 0x00},
	{MAX98522_ET_LEVEL_STEP_SIZE_MSB, 0x0F},
	{MAX98522_ET_LEVEL_STEP_SIZE_LSB, 0x00},
	{MAX98522_ET_LEVEL_PVDD_STEP_SIZE_MSB, 0x64},
	{MAX98522_ET_LEVEL_PVDD_STEP_SIZE_LSB, 0x00},
	{MAX98522_MAX_DUTY_CYCLE_MSB, 0xF3},
	{MAX98522_MAX_DUTY_CYCLE_LSB, 0x01},
	{MAX98522_MIN_PVDD_LEVEL_MSB, 0x64},
	{MAX98522_MIN_PVDD_LEVEL_LSB, 0x00},
	{MAX98522_ET_PVDD_MAX_LEVELS, 0x70},
	{MAX98522_ET_PVDD_MIN_LEVEL, 0x14},
	{MAX98522_ET_PVDD_HEADROOM, 0x00},
	{MAX98522_ET_BYPASS_HEADROOM, 0x00},
	{MAX98522_ET_SCALABLE_HEADROOM, 0x04},
	{MAX98522_ET_DELAY_TIME, 0x30},
	{MAX98522_ET_LEVEL_HOLD_TIME, 0x5B},
	{MAX98522_ET_CONTROL, 0x00},
	{MAX98522_ET_PATH_GAIN_CH_A, 0x33},
	{MAX98522_ET_PATH_GAIN_CH_B, 0x33},
	{MAX98522_ET_PVDD_RESOLUTION, 0x00},
	{MAX98522_ET_ENABLE, 0x00},
	{MAX98522_GROUP_WRITE_ADDR_CTRL, 0x00},
	{MAX98522_DEVICE_RAMP_CTRL, 0x00},
	{MAX98522_AUTO_RESTART_BEHAVIOR, 0x10},
	{MAX98522_GLOBAL_ENABLE, 0x00},
};

static int max98522_dai_set_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);
	unsigned int format;
	unsigned int invert = 0;

	dev_dbg(component->dev, "%s: fmt 0x%08X\n", __func__, fmt);

	/* Only slave mode is supported */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		max98522->provider = false;
		break;
	default:
		dev_err(component->dev, "DAI clock mode unsupported\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		invert = MAX98522_PCM_CLK_SETUP_PCM_BCLKEDGE;
		break;
	default:
		dev_err(component->dev, "DAI invert mode unsupported\n");
		return -EINVAL;
	}

	regmap_update_bits(max98522->regmap,
		MAX98522_PCM_CLK_SETUP,
		MAX98522_PCM_CLK_SETUP_PCM_BCLKEDGE,
		invert);

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format = MAX98522_PCM_MODE_CFG_PCM_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format = MAX98522_PCM_MODE_CFG_PCM_FORMAT_LJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		format = MAX98522_PCM_MODE_CFG_PCM_FORMAT_TDM_MODE_1;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		format = MAX98522_PCM_MODE_CFG_PCM_FORMAT_TDM_MODE_2;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(max98522->regmap,
		MAX98522_PCM_MODE_CFG,
		MAX98522_PCM_MODE_CFG_PCM_FORMAT_MASK,
		format);

	return 0;
}

static int max98522_get_bclk_sel(int bclk)
{
	int i;

	/* BCLKs per LRCLK */
	static int bclk_sel_table[] = {
		32, 48, 64, 96, 128, 192, 256, 384, 512, 320, 250, 125,
	};

	/* match BCLKs per LRCL41*/
	for (i = 0; i < ARRAY_SIZE(bclk_sel_table); i++) {
		if (bclk_sel_table[i] == bclk)
			return i + 2;
	}
	return 0;
}

static int max98522_set_clock(struct snd_soc_component *component,
		struct snd_pcm_hw_params *params)
{
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);

	/* BCLK/LRCLK ratio calculation */
	int blr_clk_ratio = params_channels(params)
		* snd_pcm_format_width(params_format(params));
	int value;
	if (!max98522->tdm_mode) {
		/* BCLK configuration */
		value =    (blr_clk_ratio);
		if (!value) {
			dev_err(component->dev, "format unsupported %d\n",
				params_format(params));
			return -EINVAL;
		}

		regmap_update_bits(max98522->regmap,
			MAX98522_PCM_CLK_SETUP,
			MAX98522_PCM_CLK_SETUP_BSEL_MASK,
			value);
	}
	return 0;
}

static int max98522_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component =
		dai->component;
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);
	unsigned int sampling_rate;
	unsigned int chan_sz;

	/* pcm mode configuration */
	switch (snd_pcm_format_width(params_format(params))) {
	case 16:
		chan_sz = MAX98522_PCM_MODE_CFG_PCM_CHANSZ_16;
		break;
	case 24:
		chan_sz = MAX98522_PCM_MODE_CFG_PCM_CHANSZ_24;
		break;
	case 32:
		chan_sz = MAX98522_PCM_MODE_CFG_PCM_CHANSZ_32;
		break;
	default:
		dev_err(component->dev, "format unsupported %d\n",
			params_format(params));
		goto err;
	}

	regmap_update_bits(max98522->regmap,
		MAX98522_PCM_MODE_CFG,
		MAX98522_PCM_MODE_CFG_PCM_CHANSZ_MASK, chan_sz);

	dev_info(component->dev, "format supported %d",
		params_format(params));
	dev_info(component->dev, "channels %d",
		params_channels(params));

	/* sampling rate configuration */
	switch (params_rate(params)) {
	case 8000:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_8000;
		break;
	case 11025:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_11025;
		break;
	case 12000:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_12000;
		break;
	case 16000:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_16000;
		break;
	case 22050:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_22050;
		break;
	case 24000:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_24000;
		break;
	case 32000:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_32000;
		break;
	case 44100:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_44100;
		break;
	case 48000:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_48000;
		break;
	case 88200:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_88200;
		break;
	case 96000:
		sampling_rate = MAX98522_PCM_SR_SET_1_SR_96000;
		break;
	default:
		dev_err(component->dev, "rate %d not supported\n",
			params_rate(params));
		goto err;
	}

	/* set DAI_SR to correct LRCLK frequency */
	regmap_update_bits(max98522->regmap,
		MAX98522_PCM_SR_SET_1,
		MAX98522_PCM_SR_SET_1_SR_MASK,
		sampling_rate);
	dev_info(component->dev, "rate supported %d",
		params_rate(params));

	return max98522_set_clock(component, params);

err:
	return -EINVAL;
}

static int max98522_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);

	int bsel;
	unsigned int chan_sz;

	if (!tx_mask && !rx_mask && !slots && !slot_width)
		max98522->tdm_mode = false;
	else
		max98522->tdm_mode = true;

	dev_dbg(component->dev,
		"tdm mode : %d\n", max98522->tdm_mode);

	/* BCLK configuration */
	bsel = max98522_get_bclk_sel(slots * slot_width);
	if (!bsel) {
		dev_err(component->dev, "BCLK %d not supported\n",
			slots * slot_width);
		return -EINVAL;
	}

	regmap_update_bits(max98522->regmap,
		MAX98522_PCM_CLK_SETUP,
		MAX98522_PCM_CLK_SETUP_BSEL_MASK,
		bsel);

	/* Channel size configuration */
	switch (slot_width) {
	case 16:
		chan_sz = MAX98522_PCM_MODE_CFG_PCM_CHANSZ_16;
		break;
	case 24:
		chan_sz = MAX98522_PCM_MODE_CFG_PCM_CHANSZ_24;
		break;
	case 32:
		chan_sz = MAX98522_PCM_MODE_CFG_PCM_CHANSZ_32;
		break;
	default:
		dev_err(component->dev, "format unsupported %d\n",
			slot_width);
		return -EINVAL;
	}

	regmap_update_bits(max98522->regmap,
		MAX98522_PCM_MODE_CFG,
		MAX98522_PCM_MODE_CFG_PCM_CHANSZ_MASK, chan_sz);

	return 0;
}

static int max98522_dai_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);

	max98522->sysclk = freq;
	return 0;
}

static const struct snd_soc_dai_ops max98522_dai_ops = {
	.set_sysclk = max98522_dai_set_sysclk,
	.set_fmt = max98522_dai_set_fmt,
	.hw_params = max98522_dai_hw_params,
	.set_tdm_slot = max98522_dai_tdm_slot,
};

static int max98522_dac_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(max98522->regmap,
			MAX98522_GLOBAL_ENABLE,
			MAX98522_GLOBAL_EN_MASK, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(max98522->regmap,
			MAX98522_GLOBAL_ENABLE,
			MAX98522_GLOBAL_EN_MASK, 0);
		break;
	}

	return 0;
}

static DECLARE_TLV_DB_SCALE(max98522_amp_a_digital_tlv, -9000, 50, 1);
static DECLARE_TLV_DB_SCALE(max98522_amp_b_digital_tlv, -9000, 50, 1);

static const struct snd_kcontrol_new max98522_snd_controls[] = {
	SOC_SINGLE_TLV("AMP A Digital Volume", MAX98522_AMP_A_VOL_LVL_CTRL,
		0, 180, 1,
		max98522_amp_a_digital_tlv),
	SOC_SINGLE_TLV("AMP B Digital Volume", MAX98522_AMP_B_VOL_LVL_CTRL,
		0, 180, 1,
		max98522_amp_b_digital_tlv),
};

static const struct snd_soc_dapm_widget max98522_dapm_widgets[] = {
	SND_SOC_DAPM_DAC_E("Amp Enable", "HiFi Playback",
		SND_SOC_NOPM, 0, 0, max98522_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("BE_OUT"),
};

static const struct snd_soc_dapm_route max98522_audio_map[] = {
	/* Playback */
	{"BE_OUT", NULL, "Amp Enable"},
};

static bool max98522_readable_register(struct device *dev, unsigned int reg)
{
	return true;
	// switch (reg) {
	// case MAX98522_SOFTWARE_RESET ... MAX98522_INT_EN3:
	// case MAX98522_IRQ_CTRL ... MAX98522_WDOG_CTRL:
	// case MAX98522_MEAS_ADC_THERM_WARN_THRESH
	// 	... MAX98522_BROWNOUT_INFINITE_HOLD:
	// case MAX98522_BROWNOUT_LVL_HOLD ... DSMIG_DEBUZZER_THRESHOLD:
	// case DSM_VOL_ENA ... MAX98522_R24FF_REV_ID:
	// 	return true;
	// default:
	// 	return false;
	// }
}

static bool max98522_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
	// switch (reg) {
	// case MAX98522_SOFTWARE_RESET ... MAX98522_INT_EN3:
	// case MAX98522_MEAS_ADC_CH0_READ ... MAX98522_MEAS_ADC_CH2_READ:
	// case MAX98522_PWR_GATE_STATUS ... MAX98522_BROWNOUT_STATUS:
	// case MAX98522_BROWNOUT_LOWEST_STATUS:
	// case MAX98522_ENV_TRACK_BOOST_VOUT_READ:
	// case DSM_STBASS_HPF_B0_BYTE0 ... DSM_DEBUZZER_ATTACK_TIME_BYTE2:
	// case THERMAL_RDC_RD_BACK_BYTE1 ... DSMIG_DEBUZZER_THRESHOLD:
	// case DSM_THERMAL_GAIN ... DSM_WBDRC_GAIN:
	// 	return true;
	// default:
	// 	return false;
	// }
}

#define MAX98522_RATES SNDRV_PCM_RATE_8000_48000
#define MAX98522_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver max98522_dai[] = {
	{
		.name = "max98522-aif1",
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 16,
			.rates = MAX98522_RATES,
			.formats = MAX98522_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = MAX98522_RATES,
			.formats = MAX98522_FORMATS,
		},
		.ops = &max98522_dai_ops,
	}
};

static void max98522_init_regs(struct snd_soc_component *component)
{
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);

	regmap_write(max98522->regmap, MAX98522_CLK_MONITOR_CTRL, 0x37);
	regmap_write(max98522->regmap, MAX98522_SPK_MONITOR_THRESH, 0x03);
	regmap_write(max98522->regmap, MAX98522_ENABLE_CTRLS, 0x03);
}

static int max98522_probe(struct snd_soc_component *component)
{
	struct max98522_priv *max98522 =
		snd_soc_component_get_drvdata(component);

	/* SW Reset with settle time */
	regmap_write(max98522->regmap, MAX98522_SOFTWARE_RESET, 0x01);
	msleep(20);

	/* Amp init setting */
	max98522_init_regs(component);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max98522_suspend(struct device *dev)
{
	struct max98522_priv *max98522 = dev_get_drvdata(dev);

	dev_dbg(dev, "%s:Enter\n", __func__);

	regcache_cache_only(max98522->regmap, true);
	regcache_mark_dirty(max98522->regmap);

	return 0;
}

static int max98522_resume(struct device *dev)
{
	struct max98522_priv *max98522 = dev_get_drvdata(dev);

	dev_dbg(dev, "%s:Enter\n", __func__);

	regcache_cache_only(max98522->regmap, false);
	regcache_sync(max98522->regmap);

	return 0;
}
#endif

static const struct dev_pm_ops max98522_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(max98522_suspend, max98522_resume)
};

static const struct snd_soc_component_driver soc_codec_dev_max98522 = {
	.probe = max98522_probe,
	.controls = max98522_snd_controls,
	.num_controls = ARRAY_SIZE(max98522_snd_controls),
	.dapm_widgets = max98522_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(max98522_dapm_widgets),
	.dapm_routes = max98522_audio_map,
	.num_dapm_routes = ARRAY_SIZE(max98522_audio_map),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
};


static const struct regmap_config max98522_regmap = {
	.reg_bits         = 16,
	.val_bits         = 8,
	.max_register     = MAX98522_R3FFF_REV_ID,
	.reg_defaults     = max98522_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(max98522_reg_defaults),
	.readable_reg	  = max98522_readable_register,
	.volatile_reg	  = max98522_volatile_reg,
	.cache_type       = REGCACHE_RBTREE,
};

static int max98522_i2c_probe(struct i2c_client *i2c)
{
	int ret = 0;
	int reg = 0;

	struct max98522_priv *max98522 = NULL;
	struct i2c_adapter *adapter = i2c->adapter;
	struct gpio_desc *reset_gpio;

	ret = i2c_check_functionality(adapter,
		I2C_FUNC_SMBUS_BYTE
		| I2C_FUNC_SMBUS_BYTE_DATA);
	if (!ret) {
		dev_err(&i2c->dev, "I2C check functionality failed\n");
		return -ENXIO;
	}

	max98522 = devm_kzalloc(&i2c->dev, sizeof(*max98522), GFP_KERNEL);
	if (!max98522) {
		ret = -ENOMEM;
		return ret;
	}
	i2c_set_clientdata(i2c, max98522);

	/* regmap initialization */
	max98522->regmap = devm_regmap_init_i2c(i2c, &max98522_regmap);
	if (IS_ERR(max98522->regmap)) {
		ret = PTR_ERR(max98522->regmap);
		dev_err(&i2c->dev,
			"Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	reset_gpio = devm_gpiod_get_optional(&i2c->dev,
					     "reset", GPIOD_OUT_HIGH);

	/* Power on device */
	if (reset_gpio) {
		usleep_range(1000, 2000);
		/* bring out of reset */
		gpiod_set_value_cansleep(reset_gpio, 0);
		usleep_range(1000, 2000);
	}

	/* Check Revision ID */
	ret = regmap_read(max98522->regmap,
		MAX98522_R3FFF_REV_ID, &reg);
	if (ret) {
		dev_err(&i2c->dev,
			"ret=%d, Failed to read: 0x%02X\n",
			ret, MAX98522_R3FFF_REV_ID);
		return ret;
	}
	dev_info(&i2c->dev, "MAX98522 revisionID: 0x%02X\n", reg);

	ret = devm_snd_soc_register_component(&i2c->dev,
			&soc_codec_dev_max98522,
			max98522_dai, ARRAY_SIZE(max98522_dai));

	return ret;
}

static const struct i2c_device_id max98522_i2c_id[] = {
	{ "max98522"},
	{},
};
MODULE_DEVICE_TABLE(i2c, max98522_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id max98522_of_match[] = {
	{ .compatible = "adi,max98522", },
	{}
};
MODULE_DEVICE_TABLE(of, max98522_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id max98522_acpi_match[] = {
	{ "MX98522", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, max98522_acpi_match);
#endif

static struct i2c_driver max98522_i2c_driver = {
	.driver = {
		.name = "max98522",
		.of_match_table = of_match_ptr(max98522_of_match),
		.acpi_match_table = ACPI_PTR(max98522_acpi_match),
		.pm = &max98522_pm,
	},
	.probe = max98522_i2c_probe,
	.id_table = max98522_i2c_id,
};

module_i2c_driver(max98522_i2c_driver)

MODULE_DESCRIPTION("ALSA SoC MAX98522 driver");
MODULE_AUTHOR("Mingu Hwang <mingu.hwang@analog.com>");
MODULE_LICENSE("GPL");
