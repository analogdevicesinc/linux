// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices adau1962 codec driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "adau1962.h"

#define __DEBUG 0

#define ADAU1962_REG_PLL_CLK_CTRL0		0x00
#define ADAU1962_REG_PLL_CLK_CTRL1		0x01
#define ADAU1962_REG_PDN_THRMSENS_CTRL1		0x02
#define ADAU1962_REG_PDN_CTRL2			0x03
#define ADAU1962_REG_PDN_CTRL3			0x04
#define ADAU1962_REG_THRM_TEMP_STAT		0x05
#define ADAU1962_REG_DAC_CTRL0			0x06
#define ADAU1962_REG_DAC_CTRL1			0x07
#define ADAU1962_REG_DAC_CTRL2			0x08
#define ADAU1962_REG_DAC_MUTE1			0x09
#define ADAU1962_REG_DAC_MUTE2			0x0a
#define ADAU1962_REG_DACMSTR_VOL		0x0b
#define ADAU1962_REG_DAC_VOL(x)			(0x0b + (x))
#define ADAU1962_REG_PAD_STRGTH			0x1c
#define ADAU1962_REG_DAC_POWER1			0x1d
#define ADAU1962_REG_DAC_POWER2			0x1e
#define ADAU1962_REG_DAC_POWER3			0x1f

#define ADAU1962_PLL_CLK_PUP			BIT(0)
#define ADAU1962_PLL_CLK_DLRCLK			BIT(6)
#define ADAU1962_PLL_CLK_PLLIN_MASK		(0x3 << 6)
#define ADAU1962_PLL_MCS_MASK			(0x3 << 1)

#define ADAU1962_DAC_CTRL0_MMUTE		BIT(0)

#define ADAU1962_DAC_CTRL0_FMT_MASK		(0x3 << 6)
#define ADAU1962_DAC_CTRL0_FMT_I2S		(0x0 << 6)
#define ADAU1962_DAC_CTRL0_FMT_LJ		(0x1 << 6)
#define ADAU1962_DAC_CTRL0_FMT_RJ_24BIT		(0x2 << 6)
#define ADAU1962_DAC_CTRL0_FMT_RJ_16BIT		(0x3 << 6)

#define ADAU1962_SAI_CTRL0_SAI_MASK		(0x7 << 3)
#define ADAU1962_SAI_CTRL0_SAI_I2S		(0x0 << 3)
#define ADAU1962_SAI_CTRL0_SAI_TDM_2		(0x1 << 3)
#define ADAU1962_SAI_CTRL0_SAI_TDM_4		(0x2 << 3)
#define ADAU1962_SAI_CTRL0_SAI_TDM_8		(0x3 << 3)
#define ADAU1962_SAI_CTRL0_SAI_TDM_16		(0x4 << 3)

#define ADAU1962_DAC_CTRL0_FS_MASK		(0x3)
#define ADAU1962_DAC_CTRL0_FS_32000_48000	(0x0)
#define ADAU1962_DAC_CTRL0_FS_64000_96000	(0x1)
#define ADAU1962_DAC_CTRL0_FS_128000_192000	(0x2)

#define ADAU1962_DAC_CTRL1_LRCLK_PULSE		BIT(6)
#define ADAU1962_DAC_CTRL1_LRCLK_POL		BIT(5)
#define ADAU1962_DAC_CTRL1_MSB			BIT(4)
#define ADAU1962_DAC_CTRL1_BCLKRATE_16		(0x1 << 2)
#define ADAU1962_DAC_CTRL1_BCLKRATE_32		(0x0 << 2)
#define ADAU1962_DAC_CTRL1_BCLKRATE_MASK	(0x1 << 2)
#define ADAU1962_DAC_CTRL1_BCLK_EDGE		BIT(1)
#define ADAU1962_DAC_CTRL1_MASTER		BIT(0)

#define ADAU1962_DAC_CTRL2_SLOT_WIDTH_MASK	(0x1 << 4)
#define ADAU1962_DAC_CTRL2_SLOT_WIDTH_32	(0x0 << 4)
#define ADAU1962_DAC_CTRL2_SLOT_WIDTH_16	(0x1 << 4)

#define ADAU1962_CHAN_MAP_SECOND_SLOT_OFFSET	4
#define ADAU1962_CHAN_MAP_FIRST_SLOT_OFFSET	0

struct adau1962 {
	struct regmap *regmap;
	bool right_j;
	unsigned int sysclk;
	enum adau1962_sysclk_src sysclk_src;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;

	struct snd_pcm_hw_constraint_list constraints;

	struct device *dev;
	void (*switch_mode)(struct device *dev);

	unsigned int max_master_fs;
	unsigned int slot_width;
	bool enabled;
	bool master;
};

static const struct reg_default adau1962_reg_defaults[] = {
	{ 0x00, 0x00 },
	{ 0x01, 0x2a },
	{ 0x02, 0xa0 },
	{ 0x03, 0x00 },
	{ 0x04, 0x00 },
	{ 0x05, 0x00 },
	{ 0x06, 0x01 },
	{ 0x07, 0x00 },
	{ 0x08, 0x06 },
	{ 0x09, 0x00 },
	{ 0x0a, 0x00 },
	{ 0x0b, 0x00 },
	{ 0x0c, 0x00 },
	{ 0x0d, 0x00 },
	{ 0x0e, 0x00 },
	{ 0x0f, 0x00 },
	{ 0x10, 0x00 },
	{ 0x11, 0x00 },
	{ 0x12, 0x00 },
	{ 0x13, 0x00 },
	{ 0x14, 0x00 },
	{ 0x15, 0x00 },
	{ 0x16, 0x00 },
	{ 0x17, 0x00 },
	{ 0x1c, 0x00 },
	{ 0x1d, 0xaa },
	{ 0x1e, 0xaa },
	{ 0x1f, 0xaa },
};

static const DECLARE_TLV_DB_MINMAX_MUTE(adau1962_adc_gain, -9562, 0);

#define ADAU1962_OUTPUT(x) \
	SND_SOC_DAPM_OUTPUT("AOUT" #x)
#define ADAU1962_DAC1(x) \
	SND_SOC_DAPM_DAC("DAC" #x, "Playback", ADAU1962_REG_PDN_CTRL2, \
			(x - 1), 1)
#define ADAU1962_DAC2(x) \
	SND_SOC_DAPM_DAC("DAC" #x, "Playback", ADAU1962_REG_PDN_CTRL3, \
			(x - 9), 1)

static const struct snd_soc_dapm_widget adau1962_dapm_widgets[] = {
	ADAU1962_DAC1(1),
	ADAU1962_DAC1(2),
	ADAU1962_DAC1(3),
	ADAU1962_DAC1(4),
	ADAU1962_DAC1(5),
	ADAU1962_DAC1(6),
	ADAU1962_DAC1(7),
	ADAU1962_DAC1(8),
	ADAU1962_DAC2(9),
	ADAU1962_DAC2(10),
	ADAU1962_DAC2(11),
	ADAU1962_DAC2(12),

	ADAU1962_OUTPUT(1),
	ADAU1962_OUTPUT(2),
	ADAU1962_OUTPUT(3),
	ADAU1962_OUTPUT(4),
	ADAU1962_OUTPUT(5),
	ADAU1962_OUTPUT(6),
	ADAU1962_OUTPUT(7),
	ADAU1962_OUTPUT(8),
	ADAU1962_OUTPUT(9),
	ADAU1962_OUTPUT(10),
	ADAU1962_OUTPUT(11),
	ADAU1962_OUTPUT(12),
};

#define ADAU1962_ROUTE(x) \
	{ "AOUT" #x, NULL, "DAC" #x }

static const struct snd_soc_dapm_route adau1962_dapm_routes[] = {
	ADAU1962_ROUTE(1),
	ADAU1962_ROUTE(2),
	ADAU1962_ROUTE(3),
	ADAU1962_ROUTE(4),
	ADAU1962_ROUTE(5),
	ADAU1962_ROUTE(6),
	ADAU1962_ROUTE(7),
	ADAU1962_ROUTE(8),
	ADAU1962_ROUTE(9),
	ADAU1962_ROUTE(10),
	ADAU1962_ROUTE(11),
	ADAU1962_ROUTE(12),
};

#define ADAU1962_VOLUME(x) \
	SOC_SINGLE_TLV("DAC" #x " Playback Volume", \
		ADAU1962_REG_DAC_VOL(x), \
		0, 255, 1, adau1962_adc_gain)
#define ADAU1962_PLAYBACK_SWITCH1(x) \
	SOC_SINGLE("DAC" #x " Playback Switch", \
		ADAU1962_REG_DAC_MUTE1, ((x) - 1), \
		1, 1)
#define ADAU1962_PLAYBACK_SWITCH2(x) \
	SOC_SINGLE("DAC" #x " Playback Switch", \
		ADAU1962_REG_DAC_MUTE2, ((x) - 9), \
		1, 1)
static const char *adau1962_dac_power[] = {"Low Power", "Lowest Power",
	"Best Performance", "Good Performance"};
static const char *adau1962_dac_osr[] = {"256x", "128x"};
static const struct soc_enum adau1962_enum[] = {
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_CTRL2, 1, 2, adau1962_dac_osr),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER1, 0, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER1, 2, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER1, 4, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER1, 6, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER2, 0, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER2, 2, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER2, 4, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER2, 6, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER3, 0, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER3, 2, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER3, 4, 4, adau1962_dac_power),
	SOC_ENUM_SINGLE(ADAU1962_REG_DAC_POWER3, 6, 4, adau1962_dac_power),
};
#define ADAU1962_DAC_POWER(x) \
	SOC_ENUM("DAC" #x " Power Adjust", adau1962_enum[(x)])

static const struct snd_kcontrol_new adau1962_snd_controls[] = {
	/* global DAC playback controls */
	SOC_SINGLE_TLV("DAC Playback Volume", ADAU1962_REG_DACMSTR_VOL,
			0, 255, 1, adau1962_adc_gain),
	SOC_SINGLE("DAC Playback Switch", ADAU1962_REG_DAC_CTRL0, 0, 1, 1),
	SOC_SINGLE("DAC Deemphasis Switch", ADAU1962_REG_DAC_CTRL2, 0, 1, 0),

	/* DAC1-12 specific controls */
	ADAU1962_VOLUME(1),
	ADAU1962_PLAYBACK_SWITCH1(1),
	ADAU1962_DAC_POWER(1),
	ADAU1962_VOLUME(2),
	ADAU1962_PLAYBACK_SWITCH1(2),
	ADAU1962_DAC_POWER(2),
	ADAU1962_VOLUME(3),
	ADAU1962_PLAYBACK_SWITCH1(3),
	ADAU1962_DAC_POWER(3),
	ADAU1962_VOLUME(4),
	ADAU1962_PLAYBACK_SWITCH1(4),
	ADAU1962_DAC_POWER(4),
	ADAU1962_VOLUME(5),
	ADAU1962_PLAYBACK_SWITCH1(5),
	ADAU1962_DAC_POWER(5),
	ADAU1962_VOLUME(6),
	ADAU1962_PLAYBACK_SWITCH1(6),
	ADAU1962_DAC_POWER(6),
	ADAU1962_VOLUME(7),
	ADAU1962_PLAYBACK_SWITCH1(7),
	ADAU1962_DAC_POWER(7),
	ADAU1962_VOLUME(8),
	ADAU1962_PLAYBACK_SWITCH1(8),
	ADAU1962_DAC_POWER(8),
	ADAU1962_VOLUME(9),
	ADAU1962_PLAYBACK_SWITCH2(9),
	ADAU1962_DAC_POWER(9),
	ADAU1962_VOLUME(10),
	ADAU1962_PLAYBACK_SWITCH2(10),
	ADAU1962_DAC_POWER(10),
	ADAU1962_VOLUME(11),
	ADAU1962_PLAYBACK_SWITCH2(11),
	ADAU1962_DAC_POWER(11),
	ADAU1962_VOLUME(12),
	ADAU1962_PLAYBACK_SWITCH2(12),
	ADAU1962_DAC_POWER(12),

	/* other controls */
	SOC_ENUM("DAC Oversampling Rate", adau1962_enum[0]),
};

#if __DEBUG
void adau1962_print(struct adau1962 *adau1962)
{
	int i;
	unsigned int val[4];

	for (i = 0; i < 32;) {
		regmap_read(adau1962->regmap, i, &val[0]);
		regmap_read(adau1962->regmap, i+1, &val[1]);
		regmap_read(adau1962->regmap, i+2, &val[2]);
		regmap_read(adau1962->regmap, i+3, &val[3]);
		pr_info("%02x,%02x,%02x,%02x\n", val[0], val[1], val[2], val[3]);
		i += 4;
		/* skip address 0x18-0x1B */
		if (i == 24)
			i += 4;
	}
}
#endif

/*
 * Returns the appropriate setting for ths FS field in the CTRL0 register
 * depending on the rate.
 */
static int adau1962_lookup_fs(unsigned int rate)
{
	if (rate >= 32000 && rate <= 48000)
		return ADAU1962_DAC_CTRL0_FS_32000_48000;
	else if (rate >= 64000 && rate <= 96000)
		return ADAU1962_DAC_CTRL0_FS_64000_96000;
	else if (rate >= 128000 && rate <= 192000)
		return ADAU1962_DAC_CTRL0_FS_128000_192000;
	else
		return -EINVAL;
}

static int adau1962_lookup_mcs(struct adau1962 *adau1962, unsigned int rate,
	unsigned int fs)
{
	unsigned int mcs;

	rate *= 128 >> fs;

	if (adau1962->sysclk % rate != 0)
		return -EINVAL;

	mcs = adau1962->sysclk / rate;

	/* The factors configured by MCS are 2, 3, 4, 6 */
	if (mcs < 2 || mcs > 6 || mcs == 5)
		return -EINVAL;

	mcs = mcs - 2;
	if (mcs == 4)
		mcs = 3;

	return mcs;
}

static int adau1962_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct adau1962 *adau1962 = snd_soc_component_get_drvdata(component);
	unsigned int rate = params_rate(params);
	unsigned int slot_width;
	unsigned int ctrl0, ctrl0_mask;
	unsigned int ctrl1;
	int mcs, fs;
	int ret;

	fs = adau1962_lookup_fs(rate);
	if (fs < 0)
		return fs;

	if (adau1962->sysclk_src == ADAU1962_SYSCLK_SRC_MCLK) {
		mcs = adau1962_lookup_mcs(adau1962, rate, fs);
		if (mcs < 0)
			return mcs;
	} else {
		mcs = 0;
	}

	ctrl0_mask = ADAU1962_DAC_CTRL0_FS_MASK;
	ctrl0 = fs << 1;

	if (adau1962->right_j) {
		switch (params_width(params)) {
		case 16:
			ctrl0 |= ADAU1962_DAC_CTRL0_FMT_RJ_16BIT;
			break;
		case 24:
			ctrl0 |= ADAU1962_DAC_CTRL0_FMT_RJ_24BIT;
			break;
		default:
			return -EINVAL;
		}
		ctrl0_mask |= ADAU1962_DAC_CTRL0_FMT_MASK;
	}

	if (adau1962->master) {
		switch (params_width(params)) {
		case 16:
			slot_width = 16;
			break;
		case 24:
		case 32:
			slot_width = 32;
			break;
		default:
			return -EINVAL;
		}

		/* In TDM mode there is a fixed slot width */
		if (adau1962->slot_width)
			slot_width = adau1962->slot_width;

		if (slot_width == 16)
			ctrl1 = ADAU1962_DAC_CTRL1_BCLKRATE_16;
		else
			ctrl1 = ADAU1962_DAC_CTRL1_BCLKRATE_32;

		ret = regmap_update_bits(adau1962->regmap,
			ADAU1962_REG_DAC_CTRL1,
			ADAU1962_DAC_CTRL1_BCLKRATE_MASK,
			ctrl1);
		if (ret < 0)
			return ret;
	}

	ret = regmap_update_bits(adau1962->regmap, ADAU1962_REG_DAC_CTRL0,
				ctrl0_mask, ctrl0);
	if (ret < 0)
		return ret;

	return regmap_update_bits(adau1962->regmap, ADAU1962_REG_PLL_CLK_CTRL0,
				ADAU1962_PLL_MCS_MASK, mcs << 1);
}

static int adau1962_set_bias_level(struct snd_soc_component *component,
	enum snd_soc_bias_level level)
{
//	struct adau1962 *adau1962 = snd_soc_component_get_drvdata(component);
//	int ret = 0;

//	switch (level) {
//	case SND_SOC_BIAS_ON:
//		break;
//	case SND_SOC_BIAS_PREPARE:
//		break;
//	case SND_SOC_BIAS_STANDBY:
//		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_OFF)
//			ret = adau1962_power_enable(adau1962);
//		break;
//	case SND_SOC_BIAS_OFF:
//		ret = adau1962_power_disable(adau1962);
//		break;
//	}

//	if (ret)
//		return ret;

	return 0;
}

static int adau1962_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int width)
{
	struct adau1962 *adau1962 = snd_soc_component_get_drvdata(dai->component);
	unsigned int ctrl0, ctrl1, ctrl2;
	int ret;

	if (slots == 0) {
		/* 0 = No fixed slot width */
		adau1962->slot_width = 0;
		adau1962->max_master_fs = 192000;
		return regmap_update_bits(adau1962->regmap,
			ADAU1962_REG_DAC_CTRL0, ADAU1962_SAI_CTRL0_SAI_MASK,
			ADAU1962_SAI_CTRL0_SAI_I2S);
	}

	if (rx_mask == 0 || tx_mask != 0)
		return -EINVAL;

	switch (slots) {
	case 2:
		ctrl0 = ADAU1962_SAI_CTRL0_SAI_TDM_2;
		break;
	case 4:
		ctrl0 = ADAU1962_SAI_CTRL0_SAI_TDM_4;
		break;
	case 8:
		ctrl0 = ADAU1962_SAI_CTRL0_SAI_TDM_8;
		break;
	case 16:
		ctrl0 = ADAU1962_SAI_CTRL0_SAI_TDM_16;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(adau1962->regmap, ADAU1962_REG_DAC_CTRL0,
		ADAU1962_SAI_CTRL0_SAI_MASK, ctrl0);
	if (ret)
		return ret;

	switch (width) {
	case 16:
		ctrl1 = ADAU1962_DAC_CTRL1_BCLKRATE_16;
		ctrl2 = ADAU1962_DAC_CTRL2_SLOT_WIDTH_16;
		break;
	case 24:
	case 32:
		ctrl1 = ADAU1962_DAC_CTRL1_BCLKRATE_32;
		ctrl2 = ADAU1962_DAC_CTRL2_SLOT_WIDTH_32;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(adau1962->regmap,
		ADAU1962_REG_DAC_CTRL1,
		ADAU1962_DAC_CTRL1_BCLKRATE_MASK,
		ctrl1);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(adau1962->regmap,
		ADAU1962_REG_DAC_CTRL2,
		ADAU1962_DAC_CTRL2_SLOT_WIDTH_MASK,
		ctrl2);
	if (ret < 0)
		return ret;

	adau1962->slot_width = width;

	/* In master mode the maximum bitclock is 24.576 MHz */
	adau1962->max_master_fs = min(192000, 24576000 / width / slots);
	return 0;
}

static int adau1962_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct adau1962 *adau1962 = snd_soc_component_get_drvdata(dai->component);
	unsigned int val;

	if (mute)
		val = ADAU1962_DAC_CTRL0_MMUTE;
	else
		val = 0;

	return regmap_update_bits(adau1962->regmap, ADAU1962_REG_DAC_CTRL0,
			ADAU1962_DAC_CTRL0_MMUTE, val);
}

static int adau1962_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct adau1962 *adau1962 = snd_soc_component_get_drvdata(dai->component);
	unsigned int ctrl0 = 0, ctrl1 = 0;
	bool invert_lrclk;
	int ret;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		adau1962->master = false;
		break;
	case SND_SOC_DAIFMT_CBP_CFP:
		ctrl1 |= ADAU1962_DAC_CTRL1_MASTER;
		adau1962->master = true;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		invert_lrclk = false;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1 |= ADAU1962_DAC_CTRL1_BCLK_EDGE;
		invert_lrclk = false;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		invert_lrclk = true;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		ctrl1 |= ADAU1962_DAC_CTRL1_BCLK_EDGE;
		invert_lrclk = true;
		break;
	default:
		return -EINVAL;
	}

	adau1962->right_j = false;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl0 |= ADAU1962_DAC_CTRL0_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl0 |= ADAU1962_DAC_CTRL0_FMT_LJ;
		invert_lrclk = !invert_lrclk;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl0 |= ADAU1962_DAC_CTRL0_FMT_RJ_24BIT;
		adau1962->right_j = true;
		invert_lrclk = !invert_lrclk;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1 |= ADAU1962_DAC_CTRL1_LRCLK_PULSE;
		ctrl0 |= ADAU1962_DAC_CTRL0_FMT_I2S;
		invert_lrclk = false;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ctrl1 |= ADAU1962_DAC_CTRL1_LRCLK_PULSE;
		ctrl0 |= ADAU1962_DAC_CTRL0_FMT_LJ;
		invert_lrclk = false;
		break;
	default:
		return -EINVAL;
	}

	if (invert_lrclk)
		ctrl1 |= ADAU1962_DAC_CTRL1_LRCLK_POL;

	ret = regmap_update_bits(adau1962->regmap, ADAU1962_REG_DAC_CTRL0,
		ADAU1962_DAC_CTRL0_FMT_MASK,
		ctrl0);
	if (ret)
		return ret;

	return regmap_update_bits(adau1962->regmap, ADAU1962_REG_DAC_CTRL1,
		ADAU1962_DAC_CTRL1_MASTER | ADAU1962_DAC_CTRL1_BCLK_EDGE
		| ADAU1962_DAC_CTRL1_LRCLK_POL
		| ADAU1962_DAC_CTRL1_LRCLK_PULSE,
		ctrl1);
}

static int adau1962_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
//	struct adau1962 *adau1962 = snd_soc_component_get_drvdata(dai->component);
//	u64 formats = 0;

//	if (adau1962->slot_width == 16)
//		formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE;
//	else if (adau1962->right_j || adau1962->slot_width == 24)
//		formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |
//			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE;

//	snd_pcm_hw_constraint_list(substream->runtime, 0,
//		SNDRV_PCM_HW_PARAM_RATE, &adau1962->constraints);

//	if (adau1962->master)
//		snd_pcm_hw_constraint_minmax(substream->runtime,
//			SNDRV_PCM_HW_PARAM_RATE, 8000, adau1962->max_master_fs);

//	if (formats != 0)
//		snd_pcm_hw_constraint_mask64(substream->runtime,
//			SNDRV_PCM_HW_PARAM_FORMAT, formats);
	return 0;
}

static const struct snd_soc_dai_ops adau1962_dai_ops = {
	.startup	= adau1962_startup,
	.hw_params	= adau1962_hw_params,
	.mute_stream	= adau1962_mute,
	.set_fmt	= adau1962_set_dai_fmt,
	.set_tdm_slot	= adau1962_set_tdm_slot,
};

static struct snd_soc_dai_driver adau1962_dai = {
	.name = "adau1962-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 12,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
		    SNDRV_PCM_FMTBIT_S32_LE,
		.sig_bits = 24,
	},
	.ops = &adau1962_dai_ops,
};

static const unsigned int adau1962_rates[] = {
	32000, 64000, 128000,
	44100, 88200, 176400,
	48000, 96000, 192000,
};

#define ADAU1962_RATE_CONSTRAINT_MASK_32000 0x0007
#define ADAU1962_RATE_CONSTRAINT_MASK_44100 0x0038
#define ADAU1962_RATE_CONSTRAINT_MASK_48000 0x01c0
#define ADAU1962_RATE_CONSTRAINT_MASK_LRCLK 0x01ff

static bool adau1962_check_sysclk(unsigned int mclk, unsigned int base_freq)
{
	unsigned int mcs;

	if (mclk % (base_freq * 128) != 0)
		return false;

	mcs = mclk / (128 * base_freq);
	if (mcs < 2 || mcs > 6 || mcs == 5)
		return false;

	return true;
}

static int adau1962_set_sysclk(struct snd_soc_component *component,
	int clk_id, int source, unsigned int freq, int dir)
{
	struct adau1962 *adau1962 = snd_soc_component_get_drvdata(component);
	unsigned int mask = 0;
	unsigned int clk_src;
	unsigned int ret;

	if (dir != SND_SOC_CLOCK_IN)
		return -EINVAL;

	if (clk_id != ADAU1962_SYSCLK)
		return -EINVAL;

	switch (source) {
	case ADAU1962_SYSCLK_SRC_MCLK:
		clk_src = 0;
		break;
	case ADAU1962_SYSCLK_SRC_LRCLK:
		clk_src = ADAU1962_PLL_CLK_DLRCLK;
		break;
	default:
		return -EINVAL;
	}

	if (freq != 0 && source == ADAU1962_SYSCLK_SRC_MCLK) {
		if (freq < 8192000 || freq > 36864000)
			return -EINVAL;

		if (adau1962_check_sysclk(freq, 32000))
			mask |= ADAU1962_RATE_CONSTRAINT_MASK_32000;
		if (adau1962_check_sysclk(freq, 44100))
			mask |= ADAU1962_RATE_CONSTRAINT_MASK_44100;
		if (adau1962_check_sysclk(freq, 48000))
			mask |= ADAU1962_RATE_CONSTRAINT_MASK_48000;

		if (mask == 0)
			return -EINVAL;
	} else if (source == ADAU1962_SYSCLK_SRC_LRCLK) {
		mask = ADAU1962_RATE_CONSTRAINT_MASK_LRCLK;
	}

	ret = regmap_update_bits(adau1962->regmap, ADAU1962_REG_PLL_CLK_CTRL0,
		ADAU1962_PLL_CLK_PLLIN_MASK, clk_src);
	if (ret)
		return ret;

	adau1962->constraints.mask = mask;
	adau1962->sysclk_src = source;
	adau1962->sysclk = freq;

	return 0;
}

static const struct snd_soc_component_driver adau1962_component_driver = {
	.set_bias_level = adau1962_set_bias_level,
	.set_sysclk = adau1962_set_sysclk,
	.idle_bias_on = 1, //hfeng to check here 0?1?
	.controls = adau1962_snd_controls,
	.num_controls = ARRAY_SIZE(adau1962_snd_controls),
	.dapm_widgets = adau1962_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(adau1962_dapm_widgets),
	.dapm_routes = adau1962_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(adau1962_dapm_routes),
};

int adau1962_probe(struct device *dev, struct regmap *regmap,
		void (*switch_mode)(struct device *dev))
{
	struct adau1962 *adau1962;
	int ret;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	adau1962 = devm_kzalloc(dev, sizeof(*adau1962), GFP_KERNEL);
	if (adau1962 == NULL)
		return -ENOMEM;

	adau1962->dev = dev;
	adau1962->regmap = regmap;
	adau1962->switch_mode = switch_mode;
	adau1962->max_master_fs = 192000;

	adau1962->constraints.list = adau1962_rates;
	adau1962->constraints.count = ARRAY_SIZE(adau1962_rates);

#ifndef CONFIG_ARCH_SC59X //Reset currently controlled by gpio hog
	adau1962->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	//Do not check for errors here, as the ADAU1977 may have already claimed this
#endif

	dev_set_drvdata(dev, adau1962);

#ifndef CONFIG_ARCH_SC59X //Reset currently controlled by gpio hog
	if (!IS_ERR(adau1962->reset_gpio)) {
		/* Hardware power-on reset */
		gpiod_set_value_cansleep(adau1962->reset_gpio, 1);
		msleep(38);
		gpiod_set_value_cansleep(adau1962->reset_gpio, 0);
		/* After asserting the PU/RST pin high, ADAU1962
		 * requires 300ms to stabilize
		 */
		msleep(300);
	}
#endif

	ret = regmap_update_bits(adau1962->regmap, ADAU1962_REG_PLL_CLK_CTRL0,
				ADAU1962_PLL_CLK_PUP, ADAU1962_PLL_CLK_PUP);
	if (ret)
		return ret;

#if __DEBUG
	adau1962_print(adau1962);
#endif

	return snd_soc_register_component(dev, &adau1962_component_driver,
			&adau1962_dai, 1);
}
EXPORT_SYMBOL_GPL(adau1962_probe);

void adau1962_remove(struct device *dev)
{
}
EXPORT_SYMBOL_GPL(adau1962_remove);

static bool adau1962_register_volatile(struct device *dev, unsigned int reg)
{
	return false;
}

const struct regmap_config adau1962_regmap_config = {
	.max_register = ADAU1962_REG_DAC_POWER3,
	.volatile_reg = adau1962_register_volatile,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = adau1962_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(adau1962_reg_defaults),
};
EXPORT_SYMBOL_GPL(adau1962_regmap_config);

MODULE_DESCRIPTION("Analog Devices ADAU1962 driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
