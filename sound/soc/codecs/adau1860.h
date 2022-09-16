/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ADAU1860 driver
 *
 * Copyright 2022 Analog Devices Inc.
 * Author: Bogdan Togorean <bogdan.togorean@analog.com>
 */

#ifndef __SOUND_SOC_CODECS_ADAU1761_H__
#define __SOUND_SOC_CODECS_ADAU1761_H__

#include <linux/regmap.h>

#define ADAU1860_VENDOR_ID		0x4000c000
#define ADAU1860_DEVICE_ID1		0x4000c001
#define ADAU1860_DEVICE_ID2		0x4000c002
#define ADAU1860_REVISION		0x4000c003

#define ADAU1860_ADC_DAC_HP_PWR		0x4000c004
#define ADAU1860_PLL_PGA_PWR		0x4000c005
#define ADAU1860_DMIC_PWR		0x4000c006
#define ADAU1860_SAI_CLK_PWR		0x4000c007
#define ADAU1860_CHIP_PWR		0x4000c00e

#define ADAU1860_ADC_CTRL1		0x4000c020
#define ADAU1860_ADC_CTRL2		0x4000c021
#define ADAU1860_ADC_CTRL3		0x4000c022
#define ADAU1860_ADC_CTRL4		0x4000c023
#define ADAU1860_ADC_CTRL7		0x4000c026
#define ADAU1860_ADC_MUTES		0x4000c027
#define ADAU1860_ADC0_VOL		0x4000c028
#define ADAU1860_ADC1_VOL		0x4000c029
#define ADAU1860_ADC2_VOL		0x4000c02a

#define ADAU1860_PGA0_CTRL2		0x4000c031
#define ADAU1860_PGA1_CTRL2		0x4000c033
#define ADAU1860_PGA2_CTRL2		0x4000c035
#define ADAU1860_PGA_CTRL2		0x4000c037

#define ADAU1860_DMIC_VOL0		0x4000c045
#define ADAU1860_DMIC_VOL1		0x4000c046
#define ADAU1860_DMIC_VOL2		0x4000c047
#define ADAU1860_DMIC_VOL3		0x4000c048
#define ADAU1860_DMIC_VOL4		0x4000c04c
#define ADAU1860_DMIC_VOL5		0x4000c04d
#define ADAU1860_DMIC_VOL6		0x4000c04e
#define ADAU1860_DMIC_VOL7		0x4000c04f

#define ADAU1860_DAC_VOL0		0x4000c052
#define ADAU1860_DAC_ROUTE0		0x4000c053

#define ADAU1860_HP_CTRL		0x4000c060
#define ADAU1860_HPLDO_CTRL		0x4000c066
#define ADAU1860_PB_CTRL		0x4000c06c

#define ADAU1860_SPT0_CTRL1		0x4000c0e0
#define ADAU1860_SPT0_CTRL2		0x4000c0e1
#define ADAU1860_SPT0_CTRL3		0x4000c0e2
#define ADAU1860_SPT0_ROUTE(x)		(0x4000ce3 + (x))

#define ADAU1860_SPT1_CTRL1		0x4000c0f3
#define ADAU1860_SPT1_CTRL2		0x4000c0f4
#define ADAU1860_SPT1_CTRL3		0x4000c0f5
#define ADAU1860_SPT1_ROUTE(x)		(0x4000c0f6 + (x))

#define ADAU1860_MP_MCLKO_RATE		0x4000c12f

struct device;

enum adau1860_type {
	ADAU1850,
	ADAU1860,
};

enum adau18x0_clk_src {
	ADAU18X0_CLK_SRC_EXT_CLK,
	ADAU18X0_CLK_SRC_FSYNC_0,
	ADAU18X0_CLK_SRC_FSYNC_1,
	ADAU18X0_CLK_SRC_BCLK_0,
	ADAU18X0_CLK_SRC_BCLK_1,
};

enum adau18x0_pll_src {
	ADAU18X0_PLL_SRC_MCLKIN,
	ADAU18X0_PLL_SRC_FSYNC_0,
	ADAU18X0_PLL_SRC_FSYNC_1,
	ADAU18X0_PLL_SRC_BCLK_0,
	ADAU18X0_PLL_SRC_BCLK_1,
};

struct adau18x0 {
	unsigned int sysclk;
	unsigned int pll_freq;
	struct clk *mclk;
	bool enabled;

	enum adau18x0_clk_src clk_src;
	enum adau1860_type type;
	void (*switch_mode)(struct device *dev);

	unsigned int dai_fmt;

	uint8_t pll_regs[6];

	struct gpio_desc *pd_gpio;

	unsigned int tdm_slot[2];
	bool dsp_bypass[2];

	struct regmap *regmap;
};

int adau1860_probe(struct device *dev, struct regmap *regmap,
	enum adau1860_type type, void (*switch_mode)(struct device *dev));

extern const struct regmap_config adau1860_regmap_config;

#endif
