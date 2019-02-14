/*
 * ADAU1977/ADAU1978/ADAU1979 driver
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef __SOUND_SOC_CODECS_ADAU1977_H__
#define __SOUND_SOC_CODECS_ADAU1977_H__

#include <linux/regmap.h>

struct device;

enum adau1977_type {
	ADAU1977,
	ADAU1978,
	ADAU1979,
};

int adau1977_probe(struct device *dev, struct regmap *regmap,
	enum adau1977_type type, void (*switch_mode)(struct device *dev));

extern const struct regmap_config adau1977_regmap_config;

enum adau1977_clk_id {
	ADAU1977_SYSCLK,
};

enum adau1977_sysclk_src {
	ADAU1977_SYSCLK_SRC_MCLK,
	ADAU1977_SYSCLK_SRC_LRCLK,
};

enum adau1977_micbias {
	ADAU1977_MICBIAS_5V0 = 0x0,
	ADAU1977_MICBIAS_5V5 = 0x1,
	ADAU1977_MICBIAS_6V0 = 0x2,
	ADAU1977_MICBIAS_6V5 = 0x3,
	ADAU1977_MICBIAS_7V0 = 0x4,
	ADAU1977_MICBIAS_7V5 = 0x5,
	ADAU1977_MICBIAS_8V0 = 0x6,
	ADAU1977_MICBIAS_8V5 = 0x7,
	ADAU1977_MICBIAS_9V0 = 0x8,
};

#endif
