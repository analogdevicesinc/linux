/*
 * SSM4567 amplifier audio driver
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef __SOUND_SOC_CODECS_SSM4567_H__
#define __SOUND_SOC_CODECS_SSM4567_H__

#include <linux/regmap.h>

struct device;

extern const struct regmap_config ssm4567_regmap_config;
int ssm4567_probe(struct device *dev, bool ctdm_mode, struct regmap *regmap);

#endif
