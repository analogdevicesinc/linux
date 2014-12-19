/*
 * AD7091Rx Analog -> Digital converters driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef __DRIVERS_IIO_DAC_AD7091R_BASE_H__
#define __DRIVERS_IIO_DAC_AD7091R_BASE_H__

#include <linux/types.h>
#include <linux/regmap.h>

struct device;
struct ad7091r_state;

struct ad7091r_chip_info {
	unsigned num_channels;
	const struct iio_chan_spec *channels;
};

extern const struct regmap_config ad7091r_regmap_config;

int ad7091r_probe(struct device *dev, const char *name,
		const struct ad7091r_chip_info *chip_info,
		struct regmap *map, int irq);
int ad7091r_remove(struct device *dev);

#endif /* __DRIVERS_IIO_DAC_AD7091R5_BASE_H__ */
