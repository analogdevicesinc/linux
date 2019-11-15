/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Generic IIO access driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#ifndef _IIO_REGMAP_H_
#define _IIO_REGMAP_H_

struct device;
struct regmap;
struct regmap_config;

int iio_regmap_read_config(struct device *dev,
			   struct regmap_config *regmap_cfg);
int iio_regmap_probe(struct device *dev, struct regmap *regmap,
		     const char *name);

#endif /* _IIO_REGMAP_H_ */
