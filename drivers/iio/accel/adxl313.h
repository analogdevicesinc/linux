/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ADXL313 3-Axis Digital Accelerometer
 *
 * Copyright (c) 2021 Lucas Stankus <lucas.p.stankus@gmail.com>
 */

#ifndef _ADXL313_H_
#define _ADXL313_H_

int adxl313_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name);
#endif /* _ADXL313_H_ */
