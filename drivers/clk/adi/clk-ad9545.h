/* SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause */
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright 2020 Analog Devices Inc.
 */

#ifndef _CLK_AD9545_H_
#define _CLK_AD9545_H_

struct device;
struct regmap;

int ad9545_probe(struct device *dev, struct regmap *regmap);

#endif /* _CLK_AD9545_H_ */
