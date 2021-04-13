/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ADPD188 Integrated Optical Module for Smoke Detection
 *
 * Copyright 2021 Analog Devices Inc.
 */

#ifndef _ADPD188_H_
#define _ADPD188_H_

enum supported_parts {
	ADPD188
};

int adpd188_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name, int irq);

#endif /* _ADPD188_H_ */

