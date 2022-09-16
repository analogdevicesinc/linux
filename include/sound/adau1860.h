/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * linux/sound/adau1860.h -- Platform data for ADAU1860
 *
 * Copyright 2022 Analog Devices Inc.
 */

#ifndef __ADAU1860_H
#define __ADAU1860_H

/**
 * struct adau1860_pdata - ADAU1860 Codec driver platform data
 * @input_differential: If true the input pins will be configured in
 *  differential mode.
 */
struct adau1860_pdata {
	/* Boost Controller Voltage Setting */
	bool input_differential;
};

#endif /* __ADAU1860_H */