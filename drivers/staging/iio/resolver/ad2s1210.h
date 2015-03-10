/*
 * ad2s1210.h plaform data for the ADI Resolver to Digital Converters:
 * AD2S1210
 *
 * Copyright (c) 2010-2010 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct ad2s1210_platform_data {
	struct gpio_desc *sample;
	struct gpio_desc *a[2];
	struct gpio_desc *res[2];
	bool gpioin;
	unsigned int clk_in_freq;
	bool entrl_conf_mode_en;
};
