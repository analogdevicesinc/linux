/*
 * Analog Devices AD5064 DAC driver
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __IIO_ADC_AD5064_H__
#define __IIO_ADC_AD5064_H__

/**
 * struct ad5064_platform_data - AD5064 platform data
 * @use_external_ref: If set to true use an external voltage reference connected
 * to the VREF pin, otherwise use the internal reference derived from Vdd.
 */
struct ad5064_platform_data {
	bool use_external_ref;
};

#endif
