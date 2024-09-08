/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * AD4170 ADC
 *
 * Copyright 2024 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _DT_BINDINGS_IIO_ADC_AD4170_H_
#define _DT_BINDINGS_IIO_ADC_AD4170_H_

/*
 * Excitation Current Chopping Control.
 * Use for adi,chop-iexc
 */
#define AD4170_MISC_CHOP_IEXC_OFF	0
/* Chopping of Iout_A and Iout_B Excitation Currents. */
#define AD4170_MISC_CHOP_IEXC_AB	1
/* Chopping of Iout_C and Iout_D Excitation Currents. */
#define AD4170_MISC_CHOP_IEXC_CD	2
/* Chopping of Both Pairs of Excitation Currents. */
#define AD4170_MISC_CHOP_IEXC_ABCD	3

/*
 * Chopping
 * Use for adi,chop-adc
 */
/* No Chopping. */
#define AD4170_MISC_CHOP_ADC_OFF		0
/* Chops Internal Mux. */
#define AD4170_MISC_CHOP_ADC_MUX		1
/* Chops AC Excitation Using 4 GPIO Pins. */
#define AD4170_MISC_CHOP_ADC_ACX_4PIN		2
/* Chops AC Excitation Using 2 GPIO Pins. */
#define AD4170_MISC_CHOP_ADC_ACX_2PIN		3

/*
 * Reference Selection Mode
 * Use for adi,reference-select
 */
#define AD4170_AFE_REFIN_REFIN1		0
#define AD4170_AFE_REFIN_REFIN2		1
#define AD4170_AFE_REFIN_REFOUT		2
#define AD4170_AFE_REFIN_AVDD		3

/*
 * Definitios for describing channel selections
 * Use for adi,channel-map
 */
#define AD4170_MAP_AIN0			0x00
#define AD4170_MAP_AIN1			0x01
#define AD4170_MAP_AIN2			0x02
#define AD4170_MAP_AIN3			0x03
#define AD4170_MAP_AIN4			0x04
#define AD4170_MAP_AIN5			0x05
#define AD4170_MAP_AIN6			0x06
#define AD4170_MAP_AIN7			0x07
#define AD4170_MAP_AIN8			0x08
#define AD4170_MAP_TEMP_SENSOR_P	0x11
#define AD4170_MAP_TEMP_SENSOR_N	0x11
#define AD4170_MAP_AVDD_AVSS_P		0x12
#define AD4170_MAP_AVDD_AVSS_N		0x12
#define AD4170_MAP_IOVDD_DGND_P		0x13
#define AD4170_MAP_IOVDD_DGND_N		0x13
#define AD4170_MAP_DAC_P		0x14
#define AD4170_MAP_DAC_N		0x14
#define AD4170_MAP_ALDO			0x15
#define AD4170_MAP_DLDO			0x16
#define AD4170_MAP_AVSS			0x17
#define AD4170_MAP_DGND			0x18
#define AD4170_MAP_REFIN1_P		0x19
#define AD4170_MAP_REFIN1_N		0x1A
#define AD4170_MAP_REFIN2_P		0x1B
#define AD4170_MAP_REFIN2_N		0x1C
#define AD4170_MAP_REFOUT		0x1D

#endif /* _DT_BINDINGS_IIO_ADC_AD4170_H_ */
