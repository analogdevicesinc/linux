/*
 * AD9528 SPI Low Jitter Clock Generator
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _DT_BINDINGS_IIO_FREQUENCY_AD9528_H_
#define _DT_BINDINGS_IIO_FREQUENCY_AD9528_H_

/* Output Driver Mode
 * Use for adi,driver-mode */
#define DRIVER_MODE_LVDS	0
#define DRIVER_MODE_LVDS_BOOST	1
#define DRIVER_MODE_HSTL	2

/* Output Signal Source
 * Use for adi,signal-source */
#define SOURCE_VCO		0
#define SOURCE_VCXO		1
#define SOURCE_SYSREF_VCO	2
#define SOURCE_SYSREF_VCXO	3
#define SOURCE_VCXO_INV		5
#define SOURCE_SYSREF_VCXO_INV	7

/* Reference Selection Mode
 * Use for adi,ref-mode */
#define REF_MODE_STAY_ON_REFB	0
#define REF_MODE_REVERT_TO_REFA	1
#define REF_MODE_SELECT_REFA	2
#define REF_MODE_SELECT_REFB	3
#define REF_MODE_EXT_REF	4

/* Sysref Source
 * Use for adi,sysref-src */
#define SYSREF_SRC_EXTERNAL		0
#define SYSREF_SRC_EXTERNAL_RESAMPLED	1
#define SYSREF_SRC_INTERNAL		2

/* Sysref Pattern Mode */
#define SYSREF_PATTERN_NSHOT		0
#define SYSREF_PATTERN_CONTINUOUS	1
#define SYSREF_PATTERN_PRBS		2
#define SYSREF_PATTERN_STOP		3

/* Sysref NSHOT Mode
 * Use for adi,sysref-nshot-mode */
#define SYSREF_NSHOT_1_PULSE		1
#define SYSREF_NSHOT_2_PULSES		2
#define SYSREF_NSHOT_4_PULSES		3
#define SYSREF_NSHOT_6_PULSES		4
#define SYSREF_NSHOT_8_PULSES		5

/* Sysref Trigger Mode
 * Use for adi,sysref-request-trigger-mode */
#define SYSREF_LEVEL_HIGH		0
#define SYSREF_EDGE_RISING		2
#define SYSREF_EDGE_FALLING		3


/* Rpole2 resistor
 * Use for adi,rpole2 */
#define RPOLE2_900_OHM	0
#define RPOLE2_450_OHM	1
#define RPOLE2_300_OHM	2
#define RPOLE2_225_OHM	3

/* Rzero resistor
 * Use for adi,rzero */
#define RZERO_3250_OHM	0
#define RZERO_2750_OHM	1
#define RZERO_2250_OHM	2
#define RZERO_2100_OHM	3
#define RZERO_3000_OHM	4
#define RZERO_2500_OHM	5
#define RZERO_2000_OHM	6
#define RZERO_1850_OHM	7

/* Cpole1 capacitor
 * Use for adi,cpole1 */
#define CPOLE1_0_PF	0
#define CPOLE1_8_PF	1
#define CPOLE1_16_PF	2
#define CPOLE1_24_PF	3
#define CPOLE1_32_PF	5
#define CPOLE1_40_PF	6
#define CPOLE1_48_PF	7

#endif /* _DT_BINDINGS_IIO_FREQUENCY_AD9528_H_ */
