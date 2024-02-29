/*
 * AD4170 ADC 
 *
 * Copyright 2024 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _DT_BINDINGS_IIO_ADC_AD4170_H_
#define _DT_BINDINGS_IIO_ADC_AD4170_H_

/* Excitation Current Chopping Control.
 * Use for adi,chop-iexc */
#define AD4170_CHOP_IEXC_OFF	0
	/** Chopping of Iout_A and Iout_B Excitation Currents. */
#define AD4170_CHOP_IEXC_AB	1
	/** Chopping of Iout_C and Iout_D Excitation Currents. */
#define	AD4170_CHOP_IEXC_CD	2
	/** Chopping of Both Pairs of Excitation Currents. */
#define	AD4170_CHOP_IEXC_ABCD	3

/* Chopping
 * Use for adi,chop-adc */
	/** No Chopping. */
#define AD4170_CHOP_OFF		0
	/** Chops Internal Mux. */
#define AD4170_CHOP_MUX		1
	/** Chops AC Excitation Using 4 GPIO Pins. */
#define AD4170_CHOP_ACX_4PIN  	2
	/** Chops AC Excitation Using 2 GPIO Pins. */
#define AD4170_CHOP_ACX_2PIN 	3

/* Reference Selection Mode
 * Use for adi,reference-select */
#define AD4170_REFIN_REFIN1 	0
#define AD4170_REFIN_REFIN2	1
#define	AD4170_REFIN_REFOUT	2
#define	AD4170_REFIN_AVDD	3


/* Configures Functionality of pin DIG_AUX1.
 * Use for adi,dig-aux1-function */
#define AD4170_DIG_AUX1_DISABLED "disabled"
#define AD4170_DIG_AUX1_RDY "rdy""
#define AD4170_DIG_AUX1_SYNC "sync"

/* Configures Functionality of pin DIG_AUX2.
 * Use for adi,dig-aux2-function */
#define AD4170_DIG_AUX2_DISABLED "disabled"
#define AD4170_DIG_AUX2_LDAC "ldac""
#define AD4170_DIG_AUX2_SYNC "sync"

/* Configures Functionality of pin SYNC.
 * Use for adi,sync-function */
#define AD4170_SYNC_DISABLED "disabled"
#define AD4170_SYNC_STANDARD "std""
#define AD4170_SYNC_ALTERNATE "alt"

#endif /* _DT_BINDINGS_IIO_ADC_AD4170_H_ */
