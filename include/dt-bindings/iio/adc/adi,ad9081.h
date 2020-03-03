/*
 * Driver for AD9081 and similar high-speed Analog-to-Digital converters
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#ifndef _DT_BINDINGS_IIO_ADC_AD9081_H
#define _DT_BINDINGS_IIO_ADC_AD9081_H

#define AD9081_DAC_MODE_0 0x00		/*!< I0.Q0 -> DAC0, I1.Q1 -> DAC1 */
#define AD9081_DAC_MODE_1 0x01		/*!< (I0 + I1) / 2 -> DAC0, (Q0 + Q1) / 2 -> DAC1, Data Path NCOs Bypassed */
#define AD9081_DAC_MODE_2 0x02		/*!< I0 -> DAC0, Q0 -> DAC1, Datapath0 NCO Bypassed, Datapath1 Unused */
#define AD9081_DAC_MODE_3 0x03		/*!< (I0 + I1) / 2 -> DAC0, DAC1 Output Tied To Midscale */

#define AD9081_ADC_NCO_VIF 0		/*!< Variable IF Mode */
#define AD9081_ADC_NCO_ZIF 1		/*!< Zero IF Mode */
#define AD9081_ADC_NCO_FS_4_IF 2	/*!< Fs/4 Hz IF Mode */
#define AD9081_ADC_NCO_TEST 3		/*!< Test Mode */
#define AD9081_ADC_NCO_INVALID 4	/*!< Invalid NCO Mode */

#define AD9081_ADC_4_ADC_REAL_MODE 0	/*!< Quad ADC Real Mode */
#define AD9081_ADC_4_ADC_COMP_MODE 1	/*!< Quad ADC Complex Mode */
#define AD9081_ADC_2_ADC_REAL_MODE 2	/*!< Dual ADC Real Mode */
#define AD9081_ADC_2_ADC_COMP_MODE 3	/*!< Dual ADC Complex MOde */

#define AD9081_ADC_NYQUIST_ZONE_ODD  0x00	/*!< Odd  Zone */
#define AD9081_ADC_NYQUIST_ZONE_EVEN 0x01	/*!< Even Zone */

#define FDDC_I 0
#define FDDC_Q 1

#endif
