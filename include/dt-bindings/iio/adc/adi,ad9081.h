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

/* ffh: 2 - gpio6, 3 - gpio7, 4 - gpio8, 5 - gpio9, 6 - gpio10, 7 - syncinb1_p, 8 - syncinb1_n */

#define AD9081_PERI_SEL_GPIO6		2
#define AD9081_PERI_SEL_GPIO7		3
#define AD9081_PERI_SEL_GPIO8		4
#define AD9081_PERI_SEL_GPIO9		5
#define AD9081_PERI_SEL_GPIO10		6
#define AD9081_PERI_SEL_SYNCINB1_P	7
#define AD9081_PERI_SEL_SYNCINB1_N	8

#define AD9081_FFH_CHAN_SEL_REG_MODE		0 /* 0:  Register Map control (Use ddc_nco_regmap_chan_sel) */
#define AD9081_FFH_CHAN_SEL_1GPIO_MODE		1 /* 1:  profile_pins[0]     is used. Pin level control {3'b0, profile_pins[0]} */
#define AD9081_FFH_CHAN_SEL_2GPIO_MODE		2 /* 2:  profile_pins[1 :0] are used. Pin level control {2'b0, profile_pins[1:0]} */
#define AD9081_FFH_CHAN_SEL_3GPIO_MODE		3 /* 3:  profile_pins[2 :0] are used. Pin level control {1'b0, profile_pins[2:0]} */
#define AD9081_FFH_CHAN_SEL_4GPIO_MODE		4 /* 4:  profile_pins[3 :0] are used. Pin level control { profile_pins[3:0]} */
#define AD9081_FFH_CHAN_SEL_GPIO0_EDGE_MODE	8 /* 8:  profile_pins[0] Pin edge control- increment internal counter when rising edge of profile_pins[0] Pin. */
#define AD9081_FFH_CHAN_SEL_GPIO1_EDGE_MODE	9 /* 9:  profile_pins[1] Pin edge control- increment internal counter when rising edge of profile_pins[1] Pin. */
#define AD9081_FFH_CHAN_SEL_GPIO2_EDGE_MODE	10 /* 10: profile_pins[2] Pin edge control- increment internal counter when rising edge of profile_pins[2] Pin. */
#define AD9081_FFH_CHAN_SEL_GPIO3_EDGE_MODE	11 /* 11: profile_pins[3] Pin edge control- increment internal counter when rising edge of profile_pins[3] Pin. */
#define AD9081_FFH_CHAN_SEL_FHT_EXP_MODE	12 /* 12: FHT expire based control - increment internal counter when FHT is expired. */

/*
 * Equaliser CTLE Filter Selection, Range 0 - 4, based on Jesd IL,
 * Pick lower setting for Higher Insertion loss
 * Example:
 *    adi,ctle-filter-settings /bits/ 8 <1 1 1 1 1 1 1 1>;
 */
#define AD9081_CTLE_FLT_0 0
#define AD9081_CTLE_FLT_1 1
#define AD9081_CTLE_FLT_2 2
#define AD9081_CTLE_FLT_3 3

/*
 * JESD Serializer Swing Settings
 * Example:
 *   adi,lane-swing-settings /bits/ 8 <1 1 1 1 1 1 1 1>;
 */
#define AD9081_SER_SWING_1000 0 /*!< 1000 mV Swing */
#define AD9081_SER_SWING_850 1  /*!< 850 mV Swing */
#define AD9081_SER_SWING_750 2  /*!< 750 mV Swing */
#define AD9081_SER_SWING_500 3  /*!< 500 mV Swing */

/*
 * JESD Serializer Pre-Emphasis Settings
 * Example:
 *   adi,lane-pre-emp-settings /bits/ 8 <1 1 1 1 1 1 1 1>;
 */
#define AD9081_SER_PRE_EMP_0DB 0 /*!< 0 db Pre-Emphasis */
#define AD9081_SER_PRE_EMP_3DB 1 /*!< 3 db Pre-Emphasis */
#define AD9081_SER_PRE_EMP_6DB 2 /*!< 6 db Pre-Emphasis */

/*
 * JESD Serializer Post-Emphasis Settings
 * Example:
 *   adi,lane-post-emp-settings /bits/ 8 <1 1 1 1 1 1 1 1>;
 */
#define AD9081_SER_POST_EMP_0DB 0  /*!< 0 db Post-Emphasis */
#define AD9081_SER_POST_EMP_3DB 1  /*!< 3 db Post-Emphasis */
#define AD9081_SER_POST_EMP_6DB 2  /*!< 6 db Post-Emphasis */
#define AD9081_SER_POST_EMP_9DB 3  /*!< 9 db Post-Emphasis */
#define AD9081_SER_POST_EMP_12DB 4 /*!< 12 db Post-Emphasis */

 /*
  * JESD204-FSM defines
  */
#define DEFRAMER_LINK0_TX 0
#define DEFRAMER_LINK1_TX 1
#define FRAMER_LINK0_RX 2
#define FRAMER_LINK1_RX 3

#endif
