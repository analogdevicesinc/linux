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

/*
 * PA Protection Soft-Off Trigger Bitmasks
 * Used with adi,pa-protection-soft-off-triggers property
 * Multiple triggers can be OR'd together
 * Example:
 *   adi,pa-protection-soft-off-triggers = <(AD9081_PA_SOFT_OFF_SPI |
 *                                           AD9081_PA_SOFT_OFF_TXEN |
 *                                           AD9081_PA_SOFT_OFF_JESD_ERR)>;
 */
#define AD9081_PA_SOFT_OFF_SPI              (1 << 0)  /*!< SPI soft-off */
#define AD9081_PA_SOFT_OFF_TXEN             (1 << 1)  /*!< TxEN soft-off */
#define AD9081_PA_SOFT_OFF_ROTATE           (1 << 2)  /*!< Rotate soft-off */
#define AD9081_PA_SOFT_OFF_JESD_ERR         (1 << 3)  /*!< JESD error soft-off */
#define AD9081_PA_SOFT_OFF_HWIP_ERR         (1 << 4)  /*!< Hardware IP error soft-off */
#define AD9081_PA_SOFT_OFF_LONG_PAERR       (1 << 6)  /*!< Long PA error soft-off */
#define AD9081_PA_SOFT_OFF_SHORT_PAERR      (1 << 7)  /*!< Short PA error soft-off */
#define AD9081_PA_SOFT_OFF_DLL_UNLOCK       (1 << 8)  /*!< DLL unlock soft-off */
#define AD9081_PA_SOFT_OFF_204C_CRC_ERR     (1 << 9)  /*!< 204C CRC error soft-off */
#define AD9081_PA_SOFT_OFF_HI_LO_FAIL       (1 << 11) /*!< Hi-Lo fail soft-off */
#define AD9081_PA_SOFT_OFF_SLEW_RATE_ERR    (1 << 12) /*!< Slew rate error soft-off */

/*
 * PA Protection Soft-On Trigger Bitmasks
 * Used with adi,pa-protection-soft-on-triggers property
 * Example:
 *   adi,pa-protection-soft-on-triggers = <(AD9081_PA_SOFT_ON_HI_LO_RECV |
 *                                          AD9081_PA_SOFT_ON_SPI_FORCE)>;
 */
#define AD9081_PA_SOFT_ON_HI_LO_RECV        (1 << 4)  /*!< Hi-Lo recover soft-on */
#define AD9081_PA_SOFT_ON_LONG_LEVEL        (1 << 6)  /*!< Long level soft-on */
#define AD9081_PA_SOFT_ON_SPI_FORCE         (1 << 7)  /*!< SPI force soft-on when gain is 0 */

/*
 * PA Protection Rotation Mode Values
 * Used with adi,pa-protection-rotation-mode property
 * Example:
 *   adi,pa-protection-rotation-mode = <AD9081_PA_ROTATION_JESD_AUTO>;
 */
#define AD9081_PA_ROTATION_JESD_AUTO        (1 << 0)  /*!< Enable JESD auto off/on during rotation */
#define AD9081_PA_ROTATION_DATAPATH_AUTO    (1 << 1)  /*!< Enable datapath auto soft off/on during rotation */

#endif
