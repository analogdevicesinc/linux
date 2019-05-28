/*
 * Driver for AD9208 and similar high-speed Analog-to-Digital converters
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#ifndef _DT_BINDINGS_IIO_ADC_AD9208_H
#define _DT_BINDINGS_IIO_ADC_AD9208_H

#define AD9208_FULL_BANDWIDTH_MODE 0
#define AD9208_1_DDC_MODE 1
#define AD9208_2_DDC_MODE 2
#define AD9208_4_DDC_MODE 4

#define AD9208_SYSREF_NONE 0	/* No SYSREF Support */
#define AD9208_SYSREF_ONESHOT 1	/* ONE-SHOT SYSREF */
#define AD9208_SYSREF_CONT 2	/* Continuous Sysref Synchronisation */
#define AD9208_SYSREF_MON 3	/* SYSREF monitor Mode */

#define AD9208_NCO_MODE_VIF 0	/* Variable IF Mode*/
#define AD9208_NCO_MODE_ZIF 1	/* Zero IF Mode */
#define AD9208_NCO_MODE_TEST 3	/* Test Mode*/

#define AD9208_PDN_MODE_STANDBY 0x2	/**< Standby Mode Powerup */
#define AD9208_PDN_MODE_POWERDOWN 0x3	/**< Full Powerdown Mode*/

#define AD9208_BUFF_CURR_400_UA  0x4	/**< Buffer Current set to 400 uA*/
#define AD9208_BUFF_CURR_500_UA  0x9	/**< Buffer Current set to 500 uA*/
#define AD9208_BUFF_CURR_600_UA  0x1E	/**< Buffer Current set to 600 uA*/
#define AD9208_BUFF_CURR_700_UA  0x23	/**< Buffer Current set to 700 uA*/
#define AD9208_BUFF_CURR_800_UA  0x28	/**< Buffer Current set to 800 uA*/
#define AD9208_BUFF_CURR_1000_UA  0x32	/**< Buffer Current set to 1000 uA*/

#endif
