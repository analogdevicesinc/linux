/*
 * ADF4360 SPI PLL driver
 *
 * Copyright © 2014-2018 Analog Devices Inc.
 * Copyright © 2019 Edward Kigwana.
 *
 * Parts lifted from drivers/clk/clk-adf4360.c by Lars-Peter Clausen.
 * Parts lifted from drivers/iio/frequency/adf4360.c by Michael Hennerich.
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef IIO_PLL_ADF4360_H_
#define IIO_PLL_ADF4360_H_

/* Registers */
#define ADF4360_REG_CTRL	0x00
#define ADF4360_REG_RDIV	0x01
#define ADF4360_REG_NDIV	0x02

/* Control Bit Definitions */
#define ADF4360_GEN1_CTRL_PC_5		0
#define ADF4360_GEN1_CTRL_PC_10		1
#define ADF4360_GEN1_CTRL_PC_15		2
#define ADF4360_GEN1_CTRL_PC_20		3
#define ADF4360_CTRL_CPL(x)		(((x) & 0x3) << 2)

#define ADF4360_GEN2_CTRL_PC_2_5	(0x0 << 2)
#define ADF4360_GEN2_CTRL_PC_5		(0x1 << 2)
#define ADF4360_GEN2_CTRL_PC_7_5	(0x2 << 2)
#define ADF4360_GEN2_CTRL_PC_10		(0x3 << 2)

#define ADF4360_CTRL_COUNTER_RESET	BIT(4)
#define ADF4360_CTRL_PDP		BIT(8)
#define ADF4360_CTRL_MTLD		BIT(11)
#define ADF4360_CTRL_POWER_DOWN_EN	(1 << 20)
#define ADF4360_CTRL_PL_3_5		0
#define ADF4360_CTRL_PL_5		1
#define ADF4360_CTRL_PL_7_5		2
#define ADF4360_CTRL_PL_11		3
#define ADF4360_CTRL_PL(x)		(((x) & 0x3) << 12)
#define ADF4360_CTRL_CPI1(x)		(((x) & 0x7) << 14)
#define ADF4360_CTRL_CPI2(x)		(((x) & 0x7) << 17)
#define ADF4360_CTRL_PRESCALER_8	(0 << 22)
#define ADF4360_CTRL_PRESCALER_16	(1 << 22)
#define ADF4360_CTRL_PRESCALER_32	(2 << 22)

#define ADF4360_CTRL_MUXOUT(x)		(((x) & 7) << 5)
#define ADF4360_CTRL_MUXOUT_THREE_STATE	0
#define ADF4360_CTRL_MUXOUT_LOCK_DETECT	1
#define ADF4360_CTRL_MUXOUT_NDIV	2
#define ADF4360_CTRL_MUXOUT_DVDD	3
#define ADF4360_CTRL_MUXOUT_RDIV	4
#define ADF4360_CTRL_MUXOUT_OD_LD	5
#define ADF4360_CTRL_MUXOUT_SDO		6
#define ADF4360_CTRL_MUXOUT_GND		7

#define ADF4360_CPI_0_31	0
#define ADF4360_CPI_0_62	1
#define ADF4360_CPI_0_93	2
#define ADF4360_CPI_1_25	3
#define ADF4360_CPI_1_56	4
#define ADF4360_CPI_1_87	5
#define ADF4360_CPI_2_18	6
#define ADF4360_CPI_2_50	7

/* N Counter Bit Definitions */
#define ADF4360_NDIV_A_COUNTER(x)	((x) << 2)
#define ADF4360_NDIV_B_COUNTER(x)	((x) << 8)
#define ADF4360_NDIV_OUT_DIV2		BIT(22)
#define ADF4360_NDIV_PRESCALER_DIV2	BIT(23)

/* R Counter Bit Definitions */
#define ADF4360_RDIV_R_COUNTER(x)	((x) << 2)
#define ADF4360_RDIV_ABP(x)		(((x) & 0x3) << 16)
#define ADF4360_RDIV_ABP_3_0NS		0
#define ADF4360_RDIV_ABP_1_3NS		1
#define ADF4360_RDIV_ABP_6_0NS		2
#define ADF4360_RDIV_BSC_1		(0x0 << 20)
#define ADF4360_RDIV_BSC_2		(0x1 << 20)
#define ADF4360_RDIV_BSC_4		(0x2 << 20)
#define ADF4360_RDIV_BSC_8		(0x3 << 20)

/* Specifications */
#define ADF4360_MIN_FREQ_REFIN		10000000 /* Hz */
#define ADF4360_MAX_FREQ_REFIN		250000000 /* Hz */
#define ADF4360_MAX_PFD_RATE		8000000 /* 8 MHz */
#define ADF4360_MAX_COUNTER_RATE	300000000 /* 300 MHz */

/**
 * struct adf4360_platform_data - platform specific information.
 * @name:		Optional device name.
 * @clkin:		REFin frequency in Hz.
 * @power_up_frequency:	Optional, if set in Hz the PLL tunes to the desired
 *			frequency on probe.
 * @vco_min:		Optional mininum VCO frequency in Hz for devices
 *			configured using external hardware components.
 * @vco_max:		Optional maximum VCO frequency in Hz for devices
 *			configured using external hardware components.
 * @pfd_freq:		Phase frequency detector frequency in Hz.
 * @cpl:		Core power level setting.
 * @cpi:		Loop filter charge pump current.
 * @opl:		Output power level setting.
 * @pdp:		Phase detector polarity positive enable.
 * @mtld:		Optional, mutes output until PLL lock is detected.
 * @mux_out_ctrl:	Output multiplexer configuration.
 *			Defaults to lock detect if not set.
 * @abp:		Anti backlash setting.
 * @gpio_lock_detect:	Optional, if set with a valid GPIO number,
 *			pll lock state is tested upon read.
 *			If not used - set to -1.
 */

struct adf4360_platform_data {
	char name[32];
	unsigned long power_up_frequency;
	unsigned int vco_min;
	unsigned int vco_max;
	unsigned int pfd_freq;
	unsigned int cpl;
	unsigned int cpi;
	unsigned int opl;
	bool pdp;
	bool mtld;
	unsigned int mux_out_ctrl;
	unsigned int abp;
	int gpio_lock_detect;
};

#endif /* IIO_PLL_ADF4360_H_ */
