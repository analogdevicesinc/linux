/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices common definitions for JESD204 IP cores
 *
 * Copyright 2020 Analog Devices Inc.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_ip
 * https://wiki.analog.com/resources/fpga/docs/hdl/regmap
 * https://wiki.analog.com/resources/fpga/peripherals/jesd204/jesd204_tpl_adc
 * https://wiki.analog.com/resources/fpga/peripherals/jesd204/jesd204_tpl_dac
 */
#ifndef _ADI_JESD204_H_
#define _ADI_JESD204_H_

/* JESD TPL COMMON */

#define ADI_JESD204_REG_TPL_CNTRL		0x0200
#define ADI_JESD204_REG_TPL_STATUS		0x0204
#define ADI_JESD204_REG_TPL_DESCRIPTOR_1	0x0240
#define ADI_JESD204_REG_TPL_DESCRIPTOR_2	0x0244

#define ADI_JESD204_TPL_TO_M(x)			(((x) >> 0) & 0xFF)
#define ADI_JESD204_TPL_TO_L(x)			(((x) >> 8) & 0xFF)
#define ADI_JESD204_TPL_TO_S(x)			(((x) >> 16) & 0xFF)
#define ADI_JESD204_TPL_TO_F(x)			(((x) >> 24) & 0xFF)

#define ADI_JESD204_TPL_TO_N(x)			(((x) >> 0) & 0xFF)
#define ADI_JESD204_TPL_TO_NP(x)		(((x) >> 8) & 0xFF)

#define ADI_JESD204_TPL_TO_PROFILE_NUM(x)	(((x) >> 0) & 0xF)
#define ADI_JESD204_PROFILE_SEL(x)		((x) & 0xF)

#endif
