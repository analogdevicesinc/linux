/* SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause */
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright 2020 Analog Devices Inc.
 */

#ifndef _DT_BINDINGS_CLOCK_AD9545_H_
#define _DT_BINDINGS_CLOCK_AD9545_H_

/* Input Driver Mode
 * Use for adi,single-ended-mode:
 */
#define DRIVER_MODE_AC_COUPLED_IF	0
#define DRIVER_MODE_DC_COUPLED_1V2	1
#define DRIVER_MODE_DC_COUPLED_1V8	2
#define DRIVER_MODE_IN_PULL_UP		3

/* Input Driver Mode
 * Use for adi,differential-mode:
 */
#define DRIVER_MODE_AC_COUPLED		0
#define DRIVER_MODE_DC_COUPLED		1
#define DRIVER_MODE_DC_COUPLED_LVDS	2

/* Output Driver Mode
 * Use for adi,output-mode:
 */
#define DRIVER_MODE_SINGLE_DIV_DIF	0
#define DRIVER_MODE_SINGLE_DIV		1
#define DRIVER_MODE_DUAL_DIV		2

/* Clock types */
#define AD9545_CLK_OUT			0
#define AD9545_CLK_PLL			1
#define AD9545_CLK_NCO			2
#define AD9545_CLK_AUX_TDC		3

/* PLL addresses */
#define AD9545_PLL0			0
#define AD9545_PLL1			1

/* Outputs addresses */
#define AD9545_Q0A			0
#define AD9545_Q0AA			1
#define AD9545_Q0B			2
#define AD9545_Q0BB			3
#define AD9545_Q0C			4
#define AD9545_Q0CC			5
#define AD9545_Q1A			6
#define AD9545_Q1AA			7
#define AD9545_Q1B			8
#define AD9545_Q1BB			9

/* NCO addresses */
#define AD9545_NCO0			0
#define AD9545_NCO1			1

/* TDC addresses */
#define AD9545_CLK_AUX_TDC0		0
#define AD9545_CLK_AUX_TDC1		1

/* Ex:
 * Output Q0C clock: <&ad9545_clock AD9545_CLK_OUT AD9545_Q0C>;
 * PLL0 clock: <&ad9545_clock AD9545_CLK_PLL AD9545_PLL0>;
 * NCO1 clock: <&ad9545_clock AD9545_CLK_NCO AD9545_NCO1>;
 */

#endif /* _DT_BINDINGS_CLOCK_AD9545_H_ */
