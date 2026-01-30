/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Macros for populating pinmux properties on the pincontroller
 *
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#ifndef DT_BINDINGS_PINCTRL_ADI_ADSP_H
#define DT_BINDINGS_PINCTRL_ADI_ADSP_H

#define ADI_ADSP_PINFUNC_GPIO     0
#define ADI_ADSP_PINFUNC_ALT0     1
#define ADI_ADSP_PINFUNC_ALT1     2
#define ADI_ADSP_PINFUNC_ALT2     3
#define ADI_ADSP_PINFUNC_ALT3     4

#define ADI_ADSP_PINMUX(port, pin, func) ((((port - 'A') * 16 + pin) << 8) + func)

#endif
