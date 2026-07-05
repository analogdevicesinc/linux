/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (c) 2026 Analog Devices Inc.
 */

#ifndef __DT_BINDINGS_ADI_ADSP_PINCTRL_H
#define __DT_BINDINGS_ADI_ADSP_PINCTRL_H

#define ADSP_PORT_A	0
#define ADSP_PORT_B	1
#define ADSP_PORT_C	2
#define ADSP_PORT_D	3
#define ADSP_PORT_E	4
#define ADSP_PORT_F	5
#define ADSP_PORT_G	6
#define ADSP_PORT_H	7
#define ADSP_PORT_I	8

#define ADSP_ALT0	0
#define ADSP_ALT1	1
#define ADSP_ALT2	2
#define ADSP_ALT3	3

#define ADSP_SC571	0x584
#define ADSP_SC573	0x584
#define ADSP_SC584	0x584
#define ADSP_SC589	0x589
#define ADSP_SC594	0x594
#define ADSP_SC598	0x598

#define ADSP_PINMUX(soc, port, off, alt)                                     \
	(((ADSP_##soc) << 20) | ((((ADSP_PORT_##port) * 16) + (off)) << 2) | \
	 (ADSP_ALT##alt))

#endif /* __DT_BINDINGS_ADI_ADSP_PINCTRL_H */
