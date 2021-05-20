/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2020 Analog Devices, Inc.
 */

#ifndef _DT_BINDINGS_JESD204_ADXCVR_H
#define _DT_BINDINGS_JESD204_ADXCVR_H

/*
 * adi,sys-clk-select
 */
#define XCVR_CPLL		0 /* CPLL: GTHE3, GTHE4, GTYE4, GTXE2 */
#define XCVR_QPLL1		2 /* QPLL1: GTHE3, GTHE4, GTYE4 */
#define XCVR_QPLL		3 /* QPLL0: GTHE3, GTHE4, GTYE4, GTXE2 */

/*
 * adi,out-clk-select
 */
#define XCVR_OUTCLK_PCS		1
#define XCVR_OUTCLK_PMA		2
#define XCVR_REFCLK		3
#define XCVR_REFCLK_DIV2	4
#define XCVR_PROGDIV_CLK	5 /* GTHE3, GTHE4, GTYE4 only */

#endif /* _DT_BINDINGS_JESD204_ADXCVR_H */

