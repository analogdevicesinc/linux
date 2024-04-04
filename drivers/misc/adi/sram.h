/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * SRAM driver for ADI processor on-chip memory
 *
 * (C) Copyright 2025 - Analog Devices, Inc.
 *
 * Authors:
 *    Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *    Greg Malysa <greg.malysa@timesys.com>
 */

#ifndef __MACH_SRAM_H
#define __MACH_SRAM_H

#define L2CTL0_CTL_OFFSET		0x0		/* L2CTL0 Control Register Offset */
#define L2CTL0_STAT_OFFSET		0x10	/* L2CTL0 Status Register Offset */
#define L2CTL0_ERRADDR0_OFFSET	0x40	/* L2CTL0 ECC Error Address 0 Register Offset */
#define L2CTL0_ET0_OFFSET		0x80	/* L2CTL0 Error Type 0 Register Offset */
#define L2CTL0_EADDR0_OFFSET	0x84	/* L2CTL0 Error Type 0 Address Register Offset */
#define L2CTL0_ET1_OFFSET		0x88	/* L2CTL0 Error Type 1 Register Offset */
#define L2CTL0_EADDR1_OFFSET	0x8C	/* L2CTL0 Error Type 1 Address Register Offset */

#endif /* __MACH_SRAM_H */
