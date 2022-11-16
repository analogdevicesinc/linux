/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <linux/sizes.h>

/*
 * PCI space virtual addresses
 */
#define SC59X_PCI_VIRT_BASE		((void __iomem *)0xe8000000ul)
#define SC59X_CFG_VIRT_BASE		((void __iomem *)0xe9000000ul)

#define SC58X_PCI_VIRT_BASE		((void __iomem *)0xe8000000ul)
#define SC58X_CFG_VIRT_BASE		((void __iomem *)0xe9000000ul)

/* macro to get at MMIO space when running virtually */
#define IO_ADDRESS(x)		(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)

#define __io_address(n)		((void __iomem __force *)IO_ADDRESS(n))

#endif
