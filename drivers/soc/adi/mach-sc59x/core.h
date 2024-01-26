/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * core timer and machine init for ADI processor on-chip memory
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef __ASM_ARCH_SC58X_H
#define __ASM_ARCH_SC58X_H

#include <linux/of_platform.h>
#include <linux/reboot.h>

extern void __init sc59x_init(void);
extern void __init sc59x_init_early(void);
extern void __init sc59x_init_irq(void);
extern void __init sc59x_map_io(void);
extern void sc59x_timer_init(void);
extern void sc59x_clock_init(void);
#endif
