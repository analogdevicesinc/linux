// SPDX-License-Identifier: GPL-2.0-or-later
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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/gfp.h>
#include <linux/bitops.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/phy.h>
#include <linux/sched_clock.h>

#include <asm/irq.h>

#include <linux/soc/adi/sc59x.h>
#include <linux/soc/adi/cpu.h>

#include "core.h"

/** @todo spu stuff move to sec.c and make it a real driver */
static void __iomem *spu_base;

void set_spu_securep_msec(u16 n, bool msec)
{
	/*
	 * This throws a data abort right now.
	 * I assume the SPU is inaccessible from EL1.
	 * If we need to adjust this from kernel-space,
	 * this will have to be a secure monitor call (optee?)
	 */
	spu_base = NULL;
}

EXPORT_SYMBOL(set_spu_securep_msec);
/** end @todo spu stuff */
