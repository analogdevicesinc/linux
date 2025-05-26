// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Legacy SPU-compatibility layer. No SPU functionality is used currently,
 * this will be removed when it is eliminated from all drivers.
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/export.h>
#include <linux/types.h>

void set_spu_securep_msec(u16 n, bool msec)
{
	(void)n;
	(void)msec;
}
EXPORT_SYMBOL(set_spu_securep_msec);
