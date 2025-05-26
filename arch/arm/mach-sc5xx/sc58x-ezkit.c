// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Machine entries for the sc58x boards (ezkit, mini, etc.)
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
#include <linux/of_platform.h>

#include <asm/mach/arch.h>

#include "core.h"

static const char * const sc58x_dt_board_compat[] __initconst = {
	"adi,sc58x",
	NULL
};

static void __init sc58x_init(void)
{
	pr_info("%s: registering device resources\n", __func__);
	of_platform_default_populate(NULL, NULL, NULL);
	sc5xx_init_ethernet();
}

DT_MACHINE_START(SC58X_DT, "SC58x-EZKIT (Device Tree Support)")
	.l2c_aux_val = 0,
	.l2c_aux_mask = ~0,
	.init_machine	= sc58x_init,
	.dt_compat	= sc58x_dt_board_compat,
MACHINE_END
