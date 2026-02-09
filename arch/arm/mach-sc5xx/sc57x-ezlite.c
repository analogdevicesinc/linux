// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Machine entries for the sc573 ezlite
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
#include <asm/siginfo.h>
#include <asm/signal.h>

#include "core.h"

static const char * const sc57x_dt_board_compat[] __initconst = {
	"adi,sc57x",
	NULL
};

static bool first_fault = true;

static int sc57x_abort_handler(unsigned long addr, unsigned int fsr,
		struct pt_regs *regs)
{
	if (fsr == 0x1c06 && first_fault) {
		first_fault = false;

		/*
		 * These faults with code 0x1c06 happens for no good reason,
		 * possibly left over from the CFE boot loader.
		 */
		pr_warn("External imprecise Data abort at addr=%#lx, fsr=%#x ignored.\n",
				addr, fsr);
		return 0;
	}

	/* Others should cause a fault */
	return 1;
}

static void __init sc57x_init_early(void)
{
	hook_fault_code(16 + 6, sc57x_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
}

static void __init sc57x_init(void)
{
	pr_info("%s: registering device resources\n", __func__);
	of_platform_default_populate(NULL, NULL, NULL);
	sc5xx_init_ethernet();
}

DT_MACHINE_START(SC57X_DT, "SC57x-EZLITE (Device Tree Support)")
	.l2c_aux_val = 0,
	.l2c_aux_mask = ~0,
	.init_early	= sc57x_init_early,
	.init_machine	= sc57x_init,
	.dt_compat	= sc57x_dt_board_compat,
MACHINE_END
