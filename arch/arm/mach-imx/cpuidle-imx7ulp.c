/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/module.h>
#include <asm/cpuidle.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static int imx7ulp_enter_wait(struct cpuidle_device *dev,
			    struct cpuidle_driver *drv, int index)
{
	if (index == 1)
		imx7ulp_set_lpm(WAIT);
	else
		imx7ulp_set_lpm(STOP);

	cpu_do_idle();

	imx7ulp_set_lpm(RUN);

	return index;
}

static struct cpuidle_driver imx7ulp_cpuidle_driver = {
	.name = "imx7ulp_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		/* WFI */
		ARM_CPUIDLE_WFI_STATE,
		/* WAIT */
		{
			.exit_latency = 50,
			.target_residency = 75,
			.enter = imx7ulp_enter_wait,
			.name = "WAIT",
			.desc = "PSTOP2",
		},
		/* STOP */
		{
			.exit_latency = 100,
			.target_residency = 150,
			.enter = imx7ulp_enter_wait,
			.name = "STOP",
			.desc = "PSTOP1",
		},
	},
	.state_count = 3,
	.safe_state_index = 0,
};

int __init imx7ulp_cpuidle_init(void)
{
	return cpuidle_register(&imx7ulp_cpuidle_driver, NULL);
}
