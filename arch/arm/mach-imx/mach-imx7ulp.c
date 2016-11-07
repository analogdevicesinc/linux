/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static void __init imx7ulp_init_machine(void)
{
	struct device *parent;

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static void __init imx7ulp_init_irq(void)
{
	/* TBD */
	mxc_set_cpu_type(MXC_CPU_IMX7ULP);

	irqchip_init();
	imx7ulp_pm_init();
}

static void __init imx7ulp_map_io(void)
{
	imx7ulp_pm_map_io();
}

static void __init imx7ulp_init_late(void)
{
	imx7ulp_enable_nmi();
}

static const char *const imx7ulp_dt_compat[] __initconst = {
	"fsl,imx7ulp",
	NULL,
};

DT_MACHINE_START(IMX7ulp, "Freescale i.MX7ULP (Device Tree)")
	.map_io		= imx7ulp_map_io,
	.init_irq	= imx7ulp_init_irq,
	.init_machine	= imx7ulp_init_machine,
	.init_late      = imx7ulp_init_late,
	.dt_compat	= imx7ulp_dt_compat,
MACHINE_END
