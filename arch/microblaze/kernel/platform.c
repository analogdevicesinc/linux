/*
 * Copyright 2008 Michal Simek <monstr@monstr.eu>
 *
 * based on virtex.c file
 *
 * Copyright 2007 Secret Lab Technologies Ltd.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <asm/setup.h>

#include <linux/clk-provider.h>

static struct of_device_id xilinx_of_bus_ids[] __initdata = {
	{ .compatible = "simple-bus", },
	{ .compatible = "xlnx,compound", },
	{}
};

static const __initconst struct of_device_id clk_match[] = {
	{ .compatible = "fixed-clock", .data = of_fixed_clk_setup, },
	{ /* sentinel */ }
};


static int __init microblaze_device_probe(void)
{
	of_platform_bus_probe(NULL, xilinx_of_bus_ids, NULL);
	of_clk_init(clk_match);

	return 0;
}
device_initcall(microblaze_device_probe);
