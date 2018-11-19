// SPDX-License-Identifier: GPL-2.0+
/**
 * The JESD204 framework - core logic
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#define pr_fmt(fmt) "jesd204: " fmt

#include <linux/kernel.h>
#include <linux/module.h>

static int __init jesd204_init(void)
{
	return 0;
}

static void __exit jesd204_exit(void)
{
}

subsys_initcall(jesd204_init);
module_exit(jesd204_exit);

MODULE_AUTHOR("Alexandru Ardelean <alexandru.ardelean@analog.com>");
MODULE_DESCRIPTION("JESD204 framework core");
MODULE_LICENSE("GPL");
