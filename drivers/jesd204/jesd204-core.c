/**
 * The JESD204 subsystem core
 *
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Based on elements of the IIO & CoreSight subsystems.
 */

#define pr_fmt(fmt) "jesd204: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/debugfs.h>

static dev_t jesd204_devt;

#define JESD204_DEV_MAX 256
static struct bus_type jesd204_bus_type = {
	.name = "jesd204",
};

static struct dentry *jesd204_debugfs_dentry;

static int __init jesd204_init(void)
{
	int ret;

	/* Register sysfs bus */
	ret  = bus_register(&jesd204_bus_type);
	if (ret < 0) {
		pr_err("could not register bus type\n");
		goto error_nothing;
	}

	ret = alloc_chrdev_region(&jesd204_devt, 0, JESD204_DEV_MAX, "jesd204");
	if (ret < 0) {
		pr_err("failed to allocate char dev region\n");
		goto error_unregister_bus_type;
	}

	jesd204_debugfs_dentry = debugfs_create_dir("jesd204", NULL);

	return 0;

error_unregister_bus_type:
	bus_unregister(&jesd204_bus_type);
error_nothing:
	return ret;
}

static void __exit jesd204_exit(void)
{
	if (jesd204_devt)
		unregister_chrdev_region(jesd204_devt, JESD204_DEV_MAX);
	bus_unregister(&jesd204_bus_type);
	debugfs_remove_recursive(jesd204_debugfs_dentry);
}

subsys_initcall(jesd204_init);
module_exit(jesd204_exit);

MODULE_AUTHOR("Alexandru Ardelean <alexandru.ardelean@analog.com>");
MODULE_DESCRIPTION("JESD204 core");
MODULE_LICENSE("GPL");
