// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/reboot.h>

enum {
	ADRV_REBOOT_WARM = 0,
	ADRV_REBOOT_COLD,
};

static struct kobject *reboot_mode_kobj;
static int mode;

static ssize_t reboot_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mode);
}

static ssize_t reboot_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmp_mode = 0, result = 0;

	result = sscanf(buf, "%du", &tmp_mode);
	if (result == 1) {
		if (tmp_mode == ADRV_REBOOT_WARM || tmp_mode == ADRV_REBOOT_COLD) {
			mode = tmp_mode;
			reboot_mode = (mode == ADRV_REBOOT_WARM ? REBOOT_WARM : REBOOT_COLD);
		} else {
			pr_err("Failed in %s, setting for mode = %d is not supported!\n", __func__, tmp_mode);
			return -EINVAL;
		}

		return count;
	} else {
		return -EIO;
	}
}

static struct kobj_attribute reboot_mode_attribute = __ATTR(mode, 0660, reboot_mode_show, reboot_mode_store);

static int __init reboot_mode_handler_init(void)
{
	int ret = 0;

	reboot_mode_kobj = kobject_create_and_add("adrv-reboot", kernel_kobj);

	if (!reboot_mode_kobj)
		return -ENOMEM;

	ret = sysfs_create_file(reboot_mode_kobj, &reboot_mode_attribute.attr);
	if (ret) {
		pr_err("Failed to create the mode file in /sys/kernel/reboot.\n");
		return ret;
	}

	return ret;
}

static void __exit reboot_mode_handler_exit(void)
{
	kobject_put(reboot_mode_kobj);
	sysfs_remove_file(reboot_mode_kobj, &reboot_mode_attribute.attr);
}

module_init(reboot_mode_handler_init);
module_exit(reboot_mode_handler_exit);

MODULE_AUTHOR("Analog Devices, Inc.");
MODULE_LICENSE("GPL v2");
