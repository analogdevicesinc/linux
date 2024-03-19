// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/reboot.h>

#include "adrv906x-status-reg.h"

/* This value MUST MATCH the implementation the following repos
 *
 * U-boot: /arch/arm/mach-adrv906x/adrv906x_status_reg.c
 * TF-A: /plat/adi/adrv/adrv906x/adrv906x_status_reg.c
 * OP-TEE os: /core/drivers/adi/adrv906x/adi_adrv906x_status_reg.c
 */
#define RESET_CAUSE_NS_OFFSET              0

/*
 * List of reasons reset was performed which gets stored in RESET_CAUSE
 * This enum MUST MATCH those defined in the following repos
 *
 * U-boot: /arch/arm/mach-adrv906x/include/plat_status_reg.h
 * TF-A: /plat/adi/adrv/common/include/plat_status_reg.h
 * OP-TEE os: /core/include/drivers/adi/adrv906x/adi_adrv906x_status_reg.h
 */
enum reset_cause_t {
	RESET_VALUE,
	IMG_VERIFY_FAIL,
	WATCHDOG_RESET,
	OTHER_RESET_CAUSE,
};

static struct kobject *err_kobj;
static int reset_cause;

static int wr_reset_cause(enum reset_cause_t cause)
{
	void *io;

	io = memremap(A55_SYS_CFG + SCRATCH_NS + RESET_CAUSE_NS_OFFSET, SZ_4K, MEMREMAP_WT);

	if (io == NULL) {
		pr_err("Unable to map to virtual address\n");
		return 0;
	}

	iowrite32(cause, io);

	return 0;
}

static int rd_reset_cause(void)
{
	void *io;

	io = memremap(A55_SYS_CFG + SCRATCH_NS + RESET_CAUSE_NS_OFFSET, SZ_4K, MEMREMAP_WT);

	if (io == NULL) {
		pr_err("Unable to map to virtual address\n");
		return 0;
	}

	return ioread32(io);
}

static int plat_panic_handler(struct notifier_block *nb, unsigned long reason, void *arg)
{
	if (strcmp(arg, "dm-verity device corrupted") == 0)
		wr_reset_cause(IMG_VERIFY_FAIL);
	else
		wr_reset_cause(OTHER_RESET_CAUSE);

	return NOTIFY_DONE;
}

static struct notifier_block plat_panic_notifier = {
	.notifier_call	= plat_panic_handler
};

static int plat_reboot_handler(struct notifier_block *nb, unsigned long reason, void *arg)
{
	/* In a reboot scenario, assuming that userspace triggered the reboot and will have to set the reset cause */

	return NOTIFY_DONE;
}

static struct notifier_block plat_reboot_notifier = {
	.notifier_call	= plat_reboot_handler
};

static ssize_t reset_cause_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", rd_reset_cause());
}

static ssize_t reset_cause_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (sscanf(buf, "%du", &reset_cause) == 1)
		wr_reset_cause(reset_cause);

	return count;
}

static struct kobj_attribute reset_cause_attribute = __ATTR(reset_cause, 0660, reset_cause_show, reset_cause_store);

static int __init err_handler_init(void)
{
	int ret = 0;

	err_kobj = kobject_create_and_add("err", kernel_kobj);

	if (!err_kobj)
		return -ENOMEM;

	ret = sysfs_create_file(err_kobj, &reset_cause_attribute.attr);
	if (ret) {
		pr_err("Failed to create the reset_cause file in /sys/kernel/err \n");
		return ret;
	}

	ret = register_reboot_notifier(&plat_reboot_notifier);
	if (ret) {
		pr_err("Unable to register reboot notifier\n");
		return ret;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &plat_panic_notifier);
	if (ret)
		pr_err("Unable to register panic notifier\n");

	return ret;
}

static void __exit err_handler_exit(void)
{
	kobject_put(err_kobj);
	sysfs_remove_file(err_kobj, &reset_cause_attribute.attr);

	unregister_reboot_notifier(&plat_reboot_notifier);
	atomic_notifier_chain_unregister(&panic_notifier_list, &plat_panic_notifier);
}

module_init(err_handler_init);
module_exit(err_handler_exit);

MODULE_AUTHOR("Analog Devices, Inc.");
MODULE_LICENSE("GPL v2");
