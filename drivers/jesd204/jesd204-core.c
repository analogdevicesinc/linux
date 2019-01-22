// SPDX-License-Identifier: GPL-2.0+
/**
 * The JESD204 framework - core logic
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#define pr_fmt(fmt) "jesd204: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "jesd204-priv.h"

static DEFINE_MUTEX(jesd204_device_list_lock);
static LIST_HEAD(jesd204_device_list);

static unsigned int jesd204_device_count;

static struct jesd204_dev *jesd204_dev_alloc(struct device_node *np)
{
	struct jesd204_dev *jdev;

	jdev = kzalloc(sizeof(*jdev), GFP_KERNEL);
	if (!jdev)
		return ERR_PTR(-ENOMEM);

	kref_get(&jdev->ref);

	jdev->np = of_node_get(np);
	kref_init(&jdev->ref);

	list_add(&jdev->list, &jesd204_device_list);
	jesd204_device_count++;

	return jdev;
}

static int jesd204_of_create_devices(void)
{
	struct jesd204_dev *jdev;
	struct device_node *np;
	int ret;

	mutex_lock(&jesd204_device_list_lock);

	ret = 0;
	for_each_node_with_property(np, "jesd204-device") {
		jdev = jesd204_dev_alloc(np);
		if (IS_ERR(jdev)) {
			ret = PTR_ERR(jdev);
			goto unlock;
		}
	}

unlock:
	mutex_unlock(&jesd204_device_list_lock);

	return ret;
}

static void jesd204_of_unregister_devices(void)
{
	struct jesd204_dev *jdev, *j;

	list_for_each_entry_safe(jdev, j, &jesd204_device_list, list) {
		jesd204_dev_unregister(jdev);
	}
}

/* Free memory allocated. */
static void __jesd204_dev_release(struct kref *ref)
{
	struct jesd204_dev *jdev = container_of(ref, struct jesd204_dev, ref);

	mutex_lock(&jesd204_device_list_lock);

	list_del(&jdev->list);
	of_node_put(jdev->np);

	kfree(jdev);

	jesd204_device_count--;

	mutex_unlock(&jesd204_device_list_lock);
}

/**
 * jesd204_dev_unregister() - unregister a device from the JESD204 subsystem
 * @jdev:		Device structure representing the device.
 **/
void jesd204_dev_unregister(struct jesd204_dev *jdev)
{
	if (IS_ERR_OR_NULL(jdev))
		return;

	kref_put(&jdev->ref, __jesd204_dev_release);
}
EXPORT_SYMBOL(jesd204_dev_unregister);

static int __init jesd204_init(void)
{
	int ret;

	mutex_init(&jesd204_device_list_lock);

	ret = jesd204_of_create_devices();
	if (ret < 0)
		goto error_unreg_devices;

	return 0;

error_unreg_devices:
	jesd204_of_unregister_devices();
	pr_err("framework error: %d\n", ret);
	return ret;
}

static void __exit jesd204_exit(void)
{
	jesd204_of_unregister_devices();
	mutex_destroy(&jesd204_device_list_lock);
}

subsys_initcall(jesd204_init);
module_exit(jesd204_exit);

MODULE_AUTHOR("Alexandru Ardelean <alexandru.ardelean@analog.com>");
MODULE_DESCRIPTION("JESD204 framework core");
MODULE_LICENSE("GPL");
