/*
 * mwgeneric_of_overlay.c
 *
 *  Created on: Jan 8, 2016
 *      Author: mfornero
 */

#include <linux/firmware.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/idr.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/fpga/fpga-overlay.h>

#define DRIVER_NAME "fpga_overlay"

static struct class *fpga_overlay_class = NULL;
static const char fpga_overlay_name[] = "fpga_overlay";
static DEFINE_IDA(fpga_overlay_ida);

static void fpga_overlay_ida_remove(void *opaque){
	struct fpga_overlay_dev *overlay_dev = opaque;
	ida_simple_remove(&fpga_overlay_ida, overlay_dev->dev.id);
}

int fpga_overlay_is_fragment(struct device_node *fragment)
{
	struct property *prop;
	prop = of_find_property(fragment, "target", NULL);
	if (!prop) {
		prop = of_find_property(fragment, "target-path", NULL);
		if (!prop){
			return 0;
		}
	}
	return 1;
}

EXPORT_SYMBOL_GPL(fpga_overlay_is_fragment);

static int fpga_overlay_load_fw(struct fpga_overlay_dev *overlay_dev)
{
	int ret;

	if(!overlay_dev->firmware_name)
		return -EINVAL;

	ret = request_firmware(&overlay_dev->fw, overlay_dev->firmware_name, &overlay_dev->dev);

	return ret;
}

static void fpga_overlay_release_fw(void *opaque)
{
	struct fpga_overlay_dev *overlay_dev = opaque;
	release_firmware(overlay_dev->fw);
}

static void __of_platform_depopulate(void *opaque)
{
	struct fpga_overlay_dev *overlay_dev = opaque;
	if (overlay_dev->overlay_id >= 0){
		of_platform_depopulate(&overlay_dev->dev);
	}
}

static int __apply_overlay(struct fpga_overlay_dev *overlay_dev, struct device_node *overlay)
{

	int ret;

	if(!devres_open_group(&overlay_dev->dev, __apply_overlay, GFP_KERNEL))
		return -ENOMEM;

	ret = of_overlay_create(overlay);
	if (ret < 0) {
		dev_err(&overlay_dev->dev, "failed to create overlay\n");
		goto out;
	}
	overlay_dev->overlay_id = ret;

	ret = of_platform_populate(overlay_dev->dev.of_node, NULL, NULL,&overlay_dev->dev);
	if (ret){
		dev_err(&overlay_dev->dev,"Failed to populate platform\n");
		of_overlay_destroy(overlay_dev->overlay_id);
		goto out;
	}
	devm_add_action(&overlay_dev->dev,__of_platform_depopulate, overlay_dev);

out:
	devres_close_group(&overlay_dev->dev, __apply_overlay);
	return ret;
}

static void __free_overlay(void *opaque)
{
	struct device_node *overlay = opaque;
	kfree(overlay);
}

static int __fpga_overlay_apply(struct fpga_overlay_dev *overlay_dev)
{
	struct device_node *overlay = NULL, *fragment = NULL;
	int ret = 0;

	if (overlay_dev->overlay_id >= 0) {
		dev_err(&overlay_dev->dev, "Overlay already in use\n");
		return -EBUSY;
	}

	if (!devres_open_group(&overlay_dev->dev, __fpga_overlay_apply, GFP_KERNEL)){
		return -ENOMEM;
	}

	if(overlay_dev->ops->apply_init) {
		ret = overlay_dev->ops->apply_init(overlay_dev);
		if (ret){
			dev_err(&overlay_dev->dev,"Apply Init function failed\n");
			goto out_err;
		}
	}
	if(overlay_dev->ops->apply_term) {
		devm_add_action(&overlay_dev->dev, overlay_dev->ops->apply_term, overlay_dev);
	}

	if(overlay_dev->ops->load_fw) {
		ret = overlay_dev->ops->load_fw(overlay_dev);
	} else {
		ret = fpga_overlay_load_fw(overlay_dev);
	}
	if(ret){
		dev_err(&overlay_dev->dev,"Loading overlay firmware failed\n");
		goto out_err;
	}
	if(overlay_dev->ops->release_fw) {
		devm_add_action(&overlay_dev->dev, overlay_dev->ops->release_fw, overlay_dev);
	} else {
		devm_add_action(&overlay_dev->dev, fpga_overlay_release_fw, overlay_dev);
	}


	of_fdt_unflatten_tree((void *)overlay_dev->fw->data, &overlay);
	if (!overlay) {
		dev_err(&overlay_dev->dev,"Unflattening overlay tree failed\n");
		ret = -EINVAL;
		goto out_err;
	}
	devm_add_action(&overlay_dev->dev, __free_overlay, overlay);

	of_node_set_flag(overlay, OF_DETACHED);

	for_each_child_of_node(overlay, fragment) {
		if(fpga_overlay_is_fragment(fragment)) {
			if (overlay_dev->ops->update_fragment){
				ret = overlay_dev->ops->update_fragment(overlay_dev, fragment);
				if (ret){
					dev_err(&overlay_dev->dev,"Updating fragment failed\n");
					goto out_err;
				}
			}
		}
	}

	ret = of_resolve_phandles(overlay);
	if (ret) {
		dev_err(&overlay_dev->dev,"Failed to resolve phandles\n");
		goto out_err;
	}
	devres_close_group(&overlay_dev->dev, __fpga_overlay_apply);

	ret = __apply_overlay(overlay_dev, overlay);
	if (ret)
		goto out_err;

	return 0;

out_err:
	devres_release_group(&overlay_dev->dev, __fpga_overlay_apply);
	return ret;
}

int fpga_overlay_apply(struct fpga_overlay_dev *overlay_dev)
{
	int ret;
	mutex_lock(&overlay_dev->overlay_mutex);

	ret = __fpga_overlay_apply(overlay_dev);

	mutex_unlock(&overlay_dev->overlay_mutex);
	return ret;
}

EXPORT_SYMBOL_GPL(fpga_overlay_apply);

static int __fpga_overlay_remove(struct fpga_overlay_dev *overlay_dev){
	int ret;
	if(overlay_dev->overlay_id >= 0) {
		/* Release the overlay */
		devres_release_group(&overlay_dev->dev, __apply_overlay);
		ret = of_overlay_destroy(overlay_dev->overlay_id);
		if (ret) {
			dev_err(&overlay_dev->dev,"Failed to remove overlay\n");
			return ret;
		}
		overlay_dev->overlay_id = -ENODEV;

		/* Free the memory */
		devres_release_group(&overlay_dev->dev, __fpga_overlay_apply);
	} else {
		return -EALREADY;
	}

	return 0;
}

int fpga_overlay_remove(struct fpga_overlay_dev *overlay_dev)
{
	int ret = 0;

	mutex_lock(&overlay_dev->overlay_mutex);

	ret = __fpga_overlay_remove(overlay_dev);

	mutex_unlock(&overlay_dev->overlay_mutex);
	return ret;
}

EXPORT_SYMBOL_GPL(fpga_overlay_remove);

static ssize_t status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpga_overlay_dev *overlay_dev = to_fpga_overlay(dev);

	int status;

	mutex_lock(&overlay_dev->sysfs_mutex);

	if (sysfs_streq(buf, "apply")) {
		status = __fpga_overlay_apply(overlay_dev);
	} else if (sysfs_streq(buf, "idle") || sysfs_streq(buf, "remove")) {
		status = __fpga_overlay_remove(overlay_dev);
		if (status == -EALREADY)
			status = 0;
	} else
		status = -EINVAL;

	mutex_unlock(&overlay_dev->sysfs_mutex);

	return status ? status : count;
}

static ssize_t status_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct fpga_overlay_dev *overlay_dev = to_fpga_overlay(dev);
	ssize_t cnt;

	mutex_lock(&overlay_dev->sysfs_mutex);

	if(overlay_dev->overlay_id >= 0){
		cnt = sprintf(buf, "applied");
	} else {
		cnt = sprintf(buf, "idle");
	}

	mutex_unlock(&overlay_dev->sysfs_mutex);

	return cnt;
}

DEVICE_ATTR_RW(status);

static ssize_t firmware_file_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpga_overlay_dev *overlay_dev = to_fpga_overlay(dev);

	mutex_lock(&overlay_dev->sysfs_mutex);

	if(overlay_dev->firmware_name)
		devm_kfree(dev, overlay_dev->firmware_name);

	if(!count)
		return 0;

	overlay_dev->firmware_name = devm_kzalloc(dev, count, GFP_KERNEL);
	if(!overlay_dev->firmware_name && count)
		return -ENOMEM;

	/* Remove a trailing newline */
	if (buf[count-1] == '\n')
		((char *) buf)[count-1] = 0;

	strncpy(overlay_dev->firmware_name, buf, count);

	mutex_unlock(&overlay_dev->sysfs_mutex);

	return count;
}

static ssize_t firmware_file_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct fpga_overlay_dev *overlay_dev = to_fpga_overlay(dev);
	ssize_t cnt;

	mutex_lock(&overlay_dev->sysfs_mutex);

	if (!overlay_dev->firmware_name)
		return 0;

	cnt = sprintf(buf, "%s", overlay_dev->firmware_name);

	mutex_unlock(&overlay_dev->sysfs_mutex);

	return cnt;
}

DEVICE_ATTR_RW(firmware_file);

static ssize_t name_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct fpga_overlay_dev *overlay_dev = to_fpga_overlay(dev);

	return sprintf(buf, "%s", overlay_dev->name);
}

static DEVICE_ATTR_RO(name);

static struct attribute *fpga_overlay_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_firmware_file.attr,
	&dev_attr_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fpga_overlay);

struct fpga_overlay_dev *fpga_overlay_dev_register(struct device * dev, char *name, struct fpga_overlay_ops *ops, void *priv){
	struct fpga_overlay_dev *overlay_dev;

	int rc, id;

	if (fpga_overlay_class == NULL)
		return ERR_PTR(-EPROBE_DEFER);

	if (!ops || !ops->update_fragment) {
		dev_err(dev, "Attempt to register without fpga_overlay_ops\n");
		return ERR_PTR(-EINVAL);
	}

	if (!name || !strlen(name)) {
		dev_err(dev, "Attempt to register with no name!\n");
		return ERR_PTR(-EINVAL);
	}

	if(!devres_open_group(dev, fpga_overlay_dev_register, GFP_KERNEL))
		return ERR_PTR(-ENOMEM);

	overlay_dev =  devm_kzalloc(dev, sizeof(struct fpga_overlay_dev), GFP_KERNEL);
	if(!overlay_dev) {
		return ERR_PTR(-ENOMEM);
	}

	id = ida_simple_get(&fpga_overlay_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		return ERR_PTR(id);
	}

	overlay_dev->name = name;
	overlay_dev->parent = dev;
	overlay_dev->ops = ops;
	overlay_dev->priv = priv;
	overlay_dev->overlay_id = -ENODEV;

	mutex_init(&overlay_dev->sysfs_mutex);
	mutex_init(&overlay_dev->overlay_mutex);

	device_initialize(&overlay_dev->dev);
	overlay_dev->dev.class = fpga_overlay_class;
	overlay_dev->dev.parent = dev;
	overlay_dev->dev.of_node = dev->of_node;
	overlay_dev->dev.id = id;
	devm_add_action(dev,fpga_overlay_ida_remove, overlay_dev);

	rc = dev_set_name(&overlay_dev->dev, "%s%d", fpga_overlay_name, id);
	if (rc)
		return ERR_PTR(rc);

	rc = device_add(&overlay_dev->dev);
	if (rc)
		return ERR_PTR(rc);

	devres_close_group(dev, fpga_overlay_dev_register);

	overlay_dev->firmware_name = devm_kstrdup(&overlay_dev->dev, "",GFP_KERNEL);
	if(!overlay_dev->firmware_name){
		return ERR_PTR(-ENOMEM);
	}

	return overlay_dev;
}

EXPORT_SYMBOL_GPL(fpga_overlay_dev_register);

void fpga_overlay_dev_unregister(struct fpga_overlay_dev *overlay_dev){

	if (overlay_dev->overlay_id >= 0) {
		dev_warn(&overlay_dev->dev,"Removing active overlay before releasing\n");
		fpga_overlay_remove(overlay_dev);
	}

	device_unregister(&overlay_dev->dev);
}

EXPORT_SYMBOL_GPL(fpga_overlay_dev_unregister);


static void fpga_overlay_dev_release(struct device *dev)
{
	struct fpga_overlay_dev *overlay_dev = to_fpga_overlay(dev);

	devres_release_group(overlay_dev->parent, fpga_overlay_dev_register);

}

static int __init fpga_overlay_init(void)
{
	fpga_overlay_class = class_create(THIS_MODULE, fpga_overlay_name);
	if (IS_ERR(fpga_overlay_class))
		return PTR_ERR(fpga_overlay_class);

	fpga_overlay_class->dev_groups = fpga_overlay_groups;
	fpga_overlay_class->dev_release = fpga_overlay_dev_release;

	return 0;
}

static void __exit fpga_overlay_exit(void)
{

	class_destroy(fpga_overlay_class);
	fpga_overlay_class = NULL;
}

module_init(fpga_overlay_init);
module_exit(fpga_overlay_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": FPGA Overlay Manager");
MODULE_ALIAS(DRIVER_NAME);
