// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADV7511/33/35 HDMI transmitter driver
 *
 * Copyright (C) 2018-2019 Analog Devices, All Rights Reserved.
 */

#include <linux/types.h>
#include <linux/debugfs.h>
#include <drm/drm_print.h>

#include "adv7511.h"

static int tpg_show(struct seq_file *m, void *data)
{
	struct adv7511 *adv = m->private;
	unsigned int val;
	int ret;

	ret = regmap_read(adv->regmap_cec, ADV7533_REG_TPG, &val);
	if (ret)
		return ret;

	if (!(val & ADV7533_TPG_ENABLE))
		seq_puts(m, "off\n");
	else if (val & ADV7533_TPG_RAMP)
		seq_puts(m, "ramp\n");
	else if (val & ADV7533_TPG_ENABLE)
		seq_puts(m, "colorbar\n");

	return 0;
}

static int tpg_open(struct inode *inode, struct file *file)
{
	struct drm_connector *dev = inode->i_private;

	return single_open(file, tpg_show, dev);
}

static ssize_t tpg_write(struct file *file, const char __user *ubuf,
			 size_t len, loff_t *offp)
{
	struct seq_file *m = file->private_data;
	struct adv7511 *adv = m->private;
	char buf[9];
	int ret;

	if (len > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;

	buf[len] = '\0';

	if (sysfs_streq(buf, "colorbar"))
		ret = regmap_update_bits(adv->regmap_cec, ADV7533_REG_TPG, 0xE0,
				   ADV7533_TPG_ENABLE | ADV7533_TPG_COLORBAR);
	else if (sysfs_streq(buf, "ramp"))
		ret = regmap_update_bits(adv->regmap_cec, ADV7533_REG_TPG, 0xE0,
				   ADV7533_TPG_ENABLE | ADV7533_TPG_RAMP);
	else if (sysfs_streq(buf, "off"))
		ret = regmap_update_bits(adv->regmap_cec, ADV7533_REG_TPG,
				   ADV7533_TPG_ENABLE, ~ADV7533_TPG_ENABLE);
	else
		return -EINVAL;

	if (ret)
		return ret;

	return len;
}

static const struct file_operations adv7511_tpg_fops = {
	.owner = THIS_MODULE,
	.open = tpg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = tpg_write
};

int adv7511_debugfs_init(struct adv7511 *adv7511)
{
	struct drm_bridge *bridge = &adv7511->bridge;
	struct drm_minor *minor = bridge->dev->primary;
	struct dentry *debugfs_file;

	adv7511->debugfs = debugfs_create_dir("adv7511", minor->debugfs_root);
	if (IS_ERR_OR_NULL(adv7511->debugfs))
		return -ENOMEM;

	/* tpg only on ADV7533/35*/
	if (adv7511->type == ADV7533 || adv7511->type == ADV7535) {
		debugfs_file = debugfs_create_file("tpg", 0644,
				adv7511->debugfs, adv7511, &adv7511_tpg_fops);

		if (IS_ERR_OR_NULL(debugfs_file))
			DRM_ERROR("Failed to create TPG debugfs file\n");
	}

	return 0;
}

void adv7511_debugfs_remove(struct adv7511 *adv7511)
{
	debugfs_remove_recursive(adv7511->debugfs);
}
