// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADV7511/33/35 HDMI transmitter driver
 *
 * Copyright (C) 2018-2019 Analog Devices, All Rights Reserved.
 *
 */

#include <linux/types.h>
#include <linux/debugfs.h>
#include <drm/drm_print.h>

#include "adv7511.h"

static int tpg_show(struct seq_file *m, void *data)
{
	struct adv7511 *adv = m->private;
	unsigned int val;

	regmap_read(adv->regmap_cec, ADV7533_REG_TPG, &val);

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

	if (len > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;

	buf[len] = '\0';

	if (sysfs_streq(buf, "colorbar"))
		regmap_update_bits(adv->regmap_cec, ADV7533_REG_TPG, 0xE0,
				   ADV7533_TPG_ENABLE | ADV7533_TPG_COLORBAR);
	else if (sysfs_streq(buf, "ramp"))
		regmap_update_bits(adv->regmap_cec, ADV7533_REG_TPG, 0xE0,
				   ADV7533_TPG_ENABLE | ADV7533_TPG_RAMP);
	else if (sysfs_streq(buf, "off"))
		regmap_update_bits(adv->regmap_cec, ADV7533_REG_TPG,
				   ADV7533_TPG_ENABLE, ~ADV7533_TPG_ENABLE);
	else
		return -EINVAL;

	return len;
}

static int hdcp_status_show(struct seq_file *m, void *data)
{
	struct adv7511 *adv = m->private;
	struct adv7511_hdcp *hdcp = &adv->hdcp;
	unsigned int val;

	static const char * const states[] = {
		"in reset",
		"reading EDID",
		"idle",
		"initializing HDCP",
		"HDCP enabled",
		"initializing HDCP repeater",
		"6", "7", "8", "9", "A", "B", "C", "D", "E", "F"
	};
	static const char * const errors[] = {
		"no error",
		"bad receiver BKSV",
		"Ri mismatch",
		"Pj mismatch",
		"i2c error",
		"timed out",
		"max repeater cascade exceeded",
		"hash check failed",
		"too many devices",
		"9", "A", "B", "C", "D", "E", "F"
	};

	regmap_read(adv->regmap, ADV7511_REG_DDC_STATUS, &val);

	seq_printf(m, "HDCP Controller Error: ");
	seq_printf(m, "%s\n", errors[val >> 4]);

	seq_printf(m, "HDCP Controller State: ");
	seq_printf(m, "%s\n", states[val & 0xf]);

	seq_printf(m, "HDCP KSV FIFO: \n");

	if (hdcp->ksv_fifo)
		seq_hex_dump(m, "  ", DUMP_PREFIX_OFFSET, 16, 1, hdcp->ksv_fifo,
			     hdcp->ksv_count * DRM_HDCP_KSV_LEN, false);

	return 0;
}

static int hdcp_status_open(struct inode *inode, struct file *file)
{
	struct drm_connector *dev = inode->i_private;

	return single_open(file, hdcp_status_show, dev);
}

static const struct file_operations adv7511_tpg_fops = {
	.owner = THIS_MODULE,
	.open = tpg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = tpg_write
};

static const struct file_operations adv7511_hdcp_status_fops = {
	.owner = THIS_MODULE,
	.open = hdcp_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int adv7511_debugfs_init(struct adv7511 *adv7511)
{
	struct drm_bridge *bridge = &adv7511->bridge;
	struct drm_minor *minor = bridge->dev->primary;

	adv7511->debugfs = debugfs_create_dir("adv7511", minor->debugfs_root);
	if (IS_ERR_OR_NULL(adv7511->debugfs))
		return -ENOMEM;

	if (adv7511->type == ADV7533 || adv7511->type == ADV7535)
	{
		/* tpg */
		debugfs_create_file("tpg", 0644, adv7511->debugfs, adv7511,
				    &adv7511_tpg_fops);
	}

	debugfs_create_file("hdcp", 0644, adv7511->debugfs, adv7511,
			    &adv7511_hdcp_status_fops);
	return 0;
}

void adv7511_debugfs_remove(struct adv7511 *adv7511)
{
	debugfs_remove_recursive(adv7511->debugfs);
}
