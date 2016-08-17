/*
 * FPGA Manager DebugFS
 *
 *  Copyright (C) 2016 Intel Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/debugfs.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

static struct dentry *fpga_mgr_debugfs_root;

struct fpga_mgr_debugfs {
	struct dentry *debugfs_dir;
	char *firmware_name;
	struct fpga_image_info info;
};

static ssize_t fpga_mgr_firmware_write_file(struct file *file,
					    const char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct fpga_manager *mgr = file->private_data;
	struct fpga_mgr_debugfs *debugfs = mgr->debugfs;
	char *buf;
	int ret;

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;
	if (buf[count - 1] == '\n')
		buf[count - 1] = 0;

	kfree(debugfs->firmware_name);
	debugfs->firmware_name = buf;

	ret = fpga_mgr_firmware_load(mgr, &debugfs->info, buf);
	if (ret)
		dev_err(&mgr->dev,
			"fpga_mgr_firmware_load returned with value %d\n", ret);

	return count;
}

static ssize_t fpga_mgr_firmware_read_file(struct file *file,
					   char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct fpga_manager *mgr = file->private_data;
	struct fpga_mgr_debugfs *debugfs = mgr->debugfs;
	char *buf;
	int ret;

	if (debugfs->firmware_name == NULL)
		return 0;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = snprintf(buf, PAGE_SIZE, "%s\n", debugfs->firmware_name);
	if (ret < 0) {
		kfree(buf);
		return ret;
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);
	kfree(buf);

	return ret;
}

static const struct file_operations fpga_mgr_firmware_fops = {
	.open = simple_open,
	.read = fpga_mgr_firmware_read_file,
	.write = fpga_mgr_firmware_write_file,
	.llseek = default_llseek,
};

static ssize_t fpga_mgr_image_write_file(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct fpga_manager *mgr = file->private_data;
	struct fpga_mgr_debugfs *debugfs = mgr->debugfs;
	char *buf;
	int ret;

	dev_info(&mgr->dev, "writing %d bytes to %s\n", count, mgr->name);

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	/* If firmware interface was previously used, forget it. */
	kfree(debugfs->firmware_name);
	debugfs->firmware_name = NULL;

	ret = fpga_mgr_buf_load(mgr, &debugfs->info, buf, count);
	if (ret)
		dev_err(&mgr->dev,
		       "fpga_mgr_buf_load returned with value %d\n", ret);

	return count;
}

static const struct file_operations fpga_mgr_image_fops = {
	.open = simple_open,
	.write = fpga_mgr_image_write_file,
	.llseek = default_llseek,
};

void fpga_mgr_debugfs_add(struct fpga_manager *mgr)
{
	struct fpga_mgr_debugfs *debugfs;
	struct fpga_image_info *info;

	if (!fpga_mgr_debugfs_root)
		return;

	debugfs = kzalloc(sizeof(*debugfs), GFP_KERNEL);
	if (!debugfs)
		return;

	debugfs->debugfs_dir = debugfs_create_dir(dev_name(&mgr->dev),
						  fpga_mgr_debugfs_root);

	debugfs_create_file("firmware_name", 0600, debugfs->debugfs_dir, mgr,
			    &fpga_mgr_firmware_fops);

	debugfs_create_file("image", 0200, debugfs->debugfs_dir, mgr,
			    &fpga_mgr_image_fops);

	info = &debugfs->info;
	debugfs_create_u32("flags", 0600, debugfs->debugfs_dir, &info->flags);

	mgr->debugfs = debugfs;
}

void fpga_mgr_debugfs_remove(struct fpga_manager *mgr)
{
	struct fpga_mgr_debugfs *debugfs = mgr->debugfs;

	if (!fpga_mgr_debugfs_root)
		return;

	debugfs_remove_recursive(debugfs->debugfs_dir);
	kfree(debugfs->firmware_name);
	kfree(debugfs);
}

void fpga_mgr_debugfs_init(void)
{
	fpga_mgr_debugfs_root = debugfs_create_dir("fpga_manager", NULL);
	if (!fpga_mgr_debugfs_root)
		pr_warn("fpga_mgr: Failed to create debugfs root\n");
}

void fpga_mgr_debugfs_uninit(void)
{
	debugfs_remove_recursive(fpga_mgr_debugfs_root);
}
