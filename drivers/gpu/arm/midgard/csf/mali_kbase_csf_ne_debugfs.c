// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2018-2024 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include <mali_kbase.h>
#include <csf/mali_kbase_csf_ne_debugfs.h>
#include <hw_access/mali_kbase_hw_access_regmap.h>
#include <mali_kbase_io.h>

#define BUF_SIZE 10

struct ne_control_field_data_s {
	int shift;
	int mask;
	int max_value;
};

static const struct ne_control_field_data_s ne_control_field_data[] = {
	{ NEURAL_CONTROL_LO_LATENCY_LIMIT_SHIFT, NEURAL_CONTROL_LO_LATENCY_LIMIT_MASK,
	  NEURAL_CONTROL_LATENCY_LIMIT_MAX_VALUE },
	{ NEURAL_CONTROL_HI_LATENCY_LIMIT_SHIFT, NEURAL_CONTROL_HI_LATENCY_LIMIT_MASK,
	  NEURAL_CONTROL_LATENCY_LIMIT_MAX_VALUE },
	{ NEURAL_CONTROL_MAC_STEP_CYCLES_SHIFT, NEURAL_CONTROL_MAC_STEP_CYCLES_MASK,
	  NEURAL_CONTROL_MAC_STEP_CYCLES_MAX_VALUE },
};

enum ne_control_field {
	NEURAL_CONTROL_LO_LATENCY_LIMIT,
	NEURAL_CONTROL_HI_LATENCY_LIMIT,
	NEURAL_CONTROL_MAC_STEP_CYCLES
};

static ssize_t kbase_csf_ne_control_get_field_value(struct file *file, char __user *buf,
						    size_t count, loff_t *ppos,
						    enum ne_control_field field)
{
	int size;
	char buffer[BUF_SIZE];
	struct kbase_device *kbdev = file->private_data;
	unsigned long flags;
	unsigned int field_value;
	int ret = 0;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (!kbase_io_is_gpu_powered(kbdev)) {
		dev_err(kbdev->dev, "The GPU is not powered on\n");
		ret = -EAGAIN;
	} else {
		field_value = (kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(NEURAL_CONTROL)) &
			       ne_control_field_data[field].mask) >>
			      ne_control_field_data[field].shift;
	}

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	if (!ret) {
		size = scnprintf(buffer, sizeof(buffer), "%u\n", field_value);
		ret = simple_read_from_buffer(buf, count, ppos, buffer, (size_t)size);
	}

	return ret;
}

static ssize_t kbase_csf_ne_control_set_field_value(struct file *file, const char __user *buf,
						    size_t count, loff_t *ppos,
						    enum ne_control_field field)
{
	struct kbase_device *kbdev = file->private_data;
	unsigned int latency_limit;
	unsigned int val;
	unsigned long flags;
	int ret = 0;

	CSTD_UNUSED(ppos);

	mutex_lock(&kbdev->pm.lock);
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (!kbase_io_is_gpu_powered(kbdev)) {
		dev_err(kbdev->dev, "The GPU is not powered on\n");
		ret = -EAGAIN;
	} else {
		ret = kstrtouint_from_user(buf, count, 10, &latency_limit);
		if (!ret) {
			if (latency_limit > ne_control_field_data[field].max_value)
				ret = -EINVAL;

			val = (kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(NEURAL_CONTROL)) &
			       ~ne_control_field_data[field].mask) |
			      (latency_limit << ne_control_field_data[field].shift);
			kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(NEURAL_CONTROL), val);
		}
	}

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	mutex_unlock(&kbdev->pm.lock);

	return ret ? ret : (ssize_t)count;
}

static ssize_t kbase_csf_ne_control_lo_latency_limit_get(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	return kbase_csf_ne_control_get_field_value(file, buf, count, ppos,
						    NEURAL_CONTROL_LO_LATENCY_LIMIT);
}

static ssize_t kbase_csf_ne_control_lo_latency_limit_set(struct file *file, const char __user *buf,
							 size_t count, loff_t *ppos)
{
	return kbase_csf_ne_control_set_field_value(file, buf, count, ppos,
						    NEURAL_CONTROL_LO_LATENCY_LIMIT);
}

static const struct file_operations kbase_csf_ne_control_lo_latency_limit_fops = {
	.owner = THIS_MODULE,
	.read = kbase_csf_ne_control_lo_latency_limit_get,
	.write = kbase_csf_ne_control_lo_latency_limit_set,
	.open = simple_open,
	.llseek = default_llseek
};

static ssize_t kbase_csf_ne_control_hi_latency_limit_get(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	return kbase_csf_ne_control_get_field_value(file, buf, count, ppos,
						    NEURAL_CONTROL_HI_LATENCY_LIMIT);
}

static ssize_t kbase_csf_ne_control_hi_latency_limit_set(struct file *file, const char __user *buf,
							 size_t count, loff_t *ppos)
{
	return kbase_csf_ne_control_set_field_value(file, buf, count, ppos,
						    NEURAL_CONTROL_HI_LATENCY_LIMIT);
}

static const struct file_operations kbase_csf_ne_control_hi_latency_limit_fops = {
	.owner = THIS_MODULE,
	.read = kbase_csf_ne_control_hi_latency_limit_get,
	.write = kbase_csf_ne_control_hi_latency_limit_set,
	.open = simple_open,
	.llseek = default_llseek
};

static ssize_t kbase_csf_ne_control_mac_step_cycles_get(struct file *file, char __user *buf,
							size_t count, loff_t *ppos)
{
	return kbase_csf_ne_control_get_field_value(file, buf, count, ppos,
						    NEURAL_CONTROL_MAC_STEP_CYCLES);
}

static ssize_t kbase_csf_ne_control_mac_step_cycles_set(struct file *file, const char __user *buf,
							size_t count, loff_t *ppos)
{
	return kbase_csf_ne_control_set_field_value(file, buf, count, ppos,
						    NEURAL_CONTROL_MAC_STEP_CYCLES);
}

static const struct file_operations kbase_csf_ne_control_mac_step_cycles_fops = {
	.owner = THIS_MODULE,
	.read = kbase_csf_ne_control_mac_step_cycles_get,
	.write = kbase_csf_ne_control_mac_step_cycles_set,
	.open = simple_open,
	.llseek = default_llseek
};

int kbase_csf_ne_control_debugfs_init(struct kbase_device *kbdev)
{
	struct dentry *ne_dir;
	struct dentry *file;
	const mode_t mode = 0644;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return 0;

	if (WARN_ON(IS_ERR_OR_NULL(kbdev->mali_debugfs_directory)))
		return -1;

	ne_dir = debugfs_create_dir("neural_control", kbdev->mali_debugfs_directory);
	if (IS_ERR_OR_NULL(ne_dir)) {
		dev_err(kbdev->dev, "Unable to create neural engine control debugfs directory\n");
		return -1;
	}

	file = debugfs_create_file("lo_latency_limit", mode, ne_dir, kbdev,
				   &kbase_csf_ne_control_lo_latency_limit_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev,
			 "Unable to create neural engine low latency limit debugfs entry");
		return -1;
	}

	file = debugfs_create_file("hi_latency_limit", mode, ne_dir, kbdev,
				   &kbase_csf_ne_control_hi_latency_limit_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev,
			 "Unable to create neural engine high latency limit debugfs entry");
		return -1;
	}

	file = debugfs_create_file("mac_step_cycles", mode, ne_dir, kbdev,
				   &kbase_csf_ne_control_mac_step_cycles_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev,
			 "Unable to create neural engine MAC step cycles debugfs entry");
		return -1;
	}

	return 0;
}
