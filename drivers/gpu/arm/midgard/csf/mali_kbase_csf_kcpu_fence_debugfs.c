// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2018-2023 ARM Limited. All rights reserved.
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
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#endif

#include <mali_kbase.h>
#include <csf/mali_kbase_csf_kcpu_fence_debugfs.h>
#include <mali_kbase_hwaccess_time.h>

#define BUF_SIZE 10

#if IS_ENABLED(CONFIG_DEBUG_FS)
static ssize_t kbase_csf_kcpu_queue_fence_signal_enabled_get(struct file *file, char __user *buf,
							     size_t count, loff_t *ppos)
{
	int ret;
	struct kbase_device *kbdev = file->private_data;

	if (atomic_read(&kbdev->fence_signal_timeout_enabled))
		ret = simple_read_from_buffer(buf, count, ppos, "1\n", 2);
	else
		ret = simple_read_from_buffer(buf, count, ppos, "0\n", 2);

	return ret;
};

static ssize_t kbase_csf_kcpu_queue_fence_signal_enabled_set(struct file *file,
							     const char __user *buf, size_t count,
							     loff_t *ppos)
{
	int ret;
	unsigned int enabled;
	struct kbase_device *kbdev = file->private_data;

	CSTD_UNUSED(ppos);

	ret = kstrtouint_from_user(buf, count, 10, &enabled);
	if (ret < 0)
		return ret;

	atomic_set(&kbdev->fence_signal_timeout_enabled, (int)enabled);

	return (ssize_t)count;
}

static const struct file_operations kbase_csf_kcpu_queue_fence_signal_fops = {
	.owner = THIS_MODULE,
	.read = kbase_csf_kcpu_queue_fence_signal_enabled_get,
	.write = kbase_csf_kcpu_queue_fence_signal_enabled_set,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t kbase_csf_kcpu_queue_fence_signal_timeout_get(struct file *file, char __user *buf,
							     size_t count, loff_t *ppos)
{
	int size;
	char buffer[BUF_SIZE];
	struct kbase_device *kbdev = file->private_data;
	unsigned int timeout_ms = kbase_get_timeout_ms(kbdev, KCPU_FENCE_SIGNAL_TIMEOUT);

	size = scnprintf(buffer, sizeof(buffer), "%u\n", timeout_ms);
	return simple_read_from_buffer(buf, count, ppos, buffer, (size_t)size);
}

static ssize_t kbase_csf_kcpu_queue_fence_signal_timeout_set(struct file *file,
							     const char __user *buf, size_t count,
							     loff_t *ppos)
{
	int ret;
	unsigned int timeout_ms;
	struct kbase_device *kbdev = file->private_data;

	CSTD_UNUSED(ppos);

	ret = kstrtouint_from_user(buf, count, 10, &timeout_ms);
	if (ret < 0)
		return ret;

	/* The timeout passed by the user is bounded when trying to insert it into
	 * the precomputed timeout table, so we don't need to do any more validation
	 * before-hand.
	 */
	kbase_device_set_timeout_ms(kbdev, KCPU_FENCE_SIGNAL_TIMEOUT, timeout_ms);

	return (ssize_t)count;
}

static const struct file_operations kbase_csf_kcpu_queue_fence_signal_timeout_fops = {
	.owner = THIS_MODULE,
	.read = kbase_csf_kcpu_queue_fence_signal_timeout_get,
	.write = kbase_csf_kcpu_queue_fence_signal_timeout_set,
	.open = simple_open,
	.llseek = default_llseek,
};

int kbase_csf_fence_timer_debugfs_init(struct kbase_device *kbdev)
{
	struct dentry *file;
	const mode_t mode = 0644;

	if (WARN_ON(IS_ERR_OR_NULL(kbdev->mali_debugfs_directory)))
		return -1;

	file = debugfs_create_file("fence_signal_timeout_enable", mode,
				   kbdev->mali_debugfs_directory, kbdev,
				   &kbase_csf_kcpu_queue_fence_signal_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev, "Unable to create fence signal timer toggle entry");
		return -1;
	}

	file = debugfs_create_file("fence_signal_timeout_ms", mode, kbdev->mali_debugfs_directory,
				   kbdev, &kbase_csf_kcpu_queue_fence_signal_timeout_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev, "Unable to create fence signal timeout entry");
		return -1;
	}
	return 0;
}

#else
int kbase_csf_fence_timer_debugfs_init(struct kbase_device *kbdev)
{
	CSTD_UNUSED(kbdev);
	return 0;
}

#endif
void kbase_csf_fence_timer_debugfs_term(struct kbase_device *kbdev)
{
	CSTD_UNUSED(kbdev);
}
