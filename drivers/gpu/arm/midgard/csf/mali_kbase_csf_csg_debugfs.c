// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2019-2024 ARM Limited. All rights reserved.
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

#include "mali_kbase_csf_csg_debugfs.h"

#if IS_ENABLED(CONFIG_DEBUG_FS)
#include "mali_kbase_csf_csg.h"
#include "mali_kbase_csf_tl_reader.h"
#include "mali_kbase_csf_util.h"
#include <mali_kbase.h>
#include <linux/seq_file.h>
#include <linux/version_compat_defs.h>
#include <mali_kbase_reset_gpu.h>
#include <mali_kbase_config_defaults.h>

#define MAX_SCHED_STATE_STRING_LEN (16)
/**
 * scheduler_state_to_string() - Get string name of scheduler state.
 *
 * @kbdev:       Pointer to kbase device.
 * @sched_state: Scheduler state.
 *
 * Return: Suitable string.
 */
static const char *scheduler_state_to_string(struct kbase_device *kbdev,
					     enum kbase_csf_scheduler_state sched_state)
{
	switch (sched_state) {
	case SCHED_BUSY:
		return "BUSY";
	case SCHED_INACTIVE:
		return "INACTIVE";
	case SCHED_SUSPENDED:
		return "SUSPENDED";
#ifdef KBASE_PM_RUNTIME
	case SCHED_SLEEPING:
		return "SLEEPING";
#endif
	default:
		dev_warn(kbdev->dev, "Unknown Scheduler state %d", sched_state);
		return NULL;
	}
}

/**
 * kbasep_csf_queue_show_groups() - Print per-context GPU command queue
 *                                  group debug information
 *
 * @file: The seq_file for printing to
 * @data: The debugfs dentry private data, a pointer to kbase context
 *
 * Return: Negative error code or 0 on success.
 */
static int kbasep_csf_queue_show_groups(struct seq_file *file, void *data)
{
	struct kbasep_printer *kbpr;
	struct kbase_context *const kctx = file->private;
	int ret = -EINVAL;
	CSTD_UNUSED(data);

	kbpr = kbasep_printer_file_init(file);
	if (kbpr != NULL) {
		ret = kbasep_csf_csg_dump_print(kctx, kbpr);
		kbasep_printer_term(kbpr);
	}

	return ret;
}

/**
 * kbasep_csf_csg_active_show_groups() - Print debug info for active GPU command queue groups
 *
 * @file: The seq_file for printing to
 * @data: The debugfs dentry private data, a pointer to kbase_device
 *
 * Return: Negative error code or 0 on success.
 */
static int kbasep_csf_csg_active_show_groups(struct seq_file *file, void *data)
{
	struct kbase_device *kbdev = file->private;
	struct kbasep_printer *kbpr;
	int ret = -EINVAL;
	CSTD_UNUSED(data);

	kbpr = kbasep_printer_file_init(file);
	if (kbpr != NULL) {
		ret = kbasep_csf_csg_active_dump_print(kbdev, kbpr);
		kbasep_printer_term(kbpr);
	}

	return ret;
}

static int kbasep_csf_queue_group_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, kbasep_csf_queue_show_groups, in->i_private);
}

static int kbasep_csf_active_queue_groups_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, kbasep_csf_csg_active_show_groups, in->i_private);
}

static const struct file_operations kbasep_csf_queue_group_debugfs_fops = {
	.open = kbasep_csf_queue_group_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void kbase_csf_queue_group_debugfs_init(struct kbase_context *kctx)
{
	struct dentry *file;
	const mode_t mode = 0444;

	if (WARN_ON(!kctx || IS_ERR_OR_NULL(kctx->kctx_dentry)))
		return;

	file = debugfs_create_file("groups", mode, kctx->kctx_dentry, kctx,
				   &kbasep_csf_queue_group_debugfs_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kctx->kbdev->dev,
			 "Unable to create per context queue groups debugfs entry");
	}
}

static const struct file_operations kbasep_csf_active_queue_groups_debugfs_fops = {
	.open = kbasep_csf_active_queue_groups_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int kbasep_csf_debugfs_scheduling_timer_enabled_get(void *data, u64 *val)
{
	struct kbase_device *const kbdev = data;

	*val = kbase_csf_scheduler_timer_is_enabled(kbdev);

	return 0;
}

static int kbasep_csf_debugfs_scheduling_timer_enabled_set(void *data, u64 val)
{
	struct kbase_device *const kbdev = data;

	kbase_csf_scheduler_timer_set_enabled(kbdev, val != 0);

	return 0;
}

static int kbasep_csf_debugfs_scheduling_timer_kick_set(void *data, u64 val)
{
	struct kbase_device *const kbdev = data;
	CSTD_UNUSED(val);

	kbase_csf_scheduler_kick(kbdev);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(kbasep_csf_debugfs_scheduling_timer_enabled_fops,
			 &kbasep_csf_debugfs_scheduling_timer_enabled_get,
			 &kbasep_csf_debugfs_scheduling_timer_enabled_set, "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(kbasep_csf_debugfs_scheduling_timer_kick_fops, NULL,
			 &kbasep_csf_debugfs_scheduling_timer_kick_set, "%llu\n");

/**
 * kbase_csf_debugfs_scheduler_state_get() - Get the state of scheduler.
 *
 * @file:     Object of the file that is being read.
 * @user_buf: User buffer that contains the string.
 * @count:    Length of user buffer
 * @ppos:     Offset within file object
 *
 * This function will return the current Scheduler state to Userspace
 * Scheduler may exit that state by the time the state string is received
 * by the Userspace.
 *
 * Return: 0 if Scheduler was found in an unexpected state, or the
 *         size of the state string if it was copied successfully to the
 *         User buffer or a negative value in case of an error.
 */
static ssize_t kbase_csf_debugfs_scheduler_state_get(struct file *file, char __user *user_buf,
						     size_t count, loff_t *ppos)
{
	struct kbase_device *kbdev = file->private_data;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	const char *state_string;

	kbase_csf_scheduler_lock(kbdev);
	state_string = scheduler_state_to_string(kbdev, scheduler->state);
	kbase_csf_scheduler_unlock(kbdev);

	if (!state_string)
		count = 0;

	return simple_read_from_buffer(user_buf, count, ppos, state_string, strlen(state_string));
}

/**
 * kbase_csf_debugfs_scheduler_state_set() - Set the state of scheduler.
 *
 * @file:  Object of the file that is being written to.
 * @ubuf:  User buffer that contains the string.
 * @count: Length of user buffer
 * @ppos:  Offset within file object
 *
 * This function will update the Scheduler state as per the state string
 * passed by the Userspace. Scheduler may or may not remain in new state
 * for long.
 *
 * Return: Negative value if the string doesn't correspond to a valid Scheduler
 *         state or if copy from user buffer failed, otherwise the length of
 *         the User buffer.
 */
static ssize_t kbase_csf_debugfs_scheduler_state_set(struct file *file, const char __user *ubuf,
						     size_t count, loff_t *ppos)
{
	struct kbase_device *kbdev = file->private_data;
	char buf[MAX_SCHED_STATE_STRING_LEN];

	CSTD_UNUSED(ppos);

	count = min_t(size_t, sizeof(buf) - 1, count);
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = 0;

	if (sysfs_streq(buf, "SUSPENDED"))
		kbase_csf_scheduler_pm_suspend(kbdev);
#ifdef KBASE_PM_RUNTIME
	else if (sysfs_streq(buf, "SLEEPING"))
		kbase_csf_scheduler_force_sleep(kbdev);
#endif
	else if (sysfs_streq(buf, "INACTIVE"))
		kbase_csf_scheduler_force_wakeup(kbdev);
	else {
		dev_dbg(kbdev->dev, "Bad scheduler state %s", buf);
		return -EINVAL;
	}

	return (ssize_t)count;
}

static const struct file_operations kbasep_csf_debugfs_scheduler_state_fops = {
	.owner = THIS_MODULE,
	.read = kbase_csf_debugfs_scheduler_state_get,
	.write = kbase_csf_debugfs_scheduler_state_set,
	.open = simple_open,
	.llseek = default_llseek,
};
static int kbasep_csf_debugfs_eviction_timeout_get(void *data, u64 *val)
{
	struct kbase_device *const kbdev = data;
	unsigned long flags;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	*val = kbdev->csf.csg_suspend_timeout_ms - CSG_SUSPEND_TIMEOUT_HOST_ADDED_MS;
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	return 0;
}

static int kbasep_csf_debugfs_eviction_timeout_set(void *data, u64 val)
{
	struct kbase_device *const kbdev = data;
	unsigned long flags_schd, flags_hw;
	u64 dur_ms = val;
	int ret = 0;

	if (unlikely(dur_ms < CSG_SUSPEND_TIMEOUT_FIRMWARE_MS_MIN ||
		     dur_ms > CSG_SUSPEND_TIMEOUT_FIRMWARE_MS_MAX)) {
		dev_err(kbdev->dev, "Invalid CSG suspend timeout input (%llu)", dur_ms);
		return -EFAULT;
	}
	dur_ms = dur_ms + CSG_SUSPEND_TIMEOUT_HOST_ADDED_MS;

	/* The 'fw_load_lock' is taken to synchronize against the deferred
	 * loading of FW, update will take effect after firmware gets loaded.
	 */
	mutex_lock(&kbdev->fw_load_lock);
	if (unlikely(!kbdev->csf.firmware_inited)) {
		kbase_csf_scheduler_spin_lock(kbdev, &flags_schd);
		kbdev->csf.csg_suspend_timeout_ms = (unsigned int)dur_ms;
		kbase_csf_scheduler_spin_unlock(kbdev, flags_schd);
		mutex_unlock(&kbdev->fw_load_lock);
		dev_info(kbdev->dev, "CSF set csg suspend timeout deferred till fw is loaded");
		goto end;
	}
	mutex_unlock(&kbdev->fw_load_lock);

	/* Firmware reloading is triggered by silent reset, and then update will take effect.
	 */
	kbase_csf_scheduler_pm_active(kbdev);
	if (kbase_csf_scheduler_killable_wait_mcu_active(kbdev)) {
		dev_err(kbdev->dev,
			"Unable to activate the MCU, the csg suspend timeout value shall remain unchanged");
		kbase_csf_scheduler_pm_idle(kbdev);
		ret = -EFAULT;
		goto exit;
	}
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags_hw);
	if (kbase_reset_gpu_silent(kbdev)) {
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags_hw);
		dev_err(kbdev->dev, "CSF set csg suspend timeout pending reset, try again");
		kbase_csf_scheduler_pm_idle(kbdev);
		ret = -EFAULT;
		goto exit;
	}
	/* GPU reset is placed and it will take place only after hwaccess_lock is released,
	 * update on host side should be done after GPU reset is placed and before it takes place.
	 */
	kbase_csf_scheduler_spin_lock(kbdev, &flags_schd);
	kbdev->csf.csg_suspend_timeout_ms = (unsigned int)dur_ms;
	kbase_csf_scheduler_spin_unlock(kbdev, flags_schd);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags_hw);
	/* Keep PM active until reset finished to allow FW reloading to take place,
	 * and then update request will be sent to FW during initialization.
	 */
	kbase_reset_gpu_wait(kbdev);
	kbase_csf_scheduler_pm_idle(kbdev);

end:
	dev_info(kbdev->dev, "CSF set csg suspend timeout: %u ms", (unsigned int)dur_ms);

exit:
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(kbasep_csf_debugfs_eviction_timeout_fops,
			 &kbasep_csf_debugfs_eviction_timeout_get,
			 &kbasep_csf_debugfs_eviction_timeout_set, "%llu\n");

void kbase_csf_debugfs_init(struct kbase_device *kbdev)
{
	debugfs_create_file("active_groups", 0444, kbdev->mali_debugfs_directory, kbdev,
			    &kbasep_csf_active_queue_groups_debugfs_fops);

	debugfs_create_file("scheduling_timer_enabled", 0644, kbdev->mali_debugfs_directory, kbdev,
			    &kbasep_csf_debugfs_scheduling_timer_enabled_fops);
	debugfs_create_file("scheduling_timer_kick", 0200, kbdev->mali_debugfs_directory, kbdev,
			    &kbasep_csf_debugfs_scheduling_timer_kick_fops);
	debugfs_create_file("scheduler_state", 0644, kbdev->mali_debugfs_directory, kbdev,
			    &kbasep_csf_debugfs_scheduler_state_fops);
	debugfs_create_file("eviction_timeout_ms", 0644, kbdev->mali_debugfs_directory, kbdev,
			    &kbasep_csf_debugfs_eviction_timeout_fops);

	kbase_csf_tl_reader_debugfs_init(kbdev);
}

#else
/*
 * Stub functions for when debugfs is disabled
 */
void kbase_csf_queue_group_debugfs_init(struct kbase_context *kctx)
{
}

void kbase_csf_debugfs_init(struct kbase_device *kbdev)
{
}

#endif /* CONFIG_DEBUG_FS */
