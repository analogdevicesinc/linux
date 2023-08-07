// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2022 ARM Limited. All rights reserved.
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

#include <mali_kbase.h>
#include <csf/mali_kbase_csf_firmware_log.h>
#include <csf/mali_kbase_csf_trace_buffer.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#if defined(CONFIG_DEBUG_FS)

static int kbase_csf_firmware_log_enable_mask_read(void *data, u64 *val)
{
	struct kbase_device *kbdev = (struct kbase_device *)data;
	struct firmware_trace_buffer *tb =
		kbase_csf_firmware_get_trace_buffer(kbdev, FIRMWARE_LOG_BUF_NAME);

	if (tb == NULL) {
		dev_err(kbdev->dev, "Couldn't get the firmware trace buffer");
		return -EIO;
	}
	/* The enabled traces limited to u64 here, regarded practical */
	*val = kbase_csf_firmware_trace_buffer_get_active_mask64(tb);
	return 0;
}

static int kbase_csf_firmware_log_enable_mask_write(void *data, u64 val)
{
	struct kbase_device *kbdev = (struct kbase_device *)data;
	struct firmware_trace_buffer *tb =
		kbase_csf_firmware_get_trace_buffer(kbdev, FIRMWARE_LOG_BUF_NAME);
	u64 new_mask;
	unsigned int enable_bits_count;

	if (tb == NULL) {
		dev_err(kbdev->dev, "Couldn't get the firmware trace buffer");
		return -EIO;
	}

	/* Ignore unsupported types */
	enable_bits_count = kbase_csf_firmware_trace_buffer_get_trace_enable_bits_count(tb);
	if (enable_bits_count > 64) {
		dev_dbg(kbdev->dev, "Limit enabled bits count from %u to 64", enable_bits_count);
		enable_bits_count = 64;
	}
	new_mask = val & ((1 << enable_bits_count) - 1);

	if (new_mask != kbase_csf_firmware_trace_buffer_get_active_mask64(tb))
		return kbase_csf_firmware_trace_buffer_set_active_mask64(tb, new_mask);
	else
		return 0;
}

static int kbasep_csf_firmware_log_debugfs_open(struct inode *in, struct file *file)
{
	struct kbase_device *kbdev = in->i_private;

	file->private_data = kbdev;
	dev_dbg(kbdev->dev, "Opened firmware trace buffer dump debugfs file");

	return 0;
}

static ssize_t kbasep_csf_firmware_log_debugfs_read(struct file *file, char __user *buf,
						    size_t size, loff_t *ppos)
{
	struct kbase_device *kbdev = file->private_data;
	struct kbase_csf_firmware_log *fw_log = &kbdev->csf.fw_log;
	unsigned int n_read;
	unsigned long not_copied;
	/* Limit reads to the kernel dump buffer size */
	size_t mem = MIN(size, FIRMWARE_LOG_DUMP_BUF_SIZE);
	int ret;

	struct firmware_trace_buffer *tb =
		kbase_csf_firmware_get_trace_buffer(kbdev, FIRMWARE_LOG_BUF_NAME);

	if (tb == NULL) {
		dev_err(kbdev->dev, "Couldn't get the firmware trace buffer");
		return -EIO;
	}

	if (atomic_cmpxchg(&fw_log->busy, 0, 1) != 0)
		return -EBUSY;

	/* Reading from userspace is only allowed in manual mode */
	if (fw_log->mode != KBASE_CSF_FIRMWARE_LOG_MODE_MANUAL) {
		ret = -EINVAL;
		goto out;
	}

	n_read = kbase_csf_firmware_trace_buffer_read_data(tb, fw_log->dump_buf, mem);

	/* Do the copy, if we have obtained some trace data */
	not_copied = (n_read) ? copy_to_user(buf, fw_log->dump_buf, n_read) : 0;

	if (not_copied) {
		dev_err(kbdev->dev, "Couldn't copy trace buffer data to user space buffer");
		ret = -EFAULT;
		goto out;
	}

	*ppos += n_read;
	ret = n_read;

out:
	atomic_set(&fw_log->busy, 0);
	return ret;
}

static int kbase_csf_firmware_log_mode_read(void *data, u64 *val)
{
	struct kbase_device *kbdev = (struct kbase_device *)data;
	struct kbase_csf_firmware_log *fw_log = &kbdev->csf.fw_log;

	*val = fw_log->mode;
	return 0;
}

static int kbase_csf_firmware_log_mode_write(void *data, u64 val)
{
	struct kbase_device *kbdev = (struct kbase_device *)data;
	struct kbase_csf_firmware_log *fw_log = &kbdev->csf.fw_log;
	int ret = 0;

	if (atomic_cmpxchg(&fw_log->busy, 0, 1) != 0)
		return -EBUSY;

	if (val == fw_log->mode)
		goto out;

	switch (val) {
	case KBASE_CSF_FIRMWARE_LOG_MODE_MANUAL:
		cancel_delayed_work_sync(&fw_log->poll_work);
		break;
	case KBASE_CSF_FIRMWARE_LOG_MODE_AUTO_PRINT:
		schedule_delayed_work(&fw_log->poll_work,
				      msecs_to_jiffies(KBASE_CSF_FIRMWARE_LOG_POLL_PERIOD_MS));
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	fw_log->mode = val;

out:
	atomic_set(&fw_log->busy, 0);
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(kbase_csf_firmware_log_enable_mask_fops,
			 kbase_csf_firmware_log_enable_mask_read,
			 kbase_csf_firmware_log_enable_mask_write, "%llx\n");

static const struct file_operations kbasep_csf_firmware_log_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = kbasep_csf_firmware_log_debugfs_open,
	.read = kbasep_csf_firmware_log_debugfs_read,
	.llseek = no_llseek,
};

DEFINE_DEBUGFS_ATTRIBUTE(kbase_csf_firmware_log_mode_fops, kbase_csf_firmware_log_mode_read,
			 kbase_csf_firmware_log_mode_write, "%llu\n");

#endif /* CONFIG_DEBUG_FS */

static void kbase_csf_firmware_log_poll(struct work_struct *work)
{
	struct kbase_device *kbdev =
		container_of(work, struct kbase_device, csf.fw_log.poll_work.work);
	struct kbase_csf_firmware_log *fw_log = &kbdev->csf.fw_log;

	schedule_delayed_work(&fw_log->poll_work,
			      msecs_to_jiffies(KBASE_CSF_FIRMWARE_LOG_POLL_PERIOD_MS));

	kbase_csf_firmware_log_dump_buffer(kbdev);
}

int kbase_csf_firmware_log_init(struct kbase_device *kbdev)
{
	struct kbase_csf_firmware_log *fw_log = &kbdev->csf.fw_log;

	/* Add one byte for null-termination */
	fw_log->dump_buf = kmalloc(FIRMWARE_LOG_DUMP_BUF_SIZE + 1, GFP_KERNEL);
	if (fw_log->dump_buf == NULL)
		return -ENOMEM;

	/* Ensure null-termination for all strings */
	fw_log->dump_buf[FIRMWARE_LOG_DUMP_BUF_SIZE] = 0;

	fw_log->mode = KBASE_CSF_FIRMWARE_LOG_MODE_MANUAL;

	atomic_set(&fw_log->busy, 0);
	INIT_DEFERRABLE_WORK(&fw_log->poll_work, kbase_csf_firmware_log_poll);

#if defined(CONFIG_DEBUG_FS)
	debugfs_create_file("fw_trace_enable_mask", 0644, kbdev->mali_debugfs_directory, kbdev,
			    &kbase_csf_firmware_log_enable_mask_fops);
	debugfs_create_file("fw_traces", 0444, kbdev->mali_debugfs_directory, kbdev,
			    &kbasep_csf_firmware_log_debugfs_fops);
	debugfs_create_file("fw_trace_mode", 0644, kbdev->mali_debugfs_directory, kbdev,
			    &kbase_csf_firmware_log_mode_fops);
#endif /* CONFIG_DEBUG_FS */

	return 0;
}

void kbase_csf_firmware_log_term(struct kbase_device *kbdev)
{
	struct kbase_csf_firmware_log *fw_log = &kbdev->csf.fw_log;

	cancel_delayed_work_sync(&fw_log->poll_work);
	kfree(fw_log->dump_buf);
}

void kbase_csf_firmware_log_dump_buffer(struct kbase_device *kbdev)
{
	struct kbase_csf_firmware_log *fw_log = &kbdev->csf.fw_log;
	u8 *buf = fw_log->dump_buf, *p, *pnewline, *pend, *pendbuf;
	unsigned int read_size, remaining_size;
	struct firmware_trace_buffer *tb =
		kbase_csf_firmware_get_trace_buffer(kbdev, FIRMWARE_LOG_BUF_NAME);

	if (tb == NULL) {
		dev_dbg(kbdev->dev, "Can't get the trace buffer, firmware trace dump skipped");
		return;
	}

	if (atomic_cmpxchg(&fw_log->busy, 0, 1) != 0)
		return;

	/* FW should only print complete messages, so there's no need to handle
	 * partial messages over multiple invocations of this function
	 */

	p = buf;
	pendbuf = &buf[FIRMWARE_LOG_DUMP_BUF_SIZE];

	while ((read_size = kbase_csf_firmware_trace_buffer_read_data(tb, p, pendbuf - p))) {
		pend = p + read_size;
		p = buf;

		while (p < pend && (pnewline = memchr(p, '\n', pend - p))) {
			/* Null-terminate the string */
			*pnewline = 0;

			dev_err(kbdev->dev, "FW> %s", p);

			p = pnewline + 1;
		}

		remaining_size = pend - p;

		if (!remaining_size) {
			p = buf;
		} else if (remaining_size < FIRMWARE_LOG_DUMP_BUF_SIZE) {
			/* Copy unfinished string to the start of the buffer */
			memmove(buf, p, remaining_size);
			p = &buf[remaining_size];
		} else {
			/* Print abnormally long string without newlines */
			dev_err(kbdev->dev, "FW> %s", buf);
			p = buf;
		}
	}

	if (p != buf) {
		/* Null-terminate and print last unfinished string */
		*p = 0;
		dev_err(kbdev->dev, "FW> %s", buf);
	}

	atomic_set(&fw_log->busy, 0);
}
