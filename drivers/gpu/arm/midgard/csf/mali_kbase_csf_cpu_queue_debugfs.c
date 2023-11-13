// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2020-2023 ARM Limited. All rights reserved.
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

#include "mali_kbase_csf_cpu_queue_debugfs.h"

#if IS_ENABLED(CONFIG_DEBUG_FS)
#include "mali_kbase_csf_cpu_queue.h"
#include "mali_kbase_csf_util.h"
#include <mali_kbase.h>
#include <linux/seq_file.h>

/**
 * kbasep_csf_cpu_queue_debugfs_show() - Print cpu queue information for per context
 *
 * @file: The seq_file for printing to
 * @data: The debugfs dentry private data, a pointer to kbase_context
 *
 * Return: Negative error code or 0 on success.
 */
static int kbasep_csf_cpu_queue_debugfs_show(struct seq_file *file, void *data)
{
	struct kbasep_printer *kbpr;
	struct kbase_context *const kctx = file->private;
	int ret = -EINVAL;
	CSTD_UNUSED(data);

	kbpr = kbasep_printer_file_init(file);
	if (kbpr != NULL) {
		ret = kbasep_csf_cpu_queue_dump_print(kctx, kbpr);
		kbasep_printer_term(kbpr);
	}

	return ret;
}

static int kbasep_csf_cpu_queue_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, kbasep_csf_cpu_queue_debugfs_show, in->i_private);
}

static const struct file_operations kbasep_csf_cpu_queue_debugfs_fops = {
	.open = kbasep_csf_cpu_queue_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void kbase_csf_cpu_queue_debugfs_init(struct kbase_context *kctx)
{
	struct dentry *file;

	if (WARN_ON(!kctx || IS_ERR_OR_NULL(kctx->kctx_dentry)))
		return;

	file = debugfs_create_file("cpu_queue", 0444, kctx->kctx_dentry, kctx,
				   &kbasep_csf_cpu_queue_debugfs_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kctx->kbdev->dev, "Unable to create cpu queue debugfs entry");
	}
}

#else
/*
 * Stub functions for when debugfs is disabled
 */
void kbase_csf_cpu_queue_debugfs_init(struct kbase_context *kctx)
{
}

#endif /* CONFIG_DEBUG_FS */
