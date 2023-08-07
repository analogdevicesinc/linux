/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2022-2023 ARM Limited. All rights reserved.
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

#ifndef _KBASE_CSF_SYNC_DEBUGFS_H_
#define _KBASE_CSF_SYNC_DEBUGFS_H_

#include <linux/seq_file.h>

/* Forward declaration */
struct kbase_context;

#define MALI_CSF_SYNC_DEBUGFS_VERSION 0

/**
 * kbase_csf_sync_debugfs_init() - Create a debugfs entry for CSF queue sync info
 *
 * @kctx: The kbase_context for which to create the debugfs entry
 */
void kbase_csf_sync_debugfs_init(struct kbase_context *kctx);

/**
 * kbasep_csf_sync_kcpu_dump() - Print CSF KCPU queue sync info
 *
 * @kctx: The kbase context.
 * @file: The seq_file for printing to.
 *
 * Return: Negative error code or 0 on success.
 *
 * Note: This function should not be used if kcpu_queues.lock is held. Use
 * kbasep_csf_sync_kcpu_dump_locked() instead.
 */
int kbasep_csf_sync_kcpu_dump(struct kbase_context *kctx, struct seq_file *file);

/**
 * kbasep_csf_sync_kcpu_dump() - Print CSF KCPU queue sync info
 *
 * @kctx: The kbase context.
 * @file: The seq_file for printing to.
 *
 * Return: Negative error code or 0 on success.
 */
int kbasep_csf_sync_kcpu_dump_locked(struct kbase_context *kctx, struct seq_file *file);

#endif /* _KBASE_CSF_SYNC_DEBUGFS_H_ */
