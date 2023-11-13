/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2023 ARM Limited. All rights reserved.
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

#ifndef _KBASE_CSF_SYNC_H_
#define _KBASE_CSF_SYNC_H_

/* Forward declaration */
struct kbase_context;
struct kbasep_printer;

#define MALI_CSF_SYNC_DUMP_VERSION 0

/**
 * kbasep_csf_sync_kcpu_dump_print() - Print CSF KCPU queue sync info
 *
 * @kctx: The kbase context.
 * @kbpr: Pointer to printer instance.
 *
 * Return: Negative error code or 0 on success.
 */
int kbasep_csf_sync_kcpu_dump_print(struct kbase_context *kctx, struct kbasep_printer *kbpr);

/**
 * kbasep_csf_sync_gpu_dump_print() - Print CSF GPU queue sync info
 *
 * @kctx: The kbase context
 * @kbpr: Pointer to printer instance.
 *
 * Return: Negative error code or 0 on success.
 */
int kbasep_csf_sync_gpu_dump_print(struct kbase_context *kctx, struct kbasep_printer *kbpr);

#endif /* _KBASE_CSF_SYNC_H_ */
