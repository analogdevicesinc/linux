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

#ifndef _KBASE_CSF_CPU_QUEUE_H_
#define _KBASE_CSF_CPU_QUEUE_H_

#include <linux/types.h>

/* Forward declaration */
struct base_csf_notification;
struct kbase_context;
struct kbasep_printer;

#define MALI_CSF_CPU_QUEUE_DUMP_VERSION 0

/* CPU queue dump status */
/* Dumping is done or no dumping is in progress. */
#define BASE_CSF_CPU_QUEUE_DUMP_COMPLETE 0
/* Dumping request is pending. */
#define BASE_CSF_CPU_QUEUE_DUMP_PENDING 1
/* Dumping request is issued to Userspace */
#define BASE_CSF_CPU_QUEUE_DUMP_ISSUED 2

/**
 * kbase_csf_cpu_queue_init() - Initialise cpu queue handling per context cpu queue(s)
 *
 * @kctx: The kbase_context
 */
void kbase_csf_cpu_queue_init(struct kbase_context *kctx);

/**
 * kbase_csf_cpu_queue_read_dump_req() - Read cpu queue dump request event
 *
 * @kctx: The kbase_context which cpu queue dumped belongs to.
 * @req:  Notification with cpu queue dump request.
 *
 * Return: true if needs CPU queue dump, or false otherwise.
 */
bool kbase_csf_cpu_queue_read_dump_req(struct kbase_context *kctx,
				       struct base_csf_notification *req);

/**
 * kbase_csf_cpu_queue_dump_needed() - Check the requirement for cpu queue dump
 *
 * @kctx: The kbase_context which cpu queue dumped belongs to.
 *
 * Return: true if it needs cpu queue dump, or false otherwise.
 */
bool kbase_csf_cpu_queue_dump_needed(struct kbase_context *kctx);

/**
 * kbase_csf_cpu_queue_dump_buffer() - dump buffer containing cpu queue information
 *
 * @kctx: The kbase_context which cpu queue dumped belongs to.
 * @buffer: Buffer containing the cpu queue information.
 * @buf_size: Buffer size.
 *
 * Return: Return 0 for dump successfully, or error code.
 */
int kbase_csf_cpu_queue_dump_buffer(struct kbase_context *kctx, u64 buffer, size_t buf_size);

/**
 * kbasep_csf_cpu_queue_dump_print() - Dump cpu queue information to file
 *
 * @kctx: The kbase_context which cpu queue dumped belongs to.
 * @kbpr: Pointer to printer instance.
 *
 * Return: Return 0 for dump successfully, or error code.
 */
int kbasep_csf_cpu_queue_dump_print(struct kbase_context *kctx, struct kbasep_printer *kbpr);

#endif /* _KBASE_CSF_CPU_QUEUE_H_ */
