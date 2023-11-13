// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
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

#include "mali_kbase_csf_cpu_queue.h"
#include "mali_kbase_csf_util.h"
#include <mali_kbase.h>
#include <asm/atomic.h>

void kbase_csf_cpu_queue_init(struct kbase_context *kctx)
{
	if (WARN_ON(!kctx))
		return;

	kctx->csf.cpu_queue.buffer = NULL;
	kctx->csf.cpu_queue.buffer_size = 0;
	atomic_set(&kctx->csf.cpu_queue.dump_req_status, BASE_CSF_CPU_QUEUE_DUMP_COMPLETE);
}

bool kbase_csf_cpu_queue_read_dump_req(struct kbase_context *kctx,
				       struct base_csf_notification *req)
{
	if (atomic_cmpxchg(&kctx->csf.cpu_queue.dump_req_status, BASE_CSF_CPU_QUEUE_DUMP_ISSUED,
			   BASE_CSF_CPU_QUEUE_DUMP_PENDING) != BASE_CSF_CPU_QUEUE_DUMP_ISSUED) {
		return false;
	}

	req->type = BASE_CSF_NOTIFICATION_CPU_QUEUE_DUMP;
	return true;
}

bool kbase_csf_cpu_queue_dump_needed(struct kbase_context *kctx)
{
	return (atomic_read(&kctx->csf.cpu_queue.dump_req_status) ==
		BASE_CSF_CPU_QUEUE_DUMP_ISSUED);
}

int kbase_csf_cpu_queue_dump_buffer(struct kbase_context *kctx, u64 buffer, size_t buf_size)
{
	size_t alloc_size = buf_size;
	char *dump_buffer;

	if (!buffer || !buf_size)
		return 0;

	if (alloc_size > KBASE_MEM_ALLOC_MAX_SIZE)
		return -EINVAL;

	alloc_size = (alloc_size + PAGE_SIZE) & ~(PAGE_SIZE - 1);
	dump_buffer = kzalloc(alloc_size, GFP_KERNEL);
	if (!dump_buffer)
		return -ENOMEM;

	WARN_ON(kctx->csf.cpu_queue.buffer != NULL);

	if (copy_from_user(dump_buffer, u64_to_user_ptr(buffer), buf_size)) {
		kfree(dump_buffer);
		return -EFAULT;
	}

	mutex_lock(&kctx->csf.lock);

	kfree(kctx->csf.cpu_queue.buffer);

	if (atomic_read(&kctx->csf.cpu_queue.dump_req_status) == BASE_CSF_CPU_QUEUE_DUMP_PENDING) {
		kctx->csf.cpu_queue.buffer = dump_buffer;
		kctx->csf.cpu_queue.buffer_size = buf_size;
		complete_all(&kctx->csf.cpu_queue.dump_cmp);
	} else
		kfree(dump_buffer);

	mutex_unlock(&kctx->csf.lock);

	return 0;
}

int kbasep_csf_cpu_queue_dump_print(struct kbase_context *kctx, struct kbasep_printer *kbpr)
{
	mutex_lock(&kctx->csf.lock);
	if (atomic_read(&kctx->csf.cpu_queue.dump_req_status) != BASE_CSF_CPU_QUEUE_DUMP_COMPLETE) {
		kbasep_print(kbpr, "Dump request already started! (try again)\n");
		mutex_unlock(&kctx->csf.lock);
		return -EBUSY;
	}

	atomic_set(&kctx->csf.cpu_queue.dump_req_status, BASE_CSF_CPU_QUEUE_DUMP_ISSUED);
	init_completion(&kctx->csf.cpu_queue.dump_cmp);
	kbase_event_wakeup(kctx);
	mutex_unlock(&kctx->csf.lock);

	kbasep_print(kbpr, "CPU Queues table (version:v" __stringify(
				   MALI_CSF_CPU_QUEUE_DUMP_VERSION) "):\n");

	wait_for_completion_timeout(&kctx->csf.cpu_queue.dump_cmp, msecs_to_jiffies(3000));

	mutex_lock(&kctx->csf.lock);
	if (kctx->csf.cpu_queue.buffer) {
		WARN_ON(atomic_read(&kctx->csf.cpu_queue.dump_req_status) !=
			BASE_CSF_CPU_QUEUE_DUMP_PENDING);

		/* The CPU queue dump is returned as a single formatted string */
		kbasep_puts(kbpr, kctx->csf.cpu_queue.buffer);
		kbasep_puts(kbpr, "\n");

		kfree(kctx->csf.cpu_queue.buffer);
		kctx->csf.cpu_queue.buffer = NULL;
		kctx->csf.cpu_queue.buffer_size = 0;
	} else
		kbasep_print(kbpr, "Dump error! (time out)\n");

	atomic_set(&kctx->csf.cpu_queue.dump_req_status, BASE_CSF_CPU_QUEUE_DUMP_COMPLETE);

	mutex_unlock(&kctx->csf.lock);
	return 0;
}
