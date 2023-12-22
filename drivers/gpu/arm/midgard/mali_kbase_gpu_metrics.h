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

/**
 * DOC: GPU metrics frontend APIs
 */

#ifndef _KBASE_GPU_METRICS_H_
#define _KBASE_GPU_METRICS_H_

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)

#include <linux/types.h>

struct kbase_device;
struct kbase_context;
struct kbase_gpu_metrics_ctx;

/**
 * kbase_gpu_metrics_get_tp_emit_interval() - Return the trace point emission interval.
 *
 * Return: The time interval in nanosecond for GPU metrics trace point emission.
 */
unsigned long kbase_gpu_metrics_get_tp_emit_interval(void);

/**
 * kbase_gpu_metrics_ctx_put() - Decrement the Kbase context count for the GPU metrics
 *                               context and free it if the count becomes 0.
 *
 * @kbdev:           Pointer to the GPU device.
 * @gpu_metrics_ctx: Pointer to the GPU metrics context.
 *
 * This function must be called when a Kbase context is destroyed.
 * The function would decrement the Kbase context count for the GPU metrics context and
 * free the memory if the count becomes 0.
 * The function would emit a power/gpu_work_period tracepoint for the GPU metrics context
 * if there was some GPU activity done for it since the last tracepoint was emitted.
 *
 * Note: The caller must appropriately serialize the call to this function with the
 *       call to other GPU metrics functions declared in this file.
 */
void kbase_gpu_metrics_ctx_put(struct kbase_device *kbdev,
			       struct kbase_gpu_metrics_ctx *gpu_metrics_ctx);

/**
 * kbase_gpu_metrics_ctx_get() - Increment the Kbase context count for the GPU metrics
 *                               context if it exists.
 *
 * @kbdev: Pointer to the GPU device.
 * @aid:   Unique identifier of the Application that is creating the Kbase context.
 *
 * This function must be called when a Kbase context is created.
 * The function would increment the Kbase context count for the GPU metrics context,
 * corresponding to the @aid, if it exists.
 *
 * Return: Pointer to the GPU metrics context corresponding to the @aid if it already
 * exists otherwise NULL.
 *
 * Note: The caller must appropriately serialize the call to this function with the
 *       call to other GPU metrics functions declared in this file.
 *       The caller shall allocate memory for GPU metrics context structure if the
 *       function returns NULL.
 */
struct kbase_gpu_metrics_ctx *kbase_gpu_metrics_ctx_get(struct kbase_device *kbdev, u32 aid);

/**
 * kbase_gpu_metrics_ctx_init() - Initialise the GPU metrics context
 *
 * @kbdev:           Pointer to the GPU device.
 * @gpu_metrics_ctx: Pointer to the GPU metrics context.
 * @aid:             Unique identifier of the Application for which GPU metrics
 *                   context needs to be initialized.
 *
 * This function must be called when a Kbase context is created, after the call to
 * kbase_gpu_metrics_ctx_get() returned NULL and memory for the GPU metrics context
 * structure was allocated.
 *
 * Note: The caller must appropriately serialize the call to this function with the
 *       call to other GPU metrics functions declared in this file.
 */
void kbase_gpu_metrics_ctx_init(struct kbase_device *kbdev,
				struct kbase_gpu_metrics_ctx *gpu_metrics_ctx, u32 aid);

/**
 * kbase_gpu_metrics_ctx_start_activity() - Report the start of some GPU activity
 *                                          for GPU metrics context.
 *
 * @kctx:         Pointer to the Kbase context contributing data to the GPU metrics context.
 * @timestamp_ns: CPU timestamp at which the GPU activity started.
 *
 * The provided timestamp would be later used as the "start_time_ns" for the
 * power/gpu_work_period tracepoint if this is the first GPU activity for the GPU
 * metrics context in the current work period.
 *
 * Note: The caller must appropriately serialize the call to this function with the
 *       call to other GPU metrics functions declared in this file.
 */
void kbase_gpu_metrics_ctx_start_activity(struct kbase_context *kctx, u64 timestamp_ns);

/**
 * kbase_gpu_metrics_ctx_end_activity() - Report the end of some GPU activity
 *                                        for GPU metrics context.
 *
 * @kctx:         Pointer to the Kbase context contributing data to the GPU metrics context.
 * @timestamp_ns: CPU timestamp at which the GPU activity ended.
 *
 * The provided timestamp would be later used as the "end_time_ns" for the
 * power/gpu_work_period tracepoint if this is the last GPU activity for the GPU
 * metrics context in the current work period.
 *
 * Note: The caller must appropriately serialize the call to this function with the
 *       call to other GPU metrics functions declared in this file.
 */
void kbase_gpu_metrics_ctx_end_activity(struct kbase_context *kctx, u64 timestamp_ns);

/**
 * kbase_gpu_metrics_emit_tracepoint() - Emit power/gpu_work_period tracepoint
 *                                       for active GPU metrics contexts.
 *
 * @kbdev: Pointer to the GPU device.
 * @ts:    Timestamp at which the tracepoint is being emitted.
 *
 * This function would loop through all the active GPU metrics contexts and emit a
 * power/gpu_work_period tracepoint for them.
 * The GPU metrics context that is found to be inactive since the last tracepoint
 * was emitted would be moved to the inactive list.
 * The current work period would be considered as over and a new work period would
 * begin whenever any application does the GPU activity.
 *
 * Note: The caller must appropriately serialize the call to this function with the
 *       call to other GPU metrics functions declared in this file.
 */
void kbase_gpu_metrics_emit_tracepoint(struct kbase_device *kbdev, u64 ts);

/**
 * kbase_gpu_metrics_init() - Initialise a gpu_metrics instance for a GPU
 *
 * @kbdev: Pointer to the GPU device.
 *
 * This function is called once for each @kbdev.
 *
 * Return: 0 on success, or negative on failure.
 */
int kbase_gpu_metrics_init(struct kbase_device *kbdev);

/**
 * kbase_gpu_metrics_term() - Terminate a gpu_metrics instance
 *
 * @kbdev: Pointer to the GPU device.
 */
void kbase_gpu_metrics_term(struct kbase_device *kbdev);

#endif
#endif /* _KBASE_GPU_METRICS_H_ */
