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

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)

#include "mali_power_gpu_work_period_trace.h"
#include <mali_kbase_gpu_metrics.h>
#include <mali_kbase_config_defaults.h>
#include <mali_kbase.h>

#include <linux/module.h>
#include <linux/slab.h>

/**
 * enum gpu_metrics_ctx_flags - Flags for the GPU metrics context
 *
 * @ACTIVE_INTERVAL_IN_WP: Flag set when the application first becomes active in
 *                         the current work period.
 *
 * @INSIDE_ACTIVE_LIST:    Flag to track if object is in kbase_device::gpu_metrics::active_list
 *
 * All members need to be separate bits. This enum is intended for use in a
 * bitmask where multiple values get OR-ed together.
 */
enum gpu_metrics_ctx_flags {
	ACTIVE_INTERVAL_IN_WP = 1 << 0,
	INSIDE_ACTIVE_LIST = 1 << 1,
};

static unsigned long gpu_metrics_tp_emit_interval_ns = DEFAULT_GPU_METRICS_TP_EMIT_INTERVAL_NS;

module_param(gpu_metrics_tp_emit_interval_ns, ulong, 0444);
MODULE_PARM_DESC(gpu_metrics_tp_emit_interval_ns,
		 "Time interval in nano seconds at which GPU metrics tracepoints are emitted");

static inline bool gpu_metrics_ctx_flag(struct kbase_gpu_metrics_ctx *gpu_metrics_ctx,
					enum gpu_metrics_ctx_flags flag)
{
	return (gpu_metrics_ctx->flags & flag);
}

static inline void gpu_metrics_ctx_flag_set(struct kbase_gpu_metrics_ctx *gpu_metrics_ctx,
					    enum gpu_metrics_ctx_flags flag)
{
	gpu_metrics_ctx->flags |= flag;
}

static inline void gpu_metrics_ctx_flag_clear(struct kbase_gpu_metrics_ctx *gpu_metrics_ctx,
					      enum gpu_metrics_ctx_flags flag)
{
	gpu_metrics_ctx->flags &= ~flag;
}

static inline void validate_tracepoint_data(struct kbase_gpu_metrics_ctx *gpu_metrics_ctx,
					    u64 start_time, u64 end_time, u64 total_active)
{
#ifdef CONFIG_MALI_DEBUG
	WARN(total_active > NSEC_PER_SEC, "total_active %llu > 1 second for aid %u active_cnt %u",
	     total_active, gpu_metrics_ctx->aid, gpu_metrics_ctx->active_cnt);

	WARN(start_time >= end_time, "start_time %llu >= end_time %llu for aid %u active_cnt %u",
	     start_time, end_time, gpu_metrics_ctx->aid, gpu_metrics_ctx->active_cnt);

	WARN(total_active > (end_time - start_time),
	     "total_active %llu > end_time %llu - start_time %llu for aid %u active_cnt %u",
	     total_active, end_time, start_time, gpu_metrics_ctx->aid, gpu_metrics_ctx->active_cnt);

	WARN(gpu_metrics_ctx->prev_wp_active_end_time > start_time,
	     "prev_wp_active_end_time %llu > start_time %llu for aid %u active_cnt %u",
	     gpu_metrics_ctx->prev_wp_active_end_time, start_time, gpu_metrics_ctx->aid,
	     gpu_metrics_ctx->active_cnt);
#endif
}

static void emit_tracepoint_for_active_gpu_metrics_ctx(
	struct kbase_device *kbdev, struct kbase_gpu_metrics_ctx *gpu_metrics_ctx, u64 current_time)
{
	const u64 start_time = gpu_metrics_ctx->first_active_start_time;
	u64 total_active = gpu_metrics_ctx->total_active;
	u64 end_time;

	/* Check if the GPU activity is currently ongoing */
	if (gpu_metrics_ctx->active_cnt) {
		/* The following check is to handle the race on CSF GPUs that can happen between
		 * the draining of trace buffer and FW emitting the ACT=1 event .
		 */
		if (unlikely(current_time == gpu_metrics_ctx->last_active_start_time))
			current_time++;
		end_time = current_time;
		total_active += end_time - gpu_metrics_ctx->last_active_start_time;

		gpu_metrics_ctx->first_active_start_time = current_time;
		gpu_metrics_ctx->last_active_start_time = current_time;
	} else {
		end_time = gpu_metrics_ctx->last_active_end_time;
		gpu_metrics_ctx_flag_clear(gpu_metrics_ctx, ACTIVE_INTERVAL_IN_WP);
	}

	trace_gpu_work_period(kbdev->id, gpu_metrics_ctx->aid, start_time, end_time, total_active);

	validate_tracepoint_data(gpu_metrics_ctx, start_time, end_time, total_active);
	gpu_metrics_ctx->prev_wp_active_end_time = end_time;
	gpu_metrics_ctx->total_active = 0;
}

void kbase_gpu_metrics_ctx_put(struct kbase_device *kbdev,
			       struct kbase_gpu_metrics_ctx *gpu_metrics_ctx)
{
	WARN_ON(list_empty(&gpu_metrics_ctx->link));
	WARN_ON(!gpu_metrics_ctx->kctx_count);

	gpu_metrics_ctx->kctx_count--;
	if (gpu_metrics_ctx->kctx_count)
		return;

	if (gpu_metrics_ctx_flag(gpu_metrics_ctx, ACTIVE_INTERVAL_IN_WP))
		emit_tracepoint_for_active_gpu_metrics_ctx(kbdev, gpu_metrics_ctx,
							   ktime_get_raw_ns());

	list_del_init(&gpu_metrics_ctx->link);
	kfree(gpu_metrics_ctx);
}

struct kbase_gpu_metrics_ctx *kbase_gpu_metrics_ctx_get(struct kbase_device *kbdev, u32 aid)
{
	struct kbase_gpu_metrics *gpu_metrics = &kbdev->gpu_metrics;
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx;

	list_for_each_entry(gpu_metrics_ctx, &gpu_metrics->active_list, link) {
		if (gpu_metrics_ctx->aid == aid) {
			WARN_ON(!gpu_metrics_ctx->kctx_count);
			gpu_metrics_ctx->kctx_count++;
			return gpu_metrics_ctx;
		}
	}

	list_for_each_entry(gpu_metrics_ctx, &gpu_metrics->inactive_list, link) {
		if (gpu_metrics_ctx->aid == aid) {
			WARN_ON(!gpu_metrics_ctx->kctx_count);
			gpu_metrics_ctx->kctx_count++;
			return gpu_metrics_ctx;
		}
	}

	return NULL;
}

void kbase_gpu_metrics_ctx_init(struct kbase_device *kbdev,
				struct kbase_gpu_metrics_ctx *gpu_metrics_ctx, unsigned int aid)
{
	gpu_metrics_ctx->aid = aid;
	gpu_metrics_ctx->total_active = 0;
	gpu_metrics_ctx->kctx_count = 1;
	gpu_metrics_ctx->active_cnt = 0;
	gpu_metrics_ctx->prev_wp_active_end_time = 0;
	gpu_metrics_ctx->flags = 0;
	list_add_tail(&gpu_metrics_ctx->link, &kbdev->gpu_metrics.inactive_list);
}

void kbase_gpu_metrics_ctx_start_activity(struct kbase_context *kctx, u64 timestamp_ns)
{
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx = kctx->gpu_metrics_ctx;

	gpu_metrics_ctx->active_cnt++;
	if (gpu_metrics_ctx->active_cnt == 1)
		gpu_metrics_ctx->last_active_start_time = timestamp_ns;

	if (!gpu_metrics_ctx_flag(gpu_metrics_ctx, ACTIVE_INTERVAL_IN_WP)) {
		gpu_metrics_ctx->first_active_start_time = timestamp_ns;
		gpu_metrics_ctx_flag_set(gpu_metrics_ctx, ACTIVE_INTERVAL_IN_WP);
	}

	if (!gpu_metrics_ctx_flag(gpu_metrics_ctx, INSIDE_ACTIVE_LIST)) {
		list_move_tail(&gpu_metrics_ctx->link, &kctx->kbdev->gpu_metrics.active_list);
		gpu_metrics_ctx_flag_set(gpu_metrics_ctx, INSIDE_ACTIVE_LIST);
	}
}

void kbase_gpu_metrics_ctx_end_activity(struct kbase_context *kctx, u64 timestamp_ns)
{
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx = kctx->gpu_metrics_ctx;

	if (WARN_ON_ONCE(!gpu_metrics_ctx->active_cnt))
		return;

	if (--gpu_metrics_ctx->active_cnt)
		return;

	if (likely(timestamp_ns > gpu_metrics_ctx->last_active_start_time)) {
		gpu_metrics_ctx->last_active_end_time = timestamp_ns;
		gpu_metrics_ctx->total_active +=
			timestamp_ns - gpu_metrics_ctx->last_active_start_time;
		return;
	}

	/* Due to conversion from system timestamp to CPU timestamp (which involves rounding)
	 * the value for start and end timestamp could come as same on CSF GPUs.
	 */
	if (timestamp_ns == gpu_metrics_ctx->last_active_start_time) {
		gpu_metrics_ctx->last_active_end_time = timestamp_ns + 1;
		gpu_metrics_ctx->total_active += 1;
		return;
	}

	/* The following check is to detect the situation on CSF GPUs, where 'ACT=0' event was not
	 * visible to the Kbase even though the system timestamp value sampled by FW was less than
	 * the system timestamp value sampled by Kbase just before the draining of trace buffer.
	 */
	if (gpu_metrics_ctx->last_active_start_time == gpu_metrics_ctx->first_active_start_time &&
	    gpu_metrics_ctx->prev_wp_active_end_time == gpu_metrics_ctx->first_active_start_time) {
		WARN_ON_ONCE(gpu_metrics_ctx->total_active);
		gpu_metrics_ctx->last_active_end_time =
			gpu_metrics_ctx->prev_wp_active_end_time + 1;
		gpu_metrics_ctx->total_active = 1;
		return;
	}

	WARN_ON_ONCE(1);
}

void kbase_gpu_metrics_emit_tracepoint(struct kbase_device *kbdev, u64 ts)
{
	struct kbase_gpu_metrics *gpu_metrics = &kbdev->gpu_metrics;
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx, *tmp;

	list_for_each_entry_safe(gpu_metrics_ctx, tmp, &gpu_metrics->active_list, link) {
		if (!gpu_metrics_ctx_flag(gpu_metrics_ctx, ACTIVE_INTERVAL_IN_WP)) {
			WARN_ON(!gpu_metrics_ctx_flag(gpu_metrics_ctx, INSIDE_ACTIVE_LIST));
			WARN_ON(gpu_metrics_ctx->active_cnt);
			list_move_tail(&gpu_metrics_ctx->link, &gpu_metrics->inactive_list);
			gpu_metrics_ctx_flag_clear(gpu_metrics_ctx, INSIDE_ACTIVE_LIST);
			continue;
		}

		emit_tracepoint_for_active_gpu_metrics_ctx(kbdev, gpu_metrics_ctx, ts);
	}
}

int kbase_gpu_metrics_init(struct kbase_device *kbdev)
{
	INIT_LIST_HEAD(&kbdev->gpu_metrics.active_list);
	INIT_LIST_HEAD(&kbdev->gpu_metrics.inactive_list);

	if (!gpu_metrics_tp_emit_interval_ns || (gpu_metrics_tp_emit_interval_ns >= NSEC_PER_SEC)) {
		dev_warn(
			kbdev->dev,
			"Invalid value (%lu ns) for module param gpu_metrics_tp_emit_interval_ns. Using default value: %u ns",
			gpu_metrics_tp_emit_interval_ns, DEFAULT_GPU_METRICS_TP_EMIT_INTERVAL_NS);
		gpu_metrics_tp_emit_interval_ns = DEFAULT_GPU_METRICS_TP_EMIT_INTERVAL_NS;
	}

	dev_info(kbdev->dev, "GPU metrics tracepoint support enabled");
	return 0;
}

void kbase_gpu_metrics_term(struct kbase_device *kbdev)
{
	WARN_ON_ONCE(!list_empty(&kbdev->gpu_metrics.active_list));
	WARN_ON_ONCE(!list_empty(&kbdev->gpu_metrics.inactive_list));
}

unsigned long kbase_gpu_metrics_get_tp_emit_interval(void)
{
	return gpu_metrics_tp_emit_interval_ns;
}

#endif
