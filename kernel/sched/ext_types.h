/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Early sched_ext type definitions.
 *
 * Copyright (c) 2026 Meta Platforms, Inc. and affiliates.
 * Copyright (c) 2026 Tejun Heo <tj@kernel.org>
 */
#ifndef _KERNEL_SCHED_EXT_TYPES_H
#define _KERNEL_SCHED_EXT_TYPES_H

enum scx_consts {
	SCX_DSP_DFL_MAX_BATCH		= 32,
	SCX_DSP_MAX_LOOPS		= 32,
	SCX_WATCHDOG_MAX_TIMEOUT	= 30 * HZ,

	/* per-CPU chunk size for p->scx.tid allocation, see scx_alloc_tid() */
	SCX_TID_CHUNK			= 1024,

	SCX_EXIT_BT_LEN			= 64,
	SCX_EXIT_MSG_LEN		= 1024,
	SCX_EXIT_DUMP_DFL_LEN		= 32768,

	SCX_CPUPERF_ONE			= SCHED_CAPACITY_SCALE,

	/*
	 * Iterating all tasks may take a while. Periodically drop
	 * scx_tasks_lock to avoid causing e.g. CSD and RCU stalls.
	 */
	SCX_TASK_ITER_BATCH		= 32,

	SCX_BYPASS_HOST_NTH		= 2,

	SCX_BYPASS_LB_DFL_INTV_US	= 500 * USEC_PER_MSEC,
	SCX_BYPASS_LB_DONOR_PCT		= 125,
	SCX_BYPASS_LB_MIN_DELTA_DIV	= 4,
	SCX_BYPASS_LB_BATCH		= 256,

	SCX_REENQ_LOCAL_MAX_REPEAT	= 256,

	SCX_SUB_MAX_DEPTH		= 4,
};

#endif /* _KERNEL_SCHED_EXT_TYPES_H */
