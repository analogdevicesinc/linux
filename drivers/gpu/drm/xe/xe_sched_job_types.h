/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef _XE_SCHED_JOB_TYPES_H_
#define _XE_SCHED_JOB_TYPES_H_

#include <linux/kref.h>

#include <drm/gpu_scheduler.h>

struct xe_exec_queue;
struct dma_fence;
struct dma_fence_chain;

/**
 * struct xe_job_ptrs - Per hw engine instance data
 */
struct xe_job_ptrs {
	/** @lrc_fence: Pre-allocated uinitialized lrc fence.*/
	struct dma_fence *lrc_fence;
	/** @chain_fence: Pre-allocated ninitialized fence chain node. */
	struct dma_fence_chain *chain_fence;
	/** @batch_addr: Batch buffer address. */
	u64 batch_addr;
};

/**
 * struct xe_sched_job - XE schedule job (batch buffer tracking)
 */
struct xe_sched_job {
	/** @drm: base DRM scheduler job */
	struct drm_sched_job drm;
	/** @q: Exec queue */
	struct xe_exec_queue *q;
	/** @refcount: ref count of this job */
	struct kref refcount;
	/**
	 * @fence: dma fence to indicate completion. 1 way relationship - job
	 * can safely reference fence, fence cannot safely reference job.
	 */
#define JOB_FLAG_SUBMIT		DMA_FENCE_FLAG_USER_BITS
	struct dma_fence *fence;
	/** @user_fence: write back value when BB is complete */
	struct {
		/** @user_fence.used: user fence is used */
		bool used;
		/** @user_fence.addr: address to write to */
		u64 addr;
		/** @user_fence.value: write back value */
		u64 value;
	} user_fence;
	/** @lrc_seqno: LRC seqno */
	u32 lrc_seqno;
	/** @migrate_flush_flags: Additional flush flags for migration jobs */
	u32 migrate_flush_flags;
	/** @ring_ops_flush_tlb: The ring ops need to flush TLB before payload. */
	bool ring_ops_flush_tlb;
	/** @ptrs: per instance pointers. */
	struct xe_job_ptrs ptrs[];
};

struct xe_sched_job_snapshot {
	u16 batch_addr_len;
	u64 batch_addr[];
};

#endif
