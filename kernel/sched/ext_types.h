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

/*
 * Per-cid topology info. For each topology level (core, LLC, node), records
 * the first cid in the unit and its global index. Global indices are
 * consecutive integers assigned in cid-walk order, so e.g. core_idx ranges
 * over [0, nr_cores_at_init) with no gaps. No-topo cids have all fields set
 * to -1.
 *
 * @core_cid: first cid of this cid's core (smt-sibling group)
 * @core_idx: global index of that core, in [0, nr_cores_at_init)
 * @llc_cid: first cid of this cid's LLC
 * @llc_idx: global index of that LLC, in [0, nr_llcs_at_init)
 * @node_cid: first cid of this cid's NUMA node
 * @node_idx: global index of that node, in [0, nr_nodes_at_init)
 */
struct scx_cid_topo {
	s32 core_cid;
	s32 core_idx;
	s32 llc_cid;
	s32 llc_idx;
	s32 node_cid;
	s32 node_idx;
};

/*
 * cmask: variable-length, base-windowed bitmap over cid space
 * -----------------------------------------------------------
 *
 * A cmask covers the cid range [base, base + nr_bits). bits[] is aligned to the
 * global 64-cid grid: bits[0] spans [base & ~63, (base & ~63) + 64), so the
 * first (base & 63) bits of bits[0] are head padding and any tail past base +
 * nr_bits is tail padding. Both must stay zero for the lifetime of the mask;
 * all mutating helpers preserve that invariant.
 *
 * Grid alignment means two cmasks always address bits[] against the same global
 * 64-cid windows, so cross-cmask word ops (AND, OR, ...) reduce to
 *
 *	dst->bits[i] OP= src->bits[i - delta]
 *
 * with no bit-shifting, regardless of how the two bases relate mod 64.
 */
struct scx_cmask {
	u32 base;
	u32 nr_bits;
	DECLARE_FLEX_ARRAY(u64, bits);
};

/*
 * Number of u64 words of bits[] storage that covers @nr_bits regardless of base
 * alignment. The +1 absorbs up to 63 bits of head padding when base is not
 * 64-aligned - always allocating one extra word beats branching on base or
 * splitting the compute.
 */
#define SCX_CMASK_NR_WORDS(nr_bits)	(((nr_bits) + 63) / 64 + 1)

/*
 * Define an on-stack cmask for up to @cap_bits. @name is a struct scx_cmask *
 * aliasing zero-initialized storage; call scx_cmask_init() to set base/nr_bits.
 */
#define SCX_CMASK_DEFINE(name, cap_bits)	\
	DEFINE_RAW_FLEX(struct scx_cmask, name, bits, SCX_CMASK_NR_WORDS(cap_bits))

#endif /* _KERNEL_SCHED_EXT_TYPES_H */
