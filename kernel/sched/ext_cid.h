/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Topological CPU IDs (cids)
 * --------------------------
 *
 * Raw cpu numbers are clumsy for sharding work and communication across
 * topology units, especially from BPF: the space can be sparse, numerical
 * closeness doesn't imply topological closeness (x86 hyperthreading often puts
 * SMT siblings far apart), and a range of cpu ids doesn't mean anything.
 * Sub-scheds make this acute - cpu allocation, revocation and other state are
 * constantly communicated across sub-scheds, and passing whole cpumasks scales
 * poorly with cpu count. cpumasks are also awkward in BPF: a variable-length
 * kernel type sized for the maximum NR_CPUS (4k), with verbose helper sequences
 * for every op.
 *
 * cids give every cpu a dense, topology-ordered id. CPUs sharing a core, LLC or
 * NUMA node get contiguous cid ranges, so a topology unit becomes a (start,
 * length) slice of cid space. Communication can pass a slice instead of a
 * cpumask, and BPF code can process, for example, a u64 word's worth of cids at
 * a time.
 *
 * The mapping is built once at root scheduler enable time by walking the
 * topology of online cpus only. Going by online cpus is out of necessity:
 * depending on the arch, topology info isn't reliably available for offline
 * cpus. The expected usage model is restarting the scheduler on hotplug events
 * so the mapping is rebuilt against the new online set. A scheduler that wants
 * to handle hotplug without a restart can provide its own cid and shard mapping
 * through the override interface.
 *
 * Copyright (c) 2026 Meta Platforms, Inc. and affiliates.
 * Copyright (c) 2026 Tejun Heo <tj@kernel.org>
 */
#ifndef _KERNEL_SCHED_EXT_CID_H
#define _KERNEL_SCHED_EXT_CID_H

struct scx_sched;

/*
 * Cid space (total is always num_possible_cpus()) is laid out with
 * topology-annotated cids first, then no-topo cids at the tail. The
 * topology-annotated block covers the cpus that were online when scx_cid_init()
 * ran and remains valid even after those cpus go offline. The tail block covers
 * possible-but-not-online cpus and carries all-(-1) topo info (see
 * scx_cid_topo); callers detect it via the -1 sentinels.
 *
 * See the comment above the table definitions in ext_cid.c for the
 * memory-ordering and visibility contract.
 */
extern s16 *scx_cid_to_cpu_tbl;
extern s16 *scx_cpu_to_cid_tbl;
extern struct scx_cid_topo *scx_cid_topo;

s32 scx_cid_init(struct scx_sched *sch);
int scx_cid_kfunc_init(void);

/**
 * cid_valid - Verify a cid value, to be used on ops input args
 * @sch: scx_sched to abort on error
 * @cid: cid which came from a BPF ops
 *
 * Return true if @cid is in [0, num_possible_cpus()). On failure, trigger
 * scx_error() and return false.
 */
static inline bool cid_valid(struct scx_sched *sch, s32 cid)
{
	if (likely(cid >= 0 && cid < num_possible_cpus()))
		return true;
	scx_error(sch, "invalid cid %d", cid);
	return false;
}

/**
 * __scx_cid_to_cpu - Unchecked cid->cpu table lookup
 * @cid: cid to look up. Must be in [0, num_possible_cpus()).
 *
 * Intended for callsites that have already validated @cid and that hold a
 * non-NULL @sch from scx_prog_sched() - a live sched implies the table has
 * been allocated, so no NULL check is needed here.
 */
static inline s32 __scx_cid_to_cpu(s32 cid)
{
	/* READ_ONCE pairs with WRITE_ONCE in scx_cid_arrays_alloc() */
	return READ_ONCE(scx_cid_to_cpu_tbl)[cid];
}

/**
 * __scx_cpu_to_cid - Unchecked cpu->cid table lookup
 * @cpu: cpu to look up. Must be a valid possible cpu id.
 *
 * Same usage constraints as __scx_cid_to_cpu().
 */
static inline s32 __scx_cpu_to_cid(s32 cpu)
{
	return READ_ONCE(scx_cpu_to_cid_tbl)[cpu];
}

/**
 * scx_cid_to_cpu - Translate @cid to its cpu
 * @sch: scx_sched for error reporting
 * @cid: cid to look up
 *
 * Return the cpu for @cid or a negative errno on failure. Invalid cid triggers
 * scx_error() on @sch. The cid arrays are allocated on first scheduler enable
 * and never freed, so the returned cpu is stable for the lifetime of the loaded
 * scheduler.
 */
static inline s32 scx_cid_to_cpu(struct scx_sched *sch, s32 cid)
{
	if (!cid_valid(sch, cid))
		return -EINVAL;
	return __scx_cid_to_cpu(cid);
}

/**
 * scx_cpu_to_cid - Translate @cpu to its cid
 * @sch: scx_sched for error reporting
 * @cpu: cpu to look up
 *
 * Return the cid for @cpu or a negative errno on failure. Invalid cpu triggers
 * scx_error() on @sch. Same lifetime guarantee as scx_cid_to_cpu().
 */
static inline s32 scx_cpu_to_cid(struct scx_sched *sch, s32 cpu)
{
	if (!scx_cpu_valid(sch, cpu, NULL))
		return -EINVAL;
	return __scx_cpu_to_cid(cpu);
}

#endif /* _KERNEL_SCHED_EXT_CID_H */
