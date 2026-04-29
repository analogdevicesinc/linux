/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BPF extensible scheduler class: Documentation/scheduler/sched-ext.rst
 *
 * Copyright (c) 2026 Meta Platforms, Inc. and affiliates.
 * Copyright (c) 2026 Tejun Heo <tj@kernel.org>
 */
#include <linux/cacheinfo.h>

/*
 * Per-cpu scratch cmask used by scx_call_op_set_cpumask() to synthesize a
 * cmask from a cpumask. Allocated alongside the cid arrays on first enable
 * and never freed. Sized to the full cid space. Caller holds rq lock so
 * this_cpu_ptr is safe.
 */
struct scx_cmask __percpu *scx_set_cmask_scratch;

/*
 * cid tables.
 *
 * Pointers are published once on first enable and never revoked. The default
 * mapping is populated before ops.init() runs; scx_bpf_cid_override() commits
 * before it returns. As long as the BPF scheduler only uses the tables from
 * those points onward, it sees a consistent view.
 */
s16 *scx_cid_to_cpu_tbl;
s16 *scx_cpu_to_cid_tbl;
struct scx_cid_topo *scx_cid_topo;

#define SCX_CID_TOPO_NEG	(struct scx_cid_topo) {				\
	.core_cid = -1, .core_idx = -1, .llc_cid = -1, .llc_idx = -1,		\
	.node_cid = -1, .node_idx = -1,						\
}

/*
 * Return @cpu's LLC shared_cpu_map. If cacheinfo isn't populated (offline or
 * !present), record @cpu in @fallbacks and return its node mask instead - the
 * worst that can happen is that the cpu's LLC becomes coarser than reality.
 */
static const struct cpumask *cpu_llc_mask(int cpu, struct cpumask *fallbacks)
{
	struct cpu_cacheinfo *ci = get_cpu_cacheinfo(cpu);

	if (!ci || !ci->info_list || !ci->num_leaves) {
		cpumask_set_cpu(cpu, fallbacks);
		return cpumask_of_node(cpu_to_node(cpu));
	}
	return &ci->info_list[ci->num_leaves - 1].shared_cpu_map;
}

/* Allocate the cid tables once on first enable; never freed. */
static s32 scx_cid_arrays_alloc(void)
{
	u32 npossible = num_possible_cpus();
	s16 *cid_to_cpu, *cpu_to_cid;
	struct scx_cid_topo *cid_topo;
	struct scx_cmask __percpu *set_cmask_scratch;

	if (scx_cid_to_cpu_tbl)
		return 0;

	cid_to_cpu = kzalloc_objs(*scx_cid_to_cpu_tbl, npossible, GFP_KERNEL);
	cpu_to_cid = kzalloc_objs(*scx_cpu_to_cid_tbl, nr_cpu_ids, GFP_KERNEL);
	cid_topo = kmalloc_objs(*scx_cid_topo, npossible, GFP_KERNEL);
	set_cmask_scratch = __alloc_percpu(struct_size(set_cmask_scratch, bits,
						       SCX_CMASK_NR_WORDS(npossible)),
					   sizeof(u64));

	if (!cid_to_cpu || !cpu_to_cid || !cid_topo || !set_cmask_scratch) {
		kfree(cid_to_cpu);
		kfree(cpu_to_cid);
		kfree(cid_topo);
		free_percpu(set_cmask_scratch);
		return -ENOMEM;
	}

	WRITE_ONCE(scx_cid_to_cpu_tbl, cid_to_cpu);
	WRITE_ONCE(scx_cpu_to_cid_tbl, cpu_to_cid);
	WRITE_ONCE(scx_cid_topo, cid_topo);
	WRITE_ONCE(scx_set_cmask_scratch, set_cmask_scratch);
	return 0;
}

/**
 * scx_cid_init - build the cid mapping
 * @sch: the scx_sched being initialized; used as the scx_error() target
 *
 * See "Topological CPU IDs" in ext_cid.h for the model. Walk online cpus by
 * intersection at each level (parent_scratch & this_level_mask), which keeps
 * containment correct by construction and naturally splits a physical LLC
 * straddling two NUMA nodes into two LLC units. The caller must hold
 * cpus_read_lock.
 */
s32 scx_cid_init(struct scx_sched *sch)
{
	cpumask_var_t to_walk __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t node_scratch __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t llc_scratch __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t core_scratch __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t llc_fallback __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t online_no_topo __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	u32 next_cid = 0;
	s32 next_node_idx = 0, next_llc_idx = 0, next_core_idx = 0;
	s32 cpu, ret;

	/* CMASK_MAX_WORDS in cid.bpf.h covers NR_CPUS up to 8192 */
	BUILD_BUG_ON(NR_CPUS > 8192);

	lockdep_assert_cpus_held();

	ret = scx_cid_arrays_alloc();
	if (ret)
		return ret;

	if (!zalloc_cpumask_var(&to_walk, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&node_scratch, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&llc_scratch, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&core_scratch, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&llc_fallback, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&online_no_topo, GFP_KERNEL))
		return -ENOMEM;

	/* -1 sentinels for sparse-possible cpu id holes (0 is a valid cid) */
	for (cpu = 0; cpu < nr_cpu_ids; cpu++)
		scx_cpu_to_cid_tbl[cpu] = -1;

	cpumask_copy(to_walk, cpu_online_mask);

	while (!cpumask_empty(to_walk)) {
		s32 next_cpu = cpumask_first(to_walk);
		s32 nid = cpu_to_node(next_cpu);
		s32 node_cid = next_cid;
		s32 node_idx;

		/*
		 * No NUMA info: skip and let the tail loop assign a no-topo
		 * cid. cpumask_of_node(-1) is undefined.
		 */
		if (nid < 0) {
			cpumask_clear_cpu(next_cpu, to_walk);
			continue;
		}

		node_idx = next_node_idx++;

		/* node_scratch = to_walk & this node */
		cpumask_and(node_scratch, to_walk, cpumask_of_node(nid));
		if (WARN_ON_ONCE(!cpumask_test_cpu(next_cpu, node_scratch)))
			return -EINVAL;

		while (!cpumask_empty(node_scratch)) {
			s32 ncpu = cpumask_first(node_scratch);
			const struct cpumask *llc_mask = cpu_llc_mask(ncpu, llc_fallback);
			s32 llc_cid = next_cid;
			s32 llc_idx = next_llc_idx++;

			/* llc_scratch = node_scratch & this llc */
			cpumask_and(llc_scratch, node_scratch, llc_mask);
			if (WARN_ON_ONCE(!cpumask_test_cpu(ncpu, llc_scratch)))
				return -EINVAL;

			while (!cpumask_empty(llc_scratch)) {
				s32 lcpu = cpumask_first(llc_scratch);
				const struct cpumask *sib = topology_sibling_cpumask(lcpu);
				s32 core_cid = next_cid;
				s32 core_idx = next_core_idx++;
				s32 ccpu;

				/* core_scratch = llc_scratch & this core */
				cpumask_and(core_scratch, llc_scratch, sib);
				if (WARN_ON_ONCE(!cpumask_test_cpu(lcpu, core_scratch)))
					return -EINVAL;

				for_each_cpu(ccpu, core_scratch) {
					s32 cid = next_cid++;

					scx_cid_to_cpu_tbl[cid] = ccpu;
					scx_cpu_to_cid_tbl[ccpu] = cid;
					scx_cid_topo[cid] = (struct scx_cid_topo){
						.core_cid = core_cid,
						.core_idx = core_idx,
						.llc_cid = llc_cid,
						.llc_idx = llc_idx,
						.node_cid = node_cid,
						.node_idx = node_idx,
					};

					cpumask_clear_cpu(ccpu, llc_scratch);
					cpumask_clear_cpu(ccpu, node_scratch);
					cpumask_clear_cpu(ccpu, to_walk);
				}
			}
		}
	}

	/*
	 * No-topo section: any possible cpu without a cid - normally just the
	 * not-online ones. Collect any currently-online cpus that land here in
	 * @online_no_topo so we can warn about them at the end.
	 */
	for_each_cpu(cpu, cpu_possible_mask) {
		s32 cid;

		if (__scx_cpu_to_cid(cpu) != -1)
			continue;
		if (cpu_online(cpu))
			cpumask_set_cpu(cpu, online_no_topo);

		cid = next_cid++;
		scx_cid_to_cpu_tbl[cid] = cpu;
		scx_cpu_to_cid_tbl[cpu] = cid;
		scx_cid_topo[cid] = SCX_CID_TOPO_NEG;
	}

	if (!cpumask_empty(llc_fallback))
		pr_warn("scx_cid: cpus without cacheinfo, using node mask as llc: %*pbl\n",
			cpumask_pr_args(llc_fallback));
	if (!cpumask_empty(online_no_topo))
		pr_warn("scx_cid: online cpus with no usable topology: %*pbl\n",
			cpumask_pr_args(online_no_topo));

	return 0;
}

/**
 * scx_cpumask_to_cmask - Translate a kernel cpumask into a cmask
 * @src: source cpumask
 * @dst: cmask to write
 *
 * Initialize @dst to cover the full cid space [0, num_possible_cpus()) and
 * set the bit for each cid whose cpu is in @src.
 */
void scx_cpumask_to_cmask(const struct cpumask *src, struct scx_cmask *dst)
{
	s32 cpu;

	scx_cmask_init(dst, 0, num_possible_cpus());
	for_each_cpu(cpu, src) {
		s32 cid = __scx_cpu_to_cid(cpu);

		if (cid >= 0)
			__scx_cmask_set(dst, cid);
	}
}

__bpf_kfunc_start_defs();

/**
 * scx_bpf_cid_override - Install an explicit cpu->cid mapping
 * @cpu_to_cid: array of nr_cpu_ids s32 entries (cid for each cpu)
 * @cpu_to_cid__sz: must be nr_cpu_ids * sizeof(s32) bytes
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * May only be called from ops.init() of the root scheduler. Replace the
 * topology-probed cid mapping with the caller-provided one. Each possible cpu
 * must map to a unique cid in [0, num_possible_cpus()). Topo info is cleared.
 * On invalid input, trigger scx_error() to abort the scheduler.
 */
__bpf_kfunc void scx_bpf_cid_override(const s32 *cpu_to_cid, u32 cpu_to_cid__sz,
				      const struct bpf_prog_aux *aux)
{
	cpumask_var_t seen __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	struct scx_sched *sch;
	bool alloced;
	s32 cpu, cid;

	/* GFP_KERNEL alloc must happen before the rcu read section */
	alloced = zalloc_cpumask_var(&seen, GFP_KERNEL);

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch))
		return;

	if (!alloced) {
		scx_error(sch, "scx_bpf_cid_override: failed to allocate cpumask");
		return;
	}

	if (scx_parent(sch)) {
		scx_error(sch, "scx_bpf_cid_override() only allowed from root sched");
		return;
	}

	if (cpu_to_cid__sz != nr_cpu_ids * sizeof(s32)) {
		scx_error(sch, "scx_bpf_cid_override: expected %zu bytes, got %u",
			  nr_cpu_ids * sizeof(s32), cpu_to_cid__sz);
		return;
	}

	for_each_possible_cpu(cpu) {
		s32 c = cpu_to_cid[cpu];

		if (!cid_valid(sch, c))
			return;
		if (cpumask_test_and_set_cpu(c, seen)) {
			scx_error(sch, "cid %d assigned to multiple cpus", c);
			return;
		}
		scx_cpu_to_cid_tbl[cpu] = c;
		scx_cid_to_cpu_tbl[c] = cpu;
	}

	/* Invalidate stale topo info - the override carries no topology. */
	for (cid = 0; cid < num_possible_cpus(); cid++)
		scx_cid_topo[cid] = SCX_CID_TOPO_NEG;
}

/**
 * scx_bpf_cid_to_cpu - Return the raw CPU id for @cid
 * @cid: cid to look up
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * Return the raw CPU id for @cid. Trigger scx_error() and return -EINVAL if
 * @cid is invalid. The cid<->cpu mapping is static for the lifetime of the
 * loaded scheduler, so the BPF side can cache the result to avoid repeated
 * kfunc invocations.
 */
__bpf_kfunc s32 scx_bpf_cid_to_cpu(s32 cid, const struct bpf_prog_aux *aux)
{
	struct scx_sched *sch;

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch))
		return -EINVAL;
	return scx_cid_to_cpu(sch, cid);
}

/**
 * scx_bpf_cpu_to_cid - Return the cid for @cpu
 * @cpu: cpu to look up
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * Return the cid for @cpu. Trigger scx_error() and return -EINVAL if @cpu is
 * invalid. The cid<->cpu mapping is static for the lifetime of the loaded
 * scheduler, so the BPF side can cache the result to avoid repeated kfunc
 * invocations.
 */
__bpf_kfunc s32 scx_bpf_cpu_to_cid(s32 cpu, const struct bpf_prog_aux *aux)
{
	struct scx_sched *sch;

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch))
		return -EINVAL;
	return scx_cpu_to_cid(sch, cpu);
}

/**
 * scx_bpf_cid_topo - Copy out per-cid topology info
 * @cid: cid to look up
 * @out__uninit: where to copy the topology info; fully written by this call
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * Fill @out__uninit with the topology info for @cid. Trigger scx_error() if
 * @cid is out of range. If @cid is valid but in the no-topo section, all fields
 * are set to -1.
 */
__bpf_kfunc void scx_bpf_cid_topo(s32 cid, struct scx_cid_topo *out__uninit,
				  const struct bpf_prog_aux *aux)
{
	struct scx_sched *sch;

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch) || !cid_valid(sch, cid)) {
		*out__uninit = SCX_CID_TOPO_NEG;
		return;
	}

	*out__uninit = READ_ONCE(scx_cid_topo)[cid];
}

__bpf_kfunc_end_defs();

BTF_KFUNCS_START(scx_kfunc_ids_init)
BTF_ID_FLAGS(func, scx_bpf_cid_override, KF_IMPLICIT_ARGS | KF_SLEEPABLE)
BTF_KFUNCS_END(scx_kfunc_ids_init)

static const struct btf_kfunc_id_set scx_kfunc_set_init = {
	.owner	= THIS_MODULE,
	.set	= &scx_kfunc_ids_init,
	.filter	= scx_kfunc_context_filter,
};

BTF_KFUNCS_START(scx_kfunc_ids_cid)
BTF_ID_FLAGS(func, scx_bpf_cid_to_cpu, KF_IMPLICIT_ARGS)
BTF_ID_FLAGS(func, scx_bpf_cpu_to_cid, KF_IMPLICIT_ARGS)
BTF_ID_FLAGS(func, scx_bpf_cid_topo, KF_IMPLICIT_ARGS)
BTF_KFUNCS_END(scx_kfunc_ids_cid)

static const struct btf_kfunc_id_set scx_kfunc_set_cid = {
	.owner	= THIS_MODULE,
	.set	= &scx_kfunc_ids_cid,
};

int scx_cid_kfunc_init(void)
{
	return register_btf_kfunc_id_set(BPF_PROG_TYPE_STRUCT_OPS, &scx_kfunc_set_init) ?:
		register_btf_kfunc_id_set(BPF_PROG_TYPE_STRUCT_OPS, &scx_kfunc_set_cid) ?:
		register_btf_kfunc_id_set(BPF_PROG_TYPE_TRACING, &scx_kfunc_set_cid) ?:
		register_btf_kfunc_id_set(BPF_PROG_TYPE_SYSCALL, &scx_kfunc_set_cid);
}
