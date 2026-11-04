// SPDX-License-Identifier: GPL-2.0
/*
 * A scheduler that keeps the target task waiting on a DSQ when it wakes up,
 * until it's switched out of SCHED_EXT or, if @park_ns is set, for @park_ns.
 * This lets userspace switch the target out while it's waiting.
 *
 * Copyright (c) 2026 Google LLC.
 */

#include <scx/common.bpf.h>

char _license[] SEC("license") = "GPL";

#define PARK_DSQ 0

UEI_DEFINE(uei);

s32 target_pid;		/* set by userspace */
bool park;		/* set by userspace, cleared when parking */
u64 park_ns;		/* set by userspace, how long to park, 0 for no limit */
u64 nr_parked;		/* how many times the target has been parked */
u64 nr_released;	/* how many times the target has been released */

static bool released;	/* set by the timer, cleared when parking */
static s32 target_cpu;	/* CPU to kick when the timer fires */

struct release_timer {
	struct bpf_timer timer;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, u32);
	__type(value, struct release_timer);
} release_timer SEC(".maps");

static int release_timerfn(void *map, int *key, struct bpf_timer *timer)
{
	released = true;
	scx_bpf_kick_cpu(target_cpu, 0);
	return 0;
}

s32 BPF_STRUCT_OPS(class_switch_stall_select_cpu, struct task_struct *p,
		   s32 prev_cpu, u64 wake_flags)
{
	/* Don't dispatch directly so that wakeups go to ops.enqueue(). */
	return prev_cpu;
}

void BPF_STRUCT_OPS(class_switch_stall_enqueue, struct task_struct *p,
		    u64 enq_flags)
{
	struct release_timer *rt;
	u32 key = 0;

	if (p->pid != target_pid || !park || !(enq_flags & SCX_ENQ_WAKEUP)) {
		scx_bpf_dsq_insert(p, SCX_DSQ_GLOBAL, SCX_SLICE_DFL, enq_flags);
		return;
	}

	park = false;
	released = false;
	target_cpu = scx_bpf_task_cpu(p);
	scx_bpf_dsq_insert(p, PARK_DSQ, SCX_SLICE_DFL, enq_flags);
	__sync_fetch_and_add(&nr_parked, 1);

	if (!park_ns)
		return;

	rt = bpf_map_lookup_elem(&release_timer, &key);
	if (!rt) {
		scx_bpf_error("failed to look up the release timer");
		return;
	}
	bpf_timer_start(&rt->timer, park_ns, 0);
}

void BPF_STRUCT_OPS(class_switch_stall_dispatch, s32 cpu,
		    struct task_struct *prev)
{
	if (released && scx_bpf_dsq_move_to_local(PARK_DSQ, 0))
		__sync_fetch_and_add(&nr_released, 1);
}

s32 BPF_STRUCT_OPS_SLEEPABLE(class_switch_stall_init)
{
	struct release_timer *rt;
	u32 key = 0;
	s32 ret;

	ret = scx_bpf_create_dsq(PARK_DSQ, -1);
	if (ret)
		return ret;

	rt = bpf_map_lookup_elem(&release_timer, &key);
	if (!rt)
		return -ESRCH;

	bpf_timer_init(&rt->timer, &release_timer, CLOCK_MONOTONIC);
	bpf_timer_set_callback(&rt->timer, release_timerfn);
	return 0;
}

void BPF_STRUCT_OPS(class_switch_stall_exit, struct scx_exit_info *ei)
{
	UEI_RECORD(uei, ei);
}

SEC(".struct_ops.link")
struct sched_ext_ops class_switch_stall_ops = {
	.select_cpu		= (void *)class_switch_stall_select_cpu,
	.enqueue		= (void *)class_switch_stall_enqueue,
	.dispatch		= (void *)class_switch_stall_dispatch,
	.init			= (void *)class_switch_stall_init,
	.exit			= (void *)class_switch_stall_exit,
	.flags			= SCX_OPS_SWITCH_PARTIAL,
	.name			= "class_switch_stall",
};
