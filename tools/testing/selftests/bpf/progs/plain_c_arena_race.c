// SPDX-License-Identifier: GPL-2.0
/* arena_race.c in plain C */

#define BPF_NO_KFUNC_PROTOTYPES
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_experimental.h"
#include <bpf_arena_common.h>
#include "bpf_misc.h"

int target_tid;
int pause_on_zap;
int zap_entered;
int release;
int timed_out;

static void wait_for_release(void)
{
	while (!*(volatile int *)&release)
		;
	if (!*(volatile int *)&release)
		timed_out = 1;
}

/*
 * Pause a free while it zaps user mappings: after it cleared the kernel PTEs
 * and flushed the TLB, but before it releases the range. zap_vma_range() is
 * generic MM code, so it is an fentry target even where
 * flush_tlb_kernel_range() is inline (arm64).
 */
SEC("?fentry/zap_vma_range")
__success
int BPF_PROG(trace_zap, struct vm_area_struct *vma, unsigned long address,
	     unsigned long size)
{
	if ((__u32)bpf_get_current_pid_tgid() != target_tid)
		return 0;
	zap_entered = 1;
	if (pause_on_zap)
		wait_for_release();
	return 0;
}

char _license[] SEC("license") = "GPL";
