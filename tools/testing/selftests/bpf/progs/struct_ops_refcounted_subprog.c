// SPDX-License-Identifier: GPL-2.0
#include <vmlinux.h>
#include <bpf/bpf_tracing.h>
#include "../test_kmods/bpf_testmod.h"
#include "bpf_misc.h"

char _license[] SEC("license") = "GPL";

static __noinline int pid_of(unsigned long long *ctx)
{
	struct task_struct *task = (void *)ctx[1];

	return task->pid;
}

/* A static subprog loads the referenced argument from ctx, like the main prog. */
SEC("struct_ops/test_refcounted")
__success
int refcounted_subprog(unsigned long long *ctx)
{
	int pid = pid_of(ctx);

	bpf_task_release((void *)ctx[1]);
	return pid & 1;
}

SEC(".struct_ops.link")
struct bpf_testmod_ops testmod_refcounted_subprog = {
	.test_refcounted = (void *)refcounted_subprog,
};
