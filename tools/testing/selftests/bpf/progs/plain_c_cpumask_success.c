// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Meta Platforms, Inc. and affiliates. */
/* cpumask_success.c in plain C */

#include <vmlinux.h>
#include <bpf/bpf_tracing.h>
#include <bpf/bpf_helpers.h>

#include "bpf_misc.h"
#include "cpumask_common.h"

char _license[] SEC("license") = "GPL";

int pid, nr_cpus;

static bool is_test_task(void)
{
	int cur_pid = bpf_get_current_pid_tgid() >> 32;

	return pid == cur_pid;
}

/* Mask is guaranteed to be large enough for bpf_cpumask_t. */
#define CPUMASK_TEST_MASKLEN (sizeof(cpumask_t))

/* Add an extra word for the test_populate_reject_unaligned test. */
u64 bits[CPUMASK_TEST_MASKLEN / 8 + 1];

SEC("tp_btf/task_newtask")
__success
int BPF_PROG(test_populate, struct task_struct *task, u64 clone_flags)
{
	struct bpf_cpumask *mask;
	bool bit;
	int ret;
	int i;

	if (!is_test_task())
		return 0;

	/* Set only odd bits. */
	__builtin_memset(bits, 0xaa, CPUMASK_TEST_MASKLEN);

	mask = bpf_cpumask_create();
	if (!mask) {
		err = 1;
		return 0;
	}

	/* Pass the entire bits array, the kfunc will only copy the valid bits. */
	ret = bpf_cpumask_populate(mask, bits, CPUMASK_TEST_MASKLEN);
	if (ret) {
		err = 2;
		goto out;
	}

	/*
	 * Test is there to appease the verifier. We cannot directly
	 * access NR_CPUS, the upper bound for nr_cpus, so we infer
	 * it from the size of cpumask_t.
	 */
	if (nr_cpus < 0 || nr_cpus > CPUMASK_TEST_MASKLEN * 8) {
		err = 3;
		goto out;
	}

	for (i = 0; i < nr_cpus; i++) {
		/* Odd-numbered bits should be set, even ones unset. */
		bit = bpf_cpumask_test_cpu(i, (const struct cpumask *)mask);
		if (bit == (i % 2 != 0))
			continue;

		err = 4;
		break;
	}

out:
	bpf_cpumask_release(mask);

	return 0;
}

#undef CPUMASK_TEST_MASKLEN
