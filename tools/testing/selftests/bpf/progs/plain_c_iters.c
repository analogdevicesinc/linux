// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Meta Platforms, Inc. and affiliates. */
/* iters.c in plain C */

#include <stdbool.h>
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"
#include "bpf_compiler.h"

int my_pid;

#ifdef REAL_TEST
#define MY_PID_GUARD() if (my_pid != (bpf_get_current_pid_tgid() >> 32)) return 0
#else
#define MY_PID_GUARD() ({ })
#endif

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__type(key, int);
	__type(value, int);
	__uint(max_entries, 1000);
} hash_map SEC(".maps");

SEC("?raw_tp")
__failure
__msg("invalid mem access 'map_value_or_null'")
int iter_err_too_permissive2(const void *ctx)
{
	int *map_val = NULL;
	int key = 0;

	MY_PID_GUARD();

	map_val = bpf_map_lookup_elem(&hash_map, &key);
	if (!map_val)
		return 0;

	for (int i = 0; i < 1000000; i++) {
		map_val = bpf_map_lookup_elem(&hash_map, &key);
	}

	*map_val = 123;

	return 0;
}

SEC("?raw_tp")
__failure
__msg("invalid mem access 'map_value_or_null'")
int iter_err_too_permissive3(const void *ctx)
{
	int *map_val = NULL;
	int key = 0;
	bool found = false;

	MY_PID_GUARD();

	for (int i = 0; i < 1000000; i++) {
		map_val = bpf_map_lookup_elem(&hash_map, &key);
		found = true;
	}

	if (found)
		*map_val = 123;

	return 0;
}

__u32 upper, select_n, result;
__u64 global;

static __noinline bool nest_2(char *str)
{
	/*
	 * some insns (including branch insns) to ensure stacksafe() is triggered
	 * in nest_2(). This way, stacksafe() can compare frame associated with nest_1().
	 */
	if (str[0] == 't')
		return true;
	if (str[1] == 'e')
		return true;
	if (str[2] == 's')
		return true;
	if (str[3] == 't')
		return true;
	return false;
}

static __noinline bool nest_1(int n)
{
	/* case 0: allocate stack, case 1: no allocate stack */
	switch (n) {
	case 0: {
		char comm[16];

		if (bpf_get_current_comm(comm, 16))
			return false;
		return nest_2(comm);
	}
	case 1:
		return nest_2((char *)&global);
	default:
		return false;
	}
}

SEC("raw_tp")
__success __retval(0)
int iter_subprog_check_stacksafe(const void *ctx)
{
	long i;

	for (i = 0; i < upper; i++) {
		if (!nest_1(select_n)) {
			result = 1;
			return 0;
		}
	}

	result = 2;
	return 0;
}

SEC("raw_tp")
__success
int clean_live_states(const void *ctx)
{
	char buf[1];
	int i, j, k, l, m, n, o;

	for (i = 0; i < 10; i++)
	for (j = 0; j < 10; j++)
	for (k = 0; k < 10; k++)
	for (l = 0; l < 10; l++)
	for (m = 0; m < 10; m++)
	for (n = 0; n < 10; n++)
	for (o = 0; o < 10; o++) {
		if (unlikely(bpf_get_prandom_u32()))
			buf[0] = 42;
		bpf_printk("%s", buf);
	}
	return 0;
}

char _license[] SEC("license") = "GPL";
