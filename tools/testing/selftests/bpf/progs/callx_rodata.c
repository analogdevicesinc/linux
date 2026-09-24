// SPDX-License-Identifier: GPL-2.0
/* callx through pointers to functions in .rodata, loaded by light skeleton */

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>

typedef int (*op_fn)(int);

/* set by user space before the programs are loaded, it's in .rodata too */
const volatile int bias = 1;

int op_idx;

static __noinline int add_bias(int x)
{
	return x + bias;
}

static __noinline int mul3(int x)
{
	return x * 3;
}

/* volatile, so that the compiler doesn't replace the table with direct calls */
static op_fn const volatile ops[] = { add_bias, mul3 };

SEC("socket")
int select_op(void *ctx)
{
	unsigned int i = op_idx;

	if (i >= sizeof(ops) / sizeof(ops[0]))
		return -1;
	return ops[i](10);
}

/* functions have other offsets in this program */
SEC("socket")
int both_ops(void *ctx)
{
	return ops[1](ops[0](4));
}

/* no callx here */
SEC("socket")
int read_bias(void *ctx)
{
	return bias;
}

char _license[] SEC("license") = "GPL";
