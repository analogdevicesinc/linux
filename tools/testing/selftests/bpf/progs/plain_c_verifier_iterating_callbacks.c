// SPDX-License-Identifier: GPL-2.0
/* verifier_iterating_callbacks.c in plain C */
#include "bpf_misc.h"
#include "bpf_experimental.h"

#define ARR_SZ 1000000
char arr[ARR_SZ];

SEC("socket")
__success __retval(0xd495cdc0)
int cond_break1(const void *ctx)
{
	unsigned long i;
	unsigned int sum = 0;

	for (i = 0; i < ARR_SZ; i++)
		sum += i;
	for (i = 0; i < ARR_SZ; i++) {
		sum += i + arr[i];
	}

	return sum;
}

char _license[] SEC("license") = "GPL";
