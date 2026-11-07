// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "../test_kmods/bpf_testmod_kfunc.h"
#include "bpf_misc.h"

#if (defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64)) && \
	defined(__BPF_FEATURE_STACK_ARGUMENT)

/* Force kfunc extern BTF generation for inline asm call below.
 * Uses its own SEC so it's not included as a .text subprog.
 * The '?' prefix sets autoload=false so libbpf won't load it.
 */
SEC("?tc")
int __btf_kfunc_gen(struct __sk_buff *ctx)
{
	char buf[8] = {};

	return bpf_kfunc_call_stack_arg_mem(0, 0, 0, 0, 0, buf, sizeof(buf));
}

/*
 * Test that the range of a stack arg is known across bpf-to-bpf call for kfunc stack arg.
 * subprog_call_mem_kfunc receives a size as incoming stack arg (arg6)
 * and forwards it as mem__sz (arg7) to bpf_kfunc_call_stack_arg_mem.
 */
__naked __noinline __used
static long subprog_call_mem_kfunc(long a, long b, long c, long d, long e, long size)
{
	asm volatile (
		"r1 = *(u64 *)(r11 + 8);"	/* r1 = incoming arg6 (size) */
		"r2 = 0x0807060504030201 ll;"	/* r2 = buf contents */
		"*(u64 *)(r10 - 8) = r2;"	/* store buf to stack */
		"r2 = r10;"
		"r2 += -8;"			/* r2 = &buf */
		"*(u64 *)(r11 - 8) = r2;"	/* outgoing arg6 = buf */
		"*(u64 *)(r11 - 16) = r1;"	/* outgoing arg7 = size */
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call %[bpf_kfunc_call_stack_arg_mem];"
		"exit;"
		:
		: __imm(bpf_kfunc_call_stack_arg_mem)
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: range across bpf2bpf call for kfunc")
__success
__log_level(2)
__msg("from 9 to 11: joined")
__msg("13: (79) r1 = *(u64 *)(r11 +8)        ; R1=scalar(smin=umin=smin32=umin32=4,smax=umax=smax32=umax32=6,var_off=(0x4; 0x2),step=0+2)")
__btf_func_path("btf__stack_arg_precision.bpf.o")
__naked void stack_arg_precision_bpf2bpf(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r6 = r0;"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"if r6 < 2 goto l0_%=;"
		"*(u64 *)(r11 - 8) = 4;"
		"goto l1_%=;"
	"l0_%=:"
		"*(u64 *)(r11 - 8) = 6;"
	"l1_%=:"
		"call subprog_call_mem_kfunc;"
		"exit;"
		:: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

#else

SEC("socket")
__description("stack_arg_precision: not supported, dummy test")
__skip("stack_arg_precision: not supported")
__success
int dummy_test(void)
{
	return 0;
}

#endif

char _license[] SEC("license") = "GPL";
