// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"
#include "../test_kmods/bpf_testmod_kfunc.h"

/* Keep the kfunc BTF records used by the inline assembly. */
void __kfunc_btf_root(void)
{
	asm volatile ("" :
		: "r"(&bpf_kfunc_test_uninit_struct),
		  "r"(&bpf_kfunc_test_uninit_mem),
		  "r"(&bpf_kfunc_test_uninit_alias));
}

SEC("tc")
__success __retval(10)
__flag(BPF_F_TEST_STATE_FREQ)
__caps_unpriv(CAP_BPF | CAP_NET_ADMIN)
__prepare_priv
__success_unpriv
__naked void struct_poisoned_at_checkpoint(void)
{
	asm volatile (
		"*(u64 *)(r10 - 16) = 0;"
		"*(u64 *)(r10 - 8) = 0;"
		"goto +0;"
		"r1 = r10;"
		"r1 += -16;"
		"call %[bpf_kfunc_test_uninit_struct];"
		"r0 = *(u32 *)(r10 - 16);"
		"r1 = *(u32 *)(r10 - 12);"
		"r0 += r1;"
		"r1 = *(u32 *)(r10 - 8);"
		"r0 += r1;"
		"r1 = *(u32 *)(r10 - 4);"
		"r0 += r1;"
		"exit;"
		: : __imm(bpf_kfunc_test_uninit_struct) : __clobber_all);
}

SEC("tc")
__success __retval(0x2a2a2a2a)
__flag(BPF_F_TEST_STATE_FREQ)
__caps_unpriv(CAP_BPF | CAP_NET_ADMIN)
__prepare_priv
__success_unpriv
__naked void sized_buffer_poisoned_at_checkpoint(void)
{
	asm volatile (
		"*(u64 *)(r10 - 8) = 0;"
		"goto +0;"
		"r1 = r10;"
		"r1 += -8;"
		"r2 = 8;"
		"call %[bpf_kfunc_test_uninit_mem];"
		"r0 = *(u32 *)(r10 - 8);"
		"r1 = *(u32 *)(r10 - 4);"
		"if r0 == r1 goto +1;"
		"r0 = 0;"
		"exit;"
		: : __imm(bpf_kfunc_test_uninit_mem) : __clobber_all);
}

SEC("tc")
__success __retval(7)
__caps_unpriv(CAP_BPF | CAP_NET_ADMIN)
__prepare_priv
__success_unpriv
__naked void initialized_input_alias(void)
{
	asm volatile (
		"*(u32 *)(r10 - 8) = 7;"
		"r1 = r10;"
		"r1 += -8;"
		"r2 = r1;"
		"call %[bpf_kfunc_test_uninit_alias];"
		"exit;"
		: : __imm(bpf_kfunc_test_uninit_alias) : __clobber_all);
}

SEC("tc")
__success
__caps_unpriv(CAP_BPF | CAP_NET_ADMIN)
__prepare_priv
__failure_unpriv __msg_unpriv("invalid read from stack")
__naked void uninitialized_input_alias(void)
{
	asm volatile (
		"r1 = r10;"
		"r1 += -8;"
		"r2 = r1;"
		"call %[bpf_kfunc_test_uninit_alias];"
		"exit;"
		: : __imm(bpf_kfunc_test_uninit_alias) : __clobber_all);
}

char _license[] SEC("license") = "GPL";
