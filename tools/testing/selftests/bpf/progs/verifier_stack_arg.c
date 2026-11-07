// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(max_entries, 1);
	__type(key, long long);
	__type(value, long long);
} map_hash_8b SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_RINGBUF);
	__uint(max_entries, 4096);
} ringbuf SEC(".maps");

#if defined(__BPF_FEATURE_STACK_ARGUMENT)

struct {
	__uint(type, BPF_MAP_TYPE_PROG_ARRAY);
	__uint(max_entries, 1);
	__uint(key_size, sizeof(__u32));
	__uint(value_size, sizeof(__u32));
} jmp_table SEC(".maps");

/* BTF has what the source says. The compiler dropped the arguments that are not used. */
static __naked __noinline __used
long takes_5_says_7(long a, long b, long c, long d, long e, long f, long g)
{
	asm volatile ("r0 = r1; r0 += r2; r0 += r3; r0 += r4; r0 += r5; exit;" ::: __clobber_all);
}

#define CALL_TAKES_5_SAYS_7(stores) \
	"r1 = 1; r2 = 2; r3 = 3; r4 = 4; r5 = 5;" stores "call takes_5_says_7; exit;"

/* No insn uses r11: the JIT needs no stack arguments, and a tail call is allowed */
SEC("tc")
__description("stack_arg: static subprog takes less than its prototype says")
__success __retval(15)
__load_if_JITed()
__arch_x86_64 __arch_arm64 __arch_riscv64 __arch_s390x __arch_loongarch
__naked void less_than_prototype(void)
{
	asm volatile ("r2 = %[jmp_table] ll; r3 = 0; call %[bpf_tail_call];"
		      CALL_TAKES_5_SAYS_7("")
		      :: __imm_addr(jmp_table), __imm(bpf_tail_call) : __clobber_all);
}

#endif

#if (defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64)) && \
	defined(__BPF_FEATURE_STACK_ARGUMENT)

__noinline __used
static int subprog_6args(int a, int b, int c, int d, int e, int f)
{
	return a + b + c + d + e + f;
}

__noinline __used
static int subprog_7args(int a, int b, int c, int d, int e, int f, int g)
{
	return a + b + c + d + e + f + g;
}

__noinline __used
static long subprog_deref_arg6(long a, long b, long c, long d, long e, long *f)
{
	return *f;
}

SEC("tc")
__description("stack_arg: subprog with 6 args")
__success __retval(21)
__naked void stack_arg_6args(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 8) = 6;"
		"call subprog_6args;"
		"exit;"
		::: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: two subprogs with >5 args")
__success __retval(90)
__naked void stack_arg_two_subprogs(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 8) = 10;"
		"call subprog_6args;"
		"r6 = r0;"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 16) = 30;"
		"*(u64 *)(r11 - 8) = 20;"
		"call subprog_7args;"
		"r0 += r6;"
		"exit;"
		::: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: read from uninitialized stack arg slot")
__failure
__msg("invalid read from stack arg off 8 depth 0")
__naked void stack_arg_read_uninitialized(void)
{
	asm volatile (
		"r0 = *(u64 *)(r11 + 8);"
		"r0 = 0;"
		"exit;"
		::: __clobber_all
	);
}

/* BTF has 'a'. The code also takes the 6th argument. It passes none: rsp stays where it is. */
static __naked __noinline __used long takes_6th_says_1(long a)
{
	asm volatile ("r0 = *(u64 *)(r11 + 8); r0 += r1; exit;" ::: __clobber_all);
}

SEC("tc")
__description("stack_arg: static subprog takes more than its prototype says")
__success __retval(7)
__load_if_JITed()
__arch_x86_64
__jited("func #1")
__jited("...")
__jited("	movq	%r9, %rax")
__jited("	addq	%rdi, %rax")
__jited("	leave")
__arch_arm64 __arch_riscv64
__naked void more_than_prototype(void)
{
	asm volatile ("r1 = 1; *(u64 *)(r11 - 8) = 6; call takes_6th_says_1; exit;"
		      ::: __clobber_all);
}

SEC("tc")
__description("stack_arg: static subprog takes more than its prototype says, no store")
__failure __msg("callee expects 6 args, stack arg1 is not initialized")
__naked void more_than_prototype_no_store(void)
{
	asm volatile ("r1 = 1; call takes_6th_says_1; exit;" ::: __clobber_all);
}

/* BTF is the only source for a global subprog: it has no stack arguments. */
__noinline __used long global_loads_6th(long a)
{
	asm volatile ("%0 = *(u64 *)(r11 + 8);" : "=r"(a));
	return a;
}

SEC("tc")
__description("stack_arg: global subprog loads the 6th argument")
__failure __msg("invalid read from stack arg off 8 depth 0")
__naked void global_more_than_prototype(void)
{
	asm volatile ("r1 = 1; call global_loads_6th; exit;" ::: __clobber_all);
}

SEC("tc")
__description("stack_arg: gap at offset -8, only wrote -16")
__failure
__msg("callee expects 7 args, stack arg1 is not initialized")
__naked void stack_arg_gap_at_minus8(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 16) = 30;"
		"call subprog_7args;"
		"exit;"
		::: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: pruning with different stack arg types")
__failure __log_level(2)
__flag(BPF_F_TEST_STATE_FREQ)
__msg("arg JOIN insn 8 -> 10 r1: _ + fp0-8 => fp0-8|fp0+0")
__msg("arg JOIN insn 8 -> 10 sa0: _ + fp0-8 => fp0-8|fp0+0")
__msg("R{{[0-9]}} invalid mem access 'scalar'")
__naked void stack_arg_pruning_type_mismatch(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r6 = r0;"
		/* local = 0 on program stack */
		"r7 = 0;"
		"*(u64 *)(r10 - 8) = r7;"
		/* Branch based on random value */
		"if r6 s> 3 goto l0_%=;"
		/* Path 1: store stack pointer to outgoing arg6 */
		"r1 = r10;"
		"r1 += -8;"
		"*(u64 *)(r11 - 8) = r1;"
		"goto l1_%=;"
	"l0_%=:"
		/* Path 2: store scalar to outgoing arg6 */
		"*(u64 *)(r11 - 8) = 42;"
	"l1_%=:"
		/* Call subprog that dereferences arg6 */
		"r1 = r6;"
		"r2 = 0;"
		"r3 = 0;"
		"r4 = 0;"
		"r5 = 0;"
		"call subprog_deref_arg6;"
		"exit;"
		:: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: release_reference invalidates stack arg slot")
__failure
__msg("callee expects 6 args, stack arg1 is not initialized")
__naked void stack_arg_release_ref(void)
{
	asm volatile (
		"r6 = r1;"
		/* struct bpf_sock_tuple tuple = {} */
		"r2 = 0;"
		"*(u32 *)(r10 - 8) = r2;"
		"*(u64 *)(r10 - 16) = r2;"
		"*(u64 *)(r10 - 24) = r2;"
		"*(u64 *)(r10 - 32) = r2;"
		"*(u64 *)(r10 - 40) = r2;"
		"*(u64 *)(r10 - 48) = r2;"
		/* sk = bpf_sk_lookup_tcp(ctx, &tuple, sizeof(tuple), 0, 0) */
		"r1 = r6;"
		"r2 = r10;"
		"r2 += -48;"
		"r3 = %[sizeof_bpf_sock_tuple];"
		"r4 = 0;"
		"r5 = 0;"
		"call %[bpf_sk_lookup_tcp];"
		/* r0 = sk (PTR_TO_SOCK_OR_NULL) */
		"if r0 == 0 goto l0_%=;"
		/* Store sock ref to outgoing arg6 slot */
		"*(u64 *)(r11 - 8) = r0;"
		/* Release the reference — invalidates the stack arg slot */
		"r1 = r0;"
		"call %[bpf_sk_release];"
		/* Call subprog that dereferences arg6 — should fail */
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call subprog_deref_arg6;"
	"l0_%=:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_sk_lookup_tcp),
		  __imm(bpf_sk_release),
		  __imm_const(sizeof_bpf_sock_tuple, sizeof(struct bpf_sock_tuple))
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: pkt pointer in stack arg slot invalidated after pull_data")
__failure
__msg("callee expects 6 args, stack arg1 is not initialized")
__naked void stack_arg_stale_pkt_ptr(void)
{
	asm volatile (
		"r6 = r1;"
		"r7 = *(u32 *)(r6 + %[__sk_buff_data]);"
		"r8 = *(u32 *)(r6 + %[__sk_buff_data_end]);"
		/* check pkt has at least 1 byte */
		"r0 = r7;"
		"r0 += 8;"
		"if r0 > r8 goto l0_%=;"
		/* Store valid pkt pointer to outgoing arg6 slot */
		"*(u64 *)(r11 - 8) = r7;"
		/* bpf_skb_pull_data invalidates all pkt pointers */
		"r1 = r6;"
		"r2 = 0;"
		"call %[bpf_skb_pull_data];"
		/* Call subprog that dereferences arg6 — should fail */
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call subprog_deref_arg6;"
	"l0_%=:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_skb_pull_data),
		  __imm_const(__sk_buff_data, offsetof(struct __sk_buff, data)),
		  __imm_const(__sk_buff_data_end, offsetof(struct __sk_buff, data_end))
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: null propagation rejects deref on null branch")
__failure
__msg("R{{[0-9]}} invalid mem access 'scalar'")
__naked void stack_arg_null_propagation_fail(void)
{
	asm volatile (
		"r1 = 0;"
		"*(u64 *)(r10 - 8) = r1;"
		/* r0 = bpf_map_lookup_elem(&map_hash_8b, &key) */
		"r2 = r10;"
		"r2 += -8;"
		"r1 = %[map_hash_8b] ll;"
		"call %[bpf_map_lookup_elem];"
		/* Store PTR_TO_MAP_VALUE_OR_NULL to outgoing arg6 slot */
		"*(u64 *)(r11 - 8) = r0;"
		/* null check on r0 */
		"if r0 != 0 goto l0_%=;"
		/*
		 * On null branch, outgoing slot is SCALAR(0).
		 * Call subprog that dereferences arg6 — should fail.
		 */
		"r1 = 0;"
		"r2 = 0;"
		"r3 = 0;"
		"r4 = 0;"
		"r5 = 0;"
		"call subprog_deref_arg6;"
	"l0_%=:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_map_lookup_elem),
		  __imm_addr(map_hash_8b)
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: missing store on one branch")
__failure
__msg("callee expects 7 args, stack arg1 is not initialized")
__naked void stack_arg_missing_store_one_branch(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		/* Write arg7 (r11-16) before branch */
		"*(u64 *)(r11 - 16) = 20;"
		"if r0 > 0 goto l0_%=;"
		/* Path 1: write arg6 and call */
		"*(u64 *)(r11 - 8) = 10;"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call subprog_7args;"
		"goto l1_%=;"
	"l0_%=:"
		/* Path 2: missing arg6 store, call should fail */
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call subprog_7args;"
	"l1_%=:"
		"r0 = 0;"
		"exit;"
		:: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: share a store for both branches")
__success __retval(0)
__naked void stack_arg_shared_store(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		/* Write arg7 (r11-16) before branch */
		"*(u64 *)(r11 - 16) = 20;"
		"if r0 > 0 goto l0_%=;"
		/* Path 1: write arg6 and call */
		"*(u64 *)(r11 - 8) = 10;"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call subprog_7args;"
		"goto l1_%=;"
	"l0_%=:"
		/* Path 2: also write arg6 and call */
		"*(u64 *)(r11 - 8) = 30;"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call subprog_7args;"
	"l1_%=:"
		"r0 = 0;"
		"exit;"
		:: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

/* What it stores is what it got: the path that stores nothing looks the same at the call. */
static __naked __noinline __used long stores_same_on_one_path(long a, long b, long c, long d,
							      long e, long f, long g)
{
	asm volatile (
	"r6 = *(u64 *)(r11 + 8);"
	"r7 = *(u64 *)(r11 + 16);"
	"if r1 == 0 goto l0_%=;"
	"*(u64 *)(r11 - 8) = r6;"
	"*(u64 *)(r11 - 16) = r7;"
"l0_%=:"
	"r1 = 0;"
	"r2 = 0;"
	"r3 = 0;"
	"r4 = 0;"
	"r5 = 0;"
	"call subprog_7args;"
	"exit;"
	::: __clobber_all);
}

SEC("tc")
__description("stack_arg: incoming args are not taken for outgoing args, same values")
__failure __msg("callee expects 7 args, stack arg1 is not initialized")
__naked void incoming_for_outgoing_same(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r1 = r0;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 8) = 6;"
	"*(u64 *)(r11 - 16) = 7;"
	"call stores_same_on_one_path;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

/* r1 is 6 .. 9 and r2 is r1 + 4, that is 10 .. 13: r9, which is not set, is not read */
static __naked __noinline __used long sixth_plus_4(long a, long b, long c, long d, long e, long f)
{
	asm volatile (
	"r1 = *(u64 *)(r11 + 8);"
	"r2 = r1;"
	"r2 += 4;"
	"r0 = 0;"
	"if r1 < 6 goto l0_%=;"
	"if r1 > 9 goto l0_%=;"
	"if r2 > 9 goto l0_%=;"
	"r0 = r9;"
"l0_%=:"
	"exit;"
	::: __clobber_all);
}

/* For the caller the argument is r6 + 10. The subprog has no r6. */
SEC("tc")
__description("stack_arg: the argument is linked to a register of the caller")
__success
__naked void stack_arg_is_linked(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r6 = r0;"
	"r1 = r6;"
	"r1 += 10;"
	"*(u64 *)(r11 - 8) = r1;"
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"call sixth_plus_4;"
	"r0 += r6;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

static __naked __noinline __used long sixth_is_there(long a, long b, long c, long d, long e,
						     long *f)
{
	asm volatile (
	"r0 = *(u64 *)(r11 + 8);"
	"if r0 == 0 goto l0_%=;"
	"r0 = 1;"
"l0_%=:"
	"exit;"
	::: __clobber_all);
}

SEC("tc")
__description("stack_arg: subprog does the NULL check for its caller")
__success
__naked void null_check_of_stack_arg(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = r10;"
	"r2 += -8;"
	"r1 = %[map_hash_8b] ll;"
	"call %[bpf_map_lookup_elem];"
	"r6 = r0;"
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 8) = r6;"
	"call sixth_is_there;"
	"if r0 == 0 goto l0_%=;"
	"r0 = *(u64 *)(r6 + 0);"
"l0_%=:"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_map_lookup_elem),
	  __imm_addr(map_hash_8b)
	: __clobber_all);
}

/* r1 - r5 and all 19 slots. Its prototype has no parameters: what it takes is seen in its code. */
__naked __noinline __used
static unsigned long subprog_24args(void)
{
	asm volatile (
		"r0 = r1;"
		"r0 += r2;"
		"r0 += r3;"
		"r0 += r4;"
		"r0 += r5;"
		"r1 = *(u64 *)(r11 + 8);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 16);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 24);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 32);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 40);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 48);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 56);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 64);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 72);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 80);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 88);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 96);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 104);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 112);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 120);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 128);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 136);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 144);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 152);"
		"r0 += r1;"
		"exit;"
		::: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: static subprog with 24 args")
__success __retval(300)
__naked void stack_arg_24args(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 152) = 24;"
		"*(u64 *)(r11 - 144) = 23;"
		"*(u64 *)(r11 - 136) = 22;"
		"*(u64 *)(r11 - 128) = 21;"
		"*(u64 *)(r11 - 120) = 20;"
		"*(u64 *)(r11 - 112) = 19;"
		"*(u64 *)(r11 - 104) = 18;"
		"*(u64 *)(r11 - 96) = 17;"
		"*(u64 *)(r11 - 88) = 16;"
		"*(u64 *)(r11 - 80) = 15;"
		"*(u64 *)(r11 - 72) = 14;"
		"*(u64 *)(r11 - 64) = 13;"
		"*(u64 *)(r11 - 56) = 12;"
		"*(u64 *)(r11 - 48) = 11;"
		"*(u64 *)(r11 - 40) = 10;"
		"*(u64 *)(r11 - 32) = 9;"
		"*(u64 *)(r11 - 24) = 8;"
		"*(u64 *)(r11 - 16) = 7;"
		"*(u64 *)(r11 - 8) = 6;"
		"call subprog_24args;"
		"exit;"
		::: __clobber_all
	);
}

__naked __noinline __used
static unsigned long returns_arg22(void)
{
	asm volatile (
		"r0 = *(u64 *)(r11 + 136);"
		"exit;"
		::: __clobber_all
	);
}

/* A dynptr and a slice of it, which has two ids: its own and the one of the dynptr */
#define SLICE(dynptr, slice)			\
	"r1 = r6;"				\
	"r2 = 8;"				\
	"r3 = 0;"				\
	"r4 = r10;"				\
	"r4 += -" #dynptr ";"			\
	"call %[bpf_dynptr_from_mem];"		\
	"r1 = r10;"				\
	"r1 += -" #dynptr ";"			\
	"r2 = 0;"				\
	"r3 = 8;"				\
	"call %[bpf_dynptr_data];"		\
	"*(u64 *)(r10 - " #slice ") = r0;"

#define SLICE_ARG(slice, arg)			\
	"r1 = *(u64 *)(r10 - " #slice ");"	\
	"*(u64 *)(r11 - " #arg ") = r1;"

/*
 * The 16 slices in front of it have 32 ids. What the subprog returns is still the slice of
 * the dynptr that is submitted.
 */
SEC("tc")
__description("stack_arg: ids of the last of 17 slices")
__failure __msg("R6 invalid mem access 'scalar'")
__naked void stack_arg_ids_of_17_slices(void)
{
	asm volatile (
		"r1 = 0;"
		"*(u64 *)(r10 - 408) = r1;"
		"r2 = r10;"
		"r2 += -408;"
		"r1 = %[map_hash_8b] ll;"
		"call %[bpf_map_lookup_elem];"
		"if r0 == 0 goto 2f;"
		"r6 = r0;"
		SLICE(16, 280)
		SLICE(32, 288)
		SLICE(48, 296)
		SLICE(64, 304)
		SLICE(80, 312)
		SLICE(96, 320)
		SLICE(112, 328)
		SLICE(128, 336)
		SLICE(144, 344)
		SLICE(160, 352)
		SLICE(176, 360)
		SLICE(192, 368)
		SLICE(208, 376)
		SLICE(224, 384)
		SLICE(240, 392)
		SLICE(256, 400)
		"r1 = %[ringbuf] ll;"
		"r2 = 8;"
		"r3 = 0;"
		"r4 = r10;"
		"r4 += -272;"
		"call %[bpf_ringbuf_reserve_dynptr];"
		"r1 = r10;"
		"r1 += -272;"
		"r2 = 0;"
		"r3 = 8;"
		"call %[bpf_dynptr_data];"
		"if r0 == 0 goto 1f;"
		"*(u64 *)(r11 - 136) = r0;"
		SLICE_ARG(400, 128)
		SLICE_ARG(392, 120)
		SLICE_ARG(384, 112)
		SLICE_ARG(376, 104)
		SLICE_ARG(368, 96)
		SLICE_ARG(360, 88)
		SLICE_ARG(352, 80)
		SLICE_ARG(344, 72)
		SLICE_ARG(336, 64)
		SLICE_ARG(328, 56)
		SLICE_ARG(320, 48)
		SLICE_ARG(312, 40)
		SLICE_ARG(304, 32)
		SLICE_ARG(296, 24)
		SLICE_ARG(288, 16)
		SLICE_ARG(280, 8)
		"call returns_arg22;"
		"r6 = r0;"
		"r1 = r10;"
		"r1 += -272;"
		"r2 = 0;"
		"call %[bpf_ringbuf_submit_dynptr];"
		"r1 = 0;"
		"*(u64 *)(r6 + 0) = r1;"
		"r0 = 0;"
		"exit;"
	"1:"
		"r1 = r10;"
		"r1 += -272;"
		"r2 = 0;"
		"call %[bpf_ringbuf_discard_dynptr];"
	"2:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_map_lookup_elem),
		  __imm(bpf_dynptr_from_mem),
		  __imm(bpf_dynptr_data),
		  __imm(bpf_ringbuf_reserve_dynptr),
		  __imm(bpf_ringbuf_submit_dynptr),
		  __imm(bpf_ringbuf_discard_dynptr),
		  __imm_addr(map_hash_8b),
		  __imm_addr(ringbuf)
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: write beyond max outgoing depth")
__failure
__msg("stack arg write offset -160 exceeds max 19 stack args")
__naked void stack_arg_write_beyond_max(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		/* Write to offset -160, way beyond any callee's needs */
		"*(u64 *)(r11 - 160) = 99;"
		"*(u64 *)(r11 - 16) = 20;"
		"*(u64 *)(r11 - 8) = 10;"
		"call subprog_7args;"
		"r0 = 0;"
		"exit;"
		::: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: write unused stack arg slot")
__failure
__msg("func#0 writes 5 stack arg slots, but calls only require 2")
__naked void stack_arg_write_unused_slot(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		/* Write to offset -40, unused for the callee */
		"*(u64 *)(r11 - 40) = 99;"
		"*(u64 *)(r11 - 16) = 20;"
		"*(u64 *)(r11 - 8) = 10;"
		"call subprog_7args;"
		"r0 = 0;"
		"exit;"
		::: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: write the slots of the prototype, the callee loads none")
__success __retval(15)
__load_if_JITed()
__naked void stack_arg_write_prototype_slots(void)
{
	asm volatile (CALL_TAKES_5_SAYS_7("*(u64 *)(r11 - 8) = 6; *(u64 *)(r11 - 16) = 7;")
		      ::: __clobber_all);
}

SEC("tc")
__description("stack_arg: sequential calls reuse slots")
__failure
__msg("callee expects 7 args, stack arg1 is not initialized")
__naked void stack_arg_sequential_calls(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 8) = 6;"
		"*(u64 *)(r11 - 16) = 7;"
		"call subprog_7args;"
		"r6 = r0;"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call subprog_7args;"
		"r0 += r6;"
		"exit;"
		::: __clobber_all
	);
}

/*
 * A JIT passes the first stack arguments in registers. may_goto is not a call: they must be
 * the same after it, also when it looks at the clock. That is once in 65535 iterations.
 */
__naked __noinline __used
static long loops_then_reads_8args(long a, long b, long c, long d, long e, long f, long g, long h)
{
	asm volatile (
		"r1 = 0;"
	"1:"
		"may_goto 2f;"
		"r1 += 1;"
		"if r1 < 70000 goto 1b;"
	"2:"
		"r0 = *(u64 *)(r11 + 8);"
		"r1 = *(u64 *)(r11 + 16);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 24);"
		"r0 += r1;"
		"exit;"
	);
}

SEC("tc")
__description("stack_arg: may_goto looks at the clock, then the callee reads its args")
__success __retval(876)
__naked void stack_arg_may_goto_in_callee(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 24) = 800;"
		"*(u64 *)(r11 - 16) = 70;"
		"*(u64 *)(r11 - 8) = 6;"
		"call loops_then_reads_8args;"
		"exit;"
		::: __clobber_all
	);
}

__naked __noinline __used
static long spins_then_reads_8args(long a, long b, long c, long d, long e, long f, long g, long h)
{
	asm volatile (
	"1:"
		"may_goto 2f;"
		"goto 1b;"
	"2:"
		"r0 = *(u64 *)(r11 + 8);"
		"r1 = *(u64 *)(r11 + 16);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 24);"
		"r0 += r1;"
		"exit;"
	);
}

SEC("tc")
__description("stack_arg: the time of may_goto is up, then the callee reads its args")
__success __retval(876)
__naked void stack_arg_may_goto_timeout_in_callee(void)
{
	asm volatile (
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"*(u64 *)(r11 - 24) = 800;"
		"*(u64 *)(r11 - 16) = 70;"
		"*(u64 *)(r11 - 8) = 6;"
		"call spins_then_reads_8args;"
		"exit;"
		::: __clobber_all
	);
}

__naked __noinline __used
static long reads_8args(long a, long b, long c, long d, long e, long f, long g, long h)
{
	asm volatile (
		"r0 = *(u64 *)(r11 + 8);"
		"r1 = *(u64 *)(r11 + 16);"
		"r0 += r1;"
		"r1 = *(u64 *)(r11 + 24);"
		"r0 += r1;"
		"exit;"
	);
}

SEC("tc")
__description("stack_arg: may_goto looks at the clock between the stores of the args and the call")
__success __retval(876)
__naked void stack_arg_may_goto_in_caller(void)
{
	asm volatile (
		"*(u64 *)(r11 - 24) = 800;"
		"*(u64 *)(r11 - 16) = 70;"
		"*(u64 *)(r11 - 8) = 6;"
		"r6 = 0;"
	"1:"
		"may_goto 2f;"
		"r6 += 1;"
		"if r6 < 70000 goto 1b;"
	"2:"
		"r1 = 1;"
		"r2 = 2;"
		"r3 = 3;"
		"r4 = 4;"
		"r5 = 5;"
		"call reads_8args;"
		"exit;"
		::: __clobber_all
	);
}

/* arg6 is an offset into the value of map_hash_8b. 0-7 are in bounds. */
__naked __noinline __used
static long arg6_is_map_value_off(long a, long b, long c, long d, long e, long off)
{
	asm volatile (
		"r6 = *(u64 *)(r11 + 8);"
		"r1 = 0;"
		"*(u64 *)(r10 - 8) = r1;"
		"r2 = r10;"
		"r2 += -8;"
		"r1 = %[map_hash_8b] ll;"
		"call %[bpf_map_lookup_elem];"
		"if r0 == 0 goto l0_%=;"
		"r0 += r6;"
		"r0 = *(u8 *)(r0 + 0);"
	"l0_%=:"
		"exit;"
		:
		: __imm(bpf_map_lookup_elem),
		  __imm_addr(map_hash_8b)
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: a pruned state propagates precision of arg6")
__failure __msg("invalid access to map value")
__flag(BPF_F_TEST_STATE_FREQ)
__naked void stack_arg_precision_pruned_state(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"if r0 != 0 goto l0_%=;"
		/* Path 1: arg6 = 0. It makes arg6 precise at l3. */
		"*(u64 *)(r11 - 8) = 0;"
		"goto l3_%=;"
	"l0_%=:"
		"if r0 == 1 goto l1_%=;"
		/* Path 2: arg6 = r6 = 0. Pruned at l3. It has to make r6 precise at l2. */
		"r6 = 0;"
		"goto l2_%=;"
	"l1_%=:"
		/* Path 3: arg6 = r6 = 8. Out of bounds. */
		"r6 = 8;"
	"l2_%=:"
		"*(u64 *)(r11 - 8) = r6;"
	"l3_%=:"
		"r1 = 0;"
		"r2 = 0;"
		"r3 = 0;"
		"r4 = 0;"
		"r5 = 0;"
		"call arg6_is_map_value_off;"
		"exit;"
		:: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

/* The same, but backtracking of r1 sees a store through r1 and marks all scalars precise. */
__naked __noinline __used
static long arg6_is_map_value_off_fallback(long a, long b, long c, long d, long e, long off)
{
	asm volatile (
		"r6 = *(u64 *)(r11 + 8);"
		"r1 = 0;"
		"*(u64 *)(r10 - 8) = r1;"
		"r2 = r10;"
		"r2 += -8;"
		"r1 = %[map_hash_8b] ll;"
		"call %[bpf_map_lookup_elem];"
		"if r0 == 0 goto l0_%=;"
		"r1 = r0;"
		"*(u8 *)(r1 + 0) = 0;"
		"r1 -= r0;"
		"r1 &= 0;"
		"r1 += r6;"
		"r0 += r1;"
		"r0 = *(u8 *)(r0 + 0);"
	"l0_%=:"
		"exit;"
		:
		: __imm(bpf_map_lookup_elem),
		  __imm_addr(map_hash_8b)
		: __clobber_all
	);
}

SEC("tc")
__description("stack_arg: forcing all scalars precise covers arg6")
__failure __msg("invalid access to map value")
__flag(BPF_F_TEST_STATE_FREQ)
__naked void stack_arg_precision_fallback(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"if r0 != 0 goto l0_%=;"
		"*(u64 *)(r11 - 8) = 0;"
		"goto l1_%=;"
	"l0_%=:"
		/* Out of bounds. Must not be pruned at l1. */
		"*(u64 *)(r11 - 8) = 8;"
	"l1_%=:"
		"r1 = 0;"
		"r2 = 0;"
		"r3 = 0;"
		"r4 = 0;"
		"r5 = 0;"
		"call arg6_is_map_value_off_fallback;"
		"exit;"
		:: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

#else

/* No BTF tells about this store. The insn is what JIT has to deal with. */
SEC("tc")
__description("stack_arg: r11 where JIT has no stack arguments")
__failure __msg("JIT does not support stack arguments")
__arch_s390x __arch_loongarch
__naked void r11_no_jit_support(void)
{
	asm volatile (".8byte %[st_r11]; r0 = 0; exit;"
		      :: __imm_insn(st_r11, BPF_ST_MEM(BPF_DW, 11, -8, 1)) : __clobber_all);
}

SEC("socket")
__description("stack_arg is not supported by compiler or jit, use a dummy test")
__skip("stack_arg is not supported by compiler or jit")
__success
int dummy_test(void)
{
	return 0;
}

#endif

char _license[] SEC("license") = "GPL";
