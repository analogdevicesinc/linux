// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, __u32);
} map_array SEC(".maps");

SEC("socket")
__description("invalid map type for tail call")
__failure __msg("expected prog array map for tail call")
__failure_unpriv
__naked void invalid_map_for_tail_call(void)
{
	asm volatile ("			\
	r2 = %[map_array] ll;	\
	r3 = 0;				\
	call %[bpf_tail_call];		\
	exit;				\
"	:
	: __imm(bpf_tail_call),
	  __imm_addr(map_array)
	: __clobber_all);
}

struct {
	__uint(type, BPF_MAP_TYPE_PROG_ARRAY);
	__uint(max_entries, 1);
	__uint(key_size, sizeof(__u32));
	__uint(value_size, sizeof(__u32));
} jmp_table SEC(".maps");

__used __naked
static int subprog_tail_call(void)
{
	asm volatile ("			\
	r2 = %[jmp_table] ll;		\
	r3 = 0;				\
	call %[bpf_tail_call];		\
	r0 = 0;				\
	exit;				\
"	:
	: __imm(bpf_tail_call),
	  __imm_addr(jmp_table)
	: __clobber_all);
}

/* The exception callback has no ctx, so its tail call can only be dead code */
__used __noinline
int exception_cb_tail_call(__u64 cookie)
{
	asm volatile ("			\
	r0 = 0;				\
	if r0 == 0 goto +4;		\
	r2 = %[jmp_table] ll;		\
	r3 = 0;				\
	call %[bpf_tail_call];		\
"	:
	: __imm(bpf_tail_call),
	  __imm_addr(jmp_table)
	: __clobber_common);
	return 0;
}

SEC("tc")
__description("tail call in an async callback that calls nothing")
__failure __msg("cannot tail call within callback")
__attribute__((btf_decl_tag("exception_callback:exception_cb_tail_call")))
__naked void tail_call_in_async_callback(void)
{
	asm volatile ("			\
	r0 = 0;				\
	exit;				\
"	::: __clobber_all);
}

#if (defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64)) && \
	defined(__BPF_FEATURE_STACK_ARGUMENT)

__used __noinline
static long subprog_6args(long a, long b, long c, long d, long e, long f)
{
	return f;
}

__used __naked
static int calls_subprog_tail_call(void)
{
	asm volatile ("			\
	call subprog_tail_call;		\
	exit;				\
"	::: __clobber_all);
}

SEC("tc")
__description("stack arguments two calls above a tail call")
__failure __msg("tail_calls are not allowed in programs with stack args")
__load_if_JITed()
__naked void stack_args_above_tail_call(void)
{
	asm volatile ("			\
	r6 = r1;			\
	*(u64 *)(r11 - 8) = 6;		\
	call subprog_6args;		\
	r1 = r6;			\
	call calls_subprog_tail_call;	\
	r0 = 0;				\
	exit;				\
"	::: __clobber_all);
}

#else

SEC("tc")
__description("stack arguments two calls above a tail call")
__skip("stack_arg is not supported by compiler or jit")
__success
int stack_args_above_tail_call(void)
{
	return 0;
}

#endif

/*
 * A tail call unwinds only the frame of the subprog doing it, so the
 * frames of its callers stay on the stack. With up to 33 tail calls in
 * a chain the verifier caps the stack those frames may add up to at
 * 256 bytes.
 */
SEC("tc")
__description("tail call from subprog with 240 bytes of caller stack")
__success
__naked void tail_call_caller_stack_ok(void)
{
	asm volatile ("			\
	r2 = 42;			\
	*(u64 *)(r10 - 240) = r2;	\
	call subprog_tail_call;		\
	r0 = 0;				\
	exit;				\
"	::: __clobber_all);
}

static __noinline int tail_calls(struct __sk_buff *skb)
{
	bpf_tail_call_static(skb, &jmp_table, 0);
	return skb->len;
}

static __noinline int calls_itself(struct __sk_buff *skb, int n)
{
	int ret = 0;

	if (n)
		ret = calls_itself(skb, n - 1);
	return ret + tail_calls(skb);
}

/* The frames of a subprog that calls itself stay on the stack too: up to a half of it. */
SEC("tc")
__description("tail call from subprog below a recursive subprog")
__arch_x86_64
__arch_arm64
__failure
__msg("tail_calls are not allowed when call stack of previous frames is {{[0-9]+}} bytes")
int tail_call_below_recursion(struct __sk_buff *skb)
{
	return calls_itself(skb, 2);
}

SEC("tc")
__description("tail call from subprog with 256 bytes of caller stack")
__failure
__msg("tail_calls are not allowed when call stack of previous frames is 256 bytes. Too large")
__naked void tail_call_caller_stack_too_large(void)
{
	asm volatile ("			\
	r2 = 42;			\
	*(u64 *)(r10 - 256) = r2;	\
	call subprog_tail_call;		\
	r0 = 0;				\
	exit;				\
"	::: __clobber_all);
}

char _license[] SEC("license") = "GPL";
