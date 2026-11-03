// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

extern int bpf_kfunc_implicit_arg(int a) __weak __ksym;
extern int bpf_kfunc_implicit_arg_impl(int a, struct bpf_prog_aux *aux) __weak __ksym; /* illegal */
extern int bpf_kfunc_implicit_arg_legacy(int a, int b) __weak __ksym;
extern int bpf_kfunc_implicit_arg_legacy_impl(int a, int b, struct bpf_prog_aux *aux) __weak __ksym;

char _license[] SEC("license") = "GPL";

SEC("syscall")
__retval(5)
int test_kfunc_implicit_arg(void *ctx)
{
	return bpf_kfunc_implicit_arg(5);
}

SEC("syscall")
__failure __msg("cannot find address for kernel function bpf_kfunc_implicit_arg_impl")
int test_kfunc_implicit_arg_impl_illegal(void *ctx)
{
	return bpf_kfunc_implicit_arg_impl(5, NULL);
}

SEC("syscall")
__log_level(2)
__msg(": .12....... (85) call bpf_kfunc_implicit_arg_legacy#")
__retval(7)
int test_kfunc_implicit_arg_legacy(void *ctx)
{
	return bpf_kfunc_implicit_arg_legacy(3, 4);
}

/* The prog passes all three arguments of this kfunc. */
SEC("syscall")
__log_level(2)
__msg(": .123...... (85) call bpf_kfunc_implicit_arg_legacy_impl#")
__retval(11)
int test_kfunc_implicit_arg_legacy_impl(void *ctx)
{
	return bpf_kfunc_implicit_arg_legacy_impl(5, 6, NULL);
}

/*
 * The verifier adds 'r2 = aux' in front of the call. A jump to the call has to
 * execute the new insn: the kfunc returns -EINVAL when aux is NULL.
 */
SEC("syscall")
__retval(5)
__naked void implicit_arg_jump_to_call(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r1 = 5;"
	"r2 = 0;"
	"if r0 != 0 goto 1f;"
	"r2 = 0;"
"1:"
	"call %[bpf_kfunc_implicit_arg];"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm(bpf_kfunc_implicit_arg)
	: __clobber_all);
}

/*
 * r0 and r1 are live at may_goto, r2 is the first register that is not. The
 * insns that the verifier adds for may_goto are free to use r2: 'r2 = aux'
 * comes behind them.
 */
SEC("syscall")
__retval(5)
__naked void implicit_arg_after_may_goto(void)
{
	asm volatile (
	"r6 = 1;"
"1:"
	"r0 = r6;"
	"r1 = r6;"
	"may_goto 2f;"
	"call %[bpf_kfunc_implicit_arg];"
	"if r0 != r6 goto 2f;"
	"r6 += 1;"
	"if r6 < 5 goto 1b;"
	"r0 = r6;"
"2:"
	"exit;"
	:
	: __imm(bpf_kfunc_implicit_arg)
	: __clobber_all);
}
