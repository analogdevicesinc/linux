// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023 SUSE LLC */
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

SEC("?raw_tp")
__success __log_level(2)
__msg("mark_precise: frame0: regs=r2 stack= before 3: (bf) r1 = r10")
__msg("mark_precise: frame0: regs=r2 stack= before 2: (55) if r2 != 0xfffffff8 goto pc+2")
__msg("mark_precise: frame0: regs=r2 stack= before 1: (87) r2 = -r2")
__msg("mark_precise: frame0: regs=r2 stack= before 0: (b7) r2 = 8")
__naked int bpf_neg(void)
{
	asm volatile (
		"r2 = 8;"
		"r2 = -r2;"
		"if r2 != -8 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("mark_precise: frame0: regs=r2 stack= before 3: (bf) r1 = r10")
__msg("mark_precise: frame0: regs=r2 stack= before 2: (55) if r2 != 0x0 goto pc+2")
__msg("mark_precise: frame0: regs=r2 stack= before 1: (d4) r2 = le16 r2")
__msg("mark_precise: frame0: regs=r2 stack= before 0: (b7) r2 = 0")
__naked int bpf_end_to_le(void)
{
	asm volatile (
		"r2 = 0;"
		"r2 = le16 r2;"
		"if r2 != 0 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}


SEC("?raw_tp")
__success __log_level(2)
__msg("mark_precise: frame0: regs=r2 stack= before 3: (bf) r1 = r10")
__msg("mark_precise: frame0: regs=r2 stack= before 2: (55) if r2 != 0x0 goto pc+2")
__msg("mark_precise: frame0: regs=r2 stack= before 1: (dc) r2 = be16 r2")
__msg("mark_precise: frame0: regs=r2 stack= before 0: (b7) r2 = 0")
__naked int bpf_end_to_be(void)
{
	asm volatile (
		"r2 = 0;"
		"r2 = be16 r2;"
		"if r2 != 0 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}

#if (defined(__TARGET_ARCH_arm64) || defined(__TARGET_ARCH_x86) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64) || \
	defined(__TARGET_ARCH_arm) || defined(__TARGET_ARCH_s390)) && \
	__clang_major__ >= 18

SEC("?raw_tp")
__success __log_level(2)
__msg("mark_precise: frame0: regs=r2 stack= before 3: (bf) r1 = r10")
__msg("mark_precise: frame0: regs=r2 stack= before 2: (55) if r2 != 0x0 goto pc+2")
__msg("mark_precise: frame0: regs=r2 stack= before 1: (d7) r2 = bswap16 r2")
__msg("mark_precise: frame0: regs=r2 stack= before 0: (b7) r2 = 0")
__naked int bpf_end_bswap(void)
{
	asm volatile (
		"r2 = 0;"
		"r2 = bswap16 r2;"
		"if r2 != 0 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}

#endif /* v4 instruction */

SEC("?raw_tp")
__success __log_level(2)
/*
 * Without the bug fix there will be no history between "last_idx 3 first_idx 3"
 * and "parent state regs=" lines. "R0_w=6" parts are here to help anchor
 * expected log messages to the one specific mark_chain_precision operation.
 *
 * This is quite fragile: if verifier checkpointing heuristic changes, this
 * might need adjusting.
 */
__msg("2: (07) r0 += 1                       ; R0_w=6")
__msg("3: (35) if r0 >= 0xa goto pc+1")
__msg("mark_precise: frame0: last_idx 3 first_idx 3 subseq_idx -1")
__msg("mark_precise: frame0: regs=r0 stack= before 2: (07) r0 += 1")
__msg("mark_precise: frame0: regs=r0 stack= before 1: (07) r0 += 1")
__msg("mark_precise: frame0: regs=r0 stack= before 4: (05) goto pc-4")
__msg("mark_precise: frame0: regs=r0 stack= before 3: (35) if r0 >= 0xa goto pc+1")
__msg("mark_precise: frame0: parent state regs= stack=:  R0_rw=P4")
__msg("3: R0_w=6")
__naked int state_loop_first_last_equal(void)
{
	asm volatile (
		"r0 = 0;"
	"l0_%=:"
		"r0 += 1;"
		"r0 += 1;"
		/* every few iterations we'll have a checkpoint here with
		 * first_idx == last_idx, potentially confusing precision
		 * backtracking logic
		 */
		"if r0 >= 10 goto l1_%=;"	/* checkpoint + mark_precise */
		"goto l0_%=;"
	"l1_%=:"
		"exit;"
		::: __clobber_common
	);
}

char _license[] SEC("license") = "GPL";
