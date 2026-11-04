// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

/* A stack that is not returned is noticed soon */
struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 8);
} arena SEC(".maps");

static __naked __noinline __used long returns_7(void)
{
	asm volatile (
	".8byte %[adj];"	/* r12 += 16 */
	".8byte %[st];"		/* *(u64 *)(r12 - 8) = 7 */
	".8byte %[ldx];"	/* r0 = *(u64 *)(r12 - 8) */
	"exit;"
	:
	: __imm_insn(adj, BPF_RAW_INSN(BPF_ALU64 | BPF_ADD | BPF_K, 12, 0, 0, 16)),
	  __imm_insn(st, BPF_RAW_INSN(BPF_ST | BPF_MEM | BPF_DW, 12, 0, -8, 7)),
	  __imm_insn(ldx, BPF_RAW_INSN(BPF_LDX | BPF_MEM | BPF_DW, 0, 12, -8, 0))
	: __clobber_all);
}

/* r1 is ctx, then 0: one jump to the first insn of the program */
#define LOOP					\
"1:"						\
	"if r1 == 0 goto 2f;"			\
	"r1 = 0;"				\
	"goto 1b;"				\
"2:"						\
	"call returns_7;"			\
	"exit;"

SEC("tc")
__naked int loop(void)
{
	asm volatile (LOOP ::: __clobber_all);
}

/* the same after the nop is removed */
SEC("tc")
__naked int nop_loop(void)
{
	asm volatile ("goto +0;" LOOP ::: __clobber_all);
}

/* the same through a jump table */
SEC("tc")
__naked int gotox_loop(void)
{
	asm volatile (
	".pushsection .jumptables,\"\",@progbits;"
"jt_%=:"
	".quad first_%= - tc;"
	".size jt_%=, 8;"
	".global jt_%=;"
	".popsection;"
"first_%=:"
	"if r1 == 0 goto 2f;"
	"r1 = 0;"
	"r2 = jt_%= ll;"
	"r2 = *(u64 *)(r2 + 0);"
	".8byte %[gotox];"	/* gotox r2 */
"2:"
	"call returns_7;"
	"exit;"
	:
	: __imm_insn(gotox, BPF_RAW_INSN(BPF_JMP | BPF_JA | BPF_X, BPF_REG_2, 0, 0, 0))
	: __clobber_all);
}

char _license[] SEC("license") = "GPL";
