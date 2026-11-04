// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"
#include "../../../include/linux/filter.h"

#define BPF_UHMUL64_IMM(DST, IMM) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MUL | BPF_K, DST, 0, BPF_UHMUL, IMM)

#define BPF_SHMUL64_REG(DST, SRC) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MUL | BPF_X, DST, SRC, BPF_SHMUL, 0)

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 1);
} arena SEC(".maps");

SEC("syscall")
__description("UHMUL64 with arena pointer dst")
__failure __msg("UHMUL/SHMUL with arena pointers are not supported")
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__naked void uhmul64_arena_ptr(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r0 = 1;"
	"r0 = addr_space_cast(r0, 0, 1);"
	".8byte %[hmul];"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_insn(hmul, BPF_UHMUL64_IMM(BPF_REG_0, 2))
	: __clobber_all);
}

SEC("syscall")
__description("SHMUL64 with arena pointer src")
__failure __msg("UHMUL/SHMUL with arena pointers are not supported")
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__naked void shmul64_arena_ptr(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r1 = 1;"
	"r1 = addr_space_cast(r1, 0, 1);"
	"r0 = 2;"
	".8byte %[hmul];"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_insn(hmul, BPF_SHMUL64_REG(BPF_REG_0, BPF_REG_1))
	: __clobber_all);
}

char _license[] SEC("license") = "GPL";
