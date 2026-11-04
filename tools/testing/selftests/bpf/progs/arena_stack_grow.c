// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 4096);
} arena SEC(".maps");

/* Compilers don't know r12 yet */
#define R12 12
#define ADJ(n)		BPF_RAW_INSN(BPF_ALU64 | BPF_ADD | BPF_K, R12, 0, 0, n)
#define LDX(dst, off)	BPF_RAW_INSN(BPF_LDX | BPF_MEM | BPF_DW, dst, R12, off, 0)
#define ST(off, imm)	BPF_RAW_INSN(BPF_ST | BPF_MEM | BPF_DW, R12, 0, off, imm)

/* With 4K pages a program with such a frame needs a stack of two pages, the others need one */
#define BIG 4096

int nest, freed, waited;
long frame;

static __naked __noinline __used long small_cb(void)
{
	asm volatile (
	".8byte %[adj];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

/* The callback takes a stack of its own, the first one after the stack of the cpu */
SEC("syscall")
__naked int small(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = 1;"
	"r2 = small_cb ll;"
	"r3 = 0;"
	"r4 = 0;"
	"call %[bpf_loop];"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_loop),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

static __naked __noinline __used long inner_cb(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(-16, 2))
	: __clobber_all);
}

/* Write to the start of the stack, let inner_cb() do the same in its stack, read it back */
static __naked __noinline __used long outer_cb(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	"r1 = 1;"
	"r2 = inner_cb ll;"
	"r3 = 0;"
	"r4 = 0;"
	"call %[bpf_loop];"
	".8byte %[ldx];"
	"r2 = %[frame] ll;"
	"*(u64 *)(r2 + 0) = r1;"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_loop),
	  __imm_addr(frame),
	  __imm_insn(adj, ADJ(BIG)),
	  __imm_insn(st, ST(-BIG, 1)),
	  __imm_insn(ldx, LDX(BPF_REG_1, -BIG))
	: __clobber_all);
}

/* The stack that small_cb() has used is too small for outer_cb() */
SEC("syscall")
__naked int big(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = %[nest] ll;"
	"r1 = *(u32 *)(r1 + 0);"
	"if r1 == 0 goto 1f;"
	"r1 = 1;"
	"r2 = outer_cb ll;"
	"r3 = 0;"
	"r4 = 0;"
	"call %[bpf_loop];"
"1:"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_loop),
	  __imm_addr(nest),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

/* The pages of the old stack are free. This program gets them. */
SEC("fexit/arena_free_worker")
__naked int after_free(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = %[freed] ll;"
	"r2 = 1;"
	"*(u32 *)(r1 + 0) = r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(freed),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

static long is_freed(__u64 i, void *ctx)
{
	return *(volatile int *)&freed != 0;
}

/* The old pages are queued. Keep outer_cb() here until the worker has preempted it. */
SEC("fexit/arena_free_pages")
int wait_for_worker(void *ctx)
{
	if (nest && !bpf_get_smp_processor_id()) {
		waited = 1;
		bpf_loop(1 << 23, is_freed, NULL, 0);
	}
	return 0;
}

char _license[] SEC("license") = "GPL";
