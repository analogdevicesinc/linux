// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

extern void bpf_throw(__u64 cookie) __ksym;

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 4096);
} arena SEC(".maps");

/* BTF FUNC records are not generated for kfuncs referenced from inline assembly */
void __kfunc_btf_root(void)
{
	bpf_throw(0);
}

/* Compilers don't know r12 yet */
#define R12 12
#define ADJ(n)		BPF_RAW_INSN(BPF_ALU64 | BPF_ADD | BPF_K, R12, 0, 0, n)
#define LDX(dst, off)	BPF_RAW_INSN(BPF_LDX | BPF_MEM | BPF_DW, dst, R12, off, 0)
#define STX(off, src)	BPF_RAW_INSN(BPF_STX | BPF_MEM | BPF_DW, R12, src, off, 0)
#define ST(off, imm)	BPF_RAW_INSN(BPF_ST | BPF_MEM | BPF_DW, R12, 0, off, imm)

/*
 * Fill the frame, spin long enough to be preempted, check the frame.
 * Returns the number of words that have changed.
 */
static __naked __noinline __used long spin(long seed)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[stx32];"
	"r1 += 1;"
	".8byte %[stx24];"
	"r1 += 1;"
	".8byte %[stx16];"
	"r1 += 1;"
	".8byte %[stx8];"
	"r2 = 3000000;"
"1:"
	"r2 += -1;"
	"if r2 != 0 goto 1b;"
	"r0 = 0;"
	".8byte %[ldx8];"
	"if r2 == r1 goto +1;"
	"r0 += 1;"
	"r1 -= 1;"
	".8byte %[ldx16];"
	"if r2 == r1 goto +1;"
	"r0 += 1;"
	"r1 -= 1;"
	".8byte %[ldx24];"
	"if r2 == r1 goto +1;"
	"r0 += 1;"
	"r1 -= 1;"
	".8byte %[ldx32];"
	"if r2 == r1 goto +1;"
	"r0 += 1;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(32)),
	  __imm_insn(stx32, STX(-32, BPF_REG_1)),
	  __imm_insn(stx24, STX(-24, BPF_REG_1)),
	  __imm_insn(stx16, STX(-16, BPF_REG_1)),
	  __imm_insn(stx8, STX(-8, BPF_REG_1)),
	  __imm_insn(ldx32, LDX(BPF_REG_2, -32)),
	  __imm_insn(ldx24, LDX(BPF_REG_2, -24)),
	  __imm_insn(ldx16, LDX(BPF_REG_2, -16)),
	  __imm_insn(ldx8, LDX(BPF_REG_2, -8))
	: __clobber_all);
}

/* the same, one frame deeper */
static __naked __noinline __used long spin_deeper(long seed)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[stx];"
	"r6 = r1;"
	"r1 += 100;"
	"call spin;"
	".8byte %[ldx];"
	"if r1 == r6 goto +1;"
	"r0 += 1;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(128)),
	  __imm_insn(stx, STX(-64, BPF_REG_1)),
	  __imm_insn(ldx, LDX(BPF_REG_1, -64))
	: __clobber_all);
}

SEC("syscall")
__naked int preempted(void)
{
	asm volatile (
	".8byte %[adj];"
	"call %[bpf_get_prandom_u32];"
	"r6 = r0;"
	".8byte %[stx];"
	"r1 = r6;"
	"if r6 & 1 goto 1f;"
	"call spin;"
	"goto 2f;"
"1:"
	"call spin_deeper;"
"2:"
	".8byte %[ldx];"
	"if r1 == r6 goto +1;"
	"r0 += 1;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(stx, STX(-16, BPF_REG_6)),
	  __imm_insn(ldx, LDX(BPF_REG_1, -16))
	: __clobber_all);
}

static long thrower(void);

/* The same with preemption disabled. Programs in interrupts nest. */
SEC("tc")
__naked int interrupted(void)
{
	asm volatile (
	".8byte %[adj];"
	"call %[bpf_get_prandom_u32];"
	"r6 = r0;"
	".8byte %[stx];"
	"r1 = r6;"
	"call spin_deeper;"
	".8byte %[ldx];"
	"if r1 == r6 goto +1;"
	"r0 += 1;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(stx, STX(-16, BPF_REG_6)),
	  __imm_insn(ldx, LDX(BPF_REG_1, -16))
	: __clobber_all);
}

long interrupts;

static __naked __noinline __used long scribble(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st128];"
	".8byte %[st64];"
	".8byte %[st8];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(128)),
	  __imm_insn(st128, ST(-128, -1)),
	  __imm_insn(st64, ST(-64, -1)),
	  __imm_insn(st8, ST(-8, -1))
	: __clobber_all);
}

SEC("perf_event")
__naked int in_interrupt(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st32];"
	".8byte %[st8];"
	"call scribble;"
	"r1 = %[interrupts] ll;"
	"r2 = 1;"
	"lock *(u64 *)(r1 + 0) += r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(interrupts),
	  __imm_insn(adj, ADJ(32)),
	  __imm_insn(st32, ST(-32, -1)),
	  __imm_insn(st8, ST(-8, -1))
	: __clobber_all);
}

SEC("tc")
__naked int throws_nested(void)
{
	asm volatile (
	".8byte %[adj];"
	"call thrower;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

SEC("tc")
__naked int returns_8_nested(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	".8byte %[ldx];"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(-16, 8)),
	  __imm_insn(ldx, LDX(BPF_REG_0, -16))
	: __clobber_all);
}

static __naked __noinline __used long thrower(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = 5;"
	"call %[bpf_throw];"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_throw),
	  __imm_insn(adj, ADJ(64))
	: __clobber_all);
}

SEC("syscall")
__naked int throws(void)
{
	asm volatile (
	".8byte %[adj];"
	"call thrower;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

static __naked __noinline __used long returns_7(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	".8byte %[ldx];"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(-16, 7)),
	  __imm_insn(ldx, LDX(BPF_REG_0, -16))
	: __clobber_all);
}

SEC("syscall")
__naked int prog_returns_7(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	".8byte %[ldx];"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(-16, 7)),
	  __imm_insn(ldx, LDX(BPF_REG_0, -16))
	: __clobber_all);
}

struct elem {
	struct bpf_timer t;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct elem);
} timers SEC(".maps");

long timer_runs, timer_bad;

/* The kernel calls it. Fill the frame, call a function that has a frame, check the frame. */
static __naked __noinline __used int timer_cb(void *map, int *key, struct elem *val)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st32];"
	".8byte %[st8];"
	"call returns_7;"
	".8byte %[ldx32];"
	".8byte %[ldx8];"
	"r1 += r2;"
	"r1 += r0;"
	"r2 = %[timer_runs] ll;"
	"r3 = 1;"
	"lock *(u64 *)(r2 + 0) += r3;"
	"if r1 == 49 goto 1f;"
	"r2 = %[timer_bad] ll;"
	"lock *(u64 *)(r2 + 0) += r3;"
"1:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(timer_runs),
	  __imm_addr(timer_bad),
	  __imm_insn(adj, ADJ(32)),
	  __imm_insn(st32, ST(-32, 11)),
	  __imm_insn(st8, ST(-8, 31)),
	  __imm_insn(ldx32, LDX(BPF_REG_1, -32)),
	  __imm_insn(ldx8, LDX(BPF_REG_2, -8))
	: __clobber_all);
}

SEC("syscall")
int start_timer(void *ctx)
{
	struct elem *e;
	int key = 0;

	/* the program has to have an arena */
	asm volatile ("" :: "r"(&arena));
	e = bpf_map_lookup_elem(&timers, &key);
	if (!e)
		return 2;
	bpf_timer_init(&e->t, &timers, 1 /* CLOCK_MONOTONIC */);
	if (bpf_timer_set_callback(&e->t, timer_cb))
		return 3;
	return bpf_timer_start(&e->t, 0, 0) ? 4 : 0;
}

char _license[] SEC("license") = "GPL";
