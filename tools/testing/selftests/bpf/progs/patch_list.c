// SPDX-License-Identifier: GPL-2.0
/* Programs for the tables that follow the insns which the verifier patches and removes */

#include <linux/bpf.h>
#include <stdbool.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

/*
 * MARK(n) is a line of source code with one insn, 'r0 = n', and no other insn
 * has n. The first mark of function fn_n is n. So user space can tell whether
 * the line info and the func info of the loaded program name the right insns.
 */
#define MARK(n) asm volatile ("r0 = " #n ";")
/* The line starts with a nop. When the nop goes away, 'r0 = n' gets the line. */
#define NOP_MARK(n) asm volatile ("goto +0; r0 = " #n ";")

/* gcc doesn't put the text of the lines into BTF */
#ifdef __clang__
const volatile bool has_line_text = true;
#else
const volatile bool has_line_text = false;
#endif

/* only dead code calls it */
__naked __noinline __used
static int fn_2001(void)
{
	MARK(2001);
	asm volatile ("exit;");
}

__naked __noinline __used
static int fn_3001(void)
{
	NOP_MARK(3001);
	MARK(3002);
	asm volatile ("exit;");
}

/* the setup of the may_goto count goes in front of it */
__naked __noinline __used
static int fn_4001(void)
{
	MARK(4001);
	asm volatile (".8byte %[may_goto];"
		      :: __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 1, 0)));
	MARK(4002);
	MARK(4003);
	asm volatile ("exit;");
}

SEC("socket")
__naked int fn_1001(void)
{
	MARK(1001);
	asm volatile ("if r0 != 0 goto +3;");
	MARK(1002);
	asm volatile ("call fn_2001;");
	MARK(1003);
	MARK(1004);
	/* the division gets three insns in front */
	asm volatile ("r1 = 1; r0 /= r1;");
	NOP_MARK(1005);
	/* the next insn has a line, so the line of the nop goes away */
	asm volatile ("goto +0;");
	MARK(1006);
	asm volatile ("call fn_3001;");
	MARK(1007);
	asm volatile ("call fn_4001;");
	MARK(1008);
	asm volatile ("exit;");
}

struct {
	__uint(type, BPF_MAP_TYPE_PROG_ARRAY);
	__uint(max_entries, 1);
	__uint(key_size, sizeof(__u32));
	__uint(value_size, sizeof(__u32));
} jmp_table SEC(".maps");

SEC("tc")
int poke_target(struct __sk_buff *skb)
{
	return 7;
}

__naked __noinline __used
static int poke_pad(void)
{
	asm volatile (
	"r0 = 0;"
	"exit;"
	);
}

__naked __noinline __used
static int poke_tail_call(void)
{
	asm volatile (
	"r2 = %[jmp_table] ll;"
	"r3 = 0;"
	"call %[bpf_tail_call];"
	"r0 = 1;"
	"exit;"
	:
	: __imm_addr(jmp_table),
	  __imm(bpf_tail_call)
	: __clobber_all);
}

/*
 * The divisions grow by more insns than there are between the tail call and
 * the start of its function: the index that the tail call had is in this
 * function afterwards.
 */
SEC("tc")
__naked int poke_entry(void)
{
	asm volatile (
	"r6 = r1;"
	"r1 = 1;"
	"r0 = 8;"
	"r0 /= r1;"
	"r0 /= r1;"
	"r0 /= r1;"
	"r0 /= r1;"
	"call poke_pad;"
	"r1 = r6;"
	"call poke_tail_call;"
	"exit;"
	::: __clobber_all);
}

char _license[] SEC("license") = "GPL";
