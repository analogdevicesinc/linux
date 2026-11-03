// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

/*
 * bpf_do_misc_fixups() guards each division by a register against division
 * by zero with a 4 insn patch. Jumps to a patched insn must land on the
 * first insn of its patch, other jumps must follow the insns they target.
 *
 * JITs that need explicit zero extensions, like s390x and riscv64, get one
 * more insn behind the 'w0 ^= w0' of that patch. The tests that compare such
 * code don't run there.
 */

SEC("raw_tp")
__description("patch list: forward jumps to and over patched insns")
__arch_x86_64
__arch_arm64
__success
__xlated("0: call")
__xlated("1: r1 = r0")
__xlated("2: r0 = 7")
__xlated("3: if r1 > 0x5 goto pc+1")
__xlated("4: r0 = 9")
__xlated("5: if r1 != 0x0 goto pc+2")
__xlated("6: w0 ^= w0")
__xlated("7: goto pc+1")
__xlated("8: r0 /= r1")
__xlated("9: if r0 > 0x3 goto pc+4")
__xlated("10: if r1 != 0x0 goto pc+2")
__xlated("11: w0 ^= w0")
__xlated("12: goto pc+1")
__xlated("13: r0 /= r1")
__xlated("14: exit")
__naked void patch_list_forward(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r1 = r0;"
	"r0 = 7;"
	"if r1 > 5 goto l0_%=;"
	"r0 = 9;"
"l0_%=:"
	"r0 /= r1;"
	"if r0 > 3 goto l1_%=;"
	"r0 /= r1;"
"l1_%=:"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("raw_tp")
__description("patch list: backward jump to a patched insn")
__arch_x86_64
__arch_arm64
__success
__xlated("0: call")
__xlated("1: r1 = r0")
__xlated("2: r2 = 0")
__xlated("3: if r1 != 0x0 goto pc+2")
__xlated("4: w0 ^= w0")
__xlated("5: goto pc+1")
__xlated("6: r0 /= r1")
__xlated("7: r2 += 1")
__xlated("8: if r2 < 0x3 goto pc-6")
__xlated("9: exit")
__naked void patch_list_backward(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r1 = r0;"
	"r2 = 0;"
"l0_%=:"
	"r0 /= r1;"
	"r2 += 1;"
	"if r2 < 3 goto l0_%=;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

/* The jump of may_goto leaves its own patch and crosses another one. */
SEC("raw_tp")
__description("patch list: may_goto between patched insns")
__arch_x86_64
__arch_arm64
__success
__xlated("0: r0 = ")
__xlated("2: r0 = &(void __percpu *)(r0)")
__xlated("3: *(u64 *)(r0 +0) = 65535")
__xlated("...")
__xlated("6: call")
__xlated("7: r1 = r0")
__xlated("8: if r1 != 0x0 goto pc+2")
__xlated("9: w0 ^= w0")
__xlated("10: goto pc+1")
__xlated("11: r0 /= r1")
__xlated("12: r2 = ")
__xlated("14: r2 = &(void __percpu *)(r2)")
__xlated("15: r12 = *(u64 *)(r2 +0)")
__xlated("16: if r12 == 0x0 goto pc+9")
__xlated("...")
__xlated("22: if r1 != 0x0 goto pc+2")
__xlated("23: w0 ^= w0")
__xlated("24: goto pc+1")
__xlated("25: r0 /= r1")
__xlated("26: exit")
__naked void patch_list_may_goto(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r1 = r0;"
	"r0 /= r1;"
	".8byte %[may_goto];"
	"r0 /= r1;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 1 /* offset */, 0))
	: __clobber_all);
}

/*
 * The conditional jump skips 8200 divisions, which their patches turn into
 * 32800 insns, too far for its 16-bit offset.
 */
SEC("raw_tp")
__description("patch list: jump pushed out of range by patches")
__failure __msg("cannot be patched due to 16-bit range")
__naked void patch_list_jump_out_of_range(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r1 = r0;					\
	if r0 == 0 goto 0f;				\
	.rept 8200;					\
	r0 /= r1;					\
	.endr;						\
0:	r0 = 0;						\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

/* the JITs that support gotox, as in verifier_gotox.c */
#if defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64) || \
	defined(__TARGET_ARCH_powerpc) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64)
/*
 * An indirect jump to a patched insn must land on the first insn of its
 * patch. The load of pkt_type, which is 0, becomes a load and a mask. Behind
 * the load r0 still has bits of 255, and the result isn't 7.
 *
 * mark is 0, which the verifier doesn't know. So the division stays, never
 * runs, and its patch moves the target by 3 insns. The old index of the target
 * is the 'goto +1' of that patch, which goes to the exit with 255 in r0.
 */
SEC("socket")
__description("patch list: indirect jump to a patched insn")
__success __retval(7)
__load_if_JITed()
__naked void patch_list_indirect(void)
{
	asm volatile ("					\
	.pushsection .jumptables,\"\",@progbits;	\
jt0_%=:							\
	.quad l1_%= - socket;				\
	.quad l0_%= - socket;				\
	.size jt0_%=, 16;				\
	.global jt0_%=;					\
	.popsection;					\
	r2 = *(u32 *)(r1 + %[mark]);			\
	r2 &= 8;					\
	r3 = jt0_%= ll;					\
	r3 += r2;					\
	r3 = *(u64 *)(r3 + 0);				\
	r0 = 255;					\
	.8byte %[gotox_r3];				\
l0_%=:							\
	r0 /= r2;					\
	exit;						\
l1_%=:							\
	r0 = *(u32 *)(r1 + %[pkt_type]);		\
	r0 += 7;					\
	exit;						\
"	:
	: __imm_const(mark, offsetof(struct __sk_buff, mark)),
	  __imm_const(pkt_type, offsetof(struct __sk_buff, pkt_type)),
	  __imm_insn(gotox_r3, BPF_RAW_INSN(BPF_JMP | BPF_JA | BPF_X, BPF_REG_3, 0, 0, 0))
	: __clobber_all);
}
#else
SEC("socket")
__description("patch list: indirect jump to a patched insn")
__skip("the JIT does not support gotox")
__success
int patch_list_indirect(void)
{
	return 0;
}
#endif

/*
 * may_goto jumps back by 8195 insns, by 32768 once each of the 8191
 * divisions is guarded by a patch of 4 insns. That still fits the 16 bits of
 * may_goto, but not the jump it expands to, which has to become gotol. Once
 * the budget runs out, may_goto jumps back with r6 set and the program exits.
 */
SEC("raw_tp")
__description("patch list: backward may_goto expanded to gotol")
__arch_x86_64
__arch_arm64
__success
__xlated("12: if r6 != 0x0 goto pc-3")
__xlated("...")
__xlated("32783: if r12 != 0x0 goto pc+1")
__xlated("32784: gotol pc-32773")
__naked void patch_list_may_goto_gotol(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r1 = r0;					\
	r6 = 0;						\
	goto 0f;					\
1:	r0 = 0;						\
	exit;						\
0:	if r6 != 0 goto 1b;				\
	.rept 8191;					\
	r0 /= r1;					\
	.endr;						\
	r2 = 0;						\
	r6 = 1;						\
	.8byte %[may_goto];				\
	r0 = 0;						\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, -8195 /* offset */, 0))
	: __clobber_all);
}

/*
 * The aux data follows its insn: the verifier still knows which insn loads
 * from the ctx when the dead code and the nops in front of it are gone.
 */
SEC("socket")
__description("patch list: ctx access behind removed insns")
__success __retval(50) /* TEST_DATA_LEN without the Ethernet header */
__naked void patch_list_ctx_behind_removed(void)
{
	asm volatile (
	"r0 = 0;"
	"if r0 == 0 goto l0_%=;"
	"r0 = 1;"
	"r0 = 2;"
"l0_%=:"
	"goto +0;"
	"r0 = *(u32 *)(r1 + %[len]);"
	"exit;"
	:
	: __imm_const(len, offsetof(struct __sk_buff, len))
	: __clobber_all);
}

__naked __noinline __used
static unsigned long patch_list_loop_at_start(void)
{
	asm volatile (
"l0_%=:"
	"r1 += -1;"
	".8byte %[may_goto];"
	"if r1 != 0 goto l0_%=;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 1 /* offset */, 0))
	: __clobber_all);
}

/*
 * The check of the may_goto count goes in front of the callback of bpf_loop().
 * The call of the inlined bpf_loop() lands on the check, the jump to the first
 * insn of the callback behind it.
 * This is the code for the JITs that support the timed may_goto.
 */
SEC("raw_tp")
__description("patch list: call of and jump to an insn with insns inserted in front")
__arch_x86_64
__arch_s390x
__arch_arm64
__arch_riscv64
__arch_loongarch
__success
__xlated("23: call pc+7")
__xlated("...")
__xlated("30: exit")
__xlated("31: r0 = ")
__xlated("33: r0 = &(void __percpu *)(r0)")
__xlated("34: r0 = *(u64 *)(r0 +0)")
__xlated("35: if r0 != 0x0 goto pc+2")
__xlated("36: r0 = 1")
__xlated("37: exit")
__xlated("38: r1 += -1")
__xlated("...")
__xlated("49: if r1 != 0x0 goto pc-12")
__xlated("50: r0 = 0")
__xlated("51: exit")
__naked void patch_list_insert_in_front(void)
{
	asm volatile (
	"r1 = 3;"
	"r2 = %[patch_list_loop_at_start] ll;"
	"r3 = 0;"
	"r4 = 0;"
	"call %[bpf_loop];"
	"exit;"
	:
	: __imm_addr(patch_list_loop_at_start),
	  __imm(bpf_loop)
	: __clobber_all);
}

__naked __noinline __used
static unsigned long patch_list_never_called(void)
{
	asm volatile (
	"r0 = 100;"
	"exit;"
	);
}

__naked __noinline __used
static unsigned long patch_list_nop_add1(void)
{
	asm volatile (
	"goto +0;"
	"r0 = r1;"
	"r0 += 1;"
	"exit;"
	);
}

/*
 * A function goes away as dead code, the one behind it loses its first insn,
 * and the caller both loses and gets insns in front of the call.
 */
SEC("socket")
__description("patch list: call over removed and patched insns")
__success __retval(11)
__naked void patch_list_call_over_removed(void)
{
	asm volatile (
	"r1 = 1;"
	"if r1 == 1 goto l0_%=;"
	"call patch_list_never_called;"
"l0_%=:"
	"goto +0;"
	"r2 = 2;"
	"r0 = 12;"
	"r0 /= r2;"
	"r6 = r0;"
	"r1 = 4;"
	"call patch_list_nop_add1;"
	"r0 += r6;"
	"exit;"
	::: __clobber_all);
}

/*
 * A goto or a may_goto that jumps over nops only is a nop. The nops are not
 * dead code: may_goto falls through to its nop, 'if' jumps to the other one.
 */
SEC("raw_tp")
__description("patch list: may_goto over a nop")
__success
__xlated("0: r0 = 1")
__xlated("1: exit")
__naked void patch_list_may_goto_over_nop(void)
{
	asm volatile (
	".8byte %[may_goto];"
	"goto +0;"
	"r0 = 1;"
	"exit;"
	:
	: __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 1 /* offset */, 0))
	: __clobber_all);
}

SEC("raw_tp")
__description("patch list: goto over a nop")
__success
__xlated("0: r0 = *(u64 *)(r1 +0)")
__xlated("1: if r0 == 0x0 goto pc+0")
__xlated("2: r0 = 1")
__xlated("3: exit")
__naked void patch_list_goto_over_nop(void)
{
	asm volatile (
	"r0 = *(u64 *)(r1 + 0);"
	"if r0 == 0 goto l0_%=;"
	"goto l1_%=;"
"l0_%=:"
	"goto +0;"
"l1_%=:"
	"r0 = 1;"
	"exit;"
	::: __clobber_all);
}

/* this goto also jumps over an insn that stays */
SEC("raw_tp")
__description("patch list: goto over a nop and an insn")
__success
__xlated("0: r0 = *(u64 *)(r1 +0)")
__xlated("1: if r0 == 0x0 goto pc+1")
__xlated("2: goto pc+1")
__xlated("3: r0 = 1")
__xlated("4: exit")
__naked void patch_list_goto_over_nop_and_insn(void)
{
	asm volatile (
	"r0 = *(u64 *)(r1 + 0);"
	"if r0 == 0 goto l0_%=;"
	"goto l1_%=;"
"l0_%=:"
	"goto +0;"
	"r0 = 1;"
"l1_%=:"
	"exit;"
	::: __clobber_all);
}

char _license[] SEC("license") = "GPL";
