// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <limits.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"
#include "../../../include/linux/filter.h"

#define BPF_UHMUL64_REG(DST, SRC) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MUL | BPF_X, DST, SRC, BPF_UHMUL, 0)

#define BPF_UHMUL64_IMM(DST, IMM) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MUL | BPF_K, DST, 0, BPF_UHMUL, IMM)

#define BPF_SHMUL64_REG(DST, SRC) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MUL | BPF_X, DST, SRC, BPF_SHMUL, 0)

#define BPF_SHMUL64_IMM(DST, IMM) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MUL | BPF_K, DST, 0, BPF_SHMUL, IMM)

/*
 * Inject UHMUL/SHMUL as raw instructions so these tests do not require
 * assembler support for the new mnemonics. All surrounding instructions use
 * existing BPF assembly syntax supported by the baseline selftests toolchain.
 */

#if defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64)

/*
 * BPF_PROG_TEST_RUN reports a 32-bit retval. Fold both halves of
 * (actual ^ expected) into w0 so retval == 0 checks the full 64-bit result.
 */

SEC("socket")
__description("UHMUL64, U64_MAX * U64_MAX, register source")
__success __success_unpriv __retval(0)
__naked void uhmul64_max_reg(void)
{
	asm volatile (
	"r0 = -1;"
	"r1 = -1;"
	".8byte %[hmul];"
	"r2 = -2;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_insn(hmul, BPF_UHMUL64_REG(BPF_REG_0, BPF_REG_1))
	: __clobber_all);
}

SEC("socket")
__description("UHMUL64, (1ULL << 63) * 2, register source")
__success __success_unpriv __retval(0)
__naked void uhmul64_pow2_reg(void)
{
	asm volatile (
	"r0 = 2;"
	"r1 = %[llong_min] ll;"
	".8byte %[hmul];"
	"r0 = r1;"
	"r2 = 1;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_const(llong_min, LLONG_MIN),
	  __imm_insn(hmul, BPF_UHMUL64_REG(BPF_REG_1, BPF_REG_0))
	: __clobber_all);
}

SEC("socket")
__description("UHMUL64, (1ULL << 63) * 2, R3 destination")
__success __success_unpriv __retval(0)
__naked void uhmul64_r3_dst(void)
{
	asm volatile (
	"r3 = %[llong_min] ll;"
	"r0 = 2;"
	".8byte %[hmul];"
	"r2 = 1;"
	"r3 ^= r2;"
	"r2 = r3;"
	"r2 >>= 32;"
	"w3 |= w2;"
	"r0 = r3;"
	"exit;"
	:
	: __imm_const(llong_min, LLONG_MIN),
	  __imm_insn(hmul, BPF_UHMUL64_REG(BPF_REG_3, BPF_REG_0))
	: __clobber_all);
}

SEC("socket")
__description("UHMUL64, (1ULL << 63) * 2, immediate source")
__success __success_unpriv __retval(0)
__naked void uhmul64_pow2_imm(void)
{
	asm volatile (
	"r0 = %[llong_min] ll;"
	".8byte %[hmul];"
	"r2 = 1;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_const(llong_min, LLONG_MIN),
	  __imm_insn(hmul, BPF_UHMUL64_IMM(BPF_REG_0, 2))
	: __clobber_all);
}

SEC("socket")
__description("UHMUL64, (1ULL << 63) * -2, immediate source")
__success __success_unpriv __retval(0)
__naked void uhmul64_neg_imm(void)
{
	asm volatile (
	"r0 = %[llong_min] ll;"
	".8byte %[hmul];"
	"r2 = %[llong_max] ll;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_const(llong_min, LLONG_MIN),
	  __imm_const(llong_max, LLONG_MAX),
	  __imm_insn(hmul, BPF_UHMUL64_IMM(BPF_REG_0, -2))
	: __clobber_all);
}

SEC("socket")
__description("SHMUL64, INT64_MAX * INT64_MAX, register source")
__success __success_unpriv __retval(0)
__naked void shmul64_max_reg(void)
{
	asm volatile (
	"r0 = %[llong_max] ll;"
	"r1 = %[llong_max] ll;"
	".8byte %[hmul];"
	"r2 = %[expected] ll;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_const(expected, 0x3fffffffffffffffULL),
	  __imm_const(llong_max, LLONG_MAX),
	  __imm_insn(hmul, BPF_SHMUL64_REG(BPF_REG_0, BPF_REG_1))
	: __clobber_all);
}

SEC("socket")
__description("SHMUL64, INT64_MIN * -2, register source")
__success __success_unpriv __retval(0)
__naked void shmul64_min_neg2_reg(void)
{
	asm volatile (
	"r0 = %[llong_min] ll;"
	"r1 = -2;"
	".8byte %[hmul];"
	"r2 = 1;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_const(llong_min, LLONG_MIN),
	  __imm_insn(hmul, BPF_SHMUL64_REG(BPF_REG_0, BPF_REG_1))
	: __clobber_all);
}

SEC("socket")
__description("SHMUL64, 1 * -2, immediate source")
__success __success_unpriv __retval(0)
__naked void shmul64_neg_imm(void)
{
	asm volatile (
	"r0 = 1;"
	".8byte %[hmul];"
	"r2 = -1;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_insn(hmul, BPF_SHMUL64_IMM(BPF_REG_0, -2))
	: __clobber_all);
}

SEC("socket")
__description("SHMUL64, INT64_MIN * 2, immediate source")
__success __success_unpriv __retval(0)
__naked void shmul64_min_imm(void)
{
	asm volatile (
	"r0 = %[llong_min] ll;"
	".8byte %[hmul];"
	"r2 = -1;"
	"r0 ^= r2;"
	"r2 = r0;"
	"r2 >>= 32;"
	"w0 |= w2;"
	"exit;"
	:
	: __imm_const(llong_min, LLONG_MIN),
	  __imm_insn(hmul, BPF_SHMUL64_IMM(BPF_REG_0, 2))
	: __clobber_all);
}

#else

SEC("socket")
__description("UHMUL/SHMUL are not supported by this JIT, use a dummy test")
__skip("UHMUL/SHMUL are not supported by this jit")
__success
int dummy_test(void)
{
	return 0;
}

#endif

char _license[] SEC("license") = "GPL";
