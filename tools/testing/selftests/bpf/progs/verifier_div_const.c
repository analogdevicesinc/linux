// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

/* BPF_REG_AX as the verifier dumps it */
#define REG_AX "r13"

/* JITs with UHMUL and SHMUL get the multiplication by the magic number */
#if (defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64)) && \
	(__clang_major__ >= 18 || defined(__BPF_FEATURE_SDIV_SMOD))

SEC("socket")
__description("udiv by 1000000000 shifts the dividend first")
__success
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__xlated("...")
__xlated("1: r0 >>= 9")
__xlated("2: " REG_AX " = 0x44b82fa09b5a53")
__xlated("4: r0 uh*= " REG_AX)
__xlated("5: r0 >>= 11")
__xlated("6: exit")
__naked void udiv_shifted(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 /= 1000000000;				\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("udiv by 1000000")
__success
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__xlated("...")
__xlated("1: " REG_AX " = 0x431bde82d7b634db")
__xlated("3: r0 uh*= " REG_AX)
__xlated("4: r0 >>= 18")
__xlated("5: exit")
__naked void udiv(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 /= 1000000;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("udiv by 7 takes the add sequence")
__success
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__xlated("...")
__xlated("1: " REG_AX " = 0x2492492492492493")
__xlated("3: " REG_AX " uh*= r0")
__xlated("4: r0 -= " REG_AX)
__xlated("5: r0 >>= 1")
__xlated("6: r0 += " REG_AX)
__xlated("7: r0 >>= 2")
__xlated("8: exit")
__naked void udiv_add(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 /= 7;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("umod by 1000000")
__success
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__xlated("...")
__xlated("1: " REG_AX " = 0x431bde82d7b634db")
__xlated("3: " REG_AX " uh*= r0")
__xlated("4: " REG_AX " >>= 18")
__xlated("5: " REG_AX " *= 1000000")
__xlated("6: r0 -= " REG_AX)
__xlated("7: exit")
__naked void umod(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 %%= 1000000;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("umod by 1000 keeps the division")
__success
__xlated("...")
__xlated("1: r0 %= 1000")
__xlated("2: exit")
__naked void umod_kept(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 %%= 1000;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("sdiv by 10")
__success
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__xlated("...")
__xlated("1: " REG_AX " = 0x6666666666666667")
__xlated("3: " REG_AX " sh*= r0")
__xlated("4: " REG_AX " s>>= 2")
__xlated("5: r0 = " REG_AX)
__xlated("6: r0 >>= 63")
__xlated("7: r0 += " REG_AX)
__xlated("8: exit")
__naked void sdiv(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 s/= 10;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("sdiv by 2 adds the dividend")
__success
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__xlated("...")
__xlated("1: " REG_AX " = 0x8000000000000001")
__xlated("3: " REG_AX " sh*= r0")
__xlated("4: " REG_AX " += r0")
__xlated("5: r0 = " REG_AX)
__xlated("6: r0 >>= 63")
__xlated("7: r0 += " REG_AX)
__xlated("8: exit")
__naked void sdiv_add(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 s/= 2;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("sdiv by -3 subtracts the dividend")
__success
__arch_x86_64 __arch_arm64 __arch_riscv64
__load_if_JITed()
__xlated("...")
__xlated("1: " REG_AX " = 0x5555555555555555")
__xlated("3: " REG_AX " sh*= r0")
__xlated("4: " REG_AX " -= r0")
__xlated("5: " REG_AX " s>>= 1")
__xlated("6: r0 = " REG_AX)
__xlated("7: r0 >>= 63")
__xlated("8: r0 += " REG_AX)
__xlated("9: exit")
__naked void sdiv_sub(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 s/= -3;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("smod by 10 keeps the division")
__success
__xlated("...")
__xlated("1: r0 s%= 10")
__xlated("2: exit")
__naked void smod_kept(void)
{
	asm volatile ("					\
	call %[bpf_get_prandom_u32];			\
	r0 s%%= 10;					\
	exit;						\
"	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

#else

SEC("socket")
__description("UHMUL/SHMUL or cpuv4 not supported, use a dummy test")
__skip("UHMUL/SHMUL or cpuv4 not supported")
__success
int dummy_test(void)
{
	return 0;
}

#endif

char _license[] SEC("license") = "GPL";
