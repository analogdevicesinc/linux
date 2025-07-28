// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

#if (defined(__TARGET_ARCH_arm64) || defined(__TARGET_ARCH_x86) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64) || \
	defined(__TARGET_ARCH_arm) || defined(__TARGET_ARCH_s390) || \
	defined(__TARGET_ARCH_loongarch)) && \
	__clang_major__ >= 18

SEC("socket")
__description("LDSX, S8")
__success __success_unpriv __retval(-2)
__naked void ldsx_s8(void)
{
	asm volatile (
	"r1 = 0x3fe;"
	"*(u64 *)(r10 - 8) = r1;"
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	"r0 = *(s8 *)(r10 - 8);"
#else
	"r0 = *(s8 *)(r10 - 1);"
#endif
	"exit;"
	::: __clobber_all);
}

SEC("socket")
__description("LDSX, S16")
__success __success_unpriv __retval(-2)
__naked void ldsx_s16(void)
{
	asm volatile (
	"r1 = 0x3fffe;"
	"*(u64 *)(r10 - 8) = r1;"
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	"r0 = *(s16 *)(r10 - 8);"
#else
	"r0 = *(s16 *)(r10 - 2);"
#endif
	"exit;"
	::: __clobber_all);
}

SEC("socket")
__description("LDSX, S32")
__success __success_unpriv __retval(-1)
__naked void ldsx_s32(void)
{
	asm volatile (
	"r1 = 0xfffffffe;"
	"*(u64 *)(r10 - 8) = r1;"
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	"r0 = *(s32 *)(r10 - 8);"
#else
	"r0 = *(s32 *)(r10 - 4);"
#endif
	"r0 >>= 1;"
	"exit;"
	::: __clobber_all);
}

SEC("socket")
__description("LDSX, S8 range checking, privileged")
__log_level(2) __success __retval(1)
__msg("R1_w=scalar(smin=smin32=-128,smax=smax32=127)")
__naked void ldsx_s8_range_priv(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"*(u64 *)(r10 - 8) = r0;"
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	"r1 = *(s8 *)(r10 - 8);"
#else
	"r1 = *(s8 *)(r10 - 1);"
#endif
	/* r1 with s8 range */
	"if r1 s> 0x7f goto l0_%=;"
	"if r1 s< -0x80 goto l0_%=;"
	"r0 = 1;"
"l1_%=:"
	"exit;"
"l0_%=:"
	"r0 = 2;"
	"goto l1_%=;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("LDSX, S16 range checking")
__success __success_unpriv __retval(1)
__naked void ldsx_s16_range(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"*(u64 *)(r10 - 8) = r0;"
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	"r1 = *(s16 *)(r10 - 8);"
#else
	"r1 = *(s16 *)(r10 - 2);"
#endif
	/* r1 with s16 range */
	"if r1 s> 0x7fff goto l0_%=;"
	"if r1 s< -0x8000 goto l0_%=;"
	"r0 = 1;"
"l1_%=:"
	"exit;"
"l0_%=:"
	"r0 = 2;"
	"goto l1_%=;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("socket")
__description("LDSX, S32 range checking")
__success __success_unpriv __retval(1)
__naked void ldsx_s32_range(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"*(u64 *)(r10 - 8) = r0;"
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	"r1 = *(s32 *)(r10 - 8);"
#else
	"r1 = *(s32 *)(r10 - 4);"
#endif
	/* r1 with s16 range */
	"if r1 s> 0x7fffFFFF goto l0_%=;"
	"if r1 s< -0x80000000 goto l0_%=;"
	"r0 = 1;"
"l1_%=:"
	"exit;"
"l0_%=:"
	"r0 = 2;"
	"goto l1_%=;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("xdp")
__description("LDSX, xdp s32 xdp_md->data")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_1(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[xdp_md_data]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(xdp_md_data, offsetof(struct xdp_md, data))
	: __clobber_all);
}

SEC("xdp")
__description("LDSX, xdp s32 xdp_md->data_end")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_2(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[xdp_md_data_end]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(xdp_md_data_end, offsetof(struct xdp_md, data_end))
	: __clobber_all);
}

SEC("xdp")
__description("LDSX, xdp s32 xdp_md->data_meta")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_3(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[xdp_md_data_meta]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(xdp_md_data_meta, offsetof(struct xdp_md, data_meta))
	: __clobber_all);
}

SEC("tcx/ingress")
__description("LDSX, tcx s32 __sk_buff->data")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_4(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[sk_buff_data]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(sk_buff_data, offsetof(struct __sk_buff, data))
	: __clobber_all);
}

SEC("tcx/ingress")
__description("LDSX, tcx s32 __sk_buff->data_end")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_5(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[sk_buff_data_end]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(sk_buff_data_end, offsetof(struct __sk_buff, data_end))
	: __clobber_all);
}

SEC("tcx/ingress")
__description("LDSX, tcx s32 __sk_buff->data_meta")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_6(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[sk_buff_data_meta]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(sk_buff_data_meta, offsetof(struct __sk_buff, data_meta))
	: __clobber_all);
}

SEC("flow_dissector")
__description("LDSX, flow_dissector s32 __sk_buff->data")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_7(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[sk_buff_data]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(sk_buff_data, offsetof(struct __sk_buff, data))
	: __clobber_all);
}

SEC("flow_dissector")
__description("LDSX, flow_dissector s32 __sk_buff->data_end")
__failure __msg("invalid bpf_context access")
__naked void ldsx_ctx_8(void)
{
	asm volatile (
	"r2 = *(s32 *)(r1 + %[sk_buff_data_end]);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_const(sk_buff_data_end, offsetof(struct __sk_buff, data_end))
	: __clobber_all);
}

#else

SEC("socket")
__description("cpuv4 is not supported by compiler or jit, use a dummy test")
__success
int dummy_test(void)
{
	return 0;
}

#endif

char _license[] SEC("license") = "GPL";
