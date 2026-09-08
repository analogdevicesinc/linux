/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 ARM Ltd.
 */
#ifndef __ASM_PERCPU_H
#define __ASM_PERCPU_H

#include <linux/bits.h>
#include <linux/preempt.h>
#include <linux/stringify.h>

#include <asm/alternative.h>
#include <asm/cmpxchg.h>
#include <asm/gpr-num.h>
#include <asm/stack_pointer.h>
#include <asm/sysreg.h>

static inline void set_my_cpu_offset(unsigned long off)
{
	asm volatile(ALTERNATIVE("msr tpidr_el1, %0",
				 "msr tpidr_el2, %0",
				 ARM64_HAS_VIRT_HOST_EXTN)
			:: "r" (off) : "memory");
}

static inline unsigned long __hyp_my_cpu_offset(void)
{
	/*
	 * Non-VHE hyp code runs with preemption disabled. No need to hazard
	 * the register access against barrier() as in __kern_my_cpu_offset.
	 */
	return read_sysreg(tpidr_el2);
}

#define __KERN_ASM_CPU_OFFSET(dst)					\
	ALTERNATIVE("mrs " dst ", tpidr_el1",				\
		    "mrs " dst ", tpidr_el2",				\
		       ARM64_HAS_VIRT_HOST_EXTN)

static inline unsigned long __kern_my_cpu_offset(void)
{
	unsigned long off;

	/*
	 * We want to allow caching the value, so avoid using volatile and
	 * instead use a fake stack read to hazard against barrier().
	 */
	asm(
	__KERN_ASM_CPU_OFFSET("%0")
	: "=r" (off)
	: "Q" (*(const unsigned long *)current_stack_pointer)
	);

	return off;
}

#define PCPU_GPR_PCP_SHIFT		0
#define PCPU_GPR_PCP			GENMASK(4, 0)
#define PCPU_GPR_OFF_SHIFT		5
#define PCPU_GPR_OFF			GENMASK(9, 5)
#define PCPU_GPR_ADDR_SHIFT		10
#define PCPU_GPR_ADDR			GENMASK(14, 10)

#define __VAL_PCPU_GPRS(pcp, off, addr)							\
	"("										\
		"(" __GPR_NUM(pcp)  " << " __stringify(PCPU_GPR_PCP_SHIFT) ") | "	\
		"(" __GPR_NUM(off)  " << " __stringify(PCPU_GPR_OFF_SHIFT) ") | "	\
		"(" __GPR_NUM(addr) " << " __stringify(PCPU_GPR_ADDR_SHIFT) ")"		\
	")"

#define __ASSERT_PCPU_GPRS_DISTINCT(pcp, off, addr)			\
	".if ("								\
		"(" __GPR_NUM(pcp) " == " __GPR_NUM(off) ") || "	\
		"(" __GPR_NUM(pcp) " == " __GPR_NUM(addr) ") || "	\
		"(" __GPR_NUM(off) " == " __GPR_NUM(addr) ")"		\
	"    )\n"							\
	".error \"PCPU GPRS overlap: {" pcp "," off "," addr "}\"\n"	\
	".endif\n"

#define ____PCPU_GPRS_BEGIN(gprs, pcp, off, addr)			\
	"// ____PCPU_GPRS_BEGIN(" gprs ", " pcp ", " off ", " addr")\n"	\
	__DEFINE_ASM_GPR_NUMS						\
	__DEFINE_ASM_GPR_ALIASES					\
	__ASSERT_PCPU_GPRS_DISTINCT(pcp, off, addr)			\
	"	mov w" off ", #" __VAL_PCPU_GPRS(pcp, off, addr) "\n"	\
	"	strh	w" off ", " gprs "\n"				\
	__KERN_ASM_CPU_OFFSET(off) "\n"

/*
 * Begin a PCPU GPR critical section which requires <addr> (and <off>).
 *
 * At the start of the critical section, and upon any (preemptible) exception
 * until __PCPU_GPRS_END():
 * - <off>  will be set to the current CPU's percpu offset.
 * - <addr> will be set to <pcp> + <off>.
 *
 * The <pcp>, <off>, and <addr> registers must be distinct GPRs.
 *
 * <gprs> must be '&current_thread_info()->pcpu_gprs', as a memory operand
 * which can be written both at the start and end of the critical section
 * (e.g. using "=Qo" constraints).
 */
#define __PCPU_GPRS_BEGIN(gprs, pcp, off, addr)				\
	____PCPU_GPRS_BEGIN(gprs, pcp, off, addr)			\
	"	add	" addr ", " pcp ", " off "\n"

/*
 * Begin a PCPU GPR critical section which only requires <off> and does not
 * require <addr>.
 *
 * This is only for operations that can use register-offset addressing,
 * e.g. STR <Xt>, [<Xn>, <Xm>].
 *
 * All other details are the same as __PCPU_GPRS_BEGIN().
 */
#define __PCPU_GPRS_BEGIN_OFFSET(gprs, pcp, off)			\
	____PCPU_GPRS_BEGIN(gprs, pcp, off, "xzr")

/*
 * End a PCPU GPR critical section.
 */
#define __PCPU_GPRS_END(gprs)						\
	"	strh	wzr, " gprs "\n"

#ifdef __KVM_NVHE_HYPERVISOR__
#define __my_cpu_offset __hyp_my_cpu_offset()
#else
#define __my_cpu_offset __kern_my_cpu_offset()
#endif

#define PERCPU_RW_OPS(sz)						\
static inline unsigned long __percpu_read_##sz(void *ptr)		\
{									\
	return READ_ONCE(*(u##sz *)ptr);				\
}									\
									\
static inline void __percpu_write_##sz(void *ptr, unsigned long val)	\
{									\
	WRITE_ONCE(*(u##sz *)ptr, (u##sz)val);				\
}

#define __PERCPU_OP_CASE(w, sfx, name, sz, op_llsc, op_lse)		\
static inline void							\
__percpu_##name##_case_##sz(void *ptr, unsigned long val)		\
{									\
	unsigned int loop;						\
	u##sz tmp;							\
									\
	asm volatile (ARM64_LSE_ATOMIC_INSN(				\
	/* LL/SC */							\
	"1:	ldxr" #sfx "\t%" #w "[tmp], %[ptr]\n"			\
		#op_llsc "\t%" #w "[tmp], %" #w "[tmp], %" #w "[val]\n"	\
	"	stxr" #sfx "\t%w[loop], %" #w "[tmp], %[ptr]\n"		\
	"	cbnz	%w[loop], 1b",					\
	/* LSE atomics */						\
		#op_lse #sfx "\t%" #w "[val], %" #w "[tmp], %[ptr]\n"	\
		__nops(3))						\
	: [loop] "=&r" (loop), [tmp] "=&r" (tmp),			\
	  [ptr] "+Q"(*(u##sz *)ptr)					\
	: [val] "r" ((u##sz)(val)));					\
}

#define __PERCPU_RET_OP_CASE(w, sfx, name, sz, op_llsc, op_lse)		\
static inline u##sz							\
__percpu_##name##_return_case_##sz(void *ptr, unsigned long val)	\
{									\
	unsigned int loop;						\
	u##sz ret;							\
									\
	asm volatile (ARM64_LSE_ATOMIC_INSN(				\
	/* LL/SC */							\
	"1:	ldxr" #sfx "\t%" #w "[ret], %[ptr]\n"			\
		#op_llsc "\t%" #w "[ret], %" #w "[ret], %" #w "[val]\n"	\
	"	stxr" #sfx "\t%w[loop], %" #w "[ret], %[ptr]\n"		\
	"	cbnz	%w[loop], 1b",					\
	/* LSE atomics */						\
		#op_lse #sfx "\t%" #w "[val], %" #w "[ret], %[ptr]\n"	\
		#op_llsc "\t%" #w "[ret], %" #w "[ret], %" #w "[val]\n"	\
		__nops(2))						\
	: [loop] "=&r" (loop), [ret] "=&r" (ret),			\
	  [ptr] "+Q"(*(u##sz *)ptr)					\
	: [val] "r" ((u##sz)(val)));					\
									\
	return ret;							\
}

#define PERCPU_OP(name, op_llsc, op_lse)				\
	__PERCPU_OP_CASE(w, b, name,  8, op_llsc, op_lse)		\
	__PERCPU_OP_CASE(w, h, name, 16, op_llsc, op_lse)		\
	__PERCPU_OP_CASE(w,  , name, 32, op_llsc, op_lse)		\
	__PERCPU_OP_CASE( ,  , name, 64, op_llsc, op_lse)

#define PERCPU_RET_OP(name, op_llsc, op_lse)				\
	__PERCPU_RET_OP_CASE(w, b, name,  8, op_llsc, op_lse)		\
	__PERCPU_RET_OP_CASE(w, h, name, 16, op_llsc, op_lse)		\
	__PERCPU_RET_OP_CASE(w,  , name, 32, op_llsc, op_lse)		\
	__PERCPU_RET_OP_CASE( ,  , name, 64, op_llsc, op_lse)

PERCPU_RW_OPS(8)
PERCPU_RW_OPS(16)
PERCPU_RW_OPS(32)
PERCPU_RW_OPS(64)

/*
 * Use value-returning atomics for CPU-local ops as they are more likely
 * to execute "near" to the CPU (e.g. in L1$).
 *
 * https://lore.kernel.org/r/e7d539ed-ced0-4b96-8ecd-048a5b803b85@paulmck-laptop
 */
PERCPU_OP(add, add, ldadd)
PERCPU_OP(andnot, bic, ldclr)
PERCPU_OP(or, orr, ldset)
PERCPU_RET_OP(add, add, ldadd)

#undef PERCPU_RW_OPS
#undef __PERCPU_OP_CASE
#undef __PERCPU_RET_OP_CASE
#undef PERCPU_OP
#undef PERCPU_RET_OP

/*
 * It would be nice to avoid the conditional call into the scheduler when
 * re-enabling preemption for preemptible kernels, but doing that in a way
 * which builds inside a module would mean messing directly with the preempt
 * count. If you do this, peterz and tglx will hunt you down.
 *
 * Not to mention it'll break the actual preemption model for missing a
 * preemption point when TIF_NEED_RESCHED gets set while preemption is
 * disabled.
 */

#define _pcp_protect(op, pcp, ...)					\
({									\
	preempt_disable_notrace();					\
	op(raw_cpu_ptr(&(pcp)), __VA_ARGS__);				\
	preempt_enable_notrace();					\
})

#define _pcp_protect_return(op, pcp, args...)				\
({									\
	typeof(pcp) __retval;						\
	preempt_disable_notrace();					\
	__retval = (typeof(pcp))op(raw_cpu_ptr(&(pcp)), ##args);	\
	preempt_enable_notrace();					\
	__retval;							\
})

#define this_cpu_read_1(pcp)		\
	_pcp_protect_return(__percpu_read_8, pcp)
#define this_cpu_read_2(pcp)		\
	_pcp_protect_return(__percpu_read_16, pcp)
#define this_cpu_read_4(pcp)		\
	_pcp_protect_return(__percpu_read_32, pcp)
#define this_cpu_read_8(pcp)		\
	_pcp_protect_return(__percpu_read_64, pcp)

#define this_cpu_write_1(pcp, val)	\
	_pcp_protect(__percpu_write_8, pcp, (unsigned long)(val))
#define this_cpu_write_2(pcp, val)	\
	_pcp_protect(__percpu_write_16, pcp, (unsigned long)(val))
#define this_cpu_write_4(pcp, val)	\
	_pcp_protect(__percpu_write_32, pcp, (unsigned long)(val))
#define this_cpu_write_8(pcp, val)	\
	_pcp_protect(__percpu_write_64, pcp, (unsigned long)(val))

#define this_cpu_add_1(pcp, val)	\
	_pcp_protect(__percpu_add_case_8, pcp, val)
#define this_cpu_add_2(pcp, val)	\
	_pcp_protect(__percpu_add_case_16, pcp, val)
#define this_cpu_add_4(pcp, val)	\
	_pcp_protect(__percpu_add_case_32, pcp, val)
#define this_cpu_add_8(pcp, val)	\
	_pcp_protect(__percpu_add_case_64, pcp, val)

#define this_cpu_add_return_1(pcp, val)	\
	_pcp_protect_return(__percpu_add_return_case_8, pcp, val)
#define this_cpu_add_return_2(pcp, val)	\
	_pcp_protect_return(__percpu_add_return_case_16, pcp, val)
#define this_cpu_add_return_4(pcp, val)	\
	_pcp_protect_return(__percpu_add_return_case_32, pcp, val)
#define this_cpu_add_return_8(pcp, val)	\
	_pcp_protect_return(__percpu_add_return_case_64, pcp, val)

#define this_cpu_and_1(pcp, val)	\
	_pcp_protect(__percpu_andnot_case_8, pcp, ~(u8)(val))
#define this_cpu_and_2(pcp, val)	\
	_pcp_protect(__percpu_andnot_case_16, pcp, ~(u16)(val))
#define this_cpu_and_4(pcp, val)	\
	_pcp_protect(__percpu_andnot_case_32, pcp, ~(u32)(val))
#define this_cpu_and_8(pcp, val)	\
	_pcp_protect(__percpu_andnot_case_64, pcp, ~(u64)(val))

#define this_cpu_or_1(pcp, val)		\
	_pcp_protect(__percpu_or_case_8, pcp, val)
#define this_cpu_or_2(pcp, val)		\
	_pcp_protect(__percpu_or_case_16, pcp, val)
#define this_cpu_or_4(pcp, val)		\
	_pcp_protect(__percpu_or_case_32, pcp, val)
#define this_cpu_or_8(pcp, val)		\
	_pcp_protect(__percpu_or_case_64, pcp, val)

#define this_cpu_xchg_1(pcp, val)	\
	_pcp_protect_return(xchg_relaxed, pcp, val)
#define this_cpu_xchg_2(pcp, val)	\
	_pcp_protect_return(xchg_relaxed, pcp, val)
#define this_cpu_xchg_4(pcp, val)	\
	_pcp_protect_return(xchg_relaxed, pcp, val)
#define this_cpu_xchg_8(pcp, val)	\
	_pcp_protect_return(xchg_relaxed, pcp, val)

#define this_cpu_cmpxchg_1(pcp, o, n)	\
	_pcp_protect_return(cmpxchg_relaxed, pcp, o, n)
#define this_cpu_cmpxchg_2(pcp, o, n)	\
	_pcp_protect_return(cmpxchg_relaxed, pcp, o, n)
#define this_cpu_cmpxchg_4(pcp, o, n)	\
	_pcp_protect_return(cmpxchg_relaxed, pcp, o, n)
#define this_cpu_cmpxchg_8(pcp, o, n)	\
	_pcp_protect_return(cmpxchg_relaxed, pcp, o, n)

#define this_cpu_cmpxchg64(pcp, o, n)	this_cpu_cmpxchg_8(pcp, o, n)

#define this_cpu_cmpxchg128(pcp, o, n)					\
({									\
	typedef typeof(pcp) pcp_op_T__;					\
	u128 old__, new__, ret__;					\
	pcp_op_T__ *ptr__;						\
	old__ = o;							\
	new__ = n;							\
	preempt_disable_notrace();					\
	ptr__ = raw_cpu_ptr(&(pcp));					\
	ret__ = cmpxchg128_local((void *)ptr__, old__, new__);		\
	preempt_enable_notrace();					\
	ret__;								\
})

#ifdef __KVM_NVHE_HYPERVISOR__
extern unsigned long __hyp_per_cpu_offset(unsigned int cpu);
#define __per_cpu_offset
#define per_cpu_offset(cpu)	__hyp_per_cpu_offset((cpu))
#endif

#include <asm-generic/percpu.h>

/* Redefine macros for nVHE hyp under DEBUG_PREEMPT to avoid its dependencies. */
#if defined(__KVM_NVHE_HYPERVISOR__) && defined(CONFIG_DEBUG_PREEMPT)
#undef	this_cpu_ptr
#define	this_cpu_ptr		raw_cpu_ptr
#undef	__this_cpu_read
#define	__this_cpu_read		raw_cpu_read
#undef	__this_cpu_write
#define	__this_cpu_write	raw_cpu_write
#endif

#endif /* __ASM_PERCPU_H */
