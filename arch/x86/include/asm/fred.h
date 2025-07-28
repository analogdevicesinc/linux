/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Macros for Flexible Return and Event Delivery (FRED)
 */

#ifndef ASM_X86_FRED_H
#define ASM_X86_FRED_H

#include <linux/const.h>

#include <asm/asm.h>
#include <asm/trapnr.h>

/*
 * FRED event return instruction opcodes for ERET{S,U}; supported in
 * binutils >= 2.41.
 */
#define ERETS			_ASM_BYTES(0xf2,0x0f,0x01,0xca)
#define ERETU			_ASM_BYTES(0xf3,0x0f,0x01,0xca)

/*
 * RSP is aligned to a 64-byte boundary before used to push a new stack frame
 */
#define FRED_STACK_FRAME_RSP_MASK	_AT(unsigned long, (~0x3f))

/*
 * Used for the return address for call emulation during code patching,
 * and measured in 64-byte cache lines.
 */
#define FRED_CONFIG_REDZONE_AMOUNT	1
#define FRED_CONFIG_REDZONE		(_AT(unsigned long, FRED_CONFIG_REDZONE_AMOUNT) << 6)
#define FRED_CONFIG_INT_STKLVL(l)	(_AT(unsigned long, l) << 9)
#define FRED_CONFIG_ENTRYPOINT(p)	_AT(unsigned long, (p))

#ifndef __ASSEMBLY__

#ifdef CONFIG_X86_FRED
#include <linux/kernel.h>
#include <linux/sched/task_stack.h>

#include <asm/ptrace.h>

struct fred_info {
	/* Event data: CR2, DR6, ... */
	unsigned long edata;
	unsigned long resv;
};

/* Full format of the FRED stack frame */
struct fred_frame {
	struct pt_regs   regs;
	struct fred_info info;
};

static __always_inline struct fred_info *fred_info(struct pt_regs *regs)
{
	return &container_of(regs, struct fred_frame, regs)->info;
}

static __always_inline unsigned long fred_event_data(struct pt_regs *regs)
{
	return fred_info(regs)->edata;
}

void asm_fred_entrypoint_user(void);
void asm_fred_entrypoint_kernel(void);
void asm_fred_entry_from_kvm(struct fred_ss);

__visible void fred_entry_from_user(struct pt_regs *regs);
__visible void fred_entry_from_kernel(struct pt_regs *regs);
__visible void __fred_entry_from_kvm(struct pt_regs *regs);

/* Can be called from noinstr code, thus __always_inline */
static __always_inline void fred_entry_from_kvm(unsigned int type, unsigned int vector)
{
	struct fred_ss ss = {
		.ss     =__KERNEL_DS,
		.type   = type,
		.vector = vector,
		.nmi    = type == EVENT_TYPE_NMI,
		.lm     = 1,
	};

	asm_fred_entry_from_kvm(ss);
}

void cpu_init_fred_exceptions(void);
void cpu_init_fred_rsps(void);
void fred_complete_exception_setup(void);

DECLARE_PER_CPU(unsigned long, fred_rsp0);

static __always_inline void fred_sync_rsp0(unsigned long rsp0)
{
	__this_cpu_write(fred_rsp0, rsp0);
}

static __always_inline void fred_update_rsp0(void)
{
	unsigned long rsp0 = (unsigned long) task_stack_page(current) + THREAD_SIZE;

	if (cpu_feature_enabled(X86_FEATURE_FRED) && (__this_cpu_read(fred_rsp0) != rsp0)) {
		wrmsrns(MSR_IA32_FRED_RSP0, rsp0);
		__this_cpu_write(fred_rsp0, rsp0);
	}
}
#else /* CONFIG_X86_FRED */
static __always_inline unsigned long fred_event_data(struct pt_regs *regs) { return 0; }
static inline void cpu_init_fred_exceptions(void) { }
static inline void cpu_init_fred_rsps(void) { }
static inline void fred_complete_exception_setup(void) { }
static inline void fred_entry_from_kvm(unsigned int type, unsigned int vector) { }
static inline void fred_sync_rsp0(unsigned long rsp0) { }
static inline void fred_update_rsp0(void) { }
#endif /* CONFIG_X86_FRED */
#endif /* !__ASSEMBLY__ */

#endif /* ASM_X86_FRED_H */
