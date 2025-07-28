/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RISC-V processor specific defines
 *
 * Copyright (C) 2021 Western Digital Corporation or its affiliates.
 */
#ifndef SELFTEST_KVM_PROCESSOR_H
#define SELFTEST_KVM_PROCESSOR_H

#include <linux/stringify.h>
#include <asm/csr.h>
#include "kvm_util.h"

static inline uint64_t __kvm_reg_id(uint64_t type, uint64_t subtype,
				    uint64_t idx, uint64_t size)
{
	return KVM_REG_RISCV | type | subtype | idx | size;
}

#if __riscv_xlen == 64
#define KVM_REG_SIZE_ULONG	KVM_REG_SIZE_U64
#else
#define KVM_REG_SIZE_ULONG	KVM_REG_SIZE_U32
#endif

#define RISCV_CONFIG_REG(name)		__kvm_reg_id(KVM_REG_RISCV_CONFIG, 0,		\
						     KVM_REG_RISCV_CONFIG_REG(name),	\
						     KVM_REG_SIZE_ULONG)

#define RISCV_CORE_REG(name)		__kvm_reg_id(KVM_REG_RISCV_CORE, 0,		\
						     KVM_REG_RISCV_CORE_REG(name),	\
						     KVM_REG_SIZE_ULONG)

#define RISCV_GENERAL_CSR_REG(name)	__kvm_reg_id(KVM_REG_RISCV_CSR,			\
						     KVM_REG_RISCV_CSR_GENERAL,		\
						     KVM_REG_RISCV_CSR_REG(name),	\
						     KVM_REG_SIZE_ULONG)

#define RISCV_TIMER_REG(name)		__kvm_reg_id(KVM_REG_RISCV_TIMER, 0,		\
						     KVM_REG_RISCV_TIMER_REG(name),	\
						     KVM_REG_SIZE_U64)

#define RISCV_ISA_EXT_REG(idx)		__kvm_reg_id(KVM_REG_RISCV_ISA_EXT,		\
						     KVM_REG_RISCV_ISA_SINGLE,		\
						     idx, KVM_REG_SIZE_ULONG)

#define RISCV_SBI_EXT_REG(idx)		__kvm_reg_id(KVM_REG_RISCV_SBI_EXT,		\
						     KVM_REG_RISCV_SBI_SINGLE,		\
						     idx, KVM_REG_SIZE_ULONG)

bool __vcpu_has_ext(struct kvm_vcpu *vcpu, uint64_t ext);

static inline bool __vcpu_has_isa_ext(struct kvm_vcpu *vcpu, uint64_t isa_ext)
{
	return __vcpu_has_ext(vcpu, RISCV_ISA_EXT_REG(isa_ext));
}

static inline bool __vcpu_has_sbi_ext(struct kvm_vcpu *vcpu, uint64_t sbi_ext)
{
	return __vcpu_has_ext(vcpu, RISCV_SBI_EXT_REG(sbi_ext));
}

struct ex_regs {
	unsigned long ra;
	unsigned long sp;
	unsigned long gp;
	unsigned long tp;
	unsigned long t0;
	unsigned long t1;
	unsigned long t2;
	unsigned long s0;
	unsigned long s1;
	unsigned long a0;
	unsigned long a1;
	unsigned long a2;
	unsigned long a3;
	unsigned long a4;
	unsigned long a5;
	unsigned long a6;
	unsigned long a7;
	unsigned long s2;
	unsigned long s3;
	unsigned long s4;
	unsigned long s5;
	unsigned long s6;
	unsigned long s7;
	unsigned long s8;
	unsigned long s9;
	unsigned long s10;
	unsigned long s11;
	unsigned long t3;
	unsigned long t4;
	unsigned long t5;
	unsigned long t6;
	unsigned long epc;
	unsigned long status;
	unsigned long cause;
};

#define NR_VECTORS  2
#define NR_EXCEPTIONS  32
#define EC_MASK  (NR_EXCEPTIONS - 1)

typedef void(*exception_handler_fn)(struct ex_regs *);

void vm_init_vector_tables(struct kvm_vm *vm);
void vcpu_init_vector_tables(struct kvm_vcpu *vcpu);

void vm_install_exception_handler(struct kvm_vm *vm, int vector, exception_handler_fn handler);

void vm_install_interrupt_handler(struct kvm_vm *vm, exception_handler_fn handler);

/* L3 index Bit[47:39] */
#define PGTBL_L3_INDEX_MASK			0x0000FF8000000000ULL
#define PGTBL_L3_INDEX_SHIFT			39
#define PGTBL_L3_BLOCK_SHIFT			39
#define PGTBL_L3_BLOCK_SIZE			0x0000008000000000ULL
#define PGTBL_L3_MAP_MASK			(~(PGTBL_L3_BLOCK_SIZE - 1))
/* L2 index Bit[38:30] */
#define PGTBL_L2_INDEX_MASK			0x0000007FC0000000ULL
#define PGTBL_L2_INDEX_SHIFT			30
#define PGTBL_L2_BLOCK_SHIFT			30
#define PGTBL_L2_BLOCK_SIZE			0x0000000040000000ULL
#define PGTBL_L2_MAP_MASK			(~(PGTBL_L2_BLOCK_SIZE - 1))
/* L1 index Bit[29:21] */
#define PGTBL_L1_INDEX_MASK			0x000000003FE00000ULL
#define PGTBL_L1_INDEX_SHIFT			21
#define PGTBL_L1_BLOCK_SHIFT			21
#define PGTBL_L1_BLOCK_SIZE			0x0000000000200000ULL
#define PGTBL_L1_MAP_MASK			(~(PGTBL_L1_BLOCK_SIZE - 1))
/* L0 index Bit[20:12] */
#define PGTBL_L0_INDEX_MASK			0x00000000001FF000ULL
#define PGTBL_L0_INDEX_SHIFT			12
#define PGTBL_L0_BLOCK_SHIFT			12
#define PGTBL_L0_BLOCK_SIZE			0x0000000000001000ULL
#define PGTBL_L0_MAP_MASK			(~(PGTBL_L0_BLOCK_SIZE - 1))

#define PGTBL_PTE_ADDR_MASK			0x003FFFFFFFFFFC00ULL
#define PGTBL_PTE_ADDR_SHIFT			10
#define PGTBL_PTE_RSW_MASK			0x0000000000000300ULL
#define PGTBL_PTE_RSW_SHIFT			8
#define PGTBL_PTE_DIRTY_MASK			0x0000000000000080ULL
#define PGTBL_PTE_DIRTY_SHIFT			7
#define PGTBL_PTE_ACCESSED_MASK			0x0000000000000040ULL
#define PGTBL_PTE_ACCESSED_SHIFT		6
#define PGTBL_PTE_GLOBAL_MASK			0x0000000000000020ULL
#define PGTBL_PTE_GLOBAL_SHIFT			5
#define PGTBL_PTE_USER_MASK			0x0000000000000010ULL
#define PGTBL_PTE_USER_SHIFT			4
#define PGTBL_PTE_EXECUTE_MASK			0x0000000000000008ULL
#define PGTBL_PTE_EXECUTE_SHIFT			3
#define PGTBL_PTE_WRITE_MASK			0x0000000000000004ULL
#define PGTBL_PTE_WRITE_SHIFT			2
#define PGTBL_PTE_READ_MASK			0x0000000000000002ULL
#define PGTBL_PTE_READ_SHIFT			1
#define PGTBL_PTE_PERM_MASK			(PGTBL_PTE_ACCESSED_MASK | \
						 PGTBL_PTE_DIRTY_MASK | \
						 PGTBL_PTE_EXECUTE_MASK | \
						 PGTBL_PTE_WRITE_MASK | \
						 PGTBL_PTE_READ_MASK)
#define PGTBL_PTE_VALID_MASK			0x0000000000000001ULL
#define PGTBL_PTE_VALID_SHIFT			0

#define PGTBL_PAGE_SIZE				PGTBL_L0_BLOCK_SIZE
#define PGTBL_PAGE_SIZE_SHIFT			PGTBL_L0_BLOCK_SHIFT

static inline void local_irq_enable(void)
{
	csr_set(CSR_SSTATUS, SR_SIE);
}

static inline void local_irq_disable(void)
{
	csr_clear(CSR_SSTATUS, SR_SIE);
}

#endif /* SELFTEST_KVM_PROCESSOR_H */
