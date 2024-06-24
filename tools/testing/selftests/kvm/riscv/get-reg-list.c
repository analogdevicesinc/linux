// SPDX-License-Identifier: GPL-2.0
/*
 * Check for KVM_GET_REG_LIST regressions.
 *
 * Copyright (c) 2023 Intel Corporation
 *
 */
#include <stdio.h>
#include "kvm_util.h"
#include "test_util.h"
#include "processor.h"

#define REG_MASK (KVM_REG_ARCH_MASK | KVM_REG_SIZE_MASK)

enum {
	VCPU_FEATURE_ISA_EXT = 0,
	VCPU_FEATURE_SBI_EXT,
};

static bool isa_ext_cant_disable[KVM_RISCV_ISA_EXT_MAX];

bool filter_reg(__u64 reg)
{
	switch (reg & ~REG_MASK) {
	/*
	 * Same set of ISA_EXT registers are not present on all host because
	 * ISA_EXT registers are visible to the KVM user space based on the
	 * ISA extensions available on the host. Also, disabling an ISA
	 * extension using corresponding ISA_EXT register does not affect
	 * the visibility of the ISA_EXT register itself.
	 *
	 * Based on above, we should filter-out all ISA_EXT registers.
	 *
	 * Note: The below list is alphabetically sorted.
	 */
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_A:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_C:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_D:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_F:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_H:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_I:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_M:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_V:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SMSTATEEN:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SSAIA:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SSCOFPMF:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SSTC:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SVINVAL:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SVNAPOT:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SVPBMT:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZACAS:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZBA:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZBB:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZBC:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZBKB:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZBKC:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZBKX:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZBS:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZFA:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZFH:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZFHMIN:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZICBOM:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZICBOZ:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZICNTR:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZICOND:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZICSR:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZIFENCEI:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZIHINTNTL:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZIHINTPAUSE:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZIHPM:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZKND:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZKNE:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZKNH:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZKR:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZKSED:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZKSH:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZKT:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZTSO:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVBB:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVBC:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVFH:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVFHMIN:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKB:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKG:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKNED:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKNHA:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKNHB:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKSED:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKSH:
	case KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZVKT:
	/*
	 * Like ISA_EXT registers, SBI_EXT registers are only visible when the
	 * host supports them and disabling them does not affect the visibility
	 * of the SBI_EXT register itself.
	 */
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_V01:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_TIME:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_IPI:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_RFENCE:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_SRST:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_HSM:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_PMU:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_DBCN:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_STA:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_EXPERIMENTAL:
	case KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_VENDOR:
		return true;
	/* AIA registers are always available when Ssaia can't be disabled */
	case KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(siselect):
	case KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio1):
	case KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio2):
	case KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(sieh):
	case KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(siph):
	case KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio1h):
	case KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio2h):
		return isa_ext_cant_disable[KVM_RISCV_ISA_EXT_SSAIA];
	default:
		break;
	}

	return false;
}

bool check_reject_set(int err)
{
	return err == EINVAL;
}

void finalize_vcpu(struct kvm_vcpu *vcpu, struct vcpu_reg_list *c)
{
	unsigned long isa_ext_state[KVM_RISCV_ISA_EXT_MAX] = { 0 };
	struct vcpu_reg_sublist *s;
	uint64_t feature;
	int rc;

	for (int i = 0; i < KVM_RISCV_ISA_EXT_MAX; i++)
		__vcpu_get_reg(vcpu, RISCV_ISA_EXT_REG(i), &isa_ext_state[i]);

	/*
	 * Disable all extensions which were enabled by default
	 * if they were available in the risc-v host.
	 */
	for (int i = 0; i < KVM_RISCV_ISA_EXT_MAX; i++) {
		rc = __vcpu_set_reg(vcpu, RISCV_ISA_EXT_REG(i), 0);
		if (rc && isa_ext_state[i])
			isa_ext_cant_disable[i] = true;
	}

	for (int i = 0; i < KVM_RISCV_SBI_EXT_MAX; i++) {
		rc = __vcpu_set_reg(vcpu, RISCV_SBI_EXT_REG(i), 0);
		TEST_ASSERT(!rc || (rc == -1 && errno == ENOENT), "Unexpected error");
	}

	for_each_sublist(c, s) {
		if (!s->feature)
			continue;

		switch (s->feature_type) {
		case VCPU_FEATURE_ISA_EXT:
			feature = RISCV_ISA_EXT_REG(s->feature);
			break;
		case VCPU_FEATURE_SBI_EXT:
			feature = RISCV_SBI_EXT_REG(s->feature);
			break;
		default:
			TEST_FAIL("Unknown feature type");
		}

		/* Try to enable the desired extension */
		__vcpu_set_reg(vcpu, feature, 1);

		/* Double check whether the desired extension was enabled */
		__TEST_REQUIRE(__vcpu_has_ext(vcpu, feature),
			       "%s not available, skipping tests", s->name);
	}
}

static const char *config_id_to_str(const char *prefix, __u64 id)
{
	/* reg_off is the offset into struct kvm_riscv_config */
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_CONFIG);

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_CONFIG);

	switch (reg_off) {
	case KVM_REG_RISCV_CONFIG_REG(isa):
		return "KVM_REG_RISCV_CONFIG_REG(isa)";
	case KVM_REG_RISCV_CONFIG_REG(zicbom_block_size):
		return "KVM_REG_RISCV_CONFIG_REG(zicbom_block_size)";
	case KVM_REG_RISCV_CONFIG_REG(zicboz_block_size):
		return "KVM_REG_RISCV_CONFIG_REG(zicboz_block_size)";
	case KVM_REG_RISCV_CONFIG_REG(mvendorid):
		return "KVM_REG_RISCV_CONFIG_REG(mvendorid)";
	case KVM_REG_RISCV_CONFIG_REG(marchid):
		return "KVM_REG_RISCV_CONFIG_REG(marchid)";
	case KVM_REG_RISCV_CONFIG_REG(mimpid):
		return "KVM_REG_RISCV_CONFIG_REG(mimpid)";
	case KVM_REG_RISCV_CONFIG_REG(satp_mode):
		return "KVM_REG_RISCV_CONFIG_REG(satp_mode)";
	}

	return strdup_printf("%lld /* UNKNOWN */", reg_off);
}

static const char *core_id_to_str(const char *prefix, __u64 id)
{
	/* reg_off is the offset into struct kvm_riscv_core */
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_CORE);

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_CORE);

	switch (reg_off) {
	case KVM_REG_RISCV_CORE_REG(regs.pc):
		return "KVM_REG_RISCV_CORE_REG(regs.pc)";
	case KVM_REG_RISCV_CORE_REG(regs.ra):
		return "KVM_REG_RISCV_CORE_REG(regs.ra)";
	case KVM_REG_RISCV_CORE_REG(regs.sp):
		return "KVM_REG_RISCV_CORE_REG(regs.sp)";
	case KVM_REG_RISCV_CORE_REG(regs.gp):
		return "KVM_REG_RISCV_CORE_REG(regs.gp)";
	case KVM_REG_RISCV_CORE_REG(regs.tp):
		return "KVM_REG_RISCV_CORE_REG(regs.tp)";
	case KVM_REG_RISCV_CORE_REG(regs.t0) ... KVM_REG_RISCV_CORE_REG(regs.t2):
		return strdup_printf("KVM_REG_RISCV_CORE_REG(regs.t%lld)",
			   reg_off - KVM_REG_RISCV_CORE_REG(regs.t0));
	case KVM_REG_RISCV_CORE_REG(regs.s0) ... KVM_REG_RISCV_CORE_REG(regs.s1):
		return strdup_printf("KVM_REG_RISCV_CORE_REG(regs.s%lld)",
			   reg_off - KVM_REG_RISCV_CORE_REG(regs.s0));
	case KVM_REG_RISCV_CORE_REG(regs.a0) ... KVM_REG_RISCV_CORE_REG(regs.a7):
		return strdup_printf("KVM_REG_RISCV_CORE_REG(regs.a%lld)",
			   reg_off - KVM_REG_RISCV_CORE_REG(regs.a0));
	case KVM_REG_RISCV_CORE_REG(regs.s2) ... KVM_REG_RISCV_CORE_REG(regs.s11):
		return strdup_printf("KVM_REG_RISCV_CORE_REG(regs.s%lld)",
			   reg_off - KVM_REG_RISCV_CORE_REG(regs.s2) + 2);
	case KVM_REG_RISCV_CORE_REG(regs.t3) ... KVM_REG_RISCV_CORE_REG(regs.t6):
		return strdup_printf("KVM_REG_RISCV_CORE_REG(regs.t%lld)",
			   reg_off - KVM_REG_RISCV_CORE_REG(regs.t3) + 3);
	case KVM_REG_RISCV_CORE_REG(mode):
		return "KVM_REG_RISCV_CORE_REG(mode)";
	}

	return strdup_printf("%lld /* UNKNOWN */", reg_off);
}

#define RISCV_CSR_GENERAL(csr) \
	"KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(" #csr ")"
#define RISCV_CSR_AIA(csr) \
	"KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_REG(" #csr ")"
#define RISCV_CSR_SMSTATEEN(csr) \
	"KVM_REG_RISCV_CSR_SMSTATEEN | KVM_REG_RISCV_CSR_REG(" #csr ")"

static const char *general_csr_id_to_str(__u64 reg_off)
{
	/* reg_off is the offset into struct kvm_riscv_csr */
	switch (reg_off) {
	case KVM_REG_RISCV_CSR_REG(sstatus):
		return RISCV_CSR_GENERAL(sstatus);
	case KVM_REG_RISCV_CSR_REG(sie):
		return RISCV_CSR_GENERAL(sie);
	case KVM_REG_RISCV_CSR_REG(stvec):
		return RISCV_CSR_GENERAL(stvec);
	case KVM_REG_RISCV_CSR_REG(sscratch):
		return RISCV_CSR_GENERAL(sscratch);
	case KVM_REG_RISCV_CSR_REG(sepc):
		return RISCV_CSR_GENERAL(sepc);
	case KVM_REG_RISCV_CSR_REG(scause):
		return RISCV_CSR_GENERAL(scause);
	case KVM_REG_RISCV_CSR_REG(stval):
		return RISCV_CSR_GENERAL(stval);
	case KVM_REG_RISCV_CSR_REG(sip):
		return RISCV_CSR_GENERAL(sip);
	case KVM_REG_RISCV_CSR_REG(satp):
		return RISCV_CSR_GENERAL(satp);
	case KVM_REG_RISCV_CSR_REG(scounteren):
		return RISCV_CSR_GENERAL(scounteren);
	case KVM_REG_RISCV_CSR_REG(senvcfg):
		return RISCV_CSR_GENERAL(senvcfg);
	}

	return strdup_printf("KVM_REG_RISCV_CSR_GENERAL | %lld /* UNKNOWN */", reg_off);
}

static const char *aia_csr_id_to_str(__u64 reg_off)
{
	/* reg_off is the offset into struct kvm_riscv_aia_csr */
	switch (reg_off) {
	case KVM_REG_RISCV_CSR_AIA_REG(siselect):
		return RISCV_CSR_AIA(siselect);
	case KVM_REG_RISCV_CSR_AIA_REG(iprio1):
		return RISCV_CSR_AIA(iprio1);
	case KVM_REG_RISCV_CSR_AIA_REG(iprio2):
		return RISCV_CSR_AIA(iprio2);
	case KVM_REG_RISCV_CSR_AIA_REG(sieh):
		return RISCV_CSR_AIA(sieh);
	case KVM_REG_RISCV_CSR_AIA_REG(siph):
		return RISCV_CSR_AIA(siph);
	case KVM_REG_RISCV_CSR_AIA_REG(iprio1h):
		return RISCV_CSR_AIA(iprio1h);
	case KVM_REG_RISCV_CSR_AIA_REG(iprio2h):
		return RISCV_CSR_AIA(iprio2h);
	}

	return strdup_printf("KVM_REG_RISCV_CSR_AIA | %lld /* UNKNOWN */", reg_off);
}

static const char *smstateen_csr_id_to_str(__u64 reg_off)
{
	/* reg_off is the offset into struct kvm_riscv_smstateen_csr */
	switch (reg_off) {
	case KVM_REG_RISCV_CSR_SMSTATEEN_REG(sstateen0):
		return RISCV_CSR_SMSTATEEN(sstateen0);
	}

	TEST_FAIL("Unknown smstateen csr reg: 0x%llx", reg_off);
	return NULL;
}

static const char *csr_id_to_str(const char *prefix, __u64 id)
{
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_CSR);
	__u64 reg_subtype = reg_off & KVM_REG_RISCV_SUBTYPE_MASK;

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_CSR);

	reg_off &= ~KVM_REG_RISCV_SUBTYPE_MASK;

	switch (reg_subtype) {
	case KVM_REG_RISCV_CSR_GENERAL:
		return general_csr_id_to_str(reg_off);
	case KVM_REG_RISCV_CSR_AIA:
		return aia_csr_id_to_str(reg_off);
	case KVM_REG_RISCV_CSR_SMSTATEEN:
		return smstateen_csr_id_to_str(reg_off);
	}

	return strdup_printf("%lld | %lld /* UNKNOWN */", reg_subtype, reg_off);
}

static const char *timer_id_to_str(const char *prefix, __u64 id)
{
	/* reg_off is the offset into struct kvm_riscv_timer */
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_TIMER);

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_TIMER);

	switch (reg_off) {
	case KVM_REG_RISCV_TIMER_REG(frequency):
		return "KVM_REG_RISCV_TIMER_REG(frequency)";
	case KVM_REG_RISCV_TIMER_REG(time):
		return "KVM_REG_RISCV_TIMER_REG(time)";
	case KVM_REG_RISCV_TIMER_REG(compare):
		return "KVM_REG_RISCV_TIMER_REG(compare)";
	case KVM_REG_RISCV_TIMER_REG(state):
		return "KVM_REG_RISCV_TIMER_REG(state)";
	}

	return strdup_printf("%lld /* UNKNOWN */", reg_off);
}

static const char *fp_f_id_to_str(const char *prefix, __u64 id)
{
	/* reg_off is the offset into struct __riscv_f_ext_state */
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_FP_F);

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_FP_F);

	switch (reg_off) {
	case KVM_REG_RISCV_FP_F_REG(f[0]) ...
	     KVM_REG_RISCV_FP_F_REG(f[31]):
		return strdup_printf("KVM_REG_RISCV_FP_F_REG(f[%lld])", reg_off);
	case KVM_REG_RISCV_FP_F_REG(fcsr):
		return "KVM_REG_RISCV_FP_F_REG(fcsr)";
	}

	return strdup_printf("%lld /* UNKNOWN */", reg_off);
}

static const char *fp_d_id_to_str(const char *prefix, __u64 id)
{
	/* reg_off is the offset into struct __riscv_d_ext_state */
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_FP_D);

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_FP_D);

	switch (reg_off) {
	case KVM_REG_RISCV_FP_D_REG(f[0]) ...
	     KVM_REG_RISCV_FP_D_REG(f[31]):
		return strdup_printf("KVM_REG_RISCV_FP_D_REG(f[%lld])", reg_off);
	case KVM_REG_RISCV_FP_D_REG(fcsr):
		return "KVM_REG_RISCV_FP_D_REG(fcsr)";
	}

	return strdup_printf("%lld /* UNKNOWN */", reg_off);
}

#define KVM_ISA_EXT_ARR(ext)		\
[KVM_RISCV_ISA_EXT_##ext] = "KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_" #ext

static const char *isa_ext_single_id_to_str(__u64 reg_off)
{
	static const char * const kvm_isa_ext_reg_name[] = {
		KVM_ISA_EXT_ARR(A),
		KVM_ISA_EXT_ARR(C),
		KVM_ISA_EXT_ARR(D),
		KVM_ISA_EXT_ARR(F),
		KVM_ISA_EXT_ARR(H),
		KVM_ISA_EXT_ARR(I),
		KVM_ISA_EXT_ARR(M),
		KVM_ISA_EXT_ARR(V),
		KVM_ISA_EXT_ARR(SMSTATEEN),
		KVM_ISA_EXT_ARR(SSAIA),
		KVM_ISA_EXT_ARR(SSCOFPMF),
		KVM_ISA_EXT_ARR(SSTC),
		KVM_ISA_EXT_ARR(SVINVAL),
		KVM_ISA_EXT_ARR(SVNAPOT),
		KVM_ISA_EXT_ARR(SVPBMT),
		KVM_ISA_EXT_ARR(ZACAS),
		KVM_ISA_EXT_ARR(ZBA),
		KVM_ISA_EXT_ARR(ZBB),
		KVM_ISA_EXT_ARR(ZBC),
		KVM_ISA_EXT_ARR(ZBKB),
		KVM_ISA_EXT_ARR(ZBKC),
		KVM_ISA_EXT_ARR(ZBKX),
		KVM_ISA_EXT_ARR(ZBS),
		KVM_ISA_EXT_ARR(ZFA),
		KVM_ISA_EXT_ARR(ZFH),
		KVM_ISA_EXT_ARR(ZFHMIN),
		KVM_ISA_EXT_ARR(ZICBOM),
		KVM_ISA_EXT_ARR(ZICBOZ),
		KVM_ISA_EXT_ARR(ZICNTR),
		KVM_ISA_EXT_ARR(ZICOND),
		KVM_ISA_EXT_ARR(ZICSR),
		KVM_ISA_EXT_ARR(ZIFENCEI),
		KVM_ISA_EXT_ARR(ZIHINTNTL),
		KVM_ISA_EXT_ARR(ZIHINTPAUSE),
		KVM_ISA_EXT_ARR(ZIHPM),
		KVM_ISA_EXT_ARR(ZKND),
		KVM_ISA_EXT_ARR(ZKNE),
		KVM_ISA_EXT_ARR(ZKNH),
		KVM_ISA_EXT_ARR(ZKR),
		KVM_ISA_EXT_ARR(ZKSED),
		KVM_ISA_EXT_ARR(ZKSH),
		KVM_ISA_EXT_ARR(ZKT),
		KVM_ISA_EXT_ARR(ZTSO),
		KVM_ISA_EXT_ARR(ZVBB),
		KVM_ISA_EXT_ARR(ZVBC),
		KVM_ISA_EXT_ARR(ZVFH),
		KVM_ISA_EXT_ARR(ZVFHMIN),
		KVM_ISA_EXT_ARR(ZVKB),
		KVM_ISA_EXT_ARR(ZVKG),
		KVM_ISA_EXT_ARR(ZVKNED),
		KVM_ISA_EXT_ARR(ZVKNHA),
		KVM_ISA_EXT_ARR(ZVKNHB),
		KVM_ISA_EXT_ARR(ZVKSED),
		KVM_ISA_EXT_ARR(ZVKSH),
		KVM_ISA_EXT_ARR(ZVKT),
	};

	if (reg_off >= ARRAY_SIZE(kvm_isa_ext_reg_name))
		return strdup_printf("KVM_REG_RISCV_ISA_SINGLE | %lld /* UNKNOWN */", reg_off);

	return kvm_isa_ext_reg_name[reg_off];
}

static const char *isa_ext_multi_id_to_str(__u64 reg_subtype, __u64 reg_off)
{
	const char *unknown = "";

	if (reg_off > KVM_REG_RISCV_ISA_MULTI_REG_LAST)
		unknown = " /* UNKNOWN */";

	switch (reg_subtype) {
	case KVM_REG_RISCV_ISA_MULTI_EN:
		return strdup_printf("KVM_REG_RISCV_ISA_MULTI_EN | %lld%s", reg_off, unknown);
	case KVM_REG_RISCV_ISA_MULTI_DIS:
		return strdup_printf("KVM_REG_RISCV_ISA_MULTI_DIS | %lld%s", reg_off, unknown);
	}

	return strdup_printf("%lld | %lld /* UNKNOWN */", reg_subtype, reg_off);
}

static const char *isa_ext_id_to_str(const char *prefix, __u64 id)
{
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_ISA_EXT);
	__u64 reg_subtype = reg_off & KVM_REG_RISCV_SUBTYPE_MASK;

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_ISA_EXT);

	reg_off &= ~KVM_REG_RISCV_SUBTYPE_MASK;

	switch (reg_subtype) {
	case KVM_REG_RISCV_ISA_SINGLE:
		return isa_ext_single_id_to_str(reg_off);
	case KVM_REG_RISCV_ISA_MULTI_EN:
	case KVM_REG_RISCV_ISA_MULTI_DIS:
		return isa_ext_multi_id_to_str(reg_subtype, reg_off);
	}

	return strdup_printf("%lld | %lld /* UNKNOWN */", reg_subtype, reg_off);
}

#define KVM_SBI_EXT_ARR(ext)		\
[ext] = "KVM_REG_RISCV_SBI_SINGLE | " #ext

static const char *sbi_ext_single_id_to_str(__u64 reg_off)
{
	/* reg_off is KVM_RISCV_SBI_EXT_ID */
	static const char * const kvm_sbi_ext_reg_name[] = {
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_V01),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_TIME),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_IPI),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_RFENCE),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_SRST),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_HSM),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_PMU),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_STA),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_EXPERIMENTAL),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_VENDOR),
		KVM_SBI_EXT_ARR(KVM_RISCV_SBI_EXT_DBCN),
	};

	if (reg_off >= ARRAY_SIZE(kvm_sbi_ext_reg_name))
		return strdup_printf("KVM_REG_RISCV_SBI_SINGLE | %lld /* UNKNOWN */", reg_off);

	return kvm_sbi_ext_reg_name[reg_off];
}

static const char *sbi_ext_multi_id_to_str(__u64 reg_subtype, __u64 reg_off)
{
	const char *unknown = "";

	if (reg_off > KVM_REG_RISCV_SBI_MULTI_REG_LAST)
		unknown = " /* UNKNOWN */";

	switch (reg_subtype) {
	case KVM_REG_RISCV_SBI_MULTI_EN:
		return strdup_printf("KVM_REG_RISCV_SBI_MULTI_EN | %lld%s", reg_off, unknown);
	case KVM_REG_RISCV_SBI_MULTI_DIS:
		return strdup_printf("KVM_REG_RISCV_SBI_MULTI_DIS | %lld%s", reg_off, unknown);
	}

	return strdup_printf("%lld | %lld /* UNKNOWN */", reg_subtype, reg_off);
}

static const char *sbi_ext_id_to_str(const char *prefix, __u64 id)
{
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_SBI_EXT);
	__u64 reg_subtype = reg_off & KVM_REG_RISCV_SUBTYPE_MASK;

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_SBI_EXT);

	reg_off &= ~KVM_REG_RISCV_SUBTYPE_MASK;

	switch (reg_subtype) {
	case KVM_REG_RISCV_SBI_SINGLE:
		return sbi_ext_single_id_to_str(reg_off);
	case KVM_REG_RISCV_SBI_MULTI_EN:
	case KVM_REG_RISCV_SBI_MULTI_DIS:
		return sbi_ext_multi_id_to_str(reg_subtype, reg_off);
	}

	return strdup_printf("%lld | %lld /* UNKNOWN */", reg_subtype, reg_off);
}

static const char *sbi_sta_id_to_str(__u64 reg_off)
{
	switch (reg_off) {
	case 0: return "KVM_REG_RISCV_SBI_STA | KVM_REG_RISCV_SBI_STA_REG(shmem_lo)";
	case 1: return "KVM_REG_RISCV_SBI_STA | KVM_REG_RISCV_SBI_STA_REG(shmem_hi)";
	}
	return strdup_printf("KVM_REG_RISCV_SBI_STA | %lld /* UNKNOWN */", reg_off);
}

static const char *sbi_id_to_str(const char *prefix, __u64 id)
{
	__u64 reg_off = id & ~(REG_MASK | KVM_REG_RISCV_SBI_STATE);
	__u64 reg_subtype = reg_off & KVM_REG_RISCV_SUBTYPE_MASK;

	assert((id & KVM_REG_RISCV_TYPE_MASK) == KVM_REG_RISCV_SBI_STATE);

	reg_off &= ~KVM_REG_RISCV_SUBTYPE_MASK;

	switch (reg_subtype) {
	case KVM_REG_RISCV_SBI_STA:
		return sbi_sta_id_to_str(reg_off);
	}

	return strdup_printf("%lld | %lld /* UNKNOWN */", reg_subtype, reg_off);
}

void print_reg(const char *prefix, __u64 id)
{
	const char *reg_size = NULL;

	TEST_ASSERT((id & KVM_REG_ARCH_MASK) == KVM_REG_RISCV,
		    "%s: KVM_REG_RISCV missing in reg id: 0x%llx", prefix, id);

	switch (id & KVM_REG_SIZE_MASK) {
	case KVM_REG_SIZE_U32:
		reg_size = "KVM_REG_SIZE_U32";
		break;
	case KVM_REG_SIZE_U64:
		reg_size = "KVM_REG_SIZE_U64";
		break;
	case KVM_REG_SIZE_U128:
		reg_size = "KVM_REG_SIZE_U128";
		break;
	default:
		printf("\tKVM_REG_RISCV | (%lld << KVM_REG_SIZE_SHIFT) | 0x%llx /* UNKNOWN */,\n",
		       (id & KVM_REG_SIZE_MASK) >> KVM_REG_SIZE_SHIFT, id & ~REG_MASK);
		return;
	}

	switch (id & KVM_REG_RISCV_TYPE_MASK) {
	case KVM_REG_RISCV_CONFIG:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_CONFIG | %s,\n",
				reg_size, config_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_CORE:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_CORE | %s,\n",
				reg_size, core_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_CSR:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_CSR | %s,\n",
				reg_size, csr_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_TIMER:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_TIMER | %s,\n",
				reg_size, timer_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_FP_F:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_FP_F | %s,\n",
				reg_size, fp_f_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_FP_D:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_FP_D | %s,\n",
				reg_size, fp_d_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_ISA_EXT:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_ISA_EXT | %s,\n",
				reg_size, isa_ext_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_SBI_EXT:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_SBI_EXT | %s,\n",
				reg_size, sbi_ext_id_to_str(prefix, id));
		break;
	case KVM_REG_RISCV_SBI_STATE:
		printf("\tKVM_REG_RISCV | %s | KVM_REG_RISCV_SBI_STATE | %s,\n",
				reg_size, sbi_id_to_str(prefix, id));
		break;
	default:
		printf("\tKVM_REG_RISCV | %s | 0x%llx /* UNKNOWN */,\n",
				reg_size, id & ~REG_MASK);
		return;
	}
}

/*
 * The current blessed list was primed with the output of kernel version
 * v6.5-rc3 and then later updated with new registers.
 */
static __u64 base_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CONFIG | KVM_REG_RISCV_CONFIG_REG(isa),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CONFIG | KVM_REG_RISCV_CONFIG_REG(mvendorid),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CONFIG | KVM_REG_RISCV_CONFIG_REG(marchid),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CONFIG | KVM_REG_RISCV_CONFIG_REG(mimpid),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CONFIG | KVM_REG_RISCV_CONFIG_REG(satp_mode),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.pc),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.ra),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.sp),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.gp),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.tp),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.t0),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.t1),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.t2),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s0),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s1),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a0),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a1),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a2),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a3),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a4),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a5),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a6),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.a7),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s2),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s3),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s4),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s5),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s6),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s7),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s8),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s9),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s10),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.s11),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.t3),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.t4),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.t5),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(regs.t6),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CORE | KVM_REG_RISCV_CORE_REG(mode),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(sstatus),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(sie),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(stvec),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(sscratch),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(sepc),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(scause),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(stval),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(sip),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(satp),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(scounteren),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_GENERAL | KVM_REG_RISCV_CSR_REG(senvcfg),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_TIMER | KVM_REG_RISCV_TIMER_REG(frequency),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_TIMER | KVM_REG_RISCV_TIMER_REG(time),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_TIMER | KVM_REG_RISCV_TIMER_REG(compare),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_TIMER | KVM_REG_RISCV_TIMER_REG(state),
};

/*
 * The skips_set list registers that should skip set test.
 *  - KVM_REG_RISCV_TIMER_REG(state): set would fail if it was not initialized properly.
 */
static __u64 base_skips_set[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_TIMER | KVM_REG_RISCV_TIMER_REG(state),
};

static __u64 sbi_base_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_V01,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_TIME,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_IPI,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_RFENCE,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_SRST,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_HSM,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_EXPERIMENTAL,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_VENDOR,
};

static __u64 sbi_sta_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE | KVM_RISCV_SBI_EXT_STA,
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_STATE | KVM_REG_RISCV_SBI_STA | KVM_REG_RISCV_SBI_STA_REG(shmem_lo),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_SBI_STATE | KVM_REG_RISCV_SBI_STA | KVM_REG_RISCV_SBI_STA_REG(shmem_hi),
};

static __u64 zicbom_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CONFIG | KVM_REG_RISCV_CONFIG_REG(zicbom_block_size),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZICBOM,
};

static __u64 zicboz_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CONFIG | KVM_REG_RISCV_CONFIG_REG(zicboz_block_size),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_ZICBOZ,
};

static __u64 aia_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(siselect),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio1),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio2),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(sieh),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(siph),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio1h),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_AIA | KVM_REG_RISCV_CSR_AIA_REG(iprio2h),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SSAIA,
};

static __u64 smstateen_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_CSR | KVM_REG_RISCV_CSR_SMSTATEEN | KVM_REG_RISCV_CSR_SMSTATEEN_REG(sstateen0),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_SMSTATEEN,
};

static __u64 fp_f_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[0]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[1]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[2]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[3]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[4]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[5]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[6]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[7]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[8]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[9]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[10]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[11]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[12]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[13]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[14]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[15]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[16]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[17]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[18]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[19]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[20]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[21]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[22]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[23]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[24]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[25]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[26]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[27]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[28]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[29]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[30]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(f[31]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_F | KVM_REG_RISCV_FP_F_REG(fcsr),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_F,
};

static __u64 fp_d_regs[] = {
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[0]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[1]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[2]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[3]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[4]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[5]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[6]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[7]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[8]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[9]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[10]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[11]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[12]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[13]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[14]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[15]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[16]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[17]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[18]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[19]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[20]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[21]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[22]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[23]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[24]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[25]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[26]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[27]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[28]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[29]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[30]),
	KVM_REG_RISCV | KVM_REG_SIZE_U64 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(f[31]),
	KVM_REG_RISCV | KVM_REG_SIZE_U32 | KVM_REG_RISCV_FP_D | KVM_REG_RISCV_FP_D_REG(fcsr),
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG | KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE | KVM_RISCV_ISA_EXT_D,
};

#define SUBLIST_BASE \
	{"base", .regs = base_regs, .regs_n = ARRAY_SIZE(base_regs), \
	 .skips_set = base_skips_set, .skips_set_n = ARRAY_SIZE(base_skips_set),}
#define SUBLIST_SBI_BASE \
	{"sbi-base", .feature_type = VCPU_FEATURE_SBI_EXT, .feature = KVM_RISCV_SBI_EXT_V01, \
	 .regs = sbi_base_regs, .regs_n = ARRAY_SIZE(sbi_base_regs),}
#define SUBLIST_SBI_STA \
	{"sbi-sta", .feature_type = VCPU_FEATURE_SBI_EXT, .feature = KVM_RISCV_SBI_EXT_STA, \
	 .regs = sbi_sta_regs, .regs_n = ARRAY_SIZE(sbi_sta_regs),}
#define SUBLIST_ZICBOM \
	{"zicbom", .feature = KVM_RISCV_ISA_EXT_ZICBOM, .regs = zicbom_regs, .regs_n = ARRAY_SIZE(zicbom_regs),}
#define SUBLIST_ZICBOZ \
	{"zicboz", .feature = KVM_RISCV_ISA_EXT_ZICBOZ, .regs = zicboz_regs, .regs_n = ARRAY_SIZE(zicboz_regs),}
#define SUBLIST_AIA \
	{"aia", .feature = KVM_RISCV_ISA_EXT_SSAIA, .regs = aia_regs, .regs_n = ARRAY_SIZE(aia_regs),}
#define SUBLIST_SMSTATEEN \
	{"smstateen", .feature = KVM_RISCV_ISA_EXT_SMSTATEEN, .regs = smstateen_regs, .regs_n = ARRAY_SIZE(smstateen_regs),}
#define SUBLIST_FP_F \
	{"fp_f", .feature = KVM_RISCV_ISA_EXT_F, .regs = fp_f_regs, \
		.regs_n = ARRAY_SIZE(fp_f_regs),}
#define SUBLIST_FP_D \
	{"fp_d", .feature = KVM_RISCV_ISA_EXT_D, .regs = fp_d_regs, \
		.regs_n = ARRAY_SIZE(fp_d_regs),}

#define KVM_ISA_EXT_SIMPLE_CONFIG(ext, extu)			\
static __u64 regs_##ext[] = {					\
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG |			\
	KVM_REG_RISCV_ISA_EXT | KVM_REG_RISCV_ISA_SINGLE |	\
	KVM_RISCV_ISA_EXT_##extu,				\
};								\
static struct vcpu_reg_list config_##ext = {			\
	.sublists = {						\
		SUBLIST_BASE,					\
		{						\
			.name = #ext,				\
			.feature = KVM_RISCV_ISA_EXT_##extu,	\
			.regs = regs_##ext,			\
			.regs_n = ARRAY_SIZE(regs_##ext),	\
		},						\
		{0},						\
	},							\
}								\

#define KVM_SBI_EXT_SIMPLE_CONFIG(ext, extu)			\
static __u64 regs_sbi_##ext[] = {				\
	KVM_REG_RISCV | KVM_REG_SIZE_ULONG |			\
	KVM_REG_RISCV_SBI_EXT | KVM_REG_RISCV_SBI_SINGLE |	\
	KVM_RISCV_SBI_EXT_##extu,				\
};								\
static struct vcpu_reg_list config_sbi_##ext = {		\
	.sublists = {						\
		SUBLIST_BASE,					\
		{						\
			.name = "sbi-"#ext,			\
			.feature_type = VCPU_FEATURE_SBI_EXT,	\
			.feature = KVM_RISCV_SBI_EXT_##extu,	\
			.regs = regs_sbi_##ext,			\
			.regs_n = ARRAY_SIZE(regs_sbi_##ext),	\
		},						\
		{0},						\
	},							\
}								\

#define KVM_ISA_EXT_SUBLIST_CONFIG(ext, extu)			\
static struct vcpu_reg_list config_##ext = {			\
	.sublists = {						\
		SUBLIST_BASE,					\
		SUBLIST_##extu,					\
		{0},						\
	},							\
}								\

#define KVM_SBI_EXT_SUBLIST_CONFIG(ext, extu)			\
static struct vcpu_reg_list config_sbi_##ext = {		\
	.sublists = {						\
		SUBLIST_BASE,					\
		SUBLIST_SBI_##extu,				\
		{0},						\
	},							\
}								\

/* Note: The below list is alphabetically sorted. */

KVM_SBI_EXT_SUBLIST_CONFIG(base, BASE);
KVM_SBI_EXT_SUBLIST_CONFIG(sta, STA);
KVM_SBI_EXT_SIMPLE_CONFIG(pmu, PMU);
KVM_SBI_EXT_SIMPLE_CONFIG(dbcn, DBCN);

KVM_ISA_EXT_SUBLIST_CONFIG(aia, AIA);
KVM_ISA_EXT_SUBLIST_CONFIG(fp_f, FP_F);
KVM_ISA_EXT_SUBLIST_CONFIG(fp_d, FP_D);
KVM_ISA_EXT_SIMPLE_CONFIG(h, H);
KVM_ISA_EXT_SUBLIST_CONFIG(smstateen, SMSTATEEN);
KVM_ISA_EXT_SIMPLE_CONFIG(sscofpmf, SSCOFPMF);
KVM_ISA_EXT_SIMPLE_CONFIG(sstc, SSTC);
KVM_ISA_EXT_SIMPLE_CONFIG(svinval, SVINVAL);
KVM_ISA_EXT_SIMPLE_CONFIG(svnapot, SVNAPOT);
KVM_ISA_EXT_SIMPLE_CONFIG(svpbmt, SVPBMT);
KVM_ISA_EXT_SIMPLE_CONFIG(zacas, ZACAS);
KVM_ISA_EXT_SIMPLE_CONFIG(zba, ZBA);
KVM_ISA_EXT_SIMPLE_CONFIG(zbb, ZBB);
KVM_ISA_EXT_SIMPLE_CONFIG(zbc, ZBC);
KVM_ISA_EXT_SIMPLE_CONFIG(zbkb, ZBKB);
KVM_ISA_EXT_SIMPLE_CONFIG(zbkc, ZBKC);
KVM_ISA_EXT_SIMPLE_CONFIG(zbkx, ZBKX);
KVM_ISA_EXT_SIMPLE_CONFIG(zbs, ZBS);
KVM_ISA_EXT_SIMPLE_CONFIG(zfa, ZFA);
KVM_ISA_EXT_SIMPLE_CONFIG(zfh, ZFH);
KVM_ISA_EXT_SIMPLE_CONFIG(zfhmin, ZFHMIN);
KVM_ISA_EXT_SUBLIST_CONFIG(zicbom, ZICBOM);
KVM_ISA_EXT_SUBLIST_CONFIG(zicboz, ZICBOZ);
KVM_ISA_EXT_SIMPLE_CONFIG(zicntr, ZICNTR);
KVM_ISA_EXT_SIMPLE_CONFIG(zicond, ZICOND);
KVM_ISA_EXT_SIMPLE_CONFIG(zicsr, ZICSR);
KVM_ISA_EXT_SIMPLE_CONFIG(zifencei, ZIFENCEI);
KVM_ISA_EXT_SIMPLE_CONFIG(zihintntl, ZIHINTNTL);
KVM_ISA_EXT_SIMPLE_CONFIG(zihintpause, ZIHINTPAUSE);
KVM_ISA_EXT_SIMPLE_CONFIG(zihpm, ZIHPM);
KVM_ISA_EXT_SIMPLE_CONFIG(zknd, ZKND);
KVM_ISA_EXT_SIMPLE_CONFIG(zkne, ZKNE);
KVM_ISA_EXT_SIMPLE_CONFIG(zknh, ZKNH);
KVM_ISA_EXT_SIMPLE_CONFIG(zkr, ZKR);
KVM_ISA_EXT_SIMPLE_CONFIG(zksed, ZKSED);
KVM_ISA_EXT_SIMPLE_CONFIG(zksh, ZKSH);
KVM_ISA_EXT_SIMPLE_CONFIG(zkt, ZKT);
KVM_ISA_EXT_SIMPLE_CONFIG(ztso, ZTSO);
KVM_ISA_EXT_SIMPLE_CONFIG(zvbb, ZVBB);
KVM_ISA_EXT_SIMPLE_CONFIG(zvbc, ZVBC);
KVM_ISA_EXT_SIMPLE_CONFIG(zvfh, ZVFH);
KVM_ISA_EXT_SIMPLE_CONFIG(zvfhmin, ZVFHMIN);
KVM_ISA_EXT_SIMPLE_CONFIG(zvkb, ZVKB);
KVM_ISA_EXT_SIMPLE_CONFIG(zvkg, ZVKG);
KVM_ISA_EXT_SIMPLE_CONFIG(zvkned, ZVKNED);
KVM_ISA_EXT_SIMPLE_CONFIG(zvknha, ZVKNHA);
KVM_ISA_EXT_SIMPLE_CONFIG(zvknhb, ZVKNHB);
KVM_ISA_EXT_SIMPLE_CONFIG(zvksed, ZVKSED);
KVM_ISA_EXT_SIMPLE_CONFIG(zvksh, ZVKSH);
KVM_ISA_EXT_SIMPLE_CONFIG(zvkt, ZVKT);

struct vcpu_reg_list *vcpu_configs[] = {
	&config_sbi_base,
	&config_sbi_sta,
	&config_sbi_pmu,
	&config_sbi_dbcn,
	&config_aia,
	&config_fp_f,
	&config_fp_d,
	&config_h,
	&config_smstateen,
	&config_sscofpmf,
	&config_sstc,
	&config_svinval,
	&config_svnapot,
	&config_svpbmt,
	&config_zacas,
	&config_zba,
	&config_zbb,
	&config_zbc,
	&config_zbkb,
	&config_zbkc,
	&config_zbkx,
	&config_zbs,
	&config_zfa,
	&config_zfh,
	&config_zfhmin,
	&config_zicbom,
	&config_zicboz,
	&config_zicntr,
	&config_zicond,
	&config_zicsr,
	&config_zifencei,
	&config_zihintntl,
	&config_zihintpause,
	&config_zihpm,
	&config_zknd,
	&config_zkne,
	&config_zknh,
	&config_zkr,
	&config_zksed,
	&config_zksh,
	&config_zkt,
	&config_ztso,
	&config_zvbb,
	&config_zvbc,
	&config_zvfh,
	&config_zvfhmin,
	&config_zvkb,
	&config_zvkg,
	&config_zvkned,
	&config_zvknha,
	&config_zvknhb,
	&config_zvksed,
	&config_zvksh,
	&config_zvkt,
};
int vcpu_configs_n = ARRAY_SIZE(vcpu_configs);
