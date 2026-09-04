// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025, 2026 Arm Ltd.
 */

#include <kvm/arm_vgic.h>

#include <linux/kvm_host.h>
#include <linux/bitops.h>
#include <linux/irqchip/arm-vgic-info.h>
#include <linux/irqdomain.h>

#include "vgic-v5-tables.h"
#include "vgic.h"

#define ppi_caps	kvm_vgic_global_state.vgic_v5_ppi_caps
#define irs_caps	kvm_vgic_global_state.vgic_v5_irs_caps

/*
 * Not all PPIs are guaranteed to be implemented for GICv5. Deterermine which
 * ones are, and generate a mask.
 */
static void vgic_v5_get_implemented_ppis(void)
{
	/*
	 * If we have KVM, we have EL2, which means that we have support for the
	 * EL1 and EL2 Physical & Virtual timers.
	 */
	__set_bit(GICV5_ARCH_PPI_CNTHP, ppi_caps.impl_ppi_mask);
	__set_bit(GICV5_ARCH_PPI_CNTV, ppi_caps.impl_ppi_mask);
	__set_bit(GICV5_ARCH_PPI_CNTHV, ppi_caps.impl_ppi_mask);
	__set_bit(GICV5_ARCH_PPI_CNTP, ppi_caps.impl_ppi_mask);

	/* The SW_PPI should be available */
	__set_bit(GICV5_ARCH_PPI_SW_PPI, ppi_caps.impl_ppi_mask);

	/* The PMUIRQ is available if we have the PMU */
	__assign_bit(GICV5_ARCH_PPI_PMUIRQ, ppi_caps.impl_ppi_mask, system_supports_pmuv3());
}

static u32 irs_readl_relaxed(const u32 reg_offset)
{
	return readl_relaxed(irs_caps.irs_base + reg_offset);
}

static void vgic_v5_irs_cache_id_regs(const struct gic_kvm_info *info)
{
	irs_caps.irs_base = info->gicv5_irs.base;
	irs_caps.non_coherent = info->gicv5_irs.non_coherent;

	irs_caps.idr2 = irs_readl_relaxed(GICV5_IRS_IDR2);
	irs_caps.idr3 = irs_readl_relaxed(GICV5_IRS_IDR3);
	irs_caps.idr4 = irs_readl_relaxed(GICV5_IRS_IDR4);
}

/*
 * Probe for a vGICv5 compatible interrupt controller, returning 0 on success.
 */
int vgic_v5_probe(const struct gic_kvm_info *info)
{
	bool v5_registered = false;
	int ret;

	kvm_vgic_global_state.type = VGIC_V5;

	kvm_vgic_global_state.vcpu_base = 0;
	kvm_vgic_global_state.vctrl_base = NULL;
	kvm_vgic_global_state.can_emulate_gicv2 = false;
	kvm_vgic_global_state.has_gicv4 = false;
	kvm_vgic_global_state.has_gicv4_1 = false;

	/*
	 * GICv5 is currently not supported in Protected mode. Skip the
	 * registration of GICv5 completely to make sure no guests can create a
	 * GICv5-based guest.
	 */
	if (is_protected_kvm_enabled()) {
		kvm_info("GICv5-based guests are not supported with pKVM\n");
		goto skip_v5;
	}

	vgic_v5_irs_cache_id_regs(info);
	vgic_v5_get_implemented_ppis();

	kvm_vgic_global_state.max_gic_vcpus = min(vgic_v5_irs_max_vpes(&irs_caps),
						  VGIC_V5_MAX_CPUS);

	ret = kvm_register_vgic_device(KVM_DEV_TYPE_ARM_VGIC_V5);
	if (ret) {
		kvm_err("Cannot register GICv5 KVM device.\n");
		goto skip_v5;
	}

	v5_registered = true;
	kvm_info("GCIE system register CPU interface\n");

skip_v5:
	/* If we don't support the GICv3 compat mode we're done. */
	if (!cpus_have_final_cap(ARM64_HAS_GICV5_LEGACY)) {
		if (!v5_registered)
			return -ENODEV;
		return 0;
	}

	kvm_vgic_global_state.has_gcie_v3_compat = true;

	/*
	 * The ListRegs field is 5 bits, but there is an architectural
	 * maximum of 16 list registers. Just ignore bit 4...
	 */
	kvm_vgic_global_state.nr_lr = (vgic_ich_vtr() & 0xf) + 1;

	ret = kvm_register_vgic_device(KVM_DEV_TYPE_ARM_VGIC_V3);
	if (ret) {
		kvm_err("Cannot register GICv3-legacy KVM device.\n");
		return ret;
	}

	/* We potentially limit the max VCPUs further than we need to here */
	kvm_vgic_global_state.max_gic_vcpus = min(VGIC_V3_MAX_CPUS,
						  VGIC_V5_MAX_CPUS);

	static_branch_enable(&kvm_vgic_global_state.gicv3_cpuif);
	kvm_info("GCIE legacy system register CPU interface\n");

	vgic_v3_enable_cpuif_traps();

	return 0;
}

static int vgic_v5_db_set_vcpu_affinity(struct irq_data *data, void *vcpu_info)
{
	enum gicv5_vcpu_cmd *cmd = vcpu_info;

	guard(raw_spinlock_irqsave)(&vgic_v5_irs_lock);

	switch (*cmd) {
	case VMT_L2_MAP:
	case VMTE_MAKE_VALID:
	case VMTE_MAKE_INVALID:
		/* Not yet implemented */
	default:
		return -EINVAL;
	}
}

/*
 * This set of irq_chip functions is specific for doorbells.
 */
static const struct irq_chip vgic_v5_db_irq_chip = {
	.name = "GICv5-DB",
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_eoi = irq_chip_eoi_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_get_irqchip_state = irq_chip_get_parent_state,
	.irq_set_irqchip_state = irq_chip_set_parent_state,
	.irq_set_vcpu_affinity = vgic_v5_db_set_vcpu_affinity,
	.flags = IRQCHIP_SET_TYPE_MASKED | IRQCHIP_SKIP_SET_WAKE |
		 IRQCHIP_MASK_ON_SUSPEND,
};

static void vgic_v5_irq_db_domain_free(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(domain, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}

	irq_domain_free_irqs_parent(domain, virq, nr_irqs);
}

static int vgic_v5_irq_db_domain_alloc(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs,
				       void *arg)
{
	const struct irq_chip *chip = &vgic_v5_db_irq_chip;
	struct vgic_v5_vm *vm = arg;
	struct irq_data *irqd;
	int ret;

	if (!vm) {
		kvm_err("invalid parameter for doorbell irq allocation\n");
		return -EINVAL;
	}

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, NULL);
	if (ret)
		return ret;

	for (int i = 0; i < nr_irqs; i++) {
		irq_domain_set_hwirq_and_chip(domain, virq + i, i, chip,
					      domain->host_data);
		irqd = irq_desc_get_irq_data(irq_to_desc(virq + i));
		irqd_set_single_target(irqd);
	}

	return 0;
}

static const struct irq_domain_ops vgic_v5_irq_db_domain_ops = {
	.alloc = vgic_v5_irq_db_domain_alloc,
	.free = vgic_v5_irq_db_domain_free,
};

static int vgic_v5_create_per_vm_domain(struct kvm *kvm)
{
	struct vgic_v5_vm *vm = &kvm->arch.vgic.gicv5_vm;
	int nr_vcpus = atomic_read(&kvm->online_vcpus);
	int id = task_pid_nr(current);
	int ret, db_virq = 0;

	if (!gicv5_global_data.lpi_domain) {
		kvm_err("LPI domain uninitialized, can't set up KVM Doorbells\n");
		return -ENODEV;
	}

	vm->fwnode = irq_domain_alloc_named_id_fwnode("GICv5-vpe-db", id);
	if (!vm->fwnode)
		return -ENOMEM;

	/*
	 * KVM per-VM VPE DB domain; child of LPI domain; only ever handles
	 * doorbells. We know how many doorbells we have, and therefore we
	 * create a linear domain.
	 */
	vm->domain = irq_domain_create_hierarchy(gicv5_global_data.lpi_domain,
						 0, nr_vcpus, vm->fwnode,
						 &vgic_v5_irq_db_domain_ops, vm);
	if (!vm->domain) {
		ret = -ENOMEM;
		goto err;
	}

	db_virq = irq_domain_alloc_irqs(vm->domain, nr_vcpus, NUMA_NO_NODE, vm);
	if (db_virq <= 0) {
		ret = db_virq;
		goto err;
	}

	kvm->arch.vgic.gicv5_vm.vpe_db_base = db_virq;

	return 0;

err:
	if (vm->domain)
		irq_domain_remove(vm->domain);
	if (vm->fwnode)
		irq_domain_free_fwnode(vm->fwnode);

	kvm->arch.vgic.gicv5_vm.vpe_db_base = 0;
	vm->domain = NULL;
	vm->fwnode = NULL;

	return ret;
}

static void vgic_v5_teardown_per_vm_domain(struct vgic_v5_vm *vm)
{
	if (!vm->domain)
		return;

	if (vm->vpe_db_base) {
		irq_domain_free_irqs(vm->vpe_db_base, vm->domain->revmap_size);
		vm->vpe_db_base = 0;
	}

	irq_domain_remove(vm->domain);
	irq_domain_free_fwnode(vm->fwnode);
	vm->domain = NULL;
	vm->fwnode = NULL;
}

void vgic_v5_reset(struct kvm_vcpu *vcpu)
{
	/*
	 * We always present 16-bits of ID space to the guest, irrespective of
	 * the host allowing more.
	 */
	vcpu->arch.vgic_cpu.num_id_bits = ICC_IDR0_EL1_ID_BITS_16BITS;

	/*
	 * The GICv5 architeture only supports 5-bits of priority in the
	 * CPUIF (but potentially fewer in the IRS).
	 */
	vcpu->arch.vgic_cpu.num_pri_bits = 5;
}

void vgic_v5_teardown(struct kvm *kvm)
{
	vgic_v5_teardown_per_vm_domain(&kvm->arch.vgic.gicv5_vm);
}

int vgic_v5_init(struct kvm *kvm)
{
	struct kvm_vcpu *vcpu;
	unsigned long idx;
	int ret;

	if (vgic_initialized(kvm))
		return 0;

	kvm_for_each_vcpu(idx, vcpu, kvm) {
		if (vcpu_has_nv(vcpu)) {
			kvm_err("Nested GICv5 VMs are currently unsupported\n");
			return -EINVAL;
		}
	}

	ret = vgic_v5_create_per_vm_domain(kvm);
	if (ret)
		return ret;

	/* We only allow userspace to drive the SW_PPI, if it is implemented. */
	bitmap_zero(kvm->arch.vgic.gicv5_vm.userspace_ppis,
		    VGIC_V5_NR_PRIVATE_IRQS);
	__set_bit(GICV5_ARCH_PPI_SW_PPI, kvm->arch.vgic.gicv5_vm.userspace_ppis);
	bitmap_and(kvm->arch.vgic.gicv5_vm.userspace_ppis,
		   kvm->arch.vgic.gicv5_vm.userspace_ppis,
		   ppi_caps.impl_ppi_mask, VGIC_V5_NR_PRIVATE_IRQS);

	return 0;
}

int vgic_v5_map_resources(struct kvm *kvm)
{
	if (!vgic_initialized(kvm))
		return -EBUSY;

	return 0;
}

int vgic_v5_finalize_ppi_state(struct kvm *kvm)
{
	struct kvm_vcpu *vcpu0;
	int i;

	if (!vgic_is_v5(kvm))
		return 0;

	guard(mutex)(&kvm->arch.config_lock);

	/*
	 * If SW_PPI has been advertised, then we know we already
	 * initialised the whole thing, and we can return early. Yes,
	 * this is pretty hackish as far as state tracking goes...
	 */
	if (test_bit(GICV5_ARCH_PPI_SW_PPI, kvm->arch.vgic.gicv5_vm.vgic_ppi_mask))
		return 0;

	/* The PPI state for all VCPUs should be the same. Pick the first. */
	vcpu0 = kvm_get_vcpu(kvm, 0);

	bitmap_zero(kvm->arch.vgic.gicv5_vm.vgic_ppi_mask, VGIC_V5_NR_PRIVATE_IRQS);
	bitmap_zero(kvm->arch.vgic.gicv5_vm.vgic_ppi_hmr, VGIC_V5_NR_PRIVATE_IRQS);

	for_each_set_bit(i, ppi_caps.impl_ppi_mask, VGIC_V5_NR_PRIVATE_IRQS) {
		const u32 intid = vgic_v5_make_ppi(i);
		struct vgic_irq *irq;

		irq = vgic_get_vcpu_irq(vcpu0, intid);

		/* Expose PPIs with an owner or the SW_PPI, only */
		scoped_guard(raw_spinlock_irqsave, &irq->irq_lock) {
			if (irq->owner || i == GICV5_ARCH_PPI_SW_PPI) {
				__set_bit(i, kvm->arch.vgic.gicv5_vm.vgic_ppi_mask);
				__assign_bit(i, kvm->arch.vgic.gicv5_vm.vgic_ppi_hmr,
					     irq->config == VGIC_CONFIG_LEVEL);
			}
		}

		vgic_put_irq(vcpu0->kvm, irq);
	}

	return 0;
}

static u32 vgic_v5_get_effective_priority_mask(struct kvm_vcpu *vcpu)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;
	u32 highest_ap, priority_mask, apr;

	/*
	 * If the guest's CPU has not opted to receive interrupts, then the
	 * effective running priority is the highest priority. Just return 0
	 * (the highest priority).
	 */
	if (!FIELD_GET(FEAT_GCIE_ICH_VMCR_EL2_EN, cpu_if->vgic_vmcr))
		return 0;

	/*
	 * Counting the number of trailing zeros gives the current active
	 * priority. Explicitly use the 32-bit version here as we have 32
	 * priorities. 32 then means that there are no active priorities.
	 */
	apr = cpu_if->vgic_apr;
	highest_ap = apr ? __builtin_ctz(apr) : 32;

	/*
	 * An interrupt is of sufficient priority if it is equal to or
	 * greater than the priority mask. Add 1 to the priority mask
	 * (i.e., lower priority) to match the APR logic before taking
	 * the min. This gives us the lowest priority that is masked.
	 */
	priority_mask = FIELD_GET(FEAT_GCIE_ICH_VMCR_EL2_VPMR, cpu_if->vgic_vmcr);

	return min(highest_ap, priority_mask + 1);
}

/*
 * For GICv5, the PPIs are mostly directly managed by the hardware. We (the
 * hypervisor) handle the pending, active, enable state save/restore, but
 * don't need the PPIs to be queued on a per-VCPU AP list. Therefore,
 * unlock, kick the vcpu and return.
 */
bool vgic_v5_ppi_queue_irq_unlock(struct kvm *kvm, struct vgic_irq *irq,
				  unsigned long flags)
	__releases(&irq->irq_lock)
{
	struct kvm_vcpu *vcpu;

	lockdep_assert_held(&irq->irq_lock);

	vcpu = irq->target_vcpu;

	raw_spin_unlock_irqrestore(&irq->irq_lock, flags);

	/* Directly kick the target VCPU to make sure it sees the IRQ */
	kvm_make_request(KVM_REQ_IRQ_PENDING, vcpu);
	kvm_vcpu_kick(vcpu);

	return true;
}

/*
 * Sets/clears the corresponding bit in the ICH_PPI_DVIR register.
 */
void vgic_v5_set_ppi_dvi(struct kvm_vcpu *vcpu, struct vgic_irq *irq, bool dvi)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;
	u32 ppi;

	lockdep_assert_held(&irq->irq_lock);

	ppi = vgic_v5_get_hwirq_id(irq->intid);
	assign_bit(ppi, cpu_if->vgic_ppi_dvir, dvi);
}

static const struct irq_ops vgic_v5_ppi_irq_ops = {
	.queue_irq_unlock = vgic_v5_ppi_queue_irq_unlock,
	.set_direct_injection = vgic_v5_set_ppi_dvi,
};

void vgic_v5_set_ppi_ops(struct kvm_vcpu *vcpu, u32 vintid)
{
	kvm_vgic_set_irq_ops(vcpu, vintid, &vgic_v5_ppi_irq_ops);
}

/*
 * Sync back the PPI priorities to the vgic_irq shadow state for any interrupts
 * exposed to the guest (skipping all others).
 */
static void vgic_v5_sync_ppi_priorities(struct kvm_vcpu *vcpu)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;
	u64 priorityr;
	int i;

	/*
	 * We have up to 16 PPI Priority regs, but only have a few interrupts
	 * that the guest is allowed to use. Limit our sync of PPI priorities to
	 * those actually exposed to the guest by first iterating over the mask
	 * of exposed PPIs.
	 */
	for_each_visible_v5_ppi(i, vcpu->kvm) {
		u32 intid = vgic_v5_make_ppi(i);
		struct vgic_irq *irq;
		int pri_idx, pri_reg, pri_bit;
		u8 priority;

		/*
		 * Determine which priority register and the field within it to
		 * extract.
		 */
		pri_reg = i / 8;
		pri_idx = i % 8;
		pri_bit = pri_idx * 8;

		priorityr = cpu_if->vgic_ppi_priorityr[pri_reg];
		priority = field_get(GENMASK(pri_bit + 4, pri_bit), priorityr);

		irq = vgic_get_vcpu_irq(vcpu, intid);

		scoped_guard(raw_spinlock_irqsave, &irq->irq_lock)
			irq->priority = priority;

		vgic_put_irq(vcpu->kvm, irq);
	}
}

bool vgic_v5_has_pending_ppi(struct kvm_vcpu *vcpu)
{
	unsigned int priority_mask;
	int i;

	priority_mask = vgic_v5_get_effective_priority_mask(vcpu);

	/*
	 * If the combined priority mask is 0, nothing can be signalled! In the
	 * case where the guest has disabled interrupt delivery for the vcpu
	 * (via ICV_CR0_EL1.EN->ICH_VMCR_EL2.EN), we calculate the priority mask
	 * as 0 too (the highest possible priority).
	 */
	if (!priority_mask)
		return false;

	for_each_visible_v5_ppi(i, vcpu->kvm) {
		u32 intid = vgic_v5_make_ppi(i);
		bool has_pending = false;
		struct vgic_irq *irq;

		irq = vgic_get_vcpu_irq(vcpu, intid);

		scoped_guard(raw_spinlock_irqsave, &irq->irq_lock)
			if (irq->enabled && irq->priority < priority_mask)
				has_pending = irq->hw ? vgic_get_phys_line_level(irq) : irq_is_pending(irq);

		vgic_put_irq(vcpu->kvm, irq);

		if (has_pending)
			return true;
	}

	return false;
}

/*
 * Detect any PPIs state changes, and propagate the state with KVM's
 * shadow structures.
 */
void vgic_v5_fold_ppi_state(struct kvm_vcpu *vcpu)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;
	unsigned long *activer, *pendr;
	int i;

	activer = host_data_ptr(vgic_v5_ppi_state)->activer_exit;
	pendr = host_data_ptr(vgic_v5_ppi_state)->pendr;

	for_each_visible_v5_ppi(i, vcpu->kvm) {
		u32 intid = vgic_v5_make_ppi(i);
		struct vgic_irq *irq;

		irq = vgic_get_vcpu_irq(vcpu, intid);

		scoped_guard(raw_spinlock_irqsave, &irq->irq_lock) {
			irq->active = test_bit(i, activer);

			/* This is an OR to avoid losing incoming edges! */
			if (irq->config == VGIC_CONFIG_EDGE)
				irq->pending_latch |= test_bit(i, pendr);
		}

		vgic_put_irq(vcpu->kvm, irq);
	}

	/*
	 * Re-inject the exit state as entry state next time!
	 *
	 * Note that the write of the Enable state is trapped, and hence there
	 * is nothing to explcitly sync back here as we already have the latest
	 * copy by definition.
	 */
	bitmap_copy(cpu_if->vgic_ppi_activer, activer, VGIC_V5_NR_PRIVATE_IRQS);
}

void vgic_v5_flush_ppi_state(struct kvm_vcpu *vcpu)
{
	DECLARE_BITMAP(pendr, VGIC_V5_NR_PRIVATE_IRQS);
	int i;

	/*
	 * Time to enter the guest - we first need to build the guest's
	 * ICC_PPI_PENDRx_EL1, however.
	 */
	bitmap_zero(pendr, VGIC_V5_NR_PRIVATE_IRQS);
	for_each_visible_v5_ppi(i, vcpu->kvm) {
		u32 intid = vgic_v5_make_ppi(i);
		struct vgic_irq *irq;

		irq = vgic_get_vcpu_irq(vcpu, intid);

		scoped_guard(raw_spinlock_irqsave, &irq->irq_lock) {
			__assign_bit(i, pendr, irq_is_pending(irq));
			if (irq->config == VGIC_CONFIG_EDGE)
				irq->pending_latch = false;
		}

		vgic_put_irq(vcpu->kvm, irq);
	}

	/*
	 * Copy the shadow state to the pending reg that will be written to the
	 * ICH_PPI_PENDRx_EL2 regs. While the guest is running we track any
	 * incoming changes to the pending state in the vgic_irq structures. The
	 * incoming changes are merged with the outgoing changes on the return
	 * path.
	 */
	bitmap_copy(host_data_ptr(vgic_v5_ppi_state)->pendr, pendr,
		    VGIC_V5_NR_PRIVATE_IRQS);
}

void vgic_v5_load(struct kvm_vcpu *vcpu)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;

	/*
	 * On the WFI path, vgic_load is called a second time. The first is when
	 * scheduling in the vcpu thread again, and the second is when leaving
	 * WFI. Skip the second instance as it serves no purpose and just
	 * restores the same state again.
	 */
	if (cpu_if->gicv5_vpe.resident)
		return;

	kvm_call_hyp(__vgic_v5_restore_vmcr_apr, cpu_if);

	cpu_if->gicv5_vpe.resident = true;
}

void vgic_v5_put(struct kvm_vcpu *vcpu)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;

	/*
	 * Do nothing if we're not resident. This can happen in the WFI path
	 * where we do a vgic_put in the WFI path and again later when
	 * descheduling the thread. We risk losing VMCR state if we sync it
	 * twice, so instead return early in this case.
	 */
	if (!cpu_if->gicv5_vpe.resident)
		return;

	kvm_call_hyp(__vgic_v5_save_apr, cpu_if);

	cpu_if->gicv5_vpe.resident = false;

	/* The shadow priority is only updated on entering WFI */
	if (vcpu_get_flag(vcpu, IN_WFI))
		vgic_v5_sync_ppi_priorities(vcpu);
}

void vgic_v5_get_vmcr(struct kvm_vcpu *vcpu, struct vgic_vmcr *vmcrp)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;
	u64 vmcr = cpu_if->vgic_vmcr;

	vmcrp->en = FIELD_GET(FEAT_GCIE_ICH_VMCR_EL2_EN, vmcr);
	vmcrp->pmr = FIELD_GET(FEAT_GCIE_ICH_VMCR_EL2_VPMR, vmcr);
}

void vgic_v5_set_vmcr(struct kvm_vcpu *vcpu, struct vgic_vmcr *vmcrp)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;
	u64 vmcr;

	vmcr = FIELD_PREP(FEAT_GCIE_ICH_VMCR_EL2_VPMR, vmcrp->pmr) |
	       FIELD_PREP(FEAT_GCIE_ICH_VMCR_EL2_EN, vmcrp->en);

	cpu_if->vgic_vmcr = vmcr;
}

void vgic_v5_restore_state(struct kvm_vcpu *vcpu)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;

	__vgic_v5_restore_state(cpu_if);
	__vgic_v5_restore_ppi_state(cpu_if);
	dsb(sy);
}

void vgic_v5_save_state(struct kvm_vcpu *vcpu)
{
	struct vgic_v5_cpu_if *cpu_if = &vcpu->arch.vgic_cpu.vgic_v5;

	__vgic_v5_save_state(cpu_if);
	__vgic_v5_save_ppi_state(cpu_if);
	dsb(sy);
}
