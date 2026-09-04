// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025, 2026 Arm Ltd.
 */

#include <kvm/arm_vgic.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/xarray.h>
#include <asm/kvm_mmu.h>

#include "vgic.h"
#include "vgic-v5-tables.h"

#define irs_caps	kvm_vgic_global_state.vgic_v5_irs_caps

static struct vgic_v5_vmt *vmt_info;

/* Serialises IRS MMIO interaction (commands) and accesses to IRS tables. */
DEFINE_RAW_SPINLOCK(vgic_v5_irs_lock);

/* Serialises lazy installation of shared second-level VMTs. */
static DEFINE_MUTEX(vmt_l2_lock);

static DEFINE_XARRAY(vm_info);

/* Level 1 Virtual Machine Table Entry */
#define GICV5_VMTEL1E_VALID		BIT_ULL(0)
/* Note that there is no shift for the address by design */
#define GICV5_VMTEL1E_L2_ADDR		GENMASK(51, 12)

#define GICV5_VMTEL2E_SIZE		32ULL
/* An L2 table (two-level VMT) is ALWAYS 4kB! */
#define GICV5_VMT_L2_TABLE_SIZE		4096ULL
#define GICV5_VMT_L2_TABLE_ENTRIES	(GICV5_VMT_L2_TABLE_SIZE / GICV5_VMTEL2E_SIZE)

/*
 * As the L2 VMTE is a large data structure, we are splitting it into 4 parts.
 * We only mask and shift WITHIN each part for simplicity.
 */
/* First 64-bit chunk */
#define GICV5_VMTEL2E_VALID		BIT_ULL(0)
#define GICV5_VMTEL2E_VMD_ADDR_SHIFT	3ULL
#define GICV5_VMTEL2E_VMD_ADDR		GENMASK_ULL(55, 3)
/* Second 64-bit chunk */
#define GICV5_VMTEL2E_VPET_ADDR_SHIFT	3ULL
#define GICV5_VMTEL2E_VPET_ADDR		GENMASK_ULL(55, 3)
#define GICV5_VMTEL2E_VPE_ID_BITS	GENMASK_ULL(63, 59)
/* Third & fourth 64-bit chunks (the encodings are the same for each) */
#define GICV5_VMTEL2E_IST_VALID		BIT_ULL(0)
#define GICV5_VMTEL2E_IST_L2SZ		GENMASK_ULL(2, 1)
#define GICV5_VMTEL2E_IST_ADDR_SHIFT	6ULL
#define GICV5_VMTEL2E_IST_ADDR		GENMASK_ULL(55, 6)
#define GICV5_VMTEL2E_IST_ISTSZ		GENMASK_ULL(57, 56)
#define GICV5_VMTEL2E_IST_STRUCTURE	BIT_ULL(58)
#define GICV5_VMTEL2E_IST_ID_BITS	GENMASK_ULL(63, 59)

/* Virtual PE Table Entry */
#define GICV5_VPE_VALID			BIT_ULL(0)
/* Note that there is no shift for the address by design. */
#define GICV5_VPED_ADDR_SHIFT		3ULL
#define GICV5_VPED_ADDR			GENMASK_ULL(55, 3)

/*
 * Our IRS might be coherent or non-coherent. If coherent, we can just emit a
 * DSB to ensure that we're in sync. However, when non-coherent, we need to
 * manage our cached data explicitly.
 *
 * This helper is used to handle both coherent and non-coherent IRSes, and
 * handles all combinations of cleaning and invalidating to the PoC.
 */
static void vgic_v5_clean_inval(void *va, size_t size)
{
	unsigned long base = (unsigned long)va;

	dsb(ishst);

	if (kvm_vgic_global_state.vgic_v5_irs_caps.non_coherent)
		dcache_clean_inval_poc(base, base + size);
}

/*
 * Create a linear VM Table. Directly using the number of entries supplied as
 * the size of an L2 VMTE (32 bytes) guarantees that our allocation is aligned per
 * the GICv5 requirements for the IRS_VMT_BASER.
 */
static int vgic_v5_alloc_vmt_linear(unsigned int num_entries)
{
	vmt_info->linear.vmt_base = kzalloc_objs(*vmt_info->linear.vmt_base,
						 num_entries);
	if (!vmt_info->linear.vmt_base)
		return -ENOMEM;

	vgic_v5_clean_inval(vmt_info->linear.vmt_base,
			    num_entries * sizeof(struct vmtl2_entry));

	return 0;
}

/*
 * Allocate the first level of a two-level VM table. The second-level VM tables
 * are allocated on demand (by vgic_v5_alloc_l2_vmt()).
 */
static int vgic_v5_alloc_vmt_two_level(unsigned int num_entries)
{
	/*
	 * Each L2 VMT array is always 4k-sized (covering 128 VMs). This is
	 * mandated by the GICv5 specification (GICv5 EAC0 Specification rule
	 * D_LSPBK). Hence, round up the number of entries to be at least 128
	 * (or the next highest power of two as we give the HW the number of VM
	 * ID bits).
	 */
	if (num_entries < GICV5_VMT_L2_TABLE_ENTRIES)
		num_entries = GICV5_VMT_L2_TABLE_ENTRIES;
	num_entries = roundup_pow_of_two(num_entries);

	vmt_info->l2.num_l1_ents = (num_entries / GICV5_VMT_L2_TABLE_ENTRIES);
	vmt_info->l2.vmt_base = kzalloc_objs(*vmt_info->l2.vmt_base,
					     vmt_info->l2.num_l1_ents);
	if (!vmt_info->l2.vmt_base)
		return -ENOMEM;

	vmt_info->l2.l2ptrs = kzalloc_objs(*vmt_info->l2.l2ptrs,
					   vmt_info->l2.num_l1_ents,
					   GFP_KERNEL);
	if (!vmt_info->l2.l2ptrs) {
		kfree(vmt_info->l2.vmt_base);
		return -ENOMEM;
	}

	vgic_v5_clean_inval(vmt_info->l2.vmt_base,
			    vmt_info->l2.num_l1_ents * sizeof(vmtl1_entry));

	return 0;
}

/*
 * Allocate a second level VMT, if required. This can be called eagerly, and
 * will only perform the allocation if required.
 */
static int vgic_v5_alloc_l2_vmt(struct kvm *kvm)
{
	struct kvm_vcpu *vcpu0 = kvm_get_vcpu(kvm, 0);
	u32 vm_id = vgic_v5_vm_id(kvm);
	enum gicv5_vcpu_cmd cmd = VMT_L2_MAP;
	struct vmtl2_entry *l2_table;
	unsigned int l1_index;
	int ret;

	/* Nothing to do if we have linear tables! */
	if (!vmt_info->two_level)
		return 0;

	if (vm_id == VGIC_V5_VM_ID_INVAL)
		return -EINVAL;

	/*
	 * We have 4k-sized L2 tables - this is mandated by the spec for
	 * two-level VMTs (GICv5 EAC0 Specification rule D_LSPBK). This means
	 * that we have 128 entries per L1 VMTE.
	 */
	l1_index = vm_id / GICV5_VMT_L2_TABLE_ENTRIES;

	guard(mutex)(&vmt_l2_lock);

	/* Already valid? Great! */
	if (vmt_info->l2.l2ptrs[l1_index])
		return 0;

	l2_table = kzalloc_objs(*l2_table, GICV5_VMT_L2_TABLE_ENTRIES);
	if (!l2_table)
		return -ENOMEM;

	/* The VMT is shared between all VMs. */
	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		vgic_v5_clean_inval(l2_table, GICV5_VMT_L2_TABLE_SIZE);
		vgic_v5_clean_inval(vmt_info->l2.vmt_base + l1_index,
				    sizeof(vmtl1_entry));

		WRITE_ONCE(vmt_info->l2.vmt_base[l1_index],
			   cpu_to_le64(virt_to_phys(l2_table)));

		vgic_v5_clean_inval(vmt_info->l2.vmt_base + l1_index,
				    sizeof(vmtl1_entry));

	}

	/*
	 * VMAP in the L2 VMT via the IRS. We use any of the VM's CPUs as a
	 * conduit for interacting with the host's IRS. In the current case,
	 * this lets us resolve the VM ID to pass to the hardware.
	 */
	ret = irq_set_vcpu_affinity(vgic_v5_vpe_db(vcpu0), &cmd);

	/* We've failed to make the L2 VMT valid - things are very broken! */
	if (ret) {
		scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
			/* Remove the pointer from L1 table */
			WRITE_ONCE(vmt_info->l2.vmt_base[l1_index], 0);

			vgic_v5_clean_inval(vmt_info->l2.vmt_base + l1_index,
					    sizeof(vmtl1_entry));
		}

		kfree(l2_table);
		return ret;
	}

	vmt_info->l2.l2ptrs[l1_index] = l2_table;

	return 0;
}

/*
 * Allocate the top-level VMT. This can either be linear or two-level.
 */
int vgic_v5_vmt_allocate(unsigned int max_vpes)
{
	int ret;

	/* Allocate the tracking structure */
	vmt_info = kzalloc_obj(*vmt_info, GFP_KERNEL);
	if (!vmt_info)
		return -ENOMEM;

	ida_init(&vmt_info->vm_id_ida);
	vmt_info->max_vpes = max_vpes;
	vmt_info->vmd_size = vgic_v5_irs_vmd_size(&irs_caps);
	vmt_info->vped_size = vgic_v5_irs_vped_size(&irs_caps);
	vmt_info->two_level = vgic_v5_irs_two_level_vmt_support(&irs_caps);
	vmt_info->num_entries = vgic_v5_irs_max_vms(&irs_caps);

	if (vmt_info->two_level)
		ret = vgic_v5_alloc_vmt_two_level(vmt_info->num_entries);
	else
		ret = vgic_v5_alloc_vmt_linear(vmt_info->num_entries);

	/* If anything failed, free our tracking structure before returning */
	if (ret) {
		kfree(vmt_info);
		vmt_info = NULL;
	}

	return ret;
}

/*
 * Free the VMT and associated tracking structures. This isn't strictly expected
 * to be called in general operation, but instead exists for completeness.
 */
int vgic_v5_vmt_free(void)
{
	if (!vmt_info)
		return 0;

	if (!vmt_info->two_level) {
		kfree(vmt_info->linear.vmt_base);
	} else {
		/* Free the L2 tables; kfree(NULL) is safe */
		for (int i = 0; i < vmt_info->l2.num_l1_ents; ++i)
			kfree(vmt_info->l2.l2ptrs[i]);
		kfree(vmt_info->l2.l2ptrs);

		/* And now free the L1 table */
		kfree(vmt_info->l2.vmt_base);
	}

	ida_destroy(&vmt_info->vm_id_ida);
	kfree(vmt_info);
	vmt_info = NULL;

	return 0;
}

/*
 * Look up a VMT Entry by VM ID.
 */
static struct vmtl2_entry *vgic_v5_get_l2_vmte(u32 vm_id)
{
	unsigned int l1_index, l2_index;
	struct vmtl2_entry *l2_table;

	if (vm_id == VGIC_V5_VM_ID_INVAL)
		return ERR_PTR(-EINVAL);

	if (!vmt_info->two_level)
		return &vmt_info->linear.vmt_base[vm_id];

	l1_index = vm_id / GICV5_VMT_L2_TABLE_ENTRIES;
	l2_index = vm_id % GICV5_VMT_L2_TABLE_ENTRIES;

	if (l1_index >= vmt_info->l2.num_l1_ents)
		return ERR_PTR(-E2BIG);

	if (!vmt_info->l2.l2ptrs[l1_index])
		return ERR_PTR(-EINVAL);

	l2_table = vmt_info->l2.l2ptrs[l1_index];
	return &l2_table[l2_index];
}

/*
 * Zero a VMT Entry, and flush & invalidate to the PoC, if required.
 */
static int vgic_v5_reset_vmte(struct kvm *kvm)
{
	u32 vm_id = vgic_v5_vm_id(kvm);
	struct vmtl2_entry *vmte;

	vmte = vgic_v5_get_l2_vmte(vm_id);
	if (IS_ERR(vmte))
		return PTR_ERR(vmte);

	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		/*
		 * The VMT is normal memory shared with the IRS. Invalidate before
		 * rewriting the entry so that cacheline-granular maintenance cannot
		 * later push stale data for neighbouring IRS-visible state back to
		 * memory.
		 */
		vgic_v5_clean_inval(vmte, sizeof(*vmte));

		/*
		 * Prevent the compiler from eliding the individual VMTE
		 * stores. Ordering and visibility to the IRS are provided by the
		 * surrounding cache maintenance and command protocol, not by
		 * WRITE_ONCE().
		 *
		 * The same compiler-access constraint applies to READ_ONCE() users in
		 * this file: when inspecting IRS-visible table entries, read the field
		 * exactly once and prevent the compiler from reusing, merging or
		 * tearing the access. Coherency and freshness for non-coherent IRSes
		 * still come from the surrounding cache maintenance.
		 */
		WRITE_ONCE(vmte->val[0], cpu_to_le64(0ULL));
		WRITE_ONCE(vmte->val[1], cpu_to_le64(0ULL));
		WRITE_ONCE(vmte->val[2], cpu_to_le64(0ULL));
		WRITE_ONCE(vmte->val[3], cpu_to_le64(0ULL));

		/* And make our write visible to the IRS (if non-coherent) */
		vgic_v5_clean_inval(vmte, sizeof(*vmte));
	}

	return 0;
}

/*
 * Use the IDA to allocate a new VM ID, and track it in the gicv5_vm data
 * structure. If we're out of VM IDs, the IDA catches that, and we return the
 * error (-ENOSPC). If we've previously allocated a VM ID, we catch that too and
 * return -EBUSY.
 */
int vgic_v5_allocate_vm_id(struct kvm *kvm)
{
	int id;

	if (kvm->arch.vgic.gicv5_vm.vm_id != VGIC_V5_VM_ID_INVAL)
		return -EBUSY;

	id = ida_alloc_max(&vmt_info->vm_id_ida, vmt_info->num_entries - 1u,
			   GFP_KERNEL);
	if (id < 0)
		return id;

	kvm->arch.vgic.gicv5_vm.vm_id = id;

	return 0;
}

/*
 * Release the VM ID to allow it to be reallocated in the future.
 */
void vgic_v5_release_vm_id(struct kvm *kvm)
{
	if (kvm->arch.vgic.gicv5_vm.vm_id == VGIC_V5_VM_ID_INVAL)
		return;

	ida_free(&vmt_info->vm_id_ida, kvm->arch.vgic.gicv5_vm.vm_id);
	kvm->arch.vgic.gicv5_vm.vm_id = VGIC_V5_VM_ID_INVAL;
}

/*
 * Initialise an entry in the VMT based on the index of the VM.
 *
 * Note: We don't mark the VMTE as valid as this needs to be done by
 * the hardware.
 */
int vgic_v5_vmte_init(struct kvm *kvm)
{
	size_t vmd_alloc_size, vpet_alloc_size, vped_alloc_size;
	void *vped_base = NULL, *vmd = NULL;
	struct vgic_v5_vm_info *vmi = NULL;
	u64 tmp, vmte_val0 = 0, vmte_val1;
	u32 vm_id = vgic_v5_vm_id(kvm);
	int ret, nr_cpus, nr_vcpus;
	bool vmi_inserted = false;
	struct vmtl2_entry *vmte;
	vpe_entry *vpet = NULL;
	struct kvm_vcpu *vcpu;
	u16 max_vpe_id = 0;
	unsigned long i;

	nr_vcpus = atomic_read(&kvm->online_vcpus);
	if (nr_vcpus > vmt_info->max_vpes)
		return -E2BIG;

	/*
	 * If we're using two-level VMTs, L2 is allocated on demand. For linear
	 * VMTs, this is a NOP.
	 */
	ret = vgic_v5_alloc_l2_vmt(kvm);
	if (ret)
		return ret;

	vmte = vgic_v5_get_l2_vmte(vm_id);
	if (IS_ERR(vmte))
		return PTR_ERR(vmte);

	/* If the entry is already valid, something went wrong */
	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		vgic_v5_clean_inval(vmte, sizeof(*vmte));
		if (le64_to_cpu(READ_ONCE(vmte->val[0])) & GICV5_VMTEL2E_VALID)
			return -EINVAL;
	}

	ret = vgic_v5_reset_vmte(kvm);
	if (ret)
		return ret;

	vmi = kzalloc_obj(*vmi);
	if (!vmi) {
		ret = -ENOMEM;
		goto out_fail;
	}

	ret = xa_insert(&vm_info, vm_id, vmi, GFP_KERNEL);
	if (ret)
		goto out_fail;
	vmi_inserted = true;

	/* Allocate and assign the VM Descriptor, if required. */
	if (vmt_info->vmd_size != 0) {
		vmd_alloc_size = round_up(vmt_info->vmd_size,
					  dma_get_cache_alignment());
		vmd = kzalloc(vmd_alloc_size, GFP_KERNEL);
		if (!vmd) {
			ret = -ENOMEM;
			goto out_fail;
		}

		/* Stash the VA so we can free it later */
		vmi->vmd_base = vmd;

		tmp = FIELD_PREP(GICV5_VMTEL2E_VMD_ADDR,
				 virt_to_phys(vmd) >> GICV5_VMTEL2E_VMD_ADDR_SHIFT);
		vmte_val0 = tmp;
	}

	/*
	 * Allocate and assign the VPE Table.
	 *
	 * First of all, iterate over all vcpus to find the highest VPE ID we
	 * require - we need to ensure that we have enough storage for all
	 * vcpu_id values that userspace has picked and not just the total
	 * number of vcpus. This gives us the number of VPEs required for the
	 * VM.
	 *
	 * Round up the number of VPEs to a whole power of two as we cannot
	 * describe non-powers-of-two in the VMTE field as it conveys the number
	 * of ID bits used and not the number of vPEs. IRS_IDR1.IAFFID_BITS is
	 * encoded as N - 1, so expose at least one VPE ID bit even for a
	 * single-vCPU VM to keep the views consistent.
	 */
	kvm_for_each_vcpu(i, vcpu, kvm) {
		u16 vpe_id = vgic_v5_vpe_id(vcpu);

		if (vpe_id > max_vpe_id)
			max_vpe_id = vpe_id;
	}

	nr_cpus = max(2UL, roundup_pow_of_two(max_vpe_id + 1));
	vmi->vpe_id_bits = fls(nr_cpus) - 1;

	vpet_alloc_size = round_up((size_t)nr_cpus * sizeof(*vpet),
				   dma_get_cache_alignment());
	vpet = kzalloc(vpet_alloc_size, GFP_KERNEL);
	if (!vpet) {
		ret = -ENOMEM;
		goto out_fail;
	}

	/* Stash the VA so we can free it later */
	vmi->vpet_base = vpet;

	tmp = FIELD_PREP(GICV5_VMTEL2E_VPET_ADDR,
			 virt_to_phys(vpet) >> GICV5_VMTEL2E_VPET_ADDR_SHIFT);
	tmp |= FIELD_PREP(GICV5_VMTEL2E_VPE_ID_BITS, vmi->vpe_id_bits);
	vmte_val1 = tmp;

	/*
	 * Allocate a dense VPED array indexed by vcpu_idx. The VPET is indexed
	 * by the potentially sparse vcpu_id, but using that ID here would waste
	 * memory. Given that this is not userspace visible, we can cheat a bit
	 * and use the dense index instead. This is the ONLY place that we do
	 * this.
	 *
	 * Round the requested size up to a whole cacheline. kzalloc() gurantees
	 * natural alignment, so we ensure that the cachelines cannot be shared
	 * with unrelated slab objects. VPED and cacheline sizes are powers of
	 * two, so this also preserves the required VPED alignment when a VPED
	 * is larger than a cacheline.
	 */
	vped_alloc_size = round_up((size_t)nr_vcpus * vmt_info->vped_size,
				   dma_get_cache_alignment());
	vped_base = kzalloc(vped_alloc_size, GFP_KERNEL);
	if (!vped_base) {
		ret = -ENOMEM;
		goto out_fail;
	}
	vmi->vped_base = vped_base;

	if (vmd)
		vgic_v5_clean_inval(vmd, vmd_alloc_size);
	vgic_v5_clean_inval(vpet, vpet_alloc_size);
	vgic_v5_clean_inval(vped_base, vped_alloc_size);

	/* Publish the VMTE while serialising access to the shared VMT. */
	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		WRITE_ONCE(vmte->val[0], cpu_to_le64(vmte_val0));
		WRITE_ONCE(vmte->val[1], cpu_to_le64(vmte_val1));
		vgic_v5_clean_inval(vmte, sizeof(*vmte));
	}

	kvm->arch.vgic.gicv5_vm.vmte_allocated = true;

	return 0;

out_fail:
	/* kfree(NULL) is safe so we can just kfree() at leisure */
	kfree(vmd);
	kfree(vpet);
	kfree(vped_base);
	if (vmi_inserted)
		xa_erase(&vm_info, vm_id);
	kfree(vmi);

	vgic_v5_reset_vmte(kvm);

	return ret;
}

/*
 * Release the VMT Entry, freeing up any allocated data structures before
 * zeroing the VMTE.
 *
 * The VMTE must be marked as invalid before it is released.
 */
int vgic_v5_vmte_release(struct kvm *kvm)
{
	u32 vm_id = vgic_v5_vm_id(kvm);
	struct vgic_v5_vm_info *vmi;
	struct vmtl2_entry *vmte;
	int ret;

	vmte = vgic_v5_get_l2_vmte(vm_id);
	if (IS_ERR(vmte))
		return PTR_ERR(vmte);

	/* Reject if the VMTE has not been marked as invalid! */
	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		vgic_v5_clean_inval(vmte, sizeof(*vmte));
		if (le64_to_cpu(READ_ONCE(vmte->val[0])) & GICV5_VMTEL2E_VALID)
			return -EINVAL;
	}

	vmi = xa_load(&vm_info, vm_id);
	if (!vmi)
		goto no_vmi;

	kfree(vmi->vped_base);
	kfree(vmi->vpet_base);
	kfree(vmi->vmd_base);

	xa_erase(&vm_info, vm_id);
	kfree(vmi);

no_vmi:
	/*
	 * If we didn't get far enough into allocating a VMTE to create the VM
	 * info structure, then we just zero the VMTE and move on. There's
	 * nothing else we can realistically do here.
	 */
	ret = vgic_v5_reset_vmte(kvm);
	if (ret)
		return ret;

	kvm->arch.vgic.gicv5_vm.vmte_allocated = false;

	return 0;
}

/* Provide the preallocated VPE descriptor to the hardware via the VPE Table. */
int vgic_v5_vmte_alloc_vpe(struct kvm_vcpu *vcpu)
{
	u32 vm_id = vgic_v5_vm_id(vcpu->kvm);
	u16 vpe_id = vgic_v5_vpe_id(vcpu);
	struct vgic_v5_vm_info *vmi;
	vpe_entry tmp, *vpet_base;
	void *vped;

	/* Make sure we're not over what the hardware supports */
	if (vpe_id >= vmt_info->max_vpes)
		return -E2BIG;

	vmi = xa_load(&vm_info, vm_id);
	if (!vmi)
		return -EINVAL;

	if (vpe_id >= 1 << vmi->vpe_id_bits)
		return -E2BIG;

	vpet_base = vmi->vpet_base;

	/* If the VPETE for this CPU is already valid we've gone wrong */
	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		vgic_v5_clean_inval(&vpet_base[vpe_id], sizeof(*vpet_base));
		if (le64_to_cpu(READ_ONCE(vpet_base[vpe_id])) & GICV5_VPE_VALID)
			return -EBUSY;
	}

	vped = (u8 *)vmi->vped_base +
		(size_t)vcpu->vcpu_idx * vmt_info->vped_size;

	tmp = FIELD_PREP(GICV5_VPED_ADDR, virt_to_phys(vped) >> GICV5_VPED_ADDR_SHIFT);

	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		WRITE_ONCE(vpet_base[vpe_id], cpu_to_le64(tmp));
		vgic_v5_clean_inval(vpet_base + vpe_id, sizeof(vpe_entry));
	}

	return 0;
}

/* Clear the VPE's table entry after the VMTE has been made invalid. */
int vgic_v5_vmte_free_vpe(struct kvm_vcpu *vcpu)
{
	u32 vm_id = vgic_v5_vm_id(vcpu->kvm);
	u16 vpe_id = vgic_v5_vpe_id(vcpu);
	struct vgic_v5_vm_info *vmi;
	struct vmtl2_entry *vmte;
	vpe_entry *vpet_base;

	vmte = vgic_v5_get_l2_vmte(vm_id);
	if (IS_ERR(vmte))
		return PTR_ERR(vmte);

	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		vgic_v5_clean_inval(vmte, sizeof(*vmte));
		if (le64_to_cpu(READ_ONCE(vmte->val[0])) & GICV5_VMTEL2E_VALID)
			return -EBUSY;
	}

	vmi = xa_load(&vm_info, vm_id);
	if (!vmi)
		return -EINVAL;

	if (vpe_id >= 1 << vmi->vpe_id_bits)
		return -E2BIG;

	vpet_base = vmi->vpet_base;
	scoped_guard(raw_spinlock_irqsave, &vgic_v5_irs_lock) {
		WRITE_ONCE(vpet_base[vpe_id], 0ULL);
		vgic_v5_clean_inval(vpet_base + vpe_id, sizeof(vpe_entry));
	}

	return 0;
}
