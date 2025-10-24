// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kernel-based Virtual Machine driver for Linux
 *
 * AMD SVM support
 *
 * Copyright (C) 2006 Qumranet, Inc.
 * Copyright 2010 Red Hat, Inc. and/or its affiliates.
 *
 * Authors:
 *   Yaniv Kamay  <yaniv@qumranet.com>
 *   Avi Kivity   <avi@qumranet.com>
 */

#ifndef __SVM_SVM_H
#define __SVM_SVM_H

#include <linux/kvm_types.h>
#include <linux/kvm_host.h>
#include <linux/bits.h>

#include <asm/svm.h>
#include <asm/sev-common.h>

#include "cpuid.h"
#include "kvm_cache_regs.h"

/*
 * Helpers to convert to/from physical addresses for pages whose address is
 * consumed directly by hardware.  Even though it's a physical address, SVM
 * often restricts the address to the natural width, hence 'unsigned long'
 * instead of 'hpa_t'.
 */
static inline unsigned long __sme_page_pa(struct page *page)
{
	return __sme_set(page_to_pfn(page) << PAGE_SHIFT);
}

static inline struct page *__sme_pa_to_page(unsigned long pa)
{
	return pfn_to_page(__sme_clr(pa) >> PAGE_SHIFT);
}

#define	IOPM_SIZE PAGE_SIZE * 3
#define	MSRPM_SIZE PAGE_SIZE * 2

extern bool npt_enabled;
extern int nrips;
extern int vgif;
extern bool intercept_smi;
extern bool vnmi;
extern int lbrv;

extern int tsc_aux_uret_slot __ro_after_init;

extern struct kvm_x86_ops svm_x86_ops __initdata;

/*
 * Clean bits in VMCB.
 * VMCB_ALL_CLEAN_MASK might also need to
 * be updated if this enum is modified.
 */
enum {
	VMCB_INTERCEPTS, /* Intercept vectors, TSC offset,
			    pause filter count */
	VMCB_PERM_MAP,   /* IOPM Base and MSRPM Base */
	VMCB_ASID,	 /* ASID */
	VMCB_INTR,	 /* int_ctl, int_vector */
	VMCB_NPT,        /* npt_en, nCR3, gPAT */
	VMCB_CR,	 /* CR0, CR3, CR4, EFER */
	VMCB_DR,         /* DR6, DR7 */
	VMCB_DT,         /* GDT, IDT */
	VMCB_SEG,        /* CS, DS, SS, ES, CPL */
	VMCB_CR2,        /* CR2 only */
	VMCB_LBR,        /* DBGCTL, BR_FROM, BR_TO, LAST_EX_FROM, LAST_EX_TO */
	VMCB_AVIC,       /* AVIC APIC_BAR, AVIC APIC_BACKING_PAGE,
			  * AVIC PHYSICAL_TABLE pointer,
			  * AVIC LOGICAL_TABLE pointer
			  */
	VMCB_CET,	 /* S_CET, SSP, ISST_ADDR */
	VMCB_SW = 31,    /* Reserved for hypervisor/software use */
};

#define VMCB_ALL_CLEAN_MASK (					\
	(1U << VMCB_INTERCEPTS) | (1U << VMCB_PERM_MAP) |	\
	(1U << VMCB_ASID) | (1U << VMCB_INTR) |			\
	(1U << VMCB_NPT) | (1U << VMCB_CR) | (1U << VMCB_DR) |	\
	(1U << VMCB_DT) | (1U << VMCB_SEG) | (1U << VMCB_CR2) |	\
	(1U << VMCB_LBR) | (1U << VMCB_AVIC) | (1U << VMCB_CET) | \
	(1U << VMCB_SW))

/* TPR and CR2 are always written before VMRUN */
#define VMCB_ALWAYS_DIRTY_MASK	((1U << VMCB_INTR) | (1U << VMCB_CR2))

struct kvm_sev_info {
	bool active;		/* SEV enabled guest */
	bool es_active;		/* SEV-ES enabled guest */
	bool need_init;		/* waiting for SEV_INIT2 */
	unsigned int asid;	/* ASID used for this guest */
	unsigned int handle;	/* SEV firmware handle */
	int fd;			/* SEV device fd */
	unsigned long policy;
	unsigned long pages_locked; /* Number of pages locked */
	struct list_head regions_list;  /* List of registered regions */
	u64 ap_jump_table;	/* SEV-ES AP Jump Table address */
	u64 vmsa_features;
	u16 ghcb_version;	/* Highest guest GHCB protocol version allowed */
	struct kvm *enc_context_owner; /* Owner of copied encryption context */
	struct list_head mirror_vms; /* List of VMs mirroring */
	struct list_head mirror_entry; /* Use as a list entry of mirrors */
	struct misc_cg *misc_cg; /* For misc cgroup accounting */
	atomic_t migration_in_progress;
	void *snp_context;      /* SNP guest context page */
	void *guest_req_buf;    /* Bounce buffer for SNP Guest Request input */
	void *guest_resp_buf;   /* Bounce buffer for SNP Guest Request output */
	struct mutex guest_req_mutex; /* Must acquire before using bounce buffers */
	cpumask_var_t have_run_cpus; /* CPUs that have done VMRUN for this VM. */
};

#define SEV_POLICY_NODBG	BIT_ULL(0)
#define SNP_POLICY_DEBUG	BIT_ULL(19)

struct kvm_svm {
	struct kvm kvm;

	/* Struct members for AVIC */
	u32 avic_vm_id;
	u32 *avic_logical_id_table;
	u64 *avic_physical_id_table;
	struct hlist_node hnode;

	struct kvm_sev_info sev_info;
};

struct kvm_vcpu;

struct kvm_vmcb_info {
	struct vmcb *ptr;
	unsigned long pa;
	int cpu;
	uint64_t asid_generation;
};

struct vmcb_save_area_cached {
	u64 efer;
	u64 cr4;
	u64 cr3;
	u64 cr0;
	u64 dr7;
	u64 dr6;
};

struct vmcb_ctrl_area_cached {
	u32 intercepts[MAX_INTERCEPT];
	u16 pause_filter_thresh;
	u16 pause_filter_count;
	u64 iopm_base_pa;
	u64 msrpm_base_pa;
	u64 tsc_offset;
	u32 asid;
	u8 tlb_ctl;
	u32 int_ctl;
	u32 int_vector;
	u32 int_state;
	u32 exit_code;
	u32 exit_code_hi;
	u64 exit_info_1;
	u64 exit_info_2;
	u32 exit_int_info;
	u32 exit_int_info_err;
	u64 nested_ctl;
	u32 event_inj;
	u32 event_inj_err;
	u64 next_rip;
	u64 nested_cr3;
	u64 virt_ext;
	u32 clean;
	u64 bus_lock_rip;
	union {
#if IS_ENABLED(CONFIG_HYPERV) || IS_ENABLED(CONFIG_KVM_HYPERV)
		struct hv_vmcb_enlightenments hv_enlightenments;
#endif
		u8 reserved_sw[32];
	};
};

struct svm_nested_state {
	struct kvm_vmcb_info vmcb02;
	u64 hsave_msr;
	u64 vm_cr_msr;
	u64 vmcb12_gpa;
	u64 last_vmcb12_gpa;

	/*
	 * The MSR permissions map used for vmcb02, which is the merge result
	 * of vmcb01 and vmcb12
	 */
	void *msrpm;

	/* A VMRUN has started but has not yet been performed, so
	 * we cannot inject a nested vmexit yet.  */
	bool nested_run_pending;

	/* cache for control fields of the guest */
	struct vmcb_ctrl_area_cached ctl;

	/*
	 * Note: this struct is not kept up-to-date while L2 runs; it is only
	 * valid within nested_svm_vmrun.
	 */
	struct vmcb_save_area_cached save;

	bool initialized;

	/*
	 * Indicates whether MSR bitmap for L2 needs to be rebuilt due to
	 * changes in MSR bitmap for L1 or switching to a different L2. Note,
	 * this flag can only be used reliably in conjunction with a paravirt L1
	 * which informs L0 whether any changes to MSR bitmap for L2 were done
	 * on its side.
	 */
	bool force_msr_bitmap_recalc;
};

struct vcpu_sev_es_state {
	/* SEV-ES support */
	struct sev_es_save_area *vmsa;
	struct ghcb *ghcb;
	u8 valid_bitmap[16];
	struct kvm_host_map ghcb_map;
	bool received_first_sipi;
	unsigned int ap_reset_hold_type;

	/* SEV-ES scratch area support */
	u64 sw_scratch;
	void *ghcb_sa;
	u32 ghcb_sa_len;
	bool ghcb_sa_sync;
	bool ghcb_sa_free;

	/* SNP Page-State-Change buffer entries currently being processed */
	u16 psc_idx;
	u16 psc_inflight;
	bool psc_2m;

	u64 ghcb_registered_gpa;

	struct mutex snp_vmsa_mutex; /* Used to handle concurrent updates of VMSA. */
	gpa_t snp_vmsa_gpa;
	bool snp_ap_waiting_for_reset;
	bool snp_has_guest_vmsa;
};

struct vcpu_svm {
	struct kvm_vcpu vcpu;
	/* vmcb always points at current_vmcb->ptr, it's purely a shorthand. */
	struct vmcb *vmcb;
	struct kvm_vmcb_info vmcb01;
	struct kvm_vmcb_info *current_vmcb;
	u32 asid;
	u32 sysenter_esp_hi;
	u32 sysenter_eip_hi;
	uint64_t tsc_aux;

	u64 msr_decfg;

	u64 next_rip;

	u64 spec_ctrl;

	u64 tsc_ratio_msr;
	/*
	 * Contains guest-controlled bits of VIRT_SPEC_CTRL, which will be
	 * translated into the appropriate L2_CFG bits on the host to
	 * perform speculative control.
	 */
	u64 virt_spec_ctrl;

	void *msrpm;

	ulong nmi_iret_rip;

	struct svm_nested_state nested;

	/* NMI mask value, used when vNMI is not enabled */
	bool nmi_masked;

	/*
	 * True when NMIs are still masked but guest IRET was just intercepted
	 * and KVM is waiting for RIP to change, which will signal that the
	 * intercepted IRET was retired and thus NMI can be unmasked.
	 */
	bool awaiting_iret_completion;

	/*
	 * Set when KVM is awaiting IRET completion and needs to inject NMIs as
	 * soon as the IRET completes (e.g. NMI is pending injection).  KVM
	 * temporarily steals RFLAGS.TF to single-step the guest in this case
	 * in order to regain control as soon as the NMI-blocking condition
	 * goes away.
	 */
	bool nmi_singlestep;
	u64 nmi_singlestep_guest_rflags;

	bool nmi_l1_to_l2;

	unsigned long soft_int_csbase;
	unsigned long soft_int_old_rip;
	unsigned long soft_int_next_rip;
	bool soft_int_injected;

	u32 ldr_reg;
	u32 dfr_reg;

	/* This is essentially a shadow of the vCPU's actual entry in the
	 * Physical ID table that is programmed into the VMCB, i.e. that is
	 * seen by the CPU.  If IPI virtualization is disabled, IsRunning is
	 * only ever set in the shadow, i.e. is never propagated to the "real"
	 * table, so that hardware never sees IsRunning=1.
	 */
	u64 avic_physical_id_entry;

	/*
	 * Per-vCPU list of irqfds that are eligible to post IRQs directly to
	 * the vCPU (a.k.a. device posted IRQs, a.k.a. IRQ bypass).  The list
	 * is used to reconfigure IRTEs when the vCPU is loaded/put (to set the
	 * target pCPU), when AVIC is toggled on/off (to (de)activate bypass),
	 * and if the irqfd becomes ineligible for posting (to put the IRTE
	 * back into remapped mode).
	 */
	struct list_head ir_list;
	spinlock_t ir_list_lock;

	struct vcpu_sev_es_state sev_es;

	bool guest_state_loaded;

	bool x2avic_msrs_intercepted;

	/* Guest GIF value, used when vGIF is not enabled */
	bool guest_gif;
};

struct svm_cpu_data {
	u64 asid_generation;
	u32 max_asid;
	u32 next_asid;
	u32 min_asid;

	bool bp_spec_reduce_set;

	struct vmcb *save_area;
	unsigned long save_area_pa;

	/* index = sev_asid, value = vmcb pointer */
	struct vmcb **sev_vmcbs;
};

DECLARE_PER_CPU(struct svm_cpu_data, svm_data);

void recalc_intercepts(struct vcpu_svm *svm);

static __always_inline struct kvm_svm *to_kvm_svm(struct kvm *kvm)
{
	return container_of(kvm, struct kvm_svm, kvm);
}

static __always_inline struct kvm_sev_info *to_kvm_sev_info(struct kvm *kvm)
{
	return &to_kvm_svm(kvm)->sev_info;
}

#ifdef CONFIG_KVM_AMD_SEV
static __always_inline bool sev_guest(struct kvm *kvm)
{
	return to_kvm_sev_info(kvm)->active;
}
static __always_inline bool sev_es_guest(struct kvm *kvm)
{
	struct kvm_sev_info *sev = to_kvm_sev_info(kvm);

	return sev->es_active && !WARN_ON_ONCE(!sev->active);
}

static __always_inline bool sev_snp_guest(struct kvm *kvm)
{
	struct kvm_sev_info *sev = to_kvm_sev_info(kvm);

	return (sev->vmsa_features & SVM_SEV_FEAT_SNP_ACTIVE) &&
	       !WARN_ON_ONCE(!sev_es_guest(kvm));
}
#else
#define sev_guest(kvm) false
#define sev_es_guest(kvm) false
#define sev_snp_guest(kvm) false
#endif

static inline bool ghcb_gpa_is_registered(struct vcpu_svm *svm, u64 val)
{
	return svm->sev_es.ghcb_registered_gpa == val;
}

static inline void vmcb_mark_all_dirty(struct vmcb *vmcb)
{
	vmcb->control.clean = 0;
}

static inline void vmcb_mark_all_clean(struct vmcb *vmcb)
{
	vmcb->control.clean = VMCB_ALL_CLEAN_MASK
			       & ~VMCB_ALWAYS_DIRTY_MASK;
}

static inline void vmcb_mark_dirty(struct vmcb *vmcb, int bit)
{
	vmcb->control.clean &= ~(1 << bit);
}

static inline bool vmcb_is_dirty(struct vmcb *vmcb, int bit)
{
        return !test_bit(bit, (unsigned long *)&vmcb->control.clean);
}

static __always_inline struct vcpu_svm *to_svm(struct kvm_vcpu *vcpu)
{
	return container_of(vcpu, struct vcpu_svm, vcpu);
}

/*
 * Only the PDPTRs are loaded on demand into the shadow MMU.  All other
 * fields are synchronized on VM-Exit, because accessing the VMCB is cheap.
 *
 * CR3 might be out of date in the VMCB but it is not marked dirty; instead,
 * KVM_REQ_LOAD_MMU_PGD is always requested when the cached vcpu->arch.cr3
 * is changed.  svm_load_mmu_pgd() then syncs the new CR3 value into the VMCB.
 */
#define SVM_REGS_LAZY_LOAD_SET	(1 << VCPU_EXREG_PDPTR)

static inline void vmcb_set_intercept(struct vmcb_control_area *control, u32 bit)
{
	WARN_ON_ONCE(bit >= 32 * MAX_INTERCEPT);
	__set_bit(bit, (unsigned long *)&control->intercepts);
}

static inline void vmcb_clr_intercept(struct vmcb_control_area *control, u32 bit)
{
	WARN_ON_ONCE(bit >= 32 * MAX_INTERCEPT);
	__clear_bit(bit, (unsigned long *)&control->intercepts);
}

static inline bool vmcb_is_intercept(struct vmcb_control_area *control, u32 bit)
{
	WARN_ON_ONCE(bit >= 32 * MAX_INTERCEPT);
	return test_bit(bit, (unsigned long *)&control->intercepts);
}

static inline bool vmcb12_is_intercept(struct vmcb_ctrl_area_cached *control, u32 bit)
{
	WARN_ON_ONCE(bit >= 32 * MAX_INTERCEPT);
	return test_bit(bit, (unsigned long *)&control->intercepts);
}

static inline void set_exception_intercept(struct vcpu_svm *svm, u32 bit)
{
	struct vmcb *vmcb = svm->vmcb01.ptr;

	WARN_ON_ONCE(bit >= 32);
	vmcb_set_intercept(&vmcb->control, INTERCEPT_EXCEPTION_OFFSET + bit);

	recalc_intercepts(svm);
}

static inline void clr_exception_intercept(struct vcpu_svm *svm, u32 bit)
{
	struct vmcb *vmcb = svm->vmcb01.ptr;

	WARN_ON_ONCE(bit >= 32);
	vmcb_clr_intercept(&vmcb->control, INTERCEPT_EXCEPTION_OFFSET + bit);

	recalc_intercepts(svm);
}

static inline void svm_set_intercept(struct vcpu_svm *svm, int bit)
{
	struct vmcb *vmcb = svm->vmcb01.ptr;

	vmcb_set_intercept(&vmcb->control, bit);

	recalc_intercepts(svm);
}

static inline void svm_clr_intercept(struct vcpu_svm *svm, int bit)
{
	struct vmcb *vmcb = svm->vmcb01.ptr;

	vmcb_clr_intercept(&vmcb->control, bit);

	recalc_intercepts(svm);
}

static inline bool svm_is_intercept(struct vcpu_svm *svm, int bit)
{
	return vmcb_is_intercept(&svm->vmcb->control, bit);
}

static inline bool nested_vgif_enabled(struct vcpu_svm *svm)
{
	return guest_cpu_cap_has(&svm->vcpu, X86_FEATURE_VGIF) &&
	       (svm->nested.ctl.int_ctl & V_GIF_ENABLE_MASK);
}

static inline struct vmcb *get_vgif_vmcb(struct vcpu_svm *svm)
{
	if (!vgif)
		return NULL;

	if (is_guest_mode(&svm->vcpu) && !nested_vgif_enabled(svm))
		return svm->nested.vmcb02.ptr;
	else
		return svm->vmcb01.ptr;
}

static inline void enable_gif(struct vcpu_svm *svm)
{
	struct vmcb *vmcb = get_vgif_vmcb(svm);

	if (vmcb)
		vmcb->control.int_ctl |= V_GIF_MASK;
	else
		svm->guest_gif = true;
}

static inline void disable_gif(struct vcpu_svm *svm)
{
	struct vmcb *vmcb = get_vgif_vmcb(svm);

	if (vmcb)
		vmcb->control.int_ctl &= ~V_GIF_MASK;
	else
		svm->guest_gif = false;
}

static inline bool gif_set(struct vcpu_svm *svm)
{
	struct vmcb *vmcb = get_vgif_vmcb(svm);

	if (vmcb)
		return !!(vmcb->control.int_ctl & V_GIF_MASK);
	else
		return svm->guest_gif;
}

static inline bool nested_npt_enabled(struct vcpu_svm *svm)
{
	return svm->nested.ctl.nested_ctl & SVM_NESTED_CTL_NP_ENABLE;
}

static inline bool nested_vnmi_enabled(struct vcpu_svm *svm)
{
	return guest_cpu_cap_has(&svm->vcpu, X86_FEATURE_VNMI) &&
	       (svm->nested.ctl.int_ctl & V_NMI_ENABLE_MASK);
}

static inline bool is_x2apic_msrpm_offset(u32 offset)
{
	/* 4 msrs per u8, and 4 u8 in u32 */
	u32 msr = offset * 16;

	return (msr >= APIC_BASE_MSR) &&
	       (msr < (APIC_BASE_MSR + 0x100));
}

static inline struct vmcb *get_vnmi_vmcb_l1(struct vcpu_svm *svm)
{
	if (!vnmi)
		return NULL;

	if (is_guest_mode(&svm->vcpu))
		return NULL;
	else
		return svm->vmcb01.ptr;
}

static inline bool is_vnmi_enabled(struct vcpu_svm *svm)
{
	struct vmcb *vmcb = get_vnmi_vmcb_l1(svm);

	if (vmcb)
		return !!(vmcb->control.int_ctl & V_NMI_ENABLE_MASK);
	else
		return false;
}

static inline void svm_vmgexit_set_return_code(struct vcpu_svm *svm,
						u64 response, u64 data)
{
	ghcb_set_sw_exit_info_1(svm->sev_es.ghcb, response);
	ghcb_set_sw_exit_info_2(svm->sev_es.ghcb, data);
}

static inline void svm_vmgexit_inject_exception(struct vcpu_svm *svm, u8 vector)
{
	u64 data = SVM_EVTINJ_VALID | SVM_EVTINJ_TYPE_EXEPT | vector;

	svm_vmgexit_set_return_code(svm, GHCB_HV_RESP_ISSUE_EXCEPTION, data);
}

static inline void svm_vmgexit_bad_input(struct vcpu_svm *svm, u64 suberror)
{
	svm_vmgexit_set_return_code(svm, GHCB_HV_RESP_MALFORMED_INPUT, suberror);
}

static inline void svm_vmgexit_success(struct vcpu_svm *svm, u64 data)
{
	svm_vmgexit_set_return_code(svm, GHCB_HV_RESP_NO_ACTION, data);
}

static inline void svm_vmgexit_no_action(struct vcpu_svm *svm, u64 data)
{
	svm_vmgexit_set_return_code(svm, GHCB_HV_RESP_NO_ACTION, data);
}

/*
 * The MSRPM is 8KiB in size, divided into four 2KiB ranges (the fourth range
 * is reserved).  Each MSR within a range is covered by two bits, one each for
 * read (bit 0) and write (bit 1), where a bit value of '1' means intercepted.
 */
#define SVM_MSRPM_BYTES_PER_RANGE 2048
#define SVM_BITS_PER_MSR 2
#define SVM_MSRS_PER_BYTE (BITS_PER_BYTE / SVM_BITS_PER_MSR)
#define SVM_MSRS_PER_RANGE (SVM_MSRPM_BYTES_PER_RANGE * SVM_MSRS_PER_BYTE)
static_assert(SVM_MSRS_PER_RANGE == 8192);
#define SVM_MSRPM_OFFSET_MASK (SVM_MSRS_PER_RANGE - 1)

static __always_inline int svm_msrpm_bit_nr(u32 msr)
{
	int range_nr;

	switch (msr & ~SVM_MSRPM_OFFSET_MASK) {
	case 0:
		range_nr = 0;
		break;
	case 0xc0000000:
		range_nr = 1;
		break;
	case 0xc0010000:
		range_nr = 2;
		break;
	default:
		return -EINVAL;
	}

	return range_nr * SVM_MSRPM_BYTES_PER_RANGE * BITS_PER_BYTE +
	       (msr & SVM_MSRPM_OFFSET_MASK) * SVM_BITS_PER_MSR;
}

#define __BUILD_SVM_MSR_BITMAP_HELPER(rtype, action, bitop, access, bit_rw)	\
static inline rtype svm_##action##_msr_bitmap_##access(unsigned long *bitmap,	\
						       u32 msr)			\
{										\
	int bit_nr;								\
										\
	bit_nr = svm_msrpm_bit_nr(msr);						\
	if (bit_nr < 0)								\
		return (rtype)true;						\
										\
	return bitop##_bit(bit_nr + bit_rw, bitmap);				\
}

#define BUILD_SVM_MSR_BITMAP_HELPERS(ret_type, action, bitop)			\
	__BUILD_SVM_MSR_BITMAP_HELPER(ret_type, action, bitop, read,  0)	\
	__BUILD_SVM_MSR_BITMAP_HELPER(ret_type, action, bitop, write, 1)

BUILD_SVM_MSR_BITMAP_HELPERS(bool, test, test)
BUILD_SVM_MSR_BITMAP_HELPERS(void, clear, __clear)
BUILD_SVM_MSR_BITMAP_HELPERS(void, set, __set)

#define DEBUGCTL_RESERVED_BITS (~DEBUGCTLMSR_LBR)

/* svm.c */
extern bool dump_invalid_vmcb;

void *svm_alloc_permissions_map(unsigned long size, gfp_t gfp_mask);

static inline void *svm_vcpu_alloc_msrpm(void)
{
	return svm_alloc_permissions_map(MSRPM_SIZE, GFP_KERNEL_ACCOUNT);
}

void svm_vcpu_free_msrpm(void *msrpm);
void svm_copy_lbrs(struct vmcb *to_vmcb, struct vmcb *from_vmcb);
void svm_enable_lbrv(struct kvm_vcpu *vcpu);
void svm_update_lbrv(struct kvm_vcpu *vcpu);

int svm_set_efer(struct kvm_vcpu *vcpu, u64 efer);
void svm_set_cr0(struct kvm_vcpu *vcpu, unsigned long cr0);
void svm_set_cr4(struct kvm_vcpu *vcpu, unsigned long cr4);
void disable_nmi_singlestep(struct vcpu_svm *svm);
bool svm_smi_blocked(struct kvm_vcpu *vcpu);
bool svm_nmi_blocked(struct kvm_vcpu *vcpu);
bool svm_interrupt_blocked(struct kvm_vcpu *vcpu);
void svm_set_gif(struct vcpu_svm *svm, bool value);
int svm_invoke_exit_handler(struct kvm_vcpu *vcpu, u64 exit_code);
void set_msr_interception(struct kvm_vcpu *vcpu, u32 *msrpm, u32 msr,
			  int read, int write);
void svm_complete_interrupt_delivery(struct kvm_vcpu *vcpu, int delivery_mode,
				     int trig_mode, int vec);

void svm_set_intercept_for_msr(struct kvm_vcpu *vcpu, u32 msr, int type, bool set);

static inline void svm_disable_intercept_for_msr(struct kvm_vcpu *vcpu,
						 u32 msr, int type)
{
	svm_set_intercept_for_msr(vcpu, msr, type, false);
}

static inline void svm_enable_intercept_for_msr(struct kvm_vcpu *vcpu,
						u32 msr, int type)
{
	svm_set_intercept_for_msr(vcpu, msr, type, true);
}

/* nested.c */

#define NESTED_EXIT_HOST	0	/* Exit handled on host level */
#define NESTED_EXIT_DONE	1	/* Exit caused nested vmexit  */
#define NESTED_EXIT_CONTINUE	2	/* Further checks needed      */

static inline bool nested_svm_virtualize_tpr(struct kvm_vcpu *vcpu)
{
	struct vcpu_svm *svm = to_svm(vcpu);

	return is_guest_mode(vcpu) && (svm->nested.ctl.int_ctl & V_INTR_MASKING_MASK);
}

static inline bool nested_exit_on_smi(struct vcpu_svm *svm)
{
	return vmcb12_is_intercept(&svm->nested.ctl, INTERCEPT_SMI);
}

static inline bool nested_exit_on_intr(struct vcpu_svm *svm)
{
	return vmcb12_is_intercept(&svm->nested.ctl, INTERCEPT_INTR);
}

static inline bool nested_exit_on_nmi(struct vcpu_svm *svm)
{
	return vmcb12_is_intercept(&svm->nested.ctl, INTERCEPT_NMI);
}

int __init nested_svm_init_msrpm_merge_offsets(void);

int enter_svm_guest_mode(struct kvm_vcpu *vcpu,
			 u64 vmcb_gpa, struct vmcb *vmcb12, bool from_vmrun);
void svm_leave_nested(struct kvm_vcpu *vcpu);
void svm_free_nested(struct vcpu_svm *svm);
int svm_allocate_nested(struct vcpu_svm *svm);
int nested_svm_vmrun(struct kvm_vcpu *vcpu);
void svm_copy_vmrun_state(struct vmcb_save_area *to_save,
			  struct vmcb_save_area *from_save);
void svm_copy_vmloadsave_state(struct vmcb *to_vmcb, struct vmcb *from_vmcb);
int nested_svm_vmexit(struct vcpu_svm *svm);

static inline int nested_svm_simple_vmexit(struct vcpu_svm *svm, u32 exit_code)
{
	svm->vmcb->control.exit_code   = exit_code;
	svm->vmcb->control.exit_info_1 = 0;
	svm->vmcb->control.exit_info_2 = 0;
	return nested_svm_vmexit(svm);
}

int nested_svm_exit_handled(struct vcpu_svm *svm);
int nested_svm_check_permissions(struct kvm_vcpu *vcpu);
int nested_svm_check_exception(struct vcpu_svm *svm, unsigned nr,
			       bool has_error_code, u32 error_code);
int nested_svm_exit_special(struct vcpu_svm *svm);
void nested_svm_update_tsc_ratio_msr(struct kvm_vcpu *vcpu);
void svm_write_tsc_multiplier(struct kvm_vcpu *vcpu);
void nested_copy_vmcb_control_to_cache(struct vcpu_svm *svm,
				       struct vmcb_control_area *control);
void nested_copy_vmcb_save_to_cache(struct vcpu_svm *svm,
				    struct vmcb_save_area *save);
void nested_sync_control_from_vmcb02(struct vcpu_svm *svm);
void nested_vmcb02_compute_g_pat(struct vcpu_svm *svm);
void svm_switch_vmcb(struct vcpu_svm *svm, struct kvm_vmcb_info *target_vmcb);

extern struct kvm_x86_nested_ops svm_nested_ops;

/* avic.c */
#define AVIC_REQUIRED_APICV_INHIBITS			\
(							\
	BIT(APICV_INHIBIT_REASON_DISABLED) |		\
	BIT(APICV_INHIBIT_REASON_ABSENT) |		\
	BIT(APICV_INHIBIT_REASON_HYPERV) |		\
	BIT(APICV_INHIBIT_REASON_NESTED) |		\
	BIT(APICV_INHIBIT_REASON_IRQWIN) |		\
	BIT(APICV_INHIBIT_REASON_PIT_REINJ) |		\
	BIT(APICV_INHIBIT_REASON_BLOCKIRQ) |		\
	BIT(APICV_INHIBIT_REASON_SEV)      |		\
	BIT(APICV_INHIBIT_REASON_PHYSICAL_ID_ALIASED) |	\
	BIT(APICV_INHIBIT_REASON_APIC_ID_MODIFIED) |	\
	BIT(APICV_INHIBIT_REASON_APIC_BASE_MODIFIED) |	\
	BIT(APICV_INHIBIT_REASON_LOGICAL_ID_ALIASED) |	\
	BIT(APICV_INHIBIT_REASON_PHYSICAL_ID_TOO_BIG)	\
)

bool __init avic_hardware_setup(void);
int avic_ga_log_notifier(u32 ga_tag);
int avic_alloc_physical_id_table(struct kvm *kvm);
void avic_vm_destroy(struct kvm *kvm);
int avic_vm_init(struct kvm *kvm);
void avic_init_vmcb(struct vcpu_svm *svm, struct vmcb *vmcb);
int avic_incomplete_ipi_interception(struct kvm_vcpu *vcpu);
int avic_unaccelerated_access_interception(struct kvm_vcpu *vcpu);
int avic_init_vcpu(struct vcpu_svm *svm);
void avic_vcpu_load(struct kvm_vcpu *vcpu, int cpu);
void avic_vcpu_put(struct kvm_vcpu *vcpu);
void avic_apicv_post_state_restore(struct kvm_vcpu *vcpu);
void avic_refresh_apicv_exec_ctrl(struct kvm_vcpu *vcpu);
int avic_pi_update_irte(struct kvm_kernel_irqfd *irqfd, struct kvm *kvm,
			unsigned int host_irq, uint32_t guest_irq,
			struct kvm_vcpu *vcpu, u32 vector);
void avic_vcpu_blocking(struct kvm_vcpu *vcpu);
void avic_vcpu_unblocking(struct kvm_vcpu *vcpu);
void avic_ring_doorbell(struct kvm_vcpu *vcpu);
unsigned long avic_vcpu_get_apicv_inhibit_reasons(struct kvm_vcpu *vcpu);
void avic_refresh_virtual_apic_mode(struct kvm_vcpu *vcpu);


/* sev.c */

int pre_sev_run(struct vcpu_svm *svm, int cpu);
void sev_init_vmcb(struct vcpu_svm *svm, bool init_event);
void sev_vcpu_after_set_cpuid(struct vcpu_svm *svm);
int sev_es_string_io(struct vcpu_svm *svm, int size, unsigned int port, int in);
void sev_es_recalc_msr_intercepts(struct kvm_vcpu *vcpu);
void sev_vcpu_deliver_sipi_vector(struct kvm_vcpu *vcpu, u8 vector);
void sev_es_prepare_switch_to_guest(struct vcpu_svm *svm, struct sev_es_save_area *hostsa);
void sev_es_unmap_ghcb(struct vcpu_svm *svm);

#ifdef CONFIG_KVM_AMD_SEV
int sev_mem_enc_ioctl(struct kvm *kvm, void __user *argp);
int sev_mem_enc_register_region(struct kvm *kvm,
				struct kvm_enc_region *range);
int sev_mem_enc_unregister_region(struct kvm *kvm,
				  struct kvm_enc_region *range);
int sev_vm_copy_enc_context_from(struct kvm *kvm, unsigned int source_fd);
int sev_vm_move_enc_context_from(struct kvm *kvm, unsigned int source_fd);
void sev_guest_memory_reclaimed(struct kvm *kvm);
int sev_handle_vmgexit(struct kvm_vcpu *vcpu);

/* These symbols are used in common code and are stubbed below.  */

struct page *snp_safe_alloc_page_node(int node, gfp_t gfp);
static inline struct page *snp_safe_alloc_page(void)
{
	return snp_safe_alloc_page_node(numa_node_id(), GFP_KERNEL_ACCOUNT);
}

int sev_vcpu_create(struct kvm_vcpu *vcpu);
void sev_free_vcpu(struct kvm_vcpu *vcpu);
void sev_vm_destroy(struct kvm *kvm);
void __init sev_set_cpu_caps(void);
void __init sev_hardware_setup(void);
void sev_hardware_unsetup(void);
int sev_cpu_init(struct svm_cpu_data *sd);
int sev_dev_get_attr(u32 group, u64 attr, u64 *val);
extern unsigned int max_sev_asid;
void sev_handle_rmp_fault(struct kvm_vcpu *vcpu, gpa_t gpa, u64 error_code);
int sev_gmem_prepare(struct kvm *kvm, kvm_pfn_t pfn, gfn_t gfn, int max_order);
void sev_gmem_invalidate(kvm_pfn_t start, kvm_pfn_t end);
int sev_gmem_max_mapping_level(struct kvm *kvm, kvm_pfn_t pfn, bool is_private);
struct vmcb_save_area *sev_decrypt_vmsa(struct kvm_vcpu *vcpu);
void sev_free_decrypted_vmsa(struct kvm_vcpu *vcpu, struct vmcb_save_area *vmsa);
#else
static inline struct page *snp_safe_alloc_page_node(int node, gfp_t gfp)
{
	return alloc_pages_node(node, gfp | __GFP_ZERO, 0);
}

static inline struct page *snp_safe_alloc_page(void)
{
	return snp_safe_alloc_page_node(numa_node_id(), GFP_KERNEL_ACCOUNT);
}

static inline int sev_vcpu_create(struct kvm_vcpu *vcpu) { return 0; }
static inline void sev_free_vcpu(struct kvm_vcpu *vcpu) {}
static inline void sev_vm_destroy(struct kvm *kvm) {}
static inline void __init sev_set_cpu_caps(void) {}
static inline void __init sev_hardware_setup(void) {}
static inline void sev_hardware_unsetup(void) {}
static inline int sev_cpu_init(struct svm_cpu_data *sd) { return 0; }
static inline int sev_dev_get_attr(u32 group, u64 attr, u64 *val) { return -ENXIO; }
#define max_sev_asid 0
static inline void sev_handle_rmp_fault(struct kvm_vcpu *vcpu, gpa_t gpa, u64 error_code) {}
static inline int sev_gmem_prepare(struct kvm *kvm, kvm_pfn_t pfn, gfn_t gfn, int max_order)
{
	return 0;
}
static inline void sev_gmem_invalidate(kvm_pfn_t start, kvm_pfn_t end) {}
static inline int sev_gmem_max_mapping_level(struct kvm *kvm, kvm_pfn_t pfn, bool is_private)
{
	return 0;
}

static inline struct vmcb_save_area *sev_decrypt_vmsa(struct kvm_vcpu *vcpu)
{
	return NULL;
}
static inline void sev_free_decrypted_vmsa(struct kvm_vcpu *vcpu, struct vmcb_save_area *vmsa) {}
#endif

/* vmenter.S */

void __svm_sev_es_vcpu_run(struct vcpu_svm *svm, bool spec_ctrl_intercepted,
			   struct sev_es_save_area *hostsa);
void __svm_vcpu_run(struct vcpu_svm *svm, bool spec_ctrl_intercepted);

#define DEFINE_KVM_GHCB_ACCESSORS(field)						\
static __always_inline u64 kvm_ghcb_get_##field(struct vcpu_svm *svm)			\
{											\
	return READ_ONCE(svm->sev_es.ghcb->save.field);					\
}											\
											\
static __always_inline bool kvm_ghcb_##field##_is_valid(const struct vcpu_svm *svm)	\
{											\
	return test_bit(GHCB_BITMAP_IDX(field),						\
			(unsigned long *)&svm->sev_es.valid_bitmap);			\
}											\
											\
static __always_inline u64 kvm_ghcb_get_##field##_if_valid(struct vcpu_svm *svm)	\
{											\
	return kvm_ghcb_##field##_is_valid(svm) ? kvm_ghcb_get_##field(svm) : 0;	\
}

DEFINE_KVM_GHCB_ACCESSORS(cpl)
DEFINE_KVM_GHCB_ACCESSORS(rax)
DEFINE_KVM_GHCB_ACCESSORS(rcx)
DEFINE_KVM_GHCB_ACCESSORS(rdx)
DEFINE_KVM_GHCB_ACCESSORS(rbx)
DEFINE_KVM_GHCB_ACCESSORS(rsi)
DEFINE_KVM_GHCB_ACCESSORS(sw_exit_code)
DEFINE_KVM_GHCB_ACCESSORS(sw_exit_info_1)
DEFINE_KVM_GHCB_ACCESSORS(sw_exit_info_2)
DEFINE_KVM_GHCB_ACCESSORS(sw_scratch)
DEFINE_KVM_GHCB_ACCESSORS(xcr0)
DEFINE_KVM_GHCB_ACCESSORS(xss)

#endif
