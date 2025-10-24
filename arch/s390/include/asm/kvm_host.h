/* SPDX-License-Identifier: GPL-2.0 */
/*
 * definition for kernel virtual machines on s390
 *
 * Copyright IBM Corp. 2008, 2018
 *
 *    Author(s): Carsten Otte <cotte@de.ibm.com>
 */


#ifndef ASM_KVM_HOST_H
#define ASM_KVM_HOST_H

#include <linux/types.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kvm_types.h>
#include <linux/kvm.h>
#include <linux/seqlock.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mmu_notifier.h>
#include <asm/kvm_host_types.h>
#include <asm/debug.h>
#include <asm/cpu.h>
#include <asm/fpu.h>
#include <asm/isc.h>
#include <asm/guarded_storage.h>

#define KVM_MAX_VCPUS 255

#define KVM_INTERNAL_MEM_SLOTS 1

/*
 * These seem to be used for allocating ->chip in the routing table, which we
 * don't use. 1 is as small as we can get to reduce the needed memory. If we
 * need to look at ->chip later on, we'll need to revisit this.
 */
#define KVM_NR_IRQCHIPS 1
#define KVM_IRQCHIP_NUM_PINS 1
#define KVM_HALT_POLL_NS_DEFAULT 50000

/* s390-specific vcpu->requests bit members */
#define KVM_REQ_ENABLE_IBS	KVM_ARCH_REQ(0)
#define KVM_REQ_DISABLE_IBS	KVM_ARCH_REQ(1)
#define KVM_REQ_ICPT_OPEREXC	KVM_ARCH_REQ(2)
#define KVM_REQ_START_MIGRATION KVM_ARCH_REQ(3)
#define KVM_REQ_STOP_MIGRATION  KVM_ARCH_REQ(4)
#define KVM_REQ_VSIE_RESTART	KVM_ARCH_REQ(5)
#define KVM_REQ_REFRESH_GUEST_PREFIX	\
	KVM_ARCH_REQ_FLAGS(6, KVM_REQUEST_WAIT | KVM_REQUEST_NO_WAKEUP)

struct kvm_vcpu_stat {
	struct kvm_vcpu_stat_generic generic;
	u64 exit_userspace;
	u64 exit_null;
	u64 exit_external_request;
	u64 exit_io_request;
	u64 exit_external_interrupt;
	u64 exit_stop_request;
	u64 exit_validity;
	u64 exit_instruction;
	u64 exit_pei;
	u64 halt_no_poll_steal;
	u64 instruction_lctl;
	u64 instruction_lctlg;
	u64 instruction_stctl;
	u64 instruction_stctg;
	u64 exit_program_interruption;
	u64 exit_instr_and_program;
	u64 exit_operation_exception;
	u64 deliver_ckc;
	u64 deliver_cputm;
	u64 deliver_external_call;
	u64 deliver_emergency_signal;
	u64 deliver_service_signal;
	u64 deliver_virtio;
	u64 deliver_stop_signal;
	u64 deliver_prefix_signal;
	u64 deliver_restart_signal;
	u64 deliver_program;
	u64 deliver_io;
	u64 deliver_machine_check;
	u64 exit_wait_state;
	u64 inject_ckc;
	u64 inject_cputm;
	u64 inject_external_call;
	u64 inject_emergency_signal;
	u64 inject_mchk;
	u64 inject_pfault_init;
	u64 inject_program;
	u64 inject_restart;
	u64 inject_set_prefix;
	u64 inject_stop_signal;
	u64 instruction_epsw;
	u64 instruction_gs;
	u64 instruction_io_other;
	u64 instruction_lpsw;
	u64 instruction_lpswe;
	u64 instruction_lpswey;
	u64 instruction_pfmf;
	u64 instruction_ptff;
	u64 instruction_sck;
	u64 instruction_sckpf;
	u64 instruction_stidp;
	u64 instruction_spx;
	u64 instruction_stpx;
	u64 instruction_stap;
	u64 instruction_iske;
	u64 instruction_ri;
	u64 instruction_rrbe;
	u64 instruction_sske;
	u64 instruction_ipte_interlock;
	u64 instruction_stsi;
	u64 instruction_stfl;
	u64 instruction_tb;
	u64 instruction_tpi;
	u64 instruction_tprot;
	u64 instruction_tsch;
	u64 instruction_sie;
	u64 instruction_essa;
	u64 instruction_sthyi;
	u64 instruction_sigp_sense;
	u64 instruction_sigp_sense_running;
	u64 instruction_sigp_external_call;
	u64 instruction_sigp_emergency;
	u64 instruction_sigp_cond_emergency;
	u64 instruction_sigp_start;
	u64 instruction_sigp_stop;
	u64 instruction_sigp_stop_store_status;
	u64 instruction_sigp_store_status;
	u64 instruction_sigp_store_adtl_status;
	u64 instruction_sigp_arch;
	u64 instruction_sigp_prefix;
	u64 instruction_sigp_restart;
	u64 instruction_sigp_init_cpu_reset;
	u64 instruction_sigp_cpu_reset;
	u64 instruction_sigp_unknown;
	u64 instruction_diagnose_10;
	u64 instruction_diagnose_44;
	u64 instruction_diagnose_9c;
	u64 diag_9c_ignored;
	u64 diag_9c_forward;
	u64 instruction_diagnose_258;
	u64 instruction_diagnose_308;
	u64 instruction_diagnose_500;
	u64 instruction_diagnose_other;
	u64 pfault_sync;
};

#define PGM_OPERATION			0x01
#define PGM_PRIVILEGED_OP		0x02
#define PGM_EXECUTE			0x03
#define PGM_PROTECTION			0x04
#define PGM_ADDRESSING			0x05
#define PGM_SPECIFICATION		0x06
#define PGM_DATA			0x07
#define PGM_FIXED_POINT_OVERFLOW	0x08
#define PGM_FIXED_POINT_DIVIDE		0x09
#define PGM_DECIMAL_OVERFLOW		0x0a
#define PGM_DECIMAL_DIVIDE		0x0b
#define PGM_HFP_EXPONENT_OVERFLOW	0x0c
#define PGM_HFP_EXPONENT_UNDERFLOW	0x0d
#define PGM_HFP_SIGNIFICANCE		0x0e
#define PGM_HFP_DIVIDE			0x0f
#define PGM_SEGMENT_TRANSLATION		0x10
#define PGM_PAGE_TRANSLATION		0x11
#define PGM_TRANSLATION_SPEC		0x12
#define PGM_SPECIAL_OPERATION		0x13
#define PGM_OPERAND			0x15
#define PGM_TRACE_TABEL			0x16
#define PGM_VECTOR_PROCESSING		0x1b
#define PGM_SPACE_SWITCH		0x1c
#define PGM_HFP_SQUARE_ROOT		0x1d
#define PGM_PC_TRANSLATION_SPEC		0x1f
#define PGM_AFX_TRANSLATION		0x20
#define PGM_ASX_TRANSLATION		0x21
#define PGM_LX_TRANSLATION		0x22
#define PGM_EX_TRANSLATION		0x23
#define PGM_PRIMARY_AUTHORITY		0x24
#define PGM_SECONDARY_AUTHORITY		0x25
#define PGM_LFX_TRANSLATION		0x26
#define PGM_LSX_TRANSLATION		0x27
#define PGM_ALET_SPECIFICATION		0x28
#define PGM_ALEN_TRANSLATION		0x29
#define PGM_ALE_SEQUENCE		0x2a
#define PGM_ASTE_VALIDITY		0x2b
#define PGM_ASTE_SEQUENCE		0x2c
#define PGM_EXTENDED_AUTHORITY		0x2d
#define PGM_LSTE_SEQUENCE		0x2e
#define PGM_ASTE_INSTANCE		0x2f
#define PGM_STACK_FULL			0x30
#define PGM_STACK_EMPTY			0x31
#define PGM_STACK_SPECIFICATION		0x32
#define PGM_STACK_TYPE			0x33
#define PGM_STACK_OPERATION		0x34
#define PGM_ASCE_TYPE			0x38
#define PGM_REGION_FIRST_TRANS		0x39
#define PGM_REGION_SECOND_TRANS		0x3a
#define PGM_REGION_THIRD_TRANS		0x3b
#define PGM_SECURE_STORAGE_ACCESS	0x3d
#define PGM_NON_SECURE_STORAGE_ACCESS	0x3e
#define PGM_SECURE_STORAGE_VIOLATION	0x3f
#define PGM_MONITOR			0x40
#define PGM_PER				0x80
#define PGM_CRYPTO_OPERATION		0x119

/* irq types in ascend order of priorities */
enum irq_types {
	IRQ_PEND_SET_PREFIX = 0,
	IRQ_PEND_RESTART,
	IRQ_PEND_SIGP_STOP,
	IRQ_PEND_IO_ISC_7,
	IRQ_PEND_IO_ISC_6,
	IRQ_PEND_IO_ISC_5,
	IRQ_PEND_IO_ISC_4,
	IRQ_PEND_IO_ISC_3,
	IRQ_PEND_IO_ISC_2,
	IRQ_PEND_IO_ISC_1,
	IRQ_PEND_IO_ISC_0,
	IRQ_PEND_VIRTIO,
	IRQ_PEND_PFAULT_DONE,
	IRQ_PEND_PFAULT_INIT,
	IRQ_PEND_EXT_HOST,
	IRQ_PEND_EXT_SERVICE,
	IRQ_PEND_EXT_SERVICE_EV,
	IRQ_PEND_EXT_TIMING,
	IRQ_PEND_EXT_CPU_TIMER,
	IRQ_PEND_EXT_CLOCK_COMP,
	IRQ_PEND_EXT_EXTERNAL,
	IRQ_PEND_EXT_EMERGENCY,
	IRQ_PEND_EXT_MALFUNC,
	IRQ_PEND_EXT_IRQ_KEY,
	IRQ_PEND_MCHK_REP,
	IRQ_PEND_PROG,
	IRQ_PEND_SVC,
	IRQ_PEND_MCHK_EX,
	IRQ_PEND_COUNT
};

/* We have 2M for virtio device descriptor pages. Smallest amount of
 * memory per page is 24 bytes (1 queue), so (2048*1024) / 24 = 87381
 */
#define KVM_S390_MAX_VIRTIO_IRQS 87381

/*
 * Repressible (non-floating) machine check interrupts
 * subclass bits in MCIC
 */
#define MCHK_EXTD_BIT 58
#define MCHK_DEGR_BIT 56
#define MCHK_WARN_BIT 55
#define MCHK_REP_MASK ((1UL << MCHK_DEGR_BIT) | \
		       (1UL << MCHK_EXTD_BIT) | \
		       (1UL << MCHK_WARN_BIT))

/* Exigent machine check interrupts subclass bits in MCIC */
#define MCHK_SD_BIT 63
#define MCHK_PD_BIT 62
#define MCHK_EX_MASK ((1UL << MCHK_SD_BIT) | (1UL << MCHK_PD_BIT))

#define IRQ_PEND_EXT_MASK ((1UL << IRQ_PEND_EXT_IRQ_KEY)    | \
			   (1UL << IRQ_PEND_EXT_CLOCK_COMP) | \
			   (1UL << IRQ_PEND_EXT_CPU_TIMER)  | \
			   (1UL << IRQ_PEND_EXT_MALFUNC)    | \
			   (1UL << IRQ_PEND_EXT_EMERGENCY)  | \
			   (1UL << IRQ_PEND_EXT_EXTERNAL)   | \
			   (1UL << IRQ_PEND_EXT_TIMING)     | \
			   (1UL << IRQ_PEND_EXT_HOST)       | \
			   (1UL << IRQ_PEND_EXT_SERVICE)    | \
			   (1UL << IRQ_PEND_EXT_SERVICE_EV) | \
			   (1UL << IRQ_PEND_VIRTIO)         | \
			   (1UL << IRQ_PEND_PFAULT_INIT)    | \
			   (1UL << IRQ_PEND_PFAULT_DONE))

#define IRQ_PEND_IO_MASK ((1UL << IRQ_PEND_IO_ISC_0) | \
			  (1UL << IRQ_PEND_IO_ISC_1) | \
			  (1UL << IRQ_PEND_IO_ISC_2) | \
			  (1UL << IRQ_PEND_IO_ISC_3) | \
			  (1UL << IRQ_PEND_IO_ISC_4) | \
			  (1UL << IRQ_PEND_IO_ISC_5) | \
			  (1UL << IRQ_PEND_IO_ISC_6) | \
			  (1UL << IRQ_PEND_IO_ISC_7))

#define IRQ_PEND_MCHK_MASK ((1UL << IRQ_PEND_MCHK_REP) | \
			    (1UL << IRQ_PEND_MCHK_EX))

#define IRQ_PEND_EXT_II_MASK ((1UL << IRQ_PEND_EXT_CPU_TIMER)  | \
			      (1UL << IRQ_PEND_EXT_CLOCK_COMP) | \
			      (1UL << IRQ_PEND_EXT_EMERGENCY)  | \
			      (1UL << IRQ_PEND_EXT_EXTERNAL)   | \
			      (1UL << IRQ_PEND_EXT_SERVICE)    | \
			      (1UL << IRQ_PEND_EXT_SERVICE_EV))

struct kvm_s390_interrupt_info {
	struct list_head list;
	u64	type;
	union {
		struct kvm_s390_io_info io;
		struct kvm_s390_ext_info ext;
		struct kvm_s390_pgm_info pgm;
		struct kvm_s390_emerg_info emerg;
		struct kvm_s390_extcall_info extcall;
		struct kvm_s390_prefix_info prefix;
		struct kvm_s390_stop_info stop;
		struct kvm_s390_mchk_info mchk;
	};
};

struct kvm_s390_irq_payload {
	struct kvm_s390_io_info io;
	struct kvm_s390_ext_info ext;
	struct kvm_s390_pgm_info pgm;
	struct kvm_s390_emerg_info emerg;
	struct kvm_s390_extcall_info extcall;
	struct kvm_s390_prefix_info prefix;
	struct kvm_s390_stop_info stop;
	struct kvm_s390_mchk_info mchk;
};

struct kvm_s390_local_interrupt {
	spinlock_t lock;
	DECLARE_BITMAP(sigp_emerg_pending, KVM_MAX_VCPUS);
	struct kvm_s390_irq_payload irq;
	unsigned long pending_irqs;
};

#define FIRQ_LIST_IO_ISC_0 0
#define FIRQ_LIST_IO_ISC_1 1
#define FIRQ_LIST_IO_ISC_2 2
#define FIRQ_LIST_IO_ISC_3 3
#define FIRQ_LIST_IO_ISC_4 4
#define FIRQ_LIST_IO_ISC_5 5
#define FIRQ_LIST_IO_ISC_6 6
#define FIRQ_LIST_IO_ISC_7 7
#define FIRQ_LIST_PFAULT   8
#define FIRQ_LIST_VIRTIO   9
#define FIRQ_LIST_COUNT   10
#define FIRQ_CNTR_IO       0
#define FIRQ_CNTR_SERVICE  1
#define FIRQ_CNTR_VIRTIO   2
#define FIRQ_CNTR_PFAULT   3
#define FIRQ_MAX_COUNT     4

/* mask the AIS mode for a given ISC */
#define AIS_MODE_MASK(isc) (0x80 >> isc)

#define KVM_S390_AIS_MODE_ALL    0
#define KVM_S390_AIS_MODE_SINGLE 1

struct kvm_s390_float_interrupt {
	unsigned long pending_irqs;
	unsigned long masked_irqs;
	spinlock_t lock;
	struct list_head lists[FIRQ_LIST_COUNT];
	int counters[FIRQ_MAX_COUNT];
	struct kvm_s390_mchk_info mchk;
	struct kvm_s390_ext_info srv_signal;
	int last_sleep_cpu;
	struct mutex ais_lock;
	u8 simm;
	u8 nimm;
};

struct kvm_hw_wp_info_arch {
	unsigned long addr;
	unsigned long phys_addr;
	int len;
	char *old_data;
};

struct kvm_hw_bp_info_arch {
	unsigned long addr;
	int len;
};

/*
 * Only the upper 16 bits of kvm_guest_debug->control are arch specific.
 * Further KVM_GUESTDBG flags which an be used from userspace can be found in
 * arch/s390/include/uapi/asm/kvm.h
 */
#define KVM_GUESTDBG_EXIT_PENDING 0x10000000

#define guestdbg_enabled(vcpu) \
		(vcpu->guest_debug & KVM_GUESTDBG_ENABLE)
#define guestdbg_sstep_enabled(vcpu) \
		(vcpu->guest_debug & KVM_GUESTDBG_SINGLESTEP)
#define guestdbg_hw_bp_enabled(vcpu) \
		(vcpu->guest_debug & KVM_GUESTDBG_USE_HW_BP)
#define guestdbg_exit_pending(vcpu) (guestdbg_enabled(vcpu) && \
		(vcpu->guest_debug & KVM_GUESTDBG_EXIT_PENDING))

#define KVM_GUESTDBG_VALID_MASK \
		(KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_SINGLESTEP |\
		KVM_GUESTDBG_USE_HW_BP | KVM_GUESTDBG_EXIT_PENDING)

struct kvm_guestdbg_info_arch {
	unsigned long cr0;
	unsigned long cr9;
	unsigned long cr10;
	unsigned long cr11;
	struct kvm_hw_bp_info_arch *hw_bp_info;
	struct kvm_hw_wp_info_arch *hw_wp_info;
	int nr_hw_bp;
	int nr_hw_wp;
	unsigned long last_bp;
};

struct kvm_s390_pv_vcpu {
	u64 handle;
	unsigned long stor_base;
};

struct kvm_vcpu_arch {
	struct kvm_s390_sie_block *sie_block;
	/* if vsie is active, currently executed shadow sie control block */
	struct kvm_s390_sie_block *vsie_block;
	unsigned int      host_acrs[NUM_ACRS];
	struct gs_cb      *host_gscb;
	struct kvm_s390_local_interrupt local_int;
	struct hrtimer    ckc_timer;
	struct kvm_s390_pgm_info pgm;
	struct gmap *gmap;
	struct kvm_guestdbg_info_arch guestdbg;
	unsigned long pfault_token;
	unsigned long pfault_select;
	unsigned long pfault_compare;
	bool cputm_enabled;
	/*
	 * The seqcount protects updates to cputm_start and sie_block.cputm,
	 * this way we can have non-blocking reads with consistent values.
	 * Only the owning VCPU thread (vcpu->cpu) is allowed to change these
	 * values and to start/stop/enable/disable cpu timer accounting.
	 */
	seqcount_t cputm_seqcount;
	__u64 cputm_start;
	bool gs_enabled;
	bool skey_enabled;
	/* Indicator if the access registers have been loaded from guest */
	bool acrs_loaded;
	struct kvm_s390_pv_vcpu pv;
	union diag318_info diag318_info;
};

struct kvm_vm_stat {
	struct kvm_vm_stat_generic generic;
	u64 inject_io;
	u64 inject_float_mchk;
	u64 inject_pfault_done;
	u64 inject_service_signal;
	u64 inject_virtio;
	u64 aen_forward;
	u64 gmap_shadow_create;
	u64 gmap_shadow_reuse;
	u64 gmap_shadow_r1_entry;
	u64 gmap_shadow_r2_entry;
	u64 gmap_shadow_r3_entry;
	u64 gmap_shadow_sg_entry;
	u64 gmap_shadow_pg_entry;
};

struct kvm_arch_memory_slot {
};

struct s390_map_info {
	struct list_head list;
	__u64 guest_addr;
	__u64 addr;
	struct page *page;
};

struct s390_io_adapter {
	unsigned int id;
	int isc;
	bool maskable;
	bool masked;
	bool swap;
	bool suppressible;
};

#define MAX_S390_IO_ADAPTERS ((MAX_ISC + 1) * 8)
#define MAX_S390_ADAPTER_MAPS 256

/* maximum size of facilities and facility mask is 2k bytes */
#define S390_ARCH_FAC_LIST_SIZE_BYTE (1<<11)
#define S390_ARCH_FAC_LIST_SIZE_U64 \
	(S390_ARCH_FAC_LIST_SIZE_BYTE / sizeof(u64))
#define S390_ARCH_FAC_MASK_SIZE_BYTE S390_ARCH_FAC_LIST_SIZE_BYTE
#define S390_ARCH_FAC_MASK_SIZE_U64 \
	(S390_ARCH_FAC_MASK_SIZE_BYTE / sizeof(u64))

struct kvm_s390_cpu_model {
	/* facility mask supported by kvm & hosting machine */
	__u64 fac_mask[S390_ARCH_FAC_MASK_SIZE_U64];
	struct kvm_s390_vm_cpu_subfunc subfuncs;
	/* facility list requested by guest (in dma page) */
	__u64 *fac_list;
	u64 cpuid;
	unsigned short ibc;
	/* subset of available UV-features for pv-guests enabled by user space */
	struct kvm_s390_vm_cpu_uv_feat uv_feat_guest;
};

typedef int (*crypto_hook)(struct kvm_vcpu *vcpu);

struct kvm_s390_crypto {
	struct kvm_s390_crypto_cb *crycb;
	struct rw_semaphore pqap_hook_rwsem;
	crypto_hook *pqap_hook;
	__u32 crycbd;
	__u8 aes_kw;
	__u8 dea_kw;
	__u8 apie;
};

#define APCB0_MASK_SIZE 1
struct kvm_s390_apcb0 {
	__u64 apm[APCB0_MASK_SIZE];		/* 0x0000 */
	__u64 aqm[APCB0_MASK_SIZE];		/* 0x0008 */
	__u64 adm[APCB0_MASK_SIZE];		/* 0x0010 */
	__u64 reserved18;			/* 0x0018 */
};

#define APCB1_MASK_SIZE 4
struct kvm_s390_apcb1 {
	__u64 apm[APCB1_MASK_SIZE];		/* 0x0000 */
	__u64 aqm[APCB1_MASK_SIZE];		/* 0x0020 */
	__u64 adm[APCB1_MASK_SIZE];		/* 0x0040 */
	__u64 reserved60[4];			/* 0x0060 */
};

struct kvm_s390_crypto_cb {
	struct kvm_s390_apcb0 apcb0;		/* 0x0000 */
	__u8   reserved20[0x0048 - 0x0020];	/* 0x0020 */
	__u8   dea_wrapping_key_mask[24];	/* 0x0048 */
	__u8   aes_wrapping_key_mask[32];	/* 0x0060 */
	struct kvm_s390_apcb1 apcb1;		/* 0x0080 */
};

struct kvm_s390_gisa {
	union {
		struct { /* common to all formats */
			u32 next_alert;
			u8  ipm;
			u8  reserved01[2];
			u8  iam;
		};
		struct { /* format 0 */
			u32 next_alert;
			u8  ipm;
			u8  reserved01;
			u8  : 6;
			u8  g : 1;
			u8  c : 1;
			u8  iam;
			u8  reserved02[4];
			u32 airq_count;
		} g0;
		struct { /* format 1 */
			u32 next_alert;
			u8  ipm;
			u8  simm;
			u8  nimm;
			u8  iam;
			u8  aism[8];
			u8  : 6;
			u8  g : 1;
			u8  c : 1;
			u8  reserved03[11];
			u32 airq_count;
		} g1;
		struct {
			u64 word[4];
		} u64;
	};
};

struct kvm_s390_gib {
	u32 alert_list_origin;
	u32 reserved01;
	u8:5;
	u8  nisc:3;
	u8  reserved03[3];
	u32 reserved04[5];
};

/*
 * sie_page2 has to be allocated as DMA because fac_list, crycb and
 * gisa need 31bit addresses in the sie control block.
 */
struct sie_page2 {
	__u64 fac_list[S390_ARCH_FAC_LIST_SIZE_U64];	/* 0x0000 */
	struct kvm_s390_crypto_cb crycb;		/* 0x0800 */
	struct kvm_s390_gisa gisa;			/* 0x0900 */
	struct kvm *kvm;				/* 0x0920 */
	u8 reserved928[0x1000 - 0x928];			/* 0x0928 */
};

struct vsie_page;

struct kvm_s390_vsie {
	struct mutex mutex;
	struct radix_tree_root addr_to_page;
	int page_count;
	int next;
	struct vsie_page *pages[KVM_MAX_VCPUS];
};

struct kvm_s390_gisa_iam {
	u8 mask;
	spinlock_t ref_lock;
	u32 ref_count[MAX_ISC + 1];
};

struct kvm_s390_gisa_interrupt {
	struct kvm_s390_gisa *origin;
	struct kvm_s390_gisa_iam alert;
	struct hrtimer timer;
	u64 expires;
	DECLARE_BITMAP(kicked_mask, KVM_MAX_VCPUS);
};

struct kvm_s390_pv {
	u64 handle;
	u64 guest_len;
	unsigned long stor_base;
	void *stor_var;
	bool dumping;
	void *set_aside;
	struct list_head need_cleanup;
	struct mmu_notifier mmu_notifier;
};

struct kvm_arch {
	struct esca_block *sca;
	debug_info_t *dbf;
	struct kvm_s390_float_interrupt float_int;
	struct kvm_device *flic;
	struct gmap *gmap;
	unsigned long mem_limit;
	int css_support;
	int use_irqchip;
	int use_cmma;
	int use_pfmfi;
	int use_skf;
	int use_zpci_interp;
	int user_cpu_state_ctrl;
	int user_sigp;
	int user_stsi;
	int user_instr0;
	struct s390_io_adapter *adapters[MAX_S390_IO_ADAPTERS];
	wait_queue_head_t ipte_wq;
	int ipte_lock_count;
	struct mutex ipte_mutex;
	spinlock_t start_stop_lock;
	struct sie_page2 *sie_page2;
	struct kvm_s390_cpu_model model;
	struct kvm_s390_crypto crypto;
	struct kvm_s390_vsie vsie;
	u8 epdx;
	u64 epoch;
	int migration_mode;
	atomic64_t cmma_dirty_pages;
	/* subset of available cpu features enabled by user space */
	DECLARE_BITMAP(cpu_feat, KVM_S390_VM_CPU_FEAT_NR_BITS);
	/* indexed by vcpu_idx */
	DECLARE_BITMAP(idle_mask, KVM_MAX_VCPUS);
	struct kvm_s390_gisa_interrupt gisa_int;
	struct kvm_s390_pv pv;
	struct list_head kzdev_list;
	spinlock_t kzdev_list_lock;
};

#define KVM_HVA_ERR_BAD		(-1UL)
#define KVM_HVA_ERR_RO_BAD	(-2UL)

static inline bool kvm_is_error_hva(unsigned long addr)
{
	return IS_ERR_VALUE(addr);
}

#define ASYNC_PF_PER_VCPU	64
struct kvm_arch_async_pf {
	unsigned long pfault_token;
};

bool kvm_arch_can_dequeue_async_page_present(struct kvm_vcpu *vcpu);

void kvm_arch_async_page_ready(struct kvm_vcpu *vcpu,
			       struct kvm_async_pf *work);

bool kvm_arch_async_page_not_present(struct kvm_vcpu *vcpu,
				     struct kvm_async_pf *work);

void kvm_arch_async_page_present(struct kvm_vcpu *vcpu,
				 struct kvm_async_pf *work);

static inline void kvm_arch_async_page_present_queued(struct kvm_vcpu *vcpu) {}

void kvm_arch_crypto_clear_masks(struct kvm *kvm);
void kvm_arch_crypto_set_masks(struct kvm *kvm, unsigned long *apm,
			       unsigned long *aqm, unsigned long *adm);

int __sie64a(phys_addr_t sie_block_phys, struct kvm_s390_sie_block *sie_block, u64 *rsa,
	     unsigned long gasce);

static inline int sie64a(struct kvm_s390_sie_block *sie_block, u64 *rsa, unsigned long gasce)
{
	return __sie64a(virt_to_phys(sie_block), sie_block, rsa, gasce);
}

extern char sie_exit;

bool kvm_s390_pv_is_protected(struct kvm *kvm);
bool kvm_s390_pv_cpu_is_protected(struct kvm_vcpu *vcpu);

extern int kvm_s390_enter_exit_sie(struct kvm_s390_sie_block *scb,
				   u64 *gprs, unsigned long gasce);

extern int kvm_s390_gisc_register(struct kvm *kvm, u32 gisc);
extern int kvm_s390_gisc_unregister(struct kvm *kvm, u32 gisc);

bool kvm_s390_is_gpa_in_memslot(struct kvm *kvm, gpa_t gpa);

static inline void kvm_arch_free_memslot(struct kvm *kvm,
					 struct kvm_memory_slot *slot) {}
static inline void kvm_arch_memslots_updated(struct kvm *kvm, u64 gen) {}
static inline void kvm_arch_flush_shadow_all(struct kvm *kvm) {}
static inline void kvm_arch_flush_shadow_memslot(struct kvm *kvm,
		struct kvm_memory_slot *slot) {}
static inline void kvm_arch_vcpu_blocking(struct kvm_vcpu *vcpu) {}
static inline void kvm_arch_vcpu_unblocking(struct kvm_vcpu *vcpu) {}

#define __KVM_HAVE_ARCH_VM_FREE
void kvm_arch_free_vm(struct kvm *kvm);

struct zpci_kvm_hook {
	int (*kvm_register)(void *opaque, struct kvm *kvm);
	void (*kvm_unregister)(void *opaque);
};

extern struct zpci_kvm_hook zpci_kvm_hook;

#endif
