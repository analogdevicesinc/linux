/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_MSHYPER_H
#define _ASM_X86_MSHYPER_H

#include <linux/types.h>
#include <linux/nmi.h>
#include <linux/msi.h>
#include <linux/io.h>
#include <linux/static_call.h>
#include <asm/nospec-branch.h>
#include <asm/paravirt.h>
#include <asm/msr.h>
#include <hyperv/hvhdk.h>

/*
 * Hyper-V always provides a single IO-APIC at this MMIO address.
 * Ideally, the value should be looked up in ACPI tables, but it
 * is needed for mapping the IO-APIC early in boot on Confidential
 * VMs, before ACPI functions can be used.
 */
#define HV_IOAPIC_BASE_ADDRESS 0xfec00000

#define HV_VTL_NORMAL 0x0
#define HV_VTL_SECURE 0x1
#define HV_VTL_MGMT   0x2

union hv_ghcb;

DECLARE_STATIC_KEY_FALSE(isolation_type_snp);
DECLARE_STATIC_KEY_FALSE(isolation_type_tdx);

typedef int (*hyperv_fill_flush_list_func)(
		struct hv_guest_mapping_flush_list *flush,
		void *data);

void hyperv_vector_handler(struct pt_regs *regs);

static inline unsigned char hv_get_nmi_reason(void)
{
	return 0;
}

extern u64 hv_tdx_hypercall(u64 control, u64 param1, u64 param2);
extern u64 hv_snp_hypercall(u64 control, u64 param1, u64 param2);
extern u64 hv_std_hypercall(u64 control, u64 param1, u64 param2);

#if IS_ENABLED(CONFIG_HYPERV)
extern void *hv_hypercall_pg;

extern union hv_ghcb * __percpu *hv_ghcb_pg;

bool hv_isolation_type_snp(void);
bool hv_isolation_type_tdx(void);

#ifdef CONFIG_X86_64
DECLARE_STATIC_CALL(hv_hypercall, hv_std_hypercall);
#endif

/*
 * DEFAULT INIT GPAT and SEGMENT LIMIT value in struct VMSA
 * to start AP in enlightened SEV guest.
 */
#define HV_AP_INIT_GPAT_DEFAULT		0x0007040600070406ULL
#define HV_AP_SEGMENT_LIMIT		0xffffffff

/*
 * If the hypercall involves no input or output parameters, the hypervisor
 * ignores the corresponding GPA pointer.
 */
static inline u64 hv_do_hypercall(u64 control, void *input, void *output)
{
	u64 input_address = input ? virt_to_phys(input) : 0;
	u64 output_address = output ? virt_to_phys(output) : 0;

#ifdef CONFIG_X86_64
	return static_call_mod(hv_hypercall)(control, input_address, output_address);
#else
	u32 input_address_hi = upper_32_bits(input_address);
	u32 input_address_lo = lower_32_bits(input_address);
	u32 output_address_hi = upper_32_bits(output_address);
	u32 output_address_lo = lower_32_bits(output_address);
	u64 hv_status;

	if (!hv_hypercall_pg)
		return U64_MAX;

	__asm__ __volatile__(CALL_NOSPEC
			     : "=A" (hv_status),
			       "+c" (input_address_lo), ASM_CALL_CONSTRAINT
			     : "A" (control),
			       "b" (input_address_hi),
			       "D"(output_address_hi), "S"(output_address_lo),
			       THUNK_TARGET(hv_hypercall_pg)
			     : "cc", "memory");
	return hv_status;
#endif /* !x86_64 */
}

/* Fast hypercall with 8 bytes of input and no output */
static inline u64 _hv_do_fast_hypercall8(u64 control, u64 input1)
{
#ifdef CONFIG_X86_64
	return static_call_mod(hv_hypercall)(control, input1, 0);
#else
	u32 input1_hi = upper_32_bits(input1);
	u32 input1_lo = lower_32_bits(input1);
	u64 hv_status;

	__asm__ __volatile__ (CALL_NOSPEC
			      : "=A"(hv_status),
			      "+c"(input1_lo),
			      ASM_CALL_CONSTRAINT
			      :	"A" (control),
			      "b" (input1_hi),
			      THUNK_TARGET(hv_hypercall_pg)
			      : "cc", "edi", "esi");
	return hv_status;
#endif
}

static inline u64 hv_do_fast_hypercall8(u16 code, u64 input1)
{
	u64 control = (u64)code | HV_HYPERCALL_FAST_BIT;

	return _hv_do_fast_hypercall8(control, input1);
}

/* Fast hypercall with 16 bytes of input */
static inline u64 _hv_do_fast_hypercall16(u64 control, u64 input1, u64 input2)
{
#ifdef CONFIG_X86_64
	return static_call_mod(hv_hypercall)(control, input1, input2);
#else
	u32 input1_hi = upper_32_bits(input1);
	u32 input1_lo = lower_32_bits(input1);
	u32 input2_hi = upper_32_bits(input2);
	u32 input2_lo = lower_32_bits(input2);
	u64 hv_status;

	__asm__ __volatile__ (CALL_NOSPEC
			      : "=A"(hv_status),
			      "+c"(input1_lo), ASM_CALL_CONSTRAINT
			      :	"A" (control), "b" (input1_hi),
			      "D"(input2_hi), "S"(input2_lo),
			      THUNK_TARGET(hv_hypercall_pg)
			      : "cc");
	return hv_status;
#endif
}

static inline u64 hv_do_fast_hypercall16(u16 code, u64 input1, u64 input2)
{
	u64 control = (u64)code | HV_HYPERCALL_FAST_BIT;

	return _hv_do_fast_hypercall16(control, input1, input2);
}

extern struct hv_vp_assist_page **hv_vp_assist_page;

static inline struct hv_vp_assist_page *hv_get_vp_assist_page(unsigned int cpu)
{
	if (!hv_vp_assist_page)
		return NULL;

	return hv_vp_assist_page[cpu];
}

void __init hyperv_init(void);
void hyperv_setup_mmu_ops(void);
void set_hv_tscchange_cb(void (*cb)(void));
void clear_hv_tscchange_cb(void);
void hyperv_stop_tsc_emulation(void);
int hyperv_flush_guest_mapping(u64 as);
int hyperv_flush_guest_mapping_range(u64 as,
		hyperv_fill_flush_list_func fill_func, void *data);
int hyperv_fill_flush_guest_mapping_list(
		struct hv_guest_mapping_flush_list *flush,
		u64 start_gfn, u64 end_gfn);

#ifdef CONFIG_X86_64
void hv_apic_init(void);
void __init hv_init_spinlocks(void);
bool hv_vcpu_is_preempted(int vcpu);
#else
static inline void hv_apic_init(void) {}
#endif

struct irq_domain *hv_create_pci_msi_domain(void);

int hv_map_msi_interrupt(struct irq_data *data,
			 struct hv_interrupt_entry *out_entry);
int hv_map_ioapic_interrupt(int ioapic_id, bool level, int vcpu, int vector,
		struct hv_interrupt_entry *entry);
int hv_unmap_ioapic_interrupt(int ioapic_id, struct hv_interrupt_entry *entry);

#ifdef CONFIG_AMD_MEM_ENCRYPT
bool hv_ghcb_negotiate_protocol(void);
void __noreturn hv_ghcb_terminate(unsigned int set, unsigned int reason);
int hv_snp_boot_ap(u32 apic_id, unsigned long start_ip, unsigned int cpu);
#else
static inline bool hv_ghcb_negotiate_protocol(void) { return false; }
static inline void hv_ghcb_terminate(unsigned int set, unsigned int reason) {}
static inline int hv_snp_boot_ap(u32 apic_id, unsigned long start_ip,
		unsigned int cpu) { return 0; }
#endif

#if defined(CONFIG_AMD_MEM_ENCRYPT) || defined(CONFIG_INTEL_TDX_GUEST)
void hv_vtom_init(void);
void hv_ivm_msr_write(u64 msr, u64 value);
void hv_ivm_msr_read(u64 msr, u64 *value);
#else
static inline void hv_vtom_init(void) {}
static inline void hv_ivm_msr_write(u64 msr, u64 value) {}
static inline void hv_ivm_msr_read(u64 msr, u64 *value) {}
#endif

static inline bool hv_is_synic_msr(unsigned int reg)
{
	return (reg >= HV_X64_MSR_SCONTROL) &&
	       (reg <= HV_X64_MSR_SINT15);
}

static inline bool hv_is_sint_msr(unsigned int reg)
{
	return (reg >= HV_X64_MSR_SINT0) &&
	       (reg <= HV_X64_MSR_SINT15);
}

u64 hv_get_msr(unsigned int reg);
void hv_set_msr(unsigned int reg, u64 value);
u64 hv_get_non_nested_msr(unsigned int reg);
void hv_set_non_nested_msr(unsigned int reg, u64 value);

static __always_inline u64 hv_raw_get_msr(unsigned int reg)
{
	return native_rdmsrq(reg);
}
int hv_apicid_to_vp_index(u32 apic_id);

#if IS_ENABLED(CONFIG_MSHV_ROOT) && IS_ENABLED(CONFIG_CRASH_DUMP)
void hv_root_crash_init(void);
void hv_crash_asm32(void);
void hv_crash_asm64(void);
void hv_crash_asm_end(void);
#else   /* CONFIG_MSHV_ROOT && CONFIG_CRASH_DUMP */
static inline void hv_root_crash_init(void) {}
#endif  /* CONFIG_MSHV_ROOT && CONFIG_CRASH_DUMP */

#else /* CONFIG_HYPERV */
static inline void hyperv_init(void) {}
static inline void hyperv_setup_mmu_ops(void) {}
static inline void set_hv_tscchange_cb(void (*cb)(void)) {}
static inline void clear_hv_tscchange_cb(void) {}
static inline void hyperv_stop_tsc_emulation(void) {};
static inline struct hv_vp_assist_page *hv_get_vp_assist_page(unsigned int cpu)
{
	return NULL;
}
static inline int hyperv_flush_guest_mapping(u64 as) { return -1; }
static inline int hyperv_flush_guest_mapping_range(u64 as,
		hyperv_fill_flush_list_func fill_func, void *data)
{
	return -1;
}
static inline void hv_set_msr(unsigned int reg, u64 value) { }
static inline u64 hv_get_msr(unsigned int reg) { return 0; }
static inline void hv_set_non_nested_msr(unsigned int reg, u64 value) { }
static inline u64 hv_get_non_nested_msr(unsigned int reg) { return 0; }
static inline int hv_apicid_to_vp_index(u32 apic_id) { return -EINVAL; }
#endif /* CONFIG_HYPERV */


#ifdef CONFIG_HYPERV_VTL_MODE
void __init hv_vtl_init_platform(void);
int __init hv_vtl_early_init(void);
#else
static inline void __init hv_vtl_init_platform(void) {}
static inline int __init hv_vtl_early_init(void) { return 0; }
#endif

#include <asm-generic/mshyperv.h>

#endif
