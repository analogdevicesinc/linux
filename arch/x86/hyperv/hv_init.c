// SPDX-License-Identifier: GPL-2.0-only
/*
 * X86 specific Hyper-V initialization code.
 *
 * Copyright (C) 2016, Microsoft, Inc.
 *
 * Author : K. Y. Srinivasan <kys@microsoft.com>
 */

#define pr_fmt(fmt)  "Hyper-V: " fmt

#include <linux/efi.h>
#include <linux/types.h>
#include <linux/bitfield.h>
#include <linux/io.h>
#include <asm/apic.h>
#include <asm/desc.h>
#include <asm/e820/api.h>
#include <asm/sev.h>
#include <asm/hypervisor.h>
#include <hyperv/hvhdk.h>
#include <asm/mshyperv.h>
#include <asm/msr.h>
#include <asm/idtentry.h>
#include <asm/set_memory.h>
#include <linux/kexec.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/cpuhotplug.h>
#include <linux/syscore_ops.h>
#include <clocksource/hyperv_timer.h>
#include <linux/highmem.h>
#include <linux/export.h>

void *hv_hypercall_pg;

#ifdef CONFIG_X86_64
static u64 __hv_hyperfail(u64 control, u64 param1, u64 param2)
{
	return U64_MAX;
}

DEFINE_STATIC_CALL(__hv_hypercall, __hv_hyperfail);

u64 hv_std_hypercall(u64 control, u64 param1, u64 param2)
{
	u64 hv_status;

	register u64 __r8 asm("r8") = param2;
	asm volatile ("call " STATIC_CALL_TRAMP_STR(__hv_hypercall)
		      : "=a" (hv_status), ASM_CALL_CONSTRAINT,
		        "+c" (control), "+d" (param1), "+r" (__r8)
		      : : "cc", "memory", "r9", "r10", "r11");

	return hv_status;
}

typedef u64 (*hv_hypercall_f)(u64 control, u64 param1, u64 param2);

static inline void hv_set_hypercall_pg(void *ptr)
{
	hv_hypercall_pg = ptr;

	if (!ptr)
		ptr = &__hv_hyperfail;
	static_call_update(__hv_hypercall, (hv_hypercall_f)ptr);
}
#else
static inline void hv_set_hypercall_pg(void *ptr)
{
	hv_hypercall_pg = ptr;
}
EXPORT_SYMBOL_GPL(hv_hypercall_pg);
#endif

union hv_ghcb * __percpu *hv_ghcb_pg;

/* Storage to save the hypercall page temporarily for hibernation */
static void *hv_hypercall_pg_saved;

struct hv_vp_assist_page **hv_vp_assist_page;
EXPORT_SYMBOL_GPL(hv_vp_assist_page);

static int hyperv_init_ghcb(void)
{
	u64 ghcb_gpa;
	void *ghcb_va;
	void **ghcb_base;

	if (!ms_hyperv.paravisor_present || !hv_isolation_type_snp())
		return 0;

	if (!hv_ghcb_pg)
		return -EINVAL;

	/*
	 * GHCB page is allocated by paravisor. The address
	 * returned by MSR_AMD64_SEV_ES_GHCB is above shared
	 * memory boundary and map it here.
	 */
	rdmsrq(MSR_AMD64_SEV_ES_GHCB, ghcb_gpa);

	/* Mask out vTOM bit. ioremap_cache() maps decrypted */
	ghcb_gpa &= ~ms_hyperv.shared_gpa_boundary;
	ghcb_va = (void *)ioremap_cache(ghcb_gpa, HV_HYP_PAGE_SIZE);
	if (!ghcb_va)
		return -ENOMEM;

	ghcb_base = (void **)this_cpu_ptr(hv_ghcb_pg);
	*ghcb_base = ghcb_va;

	return 0;
}

static int hv_cpu_init(unsigned int cpu)
{
	union hv_vp_assist_msr_contents msr = { 0 };
	struct hv_vp_assist_page **hvp;
	int ret;

	ret = hv_common_cpu_init(cpu);
	if (ret)
		return ret;

	if (!hv_vp_assist_page)
		return 0;

	hvp = &hv_vp_assist_page[cpu];
	if (hv_root_partition()) {
		/*
		 * For root partition we get the hypervisor provided VP assist
		 * page, instead of allocating a new page.
		 */
		rdmsrq(HV_X64_MSR_VP_ASSIST_PAGE, msr.as_uint64);
		*hvp = memremap(msr.pfn << HV_X64_MSR_VP_ASSIST_PAGE_ADDRESS_SHIFT,
				PAGE_SIZE, MEMREMAP_WB);
	} else {
		/*
		 * The VP assist page is an "overlay" page (see Hyper-V TLFS's
		 * Section 5.2.1 "GPA Overlay Pages"). Here it must be zeroed
		 * out to make sure we always write the EOI MSR in
		 * hv_apic_eoi_write() *after* the EOI optimization is disabled
		 * in hv_cpu_die(), otherwise a CPU may not be stopped in the
		 * case of CPU offlining and the VM will hang.
		 */
		if (!*hvp) {
			*hvp = __vmalloc(PAGE_SIZE, GFP_KERNEL | __GFP_ZERO);

			/*
			 * Hyper-V should never specify a VM that is a Confidential
			 * VM and also running in the root partition. Root partition
			 * is blocked to run in Confidential VM. So only decrypt assist
			 * page in non-root partition here.
			 */
			if (*hvp && !ms_hyperv.paravisor_present && hv_isolation_type_snp()) {
				WARN_ON_ONCE(set_memory_decrypted((unsigned long)(*hvp), 1));
				memset(*hvp, 0, PAGE_SIZE);
			}
		}

		if (*hvp)
			msr.pfn = vmalloc_to_pfn(*hvp);

	}
	if (!WARN_ON(!(*hvp))) {
		msr.enable = 1;
		wrmsrq(HV_X64_MSR_VP_ASSIST_PAGE, msr.as_uint64);
	}

	/* Allow Hyper-V stimer vector to be injected from Hypervisor. */
	if (ms_hyperv.misc_features & HV_STIMER_DIRECT_MODE_AVAILABLE)
		apic_update_vector(cpu, HYPERV_STIMER0_VECTOR, true);

	return hyperv_init_ghcb();
}

static void (*hv_reenlightenment_cb)(void);

static void hv_reenlightenment_notify(struct work_struct *dummy)
{
	struct hv_tsc_emulation_status emu_status;

	rdmsrq(HV_X64_MSR_TSC_EMULATION_STATUS, *(u64 *)&emu_status);

	/* Don't issue the callback if TSC accesses are not emulated */
	if (hv_reenlightenment_cb && emu_status.inprogress)
		hv_reenlightenment_cb();
}
static DECLARE_DELAYED_WORK(hv_reenlightenment_work, hv_reenlightenment_notify);

void hyperv_stop_tsc_emulation(void)
{
	u64 freq;
	struct hv_tsc_emulation_status emu_status;

	rdmsrq(HV_X64_MSR_TSC_EMULATION_STATUS, *(u64 *)&emu_status);
	emu_status.inprogress = 0;
	wrmsrq(HV_X64_MSR_TSC_EMULATION_STATUS, *(u64 *)&emu_status);

	rdmsrq(HV_X64_MSR_TSC_FREQUENCY, freq);
	tsc_khz = div64_u64(freq, 1000);
}
EXPORT_SYMBOL_GPL(hyperv_stop_tsc_emulation);

static inline bool hv_reenlightenment_available(void)
{
	/*
	 * Check for required features and privileges to make TSC frequency
	 * change notifications work.
	 */
	return ms_hyperv.features & HV_ACCESS_FREQUENCY_MSRS &&
		ms_hyperv.misc_features & HV_FEATURE_FREQUENCY_MSRS_AVAILABLE &&
		ms_hyperv.features & HV_ACCESS_REENLIGHTENMENT;
}

DEFINE_IDTENTRY_SYSVEC(sysvec_hyperv_reenlightenment)
{
	apic_eoi();
	inc_irq_stat(irq_hv_reenlightenment_count);
	schedule_delayed_work(&hv_reenlightenment_work, HZ/10);
}

void set_hv_tscchange_cb(void (*cb)(void))
{
	struct hv_reenlightenment_control re_ctrl = {
		.vector = HYPERV_REENLIGHTENMENT_VECTOR,
		.enabled = 1,
	};
	struct hv_tsc_emulation_control emu_ctrl = {.enabled = 1};

	if (!hv_reenlightenment_available()) {
		pr_warn("reenlightenment support is unavailable\n");
		return;
	}

	if (!hv_vp_index)
		return;

	hv_reenlightenment_cb = cb;

	/* Make sure callback is registered before we write to MSRs */
	wmb();

	re_ctrl.target_vp = hv_vp_index[get_cpu()];

	wrmsrq(HV_X64_MSR_REENLIGHTENMENT_CONTROL, *((u64 *)&re_ctrl));
	wrmsrq(HV_X64_MSR_TSC_EMULATION_CONTROL, *((u64 *)&emu_ctrl));

	put_cpu();
}
EXPORT_SYMBOL_GPL(set_hv_tscchange_cb);

void clear_hv_tscchange_cb(void)
{
	struct hv_reenlightenment_control re_ctrl;

	if (!hv_reenlightenment_available())
		return;

	rdmsrq(HV_X64_MSR_REENLIGHTENMENT_CONTROL, *(u64 *)&re_ctrl);
	re_ctrl.enabled = 0;
	wrmsrq(HV_X64_MSR_REENLIGHTENMENT_CONTROL, *(u64 *)&re_ctrl);

	hv_reenlightenment_cb = NULL;
}
EXPORT_SYMBOL_GPL(clear_hv_tscchange_cb);

static int hv_cpu_die(unsigned int cpu)
{
	struct hv_reenlightenment_control re_ctrl;
	unsigned int new_cpu;
	void **ghcb_va;

	if (hv_ghcb_pg) {
		ghcb_va = (void **)this_cpu_ptr(hv_ghcb_pg);
		if (*ghcb_va)
			iounmap(*ghcb_va);
		*ghcb_va = NULL;
	}

	if (ms_hyperv.misc_features & HV_STIMER_DIRECT_MODE_AVAILABLE)
		apic_update_vector(cpu, HYPERV_STIMER0_VECTOR, false);

	hv_common_cpu_die(cpu);

	if (hv_vp_assist_page && hv_vp_assist_page[cpu]) {
		union hv_vp_assist_msr_contents msr = { 0 };
		if (hv_root_partition()) {
			/*
			 * For root partition the VP assist page is mapped to
			 * hypervisor provided page, and thus we unmap the
			 * page here and nullify it, so that in future we have
			 * correct page address mapped in hv_cpu_init.
			 */
			memunmap(hv_vp_assist_page[cpu]);
			hv_vp_assist_page[cpu] = NULL;
			rdmsrq(HV_X64_MSR_VP_ASSIST_PAGE, msr.as_uint64);
			msr.enable = 0;
		}
		wrmsrq(HV_X64_MSR_VP_ASSIST_PAGE, msr.as_uint64);
	}

	if (hv_reenlightenment_cb == NULL)
		return 0;

	rdmsrq(HV_X64_MSR_REENLIGHTENMENT_CONTROL, *((u64 *)&re_ctrl));
	if (re_ctrl.target_vp == hv_vp_index[cpu]) {
		/*
		 * Reassign reenlightenment notifications to some other online
		 * CPU or just disable the feature if there are no online CPUs
		 * left (happens on hibernation).
		 */
		new_cpu = cpumask_any_but(cpu_online_mask, cpu);

		if (new_cpu < nr_cpu_ids)
			re_ctrl.target_vp = hv_vp_index[new_cpu];
		else
			re_ctrl.enabled = 0;

		wrmsrq(HV_X64_MSR_REENLIGHTENMENT_CONTROL, *((u64 *)&re_ctrl));
	}

	return 0;
}

static int __init hv_pci_init(void)
{
	bool gen2vm = efi_enabled(EFI_BOOT);

	/*
	 * A Generation-2 VM doesn't support legacy PCI/PCIe, so both
	 * raw_pci_ops and raw_pci_ext_ops are NULL, and pci_subsys_init() ->
	 * pcibios_init() doesn't call pcibios_resource_survey() ->
	 * e820__reserve_resources_late(); as a result, any emulated persistent
	 * memory of E820_TYPE_PRAM (12) via the kernel parameter
	 * memmap=nn[KMG]!ss is not added into iomem_resource and hence can't be
	 * detected by register_e820_pmem(). Fix this by directly calling
	 * e820__reserve_resources_late() here: e820__reserve_resources_late()
	 * depends on e820__reserve_resources(), which has been called earlier
	 * from setup_arch(). Note: e820__reserve_resources_late() also adds
	 * any memory of E820_TYPE_PMEM (7) into iomem_resource, and
	 * acpi_nfit_register_region() -> acpi_nfit_insert_resource() ->
	 * region_intersects() returns REGION_INTERSECTS, so the memory of
	 * E820_TYPE_PMEM won't get added twice.
	 *
	 * We return 0 here so that pci_arch_init() won't print the warning:
	 * "PCI: Fatal: No config space access function found"
	 */
	if (gen2vm) {
		e820__reserve_resources_late();
		return 0;
	}

	/* For Generation-1 VM, we'll proceed in pci_arch_init().  */
	return 1;
}

static int hv_suspend(void)
{
	union hv_x64_msr_hypercall_contents hypercall_msr;
	int ret;

	if (hv_root_partition())
		return -EPERM;

	/*
	 * Reset the hypercall page as it is going to be invalidated
	 * across hibernation. Setting hv_hypercall_pg to NULL ensures
	 * that any subsequent hypercall operation fails safely instead of
	 * crashing due to an access of an invalid page. The hypercall page
	 * pointer is restored on resume.
	 */
	hv_hypercall_pg_saved = hv_hypercall_pg;
	hv_set_hypercall_pg(NULL);

	/* Disable the hypercall page in the hypervisor */
	rdmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);
	hypercall_msr.enable = 0;
	wrmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);

	ret = hv_cpu_die(0);
	return ret;
}

static void hv_resume(void)
{
	union hv_x64_msr_hypercall_contents hypercall_msr;
	int ret;

	ret = hv_cpu_init(0);
	WARN_ON(ret);

	/* Re-enable the hypercall page */
	rdmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);
	hypercall_msr.enable = 1;
	hypercall_msr.guest_physical_address =
		vmalloc_to_pfn(hv_hypercall_pg_saved);
	wrmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);

	hv_set_hypercall_pg(hv_hypercall_pg_saved);
	hv_hypercall_pg_saved = NULL;

	/*
	 * Reenlightenment notifications are disabled by hv_cpu_die(0),
	 * reenable them here if hv_reenlightenment_cb was previously set.
	 */
	if (hv_reenlightenment_cb)
		set_hv_tscchange_cb(hv_reenlightenment_cb);
}

/* Note: when the ops are called, only CPU0 is online and IRQs are disabled. */
static struct syscore_ops hv_syscore_ops = {
	.suspend	= hv_suspend,
	.resume		= hv_resume,
};

static void (* __initdata old_setup_percpu_clockev)(void);

static void __init hv_stimer_setup_percpu_clockev(void)
{
	/*
	 * Ignore any errors in setting up stimer clockevents
	 * as we can run with the LAPIC timer as a fallback.
	 */
	(void)hv_stimer_alloc(false);

	/*
	 * Still register the LAPIC timer, because the direct-mode STIMER is
	 * not supported by old versions of Hyper-V. This also allows users
	 * to switch to LAPIC timer via /sys, if they want to.
	 */
	if (old_setup_percpu_clockev)
		old_setup_percpu_clockev();
}

/*
 * This function is to be invoked early in the boot sequence after the
 * hypervisor has been detected.
 *
 * 1. Setup the hypercall page.
 * 2. Register Hyper-V specific clocksource.
 * 3. Setup Hyper-V specific APIC entry points.
 */
void __init hyperv_init(void)
{
	u64 guest_id;
	union hv_x64_msr_hypercall_contents hypercall_msr;
	int cpuhp;

	if (x86_hyper_type != X86_HYPER_MS_HYPERV)
		return;

	if (hv_common_init())
		return;

	/*
	 * The VP assist page is useless to a TDX guest: the only use we
	 * would have for it is lazy EOI, which can not be used with TDX.
	 */
	if (hv_isolation_type_tdx())
		hv_vp_assist_page = NULL;
	else
		hv_vp_assist_page = kcalloc(nr_cpu_ids,
					    sizeof(*hv_vp_assist_page),
					    GFP_KERNEL);
	if (!hv_vp_assist_page) {
		ms_hyperv.hints &= ~HV_X64_ENLIGHTENED_VMCS_RECOMMENDED;

		if (!hv_isolation_type_tdx())
			goto common_free;
	}

	if (ms_hyperv.paravisor_present && hv_isolation_type_snp()) {
		/* Negotiate GHCB Version. */
		if (!hv_ghcb_negotiate_protocol())
			hv_ghcb_terminate(SEV_TERM_SET_GEN,
					  GHCB_SEV_ES_PROT_UNSUPPORTED);

		hv_ghcb_pg = alloc_percpu(union hv_ghcb *);
		if (!hv_ghcb_pg)
			goto free_vp_assist_page;
	}

	cpuhp = cpuhp_setup_state(CPUHP_AP_HYPERV_ONLINE, "x86/hyperv_init:online",
				  hv_cpu_init, hv_cpu_die);
	if (cpuhp < 0)
		goto free_ghcb_page;

	/*
	 * Setup the hypercall page and enable hypercalls.
	 * 1. Register the guest ID
	 * 2. Enable the hypercall and register the hypercall page
	 *
	 * A TDX VM with no paravisor only uses TDX GHCI rather than hv_hypercall_pg:
	 * when the hypercall input is a page, such a VM must pass a decrypted
	 * page to Hyper-V, e.g. hv_post_message() uses the per-CPU page
	 * hyperv_pcpu_input_arg, which is decrypted if no paravisor is present.
	 *
	 * A TDX VM with the paravisor uses hv_hypercall_pg for most hypercalls,
	 * which are handled by the paravisor and the VM must use an encrypted
	 * input page: in such a VM, the hyperv_pcpu_input_arg is encrypted and
	 * used in the hypercalls, e.g. see hv_mark_gpa_visibility() and
	 * hv_arch_irq_unmask(). Such a VM uses TDX GHCI for two hypercalls:
	 * 1. HVCALL_SIGNAL_EVENT: see vmbus_set_event() and _hv_do_fast_hypercall8().
	 * 2. HVCALL_POST_MESSAGE: the input page must be a decrypted page, i.e.
	 * hv_post_message() in such a VM can't use the encrypted hyperv_pcpu_input_arg;
	 * instead, hv_post_message() uses the post_msg_page, which is decrypted
	 * in such a VM and is only used in such a VM.
	 */
	guest_id = hv_generate_guest_id(LINUX_VERSION_CODE);
	wrmsrq(HV_X64_MSR_GUEST_OS_ID, guest_id);

	/* With the paravisor, the VM must also write the ID via GHCB/GHCI */
	hv_ivm_msr_write(HV_X64_MSR_GUEST_OS_ID, guest_id);

	/* A TDX VM with no paravisor only uses TDX GHCI rather than hv_hypercall_pg */
	if (hv_isolation_type_tdx() && !ms_hyperv.paravisor_present)
		goto skip_hypercall_pg_init;

	hv_hypercall_pg = __vmalloc_node_range(PAGE_SIZE, 1, MODULES_VADDR,
			MODULES_END, GFP_KERNEL, PAGE_KERNEL_ROX,
			VM_FLUSH_RESET_PERMS, NUMA_NO_NODE,
			__builtin_return_address(0));
	if (hv_hypercall_pg == NULL)
		goto clean_guest_os_id;

	rdmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);
	hypercall_msr.enable = 1;

	if (hv_root_partition()) {
		struct page *pg;
		void *src;

		/*
		 * For the root partition, the hypervisor will set up its
		 * hypercall page. The hypervisor guarantees it will not show
		 * up in the root's address space. The root can't change the
		 * location of the hypercall page.
		 *
		 * Order is important here. We must enable the hypercall page
		 * so it is populated with code, then copy the code to an
		 * executable page.
		 */
		wrmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);

		pg = vmalloc_to_page(hv_hypercall_pg);
		src = memremap(hypercall_msr.guest_physical_address << PAGE_SHIFT, PAGE_SIZE,
				MEMREMAP_WB);
		BUG_ON(!src);
		memcpy_to_page(pg, 0, src, HV_HYP_PAGE_SIZE);
		memunmap(src);

		hv_remap_tsc_clocksource();
		hv_root_crash_init();
	} else {
		hypercall_msr.guest_physical_address = vmalloc_to_pfn(hv_hypercall_pg);
		wrmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);
	}

	hv_set_hypercall_pg(hv_hypercall_pg);

skip_hypercall_pg_init:
	/*
	 * hyperv_init() is called before LAPIC is initialized: see
	 * apic_intr_mode_init() -> x86_platform.apic_post_init() and
	 * apic_bsp_setup() -> setup_local_APIC(). The direct-mode STIMER
	 * depends on LAPIC, so hv_stimer_alloc() should be called from
	 * x86_init.timers.setup_percpu_clockev.
	 */
	old_setup_percpu_clockev = x86_init.timers.setup_percpu_clockev;
	x86_init.timers.setup_percpu_clockev = hv_stimer_setup_percpu_clockev;

	hv_apic_init();

	x86_init.pci.arch_init = hv_pci_init;

	register_syscore_ops(&hv_syscore_ops);

	if (ms_hyperv.priv_high & HV_ACCESS_PARTITION_ID)
		hv_get_partition_id();

#ifdef CONFIG_PCI_MSI
	/*
	 * If we're running as root, we want to create our own PCI MSI domain.
	 * We can't set this in hv_pci_init because that would be too late.
	 */
	if (hv_root_partition())
		x86_init.irqs.create_pci_msi_domain = hv_create_pci_msi_domain;
#endif

	/* Query the VMs extended capability once, so that it can be cached. */
	hv_query_ext_cap(0);

	/* Find the VTL */
	ms_hyperv.vtl = get_vtl();

	if (ms_hyperv.vtl > 0) /* non default VTL */
		hv_vtl_early_init();

	return;

clean_guest_os_id:
	wrmsrq(HV_X64_MSR_GUEST_OS_ID, 0);
	hv_ivm_msr_write(HV_X64_MSR_GUEST_OS_ID, 0);
	cpuhp_remove_state(CPUHP_AP_HYPERV_ONLINE);
free_ghcb_page:
	free_percpu(hv_ghcb_pg);
free_vp_assist_page:
	kfree(hv_vp_assist_page);
	hv_vp_assist_page = NULL;
common_free:
	hv_common_free();
}

/*
 * This routine is called before kexec/kdump, it does the required cleanup.
 */
void hyperv_cleanup(void)
{
	union hv_x64_msr_hypercall_contents hypercall_msr;
	union hv_reference_tsc_msr tsc_msr;

	/* Reset our OS id */
	wrmsrq(HV_X64_MSR_GUEST_OS_ID, 0);
	hv_ivm_msr_write(HV_X64_MSR_GUEST_OS_ID, 0);

	/*
	 * Reset hypercall page reference before reset the page,
	 * let hypercall operations fail safely rather than
	 * panic the kernel for using invalid hypercall page
	 */
	hv_hypercall_pg = NULL;

	/* Reset the hypercall page */
	hypercall_msr.as_uint64 = hv_get_msr(HV_X64_MSR_HYPERCALL);
	hypercall_msr.enable = 0;
	hv_set_msr(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);

	/* Reset the TSC page */
	tsc_msr.as_uint64 = hv_get_msr(HV_X64_MSR_REFERENCE_TSC);
	tsc_msr.enable = 0;
	hv_set_msr(HV_X64_MSR_REFERENCE_TSC, tsc_msr.as_uint64);
}

void hyperv_report_panic(struct pt_regs *regs, long err, bool in_die)
{
	static bool panic_reported;
	u64 guest_id;

	if (in_die && !panic_on_oops)
		return;

	/*
	 * We prefer to report panic on 'die' chain as we have proper
	 * registers to report, but if we miss it (e.g. on BUG()) we need
	 * to report it on 'panic'.
	 */
	if (panic_reported)
		return;
	panic_reported = true;

	rdmsrq(HV_X64_MSR_GUEST_OS_ID, guest_id);

	wrmsrq(HV_X64_MSR_CRASH_P0, err);
	wrmsrq(HV_X64_MSR_CRASH_P1, guest_id);
	wrmsrq(HV_X64_MSR_CRASH_P2, regs->ip);
	wrmsrq(HV_X64_MSR_CRASH_P3, regs->ax);
	wrmsrq(HV_X64_MSR_CRASH_P4, regs->sp);

	/*
	 * Let Hyper-V know there is crash data available
	 */
	wrmsrq(HV_X64_MSR_CRASH_CTL, HV_CRASH_CTL_CRASH_NOTIFY);
}
EXPORT_SYMBOL_GPL(hyperv_report_panic);

bool hv_is_hyperv_initialized(void)
{
	union hv_x64_msr_hypercall_contents hypercall_msr;

	/*
	 * Ensure that we're really on Hyper-V, and not a KVM or Xen
	 * emulation of Hyper-V
	 */
	if (x86_hyper_type != X86_HYPER_MS_HYPERV)
		return false;

	/* A TDX VM with no paravisor uses TDX GHCI call rather than hv_hypercall_pg */
	if (hv_isolation_type_tdx() && !ms_hyperv.paravisor_present)
		return true;
	/*
	 * Verify that earlier initialization succeeded by checking
	 * that the hypercall page is setup
	 */
	hypercall_msr.as_uint64 = 0;
	rdmsrq(HV_X64_MSR_HYPERCALL, hypercall_msr.as_uint64);

	return hypercall_msr.enable;
}
EXPORT_SYMBOL_GPL(hv_is_hyperv_initialized);

int hv_apicid_to_vp_index(u32 apic_id)
{
	u64 control;
	u64 status;
	unsigned long irq_flags;
	struct hv_get_vp_from_apic_id_in *input;
	u32 *output, ret;

	local_irq_save(irq_flags);

	input = *this_cpu_ptr(hyperv_pcpu_input_arg);
	memset(input, 0, sizeof(*input));
	input->partition_id = HV_PARTITION_ID_SELF;
	input->apic_ids[0] = apic_id;

	output = *this_cpu_ptr(hyperv_pcpu_output_arg);

	control = HV_HYPERCALL_REP_COMP_1 | HVCALL_GET_VP_INDEX_FROM_APIC_ID;
	status = hv_do_hypercall(control, input, output);
	ret = output[0];

	local_irq_restore(irq_flags);

	if (!hv_result_success(status)) {
		pr_err("failed to get vp index from apic id %d, status %#llx\n",
		       apic_id, status);
		return -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(hv_apicid_to_vp_index);
