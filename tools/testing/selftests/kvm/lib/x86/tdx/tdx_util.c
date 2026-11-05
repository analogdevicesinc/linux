// SPDX-License-Identifier: GPL-2.0-only

#include <linux/sizes.h>

#include "processor.h"
#include "tdx/td_boot.h"
#include "tdx/tdx_util.h"

/* Arbitrarily selected to avoid overlaps with anything else */
#define TD_BOOT_CODE_SLOT	20
#define TD_BOOT_PARAMETERS_SLOT	21

#define X86_RESET_VECTOR	0xfffffff0ul

void tdx_vm_setup_boot_code_region(struct kvm_vm *vm)
{
	const size_t boot_code_size = td_boot_reset_vector_trampoline - td_boot;
	const gpa_t boot_code_gpa = X86_RESET_VECTOR - boot_code_size;
	const size_t total_size = td_boot_code_end - td_boot;

	gpa_t alloc_gpa = round_down(boot_code_gpa, PAGE_SIZE);
	size_t nr_pages = (SZ_4G - alloc_gpa) / PAGE_SIZE;
	u64 gmem_flags = 0;
	gpa_t gpa;
	u8 *hva;

	if (kvm_has_gmem_attributes)
		gmem_flags |= GUEST_MEMFD_FLAG_INIT_SHARED | GUEST_MEMFD_FLAG_MMAP;
	vm_mem_add(vm, VM_MEM_SRC_SHMEM, alloc_gpa, TD_BOOT_CODE_SLOT,
		   nr_pages, KVM_MEM_GUEST_MEMFD, -1, 0, gmem_flags);

	gpa = ____vm_phy_pages_alloc(vm, nr_pages, alloc_gpa,
				     TD_BOOT_CODE_SLOT, true, false);
	TEST_ASSERT(gpa == alloc_gpa, "Failed vm_phy_pages_alloc");

	virt_map(vm, alloc_gpa, alloc_gpa, nr_pages);
	hva = addr_gpa2hva(vm, boot_code_gpa);
	memcpy(hva, td_boot, total_size);

	hva += boot_code_size;
	TEST_ASSERT(hva == addr_gpa2hva(vm, X86_RESET_VECTOR),
		    "Expected RESET vector at hva 0x%lx, got %lx",
		    (unsigned long)addr_gpa2hva(vm, X86_RESET_VECTOR), (unsigned long)hva);
}

void tdx_vm_setup_boot_parameters_region(struct kvm_vm *vm, u32 nr_runnable_vcpus)
{
	size_t boot_params_size = sizeof(struct td_boot_parameters) +
		nr_runnable_vcpus * sizeof(struct td_per_vcpu_parameters);
	int npages = DIV_ROUND_UP(boot_params_size, PAGE_SIZE);
	u64 gmem_flags = 0;
	gpa_t gpa;

	if (kvm_has_gmem_attributes)
		gmem_flags |= GUEST_MEMFD_FLAG_INIT_SHARED | GUEST_MEMFD_FLAG_MMAP;
	vm_mem_add(vm, VM_MEM_SRC_SHMEM, TD_BOOT_PARAMETERS_GPA,
		   TD_BOOT_PARAMETERS_SLOT, npages,
		   KVM_MEM_GUEST_MEMFD, -1, 0, gmem_flags);
	gpa = ____vm_phy_pages_alloc(vm, npages, TD_BOOT_PARAMETERS_GPA,
				     TD_BOOT_PARAMETERS_SLOT, true, false);
	TEST_ASSERT(gpa == TD_BOOT_PARAMETERS_GPA, "Failed vm_phy_pages_alloc");

	virt_map(vm, TD_BOOT_PARAMETERS_GPA, TD_BOOT_PARAMETERS_GPA, npages);
}

void tdx_vm_load_common_boot_parameters(struct kvm_vm *vm)
{
	struct td_boot_parameters *params = addr_gpa2hva(vm, TD_BOOT_PARAMETERS_GPA);
	u32 cr4;

	cr4 = kvm_get_default_cr4(vm->mmu.pgtable_levels);

	/* Per the TDX spec, CR4 bit MCE is fixed to 1 */
	cr4 |= X86_CR4_MCE;

	/* Per the TDX spec, CR4 bit VMXE and SMXE are fixed to 0 */
	cr4 &= ~(X86_CR4_VMXE | X86_CR4_SMXE);

	/* Set parameters! */
	params->cr0 = kvm_get_default_cr0();
	TEST_ASSERT(vm->mmu.pgd < (1ULL << 32),
		    "PGD must be within 32-bit address space for 32-bit boot code");
	params->cr3 = vm->mmu.pgd;
	params->cr4 = cr4;
	TEST_ASSERT(vm->arch.idt < (1ULL << 32),
		    "idt must be within 32-bit address space for 32-bit boot code");
	params->idtr.base = vm->arch.idt;
	params->idtr.limit = kvm_get_default_idt_limit();
	TEST_ASSERT(vm->arch.gdt < (1ULL << 32),
		    "gdt must be within 32-bit address space for 32-bit boot code");
	params->gdtr.base = vm->arch.gdt;
	params->gdtr.limit = kvm_get_default_gdt_limit();
}

static const struct kvm_tdx_capabilities *tdx_read_capabilities(struct kvm_vm *vm)
{
	static struct kvm_tdx_capabilities *tdx_cap;
	int nr_cpuid_configs = 4;
	int rc = -1;
	int i;

	if (tdx_cap)
		return tdx_cap;

	do {
		nr_cpuid_configs *= 2;

		tdx_cap = realloc(tdx_cap, sizeof(*tdx_cap) +
					   (sizeof(struct kvm_cpuid_entry2) * nr_cpuid_configs));
		TEST_ASSERT(tdx_cap,
			    "Could not allocate memory for tdx capability nr_cpuid_configs %d",
			    nr_cpuid_configs);

		tdx_cap->cpuid.nent = nr_cpuid_configs;
		rc = __tdx_vm_ioctl(vm, KVM_TDX_CAPABILITIES, 0, tdx_cap, NULL);
	} while (rc < 0 && errno == E2BIG);

	TEST_ASSERT(rc == 0, "KVM_TDX_CAPABILITIES failed: %d %d",
		    rc, errno);

	pr_debug("tdx_cap: supported_attrs: 0x%016llx\n"
		 "tdx_cap: supported_xfam 0x%016llx\n",
		 tdx_cap->supported_attrs, tdx_cap->supported_xfam);

	for (i = 0; i < tdx_cap->cpuid.nent; i++) {
		const struct kvm_cpuid_entry2 *config = &tdx_cap->cpuid.entries[i];

		pr_debug("cpuid config[%d]: leaf 0x%x sub_leaf 0x%x eax 0x%08x ebx 0x%08x ecx 0x%08x edx 0x%08x\n",
			 i, config->function, config->index,
			 config->eax, config->ebx, config->ecx, config->edx);
	}

	return tdx_cap;
}

/*
 * For each CPUID leaf in @cpuid_data, filter out unsupported bits based on the
 * capabilities reported by KVM.
 */
static void tdx_filter_cpuid(struct kvm_vm *vm,
			     struct kvm_cpuid2 *cpuid_data)
{
	const struct kvm_tdx_capabilities *tdx_cap;
	const struct kvm_cpuid_entry2 *config;
	struct kvm_cpuid_entry2 *e;
	int i;

	tdx_cap = tdx_read_capabilities(vm);

	i = 0;
	while (i < cpuid_data->nent) {
		e = cpuid_data->entries + i;
		config = __get_cpuid_entry(&tdx_cap->cpuid, e->function, e->index);

		if (!config) {
			int left = cpuid_data->nent - i - 1;

			if (left > 0)
				memmove(cpuid_data->entries + i,
					cpuid_data->entries + i + 1,
					sizeof(*cpuid_data->entries) * left);
			cpuid_data->nent--;
			continue;
		}

		e->eax &= config->eax;
		e->ebx &= config->ebx;
		e->ecx &= config->ecx;
		e->edx &= config->edx;

		i++;
	}
}

void tdx_init_vm(struct kvm_vm *vm)
{
	const struct kvm_cpuid2 *tmp = kvm_get_supported_cpuid();
	struct kvm_tdx_init_vm *init_vm;

	init_vm = calloc(1, struct_size(init_vm, cpuid.entries, tmp->nent));
	TEST_ASSERT(init_vm, "init_vm allocation failed");

	memcpy(&init_vm->cpuid, tmp, kvm_cpuid2_size(tmp->nent));
	init_vm->cpuid.padding = 0;
	tdx_filter_cpuid(vm, &init_vm->cpuid);

	/*
	 * Set attributes and xfam to 0 to use KVM's baseline config, as KVM
	 * automatically ORs in the mandatory fixed-1 bits during
	 * KVM_TDX_INIT_VM.  Note, TDX_TD_ATTR_SEPT_VE_DISABLE is not required
	 * because all initial private memory is pre-mapped before running the
	 * TD.
	 */
	init_vm->attributes = 0;
	init_vm->xfam = 0;

	tdx_vm_ioctl(vm, KVM_TDX_INIT_VM, 0, init_vm);

	free(init_vm);
}
