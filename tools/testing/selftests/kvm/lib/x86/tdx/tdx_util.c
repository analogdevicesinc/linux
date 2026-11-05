// SPDX-License-Identifier: GPL-2.0-only

#include "processor.h"
#include "tdx/tdx_util.h"

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
