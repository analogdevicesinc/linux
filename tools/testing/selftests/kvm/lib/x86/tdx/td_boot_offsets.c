// SPDX-License-Identifier: GPL-2.0
#define COMPILE_OFFSETS

#include <linux/kbuild.h>

#include "tdx/td_boot.h"

static void __attribute__((used)) common(void)
{
	OFFSET(TD_BOOT_PARAMETERS_CR0, td_boot_parameters, cr0);
	OFFSET(TD_BOOT_PARAMETERS_CR3, td_boot_parameters, cr3);
	OFFSET(TD_BOOT_PARAMETERS_CR4, td_boot_parameters, cr4);
	OFFSET(TD_BOOT_PARAMETERS_GDT, td_boot_parameters, gdtr);
	OFFSET(TD_BOOT_PARAMETERS_IDT, td_boot_parameters, idtr);
	OFFSET(TD_BOOT_PARAMETERS_PER_VCPU, td_boot_parameters, per_vcpu);
	OFFSET(TD_PER_VCPU_PARAMETERS_ESP_GVA, td_per_vcpu_parameters, esp_gva);
	OFFSET(TD_PER_VCPU_PARAMETERS_GUEST_CODE, td_per_vcpu_parameters,
	       guest_code);
	DEFINE(SIZEOF_TD_PER_VCPU_PARAMETERS,
	       sizeof(struct td_per_vcpu_parameters));
}
