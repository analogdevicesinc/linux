// SPDX-License-Identifier: GPL-2.0-only
/*
 *  RISC-V Specific Low-Level ACPI Boot Support
 *
 *  Copyright (C) 2013-2014, Linaro Ltd.
 *	Author: Al Stone <al.stone@linaro.org>
 *	Author: Graeme Gregory <graeme.gregory@linaro.org>
 *	Author: Hanjun Guo <hanjun.guo@linaro.org>
 *	Author: Tomasz Nowicki <tomasz.nowicki@linaro.org>
 *	Author: Naresh Bhat <naresh.bhat@linaro.org>
 *
 *  Copyright (C) 2021-2023, Ventana Micro Systems Inc.
 *	Author: Sunil V L <sunilvl@ventanamicro.com>
 */

#include <linux/acpi.h>
#include <linux/efi.h>
#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>
#include <linux/pci.h>
#include <linux/serial_core.h>

int acpi_noirq = 1;		/* skip ACPI IRQ initialization */
int acpi_disabled = 1;
EXPORT_SYMBOL(acpi_disabled);

int acpi_pci_disabled = 1;	/* skip ACPI PCI scan and IRQ initialization */
EXPORT_SYMBOL(acpi_pci_disabled);

static bool param_acpi_off __initdata;
static bool param_acpi_on __initdata;
static bool param_acpi_force __initdata;

static struct acpi_madt_rintc cpu_madt_rintc[NR_CPUS];

static int __init parse_acpi(char *arg)
{
	if (!arg)
		return -EINVAL;

	/* "acpi=off" disables both ACPI table parsing and interpreter */
	if (strcmp(arg, "off") == 0)
		param_acpi_off = true;
	else if (strcmp(arg, "on") == 0) /* prefer ACPI over DT */
		param_acpi_on = true;
	else if (strcmp(arg, "force") == 0) /* force ACPI to be enabled */
		param_acpi_force = true;
	else
		return -EINVAL;	/* Core will print when we return error */

	return 0;
}
early_param("acpi", parse_acpi);

/*
 * acpi_fadt_sanity_check() - Check FADT presence and carry out sanity
 *			      checks on it
 *
 * Return 0 on success,  <0 on failure
 */
static int __init acpi_fadt_sanity_check(void)
{
	struct acpi_table_header *table;
	struct acpi_table_fadt *fadt;
	acpi_status status;
	int ret = 0;

	/*
	 * FADT is required on riscv; retrieve it to check its presence
	 * and carry out revision and ACPI HW reduced compliancy tests
	 */
	status = acpi_get_table(ACPI_SIG_FADT, 0, &table);
	if (ACPI_FAILURE(status)) {
		const char *msg = acpi_format_exception(status);

		pr_err("Failed to get FADT table, %s\n", msg);
		return -ENODEV;
	}

	fadt = (struct acpi_table_fadt *)table;

	/*
	 * The revision in the table header is the FADT's Major revision. The
	 * FADT also has a minor revision, which is stored in the FADT itself.
	 *
	 * TODO: Currently, we check for 6.5 as the minimum version to check
	 * for HW_REDUCED flag. However, once RISC-V updates are released in
	 * the ACPI spec, we need to update this check for exact minor revision
	 */
	if (table->revision < 6 || (table->revision == 6 && fadt->minor_revision < 5))
		pr_err(FW_BUG "Unsupported FADT revision %d.%d, should be 6.5+\n",
		       table->revision, fadt->minor_revision);

	if (!(fadt->flags & ACPI_FADT_HW_REDUCED)) {
		pr_err("FADT not ACPI hardware reduced compliant\n");
		ret = -EINVAL;
	}

	/*
	 * acpi_get_table() creates FADT table mapping that
	 * should be released after parsing and before resuming boot
	 */
	acpi_put_table(table);
	return ret;
}

/*
 * acpi_boot_table_init() called from setup_arch(), always.
 *	1. find RSDP and get its address, and then find XSDT
 *	2. extract all tables and checksums them all
 *	3. check ACPI FADT HW reduced flag
 *
 * We can parse ACPI boot-time tables such as MADT after
 * this function is called.
 *
 * On return ACPI is enabled if either:
 *
 * - ACPI tables are initialized and sanity checks passed
 * - acpi=force was passed in the command line and ACPI was not disabled
 *   explicitly through acpi=off command line parameter
 *
 * ACPI is disabled on function return otherwise
 */
void __init acpi_boot_table_init(void)
{
	/*
	 * Enable ACPI instead of device tree unless
	 * - ACPI has been disabled explicitly (acpi=off), or
	 * - firmware has not populated ACPI ptr in EFI system table
	 *   and ACPI has not been [force] enabled (acpi=on|force)
	 */
	if (param_acpi_off ||
	    (!param_acpi_on && !param_acpi_force &&
	     efi.acpi20 == EFI_INVALID_TABLE_ADDR))
		goto done;

	/*
	 * ACPI is disabled at this point. Enable it in order to parse
	 * the ACPI tables and carry out sanity checks
	 */
	enable_acpi();

	/*
	 * If ACPI tables are initialized and FADT sanity checks passed,
	 * leave ACPI enabled and carry on booting; otherwise disable ACPI
	 * on initialization error.
	 * If acpi=force was passed on the command line it forces ACPI
	 * to be enabled even if its initialization failed.
	 */
	if (acpi_table_init() || acpi_fadt_sanity_check()) {
		pr_err("Failed to init ACPI tables\n");
		if (!param_acpi_force)
			disable_acpi();
	}

done:
	if (acpi_disabled) {
		if (earlycon_acpi_spcr_enable)
			early_init_dt_scan_chosen_stdout();
	} else {
		acpi_parse_spcr(earlycon_acpi_spcr_enable, true);
	}
}

static int acpi_parse_madt_rintc(union acpi_subtable_headers *header, const unsigned long end)
{
	struct acpi_madt_rintc *rintc = (struct acpi_madt_rintc *)header;
	int cpuid;

	if (!(rintc->flags & ACPI_MADT_ENABLED))
		return 0;

	cpuid = riscv_hartid_to_cpuid(rintc->hart_id);
	/*
	 * When CONFIG_SMP is disabled, mapping won't be created for
	 * all cpus.
	 * CPUs more than num_possible_cpus, will be ignored.
	 */
	if (cpuid >= 0 && cpuid < num_possible_cpus())
		cpu_madt_rintc[cpuid] = *rintc;

	return 0;
}

/*
 * Instead of parsing (and freeing) the ACPI table, cache
 * the RINTC structures since they are frequently used
 * like in  cpuinfo.
 */
void __init acpi_init_rintc_map(void)
{
	if (acpi_table_parse_madt(ACPI_MADT_TYPE_RINTC, acpi_parse_madt_rintc, 0) <= 0) {
		pr_err("No valid RINTC entries exist\n");
		BUG();
	}
}

struct acpi_madt_rintc *acpi_cpu_get_madt_rintc(int cpu)
{
	return &cpu_madt_rintc[cpu];
}

/*
 * __acpi_map_table() will be called before paging_init(), so early_ioremap()
 * or early_memremap() should be called here to for ACPI table mapping.
 */
void __init __iomem *__acpi_map_table(unsigned long phys, unsigned long size)
{
	if (!size)
		return NULL;

	return early_memremap(phys, size);
}

void __init __acpi_unmap_table(void __iomem *map, unsigned long size)
{
	if (!map || !size)
		return;

	early_memunmap(map, size);
}

void __iomem *acpi_os_ioremap(acpi_physical_address phys, acpi_size size)
{
	efi_memory_desc_t *md, *region = NULL;
	pgprot_t prot;

	if (WARN_ON_ONCE(!efi_enabled(EFI_MEMMAP)))
		return NULL;

	for_each_efi_memory_desc(md) {
		u64 end = md->phys_addr + (md->num_pages << EFI_PAGE_SHIFT);

		if (phys < md->phys_addr || phys >= end)
			continue;

		if (phys + size > end) {
			pr_warn(FW_BUG "requested region covers multiple EFI memory regions\n");
			return NULL;
		}
		region = md;
		break;
	}

	/*
	 * It is fine for AML to remap regions that are not represented in the
	 * EFI memory map at all, as it only describes normal memory, and MMIO
	 * regions that require a virtual mapping to make them accessible to
	 * the EFI runtime services.
	 */
	prot = PAGE_KERNEL_IO;
	if (region) {
		switch (region->type) {
		case EFI_LOADER_CODE:
		case EFI_LOADER_DATA:
		case EFI_BOOT_SERVICES_CODE:
		case EFI_BOOT_SERVICES_DATA:
		case EFI_CONVENTIONAL_MEMORY:
		case EFI_PERSISTENT_MEMORY:
			if (memblock_is_map_memory(phys) ||
			    !memblock_is_region_memory(phys, size)) {
				pr_warn(FW_BUG "requested region covers kernel memory\n");
				return NULL;
			}

			/*
			 * Mapping kernel memory is permitted if the region in
			 * question is covered by a single memblock with the
			 * NOMAP attribute set: this enables the use of ACPI
			 * table overrides passed via initramfs.
			 * This particular use case only requires read access.
			 */
			fallthrough;

		case EFI_RUNTIME_SERVICES_CODE:
			/*
			 * This would be unusual, but not problematic per se,
			 * as long as we take care not to create a writable
			 * mapping for executable code.
			 */
			prot = PAGE_KERNEL_RO;
			break;

		case EFI_ACPI_RECLAIM_MEMORY:
			/*
			 * ACPI reclaim memory is used to pass firmware tables
			 * and other data that is intended for consumption by
			 * the OS only, which may decide it wants to reclaim
			 * that memory and use it for something else. We never
			 * do that, but we usually add it to the linear map
			 * anyway, in which case we should use the existing
			 * mapping.
			 */
			if (memblock_is_map_memory(phys))
				return (void __iomem *)__va(phys);
			fallthrough;

		default:
			if (region->attribute & EFI_MEMORY_WB)
				prot = PAGE_KERNEL;
			else if ((region->attribute & EFI_MEMORY_WC) ||
				 (region->attribute & EFI_MEMORY_WT))
				prot = pgprot_writecombine(PAGE_KERNEL);
		}
	}

	return ioremap_prot(phys, size, pgprot_val(prot));
}

#ifdef CONFIG_PCI

/*
 * raw_pci_read/write - Platform-specific PCI config space access.
 */
int raw_pci_read(unsigned int domain, unsigned int bus,
		 unsigned int devfn, int reg, int len, u32 *val)
{
	struct pci_bus *b = pci_find_bus(domain, bus);

	if (!b)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return b->ops->read(b, devfn, reg, len, val);
}

int raw_pci_write(unsigned int domain, unsigned int bus,
		  unsigned int devfn, int reg, int len, u32 val)
{
	struct pci_bus *b = pci_find_bus(domain, bus);

	if (!b)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return b->ops->write(b, devfn, reg, len, val);
}

#endif	/* CONFIG_PCI */
