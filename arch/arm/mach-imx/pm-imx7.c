/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/busfreq-imx.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <asm/cacheflush.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/tlb.h>

#include "common.h"
#include "hardware.h"

#define M4_OCRAMS_RESERVED_SIZE	0xc

extern unsigned long iram_tlb_base_addr;
extern unsigned long iram_tlb_phys_addr;

static struct map_desc imx7_pm_io_desc[] __initdata = {
	imx_map_entry(MX7D, AIPS1, MT_DEVICE),
	imx_map_entry(MX7D, AIPS2, MT_DEVICE),
	imx_map_entry(MX7D, AIPS3, MT_DEVICE),
};

static const char * const low_power_ocram_match[] __initconst = {
	"fsl,lpm-sram",
	NULL
};

static struct map_desc iram_tlb_io_desc __initdata = {
	/* .virtual and .pfn are run-time assigned */
	.length     = SZ_1M,
	.type       = MT_MEMORY_RWX_NONCACHED,
};

static int __init imx7_dt_find_lpsram(unsigned long node, const char *uname,
				      int depth, void *data)
{
	unsigned long lpram_addr;
	const __be32 *prop = of_get_flat_dt_prop(node, "reg", NULL);

	if (of_flat_dt_match(node, low_power_ocram_match)) {
		if (!prop)
			return -EINVAL;

		lpram_addr = be32_to_cpup(prop);

		/* We need to create a 1M page table entry. */
		iram_tlb_io_desc.virtual = IMX_IO_P2V(lpram_addr & 0xFFF00000);
		iram_tlb_io_desc.pfn = __phys_to_pfn(lpram_addr & 0xFFF00000);
		iram_tlb_phys_addr = lpram_addr;
		iram_tlb_base_addr = IMX_IO_P2V(lpram_addr);
		iotable_init(&iram_tlb_io_desc, 1);
	}

	return 0;
}

void __init imx7_pm_map_io(void)
{
	unsigned long i, j;

	iotable_init(imx7_pm_io_desc, ARRAY_SIZE(imx7_pm_io_desc));
	/*
	 * Get the address of IRAM or OCRAM to be used by the low
	 * power code from the device tree.
	 */
	WARN_ON(of_scan_flat_dt(imx7_dt_find_lpsram, NULL));

	/* Return if no IRAM space is allocated for suspend/resume code. */
	if (!iram_tlb_base_addr) {
		pr_warn("No valid ocram available for suspend/resume!\n");
		return;
	}

	/* TODO: Handle M4 in TEE? */
	/* Set all entries to 0 except first 3 words reserved for M4. */
	memset((void *)(iram_tlb_base_addr + M4_OCRAMS_RESERVED_SIZE),
		0, MX7_IRAM_TLB_SIZE - M4_OCRAMS_RESERVED_SIZE);

	/*
	 * Make sure the IRAM virtual address has a mapping in the IRAM
	 * page table.
	 *
	 * Only use the top 12 bits [31-20] when storing the physical
	 * address in the page table as only these bits are required
	 * for 1M mapping.
	 */
	j = ((iram_tlb_base_addr >> 20) << 2) / 4;
	*((unsigned long *)iram_tlb_base_addr + j) =
		(iram_tlb_phys_addr & 0xFFF00000) | TT_ATTRIB_NON_CACHEABLE_1M;

	/*
	 * Make sure the AIPS1 virtual address has a mapping in the
	 * IRAM page table.
	 */
	for (i = 0; i < 4; i++) {
		j = ((IMX_IO_P2V(MX7D_AIPS1_BASE_ADDR + i * 0x100000) >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7D_AIPS1_BASE_ADDR + i * 0x100000) & 0xFFF00000) |
			TT_ATTRIB_NON_CACHEABLE_1M;
	}

	/*
	 * Make sure the AIPS2 virtual address has a mapping in the
	 * IRAM page table.
	 */
	for (i = 0; i < 4; i++) {
		j = ((IMX_IO_P2V(MX7D_AIPS2_BASE_ADDR + i * 0x100000) >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7D_AIPS2_BASE_ADDR + i * 0x100000) & 0xFFF00000) |
			TT_ATTRIB_NON_CACHEABLE_1M;
	}

	/*
	 * Make sure the AIPS3 virtual address has a mapping
	 * in the IRAM page table.
	 */
	for (i = 0; i < 4; i++) {
		j = ((IMX_IO_P2V(MX7D_AIPS3_BASE_ADDR + i * 0x100000) >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7D_AIPS3_BASE_ADDR + i * 0x100000) & 0xFFF00000) |
			TT_ATTRIB_NON_CACHEABLE_1M;
	}

	/*
	 * Make sure the GIC virtual address has a mapping in the
	 * IRAM page table.
	 */
	j = ((IMX_IO_P2V(MX7D_GIC_BASE_ADDR) >> 20) << 2) / 4;
	*((unsigned long *)iram_tlb_base_addr + j) =
		(MX7D_GIC_BASE_ADDR & 0xFFF00000) | TT_ATTRIB_NON_CACHEABLE_1M;
}
