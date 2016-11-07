/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/genalloc.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/tlb.h>

#include "common.h"
#include "hardware.h"

#define PMPROT	0x8
#define PMCTRL	0x10
#define PMSTAT	0x18
#define SRS	0x20
#define RPC	0x24
#define SSRS	0x28
#define SRIE	0x2c
#define SRIF	0x30
#define CSRE	0x34
#define MR	0x40

#define PMC_HSRUN		0x4
#define PMC_RUN			0x8
#define PMC_VLPR		0xc
#define PMC_STOP		0x10
#define PMC_VLPS		0x14
#define PMC_LLS			0x18
#define PMC_VLLS		0x1c
#define PMC_STATUS		0x20
#define PMC_CTRL		0x24
#define PMC_SRAMCTRL_0		0x28
#define PMC_SRAMCTRL_1		0x2c
#define PMC_SRAMCTRL_2		0x30

#define BM_PMPROT_AHSRUN	(1 << 7)
#define BM_PMPROT_AVLP		(1 << 5)
#define BM_PMPROT_ALLS		(1 << 3)
#define BM_PMPROT_AVLLS		(1 << 1)

#define BM_PMCTRL_STOPA		(1 << 24)
#define BM_PMCTRL_PSTOPO	(3 << 16)
#define BM_PMCTRL_RUNM		(3 << 8)
#define BM_PMCTRL_STOPM		(7 << 0)

#define BM_CTRL_LDOEN		(1 << 31)
#define BM_CTRL_LDOOKDIS	(1 << 30)

#define BM_VLLS_MON1P2HVDHP	(1 << 5)
#define BM_VLLS_MON1P2LVDHP	(1 << 4)

#define BP_PMCTRL_STOPM		0
#define BP_PMCTRL_PSTOPO	16

static void __iomem *smc1_base;
static void __iomem *pmc0_base;

extern unsigned long iram_tlb_base_addr;
extern unsigned long iram_tlb_phys_addr;

int imx7ulp_set_lpm(enum imx7ulp_cpu_pwr_mode mode)
{
	u32 val1 = BM_PMPROT_AVLP | BM_PMPROT_AVLLS;
	u32 val2 = readl_relaxed(smc1_base + PMCTRL);
	u32 val3 = readl_relaxed(pmc0_base + PMC_CTRL);

	val2 &= ~(BM_PMCTRL_RUNM |
		BM_PMCTRL_STOPM | BM_PMCTRL_PSTOPO);
	val3 |= BM_CTRL_LDOOKDIS;

	switch (mode) {
	case RUN:
		/* system/bus clock enabled */
		val2 |= 0x3 << BP_PMCTRL_PSTOPO;
		break;
	case WAIT:
		/* system clock disabled, bus clock enabled */
		val2 |= 0x2 << BP_PMCTRL_PSTOPO;
		break;
	case STOP:
		/* system/bus clock disabled */
		val2 |= 0x1 << BP_PMCTRL_PSTOPO;
		break;
	case VLPS:
		val2 |= 0x2 << BP_PMCTRL_STOPM;
		break;
	case VLLS:
		val2 |= 0x4 << BP_PMCTRL_STOPM;
		break;
	default:
		return -EINVAL;
	}

	writel_relaxed(val1, smc1_base + PMPROT);
	writel_relaxed(val2, smc1_base + PMCTRL);
	writel_relaxed(val3, pmc0_base + PMC_CTRL);

	return 0;
}

static struct map_desc iram_tlb_io_desc __initdata = {
	/* .virtual and .pfn are run-time assigned */
	.length     = SZ_1M,
	.type       = MT_MEMORY_RWX_NONCACHED,
};

static const char * const low_power_ocram_match[] __initconst = {
	"fsl,lpm-sram",
	NULL
};

static struct map_desc imx7ulp_pm_io_desc[] __initdata = {
	imx_map_entry(MX7ULP, AIPS1, MT_DEVICE),
	imx_map_entry(MX7ULP, AIPS2, MT_DEVICE),
};

static int __init imx7ulp_dt_find_lpsram(unsigned long node, const char *uname,
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

void __init imx7ulp_pm_map_io(void)
{
	unsigned long i, j;

	iotable_init(imx7ulp_pm_io_desc, ARRAY_SIZE(imx7ulp_pm_io_desc));
	/*
	 * Get the address of IRAM or OCRAM to be used by the low
	 * power code from the device tree.
	 */
	WARN_ON(of_scan_flat_dt(imx7ulp_dt_find_lpsram, NULL));

	/* Return if no IRAM space is allocated for suspend/resume code. */
	if (!iram_tlb_base_addr) {
		pr_warn("No valid ocram available for suspend/resume!\n");
		return;
	}

	/* Set all entries to 0 except first 3 words reserved for M4. */
	memset((void *)iram_tlb_base_addr, 0, MX7ULP_IRAM_TLB_SIZE);

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
		j = ((IMX_IO_P2V(MX7ULP_AIPS1_BASE_ADDR + i * 0x100000) >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7ULP_AIPS1_BASE_ADDR + i * 0x100000) & 0xFFF00000) |
			TT_ATTRIB_NON_CACHEABLE_1M;
	}
	/*
	 * Make sure the AIPS2 virtual address has a mapping in the
	 * IRAM page table.
	 */
	for (i = 0; i < 4; i++) {
		j = ((IMX_IO_P2V(MX7ULP_AIPS2_BASE_ADDR + i * 0x100000) >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7ULP_AIPS2_BASE_ADDR + i * 0x100000) & 0xFFF00000) |
			TT_ATTRIB_NON_CACHEABLE_1M;
	}
}

void __init imx7ulp_pm_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-smc1");
	smc1_base = of_iomap(np, 0);
	WARN_ON(!smc1_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pmc0");
	pmc0_base = of_iomap(np, 0);
	WARN_ON(!pmc0_base);

	imx7ulp_set_lpm(RUN);
}
