/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright NXP 2017.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MX7ULP_IOMAP_H__
#define __ASM_ARCH_MX7ULP_IOMAP_H__

#define MX7ULP_IO_P2V(x)                  IMX_IO_P2V(x)
#define MX7ULP_IO_ADDRESS(x)              IOMEM(MX7ULP_IO_P2V(x))

#define MX7ULP_AIPS1_BASE_ADDR		0x40000000
#define MX7ULP_AIPS1_SIZE		0x100000
#define MX7ULP_AIPS2_BASE_ADDR		0x40300000
#define MX7ULP_AIPS2_SIZE		0x100000
#define MX7ULP_AIPS3_BASE_ADDR		0x40400000
#define MX7ULP_AIPS3_SIZE		0x100000
#define MX7ULP_AIPS4_BASE_ADDR		0x40a00000
#define MX7ULP_AIPS4_SIZE		0x100000
#define MX7ULP_AIPS5_BASE_ADDR		0x41000000
#define MX7ULP_AIPS5_SIZE		0x100000
#define MX7ULP_GPIOC_BASE_ADDR		0x400f0000
#define MX7ULP_GPIOC_SIZE		0x1000
#define MX7ULP_PCC3_BASE_ADDR		0x40b30000
#define MX7ULP_PCC3_SIZE		0x1000
#define MX7ULP_SCG1_BASE_ADDR		0x403e0000
#define MX7ULP_SCG1_SIZE		0x1000
#define MX7ULP_PCC2_BASE_ADDR		0x403f0000
#define MX7ULP_PCC2_SIZE		0x1000
#define MX7ULP_SIM_BASE_ADDR		0x410a3000
#define MX7ULP_SIM_SIZE			0x1000
#define MX7ULP_PMC1_BASE_ADDR		0x40400000
#define MX7ULP_PMC1_SIZE		0x1000
#define MX7ULP_SMC1_BASE_ADDR		0x40410000
#define MX7ULP_SMC1_SIZE		0x1000
#define MX7ULP_MMDC_BASE_ADDR		0x40ab0000
#define MX7ULP_MMDC_SIZE		0x1000
#define MX7ULP_IOMUXC1_BASE_ADDR	0x40ac0000
#define MX7ULP_IOMUXC1_BASE__SIZE	0x1000
#define MX7ULP_MMDC_IO_BASE_ADDR	0x40ad0000
#define MX7ULP_MMDC_IO_SIZE		0x1000

/* below is just used for static mapping of the AIPSx's memory region */
#define MX7ULP_AIPS_VIRT_BASE(x)	(0xf4000000 + ((x) * SZ_1M))

#define mx7ulp_aips_map_entry(index, _type) {				\
	.virtual = MX7ULP_AIPS_VIRT_BASE(index),			\
	.pfn = __phys_to_pfn(MX7ULP_AIPS ## index ## _BASE_ADDR),	\
	.length	= SZ_1M,						\
	.type = _type,							\
}

#define TT_ATTRIB_NON_CACHEABLE_1M	0x802
#define MX7ULP_IRAM_TLB_SIZE		0x4000
#define MX7ULP_SUSPEND_OCRAM_SIZE	0x1000

#endif
