/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 *  * This program is free software; you can redistribute it and/or modify
 *   * it under the terms of the GNU General Public License version 2 as
 *    * published by the Free Software Foundation.
 *     */

#ifndef __ASM_ARCH_MX7ULP_IOMAP_H__
#define __ASM_ARCH_MX7ULP_IOMAP_H__

#define MX7ULP_IO_P2V(x)                  IMX_IO_P2V(x)
#define MX7ULP_IO_ADDRESS(x)              IOMEM(MX7ULP_IO_P2V(x))

#define MX7ULP_AIPS1_BASE_ADDR		0x40a00000
#define MX7ULP_AIPS1_SIZE		0x400000
#define MX7ULP_AIPS2_BASE_ADDR		0x41000000
#define MX7ULP_AIPS2_SIZE		0x400000
#define MX7ULP_PCC3_BASE_ADDR		0x40b30000
#define MX7ULP_PCC3_SIZE		0x1000
#define MX7ULP_SCG1_BASE_ADDR		0x403e0000
#define MX7ULP_SCG1_SIZE		0x1000
#define MX7ULP_SIM_BASE_ADDR		0x410a3000
#define MX7ULP_SIM_SIZE			0x1000
#define MX7ULP_MMDC_BASE_ADDR		0x40ab0000
#define MX7ULP_MMDC_SIZE		0x1000
#define MX7ULP_MMDC_IO_BASE_ADDR	0x40ad0000
#define MX7ULP_MMDC_IO_SIZE		0x1000

#define TT_ATTRIB_NON_CACHEABLE_1M	0x802
#define MX7ULP_IRAM_TLB_SIZE		0x4000
#define MX7ULP_SUSPEND_OCRAM_SIZE	0x1000

#endif
