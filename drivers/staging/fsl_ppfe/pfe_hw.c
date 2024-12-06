// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include "pfe_mod.h"
#include "pfe_hw.h"

/* Functions to handle most of pfe hw register initialization */
int pfe_hw_init(struct pfe *pfe, int resume)
{
	struct class_cfg class_cfg = {
		.pe_sys_clk_ratio = PE_SYS_CLK_RATIO,
		.route_table_baseaddr = pfe->ddr_phys_baseaddr +
					ROUTE_TABLE_BASEADDR,
		.route_table_hash_bits = ROUTE_TABLE_HASH_BITS,
	};

	struct tmu_cfg tmu_cfg = {
		.pe_sys_clk_ratio = PE_SYS_CLK_RATIO,
		.llm_base_addr = pfe->ddr_phys_baseaddr + TMU_LLM_BASEADDR,
		.llm_queue_len = TMU_LLM_QUEUE_LEN,
	};

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	struct util_cfg util_cfg = {
		.pe_sys_clk_ratio = PE_SYS_CLK_RATIO,
	};
#endif

	struct BMU_CFG bmu1_cfg = {
		.baseaddr = CBUS_VIRT_TO_PFE(LMEM_BASE_ADDR +
						BMU1_LMEM_BASEADDR),
		.count = BMU1_BUF_COUNT,
		.size = BMU1_BUF_SIZE,
		.low_watermark = 10,
		.high_watermark = 15,
	};

	struct BMU_CFG bmu2_cfg = {
		.baseaddr = DDR_PHYS_TO_PFE(pfe->ddr_phys_baseaddr +
						BMU2_DDR_BASEADDR),
		.count = BMU2_BUF_COUNT,
		.size = BMU2_BUF_SIZE,
		.low_watermark = 250,
		.high_watermark = 253,
	};

	struct gpi_cfg egpi1_cfg = {
		.lmem_rtry_cnt = EGPI1_LMEM_RTRY_CNT,
		.tmlf_txthres = EGPI1_TMLF_TXTHRES,
		.aseq_len = EGPI1_ASEQ_LEN,
		.mtip_pause_reg = CBUS_VIRT_TO_PFE(EMAC1_BASE_ADDR +
						EMAC_TCNTRL_REG),
	};

	struct gpi_cfg egpi2_cfg = {
		.lmem_rtry_cnt = EGPI2_LMEM_RTRY_CNT,
		.tmlf_txthres = EGPI2_TMLF_TXTHRES,
		.aseq_len = EGPI2_ASEQ_LEN,
		.mtip_pause_reg = CBUS_VIRT_TO_PFE(EMAC2_BASE_ADDR +
						EMAC_TCNTRL_REG),
	};

	struct gpi_cfg hgpi_cfg = {
		.lmem_rtry_cnt = HGPI_LMEM_RTRY_CNT,
		.tmlf_txthres = HGPI_TMLF_TXTHRES,
		.aseq_len = HGPI_ASEQ_LEN,
		.mtip_pause_reg = 0,
	};

	pr_info("%s\n", __func__);

#if !defined(LS1012A_PFE_RESET_WA)
	/* LS1012A needs this to make PE work correctly */
	writel(0x3,     CLASS_PE_SYS_CLK_RATIO);
	writel(0x3,     TMU_PE_SYS_CLK_RATIO);
	writel(0x3,     UTIL_PE_SYS_CLK_RATIO);
	usleep_range(10, 20);
#endif

	pr_info("CLASS version: %x\n", readl(CLASS_VERSION));
	pr_info("TMU version: %x\n", readl(TMU_VERSION));

	pr_info("BMU1 version: %x\n", readl(BMU1_BASE_ADDR +
		BMU_VERSION));
	pr_info("BMU2 version: %x\n", readl(BMU2_BASE_ADDR +
		BMU_VERSION));

	pr_info("EGPI1 version: %x\n", readl(EGPI1_BASE_ADDR +
		GPI_VERSION));
	pr_info("EGPI2 version: %x\n", readl(EGPI2_BASE_ADDR +
		GPI_VERSION));
	pr_info("HGPI version: %x\n", readl(HGPI_BASE_ADDR +
		GPI_VERSION));

	pr_info("HIF version: %x\n", readl(HIF_VERSION));
	pr_info("HIF NOPCY version: %x\n", readl(HIF_NOCPY_VERSION));

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	pr_info("UTIL version: %x\n", readl(UTIL_VERSION));
#endif
	while (!(readl(TMU_CTRL) & ECC_MEM_INIT_DONE))
		;

	hif_rx_disable();
	hif_tx_disable();

	bmu_init(BMU1_BASE_ADDR, &bmu1_cfg);

	pr_info("bmu_init(1) done\n");

	bmu_init(BMU2_BASE_ADDR, &bmu2_cfg);

	pr_info("bmu_init(2) done\n");

	class_cfg.resume = resume ? 1 : 0;

	class_init(&class_cfg);

	pr_info("class_init() done\n");

	tmu_init(&tmu_cfg);

	pr_info("tmu_init() done\n");
#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	util_init(&util_cfg);

	pr_info("util_init() done\n");
#endif
	gpi_init(EGPI1_BASE_ADDR, &egpi1_cfg);

	pr_info("gpi_init(1) done\n");

	gpi_init(EGPI2_BASE_ADDR, &egpi2_cfg);

	pr_info("gpi_init(2) done\n");

	gpi_init(HGPI_BASE_ADDR, &hgpi_cfg);

	pr_info("gpi_init(hif) done\n");

	bmu_enable(BMU1_BASE_ADDR);

	pr_info("bmu_enable(1) done\n");

	bmu_enable(BMU2_BASE_ADDR);

	pr_info("bmu_enable(2) done\n");

	return 0;
}

void pfe_hw_exit(struct pfe *pfe)
{
	pr_info("%s\n", __func__);

	bmu_disable(BMU1_BASE_ADDR);
	bmu_reset(BMU1_BASE_ADDR);

	bmu_disable(BMU2_BASE_ADDR);
	bmu_reset(BMU2_BASE_ADDR);
}
