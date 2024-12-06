/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#ifndef _PFE_MOD_H_
#define _PFE_MOD_H_

#include <linux/device.h>
#include <linux/elf.h>

extern unsigned int us;

struct pfe;

#include "pfe_hw.h"
#include "pfe_firmware.h"
#include "pfe_ctrl.h"
#include "pfe_hif.h"
#include "pfe_hif_lib.h"
#include "pfe_eth.h"
#include "pfe_sysfs.h"
#include "pfe_perfmon.h"
#include "pfe_debugfs.h"

#define PHYID_MAX_VAL 32

struct pfe_tmu_credit {
	/* Number of allowed TX packet in-flight, matches TMU queue size */
	unsigned int tx_credit[NUM_GEMAC_SUPPORT][EMAC_TXQ_CNT];
	unsigned int tx_credit_max[NUM_GEMAC_SUPPORT][EMAC_TXQ_CNT];
	unsigned int tx_packets[NUM_GEMAC_SUPPORT][EMAC_TXQ_CNT];
};

struct pfe {
	struct regmap	*scfg;
	unsigned long ddr_phys_baseaddr;
	void *ddr_baseaddr;
	unsigned int ddr_size;
	void *cbus_baseaddr;
	void *apb_baseaddr;
	unsigned long iram_phys_baseaddr;
	void *iram_baseaddr;
	unsigned long ipsec_phys_baseaddr;
	void *ipsec_baseaddr;
	int hif_irq;
	int wol_irq;
	int hif_client_irq;
	struct device *dev;
	struct dentry *dentry;
	struct pfe_ctrl ctrl;
	struct pfe_hif hif;
	struct pfe_eth eth;
	struct pfe_mdio mdio;
	struct hif_client_s *hif_client[HIF_CLIENTS_MAX];
#if defined(CFG_DIAGS)
	struct pfe_diags diags;
#endif
	struct pfe_tmu_credit tmu_credit;
	struct pfe_cpumon cpumon;
	struct pfe_memmon memmon;
	int wake;
	int mdio_muxval[PHYID_MAX_VAL];
	struct clk *hfe_clock;
};

extern struct pfe *pfe;

int pfe_probe(struct pfe *pfe);
int pfe_remove(struct pfe *pfe);

/* DDR Mapping in reserved memory*/
#define ROUTE_TABLE_BASEADDR	0
#define ROUTE_TABLE_HASH_BITS	15	/* 32K entries */
#define ROUTE_TABLE_SIZE	((1 << ROUTE_TABLE_HASH_BITS) \
				  * CLASS_ROUTE_SIZE)
#define BMU2_DDR_BASEADDR	(ROUTE_TABLE_BASEADDR + ROUTE_TABLE_SIZE)
#define BMU2_BUF_COUNT		(4096 - 256)
/* This is to get a total DDR size of 12MiB */
#define BMU2_DDR_SIZE		(DDR_BUF_SIZE * BMU2_BUF_COUNT)
#define UTIL_CODE_BASEADDR	(BMU2_DDR_BASEADDR + BMU2_DDR_SIZE)
#define UTIL_CODE_SIZE		(128 * SZ_1K)
#define UTIL_DDR_DATA_BASEADDR	(UTIL_CODE_BASEADDR + UTIL_CODE_SIZE)
#define UTIL_DDR_DATA_SIZE	(64 * SZ_1K)
#define CLASS_DDR_DATA_BASEADDR	(UTIL_DDR_DATA_BASEADDR + UTIL_DDR_DATA_SIZE)
#define CLASS_DDR_DATA_SIZE	(32 * SZ_1K)
#define TMU_DDR_DATA_BASEADDR	(CLASS_DDR_DATA_BASEADDR + CLASS_DDR_DATA_SIZE)
#define TMU_DDR_DATA_SIZE	(32 * SZ_1K)
#define TMU_LLM_BASEADDR	(TMU_DDR_DATA_BASEADDR + TMU_DDR_DATA_SIZE)
#define TMU_LLM_QUEUE_LEN	(8 * 512)
/* Must be power of two and at least 16 * 8 = 128 bytes */
#define TMU_LLM_SIZE		(4 * 16 * TMU_LLM_QUEUE_LEN)
/* (4 TMU's x 16 queues x queue_len) */

#define DDR_MAX_SIZE		(TMU_LLM_BASEADDR + TMU_LLM_SIZE)

/* LMEM Mapping */
#define BMU1_LMEM_BASEADDR	0
#define BMU1_BUF_COUNT		256
#define BMU1_LMEM_SIZE		(LMEM_BUF_SIZE * BMU1_BUF_COUNT)

#endif /* _PFE_MOD_H */
