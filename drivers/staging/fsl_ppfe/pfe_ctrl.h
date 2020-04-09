/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#ifndef _PFE_CTRL_H_
#define _PFE_CTRL_H_

#include <linux/dmapool.h>

#include "pfe/pfe.h"

#define DMA_BUF_SIZE_128	0x80	/* enough for 1 conntracks */
#define DMA_BUF_SIZE_256	0x100
/* enough for 2 conntracks, 1 bridge entry or 1 multicast entry */
#define DMA_BUF_SIZE_512	0x200
/* 512bytes dma allocated buffers used by rtp relay feature */
#define DMA_BUF_MIN_ALIGNMENT	8
#define DMA_BUF_BOUNDARY	(4 * 1024)
/* bursts can not cross 4k boundary */

#define CMD_TX_ENABLE	0x0501
#define CMD_TX_DISABLE	0x0502

#define CMD_RX_LRO		0x0011
#define CMD_PKTCAP_ENABLE       0x0d01
#define CMD_QM_EXPT_RATE	0x020c

#define CLASS_DM_SH_STATIC		(0x800)
#define CLASS_DM_CPU_TICKS		(CLASS_DM_SH_STATIC)
#define CLASS_DM_SYNC_MBOX		(0x808)
#define CLASS_DM_MSG_MBOX		(0x810)
#define CLASS_DM_DROP_CNTR		(0x820)
#define CLASS_DM_RESUME			(0x854)
#define CLASS_DM_PESTATUS		(0x860)
#define CLASS_DM_CRC_VALIDATED		(0x14b0)

#define TMU_DM_SH_STATIC		(0x80)
#define TMU_DM_CPU_TICKS		(TMU_DM_SH_STATIC)
#define TMU_DM_SYNC_MBOX		(0x88)
#define TMU_DM_MSG_MBOX			(0x90)
#define TMU_DM_RESUME			(0xA0)
#define TMU_DM_PESTATUS			(0xB0)
#define TMU_DM_CONTEXT			(0x300)
#define TMU_DM_TX_TRANS			(0x480)

#define UTIL_DM_SH_STATIC		(0x0)
#define UTIL_DM_CPU_TICKS		(UTIL_DM_SH_STATIC)
#define UTIL_DM_SYNC_MBOX		(0x8)
#define UTIL_DM_MSG_MBOX		(0x10)
#define UTIL_DM_DROP_CNTR		(0x20)
#define UTIL_DM_RESUME			(0x40)
#define UTIL_DM_PESTATUS		(0x50)

struct pfe_ctrl {
	struct mutex mutex; /* to serialize pfe control access */
	spinlock_t lock;

	void *dma_pool;
	void *dma_pool_512;
	void *dma_pool_128;

	struct device *dev;

	void *hash_array_baseaddr;		/*
						 * Virtual base address of
						 * the conntrack hash array
						 */
	unsigned long hash_array_phys_baseaddr; /*
						 * Physical base address of
						 * the conntrack hash array
						 */

	int (*event_cb)(u16, u16, u16*);

	unsigned long sync_mailbox_baseaddr[MAX_PE]; /*
						      * Sync mailbox PFE
						      * internal address,
						      * initialized
						      * when parsing elf images
						      */
	unsigned long msg_mailbox_baseaddr[MAX_PE]; /*
						     * Msg mailbox PFE internal
						     * address, initialized
						     * when parsing elf images
						     */
	unsigned int sys_clk;			/* AXI clock value, in KHz */
};

int pfe_ctrl_init(struct pfe *pfe);
void pfe_ctrl_exit(struct pfe *pfe);
int pe_sync_stop(struct pfe_ctrl *ctrl, int pe_mask);
void pe_start(struct pfe_ctrl *ctrl, int pe_mask);
int pe_reset_all(struct pfe_ctrl *ctrl);
void pfe_ctrl_suspend(struct pfe_ctrl *ctrl);
void pfe_ctrl_resume(struct pfe_ctrl *ctrl);
int relax(unsigned long end);

#endif /* _PFE_CTRL_H_ */
