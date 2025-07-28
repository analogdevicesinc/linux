/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
 */

#ifndef _DPU_HW_INTERRUPTS_H
#define _DPU_HW_INTERRUPTS_H

#include <linux/types.h>

#include "dpu_hwio.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_util.h"
#include "dpu_hw_mdss.h"

/* When making changes be sure to sync with dpu_intr_set */
enum dpu_hw_intr_reg {
	MDP_SSPP_TOP0_INTR,
	MDP_SSPP_TOP0_INTR2,
	MDP_SSPP_TOP0_HIST_INTR,
	/* All MDP_INTFn_INTR should come sequentially */
	MDP_INTF0_INTR,
	MDP_INTF1_INTR,
	MDP_INTF2_INTR,
	MDP_INTF3_INTR,
	MDP_INTF4_INTR,
	MDP_INTF5_INTR,
	MDP_INTF6_INTR,
	MDP_INTF7_INTR,
	MDP_INTF8_INTR,
	MDP_INTF1_TEAR_INTR,
	MDP_INTF2_TEAR_INTR,
	MDP_AD4_0_INTR,
	MDP_AD4_1_INTR,
	MDP_INTR_MAX,
};

#define MDP_INTFn_INTR(intf)	(MDP_INTF0_INTR + (intf - INTF_0))

#define DPU_IRQ_IDX(reg_idx, offset)	(1 + reg_idx * 32 + offset)
#define DPU_IRQ_REG(irq_idx)	((irq_idx - 1) / 32)
#define DPU_IRQ_BIT(irq_idx)	((irq_idx - 1) % 32)

#define DPU_NUM_IRQS		(MDP_INTR_MAX * 32)

struct dpu_hw_intr_entry {
	void (*cb)(void *arg);
	void *arg;
	atomic_t count;
};

/**
 * struct dpu_hw_intr: hw interrupts handling data structure
 * @hw:               virtual address mapping
 * @ops:              function pointer mapping for IRQ handling
 * @cache_irq_mask:   array of IRQ enable masks reg storage created during init
 * @save_irq_status:  array of IRQ status reg storage created during init
 * @irq_lock:         spinlock for accessing IRQ resources
 * @irq_cb_tbl:       array of IRQ callbacks
 */
struct dpu_hw_intr {
	struct dpu_hw_blk_reg_map hw;
	u32 cache_irq_mask[MDP_INTR_MAX];
	u32 *save_irq_status;
	spinlock_t irq_lock;
	unsigned long irq_mask;
	const struct dpu_intr_reg *intr_set;

	struct dpu_hw_intr_entry irq_tbl[DPU_NUM_IRQS];
};

/**
 * dpu_hw_intr_init(): Initializes the interrupts hw object
 * @dev:  Corresponding device for devres management
 * @addr: mapped register io address of MDP
 * @m:    pointer to MDSS catalog data
 */
struct dpu_hw_intr *dpu_hw_intr_init(struct drm_device *dev,
				     void __iomem *addr,
				     const struct dpu_mdss_cfg *m);

#endif
