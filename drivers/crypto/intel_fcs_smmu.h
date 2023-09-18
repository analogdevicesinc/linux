// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022, Intel Corporation
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/hw_random.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/firmware/intel/stratix10-svc-client.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>

#include <uapi/linux/intel_fcs-ioctl.h>

#ifndef INTEL_FCS_SMMU_H
#define INTEL_FCS_SMMU_H

extern uint64_t smmu_sdm_l3_def_table[512];
extern uint64_t smmu_sdm_el3_l1_table[512];
extern uint64_t smmu_sdm_el3_l2_table[512];
extern uint64_t *l2_table;
extern uint64_t *l1_table;
extern uint64_t *l3_tables[512];

struct intel_fcs_priv {
	struct stratix10_svc_chan *chan;
	struct stratix10_svc_client client;
	struct completion completion;
	struct mutex lock;
	struct miscdevice miscdev;
	unsigned int status;
	void *kbuf;
	unsigned int size;
	unsigned int cid_low;
	unsigned int cid_high;
	unsigned int sid;
	struct hwrng rng;
	const struct socfpga_fcs_data *p_data;
};

int smmu_program_reg(struct intel_fcs_priv *priv, uint32_t reg_add, uint32_t reg_value);
int smmu_read_reg(struct intel_fcs_priv *priv, uint32_t reg_add);
void invalidate_smmu_tlb_entries(struct intel_fcs_priv *priv);
void intel_fcs_smmu_init(struct intel_fcs_priv *priv);
void context_bank_disable(struct intel_fcs_priv *priv);
void context_bank_enable(struct intel_fcs_priv *priv);
void fill_l3_table(uint64_t phys, int l2_idx, int l3_idx);

#endif
