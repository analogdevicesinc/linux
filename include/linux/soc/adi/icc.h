/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

/*
 * @todo next step is to unify the arm/mach-sc5xx headers under linux/soc/adi
 * for now we forward to them based on platform
 */

#ifndef SOC_ADI_ICC_H
#define SOC_ADI_ICC_H

#include <linux/device.h>
#include <linux/types.h>
#include <asm/cacheflush.h>

#define sm_atomic_read(v) ioread16(v)
#define sm_atomic_write(i, v) iowrite16(v, i)
#define invalidate_dcache_range(start, end) __sync_cache_range_r((void *)start, end - start)
#define flush_dcache_range(start, end) __sync_cache_range_w((void *)start, end - start)
#define arm_core_id()	0

#ifdef CONFIG_ARCH_SC59X_64
#define ICC_CODE_START		0x20080000
#elif defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC57X)
#define ICC_CODE_START		0x20000000
/* RCU0 (reset core unit) register structure */
struct rcu_reg {
	/* RCU0 Control Register */
	u32 reg_rcu_ctl;			/* 0x00 */
	/* RCU0 Status Register */
	u32 reg_rcu_stat;			/* 0x04 */
	/* RCU0 Core Reset Control Register */
	u32 reg_rcu_crctl;			/* 0x08 */
	/* RCU0 Core Reset Status Register */
	u32 reg_rcu_crstat;		/* 0x0c */
	/* reg pad from 0x10 to 0x17 */
	u8 pad_0x10_0x17[0x18 - 0x10];	/* 0x10 ~ 0x17 */
	/* RCU0 System Reset Status Register */
	u32 reg_rcu_srrqstat;		/* 0x18 */
	/* RCU0 System Interface Disable Register */
	u32 reg_rcu_sidis;			/* 0x1c */
	/* RCU0 System Interface Status Register */
	u32 reg_rcu_sistat;		/* 0x20 */
	/* reg pad from 0x24 to 0x27 */
	u8 pad_0x24_0x27[0x28 - 0x24];	/* 0x24 ~ 0x27 */
	/* RCU0 Boot Code Register */
	u32 reg_rcu_bcode;			/* 0x28 */
	/* Software Vector Register 0 to 2 */
	u32 reg_rcu_svect0;		/* 0x2c */
	u32 reg_rcu_svect1;		/* 0x30 */
	u32 reg_rcu_svect2;		/* 0x34 */
	/* reg pad from 0x38 to 0x6b */
	u8 pad_0x38_0x6b[0x6C - 0x38];	/* 0x38 ~ 0x6b */
	/* RCU0 Message Register */
	u32 reg_rcu_msg;			/* 0x6C */
	/* RCU0 Message Set Bits Register */
	u32 reg_rcu_msg_set;		/* 0x70 */
	/* RCU0 Message Clear Bits Register */
	u32 reg_rcu_msg_clr;		/* 0x74 */
};
#elif defined(CONFIG_ARCH_SC58X)
#define ICC_CODE_START		0x20080000
/* RCU0 (reset core unit) register structure */
struct rcu_reg {
	/* RCU0 Control Register */
	u32 reg_rcu_ctl;			/* 0x00 */
	/* RCU0 Status Register */
	u32 reg_rcu_stat;			/* 0x04 */
	/* RCU0 Core Reset Control Register */
	u32 reg_rcu_crctl;			/* 0x08 */
	/* RCU0 Core Reset Status Register */
	u32 reg_rcu_crstat;		/* 0x0c */
	/* RCU0 System Interface Disable Register */
	u32 reg_rcu_sidis;			/* 0x10 */
	/* RCU0 System Interface Status Register */
	u32 reg_rcu_sistat;		/* 0x14 */
	/* RCU0 SVECT Lock Register */
	u32 reg_rcu_svect_lck;		/* 0x18 */
	/* RCU0 Boot Code Register */
	u32 reg_rcu_bcode;			/* 0x1c */
	/* Software Vector Register 0 to 2 */
	u32 reg_rcu_svect0;		/* 0x20 */
	u32 reg_rcu_svect1;		/* 0x24 */
	u32 reg_rcu_svect2;		/* 0x28 */
	/* reg pad from 0x2c to 0x63 */
	u8 pad_0x2c_0x59[0x60 - 0x2c];	/* 0x2c ~ 0x59 */
	/* RCU0 Message Register */
	u32 reg_rcu_msg;			/* 0x60 */
	/* RCU0 Message Set Bits Register */
	u32 reg_rcu_msg_set;		/* 0x64 */
	/* RCU0 Message Clear Bits Register */
	u32 reg_rcu_msg_clr;		/* 0x68 */
};
#endif

#define ADI_RESOURCE_TABLE_TAG "AD-RESOURCE-TBL"
#define ADI_RSC_TABLE_INIT_MAGIC (0xADE0AD0E)
#define ADI_RESOURCE_TABLE_VERSION (1)
struct adi_resource_table_hdr {
	u8 tag[16];
	u32 version;
	u32 initialized;
	u32 reserved[8];
} __packed;

struct adi_tru;

struct adi_tru *get_adi_tru_from_node(struct device *dev);
void put_adi_tru(struct adi_tru *tru);
int adi_tru_trigger_device(struct adi_tru *tru, struct device *dev);
int adi_tru_trigger(struct adi_tru *tru, u32 master);
int adi_tru_set_trigger_by_id(struct adi_tru *tru, u32 master, u32 slave);
extern int adi_tru_probe(struct platform_device *pdev);
extern void adi_tru_remove(struct platform_device *pdev);
extern int adi_tru_set_trigger(struct adi_tru *tru,
			       struct device_node *master,
			       struct device_node *slave);

#endif
