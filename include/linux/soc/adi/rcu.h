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

#ifndef SOC_ADI_RCU_H
#define SOC_ADI_RCU_H

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/types.h>

/* Register offsets */
#define ADI_RCU_REG_CTL				0x00
#define ADI_RCU_REG_STAT			0x04
#define ADI_RCU_REG_CRCTL			0x08
#define ADI_RCU_REG_CRSTAT			0x0c

#ifdef CONFIG_ARCH_SC58X
#define ADI_RCU_REG_SIDIS			0x10
#define ADI_RCU_REG_SISTAT			0x14
#define ADI_RCU_REG_SVECT_LCK		0x18
#define ADI_RCU_REG_BCODE			0x1c
#define ADI_RCU_REG_SVECT0			0x20
#define ADI_RCU_REG_SVECT1			0x24
#define ADI_RCU_REG_SVECT2			0x28
#define ADI_RCU_REG_MSG				0x60
#define ADI_RCU_REG_MSG_SET			0x64
#define ADI_RCU_REG_MSG_CLR			0x68
#else
#define ADI_RCU_REG_SRRQSTAT		0x18
#define ADI_RCU_REG_SIDIS			0x1c
#define ADI_RCU_REG_SISTAT			0x20
#define ADI_RCU_REG_BCODE			0x28
#define ADI_RCU_REG_SVECT0			0x2c
#define ADI_RCU_REG_SVECT1			0x30
#define ADI_RCU_REG_SVECT2			0x34
#define ADI_RCU_REG_MSG				0x6c
#define ADI_RCU_REG_MSG_SET			0x70
#define ADI_RCU_REG_MSG_CLR			0x74
#endif

/* Register bit definitions */
#define ADI_RCU_CTL_SYSRST		BIT(0)

/* Bit values for the RCU0_MSG register */
#define RCU0_MSG_C0IDLE			0x00000100	/* Core 0 Idle */
#define RCU0_MSG_C1IDLE			0x00000200	/* Core 1 Idle */
#define RCU0_MSG_C2IDLE			0x00000400	/* Core 2 Idle */
#define RCU0_MSG_CRR0			0x00001000	/* Core 0 reset request */
#define RCU0_MSG_CRR1			0x00002000	/* Core 1 reset request */
#define RCU0_MSG_CRR2			0x00004000	/* Core 2 reset request */
#define RCU0_MSG_C1ACTIVATE		0x00080000	/* Core 1 Activated */
#define RCU0_MSG_C2ACTIVATE		0x00100000	/* Core 2 Activated */

struct adi_rcu;
struct adi_sec;

/*
 * Get the RCU instance connected to the given device as a device tree phandle
 * in a property named "adi,rcu"
 *
 * call put_adi_rcu when done with the rcu entirely
 */
struct adi_rcu *get_adi_rcu_from_node(struct device *dev);
void put_adi_rcu(struct adi_rcu *rcu);
void adi_rcu_msg_set(struct adi_rcu *rcu, u32 bits);
void adi_rcu_msg_clear(struct adi_rcu *rcu, u32 bits);

/*
 * These are defined for use with SHARC cores not ARM cores
 */
int adi_rcu_check_coreid_valid(struct adi_rcu *rcu, int coreid);
int adi_rcu_reset_core(struct adi_rcu *rcu, int coreid);
int adi_rcu_start_core(struct adi_rcu *rcu, int coreid);
int adi_rcu_stop_core(struct adi_rcu *rcu, int coreid, int coreirq);
int adi_rcu_is_core_idle(struct adi_rcu *rcu, int coreid);

void adi_rcu_set_sec(struct adi_rcu *rcu, struct adi_sec *sec);

u32 adi_rcu_readl(struct adi_rcu *rcu, int offset);
void adi_rcu_writel(u32 val, struct adi_rcu *rcu, int offset);

#endif
