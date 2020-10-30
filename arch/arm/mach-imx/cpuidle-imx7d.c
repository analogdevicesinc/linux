/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/busfreq-imx.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/delay.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/psci.h>
#include <asm/cp15.h>
#include <asm/cpuidle.h>
#include <asm/fncpy.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/tlb.h>

#include <uapi/linux/psci.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

#define XTALOSC24M_OSC_CONFIG0	0x10
#define XTALOSC24M_OSC_CONFIG1	0x20
#define XTALOSC24M_OSC_CONFIG2	0x30
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_CUR_SHIFT	24
#define XTALOSC24M_OSC_CONFIG0_HYST_MINUS_MASK		0xf
#define XTALOSC24M_OSC_CONFIG0_HYST_MINUS_SHIFT		16
#define XTALOSC24M_OSC_CONFIG0_HYST_PLUS_MASK		0xf
#define XTALOSC24M_OSC_CONFIG0_HYST_PLUS_SHIFT		12
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_SHIFT	4
#define XTALOSC24M_OSC_CONFIG0_ENABLE_SHIFT		1
#define XTALOSC24M_OSC_CONFIG0_START_SHIFT		0
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_CUR_SHIFT	20
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_TRG_SHIFT	0
#define XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_MASK	0xfff
#define XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_SHIFT	0

#define XTALOSC_CTRL_24M				0x0
#define XTALOSC_CTRL_24M_RC_OSC_EN_SHIFT		13
#define REG_SET						0x4

static void __iomem *wfi_iram_base;
static void __iomem *wfi_iram_base_phys;
extern unsigned long iram_tlb_phys_addr;

struct imx7_pm_base {
	phys_addr_t pbase;
	void __iomem *vbase;
};

struct imx7_cpuidle_pm_info {
	phys_addr_t vbase; /* The virtual address of pm_info. */
	phys_addr_t pbase; /* The physical address of pm_info. */
	phys_addr_t resume_addr; /* The physical resume address for asm code */
	u32 pm_info_size;
	u32 ttbr;
	u32 num_online_cpus;
	u32 num_lpi_cpus;
	atomic_t val;
	atomic_t flag0;
	atomic_t flag1;
	struct imx7_pm_base ddrc_base;
	struct imx7_pm_base ccm_base;
	struct imx7_pm_base anatop_base;
	struct imx7_pm_base src_base;
	struct imx7_pm_base iomuxc_gpr_base;
	struct imx7_pm_base gpc_base;
	struct imx7_pm_base gic_dist_base;
} __aligned(8);

static atomic_t master_lpi = ATOMIC_INIT(0);
static atomic_t master_wait = ATOMIC_INIT(0);

static void (*imx7d_wfi_in_iram_fn)(void __iomem *iram_vbase);
static struct imx7_cpuidle_pm_info *cpuidle_pm_info;

#define MX7D_POWERDWN_IDLE_PARAM	\
	((1 << PSCI_0_2_POWER_STATE_ID_SHIFT) | \
	 (1 << PSCI_0_2_POWER_STATE_AFFL_SHIFT) | \
	 (PSCI_POWER_STATE_TYPE_POWER_DOWN << PSCI_0_2_POWER_STATE_TYPE_SHIFT))

#define MX7D_STANDBY_IDLE_PARAM	\
	((1 << PSCI_0_2_POWER_STATE_ID_SHIFT) | \
	 (1 << PSCI_0_2_POWER_STATE_AFFL_SHIFT) | \
	 (PSCI_POWER_STATE_TYPE_STANDBY << PSCI_0_2_POWER_STATE_TYPE_SHIFT))

/* Mapped for the kernel, unlike cpuidle_pm_info->gic_dist_base.vbase */
static void __iomem *imx7d_cpuidle_gic_base;

static void imx_pen_lock(int cpu)
{
	if (cpu == 0) {
		atomic_set(&cpuidle_pm_info->flag0, 1);
		dsb();
		atomic_set(&cpuidle_pm_info->val, cpu);
		do {
			dsb();
		} while (atomic_read(&cpuidle_pm_info->flag1) == 1
			&& atomic_read(&cpuidle_pm_info->val) == cpu)
			;
	} else {
		atomic_set(&cpuidle_pm_info->flag1, 1);
		dsb();
		atomic_set(&cpuidle_pm_info->val, cpu);
		do {
			dsb();
		} while (atomic_read(&cpuidle_pm_info->flag0) == 1
			&& atomic_read(&cpuidle_pm_info->val) == cpu)
			;
	}
}

static void imx_pen_unlock(int cpu)
{
	dsb();
	if (cpu == 0)
		atomic_set(&cpuidle_pm_info->flag0, 0);
	else
		atomic_set(&cpuidle_pm_info->flag1, 0);
}

static int imx7d_idle_finish(unsigned long val)
{
	if (psci_ops.cpu_suspend)
		psci_ops.cpu_suspend(MX7D_POWERDWN_IDLE_PARAM, __pa(cpu_resume));
	else
		imx7d_wfi_in_iram_fn(wfi_iram_base);

	return 0;
}

static bool imx7d_gic_sgis_pending(void)
{
	void __iomem *sgip_base = imx7d_cpuidle_gic_base + 0x1f20;

	return (readl_relaxed(sgip_base + 0x0) |
		readl_relaxed(sgip_base + 0x4) |
		readl_relaxed(sgip_base + 0x8) |
		readl_relaxed(sgip_base + 0xc));
}

static DEFINE_SPINLOCK(psci_lock);
static int imx7d_enter_low_power_idle(struct cpuidle_device *dev,
			    struct cpuidle_driver *drv, int index)
{
	int mode = get_bus_freq_mode();


	if ((index == 1) || ((mode != BUS_FREQ_LOW) && index == 2)) {
		index = 1;
		if (atomic_inc_return(&master_wait) == num_online_cpus())
			imx_gpcv2_set_lpm_mode(WAIT_UNCLOCKED);

		rcu_idle_enter();
		cpu_do_idle();
		rcu_idle_exit();

		atomic_dec(&master_wait);
		imx_gpcv2_set_lpm_mode(WAIT_CLOCKED);
	} else {
		if (psci_ops.cpu_suspend) {
			cpu_pm_enter();
			spin_lock(&psci_lock);
			if (atomic_inc_return(&master_lpi) == num_online_cpus()) {
				if (imx7d_gic_sgis_pending()) {
					atomic_dec(&master_lpi);
					index = -1;
					goto psci_skip_lpi_flow;
				}

				imx_gpcv2_set_lpm_mode(WAIT_UNCLOCKED);
				imx_gpcv2_set_cpu_power_gate_in_idle(true);

				cpu_cluster_pm_enter();
			}
			spin_unlock(&psci_lock);

			rcu_idle_enter();
			cpu_suspend(0, imx7d_idle_finish);
			rcu_idle_exit();

			spin_lock(&psci_lock);
			if (atomic_read(&master_lpi) == num_online_cpus()) {
				cpu_cluster_pm_exit();
				imx_gpcv2_set_cpu_power_gate_in_idle(false);
				imx_gpcv2_set_lpm_mode(WAIT_CLOCKED);
			}

			atomic_dec(&master_lpi);
psci_skip_lpi_flow:
			spin_unlock(&psci_lock);
			cpu_pm_exit();
		} else {
			imx_pen_lock(dev->cpu);
			cpuidle_pm_info->num_online_cpus = num_online_cpus();
			++cpuidle_pm_info->num_lpi_cpus;
			cpu_pm_enter();
			if (cpuidle_pm_info->num_lpi_cpus ==
					cpuidle_pm_info->num_online_cpus) {
				/*
				 * GPC will not wake on SGIs so check for them
				 * manually here. At this point we know the other cpu
				 * is in wfi or waiting for the lock and can't send
				 * any additional IPIs.
				 */
				if (imx7d_gic_sgis_pending()) {
					index = -1;
					goto skip_lpi_flow;
				}
				imx_gpcv2_set_lpm_mode(WAIT_UNCLOCKED);
				imx_gpcv2_set_cpu_power_gate_in_idle(true);
				cpu_cluster_pm_enter();
			} else {
				imx_set_cpu_jump(dev->cpu, ca7_cpu_resume);
			}

			rcu_idle_enter();
			cpu_suspend(0, imx7d_idle_finish);
			rcu_idle_exit();

			if (cpuidle_pm_info->num_lpi_cpus ==
					cpuidle_pm_info->num_online_cpus) {
				cpu_cluster_pm_exit();
				imx_gpcv2_set_cpu_power_gate_in_idle(false);
				imx_gpcv2_set_lpm_mode(WAIT_CLOCKED);
			}

skip_lpi_flow:
			cpu_pm_exit();
			--cpuidle_pm_info->num_lpi_cpus;
			imx_pen_unlock(dev->cpu);
		}
	}

	return index;
}

static struct cpuidle_driver imx7d_cpuidle_driver = {
	.name = "imx7d_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		/* WFI */
		ARM_CPUIDLE_WFI_STATE,
		/* WAIT MODE */
		{
			.exit_latency = 50,
			.target_residency = 75,
			.flags = CPUIDLE_FLAG_TIMER_STOP | CPUIDLE_FLAG_RCU_IDLE,
			.enter = imx7d_enter_low_power_idle,
			.name = "WAIT",
			.desc = "Clock off",
		},
		/* LOW POWER IDLE */
		{
			.exit_latency = 12000,
			.target_residency = 22000,
			.flags = CPUIDLE_FLAG_TIMER_STOP | CPUIDLE_FLAG_RCU_IDLE,
			.enter = imx7d_enter_low_power_idle,
			.name = "LOW-POWER-IDLE",
			.desc = "ARM power off",
		},
	},
	.state_count = 3,
	.safe_state_index = 0,
};

int imx7d_enable_rcosc(void)
{
	void __iomem *anatop_base =
		(void __iomem *)IMX_IO_P2V(MX7D_ANATOP_BASE_ADDR);
	u32 val;

	imx_gpcv2_set_lpm_mode(WAIT_CLOCKED);
	/* set RC-OSC freq and turn it on */
	writel_relaxed(0x1 << XTALOSC_CTRL_24M_RC_OSC_EN_SHIFT,
		anatop_base + XTALOSC_CTRL_24M + REG_SET);
	/*
	 * config RC-OSC freq
	 * tune_enable = 1;tune_start = 1;hyst_plus = 0;hyst_minus = 0;
	 * osc_prog = 0xa7;
	 */
	writel_relaxed(
		0x4 << XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_CUR_SHIFT |
		0xa7 << XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_SHIFT |
		0x1 << XTALOSC24M_OSC_CONFIG0_ENABLE_SHIFT |
		0x1 << XTALOSC24M_OSC_CONFIG0_START_SHIFT,
		anatop_base + XTALOSC24M_OSC_CONFIG0);
	/* set count_trg = 0x2dc */
	writel_relaxed(
		0x40 << XTALOSC24M_OSC_CONFIG1_COUNT_RC_CUR_SHIFT |
		0x2dc << XTALOSC24M_OSC_CONFIG1_COUNT_RC_TRG_SHIFT,
		anatop_base + XTALOSC24M_OSC_CONFIG1);
	/* wait at least 4ms according to hardware design */
	mdelay(6);
	/*
	 * now add some hysteresis, hyst_plus=3, hyst_minus=3
	 * (the minimum hysteresis that looks good is 2)
	 */
	val = readl_relaxed(anatop_base + XTALOSC24M_OSC_CONFIG0);
	val &= ~((XTALOSC24M_OSC_CONFIG0_HYST_MINUS_MASK <<
		XTALOSC24M_OSC_CONFIG0_HYST_MINUS_SHIFT) |
		(XTALOSC24M_OSC_CONFIG0_HYST_PLUS_MASK <<
		XTALOSC24M_OSC_CONFIG0_HYST_PLUS_SHIFT));
	val |= (0x3 << XTALOSC24M_OSC_CONFIG0_HYST_MINUS_SHIFT) |
		(0x3 << XTALOSC24M_OSC_CONFIG0_HYST_PLUS_SHIFT);
	writel_relaxed(val, anatop_base + XTALOSC24M_OSC_CONFIG0);
	/* set the count_1m_trg = 0x2d7 */
	val = readl_relaxed(anatop_base + XTALOSC24M_OSC_CONFIG2);
	val &= ~(XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_MASK <<
		XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_SHIFT);
	val |= 0x2d7 << XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_SHIFT;
	writel_relaxed(val, anatop_base + XTALOSC24M_OSC_CONFIG2);
	/*
	 * hardware design require to write XTALOSC24M_OSC_CONFIG0 or
	 * XTALOSC24M_OSC_CONFIG1 to
	 * make XTALOSC24M_OSC_CONFIG2 write work
	 */
	val = readl_relaxed(anatop_base + XTALOSC24M_OSC_CONFIG1);
	writel_relaxed(val, anatop_base + XTALOSC24M_OSC_CONFIG1);

	return 0;
}

int __init imx7d_cpuidle_init(void)
{
	wfi_iram_base_phys = (void *)(iram_tlb_phys_addr +
		MX7_CPUIDLE_OCRAM_ADDR_OFFSET);

	/* Make sure wfi_iram_base is 8 byte aligned. */
	if ((uintptr_t)(wfi_iram_base_phys) & (FNCPY_ALIGN - 1))
		wfi_iram_base_phys += FNCPY_ALIGN -
		((uintptr_t)wfi_iram_base_phys % (FNCPY_ALIGN));

	wfi_iram_base = (void *)IMX_IO_P2V((unsigned long) wfi_iram_base_phys);

	cpuidle_pm_info = wfi_iram_base;
	cpuidle_pm_info->vbase = (phys_addr_t) wfi_iram_base;
	cpuidle_pm_info->pbase = (phys_addr_t) wfi_iram_base_phys;
	cpuidle_pm_info->pm_info_size = sizeof(*cpuidle_pm_info);
	cpuidle_pm_info->resume_addr = virt_to_phys(ca7_cpu_resume);
	cpuidle_pm_info->num_online_cpus = num_online_cpus();

	cpuidle_pm_info->ddrc_base.pbase = MX7D_DDRC_BASE_ADDR;
	cpuidle_pm_info->ddrc_base.vbase =
		(void __iomem *)IMX_IO_P2V(MX7D_DDRC_BASE_ADDR);

	cpuidle_pm_info->ccm_base.pbase = MX7D_CCM_BASE_ADDR;
	cpuidle_pm_info->ccm_base.vbase =
		(void __iomem *)IMX_IO_P2V(MX7D_CCM_BASE_ADDR);

	cpuidle_pm_info->anatop_base.pbase = MX7D_ANATOP_BASE_ADDR;
	cpuidle_pm_info->anatop_base.vbase =
		(void __iomem *)IMX_IO_P2V(MX7D_ANATOP_BASE_ADDR);

	cpuidle_pm_info->src_base.pbase = MX7D_SRC_BASE_ADDR;
	cpuidle_pm_info->src_base.vbase =
		(void __iomem *)IMX_IO_P2V(MX7D_SRC_BASE_ADDR);

	cpuidle_pm_info->iomuxc_gpr_base.pbase = MX7D_IOMUXC_GPR_BASE_ADDR;
	cpuidle_pm_info->iomuxc_gpr_base.vbase =
		(void __iomem *)IMX_IO_P2V(MX7D_IOMUXC_GPR_BASE_ADDR);

	cpuidle_pm_info->gpc_base.pbase = MX7D_GPC_BASE_ADDR;
	cpuidle_pm_info->gpc_base.vbase =
		(void __iomem *)IMX_IO_P2V(MX7D_GPC_BASE_ADDR);

	cpuidle_pm_info->gic_dist_base.pbase = MX7D_GIC_BASE_ADDR;
	cpuidle_pm_info->gic_dist_base.vbase =
		(void __iomem *)IMX_IO_P2V(MX7D_GIC_BASE_ADDR);

	imx7d_cpuidle_gic_base = ioremap(MX7D_GIC_BASE_ADDR, MX7D_GIC_SIZE);

	imx7d_enable_rcosc();

	/* code size should include cpuidle_pm_info size */
	if (!psci_ops.cpu_suspend) {
		imx7d_wfi_in_iram_fn = (void *)fncpy(wfi_iram_base +
			sizeof(*cpuidle_pm_info),
			&imx7d_low_power_idle,
			MX7_CPUIDLE_OCRAM_SIZE - sizeof(*cpuidle_pm_info));
	}

	return cpuidle_register(&imx7d_cpuidle_driver, NULL);
}
