/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Support Power Management
 *
 * Copyright 2014-2015 Freescale Semiconductor Inc.
 */
#ifndef __PPC_FSL_PM_H
#define __PPC_FSL_PM_H

#ifndef __ASSEMBLY__
#include <linux/suspend.h>

#define E500_PM_PH10	1
#define E500_PM_PH15	2
#define E500_PM_PH20	3
#define E500_PM_PH30	4
#define E500_PM_DOZE	E500_PM_PH10
#define E500_PM_NAP	E500_PM_PH15

#define PLAT_PM_SLEEP	20
#define PLAT_PM_LPM20	30

#define FSL_PM_SLEEP		(1 << 0)
#define FSL_PM_DEEP_SLEEP	(1 << 1)

struct fsl_pm_ops {
	/* mask pending interrupts to the RCPM from MPIC */
	void (*irq_mask)(int cpu);

	/* unmask pending interrupts to the RCPM from MPIC */
	void (*irq_unmask)(int cpu);
	void (*cpu_enter_state)(int cpu, int state);
	void (*cpu_exit_state)(int cpu, int state);
	void (*cpu_up_prepare)(int cpu);
	void (*cpu_die)(int cpu);
	int (*plat_enter_sleep)(void);
	void (*freeze_time_base)(bool freeze);

	/* keep the power of IP blocks during sleep/deep sleep */
	void (*set_ip_power)(bool enable, u32 mask);

	/* get platform supported power management modes */
	unsigned int (*get_pm_modes)(void);
};

extern const struct fsl_pm_ops *qoriq_pm_ops;

struct fsm_reg_vals {
	u32 offset;
	u32 value;
};

void fsl_fsm_setup(void __iomem *base, struct fsm_reg_vals *val);
void fsl_epu_setup_default(void __iomem *epu_base);
void fsl_npc_setup_default(void __iomem *npc_base);
void fsl_fsm_clean(void __iomem *base, struct fsm_reg_vals *val);
void fsl_epu_clean_default(void __iomem *epu_base);

extern int fsl_dp_iomap(void);
extern void fsl_dp_iounmap(void);

extern int fsl_enter_epu_deepsleep(void);
extern void fsl_dp_enter_low(void __iomem *ccsr_base, void __iomem *dcsr_base,
			     void __iomem *pld_base, int pld_flag);
extern void fsl_booke_deep_sleep_resume(void);

int __init fsl_rcpm_init(void);

void set_pm_suspend_state(suspend_state_t state);
suspend_state_t pm_suspend_state(void);

void fsl_set_power_except(struct device *dev, int on);
#endif	/* __ASSEMBLY__ */

#define T1040QDS_TETRA_FLAG	1
#define T104xRDB_CPLD_FLAG	2

#endif /* __PPC_FSL_PM_H */
