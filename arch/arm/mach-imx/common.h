/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2004-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */


#ifndef __ASM_ARCH_MXC_COMMON_H__
#define __ASM_ARCH_MXC_COMMON_H__

#include <linux/reboot.h>
#include <soc/imx/src.h>

struct irq_data;
struct platform_device;
struct pt_regs;
struct clk;
struct device_node;
enum mxc_cpu_pwr_mode;
struct of_device_id;

void mx31_map_io(void);
void mx35_map_io(void);
void imx21_init_early(void);
void imx31_init_early(void);
void imx35_init_early(void);
void mx31_init_irq(void);
void mx35_init_irq(void);
void mxc_set_cpu_type(unsigned int type);
void mxc_restart(enum reboot_mode, const char *);
void mxc_arch_reset_init(void __iomem *);
void imx1_reset_init(void __iomem *);
void imx_set_aips(void __iomem *);
void imx_aips_allow_unprivileged_access(const char *compat);
int mxc_device_init(void);
void imx_set_soc_revision(unsigned int rev);
void imx_init_revision_from_anatop(void);
void imx6_enable_rbc(bool enable);
void imx_gpc_check_dt(void);
void imx_gpc_set_arm_power_in_lpm(bool power_off);
void imx_gpc_set_l2_mem_power_in_lpm(bool power_off);
void imx_gpc_set_arm_power_up_timing(u32 sw2iso, u32 sw);
void imx_gpc_set_arm_power_down_timing(u32 sw2iso, u32 sw);
void imx_gpcv2_pre_suspend(bool arm_power_off);
void imx_gpcv2_post_resume(void);
unsigned int imx_gpcv2_is_mf_mix_off(void);
void imx_gpcv2_enable_wakeup_for_m4(void);
void imx_gpcv2_disable_wakeup_for_m4(void);
void imx25_pm_init(void);
void imx27_pm_init(void);
void imx5_pmu_init(void);
#ifdef CONFIG_HAVE_IMX_MU
int imx_mu_lpm_ready(bool ready);
#else
static inline int imx_mu_lpm_ready(bool ready) { return 0; }
#endif

enum mxc_cpu_pwr_mode {
	WAIT_CLOCKED,		/* wfi only */
	WAIT_UNCLOCKED,		/* WAIT */
	WAIT_UNCLOCKED_POWER_OFF,	/* WAIT + SRPG */
	STOP_POWER_ON,		/* just STOP */
	STOP_POWER_OFF,		/* STOP + SRPG */
};

enum ulp_cpu_pwr_mode {
	ULP_PM_HSRUN,    /* High speed run mode */
	ULP_PM_RUN,      /* Run mode */
	ULP_PM_WAIT,     /* Wait mode */
	ULP_PM_STOP,     /* Stop mode */
	ULP_PM_VLPS,     /* Very low power stop mode */
	ULP_PM_VLLS,     /* very low leakage stop mode */
};

void imx_enable_cpu(int cpu, bool enable);
void imx_set_cpu_jump(int cpu, void *jump_addr);
u32 imx_get_cpu_arg(int cpu);
void imx_set_cpu_arg(int cpu, u32 arg);
#ifdef CONFIG_SMP
void v7_secondary_startup(void);
void imx_scu_map_io(void);
void imx_smp_prepare(void);
#else
static inline void imx_scu_map_io(void) {}
static inline void imx_smp_prepare(void) {}
#endif
void imx6sx_set_m4_highfreq(bool high_freq);
void imx_mu_enable_m4_irqs_in_gic(bool enable);
#ifdef CONFIG_HAVE_IMX_GPC
void imx_gpc_add_m4_wake_up_irq(u32 irq, bool enable);
unsigned int imx_gpc_is_m4_sleeping(void);
#else
static inline void imx_gpc_add_m4_wake_up_irq(u32 irq, bool enable) {}
static inline unsigned int imx_gpc_is_m4_sleeping(void) { return 0; }
#endif
#ifdef CONFIG_HAVE_IMX_GPCV2
int imx_gpcv2_mf_power_on(unsigned int irq, unsigned int on);
void imx_gpcv2_set_core1_pdn_pup_by_software(bool pdn);
void imx_gpcv2_add_m4_wake_up_irq(u32 hwirq, bool enable);
#else
static inline int imx_gpcv2_mf_power_on(unsigned int irq, unsigned int on) { return 0; }
static inline void imx_gpcv2_set_core1_pdn_pup_by_software(bool pdn) {}
static inline void imx_gpcv2_add_m4_wake_up_irq(u32 hwirq, bool enable) {}
#endif
void imx_gpc_hold_m4_in_sleep(void);
void imx_gpc_release_m4_in_sleep(void);
void __init imx_gpcv2_check_dt(void);
void imx_gpcv2_set_lpm_mode(enum mxc_cpu_pwr_mode mode);
void imx_gpcv2_set_cpu_power_gate_in_idle(bool pdn);
void imx_gpcv2_enable_rbc(bool enable);
bool imx_mu_is_m4_in_low_freq(void);
bool imx_mu_is_m4_in_stop(void);
void imx_mu_set_m4_run_mode(void);
void imx_src_init(void);
void imx_gpc_pre_suspend(bool arm_power_off);
void imx_gpc_post_resume(void);
void imx_gpc_switch_pupscr_clk(bool flag);
void imx_gpc_mask_all(void);
void imx_gpc_restore_all(void);
void imx_gpc_hwirq_mask(unsigned int hwirq);
void imx_gpc_hwirq_unmask(unsigned int hwirq);
unsigned int imx_gpc_is_mf_mix_off(void);
void imx_anatop_init(void);
void imx_anatop_pre_suspend(void);
void imx_anatop_post_resume(void);
int imx6_set_lpm(enum mxc_cpu_pwr_mode mode);
void imx6_set_int_mem_clk_lpm(bool enable);
void imx6_enet_mac_init(const char *enet_compat, const char *ocotp_compat);
void imx6sl_low_power_idle(void);
void imx6sll_low_power_idle(void);
void imx6sx_low_power_idle(void);
void imx6ul_low_power_idle(void);
void imx6ull_low_power_idle(void);
void imx7d_low_power_idle(void);
#ifdef CONFIG_HAVE_IMX_MMDC
int imx_mmdc_get_ddr_type(void);
int imx_mmdc_get_lpddr2_2ch_mode(void);
#else
static inline int imx_mmdc_get_ddr_type(void) { return 0; }
static inline int imx_mmdc_get_lpddr2_2ch_mode(void) { return 0; }
#endif
int imx7ulp_set_lpm(enum ulp_cpu_pwr_mode mode);
u32 imx7ulp_get_mode(void);
void imx_busfreq_map_io(void);
void imx7_pm_map_io(void);
void imx6_pm_map_io(void);
void imx7ulp_pm_map_io(void);
void imx7ulp_enable_nmi(void);
void imx7ulp_poweroff(void);

void imx_cpu_die(unsigned int cpu);
int imx_cpu_kill(unsigned int cpu);

#ifdef CONFIG_SUSPEND
void ca7_cpu_resume(void);
void imx53_suspend(void __iomem *ocram_vbase);
extern const u32 imx53_suspend_sz;
void imx6_suspend(void __iomem *ocram_vbase);
void imx7_suspend(void __iomem *ocram_vbase);
void imx7ulp_cpu_resume(void);
void imx7ulp_suspend(void __iomem *ocram_vbase);
#else
static inline void ca7_cpu_resume(void) {}
static inline void imx53_suspend(void __iomem *ocram_vbase) {}
static const u32 imx53_suspend_sz;
static inline void imx6_suspend(void __iomem *ocram_vbase) {}
static inline void imx7_suspend(void __iomem *ocram_vbase) {}
static inline void imx7ulp_cpu_resume(void) {}
static inline void imx7ulp_suspend(void __iomem *ocram_vbase) {}
#endif

void v7_cpu_resume(void);

#ifdef CONFIG_HAVE_IMX_DDRC
int imx_ddrc_get_ddr_type(void);
#else
static inline int imx_ddrc_get_ddr_type(void) { return 0; }
#endif

void imx6_pm_ccm_init(const char *ccm_compat);
void imx6q_pm_init(void);
void imx6dl_pm_init(void);
void imx6sl_pm_init(void);
void imx6sx_pm_init(void);
void imx6ul_pm_init(void);
void imx7d_pm_init(void);
void imx7ulp_pm_init(void);

#ifdef CONFIG_PM
void imx51_pm_init(void);
void imx53_pm_init(void);
#else
static inline void imx51_pm_init(void) {}
static inline void imx53_pm_init(void) {}
#endif

#ifdef CONFIG_NEON
int mx51_neon_fixup(void);
#else
static inline int mx51_neon_fixup(void) { return 0; }
#endif

#ifdef CONFIG_CACHE_L2X0
void imx_init_l2cache(void);
#else
static inline void imx_init_l2cache(void) {}
#endif

extern const struct smp_operations imx_smp_ops;
extern const struct smp_operations ls1021a_smp_ops;

extern bool uart_from_osc;
#endif
