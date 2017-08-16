/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright (C) 2017 NXP
 */

#ifndef _MC_ME_H
#define _MC_ME_H

/* MC_ME registers definitions */
/* MC_ME_GS */
 #define MC_ME_GS(mc_me)                ((mc_me) + 0x00000000)

/* MC_ME_MCTL */
#define MC_ME_MCTL(mc_me)               ((mc_me) + 0x00000004)
#define MC_ME_MCTL_RESET                (0x0 << 28)
#define MC_ME_MCTL_TEST                 (0x1 << 28)
#define MC_ME_MCTL_DRUN                 (0x3 << 28)
#define MC_ME_MCTL_RUN0                 (0x4 << 28)
#define MC_ME_MCTL_RUN1                 (0x5 << 28)
#define MC_ME_MCTL_RUN2                 (0x6 << 28)
#define MC_ME_MCTL_RUN3                 (0x7 << 28)

#define MC_ME_GS_S_MTRANS               (1 << 27)

#define MC_ME_MCTL_KEY                  (0x00005AF0)
#define MC_ME_MCTL_INVERTEDKEY          (0x0000A50F)

/*
 * MC_ME_RESET_MC/MC_ME_TEST_MC
 * MC_ME_DRUN_MC
 * MC_ME_RUNn_MC
 */
#define MC_ME_RESET_MC(mc_me)           ((mc_me) + 0x00000020)
#define MC_ME_TEST_MC(mc_me)            ((mc_me) + 0x00000024)
#define MC_ME_DRUN_MC(mc_me)            ((mc_me) + 0x0000002C)
#define MC_ME_RUNn_MC(mc_me, n)         ((mc_me) + 0x00000030 + 0x4 * (n))
#define MC_ME_MODE_MC_SYSCLK_OFFSET     (0)
#define MC_ME_MODE_MC_SYSCLK_SIZE       (0x3)
#define MC_ME_MODE_MC_SYSCLK(val)       (MC_ME_MODE_MC_SYSCLK_MASK & (val))
#define MC_ME_MODE_MC_SYSCLK_MASK       (0x0000000F)
#define MC_ME_MODE_MC_FIRCON            (1 << 4)
#define MC_ME_MODE_MC_XOSCON            (1 << 5)
#define MC_ME_MODE_MC_ARMPLL            (1 << 6)
#define MC_ME_MODE_MC_PERIPHPLL         (1 << 7)
#define MC_ME_MODE_MC_ENETPLL           (1 << 8)
#define MC_ME_MODE_MC_DDRPLL            (1 << 9)
#define MC_ME_MODE_MC_VIDEOPLL          (1 << 10)
#define MC_ME_MODE_MC_MVRON             (1 << 20)

/* MC_ME_DRUN_SEC_CC_I */
#define MC_ME_DRUN_SEC_CC_I(mc_me)              ((mc_me) + 0x260)
/* MC_ME_RUNn_SEC_CC_I */
#define MC_ME_RUNn_SEC_CC_I(mc_me, n)           ((mc_me) + 0x270 + (n) * 0x10)
#define MC_ME_MODE_SEC_CC_I_SYSCLK1_OFFSET      (4)
#define MC_ME_MODE_SEC_CC_I_SYSCLK2_OFFSET      (8)
#define MC_ME_MODE_SEC_CC_I_SYSCLK3_OFFSET      (12)
/* Consider only the defined clocks */
#define MC_ME_MODE_SEC_CC_I_SYSCLK1_SIZE        (0x3)
#define MC_ME_MODE_SEC_CC_I_SYSCLK2_SIZE        (0x3)
#define MC_ME_MODE_SEC_CC_I_SYSCLK3_SIZE        (0x3)

/* MC_ME_RUN_PCn */
#define MC_ME_RUN_PCn(mc_me, n)         (mc_me + 0x00000080 + 0x4 * (n))

#define MC_ME_RUN_PCn_MAX_IDX           (7)
#define MC_ME_RUN_PCn_RESET             (1 << 0)
#define MC_ME_RUN_PCn_TEST              (1 << 1)
#define MC_ME_RUN_PCn_DRUN              (1 << 3)
#define MC_ME_RUN_PCn_RUN0              (1 << 4)
#define MC_ME_RUN_PCn_RUN1              (1 << 5)
#define MC_ME_RUN_PCn_RUN2              (1 << 6)
#define MC_ME_RUN_PCn_RUN3              (1 << 7)

#define MC_ME_PCTLn(mc_me, n)           (mc_me + 0xC0 + 4 * (n >> 2) + \
					(3 - (n) % 4))

static inline void entry_to_target_mode(void __iomem *mc_me, u32 mode)
{
	writel_relaxed(mode | MC_ME_MCTL_KEY, MC_ME_MCTL(mc_me));
	writel_relaxed(mode | MC_ME_MCTL_INVERTEDKEY, MC_ME_MCTL(mc_me));
	while ((readl_relaxed(MC_ME_GS(mc_me)) &
		MC_ME_GS_S_MTRANS) != 0x00000000)
		;
}

static inline void enable_cpumodes_onperipheralconfig(void __iomem *mc_me,
						      u32 modes, u32 run_pc_idx)
{
	WARN_ON(run_pc_idx > MC_ME_RUN_PCn_MAX_IDX);
	if (run_pc_idx > MC_ME_RUN_PCn_MAX_IDX)
		return;

	writel_relaxed(modes, MC_ME_RUN_PCn(mc_me, run_pc_idx));
}

static inline void enable_clocks_sources(u32 flags, u32 clks,
					 void __iomem *xrun_mc_addr)
{
	writel_relaxed(readl_relaxed(xrun_mc_addr) | flags | clks,
		       xrun_mc_addr);
}

static inline void enable_sysclock(u32 clk, void __iomem *xrun_mc_addr)
{
	writel_relaxed(readl_relaxed(xrun_mc_addr) & clk,
		       xrun_mc_addr);
}

#endif
