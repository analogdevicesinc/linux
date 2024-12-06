// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 */

#include <linux/of_address.h>
#include <linux/clk.h>
#include <dt-bindings/clock/s32v234-clock.h>

#include "clk.h"

static void __iomem *mc_cgm0_base;
static void __iomem *mc_cgm1_base;
static void __iomem *mc_cgm2_base;
static void __iomem *mc_cgm3_base;
static void __iomem *mc_me_base;
static void __iomem *src_base;

DEFINE_SPINLOCK(s32v234_lock);

/* sources for multiplexer clocks, this is used multiple times */
PNAME(osc_sels) = {"firc", "fxosc", };

PNAME(sys_sels) = {"firc", "fxosc", "armpll_dfs0", };

PNAME(can_sels) = {"firc", "fxosc", "dummy",
		   "periphpll_phi0_div5", };

PNAME(lin_sels) = {"firc", "fxosc", "dummy",
		   "periphpll_phi0_div3", "dummy", "dummy",
		   "dummy", "dummy", "sys6",};

PNAME(sdhc_sels) = {"firc", "fxosc", "dummy", "dummy",
		    "enetpll_dfs3",};

PNAME(enet_sels) = {"firc", "fxosc", "dummy",
		    "dummy", "enetpll_phi0",};

PNAME(enet_time_sels) = {"firc", "fxosc", "dummy",
			 "dummy", "enetpll_phi0",};

static struct clk *clk[S32V234_CLK_END];
static struct clk_onecell_data clk_data;

static void __init s32v234_clocks_init(struct device_node *mc_cgm0_node)
{
	struct device_node *np;

	clk[S32V234_CLK_DUMMY] = s32_clk_fixed("dummy", 0);
	clk[S32V234_CLK_FXOSC] = s32_obtain_fixed_clock("fxosc", 0);
	clk[S32V234_CLK_FIRC] = s32_obtain_fixed_clock("firc", 0);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_me");
	mc_me_base = of_iomap(np, 0);
	if (WARN_ON(!mc_me_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-src");
	src_base = of_iomap(np, 0);
	if (WARN_ON(!src_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm1");
	mc_cgm1_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm1_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm2");
	mc_cgm2_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm2_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm3");
	mc_cgm3_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm3_base))
		return;

	np = mc_cgm0_node;
	mc_cgm0_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm0_base))
		return;

	enable_cpumodes_onperipheralconfig(mc_me_base, MC_ME_RUN_PCn_DRUN |
					    MC_ME_RUN_PCn_RUN0 |
					    MC_ME_RUN_PCn_RUN1 |
					    MC_ME_RUN_PCn_RUN2 |
					    MC_ME_RUN_PCn_RUN3,
					    0);

	/* turn on XOSC and FIRC */
	enable_clocks_sources(MC_ME_MODE_MC_MVRON, MC_ME_MODE_MC_XOSCON |
			      MC_ME_MODE_MC_FIRCON,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* transition the core to RUN0 mode */
	entry_to_target_mode(mc_me_base, MC_ME_MCTL_RUN0);

	clk[S32V234_CLK_ARMPLL_SRC_SEL] = s32_clk_mux("armpll_sel",
		SRC_GPR1, SRC_GPR1_ARMPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), &s32v234_lock);

	clk[S32V234_CLK_PERIPHPLL_SRC_SEL] = s32_clk_mux("periphpll_sel",
		SRC_GPR1, SRC_GPR1_PERIPHPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), &s32v234_lock);

	clk[S32V234_CLK_ENETPLL_SRC_SEL] = s32_clk_mux("enetpll_sel",
		SRC_GPR1, SRC_GPR1_ENETPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), &s32v234_lock);

	/* ARM_PLL */
	clk[S32V234_CLK_ARMPLL_VCO] = s32v234_clk_plldig(S32_PLLDIG_ARM,
		"armpll_vco", "armpll_sel", ARMPLL_PLLDIG(mc_cgm0_base),
		ARMPLL_PLLDIG_PLLDV_MFD, ARMPLL_PLLDIG_PLLDV_MFN,
		ARMPLL_PLLDIG_PLLDV_RFDPHI0, ARMPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_ARMPLL_PHI0] = s32v234_clk_plldig_phi(S32_PLLDIG_ARM,
		"armpll_phi0", "armpll_vco",
		ARMPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_ARMPLL_PHI1] = s32v234_clk_plldig_phi(S32_PLLDIG_ARM,
		"armpll_phi1", "armpll_vco",
		ARMPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_ARMPLL_DFS0] = s32v234_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs0", "armpll_phi1",
		 ARMPLL_PLLDIG_DFS(mc_cgm0_base), 0,
		 ARMPLL_PLLDIG_DFS0_MFN);

	clk[S32V234_CLK_ARMPLL_DFS1] = s32v234_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs1", "armpll_phi1",
		 ARMPLL_PLLDIG_DFS(mc_cgm0_base), 1,
		 ARMPLL_PLLDIG_DFS1_MFN);

	clk[S32V234_CLK_ARMPLL_DFS2] = s32v234_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs2", "armpll_phi1",
		 ARMPLL_PLLDIG_DFS(mc_cgm0_base), 2,
		 ARMPLL_PLLDIG_DFS2_MFN);

	clk[S32V234_CLK_SYS_SEL] = s32_clk_mux("sys_sel",
		MC_ME_RUNn_MC(mc_me_base, 0),
		MC_ME_MODE_MC_SYSCLK_OFFSET,
		MC_ME_MODE_MC_SYSCLK_SIZE,
		sys_sels, ARRAY_SIZE(sys_sels), &s32v234_lock);

	clk[S32V234_CLK_SYS3] = s32_clk_divider("sys3", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 0), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE, &s32v234_lock);

	clk[S32V234_CLK_SYS6] = s32_clk_divider("sys6", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 1), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE, &s32v234_lock);

	clk[S32V234_CLK_SYS6_DIV2] = s32_clk_divider("sys6_div2", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 2), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE, &s32v234_lock);

	/* PERIPH_PLL */
	clk[S32V234_CLK_PERIPHPLL_VCO] = s32v234_clk_plldig(S32_PLLDIG_PERIPH,
		"periphpll_vco", "periphpll_sel",
		PERIPHPLL_PLLDIG(mc_cgm0_base),
		PERIPHPLL_PLLDIG_PLLDV_MFD, PERIPHPLL_PLLDIG_PLLDV_MFN,
		PERIPHPLL_PLLDIG_PLLDV_RFDPHI0,
		PERIPHPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_PERIPHPLL_PHI0] =
		s32v234_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi0", "periphpll_vco",
		PERIPHPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_PERIPHPLL_PHI1] =
		s32v234_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi1", "periphpll_vco",
		PERIPHPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV3] = s32_clk_fixed_factor(
		"periphpll_phi0_div3", "periphpll_phi0", 1, 3);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV5] = s32_clk_fixed_factor(
		"periphpll_phi0_div5", "periphpll_phi0", 1, 5);

	clk[S32V234_CLK_CAN_SEL] = s32_clk_mux("can_sel",
		CGM_ACn_SC(mc_cgm0_base, 6),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		can_sels, ARRAY_SIZE(can_sels), &s32v234_lock);

	/* CAN Clock */
	clk[S32V234_CLK_CAN] = s32_clk_divider("can", "can_sel",
		CGM_ACn_DCm(mc_cgm0_base, 6, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE, &s32v234_lock);

	/* Lin Clock */
	clk[S32V234_CLK_LIN_SEL] = s32_clk_mux("lin_sel",
		CGM_ACn_SC(mc_cgm0_base, 3),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		lin_sels, ARRAY_SIZE(lin_sels), &s32v234_lock);

	clk[S32V234_CLK_LIN] = s32_clk_divider("lin", "lin_sel",
		CGM_ACn_DCm(mc_cgm0_base, 3, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE, &s32v234_lock);

	clk[S32V234_CLK_LIN_IPG] = s32_clk_fixed_factor("lin_ipg",
		"lin", 1, 2);

	/* enable PERIPHPLL */
	enable_clocks_sources(0, MC_ME_MODE_MC_PERIPHPLL,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* ENET_PLL */
	clk[S32V234_CLK_ENETPLL_VCO] = s32v234_clk_plldig(S32_PLLDIG_ENET,
		"enetpll_vco", "enetpll_sel", ENETPLL_PLLDIG(mc_cgm0_base),
		ENETPLL_PLLDIG_PLLDV_MFD, ENETPLL_PLLDIG_PLLDV_MFN,
		ENETPLL_PLLDIG_PLLDV_RFDPHI0, ENETPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_ENETPLL_PHI0] = s32v234_clk_plldig_phi(S32_PLLDIG_ENET,
		"enetpll_phi0", "enetpll_vco",
		ENETPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_ENETPLL_PHI1] = s32v234_clk_plldig_phi(S32_PLLDIG_ENET,
		"enetpll_phi1", "enetpll_vco",
		ENETPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_ENETPLL_DFS0] = s32v234_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs0", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 0,
		 ENETPLL_PLLDIG_DFS0_MFN);

	clk[S32V234_CLK_ENETPLL_DFS1] = s32v234_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs1", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 1,
		 ENETPLL_PLLDIG_DFS1_MFN);

	clk[S32V234_CLK_ENETPLL_DFS2] = s32v234_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs2", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 2,
		 ENETPLL_PLLDIG_DFS2_MFN);

	clk[S32V234_CLK_ENETPLL_DFS3] = s32v234_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs3", "enetpll_phi1",
		 ENETPLL_PLLDIG_DFS(mc_cgm0_base), 3,
		 ENETPLL_PLLDIG_DFS3_MFN);

	/* ENET Clock */
	clk[S32V234_CLK_ENET_SEL] = s32_clk_mux("enet_sel",
		CGM_ACn_SC(mc_cgm2_base, 2),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		enet_sels, ARRAY_SIZE(enet_sels), &s32v234_lock);

	clk[S32V234_CLK_ENET_TIME_SEL] = s32_clk_mux("enet_time_sel",
		CGM_ACn_SC(mc_cgm0_base, 7),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		enet_time_sels, ARRAY_SIZE(enet_time_sels), &s32v234_lock);

	clk[S32V234_CLK_ENET] = s32_clk_divider("enet", "enet_sel",
		CGM_ACn_DCm(mc_cgm2_base, 2, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE, &s32v234_lock);

	clk[S32V234_CLK_ENET_TIME] = s32_clk_divider("enet_time",
		"enet_time_sel",
		CGM_ACn_DCm(mc_cgm0_base, 7, 1),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE, &s32v234_lock);

	/* SDHC Clock */
	clk[S32V234_CLK_SDHC_SEL] = s32_clk_mux("sdhc_sel",
		CGM_ACn_SC(mc_cgm0_base, 15),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		sdhc_sels, ARRAY_SIZE(sdhc_sels), &s32v234_lock);

	clk[S32V234_CLK_SDHC] = s32_clk_divider("sdhc", "sdhc_sel",
		CGM_ACn_DCm(mc_cgm0_base, 15, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE, &s32v234_lock);

	/* set the system clock */
	enable_sysclock(MC_ME_MODE_MC_SYSCLK(0x2),
			MC_ME_RUNn_MC(mc_me_base, 0));

	/* transition the core to RUN0 mode */
	entry_to_target_mode(mc_me_base, MC_ME_MCTL_RUN0);

	/* Add the clocks to provider list */
	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(S32V234, "fsl,s32v234-mc_cgm0", s32v234_clocks_init);
