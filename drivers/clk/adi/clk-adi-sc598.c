// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Clock support for ADI processor
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Author: Greg Malysa <greg.malysa@timesys.com>
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 */

#include <dt-bindings/clock/adi-sc5xx-clock.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>
#include <linux/spinlock.h>

#include "clk.h"

static DEFINE_SPINLOCK(cdu_lock);

static struct clk *clks[ADSP_SC598_CLK_END];
static struct clk_onecell_data clk_data;

static const char *const cgu1_in_sels[] = { "sys_clkin0", "sys_clkin1" };
static const char *const cgu0_s1sels[] = { "cgu0_s1seldiv", "cgu0_s1selexdiv" };
static const char *const cgu1_s0sels[] = { "cgu1_s0seldiv", "cgu1_s0selexdiv" };
static const char *const cgu1_s1sels[] = { "cgu1_s1seldiv", "cgu1_s1selexdiv" };

static const char *const sharc0_sels[] = { "cclk0_0" };
static const u32 sharc0_parent_sel[] = { 0 };

static const char *const sharc1_sels[] = { "cclk0_0" };
static const u32 sharc1_parent_sel[] = { 0 };

static const char *const arm_sels[] = { "cclk2_0", "cclk2_1" };
static const u32 arm_parent_sel[] = { 2, 3 };

static const char *const cdu_ddr_sels[] = { "dclk_0", "dclk_1" };
static const u32 cdu_ddr_parent_sel[] = { 0, 1 };

static const char *const can_sels[] = { "oclk_1" };
static const u32 can_parent_sel[] = { 1 };

static const char *const spdif_sels[] = { "sclk1_0" };
static const u32 spdif_parent_sel[] = { 0 };

static const char *const spi_sels[] = { "sclk0_0", "oclk_0" };
static const u32 spi_parent_sel[] = { 0, 1 };

static const char *const gige_sels[] = { "sclk0_0", "sclk0_1", "oclk_0" };
static const u32 gige_parent_sel[] = { 0, 1, 2 };

static const char *const lp_sels[] = { "oclk_0", "sclk0_0", "cclk0_1" };
static const u32 lp_parent_sel[] = { 0, 1, 2 };

static const char *const lp_ddr_sels[] = { "oclk_0", "dclk_0", "sysclk_1" };
static const u32 lp_ddr_parent_sel[] = { 0, 1, 2 };

static const char *const ospi_refclk_sels[] = {
	"sysclk_0", "sclk0_0", "sclk1_1"
};
static const u32 ospi_refclk_parent_sel[] = { 0, 1, 2 };

static const char *const trace_sels[] = { "sclk0_0" };
static const u32 trace_parent_sel[] = { 0 };

static const char *const emmc_sels[] = {
	"oclk_0", "sclk0_1", "dclk_0_half", "dclk_1_half"
};
static const u32 emmc_parent_sel[] = { 0, 1, 2, 3 };

static const char *const emmc_timer_sels[] = { "sclk1_1_half" };
static const u32 emmc_timer_parent_sel[] = { 1 };

static const char *const ddr_sels[] = { "cdu_ddr", "3pll_ddiv" };

static void sc598_clock_probe(struct device_node *np)
{
	void __iomem *cgu0;
	void __iomem *cgu1;
	void __iomem *cdu;
	void __iomem *pll3;
	int ret;
	int i;

	cgu0 = of_iomap(np, 0);
	if (!cgu0) {
		pr_err("Unable to remap CGU0 address (resource 0)\n");
		return;
	}

	cgu1 = of_iomap(np, 1);
	if (!cgu1) {
		pr_err("Unable to remap CGU1 address (resource 1)\n");
		return;
	}

	cdu = of_iomap(np, 2);
	if (!cdu) {
		pr_err("Unable to remap CDU address (resource 2)\n");
		return;
	}

	pll3 = of_iomap(np, 3);
	if (!pll3) {
		pr_err
		    ("Unable to remap PLL3 control register (resource 3)\n");
		return;
	}
	// We only access this one register for pll3
	pll3 = pll3 + PLL3_OFFSET;

	// Input clock configuration
	clks[ADSP_SC598_CLK_DUMMY] =
	    clk_register_fixed_rate(NULL, "dummy", NULL, 0, 0);
	clks[ADSP_SC598_CLK_SYS_CLKIN0] =
	    of_clk_get_by_name(np, "sys_clkin0");
	clks[ADSP_SC598_CLK_SYS_CLKIN1] =
	    of_clk_get_by_name(np, "sys_clkin1");
	clks[ADSP_SC598_CLK_CGU1_IN] =
	    clk_register_mux(NULL, "cgu1_in_sel", cgu1_in_sels, 2,
			     CLK_SET_RATE_PARENT, cdu + CDU_CLKINSEL, 0, 1,
			     0, &cdu_lock);

	// 3rd pll reuses cgu1 clk in selection, feeds directly into 3pll df
	// changing the cgu1 in sel mux will affect 3pll so reuse the same clocks

	// CGU configuration and internal clocks
	clks[ADSP_SC598_CLK_CGU0_PLL_IN] =
	    clk_register_divider(NULL, "cgu0_df", "sys_clkin0",
				 CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 0, 1,
				 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_PLL_IN] =
	    clk_register_divider(NULL, "cgu1_df", "cgu1_in_sel",
				 CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 0, 1,
				 0, &cdu_lock);
	clks[ADSP_SC598_CLK_3PLL_PLL_IN] =
	    clk_register_divider(NULL, "3pll_df", "cgu1_in_sel",
				 CLK_SET_RATE_PARENT, pll3, 3, 1, 0,
				 &cdu_lock);

	// VCO output inside PLL
	clks[ADSP_SC598_CLK_CGU0_VCO_OUT] =
	    sc5xx_cgu_pll("cgu0_vco", "cgu0_df", cgu0 + CGU_CTL,
			  CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, true,
			  &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_VCO_OUT] =
	    sc5xx_cgu_pll("cgu1_vco", "cgu1_df", cgu1 + CGU_CTL,
			  CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, true,
			  &cdu_lock);
	clks[ADSP_SC598_CLK_3PLL_VCO_OUT] =
	    sc5xx_cgu_pll("3pll_vco", "3pll_df", pll3, PLL3_MSEL_SHIFT,
			  PLL3_MSEL_WIDTH, 1, true, &cdu_lock);

	// Final PLL output
	clks[ADSP_SC598_CLK_CGU0_PLLCLK] = clk_register_fixed_factor(NULL,
								     "cgu0_pllclk",
								     "cgu0_vco",
								     CLK_SET_RATE_PARENT,
								     1, 2);
	clks[ADSP_SC598_CLK_CGU1_PLLCLK] =
	    clk_register_fixed_factor(NULL, "cgu1_pllclk", "cgu1_vco",
				      CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC598_CLK_3PLL_PLLCLK] =
	    clk_register_fixed_factor(NULL, "3pll_pllclk", "3pll_vco",
				      CLK_SET_RATE_PARENT, 1, 2);

	// Dividers from pll output
	clks[ADSP_SC598_CLK_CGU0_CDIV] =
	    cgu_divider("cgu0_cdiv", "cgu0_pllclk", cgu0 + CGU_DIV, 0, 5,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_SYSCLK] =
	    cgu_divider("sysclk_0", "cgu0_pllclk", cgu0 + CGU_DIV, 8, 5, 0,
			&cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_DDIV] =
	    cgu_divider("cgu0_ddiv", "cgu0_pllclk", cgu0 + CGU_DIV, 16, 5,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_ODIV] =
	    cgu_divider("cgu0_odiv", "cgu0_pllclk", cgu0 + CGU_DIV, 22, 7,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_S0SELDIV] =
	    cgu_divider("cgu0_s0seldiv", "sysclk_0", cgu0 + CGU_DIV, 5, 3,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_S1SELDIV] =
	    cgu_divider("cgu0_s1seldiv", "sysclk_0", cgu0 + CGU_DIV, 13, 3,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_S1SELEXDIV] =
	    cgu_divider("cgu0_s1selexdiv", "cgu0_pllclk", cgu0 + CGU_DIVEX,
			16, 8, 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_S1SEL] =
	    clk_register_mux(NULL, "cgu0_sclk1sel", cgu0_s1sels, 2,
			     CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 17, 1, 0,
			     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_CCLK2] =
	    clk_register_fixed_factor(NULL, "cclk2_0", "cgu0_vco",
				      CLK_SET_RATE_PARENT, 1, 3);

	clks[ADSP_SC598_CLK_CGU1_CDIV] =
	    cgu_divider("cgu1_cdiv", "cgu1_pllclk", cgu1 + CGU_DIV, 0, 5,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_SYSCLK] =
	    cgu_divider("sysclk_1", "cgu1_pllclk", cgu1 + CGU_DIV, 8, 5, 0,
			&cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_DDIV] =
	    cgu_divider("cgu1_ddiv", "cgu1_pllclk", cgu1 + CGU_DIV, 16, 5,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_ODIV] =
	    cgu_divider("cgu1_odiv", "cgu1_pllclk", cgu1 + CGU_DIV, 22, 7,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S0SELDIV] =
	    cgu_divider("cgu1_s0seldiv", "sysclk_1", cgu1 + CGU_DIV, 5, 3,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S1SELDIV] =
	    cgu_divider("cgu1_s1seldiv", "sysclk_1", cgu1 + CGU_DIV, 13, 3,
			0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S0SELEXDIV] =
	    cgu_divider("cgu1_s0selexdiv", "cgu1_pllclk", cgu1 + CGU_DIVEX,
			0, 8, 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S1SELEXDIV] =
	    cgu_divider("cgu1_s1selexdiv", "cgu1_pllclk", cgu1 + CGU_DIVEX,
			16, 8, 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S0SEL] =
	    clk_register_mux(NULL, "cgu1_sclk0sel", cgu1_s0sels, 2,
			     CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 16, 1, 0,
			     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S1SEL] =
	    clk_register_mux(NULL, "cgu1_sclk1sel", cgu1_s1sels, 2,
			     CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 17, 1, 0,
			     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_CCLK2] =
	    clk_register_fixed_factor(NULL, "cclk2_1", "cgu1_vco",
				      CLK_SET_RATE_PARENT, 1, 3);

	clks[ADSP_SC598_CLK_3PLL_DDIV] =
	    clk_register_divider(NULL, "3pll_ddiv", "3pll_pllclk",
				 CLK_SET_RATE_PARENT, pll3, 12, 5, 0,
				 &cdu_lock);

	// Gates to enable CGU outputs
	clks[ADSP_SC598_CLK_CGU0_CCLK0] = cgu_gate("cclk0_0", "cgu0_cdiv",
						   cgu0 + CGU_CCBF_DIS, 0,
						   &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_OCLK] =
	    cgu_gate("oclk_0", "cgu0_odiv", cgu0 + CGU_SCBF_DIS, 3,
		     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_DCLK] =
	    cgu_gate("dclk_0", "cgu0_ddiv", cgu0 + CGU_SCBF_DIS, 2,
		     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_SCLK1] =
	    cgu_gate("sclk1_0", "cgu0_sclk1sel", cgu0 + CGU_SCBF_DIS, 1,
		     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_SCLK0] =
	    cgu_gate("sclk0_0", "cgu0_s0seldiv", cgu0 + CGU_SCBF_DIS, 0,
		     &cdu_lock);

	clks[ADSP_SC598_CLK_CGU1_CCLK0] = cgu_gate("cclk0_1", "cgu1_cdiv",
						   cgu1 + CGU_CCBF_DIS, 0,
						   &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_OCLK] =
	    cgu_gate("oclk_1", "cgu1_odiv", cgu1 + CGU_SCBF_DIS, 3,
		     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_DCLK] =
	    cgu_gate("dclk_1", "cgu1_ddiv", cgu1 + CGU_SCBF_DIS, 2,
		     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_SCLK1] =
	    cgu_gate("sclk1_1", "cgu1_sclk1sel", cgu1 + CGU_SCBF_DIS, 1,
		     &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_SCLK0] =
	    cgu_gate("sclk0_1", "cgu1_sclk0sel", cgu1 + CGU_SCBF_DIS, 0,
		     &cdu_lock);

	// Extra half rate clocks generated in the CDU
	clks[ADSP_SC598_CLK_DCLK0_HALF] =
	    clk_register_fixed_factor(NULL, "dclk_0_half", "dclk_0",
				      CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC598_CLK_DCLK1_HALF] =
	    clk_register_fixed_factor(NULL, "dclk_1_half", "dclk_1",
				      CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC598_CLK_CGU1_SCLK1_HALF] =
	    clk_register_fixed_factor(NULL, "sclk1_1_half", "sclk1_1",
				      CLK_SET_RATE_PARENT, 1, 2);
	
	/* CDU output clocks: CDU_CFG[n] mux + gate in one clock */
	clks[ADSP_SC598_CLK_SHARC0] =
		sc5xx_cdu_register("sharc0", cdu, 0, sharc0_sels,
				   sharc0_parent_sel, ARRAY_SIZE(sharc0_sels),
				   CLK_IS_CRITICAL, &cdu_lock);
	clks[ADSP_SC598_CLK_SHARC1] =
		sc5xx_cdu_register("sharc1", cdu, 1, sharc1_sels,
				   sharc1_parent_sel, ARRAY_SIZE(sharc1_sels),
				   CLK_IS_CRITICAL, &cdu_lock);
	clks[ADSP_SC598_CLK_ARM] =
		sc5xx_cdu_register("arm", cdu, 2, arm_sels,
				   arm_parent_sel, ARRAY_SIZE(arm_sels),
				   CLK_IS_CRITICAL, &cdu_lock);
	clks[ADSP_SC598_CLK_CDU_DDR] =
		sc5xx_cdu_register("cdu_ddr", cdu, 3, cdu_ddr_sels,
				   cdu_ddr_parent_sel, ARRAY_SIZE(cdu_ddr_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_CAN] =
		sc5xx_cdu_register("can", cdu, 4, can_sels,
				   can_parent_sel, ARRAY_SIZE(can_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_SPDIF] =
		sc5xx_cdu_register("spdif", cdu, 5, spdif_sels,
				   spdif_parent_sel, ARRAY_SIZE(spdif_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_SPI] =
		sc5xx_cdu_register("spi", cdu, 6, spi_sels,
				   spi_parent_sel, ARRAY_SIZE(spi_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_GIGE] =
		sc5xx_cdu_register("gige", cdu, 7, gige_sels,
				   gige_parent_sel, ARRAY_SIZE(gige_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_LP] =
		sc5xx_cdu_register("lp", cdu, 8, lp_sels,
				   lp_parent_sel, ARRAY_SIZE(lp_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_LP_DDR] =
		sc5xx_cdu_register("lp_ddr", cdu, 9, lp_ddr_sels,
				   lp_ddr_parent_sel, ARRAY_SIZE(lp_ddr_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_OSPI_REFCLK] =
		sc5xx_cdu_register("ospi_refclk", cdu, 10, ospi_refclk_sels,
				   ospi_refclk_parent_sel,
				   ARRAY_SIZE(ospi_refclk_sels),
				   0, &cdu_lock);
	/* CLKO11 is internal and not user configurable. */
	clks[ADSP_SC598_CLK_TRACE] =
		sc5xx_cdu_register("trace", cdu, 12, trace_sels,
				   trace_parent_sel, ARRAY_SIZE(trace_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_EMMC] =
		sc5xx_cdu_register("emmc", cdu, 13, emmc_sels,
				   emmc_parent_sel, ARRAY_SIZE(emmc_sels),
				   0, &cdu_lock);
	clks[ADSP_SC598_CLK_EMMC_TIMER_QMC] =
		sc5xx_cdu_register("emmc_timer_qmc", cdu, 14, emmc_timer_sels,
				   emmc_timer_parent_sel,
				   ARRAY_SIZE(emmc_timer_sels),
				   0, &cdu_lock);


	// Dedicated DDR output mux
	clks[ADSP_SC598_CLK_DDR] =
	    clk_register_mux(NULL, "ddr", ddr_sels, 2,
			     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, pll3,
			     11, 1, 0, &cdu_lock);

	ret = cdu_check_clocks(clks, ARRAY_SIZE(clks));
	if (ret)
		goto cleanup;

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
	if (ret < 0) {
		pr_err("Failed to register SoC clock information\n");
		goto cleanup;
	}

	return;

cleanup:
	for (i = 0; i < ARRAY_SIZE(clks); i++)
		clk_unregister(clks[i]);
}

CLK_OF_DECLARE(sc598_clocks, "adi,sc598-clocks", sc598_clock_probe);
