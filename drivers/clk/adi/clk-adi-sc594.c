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

static struct clk *clks[ADSP_SC594_CLK_END];
static struct clk_onecell_data clk_data;

static const char * const cgu0_s1sels[] = {"cgu0_s1seldiv", "cgu0_s1selexdiv"};
static const char * const cgu1_s1sels[] = {"cgu1_s1seldiv", "cgu1_s1selexdiv"};

static void sc594_clock_probe(struct device_node *np)
{
	void __iomem *cgu0;
	void __iomem *cgu1;
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

	// Input clock configuration
	clks[ADSP_SC594_CLK_SYS_CLKIN0] = of_clk_get_by_name(np, "sys_clkin0");
	clks[ADSP_SC594_CLK_SYS_CLKIN1] = of_clk_get_by_name(np, "sys_clkin1");

	// CGU configuration and internal clocks
	clks[ADSP_SC594_CLK_CGU0_PLL_IN] = clk_register_divider(NULL, "cgu0_df",
		"sys_clkin0", CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 0, 1, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_PLL_IN] = clk_register_divider(NULL, "cgu1_df",
		"cdu_clkinsel", CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 0, 1, 0, &cdu_lock);

	// VCO output inside PLL
	clks[ADSP_SC594_CLK_CGU0_VCO_OUT] = sc5xx_cgu_pll("cgu0_vco", "cgu0_df",
		cgu0 + CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, false, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_VCO_OUT] = sc5xx_cgu_pll("cgu1_vco", "cgu1_df",
		cgu1 + CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, false, &cdu_lock);

	// Final PLL output
	clks[ADSP_SC594_CLK_CGU0_PLLCLK] = clk_register_fixed_factor(NULL,
		"cgu0_pllclk", "cgu0_vco", CLK_SET_RATE_PARENT, 1, 1);
	clks[ADSP_SC594_CLK_CGU1_PLLCLK] = clk_register_fixed_factor(NULL,
		"cgu1_pllclk", "cgu1_vco", CLK_SET_RATE_PARENT, 1, 1);

	// Dividers from pll output
	clks[ADSP_SC594_CLK_CGU0_CDIV] = cgu_divider("cgu0_cdiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 0, 5, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_SYSCLK] = cgu_divider("sysclk_0", "cgu0_pllclk",
		cgu0 + CGU_DIV, 8, 5, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_DDIV] = cgu_divider("cgu0_ddiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 16, 5, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_ODIV] = cgu_divider("cgu0_odiv", "cgu0_pllclk",
		cgu0 + CGU_DIV, 22, 7, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_S0SELDIV] = cgu_divider("cgu0_s0seldiv",
		"sysclk_0", cgu0 + CGU_DIV, 5, 3, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_S1SELDIV] = cgu_divider("cgu0_s1seldiv",
		"sysclk_0", cgu0 + CGU_DIV, 13, 3, 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_S1SELEXDIV] = cgu_divider("cgu0_s1selexdiv",
		"cgu0_pllclk", cgu0 + CGU_DIVEX, 16, 8, 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU0_S1SEL] = clk_register_mux(NULL, "cgu0_sclk1sel",
		cgu0_s1sels, 2, CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 17, 1, 0, &cdu_lock);

	clks[ADSP_SC594_CLK_CGU1_CDIV] = cgu_divider("cgu1_cdiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 0, 5, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_SYSCLK] = cgu_divider("sysclk_1", "cgu1_pllclk",
		cgu1 + CGU_DIV, 8, 5, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_DDIV] = cgu_divider("cgu1_ddiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 16, 5, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_ODIV] = cgu_divider("cgu1_odiv", "cgu1_pllclk",
		cgu1 + CGU_DIV, 22, 7, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_S0SELDIV] = cgu_divider("cgu1_s0seldiv",
		"sysclk_1", cgu1 + CGU_DIV, 5, 3, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_S1SELDIV] = cgu_divider("cgu1_s1seldiv",
		"sysclk_1", cgu1 + CGU_DIV, 13, 3, 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S1SELEXDIV] = cgu_divider("cgu1_s1selexdiv",
		"cgu1_pllclk", cgu1 + CGU_DIVEX, 16, 8, 0, &cdu_lock);
	clks[ADSP_SC598_CLK_CGU1_S1SEL] = clk_register_mux(NULL, "cgu1_sclk1sel",
		cgu1_s1sels, 2, CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 17, 1, 0, &cdu_lock);

	// Gates to enable CGU outputs
	clks[ADSP_SC594_CLK_CGU0_CCLK0] = cgu_gate("cclk0_0", "cgu0_cdiv",
		cgu0 + CGU_CCBF_DIS, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_CCLK1] = cgu_gate("cclk1_0", "cgu0_cdiv",
		cgu1 + CGU_CCBF_DIS, 1, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_OCLK] = cgu_gate("oclk_0", "cgu0_odiv",
		cgu0 + CGU_SCBF_DIS, 3, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_DCLK] = cgu_gate("dclk_0", "cgu0_ddiv",
		cgu0 + CGU_SCBF_DIS, 2, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_SCLK1] = cgu_gate("sclk1_0", "cgu0_sclk1sel",
		cgu0 + CGU_SCBF_DIS, 1, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU0_SCLK0] = cgu_gate("sclk0_0", "cgu0_s0seldiv",
		cgu0 + CGU_SCBF_DIS, 0, &cdu_lock);

	clks[ADSP_SC594_CLK_CGU1_CCLK0] = cgu_gate("cclk0_1", "cgu1_cdiv",
		cgu1 + CGU_CCBF_DIS, 0, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_CCLK1] = cgu_gate("cclk1_1", "cgu1_cdiv",
		cgu1 + CGU_CCBF_DIS, 1, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_OCLK] = cgu_gate("oclk_1", "cgu1_odiv",
		cgu1 + CGU_SCBF_DIS, 3, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_DCLK] = cgu_gate("dclk_1", "cgu1_ddiv",
		cgu1 + CGU_SCBF_DIS, 2, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_SCLK1] = cgu_gate("sclk1_1", "cgu1_sclk1sel",
		cgu1 + CGU_SCBF_DIS, 1, &cdu_lock);
	clks[ADSP_SC594_CLK_CGU1_SCLK0] = cgu_gate("sclk0_1", "cgu1_s0seldiv",
		cgu1 + CGU_SCBF_DIS, 0, &cdu_lock);

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

CLK_OF_DECLARE(sc594_clocks, "adi,sc594-clocks", sc594_clock_probe);
