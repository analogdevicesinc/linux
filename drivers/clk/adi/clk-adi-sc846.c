// SPDX-License-Identifier: GPL-2.0
/*
 * Clock support for ADI SC846 processor
 *
 * (C) Copyright 2026 - Analog Devices, Inc.
 *
 * Contact: Analog Devices Inc. <linux@analog.com>
 *
 */

#include <dt-bindings/clock/adi-sc5xx-clock.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "clk.h"

static DEFINE_SPINLOCK(cdu_lock);

static struct clk *clks[ADSP_SC846_CLK_END];
static struct clk_onecell_data clk_data;

static const char *const cgu0_s1sels[] = { "cgu0_s1seldiv", "cgu0_s1selexdiv" };
static const char *const cgu1_s0sels[] = { "cgu1_s0seldiv", "cgu1_s0selexdiv" };
static const char *const cgu1_s1sels[] = { "cgu1_s1seldiv", "cgu1_s1selexdiv" };
static const char *const sharc0_sels[] = { "cclk0_0", "cclk0_1", "cclk2_0", "dummy" };
static const char *const arm_sels[] = { "cclk0_0", "cclk0_1", "cclk2_0", "dummy" };
static const char *const ddr_sels[] = { "dclk_0", "dclk_1", "dclk1_0", "dummy" };
static const char *const can_sels[] = { "oclk_0", "oclk_1", "dummy", "dummy" };
static const char *const spdif_sels[] = { "sclk1_0", "dummy", "dummy", "dummy" };
static const char *const spi_sels[] = { "sclk0_0", "oclk_0", "dummy", "dummy" };
static const char *const gige_sels[] = { "sclk0_0", "sclk0_1", "dummy", "dummy" };
static const char *const xspi1_sels[] = { "sclk0_0", "oclk_0", "sclk1_1", "dummy" };
static const char *const lp_sels[] = { "dclk_0", "cclk2_0", "dclk_1", "cclk2_1" };
static const char *const xspi0_sels[] = { "sclk0_0", "oclk_0", "sclk1_1", "dummy" };
static const char *const trace_sels[] = { "sclk0_0", "dummy", "dummy", "dummy" };
static const char *const pwm_sels[] = { "sysclk_0", "clkpwm", "dummy", "dummy" };
static const char *const mshc_sels[] = { "sclk0_0", "sysclk_1", "dummy", "dummy" };

static void sc846_clock_probe(struct device_node *np)
{
	void __iomem *cgu0;
	void __iomem *cgu1;
	void __iomem *cdu;
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

	/* Input clock configuration */
	clks[ADSP_SC846_CLK_DUMMY] = clk_register_fixed_rate(NULL, "dummy", NULL, 0, 0);
	clks[ADSP_SC846_CLK_SYS_CLKIN0] = of_clk_get_by_name(np, "sys_clkin0");

	/* CGU configuration and internal clocks */
	clks[ADSP_SC846_CLK_CGU0_PLL_IN] =
	    clk_register_divider(NULL, "cgu0_df", "sys_clkin0",
				 CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 0, 1,
				 0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_PLL_IN] =
	    clk_register_divider(NULL, "cgu1_df", "sys_clkin0",
				 CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 0, 1,
				 0, &cdu_lock);

	/* VCO output inside PLL */
	clks[ADSP_SC846_CLK_CGU0_VCO_OUT] =
	    sc5xx_cgu_pll("cgu0_vco", "cgu0_df", cgu0 + CGU_CTL,
			  CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, true,
			  &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_VCO_OUT] =
	    sc5xx_cgu_pll("cgu1_vco", "cgu1_df", cgu1 + CGU_CTL,
			  CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, true,
			  &cdu_lock);

	/* Final PLL output */
	clks[ADSP_SC846_CLK_CGU0_PLLCLK] =
		clk_register_fixed_factor(NULL, "cgu0_pllclk", "cgu0_vco",
					  CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC846_CLK_CGU1_PLLCLK] =
		clk_register_fixed_factor(NULL, "cgu1_pllclk", "cgu1_vco",
					  CLK_SET_RATE_PARENT, 1, 2);
	/* Fixed factor clocks */
	clks[ADSP_SC846_CLK_CGU0_PLLCLK_HALF] =
		clk_register_fixed_factor(NULL, "cgu0_pllclk_half", "cgu0_pllclk",
					  CLK_SET_RATE_PARENT, 1, 2);
	clks[ADSP_SC846_CLK_CGU1_PLLCLK_HALF] =
		clk_register_fixed_factor(NULL, "cgu1_pllclk_half", "cgu1_pllclk",
					  CLK_SET_RATE_PARENT, 1, 2);
	/* Dividers from pll output */
	clks[ADSP_SC846_CLK_CGU0_CDIV] =
	    cgu_divider("cgu0_cdiv", "cgu0_pllclk_half", cgu0 + CGU_DIV, 0, 5,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_SYSCLK] =
	    cgu_divider("sysclk_0", "cgu0_pllclk_half", cgu0 + CGU_DIV, 8, 5, 0,
			&cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_DDIV] =
	    cgu_divider("cgu0_ddiv", "cgu0_pllclk_half", cgu0 + CGU_DIV, 16, 5,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_ODIV] =
	    cgu_divider("cgu0_odiv", "cgu0_pllclk_half", cgu0 + CGU_DIV, 22, 7,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_S0SELDIV] =
	    cgu_divider("cgu0_s0seldiv", "sysclk_0", cgu0 + CGU_DIV, 5, 3,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_S1SELDIV] =
	    cgu_divider("cgu0_s1seldiv", "sysclk_0", cgu0 + CGU_DIV, 13, 3,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_S1SELEXDIV] =
	    cgu_divider("cgu0_s1selexdiv", "cgu0_pllclk_half", cgu0 + CGU_DIVEX,
			16, 8, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_S1SEL] =
	    clk_register_mux(NULL, "cgu0_sclk1sel", cgu0_s1sels, 2,
			     CLK_SET_RATE_PARENT, cgu0 + CGU_CTL, 17, 1, 0,
			     &cdu_lock);

	/* CCLK2_0 and DCLK1_0 */
	clks[ADSP_SC846_CLK_CGU0_CCLK2_0] =
	    clk_register_fixed_factor(NULL, "cclk2_0", "cgu0_pllclk_half",
				      CLK_SET_RATE_PARENT, 1, 3);
	clks[ADSP_SC846_CLK_CGU0_DCLK1_0] =
	    clk_register_fixed_factor(NULL, "dclk1_0", "cgu0_pllclk_half",
				      CLK_SET_RATE_PARENT, 1, 5);

	clks[ADSP_SC846_CLK_CGU1_CDIV] =
	    cgu_divider("cgu1_cdiv", "cgu1_pllclk_half", cgu1 + CGU_DIV, 0, 5,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_SYSCLK] =
	    cgu_divider("sysclk_1", "cgu1_pllclk_half", cgu1 + CGU_DIV, 8, 5, 0,
			&cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_DDIV] =
	    cgu_divider("cgu1_ddiv", "cgu1_pllclk_half", cgu1 + CGU_DIV, 16, 5,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_ODIV] =
	    cgu_divider("cgu1_odiv", "cgu1_pllclk_half", cgu1 + CGU_DIV, 22, 7,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_S0SELDIV] =
	    cgu_divider("cgu1_s0seldiv", "sysclk_1", cgu1 + CGU_DIV, 5, 3,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_S1SELDIV] =
	    cgu_divider("cgu1_s1seldiv", "sysclk_1", cgu1 + CGU_DIV, 13, 3,
			0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_S0SELEXDIV] =
	    cgu_divider("cgu1_s0selexdiv", "cgu1_pllclk_half", cgu1 + CGU_DIVEX,
			0, 8, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_S1SELEXDIV] =
	    cgu_divider("cgu1_s1selexdiv", "cgu1_pllclk_half", cgu1 + CGU_DIVEX,
			16, 8, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_S0SEL] =
	    clk_register_mux(NULL, "cgu1_sclk0sel", cgu1_s0sels, 2,
			     CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 16, 1, 0,
			     &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_S1SEL] =
	    clk_register_mux(NULL, "cgu1_sclk1sel", cgu1_s1sels, 2,
			     CLK_SET_RATE_PARENT, cgu1 + CGU_CTL, 17, 1, 0,
			     &cdu_lock);

	/* CCLK2_1 and DCLK1_1 */
	clks[ADSP_SC846_CLK_CGU1_CCLK2_1] =
	    clk_register_fixed_factor(NULL, "cclk2_1", "cgu1_pllclk_half",
				      CLK_SET_RATE_PARENT, 1, 3);
	clks[ADSP_SC846_CLK_CGU1_DCLK1_1] =
	    clk_register_fixed_factor(NULL, "dclk1_1", "cgu1_pllclk_half",
				      CLK_SET_RATE_PARENT, 1, 5);

	/* Gates to enable CGU outputs */
	clks[ADSP_SC846_CLK_CGU0_CCLK0] = cgu_gate("cclk0_0", "cgu0_cdiv",
						   cgu0 + CGU_CCBF_DIS, 0,
						   &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_OCLK] = cgu_gate("oclk_0", "cgu0_odiv",
						  cgu0 + CGU_SCBF_DIS, 3, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_DCLK] = cgu_gate("dclk_0", "cgu0_ddiv",
						  cgu0 + CGU_SCBF_DIS, 2, &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_SCLK1] =
	    cgu_gate("sclk1_0", "cgu0_sclk1sel", cgu0 + CGU_SCBF_DIS, 1,
		     &cdu_lock);
	clks[ADSP_SC846_CLK_CGU0_SCLK0] =
	    cgu_gate("sclk0_0", "cgu0_s0seldiv", cgu0 + CGU_SCBF_DIS, 0,
		     &cdu_lock);

	clks[ADSP_SC846_CLK_CGU1_CCLK0] = cgu_gate("cclk0_1", "cgu1_cdiv",
						   cgu1 + CGU_CCBF_DIS, 0,
						   &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_OCLK] =
	    cgu_gate("oclk_1", "cgu1_odiv", cgu1 + CGU_SCBF_DIS, 3,
		     &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_DCLK] =
	    cgu_gate("dclk_1", "cgu1_ddiv", cgu1 + CGU_SCBF_DIS, 2,
		     &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_SCLK1] =
	    cgu_gate("sclk1_1", "cgu1_sclk1sel", cgu1 + CGU_SCBF_DIS, 1,
		     &cdu_lock);
	clks[ADSP_SC846_CLK_CGU1_SCLK0] =
	    cgu_gate("sclk0_1", "cgu1_sclk0sel", cgu1 + CGU_SCBF_DIS, 0,
		     &cdu_lock);

	/* CDU output muxes */
	clks[ADSP_SC846_CLK_SHARC0_SEL] = cdu_mux("sharc0_sel", cdu + CDU_CFG0,
						  sharc0_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_ARM0_SEL] = cdu_mux("arm0_sel", cdu + CDU_CFG1,
						arm_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_ARM1_SEL] = cdu_mux("arm1_sel", cdu + CDU_CFG2,
						arm_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_CDU_DDR_SEL] = cdu_mux("ddr_sel", cdu + CDU_CFG3,
						   ddr_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_CAN_SEL] = cdu_mux("can_sel", cdu + CDU_CFG4,
					       can_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_SPDIF_SEL] = cdu_mux("spdif_sel", cdu + CDU_CFG5,
						 spdif_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_SPI_SEL] = cdu_mux("spi_sel", cdu + CDU_CFG6,
					       spi_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_GIGE_SEL] = cdu_mux("gige_sel", cdu + CDU_CFG7,
						gige_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_XSPI1_SEL] = cdu_mux("xspi1_sel", cdu + CDU_CFG8,
						 xspi1_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_LP_SEL] = cdu_mux("lp_sel", cdu + CDU_CFG9, lp_sels,
					      &cdu_lock);
	clks[ADSP_SC846_CLK_XSPI0_SEL] = cdu_mux("xspi0_sel", cdu + CDU_CFG10,
						 xspi0_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_TRACE_SEL] = cdu_mux("trace_sel", cdu + CDU_CFG12,
						 trace_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_PWM_SEL] = cdu_mux("pwm_sel", cdu + CDU_CFG13,
					       pwm_sels, &cdu_lock);
	clks[ADSP_SC846_CLK_MSHC_SEL] = cdu_mux("mshc_sel", cdu + CDU_CFG14,
						mshc_sels, &cdu_lock);

	/* CDU output enable gates */
	clks[ADSP_SC846_CLK_SHARC0] = cdu_gate("sharc0", "sharc0_sel",
						cdu + CDU_CFG0, CLK_IS_CRITICAL,
						&cdu_lock);
	clks[ADSP_SC846_CLK_ARM0] = cdu_gate("arm0", "arm0_sel", cdu + CDU_CFG1,
					     CLK_IS_CRITICAL, &cdu_lock);
	clks[ADSP_SC846_CLK_ARM1] = cdu_gate("arm1", "arm1_sel", cdu + CDU_CFG2,
					     CLK_IS_CRITICAL, &cdu_lock);
	clks[ADSP_SC846_CLK_CDU_DDR] = cdu_gate("cdu_ddr", "ddr_sel",
						cdu + CDU_CFG3, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_CAN] = cdu_gate("can", "can_sel", cdu + CDU_CFG4, 0,
					    &cdu_lock);
	clks[ADSP_SC846_CLK_SPDIF] = cdu_gate("spdif", "spdif_sel",
					      cdu + CDU_CFG5, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_SPI] = cdu_gate("spi", "spi_sel", cdu + CDU_CFG6, 0,
					    &cdu_lock);
	clks[ADSP_SC846_CLK_GIGE] = cdu_gate("gige", "gige_sel", cdu + CDU_CFG7,
					     0, &cdu_lock);
	clks[ADSP_SC846_CLK_XSPI1] = cdu_gate("xspi1", "xspi1_sel",
					      cdu + CDU_CFG8, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_LP] = cdu_gate("lp", "lp_sel", cdu + CDU_CFG9, 0,
					   &cdu_lock);
	clks[ADSP_SC846_CLK_XSPI0] = cdu_gate("xspi0", "xspi0_sel",
					      cdu + CDU_CFG10, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_TRACE] = cdu_gate("trace", "trace_sel",
					      cdu + CDU_CFG12, 0, &cdu_lock);
	clks[ADSP_SC846_CLK_PWM] = cdu_gate("pwm", "pwm_sel", cdu + CDU_CFG13,
					    0, &cdu_lock);
	clks[ADSP_SC846_CLK_MSHC] = cdu_gate("mshc", "mshc_sel",
					     cdu + CDU_CFG14, 0, &cdu_lock);

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

static const struct of_device_id adi_sc846_clock_of_match[] = {
	{ .compatible = "adi,sc846-clocks" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_sc846_clock_of_match);

CLK_OF_DECLARE(sc846_clocks, "adi,sc846-clocks", sc846_clock_probe);

MODULE_AUTHOR("Analog Devices Inc.");
MODULE_DESCRIPTION("ADI SC846 Clock Driver");
MODULE_LICENSE("GPL v2");
