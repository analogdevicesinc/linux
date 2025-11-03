// SPDX-License-Identifier: GPL-2.0
/*
 * Clock support for ADI processor
 *
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#include <dt-bindings/clock/adi-sc5xx-clock.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "clk.h"

static const char * const cgu1_in_sels[] = {"sys_clkin0", "sys_clkin1"};
static const char * const cgu0_s1sels[] = {"cgu0_s1seldiv", "cgu0_s1selexdiv"};
static const char * const cgu1_s0sels[] = {"cgu1_s0seldiv", "cgu1_s0selexdiv"};
static const char * const cgu1_s1sels[] = {"cgu1_s1seldiv", "cgu1_s1selexdiv"};
static const char * const sharc0_sels[] = {"cclk0_0", "dummy", "dummy", "dummy"};
static const char * const sharc1_sels[] = {"cclk0_0", "dummy", "dummy", "dummy"};
static const char * const arm_sels[] = {"dummy", "dummy", "cclk2_0", "cclk2_1"};
static const char * const cdu_ddr_sels[] = {"dclk_0", "dclk_1", "dummy", "dummy"};
static const char * const can_sels[] = {"dummy", "oclk_1", "dummy", "dummy"};
static const char * const spdif_sels[] = {"sclk1_0", "dummy", "dummy", "dummy"};
static const char * const spi_sels[] = {"sclk0_0", "oclk_0", "dummy", "dummy"};
static const char * const gige_sels[] = {"sclk0_0", "sclk0_1", "dummy", "dummy"};
static const char * const lp_sels[] = {"oclk_0", "sclk0_0", "cclk0_1", "dummy"};
static const char * const lp_ddr_sels[] = {"oclk_0", "dclk_0", "sysclk_1", "dummy"};
static const char * const ospi_refclk_sels[] = {"sysclk_0", "sclk0_0", "sclk1_1", "dummy"};
static const char * const trace_sels[] = {"sclk0_0", "dummy", "dummy", "dummy"};
static const char * const emmc_sels[] = {"oclk_0", "sclk0_1", "dclk_0_half", "dclk_1_half"};
static const char * const emmc_timer_sels[] = {"dummy", "sclk1_1_half", "dummy", "dummy"};

static const char * const ddr_sels[] = {"cdu_ddr", "3pll_ddiv"};

static struct adi_clk_branch cgu0_branches[] __initdata = {
	/*
	 * Clock Generation Unit 0
	 */
	DIVIDER(ADSP_CLK_CGU0_PLL_IN, "cgu0_df", "sys_clkin0", CGU_CTL,
		0, 1, 0),
	PLL(ADSP_CLK_CGU0_VCO_OUT, "cgu0_vco_msel", "cgu0_df", CGU_CTL,
		CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0),
	FIXED(ADSP_CLK_CGU0_VCO_2_OUT, "cgu0_vco", "cgu0_vco_msel", CLK_SET_RATE_PARENT, 2, 1),
	FIXED(ADSP_CLK_CGU0_PLLCLK, "cgu0_pllclk", "cgu0_vco", CLK_SET_RATE_PARENT, 1, 2),
	DIVIDER(ADSP_CLK_CGU0_CDIV, "cgu0_cdiv", "cgu0_pllclk", CGU_DIV,
		0, 5, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU0_SYSCLK, "sysclk_0", "cgu0_pllclk", CGU_DIV,
		8, 5, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU0_DDIV, "cgu0_ddiv", "cgu0_pllclk", CGU_DIV,
		16, 5, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU0_ODIV, "cgu0_odiv", "cgu0_pllclk", CGU_DIV,
		22, 7, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU0_S0SELDIV, "cgu0_s0seldiv", "sysclk_0", CGU_DIV,
		5, 3, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU0_S1SELDIV, "cgu0_s1seldiv", "sysclk_0", CGU_DIV,
		13, 3, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU0_S1SELEXDIV, "cgu0_s1selexdiv", "cgu0_pllclk",
		CGU_DIVEX, 16, 8, CLK_DIVIDER_MAX_AT_ZERO),
	MUX(ADSP_CLK_CGU0_S1SEL, "cgu0_sclk1sel", cgu0_s1sels, CLK_SET_RATE_PARENT, CGU_CTL, 17, 1),
	FIXED(ADSP_CLK_CGU0_CCLK2, "cclk2_0", "cgu0_vco", CLK_SET_RATE_PARENT, 1, 3),
	CGU_GATE(ADSP_CLK_CGU0_CCLK0, "cclk0_0", "cgu0_cdiv", CGU_CCBF_DIS, 0),
	CGU_GATE(ADSP_CLK_CGU0_OCLK, "oclk_0", "cgu0_odiv", CGU_SCBF_DIS, 3),
	CGU_GATE(ADSP_CLK_CGU0_DCLK, "dclk_0", "cgu0_ddiv", CGU_SCBF_DIS, 2),
	CGU_GATE(ADSP_CLK_CGU0_SCLK1, "sclk1_0", "cgu0_sclk1sel", CGU_SCBF_DIS, 1),
	CGU_GATE(ADSP_CLK_CGU0_SCLK0, "sclk0_0", "cgu0_s0seldiv", CGU_SCBF_DIS, 0),
	FIXED(ADSP_CLK_DCLK0_HALF, "dclk_0_half", "dclk_0", CLK_SET_RATE_PARENT, 1, 2),
};

static struct adi_clk_branch cgu1_branches[] __initdata = {
	/*
	 * Clock Generation Unit 1
	 */
	DIVIDER(ADSP_CLK_CGU1_PLL_IN, "cgu1_df", "cgu1_in_sel", CGU_CTL,
		0, 1, 0),
	PLL(ADSP_CLK_CGU1_VCO_OUT, "cgu1_vco_msel", "cgu1_df", CGU_CTL,
		CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0),
	FIXED(ADSP_CLK_CGU1_VCO_2_OUT, "cgu1_vco", "cgu1_vco_msel", CLK_SET_RATE_PARENT, 2, 1),
	FIXED(ADSP_CLK_CGU1_PLLCLK, "cgu1_pllclk", "cgu1_vco", CLK_SET_RATE_PARENT, 1, 2),
	DIVIDER(ADSP_CLK_CGU1_CDIV, "cgu1_cdiv", "cgu1_pllclk", CGU_DIV,
		0, 5, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU1_SYSCLK, "sysclk_1", "cgu1_pllclk", CGU_DIV,
		8, 5, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU1_DDIV, "cgu1_ddiv", "cgu1_pllclk", CGU_DIV,
		16, 5, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU1_ODIV, "cgu1_odiv", "cgu1_pllclk", CGU_DIV,
		22, 7, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU1_S0SELDIV, "cgu1_s0seldiv", "sysclk_1", CGU_DIV,
		5, 3, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU1_S1SELDIV, "cgu1_s1seldiv", "sysclk_1", CGU_DIV,
		13, 3, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU1_S0SELEXDIV, "cgu1_s0selexdiv", "cgu1_pllclk", CGU_DIVEX, 0, 8, CLK_DIVIDER_MAX_AT_ZERO),
	DIVIDER(ADSP_CLK_CGU1_S1SELEXDIV, "cgu1_s1selexdiv", "cgu1_pllclk", CGU_DIVEX, 16, 8, CLK_DIVIDER_MAX_AT_ZERO),
	MUX(ADSP_CLK_CGU1_S0SEL, "cgu1_sclk0sel", cgu1_s0sels, CLK_SET_RATE_PARENT, CGU_CTL, 16, 1),
	MUX(ADSP_CLK_CGU1_S1SEL, "cgu1_sclk1sel", cgu1_s1sels, CLK_SET_RATE_PARENT, CGU_CTL, 17, 1),
	FIXED(ADSP_CLK_CGU1_CCLK2, "cclk2_1", "cgu1_vco", CLK_SET_RATE_PARENT, 1, 3),
	CGU_GATE(ADSP_CLK_CGU1_CCLK0, "cclk0_1", "cgu1_cdiv", CGU_CCBF_DIS, 0),
	CGU_GATE(ADSP_CLK_CGU1_OCLK, "oclk_1", "cgu1_odiv", CGU_SCBF_DIS, 3),
	CGU_GATE(ADSP_CLK_CGU1_DCLK, "dclk_1", "cgu1_ddiv", CGU_SCBF_DIS, 2),
	CGU_GATE(ADSP_CLK_CGU1_SCLK1, "sclk1_1", "cgu1_sclk1sel", CGU_SCBF_DIS, 1),
	CGU_GATE(ADSP_CLK_CGU1_SCLK0, "sclk0_1", "cgu1_sclk0sel", CGU_SCBF_DIS, 0),
	FIXED(ADSP_CLK_CGU1_SCLK1_HALF, "sclk1_1_half", "sclk1_1", CLK_SET_RATE_PARENT, 1, 2),
};

static struct adi_clk_branch cdu_branches[] __initdata = {
	/*
	 * Clock Distribution Unit
	 */
	MUX(ADSP_CLK_CGU1_IN, "cgu1_in_sel", cgu1_in_sels, CLK_SET_RATE_PARENT, CDU_CLKINSEL, 0, 1),
	CDU_MUX(ADSP_CLK_SHARC0_SEL, "sharc0_sel", sharc0_sels,  CDU_CFG0,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_SHARC1_SEL, "sharc1_sel", sharc1_sels, CDU_CFG1,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_ARM_SEL, "arm_sel", arm_sels, CDU_CFG2,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_CDU_DDR_SEL, "cdu_ddr_sel", cdu_ddr_sels, CDU_CFG3,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	FIXED(ADSP_CLK_DCLK1_HALF, "dclk_1_half", "dclk_1", CLK_SET_RATE_PARENT, 1, 2),
	CDU_MUX(ADSP_CLK_CAN_SEL, "can_sel", can_sels, CDU_CFG4,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_SPDIF_SEL, "spdif_sel", spdif_sels, CDU_CFG5,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_SPI_SEL, "spi_sel", spi_sels, CDU_CFG6,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_GIGE_SEL, "gige_sel", gige_sels, CDU_CFG7,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_LP_SEL, "lp_sel", lp_sels, CDU_CFG8,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_LP_DDR_SEL, "lp_ddr_sel", lp_ddr_sels, CDU_CFG9,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_OSPI_REFCLK_SEL, "ospi_refclk_sel", ospi_refclk_sels, CDU_CFG10,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_TRACE_SEL, "trace_sel", trace_sels, CDU_CFG12,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_EMMC_SEL, "emmc_sel", emmc_sels, CDU_CFG13,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),
	CDU_MUX(ADSP_CLK_EMMC_TIMER_QMC_SEL, "emmc_timer_qmc_sel", emmc_timer_sels, CDU_CFG14,
		CDU_MUX_SHIFT, CDU_MUX_WIDTH),

	CDU_GATE(ADSP_CLK_SHARC0, "sharc0", "sharc0_sel", CDU_CFG0, CLK_IS_CRITICAL),
	CDU_GATE(ADSP_CLK_SHARC1, "sharc1", "sharc1_sel", CDU_CFG1, CLK_IS_CRITICAL),
	CDU_GATE(ADSP_CLK_ARM, "arm", "arm_sel", CDU_CFG2, CLK_IS_CRITICAL),
	CDU_GATE(ADSP_CLK_CDU_DDR, "cdu_ddr", "cdu_ddr_sel", CDU_CFG3, 0),
	CDU_GATE(ADSP_CLK_CAN, "can", "can_sel", CDU_CFG4, 0),
	CDU_GATE(ADSP_CLK_SPDIF, "spdif", "spdif_sel", CDU_CFG5, 0),
	CDU_GATE(ADSP_CLK_SPI, "spi", "spi_sel", CDU_CFG6, 0),
	CDU_GATE(ADSP_CLK_GIGE, "gige", "gige_sel", CDU_CFG7, 0),
	CDU_GATE(ADSP_CLK_LP, "lp", "lp_sel", CDU_CFG8, 0),
	CDU_GATE(ADSP_CLK_LP_DDR, "lp_ddr", "lp_ddr_sel", CDU_CFG9, 0),
	CDU_GATE(ADSP_CLK_OSPI_REFCLK, "ospi_refclk", "ospi_refclk_sel", CDU_CFG10, 0),
	CDU_GATE(ADSP_CLK_TRACE, "trace", "trace_sel", CDU_CFG12, 0),
	CDU_GATE(ADSP_CLK_EMMC, "emmc", "emmc_sel", CDU_CFG13, 0),
	CDU_GATE(ADSP_CLK_EMMC_TIMER_QMC, "emmc_timer_qmc", "emmc_timer_qmc_sel", CDU_CFG14, 0),
};

static struct adi_clk_branch pll3_branches[] __initdata = {
	DIVIDER(ADSP_CLK_3PLL_PLL_IN, "3pll_df", "cgu1_in_sel", PLL3_CONTROL,
		3, 1, 0),
	PLL(ADSP_CLK_3PLL_VCO_OUT, "pll3_vco_msel", "3pll_df", PLL3_CONTROL,
		PLL3_MSEL_SHIFT, PLL3_MSEL_WIDTH, 1),
	FIXED(ADSP_CLK_3PLL_VCO_2_OUT, "3pll_vco", "pll3_vco_msel", CLK_SET_RATE_PARENT, 2, 1),
	FIXED(ADSP_CLK_3PLL_PLLCLK, "3pll_pllclk", "3pll_vco", CLK_SET_RATE_PARENT, 1, 2),
	DIVIDER(ADSP_CLK_3PLL_DDIV, "3pll_ddiv", "3pll_pllclk", PLL3_CONTROL,
		12, 5, 0),
	MUX(ADSP_CLK_DDR, "ddr", ddr_sels, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, PLL3_CONTROL,
		11, 1),
};

static void sc5xx_clock_setup(struct device_node *np,
			      struct adi_clk_branch *branch_list,
			      unsigned int nr_clk)
{
	struct adi_clk_provider *ctx;
	void __iomem *reg_base;

	reg_base = of_iomap(np, 0);
	if (IS_ERR(reg_base)) {
		pr_err("Unable to remap clock registers\n");
		return;
	}

	ctx = adi_clk_init(np, reg_base, ADSP_CLK_NR_CLKS);
	if (IS_ERR(ctx)) {
		pr_err("%s: clk init failed\n", __func__);
		iounmap(reg_base);
		return;
	}

	adi_clk_register_branches(ctx, branch_list, nr_clk);

	if (of_clk_add_provider(np, of_clk_src_onecell_get, &ctx->clk_data))
		pr_err("%s: could not register clock provider\n", __func__);
}

static void sc598_clock_setup_cgu0(struct device_node *np)
{
	sc5xx_clock_setup(np, cgu0_branches, ARRAY_SIZE(cgu0_branches));
}

static void sc598_clock_setup_cgu1(struct device_node *np)
{
	sc5xx_clock_setup(np, cgu1_branches, ARRAY_SIZE(cgu1_branches));
}

static void sc598_clock_setup_cdu(struct device_node *np)
{
	sc5xx_clock_setup(np, cdu_branches, ARRAY_SIZE(cdu_branches));
}

static void sc598_clock_setup_pll(struct device_node *np)
{
	sc5xx_clock_setup(np, pll3_branches, ARRAY_SIZE(pll3_branches));
}

CLK_OF_DECLARE(adi_sc598_clocks_cgu0, "adi,sc598-cgu0", sc598_clock_setup_cgu0);
CLK_OF_DECLARE(adi_sc598_clocks_cgu1, "adi,sc598-cgu1", sc598_clock_setup_cgu1);
CLK_OF_DECLARE(adi_sc598_clocks_cdu, "adi,sc598-cdu", sc598_clock_setup_cdu);
CLK_OF_DECLARE(adi_sc598_clocks_pll, "adi,sc598-pll", sc598_clock_setup_pll);

MODULE_DESCRIPTION("Analog Devices Clock driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Malysa <malysagreg@gmail.com>");
