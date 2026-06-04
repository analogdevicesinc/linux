// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock driver for ADSP-SC5xx SoCs
 *
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * Author: Qasim Ijaz <qasim.ijaz@analog.com>
 * Contact: linux@analog.com
 */

#include <dt-bindings/clock/adi-sc5xx-clock.h>
#include "clk.h"

PARENT_CLKS(cgu1_in_parents)	    = { "sys_clkin0", "sys_clkin1" };
PARENT_CLKS(cgu0_sclk1_parents)	    = { "cgu0_s1seldiv", "cgu0_s1selexdiv" };
PARENT_CLKS(cgu1_sclk0_parents)	    = { "cgu1_s0seldiv", "cgu1_s0selexdiv" };
PARENT_CLKS(cgu1_sclk1_parents)	    = { "cgu1_s1seldiv", "cgu1_s1selexdiv" };
PARENT_CLKS(sharc0_parents)	    = { "cclk0_0" };
PARENT_CLKS(sharc1_parents)	    = { "cclk0_0" };
PARENT_CLKS(arm_parents)	    = { "cclk2_0", "cclk2_1" };
PARENT_CLKS(cdu_ddr_parents)	    = { "dclk_0", "dclk_1" };
PARENT_CLKS(can_parents)	    = { "oclk_1" };
PARENT_CLKS(spdif_parents)	    = { "sclk1_0" };
PARENT_CLKS(spi_parents)	    = { "sclk0_0", "oclk_0" };
PARENT_CLKS(gige_parents)	    = { "sclk0_0", "sclk0_1", "oclk_0" };
PARENT_CLKS(lp_parents)		    = { "oclk_0", "sclk0_0", "cclk0_1" };
PARENT_CLKS(lp_ddr_parents)	    = { "oclk_0", "dclk_0", "sysclk_1" };
PARENT_CLKS(ospi_refclk_parents)    = { "sysclk_0", "sclk0_0", "sclk1_1" };
PARENT_CLKS(trace_parents)	    = { "sclk0_0" };
PARENT_CLKS(emmc_parents)	    = { "oclk_0", "sclk0_1", "dclk_0_half", "dclk_1_half" };
PARENT_CLKS(emmc_timer_qmc_parents) = { "sclk1_1_half" };
PARENT_CLKS(ddr_parents)	    = { "cdu_ddr", "3pll_ddiv" };

static const struct sc5xx_mux_clock sc598_mux_clks[] __initconst = {
	CMUX(ADSP_SC598_CLK_CDU_SHARC0, {}),
	CMUX(ADSP_SC598_CLK_CDU_SHARC1, {}),
	CMUX(ADSP_SC598_CLK_CDU_ARM, {}),
	MUX(ADSP_SC598_CLK_CDU_DDR, ),
	MUX(ADSP_SC598_CLK_CDU_CAN, ),
	MUX(ADSP_SC598_CLK_CDU_SPDIF, ),
	MUX(ADSP_SC598_CLK_CDU_SPI, ),
	MUX(ADSP_SC598_CLK_CDU_GIGE, ),
	MUX(ADSP_SC598_CLK_CDU_LP, ),
	MUX(ADSP_SC598_CLK_CDU_LP_DDR, ),
	MUX(ADSP_SC598_CLK_CDU_OSPI_REFCLK, ),
	MUX(ADSP_SC598_CLK_CDU_TRACE, ),
	MUX(ADSP_SC598_CLK_CDU_EMMC, ),
	MUX(ADSP_SC598_CLK_CDU_EMMC_TIMER_QMC, ),
};

static const struct sc5xx_mux_clock sc594_mux_clks[] __initconst = {
	CMUX(ADSP_SC598_CLK_CDU_SHARC0, ),
	CMUX(ADSP_SC598_CLK_CDU_SHARC1, ),
	CMUX(ADSP_SC598_CLK_CDU_ARM, ),
	MUX(ADSP_SC598_CLK_CDU_DDR, ),
	MUX(ADSP_SC598_CLK_CDU_CAN, ),
	MUX(ADSP_SC598_CLK_CDU_SPDIF, ),
	MUX(ADSP_SC598_CLK_CDU_SPI, ),
	MUX(ADSP_SC598_CLK_CDU_GIGE, ),
	MUX(ADSP_SC598_CLK_CDU_LP, ),
	MUX(ADSP_SC598_CLK_CDU_LP_DDR, ),
	MUX(ADSP_SC598_CLK_CDU_OSPI_REFCLK, ),
	MUX(ADSP_SC598_CLK_CDU_TRACE, ),
};

static const struct sc5xx_mux_clock sc573_mux_clks[] __initconst = {
	CMUX(ADSP_SC598_CLK_CDU_SHARC0, ),
	CMUX(ADSP_SC598_CLK_CDU_SHARC1, ),
	CMUX(ADSP_SC598_CLK_CDU_ARM, ),
	MUX(ADSP_SC598_CLK_CDU_DDR, ),
	MUX(ADSP_SC598_CLK_CDU_CAN, ),
	MUX(ADSP_SC598_CLK_CDU_SPDIF, ),
	MUX(ADSP_SC598_CLK_CDU_GIGE, ),
	MUX(ADSP_SC598_CLK_CDU_SDIO, ),
};

static const struct sc5xx_mux_clock sc589_mux_clks[] __initconst = {
	CMUX(ADSP_SC598_CLK_CDU_SHARC0, ),
	CMUX(ADSP_SC598_CLK_CDU_SHARC1, ),
	CMUX(ADSP_SC598_CLK_CDU_ARM, ),
	MUX(ADSP_SC598_CLK_CDU_DDR, ),
	MUX(ADSP_SC598_CLK_CDU_CAN, ),
	MUX(ADSP_SC598_CLK_CDU_SPDIF, ),
	MUX(ADSP_SC598_CLK_CDU_GIGE, ),
	MUX(ADSP_SC598_CLK_CDU_LP, ),
	MUX(ADSP_SC598_CLK_CDU_SDIO, ),
};

static void __init sc5xx_early_clock_probe(struct device_node *np)
{

}
CLK_OF_DECLARE();

static int sc5xx_clock_probe(struct platform_device *pdev)
{
	return 0;
}

static void sc5xx_clock_remove(struct platform_device *pdev)
{
}

static const struct of_device_id sc5xx_clock_of_match[] = {
	{
		.compatible = "adi,sc598-cgu0",
		.data = &sc598_cgu0_info,
	}, {
		.compatible = "adi,sc598-cgu1",
		.data = &sc598_cgu1_info,
	}, {
		.compatible = "adi,sc598-pll2",
		.data = &sc598_pll2_info,
	}, {
		.compatible = "adi,sc598-cdu",
		.data = &sc598_cdu_info,
	}, {
		.compatible = "adi,sc589-cgu0",
		.data = &sc589_cgu0_info,
	}, {
		.compatible = "adi,sc589-cgu1",
		.data = &sc589_cgu1_info,
	}, {
		.compatible = "adi,sc589-cdu",
		.data = &sc589_cdu_info,
	}, {
		.compatible = "adi,sc573-cgu0",
		.data = &sc573_cgu0_info,
	}, {
		.compatible = "adi,sc573-cgu1",
		.data = &sc573_cgu1_info,
	}, {
		.compatible = "adi,sc573-cdu",
		.data = &sc573_cdu_info,
	}, {
		.compatible = "adi,sc594-cgu0",
		.data = &sc594_cgu0_info,
	}, {
		.compatible = "adi,sc594-cgu1",
		.data = &sc594_cgu1_info,
	}, {
		.compatible = "adi,sc594-cdu",
		.data = &sc594_cdu_info,
	}, {
	},
};

static struct platform_driver sc5xx_clock_driver = {
	.driver	= {
		.name = "sc5xx_clock",
		.of_match_table = sc5xx_clock_of_match,
	},
	.probe = sc5xx_clock_probe,
	.remove = sc5xx_clock_remove,
};

static int __init sc5xx_clock_init(void)
{
	return platform_driver_register(&sc5xx_clock_driver);
}
core_initcall(sc5xx_clock_init);

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Clock driver for ADSP-SC5xx SoCs");
MODULE_LICENSE("GPLv2");
