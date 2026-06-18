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

#define IN0_CLKON	0x0
#define IN1_CLKON	0x1
#define IN2_CLKON	0x2
#define IN3_CLKON	0x3

/* Number of clocks per clock controller for each SoC */
#define NUM_CLKS_SC594_CGU0	(ADSP_SC594_CLK_CGU_CCLK1 + 1)
#define NUM_CLKS_SC594_CGU1	(ADSP_SC594_CLK_CGU_CCLK1 + 1)
#define NUM_CLKS_SC594_CDU	(ADSP_SC594_CLK_CDU_TRACE + 1)
#define NUM_CLKS_SC598_CGU0	(ADSP_SC598_CLK_CGU_VCO_OUT + 1)
#define NUM_CLKS_SC598_CGU1	(ADSP_SC598_CLK_CGU_S0SEL + 1)
#define NUM_CLKS_SC598_CDU	(ADSP_SC598_CLK_CDU_DDR_SEL + 1)
#define NUM_CLKS_SC598_PLL2	(ADSP_SC598_CLK_PLL2_DDIV + 1)
#define NUM_CLKS_SC58X_CGU0	(ADSP_SC58X_CLK_CGU_OCLK_HALF + 1)
#define NUM_CLKS_SC58X_CGU1	(ADSP_SC58X_CLK_CGU_S0SEL + 1)
#define NUM_CLKS_SC58X_CDU	(ADSP_SC58X_CLK_CDU_SDIO + 1)
#define NUM_CLKS_SC57X_CGU0	(ADSP_SC57X_CLK_CGU_OCLK0_HALF + 1)
#define NUM_CLKS_SC57X_CGU1	(ADSP_SC57X_CLK_CGU_S0SEL + 1)
#define NUM_CLKS_SC57X_CDU	(ADSP_SC57X_CLK_CDU_SDIO + 1)

/* CDU CLKOn input clock source configurations */
MUX_TABLE(CDU_CLKO_SEL1)	= { IN0_CLKON };
MUX_TABLE(CDU_CLKO_SEL2)	= { IN1_CLKON };
MUX_TABLE(CDU_CLKO_SEL3)	= { IN0_CLKON, IN1_CLKON };
MUX_TABLE(CDU_CLKO_SEL4)	= { IN0_CLKON, IN1_CLKON, IN2_CLKON };
MUX_TABLE(CDU_CLKO_SEL5)	= { IN0_CLKON, IN2_CLKON, IN3_CLKON };
MUX_TABLE(CDU_CLKO_SEL6)	= { IN0_CLKON, IN1_CLKON, IN2_CLKON, IN3_CLKON };

/* ADSP-SC5XX CGU0 and CGU1 root clocks */
CLK_PARENT_DATA(cgu0_parents)		 = { { .fw_name = "sys_clkin0"   }, };
CLK_PARENT_DATA(cgu1_parents)		 = { { .fw_name = "cdu_clkinsel" }, };

/* ADSP-SC598 mux parents */
CLK_PARENT_DATA(cdu_sharc0_parents)	 = { { .fw_name = "cclk0_0"    }, };
CLK_PARENT_DATA(cdu_sharc1_parents)	 = { { .fw_name = "cclk0_0"    }, };
CLK_PARENT_DATA(cdu_can_parents)	 = { { .fw_name = "oclk_1"     }, };
CLK_PARENT_DATA(cdu_spdif_parents)	 = { { .fw_name = "sclk1_0"    }, };
CLK_PARENT_DATA(cdu_trace_parents)	 = { { .fw_name = "sclk0_0"    }, };
CLK_PARENT_DATA(cdu_emmc_timer_parents)	 = { { .fw_name = "sclk1_1/2"  }, };
CLK_PARENT_DATA(cdu_clkinsel_parents)	 = { { .name = "sys_clkin0"    }, { .name = "sys_clkin1" }, };
CLK_PARENT_DATA(cdu_arm_parents)	 = { { .fw_name = "cclk2_0"    }, { .fw_name = "cclk2_1" }, };
CLK_PARENT_DATA(cdu_ddr_parents)	 = { { .fw_name = "dclk_0"     }, { .fw_name = "dclk_1"  }, };
CLK_PARENT_DATA(cdu_spi_parents)	 = { { .fw_name = "sclk0_0"    }, { .fw_name = "oclk_0"  }, };
CLK_PARENT_DATA(cdu_gige_parents)	 = { { .fw_name = "sclk0_0"    }, { .fw_name = "sclk0_1" }, { .fw_name = "oclk_0"   }, };
CLK_PARENT_DATA(cdu_lp_parents)		 = { { .fw_name = "oclk_0"     }, { .fw_name = "sclk0_0" }, { .fw_name = "cclk0_1"  }, };
CLK_PARENT_DATA(cdu_lpddr_parents)	 = { { .fw_name = "oclk_0"     }, { .fw_name = "dclk_0"  }, { .fw_name = "sysclk_1" }, };
CLK_PARENT_DATA(cdu_ospi_refclk_parents) = { { .fw_name = "sysclk_0"   }, { .fw_name = "sclk0_0" }, { .fw_name = "sclk1_1"  }, };
CLK_PARENT_DATA(cdu_emmc_parents)	 = { { .fw_name = "oclk_0"     }, { .fw_name = "sclk0_1" }, { .fw_name = "dclk_0/2" },
					     { .fw_name = "dclk_1/2"   }, };
/* ADSP-SC5XX CLKINSEL mux */
static const struct sc5xx_clkinsel_clock sc5xx_clkinsel_clks[] __initdata = {
	CLKINSEL(ADSP_SC5XX_CLK_CDU_CLKINSEL, "cdu_clkinsel", cdu_clkinsel_parents, 0),
};

/* ADSP-SC5XX CGU PLL divider clocks */
/* TODO: store parents in the final clocks struct during registration instead of here, make new macro for this */
static const struct sc5xx_div_clock sc5xx_cgu_div_clks_pll[] = {
	DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu1_df_div", CGU_CTL, 0, 1), 
};

/* ADSP-SC598 CDU mux clocks */
static const struct sc5xx_mux_clock sc598_mux_clks[] __initdata = {
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC0, "cdu_sharc0", cdu_sharc0_parents, 0, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC1, "cdu_sharc1", cdu_sharc1_parents, 1, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_ARM, "cdu_arm",	cdu_arm_parents, 2, CDU_CLKO_SEL5, 0),
	MUX(ADSP_SC5XX_CLK_CDU_DDR, "cdu_ddr", cdu_ddr_parents, 3, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_CAN, "cdu_can", cdu_can_parents, 4, CDU_CLKO_SEL2, 0),
	MUX(ADSP_SC5XX_CLK_CDU_SPDIF, "cdu_spdif", cdu_spdif_parents, 5, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC598_CLK_CDU_SPI, "cdu_spi", cdu_spi_parents, 6, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_GIGE, "cdu_gige", cdu_gige_parents, 7, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC598_CLK_CDU_LP, "cdu_lp", cdu_lp_parents, 8, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC598_CLK_CDU_LPDDR, "cdu_lpddr", cdu_lpddr_parents, 9, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC598_CLK_CDU_TRACE, "cdu_trace", cdu_trace_parents, 12, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC598_CLK_CDU_EMMC, "cdu_emmc", cdu_emmc_parents, 13, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC598_CLK_CDU_EMMC_TIMER_QMC, "cdu_emmc_timer", cdu_emmc_timer_parents, 14, CDU_CLKO_SEL2, 0),
	MUX(ADSP_SC598_CLK_CDU_OSPI_REFCLK, "cdu_ospi_refclk", cdu_ospi_refclk_parents, 10, CDU_CLKO_SEL4, 0),
};

/* ADSP-SC598 CGU0 divider clocks */
static const struct sc5xx_div_clock sc598_div_clks_cgu0[] = {
	DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu0_df_div",  
	DIV(ADSP_SC5XX_CLK_CGU_CSEL_DIV, "cgu0_csel_div", 
	DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV, "cgu0_syssel_div", 
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV, "cgu0_dsel_div", 
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV, "cgu0_osel_div", 
	DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV, "cgu0_s0sel_div", 
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV, "cgu0_s1sel_div", 
	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV, "cgu0_s1selex_div", 
};

/* ADSP-SC598 CGU1 divider clocks */
static const struct sc5xx_div_clock sc598_div_clks_cgu1[] = {

};

static void __init sc5xx_early_clock_probe(struct device_node *np)
{

}
CLK_OF_DECLARE();

static int sc5xx_clock_probe(struct platform_device *pdev)
{
	struct device *dev = pdev->dev;

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
