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
#include <linux/platform_device.h>
#include <linux/clk-provider.h>

#include "clk.h"

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define IN0_CLKON	0x0
#define IN1_CLKON	0x1
#define IN2_CLKON	0x2
#define IN3_CLKON	0x3

#define CDU_CLKO0	0x0
#define CDU_CLKO1	0x1
#define CDU_CLKO2	0x2
#define CDU_CLKO3	0x3
#define CDU_CLKO4	0x4
#define CDU_CLKO5	0x5
#define CDU_CLKO6	0x6
#define CDU_CLKO7	0x7
#define CDU_CLKO8	0x8
#define CDU_CLKO9	0x9
#define CDU_CLKO10	0xA
#define CDU_CLKO12	0xC
#define CDU_CLKO13	0xD
#define CDU_CLKO14	0xE

#define CLK_PARENT_DATA(_name)	\
	static const struct clk_parent_data _name[]

#define HW_PARENTS(_name, ...)	\
	static const u8 _name[] = { __VA_ARGS__ }

#define MUX_TABLE(_name)	\
	static const u32 _name[]

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

/* ADSP-SC598 mux parents */
CLK_PARENT_DATA(cdu_sharc0_parents)	 = { { .fw_name = "cclk0_0"    }, };
CLK_PARENT_DATA(cdu_sharc1_parents)	 = { { .fw_name = "cclk0_0"    }, };
CLK_PARENT_DATA(cdu_can_parents)	 = { { .fw_name = "oclk_1"     }, };
CLK_PARENT_DATA(cdu_spdif_parents)	 = { { .fw_name = "sclk1_0"    }, };
CLK_PARENT_DATA(cdu_trace_parents)	 = { { .fw_name = "sclk0_0"    }, };
CLK_PARENT_DATA(cdu_emmc_timer_parents)	 = { { .fw_name = "sclk1_1/2"  }, };
CLK_PARENT_DATA(cdu_clkinsel_parents)	 = { { .fw_name = "sys_clkin0" }, { .fw_name = "sys_clkin1" }, };
CLK_PARENT_DATA(cdu_arm_parents)	 = { { .fw_name = "cclk2_0"    }, { .fw_name = "cclk2_1"    }, };
CLK_PARENT_DATA(cdu_ddr_parents)	 = { { .fw_name = "dclk_0"     }, { .fw_name = "dclk_1"     }, };
CLK_PARENT_DATA(cdu_spi_parents)	 = { { .fw_name = "sclk0_0"    }, { .fw_name = "oclk_0"     }, };
CLK_PARENT_DATA(cdu_gige_parents)	 = { { .fw_name = "sclk0_0"    }, { .fw_name = "sclk0_1"    }, { .fw_name = "oclk_0"   }, };
CLK_PARENT_DATA(cdu_lp_parents)		 = { { .fw_name = "oclk_0"     }, { .fw_name = "sclk0_0"    }, { .fw_name = "cclk0_1"  }, };
CLK_PARENT_DATA(cdu_lpddr_parents)	 = { { .fw_name = "oclk_0"     }, { .fw_name = "dclk_0"     }, { .fw_name = "sysclk_1" }, };
CLK_PARENT_DATA(cdu_ospi_refclk_parents) = { { .fw_name = "sysclk_0"   }, { .fw_name = "sclk0_0"    }, { .fw_name = "sclk1_1"  }, };
CLK_PARENT_DATA(cdu_emmc_parents)	 = { { .fw_name = "oclk_0"     }, { .fw_name = "sclk0_1"    }, { .fw_name = "dclk_0/2" }, 
					     { .fw_name = "dclk_1/2"   }, };

/* ADSP-SC5XX CGU0 and CGU1 parent clocks */
CLK_PARENT_DATA(cgu0_parents)	= { { .fw_name = "sys_clkin0"   }, };
CLK_PARENT_DATA(cgu1_parents)	= { { .fw_name = "cdu_clkinsel" }, };

/* ADSP-SC598 HW parent refs */
HW_PARENTS(syssel_parent)	= { ADSP_SC5XX_CLK_CGU_SYSSEL_DIV };
HW_PARENTS(pllclk_parent)	= { ADSP_SC5XX_CLK_CGU_PLLCLK };
HW_PARENTS(vco_parent)		= { ADSP_SC5XX_CLK_CGU_DF_DIV };

/* ADSP-SC5XX CLKINSEL mux */
static const struct sc5xx_clkinsel_clock sc5xx_clkinsel_clks[] = {
	CLKINSEL(ADSP_SC5XX_CLK_CDU_CLKINSEL, "cdu_clkinsel", cdu_clkinsel_parents, 0),
};

/* ADSP-SC5XX CGU0 PLL divider clocks */
static const struct sc5xx_div_clock sc5xx_cgu0_div_clks_pll[] = {
	DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu0_df_div", cgu0_parents, CGU_CTL, 0, 1), 
};

/* ADSP-SC598 CDU mux clocks */
static const struct sc5xx_mux_clock sc598_mux_clks[] = {
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC0, "cdu_sharc0", cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC1, "cdu_sharc1", cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_ARM, "cdu_arm", cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL5, 0),
	MUX(ADSP_SC5XX_CLK_CDU_DDR, "cdu_ddr", cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_CAN, "cdu_can", cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL2, 0),
	MUX(ADSP_SC5XX_CLK_CDU_SPDIF, "cdu_spdif", cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC598_CLK_CDU_SPI, "cdu_spi", cdu_spi_parents, CDU_CLKO6, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_GIGE, "cdu_gige", cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC598_CLK_CDU_LP, "cdu_lp", cdu_lp_parents, CDU_CLKO8, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC598_CLK_CDU_LPDDR, "cdu_lpddr", cdu_lpddr_parents, CDU_CLKO9, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC598_CLK_CDU_TRACE, "cdu_trace", cdu_trace_parents, CDU_CLKO12, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC598_CLK_CDU_EMMC, "cdu_emmc", cdu_emmc_parents, CDU_CLKO13, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC598_CLK_CDU_EMMC_TIMER_QMC, "cdu_emmc_timer", cdu_emmc_timer_parents, CDU_CLKO14, CDU_CLKO_SEL2, 0),
	MUX(ADSP_SC598_CLK_CDU_OSPI_REFCLK, "cdu_ospi_refclk", cdu_ospi_refclk_parents, CDU_CLKO10, CDU_CLKO_SEL4, 0),
};

/* ADSP-SC598 CGU0 divider clocks */
static const struct sc5xx_div_clock sc598_div_clks_cgu0[] = {
	DIV(ADSP_SC5XX_CLK_CGU_CSEL_DIV, "cgu0_csel_div", pllclk_parent, 0, 0, 5),
	DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV, "cgu0_syssel_div", pllclk_parent, 0, 8, 5),
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV, "cgu0_dsel_div", pllclk_parent, 0, 16, 5),
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV, "cgu0_osel_div", pllclk_parent, 0, 22, 7),
	DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV, "cgu0_s0sel_div", syssel_parent, 0, 5, 3),
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV, "cgu0_s1sel_div", syssel_parent, 0, 13, 3),
	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV, "cgu0_s1selex_div", pllclk_parent, 0, 16, 8),
};

/* ADSP-SC5XX PLL clocks */
static const struct sc5xx_pll_clock sc598_pll_clks[] = {
	PLL(ADSP_SC598_CLK_CGU_VCO_OUT, "pll_vco_out", vco_parent, CGU_CTL, 8, 7, 0, true),
};

static const struct sc5xx_fixed_clock sc598_fixed_clks_cgu0[] = {
	FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK
	FFACTOR(ADSP_SC598_CLK_CGU_CCLK2
	FFACTOR(ADSP_SC598_CLK_CGU_DCLK_HALF
};

/* ADSP-SC5XX CGU1 PLL divider clocks */
static const struct sc5xx_div_clock sc5xx_cgu1_div_clks_pll[] = {
	DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu1_df_div", cgu1_parents, CGU_CTL, 0, 1),
};

/* ADSP-SC598 CGU1 divider clocks */
static const struct sc5xx_div_clock sc598_div_clks_cgu1[] = {

};

/**
 * This function will register minimal CGU0 clocks for Cortex-A5 
 * based SoCs such as the SC589, SC573 and SC594 due to early 
 * clock timer requirements:
 *
 * [DF clock] -> [VCO clock] -> [PLLCLK clock] -> [SYSSEL clock] -> [S0SEL clock]
 */
static void __init sc5xx_early_clock_probe(struct device_node *np)
{

}
CLK_OF_DECLARE_DRIVER(sc589_early_clk, "adi,sc589-cgu0", sc5xx_early_clock_probe);
CLK_OF_DECLARE_DRIVER(sc573_early_clk, "adi,sc573-cgu0", sc5xx_early_clock_probe);
CLK_OF_DECLARE_DRIVER(sc594_early_clk, "adi,sc594-cgu0", sc5xx_early_clock_probe);

static int sc5xx_clock_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        const struct sc5xx_clock_data *clk_data;
        void __iomem *reg;

        clk_data = device_get_match_data(dev);
        if (!clk_data)
                return dev_err_probe(dev, -ENODEV, "missing match data\n");

        reg = devm_platform_ioremap_resource(pdev, 0);
        if (IS_ERR(reg))
                return dev_err_probe(dev, PTR_ERR(reg),
                                     "failed to map clock registers\n");

	return sc5xx_register_clocks(dev, reg, clk_data);
}

static void sc5xx_clock_remove(struct platform_device *pdev)
{
}

static const struct of_device_id sc5xx_clock_of_match[] = {
	{ .compatible = "adi,sc598-cgu0", .data = &sc598_cgu0_info, },
	{ .compatible = "adi,sc598-cgu1", .data = &sc598_cgu1_info, },
	{ .compatible = "adi,sc598-pll2", .data = &sc598_pll2_info, },
	{ .compatible = "adi,sc598-cdu",  .data = &sc598_cdu_info,  },
	{ .compatible = "adi,sc589-cgu0", .data = &sc589_cgu0_info, },
	{ .compatible = "adi,sc589-cgu1", .data = &sc589_cgu1_info, },
	{ .compatible = "adi,sc589-cdu",  .data = &sc589_cdu_info,  },
	{ .compatible = "adi,sc573-cgu0", .data = &sc573_cgu0_info, },
	{ .compatible = "adi,sc573-cgu1", .data = &sc573_cgu1_info, }, 
	{ .compatible = "adi,sc573-cdu",  .data = &sc573_cdu_info,  }, 
	{ .compatible = "adi,sc594-cgu0", .data = &sc594_cgu0_info, }, 
	{ .compatible = "adi,sc594-cgu1", .data = &sc594_cgu1_info, }, 
	{ .compatible = "adi,sc594-cdu",  .data = &sc594_cdu_info,  }, 
	{ },
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
