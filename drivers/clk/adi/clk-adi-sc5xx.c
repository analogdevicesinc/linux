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

#define HW_PARENTS(_name)	\
	static const u8 _name[]

#define MUX_TABLE(_name)	\
	static const u32 _name[]

/* Number of clocks per clock controller for each SoC */
#define NUM_CLKS_SC594_CGU0	(ADSP_SC594_CLK_CGU_CCLK1 + 1)
#define NUM_CLKS_SC594_CGU1	(ADSP_SC594_CLK_CGU_CCLK1 + 1)
#define NUM_CLKS_SC594_CDU	(ADSP_SC594_CLK_CDU_TRACE + 1)
#define NUM_CLKS_SC598_CGU0	(ADSP_SC598_CLK_CGU_S1SEL + 1)
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

/* CDU CLKINSEL parents */
CLK_PARENT_DATA(cdu_clkinsel_parents)		= { { .fw_name = "sys_clkin0" }, { .fw_name = "sys_clkin1" }, };

/* ADSP-SC598 mux parents */
CLK_PARENT_DATA(sc598_cdu_sharc0_parents)	= { { .fw_name = "cclk0_0"   }, };
CLK_PARENT_DATA(sc598_cdu_sharc1_parents)	= { { .fw_name = "cclk0_0"   }, };
CLK_PARENT_DATA(sc598_cdu_can_parents)		= { { .fw_name = "oclk_1"    }, };
CLK_PARENT_DATA(sc598_cdu_spdif_parents)	= { { .fw_name = "sclk1_0"   }, };
CLK_PARENT_DATA(sc598_cdu_trace_parents)	= { { .fw_name = "sclk0_0"   }, };
CLK_PARENT_DATA(sc598_cdu_emmc_timer_parents)	= { { .fw_name = "sclk1_1/2" }, };
CLK_PARENT_DATA(sc598_cdu_arm_parents)	 	= { { .fw_name = "cclk2_0"   }, { .fw_name = "cclk2_1" }, };
CLK_PARENT_DATA(sc598_cdu_ddr_parents)	 	= { { .fw_name = "dclk_0"    }, { .fw_name = "dclk_1"  }, };
CLK_PARENT_DATA(sc598_cdu_spi_parents)	  	= { { .fw_name = "sclk0_0"   }, { .fw_name = "oclk_0"  }, };
CLK_PARENT_DATA(sc598_cdu_gige_parents)		= { { .fw_name = "sclk0_0"   }, { .fw_name = "sclk0_1" }, { .fw_name = "oclk_0"   }, };
CLK_PARENT_DATA(sc598_cdu_lp_parents)		= { { .fw_name = "oclk_0"    }, { .fw_name = "sclk0_0" }, { .fw_name = "cclk0_1"  }, };
CLK_PARENT_DATA(sc598_cdu_lpddr_parents)	= { { .fw_name = "oclk_0"    }, { .fw_name = "dclk_0"  }, { .fw_name = "sysclk_1" }, };
CLK_PARENT_DATA(sc598_cdu_ospi_refclk_parents) 	= { { .fw_name = "sysclk_0"  }, { .fw_name = "sclk0_0" }, { .fw_name = "sclk1_1"  }, };
CLK_PARENT_DATA(sc598_cdu_emmc_parents)	 	= { { .fw_name = "oclk_0"    }, { .fw_name = "sclk0_1" }, { .fw_name = "dclk_0/2" }, 
						    { .fw_name = "dclk_1/2"  }, };
/* ADSP-SC594 mux parents */
CLK_PARENT_DATA(sc594_cdu_sharc0_parents) 	= { { .fw_name = "cclk0_0"  }, };
CLK_PARENT_DATA(sc594_cdu_sharc1_parents)	= { { .fw_name = "cclk0_0"  }, };
CLK_PARENT_DATA(sc594_cdu_arm_parents) 		= { { .fw_name = "cclk1_0"  }, };
CLK_PARENT_DATA(sc594_cdu_spdif_parents)	= { { .fw_name = "sclk1_0"  }, };
CLK_PARENT_DATA(sc594_cdu_trace_parents)	= { { .fw_name = "sclk0_0"  }, }; 
CLK_PARENT_DATA(sc594_cdu_can_parents)		= { { .fw_name = "oclk_0"   }, { .fw_name = "oclk_1"  }, };
CLK_PARENT_DATA(sc594_cdu_ddr_parents)		= { { .fw_name = "dclk_0"   }, { .fw_name = "dclk_1"  }, };
CLK_PARENT_DATA(sc594_cdu_spi_parents)		= { { .fw_name = "sclk0_0"  }, { .fw_name = "oclk_0"  }, }; 
CLK_PARENT_DATA(sc594_cdu_gige_parents)		= { { .fw_name = "sclk0_0"  }, { .fw_name = "sclk0_1" }, }; 
CLK_PARENT_DATA(sc594_cdu_lp_parents)		= { { .fw_name = "oclk_0"   }, { .fw_name = "sclk0_0" }, { .fw_name = "cclk0_1"  }, };
CLK_PARENT_DATA(sc594_cdu_lpddr_parents)	= { { .fw_name = "oclk_0"   }, { .fw_name = "dclk_0"  }, { .fw_name = "sysclk_1" }, };
CLK_PARENT_DATA(sc594_cdu_ospi_refclk_parents)	= { { .fw_name = "sysclk_0" }, { .fw_name = "sclk0_0" }, { .fw_name = "sclk1_1"  }, }; 

/* ADSP-SC589 mux parents */
CLK_PARENT_DATA(sc589_cdu_sharc0_parents)	= { { .fw_name = "cclk0_0"  }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc589_cdu_sharc1_parents)	= { { .fw_name = "cclk0_0"  }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc589_cdu_arm_parents)	 	= { { .fw_name = "cclk1_0"  }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc589_cdu_ddr_parents)    	= { { .fw_name = "dclk_0"   }, { .fw_name = "dclk_1"    }, };
CLK_PARENT_DARA(sc589_cdu_reserved_parents)	= { { .fw_name = "oclk_0"   }, { .fw_name = "cclk0_1"   }, };
CLK_PARENT_DATA(sc589_cdu_can_parents)    	= { { .fw_name = "oclk_0"   }, { .fw_name = "oclk_1"    }, { .fw_name = "dclk_1"  }, };
CLK_PARENT_DATA(sc589_cdu_spdif_parents) 	= { { .fw_name = "oclk_0"   }, { .fw_name = "oclk_1"    }, { .fw_name = "dclk_1"  }, { .fw_name = "dclk_0" }, };
CLK_PARENT_DATA(sc589_cdu_gige_parents) 	= { { .fw_name = "sclk1_0"  }, { .fw_name = "sclk1_1"   }, { .fw_name = "cclk0_1" }, { .fw_name = "oclk_0" }, };
CLK_PARENT_DATA(sc589_cdu_lp_parents) 		= { { .fw_name = "sclk0_0"  }, { .fw_name = "sclk0_1"   }, { .fw_name = "cclk1_1" }, { .fw_name = "dclk_1" }, };
CLK_PARENT_DATA(sc589_cdu_sdio_parents) 	= { { .fw_name = "oclk_0/2" }, { .fw_name = "cclk1_1/2" }, { .fw_name = "cclk1_1" }, { .fw_name = "dclk_1" }, };

/* ADSP-SC573 mux parents */
CLK_PARENT_DATA(sc573_cdu_sharc0_parents) 	= { { .fw_name = "cclk0_0"  }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc573_cdu_sharc1_parents) 	= { { .fw_name = "cclk0_0"  }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc573_cdu_arm_parents)		= { { .fw_name = "cclk1_0"  }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc573_cdu_ddr_parents)		= { { .fw_name = "dclk_0"   }, { .fw_name = "dclk_1"    }, };
CLK_PARENT_DATA(sc573_cdu_can_parents)	 	= { { .fw_name = "oclk_0"   }, { .fw_name = "oclk_1"    }, { .fw_name = "dclk_1"  }, { .fw_name = "oclk_0/2" }, };
CLK_PARENT_DATA(sc573_cdu_spdif_parents)	= { { .fw_name = "oclk_0"   }, { .fw_name = "oclk_1"    }, { .fw_name = "dclk_1"  }, { .fw_name = "dclk_0"   }, };
CLK_PARENT_DATA(sc573_cdu_gige_parents)		= { { .fw_name = "sclk1_0"  }, { .fw_name = "sclk1_1"   }, { .fw_name = "cclk0_1" }, { .fw_name = "oclk_0"   }, };
CLK_PARENT_DATA(sc573_cdu_sdio_parents)		= { { .fw_name = "oclk_0/2" }, { .fw_name = "cclk1_1/2" }, { .fw_name = "cclk1_1" }, { .fw_name = "dclk_1"   }, };

/* ADSP-SC5XX CGU0 and CGU1 parent clocks */
CLK_PARENT_DATA(cgu0_parents)	= { { .fw_name = "sys_clkin0"   }, };
CLK_PARENT_DATA(cgu1_parents)	= { { .fw_name = "cdu_clkinsel" }, };

/* ADSP-SC5XX HW parent refs */
HW_PARENTS(syssel_parent)	= { ADSP_SC5XX_CLK_CGU_SYSSEL_DIV };
HW_PARENTS(pllclk_parent)	= { ADSP_SC5XX_CLK_CGU_PLLCLK };
HW_PARENTS(df_parent)		= { ADSP_SC5XX_CLK_CGU_DF_DIV };
HW_PARENTS(vco_parent)		= { ADSP_SC5XX_CLK_CGU_VCO_OUT };
HW_PARENTS(s0sel_div_parent)	= { ADSP_SC5XX_CLK_CGU_S0SEL_DIV };
HW_PARENTS(oclk_gate_parent)	= { ADSP_SC5XX_CLK_CGU_OCLK };

/* ADSP-SC5XX CLKINSEL mux for CGU1 and PLL3 */
static const struct sc5xx_clkinsel_clock sc5xx_clkinsel_clks[] = {
	CLKINSEL(ADSP_SC5XX_CLK_CDU_CLKINSEL, "cdu_clkinsel",
		 cdu_clkinsel_parents, 0),
};

/*----------------------*/

/* Early clocks for ADSP-SC589, ADSP-SC594, ADSP-SC573 CGU0 */
static const struct sc5xx_clk sc5xx_early_clks_cgu0[] = {
        DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu0_df_div",
	    cgu0_parents, CGU_CTL, 0, 1),
        PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT, "cgu0_vco",
	    df_parent, CGU_CTL, 8, 7, 0, false),
        FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK, "cgu0_pllclk",
		vco_parent, 1, 1, 0),
        DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV, "cgu0_syssel_div",
	    pllclk_parent, CGU_DIV, 8, 5),
        DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV, "cgu0_s0sel_div",
	    syssel_parent, CGU_DIV, 5, 3),
        GATE(ADSP_SC5XX_CLK_CGU_SCLK0, "cgu0_sclk0_gate",
	     s0sel_div_parent, CGU_SCBF_DIS, 0, 0),
};

/* Early clocks for ADSP-SC598 CGU0 */
static const struct sc5xx_clk sc598_early_clks_cgu0[] = {
        DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu0_df_div",
	    cgu0_parents, CGU_CTL, 0, 1),
        PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT, "cgu0_vco",
	    df_parent, CGU_CTL, 8, 7, 0, true),
        FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK, "cgu0_pllclk",
		vco_parent, 1, 2, 0),
        DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV, "cgu0_syssel_div",
	    pllclk_parent, CGU_DIV, 8, 5),
        DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV, "cgu0_s0sel_div",
	    syssel_parent, CGU_DIV, 5, 3),
        GATE(ADSP_SC5XX_CLK_CGU_SCLK0, "cgu0_sclk0_gate",
	     s0sel_div_parent, CGU_SCBF_DIS, 0, 0),
};

/* ADSP-SC598 CGU0 clocks */
static const struct sc5xx_clk sc598_cgu0_clks[] = {
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV, 
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV
	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV
	GATE(ADSP_SC5XX_CLK_CGU_CCLK0_GATE
	GATE(ADSP_SC5XX_CLK_CGU_DCLK_GATE
	GATEE(ADSP_SC5XX_CLK_CGU_OCLK_GATE
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1_GATE
	FFACTORADSP_SC598_CLK_CGU_DCLK_HALF
	FFACTOR(ADSP_SC598_CLK_CGU_CCLK2
	MUX(ADSP_SC598_CLK_CGU_S1SEL
};

/* ADSP-SC598 CGU1 clocks */
static const struct sc5xx_clk sc598_cgu1_clks[] = {
	DIV(ADSP_SC5XX_CLK_CGU_DF_DIV,
	PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT,
	FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK,
	DIV(ADSP_SC5XX_CLK_CGU_CSEL_DIV,
	DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV,
	DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV,
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV,
	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV,
	DIV(ADSP_SC598_CLK_CGU_S0SELEX_DIV,
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV, 
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV, 
	GATE(ADSP_SC5XX_CLK_CGU_CCLK0_GATE, 
	GATE(ADSP_SC5XX_CLK_CGU_DCLK_GATE, 
	GATE(ADSP_SC5XX_CLK_CGU_OCLK_GATE, 
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1_GATE, 
	GATE(ADSP_SC5XX_CLK_CGU_SCLK0_GATE, 
	FFACTOR(ADSP_SC598_CLK_CGU_DCLK_HALF, 
	FFACTOR(ADSP_SC598_CLK_CGU_CCLK2, 
	FFACTOR(ADSP_SC598_CLK_CGU_SCLK_HALF, 
	MUX(ADSP_SC598_CLK_CGU_S0SEL, 
	MUX(ADSP_SC598_CLK_CGU_S1SEL, 
};

/* ADSP-SC589 PLL2 clocks */
static const struct sc5xx_clk sc598_pll2_clks[] = {
	DIV(ADSP_SC598_CLK_PLL2_DF_DIV
	PLL(ADSP_SC598_CLK_PLL2_VCO_OUT
	FFACTOR(ADSP_SC598_CLK_PLL2_PLLCLK
	DIV(ADSP_SC598_CLK_PLL2_DDIV
	MUX(ADSP_SC598_CLK_DDR
};



/*----------------------*/


/* ADSP-SC5XX CDU mux clocks */
static const struct sc5xx_mux_clock sc598_mux_clks[] = {
	MUX(ADSP_SC5XX_CLK_CDU_DDR, "cdu_ddr",
	    sc598_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_CAN, "cdu_can",
	    sc598_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL2, 0),
	MUX(ADSP_SC5XX_CLK_CDU_SPDIF, "cdu_spdif",
	    sc598_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC598_CLK_CDU_SPI, "cdu_spi",
	    sc598_cdu_spi_parents, CDU_CLKO6, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_GIGE, "cdu_gige",
	    sc598_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC598_CLK_CDU_LP, "cdu_lp",
	    sc598_cdu_lp_parents, CDU_CLKO8, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC598_CLK_CDU_LPDDR, "cdu_lpddr",
	    sc598_cdu_lpddr_parents, CDU_CLKO9, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC598_CLK_CDU_TRACE, "cdu_trace",
	    sc598_cdu_trace_parents, CDU_CLKO12, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC598_CLK_CDU_EMMC, "cdu_emmc",
	    sc598_cdu_emmc_parents, CDU_CLKO13, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC598_CLK_CDU_EMMC_TIMER_QMC, "cdu_emmc_timer",
	    sc598_cdu_emmc_timer_parents, CDU_CLKO14, CDU_CLKO_SEL2, 0),
	MUX(ADSP_SC598_CLK_CDU_OSPI_REFCLK, "cdu_ospi_refclk",
	    sc598_cdu_ospi_refclk_parents, CDU_CLKO10, CDU_CLKO_SEL4, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC0, "cdu_sharc0",
	     sc598_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC1, "cdu_sharc1",
	     sc598_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_ARM, "cdu_arm",
	     sc598_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL5, 0),
};

static const struct sc5xx_mux_clock sc594_mux_clks[] = {
	MUX(ADSP_SC5XX_CLK_CDU_DDR, "cdu_ddr",
	    sc594_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_CAN, "cdu_can",
	    sc594_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_SPDIF, "cdu_spdif",
	    sc594_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC594_CLK_CDU_SPI, "cdu_spi",
	    sc594_cdu_spi_parents, CDU_CLKO6, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_GIGE, "cdu_gige",
	    sc594_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC594_CLK_CDU_LP, "cdu_lp",
	    sc594_cdu_lp_parents, CDU_CLKO8, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC594_CLK_CDU_LPDDR, "cdu_lpddr",
	    sc594_cdu_lpddr_parents, CDU_CLKO9, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC594_CLK_CDU_TRACE, "cdu_trace",
	    sc594_cdu_trace_parents, CDU_CLKO12, CDU_CLKO_SEL1, 0),
	MUX(ADSP_SC594_CLK_CDU_OSPI_REFCLK, "cdu_ospi_refclk",
	    sc594_cdu_ospi_refclk_parents, CDU_CLKO10, CDU_CLKO_SEL4, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC0, "cdu_sharc0",
	     sc594_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC1, "cdu_sharc1",
	     sc594_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL1, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_ARM, "cdu_arm",
	     sc594_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL1, 0),
};

static const struct sc5xx_mux_clock sc589_mux_clks[] = {
	MUX(ADSP_SC5XX_CLK_CDU_DDR, "cdu_ddr",
	    sc589_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_CAN, "cdu_can",
	    sc589_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL4, 0),
	MUX(ADSP_SC5XX_CLK_CDU_SPDIF, "cdu_spdif",
	    sc589_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC5XX_CLK_CDU_GIGE, "cdu_gige",
	    sc589_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC58X_CLK_CDU_LP, "cdu_lp",
	    sc589_cdu_lp_parents, CDU_CLKO8, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC58X_CLK_CDU_SDIO, "cdu_sdio",
	    sc589_cdu_sdio_parents, CDU_CLKO9, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC58X_CLK_CDU_RESERVED, "cdu_reserved",
	    sc589_cdu_reserved_parents, CDU_CLKO6, CDU_CLKO_SEL3, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC0, "cdu_sharc0",
	     sc589_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL3, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC1, "cdu_sharc1",
	     sc589_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL3, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_ARM, "cdu_arm",
	     sc589_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL3, 0),
};

static const struct sc5xx_mux_clock sc573_mux_clks[] = {
	MUX(ADSP_SC5XX_CLK_CDU_DDR, "cdu_ddr",
	    sc573_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	MUX(ADSP_SC5XX_CLK_CDU_CAN, "cdu_can",
	    sc573_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC5XX_CLK_CDU_SPDIF, "cdu_spdif",
	    sc573_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC5XX_CLK_CDU_GIGE, "cdu_gige",
	    sc573_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL6, 0),
	MUX(ADSP_SC57X_CLK_CDU_SDIO, "cdu_sdio",
	    sc573_cdu_sdio_parents, CDU_CLKO9, CDU_CLKO_SEL6, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC0, "cdu_sharc0",
	     sc573_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL3, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_SHARC1, "cdu_sharc1",
	     sc573_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL3, 0),
	CMUX(ADSP_SC5XX_CLK_CDU_ARM, "cdu_arm",
	     sc573_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL3, 0),
};

/*--- common DIVIDER clocks ---*/

/* ADSP-SC5XX common divider clocks for CGU0 */
static const struct sc5xx_div_clock common_sc5xx_div_clks_cgu0[] = {
	DIV(ADSP_SC5XX_CLK_CGU_CSEL_DIV, "cgu0_csel_div", pllclk_parent, 0, 0, 5),
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV, "cgu0_dsel_div", pllclk_parent, 0, 16, 5),
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV, "cgu0_osel_div", pllclk_parent, 0, 22, 7),
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV, "cgu0_s1sel_div", syssel_parent, 0, 13, 3),
};

/* ADSP-SC5XX common divider clocks for CGU1 */
static const struct sc5xx_div_clock common_sc5xx_div_clks_cgu1[] = {
	DIV(ADSP_SC5XX_CLK_CGU_CSEL_DIV, "cgu1_csel_div", pllclk_parent, 0, 0, 5),
	DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV, "cgu1_syssel_div", pllclk_parent, 0, 8, 5),
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV, "cgu1_dsel_div", pllclk_parent, 0, 16, 5),
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV, "cgu1_osel_div", pllclk_parent, 0, 22, 7),
	DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV, "cgu1_s0sel_div", syssel_parent, 0, 5, 3),
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV, "cgu1_s1sel_div", syssel_parent, 0, 13, 3),
};

/*--- per SoC DIVIDER clocks ---*/

/* ADSP-SC598 extra divider clocks for CGU0 */
static const struct sc5xx_div_clock extra_sc598_div_clks_cgu0[] = {
	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV, "cgu0_s1selex_div", pllclk_parent, CGU_DIVEX, 16, 8),
};

/* ADSP-SC594 extra divider clocks for CGU0 */
static const struct sc5xx_div_clock extra_sc594_div_clks_cgu0[] = {
	DIV(ADSP_SC594_CLK_CGU_S1SELEXDIV, "cgu0_s1selexdiv", pllclk_parent, CGU_DIVEX, 16, 8),
};

/* ADSP-SC598 extra divider clocks for CGU1 */
static const struct sc5xx_div_clock extra_sc598_div_clks_cgu1[] = {
	DIV(ADSP_SC598_CLK_CGU_S0SELEX_DIV, "cgu1_s0selex_div", pllclk_parent, CGU_DIVEX, 0, 8),
	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV, "cgu1_s1selex_div", pllclk_parent, CGU_DIVEX, 16, 8),
};

/* ADSP-SC594 extra divider clocks for CGU1 */
static const struct sc5xx_div_clock extra_sc594_div_clks_cgu1[] = {
	DIV(ADSP_SC594_CLK_CGU_S1SELEXDIV, "cgu1_s1selexdiv", pllclk_parent, CGU_DIVEX, 16, 8),
};

/*--- CGU1 PLL clocks ---*/

/* ADSP-SC594, ADSP-SC589, ADSP-SC573 initial PLL clocks for CGU1 */
static const struct sc5xx_clk sc5xx_a5_cgu1_pll_clks[] = {
	CLK_DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu1_df", cgu1_parents, CGU_CTL, 0, 1),
	CLK_PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT, "cgu1_vco", df_parent, CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, false),
	CLK_FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK, "cgu1_pllclk", vco_parent, 1, 1, CLK_SET_RATE_PARENT),
};

/* ADSP-SC598 initial PLL clocks for CGU1 */
static const struct sc5xx_clk sc598_cgu1_pll_clks[] = {
	CLK_DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu1_df", cgu1_parents, CGU_CTL, 0, 1),
	CLK_PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT, "cgu1_vco", df_parent, CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, true),
	CLK_FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK, "cgu1_pllclk", vco_parent, 1, 2, CLK_SET_RATE_PARENT),
};


/*--- EARLY clocks ---*/

/* ADSP-SC594, ADSP-SC589, ADSP-SC573 early clocks for CGU0 */
static const struct sc5xx_clk sc5xx_a5_early_clks_cgu0[] = {
	CLK_DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu0_df_div", cgu0_parents, CGU_CTL, 0, 1),
	CLK_PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT, "cgu0_vco", df_parent, CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, false),
	CLK_FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK, "cgu0_pllclk", vco_parent, 1, 1, 0),
	CLK_DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV, "cgu0_syssel_div", pllclk_parent, CGU_DIV, 8, 5),
	CLK_DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV, "cgu0_s0sel_div",  syssel_parent, CGU_DIV, 5, 3),
	CLK_GATE(ADSP_SC5XX_CLK_CGU_SCLK0, "cgu0_sclk0_gate", s0sel_div_parent, CGU_SCBF_DIS, 0, 0),
};

/* ADSP-SC598 early clocks for CGU0 */
static const struct sc5xx_clk sc598_early_clks_cgu0[] = {
	CLK_DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu0_df_div", cgu0_parents, CGU_CTL, 0, 1),
	CLK_PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT, "cgu0_vco", df_parent, CGU_CTL, CGU_MSEL_SHIFT, CGU_MSEL_WIDTH, 0, true),
	CLK_FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK, "cgu0_pllclk", vco_parent, 1, 2, 0),
	CLK_DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV, "cgu0_syssel_div", pllclk_parent, CGU_DIV, 8, 5),
	CLK_DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV, "cgu0_s0sel_div", syssel_parent, CGU_DIV, 5, 3),
	CLK_GATE(ADSP_SC5XX_CLK_CGU_SCLK0, "cgu0_sclk0_gate", s0sel_div_parent, CGU_SCBF_DIS, 0, 0),
};


/*--- common CGU GATE clocks ---*/

/* Common SC5XX CGU0 gate clocks */
static const struct sc5xx_gate_clock common_sc5xx_gate_clks_cgu0[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_CCLK0, "cclk0_0", csel_div_parent, CGU_CCBF_DIS, 0, 0),
	GATE(ADSP_SC5XX_CLK_CGU_OCLK, "oclk_0", osel_div_parent, CGU_SCBF_DIS, 3, 0),
	GATE(ADSP_SC5XX_CLK_CGU_DCLK, "dclk_0", dsel_div_parent, CGU_SCBF_DIS, 2, 0),
};

/* Common SC5XX CGU1 gate clocks */
static const struct sc5xx_gate_clock common_sc5xx_gate_clks_cgu1[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_CCLK0, "cclk0_1", csel_div_parent, CGU_CCBF_DIS, 0, 0),
	GATE(ADSP_SC5XX_CLK_CGU_OCLK, "oclk_1", osel_div_parent, CGU_SCBF_DIS, 3, 0),
	GATE(ADSP_SC5XX_CLK_CGU_DCLK, "dclk_1", dsel_div_parent, CGU_SCBF_DIS, 2, 0),
};

/* SC573, SC589 extra gate clocks */
static const struct sc5xx_gate_clock sc57x_sc58x_gate_clks_cgu0[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_CCLK1, "cclk1_0", csel_div_parent, CGU_CCBF_DIS, 1, 0),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1, "sclk1_0", s1sel_div_parent, CGU_SCBF_DIS, 1, 0),
};

static const struct sc5xx_gate_clock sc57x_sc58x_gate_clks_cgu1[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_CCLK1, "cclk1_1", csel_div_parent, CGU_CCBF_DIS, 1, 0),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK0, "sclk0_1", s0sel_div_parent, CGU_SCBF_DIS, 0, 0),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1, "sclk1_1", s1sel_div_parent, CGU_SCBF_DIS, 1, 0),
};

/* SC594 extra gate clocks */
static const struct sc5xx_gate_clock sc594_gate_clks_cgu0[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_CCLK1, "cclk1_0", csel_div_parent, CGU_CCBF_DIS, 1, 0),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1, "sclk1_0", sclk1sel_parent, CGU_SCBF_DIS, 1, 0),
};

static const struct sc5xx_gate_clock sc594_gate_clks_cgu1[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_CCLK1, "cclk1_1", csel_div_parent, CGU_CCBF_DIS, 1, 0),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK0, "sclk0_1", s0sel_div_parent, CGU_SCBF_DIS, 0, 0),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1, "sclk1_1", sclk1sel_parent, CGU_SCBF_DIS, 1, 0),
};

/* SC598 extra gate clocks */
static const struct sc5xx_gate_clock sc598_gate_clks_cgu0[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1, "sclk1_0", sclk1sel_parent, CGU_SCBF_DIS, 1, 0),
};

static const struct sc5xx_gate_clock sc598_gate_clks_cgu1[] __initconst = {
	GATE(ADSP_SC5XX_CLK_CGU_SCLK0, "sclk0_1", sclk0sel_parent, CGU_SCBF_DIS, 0, 0),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1, "sclk1_1", sclk1sel_parent, CGU_SCBF_DIS, 1, 0),
};



/*--- FIXED FACTOR clocks ---*/

/* ADSP-SC57x post-gate fixed-factor clocks for CGU0 */
static const struct sc5xx_fixed_factor_clock sc57x_fixed_factor_clks_cgu0[] __initconst = {
	FFACTOR(ADSP_SC57X_CLK_OCLK0_HALF, "oclk_0_half", oclk_gate_parent, 1, 2, CLK_SET_RATE_PARENT),
};

/* ADSP-SC57x post-gate fixed-factor clocks for CGU1 */
static const struct sc5xx_fixed_factor_clock sc57x_fixed_factor_clks_cgu1[] __initconst = {
	FFACTOR(ADSP_SC57X_CLK_CCLK1_1_HALF, "cclk1_1_half", cclk1_parent, 1, 2, CLK_SET_RATE_PARENT),
};

/* ADSP-SC58x post-gate fixed-factor clocks for CGU0 */
static const struct sc5xx_fixed_factor_clock sc58x_fixed_factor_clks_cgu0[] __initconst = {
	FFACTOR(ADSP_SC58X_CLK_OCLK0_HALF, "oclk_0_half", oclk_parent, 1, 2, CLK_SET_RATE_PARENT),
};

/* ADSP-SC58x post-gate fixed-factor clocks for CGU1 */
static const struct sc5xx_fixed_factor_clock sc58x_fixed_factor_clks_cgu1[] __initconst = {
	FFACTOR(ADSP_SC58X_CLK_CCLK1_1_HALF, "cclk1_1_half", cclk1_parent, 1, 2, CLK_SET_RATE_PARENT),
};

/* ADSP-SC598 VCO-derived fixed-factor clocks for CGU0 */
static const struct sc5xx_fixed_factor_clock sc598_vco_fixed_factor_clks_cgu0[] __initconst = {
	FFACTOR(ADSP_SC598_CLK_CGU0_CCLK2, "cclk2_0", vco_parent, 1, 3, CLK_SET_RATE_PARENT),
};

/* ADSP-SC598 VCO-derived fixed-factor clocks for CGU1 */
static const struct sc5xx_fixed_factor_clock sc598_vco_fixed_factor_clks_cgu1[] __initconst = {
	FFACTOR(ADSP_SC598_CLK_CGU1_CCLK2, "cclk2_1", vco_parent, 1, 3, CLK_SET_RATE_PARENT),
};

/* ADSP-SC598 post-gate fixed-factor clocks for CGU0 */
static const struct sc5xx_fixed_factor_clock sc598_post_gate_fixed_factor_clks_cgu0[] __initconst = {
	FFACTOR(ADSP_SC598_CLK_DCLK0_HALF, "dclk_0_half", dclk_parent, 1, 2, CLK_SET_RATE_PARENT),
};

/* ADSP-SC598 post-gate fixed-factor clocks for CGU1 */
static const struct sc5xx_fixed_factor_clock sc598_post_gate_fixed_factor_clks_cgu1[] __initconst = {
	FFACTOR(ADSP_SC598_CLK_DCLK1_HALF, "dclk_1_half", dclk_parent, 1, 2, CLK_SET_RATE_PARENT),
	FFACTOR(ADSP_SC598_CLK_CGU1_SCLK1_HALF, "sclk1_1_half", sclk1_parent, 1, 2, CLK_SET_RATE_PARENT),
};


/*--- CGU SELEXDIV mux clocks ---*/

/* SC594 has SELEX muxes for CGU0/CGU1 SCLK1 */
static const struct sc5xx_mux_clock sc594_mux_clks_cgu0[] __initconst = {
	MUX(ADSP_SC594_CLK_CGU_S1SEL, "cgu0_sclk1sel", cgu0_s1sel_parents, CGU_CTL, 17, 1, CLK_SET_RATE_PARENT),
};

static const struct sc5xx_mux_clock sc594_mux_clks_cgu1[] __initconst = {
	MUX(ADSP_SC594_CLK_CGU_S1SEL, "cgu1_sclk1sel", cgu1_s1sel_parents, CGU_CTL, 17, 1, CLK_SET_RATE_PARENT),
};

/* SC598 has SELEX muxes for CGU0 SCLK1 and CGU1 SCLK0/SCLK1. */
static const struct sc5xx_mux_clock sc598_mux_clks_cgu0[] __initconst = {
	MUX(ADSP_SC598_CLK_CGU_S1SEL, "cgu0_sclk1sel", cgu0_s1sel_parents, CGU_CTL, 17, 1, CLK_SET_RATE_PARENT),
};

static const struct sc5xx_mux_clock sc598_mux_clks_cgu1[] __initconst = {
	MUX(ADSP_SC598_CLK_CGU_S0SEL, "cgu1_sclk0sel", cgu1_s0sel_parents, CGU_CTL, 16, 1, CLK_SET_RATE_PARENT),
	MUX(ADSP_SC598_CLK_CGU_S1SEL, "cgu1_sclk1sel", cgu1_s1sel_parents, CGU_CTL, 17, 1, CLK_SET_RATE_PARENT),
};





/* ADSP-SC5XX CGU1 PLL divider clocks */
static const struct sc5xx_div_clock sc5xx_cgu1_div_clks_pll[] = {
	DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu1_df_div", cgu1_parents, CGU_CTL, 0, 1),
};

/**
 * Register minimal CGU0 clocks for SC594, SC589, SC573
 * and SC598 SoCs to satisfy early clock timer requirements
 * but clear OF_POPULATED for the corresponding node.
 *
 * The early CGU0 clocks being registered are:
 * [DF clock] -> [VCO clock] -> [PLLCLK clock] -> 
 * 	[SYSSEL clock] -> [S0SEL clock] -> [SCLK0 gate]
 */
static void __init sc5xx_register_early_clocks(struct device_node *np, 
						const struct sc5xx_early_clk *clk_data,
						unsigned int num_early_clks,
						unsigned int num_provider_clks)
{
	void __iomem *reg_base;

	reg_base = of_iomap(np, 0);
	if (!reg_base) {
		pr_err("%pOF: failed to map clock registers\n", np);
		return;
	}

	
	
}

static void __init sc573_early_clock_probe(struct device_node *np)
{
	sc5xx_register_early_clocks(np, sc5xx_a5_early_clks,
				    ARRAY_SIZE(sc5xx_a5_early_clks),
				    NUM_CLKS_SC57X_CGU0);
}
CLK_OF_DECLARE_DRIVER(sc573_early_clk, "adi,sc573-cgu0", sc573_early_clock_probe);

static void __init sc589_early_clock_probe(struct device_node *np)
{
	sc5xx_register_early_clocks(np, sc5xx_a5_early_clks,
				    ARRAY_SIZE(sc5xx_a5_early_clks),
				    NUM_CLKS_SC58X_CGU0);
}
CLK_OF_DECLARE_DRIVER(sc589_early_clk, "adi,sc589-cgu0", sc589_early_clock_probe);

static void __init sc594_early_clock_probe(struct device_node *np)
{
	sc5xx_register_early_clocks(np, sc5xx_a5_early_clks,
				    ARRAY_SIZE(sc5xx_a5_early_clks),
				    NUM_CLKS_SC594_CGU0);
}
CLK_OF_DECLARE_DRIVER(sc594_early_clk, "adi,sc594-cgu0", sc594_early_clock_probe);

static void __init sc598_early_clock_probe(struct device_node *np)
{
	sc5xx_register_early_clocks(np, sc598_early_clks,
				    ARRAY_SIZE(sc598_early_clks),
				    NUM_CLKS_SC598_CGU0);
}
CLK_OF_DECLARE_DRIVER(sc598_early_clk, "adi,sc598-cgu0", sc598_early_clock_probe);

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
