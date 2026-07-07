// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock driver for ADSP-SC5xx SoCs
 *
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * Author: Qasim Ijaz <qasim.ijaz@analog.com>
 * Contact: linux@analog.com
 *
 * The Analog Devices ADSP-SC5XX SoCs contain a
 * clock architecture that consists of common HW blocks
 * which are resposible for generating, routing
 * and controlling clocks throughout the SoC:
 *
 * The Clock Generation Unit (CGU) includes a PLL which
 * produces a PLL clock. The PLLCLK can then be divided 
 * down via dividers such as CSEL, DSEL, OSEL, etc. to
 * produce various system clocks. These clocks are then
 * routed to the Clock Distribution Unit (CDU) which can
 * clock various pheripherals across the SoC.
 *
 * The Clock Distribution Unit (CDU) consists of an array of 
 * multiplexors that select clocks originated from multiple
 * clock sources. These sources are different clocks
 * that are generated from the Clock Generation Units (CGUs).
 *
 * Root clocks				     CGU0/CGU1		
 *				   +----------------------------+
 *				   |			 ...    |    ...
 * SYS_CLKIN0--------------------> | +---+	         +----+ |--> CCLK -->    CDU
 *	      |			   | |PLL|    	         |CSEL| |    ...      +--------+
 *	      +--> +----------+    | |   | --> PLLCLK -> +----+ |--> SCLK --> |        | -->
 * SYS_CLKIN1----> | CLKINSEL |--> | +---+         	 +----+ |    ...      |        | -->
 * 		   +----------+    |		  	 |DSEL| |--> DCLK --> |        | -->
 * 		   		   |			 +----+ |    ...      +--------+	 
 * 		   		   |			 ...    |--> OCLK -->
 * 		    	           +----------------------------+    ...
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

static DEFINE_SPINLOCK(sc5xx_clk_lock);

#define CLK_PARENT_DATA(_name)	\
	static const struct clk_parent_data _name[]

#define CLK_PARENT_HW(_name)	\
	static const unsigned int _name[]

#define MUX_TABLE(_name)	\
	static const u32 _name[]

/* Number of clocks per clock controller for each SoC */
#define NUM_CLKS_SC594_CGU0	(ADSP_SC594_CLK_CGU_CCLK1 + 1)
#define NUM_CLKS_SC594_CGU1	(ADSP_SC594_CLK_CGU_CCLK1 + 1)
#define NUM_CLKS_SC594_CDU	(ADSP_SC594_CLK_CDU_TRACE + 1)
#define NUM_CLKS_SC598_CGU0	(ADSP_SC598_CLK_CGU_S1SELEXEN + 1)
#define NUM_CLKS_SC598_CGU1	(ADSP_SC598_CLK_CGU_S0SELEXEN + 1)
#define NUM_CLKS_SC598_CDU	(ADSP_SC598_CLK_CDU_TRACE + 1)
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
CLK_PARENT_DATA(sc589_cdu_reserved_parents)	= { { .fw_name = "oclk_0"   }, { .fw_name = "cclk0_1"   }, };
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

/* ADSP-SC598 PLL2 parents */
CLK_PARENT_DATA(pll2_ddr_parents)		= { { .fw_name = "cdu_ddr"  }, { .name = "3pll_ddiv"    }, };

/* ADSP-SC5XX CGU0 and CGU1 parent clocks */
CLK_PARENT_DATA(cgu0_parents)			= { { .fw_name = "sys_clkin0"   }, };
CLK_PARENT_DATA(cgu1_parents)			= { { .fw_name = "cdu_clkinsel" }, };

/* ADSP-SC5XX HW parent refs */
CLK_PARENT_HW(df_parent)			= { ADSP_SC5XX_CLK_CGU_DF_DIV     };
CLK_PARENT_HW(osel_parent)			= { ADSP_SC5XX_CLK_CGU_OSEL_DIV   };
CLK_PARENT_HW(dsel_parent)			= { ADSP_SC5XX_CLK_CGU_DSEL_DIV   };
CLK_PARENT_HW(csel_parent)			= { ADSP_SC5XX_CLK_CGU_CSEL_DIV   };
CLK_PARENT_HW(syssel_parent)			= { ADSP_SC5XX_CLK_CGU_SYSSEL_DIV };
CLK_PARENT_HW(s0sel_div_parent)			= { ADSP_SC5XX_CLK_CGU_S0SEL_DIV  };
CLK_PARENT_HW(pllclk_parent)	 		= { ADSP_SC5XX_CLK_CGU_PLLCLK     };
CLK_PARENT_HW(vco_parent)			= { ADSP_SC5XX_CLK_CGU_VCO_OUT    };
CLK_PARENT_HW(oclk_gate_parent)			= { ADSP_SC5XX_CLK_CGU_OCLK_GATE  };
CLK_PARENT_HW(dclk_gate_parent)			= { ADSP_SC5XX_CLK_CGU_DCLK_GATE  };
CLK_PARENT_HW(sclk1_gate_parent)		= { ADSP_SC5XX_CLK_CGU_SCLK1_GATE };
CLK_PARENT_HW(pll2_df_parent)			= { ADSP_SC598_CLK_PLL2_DF_DIV    };
CLK_PARENT_HW(pll2_vco_parent) 			= { ADSP_SC598_CLK_PLL2_VCO_OUT   };
CLK_PARENT_HW(pll2_pllclk_parent)		= { ADSP_SC598_CLK_PLL2_PLLCLK    };
CLK_PARENT_HW(s1selexen_parent)			= { ADSP_SC598_CLK_CGU_S1SELEXEN  };
CLK_PARENT_HW(s0selexen_parent)			= { ADSP_SC598_CLK_CGU_S0SELEXEN  };
CLK_PARENT_HW(s1sel_s1selex_parent) 		= { ADSP_SC5XX_CLK_CGU_S1SEL_DIV, ADSP_SC598_CLK_CGU_S1SELEX_DIV };
CLK_PARENT_HW(s0sel_s0selex_parent)		= { ADSP_SC5XX_CLK_CGU_S0SEL_DIV, ADSP_SC598_CLK_CGU_S0SELEX_DIV };

/* ADSP-SC5XX PLL root clocks */
DIV(ADSP_SC5XX_CLK_CGU_DF_DIV; "cgu0_df_div", cgu0_parents, 0, 1);
DIV(ADSP_SC5XX_CLK_CGU_DF_DIV, "cgu1_df_div", cgu1_parents, 0, 1);


/* ADSP-SC5XX CLKINSEL mux for CGU1 and PLL3 */
static const struct sc5xx_clk sc5xx_clkinsel_clks[] = {
	CLKINSEL(ADSP_SC5XX_CLK_CDU_CLKINSEL,
		"cdu_clkinsel", SC5XX_PARENT_DATA(cdu_clkinsel_parents), 0),
};

/*----------------------*/

/* Early clocks for ADSP-SC589, ADSP-SC594, ADSP-SC573 CGU0 */
static const struct sc5xx_clk sc5xx_early_clks_cgu0[] = {
        PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT,
		"cgu0_vco", df_parent, 8, 7, 0, false),
        FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK,
		"cgu0_pllclk", vco_parent, 1, 1, 0),
	DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV,
		"cgu0_syssel_div", pllclk_parent, 8, 5),
        DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV,
		"cgu0_s0sel_div", syssel_parent, 5, 3),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK0_GATE,
		"cgu0_sclk0_gate", s0sel_div_parent, 0, 0),
};

/* Early clocks for ADSP-SC598 CGU0 */
static const struct sc5xx_clk sc598_early_clks_cgu0[] = {
        PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT,
		"cgu0_vco", df_parent, 8, 7, 0, true),        
	FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK,
		"cgu0_pllclk", vco_parent, 1, 2, 0),
	DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV,
		"cgu0_syssel_div", pllclk_parent, 8, 5),
        DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV,
		"cgu0_s0sel_div", syssel_parent, 5, 3),
        GATE(ADSP_SC5XX_CLK_CGU_SCLK0_GATE,
		"cgu0_sclk0_gate", s0sel_div_parent, 0, 0),
};

/* ADSP-SC598 CGU0 clocks */
static const struct sc5xx_clk sc598_cgu0_clks[] = {
	DIV(ADSP_SC5XX_CLK_CGU_CSEL_DIV,
		"cgu0_cdiv", pllclk_parent, 0, 5),
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV,
		"cgu0_dsel", pllclk_parent, 16, 5),
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV,
		"cgu0_osel", pllclk_parent, 22, 7),
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV,
		"cgu0_s1sel", syssel_parent, 13, 3, 0),
 	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV,
		"cgu0_s1selex_div", pllclk_parent, 16, 8, 0), 
	MUX(ADSP_SC598_CLK_CGU_S1SELEXEN,
		"cgu0_s1selexen", s1sel_s1selex_parent, 2, 17, 1, 0),
	GATE(ADSP_SC5XX_CLK_CGU_CCLK0_GATE,
		"cgu0_cclk0_0", csel_parent, CGU_CCBF_DIS, 0),
	GATE(ADSP_SC5XX_CLK_CGU_DCLK_GATE,
		"cgu0_dclk_0", dsel_parent, CGU_SCBF_DIS, 2),
	GATE(ADSP_SC5XX_CLK_CGU_OCLK_GATE,
		"cgu0_oclk_0", osel_parent, CGU_SCBF_DIS, 3),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1_GATE,
		"cgu0_sclk1_0", s1selexen_parent, CGU_SCBF_DIS, 1),
	FFACTOR(ADSP_SC598_CLK_CGU_DCLK_HALF,
		"cgu0_dclk_0/2", dclk_gate_parent, 1, 2),
	FFACTOR(ADSP_SC598_CLK_CGU_CCLK2,
		"cgu0_cclk2_0", vco_parent, 1, 3),
};

/* ADSP-SC598 CGU1 clocks */
static const struct sc5xx_clk sc598_cgu1_clks[] = {
	PLL(ADSP_SC5XX_CLK_CGU_VCO_OUT,
		"cgu1_vco", df_parent, 8, 7, 0, true),
	FFACTOR(ADSP_SC5XX_CLK_CGU_PLLCLK,
		"cgu1_pllclk", vco_parent, 1, 2),
	DIV(ADSP_SC5XX_CLK_CGU_CSEL_DIV,
		"cgu1_cdiv", pllclk_parent, 0, 5, 0),
	DIV(ADSP_SC5XX_CLK_CGU_OSEL_DIV,
		"cgu1_odiv", pllclk_parent, 22, 7, 0),
	DIV(ADSP_SC5XX_CLK_CGU_DSEL_DIV,
		"cgu1_ddiv", pllclk_parent, 16, 5, 0),
	DIV(ADSP_SC5XX_CLK_CGU_SYSSEL_DIV,
		"cgu1_sysclk_1", pllclk_parent, 8, 5, 0),
	DIV(ADSP_SC5XX_CLK_CGU_S0SEL_DIV,
		"cgu1_s0sel_div", syssel_parent, 5, 3, 0),
	DIV(ADSP_SC5XX_CLK_CGU_S1SEL_DIV,
		"cgu1_s1sel_div", syssel_parent, 13, 3, 0),
	DIV(ADSP_SC598_CLK_CGU_S1SELEX_DIV,
		"cgu1_s1selex_div", pllclk_parent, 16, 8, 0),
	DIV(ADSP_SC598_CLK_CGU_S0SELEX_DIV,
		"cgu1_s0selex_div", pllclk_parent, 0, 8, 0),
	MUX(ADSP_SC598_CLK_CGU_S1SELEXEN,
		"cgu1_sclk1sel", s1sel_s1selex_parent, 2, 17, 1, 0),
	MUX(ADSP_SC598_CLK_CGU_S0SELEXEN,
		"cgu1_sclk0sel", s0sel_s0selex_parent, 2, 16, 1, 0),
	GATE(ADSP_SC5XX_CLK_CGU_CCLK0_GATE,
		"cgu1_cclk0_1", csel_parent, CGU_CCBF_DIS, 0),
	GATE(ADSP_SC5XX_CLK_CGU_DCLK_GATE,
		"cgu1_dclk_1", dsel_parent, CGU_SCBF_DIS, 2),
	GATE(ADSP_SC5XX_CLK_CGU_OCLK_GATE,
		"cgu1_oclk_1", osel_parent, CGU_SCBF_DIS, 3),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK1_GATE,
		"cgu1_sclk1_1", s1selexen_parent, CGU_SCBF_DIS, 1),
	GATE(ADSP_SC5XX_CLK_CGU_SCLK0_GATE,
		"cgu1_sclk0_1", s0selexen_parent, CGU_SCBF_DIS, 0),
	FFACTOR(ADSP_SC598_CLK_CGU_DCLK_HALF,
		"cgu1_dclk_1/2", dclk_gate_parent, 1, 2),
	FFACTOR(ADSP_SC598_CLK_CGU_CCLK2,
		"cgu1_cclk2_1", vco_parent, 1, 3),
	FFACTOR(ADSP_SC598_CLK_CGU_SCLK_HALF,
		"cgu1_sclk1_1/2", sclk1_gate_parent, 1, 2),
};

/* ADSP-SC598 PLL2 clocks */
static const struct sc5xx_clk sc598_pll2_clks[] = {
	DIV(ADSP_SC598_CLK_PLL2_DF_DIV,
		"3pll_df", cgu1_parents, 3, 1, 0),
	PLL(ADSP_SC598_CLK_PLL2_VCO_OUT,
		"3pll_vco", pll2_df_parent, 4, 7, 1, true), 
	FFACTOR(ADSP_SC598_CLK_PLL2_PLLCLK,
		"3pll_pllclk", pll2_vco_parent, 1, 2),
	DIV(ADSP_SC598_CLK_PLL2_DDIV,
		"3pll_ddiv", pll2_pllclk_parent, 12, 5, 0),
	MUX(ADSP_SC598_CLK_DDR,
		"ddr", pll2_ddr_parents, 2, 11, 1, 0),
};

/*----------------------*/

/* ADSP-SC598 CDU mux clocks */
static const struct sc5xx_clk sc598_mux_clks[] = {
	CDU_MUX(ADSP_SC5XX_CLK_CDU_DDR, 
		"cdu_ddr", sc598_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_CAN,
		"cdu_can", sc598_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL2, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_SPDIF,
		"cdu_spdif", sc598_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL1, 0),
	CDU_MUX(ADSP_SC598_CLK_CDU_SPI,
		"cdu_spi", sc598_cdu_spi_parents, CDU_CLKO6, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_GIGE,
		"cdu_gige", sc598_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL4, 0),
	CDU_MUX(ADSP_SC598_CLK_CDU_LP,
		"cdu_lp", sc598_cdu_lp_parents, CDU_CLKO8, CDU_CLKO_SEL4, 0),
	CDU_MUX(ADSP_SC598_CLK_CDU_LPDDR,
		"cdu_lpddr", sc598_cdu_lpddr_parents, CDU_CLKO9, CDU_CLKO_SEL4, 0),
	CDU_MUX(ADSP_SC598_CLK_CDU_TRACE,
		"cdu_trace", sc598_cdu_trace_parents, CDU_CLKO12, CDU_CLKO_SEL1, 0),
	CDU_MUX(ADSP_SC598_CLK_CDU_EMMC,
		"cdu_emmc", sc598_cdu_emmc_parents, CDU_CLKO13, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC598_CLK_CDU_EMMC_TIMER_QMC,
		"cdu_emmc_timer", sc598_cdu_emmc_timer_parents, CDU_CLKO14, CDU_CLKO_SEL2, 0),
	CDU_MUX(ADSP_SC598_CLK_CDU_OSPI_REFCLK,
		"cdu_ospi_refclk", sc598_cdu_ospi_refclk_parents, CDU_CLKO10, CDU_CLKO_SEL4, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC0,
		"cdu_sharc0", sc598_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL1, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC1,
		"cdu_sharc1", sc598_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL1, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_ARM,
		"cdu_arm", sc598_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL5, 0),
};

/* ADSP-SC594 CDU mux clocks */
static const struct sc5xx_clk sc594_mux_clks[] = {
	CDU_MUX(ADSP_SC5XX_CLK_CDU_DDR,
		"cdu_ddr", sc594_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_CAN,
		"cdu_can", sc594_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_SPDIF,
		"cdu_spdif", sc594_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL1, 0),
	CDU_MUX(ADSP_SC594_CLK_CDU_SPI,
		"cdu_spi", sc594_cdu_spi_parents, CDU_CLKO6, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_GIGE,
		"cdu_gige", sc594_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC594_CLK_CDU_LP,
		"cdu_lp", sc594_cdu_lp_parents, CDU_CLKO8, CDU_CLKO_SEL4, 0),
	CDU_MUX(ADSP_SC594_CLK_CDU_LPDDR,
		"cdu_lpddr", sc594_cdu_lpddr_parents, CDU_CLKO9, CDU_CLKO_SEL4, 0),
	CDU_MUX(ADSP_SC594_CLK_CDU_TRACE,
		"cdu_trace", sc594_cdu_trace_parents, CDU_CLKO12, CDU_CLKO_SEL1, 0),
	CDU_MUX(ADSP_SC594_CLK_CDU_OSPI_REFCLK,
		"cdu_ospi_refclk", sc594_cdu_ospi_refclk_parents, CDU_CLKO10, CDU_CLKO_SEL4, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC0,
		"cdu_sharc0", sc594_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL1, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC1,
		"cdu_sharc1", sc594_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL1, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_ARM,
		"cdu_arm", sc594_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL1, 0),
};

/* ADSP-SC589 CDU mux clocks */
static const struct sc5xx_clk sc589_mux_clks[] = {
	CDU_MUX(ADSP_SC5XX_CLK_CDU_DDR,
		"cdu_ddr", sc589_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_CAN,
		"cdu_can", sc589_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL4, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_SPDIF,
		"cdu_spdif", sc589_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_GIGE,
		"cdu_gige", sc589_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC58X_CLK_CDU_LP,
		"cdu_lp", sc589_cdu_lp_parents, CDU_CLKO8, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC58X_CLK_CDU_SDIO,
		"cdu_sdio", sc589_cdu_sdio_parents, CDU_CLKO9, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC58X_CLK_CDU_RESERVED,
		"cdu_reserved", sc589_cdu_reserved_parents, CDU_CLKO6, CDU_CLKO_SEL3, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC0,
		"cdu_sharc0", sc589_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL3, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC1,
		"cdu_sharc1", sc589_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL3, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_ARM,
		"cdu_arm", sc589_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL3, 0),
};

/* ADSP-SC573 CDU mux clocks */
static const struct sc5xx_clk sc573_mux_clks[] = {
	CDU_MUX(ADSP_SC5XX_CLK_CDU_DDR,
		"cdu_ddr", sc573_cdu_ddr_parents, CDU_CLKO3, CDU_CLKO_SEL3, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_CAN,
		"cdu_can", sc573_cdu_can_parents, CDU_CLKO4, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_SPDIF,
		"cdu_spdif", sc573_cdu_spdif_parents, CDU_CLKO5, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC5XX_CLK_CDU_GIGE,
		"cdu_gige", sc573_cdu_gige_parents, CDU_CLKO7, CDU_CLKO_SEL6, 0),
	CDU_MUX(ADSP_SC57X_CLK_CDU_SDIO,
		"cdu_sdio", sc573_cdu_sdio_parents, CDU_CLKO9, CDU_CLKO_SEL6, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC0,
		"cdu_sharc0", sc573_cdu_sharc0_parents, CDU_CLKO0, CDU_CLKO_SEL3, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_SHARC1,
		"cdu_sharc1", sc573_cdu_sharc1_parents, CDU_CLKO1, CDU_CLKO_SEL3, 0),
	CDU_CMUX(ADSP_SC5XX_CLK_CDU_ARM,
		"cdu_arm", sc573_cdu_arm_parents, CDU_CLKO2, CDU_CLKO_SEL3, 0),
};

static struct clk_hw *sc5xx_register_one(struct device *dev, void __iomem *reg_base,
			      		 struct clk_hw_onecell_data *clk_data,
			      		 const struct sc5xx_clk *clk)
{
	switch (clk->type) {
	case SC5XX_CLK_CDU_MUX:
		const struct sc5xx_cdu_mux *cdu_mux = &clk->cdu_mux;

		return sc5xx_cdu_register(dev, cdu_mux->name, reg_base, cdu_mux->cdu_clko,
					  cdu_mux->parent_data, cdu_mux->mux_table,
					  cdu_mux->num_parents, cdu_mux->flags,
					  sc5xx_clk_lock);
	case SC5XX_CLK_DIV:		
		const struct sc5xx_div_clock *div = &clk->div;
		
		struct clk_hw *hw = clk_data->hws[div->
		
		return clk_hw_register_divider_parent_data(dev, div->name, div->parent_data, div->flags, 
	case SC5XX_CLK_PLL:
		const struct sc5xx_pll_clock *pll = &clk->pll;
	
		return 
	case SC5XX_CLK_FFACTOR:
		const sc5xx_fixed_factor_clock *ffactor = &clk->ffactor;
	
	
		return 
	case SC5XX_CLK_GATE:
		const sc5xx_gate_clock *gate = &clk->gate;
	
		return 
	case SC5XX_CLK_MUX:
		const sc5xx_mux_clock *mux = &clk->mux;
	
		return 
	case SC5XX_CLK_CLKINSEL:
		const struct sc5xx_clkinsel_clock *clkinsel = &clk->clkinsel;

		return sc5xx_cdu_clkin_register(dev, clkinsel->name, reg_base, clkinsel->parent_data,
						clkinsel->num_parents, clkinsel->flags,
						sc5xx_clk_lock);
	default:
		pr_err("Unknown clock type %u\n", clk->type);
		return ERR_PTR(-EINVAL);
	}	
}

/**
 * sc5xx_register_early_clocks - Register ADSP-SC5XX early clocks
 * @np: clock controller device node
 * @early_clks: early clocks to register
 * @num_early_clks: number of early clocks in @early_clks
 * @num_provider_clks: number of clocks for the clock controller
 *
 * Register the minimal CGU0 clocks needed to satisfy
 * early clock timer requirements on the ADSP-SCXX SoCs:
 *
 * 	DF -> VCO -> PLLCLK -> SYSSEL -> S0SEL -> SCLK0
 */
static void sc5xx_register_early_clocks(struct device_node *np, 
					const struct sc5xx_clk *early_clks,
					unsigned int num_early_clks,
					unsigned int num_provider_clks)
{
	struct clk_hw_onecell_data *clk_data;
	void __iomem *reg_base;
	int ret;

	reg_base = of_iomap(np, 0);
	if (!reg_base) {
		pr_err("%pOF: failed to map clock registers\n", np);
		return;
	}

	clk_data = kzalloc(struct_size(clk_data, hws, num_provider_clks), GFP_KERNEL);
	if (!clk_data)
		goto out_unmap;
	
	clk_data->nums = num_provider_clks;	

	for (int i = 0; i < num_early_clks; i++) {
		ret = sc5xx_register_one_early(NULL, reg_base, clk_data, &clk_data[i]);	
	}

out_unmap:
	iounmap(reg_base);
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
		.suppress_bind_attrs = true,
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
