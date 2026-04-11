// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G3L CPG driver
 *
 * Copyright (C) 2026 Renesas Electronics Corp.
 */

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <dt-bindings/clock/renesas,r9a08g046-cpg.h>

#include "rzg2l-cpg.h"

/* RZ/G3L Specific registers. */
#define G3L_CPG_PL2_DDIV		(0x204)
#define G3L_CPG_PL3_DDIV		(0x208)
#define G3L_CLKDIVSTATUS		(0x280)

/* RZ/G3L Specific division configuration.  */
#define G3L_DIVPL2A		DDIV_PACK(G3L_CPG_PL2_DDIV, 0, 2)
#define G3L_DIVPL2B		DDIV_PACK(G3L_CPG_PL2_DDIV, 4, 2)
#define G3L_DIVPL3A		DDIV_PACK(G3L_CPG_PL3_DDIV, 0, 2)

/* RZ/G3L Clock status configuration. */
#define G3L_DIVPL2A_STS		DDIV_PACK(G3L_CLKDIVSTATUS, 4, 1)
#define G3L_DIVPL2B_STS		DDIV_PACK(G3L_CLKDIVSTATUS, 5, 1)
#define G3L_DIVPL3A_STS		DDIV_PACK(G3L_CLKDIVSTATUS, 8, 1)

enum clk_ids {
	/* Core Clock Outputs exported to DT */
	LAST_DT_CORE_CLK = R9A08G046_USB_SCLK,

	/* External Input Clocks */
	CLK_EXTAL,
	CLK_ETH0_TXC_TX_CLK_IN,
	CLK_ETH0_RXC_RX_CLK_IN,
	CLK_ETH1_TXC_TX_CLK_IN,
	CLK_ETH1_RXC_RX_CLK_IN,

	/* Internal Core Clocks */
	CLK_PLL2,
	CLK_PLL2_DIV2,
	CLK_PLL3,
	CLK_PLL3_DIV2,

	/* Module Clocks */
	MOD_CLK_BASE,
};

/* Divider tables */
static const struct clk_div_table dtable_4_128[] = {
	{ 0, 4 },
	{ 1, 8 },
	{ 2, 16 },
	{ 3, 128 },
	{ 0, 0 },
};

static const struct clk_div_table dtable_8_256[] = {
	{ 0, 8 },
	{ 1, 16 },
	{ 2, 32 },
	{ 3, 256 },
	{ 0, 0 },
};

static const struct cpg_core_clk r9a08g046_core_clks[] __initconst = {
	/* External Clock Inputs */
	DEF_INPUT("extal", CLK_EXTAL),
	DEF_INPUT("eth0_txc_tx_clk", CLK_ETH0_TXC_TX_CLK_IN),
	DEF_INPUT("eth0_rxc_rx_clk", CLK_ETH0_RXC_RX_CLK_IN),
	DEF_INPUT("eth1_txc_tx_clk", CLK_ETH1_TXC_TX_CLK_IN),
	DEF_INPUT("eth1_rxc_rx_clk", CLK_ETH1_RXC_RX_CLK_IN),

	/* Internal Core Clocks */
	DEF_FIXED(".pll2", CLK_PLL2, CLK_EXTAL, 200, 3),
	DEF_FIXED(".pll3", CLK_PLL3, CLK_EXTAL, 200, 3),
	DEF_FIXED(".pll2_div2", CLK_PLL2_DIV2, CLK_PLL2, 1, 2),
	DEF_FIXED(".pll3_div2", CLK_PLL3_DIV2, CLK_PLL3, 1, 2),

	/* Core output clk */
	DEF_G3S_DIV("P0", R9A08G046_CLK_P0, CLK_PLL2_DIV2, G3L_DIVPL2B, G3L_DIVPL2B_STS,
		    dtable_8_256, 0, 0, 0, NULL),
	DEF_G3S_DIV("P1", R9A08G046_CLK_P1, CLK_PLL3_DIV2, G3L_DIVPL3A, G3L_DIVPL3A_STS,
		    dtable_4_128, 0, 0, 0, NULL),
	DEF_G3S_DIV("P3", R9A08G046_CLK_P3, CLK_PLL2_DIV2, G3L_DIVPL2A, G3L_DIVPL2A_STS,
		    dtable_4_128, 0, 0, 0, NULL),
};

static const struct rzg2l_mod_clk r9a08g046_mod_clks[] = {
	DEF_MOD("gic_gicclk",		R9A08G046_GIC600_GICCLK, R9A08G046_CLK_P1, 0x514, 0,
					MSTOP(BUS_PERI_COM, BIT(12))),
	DEF_MOD("ia55_pclk",		R9A08G046_IA55_PCLK, R9A08G046_CLK_P0, 0x518, 0,
					MSTOP(BUS_PERI_CPU, BIT(13))),
	DEF_MOD("ia55_clk",		R9A08G046_IA55_CLK, R9A08G046_CLK_P1, 0x518, 1,
					MSTOP(BUS_PERI_CPU, BIT(13))),
	DEF_MOD("dmac_aclk",		R9A08G046_DMAC_ACLK, R9A08G046_CLK_P3, 0x52c, 0,
					MSTOP(BUS_REG1, BIT(2))),
	DEF_MOD("dmac_pclk",		R9A08G046_DMAC_PCLK, R9A08G046_CLK_P3, 0x52c, 1,
					MSTOP(BUS_REG1, BIT(3))),
	DEF_MOD("scif0_clk_pck",	R9A08G046_SCIF0_CLK_PCK, R9A08G046_CLK_P0, 0x584, 0,
					MSTOP(BUS_MCPU2, BIT(1))),
};

static const struct rzg2l_reset r9a08g046_resets[] = {
	DEF_RST(R9A08G046_GIC600_GICRESET_N, 0x814, 0),
	DEF_RST(R9A08G046_GIC600_DBG_GICRESET_N, 0x814, 1),
	DEF_RST(R9A08G046_IA55_RESETN, 0x818, 0),
	DEF_RST(R9A08G046_DMAC_ARESETN, 0x82c, 0),
	DEF_RST(R9A08G046_DMAC_RST_ASYNC, 0x82c, 1),
	DEF_RST(R9A08G046_SCIF0_RST_SYSTEM_N, 0x884, 0),
};

static const unsigned int r9a08g046_crit_mod_clks[] __initconst = {
	MOD_CLK_BASE + R9A08G046_GIC600_GICCLK,
	MOD_CLK_BASE + R9A08G046_IA55_CLK,
	MOD_CLK_BASE + R9A08G046_DMAC_ACLK,
};

static const unsigned int r9a08g046_crit_resets[] = {
	R9A08G046_DMAC_ARESETN,
	R9A08G046_DMAC_RST_ASYNC,
};

const struct rzg2l_cpg_info r9a08g046_cpg_info = {
	/* Core Clocks */
	.core_clks = r9a08g046_core_clks,
	.num_core_clks = ARRAY_SIZE(r9a08g046_core_clks),
	.last_dt_core_clk = LAST_DT_CORE_CLK,
	.num_total_core_clks = MOD_CLK_BASE,

	/* Critical Module Clocks */
	.crit_mod_clks = r9a08g046_crit_mod_clks,
	.num_crit_mod_clks = ARRAY_SIZE(r9a08g046_crit_mod_clks),

	/* Module Clocks */
	.mod_clks = r9a08g046_mod_clks,
	.num_mod_clks = ARRAY_SIZE(r9a08g046_mod_clks),
	.num_hw_mod_clks = R9A08G046_BSC_X_BCK_BSC + 1,

	/* Resets */
	.resets = r9a08g046_resets,
	.num_resets = R9A08G046_BSC_X_PRESET_BSC + 1, /* Last reset ID + 1 */

	/* Critical Resets */
	.crit_resets = r9a08g046_crit_resets,
	.num_crit_resets = ARRAY_SIZE(r9a08g046_crit_resets),

	.has_clk_mon_regs = true,
};
