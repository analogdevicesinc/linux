// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022, Intel Corporation
 */
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/agilex5-clock.h>

#include "stratix10-clk.h"

static const struct clk_parent_data pll_mux[] = {
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data boot_mux[] = {
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
};

static const struct clk_parent_data core0_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c0",
		.name = "peri_pll_c0",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data core1_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c0",
		.name = "peri_pll_c0",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data core2_free_mux[] = {
	{
		.fw_name = "main_pll_c0",
		.name = "main_pll_c0",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data core3_free_mux[] = {
	{
		.fw_name = "main_pll_c0",
		.name = "main_pll_c0",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data dsu_free_mux[] = {
	{
		.fw_name = "main_pll_c2",
		.name = "main_pll_c2",
	},
	{
		.fw_name = "peri_pll_c0",
		.name = "peri_pll_c0",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data noc_free_mux[] = {
	{
		.fw_name = "main_pll_c3",
		.name = "main_pll_c3",
	},
	{
		.fw_name = "peri_pll_c1",
		.name = "peri_pll_c1",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data emaca_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c3",
		.name = "peri_pll_c3",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data emacb_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c3",
		.name = "peri_pll_c3",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data emac_ptp_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c3",
		.name = "peri_pll_c3",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data gpio_db_free_mux[] = {
	{
		.fw_name = "main_pll_c3",
		.name = "main_pll_c3",
	},
	{
		.fw_name = "peri_pll_c1",
		.name = "peri_pll_c1",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data psi_ref_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c3",
		.name = "peri_pll_c3",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data usb31_free_mux[] = {
	{
		.fw_name = "main_pll_c3",
		.name = "main_pll_c3",
	},
	{
		.fw_name = "peri_pll_c2",
		.name = "peri_pll_c2",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data s2f_usr0_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c3",
		.name = "peri_pll_c3",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data s2f_usr1_free_mux[] = {
	{
		.fw_name = "main_pll_c1",
		.name = "main_pll_c1",
	},
	{
		.fw_name = "peri_pll_c3",
		.name = "peri_pll_c3",
	},
	{
		.fw_name = "osc1",
		.name = "osc1",
	},
	{
		.fw_name = "cb-intosc-hs-div2-clk",
		.name = "cb-intosc-hs-div2-clk",
	},
	{
		.fw_name = "f2s-free-clk",
		.name = "f2s-free-clk",
	},
};

static const struct clk_parent_data core0_mux[] = {
	{
		.fw_name = "core0_free_clk",
		.name = "core0_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data core1_mux[] = {
	{
		.fw_name = "core1_free_clk",
		.name = "core1_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data core2_mux[] = {
	{
		.fw_name = "core2_free_clk",
		.name = "core2_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data core3_mux[] = {
	{
		.fw_name = "core3_free_clk",
		.name = "core3_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data dsu_mux[] = {
	{
		.fw_name = "dsu_free_clk",
		.name = "dsu_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data emac_mux[] = {
	{
		.fw_name = "emaca_free_clk",
		.name = "emaca_free_clk",
	},
	{
		.fw_name = "emacb_free_clk",
		.name = "emacb_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data noc_mux[] = {
	{
		.fw_name = "noc_free_clk",
		.name = "noc_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data s2f_user0_mux[] = {
	{
		.fw_name = "s2f_user0_free_clk",
		.name = "s2f_user0_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data s2f_user1_mux[] = {
	{
		.fw_name = "s2f_user1_free_clk",
		.name = "s2f_user1_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data psi_mux[] = {
	{
		.fw_name = "psi_ref_free_clk",
		.name = "psi_ref_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data gpio_db_mux[] = {
	{
		.fw_name = "gpio_db_free_clk",
		.name = "gpio_db_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data emac_ptp_mux[] = {
	{
		.fw_name = "emac_ptp_free_clk",
		.name = "emac_ptp_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

static const struct clk_parent_data usb31_mux[] = {
	{
		.fw_name = "usb31_free_clk",
		.name = "usb31_free_clk",
	},
	{
		.fw_name = "boot_clk",
		.name = "boot_clk",
	},
};

/*
 * TODO - Clocks in AO (always on) controller
 * 2 main PLLs only
 */
static const struct stratix10_pll_clock agilex5_pll_clks[] = {
	{ AGILEX5_BOOT_CLK, "boot_clk", boot_mux, ARRAY_SIZE(boot_mux), 0,
	  0x0 },
	{ AGILEX5_MAIN_PLL_CLK, "main_pll", pll_mux, ARRAY_SIZE(pll_mux), 0,
	  0x48 },
	{ AGILEX5_PERIPH_PLL_CLK, "periph_pll", pll_mux, ARRAY_SIZE(pll_mux), 0,
	  0x9C },
};

static const struct stratix10_perip_c_clock agilex5_main_perip_c_clks[] = {
	{ AGILEX5_MAIN_PLL_C0_CLK, "main_pll_c0", "main_pll", NULL, 1, 0,
	  0x5C },
	{ AGILEX5_MAIN_PLL_C1_CLK, "main_pll_c1", "main_pll", NULL, 1, 0,
	  0x60 },
	{ AGILEX5_MAIN_PLL_C2_CLK, "main_pll_c2", "main_pll", NULL, 1, 0,
	  0x64 },
	{ AGILEX5_MAIN_PLL_C3_CLK, "main_pll_c3", "main_pll", NULL, 1, 0,
	  0x68 },
	{ AGILEX5_PERIPH_PLL_C0_CLK, "peri_pll_c0", "periph_pll", NULL, 1, 0,
	  0xB0 },
	{ AGILEX5_PERIPH_PLL_C1_CLK, "peri_pll_c1", "periph_pll", NULL, 1, 0,
	  0xB4 },
	{ AGILEX5_PERIPH_PLL_C2_CLK, "peri_pll_c2", "periph_pll", NULL, 1, 0,
	  0xB8 },
	{ AGILEX5_PERIPH_PLL_C3_CLK, "peri_pll_c3", "periph_pll", NULL, 1, 0,
	  0xBC },
};

/* Non-SW clock-gated enabled clocks */
static const struct stratix10_perip_cnt_clock agilex5_main_perip_cnt_clks[] = {
	{ AGILEX5_CORE0_FREE_CLK, "core0_free_clk", NULL, core0_free_mux,
	ARRAY_SIZE(core0_free_mux), 0, 0x0104, 0, 0, 0},
	{ AGILEX5_CORE1_FREE_CLK, "core1_free_clk", NULL, core1_free_mux,
	ARRAY_SIZE(core1_free_mux), 0, 0x0104, 0, 0, 0},
	{ AGILEX5_CORE2_FREE_CLK, "core2_free_clk", NULL, core2_free_mux,
	ARRAY_SIZE(core2_free_mux), 0, 0x010C, 0, 0, 0},
	{ AGILEX5_CORE3_FREE_CLK, "core3_free_clk", NULL, core3_free_mux,
	ARRAY_SIZE(core3_free_mux), 0, 0x0110, 0, 0, 0},
	{ AGILEX5_DSU_FREE_CLK, "dsu_free_clk", NULL, dsu_free_mux,
	ARRAY_SIZE(dsu_free_mux), 0, 0x0100, 0, 0, 0},
	{ AGILEX5_NOC_FREE_CLK, "noc_free_clk", NULL, noc_free_mux,
	  ARRAY_SIZE(noc_free_mux), 0, 0x40, 0, 0, 0 },
	{ AGILEX5_EMAC_A_FREE_CLK, "emaca_free_clk", NULL, emaca_free_mux,
	  ARRAY_SIZE(emaca_free_mux), 0, 0xD4, 0, 0x88, 0 },
	{ AGILEX5_EMAC_B_FREE_CLK, "emacb_free_clk", NULL, emacb_free_mux,
	  ARRAY_SIZE(emacb_free_mux), 0, 0xD8, 0, 0x88, 1 },
	{ AGILEX5_EMAC_PTP_FREE_CLK, "emac_ptp_free_clk", NULL,
	  emac_ptp_free_mux, ARRAY_SIZE(emac_ptp_free_mux), 0, 0xDC, 0, 0x88,
	  2 },
	{ AGILEX5_GPIO_DB_FREE_CLK, "gpio_db_free_clk", NULL, gpio_db_free_mux,
	  ARRAY_SIZE(gpio_db_free_mux), 0, 0xE0, 0, 0x88, 3 },
	{ AGILEX5_S2F_USER0_FREE_CLK, "s2f_user0_free_clk", NULL,
	  s2f_usr0_free_mux, ARRAY_SIZE(s2f_usr0_free_mux), 0, 0xE8, 0, 0x30,
	  2 },
	{ AGILEX5_S2F_USER1_FREE_CLK, "s2f_user1_free_clk", NULL,
	  s2f_usr1_free_mux, ARRAY_SIZE(s2f_usr1_free_mux), 0, 0xEC, 0, 0x88,
	  5 },
	{ AGILEX5_PSI_REF_FREE_CLK, "psi_ref_free_clk", NULL, psi_ref_free_mux,
	  ARRAY_SIZE(psi_ref_free_mux), 0, 0xF0, 0, 0x88, 6 },
	{ AGILEX5_USB31_FREE_CLK, "usb31_free_clk", NULL, usb31_free_mux,
	  ARRAY_SIZE(usb31_free_mux), 0, 0xF8, 0, 0x88, 7},
};

/* SW Clock gate enabled clocks */
static const struct stratix10_gate_clock agilex5_gate_clks[] = {

	/* TODO HW Managed Clocks list */

	/* TODO SW Managed Clocks list */

	/* Main PLL0 Begin */
	/* MPU clocks */
	{ AGILEX5_CORE0_CLK, "core0_clk", NULL, core0_mux,
	  ARRAY_SIZE(core0_mux), 0, 0x24, 8, 0, 0, 0, 0x30, 5, 0 },
	{ AGILEX5_CORE1_CLK, "core1_clk", NULL, core1_mux,
	  ARRAY_SIZE(core1_mux), 0, 0x24, 9, 0, 0, 0, 0x30, 5, 0 },
	{ AGILEX5_CORE2_CLK, "core2_clk", NULL, core2_mux,
	  ARRAY_SIZE(core2_mux), 0, 0x24, 10, 0, 0, 0, 0x30, 6, 0 },
	{ AGILEX5_CORE3_CLK, "core3_clk", NULL, core3_mux,
	  ARRAY_SIZE(core3_mux), 0, 0x24, 11, 0, 0, 0, 0x30, 7, 0 },
	{ AGILEX5_MPU_CLK, "dsu_clk", NULL, dsu_mux, ARRAY_SIZE(dsu_mux), 0, 0,
	  0, 0, 0, 0, 0x34, 4, 0 },
	{ AGILEX5_MPU_PERIPH_CLK, "mpu_periph_clk", NULL, dsu_mux,
	  ARRAY_SIZE(dsu_mux), 0, 0, 0, 0x44, 20, 2, 0x34, 4, 0 },
	{ AGILEX5_MPU_CCU_CLK, "mpu_ccu_clk", NULL, dsu_mux,
	  ARRAY_SIZE(dsu_mux), 0, 0, 0, 0x44, 18, 2, 0x34, 4, 0 },

	/* ANGTS TODO l4 main clk has no divider now. To check. */
	{ AGILEX5_L4_MAIN_CLK, "l4_main_clk", NULL, noc_mux,
	  ARRAY_SIZE(noc_mux), 0, 0x24, 1, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_L4_MP_CLK, "l4_mp_clk", NULL, noc_mux, ARRAY_SIZE(noc_mux), 0,
	  0x24, 2, 0x44, 4, 2, 0x30, 1, 0 },
	{ AGILEX5_L4_SYS_FREE_CLK, "l4_sys_free_clk", NULL, noc_mux,
	  ARRAY_SIZE(noc_mux), 0, 0, 0, 0x44, 2, 2, 0x30, 1, 0 },
	{ AGILEX5_L4_SP_CLK, "l4_sp_clk", NULL, noc_mux, ARRAY_SIZE(noc_mux),
	  CLK_IS_CRITICAL, 0x24, 3, 0x44, 6, 2, 0x30, 1, 0 },

	/* Core sight clocks*/
	{ AGILEX5_CS_AT_CLK, "cs_at_clk", NULL, noc_mux, ARRAY_SIZE(noc_mux), 0,
	  0x24, 4, 0x44, 24, 2, 0x30, 1, 0 },
	{ AGILEX5_CS_TRACE_CLK, "cs_trace_clk", NULL, noc_mux,
	  ARRAY_SIZE(noc_mux), 0, 0x24, 4, 0x44, 26, 2, 0x30, 1, 0 },
	{ AGILEX5_CS_PDBG_CLK, "cs_pdbg_clk", "cs_at_clk", NULL, 1, 0, 0x24, 4,
	  0x44, 28, 1, 0, 0, 0 },
	/* Main PLL0 End */

	/* Main Peripheral PLL1 Begin */
	{ AGILEX5_EMAC0_CLK, "emac0_clk", NULL, emac_mux, ARRAY_SIZE(emac_mux),
	  0, 0x7C, 0, 0, 0, 0, 0x94, 26, 0 },
	{ AGILEX5_EMAC1_CLK, "emac1_clk", NULL, emac_mux, ARRAY_SIZE(emac_mux),
	  0, 0x7C, 1, 0, 0, 0, 0x94, 27, 0 },
	{ AGILEX5_EMAC2_CLK, "emac2_clk", NULL, emac_mux, ARRAY_SIZE(emac_mux),
	  0, 0x7C, 2, 0, 0, 0, 0x94, 28, 0 },
	{ AGILEX5_EMAC_PTP_CLK, "emac_ptp_clk", NULL, emac_ptp_mux,
	  ARRAY_SIZE(emac_ptp_mux), 0, 0x7C, 3, 0, 0, 0, 0x88, 2, 0 },
	{ AGILEX5_GPIO_DB_CLK, "gpio_db_clk", NULL, gpio_db_mux,
	  ARRAY_SIZE(gpio_db_mux), 0, 0x7C, 4, 0x98, 0, 16, 0x88, 3, 1 },
	  /* Main Peripheral PLL1 End */

	  /* Peripheral clocks  */
	{ AGILEX5_S2F_USER0_CLK, "s2f_user0_clk", NULL, s2f_user0_mux,
	  ARRAY_SIZE(s2f_user0_mux), 0, 0x24, 6, 0, 0, 0, 0x30, 2, 0 },
	{ AGILEX5_S2F_USER1_CLK, "s2f_user1_clk", NULL, s2f_user1_mux,
	  ARRAY_SIZE(s2f_user1_mux), 0, 0x7C, 6, 0, 0, 0, 0x88, 5, 0 },
	{ AGILEX5_PSI_REF_CLK, "psi_ref_clk", NULL, psi_mux,
	  ARRAY_SIZE(psi_mux), 0, 0x7C, 7, 0, 0, 0, 0x88, 6, 0 },
	{ AGILEX5_USB31_SUSPEND_CLK, "usb31_suspend_clk", NULL, usb31_mux,
	  ARRAY_SIZE(usb31_mux), 0, 0x7C, 25, 0, 0, 0, 0x88, 7, 0 },
	{ AGILEX5_USB31_BUS_CLK_EARLY, "usb31_bus_clk_early", "l4_main_clk",
	  NULL, 1, 0, 0x7C, 25, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_USB2OTG_HCLK, "usb2otg_hclk", "l4_mp_clk", NULL, 1, 0, 0x7C,
	  8, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SPIM_0_CLK, "spim_0_clk", "l4_mp_clk", NULL, 1, 0, 0x7C, 9,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SPIM_1_CLK, "spim_1_clk", "l4_mp_clk", NULL, 1, 0, 0x7C, 11,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SPIS_0_CLK, "spis_0_clk", "l4_sp_clk", NULL, 1, 0, 0x7C, 12,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SPIS_1_CLK, "spis_1_clk", "l4_sp_clk", NULL, 1, 0, 0x7C, 13,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_DMA_CORE_CLK, "dma_core_clk", "l4_mp_clk", NULL, 1, 0, 0x7C,
	  14, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_DMA_HS_CLK, "dma_hs_clk", "l4_mp_clk", NULL, 1, 0, 0x7C, 14,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_I3C_0_CORE_CLK, "i3c_0_core_clk", "l4_mp_clk", NULL, 1, 0,
	  0x7C, 18, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_I3C_1_CORE_CLK, "i3c_1_core_clk", "l4_mp_clk", NULL, 1, 0,
	  0x7C, 19, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_I2C_0_PCLK, "i2c_0_pclk", "l4_sp_clk", NULL, 1, 0, 0x7C, 15,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_I2C_1_PCLK, "i2c_1_pclk", "l4_sp_clk", NULL, 1, 0, 0x7C, 16,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_I2C_EMAC0_PCLK, "i2c_emac0_pclk", "l4_sp_clk", NULL, 1, 0,
	  0x7C, 17, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_I2C_EMAC1_PCLK, "i2c_emac1_pclk", "l4_sp_clk", NULL, 1, 0,
	  0x7C, 22, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_I2C_EMAC2_PCLK, "i2c_emac2_pclk", "l4_sp_clk", NULL, 1, 0,
	  0x7C, 27, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_UART_0_PCLK, "uart_0_pclk", "l4_sp_clk", NULL, 1, 0, 0x7C, 20,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_UART_1_PCLK, "uart_1_pclk", "l4_sp_clk", NULL, 1, 0, 0x7C, 21,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SPTIMER_0_PCLK, "sptimer_0_pclk", "l4_sp_clk", NULL, 1, 0,
	  0x7C, 23, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SPTIMER_1_PCLK, "sptimer_1_pclk", "l4_sp_clk", NULL, 1, 0,
	  0x7C, 24, 0, 0, 0, 0, 0, 0 },

	/*NAND, SD/MMC and SoftPHY overall clocking*/
	{ AGILEX5_DFI_CLK, "dfi_clk", "l4_mp_clk", NULL, 1, 0, 0, 0, 0x44, 16,
	  2, 0, 0, 0 },
	{ AGILEX5_NAND_NF_CLK, "nand_nf_clk", "dfi_clk", NULL, 1, 0, 0x7C, 10,
	  0, 0, 0, 0, 0, 0 },
	{ AGILEX5_NAND_BCH_CLK, "nand_bch_clk", "l4_mp_clk", NULL, 1, 0, 0x7C,
	  10, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SDMMC_SDPHY_REG_CLK, "sdmmc_sdphy_reg_clk", "l4_mp_clk", NULL,
	  1, 0, 0x7C, 5, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SDMCLK, "sdmclk", "dfi_clk", NULL, 1, 0, 0x7C, 5, 0, 0, 0, 0,
	  0, 0 },
	{ AGILEX5_SOFTPHY_REG_PCLK, "softphy_reg_pclk", "l4_mp_clk", NULL, 1, 0,
	  0x7C, 26, 0, 0, 0, 0, 0, 0 },
	{ AGILEX5_SOFTPHY_PHY_CLK, "softphy_phy_clk", "l4_mp_clk", NULL, 1, 0,
	  0x7C, 26, 0x44, 16, 2, 0, 0, 0 },
	{ AGILEX5_SOFTPHY_CTRL_CLK, "softphy_ctrl_clk", "dfi_clk", NULL, 1, 0,
	  0x7C, 26, 0, 0, 0, 0, 0, 0 },
};

static int
agilex5_clk_register_c_perip(const struct stratix10_perip_c_clock *clks,
			     int nums, struct stratix10_clock_data *data)
{
	struct clk_hw *hw_clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		hw_clk = s10_register_periph(&clks[i], base);
		if (IS_ERR(hw_clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       clks[i].name);
			continue;
		}
		data->clk_data.hws[clks[i].id] = hw_clk;
	}
	return 0;
}

static int
agilex5_clk_register_cnt_perip(const struct stratix10_perip_cnt_clock *clks,
			       int nums, struct stratix10_clock_data *data)
{
	struct clk_hw *hw_clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		hw_clk = s10_register_cnt_periph(&clks[i], base);
		if (IS_ERR(hw_clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       clks[i].name);
			continue;
		}
		data->clk_data.hws[clks[i].id] = hw_clk;
	}

	return 0;
}

static int agilex5_clk_register_gate(const struct stratix10_gate_clock *clks,
				     int nums,
				     struct stratix10_clock_data *data)
{
	struct clk_hw *hw_clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		hw_clk = agilex_register_gate(&clks[i], base);
		if (IS_ERR(hw_clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       clks[i].name);
			continue;
		}
		data->clk_data.hws[clks[i].id] = hw_clk;
	}

	return 0;
}

static int agilex5_clk_register_pll(const struct stratix10_pll_clock *clks,
				    int nums, struct stratix10_clock_data *data)
{
	struct clk_hw *hw_clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		hw_clk = agilex5_register_pll(&clks[i], base);
		if (IS_ERR(hw_clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       clks[i].name);
			continue;
		}
		data->clk_data.hws[clks[i].id] = hw_clk;
	}

	return 0;
}

static int agilex5_clkmgr_init(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct stratix10_clock_data *clk_data;
	struct resource *res;
	void __iomem *base;
	int i, num_clks;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	num_clks = AGILEX5_NUM_CLKS;

	clk_data = devm_kzalloc(
		dev, struct_size(clk_data, clk_data.hws, num_clks), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	for (i = 0; i < num_clks; i++)
		clk_data->clk_data.hws[i] = ERR_PTR(-ENOENT);

	clk_data->base = base;
	clk_data->clk_data.num = num_clks;

	agilex5_clk_register_pll(agilex5_pll_clks, ARRAY_SIZE(agilex5_pll_clks),
				 clk_data);

	agilex5_clk_register_c_perip(agilex5_main_perip_c_clks,
				     ARRAY_SIZE(agilex5_main_perip_c_clks),
				     clk_data);

	agilex5_clk_register_cnt_perip(agilex5_main_perip_cnt_clks,
				       ARRAY_SIZE(agilex5_main_perip_cnt_clks),
				       clk_data);

	agilex5_clk_register_gate(agilex5_gate_clks,
				  ARRAY_SIZE(agilex5_gate_clks), clk_data);

	of_clk_add_hw_provider(np, of_clk_hw_onecell_get, &clk_data->clk_data);
	return 0;
}

static int agilex5_clkmgr_probe(struct platform_device *pdev)
{
	int (*probe_func)(struct platform_device *init_func);

	probe_func = of_device_get_match_data(&pdev->dev);
	if (!probe_func)
		return -ENODEV;
	return probe_func(pdev);
}

static const struct of_device_id agilex5_clkmgr_match_table[] = {
	{ .compatible = "intel,agilex5-clkmgr", .data = agilex5_clkmgr_init },
	{}
};

static struct platform_driver agilex5_clkmgr_driver = {
	.probe		= agilex5_clkmgr_probe,
	.driver		= {
		.name	= "agilex5-clkmgr",
		.suppress_bind_attrs = true,
		.of_match_table = agilex5_clkmgr_match_table,
	},
};

static int __init agilex5_clk_init(void)
{
	return platform_driver_register(&agilex5_clkmgr_driver);
}
core_initcall(agilex5_clk_init);
