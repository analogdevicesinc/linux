/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <dt-bindings/clock/imx7ulp-clock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "clk.h"

static const char *pll_pre_sels[]	= { "sosc", "firc", };
static const char *spll_pfd_sels[]	= { "spll_pfd0", "spll_pfd1", "spll_pfd2", "spll_pfd3", };
static const char *spll_sels[]		= { "spll", "spll_pfd_sel", };
static const char *apll_pfd_sels[]	= { "apll_pfd0", "apll_pfd1", "apll_pfd2", "apll_pfd3", };
static const char *apll_sels[]		= { "apll", "apll_pfd_sel", };
static const char *sys_sels[]		= { "dummy", "sosc", "sirc", "firc", "rosc", "apll_sel", "spll_sel", "upll", };
static const char *arm_sels[]		= { "core_div", "dummy", "dummy", "hsrun_core", };
static const char *ddr_sels[]		= { "apll_pfd_sel", "upll", };
static const char *nic_sels[]		= { "firc", "ddr_div", };
static const char *periph_plat_sels[]	= { "dummy", "nic1_bus", "nic1_div", "ddr_div", "apll_pfd2", "apll_pfd1", "apll_pfd0", "upll", };
/* the dummy in only a space holder of spll_bus clk */
static const char *periph_slow_sels[]	= { "dummy", "sosc", "dummy", "firc", "rosc", "nic1_bus", "nic1_div", "dummy", };
static struct clk *clks[IMX7ULP_CLK_END];
static struct clk_onecell_data clk_data;

static const char *cm4_pll_pre_sels[]	= { "cm4_sosc", "cm4_firc", };
static const char *cm4_spll_pfd_sels[]	= { "cm4_spll_pfd0", "cm4_spll_pfd1", "cm4_spll_pfd2", "cm4_spll_pfd3", };
static const char *cm4_spll_sels[]		= { "cm4_spll_vco", "cm4_spll_pfd_sel", };
static const char *cm4_apll_pfd_sels[]	= { "cm4_apll_pfd0", "cm4_apll_pfd1", "cm4_apll_pfd2", "cm4_apll_pfd3", };
static const char *cm4_apll_sels[]		= { "cm4_apll_vco_post_div2", "cm4_apll_pfd_sel", };
static const char *cm4_sys_sels[]		= { "cm4_dummy", "cm4_sosc", "cm4_sirc", "cm4_firc", "cm4_rosc", "cm4_apll_sel", "cm4_spll_sel", "cm4_dummy", };
static const char *cm4_periph_slow_sels[]	= { "cm4_dummy", "cm4_sosc", "cm4_sirc", "cm4_firc", "cm4_rosc", "cm4_bus_div", "cm4_spll_pfd2", "cm4_apll_pfd0_pre_div", };
static const char *scg0_clkout_sels[]   = { "dummy", "cm4_sosc", "cm4_sirc", "cm4_firc", "cm4_rosc", "cm4_apll_sel", "cm4_spll_sel", "dummy"};

static struct clk *clks_cm4[IMX7ULP_CM4_CLK_END];
static struct clk_onecell_data clk_data_cm4;


static int const clks_init_on[] __initconst = {
	IMX7ULP_CLK_BUS_DIV,
	IMX7ULP_CLK_ARM,
	IMX7ULP_CLK_NIC0_DIV,
	IMX7ULP_CLK_NIC1_DIV,
	IMX7ULP_CLK_NIC1_BUS_DIV,
	IMX7ULP_CLK_MMDC,
};

/* used by sosc/sirc/firc/ddr/spll/apll dividers */
static const struct clk_div_table ulp_div_table[] = {
	{ .val = 1, .div = 1, },
	{ .val = 2, .div = 2, },
	{ .val = 3, .div = 4, },
	{ .val = 4, .div = 8, },
	{ .val = 5, .div = 16, },
	{ .val = 6, .div = 32, },
	{ .val = 7, .div = 64, },
	{ }
};

static void __init imx7ulp_clocks_init(struct device_node *scg_node)
{
	struct device_node *np;
	void __iomem *base;
	void __iomem *smc_base;
	int i;

	clks[IMX7ULP_CLK_DUMMY]		= imx_clk_fixed("dummy", 0);

	clks[IMX7ULP_CLK_ROSC]		= of_clk_get_by_name(scg_node, "rosc");
	clks[IMX7ULP_CLK_SOSC]		= of_clk_get_by_name(scg_node, "sosc");
	clks[IMX7ULP_CLK_SIRC]		= of_clk_get_by_name(scg_node, "sirc");
	clks[IMX7ULP_CLK_FIRC]		= of_clk_get_by_name(scg_node, "firc");
	clks[IMX7ULP_CLK_MIPI_PLL]	= of_clk_get_by_name(scg_node, "mpll");
	clks[IMX7ULP_CLK_UPLL]		= of_clk_get_by_name(scg_node, "upll");

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-smc1");
	smc_base = of_iomap(np, 0);
	WARN_ON(!smc_base);

	np = scg_node;
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks[IMX7ULP_CLK_SPLL_PRE_SEL] = imx_clk_mux("spll_pre_sel", base + 0x608, 0, 1, pll_pre_sels, ARRAY_SIZE(pll_pre_sels));
	clks[IMX7ULP_CLK_APLL_PRE_SEL] = imx_clk_mux("apll_pre_sel", base + 0x508, 0, 1, pll_pre_sels, ARRAY_SIZE(pll_pre_sels));
	/*						 name		parent_name	reg		shift	width */
	clks[IMX7ULP_CLK_SPLL_PRE_DIV] = imx_clk_divider("spll_pre_div", "spll_pre_sel", base + 0x608,  8,	3);
	clks[IMX7ULP_CLK_APLL_PRE_DIV] = imx_clk_divider("apll_pre_div", "apll_pre_sel", base + 0x508,	8,	3);
	/*					name	parent_name	base*/
	clks[IMX7ULP_CLK_SPLL] = imx_clk_pllv4("spll",  "spll_pre_div", base + 0x600);
	clks[IMX7ULP_CLK_APLL] = imx_clk_pllv4("apll",  "apll_pre_div", base + 0x500);

	/* SPLL PFDs */
	clks[IMX7ULP_CLK_SPLL_PFD0] = imx_clk_pfdv2("spll_pfd0", "spll", base + 0x60C, 0);
	clks[IMX7ULP_CLK_SPLL_PFD1] = imx_clk_pfdv2("spll_pfd1", "spll", base + 0x60C, 1);
	clks[IMX7ULP_CLK_SPLL_PFD2] = imx_clk_pfdv2("spll_pfd2", "spll", base + 0x60C, 2);
	clks[IMX7ULP_CLK_SPLL_PFD3] = imx_clk_pfdv2("spll_pfd3", "spll", base + 0x60C, 3);
	/* APLL PFDs */
	clks[IMX7ULP_CLK_APLL_PFD0] = imx_clk_pfdv2("apll_pfd0", "apll", base + 0x50C, 0);
	clks[IMX7ULP_CLK_APLL_PFD1] = imx_clk_pfdv2("apll_pfd1", "apll", base + 0x50C, 1);
	clks[IMX7ULP_CLK_APLL_PFD2] = imx_clk_pfdv2("apll_pfd2", "apll", base + 0x50C, 2);
	clks[IMX7ULP_CLK_APLL_PFD3] = imx_clk_pfdv2("apll_pfd3", "apll", base + 0x50C, 3);

	clks[IMX7ULP_CLK_SPLL_PFD_SEL] = imx_clk_mux("spll_pfd_sel", base + 0x608, 14, 2, spll_pfd_sels, ARRAY_SIZE(spll_pfd_sels));
	clks[IMX7ULP_CLK_APLL_PFD_SEL] = imx_clk_mux("apll_pfd_sel", base + 0x508, 14, 2, apll_pfd_sels, ARRAY_SIZE(apll_pfd_sels));

	clks[IMX7ULP_CLK_SPLL_SEL] = imx_clk_mux("spll_sel", base + 0x608, 1, 1, spll_sels, ARRAY_SIZE(spll_sels));
	clks[IMX7ULP_CLK_APLL_SEL] = imx_clk_mux("apll_sel", base + 0x508, 1, 1, apll_sels, ARRAY_SIZE(apll_sels));

	clks[IMX7ULP_CLK_SYS_SEL] = imx_clk_mux_glitchless("sys_sel", base + 0x14, 24, 4, sys_sels, ARRAY_SIZE(sys_sels));
	clks[IMX7ULP_CLK_HSRUN_SYS_SEL] = imx_clk_mux_glitchless("hsrun_sys_sel", base + 0x1c, 24, 4, sys_sels, ARRAY_SIZE(sys_sels));
	clks[IMX7ULP_CLK_DDR_SEL] = imx_clk_mux("ddr_sel", base + 0x30, 24, 1, ddr_sels, ARRAY_SIZE(ddr_sels));
	clks[IMX7ULP_CLK_NIC_SEL] = imx_clk_mux("nic_sel", base + 0x40, 28, 1, nic_sels, ARRAY_SIZE(nic_sels));

	clks[IMX7ULP_CLK_CORE_DIV] = imx_clk_divider_flags("core_div", "sys_sel", base + 0x14, 16, 4, CLK_SET_RATE_PARENT);
	clks[IMX7ULP_CLK_HSRUN_CORE] = imx_clk_divider_flags("hsrun_core", "hsrun_sys_sel", base + 0x1c, 16, 4, CLK_SET_RATE_PARENT);
	clks[IMX7ULP_CLK_PLAT_DIV] = imx_clk_divider("plat_div", "core_div", base + 0x14, 12, 4);
	/* Fake mux */
	clks[IMX7ULP_CLK_ARM] = imx_clk_mux_glitchless("arm", smc_base + 0x10, 8, 2, arm_sels, ARRAY_SIZE(arm_sels));

	clks[IMX7ULP_CLK_DDR_DIV] = clk_register_divider_table(NULL, "ddr_div", "ddr_sel", CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, base + 0x30, 0, 3,
							       CLK_DIVIDER_ZERO_GATE, ulp_div_table, &imx_ccm_lock);

	clks[IMX7ULP_CLK_NIC0_DIV] = imx_clk_divider("nic0_div", "nic_sel",  base + 0x40, 24, 4);
	clks[IMX7ULP_CLK_GPU_DIV]  = imx_clk_divider("gpu_div",  "nic0_div", base + 0x40, 20, 4);
	clks[IMX7ULP_CLK_NIC1_DIV] = imx_clk_divider("nic1_div", "nic0_div", base + 0x40, 16, 4);
	clks[IMX7ULP_CLK_NIC1_BUS_DIV] = imx_clk_divider("nic1_bus", "nic0_div", base + 0x40, 4, 4);

	/* PCC2 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pcc2");
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks[IMX7ULP_CLK_DMA1]		= imx_clk_gate("dma1", "nic1_bus", base + 0x20, 30);
	clks[IMX7ULP_CLK_RGPIO2P1]	= imx_clk_gate("rgpio2p1", "nic1_bus",	base + 0x3c, 30);
	clks[IMX7ULP_CLK_DMA_MUX1]	= imx_clk_gate("dma_mux1", "nic1_bus",	base + 0x84, 30);
	clks[IMX7ULP_CLK_CAAM]		= imx_clk_gate("caam", "nic1_div",	base + 0x90, 30);
	clks[IMX7ULP_CLK_LPTPM4]	= imx_clk_composite("lptpm4",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x94);
	clks[IMX7ULP_CLK_LPTPM5]	= imx_clk_composite("lptmp5",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x98);
	clks[IMX7ULP_CLK_LPIT1]		= imx_clk_composite("lpit1",   periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x9C);
	clks[IMX7ULP_CLK_LPSPI2]	= imx_clk_composite("lpspi2",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0xA4);
	clks[IMX7ULP_CLK_LPSPI3]	= imx_clk_composite("lpspi3",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0xA8);
	clks[IMX7ULP_CLK_LPI2C4]	= imx_clk_composite("lpi2c4",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0xAC);
	clks[IMX7ULP_CLK_LPI2C5]	= imx_clk_composite("lpi2c5",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0xB0);
	clks[IMX7ULP_CLK_LPUART4]	= imx_clk_composite("lpuart4", periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0xB4);
	clks[IMX7ULP_CLK_LPUART5]	= imx_clk_composite("lpuart5", periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0xB8);
	clks[IMX7ULP_CLK_FLEXIO1]	= imx_clk_composite("flexio",  periph_slow_sels, ARRAY_SIZE(periph_plat_sels), true, false, true, base + 0xC4);
	clks[IMX7ULP_CLK_USB0]		= imx_clk_composite("usb0",    periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, true,  true, base + 0xCC);
	clks[IMX7ULP_CLK_USB1]		= imx_clk_composite("usb1",    periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, true,  true, base + 0xD0);
	clks[IMX7ULP_CLK_USB_PHY]	= imx_clk_gate("usb_phy",  "nic1_bus", base + 0xD4, 30);
	clks[IMX7ULP_CLK_USDHC0]	= imx_clk_composite("usdhc0",  periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, true,  true, base + 0xDC);
	clks[IMX7ULP_CLK_USDHC1]	= imx_clk_composite("usdhc1",  periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, true,  true, base + 0xE0);
	clks[IMX7ULP_CLK_WDG1]		= imx_clk_composite("wdg1",    periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, true,  true, base + 0xF4);
	clks[IMX7ULP_CLK_WDG2]		= imx_clk_composite("sdg2",    periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, true,  true, base + 0x10C);

	/* PCC3 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pcc3");
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks[IMX7ULP_CLK_LPTPM6]	= imx_clk_composite("lptpm6",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x84);
	clks[IMX7ULP_CLK_LPTPM7]	= imx_clk_composite("lptpm7",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x88);
	clks[IMX7ULP_CLK_LPI2C6]	= imx_clk_composite("lpi2c6",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x90);
	clks[IMX7ULP_CLK_LPI2C7]	= imx_clk_composite("lpi2c7",  periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x94);
	clks[IMX7ULP_CLK_LPUART6]	= imx_clk_composite("lpuart6", periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x98);
	clks[IMX7ULP_CLK_LPUART7]	= imx_clk_composite("lpuart7", periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, false, true, base + 0x9C);
	clks[IMX7ULP_CLK_VIU]		= imx_clk_gate("viu", "nic1_div", base + 0xA0, 30);
	clks[IMX7ULP_CLK_DSI]		= imx_clk_composite("dsi",     periph_slow_sels, ARRAY_SIZE(periph_slow_sels), true, true,  true, base + 0xA4);
	clks[IMX7ULP_CLK_LCDIF]		= imx_clk_composite("lcdif",   periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, true,  true, base + 0xA8);
	clks[IMX7ULP_CLK_MMDC]		= imx_clk_gate("mmdc", "nic1_div", base + 0xAC, 30);
	clks[IMX7ULP_CLK_GPU3D]		= imx_clk_composite("gpu3d",   periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, false, true, base + 0x140);
	clks[IMX7ULP_CLK_PCTLC]		= imx_clk_gate("pctlc", "nic1_bus", base + 0xb8, 30);
	clks[IMX7ULP_CLK_PCTLD]		= imx_clk_gate("pctld", "nic1_bus", base + 0xbc, 30);
	clks[IMX7ULP_CLK_PCTLE]		= imx_clk_gate("pctle", "nic1_bus", base + 0xc0, 30);
	clks[IMX7ULP_CLK_PCTLF]		= imx_clk_gate("pctlf", "nic1_bus", base + 0xc4, 30);
	clks[IMX7ULP_CLK_GPU2D]		= imx_clk_composite("gpu2d",   periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, false, true, base + 0x144);

	imx_check_clocks(clks, ARRAY_SIZE(clks));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(scg_node, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clks_init_on); i++)
		imx_clk_prepare_enable(clks[clks_init_on[i]]);
        imx_clk_set_parent(clks[IMX7ULP_CLK_GPU2D], clks[IMX7ULP_CLK_APLL_PFD2]);
        imx_clk_set_parent(clks[IMX7ULP_CLK_GPU3D], clks[IMX7ULP_CLK_APLL_PFD2]);

	/* make sure PFD is gated before setting its rate */
	clk_prepare_enable(clks[IMX7ULP_CLK_APLL_PFD2]);
	clk_disable_unprepare(clks[IMX7ULP_CLK_APLL_PFD2]);
	imx_clk_set_rate(clks[IMX7ULP_CLK_APLL_PFD2], 350000000);

	pr_info("i.MX7ULP clock tree init done.\n");
}

CLK_OF_DECLARE(imx7ulp, "fsl,imx7ulp-scg1", imx7ulp_clocks_init);

static struct clk_div_table apll_pfd0_div_table[] = {
	{ .val = 1, .div = 1, },
	{ .val = 0, .div = 2, },
	{ /* sentinel */ }
};

static u32 share_count_sai0;
static u32 share_count_sai1;

static void __init imx7ulp_cm4_clocks_init(struct device_node *scg_node)
{
	struct device_node *np, *np_sim;
	void __iomem *base;
	void __iomem *base_sim;

	clks_cm4[IMX7ULP_CM4_CLK_DUMMY]		= imx_clk_fixed("cm4_dummy", 0);

	clks_cm4[IMX7ULP_CM4_CLK_ROSC]		= of_clk_get_by_name(scg_node, "cm4_rosc");
	clks_cm4[IMX7ULP_CM4_CLK_SOSC]		= of_clk_get_by_name(scg_node, "cm4_sosc");
	clks_cm4[IMX7ULP_CM4_CLK_SIRC] 		= of_clk_get_by_name(scg_node, "cm4_sirc");
	clks_cm4[IMX7ULP_CM4_CLK_FIRC]		= of_clk_get_by_name(scg_node, "cm4_firc");

	np = scg_node;
	base = of_iomap(np, 0);
	WARN_ON(!base);

	np_sim = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-sim");
	base_sim = of_iomap(np_sim, 0);
	WARN_ON(!base_sim);

	clks_cm4[IMX7ULP_CM4_CLK_SPLL_VCO_PRE_SEL] = imx_clk_mux("cm4_spll_vco_pre_sel", base + 0x608, 0, 1, cm4_pll_pre_sels, ARRAY_SIZE(cm4_pll_pre_sels));
	clks_cm4[IMX7ULP_CM4_CLK_APLL_VCO_PRE_SEL] = imx_clk_mux("cm4_apll_vco_pre_sel", base + 0x508, 0, 1, cm4_pll_pre_sels, ARRAY_SIZE(cm4_pll_pre_sels));
	/*						 name		parent_name	reg		shift 	width */
	clks_cm4[IMX7ULP_CM4_CLK_SPLL_VCO_PRE_DIV] = imx_clk_divider("cm4_spll_vco_pre_div", "cm4_spll_vco_pre_sel", base + 0x608,  8,	3);
	clks_cm4[IMX7ULP_CM4_CLK_APLL_VCO_PRE_DIV] = imx_clk_divider("cm4_apll_vco_pre_div", "cm4_apll_vco_pre_sel", base + 0x508,	8,	3);
	/*					name	parent_name	base*/
	clks_cm4[IMX7ULP_CM4_CLK_SPLL_VCO] = imx_clk_pllv5("cm4_spll_vco",  "cm4_spll_vco_pre_div", base + 0x600);
	clks_cm4[IMX7ULP_CM4_CLK_APLL_VCO] = imx_clk_pllv4("cm4_apll_vco",  "cm4_apll_vco_pre_div", base + 0x500);

	clks_cm4[IMX7ULP_CM4_CLK_APLL_VCO_POST_DIV1] = imx_clk_divider("cm4_apll_vco_post_div1", "cm4_apll_vco", base + 0x508,	24,	4);
	clks_cm4[IMX7ULP_CM4_CLK_APLL_VCO_POST_DIV2] = imx_clk_divider("cm4_apll_vco_post_div2", "cm4_apll_vco_post_div1", base + 0x508,	28,	4);

	/* SPLL PFDs */
	clks_cm4[IMX7ULP_CM4_CLK_SPLL_PFD0] = imx_clk_pfdv2("cm4_spll_pfd0", "cm4_spll_vco", base + 0x60C, 0);
	clks_cm4[IMX7ULP_CM4_CLK_SPLL_PFD1] = imx_clk_pfdv2("cm4_spll_pfd1", "cm4_spll_vco", base + 0x60C, 1);
	clks_cm4[IMX7ULP_CM4_CLK_SPLL_PFD2] = imx_clk_pfdv2("cm4_spll_pfd2", "cm4_spll_vco", base + 0x60C, 2);
	clks_cm4[IMX7ULP_CM4_CLK_SPLL_PFD3] = imx_clk_pfdv2("cm4_spll_pfd3", "cm4_spll_vco", base + 0x60C, 3);
	/* APLL PFDs */
	clks_cm4[IMX7ULP_CM4_CLK_APLL_PFD0] = imx_clk_pfdv2("cm4_apll_pfd0", "cm4_apll_vco", base + 0x50C, 0);
	clks_cm4[IMX7ULP_CM4_CLK_APLL_PFD1] = imx_clk_pfdv2("cm4_apll_pfd1", "cm4_apll_vco", base + 0x50C, 1);
	clks_cm4[IMX7ULP_CM4_CLK_APLL_PFD2] = imx_clk_pfdv2("cm4_apll_pfd2", "cm4_apll_vco", base + 0x50C, 2);
	clks_cm4[IMX7ULP_CM4_CLK_APLL_PFD3] = imx_clk_pfdv2("cm4_apll_pfd3", "cm4_apll_vco", base + 0x50C, 3);

	clks_cm4[IMX7ULP_CM4_CLK_APLL_PFD0_PRE_DIV] = clk_register_divider_table(NULL, "cm4_apll_pfd0_pre_div", "cm4_apll_pfd0", CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, base_sim + 0x2c, 5, 1, 0, apll_pfd0_div_table, &imx_ccm_lock);

	clks_cm4[IMX7ULP_CM4_CLK_SPLL_PFD_SEL] = imx_clk_mux("cm4_spll_pfd_sel", base + 0x608, 14, 2, cm4_spll_pfd_sels, ARRAY_SIZE(cm4_spll_pfd_sels));
	clks_cm4[IMX7ULP_CM4_CLK_APLL_PFD_SEL] = imx_clk_mux("cm4_apll_pfd_sel", base + 0x508, 14, 2, cm4_apll_pfd_sels, ARRAY_SIZE(cm4_apll_pfd_sels));

	clks_cm4[IMX7ULP_CM4_CLK_SPLL_SEL] = imx_clk_mux("cm4_spll_sel", base + 0x608, 1, 1, cm4_spll_sels, ARRAY_SIZE(cm4_spll_sels));
	clks_cm4[IMX7ULP_CM4_CLK_APLL_SEL] = imx_clk_mux("cm4_apll_sel", base + 0x508, 1, 1, cm4_apll_sels, ARRAY_SIZE(cm4_apll_sels));

	clks_cm4[IMX7ULP_CM4_CLK_SYS_SEL]  = imx_clk_mux("cm4_sys_sel", base + 0x14, 24, 4, cm4_sys_sels, ARRAY_SIZE(cm4_sys_sels));

	clks_cm4[IMX7ULP_CM4_CLK_CORE_DIV] = imx_clk_divider("cm4_core_div", "cm4_sys_sel", base + 0x14, 16, 4);
	clks_cm4[IMX7ULP_CM4_CLK_PLAT_DIV] = imx_clk_divider("cm4_plat_div", "cm4_core_div", base + 0x14, 12, 4);
	clks_cm4[IMX7ULP_CM4_CLK_BUS_DIV]  = imx_clk_divider("cm4_bus_div",  "cm4_core_div", base + 0x14, 4, 4);
	clks_cm4[IMX7ULP_CM4_CLK_SLOW_DIV] = imx_clk_divider("cm4_slow_div", "cm4_core_div", base + 0x14, 0, 4);

	clks_cm4[IMX7ULP_CLK_SCG0_CLKOUT] = imx_clk_mux("scg0_clkout", base + 0x20, 24, 4, scg0_clkout_sels, ARRAY_SIZE(scg0_clkout_sels));

	/* PCG0 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pcc0");
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks_cm4[IMX7ULP_CM4_CLK_SAI0_SEL]  = imx_clk_mux("cm4_sai0_sel", base + 0xDC, 24, 3, cm4_periph_slow_sels, ARRAY_SIZE(cm4_periph_slow_sels));
	clks_cm4[IMX7ULP_CM4_CLK_SAI0_DIV]  = imx_clk_divider("cm4_sai0_div", "cm4_sai0_sel", base + 0xDC, 0, 8);
	clks_cm4[IMX7ULP_CM4_CLK_SAI0_ROOT] = imx_clk_gate2_shared("cm4_sai0_root", "cm4_sai0_div", base + 0xDC, 30, &share_count_sai0);
	clks_cm4[IMX7ULP_CM4_CLK_SAI0_IPG]  = imx_clk_gate2_shared("cm4_sai0_ipg", "cm4_bus_div", base + 0xDC, 30, &share_count_sai0);

	/* PCG1 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pcc1");
	base = of_iomap(np, 0);
	WARN_ON(!base);


	clks_cm4[IMX7ULP_CM4_CLK_SAI1_SEL]  = imx_clk_mux("cm4_sai1_sel", base + 0xA8, 24, 3, cm4_periph_slow_sels, ARRAY_SIZE(cm4_periph_slow_sels));
	clks_cm4[IMX7ULP_CM4_CLK_SAI1_DIV]  = imx_clk_divider("cm4_sai1_div", "cm4_sai1_sel", base + 0xA8, 0, 8);
	clks_cm4[IMX7ULP_CM4_CLK_SAI1_ROOT] = imx_clk_gate2_shared("cm4_sai1_root", "cm4_sai1_div", base + 0xA8, 30, &share_count_sai1);
	clks_cm4[IMX7ULP_CM4_CLK_SAI1_IPG] = imx_clk_gate2_shared("cm4_sai1_ipg", "cm4_bus_div", base + 0xA8, 30, &share_count_sai1);

	imx_check_clocks(clks_cm4, ARRAY_SIZE(clks_cm4));

	clk_data_cm4.clks = clks_cm4;
	clk_data_cm4.clk_num = ARRAY_SIZE(clks_cm4);
	of_clk_add_provider(scg_node, of_clk_src_onecell_get, &clk_data_cm4);

	imx_clk_prepare_enable(clks_cm4[IMX7ULP_CM4_CLK_SYS_SEL]);

	pr_info("i.MX7ULP cm4 clock tree init.\n");
}
CLK_OF_DECLARE(imx7ulp_cm4, "fsl,imx7ulp-scg0", imx7ulp_cm4_clocks_init);
