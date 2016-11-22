/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
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

static const char *pll_pre_sels[]	= { "osc", "firc", };
static const char *spll_pfd_sels[]	= { "spll_pfd0", "spll_pfd1", "spll_pfd2", "spll_pfd3", };
static const char *spll_sels[]		= { "spll", "spll_pfd_sel", };
static const char *apll_pfd_sels[]	= { "apll_pfd0", "apll_pfd1", "apll_pfd2", "apll_pfd3", };
static const char *apll_sels[]		= { "apll", "apll_pfd_sel", };
static const char *sys_sels[]		= { "dummy", "osc", "sirc", "firc", "ckil", "apll_sel", "spll_sel", "upll", };
static const char *ddr_sels[]		= { "apll_pfd_sel", "upll", };
static const char *nic_sels[]		= { "firc", "ddr_div", };
static const char *periph_plat_sels[]	= { "dummy", "nic1_bus", "nic1_div", "ddr_div", "apll_pfd2", "apll_pfd1", "apll_pfd0", "upll", };
/* the dummy in only a space holder of spll_bus clk */
static const char *periph_slow_sels[]	= { "dummy", "osc", "mpll", "firc", "ckil", "nic1_bus", "nic1_div", "dummy", };
static struct clk *clks[IMX7ULP_CLK_END];
static struct clk_onecell_data clk_data;

static int const clks_init_on[] __initconst = {
	IMX7ULP_CLK_BUS_DIV,
	IMX7ULP_CLK_PLAT_DIV,
	IMX7ULP_CLK_NIC0_DIV,
	IMX7ULP_CLK_NIC1_DIV,
	IMX7ULP_CLK_NIC1_BUS_DIV,
	IMX7ULP_CLK_MMDC,
	IMX7ULP_CLK_RGPIO2P1,
};

static void __init imx7ulp_clocks_init(struct device_node *scg_node)
{
	struct device_node *np;
	void __iomem *base;
	int i;

	clks[IMX7ULP_CLK_DUMMY]		= imx_clk_fixed("dummy", 0);

	clks[IMX7ULP_CLK_CKIL]		= of_clk_get_by_name(scg_node, "ckil");
	clks[IMX7ULP_CLK_OSC]		= of_clk_get_by_name(scg_node, "osc");
	clks[IMX7ULP_CLK_SIRC]		= of_clk_get_by_name(scg_node, "sirc");
	clks[IMX7ULP_CLK_FIRC]		= of_clk_get_by_name(scg_node, "firc");
	clks[IMX7ULP_CLK_MIPI_PLL]	= of_clk_get_by_name(scg_node, "mpll");
	clks[IMX7ULP_CLK_UPLL]		= of_clk_get_by_name(scg_node, "upll");

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

	clks[IMX7ULP_CLK_SYS_SEL] = imx_clk_mux("sys_sel", base + 0x14, 24, 4, sys_sels, ARRAY_SIZE(sys_sels));
	clks[IMX7ULP_CLK_DDR_SEL] = imx_clk_mux("ddr_sel", base + 0x30, 24, 1, ddr_sels, ARRAY_SIZE(ddr_sels));
	clks[IMX7ULP_CLK_NIC_SEL] = imx_clk_mux("nic_sel", base + 0x40, 28, 1, nic_sels, ARRAY_SIZE(nic_sels));

	clks[IMX7ULP_CLK_CORE_DIV] = imx_clk_divider("core_div", "sys_sel", base + 0x14, 16, 4);
	clks[IMX7ULP_CLK_PLAT_DIV] = imx_clk_divider("plat_div", "core_div", base + 0x14, 12, 4);

	clks[IMX7ULP_CLK_DDR_DIV] = clk_register_divider(NULL, "ddr_div", "ddr_sel", CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, base + 0x30, 0, 3, CLK_DIVIDER_ONE_BASED, &imx_ccm_lock);
	clks[IMX7ULP_CLK_NIC0_DIV] = imx_clk_divider("nic0_div", "nic_sel",  base + 0x40, 24, 4);
	clks[IMX7ULP_CLK_GPU_DIV]  = imx_clk_divider("gpu_div",  "nic0_div", base + 0x40, 20, 4);
	clks[IMX7ULP_CLK_NIC1_DIV] = imx_clk_divider("nic1_div", "nic0_div", base + 0x40, 16, 4);
	clks[IMX7ULP_CLK_NIC1_BUS_DIV] = imx_clk_divider("nic1_bus", "nic1_div", base + 0x40, 4, 4);

	/* PCC2 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pcc2");
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks[IMX7ULP_CLK_DMA1]		= imx_clk_gate("dma1", "nic1_bus", base + 0x20, 30);
	clks[IMX7ULP_CLK_RGPIO2P1]	= imx_clk_gate("gpio", "nic1_bus", base + 0x3c, 30);
	clks[IMX7ULP_CLK_DMA_MUX1]	= imx_clk_gate("dma_mux1", "nic1_bus",	base + 0x84, 30);
	clks[IMX7ULP_CLK_SNVS]		= imx_clk_gate("snvs", "nic1_bus",	base + 0x8c, 30);
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
	clks[IMX7ULP_CLK_GPU2D]		= imx_clk_composite("gpu2d",   periph_plat_sels, ARRAY_SIZE(periph_plat_sels), true, false, true, base + 0x144);

	imx_check_clocks(clks, ARRAY_SIZE(clks));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(scg_node, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clks_init_on); i++)
		imx_clk_prepare_enable(clks[clks_init_on[i]]);

	pr_info("i.MX7ULP clock tree init done.\n");
}

CLK_OF_DECLARE(imx7ulp, "fsl,imx7ulp-scg1", imx7ulp_clocks_init);
