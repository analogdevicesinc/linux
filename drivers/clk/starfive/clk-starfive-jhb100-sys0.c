// SPDX-License-Identifier: GPL-2.0
/*
 * StarFive JHB100 System-0 Clock Driver
 *
 * Copyright (C) 2024 StarFive Technology Co., Ltd.
 *
 * Author: Changhuang Liang <changhuang.liang@starfivetech.com>
 *
 */

#include <dt-bindings/clock/starfive,jhb100-crg.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include "clk-starfive-jh71x0.h"

/* external clocks */
#define JHB100_SYS0CLK_OSC			JH71X0_EXT_PARENT("osc")
#define JHB100_SYS0CLK_PLL0			JH71X0_EXT_PARENT("pll0")
#define JHB100_SYS0CLK_PLL1			JH71X0_EXT_PARENT("pll1")
#define JHB100_SYS0CLK_PLL2			JH71X0_EXT_PARENT("pll2")

static const struct jh71x0_crg_clk_data jhb100_sys0crg_clk_data[] = {
	/* bmcpcierp */
	JH71X0__DIV(JHB100_SYS0CLK_BMCPCIERP_NCNOC_MAIN, "bmcpcierp_ncnoc_main", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_BMCPCIERP_NCNOC_CFG, "bmcpcierp_ncnoc_cfg", 12,
		    JHB100_SYS0CLK_PLL1),
	JH71X0__DIV(JHB100_SYS0CLK_PCIE_REF_CML, "pcie_ref_cml", 24,
		    JHB100_SYS0CLK_PLL0),
	JH71X0_GATE(JHB100_SYS0CLK_BMCPCIERP_NCNOC_DATA_INIT, "bmcpcierp_ncnoc_data_init",
		    0, JH71X0_INT_PARENT(JHB100_SYS0CLK_BMCPCIERP_NCNOC_MAIN)),
	JH71X0_GATE(JHB100_SYS0CLK_BMCPCIERP_NCNOC_CFG_INIT, "bmcpcierp_ncnoc_cfg_init",
		    0, JH71X0_INT_PARENT(JHB100_SYS0CLK_BMCPCIERP_NCNOC_CFG)),
	JH71X0_GATE(JHB100_SYS0CLK_BMCPCIERP_NCNOC_TARG, "bmcpcierp_ncnoc_targ",
		    0, JH71X0_INT_PARENT(JHB100_SYS0CLK_BMCPCIERP_NCNOC_MAIN)),
	JH71X0_GATE(JHB100_SYS0CLK_BMCPCIERP_PCU, "bmcpcierp_pcu", 0, JHB100_SYS0CLK_OSC),
	/* hostss0 */
	JH71X0__DIV(JHB100_SYS0CLK_HOSTSS0_NCNOC_CFG, "hostss0_ncnoc_cfg", 12,
		    JHB100_SYS0CLK_PLL1),
	JH71X0__DIV(JHB100_SYS0CLK_HOSTSS0_NCNOC_DATA, "hostss0_ncnoc_data", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_HOSTSS0_PHY_SCAN_400, "hostss0_phy_scan_400", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_GPIO_ESPI0_EXT, "gpio_espi0_ext", 14,
		    JHB100_SYS0CLK_PLL2),
	/* bmcusb */
	JH71X0__DIV(JHB100_SYS0CLK_BMCUSB_NCNOC_INIT, "bmcusb_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_BMCUSB_NCNOC_TARG, "bmcusb_ncnoc_targ", 6,
		    JHB100_SYS0CLK_PLL1),
	JH71X0__DIV(JHB100_SYS0CLK_BMCUSB_SCANCLK, "bmcusb_scanclk", 5,
		    JHB100_SYS0CLK_PLL0),
	JH71X0_GATE(JHB100_SYS0CLK_BMCUSB_480M_SCANCLK, "bmcusb_480m_scanclk",
		    0, JH71X0_INT_PARENT(JHB100_SYS0CLK_BMCUSB_SCANCLK)),
	/* vce */
	JH71X0__DIV(JHB100_SYS0CLK_VCE_NCNOC_INIT, "vce_ncnoc_init", 10,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_VCE_NCNOC_TARG, "vce_ncnoc_targ", 12,
		    JHB100_SYS0CLK_PLL1),
	/* bmcperiph2 */
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER2_NCNOC_INIT, "bmcper2_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER2_NCNOC_TARG, "bmcper2_ncnoc_targ", 12,
		    JHB100_SYS0CLK_PLL1),
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER2_CFG_400, "bmcper2_cfg_400", 8,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER2_CFG_125, "bmcper2_cfg_125", 10,
		    JHB100_SYS0CLK_PLL1),
	/* hostss1 */
	JH71X0__DIV(JHB100_SYS0CLK_HOSTSS1_NCNOC_DATA, "hostss1_ncnoc_data", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_HOSTSS1_PHY_SCAN_400, "hostss1_phy_scan_400", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0_GATE(JHB100_SYS0CLK_HOSTSS1_PHY_SCAN_400_ICG_BUF,
		    "hostss1_phy_scan_400_icg_buf", 0,
		    JH71X0_INT_PARENT(JHB100_SYS0CLK_HOSTSS1_PHY_SCAN_400)),
	/* npu */
	JH71X0__DIV(JHB100_SYS0CLK_NPU_NCNOC_INIT, "npu_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	/* vout */
	JH71X0__DIV(JHB100_SYS0CLK_VOUT_NCNOC_INIT, "vout_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_VOUT_AUX, "vout_aux", 150,
		    JHB100_SYS0CLK_PLL0),
	/* bmcperiph3 */
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER3_NCNOC_INIT, "bmcper3_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	/* hostusb */
	JH71X0__DIV(JHB100_SYS0CLK_HOSTUSB_NCNOC_INIT, "hostusb_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	/* hostusbcmn */
	JH71X0__DIV(JHB100_SYS0CLK_HOSTUSBCMN_CFG_480, "hostusbcmn_cfg_480", 5,
		    JHB100_SYS0CLK_PLL0),
	/* bmcperiph1 */
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER1_NCNOC_INIT, "bmcper1_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER1_CFG_800, "bmcper1_cfg_800", 4,
		    JHB100_SYS0CLK_PLL0),
	/* bmcperiph0 */
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER0_NCNOC_INIT, "bmcper0_ncnoc_init", 6,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER0_CFG_400, "bmcper0_cfg_400", 8,
		    JHB100_SYS0CLK_PLL0),
	JH71X0__DIV(JHB100_SYS0CLK_BMCPER0_CFG_800, "bmcper0_cfg_800", 8,
		    JHB100_SYS0CLK_PLL0),
	/* gpu0 */
	JH71X0__DIV(JHB100_SYS0CLK_GPU0_NCNOC_INIT, "gpu0_ncnoc_init", 10,
		    JHB100_SYS0CLK_PLL0),
	/* gpu1 */
	JH71X0__DIV(JHB100_SYS0CLK_GPU1_NCNOC_INIT, "gpu1_ncnoc_init", 10,
		    JHB100_SYS0CLK_PLL0),
};

static const struct jh71x0_crg_domain_info jhb100_sys0crg_info = {
	.clk_data	= jhb100_sys0crg_clk_data,
	.num_clk	= ARRAY_SIZE(jhb100_sys0crg_clk_data),
	.rst_name	= "jhb100-r-sys0",
};

static const struct of_device_id jhb100_sys0crg_match[] = {
	{
		.compatible = "starfive,jhb100-sys0crg",
		.data = &jhb100_sys0crg_info,
	},
	{ /* sentinel */ }
};

static struct platform_driver jhb100_sys0crg_driver = {
	.probe = jh71x0_crg_probe,
	.driver = {
		.name = "clk-starfive-jhb100-sys0",
		.of_match_table = jhb100_sys0crg_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(jhb100_sys0crg_driver);
