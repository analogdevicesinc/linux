/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <dt-bindings/clock/imx8qxp-clock.h>
#include <dt-bindings/soc/imx8_pd.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

#include <soc/imx8/imx8qxp/lpcg.h>
#include <soc/imx8/sc/sci.h>
#include "clk-imx8.h"

#define STR_VALUE(arg)      #arg
#define FUNCTION_NAME(name) STR_VALUE(name)

static const char *aud_clk_sels[] = {
	"aud_acm_aud_rec_clk0_clk",
	"aud_acm_aud_rec_clk1_clk",
	"ext_aud_mclk0",
	"ext_aud_mclk1",
	"esai0_rx_clk",
	"esai0_rx_hf_clk",
	"esai0_tx_clk",
	"esai0_tx_hf_clk",
	"spdif0_rx",
	"sai0_rx_bclk",
	"sai0_tx_bclk",
	"sai1_rx_bclk",
	"sai1_tx_bclk",
	"sai2_rx_bclk",
	"sai3_rx_bclk",
};

static const char *mclk_out_sels[] = {
	"aud_acm_aud_rec_clk0_clk",
	"aud_acm_aud_rec_clk1_clk",
	"dummy",
	"dummy",
	"spdif0_rx",
	"dummy",
	"dummy",
	"sai4_rx_bclk",
};

static const char *sai_mclk_sels[] = {
	"aud_acm_aud_pll_clk0_clk",
	"aud_acm_aud_pll_clk1_clk",
	"acm_aud_clk0_clk",
	"acm_aud_clk1_clk",
};

static const char *esai_mclk_sels[] = {
	"aud_acm_aud_pll_clk0_clk",
	"aud_acm_aud_pll_clk1_clk",
	"acm_aud_clk0_clk",
	"acm_aud_clk1_clk",
};

static const char *spdif_mclk_sels[] = {
	"aud_acm_aud_pll_clk0_clk",
	"aud_acm_aud_pll_clk1_clk",
	"acm_aud_clk0_clk",
	"acm_aud_clk1_clk",
};

static const char *mqs_mclk_sels[] = {
	"aud_acm_aud_pll_clk0_clk",
	"aud_acm_aud_pll_clk1_clk",
	"acm_aud_clk0_clk",
	"acm_aud_clk1_clk",
};

static const char *mipi0_sels[] = {
	"dummy",
	"mipi0_pll_clk",
	"mipi0_pll_div2_clk",
	"dummy",
	"mipi0_lvds_bypass_clk",
};

static const char *mipi1_sels[] = {
	"dummy",
	"mipi1_pll_clk",
	"mipi1_pll_div2_clk",
	"dummy",
	"mipi1_lvds_bypass_clk",
};
static struct clk *clks[IMX8QXP_CLK_END];
static struct clk_onecell_data clk_data;

static const char *enet_sels[] = { "enet_25MHz", "enet_125MHz", };
static const char *enet0_rmii_tx_sels[] = { "enet0_ref_div", "dummy", };
static const char *enet1_rmii_tx_sels[] = { "enet1_ref_div", "dummy", };

static int imx8qxp_clk_probe(struct platform_device *pdev)
{
	struct device_node *ccm_node = pdev->dev.of_node;
	struct device_node *np_acm;
	void __iomem *base_acm;
	int i, ret;

	ret = imx8_clk_mu_init();
	if (ret)
		return ret;

	pr_info("***** imx8qxp_clocks_init *****\n");

	/* Fixed clocks */
	clks[IMX8QXP_CLK_DUMMY] = imx_clk_fixed("dummy", 0);
	clks[IMX8QXP_24MHZ]	= imx_clk_fixed("xtal_24MHz", SC_24MHZ);
	clks[IMX8QXP_GPT_3M]	= imx_clk_fixed("gpt_3m", 3000000);
	clks[IMX8QXP_32KHZ]	= imx_clk_fixed("xtal_32KHz", SC_32KHZ);

	/* ARM core */
	clks[IMX8QXP_A35_DIV] = imx_clk_divider_scu("a35_div", SC_R_A35, SC_PM_CLK_CPU);

	/* User Defined PLLs dividers */
	clks[IMX8QXP_AUD_PLL0_DIV] = imx_clk_divider_scu("audio_pll0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_PLL);
	clks[IMX8QXP_AUD_PLL1_DIV] = imx_clk_divider_scu("audio_pll1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_PLL);

	/* User Defined PLLs clocks */
	clks[IMX8QXP_AUD_PLL0]     = imx_clk_gate_scu("audio_pll0_clk", "audio_pll0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QXP_AUD_PLL1]     = imx_clk_gate_scu("audio_pll1_clk", "audio_pll1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);

	clks[IMX8QXP_IPG_DMA_CLK_ROOT] = imx_clk_fixed("ipg_dma_clk_root", SC_120MHZ);
	clks[IMX8QXP_IPG_AUD_CLK_ROOT] = imx_clk_fixed("ipg_aud_clk_root", SC_150MHZ);
	clks[IMX8QXP_AXI_CONN_CLK_ROOT] = imx_clk_fixed("axi_conn_clk_root", SC_333MHZ);
	clks[IMX8QXP_AHB_CONN_CLK_ROOT] = imx_clk_fixed("ahb_conn_clk_root", SC_166MHZ);
	clks[IMX8QXP_IPG_CONN_CLK_ROOT] = imx_clk_fixed("ipg_conn_clk_root", SC_83MHZ);
	clks[IMX8QXP_DC_AXI_EXT_CLK]	= imx_clk_fixed("axi_ext_dc_clk_root", SC_800MHZ);
	clks[IMX8QXP_DC_AXI_INT_CLK]	= imx_clk_fixed("axi_int_dc_clk_root", SC_400MHZ);
	clks[IMX8QXP_DC_CFG_CLK]	= imx_clk_fixed("cfg_dc_clk_root", SC_100MHZ);
	clks[IMX8QXP_MIPI_IPG_CLK]	= imx_clk_fixed("ipg_mipi_clk_root", SC_120MHZ);
	clks[IMX8QXP_IMG_AXI_CLK]	= imx_clk_fixed("axi_img_clk_root", SC_400MHZ);
	clks[IMX8QXP_IMG_IPG_CLK]	= imx_clk_fixed("ipg_img_clk_root", SC_200MHZ);
	clks[IMX8QXP_IMG_PXL_CLK]	= imx_clk_fixed("pxl_img_clk_root", SC_600MHZ);
	clks[IMX8QXP_HSIO_AXI_CLK]	= imx_clk_fixed("axi_hsio_clk_root", SC_400MHZ);
	clks[IMX8QXP_HSIO_PER_CLK]	= imx_clk_fixed("per_hsio_clk_root", SC_133MHZ);
	clks[IMX8QXP_CM40_IPG_CLK]	= imx_clk_fixed("ipg_cm40_clk_root", SC_132MHZ);
	clks[IMX8QXP_MIPI0_DSI_PLL_CLK]		= imx_clk_fixed("mipi0_pll_clk", SC_864MHZ);
	clks[IMX8QXP_MIPI0_DSI_PLL_DIV2_CLK]	= imx_clk_fixed("mipi0_pll_div2_clk", SC_432MHZ);
	clks[IMX8QXP_MIPI1_DSI_PLL_CLK]		= imx_clk_fixed("mipi1_pll_clk", SC_864MHZ);
	clks[IMX8QXP_MIPI1_DSI_PLL_DIV2_CLK]	= imx_clk_fixed("mipi1_pll_div2_clk", SC_432MHZ);

	clks[IMX8QXP_UART0_DIV] = imx_clk_divider_scu("uart0_div", SC_R_UART_0, SC_PM_CLK_PER);
	clks[IMX8QXP_UART0_IPG_CLK] = imx_clk_gate2_scu("uart0_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPUART_0_LPCG), 16, FUNCTION_NAME(PD_DMA_UART0));
	clks[IMX8QXP_UART0_CLK] = imx_clk_gate_scu("uart0_clk", "uart0_div", SC_R_UART_0, SC_PM_CLK_PER, (void __iomem *)(LPUART_0_LPCG), 0, 0);

	clks[IMX8QXP_GPU0_CORE_DIV] = imx_clk_divider_scu("gpu_core0_div", SC_R_GPU_0_PID0, SC_PM_CLK_PER);
	clks[IMX8QXP_GPU0_SHADER_DIV] = imx_clk_divider_scu("gpu_shader0_div", SC_R_GPU_0_PID0, SC_PM_CLK_MISC);
	clks[IMX8QXP_GPU0_CORE_CLK] = imx_clk_gate_scu("gpu_core0_clk", "gpu_core0_div", SC_R_GPU_0_PID0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_GPU0_SHADER_CLK] = imx_clk_gate_scu("gpu_shader0_clk", "gpu_shader0_div", SC_R_GPU_0_PID0, SC_PM_CLK_MISC, NULL, 0, 0);

	/* LSIO SS */
	clks[IMX8QXP_LSIO_MEM_CLK] = imx_clk_fixed("lsio_mem_clk_root", SC_100MHZ);
	clks[IMX8QXP_LSIO_BUS_CLK] = imx_clk_fixed("lsio_bus_clk_root", SC_200MHZ);

	clks[IMX8QXP_LSIO_PWM0_DIV] = imx_clk_divider_scu("pwm_0_div", SC_R_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM0_IPG_S_CLK] = imx_clk_gate_scu("pwm_0_ipg_s_clk", "pwm_0_div", SC_R_PWM_0, SC_PM_CLK_PER, (void __iomem *)(PWM_0_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM0_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_0_ipg_slv_clk", "pwm_0_ipg_s_clk", SC_R_PWM_0, SC_PM_CLK_PER, (void __iomem *)(PWM_0_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM0_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_0_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_0_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_0));
	clks[IMX8QXP_LSIO_PWM0_HF_CLK] = imx_clk_gate_scu("pwm_0_hf_clk", "pwm_0_ipg_slv_clk", SC_R_PWM_0, SC_PM_CLK_PER, (void __iomem *)(PWM_0_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM0_CLK] = imx_clk_gate_scu("pwm_0_clk", "pwm_0_ipg_slv_clk", SC_R_PWM_0, SC_PM_CLK_PER, (void __iomem *)(PWM_0_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_PWM1_DIV] = imx_clk_divider_scu("pwm_1_div", SC_R_PWM_1, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM1_IPG_S_CLK] = imx_clk_gate_scu("pwm_1_ipg_s_clk", "pwm_1_div", SC_R_PWM_1, SC_PM_CLK_PER, (void __iomem *)(PWM_1_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM1_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_1_ipg_slv_clk", "pwm_1_ipg_s_clk", SC_R_PWM_1, SC_PM_CLK_PER, (void __iomem *)(PWM_1_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM1_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_1_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_1_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_1));
	clks[IMX8QXP_LSIO_PWM1_HF_CLK] = imx_clk_gate_scu("pwm_1_hf_clk", "pwm_1_ipg_slv_clk", SC_R_PWM_1, SC_PM_CLK_PER, (void __iomem *)(PWM_1_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM1_CLK] = imx_clk_gate_scu("pwm_1_clk", "pwm_1_ipg_slv_clk", SC_R_PWM_1, SC_PM_CLK_PER, (void __iomem *)(PWM_1_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_PWM2_DIV] = imx_clk_divider_scu("pwm_2_div", SC_R_PWM_2, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM2_IPG_S_CLK] = imx_clk_gate_scu("pwm_2_ipg_s_clk", "pwm_2_div", SC_R_PWM_2, SC_PM_CLK_PER, (void __iomem *)(PWM_2_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM2_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_2_ipg_slv_clk", "pwm_2_ipg_s_clk", SC_R_PWM_2, SC_PM_CLK_PER, (void __iomem *)(PWM_2_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM2_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_2_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_2_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_2));
	clks[IMX8QXP_LSIO_PWM2_HF_CLK] = imx_clk_gate_scu("pwm_2_hf_clk", "pwm_2_ipg_slv_clk", SC_R_PWM_2, SC_PM_CLK_PER, (void __iomem *)(PWM_2_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM2_CLK] = imx_clk_gate_scu("pwm_2_clk", "pwm_2_ipg_slv_clk", SC_R_PWM_2, SC_PM_CLK_PER, (void __iomem *)(PWM_2_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_PWM3_DIV] = imx_clk_divider_scu("pwm_3_div", SC_R_PWM_3, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM3_IPG_S_CLK] = imx_clk_gate_scu("pwm_3_ipg_s_clk", "pwm_3_div", SC_R_PWM_3, SC_PM_CLK_PER, (void __iomem *)(PWM_3_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM3_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_3_ipg_slv_clk", "pwm_3_ipg_s_clk", SC_R_PWM_3, SC_PM_CLK_PER, (void __iomem *)(PWM_3_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM3_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_3_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_3_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_3));
	clks[IMX8QXP_LSIO_PWM3_HF_CLK] = imx_clk_gate_scu("pwm_3_hf_clk", "pwm_3_ipg_slv_clk", SC_R_PWM_3, SC_PM_CLK_PER, (void __iomem *)(PWM_3_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM3_CLK] = imx_clk_gate_scu("pwm_3_clk", "pwm_3_ipg_slv_clk", SC_R_PWM_3, SC_PM_CLK_PER, (void __iomem *)(PWM_3_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_PWM4_DIV] = imx_clk_divider_scu("pwm_4_div", SC_R_PWM_4, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM4_IPG_S_CLK] = imx_clk_gate_scu("pwm_4_ipg_s_clk", "pwm_4_div", SC_R_PWM_4, SC_PM_CLK_PER, (void __iomem *)(PWM_4_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM4_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_4_ipg_slv_clk", "pwm_4_ipg_s_clk", SC_R_PWM_4, SC_PM_CLK_PER, (void __iomem *)(PWM_4_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM4_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_4_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_4_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_4));
	clks[IMX8QXP_LSIO_PWM4_HF_CLK] = imx_clk_gate_scu("pwm_4_hf_clk", "pwm_4_ipg_slv_clk", SC_R_PWM_4, SC_PM_CLK_PER, (void __iomem *)(PWM_4_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM4_CLK] = imx_clk_gate_scu("pwm_4_clk", "pwm_4_ipg_slv_clk", SC_R_PWM_4, SC_PM_CLK_PER, (void __iomem *)(PWM_4_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_PWM5_DIV] = imx_clk_divider_scu("pwm_5_div", SC_R_PWM_5, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM5_IPG_S_CLK] = imx_clk_gate_scu("pwm_5_ipg_s_clk", "pwm_5_div", SC_R_PWM_5, SC_PM_CLK_PER, (void __iomem *)(PWM_5_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM5_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_5_ipg_slv_clk", "pwm_5_ipg_s_clk", SC_R_PWM_5, SC_PM_CLK_PER, (void __iomem *)(PWM_5_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM5_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_5_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_5_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_5));
	clks[IMX8QXP_LSIO_PWM5_HF_CLK] = imx_clk_gate_scu("pwm_5_hf_clk", "pwm_5_ipg_slv_clk", SC_R_PWM_5, SC_PM_CLK_PER, (void __iomem *)(PWM_5_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM5_CLK] = imx_clk_gate_scu("pwm_5_clk", "pwm_5_ipg_slv_clk", SC_R_PWM_5, SC_PM_CLK_PER, (void __iomem *)(PWM_5_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_PWM6_DIV] = imx_clk_divider_scu("pwm_6_div", SC_R_PWM_6, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM6_IPG_S_CLK] = imx_clk_gate_scu("pwm_6_ipg_s_clk", "pwm_6_div", SC_R_PWM_6, SC_PM_CLK_PER, (void __iomem *)(PWM_6_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM6_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_6_ipg_slv_clk", "pwm_6_ipg_s_clk", SC_R_PWM_6, SC_PM_CLK_PER, (void __iomem *)(PWM_6_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM6_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_6_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_6_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_6));
	clks[IMX8QXP_LSIO_PWM6_HF_CLK] = imx_clk_gate_scu("pwm_6_hf_clk", "pwm_6_ipg_slv_clk", SC_R_PWM_6, SC_PM_CLK_PER, (void __iomem *)(PWM_6_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM6_CLK] = imx_clk_gate_scu("pwm_6_clk", "pwm_6_ipg_slv_clk", SC_R_PWM_6, SC_PM_CLK_PER, (void __iomem *)(PWM_6_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_PWM7_DIV] = imx_clk_divider_scu("pwm_7_div", SC_R_PWM_7, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_PWM7_IPG_S_CLK] = imx_clk_gate_scu("pwm_7_ipg_s_clk", "pwm_7_div", SC_R_PWM_7, SC_PM_CLK_PER, (void __iomem *)(PWM_7_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_PWM7_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_7_ipg_slv_clk", "pwm_7_ipg_s_clk", SC_R_PWM_7, SC_PM_CLK_PER, (void __iomem *)(PWM_7_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_PWM7_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_7_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(PWM_7_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_7));
	clks[IMX8QXP_LSIO_PWM7_HF_CLK] = imx_clk_gate_scu("pwm_7_hf_clk", "pwm_7_ipg_slv_clk", SC_R_PWM_7, SC_PM_CLK_PER, (void __iomem *)(PWM_7_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_PWM7_CLK] = imx_clk_gate_scu("pwm_7_clk", "pwm_7_ipg_slv_clk", SC_R_PWM_7, SC_PM_CLK_PER, (void __iomem *)(PWM_7_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_GPT0_DIV] = imx_clk_divider_scu("gpt_0_div", SC_R_GPT_4, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_GPT0_IPG_S_CLK] = imx_clk_gate_scu("gpt_0_ipg_s_clk", "gpt_0_div", SC_R_GPT_0, SC_PM_CLK_PER, (void __iomem *)(GPT_0_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_GPT0_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_0_ipg_slv_clk", "gpt_0_ipg_s_clk", SC_R_GPT_0, SC_PM_CLK_PER, (void __iomem *)(GPT_0_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_GPT0_CLK] = imx_clk_gate_scu("gpt_0_clk", "gpt_0_ipg_slv_clk", SC_R_GPT_0, SC_PM_CLK_PER, (void __iomem *)(GPT_0_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_GPT0_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_0_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(GPT_0_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_0));
	clks[IMX8QXP_LSIO_GPT0_HF_CLK] = imx_clk_gate_scu("gpt_0_hf_clk", "gpt_0_ipg_slv_clk", SC_R_GPT_0, SC_PM_CLK_PER, (void __iomem *)(GPT_0_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_GPT1_DIV] = imx_clk_divider_scu("gpt_1_div", SC_R_GPT_4, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_GPT1_IPG_S_CLK] = imx_clk_gate_scu("gpt_1_ipg_s_clk", "gpt_1_div", SC_R_GPT_1, SC_PM_CLK_PER, (void __iomem *)(GPT_1_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_GPT1_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_1_ipg_slv_clk", "gpt_1_ipg_s_clk", SC_R_GPT_1, SC_PM_CLK_PER, (void __iomem *)(GPT_1_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_GPT1_CLK] = imx_clk_gate_scu("gpt_1_clk", "gpt_1_ipg_slv_clk", SC_R_GPT_1, SC_PM_CLK_PER, (void __iomem *)(GPT_1_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_GPT1_HF_CLK] = imx_clk_gate_scu("gpt_1_hf_clk", "gpt_1_ipg_slv_clk", SC_R_GPT_1, SC_PM_CLK_PER, (void __iomem *)(GPT_1_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_GPT1_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_1_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(GPT_1_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_1));
	clks[IMX8QXP_LSIO_GPT2_DIV] = imx_clk_divider_scu("gpt_2_div", SC_R_GPT_4, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_GPT2_IPG_S_CLK] = imx_clk_gate_scu("gpt_2_ipg_s_clk", "gpt_2_div", SC_R_GPT_2, SC_PM_CLK_PER, (void __iomem *)(GPT_2_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_GPT2_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_2_ipg_slv_clk", "gpt_2_ipg_s_clk", SC_R_GPT_2, SC_PM_CLK_PER, (void __iomem *)(GPT_2_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_GPT2_CLK] = imx_clk_gate_scu("gpt_2_clk", "gpt_2_ipg_slv_clk", SC_R_GPT_2, SC_PM_CLK_PER, (void __iomem *)(GPT_2_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_GPT2_HF_CLK] = imx_clk_gate_scu("gpt_2_hf_clk", "gpt_2_ipg_slv_clk", SC_R_GPT_2, SC_PM_CLK_PER, (void __iomem *)(GPT_2_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_GPT2_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_2_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(GPT_2_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_2));
	clks[IMX8QXP_LSIO_GPT3_DIV] = imx_clk_divider_scu("gpt_3_div", SC_R_GPT_4, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_GPT3_IPG_S_CLK] = imx_clk_gate_scu("gpt_3_ipg_s_clk", "gpt_3_div", SC_R_GPT_3, SC_PM_CLK_PER, (void __iomem *)(GPT_3_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_GPT3_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_3_ipg_slv_clk", "gpt_3_ipg_s_clk", SC_R_GPT_3, SC_PM_CLK_PER, (void __iomem *)(GPT_3_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_GPT3_CLK] = imx_clk_gate_scu("gpt_3_clk", "gpt_3_ipg_slv_clk", SC_R_GPT_3, SC_PM_CLK_PER, (void __iomem *)(GPT_3_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_GPT3_HF_CLK] = imx_clk_gate_scu("gpt_3_hf_clk", "gpt_3_ipg_slv_clk", SC_R_GPT_3, SC_PM_CLK_PER, (void __iomem *)(GPT_3_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_GPT3_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_3_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(GPT_3_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_3));
	clks[IMX8QXP_LSIO_GPT4_DIV] = imx_clk_divider_scu("gpt_4_div", SC_R_GPT_4, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_GPT4_IPG_S_CLK] = imx_clk_gate_scu("gpt_4_ipg_s_clk", "gpt_4_div", SC_R_GPT_4, SC_PM_CLK_PER, (void __iomem *)(GPT_4_LPCG), 0x10, 0);
	clks[IMX8QXP_LSIO_GPT4_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_4_ipg_slv_clk", "gpt_4_ipg_s_clk", SC_R_GPT_4, SC_PM_CLK_PER, (void __iomem *)(GPT_4_LPCG), 0x14, 0);
	clks[IMX8QXP_LSIO_GPT4_CLK] = imx_clk_gate_scu("gpt_4_clk", "gpt_4_ipg_slv_clk", SC_R_GPT_4, SC_PM_CLK_PER, (void __iomem *)(GPT_4_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_GPT4_HF_CLK] = imx_clk_gate_scu("gpt_4_hf_clk", "gpt_4_ipg_slv_clk", SC_R_GPT_4, SC_PM_CLK_PER, (void __iomem *)(GPT_4_LPCG), 4, 0);
	clks[IMX8QXP_LSIO_GPT4_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_4_ipg_mstr_clk", "lsio_bus_clk_root", (void __iomem *)(GPT_4_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_4));
	clks[IMX8QXP_LSIO_FSPI0_DIV] = imx_clk_divider_scu("fspi_0_div", SC_R_FSPI_0, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_FSPI0_HCLK] = imx_clk_gate2_scu("fspi0_hclk_clk", "lsio_mem_clk_root", (void __iomem *)(FSPI_0_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_FSPI_0));
	clks[IMX8QXP_LSIO_FSPI0_IPG_S_CLK] = imx_clk_gate2_scu("fspi0_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(FSPI_0_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_FSPI_0));
	clks[IMX8QXP_LSIO_FSPI0_IPG_CLK] = imx_clk_gate2_scu("fspi0_ipg_clk", "fspi0_ipg_s_clk", (void __iomem *)(FSPI_0_LPCG), 0x14, FUNCTION_NAME(PD_LSIO_FSPI_0));
	clks[IMX8QXP_LSIO_FSPI0_CLK] = imx_clk_gate_scu("fspi_0_clk", "fspi_0_div", SC_R_FSPI_0, SC_PM_CLK_PER, (void __iomem *)(FSPI_0_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_FSPI1_DIV] = imx_clk_divider_scu("fspi_1_div", SC_R_FSPI_1, SC_PM_CLK_PER);
	clks[IMX8QXP_LSIO_FSPI1_HCLK] = imx_clk_gate2_scu("fspi1_hclk_clk", "lsio_mem_clk_root", (void __iomem *)(FSPI_1_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_FSPI_1));
	clks[IMX8QXP_LSIO_FSPI1_IPG_S_CLK] = imx_clk_gate2_scu("fspi1_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(FSPI_1_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_FSPI_1));
	clks[IMX8QXP_LSIO_FSPI1_IPG_CLK] = imx_clk_gate2_scu("fspi1_ipg_clk", "fspi1_ipg_s_clk", (void __iomem *)(FSPI_1_LPCG), 0x14, FUNCTION_NAME(PD_LSIO_FSPI_1));
	clks[IMX8QXP_LSIO_FSPI1_CLK] = imx_clk_gate_scu("fspi_1_clk", "fspi_1_div", SC_R_FSPI_1, SC_PM_CLK_PER, (void __iomem *)(FSPI_1_LPCG), 0, 0);
	clks[IMX8QXP_LSIO_GPIO0_IPG_S_CLK] = imx_clk_gate2_scu("gpio0_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_0_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_0));
	clks[IMX8QXP_LSIO_GPIO1_IPG_S_CLK] = imx_clk_gate2_scu("gpio1_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_1_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_1));
	clks[IMX8QXP_LSIO_GPIO2_IPG_S_CLK] = imx_clk_gate2_scu("gpio2_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_2_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_2));
	clks[IMX8QXP_LSIO_GPIO3_IPG_S_CLK] = imx_clk_gate2_scu("gpio3_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_3_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_3));
	clks[IMX8QXP_LSIO_GPIO4_IPG_S_CLK] = imx_clk_gate2_scu("gpio4_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_4_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_4));
	clks[IMX8QXP_LSIO_GPIO5_IPG_S_CLK] = imx_clk_gate2_scu("gpio5_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_5_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_5));
	clks[IMX8QXP_LSIO_GPIO6_IPG_S_CLK] = imx_clk_gate2_scu("gpio6_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_6_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_6));
	clks[IMX8QXP_LSIO_GPIO7_IPG_S_CLK] = imx_clk_gate2_scu("gpio7_ipg_s_clk", "lsio_bus_clk_root", (void __iomem *)(GPIO_7_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_7));
	clks[IMX8QXP_LSIO_ROMCP_REG_CLK] = imx_clk_gate2_scu("romcp_reg_clk", "lsio_bus_clk_root", (void __iomem *)(ROMCP_LPCG), 0x10, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QXP_LSIO_ROMCP_CLK] = imx_clk_gate2_scu("romcp_clk", "lsio_mem_clk_root", (void __iomem *)(ROMCP_LPCG), 0x0, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QXP_LSIO_96KROM_CLK] = imx_clk_gate2_scu("96krom_clk", "lsio_mem_clk_root", (void __iomem *)(ROMCP_LPCG), 0x4, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QXP_LSIO_OCRAM_MEM_CLK] = imx_clk_gate2_scu("ocram_lk", "lsio_mem_clk_root", (void __iomem *)(OCRAM_LPCG), 0x4, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QXP_LSIO_OCRAM_CTRL_CLK] = imx_clk_gate2_scu("ocram_ctrl_clk", "lsio_mem_clk_root", (void __iomem *)(OCRAM_LPCG), 0x0, FUNCTION_NAME(PD_LSIO));

	/* ADMA SS */
	clks[IMX8QXP_UART1_IPG_CLK] = imx_clk_gate2_scu("uart1_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPUART_1_LPCG), 16, FUNCTION_NAME(PD_DMA_UART1));
	clks[IMX8QXP_UART2_IPG_CLK] = imx_clk_gate2_scu("uart2_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPUART_2_LPCG), 16, FUNCTION_NAME(PD_DMA_UART2));
	clks[IMX8QXP_UART3_IPG_CLK] = imx_clk_gate2_scu("uart3_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPUART_3_LPCG), 16, FUNCTION_NAME(PD_DMA_UART3));
	clks[IMX8QXP_UART1_DIV] = imx_clk_divider_scu("uart1_div", SC_R_UART_1, SC_PM_CLK_PER);
	clks[IMX8QXP_UART2_DIV] = imx_clk_divider_scu("uart2_div", SC_R_UART_2, SC_PM_CLK_PER);
	clks[IMX8QXP_UART3_DIV] = imx_clk_divider_scu("uart3_div", SC_R_UART_3, SC_PM_CLK_PER);
	clks[IMX8QXP_UART1_CLK] = imx_clk_gate_scu("uart1_clk", "uart1_div", SC_R_UART_1, SC_PM_CLK_PER, (void __iomem *)(LPUART_1_LPCG), 0, 0);
	clks[IMX8QXP_UART2_CLK] = imx_clk_gate_scu("uart2_clk", "uart2_div", SC_R_UART_2, SC_PM_CLK_PER, (void __iomem *)(LPUART_2_LPCG), 0, 0);
	clks[IMX8QXP_UART3_CLK] = imx_clk_gate_scu("uart3_clk", "uart3_div", SC_R_UART_3, SC_PM_CLK_PER, (void __iomem *)(LPUART_3_LPCG), 0, 0);
	clks[IMX8QXP_SPI0_IPG_CLK]   = imx_clk_gate2_scu("spi0_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPSPI_0_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_0));
	clks[IMX8QXP_SPI1_IPG_CLK]   = imx_clk_gate2_scu("spi1_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPSPI_1_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_1));
	clks[IMX8QXP_SPI2_IPG_CLK]   = imx_clk_gate2_scu("spi2_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPSPI_2_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_2));
	clks[IMX8QXP_SPI3_IPG_CLK]   = imx_clk_gate2_scu("spi3_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPSPI_3_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_3));
	clks[IMX8QXP_SPI0_DIV] = imx_clk_divider_scu("spi0_div", SC_R_SPI_0, SC_PM_CLK_PER);
	clks[IMX8QXP_SPI1_DIV] = imx_clk_divider_scu("spi1_div", SC_R_SPI_1, SC_PM_CLK_PER);
	clks[IMX8QXP_SPI2_DIV] = imx_clk_divider_scu("spi2_div", SC_R_SPI_2, SC_PM_CLK_PER);
	clks[IMX8QXP_SPI3_DIV] = imx_clk_divider_scu("spi3_div", SC_R_SPI_3, SC_PM_CLK_PER);
	clks[IMX8QXP_SPI0_CLK] = imx_clk_gate_scu("spi0_clk", "spi0_div", SC_R_SPI_0, SC_PM_CLK_PER, (void __iomem *)(LPSPI_0_LPCG), 0, 0);
	clks[IMX8QXP_SPI1_CLK] = imx_clk_gate_scu("spi1_clk", "spi1_div", SC_R_SPI_2, SC_PM_CLK_PER, (void __iomem *)(LPSPI_1_LPCG), 0, 0);
	clks[IMX8QXP_SPI2_CLK] = imx_clk_gate_scu("spi2_clk", "spi2_div", SC_R_SPI_2, SC_PM_CLK_PER, (void __iomem *)(LPSPI_2_LPCG), 0, 0);
	clks[IMX8QXP_SPI3_CLK] = imx_clk_gate_scu("spi3_clk", "spi3_div", SC_R_SPI_3, SC_PM_CLK_PER, (void __iomem *)(LPSPI_3_LPCG), 0, 0);
	clks[IMX8QXP_CAN0_IPG_CHI_CLK]   = imx_clk_gate2_scu("can0_ipg_chi_clk", "ipg_dma_clk_root", (void __iomem *)(FLEX_CAN_0_LPCG), 20, FUNCTION_NAME(PD_DMA_CAN_0));
	clks[IMX8QXP_CAN0_IPG_CLK]   = imx_clk_gate2_scu("can0_ipg_clk", "can0_ipg_chi_clk", (void __iomem *)(FLEX_CAN_0_LPCG), 16, FUNCTION_NAME(PD_DMA_CAN_0));
	clks[IMX8QXP_CAN1_IPG_CHI_CLK]   = imx_clk_gate2_scu("can1_ipg_chi_clk", "ipg_dma_clk_root", (void __iomem *)(FLEX_CAN_1_LPCG), 20, FUNCTION_NAME(PD_DMA_CAN_1));
	clks[IMX8QXP_CAN1_IPG_CLK]   = imx_clk_gate2_scu("can1_ipg_clk", "can1_ipg_chi_clk", (void __iomem *)(FLEX_CAN_1_LPCG), 16, FUNCTION_NAME(PD_DMA_CAN_1));
	clks[IMX8QXP_CAN2_IPG_CHI_CLK]   = imx_clk_gate2_scu("can2_ipg_chi_clk", "ipg_dma_clk_root", (void __iomem *)(FLEX_CAN_2_LPCG), 20, FUNCTION_NAME(PD_DMA_CAN_2));
	clks[IMX8QXP_CAN2_IPG_CLK]   = imx_clk_gate2_scu("can2_ipg_clk", "can2_ipg_chi_clk", (void __iomem *)(FLEX_CAN_2_LPCG), 16, FUNCTION_NAME(PD_DMA_CAN_2));
	clks[IMX8QXP_CAN0_DIV] = imx_clk_divider_scu("can0_div", SC_R_CAN_0, SC_PM_CLK_PER);
	clks[IMX8QXP_CAN1_DIV] = imx_clk_divider_scu("can1_div", SC_R_CAN_1, SC_PM_CLK_PER);
	clks[IMX8QXP_CAN2_DIV] = imx_clk_divider_scu("can2_div", SC_R_CAN_2, SC_PM_CLK_PER);
	clks[IMX8QXP_CAN0_CLK] = imx_clk_gate_scu("can0_clk", "can0_div", SC_R_CAN_0, SC_PM_CLK_PER, (void __iomem *)(FLEX_CAN_0_LPCG), 0, 0);
	clks[IMX8QXP_CAN1_CLK] = imx_clk_gate_scu("can1_clk", "can1_div", SC_R_CAN_1, SC_PM_CLK_PER, (void __iomem *)(FLEX_CAN_1_LPCG), 0, 0);
	clks[IMX8QXP_CAN2_CLK] = imx_clk_gate_scu("can2_clk", "can2_div", SC_R_CAN_2, SC_PM_CLK_PER, (void __iomem *)(FLEX_CAN_2_LPCG), 0, 0);
	clks[IMX8QXP_I2C0_IPG_CLK]   = imx_clk_gate2_scu("i2c0_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPI2C_0_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_0));
	clks[IMX8QXP_I2C1_IPG_CLK]   = imx_clk_gate2_scu("i2c1_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPI2C_1_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_1));
	clks[IMX8QXP_I2C2_IPG_CLK]   = imx_clk_gate2_scu("i2c2_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPI2C_2_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_2));
	clks[IMX8QXP_I2C3_IPG_CLK]   = imx_clk_gate2_scu("i2c3_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPI2C_3_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_3));
	clks[IMX8QXP_I2C0_DIV] = imx_clk_divider_scu("i2c0_div", SC_R_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QXP_I2C1_DIV] = imx_clk_divider_scu("i2c1_div", SC_R_I2C_1, SC_PM_CLK_PER);
	clks[IMX8QXP_I2C2_DIV] = imx_clk_divider_scu("i2c2_div", SC_R_I2C_2, SC_PM_CLK_PER);
	clks[IMX8QXP_I2C3_DIV] = imx_clk_divider_scu("i2c3_div", SC_R_I2C_3, SC_PM_CLK_PER);
	clks[IMX8QXP_I2C0_CLK] = imx_clk_gate_scu("i2c0_clk", "i2c0_div", SC_R_I2C_0, SC_PM_CLK_PER, (void __iomem *)(LPI2C_0_LPCG), 0, 0);
	clks[IMX8QXP_I2C1_CLK] = imx_clk_gate_scu("i2c1_clk", "i2c1_div", SC_R_I2C_1, SC_PM_CLK_PER, (void __iomem *)(LPI2C_1_LPCG), 0, 0);
	clks[IMX8QXP_I2C2_CLK] = imx_clk_gate_scu("i2c2_clk", "i2c2_div", SC_R_I2C_2, SC_PM_CLK_PER, (void __iomem *)(LPI2C_2_LPCG), 0, 0);
	clks[IMX8QXP_I2C3_CLK] = imx_clk_gate_scu("i2c3_clk", "i2c3_div", SC_R_I2C_3, SC_PM_CLK_PER, (void __iomem *)(LPI2C_3_LPCG), 0, 0);
	clks[IMX8QXP_FTM0_IPG_CLK] = imx_clk_gate2_scu("ftm0_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(FTM_0_LPCG), 16, FUNCTION_NAME(PD_DMA_FTM_0));
	clks[IMX8QXP_FTM1_IPG_CLK] = imx_clk_gate2_scu("ftm1_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(FTM_1_LPCG), 16, FUNCTION_NAME(PD_DMA_FTM_1));
	clks[IMX8QXP_FTM0_DIV] = imx_clk_divider_scu("ftm0_div", SC_R_FTM_0, SC_PM_CLK_PER);
	clks[IMX8QXP_FTM1_DIV] = imx_clk_divider_scu("ftm1_div", SC_R_FTM_1, SC_PM_CLK_PER);
	clks[IMX8QXP_FTM0_CLK] = imx_clk_gate_scu("ftm0_clk", "ftm0_div", SC_R_FTM_0, SC_PM_CLK_PER, (void __iomem *)(FTM_0_LPCG), 0, 0);
	clks[IMX8QXP_FTM1_CLK] = imx_clk_gate_scu("ftm1_clk", "ftm1_div", SC_R_FTM_1, SC_PM_CLK_PER, (void __iomem *)(FTM_1_LPCG), 0, 0);
	clks[IMX8QXP_ADC0_IPG_CLK] = imx_clk_gate2_scu("adc0_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(ADC_0_LPCG), 16, FUNCTION_NAME(PD_DMA_ADC_0));
	clks[IMX8QXP_ADC0_DIV] = imx_clk_divider_scu("adc0_div", SC_R_ADC_0, SC_PM_CLK_PER);
	clks[IMX8QXP_ADC0_CLK] = imx_clk_gate_scu("adc0_clk", "adc0_div", SC_R_ADC_0, SC_PM_CLK_PER, (void __iomem *)(ADC_0_LPCG), 0, 0);
	clks[IMX8QXP_PWM_IPG_CLK] = imx_clk_gate2_scu("pwm_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(PWM_LPCG), 16, FUNCTION_NAME(PD_DMA_PWM_0));
	clks[IMX8QXP_PWM_DIV] = imx_clk_divider_scu("pwm_div", SC_R_LCD_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QXP_PWM_CLK] = imx_clk_gate_scu("pwm_clk", "pwm_div", SC_R_LCD_0_PWM_0, SC_PM_CLK_PER, (void __iomem *)(PWM_LPCG), 0, 0);
	clks[IMX8QXP_LCD_IPG_CLK] = imx_clk_gate2_scu("lcd_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LCD_LPCG), 16, FUNCTION_NAME(PD_DMA_LCD_0));
	clks[IMX8QXP_LCD_DIV] = imx_clk_divider_scu("lcd_div", SC_R_LCD_0, SC_PM_CLK_PER);
	clks[IMX8QXP_LCD_CLK] = imx_clk_gate_scu("lcd_clk", "lcd_div", SC_R_LCD_0, SC_PM_CLK_PER, (void __iomem *)(LCD_LPCG), 0, 0);

	/* Connectivity */
	clks[IMX8QXP_SDHC0_IPG_CLK] = imx_clk_gate2_scu("sdhc0_ipg_clk", "ipg_conn_clk_root", (void __iomem *)(USDHC_0_LPCG), 16, FUNCTION_NAME(PD_CONN_SDHC_0));
	clks[IMX8QXP_SDHC1_IPG_CLK] = imx_clk_gate2_scu("sdhc1_ipg_clk", "ipg_conn_clk_root", (void __iomem *)(USDHC_1_LPCG), 16, FUNCTION_NAME(PD_CONN_SDHC_1));
	clks[IMX8QXP_SDHC2_IPG_CLK] = imx_clk_gate2_scu("sdhc2_ipg_clk", "ipg_conn_clk_root", (void __iomem *)(USDHC_2_LPCG), 16, FUNCTION_NAME(PD_CONN_SDHC_2));
	clks[IMX8QXP_SDHC0_DIV] = imx_clk_divider_scu("sdhc0_div", SC_R_SDHC_0, SC_PM_CLK_PER);
	clks[IMX8QXP_SDHC1_DIV] = imx_clk_divider_scu("sdhc1_div", SC_R_SDHC_1, SC_PM_CLK_PER);
	clks[IMX8QXP_SDHC2_DIV] = imx_clk_divider_scu("sdhc2_div", SC_R_SDHC_2, SC_PM_CLK_PER);
	clks[IMX8QXP_SDHC0_CLK] = imx_clk_gate_scu("sdhc0_clk", "sdhc0_div", SC_R_SDHC_0, SC_PM_CLK_PER, (void __iomem *)(USDHC_0_LPCG), 0, 0);
	clks[IMX8QXP_SDHC1_CLK] = imx_clk_gate_scu("sdhc1_clk", "sdhc1_div", SC_R_SDHC_1, SC_PM_CLK_PER, (void __iomem *)(USDHC_1_LPCG), 0, 0);
	clks[IMX8QXP_SDHC2_CLK] = imx_clk_gate_scu("sdhc2_clk", "sdhc2_div", SC_R_SDHC_2, SC_PM_CLK_PER, (void __iomem *)(USDHC_2_LPCG), 0, 0);
	clks[IMX8QXP_ENET0_ROOT_DIV] = imx_clk_divider_scu("enet0_root_div", SC_R_ENET_0, SC_PM_CLK_PER);
	clks[IMX8QXP_ENET0_REF_DIV] = imx_clk_divider3_scu("enet0_ref_div", "enet0_root_clk", SC_R_ENET_0, SC_C_CLKDIV);
	clks[IMX8QXP_ENET1_REF_DIV] = imx_clk_divider3_scu("enet1_ref_div", "enet1_root_clk", SC_R_ENET_1, SC_C_CLKDIV);
	clks[IMX8QXP_ENET0_BYPASS_DIV] = imx_clk_divider_scu("enet0_bypass_div", SC_R_ENET_0, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_ENET0_RGMII_DIV] = imx_clk_divider_scu("enet0_rgmii_div", SC_R_ENET_0, SC_PM_CLK_MISC0);
	clks[IMX8QXP_ENET1_ROOT_DIV] = imx_clk_divider_scu("enet1_root_div", SC_R_ENET_1, SC_PM_CLK_PER);
	clks[IMX8QXP_ENET1_BYPASS_DIV] = imx_clk_divider_scu("enet1_bypass_div", SC_R_ENET_1, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_ENET1_RGMII_DIV] = imx_clk_divider_scu("enet1_rgmii_div", SC_R_ENET_1, SC_PM_CLK_MISC0);
	clks[IMX8QXP_ENET0_AHB_CLK]   = imx_clk_gate2_scu("enet0_ahb_clk", "axi_conn_clk_root", (void __iomem *)(ENET_0_LPCG), 8, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QXP_ENET0_IPG_S_CLK] = imx_clk_gate2_scu("enet0_ipg_s_clk", "ipg_conn_clk_root", (void __iomem *)(ENET_0_LPCG), 20, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QXP_ENET0_IPG_CLK]   = imx_clk_gate2_scu("enet0_ipg_clk", "enet0_ipg_s_clk", (void __iomem *)(ENET_0_LPCG), 16, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QXP_ENET1_AHB_CLK]   = imx_clk_gate2_scu("enet1_ahb_clk", "axi_conn_clk_root", (void __iomem *)(ENET_1_LPCG), 8, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QXP_ENET1_IPG_S_CLK] = imx_clk_gate2_scu("enet1_ipg_s_clk", "ipg_conn_clk_root", (void __iomem *)(ENET_1_LPCG), 20, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QXP_ENET1_IPG_CLK]   = imx_clk_gate2_scu("enet1_ipg_clk", "enet1_ipg_s_clk", (void __iomem *)(ENET_1_LPCG), 16, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QXP_ENET0_ROOT_CLK] = imx_clk_gate_scu("enet0_root_clk", "enet0_root_div", SC_R_ENET_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_ENET1_ROOT_CLK] = imx_clk_gate_scu("enet1_root_clk", "enet1_root_div", SC_R_ENET_1, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_ENET0_TX_CLK] = imx_clk_gate2_scu("enet0_tx_2x_clk", "enet0_root_clk", (void __iomem *)(ENET_0_LPCG), 4, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QXP_ENET1_TX_CLK] = imx_clk_gate2_scu("enet1_tx_2x_clk", "enet1_root_clk", (void __iomem *)(ENET_1_LPCG), 4, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QXP_ENET0_PTP_CLK] = imx_clk_gate2_scu("enet0_ptp_clk", "enet0_root_clk", (void __iomem *)(ENET_0_LPCG), 0, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QXP_ENET1_PTP_CLK] = imx_clk_gate2_scu("enet1_ptp_clk", "enet1_root_clk", (void __iomem *)(ENET_1_LPCG), 0, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QXP_ENET0_REF_25MHZ_125MHZ_SEL] = imx_clk_mux_gpr_scu("enet0_ref_25_125_sel", enet_sels, ARRAY_SIZE(enet_sels), SC_R_ENET_0, SC_C_SEL_125);
	clks[IMX8QXP_ENET1_REF_25MHZ_125MHZ_SEL] = imx_clk_mux_gpr_scu("enet1_ref_25_125_sel", enet_sels, ARRAY_SIZE(enet_sels), SC_R_ENET_1, SC_C_SEL_125);
	clks[IMX8QXP_ENET0_RMII_TX_SEL] = imx_clk_mux_gpr_scu("enet0_rmii_tx_sel", enet0_rmii_tx_sels, ARRAY_SIZE(enet0_rmii_tx_sels), SC_R_ENET_0, SC_C_TXCLK);
	clks[IMX8QXP_ENET1_RMII_TX_SEL] = imx_clk_mux_gpr_scu("enet1_rmii_tx_sel", enet1_rmii_tx_sels, ARRAY_SIZE(enet1_rmii_tx_sels), SC_R_ENET_1, SC_C_TXCLK);
	clks[IMX8QXP_ENET0_RGMII_TX_CLK] = imx_clk_gate2_scu("enet0_rgmii_tx_clk", "enet0_rmii_tx_sel", (void __iomem *)(ENET_0_LPCG), 12, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QXP_ENET1_RGMII_TX_CLK] = imx_clk_gate2_scu("enet1_rgmii_tx_clk", "enet1_rmii_tx_sel", (void __iomem *)(ENET_1_LPCG), 12, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QXP_ENET0_RMII_RX_CLK] = imx_clk_gate2_scu("enet0_rgmii_rx_clk", "enet0_rgmii_div", (void __iomem *)(ENET_0_LPCG + 0x4), 0, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QXP_ENET1_RMII_RX_CLK] = imx_clk_gate2_scu("enet1_rgmii_rx_clk", "enet1_rgmii_div", (void __iomem *)(ENET_1_LPCG + 0x4), 0, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QXP_ENET0_REF_25MHZ_125MHZ_CLK] = imx_clk_gate3_scu("enet0_ref_25_125_clk", "enet0_ref_25_125_sel", SC_R_ENET_0, SC_C_DISABLE_125, true);
	clks[IMX8QXP_ENET1_REF_25MHZ_125MHZ_CLK] = imx_clk_gate3_scu("enet1_ref_25_125_clk", "enet1_ref_25_125_sel", SC_R_ENET_1, SC_C_DISABLE_125, true);
	clks[IMX8QXP_ENET0_REF_50MHZ_CLK] = imx_clk_gate3_scu("enet0_ref_50_clk", NULL, SC_R_ENET_0, SC_C_DISABLE_50, true);
	clks[IMX8QXP_ENET1_REF_50MHZ_CLK] = imx_clk_gate3_scu("enet1_ref_50_clk", NULL, SC_R_ENET_1, SC_C_DISABLE_50, true);
	clks[IMX8QXP_GPMI_BCH_IO_DIV] = imx_clk_divider_scu("gpmi_io_div", SC_R_NAND, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_GPMI_BCH_DIV] = imx_clk_divider_scu("gpmi_bch_div", SC_R_NAND, SC_PM_CLK_PER);
	clks[IMX8QXP_GPMI_APB_CLK]   = imx_clk_gate2_scu("gpmi_apb_clk", "axi_conn_clk_root", (void __iomem *)(NAND_LPCG), 16, FUNCTION_NAME(PD_CONN_NAND));
	clks[IMX8QXP_GPMI_APB_BCH_CLK]   = imx_clk_gate2_scu("gpmi_apb_bch_clk", "axi_conn_clk_root", (void __iomem *)(NAND_LPCG), 20, FUNCTION_NAME(PD_CONN_NAND));
	clks[IMX8QXP_GPMI_BCH_IO_CLK] = imx_clk_gate_scu("gpmi_io_clk", "gpmi_io_div", SC_R_NAND, SC_PM_CLK_MST_BUS, (void __iomem *)(NAND_LPCG), 4, 0);
	clks[IMX8QXP_GPMI_BCH_CLK] = imx_clk_gate_scu("gpmi_bch_clk", "gpmi_bch_div", SC_R_NAND, SC_PM_CLK_PER, (void __iomem *)(NAND_LPCG), 0, 0);
	clks[IMX8QXP_APBHDMA_CLK] = imx_clk_gate2_scu("gpmi_clk", "axi_conn_clk_root", (void __iomem *)(NAND_LPCG + 0x4), 16, FUNCTION_NAME(PD_CONN_NAND));
	clks[IMX8QXP_USB3_ACLK_DIV] = imx_clk_divider_scu("usb3_aclk_div", SC_R_USB_2, SC_PM_CLK_PER);
	clks[IMX8QXP_USB3_BUS_DIV] = imx_clk_divider_scu("usb3_bus_div", SC_R_USB_2, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_USB3_LPM_DIV] = imx_clk_divider_scu("usb3_lpm_div", SC_R_USB_2, SC_PM_CLK_MISC);
	clks[IMX8QXP_USB2_OH_AHB_CLK]   = imx_clk_gate2_scu("usboh3", "ahb_conn_clk_root", (void __iomem *)(USB_2_LPCG), 24, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QXP_USB2_OH_IPG_S_CLK]   = imx_clk_gate2_scu("usboh3_ipg_s", "ipg_conn_clk_root", (void __iomem *)(USB_2_LPCG), 16, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QXP_USB2_OH_IPG_S_PL301_CLK]   = imx_clk_gate2_scu("usboh3_ipg_pl301_s", "ipg_conn_clk_root", (void __iomem *)(USB_2_LPCG), 20, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QXP_USB2_PHY_IPG_CLK]   = imx_clk_gate2_scu("usboh3_phy_clk", "ipg_conn_clk_root", (void __iomem *)(USB_2_LPCG), 28, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QXP_USB3_IPG_CLK]   = imx_clk_gate2_scu("usb3_ipg_clk", "ipg_conn_clk_root", (void __iomem *)(USB_3_LPCG), 16, FUNCTION_NAME(PD_CONN_USB_2));
	clks[IMX8QXP_USB3_CORE_PCLK]   = imx_clk_gate2_scu("usb3_core_clk", "ipg_conn_clk_root", (void __iomem *)(USB_3_LPCG), 20, FUNCTION_NAME(PD_CONN_USB_2));
	clks[IMX8QXP_USB3_PHY_CLK]   = imx_clk_gate2_scu("usb3_phy_clk", "usb3_ipg_clk", (void __iomem *)(USB_3_LPCG), 24, FUNCTION_NAME(PD_CONN_USB_2_PHY));
	clks[IMX8QXP_USB3_ACLK] = imx_clk_gate_scu("usb3_aclk", "usb3_aclk_div", SC_R_USB_2, SC_PM_CLK_PER, (void __iomem *)(USB_3_LPCG), 28, 0);
	clks[IMX8QXP_USB3_BUS_CLK] = imx_clk_gate_scu("usb3_bus_clk", "usb3_bus_div", SC_R_USB_2, SC_PM_CLK_MST_BUS, (void __iomem *)(USB_3_LPCG), 0, 0);
	clks[IMX8QXP_USB3_LPM_CLK] = imx_clk_gate_scu("usb3_lpm_clk", "usb3_lpm_div", SC_R_USB_2, SC_PM_CLK_MISC, (void __iomem *)(USB_3_LPCG), 4, 0);
	clks[IMX8QXP_EDMA_CLK]   = imx_clk_gate2_scu("edma_clk", "axi_conn_clk_root", (void __iomem *)(EDMA_LPCG), 0, FUNCTION_NAME(PD_CONN_DMA_4_CH0));
	clks[IMX8QXP_EDMA_IPG_CLK]   = imx_clk_gate2_scu("edma_ipg_clk", "ipg_conn_clk_root", (void __iomem *)(EDMA_LPCG), 16, FUNCTION_NAME(PD_CONN_DMA_4_CH0));
	clks[IMX8QXP_MLB_HCLK]   = imx_clk_gate2_scu("mlb_hclk", "axi_conn_clk_root", (void __iomem *)(MLB_LPCG), 20, FUNCTION_NAME(PD_CONN_MLB_0));
	clks[IMX8QXP_MLB_CLK]   = imx_clk_gate2_scu("mlb_clk", "mlb_hclk", (void __iomem *)(MLB_LPCG), 0, FUNCTION_NAME(PD_CONN_MLB_0));
	clks[IMX8QXP_MLB_IPG_CLK]   = imx_clk_gate2_scu("mlb_ipg_clk", "ipg_conn_clk_root", (void __iomem *)(MLB_LPCG), 16, FUNCTION_NAME(PD_CONN_MLB_0));

	/* Display controller - DC0 SS */
	clks[IMX8QXP_DC0_PLL0_DIV] = imx_clk_divider_scu("dc0_pll0_div", SC_R_DC_0_PLL_0, SC_PM_CLK_PLL);
	clks[IMX8QXP_DC0_PLL1_DIV] = imx_clk_divider_scu("dc0_pll1_div", SC_R_DC_0_PLL_1, SC_PM_CLK_PLL);
	clks[IMX8QXP_DC0_PLL0_CLK] = imx_clk_gate_scu("dc0_pll0_clk", "dc0_pll0_div", SC_R_DC_0_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QXP_DC0_PLL1_CLK] = imx_clk_gate_scu("dc0_pll1_clk", "dc0_pll1_div", SC_R_DC_0_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QXP_DC0_DISP0_DIV] = imx_clk_divider_scu("dc0_disp0_div", SC_R_DC_0, SC_PM_CLK_MISC0);
	clks[IMX8QXP_DC0_DISP1_DIV] = imx_clk_divider_scu("dc0_disp1_div", SC_R_DC_0, SC_PM_CLK_MISC1);
	clks[IMX8QXP_DC0_DISP0_CLK] = imx_clk_gate_scu("dc0_disp0_clk", "dc0_disp0_div", SC_R_DC_0, SC_PM_CLK_MISC0, (void __iomem *)(DC_0_LPCG), 0, 0);
	clks[IMX8QXP_DC0_DISP1_CLK] = imx_clk_gate_scu("dc0_disp1_clk", "dc0_disp1_div", SC_R_DC_0, SC_PM_CLK_MISC1, (void __iomem *)(DC_0_LPCG), 4, 0);
	clks[IMX8QXP_DC0_BYPASS_0_DIV] = imx_clk_divider_scu("dc0_bypass0_div", SC_R_DC_0_VIDEO0, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_DC0_BYPASS_1_DIV] = imx_clk_divider_scu("dc0_bypass1_div", SC_R_DC_0_VIDEO1, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_DC0_PRG0_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg0_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x20), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG0_APB_CLK] = imx_clk_gate2_scu("dc0_prg0_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x20), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG1_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg1_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x24), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG1_APB_CLK] = imx_clk_gate2_scu("dc0_prg1_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x24), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG2_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg2_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x28), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG2_APB_CLK] = imx_clk_gate2_scu("dc0_prg2_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x28), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG3_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg3_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x34), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG3_APB_CLK] = imx_clk_gate2_scu("dc0_prg3_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x34), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG4_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg4_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x38), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG4_APB_CLK] = imx_clk_gate2_scu("dc0_prg4_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x38), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG5_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg5_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x3c), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG5_APB_CLK] = imx_clk_gate2_scu("dc0_prg5_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x3c), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG6_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg6_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x40), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG6_APB_CLK] = imx_clk_gate2_scu("dc0_prg6_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x40), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG7_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg7_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x44), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG7_APB_CLK] = imx_clk_gate2_scu("dc0_prg7_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x44), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG8_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg8_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x48), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG8_APB_CLK] = imx_clk_gate2_scu("dc0_prg8_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x48), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR0_APB_CLK] = imx_clk_gate2_scu("dc0_dpr0_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x18), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR0_B_CLK] = imx_clk_gate2_scu("dc0_dpr0_b_clk", "axi_ext_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x18), 20, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_RTRAM0_CLK] = imx_clk_gate2_scu("dc0_rtrm0_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x1C), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR1_APB_CLK] = imx_clk_gate2_scu("dc0_dpr1_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x2C), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR1_B_CLK] = imx_clk_gate2_scu("dc0_dpr1_b_clk", "axi_ext_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x2C), 20, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_RTRAM1_CLK] = imx_clk_gate2_scu("dc0_rtrm1_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x30), 0, FUNCTION_NAME(PD_DC_0));

	/* Display interface - MIPI-LVDS SS */
	clks[IMX8QXP_MIPI0_BYPASS_CLK] = imx_clk_divider_scu("mipi0_bypass_clk", SC_R_MIPI_0, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI0_PIXEL_DIV] = imx_clk_divider_scu("mipi0_pixel_div", SC_R_MIPI_0, SC_PM_CLK_PER);
	clks[IMX8QXP_MIPI0_PIXEL_CLK] = imx_clk_gate_scu("mipi0_pixel_clk", "mipi0_pixel_div", SC_R_MIPI_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_LVDS_PIXEL_DIV] = imx_clk_divider_scu("mipi0_lvds_pixel_div", SC_R_LVDS_0, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI0_LVDS_PIXEL_CLK] = imx_clk_gate_scu("mipi0_lvds_pixel_clk", "mipi0_lvds_pixel_div", SC_R_LVDS_0, SC_PM_CLK_MISC2, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_LVDS_BYPASS_CLK] = imx_clk_divider_scu("mipi0_lvds_bypass_clk", SC_R_LVDS_0, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI0_LVDS_PHY_DIV] = imx_clk_divider_scu("mipi0_lvds_phy_div", SC_R_LVDS_0, SC_PM_CLK_MISC3);
	clks[IMX8QXP_MIPI0_LVDS_PHY_CLK] = imx_clk_gate_scu("mipi0_lvds_phy_clk", "mipi0_lvds_phy_div", SC_R_LVDS_0, SC_PM_CLK_MISC3, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_DSI_TX_ESC_SEL] = imx_clk_mux2_scu("mipi0_dsi_tx_esc_sel", mipi0_sels, ARRAY_SIZE(mipi0_sels), SC_R_MIPI_0, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI0_DSI_TX_ESC_DIV] = imx_clk_divider2_scu("mipi0_dsi_tx_esc_div", "mipi0_dsi_tx_esc_sel", SC_R_MIPI_0, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI0_DSI_TX_ESC_CLK] = imx_clk_gate_scu("mipi0_dsi_tx_esc_clk", "mipi0_dsi_tx_esc_div", SC_R_MIPI_0, SC_PM_CLK_MST_BUS, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_DSI_RX_ESC_SEL] = imx_clk_mux2_scu("mipi0_dsi_rx_esc_sel", mipi0_sels, ARRAY_SIZE(mipi0_sels), SC_R_MIPI_0, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI0_DSI_RX_ESC_DIV] = imx_clk_divider2_scu("mipi0_dsi_rx_esc_div", "mipi0_dsi_rx_esc_sel", SC_R_MIPI_0, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI0_DSI_RX_ESC_CLK] = imx_clk_gate_scu("mipi0_dsi_rx_esc_clk", "mipi0_dsi_rx_esc_div", SC_R_MIPI_0, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_DSI_PHY_SEL] = imx_clk_mux2_scu("mipi0_dsi_phy_sel", mipi0_sels, ARRAY_SIZE(mipi0_sels), SC_R_MIPI_0, SC_PM_CLK_PHY);
	clks[IMX8QXP_MIPI0_DSI_PHY_DIV] = imx_clk_divider2_scu("mipi0_dsi_phy_div", "mipi0_dsi_phy_sel", SC_R_MIPI_0, SC_PM_CLK_PHY);
	clks[IMX8QXP_MIPI0_DSI_PHY_CLK] = imx_clk_gate_scu("mipi0_dsi_phy_clk", "mipi0_dsi_phy_div", SC_R_MIPI_0, SC_PM_CLK_PHY, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_I2C0_DIV] = imx_clk_divider_scu("mipi0_i2c0_div", SC_R_MIPI_0_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI0_I2C1_DIV] = imx_clk_divider_scu("mipi0_i2c1_div", SC_R_MIPI_0_I2C_1, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI0_I2C0_CLK] = imx_clk_gate_scu("mipi0_i2c0_clk", "mipi0_i2c0_div", SC_R_MIPI_0_I2C_0, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI0_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_MIPI0_I2C1_CLK] = imx_clk_gate_scu("mipi0_i2c1_clk", "mipi0_i2c1_div", SC_R_MIPI_0_I2C_1, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI0_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_MIPI0_I2C0_IPG_S_CLK] = imx_clk_gate2_scu("mipi0_i2c0_ipg_s", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x10), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C0));
	clks[IMX8QXP_MIPI0_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi0_i2c0_ipg_clk", "mipi0_i2c0_ipg_s", (void __iomem *)(DI_MIPI0_LPCG), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C0));
	clks[IMX8QXP_MIPI0_I2C1_IPG_S_CLK] = imx_clk_gate2_scu("mipi0_i2c1_ipg_s", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x14), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C1));
	clks[IMX8QXP_MIPI0_I2C1_IPG_CLK] = imx_clk_gate2_scu("mipi0_i2c1_ipg_clk", "mipi0_i2c1_ipg_s", (void __iomem *)(DI_MIPI0_LPCG), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C1));
	clks[IMX8QXP_MIPI0_PWM_IPG_S_CLK] = imx_clk_gate2_scu("mipi0_pwm_ipg_s", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0xC), 0, FUNCTION_NAME(PD_MIPI_0_DSI_PWM0));
	clks[IMX8QXP_MIPI0_PWM_IPG_CLK] = imx_clk_gate2_scu("mipi0_pwm_ipg_clk", "mipi0_pwm_ipg_s", (void __iomem *)(DI_MIPI0_LPCG + 0xC), 0, FUNCTION_NAME(PD_MIPI_0_DSI_PWM0));
	clks[IMX8QXP_MIPI0_PWM_32K_CLK] = imx_clk_gate2_scu("mipi0_pwm_32K_clk", "xtal_32KHz", (void __iomem *)(DI_MIPI0_LPCG + 0xC), 0, FUNCTION_NAME(PD_MIPI_0_DSI_PWM0));
	clks[IMX8QXP_MIPI0_GPIO_IPG_CLK] = imx_clk_gate2_scu("mipi0_gpio_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x8), 0, FUNCTION_NAME(PD_MIPI_0_GPIO_0));
	clks[IMX8QXP_MIPI0_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi0_lis_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x0), 16, FUNCTION_NAME(PD_MIPI_0_DSI));
	clks[IMX8QXP_MIPI1_BYPASS_CLK] = imx_clk_divider_scu("mipi1_bypass_clk", SC_R_MIPI_1, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI1_PIXEL_DIV] = imx_clk_divider_scu("mipi1_pixel_div", SC_R_MIPI_1, SC_PM_CLK_PER);
	clks[IMX8QXP_MIPI1_PIXEL_CLK] = imx_clk_gate_scu("mipi1_pixel_clk", "mipi1_pixel_div", SC_R_MIPI_1, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_LVDS_PIXEL_DIV] = imx_clk_divider_scu("mipi1_lvds_pixel_div", SC_R_LVDS_1, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI1_LVDS_PIXEL_CLK] = imx_clk_gate_scu("mipi1_lvds_pixel_clk", "mipi1_lvds_pixel_div", SC_R_LVDS_1, SC_PM_CLK_MISC2, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_LVDS_BYPASS_CLK] = imx_clk_divider_scu("mipi1_lvds_bypass_clk", SC_R_LVDS_1, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI1_LVDS_PHY_DIV] = imx_clk_divider_scu("mipi1_lvds_phy_div", SC_R_LVDS_1, SC_PM_CLK_MISC3);
	clks[IMX8QXP_MIPI1_LVDS_PHY_CLK] = imx_clk_gate_scu("mipi1_lvds_phy_clk", "mipi1_lvds_phy_div", SC_R_LVDS_1, SC_PM_CLK_MISC3, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_DSI_TX_ESC_SEL] = imx_clk_mux2_scu("mipi1_dsi_tx_esc_sel", mipi1_sels, ARRAY_SIZE(mipi1_sels), SC_R_MIPI_1, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI1_DSI_TX_ESC_DIV] = imx_clk_divider2_scu("mipi1_dsi_tx_esc_div", "mipi1_dsi_tx_esc_sel", SC_R_MIPI_1, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI1_DSI_TX_ESC_CLK] = imx_clk_gate_scu("mipi1_dsi_tx_esc_clk", "mipi1_dsi_tx_esc_div", SC_R_MIPI_1, SC_PM_CLK_MST_BUS, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_DSI_RX_ESC_SEL] = imx_clk_mux2_scu("mipi1_dsi_rx_esc_sel", mipi1_sels, ARRAY_SIZE(mipi1_sels), SC_R_MIPI_1, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI1_DSI_RX_ESC_DIV] = imx_clk_divider2_scu("mipi1_dsi_rx_esc_div", "mipi1_dsi_rx_esc_sel", SC_R_MIPI_1, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI1_DSI_RX_ESC_CLK] = imx_clk_gate_scu("mipi1_dsi_rx_esc_clk", "mipi1_dsi_rx_esc_div", SC_R_MIPI_1, SC_PM_CLK_SLV_BUS, NULL, 0, 0);

	clks[IMX8QXP_MIPI1_I2C0_DIV] = imx_clk_divider_scu("mipi1_i2c0_div", SC_R_MIPI_1_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI1_I2C1_DIV] = imx_clk_divider_scu("mipi1_i2c1_div", SC_R_MIPI_1_I2C_1, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI1_I2C0_CLK] = imx_clk_gate_scu("mipi1_i2c0_clk", "mipi1_i2c0_div", SC_R_MIPI_1_I2C_0, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI1_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_MIPI1_I2C1_CLK] = imx_clk_gate_scu("mipi1_i2c1_clk", "mipi1_i2c1_div", SC_R_MIPI_1_I2C_1, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI1_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_MIPI1_I2C0_IPG_S_CLK] = imx_clk_gate2_scu("mipi1_i2c0_ipg_s", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x10), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C0));
	clks[IMX8QXP_MIPI1_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi1_i2c0_ipg_clk", "mipi1_i2c0_ipg_s", (void __iomem *)(DI_MIPI1_LPCG), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C0));
	clks[IMX8QXP_MIPI1_I2C1_IPG_S_CLK] = imx_clk_gate2_scu("mipi1_i2c1_ipg_s", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x14), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C1));
	clks[IMX8QXP_MIPI1_I2C1_IPG_CLK] = imx_clk_gate2_scu("mipi1_i2c1_ipg_clk", "mipi1_i2c1_ipg_s", (void __iomem *)(DI_MIPI1_LPCG), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C1));
	clks[IMX8QXP_MIPI1_PWM_IPG_S_CLK] = imx_clk_gate2_scu("mipi1_pwm_ipg_s", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0xC), 0, FUNCTION_NAME(PD_MIPI_1_DSI_PWM0));
	clks[IMX8QXP_MIPI1_PWM_IPG_CLK] = imx_clk_gate2_scu("mipi1_pwm_ipg_clk", "mipi1_pwm_ipg_s", (void __iomem *)(DI_MIPI1_LPCG + 0xC), 0, FUNCTION_NAME(PD_MIPI_1_DSI_PWM0));
	clks[IMX8QXP_MIPI1_PWM_32K_CLK] = imx_clk_gate2_scu("mipi1_pwm_32K_clk", "xtal_32KHz", (void __iomem *)(DI_MIPI1_LPCG + 0xC), 0, FUNCTION_NAME(PD_MIPI_1_DSI_PWM0));
	clks[IMX8QXP_MIPI1_GPIO_IPG_CLK] = imx_clk_gate2_scu("mipi1_gpio_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x8), 0, FUNCTION_NAME(PD_MIPI_1_GPIO_0));
	clks[IMX8QXP_MIPI1_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi1_lis_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x0), 16, FUNCTION_NAME(PD_MIPI_1_DSI));

	/* Imaging SS */
	clks[IMX8QXP_IMG_JPEG_ENC_IPG_CLK] = imx_clk_gate2_scu("img_jpeg_enc_ipg_clk", "ipg_img_clk_root", (void __iomem *)(IMG_JPEG_ENC_LPCG), 16, FUNCTION_NAME(PD_IMAGING_JPEG_ENC));
	clks[IMX8QXP_IMG_JPEG_ENC_CLK] = imx_clk_gate2_scu("img_jpeg_enc_clk", "img_jpeg_enc_ipg_clk", (void __iomem *)(IMG_JPEG_ENC_LPCG), 0, FUNCTION_NAME(PD_IMAGING_JPEG_ENC));
	clks[IMX8QXP_IMG_JPEG_DEC_IPG_CLK] = imx_clk_gate2_scu("img_jpeg_dec_ipg_clk", "ipg_img_clk_root", (void __iomem *)(IMG_JPEG_DEC_LPCG), 16, FUNCTION_NAME(PD_IMAGING_JPEG_DEC));
	clks[IMX8QXP_IMG_JPEG_DEC_CLK] = imx_clk_gate2_scu("img_jpeg_dec_clk", "img_jpeg_dec_ipg_clk", (void __iomem *)(IMG_JPEG_DEC_LPCG), 0, FUNCTION_NAME(PD_IMAGING_JPEG_DEC));
	clks[IMX8QXP_IMG_PXL_LINK_DC0_CLK] = imx_clk_gate2_scu("img_pxl_link_dc0_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PXL_LINK_DC0_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QXP_IMG_PXL_LINK_DC1_CLK] = imx_clk_gate2_scu("img_pxl_link_dc1_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PXL_LINK_DC1_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QXP_IMG_PXL_LINK_CSI0_CLK] = imx_clk_gate2_scu("img_pxl_link_csi0_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PXL_LINK_CSI0_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QXP_IMG_PXL_LINK_CSI1_CLK] = imx_clk_gate2_scu("img_pxl_link_csi1_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PXL_LINK_CSI1_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QXP_IMG_PXL_LINK_HDMI_IN_CLK] = imx_clk_gate2_scu("img_pxl_link_hdmi_in_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PXL_LINK_HDMI_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QXP_IMG_PDMA_0_CLK] = imx_clk_gate2_scu("img_pdma0_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_0_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA0));
	clks[IMX8QXP_IMG_PDMA_1_CLK] = imx_clk_gate2_scu("img_pdma1_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_1_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA1));
	clks[IMX8QXP_IMG_PDMA_2_CLK] = imx_clk_gate2_scu("img_pdma2_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_2_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA2));
	clks[IMX8QXP_IMG_PDMA_3_CLK] = imx_clk_gate2_scu("img_pdma3_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_3_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA3));
	clks[IMX8QXP_IMG_PDMA_4_CLK] = imx_clk_gate2_scu("img_pdma4_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_4_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA4));
	clks[IMX8QXP_IMG_PDMA_5_CLK] = imx_clk_gate2_scu("img_pdma5_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_5_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA5));
	clks[IMX8QXP_IMG_PDMA_6_CLK] = imx_clk_gate2_scu("img_pdma6_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_6_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA6));
	clks[IMX8QXP_IMG_PDMA_7_CLK] = imx_clk_gate2_scu("img_pdma7_clk", "pxl_img_clk_root", (void __iomem *)(IMG_PDMA_7_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA7));

	/* MIPI CSI SS */
	clks[IMX8QXP_CSI0_I2C0_DIV] = imx_clk_divider_scu("mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QXP_CSI0_PWM0_DIV] = imx_clk_divider_scu("mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QXP_CSI0_CORE_DIV] = imx_clk_divider_scu("mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER);
	clks[IMX8QXP_CSI0_ESC_DIV] = imx_clk_divider_scu("mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC);
	clks[IMX8QXP_CSI0_IPG_CLK_S] = imx_clk_gate2_scu("mipi_csi0_ipg_s", "ipg_mipi_csi_clk_root", (void __iomem *)(MIPI_CSI_0_LPCG + 0x8), 16, FUNCTION_NAME(PD_MIPI_CSI0));
	clks[IMX8QXP_CSI0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_ipg", "mipi_csi0_ipg_s", (void __iomem *)(MIPI_CSI_0_LPCG), 16, FUNCTION_NAME(PD_MIPI_CSI0));
	clks[IMX8QXP_CSI0_APB_CLK] = imx_clk_gate2_scu("mipi_csi0_apb_clk", "ipg_mipi_csi_clk_root", (void __iomem *)(MIPI_CSI_0_LPCG + 0x4), 16,  FUNCTION_NAME(PD_MIPI_CSI0));
	clks[IMX8QXP_CSI0_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_i2c0_ipg_s", "ipg_mipi_csi_clk_root", (void __iomem *)(MIPI_CSI_0_LPCG + 0x14), 16, FUNCTION_NAME(PD_MIPI_CSI0_I2C));
	clks[IMX8QXP_CSI0_I2C0_CLK] = imx_clk_gate_scu("mipi_csi0_i2c0_clk", "mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER, (void __iomem *)(MIPI_CSI_0_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_CSI0_PWM0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_pwm0_ipg_s", "ipg_mipi_csi_clk_root", (void __iomem *)(MIPI_CSI_0_LPCG + 0x10), 16, FUNCTION_NAME(PD_MIPI_CSI0_PWM));
	clks[IMX8QXP_CSI0_PWM0_CLK] = imx_clk_gate_scu("mipi_csi0_pwm0_clk", "mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER, (void __iomem *)(MIPI_CSI_0_LPCG + 0x10), 0, 0);
	clks[IMX8QXP_CSI0_CORE_CLK] = imx_clk_gate_scu("mipi_csi0_core_clk", "mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER, (void __iomem *)(MIPI_CSI_0_LPCG + 0x18), 16, 0);
	clks[IMX8QXP_CSI0_ESC_CLK] = imx_clk_gate_scu("mipi_csi0_esc_clk", "mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC, (void __iomem *)(MIPI_CSI_0_LPCG + 0x1C), 16, 0);

	/* HSIO SS */
	clks[IMX8QXP_HSIO_PCIE_MSTR_AXI_CLK] = imx_clk_gate2_scu("hsio_pcie_mstr_axi_clk", "axi_hsio_clk_root", (void __iomem *)(HSIO_PCIE_X1_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QXP_HSIO_PCIE_SLV_AXI_CLK] = imx_clk_gate2_scu("hsio_pcie_slv_axi_clk", "axi_hsio_clk_root", (void __iomem *)(HSIO_PCIE_X1_LPCG), 20, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QXP_HSIO_PCIE_DBI_AXI_CLK] = imx_clk_gate2_scu("hsio_pcie_dbi_axi_clk", "axi_hsio_clk_root", (void __iomem *)(HSIO_PCIE_X1_LPCG), 24, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QXP_HSIO_PCIE_X1_PER_CLK] = imx_clk_gate2_scu("hsio_pcie_x1_per_clk", "per_hsio_clk_root", (void __iomem *)(HSIO_PCIE_X1_CRR3_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QXP_HSIO_PHY_X1_PER_CLK] = imx_clk_gate2_scu("hsio_phy_x1_per_clk", "per_hsio_clk_root", (void __iomem *)(HSIO_PHY_X1_CRR1_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QXP_HSIO_MISC_PER_CLK] = imx_clk_gate2_scu("hsio_misc_per_clk", "per_hsio_clk_root", (void __iomem *)(HSIO_MISC_LPCG), 16, FUNCTION_NAME(PD_HSIO));
	clks[IMX8QXP_HSIO_PHY_X1_APB_CLK] = imx_clk_gate2_scu("hsio_phy_x1_apb_clk", "per_hsio_clk_root", (void __iomem *)(HSIO_PHY_X1_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QXP_HSIO_GPIO_CLK] = imx_clk_gate2_scu("hsio_gpio_clk", "per_hsio_clk_root", (void __iomem *)(HSIO_GPIO_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QXP_HSIO_PHY_X1_PCLK] = imx_clk_gate2_scu("hsio_phy_x1_pclk", "dummy", (void __iomem *)(HSIO_PHY_X1_LPCG), 0, FUNCTION_NAME(PD_HSIO_PCIE_B));

	/* CM40 */
	clks[IMX8QXP_CM40_I2C_DIV]     = imx_clk_divider_scu("cm40_i2c_div", SC_R_M4_0_I2C, SC_PM_CLK_PER);
	clks[IMX8QXP_CM40_I2C_CLK]     = imx_clk_gate_scu("cm40_i2c_clk", "cm40_i2c_div", SC_R_M4_0_I2C, SC_PM_CLK_PER, (void __iomem *)(CM40_I2C_LPCG), 0, 0);
	clks[IMX8QXP_CM40_I2C_IPG_CLK] = imx_clk_gate2_scu("cm40_i2c_ipg_clk", "ipg_cm40_clk_root", (void __iomem *)(CM40_I2C_LPCG), 16, FUNCTION_NAME(PD_CM40_I2C));

	/* Audio */
	clks[IMX8QXP_AUD_ACM_AUD_PLL_CLK0_DIV] = imx_clk_divider2_scu("aud_acm_aud_pll_clk0_div", "audio_pll0_clk", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC0);
	clks[IMX8QXP_AUD_ACM_AUD_PLL_CLK1_DIV] = imx_clk_divider2_scu("aud_acm_aud_pll_clk1_div", "audio_pll1_clk", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC0);
	clks[IMX8QXP_AUD_ACM_AUD_REC_CLK0_DIV] = imx_clk_divider2_scu("aud_acm_aud_rec_clk0_div", "audio_pll0_clk", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC1);
	clks[IMX8QXP_AUD_ACM_AUD_REC_CLK1_DIV] = imx_clk_divider2_scu("aud_acm_aud_rec_clk1_div", "audio_pll1_clk", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC1);
	clks[IMX8QXP_AUD_ACM_AUD_PLL_CLK0_CLK] = imx_clk_gate_scu("aud_acm_aud_pll_clk0_clk", "aud_acm_aud_pll_clk0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC0, (void __iomem *)(AUD_PLL_CLK0_LPCG), 0, 0);
	clks[IMX8QXP_AUD_ACM_AUD_PLL_CLK1_CLK] = imx_clk_gate_scu("aud_acm_aud_pll_clk1_clk", "aud_acm_aud_pll_clk1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC0, (void __iomem *)(AUD_PLL_CLK1_LPCG), 0, 0);
	clks[IMX8QXP_AUD_ACM_AUD_REC_CLK0_CLK] = imx_clk_gate_scu("aud_acm_aud_rec_clk0_clk", "aud_acm_aud_rec_clk0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC1, (void __iomem *)(AUD_REC_CLK0_LPCG), 0, 0);
	clks[IMX8QXP_AUD_ACM_AUD_REC_CLK1_CLK] = imx_clk_gate_scu("aud_acm_aud_rec_clk1_clk", "aud_acm_aud_rec_clk1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC1, (void __iomem *)(AUD_REC_CLK1_LPCG), 0, 0);

	/* VPU */
	clks[IMX8QXP_VPU_ENC_CLK] = imx_clk_gate_scu("vpu_enc_clk", "dummy", SC_R_VPU_ENC_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_VPU_DEC_CLK] = imx_clk_gate_scu("vpu_dec_clk", "dummy", SC_R_VPU_DEC_0, SC_PM_CLK_PER, NULL, 0, 0);

	np_acm = of_find_compatible_node(NULL, NULL, "nxp,imx8qm-acm");
	base_acm = of_iomap(np_acm, 0);
	WARN_ON(!base_acm);

	clks[IMX8QXP_EXT_AUD_MCLK0]	= imx_clk_fixed("ext_aud_mclk0", 0);
	clks[IMX8QXP_EXT_AUD_MCLK1]	= imx_clk_fixed("ext_aud_mclk1", 0);
	clks[IMX8QXP_ESAI0_RX_CLK]	= imx_clk_fixed("esai0_rx_clk", 0);
	clks[IMX8QXP_ESAI0_RX_HF_CLK]	= imx_clk_fixed("esai0_rx_hf_clk", 0);
	clks[IMX8QXP_ESAI0_TX_CLK]	= imx_clk_fixed("esai0_tx_clk", 0);
	clks[IMX8QXP_ESAI0_TX_HF_CLK]	= imx_clk_fixed("esai0_tx_hf_clk", 0);
	clks[IMX8QXP_SPDIF0_RX]		= imx_clk_fixed("spdif0_rx", 0);
	clks[IMX8QXP_SAI0_RX_BCLK]	= imx_clk_fixed("sai0_rx_bclk", 0);
	clks[IMX8QXP_SAI0_TX_BCLK]	= imx_clk_fixed("sai0_tx_bclk", 0);
	clks[IMX8QXP_SAI1_RX_BCLK]	= imx_clk_fixed("sai1_rx_bclk", 0);
	clks[IMX8QXP_SAI1_TX_BCLK]	= imx_clk_fixed("sai1_tx_bclk", 0);
	clks[IMX8QXP_SAI2_RX_BCLK]	= imx_clk_fixed("sai2_rx_bclk", 0);
	clks[IMX8QXP_SAI3_RX_BCLK]	= imx_clk_fixed("sai3_rx_bclk", 0);
	clks[IMX8QXP_SAI4_RX_BCLK]	= imx_clk_fixed("sai4_rx_bclk", 0);

	clks[IMX8QXP_ACM_AUD_CLK0_SEL]      = imx_clk_mux_scu("acm_aud_clk0_sel",    base_acm+0x000000, 0, 5, aud_clk_sels, ARRAY_SIZE(aud_clk_sels), FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QXP_ACM_AUD_CLK0_CLK]      = imx_clk_gate_scu("acm_aud_clk0_clk", "acm_aud_clk0_sel", SC_R_AUDIO_CLK_0, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
	clks[IMX8QXP_ACM_AUD_CLK1_SEL]      = imx_clk_mux_scu("acm_aud_clk1_sel",    base_acm+0x010000, 0, 5, aud_clk_sels, ARRAY_SIZE(aud_clk_sels), FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QXP_ACM_AUD_CLK1_CLK]      = imx_clk_gate_scu("acm_aud_clk1_clk", "acm_aud_clk1_sel", SC_R_AUDIO_CLK_1, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
	clks[IMX8QXP_ACM_MCLKOUT0_SEL]      = imx_clk_mux_scu("acm_mclkout0_sel",    base_acm+0x020000, 0, 3, mclk_out_sels, ARRAY_SIZE(mclk_out_sels), FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QXP_ACM_MCLKOUT1_SEL]      = imx_clk_mux_scu("acm_mclkout1_sel",    base_acm+0x030000, 0, 3, mclk_out_sels, ARRAY_SIZE(mclk_out_sels), FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QXP_ACM_ASRC0_MUX_CLK_SEL] = imx_clk_mux_scu("acm_asrc0_mclk_sel",  base_acm+0x040000, 0, 2, NULL, 0, FUNCTION_NAME(PD_AUD_ASRC_0));
	clks[IMX8QXP_ACM_ASRC0_MUX_CLK_CLK] = imx_clk_gate_scu("aud_asrc0_mux_clk", "acm_asrc0_mclk_sel", SC_R_ASRC_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_ACM_ASRC1_MUX_CLK_SEL] = imx_clk_mux_scu("acm_asrc1_mclk_sel",  base_acm+0x050000, 0, 2, NULL, 0, FUNCTION_NAME(PD_AUD_ASRC_1));
	clks[IMX8QXP_ACM_ASRC1_MUX_CLK_CLK] = imx_clk_gate_scu("aud_asrc1_mux_clk", "acm_asrc1_mclk_sel", SC_R_ASRC_1, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_ACM_ESAI0_MCLK_SEL]    = imx_clk_mux_scu("acm_esai0_mclk_sel",  base_acm+0x060000, 0, 2, esai_mclk_sels, ARRAY_SIZE(esai_mclk_sels), FUNCTION_NAME(PD_AUD_ESAI_0));
	clks[IMX8QXP_ACM_SAI0_MCLK_SEL]     = imx_clk_mux_scu("acm_sai0_mclk_sel",   base_acm+0x0E0000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_0));
	clks[IMX8QXP_ACM_SAI1_MCLK_SEL]     = imx_clk_mux_scu("acm_sai1_mclk_sel",   base_acm+0x0F0000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_1));
	clks[IMX8QXP_ACM_SAI2_MCLK_SEL]     = imx_clk_mux_scu("acm_sai2_mclk_sel",   base_acm+0x100000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_2));
	clks[IMX8QXP_ACM_SAI3_MCLK_SEL]     = imx_clk_mux_scu("acm_sai3_mclk_sel",   base_acm+0x110000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_3));
	clks[IMX8QXP_ACM_SAI4_MCLK_SEL]     = imx_clk_mux_scu("acm_sai4_mclk_sel",   base_acm+0x140000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_4));
	clks[IMX8QXP_ACM_SAI5_MCLK_SEL]     = imx_clk_mux_scu("acm_sai5_mclk_sel",   base_acm+0x150000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_5));
	clks[IMX8QXP_ACM_SPDIF0_TX_CLK_SEL] = imx_clk_mux_scu("acm_spdif0_mclk_sel", base_acm+0x1A0000, 0, 2, spdif_mclk_sels, ARRAY_SIZE(spdif_mclk_sels), FUNCTION_NAME(PD_AUD_SPDIF_0));
	clks[IMX8QXP_ACM_MQS_TX_CLK_SEL]    = imx_clk_mux_scu("acm_mqs_mclk_sel",    base_acm+0x1C0000, 0, 2, mqs_mclk_sels, ARRAY_SIZE(mqs_mclk_sels), FUNCTION_NAME(PD_AUD_MQS_0));

	clks[IMX8QXP_AUD_AMIX_IPG]       = imx_clk_gate2_scu("aud_amix_ipg_clk",   "ipg_aud_clk_root", (void __iomem *)(AUD_AMIX_LPCG), 0, FUNCTION_NAME(PD_AUD_AMIX));
	clks[IMX8QXP_AUD_ESAI_0_IPG]     = imx_clk_gate2_scu("aud_esai0_ipg_clk",  "ipg_aud_clk_root", (void __iomem *)(AUD_ESAI_0_LPCG), 16, FUNCTION_NAME(PD_AUD_ESAI_0));
	clks[IMX8QXP_AUD_ESAI_0_EXTAL_IPG] = imx_clk_gate2_scu("aud_esai0_extal_clk", "acm_esai0_mclk_sel", (void __iomem *)(AUD_ESAI_0_LPCG), 0, FUNCTION_NAME(PD_AUD_ESAI_0));
	clks[IMX8QXP_AUD_SAI_0_IPG]      = imx_clk_gate2_scu("aud_sai0_ipg_clk",   "ipg_aud_clk_root", (void __iomem *)(AUD_SAI_0_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_0));
	clks[IMX8QXP_AUD_SAI_0_MCLK]     = imx_clk_gate2_scu("aud_sai0_mclk_clk",  "acm_sai0_mclk_sel", (void __iomem *)(AUD_SAI_0_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_0));
	clks[IMX8QXP_AUD_SAI_1_IPG]      = imx_clk_gate2_scu("aud_sai1_ipg_clk",   "ipg_aud_clk_root", (void __iomem *)(AUD_SAI_1_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_1));
	clks[IMX8QXP_AUD_SAI_1_MCLK]     = imx_clk_gate2_scu("aud_sai1_mclk_clk",  "acm_sai1_mclk_sel", (void __iomem *)(AUD_SAI_1_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_1));
	clks[IMX8QXP_AUD_SAI_2_IPG]      = imx_clk_gate2_scu("aud_sai2_ipg_clk",   "ipg_aud_clk_root", (void __iomem *)(AUD_SAI_2_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_2));
	clks[IMX8QXP_AUD_SAI_2_MCLK]     = imx_clk_gate2_scu("aud_sai2_mclk_clk",  "acm_sai2_mclk_sel", (void __iomem *)(AUD_SAI_2_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_2));
	clks[IMX8QXP_AUD_SAI_3_IPG]      = imx_clk_gate2_scu("aud_sai3_ipg_clk",   "ipg_aud_clk_root", (void __iomem *)(AUD_SAI_3_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_3));
	clks[IMX8QXP_AUD_SAI_3_MCLK]     = imx_clk_gate2_scu("aud_sai3_mclk_clk",  "acm_sai3_mclk_sel", (void __iomem *)(AUD_SAI_3_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_3));
	clks[IMX8QXP_AUD_SAI_4_IPG]      = imx_clk_gate2_scu("aud_sai4_ipg_clk",   "ipg_aud_clk_root", (void __iomem *)(AUD_SAI_4_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_4));
	clks[IMX8QXP_AUD_SAI_4_MCLK]     = imx_clk_gate2_scu("aud_sai4_mclk_clk",  "acm_sai4_mclk_sel", (void __iomem *)(AUD_SAI_4_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_4));
	clks[IMX8QXP_AUD_SAI_5_IPG]      = imx_clk_gate2_scu("aud_sai5_ipg_clk",   "ipg_aud_clk_root", (void __iomem *)(AUD_SAI_5_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_5));
	clks[IMX8QXP_AUD_SAI_5_MCLK]     = imx_clk_gate2_scu("aud_sai5_mclk_clk",  "acm_sai5_mclk_sel", (void __iomem *)(AUD_SAI_5_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_5));
	clks[IMX8QXP_AUD_MQS_IPG]        = imx_clk_gate2_scu("aud_mqs_ipg",        "ipg_aud_clk_root", (void __iomem *)(AUD_MQS_LPCG), 16, FUNCTION_NAME(PD_AUD_MQS_0));
	clks[IMX8QXP_AUD_MQS_HMCLK]      = imx_clk_gate2_scu("aud_mqs_hm_clk",     "acm_mqs_mclk_sel", (void __iomem *)(AUD_MQS_LPCG), 0, FUNCTION_NAME(PD_AUD_MQS_0));
	clks[IMX8QXP_AUD_GPT5_IPG]       = imx_clk_gate2_scu("aud_gpt5_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_5_LPCG), 16, FUNCTION_NAME(PD_AUD_GPT_5));
	clks[IMX8QXP_AUD_GPT5_CLKIN]     = imx_clk_gate2_scu("aud_gpt5_clkin", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_5_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_5));
	clks[IMX8QXP_AUD_GPT6_IPG]       = imx_clk_gate2_scu("aud_gpt6_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_6_LPCG), 16, FUNCTION_NAME(PD_AUD_GPT_6));
	clks[IMX8QXP_AUD_GPT6_CLKIN]     = imx_clk_gate2_scu("aud_gpt6_clkin", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_6_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_6));
	clks[IMX8QXP_AUD_GPT7_IPG]       = imx_clk_gate2_scu("aud_gpt7_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_7_LPCG), 16, FUNCTION_NAME(PD_AUD_GPT_7));
	clks[IMX8QXP_AUD_GPT7_CLKIN]     = imx_clk_gate2_scu("aud_gpt7_clkin", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_7_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_7));
	clks[IMX8QXP_AUD_GPT8_IPG]       = imx_clk_gate2_scu("aud_gpt8_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_8_LPCG), 16, FUNCTION_NAME(PD_AUD_GPT_8));
	clks[IMX8QXP_AUD_GPT8_CLKIN]     = imx_clk_gate2_scu("aud_gpt8_clkin", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_8_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_8));
	clks[IMX8QXP_AUD_GPT9_IPG]       = imx_clk_gate2_scu("aud_gpt9_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_9_LPCG), 16, FUNCTION_NAME(PD_AUD_GPT_9));
	clks[IMX8QXP_AUD_GPT9_CLKIN]     = imx_clk_gate2_scu("aud_gpt9_clkin", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_9_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_9));
	clks[IMX8QXP_AUD_GPT10_IPG]      = imx_clk_gate2_scu("aud_gpt10_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_10_LPCG), 16, FUNCTION_NAME(PD_AUD_GPT_10));
	clks[IMX8QXP_AUD_GPT10_CLKIN]    = imx_clk_gate2_scu("aud_gpt10_clkin", "ipg_aud_clk_root", (void __iomem *)(AUD_GPT_10_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_10));
	clks[IMX8QXP_AUD_MCLKOUT0]       = imx_clk_gate2_scu("aud_mclkout0", "acm_mclkout0_sel", (void __iomem *)(AUD_MCLKOUT0_LPCG), 0, FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QXP_AUD_MCLKOUT1]       = imx_clk_gate2_scu("aud_mclkout1", "acm_mclkout1_sel", (void __iomem *)(AUD_MCLKOUT1_LPCG), 0, FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QXP_AUD_SPDIF_0_GCLKW]  = imx_clk_gate2_scu("spdif0_gclkw", "ipg_aud_clk_root", (void __iomem *)(AUD_SPDIF_0_LPCG), 16, FUNCTION_NAME(PD_AUD_SPDIF_0));
	clks[IMX8QXP_AUD_SPDIF_0_TX_CLK] = imx_clk_gate2_scu("spdif0_tx_clk", "acm_spdif0_mclk_sel", (void __iomem *)(AUD_SPDIF_0_LPCG), 0, FUNCTION_NAME(PD_AUD_SPDIF_0));
	clks[IMX8QXP_AUD_ASRC_0_IPG]     = imx_clk_gate2_scu("aud_asrc0_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_ASRC_0_LPCG), 16, FUNCTION_NAME(PD_AUD_ASRC_0));
	clks[IMX8QXP_AUD_ASRC_1_IPG]     = imx_clk_gate2_scu("aud_asrc1_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_ASRC_1_LPCG), 16, FUNCTION_NAME(PD_AUD_ASRC_1));
	clks[IMX8QXP_AUD_HIFI_ADB_ACLK]  = imx_clk_gate2_scu("aud_hifi_adb_aclk", "ipg_aud_clk_root", (void __iomem *)(AUD_HIFI_LPCG), 16, FUNCTION_NAME(PD_AUD_HIFI));
	clks[IMX8QXP_AUD_HIFI_IPG]       = imx_clk_gate2_scu("aud_hifi_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_HIFI_LPCG), 20, FUNCTION_NAME(PD_AUD_HIFI));
	clks[IMX8QXP_AUD_HIFI_CORE_CLK]  = imx_clk_gate2_scu("aud_hifi_core_clk", "ipg_aud_clk_root", (void __iomem *)(AUD_HIFI_LPCG), 28, FUNCTION_NAME(PD_AUD_HIFI));
	clks[IMX8QXP_AUD_OCRAM_IPG]      = imx_clk_gate2_scu("aud_ocram_ipg", "ipg_aud_clk_root", (void __iomem *)(AUD_OCRAM_LPCG), 16, FUNCTION_NAME(PD_AUD_OCRAM));

	for (i = 0; i < ARRAY_SIZE(clks); i++)
		if (IS_ERR(clks[i]))
			pr_err("i.MX8QXP clk %d: register failed with %ld\n",
				i, PTR_ERR(clks[i]));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(ccm_node, of_clk_src_onecell_get, &clk_data);

	return 0;
}

static const struct of_device_id imx8qxp_match[] = {
	{ .compatible = "fsl,imx8qxp-clk", },
	{ /* sentinel value */ }
};

static struct platform_driver imx8qxp_clk_driver = {
	.driver = {
		.name = "imx8qxp-clk",
		.of_match_table = imx8qxp_match,
	},
	.probe = imx8qxp_clk_probe,
};

static int __init imx8qxp_clk_init(void)
{
	return platform_driver_register(&imx8qxp_clk_driver);
}
core_initcall(imx8qxp_clk_init);
