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

#include <dt-bindings/clock/imx8qm-clock.h>
#include <dt-bindings/soc/imx8_pd.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/types.h>

#include <soc/imx8/imx8qm/lpcg.h>
#include <soc/imx8/sc/sci.h>

#include "clk-imx8.h"

#define STR_VALUE(arg)      #arg
#define FUNCTION_NAME(name) STR_VALUE(name)

static const char *aud_clk_sels[] = {
	"aud_acm_aud_rec_clk0_clk",
	"aud_acm_aud_rec_clk1_clk",
	"mlb_clk",
	"hdmi_rx_mclk",
	"ext_aud_mclk0",
	"ext_aud_mclk1",
	"esai0_rx_clk",
	"esai0_rx_hf_clk",
	"esai0_tx_clk",
	"esai0_tx_hf_clk",
	"esai1_rx_clk",
	"esai1_rx_hf_clk",
	"esai1_tx_clk",
	"esai1_tx_hf_clk",
	"spdif0_rx",
	"spdif1_rx",
	"sai0_rx_bclk",
	"sai0_tx_bclk",
	"sai1_rx_bclk",
	"sai1_tx_bclk",
	"sai2_rx_bclk",
	"sai3_rx_bclk",
	"hdmi_rx_sai0_rx_bclk",
};

static const char *mclk_out_sels[] = {
	"aud_acm_aud_rec_clk0_clk",
	"aud_acm_aud_rec_clk1_clk",
	"mlb_clk",
	"hdmi_rx_mclk",
	"spdif0_rx",
	"spdif1_rx",
	"hdmi_rx_sai0_rx_bclk",
	"sai6_rx_bclk",
};

static const char *sai_mclk_sels[] = {
	"aud_acm_aud_pll_clk0_clk",
	"aud_acm_aud_pll_clk1_clk",
	"acm_aud_clk0_clk",
	"acm_aud_clk1_clk",
};

static const char *asrc_mux_clk_sels[] = {
	"hdmi_rx_sai0_rx_bclk",
	"hdmi_tx_sai0_tx_bclk",
	"dummy",
	"mlb_clk",
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

static const char *dc0_sels[] = {
	"dummy",
	"dummy",
	"dc0_pll0_clk",
	"dc0_pll1_clk",
	"dc0_bypass0_div",
};

static const char *dc1_sels[] = {
	"dummy",
	"dummy",
	"dc1_pll0_clk",
	"dc1_pll1_clk",
	"dc1_bypass0_div",
};


static struct clk *clks[IMX8QM_CLK_END];
static struct clk_onecell_data clk_data;

static const char *enet_sels[] = {"enet_25MHz", "enet_125MHz",};
static const char *enet0_rmii_tx_sels[] = {"enet0_ref_div", "dummy",};
static const char *enet1_rmii_tx_sels[] = {"enet1_ref_div", "dummy",};

#define LPCG_ADDR(arg) ((void __iomem *)(arg))

static int imx8qm_clk_probe(struct platform_device *pdev)
{
	struct device_node *ccm_node = pdev->dev.of_node;
	struct device_node *np_acm;
	void __iomem *base_acm;
	int i, ret;

	ret = imx8_clk_mu_init();
	if (ret)
		return ret;

	pr_info("***** imx8qm_clocks_init *****\n");

	clks[IMX8QM_CLK_DUMMY] = imx_clk_fixed("dummy", 0);

	/* ARM core */
	clks[IMX8QM_A53_DIV] = imx_clk_divider_scu("a53_div", SC_R_A53, SC_PM_CLK_CPU);
	clks[IMX8QM_A72_DIV] = imx_clk_divider_scu("a72_div", SC_R_A72, SC_PM_CLK_CPU);

	/* User Defined PLLs dividers */
	clks[IMX8QM_AUD_PLL0_DIV] = imx_clk_divider_scu("audio_pll0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_PLL);
	clks[IMX8QM_AUD_PLL1_DIV] = imx_clk_divider_scu("audio_pll1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_PLL);
	clks[IMX8QM_DC0_PLL0_DIV] = imx_clk_divider_scu("dc0_pll0_div", SC_R_DC_0_PLL_0, SC_PM_CLK_PLL);
	clks[IMX8QM_DC0_PLL1_DIV] = imx_clk_divider_scu("dc0_pll1_div", SC_R_DC_0_PLL_1, SC_PM_CLK_PLL);
	clks[IMX8QM_DC1_PLL1_DIV] = imx_clk_divider_scu("dc1_pll0_div", SC_R_DC_1_PLL_0, SC_PM_CLK_PLL);
	clks[IMX8QM_DC1_PLL1_DIV] = imx_clk_divider_scu("dc1_pll1_div", SC_R_DC_1_PLL_1, SC_PM_CLK_PLL);
	clks[IMX8QM_HDMI_AV_PLL_DIV] = imx_clk_divider_scu("hdmi_av_pll_div", SC_R_HDMI_PLL_1, SC_PM_CLK_PLL);
	clks[IMX8QM_HDMI_DIG_PLL_DIV] = imx_clk_divider_scu("hdmi_dig_pll_div", SC_R_HDMI_PLL_0, SC_PM_CLK_PLL);

	/* User Defined PLLs clocks*/
	clks[IMX8QM_AUD_PLL0] = imx_clk_gate_scu("audio_pll0_clk", "audio_pll0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_AUD_PLL1] = imx_clk_gate_scu("audio_pll1_clk", "audio_pll1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_DC0_PLL0_CLK] = imx_clk_gate_scu("dc0_pll0_clk", "dc0_pll0_div", SC_R_DC_0_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_DC0_PLL1_CLK] = imx_clk_gate_scu("dc0_pll1_clk", "dc0_pll1_div", SC_R_DC_0_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_DC1_PLL0_CLK] = imx_clk_gate_scu("dc1_pll0_clk", "dc1_pll0_div", SC_R_DC_1_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_DC1_PLL1_CLK] = imx_clk_gate_scu("dc1_pll1_clk", "dc1_pll1_div", SC_R_DC_1_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_HDMI_AV_PLL_CLK] = imx_clk_gate_scu("hdmi_av_pll_clk", "hdmi_av_pll_div", SC_R_HDMI_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_HDMI_DIG_PLL_CLK] = imx_clk_gate_scu("hdmi_dig_pll_clk", "hdmi_dig_pll_div", SC_R_HDMI_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);

	/* DMA SS */
	clks[IMX8QM_UART0_DIV] = imx_clk_divider_scu("uart0_div", SC_R_UART_0, SC_PM_CLK_PER);
	clks[IMX8QM_UART1_DIV] = imx_clk_divider_scu("uart1_div", SC_R_UART_1, SC_PM_CLK_PER);
	clks[IMX8QM_UART2_DIV] = imx_clk_divider_scu("uart2_div", SC_R_UART_2, SC_PM_CLK_PER);
	clks[IMX8QM_UART3_DIV] = imx_clk_divider_scu("uart3_div", SC_R_UART_3, SC_PM_CLK_PER);
	clks[IMX8QM_UART4_DIV] = imx_clk_divider_scu("uart4_div", SC_R_UART_4, SC_PM_CLK_PER);
	clks[IMX8QM_SPI0_DIV] = imx_clk_divider_scu("spi0_div", SC_R_SPI_0, SC_PM_CLK_PER);
	clks[IMX8QM_SPI1_DIV] = imx_clk_divider_scu("spi1_div", SC_R_SPI_1, SC_PM_CLK_PER);
	clks[IMX8QM_SPI2_DIV] = imx_clk_divider_scu("spi2_div", SC_R_SPI_2, SC_PM_CLK_PER);
	clks[IMX8QM_SPI3_DIV] = imx_clk_divider_scu("spi3_div", SC_R_SPI_3, SC_PM_CLK_PER);
	clks[IMX8QM_EMVSIM0_DIV] = imx_clk_divider_scu("emvsim0_div", SC_R_EMVSIM_0, SC_PM_CLK_PER);
	clks[IMX8QM_EMVSIM1_DIV] = imx_clk_divider_scu("emvsim1_div", SC_R_EMVSIM_1, SC_PM_CLK_PER);
	clks[IMX8QM_CAN0_DIV] = imx_clk_divider_scu("can0_div", SC_R_CAN_0, SC_PM_CLK_PER);
	clks[IMX8QM_CAN1_DIV] = imx_clk_divider_scu("can1_div", SC_R_CAN_1, SC_PM_CLK_PER);
	clks[IMX8QM_CAN2_DIV] = imx_clk_divider_scu("can2_div", SC_R_CAN_2, SC_PM_CLK_PER);
	clks[IMX8QM_I2C0_DIV] = imx_clk_divider_scu("i2c0_div", SC_R_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QM_I2C1_DIV] = imx_clk_divider_scu("i2c1_div", SC_R_I2C_1, SC_PM_CLK_PER);
	clks[IMX8QM_I2C2_DIV] = imx_clk_divider_scu("i2c2_div", SC_R_I2C_2, SC_PM_CLK_PER);
	clks[IMX8QM_I2C3_DIV] = imx_clk_divider_scu("i2c3_div", SC_R_I2C_3, SC_PM_CLK_PER);
	clks[IMX8QM_I2C4_DIV] = imx_clk_divider_scu("i2c4_div", SC_R_I2C_4, SC_PM_CLK_PER);
	clks[IMX8QM_FTM0_DIV] = imx_clk_divider_scu("ftm0_div", SC_R_FTM_0, SC_PM_CLK_PER);
	clks[IMX8QM_FTM1_DIV] = imx_clk_divider_scu("ftm1_div", SC_R_FTM_1, SC_PM_CLK_PER);
	clks[IMX8QM_ADC0_DIV] = imx_clk_divider_scu("adc0_div", SC_R_ADC_0, SC_PM_CLK_PER);
	clks[IMX8QM_ADC1_DIV] = imx_clk_divider_scu("adc1_div", SC_R_ADC_1, SC_PM_CLK_PER);

	/* LSIO SS */
	clks[IMX8QM_PWM0_DIV] = imx_clk_divider_scu("pwm_0_div", SC_R_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_PWM1_DIV] = imx_clk_divider_scu("pwm_1_div", SC_R_PWM_1, SC_PM_CLK_PER);
	clks[IMX8QM_PWM2_DIV] = imx_clk_divider_scu("pwm_2_div", SC_R_PWM_2, SC_PM_CLK_PER);
	clks[IMX8QM_PWM3_DIV] = imx_clk_divider_scu("pwm_3_div", SC_R_PWM_3, SC_PM_CLK_PER);
	clks[IMX8QM_PWM4_DIV] = imx_clk_divider_scu("pwm_4_div", SC_R_PWM_4, SC_PM_CLK_PER);
	clks[IMX8QM_PWM5_DIV] = imx_clk_divider_scu("pwm_5_div", SC_R_PWM_5, SC_PM_CLK_PER);
	clks[IMX8QM_PWM6_DIV] = imx_clk_divider_scu("pwm_6_div", SC_R_PWM_6, SC_PM_CLK_PER);
	clks[IMX8QM_PWM7_DIV] = imx_clk_divider_scu("pwm_7_div", SC_R_PWM_7, SC_PM_CLK_PER);
	clks[IMX8QM_FSPI0_DIV] = imx_clk_divider_scu("fspi_0_div", SC_R_FSPI_0, SC_PM_CLK_PER);
	clks[IMX8QM_FSPI1_DIV] = imx_clk_divider_scu("fspi_1_div", SC_R_FSPI_1, SC_PM_CLK_PER);
	clks[IMX8QM_GPT0_DIV] = imx_clk_divider_scu("gpt_0_div", SC_R_GPT_0, SC_PM_CLK_PER);
	clks[IMX8QM_GPT1_DIV] = imx_clk_divider_scu("gpt_1_div", SC_R_GPT_1, SC_PM_CLK_PER);
	clks[IMX8QM_GPT2_DIV] = imx_clk_divider_scu("gpt_2_div", SC_R_GPT_2, SC_PM_CLK_PER);
	clks[IMX8QM_GPT3_DIV] = imx_clk_divider_scu("gpt_3_div", SC_R_GPT_3, SC_PM_CLK_PER);
	clks[IMX8QM_GPT4_DIV] = imx_clk_divider_scu("gpt_4_div", SC_R_GPT_4, SC_PM_CLK_PER);

	/* lvds subsystem */
	clks[IMX8QM_LVDS0_BYPASS_CLK] = imx_clk_divider_scu("lvds0_bypass_clk", SC_R_LVDS_0, SC_PM_CLK_BYPASS);
	clks[IMX8QM_LVDS0_PIXEL_DIV] = imx_clk_divider_scu("lvds0_pixel_div", SC_R_LVDS_0, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS0_I2C0_DIV] = imx_clk_divider_scu("lvds0_i2c0_div", SC_R_LVDS_0_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS0_I2C1_DIV] = imx_clk_divider_scu("lvds0_i2c1_div", SC_R_LVDS_0_I2C_1, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS0_PWM0_DIV] = imx_clk_divider_scu("lvds0_pwm0_div", SC_R_LVDS_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS0_PHY_DIV] = imx_clk_divider_scu("lvds0_phy_div", SC_R_LVDS_0, SC_PM_CLK_PHY);
	clks[IMX8QM_LVDS1_BYPASS_CLK] = imx_clk_divider_scu("lvds1_bypass_clk", SC_R_LVDS_1, SC_PM_CLK_BYPASS);
	clks[IMX8QM_LVDS1_PIXEL_DIV] = imx_clk_divider_scu("lvds1_pixel_div", SC_R_LVDS_1, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_I2C0_DIV] = imx_clk_divider_scu("lvds1_i2c0_div", SC_R_LVDS_1_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_I2C1_DIV] = imx_clk_divider_scu("lvds1_i2c1_div", SC_R_LVDS_1_I2C_1, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_PWM0_DIV] = imx_clk_divider_scu("lvds1_pwm0_div", SC_R_LVDS_1_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_PHY_DIV] = imx_clk_divider_scu("lvds1_phy_div", SC_R_LVDS_1, SC_PM_CLK_PHY);

	/* vpu/zpu subsystem */
	clks[IMX8QM_VPU_DDR_DIV] = imx_clk_divider_scu("vpu_ddr_div", SC_R_VPU_PID0, SC_PM_CLK_SLV_BUS);
	clks[IMX8QM_VPU_SYS_DIV] = imx_clk_divider_scu("vpu_sys_div", SC_R_VPU_PID0, SC_PM_CLK_MST_BUS);
	clks[IMX8QM_VPU_XUVI_DIV] = imx_clk_divider_scu("vpu_xuvi_div", SC_R_VPU_PID0, SC_PM_CLK_PER);
	clks[IMX8QM_VPU_UART_DIV] = imx_clk_divider_scu("vpu_uart_div", SC_R_VPU_UART, SC_PM_CLK_PER);
	clks[IMX8QM_VPU_CORE_DIV] = imx_clk_divider_scu("vpu_core_div", SC_R_VPUCORE, SC_PM_CLK_PER);

	/* gpu */
	clks[IMX8QM_GPU0_CORE_DIV] = imx_clk_divider_scu("gpu_core0_div", SC_R_GPU_0_PID0, SC_PM_CLK_PER);
	clks[IMX8QM_GPU0_SHADER_DIV] = imx_clk_divider_scu("gpu_shader0_div", SC_R_GPU_0_PID0, SC_PM_CLK_MISC);
	clks[IMX8QM_GPU1_CORE_DIV] = imx_clk_divider_scu("gpu_core1_div", SC_R_GPU_1_PID0, SC_PM_CLK_PER);
	clks[IMX8QM_GPU1_SHADER_DIV] = imx_clk_divider_scu("gpu_shader1_div", SC_R_GPU_1_PID0, SC_PM_CLK_MISC);

	/* Connectivity */
	clks[IMX8QM_SDHC0_DIV] = imx_clk_divider_scu("sdhc0_div", SC_R_SDHC_0, SC_PM_CLK_PER);
	clks[IMX8QM_SDHC1_DIV] = imx_clk_divider_scu("sdhc1_div", SC_R_SDHC_1, SC_PM_CLK_PER);
	clks[IMX8QM_SDHC2_DIV] = imx_clk_divider_scu("sdhc2_div", SC_R_SDHC_2, SC_PM_CLK_PER);
	clks[IMX8QM_ENET0_ROOT_DIV] = imx_clk_divider_scu("enet0_root_div", SC_R_ENET_0, SC_PM_CLK_PER);
	clks[IMX8QM_ENET0_REF_DIV] = imx_clk_divider3_scu("enet0_ref_div", "enet0_root_clk", SC_R_ENET_0, SC_C_CLKDIV);
	clks[IMX8QM_ENET1_REF_DIV] = imx_clk_divider3_scu("enet1_ref_div", "enet1_root_clk", SC_R_ENET_1, SC_C_CLKDIV);
	clks[IMX8QM_ENET0_BYPASS_DIV] = imx_clk_divider_scu("enet0_bypass_div", SC_R_ENET_0, SC_PM_CLK_BYPASS);
	clks[IMX8QM_ENET0_RGMII_DIV] = imx_clk_divider_scu("enet0_rgmii_div", SC_R_ENET_0, SC_PM_CLK_MISC0);
	clks[IMX8QM_ENET1_ROOT_DIV] = imx_clk_divider_scu("enet1_root_div", SC_R_ENET_1, SC_PM_CLK_PER);
	clks[IMX8QM_ENET1_BYPASS_DIV] = imx_clk_divider_scu("enet1_bypass_div", SC_R_ENET_1, SC_PM_CLK_BYPASS);
	clks[IMX8QM_ENET1_RGMII_DIV] = imx_clk_divider_scu("enet1_rgmii_div", SC_R_ENET_1, SC_PM_CLK_MISC0);
	clks[IMX8QM_GPMI_BCH_IO_DIV] = imx_clk_divider_scu("gpmi_io_div", SC_R_NAND, SC_PM_CLK_MST_BUS);
	clks[IMX8QM_GPMI_BCH_DIV] = imx_clk_divider_scu("gpmi_bch_div", SC_R_NAND, SC_PM_CLK_PER);
	clks[IMX8QM_USB3_ACLK_DIV] = imx_clk_divider_scu("usb3_aclk_div", SC_R_USB_2, SC_PM_CLK_PER);
	clks[IMX8QM_USB3_BUS_DIV] = imx_clk_divider_scu("usb3_bus_div", SC_R_USB_2, SC_PM_CLK_MST_BUS);
	clks[IMX8QM_USB3_LPM_DIV] = imx_clk_divider_scu("usb3_lpm_div", SC_R_USB_2, SC_PM_CLK_MISC);

	/* Audio */
	clks[IMX8QM_AUD_ACM_AUD_PLL_CLK0_DIV] = imx_clk_divider2_scu("aud_acm_aud_pll_clk0_div", "audio_pll0_clk", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC0);
	clks[IMX8QM_AUD_ACM_AUD_PLL_CLK1_DIV] = imx_clk_divider2_scu("aud_acm_aud_pll_clk1_div", "audio_pll1_clk", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC0);
	clks[IMX8QM_AUD_ACM_AUD_REC_CLK0_DIV] = imx_clk_divider2_scu("aud_acm_aud_rec_clk0_div", "audio_pll0_clk", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC1);
	clks[IMX8QM_AUD_ACM_AUD_REC_CLK1_DIV] = imx_clk_divider2_scu("aud_acm_aud_rec_clk1_div", "audio_pll1_clk", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC1);

	/* MIPI CSI */
	clks[IMX8QM_CSI0_I2C0_DIV] = imx_clk_divider_scu("mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI0_PWM0_DIV] = imx_clk_divider_scu("mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI0_CORE_DIV] = imx_clk_divider_scu("mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI0_ESC_DIV] = imx_clk_divider_scu("mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC);
	clks[IMX8QM_CSI1_I2C0_DIV] = imx_clk_divider_scu("mipi_csi1_i2c0_div", SC_R_CSI_1_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI1_PWM0_DIV] = imx_clk_divider_scu("mipi_csi1_pwm0_div", SC_R_CSI_1_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI1_CORE_DIV] = imx_clk_divider_scu("mipi_csi1_core_div", SC_R_CSI_1, SC_PM_CLK_PER);
	clks[IMX8QM_CSI1_ESC_DIV] = imx_clk_divider_scu("mipi_csi1_esc_div", SC_R_CSI_1, SC_PM_CLK_MISC);

	/* RX-HDMI */
	clks[IMX8QM_HDMI_RX_I2S_BYPASS_CLK] = imx_clk_divider_scu("hdmi_rx_i2s_bypass_clk", SC_R_HDMI_RX_BYPASS, SC_PM_CLK_MISC0);
	clks[IMX8QM_HDMI_RX_BYPASS_CLK] = imx_clk_divider_scu("hdmi_rx_bypass_clk", SC_R_HDMI_RX_BYPASS, SC_PM_CLK_MISC1);
	clks[IMX8QM_HDMI_RX_SPDIF_BYPASS_CLK] = imx_clk_divider_scu("hdmi_rx_spdif_bypass_clk", SC_R_HDMI_RX_BYPASS, SC_PM_CLK_MISC2);
	clks[IMX8QM_HDMI_RX_I2C0_DIV] = imx_clk_divider_scu("hdmi_rx_i2c0_div", SC_R_HDMI_RX_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QM_HDMI_RX_SPDIF_DIV] = imx_clk_divider_scu("hdmi_rx_spdif_div", SC_R_HDMI_RX, SC_PM_CLK_MISC0);
	clks[IMX8QM_HDMI_RX_HD_REF_DIV] = imx_clk_divider_scu("hdmi_rx_hd_ref_div", SC_R_HDMI_RX, SC_PM_CLK_MISC1);
	clks[IMX8QM_HDMI_RX_HD_CORE_DIV] = imx_clk_divider_scu("hdmi_rx_hd_core_div", SC_R_HDMI_RX, SC_PM_CLK_MISC2);
	clks[IMX8QM_HDMI_RX_PXL_DIV] = imx_clk_divider_scu("hdmi_rx_pxl_div", SC_R_HDMI_RX, SC_PM_CLK_MISC3);
	clks[IMX8QM_HDMI_RX_I2S_DIV] = imx_clk_divider_scu("hdmi_rx_i2s_div", SC_R_HDMI_RX, SC_PM_CLK_MISC4);
	clks[IMX8QM_HDMI_RX_PWM_DIV] = imx_clk_divider_scu("hdmi_rx_pwm_div", SC_R_HDMI_RX_PWM_0, SC_PM_CLK_MISC2);

	/* DC SS */
	clks[IMX8QM_DC0_DISP0_SEL] = imx_clk_mux2_scu("dc0_disp0_sel", dc0_sels, ARRAY_SIZE(dc0_sels), SC_R_DC_0, SC_PM_CLK_MISC0);
	clks[IMX8QM_DC0_DISP0_DIV] = imx_clk_divider2_scu("dc0_disp0_div", "dc0_disp0_sel", SC_R_DC_0, SC_PM_CLK_MISC0);
	clks[IMX8QM_DC0_DISP1_SEL] = imx_clk_mux2_scu("dc0_disp1_sel", dc0_sels, ARRAY_SIZE(dc0_sels), SC_R_DC_0, SC_PM_CLK_MISC1);
	clks[IMX8QM_DC0_DISP1_DIV] = imx_clk_divider2_scu("dc0_disp1_div", "dc0_disp1_sel", SC_R_DC_0, SC_PM_CLK_MISC1);
	clks[IMX8QM_DC0_BYPASS_0_DIV] = imx_clk_divider_scu("dc0_bypass0_div", SC_R_DC_0_VIDEO0, SC_PM_CLK_MISC);
	clks[IMX8QM_DC0_BYPASS_1_DIV] = imx_clk_divider_scu("dc0_bypass1_div", SC_R_DC_0_VIDEO1, SC_PM_CLK_MISC);
	clks[IMX8QM_DC1_DISP0_SEL] = imx_clk_mux2_scu("dc1_disp0_sel", dc1_sels, ARRAY_SIZE(dc1_sels), SC_R_DC_1, SC_PM_CLK_MISC0);
	clks[IMX8QM_DC1_DISP0_DIV] = imx_clk_divider2_scu("dc1_disp0_div", "dc1_disp0_sel", SC_R_DC_1, SC_PM_CLK_MISC0);
	clks[IMX8QM_DC1_DISP1_SEL] = imx_clk_mux2_scu("dc1_disp1_sel", dc1_sels, ARRAY_SIZE(dc1_sels), SC_R_DC_1, SC_PM_CLK_MISC1);
	clks[IMX8QM_DC1_DISP1_DIV] = imx_clk_divider2_scu("dc1_disp1_div", "dc1_disp1_sel", SC_R_DC_1, SC_PM_CLK_MISC1);
	clks[IMX8QM_DC1_BYPASS_0_DIV] = imx_clk_divider_scu("dc1_bypass0_div", SC_R_DC_1_VIDEO0, SC_PM_CLK_BYPASS);
	clks[IMX8QM_DC1_BYPASS_1_DIV] = imx_clk_divider_scu("dc1_bypass1_div", SC_R_DC_1_VIDEO1, SC_PM_CLK_BYPASS);

	/* HDMI SS */
	clks[IMX8QM_HDMI_IPG_DIV] = imx_clk_divider_scu("hdmi_ipg_div", SC_R_HDMI, SC_PM_CLK_MISC);
	clks[IMX8QM_HDMI_I2S_BYPASS_CLK] = imx_clk_divider_scu("hdmi_i2s_bypass_clk", SC_R_HDMI_I2S, SC_PM_CLK_MISC0);
	clks[IMX8QM_HDMI_I2C0_DIV] = imx_clk_divider_scu("hdmi_i2c0_div", SC_R_HDMI_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QM_HDMI_PXL_DIV] = imx_clk_divider_scu("hdmi_pxl_div", SC_R_HDMI, SC_PM_CLK_MISC3);
	clks[IMX8QM_HDMI_PXL_LINK_DIV] = imx_clk_divider_scu("hdmi_pxl_link_div", SC_R_HDMI, SC_PM_CLK_MISC1);
	clks[IMX8QM_HDMI_PXL_MUX_DIV] = imx_clk_divider_scu("hdmi_pxl_mux_div", SC_R_HDMI, SC_PM_CLK_MISC0);
	clks[IMX8QM_HDMI_I2S_DIV] = imx_clk_divider_scu("hdmi_i2s_div", SC_R_HDMI, SC_PM_CLK_BYPASS);
	clks[IMX8QM_HDMI_HDP_CORE_DIV] = imx_clk_divider_scu("hdmi_core_div", SC_R_HDMI, SC_PM_CLK_MISC2);

	/* MIPI -DI SS */
	clks[IMX8QM_MIPI0_BYPASS_CLK] = imx_clk_divider_scu("mipi0_bypass_clk", SC_R_MIPI_0, SC_PM_CLK_BYPASS);
	clks[IMX8QM_MIPI0_PWM0_DIV] = imx_clk_divider_scu("mipi0_pwm0_div", SC_R_MIPI_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_MIPI0_DSI_TX_ESC_DIV] = imx_clk_divider_scu("mipi0_dsi_tx_esc_div", SC_R_MIPI_0, SC_PM_CLK_MST_BUS);
	clks[IMX8QM_MIPI0_DSI_RX_ESC_DIV] = imx_clk_divider_scu("mipi0_dsi_rx_esc_div", SC_R_MIPI_0, SC_PM_CLK_SLV_BUS);
	clks[IMX8QM_MIPI0_PXL_DIV] = imx_clk_divider_scu("mipi0_pxl_div", SC_R_MIPI_0, SC_PM_CLK_PER);
	clks[IMX8QM_MIPI1_BYPASS_CLK] = imx_clk_divider_scu("mipi1_bypass_clk", SC_R_MIPI_1, SC_PM_CLK_BYPASS);
	clks[IMX8QM_MIPI1_PWM0_DIV] = imx_clk_divider_scu("mipi1_pwm0_div", SC_R_MIPI_1_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_MIPI1_DSI_TX_ESC_DIV] = imx_clk_divider_scu("mipi1_dsi_tx_esc_div", SC_R_MIPI_1, SC_PM_CLK_MST_BUS);
	clks[IMX8QM_MIPI1_DSI_RX_ESC_DIV] = imx_clk_divider_scu("mipi1_dsi_rx_esc_div", SC_R_MIPI_1, SC_PM_CLK_SLV_BUS);
	clks[IMX8QM_MIPI1_PXL_DIV] = imx_clk_divider_scu("mipi1_pxl_div", SC_R_MIPI_1, SC_PM_CLK_PER);

	/* Fixed clocks. */
	clks[IMX8QM_IPG_DMA_CLK_ROOT] = imx_clk_fixed("ipg_dma_clk_root", SC_120MHZ);
	clks[IMX8QM_IPG_AUD_CLK_ROOT] = imx_clk_fixed("ipg_aud_clk_root", SC_175MHZ);
	clks[IMX8QM_AXI_CONN_CLK_ROOT] = imx_clk_fixed("axi_conn_clk_root", SC_333MHZ);
	clks[IMX8QM_AHB_CONN_CLK_ROOT] = imx_clk_fixed("ahb_conn_clk_root", SC_166MHZ);
	clks[IMX8QM_IPG_CONN_CLK_ROOT] = imx_clk_fixed("ipg_conn_clk_root", SC_83MHZ);
	clks[IMX8QM_IPG_MIPI_CSI_CLK_ROOT] = imx_clk_fixed("ipg_mipi_csi_clk_root", SC_120MHZ);
	clks[IMX8QM_DC_AXI_EXT_CLK] = imx_clk_fixed("axi_ext_dc_clk_root", SC_800MHZ);
	clks[IMX8QM_DC_AXI_INT_CLK] = imx_clk_fixed("axi_int_dc_clk_root", SC_400MHZ);
	clks[IMX8QM_DC_CFG_CLK] = imx_clk_fixed("cfg_dc_clk_root", SC_100MHZ);
	clks[IMX8QM_LVDS_IPG_CLK] = imx_clk_fixed("ipg_lvds_clk_root", SC_24MHZ);
	clks[IMX8QM_IMG_AXI_CLK] = imx_clk_fixed("axi_img_clk_root", SC_400MHZ);
	clks[IMX8QM_IMG_IPG_CLK] = imx_clk_fixed("ipg_img_clk_root", SC_200MHZ);
	clks[IMX8QM_IMG_PXL_CLK] = imx_clk_fixed("pxl_img_clk_root", SC_600MHZ);
	clks[IMX8QM_HSIO_AXI_CLK] = imx_clk_fixed("axi_hsio_clk_root", SC_400MHZ);
	clks[IMX8QM_HSIO_PER_CLK] = imx_clk_fixed("per_hsio_clk_root", SC_133MHZ);
	clks[IMX8QM_HDMI_RX_IPG_CLK] = imx_clk_fixed("ipg_hdmi_rx_clk_root", SC_400MHZ);
	clks[IMX8QM_ENET_25MHZ_CLK] = imx_clk_fixed("enet_25MHz", SC_25MHZ);
	clks[IMX8QM_ENET_125MHZ_CLK] = imx_clk_fixed("enet_125MHz", SC_125MHZ);
	clks[IMX8QM_LSIO_BUS_CLK] = imx_clk_fixed("lsio_bus_clk_root", SC_100MHZ);
	clks[IMX8QM_LSIO_MEM_CLK] = imx_clk_fixed("lsio_mem_clk_root", SC_200MHZ);
	clks[IMX8QM_24MHZ]    = imx_clk_fixed("xtal_24MHz", 24000000);
	clks[IMX8QM_GPT_3M]    = imx_clk_fixed("gpt_3m", 3000000);
	clks[IMX8QM_32KHZ]    = imx_clk_fixed("xtal_32KHz", 32768);
	clks[IMX8QM_MIPI0_CLK_ROOT] = imx_clk_fixed("mipi0_clk_root", SC_120MHZ);
	clks[IMX8QM_MIPI1_CLK_ROOT] = imx_clk_fixed("mipi1_clk_root", SC_120MHZ);

	/* Conectivity */
	clks[IMX8QM_SDHC0_IPG_CLK] = imx_clk_gate2_scu("sdhc0_ipg_clk", "ipg_conn_clk_root", LPCG_ADDR(USDHC_0_LPCG), 16, FUNCTION_NAME(PD_CONN_SDHC_0));
	clks[IMX8QM_SDHC1_IPG_CLK] = imx_clk_gate2_scu("sdhc1_ipg_clk", "ipg_conn_clk_root", LPCG_ADDR(USDHC_1_LPCG), 16, FUNCTION_NAME(PD_CONN_SDHC_1));
	clks[IMX8QM_SDHC2_IPG_CLK] = imx_clk_gate2_scu("sdhc2_ipg_clk", "ipg_conn_clk_root", LPCG_ADDR(USDHC_2_LPCG), 16, FUNCTION_NAME(PD_CONN_SDHC_2));
	clks[IMX8QM_SDHC0_CLK] = imx_clk_gate_scu("sdhc0_clk", "sdhc0_div", SC_R_SDHC_0, SC_PM_CLK_PER, LPCG_ADDR(USDHC_0_LPCG), 0, 0);
	clks[IMX8QM_SDHC1_CLK] = imx_clk_gate_scu("sdhc1_clk", "sdhc1_div", SC_R_SDHC_1, SC_PM_CLK_PER, LPCG_ADDR(USDHC_1_LPCG), 0, 0);
	clks[IMX8QM_SDHC2_CLK] = imx_clk_gate_scu("sdhc2_clk", "sdhc2_div", SC_R_SDHC_2, SC_PM_CLK_PER, LPCG_ADDR(USDHC_2_LPCG), 0, 0);
	clks[IMX8QM_ENET0_AHB_CLK]   = imx_clk_gate2_scu("enet0_ahb_clk", "axi_conn_clk_root", LPCG_ADDR(ENET_0_LPCG), 8, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QM_ENET0_IPG_S_CLK] = imx_clk_gate2_scu("enet0_ipg_s_clk", "ipg_conn_clk_root", LPCG_ADDR(ENET_0_LPCG), 20, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QM_ENET0_IPG_CLK]   = imx_clk_gate2_scu("enet0_ipg_clk", "enet0_ipg_s_clk", LPCG_ADDR(ENET_0_LPCG), 16, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QM_ENET1_AHB_CLK]   = imx_clk_gate2_scu("enet1_ahb_clk", "axi_conn_clk_root", LPCG_ADDR(ENET_1_LPCG), 8, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QM_ENET1_IPG_S_CLK] = imx_clk_gate2_scu("enet1_ipg_s_clk", "ipg_conn_clk_root", LPCG_ADDR(ENET_1_LPCG), 20, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QM_ENET1_IPG_CLK]   = imx_clk_gate2_scu("enet1_ipg_clk", "enet1_ipg_s_clk", LPCG_ADDR(ENET_1_LPCG), 16, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QM_ENET0_ROOT_CLK] = imx_clk_gate_scu("enet0_root_clk", "enet0_root_div", SC_R_ENET_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_ENET1_ROOT_CLK] = imx_clk_gate_scu("enet1_root_clk", "enet1_root_div", SC_R_ENET_1, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_ENET0_TX_CLK] = imx_clk_gate2_scu("enet0_tx_2x_clk", "enet0_root_clk", LPCG_ADDR(ENET_0_LPCG), 4, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QM_ENET1_TX_CLK] = imx_clk_gate2_scu("enet1_tx_2x_clk", "enet1_root_clk", LPCG_ADDR(ENET_1_LPCG), 4, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QM_ENET0_PTP_CLK] = imx_clk_gate2_scu("enet0_ptp_clk", "enet0_root_clk", LPCG_ADDR(ENET_0_LPCG), 0, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QM_ENET1_PTP_CLK] = imx_clk_gate2_scu("enet1_ptp_clk", "enet1_root_clk", LPCG_ADDR(ENET_1_LPCG), 0, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QM_ENET0_REF_25MHZ_125MHZ_SEL] = imx_clk_mux_gpr_scu("enet0_ref_25_125_sel", enet_sels, ARRAY_SIZE(enet_sels), SC_R_ENET_0, SC_C_SEL_125);
	clks[IMX8QM_ENET1_REF_25MHZ_125MHZ_SEL] = imx_clk_mux_gpr_scu("enet1_ref_25_125_sel", enet_sels, ARRAY_SIZE(enet_sels), SC_R_ENET_1, SC_C_SEL_125);
	clks[IMX8QM_ENET0_RMII_TX_SEL] = imx_clk_mux_gpr_scu("enet0_rmii_tx_sel", enet0_rmii_tx_sels, ARRAY_SIZE(enet0_rmii_tx_sels), SC_R_ENET_0, SC_C_TXCLK);
	clks[IMX8QM_ENET1_RMII_TX_SEL] = imx_clk_mux_gpr_scu("enet1_rmii_tx_sel", enet1_rmii_tx_sels, ARRAY_SIZE(enet1_rmii_tx_sels), SC_R_ENET_1, SC_C_TXCLK);
	clks[IMX8QM_ENET0_RGMII_TX_CLK] = imx_clk_gate2_scu("enet0_rgmii_tx_clk", "enet0_rmii_tx_sel", LPCG_ADDR(ENET_0_LPCG), 12, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QM_ENET1_RGMII_TX_CLK] = imx_clk_gate2_scu("enet1_rgmii_tx_clk", "enet1_rmii_tx_sel", LPCG_ADDR(ENET_1_LPCG), 12, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QM_ENET0_RMII_RX_CLK] = imx_clk_gate2_scu("enet0_rgmii_rx_clk", "enet0_rgmii_div", LPCG_ADDR(ENET_0_LPCG + 0x4), 0, FUNCTION_NAME(PD_CONN_ENET_0));
	clks[IMX8QM_ENET1_RMII_RX_CLK] = imx_clk_gate2_scu("enet1_rgmii_rx_clk", "enet1_rgmii_div", LPCG_ADDR(ENET_1_LPCG + 0x4), 0, FUNCTION_NAME(PD_CONN_ENET_1));
	clks[IMX8QM_ENET0_REF_25MHZ_125MHZ_CLK] = imx_clk_gate3_scu("enet0_ref_25_125_clk", "enet0_ref_25_125_sel", SC_R_ENET_0, SC_C_DISABLE_125, true);
	clks[IMX8QM_ENET1_REF_25MHZ_125MHZ_CLK] = imx_clk_gate3_scu("enet1_ref_25_125_clk", "enet1_ref_25_125_sel", SC_R_ENET_1, SC_C_DISABLE_125, true);
	clks[IMX8QM_ENET0_REF_50MHZ_CLK] = imx_clk_gate3_scu("enet0_ref_50_clk", NULL, SC_R_ENET_0, SC_C_DISABLE_50, true);
	clks[IMX8QM_ENET1_REF_50MHZ_CLK] = imx_clk_gate3_scu("enet1_ref_50_clk", NULL, SC_R_ENET_1, SC_C_DISABLE_50, true);
	clks[IMX8QM_GPMI_APB_CLK]   = imx_clk_gate2_scu("gpmi_apb_clk", "axi_conn_clk_root", LPCG_ADDR(NAND_LPCG), 16, FUNCTION_NAME(PD_CONN_NAND));
	clks[IMX8QM_GPMI_APB_BCH_CLK]   = imx_clk_gate2_scu("gpmi_apb_bch_clk", "axi_conn_clk_root", LPCG_ADDR(NAND_LPCG), 20, FUNCTION_NAME(PD_CONN_NAND));
	clks[IMX8QM_GPMI_BCH_IO_CLK] = imx_clk_gate_scu("gpmi_io_clk", "gpmi_io_div", SC_R_NAND, SC_PM_CLK_MST_BUS, LPCG_ADDR(NAND_LPCG), 4, 0);
	clks[IMX8QM_GPMI_BCH_CLK] = imx_clk_gate_scu("gpmi_bch_clk", "gpmi_bch_div", SC_R_NAND, SC_PM_CLK_PER, LPCG_ADDR(NAND_LPCG), 0, 0);
	clks[IMX8QM_APBHDMA_CLK] = imx_clk_gate2_scu("gpmi_clk", "axi_conn_clk_root", LPCG_ADDR(NAND_LPCG + 0x4), 16, FUNCTION_NAME(PD_CONN_NAND));
	clks[IMX8QM_USB2_OH_AHB_CLK]   = imx_clk_gate2_scu("usboh3", "ahb_conn_clk_root", LPCG_ADDR(USB_2_LPCG), 24, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QM_USB2_OH_IPG_S_CLK]   = imx_clk_gate2_scu("usboh3_ipg_s", "ipg_conn_clk_root", LPCG_ADDR(USB_2_LPCG), 16, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QM_USB2_OH_IPG_S_PL301_CLK]   = imx_clk_gate2_scu("usboh3_ipg_pl301_s", "ipg_conn_clk_root", LPCG_ADDR(USB_2_LPCG), 20, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QM_USB2_PHY_IPG_CLK]   = imx_clk_gate2_scu("usboh3_phy_clk", "ipg_conn_clk_root", LPCG_ADDR(USB_2_LPCG), 28, FUNCTION_NAME(PD_CONN_USB_0));
	clks[IMX8QM_USB3_IPG_CLK]   = imx_clk_gate2_scu("usb3_ipg_clk", "ipg_conn_clk_root", LPCG_ADDR(USB_3_LPCG), 16, FUNCTION_NAME(PD_CONN_USB_2));
	clks[IMX8QM_USB3_CORE_PCLK]   = imx_clk_gate2_scu("usb3_core_clk", "ipg_conn_clk_root", LPCG_ADDR(USB_3_LPCG), 20, FUNCTION_NAME(PD_CONN_USB_2));
	clks[IMX8QM_USB3_PHY_CLK]   = imx_clk_gate2_scu("usb3_phy_clk", "usb3_ipg_clk", LPCG_ADDR(USB_3_LPCG), 24, FUNCTION_NAME(PD_CONN_USB_2_PHY));
	clks[IMX8QM_USB3_ACLK] = imx_clk_gate_scu("usb3_aclk", "usb3_aclk_div", SC_R_USB_2, SC_PM_CLK_PER, LPCG_ADDR(USB_3_LPCG), 28, 0);
	clks[IMX8QM_USB3_BUS_CLK] = imx_clk_gate_scu("usb3_bus_clk", "usb3_bus_div", SC_R_USB_2, SC_PM_CLK_MST_BUS, LPCG_ADDR(USB_3_LPCG), 0, 0);
	clks[IMX8QM_USB3_LPM_CLK] = imx_clk_gate_scu("usb3_lpm_clk", "usb3_lpm_div", SC_R_USB_2, SC_PM_CLK_MISC, LPCG_ADDR(USB_3_LPCG), 4, 0);
	clks[IMX8QM_EDMA_CLK]   = imx_clk_gate2_scu("edma_clk", "axi_conn_clk_root", LPCG_ADDR(EDMA_LPCG), 0, FUNCTION_NAME(PD_CONN_DMA_4_CH0));
	clks[IMX8QM_EDMA_IPG_CLK]   = imx_clk_gate2_scu("edma_ipg_clk", "ipg_conn_clk_root", LPCG_ADDR(EDMA_LPCG), 16, FUNCTION_NAME(PD_CONN_DMA_4_CH0));
	clks[IMX8QM_MLB_HCLK]   = imx_clk_gate2_scu("mlb_hclk", "axi_conn_clk_root", LPCG_ADDR(MLB_LPCG), 20, FUNCTION_NAME(PD_CONN_MLB_0));
	clks[IMX8QM_MLB_CLK]   = imx_clk_gate2_scu("mlb_clk", "mlb_hclk", LPCG_ADDR(MLB_LPCG), 0, FUNCTION_NAME(PD_CONN_MLB_0));
	clks[IMX8QM_MLB_IPG_CLK]   = imx_clk_gate2_scu("mlb_ipg_clk", "ipg_conn_clk_root", LPCG_ADDR(MLB_LPCG), 16, FUNCTION_NAME(PD_CONN_MLB_0));

	/* DMA */
	clks[IMX8QM_UART0_IPG_CLK] = imx_clk_gate2_scu("uart0_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPUART_0_LPCG), 16, FUNCTION_NAME(PD_DMA_UART0));
	clks[IMX8QM_UART1_IPG_CLK] = imx_clk_gate2_scu("uart1_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPUART_1_LPCG), 16, FUNCTION_NAME(PD_DMA_UART1));
	clks[IMX8QM_UART2_IPG_CLK] = imx_clk_gate2_scu("uart2_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPUART_2_LPCG), 16, FUNCTION_NAME(PD_DMA_UART2));
	clks[IMX8QM_UART3_IPG_CLK] = imx_clk_gate2_scu("uart3_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPUART_3_LPCG), 16, FUNCTION_NAME(PD_DMA_UART3));
	clks[IMX8QM_UART4_IPG_CLK] = imx_clk_gate2_scu("uart4_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPUART_4_LPCG), 16, FUNCTION_NAME(PD_DMA_UART4));
	clks[IMX8QM_UART0_CLK] = imx_clk_gate_scu("uart0_clk", "uart0_div", SC_R_UART_0, SC_PM_CLK_PER, LPCG_ADDR(LPUART_0_LPCG), 0, 0);
	clks[IMX8QM_UART1_CLK] = imx_clk_gate_scu("uart1_clk", "uart1_div", SC_R_UART_1, SC_PM_CLK_PER, LPCG_ADDR(LPUART_1_LPCG), 0, 0);
	clks[IMX8QM_UART2_CLK] = imx_clk_gate_scu("uart2_clk", "uart2_div", SC_R_UART_2, SC_PM_CLK_PER, LPCG_ADDR(LPUART_2_LPCG), 0, 0);
	clks[IMX8QM_UART3_CLK] = imx_clk_gate_scu("uart3_clk", "uart3_div", SC_R_UART_3, SC_PM_CLK_PER, LPCG_ADDR(LPUART_3_LPCG), 0, 0);
	clks[IMX8QM_UART4_CLK] = imx_clk_gate_scu("uart4_clk", "uart4_div", SC_R_UART_4, SC_PM_CLK_PER, LPCG_ADDR(LPUART_4_LPCG), 0, 0);
	clks[IMX8QM_SPI0_IPG_CLK]   = imx_clk_gate2_scu("spi0_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPSPI_0_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_0));
	clks[IMX8QM_SPI1_IPG_CLK]   = imx_clk_gate2_scu("spi1_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPSPI_1_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_1));
	clks[IMX8QM_SPI2_IPG_CLK]   = imx_clk_gate2_scu("spi2_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPSPI_2_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_2));
	clks[IMX8QM_SPI3_IPG_CLK]   = imx_clk_gate2_scu("spi3_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPSPI_3_LPCG), 16, FUNCTION_NAME(PD_DMA_SPI_3));
	clks[IMX8QM_SPI0_CLK] = imx_clk_gate_scu("spi0_clk", "spi0_div", SC_R_SPI_0, SC_PM_CLK_PER, LPCG_ADDR(LPSPI_0_LPCG), 0, 0);
	clks[IMX8QM_SPI1_CLK] = imx_clk_gate_scu("spi1_clk", "spi1_div", SC_R_SPI_2, SC_PM_CLK_PER, LPCG_ADDR(LPSPI_1_LPCG), 0, 0);
	clks[IMX8QM_SPI2_CLK] = imx_clk_gate_scu("spi2_clk", "spi2_div", SC_R_SPI_2, SC_PM_CLK_PER, LPCG_ADDR(LPSPI_2_LPCG), 0, 0);
	clks[IMX8QM_SPI3_CLK] = imx_clk_gate_scu("spi3_clk", "spi3_div", SC_R_SPI_3, SC_PM_CLK_PER, LPCG_ADDR(LPSPI_3_LPCG), 0, 0);
	clks[IMX8QM_EMVSIM0_IPG_CLK]   = imx_clk_gate2_scu("emvsim0_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(EMVSIM_0_LPCG), 16, FUNCTION_NAME(PD_DMA_EMVSIM_0));
	clks[IMX8QM_EMVSIM1_IPG_CLK]   = imx_clk_gate2_scu("emvsim1_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(EMVSIM_1_LPCG), 16, FUNCTION_NAME(PD_DMA_EMVSIM_1));
	clks[IMX8QM_EMVSIM0_CLK] = imx_clk_gate_scu("emvsim0_clk", "emvsim0_div", SC_R_EMVSIM_0, SC_PM_CLK_PER, LPCG_ADDR(EMVSIM_0_LPCG), 0, 0);
	clks[IMX8QM_EMVSIM1_CLK] = imx_clk_gate_scu("emvsim1_clk", "emvsim1_div", SC_R_EMVSIM_0, SC_PM_CLK_PER, LPCG_ADDR(EMVSIM_1_LPCG), 0, 0);
	clks[IMX8QM_CAN0_IPG_CHI_CLK]   = imx_clk_gate2_scu("can0_ipg_chi_clk", "ipg_dma_clk_root", LPCG_ADDR(FLEX_CAN_0_LPCG), 20, FUNCTION_NAME(PD_DMA_CAN_0));
	clks[IMX8QM_CAN0_IPG_CLK]   = imx_clk_gate2_scu("can0_ipg_clk", "can0_ipg_chi_clk", LPCG_ADDR(FLEX_CAN_0_LPCG), 16, FUNCTION_NAME(PD_DMA_CAN_0));
	clks[IMX8QM_CAN1_IPG_CHI_CLK]   = imx_clk_gate2_scu("can1_ipg_chi_clk", "ipg_dma_clk_root", LPCG_ADDR(FLEX_CAN_1_LPCG), 20, FUNCTION_NAME(PD_DMA_CAN_1));
	clks[IMX8QM_CAN1_IPG_CLK]   = imx_clk_gate2_scu("can1_ipg_clk", "can1_ipg_chi_clk", LPCG_ADDR(FLEX_CAN_1_LPCG), 16, FUNCTION_NAME(PD_DMA_CAN_1));
	clks[IMX8QM_CAN2_IPG_CHI_CLK]   = imx_clk_gate2_scu("can2_ipg_chi_clk", "ipg_dma_clk_root", LPCG_ADDR(FLEX_CAN_2_LPCG), 20, FUNCTION_NAME(PD_DMA_CAN_2));
	clks[IMX8QM_CAN2_IPG_CLK]   = imx_clk_gate2_scu("can2_ipg_clk", "can2_ipg_chi_clk", LPCG_ADDR(FLEX_CAN_2_LPCG), 16, FUNCTION_NAME(PD_DMA_CAN_2));
	clks[IMX8QM_CAN0_CLK] = imx_clk_gate_scu("can0_clk", "can0_div", SC_R_CAN_0, SC_PM_CLK_PER, LPCG_ADDR(FLEX_CAN_0_LPCG), 0, 0);
	clks[IMX8QM_CAN1_CLK] = imx_clk_gate_scu("can1_clk", "can1_div", SC_R_CAN_1, SC_PM_CLK_PER, LPCG_ADDR(FLEX_CAN_1_LPCG), 0, 0);
	clks[IMX8QM_CAN2_CLK] = imx_clk_gate_scu("can2_clk", "can2_div", SC_R_CAN_2, SC_PM_CLK_PER, LPCG_ADDR(FLEX_CAN_2_LPCG), 0, 0);
	clks[IMX8QM_I2C0_IPG_CLK]   = imx_clk_gate2_scu("i2c0_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPI2C_0_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_0));
	clks[IMX8QM_I2C1_IPG_CLK]   = imx_clk_gate2_scu("i2c1_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPI2C_1_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_1));
	clks[IMX8QM_I2C2_IPG_CLK]   = imx_clk_gate2_scu("i2c2_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPI2C_2_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_2));
	clks[IMX8QM_I2C3_IPG_CLK]   = imx_clk_gate2_scu("i2c3_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPI2C_3_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_3));
	clks[IMX8QM_I2C4_IPG_CLK]   = imx_clk_gate2_scu("i2c4_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(LPI2C_4_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_4));
	clks[IMX8QM_I2C0_CLK] = imx_clk_gate_scu("i2c0_clk", "i2c0_div", SC_R_I2C_0, SC_PM_CLK_PER, LPCG_ADDR(LPI2C_0_LPCG), 0, 0);
	clks[IMX8QM_I2C1_CLK] = imx_clk_gate_scu("i2c1_clk", "i2c1_div", SC_R_I2C_1, SC_PM_CLK_PER, LPCG_ADDR(LPI2C_1_LPCG), 0, 0);
	clks[IMX8QM_I2C2_CLK] = imx_clk_gate_scu("i2c2_clk", "i2c2_div", SC_R_I2C_2, SC_PM_CLK_PER, LPCG_ADDR(LPI2C_2_LPCG), 0, 0);
	clks[IMX8QM_I2C3_CLK] = imx_clk_gate_scu("i2c3_clk", "i2c3_div", SC_R_I2C_3, SC_PM_CLK_PER, LPCG_ADDR(LPI2C_3_LPCG), 0, 0);
	clks[IMX8QM_I2C4_CLK] = imx_clk_gate_scu("i2c4_clk", "i2c4_div", SC_R_I2C_4, SC_PM_CLK_PER, LPCG_ADDR(LPI2C_4_LPCG), 0, 0);
	clks[IMX8QM_FTM0_IPG_CLK] = imx_clk_gate2_scu("ftm0_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(FTM_0_LPCG), 16, FUNCTION_NAME(PD_DMA_FTM_0));
	clks[IMX8QM_FTM1_IPG_CLK] = imx_clk_gate2_scu("ftm1_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(FTM_1_LPCG), 16, FUNCTION_NAME(PD_DMA_FTM_1));
	clks[IMX8QM_FTM0_CLK] = imx_clk_gate_scu("ftm0_clk", "ftm0_div", SC_R_FTM_0, SC_PM_CLK_PER, LPCG_ADDR(FTM_0_LPCG), 0, 0);
	clks[IMX8QM_FTM1_CLK] = imx_clk_gate_scu("ftm1_clk", "ftm1_div", SC_R_FTM_1, SC_PM_CLK_PER, LPCG_ADDR(FTM_1_LPCG), 0, 0);
	clks[IMX8QM_ADC0_IPG_CLK] = imx_clk_gate2_scu("adc0_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(ADC_0_LPCG), 16, FUNCTION_NAME(PD_DMA_ADC_0));
	clks[IMX8QM_ADC1_IPG_CLK] = imx_clk_gate2_scu("adc1_ipg_clk", "ipg_dma_clk_root", LPCG_ADDR(ADC_1_LPCG), 16, FUNCTION_NAME(PD_DMA_ADC_1));
	clks[IMX8QM_ADC0_CLK] = imx_clk_gate_scu("adc0_clk", "adc0_div", SC_R_ADC_0, SC_PM_CLK_PER, LPCG_ADDR(ADC_0_LPCG), 0, 0);
	clks[IMX8QM_ADC1_CLK] = imx_clk_gate_scu("adc1_clk", "adc1_div", SC_R_ADC_1, SC_PM_CLK_PER, LPCG_ADDR(ADC_1_LPCG), 0, 0);

	/* LSIO SS */
	clks[IMX8QM_PWM0_IPG_S_CLK] = imx_clk_gate_scu("pwm_0_ipg_s_clk", "pwm_0_div", SC_R_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(PWM_0_LPCG), 0x10, 0);
	clks[IMX8QM_PWM0_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_0_ipg_slv_clk", "pwm_0_ipg_s_clk", SC_R_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(PWM_0_LPCG), 0x14, 0);
	clks[IMX8QM_PWM0_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_0_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_0_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_0));
	clks[IMX8QM_PWM0_HF_CLK] = imx_clk_gate_scu("pwm_0_hf_clk", "pwm_0_ipg_slv_clk", SC_R_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(PWM_0_LPCG), 4, 0);
	clks[IMX8QM_PWM0_CLK] = imx_clk_gate_scu("pwm_0_clk", "pwm_0_ipg_slv_clk", SC_R_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(PWM_0_LPCG), 0, 0);
	clks[IMX8QM_PWM1_IPG_S_CLK] = imx_clk_gate_scu("pwm_1_ipg_s_clk", "pwm_1_div", SC_R_PWM_1, SC_PM_CLK_PER, LPCG_ADDR(PWM_1_LPCG), 0x10, 0);
	clks[IMX8QM_PWM1_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_1_ipg_slv_clk", "pwm_1_ipg_s_clk", SC_R_PWM_1, SC_PM_CLK_PER, LPCG_ADDR(PWM_1_LPCG), 0x14, 0);
	clks[IMX8QM_PWM1_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_1_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_1_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_1));
	clks[IMX8QM_PWM1_HF_CLK] = imx_clk_gate_scu("pwm_1_hf_clk", "pwm_1_ipg_slv_clk", SC_R_PWM_1, SC_PM_CLK_PER, LPCG_ADDR(PWM_1_LPCG), 4, 0);
	clks[IMX8QM_PWM1_CLK] = imx_clk_gate_scu("pwm_1_clk", "pwm_1_ipg_slv_clk", SC_R_PWM_1, SC_PM_CLK_PER, LPCG_ADDR(PWM_1_LPCG), 0, 0);
	clks[IMX8QM_PWM2_IPG_S_CLK] = imx_clk_gate_scu("pwm_2_ipg_s_clk", "pwm_2_div", SC_R_PWM_2, SC_PM_CLK_PER, LPCG_ADDR(PWM_2_LPCG), 0x10, 0);
	clks[IMX8QM_PWM2_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_2_ipg_slv_clk", "pwm_2_ipg_s_clk", SC_R_PWM_2, SC_PM_CLK_PER, LPCG_ADDR(PWM_2_LPCG), 0x14, 0);
	clks[IMX8QM_PWM2_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_2_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_2_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_2));
	clks[IMX8QM_PWM2_HF_CLK] = imx_clk_gate_scu("pwm_2_hf_clk", "pwm_2_ipg_slv_clk", SC_R_PWM_2, SC_PM_CLK_PER, LPCG_ADDR(PWM_2_LPCG), 4, 0);
	clks[IMX8QM_PWM2_CLK] = imx_clk_gate_scu("pwm_2_clk", "pwm_2_ipg_slv_clk", SC_R_PWM_2, SC_PM_CLK_PER, LPCG_ADDR(PWM_2_LPCG), 0, 0);
	clks[IMX8QM_PWM3_IPG_S_CLK] = imx_clk_gate_scu("pwm_3_ipg_s_clk", "pwm_3_div", SC_R_PWM_3, SC_PM_CLK_PER, LPCG_ADDR(PWM_3_LPCG), 0x10, 0);
	clks[IMX8QM_PWM3_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_3_ipg_slv_clk", "pwm_3_ipg_s_clk", SC_R_PWM_3, SC_PM_CLK_PER, LPCG_ADDR(PWM_3_LPCG), 0x14, 0);
	clks[IMX8QM_PWM3_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_3_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_3_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_3));
	clks[IMX8QM_PWM3_HF_CLK] = imx_clk_gate_scu("pwm_3_hf_clk", "pwm_3_ipg_slv_clk", SC_R_PWM_3, SC_PM_CLK_PER, LPCG_ADDR(PWM_3_LPCG), 4, 0);
	clks[IMX8QM_PWM3_CLK] = imx_clk_gate_scu("pwm_3_clk", "pwm_3_ipg_slv_clk", SC_R_PWM_3, SC_PM_CLK_PER, LPCG_ADDR(PWM_3_LPCG), 0, 0);
	clks[IMX8QM_PWM4_IPG_S_CLK] = imx_clk_gate_scu("pwm_4_ipg_s_clk", "pwm_4_div", SC_R_PWM_4, SC_PM_CLK_PER, LPCG_ADDR(PWM_4_LPCG), 0x10, 0);
	clks[IMX8QM_PWM4_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_4_ipg_slv_clk", "pwm_4_ipg_s_clk", SC_R_PWM_4, SC_PM_CLK_PER, LPCG_ADDR(PWM_4_LPCG), 0x14, 0);
	clks[IMX8QM_PWM4_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_4_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_4_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_4));
	clks[IMX8QM_PWM4_HF_CLK] = imx_clk_gate_scu("pwm_4_hf_clk", "pwm_4_ipg_slv_clk", SC_R_PWM_4, SC_PM_CLK_PER, LPCG_ADDR(PWM_4_LPCG), 4, 0);
	clks[IMX8QM_PWM4_CLK] = imx_clk_gate_scu("pwm_4_clk", "pwm_4_ipg_slv_clk", SC_R_PWM_4, SC_PM_CLK_PER, LPCG_ADDR(PWM_4_LPCG), 0, 0);
	clks[IMX8QM_PWM5_IPG_S_CLK] = imx_clk_gate_scu("pwm_5_ipg_s_clk", "pwm_5_div", SC_R_PWM_5, SC_PM_CLK_PER, LPCG_ADDR(PWM_5_LPCG), 0x10, 0);
	clks[IMX8QM_PWM5_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_5_ipg_slv_clk", "pwm_5_ipg_s_clk", SC_R_PWM_5, SC_PM_CLK_PER, LPCG_ADDR(PWM_5_LPCG), 0x14, 0);
	clks[IMX8QM_PWM5_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_5_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_5_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_5));
	clks[IMX8QM_PWM5_HF_CLK] = imx_clk_gate_scu("pwm_5_hf_clk", "pwm_5_ipg_slv_clk", SC_R_PWM_5, SC_PM_CLK_PER, LPCG_ADDR(PWM_5_LPCG), 4, 0);
	clks[IMX8QM_PWM5_CLK] = imx_clk_gate_scu("pwm_5_clk", "pwm_5_ipg_slv_clk", SC_R_PWM_5, SC_PM_CLK_PER, LPCG_ADDR(PWM_5_LPCG), 0, 0);
	clks[IMX8QM_PWM6_IPG_S_CLK] = imx_clk_gate_scu("pwm_6_ipg_s_clk", "pwm_6_div", SC_R_PWM_6, SC_PM_CLK_PER, LPCG_ADDR(PWM_6_LPCG), 0x10, 0);
	clks[IMX8QM_PWM6_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_6_ipg_slv_clk", "pwm_6_ipg_s_clk", SC_R_PWM_6, SC_PM_CLK_PER, LPCG_ADDR(PWM_6_LPCG), 0x14, 0);
	clks[IMX8QM_PWM6_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_6_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_6_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_6));
	clks[IMX8QM_PWM6_HF_CLK] = imx_clk_gate_scu("pwm_6_hf_clk", "pwm_6_ipg_slv_clk", SC_R_PWM_6, SC_PM_CLK_PER, LPCG_ADDR(PWM_6_LPCG), 4, 0);
	clks[IMX8QM_PWM6_CLK] = imx_clk_gate_scu("pwm_6_clk", "pwm_6_ipg_slv_clk", SC_R_PWM_6, SC_PM_CLK_PER, LPCG_ADDR(PWM_6_LPCG), 0, 0);
	clks[IMX8QM_PWM7_IPG_S_CLK] = imx_clk_gate_scu("pwm_7_ipg_s_clk", "pwm_7_div", SC_R_PWM_7, SC_PM_CLK_PER, LPCG_ADDR(PWM_7_LPCG), 0x10, 0);
	clks[IMX8QM_PWM7_IPG_SLV_CLK] = imx_clk_gate_scu("pwm_7_ipg_slv_clk", "pwm_7_ipg_s_clk", SC_R_PWM_7, SC_PM_CLK_PER, LPCG_ADDR(PWM_7_LPCG), 0x14, 0);
	clks[IMX8QM_PWM7_IPG_MSTR_CLK] = imx_clk_gate2_scu("pwm_7_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(PWM_7_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_PWM_7));
	clks[IMX8QM_PWM7_HF_CLK] = imx_clk_gate_scu("pwm_7_hf_clk", "pwm_7_ipg_slv_clk", SC_R_PWM_7, SC_PM_CLK_PER, LPCG_ADDR(PWM_7_LPCG), 4, 0);
	clks[IMX8QM_PWM7_CLK] = imx_clk_gate_scu("pwm_7_clk", "pwm_7_ipg_slv_clk", SC_R_PWM_7, SC_PM_CLK_PER, LPCG_ADDR(PWM_7_LPCG), 0, 0);
	clks[IMX8QM_GPT0_IPG_S_CLK] = imx_clk_gate_scu("gpt_0_ipg_s_clk", "gpt_0_div", SC_R_GPT_0, SC_PM_CLK_PER, LPCG_ADDR(GPT_0_LPCG), 0x10, 0);
	clks[IMX8QM_GPT0_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_0_ipg_slv_clk", "gpt_0_ipg_s_clk", SC_R_GPT_0, SC_PM_CLK_PER, LPCG_ADDR(GPT_0_LPCG), 0x14, 0);
	clks[IMX8QM_GPT0_CLK] = imx_clk_gate_scu("gpt_0_clk", "gpt_0_ipg_slv_clk", SC_R_GPT_0, SC_PM_CLK_PER, LPCG_ADDR(GPT_0_LPCG), 0, 0);
	clks[IMX8QM_GPT0_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_0_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(GPT_0_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_0));
	clks[IMX8QM_GPT0_HF_CLK] = imx_clk_gate_scu("gpt_0_hf_clk", "gpt_0_ipg_slv_clk", SC_R_GPT_0, SC_PM_CLK_PER, LPCG_ADDR(GPT_0_LPCG), 4, 0);
	clks[IMX8QM_GPT1_IPG_S_CLK] = imx_clk_gate_scu("gpt_1_ipg_s_clk", "gpt_1_div", SC_R_GPT_1, SC_PM_CLK_PER, LPCG_ADDR(GPT_1_LPCG), 0x10, 0);
	clks[IMX8QM_GPT1_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_1_ipg_slv_clk", "gpt_1_ipg_s_clk", SC_R_GPT_1, SC_PM_CLK_PER, LPCG_ADDR(GPT_1_LPCG), 0x14, 0);
	clks[IMX8QM_GPT1_CLK] = imx_clk_gate_scu("gpt_1_clk", "gpt_1_ipg_slv_clk", SC_R_GPT_1, SC_PM_CLK_PER, LPCG_ADDR(GPT_1_LPCG), 0, 0);
	clks[IMX8QM_GPT1_HF_CLK] = imx_clk_gate_scu("gpt_1_hf_clk", "gpt_1_ipg_slv_clk", SC_R_GPT_1, SC_PM_CLK_PER, LPCG_ADDR(GPT_1_LPCG), 4, 0);
	clks[IMX8QM_GPT1_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_1_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(GPT_1_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_1));
	clks[IMX8QM_GPT2_IPG_S_CLK] = imx_clk_gate_scu("gpt_2_ipg_s_clk", "gpt_2_div", SC_R_GPT_2, SC_PM_CLK_PER, LPCG_ADDR(GPT_2_LPCG), 0x10, 0);
	clks[IMX8QM_GPT2_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_2_ipg_slv_clk", "gpt_2_ipg_s_clk", SC_R_GPT_2, SC_PM_CLK_PER, LPCG_ADDR(GPT_2_LPCG), 0x14, 0);
	clks[IMX8QM_GPT2_CLK] = imx_clk_gate_scu("gpt_2_clk", "gpt_2_ipg_slv_clk", SC_R_GPT_2, SC_PM_CLK_PER, LPCG_ADDR(GPT_2_LPCG), 0, 0);
	clks[IMX8QM_GPT2_HF_CLK] = imx_clk_gate_scu("gpt_2_hf_clk", "gpt_2_ipg_slv_clk", SC_R_GPT_2, SC_PM_CLK_PER, LPCG_ADDR(GPT_2_LPCG), 4, 0);
	clks[IMX8QM_GPT2_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_2_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(GPT_2_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_2));
	clks[IMX8QM_GPT3_IPG_S_CLK] = imx_clk_gate_scu("gpt_3_ipg_s_clk", "gpt_3_div", SC_R_GPT_3, SC_PM_CLK_PER, LPCG_ADDR(GPT_3_LPCG), 0x10, 0);
	clks[IMX8QM_GPT3_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_3_ipg_slv_clk", "gpt_3_ipg_s_clk", SC_R_GPT_3, SC_PM_CLK_PER, LPCG_ADDR(GPT_3_LPCG), 0x14, 0);
	clks[IMX8QM_GPT3_CLK] = imx_clk_gate_scu("gpt_3_clk", "gpt_3_ipg_slv_clk", SC_R_GPT_3, SC_PM_CLK_PER, LPCG_ADDR(GPT_3_LPCG), 0, 0);
	clks[IMX8QM_GPT3_HF_CLK] = imx_clk_gate_scu("gpt_3_hf_clk", "gpt_3_ipg_slv_clk", SC_R_GPT_3, SC_PM_CLK_PER, LPCG_ADDR(GPT_3_LPCG), 4, 0);
	clks[IMX8QM_GPT3_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_3_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(GPT_3_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_3));
	clks[IMX8QM_GPT4_IPG_S_CLK] = imx_clk_gate_scu("gpt_4_ipg_s_clk", "gpt_4_div", SC_R_GPT_4, SC_PM_CLK_PER, LPCG_ADDR(GPT_4_LPCG), 0x10, 0);
	clks[IMX8QM_GPT4_IPG_SLV_CLK] = imx_clk_gate_scu("gpt_4_ipg_slv_clk", "gpt_4_ipg_s_clk", SC_R_GPT_4, SC_PM_CLK_PER, LPCG_ADDR(GPT_4_LPCG), 0x14, 0);
	clks[IMX8QM_GPT4_CLK] = imx_clk_gate_scu("gpt_4_clk", "gpt_4_ipg_slv_clk", SC_R_GPT_4, SC_PM_CLK_PER, LPCG_ADDR(GPT_4_LPCG), 0, 0);
	clks[IMX8QM_GPT4_HF_CLK] = imx_clk_gate_scu("gpt_4_hf_clk", "gpt_4_ipg_slv_clk", SC_R_GPT_4, SC_PM_CLK_PER, LPCG_ADDR(GPT_4_LPCG), 4, 0);
	clks[IMX8QM_GPT4_IPG_MSTR_CLK] = imx_clk_gate2_scu("gpt_4_ipg_mstr_clk", "lsio_bus_clk_root", LPCG_ADDR(GPT_4_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_GPT_4));
	clks[IMX8QM_FSPI0_HCLK] = imx_clk_gate2_scu("fspi0_hclk_clk", "lsio_mem_clk_root", LPCG_ADDR(FSPI_0_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_FSPI_0));
	clks[IMX8QM_FSPI0_IPG_S_CLK] = imx_clk_gate2_scu("fspi0_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(FSPI_0_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_FSPI_0));
	clks[IMX8QM_FSPI0_IPG_CLK] = imx_clk_gate2_scu("fspi0_ipg_clk", "fspi0_ipg_s_clk", LPCG_ADDR(FSPI_0_LPCG), 0x14, FUNCTION_NAME(PD_LSIO_FSPI_0));
	clks[IMX8QM_FSPI0_CLK] = imx_clk_gate_scu("fspi_0_clk", "fspi_0_div", SC_R_FSPI_0, SC_PM_CLK_PER, LPCG_ADDR(FSPI_0_LPCG), 0, 0);
	clks[IMX8QM_FSPI1_HCLK] = imx_clk_gate2_scu("fspi1_hclk_clk", "lsio_mem_clk_root", LPCG_ADDR(FSPI_1_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_FSPI_1));
	clks[IMX8QM_FSPI1_IPG_S_CLK] = imx_clk_gate2_scu("fspi1_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(FSPI_1_LPCG), 0x18, FUNCTION_NAME(PD_LSIO_FSPI_1));
	clks[IMX8QM_FSPI1_IPG_CLK] = imx_clk_gate2_scu("fspi1_ipg_clk", "fspi1_ipg_s_clk", LPCG_ADDR(FSPI_1_LPCG), 0x14, FUNCTION_NAME(PD_LSIO_FSPI_1));
	clks[IMX8QM_FSPI1_CLK] = imx_clk_gate_scu("fspi_1_clk", "fspi_1_div", SC_R_FSPI_1, SC_PM_CLK_PER, LPCG_ADDR(FSPI_1_LPCG), 0, 0);
	clks[IMX8QM_GPIO0_IPG_S_CLK] = imx_clk_gate2_scu("gpio0_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_0_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_0));
	clks[IMX8QM_GPIO1_IPG_S_CLK] = imx_clk_gate2_scu("gpio1_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_1_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_1));
	clks[IMX8QM_GPIO2_IPG_S_CLK] = imx_clk_gate2_scu("gpio2_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_2_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_2));
	clks[IMX8QM_GPIO3_IPG_S_CLK] = imx_clk_gate2_scu("gpio3_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_3_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_3));
	clks[IMX8QM_GPIO4_IPG_S_CLK] = imx_clk_gate2_scu("gpio4_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_4_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_4));
	clks[IMX8QM_GPIO5_IPG_S_CLK] = imx_clk_gate2_scu("gpio5_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_5_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_5));
	clks[IMX8QM_GPIO6_IPG_S_CLK] = imx_clk_gate2_scu("gpio6_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_6_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_6));
	clks[IMX8QM_GPIO7_IPG_S_CLK] = imx_clk_gate2_scu("gpio7_ipg_s_clk", "lsio_bus_clk_root", LPCG_ADDR(GPIO_7_LPCG), 0x10, FUNCTION_NAME(PD_LSIO_GPIO_7));
	clks[IMX8QM_ROMCP_REG_CLK] = imx_clk_gate2_scu("romcp_reg_clk", "lsio_bus_clk_root", LPCG_ADDR(ROMCP_LPCG), 0x10, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QM_ROMCP_CLK] = imx_clk_gate2_scu("romcp_clk", "lsio_mem_clk_root", LPCG_ADDR(ROMCP_LPCG), 0x0, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QM_96KROM_CLK] = imx_clk_gate2_scu("96krom_clk", "lsio_mem_clk_root", LPCG_ADDR(ROMCP_LPCG), 0x4, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QM_OCRAM_MEM_CLK] = imx_clk_gate2_scu("ocram_lk", "lsio_mem_clk_root", LPCG_ADDR(OCRAM_LPCG), 0x4, FUNCTION_NAME(PD_LSIO));
	clks[IMX8QM_OCRAM_CTRL_CLK] = imx_clk_gate2_scu("ocram_ctrl_clk", "lsio_mem_clk_root", LPCG_ADDR(OCRAM_LPCG), 0x0, FUNCTION_NAME(PD_LSIO));

	/* Audio */
	clks[IMX8QM_AUD_ACM_AUD_PLL_CLK0_CLK] = imx_clk_gate_scu("aud_acm_aud_pll_clk0_clk", "aud_acm_aud_pll_clk0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC0, LPCG_ADDR(AUD_PLL_CLK0_LPCG), 0, 0);
	clks[IMX8QM_AUD_ACM_AUD_PLL_CLK1_CLK] = imx_clk_gate_scu("aud_acm_aud_pll_clk1_clk", "aud_acm_aud_pll_clk1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC0, LPCG_ADDR(AUD_PLL_CLK1_LPCG), 0, 0);
	clks[IMX8QM_AUD_ACM_AUD_REC_CLK0_CLK] = imx_clk_gate_scu("aud_acm_aud_rec_clk0_clk", "aud_acm_aud_rec_clk0_div", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC1, LPCG_ADDR(AUD_REC_CLK0_LPCG), 0, 0);
	clks[IMX8QM_AUD_ACM_AUD_REC_CLK1_CLK] = imx_clk_gate_scu("aud_acm_aud_rec_clk1_clk", "aud_acm_aud_rec_clk1_div", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC1, LPCG_ADDR(AUD_REC_CLK1_LPCG), 0, 0);

	clks[IMX8QM_HDMI_RX_MCLK]	= imx_clk_fixed("hdmi_rx_mclk", 0);
	clks[IMX8QM_EXT_AUD_MCLK0]	= imx_clk_fixed("ext_aud_mclk0", 0);
	clks[IMX8QM_EXT_AUD_MCLK1]	= imx_clk_fixed("ext_aud_mclk1", 0);
	clks[IMX8QM_ESAI0_RX_CLK]	= imx_clk_fixed("esai0_rx_clk", 0);
	clks[IMX8QM_ESAI0_RX_HF_CLK]	= imx_clk_fixed("esai0_rx_hf_clk", 0);
	clks[IMX8QM_ESAI0_TX_CLK]	= imx_clk_fixed("esai0_tx_clk", 0);
	clks[IMX8QM_ESAI0_TX_HF_CLK]	= imx_clk_fixed("esai0_tx_hf_clk", 0);
	clks[IMX8QM_ESAI1_RX_CLK]	= imx_clk_fixed("esai1_rx_clk", 0);
	clks[IMX8QM_ESAI1_RX_HF_CLK]	= imx_clk_fixed("esai1_rx_hf_clk", 0);
	clks[IMX8QM_ESAI1_TX_CLK]	= imx_clk_fixed("esai1_tx_clk", 0);
	clks[IMX8QM_ESAI1_TX_HF_CLK]	= imx_clk_fixed("esai1_tx_hf_clk", 0);
	clks[IMX8QM_SPDIF0_RX]		= imx_clk_fixed("spdif0_rx", 0);
	clks[IMX8QM_SPDIF1_RX]		= imx_clk_fixed("spdif1_rx", 0);
	clks[IMX8QM_SAI0_RX_BCLK]	= imx_clk_fixed("sai0_rx_bclk", 0);
	clks[IMX8QM_SAI0_TX_BCLK]	= imx_clk_fixed("sai0_tx_bclk", 0);
	clks[IMX8QM_SAI1_RX_BCLK]	= imx_clk_fixed("sai1_rx_bclk", 0);
	clks[IMX8QM_SAI1_TX_BCLK]	= imx_clk_fixed("sai1_tx_bclk", 0);
	clks[IMX8QM_SAI2_RX_BCLK]	= imx_clk_fixed("sai2_rx_bclk", 0);
	clks[IMX8QM_SAI3_RX_BCLK]	= imx_clk_fixed("sai3_rx_bclk", 0);
	clks[IMX8QM_HDMI_RX_SAI0_RX_BCLK]  = imx_clk_fixed("hdmi_rx_sai0_rx_bclk", 0);
	clks[IMX8QM_SAI6_RX_BCLK]          = imx_clk_fixed("sai6_rx_bclk", 0);
	clks[IMX8QM_HDMI_TX_SAI0_TX_BCLK]  = imx_clk_fixed("hdmi_tx_sai0_tx_bclk", 0);

	np_acm = of_find_compatible_node(NULL, NULL, "nxp,imx8qm-acm");
	if (np_acm) {
		base_acm = of_iomap(np_acm, 0);
		WARN_ON(!base_acm);

		clks[IMX8QM_ACM_AUD_CLK0_SEL] = imx_clk_mux_scu("acm_aud_clk0_sel", base_acm + 0x00000, 0, 5, aud_clk_sels, ARRAY_SIZE(aud_clk_sels), FUNCTION_NAME(PD_AUDIO));
		clks[IMX8QM_ACM_AUD_CLK0_CLK] = imx_clk_gate_scu("acm_aud_clk0_clk", "acm_aud_clk0_sel", SC_R_AUDIO_CLK_0, SC_PM_CLK_SLV_BUS, NULL, 0, 0);

		clks[IMX8QM_ACM_AUD_CLK1_SEL] = imx_clk_mux_scu("acm_aud_clk1_sel", base_acm + 0x10000, 0, 5, aud_clk_sels, ARRAY_SIZE(aud_clk_sels), FUNCTION_NAME(PD_AUDIO));
		clks[IMX8QM_ACM_AUD_CLK1_CLK] = imx_clk_gate_scu("acm_aud_clk1_clk", "acm_aud_clk1_sel", SC_R_AUDIO_CLK_1, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
		clks[IMX8QM_ACM_MCLKOUT0_SEL] = imx_clk_mux_scu("acm_mclkout0_sel", base_acm + 0x20000, 0, 3, mclk_out_sels, ARRAY_SIZE(mclk_out_sels), FUNCTION_NAME(PD_AUDIO));
		clks[IMX8QM_ACM_MCLKOUT1_SEL] = imx_clk_mux_scu("acm_mclkout1_sel", base_acm + 0x30000, 0, 3, mclk_out_sels, ARRAY_SIZE(mclk_out_sels), FUNCTION_NAME(PD_AUDIO));
		clks[IMX8QM_ACM_SAI0_MCLK_SEL] = imx_clk_mux_scu("acm_sai0_mclk_sel", base_acm + 0xE0000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_0));
		clks[IMX8QM_ACM_SAI1_MCLK_SEL] = imx_clk_mux_scu("acm_sai1_mclk_sel", base_acm + 0xF0000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_1));
		clks[IMX8QM_ACM_SAI2_MCLK_SEL] = imx_clk_mux_scu("acm_sai2_mclk_sel", base_acm + 0x100000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_2));
		clks[IMX8QM_ACM_SAI3_MCLK_SEL] = imx_clk_mux_scu("acm_sai3_mclk_sel", base_acm + 0x110000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_3));
		clks[IMX8QM_ACM_HDMI_RX_SAI0_MCLK_SEL] = imx_clk_mux_scu("acm_hdmi_rx_sai0_mclk_sel", base_acm + 0x120000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_4));
		clks[IMX8QM_ACM_HDMI_TX_SAI0_MCLK_SEL] = imx_clk_mux_scu("acm_hdmi_tx_sai0_mclk_sel", base_acm + 0x130000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_5));
		clks[IMX8QM_ACM_SAI6_MCLK_SEL] = imx_clk_mux_scu("acm_sai6_mclk_sel", base_acm + 0x140000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_6));
		clks[IMX8QM_ACM_SAI7_MCLK_SEL] = imx_clk_mux_scu("acm_sai7_mclk_sel", base_acm + 0x150000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels), FUNCTION_NAME(PD_AUD_SAI_7));
		clks[IMX8QM_ACM_SPDIF0_TX_CLK_SEL] = imx_clk_mux_scu("acm_spdif0_mclk_sel", base_acm + 0x1A0000, 0, 2, spdif_mclk_sels, ARRAY_SIZE(spdif_mclk_sels), FUNCTION_NAME(PD_AUD_SPDIF_0));
		clks[IMX8QM_ACM_SPDIF1_TX_CLK_SEL] = imx_clk_mux_scu("acm_spdif1_mclk_sel", base_acm + 0x1B0000, 0, 2, spdif_mclk_sels, ARRAY_SIZE(spdif_mclk_sels), FUNCTION_NAME(PD_AUD_SPDIF_1));
		clks[IMX8QM_ACM_MQS_TX_CLK_SEL] = imx_clk_mux_scu("acm_mqs_mclk_sel", base_acm + 0x1C0000, 0, 2, mqs_mclk_sels, ARRAY_SIZE(mqs_mclk_sels), FUNCTION_NAME(PD_AUD_MQS_0));
		clks[IMX8QM_ACM_ASRC0_MUX_CLK_SEL] = imx_clk_mux_scu("acm_asrc0_mclk_sel", base_acm + 0x40000, 0, 2, asrc_mux_clk_sels, ARRAY_SIZE(asrc_mux_clk_sels), FUNCTION_NAME(PD_AUD_ASRC_0));
		clks[IMX8QM_ACM_ASRC1_MUX_CLK_SEL] = imx_clk_mux_scu("acm_asrc1_mclk_sel", base_acm + 0x50000, 0, 2, asrc_mux_clk_sels, ARRAY_SIZE(asrc_mux_clk_sels), FUNCTION_NAME(PD_AUD_ASRC_1));
		clks[IMX8QM_ACM_ESAI0_MCLK_SEL] = imx_clk_mux_scu("acm_esai0_mclk_sel", base_acm + 0x60000, 0, 2, esai_mclk_sels, ARRAY_SIZE(esai_mclk_sels), FUNCTION_NAME(PD_AUD_ESAI_0));
		clks[IMX8QM_ACM_ESAI1_MCLK_SEL] = imx_clk_mux_scu("acm_esai1_mclk_sel", base_acm + 0x70000, 0, 2, esai_mclk_sels, ARRAY_SIZE(esai_mclk_sels), FUNCTION_NAME(PD_AUD_ESAI_1));
	} else
		printk("clk-imx8qm: missing acm node, skipping\n");

	clks[IMX8QM_AUD_AMIX_IPG] = imx_clk_gate2_scu("aud_amix_ipg_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_AMIX_LPCG), 0, FUNCTION_NAME(PD_AUD_AMIX));
	clks[IMX8QM_AUD_ESAI_0_IPG] = imx_clk_gate2_scu("aud_esai0_ipg_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_ESAI_0_LPCG), 0, FUNCTION_NAME(PD_AUD_ESAI_0));
	clks[IMX8QM_AUD_ESAI_1_IPG] = imx_clk_gate2_scu("aud_esai1_ipg_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_ESAI_1_LPCG), 0, FUNCTION_NAME(PD_AUD_ESAI_1));
	clks[IMX8QM_AUD_ESAI_0_EXTAL_IPG] = imx_clk_gate2_scu("aud_esai0_extal_ipg_clk", "acm_esai0_mclk_sel", LPCG_ADDR(AUD_ESAI_0_LPCG), 16, FUNCTION_NAME(PD_AUD_ESAI_0));
	clks[IMX8QM_AUD_ESAI_1_EXTAL_IPG] = imx_clk_gate2_scu("aud_esai1_extal_ipg_clk", "acm_esai1_mclk_sel", LPCG_ADDR(AUD_ESAI_1_LPCG), 16, FUNCTION_NAME(PD_AUD_ESAI_1));
	clks[IMX8QM_AUD_SAI_0_IPG_S] = imx_clk_gate2_scu("aud_sai0_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_SAI_0_LPCG), 8, FUNCTION_NAME(PD_AUD_SAI_0));
	clks[IMX8QM_AUD_SAI_0_IPG] = imx_clk_gate2_scu("aud_sai0_ipg_clk", "aud_sai0_ipg_s_clk", LPCG_ADDR(AUD_SAI_0_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_0));
	clks[IMX8QM_AUD_SAI_0_MCLK] = imx_clk_gate2_scu("aud_sai0_mclk_clk", "acm_sai0_mclk_sel", LPCG_ADDR(AUD_SAI_0_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_0));
	clks[IMX8QM_AUD_SAI_1_IPG_S] = imx_clk_gate2_scu("aud_sai1_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_SAI_1_LPCG), 8, FUNCTION_NAME(PD_AUD_SAI_1));
	clks[IMX8QM_AUD_SAI_1_IPG] = imx_clk_gate2_scu("aud_sai1_ipg_clk", "aud_sai1_ipg_s_clk", LPCG_ADDR(AUD_SAI_1_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_1));
	clks[IMX8QM_AUD_SAI_1_MCLK] = imx_clk_gate2_scu("aud_sai1_mclk_clk", "acm_sai1_mclk_sel", LPCG_ADDR(AUD_SAI_1_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_1));
	clks[IMX8QM_AUD_SAI_2_IPG_S] = imx_clk_gate2_scu("aud_sai2_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_SAI_2_LPCG), 8, FUNCTION_NAME(PD_AUD_SAI_2));
	clks[IMX8QM_AUD_SAI_2_IPG] = imx_clk_gate2_scu("aud_sai2_ipg_clk", "aud_sai2_ipg_s_clk", LPCG_ADDR(AUD_SAI_2_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_2));
	clks[IMX8QM_AUD_SAI_2_MCLK] = imx_clk_gate2_scu("aud_sai2_mclk_clk", "acm_sai2_mclk_sel", LPCG_ADDR(AUD_SAI_2_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_2));
	clks[IMX8QM_AUD_SAI_3_IPG_S] = imx_clk_gate2_scu("aud_sai3_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_SAI_3_LPCG), 8, FUNCTION_NAME(PD_AUD_SAI_3));
	clks[IMX8QM_AUD_SAI_3_IPG] = imx_clk_gate2_scu("aud_sai3_ipg_clk", "aud_sai3_ipg_s_clk", LPCG_ADDR(AUD_SAI_3_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_3));
	clks[IMX8QM_AUD_SAI_3_MCLK] = imx_clk_gate2_scu("aud_sai3_mclk_clk", "acm_sai3_mclk_sel", LPCG_ADDR(AUD_SAI_3_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_3));
	clks[IMX8QM_AUD_SAI_6_IPG_S] = imx_clk_gate2_scu("aud_sai6_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_SAI_6_LPCG), 8, FUNCTION_NAME(PD_AUD_SAI_6));
	clks[IMX8QM_AUD_SAI_6_IPG] = imx_clk_gate2_scu("aud_sai6_ipg_clk", "aud_sai6_ipg_s_clk", LPCG_ADDR(AUD_SAI_6_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_6));
	clks[IMX8QM_AUD_SAI_6_MCLK] = imx_clk_gate2_scu("aud_sai6_mclk_clk", "acm_sai6_mclk_sel", LPCG_ADDR(AUD_SAI_6_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_6));
	clks[IMX8QM_AUD_SAI_7_IPG_S] = imx_clk_gate2_scu("aud_sai7_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_SAI_7_LPCG), 8, FUNCTION_NAME(PD_AUD_SAI_7));
	clks[IMX8QM_AUD_SAI_7_IPG] = imx_clk_gate2_scu("aud_sai7_ipg_clk", "aud_sai7_ipg_s_clk", LPCG_ADDR(AUD_SAI_7_LPCG), 0, FUNCTION_NAME(PD_AUD_SAI_7));
	clks[IMX8QM_AUD_SAI_7_MCLK] = imx_clk_gate2_scu("aud_sai7_mclk_clk", "acm_sai7_mclk_sel", LPCG_ADDR(AUD_SAI_7_LPCG), 16, FUNCTION_NAME(PD_AUD_SAI_7));
	clks[IMX8QM_AUD_SAI_HDMIRX0_IPG_S] = imx_clk_gate2_scu("aud_sai_hdmirx0_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_HDMI_RX_SAI_0_LPCG), 8, NULL);
	clks[IMX8QM_AUD_SAI_HDMIRX0_IPG] = imx_clk_gate2_scu("aud_sai_hdmirx0_ipg_clk", "aud_sai_hdmirx0_ipg_s_clk", LPCG_ADDR(AUD_HDMI_RX_SAI_0_LPCG), 0, NULL);
	clks[IMX8QM_AUD_SAI_HDMIRX0_MCLK] = imx_clk_gate2_scu("aud_sai_hdmirx0_mclk_clk", "acm_hdmi_rx_sai0_mclk_sel", LPCG_ADDR(AUD_HDMI_RX_SAI_0_LPCG), 16, NULL);
	clks[IMX8QM_AUD_SAI_HDMITX0_IPG_S] = imx_clk_gate2_scu("aud_sai_hdmitx0_ipg_s_clk", "ipg_aud_clk_root", LPCG_ADDR(AUD_HDMI_TX_SAI_0_LPCG), 8, NULL);
	clks[IMX8QM_AUD_SAI_HDMITX0_IPG] = imx_clk_gate2_scu("aud_sai_hdmitx0_ipg_clk", "aud_sai_hdmitx0_ipg_s_clk", LPCG_ADDR(AUD_HDMI_TX_SAI_0_LPCG), 0, NULL);
	clks[IMX8QM_AUD_SAI_HDMITX0_MCLK] = imx_clk_gate2_scu("aud_sai_hdmitx0_mclk_clk", "acm_hdmi_tx_sai0_mclk_sel", LPCG_ADDR(AUD_HDMI_TX_SAI_0_LPCG), 16, NULL);
	clks[IMX8QM_AUD_MQS_IPG] = imx_clk_gate2_scu("aud_mqs_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_MQS_LPCG), 0, FUNCTION_NAME(PD_AUD_MQS_0));
	clks[IMX8QM_AUD_MQS_HMCLK] = imx_clk_gate2_scu("aud_mqs_hm_clk", "acm_mqs_mclk_sel", LPCG_ADDR(AUD_MQS_LPCG), 16, FUNCTION_NAME(PD_AUD_MQS_0));
	clks[IMX8QM_AUD_GPT5_IPG_S] = imx_clk_gate2_scu("aud_gpt5_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_GPT_5_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_5));
	clks[IMX8QM_AUD_GPT5_24M_CLK] = imx_clk_gate2_scu("aud_gpt5_24MHz", "xtal_24MHz", LPCG_ADDR(AUD_GPT_5_LPCG), 20, FUNCTION_NAME(PD_AUD_GPT_5));
	clks[IMX8QM_AUD_GPT6_IPG_S] = imx_clk_gate2_scu("aud_gpt6_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_GPT_6_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_6));
	clks[IMX8QM_AUD_GPT6_24M_CLK] = imx_clk_gate2_scu("aud_gpt6_24MHz", "xtal_24MHz", LPCG_ADDR(AUD_GPT_6_LPCG), 20, FUNCTION_NAME(PD_AUD_GPT_6));
	clks[IMX8QM_AUD_GPT7_IPG_S] = imx_clk_gate2_scu("aud_gpt7_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_GPT_7_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_7));
	clks[IMX8QM_AUD_GPT7_24M_CLK] = imx_clk_gate2_scu("aud_gpt7_24MHz", "xtal_24MHz", LPCG_ADDR(AUD_GPT_7_LPCG), 20, FUNCTION_NAME(PD_AUD_GPT_7));
	clks[IMX8QM_AUD_GPT8_IPG_S] = imx_clk_gate2_scu("aud_gpt8_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_GPT_8_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_8));
	clks[IMX8QM_AUD_GPT8_24M_CLK] = imx_clk_gate2_scu("aud_gpt8_24MHz", "xtal_24MHz", LPCG_ADDR(AUD_GPT_8_LPCG), 20, FUNCTION_NAME(PD_AUD_GPT_8));
	clks[IMX8QM_AUD_GPT9_IPG_S] = imx_clk_gate2_scu("aud_gpt9_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_GPT_9_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_9));
	clks[IMX8QM_AUD_GPT9_24M_CLK] = imx_clk_gate2_scu("aud_gpt9_24MHz", "xtal_24MHz", LPCG_ADDR(AUD_GPT_9_LPCG), 20, FUNCTION_NAME(PD_AUD_GPT_9));
	clks[IMX8QM_AUD_GPT10_IPG_S] = imx_clk_gate2_scu("aud_gpt10_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_GPT_10_LPCG), 0, FUNCTION_NAME(PD_AUD_GPT_9));
	clks[IMX8QM_AUD_GPT10_24M_CLK] = imx_clk_gate2_scu("aud_gpt10_24MHz", "xtal_24MHz", LPCG_ADDR(AUD_GPT_10_LPCG), 20, FUNCTION_NAME(PD_AUD_GPT_9));
	clks[IMX8QM_AUD_MCLKOUT0] = imx_clk_gate2_scu("aud_mclkout0", "acm_mclkout0_sel", LPCG_ADDR(AUD_MCLKOUT0_LPCG), 0, FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QM_AUD_MCLKOUT1] = imx_clk_gate2_scu("aud_mclkout1", "acm_mclkout1_sel", LPCG_ADDR(AUD_MCLKOUT1_LPCG), 0, FUNCTION_NAME(PD_AUDIO));
	clks[IMX8QM_AUD_SPDIF_0_IPG_S] = imx_clk_gate2_scu("spdif0_ipg_s", "ipg_aud_clk_root", LPCG_ADDR(AUD_SPDIF_0_LPCG), 0, FUNCTION_NAME(PD_AUD_SPDIF_0));
	clks[IMX8QM_AUD_SPDIF_0_GCLKW] = imx_clk_gate2_scu("spdif0_gclkw", "spdif0_ipg_s", LPCG_ADDR(AUD_SPDIF_0_LPCG), 16, FUNCTION_NAME(PD_AUD_SPDIF_0));
	clks[IMX8QM_AUD_SPDIF_0_TX_CLK] = imx_clk_gate2_scu("spdif0_tx_clk", "acm_spdif0_mclk_sel", LPCG_ADDR(AUD_SPDIF_0_LPCG), 20, FUNCTION_NAME(PD_AUD_SPDIF_0));
	clks[IMX8QM_AUD_SPDIF_1_IPG_S] = imx_clk_gate2_scu("spdif1_ipg_s", "ipg_aud_clk_root", LPCG_ADDR(AUD_SPDIF_1_LPCG), 0, FUNCTION_NAME(PD_AUD_SPDIF_1));
	clks[IMX8QM_AUD_SPDIF_1_GCLKW] = imx_clk_gate2_scu("spdif1_gclkw", "spdif1_ipg_s", LPCG_ADDR(AUD_SPDIF_1_LPCG), 16, FUNCTION_NAME(PD_AUD_SPDIF_1));
	clks[IMX8QM_AUD_SPDIF_1_TX_CLK] = imx_clk_gate2_scu("spdif1_tx_clk", "acm_spdif1_mclk_sel", LPCG_ADDR(AUD_SPDIF_1_LPCG), 20, FUNCTION_NAME(PD_AUD_SPDIF_1));
	clks[IMX8QM_AUD_ASRC_0_IPG] = imx_clk_gate2_scu("aud_asrc0_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_ASRC_0_LPCG), 0, FUNCTION_NAME(PD_AUD_ASRC_0));
	clks[IMX8QM_AUD_ASRC_0_MEM] = imx_clk_gate2_scu("aud_asrc0_mem", "ipg_aud_clk_root", LPCG_ADDR(AUD_ASRC_0_LPCG), 8, FUNCTION_NAME(PD_AUD_ASRC_0));
	clks[IMX8QM_AUD_ASRC_1_IPG] = imx_clk_gate2_scu("aud_asrc1_ipg", "ipg_aud_clk_root", LPCG_ADDR(AUD_ASRC_1_LPCG), 0, FUNCTION_NAME(PD_AUD_ASRC_1));
	clks[IMX8QM_AUD_ASRC_1_MEM] = imx_clk_gate2_scu("aud_asrc1_mem", "ipg_aud_clk_root", LPCG_ADDR(AUD_ASRC_1_LPCG), 8, FUNCTION_NAME(PD_AUD_ASRC_1));
	clks[IMX8QM_ACM_ASRC0_MUX_CLK_CLK] = imx_clk_gate_scu("aud_asrc0_mux_clk", "acm_asrc0_mclk_sel", SC_R_ASRC_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_ACM_ASRC1_MUX_CLK_CLK] = imx_clk_gate_scu("aud_asrc1_mux_clk", "acm_asrc1_mclk_sel", SC_R_ASRC_1, SC_PM_CLK_PER, NULL, 0, 0);

	/* MIPI CSI */
	clks[IMX8QM_CSI0_IPG_CLK_S] = imx_clk_gate2_scu("mipi_csi0_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_0_LPCG + 0x8), 16, FUNCTION_NAME(PD_MIPI_CSI0));
	clks[IMX8QM_CSI0_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_ipg", "mipi_csi0_ipg_s", LPCG_ADDR(MIPI_CSI_0_LPCG), 16, FUNCTION_NAME(PD_MIPI_CSI0));
	clks[IMX8QM_CSI0_APB_CLK] = imx_clk_gate2_scu("mipi_csi0_apb_clk", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_0_LPCG + 0x4), 16,  FUNCTION_NAME(PD_MIPI_CSI0));
	clks[IMX8QM_CSI0_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_i2c0_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_0_LPCG + 0x14), 16, FUNCTION_NAME(PD_MIPI_CSI0_I2C));
	clks[IMX8QM_CSI0_I2C0_CLK] = imx_clk_gate_scu("mipi_csi0_i2c0_clk", "mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x14), 0, 0);
	clks[IMX8QM_CSI0_PWM0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_pwm0_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_0_LPCG + 0x10), 16, FUNCTION_NAME(PD_MIPI_CSI0_PWM));
	clks[IMX8QM_CSI0_PWM0_CLK] = imx_clk_gate_scu("mipi_csi0_pwm0_clk", "mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x10), 0, 0);
	clks[IMX8QM_CSI0_CORE_CLK] = imx_clk_gate_scu("mipi_csi0_core_clk", "mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x18), 16, 0);
	clks[IMX8QM_CSI0_ESC_CLK] = imx_clk_gate_scu("mipi_csi0_esc_clk", "mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x1C), 16, 0);
	clks[IMX8QM_CSI1_IPG_CLK_S] = imx_clk_gate2_scu("mipi_csi1_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_1_LPCG + 0x8), 16, FUNCTION_NAME(PD_MIPI_CSI1));
	clks[IMX8QM_CSI1_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi_csi1_ipg", "mipi_csi1_ipg_s", LPCG_ADDR(MIPI_CSI_1_LPCG), 16, FUNCTION_NAME(PD_MIPI_CSI1));
	clks[IMX8QM_CSI1_APB_CLK] = imx_clk_gate2_scu("mipi_csi1_apb_clk", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_1_LPCG + 0x4), 16, FUNCTION_NAME(PD_MIPI_CSI1));
	clks[IMX8QM_CSI1_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi1_i2c0_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_1_LPCG + 0x14), 16, FUNCTION_NAME(PD_MIPI_CSI0_I2C));
	clks[IMX8QM_CSI1_I2C0_CLK] = imx_clk_gate_scu("mipi_csi1_i2c0_clk", "mipi_csi1_i2c0_div", SC_R_CSI_1_I2C_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_1_LPCG + 0x14), 0, 0);
	clks[IMX8QM_CSI1_PWM0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi1_pwm0_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_1_LPCG + 0x10), 16, FUNCTION_NAME(PD_MIPI_CSI0_PWM));
	clks[IMX8QM_CSI1_PWM0_CLK] = imx_clk_gate_scu("mipi_csi1_pwm0_clk", "mipi_csi1_pwm0_div", SC_R_CSI_1_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_1_LPCG + 0x10), 0, 0);
	clks[IMX8QM_CSI1_CORE_CLK] = imx_clk_gate_scu("mipi_csi1_core_clk", "mipi_csi1_core_div", SC_R_CSI_1, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_1_LPCG + 0x18), 16, 0);
	clks[IMX8QM_CSI1_ESC_CLK] = imx_clk_gate_scu("mipi_csi1_esc_clk", "mipi_csi1_esc_div", SC_R_CSI_1, SC_PM_CLK_MISC, LPCG_ADDR(MIPI_CSI_1_LPCG + 0x1C), 16, 0);

	/* RX-HDMI */
	clks[IMX8QM_HDMI_RX_GPIO_IPG_S_CLK] = imx_clk_gate2_scu("hdmi_rx_gpio_ipg_s_clk", "ipg_hdmi_rx_clk_root", LPCG_ADDR(RX_HDMI_LPCG), 0, FUNCTION_NAME(PD_HDMI_RX));
	clks[IMX8QM_HDMI_RX_PWM_IPG_S_CLK] = imx_clk_gate2_scu("hdmi_rx_pwm_ipg_s_clk", "ipg_hdmi_rx_clk_root", LPCG_ADDR(RX_HDMI_LPCG + 0x8), 0, FUNCTION_NAME(PD_HDMI_RX));
	clks[IMX8QM_HDMI_RX_PWM_IPG_CLK] = imx_clk_gate2_scu("hdmi_rx_pwm_ipg_clk", "hdmi_rx_pwm_ipg_s_clk", LPCG_ADDR(RX_HDMI_LPCG + 0x4), 0, FUNCTION_NAME(PD_HDMI_RX));
	clks[IMX8QM_HDMI_RX_I2C_IPG_S_CLK] = imx_clk_gate2_scu("hdmi_rx_i2c_ipg_s_clk", "ipg_hdmi_rx_clk_root", LPCG_ADDR(RX_HDMI_LPCG + 0x1C), 0, FUNCTION_NAME(PD_HDMI_RX_I2C));
	clks[IMX8QM_HDMI_RX_I2C_IPG_CLK] = imx_clk_gate2_scu("hdmi_rx_i2c_ipg_clk", "hdmi_rx_i2c_ipg_s_clk", LPCG_ADDR(RX_HDMI_LPCG + 0x18), 0, FUNCTION_NAME(PD_HDMI_RX_I2C));
	clks[IMX8QM_HDMI_RX_I2C_DIV_CLK] = imx_clk_gate2_scu("hdmi_rx_i2c0_div_clk", "hdmi_rx_i2c0_div", LPCG_ADDR(RX_HDMI_LPCG + 0x14), 0, FUNCTION_NAME(PD_HDMI_RX_I2C));
	clks[IMX8QM_HDMI_RX_I2C0_CLK] = imx_clk_gate_scu("hdmi_rx_i2c0_clk", "hdmi_rx_i2c0_div_clk", SC_R_HDMI_RX_I2C_0, SC_PM_CLK_MISC2, LPCG_ADDR(RX_HDMI_LPCG + 0x10), 0, 0);
	clks[IMX8QM_HDMI_RX_SPDIF_CLK] = imx_clk_gate_scu("hdmi_rx_spdif_clk", "hdmi_rx_spdif_div", SC_R_HDMI_RX, SC_PM_CLK_MISC0, NULL, 0, 0);
	clks[IMX8QM_HDMI_RX_HD_REF_CLK] = imx_clk_gate_scu("hdmi_rx_hd_ref_clk", "hdmi_rx_hd_ref_div", SC_R_HDMI_RX, SC_PM_CLK_MISC1, NULL, 0, 0);
	clks[IMX8QM_HDMI_RX_HD_CORE_CLK] = imx_clk_gate_scu("hdmi_rx_hd_core_clk", "hdmi_rx_hd_core_div", SC_R_HDMI_RX, SC_PM_CLK_MISC2, LPCG_ADDR(RX_HDMI_LPCG + 0x28), 0, 0);
	clks[IMX8QM_HDMI_RX_PXL_CLK] = imx_clk_gate_scu("hdmi_rx_pxl_clk", "hdmi_rx_pxl_div", SC_R_HDMI_RX, SC_PM_CLK_MISC3, LPCG_ADDR(RX_HDMI_LPCG + 0x2C), 0, 0);
	clks[IMX8QM_HDMI_RX_I2S_CLK] = imx_clk_gate_scu("hdmi_rx_i2s_clk", "hdmi_rx_i2s_div", SC_R_HDMI_RX, SC_PM_CLK_MISC4, NULL, 0, 0);
	clks[IMX8QM_HDMI_RX_PWM_CLK] = imx_clk_gate_scu("hdmi_rx_pwm_clk", "hdmi_rx_pwm_div", SC_R_HDMI_RX_PWM_0, SC_PM_CLK_MISC2, LPCG_ADDR(RX_HDMI_LPCG + 0xC), 0, 0);
	clks[IMX8QM_HDMI_RX_SINK_PCLK] = imx_clk_gate2_scu("hdmi_rx_sink_pclk", "ipg_hdmi_rx_clk_root", LPCG_ADDR(RX_HDMI_LPCG + 0x20), 0, FUNCTION_NAME(PD_HDMI_RX));
	clks[IMX8QM_HDMI_RX_SINK_SCLK] = imx_clk_gate2_scu("hdmi_rx_sink_sclk", "ipg_hdmi_rx_clk_root", LPCG_ADDR(RX_HDMI_LPCG + 0x24), 0, FUNCTION_NAME(PD_HDMI_RX));
	clks[IMX8QM_HDMI_RX_PXL_ENC_CLK] = imx_clk_gate2_scu("hdmi_rx_sink_enc_clk", "hdmi_rx_pxl_clk", LPCG_ADDR(RX_HDMI_LPCG + 0x30), 0, FUNCTION_NAME(PD_HDMI_RX));

	/* MIPI-DI */
	clks[IMX8QM_MIPI0_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi0_lis_ipg_clk", "mipi0_clk_root", LPCG_ADDR(MIPI_DSI_0_LPCG + 0x0), 0, FUNCTION_NAME(PD_MIPI_0_DSI));
	clks[IMX8QM_MIPI0_I2C0_DIV] = imx_clk_divider_scu("mipi0_i2c0_div", SC_R_MIPI_0_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QM_MIPI0_I2C0_CLK] = imx_clk_gate_scu("mipi0_i2c0_clk", "mipi0_i2c0_div", SC_R_MIPI_0_I2C_0, SC_PM_CLK_MISC2, LPCG_ADDR(MIPI_DSI_0_LPCG + 0x1c), 0, 0);
	clks[IMX8QM_MIPI0_I2C0_IPG_S_CLK] = imx_clk_gate2_scu("mipi0_i2c0_ipg_s", "mipi0_clk_root", LPCG_ADDR(MIPI_DSI_0_LPCG + 0x18), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C0));
	clks[IMX8QM_MIPI0_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi0_i2c0_ipg", "mipi0_i2c0_ipg_s", LPCG_ADDR(MIPI_DSI_0_LPCG + 0x14), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C0));
	clks[IMX8QM_MIPI0_I2C1_DIV] = imx_clk_divider_scu("mipi0_i2c1_div", SC_R_MIPI_0_I2C_1, SC_PM_CLK_MISC2);
	clks[IMX8QM_MIPI0_I2C1_CLK] = imx_clk_gate_scu("mipi0_i2c1_clk", "mipi0_i2c1_div", SC_R_MIPI_0_I2C_1, SC_PM_CLK_MISC2, LPCG_ADDR(MIPI_DSI_0_LPCG + 0x2C), 0, 0);
	clks[IMX8QM_MIPI0_I2C1_IPG_S_CLK] = imx_clk_gate2_scu("mipi0_i2c1_ipg_s", "mipi0_clk_root", LPCG_ADDR(MIPI_DSI_0_LPCG + 0x28), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C1));
	clks[IMX8QM_MIPI0_I2C1_IPG_CLK] = imx_clk_gate2_scu("mipi0_i2c1_ipg", "mipi0_i2c1_ipg_s", LPCG_ADDR(MIPI_DSI_0_LPCG + 0x24), 0, FUNCTION_NAME(PD_MIPI_0_DSI_I2C1));
	clks[IMX8QM_MIPI0_PWM0_CLK] = imx_clk_gate_scu("mipi0_pwm0_clk", "mipi0_pwm0_div", SC_R_MIPI_0_PWM_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_MIPI0_DSI_TX_ESC_CLK] = imx_clk_gate_scu("mipi0_dsi_tx_esc_clk", "mipi0_dsi_tx_esc_div", SC_R_MIPI_0, SC_PM_CLK_MST_BUS, NULL, 0, 0);
	clks[IMX8QM_MIPI0_DSI_RX_ESC_CLK] = imx_clk_gate_scu("mipi0_dsi_rx_esc_clk", "mipi0_dsi_rx_esc_div", SC_R_MIPI_0, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
	clks[IMX8QM_MIPI0_PXL_CLK] = imx_clk_gate_scu("mipi0_pxl_clk", "mipi0_pxl_div", SC_R_MIPI_0, SC_PM_CLK_PER, NULL, 0, 0);

	clks[IMX8QM_MIPI1_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi1_lis_ipg_clk", "mipi1_clk_root", LPCG_ADDR(MIPI_DSI_1_LPCG + 0x0), 0, FUNCTION_NAME(PD_MIPI_1_DSI));

	clks[IMX8QM_MIPI1_I2C0_DIV] = imx_clk_divider_scu("mipi1_i2c0_div", SC_R_MIPI_1_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QM_MIPI1_I2C0_CLK] = imx_clk_gate_scu("mipi1_i2c0_clk", "mipi1_i2c0_div", SC_R_MIPI_1_I2C_0, SC_PM_CLK_MISC2, LPCG_ADDR(MIPI_DSI_1_LPCG + 0x1c), 0, 0);
	clks[IMX8QM_MIPI1_I2C0_IPG_S_CLK] = imx_clk_gate2_scu("mipi1_i2c0_ipg_s", "mipi1_clk_root", LPCG_ADDR(MIPI_DSI_1_LPCG + 0x18), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C0));
	clks[IMX8QM_MIPI1_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi1_i2c0_ipg", "mipi1_i2c0_ipg_s", LPCG_ADDR(MIPI_DSI_1_LPCG + 0x14), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C0));
	clks[IMX8QM_MIPI1_I2C1_DIV] = imx_clk_divider_scu("mipi1_i2c1_div", SC_R_MIPI_1_I2C_1, SC_PM_CLK_MISC2);
	clks[IMX8QM_MIPI1_I2C1_CLK] = imx_clk_gate_scu("mipi1_i2c1_clk", "mipi1_i2c1_div", SC_R_MIPI_1_I2C_1, SC_PM_CLK_MISC2, LPCG_ADDR(MIPI_DSI_1_LPCG + 0x2C), 0, 0);
	clks[IMX8QM_MIPI1_I2C1_IPG_S_CLK] = imx_clk_gate2_scu("mipi1_i2c1_ipg_s", "mipi1_clk_root", LPCG_ADDR(MIPI_DSI_1_LPCG + 0x28), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C1));
	clks[IMX8QM_MIPI1_I2C1_IPG_CLK] = imx_clk_gate2_scu("mipi1_i2c1_ipg", "mipi1_i2c1_ipg_s", LPCG_ADDR(MIPI_DSI_1_LPCG + 0x24), 0, FUNCTION_NAME(PD_MIPI_1_DSI_I2C1));
	clks[IMX8QM_MIPI1_PWM0_CLK] = imx_clk_gate_scu("mipi1_pwm0_clk", "mipi1_pwm0_div", SC_R_MIPI_1_PWM_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_MIPI1_DSI_TX_ESC_CLK] = imx_clk_gate_scu("mipi1_dsi_tx_esc_clk", "mipi1_dsi_tx_esc_div", SC_R_MIPI_1, SC_PM_CLK_MST_BUS, NULL, 0, 0);
	clks[IMX8QM_MIPI1_DSI_RX_ESC_CLK] = imx_clk_gate_scu("mipi1_dsi_rx_esc_clk", "mipi1_dsi_rx_esc_div", SC_R_MIPI_1, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
	clks[IMX8QM_MIPI1_PXL_CLK] = imx_clk_gate_scu("mipi1_pxl_clk", "mipi1_pxl_div", SC_R_MIPI_1, SC_PM_CLK_PER, NULL, 0, 0);

	/* Display controller */
	/* DC0 */
	clks[IMX8QM_DC0_DISP0_CLK] = imx_clk_gate_scu("dc0_disp0_clk", "dc0_disp0_div", SC_R_DC_0, SC_PM_CLK_MISC0, LPCG_ADDR(DC_0_LPCG), 0, 0);
	clks[IMX8QM_DC0_DISP1_CLK] = imx_clk_gate_scu("dc0_disp1_clk", "dc0_disp1_div", SC_R_DC_0, SC_PM_CLK_MISC1, LPCG_ADDR(DC_0_LPCG), 4, 0);
	clks[IMX8QM_DC0_PRG0_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg0_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x20), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG0_APB_CLK] = imx_clk_gate2_scu("dc0_prg0_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x20), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG1_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg1_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x24), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG1_APB_CLK] = imx_clk_gate2_scu("dc0_prg1_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x24), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG2_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg2_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x28), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG2_APB_CLK] = imx_clk_gate2_scu("dc0_prg2_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x28), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG3_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg3_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x34), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG3_APB_CLK] = imx_clk_gate2_scu("dc0_prg3_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x34), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG4_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg4_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x38), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG4_APB_CLK] = imx_clk_gate2_scu("dc0_prg4_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x38), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG5_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg5_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x3c), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG5_APB_CLK] = imx_clk_gate2_scu("dc0_prg5_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x3c), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG6_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg6_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x40), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG6_APB_CLK] = imx_clk_gate2_scu("dc0_prg6_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x40), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG7_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg7_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x44), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG7_APB_CLK] = imx_clk_gate2_scu("dc0_prg7_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x44), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG8_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg8_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x48), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_PRG8_APB_CLK] = imx_clk_gate2_scu("dc0_prg8_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x48), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_DPR0_APB_CLK] = imx_clk_gate2_scu("dc0_dpr0_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x18), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_DPR0_B_CLK] = imx_clk_gate2_scu("dc0_dpr0_b_clk", "axi_ext_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x18), 20, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_DPR1_APB_CLK] = imx_clk_gate2_scu("dc0_dpr1_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x2c), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_DPR1_B_CLK] = imx_clk_gate2_scu("dc0_dpr1_b_clk", "axi_ext_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x2c), 20, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_RTRAM0_CLK] = imx_clk_gate2_scu("dc0_rtrm0_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x1C), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QM_DC0_RTRAM1_CLK] = imx_clk_gate2_scu("dc0_rtrm1_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_0_LPCG + 0x30), 0, FUNCTION_NAME(PD_DC_0));
	/* DC1 */
	clks[IMX8QM_DC1_DISP0_CLK] = imx_clk_gate_scu("dc1_disp0_clk", "dc1_disp0_div", SC_R_DC_1, SC_PM_CLK_MISC0, LPCG_ADDR(DC_1_LPCG), 0, 0);
	clks[IMX8QM_DC1_DISP1_CLK] = imx_clk_gate_scu("dc1_disp1_clk", "dc1_disp1_div", SC_R_DC_1, SC_PM_CLK_MISC1, LPCG_ADDR(DC_1_LPCG), 4, 0);
	clks[IMX8QM_DC1_PRG0_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg0_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x20), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG0_APB_CLK] = imx_clk_gate2_scu("dc1_prg0_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x20), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG1_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg1_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x24), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG1_APB_CLK] = imx_clk_gate2_scu("dc1_prg1_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x24), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG2_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg2_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x28), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG2_APB_CLK] = imx_clk_gate2_scu("dc1_prg2_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x28), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG3_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg3_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x34), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG3_APB_CLK] = imx_clk_gate2_scu("dc1_prg3_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x34), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG4_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg4_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x38), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG4_APB_CLK] = imx_clk_gate2_scu("dc1_prg4_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x38), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG5_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg5_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x3c), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG5_APB_CLK] = imx_clk_gate2_scu("dc1_prg5_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x3c), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG6_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg6_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x40), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG6_APB_CLK] = imx_clk_gate2_scu("dc1_prg6_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x40), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG7_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg7_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x44), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG7_APB_CLK] = imx_clk_gate2_scu("dc1_prg7_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x44), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG8_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg8_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x48), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG8_APB_CLK] = imx_clk_gate2_scu("dc1_prg8_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x48), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR0_APB_CLK] = imx_clk_gate2_scu("dc1_dpr0_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x18), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR0_B_CLK] = imx_clk_gate2_scu("dc1_dpr0_b_clk", "axi_ext_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x18), 20, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR1_APB_CLK] = imx_clk_gate2_scu("dc1_dpr1_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x2c), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR1_B_CLK] = imx_clk_gate2_scu("dc1_dpr1_b_clk", "axi_ext_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x2c), 20, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_RTRAM0_CLK] = imx_clk_gate2_scu("dc1_rtrm0_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x1C), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_RTRAM1_CLK] = imx_clk_gate2_scu("dc1_rtrm1_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x30), 0, FUNCTION_NAME(PD_DC_1));

	/* HDMI SS */
	clks[IMX8QM_HDMI_IPG_CLK] = imx_clk_gate_scu("ipg_hdmi_clk_root", "ipg_hdmi_div", SC_R_HDMI, SC_PM_CLK_MISC, NULL, 0, 0);
	clks[IMX8QM_HDMI_I2C0_CLK] = imx_clk_gate_scu("hdmi_i2c0_clk", "hdmi_i2c0_div", SC_R_HDMI_I2C_0, SC_PM_CLK_MISC2, LPCG_ADDR(DI_HDMI_LPCG), 0, 0);
	clks[IMX8QM_HDMI_PXL_MUX_CLK] = imx_clk_gate_scu("hdmi_pxl_mux_clk", "hdmi_pxl_mux_div", SC_R_HDMI, SC_PM_CLK_MISC0, NULL, 0, 0);
	clks[IMX8QM_HDMI_PXL_LINK_CLK] = imx_clk_gate_scu("hdmi_pxl_link_clk", "hdmi_pxl_link_div", SC_R_HDMI, SC_PM_CLK_MISC1, NULL, 0, 0);
	clks[IMX8QM_HDMI_HDP_CORE_CLK] = imx_clk_gate_scu("hdmi_hdp_core_clk", "hdmi_core_div", SC_R_HDMI, SC_PM_CLK_MISC2, NULL, 0, 0);
	clks[IMX8QM_HDMI_PXL_CLK] = imx_clk_gate_scu("hdmi_pxl_clk", "hdmi_pxl_div", SC_R_HDMI, SC_PM_CLK_MISC3, NULL, 0, 0);
	clks[IMX8QM_HDMI_I2S_CLK] = imx_clk_gate_scu("hdmi_i2s_clk", "hdmi_i2s_div", SC_R_HDMI, SC_PM_CLK_BYPASS, LPCG_ADDR(DI_HDMI_LPCG + 0xC), 0, 0);
	clks[IMX8QM_HDMI_I2C_IPG_CLK] = imx_clk_gate2_scu("hdmi_i2c_ipg_clk", "ipg_hdmi_clk_root", LPCG_ADDR(DI_HDMI_LPCG), 16, FUNCTION_NAME(PD_HDMI_I2C_0));
	clks[IMX8QM_HDMI_PWM_IPG_CLK] = imx_clk_gate2_scu("hdmi_pwm_ipg_clk", "ipg_hdmi_clk_root", LPCG_ADDR(DI_HDMI_LPCG + 0x8), 16, FUNCTION_NAME(PD_HDMI_PWM_0));
	clks[IMX8QM_HDMI_GPIO_IPG_CLK] = imx_clk_gate2_scu("hdmi_gpio_ipg_clk", "ipg_hdmi_clk_root", LPCG_ADDR(DI_HDMI_LPCG + 0x10), 16, FUNCTION_NAME(PD_HDMI_GPIO_0));
	clks[IMX8QM_HDMI_LIS_IPG_CLK] = imx_clk_gate2_scu("hdmi_lis_ipg_clk", "ipg_hdmi_clk_root", LPCG_ADDR(DI_HDMI_LPCG + 0x4), 16, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_MSI_HCLK] = imx_clk_gate2_scu("hdmi_msi_hclk_clk", "ipg_hdmi_clk_root", LPCG_ADDR(DI_HDMI_LPCG + 0x14), 0, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_PXL_LPCG_CLK] = imx_clk_gate2_scu("hdmi_pxl_lpcg_clk", "dummy", LPCG_ADDR(DI_HDMI_LPCG + 0x18), 0, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_PXL_EVEN_CLK] = imx_clk_gate2_scu("hdmi_pxl_even_clk", "hdmi_pxl_lpcg_clk", NULL, 0, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_HDP_CLK] = imx_clk_gate2_scu("hdmi_pxl_odd_clk", "hdmi_pxl_lpcg_clk", NULL, 0, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_PXL_DBL_CLK] = imx_clk_gate2_scu("hdmi_pxl_dbl_clk", "hdmi_pxl_lpcg_clk", NULL, 0, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_VIF_CLK] = imx_clk_gate2_scu("hdmi_vif_clk", "hdmi_pxl_mux_clk", LPCG_ADDR(DI_HDMI_LPCG + 0x1C), 0, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_APB_CLK] = imx_clk_gate2_scu("hdmi_apb_clk", "ipg_hdmi_clk_root", LPCG_ADDR(DI_HDMI_LPCG + 0x28), 16, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_APB_MUX_CSR_CLK] = imx_clk_gate2_scu("hdmi_apb_mux_csr_clk", "hdmi_apb_clk", LPCG_ADDR(DI_HDMI_LPCG + 0x20), 16, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_APB_MUX_CTRL_CLK] = imx_clk_gate2_scu("hdmi_apb_mux_ctrl_clk", "hdmi_apb_clk", LPCG_ADDR(DI_HDMI_LPCG + 0x24), 16, FUNCTION_NAME(PD_HDMI));
	clks[IMX8QM_HDMI_HDP_PHY_CLK] = imx_clk_gate2_scu("hdmi_pclk", "ipg_hdmi_clk_root", LPCG_ADDR(DI_HDMI_LPCG + 0x1C), 16, FUNCTION_NAME(PD_HDMI));

	/* lvds subsystem */
	clks[IMX8QM_LVDS0_PIXEL_CLK] = imx_clk_gate_scu("lvds0_pixel_clk", "lvds0_pixel_div", SC_R_LVDS_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_LVDS0_I2C0_CLK] = imx_clk_gate_scu("lvds0_i2c0_clk", "lvds0_i2c0_div", SC_R_LVDS_0_I2C_0, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_0_LPCG + 0x10), 0, 0);
	clks[IMX8QM_LVDS0_I2C1_CLK] = imx_clk_gate_scu("lvds0_i2c1_clk", "lvds0_i2c1_div", SC_R_LVDS_0_I2C_1, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_0_LPCG + 0x14), 0, 0);
	clks[IMX8QM_LVDS0_PWM0_CLK] = imx_clk_gate_scu("lvds0_pwm0_clk", "lvds0_pwm0_div", SC_R_LVDS_0_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_0_LPCG + 0x0C), 0, 0);
	clks[IMX8QM_LVDS0_PHY_CLK] = imx_clk_gate_scu("lvds0_phy_clk", "lvds0_phy_div", SC_R_LVDS_0, SC_PM_CLK_PHY, NULL, 0, 0);
	clks[IMX8QM_LVDS0_I2C0_IPG_CLK] = imx_clk_gate2_scu("lvds0_i2c0_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_0_LPCG + 0x10), 16, FUNCTION_NAME(PD_LVDS0_I2C0));
	clks[IMX8QM_LVDS0_I2C1_IPG_CLK] = imx_clk_gate2_scu("lvds0_i2c1_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_0_LPCG + 0x14), 16, FUNCTION_NAME(PD_LVDS0_I2C1));
	clks[IMX8QM_LVDS0_PWM0_IPG_CLK] = imx_clk_gate2_scu("lvds0_pwm0_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_0_LPCG + 0x0C), 16, FUNCTION_NAME(PD_LVDS0_PWM));
	clks[IMX8QM_LVDS0_GPIO_IPG_CLK] = imx_clk_gate2_scu("lvds0_gpio_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_0_LPCG + 0x08), 16, FUNCTION_NAME(PD_LVDS0_GPIO));
	clks[IMX8QM_LVDS0_LIS_IPG_CLK] = imx_clk_gate2_scu("lvds0_lis_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_0_LPCG + 0x0), 16, FUNCTION_NAME(PD_LVDS0));

	clks[IMX8QM_LVDS1_PIXEL_CLK] = imx_clk_gate_scu("lvds1_pixel_clk", "lvds1_pixel_div", SC_R_LVDS_1, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_LVDS1_I2C0_CLK] = imx_clk_gate_scu("lvds1_i2c0_clk", "lvds1_i2c0_div", SC_R_LVDS_1_I2C_0, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_1_LPCG + 0x10), 0, 0);
	clks[IMX8QM_LVDS1_I2C1_CLK] = imx_clk_gate_scu("lvds1_i2c1_clk", "lvds1_i2c1_div", SC_R_LVDS_1_I2C_1, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_1_LPCG + 0x14), 0, 0);
	clks[IMX8QM_LVDS1_PWM0_CLK] = imx_clk_gate_scu("lvds1_pwm0_clk", "lvds1_pwm0_div", SC_R_LVDS_1_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_1_LPCG + 0x0C), 0, 0);
	clks[IMX8QM_LVDS1_PHY_CLK] = imx_clk_gate_scu("lvds1_phy_clk", "lvds1_phy_div", SC_R_LVDS_1, SC_PM_CLK_PHY, NULL, 0, 0);
	clks[IMX8QM_LVDS1_I2C0_IPG_CLK] = imx_clk_gate2_scu("lvds1_i2c0_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x10), 16, FUNCTION_NAME(PD_LVDS1_I2C0));
	clks[IMX8QM_LVDS1_I2C1_IPG_CLK] = imx_clk_gate2_scu("lvds1_i2c1_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x14), 16, FUNCTION_NAME(PD_LVDS1_I2C1));
	clks[IMX8QM_LVDS1_PWM0_IPG_CLK] = imx_clk_gate2_scu("lvds1_pwm0_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x0C), 16, FUNCTION_NAME(PD_LVDS1_PWM));
	clks[IMX8QM_LVDS1_GPIO_IPG_CLK] = imx_clk_gate2_scu("lvds1_gpio_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x08), 16, FUNCTION_NAME(PD_LVDS1_GPIO));
	clks[IMX8QM_LVDS1_LIS_IPG_CLK] = imx_clk_gate2_scu("lvds1_lis_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x0), 16, FUNCTION_NAME(PD_LVDS1));

	/* vpu/zpu subsystem */
	clks[IMX8QM_VPU_DDR_CLK] = imx_clk_gate_scu("vpu_ddr_clk", "vpu_ddr_div", SC_R_VPU_PID0, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
	clks[IMX8QM_VPU_SYS_CLK] = imx_clk_gate_scu("vpu_sys_clk", "vpu_sys_div", SC_R_VPU_PID0, SC_PM_CLK_MST_BUS, NULL, 0, 0);
	clks[IMX8QM_VPU_XUVI_CLK] = imx_clk_gate_scu("vpu_xuvi_clk", "vpu_xuvi_div", SC_R_VPU_PID0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_VPU_UART_CLK] = imx_clk_gate_scu("vpu_uart_clk", "vpu_uart_div", SC_R_VPU_UART, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_VPU_CORE_CLK] = imx_clk_gate_scu("vpu_core_clk", "vpu_core_div", SC_R_VPUCORE, SC_PM_CLK_PER, NULL, 0, 0);

	/* gpu */
	clks[IMX8QM_GPU0_CORE_CLK] = imx_clk_gate_scu("gpu_core0_clk", "gpu_core0_div", SC_R_GPU_0_PID0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_GPU0_SHADER_CLK] = imx_clk_gate_scu("gpu_shader0_clk", "gpu_shader0_div", SC_R_GPU_0_PID0, SC_PM_CLK_MISC, NULL, 0, 0);
	clks[IMX8QM_GPU1_CORE_CLK] = imx_clk_gate_scu("gpu_core1_clk", "gpu_core1_div", SC_R_GPU_1_PID0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_GPU1_SHADER_CLK] = imx_clk_gate_scu("gpu_shader1_clk", "gpu_shader1_div", SC_R_GPU_1_PID0, SC_PM_CLK_MISC, NULL, 0, 0);

	/* Imaging SS */
	clks[IMX8QM_IMG_JPEG_ENC_IPG_CLK] = imx_clk_gate2_scu("img_jpeg_enc_ipg_clk", "ipg_img_clk_root", LPCG_ADDR(IMG_JPEG_ENC_LPCG), 16, FUNCTION_NAME(PD_IMAGING_JPEG_ENC));
	clks[IMX8QM_IMG_JPEG_ENC_CLK] = imx_clk_gate2_scu("img_jpeg_enc_clk", "img_jpeg_enc_ipg_clk", LPCG_ADDR(IMG_JPEG_ENC_LPCG), 0, FUNCTION_NAME(PD_IMAGING_JPEG_ENC));
	clks[IMX8QM_IMG_JPEG_DEC_IPG_CLK] = imx_clk_gate2_scu("img_jpeg_dec_ipg_clk", "ipg_img_clk_root", LPCG_ADDR(IMG_JPEG_DEC_LPCG), 16, FUNCTION_NAME(PD_IMAGING_JPEG_DEC));
	clks[IMX8QM_IMG_JPEG_DEC_CLK] = imx_clk_gate2_scu("img_jpeg_dec_clk", "img_jpeg_dec_ipg_clk", LPCG_ADDR(IMG_JPEG_DEC_LPCG), 0, FUNCTION_NAME(PD_IMAGING_JPEG_DEC));
	clks[IMX8QM_IMG_PXL_LINK_DC0_CLK] = imx_clk_gate2_scu("img_pxl_link_dc0_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PXL_LINK_DC0_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QM_IMG_PXL_LINK_DC1_CLK] = imx_clk_gate2_scu("img_pxl_link_dc1_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PXL_LINK_DC1_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QM_IMG_PXL_LINK_CSI0_CLK] = imx_clk_gate2_scu("img_pxl_link_csi0_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PXL_LINK_CSI0_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QM_IMG_PXL_LINK_CSI1_CLK] = imx_clk_gate2_scu("img_pxl_link_csi1_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PXL_LINK_CSI1_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QM_IMG_PXL_LINK_HDMI_IN_CLK] = imx_clk_gate2_scu("img_pxl_link_hdmi_in_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PXL_LINK_HDMI_LPCG), 0, FUNCTION_NAME(PD_IMAGING));
	clks[IMX8QM_IMG_PDMA_0_CLK] = imx_clk_gate2_scu("img_pdma0_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_0_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA0));
	clks[IMX8QM_IMG_PDMA_1_CLK] = imx_clk_gate2_scu("img_pdma1_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_1_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA1));
	clks[IMX8QM_IMG_PDMA_2_CLK] = imx_clk_gate2_scu("img_pdma2_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_2_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA2));
	clks[IMX8QM_IMG_PDMA_3_CLK] = imx_clk_gate2_scu("img_pdma3_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_3_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA3));
	clks[IMX8QM_IMG_PDMA_4_CLK] = imx_clk_gate2_scu("img_pdma4_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_4_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA4));
	clks[IMX8QM_IMG_PDMA_5_CLK] = imx_clk_gate2_scu("img_pdma5_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_5_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA5));
	clks[IMX8QM_IMG_PDMA_6_CLK] = imx_clk_gate2_scu("img_pdma6_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_6_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA6));
	clks[IMX8QM_IMG_PDMA_7_CLK] = imx_clk_gate2_scu("img_pdma7_clk", "pxl_img_clk_root", LPCG_ADDR(IMG_PDMA_7_LPCG), 0, FUNCTION_NAME(PD_IMAGING_PDMA7));

	/* HSIO */
	clks[IMX8QM_HSIO_PCIE_A_MSTR_AXI_CLK] = imx_clk_gate2_scu("hsio_pcieA_mstr_axi_clk", "axi_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X2_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_PCIE_A_SLV_AXI_CLK] = imx_clk_gate2_scu("hsio_pcieA_slv_axi_clk", "axi_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X2_LPCG), 20, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_PCIE_A_DBI_AXI_CLK] = imx_clk_gate2_scu("hsio_pcieA_dbi_axi_clk", "axi_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X2_LPCG), 24, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_PCIE_B_MSTR_AXI_CLK] = imx_clk_gate2_scu("hsio_pcieB_mstr_axi_clk", "axi_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X1_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_PCIE_B_SLV_AXI_CLK] = imx_clk_gate2_scu("hsio_pcieB_slv_axi_clk", "axi_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X1_LPCG), 20, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_PCIE_B_DBI_AXI_CLK] = imx_clk_gate2_scu("hsio_pcieB_dbi_axi_clk", "axi_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X1_LPCG), 24, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_PCIE_X1_PER_CLK] = imx_clk_gate2_scu("hsio_pcie_x1_per_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X1_CRR3_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_PCIE_X2_PER_CLK] = imx_clk_gate2_scu("hsio_pcie_x2_per_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_PCIE_X2_CRR2_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_SATA_PER_CLK] = imx_clk_gate2_scu("hsio_sata_per_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_SATA_CRR4_LPCG), 16, FUNCTION_NAME(PD_HSIO_SATA_0));
	clks[IMX8QM_HSIO_PHY_X1_PER_CLK] = imx_clk_gate2_scu("hsio_phy_x1_per_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_PHY_X1_CRR1_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_PHY_X2_PER_CLK] = imx_clk_gate2_scu("hsio_phy_x2_per_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_PHY_X2_CRR0_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_MISC_PER_CLK] = imx_clk_gate2_scu("hsio_misc_per_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_MISC_LPCG), 16, FUNCTION_NAME(PD_HSIO));
	clks[IMX8QM_HSIO_PHY_X1_APB_CLK] = imx_clk_gate2_scu("hsio_phy_x1_apb_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_PHY_X1_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_PHY_X2_APB_0_CLK] = imx_clk_gate2_scu("hsio_phy_x2_apb_0_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_PHY_X2_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_PHY_X2_APB_1_CLK] = imx_clk_gate2_scu("hsio_phy_x2_apb_1_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_PHY_X2_LPCG), 20, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_SATA_CLK] = imx_clk_gate2_scu("hsio_sata_clk", "axi_hsio_clk_root", LPCG_ADDR(HSIO_SATA_LPCG), 16, FUNCTION_NAME(PD_HSIO_SATA_0));
	clks[IMX8QM_HSIO_GPIO_CLK] = imx_clk_gate2_scu("hsio_gpio_clk", "per_hsio_clk_root", LPCG_ADDR(HSIO_GPIO_LPCG), 16, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_PHY_X1_PCLK] = imx_clk_gate2_scu("hsio_phy_x1_pclk", "dummy", LPCG_ADDR(HSIO_PHY_X1_LPCG), 0, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_PHY_X2_PCLK_0] = imx_clk_gate2_scu("hsio_phy_x2_pclk_0", "dummy", LPCG_ADDR(HSIO_PHY_X2_LPCG), 0, FUNCTION_NAME(PD_HSIO_PCIE_A));
	clks[IMX8QM_HSIO_PHY_X2_PCLK_1] = imx_clk_gate2_scu("hsio_phy_x2_pclk_1", "dummy", LPCG_ADDR(HSIO_PHY_X2_LPCG), 4, FUNCTION_NAME(PD_HSIO_PCIE_B));
	clks[IMX8QM_HSIO_SATA_EPCS_RX_CLK] = imx_clk_gate2_scu("hsio_sata_epcs_rx_clk", "dummy", LPCG_ADDR(HSIO_PHY_X1_LPCG), 8, FUNCTION_NAME(PD_HSIO_SATA_0));
	clks[IMX8QM_HSIO_SATA_EPCS_TX_CLK] = imx_clk_gate2_scu("hsio_sata_epcs_tx_clk", "dummy", LPCG_ADDR(HSIO_PHY_X1_LPCG), 4, FUNCTION_NAME(PD_HSIO_SATA_0));

	for (i = 0; i < ARRAY_SIZE(clks); i++)
		if (IS_ERR(clks[i]))
			pr_err("i.MX8QM clk %d: register failed with %ld\n",
				i, PTR_ERR(clks[i]));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(ccm_node, of_clk_src_onecell_get, &clk_data);

	return 0;
}

static const struct of_device_id imx8qm_match[] = {
	{ .compatible = "fsl,imx8qm-clk", },
	{ /* sentinel value */ }
};

static struct platform_driver imx8qm_clk_driver = {
	.driver = {
		.name = "imx8qm-clk",
		.of_match_table = imx8qm_match,
	},
	.probe = imx8qm_clk_probe,
};

static int __init imx8qm_clk_init(void)
{
	return platform_driver_register(&imx8qm_clk_driver);
}
core_initcall(imx8qm_clk_init);
