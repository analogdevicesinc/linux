// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Rockchip Electronics Co. Ltd.
 * Author: Elaine Zhang <zhangqing@rock-chips.com>
 */

#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <dt-bindings/clock/rockchip,rv1103b-cru.h>
#include "clk.h"

#define RV1103B_GRF_SOC_STATUS0		0x10
#define RV1103B_FRAC_MAX_PRATE		1200000000
#define PVTPLL_SRC_SEL_PVTPLL		(BIT(0) | BIT(16))

enum rv1103b_plls {
	dpll,
	gpll,
};

static struct rockchip_pll_rate_table rv1103b_pll_rates[] = {
	/* _mhz, _refdiv, _fbdiv, _postdiv1, _postdiv2, _dsmpd, _frac */
	RK3036_PLL_RATE(1200000000, 1, 100, 2, 1, 1, 0),
	RK3036_PLL_RATE(1188000000, 1, 99, 2, 1, 1, 0),
	RK3036_PLL_RATE(1000000000, 3, 250, 2, 1, 1, 0),
	{ /* sentinel */ },
};

#define RV1103B_DIV_ACLK_CORE_MASK	0x1f
#define RV1103B_DIV_ACLK_CORE_SHIFT	0
#define RV1103B_DIV_PCLK_DBG_MASK	0x1f
#define RV1103B_DIV_PCLK_DBG_SHIFT	8

#define RV1103B_CLKSEL0(_aclk_core)						\
{										\
	.reg = RV1103B_CORECLKSEL_CON(2),					\
	.val = HIWORD_UPDATE(_aclk_core - 1, RV1103B_DIV_ACLK_CORE_MASK,	\
			     RV1103B_DIV_ACLK_CORE_SHIFT),			\
}

#define RV1103B_CLKSEL1(_pclk_dbg)						\
{										\
	.reg = RV1103B_CORECLKSEL_CON(2),					\
	.val = HIWORD_UPDATE(_pclk_dbg - 1, RV1103B_DIV_PCLK_DBG_MASK,		\
			     RV1103B_DIV_PCLK_DBG_SHIFT),			\
}

#define RV1103B_CPUCLK_RATE(_prate, _aclk_core, _pclk_dbg)			\
{										\
	.prate = _prate,							\
	.divs = {								\
		RV1103B_CLKSEL0(_aclk_core),					\
		RV1103B_CLKSEL1(_pclk_dbg),					\
	},									\
}

static struct rockchip_cpuclk_rate_table rv1103b_cpuclk_rates[] __initdata = {
	RV1103B_CPUCLK_RATE(1608000000, 4, 10),
	RV1103B_CPUCLK_RATE(1512000000, 4, 10),
	RV1103B_CPUCLK_RATE(1416000000, 4, 10),
	RV1103B_CPUCLK_RATE(1296000000, 3, 10),
	RV1103B_CPUCLK_RATE(1200000000, 3, 10),
	RV1103B_CPUCLK_RATE(1188000000, 3, 8),
	RV1103B_CPUCLK_RATE(1104000000, 2, 8),
	RV1103B_CPUCLK_RATE(1008000000, 2, 8),
	RV1103B_CPUCLK_RATE(816000000, 2, 6),
	RV1103B_CPUCLK_RATE(600000000, 2, 4),
	RV1103B_CPUCLK_RATE(594000000, 2, 4),
	RV1103B_CPUCLK_RATE(408000000, 1, 3),
	RV1103B_CPUCLK_RATE(396000000, 1, 3),
};

PNAME(mux_pll_p)		= { "xin24m" };
PNAME(mux_200m_100m_p)		= { "clk_gpll_div6", "clk_gpll_div12" };
PNAME(mux_gpll_24m_p)		= { "gpll", "xin24m" };
PNAME(mux_480m_400m_300m_200m_p) = { "clk_gpll_div2p5", "clk_gpll_div3", "clk_gpll_div4", "clk_gpll_div6" };
PNAME(mux_480m_400m_300m_p)	= { "clk_gpll_div2p5", "clk_gpll_div3", "clk_gpll_div4" };
PNAME(mux_300m_200m_p)		= { "clk_gpll_div4", "clk_gpll_div6" };
PNAME(mux_600m_480m_400m_p)	= { "clk_gpll_div2", "clk_gpll_div2p5", "clk_gpll_div3" };
PNAME(mux_400m_300m_p)		= { "clk_gpll_div3", "clk_gpll_div4" };
PNAME(mux_100m_24m_p)		= { "clk_gpll_div12", "xin24m" };
PNAME(mux_200m_24m_p)		= { "clk_gpll_div6", "xin24m" };
PNAME(mux_200m_100m_50m_24m_p)	= { "clk_gpll_div6", "clk_gpll_div12", "clk_gpll_div24", "xin24m" };
PNAME(mux_300m_200m_100m_p)	= { "clk_gpll_div4", "clk_gpll_div6", "clk_gpll_div12" };
PNAME(sclk_uart0_src_p)		= { "clk_uart0_src", "clk_uart0_frac", "xin24m" };
PNAME(sclk_uart1_src_p)		= { "clk_uart1_src", "clk_uart1_frac", "xin24m" };
PNAME(sclk_uart2_src_p)		= { "clk_uart2_src", "clk_uart2_frac", "xin24m" };
PNAME(mclk_sai_src_p)		= { "clk_sai_src", "clk_sai_frac", "mclk_sai_from_io", "xin_osc0_half" };
PNAME(clk_freq_pwm0_src_p)	= { "sclk_sai_from_io", "mclk_sai_from_io", "clk_testout_out" };
PNAME(clk_counter_pwm0_src_p)	= { "sclk_sai_from_io", "mclk_sai_from_io", "clk_testout_out" };
PNAME(clk_mipi0_out2io_p)	= { "clk_ref_mipi0", "xin24m" };
PNAME(clk_mipi1_out2io_p)	= { "clk_ref_mipi1", "xin24m" };
PNAME(mclk_sai_out2io_p)	= { "mclk_sai_src", "xin_osc0_half" };
PNAME(aclk_npu_root_p)		= { "clk_npu_src", "clk_npu_pvtpll" };
PNAME(clk_core_vepu_p)		= { "clk_vepu_src", "clk_vepu_pvtpll" };
PNAME(lsclk_vi_root_p)		= { "clk_gpll_div6", "lsclk_vi_100m" };
PNAME(clk_core_isp_p)		= { "clk_isp_src", "clk_isp_pvtpll_src" };
PNAME(lsclk_pmu_root_p)		= { "xin24m", "clk_rc_osc_io" };
PNAME(xin_rc_div_p)		= { "xin24m", "clk_rc_osc_io" };
PNAME(clk_32k_p)		= { "xin_rc_div", "clk_32k_rtc", "clk_32k_io" };
PNAME(dbclk_pmu_gpio0_p)	= { "xin24m", "clk_32k" };
PNAME(sclk_sfc_2x_pmu1_p)	= { "clk_gpll_div12", "clk_rc_osc_io" };
PNAME(mux_armclk_p)		= { "armclk_gpll", "clk_core_pvtpll" };

static struct rockchip_pll_clock rv1103b_pll_clks[] __initdata = {
	[dpll] = PLL(pll_rk3328, PLL_DPLL, "dpll", mux_pll_p,
		     CLK_IS_CRITICAL, RV1103B_PLL_CON(16),
		     RV1103B_MODE_CON, 0, 10, 0, rv1103b_pll_rates),
	[gpll] = PLL(pll_rk3328, PLL_GPLL, "gpll", mux_pll_p,
		     CLK_IS_CRITICAL, RV1103B_PLL_CON(24),
		     RV1103B_MODE_CON, 0, 10, 0, rv1103b_pll_rates),
};

#define MFLAGS CLK_MUX_HIWORD_MASK
#define DFLAGS CLK_DIVIDER_HIWORD_MASK
#define GFLAGS (CLK_GATE_HIWORD_MASK | CLK_GATE_SET_TO_DISABLE)

static struct rockchip_clk_branch rv1103b_clk_uart0_fracmux __initdata =
	MUX(SCLK_UART0_SRC, "sclk_uart0_src", sclk_uart0_src_p, CLK_SET_RATE_PARENT,
			RV1103B_CLKSEL_CON(32), 8, 2, MFLAGS);

static struct rockchip_clk_branch rv1103b_clk_uart1_fracmux __initdata =
	MUX(SCLK_UART1_SRC, "sclk_uart1_src", sclk_uart1_src_p, CLK_SET_RATE_PARENT,
			RV1103B_CLKSEL_CON(32), 10, 2, MFLAGS);

static struct rockchip_clk_branch rv1103b_clk_uart2_fracmux __initdata =
	MUX(SCLK_UART2_SRC, "sclk_uart2_src", sclk_uart2_src_p, CLK_SET_RATE_PARENT,
			RV1103B_CLKSEL_CON(32), 12, 2, MFLAGS);

static struct rockchip_clk_branch rv1103b_rcdiv_pmu_fracmux __initdata =
	MUX(CLK_32K, "clk_32k", clk_32k_p, CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT,
			RK3568_PMU_CLKSEL_CON(0), 0, 2, MFLAGS);

static struct rockchip_clk_branch rv1103b_clk_branches[] __initdata = {

	/*       Clock Definition       */
	FACTOR(XIN_OSC0_HALF, "xin_osc0_half", "xin24m", 0, 1, 2),

	COMPOSITE_NOGATE(0, "armclk_gpll", mux_gpll_24m_p, CLK_IS_CRITICAL,
			RV1103B_CLKSEL_CON(37), 12, 1, MFLAGS, 13, 3, DFLAGS),

	/* pd_top */
	COMPOSITE_NOMUX(CLK_GPLL_DIV24, "clk_gpll_div24", "gpll", 0,
			RV1103B_CLKSEL_CON(0), 0, 5, DFLAGS,
			RV1103B_CLKGATE_CON(0), 0, GFLAGS),
	COMPOSITE_NOMUX(CLK_GPLL_DIV12, "clk_gpll_div12", "gpll", 0,
			RV1103B_CLKSEL_CON(0), 5, 5, DFLAGS,
			RV1103B_CLKGATE_CON(0), 1, GFLAGS),
	COMPOSITE_NOMUX(CLK_GPLL_DIV6, "clk_gpll_div6", "gpll", 0,
			RV1103B_CLKSEL_CON(1), 0, 5, DFLAGS,
			RV1103B_CLKGATE_CON(0), 3, GFLAGS),
	COMPOSITE_NOMUX(CLK_GPLL_DIV4, "clk_gpll_div4", "gpll", 0,
			RV1103B_CLKSEL_CON(1), 10, 5, DFLAGS,
			RV1103B_CLKGATE_CON(0), 5, GFLAGS),
	COMPOSITE_NOMUX(CLK_GPLL_DIV3, "clk_gpll_div3", "gpll", 0,
			RV1103B_CLKSEL_CON(2), 0, 5, DFLAGS,
			RV1103B_CLKGATE_CON(0), 7, GFLAGS),
	COMPOSITE_NOMUX_HALFDIV(CLK_GPLL_DIV2P5, "clk_gpll_div2p5", "gpll", 0,
			RV1103B_CLKSEL_CON(2), 5, 5, DFLAGS,
			RV1103B_CLKGATE_CON(0), 8, GFLAGS),
	COMPOSITE_NOMUX(CLK_GPLL_DIV2, "clk_gpll_div2", "gpll", 0,
			RV1103B_CLKSEL_CON(2), 10, 5, DFLAGS,
			RV1103B_CLKGATE_CON(0), 9, GFLAGS),
	COMPOSITE_NOMUX(CLK_UART0_SRC, "clk_uart0_src", "gpll", 0,
			RV1103B_CLKSEL_CON(5), 0, 5, DFLAGS,
			RV1103B_CLKGATE_CON(1), 0, GFLAGS),
	COMPOSITE_NOMUX(CLK_UART1_SRC, "clk_uart1_src", "gpll", 0,
			RV1103B_CLKSEL_CON(5), 5, 5, DFLAGS,
			RV1103B_CLKGATE_CON(1), 1, GFLAGS),
	COMPOSITE_NOMUX(CLK_UART2_SRC, "clk_uart2_src", "gpll", 0,
			RV1103B_CLKSEL_CON(5), 10, 5, DFLAGS,
			RV1103B_CLKGATE_CON(1), 2, GFLAGS),
	COMPOSITE_FRACMUX(CLK_UART0_FRAC, "clk_uart0_frac", "clk_uart0_src", 0,
			RV1103B_CLKSEL_CON(10), 0,
			RV1103B_CLKGATE_CON(1), 6, GFLAGS,
			&rv1103b_clk_uart0_fracmux),
	COMPOSITE_FRACMUX(CLK_UART1_FRAC, "clk_uart1_frac", "clk_uart1_src", 0,
			RV1103B_CLKSEL_CON(11), 0,
			RV1103B_CLKGATE_CON(1), 7, GFLAGS,
			&rv1103b_clk_uart1_fracmux),
	COMPOSITE_FRACMUX(CLK_UART2_FRAC, "clk_uart2_frac", "clk_uart2_src", 0,
			RV1103B_CLKSEL_CON(12), 0,
			RV1103B_CLKGATE_CON(1), 8, GFLAGS,
			&rv1103b_clk_uart2_fracmux),
	GATE(SCLK_UART0, "sclk_uart0", "sclk_uart0_src", 0,
			RV1103B_CLKGATE_CON(3), 3, GFLAGS),
	GATE(SCLK_UART1, "sclk_uart1", "sclk_uart1_src", 0,
			RV1103B_CLKGATE_CON(3), 4, GFLAGS),
	GATE(SCLK_UART2, "sclk_uart2", "sclk_uart2_src", 0,
			RV1103B_CLKGATE_CON(3), 8, GFLAGS),

	COMPOSITE_NOMUX(CLK_SAI_SRC, "clk_sai_src", "gpll", 0,
			RV1103B_CLKSEL_CON(20), 0, 5, DFLAGS,
			RV1103B_CLKGATE_CON(1), 12, GFLAGS),
	MUX(MCLK_SAI_SRC, "mclk_sai_src", mclk_sai_src_p, CLK_SET_RATE_PARENT,
			RV1103B_CLKSEL_CON(35), 10, 2, MFLAGS),
	GATE(MCLK_SAI, "mclk_sai", "mclk_sai_src", 0,
			RV1103B_CLKGATE_CON(5), 5, GFLAGS),

	COMPOSITE_NODIV(LSCLK_NPU_SRC, "lsclk_npu_src", mux_200m_100m_p, CLK_IS_CRITICAL,
			RV1103B_CLKSEL_CON(30), 0, 1, MFLAGS,
			RV1103B_CLKGATE_CON(2), 0, GFLAGS),
	COMPOSITE(CLK_NPU_SRC, "clk_npu_src", mux_gpll_24m_p, 0,
			RV1103B_CLKSEL_CON(37), 0, 1, MFLAGS, 1, 2, DFLAGS,
			RV1103B_CLKGATE_CON(5), 12, GFLAGS),
	COMPOSITE_NODIV(ACLK_VEPU_SRC, "aclk_vepu_src", mux_480m_400m_300m_200m_p, 0,
			RV1103B_CLKSEL_CON(30), 8, 2, MFLAGS,
			RV1103B_CLKGATE_CON(2), 4, GFLAGS),
	COMPOSITE(CLK_VEPU_SRC, "clk_vepu_src", mux_gpll_24m_p, 0,
			RV1103B_CLKSEL_CON(37), 4, 1, MFLAGS, 5, 2, DFLAGS,
			RV1103B_CLKGATE_CON(5), 13, GFLAGS),
	COMPOSITE_NODIV(ACLK_VI_SRC, "aclk_vi_src", mux_480m_400m_300m_p, CLK_IS_CRITICAL,
			RV1103B_CLKSEL_CON(30), 12, 2, MFLAGS,
			RV1103B_CLKGATE_CON(2), 8, GFLAGS),
	COMPOSITE(CLK_ISP_SRC, "clk_isp_src", mux_gpll_24m_p, 0,
			RV1103B_CLKSEL_CON(37), 8, 1, MFLAGS, 9, 2, DFLAGS,
			RV1103B_CLKGATE_CON(5), 14, GFLAGS),
	COMPOSITE_NODIV(DCLK_VICAP, "dclk_vicap", mux_300m_200m_p, 0,
			RV1103B_CLKSEL_CON(30), 14, 1, MFLAGS,
			RV1103B_CLKGATE_CON(2), 9, GFLAGS),
	COMPOSITE(CCLK_EMMC, "cclk_emmc", mux_gpll_24m_p, 0,
			RV1103B_CLKSEL_CON(31), 15, 1, MFLAGS, 0, 8, DFLAGS,
			RV1103B_CLKGATE_CON(2), 10, GFLAGS),
	COMPOSITE(CCLK_SDMMC0, "cclk_sdmmc0", mux_gpll_24m_p, 0,
			RV1103B_CLKSEL_CON(32), 15, 1, MFLAGS, 0, 8, DFLAGS,
			RV1103B_CLKGATE_CON(2), 11, GFLAGS),
	COMPOSITE(SCLK_SFC_2X, "sclk_sfc_2x", mux_gpll_24m_p, 0,
			RV1103B_CLKSEL_CON(33), 15, 1, MFLAGS, 0, 8, DFLAGS,
			RV1103B_CLKGATE_CON(2), 12, GFLAGS),
	COMPOSITE_NODIV(LSCLK_PERI_SRC, "lsclk_peri_src", mux_300m_200m_p, CLK_IS_CRITICAL,
			RV1103B_CLKSEL_CON(31), 9, 1, MFLAGS,
			RV1103B_CLKGATE_CON(3), 0, GFLAGS),
	COMPOSITE_NODIV(ACLK_PERI_SRC, "aclk_peri_src", mux_600m_480m_400m_p, CLK_IS_CRITICAL,
			RV1103B_CLKSEL_CON(31), 10, 2, MFLAGS,
			RV1103B_CLKGATE_CON(3), 1, GFLAGS),
	COMPOSITE_NODIV(HCLK_HPMCU, "hclk_hpmcu", mux_400m_300m_p, 0,
			RV1103B_CLKSEL_CON(31), 12, 1, MFLAGS,
			RV1103B_CLKGATE_CON(3), 2, GFLAGS),
		COMPOSITE_NODIV(CLK_I2C_PMU, "clk_i2c_pmu", mux_100m_24m_p, 0,
			RV1103B_CLKSEL_CON(34), 0, 1, MFLAGS,
			RV1103B_CLKGATE_CON(4), 0, GFLAGS),
	COMPOSITE_NODIV(CLK_I2C_PERI, "clk_i2c_peri", mux_200m_24m_p, 0,
			RV1103B_CLKSEL_CON(34), 1, 1, MFLAGS,
			RV1103B_CLKGATE_CON(4), 4, GFLAGS),
	COMPOSITE_NODIV(CLK_SPI0, "clk_spi0", mux_200m_100m_50m_24m_p, 0,
			RV1103B_CLKSEL_CON(34), 2, 2, MFLAGS,
			RV1103B_CLKGATE_CON(4), 5, GFLAGS),
	COMPOSITE_NODIV(CLK_PWM0_SRC, "clk_pwm0_src", mux_100m_24m_p, 0,
			RV1103B_CLKSEL_CON(34), 12, 1, MFLAGS,
			RV1103B_CLKGATE_CON(4), 10, GFLAGS),
	COMPOSITE_NODIV(CLK_PWM1, "clk_pwm1", mux_100m_24m_p, 0,
			RV1103B_CLKSEL_CON(34), 13, 1, MFLAGS,
			RV1103B_CLKGATE_CON(4), 11, GFLAGS),
	COMPOSITE_NODIV(CLK_PWM2, "clk_pwm2", mux_100m_24m_p, 0,
			RV1103B_CLKSEL_CON(34), 14, 1, MFLAGS,
			RV1103B_CLKGATE_CON(4), 12, GFLAGS),
	COMPOSITE_NODIV(DCLK_DECOM_SRC, "dclk_decom_src", mux_480m_400m_300m_p, 0,
			RV1103B_CLKSEL_CON(35), 0, 2, MFLAGS,
			RV1103B_CLKGATE_CON(5), 0, GFLAGS),
	COMPOSITE(CCLK_SDMMC1, "cclk_sdmmc1", mux_gpll_24m_p, 0,
			RV1103B_CLKSEL_CON(36), 15, 1, MFLAGS, 0, 8, DFLAGS,
			RV1103B_CLKGATE_CON(5), 1, GFLAGS),
	COMPOSITE_NODIV(CLK_CORE_CRYPTO, "clk_core_crypto", mux_300m_200m_100m_p, 0,
			RV1103B_CLKSEL_CON(35), 2, 2, MFLAGS,
			RV1103B_CLKGATE_CON(5), 2, GFLAGS),
	COMPOSITE_NODIV(CLK_PKA_CRYPTO, "clk_pka_crypto", mux_300m_200m_100m_p, 0,
			RV1103B_CLKSEL_CON(35), 4, 2, MFLAGS,
			RV1103B_CLKGATE_CON(5), 3, GFLAGS),
	COMPOSITE_NODIV(CLK_CORE_RGA, "clk_core_rga", mux_400m_300m_p, 0,
			RV1103B_CLKSEL_CON(35), 8, 1, MFLAGS,
			RV1103B_CLKGATE_CON(5), 4, GFLAGS),

	GATE(PCLK_TOP_ROOT, "pclk_top_root", "clk_gpll_div12", CLK_IS_CRITICAL,
			RV1103B_CLKGATE_CON(6), 0, GFLAGS),
	COMPOSITE_NOMUX(CLK_REF_MIPI0, "clk_ref_mipi0", "clk_gpll_div2", 0,
			RV1103B_CLKSEL_CON(40), 0, 5, DFLAGS,
			RV1103B_CLKGATE_CON(6), 3, GFLAGS),
	COMPOSITE_NODIV(CLK_MIPI0_OUT2IO, "clk_mipi0_out2io", clk_mipi0_out2io_p, CLK_SET_RATE_PARENT,
			RV1103B_CLKSEL_CON(40), 6, 1, MFLAGS,
			RV1103B_CLKGATE_CON(6), 4, GFLAGS),
	COMPOSITE_NOMUX(CLK_REF_MIPI1, "clk_ref_mipi1", "clk_gpll_div2", 0,
			RV1103B_CLKSEL_CON(40), 8, 5, DFLAGS,
			RV1103B_CLKGATE_CON(6), 5, GFLAGS),
	COMPOSITE_NODIV(CLK_MIPI1_OUT2IO, "clk_mipi1_out2io", clk_mipi1_out2io_p, CLK_SET_RATE_PARENT,
			RV1103B_CLKSEL_CON(40), 14, 1, MFLAGS,
			RV1103B_CLKGATE_CON(6), 6, GFLAGS),
	COMPOSITE(MCLK_SAI_OUT2IO, "mclk_sai_out2io", mclk_sai_out2io_p, 0,
			RV1103B_CLKSEL_CON(41), 7, 1, MFLAGS, 13, 3, DFLAGS,
			RV1103B_CLKGATE_CON(6), 9, GFLAGS),

	/* pd_vpu */
	COMPOSITE_NODIV(ACLK_NPU_ROOT, "aclk_npu_root", aclk_npu_root_p, CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			RV1103B_NPUCLKSEL_CON(0), 1, 1, MFLAGS,
			RV1103B_NPUCLKGATE_CON(0), 1, GFLAGS),
	GATE(HCLK_RKNN, "hclk_rknn", "lsclk_npu_src", 0,
			RV1103B_NPUCLKGATE_CON(0), 4, GFLAGS),
	GATE(ACLK_RKNN, "aclk_rknn", "aclk_npu_root", 0,
			RV1103B_NPUCLKGATE_CON(0), 5, GFLAGS),

	/* pd_vepu */
	COMPOSITE_NOMUX(LSCLK_VEPU_ROOT, "lsclk_vepu_root", "aclk_vepu_src", CLK_IS_CRITICAL,
			RV1103B_VEPUCLKSEL_CON(0), 2, 2, DFLAGS,
			RV1103B_VEPUCLKGATE_CON(0), 0, GFLAGS),
	GATE(HCLK_VEPU, "hclk_vepu", "lsclk_vepu_root", 0,
			RV1103B_VEPUCLKGATE_CON(0), 4, GFLAGS),
	GATE(ACLK_VEPU, "aclk_vepu", "aclk_vepu_src", 0,
			RV1103B_VEPUCLKGATE_CON(0), 5, GFLAGS),
	COMPOSITE_NODIV(CLK_CORE_VEPU, "clk_core_vepu", clk_core_vepu_p, 0,
			RV1103B_VEPUCLKSEL_CON(0), 1, 1, MFLAGS,
			RV1103B_VEPUCLKGATE_CON(0), 6, GFLAGS),
	GATE(PCLK_ACODEC, "pclk_acodec", "lsclk_vepu_root", 0,
			RV1103B_VEPUCLKGATE_CON(0), 13, GFLAGS),
	GATE(PCLK_USBPHY, "pclk_usbphy", "lsclk_vepu_root", 0,
			RV1103B_VEPUCLKGATE_CON(0), 14, GFLAGS),

	/* pd_vi */
	FACTOR(LSCLK_VI_100M, "lsclk_vi_100m", "clk_gpll_div6", 0, 1, 2),
	COMPOSITE_NODIV(LSCLK_VI_ROOT, "lsclk_vi_root", lsclk_vi_root_p, CLK_IS_CRITICAL,
			RV1103B_VICLKSEL_CON(0), 3, 1, MFLAGS,
			RV1103B_VICLKGATE_CON(0), 0, GFLAGS),
	GATE(HCLK_ISP, "hclk_isp", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(0), 4, GFLAGS),
	GATE(ACLK_ISP, "aclk_isp", "aclk_vi_src", 0,
			RV1103B_VICLKGATE_CON(0), 5, GFLAGS),
	COMPOSITE_NODIV(CLK_CORE_ISP, "clk_core_isp", clk_core_isp_p, 0,
			RV1103B_VICLKSEL_CON(0), 1, 1, MFLAGS,
			RV1103B_VICLKGATE_CON(0), 6, GFLAGS),
	GATE(ACLK_VICAP, "aclk_vicap", "aclk_vi_src", 0,
			RV1103B_VICLKGATE_CON(1), 2, GFLAGS),
	GATE(HCLK_VICAP, "hclk_vicap", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(1), 3, GFLAGS),
	GATE(ISP0CLK_VICAP, "isp0clk_vicap", "clk_core_isp", 0,
			RV1103B_VICLKGATE_CON(1), 8, GFLAGS),
	GATE(PCLK_CSI2HOST0, "pclk_csi2host0", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(1), 9, GFLAGS),
	GATE(PCLK_CSI2HOST1, "pclk_csi2host1", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(1), 11, GFLAGS),
	GATE(HCLK_EMMC, "hclk_emmc", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(1), 13, GFLAGS),
	GATE(HCLK_SFC, "hclk_sfc", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(1), 14, GFLAGS),
	GATE(HCLK_SFC_XIP, "hclk_sfc_xip", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(1), 15, GFLAGS),
	GATE(HCLK_SDMMC0, "hclk_sdmmc0", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(2), 0, GFLAGS),
	GATE(PCLK_CSIPHY, "pclk_csiphy", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(2), 2, GFLAGS),
	GATE(PCLK_GPIO1, "pclk_gpio1", "lsclk_vi_root", 0,
			RV1103B_VICLKGATE_CON(2), 3, GFLAGS),
	GATE(DBCLK_GPIO1, "dbclk_gpio1", "xin24m", 0,
			RV1103B_VICLKGATE_CON(2), 4, GFLAGS),

	/* pd_ddr */
	GATE(LSCLK_DDR_ROOT, "lsclk_ddr_root", "clk_gpll_div12", CLK_IS_CRITICAL,
			RV1103B_DDRCLKGATE_CON(0), 0, GFLAGS),
	GATE(CLK_TIMER_DDRMON, "clk_timer_ddrmon", "xin24m", 0,
			RV1103B_DDRCLKGATE_CON(0), 4, GFLAGS),
	FACTOR(0, "sclk_ddr", "dpll", 0, 1, 2),

	/* pd_pmu */
	COMPOSITE(LSCLK_PMU_ROOT, "lsclk_pmu_root", lsclk_pmu_root_p, CLK_IS_CRITICAL,
			RV1103B_PMUCLKSEL_CON(2), 4, 1, MFLAGS, 0, 2, DFLAGS,
			RV1103B_PMUCLKGATE_CON(0), 0, GFLAGS),
	GATE(PCLK_PMU, "pclk_pmu", "lsclk_pmu_root", CLK_IS_CRITICAL,
			RV1103B_PMUCLKGATE_CON(0), 2, GFLAGS),
	MUX(XIN_RC_SRC, "xin_rc_src", xin_rc_div_p, 0,
			RV1103B_PMUCLKSEL_CON(0), 2, 1, MFLAGS),
	COMPOSITE_FRACMUX(XIN_RC_DIV, "xin_rc_div", "xin_rc_src", CLK_SET_RATE_PARENT | CLK_IS_CRITICAL,
			RV1103B_PMUCLKSEL_CON(1), 0,
			RV1103B_PMUCLKGATE_CON(0), 3, GFLAGS,
			&rv1103b_rcdiv_pmu_fracmux),
	GATE(PCLK_PMU_GPIO0, "pclk_pmu_gpio0", "lsclk_pmu_root", 0,
			RV1103B_PMUCLKGATE_CON(0), 4, GFLAGS),
	COMPOSITE_NODIV(DBCLK_PMU_GPIO0, "dbclk_pmu_gpio0", dbclk_pmu_gpio0_p, 0,
			RK3568_PMU_CLKSEL_CON(0), 3, 1, MFLAGS,
			RV1103B_PMUCLKGATE_CON(0), 5, GFLAGS),
	GATE(PCLK_PWM0, "pclk_pwm0", "lsclk_pmu_root", 0,
			RV1103B_PMUCLKGATE_CON(2), 0, GFLAGS),
	GATE(CLK_PWM0, "clk_pwm0", "clk_pwm0_src", 0,
			RV1103B_PMUCLKGATE_CON(2), 1, GFLAGS),
	GATE(CLK_OSC_PWM0, "clk_osc_pwm0", "xin24m", 0,
			RV1103B_PMUCLKGATE_CON(2), 2, GFLAGS),
	GATE(CLK_RC_PWM0, "clk_rc_pwm0", "clk_32k", 0,
			RV1103B_PMUCLKGATE_CON(2), 3, GFLAGS),
	GATE(PCLK_I2C0, "pclk_i2c0", "lsclk_pmu_root", 0,
			RV1103B_PMUCLKGATE_CON(0), 12, GFLAGS),
	GATE(CLK_I2C0, "clk_i2c0", "clk_i2c_pmu", 0,
			RV1103B_PMUCLKGATE_CON(0), 13, GFLAGS),
	GATE(PCLK_UART0, "pclk_uart0", "lsclk_pmu_root", 0,
			RV1103B_PMUCLKGATE_CON(0), 14, GFLAGS),
	GATE(CLK_REFOUT, "clk_refout", "xin24m", 0,
			RV1103B_PMUCLKGATE_CON(1), 4, GFLAGS),
	GATE(CLK_PREROLL, "clk_preroll", "lsclk_pmu_root", 0,
			RV1103B_PMUCLKGATE_CON(1), 6, GFLAGS),
	GATE(CLK_PREROLL_32K, "clk_preroll_32k", "clk_32k", 0,
			RV1103B_PMUCLKGATE_CON(1), 7, GFLAGS),
	GATE(CLK_LPMCU_PMU, "clk_lpmcu_pmu", "lsclk_pmu_root", 0,
			RV1103B_PMUCLKGATE_CON(2), 12, GFLAGS),

	/* pd_pmu1 */
	GATE(PCLK_SPI2AHB, "pclk_spi2ahb", "lsclk_pmu_root", 0,
			RV1103B_PMU1CLKGATE_CON(0), 0, GFLAGS),
	GATE(HCLK_SPI2AHB, "hclk_spi2ahb", "lsclk_pmu_root", 0,
			RV1103B_PMU1CLKGATE_CON(0), 1, GFLAGS),
	GATE(PCLK_WDT_LPMCU, "pclk_wdt_lpmcu", "lsclk_pmu_root", 0,
			RV1103B_PMU1CLKGATE_CON(0), 9, GFLAGS),
	GATE(TCLK_WDT_LPMCU, "tclk_wdt_lpmcu", "xin24m", 0,
			RV1103B_PMU1CLKGATE_CON(0), 10, GFLAGS),
	GATE(HCLK_SFC_PMU1, "hclk_sfc_pmu1", "lsclk_pmu_root", 0,
			RV1103B_PMU1CLKGATE_CON(0), 12, GFLAGS),
	GATE(HCLK_SFC_XIP_PMU1, "hclk_sfc_xip_pmu1", "lsclk_pmu_root", 0,
			RV1103B_PMU1CLKGATE_CON(0), 13, GFLAGS),
	COMPOSITE_NODIV(SCLK_SFC_2X_PMU1, "sclk_sfc_2x_pmu1", sclk_sfc_2x_pmu1_p, 0,
			RV1103B_PMU1CLKSEL_CON(0), 8, 1, MFLAGS,
			RV1103B_PMU1CLKGATE_CON(0), 14, GFLAGS),
	GATE(CLK_LPMCU, "clk_lpmcu", "lsclk_pmu_root", 0,
			RV1103B_PMU1CLKGATE_CON(1), 0, GFLAGS),
	GATE(CLK_LPMCU_RTC, "clk_lpmcu_rtc", "xin24m", 0,
			RV1103B_PMU1CLKGATE_CON(1), 4, GFLAGS),
	GATE(PCLK_LPMCU_MAILBOX, "pclk_lpmcu_mailbox", "lsclk_pmu_root", 0,
			RV1103B_PMU1CLKGATE_CON(1), 8, GFLAGS),

	/* pd_peri */
	COMPOSITE_NOMUX(PCLK_PERI_ROOT, "pclk_peri_root", "lsclk_peri_src", CLK_IS_CRITICAL,
			RV1103B_PERICLKSEL_CON(0), 0, 2, DFLAGS,
			RV1103B_PERICLKGATE_CON(0), 0, GFLAGS),
	COMPOSITE_NOMUX(PCLK_RTC_ROOT, "pclk_rtc_root", "lsclk_peri_src", CLK_IS_CRITICAL,
			RV1103B_PERICLKSEL_CON(2), 12, 4, DFLAGS,
			RV1103B_PERICLKGATE_CON(0), 8, GFLAGS),
	GATE(CLK_TIMER_ROOT, "clk_timer_root", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(0), 1, GFLAGS),
	GATE(PCLK_TIMER, "pclk_timer", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(1), 0, GFLAGS),
	GATE(CLK_TIMER0, "clk_timer0", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 1, GFLAGS),
	GATE(CLK_TIMER1, "clk_timer1", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 2, GFLAGS),
	GATE(CLK_TIMER2, "clk_timer2", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 3, GFLAGS),
	GATE(CLK_TIMER3, "clk_timer3", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 4, GFLAGS),
	GATE(CLK_TIMER4, "clk_timer4", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 5, GFLAGS),
	GATE(CLK_TIMER5, "clk_timer5", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 6, GFLAGS),
	GATE(PCLK_STIMER, "pclk_stimer", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(1), 7, GFLAGS),
	GATE(CLK_STIMER0, "clk_stimer0", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 8, GFLAGS),
	GATE(CLK_STIMER1, "clk_stimer1", "clk_timer_root", 0,
			RV1103B_PERICLKGATE_CON(1), 9, GFLAGS),
	GATE(PCLK_WDT_NS, "pclk_wdt_ns", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(2), 0, GFLAGS),
	GATE(TCLK_WDT_NS, "tclk_wdt_ns", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(2), 1, GFLAGS),
	GATE(PCLK_WDT_S, "pclk_wdt_s", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(2), 2, GFLAGS),
	GATE(TCLK_WDT_S, "tclk_wdt_s", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(2), 3, GFLAGS),
	GATE(PCLK_WDT_HPMCU, "pclk_wdt_hpmcu", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(2), 4, GFLAGS),
	GATE(TCLK_WDT_HPMCU, "tclk_wdt_hpmcu", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(2), 5, GFLAGS),
	GATE(PCLK_I2C1, "pclk_i2c1", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(2), 6, GFLAGS),
	GATE(CLK_I2C1, "clk_i2c1", "clk_i2c_peri", 0,
			RV1103B_PERICLKGATE_CON(2), 7, GFLAGS),
	GATE(PCLK_I2C2, "pclk_i2c2", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(2), 8, GFLAGS),
	GATE(CLK_I2C2, "clk_i2c2", "clk_i2c_peri", 0,
			RV1103B_PERICLKGATE_CON(2), 9, GFLAGS),
	GATE(PCLK_I2C3, "pclk_i2c3", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(2), 10, GFLAGS),
	GATE(CLK_I2C3, "clk_i2c3", "clk_i2c_peri", 0,
			RV1103B_PERICLKGATE_CON(2), 11, GFLAGS),
	GATE(PCLK_I2C4, "pclk_i2c4", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(2), 12, GFLAGS),
	GATE(CLK_I2C4, "clk_i2c4", "clk_i2c_peri", 0,
			RV1103B_PERICLKGATE_CON(2), 13, GFLAGS),
	GATE(PCLK_SPI0, "pclk_spi0", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(3), 10, GFLAGS),
	GATE(PCLK_PWM1, "pclk_pwm1", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(4), 6, GFLAGS),
	GATE(CLK_OSC_PWM1, "clk_osc_pwm1", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(4), 8, GFLAGS),
	GATE(PCLK_PWM2, "pclk_pwm2", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(4), 12, GFLAGS),
	GATE(CLK_OSC_PWM2, "clk_osc_pwm2", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(4), 13, GFLAGS),
	GATE(PCLK_UART2, "pclk_uart2", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(3), 0, GFLAGS),
	GATE(PCLK_UART1, "pclk_uart1", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(3), 2, GFLAGS),
	GATE(ACLK_RKDMA, "aclk_rkdma", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(5), 8, GFLAGS),
	GATE(PCLK_TSADC, "pclk_tsadc", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(5), 9, GFLAGS),
	COMPOSITE_NOMUX(CLK_TSADC, "clk_tsadc", "xin24m", 0,
			RV1103B_PERICLKSEL_CON(0), 4, 5, DFLAGS,
			RV1103B_PERICLKGATE_CON(5), 10, GFLAGS),
	COMPOSITE_NOMUX(CLK_TSADC_TSEN, "clk_tsadc_tsen", "xin24m", 0,
			RV1103B_PERICLKSEL_CON(0), 10, 5, DFLAGS,
			RV1103B_PERICLKGATE_CON(5), 11, GFLAGS),
	GATE(PCLK_SARADC, "pclk_saradc", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(5), 12, GFLAGS),
	COMPOSITE_NOMUX(CLK_SARADC, "clk_saradc", "xin24m", 0,
			RV1103B_PERICLKSEL_CON(1), 0, 3, DFLAGS,
			RV1103B_PERICLKGATE_CON(5), 13, GFLAGS),
	GATE(PCLK_GPIO2, "pclk_gpio2", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(6), 3, GFLAGS),
	GATE(DBCLK_GPIO2, "dbclk_gpio2", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(6), 4, GFLAGS),
	GATE(ACLK_USBOTG, "aclk_usbotg", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(6), 9, GFLAGS),
	GATE(CLK_REF_USBOTG, "clk_ref_usbotg", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(6), 10, GFLAGS),
	GATE(HCLK_SDMMC1, "hclk_sdmmc1", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(7), 0, GFLAGS),
	GATE(HCLK_SAI, "hclk_sai", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(7), 1, GFLAGS),
	GATE(ACLK_CRYPTO, "aclk_crypto", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(8), 2, GFLAGS),
	GATE(HCLK_CRYPTO, "hclk_crypto", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(8), 3, GFLAGS),
	GATE(HCLK_RK_RNG_S, "hclk_rk_rng_s", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(8), 5, GFLAGS),
	GATE(HCLK_RK_RNG_NS, "hclk_rk_rng_ns", "hclk_rk_rng_s", 0,
			RV1103B_PERICLKGATE_CON(8), 4, GFLAGS),
	GATE(PCLK_OTPC_NS, "pclk_otpc_ns", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(8), 6, GFLAGS),
	GATE(CLK_OTPC_ROOT_NS, "clk_otpc_root_ns", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(8), 7, GFLAGS),
	GATE(CLK_SBPI_OTPC_NS, "clk_sbpi_otpc_ns", "clk_otpc_root_ns", 0,
			RV1103B_PERICLKGATE_CON(8), 8, GFLAGS),
	COMPOSITE_NOMUX(CLK_USER_OTPC_NS, "clk_user_otpc_ns", "clk_otpc_root_ns", 0,
			RV1103B_PERICLKSEL_CON(1), 4, 3, DFLAGS,
			RV1103B_PERICLKGATE_CON(8), 9, GFLAGS),
	GATE(PCLK_OTPC_S, "pclk_otpc_s", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(8), 10, GFLAGS),
	GATE(CLK_OTPC_ROOT_S, "clk_otpc_root_s", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(8), 11, GFLAGS),
	GATE(CLK_SBPI_OTPC_S, "clk_sbpi_otpc_s", "clk_otpc_root_s", 0,
			RV1103B_PERICLKGATE_CON(8), 12, GFLAGS),
	COMPOSITE_NOMUX(CLK_USER_OTPC_S, "clk_user_otpc_s", "clk_otpc_root_s", 0,
			RV1103B_PERICLKSEL_CON(1), 8, 3, DFLAGS,
			RV1103B_PERICLKGATE_CON(8), 13, GFLAGS),
	GATE(PCLK_OTP_MASK, "pclk_otp_mask", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(8), 15, GFLAGS),
	GATE(HCLK_RGA, "hclk_rga", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(9), 0, GFLAGS),
	GATE(ACLK_RGA, "aclk_rga", "aclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(9), 1, GFLAGS),
	GATE(ACLK_MAC, "aclk_mac", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(9), 3, GFLAGS),
	GATE(PCLK_MAC, "pclk_mac", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(9), 4, GFLAGS),
	GATE(CLK_MACPHY, "clk_macphy", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(9), 11, GFLAGS),
	GATE(ACLK_SPINLOCK, "aclk_spinlock", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(10), 0, GFLAGS),
	GATE(HCLK_CACHE, "hclk_cache", "hclk_hpmcu", 0,
			RV1103B_PERICLKGATE_CON(10), 1, GFLAGS),
	GATE(PCLK_HPMCU_MAILBOX, "pclk_hpmcu_mailbox", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(10), 2, GFLAGS),
	GATE(PCLK_HPMCU_INTMUX, "pclk_hpmcu_intmux", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(10), 3, GFLAGS),
	GATE(CLK_HPMCU, "clk_hpmcu", "hclk_hpmcu", 0,
			RV1103B_PERICLKGATE_CON(10), 4, GFLAGS),
	GATE(CLK_HPMCU_RTC, "clk_hpmcu_rtc", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(10), 8, GFLAGS),
	GATE(DCLK_DECOM, "dclk_decom", "dclk_decom_src", 0,
			RV1103B_PERICLKGATE_CON(11), 0, GFLAGS),
	GATE(ACLK_DECOM, "aclk_decom", "aclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(11), 1, GFLAGS),
	GATE(PCLK_DECOM, "pclk_decom", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(11), 2, GFLAGS),
	GATE(ACLK_SYS_SRAM, "aclk_sys_sram", "lsclk_peri_src", CLK_IS_CRITICAL,
			RV1103B_PERICLKGATE_CON(11), 3, GFLAGS),
	GATE(PCLK_DMA2DDR, "pclk_dma2ddr", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(11), 4, GFLAGS),
	GATE(ACLK_DMA2DDR, "aclk_dma2ddr", "aclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(11), 5, GFLAGS),
	GATE(PCLK_DCF, "pclk_dcf", "pclk_peri_root", 0,
			RV1103B_PERICLKGATE_CON(11), 6, GFLAGS),
	GATE(ACLK_DCF, "aclk_dcf", "lsclk_peri_src", 0,
			RV1103B_PERICLKGATE_CON(11), 7, GFLAGS),
	COMPOSITE_NOMUX(MCLK_ACODEC_TX, "mclk_acodec_tx", "mclk_sai_src", 0,
			RV1103B_PERICLKSEL_CON(2), 0, 3, DFLAGS,
			RV1103B_PERICLKGATE_CON(11), 9, GFLAGS),
	GATE(CLK_REF_USBPHY, "clk_ref_usbphy", "xin24m", 0,
			RV1103B_PERICLKGATE_CON(11), 12, GFLAGS),

	/* io */
	COMPOSITE_NODIV(CLK_FREQ_PWM0_SRC, "clk_freq_pwm0_src", clk_freq_pwm0_src_p, 0,
			RV1103B_CLKSEL_CON(35), 12, 2, MFLAGS,
			RV1103B_CLKGATE_CON(5), 6, GFLAGS),
	GATE(CLK_FREQ_PWM0, "clk_freq_pwm0", "clk_freq_pwm0_src", 0,
			RV1103B_PMUCLKGATE_CON(2), 4, GFLAGS),
	COMPOSITE_NODIV(CLK_COUNTER_PWM0_SRC, "clk_counter_pwm0_src", clk_counter_pwm0_src_p, 0,
			RV1103B_CLKSEL_CON(35), 14, 2, MFLAGS,
			RV1103B_CLKGATE_CON(5), 7, GFLAGS),
	GATE(CLK_COUNTER_PWM0, "clk_counter_pwm0", "clk_counter_pwm0_src", 0,
			RV1103B_PMUCLKGATE_CON(2), 5, GFLAGS),
	GATE(SCLK_SPI2AHB, "sclk_spi2ahb", "sclk_spi2ahb_io", 0,
			RV1103B_PMU1CLKGATE_CON(0), 2, GFLAGS),
	GATE(CLK_UTMI_USBOTG, "clk_utmi_usbotg", "clk_utmi_usbotg_io", 0,
			RV1103B_PERICRU_IP_CON, 14, GFLAGS),
};

static struct rockchip_clk_branch rv1103b_armclk __initdata =
	MUX(ARMCLK, "armclk", mux_armclk_p, CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
			RV1103B_CORECLKSEL_CON(0), 1, 1, MFLAGS);

static void __init rv1103b_clk_init(struct device_node *np)
{
	struct rockchip_clk_provider *ctx;
	unsigned long clk_nr;
	void __iomem *reg_base;

	clk_nr = rockchip_clk_find_max_clk_id(rv1103b_clk_branches,
					      ARRAY_SIZE(rv1103b_clk_branches)) + 1;
	reg_base = of_iomap(np, 0);
	if (!reg_base) {
		pr_err("%s: could not map cru region\n", __func__);
		return;
	}

	ctx = rockchip_clk_init(np, reg_base, clk_nr);
	if (IS_ERR(ctx)) {
		pr_err("%s: rockchip clk init failed\n", __func__);
		iounmap(reg_base);
		return;
	}

	rockchip_clk_register_plls(ctx, rv1103b_pll_clks,
				   ARRAY_SIZE(rv1103b_pll_clks),
				   RV1103B_GRF_SOC_STATUS0);

	rockchip_clk_register_branches(ctx, rv1103b_clk_branches,
				       ARRAY_SIZE(rv1103b_clk_branches));

	rockchip_clk_register_armclk_multi_pll(ctx, &rv1103b_armclk,
					       rv1103b_cpuclk_rates,
					       ARRAY_SIZE(rv1103b_cpuclk_rates));

	rockchip_register_restart_notifier(ctx, RV1103B_GLB_SRST_FST, NULL);

	rockchip_clk_of_add_provider(np, ctx);

	/* pvtpll src init */
	writel_relaxed(PVTPLL_SRC_SEL_PVTPLL, reg_base + RV1103B_CORECLKSEL_CON(0));
	writel_relaxed(PVTPLL_SRC_SEL_PVTPLL, reg_base + RV1103B_NPUCLKSEL_CON(0));
	writel_relaxed(PVTPLL_SRC_SEL_PVTPLL, reg_base + RV1103B_VICLKSEL_CON(0));
	writel_relaxed(PVTPLL_SRC_SEL_PVTPLL, reg_base + RV1103B_VEPUCLKSEL_CON(0));
}

CLK_OF_DECLARE(rv1103b_cru, "rockchip,rv1103b-cru", rv1103b_clk_init);
