// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *             https://www.samsung.com
 * Copyright (c) 2025  Axis Communications AB.
 *             https://www.axis.com
 *
 * Common Clock Framework support for ARTPEC-9 SoC.
 */

#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/axis,artpec9-clk.h>

#include "clk.h"
#include "clk-exynos-arm64.h"

/* NOTE: Must be equal to the last clock ID increased by one */
#define CMU_CMU_NR_CLK				(CLK_DOUT_CMU_VIO_AUDIO + 1)
#define CMU_BUS_NR_CLK				(CLK_MOUT_BUS_ACLK_USER + 1)
#define CMU_CORE_NR_CLK				(CLK_MOUT_CORE_ACLK_USER + 1)
#define CMU_CPUCL_NR_CLK			(CLK_GOUT_CPUCL_CSSYS_IPCLKPORT_PCLKDBG + 1)
#define CMU_FSYS0_NR_CLK			(CLK_GOUT_FSYS0_PWM_IPCLKPORT_I_PCLK_S0 + 1)
#define CMU_FSYS1_NR_CLK			(CLK_GOUT_FSYS1_XHB_USB_IPCLKPORT_CLK + 1)
#define CMU_IMEM_NR_CLK				(CLK_GOUT_IMEM_PCLK_TMU0_APBIF + 1)
#define CMU_PERI_NR_CLK				(CLK_GOUT_PERI_UART2_SCLK_UART + 1)

/* Register Offset definitions for CMU_CMU (0x12c00000) */
#define PLL_LOCKTIME_PLL_AUDIO					0x0000
#define PLL_LOCKTIME_PLL_SHARED0				0x0004
#define PLL_LOCKTIME_PLL_SHARED1				0x0008
#define PLL_CON0_PLL_AUDIO					0x0100
#define PLL_CON0_PLL_SHARED0					0x0120
#define PLL_CON0_PLL_SHARED1					0x0140
#define CLK_CON_MUX_CLKCMU_BUS					0x1000
#define CLK_CON_MUX_CLKCMU_CDC_CORE				0x1004
#define CLK_CON_MUX_CLKCMU_CORE_MAIN				0x1008
#define CLK_CON_MUX_CLKCMU_CPUCL_SWITCH				0x100c
#define CLK_CON_MUX_CLKCMU_VIO_AUDIO				0x1010
#define CLK_CON_MUX_CLKCMU_DLP_CORE				0x1014
#define CLK_CON_MUX_CLKCMU_FSYS0_BUS				0x1018
#define CLK_CON_MUX_CLKCMU_FSYS0_IP				0x101c
#define CLK_CON_MUX_CLKCMU_FSYS1_BUS				0x1020
#define CLK_CON_MUX_CLKCMU_FSYS1_SCAN0				0x1024
#define CLK_CON_MUX_CLKCMU_FSYS1_SCAN1				0x1028
#define CLK_CON_MUX_CLKCMU_GPU_2D				0x102c
#define CLK_CON_MUX_CLKCMU_GPU_3D				0x1030
#define CLK_CON_MUX_CLKCMU_IMEM_ACLK				0x1034
#define CLK_CON_MUX_CLKCMU_IMEM_CA5				0x1038
#define CLK_CON_MUX_CLKCMU_IMEM_JPEG				0x103c
#define CLK_CON_MUX_CLKCMU_IMEM_SSS				0x1040
#define CLK_CON_MUX_CLKCMU_IPA_CORE				0x1044
#define CLK_CON_MUX_CLKCMU_MIF_BUSP				0x1048
#define CLK_CON_MUX_CLKCMU_MIF_SWITCH				0x104c
#define CLK_CON_MUX_CLKCMU_PERI_DISP				0x1050
#define CLK_CON_MUX_CLKCMU_PERI_IP				0x1054
#define CLK_CON_MUX_CLKCMU_RSP_CORE				0x1058
#define CLK_CON_MUX_CLKCMU_TRFM					0x105c
#define CLK_CON_MUX_CLKCMU_VIO_CORE				0x1060
#define CLK_CON_MUX_CLKCMU_VIO_CORE_L				0x1064
#define CLK_CON_MUX_CLKCMU_VIP0					0x1068
#define CLK_CON_MUX_CLKCMU_VIP1					0x106c
#define CLK_CON_MUX_CLKCMU_VPP_CORE				0x1070
#define CLK_CON_DIV_CLKCMU_ADD					0x1800
#define CLK_CON_DIV_CLKCMU_BUS					0x1804
#define CLK_CON_DIV_CLKCMU_CDC_CORE				0x1808
#define CLK_CON_DIV_CLKCMU_CORE_MAIN				0x180c
#define CLK_CON_DIV_CLKCMU_CPUCL_SWITCH				0x1810
#define CLK_CON_DIV_CLKCMU_DLP_CORE				0x1814
#define CLK_CON_DIV_CLKCMU_FSYS0_BUS				0x1818
#define CLK_CON_DIV_CLKCMU_FSYS0_IP				0x181c
#define CLK_CON_DIV_CLKCMU_FSYS1_BUS				0x1820
#define CLK_CON_DIV_CLKCMU_FSYS1_SCAN0				0x1824
#define CLK_CON_DIV_CLKCMU_FSYS1_SCAN1				0x1828
#define CLK_CON_DIV_CLKCMU_GPU_2D				0x182c
#define CLK_CON_DIV_CLKCMU_GPU_3D				0x1830
#define CLK_CON_DIV_CLKCMU_IMEM_ACLK				0x1834
#define CLK_CON_DIV_CLKCMU_IMEM_CA5				0x1838
#define CLK_CON_DIV_CLKCMU_IMEM_JPEG				0x183c
#define CLK_CON_DIV_CLKCMU_IMEM_SSS				0x1840
#define CLK_CON_DIV_CLKCMU_IPA_CORE				0x1844
#define CLK_CON_DIV_CLKCMU_LCPU					0x1848
#define CLK_CON_DIV_CLKCMU_MIF_BUSP				0x184c
#define CLK_CON_DIV_CLKCMU_MIF_SWITCH				0x1850
#define CLK_CON_DIV_CLKCMU_PERI_DISP				0x1854
#define CLK_CON_DIV_CLKCMU_PERI_IP				0x1858
#define CLK_CON_DIV_CLKCMU_RSP_CORE				0x185c
#define CLK_CON_DIV_CLKCMU_TRFM					0x1860
#define CLK_CON_DIV_CLKCMU_VIO_AUDIO				0x1864
#define CLK_CON_DIV_CLKCMU_VIO_CORE				0x1868
#define CLK_CON_DIV_CLKCMU_VIO_CORE_L				0x186c
#define CLK_CON_DIV_CLKCMU_VIP0					0x1870
#define CLK_CON_DIV_CLKCMU_VIP1					0x1874
#define CLK_CON_DIV_CLKCMU_VPP_CORE				0x1878
#define CLK_CON_DIV_PLL_SHARED0_DIV2				0x187c
#define CLK_CON_DIV_PLL_SHARED0_DIV3				0x1880
#define CLK_CON_DIV_PLL_SHARED0_DIV4				0x1884
#define CLK_CON_DIV_PLL_SHARED1_DIV2				0x1888
#define CLK_CON_DIV_PLL_SHARED1_DIV3				0x188c
#define CLK_CON_DIV_PLL_SHARED1_DIV4				0x1890

static const unsigned long cmu_cmu_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_AUDIO,
	PLL_LOCKTIME_PLL_SHARED0,
	PLL_LOCKTIME_PLL_SHARED1,
	PLL_CON0_PLL_AUDIO,
	PLL_CON0_PLL_SHARED0,
	PLL_CON0_PLL_SHARED1,
	CLK_CON_MUX_CLKCMU_BUS,
	CLK_CON_MUX_CLKCMU_CDC_CORE,
	CLK_CON_MUX_CLKCMU_CORE_MAIN,
	CLK_CON_MUX_CLKCMU_CPUCL_SWITCH,
	CLK_CON_MUX_CLKCMU_DLP_CORE,
	CLK_CON_MUX_CLKCMU_FSYS0_BUS,
	CLK_CON_MUX_CLKCMU_FSYS0_IP,
	CLK_CON_MUX_CLKCMU_FSYS1_BUS,
	CLK_CON_MUX_CLKCMU_FSYS1_SCAN0,
	CLK_CON_MUX_CLKCMU_FSYS1_SCAN1,
	CLK_CON_MUX_CLKCMU_GPU_2D,
	CLK_CON_MUX_CLKCMU_GPU_3D,
	CLK_CON_MUX_CLKCMU_IMEM_ACLK,
	CLK_CON_MUX_CLKCMU_IMEM_CA5,
	CLK_CON_MUX_CLKCMU_IMEM_JPEG,
	CLK_CON_MUX_CLKCMU_IMEM_SSS,
	CLK_CON_MUX_CLKCMU_IPA_CORE,
	CLK_CON_MUX_CLKCMU_MIF_BUSP,
	CLK_CON_MUX_CLKCMU_MIF_SWITCH,
	CLK_CON_MUX_CLKCMU_PERI_DISP,
	CLK_CON_MUX_CLKCMU_PERI_IP,
	CLK_CON_MUX_CLKCMU_RSP_CORE,
	CLK_CON_MUX_CLKCMU_TRFM,
	CLK_CON_MUX_CLKCMU_VIO_CORE,
	CLK_CON_MUX_CLKCMU_VIO_CORE_L,
	CLK_CON_MUX_CLKCMU_VIP0,
	CLK_CON_MUX_CLKCMU_VIP1,
	CLK_CON_MUX_CLKCMU_VPP_CORE,
	CLK_CON_DIV_CLKCMU_ADD,
	CLK_CON_DIV_CLKCMU_BUS,
	CLK_CON_DIV_CLKCMU_CDC_CORE,
	CLK_CON_DIV_CLKCMU_CORE_MAIN,
	CLK_CON_DIV_CLKCMU_CPUCL_SWITCH,
	CLK_CON_DIV_CLKCMU_VIO_AUDIO,
	CLK_CON_DIV_CLKCMU_DLP_CORE,
	CLK_CON_DIV_CLKCMU_FSYS0_BUS,
	CLK_CON_DIV_CLKCMU_FSYS0_IP,
	CLK_CON_DIV_CLKCMU_FSYS1_BUS,
	CLK_CON_DIV_CLKCMU_FSYS1_SCAN0,
	CLK_CON_DIV_CLKCMU_FSYS1_SCAN1,
	CLK_CON_DIV_CLKCMU_GPU_2D,
	CLK_CON_DIV_CLKCMU_GPU_3D,
	CLK_CON_DIV_CLKCMU_IMEM_ACLK,
	CLK_CON_DIV_CLKCMU_IMEM_CA5,
	CLK_CON_DIV_CLKCMU_IMEM_JPEG,
	CLK_CON_DIV_CLKCMU_IMEM_SSS,
	CLK_CON_DIV_CLKCMU_IPA_CORE,
	CLK_CON_DIV_CLKCMU_LCPU,
	CLK_CON_DIV_CLKCMU_MIF_BUSP,
	CLK_CON_DIV_CLKCMU_MIF_SWITCH,
	CLK_CON_DIV_CLKCMU_PERI_DISP,
	CLK_CON_DIV_CLKCMU_PERI_IP,
	CLK_CON_DIV_CLKCMU_RSP_CORE,
	CLK_CON_DIV_CLKCMU_TRFM,
	CLK_CON_DIV_CLKCMU_VIO_AUDIO,
	CLK_CON_DIV_CLKCMU_VIO_CORE,
	CLK_CON_DIV_CLKCMU_VIO_CORE_L,
	CLK_CON_DIV_CLKCMU_VIP0,
	CLK_CON_DIV_CLKCMU_VIP1,
	CLK_CON_DIV_CLKCMU_VPP_CORE,
	CLK_CON_DIV_PLL_SHARED0_DIV2,
	CLK_CON_DIV_PLL_SHARED0_DIV3,
	CLK_CON_DIV_PLL_SHARED0_DIV4,
	CLK_CON_DIV_PLL_SHARED1_DIV2,
	CLK_CON_DIV_PLL_SHARED1_DIV3,
	CLK_CON_DIV_PLL_SHARED1_DIV4,
};

static const struct samsung_pll_rate_table artpec9_pll_audio_rates[] __initconst = {
	PLL_A9FRACO_RATE(25 * MHZ, 589824000U, 94, 1, 3, 6238440),
};

static const struct samsung_pll_clock cmu_cmu_pll_clks[] __initconst = {
	PLL(pll_a9fracm, CLK_FOUT_SHARED0_PLL, "fout_pll_shared0", "fin_pll",
	    PLL_LOCKTIME_PLL_SHARED0, PLL_CON0_PLL_SHARED0, NULL),
	PLL(pll_a9fracm, CLK_FOUT_SHARED1_PLL, "fout_pll_shared1", "fin_pll",
	    PLL_LOCKTIME_PLL_SHARED1, PLL_CON0_PLL_SHARED1, NULL),
	PLL(pll_a9fraco, CLK_FOUT_AUDIO_PLL, "fout_pll_audio", "fin_pll",
	    PLL_LOCKTIME_PLL_AUDIO, PLL_CON0_PLL_AUDIO, artpec9_pll_audio_rates),
};

PNAME(mout_clkcmu_bus_bus_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div2",
				 "dout_pll_shared1_div3", "dout_pll_shared1_div4" };
PNAME(mout_clkcmu_cdc_core_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div3",
				  "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_core_main_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div2",
				   "dout_pll_shared1_div3", "dout_pll_shared1_div4" };
PNAME(mout_clkcmu_cpucl_switch_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div2",
				      "dout_pll_shared0_div3", "dout_pll_shared1_div3" };
PNAME(mout_clkcmu_dlp_core_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				  "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_fsys0_bus_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				   "dout_pll_shared1_div4", "dout_pll_shared1_div2" };
PNAME(mout_clkcmu_fsys0_ip_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div3",
				  "dout_pll_shared1_div2", "dout_pll_shared0_div3" };
PNAME(mout_clkcmu_fsys1_bus_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				   "dout_pll_shared1_div2", "dout_pll_shared0_div4" };
PNAME(mout_clkcmu_fsys1_scan0_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div4" };
PNAME(mout_clkcmu_fsys1_scan1_p) = { "dout_pll_shared1_div3", "dout_pll_shared1_div4" };
PNAME(mout_clkcmu_gpu_3d_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				"dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_gpu_2d_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				"dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_imem_aclk_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				   "dout_pll_shared1_div4", "dout_pll_shared1_div2" };
PNAME(mout_clkcmu_imem_ca5_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div2",
				  "dout_pll_shared1_div3", "mout_clk_pll_shared1" };
PNAME(mout_clkcmu_imem_jpeg_p) = { "dout_pll_shared0_div2", "dout_pll_shared0_div3",
				   "dout_pll_shared1_div2", "dout_pll_shared1_div3" };
PNAME(mout_clkcmu_imem_sss_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div2" };
PNAME(mout_clkcmu_ipa_core_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				  "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_mif_switch_p) = { "fout_pll_shared1", "mout_clkcmu_pll_shared0",
				    "dout_pll_shared0_div2", "dout_pll_shared0_div3" };
PNAME(mout_clkcmu_mif_busp_p) = { "dout_pll_shared1_div3", "dout_pll_shared1_div4",
				  "dout_pll_shared0_div4", "dout_pll_shared0_div2" };
PNAME(mout_clkcmu_peri_disp_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				   "dout_pll_shared1_div4", "dout_pll_shared1_div2" };
PNAME(mout_clkcmu_peri_ip_p) = { "fout_pll_fsys1", "dout_pll_shared1_2",
				 "dout_pll_shared1_div4", "dout_pll_shared0_div2" };
PNAME(mout_clkcmu_rsp_core_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				  "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_trfm_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
			      "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_vio_core_l_p) = { "dout_pll_shared0_div2", "dout_pll_shared1_div3",
				    "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_vio_core_p) = { "fout_pll_fsys1", "dout_pll_shared0_div2",
				  "dout_pll_shared1_div3", "dout_pll_shared1_div2" };
PNAME(mout_clkcmu_vio_audio_p) = { "fout_pll_audio", "mout_clkcmu_pll_audio" };
PNAME(mout_clkcmu_vip0_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
			      "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_vip1_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
			      "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_vpp_core_p) = { "dout_pll_shared1_div3", "dout_pll_shared0_div2",
				  "dout_pll_shared1_div2", "mout_clk_pll_fsys1" };
PNAME(mout_clkcmu_pll_shared0_p) = { "fin_pll", "fout_pll_shared0" };
PNAME(mout_clkcmu_pll_shared1_p) = { "fin_pll", "fout_pll_shared1" };
PNAME(mout_clkcmu_pll_audio_p) = { "fin_pll", "fout_pll_audio" };

static const struct samsung_mux_clock cmu_cmu_mux_clks[] __initconst = {
	MUX(0, "mout_clkcmu_pll_shared0", mout_clkcmu_pll_shared0_p, PLL_CON0_PLL_SHARED0, 4, 1),
	MUX(0, "mout_clkcmu_pll_shared1", mout_clkcmu_pll_shared1_p, PLL_CON0_PLL_SHARED1, 4, 1),
	MUX(0, "mout_clkcmu_pll_audio", mout_clkcmu_pll_audio_p, PLL_CON0_PLL_AUDIO, 4, 1),
	MUX(0, "mout_clkcmu_bus_bus", mout_clkcmu_bus_bus_p, CLK_CON_MUX_CLKCMU_BUS, 0, 2),
	nMUX(0, "mout_clkcmu_cdc_core", mout_clkcmu_cdc_core_p, CLK_CON_MUX_CLKCMU_CDC_CORE, 0, 2),
	MUX(0, "mout_clkcmu_core_main", mout_clkcmu_core_main_p,
	    CLK_CON_MUX_CLKCMU_CORE_MAIN, 0, 2),
	MUX(0, "mout_clkcmu_cpucl_switch", mout_clkcmu_cpucl_switch_p,
	    CLK_CON_MUX_CLKCMU_CPUCL_SWITCH, 0, 2),
	nMUX(0, "mout_clkcmu_dlp_core", mout_clkcmu_dlp_core_p, CLK_CON_MUX_CLKCMU_DLP_CORE, 0, 2),
	MUX(0, "mout_clkcmu_fsys0_bus", mout_clkcmu_fsys0_bus_p,
	    CLK_CON_MUX_CLKCMU_FSYS0_BUS, 0, 2),
	MUX(0, "mout_clkcmu_fsys0_ip", mout_clkcmu_fsys0_ip_p, CLK_CON_MUX_CLKCMU_FSYS0_IP, 0, 2),
	MUX(0, "mout_clkcmu_fsys1_bus", mout_clkcmu_fsys1_bus_p,
	    CLK_CON_MUX_CLKCMU_FSYS1_BUS, 0, 2),
	MUX(0, "mout_clkcmu_fsys1_scan0", mout_clkcmu_fsys1_scan0_p,
	    CLK_CON_MUX_CLKCMU_FSYS1_SCAN0, 0, 1),
	MUX(0, "mout_clkcmu_fsys1_scan1", mout_clkcmu_fsys1_scan1_p,
	    CLK_CON_MUX_CLKCMU_FSYS1_SCAN1, 0, 1),
	MUX(0, "mout_clkcmu_gpu_2d", mout_clkcmu_gpu_2d_p, CLK_CON_MUX_CLKCMU_GPU_2D, 0, 2),
	MUX(0, "mout_clkcmu_gpu_3d", mout_clkcmu_gpu_3d_p, CLK_CON_MUX_CLKCMU_GPU_3D, 0, 2),
	MUX(0, "mout_clkcmu_imem_aclk", mout_clkcmu_imem_aclk_p,
	    CLK_CON_MUX_CLKCMU_IMEM_ACLK, 0, 2),
	MUX(0, "mout_clkcmu_imem_ca5", mout_clkcmu_imem_ca5_p, CLK_CON_MUX_CLKCMU_IMEM_CA5, 0, 2),
	MUX(0, "mout_clkcmu_imem_jpeg", mout_clkcmu_imem_jpeg_p,
	    CLK_CON_MUX_CLKCMU_IMEM_JPEG, 0, 2),
	MUX(0, "mout_clkcmu_imem_sss", mout_clkcmu_imem_sss_p, CLK_CON_MUX_CLKCMU_IMEM_SSS, 0, 1),
	MUX(0, "mout_clkcmu_ipa_core", mout_clkcmu_ipa_core_p, CLK_CON_MUX_CLKCMU_IPA_CORE, 0, 2),
	MUX(0, "mout_clkcmu_mif_busp", mout_clkcmu_mif_busp_p, CLK_CON_MUX_CLKCMU_MIF_BUSP, 0, 2),
	MUX(0, "mout_clkcmu_mif_switch", mout_clkcmu_mif_switch_p,
	    CLK_CON_MUX_CLKCMU_MIF_SWITCH, 0, 2),
	MUX(0, "mout_clkcmu_peri_disp", mout_clkcmu_peri_disp_p,
	    CLK_CON_MUX_CLKCMU_PERI_DISP, 0, 2),
	MUX(0, "mout_clkcmu_peri_ip", mout_clkcmu_peri_ip_p, CLK_CON_MUX_CLKCMU_PERI_IP, 0, 2),
	MUX(0, "mout_clkcmu_rsp_core", mout_clkcmu_rsp_core_p, CLK_CON_MUX_CLKCMU_RSP_CORE, 0, 2),
	MUX(0, "mout_clkcmu_trfm", mout_clkcmu_trfm_p, CLK_CON_MUX_CLKCMU_TRFM, 0, 2),
	MUX(0, "mout_clkcmu_vio_core", mout_clkcmu_vio_core_p, CLK_CON_MUX_CLKCMU_VIO_CORE, 0, 2),
	MUX(0, "mout_clkcmu_vio_core_l", mout_clkcmu_vio_core_l_p,
	    CLK_CON_MUX_CLKCMU_VIO_CORE_L, 0, 2),
	MUX(0, "mout_clkcmu_vio_audio", mout_clkcmu_vio_audio_p,
	    CLK_CON_MUX_CLKCMU_VIO_AUDIO, 0, 1),
	MUX(0, "mout_clkcmu_vip0", mout_clkcmu_vip0_p, CLK_CON_MUX_CLKCMU_VIP0, 0, 2),
	MUX(0, "mout_clkcmu_vip1", mout_clkcmu_vip1_p, CLK_CON_MUX_CLKCMU_VIP1, 0, 2),
	MUX(0, "mout_clkcmu_vpp_core", mout_clkcmu_vpp_core_p, CLK_CON_MUX_CLKCMU_VPP_CORE, 0, 2),
};

static const struct samsung_div_clock cmu_cmu_div_clks[] __initconst = {
	DIV(CLK_DOUT_CMU_ADD, "dout_clkcmu_add", "gate_clkcmu_add", CLK_CON_DIV_CLKCMU_ADD, 0, 8),
	DIV(CLK_DOUT_CMU_BUS, "dout_clkcmu_bus",
	    "gate_clkcmu_bus_bus", CLK_CON_DIV_CLKCMU_BUS, 0, 4),
	DIV_F(CLK_DOUT_CMU_CDC_CORE, "dout_clkcmu_cdc_core",
	      "gate_clkcmu_cdc_core", CLK_CON_DIV_CLKCMU_CDC_CORE, 0, 4, CLK_SET_RATE_PARENT, 0),
	DIV(CLK_DOUT_CMU_CORE_MAIN, "dout_clkcmu_core_main",
	    "gate_clkcmu_core_main", CLK_CON_DIV_CLKCMU_CORE_MAIN, 0, 4),
	DIV(CLK_DOUT_CMU_CPUCL_SWITCH, "dout_clkcmu_cpucl_switch",
	    "gate_clkcmu_cpucl_switch", CLK_CON_DIV_CLKCMU_CPUCL_SWITCH, 0, 3),
	DIV_F(CLK_DOUT_CMU_DLP_CORE, "dout_clkcmu_dlp_core",
	      "gate_clkcmu_dlp_core", CLK_CON_DIV_CLKCMU_DLP_CORE, 0, 4, CLK_SET_RATE_PARENT, 0),
	DIV(CLK_DOUT_CMU_FSYS0_BUS, "dout_clkcmu_fsys0_bus",
	    "gate_clkcmu_fsys0_bus", CLK_CON_DIV_CLKCMU_FSYS0_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS0_IP, "dout_clkcmu_fsys0_ip",
	    "gate_clkcmu_fsys0_ip", CLK_CON_DIV_CLKCMU_FSYS0_IP, 0, 9),
	DIV(CLK_DOUT_CMU_FSYS1_BUS, "dout_clkcmu_fsys1_bus",
	    "gate_clkcmu_fsys1_bus", CLK_CON_DIV_CLKCMU_FSYS1_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS1_SCAN0, "dout_clkcmu_fsys1_scan0",
	    "gate_clkcmu_fsys1_scan0", CLK_CON_DIV_CLKCMU_FSYS1_SCAN0, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS1_SCAN1, "dout_clkcmu_fsys1_scan1",
	    "gate_clkcmu_fsys1_scan1", CLK_CON_DIV_CLKCMU_FSYS1_SCAN1, 0, 4),
	DIV(CLK_DOUT_CMU_GPU_2D, "dout_clkcmu_gpu_2d",
	    "gate_clkcmu_gpu_2d", CLK_CON_DIV_CLKCMU_GPU_2D, 0, 4),
	DIV(CLK_DOUT_CMU_GPU_3D, "dout_clkcmu_gpu_3d",
	    "gate_clkcmu_gpu_3d", CLK_CON_DIV_CLKCMU_GPU_3D, 0, 4),
	DIV(CLK_DOUT_CMU_IMEM_ACLK, "dout_clkcmu_imem_aclk",
	    "gate_clkcmu_imem_aclk", CLK_CON_DIV_CLKCMU_IMEM_ACLK, 0, 4),
	DIV(CLK_DOUT_CMU_IMEM_CA5, "dout_clkcmu_imem_ca5",
	    "gate_clkcmu_imem_ca5", CLK_CON_DIV_CLKCMU_IMEM_CA5, 0, 4),
	DIV(CLK_DOUT_CMU_IMEM_JPEG, "dout_clkcmu_imem_jpeg",
	    "gate_clkcmu_imem_jpeg", CLK_CON_DIV_CLKCMU_IMEM_JPEG, 0, 4),
	DIV(CLK_DOUT_CMU_IMEM_SSS, "dout_clkcmu_imem_sss",
	    "gate_clkcmu_imem_sss", CLK_CON_DIV_CLKCMU_IMEM_SSS, 0, 4),
	DIV(CLK_DOUT_CMU_IPA_CORE, "dout_clkcmu_ipa_core",
	    "gate_clkcmu_ipa_core", CLK_CON_DIV_CLKCMU_IPA_CORE, 0, 4),
	DIV(CLK_DOUT_CMU_LCPU, "dout_clkcmu_lcpu",
	    "gate_clkcmu_lcpu", CLK_CON_DIV_CLKCMU_LCPU, 0, 4),
	DIV(CLK_DOUT_CMU_MIF_BUSP, "dout_clkcmu_mif_busp",
	    "gate_clkcmu_mif_busp", CLK_CON_DIV_CLKCMU_MIF_BUSP, 0, 3),
	DIV(CLK_DOUT_CMU_MIF_SWITCH, "dout_clkcmu_mif_switch",
	    "gate_clkcmu_mif_switch", CLK_CON_DIV_CLKCMU_MIF_SWITCH, 0, 4),
	DIV(CLK_DOUT_CMU_PERI_DISP, "dout_clkcmu_peri_disp",
	    "gate_clkcmu_peri_disp", CLK_CON_DIV_CLKCMU_PERI_DISP, 0, 4),
	DIV(CLK_DOUT_CMU_PERI_IP, "dout_clkcmu_peri_ip",
	    "gate_clkcmu_peri_ip", CLK_CON_DIV_CLKCMU_PERI_IP, 0, 4),
	DIV(CLK_DOUT_CMU_RSP_CORE, "dout_clkcmu_rsp_core",
	    "gate_clkcmu_rsp_core", CLK_CON_DIV_CLKCMU_RSP_CORE, 0, 4),
	DIV(CLK_DOUT_CMU_TRFM, "dout_clkcmu_trfm",
	    "gate_clkcmu_trfm", CLK_CON_DIV_CLKCMU_TRFM, 0, 4),
	DIV(CLK_DOUT_CMU_VIO_CORE, "dout_clkcmu_vio_core",
	    "gate_clkcmu_vio_core", CLK_CON_DIV_CLKCMU_VIO_CORE, 0, 4),
	DIV(CLK_DOUT_CMU_VIO_CORE_L, "dout_clkcmu_vio_core_l",
	    "gate_clkcmu_vio_core_l", CLK_CON_DIV_CLKCMU_VIO_CORE_L, 0, 4),
	DIV(CLK_DOUT_CMU_VIO_AUDIO, "dout_clkcmu_vio_audio",
	    "gate_clkcmu_vio_audio", CLK_CON_DIV_CLKCMU_VIO_AUDIO, 0, 4),
	DIV(CLK_DOUT_CMU_VIP0, "dout_clkcmu_vip0",
	    "gate_clkcmu_vip0", CLK_CON_DIV_CLKCMU_VIP0, 0, 4),
	DIV(CLK_DOUT_CMU_VIP1, "dout_clkcmu_vip1",
	    "gate_clkcmu_vip1", CLK_CON_DIV_CLKCMU_VIP1, 0, 4),
	DIV(CLK_DOUT_CMU_VPP_CORE, "dout_clkcmu_vpp_core",
	    "gate_clkcmu_vpp_core", CLK_CON_DIV_CLKCMU_VPP_CORE, 0, 4),
	DIV(CLK_DOUT_SHARED0_DIV2, "dout_pll_shared0_div2",
	    "mout_clkcmu_pll_shared0", CLK_CON_DIV_PLL_SHARED0_DIV2, 0, 1),
	DIV(CLK_DOUT_SHARED0_DIV3, "dout_pll_shared0_div3",
	    "mout_clkcmu_pll_shared0", CLK_CON_DIV_PLL_SHARED0_DIV3, 0, 2),
	DIV(CLK_DOUT_SHARED0_DIV4, "dout_pll_shared0_div4",
	    "dout_pll_shared0_div2", CLK_CON_DIV_PLL_SHARED0_DIV4, 0, 1),
	DIV(CLK_DOUT_SHARED1_DIV2, "dout_pll_shared1_div2",
	    "mout_clkcmu_pll_shared1", CLK_CON_DIV_PLL_SHARED1_DIV2, 0, 1),
	DIV(CLK_DOUT_SHARED1_DIV3, "dout_pll_shared1_div3",
	    "mout_clkcmu_pll_shared1", CLK_CON_DIV_PLL_SHARED1_DIV3, 0, 2),
	DIV(CLK_DOUT_SHARED1_DIV4, "dout_pll_shared1_div4",
	    "dout_pll_shared1_div2", CLK_CON_DIV_PLL_SHARED1_DIV4, 0, 1),
};

static const struct samsung_cmu_info cmu_cmu_info __initconst = {
	.pll_clks		= cmu_cmu_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(cmu_cmu_pll_clks),
	.mux_clks		= cmu_cmu_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_cmu_mux_clks),
	.div_clks		= cmu_cmu_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cmu_cmu_div_clks),
	.nr_clk_ids		= CMU_CMU_NR_CLK,
	.clk_regs		= cmu_cmu_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_cmu_clk_regs),
};

/* Register Offset definitions for CMU_BUS (0x13410000) */
#define PLL_CON0_MUX_CLK_BUS_ACLK_USER				0x0100

static const unsigned long cmu_bus_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLK_BUS_ACLK_USER,
};

PNAME(mout_clk_bus_aclk_user_p) = {"fin_pll", "dout_clkcmu_bus_bus",};

static const struct samsung_mux_clock cmu_bus_mux_clks[] __initconst = {
	MUX(CLK_MOUT_BUS_ACLK_USER, "mout_clk_bus_aclk_user", mout_clk_bus_aclk_user_p,
	    PLL_CON0_MUX_CLK_BUS_ACLK_USER, 4, 1),
};

static const struct samsung_cmu_info cmu_bus_info __initconst = {
	.mux_clks		= cmu_bus_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_bus_mux_clks),
	.nr_clk_ids		= CMU_BUS_NR_CLK,
	.clk_regs		= cmu_bus_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_bus_clk_regs),
};

/* Register Offset definitions for CMU_CORE (0x12c10000) */
#define PLL_CON0_MUX_CLK_CORE_ACLK_USER				0x0100

static const unsigned long cmu_core_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLK_CORE_ACLK_USER,
};

PNAME(mout_clk_core_aclk_user_p) = {"fin_pll", "dout_clkcmu_core_main",};

static const struct samsung_mux_clock cmu_core_mux_clks[] __initconst = {
	MUX(CLK_MOUT_CORE_ACLK_USER, "mout_clk_core_aclk_user", mout_clk_core_aclk_user_p,
	    PLL_CON0_MUX_CLK_CORE_ACLK_USER, 4, 1),
};

static const struct samsung_cmu_info cmu_core_info __initconst = {
	.mux_clks		= cmu_core_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_core_mux_clks),
	.nr_clk_ids		= CMU_CORE_NR_CLK,
	.clk_regs		= cmu_core_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_core_clk_regs),
};

/* Register Offset definitions for CMU_CPUCL (0x12810000) */
#define PLL_LOCKTIME_PLL0_CPUCL					0x0000
#define PLL_LOCKTIME_PLL1_CPUCL					0x0008
#define PLL_CON0_MUX_CLKCMU_CPUCL_SWITCH_SCU_USER		0x0100
#define PLL_CON0_MUX_CLKCMU_CPUCL_SWITCH_USER			0x0120
#define PLL_CON0_PLL0_CPUCL					0x0140
#define PLL_CON0_PLL1_CPUCL					0x0160
#define CLK_CON_MUX_CLK_CPUCL_PLL				0x1000
#define CLK_CON_MUX_CLK_CPUCL_PLL_SCU				0x1004
#define CLK_CON_DIV_CLK_CPUCL_CLUSTER_PERIPHCLK			0x1800
#define CLK_CON_DIV_CLK_CPUCL_CLUSTER_GICCLK			0x1804
#define CLK_CON_DIV_CLK_CPUCL_CLUSTER_PCLK			0x1808
#define CLK_CON_DIV_CLK_CPUCL_CMUREF				0x180c
#define CLK_CON_DIV_CLK_CPUCL_CPU				0x1810
#define CLK_CON_DIV_CLK_CPUCL_CLUSTER_ATCLK			0x1818
#define CLK_CON_DIV_CLK_CPUCL_CLUSTER_SCU			0x181c
#define CLK_CON_DIV_CLK_CPUCL_DBG				0x1820
#define CLK_CON_GAT_CLK_CLUSTER_CPU				0x2008
#define CLK_CON_GAT_CLK_CPUCL_SHORTSTOP				0x200c
#define CSSYS_IPCLKPORT_ATCLK					0x2070
#define CSSYS_IPCLKPORT_PCLKDBG					0x2074
#define DMYQCH_CON_CSSYS_QCH					0x3000
#define DMYQCH_CON_CLUSTER_QCH_CORECLK0				0x3104
#define DMYQCH_CON_CLUSTER_QCH_CORECLK1				0x3108
#define DMYQCH_CON_CLUSTER_QCH_CORECLK2				0x310c
#define DMYQCH_CON_CLUSTER_QCH_CORECLK3				0x3110
#define DMYQCH_CON_CLUSTER_QCH_CORECLK4				0x3114
#define DMYQCH_CON_CLUSTER_QCH_CORECLK5				0x3118
#define DMYQCH_CON_CLUSTER_QCH_PERIPHCLK			0x311c

static const unsigned long cmu_cpucl_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL0_CPUCL,
	PLL_LOCKTIME_PLL1_CPUCL,
	PLL_CON0_MUX_CLKCMU_CPUCL_SWITCH_SCU_USER,
	PLL_CON0_MUX_CLKCMU_CPUCL_SWITCH_USER,
	PLL_CON0_PLL0_CPUCL,
	PLL_CON0_PLL1_CPUCL,
	CLK_CON_MUX_CLK_CPUCL_PLL,
	CLK_CON_MUX_CLK_CPUCL_PLL_SCU,
	CLK_CON_DIV_CLK_CPUCL_CLUSTER_PERIPHCLK,
	CLK_CON_DIV_CLK_CPUCL_CLUSTER_GICCLK,
	CLK_CON_DIV_CLK_CPUCL_CLUSTER_PCLK,
	CLK_CON_DIV_CLK_CPUCL_CMUREF,
	CLK_CON_DIV_CLK_CPUCL_CPU,
	CLK_CON_DIV_CLK_CPUCL_CLUSTER_ATCLK,
	CLK_CON_DIV_CLK_CPUCL_CLUSTER_SCU,
	CLK_CON_DIV_CLK_CPUCL_DBG,
	CLK_CON_GAT_CLK_CLUSTER_CPU,
	CLK_CON_GAT_CLK_CPUCL_SHORTSTOP,
	CSSYS_IPCLKPORT_ATCLK,
	CSSYS_IPCLKPORT_PCLKDBG,
	DMYQCH_CON_CSSYS_QCH,
	DMYQCH_CON_CLUSTER_QCH_CORECLK0,
	DMYQCH_CON_CLUSTER_QCH_CORECLK1,
	DMYQCH_CON_CLUSTER_QCH_CORECLK2,
	DMYQCH_CON_CLUSTER_QCH_CORECLK3,
	DMYQCH_CON_CLUSTER_QCH_CORECLK4,
	DMYQCH_CON_CLUSTER_QCH_CORECLK5,
	DMYQCH_CON_CLUSTER_QCH_PERIPHCLK,
};

/* rate_table must be in descending order  */
static const struct samsung_pll_rate_table artpec9_pll_cpucl_rates[] __initconst = {
	PLL_35XX_RATE(25 * MHZ, 1400000000U, 56, 1, 0),
	PLL_35XX_RATE(25 * MHZ, 1100000000U, 44, 1, 0),
	PLL_35XX_RATE(25 * MHZ,  850000000U, 34, 1, 0),
};

static const struct samsung_pll_clock cmu_cpucl_pll_clks[] __initconst = {
	PLL(pll_a9fracm, CLK_FOUT_CPUCL_PLL0, "fout_pll0_cpucl", "fin_pll",
	    PLL_LOCKTIME_PLL0_CPUCL, PLL_CON0_PLL0_CPUCL, artpec9_pll_cpucl_rates),
	PLL(pll_a9fracm, CLK_FOUT_CPUCL_PLL1, "fout_pll1_cpucl", "fin_pll",
	    PLL_LOCKTIME_PLL1_CPUCL, PLL_CON0_PLL1_CPUCL, artpec9_pll_cpucl_rates),
};

PNAME(mout_clkcmu_cpucl_switch_scu_user_p) = { "fin_pll", "dout_clkcmu_cpucl_switch" };
PNAME(mout_clkcmu_cpucl_switch_user_p) = { "fin_pll", "dout_clkcmu_cpucl_switch" };
PNAME(mout_pll0_cpucl_p) = { "fin_pll", "fout_pll0_cpucl" };
PNAME(mout_clk_cpucl_pll0_p) = { "mout_pll0_cpucl", "mout_clkcmu_cpucl_switch_user" };
PNAME(mout_pll1_cpucl_p) = { "fin_pll", "fout_pll1_cpucl" };
PNAME(mout_clk_cpucl_pll_scu_p) = { "mout_pll1_cpucl", "mout_clkcmu_cpucl_switch_scu_user" };

static const struct samsung_mux_clock cmu_cpucl_mux_clks[] __initconst = {
	MUX_F(0, "mout_pll0_cpucl", mout_pll0_cpucl_p,
	      PLL_CON0_PLL0_CPUCL, 4, 1, CLK_SET_RATE_PARENT | CLK_RECALC_NEW_RATES, 0),
	MUX_F(0, "mout_pll1_cpucl", mout_pll1_cpucl_p,
	      PLL_CON0_PLL1_CPUCL, 4, 1, CLK_SET_RATE_PARENT | CLK_RECALC_NEW_RATES, 0),
	MUX(CLK_MOUT_CPUCL_SWITCH_SCU_USER, "mout_clkcmu_cpucl_switch_scu_user",
	    mout_clkcmu_cpucl_switch_scu_user_p, PLL_CON0_MUX_CLKCMU_CPUCL_SWITCH_SCU_USER, 4, 1),
	MUX(CLK_MOUT_CPUCL_SWITCH_USER, "mout_clkcmu_cpucl_switch_user",
	    mout_clkcmu_cpucl_switch_user_p, PLL_CON0_MUX_CLKCMU_CPUCL_SWITCH_USER, 4, 1),
	MUX_F(CLK_MOUT_CPUCL_PLL0, "mout_clk_cpucl_pll0",
	      mout_clk_cpucl_pll0_p, CLK_CON_MUX_CLK_CPUCL_PLL, 0, 1, CLK_SET_RATE_PARENT, 0),
	MUX_F(CLK_MOUT_CPUCL_PLL_SCU, "mout_clk_cpucl_pll_scu", mout_clk_cpucl_pll_scu_p,
	      CLK_CON_MUX_CLK_CPUCL_PLL_SCU, 0, 1, CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_fixed_factor_clock cpucl_ffactor_clks[] __initconst = {
	FFACTOR(CLK_DOUT_CPUCL_CPU, "dout_clk_cpucl_cpu",
		"mout_clk_cpucl_pll0", 1, 1, CLK_SET_RATE_PARENT),
};

static const struct samsung_div_clock cmu_cpucl_div_clks[] __initconst = {
	DIV(CLK_DOUT_CPUCL_CLUSTER_PERIPHCLK, "dout_clk_cluster_periphclk",
	    "clk_con_gat_clk_cluster_cpu", CLK_CON_DIV_CLK_CPUCL_CLUSTER_PERIPHCLK, 0, 4),
	DIV(CLK_DOUT_CPUCL_CLUSTER_GICCLK, "dout_clk_cluster_gicclk",
	    "clk_con_gat_clk_cluster_cpu", CLK_CON_DIV_CLK_CPUCL_CLUSTER_GICCLK, 0, 4),
	DIV(CLK_DOUT_CPUCL_CLUSTER_PCLK, "dout_clk_cluster_pclk",
	    "clk_con_gat_clk_cluster_cpu", CLK_CON_DIV_CLK_CPUCL_CLUSTER_PCLK, 0, 4),
	DIV(CLK_DOUT_CPUCL_CMUREF, "dout_clk_cpucl_cmuref",
	    "dout_clk_cpucl_cpu", CLK_CON_DIV_CLK_CPUCL_CMUREF, 0, 3),
	DIV(CLK_DOUT_CPUCL_CLUSTER_ATCLK, "dout_clk_cluster_atclk",
	    "clk_con_gat_clk_cluster_cpu", CLK_CON_DIV_CLK_CPUCL_CLUSTER_ATCLK, 0, 4),
	DIV_F(CLK_DOUT_CPUCL_CLUSTER_SCU, "dout_clk_cluster_scu", "mout_clk_cpucl_pll_scu",
	      CLK_CON_DIV_CLK_CPUCL_CLUSTER_SCU, 0, 4, CLK_SET_RATE_PARENT, 0),
	DIV(CLK_DOUT_CPUCL_DBG, "dout_clk_cpucl_dbg",
	    "dout_clk_cpucl_cpu", CLK_CON_DIV_CLK_CPUCL_DBG, 0, 4),
};

static const struct samsung_gate_clock cmu_cpucl_gate_clks[] __initconst = {
	GATE(CLK_GOUT_CPUCL_CLUSTER_CPU, "clk_con_gat_clk_cluster_cpu",
	     "clk_con_gat_clk_cpucl_shortstop", CLK_CON_GAT_CLK_CLUSTER_CPU, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CPUCL_SHORTSTOP, "clk_con_gat_clk_cpucl_shortstop", "dout_clk_cpucl_cpu",
	     CLK_CON_GAT_CLK_CPUCL_SHORTSTOP, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CPUCL_CSSYS_IPCLKPORT_ATCLK, "cssys_ipclkport_atclk", "dout_clk_cpucl_dbg",
	     CSSYS_IPCLKPORT_ATCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CPUCL_CSSYS_IPCLKPORT_PCLKDBG, "cssys_ipclkport_pclkdbg",
	     "dout_clk_cpucl_dbg", CSSYS_IPCLKPORT_PCLKDBG, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info cmu_cpucl_info __initconst = {
	.pll_clks		= cmu_cpucl_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(cmu_cpucl_pll_clks),
	.fixed_factor_clks	= cpucl_ffactor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(cpucl_ffactor_clks),
	.mux_clks		= cmu_cpucl_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_cpucl_mux_clks),
	.div_clks		= cmu_cpucl_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cmu_cpucl_div_clks),
	.gate_clks              = cmu_cpucl_gate_clks,
	.nr_gate_clks           = ARRAY_SIZE(cmu_cpucl_gate_clks),
	.nr_clk_ids		= CMU_CPUCL_NR_CLK,
	.clk_regs		= cmu_cpucl_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_cpucl_clk_regs),
};

/* Register Offset definitions for CMU_FSYS0 (0x14410000) */
#define PLL_CON0_MUX_CLK_FSYS0_BUS_USER				0x0100
#define PLL_CON0_MUX_CLK_FSYS0_IP_USER				0x0120
#define PLL_CON0_MUX_CLK_FSYS0_MAIN_USER			0x0140
#define CLK_CON_DIV_CLK_FSYS0_125				0x1800
#define CLK_CON_DIV_CLK_FSYS0_ADC				0x1804
#define CLK_CON_DIV_CLK_FSYS0_BUS_300				0x1808
#define CLK_CON_DIV_CLK_FSYS0_EQOS0				0x1814
#define CLK_CON_DIV_CLK_FSYS0_EQOS1				0x1818
#define CLK_CON_DIV_CLK_FSYS0_EQOS_250				0x181C
#define CLK_CON_DIV_CLK_FSYS0_MMC_CARD0				0x1820
#define CLK_CON_DIV_CLK_FSYS0_MMC_CARD1				0x1824
#define CLK_CON_DIV_CLK_FSYS0_MMC_CARD2				0x1828
#define CLK_CON_DIV_CLK_FSYS0_QSPI				0x182c
#define CLK_CON_DIV_CLK_FSYS0_SFMC_NAND				0x1830
#define CLK_CON_FSYS0_I2C0_IPCLKPORT_I_PCLK			0x2040
#define CLK_CON_FSYS0_I2C1_IPCLKPORT_I_PCLK			0x2044
#define CLK_CON_MMC0_IPCLKPORT_I_ACLK				0x2078
#define CLK_CON_MMC1_IPCLKPORT_I_ACLK				0x2080
#define CLK_CON_MMC2_IPCLKPORT_I_ACLK				0x2088
#define CLK_CON_PWM_IPCLKPORT_I_PCLK_S0				0x2090
#define CLK_CON_DMYQCH_CON_ADC_WRAP_QCH				0x3000
#define CLK_CON_DMYQCH_CON_EQOS_TOP0_QCH			0x3004
#define CLK_CON_DMYQCH_CON_EQOS_TOP1_QCH			0x3008
#define CLK_CON_DMYQCH_CON_FSYS0_I3C0_QCH			0x3010
#define CLK_CON_DMYQCH_CON_FSYS0_I3C1_QCH			0x3014
#define CLK_CON_DMYQCH_CON_MMC0_QCH				0x3018
#define CLK_CON_DMYQCH_CON_MMC1_QCH				0x301c
#define CLK_CON_DMYQCH_CON_MMC2_QCH				0x3020
#define CLK_CON_DMYQCH_CON_QSPI_QCH				0x3024
#define CLK_CON_DMYQCH_CON_SFMC_QCH				0x3028

static const unsigned long cmu_fsys0_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLK_FSYS0_BUS_USER,
	PLL_CON0_MUX_CLK_FSYS0_IP_USER,
	PLL_CON0_MUX_CLK_FSYS0_MAIN_USER,
	CLK_CON_DIV_CLK_FSYS0_125,
	CLK_CON_DIV_CLK_FSYS0_ADC,
	CLK_CON_DIV_CLK_FSYS0_BUS_300,
	CLK_CON_DIV_CLK_FSYS0_EQOS0,
	CLK_CON_DIV_CLK_FSYS0_EQOS1,
	CLK_CON_DIV_CLK_FSYS0_EQOS_250,
	CLK_CON_DIV_CLK_FSYS0_MMC_CARD0,
	CLK_CON_DIV_CLK_FSYS0_MMC_CARD1,
	CLK_CON_DIV_CLK_FSYS0_MMC_CARD2,
	CLK_CON_DIV_CLK_FSYS0_QSPI,
	CLK_CON_DIV_CLK_FSYS0_SFMC_NAND,
	CLK_CON_FSYS0_I2C0_IPCLKPORT_I_PCLK,
	CLK_CON_FSYS0_I2C1_IPCLKPORT_I_PCLK,
	CLK_CON_MMC0_IPCLKPORT_I_ACLK,
	CLK_CON_MMC1_IPCLKPORT_I_ACLK,
	CLK_CON_MMC2_IPCLKPORT_I_ACLK,
	CLK_CON_PWM_IPCLKPORT_I_PCLK_S0,
	CLK_CON_DMYQCH_CON_ADC_WRAP_QCH,
	CLK_CON_DMYQCH_CON_EQOS_TOP0_QCH,
	CLK_CON_DMYQCH_CON_EQOS_TOP1_QCH,
	CLK_CON_DMYQCH_CON_FSYS0_I3C0_QCH,
	CLK_CON_DMYQCH_CON_FSYS0_I3C1_QCH,
	CLK_CON_DMYQCH_CON_MMC0_QCH,
	CLK_CON_DMYQCH_CON_MMC1_QCH,
	CLK_CON_DMYQCH_CON_MMC2_QCH,
	CLK_CON_DMYQCH_CON_QSPI_QCH,
	CLK_CON_DMYQCH_CON_SFMC_QCH,
};

PNAME(mout_fsys0_bus_user_p) = { "fin_pll", "dout_clkcmu_fsys0_bus" };
PNAME(mout_fsys0_ip_user_p) = { "fin_pll", "dout_clkcmu_fsys0_ip" };
PNAME(mout_fsys0_main_user_p) = { "fin_pll", "fout_pll_fsys1" };

static const struct samsung_mux_clock cmu_fsys0_mux_clks[] __initconst = {
	MUX(CLK_MOUT_FSYS0_BUS_USER, "mout_fsys0_bus_user",
	    mout_fsys0_bus_user_p, PLL_CON0_MUX_CLK_FSYS0_BUS_USER, 4, 1),
	MUX(CLK_MOUT_FSYS0_IP_USER, "mout_fsys0_ip_user",
	    mout_fsys0_ip_user_p, PLL_CON0_MUX_CLK_FSYS0_IP_USER, 4, 1),
	MUX(CLK_MOUT_FSYS0_MAIN_USER, "mout_fsys0_main_user",
	    mout_fsys0_main_user_p, PLL_CON0_MUX_CLK_FSYS0_MAIN_USER, 4, 1),
};

static const struct samsung_div_clock cmu_fsys0_div_clks[] __initconst = {
	DIV(CLK_DOUT_FSYS0_125, "dout_fsys0_125", "mout_fsys0_main_user",
	    CLK_CON_DIV_CLK_FSYS0_125, 0, 5),
	DIV(CLK_DOUT_FSYS0_ADC, "dout_fsys0_adc", "mout_fsys0_main_user",
	    CLK_CON_DIV_CLK_FSYS0_ADC, 0, 7),
	DIV(CLK_DOUT_FSYS0_BUS_300, "dout_fsys0_bus_300", "mout_fsys0_bus_user",
	    CLK_CON_DIV_CLK_FSYS0_BUS_300, 0, 4),
	DIV(CLK_DOUT_FSYS0_EQOS0, "dout_fsys0_eqos0", "dout_fsys0_eqos_250",
	    CLK_CON_DIV_CLK_FSYS0_EQOS0, 0, 7),
	DIV(CLK_DOUT_FSYS0_EQOS1, "dout_fsys0_eqos1", "dout_fsys0_eqos_250",
	    CLK_CON_DIV_CLK_FSYS0_EQOS1, 0, 7),
	DIV(0, "dout_fsys0_eqos_250", "mout_fsys0_main_user",
	    CLK_CON_DIV_CLK_FSYS0_EQOS_250, 0, 4),
	DIV(CLK_DOUT_FSYS0_MMC_CARD0, "dout_fsys0_mmc_card0", "mout_fsys0_ip_user",
	    CLK_CON_DIV_CLK_FSYS0_MMC_CARD0, 0, 10),
	DIV(CLK_DOUT_FSYS0_MMC_CARD1, "dout_fsys0_mmc_card1", "mout_fsys0_ip_user",
	    CLK_CON_DIV_CLK_FSYS0_MMC_CARD1, 0, 10),
	DIV(CLK_DOUT_FSYS0_MMC_CARD2, "dout_fsys0_mmc_card2", "mout_fsys0_ip_user",
	    CLK_CON_DIV_CLK_FSYS0_MMC_CARD2, 0, 10),
	DIV(CLK_DOUT_FSYS0_QSPI, "dout_fsys0_qspi", "mout_fsys0_ip_user",
	    CLK_CON_DIV_CLK_FSYS0_QSPI, 0, 4),
	DIV(CLK_DOUT_FSYS0_SFMC_NAND, "dout_fsys0_sfmc_nand", "mout_fsys0_ip_user",
	    CLK_CON_DIV_CLK_FSYS0_SFMC_NAND, 0, 4),
};

static const struct samsung_gate_clock cmu_fsys0_gate_clks[] __initconst = {
	GATE(0, "adc_wrap_ipclkport_clk", "dout_fsys0_adc",
	     CLK_CON_DMYQCH_CON_ADC_WRAP_QCH, 1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP0_IPCLKPORT_ACLK_I, "eqos_top0_ipclkport_aclk_i",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_EQOS_TOP0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP0_IPCLKPORT_CLK_CSR_I, "eqos_top0_ipclkport_clk_csr_i",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_EQOS_TOP0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP0_IPCLKPORT_I_RGMII_PHASE_CLK_250,
	     "eqos_top0_ipclkport_i_rgmii_phase_clk_250",
	     "dout_fsys0_eqos_250", CLK_CON_DMYQCH_CON_EQOS_TOP0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP0_IPCLKPORT_I_RGMII_TXCLK, "eqos_top0_ipclkport_i_rgmii_txclk",
	     "dout_fsys0_eqos0", CLK_CON_DMYQCH_CON_EQOS_TOP0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP1_IPCLKPORT_I_RGMII_PHASE_CLK_250,
	     "eqos_top1_ipclkport_i_rgmii_phase_clk_250",
	     "dout_fsys0_eqos_250", CLK_CON_DMYQCH_CON_EQOS_TOP1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP1_IPCLKPORT_I_RGMII_TXCLK, "eqos_top1_ipclkport_i_rgmii_txclk",
	     "dout_fsys0_eqos1", CLK_CON_DMYQCH_CON_EQOS_TOP1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP1_IPCLKPORT_ACLK_I, "eqos_top1_ipclkport_aclk_i",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_EQOS_TOP1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_EQOS_TOP1_IPCLKPORT_CLK_CSR_I, "eqos_top1_ipclkport_clk_csr_i",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_EQOS_TOP1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_I3C0_IPCLKPORT_I_APB_S_PCLK, "i3c0_ipclkport_i_apb_s_pclk",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_FSYS0_I3C0_QCH, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_I3C0_IPCLKPORT_I_CORE_CLK, "i3c0_ipclkport_i_core_clk",
	     "dout_fsys0_125", CLK_CON_DMYQCH_CON_FSYS0_I3C0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_I3C0_IPCLKPORT_I_DMA_CLK, "i3c0_ipclkport_i_dma_clk",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_FSYS0_I3C0_QCH,
	     1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_I3C0_IPCLKPORT_I_HDR_TX_CLK, "i3c0_ipclkport_i_hdr_tx_clk",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_FSYS0_I3C0_QCH,
	     1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_I3C1_IPCLKPORT_I_APB_S_PCLK, "i3c1_ipclkport_i_apb_s_pclk",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_FSYS0_I3C1_QCH,
	     1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_I3C1_IPCLKPORT_I_CORE_CLK, "i3c1_ipclkport_i_core_clk",
	     "dout_fsys0_125", CLK_CON_DMYQCH_CON_FSYS0_I3C1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_I3C1_IPCLKPORT_I_DMA_CLK, "i3c1_ipclkport_i_dma_clk",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_FSYS0_I3C1_QCH,
	     1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_I3C1_IPCLKPORT_I_HDR_TX_CLK, "i3c1_ipclkport_i_hdr_tx_clk",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_FSYS0_I3C1_QCH,
	     1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_MMC0_IPCLKPORT_SDCLKIN, "mmc0_ipclkport_sdclkin",
	     "dout_fsys0_mmc_card0", CLK_CON_DMYQCH_CON_MMC0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_MMC1_IPCLKPORT_SDCLKIN, "mmc1_ipclkport_sdclkin",
	     "dout_fsys0_mmc_card1", CLK_CON_DMYQCH_CON_MMC1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_MMC2_IPCLKPORT_SDCLKIN, "mmc2_ipclkport_sdclkin",
	     "dout_fsys0_mmc_card2", CLK_CON_DMYQCH_CON_MMC2_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_QSPI_IPCLKPORT_HCLK, "qspi_ipclkport_hclk",
	     "dout_fsys0_bus_300", CLK_CON_DMYQCH_CON_QSPI_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_QSPI_IPCLKPORT_SSI_CLK, "qspi_ipclkport_ssi_clk", "dout_fsys0_qspi",
	     CLK_CON_DMYQCH_CON_QSPI_QCH, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_SFMC_IPCLKPORT_I_ACLK_NAND, "sfmc_ipclkport_i_aclk_nand",
	     "dout_fsys0_sfmc_nand", CLK_CON_DMYQCH_CON_SFMC_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS0_I2C0_IPCLKPORT_I_PCLK, "i2c0_ipclkport_i_pclk", "dout_fsys0_bus_300",
	     CLK_CON_FSYS0_I2C0_IPCLKPORT_I_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_I2C1_IPCLKPORT_I_PCLK, "i2c1_ipclkport_i_pclk", "dout_fsys0_bus_300",
	     CLK_CON_FSYS0_I2C1_IPCLKPORT_I_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_MMC0_IPCLKPORT_I_ACLK, "mmc0_ipclkport_i_aclk", "dout_fsys0_bus_300",
	     CLK_CON_MMC0_IPCLKPORT_I_ACLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_MMC1_IPCLKPORT_I_ACLK, "mmc1_ipclkport_i_aclk", "dout_fsys0_bus_300",
	     CLK_CON_MMC1_IPCLKPORT_I_ACLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_MMC2_IPCLKPORT_I_ACLK, "mmc2_ipclkport_i_aclk", "dout_fsys0_bus_300",
	     CLK_CON_MMC2_IPCLKPORT_I_ACLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS0_PWM_IPCLKPORT_I_PCLK_S0, "pwm_ipclkport_i_pclk", "dout_fsys0_bus_300",
	     CLK_CON_PWM_IPCLKPORT_I_PCLK_S0, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info cmu_fsys0_info __initconst = {
	.mux_clks		= cmu_fsys0_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_fsys0_mux_clks),
	.div_clks		= cmu_fsys0_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cmu_fsys0_div_clks),
	.gate_clks              = cmu_fsys0_gate_clks,
	.nr_gate_clks           = ARRAY_SIZE(cmu_fsys0_gate_clks),
	.nr_clk_ids		= CMU_FSYS0_NR_CLK,
	.clk_regs		= cmu_fsys0_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_fsys0_clk_regs),
};

/* Register Offset definitions for CMU_FSYS1 (0x14c10000) */
#define PLL_LOCKTIME_PLL_FSYS1						0x0000
#define PLL_CON0_MUX_CLK_FSYS1_BUS_USER					0x0100
#define PLL_CON0_MUX_CLK_FSYS1_SCAN0_USER				0x0120
#define PLL_CON0_MUX_CLK_FSYS1_SCAN1_USER				0x0140
#define PLL_CON0_PLL_FSYS1						0x0160
#define CLK_CON_DIV_CLK_FSYS1_200					0x1808
#define CLK_CON_DIV_CLK_FSYS1_BUS_300					0x1810
#define CLK_CON_DIV_CLK_FSYS1_OTP_MEM					0x1814
#define CLK_CON_DIV_CLK_FSYS1_PCIE_PHY_REFCLK_SYSPLL			0x1818
#define CLK_CON_FSYS1_UART0_IPCLKPORT_I_PCLK				0x202c
#define CLK_CON_FSYS1_UART0_IPCLKPORT_I_SCLK_UART			0x2030
#define CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_PHY_APB2CR_PCLK_300		0x205c
#define CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_DBI_ACLK_SOC		0x2068
#define CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_MSTR_ACLK_SOC	0x206c
#define CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_SLV_ACLK_SOC		0x2070
#define CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_DBI_ACLK_SOC		0x2078
#define CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_MSTR_ACLK_SOC	0x2080
#define CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_SLV_ACLK_SOC		0x2084
#define CLK_CON_USB20DRD_IPCLKPORT_ACLK_PHYCTRL_20			0x209c
#define CLK_CON_USB20DRD_IPCLKPORT_BUS_CLK_EARLY			0x20a0
#define CLK_CON_XHB_AHBBR_FSYS1_IPCLKPORT_CLK				0x20a8
#define CLK_CON_XHB_USB_IPCLKPORT_CLK					0x20ac
#define CLK_CON_DMYQCH_CON_TZ400_QCH					0x3004
#define CLK_CON_DMYQCH_CON_PCIE_TOP_QCH_PHY_100				0x309c
#define CLK_CON_QCH_CON_MMU_FSYS1_QCH_U_TBU_0_0				0x3050
#define CLK_CON_QCH_CON_MMU_FSYS1_QCH_U_TBU_1_0				0x3058

static const unsigned long cmu_fsys1_clk_regs[] __initconst = {
	PLL_LOCKTIME_PLL_FSYS1,
	PLL_CON0_MUX_CLK_FSYS1_BUS_USER,
	PLL_CON0_MUX_CLK_FSYS1_SCAN0_USER,
	PLL_CON0_MUX_CLK_FSYS1_SCAN1_USER,
	PLL_CON0_PLL_FSYS1,
	CLK_CON_DIV_CLK_FSYS1_200,
	CLK_CON_DIV_CLK_FSYS1_BUS_300,
	CLK_CON_DIV_CLK_FSYS1_OTP_MEM,
	CLK_CON_DIV_CLK_FSYS1_PCIE_PHY_REFCLK_SYSPLL,
	CLK_CON_FSYS1_UART0_IPCLKPORT_I_PCLK,
	CLK_CON_FSYS1_UART0_IPCLKPORT_I_SCLK_UART,
	CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_PHY_APB2CR_PCLK_300,
	CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_DBI_ACLK_SOC,
	CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_MSTR_ACLK_SOC,
	CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_SLV_ACLK_SOC,
	CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_DBI_ACLK_SOC,
	CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_MSTR_ACLK_SOC,
	CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_SLV_ACLK_SOC,
	CLK_CON_USB20DRD_IPCLKPORT_ACLK_PHYCTRL_20,
	CLK_CON_USB20DRD_IPCLKPORT_BUS_CLK_EARLY,
	CLK_CON_XHB_AHBBR_FSYS1_IPCLKPORT_CLK,
	CLK_CON_XHB_USB_IPCLKPORT_CLK,
	CLK_CON_DMYQCH_CON_TZ400_QCH,
	CLK_CON_DMYQCH_CON_PCIE_TOP_QCH_PHY_100,
	CLK_CON_QCH_CON_MMU_FSYS1_QCH_U_TBU_0_0,
	CLK_CON_QCH_CON_MMU_FSYS1_QCH_U_TBU_1_0
};

static const struct samsung_pll_rate_table artpec9_pll_fsys1_rates[] __initconst = {
	PLL_35XX_RATE(25 * MHZ, 2000000000U, 80, 1, 0),
};

static const struct samsung_pll_clock cmu_fsys1_pll_clks[] __initconst = {
	PLL(pll_a9fracm, CLK_FOUT_FSYS1_PLL, "fout_pll_fsys1", "fin_pll",
	    PLL_LOCKTIME_PLL_FSYS1, PLL_CON0_PLL_FSYS1, artpec9_pll_fsys1_rates),
};

PNAME(mout_fsys1_scan0_user_p) = { "fin_pll", "dout_clkcmu_fsys1_scan0" };
PNAME(mout_fsys1_scan1_user_p) = { "fin_pll", "dout_clkcmu_fsys1_scan1" };
PNAME(mout_fsys1_bus_user_p) = { "fin_pll", "dout_clkcmu_fsys1_bus" };
PNAME(mout_fsys_pll_fsys_p) = { "fin_pll", "fout_pll_fsys1" };

static const struct samsung_mux_clock cmu_fsys1_mux_clks[] __initconst = {
	MUX(0, "mout_clk_pll_fsys1", mout_fsys_pll_fsys_p, PLL_CON0_PLL_FSYS1, 4, 1),
	MUX(CLK_MOUT_FSYS1_SCAN0_USER, "mout_fsys1_scan0_user",
	    mout_fsys1_scan0_user_p, PLL_CON0_MUX_CLK_FSYS1_SCAN0_USER, 4, 1),
	MUX(CLK_MOUT_FSYS1_SCAN1_USER, "mout_fsys1_scan1_user",
	    mout_fsys1_scan1_user_p, PLL_CON0_MUX_CLK_FSYS1_SCAN1_USER, 4, 1),
	MUX(CLK_MOUT_FSYS1_BUS_USER, "mout_fsys1_bus_user",
	    mout_fsys1_bus_user_p, PLL_CON0_MUX_CLK_FSYS1_BUS_USER, 4, 1),
};

static const struct samsung_div_clock cmu_fsys1_div_clks[] __initconst = {
	DIV(CLK_DOUT_FSYS1_200, "dout_fsys1_200", "mout_clk_pll_fsys1",
	    CLK_CON_DIV_CLK_FSYS1_200, 0, 4),
	DIV(CLK_DOUT_FSYS1_BUS_300, "dout_fsys1_bus_300", "mout_fsys1_bus_user",
	    CLK_CON_DIV_CLK_FSYS1_BUS_300, 0, 4),
	DIV(CLK_DOUT_FSYS1_OTP_MEM, "dout_fsys1_otp_mem", "fin_pll",
	    CLK_CON_DIV_CLK_FSYS1_OTP_MEM, 0, 4),
	DIV(CLK_DOUT_FSYS1_PCIE_PHY_REFCLK_SYSPLL, "dout_fsys1_pcie_phy_refclk_syspll",
	    "mout_clk_pll_fsys1", CLK_CON_DIV_CLK_FSYS1_PCIE_PHY_REFCLK_SYSPLL, 0, 5),
};

static const struct samsung_gate_clock cmu_fsys1_gate_clks[] __initconst = {
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_PHY_APB2CR_PCLK_100,
	     "pcie_top_ipclkport_pcie_phy_apb2cr_pclk_100", "dout_fsys1_pcie_phy_refclk_syspll",
	     CLK_CON_DMYQCH_CON_PCIE_TOP_QCH_PHY_100, 1, CLK_SET_RATE_PARENT, 0),
	GATE(0, "tzc400_ipclkport_aclk0", "mout_fsys1_bus_user",
	     CLK_CON_DMYQCH_CON_TZ400_QCH, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "tzc400_ipclkport_aclk1", "mout_fsys1_bus_user",
	     CLK_CON_DMYQCH_CON_TZ400_QCH, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "tzc400_ipclkport_pclk", "dout_fsys1_bus_300",
	     CLK_CON_DMYQCH_CON_TZ400_QCH, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_UART0_PCLK, "uart", "dout_fsys1_bus_300",
	     CLK_CON_FSYS1_UART0_IPCLKPORT_I_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_UART0_SCLK_UART, "clk_uart_baud0", "dout_fsys1_200",
	     CLK_CON_FSYS1_UART0_IPCLKPORT_I_SCLK_UART, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_PHY_APB2CR_PCLK_300,
	     "pcie_top_ipclkport_pcie_phy_apb2cr_pclk_300", "dout_fsys1_bus_300",
	     CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_PHY_APB2CR_PCLK_300, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_SUB_CON_X1_DBI_ACLK_SOC,
	     "pcie_top_ipclkport_pcie_sub_con_x1_dbi_aclk_soc", "dout_fsys1_bus_300",
	     CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_DBI_ACLK_SOC,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_SUB_CON_X1_MSTR_ACLK_SOC,
	     "pcie_top_ipclkport_pcie_sub_con_x1_mstr_aclk_soc", "mout_fsys1_bus_user",
	     CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_MSTR_ACLK_SOC,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_SUB_CON_X1_SLV_ACLK_SOC,
	     "pcie_top_ipclkport_pcie_sub_con_x1_slv_aclk_soc", "mout_fsys1_bus_user",
	     CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X1_SLV_ACLK_SOC,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_SUB_CON_X2_DBI_ACLK_SOC,
	     "pcie_top_ipclkport_pcie_sub_con_x2_dbi_aclk_soc", "dout_fsys1_bus_300",
	     CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_DBI_ACLK_SOC,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_SUB_CON_X2_MSTR_ACLK_SOC,
	     "pcie_top_ipclkport_pcie_sub_con_x2_mstr_aclk_soc", "mout_fsys1_bus_user",
	     CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_MSTR_ACLK_SOC,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_IPCLKPORT_PCIE_SUB_CON_X2_SLV_ACLK_SOC,
	     "pcie_top_ipclkport_pcie_sub_con_x2_slv_aclk_soc", "mout_fsys1_bus_user",
	     CLK_CON_PCIE_TOP_IPCLKPORT_PCIE_SUB_CON_X2_SLV_ACLK_SOC,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_USB20DRD_IPCLKPORT_ACLK_PHYCTRL_20,
	     "usb20drd_ipclkport_aclk_phyctrl_20", "dout_fsys1_bus_300",
	     CLK_CON_USB20DRD_IPCLKPORT_ACLK_PHYCTRL_20,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_USB20DRD_IPCLKPORT_BUS_CLK_EARLY, "usb20drd_ipclkport_bus_clk_early",
	     "dout_fsys1_bus_300", CLK_CON_USB20DRD_IPCLKPORT_BUS_CLK_EARLY,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_XHB_AHBBR_FSYS1_IPCLKPORT_CLK, "xhb_ahbbr_fsys1_ipclkport_clk",
	     "dout_fsys1_bus_300", CLK_CON_XHB_AHBBR_FSYS1_IPCLKPORT_CLK,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_XHB_USB_IPCLKPORT_CLK, "xhb_usb_ipclkport_clk", "dout_fsys1_bus_300",
	     CLK_CON_XHB_USB_IPCLKPORT_CLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "qch_con_mmu_fsys1_qch_u_tbu_0_0", "mout_fsys1_bus_user",
	     CLK_CON_QCH_CON_MMU_FSYS1_QCH_U_TBU_0_0, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "qch_con_mmu_fsys1_qch_u_tbu_1_0", "mout_fsys1_bus_user",
	     CLK_CON_QCH_CON_MMU_FSYS1_QCH_U_TBU_1_0, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info cmu_fsys1_info __initconst = {
	.pll_clks		= cmu_fsys1_pll_clks,
	.nr_pll_clks		= ARRAY_SIZE(cmu_fsys1_pll_clks),
	.mux_clks		= cmu_fsys1_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_fsys1_mux_clks),
	.div_clks		= cmu_fsys1_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cmu_fsys1_div_clks),
	.gate_clks              = cmu_fsys1_gate_clks,
	.nr_gate_clks           = ARRAY_SIZE(cmu_fsys1_gate_clks),
	.nr_clk_ids		= CMU_FSYS1_NR_CLK,
	.clk_regs		= cmu_fsys1_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_fsys1_clk_regs),
};

/* Register Offset definitions for CMU_IMEM (0x10010000) */
#define PLL_CON0_MUX_CLK_IMEM_ACLK_USER				0x0100
#define PLL_CON0_MUX_CLK_IMEM_CA5_USER				0x0120
#define PLL_CON0_MUX_CLK_IMEM_JPEG_USER				0x0140
#define PLL_CON0_MUX_CLK_IMEM_SSS_USER				0x0160
#define CLK_CON_MCT0_IPCLKPORT_PCLK					0x20b4
#define CLK_CON_MCT1_IPCLKPORT_PCLK					0x20b8
#define CLK_CON_MCT2_IPCLKPORT_PCLK					0x20bc
#define CLK_CON_MCT3_IPCLKPORT_PCLK					0x20c0
#define CLK_CON_TMU_APB_IPCLKPORT_PCLK					0x20d4
#define CLK_CON_DMYQCH_CON_CA5_0_QCH					0x3008
#define CLK_CON_DMYQCH_CON_CA5_1_QCH					0x3018
#define CLK_CON_DMYQCH_CON_INTMEM_QCH					0x3020
#define CLK_CON_QCH_CON_GIC_CA55_QCHANNEL_SLAVE_0			0x306c
#define CLK_CON_QCH_CON_GIC_CA5_0_QCH					0x3078
#define CLK_CON_QCH_CON_GIC_CA5_1_QCH					0x307c
#define CLK_CON_QCH_CON_MMU_IMEM_QCH_U_TBU_0_0				0x30ac
#define CLK_CON_QCH_CON_MMU_IMEM_QCH_U_TBU_1_0				0x30b4

static const unsigned long cmu_imem_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLK_IMEM_ACLK_USER,
	PLL_CON0_MUX_CLK_IMEM_CA5_USER,
	PLL_CON0_MUX_CLK_IMEM_JPEG_USER,
	PLL_CON0_MUX_CLK_IMEM_SSS_USER,
	CLK_CON_MCT0_IPCLKPORT_PCLK,
	CLK_CON_MCT1_IPCLKPORT_PCLK,
	CLK_CON_MCT2_IPCLKPORT_PCLK,
	CLK_CON_MCT3_IPCLKPORT_PCLK,
	CLK_CON_TMU_APB_IPCLKPORT_PCLK,
	CLK_CON_DMYQCH_CON_CA5_0_QCH,
	CLK_CON_DMYQCH_CON_CA5_1_QCH,
	CLK_CON_DMYQCH_CON_INTMEM_QCH,
	CLK_CON_QCH_CON_GIC_CA55_QCHANNEL_SLAVE_0,
	CLK_CON_QCH_CON_GIC_CA5_0_QCH,
	CLK_CON_QCH_CON_GIC_CA5_1_QCH,
	CLK_CON_QCH_CON_MMU_IMEM_QCH_U_TBU_0_0,
	CLK_CON_QCH_CON_MMU_IMEM_QCH_U_TBU_1_0
};

PNAME(mout_imem_aclk_user_p) = { "fin_pll", "dout_clkcmu_imem_aclk" };
PNAME(mout_imem_ca5_user_p) = { "fin_pll", "dout_clkcmu_imem_ca5" };
PNAME(mout_imem_jpeg_user_p) = { "fin_pll", "dout_clkcmu_imem_jpeg" };
PNAME(mout_imem_sss_user_p) = { "fin_pll", "dout_clkcmu_imem_sss" };

static const struct samsung_mux_clock cmu_imem_mux_clks[] __initconst = {
	MUX(CLK_MOUT_IMEM_ACLK_USER, "mout_clk_imem_aclk_user",
	    mout_imem_aclk_user_p, PLL_CON0_MUX_CLK_IMEM_ACLK_USER, 4, 1),
	MUX(CLK_MOUT_IMEM_CA5_USER, "mout_clk_imem_ca5_user",
	    mout_imem_ca5_user_p, PLL_CON0_MUX_CLK_IMEM_CA5_USER, 4, 1),
	MUX(CLK_MOUT_IMEM_SSS_USER, "mout_clk_imem_sss_user",
	    mout_imem_sss_user_p, PLL_CON0_MUX_CLK_IMEM_SSS_USER, 4, 1),
	MUX(CLK_MOUT_IMEM_JPEG_USER, "mout_clk_imem_jpeg_user",
	    mout_imem_jpeg_user_p, PLL_CON0_MUX_CLK_IMEM_JPEG_USER, 4, 1),
};

static const struct samsung_fixed_factor_clock imem_ffactor_clks[] __initconst = {
	FFACTOR(CLK_DOUT_IMEM_PCLK, "dout_clk_imem_pclk", "mout_clk_imem_aclk_user", 1, 2, 0),
};

static const struct samsung_gate_clock cmu_imem_gate_clks[] __initconst = {
	GATE(CLK_GOUT_IMEM_CA5_0_IPCLKPORT_ATCLK, "ca5_0_ipclkport_atclk",
	     "mout_clk_imem_ca5_user", CLK_CON_DMYQCH_CON_CA5_0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_IMEM_CA5_0_IPCLKPORT_CLKIN, "ca5_0_ipclkport_clkin",
	     "mout_clk_imem_ca5_user", CLK_CON_DMYQCH_CON_CA5_0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_IMEM_CA5_0_IPCLKPORT_PCLK_DBG, "ca5_0_ipclkport_pclk_dbg",
	     "mout_clk_imem_ca5_user", CLK_CON_DMYQCH_CON_CA5_0_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_IMEM_CA5_1_IPCLKPORT_ATCLK, "ca5_1_ipclkport_atclk",
	     "mout_clk_imem_ca5_user", CLK_CON_DMYQCH_CON_CA5_1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_IMEM_CA5_1_IPCLKPORT_CLKIN, "ca5_1_ipclkport_clkin",
	     "mout_clk_imem_ca5_user", CLK_CON_DMYQCH_CON_CA5_1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_IMEM_CA5_1_IPCLKPORT_PCLK_DBG, "ca5_1_ipclkport_pclk_dbg",
	     "mout_clk_imem_ca5_user", CLK_CON_DMYQCH_CON_CA5_1_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(0, "intmem_ipclkport_aclk", "mout_clk_imem_aclk_user",
	     CLK_CON_DMYQCH_CON_INTMEM_QCH, 1, CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_IMEM_MCT0_PCLK, "mct0", "dout_clk_imem_pclk",
	     CLK_CON_MCT0_IPCLKPORT_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_IMEM_MCT1_PCLK, "mct1", "dout_clk_imem_pclk",
	     CLK_CON_MCT1_IPCLKPORT_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_IMEM_MCT2_PCLK, "mct2", "dout_clk_imem_pclk",
	     CLK_CON_MCT2_IPCLKPORT_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_IMEM_MCT3_PCLK, "mct3", "dout_clk_imem_pclk",
	     CLK_CON_MCT3_IPCLKPORT_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_IMEM_PCLK_TMU0_APBIF, "tmu_apb_ipclkport_pclk", "dout_clk_imem_pclk",
	     CLK_CON_TMU_APB_IPCLKPORT_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "qch_con_gic_ca55_qchannel_slave_0", "dout_clk_imem_pclk",
	     CLK_CON_QCH_CON_GIC_CA55_QCHANNEL_SLAVE_0, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "qch_con_gic_ca5_0_qch", "dout_clk_imem_pclk",
	     CLK_CON_QCH_CON_GIC_CA5_0_QCH, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "qch_con_gic_ca5_1_qch", "dout_clk_imem_pclk",
	     CLK_CON_QCH_CON_GIC_CA5_1_QCH, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "qch_con_mmu_imem_qch_u_tbu_0_0", "mout_clk_imem_ca5_user",
	     CLK_CON_QCH_CON_MMU_IMEM_QCH_U_TBU_0_0, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(0, "qch_con_mmu_imem_qch_u_tbu_1_0", "mout_clk_imem_ca5_user",
	     CLK_CON_QCH_CON_MMU_IMEM_QCH_U_TBU_1_0, 1, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info cmu_imem_info __initconst = {
	.fixed_factor_clks	= imem_ffactor_clks,
	.nr_fixed_factor_clks	= ARRAY_SIZE(imem_ffactor_clks),
	.mux_clks		= cmu_imem_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_imem_mux_clks),
	.gate_clks              = cmu_imem_gate_clks,
	.nr_gate_clks           = ARRAY_SIZE(cmu_imem_gate_clks),
	.nr_clk_ids		= CMU_IMEM_NR_CLK,
	.clk_regs		= cmu_imem_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_imem_clk_regs),
};

static void __init artpec9_cmu_imem_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &cmu_imem_info);
}

CLK_OF_DECLARE(artpec9_cmu_imem, "axis,artpec9-cmu-imem", artpec9_cmu_imem_init);

/* Register Offset definitions for CMU_PERI (0x14010000) */
#define PLL_CON0_MUX_CLK_PERI_DISP_USER				0x0100
#define PLL_CON0_MUX_CLK_PERI_IP_USER				0x0120
#define CLK_CON_DIV_CLK_PERI_125				0x1800
#define CLK_CON_DIV_CLK_PERI_PCLK				0x180c
#define CLK_CON_DIV_CLK_PERI_SPI				0x1810
#define CLK_CON_DIV_CLK_PERI_UART1				0x1814
#define CLK_CON_DIV_CLK_PERI_UART2				0x1818
#define CLK_CON_APB_ASYNC_DSIM_IPCLKPORT_PCLKS				0x2000
#define CLK_CON_PERI_I2C2_IPCLKPORT_I_PCLK				0x202c
#define CLK_CON_PERI_I2C3_IPCLKPORT_I_PCLK				0x2030
#define CLK_CON_PERI_SPI0_IPCLKPORT_I_PCLK				0x2054
#define CLK_CON_PERI_SPI0_IPCLKPORT_I_SCLK_SPI				0x2058
#define CLK_CON_PERI_UART1_IPCLKPORT_I_PCLK				0x205c
#define CLK_CON_PERI_UART1_IPCLKPORT_I_SCLK_UART			0x2060
#define CLK_CON_PERI_UART2_IPCLKPORT_I_PCLK				0x2064
#define CLK_CON_PERI_UART2_IPCLKPORT_I_SCLK_UART			0x2068
#define CLK_CON_DMYQCH_CON_DMA4DSIM_QCH					0x3000
#define CLK_CON_DMYQCH_CON_PERI_I3C2_QCH				0x3004
#define CLK_CON_DMYQCH_CON_PERI_I3C3_QCH				0x3008

static const unsigned long cmu_peri_clk_regs[] __initconst = {
	PLL_CON0_MUX_CLK_PERI_DISP_USER,
	PLL_CON0_MUX_CLK_PERI_IP_USER,
	CLK_CON_DIV_CLK_PERI_125,
	CLK_CON_DIV_CLK_PERI_PCLK,
	CLK_CON_DIV_CLK_PERI_SPI,
	CLK_CON_DIV_CLK_PERI_UART1,
	CLK_CON_DIV_CLK_PERI_UART2,
	CLK_CON_APB_ASYNC_DSIM_IPCLKPORT_PCLKS,
	CLK_CON_PERI_I2C2_IPCLKPORT_I_PCLK,
	CLK_CON_PERI_I2C3_IPCLKPORT_I_PCLK,
	CLK_CON_PERI_SPI0_IPCLKPORT_I_PCLK,
	CLK_CON_PERI_SPI0_IPCLKPORT_I_SCLK_SPI,
	CLK_CON_PERI_UART1_IPCLKPORT_I_PCLK,
	CLK_CON_PERI_UART1_IPCLKPORT_I_SCLK_UART,
	CLK_CON_PERI_UART2_IPCLKPORT_I_PCLK,
	CLK_CON_PERI_UART2_IPCLKPORT_I_SCLK_UART,
	CLK_CON_DMYQCH_CON_DMA4DSIM_QCH,
	CLK_CON_DMYQCH_CON_PERI_I3C2_QCH,
	CLK_CON_DMYQCH_CON_PERI_I3C3_QCH,
};

PNAME(mout_peri_ip_user_p) = { "fin_pll", "dout_clkcmu_peri_ip" };
PNAME(mout_peri_disp_user_p) = { "fin_pll", "dout_clkcmu_peri_disp" };

static const struct samsung_mux_clock cmu_peri_mux_clks[] __initconst = {
	MUX(CLK_MOUT_PERI_IP_USER, "mout_peri_ip_user", mout_peri_ip_user_p,
	    PLL_CON0_MUX_CLK_PERI_IP_USER, 4, 1),
	MUX(CLK_MOUT_PERI_DISP_USER, "mout_peri_disp_user", mout_peri_disp_user_p,
	    PLL_CON0_MUX_CLK_PERI_DISP_USER, 4, 1),
};

static const struct samsung_div_clock cmu_peri_div_clks[] __initconst = {
	DIV(CLK_DOUT_PERI_125, "dout_peri_125", "mout_peri_ip_user",
	    CLK_CON_DIV_CLK_PERI_125, 0, 4),
	DIV(CLK_DOUT_PERI_PCLK, "dout_peri_pclk", "mout_peri_ip_user",
	    CLK_CON_DIV_CLK_PERI_PCLK, 0, 4),
	DIV(CLK_DOUT_PERI_SPI, "dout_peri_spi", "mout_peri_ip_user",
	    CLK_CON_DIV_CLK_PERI_SPI, 0, 13),
	DIV(CLK_DOUT_PERI_UART1, "dout_peri_uart1", "mout_peri_ip_user",
	    CLK_CON_DIV_CLK_PERI_UART1, 0, 10),
	DIV(CLK_DOUT_PERI_UART2, "dout_peri_uart2", "mout_peri_ip_user",
	    CLK_CON_DIV_CLK_PERI_UART2, 0, 10),
};

static const struct samsung_gate_clock cmu_peri_gate_clks[] __initconst = {
	GATE(CLK_GOUT_PERI_DMA4DSIM_IPCLKPORT_CLK_APB_CLK, "dma4dsim_ipclkport_clk_apb_clk",
	     "dout_peri_pclk", CLK_CON_DMYQCH_CON_DMA4DSIM_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_DMA4DSIM_IPCLKPORT_CLK_AXI_CLK, "dma4dsim_ipclkport_clk_axi_clk",
	     "mout_peri_disp_user", CLK_CON_DMYQCH_CON_DMA4DSIM_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I3C2_IPCLKPORT_I_APB_S_PCLK, "peri_i3c2_ipclkport_i_apb_s_pclk",
	     "dout_peri_pclk", CLK_CON_DMYQCH_CON_PERI_I3C2_QCH, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_I3C2_IPCLKPORT_I_CORE_CLK, "peri_i3c2_ipclkport_i_core_clk",
	     "dout_peri_125", CLK_CON_DMYQCH_CON_PERI_I3C2_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I3C2_IPCLKPORT_I_DMA_CLK, "peri_i3c2_ipclkport_i_dma_clk",
	     "dout_peri_pclk", CLK_CON_DMYQCH_CON_PERI_I3C2_QCH, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_I3C2_IPCLKPORT_I_HDR_TX_CLK, "peri_i3c2_ipclkport_i_hdr_tx_clk",
	     "dout_peri_pclk", CLK_CON_DMYQCH_CON_PERI_I3C2_QCH, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_I3C3_IPCLKPORT_I_APB_S_PCLK, "peri_i3c3_ipclkport_i_apb_s_pclk",
	     "dout_peri_pclk", CLK_CON_DMYQCH_CON_PERI_I3C3_QCH, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_I3C3_IPCLKPORT_I_CORE_CLK, "peri_i3c3_ipclkport_i_core_clk",
	     "dout_peri_125", CLK_CON_DMYQCH_CON_PERI_I3C3_QCH, 1, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERI_I3C3_IPCLKPORT_I_DMA_CLK, "peri_i3c3_ipclkport_i_dma_clk",
	     "dout_peri_pclk", CLK_CON_DMYQCH_CON_PERI_I3C3_QCH, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_I3C3_IPCLKPORT_I_HDR_TX_CLK, "peri_i3c3_ipclkport_i_hdr_tx_clk",
	     "dout_peri_pclk", CLK_CON_DMYQCH_CON_PERI_I3C3_QCH, 1,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_APB_ASYNC_DSIM_IPCLKPORT_PCLKS, "apb_async_dsim_ipclkport_pclks",
	     "dout_peri_pclk", CLK_CON_APB_ASYNC_DSIM_IPCLKPORT_PCLKS, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_I2C2_IPCLKPORT_I_PCLK, "peri_i2c2_ipclkport_i_pclk",
	     "dout_peri_pclk", CLK_CON_PERI_I2C2_IPCLKPORT_I_PCLK, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_I2C3_IPCLKPORT_I_PCLK, "peri_i2c3_ipclkport_i_pclk",
	     "dout_peri_pclk", CLK_CON_PERI_I2C3_IPCLKPORT_I_PCLK, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_SPI0_PCLK, "peri_spi0_ipclkport_i_pclk",
	     "dout_peri_pclk", CLK_CON_PERI_SPI0_IPCLKPORT_I_PCLK, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_SPI0_SCLK_SPI, "peri_spi0_ipclkport_i_sclk_spi",
	     "dout_peri_spi", CLK_CON_PERI_SPI0_IPCLKPORT_I_SCLK_SPI, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_UART1_PCLK, "uart1", "dout_peri_pclk",
	     CLK_CON_PERI_UART1_IPCLKPORT_I_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_UART1_SCLK_UART, "clk_uart_baud1", "dout_peri_uart1",
	     CLK_CON_PERI_UART1_IPCLKPORT_I_SCLK_UART, 21,
	     CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_UART2_PCLK, "uart2", "dout_peri_pclk",
	     CLK_CON_PERI_UART2_IPCLKPORT_I_PCLK, 21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERI_UART2_SCLK_UART, "clk_uart_baud2", "dout_peri_uart2",
	     CLK_CON_PERI_UART2_IPCLKPORT_I_SCLK_UART,
	     21, CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0),
};

static const struct samsung_cmu_info cmu_peri_info __initconst = {
	.mux_clks		= cmu_peri_mux_clks,
	.nr_mux_clks		= ARRAY_SIZE(cmu_peri_mux_clks),
	.div_clks		= cmu_peri_div_clks,
	.nr_div_clks		= ARRAY_SIZE(cmu_peri_div_clks),
	.gate_clks              = cmu_peri_gate_clks,
	.nr_gate_clks           = ARRAY_SIZE(cmu_peri_gate_clks),
	.nr_clk_ids		= CMU_PERI_NR_CLK,
	.clk_regs		= cmu_peri_clk_regs,
	.nr_clk_regs		= ARRAY_SIZE(cmu_peri_clk_regs),
};

static int __init artpec9_cmu_probe(struct platform_device *pdev)
{
	const struct samsung_cmu_info *info;
	struct device *dev = &pdev->dev;

	info = of_device_get_match_data(dev);
	exynos_arm64_register_cmu(dev, dev->of_node, info);

	return 0;
}

static const struct of_device_id artpec9_cmu_of_match[] = {
	{
		.compatible = "axis,artpec9-cmu-cmu",
		.data = &cmu_cmu_info,
	}, {
		.compatible = "axis,artpec9-cmu-bus",
		.data = &cmu_bus_info,
	}, {
		.compatible = "axis,artpec9-cmu-core",
		.data = &cmu_core_info,
	}, {
		.compatible = "axis,artpec9-cmu-cpucl",
		.data = &cmu_cpucl_info,
	}, {
		.compatible = "axis,artpec9-cmu-fsys0",
		.data = &cmu_fsys0_info,
	}, {
		.compatible = "axis,artpec9-cmu-fsys1",
		.data = &cmu_fsys1_info,
	}, {
		.compatible = "axis,artpec9-cmu-peri",
		.data = &cmu_peri_info,
	}, {
	},
};

static struct platform_driver artpec9_cmu_driver __refdata = {
	.driver = {
		.name = "artpec9-cmu",
		.of_match_table = artpec9_cmu_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = artpec9_cmu_probe,
};

static int __init artpec9_cmu_init(void)
{
	return platform_driver_register(&artpec9_cmu_driver);
}
core_initcall(artpec9_cmu_init);
