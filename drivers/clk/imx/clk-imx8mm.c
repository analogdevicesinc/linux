/*
 * Copyright 2017-2018 NXP.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <dt-bindings/clock/imx8mm-clock.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <soc/imx8/soc.h>

#include "clk.h"

static u32 share_count_sai1;
static u32 share_count_sai2;
static u32 share_count_sai3;
static u32 share_count_sai4;
static u32 share_count_sai5;
static u32 share_count_sai6;
static u32 share_count_dcss;
static u32 share_count_pdm;

/* IDs of PLLs available on i.MX8 Mini */
enum {
	ARM_PLL,
	GPU_PLL,
	VPU_PLL,
	SYS_PLL1,
	SYS_PLL2,
	SYS_PLL3,
	DRAM_PLL,
	AUDIO_PLL1,
	AUDIO_PLL2,
	VIDEO_PLL2,
	NR_PLLS,
};

#define PLL_1416X_RATE(_rate, _m, _p, _s)		\
	{						\
		.rate	=	(_rate),		\
		.mdiv	=	(_m),			\
		.pdiv	=	(_p),			\
		.sdiv	=	(_s),			\
	}

#define PLL_1443X_RATE(_rate, _m, _p, _s, _k)		\
	{						\
		.rate	=	(_rate),		\
		.mdiv	=	(_m),			\
		.pdiv	=	(_p),			\
		.sdiv	=	(_s),			\
		.kdiv	=	(_k),			\
	}

static const struct imx_int_pll_rate_table imx8mm_intpll_tbl[] = {
	PLL_1416X_RATE(1800000000U, 225, 3, 0),
	PLL_1416X_RATE(1600000000U, 200, 3, 0),
	PLL_1416X_RATE(1200000000U, 300, 3, 1),
	PLL_1416X_RATE(1000000000U, 250, 3, 1),
	PLL_1416X_RATE(800000000U,  200, 3, 1),
	PLL_1416X_RATE(750000000U,  250, 2, 2),
	PLL_1416X_RATE(700000000U,  350, 3, 2),
	PLL_1416X_RATE(600000000U,  300, 3, 2),
};

static const struct imx_int_pll_rate_table imx8mm_audiopll_tbl[] = {
	PLL_1443X_RATE(786432000U, 655, 5, 2, 23593),
	PLL_1443X_RATE(722534400U, 301, 5, 1, 3670),
};

static const struct imx_int_pll_rate_table imx8mm_videopll_tbl[] = {
	PLL_1443X_RATE(650000000U, 325, 3, 2, 0),
	PLL_1443X_RATE(594000000U, 198, 2, 2, 0),
};

static const struct imx_int_pll_rate_table imx8mm_drampll_tbl[] = {
	PLL_1443X_RATE(650000000U, 325, 3, 2, 0),
};

static struct imx_int_pll_clk imx8mm_audio_pll __initdata = {
		.type = PLL_1443X,
		.rate_table = imx8mm_audiopll_tbl,
};

static struct imx_int_pll_clk imx8mm_video_pll __initdata = {
		.type = PLL_1443X,
		.rate_table = imx8mm_videopll_tbl,
};

static struct imx_int_pll_clk imx8mm_dram_pll __initdata = {
		.type = PLL_1443X,
		.rate_table = imx8mm_drampll_tbl,
};

static struct imx_int_pll_clk imx8mm_arm_pll __initdata = {
		.type = PLL_1416X,
		.rate_table = imx8mm_intpll_tbl,
};

static struct imx_int_pll_clk imx8mm_gpu_pll __initdata = {
		.type = PLL_1416X,
		.rate_table = imx8mm_intpll_tbl,
};

static struct imx_int_pll_clk imx8mm_vpu_pll __initdata = {
		.type = PLL_1416X,
		.rate_table = imx8mm_intpll_tbl,
};

static struct imx_int_pll_clk imx8mm_sys_pll __initdata = {
		.type = PLL_1416X,
		.rate_table = imx8mm_intpll_tbl,
};

static const char *pll_ref_sels[] = { "osc_24m", "dummy", "dummy", "dummy", };
static const char *audio_pll1_bypass_sels[] = {"audio_pll1", "audio_pll1_ref_sel", };
static const char *audio_pll2_bypass_sels[] = {"audio_pll2", "audio_pll2_ref_sel", };
static const char *video_pll1_bypass_sels[] = {"video_pll1", "video_pll1_ref_sel", };
static const char *dram_pll_bypass_sels[] = {"dram_pll", "dram_pll_ref_sel", };
static const char *gpu_pll_bypass_sels[] = {"gpu_pll", "gpu_pll_ref_sel", };
static const char *vpu_pll_bypass_sels[] = {"vpu_pll", "vpu_pll_ref_sel", };
static const char *arm_pll_bypass_sels[] = {"arm_pll", "arm_pll_ref_sel", };
static const char *sys_pll1_bypass_sels[] = {"sys_pll1", "sys_pll1_ref_sel", };
static const char *sys_pll2_bypass_sels[] = {"sys_pll2", "sys_pll2_ref_sel", };
static const char *sys_pll3_bypass_sels[] = {"sys_pll3", "sys_pll3_ref_sel", };

/* CCM ROOT */
static const char *imx8mm_a53_sels[] = {"osc_24m", "arm_pll_out", "sys_pll2_500m", "sys_pll2_1000m",
					"sys_pll1_800m", "sys_pll1_400m", "audio_pll1_out", "sys_pll3_out", };

static const char *imx8mm_m4_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll2_250m", "sys_pll1_266m",
				       "sys_pll1_800m", "audio_pll1_out", "video_pll1_out", "sys_pll3_out", };

static const char *imx8mm_vpu_sels[] = {"osc_24m", "arm_pll_out", "sys_pll2_500m", "sys_pll2_1000m",
					"sys_pll1_800m", "sys_pll1_400m", "audio_pll1_out", "vpu_pll_out", };

static const char *imx8mm_gpu3d_sels[] = {"osc_24m", "gpu_pll_out", "sys_pll1_800m", "sys_pll3_out",
					  "sys_pll2_1000m", "audio_pll1_out", "video_pll1_out", "audio_pll2_out", };

static const char *imx8mm_gpu2d_sels[] = {"osc_24m", "gpu_pll_out", "sys_pll1_800m", "sys_pll3_out",
					  "sys_pll2_1000m", "audio_pll1_out", "video_pll1_out", "audio_pll2_out", };

static const char *imx8mm_main_axi_sels[] = {"osc_24m", "sys_pll2_333m", "sys_pll1_800m", "sys_pll2_250m",
					     "sys_pll2_1000m", "audio_pll1_out", "video_pll1_out", "sys_pll1_100m",};

static const char *imx8mm_enet_axi_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll1_800m", "sys_pll2_250m",
					     "sys_pll2_200m", "audio_pll1_out", "video_pll1_out", "sys_pll3_out", };

static const char *imx8mm_nand_usdhc_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll1_800m", "sys_pll2_200m",
					       "sys_pll1_133m", "sys_pll3_out", "sys_pll2_250m", "audio_pll1_out", };

static const char *imx8mm_vpu_bus_sels[] = {"osc_24m", "sys_pll1_800m", "vpu_pll_out", "audio_pll2_out",
					    "sys_pll3_out", "sys_pll2_1000m", "sys_pll2_200m", "sys_pll1_100m", };

static const char *imx8mm_disp_axi_sels[] = {"osc_24m", "sys_pll2_1000m", "sys_pll1_800m", "sys_pll3_out",
					     "sys_pll1_40m", "audio_pll2_out", "clk_ext1", "clk_ext4", };

static const char *imx8mm_disp_apb_sels[] = {"osc_24m", "sys_pll2_125m", "sys_pll1_800m", "sys_pll3_out",
					     "sys1_pll_40m", "audio_pll2_out", "clk_ext1", "clk_ext3", };

static const char *imx8mm_disp_rtrm_sels[] = {"osc_24m", "sys_pll1_800m", "sys_pll2_200m", "sys_pll2_1000m",
					      "audio_pll1_out", "video_pll1_out", "clk_ext2", "clk_ext3", };

static const char *imx8mm_usb_bus_sels[] = {"osc_24m", "sys_pll2_500m", "sys_pll1_800m", "sys_pll2_100m",
					    "sys_pll2_200m", "clk_ext2", "clk_ext4", "audio_pll2_out", };

static const char *imx8mm_gpu_axi_sels[] = {"osc_24m", "sys_pll1_800m", "gpu_pll_out", "sys_pll3_out", "sys_pll2_1000m",
					    "audio_pll1_out", "video_pll1_out", "audio_pll2_out", };

static const char *imx8mm_gpu_ahb_sels[] = {"osc_24m", "sys_pll1_800m", "gpu_pll_out", "sys_pll3_out", "sys_pll2_1000m",
					    "audio_pll1_out", "video_pll1_out", "audio_pll2_out", };

static const char *imx8mm_noc_sels[] = {"osc_24m", "sys_pll1_800m", "sys_pll3_out", "sys_pll2_1000m", "sys_pll2_500m",
					"audio_pll1_out", "video_pll1_out", "audio_pll2_out", };

static const char *imx8mm_noc_apb_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll3_out", "sys_pll2_333m", "sys_pll2_200m",
					    "sys_pll1_800m", "audio_pll1_out", "video_pll1_out", };

static const char *imx8mm_ahb_sels[] = {"osc_24m", "sys_pll1_133m", "sys_pll1_800m", "sys_pll1_400m",
					"sys_pll2_125m", "sys_pll3_out", "audio_pll1_out", "video_pll1_out", };

static const char *imx8mm_audio_ahb_sels[] = {"osc_24m", "sys_pll2_500m", "sys_pll1_800m", "sys_pll2_1000m",
					      "sys_pll2_166m", "sys_pll3_out", "audio_pll1_out", "video_pll1_out", };

static const char *imx8mm_dram_alt_sels[] = {"osc_24m", "sys_pll1_800m", "sys_pll1_100m", "sys_pll2_500m",
					     "sys_pll2_1000m", "sys_pll3_out", "audio_pll1_out", "sys_pll1_266m", };

static const char *imx8mm_dram_apb_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m", "sys_pll1_160m",
					     "sys_pll1_800m", "sys_pll3_out", "sys_pll2_250m", "audio_pll2_out", };

static const char *imx8mm_vpu_g1_sels[] = {"osc_24m", "vpu_pll_out", "sys_pll1_800m", "sys_pll2_1000m",
					   "sys_pll1_100m", "sys_pll2_125m", "sys_pll3_out", "audio_pll1_out", };

static const char *imx8mm_vpu_g2_sels[] = {"osc_24m", "vpu_pll_out", "sys_pll1_800m", "sys_pll2_1000m",
					   "sys_pll1_100m", "sys_pll2_125m", "sys_pll3_out", "audio_pll1_out", };

static const char *imx8mm_disp_dtrc_sels[] = {"osc_24m", "video_pll2_out", "sys_pll1_800m", "sys_pll2_1000m",
					      "sys_pll1_160m", "video_pll1_out", "sys_pll3_out", "audio_pll2_out", };

static const char *imx8mm_disp_dc8000_sels[] = {"osc_24m", "video_pll2_out", "sys_pll1_800m", "sys_pll2_1000m",
						"sys_pll1_160m", "video_pll1_out", "sys_pll3_out", "audio_pll2_out", };

static const char *imx8mm_pcie1_ctrl_sels[] = {"osc_24m", "sys_pll2_250m", "sys_pll2_200m", "sys_pll1_266m",
					       "sys_pll1_800m", "sys_pll2_500m", "sys_pll2_333m", "sys_pll3_out", };

static const char *imx8mm_pcie1_phy_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll2_500m", "clk_ext1", "clk_ext2",
					      "clk_ext3", "clk_ext4", "sys_pll1_400m", };

static const char *imx8mm_pcie1_aux_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll2_50m", "sys_pll3_out",
					      "sys_pll2_100m", "sys_pll1_80m", "sys_pll1_160m", "sys_pll1_200m", };

static const char *imx8mm_dc_pixel_sels[] = {"osc_24m", "video_pll1_out", "audio_pll2_out", "audio_pll1_out",
					     "sys_pll1_800m", "sys_pll2_1000m", "sys_pll3_out", "clk_ext4", };

static const char *imx8mm_lcdif_pixel_sels[] = {"osc_24m", "video_pll1_out", "audio_pll2_out", "audio_pll1_out",
						"sys_pll1_800m", "sys_pll2_1000m", "sys_pll3_out", "clk_ext4", };

static const char *imx8mm_sai1_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					 "sys_pll1_133m", "osc_hdmi", "clk_ext1", "clk_ext2", };

static const char *imx8mm_sai2_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					 "sys_pll1_133m", "osc_hdmi", "clk_ext2", "clk_ext3", };

static const char *imx8mm_sai3_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					 "sys_pll1_133m", "osc_hdmi", "clk_ext3", "clk_ext4", };

static const char *imx8mm_sai4_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					 "sys_pll1_133m", "osc_hdmi", "clk_ext1", "clk_ext2", };

static const char *imx8mm_sai5_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					 "sys_pll1_133m", "osc_hdmi", "clk_ext2", "clk_ext3", };

static const char *imx8mm_sai6_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					 "sys_pll1_133m", "osc_hdmi", "clk_ext3", "clk_ext4", };

static const char *imx8mm_spdif1_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					   "sys_pll1_133m", "osc_hdmi", "clk_ext2", "clk_ext3", };

static const char *imx8mm_spdif2_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out", "video_pll1_out",
					   "sys_pll1_133m", "osc_hdmi", "clk_ext3", "clk_ext4", };

static const char *imx8mm_enet_ref_sels[] = {"osc_24m", "sys_pll2_125m", "sys_pll2_50m", "sys_pll2_100m",
					     "sys_pll1_160m", "audio_pll1_out", "video_pll1_out", "clk_ext4", };

static const char *imx8mm_enet_timer_sels[] = {"osc_24m", "sys_pll2_100m", "audio_pll1_out", "clk_ext1", "clk_ext2",
					       "clk_ext3", "clk_ext4", "video_pll1_out", };

static const char *imx8mm_enet_phy_sels[] = {"osc_24m", "sys_pll2_50m", "sys_pll2_125m", "sys_pll2_200m",
					     "sys_pll2_500m", "video_pll1_out", "audio_pll2_out", };

static const char *imx8mm_nand_sels[] = {"osc_24m", "sys_pll2_500m", "audio_pll1_out", "sys_pll1_400m",
					 "audio_pll2_out", "sys_pll3_out", "sys_pll2_250m", "video_pll1_out", };

static const char *imx8mm_qspi_sels[] = {"osc_24m", "sys1_pll_400m", "sys_pll1_800m", "sys2_pll_500m",
					 "audio_pll2_out", "sys1_pll_266m", "sys3_pll2_out", "sys1_pll_100m", };

static const char *imx8mm_usdhc1_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll1_800m", "sys_pll2_500m",
					   "sys_pll3_out", "sys_pll1_266m", "audio_pll2_out", "sys_pll1_100m", };

static const char *imx8mm_usdhc2_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll1_800m", "sys_pll2_500m",
					   "sys_pll3_out", "sys_pll1_266m", "audio_pll2_out", "sys_pll1_100m", };

static const char *imx8mm_i2c1_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m", "sys_pll3_out", "audio_pll1_out",
					 "video_pll1_out", "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mm_i2c2_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m", "sys_pll3_out", "audio_pll1_out",
					 "video_pll1_out", "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mm_i2c3_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m", "sys_pll3_out", "audio_pll1_out",
					 "video_pll1_out", "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mm_i2c4_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m", "sys_pll3_out", "audio_pll1_out",
					 "video_pll1_out", "audio_pll2_out", "sys_pll1_133m", };

static const char *imx8mm_uart1_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m", "sys_pll2_100m",
					  "sys_pll3_out", "clk_ext2", "clk_ext4", "audio_pll2_out", };

static const char *imx8mm_uart2_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m", "sys_pll2_100m",
					  "sys_pll3_out", "clk_ext2", "clk_ext3", "audio_pll2_out", };

static const char *imx8mm_uart3_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m", "sys_pll2_100m",
					  "sys_pll3_out", "clk_ext2", "clk_ext4", "audio_pll2_out", };

static const char *imx8mm_uart4_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m", "sys_pll2_100m",
					  "sys_pll3_out", "clk_ext2", "clk_ext3", "audio_pll2_out", };

static const char *imx8mm_usb_core_sels[] = {"osc_24m", "sys_pll1_100m", "sys_pll1_40m", "sys_pll2_100m",
					     "sys_pll2_200m", "clk_ext2", "clk_ext3", "audio_pll2_out", };

static const char *imx8mm_usb_phy_sels[] = {"osc_24m", "sys_pll1_100m", "sys_pll1_40m", "sys_pll2_100m",
					     "sys_pll2_200m", "clk_ext2", "clk_ext3", "audio_pll2_out", };

static const char *imx8mm_ecspi1_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m", "sys_pll1_160m",
					   "sys_pll1_800m", "sys_pll3_out", "sys_pll2_250m", "audio_pll2_out", };

static const char *imx8mm_ecspi2_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m", "sys_pll1_160m",
					   "sys_pll1_800m", "sys_pll3_out", "sys_pll2_250m", "audio_pll2_out", };

static const char *imx8mm_pwm1_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m", "sys_pll1_40m",
					 "sys_pll3_out", "clk_ext1", "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mm_pwm2_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m", "sys_pll1_40m",
					 "sys_pll3_out", "clk_ext1", "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mm_pwm3_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m", "sys_pll1_40m",
					 "sys3_pll2_out", "clk_ext2", "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mm_pwm4_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m", "sys_pll1_40m",
					 "sys_pll3_out", "clk_ext2", "sys_pll1_80m", "video_pll1_out", };

static const char *imx8mm_gpt1_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_400m", "sys_pll1_40m",
					 "video_pll1_out", "sys_pll1_800m", "audio_pll1_out", "clk_ext1" };

static const char *imx8mm_wdog_sels[] = {"osc_24m", "sys_pll1_133m", "sys_pll1_160m", "vpu_pll_out",
					 "sys_pll2_125m", "sys_pll3_out", "sys_pll1_80m", "sys_pll2_166m", };

static const char *imx8mm_wrclk_sels[] = {"osc_24m", "sys_pll1_40m", "vpu_pll_out", "sys_pll3_out", "sys_pll2_200m",
					  "sys_pll1_266m", "sys_pll2_500m", "sys_pll1_100m", };

static const char *imx8mm_dsi_core_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll2_250m", "sys_pll1_800m",
					     "sys_pll2_1000m", "sys_pll3_out", "audio_pll2_out", "video_pll1_out", };

static const char *imx8mm_dsi_phy_sels[] = {"osc_24m", "sys_pll2_125m", "sys_pll2_100m", "sys_pll1_800m",
					    "sys_pll2_1000m", "clk_ext2", "audio_pll2_out", "video_pll1_out", };

static const char *imx8mm_dsi_dbi_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll2_100m", "sys_pll1_800m",
					    "sys_pll2_1000m", "sys_pll3_out", "audio_pll2_out", "video_pll1_out", };

static const char *imx8mm_usdhc3_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll1_800m", "sys_pll2_500m",
					   "sys_pll3_out", "sys_pll1_266m", "audio_pll2_clk", "sys_pll1_100m", };

static const char *imx8mm_csi1_core_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll2_250m", "sys_pll1_800m",
					      "sys_pll2_1000m", "sys_pll3_out", "audio_pll2_out", "video_pll1_out", };

static const char *imx8mm_csi1_phy_sels[] = {"osc_24m", "sys_pll2_333m", "sys_pll2_100m", "sys_pll1_800m",
					     "sys_pll2_1000m", "clk_ext2", "audio_pll2_out", "video_pll1_out", };

static const char *imx8mm_csi1_esc_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_80m", "sys_pll1_800m",
					     "sys_pll2_1000m", "sys_pll3_out", "clk_ext3", "audio_pll2_out", };

static const char *imx8mm_csi2_core_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll2_250m", "sys_pll1_800m",
					      "sys_pll2_1000m", "sys_pll3_out", "audio_pll2_out", "video_pll1_out", };

static const char *imx8mm_csi2_phy_sels[] = {"osc_24m", "sys_pll2_333m", "sys_pll2_100m", "sys_pll1_800m",
					     "sys_pll2_1000m", "clk_ext2", "audio_pll2_out", "video_pll1_out", };

static const char *imx8mm_csi2_esc_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_80m", "sys_pll1_800m",
					     "sys_pll2_1000m", "sys_pll3_out", "clk_ext3", "audio_pll2_out", };

static const char *imx8mm_pcie2_ctrl_sels[] = {"osc_24m", "sys_pll2_250m", "sys_pll2_200m", "sys_pll1_266m",
					       "sys_pll1_800m", "sys_pll2_500m", "sys_pll2_333m", "sys_pll3_out", };

static const char *imx8mm_pcie2_phy_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll2_500m", "clk_ext1",
					      "clk_ext2", "clk_ext3", "clk_ext4", "sys_pll1_400m", };

static const char *imx8mm_pcie2_aux_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll2_50m", "sys_pll3_out",
					      "sys_pll2_100m", "sys_pll1_80m", "sys_pll1_160m", "sys_pll1_200m", };

static const char *imx8mm_ecspi3_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m", "sys_pll1_160m",
					   "sys_pll1_800m", "sys_pll3_out", "sys_pll2_250m", "audio_pll2_out", };

static const char *imx8mm_pdm_sels[] = {"osc_24m", "sys_pll2_100m", "audio_pll1_out", "sys_pll1_800m",
					"sys_pll2_1000m", "sys_pll3_out", "clk_ext3", "audio_pll2_out", };

static const char *imx8mm_vpu_h1_sels[] = {"osc_24m", "vpu_pll_out", "sys_pll1_800m", "sys_pll2_1000m",
					   "audio_pll2_clk", "sys_pll2_125m", "sys_pll3_clk", "audio_pll1_out", };

static int const clks_init_on[] __initconst = {
	IMX8MM_CLK_DRAM_ALT_DIV, IMX8MM_CLK_AHB_CG,
	IMX8MM_CLK_NOC_CG, IMX8MM_CLK_NOC_APB_CG,
	IMX8MM_CLK_USB_BUS_CG, IMX8MM_CLK_NAND_USDHC_BUS_CG,
	IMX8MM_CLK_MAIN_AXI_CG, IMX8MM_CLK_AUDIO_AHB_CG,
	IMX8MM_CLK_DRAM_APB_DIV, IMX8MM_CLK_A53_DIV,
	IMX8MM_DRAM_PLL_OUT, IMX8MM_ARM_PLL_OUT,
};

static struct clk *clks[IMX8MM_CLK_END];
static struct clk_onecell_data clk_data;

static void __init imx8mm_clocks_init(struct device_node *ccm_node)
{
	struct device_node *np;
	void __iomem *base;
	int i;

	check_m4_enabled();

	clks[IMX8MM_CLK_DUMMY] = imx_clk_fixed("dummy", 0);
	clks[IMX8MM_CLK_24M] = of_clk_get_by_name(ccm_node, "osc_24m");
	clks[IMX8MM_CLK_32K] = of_clk_get_by_name(ccm_node, "osc_32k"); /* Check more */
	clks[IMX8MM_CLK_EXT1] = of_clk_get_by_name(ccm_node, "clk_ext1");
	clks[IMX8MM_CLK_EXT2] = of_clk_get_by_name(ccm_node, "clk_ext2");
	clks[IMX8MM_CLK_EXT3] = of_clk_get_by_name(ccm_node, "clk_ext3");
	clks[IMX8MM_CLK_EXT4] = of_clk_get_by_name(ccm_node, "clk_ext4");

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mm-anatop");
	base = of_iomap(np, 0);
	WARN_ON(!base);

	clks[IMX8MM_AUDIO_PLL1_REF_SEL] = imx_clk_mux("audio_pll1_ref_sel", base + 0x0, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_AUDIO_PLL2_REF_SEL] = imx_clk_mux("audio_pll2_ref_sel", base + 0x14, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_VIDEO_PLL1_REF_SEL] = imx_clk_mux("video_pll1_ref_sel", base + 0x28, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_DRAM_PLL_REF_SEL] = imx_clk_mux("dram_pll_ref_sel", base + 0x50, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_GPU_PLL_REF_SEL] = imx_clk_mux("gpu_pll_ref_sel", base + 0x64, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_VPU_PLL_REF_SEL] = imx_clk_mux("vpu_pll_ref_sel", base + 0x74, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_ARM_PLL_REF_SEL] = imx_clk_mux("arm_pll_ref_sel", base + 0x84, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_SYS_PLL1_REF_SEL] = imx_clk_mux("sys_pll1_ref_sel", base + 0x94, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_SYS_PLL2_REF_SEL] = imx_clk_mux("sys_pll2_ref_sel", base + 0x104, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	clks[IMX8MM_SYS_PLL3_REF_SEL] = imx_clk_mux("sys_pll3_ref_sel", base + 0x114, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));

	clks[IMX8MM_AUDIO_PLL1] = imx_clk_int_pll("audio_pll1", "audio_pll1_ref_sel", base, &imx8mm_audio_pll);
	clks[IMX8MM_AUDIO_PLL2] = imx_clk_int_pll("audio_pll2", "audio_pll2_ref_sel", base + 0x14, &imx8mm_audio_pll);
	clks[IMX8MM_VIDEO_PLL1] = imx_clk_int_pll("video_pll1", "video_pll1_ref_sel", base + 0x28, &imx8mm_video_pll);
	clks[IMX8MM_DRAM_PLL] = imx_clk_int_pll("dram_pll", "dram_pll_ref_sel", base + 0x50, &imx8mm_dram_pll);
	clks[IMX8MM_GPU_PLL] = imx_clk_int_pll("gpu_pll", "gpu_pll_ref_sel", base + 0x64, &imx8mm_gpu_pll);
	clks[IMX8MM_VPU_PLL] = imx_clk_int_pll("vpu_pll", "vpu_pll_ref_sel", base + 0x74, &imx8mm_vpu_pll);
	clks[IMX8MM_ARM_PLL] = imx_clk_int_pll("arm_pll", "arm_pll_ref_sel", base + 0x84, &imx8mm_arm_pll);
	clks[IMX8MM_SYS_PLL1] = imx_clk_int_pll("sys_pll1", "sys_pll1_ref_sel", base + 0x94, &imx8mm_sys_pll);
	clks[IMX8MM_SYS_PLL2] = imx_clk_int_pll("sys_pll2", "sys_pll2_ref_sel", base + 0x104, &imx8mm_sys_pll);
	clks[IMX8MM_SYS_PLL3] = imx_clk_int_pll("sys_pll3", "sys_pll3_ref_sel", base + 0x114, &imx8mm_sys_pll);

	/* PLL bypass out */
	clks[IMX8MM_AUDIO_PLL1_BYPASS] = imx_clk_mux_flags("audio_pll1_bypass", base, 4, 1, audio_pll1_bypass_sels, ARRAY_SIZE(audio_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_AUDIO_PLL2_BYPASS] = imx_clk_mux_flags("audio_pll2_bypass", base + 0x14, 4, 1, audio_pll2_bypass_sels, ARRAY_SIZE(audio_pll2_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_VIDEO_PLL1_BYPASS] = imx_clk_mux_flags("video_pll1_bypass", base + 0x28, 4, 1, video_pll1_bypass_sels, ARRAY_SIZE(video_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_DRAM_PLL_BYPASS] = imx_clk_mux_flags("dram_pll_bypass", base + 0x50, 4, 1, dram_pll_bypass_sels, ARRAY_SIZE(dram_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_GPU_PLL_BYPASS] = imx_clk_mux_flags("gpu_pll_bypass", base + 0x64, 4, 1, gpu_pll_bypass_sels, ARRAY_SIZE(gpu_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_VPU_PLL_BYPASS] = imx_clk_mux_flags("vpu_pll_bypass", base + 0x74, 4, 1, vpu_pll_bypass_sels, ARRAY_SIZE(vpu_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_ARM_PLL_BYPASS] = imx_clk_mux_flags("arm_pll_bypass", base + 0x84, 4, 1, arm_pll_bypass_sels, ARRAY_SIZE(arm_pll_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_SYS_PLL1_BYPASS] = imx_clk_mux_flags("sys_pll1_bypass", base + 0x94, 4, 1, sys_pll1_bypass_sels, ARRAY_SIZE(sys_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_SYS_PLL2_BYPASS] = imx_clk_mux_flags("sys_pll2_bypass", base + 0x104, 4, 1, sys_pll2_bypass_sels, ARRAY_SIZE(sys_pll2_bypass_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MM_SYS_PLL3_BYPASS] = imx_clk_mux_flags("sys_pll3_bypass", base + 0x114, 4, 1, sys_pll3_bypass_sels, ARRAY_SIZE(sys_pll3_bypass_sels), CLK_SET_RATE_PARENT);

	/* unbypass all the plls */
	clk_set_parent(clks[IMX8MM_AUDIO_PLL1_BYPASS], clks[IMX8MM_AUDIO_PLL1]);
	clk_set_parent(clks[IMX8MM_AUDIO_PLL2_BYPASS], clks[IMX8MM_AUDIO_PLL2]);
	clk_set_parent(clks[IMX8MM_VIDEO_PLL1_BYPASS], clks[IMX8MM_VIDEO_PLL1]);
	clk_set_parent(clks[IMX8MM_DRAM_PLL_BYPASS], clks[IMX8MM_DRAM_PLL]);
	clk_set_parent(clks[IMX8MM_GPU_PLL_BYPASS], clks[IMX8MM_GPU_PLL]);
	clk_set_parent(clks[IMX8MM_VPU_PLL_BYPASS], clks[IMX8MM_VPU_PLL]);
	clk_set_parent(clks[IMX8MM_ARM_PLL_BYPASS], clks[IMX8MM_ARM_PLL]);
	clk_set_parent(clks[IMX8MM_SYS_PLL1_BYPASS], clks[IMX8MM_SYS_PLL1]);
	clk_set_parent(clks[IMX8MM_SYS_PLL2_BYPASS], clks[IMX8MM_SYS_PLL2]);
	clk_set_parent(clks[IMX8MM_SYS_PLL3_BYPASS], clks[IMX8MM_SYS_PLL3]);

	/* PLL out gate */
	clks[IMX8MM_AUDIO_PLL1_OUT] = imx_clk_gate("audio_pll1_out", "audio_pll1_bypass", base, 13);
	clks[IMX8MM_AUDIO_PLL2_OUT] = imx_clk_gate("audio_pll2_out", "audio_pll2_bypass", base + 0x14, 13);
	clks[IMX8MM_VIDEO_PLL1_OUT] = imx_clk_gate("video_pll1_out", "video_pll1_bypass", base + 0x28, 13);
	clks[IMX8MM_DRAM_PLL_OUT] = imx_clk_gate("dram_pll_out", "dram_pll_bypass", base + 0x50, 13);
	clks[IMX8MM_GPU_PLL_OUT] = imx_clk_gate("gpu_pll_out", "gpu_pll_bypass", base + 0x64, 13);
	clks[IMX8MM_VPU_PLL_OUT] = imx_clk_gate("vpu_pll_out", "vpu_pll_bypass", base + 0x74, 13);
	clks[IMX8MM_ARM_PLL_OUT] = imx_clk_gate("arm_pll_out", "arm_pll_bypass", base + 0x84, 13);
	clks[IMX8MM_SYS_PLL1_OUT] = imx_clk_gate("sys_pll1_out", "sys_pll1_bypass", base + 0x94, 13);
	clks[IMX8MM_SYS_PLL2_OUT] = imx_clk_gate("sys_pll2_out", "sys_pll2_bypass", base + 0x104, 13);
	clks[IMX8MM_SYS_PLL3_OUT] = imx_clk_gate("sys_pll3_out", "sys_pll3_bypass", base + 0x114, 13);

	/* SYS PLL fixed output */
	clks[IMX8MM_SYS_PLL1_40M] = imx_clk_fixed_factor("sys_pll1_40m", "sys_pll1_out", 1, 20);
	clks[IMX8MM_SYS_PLL1_80M] = imx_clk_fixed_factor("sys_pll1_80m", "sys_pll1_out", 1, 10);
	clks[IMX8MM_SYS_PLL1_100M] = imx_clk_fixed_factor("sys_pll1_100m", "sys_pll1_out", 1, 8);
	clks[IMX8MM_SYS_PLL1_133M] = imx_clk_fixed_factor("sys_pll1_133m", "sys_pll1_out", 1, 6);
	clks[IMX8MM_SYS_PLL1_160M] = imx_clk_fixed_factor("sys_pll1_160m", "sys_pll1_out", 1, 5);
	clks[IMX8MM_SYS_PLL1_200M] = imx_clk_fixed_factor("sys_pll1_200m", "sys_pll1_out", 1, 4);
	clks[IMX8MM_SYS_PLL1_266M] = imx_clk_fixed_factor("sys_pll1_266m", "sys_pll1_out", 1, 3);
	clks[IMX8MM_SYS_PLL1_400M] = imx_clk_fixed_factor("sys_pll1_400m", "sys_pll1_out", 1, 2);
	clks[IMX8MM_SYS_PLL1_800M] = imx_clk_fixed_factor("sys_pll1_800m", "sys_pll1_out", 1, 1);

	clks[IMX8MM_SYS_PLL2_50M] = imx_clk_fixed_factor("sys_pll2_50m", "sys_pll2_out", 1, 20);
	clks[IMX8MM_SYS_PLL2_100M] = imx_clk_fixed_factor("sys_pll2_100m", "sys_pll2_out", 1, 10);
	clks[IMX8MM_SYS_PLL2_125M] = imx_clk_fixed_factor("sys_pll2_125m", "sys_pll2_out", 1, 8);
	clks[IMX8MM_SYS_PLL2_166M] = imx_clk_fixed_factor("sys_pll2_166m", "sys_pll2_out", 1, 6);
	clks[IMX8MM_SYS_PLL2_200M] = imx_clk_fixed_factor("sys_pll2_200m", "sys_pll2_out", 1, 5);
	clks[IMX8MM_SYS_PLL2_250M] = imx_clk_fixed_factor("sys_pll2_250m", "sys_pll2_out", 1, 4);
	clks[IMX8MM_SYS_PLL2_333M] = imx_clk_fixed_factor("sys_pll2_333m", "sys_pll2_out", 1, 3);
	clks[IMX8MM_SYS_PLL2_500M] = imx_clk_fixed_factor("sys_pll2_500m", "sys_pll2_out", 1, 2);
	clks[IMX8MM_SYS_PLL2_1000M] = imx_clk_fixed_factor("sys_pll2_1000m", "sys_pll2_out", 1, 1);

	np = ccm_node;
	base = of_iomap(np, 0);
	WARN_ON(!base);

	/* Core Slice */
	clks[IMX8MM_CLK_A53_SRC] = imx_clk_mux2("arm_a53_src", base + 0x8000, 24, 3, imx8mm_a53_sels, ARRAY_SIZE(imx8mm_a53_sels));
	clks[IMX8MM_CLK_M4_SRC] = imx_clk_mux2("arm_m4_src", base + 0x8080, 24, 3, imx8mm_m4_sels, ARRAY_SIZE(imx8mm_m4_sels));
	clks[IMX8MM_CLK_VPU_SRC] = imx_clk_mux2("vpu_src", base + 0x8100, 24, 3, imx8mm_vpu_sels, ARRAY_SIZE(imx8mm_vpu_sels));
	clks[IMX8MM_CLK_GPU3D_SRC] = imx_clk_mux2("gpu3d_src", base + 0x8180, 24, 3,  imx8mm_gpu3d_sels, ARRAY_SIZE(imx8mm_gpu3d_sels));
	clks[IMX8MM_CLK_GPU2D_SRC] = imx_clk_mux2("gpu2d_src", base + 0x8200, 24, 3, imx8mm_gpu2d_sels,  ARRAY_SIZE(imx8mm_gpu2d_sels));
	clks[IMX8MM_CLK_A53_CG] = imx_clk_gate3("arm_a53_cg", "arm_a53_src", base + 0x8000, 28);
	clks[IMX8MM_CLK_M4_CG] = imx_clk_gate3("arm_m4_cg", "arm_m4_src", base + 0x8080, 28);
	clks[IMX8MM_CLK_VPU_CG] = imx_clk_gate3("vpu_cg", "vpu_src", base + 0x8100, 28);
	clks[IMX8MM_CLK_GPU3D_CG] = imx_clk_gate3("gpu3d_cg", "gpu3d_src", base + 0x8180, 28);
	clks[IMX8MM_CLK_GPU2D_CG] = imx_clk_gate3("gpu2d_cg", "gpu2d_src", base + 0x8200, 28);

	clks[IMX8MM_CLK_A53_DIV] = imx_clk_divider2("arm_a53_div", "arm_a53_cg", base + 0x8000, 0, 3);
	clks[IMX8MM_CLK_M4_DIV] = imx_clk_divider2("arm_m4_div", "arm_m4_cg", base + 0x8080, 0, 3);
	clks[IMX8MM_CLK_VPU_DIV] = imx_clk_divider2("vpu_div", "vpu_cg", base + 0x8100, 0, 3);
	clks[IMX8MM_CLK_GPU3D_DIV] = imx_clk_divider2("gpu3d_div", "gpu3d_cg", base + 0x8180, 0, 3);
	clks[IMX8MM_CLK_GPU2D_DIV] = imx_clk_divider2("gpu2d_div", "gpu2d_cg", base + 0x8200, 0, 3);

	/* BUS */
	clks[IMX8MM_CLK_MAIN_AXI_SRC] = imx_clk_mux2("main_axi_src", base + 0x8800, 24, 3, imx8mm_main_axi_sels, ARRAY_SIZE(imx8mm_main_axi_sels));
	clks[IMX8MM_CLK_ENET_AXI_SRC] = imx_clk_mux2("enet_axi_src", base + 0x8880, 24, 3, imx8mm_enet_axi_sels, ARRAY_SIZE(imx8mm_enet_axi_sels));
	clks[IMX8MM_CLK_NAND_USDHC_BUS_SRC] = imx_clk_mux2("nand_usdhc_bus_src", base + 0x8900, 24, 3, imx8mm_nand_usdhc_sels, ARRAY_SIZE(imx8mm_nand_usdhc_sels));
	clks[IMX8MM_CLK_VPU_BUS_SRC] = imx_clk_mux2("vpu_bus_src", base + 0x8980, 24, 3, imx8mm_vpu_bus_sels, ARRAY_SIZE(imx8mm_vpu_bus_sels));
	clks[IMX8MM_CLK_DISP_AXI_SRC] = imx_clk_mux2("disp_axi_src", base + 0x8a00, 24, 3, imx8mm_disp_axi_sels, ARRAY_SIZE(imx8mm_disp_axi_sels));
	clks[IMX8MM_CLK_DISP_APB_SRC] = imx_clk_mux2("disp_apb_src", base + 0x8a80, 24, 3, imx8mm_disp_apb_sels, ARRAY_SIZE(imx8mm_disp_apb_sels));
	clks[IMX8MM_CLK_DISP_RTRM_SRC] = imx_clk_mux2("disp_rtrm_src", base + 0x8b00, 24, 3, imx8mm_disp_rtrm_sels, ARRAY_SIZE(imx8mm_disp_rtrm_sels));
	clks[IMX8MM_CLK_USB_BUS_SRC] = imx_clk_mux2("usb_bus_src", base + 0x8b80, 24, 3, imx8mm_usb_bus_sels, ARRAY_SIZE(imx8mm_usb_bus_sels));
	clks[IMX8MM_CLK_GPU_AXI_SRC] = imx_clk_mux2("gpu_axi_src", base + 0x8c00, 24, 3, imx8mm_gpu_axi_sels, ARRAY_SIZE(imx8mm_gpu_axi_sels));
	clks[IMX8MM_CLK_GPU_AHB_SRC] = imx_clk_mux2("gpu_ahb_src", base + 0x8c80, 24, 3, imx8mm_gpu_ahb_sels, ARRAY_SIZE(imx8mm_gpu_ahb_sels));
	clks[IMX8MM_CLK_NOC_SRC] = imx_clk_mux2("noc_src", base + 0x8d00, 24, 3, imx8mm_noc_sels, ARRAY_SIZE(imx8mm_noc_sels));
	clks[IMX8MM_CLK_NOC_APB_SRC] = imx_clk_mux2("noc_apb_src", base + 0x8d80, 24, 3, imx8mm_noc_apb_sels, ARRAY_SIZE(imx8mm_noc_apb_sels));

	clks[IMX8MM_CLK_MAIN_AXI_CG] = imx_clk_gate3("main_axi_cg", "main_axi_src", base + 0x8800, 28);
	clks[IMX8MM_CLK_ENET_AXI_CG] = imx_clk_gate3("enet_axi_cg", "enet_axi_src", base + 0x8880, 28);
	clks[IMX8MM_CLK_NAND_USDHC_BUS_CG] = imx_clk_gate3("nand_usdhc_bus_cg", "nand_usdhc_bus_src", base + 0x8900, 28);
	clks[IMX8MM_CLK_VPU_BUS_CG] = imx_clk_gate3("vpu_bus_cg", "vpu_bus_src", base + 0x8980, 28);
	clks[IMX8MM_CLK_DISP_AXI_CG] = imx_clk_gate3("disp_axi_cg", "disp_axi_src", base + 0x8a00, 28);
	clks[IMX8MM_CLK_DISP_APB_CG] = imx_clk_gate3("disp_apb_cg", "disp_apb_src", base + 0x8a80, 28);
	clks[IMX8MM_CLK_DISP_RTRM_CG] = imx_clk_gate3("disp_rtrm_cg", "disp_rtrm_src", base + 0x8b00, 28);
	clks[IMX8MM_CLK_USB_BUS_CG] = imx_clk_gate3("usb_bus_cg", "usb_bus_src", base + 0x8b80, 28);
	clks[IMX8MM_CLK_GPU_AXI_CG] = imx_clk_gate3("gpu_axi_cg", "gpu_axi_src", base + 0x8c00, 28);
	clks[IMX8MM_CLK_GPU_AHB_CG] = imx_clk_gate3("gpu_ahb_cg", "gpu_ahb_src", base + 0x8c80, 28);
	clks[IMX8MM_CLK_NOC_CG] = imx_clk_gate3("noc_cg", "noc_src", base + 0x8d00, 28);
	clks[IMX8MM_CLK_NOC_APB_CG] = imx_clk_gate3("noc_apb_cg", "noc_apb_src", base + 0x8d80, 28);

	clks[IMX8MM_CLK_MAIN_AXI_PRE_DIV] = imx_clk_divider2("main_axi_pre_div", "main_axi_cg", base + 0x8800, 16, 3);
	clks[IMX8MM_CLK_ENET_AXI_PRE_DIV] = imx_clk_divider2("enet_axi_pre_div", "enet_axi_cg", base + 0x8880, 16, 3);
	clks[IMX8MM_CLK_NAND_USDHC_BUS_PRE_DIV] = imx_clk_divider2("nand_usdhc_bus_pre_div", "nand_usdhc_bus_cg", base + 0x8900, 16, 3);
	clks[IMX8MM_CLK_VPU_BUS_PRE_DIV] = imx_clk_divider2("vpu_bus_pre_div", "vpu_bus_cg", base + 0x8980, 16, 3);
	clks[IMX8MM_CLK_DISP_AXI_PRE_DIV] = imx_clk_divider2("disp_axi_pre_div", "disp_axi_cg", base + 0x8a00, 16, 3);
	clks[IMX8MM_CLK_DISP_APB_PRE_DIV] = imx_clk_divider2("disp_apb_pre_div", "disp_apb_cg", base + 0x8a80, 16, 3);
	clks[IMX8MM_CLK_DISP_RTRM_PRE_DIV] = imx_clk_divider2("disp_rtrm_pre_div", "disp_rtrm_cg", base + 0x8b00, 16, 3);
	clks[IMX8MM_CLK_USB_BUS_PRE_DIV] = imx_clk_divider2("usb_bus_pre_div", "usb_bus_cg", base + 0x8b80, 16, 3);
	clks[IMX8MM_CLK_GPU_AXI_PRE_DIV] = imx_clk_divider2("gpu_axi_pre_div", "gpu_axi_cg", base + 0x8c00, 16, 3);
	clks[IMX8MM_CLK_GPU_AHB_PRE_DIV] = imx_clk_divider2("gpu_ahb_pre_div", "gpu_ahb_cg", base + 0x8c80, 16, 3);
	clks[IMX8MM_CLK_NOC_PRE_DIV] = imx_clk_divider2("noc_pre_div", "noc_cg", base + 0x8d00, 16, 3);
	clks[IMX8MM_CLK_NOC_APB_PRE_DIV] = imx_clk_divider2("noc_apb_pre_div", "noc_apb_cg", base + 0x8d80, 16, 3);

	clks[IMX8MM_CLK_MAIN_AXI_DIV] = imx_clk_divider2("main_axi_div", "main_axi_pre_div", base + 0x8800, 0, 6);
	clks[IMX8MM_CLK_ENET_AXI_DIV] = imx_clk_divider2("enet_axi_div", "enet_axi_pre_div", base + 0x8880, 0, 6);
	clks[IMX8MM_CLK_NAND_USDHC_BUS_DIV] = imx_clk_divider2("nand_usdhc_bus_div", "nand_usdhc_bus_pre_div", base + 0x8900, 0, 6);
	clks[IMX8MM_CLK_VPU_BUS_DIV] = imx_clk_divider2("vpu_bus_div", "vpu_bus_pre_div", base + 0x8980, 0, 6);
	clks[IMX8MM_CLK_DISP_AXI_DIV] = imx_clk_divider2("disp_axi_div", "disp_axi_pre_div", base + 0x8a00, 0, 6);
	clks[IMX8MM_CLK_DISP_APB_DIV] = imx_clk_divider2("disp_apb_div", "disp_apb_pre_div", base + 0x8a80, 0, 6);
	clks[IMX8MM_CLK_DISP_RTRM_DIV] = imx_clk_divider2("disp_rtrm_div", "disp_rtrm_pre_div", base + 0x8b00, 0, 6);
	clks[IMX8MM_CLK_USB_BUS_DIV] = imx_clk_divider2("usb_bus_div", "usb_bus_pre_div", base + 0x8b80, 0, 6);
	clks[IMX8MM_CLK_GPU_AXI_DIV] = imx_clk_divider2("gpu_axi_div", "gpu_axi_pre_div", base + 0x8c00, 0, 6);
	clks[IMX8MM_CLK_GPU_AHB_DIV] = imx_clk_divider2("gpu_ahb_div", "gpu_ahb_pre_div", base + 0x8c80, 0, 6);
	clks[IMX8MM_CLK_NOC_DIV] = imx_clk_divider2("noc_div", "noc_pre_div", base + 0x8d00, 0, 6);
	clks[IMX8MM_CLK_NOC_APB_DIV] = imx_clk_divider2("noc_apb_div", "noc_apb_pre_div", base + 0x8d80, 0, 6);

	/* AHB */
	clks[IMX8MM_CLK_AHB_SRC] = imx_clk_mux2("ahb_src", base + 0x9000, 24, 3, imx8mm_ahb_sels, ARRAY_SIZE(imx8mm_ahb_sels));
	clks[IMX8MM_CLK_AUDIO_AHB_SRC] = imx_clk_mux2("audio_ahb_src", base + 0x9100, 24, 3, imx8mm_audio_ahb_sels, ARRAY_SIZE(imx8mm_audio_ahb_sels));
	clks[IMX8MM_CLK_AHB_CG] = imx_clk_gate3("ahb_cg", "ahb_src", base + 0x9000, 28);
	clks[IMX8MM_CLK_AUDIO_AHB_CG] = imx_clk_gate3("audio_ahb_cg", "audio_ahb_src", base + 0x9100, 28);
	clks[IMX8MM_CLK_AHB_PRE_DIV] = imx_clk_divider2("ahb_pre_div", "ahb_cg", base + 0x9000, 16, 3);
	clks[IMX8MM_CLK_AUDIO_AHB_PRE_DIV] = imx_clk_divider2("audio_ahb_pre_div", "audio_ahb_cg", base + 0x9100, 16, 3);
	clks[IMX8MM_CLK_AHB_DIV] = imx_clk_divider2("ahb_div", "ahb_pre_div", base + 0x9000, 0, 6);
	clks[IMX8MM_CLK_AUDIO_AHB_DIV] = imx_clk_divider2("audio_ahb_div", "audio_ahb_pre_div", base + 0x9100, 0, 6);

	/* IPG */
	clks[IMX8MM_CLK_IPG_ROOT] = imx_clk_divider2("ipg_root", "ahb_div", base + 0x9080, 0, 1);
	clks[IMX8MM_CLK_IPG_AUDIO_ROOT] = imx_clk_divider2("ipg_audio_root", "audio_ahb_div", base + 0x9180, 0, 1);

	/* IP */
	clks[IMX8MM_CLK_DRAM_ALT_SRC] = imx_clk_mux2("dram_alt_src", base + 0xa000, 24, 3, imx8mm_dram_alt_sels, ARRAY_SIZE(imx8mm_dram_alt_sels));
	clks[IMX8MM_CLK_DRAM_APB_SRC] = imx_clk_mux2("dram_apb_src", base + 0xa080, 24, 3, imx8mm_dram_apb_sels, ARRAY_SIZE(imx8mm_dram_apb_sels));
	clks[IMX8MM_CLK_VPU_G1_SRC] = imx_clk_mux2("vpu_g1_src", base + 0xa100, 24, 3, imx8mm_vpu_g1_sels, ARRAY_SIZE(imx8mm_vpu_g1_sels));
	clks[IMX8MM_CLK_VPU_G2_SRC] = imx_clk_mux2("vpu_g2_src", base + 0xa180, 24, 3, imx8mm_vpu_g2_sels, ARRAY_SIZE(imx8mm_vpu_g2_sels));
	clks[IMX8MM_CLK_DISP_DTRC_SRC] = imx_clk_mux2("disp_dtrc_src", base + 0xa200, 24, 3, imx8mm_disp_dtrc_sels, ARRAY_SIZE(imx8mm_disp_dtrc_sels));
	clks[IMX8MM_CLK_DISP_DC8000_SRC] = imx_clk_mux2("disp_dc8000_src", base + 0xa280, 24, 3, imx8mm_disp_dc8000_sels, ARRAY_SIZE(imx8mm_disp_dc8000_sels));
	clks[IMX8MM_CLK_PCIE1_CTRL_SRC] = imx_clk_mux2("pcie1_ctrl_src", base + 0xa300, 24, 3, imx8mm_pcie1_ctrl_sels, ARRAY_SIZE(imx8mm_pcie1_ctrl_sels));
	clks[IMX8MM_CLK_PCIE1_PHY_SRC] = imx_clk_mux2("pcie1_phy_src", base + 0xa380, 24, 3, imx8mm_pcie1_phy_sels, ARRAY_SIZE(imx8mm_pcie1_phy_sels));
	clks[IMX8MM_CLK_PCIE1_AUX_SRC] = imx_clk_mux2("pcie1_aux_src", base + 0xa400, 24, 3, imx8mm_pcie1_aux_sels, ARRAY_SIZE(imx8mm_pcie1_aux_sels));
	clks[IMX8MM_CLK_DC_PIXEL_SRC] = imx_clk_mux2("dc_pixel_src", base + 0xa480, 24, 3, imx8mm_dc_pixel_sels, ARRAY_SIZE(imx8mm_dc_pixel_sels));
	clks[IMX8MM_CLK_LCDIF_PIXEL_SRC] = imx_clk_mux2("lcdif_pixel_src", base + 0xa500, 24, 3, imx8mm_lcdif_pixel_sels, ARRAY_SIZE(imx8mm_lcdif_pixel_sels));
	clks[IMX8MM_CLK_SAI1_SRC] = imx_clk_mux2("sai1_src", base + 0xa580, 24, 3, imx8mm_sai1_sels, ARRAY_SIZE(imx8mm_sai1_sels));
	clks[IMX8MM_CLK_SAI2_SRC] = imx_clk_mux2("sai2_src", base + 0xa600, 24, 3, imx8mm_sai2_sels, ARRAY_SIZE(imx8mm_sai2_sels));
	clks[IMX8MM_CLK_SAI3_SRC] = imx_clk_mux2("sai3_src", base + 0xa680, 24, 3, imx8mm_sai3_sels, ARRAY_SIZE(imx8mm_sai3_sels));
	clks[IMX8MM_CLK_SAI4_SRC] = imx_clk_mux2("sai4_src", base + 0xa700, 24, 3, imx8mm_sai4_sels, ARRAY_SIZE(imx8mm_sai4_sels));
	clks[IMX8MM_CLK_SAI5_SRC] = imx_clk_mux2("sai5_src", base + 0xa780, 24, 3, imx8mm_sai5_sels, ARRAY_SIZE(imx8mm_sai5_sels));
	clks[IMX8MM_CLK_SAI6_SRC] = imx_clk_mux2("sai6_src", base + 0xa800, 24, 3, imx8mm_sai6_sels, ARRAY_SIZE(imx8mm_sai6_sels));
	clks[IMX8MM_CLK_SPDIF1_SRC] = imx_clk_mux2("spdif1_src", base + 0xa880, 24, 3, imx8mm_spdif1_sels, ARRAY_SIZE(imx8mm_spdif1_sels));
	clks[IMX8MM_CLK_SPDIF2_SRC] = imx_clk_mux2("spdif2_src", base + 0xa900, 24, 3, imx8mm_spdif2_sels, ARRAY_SIZE(imx8mm_spdif2_sels));
	clks[IMX8MM_CLK_ENET_REF_SRC] = imx_clk_mux2("enet_ref_src", base + 0xa980, 24, 3, imx8mm_enet_ref_sels, ARRAY_SIZE(imx8mm_enet_ref_sels));
	clks[IMX8MM_CLK_ENET_TIMER_SRC] = imx_clk_mux2("enet_timer_src", base + 0xaa00, 24, 3, imx8mm_enet_timer_sels, ARRAY_SIZE(imx8mm_enet_timer_sels));
	clks[IMX8MM_CLK_ENET_PHY_REF_SRC] = imx_clk_mux2("enet_phy_src", base + 0xaa80, 24, 3, imx8mm_enet_phy_sels, ARRAY_SIZE(imx8mm_enet_phy_sels));
	clks[IMX8MM_CLK_NAND_SRC] = imx_clk_mux2("nand_src", base + 0xab00, 24, 3, imx8mm_nand_sels, ARRAY_SIZE(imx8mm_nand_sels));
	clks[IMX8MM_CLK_QSPI_SRC] = imx_clk_mux2("qspi_src", base + 0xab80, 24, 3, imx8mm_qspi_sels, ARRAY_SIZE(imx8mm_qspi_sels));
	clks[IMX8MM_CLK_USDHC1_SRC] = imx_clk_mux2("usdhc1_src", base + 0xac00, 24, 3, imx8mm_usdhc1_sels, ARRAY_SIZE(imx8mm_usdhc1_sels));
	clks[IMX8MM_CLK_USDHC2_SRC] = imx_clk_mux2("usdhc2_src", base + 0xac80, 24, 3, imx8mm_usdhc2_sels, ARRAY_SIZE(imx8mm_usdhc2_sels));
	clks[IMX8MM_CLK_I2C1_SRC] = imx_clk_mux2("i2c1_src", base + 0xad00, 24, 3, imx8mm_i2c1_sels, ARRAY_SIZE(imx8mm_i2c1_sels));
	clks[IMX8MM_CLK_I2C2_SRC] = imx_clk_mux2("i2c2_src", base + 0xad80, 24, 3, imx8mm_i2c2_sels, ARRAY_SIZE(imx8mm_i2c2_sels));
	clks[IMX8MM_CLK_I2C3_SRC] = imx_clk_mux2("i2c3_src", base + 0xae00, 24, 3, imx8mm_i2c3_sels, ARRAY_SIZE(imx8mm_i2c3_sels));
	clks[IMX8MM_CLK_I2C4_SRC] = imx_clk_mux2("i2c4_src", base + 0xae80, 24, 3, imx8mm_i2c4_sels, ARRAY_SIZE(imx8mm_i2c4_sels));
	clks[IMX8MM_CLK_UART1_SRC] = imx_clk_mux2("uart1_src", base + 0xaf00, 24, 3, imx8mm_uart1_sels, ARRAY_SIZE(imx8mm_uart1_sels));
	clks[IMX8MM_CLK_UART2_SRC] = imx_clk_mux2("uart2_src", base + 0xaf80, 24, 3, imx8mm_uart2_sels, ARRAY_SIZE(imx8mm_uart2_sels));
	clks[IMX8MM_CLK_UART3_SRC] = imx_clk_mux2("uart3_src", base + 0xb000, 24, 3, imx8mm_uart3_sels, ARRAY_SIZE(imx8mm_uart3_sels));
	clks[IMX8MM_CLK_UART4_SRC] = imx_clk_mux2("uart4_src", base + 0xb080, 24, 3, imx8mm_uart4_sels, ARRAY_SIZE(imx8mm_uart4_sels));
	clks[IMX8MM_CLK_USB_CORE_REF_SRC] = imx_clk_mux2("usb_core_ref_src", base + 0xb100, 24, 3, imx8mm_usb_core_sels, ARRAY_SIZE(imx8mm_usb_core_sels));
	clks[IMX8MM_CLK_USB_PHY_REF_SRC] = imx_clk_mux2("usb_phy_ref_src", base + 0xb180, 24, 3, imx8mm_usb_phy_sels, ARRAY_SIZE(imx8mm_usb_phy_sels));
	clks[IMX8MM_CLK_ECSPI1_SRC] = imx_clk_mux2("ecspi1_src", base + 0xb280, 24, 3, imx8mm_ecspi1_sels, ARRAY_SIZE(imx8mm_ecspi1_sels));
	clks[IMX8MM_CLK_ECSPI2_SRC] = imx_clk_mux2("ecspi2_src", base + 0xb300, 24, 3, imx8mm_ecspi2_sels, ARRAY_SIZE(imx8mm_ecspi2_sels));
	clks[IMX8MM_CLK_PWM1_SRC] = imx_clk_mux2("pwm1_src", base + 0xb380, 24, 3, imx8mm_pwm1_sels, ARRAY_SIZE(imx8mm_pwm1_sels));
	clks[IMX8MM_CLK_PWM2_SRC] = imx_clk_mux2("pwm2_src", base + 0xb400, 24, 3, imx8mm_pwm2_sels, ARRAY_SIZE(imx8mm_pwm2_sels));
	clks[IMX8MM_CLK_PWM3_SRC] = imx_clk_mux2("pwm3_src", base + 0xb480, 24, 3, imx8mm_pwm3_sels, ARRAY_SIZE(imx8mm_pwm3_sels));
	clks[IMX8MM_CLK_PWM4_SRC] = imx_clk_mux2("pwm4_src", base + 0xb500, 24, 3, imx8mm_pwm4_sels, ARRAY_SIZE(imx8mm_pwm4_sels));
	clks[IMX8MM_CLK_GPT1_SRC] = imx_clk_mux2("gpt1_src", base + 0xb580, 24, 3, imx8mm_gpt1_sels, ARRAY_SIZE(imx8mm_gpt1_sels));
	clks[IMX8MM_CLK_WDOG_SRC] = imx_clk_mux2("wdog_src", base + 0xb900, 24, 3, imx8mm_wdog_sels, ARRAY_SIZE(imx8mm_wdog_sels));
	clks[IMX8MM_CLK_WRCLK_SRC] = imx_clk_mux2("wrclk_src", base + 0xb980, 24, 3, imx8mm_wrclk_sels, ARRAY_SIZE(imx8mm_wrclk_sels));
	clks[IMX8MM_CLK_DSI_CORE_SRC] = imx_clk_mux2("dsi_core_src", base + 0xbb00, 24, 3, imx8mm_dsi_core_sels, ARRAY_SIZE(imx8mm_dsi_core_sels));
	clks[IMX8MM_CLK_DSI_PHY_REF_SRC] = imx_clk_mux2("dsi_phy_ref_src", base + 0xbb80, 24, 3, imx8mm_dsi_phy_sels, ARRAY_SIZE(imx8mm_dsi_phy_sels));
	clks[IMX8MM_CLK_DSI_DBI_SRC] = imx_clk_mux2("dsi_dbi_src", base + 0xbc00, 24, 3, imx8mm_dsi_dbi_sels, ARRAY_SIZE(imx8mm_dsi_dbi_sels));
	clks[IMX8MM_CLK_USDHC3_SRC] = imx_clk_mux2("usdhc3_src", base + 0xbc80, 24, 3, imx8mm_usdhc3_sels, ARRAY_SIZE(imx8mm_usdhc3_sels));
	clks[IMX8MM_CLK_CSI1_CORE_SRC] = imx_clk_mux2("csi1_core_src", base + 0xbd00, 24, 3, imx8mm_csi1_core_sels, ARRAY_SIZE(imx8mm_csi1_core_sels));
	clks[IMX8MM_CLK_CSI1_PHY_REF_SRC] = imx_clk_mux2("csi1_phy_ref_src", base + 0xbd80, 24, 3, imx8mm_csi1_phy_sels, ARRAY_SIZE(imx8mm_csi1_phy_sels));
	clks[IMX8MM_CLK_CSI1_ESC_SRC] = imx_clk_mux2("csi1_esc_src", base + 0xbe00, 24, 3, imx8mm_csi1_esc_sels, ARRAY_SIZE(imx8mm_csi1_esc_sels));
	clks[IMX8MM_CLK_CSI2_CORE_SRC] = imx_clk_mux2("csi2_core_src", base + 0xbe80, 24, 3, imx8mm_csi2_core_sels, ARRAY_SIZE(imx8mm_csi2_core_sels));
	clks[IMX8MM_CLK_CSI2_PHY_REF_SRC] = imx_clk_mux2("csi2_phy_ref_src", base + 0xbf00, 24, 3, imx8mm_csi2_phy_sels, ARRAY_SIZE(imx8mm_csi2_phy_sels));
	clks[IMX8MM_CLK_CSI2_ESC_SRC] = imx_clk_mux2("csi2_esc_src", base + 0xbf80, 24, 3, imx8mm_csi2_esc_sels, ARRAY_SIZE(imx8mm_csi2_esc_sels));
	clks[IMX8MM_CLK_PCIE2_CTRL_SRC] = imx_clk_mux2("pcie2_ctrl_src", base + 0xc000, 24, 3, imx8mm_pcie2_ctrl_sels, ARRAY_SIZE(imx8mm_pcie2_ctrl_sels));
	clks[IMX8MM_CLK_PCIE2_PHY_SRC] = imx_clk_mux2("pcie2_phy_src", base + 0xc080, 24, 3, imx8mm_pcie2_phy_sels, ARRAY_SIZE(imx8mm_pcie2_phy_sels));
	clks[IMX8MM_CLK_PCIE2_AUX_SRC] = imx_clk_mux2("pcie2_aux_src", base + 0xc100, 24, 3, imx8mm_pcie2_aux_sels, ARRAY_SIZE(imx8mm_pcie2_aux_sels));
	clks[IMX8MM_CLK_ECSPI3_SRC] = imx_clk_mux2("ecspi3_src", base + 0xc180, 24, 3, imx8mm_ecspi3_sels, ARRAY_SIZE(imx8mm_ecspi3_sels));
	clks[IMX8MM_CLK_PDM_SRC] = imx_clk_mux2("pdm_src", base + 0xc200, 24, 3, imx8mm_pdm_sels, ARRAY_SIZE(imx8mm_pdm_sels));
	clks[IMX8MM_CLK_VPU_H1_SRC] = imx_clk_mux2("vpu_h1_src", base + 0xc280, 24, 3, imx8mm_vpu_h1_sels, ARRAY_SIZE(imx8mm_vpu_h1_sels));

	clks[IMX8MM_CLK_DRAM_ALT_CG] = imx_clk_gate3("dram_alt_cg", "dram_alt_src", base + 0xa000, 28);
	clks[IMX8MM_CLK_DRAM_APB_CG] = imx_clk_gate3("dram_apb_cg", "dram_apb_src", base + 0xa080, 28);
	clks[IMX8MM_CLK_VPU_G1_CG] = imx_clk_gate3("vpu_g1_cg", "vpu_g1_src", base + 0xa100, 28);
	clks[IMX8MM_CLK_VPU_G2_CG] = imx_clk_gate3("vpu_g2_cg", "vpu_g2_src", base + 0xa180, 28);
	clks[IMX8MM_CLK_DISP_DTRC_CG] = imx_clk_gate3("disp_dtrc_cg", "disp_dtrc_src", base + 0xa200, 28);
	clks[IMX8MM_CLK_DISP_DC8000_CG] = imx_clk_gate3("disp_dc8000_cg", "disp_dc8000_src", base + 0xa280, 28);
	clks[IMX8MM_CLK_PCIE1_CTRL_CG] = imx_clk_gate3("pcie1_ctrl_cg", "pcie1_ctrl_src", base + 0xa300, 28);
	clks[IMX8MM_CLK_PCIE1_PHY_CG] = imx_clk_gate3("pcie1_phy_cg", "pcie1_phy_src", base + 0xa380, 28);
	clks[IMX8MM_CLK_PCIE1_AUX_CG] = imx_clk_gate3("pcie1_aux_cg", "pcie1_aux_src", base + 0xa400, 28);
	clks[IMX8MM_CLK_DC_PIXEL_CG] = imx_clk_gate3("dc_pixel_cg", "dc_pixel_src", base + 0xa480, 28);
	clks[IMX8MM_CLK_LCDIF_PIXEL_CG] = imx_clk_gate3("lcdif_pixel_cg", "lcdif_pixel_src", base + 0xa500, 28);
	clks[IMX8MM_CLK_SAI1_CG] = imx_clk_gate3("sai1_cg", "sai1_src", base + 0xa580, 28);
	clks[IMX8MM_CLK_SAI2_CG] = imx_clk_gate3("sai2_cg", "sai2_src", base + 0xa600, 28);
	clks[IMX8MM_CLK_SAI3_CG] = imx_clk_gate3("sai3_cg", "sai3_src", base + 0xa680, 28);
	clks[IMX8MM_CLK_SAI4_CG] = imx_clk_gate3("sai4_cg", "sai4_src", base + 0xa700, 28);
	clks[IMX8MM_CLK_SAI5_CG] = imx_clk_gate3("sai5_cg", "sai5_src", base + 0xa780, 28);
	clks[IMX8MM_CLK_SAI6_CG] = imx_clk_gate3("sai6_cg", "sai6_src", base + 0xa800, 28);
	clks[IMX8MM_CLK_SPDIF1_CG] = imx_clk_gate3("spdif1_cg", "spdif1_src", base + 0xa880, 28);
	clks[IMX8MM_CLK_SPDIF2_CG] = imx_clk_gate3("spdif2_cg", "spdif2_src", base + 0xa900, 28);
	clks[IMX8MM_CLK_ENET_REF_CG] = imx_clk_gate3("enet_ref_cg", "enet_ref_src", base + 0xa980, 28);
	clks[IMX8MM_CLK_ENET_TIMER_CG] = imx_clk_gate3("enet_timer_cg", "enet_timer_src", base + 0xaa00, 28);
	clks[IMX8MM_CLK_ENET_PHY_REF_CG] = imx_clk_gate3("enet_phy_cg", "enet_phy_src", base + 0xaa80, 28);
	clks[IMX8MM_CLK_NAND_CG] = imx_clk_gate3("nand_cg", "nand_src", base + 0xab00, 28);
	clks[IMX8MM_CLK_QSPI_CG] = imx_clk_gate3("qspi_cg", "qspi_src", base + 0xab80, 28);
	clks[IMX8MM_CLK_USDHC1_CG] = imx_clk_gate3("usdhc1_cg", "usdhc1_src", base + 0xac00, 28);
	clks[IMX8MM_CLK_USDHC2_CG] = imx_clk_gate3("usdhc2_cg", "usdhc2_src", base + 0xac80, 28);
	clks[IMX8MM_CLK_I2C1_CG] = imx_clk_gate3("i2c1_cg", "i2c1_src", base + 0xad00, 28);
	clks[IMX8MM_CLK_I2C2_CG] = imx_clk_gate3("i2c2_cg", "i2c2_src", base + 0xad80, 28);
	clks[IMX8MM_CLK_I2C3_CG] = imx_clk_gate3("i2c3_cg", "i2c3_src", base + 0xae00, 28);
	clks[IMX8MM_CLK_I2C4_CG] = imx_clk_gate3("i2c4_cg", "i2c4_src", base + 0xae80, 28);
	clks[IMX8MM_CLK_UART1_CG] = imx_clk_gate3("uart1_cg", "uart1_src", base + 0xaf00, 28);
	clks[IMX8MM_CLK_UART2_CG] = imx_clk_gate3("uart2_cg", "uart2_src", base + 0xaf80, 28);
	clks[IMX8MM_CLK_UART3_CG] = imx_clk_gate3("uart3_cg", "uart3_src", base + 0xb000, 28);
	clks[IMX8MM_CLK_UART4_CG] = imx_clk_gate3("uart4_cg", "uart4_src", base + 0xb080, 28);
	clks[IMX8MM_CLK_USB_CORE_REF_CG] = imx_clk_gate3("usb_core_ref_cg", "usb_core_ref_src", base + 0xb100, 28);
	clks[IMX8MM_CLK_USB_PHY_REF_CG] = imx_clk_gate3("usb_phy_ref_cg", "usb_phy_ref_src", base + 0xb180, 28);
	clks[IMX8MM_CLK_ECSPI1_CG] = imx_clk_gate3("ecspi1_cg", "ecspi1_src",  base + 0xb280, 28);
	clks[IMX8MM_CLK_ECSPI2_CG] = imx_clk_gate3("ecspi2_cg", "ecspi2_src", base + 0xb300, 28);
	clks[IMX8MM_CLK_PWM1_CG] = imx_clk_gate3("pwm1_cg", "pwm1_src", base + 0xb380, 28);
	clks[IMX8MM_CLK_PWM2_CG] = imx_clk_gate3("pwm2_cg", "pwm2_src", base + 0xb400, 28);
	clks[IMX8MM_CLK_PWM3_CG] = imx_clk_gate3("pwm3_cg", "pwm3_src", base + 0xb480, 28);
	clks[IMX8MM_CLK_PWM4_CG] = imx_clk_gate3("pwm4_cg", "pwm4_src", base + 0xb500, 28);
	clks[IMX8MM_CLK_GPT1_CG] = imx_clk_gate3("gpt1_cg", "gpt1_src", base + 0xb580, 28);
	clks[IMX8MM_CLK_WDOG_CG] = imx_clk_gate3("wdog_cg", "wdog_src", base + 0xb900, 28);
	clks[IMX8MM_CLK_WRCLK_CG] = imx_clk_gate3("wrclk_cg", "wrclk_src", base + 0xb980, 28);
	clks[IMX8MM_CLK_DSI_CORE_CG] = imx_clk_gate3("dsi_core_cg", "dsi_core_src", base + 0xbb00, 28);
	clks[IMX8MM_CLK_DSI_PHY_REF_CG] = imx_clk_gate3("dsi_phy_ref_cg", "dsi_phy_ref_src", base + 0xbb80, 28);
	clks[IMX8MM_CLK_DSI_DBI_CG] = imx_clk_gate3("dsi_dbi_cg", "dsi_dbi_src", base + 0xbc00, 28);
	clks[IMX8MM_CLK_USDHC3_CG] = imx_clk_gate3("usdhc3_cg", "usdhc3_src", base + 0xbc80, 28);
	clks[IMX8MM_CLK_CSI1_CORE_CG] = imx_clk_gate3("csi1_core_cg", "csi1_core_src", base + 0xbd00, 28);
	clks[IMX8MM_CLK_CSI1_PHY_REF_CG] = imx_clk_gate3("csi1_phy_ref_cg", "csi1_phy_ref_src", base + 0xbd80, 28);
	clks[IMX8MM_CLK_CSI1_ESC_CG] = imx_clk_gate3("csi1_esc_cg", "csi1_esc_src", base + 0xbe00, 28);
	clks[IMX8MM_CLK_CSI2_CORE_CG] = imx_clk_gate3("csi2_core_cg", "csi2_core_src", base + 0xbe80, 28);
	clks[IMX8MM_CLK_CSI2_PHY_REF_CG] = imx_clk_gate3("csi2_phy_ref_cg", "csi2_phy_ref_src", base + 0xbf00, 28);
	clks[IMX8MM_CLK_CSI2_ESC_CG] = imx_clk_gate3("csi2_esc_cg", "csi2_esc_src", base + 0xbf80, 28);
	clks[IMX8MM_CLK_PCIE2_CTRL_CG] = imx_clk_gate3("pcie2_ctrl_cg", "pcie2_ctrl_src", base + 0xc000, 28);
	clks[IMX8MM_CLK_PCIE2_PHY_CG] = imx_clk_gate3("pcie2_phy_cg", "pcie2_phy_src", base + 0xc080, 28);
	clks[IMX8MM_CLK_PCIE2_AUX_CG] = imx_clk_gate3("pcie2_aux_cg", "pcie2_aux_src", base + 0xc100, 28);
	clks[IMX8MM_CLK_ECSPI3_CG] = imx_clk_gate3("ecspi3_cg", "ecspi3_src", base + 0xc180, 28);
	clks[IMX8MM_CLK_PDM_CG] = imx_clk_gate3("pdm_cg", "pdm_src", base + 0xc200, 28);
	clks[IMX8MM_CLK_VPU_H1_CG] = imx_clk_gate3("vpu_h1_cg", "vpu_h1_src", base + 0xc280, 28);

	clks[IMX8MM_CLK_DRAM_ALT_PRE_DIV] = imx_clk_divider2("dram_alt_pre_div", "dram_alt_cg", base + 0xa000, 16, 3);
	clks[IMX8MM_CLK_DRAM_APB_PRE_DIV] = imx_clk_divider2("dram_apb_pre_div", "dram_apb_cg", base + 0xa080, 16, 3);
	clks[IMX8MM_CLK_VPU_G1_PRE_DIV] = imx_clk_divider2("vpu_g1_pre_div", "vpu_g1_cg", base + 0xa100, 16, 3);
	clks[IMX8MM_CLK_VPU_G2_PRE_DIV] = imx_clk_divider2("vpu_g2_pre_div", "vpu_g2_cg", base + 0xa180, 16, 3);
	clks[IMX8MM_CLK_DISP_DTRC_PRE_DIV] = imx_clk_divider2("disp_dtrc_pre_div", "disp_dtrc_cg", base + 0xa200, 16, 3);
	clks[IMX8MM_CLK_DISP_DC8000_PRE_DIV] = imx_clk_divider2("disp_dc8000_pre_div", "disp_dc8000_cg", base + 0xa280, 16, 3);
	clks[IMX8MM_CLK_PCIE1_CTRL_PRE_DIV] = imx_clk_divider2("pcie1_ctrl_pre_div", "pcie1_ctrl_cg", base + 0xa300, 16, 3);
	clks[IMX8MM_CLK_PCIE1_PHY_PRE_DIV] = imx_clk_divider2("pcie1_phy_pre_div", "pcie1_phy_cg", base + 0xa380, 16, 3);
	clks[IMX8MM_CLK_PCIE1_AUX_PRE_DIV] = imx_clk_divider2("pcie1_aux_pre_div", "pcie1_aux_cg", base + 0xa400, 16, 3);
	clks[IMX8MM_CLK_DC_PIXEL_PRE_DIV] = imx_clk_divider2("dc_pixel_pre_div", "dc_pixel_cg", base + 0xa480, 16, 3);
	clks[IMX8MM_CLK_LCDIF_PIXEL_PRE_DIV] = imx_clk_divider2("lcdif_pixel_pre_div", "lcdif_pixel_cg", base + 0xa500, 16, 3);
	clks[IMX8MM_CLK_SAI1_PRE_DIV] = imx_clk_divider2("sai1_pre_div", "sai1_cg", base + 0xa580, 16, 3);
	clks[IMX8MM_CLK_SAI2_PRE_DIV] = imx_clk_divider2("sai2_pre_div", "sai2_cg", base + 0xa600, 16, 3);
	clks[IMX8MM_CLK_SAI3_PRE_DIV] = imx_clk_divider2("sai3_pre_div", "sai3_cg", base + 0xa680, 16, 3);
	clks[IMX8MM_CLK_SAI4_PRE_DIV] = imx_clk_divider2("sai4_pre_div", "sai4_cg", base + 0xa700, 16, 3);
	clks[IMX8MM_CLK_SAI5_PRE_DIV] = imx_clk_divider2("sai5_pre_div", "sai5_cg", base + 0xa780, 16, 3);
	clks[IMX8MM_CLK_SAI6_PRE_DIV] = imx_clk_divider2("sai6_pre_div", "sai6_cg", base + 0xa800, 16, 3);
	clks[IMX8MM_CLK_SPDIF1_PRE_DIV] = imx_clk_divider2("spdif1_pre_div", "spdif1_cg", base + 0xa880, 16, 3);
	clks[IMX8MM_CLK_SPDIF2_PRE_DIV] = imx_clk_divider2("spdif2_pre_div", "spdif2_cg", base + 0xa900, 16, 3);
	clks[IMX8MM_CLK_ENET_REF_PRE_DIV] = imx_clk_divider2("enet_ref_pre_div", "enet_ref_cg", base + 0xa980, 16, 3);
	clks[IMX8MM_CLK_ENET_TIMER_PRE_DIV] = imx_clk_divider2("enet_timer_pre_div", "enet_timer_cg", base + 0xaa00, 16, 3);
	clks[IMX8MM_CLK_ENET_PHY_REF_PRE_DIV] = imx_clk_divider2("enet_phy_pre_div", "enet_phy_cg", base + 0xaa80, 16, 3);
	clks[IMX8MM_CLK_NAND_PRE_DIV] = imx_clk_divider2("nand_pre_div", "nand_cg", base + 0xab00, 16, 3);
	clks[IMX8MM_CLK_QSPI_PRE_DIV] = imx_clk_divider2("qspi_pre_div", "qspi_cg", base + 0xab80, 16, 3);
	clks[IMX8MM_CLK_USDHC1_PRE_DIV] = imx_clk_divider2("usdhc1_pre_div", "usdhc1_cg", base + 0xac00, 16, 3);
	clks[IMX8MM_CLK_USDHC2_PRE_DIV] = imx_clk_divider2("usdhc2_pre_div", "usdhc2_cg", base + 0xac80, 16, 3);
	clks[IMX8MM_CLK_I2C1_PRE_DIV] = imx_clk_divider2("i2c1_pre_div", "i2c1_cg", base + 0xad00, 16, 3);
	clks[IMX8MM_CLK_I2C2_PRE_DIV] = imx_clk_divider2("i2c2_pre_div", "i2c2_cg", base + 0xad80, 16, 3);
	clks[IMX8MM_CLK_I2C3_PRE_DIV] = imx_clk_divider2("i2c3_pre_div", "i2c3_cg", base + 0xae00, 16, 3);
	clks[IMX8MM_CLK_I2C4_PRE_DIV] = imx_clk_divider2("i2c4_pre_div", "i2c4_cg", base + 0xae80, 16, 3);
	clks[IMX8MM_CLK_UART1_PRE_DIV] = imx_clk_divider2("uart1_pre_div", "uart1_cg", base + 0xaf00, 16, 3);
	clks[IMX8MM_CLK_UART2_PRE_DIV] = imx_clk_divider2("uart2_pre_div", "uart2_cg", base + 0xaf80, 16, 3);
	clks[IMX8MM_CLK_UART3_PRE_DIV] = imx_clk_divider2("uart3_pre_div", "uart3_cg", base + 0xb000, 16, 3);
	clks[IMX8MM_CLK_UART4_PRE_DIV] = imx_clk_divider2("uart4_pre_div", "uart4_cg", base + 0xb080, 16, 3);
	clks[IMX8MM_CLK_USB_CORE_REF_PRE_DIV] = imx_clk_divider2("usb_core_ref_pre_div", "usb_core_ref_cg", base + 0xb100, 16, 3);
	clks[IMX8MM_CLK_USB_PHY_REF_PRE_DIV] = imx_clk_divider2("usb_phy_ref_pre_div", "usb_phy_ref_cg", base + 0xb180, 16, 3);
	clks[IMX8MM_CLK_ECSPI1_PRE_DIV] = imx_clk_divider2("ecspi1_pre_div", "ecspi1_cg", base + 0xb280, 16, 3);
	clks[IMX8MM_CLK_ECSPI2_PRE_DIV] = imx_clk_divider2("ecspi2_pre_div", "ecspi2_cg", base + 0xb300, 16, 3);
	clks[IMX8MM_CLK_PWM1_PRE_DIV] = imx_clk_divider2("pwm1_pre_div", "pwm1_cg", base + 0xb380, 16, 3);
	clks[IMX8MM_CLK_PWM2_PRE_DIV] = imx_clk_divider2("pwm2_pre_div", "pwm2_cg", base + 0xb400, 16, 3);
	clks[IMX8MM_CLK_PWM3_PRE_DIV] = imx_clk_divider2("pwm3_pre_div", "pwm3_cg", base + 0xb480, 16, 3);
	clks[IMX8MM_CLK_PWM4_PRE_DIV] = imx_clk_divider2("pwm4_pre_div", "pwm4_cg", base + 0xb500, 16, 3);
	clks[IMX8MM_CLK_GPT1_PRE_DIV] = imx_clk_divider2("gpt1_pre_div", "gpt1_cg", base + 0xb580, 16, 3);
	clks[IMX8MM_CLK_WDOG_PRE_DIV] = imx_clk_divider2("wdog_pre_div", "wdog_cg", base + 0xb900, 16, 3);
	clks[IMX8MM_CLK_WRCLK_PRE_DIV] = imx_clk_divider2("wrclk_pre_div", "wrclk_cg", base + 0xb980, 16, 3);
	clks[IMX8MM_CLK_DSI_CORE_PRE_DIV] = imx_clk_divider2("dsi_core_pre_div", "dsi_core_cg", base + 0xbb00, 16, 3);
	clks[IMX8MM_CLK_DSI_PHY_REF_PRE_DIV] = imx_clk_divider2("dsi_phy_ref_pre_div", "dsi_phy_ref_cg", base + 0xbb80, 16, 3);
	clks[IMX8MM_CLK_DSI_DBI_PRE_DIV] = imx_clk_divider2("dsi_dbi_pre_div", "dsi_dbi_cg", base + 0xbc00, 16, 3);
	clks[IMX8MM_CLK_USDHC3_PRE_DIV] = imx_clk_divider2("usdhc3_pre_div", "usdhc3_cg", base + 0xbc80, 16, 3);
	clks[IMX8MM_CLK_CSI1_CORE_PRE_DIV] = imx_clk_divider2("csi1_core_pre_div", "csi1_core_cg", base + 0xbd00, 16, 3);
	clks[IMX8MM_CLK_CSI1_PHY_REF_PRE_DIV] = imx_clk_divider2("csi1_phy_ref_pre_div", "csi1_phy_ref_cg", base + 0xbd80, 16, 3);
	clks[IMX8MM_CLK_CSI1_ESC_PRE_DIV] = imx_clk_divider2("csi1_esc_pre_div", "csi1_esc_cg", base + 0xbe00, 16, 3);
	clks[IMX8MM_CLK_CSI2_CORE_PRE_DIV] = imx_clk_divider2("csi2_core_pre_div", "csi2_core_cg", base + 0xbe80, 16, 3);
	clks[IMX8MM_CLK_CSI2_PHY_REF_PRE_DIV] = imx_clk_divider2("csi2_phy_ref_pre_div", "csi2_phy_ref_cg", base + 0xbf00, 16, 3);
	clks[IMX8MM_CLK_CSI2_ESC_PRE_DIV] = imx_clk_divider2("csi2_esc_pre_div", "csi2_esc_cg", base + 0xbf80, 16, 3);
	clks[IMX8MM_CLK_PCIE2_CTRL_PRE_DIV] = imx_clk_divider2("pcie2_ctrl_pre_div", "pcie2_ctrl_cg", base + 0xc000, 16, 3);
	clks[IMX8MM_CLK_PCIE2_PHY_PRE_DIV] = imx_clk_divider2("pcie2_phy_pre_div", "pcie2_phy_cg", base + 0xc080, 16, 3);
	clks[IMX8MM_CLK_PCIE2_AUX_PRE_DIV] = imx_clk_divider2("pcie2_aux_pre_div", "pcie2_aux_cg", base + 0xc100, 16, 3);
	clks[IMX8MM_CLK_ECSPI3_PRE_DIV] = imx_clk_divider2("ecspi3_pre_div", "ecspi3_cg", base + 0xc180, 16, 3);
	clks[IMX8MM_CLK_PDM_PRE_DIV] = imx_clk_divider2("pdm_pre_div", "pdm_cg", base + 0xc200, 16, 3);
	clks[IMX8MM_CLK_VPU_H1_PRE_DIV] = imx_clk_divider2("vpu_h1_pre_div", "vpu_h1_cg", base + 0xc280, 16, 3);

	clks[IMX8MM_CLK_DRAM_ALT_DIV] = imx_clk_divider2("dram_alt_div", "dram_alt_pre_div", base + 0xa000, 0, 6);
	clks[IMX8MM_CLK_DRAM_APB_DIV] = imx_clk_divider2("dram_apb_div", "dram_apb_pre_div", base + 0xa080, 0, 6);
	clks[IMX8MM_CLK_VPU_G1_DIV] = imx_clk_divider2("vpu_g1_div", "vpu_g1_pre_div", base + 0xa100, 0, 6);
	clks[IMX8MM_CLK_VPU_G2_DIV] = imx_clk_divider2("vpu_g2_div", "vpu_g2_pre_div", base + 0xa180, 0, 6);
	clks[IMX8MM_CLK_DISP_DTRC_DIV] = imx_clk_divider2("disp_dtrc_div", "disp_dtrc_pre_div", base + 0xa200, 0, 6);
	clks[IMX8MM_CLK_DISP_DC8000_DIV] = imx_clk_divider2("disp_dc8000_div", "disp_dc8000_pre_div", base + 0xa280, 0, 6);
	clks[IMX8MM_CLK_PCIE1_CTRL_DIV] = imx_clk_divider2("pcie1_ctrl_div", "pcie1_ctrl_pre_div", base + 0xa300, 0, 6);
	clks[IMX8MM_CLK_PCIE1_PHY_DIV] = imx_clk_divider2("pcie1_phy_div", "pcie1_phy_pre_div", base + 0xa380, 0, 6);
	clks[IMX8MM_CLK_PCIE1_AUX_DIV] = imx_clk_divider2("pcie1_aux_div", "pcie1_aux_pre_div", base + 0xa400, 0, 6);
	clks[IMX8MM_CLK_DC_PIXEL_DIV] = imx_clk_divider2("dc_pixel_div", "dc_pixel_pre_div", base + 0xa480, 0, 6);
	clks[IMX8MM_CLK_LCDIF_PIXEL_DIV] = imx_clk_divider2("lcdif_pixel_div", "lcdif_pixel_pre_div", base + 0xa500, 0, 6);
	clks[IMX8MM_CLK_SAI1_DIV] = imx_clk_divider2("sai1_div", "sai1_pre_div", base + 0xa580, 0, 6);
	clks[IMX8MM_CLK_SAI2_DIV] = imx_clk_divider2("sai2_div", "sai2_pre_div", base + 0xa600, 0, 6);
	clks[IMX8MM_CLK_SAI3_DIV] = imx_clk_divider2("sai3_div", "sai3_pre_div", base + 0xa680, 0, 6);
	clks[IMX8MM_CLK_SAI4_DIV] = imx_clk_divider2("sai4_div", "sai4_pre_div", base + 0xa700, 0, 6);
	clks[IMX8MM_CLK_SAI5_DIV] = imx_clk_divider2("sai5_div", "sai5_pre_div", base + 0xa780, 0, 6);
	clks[IMX8MM_CLK_SAI6_DIV] = imx_clk_divider2("sai6_div", "sai6_pre_div", base + 0xa800, 0, 6);
	clks[IMX8MM_CLK_SPDIF1_DIV] = imx_clk_divider2("spdif1_div", "spdif1_pre_div", base + 0xa880, 0, 6);
	clks[IMX8MM_CLK_SPDIF2_DIV] = imx_clk_divider2("spdif2_div", "spdif2_pre_div", base + 0xa900, 0, 6);
	clks[IMX8MM_CLK_ENET_REF_DIV] = imx_clk_divider2("enet_ref_div", "enet_ref_pre_div", base + 0xa980, 0, 6);
	clks[IMX8MM_CLK_ENET_TIMER_DIV] = imx_clk_divider2("enet_timer_div", "enet_timer_pre_div", base + 0xaa00, 0, 6);
	clks[IMX8MM_CLK_ENET_PHY_REF_DIV] = imx_clk_divider2("enet_phy_div", "enet_phy_pre_div", base + 0xaa80, 0, 6);
	clks[IMX8MM_CLK_NAND_DIV] = imx_clk_divider2("nand_div", "nand_pre_div", base + 0xab00, 0, 6);
	clks[IMX8MM_CLK_QSPI_DIV] = imx_clk_divider2("qspi_div", "qspi_pre_div", base + 0xab80, 0, 6);
	clks[IMX8MM_CLK_USDHC1_DIV] = imx_clk_divider2("usdhc1_div", "usdhc1_pre_div", base + 0xac00, 0, 6);
	clks[IMX8MM_CLK_USDHC2_DIV] = imx_clk_divider2("usdhc2_div", "usdhc2_pre_div", base + 0xac80, 0, 6);
	clks[IMX8MM_CLK_I2C1_DIV] = imx_clk_divider2("i2c1_div", "i2c1_pre_div", base + 0xad00, 0, 6);
	clks[IMX8MM_CLK_I2C2_DIV] = imx_clk_divider2("i2c2_div", "i2c2_pre_div", base + 0xad80, 0, 6);
	clks[IMX8MM_CLK_I2C3_DIV] = imx_clk_divider2("i2c3_div", "i2c3_pre_div", base + 0xae00, 0, 6);
	clks[IMX8MM_CLK_I2C4_DIV] = imx_clk_divider2("i2c4_div", "i2c4_pre_div", base + 0xae80, 0, 6);
	clks[IMX8MM_CLK_UART1_DIV] = imx_clk_divider2("uart1_div", "uart1_pre_div", base + 0xaf00, 0, 6);
	clks[IMX8MM_CLK_UART2_DIV] = imx_clk_divider2("uart2_div", "uart2_pre_div", base + 0xaf80, 0, 6);
	clks[IMX8MM_CLK_UART3_DIV] = imx_clk_divider2("uart3_div", "uart3_pre_div", base + 0xb000, 0, 6);
	clks[IMX8MM_CLK_UART4_DIV] = imx_clk_divider2("uart4_div", "uart4_pre_div", base + 0xb080, 0, 6);
	clks[IMX8MM_CLK_USB_CORE_REF_DIV] = imx_clk_divider2("usb_core_ref_div", "usb_core_ref_pre_div", base + 0xb100, 0, 6);
	clks[IMX8MM_CLK_USB_PHY_REF_DIV] = imx_clk_divider2("usb_phy_ref_div", "usb_phy_ref_pre_div", base + 0xb180, 0, 6);
	clks[IMX8MM_CLK_ECSPI1_DIV] = imx_clk_divider2("ecspi1_div", "ecspi1_pre_div", base + 0xb280, 0, 6);
	clks[IMX8MM_CLK_ECSPI2_DIV] = imx_clk_divider2("ecspi2_div", "ecspi2_pre_div", base + 0xb300, 0, 6);
	clks[IMX8MM_CLK_PWM1_DIV] = imx_clk_divider2("pwm1_div", "pwm1_pre_div", base + 0xb380, 0, 6);
	clks[IMX8MM_CLK_PWM2_DIV] = imx_clk_divider2("pwm2_div", "pwm2_pre_div", base + 0xb400, 0, 6);
	clks[IMX8MM_CLK_PWM3_DIV] = imx_clk_divider2("pwm3_div", "pwm3_pre_div", base + 0xb480, 0, 6);
	clks[IMX8MM_CLK_PWM4_DIV] = imx_clk_divider2("pwm4_div", "pwm4_pre_div", base + 0xb500, 0, 6);
	clks[IMX8MM_CLK_GPT1_DIV] = imx_clk_divider2("gpt1_div", "gpt1_pre_div", base + 0xb580, 0, 6);
	clks[IMX8MM_CLK_WDOG_DIV] = imx_clk_divider2("wdog_div", "wdog_pre_div", base + 0xb900, 0, 6);
	clks[IMX8MM_CLK_WRCLK_DIV] = imx_clk_divider2("wrclk_div", "wrclk_pre_div", base + 0xb980, 0, 6);
	clks[IMX8MM_CLK_DSI_CORE_DIV] = imx_clk_divider2("dsi_core_div", "dsi_core_pre_div", base + 0xbb00, 0, 6);
	clks[IMX8MM_CLK_DSI_PHY_REF_DIV] = imx_clk_divider2("dsi_phy_ref_div", "dsi_phy_ref_pre_div", base + 0xbb80, 0, 6);
	clks[IMX8MM_CLK_DSI_DBI_DIV] = imx_clk_divider2("dsi_dbi_div", "dsi_dbi_pre_div", base + 0xbc00, 0, 6);
	clks[IMX8MM_CLK_USDHC3_DIV] = imx_clk_divider2("usdhc3_div", "usdhc3_pre_div", base + 0xbc80, 0, 6);
	clks[IMX8MM_CLK_CSI1_CORE_DIV] = imx_clk_divider2("csi1_core_div", "csi1_core_pre_div", base + 0xbd00, 0, 6);
	clks[IMX8MM_CLK_CSI1_PHY_REF_DIV] = imx_clk_divider2("csi1_phy_ref_div", "csi1_phy_ref_pre_div", base + 0xbd80, 0, 6);
	clks[IMX8MM_CLK_CSI1_ESC_DIV] = imx_clk_divider2("csi1_esc_div", "csi1_esc_pre_div", base + 0xbe00, 0, 6);
	clks[IMX8MM_CLK_CSI2_CORE_DIV] = imx_clk_divider2("csi2_core_div", "csi2_core_pre_div", base + 0xbe80, 0, 6);
	clks[IMX8MM_CLK_CSI2_PHY_REF_DIV] = imx_clk_divider2("csi2_phy_ref_div", "csi2_phy_ref_pre_div", base + 0xbf00, 0, 6);
	clks[IMX8MM_CLK_CSI2_ESC_DIV] = imx_clk_divider2("csi2_esc_div", "csi2_esc_pre_div", base + 0xbf80, 0, 6);
	clks[IMX8MM_CLK_PCIE2_CTRL_DIV] = imx_clk_divider2("pcie2_ctrl_div", "pcie2_ctrl_pre_div", base + 0xc000, 0, 6);
	clks[IMX8MM_CLK_PCIE2_PHY_DIV] = imx_clk_divider2("pcie2_phy_div", "pcie2_phy_pre_div", base + 0xc080, 0, 6);
	clks[IMX8MM_CLK_PCIE2_AUX_DIV] = imx_clk_divider2("pcie2_aux_div", "pcie2_aux_pre_div", base + 0xc100, 0, 6);
	clks[IMX8MM_CLK_ECSPI3_DIV] = imx_clk_divider2("ecspi3_div", "ecspi3_pre_div", base + 0xc180, 0, 6);
	clks[IMX8MM_CLK_PDM_DIV] = imx_clk_divider2("pdm_div", "pdm_pre_div", base + 0xc200, 0, 6);
	clks[IMX8MM_CLK_VPU_H1_DIV] = imx_clk_divider2("vpu_h1_div", "vpu_h1_pre_div", base + 0xc280, 0, 6);

	/* CCGR */
	clks[IMX8MM_CLK_ECSPI1_ROOT] = imx_clk_gate4("ecspi1_root_clk", "ecspi1_div", base + 0x4070, 0);
	clks[IMX8MM_CLK_ECSPI2_ROOT] = imx_clk_gate4("ecspi2_root_clk", "ecspi2_div", base + 0x4080, 0);
	clks[IMX8MM_CLK_ECSPI3_ROOT] = imx_clk_gate4("ecspi3_root_clk", "ecspi3_div", base + 0x4090, 0);
	clks[IMX8MM_CLK_ENET1_ROOT] = imx_clk_gate4("enet1_root_clk", "enet_axi_div", base + 0x40a0, 0);
	clks[IMX8MM_CLK_GPT1_ROOT] = imx_clk_gate4("gpt1_root_clk", "gpt1_div", base + 0x4100, 0);
	clks[IMX8MM_CLK_I2C1_ROOT] = imx_clk_gate4("i2c1_root_clk", "i2c1_div", base + 0x4170, 0);
	clks[IMX8MM_CLK_I2C2_ROOT] = imx_clk_gate4("i2c2_root_clk", "i2c2_div", base + 0x4180, 0);
	clks[IMX8MM_CLK_I2C3_ROOT] = imx_clk_gate4("i2c3_root_clk", "i2c3_div", base + 0x4190, 0);
	clks[IMX8MM_CLK_I2C4_ROOT] = imx_clk_gate4("i2c4_root_clk", "i2c4_div", base + 0x41a0, 0);
	clks[IMX8MM_CLK_MU_ROOT] = imx_clk_gate4("mu_root_clk", "ipg_root", base + 0x4210, 0);
	clks[IMX8MM_CLK_OCOTP_ROOT] = imx_clk_gate4("ocotp_root_clk", "ipg_root", base + 0x4220, 0);
	clks[IMX8MM_CLK_PCIE1_ROOT] = imx_clk_gate4("pcie1_root_clk", "pcie1_ctrl_div", base + 0x4250, 0);
	clks[IMX8MM_CLK_PWM1_ROOT] = imx_clk_gate4("pwm1_root_clk", "pwm1_div", base + 0x4280, 0);
	clks[IMX8MM_CLK_PWM2_ROOT] = imx_clk_gate4("pwm2_root_clk", "pwm2_div", base + 0x4290, 0);
	clks[IMX8MM_CLK_PWM3_ROOT] = imx_clk_gate4("pwm3_root_clk", "pwm3_div", base + 0x42a0, 0);
	clks[IMX8MM_CLK_PWM4_ROOT] = imx_clk_gate4("pwm4_root_clk", "pwm4_div", base + 0x42b0, 0);
	clks[IMX8MM_CLK_QSPI_ROOT] = imx_clk_gate4("qspi_root_clk", "qspi_div", base + 0x42f0, 0);
	clks[IMX8MM_CLK_NAND_ROOT] = imx_clk_gate4("nand_root_clk", "nand_div", base + 0x4300, 0);
	clks[IMX8MM_CLK_SAI1_ROOT] = imx_clk_gate2_shared2("sai1_root_clk", "sai1_div", base + 0x4330, 0, &share_count_sai1);
	clks[IMX8MM_CLK_SAI1_IPG] = imx_clk_gate2_shared2("sai1_ipg_clk", "ipg_audio_root", base + 0x4330, 0, &share_count_sai1);
	clks[IMX8MM_CLK_SAI2_ROOT] = imx_clk_gate2_shared2("sai2_root_clk", "sai2_div", base + 0x4340, 0, &share_count_sai2);
	clks[IMX8MM_CLK_SAI2_IPG] = imx_clk_gate2_shared2("sai2_ipg_clk", "ipg_root", base + 0x4340, 0, &share_count_sai2);
	clks[IMX8MM_CLK_SAI3_ROOT] = imx_clk_gate2_shared2("sai3_root_clk", "sai3_div", base + 0x4350, 0, &share_count_sai3);
	clks[IMX8MM_CLK_SAI3_IPG] = imx_clk_gate2_shared2("sai3_ipg_clk", "ipg_root", base + 0x4350, 0, &share_count_sai3);
	clks[IMX8MM_CLK_SAI4_ROOT] = imx_clk_gate2_shared2("sai4_root_clk", "sai4_div", base + 0x4360, 0, &share_count_sai4);
	clks[IMX8MM_CLK_SAI4_IPG] = imx_clk_gate2_shared2("sai4_ipg_clk", "ipg_audio_root", base + 0x4360, 0, &share_count_sai4);
	clks[IMX8MM_CLK_SAI5_ROOT] = imx_clk_gate2_shared2("sai5_root_clk", "sai5_div", base + 0x4370, 0, &share_count_sai5);
	clks[IMX8MM_CLK_SAI5_IPG] = imx_clk_gate2_shared2("sai5_ipg_clk", "ipg_audio_root", base + 0x4370, 0, &share_count_sai5);
	clks[IMX8MM_CLK_SAI6_ROOT] = imx_clk_gate2_shared2("sai6_root_clk", "sai6_div", base + 0x4380, 0, &share_count_sai6);
	clks[IMX8MM_CLK_SAI6_IPG] = imx_clk_gate2_shared2("sai6_ipg_clk", "ipg_audio_root", base + 0x4380, 0, &share_count_sai6);
	clks[IMX8MM_CLK_UART1_ROOT] = imx_clk_gate4("uart1_root_clk", "uart1_div", base + 0x4490, 0);
	clks[IMX8MM_CLK_UART2_ROOT] = imx_clk_gate4("uart2_root_clk", "uart2_div", base + 0x44a0, 0);
	clks[IMX8MM_CLK_UART3_ROOT] = imx_clk_gate4("uart3_root_clk", "uart3_div", base + 0x44b0, 0);
	clks[IMX8MM_CLK_UART4_ROOT] = imx_clk_gate4("uart4_root_clk", "uart4_div", base + 0x44c0, 0);
	clks[IMX8MM_CLK_USB1_CTRL_ROOT] = imx_clk_gate4("usb1_ctrl_root_clk", "usb_core_ref_div", base + 0x44d0, 0);
	clks[IMX8MM_CLK_GPU3D_ROOT] = imx_clk_gate4("gpu3d_root_clk", "gpu3d_div", base + 0x44f0, 0);
	clks[IMX8MM_CLK_USDHC1_ROOT] = imx_clk_gate4("usdhc1_root_clk", "usdhc1_div", base + 0x4510, 0);
	clks[IMX8MM_CLK_USDHC2_ROOT] = imx_clk_gate4("usdhc2_root_clk", "usdhc2_div", base + 0x4520, 0);
	clks[IMX8MM_CLK_WDOG1_ROOT] = imx_clk_gate4("wdog1_root_clk", "wdog_div", base + 0x4530, 0);
	clks[IMX8MM_CLK_WDOG2_ROOT] = imx_clk_gate4("wdog2_root_clk", "wdog_div", base + 0x4540, 0);
	clks[IMX8MM_CLK_WDOG3_ROOT] = imx_clk_gate4("wdog3_root_clk", "wdog_div", base + 0x4550, 0);
	clks[IMX8MM_CLK_VPU_G1_ROOT] = imx_clk_gate4("vpu_g1_root_clk", "vpu_g1_div", base + 0x4560, 0);
	clks[IMX8MM_CLK_GPU_BUS_ROOT] = imx_clk_gate4("gpu_root_clk", "gpu_axi_div", base + 0x4570, 0);
	clks[IMX8MM_CLK_VPU_H1_ROOT] = imx_clk_gate4("vpu_h1_root_clk", "vpu_h1_div", base + 0x4590, 0);
	clks[IMX8MM_CLK_VPU_G2_ROOT] = imx_clk_gate4("vpu_g2_root_clk", "vpu_g2_div", base + 0x45a0, 0);
	clks[IMX8MM_CLK_PDM_ROOT] = imx_clk_gate2_shared2("pdm_root_clk", "pdm_div", base + 0x45b0, 0, &share_count_pdm);
	clks[IMX8MM_CLK_PDM_IPG]  = imx_clk_gate2_shared2("pdm_ipg_clk", "ipg_audio_root", base + 0x45b0, 0, &share_count_pdm);
	clks[IMX8MM_CLK_DISP_ROOT] = imx_clk_gate2_shared2("disp_root_clk", "disp_dc8000_div", base + 0x45d0, 0, &share_count_dcss);
	clks[IMX8MM_CLK_DISP_AXI_ROOT]  = imx_clk_gate2_shared2("disp_axi_root_clk", "disp_axi_div", base + 0x45d0, 0, &share_count_dcss);
	clks[IMX8MM_CLK_DISP_APB_ROOT]  = imx_clk_gate2_shared2("disp_apb_root_clk", "disp_apb_div", base + 0x45d0, 0, &share_count_dcss);
	clks[IMX8MM_CLK_DISP_RTRM_ROOT] = imx_clk_gate2_shared2("disp_rtrm_root_clk", "disp_rtrm_div", base + 0x45d0, 0, &share_count_dcss);
	clks[IMX8MM_CLK_USDHC3_ROOT] = imx_clk_gate4("usdhc3_root_clk", "usdhc3_div", base + 0x45e0, 0);
	clks[IMX8MM_CLK_TMU_ROOT] = imx_clk_gate4("tmu_root_clk", "ipg_root", base + 0x4620, 0);
	clks[IMX8MM_CLK_VPU_DEC_ROOT] = imx_clk_gate4("vpu_dec_root_clk", "vpu_bus_div", base + 0x4630, 0);
	clks[IMX8MM_CLK_SDMA1_ROOT] = imx_clk_gate4("sdma1_clk", "ipg_root", base + 0x43a0, 0);
	clks[IMX8MM_CLK_SDMA2_ROOT] = imx_clk_gate4("sdma2_clk", "ipg_audio_root", base + 0x43b0, 0);
	clks[IMX8MM_CLK_SDMA3_ROOT] = imx_clk_gate4("sdma3_clk", "ipg_audio_root", base + 0x45f0, 0);
	clks[IMX8MM_CLK_GPU2D_ROOT] = imx_clk_gate4("gpu2d_root_clk", "gpu2d_div", base + 0x4660, 0);

	clks[IMX8MM_CLK_GPT_3M] = imx_clk_fixed_factor("gpt_3m", "osc_24m", 1, 8);

	clks[IMX8MM_CLK_ARM] = imx_clk_cpu("arm", "arm_a53_div",
					   clks[IMX8MM_CLK_A53_DIV],
					   clks[IMX8MM_CLK_A53_SRC],
					   clks[IMX8MM_ARM_PLL_OUT],
					   clks[IMX8MM_CLK_24M]);

	for (i = 0; i < IMX8MM_CLK_END; i++)
		if (IS_ERR(clks[i]))
			pr_err("i.MX8mm clk %u register failed with %ld\n",
			       i, PTR_ERR(clks[i]));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clks_init_on); i++)
		clk_prepare_enable(clks[clks_init_on[i]]);

	/* increase NOC clock to design target */
	clk_set_rate(clks[IMX8MM_SYS_PLL3], 750000000);
	clk_set_parent(clks[IMX8MM_CLK_NOC_SRC], clks[IMX8MM_SYS_PLL3_OUT]);

	pr_info("i.MX8MM clock driver init done\n");
}

CLK_OF_DECLARE(imx8mm, "fsl,imx8mm-ccm", imx8mm_clocks_init);
