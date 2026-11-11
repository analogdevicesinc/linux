// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,glymur-av1ecc.h>

#include "clk-alpha-pll.h"
#include "clk-branch.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-regmap.h"
#include "clk-regmap-divider.h"
#include "common.h"
#include "gdsc.h"
#include "reset.h"

enum {
	DT_BI_TCXO,
	DT_IFACE,
};

enum {
	P_AV1E_CC_PLL0_OUT_EVEN,
	P_AV1E_CC_PLL0_OUT_MAIN,
	P_AV1E_CC_PLL0_OUT_ODD,
	P_BI_TCXO,
};

static const struct pll_vco taycan_eko_t_vco[] = {
	{ 249600000, 2500000000, 0 },
};

/* 600.0 MHz Configuration */
static const struct alpha_pll_config av1e_cc_pll0_config = {
	.l = 0x1f,
	.cal_l = 0x48,
	.alpha = 0x4000,
	.config_ctl_val = 0x25c400e7,
	.config_ctl_hi_val = 0x0a8060e0,
	.config_ctl_hi1_val = 0xf51dea20,
	.user_ctl_val = 0x00000008,
	.user_ctl_hi_val = 0x00000002,
};

static struct clk_alpha_pll av1e_cc_pll0 = {
	.offset = 0x0,
	.config = &av1e_cc_pll0_config,
	.vco_table = taycan_eko_t_vco,
	.num_vco = ARRAY_SIZE(taycan_eko_t_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_TAYCAN_EKO_T],
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_pll0",
			.parent_data = &(const struct clk_parent_data) {
				.index = DT_BI_TCXO,
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_taycan_eko_t_ops,
		},
	},
};

static const struct parent_map av1e_cc_parent_map_0[] = {
	{ P_BI_TCXO, 0 },
	{ P_AV1E_CC_PLL0_OUT_MAIN, 1 },
	{ P_AV1E_CC_PLL0_OUT_ODD, 2 },
	{ P_AV1E_CC_PLL0_OUT_EVEN, 3 },
};

static const struct clk_parent_data av1e_cc_parent_data_0[] = {
	{ .index = DT_BI_TCXO },
	{ .hw = &av1e_cc_pll0.clkr.hw },
	{ .hw = &av1e_cc_pll0.clkr.hw },
	{ .hw = &av1e_cc_pll0.clkr.hw },
};

static const struct parent_map av1e_cc_parent_map_1[] = {
	{ P_BI_TCXO, 0 },
};

static const struct clk_parent_data av1e_cc_parent_data_1[] = {
	{ .index = DT_BI_TCXO },
};

static const struct freq_tbl ftbl_av1e_cc_av1e_core_clk_src[] = {
	F(300000000, P_AV1E_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(400000000, P_AV1E_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(600000000, P_AV1E_CC_PLL0_OUT_MAIN, 2, 0, 0),
	{ }
};

static struct clk_rcg2 av1e_cc_av1e_core_clk_src = {
	.cmd_rcgr = 0x4018,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = av1e_cc_parent_map_0,
	.freq_tbl = ftbl_av1e_cc_av1e_core_clk_src,
	.hw_clk_ctrl = true,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "av1e_cc_av1e_core_clk_src",
		.parent_data = av1e_cc_parent_data_0,
		.num_parents = ARRAY_SIZE(av1e_cc_parent_data_0),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_shared_ops,
	},
};

static const struct freq_tbl ftbl_av1e_cc_av1e_xo_clk_src[] = {
	F(19200000, P_BI_TCXO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 av1e_cc_av1e_xo_clk_src = {
	.cmd_rcgr = 0x601c,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = av1e_cc_parent_map_1,
	.freq_tbl = ftbl_av1e_cc_av1e_xo_clk_src,
	.hw_clk_ctrl = true,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "av1e_cc_av1e_xo_clk_src",
		.parent_data = av1e_cc_parent_data_1,
		.num_parents = ARRAY_SIZE(av1e_cc_parent_data_1),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_regmap_div av1e_cc_av1e_noc_ahb_div_clk_src = {
	.reg = 0x5020,
	.shift = 0,
	.width = 4,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "av1e_cc_av1e_noc_ahb_div_clk_src",
		.parent_hws = (const struct clk_hw*[]) {
			&av1e_cc_av1e_core_clk_src.clkr.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_regmap_div_ro_ops,
	},
};

static struct clk_branch av1e_cc_ahb_clk = {
	.halt_reg = 0x6000,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x6000,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_ahb_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_noc_ahb_div_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch av1e_cc_av1e_core_apb_clk = {
	.halt_reg = 0x4044,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4044,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_av1e_core_apb_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_core_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch av1e_cc_av1e_core_axi_clk = {
	.halt_reg = 0x4040,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4040,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_av1e_core_axi_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_core_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch av1e_cc_av1e_core_clk = {
	.halt_reg = 0x4030,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4030,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_av1e_core_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_core_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch av1e_cc_av1e_gdsc_noc_ahb_clk = {
	.halt_reg = 0x5024,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5024,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_av1e_gdsc_noc_ahb_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_noc_ahb_div_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch av1e_cc_av1e_noc_ahb_clk = {
	.halt_reg = 0x5028,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x5028,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_av1e_noc_ahb_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_noc_ahb_div_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch av1e_cc_av1e_noc_core_axi_clk = {
	.halt_reg = 0x5004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5004,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_av1e_noc_core_axi_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_core_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch av1e_cc_av1e_noc_xo_clk = {
	.halt_reg = 0x6038,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x6038,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "av1e_cc_av1e_noc_xo_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&av1e_cc_av1e_xo_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct gdsc av1e_cc_core_gdsc = {
	.gdscr = 0x4004,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0xf,
	.pd = {
		.name = "av1e_cc_core_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE,
};

static struct clk_regmap *av1e_cc_glymur_clocks[] = {
	[AV1E_CC_AHB_CLK] = &av1e_cc_ahb_clk.clkr,
	[AV1E_CC_AV1E_CORE_APB_CLK] = &av1e_cc_av1e_core_apb_clk.clkr,
	[AV1E_CC_AV1E_CORE_AXI_CLK] = &av1e_cc_av1e_core_axi_clk.clkr,
	[AV1E_CC_AV1E_CORE_CLK] = &av1e_cc_av1e_core_clk.clkr,
	[AV1E_CC_AV1E_CORE_CLK_SRC] = &av1e_cc_av1e_core_clk_src.clkr,
	[AV1E_CC_AV1E_GDSC_NOC_AHB_CLK] = &av1e_cc_av1e_gdsc_noc_ahb_clk.clkr,
	[AV1E_CC_AV1E_NOC_AHB_CLK] = &av1e_cc_av1e_noc_ahb_clk.clkr,
	[AV1E_CC_AV1E_NOC_AHB_DIV_CLK_SRC] = &av1e_cc_av1e_noc_ahb_div_clk_src.clkr,
	[AV1E_CC_AV1E_NOC_CORE_AXI_CLK] = &av1e_cc_av1e_noc_core_axi_clk.clkr,
	[AV1E_CC_AV1E_NOC_XO_CLK] = &av1e_cc_av1e_noc_xo_clk.clkr,
	[AV1E_CC_AV1E_XO_CLK_SRC] = &av1e_cc_av1e_xo_clk_src.clkr,
	[AV1E_CC_PLL0] = &av1e_cc_pll0.clkr,
};

static struct gdsc *av1e_cc_glymur_gdscs[] = {
	[AV1E_CC_CORE_GDSC] = &av1e_cc_core_gdsc,
};

static const struct qcom_reset_map av1e_cc_glymur_resets[] = {
	[AV1E_CC_AV1E_CORE_AXI_CLK_ARES] = { 0x4040, 2 },
	[AV1E_CC_AV1E_CORE_CLK_ARES] = { 0x4030, 2 },
	[AV1E_CC_AV1E_GDSC_NOC_AHB_CLK_ARES] = { 0x5024, 2 },
	[AV1E_CC_AV1E_NOC_CORE_AXI_CLK_ARES] = { 0x5004, 2 },
	[AV1E_CC_AV1E_NOC_XO_CLK_ARES] = { 0x6038, 2 },
	[AV1E_CC_CORE_BCR] = { 0x4000 },
	[AV1E_CC_NOC_BCR] = { 0x5000 },
};

static struct clk_alpha_pll *av1e_cc_glymur_plls[] = {
	&av1e_cc_pll0,
};

static const u32 av1e_cc_glymur_critical_cbcrs[] = {
	0x6034, /* AV1E_CC_AV1E_CC_XO_CLK */
	0x6054, /* AV1E_CC_SLEEP_CLK */
};

static const struct regmap_config av1e_cc_glymur_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0xa01c,
	.fast_io = true,
};

static const struct qcom_cc_driver_data av1e_cc_glymur_driver_data = {
	.alpha_plls = av1e_cc_glymur_plls,
	.num_alpha_plls = ARRAY_SIZE(av1e_cc_glymur_plls),
	.clk_cbcrs = av1e_cc_glymur_critical_cbcrs,
	.num_clk_cbcrs = ARRAY_SIZE(av1e_cc_glymur_critical_cbcrs),
};

static const struct qcom_cc_desc av1e_cc_glymur_desc = {
	.config = &av1e_cc_glymur_regmap_config,
	.clks = av1e_cc_glymur_clocks,
	.num_clks = ARRAY_SIZE(av1e_cc_glymur_clocks),
	.resets = av1e_cc_glymur_resets,
	.num_resets = ARRAY_SIZE(av1e_cc_glymur_resets),
	.gdscs = av1e_cc_glymur_gdscs,
	.num_gdscs = ARRAY_SIZE(av1e_cc_glymur_gdscs),
	.use_rpm = true,
	.driver_data = &av1e_cc_glymur_driver_data,
};

static const struct of_device_id av1e_cc_glymur_match_table[] = {
	{ .compatible = "qcom,glymur-av1ecc" },
	{ }
};
MODULE_DEVICE_TABLE(of, av1e_cc_glymur_match_table);

static int av1e_cc_glymur_probe(struct platform_device *pdev)
{
	return qcom_cc_probe(pdev, &av1e_cc_glymur_desc);
}

static struct platform_driver av1e_cc_glymur_driver = {
	.probe = av1e_cc_glymur_probe,
	.driver = {
		.name = "av1ecc-glymur",
		.of_match_table = av1e_cc_glymur_match_table,
	},
};

module_platform_driver(av1e_cc_glymur_driver);

MODULE_DESCRIPTION("QTI AV1ECC Glymur Driver");
MODULE_LICENSE("GPL");
