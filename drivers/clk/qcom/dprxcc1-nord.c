// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,nord-dprxcc.h>

#include "clk-alpha-pll.h"
#include "clk-branch.h"
#include "clk-rcg.h"
#include "clk-regmap.h"
#include "clk-regmap-divider.h"
#include "common.h"
#include "reset.h"

enum {
	DT_BI_TCXO,
	DT_AHB_CLK,
	DT_SLEEP_CLK,
};

enum {
	P_BI_TCXO,
	P_DPRX_1_DPRX_CC_PLL0_OUT_EVEN,
	P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN,
	P_DPRX_1_DPRX_CC_PLL0_OUT_ODD,
	P_DPRX_1_DPRX_CC_PLL1_OUT_EVEN,
	P_DPRX_1_DPRX_CC_PLL1_OUT_MAIN,
	P_SLEEP_CLK,
};

#define DPRX_CC_CORE_AHB_DIV_CFG	0x80b0
#define DPRX_CC_CORE_AHB_DIV		GENMASK(3, 0)
#define DPRX_CC_CORE_AUD_DIV_CFG	0x80d0
#define DPRX_CC_CORE_AUD_DIV		GENMASK(8, 0)

static const struct pll_vco lucid_ole_vco[] = {
	{ 249600000, 2300000000, 0 },
};

static const struct pll_vco zonda_ole_vco[] = {
	{ 700000000, 3600000000, 0 },
};

/* 550.0 MHz Configuration */
static const struct alpha_pll_config dprx_1_dprx_cc_pll0_config = {
	.l = 0x1c,
	.alpha = 0xa555,
	.config_ctl_val = 0x20485699,
	.config_ctl_hi_val = 0x00182261,
	.config_ctl_hi1_val = 0x82aa299c,
	.test_ctl_val = 0x00000000,
	.test_ctl_hi_val = 0x00000003,
	.test_ctl_hi1_val = 0x00009000,
	.test_ctl_hi2_val = 0x00000034,
	.user_ctl_val = 0x00000000,
	.user_ctl_hi_val = 0x00400005,
};

static struct clk_alpha_pll dprx_1_dprx_cc_pll0 = {
	.offset = 0x0,
	.config = &dprx_1_dprx_cc_pll0_config,
	.vco_table = lucid_ole_vco,
	.num_vco = ARRAY_SIZE(lucid_ole_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_LUCID_OLE],
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "dprx_1_dprx_cc_pll0",
			.parent_data = &(const struct clk_parent_data) {
				.index = DT_BI_TCXO,
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_lucid_evo_ops,
		},
	},
};

/* 1080.0 MHz Configuration */
static const struct alpha_pll_config dprx_1_dprx_cc_pll1_config = {
	.l = 0x38,
	.alpha = 0x4000000,
	.config_ctl_val = 0x08240800,
	.config_ctl_hi_val = 0x05008001,
	.config_ctl_hi1_val = 0x00000000,
	.config_ctl_hi2_val = 0x00000000,
	.user_ctl_val = 0x00000000,
	.user_ctl_hi_val = 0x02000080,
};

static struct clk_alpha_pll dprx_1_dprx_cc_pll1 = {
	.offset = 0x1000,
	.alpha_width = 28,
	.config = &dprx_1_dprx_cc_pll1_config,
	.vco_table = zonda_ole_vco,
	.num_vco = ARRAY_SIZE(zonda_ole_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_ZONDA_OLE],
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "dprx_1_dprx_cc_pll1",
			.parent_data = &(const struct clk_parent_data) {
				.index = DT_BI_TCXO,
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_zonda_ole_ops,
		},
	},
};

static const struct parent_map dprx_cc_1_parent_map_0[] = {
	{ P_BI_TCXO, 0 },
	{ P_DPRX_1_DPRX_CC_PLL0_OUT_EVEN, 1 },
	{ P_DPRX_1_DPRX_CC_PLL0_OUT_ODD, 2 },
	{ P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 4 },
	{ P_SLEEP_CLK, 5 },
};

static const struct clk_parent_data dprx_cc_1_parent_data_0[] = {
	{ .index = DT_BI_TCXO },
	{ .hw = &dprx_1_dprx_cc_pll0.clkr.hw },
	{ .hw = &dprx_1_dprx_cc_pll0.clkr.hw },
	{ .hw = &dprx_1_dprx_cc_pll0.clkr.hw },
	{ .index = DT_SLEEP_CLK },
};

static const struct parent_map dprx_cc_1_parent_map_1[] = {
	{ P_BI_TCXO, 0 },
	{ P_DPRX_1_DPRX_CC_PLL1_OUT_EVEN, 3 },
	{ P_SLEEP_CLK, 5 },
	{ P_DPRX_1_DPRX_CC_PLL1_OUT_MAIN, 6 },
};

static const struct clk_parent_data dprx_cc_1_parent_data_1[] = {
	{ .index = DT_BI_TCXO },
	{ .hw = &dprx_1_dprx_cc_pll1.clkr.hw },
	{ .index = DT_SLEEP_CLK },
	{ .hw = &dprx_1_dprx_cc_pll1.clkr.hw },
};

static const struct freq_tbl ftbl_dprx_1_dprx_cc_audio_clk_src[] = {
	F(216000000, P_DPRX_1_DPRX_CC_PLL1_OUT_MAIN, 5, 0, 0),
	F(300000000, P_DPRX_1_DPRX_CC_PLL1_OUT_MAIN, 5, 0, 0),
	F(360000000, P_DPRX_1_DPRX_CC_PLL1_OUT_MAIN, 5, 0, 0),
	{ }
};

static struct clk_rcg2 dprx_1_dprx_cc_audio_clk_src = {
	.cmd_rcgr = 0x80c8,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = dprx_cc_1_parent_map_1,
	.freq_tbl = ftbl_dprx_1_dprx_cc_audio_clk_src,
	.hw_clk_ctrl = true,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "dprx_1_dprx_cc_audio_clk_src",
		.parent_data = dprx_cc_1_parent_data_1,
		.num_parents = ARRAY_SIZE(dprx_cc_1_parent_data_1),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_shared_ops,
	},
};

static const struct freq_tbl ftbl_dprx_1_dprx_cc_core_rcg0_clk_src[] = {
	F(220000000, P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 2.5, 0, 0),
	F(440000000, P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 2.5, 0, 0),
	{ }
};

static struct clk_rcg2 dprx_1_dprx_cc_core_rcg0_clk_src = {
	.cmd_rcgr = 0x8004,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = dprx_cc_1_parent_map_0,
	.freq_tbl = ftbl_dprx_1_dprx_cc_core_rcg0_clk_src,
	.hw_clk_ctrl = true,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "dprx_1_dprx_cc_core_rcg0_clk_src",
		.parent_data = dprx_cc_1_parent_data_0,
		.num_parents = ARRAY_SIZE(dprx_cc_1_parent_data_0),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_shared_ops,
	},
};

static const struct freq_tbl ftbl_dprx_1_dprx_cc_core_rcg2_clk_src[] = {
	F(275000000, P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(440000000, P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 2.5, 0, 0),
	F(550000000, P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 2, 0, 0),
	{ }
};

static struct clk_rcg2 dprx_1_dprx_cc_core_rcg2_clk_src = {
	.cmd_rcgr = 0x8068,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = dprx_cc_1_parent_map_0,
	.freq_tbl = ftbl_dprx_1_dprx_cc_core_rcg2_clk_src,
	.hw_clk_ctrl = true,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "dprx_1_dprx_cc_core_rcg2_clk_src",
		.parent_data = dprx_cc_1_parent_data_0,
		.num_parents = ARRAY_SIZE(dprx_cc_1_parent_data_0),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_shared_ops,
	},
};

static const struct freq_tbl ftbl_dprx_1_dprx_cc_core_rcg3_clk_src[] = {
	F(275000000, P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(550000000, P_DPRX_1_DPRX_CC_PLL0_OUT_MAIN, 2, 0, 0),
	{ }
};

static struct clk_rcg2 dprx_1_dprx_cc_core_rcg3_clk_src = {
	.cmd_rcgr = 0x8098,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = dprx_cc_1_parent_map_0,
	.freq_tbl = ftbl_dprx_1_dprx_cc_core_rcg3_clk_src,
	.hw_clk_ctrl = true,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "dprx_1_dprx_cc_core_rcg3_clk_src",
		.parent_data = dprx_cc_1_parent_data_0,
		.num_parents = ARRAY_SIZE(dprx_cc_1_parent_data_0),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_shared_ops,
	},
};

static struct clk_regmap_div dprx_1_dprx_cc_core_ahb_div_clk_src = {
	.reg = 0x80b0,
	.shift = 0,
	.width = 4,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "dprx_1_dprx_cc_core_ahb_div_clk_src",
		.parent_hws = (const struct clk_hw*[]) {
			&dprx_1_dprx_cc_core_rcg3_clk_src.clkr.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_regmap_div_ro_ops,
	},
};

static struct clk_regmap_div dprx_1_dprx_cc_core_aud_div_clk_src = {
	.reg = 0x80d0,
	.shift = 0,
	.width = 9,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "dprx_1_dprx_cc_core_aud_div_clk_src",
		.parent_hws = (const struct clk_hw*[]) {
			&dprx_1_dprx_cc_audio_clk_src.clkr.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_regmap_div_ro_ops,
	},
};

static struct clk_branch dprx_1_dprx_cc_ahb_clk = {
	.halt_reg = 0x80b4,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x80b4,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x80b4,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "dprx_1_dprx_cc_ahb_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&dprx_1_dprx_cc_core_ahb_div_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch dprx_1_dprx_cc_aud_clk = {
	.halt_reg = 0x80d4,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x80d4,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x80d4,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "dprx_1_dprx_cc_aud_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&dprx_1_dprx_cc_core_aud_div_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch dprx_1_dprx_cc_mdp_mem_clk = {
	.halt_reg = 0x8088,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x8088,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x8088,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "dprx_1_dprx_cc_mdp_mem_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&dprx_1_dprx_cc_core_rcg2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch dprx_1_dprx_cc_mmcx_xo_clk = {
	.halt_reg = 0x80d8,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x80d8,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x80d8,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "dprx_1_dprx_cc_mmcx_xo_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch dprx_1_dprx_cc_vid_mem_clk = {
	.halt_reg = 0x8028,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x8028,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x8028,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "dprx_1_dprx_cc_vid_mem_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&dprx_1_dprx_cc_core_rcg0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_regmap *dprx_cc_1_nord_clocks[] = {
	[DPRX_CC_AHB_CLK] = &dprx_1_dprx_cc_ahb_clk.clkr,
	[DPRX_CC_AUD_CLK] = &dprx_1_dprx_cc_aud_clk.clkr,
	[DPRX_CC_AUDIO_CLK_SRC] = &dprx_1_dprx_cc_audio_clk_src.clkr,
	[DPRX_CC_CORE_AHB_DIV_CLK_SRC] = &dprx_1_dprx_cc_core_ahb_div_clk_src.clkr,
	[DPRX_CC_CORE_AUD_DIV_CLK_SRC] = &dprx_1_dprx_cc_core_aud_div_clk_src.clkr,
	[DPRX_CC_CORE_RCG0_CLK_SRC] = &dprx_1_dprx_cc_core_rcg0_clk_src.clkr,
	[DPRX_CC_CORE_RCG2_CLK_SRC] = &dprx_1_dprx_cc_core_rcg2_clk_src.clkr,
	[DPRX_CC_CORE_RCG3_CLK_SRC] = &dprx_1_dprx_cc_core_rcg3_clk_src.clkr,
	[DPRX_CC_MDP_MEM_CLK] = &dprx_1_dprx_cc_mdp_mem_clk.clkr,
	[DPRX_CC_MMCX_XO_CLK] = &dprx_1_dprx_cc_mmcx_xo_clk.clkr,
	[DPRX_CC_PLL0] = &dprx_1_dprx_cc_pll0.clkr,
	[DPRX_CC_PLL1] = &dprx_1_dprx_cc_pll1.clkr,
	[DPRX_CC_VID_MEM_CLK] = &dprx_1_dprx_cc_vid_mem_clk.clkr,
};

static const struct qcom_reset_map dprx_cc_1_nord_resets[] = {
	[DPRX_CC_DPRX_BLK_BCR] = { 0x8000 },
};

static struct clk_alpha_pll *dprx_cc_1_nord_plls[] = {
	&dprx_1_dprx_cc_pll0,
	&dprx_1_dprx_cc_pll1,
};

static const struct regmap_config dprx_cc_1_nord_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x8104,
	.fast_io = true,
};

static void dprx_cc_1_nord_regs_configure(struct device *dev, struct regmap *regmap)
{
	/* Configure dividers for core audio and AHB clocks. */
	regmap_update_bits(regmap, DPRX_CC_CORE_AUD_DIV_CFG,
			   DPRX_CC_CORE_AUD_DIV, 0x13);
	regmap_update_bits(regmap, DPRX_CC_CORE_AHB_DIV_CFG,
			   DPRX_CC_CORE_AHB_DIV, 0x0a);
}

static const struct qcom_cc_driver_data dprx_cc_1_nord_driver_data = {
	.alpha_plls = dprx_cc_1_nord_plls,
	.num_alpha_plls = ARRAY_SIZE(dprx_cc_1_nord_plls),
	.clk_regs_configure = dprx_cc_1_nord_regs_configure,
};

static const struct qcom_cc_desc dprx_cc_1_nord_desc = {
	.config = &dprx_cc_1_nord_regmap_config,
	.clks = dprx_cc_1_nord_clocks,
	.num_clks = ARRAY_SIZE(dprx_cc_1_nord_clocks),
	.resets = dprx_cc_1_nord_resets,
	.num_resets = ARRAY_SIZE(dprx_cc_1_nord_resets),
	.use_rpm = true,
	.driver_data = &dprx_cc_1_nord_driver_data,
};

static const struct of_device_id dprx_cc_1_nord_match_table[] = {
	{ .compatible = "qcom,nord-dprxcc1" },
	{ }
};
MODULE_DEVICE_TABLE(of, dprx_cc_1_nord_match_table);

static int dprx_cc_1_nord_probe(struct platform_device *pdev)
{
	return qcom_cc_probe(pdev, &dprx_cc_1_nord_desc);
}

static struct platform_driver dprx_cc_1_nord_driver = {
	.probe = dprx_cc_1_nord_probe,
	.driver = {
		.name = "dprxcc1-nord",
		.of_match_table = dprx_cc_1_nord_match_table,
	},
};

module_platform_driver(dprx_cc_1_nord_driver);

MODULE_DESCRIPTION("QTI DPRXCC1 NORD Driver");
MODULE_LICENSE("GPL");
