// SPDX-License-Identifier: (GPL-2.0-only OR MIT)
/*
 * Amlogic S7 PLL Clock Controller Driver
 *
 * Copyright (c) 2026 BayLibre, SAS.
 * Author: Carlo Caione <ccaione@baylibre.com>
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "clk-regmap.h"
#include "clk-pll.h"
#include "meson-clkc-utils.h"
#include <dt-bindings/clock/amlogic,s7-pll-clkc.h>

#define FIXPLL_CTRL0			0x040
#define FIXPLL_CTRL1			0x044
#define FIXPLL_STS			0x05c
#define GP0PLL_CTRL0			0x080
#define GP0PLL_CTRL1			0x084
#define GP0PLL_CTRL2			0x088
#define GP0PLL_CTRL3			0x08c
#define GP0PLL_STS			0x09c
#define HIFI0PLL_CTRL0			0x100
#define HIFI0PLL_CTRL1			0x104
#define HIFI0PLL_CTRL2			0x108
#define HIFI0PLL_CTRL3			0x10c
#define HIFI1PLL_CTRL0			0x110
#define HIFI1PLL_CTRL1			0x114
#define HIFI1PLL_CTRL2			0x118
#define HIFI1PLL_CTRL3			0x11c
#define HIFI0PLL_STS			0x120
#define HIFI1PLL_STS			0x124
#define HDMIPLL_CTRL0			0x1c0
#define HDMIPLL_CTRL1			0x1c4
#define HDMIPLL_CTRL2			0x1c8
#define HDMIPLL_CTRL3			0x1cc
#define HDMIPLL_STS			0x1dc

static struct clk_regmap s7_fixed_pll_vco = {
	.data = &(struct meson_clk_pll_data) {
		.en = {
			.reg_off = FIXPLL_CTRL0,
			.shift   = 28,
			.width   = 1,
		},
		.m = {
			.reg_off = FIXPLL_CTRL0,
			.shift   = 0,
			.width   = 9,
		},
		.n = {
			.reg_off = FIXPLL_CTRL0,
			.shift   = 11,
			.width   = 5,
		},
		.l = {
			.reg_off = FIXPLL_STS,
			.shift   = 31,
			.width   = 1,
		},
		.rst = {
			.reg_off = FIXPLL_CTRL0,
			.shift   = 29,
			.width   = 1,
		},
	},
	.hw.init = &(struct clk_init_data) {
		.name = "fixed_pll_vco",
		.ops = &meson_clk_pll_ro_ops,
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "fix",
		},
		.num_parents = 1,
	},
};

static struct clk_regmap s7_fixed_pll = {
	.data = &(struct clk_regmap_div_data) {
		.offset = FIXPLL_CTRL0,
		.shift = 9,
		.width = 2,
		.flags = CLK_DIVIDER_POWER_OF_TWO | CLK_DIVIDER_READ_ONLY,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "fixed_pll",
		.ops = &clk_regmap_divider_ro_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_fixed_pll_vco.hw
		},
		.num_parents = 1,
	},
};

#define S7_FCLK_DIV(_name, _mult, _divider)				\
struct clk_fixed_factor s7_##_name##_div = {				\
	.mult = (_mult),						\
	.div = (_divider),						\
	.hw.init = &(struct clk_init_data) {				\
		.name = #_name "_div",					\
		.ops = &clk_fixed_factor_ops,				\
		.parent_hws = (const struct clk_hw *[]) {		\
			&s7_fixed_pll.hw				\
		},							\
		.num_parents = 1,					\
	},								\
}

#define S7_FCLK(_name, _reg, _bit)					\
struct clk_regmap s7_##_name = {					\
	.data = &(struct clk_regmap_gate_data) {			\
		.offset = (_reg),					\
		.bit_idx = (_bit),					\
	},								\
	.hw.init = &(struct clk_init_data) {				\
		.name = #_name,						\
		.ops = &clk_regmap_gate_ro_ops,				\
		.parent_hws = (const struct clk_hw *[]) {		\
			&s7_##_name##_div.hw				\
		},							\
		.num_parents = 1,					\
	},								\
}

/*
 * The datasheet documents FIXPLL_CTRL1[30:26] as a single fclk_en field, but
 * each bit enables one divider.
 */
static S7_FCLK_DIV(fclk_div2, 1, 2);
static S7_FCLK(fclk_div2, FIXPLL_CTRL1, 30);
static S7_FCLK_DIV(fclk_div3, 1, 3);
static S7_FCLK(fclk_div3, FIXPLL_CTRL1, 26);
static S7_FCLK_DIV(fclk_div4, 1, 4);
static S7_FCLK(fclk_div4, FIXPLL_CTRL1, 27);
static S7_FCLK_DIV(fclk_div5, 1, 5);
static S7_FCLK(fclk_div5, FIXPLL_CTRL1, 28);
static S7_FCLK_DIV(fclk_div7, 1, 7);
static S7_FCLK(fclk_div7, FIXPLL_CTRL1, 29);
static S7_FCLK_DIV(fclk_div2p5, 2, 5);
static S7_FCLK(fclk_div2p5, FIXPLL_CTRL0, 25);

static const struct pll_mult_range s7_pll_mult_range = {
	.min = 134,
	.max = 265,
};

/*
 * NOTE: each VCO must be registered before its div0p5 divider.
 *
 * div0p5 divides the PLL input by 2 when set. Linux doesn't change it at
 * runtime, but the VCO init sequence writes it (set for GP0, HIFI0 and HIFI1,
 * clear for HDMI) unless the PLL is running and has
 * CLK_MESON_PLL_NOINIT_ENABLED. A divider registered before that would keep
 * the power-on value as its rate.
 *
 * meson_clkc_mmio_probe() registers the clocks in ID order, so each VCO has a
 * lower ID than its divider: the VCO init runs at registration while the VCO
 * is still an orphan, then the divider reads the final value and the framework
 * recalculates the VCO rate. CLK_GET_RATE_NOCACHE makes the divider read the
 * register again whenever its rate is queried.
 *
 * WARNING: the multiplier ranges assume the init values, div0p5 set for GP0,
 * HIFI0 and HIFI1 and clear for HDMI. A PLL the firmware leaves running with
 * the other setting is reported correctly, but the same multipliers then give
 * twice (GP0/HIFI) or half (HDMI) the DCO rate, possibly outside the hardware
 * limits, and the driver still accepts them.
 */
static const struct clk_div_table s7_pll_div0p5_table[] = {
	{ .val = 0, .div = 1 },
	{ .val = 1, .div = 2 },
	{ /* sentinel */ }
};

static const struct reg_sequence s7_gp0_pll_init_regs[] = {
	{ .reg = GP0PLL_CTRL0,	.def = 0x02520800 },
	{ .reg = GP0PLL_CTRL1,	.def = 0x09900322 },
	{ .reg = GP0PLL_CTRL2,	.def = 0x00000000 },
	{ .reg = GP0PLL_CTRL3,	.def = 0x00000006 },
};

static struct clk_regmap s7_gp0_pll_div0p5 = {
	.data = &(struct clk_regmap_div_data) {
		.offset = GP0PLL_CTRL1,
		.shift = 27,
		.width = 1,
		.table = s7_pll_div0p5_table,
		.flags = CLK_DIVIDER_READ_ONLY,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "gp0_pll_div0p5",
		.ops = &clk_regmap_divider_ro_ops,
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "gp0",
		},
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
	},
};

static struct clk_regmap s7_gp0_pll_vco = {
	.data = &(struct meson_clk_pll_data) {
		.en = {
			.reg_off = GP0PLL_CTRL0,
			.shift   = 28,
			.width   = 1,
		},
		.m = {
			.reg_off = GP0PLL_CTRL0,
			.shift   = 0,
			.width   = 9,
		},
		.frac = {
			.reg_off = GP0PLL_CTRL2,
			.shift   = 0,
			.width   = 17,
		},
		.l = {
			.reg_off = GP0PLL_STS,
			.shift   = 31,
			.width   = 1,
		},
		.rst = {
			.reg_off = GP0PLL_CTRL0,
			.shift   = 29,
			.width   = 1,
		},
		.range = &s7_pll_mult_range,
		.init_regs = s7_gp0_pll_init_regs,
		.init_count = ARRAY_SIZE(s7_gp0_pll_init_regs),
		.flags = CLK_MESON_PLL_NOINIT_ENABLED,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "gp0_pll_vco",
		.ops = &meson_clk_pll_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_gp0_pll_div0p5.hw
		},
		.num_parents = 1,
	},
};

static struct clk_regmap s7_gp0_pll = {
	.data = &(struct clk_regmap_div_data) {
		.offset = GP0PLL_CTRL0,
		.shift = 9,
		.width = 2,
		.flags = CLK_DIVIDER_POWER_OF_TWO |
			 CLK_DIVIDER_ROUND_CLOSEST,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "gp0_pll",
		.ops = &clk_regmap_divider_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_gp0_pll_vco.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static const struct reg_sequence s7_hifi0_pll_init_regs[] = {
	{ .reg = HIFI0PLL_CTRL0,	.def = 0x02520800 },
	{ .reg = HIFI0PLL_CTRL1,	.def = 0x09900322 },
	{ .reg = HIFI0PLL_CTRL2,	.def = 0x00000000 },
	{ .reg = HIFI0PLL_CTRL3,	.def = 0x00000006 },
};

static struct clk_regmap s7_hifi0_pll_div0p5 = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HIFI0PLL_CTRL1,
		.shift = 27,
		.width = 1,
		.table = s7_pll_div0p5_table,
		.flags = CLK_DIVIDER_READ_ONLY,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hifi0_pll_div0p5",
		.ops = &clk_regmap_divider_ro_ops,
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "hifi0",
		},
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
	},
};

static struct clk_regmap s7_hifi0_pll_vco = {
	.data = &(struct meson_clk_pll_data) {
		.en = {
			.reg_off = HIFI0PLL_CTRL0,
			.shift   = 28,
			.width   = 1,
		},
		.m = {
			.reg_off = HIFI0PLL_CTRL0,
			.shift   = 0,
			.width   = 9,
		},
		.frac = {
			.reg_off = HIFI0PLL_CTRL2,
			.shift   = 0,
			.width   = 17,
		},
		.l = {
			.reg_off = HIFI0PLL_STS,
			.shift   = 31,
			.width   = 1,
		},
		.rst = {
			.reg_off = HIFI0PLL_CTRL0,
			.shift   = 29,
			.width   = 1,
		},
		.frac_max = 100000,
		.range = &s7_pll_mult_range,
		.init_regs = s7_hifi0_pll_init_regs,
		.init_count = ARRAY_SIZE(s7_hifi0_pll_init_regs),
		.flags = CLK_MESON_PLL_ROUND_CLOSEST,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hifi0_pll_vco",
		.ops = &meson_clk_pll_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hifi0_pll_div0p5.hw
		},
		.num_parents = 1,
	},
};

static struct clk_regmap s7_hifi0_pll = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HIFI0PLL_CTRL0,
		.shift = 9,
		.width = 2,
		.flags = CLK_DIVIDER_POWER_OF_TWO |
			 CLK_DIVIDER_ROUND_CLOSEST,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hifi0_pll",
		.ops = &clk_regmap_divider_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hifi0_pll_vco.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static const struct reg_sequence s7_hifi1_pll_init_regs[] = {
	{ .reg = HIFI1PLL_CTRL0,	.def = 0x02520800 },
	{ .reg = HIFI1PLL_CTRL1,	.def = 0x09900322 },
	{ .reg = HIFI1PLL_CTRL2,	.def = 0x00000000 },
	{ .reg = HIFI1PLL_CTRL3,	.def = 0x00000006 },
};

static struct clk_regmap s7_hifi1_pll_div0p5 = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HIFI1PLL_CTRL1,
		.shift = 27,
		.width = 1,
		.table = s7_pll_div0p5_table,
		.flags = CLK_DIVIDER_READ_ONLY,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hifi1_pll_div0p5",
		.ops = &clk_regmap_divider_ro_ops,
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "hifi1",
		},
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
	},
};

static struct clk_regmap s7_hifi1_pll_vco = {
	.data = &(struct meson_clk_pll_data) {
		.en = {
			.reg_off = HIFI1PLL_CTRL0,
			.shift   = 28,
			.width   = 1,
		},
		.m = {
			.reg_off = HIFI1PLL_CTRL0,
			.shift   = 0,
			.width   = 9,
		},
		.frac = {
			.reg_off = HIFI1PLL_CTRL2,
			.shift   = 0,
			.width   = 17,
		},
		.l = {
			.reg_off = HIFI1PLL_STS,
			.shift   = 31,
			.width   = 1,
		},
		.rst = {
			.reg_off = HIFI1PLL_CTRL0,
			.shift   = 29,
			.width   = 1,
		},
		.frac_max = 100000,
		.range = &s7_pll_mult_range,
		.init_regs = s7_hifi1_pll_init_regs,
		.init_count = ARRAY_SIZE(s7_hifi1_pll_init_regs),
		.flags = CLK_MESON_PLL_ROUND_CLOSEST,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hifi1_pll_vco",
		.ops = &meson_clk_pll_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hifi1_pll_div0p5.hw
		},
		.num_parents = 1,
	},
};

static struct clk_regmap s7_hifi1_pll = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HIFI1PLL_CTRL0,
		.shift = 9,
		.width = 2,
		.flags = CLK_DIVIDER_POWER_OF_TWO |
			 CLK_DIVIDER_ROUND_CLOSEST,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hifi1_pll",
		.ops = &clk_regmap_divider_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hifi1_pll_vco.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static const struct pll_mult_range s7_hdmi_pll_mult_range = {
	.min = 125,
	.max = 249,
};

static const struct reg_sequence s7_hdmi_pll_init_regs[] = {
	{ .reg = HDMIPLL_CTRL0,	.def = 0x00881000 },
	{ .reg = HDMIPLL_CTRL1,	.def = 0x2c6011cc },
	{ .reg = HDMIPLL_CTRL2,	.def = 0x86801000 },
	{ .reg = HDMIPLL_CTRL3,	.def = 0x00000000 },
};

static struct clk_regmap s7_hdmi_pll_div0p5 = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HDMIPLL_CTRL1,
		.shift = 19,
		.width = 1,
		.table = s7_pll_div0p5_table,
		.flags = CLK_DIVIDER_READ_ONLY,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_div0p5",
		.ops = &clk_regmap_divider_ro_ops,
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "hdmi",
		},
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
	},
};

static struct clk_regmap s7_hdmi_pll_vco = {
	.data = &(struct meson_clk_pll_data) {
		.en = {
			.reg_off = HDMIPLL_CTRL0,
			.shift   = 28,
			.width   = 1,
		},
		.m = {
			.reg_off = HDMIPLL_CTRL0,
			.shift   = 0,
			.width   = 9,
		},
		.frac = {
			.reg_off = HDMIPLL_CTRL3,
			.shift   = 0,
			.width   = 17,
		},
		.l = {
			.reg_off = HDMIPLL_STS,
			.shift   = 31,
			.width   = 1,
		},
		.rst = {
			.reg_off = HDMIPLL_CTRL0,
			.shift   = 29,
			.width   = 1,
		},
		.range = &s7_hdmi_pll_mult_range,
		.init_regs = s7_hdmi_pll_init_regs,
		.init_count = ARRAY_SIZE(s7_hdmi_pll_init_regs),
		.flags = CLK_MESON_PLL_RST_ACTIVE_LOW |
			 CLK_MESON_PLL_NOINIT_ENABLED,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_vco",
		.ops = &meson_clk_pll_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll_div0p5.hw
		},
		.num_parents = 1,
	},
};

/*
 * The datasheet documents HDMIPLL_CTRL2[22:19] and [18:15] as single fields.
 * [22:19] holds two cascaded power-of-two dividers, od0 [20:19] and od1
 * [22:21]. [18:15] holds two parallel ones: [16:15] is hdmi_pll, [18:17]
 * feeds the PHY and is not modelled.
 */
static struct clk_regmap s7_hdmi_pll_od0 = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HDMIPLL_CTRL2,
		.shift = 19,
		.width = 2,
		.flags = CLK_DIVIDER_POWER_OF_TWO,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_od0",
		.ops = &clk_regmap_divider_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll_vco.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_regmap s7_hdmi_pll_od1 = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HDMIPLL_CTRL2,
		.shift = 21,
		.width = 2,
		.flags = CLK_DIVIDER_POWER_OF_TWO,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_od1",
		.ops = &clk_regmap_divider_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll_od0.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_regmap s7_hdmi_pll = {
	.data = &(struct clk_regmap_div_data) {
		.offset = HDMIPLL_CTRL2,
		.shift = 15,
		.width = 2,
		.flags = CLK_DIVIDER_POWER_OF_TWO,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll",
		.ops = &clk_regmap_divider_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll_od1.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_fixed_factor s7_hdmi_pll_pix_div5 = {
	.mult = 1,
	.div = 5,
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_pix_div5",
		.ops = &clk_fixed_factor_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_fixed_factor s7_hdmi_pll_pix_div6p25 = {
	.mult = 4,
	.div = 25,
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_pix_div6p25",
		.ops = &clk_fixed_factor_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_fixed_factor s7_hdmi_pll_pix_div7p5 = {
	.mult = 2,
	.div = 15,
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_pix_div7p5",
		.ops = &clk_fixed_factor_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll.hw
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_regmap s7_hdmi_pll_pix = {
	.data = &(struct clk_regmap_mux_data) {
		.offset = HDMIPLL_CTRL0,
		.mask = 0x3,
		.shift = 9,
	},
	.hw.init = &(struct clk_init_data) {
		.name = "hdmi_pll_pix",
		.ops = &clk_regmap_mux_ops,
		.parent_hws = (const struct clk_hw *[]) {
			&s7_hdmi_pll_pix_div5.hw,
			&s7_hdmi_pll_pix_div6p25.hw,
			&s7_hdmi_pll_pix_div7p5.hw
		},
		.num_parents = 3,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_hw *s7_pll_hws[] = {
	[CLKID_FIXED_PLL_VCO]		= &s7_fixed_pll_vco.hw,
	[CLKID_FIXED_PLL]		= &s7_fixed_pll.hw,
	[CLKID_FCLK_DIV2_DIV]		= &s7_fclk_div2_div.hw,
	[CLKID_FCLK_DIV2]		= &s7_fclk_div2.hw,
	[CLKID_FCLK_DIV3_DIV]		= &s7_fclk_div3_div.hw,
	[CLKID_FCLK_DIV3]		= &s7_fclk_div3.hw,
	[CLKID_FCLK_DIV4_DIV]		= &s7_fclk_div4_div.hw,
	[CLKID_FCLK_DIV4]		= &s7_fclk_div4.hw,
	[CLKID_FCLK_DIV5_DIV]		= &s7_fclk_div5_div.hw,
	[CLKID_FCLK_DIV5]		= &s7_fclk_div5.hw,
	[CLKID_FCLK_DIV7_DIV]		= &s7_fclk_div7_div.hw,
	[CLKID_FCLK_DIV7]		= &s7_fclk_div7.hw,
	[CLKID_FCLK_DIV2P5_DIV]		= &s7_fclk_div2p5_div.hw,
	[CLKID_FCLK_DIV2P5]		= &s7_fclk_div2p5.hw,
	[CLKID_GP0_PLL_VCO]		= &s7_gp0_pll_vco.hw,
	[CLKID_GP0_PLL_DIV0P5]		= &s7_gp0_pll_div0p5.hw,
	[CLKID_GP0_PLL]			= &s7_gp0_pll.hw,
	[CLKID_HIFI0_PLL_VCO]		= &s7_hifi0_pll_vco.hw,
	[CLKID_HIFI0_PLL_DIV0P5]	= &s7_hifi0_pll_div0p5.hw,
	[CLKID_HIFI0_PLL]		= &s7_hifi0_pll.hw,
	[CLKID_HIFI1_PLL_VCO]		= &s7_hifi1_pll_vco.hw,
	[CLKID_HIFI1_PLL_DIV0P5]	= &s7_hifi1_pll_div0p5.hw,
	[CLKID_HIFI1_PLL]		= &s7_hifi1_pll.hw,
	[CLKID_HDMI_PLL_VCO]		= &s7_hdmi_pll_vco.hw,
	[CLKID_HDMI_PLL_DIV0P5]		= &s7_hdmi_pll_div0p5.hw,
	[CLKID_HDMI_PLL_OD0]		= &s7_hdmi_pll_od0.hw,
	[CLKID_HDMI_PLL_OD1]		= &s7_hdmi_pll_od1.hw,
	[CLKID_HDMI_PLL]		= &s7_hdmi_pll.hw,
	[CLKID_HDMI_PLL_PIX_DIV5]	= &s7_hdmi_pll_pix_div5.hw,
	[CLKID_HDMI_PLL_PIX_DIV6P25]	= &s7_hdmi_pll_pix_div6p25.hw,
	[CLKID_HDMI_PLL_PIX_DIV7P5]	= &s7_hdmi_pll_pix_div7p5.hw,
	[CLKID_HDMI_PLL_PIX]		= &s7_hdmi_pll_pix.hw,
};

static const struct meson_clkc_data s7_pll_clkc_data = {
	.hw_clks = {
		.hws = s7_pll_hws,
		.num = ARRAY_SIZE(s7_pll_hws),
	},
};

static const struct of_device_id s7_pll_clkc_match_table[] = {
	{
		.compatible = "amlogic,s7-pll-clkc",
		.data = &s7_pll_clkc_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s7_pll_clkc_match_table);

static struct platform_driver s7_pll_clkc_driver = {
	.probe		= meson_clkc_mmio_probe,
	.driver		= {
		.name	= "s7-pll-clkc",
		.of_match_table = s7_pll_clkc_match_table,
	},
};
module_platform_driver(s7_pll_clkc_driver);

MODULE_DESCRIPTION("Amlogic S7 PLL Clock Controller driver");
MODULE_AUTHOR("Carlo Caione <ccaione@baylibre.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CLK_MESON");
