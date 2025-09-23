/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Clock support for ADI processors
 *
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#ifndef CLK_ADI_CLK_H
#define CLK_ADI_CLK_H

#include <linux/clk.h>
#include <linux/clk-provider.h>

#define CGU_CTL         0x00
#define CGU_PLLCTL      0x04
#define CGU_STAT        0x08
#define CGU_DIV         0x0C
#define CGU_CLKOUTSEL   0x10
#define CGU_OSCWDCTL    0x14
#define CGU_TSCTL       0x18
#define CGU_TSVALUE0    0x1C
#define CGU_TSVALUE1    0x20
#define CGU_TSCOUNT0    0x24
#define CGU_TSCOUNT1    0x28
#define CGU_CCBF_DIS    0x2C
#define CGU_CCBF_STAT   0x30
#define CGU_SCBF_DIS    0x38
#define CGU_SCBF_STAT   0x3C
#define CGU_DIVEX       0x40
#define CGU_REVID       0x48

#define CDU_CFG0     0x00
#define CDU_CFG1     0x04
#define CDU_CFG2     0x08
#define CDU_CFG3     0x0C
#define CDU_CFG4     0x10
#define CDU_CFG5     0x14
#define CDU_CFG6     0x18
#define CDU_CFG7     0x1C
#define CDU_CFG8     0x20
#define CDU_CFG9     0x24
#define CDU_CFG10    0x28
#define CDU_CFG11    0x2C
#define CDU_CFG12    0x30
#define CDU_CFG13    0x34
#define CDU_CFG14    0x38

#define PLL3_CONTROL 0x2c

#define CDU_CLKINSEL 0x44

#define CGU_MSEL_SHIFT 8
#define CGU_MSEL_WIDTH 7

#define PLL3_MSEL_SHIFT 4
#define PLL3_MSEL_WIDTH 7

#define CDU_MUX_SIZE 4
#define CDU_MUX_SHIFT 1
#define CDU_MUX_WIDTH 2
#define CDU_EN_BIT 0

struct adi_clk_provider {
	void __iomem		*reg_base;
	struct clk_onecell_data	clk_data;
	struct device_node	*clk_node;
	struct regmap		*grf;
	spinlock_t		lock;
};

enum adi_clk_branch_type {
	branch_divider,
	branch_cdu_mux,
	branch_mux,
	branch_pll,
	branch_cgu_gate,
	branch_cdu_gate,
	branch_of_node,
};

struct adi_clk_branch {
	unsigned int			id;
	enum adi_clk_branch_type	branch_type;
	const char			*name;
	const char			*const *parent_names;
	u8				num_parents;
	unsigned long			flags;
	unsigned long			offset;
	u8				div_flags;
	u8				shift;
	u8				div_width;
	u32				mux_width;
	unsigned int			fact_mult;
	unsigned int			fact_div;
	u8				gate_bit;
	u8				pll_shift;
	u8				pll_width;
	u32				pll_m;
	unsigned long			rate;
};

#define DIVIDER(_id, cname, pname, o, s, w, df) \
{ \
	.id = _id, \
	.branch_type = branch_divider, \
	.name = cname, \
	.parent_names = (const char *[]){ pname }, \
	.num_parents = 1, \
	.div_flags = df, \
	.offset = o, \
	.shift = s, \
	.div_width = w, \
}

#define CDU_MUX(_id, cname, pname, o, s, w) \
{ \
	.id = _id, \
	.branch_type = branch_cdu_mux, \
	.name = cname, \
	.parent_names = pname, \
	.num_parents = ARRAY_SIZE(pname), \
	.offset = o, \
	.shift = s, \
	.mux_width = w, \
}

#define MUX(_id, cname, pname, f, o, s, w) \
{ \
	.id = _id, \
	.branch_type = branch_mux, \
	.name = cname, \
	.parent_names = pname, \
	.num_parents = ARRAY_SIZE(pname), \
	.flags = f, \
	.offset = o, \
	.shift = s, \
	.mux_width = w, \
}

#define FIXED(_id, cname, pname, f, mult, div) \
{ \
	.id = _id, \
	.branch_type = branch_mux, \
	.name = cname, \
	.parent_names = (const char *[]){ pname }, \
	.num_parents = 1, \
	.flags = f, \
	.fact_mult = mult, \
	.fact_div = div, \
}

#define PLL(_id, cname, pname, o, s, w, m_offset) \
{ \
	.id = _id, \
	.branch_type = branch_pll, \
	.name = cname, \
	.parent_names = (const char *[]){ pname }, \
	.num_parents = 1, \
	.offset = o, \
	.pll_shift = s, \
	.pll_width = w, \
	.pll_m = m_offset, \
}

#define CGU_GATE(_id, cname, pname, o, b) \
{ \
	.id = _id, \
	.branch_type = branch_cgu_gate, \
	.name = cname, \
	.parent_names = (const char *[]){ pname }, \
	.num_parents = 1, \
	.offset = o, \
	.gate_bit = b, \
}

#define CDU_GATE(_id, cname, pname, o, f) \
{ \
	.id = _id, \
	.branch_type = branch_cdu_gate, \
	.name = cname, \
	.parent_names = (const char *[]){ pname }, \
	.num_parents = 1, \
	.offset = o, \
	.flags = f, \
}

#define OF_CLOCK(_id, cname) \
{ \
	.id = _id, \
	.name = cname, \
	.branch_type = branch_of_node, \
}

struct clk_sc5xx_cgu_pll *to_clk_sc5xx_cgu_pll(struct clk_hw *hw);
struct clk *sc5xx_cgu_pll(const char *name, const char *parent_name,
			  void __iomem *base, u8 shift, u8 width, u32 m_offset, spinlock_t *lock);

struct adi_clk_provider *adi_clk_init(struct device_node *np,
				      void __iomem *base, unsigned long nr_clks);

void adi_clk_register_branches(struct adi_clk_provider *ctx,
			       struct adi_clk_branch *list,
			       unsigned int nr_clk);
#endif
