/*
 * Copyright 2017 NXP.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>

#include "clk.h"

/* PLL CFGs */
#define PLL_CFG0	0x0
#define PLL_CFG1	0x4
#define PLL_CFG2	0x8

#define PLL_DIVF1_SHIFT	13
#define PLL_DIVF2_SHIFT	7
#define PLL_DIVF_MASK	0x3f

#define PLL_DIVR1_SHIFT	25
#define PLL_DIVR2_SHIFT	19
#define PLL_DIVR1_MASK	0x3
#define PLL_DIVR2_MASK	0x3f
#define PLL_REF_SHIFT	0
#define PLL_REF_MASK	0x3

#define PLL_LOCK	31
#define PLL_PD		7

#define OSC_25M		25000000
#define OSC_27M		27000000

struct clk_sccg_pll {
	struct clk_hw	hw;
	void __iomem	*base;
};

#define to_clk_sccg_pll(_hw) container_of(_hw, struct clk_sccg_pll, hw)

static int clk_pll1_is_prepared(struct clk_hw *hw)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	return (val & (1 << PLL_PD)) ? 0 : 1;
}

static unsigned long clk_pll1_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val, divf;

	val = readl_relaxed(pll->base + PLL_CFG2);
	divf = (val >> PLL_DIVF1_SHIFT) & PLL_DIVF_MASK;

	return parent_rate * 2 * (divf + 1);
}

static long clk_pll1_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	u32 div;

	div = rate / (parent_rate * 2);

	return parent_rate * div * 2;
}

static int clk_pll1_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;
	u32 divf;

	divf = rate / (parent_rate * 2);

	val = readl_relaxed(pll->base + PLL_CFG2);
	val &= ~(PLL_DIVF_MASK << PLL_DIVF1_SHIFT);
	val |= (divf - 1) << PLL_DIVF1_SHIFT;
	writel_relaxed(val, pll->base + PLL_CFG2);
	/* FIXME  PLL lock check */

	return 0;
}

static int clk_pll1_prepare(struct clk_hw *hw)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base);
	val &= ~(1 << PLL_PD);
	writel_relaxed(val, pll->base);

	/* wait for PLL locked */

	return 0;
}

static void clk_pll1_unprepare(struct clk_hw *hw)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base);
	val |= (1 << PLL_PD);
	writel_relaxed(val, pll->base);
}

static unsigned long clk_pll2_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);
	u32 val, ref, divr1, divf1, divr2, divf2;
	u64 temp64;

	val = readl_relaxed(pll->base + PLL_CFG0);
	switch ((val >> PLL_REF_SHIFT) & PLL_REF_MASK) {
	case 0:
		ref = OSC_25M;
		break;
	case 1:
		ref = OSC_27M;
		break;
	default:
		ref = OSC_25M;
		break;
	}

	val = readl_relaxed(pll->base + PLL_CFG2);
	divr1 = (val >> PLL_DIVR1_SHIFT) & PLL_DIVR1_MASK;
	divr2 = (val >> PLL_DIVR2_SHIFT) & PLL_DIVR2_MASK;
	divf1 = (val >> PLL_DIVF1_SHIFT) & PLL_DIVF_MASK;
	divf2 = (val >> PLL_DIVF2_SHIFT) & PLL_DIVF_MASK;

	temp64 = ref * 2;
	temp64 *= (divf1 + 1) * (divf2 + 1);

	do_div(temp64, (divr1 + 1) * (divr2 + 1));

	return (unsigned long)temp64;
}

static long clk_pll2_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	u32 div;
	unsigned long parent_rate = *prate;

	/* FIXME */
	div = rate / (parent_rate);

	return parent_rate * div;
}

static int clk_pll2_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	u32 val;
	u32 divf;
	struct clk_sccg_pll *pll = to_clk_sccg_pll(hw);

	divf = rate / (parent_rate);

	val = readl_relaxed(pll->base + PLL_CFG2);
	val &= ~(PLL_DIVF_MASK << PLL_DIVF2_SHIFT);
	val |= (divf - 1) << PLL_DIVF2_SHIFT;
	writel_relaxed(val, pll->base + PLL_CFG2);
	/* FIXME  PLL lock check */

	return 0;
}

static const struct clk_ops clk_sccg_pll1_ops = {
	.is_prepared	= clk_pll1_is_prepared,
	.recalc_rate	= clk_pll1_recalc_rate,
	.round_rate	= clk_pll1_round_rate,
	.set_rate	= clk_pll1_set_rate,
};

static const struct clk_ops clk_sccg_pll2_ops = {
	.prepare	= clk_pll1_prepare,
	.unprepare	= clk_pll1_unprepare,
	.recalc_rate	= clk_pll2_recalc_rate,
	.round_rate	= clk_pll2_round_rate,
	.set_rate	= clk_pll2_set_rate,
};

struct clk *imx_clk_sccg_pll(const char *name, const char *parent_name,
			     void __iomem *base, enum imx_sccg_pll_type pll_type)
{
	struct clk_sccg_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base = base;
	init.name = name;
	switch (pll_type) {
	case SCCG_PLL1:
		init.ops = &clk_sccg_pll1_ops;
		break;
	case SCCG_PLL2:
		init.ops = &clk_sccg_pll2_ops;
		break;
	}

	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
