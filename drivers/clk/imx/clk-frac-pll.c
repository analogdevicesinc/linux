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

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>

#include "clk.h"

#define PLL_CFG0 	0x0
#define PLL_CFG1	0x4

#define PLL_LOCK_STATUS	(0x1 << 31)
#define PLL_CLKE	21
#define PLL_PD		19

#define PLL_NEWDIV_VAL		12
#define PLL_FRAC_DIV_MASK	0xffffff
#define PLL_INT_DIV_MASK	0x7f
#define PLL_FRAC_DENOM		0x1000000

struct clk_frac_pll {
	struct clk_hw	hw;
	void __iomem	*base;
};

#define to_clk_frac_pll(_hw) container_of(_hw, struct clk_frac_pll, hw);

static int clk_pll_prepare(struct clk_hw *hw)
{
	u32 val;
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);

	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~(1 << PLL_PD);
	writel_relaxed(val, pll->base + PLL_CFG0);

	return 0;
}

static void clk_pll_unprepare(struct clk_hw *hw)
{
	u32 val;
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);

	val = readl_relaxed(pll->base + PLL_CFG0);
	val |= (1 << PLL_PD);
	writel_relaxed(val, pll->base + PLL_CFG0);
}

static int clk_pll_is_prepared(struct clk_hw *hw)
{
	u32 val;
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);

	val = readl_relaxed(pll->base + PLL_CFG0);
	return (val & (1 << PLL_PD)) ? 0 : 1;
}

static unsigned long clk_pll_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	u32 val, divff, divfi, divq;
	u64 temp64;
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);

	val = readl_relaxed(pll->base + PLL_CFG0);
	divq = ((val & 0x1f) + 1) * 2;
	val = readl_relaxed(pll->base + PLL_CFG1);
	divff = (val >> 7) & PLL_FRAC_DIV_MASK;
	divfi = (val & PLL_INT_DIV_MASK);

	pr_debug("divq:%d, divfi:%d, divff:%d\n", divq, divfi, divff);
	temp64 = (u64)parent_rate * 8;
	temp64 *= (divff + 1);
	do_div(temp64, PLL_FRAC_DENOM);
	temp64 /= divq;

	return parent_rate * 8 * (divfi + 1) / divq + (unsigned long)temp64;
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	u32 divff, divfi;
	u64 temp64;
	unsigned long parent_rate = *prate;

	parent_rate *= 8;
	divfi = rate / parent_rate;
	temp64 = (u64)(rate - divfi * parent_rate);
	temp64 *= PLL_FRAC_DENOM;
	do_div(temp64, parent_rate);
	divff = temp64;

	temp64 = (u64)parent_rate;
	temp64 *= divff;
	do_div(temp64, PLL_FRAC_DENOM);

	return parent_rate * divfi + (unsigned long)temp64;
}

/*
 * To simplify the clock calculation, we can keep the
 * 'PLL_OUTPUT_VAL' to zero(means the PLL output
 * will be dividered by 2. So the PLL output can use
 * below formula:
 * pllout = parent_rate * 8 / 2 * DIVF_VAL;
 * where DIVF_VAL = 1 + DIVFI + DIVFF / 2^24.
 */
static int clk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	u32 val, divfi, divff;
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);

	parent_rate *= 4;
	divfi = rate / parent_rate;
	divff = rate - parent_rate * divfi;
	divff *= (PLL_FRAC_DENOM / parent_rate);

	val = readl_relaxed(pll->base + PLL_CFG1);
	val &= ~((PLL_FRAC_DIV_MASK << 7) | (PLL_INT_DIV_MASK));
	val |= ((divff << 7) | (divfi - 1));
	writel_relaxed(val, pll->base + PLL_CFG1);

	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~0x1f;
	writel_relaxed(val, pll->base + PLL_CFG0);

	val = readl_relaxed(pll->base + PLL_CFG0);
	val |= (1 << PLL_NEWDIV_VAL);
	writel_relaxed(val, pll->base + PLL_CFG0);

	return 0;
}

static const struct clk_ops clk_frac_pll_ops = {
	.prepare	= clk_pll_prepare,
	.unprepare	= clk_pll_unprepare,
	.is_prepared	= clk_pll_is_prepared,
	.recalc_rate	= clk_pll_recalc_rate,
	.round_rate	= clk_pll_round_rate,
	.set_rate	= clk_pll_set_rate,
};

struct clk *imx_clk_frac_pll(const char *name, const char *parent_name,
			     void __iomem *base)
{
	struct clk_frac_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base = base;
	init.name = name;
	init.ops = &clk_frac_pll_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
