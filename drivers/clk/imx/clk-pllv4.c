/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/slab.h>

#include "clk.h"

#define PLL_EN			BIT(0)
#define BP_PLL_DIV		16
#define BM_PLL_DIV		(0x7f << 16)
#define PLL_CFG_OFFSET		0x08
#define PLL_NUM_OFFSET		0x10
#define PLL_DENOM_OFFSET	0x14

struct clk_pllv4 {
	struct clk_hw	hw;
	void __iomem	*base;
	u32		div_mask;
	u32		div_shift;
	u32		cfg_offset;
	u32		num_offset;
	u32		denom_offset;
};

#define to_clk_pllv4(__hw) container_of(__hw, struct clk_pllv4, hw)

static unsigned long clk_pllv4_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct clk_pllv4 *pll = to_clk_pllv4(hw);
	u32 mfn = readl_relaxed(pll->base + pll->num_offset);
	u32 mfd = readl_relaxed(pll->base + pll->denom_offset);
	u32 div = (readl_relaxed(pll->base + pll->cfg_offset)
		& pll->div_mask) >> pll->div_shift;
	u64 temp64 = (u64)parent_rate;

	temp64 *= mfn;
	do_div(temp64, mfd);

	return (parent_rate * div) + (u32)temp64;
}

static long clk_pllv4_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	unsigned long min_rate = parent_rate * 16;
	unsigned long max_rate = parent_rate * 30;
	u32 div;
	u32 mfn, mfd = 1000000;
	u64 temp64;

	if (rate > max_rate)
		rate = max_rate;
	else if (rate < min_rate)
		rate = min_rate;

	div = rate / parent_rate;
	temp64 = (u64) (rate - div * parent_rate);
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	return parent_rate * div + parent_rate / mfd * mfn;
}

static int clk_pllv4_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pllv4 *pll = to_clk_pllv4(hw);
	unsigned long min_rate = parent_rate * 16;
	unsigned long max_rate = parent_rate * 30;
	u32 val, div;
	u32 mfn, mfd = 1000000;
	u64 temp64;

	if (rate < min_rate || rate > max_rate)
		return -EINVAL;

	div = rate / parent_rate;
	temp64 = (u64) (rate - div * parent_rate);
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	val = readl_relaxed(pll->base + pll->cfg_offset);
	val &= ~pll->div_mask;
	val |= (div << pll->div_shift);
	writel_relaxed(val, pll->base + pll->cfg_offset);
	writel_relaxed(mfn, pll->base + pll->num_offset);
	writel_relaxed(mfd, pll->base + pll->denom_offset);

	return 0;
}

static int clk_pllv4_enable(struct clk_hw *hw)
{
	u32 val;
	struct clk_pllv4 *pll = to_clk_pllv4(hw);

	val = readl_relaxed(pll->base);
	val |= PLL_EN;
	writel_relaxed(val, pll->base);

	return 0;
}

static void clk_pllv4_disable(struct clk_hw *hw)
{
	u32 val;
	struct clk_pllv4 *pll = to_clk_pllv4(hw);

	val = readl_relaxed(pll->base);
	val &= ~PLL_EN;
	writel_relaxed(val, pll->base);
}

static int clk_pllv4_is_enabled(struct clk_hw *hw)
{
	struct clk_pllv4 *pll = to_clk_pllv4(hw);

	if (readl_relaxed(pll->base) & PLL_EN)
		return 1;

	return 0;
}

static const struct clk_ops clk_pllv4_ops = {
	.recalc_rate	= clk_pllv4_recalc_rate,
	.round_rate	= clk_pllv4_round_rate,
	.set_rate	= clk_pllv4_set_rate,
	.enable		= clk_pllv4_enable,
	.disable	= clk_pllv4_disable,
	.is_enabled	= clk_pllv4_is_enabled,
};

struct clk *imx_clk_pllv4(const char *name, const char *parent_name,
			  void __iomem *base)
{
	struct clk_pllv4 *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base = base;
	pll->div_mask = BM_PLL_DIV;
	pll->div_shift = BP_PLL_DIV;
	pll->cfg_offset = PLL_CFG_OFFSET;
	pll->num_offset = PLL_NUM_OFFSET;
	pll->denom_offset = PLL_DENOM_OFFSET;

	init.name = name;
	init.ops = &clk_pllv4_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_GATE;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
