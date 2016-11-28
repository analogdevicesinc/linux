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

#define PLL_EN		BIT(0)
#define BP_PLL_DIV		16
#define BM_PLL_DIV		(0x7 << 16)
#define PLL_CFG_OFFSET		0x08

struct clk_pllv5 {
	struct clk_hw	hw;
	void __iomem	*base;
	u32		div_mask;
	u32		div_shift;
	u32		cfg_offset;
};

#define to_clk_pllv5(__hw) container_of(__hw, struct clk_pllv5, hw)

static unsigned long clk_pllv5_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct clk_pllv5 *pll = to_clk_pllv5(hw);
	u32 val = (readl_relaxed(pll->base + pll->cfg_offset) & pll->div_mask) >> pll->div_shift;
	u32 div;

	switch (val) {
	case 1:
		div = 15;
		break;
	case 2:
		div = 16;
		break;
	case 3:
		div = 20;
		break;
	case 4:
		div = 22;
		break;
	case 5:
		div = 25;
		break;
	case 6:
		div = 30;
		break;
	default:
		div = 20;
		break;
	}

	return parent_rate * div;
}

static long clk_pllv5_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	u32 div;

	div = rate / parent_rate;

	if (div == 15 || div == 16 ||
		div == 20 || div == 22 ||
		div == 25 || div == 30)
		return parent_rate * div;
	else
		return parent_rate * 20;
}

static int clk_pllv5_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pllv5 *pll = to_clk_pllv5(hw);
	unsigned long min_rate = parent_rate * 15;
	unsigned long max_rate = parent_rate * 30;
	u32 val, div, reg;

	if (rate < min_rate || rate > max_rate)
		return -EINVAL;

	div = rate / parent_rate;

	switch (div) {
	case 15:
		val = 1;
		break;
	case 16:
		val = 2;
		break;
	case 20:
		val = 3;
		break;
	case 22:
		val = 4;
		break;
	case 25:
		val = 5;
		break;
	case 30:
		val = 6;
		break;
	default:
		val = 3;
		break;
	}

	reg = readl_relaxed(pll->base + pll->cfg_offset);
	reg &= ~pll->div_mask;
	reg |= (val << pll->div_shift);
	writel_relaxed(val, pll->base + pll->cfg_offset);

	return 0;
}

static int clk_pllv5_enable(struct clk_hw *hw)
{
	u32 val;
	struct clk_pllv5 *pll = to_clk_pllv5(hw);

	val = readl_relaxed(pll->base);
	val |= PLL_EN;
	writel_relaxed(val, pll->base);

	return 0;
}

static void clk_pllv5_disable(struct clk_hw *hw)
{
	u32 val;
	struct clk_pllv5 *pll = to_clk_pllv5(hw);

	val = readl_relaxed(pll->base);
	val &= ~PLL_EN;
	writel_relaxed(val, pll->base);
}

static int clk_pllv5_is_enabled(struct clk_hw *hw)
{
	struct clk_pllv5 *pll = to_clk_pllv5(hw);

	if (readl_relaxed(pll->base) & PLL_EN)
		return 0;

	return 1;
}

static const struct clk_ops clk_pllv5_ops = {
	.recalc_rate 	= clk_pllv5_recalc_rate,
	.round_rate	= clk_pllv5_round_rate,
	.set_rate	= clk_pllv5_set_rate,
	.enable		= clk_pllv5_enable,
	.disable	= clk_pllv5_disable,
	.is_enabled	= clk_pllv5_is_enabled,
};

struct clk *imx_clk_pllv5(const char *name, const char *parent_name,
			  void __iomem *base)
{
	struct clk_pllv5 *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->base = base;
	pll->div_mask = BM_PLL_DIV;
	pll->div_shift = BP_PLL_DIV;
	pll->cfg_offset = PLL_CFG_OFFSET;

	init.name = name;
	init.ops = &clk_pllv5_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}

