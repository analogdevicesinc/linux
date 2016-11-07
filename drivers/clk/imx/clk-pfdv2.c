/*
 * Copyright 2016 Freescale Semiconductor, Inc.
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
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>

#include "clk.h"

/**
 * struct clk_pfdv2 - IMX PFD clock
 * @clk_hw:	clock source
 * @reg:	PFD register address
 * @idx:	the index of PFD encoded in the register
 *
 */

struct clk_pfdv2 {
	struct clk_hw	hw;
	void __iomem	*reg;
	u8		idx;
};

#define to_clk_pfdv2(_hw) container_of(_hw, struct clk_pfdv2, hw)

static int clk_pfd_enable(struct clk_hw *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u32 val;

	val = readl_relaxed(pfd->reg);
	val &= ~(1 << ((pfd->idx + 1) * 8 - 1));
	writel_relaxed(val, pfd->reg);

	return 0;
}

static void clk_pfd_disable(struct clk_hw *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u32 val;

	val = readl_relaxed(pfd->reg);
	val |= 1 << ((pfd->idx + 1) * 8 - 1);
	writel_relaxed(val, pfd->reg);
}

static unsigned long clk_pfd_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u64 tmp = parent_rate;
	u8 frac = (readl_relaxed(pfd->reg) >> (pfd->idx * 8)) & 0x3f;

	/*
	 * The reset value of pfd field is zero, so add one to avoid div
	 * by zero, optimize this late.
	 */
	if (!frac)
		frac += 1;

	tmp *= 18;
	do_div(tmp, frac);

	return tmp;
}

static long clk_pfd_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	u64 tmp = *prate;
	u8 frac;

	tmp = tmp * 18 + rate / 2;
	do_div(tmp, rate);
	frac = tmp;
	if (frac < 12)
		frac = 12;
	else if (frac > 35)
		frac = 35;
	tmp = *prate;
	tmp *= 18;
	do_div(tmp, frac);

	return tmp;
}

static int clk_pfd_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u64 tmp = parent_rate;
	u32 val;
	u8 frac;

	tmp = tmp * 18 + rate / 2;
	do_div(tmp, rate);
	frac = tmp;
	if (frac < 12)
		frac = 12;
	else if (frac > 35)
		frac = 35;

	val = readl_relaxed(pfd->reg);
	val &= ~(0x3f << (pfd->idx * 8));
	val |= frac << (pfd->idx * 8);
	writel_relaxed(val, pfd->reg);

	return 0;
}

static int clk_pfd_is_enabled(struct clk_hw *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);

	if (readl_relaxed(pfd->reg) & (1 << ((pfd->idx + 1) * 8 - 1)))
		return 0;

	return 1;
}

static const struct clk_ops clk_pfdv2_ops = {
	.enable		= clk_pfd_enable,
	.disable	= clk_pfd_disable,
	.recalc_rate	= clk_pfd_recalc_rate,
	.round_rate	= clk_pfd_round_rate,
	.set_rate	= clk_pfd_set_rate,
	.is_enabled     = clk_pfd_is_enabled,
};

struct clk *imx_clk_pfdv2(const char *name, const char *parent_name,
			void __iomem *reg, u8 idx)
{
	struct clk_pfdv2 *pfd;
	struct clk *clk;
	struct clk_init_data init;

	pfd = kzalloc(sizeof(*pfd), GFP_KERNEL);
	if (!pfd)
		return ERR_PTR(-ENOMEM);

	pfd->reg = reg;
	pfd->idx = idx;

	init.name = name;
	init.ops = &clk_pfdv2_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pfd->hw.init = &init;

	clk = clk_register(NULL, &pfd->hw);
	if (IS_ERR(clk))
		kfree(pfd);

	return clk;
}
