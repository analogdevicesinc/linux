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
 * @gate_bit:	Gate bit offset
 * @vld_bit:	Valid bit offset
 * @frac_off:	PLL Fractional Divider offset
 */

struct clk_pfdv2 {
	struct clk_hw	hw;
	void __iomem	*reg;
	u32		gate_bit;
	u32		vld_bit;
	u8		frac_off;
};

#define to_clk_pfdv2(_hw) container_of(_hw, struct clk_pfdv2, hw)

#define CLK_PFDV2_FRAC_MASK 0x3f

static int clk_pfd_enable(struct clk_hw *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u32 val;

	val = readl_relaxed(pfd->reg);
	val &= ~pfd->gate_bit;
	writel_relaxed(val, pfd->reg);

	return 0;
}

static void clk_pfd_disable(struct clk_hw *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u32 val;

	val = readl_relaxed(pfd->reg);
	val |= pfd->gate_bit;
	writel_relaxed(val, pfd->reg);
}

static unsigned long clk_pfd_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u64 tmp = parent_rate;
	u8 frac;

	frac = (readl_relaxed(pfd->reg) >> pfd->frac_off)
		& CLK_PFDV2_FRAC_MASK;

	if (!frac) {
		pr_debug("clk_pfdv2: %s invalid pfd frac value 0\n",
			 clk_hw_get_name(hw));
		return 0;
	}

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

static int clk_pfd_is_enabled(struct clk_hw *hw)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);

	if (readl_relaxed(pfd->reg) & pfd->gate_bit)
		return 0;

	return 1;
}

static int clk_pfd_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pfdv2 *pfd = to_clk_pfdv2(hw);
	u64 tmp = parent_rate;
	u32 val;
	u8 frac;

	if (!rate)
		return -EINVAL;

	/* PFD can NOT change rate without gating */
	WARN_ON(clk_pfd_is_enabled(hw));

	tmp = tmp * 18 + rate / 2;
	do_div(tmp, rate);
	frac = tmp;
	if (frac < 12)
		frac = 12;
	else if (frac > 35)
		frac = 35;

	val = readl_relaxed(pfd->reg);
	val &= ~(CLK_PFDV2_FRAC_MASK << pfd->frac_off);
	val |= frac << pfd->frac_off;
	writel_relaxed(val, pfd->reg);

	return 0;
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
	pfd->gate_bit = 1 << ((idx + 1) * 8 - 1);
	pfd->vld_bit = pfd->gate_bit >> 1;
	pfd->frac_off = idx * 8;

	init.name = name;
	init.ops = &clk_pfdv2_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_GATE;

	pfd->hw.init = &init;

	clk = clk_register(NULL, &pfd->hw);
	if (IS_ERR(clk))
		kfree(pfd);

	return clk;
}
