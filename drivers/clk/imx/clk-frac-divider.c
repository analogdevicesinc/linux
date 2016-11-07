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

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/gcd.h>

#include "clk.h"

#define to_clk_frac_divider(_hw) container_of(_hw, struct clk_frac_divider, hw)

static unsigned long clk_frac_divider_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct clk_frac_divider *fd = to_clk_frac_divider(hw);
	u32 val, m, n;
	u64 ret;

	val = readl_relaxed(fd->reg);

	m = (val & fd->mmask) >> fd->mshift;
	n = (val & fd->nmask) >> fd->nshift;

	ret = (u64)parent_rate * (m + 1);
	do_div(ret, n + 1);

	return ret;
}

static long clk_frac_divider_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	struct clk_frac_divider *fd = to_clk_frac_divider(hw);
	unsigned long div;
	unsigned maxn = (fd->nmask >> fd->nshift) + 2;

	if (!rate || rate >= *parent_rate)
		return *parent_rate;

	div = gcd(*parent_rate, rate);

	while ((*parent_rate / div) > maxn) {
		div <<= 1;
		rate <<= 1;
	}

	return rate;
}

static int clk_frac_divider_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct clk_frac_divider *fd = to_clk_frac_divider(hw);
	unsigned long div;
	unsigned long m, n;
	u32 val;

	div = gcd(parent_rate, rate);
	m = rate / div - 1;
	n = parent_rate / div - 1;
	if (m && !n)
		return -EINVAL;

	val = readl_relaxed(fd->reg);
	val &= ~(fd->mmask | fd->nmask);
	val |= (m << fd->mshift) | (n << fd->nshift);
	writel_relaxed(val, fd->reg);

	return 0;
}

const struct clk_ops clk_frac_divider_ops = {
	.recalc_rate = clk_frac_divider_recalc_rate,
	.round_rate = clk_frac_divider_round_rate,
	.set_rate = clk_frac_divider_set_rate,
};
EXPORT_SYMBOL_GPL(clk_frac_divider_ops);
