// SPDX-License-Identifier: GPL-2.0
/*
 * CGU PLL driver for ADI SC59X processors
 *
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/device.h>

#include "clk.h"

struct clk_sc5xx_cgu_pll {
	struct clk_hw hw;
	void __iomem *base;
	spinlock_t *lock;
	int prepared;
	u32 mask;
	u32 msel;
	u32 m_offset;
	u8 shift;
};

struct clk_sc5xx_cgu_pll *to_clk_sc5xx_cgu_pll(struct clk_hw *hw)
{
	return container_of(hw, struct clk_sc5xx_cgu_pll, hw);
}

static long sc5xx_cgu_pll_round_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long *parent_rate)
{
	struct clk_sc5xx_cgu_pll *pll = to_clk_sc5xx_cgu_pll(hw);
	unsigned long m, m2, new_rate, nr2, prate2;
	unsigned long prate = *parent_rate;
	struct clk_hw *parent_hw;
	int parent_inc;

	parent_hw = clk_hw_get_parent(hw);

	if (!prate)
		return 0;

	m = rate / prate;

	if (m > pll->msel) {
		/* cannot scale this far, need bigger input */
		parent_inc = m / pll->msel;
		prate = clk_hw_round_rate(parent_hw, prate * (parent_inc + 1));
	} else if (m == 0) {
		pr_err("%s: Cannot use VCO to reduce parent clock rate, requested %lu, clamping to %lu\n",
			__func__, rate, prate);
		return prate;
	}

	new_rate = prate * m;

	if (new_rate != rate) {
		/*
		 * Check if we could get an integer match by halving parent rate since we
		 * know at least about the DF bit before the VCO, although we don't know
		 * if we're already using it or not
		 */
		prate2 = clk_hw_round_rate(parent_hw, prate / 2);
		m2 = rate / prate2;
		nr2 = prate * m2;
		if (m2 <= pll->msel && nr2 == rate) {
			new_rate = nr2;
			prate = prate2;
		}
	}

	*parent_rate = prate;
	return new_rate;
}

static unsigned long sc5xx_cgu_pll_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct clk_sc5xx_cgu_pll *pll = to_clk_sc5xx_cgu_pll(hw);
	u32 reg = readl(pll->base);
	u32 m = ((reg & pll->mask) >> pll->shift) + pll->m_offset;

	if (m == 0)
		m = pll->msel;

	return parent_rate * m;
}

static const struct clk_ops clk_sc5xx_cgu_pll_ops = {
	.recalc_rate = sc5xx_cgu_pll_recalc_rate,
	.round_rate = sc5xx_cgu_pll_round_rate,
};

struct clk *sc5xx_cgu_pll(const char *name, const char *parent_name,
	void __iomem *base, u8 shift, u8 width, u32 m_offset,
		spinlock_t *lock)
{
	struct clk_sc5xx_cgu_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.ops = &clk_sc5xx_cgu_pll_ops;

	pll->base = base;
	pll->hw.init = &init;
	pll->lock = lock;
	pll->shift = shift;
	pll->mask = GENMASK(width - 1, 0) << shift;
	pll->msel = pll->mask + 1;
	pll->m_offset = m_offset;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: Failed to register, code %lu\n", __func__,
			PTR_ERR(clk));
	}

	return clk;
}

MODULE_DESCRIPTION("Analog Devices CLock PLL driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Malysa <malysagreg@gmail.com>");
