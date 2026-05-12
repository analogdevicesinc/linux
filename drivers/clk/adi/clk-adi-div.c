// SPDX-License-Identifier: GPL-2.0-only
/*
 * Divider clock support for ADSP-SC5xx processors
 *
 * Copyright (C) 2026 Analog Devices Inc.
 */

struct sc5xx_div {
	spinlock_t *lock;
	void __iomem *base;
	struct clk_hw clk_hw;
	u8 shift;
	u8 width;
};

static inline sc5xx_div *to_sc5xx_div(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct sc5xx_div, clk_hw);
}

static u32 sc5xx_div_readl(struct sc5xx_div *clk_div, unsigned int offset)
{
	return readl(clk_div->base + offset);
}

static void sc5xx_div_writel(struct sc5xx_div *clk_div,
				unsigned int offset, u32 val)
{
	writel(val, clk_div->base + offset);
}

static int set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{

}

static unsigned long recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{

}

static long round_rate(struct clk_hw *hw, unsigned long rate, 
			unsigned long *parent_rate)
{

}

static int *determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{

}

static void debug_init(struct clk_hw *hw, struct dentry *dentry)
{

}

static const struct clk_ops sc5xx_div_ops = {
	.set_rate = 
	.recalc_rate = 
	.round_rate = 
	.determine_rate = // maybe implement if round_rate doesnt suffice?
	.debug_init = 	
};
	u8 width;

struct clk *sc5xx_div_register(const char *clock_name, void __iomem *base,
				const char * const *parent_names,
				unsigned long clock_flags, spinlock_t *lock)
{
	struct sc5xx_div *clk_div;
	struct clk_init_data init = { };
	struct clk *clk;

	clk_div = kzalloc(sizeof(*clk_div), GFP_KERNEL);
	if (!clk_div)
		return ERR_PTR(-ENOMEM);

	init.name = clock_name;
	init.ops = &sc5xx_div_ops;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	init.flags = CLK_SET_RATE_PARENT | clock_flags;

	clk_div->clk_hw.init = &init;
	clk_div->lock = lock;


	clk = clk_register(NULL, &clk_div->clk_hw);
	if (IS_ERR(clk))
		kfree(cdu_clk);

	return clk;
}

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices Inc ADSP-SC5XX SoC divider clock driver");
MODULE_LICENSE("GPL v2");
