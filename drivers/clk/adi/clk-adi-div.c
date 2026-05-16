// SPDX-License-Identifier: GPL-2.0-only
/*
 * Divider clock support for ADSP-SC5xx processors
 *
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * Author: Qasim Ijaz <qasim.ijaz@analog.com>
 * Contact: linux@analog.com
 */

#define SC5XX_CGU_CTL              0x00
#define SC5XX_CGU_STAT             0x08
#define SC5XX_CGU_DIV              0x0c
#define SC5XX_CGU_DIVEX            0x40

#define SC5XX_CGU_CTL_S0SELEXEN    BIT(16)
#define SC5XX_CGU_CTL_S1SELEXEN    BIT(17)
#define SC5XX_CGU_CTL_LOCK         BIT(31)

#define SC5XX_CGU_STAT_PLLEN       BIT(0)
#define SC5XX_CGU_STAT_PLLBP       BIT(1)
#define SC5XX_CGU_STAT_PLOCK       BIT(2)
#define SC5XX_CGU_STAT_CLKSALGN    BIT(3)
#define SC5XX_CGU_STAT_ADDRERR     BIT(16)
#define SC5XX_CGU_STAT_LWERR       BIT(17)
#define SC5XX_CGU_STAT_WDIVERR     BIT(20)

#define SC5XX_CGU_STAT_DIV_ERRORS \
	(SC5XX_CGU_STAT_ADDRERR | \
	 SC5XX_CGU_STAT_LWERR | \
	 SC5XX_CGU_STAT_WDIVERR)

#define SC5XX_CGU_DIV_CSEL         GENMASK(4, 0)
#define SC5XX_CGU_DIV_S0SEL        GENMASK(7, 5)
#define SC5XX_CGU_DIV_SYSSEL       GENMASK(12, 8)
#define SC5XX_CGU_DIV_S1SEL        GENMASK(15, 13)
#define SC5XX_CGU_DIV_DSEL         GENMASK(20, 16)
#define SC5XX_CGU_DIV_OSEL         GENMASK(28, 22)
#define SC5XX_CGU_DIV_ALGN         BIT(29)
#define SC5XX_CGU_DIV_UPDT         BIT(30)
#define SC5XX_CGU_DIV_LOCK         BIT(31)

#define SC5XX_CGU_DIVEX_S0SELEX    GENMASK(7, 0)
#define SC5XX_CGU_DIVEX_S1SELEX    GENMASK(23, 16)

#define SC5XX_CGU_POLL_DELAY_US    1
#define SC5XX_CGU_POLL_TIMEOUT_US  500

struct sc5xx_div {
	u8 shift;
	u8 width;
	spinlock_t *lock;
	void __iomem *base;
	struct clk_hw clk_hw;
	unsigned long soc_flags;
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
	struct sc5xx_div *clk_div = to_sc5xx_div(hw);
	unsigned long flags;
	int ret, val, div;
	u32 stat;

	spin_lock_irqsave(clk_div->lock, flags);

	/* If PLL is in bypass mode then don't change rate yet */
	stat = sc5xx_div_readl(clk_div->base, SC5XX_CGU_STAT);
	if (stat & SC5XX_CGU_STAT_PLLBP) {
		ret = -EBUSY;
		goto out;
	}

	div = divider_get_val(rate, parent_rate, NULL,
				clk_div->width, CLK_DIVIDER_MAX_AT_ZERO);

	if (clk_div->soc_flags & SC5XX_SYSCLK_CCLK) {
		// maybe add a check to ensure cclk is not smaller than sysclk	
	}

	/* Ensure clock alignment sequence is not in progress */
	ret = readl_poll_timeout_atomic(clk_div->base + SC5XX_CGU_STAT, 
					val, !(val & SC5XX_CGU_STAT_CLKSALGN),
					SC5XX_CGU_POLL_DELAY_US,
					SC5XX_CGU_POLL_TIMEOUT_US);
	if (ret)
		goto out;
	
	if (clk_div->soc_flags & SC5XX_EXTENSION_DIV) {
	
	}
	else {
		reg = sc5xx_div_readl(clk_div, SC5XX_CGU_DIV);
		


	}

out:
	spin_lock_irqrestore(clk_div->lock, flags);	

	return ret;	
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
				const char * const *parent_names, u8 width,
				u8 shift unsigned long clock_flags, 
				unsigned long soc_flags, spinlock_t *lock)
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
	clk_div->width = width;
	clk_div->shift = shift;
	clk_div->soc_flags = soc_flags;
	clk_div->mask = ((1 << width) - 1) << shift;
			or use GENMASK() with shift and width

	clk = clk_register(NULL, &clk_div->clk_hw);
	if (IS_ERR(clk))
		kfree(cdu_clk);

	return clk;
}

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices Inc ADSP-SC5XX SoC divider clock driver");
MODULE_LICENSE("GPL v2");
