// SPDX-License-Identifier: GPL-2.0-only
/*
 * Divider clock CCF support for Analog Devices ADSP SoCs.
 *
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * The ADSP SoCs contain multiple divider clocks, that allow users
 * to change the frequency system clocks operate at. Updating the
 * dividers involves updating the divider register and aligning
 * the system clocks.
 */

/* CGU register offsets */
#define ADSP_CGU_CTL				0x00
#define ADSP_CGU_PLLCTL				0x04
#define ADSP_CGU_STAT				0x08
#define ADSP_CGU_DIV				0x0c
#define ADSP_CGU_CLKOUTSEL			0x10
#define ADSP_CGU_DIVEX				0x40
#define ADSP_CGU_REVID				0x48

#define ADSP_CGU_DIV_CSEL_MASK			GENMASK(4, 0)
#define ADSP_CGU_DIV_S0SEL_MASK			GENMASK(7, 5)
#define ADSP_CGU_DIV_SYSSEL_MASK		GENMASK(12, 8)
#define ADSP_CGU_DIV_S1SEL_MASK			GENMASK(15, 13)
#define ADSP_CGU_DIV_DSEL_MASK			GENMASK(20, 16)
#define ADSP_CGU_DIV_OSEL_MASK			GENMASK(28, 22)
#define ADSP_CGU_DIV_ALGN			BIT(29)
#define ADSP_CGU_DIV_UPDT			BIT(30)
#define ADSP_CGU_DIV_LOCK			BIT(31)

#define ADSP_CGU_DIVEX_S0SELEX_MASK		GENMASK(7, 0)
#define ADSP_CGU_DIVEX_S1SELEX_MASK		GENMASK(23, 16)

#define ADSP_CGU_CTL_S0SELEXEN			BIT(16)
#define ADSP_CGU_CTL_S1SELEXEN			BIT(17)

#define ADSP_CGU_STAT_CLKSALGN			BIT(3)
#define ADSP_CGU_STAT_ADDRERR			BIT(16)
#define ADSP_CGU_STAT_LWERR			BIT(17)
#define ADSP_CGU_STAT_WDFMSERR			BIT(19)
#define ADSP_CGU_STAT_WDIVERR			BIT(20)
#define ADSP_CGU_STAT_PCFGERR			BIT(21)

#define ADSP_CGU_CLK_DIV_FLAGS			CLK_DIVIDER_MAX_AT_ZERO

struct adsp_div {
	spinlock_t lock;
	void __iomem *base;
	struct clk_hw clk_hw;
};

static inline struct adsp_div *to_adsp_div(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct adsp_div, clk_hw);
}

static const struct clk_ops adsp_div_ops = {
	.recalc_rate = adsp_div_recalc_rate,
	.determine_rate = clk_hw_determine_rate_no_reparent,
	.set_rate = adsp_div_set_rate,
	.debug_init = adsp_div_debug_init,	
};

static int adsp_div_register(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	


	return 0;
}

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADSP SoC divider clock support");
MODULE_LICENSE("GPL");
