// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "clk.h"

/*
 * struct clk_plldig - S32 PLLDIG clock
 * @clk_hw:	   clock source
 * @base:	   base address of PLL registers
 * @plldv_mfd:	   multiplication loop factor divider
 * @plldv_rfdphi:  PHI reduced frequency divider
 * @plldv_rfdphi1: PHI reduced frequency divider
 *
 * PLLDIG clock version 1, found on S32 series.
 */
struct clk_plldig {
	struct clk_hw	hw;
	void __iomem	*base;
	enum s32v234_plldig_type type;
	u32		plldv_mfd;
	u32		pllfd_mfn;
	u32		plldv_rfdphi;
	u32		plldv_rfdphi1;
};

#define to_clk_plldig(_hw) container_of(_hw, struct clk_plldig, hw)

static unsigned long get_pllx_max_vco_rate(enum s32v234_plldig_type plltype)
{
	switch (plltype) {
	case S32_PLLDIG_PERIPH:
		return PERIPHPLL_MAX_VCO_RATE;
	default:
		pr_warn("Unsupported PLL.\n");
			return -EINVAL;
	}
}

static unsigned long get_pllx_phiy_max_rate(enum s32v234_plldig_type plltype,
					    unsigned int phino)
{
	switch (plltype) {
	case S32_PLLDIG_PERIPH:
		switch (phino) {
		case 0:
			return PERIPHPLL_MAX_PHI0_MAX_RATE;
		case 1:
			return PERIPHPLL_MAX_PHI1_MAX_RATE;
		default:
			break;
		}
		break;
	default:
		pr_warn("Unsupported PLL.\n");
		break;
	}
	return -EINVAL;
}

static int clk_plldig_prepare(struct clk_hw *hw)
{
	return 0;
}

static void clk_plldig_unprepare(struct clk_hw *hw)
{
}

static unsigned long clk_plldig_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	u32 plldv = readl_relaxed(PLLDIG_PLLDV(pll->base));
	u32 pllfd = readl_relaxed(PLLDIG_PLLFD(pll->base));
	u32 prediv, mfd, mfn, vco;

	prediv = (plldv & PLLDIG_PLLDV_PREDIV_MASK)
			 >> PLLDIG_PLLDV_PREDIV_OFFSET;
	mfd = (plldv & PLLDIG_PLLDV_MFD_MASK);

	mfn = (pllfd & PLLDIG_PLLFD_MFN_MASK);

	if (prediv == 0)
		prediv = 1;

	/*
	 * This formula is from platform reference manual
	 * (Rev. 1, 6/2015), PLLDIG chapter.
	 */
	vco = (parent_rate / prediv) * (mfd + mfn / 20480);

	return vco;
}

static long clk_plldig_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	unsigned long max_allowed_rate = get_pllx_max_vco_rate(pll->type);

	if (rate > max_allowed_rate)
		rate = max_allowed_rate;
	else if (rate < MIN_VCO_RATE)
		rate = MIN_VCO_RATE;

	return rate;
}

static int clk_plldig_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	u32 pllfd, prediv;

	unsigned long max_allowed_rate = get_pllx_max_vco_rate(pll->type);
	unsigned long phi0_max_rate =  get_pllx_phiy_max_rate(pll->type, 0);
	unsigned long phi1_max_rate =  get_pllx_phiy_max_rate(pll->type, 1);

	if (rate < MIN_VCO_RATE || rate > max_allowed_rate)
		return -EINVAL;

	if (((rate / pll->plldv_rfdphi) > phi0_max_rate) ||
	    ((rate / pll->plldv_rfdphi) > phi1_max_rate))
		return -EINVAL;

	pllfd = readl_relaxed(PLLDIG_PLLFD(pll->base));
	prediv = (parent_rate / rate) *
		 (pll->plldv_mfd + pll->pllfd_mfn / 20480);

	writel_relaxed(PLLDIG_PLLDV_RFDPHI1_SET(pll->plldv_rfdphi1) |
			PLLDIG_PLLDV_RFDPHI_SET(pll->plldv_rfdphi) |
			PLLDIG_PLLDV_PREDIV_SET(prediv) |
			PLLDIG_PLLDV_MFD_SET(pll->plldv_mfd),
			PLLDIG_PLLDV(pll->base));

	writel_relaxed(pllfd | PLLDIG_PLLFD_MFN_SET(pll->pllfd_mfn),
		       PLLDIG_PLLFD(pll->base));

	/*
	 * To be implemented the wait_lock or an equivalent state
	 * return clk_plldig_wait_lock(pll);
	 */
	return 0;
}

static const struct clk_ops clk_plldig_ops = {
	.prepare	= clk_plldig_prepare,
	.unprepare	= clk_plldig_unprepare,
	.recalc_rate	= clk_plldig_recalc_rate,
	.round_rate	= clk_plldig_round_rate,
	.set_rate	= clk_plldig_set_rate,
};

struct clk *s32v234_clk_plldig_phi(enum s32v234_plldig_type type,
				   const char *name, const char *parent,
				   void __iomem *base, u32 phi)
{
	u32 plldv, rfd_phi;

	if (!base)
		return ERR_PTR(-ENOMEM);

	plldv = readl_relaxed(PLLDIG_PLLDV(base));

	switch (phi) {
	/* PHI0 */
	case 0:
		rfd_phi = (plldv & PLLDIG_PLLDV_RFDPHI_MASK)
			  >> PLLDIG_PLLDV_RFDPHI_OFFSET;
		break;
	/* PHI1 */
	case 1:
		rfd_phi = (plldv & PLLDIG_PLLDV_RFDPHI1_MASK)
			  >> PLLDIG_PLLDV_RFDPHI1_OFFSET;

		if (rfd_phi == 0)
			rfd_phi = 1;

		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, 1, rfd_phi);
}

struct clk *s32v234_clk_plldig(enum s32v234_plldig_type type, const char *name,
			       const char *parent_name, void __iomem *base,
			       u32 plldv_mfd, u32 pllfd_mfn,
			       u32 plldv_rfdphi, u32 plldv_rfdphi1)
{
	struct clk_plldig *pll;
	const struct clk_ops *ops;
	struct clk *clk;
	struct clk_init_data init;

	if (plldv_rfdphi > PLLDIG_PLLDV_RFDPHI_MAXVALUE)
		return ERR_PTR(-EINVAL);

	if (plldv_rfdphi1 > PLLDIG_PLLDV_RFDPHI1_MAXVALUE)
		return ERR_PTR(-EINVAL);

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	ops = &clk_plldig_ops;

	pll->base = base;
	pll->type = type;
	pll->plldv_mfd = plldv_mfd;
	pll->pllfd_mfn = pllfd_mfn;
	pll->plldv_rfdphi = plldv_rfdphi;
	pll->plldv_rfdphi1 = plldv_rfdphi1;

	init.name = name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
