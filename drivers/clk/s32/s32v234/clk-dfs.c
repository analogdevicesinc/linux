/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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
 * struct clk_dfs - S32 DFS clock
 * @clk_hw:	clock source
 * @reg:	DFS register address
 * @idx:	the index of DFS encoded in the register
 *
 * DFS clock found on S32 series. Each register for DFS has 4 clk_dfs
 * data encoded, and member idx is used to specify the one.
 * Only ARMPLL(3 DFS), ENETPLL(4 DFS) and DDRPLL(3 DFS) has DFS outputs.
 */
struct clk_dfs {
	struct clk_hw	hw;
	void __iomem	*reg;
	enum s32v234_plldig_type plltype;
	u8		idx;
	u32		mfn;
};

#define to_clk_dfs(_hw) container_of(_hw, struct clk_dfs, hw)

static int get_pllx_dfs_nr(enum s32v234_plldig_type plltype)
{
	switch (plltype) {
	case S32_PLLDIG_ARM:
		return ARMPLL_DFS_NR;
	case S32_PLLDIG_ENET:
		return ENETPLL_DFS_NR;
	case S32_PLLDIG_DDR:
		return DDRPLL_DFS_NR;
	case S32_PLLDIG_PERIPH:
	case S32_PLLDIG_VIDEO:
		pr_warn("Current selected PLL has no DFS\n");
		break;
	}

	return -EINVAL;
}
static unsigned long get_pllx_dfsy_max_rate(enum s32v234_plldig_type plltype,
					    int dfsno)
{
	switch (plltype) {
	case S32_PLLDIG_ARM:
		switch (dfsno) {
		case 0:
			return ARMPLL_DFS0_MAX_RATE;
		case 1:
			return ARMPLL_DFS1_MAX_RATE;
		case 2:
			return ARMPLL_DFS2_MAX_RATE;
		}
		break;
	case S32_PLLDIG_ENET:
		switch (dfsno) {
		case 0:
			return ENETPLL_DFS0_MAX_RATE;
		case 1:
			return ENETPLL_DFS1_MAX_RATE;
		case 2:
			return ENETPLL_DFS2_MAX_RATE;
		case 3:
			return ENETPLL_DFS3_MAX_RATE;
		}
		break;
	case S32_PLLDIG_DDR:
		switch (dfsno) {
		case 0:
			return DDRPLL_DFS0_MAX_RATE;
		case 1:
			return DDRPLL_DFS1_MAX_RATE;
		case 2:
			return DDRPLL_DFS2_MAX_RATE;
		}
		break;
	case S32_PLLDIG_PERIPH:
	case S32_PLLDIG_VIDEO:
		pr_warn("Current selected PLL has no DFS.");
		break;
	default:
		pr_warn("Unsupported PLL. Use %d or %d\n",
			S32_PLLDIG_ARM,	S32_PLLDIG_VIDEO);
		break;
	}

	return -EINVAL;
}
static int clk_dfs_enable(struct clk_hw *hw)
{
	/*
	 * TODO: When SOC is available, this function
	 * should be tested and implemented for DFS
	 * if it is possible
	 */
	return 0;
}

static void clk_dfs_disable(struct clk_hw *hw)
{
	/*
	 * TODO: When SOC is available, this function
	 * should be tested and implemented for DFS
	 * if it is possible
	 */
}

static unsigned long clk_dfs_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);
	u32 mfn, mfi, rate;
	u32 dvport = readl_relaxed(DFS_DVPORTn(dfs->reg, dfs->idx));

	mfn = (dvport & DFS_DVPORTn_MFN_MASK) >> DFS_DVPORTn_MFN_OFFSET;
	mfi = (dvport & DFS_DVPORTn_MFI_MASK) >> DFS_DVPORTn_MFI_OFFSET;
	mfi <<= 8;
	rate = parent_rate / (mfi + mfn);
	rate <<= 8;

	return rate;
}

static long clk_dfs_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);
	unsigned long max_allowed_rate;

	max_allowed_rate = get_pllx_dfsy_max_rate(dfs->plltype, dfs->idx);

	if (rate > max_allowed_rate)
		rate = max_allowed_rate;

	return rate;
}

static int clk_dfs_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);
	u32 mfi;
	u32 portreset = readl_relaxed(DFS_PORTRESET(dfs->reg));

	writel_relaxed(DFS_CTRL_DLL_RESET, DFS_CTRL(dfs->reg));
	writel_relaxed(portreset | ~DFS_PORTRESET_PORTRESET_SET(dfs->idx),
			DFS_PORTRESET(dfs->reg));

	mfi = parent_rate/rate;
	writel_relaxed(DFS_DVPORTn_MFI_SET(mfi) |
		       DFS_DVPORTn_MFN_SET(dfs->mfn),
		       DFS_DVPORTn(dfs->reg, dfs->idx));

	writel_relaxed(~DFS_CTRL_DLL_RESET, DFS_CTRL(dfs->reg));

	while (readl_relaxed(DFS_PORTSR(dfs->reg)) & (1 << (dfs->idx)))
		;

	return 0;
}

static int clk_dfs_is_enabled(struct clk_hw *hw)
{
	struct clk_dfs *dfs = to_clk_dfs(hw);

	/* Check if current DFS output port is locked */
	if (readl_relaxed(DFS_PORTSR(dfs->reg)) & (1 << (dfs->idx)))
		return 0;

	return 1;
}

static const struct clk_ops clk_dfs_ops = {
	.enable		= clk_dfs_enable,
	.disable	= clk_dfs_disable,
	.recalc_rate	= clk_dfs_recalc_rate,
	.round_rate	= clk_dfs_round_rate,
	.set_rate	= clk_dfs_set_rate,
	.is_enabled	= clk_dfs_is_enabled,
};

struct clk *s32v234_clk_dfs(enum s32v234_plldig_type type, const char *name,
			const char *parent_name, void __iomem *reg,
			u8 idx, u32 mfn)
{
	struct clk_dfs *dfs;
	struct clk *clk;
	struct clk_init_data init;

	/* PERIPH and VIDEO PLL do not have DFS */
	if (type == S32_PLLDIG_PERIPH || type == S32_PLLDIG_VIDEO)
		return ERR_PTR(-EINVAL);

	/* check if DFS index is valid for current pll */
	if (idx >= get_pllx_dfs_nr(type))
		return ERR_PTR(-EINVAL);

	dfs = kzalloc(sizeof(*dfs), GFP_KERNEL);
	if (!dfs)
		return ERR_PTR(-ENOMEM);

	dfs->reg = reg;
	dfs->idx = idx;
	dfs->mfn = mfn;
	dfs->plltype = type;

	init.name = name;
	init.ops = &clk_dfs_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	dfs->hw.init = &init;

	clk = clk_register(NULL, &dfs->hw);
	if (IS_ERR(clk))
		kfree(dfs);

	return clk;
}
