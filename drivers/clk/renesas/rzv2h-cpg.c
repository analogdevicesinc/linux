// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/V2H(P) Clock Pulse Generator
 *
 * Copyright (C) 2024 Renesas Electronics Corp.
 *
 * Based on rzg2l-cpg.c
 *
 * Copyright (C) 2015 Glider bvba
 * Copyright (C) 2013 Ideas On Board SPRL
 * Copyright (C) 2015 Renesas Electronics Corp.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>
#include <linux/refcount.h>
#include <linux/reset-controller.h>
#include <linux/string_choices.h>

#include <dt-bindings/clock/renesas-cpg-mssr.h>

#include "rzv2h-cpg.h"

#ifdef DEBUG
#define WARN_DEBUG(x)		WARN_ON(x)
#else
#define WARN_DEBUG(x)		do { } while (0)
#endif

#define GET_CLK_ON_OFFSET(x)	(0x600 + ((x) * 4))
#define GET_CLK_MON_OFFSET(x)	(0x800 + ((x) * 4))
#define GET_RST_OFFSET(x)	(0x900 + ((x) * 4))
#define GET_RST_MON_OFFSET(x)	(0xA00 + ((x) * 4))

#define CPG_BUS_1_MSTOP		(0xd00)
#define CPG_BUS_MSTOP(m)	(CPG_BUS_1_MSTOP + ((m) - 1) * 4)

#define CPG_PLL_STBY(x)		((x))
#define CPG_PLL_STBY_RESETB	BIT(0)
#define CPG_PLL_STBY_RESETB_WEN	BIT(16)
#define CPG_PLL_CLK1(x)		((x) + 0x004)
#define CPG_PLL_CLK1_KDIV(x)	((s16)FIELD_GET(GENMASK(31, 16), (x)))
#define CPG_PLL_CLK1_MDIV(x)	FIELD_GET(GENMASK(15, 6), (x))
#define CPG_PLL_CLK1_PDIV(x)	FIELD_GET(GENMASK(5, 0), (x))
#define CPG_PLL_CLK2(x)		((x) + 0x008)
#define CPG_PLL_CLK2_SDIV(x)	FIELD_GET(GENMASK(2, 0), (x))
#define CPG_PLL_MON(x)		((x) + 0x010)
#define CPG_PLL_MON_RESETB	BIT(0)
#define CPG_PLL_MON_LOCK	BIT(4)

#define DDIV_DIVCTL_WEN(shift)		BIT((shift) + 16)

#define GET_MOD_CLK_ID(base, index, bit)		\
			((base) + ((((index) * (16))) + (bit)))

#define CPG_CLKSTATUS0		(0x700)

/**
 * struct rzv2h_cpg_priv - Clock Pulse Generator Private Data
 *
 * @dev: CPG device
 * @base: CPG register block base address
 * @rmw_lock: protects register accesses
 * @clks: Array containing all Core and Module Clocks
 * @num_core_clks: Number of Core Clocks in clks[]
 * @num_mod_clks: Number of Module Clocks in clks[]
 * @resets: Array of resets
 * @num_resets: Number of Module Resets in info->resets[]
 * @last_dt_core_clk: ID of the last Core Clock exported to DT
 * @ff_mod_status_ops: Fixed Factor Module Status Clock operations
 * @mstop_count: Array of mstop values
 * @rcdev: Reset controller entity
 */
struct rzv2h_cpg_priv {
	struct device *dev;
	void __iomem *base;
	spinlock_t rmw_lock;

	struct clk **clks;
	unsigned int num_core_clks;
	unsigned int num_mod_clks;
	struct rzv2h_reset *resets;
	unsigned int num_resets;
	unsigned int last_dt_core_clk;

	struct clk_ops *ff_mod_status_ops;

	atomic_t *mstop_count;

	struct reset_controller_dev rcdev;
};

#define rcdev_to_priv(x)	container_of(x, struct rzv2h_cpg_priv, rcdev)

struct pll_clk {
	struct rzv2h_cpg_priv *priv;
	struct clk_hw hw;
	struct pll pll;
};

#define to_pll(_hw)	container_of(_hw, struct pll_clk, hw)

/**
 * struct mod_clock - Module clock
 *
 * @priv: CPG private data
 * @mstop_data: mstop data relating to module clock
 * @hw: handle between common and hardware-specific interfaces
 * @no_pm: flag to indicate PM is not supported
 * @on_index: register offset
 * @on_bit: ON/MON bit
 * @mon_index: monitor register offset
 * @mon_bit: monitor bit
 * @ext_clk_mux_index: mux index for external clock source, or -1 if internal
 */
struct mod_clock {
	struct rzv2h_cpg_priv *priv;
	unsigned int mstop_data;
	struct clk_hw hw;
	bool no_pm;
	u8 on_index;
	u8 on_bit;
	s8 mon_index;
	u8 mon_bit;
	s8 ext_clk_mux_index;
};

#define to_mod_clock(_hw) container_of(_hw, struct mod_clock, hw)

/**
 * struct ddiv_clk - DDIV clock
 *
 * @priv: CPG private data
 * @div: divider clk
 * @mon: monitor bit in CPG_CLKSTATUS0 register
 */
struct ddiv_clk {
	struct rzv2h_cpg_priv *priv;
	struct clk_divider div;
	u8 mon;
};

#define to_ddiv_clock(_div) container_of(_div, struct ddiv_clk, div)

/**
 * struct rzv2h_ff_mod_status_clk - Fixed Factor Module Status Clock
 *
 * @priv: CPG private data
 * @conf: fixed mod configuration
 * @fix: fixed factor clock
 */
struct rzv2h_ff_mod_status_clk {
	struct rzv2h_cpg_priv *priv;
	struct fixed_mod_conf conf;
	struct clk_fixed_factor fix;
};

#define to_rzv2h_ff_mod_status_clk(_hw) \
	container_of(_hw, struct rzv2h_ff_mod_status_clk, fix.hw)

static int rzv2h_cpg_pll_clk_is_enabled(struct clk_hw *hw)
{
	struct pll_clk *pll_clk = to_pll(hw);
	struct rzv2h_cpg_priv *priv = pll_clk->priv;
	u32 val = readl(priv->base + CPG_PLL_MON(pll_clk->pll.offset));

	/* Ensure both RESETB and LOCK bits are set */
	return (val & (CPG_PLL_MON_RESETB | CPG_PLL_MON_LOCK)) ==
	       (CPG_PLL_MON_RESETB | CPG_PLL_MON_LOCK);
}

static int rzv2h_cpg_pll_clk_enable(struct clk_hw *hw)
{
	struct pll_clk *pll_clk = to_pll(hw);
	struct rzv2h_cpg_priv *priv = pll_clk->priv;
	struct pll pll = pll_clk->pll;
	u32 stby_offset;
	u32 mon_offset;
	u32 val;
	int ret;

	if (rzv2h_cpg_pll_clk_is_enabled(hw))
		return 0;

	stby_offset = CPG_PLL_STBY(pll.offset);
	mon_offset = CPG_PLL_MON(pll.offset);

	writel(CPG_PLL_STBY_RESETB_WEN | CPG_PLL_STBY_RESETB,
	       priv->base + stby_offset);

	/*
	 * Ensure PLL enters into normal mode
	 *
	 * Note: There is no HW information about the worst case latency.
	 *
	 * Since this latency might depend on external crystal or PLL rate,
	 * use a "super" safe timeout value.
	 */
	ret = readl_poll_timeout_atomic(priv->base + mon_offset, val,
			(val & (CPG_PLL_MON_RESETB | CPG_PLL_MON_LOCK)) ==
			(CPG_PLL_MON_RESETB | CPG_PLL_MON_LOCK), 200, 2000);
	if (ret)
		dev_err(priv->dev, "Failed to enable PLL 0x%x/%pC\n",
			stby_offset, hw->clk);

	return ret;
}

static unsigned long rzv2h_cpg_pll_clk_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	struct pll_clk *pll_clk = to_pll(hw);
	struct rzv2h_cpg_priv *priv = pll_clk->priv;
	struct pll pll = pll_clk->pll;
	unsigned int clk1, clk2;
	u64 rate;

	if (!pll.has_clkn)
		return 0;

	clk1 = readl(priv->base + CPG_PLL_CLK1(pll.offset));
	clk2 = readl(priv->base + CPG_PLL_CLK2(pll.offset));

	rate = mul_u64_u32_shr(parent_rate, (CPG_PLL_CLK1_MDIV(clk1) << 16) +
			       CPG_PLL_CLK1_KDIV(clk1), 16 + CPG_PLL_CLK2_SDIV(clk2));

	return DIV_ROUND_CLOSEST_ULL(rate, CPG_PLL_CLK1_PDIV(clk1));
}

static const struct clk_ops rzv2h_cpg_pll_ops = {
	.is_enabled = rzv2h_cpg_pll_clk_is_enabled,
	.enable = rzv2h_cpg_pll_clk_enable,
	.recalc_rate = rzv2h_cpg_pll_clk_recalc_rate,
};

static struct clk * __init
rzv2h_cpg_pll_clk_register(const struct cpg_core_clk *core,
			   struct rzv2h_cpg_priv *priv,
			   const struct clk_ops *ops)
{
	struct device *dev = priv->dev;
	struct clk_init_data init;
	const struct clk *parent;
	const char *parent_name;
	struct pll_clk *pll_clk;
	int ret;

	parent = priv->clks[core->parent];
	if (IS_ERR(parent))
		return ERR_CAST(parent);

	pll_clk = devm_kzalloc(dev, sizeof(*pll_clk), GFP_KERNEL);
	if (!pll_clk)
		return ERR_PTR(-ENOMEM);

	parent_name = __clk_get_name(parent);
	init.name = core->name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll_clk->hw.init = &init;
	pll_clk->pll = core->cfg.pll;
	pll_clk->priv = priv;

	ret = devm_clk_hw_register(dev, &pll_clk->hw);
	if (ret)
		return ERR_PTR(ret);

	return pll_clk->hw.clk;
}

static unsigned long rzv2h_ddiv_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	unsigned int val;

	val = readl(divider->reg) >> divider->shift;
	val &= clk_div_mask(divider->width);

	return divider_recalc_rate(hw, parent_rate, val, divider->table,
				   divider->flags, divider->width);
}

static int rzv2h_ddiv_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	struct clk_divider *divider = to_clk_divider(hw);

	return divider_determine_rate(hw, req, divider->table, divider->width,
				      divider->flags);
}

static inline int rzv2h_cpg_wait_ddiv_clk_update_done(void __iomem *base, u8 mon)
{
	u32 bitmask = BIT(mon);
	u32 val;

	if (mon == CSDIV_NO_MON)
		return 0;

	return readl_poll_timeout_atomic(base + CPG_CLKSTATUS0, val, !(val & bitmask), 10, 200);
}

static int rzv2h_ddiv_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	struct ddiv_clk *ddiv = to_ddiv_clock(divider);
	struct rzv2h_cpg_priv *priv = ddiv->priv;
	unsigned long flags = 0;
	int value;
	u32 val;
	int ret;

	value = divider_get_val(rate, parent_rate, divider->table,
				divider->width, divider->flags);
	if (value < 0)
		return value;

	spin_lock_irqsave(divider->lock, flags);

	ret = rzv2h_cpg_wait_ddiv_clk_update_done(priv->base, ddiv->mon);
	if (ret)
		goto ddiv_timeout;

	val = readl(divider->reg) | DDIV_DIVCTL_WEN(divider->shift);
	val &= ~(clk_div_mask(divider->width) << divider->shift);
	val |= (u32)value << divider->shift;
	writel(val, divider->reg);

	ret = rzv2h_cpg_wait_ddiv_clk_update_done(priv->base, ddiv->mon);

ddiv_timeout:
	spin_unlock_irqrestore(divider->lock, flags);
	return ret;
}

static const struct clk_ops rzv2h_ddiv_clk_divider_ops = {
	.recalc_rate = rzv2h_ddiv_recalc_rate,
	.determine_rate = rzv2h_ddiv_determine_rate,
	.set_rate = rzv2h_ddiv_set_rate,
};

static struct clk * __init
rzv2h_cpg_ddiv_clk_register(const struct cpg_core_clk *core,
			    struct rzv2h_cpg_priv *priv)
{
	struct ddiv cfg_ddiv = core->cfg.ddiv;
	struct clk_init_data init = {};
	struct device *dev = priv->dev;
	u8 shift = cfg_ddiv.shift;
	u8 width = cfg_ddiv.width;
	const struct clk *parent;
	const char *parent_name;
	struct clk_divider *div;
	struct ddiv_clk *ddiv;
	int ret;

	parent = priv->clks[core->parent];
	if (IS_ERR(parent))
		return ERR_CAST(parent);

	parent_name = __clk_get_name(parent);

	if ((shift + width) > 16)
		return ERR_PTR(-EINVAL);

	ddiv = devm_kzalloc(priv->dev, sizeof(*ddiv), GFP_KERNEL);
	if (!ddiv)
		return ERR_PTR(-ENOMEM);

	init.name = core->name;
	if (cfg_ddiv.no_rmw)
		init.ops = &clk_divider_ops;
	else
		init.ops = &rzv2h_ddiv_clk_divider_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_PARENT;

	ddiv->priv = priv;
	ddiv->mon = cfg_ddiv.monbit;
	div = &ddiv->div;
	div->reg = priv->base + cfg_ddiv.offset;
	div->shift = shift;
	div->width = width;
	div->flags = core->flag;
	div->lock = &priv->rmw_lock;
	div->hw.init = &init;
	div->table = core->dtable;

	ret = devm_clk_hw_register(dev, &div->hw);
	if (ret)
		return ERR_PTR(ret);

	return div->hw.clk;
}

static struct clk * __init
rzv2h_cpg_mux_clk_register(const struct cpg_core_clk *core,
			   struct rzv2h_cpg_priv *priv)
{
	struct smuxed mux = core->cfg.smux;
	const struct clk_hw *clk_hw;

	clk_hw = devm_clk_hw_register_mux(priv->dev, core->name,
					  core->parent_names, core->num_parents,
					  core->flag, priv->base + mux.offset,
					  mux.shift, mux.width,
					  core->mux_flags, &priv->rmw_lock);
	if (IS_ERR(clk_hw))
		return ERR_CAST(clk_hw);

	return clk_hw->clk;
}

static int
rzv2h_clk_ff_mod_status_is_enabled(struct clk_hw *hw)
{
	struct rzv2h_ff_mod_status_clk *fix = to_rzv2h_ff_mod_status_clk(hw);
	struct rzv2h_cpg_priv *priv = fix->priv;
	u32 offset = GET_CLK_MON_OFFSET(fix->conf.mon_index);
	u32 bitmask = BIT(fix->conf.mon_bit);
	u32 val;

	val = readl(priv->base + offset);
	return !!(val & bitmask);
}

static struct clk * __init
rzv2h_cpg_fixed_mod_status_clk_register(const struct cpg_core_clk *core,
					struct rzv2h_cpg_priv *priv)
{
	struct rzv2h_ff_mod_status_clk *clk_hw_data;
	struct clk_init_data init = { };
	struct clk_fixed_factor *fix;
	const struct clk *parent;
	const char *parent_name;
	int ret;

	WARN_DEBUG(core->parent >= priv->num_core_clks);
	parent = priv->clks[core->parent];
	if (IS_ERR(parent))
		return ERR_CAST(parent);

	parent_name = __clk_get_name(parent);
	parent = priv->clks[core->parent];
	if (IS_ERR(parent))
		return ERR_CAST(parent);

	clk_hw_data = devm_kzalloc(priv->dev, sizeof(*clk_hw_data), GFP_KERNEL);
	if (!clk_hw_data)
		return ERR_PTR(-ENOMEM);

	clk_hw_data->priv = priv;
	clk_hw_data->conf = core->cfg.fixed_mod;

	init.name = core->name;
	init.ops = priv->ff_mod_status_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	fix = &clk_hw_data->fix;
	fix->hw.init = &init;
	fix->mult = core->mult;
	fix->div = core->div;

	ret = devm_clk_hw_register(priv->dev, &clk_hw_data->fix.hw);
	if (ret)
		return ERR_PTR(ret);

	return clk_hw_data->fix.hw.clk;
}

static struct clk
*rzv2h_cpg_clk_src_twocell_get(struct of_phandle_args *clkspec,
			       void *data)
{
	unsigned int clkidx = clkspec->args[1];
	struct rzv2h_cpg_priv *priv = data;
	struct device *dev = priv->dev;
	const char *type;
	struct clk *clk;

	switch (clkspec->args[0]) {
	case CPG_CORE:
		type = "core";
		if (clkidx > priv->last_dt_core_clk) {
			dev_err(dev, "Invalid %s clock index %u\n", type, clkidx);
			return ERR_PTR(-EINVAL);
		}
		clk = priv->clks[clkidx];
		break;

	case CPG_MOD:
		type = "module";
		if (clkidx >= priv->num_mod_clks) {
			dev_err(dev, "Invalid %s clock index %u\n", type, clkidx);
			return ERR_PTR(-EINVAL);
		}
		clk = priv->clks[priv->num_core_clks + clkidx];
		break;

	default:
		dev_err(dev, "Invalid CPG clock type %u\n", clkspec->args[0]);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR(clk))
		dev_err(dev, "Cannot get %s clock %u: %ld", type, clkidx,
			PTR_ERR(clk));
	else
		dev_dbg(dev, "clock (%u, %u) is %pC at %lu Hz\n",
			clkspec->args[0], clkspec->args[1], clk,
			clk_get_rate(clk));
	return clk;
}

static void __init
rzv2h_cpg_register_core_clk(const struct cpg_core_clk *core,
			    struct rzv2h_cpg_priv *priv)
{
	struct clk *clk = ERR_PTR(-EOPNOTSUPP), *parent;
	unsigned int id = core->id, div = core->div;
	struct device *dev = priv->dev;
	const char *parent_name;
	struct clk_hw *clk_hw;

	WARN_DEBUG(id >= priv->num_core_clks);
	WARN_DEBUG(PTR_ERR(priv->clks[id]) != -ENOENT);

	switch (core->type) {
	case CLK_TYPE_IN:
		clk = of_clk_get_by_name(priv->dev->of_node, core->name);
		break;
	case CLK_TYPE_FF:
		WARN_DEBUG(core->parent >= priv->num_core_clks);
		parent = priv->clks[core->parent];
		if (IS_ERR(parent)) {
			clk = parent;
			goto fail;
		}

		parent_name = __clk_get_name(parent);
		clk_hw = devm_clk_hw_register_fixed_factor(dev, core->name,
							   parent_name, CLK_SET_RATE_PARENT,
							   core->mult, div);
		if (IS_ERR(clk_hw))
			clk = ERR_CAST(clk_hw);
		else
			clk = clk_hw->clk;
		break;
	case CLK_TYPE_FF_MOD_STATUS:
		if (!priv->ff_mod_status_ops) {
			priv->ff_mod_status_ops =
				devm_kzalloc(dev, sizeof(*priv->ff_mod_status_ops), GFP_KERNEL);
			if (!priv->ff_mod_status_ops) {
				clk = ERR_PTR(-ENOMEM);
				goto fail;
			}
			memcpy(priv->ff_mod_status_ops, &clk_fixed_factor_ops,
			       sizeof(const struct clk_ops));
			priv->ff_mod_status_ops->is_enabled = rzv2h_clk_ff_mod_status_is_enabled;
		}
		clk = rzv2h_cpg_fixed_mod_status_clk_register(core, priv);
		break;
	case CLK_TYPE_PLL:
		clk = rzv2h_cpg_pll_clk_register(core, priv, &rzv2h_cpg_pll_ops);
		break;
	case CLK_TYPE_DDIV:
		clk = rzv2h_cpg_ddiv_clk_register(core, priv);
		break;
	case CLK_TYPE_SMUX:
		clk = rzv2h_cpg_mux_clk_register(core, priv);
		break;
	default:
		goto fail;
	}

	if (IS_ERR(clk))
		goto fail;

	dev_dbg(dev, "Core clock %pC at %lu Hz\n", clk, clk_get_rate(clk));
	priv->clks[id] = clk;
	return;

fail:
	dev_err(dev, "Failed to register core clock %s: %ld\n",
		core->name, PTR_ERR(clk));
}

static void rzv2h_mod_clock_mstop_enable(struct rzv2h_cpg_priv *priv,
					 u32 mstop_data)
{
	unsigned long mstop_mask = FIELD_GET(BUS_MSTOP_BITS_MASK, mstop_data);
	u16 mstop_index = FIELD_GET(BUS_MSTOP_IDX_MASK, mstop_data);
	atomic_t *mstop = &priv->mstop_count[mstop_index * 16];
	unsigned long flags;
	unsigned int i;
	u32 val = 0;

	spin_lock_irqsave(&priv->rmw_lock, flags);
	for_each_set_bit(i, &mstop_mask, 16) {
		if (!atomic_read(&mstop[i]))
			val |= BIT(i) << 16;
		atomic_inc(&mstop[i]);
	}
	if (val)
		writel(val, priv->base + CPG_BUS_MSTOP(mstop_index));
	spin_unlock_irqrestore(&priv->rmw_lock, flags);
}

static void rzv2h_mod_clock_mstop_disable(struct rzv2h_cpg_priv *priv,
					  u32 mstop_data)
{
	unsigned long mstop_mask = FIELD_GET(BUS_MSTOP_BITS_MASK, mstop_data);
	u16 mstop_index = FIELD_GET(BUS_MSTOP_IDX_MASK, mstop_data);
	atomic_t *mstop = &priv->mstop_count[mstop_index * 16];
	unsigned long flags;
	unsigned int i;
	u32 val = 0;

	spin_lock_irqsave(&priv->rmw_lock, flags);
	for_each_set_bit(i, &mstop_mask, 16) {
		if (!atomic_read(&mstop[i]) ||
		    atomic_dec_and_test(&mstop[i]))
			val |= BIT(i) << 16 | BIT(i);
	}
	if (val)
		writel(val, priv->base + CPG_BUS_MSTOP(mstop_index));
	spin_unlock_irqrestore(&priv->rmw_lock, flags);
}

static int rzv2h_parent_clk_mux_to_index(struct clk_hw *hw)
{
	struct clk_hw *parent_hw;
	struct clk *parent_clk;
	struct clk_mux *mux;
	u32 val;

	/* This will always succeed, so no need to check for IS_ERR() */
	parent_clk = clk_get_parent(hw->clk);

	parent_hw = __clk_get_hw(parent_clk);
	mux = to_clk_mux(parent_hw);

	val = readl(mux->reg) >> mux->shift;
	val &= mux->mask;
	return clk_mux_val_to_index(parent_hw, mux->table, 0, val);
}

static int rzv2h_mod_clock_is_enabled(struct clk_hw *hw)
{
	struct mod_clock *clock = to_mod_clock(hw);
	struct rzv2h_cpg_priv *priv = clock->priv;
	int mon_index = clock->mon_index;
	u32 bitmask;
	u32 offset;

	if (clock->ext_clk_mux_index >= 0 &&
	    rzv2h_parent_clk_mux_to_index(hw) == clock->ext_clk_mux_index)
		mon_index = -1;

	if (mon_index >= 0) {
		offset = GET_CLK_MON_OFFSET(mon_index);
		bitmask = BIT(clock->mon_bit);

		if (!(readl(priv->base + offset) & bitmask))
			return 0;
	}

	offset = GET_CLK_ON_OFFSET(clock->on_index);
	bitmask = BIT(clock->on_bit);

	return readl(priv->base + offset) & bitmask;
}

static int rzv2h_mod_clock_endisable(struct clk_hw *hw, bool enable)
{
	bool enabled = rzv2h_mod_clock_is_enabled(hw);
	struct mod_clock *clock = to_mod_clock(hw);
	unsigned int reg = GET_CLK_ON_OFFSET(clock->on_index);
	struct rzv2h_cpg_priv *priv = clock->priv;
	u32 bitmask = BIT(clock->on_bit);
	struct device *dev = priv->dev;
	u32 value;
	int error;

	dev_dbg(dev, "CLK_ON 0x%x/%pC %s\n", reg, hw->clk,
		str_on_off(enable));

	if (enabled == enable)
		return 0;

	value = bitmask << 16;
	if (enable) {
		value |= bitmask;
		writel(value, priv->base + reg);
		if (clock->mstop_data != BUS_MSTOP_NONE)
			rzv2h_mod_clock_mstop_enable(priv, clock->mstop_data);
	} else {
		if (clock->mstop_data != BUS_MSTOP_NONE)
			rzv2h_mod_clock_mstop_disable(priv, clock->mstop_data);
		writel(value, priv->base + reg);
	}

	if (!enable || clock->mon_index < 0)
		return 0;

	reg = GET_CLK_MON_OFFSET(clock->mon_index);
	bitmask = BIT(clock->mon_bit);
	error = readl_poll_timeout_atomic(priv->base + reg, value,
					  value & bitmask, 0, 10);
	if (error)
		dev_err(dev, "Failed to enable CLK_ON 0x%x/%pC\n",
			GET_CLK_ON_OFFSET(clock->on_index), hw->clk);

	return error;
}

static int rzv2h_mod_clock_enable(struct clk_hw *hw)
{
	return rzv2h_mod_clock_endisable(hw, true);
}

static void rzv2h_mod_clock_disable(struct clk_hw *hw)
{
	rzv2h_mod_clock_endisable(hw, false);
}

static const struct clk_ops rzv2h_mod_clock_ops = {
	.enable = rzv2h_mod_clock_enable,
	.disable = rzv2h_mod_clock_disable,
	.is_enabled = rzv2h_mod_clock_is_enabled,
};

static void __init
rzv2h_cpg_register_mod_clk(const struct rzv2h_mod_clk *mod,
			   struct rzv2h_cpg_priv *priv)
{
	struct mod_clock *clock = NULL;
	struct device *dev = priv->dev;
	struct clk_init_data init;
	struct clk *parent, *clk;
	const char *parent_name;
	unsigned int id;
	int ret;

	id = GET_MOD_CLK_ID(priv->num_core_clks, mod->on_index, mod->on_bit);
	WARN_DEBUG(id >= priv->num_core_clks + priv->num_mod_clks);
	WARN_DEBUG(mod->parent >= priv->num_core_clks + priv->num_mod_clks);
	WARN_DEBUG(PTR_ERR(priv->clks[id]) != -ENOENT);

	parent = priv->clks[mod->parent];
	if (IS_ERR(parent)) {
		clk = parent;
		goto fail;
	}

	clock = devm_kzalloc(dev, sizeof(*clock), GFP_KERNEL);
	if (!clock) {
		clk = ERR_PTR(-ENOMEM);
		goto fail;
	}

	init.name = mod->name;
	init.ops = &rzv2h_mod_clock_ops;
	init.flags = CLK_SET_RATE_PARENT;
	if (mod->critical)
		init.flags |= CLK_IS_CRITICAL;

	parent_name = __clk_get_name(parent);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clock->on_index = mod->on_index;
	clock->on_bit = mod->on_bit;
	clock->mon_index = mod->mon_index;
	clock->mon_bit = mod->mon_bit;
	clock->no_pm = mod->no_pm;
	clock->ext_clk_mux_index = mod->ext_clk_mux_index;
	clock->priv = priv;
	clock->hw.init = &init;
	clock->mstop_data = mod->mstop_data;

	ret = devm_clk_hw_register(dev, &clock->hw);
	if (ret) {
		clk = ERR_PTR(ret);
		goto fail;
	}

	priv->clks[id] = clock->hw.clk;

	/*
	 * Ensure the module clocks and MSTOP bits are synchronized when they are
	 * turned ON by the bootloader. Enable MSTOP bits for module clocks that were
	 * turned ON in an earlier boot stage.
	 */
	if (clock->mstop_data != BUS_MSTOP_NONE &&
	    !mod->critical && rzv2h_mod_clock_is_enabled(&clock->hw)) {
		rzv2h_mod_clock_mstop_enable(priv, clock->mstop_data);
	} else if (clock->mstop_data != BUS_MSTOP_NONE && mod->critical) {
		unsigned long mstop_mask = FIELD_GET(BUS_MSTOP_BITS_MASK, clock->mstop_data);
		u16 mstop_index = FIELD_GET(BUS_MSTOP_IDX_MASK, clock->mstop_data);
		atomic_t *mstop = &priv->mstop_count[mstop_index * 16];
		unsigned long flags;
		unsigned int i;
		u32 val = 0;

		/*
		 * Critical clocks are turned ON immediately upon registration, and the
		 * MSTOP counter is updated through the rzv2h_mod_clock_enable() path.
		 * However, if the critical clocks were already turned ON by the initial
		 * bootloader, synchronize the atomic counter here and clear the MSTOP bit.
		 */
		spin_lock_irqsave(&priv->rmw_lock, flags);
		for_each_set_bit(i, &mstop_mask, 16) {
			if (atomic_read(&mstop[i]))
				continue;
			val |= BIT(i) << 16;
			atomic_inc(&mstop[i]);
		}
		if (val)
			writel(val, priv->base + CPG_BUS_MSTOP(mstop_index));
		spin_unlock_irqrestore(&priv->rmw_lock, flags);
	}

	return;

fail:
	dev_err(dev, "Failed to register module clock %s: %ld\n",
		mod->name, PTR_ERR(clk));
}

static int __rzv2h_cpg_assert(struct reset_controller_dev *rcdev,
			      unsigned long id, bool assert)
{
	struct rzv2h_cpg_priv *priv = rcdev_to_priv(rcdev);
	unsigned int reg = GET_RST_OFFSET(priv->resets[id].reset_index);
	u32 mask = BIT(priv->resets[id].reset_bit);
	u8 monbit = priv->resets[id].mon_bit;
	u32 value = mask << 16;
	int ret;

	dev_dbg(rcdev->dev, "%s id:%ld offset:0x%x\n",
		assert ? "assert" : "deassert", id, reg);

	if (!assert)
		value |= mask;
	writel(value, priv->base + reg);

	reg = GET_RST_MON_OFFSET(priv->resets[id].mon_index);
	mask = BIT(monbit);

	ret = readl_poll_timeout_atomic(priv->base + reg, value,
					assert == !!(value & mask), 10, 200);
	if (ret && !assert) {
		value = mask << 16;
		writel(value, priv->base + GET_RST_OFFSET(priv->resets[id].reset_index));
	}

	return ret;
}

static int rzv2h_cpg_assert(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	return __rzv2h_cpg_assert(rcdev, id, true);
}

static int rzv2h_cpg_deassert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	return __rzv2h_cpg_assert(rcdev, id, false);
}

static int rzv2h_cpg_reset(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	int ret;

	ret = rzv2h_cpg_assert(rcdev, id);
	if (ret)
		return ret;

	return rzv2h_cpg_deassert(rcdev, id);
}

static int rzv2h_cpg_status(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct rzv2h_cpg_priv *priv = rcdev_to_priv(rcdev);
	unsigned int reg = GET_RST_MON_OFFSET(priv->resets[id].mon_index);
	u8 monbit = priv->resets[id].mon_bit;

	return !!(readl(priv->base + reg) & BIT(monbit));
}

static const struct reset_control_ops rzv2h_cpg_reset_ops = {
	.reset = rzv2h_cpg_reset,
	.assert = rzv2h_cpg_assert,
	.deassert = rzv2h_cpg_deassert,
	.status = rzv2h_cpg_status,
};

static int rzv2h_cpg_reset_xlate(struct reset_controller_dev *rcdev,
				 const struct of_phandle_args *reset_spec)
{
	struct rzv2h_cpg_priv *priv = rcdev_to_priv(rcdev);
	unsigned int id = reset_spec->args[0];
	u8 rst_index = id / 16;
	u8 rst_bit = id % 16;
	unsigned int i;

	for (i = 0; i < rcdev->nr_resets; i++) {
		if (rst_index == priv->resets[i].reset_index &&
		    rst_bit == priv->resets[i].reset_bit)
			return i;
	}

	return -EINVAL;
}

static int rzv2h_cpg_reset_controller_register(struct rzv2h_cpg_priv *priv)
{
	priv->rcdev.ops = &rzv2h_cpg_reset_ops;
	priv->rcdev.of_node = priv->dev->of_node;
	priv->rcdev.dev = priv->dev;
	priv->rcdev.of_reset_n_cells = 1;
	priv->rcdev.of_xlate = rzv2h_cpg_reset_xlate;
	priv->rcdev.nr_resets = priv->num_resets;

	return devm_reset_controller_register(priv->dev, &priv->rcdev);
}

/**
 * struct rzv2h_cpg_pd - RZ/V2H power domain data structure
 * @priv: pointer to CPG private data structure
 * @genpd: generic PM domain
 */
struct rzv2h_cpg_pd {
	struct rzv2h_cpg_priv *priv;
	struct generic_pm_domain genpd;
};

static bool rzv2h_cpg_is_pm_clk(struct rzv2h_cpg_pd *pd,
				const struct of_phandle_args *clkspec)
{
	if (clkspec->np != pd->genpd.dev.of_node || clkspec->args_count != 2)
		return false;

	switch (clkspec->args[0]) {
	case CPG_MOD: {
		struct rzv2h_cpg_priv *priv = pd->priv;
		unsigned int id = clkspec->args[1];
		struct mod_clock *clock;

		if (id >= priv->num_mod_clks)
			return false;

		if (priv->clks[priv->num_core_clks + id] == ERR_PTR(-ENOENT))
			return false;

		clock = to_mod_clock(__clk_get_hw(priv->clks[priv->num_core_clks + id]));

		return !clock->no_pm;
	}

	case CPG_CORE:
	default:
		return false;
	}
}

static int rzv2h_cpg_attach_dev(struct generic_pm_domain *domain, struct device *dev)
{
	struct rzv2h_cpg_pd *pd = container_of(domain, struct rzv2h_cpg_pd, genpd);
	struct device_node *np = dev->of_node;
	struct of_phandle_args clkspec;
	bool once = true;
	struct clk *clk;
	unsigned int i;
	int error;

	for (i = 0; !of_parse_phandle_with_args(np, "clocks", "#clock-cells", i, &clkspec); i++) {
		if (!rzv2h_cpg_is_pm_clk(pd, &clkspec)) {
			of_node_put(clkspec.np);
			continue;
		}

		if (once) {
			once = false;
			error = pm_clk_create(dev);
			if (error) {
				of_node_put(clkspec.np);
				goto err;
			}
		}
		clk = of_clk_get_from_provider(&clkspec);
		of_node_put(clkspec.np);
		if (IS_ERR(clk)) {
			error = PTR_ERR(clk);
			goto fail_destroy;
		}

		error = pm_clk_add_clk(dev, clk);
		if (error) {
			dev_err(dev, "pm_clk_add_clk failed %d\n",
				error);
			goto fail_put;
		}
	}

	return 0;

fail_put:
	clk_put(clk);

fail_destroy:
	pm_clk_destroy(dev);
err:
	return error;
}

static void rzv2h_cpg_detach_dev(struct generic_pm_domain *unused, struct device *dev)
{
	if (!pm_clk_no_clocks(dev))
		pm_clk_destroy(dev);
}

static void rzv2h_cpg_genpd_remove_simple(void *data)
{
	pm_genpd_remove(data);
}

static int __init rzv2h_cpg_add_pm_domains(struct rzv2h_cpg_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct rzv2h_cpg_pd *pd;
	int ret;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->genpd.name = np->name;
	pd->priv = priv;
	pd->genpd.flags |= GENPD_FLAG_ALWAYS_ON | GENPD_FLAG_PM_CLK | GENPD_FLAG_ACTIVE_WAKEUP;
	pd->genpd.attach_dev = rzv2h_cpg_attach_dev;
	pd->genpd.detach_dev = rzv2h_cpg_detach_dev;
	ret = pm_genpd_init(&pd->genpd, &pm_domain_always_on_gov, false);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, rzv2h_cpg_genpd_remove_simple, &pd->genpd);
	if (ret)
		return ret;

	return of_genpd_add_provider_simple(np, &pd->genpd);
}

static void rzv2h_cpg_del_clk_provider(void *data)
{
	of_clk_del_provider(data);
}

static int __init rzv2h_cpg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct rzv2h_cpg_info *info;
	struct rzv2h_cpg_priv *priv;
	unsigned int nclks, i;
	struct clk **clks;
	int error;

	info = of_device_get_match_data(dev);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->rmw_lock);

	priv->dev = dev;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	nclks = info->num_total_core_clks + info->num_hw_mod_clks;
	clks = devm_kmalloc_array(dev, nclks, sizeof(*clks), GFP_KERNEL);
	if (!clks)
		return -ENOMEM;

	priv->mstop_count = devm_kcalloc(dev, info->num_mstop_bits,
					 sizeof(*priv->mstop_count), GFP_KERNEL);
	if (!priv->mstop_count)
		return -ENOMEM;

	/* Adjust for CPG_BUS_m_MSTOP starting from m = 1 */
	priv->mstop_count -= 16;

	priv->resets = devm_kmemdup_array(dev, info->resets, info->num_resets,
					  sizeof(*info->resets), GFP_KERNEL);
	if (!priv->resets)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->clks = clks;
	priv->num_core_clks = info->num_total_core_clks;
	priv->num_mod_clks = info->num_hw_mod_clks;
	priv->last_dt_core_clk = info->last_dt_core_clk;
	priv->num_resets = info->num_resets;

	for (i = 0; i < nclks; i++)
		clks[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < info->num_core_clks; i++)
		rzv2h_cpg_register_core_clk(&info->core_clks[i], priv);

	for (i = 0; i < info->num_mod_clks; i++)
		rzv2h_cpg_register_mod_clk(&info->mod_clks[i], priv);

	error = of_clk_add_provider(np, rzv2h_cpg_clk_src_twocell_get, priv);
	if (error)
		return error;

	error = devm_add_action_or_reset(dev, rzv2h_cpg_del_clk_provider, np);
	if (error)
		return error;

	error = rzv2h_cpg_add_pm_domains(priv);
	if (error)
		return error;

	error = rzv2h_cpg_reset_controller_register(priv);
	if (error)
		return error;

	return 0;
}

static const struct of_device_id rzv2h_cpg_match[] = {
#ifdef CONFIG_CLK_R9A09G047
	{
		.compatible = "renesas,r9a09g047-cpg",
		.data = &r9a09g047_cpg_info,
	},
#endif
#ifdef CONFIG_CLK_R9A09G056
	{
		.compatible = "renesas,r9a09g056-cpg",
		.data = &r9a09g056_cpg_info,
	},
#endif
#ifdef CONFIG_CLK_R9A09G057
	{
		.compatible = "renesas,r9a09g057-cpg",
		.data = &r9a09g057_cpg_info,
	},
#endif
	{ /* sentinel */ }
};

static struct platform_driver rzv2h_cpg_driver = {
	.driver		= {
		.name	= "rzv2h-cpg",
		.of_match_table = rzv2h_cpg_match,
	},
};

static int __init rzv2h_cpg_init(void)
{
	return platform_driver_probe(&rzv2h_cpg_driver, rzv2h_cpg_probe);
}

subsys_initcall(rzv2h_cpg_init);

MODULE_DESCRIPTION("Renesas RZ/V2H CPG Driver");
