/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <soc/imx8/sc/sci.h>

#include "clk-imx8.h"

/*
 * DOC: basic adjustable multiplexer clock that cannot gate
 *
 * Traits of this clock:
 * prepare - clk_prepare only ensures that parents are prepared
 * enable - clk_enable only ensures that parents are enabled
 * rate - rate is only affected by parent switching.  No clk_set_rate support
 * parent - parent is adjustable through clk_set_parent
 */

struct clk_mux_scu {
	struct clk_hw	hw;
	void __iomem	*reg;
	u32		*table;
	u32		mask;
	u8		shift;
	u8		flags;
	u32		val;
	bool		update;
	spinlock_t	*lock;
	char *pd_name;
	struct generic_pm_domain *pd;
};

struct clk_mux_gpr_scu {
	struct clk_hw hw;
	sc_rsrc_t	rsrc_id;
	sc_ctrl_t	gpr_id;
};

struct clk_mux2_scu {
	struct clk_hw	hw;
	sc_rsrc_t	rsrc_id;
	sc_pm_clk_t	clk_type;
};

#define to_clk_mux_scu(_hw) container_of(_hw, struct clk_mux_scu, hw)
#define to_clk_mux_gpr_scu(_hw) container_of(_hw, struct clk_mux_gpr_scu, hw)
#define to_clk_mux2_scu(_hw) container_of(_hw, struct clk_mux2_scu, hw)

/* Get the power domain associated with the clock from the device tree. */
static void populate_mux_pd(struct clk_mux_scu *clk)
{
	struct device_node *np;
	struct of_phandle_args pd_args;

	np = of_find_node_by_name(NULL, clk->pd_name);
	if (np) {
		pd_args.np = np;
		pd_args.args_count = 0;
		clk->pd = genpd_get_from_provider(&pd_args);
		if (IS_ERR(clk->pd))
			pr_warn("%s: failed to get pd\n", __func__);
	}
}

static int check_mux_pd(struct clk_mux_scu *mux)
{
	if (!ccm_ipc_handle)
		return -1;

	if (mux->pd == NULL && mux->pd_name)
		populate_mux_pd(mux);

	if (IS_ERR_OR_NULL(mux->pd))
		return -1;

	if (mux->pd->status != GPD_STATE_ACTIVE)
		return -1;

	return 0;
}

static u8 clk_mux_get_parent_scu(struct clk_hw *hw)
{
	struct clk_mux_scu *mux = to_clk_mux_scu(hw);
	int num_parents = clk_hw_get_num_parents(hw);
	u32 val;

	/*
	 * FIXME need a mux-specific flag to determine if val is bitwise or numeric
	 * e.g. sys_clkin_ck's clksel field is 3 bits wide, but ranges from 0x1
	 * to 0x7 (index starts at one)
	 * OTOH, pmd_trace_clk_mux_ck uses a separate bit for each clock, so
	 * val = 0x4 really means "bit 2, index starts at bit 0"
	 */
	val = mux->val >> mux->shift;
	val &= mux->mask;

	if (mux->table) {
		int i;

		for (i = 0; i < num_parents; i++)
			if (mux->table[i] == val)
				return i;
		return -EINVAL;
	}

	if (val && (mux->flags & CLK_MUX_INDEX_BIT))
		val = ffs(val) - 1;

	if (val && (mux->flags & CLK_MUX_INDEX_ONE))
		val--;

	if (val >= num_parents)
		return -EINVAL;

	return val;
}

static int clk_mux_prepare_scu(struct clk_hw *hw)
{
	struct clk_mux_scu *mux = to_clk_mux_scu(hw);
	unsigned long flags = 0;
	int ret;

	ret = check_mux_pd(mux);
	if (ret)
		return ret;

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);

	if (mux->update) {
		clk_writel(mux->val, mux->reg);
		mux->update = 0;
	}

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);

	return 0;
}

static int clk_mux_set_parent_scu(struct clk_hw *hw, u8 index)
{
	struct clk_mux_scu *mux = to_clk_mux_scu(hw);
	unsigned long flags = 0;
	int ret;

	ret = check_mux_pd(mux);

	if (mux->table) {
		index = mux->table[index];
	} else {
		if (mux->flags & CLK_MUX_INDEX_BIT)
			index = 1 << index;
		if (mux->flags & CLK_MUX_INDEX_ONE)
			index++;
	}

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);

	if (mux->flags & CLK_MUX_HIWORD_MASK) {
		mux->val = mux->mask << (mux->shift + 16);
	} else {
		mux->val &= ~(mux->mask << mux->shift);
	}
	mux->val |= index << mux->shift;
	mux->update = (ret != 0);

	if (ret == 0)
		clk_writel(mux->val, mux->reg);

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);

	return 0;
}

const struct clk_ops clk_mux_scu_ops = {
	.prepare = clk_mux_prepare_scu,
	.get_parent = clk_mux_get_parent_scu,
	.set_parent = clk_mux_set_parent_scu,
	.determine_rate = __clk_mux_determine_rate,
};

const struct clk_ops clk_mux_ro_scu_ops = {
	.get_parent = clk_mux_get_parent_scu,
};

struct clk *clk_register_mux_table_scu(struct device *dev, const char *name,
		const char **parent_names, u8 num_parents, unsigned long flags,
		void __iomem *reg, u8 shift, u32 mask,
		u8 clk_mux_flags, u32 *table, spinlock_t *lock,
		const char *pd_name)
{
	struct clk_mux_scu *mux;
	struct clk *clk;
	struct clk_init_data init;
	u8 width = 0;

	if (clk_mux_flags & CLK_MUX_HIWORD_MASK) {
		width = fls(mask) - ffs(mask) + 1;
		if (width + shift > 16) {
			pr_err("mux value exceeds LOWORD field\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* allocate the mux */
	mux = kzalloc(sizeof(struct clk_mux_scu), GFP_KERNEL);
	if (!mux) {
		pr_err("%s: could not allocate mux clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	if (clk_mux_flags & CLK_MUX_READ_ONLY)
		init.ops = &clk_mux_ro_scu_ops;
	else
		init.ops = &clk_mux_scu_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	/* struct clk_mux_scu assignments */
	mux->reg = reg;
	mux->shift = shift;
	mux->mask = mask;
	mux->flags = clk_mux_flags;
	mux->lock = lock;
	mux->table = table;
	mux->hw.init = &init;
	mux->pd_name = NULL;
	if (pd_name) {
		mux->pd_name = kzalloc(strlen(pd_name) + 1, GFP_KERNEL);
		strcpy(mux->pd_name, pd_name);
	}

	clk = clk_register(dev, &mux->hw);

	if (IS_ERR(clk)) {
		kfree(mux->pd_name);
		kfree(mux);
	}

	return clk;
}

struct clk *clk_register_mux_scu(struct device *dev, const char *name,
		const char **parent_names, u8 num_parents, unsigned long flags,
		void __iomem *reg, u8 shift, u8 width,
		u8 clk_mux_flags, spinlock_t *lock,
		const char *pd_name)
{
	u32 mask = BIT(width) - 1;

	return clk_register_mux_table_scu(dev, name, parent_names, num_parents,
				      flags, reg, shift, mask, clk_mux_flags,
				      NULL, lock, pd_name);
}

void clk_unregister_mux_scu(struct clk *clk)
{
	struct clk_mux_scu *mux;
	struct clk_hw *hw;

	hw = __clk_get_hw(clk);
	if (!hw)
		return;

	mux = to_clk_mux_scu(hw);

	clk_unregister(clk);
	kfree(mux);
}

static u8 clk_mux_gpr_scu_get_parent(struct clk_hw *hw)
{
	struct clk_mux_gpr_scu *gpr_mux = to_clk_mux_gpr_scu(hw);
	u32 val = 0;

	if (!ccm_ipc_handle)
		return 0;

	sc_misc_get_control(ccm_ipc_handle,
		gpr_mux->rsrc_id, gpr_mux->gpr_id, &val);

	return (u8)val;
}

static int clk_mux_gpr_scu_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_mux_gpr_scu *gpr_mux = to_clk_mux_gpr_scu(hw);

	if (!ccm_ipc_handle)
		return -1;

	sc_misc_set_control(ccm_ipc_handle,
		gpr_mux->rsrc_id, gpr_mux->gpr_id, index);

	return 0;
}

static const struct clk_ops clk_mux_gpr_scu_ops = {
	.get_parent = clk_mux_gpr_scu_get_parent,
	.set_parent = clk_mux_gpr_scu_set_parent,
};

struct clk *clk_register_mux_gpr_scu(struct device *dev, const char *name,
		const char **parents, int num_parents, spinlock_t *lock,
		sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id)
{
	struct clk_mux_gpr_scu *gpr_scu_mux;
	struct clk *clk;
	struct clk_init_data init;

	if (rsrc_id >= SC_R_LAST)
		return NULL;

	if (gpr_id >= SC_C_LAST)
		return NULL;

	gpr_scu_mux = kzalloc(sizeof(struct clk_mux_gpr_scu), GFP_KERNEL);
	if (!gpr_scu_mux)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_mux_gpr_scu_ops;
	init.parent_names = parents;
	init.num_parents = num_parents;
	init.flags = 0;

	gpr_scu_mux->hw.init = &init;
	gpr_scu_mux->rsrc_id = rsrc_id;
	gpr_scu_mux->gpr_id = gpr_id;

	clk = clk_register(NULL, &gpr_scu_mux->hw);
	if (IS_ERR(clk))
		kfree(gpr_scu_mux);

	return clk;
}

static u8 clk_mux2_scu_get_parent(struct clk_hw *hw)
{
	struct clk_mux2_scu *mux = to_clk_mux2_scu(hw);
	sc_pm_clk_parent_t parent;
	sc_err_t ret;

	if (!ccm_ipc_handle)
		return -EBUSY;

	ret = sc_pm_get_clock_parent(ccm_ipc_handle, mux->rsrc_id,
				     mux->clk_type, &parent);
	if (ret != SC_ERR_NONE)
		return -EINVAL;

	return (u8)parent;
}

static int clk_mux2_scu_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_mux2_scu *mux = to_clk_mux2_scu(hw);
	sc_err_t ret;

	if (!ccm_ipc_handle)
		return -EBUSY;

	ret = sc_pm_set_clock_parent(ccm_ipc_handle, mux->rsrc_id,
				     mux->clk_type, index);
	if (ret != SC_ERR_NONE)
		return -EINVAL;

	return 0;
}

static const struct clk_ops clk_mux2_scu_ops = {
	.get_parent = clk_mux2_scu_get_parent,
	.set_parent = clk_mux2_scu_set_parent,
};


struct clk *clk_register_mux2_scu(struct device *dev, const char *name,
				  const char **parents, int num_parents,
				  unsigned long flags, sc_rsrc_t rsrc_id,
				  sc_pm_clk_t clk_type)
{
	struct clk_mux2_scu *mux;
	struct clk *clk;
	struct clk_init_data init;

	if (rsrc_id >= SC_R_LAST)
		return ERR_PTR(-EINVAL);

	mux = kzalloc(sizeof(struct clk_mux2_scu), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_mux2_scu_ops;
	init.parent_names = parents;
	init.num_parents = num_parents;
	init.flags = flags;

	mux->hw.init = &init;
	mux->rsrc_id = rsrc_id;
	mux->clk_type = clk_type;

	clk = clk_register(NULL, &mux->hw);
	if (IS_ERR(clk))
		kfree(mux);

	return clk;
}
