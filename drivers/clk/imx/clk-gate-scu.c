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
 * DOC: basic gatable clock which can gate and ungate it's ouput
 *
 * Traits of this clock:
 * prepare - clk_(un)prepare only ensures parent is (un)prepared
 * enable - clk_enable and clk_disable are functional & control gating
 * rate - inherits rate from parent.  No clk_set_rate support
 * parent - fixed parent.  No clk_set_parent support
 */

struct clk_gate_scu {
	struct clk_hw hw;
	void __iomem	*reg;
	u8		bit_idx;
	bool	hw_gate;
	u8		flags;
	spinlock_t	*lock;
	sc_rsrc_t	rsrc_id;
	sc_pm_clk_t	clk_type;
};

struct clk_gate2_scu {
	struct clk_hw hw;
	void __iomem	*reg;
	u8		bit_idx;
	u8		flags;
	spinlock_t	*lock;
	char *pd_name;
	struct generic_pm_domain *pd;
};

struct clk_gate3_scu {
	struct clk_hw hw;
	spinlock_t	*lock;
	sc_rsrc_t	rsrc_id;
	sc_ctrl_t	gpr_id;
	bool	invert;
};

#define to_clk_gate_scu(_hw) container_of(_hw, struct clk_gate_scu, hw)
#define to_clk_gate2_scu(_hw) container_of(_hw, struct clk_gate2_scu, hw)
#define to_clk_gate3_scu(_hw) container_of(_hw, struct clk_gate3_scu, hw)

/* Write to the LPCG bits. */
static int clk_gate_scu_enable(struct clk_hw *hw)
{
	struct clk_gate_scu *gate = to_clk_gate_scu(hw);
	u32 reg;

	if (!ccm_ipc_handle)
		return -1;

	if (gate->reg) {
		reg = readl(gate->reg);
		if (gate->hw_gate)
			reg |= (0x3 << gate->bit_idx);
		else
			reg |= (0x2 << gate->bit_idx);
		writel(reg, gate->reg);
	}

	return 0;
}

/* Write to the LPCG bits. */
static void clk_gate_scu_disable(struct clk_hw *hw)
{
	struct clk_gate_scu *gate = to_clk_gate_scu(hw);
	u32 reg;

	if (!ccm_ipc_handle)
		return;

	if (gate->reg) {
		reg = readl(gate->reg);
		if (gate->hw_gate)
			reg &= ~(0x3 << gate->bit_idx);
		else
			reg &= ~(0x2 << gate->bit_idx);
		writel(reg, gate->reg);
	}
}

static int clk_gate_scu_prepare(struct clk_hw *hw)
{
	struct clk_gate_scu *gate = to_clk_gate_scu(hw);
	sc_err_t sci_err = SC_ERR_NONE;

	if (!ccm_ipc_handle)
		return -1;

	/* Enable the clock at the DSC slice level */
	sci_err = sc_pm_clock_enable(ccm_ipc_handle, gate->rsrc_id,
		gate->clk_type, true, gate->hw_gate);

	return sci_err;
}

static void clk_gate_scu_unprepare(struct clk_hw *hw)
{
	struct clk_gate_scu *gate = to_clk_gate_scu(hw);
	sc_err_t sci_err;

	if (!ccm_ipc_handle)
		return;

	sci_err = sc_pm_clock_enable(ccm_ipc_handle, gate->rsrc_id,
		gate->clk_type, false, false);
	if (sci_err)
		pr_err("clk gate scu unprepare clk fail!\n");
}

static struct clk_ops clk_gate_scu_ops = {
	.prepare = clk_gate_scu_prepare,
	.unprepare = clk_gate_scu_unprepare,
	.enable = clk_gate_scu_enable,
	.disable = clk_gate_scu_disable,
};

struct clk *clk_register_gate_scu(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		u8 clk_gate_scu_flags, spinlock_t *lock,
		sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type,
		void __iomem *reg, u8 bit_idx, bool hw_gate)
{
	struct clk_gate_scu *gate;
	struct clk *clk;
	struct clk_init_data init;

	gate = kzalloc(sizeof(struct clk_gate_scu), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	/* struct clk_gate_scu assignments */
	gate->flags = clk_gate_scu_flags;
	gate->lock = lock;
	gate->rsrc_id = rsrc_id;
	gate->clk_type = clk_type;
	if (reg != NULL)
		gate->reg = ioremap((phys_addr_t)reg, SZ_64K);
	else
		gate->reg = NULL;
	gate->bit_idx = bit_idx;
	gate->hw_gate = hw_gate;

	init.name = name;
	init.ops = &clk_gate_scu_ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);
	if (IS_ERR(clk)) {
		iounmap(gate->reg);
		kfree(gate);
	}

	return clk;
}

/* Get the power domain associated with the clock from the device tree. */
static void populate_gate_pd(struct clk_gate2_scu *clk)
{
	struct device_node *np;
	struct of_phandle_args pd_args;

	np = of_find_node_by_name(NULL, clk->pd_name);
	if (np) {
		pd_args.np = np;
		pd_args.args_count = 0;
		clk->pd = genpd_get_from_provider(&pd_args);
	}
}

/* Write to the LPCG bits. */
static int clk_gate2_scu_enable(struct clk_hw *hw)
{
	struct clk_gate2_scu *gate = to_clk_gate2_scu(hw);
	u32 reg;

	if (!ccm_ipc_handle)
		return -1;

	if (gate->pd == NULL && gate->pd_name)
		populate_gate_pd(gate);

	if (!gate->pd)
		return -1;

	if (gate->pd->status != GPD_STATE_ACTIVE)
		return -1;

	if (gate->reg) {
		reg = readl(gate->reg);
		reg |= (0x2 << gate->bit_idx);
		writel(reg, gate->reg);
	}

	return 0;
}

/* Write to the LPCG bits. */
static void clk_gate2_scu_disable(struct clk_hw *hw)
{
	struct clk_gate2_scu *gate = to_clk_gate2_scu(hw);
	u32 reg;

	if (!ccm_ipc_handle)
		return;

	if (gate->pd == NULL && gate->pd_name)
		populate_gate_pd(gate);

	if (!gate->pd)
		return;

	if (gate->pd->status != GPD_STATE_ACTIVE)
		return;

	if (gate->reg) {
		reg = readl(gate->reg);
		reg &= ~(0x2 << gate->bit_idx);
		writel(reg, gate->reg);
	}
}

static int clk_gate2_scu_is_enabled(struct clk_hw *hw)
{
	struct clk_gate2_scu *gate = to_clk_gate2_scu(hw);
	u32 val;

	if (gate->pd == NULL && gate->pd_name)
		populate_gate_pd(gate);

	if (!gate->pd)
		return 0;

	if (gate->pd->status != GPD_STATE_ACTIVE)
		return 0;

	val = readl(gate->reg);

	if (((val >> gate->bit_idx) & 2) == 2)
		return 1;

	return 0;
}


static struct clk_ops clk_gate2_scu_ops = {
	.enable = clk_gate2_scu_enable,
	.disable = clk_gate2_scu_disable,
	.is_enabled = clk_gate2_scu_is_enabled,
};

struct clk *clk_register_gate2_scu(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 bit_idx,
		u8 clk_gate_flags, spinlock_t *lock, const char *pd_name)
{
	struct clk_gate2_scu *gate;
	struct clk *clk;
	struct clk_init_data init;

	gate = kzalloc(sizeof(struct clk_gate2_scu), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	/* struct clk_gate_scu assignments */
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	if (reg != NULL)
		gate->reg = ioremap((phys_addr_t)reg, SZ_64K);
	else
		gate->reg = NULL;
	gate->bit_idx = bit_idx;
	gate->pd = NULL;
	gate->pd_name = NULL;
	if (pd_name) {
		gate->pd_name = kzalloc(strlen(pd_name) + 1, GFP_KERNEL);
		strcpy(gate->pd_name, pd_name);
	}

	init.name = name;
	init.ops = &clk_gate2_scu_ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);
	if (IS_ERR(clk)) {
		iounmap(gate->reg);
		kfree(gate->pd_name);
		kfree(gate);
	}

	return clk;
}

static int clk_gate3_scu_enable(struct clk_hw *hw)
{
	struct clk_gate3_scu *gate = to_clk_gate3_scu(hw);
	uint32_t val;

	if (!ccm_ipc_handle)
		return -1;

	val = (gate->invert) ? 0 : 1;

	return sc_misc_set_control(ccm_ipc_handle,
		gate->rsrc_id, gate->gpr_id, val);
}

/* Write to the LPCG bits. */
static void clk_gate3_scu_disable(struct clk_hw *hw)
{
	struct clk_gate3_scu *gate = to_clk_gate3_scu(hw);
	uint32_t val;

	if (!ccm_ipc_handle)
		return;

	val = (gate->invert) ? 1 : 0;
	sc_misc_set_control(ccm_ipc_handle, gate->rsrc_id, gate->gpr_id, val);
}

static struct clk_ops clk_gate3_scu_ops = {
	.enable = clk_gate3_scu_enable,
	.disable = clk_gate3_scu_disable,
};

struct clk *clk_register_gate3_scu(struct device *dev, const char *name,
		const char *parent_name, spinlock_t *lock,
		sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id, bool invert_flag)
{
	struct clk_gate3_scu *gate;
	struct clk *clk;
	struct clk_init_data init;

	gate = kzalloc(sizeof(struct clk_gate_scu), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	/* struct clk_gate_scu assignments */
	gate->lock = lock;
	gate->rsrc_id = rsrc_id;
	gate->gpr_id = gpr_id;
	gate->invert = invert_flag;

	init.name = name;
	init.ops = &clk_gate3_scu_ops;
	init.flags = 0;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);
	if (IS_ERR(clk))
		kfree(gate);

	return clk;
}
