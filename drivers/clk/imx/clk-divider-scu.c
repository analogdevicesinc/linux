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
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <soc/imx8/sc/sci.h>

#include "clk-imx8.h"

struct clk_divider_scu {
	struct clk_divider div;
	sc_rsrc_t	rsrc_id;
	sc_pm_clk_t	clk_type;
};

struct clk_divider3_scu {
	struct clk_divider div;
	sc_rsrc_t	rsrc_id;
	sc_ctrl_t	 gpr_id;
};

static inline struct clk_divider3_scu *to_clk_divider3_scu(struct clk_hw *hw)
{
	struct clk_divider *div = container_of(hw, struct clk_divider, hw);

	return container_of(div, struct clk_divider3_scu, div);
}

static inline struct clk_divider_scu *to_clk_divider_scu(struct clk_hw *hw)
{
	struct clk_divider *div = container_of(hw, struct clk_divider, hw);

	return container_of(div, struct clk_divider_scu, div);
}

static unsigned long clk_divider_scu_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct clk_divider_scu *clk = to_clk_divider_scu(hw);
	sc_err_t sci_err;
	sc_pm_clock_rate_t rate = 0;

	if (!ccm_ipc_handle)
		return 0;

	sci_err = sc_pm_get_clock_rate(ccm_ipc_handle, clk->rsrc_id,
		clk->clk_type, &rate);

	return sci_err ? 0 : rate;
}

static long clk_divider_scu_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	/* TODO */
	*prate = rate;

	return rate;
}

static int clk_divider_scu_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_divider_scu *clk = to_clk_divider_scu(hw);
	sc_err_t sci_err;

	if (!ccm_ipc_handle) {
		return -EAGAIN;
	}

	sci_err = sc_pm_set_clock_rate(ccm_ipc_handle, clk->rsrc_id,
		clk->clk_type, (sc_pm_clock_rate_t *)&rate);

	return sci_err ? -EINVAL : 0;
}

static struct clk_ops clk_divider_scu_ops = {
	.recalc_rate = clk_divider_scu_recalc_rate,
	.round_rate = clk_divider_scu_round_rate,
	.set_rate = clk_divider_scu_set_rate,
};

struct clk *imx_clk_divider_scu(const char *name,
				sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type)
{
	struct clk_divider_scu *div_clk;
	struct clk *clk;
	struct clk_init_data init;

	div_clk = kzalloc(sizeof(*div_clk), GFP_KERNEL);
	if (!div_clk)
		return ERR_PTR(-ENOMEM);

	div_clk->rsrc_id = rsrc_id;
	div_clk->clk_type = clk_type;

	init.name = name;
	init.ops = &clk_divider_scu_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	init.num_parents = 0;
	div_clk->div.hw.init = &init;

	clk = clk_register(NULL, &div_clk->div.hw);
	if (IS_ERR(clk))
		kfree(div_clk);

	return clk;
}

struct clk *imx_clk_divider2_scu(const char *name, const char *parent_name,
				sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type)
{
	struct clk_divider_scu *div_clk;
	struct clk *clk;
	struct clk_init_data init;

	div_clk = kzalloc(sizeof(*div_clk), GFP_KERNEL);
	if (!div_clk)
		return ERR_PTR(-ENOMEM);

	div_clk->rsrc_id = rsrc_id;
	div_clk->clk_type = clk_type;

	init.name = name;
	init.ops = &clk_divider_scu_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;
	div_clk->div.hw.init = &init;

	clk = clk_register(NULL, &div_clk->div.hw);
	if (IS_ERR(clk))
		kfree(div_clk);

	return clk;
}

static unsigned long clk_divider3_scu_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct clk_divider3_scu *clk = to_clk_divider3_scu(hw);
	uint32_t val;
	sc_err_t sci_err;
	sc_pm_clock_rate_t rate = 0;

	if (!ccm_ipc_handle)
		return 0;

	sci_err = sc_misc_get_control(ccm_ipc_handle, clk->rsrc_id,
		clk->gpr_id, &val);

	rate  = (val) ? parent_rate / 2 : parent_rate;

	return sci_err ? 0 : rate;
}

static long clk_divider3_scu_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	if (rate < *prate)
		rate = *prate / 2;
	else
		rate = *prate;

	return rate;
}

static int clk_divider3_scu_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_divider3_scu *clk = to_clk_divider3_scu(hw);
	uint32_t val;
	sc_err_t sci_err;

	if (!ccm_ipc_handle) {
		return -EAGAIN;
	}

	val = (rate < parent_rate) ? 1 : 0;
	sci_err = sc_misc_set_control(ccm_ipc_handle, clk->rsrc_id,
		clk->gpr_id, val);

	return sci_err ? -EINVAL : 0;
}

static struct clk_ops clk_divider3_scu_ops = {
	.recalc_rate = clk_divider3_scu_recalc_rate,
	.round_rate = clk_divider3_scu_round_rate,
	.set_rate = clk_divider3_scu_set_rate,
};

struct clk *imx_clk_divider3_scu(const char *name, const char *parent_name,
		sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id)
{
	struct clk_divider3_scu *div;
	struct clk *clk;
	struct clk_init_data init;

	div = kzalloc(sizeof(struct clk_divider3_scu), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	div->rsrc_id = rsrc_id;
	div->gpr_id = gpr_id;

	init.name = name;
	init.ops = &clk_divider3_scu_ops;
	init.flags = 0;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	div->div.hw.init = &init;

	clk = clk_register(NULL, &div->div.hw);
	if (IS_ERR(clk))
		kfree(div);

	return clk;
}
