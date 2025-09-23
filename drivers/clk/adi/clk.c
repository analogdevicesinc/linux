// SPDX-License-Identifier: GPL-2.0
/*
 * Clock support for ADI processor
 *
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#include <linux/slab.h>
#include <linux/clk.h>

#include "clk.h"

struct adi_clk_provider *adi_clk_init(struct device_node *np,
				      void __iomem *reg_base,
				      unsigned long nr_clks)
{
	struct adi_clk_provider *ctx;
	struct clk **clk_table;
	int i;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	clk_table = kcalloc(nr_clks, sizeof(struct clk *), GFP_KERNEL);
	if (!clk_table)
		goto err_free;

	for (i = 0; i < nr_clks; ++i)
		clk_table[i] = ERR_PTR(-ENOENT);

	ctx->reg_base = reg_base;
	ctx->clk_data.clks = clk_table;
	ctx->clk_data.clk_num = nr_clks;
	ctx->clk_node = np;
	spin_lock_init(&ctx->lock);

	return ctx;

err_free:
	kfree(ctx);

	return ERR_PTR(-ENOMEM);
}

static void adi_clk_add_lookup(struct adi_clk_provider *ctx,
			       struct clk *clk, unsigned int id)
{
	ctx->clk_data.clks[id] = clk;
}

void adi_clk_register_branches(struct adi_clk_provider *ctx,
			       struct adi_clk_branch *list,
			       unsigned int nr_clk)
{
	struct clk *clk = NULL;
	unsigned int idx;

	for (idx = 0; idx < nr_clk; idx++, list++) {
		clk = NULL;
		switch (list->branch_type) {
		case branch_divider:
			clk = clk_register_divider(NULL, list->name,
				list->parent_names[0],
				CLK_SET_RATE_PARENT,
				ctx->reg_base + list->offset,
				list->shift,
				list->div_width,
				list->div_flags,
				&ctx->lock);
			break;
		case branch_mux:
			clk = clk_register_mux(NULL,
				list->name,
				list->parent_names,
				list->num_parents, //check if that match
				list->flags,
				ctx->reg_base + list->offset,
				list->shift,
				list->mux_width,
				0,
				&ctx->lock);
			break;
		case branch_cdu_mux:
			clk = clk_register_mux(NULL,
				list->name,
				list->parent_names,
				list->num_parents, //check if that match
				CLK_SET_RATE_PARENT,
				ctx->reg_base + list->offset,
				CDU_MUX_SHIFT,
				CDU_MUX_WIDTH,
				0,
				&ctx->lock);
			break;
		case branch_pll:
			clk = sc5xx_cgu_pll(list->name,
				list->parent_names[0],
				ctx->reg_base + list->offset,
				list->pll_shift,
				list->pll_width,
				list->pll_m,
				&ctx->lock);
			break;
		case branch_cgu_gate:
			clk = clk_register_gate(NULL,
				list->name,
				list->parent_names[0],
				CLK_SET_RATE_PARENT,
				ctx->reg_base + list->offset,
				list->gate_bit,
				CLK_GATE_SET_TO_DISABLE,
				&ctx->lock);

			break;
		case branch_cdu_gate:
			clk = clk_register_gate(NULL,
				list->name,
				list->parent_names[0],
				CLK_SET_RATE_PARENT | list->flags,
				ctx->reg_base + list->offset,
				CDU_EN_BIT,
				0,
				&ctx->lock);

			break;
		case branch_of_node:
			clk = of_clk_get_by_name(ctx->clk_node, list->name);
			break;
		}

		/* none of the cases above matched */
		if (!clk) {
			pr_err("%s: unknown clock type %d\n",
			       __func__, list->branch_type);
			continue;
		}

		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s: %ld\n",
			       __func__, list->name, PTR_ERR(clk));
			continue;
		}

		adi_clk_add_lookup(ctx, clk, list->id);
	}
}
