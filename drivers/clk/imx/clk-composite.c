/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>

#include "clk.h"

#define PCG_CGC		BIT(30)
#define PCG_PCS_SHIFT	24
#define PCG_PCS_MASK	0x7

struct clk *imx_clk_composite(const char *name, const char **parent_name,
			int num_parents, bool mux_present, bool rate_present, bool gate_present,
			 void __iomem *reg)
{
	struct clk_gate *gate = NULL;
	struct clk_mux *mux = NULL;
	struct clk_fractional_divider *div = NULL;
	struct clk_hw *mux_hw = NULL, *div_hw = NULL, *gate_hw = NULL;
	struct clk *clk;

	/* check if the mux is present in this composite clk. */
	if (mux_present) {
		mux = kzalloc(sizeof(*mux), GFP_KERNEL);
		if (!mux) {
			pr_err("%s: could not allocate mux clk\n", __func__);
			return ERR_PTR(-ENOMEM);
		}
		mux_hw = &mux->hw;
		/* init the mux struct */
		mux->reg = reg;
		mux->shift = PCG_PCS_SHIFT;
		mux->mask = PCG_PCS_MASK;
		/* mux->lock */
	}

	if (rate_present) {
		div = kzalloc(sizeof(*div), GFP_KERNEL);
		if (!div) {
			pr_err("%s: counld not allocate divider clk\n", __func__);
			kfree(mux);
			return ERR_PTR(-ENOMEM);
		}
		div_hw = &div->hw;
		/* init the div struct */
		div->reg = reg;
		div->mshift = 3;
		div->mwidth = 1;
		div->mmask  = (0x1) << 3;
		div->nshift = 0;
		div->nwidth = 3;
		div->nmask  = 0x7;
		div->flags = CLK_FRAC_DIVIDER_ZERO_BASED;
	}

	if (gate_present) {
		gate = kzalloc(sizeof(*gate), GFP_KERNEL);
		if (!gate) {
			pr_err("%s: could not allocate gate clk\n", __func__);
			kfree(mux);
			kfree(div);
			return ERR_PTR(-ENOMEM);
		}
		gate_hw = &gate->hw;
		/* init the gate struct */
		gate->reg = reg;
		gate->bit_idx = 30;
		/* gate->lock */
	}

	/* register the composite clk itself */
	clk = clk_register_composite(NULL, name, parent_name, num_parents,
				mux_hw, &clk_mux_ops, div_hw, &clk_fractional_divider_ops,
				gate_hw, &clk_gate_ops, CLK_SET_RATE_GATE | CLK_SET_PARENT_GATE);

	if (IS_ERR(clk)) {
		kfree(mux);
		kfree(div);
		kfree(gate);
	}

	return clk;
}
