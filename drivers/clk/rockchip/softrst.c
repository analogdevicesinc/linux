// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2014 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 */

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/reset-controller.h>
#include "clk.h"

struct rockchip_softrst {
	struct reset_controller_dev	rcdev;
	const int			*lut;
	void __iomem			*reg_base;
	int				num_regs;
};

#define NUM_PER_REG	16

static int rockchip_softrst_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct rockchip_softrst *softrst = container_of(rcdev,
						     struct rockchip_softrst,
						     rcdev);
	int bank, offset;

	if (softrst->lut)
		id = softrst->lut[id];

	bank = id / NUM_PER_REG;
	offset = id % NUM_PER_REG;

	writel(BIT(offset) | BIT(offset) << 16, softrst->reg_base + (bank * 4));

	return 0;
}

static int rockchip_softrst_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct rockchip_softrst *softrst = container_of(rcdev,
						     struct rockchip_softrst,
						     rcdev);
	int bank, offset;

	if (softrst->lut)
		id = softrst->lut[id];

	bank = id / NUM_PER_REG;
	offset = id % NUM_PER_REG;

	writel(BIT(offset) << 16, softrst->reg_base + (bank * 4));

	return 0;
}

static const struct reset_control_ops rockchip_softrst_ops = {
	.assert		= rockchip_softrst_assert,
	.deassert	= rockchip_softrst_deassert,
};

void rockchip_register_softrst_lut(struct device_node *np,
				   const int *lookup_table,
				   unsigned int num_regs,
				   void __iomem *base)
{
	struct rockchip_softrst *softrst;
	int ret;

	softrst = kzalloc_obj(*softrst);
	if (!softrst)
		return;

	softrst->reg_base = base;
	softrst->lut = lookup_table;
	softrst->num_regs = num_regs;

	softrst->rcdev.owner = THIS_MODULE;
	if (lookup_table)
		softrst->rcdev.nr_resets = num_regs;
	else
		softrst->rcdev.nr_resets = num_regs * NUM_PER_REG;
	softrst->rcdev.ops = &rockchip_softrst_ops;
	softrst->rcdev.of_node = np;
	ret = reset_controller_register(&softrst->rcdev);
	if (ret) {
		pr_err("%s: could not register reset controller, %d\n",
		       __func__, ret);
		kfree(softrst);
	}
};
EXPORT_SYMBOL_GPL(rockchip_register_softrst_lut);
