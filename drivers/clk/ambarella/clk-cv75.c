// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ambarella CV75 RCT clock controller
 *
 * Copyright (C) 2026 Ambarella, Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <dt-bindings/clock/ambarella,cv75-rct.h>

#define CV75_NUM_CLKS		(CV75_GCLK_APB + 1)

#define PLL_CTRL_BYPASS		BIT(2)
#define PLL_CTRL_FRAC_MODE	BIT(3)
#define PLL_CTRL_FORCE_RESET	BIT(4)
#define PLL_CTRL_POWER_DOWN	BIT(5)
#define PLL_CTRL_HALT_VCO	BIT(6)
#define PLL_CTRL_SDIV		GENMASK(15, 12)
#define PLL_CTRL_SOUT		GENMASK(19, 16)
#define PLL_CTRL_INTP		GENMASK(30, 24)

#define PLL_CTRL2_VCODIV_DIV2	BIT(8)
#define PLL_CTRL2_FSDIV_DIV2	BIT(9)
#define PLL_CTRL2_FSOUT_DIV2	BIT(11)
#define PLL_CTRL2_BYPASS_HSDIV	BIT(12)

struct cv75_pll_regs {
	u32 ctrl;
	u32 frac;
	u32 ctrl2;
};

struct cv75_pll_hw {
	struct clk_hw hw;
	struct cv75_pll_regs regs;
};

struct cv75_clk {
	void __iomem *base;
	spinlock_t lock;
	struct cv75_pll_hw core;
	struct cv75_pll_hw enet;
	struct cv75_pll_hw sd;
};

struct cv75_pll {
	const char *name;
	struct cv75_pll_hw *hw;
	unsigned long flags;
	struct cv75_pll_regs regs;
	int id;
};

struct cv75_fixed_factor {
	const char *name;
	u32 id;
	struct u16_fract ratio;
};

struct cv75_composite {
	const char *name;
	const struct clk_parent_data *parents;
	unsigned long flags;
	u32 id;
	u16 mux_reg;
	u16 div_reg;
	u8 num_parents;
	u8 mux_shift;
	u8 mux_width;
	u8 div_shift;
	u8 div_width;
};

struct cv75_composite_hw {
	struct clk_mux mux;
	struct clk_divider div;
	struct clk_gate gate;
};

static struct cv75_clk cv75_ccu;

#define CV75_PLL(_id, _name, _ctrl, _frac, _ctrl2, _flags, _hw) {	\
	.name	= _name,						\
	.hw	= _hw,							\
	.flags	= _flags,						\
	.regs	= {							\
		.ctrl = _ctrl,						\
		.frac = _frac,						\
		.ctrl2 = _ctrl2,					\
	},								\
	.id	= _id,							\
}

static const struct cv75_pll cv75_plls[] = {
	CV75_PLL(CV75_GCLK_CORE, "core", 0x000, 0x004, 0x100,
		 0, &cv75_ccu.core),
	CV75_PLL(-1, "pll_enet", 0x520, 0x524, 0x528,
		 0, &cv75_ccu.enet),
	CV75_PLL(-1, "pll_sd", 0x4ac, 0x4b0, 0x4b4,
		 0, &cv75_ccu.sd),
};

static const struct cv75_fixed_factor cv75_fixed_factors[] = {
	{ .id = CV75_GCLK_AHB, .name = "ahb", .ratio = { 1, 2 } },
	{ .id = CV75_GCLK_APB, .name = "apb", .ratio = { 1, 4 } },
};

static const struct clk_parent_data cv75_uart_parents[] = {
	{ .fw_name = "osc" },
	{ .hw = &cv75_ccu.core.hw },
	{ .hw = &cv75_ccu.enet.hw },
	{ .hw = &cv75_ccu.sd.hw },
};

#define CV75_COMPOSITE(_id, _name, _parents, _mreg, _mshift, _mwidth,	\
		       _dreg, _dshift, _dwidth, _flags)			\
{									\
	.id		= _id,						\
	.name		= _name,					\
	.parents	= _parents,					\
	.num_parents	= ARRAY_SIZE(_parents),				\
	.mux_reg	= _mreg,					\
	.mux_shift	= _mshift,					\
	.mux_width	= _mwidth,					\
	.div_reg	= _dreg,					\
	.div_shift	= _dshift,					\
	.div_width	= _dwidth,					\
	.flags		= _flags,					\
}

static const struct cv75_composite cv75_composites[] = {
	CV75_COMPOSITE(CV75_GCLK_UART0, "uart0", cv75_uart_parents,
		       0x1c8, 0, 2, 0x038, 0, 24,
		       CLK_SET_RATE_NO_REPARENT),
};

#define to_cv75_pll(_hw) container_of(_hw, struct cv75_pll_hw, hw)

static unsigned long cv75_pll_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct cv75_pll_hw *pll = to_cv75_pll(hw);
	u32 ctrl, ctrl2, frac_val;
	u32 vcodiv, fsdiv, fsout;
	u32 intp, sdiv, sout;
	u64 vco, frac = 0;

	ctrl = readl_relaxed(cv75_ccu.base + pll->regs.ctrl);
	if (ctrl & (PLL_CTRL_POWER_DOWN | PLL_CTRL_HALT_VCO | PLL_CTRL_FORCE_RESET))
		return 0;

	if (ctrl & PLL_CTRL_BYPASS)
		return parent_rate;

	ctrl2 = readl_relaxed(cv75_ccu.base + pll->regs.ctrl2);
	intp = FIELD_GET(PLL_CTRL_INTP, ctrl) + 1;
	sdiv = FIELD_GET(PLL_CTRL_SDIV, ctrl) + 1;
	sout = FIELD_GET(PLL_CTRL_SOUT, ctrl) + 1;
	vcodiv = (ctrl2 & PLL_CTRL2_VCODIV_DIV2) ? 2 : 1;
	fsdiv = (ctrl2 & PLL_CTRL2_FSDIV_DIV2) ? 2 : 1;
	fsout = (ctrl2 & PLL_CTRL2_FSOUT_DIV2) ? 2 : 1;

	vco = (u64)parent_rate * vcodiv * fsdiv * intp * sdiv;
	if (ctrl & PLL_CTRL_FRAC_MODE) {
		frac_val = readl_relaxed(cv75_ccu.base + pll->regs.frac);
		frac = mul_u64_u32_shr((u64)parent_rate * vcodiv * fsdiv * sdiv,
				       frac_val, 32);
		vco += frac;
	}

	if (!(ctrl2 & PLL_CTRL2_BYPASS_HSDIV))
		vco = div_u64(vco, vcodiv * fsout * sout);

	return vco;
}

static const struct clk_ops cv75_pll_ops = {
	.recalc_rate = cv75_pll_recalc_rate,
};

static int cv75_register_plls(struct device *dev,
			      struct clk_hw_onecell_data *data)
{
	struct clk_parent_data parent_data = { .fw_name = "osc" };

	for (unsigned int i = 0; i < ARRAY_SIZE(cv75_plls); i++) {
		const struct cv75_pll *desc = &cv75_plls[i];
		struct cv75_pll_hw *pll = desc->hw;
		struct clk_init_data init = {};
		int ret;

		pll->regs = desc->regs;

		init.name = desc->name;
		init.ops = &cv75_pll_ops;
		init.parent_data = &parent_data;
		init.num_parents = 1;
		init.flags = desc->flags;
		pll->hw.init = &init;

		ret = devm_clk_hw_register(dev, &pll->hw);
		if (ret)
			return ret;

		if (desc->id >= 0)
			data->hws[desc->id] = &pll->hw;
	}

	return 0;
}

static int cv75_register_fixed_factors(struct device *dev,
				       struct clk_hw_onecell_data *data)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(cv75_fixed_factors); i++) {
		const struct cv75_fixed_factor *f = &cv75_fixed_factors[i];
		struct clk_hw *hw;

		hw = devm_clk_hw_register_fixed_factor_parent_hw(dev, f->name,
								 &cv75_ccu.core.hw, 0,
								 f->ratio.numerator,
								 f->ratio.denominator);
		if (IS_ERR(hw))
			return PTR_ERR(hw);

		data->hws[f->id] = hw;
	}

	return 0;
}

static struct clk_hw *cv75_register_composite(struct device *dev,
					      const struct cv75_composite *c)
{
	const struct clk_ops *mux_ops = NULL, *div_ops = NULL, *gate_ops = NULL;
	struct clk_hw *mux_hw = NULL, *div_hw = NULL, *gate_hw = NULL;
	struct cv75_composite_hw *comp;

	comp = devm_kzalloc(dev, sizeof(*comp), GFP_KERNEL);
	if (!comp)
		return ERR_PTR(-ENOMEM);

	if (c->mux_width) {
		comp->mux.reg = cv75_ccu.base + c->mux_reg;
		comp->mux.shift = c->mux_shift;
		comp->mux.mask = GENMASK(c->mux_width - 1, 0);
		comp->mux.lock = &cv75_ccu.lock;
		mux_hw = &comp->mux.hw;
		mux_ops = &clk_mux_ops;
	}

	if (c->div_width) {
		comp->div.reg = cv75_ccu.base + c->div_reg;
		comp->div.shift = c->div_shift;
		comp->div.width = c->div_width;
		comp->div.flags = CLK_DIVIDER_ONE_BASED;
		comp->div.lock = &cv75_ccu.lock;
		div_hw = &comp->div.hw;
		div_ops = &clk_divider_ops;

		comp->gate.reg = cv75_ccu.base + c->div_reg;
		comp->gate.bit_idx = c->div_shift + c->div_width;
		comp->gate.flags = CLK_GATE_SET_TO_DISABLE;
		comp->gate.lock = &cv75_ccu.lock;
		gate_hw = &comp->gate.hw;
		gate_ops = &clk_gate_ops;
	}

	return devm_clk_hw_register_composite_pdata(dev, c->name, c->parents,
						    c->num_parents,
						    mux_hw, mux_ops,
						    div_hw, div_ops,
						    gate_hw, gate_ops, c->flags);
}

static int cv75_register_composites(struct device *dev,
				    struct clk_hw_onecell_data *data)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(cv75_composites); i++) {
		const struct cv75_composite *c = &cv75_composites[i];
		struct clk_hw *hw;

		hw = cv75_register_composite(dev, c);
		if (IS_ERR(hw))
			return PTR_ERR(hw);

		data->hws[c->id] = hw;
	}

	return 0;
}

static int cv75_rct_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	cv75_ccu.base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cv75_ccu.base))
		return PTR_ERR(cv75_ccu.base);

	data = devm_kzalloc(dev, struct_size(data, hws, CV75_NUM_CLKS), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = CV75_NUM_CLKS;

	spin_lock_init(&cv75_ccu.lock);

	ret = cv75_register_plls(dev, data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register plls\n");

	ret = cv75_register_fixed_factors(dev, data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register fixed factors\n");

	ret = cv75_register_composites(dev, data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register composites\n");

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add clock provider\n");

	return 0;
}

static const struct of_device_id cv75_rct_match[] = {
	{ .compatible = "ambarella,cv75-rct" },
	{ }
};
MODULE_DEVICE_TABLE(of, cv75_rct_match);

static struct platform_driver cv75_rct_driver = {
	.probe = cv75_rct_probe,
	.driver = {
		.name = "ambarella-cv75-rct",
		.of_match_table = cv75_rct_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(cv75_rct_driver);

MODULE_DESCRIPTION("Ambarella CV75 RCT clock controller");
MODULE_LICENSE("GPL");
