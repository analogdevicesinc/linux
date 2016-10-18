/*
 * Copyright (C) 2010-2011 Canonical Ltd <jeremy.kerr@canonical.com>
 * Copyright (C) 2011-2012 Mike Turquette, Linaro Ltd <mturquette@linaro.org>
 * Copyright (C) 2016 Michael Hennerich, Analog Devices Inc. <michael.Hennerich@analog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Adjustable rate clock implementation
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct clk_adjustable_rate {
	struct		clk_hw hw;
	unsigned long	adjustable_rate;
	unsigned long	fixed_accuracy;
	unsigned long	min_rate;
	unsigned long	max_rate;
	u8		flags;
};

#define to_clk_adjustable_rate(_hw) container_of(_hw, struct clk_adjustable_rate, hw)

/*
 * DOC: basic adjustable-rate clock that cannot gate
 *
 * Traits of this clock:
 * prepare - clk_(un)prepare only ensures parents are prepared
 * enable - clk_enable only ensures parents are enabled
 * rate - rate is always a adjustable value.  No clk_set_rate support
 * parent - adjustable parent.  No clk_set_parent support
 */

static unsigned long clk_adjustable_rate_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return to_clk_adjustable_rate(hw)->adjustable_rate;
}

static unsigned long clk_adjustable_rate_recalc_accuracy(struct clk_hw *hw,
		unsigned long parent_accuracy)
{
	return to_clk_adjustable_rate(hw)->fixed_accuracy;
}

static int clk_adjustable_rate_set_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long parent_rate)
{
	if (rate > to_clk_adjustable_rate(hw)->max_rate ||
		rate < to_clk_adjustable_rate(hw)->min_rate)
		return -EINVAL;

	to_clk_adjustable_rate(hw)->adjustable_rate = rate;

	return 0;
}

static long clk_adjustable_rate_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *parent_rate)
{
	return rate;
}

const struct clk_ops clk_adjustable_rate_ops = {
	.recalc_rate = clk_adjustable_rate_recalc_rate,
	.recalc_accuracy = clk_adjustable_rate_recalc_accuracy,
	.set_rate = clk_adjustable_rate_set_rate,
	.round_rate = clk_adjustable_rate_round_rate,
};


/**
 * clk_register_adjustable_rate_with_accuracy - register adjustable-rate clock with the
 *					   clock framework
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @adjustable_rate: adjustable clock rate
 * @fixed_accuracy: non-adjustable clock rate
 */
struct clk *clk_register_adjustable_rate_with_accuracy(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		unsigned long adjustable_rate, unsigned long fixed_accuracy)
{
	struct clk_adjustable_rate *adjustable;
	struct clk *clk;
	struct clk_init_data init;
	u64 delta;

	/* allocate adjustable-rate clock */
	adjustable = kzalloc(sizeof(*adjustable), GFP_KERNEL);
	if (!adjustable)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_adjustable_rate_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = (parent_name ? &parent_name: NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_adjustable_rate assignments */
	adjustable->adjustable_rate = adjustable_rate;
	adjustable->fixed_accuracy = fixed_accuracy;
	adjustable->hw.init = &init;

	delta = (u64) adjustable_rate * (u64) fixed_accuracy;
	do_div(delta, 1000000000U);
	adjustable->max_rate = adjustable_rate + delta;
	adjustable->min_rate = adjustable_rate - delta;

	/* register the clock */
	clk = clk_register(dev, &adjustable->hw);
	if (IS_ERR(clk))
		kfree(adjustable);

	return clk;
}

/**
 * clk_register_adjustable_rate - register adjustable-rate clock with the clock framework
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @adjustable_rate: adjustable clock rate
 */
struct clk *clk_register_adjustable_rate(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		unsigned long adjustable_rate)
{
	return clk_register_adjustable_rate_with_accuracy(dev, name, parent_name,
						     flags, adjustable_rate, 0);
}

void clk_unregister_adjustable_rate(struct clk *clk)
{
	struct clk_hw *hw;

	hw = __clk_get_hw(clk);
	if (!hw)
		return;

	clk_unregister(clk);
	kfree(to_clk_adjustable_rate(hw));
}

#ifdef CONFIG_OF
/**
 * of_adjustable_clk_setup() - Setup function for simple adjustable rate clock
 */
struct clk *_of_adjustable_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	u32 rate;
	u32 accuracy = 0;
	int ret;

	if (of_property_read_u32(node, "clock-frequency", &rate))
		return ERR_PTR(-EIO);

	if (of_property_read_u32(node, "clock-accuracy", &accuracy))
		return ERR_PTR(-EIO);

	of_property_read_string(node, "clock-output-names", &clk_name);

	clk = clk_register_adjustable_rate_with_accuracy(NULL, clk_name, NULL,
						    0, rate, accuracy);
	if (IS_ERR(clk))
		return clk;

	ret = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (ret) {
		clk_unregister(clk);
		return ERR_PTR(ret);
	}

	return clk;
}

void of_adjustable_clk_setup(struct device_node *node)
{
	if (!IS_ERR(_of_adjustable_clk_setup(node)))
		of_node_set_flag(node, OF_POPULATED);
}
EXPORT_SYMBOL_GPL(of_adjustable_clk_setup);
CLK_OF_DECLARE(adjustable_clk, "adjustable-clock", of_adjustable_clk_setup);

static int of_adjustable_clk_remove(struct platform_device *pdev)
{
	struct clk *clk = platform_get_drvdata(pdev);

	if (clk)
		clk_unregister_adjustable_rate(clk);

	return 0;
}

static int of_adjustable_clk_probe(struct platform_device *pdev)
{
	struct clk *clk;

	/*
	 * This function is not executed when of_adjustable_clk_setup
	 * succeeded.
	 */

	clk = _of_adjustable_clk_setup(pdev->dev.of_node);

	if (IS_ERR(clk))
		return PTR_ERR(clk);

	platform_set_drvdata(pdev, clk);

	return 0;
}

static const struct of_device_id of_adjustable_clk_ids[] = {
	{ .compatible = "adjustable-clock" },
	{ },
};
MODULE_DEVICE_TABLE(of, of_adjustable_clk_ids);

static struct platform_driver of_adjustable_clk_driver = {
	.driver = {
		.name = "of_adjustable_clk",
		.of_match_table = of_adjustable_clk_ids,
	},
	.probe = of_adjustable_clk_probe,
	.remove = of_adjustable_clk_remove,
};

builtin_platform_driver(of_adjustable_clk_driver);
#endif
