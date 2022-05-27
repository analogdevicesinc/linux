// SPDX-License-Identifier: GPL-2.0

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/adi,ad242x.h>

#define AD242X_NUM_CLKS 2

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

struct ad242x_clk_hw {
	struct clk_hw hw;
	struct clk_init_data init;
	struct a2b_node *node;
	u8 reg;
};

struct ad242x_clk_driver_data {
	struct ad242x_clk_hw hw[AD242X_NUM_CLKS];
};

static inline struct ad242x_clk_hw *to_ad242x_clk(struct clk_hw *hw)
{
	return container_of(hw, struct ad242x_clk_hw, hw);
}

static int ad242x_clk_prepare(struct clk_hw *hw)
{
	struct ad242x_clk_hw *clk_hw = to_ad242x_clk(hw);

	return regmap_update_bits(clk_hw->node->regmap, clk_hw->reg,
				  A2B_CLKCFG_EN, A2B_CLKCFG_EN);
}

static void ad242x_clk_unprepare(struct clk_hw *hw)
{
	struct ad242x_clk_hw *clk_hw = to_ad242x_clk(hw);

	regmap_update_bits(clk_hw->node->regmap, clk_hw->reg, A2B_CLKCFG_EN, 0);
}

static void ad242x_do_div(unsigned long rate, unsigned long parent_rate,
			  unsigned long *prediv, unsigned long *div)
{
	if (rate < parent_rate / 32UL)
		*prediv = 32UL;
	else
		*prediv = 2UL;

	parent_rate /= *prediv;
	*div = parent_rate / rate;
}

static int ad242x_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct ad242x_clk_hw *clk_hw = to_ad242x_clk(hw);
	unsigned long pll_rate = parent_rate * 2048UL;
	unsigned long prediv, div;
	unsigned int val = 0;

	if (rate > pll_rate / 4 || rate < pll_rate / 1024UL)
		return -EINVAL;

	ad242x_do_div(rate, pll_rate, &prediv, &div);

	if (prediv == 32UL)
		val |= A2B_CLKCFG_PDIV32;

	val |= A2B_CLKCFG_DIV((div / 2UL) - 1UL);

	return regmap_update_bits(clk_hw->node->regmap, clk_hw->reg,
				  A2B_CLKCFG_DIVMSK | A2B_CLKCFG_PDIV32, val);
}

static unsigned long ad242x_clk_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct ad242x_clk_hw *clk_hw = to_ad242x_clk(hw);
	unsigned long pll_rate = parent_rate * 2048UL;
	unsigned long prediv, div;
	unsigned int val;
	int ret;

	ret = regmap_read(clk_hw->node->regmap, clk_hw->reg, &val);
	if (ret < 0)
		return ret;

	prediv = (val & A2B_CLKCFG_PDIV32) ? 32UL : 2UL;
	div = 2UL * ((val & A2B_CLKCFG_DIVMSK) + 1UL);

	return pll_rate / (prediv * div);
}

static long ad242x_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	unsigned long pll_rate = *parent_rate * 2048UL;
	unsigned long prediv, div;

	if (rate > pll_rate / 4 || rate < pll_rate / 1024UL)
		return -EINVAL;

	ad242x_do_div(rate, pll_rate, &prediv, &div);

	return pll_rate / (prediv * div);
}

static int ad242x_clk_get_phase(struct clk_hw *hw)
{
	struct ad242x_clk_hw *clk_hw = to_ad242x_clk(hw);
	unsigned int val;
	int ret;

	ret = regmap_read(clk_hw->node->regmap, clk_hw->reg, &val);
	if (ret < 0)
		return ret;

	return (val & A2B_CLKCFG_INV) ? 180 : 0;
}

static int ad242x_clk_set_phase(struct clk_hw *hw, int phase)
{
	struct ad242x_clk_hw *clk_hw = to_ad242x_clk(hw);
	unsigned int val;

	switch (phase) {
	case 0:
		val = 0;
		break;
	case 180:
		val = A2B_CLKCFG_INV;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(clk_hw->node->regmap, clk_hw->reg,
				  A2B_CLKCFG_INV, val);
}

static const struct clk_ops ad242x_clk_ops = {
	.prepare = ad242x_clk_prepare,
	.unprepare = ad242x_clk_unprepare,
	.get_phase = ad242x_clk_get_phase,
	.set_phase = ad242x_clk_set_phase,
	.recalc_rate = ad242x_clk_recalc_rate,
	.round_rate = ad242x_clk_round_rate,
	.set_rate = ad242x_clk_set_rate,
};

static struct clk_hw *ad242x_of_clk_get(struct of_phandle_args *clkspec,
					void *data)
{
	struct ad242x_clk_driver_data *drvdata = data;
	unsigned int idx = clkspec->args[0];

	if (idx >= ARRAY_SIZE(drvdata->hw))
		return ERR_PTR(-EINVAL);

	return &drvdata->hw[idx].hw;
}

static int ad242x_clk_probe(struct platform_device *pdev)
{
	const char *clk_names[AD242X_NUM_CLKS] = { "clkout1", "clkout2" };
	u8 regs[AD242X_NUM_CLKS] = { A2B_CLK1CFG, A2B_CLK2CFG };
	struct ad242x_clk_driver_data *drvdata;
	struct device *dev = &pdev->dev;
	const char *sync_clk_name;
	struct a2b_node *node;
	int i, ret;

	if (!dev->of_node)
		return -ENODEV;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	node = dev_to_a2b_node(dev->parent);
	sync_clk_name = __clk_get_name(a2b_node_get_sync_clk(node));

	for (i = 0; i < AD242X_NUM_CLKS; i++) {
		const char *name;

		if (of_property_read_string_index(
			    dev->of_node, "clock-output-names", i, &name) == 0)
			drvdata->hw[i].init.name = name;
		else
			drvdata->hw[i].init.name = clk_names[i];

		drvdata->hw[i].reg = regs[i];
		drvdata->hw[i].init.ops = &ad242x_clk_ops;
		drvdata->hw[i].init.num_parents = 1;
		drvdata->hw[i].init.parent_names = &sync_clk_name;
		drvdata->hw[i].hw.init = &drvdata->hw[i].init;
		drvdata->hw[i].node = node;

		ret = devm_clk_hw_register(dev, &drvdata->hw[i].hw);
		if (ret < 0)
			return ret;
	}

	return devm_of_clk_add_hw_provider(dev, ad242x_of_clk_get, drvdata);
}

static const struct of_device_id ad242x_dt_ids[] = {
	{
		.compatible = "adi,ad2428w-clk",
	},
	{}
};
MODULE_DEVICE_TABLE(of, ad242x_dt_ids);

static struct platform_driver ad242x_clk_driver = {
	.probe = ad242x_clk_probe,
	.driver = {
		.name = "ad242x-clk",
		.of_match_table	= ad242x_dt_ids,
	},
};
module_platform_driver(ad242x_clk_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("AD242x clock driver");
MODULE_LICENSE("GPL v2");
