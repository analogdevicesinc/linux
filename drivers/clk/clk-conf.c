// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 * Sylwester Nawrocki <s.nawrocki@samsung.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/clk-conf.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/slab.h>

static int __set_clk_parents(struct device_node *node, bool clk_supplier)
{
	struct of_phandle_args clkspec;
	int index, rc, num_parents;
	struct clk *clk, *pclk;

	num_parents = of_count_phandle_with_args(node, "assigned-clock-parents",
						 "#clock-cells");
	if (num_parents == -EINVAL)
		pr_err("clk: invalid value of clock-parents property at %pOF\n",
		       node);

	for (index = 0; index < num_parents; index++) {
		rc = of_parse_phandle_with_args(node, "assigned-clock-parents",
					"#clock-cells",	index, &clkspec);
		if (rc < 0) {
			/* skip empty (null) phandles */
			if (rc == -ENOENT)
				continue;
			else
				return rc;
		}
		if (clkspec.np == node && !clk_supplier) {
			of_node_put(clkspec.np);
			return 0;
		}
		pclk = of_clk_get_from_provider(&clkspec);
		of_node_put(clkspec.np);
		if (IS_ERR(pclk)) {
			if (PTR_ERR(pclk) != -EPROBE_DEFER)
				pr_warn("clk: couldn't get parent clock %d for %pOF\n",
					index, node);
			return PTR_ERR(pclk);
		}

		rc = of_parse_phandle_with_args(node, "assigned-clocks",
					"#clock-cells", index, &clkspec);
		if (rc < 0)
			goto err;
		if (clkspec.np == node && !clk_supplier) {
			of_node_put(clkspec.np);
			rc = 0;
			goto err;
		}
		clk = of_clk_get_from_provider(&clkspec);
		of_node_put(clkspec.np);
		if (IS_ERR(clk)) {
			if (PTR_ERR(clk) != -EPROBE_DEFER)
				pr_warn("clk: couldn't get assigned clock %d for %pOF\n",
					index, node);
			rc = PTR_ERR(clk);
			goto err;
		}

		rc = clk_set_parent(clk, pclk);
		if (rc < 0)
			pr_err("clk: failed to reparent %s to %s: %d\n",
			       __clk_get_name(clk), __clk_get_name(pclk), rc);
		clk_put(clk);
		clk_put(pclk);
	}
	return 0;
err:
	clk_put(pclk);
	return rc;
}

static int __set_clk_rates(struct device_node *node, bool clk_supplier)
{
	struct of_phandle_args clkspec;
	int rc, count, count_64, index;
	struct clk *clk;
	u64 *rates_64 __free(kfree) = NULL;
	u32 *rates __free(kfree) = NULL;

	count = of_property_count_u32_elems(node, "assigned-clock-rates");
	count_64 = of_property_count_u64_elems(node, "assigned-clock-rates-u64");
	if (count_64 > 0) {
		count = count_64;
		rates_64 = kcalloc(count, sizeof(*rates_64), GFP_KERNEL);
		if (!rates_64)
			return -ENOMEM;

		rc = of_property_read_u64_array(node,
						"assigned-clock-rates-u64",
						rates_64, count);
	} else if (count > 0) {
		rates = kcalloc(count, sizeof(*rates), GFP_KERNEL);
		if (!rates)
			return -ENOMEM;

		rc = of_property_read_u32_array(node, "assigned-clock-rates",
						rates, count);
	} else {
		return 0;
	}

	if (rc)
		return rc;

	for (index = 0; index < count; index++) {
		unsigned long rate;

		if (rates_64)
			rate = rates_64[index];
		else
			rate = rates[index];

		if (rate) {
			rc = of_parse_phandle_with_args(node, "assigned-clocks",
					"#clock-cells",	index, &clkspec);
			if (rc < 0) {
				/* skip empty (null) phandles */
				if (rc == -ENOENT)
					continue;
				else
					return rc;
			}
			if (clkspec.np == node && !clk_supplier) {
				of_node_put(clkspec.np);
				return 0;
			}

			clk = of_clk_get_from_provider(&clkspec);
			of_node_put(clkspec.np);
			if (IS_ERR(clk)) {
				if (PTR_ERR(clk) != -EPROBE_DEFER)
					pr_warn("clk: couldn't get clock %d for %pOF\n",
						index, node);
				return PTR_ERR(clk);
			}

			rc = clk_set_rate(clk, rate);
			if (rc < 0)
				pr_err("clk: couldn't set %s clk rate to %lu (%d), current rate: %lu\n",
				       __clk_get_name(clk), rate, rc,
				       clk_get_rate(clk));
			clk_put(clk);
		}
	}
	return 0;
}

static int __set_clk_nshot(struct device_node *node, bool clk_supplier)
{
	struct of_phandle_args clkspec;
	int rc, index = 0;
	struct clk *clk;
	u32 nshot;

	of_property_for_each_u32(node, "assigned-clock-nshot", nshot) {
		if (nshot) {
			rc = of_parse_phandle_with_args(node, "assigned-clocks",
							"#clock-cells",	index, &clkspec);
			if (rc < 0) {
				/* skip empty (null) phandles */
				if (rc == -ENOENT)
					continue;

				return rc;
			}

			if (clkspec.np == node && !clk_supplier) {
				of_node_put(clkspec.np);
				return 0;
			}

			clk = of_clk_get_from_provider(&clkspec);
			if (IS_ERR(clk)) {
				if (PTR_ERR(clk) != -EPROBE_DEFER)
					pr_err("clk: couldn't get clock %d for %pOF\n", index,
					       node);
				of_node_put(clkspec.np);
				return PTR_ERR(clk);
			}

			rc = clk_set_nshot(clk, nshot);
			if (rc < 0)
				pr_warn("clk: couldn't set %s clk nshot to %u (%d), current nshot: %u\n",
					__clk_get_name(clk), nshot, rc, clk_get_nshot(clk));

			clk_put(clk);
			of_node_put(clkspec.np);
		}
		index++;
	}

	return 0;
}

/**
 * of_clk_set_defaults() - parse and set assigned clocks configuration
 * @node: device node to apply clock settings for
 * @clk_supplier: true if clocks supplied by @node should also be considered
 *
 * This function parses 'assigned-{clocks/clock-parents/clock-rates}' properties
 * and sets any specified clock parents and rates. The @clk_supplier argument
 * should be set to true if @node may be also a clock supplier of any clock
 * listed in its 'assigned-clocks' or 'assigned-clock-parents' properties.
 * If @clk_supplier is false the function exits returning 0 as soon as it
 * determines the @node is also a supplier of any of the clocks.
 */
int of_clk_set_defaults(struct device_node *node, bool clk_supplier)
{
	int rc;

	if (!node)
		return 0;

	rc = __set_clk_parents(node, clk_supplier);
	if (rc < 0)
		return rc;

	rc = __set_clk_rates(node, clk_supplier);
	if (rc < 0)
		return rc;

	return __set_clk_nshot(node, clk_supplier);
}
EXPORT_SYMBOL_GPL(of_clk_set_defaults);
