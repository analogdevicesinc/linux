// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 *	Dong Aisheng <aisheng.dong@nxp.com>
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "clk-scu.h"

#define IMX_LPCG_MAX_CLKS	8

static int imx8qxp_lpcg_clk_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char *output_names[IMX_LPCG_MAX_CLKS];
	const char *parent_names[IMX_LPCG_MAX_CLKS];
	unsigned int bit_offset[IMX_LPCG_MAX_CLKS];
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw **clk_hws;
	void __iomem *base;
	bool autogate;
	int count;
	int ret;
	int i;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	count = of_property_count_u32_elems(np, "bit-offset");
	if (count < 0) {
		dev_err(&pdev->dev, "failed to count clocks\n");
		return -EINVAL;
	}

	clk_data = devm_kzalloc(&pdev->dev, struct_size(clk_data, hws, count),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = count;
	clk_hws = clk_data->hws;

	ret = of_property_read_u32_array(np, "bit-offset", bit_offset,
					 clk_data->num);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to read clocks bit-offset\n");
		return -EINVAL;
	}

	ret = of_clk_parent_fill(np, parent_names, clk_data->num);
	if (ret != clk_data->num) {
		dev_err(&pdev->dev, "failed to get clock parent names\n");
		return -EINVAL;
	}

	ret = of_property_read_string_array(np, "clock-output-names",
					    output_names, clk_data->num);
	if (ret != clk_data->num) {
		dev_err(&pdev->dev, "failed to read clock-output-names\n");
		return -EINVAL;
	}

	autogate = of_property_read_bool(np, "hw-autogate");

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 500);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	for (i = 0; i < clk_data->num; i++) {
		if (bit_offset[i] > 31) {
			dev_warn(&pdev->dev, "invalid bit offset of clock %d\n",
				 i);
			ret = -EINVAL;
			goto unreg;
		}

		clk_hws[i] = imx_clk_lpcg_scu_dev(&pdev->dev, output_names[i],
						  parent_names[i], 0, base,
						  bit_offset[i], autogate);
		if (IS_ERR(clk_hws[i])) {
			dev_warn(&pdev->dev, "failed to register clock %d\n",
				 i);
			ret = PTR_ERR(clk_hws[i]);
			goto unreg;
		}
	}

	ret = devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_onecell_get,
					  clk_data);
	if (ret)
		goto unreg;

	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;

unreg:
	while (--i >= 0) {
		if (clk_hws[i])
			imx_clk_lpcg_scu_unregister(clk_hws[i]);
	}

	pm_runtime_disable(&pdev->dev);

	return ret;
}

static const struct of_device_id imx8qxp_lpcg_match[] = {
	{ .compatible = "fsl,imx8qxp-lpcg", NULL },
	{ /* sentinel */ }
};

static struct platform_driver imx8qxp_lpcg_clk_driver = {
	.driver = {
		.name = "imx8qxp-lpcg-clk",
		.of_match_table = imx8qxp_lpcg_match,
		.pm = &imx_clk_lpcg_scu_pm_ops,
		.suppress_bind_attrs = true,
	},
	.probe = imx8qxp_lpcg_clk_probe,
};

module_platform_driver(imx8qxp_lpcg_clk_driver);

MODULE_AUTHOR("Aisheng Dong <aisheng.dong@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX8QXP LPCG clock driver");
MODULE_LICENSE("GPL v2");
