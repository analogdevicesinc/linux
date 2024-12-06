// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2021 NXP
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <dt-bindings/reset/imx8ulp-sim-reset.h>

#define AVD_SIM_SYSCTRL0	0x8

struct imx8ulp_sim_reset {
	struct reset_controller_dev	rcdev;
	struct regmap			*regmap;
};

static const u32 imx8ulp_sim_reset_bits[IMX8ULP_SIM_RESET_NUM] = {
	[IMX8ULP_SIM_RESET_MIPI_DSI_RST_DPI_N] = BIT(3),
	[IMX8ULP_SIM_RESET_MIPI_DSI_RST_ESC_N] = BIT(4),
	[IMX8ULP_SIM_RESET_MIPI_DSI_RST_BYTE_N] = BIT(5),
};

static inline struct imx8ulp_sim_reset *
to_imx8ulp_sim_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct imx8ulp_sim_reset, rcdev);
}

static int imx8ulp_sim_reset_assert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct imx8ulp_sim_reset *simr = to_imx8ulp_sim_reset(rcdev);
	const u32 bit = imx8ulp_sim_reset_bits[id];

	return regmap_update_bits(simr->regmap, AVD_SIM_SYSCTRL0, bit, 0);
}

static int imx8ulp_sim_reset_deassert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	struct imx8ulp_sim_reset *simr = to_imx8ulp_sim_reset(rcdev);
	const u32 bit = imx8ulp_sim_reset_bits[id];

	return regmap_update_bits(simr->regmap, AVD_SIM_SYSCTRL0, bit, bit);
}

static const struct reset_control_ops imx8ulp_sim_reset_ops = {
	.assert		= imx8ulp_sim_reset_assert,
	.deassert	= imx8ulp_sim_reset_deassert,
};

static const struct of_device_id imx8ulp_sim_reset_dt_ids[] = {
	{ .compatible = "nxp,imx8ulp-avd-sim-reset", },
	{ /* sentinel */ },
};

static int imx8ulp_sim_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx8ulp_sim_reset *simr;
	int ret;

	simr = devm_kzalloc(dev, sizeof(*simr), GFP_KERNEL);
	if (!simr)
		return -ENOMEM;

	simr->regmap = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(simr->regmap)) {
		ret = PTR_ERR(simr->regmap);
		dev_err(dev, "failed to get regmap: %d\n", ret);
		return ret;
	}

	simr->rcdev.owner = THIS_MODULE;
	simr->rcdev.nr_resets = IMX8ULP_SIM_RESET_NUM;
	simr->rcdev.ops = &imx8ulp_sim_reset_ops;
	simr->rcdev.of_node = dev->of_node;

	return devm_reset_controller_register(dev, &simr->rcdev);
}

static struct platform_driver imx8ulp_sim_reset_driver = {
	.probe	= imx8ulp_sim_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= imx8ulp_sim_reset_dt_ids,
	},
};
module_platform_driver(imx8ulp_sim_reset_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX8ulp System Integretion Module reset driver");
MODULE_LICENSE("GPL v2");
