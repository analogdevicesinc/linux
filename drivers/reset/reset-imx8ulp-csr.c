// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2024 NXP
 */

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

enum {
	CSR_SW_RESETN = 0,
	CSR_CTL_CLK_OFF,
	NUM_CSR_RESETS
};

struct imx8ulp_csr_reset {
	struct reset_controller_dev	rcdev;
	struct regmap			*regmap;

	u32				offset;
};

static inline struct imx8ulp_csr_reset *
to_imx8ulp_csr_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct imx8ulp_csr_reset, rcdev);
}

static int imx8ulp_csr_write(struct imx8ulp_csr_reset *csrr,
			     unsigned long id, u32 val)
{
	return regmap_update_bits(csrr->regmap, csrr->offset, BIT(id), val);
}

static int imx8ulp_csr_reset_assert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct imx8ulp_csr_reset *csrr = to_imx8ulp_csr_reset(rcdev);

	return imx8ulp_csr_write(csrr, id, 0x1);
}

static int imx8ulp_csr_reset_deassert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	struct imx8ulp_csr_reset *csrr = to_imx8ulp_csr_reset(rcdev);

	return imx8ulp_csr_write(csrr, id, 0x0);
}

static const struct reset_control_ops imx8ulp_csr_reset_ops = {
	.assert		= imx8ulp_csr_reset_assert,
	.deassert	= imx8ulp_csr_reset_deassert,
};

static const struct of_device_id imx8ulp_csr_reset_dt_ids[] = {
	{ .compatible = "nxp,imx8ulp-csr-regs-reset", },
	{ /* sentinel */ },
};

static int imx8ulp_csr_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx8ulp_csr_reset *csrr;
	int ret;

	csrr = devm_kzalloc(dev, sizeof(*csrr), GFP_KERNEL);
	if (!csrr)
		return -ENOMEM;

	csrr->regmap = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(csrr->regmap)) {
		ret = PTR_ERR(csrr->regmap);
		return dev_err_probe(dev, ret, "failed to get regmap\n");
	}

	ret = of_property_read_u32(dev->of_node, "fsl,offset", &csrr->offset);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get fsl,offset property\n");

	csrr->rcdev.owner = THIS_MODULE;
	csrr->rcdev.nr_resets = NUM_CSR_RESETS;
	csrr->rcdev.ops = &imx8ulp_csr_reset_ops;
	csrr->rcdev.of_node = dev->of_node;
	csrr->rcdev.dev = dev;

	platform_set_drvdata(pdev, csrr);

	return devm_reset_controller_register(dev, &csrr->rcdev);
}

static struct platform_driver imx8ulp_csr_reset_driver = {
	.probe = imx8ulp_csr_reset_probe,
	.driver = {
		.name = "imx8ulp_csr_reset",
		.of_match_table = imx8ulp_csr_reset_dt_ids,
	},
};
module_platform_driver(imx8ulp_csr_reset_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX8ulp MIPI CSI CSR module reset driver");
MODULE_LICENSE("GPL v2");
