/*
 * Freescale ALSA SoC Digital Audio Interface (ACM) driver.
 *
 * Copyright 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software, you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or(at your
 * option) any later version.
 *
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/of_platform.h>

static int fsl_acm_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;

	pr_info("***** imx8qm_acm_init *****\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	return 0;
}

static const struct of_device_id fsl_acm_ids[] = {
	{ .compatible = "nxp,imx8qm-acm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_acm_ids);

static struct platform_driver fsl_acm_driver = {
	.probe = fsl_acm_probe,
	.driver = {
		.name = "fsl-acm",
		.of_match_table = fsl_acm_ids,
	},
};
module_platform_driver(fsl_acm_driver);

MODULE_DESCRIPTION("Freescale Soc ACM Interface");
MODULE_ALIAS("platform:fsl-acm");
MODULE_LICENSE("GPL");
