/*
 * Platform Driver Support for FPGA Freeze Bridge Controller
 *
 *  Copyright (C) 2016 Altera Corporation. All rights reserved.
 *  Copyright (C) 2016 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "altera-freeze-bridge.h"
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/module.h>

static int altera_freeze_br_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	void __iomem *reg_base;

	if (!np)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	reg_base = devm_ioremap_resource(dev, res);

	if (IS_ERR(reg_base))
		return PTR_ERR(reg_base);

	return altera_freeze_br_probe(dev, reg_base);
}

static int altera_freeze_br_platform_remove(struct platform_device *pdev)
{
	return altera_freeze_br_remove(&pdev->dev);
}

static const struct of_device_id altera_freeze_br_of_match[] = {
	{ .compatible = "altr,freeze-bridge-controller", },
	{},
};
MODULE_DEVICE_TABLE(of, altera_freeze_br_of_match);

static struct platform_driver altera_freeze_br_driver = {
	.probe = altera_freeze_br_platform_probe,
	.remove = altera_freeze_br_platform_remove,
	.driver = {
		.name	= "altera_freeze_br",
		.of_match_table = of_match_ptr(altera_freeze_br_of_match),
	},
};

module_platform_driver(altera_freeze_br_driver);
