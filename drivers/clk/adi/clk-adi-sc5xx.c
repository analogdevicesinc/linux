// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock driver for ADSP-SC5xx SoCs
 *
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * Author: Qasim Ijaz <qasim.ijaz@analog.com>
 * Contact: linux@analog.com
 */

#include "clk.h"

static void __init sc5xx_early_clock_probe(struct device_node *np)
{

}
CLK_OF_DECLARE();

static int sc5xx_clock_probe(struct platform_device *pdev)
{
	return 0;
}

static void sc5xx_clock_remove(struct platform_device *pdev)
{
}

static const struct of_device_id sc5xx_clock_of_match[] = {
	{
		.compatible = "adi,sc598-cgu0",
		.data = &sc598_cgu0_info,
	}, {
		.compatible = "adi,sc598-cgu1",
		.data = &sc598_cgu1_info,
	}, {
		.compatible = "adi,sc598-cgu2",
		.data = &sc598_cgu2_info,
	}, {
		.compatible = "adi,sc598-cdu",
		.data = &sc598_cdu_info,
	}, {
		.compatible = "adi,sc589-cgu0",
		.data = &sc589_cgu0_info,
	}, {
		.compatible = "adi,sc589-cgu1",
		.data = &sc589_cgu1_info,
	}, {
		.compatible = "adi,sc589-cdu",
		.data = &sc589_cdu_info,
	}, {
		.compatible = "adi,sc573-cgu0",
		.data = &sc573_cgu0_info,
	}, {
		.compatible = "adi,sc573-cgu1",
		.data = &sc573_cgu1_info,
	}, {
		.compatible = "adi,sc573-cdu",
		.data = &sc573_cdu_info,
	}, {
		.compatible = "adi,sc594-cgu0",
		.data = &sc594_cgu0_info,
	}, {
		.compatible = "adi,sc594-cgu1",
		.data = &sc594_cgu1_info,
	}, {
		.compatible = "adi,sc594-cdu",
		.data = &sc594_cdu_info,
	}, {
	},
};

static struct platform_driver sc5xx_clock_driver = {
	.driver	= {
		.name = "sc5xx_clock",
		.of_match_table = sc5xx_clock_of_match,
	},
	.probe = sc5xx_clock_probe,
	.remove = sc5xx_clock_remove,
};

static int __init sc5xx_clock_init(void)
{
	return platform_driver_register(&sc5xx_clock_driver);
}
core_initcall(sc5xx_clock_init);

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Clock driver for ADSP-SC5xx SoCs");
MODULE_LICENSE("GPLv2");
