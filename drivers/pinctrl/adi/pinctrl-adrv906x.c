// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2022, Analog Devices Incorporated, All Rights Reserved
 */

#include <dt-bindings/pinctrl/pinctrl-adi-adrv906x-io-pad.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-adi.h"
#include "pinctrl-adrv906x-init-tbl.h"

static const struct of_device_id adi_adrv906x_pinctrl_of_match[] = {
	{ .compatible = "adi,adrv906x-pinctrl", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adi_adrv906x_pinctrl_of_match);

static struct adi_pinctrl_soc_info adi_adrv906x_pinctrl_info = {
	.pins			= adi_adrv906x_pinctrl_pads,
	.npins			= ARRAY_SIZE(adi_adrv906x_pinctrl_pads),
	.adi_pinconf_get	= adi_pinconf_get_smc,
	.adi_pinconf_set	= adi_pinconf_set_smc,
	.adi_pinctrl_parse_pin	= NULL,
};

static int adi_adrv906x_pinctrl_probe(struct platform_device *pdev)
{
	return adi_pinctrl_probe(pdev, &adi_adrv906x_pinctrl_info);
}

static struct platform_driver adi_adrv906x_pinctrl_driver = {
	.driver				={
		.name			= "adrv906x-pinctrl",
		.of_match_table		= of_match_ptr(adi_adrv906x_pinctrl_of_match),
		.suppress_bind_attrs	= true,
	},
	.probe				= adi_adrv906x_pinctrl_probe,
};

static int __init adi_adrv906x_pinctrl_init(void)
{
	return platform_driver_register(&adi_adrv906x_pinctrl_driver);
}
arch_initcall(adi_adrv906x_pinctrl_init);

MODULE_AUTHOR("Howard Massey <Howard.Massey@analog.com>");
MODULE_DESCRIPTION("ADI ADRV906X pinctrl driver");
MODULE_LICENSE("GPL v2");
