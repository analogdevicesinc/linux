// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pinctrl / GPIO driver for StarFive JHB100 SoC eMMC domain
 *
 * Copyright (C) 2024 StarFive Technology Co., Ltd.
 * Author: Alex Soo <yuklin.soo@starfivetech.com>
 *
 */

#include <linux/platform_device.h>

#include "pinctrl-starfive-jhb100.h"

static const struct jhb100_pin_layout_desc jhb100_emmc_pl_desc[] = {
	{ .pin_start = 0, .pin_cnt = 12, .name = "gpio", .gpio_func_sel = 0 },
	{ 0xff },
};

static const struct config_reg_layout_desc jhb100_emmc_pinctrl_crl_desc[] = {
	{
		.pin_start					= 0,
		.pin_cnt					= 12,
		.fields[PAD_CFG_DEBOUNCE_WIDTH]			= { .shift = 0, .width = 17 },
	},
	{ 0xff },
};

static const struct starfive_pinctrl_regs jhb100_emmc_pinctrl_regs = {
	.func_sel		= { .reg = 0x3c, .width_per_pin = 2 },
	.config			= 0x00,
	.output			= 0x30,
	.output_en		= 0x34,
	.gpio_status		= 0x38,
	.irq_en			= 0x40,
	.irq_status		= 0x44,
	.irq_clr		= 0x48,
	.irq_trigger		= 0x4c,
	.irq_level		= 0x50,
	.irq_both_edge		= 0x54,
	.irq_edge		= 0x58,
};

static const struct jhb100_pinctrl_func_maps jhb100_func_maps_emmc[] = {
	{ .func = "emmc",		.val = 1 },
	{ .func = "gpio",		.val = 0 },
};

static const struct jhb100_pinctrl_domain_info jhb100_emmc_pinctrl_info = {
	.name			= "jhb100-emmc",
	.pl_desc		= jhb100_emmc_pl_desc,
	.crl_desc		= jhb100_emmc_pinctrl_crl_desc,
	.regs			= &jhb100_emmc_pinctrl_regs,
	.fmaps			= jhb100_func_maps_emmc,
	.num_maps		= ARRAY_SIZE(jhb100_func_maps_emmc),
};

static const struct of_device_id jhb100_emmc_pinctrl_of_match[] = {
	{
		.compatible = "starfive,jhb100-emmc-pinctrl",
		.data = &jhb100_emmc_pinctrl_info,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jhb100_emmc_pinctrl_of_match);

static struct platform_driver jhb100_emmc_pinctrl_driver = {
	.probe = jhb100_pinctrl_probe,
	.driver = {
		.name = "starfive-jhb100-emmc-pinctrl",
		.of_match_table = jhb100_emmc_pinctrl_of_match,
	},
};
module_platform_driver(jhb100_emmc_pinctrl_driver);

MODULE_DESCRIPTION("Pinctrl driver for StarFive JHB100 SoC eMMC domain");
MODULE_AUTHOR("Alex Soo <yuklin.soo@starfivetech.com>");
MODULE_LICENSE("GPL");
