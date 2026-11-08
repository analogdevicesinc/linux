// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pinctrl / GPIO driver for StarFive JHB100 SoC ADC-1 domain
 *
 * Copyright (C) 2024 StarFive Technology Co., Ltd.
 * Author: Alex Soo <yuklin.soo@starfivetech.com>
 *
 */

#include <linux/platform_device.h>

#include "pinctrl-starfive-jhb100.h"

static const struct jhb100_pin_layout_desc jhb100_adc1_pl_desc[] = {
	{ .pin_start = 0, .pin_cnt = 8, .name = "gpio", .gpio_func_sel = 0 },
	{ 0xff },
};

static const struct config_reg_layout_desc jhb100_adc1_pinctrl_crl_desc[] = {
	{
		.pin_start					= 0,
		.pin_cnt					= 8,
		.fields[PAD_CFG_DRIVE_STRENGTH_3BIT]		= { .shift = 0, .width = 3 },
		.fields[PAD_CFG_INPUT_ENABLE]			= { .shift = 3, .width = 1 },
		.fields[PAD_CFG_PULL_DOWN]			= { .shift = 4, .width = 1 },
		.fields[PAD_CFG_PULL_UP]			= { .shift = 5, .width = 1 },
		.fields[PAD_CFG_SLEW_RATE]			= { .shift = 6, .width = 1 },
		.fields[PAD_CFG_SCHMITT_TRIGGER_SELECT]		= { .shift = 7, .width = 1 },
		.fields[PAD_CFG_DEBOUNCE_WIDTH]			= { .shift = 15, .width = 17 },
	},
	{ 0xff },
};

static const struct starfive_pinctrl_regs jhb100_adc1_pinctrl_regs = {
	.func_sel		= { .reg = 0x2c, .width_per_pin = 2 },
	.config			= 0x00,
	.output			= 0x20,
	.output_en		= 0x24,
	.gpio_status		= 0x28,
	.irq_en			= 0x30,
	.irq_status		= 0x34,
	.irq_clr		= 0x38,
	.irq_trigger		= 0x3c,
	.irq_level		= 0x40,
	.irq_both_edge		= 0x44,
	.irq_edge		= 0x48,
};

static const struct jhb100_pinctrl_func_maps jhb100_func_maps_adc1[] = {
	{ .func = "adc",		.val = 1 },
	{ .func = "gpio",		.val = 0 },
	{ .func = "host1_port80",	.val = 2 },
	{ .func = "scap_trigger",	.val = 3 },
};

static const struct jhb100_pinctrl_domain_info jhb100_adc1_pinctrl_info = {
	.name			= "jhb100-adc1",
	.pl_desc		= jhb100_adc1_pl_desc,
	.crl_desc		= jhb100_adc1_pinctrl_crl_desc,
	.regs			= &jhb100_adc1_pinctrl_regs,
	.fmaps			= jhb100_func_maps_adc1,
	.num_maps		= ARRAY_SIZE(jhb100_func_maps_adc1),
};

static const struct of_device_id jhb100_adc1_pinctrl_of_match[] = {
	{
		.compatible = "starfive,jhb100-adc1-pinctrl",
		.data = &jhb100_adc1_pinctrl_info,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jhb100_adc1_pinctrl_of_match);

static struct platform_driver jhb100_adc1_pinctrl_driver = {
	.probe = jhb100_pinctrl_probe,
	.driver = {
		.name = "starfive-jhb100-adc1-pinctrl",
		.of_match_table = jhb100_adc1_pinctrl_of_match,
	},
};
module_platform_driver(jhb100_adc1_pinctrl_driver);

MODULE_DESCRIPTION("Pinctrl driver for StarFive JHB100 SoC ADC-1 domain");
MODULE_AUTHOR("Alex Soo <yuklin.soo@starfivetech.com>");
MODULE_LICENSE("GPL");
