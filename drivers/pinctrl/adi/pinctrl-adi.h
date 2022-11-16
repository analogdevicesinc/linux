/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices ADSP family pinctrl driver.
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Author: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef DRIVERS_PINCTRL_ADI_PINCTRL_ADI_H_
#define DRIVERS_PINCTRL_ADI_PINCTRL_ADI_H_

#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>

int adsp_pinctrl_probe(struct platform_device *pdev,
	const struct pinctrl_pin_desc *pins, size_t npins);

#endif
