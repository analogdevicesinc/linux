/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Freescale Semiconductor, Inc.
 *
 * Author: Lee Jones <lee.jones@linaro.org>
 */

#ifndef __LINUX_CLK_IMX_H
#define __LINUX_CLK_IMX_H

#include <linux/types.h>

void imx6sl_set_wait_clk(bool enter);
void imx6sx_set_m4_highfreq(bool high_freq);

#endif
