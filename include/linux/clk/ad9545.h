/* SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause */
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#ifndef _AD9545_H_
#define _AD9545_H_

#include <linux/clk.h>

int ad9545_select_source(struct clk *tuning_clk, struct clk *out_clk);

#endif /* _AD9545_H_ */
