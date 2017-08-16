/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 */

#ifndef __MACH_S32_CLK_H
#define __MACH_S32_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>

#define PNAME(x) \
	static const char *x[] __initconst

void s32_check_clocks(struct clk *clks[], unsigned int count);

struct clk *s32_obtain_fixed_clock(
			const char *name, unsigned long rate);

static inline struct clk *s32_clk_fixed(const char *name, unsigned long rate)
{
	return clk_register_fixed_rate(NULL, name, NULL, 0, rate);
}

static inline struct clk *s32_clk_divider(const char *name, const char *parent,
					  void __iomem *reg, u8 shift, u8 width,
					  spinlock_t *lock)
{
	struct clk *tmp_clk = clk_register_divider(NULL, name, parent,
			      CLK_SET_RATE_PARENT,
			      reg, shift, width, 0, lock);

	return tmp_clk;
}

static inline struct clk *s32_clk_mux(const char *name, void __iomem *reg,
				      u8 shift, u8 width, const char **parents,
				      u8 num_parents, spinlock_t *lock)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0, lock);
}

static inline struct clk *s32_clk_fixed_factor(const char *name,
					       const char *parent,
					       unsigned int mult,
					       unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

#endif
