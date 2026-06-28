/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Clock support for ADI processors
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Author: Greg Malysa <greg.malysa@timesys.com>
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 */

#ifndef CLK_ADI_CLK_H
#define CLK_ADI_CLK_H

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#define CGU_CTL         0x00
#define CGU_PLLCTL      0x04
#define CGU_STAT        0x08
#define CGU_DIV         0x0C
#define CGU_CLKOUTSEL   0x10
#define CGU_OSCWDCTL    0x14
#define CGU_TSCTL       0x18
#define CGU_TSVALUE0    0x1C
#define CGU_TSVALUE1    0x20
#define CGU_TSCOUNT0    0x24
#define CGU_TSCOUNT1    0x28
#define CGU_CCBF_DIS    0x2C
#define CGU_CCBF_STAT   0x30
#define CGU_SCBF_DIS    0x38
#define CGU_SCBF_STAT   0x3C
#define CGU_DIVEX       0x40
#define CGU_REVID       0x48

#define CDU_CFG0     0x00
#define CDU_CFG1     0x04
#define CDU_CFG2     0x08
#define CDU_CFG3     0x0C
#define CDU_CFG4     0x10
#define CDU_CFG5     0x14
#define CDU_CFG6     0x18
#define CDU_CFG7     0x1C
#define CDU_CFG8     0x20
#define CDU_CFG9     0x24
#define CDU_CFG10    0x28
#define CDU_CFG11    0x2C
#define CDU_CFG12    0x30
#define CDU_CFG13    0x34
#define CDU_CFG14    0x38

#define PLL3_OFFSET 0x2c

#define CDU_CLKINSEL 0x44

#define CGU_MSEL_SHIFT 8
#define CGU_MSEL_WIDTH 7

#define PLL3_MSEL_SHIFT 4
#define PLL3_MSEL_WIDTH 7

#define CDU_MUX_SIZE 4
#define CDU_MUX_SHIFT 1
#define CDU_MUX_WIDTH 2
#define CDU_EN_BIT 0

enum sc5xx_clk_type {
	SC5XX_CLK_DIV,
	SC5XX_CLK_PLL,
	SC5XX_CLK_FFACTOR,
	SC5XX_CLK_GATE,
	SC5XX_CLK_MUX,
	SC5XX_CLK_CLKINSEL,
	SC5XX_CLK_CDU_MUX,
};

struct sc5xx_div_clock {
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	unsigned long flags;
	u32 offset;
	u8 shift;
	u8 width;
	u8 div_flags;
};

#define DIV(_id, _name, _parent, _offset, _shift, _width, _flags,	\
	    _div_flags)							\
{									\
	.type = SC5XX_CLK_DIV,						\
	.div = {							\
		.id = (_id),						\
		.name = (_name),					\
		.parent_data = (_parent),				\
		.flags = (_flags),					\
		.offset = (_offset),					\
		.shift = (_shift),					\
		.width = (_width),					\
		.div_flags = (_div_flags),				\
	},								\
}

struct sc5xx_pll_clock {
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	unsigned long flags;
	u32 offset;
	u8 shift;
	u8 width;
	u32 m_offset;
	bool half_m;
};

#define PLL(_id, _name, _parent, _offset, _shift, _width,		\
	    _m_offset, _half_m, _flags)					\
{									\
	.type = SC5XX_CLK_PLL,						\
	.pll = {							\
		.id = (_id),						\
		.name = (_name),					\
		.parent_data = (_parent),				\
		.flags = (_flags),					\
		.offset = (_offset),					\
		.shift = (_shift),					\
		.width = (_width),					\
		.m_offset = (_m_offset),				\
		.half_m = (_half_m),					\
	},								\
}

struct sc5xx_fixed_factor_clock {
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	unsigned long flags;
	unsigned int mult;
	unsigned int div;
};

#define FFACTOR(_id, _name, _parent, _mult, _div, _flags)		\
{									\
	.type = SC5XX_CLK_FFACTOR,					\
	.ffactor = {							\
		.id = (_id),						\
		.name = (_name),					\
		.parent_data = (_parent),				\
		.flags = (_flags),					\
		.mult = (_mult),					\
		.div = (_div),						\
	},								\
}

struct sc5xx_gate_clock {
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	unsigned long flags;
	u32 offset;
	u8 bit_idx;
	u8 gate_flags;
};

#define GATE(_id, _name, _parent, _offset, _bit_idx, _flags,		\
	     _gate_flags)						\
{									\
	.type = SC5XX_CLK_GATE,						\
	.gate = {							\
		.id = (_id),						\
		.name = (_name),					\
		.parent_data = (_parent),				\
		.flags = (_flags),					\
		.offset = (_offset),					\
		.bit_idx = (_bit_idx),					\
		.gate_flags = (_gate_flags),				\
	},								\
}

struct sc5xx_mux_clock {
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	u8 num_parents;
	const u32 *mux_table;
	u8 cdu_clko;
	unsigned long flags;
};

#define MUX(_id, _name, _parents, _clko, _table, _flags)		\
{									\
	.type = SC5XX_CLK_MUX,						\
	.mux = {							\
		.id = (_id),						\
		.name = (_name),					\
		.parent_data = (_parents),				\
		.num_parents = ARRAY_SIZE(_parents),			\
		.mux_table = (_table),					\
		.cdu_clko = (_clko),					\
		.flags = (_flags),					\
	},								\
}

#define CMUX(_id, _name, _parents, _clko, _table, _flags)		\
	MUX(_id, _name, _parents, _clko, _table,			\
	    (_flags) | CLK_IS_CRITICAL)

struct sc5xx_clkinsel_clock {
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	u8 num_parents;
	unsigned long flags;
};

#define CLKINSEL(_id, _name, _parents, _flags)				\
{									\
	.type = SC5XX_CLK_CLKINSEL,					\
	.clkinsel = {							\
		.id = (_id),						\
		.name = (_name),					\
		.parent_data = (_parents),				\
		.num_parents = ARRAY_SIZE(_parents),			\
		.flags = (_flags),					\
	},								\
}

struct sc5xx_cdu_mux {
	unsigned int id;
	const char *name;
	u8 cdu_clko;
	const struct clk_parent_data *parent_data;
	const u32 *mux_table;
	u8 num_parents;
	unsigned long flags;
};

#define CDU_MUX(_id, _name, _parents, _cdu_clko, _mux_table, _flags)	\
{									\
	.type = SC5XX_CLK_CDU_MUX,					\
	.cdu_mux = {							\
		.id = (_id),						\
		.name = (_name),					\
		.cdu_clko = (_cdu_clko),				\
		.parent_data = (_parents),				\
		.mux_table = (_mux_table),				\
		.num_parents = ARRAY_SIZE(_parents),			\
		.flags = (_flags),					\
	},								\
}

#define CDU_CMUX(_id, _name, _parents, _cdu_clko, _mux_table, _flags)	\
{									\
	.type = SC5XX_CLK_CDU_MUX,					\
	.cdu_mux = {							\
		.id = (_id),						\
		.name = (_name),					\
		.cdu_clko = (_cdu_clko),				\
		.parent_data = (_parents),				\
		.mux_table = (_mux_table),				\
		.num_parents = ARRAY_SIZE(_parents),			\
		.flags = (_flags) | (CLK_IS_CRITICAL),			\
	},								\
}

struct sc5xx_clk {
	enum sc5xx_clk_type type;

	union {
		struct sc5xx_div_clock div;
		struct sc5xx_pll_clock pll;
		struct sc5xx_fixed_factor_clock ffactor;
		struct sc5xx_gate_clock gate;
		struct sc5xx_mux_clock mux;
		struct sc5xx_clkinsel_clock clkinsel;
		struct sc5xx_cdu_mux cdu_mux;
	};
};

struct clk *sc5xx_cdu_register(const char *clock_name, void __iomem *base,
			       u8 cdu_clko,
			       const char * const *parent_names,
			       const u32 *parent_sel,
			       u8 num_parents,
			       unsigned long clock_flags,
			       spinlock_t *lock);

void sc5xx_cdu_print_revision(const char *soc_name, void __iomem *base);

struct clk_sc5xx_cgu_pll *to_clk_sc5xx_cgu_pll(struct clk_hw *hw);

struct clk *sc5xx_cgu_pll(const char *name, const char *parent_name,
			  void __iomem *base, u8 shift, u8 width,
			  u32 m_offset, bool half_m, spinlock_t *lock);

/**
 * All CDU clock muxes are the same size
 */
static inline struct clk *cdu_mux(const char *name, void __iomem *reg,
				  const char *const *parents,
				  spinlock_t *cdu_lock)
{
	return clk_register_mux(NULL, name, parents, CDU_MUX_SIZE,
				CLK_SET_RATE_PARENT, reg, CDU_MUX_SHIFT,
				CDU_MUX_WIDTH, 0, cdu_lock);
}

static inline struct clk *cgu_divider(const char *name, const char *parent,
				      void __iomem *reg, u8 shift,
				      u8 width, u8 extra_flags,
				      spinlock_t *cdu_lock)
{
	return clk_register_divider(NULL, name, parent,
				    CLK_SET_RATE_PARENT, reg, shift, width,
				    CLK_DIVIDER_MAX_AT_ZERO | extra_flags,
				    cdu_lock);
}

static inline struct clk *cdu_gate(const char *name, const char *parent,
				   void __iomem *reg, u32 flags,
				   spinlock_t *cdu_lock)
{
	return clk_register_gate(NULL, name, parent,
				 CLK_SET_RATE_PARENT | flags, reg,
				 CDU_EN_BIT, 0, cdu_lock);
}

static inline struct clk *cgu_gate(const char *name, const char *parent,
				   void __iomem *reg, u8 bit,
				   spinlock_t *cdu_lock)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT,
				 reg, bit, CLK_GATE_SET_TO_DISABLE,
				 cdu_lock);
}

static inline int cdu_check_clocks(struct clk *clks[], size_t count)
{
	size_t i;

	for (i = 0; i < count; ++i) {
		if (IS_ERR(clks[i])) {
			pr_err("Clock %zu failed to register: %ld\n", i,
			       PTR_ERR(clks[i]));
			return PTR_ERR(clks[i]);
		}
	}

	return 0;
}

#endif
