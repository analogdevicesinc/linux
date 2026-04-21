/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2026, Beijing ESWIN Computing Technology Co., Ltd..
 * All rights reserved.
 *
 * Authors:
 *	Yifeng Huang <huangyifeng@eswincomputing.com>
 *	Xuyang Dong <dongxuyang@eswincomputing.com>
 */

#ifndef __ESWIN_COMMON_H__
#define __ESWIN_COMMON_H__

#define APLL_HIGH_FREQ	983040000
#define APLL_LOW_FREQ	225792000
#define PLL_HIGH_FREQ	1800000000
#define PLL_LOW_FREQ	24000000

/*
 * ESWIN_PRIV_DIV_MIN_2: If ESWIN_PRIV_DIV_MIN_2 is set, the minimum value of
 *	the register is 2, i.e. the minimum division ratio is 2.
 */
#define ESWIN_PRIV_DIV_MIN_2	BIT(0)

enum eswin_clk_type {
	CLK_FIXED_FACTOR,
	CLK_MUX,
	CLK_DIVIDER,
	CLK_GATE,
};

struct eswin_clock_data {
	void __iomem *base;
	struct clk_hw *original_clk;
	struct notifier_block pll_nb;
	spinlock_t lock; /* protect register read-modify-write cycle */
	struct clk_hw_onecell_data clk_data;
};

struct eswin_divider_clock {
	struct clk_hw hw;
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	void __iomem *ctrl_reg; /* register address of the divider clock */
	unsigned long flags;
	unsigned long reg; /* register offset */
	u8 shift;
	u8 width;
	unsigned long div_flags;
	unsigned long priv_flag;
	spinlock_t *lock; /* protect register read-modify-write cycle */
};

struct eswin_fixed_rate_clock {
	struct clk_hw hw;
	unsigned int id;
	const char *name;
	unsigned long flags;
	unsigned long rate;
};

struct eswin_fixed_factor_clock {
	struct clk_hw hw;
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	unsigned long mult;
	unsigned long div;
	unsigned long flags;
};

struct eswin_gate_clock {
	struct clk_hw hw;
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	unsigned long flags;
	unsigned long reg;
	u8 bit_idx;
	u8 gate_flags;
};

struct eswin_mux_clock {
	struct clk_hw hw;
	unsigned int id;
	const char *name;
	const struct clk_parent_data *parent_data;
	u8 num_parents;
	unsigned long flags;
	unsigned long reg;
	u8 shift;
	u8 width;
	u8 mux_flags;
	u32 *table;
};

struct eswin_pll_clock {
	struct clk_hw hw;
	u32 id;
	const char *name;
	const struct clk_parent_data *parent_data;
	const u32 ctrl_reg0;
	const u8 fbdiv_shift;

	const u32 ctrl_reg1;
	const u8 frac_shift;

	const u32 ctrl_reg2;

	const u32 status_reg;
	const u8 lock_shift;
	const u8 lock_width;

	const u64 max_rate;
	const u64 min_rate;
};

struct eswin_clk_pll {
	struct clk_hw hw;
	u32 id;
	void __iomem *ctrl_reg0;
	u8 fbdiv_shift;

	void __iomem *ctrl_reg1;
	u8 frac_shift;

	void __iomem *ctrl_reg2;

	void __iomem *status_reg;
	u8 lock_shift;
	u8 lock_width;

	u64 max_rate;
	u64 min_rate;
};

struct eswin_clk_info {
	unsigned int type;
	unsigned int pid;
	unsigned int id;
	struct clk_hw hw;
	union {
		struct eswin_divider_clock div;
		struct eswin_fixed_factor_clock factor;
		struct eswin_gate_clock gate;
		struct eswin_mux_clock mux;
	} data;
};

struct eswin_clock_data *eswin_clk_init(struct platform_device *pdev,
					size_t nr_clks);
int eswin_clk_register_fixed_rate(struct device *dev,
				  struct eswin_fixed_rate_clock *clks,
				  int nums, struct eswin_clock_data *data);
int eswin_clk_register_pll(struct device *dev, struct eswin_pll_clock *clks,
			   int nums, struct eswin_clock_data *data);
int eswin_clk_register_fixed_factor(struct device *dev,
				    struct eswin_fixed_factor_clock *clks,
				    int nums, struct eswin_clock_data *data);
int eswin_clk_register_mux(struct device *dev, struct eswin_mux_clock *clks,
			   int nums, struct eswin_clock_data *data);
int eswin_clk_register_divider(struct device *dev,
			       struct eswin_divider_clock *clks,
			       int nums, struct eswin_clock_data *data);
int eswin_clk_register_gate(struct device *dev, struct eswin_gate_clock *clks,
			    int nums, struct eswin_clock_data *data);
int eswin_clk_register_clks(struct device *dev, struct eswin_clk_info *clks,
			    int nums, struct eswin_clock_data *data);
struct clk_hw *eswin_register_clkdiv(struct device *dev, unsigned int id,
				     const char *name,
				     const struct clk_hw *parent_hw,
				     unsigned long flags, void __iomem *reg,
				     u8 shift, u8 width,
				     unsigned long clk_divider_flags,
				     unsigned long priv_flag, spinlock_t *lock);

#define ESWIN_DIV(_id, _name, _pdata, _flags, _reg, _shift, _width,	\
		  _dflags, _pflag)					\
	{								\
		.id		= _id,					\
		.name		= _name,				\
		.parent_data	= _pdata,				\
		.flags		= _flags,				\
		.reg		= _reg,					\
		.shift		= _shift,				\
		.width		= _width,				\
		.div_flags	= _dflags,				\
		.priv_flag	= _pflag,				\
	}

#define ESWIN_DIV_TYPE(_id, _name, _pid, _flags, _reg, _shift, _width,	\
		       _dflags, _pflag)					\
	{								\
		.type	= CLK_DIVIDER,					\
		.pid	= _pid,						\
		.id	= _id,						\
		.data	= {						\
				.div = {				\
					.name	= _name,		\
					.flags		= _flags,	\
					.reg		= _reg,		\
					.shift		= _shift,	\
					.width		= _width,	\
					.div_flags	= _dflags,	\
					.priv_flag	= _pflag,	\
				},					\
		},							\
	}

#define ESWIN_FACTOR(_id, _name, _pdata, _mult, _div, _flags)		\
	{								\
		.id		= _id,					\
		.name		= _name,				\
		.parent_data	= _pdata,				\
		.mult		= _mult,				\
		.div		= _div,					\
		.flags		= _flags,				\
	}

#define ESWIN_FACTOR_TYPE(_id, _name, _pid, _mult, _div, _flags)	\
	{								\
		.type	= CLK_FIXED_FACTOR,				\
		.pid	= _pid,						\
		.id	= _id,						\
		.data	= {						\
				.factor = {				\
					    .name	= _name,	\
					    .mult	= _mult,	\
					    .div	= _div,		\
					    .flags	= _flags,	\
				},					\
		},							\
	}

#define ESWIN_FIXED(_id, _name, _flags, _rate)				\
	{								\
		.id		= _id,					\
		.name		= _name,				\
		.flags		= _flags,				\
		.rate		= _rate,				\
	}

#define ESWIN_GATE(_id, _name, _pdata, _flags, _reg, _idx, _gflags)	\
	{								\
		.id		= _id,					\
		.name		= _name,				\
		.parent_data	= _pdata,				\
		.flags		= _flags,				\
		.reg		= _reg,					\
		.bit_idx	= _idx,					\
		.gate_flags	= _gflags,				\
	}

#define ESWIN_GATE_TYPE(_id, _name, _pid, _flags, _reg, _idx, _gflags)	\
	{								\
		.type	= CLK_GATE,					\
		.pid	= _pid,						\
		.id	= _id,						\
		.data	= {						\
				.gate = {				\
					.name	= _name,		\
					.flags		= _flags,	\
					.reg		= _reg,		\
					.bit_idx	= _idx,		\
					.gate_flags	= _gflags,	\
				},					\
		},							\
	}

#define ESWIN_MUX(_id, _name, _pdata, _num_parents, _flags, _reg,	\
		  _shift, _width, _mflags)				\
	{								\
		.id		= _id,					\
		.name		= _name,				\
		.parent_data	= _pdata,				\
		.num_parents	= _num_parents,				\
		.flags		= _flags,				\
		.reg		= _reg,					\
		.shift		= _shift,				\
		.width		= _width,				\
		.mux_flags	= _mflags,				\
		.table		= NULL,					\
	}

#define ESWIN_MUX_TBL(_id, _name, _pdata, _num_parents, _flags, _reg,	\
		      _shift, _width, _mflags, _table)			\
	{								\
		.id		= _id,					\
		.name		= _name,				\
		.parent_data	= _pdata,				\
		.num_parents	= _num_parents,				\
		.flags		= _flags,				\
		.reg		= _reg,					\
		.shift		= _shift,				\
		.width		= _width,				\
		.mux_flags	= _mflags,				\
		.table		= _table,				\
	}

#define ESWIN_MUX_TYPE(_id, _name, _pdata, _num_parents, _flags, _reg,	\
		       _shift, _width, _mflags, _table)			\
	{								\
		.type	= CLK_MUX,					\
		.id	= _id,						\
		.data	= {						\
				.mux = {				\
					.name	= _name,		\
					.parent_data	= _pdata,	\
					.num_parents	= _num_parents,	\
					.flags		= _flags,	\
					.reg		= _reg,		\
					.shift		= _shift,	\
					.width		= _width,	\
					.mux_flags	= _mflags,	\
					.table		= _table,	\
				},					\
		},							\
	}

#define ESWIN_PLL(_id, _name, _pdata, _reg0, _fb_shift, _reg1,		\
		  _frac_shift, _reg2, _reg, _lock_shift, _lock_width,	\
		  _max_rate, _min_rate)					\
	{								\
		.id		= _id,					\
		.name		= _name,				\
		.parent_data	= _pdata,				\
		.ctrl_reg0	= _reg0,				\
		.fbdiv_shift	= _fb_shift,				\
		.ctrl_reg1	= _reg1,				\
		.frac_shift	= _frac_shift,				\
		.ctrl_reg2	= _reg2,				\
		.status_reg	= _reg,					\
		.lock_shift	= _lock_shift,				\
		.lock_width	= _lock_width,				\
		.max_rate	= _max_rate,				\
		.min_rate	= _min_rate,				\
	}

#endif /* __ESWIN_COMMON_H__ */
