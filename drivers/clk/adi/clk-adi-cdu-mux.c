// SPDX-License-Identifier: GPL-2.0-only
/*
 * CDU mux clock driver for ADSP-SC5xx processors
 *
 * Copyright (C) 2026 Analog Devices Inc.
 */

#include <linux/clk-provider.h>
#include <linux/iopoll.h>
#include <linux/slab.h>

/* There are 4 selectable inputs per CDU_CFG[n] */

#define CDU_CFG_EN		BIT(0)
#define CDU_CFG_LOCK            BIT(31)
#define CDU_CFG_SEL_MASK	GENMASK(2, 1)

#define CDU_CFG_POLL_DELAY_US   1
#define CDU_CFG_POLL_TIMEOUT_US 100

struct clk_adi_cdu_mux {
	struct clk_hw hw;
	void __iomem *cdu_cfg;
	void __iomem *cdu_stat;
	const u8 *parent_sel;
	spinlock_t *cdu_lock;
	u8 clko_stat_bit;
};

static inline struct clk_adi_cdu_mux *to_clk_adi_cdu_mux(struct clk_hw *hw)
{
	return container_of(hw, struct clk_adi_cdu_mux, hw);
}

static int sc5xx_cdu_wait_ready(struct clk_adi_cdu_mux *cdu_mux)
{
	u32 stat;

	return readl_poll_timeout_atomic(cdu_mux->cdu_stat, stat,
					 !(stat & BIT(cdu_mux->clko_stat_bit)),
					 CDU_CFG_POLL_DELAY_US,
					 CDU_CFG_POLL_TIMEOUT_US);
}

static int sc5xx_cdu_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	unsigned long flags;
        u8 input_sel;
	u32 reg;
	int ret;

	input_sel = cdu_mux->parent_sel[index];
	spin_lock_irqsave(cdu_mux->cdu_lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_mux);
	if (ret)
		return ret;

	reg = readl(cdu_mux->cdu_cfg);
	if (reg & CDU_CFG_LOCK) {
		spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);
		return -EPERM;
	} 	

	reg &= ~CDU_CFG_SEL_MASK;
	reg |= FIELD_PREP(CDU_CFG_SEL_MASK, input_sel);

	writel(reg, cdu_mux->cdu_cfg);

	spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);

	return sc5xx_cdu_wait_ready(cdu_mux);	
}

static u8 sc5xx_cdu_get_parent(struct clk_hw *hw)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	unsigned long flags;
	unsigned int num_parents;
	u8 index, input_sel;
	u32 reg;

	spin_lock_irqsave(cdu_mux->cdu_lock, flags);
	reg = readl(cdu_mux->cdu_cfg);
	spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);

	input_sel = FIELD_GET(CDU_CFG_SEL_MASK, reg);
	num_parents = clk_hw_get_num_parents(hw);

	for (index = 0; index < num_parents; index++) {
		if (cdu_mux->parent_sel[index] == input_sel)
			return index;
	}

	return 0;
}

static int sc5xx_cdu_enable(struct clk_hw *hw)
{
		
}

static void sc5xx_cdu_disable(struct clk_hw *hw)
{
}

static int sc5xx_cdu_is_enabled(struct clk_hw *hw)
{
}

static const struct clk_ops adi_cdu_mux_ops = {
	.set_parent = sc5xx_cdu_set_parent,
	.get_parent = sc5xx_cdu_get_parent,
	.enable     = sc5xx_cdu_enable,
	.disable    = sc5xx_cdu_disable,
	.is_enabled = sc5xx_cdu_is_enabled,
};

struct clk *sc5xx_adi_cdu_mux(const char *clock_name, void __iomem *cdu_cfg,
			      const char * const *parent_names, u8 cdu_cfg_sel_inputs,
			      unsigned long clock_flags, spinlock_t *cdu_lock)
{
	struct clk_adi_cdu_mux *cdu_mux;
	struct clk_init_data init = {};
	struct clk *clk;

	cdu_mux = kzalloc(sizeof(*cdu_mux), GFP_KERNEL);
	if (!cdu_mux)
		return ERR_PTR(-ENOMEM);

	init.name = clock_name;
	init.ops = &adi_cdu_mux_ops;
	init.parent_names = parent_names;
	init.num_parents = cdu_cfg_sel_inputs;
	init.flags = clock_flags;

	cdu_mux->hw.init = &init;
	cdu_mux->cdu_cfg = cdu_cfg;
	cdu_mux->cdu_lock = cdu_lock;

	clk = clk_register(NULL, &cdu_mux->hw);
	if (IS_ERR(clk)) {
		kfree(cdu_mux);
		return clk;
	}

	return clk;
}

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices Inc ADSP-SC5XX Clock Distribution Unit (CDU) driver");
MODULE_LICENSE("GPL v2");
