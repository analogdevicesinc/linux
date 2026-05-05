// SPDX-License-Identifier: GPL-2.0-only
/*
 * CDU multiplexer clock driver for ADSP-SC5xx processors
 *
 * Copyright (C) 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define CDU_CFG_EN		BIT(0)
#define CDU_CFG_LOCK            BIT(31)
#define CDU_CFG_SEL_MASK	GENMASK(2, 1)
#define CDU_REVID_MAJOR		GENMASK(7, 4)
#define CDU_REVID_REV		GENMASK(3, 0)
#define CDU_REVID_OFFSET	0x48

#define CDU_CFG_POLL_DELAY_US   2
#define CDU_CFG_POLL_TIMEOUT_US 10

struct clk_adi_cdu_mux {
	struct clk_hw hw;
	u8 clko_stat_bit;
	spinlock_t *cdu_lock;
	const u32 *parent_sel;
	void __iomem *cdu_cfg;
	void __iomem *cdu_stat;
};

enum sc5xx_cdu_en_state {
	SC5XX_CDU_EN_DISABLE,
	SC5XX_CDU_EN_ENABLE,
};

static inline struct clk_adi_cdu_mux *to_clk_adi_cdu_mux(struct clk_hw *hw)
{
	return container_of(hw, struct clk_adi_cdu_mux, hw);
}

static int sc5xx_cdu_cfg_check_unlocked(u32 reg)
{
	if (reg & CDU_CFG_LOCK)
		return -EBUSY;

	return 0;
}

static int sc5xx_cdu_wait_ready(struct clk_adi_cdu_mux *cdu_mux)
{
	u32 stat;

	return readl_poll_timeout_atomic(cdu_mux->cdu_stat, stat,
			!(stat & BIT(cdu_mux->clko_stat_bit)),
			CDU_CFG_POLL_DELAY_US,
			CDU_CFG_POLL_TIMEOUT_US);
}

static inline u32 sc5xx_cdu_read_cfg(struct clk_adi_cdu_mux *cdu_mux)
{
	return readl(cdu_mux->cdu_cfg);
}

static inline void sc5xx_cdu_write_cfg(struct clk_adi_cdu_mux *cdu_mux, u32 reg)
{
	writel(reg, cdu_mux->cdu_cfg);
}

void sc5xx_cdu_print_revision(const char *soc_name, void __iomem *cdu)
{
	u32 revid = readl(cdu + CDU_REVID_OFFSET);

	pr_info("%s CDU revision: major=%u rev=%u (0x%08x)\n",
		soc_name,
		FIELD_GET(CDU_REVID_MAJOR, revid),
		FIELD_GET(CDU_REVID_REV, revid),
		revid);
}

/* Caller must hold cdu_mux->cdu_lock */
static int sc5xx_cdu_cfg_update_en(struct clk_adi_cdu_mux *cdu_mux, 
					enum sc5xx_cdu_en_state state)
{
	u32 reg;
	int ret;

	reg = sc5xx_cdu_read_cfg(cdu_mux);

	ret = sc5xx_cdu_cfg_check_unlocked(reg);
	if (ret)
		return ret;

	if (state == SC5XX_CDU_EN_ENABLE)
		reg |= CDU_CFG_EN;
	else
		reg &= ~CDU_CFG_EN;

	sc5xx_cdu_write_cfg(cdu_mux, reg);

	return 0;
}

static int sc5xx_cdu_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	unsigned long flags;
	unsigned int input_sel;
	u32 reg;
	int ret;

	input_sel = clk_mux_index_to_val(cdu_mux->parent_sel, 0, index);

	spin_lock_irqsave(cdu_mux->cdu_lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_mux);
	if (ret)
		goto out;

	reg = sc5xx_cdu_read_cfg(cdu_mux);

	ret = sc5xx_cdu_cfg_check_unlocked(reg);
	if (ret)
		goto out;

	reg &= ~CDU_CFG_SEL_MASK;
	reg |= FIELD_PREP(CDU_CFG_SEL_MASK, input_sel);

	sc5xx_cdu_write_cfg(cdu_mux, reg);

	ret = sc5xx_cdu_wait_ready(cdu_mux);
	
out:
	spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);
	
	return ret;
}

static u8 sc5xx_cdu_get_parent(struct clk_hw *hw)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	unsigned long flags;
	u32 reg, input_sel;

	spin_lock_irqsave(cdu_mux->cdu_lock, flags);
	reg = sc5xx_cdu_read_cfg(cdu_mux);
	spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);

	input_sel = FIELD_GET(CDU_CFG_SEL_MASK, reg);

	return clk_mux_val_to_index(hw, cdu_mux->parent_sel, 0, input_sel);
}

static int sc5xx_cdu_enable(struct clk_hw *hw)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(cdu_mux->cdu_lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_mux);
	if (ret)
		goto out;

	ret = sc5xx_cdu_cfg_update_en(cdu_mux, SC5XX_CDU_EN_ENABLE);
	if (ret)
		goto out;

	ret = sc5xx_cdu_wait_ready(cdu_mux);

out:
	spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);

	return ret;
}

static void sc5xx_cdu_disable(struct clk_hw *hw)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(cdu_mux->cdu_lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_mux);
	if (ret)
		goto out;

	ret = sc5xx_cdu_cfg_update_en(cdu_mux, SC5XX_CDU_EN_DISABLE);
	if (ret)
		goto out;

	sc5xx_cdu_wait_ready(cdu_mux);

out:
	spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);
}

static int sc5xx_cdu_is_enabled(struct clk_hw *hw)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	unsigned long flags;
	u32 reg;
	
	spin_lock_irqsave(cdu_mux->cdu_lock, flags);
	reg = sc5xx_cdu_read_cfg(cdu_mux);
	spin_unlock_irqrestore(cdu_mux->cdu_lock, flags);

	return reg & CDU_CFG_EN;
}

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>

ssize_t sc5xx_cdu_debug_read(struct file *, char __user *, size_t, loff_t *)
{

}

static void sc5xx_cdu_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct clk_adi_cdu_mux *cdu_mux = to_clk_adi_cdu_mux(hw);
	char debugfs_entry_name[12];

	snprintf(debugfs_entry_name, sizeof(debugfs_entry_name), 
		"cdu_cfg[%u]", cdu_mux->clko_stat_bit);

	debugfs_create_file(debugfs_entry_name, 0444, dentry, hw, )
}

static const struct file_operations sc5xx_cdu_debug_fops = {
	.owner = THIS_MODULE,
	.read  = sc5xx_cdu_debug_read,
};
#endif

static const struct clk_ops adi_cdu_mux_ops = {
	.set_parent = sc5xx_cdu_set_parent,
	.get_parent = sc5xx_cdu_get_parent,
	.enable     = sc5xx_cdu_enable,
	.disable    = sc5xx_cdu_disable,
	.is_enabled = sc5xx_cdu_is_enabled,
#ifdef CONFIG_DEBUG_FS
	.debug_init = sc5xx_cdu_debug_init,
#endif
};

struct clk *sc5xx_adi_cdu_mux(const char *clock_name, void __iomem *cdu_cfg,
			      void __iomem *cdu_stat, u8 clko_stat_bit,
			      const char * const *parent_names, const u32 *parent_sel, 
			      u8 num_parents, unsigned long clock_flags, 
			      spinlock_t *cdu_lock)
{
	struct clk_adi_cdu_mux *cdu_mux;
	struct clk_init_data init = { };
	struct clk *clk;

	cdu_mux = kzalloc(sizeof(*cdu_mux), GFP_KERNEL);
	if (!cdu_mux)
		return ERR_PTR(-ENOMEM);

	init.name = clock_name;
	init.ops = &adi_cdu_mux_ops;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	init.flags = clock_flags;

	cdu_mux->hw.init = &init;
	cdu_mux->cdu_cfg = cdu_cfg;
	cdu_mux->cdu_stat = cdu_stat;
	cdu_mux->parent_sel = parent_sel; 
	cdu_mux->cdu_lock = cdu_lock;
	cdu_mux->clko_stat_bit = clko_stat_bit; 

	clk = clk_register(NULL, &cdu_mux->hw);
	if (IS_ERR(clk))
		kfree(cdu_mux);

	return clk;
}

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices Inc ADSP-SC5XX Clock Distribution Unit (CDU) driver");
MODULE_LICENSE("GPL v2");
