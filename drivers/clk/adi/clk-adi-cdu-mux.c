// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock Distribution Unit driver for ADSP-SC5xx processors
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

#define SC5XX_CDU_CFG(n)	((n) * sizeof(u32))

#define SC5XX_CDU_STAT		0x40
#define SC5XX_CDU_REVID		0x48

#define SC5XX_CDU_CFG_EN	BIT(0)
#define SC5XX_CDU_CFG_LOCK	BIT(31)
#define SC5XX_CDU_CFG_SEL	GENMASK(2, 1)

#define SC5XX_CDU_REVID_MAJOR	GENMASK(7, 4)
#define SC5XX_CDU_REVID_REV	GENMASK(3, 0)

#define SC5XX_CDU_POLL_DELAY	2
#define SC5XX_CDU_POLL_TIMEOUT	10

struct sc5xx_cdu {
	u8 cdu_clko;
	spinlock_t *lock;
	void __iomem *base;
	struct clk_hw clk_hw;
	const u32 *parent_sel;
};

enum sc5xx_cdu_en_state {
	SC5XX_CDU_EN_DISABLE,
	SC5XX_CDU_EN_ENABLE,
};

void sc5xx_cdu_print_revision(const char *soc_name, void __iomem *base)
{
	u32 revid = readl(base + SC5XX_CDU_REVID);

	pr_info("%s CDU revision: major=%u rev=%u (0x%08x)\n",
		soc_name,
		FIELD_GET(SC5XX_CDU_REVID_MAJOR, revid),
		FIELD_GET(SC5XX_CDU_REVID_REV, revid),
		revid);
}

static inline struct sc5xx_cdu *to_sc5xx_cdu(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct sc5xx_cdu, clk_hw);
}

static int sc5xx_cdu_check_unlocked(u32 reg)
{
	if (reg & SC5XX_CDU_CFG_LOCK)
		return -EBUSY;

	return 0;
}

static int sc5xx_cdu_wait_ready(struct sc5xx_cdu *cdu_clk)
{
	u32 stat;

	return readl_poll_timeout_atomic(cdu_clk->base + SC5XX_CDU_STAT, 
			stat, !(stat & BIT(cdu_clk->cdu_clko)),
			SC5XX_CDU_POLL_DELAY, 
			SC5XX_CDU_POLL_TIMEOUT);
}

static u32 sc5xx_cdu_read(struct sc5xx_cdu *cdu_clk, unsigned int offset)
{
	return readl(cdu_clk->base + offset);
}

static void sc5xx_cdu_write(struct sc5xx_cdu *cdu_clk, 
			    unsigned int offset, u32 val)
{
	writel(val, cdu_clk->base + offset);
}

static unsigned int sc5xx_cdu_cfg(struct sc5xx_cdu *cdu_clk)
{
	return SC5XX_CDU_CFG(cdu_clk->cdu_clko);
}

/* Caller must hold cdu_clk->lock */
static int sc5xx_cdu_update_en(struct sc5xx_cdu *cdu_clk, 
			       enum sc5xx_cdu_en_state state)
{
	u32 reg;
	int ret;

	reg = sc5xx_cdu_read(cdu_clk, sc5xx_cdu_cfg(cdu_clk));
	
	ret = sc5xx_cdu_check_unlocked(reg);
	if (ret)
		return ret;

	if (state == SC5XX_CDU_EN_ENABLE)
		reg |= SC5XX_CDU_CFG_EN;
	else
		reg &= ~SC5XX_CDU_CFG_EN;
	
	sc5xx_cdu_write(cdu_clk, sc5xx_cdu_cfg(cdu_clk), reg);
	
	return 0;
}

static int sc5xx_cdu_set_parent(struct clk_hw *clk_hw, u8 index)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	unsigned int input_sel;
	u32 reg;
	int ret;

	input_sel = clk_mux_index_to_val(cdu_clk->parent_sel, 0, index);

	spin_lock_irqsave(cdu_clk->lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_clk);
	if (ret)
		goto out;

	reg = sc5xx_cdu_read(cdu_clk, sc5xx_cdu_cfg(cdu_clk));

	ret = sc5xx_cdu_check_unlocked(reg);
	if (ret)
		goto out;

	reg &= ~SC5XX_CDU_CFG_SEL;
	reg |= FIELD_PREP(SC5XX_CDU_CFG_SEL, input_sel);

	sc5xx_cdu_write(cdu_clk, sc5xx_cdu_cfg(cdu_clk), reg);

	ret = sc5xx_cdu_wait_ready(cdu_clk);
	
out:
	spin_unlock_irqrestore(cdu_clk->lock, flags);
	
	return ret;
}

static u8 sc5xx_cdu_get_parent(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	u32 reg, input_sel;

	spin_lock_irqsave(cdu_clk->lock, flags);
	reg = sc5xx_cdu_read(cdu_clk, sc5xx_cdu_cfg(cdu_clk));
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	input_sel = FIELD_GET(SC5XX_CDU_CFG_SEL, reg);

	return clk_mux_val_to_index(clk_hw, cdu_clk->parent_sel, 0, input_sel);
}

static int sc5xx_cdu_enable(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(cdu_clk->lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_clk);
	if (ret)
		goto out;

	ret = sc5xx_cdu_update_en(cdu_clk, SC5XX_CDU_EN_ENABLE);
	if (ret)
		goto out;

	ret = sc5xx_cdu_wait_ready(cdu_clk);

out:
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	return ret;
}

static void sc5xx_cdu_disable(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(cdu_clk->lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_clk);
	if (ret)
		goto out;

	ret = sc5xx_cdu_update_en(cdu_clk, SC5XX_CDU_EN_DISABLE);
	if (ret)
		goto out;

	sc5xx_cdu_wait_ready(cdu_clk);

out:
	spin_unlock_irqrestore(cdu_clk->lock, flags);
}

static int sc5xx_cdu_is_enabled(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	u32 reg;
	
	spin_lock_irqsave(cdu_clk->lock, flags);
	reg = sc5xx_cdu_read(cdu_clk, sc5xx_cdu_cfg(cdu_clk));
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	return reg & SC5XX_CDU_CFG_EN;
}

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>

ssize_t sc5xx_cdu_debug_read(struct file *, char __user *, size_t, loff_t *)
{

}

static void sc5xx_cdu_debug_init(struct clk_hw *clk_hw, struct dentry *dentry)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	char debugfs_entry_name[12];

	snprintf(debugfs_entry_name, sizeof(debugfs_entry_name), 
		"cdu_cfg[%u]", cdu_clk->cdu_clko);

	debugfs_create_file(debugfs_entry_name, 0444, dentry, clk_hw, )
}

static const struct file_operations sc5xx_cdu_debug_fops = {
	.owner = THIS_MODULE,
	.read  = sc5xx_cdu_debug_read,
};
#endif

static const struct clk_ops sc5xx_cdu_ops = {
	.set_parent = sc5xx_cdu_set_parent,
	.get_parent = sc5xx_cdu_get_parent,
	.enable     = sc5xx_cdu_enable,
	.disable    = sc5xx_cdu_disable,
	.is_enabled = sc5xx_cdu_is_enabled,
#ifdef CONFIG_DEBUG_FS
	.debug_init = sc5xx_cdu_debug_init,
#endif
};

struct clk *sc5xx_cdu_register(const char *clock_name, void __iomem *base,
				u8 cdu_clko, const char * const *parent_names,
				const u32 *parent_sel, u8 num_parents, 
				unsigned long clock_flags, spinlock_t *lock)
{
	struct sc5xx_cdu *cdu_clk;
	struct clk_init_data init = { };
	struct clk *clk;

	cdu_clk = kzalloc(sizeof(*cdu_clk), GFP_KERNEL);
	if (!cdu_clk)
		return ERR_PTR(-ENOMEM);

	init.name = clock_name;
	init.ops = &sc5xx_cdu_ops;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	init.flags = clock_flags;

	cdu_clk->clk_hw.init = &init;
	cdu_clk->base = base;
	cdu_clk->lock = lock;
	cdu_clk->parent_sel = parent_sel; 
	cdu_clk->cdu_clko = cdu_clko; 

	clk = clk_register(NULL, &cdu_clk->clk_hw);
	if (IS_ERR(clk))
		kfree(cdu_clk);

	return clk;
}

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices Inc ADSP-SC5XX Clock Distribution Unit (CDU) driver");
MODULE_LICENSE("GPL v2");
