/*
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/sizes.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>

#include "video/imx-dcss.h"
#include "dcss-prv.h"

#define DCSS_CTXLD_DEVNAME		"dcss_ctxld"

#define DCSS_CTXLD_CONTROL_STATUS	0x0
#define   CTXLD_ENABLE			BIT(0)
#define   ARB_SEL			BIT(1)
#define   RD_ERR_EN			BIT(2)
#define   DB_COMP_EN			BIT(3)
#define   SB_HP_COMP_EN			BIT(4)
#define   SB_LP_COMP_EN			BIT(5)
#define   DB_PEND_SB_REC_EN		BIT(6)
#define   SB_PEND_DISP_ACTIVE_EN	BIT(7)
#define   AHB_ERR_EN			BIT(8)
#define   RD_ERR			BIT(16)
#define   DB_COMP			BIT(17)
#define   SB_HP_COMP			BIT(18)
#define   SB_LP_COMP			BIT(19)
#define   DB_PEND_SB_REC		BIT(20)
#define   SB_PEND_DISP_ACTIVE		BIT(21)
#define   AHB_ERR			BIT(22)
#define DCSS_CTXLD_DB_BASE_ADDR		0x10
#define DCSS_CTXLD_DB_COUNT		0x14
#define DCSS_CTXLD_SB_BASE_ADDR		0x18
#define DCSS_CTXLD_SB_COUNT		0x1C
#define   SB_HP_COUNT_POS		0
#define   SB_HP_COUNT_MASK		0xffff
#define   SB_LP_COUNT_POS		16
#define   SB_LP_COUNT_MASK		0xffff0000
#define DCSS_AHB_ERR_ADDR		0x20

#define CTXLD_IRQ_NAME			"ctx_ld" /* irq steer irq name */
#define CTXLD_IRQ_COMPLETION		(DB_COMP | SB_HP_COMP | SB_LP_COMP)
#define CTXLD_IRQ_ERROR			(RD_ERR | DB_PEND_SB_REC | \
					 SB_PEND_DISP_ACTIVE | AHB_ERR)

/* The following sizes are in entries, 8 bytes each */
#define CTXLD_DB_CTX_ENTRIES		1024	/* max 65536 */
#define CTXLD_SB_LP_CTX_ENTRIES		10240	/* max 65536 */
#define CTXLD_SB_HP_CTX_ENTRIES		20000	/* max 65536 */
#define CTXLD_SB_CTX_ENTRIES		(CTXLD_SB_LP_CTX_ENTRIES + \
					 CTXLD_SB_HP_CTX_ENTRIES)

static struct dcss_debug_reg ctxld_debug_reg[] = {
	DCSS_DBG_REG(DCSS_CTXLD_CONTROL_STATUS),
	DCSS_DBG_REG(DCSS_CTXLD_DB_BASE_ADDR),
	DCSS_DBG_REG(DCSS_CTXLD_DB_COUNT),
	DCSS_DBG_REG(DCSS_CTXLD_SB_BASE_ADDR),
	DCSS_DBG_REG(DCSS_CTXLD_SB_COUNT),
	DCSS_DBG_REG(DCSS_AHB_ERR_ADDR),
};

/* Sizes, in entries, of the DB, SB_HP and SB_LP context regions. */
static u16 dcss_ctxld_ctx_size[3] = {
	CTXLD_DB_CTX_ENTRIES,
	CTXLD_SB_HP_CTX_ENTRIES,
	CTXLD_SB_LP_CTX_ENTRIES
};

/* this represents an entry in the context loader map */
struct dcss_ctxld_item {
	u32 val;
	u32 ofs;
};

#define CTX_ITEM_SIZE			sizeof(struct dcss_ctxld_item)

struct dcss_ctxld_priv {
	struct dcss_soc *dcss;
	void __iomem *ctxld_reg;
	int irq;
	bool irq_en;

	struct dcss_ctxld_item *db[2];
	struct dcss_ctxld_item *sb_hp[2];
	struct dcss_ctxld_item *sb_lp[2];

	dma_addr_t db_paddr[2];
	dma_addr_t sb_paddr[2];

	u16 ctx_size[2][3]; /* holds the sizes of DB, SB_HP and SB_LP ctx */
	u8 current_ctx;

	bool in_use;
	bool run_again;

	struct mutex mutex; /* protects concurent access to private data */
};

#ifdef CONFIG_DEBUG_FS
void dcss_ctxld_dump_regs(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	int j;

	seq_puts(s, ">> Dumping CTXLD:\n");
	for (j = 0; j < ARRAY_SIZE(ctxld_debug_reg); j++) {
		seq_printf(s, "%-35s(0x%04x) -> 0x%08x\n",
			   ctxld_debug_reg[j].name,
			   ctxld_debug_reg[j].ofs,
			   dcss_readl(dcss->ctxld_priv->ctxld_reg +
				      ctxld_debug_reg[j].ofs));
	}
}
#endif

static int __dcss_ctxld_enable(struct dcss_ctxld_priv *ctxld);

static irqreturn_t dcss_ctxld_irq_handler_thread(int irq, void *data)
{
	struct dcss_ctxld_priv *ctxld = data;
	u32 status;

	status = dcss_readl(ctxld->ctxld_reg + DCSS_CTXLD_CONTROL_STATUS);

	mutex_lock(&ctxld->mutex);
	if (!(status & CTXLD_ENABLE) && ctxld->in_use) {
		ctxld->in_use = false;

		if (ctxld->run_again) {
			__dcss_ctxld_enable(ctxld);
			ctxld->run_again = false;
			goto exit;
		}

		if (ctxld->dcss->dcss_disable_callback) {
			struct dcss_dtg_priv *dtg = ctxld->dcss->dtg_priv;

			ctxld->dcss->dcss_disable_callback(dtg);
		}
	}

exit:
	mutex_unlock(&ctxld->mutex);

	return IRQ_HANDLED;
}

static irqreturn_t dcss_ctxld_irq_handler(int irq, void *data)
{
	struct dcss_ctxld_priv *priv = data;
	u32 irq_status;

	irq_status = dcss_readl(priv->ctxld_reg + DCSS_CTXLD_CONTROL_STATUS);

	if (irq_status & CTXLD_IRQ_COMPLETION) {
		dcss_clr(irq_status & CTXLD_IRQ_COMPLETION,
			 priv->ctxld_reg + DCSS_CTXLD_CONTROL_STATUS);

		return IRQ_WAKE_THREAD;
	} else if (irq_status & CTXLD_IRQ_ERROR) {
		/*
		 * Except for throwing an error message and clearing the status
		 * register, there's not much we can do here.
		 */
		dev_err(priv->dcss->dev, "ctxld: error encountered: %08x\n",
			irq_status);
		dev_err(priv->dcss->dev, "ctxld: db=%d, sb_hp=%d, sb_lp=%d\n",
			priv->ctx_size[priv->current_ctx ^ 1][CTX_DB],
			priv->ctx_size[priv->current_ctx ^ 1][CTX_SB_HP],
			priv->ctx_size[priv->current_ctx ^ 1][CTX_SB_LP]);

		/* clear the interrupts */
		dcss_clr((irq_status & CTXLD_IRQ_ERROR),
			 priv->ctxld_reg + DCSS_CTXLD_CONTROL_STATUS);
	}

	return IRQ_HANDLED;
}

static int dcss_ctxld_irq_config(struct dcss_ctxld_priv *ctxld)
{
	struct dcss_soc *dcss = ctxld->dcss;
	struct platform_device *pdev = to_platform_device(dcss->dev);
	int ret;

	ctxld->irq = platform_get_irq_byname(pdev, CTXLD_IRQ_NAME);
	if (ctxld->irq < 0) {
		dev_err(dcss->dev, "ctxld: can't get irq number\n");
		return ctxld->irq;
	}

	ret = devm_request_threaded_irq(dcss->dev, ctxld->irq,
					dcss_ctxld_irq_handler,
					dcss_ctxld_irq_handler_thread,
					IRQF_ONESHOT | IRQF_TRIGGER_RISING,
					DCSS_CTXLD_DEVNAME, ctxld);
	if (ret) {
		dev_err(dcss->dev, "ctxld: irq request failed.\n");
		return ret;
	}

	ctxld->irq_en = true;

	return 0;
}

void dcss_ctxld_hw_cfg(struct dcss_soc *dcss)
{
	struct dcss_ctxld_priv *ctxld = dcss->ctxld_priv;

	dcss_writel(RD_ERR_EN | DB_COMP_EN | SB_HP_COMP_EN | SB_LP_COMP_EN |
		    DB_PEND_SB_REC_EN | AHB_ERR_EN | RD_ERR | AHB_ERR,
		    ctxld->ctxld_reg + DCSS_CTXLD_CONTROL_STATUS);
}

/**
 * dcss_ctxld_alloc_ctx - Allocate context memory.
 *
 * @ctxld: Pointer to ctxld.
 *
 * Returns:
 * Zeron on success, negative errno on failure.
 */
static int dcss_ctxld_alloc_ctx(struct dcss_ctxld_priv *ctxld)
{
	struct dcss_soc *dcss = ctxld->dcss;
	struct dcss_ctxld_item *ctx;
	int i;
	dma_addr_t dma_handle;

	for (i = 0; i < 2; i++) {
		ctx = dmam_alloc_coherent(dcss->dev,
					  CTXLD_DB_CTX_ENTRIES * sizeof(*ctx),
					  &dma_handle, GFP_KERNEL);
		if (!ctx)
			return -ENOMEM;

		ctxld->db[i] = ctx;
		ctxld->db_paddr[i] = dma_handle;

		ctx = dmam_alloc_coherent(dcss->dev,
					  CTXLD_SB_CTX_ENTRIES * sizeof(*ctx),
					  &dma_handle, GFP_KERNEL);
		if (!ctx)
			return -ENOMEM;

		ctxld->sb_hp[i] = ctx;
		ctxld->sb_lp[i] = ctx + CTXLD_SB_HP_CTX_ENTRIES;

		ctxld->sb_paddr[i] = dma_handle;
	}

	return 0;
}

int dcss_ctxld_init(struct dcss_soc *dcss, unsigned long ctxld_base)
{
	struct dcss_ctxld_priv *priv;
	int ret;

	priv = devm_kzalloc(dcss->dev, sizeof(struct dcss_ctxld_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dcss->ctxld_priv = priv;
	priv->dcss = dcss;

	ret = dcss_ctxld_alloc_ctx(priv);
	if (ret) {
		dev_err(dcss->dev, "ctxld: cannot allocate context memory.\n");
		return ret;
	}

	mutex_init(&priv->mutex);

	priv->ctxld_reg = devm_ioremap(dcss->dev, ctxld_base, SZ_4K);
	if (!priv->ctxld_reg) {
		dev_err(dcss->dev, "ctxld: unable to remap ctxld base\n");
		return -ENOMEM;
	}

	ret = dcss_ctxld_irq_config(priv);
	if (!ret)
		return ret;

	dcss_ctxld_hw_cfg(dcss);

	return 0;
}

void dcss_ctxld_exit(struct dcss_soc *dcss)
{
}

static int __dcss_ctxld_enable(struct dcss_ctxld_priv *ctxld)
{
	int curr_ctx = ctxld->current_ctx;
	u32 db_base, sb_base, sb_count;
	u32 sb_hp_cnt, sb_lp_cnt, db_cnt;

	sb_hp_cnt = ctxld->ctx_size[curr_ctx][CTX_SB_HP];
	sb_lp_cnt = ctxld->ctx_size[curr_ctx][CTX_SB_LP];
	db_cnt = ctxld->ctx_size[curr_ctx][CTX_DB];

	/* make sure SB_LP context area comes after SB_HP */
	if (sb_lp_cnt &&
	    ctxld->sb_lp[curr_ctx] != ctxld->sb_hp[curr_ctx] + sb_hp_cnt) {
		struct dcss_ctxld_item *sb_lp_adjusted;

		sb_lp_adjusted = ctxld->sb_hp[curr_ctx] + sb_hp_cnt;

		memcpy(sb_lp_adjusted, ctxld->sb_lp[curr_ctx],
		       sb_lp_cnt * CTX_ITEM_SIZE);
	}

	db_base = db_cnt ? ctxld->db_paddr[curr_ctx] : 0;

	dcss_writel(db_base, ctxld->ctxld_reg + DCSS_CTXLD_DB_BASE_ADDR);
	dcss_writel(db_cnt, ctxld->ctxld_reg + DCSS_CTXLD_DB_COUNT);

	if (sb_hp_cnt)
		sb_count = ((sb_hp_cnt << SB_HP_COUNT_POS) & SB_HP_COUNT_MASK) |
			   ((sb_lp_cnt << SB_LP_COUNT_POS) & SB_LP_COUNT_MASK);
	else
		sb_count = (sb_lp_cnt << SB_HP_COUNT_POS) & SB_HP_COUNT_MASK;

	sb_base = sb_count ? ctxld->sb_paddr[curr_ctx] : 0;

	dcss_writel(sb_base, ctxld->ctxld_reg + DCSS_CTXLD_SB_BASE_ADDR);
	dcss_writel(sb_count, ctxld->ctxld_reg + DCSS_CTXLD_SB_COUNT);

	/* enable the context loader */
	dcss_set(CTXLD_ENABLE, ctxld->ctxld_reg + DCSS_CTXLD_CONTROL_STATUS);

	ctxld->in_use = true;

	/*
	 * Toggle the current context to the alternate one so that any updates
	 * in the modules' settings take place there.
	 */
	ctxld->current_ctx ^= 1;

	ctxld->ctx_size[ctxld->current_ctx][CTX_DB] = 0;
	ctxld->ctx_size[ctxld->current_ctx][CTX_SB_HP] = 0;
	ctxld->ctx_size[ctxld->current_ctx][CTX_SB_LP] = 0;

	return 0;
}

/**
 * dcss_ctxld_enable - Enable context loader module.
 *
 * @dcss: pointer to dcss_soc.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int dcss_ctxld_enable(struct dcss_soc *dcss)
{
	struct dcss_ctxld_priv *ctxld = dcss->ctxld_priv;

	mutex_lock(&ctxld->mutex);
	if (ctxld->in_use) {
		ctxld->run_again = true;
		mutex_unlock(&ctxld->mutex);
		return 0;
	}

	__dcss_ctxld_enable(ctxld);

	mutex_unlock(&ctxld->mutex);

	return 0;
}
EXPORT_SYMBOL(dcss_ctxld_enable);

void dcss_ctxld_write(struct dcss_soc *dcss, u32 ctx_id, u32 val, u32 reg_ofs)
{
	struct dcss_ctxld_priv *ctxld = dcss->ctxld_priv;
	int curr_ctx = ctxld->current_ctx;
	struct dcss_ctxld_item *ctx[] = {
		[CTX_DB] = ctxld->db[curr_ctx],
		[CTX_SB_HP] = ctxld->sb_hp[curr_ctx],
		[CTX_SB_LP] = ctxld->sb_lp[curr_ctx]
	};
	int item_idx = ctxld->ctx_size[curr_ctx][ctx_id];

	/* if we hit this, we've got to increase the maximum context size */
	BUG_ON(dcss_ctxld_ctx_size[ctx_id] - 1 < item_idx);

	mutex_lock(&ctxld->mutex);
	ctx[ctx_id][item_idx].val = val;
	ctx[ctx_id][item_idx].ofs = reg_ofs;
	ctxld->ctx_size[curr_ctx][ctx_id] += 1;
	mutex_unlock(&ctxld->mutex);
}

int dcss_ctxld_resume(struct dcss_soc *dcss)
{
	struct dcss_ctxld_priv *ctxld = dcss->ctxld_priv;

	dcss_ctxld_hw_cfg(dcss);

	if (!ctxld->irq_en) {
		enable_irq(dcss->ctxld_priv->irq);
		ctxld->irq_en = true;
	}

	return 0;
}

int dcss_ctxld_suspend(struct dcss_soc *dcss)
{
	int ret = 0;
	struct dcss_ctxld_priv *ctxld = dcss->ctxld_priv;
	int wait_time_ms = 0;

	while (ctxld->in_use && wait_time_ms < 500) {
		msleep(20);
		wait_time_ms += 20;
	}

	if (wait_time_ms > 500)
		return -ETIMEDOUT;

	mutex_lock(&ctxld->mutex);

	if (ctxld->irq_en) {
		disable_irq_nosync(dcss->ctxld_priv->irq);
		ctxld->irq_en = false;
	}

	/* reset context region and sizes */
	ctxld->current_ctx = 0;
	ctxld->ctx_size[0][CTX_DB] = 0;
	ctxld->ctx_size[0][CTX_SB_HP] = 0;
	ctxld->ctx_size[0][CTX_SB_LP] = 0;

	mutex_unlock(&ctxld->mutex);
	return ret;
}

#ifdef CONFIG_DEBUG_FS
void dcss_ctxld_dump(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	struct dcss_ctxld_priv *ctxld = dcss->ctxld_priv;
	int curr_ctx = ctxld->current_ctx;
	int i;
	struct dcss_ctxld_item *ctx_db, *ctx_sb_hp, *ctx_sb_lp;
	u32 ctx_db_size, ctx_sb_hp_size, ctx_sb_lp_size;

	ctx_db_size = ctxld->ctx_size[curr_ctx ^ 1][CTX_DB];
	ctx_sb_hp_size = ctxld->ctx_size[curr_ctx ^ 1][CTX_SB_HP];
	ctx_sb_lp_size = ctxld->ctx_size[curr_ctx ^ 1][CTX_SB_LP];

	ctx_db = ctxld->db[curr_ctx ^ 1];
	ctx_sb_hp = ctxld->sb_hp[curr_ctx ^ 1];
	ctx_sb_lp = ctxld->sb_hp[curr_ctx ^ 1] + ctx_sb_hp_size;

	seq_puts(s, ">> Dumping loaded context:\n");
	seq_puts(s, "\t>>Dumping CTX_DB:\n");
	for (i = 0; i < ctx_db_size; i++)
		seq_printf(s, "\t0x%16llx -> 0x%08x : 0x%08x\n",
			   (u64)&ctx_db[i], ctx_db[i].ofs, ctx_db[i].val);
	seq_puts(s, "\t>>Dumping CTX_DB_HP:\n");
	for (i = 0; i < ctx_sb_hp_size; i++)
		seq_printf(s, "\t0x%16llx -> 0x%08x : 0x%08x\n",
			   (u64)&ctx_sb_hp[i], ctx_sb_hp[i].ofs,
			   ctx_sb_hp[i].val);
	seq_puts(s, "\t>>Dumping CTX_DB_LP:\n");
	for (i = 0; i < ctx_sb_lp_size; i++)
		seq_printf(s, "\t0x%16llx -> 0x%08x : 0x%08x\n",
			   (u64)&ctx_sb_lp[i], ctx_sb_lp[i].ofs,
			   ctx_sb_lp[i].val);
}
#endif
