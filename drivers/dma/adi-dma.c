// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/dmapool.h>

#include "dmaengine.h"
#include "adi-dma.h"

#define ADI_MEMSET_SIZE (4 * sizeof(uint64_t))

struct adi_dma_hw {
	int has_mdma;
};

struct adi_dma_filter_data {
	u32 id;
};

/*
 * register fields used to program dde engine
 */
struct adi_dde_descriptor {
	uint32_t next;
	uint32_t start;
	uint32_t cfg;
	uint32_t xcnt;
	uint32_t xmod;
	uint32_t ycnt;
	uint32_t ymod;
	struct list_head dde_desc_unit;
	dma_addr_t dma_handle;
};

struct adi_dma_descriptor {
	//dde descriptor used for register-based or descriptor based
	//mode
	struct adi_dde_descriptor dde_descriptor_src;
	//destination descriptor config to support memory copy/set operations
	struct adi_dde_descriptor dde_descriptor_dest;

	// additional bookkeeping
	struct dma_async_tx_descriptor tx;
	struct dmaengine_result result;
	struct list_head node;
	struct list_head cb_node;

	enum dma_transfer_direction direction;

	/* a cyclic descriptor will reuse itself, triggering callbacks as expected,
	 * and will not free itself when it finishes
	 */
	int cyclic;

	/* physical address of source location, in case of peripheral<->mem, the
	 * mem address is ALWAYS here and dest is unused.
	 */
	dma_addr_t src;

	/* physical address of destination, only used for MDMA */
	dma_addr_t dest;

	/* virtual address of memset buffer, used only with memset */
	uint64_t *memset;
	bool scattergather_mode;
	struct list_head dde_desc_list;
};

struct adi_dma_channel {
	int id;
	struct adi_dma *dma;
	void __iomem *iosrc;
	void __iomem *iodest;
	int running;
	int use_interrupts;
	int src_irq;
	int src_err_irq;
	int dest_irq;
	int dest_err_irq;
	/* descriptor in flight */
	struct adi_dma_descriptor *current_desc;
	/* descriptors to process */
	struct list_head pending;
	/* descriptors to call callbacks on */
	struct list_head cb_pending;
	struct dma_chan chan;
	struct dma_slave_config config;
	spinlock_t lock;
	bool desc_mode;
	bool ch_sync_disable;
	int periph_intf_width;
};

struct adi_dma {
	struct device *dev;
	struct dma_device dma_device;
	void __iomem *ioaddr;
	const struct adi_dma_hw *hw_cfg;
	spinlock_t lock;
	struct dma_pool *dde_desc_pool;
};

static struct adi_dma_hw adi_peripheral_dma_data = {
	.has_mdma	= 0,
};

static struct adi_dma_hw adi_mdma_data = {
	.has_mdma	= 1,
};

static const struct of_device_id dma_dt_ids[] = {
	{ .compatible = "adi,dma-controller",  .data = &adi_peripheral_dma_data },
	{ .compatible = "adi,mdma-controller", .data = &adi_mdma_data		},
	{ }
};
MODULE_DEVICE_TABLE(of, dma_dt_ids);


static void __adi_dma_enable_irqs(struct adi_dma_channel *);
static void __adi_dma_disable_irqs(struct adi_dma_channel *);
static void __adi_dma_clear_and_reset(struct adi_dma_channel *channel);

static irqreturn_t adi_dma_handler(int irq, void *id);
static irqreturn_t adi_dma_error_handler(int irq, void *id);
static irqreturn_t adi_dma_thread_handler(int irq, void *id);
static void __adi_dma_process_descriptor(struct adi_dma_descriptor *desc);
static void __adi_dma_process_descriptor_list(struct adi_dma_descriptor *desc);
static void __adi_dma_free_slave_sg_desc_list(struct adi_dma_descriptor *desc);
static struct adi_dde_descriptor *adi_dma_alloc_dde_descriptor(struct adi_dma *dma);

static int adi_dma_init_channel_interrupts(struct adi_dma *dma, struct device_node *node,
					   struct adi_dma_channel *channel)
{
	int irq;
	int ret;

	irq = of_irq_get_byname(node, "complete");
	if (irq <= 0) {
		dev_err(dma->dev, "Missing complete IRQ for channel %s\n", node->full_name);
		return irq ? irq : -ENOENT;
	}

	channel->src_irq = irq;

	ret = devm_request_threaded_irq(dma->dev, irq, adi_dma_handler,
					adi_dma_thread_handler, 0, "dma controller irq", channel);
	if (ret) {
		dev_err(dma->dev, "Failed to request IRQ %d\n", ret);
		return ret;
	}

	irq = of_irq_get_byname(node, "error");
	if (irq <= 0) {
		dev_err(dma->dev, "Missing error IRQ for channel %s\n", node->full_name);
		return irq ? irq : -ENOENT;
	}

	channel->src_err_irq = irq;

	ret = devm_request_threaded_irq(dma->dev, irq, adi_dma_error_handler,
					adi_dma_thread_handler, 0, "dma controller error irq", channel);
	if (ret) {
		dev_err(dma->dev, "Failed to request IRQ %d\n", ret);
		return ret;
	}

	if (dma->hw_cfg->has_mdma) {
		irq = of_irq_get_byname(node, "complete2");
		if (irq <= 0) {
			dev_err(dma->dev, "Missing complete2 IRQ for channel %s\n",
				node->full_name);
			return irq ? irq : -ENOENT;
		}

		channel->dest_irq = irq;

		ret = devm_request_threaded_irq(dma->dev, irq, adi_dma_handler,
						adi_dma_thread_handler, 0, "dma controller irq", channel);
		if (ret) {
			dev_err(dma->dev, "Failed to request IRQ %d\n", ret);
			return ret;
		}

		irq = of_irq_get_byname(node, "error2");
		if (irq <= 0) {
			dev_err(dma->dev, "Missing error2 IRQ for channel %s\n", node->full_name);
			return irq ? irq : -ENOENT;
		}

		channel->dest_err_irq = irq;

		ret = devm_request_threaded_irq(dma->dev, irq, adi_dma_error_handler,
						adi_dma_thread_handler, 0, "dma controller error irq", channel);
		if (ret) {
			dev_err(dma->dev, "Failed to request IRQ %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int adi_dma_init_channel(struct adi_dma *dma, struct device_node *node, unsigned int start, unsigned int len)
{
	struct adi_dma_channel *channel;
	int ret;
	u32 offset;
	u32 skip_int = 0;

	channel = devm_kzalloc(dma->dev, sizeof(*channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	if (of_property_read_u32(node, "adi,id", &channel->id)) {
		dev_err(dma->dev, "Missing adi,id for channel %s\n", node->full_name);
		return -ENOENT;
	}

	if (of_property_read_u32(node, "adi,src-offset", &offset)) {
		dev_err(dma->dev, "Missing adi,src-offset for channel %s\n",
			node->full_name);
		return -ENOENT;
	}

	channel->iosrc = devm_ioremap(dma->dev, start + offset, len);

	channel->dma = dma;
	channel->current_desc = NULL;
	spin_lock_init(&channel->lock);
	INIT_LIST_HEAD(&channel->pending);
	INIT_LIST_HEAD(&channel->cb_pending);

	channel->config = (struct dma_slave_config) {
		.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE,
		.src_maxburst = 1,
		.dst_maxburst = 1,
	};

	if (dma->hw_cfg->has_mdma) {
		if (of_property_read_u32(node, "adi,dest-offset", &offset)) {
			dev_err(dma->dev, "Missing adi,dest-offset for channel %s\n",
				node->full_name);
			return -ENOENT;
		}
		channel->iodest = dma->ioaddr + offset;
	}

	of_property_read_u32(node, "adi,skip-interrupts", &skip_int);
	channel->use_interrupts = !skip_int;

	if (channel->use_interrupts) {
		ret = adi_dma_init_channel_interrupts(dma, node, channel);
		if (ret)
			return ret;
	}

	channel->desc_mode = of_property_read_bool(node, "adi,dde-descriptor-mode");
	dev_info(dma->dev, "descriptor list mode is %d", channel->desc_mode);
	/*
	 * create dde descriptors dma pool for coherent access
	 */
	if (!dma->dde_desc_pool) {
		dma->dde_desc_pool = dma_pool_create("dde desc", dma->dev,
						     sizeof(struct adi_dde_descriptor), 4, 0);
		if (!dma->dde_desc_pool)
			return -ENOMEM;
	}
	channel->ch_sync_disable = of_property_read_bool(node, "adi,dde-sync-bit-disable");
	dev_info(dma->dev, "dde-sync-bit-disable is %d", channel->ch_sync_disable);

	if (of_property_read_u32(node, "periph-intf-width", &channel->periph_intf_width)) {
		dev_info(dma->dev, "channel %s: peripheral interface data width is not fixed.\n",
			 node->full_name);
		channel->periph_intf_width = -1;
	}

	// start with interrupts disabled, enable them when transactions appear
	channel->running = 1;
	__adi_dma_disable_irqs(channel);
	__adi_dma_clear_and_reset(channel);

	dma_cookie_init(&channel->chan);
	channel->chan.device = &dma->dma_device;
	list_add_tail(&channel->chan.device_node, &dma->dma_device.channels);
	return 0;
}

static struct adi_dma_descriptor *adi_dma_to_adi_desc(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct adi_dma_descriptor, tx);
}

static struct adi_dma_channel *adi_dma_to_adi_channel(struct dma_chan *chan)
{
	return container_of(chan, struct adi_dma_channel, chan);
}

static struct adi_dma_descriptor *adi_dma_alloc_descriptor(struct adi_dma *dma)
{
	struct adi_dma_descriptor *ret = NULL;

	ret = devm_kzalloc(dma->dev, sizeof(*ret), GFP_NOWAIT);
	dev_dbg(dma->dev, "%s: new desc allocated %px\n", __func__, ret);

	return ret;
}

static int adi_dma_desc_free(struct dma_async_tx_descriptor *tx)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(tx->chan);
	struct adi_dma_descriptor *desc = adi_dma_to_adi_desc(tx);
	struct adi_dma *dma = adi_chan->dma;

	dev_dbg(dma->dev, "%s: free desc %px\n", __func__, desc);

	if (desc->memset)
		dmam_free_coherent(dma->dev, ADI_MEMSET_SIZE, desc->memset, desc->src);

	devm_kfree(dma->dev, desc);
	return 0;
}

static struct adi_dde_descriptor *adi_dma_alloc_dde_descriptor(struct adi_dma *dma)
{
	struct adi_dde_descriptor *desc;
	dma_addr_t dma_handle;

	desc = dma_pool_zalloc(dma->dde_desc_pool, GFP_KERNEL, &dma_handle);
	desc->dma_handle = dma_handle;
	return desc;
}

/**
 * Only used by MDMA for determining access sizes, don't use with dma that is
 * attached directly to a peripheral
 */
static void adi_dma_get_txn_align(dma_addr_t src, dma_addr_t dst, size_t size,
				  u32 *conf, u32 *shift)
{
	if (dst % 32 == 0 && src % 32 == 0 && size % 32 == 0) {
		*conf = WDSIZE_256;
		*shift = 5;
	} else if (dst % 16 == 0 && src % 16 == 0 && size % 16 == 0) {
		*conf = WDSIZE_128;
		*shift = 4;
	} else if (dst % 8 == 0 && src % 8 == 0 && size % 8 == 0) {
		*conf = WDSIZE_64;
		*shift = 3;
	} else if (dst % 4 == 0 && src % 4 == 0 && size % 4 == 0) {
		*conf = WDSIZE_32;
		*shift = 2;
	} else if (dst % 2 == 0 && src % 2 == 0 && size % 2 == 0) {
		*conf = WDSIZE_16;
		*shift = 1;
	} else {
		*conf = WDSIZE_8;
		*shift = 0;
	}
}

/**
 * Only used with peripheral attached DMA and considers the burst characteristics
 * of the peripheral in addition to the memory and transaction size to make sure
 * that reads/writes work properly
 */
static void adi_dma_get_periph_align(struct adi_dma_channel *adi_chan,
				     enum dma_transfer_direction direction, dma_addr_t mem, size_t len,
				     u32 *conf, u32 *shift)
{
	struct dma_slave_config *cfg = &adi_chan->config;
	u32 burst = 0;
	int fixed_periph_width = adi_chan->periph_intf_width;
	int psize;

	if (fixed_periph_width != -1) {
		switch (fixed_periph_width) {
		case 1:
			psize = PSIZE_8;
			break;
		case 2:
			psize = PSIZE_16;
			break;
		case 4:
			psize = PSIZE_32;
			break;
		default:
		case 8:
			psize = PSIZE_64;
			break;
		}
	}
	if (direction == DMA_DEV_TO_MEM)
		burst = cfg->src_maxburst * cfg->src_addr_width;
	else
		burst = cfg->dst_maxburst * cfg->src_addr_width;

	if (mem % 8 == 0 && len % 8 == 0 && burst >= 8) {
		*conf = WDSIZE_64 | ((fixed_periph_width != -1) ? psize : PSIZE_64);
		*shift = 3;
	} else if (mem % 4 == 0 && len % 4 == 0 && burst >= 4) {
		*conf = WDSIZE_32 | ((fixed_periph_width != -1) ? psize : PSIZE_32);
		*shift = 2;
	} else if (mem % 2 == 0 && len % 2 == 0 && burst >= 2) {
		*conf = WDSIZE_16 | ((fixed_periph_width != -1) ? psize : PSIZE_16);
		*shift = 1;
	} else {
		*conf = WDSIZE_8 | ((fixed_periph_width != -1) ? psize : PSIZE_8);
		*shift = 0;
	}
}

static dma_cookie_t adi_dma_submit(struct dma_async_tx_descriptor *tx)
{
	struct adi_dma_descriptor *adi_desc = adi_dma_to_adi_desc(tx);
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(tx->chan);
	unsigned long flags;
	dma_cookie_t cookie;

	dev_dbg(adi_chan->dma->dev, "%s: submit desc %px\n", __func__, adi_desc);

	spin_lock_irqsave(&adi_chan->lock, flags);
	cookie = dma_cookie_assign(tx);
	list_add_tail(&adi_desc->node, &adi_chan->pending);
	spin_unlock_irqrestore(&adi_chan->lock, flags);

	dev_dbg(adi_chan->dma->dev, "%s: produced cookie %d\n", __func__, cookie);
	return cookie;
}

static void __adi_set_descriptor_list_config(void __iomem *ch_base,
					     struct adi_dde_descriptor *dde_desc)
{
	set_dma_start_addr(ch_base, dde_desc->start);
	set_dma_x_count(ch_base, dde_desc->xcnt);
	set_dma_x_modify(ch_base, dde_desc->xmod);

	/*
	 * set current desc config to descriptor list
	 * mode if next desc is available
	 */
	if (dde_desc->next)
		set_dma_next_desc_addr(ch_base, dde_desc->dma_handle);
}

/**
 * process DDE descriptors in descriptor list mode
 */
static void __adi_dma_process_descriptor_list(struct adi_dma_descriptor *desc)
{
	struct adi_dma_channel *channel = adi_dma_to_adi_channel(desc->tx.chan);
	struct adi_dma *dma = channel->dma;
	struct adi_dde_descriptor *dde_desc_1, *dde_desc_2;

	dev_dbg(dma->dev, "%s: process desc at %px\n", __func__, desc);

	if (get_dma_curr_irqstat(channel->iosrc) & DMA_RUN)
		dev_err(dma->dev, "processing a new descriptor while running\n");

	dde_desc_1 = list_first_entry(&desc->dde_desc_list,
				      struct adi_dde_descriptor, dde_desc_unit);
	__adi_set_descriptor_list_config(channel->iosrc, dde_desc_1);

	if (desc->direction == DMA_MEM_TO_MEM) {
		dde_desc_1 = &desc->dde_descriptor_src;
		dde_desc_2 = &desc->dde_descriptor_dest;
		__adi_set_descriptor_list_config(channel->iosrc, dde_desc_1);
		__adi_set_descriptor_list_config(channel->iodest, dde_desc_2);
	}
	channel->current_desc = desc;

	clear_dma_irqstat(channel->iosrc);
	if (desc->direction == DMA_MEM_TO_MEM)
		clear_dma_irqstat(channel->iodest);

	set_dma_config(channel->iosrc, dde_desc_1->cfg);
	__adi_dma_enable_irqs(channel);
}

/**
 * Load descriptor information into hardware registers
 * must be holding channel lock here
 */
static void __adi_dma_process_descriptor(struct adi_dma_descriptor *desc)
{
	struct adi_dma_channel *channel = adi_dma_to_adi_channel(desc->tx.chan);
	struct adi_dma *dma = channel->dma;
	struct adi_dde_descriptor *dde_desc;

	dev_dbg(dma->dev, "%s: process desc at %px", __func__, desc);

	if (get_dma_curr_irqstat(channel->iosrc) & DMA_RUN)
		dev_err(dma->dev, "processing a new descriptor while running\n");

	if (desc->scattergather_mode) {
		if (list_empty(&desc->dde_desc_list)) {
			dev_err(dma->dev, "%s , channel %d, dma desc contains empty dde descriptor list", __func__, channel->id);
			return;
		}
		dde_desc = list_first_entry(&desc->dde_desc_list, struct adi_dde_descriptor, dde_desc_unit);
		list_del(&dde_desc->dde_desc_unit);
	} else {
		dde_desc = &desc->dde_descriptor_src;
	}

	channel->current_desc = desc;
	desc->src = dde_desc->start;

	dev_dbg(dma->dev, "dma config: src = 0x%llx, dst = 0x%llx\n", desc->src, desc->dest);
	dev_dbg(dma->dev, "  xcount = %d, xmod = %d, cfg = 0x%x\n", dde_desc->xcnt, dde_desc->xmod, dde_desc->cfg);

	if (dde_desc->cfg & DMA2D) {
		dev_dbg(dma->dev, "  ycount = %d, ymod = %d\n", dde_desc->ycnt, dde_desc->ymod);
		set_dma_y_count(channel->iosrc, dde_desc->ycnt);
		set_dma_y_modify(channel->iosrc, dde_desc->ymod);
	}
	set_dma_start_addr(channel->iosrc, desc->src);
	set_dma_x_count(channel->iosrc, dde_desc->xcnt);

	/* In memset mode, we use xmod = 0 to copy from the same address repeatedly,
	 * and xmod only applies to the destination location
	 */
	if (desc->memset)
		set_dma_x_modify(channel->iosrc, 0);
	else
		set_dma_x_modify(channel->iosrc, dde_desc->xmod);

	clear_dma_irqstat(channel->iosrc);
	set_dma_config(channel->iosrc, dde_desc->cfg);

	if (desc->direction == DMA_MEM_TO_MEM) {
		u32 extra_config = WNR;

		if (dde_desc->cfg & DMA2D) {
			set_dma_y_count(channel->iodest, dde_desc->ycnt);
			set_dma_y_modify(channel->iodest, dde_desc->ymod);
			extra_config |= DI_EN_Y;
		} else {
			extra_config |= DI_EN_X;
		}

		dev_dbg(dma->dev, "  extracfg = 0x%x\n", dde_desc->cfg | extra_config);

		set_dma_start_addr(channel->iodest, desc->dest);
		set_dma_x_count(channel->iodest, dde_desc->xcnt);
		set_dma_x_modify(channel->iodest, dde_desc->xmod);
		clear_dma_irqstat(channel->iodest);
		set_dma_config(channel->iodest, dde_desc->cfg | extra_config);
	}

	if (desc->scattergather_mode)
		dma_pool_free(dma->dde_desc_pool, dde_desc, dde_desc->dma_handle);

	// For first descriptor enable IRQs again
	__adi_dma_enable_irqs(channel);
}

/**
 * must be holding channel lock here
 */
static void __adi_dma_issue_pending(struct adi_dma_channel *adi_chan)
{
	struct adi_dma_descriptor *desc;

	if (!adi_chan->current_desc) {
		if (!list_empty(&adi_chan->pending)) {
			desc = list_first_entry(&adi_chan->pending, struct adi_dma_descriptor,
						node);
			list_del(&desc->node);
			if (adi_chan->desc_mode == true)
				__adi_dma_process_descriptor_list(desc);
			else
				__adi_dma_process_descriptor(desc);
		} else {
			// Final descriptor ended, disable things
			__adi_dma_disable_irqs(adi_chan);
		}
	}
}

static void adi_dma_issue_pending(struct dma_chan *chan)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	unsigned long flags;

	dev_dbg(adi_chan->dma->dev, "%s: run\n", __func__);

	spin_lock_irqsave(&adi_chan->lock, flags);
	__adi_dma_issue_pending(adi_chan);
	spin_unlock_irqrestore(&adi_chan->lock, flags);
}

static enum dma_status adi_dma_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
					 struct dma_tx_state *txstate)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma_descriptor *desc = adi_chan->current_desc;
	u32 done, bytes, start;
	enum dma_status ret;
	struct adi_dde_descriptor *dde_desc = &desc->dde_descriptor_src;
	struct list_head *curr;
	struct list_head *tmp;
	bool desc_done = true;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	if (!desc)
		return DMA_COMPLETE;

	if (desc->result.result != DMA_TRANS_NOERROR)
		return DMA_ERROR;

	spin_lock(&adi_chan->lock);
	txstate->residue = 0;

	//check sg based transfers
	if (desc->scattergather_mode) {
		if (list_empty(&desc->dde_desc_list)) {
			spin_unlock(&adi_chan->lock);
			return DMA_COMPLETE;
		}
		start = get_dma_start_addr(adi_chan->iosrc);
		list_for_each_safe(curr, tmp, &desc->dde_desc_list) {
			dde_desc = list_entry(curr, struct adi_dde_descriptor, dde_desc_unit);
			if (adi_chan->desc_mode) {
				if (dde_desc->start != start)
					desc_done = false;
				if (!desc_done)
					txstate->residue += (dde_desc->xcnt * dde_desc->xmod);
			} else {
				txstate->residue += (dde_desc->xcnt * dde_desc->xmod);
			}
		}
	} else {
		// @todo this assumes ymod is one element, which is currently true in the only 2D case we support
		done = get_dma_curr_addr(adi_chan->iosrc) - desc->src;
		bytes = (dde_desc->xcnt * dde_desc->xmod);
		if (dde_desc->cfg & DMA2D)
			bytes = bytes * dde_desc->ycnt;
		txstate->residue = bytes - done;
	}
	spin_unlock(&adi_chan->lock);
	return DMA_IN_PROGRESS;
}

static void __adi_dma_enable_irqs(struct adi_dma_channel *adi_chan)
{
	if (adi_chan->running)
		return;

	adi_chan->running = 1;

	if (adi_chan->use_interrupts) {
		enable_irq(adi_chan->src_irq);
		enable_irq(adi_chan->src_err_irq);

		if (adi_chan->iodest) {
			enable_irq(adi_chan->dest_irq);
			enable_irq(adi_chan->dest_err_irq);
		}
	}
}

static void __adi_dma_disable_irqs(struct adi_dma_channel *adi_chan)
{
	if (!adi_chan->running)
		return;

	adi_chan->running = 0;

	if (adi_chan->use_interrupts) {
		disable_irq_nosync(adi_chan->src_irq);
		disable_irq_nosync(adi_chan->src_err_irq);

		if (adi_chan->iodest) {
			disable_irq_nosync(adi_chan->dest_irq);
			disable_irq_nosync(adi_chan->dest_err_irq);
		}
	}
}

static void __adi_dma_enable(struct adi_dma_channel *adi_chan)
{
	u32 cfg;

	cfg = get_dma_config(adi_chan->iosrc);
	set_dma_config(adi_chan->iosrc, cfg | DMAEN);

	if (adi_chan->iodest) {
		cfg = get_dma_config(adi_chan->iodest);
		set_dma_config(adi_chan->iodest, cfg | DMAEN);
	}

	__adi_dma_enable_irqs(adi_chan);
}

static void __adi_dma_disable(struct adi_dma_channel *adi_chan)
{
	u32 cfg;

	cfg = get_dma_config(adi_chan->iosrc);
	set_dma_config(adi_chan->iosrc, cfg & ~DMAEN);

	if (adi_chan->iodest) {
		cfg = get_dma_config(adi_chan->iodest);
		set_dma_config(adi_chan->iodest, cfg & ~DMAEN);
	}

	__adi_dma_disable_irqs(adi_chan);
}

static int adi_dma_pause(struct dma_chan *chan)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	unsigned long flags;

	spin_lock_irqsave(&adi_chan->lock, flags);
	if (adi_chan->current_desc)
		__adi_dma_disable(adi_chan);
	spin_unlock_irqrestore(&adi_chan->lock, flags);

	return 0;
}

static int adi_dma_resume(struct dma_chan *chan)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	unsigned long flags;

	spin_lock_irqsave(&adi_chan->lock, flags);
	if (adi_chan->current_desc)
		__adi_dma_enable(adi_chan);
	spin_unlock_irqrestore(&adi_chan->lock, flags);

	return 0;
}

static int adi_dma_terminate_all(struct dma_chan *chan)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma_descriptor *desc;
	unsigned long flags;
	struct list_head *curr;
	struct list_head *tmp;

	spin_lock_irqsave(&adi_chan->lock, flags);

	// Disable regardless of status to clear config that may have been modified
	// externally (for example in bootrom during resume)
	__adi_dma_clear_and_reset(adi_chan);
	__adi_dma_disable_irqs(adi_chan);

	if (adi_chan->current_desc) {
		desc = adi_chan->current_desc;
		desc->tx.desc_free(&desc->tx);
		adi_chan->current_desc = NULL;
	}

	list_for_each_safe(curr, tmp, &adi_chan->pending) {
		desc = list_entry(curr, struct adi_dma_descriptor, node);
		list_del(curr);
		desc->tx.desc_free(&desc->tx);
	}

	list_for_each_safe(curr, tmp, &adi_chan->cb_pending) {
		desc = list_entry(curr, struct adi_dma_descriptor, cb_node);
		list_del(curr);
		desc->tx.desc_free(&desc->tx);
	}

	spin_unlock_irqrestore(&adi_chan->lock, flags);

	return 0;
}

static void adi_dma_synchronize(struct dma_chan *chan)
{
	// terminate all doesn't sleep and also has nothing asynchronous to wait on
}

static int adi_dma_slave_config(struct dma_chan *chan,
				struct dma_slave_config *config)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);

	adi_chan->config = *config;
	return 0;
}

static void __adi_dma_clear_only(struct adi_dma_channel *channel)
{
	clear_dma_irqstat(channel->iosrc);

	if (channel->iodest)
		clear_dma_irqstat(channel->iodest);
}

static void __adi_dma_clear_and_reset(struct adi_dma_channel *channel)
{
	set_dma_config(channel->iosrc, 0);
	clear_dma_irqstat(channel->iosrc);

	if (channel->iodest) {
		set_dma_config(channel->iodest, 0);
		clear_dma_irqstat(channel->iodest);
	}
}

static irqreturn_t __adi_dma_handler(struct adi_dma_channel *channel,
				     enum dmaengine_tx_result result)
{
	struct adi_dma_descriptor *desc;
	u32 stat = 0;
	u32 stat2 = 0;
	irqreturn_t ret = IRQ_WAKE_THREAD;

	spin_lock(&channel->lock);

	stat = get_dma_curr_irqstat(channel->iosrc);

	dev_dbg(channel->dma->dev, "%s: got irqstat = 0x%x\n", __func__, stat);

	if (channel->iodest) {
		stat2 = get_dma_curr_irqstat(channel->iodest);
		dev_dbg(channel->dma->dev, "%s: got dest irqstat = 0x%x\n", __func__, stat);
	}

	// If we're not running, clear interrupt status
	if (!channel->running) {
		__adi_dma_clear_and_reset(channel);
		dev_err(channel->dma->dev,
			"channel %d: received interrupt while not runnnig\n", channel->id);
		ret = IRQ_HANDLED;
		goto done;
	}

	// DMA transaction still running, some peripherals will do this
	// before the transaction is finished because they signal the DMA channel
	// for more data on the same interrupt line
	if (!(stat & DMA_DONE) && !(stat2 & DMA_DONE)) {
		dev_err(channel->dma->dev, "channel %d: dma with not-done status 0x%x", channel->id, stat);
		ret = IRQ_HANDLED;
		goto done;
	}

	if (!channel->current_desc) {
		dev_err(channel->dma->dev, "channel %d: interrupt with no active desc\n",
			channel->id);
		ret = IRQ_HANDLED;
		goto done;
	}

	desc = channel->current_desc;
	dev_dbg(channel->dma->dev, "%s: current descriptor %px\n", __func__, desc);

	if (desc->cyclic || channel->desc_mode == true)
		__adi_dma_clear_only(channel);
	else
		__adi_dma_clear_and_reset(channel);

	/* register based mode, process sg list  */
	if ((desc->scattergather_mode) && (channel->desc_mode == false)) {
		if (!list_empty(&desc->dde_desc_list)) {
			__adi_dma_process_descriptor(desc);
			ret = IRQ_HANDLED;
			goto done;
		}
	}

	desc->result.result = result;
	desc->result.residue = 0;
	list_add_tail(&desc->cb_node, &channel->cb_pending);

	if (!desc->cyclic) {
		channel->current_desc = NULL;
		__adi_dma_issue_pending(channel);
	}

done:
	spin_unlock(&channel->lock);
	return ret;
}

static irqreturn_t adi_dma_handler(int irq, void *id)
{
	struct adi_dma_channel *channel = id;

	return __adi_dma_handler(channel, DMA_TRANS_NOERROR);
}

static irqreturn_t adi_dma_error_handler(int irq, void *id)
{
	struct adi_dma_channel *channel = id;
	enum dmaengine_tx_result result;
	u32 stat;

	// This is only meaningful for memcpy, as peripherals should be interpreted
	// based on dev-to-mem or mem-to-dev direction
	if (irq == channel->src_err_irq)
		result = DMA_TRANS_READ_FAILED;
	else
		result = DMA_TRANS_WRITE_FAILED;

	spin_lock(&channel->lock);

	// stop on this descriptor, user needs to read out the status and see what is
	// wrong, then terminate, and then queue new descriptors
	if (channel->current_desc) {
		stat = get_dma_curr_irqstat(channel->iosrc);
		dev_err(channel->dma->dev, "DMA error on channel %d, stat = 0x%x\n",
			channel->id, stat);
		channel->current_desc->result.result = result;
		__adi_dma_disable_irqs(channel);
	}

	__adi_dma_clear_and_reset(channel);

	spin_unlock(&channel->lock);
	return IRQ_HANDLED;
}

static irqreturn_t adi_dma_thread_handler(int irq, void *id)
{
	struct adi_dma_channel *channel = id;
	struct adi_dma_descriptor *desc;
	struct dmaengine_desc_callback cb;
	unsigned long flags;
	u32 stat;

	spin_lock_irqsave(&channel->lock, flags);

	while (!list_empty(&channel->cb_pending)) {
		desc = list_first_entry(&channel->cb_pending, struct adi_dma_descriptor,
					cb_node);
		list_del(&desc->cb_node);

		if (channel->desc_mode == true) {
			stat = get_dma_curr_irqstat(channel->iosrc);
			/*
			 *  we might miss interrupts when processing
			 *  descriptor list.
			 *
			 *  poll DMA engine to ensure we get all
			 *  data before terminating.
			 *
			 *  if DMA is idle -> all descriptors are processed
			 *  if DMA is not idle- > wait till DMA's start address is
			 *  the same as its current address:
			 *  either all descriptors are processed or
			 *  there are no more data left to be xferred
			 */
			if (!DMA_IDLE(stat)) {
				/*
				 * dma current address is advancing. wait here
				 */
				while (get_dma_start_addr(channel->iosrc) != get_dma_curr_addr(channel->iosrc))
					udelay(1);
			}
			/* release descriptor list and call it done */
			__adi_dma_free_slave_sg_desc_list(desc);
			__adi_dma_clear_and_reset(channel);
		}

		if (!desc->cyclic)
			dma_cookie_complete(&desc->tx);
		dmaengine_desc_get_callback(&desc->tx, &cb);

		spin_unlock_irqrestore(&channel->lock, flags);
		dmaengine_desc_callback_invoke(&cb, &desc->result);

		if (!desc->cyclic)
			desc->tx.desc_free(&desc->tx);

		spin_lock_irqsave(&channel->lock, flags);
	}

	spin_unlock_irqrestore(&channel->lock, flags);
	return IRQ_HANDLED;
}

/**
 * This never generates 2D memcpy but can handle up to 4 GB anyway
 */
static void adi_dma_memcpy_config(struct adi_dma_descriptor *desc,
				  struct adi_dma_channel *adi_chan,
				  dma_addr_t dst,
				  dma_addr_t src, size_t size)
{
	u32 conf, shift;
	s16 mod;
	struct adi_dde_descriptor *dde_desc_src = &desc->dde_descriptor_src;
	struct adi_dde_descriptor *dde_desc_dst = &desc->dde_descriptor_dest;


	adi_dma_get_txn_align(src, dst, size, &conf, &shift);

	// Run memcpy backwards if the two regions might overlap
	mod = 1 << shift;
	if (src < dst) {
		mod *= -1;
		dst += size + mod;
		src += size + mod;
	}
	size >>= shift;

	dde_desc_src->xcnt = size;
	dde_desc_src->xmod = mod;
	dde_desc_src->cfg = conf | DMAEN;
	desc->src = src;
	desc->dest = dst;
	if (adi_chan->desc_mode == true) {
		//set up source
		// descriptor list mode, 2D mode, restart on synchronize, enable dma channel,
		dde_desc_src->cfg = conf;
		dde_desc_src->start = src;

		//setup destination
		dde_desc_dst->cfg = conf | DMAEN | WNR | DI_EN_X;
		dde_desc_dst->start = dst;
		dde_desc_dst->xcnt = size;
		dde_desc_dst->xmod = mod;
	}
}

static struct dma_async_tx_descriptor *adi_dma_prep_memcpy(struct dma_chan *chan,
							   dma_addr_t dst, dma_addr_t src, size_t len, unsigned long flags)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma *dma = adi_chan->dma;
	struct adi_dma_descriptor *desc;

	if (!dma->hw_cfg->has_mdma)
		return NULL;

	desc = adi_dma_alloc_descriptor(dma);
	if (!desc)
		return NULL;

	dev_dbg(dma->dev, "%s: using desc at %px\n", __func__, desc);

	adi_dma_memcpy_config(desc, adi_chan, dst, src, len);
	desc->direction = DMA_MEM_TO_MEM;

	dma_async_tx_descriptor_init(&desc->tx, chan);
	desc->tx.flags = flags;
	desc->tx.tx_submit = adi_dma_submit;
	desc->tx.desc_free = adi_dma_desc_free;

	return &desc->tx;
}

static struct dma_async_tx_descriptor *adi_dma_prep_memset(struct dma_chan *chan,
							   dma_addr_t dest, int value, size_t len, unsigned long flags)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma *dma = adi_chan->dma;
	struct adi_dma_descriptor *desc;
	u8 byte = (u8)value;
	u64 bigword = byte * 0x01010101010101ull;
	u32 conf, shift;
	s16 mod;
	struct adi_dde_descriptor *dde_desc_src, *dde_desc_dst;

	if (!dma->hw_cfg->has_mdma)
		return NULL;

	desc = adi_dma_alloc_descriptor(dma);
	if (!desc)
		return NULL;
	dev_dbg(dma->dev, "%s: using desc at %px\n", __func__, desc);

	dde_desc_src = &desc->dde_descriptor_src;
	dde_desc_dst = &desc->dde_descriptor_dest;

	desc->memset = dmam_alloc_coherent(dma->dev, ADI_MEMSET_SIZE, &desc->src,
					   GFP_NOWAIT);
	if (!desc->memset) {
		dev_err(dma->dev, "%s, dmam_alloc_coherent failed\n", __func__);
		devm_kfree(dma->dev, desc);
		return NULL;
	}

	adi_dma_get_txn_align(desc->src, dest, len, &conf, &shift);

	desc->memset[0] = bigword;
	desc->memset[1] = bigword;
	desc->memset[2] = bigword;
	desc->memset[3] = bigword;

	mod = 1 << shift;
	len >>= shift;

	dde_desc_src->xcnt = len;
	dde_desc_src->xmod = mod;
	if (adi_chan->desc_mode == true) {
		//set up source
		// descriptor list mode, 2D mode, restart on synchronize, enable dma channel,
		dde_desc_src->cfg = conf | DMAEN;
		dde_desc_src->start = lower_32_bits((dma_addr_t)desc->memset);
		//use the same address to set memory
		dde_desc_src->xmod = 0;
		//setup destination
		dde_desc_dst->cfg = conf | DMAEN | WNR | DI_EN_X;
		dde_desc_dst->start = dest;
		dde_desc_dst->xcnt = len;
		dde_desc_dst->xmod = mod;
	} else {
		dde_desc_src->cfg = conf | DMAEN;
	}
	// desc->src set above when memset buf is allocated
	desc->dest = dest;
	desc->direction = DMA_MEM_TO_MEM;

	dma_async_tx_descriptor_init(&desc->tx, chan);
	desc->tx.flags = flags;
	desc->tx.tx_submit = adi_dma_submit;
	desc->tx.desc_free = adi_dma_desc_free;

	return &desc->tx;
}

static void __adi_dma_free_slave_sg_desc_list(struct adi_dma_descriptor *desc)
{
	struct dma_chan *chan = desc->tx.chan;
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma *dma = adi_chan->dma;
	struct adi_dde_descriptor *dde_desc;
	struct list_head *curr;
	struct list_head *tmp;

	list_for_each_safe(curr, tmp, &desc->dde_desc_list) {
		dde_desc = list_entry(curr, struct adi_dde_descriptor, dde_desc_unit);
		list_del(curr);
		dma_pool_free(dma->dde_desc_pool, dde_desc, dde_desc->dma_handle);
	}
}

static void __adi_dma_prep_slave_sg_desc_list(struct adi_dma_descriptor *desc, struct scatterlist *sgl)
{
	struct dma_chan *chan = desc->tx.chan;
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma *dma = adi_chan->dma;
	size_t txlen;
	dma_addr_t src;
	u32 conf, shift, sync;
	struct adi_dde_descriptor *dde_desc, *dde_desc_next;

	INIT_LIST_HEAD(&desc->dde_desc_list);

	dde_desc = adi_dma_alloc_dde_descriptor(dma);

	sync = adi_chan->ch_sync_disable ? 0 : DMASYNC;

	while (sgl != NULL) {
		txlen = sg_dma_len(sgl);
		src = sg_dma_address(sgl);

		adi_dma_get_periph_align(adi_chan, desc->direction, src, txlen, &conf, &shift);

		if (desc->direction == DMA_DEV_TO_MEM)
			conf |= WNR;

		conf |= DI_EN_X;

		dde_desc->xcnt = txlen >> shift;
		dde_desc->xmod = 1 << shift;
		dde_desc->start = src;
		dde_desc->cfg = conf | sync | DMAEN;

		sgl = sg_next(sgl);
		if (sgl) {
			dde_desc_next = adi_dma_alloc_dde_descriptor(dma);
			if (adi_chan->desc_mode == true) {
				dde_desc->cfg |= DMAFLOW_LIST | NDSIZE_4;
				dde_desc->next = dde_desc_next->dma_handle;
			}
		}
		list_add_tail(&dde_desc->dde_desc_unit, &desc->dde_desc_list);
		dde_desc = dde_desc_next;
	}
}

static struct dma_async_tx_descriptor *adi_dma_prep_slave_sg(struct dma_chan *chan,
							     struct scatterlist *sgl, unsigned int sg_len,
							     enum dma_transfer_direction direction, unsigned long flags, void *context)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma *dma = adi_chan->dma;
	struct adi_dma_descriptor *desc = NULL;

	desc = adi_dma_alloc_descriptor(dma);
	if (!desc)
		return NULL;

	dev_dbg(dma->dev, "%s: using desc at %px\n", __func__, desc);

	dma_async_tx_descriptor_init(&desc->tx, chan);
	desc->tx.flags = flags;
	desc->tx.tx_submit = adi_dma_submit;
	desc->tx.desc_free = adi_dma_desc_free;
	desc->scattergather_mode = true;
	desc->direction = direction;
	/*
	 * parse sgl and add reqs to the dde dma decriptor list
	 */
	__adi_dma_prep_slave_sg_desc_list(desc, sgl);

	return &desc->tx;
}

static struct dma_async_tx_descriptor *adi_dma_prep_cyclic(struct dma_chan *chan,
							   dma_addr_t buf, size_t len, size_t period_len,
							   enum dma_transfer_direction direction, unsigned long flags)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma *dma = adi_chan->dma;
	struct adi_dma_descriptor *desc;
	u32 conf, shift, sync;
	struct adi_dde_descriptor *dde_desc;

	desc = adi_dma_alloc_descriptor(dma);
	if (!desc)
		return NULL;

	dev_dbg(dma->dev, "%s: using desc at %px\n", __func__, desc);

	dde_desc = &desc->dde_descriptor_src;

	adi_dma_get_periph_align(adi_chan, direction, buf, period_len, &conf, &shift);

	dde_desc->xcnt = period_len >> shift;
	dde_desc->xmod = 1 << shift;
	dde_desc->ycnt = len / period_len;
	dde_desc->ymod = dde_desc->xmod;

	// Interpret prep interrupt to mean interrupt between each period,
	// without it only interrupt after all periods for bookkeeping
	if (flags & DMA_PREP_INTERRUPT)
		conf |= DI_EN_X;
	else
		conf |= DI_EN_Y;

	if (direction == DMA_DEV_TO_MEM)
		conf |= WNR;

	sync = adi_chan->ch_sync_disable ? 0 : DMASYNC;

	if (adi_chan->desc_mode == true) {
		// descriptor list mode, 2D mode, restart on synchronize, enable dma channel,
		dde_desc->cfg = conf | DMAFLOW_LIST | NDSIZE_6 | DMA2D | sync | DMAEN;
		dde_desc->start = buf;
		//set the next descriptor to point to itself.
		dde_desc->next = (uintptr_t)dde_desc;
	} else {
		// autoflow mode, 2D mode, restart on synchronize, enable dma channel,
		dde_desc->cfg = conf | DMAFLOW_AUTO | DMA2D | sync | DMAEN;
		desc->src = buf;
	}
	desc->direction = direction;
	desc->cyclic = 1;

	dma_async_tx_descriptor_init(&desc->tx, chan);
	desc->tx.flags = flags;
	desc->tx.tx_submit = adi_dma_submit;
	desc->tx.desc_free = adi_dma_desc_free;

	return &desc->tx;
}

static bool adi_dma_filter(struct dma_chan *chan, void *data)
{
	struct adi_dma_channel *adi_chan = adi_dma_to_adi_channel(chan);
	struct adi_dma_filter_data *adi_data = data;

	if (adi_chan->id == adi_data->id)
		return true;

	return false;
}

static struct dma_chan *adi_dma_translate(struct of_phandle_args *args,
					  struct of_dma *ofdma)
{
	dma_cap_mask_t mask;
	struct adi_dma_filter_data data;

	if (args->args_count != 1)
		return NULL;

	data.id = (u32)args->args[0];
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	return __dma_request_channel(&mask, adi_dma_filter, &data, ofdma->of_node);
}

static int adi_dma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	const struct of_device_id *of_id;
	struct adi_dma *dma;
	struct resource *res;
	void __iomem *base = NULL;
	int ret;
	u32 buswidths;

	dma = devm_kzalloc(dev, sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	spin_lock_init(&dma->lock);
	dma->dev = dev;

	of_id = of_match_device(dma_dt_ids, dev);
	if (!of_id) {
		dev_err(dev, "No matching device data found...?\n");
		return -ENOENT;
	}

	dma->hw_cfg = of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dma->ioaddr = base;

	INIT_LIST_HEAD(&dma->dma_device.channels);

	dma->dma_device.device_issue_pending = adi_dma_issue_pending;
	dma->dma_device.device_tx_status = adi_dma_tx_status;
	dma->dma_device.device_pause = adi_dma_pause;
	dma->dma_device.device_resume = adi_dma_resume;
	dma->dma_device.device_terminate_all = adi_dma_terminate_all;
	dma->dma_device.device_synchronize = adi_dma_synchronize;

	buswidths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		    BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);

	if (dma->hw_cfg->has_mdma) {
		dev_info(dev, "Creating new MDMA controller instance\n");
		dma_cap_set(DMA_MEMCPY, dma->dma_device.cap_mask);
		dma_cap_set(DMA_MEMSET, dma->dma_device.cap_mask);

		buswidths |= BIT(DMA_SLAVE_BUSWIDTH_16_BYTES) |
			     BIT(DMA_SLAVE_BUSWIDTH_32_BYTES);

		dma->dma_device.directions = BIT(DMA_MEM_TO_MEM);
		dma->dma_device.src_addr_widths = buswidths;
		dma->dma_device.dst_addr_widths = buswidths;
		dma->dma_device.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
		dma->dma_device.copy_align = 0;
		dma->dma_device.fill_align = 0;

		dma->dma_device.device_prep_dma_memcpy = adi_dma_prep_memcpy;
		dma->dma_device.device_prep_dma_memset = adi_dma_prep_memset;
	} else {
		dev_info(dev, "Creating new peripheral DMA controller instance\n");
		dma_cap_set(DMA_SLAVE, dma->dma_device.cap_mask);
		dma_cap_set(DMA_CYCLIC, dma->dma_device.cap_mask);
		dma_cap_set(DMA_PRIVATE, dma->dma_device.cap_mask);

		dma->dma_device.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
		dma->dma_device.src_addr_widths = buswidths;
		dma->dma_device.dst_addr_widths = buswidths;
		dma->dma_device.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

		dma->dma_device.device_config = adi_dma_slave_config;
		dma->dma_device.device_prep_slave_sg = adi_dma_prep_slave_sg;
		dma->dma_device.device_prep_dma_cyclic = adi_dma_prep_cyclic;
	}

	child = NULL;
	while ((child = of_get_next_child(np, child))) {
		ret = adi_dma_init_channel(dma, child, res->start, res->end - res->start + 1);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	platform_set_drvdata(pdev, dma);

	dma->dma_device.dev = dev;
	ret = dmaenginem_async_device_register(&dma->dma_device);
	if (ret) {
		dev_err(dev, "Unable to register async transaction DMA engine\n");
		return ret;
	}

	ret = of_dma_controller_register(np, adi_dma_translate, dma);
	if (ret) {
		dev_err(&pdev->dev, "failed to register controller\n");
		return ret;
	}

	return 0;
}

static void adi_dma_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adi_dma *dma;

	/* everything else allocated with devm, we don't have to free anything */
	dma = platform_get_drvdata(pdev);
	if (!dma->dde_desc_pool)
		dma_pool_destroy(dma->dde_desc_pool);

	of_dma_controller_free(np);
}

static struct platform_driver dma_driver = {
	.driver			= {
		.name		= "adi-dma",
		.of_match_table = dma_dt_ids,
	},
	.probe			= adi_dma_probe,
	.remove			= adi_dma_remove,
};
module_platform_driver(dma_driver);

MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
MODULE_DESCRIPTION("SC5xx DMA Controller Driver");
MODULE_LICENSE("GPL");
