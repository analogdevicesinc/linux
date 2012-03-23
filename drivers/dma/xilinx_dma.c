/*
 * Xilinx DMA Engine support
 *
 * Copyright (C) 2010 Xilinx, Inc. All rights reserved.
 * Copyright (C) 2012 Analog Device Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Based on the Freescale DMA driver.
 *
 * Description:
 * This driver supports three Xilinx DMA engines:
 *  . Axi DMA engine, it does transfers between memory and device. It can be
 *    configured to have one channel or two channels. If configured as two
 *    channels, one is to transmit to a device and another is to receive from
 *    a device.
 *  . Axi VDMA engine, it does transfers between memory and video devices.
 *    It can be configured to have one channel or two channels. If configured
 *    as two channels, one is to transmit to the video device and another is
 *    to receive from the video device.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dmapool.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/amba/xilinx_dma.h>
#include <linux/debugfs.h>
#include <linux/sched.h>

/* Hw specific definitions
 */
#define XILINX_DMA_MAX_CHANS_PER_DEVICE  0x2
#define XILINX_DMA_MAX_TRANS_LEN         0x7FFFFF

/* General register bits definitions
 */
#define XILINX_DMA_CR_RESET_MASK    0x00000004  /* Reset DMA engine */
#define XILINX_DMA_CR_RUNSTOP_MASK  0x00000001  /* Start/stop DMA engine */

#define XILINX_DMA_SR_HALTED_MASK   0x00000001  /* DMA channel halted */
#define XILINX_DMA_SR_IDLE_MASK     0x00000002  /* DMA channel idle */

#define XILINX_DMA_SR_ERR_INTERNAL_MASK 0x00000010 /* Datamover internal err */
#define XILINX_DMA_SR_ERR_SLAVE_MASK    0x00000020 /* Datamover slave err */
#define XILINX_DMA_SR_ERR_DECODE_MASK   0x00000040 /* Datamover decode err */
#define XILINX_DMA_SR_ERR_SG_INT_MASK   0x00000100 /* SG internal err */
#define XILINX_DMA_SR_ERR_SG_SLV_MASK   0x00000200 /* SG slave err */
#define XILINX_DMA_SR_ERR_SG_DEC_MASK   0x00000400 /* SG decode err */
#define XILINX_DMA_SR_ERR_ALL_MASK      0x00000770 /* All errors */

#define XILINX_DMA_XR_IRQ_IOC_MASK	0x00001000 /* Completion interrupt */
#define XILINX_DMA_XR_IRQ_DELAY_MASK	0x00002000 /* Delay interrupt */
#define XILINX_DMA_XR_IRQ_ERROR_MASK	0x00004000 /* Error interrupt */
#define XILINX_DMA_XR_IRQ_ALL_MASK	    0x00007000 /* All interrupts */

#define XILINX_DMA_XR_DELAY_MASK    0xFF000000 /* Delay timeout counter */
#define XILINX_DMA_XR_COALESCE_MASK 0x00FF0000 /* Coalesce counter */

#define XILINX_DMA_DELAY_SHIFT    24
#define XILINX_DMA_COALESCE_SHIFT 16

#define XILINX_DMA_DELAY_MAX     0xFF /**< Maximum delay counter value */
#define XILINX_DMA_COALESCE_MAX  0xFF /**< Maximum coalescing counter value */

#define XILINX_DMA_RX_CHANNEL_OFFSET      0x30

/* Axi VDMA special register bits
 */
#define XILINX_VDMA_CIRC_EN         0x00000002  /* Circular mode */
#define XILINX_VDMA_SYNC_EN         0x00000008  /* Sync enable mode */
#define XILINX_VDMA_FRMCNT_EN       0x00000010  /* Frm Cnt enable mode */
#define XILINX_VDMA_MSTR_MASK       0x00000F00  /* Master in control */

#define XILINX_VDMA_MSTR_SHIFT      8
#define XILINX_VDMA_WR_REF_SHIFT    8

#define XILINX_VDMA_FRMDLY_SHIFT  24

#define XILINX_VDMA_DIRECT_REG_OFFSET     0x50
#define XILINX_VDMA_CHAN_DIRECT_REG_SIZE  0x50

/* BD definitions for Axi DMA
 */
#define XILINX_DMA_BD_STS_COMPL_MASK 0x80000000
#define XILINX_DMA_BD_STS_ERR_MASK   0x70000000
#define XILINX_DMA_BD_STS_ALL_MASK   0xF0000000

/* Axi DMA BD special bits definitions
 */
#define XILINX_DMA_BD_SOP       0x08000000    /* Start of packet bit */
#define XILINX_DMA_BD_EOP       0x04000000    /* End of packet bit */

/* Feature encodings
 */
#define XILINX_DMA_FTR_DATA_WIDTH_MASK 0x000000FF /* Data width mask, 1024 */
#define XILINX_DMA_FTR_HAS_SG          0x00000100 /* Has SG */
#define XILINX_DMA_FTR_HAS_SG_SHIFT    8          /* Has SG shift */
#define XILINX_DMA_FTR_STSCNTRL_STRM   0x00010000 /* Optional feature for dma */

/* Delay loop counter to prevent hardware failure
 */
#define XILINX_DMA_RESET_LOOP            1000000
#define XILINX_DMA_HALT_LOOP             1000000

/* IO accessors
 */
#define DMA_OUT(addr, val)  (iowrite32(val, addr))
#define DMA_IN(addr)  (ioread32(addr))

/* Hardware descriptor
 *
 * shared by all Xilinx DMA engines
 */
struct xilinx_dma_desc_hw {
	u32 next_desc;	/* 0x00 */
	u32 pad1;       /* 0x04 */
	u32 buf_addr;   /* 0x08 */
	u32 pad2;       /* 0x0C */
	u32 addr_vsize; /* 0x10 */
	u32 hsize;      /* 0x14 */
	u32 control;    /* 0x18 */
	u32 status;     /* 0x1C */
	u32 app_0;      /* 0x20 */
	u32 app_1;      /* 0x24 */
	u32 app_2;      /* 0x28 */
	u32 app_3;      /* 0x2C */
	u32 app_4;      /* 0x30 */
} __aligned(64);

struct xilinx_dma_desc_sw {
	struct xilinx_dma_desc_hw *hw;
	dma_addr_t phys;
};

struct xilinx_dma_transfer {
	struct dma_async_tx_descriptor async_tx;
	struct list_head head;

	bool cyclic;
	unsigned int completed_descs;

	unsigned int current_desc;
	unsigned int num_descs;
	struct xilinx_dma_desc_sw descs[];
};

struct xdma_regs {
	u32 cr;     /* 0x00 Control Register */
	u32 sr;     /* 0x04 Status Register */
	u32 cdr;    /* 0x08 Current Descriptor Register */
	u32 pad1;
	u32 tdr;    /* 0x10 Tail Descriptor Register */
	u32 pad2;
	u32 src;    /* 0x18 Source Address Register (cdma) */
	u32 pad3;
	u32 dst;    /* 0x20 Destination Address Register (cdma) */
	u32 pad4;
	u32 btt_ref;/* 0x28 Bytes To Transfer (cdma) or park_ref (vdma) */
	u32 version;         /* 0x2c version (vdma) */
};

static struct debugfs_reg32 xilinx_dma_debugfs_regs[] = {
	{ "Control", 0x00 },
	{ "Status", 0x04 },
	{ "Current descriptor", 0x08 },
	{ "Tail descriptor", 0x10 },
	{ "Vertical size", 0x50 },
	{ "Horizontal size", 0x54 },
	{ "Frame delay/stride", 0x58 },
	{ "Frame address 0", 0x5c },
	{ "Frame address 1", 0x60 },
	{ "Frame address 2", 0x64 },
	{ "Frame address 3", 0x68 },
};

struct vdma_addr_regs {
	u32 vsize;          /* 0x0 Vertical size */
	u32 hsize;          /* 0x4 Horizontal size */
	u32 frmdly_stride;  /* 0x8 Frame delay and stride */
	u32 buf_addr[16];   /* 0xC - 0x48 Src addresses */
};

/* Per DMA specific operations should be embedded in the channel structure
 */
struct xilinx_dma_chan {
	struct xdma_regs __iomem *regs;   /* Control status registers */
	struct vdma_addr_regs *addr_regs; /* Direct address registers */
	dma_cookie_t completed_cookie;	  /* The maximum cookie completed */
	spinlock_t lock;                  /* Descriptor operation lock */
	struct list_head active_list;	  /* Active descriptors */
	struct list_head pending_list;	  /* Descriptors waiting */
	struct dma_chan common;           /* DMA common channel */
	struct dma_pool *desc_pool;       /* Descriptors pool */
	struct device *dev;               /* The dma device */
	int    irq;                       /* Channel IRQ */
	int    id;                        /* Channel ID */
	enum dma_transfer_direction direction;/* Transfer direction */
	int    max_len;                   /* Maximum data len per transfer */
	int    num_frms;                  /* Number of frames */
	int    has_SG;                    /* Support scatter transfers */
	int    has_DRE;                   /* Support unaligned transfers */
	int    genlock;                   /* Support genlock mode */
	int    err;                       /* Channel has errors */
	struct tasklet_struct tasklet;    /* Cleanup work after irq */
	u32    feature;                   /* IP feature */
	void   (*start_transfer)(struct xilinx_dma_chan *chan);
	struct xilinx_dma_config config;  /* Device configuration info */

	bool cyclic;
	struct debugfs_regset32 debugfs_regset;
};

struct xilinx_dma_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct xilinx_dma_chan *chan[XILINX_DMA_MAX_CHANS_PER_DEVICE];
	u32 feature;
	int irq;
};

static dma_cookie_t xilinx_dma_tx_submit(struct dma_async_tx_descriptor *tx);

#define to_xilinx_chan(chan) container_of(chan, struct xilinx_dma_chan, common)

static void xilinx_dma_free_transfer(struct xilinx_dma_chan *chan,
	struct xilinx_dma_transfer *t)
{
	unsigned int i;
	for (i = 0; i < t->num_descs; ++i)
		dma_pool_free(chan->desc_pool, t->descs[i].hw, t->descs[i].phys);
	kfree(t);
}

static struct xilinx_dma_transfer *xilinx_dma_alloc_transfer(
	struct xilinx_dma_chan *chan, unsigned int num_descs)
{
	struct xilinx_dma_desc_hw *new, *prev;
	struct xilinx_dma_transfer *t;
	dma_addr_t phys;

	if (num_descs == 0)
		return NULL;

	t = kzalloc(sizeof(*t) + num_descs * sizeof(*t->descs), GFP_ATOMIC);
	if (!t)
		return NULL;

	dma_async_tx_descriptor_init(&t->async_tx, &chan->common);
	t->async_tx.tx_submit = xilinx_dma_tx_submit;
	t->async_tx.cookie = -EBUSY;

	prev = NULL;
	new = NULL;
	for (; t->num_descs < num_descs; t->num_descs++) {
		new = dma_pool_alloc(chan->desc_pool, GFP_ATOMIC, &phys);
		if (!new) {
			dev_err(chan->dev, "No free memory for link descriptor\n");
			goto err_free;
		}
		memset(new, 0, sizeof(*new));

		if (prev)
			prev->next_desc = phys;

		t->descs[t->num_descs].hw = new;
		t->descs[t->num_descs].phys = phys;
		prev = new;
	}

	/* Link the last BD with the first BD */
	new->next_desc = t->descs[0].phys;

	return t;
err_free:
	xilinx_dma_free_transfer(chan, t);
	return NULL;
}

static void xilinx_dma_free_transfer_list(struct xilinx_dma_chan *chan,
	struct list_head *list)
{
	struct xilinx_dma_transfer *t, *_t;
	list_for_each_entry_safe(t, _t, list, head)
		xilinx_dma_free_transfer(chan, t);
	INIT_LIST_HEAD(list);
}

static void xilinx_dma_free_transfers(struct xilinx_dma_chan *chan)
{
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	xilinx_dma_free_transfer_list(chan, &chan->active_list);
	xilinx_dma_free_transfer_list(chan, &chan->pending_list);
	spin_unlock_irqrestore(&chan->lock, flags);
}

/* Required functions
 */
static int xilinx_dma_alloc_chan_resources(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);

	/* Has this channel already been allocated? */
	if (chan->desc_pool)
		return 1;

	/*
	 * We need the descriptor to be aligned to 64bytes
	 * for meeting Xilinx DMA specification requirement.
	 */
	chan->desc_pool = dma_pool_create("xilinx_dma_desc_pool",
				  chan->dev,
				  sizeof(struct xilinx_dma_desc_hw),
				  __alignof__(struct xilinx_dma_desc_hw), 0);
	if (!chan->desc_pool) {
		dev_err(chan->dev, "unable to allocate channel %d "
				   "descriptor pool\n", chan->id);
		return -ENOMEM;
	}

	chan->completed_cookie = 0;

	/* there is at least one descriptor free to be allocated */
	return 1;
}

static void xilinx_dma_free_chan_resources(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);

	dev_dbg(chan->dev, "Free all channel resources.\n");
	xilinx_dma_free_transfers(chan);
	dma_pool_destroy(chan->desc_pool);
	chan->desc_pool = NULL;
}

static enum dma_status xilinx_dma_desc_status(struct xilinx_dma_chan *chan,
					  struct xilinx_dma_transfer *t)
{
	return dma_async_is_complete(t->async_tx.cookie,
				     chan->completed_cookie,
				     chan->common.cookie);
}

static void xilinx_dma_chan_handle_cyclic(struct xilinx_dma_chan *chan,
	struct xilinx_dma_transfer *t, unsigned long *flags)
{
	unsigned int completed_descs;
	dma_async_tx_callback callback;
	void *callback_param;
	unsigned int i;

	/* We have to be carefull not to dereference 't' anymore after a call to
	 * the callback function, since it might call terminate_all and as a
	 * result 't' might be already freed. */
	callback = t->async_tx.callback;
	callback_param = t->async_tx.callback_param;
	completed_descs = t->completed_descs;
	t->completed_descs = 0;

	spin_unlock_irqrestore(&chan->lock, *flags);
	for (i = 0; i < completed_descs; i++)
		callback(callback_param);
	spin_lock_irqsave(&chan->lock, *flags);
}

static void xilinx_chan_desc_cleanup(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_transfer *t;
	dma_async_tx_callback callback;
	void *callback_param;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	/* terminate_all might be called from the callback, so we can't iterate over
	 * the list using list_for_each_entry_safe */
	while (!list_empty(&chan->active_list)) {
		t = list_first_entry(&chan->active_list, struct xilinx_dma_transfer, head);

		if (t->cyclic) {
			xilinx_dma_chan_handle_cyclic(chan, t, &flags);
			break;
		}

		if (xilinx_dma_desc_status(chan, t) == DMA_IN_PROGRESS)
			break;

		list_del(&t->head);

		callback = t->async_tx.callback;
		callback_param = t->async_tx.callback_param;
		if (callback) {
			spin_unlock_irqrestore(&chan->lock, flags);
			callback(callback_param);
			spin_lock_irqsave(&chan->lock, flags);
		}

		dma_run_dependencies(&t->async_tx);
		xilinx_dma_free_transfer(chan, t);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

static enum dma_status xilinx_tx_status(struct dma_chan *dchan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	dma_cookie_t last_used;
	dma_cookie_t last_complete;

	xilinx_chan_desc_cleanup(chan);

	last_used = dchan->cookie;
	last_complete = chan->completed_cookie;

	dma_set_tx_state(txstate, last_complete, last_used, 0);

	return dma_async_is_complete(cookie, last_complete, last_used);
}

static int xilinx_dma_is_running(struct xilinx_dma_chan *chan)
{
	return !(DMA_IN(&chan->regs->sr) & XILINX_DMA_SR_HALTED_MASK) &&
	   (DMA_IN(&chan->regs->cr) & XILINX_DMA_CR_RUNSTOP_MASK);
}

static int xilinx_dma_is_idle(struct xilinx_dma_chan *chan)
{
	return DMA_IN(&chan->regs->sr) & XILINX_DMA_SR_IDLE_MASK;
}

static int xilinx_dma_wait_idle(struct xilinx_dma_chan *chan)
{
	unsigned long timeout = 10000;

	do {
		if (xilinx_dma_is_idle(chan))
			break;
	} while (--timeout);

	if (!xilinx_dma_is_idle(chan))
		return -ETIMEDOUT;

	return 0;
}

#define XILINX_DMA_DRIVER_DEBUG 0

#if (XILINX_DMA_DRIVER_DEBUG == 1)
static void desc_dump(struct xilinx_dma_desc_hw *hw)
{
	printk(KERN_INFO "hw desc %x:\n", (unsigned int)hw);
	printk(KERN_INFO "\tnext_desc %x\n", hw->next_desc);
	printk(KERN_INFO "\tbuf_addr %x\n", hw->buf_addr);
	printk(KERN_INFO "\taddr_vsize %x\n", hw->addr_vsize);
	printk(KERN_INFO "\thsize %x\n", hw->hsize);
	printk(KERN_INFO "\tcontrol %x\n", hw->control);
	printk(KERN_INFO "\tstatus %x\n", hw->status);
}
#endif

static int xilinx_dma_wait_status(struct xilinx_dma_chan *chan, uint32_t mask,
		uint32_t value)
{
	unsigned long timeout = 10000;
	uint32_t status;

	do {
		status = DMA_IN(&chan->regs->cr);
		printk("status: %x, mask: %x, value: %x\n", status, mask, value);
		if ((status & mask) == value)
			break;
	} while (--timeout);

	if ((status & mask) != value)
		return -ETIMEDOUT;

	return 0;
}

static int xilinx_dma_reset(struct xilinx_dma_chan *chan)
{
	int ret;

	DMA_OUT(&chan->regs->cr,
	       DMA_IN(&chan->regs->cr) | XILINX_DMA_CR_RESET_MASK);

	ret = xilinx_dma_wait_status(chan, XILINX_DMA_CR_RESET_MASK, 0);

	if (ret) {
		dev_err(chan->dev, "reset timeout, cr %x, sr %x\n",
		    DMA_IN(&chan->regs->cr), DMA_IN(&chan->regs->sr));
		return 1;
	}

	/* re-apply config */
	dmaengine_device_control(&chan->common, DMA_SLAVE_CONFIG,
			(unsigned long)&chan->config);

	return 0;
}

static void xilinx_dma_start_stop(struct xilinx_dma_chan *chan, bool start)
{
	uint32_t status;
	uint32_t value;
	int ret;

	if (start)
		value = XILINX_DMA_CR_RUNSTOP_MASK;
	else
		value = 0;

	status = DMA_IN(&chan->regs->cr);
	status &= ~XILINX_DMA_CR_RUNSTOP_MASK;
	status |= value;
	DMA_OUT(&chan->regs->cr, status);

	ret = xilinx_dma_wait_status(chan, XILINX_DMA_CR_RUNSTOP_MASK, value);
	if (ret) {
		dev_dbg(chan->dev, "Cannot %s channel %x: %x\n",
			start ? "start" : "stop", chan->id,
		    DMA_IN(&chan->regs->cr));
		chan->err = 1;
	}
}

/* Stop the hardware, the ongoing transfer will be finished */
static void dma_halt(struct xilinx_dma_chan *chan)
{
	xilinx_dma_start_stop(chan, false);
}

/* Start the hardware. Transfers are not started yet */
static void dma_start(struct xilinx_dma_chan *chan)
{
	xilinx_dma_start_stop(chan, true);
}

static void xilinx_dma_start_transfer(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_transfer *last_transfer, *first_transfer;
	dma_addr_t first_addr, last_addr;
	struct xilinx_dma_desc_hw *hw;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->pending_list))
		goto out_unlock;

	if (chan->err) {
		dev_err(chan->dev, "Failed to start transfer\n");
		goto out_unlock;
	}

	/* If hardware is busy, cannot submit
	 */
	if (xilinx_dma_is_running(chan) && !xilinx_dma_is_idle(chan)) {
		dev_info(chan->dev, "DMA controller still busy\n");
		goto out_unlock;
	}

	ret = xilinx_dma_wait_idle(chan);
	if (ret)
		xilinx_dma_reset(chan);


	/* If hardware is idle, then all descriptors on active list are
	 * done, start new transfers
	 */
	dma_halt(chan);

	if (chan->err)
		goto out_unlock;

	first_transfer = list_first_entry(&chan->pending_list,
			struct xilinx_dma_transfer, head);

	if (chan->has_SG) {
		uint32_t status;
		last_transfer = list_entry(chan->pending_list.prev,
				struct xilinx_dma_transfer, head);

		first_addr = first_transfer->descs[0].phys;
		last_addr = last_transfer->descs[last_transfer->num_descs-1].phys;

		DMA_OUT(&chan->regs->cdr, first_addr);

		dma_start(chan);

		if (chan->err)
			goto out_unlock;
		list_splice_tail_init(&chan->pending_list, &chan->active_list);

		/* Clear pending interrupts and enable interrupts */
		DMA_OUT(&chan->regs->sr, XILINX_DMA_XR_IRQ_ALL_MASK);
		DMA_OUT(&chan->regs->cr,
			DMA_IN(&chan->regs->cr) | XILINX_DMA_XR_IRQ_ALL_MASK);
		status = DMA_IN(&chan->regs->sr);
		/* Update tail ptr register and start the transfer
		*/
		DMA_OUT(&chan->regs->tdr, last_addr);
	} else {
		/* In simple mode */

		list_move_tail(&first_transfer->head, &chan->active_list);

		dma_start(chan);

		if (chan->err)
			goto out_unlock;

		hw = first_transfer->descs[0].hw;

		/* Enable interrupts
		*/
		DMA_OUT(&chan->regs->cr,
			DMA_IN(&chan->regs->cr) | XILINX_DMA_XR_IRQ_ALL_MASK);

		DMA_OUT(&chan->regs->src, hw->buf_addr);

		/* Start the transfer
		*/
		DMA_OUT(&chan->regs->btt_ref,
			hw->control & XILINX_DMA_MAX_TRANS_LEN);
	}

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

static void xilinx_dma_issue_pending(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	chan->start_transfer(chan);
}

/**
 * xilinx_dma_update_completed_cookie - Update the completed cookie.
 * @chan : xilinx DMA channel
 *
 * CONTEXT: hardirq
 */
static void xilinx_dma_update_completed_cookie(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_desc_hw *hw = NULL;
	unsigned long flags;
	dma_cookie_t cookie = -EBUSY;
	bool done = 0;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->active_list)) {
		dev_dbg(chan->dev, "no running descriptors\n");
		goto out_unlock;
	}

	if ((!(chan->feature & XILINX_DMA_IP_VDMA)) && chan->has_SG) {
		/* Get the last completed descriptor, update the cookie to that */
		list_for_each_entry(t, &chan->active_list, head) {
			if (t->cyclic) {
				while (true) {
					hw = t->descs[t->current_desc].hw;
					if (!(hw->status & XILINX_DMA_BD_STS_ALL_MASK))
						break;
					t->completed_descs++;
					hw->status = 0;
					DMA_OUT(&chan->regs->tdr, t->descs[t->current_desc].phys);

					t->current_desc++;
					if (t->current_desc == t->num_descs)
						t->current_desc = 0;
				}
			} else {
				for (; t->current_desc < t->num_descs; t->current_desc++) {
					hw = t->descs[t->current_desc].hw;
					if (!(hw->status & XILINX_DMA_BD_STS_ALL_MASK))
						break;
				}
				if (t->current_desc != t->num_descs)
					break;

				done = true;
				cookie = t->async_tx.cookie;
			}
		}
	} else {
		/* In non-SG mode, there is only one transfer active at a time */
		t = list_first_entry(&chan->active_list,
				struct xilinx_dma_transfer, head);
		t->current_desc++;
		t->completed_descs++;
		if (t->current_desc == t->num_descs) {
			if (t->cyclic) {
				t->current_desc = 0;
			} else {
				done = true;
				cookie = t->async_tx.cookie;
			}
		}
	}

	if (done)
		chan->completed_cookie = cookie;

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

static irqreturn_t dma_intr_handler(int irq, void *data)
{
	struct xilinx_dma_chan *chan = data;
	u32 stat;

	stat = DMA_IN(&chan->regs->sr);
	if (!(stat & XILINX_DMA_XR_IRQ_ALL_MASK))
		return IRQ_NONE;

	/* Ack the interrupts
	 */
	DMA_OUT(&chan->regs->sr, XILINX_DMA_XR_IRQ_ALL_MASK);

	if (stat & XILINX_DMA_XR_IRQ_ERROR_MASK) {
		dev_err(chan->dev, "Channel %x has errors %x, cr %x, cdr %x tdr %x\n",
		    (unsigned int)chan, (unsigned int)stat,
		    (unsigned int)DMA_IN(&chan->regs->cr),
		    (unsigned int)DMA_IN(&chan->regs->cdr),
		    (unsigned int)DMA_IN(&chan->regs->tdr));
		chan->err = 1;
		dma_halt(chan);
	}

	/* Device takes too long to do the transfer when user requires
	 * responsiveness
	 */
	if (stat & XILINX_DMA_XR_IRQ_DELAY_MASK)
		dev_dbg(chan->dev, "Inter-packet latency too long\n");

	if (stat & XILINX_DMA_XR_IRQ_IOC_MASK) {
		xilinx_dma_update_completed_cookie(chan);
		chan->start_transfer(chan);
	}

	tasklet_schedule(&chan->tasklet);
	return IRQ_HANDLED;
}

static void dma_do_tasklet(unsigned long data)
{
	struct xilinx_dma_chan *chan = (struct xilinx_dma_chan *)data;
	xilinx_chan_desc_cleanup(chan);
}

/* Append the descriptor list to the pending list */
static void append_desc_queue(struct xilinx_dma_chan *chan,
			struct xilinx_dma_transfer *t)
{
	struct xilinx_dma_transfer *tail = container_of(chan->pending_list.prev,
					struct xilinx_dma_transfer, head);
	struct xilinx_dma_desc_hw *hw;

	if (!list_empty(&chan->pending_list)) {
		/* Add the hardware descriptor to the chain of hardware descriptors
		 * that already exists in memory.
		 */
		hw = tail->descs[tail->num_descs-1].hw;
		hw->next_desc = t->descs[0].phys;
	}

	/* Add the software descriptor and all children to the list
	 * of pending transactions
	 */
	list_add_tail(&t->head, &chan->pending_list);
}

/* Assign cookie to each descriptor, and append the descriptors to the pending
 * list
 */
static dma_cookie_t xilinx_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(tx->chan);
	struct xilinx_dma_transfer *t = container_of(tx,
				struct xilinx_dma_transfer, async_tx);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	if (chan->cyclic)
		goto err;

	if (chan->err) {
		/* If reset fails, need to hard reset the system.
		 * Channel is no longer functional
		 */
		if (!xilinx_dma_reset(chan))
			chan->err = 0;
		else
			goto err;
	}

	t->async_tx.cookie = dma_chan_generate_cookie(&chan->common);

	/* put this transaction onto the tail of the pending queue */
	append_desc_queue(chan, t);

	if (t->cyclic)
		chan->cyclic = true;

	spin_unlock_irqrestore(&chan->lock, flags);

	return t->async_tx.cookie;
err:
	spin_unlock_irqrestore(&chan->lock, flags);
	xilinx_dma_free_transfer(chan, t);
	return -EBUSY;
}

/**
 * xilinx_dma_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: DMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: transfer ack flags
 */
static struct dma_async_tx_descriptor *xilinx_dma_prep_dma_cyclic(
	struct dma_chan *dchan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction)
{
	struct xilinx_dma_desc_hw *hw;
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_chan *chan;
	unsigned int num_periods;
	unsigned int i;

	if (!dchan)
		return NULL;

	chan = to_xilinx_chan(dchan);

	if (chan->direction != direction)
		return NULL;

	num_periods = buf_len / period_len;

	t = xilinx_dma_alloc_transfer(chan, num_periods);
	if (!t)
		return NULL;

	for (i = 0; i < num_periods; ++i) {
		hw = t->descs[i].hw;
		hw->buf_addr = buf_addr;
		hw->control = period_len;
		hw->control |= XILINX_DMA_BD_SOP | XILINX_DMA_BD_EOP;
		buf_addr += period_len;
	}

	t->cyclic = true;

	return &t->async_tx;
}


/**
 * xilinx_dma_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: DMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: transfer ack flags
 */
static struct dma_async_tx_descriptor *xilinx_dma_prep_slave_sg(
	struct dma_chan *dchan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags)
{
	struct xilinx_dma_desc_hw *hw;
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_chan *chan;
	unsigned int total_len = 0;
	unsigned int num_descs = 0;
	struct scatterlist *sg;
	dma_addr_t dma_src;
	size_t num_bytes;
	size_t sg_used;
	unsigned int i, j;

	if (!dchan)
		return NULL;

	chan = to_xilinx_chan(dchan);

	if (chan->direction != direction)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		total_len += sg_dma_len(sg);
		num_descs += DIV_ROUND_UP(sg_dma_len(sg), chan->max_len);
	}

	t = xilinx_dma_alloc_transfer(chan, num_descs);
	if (!t)
		return NULL;

	/*
	 * Build transactions using information in the scatter gather list
	 */
	j = 0;
	for_each_sg(sgl, sg, sg_len, i) {
		sg_used = 0;

		/* Loop until the entire scatterlist entry is used */
		while (sg_used < sg_dma_len(sg)) {
			/*
			 * Calculate the maximum number of bytes to transfer,
			 * making sure it is less than the hw limit
			 */
			num_bytes = min_t(size_t, sg_dma_len(sg) - sg_used,
					chan->max_len);

			dma_src = sg_dma_address(sg) + sg_used;

			hw = t->descs[j].hw;
			hw->buf_addr = dma_src;
			hw->control = num_bytes;
			j++;
		}
	}

	/* Set EOP to the last link descriptor of new list and
	   SOP to the first link descriptor. */
	t->descs[0].hw->control |= XILINX_DMA_BD_SOP;
	t->descs[t->num_descs-1].hw->control |= XILINX_DMA_BD_EOP;

	t->async_tx.flags = flags;

	return &t->async_tx;
}

/**
 * xilinx_vdma_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: VDMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: transfer ack flags
 */
static struct dma_async_tx_descriptor *xilinx_vdma_prep_slave_sg(
	struct dma_chan *dchan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags)
{
	struct xilinx_dma_desc_hw *hw;
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_chan *chan;
	struct scatterlist *sg;
	unsigned int i, j;
	dma_addr_t dma_src;

	if (!dchan)
		return NULL;

	chan = to_xilinx_chan(dchan);

	if (chan->direction != direction)
		return NULL;

	/* Enforce one sg entry for one frame */
	if (chan->num_frms % sg_len != 0) {
		dev_err(chan->dev, "number of entries %d not the "
		    "same as num stores %d\n", sg_len, chan->num_frms);

		return NULL;
	}

	t = xilinx_dma_alloc_transfer(chan, sg_len);
	if (!t)
		return NULL;

	if (!chan->has_SG) {
		DMA_OUT(&chan->addr_regs->hsize, chan->config.hsize);
		DMA_OUT(&chan->addr_regs->frmdly_stride,
		     chan->config.frm_dly << XILINX_VDMA_FRMDLY_SHIFT |
		     chan->config.stride);
	}

	for (j = 0; j < chan->num_frms / sg_len; ++j) {
		/* Build transactions using information in the scatter gather list
		 */
		for_each_sg(sgl, sg, sg_len, i) {
			dma_src = sg_dma_address(sg);
			if (chan->has_SG) {
				hw = t->descs[j * sg_len + i].hw;
				hw->buf_addr = dma_src;

				/* Fill in the descriptor */
				hw->addr_vsize = chan->config.vsize;
				hw->hsize = chan->config.hsize;
				hw->control = (chan->config.frm_dly <<
						XILINX_VDMA_FRMDLY_SHIFT) |
						chan->config.stride;
			} else {
				/* Update the registers */
				DMA_OUT(&(chan->addr_regs->buf_addr[j * sg_len + i]), dma_src);
			}
		}
	}

	t->async_tx.flags = flags;

	return &t->async_tx;
}

static void xilinx_vdma_start_transfer(struct xilinx_dma_chan *chan)
{
	unsigned long flags;
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_config *config;
	u32 reg;

	if (chan->err)
		return;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->pending_list))
		goto out_unlock;

	/* If it is SG mode and hardware is busy, cannot submit
	 */
	if (chan->has_SG && xilinx_dma_is_running(chan) && !xilinx_dma_is_idle(chan)) {
		dev_dbg(chan->dev, "DMA controller still busy\n");
		goto out_unlock;
	}

	/* If hardware is idle, then all descriptors on the running lists are
	 * done, start new transfers
	 */
	dma_halt(chan);

	if (chan->err)
		goto out_unlock;

	t = list_first_entry(&chan->pending_list, struct xilinx_dma_transfer, head);

	if (chan->has_SG)
		DMA_OUT(&chan->regs->cdr, t->descs[0].phys);

	/* Configure the hardware using info in the config structure */
	config = &chan->config;
	reg = DMA_IN(&chan->regs->cr);

	if (config->frm_cnt_en)
		reg |= XILINX_VDMA_FRMCNT_EN;
	else
		reg &= ~XILINX_VDMA_FRMCNT_EN;

	/* With SG, start with circular mode, so that BDs can be fetched.
	 * In direct register mode, if not parking, enable circular mode */
	if ((chan->has_SG) || (!config->park))
		reg |= XILINX_VDMA_CIRC_EN;

	DMA_OUT(&chan->regs->cr, reg);

	if ((config->park_frm >= 0) && (config->park_frm < chan->num_frms)) {
		if (config->direction == DMA_MEM_TO_DEV) {
			DMA_OUT(&chan->regs->btt_ref,
			    config->park_frm << XILINX_VDMA_WR_REF_SHIFT);
		} else {
			DMA_OUT(&chan->regs->btt_ref, config->park_frm);
		}
	}

	/* Start the hardware
	 */
	dma_start(chan);

	if (chan->err)
		goto out_unlock;
	list_splice_tail_init(&chan->pending_list, &chan->active_list);

	/* Enable interrupts
	 *
	 * park/genlock testing does not use interrupts */
	/*
	if (!chan->config.disable_intr) {
		DMA_OUT(&chan->regs->cr,
		   DMA_IN(&chan->regs->cr) | XILINX_DMA_XR_IRQ_ALL_MASK);
	}*/

	/* Start the transfer
	 */
	if (chan->has_SG)
		DMA_OUT(&chan->regs->tdr, t->descs[t->num_descs-1].phys);
	else
		DMA_OUT(&chan->addr_regs->vsize, config->vsize);

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

static int xilinx_dma_terminate_all(struct xilinx_dma_chan *chan)
{
	/* Disable intr
	 */
	DMA_OUT(&chan->regs->cr,
	   DMA_IN(&chan->regs->cr) & ~XILINX_DMA_XR_IRQ_ALL_MASK);

	/* Halt the DMA engine */
	dma_halt(chan);
	xilinx_dma_free_transfers(chan);
	chan->cyclic = false;

	return 0;
}

/* Run-time configuration for Axi VDMA, supports:
 * . halt the channel
 * . configure interrupt coalescing and inter-packet delay threshold
 * . start/stop parking
 * . enable genlock
 * . set transfer information using config struct
 */
static int xilinx_vdma_device_control(struct dma_chan *dchan,
				  enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct xilinx_dma_chan *chan;
	struct xilinx_dma_config *cfg = (struct xilinx_dma_config *)arg;
	u32 reg;

	if (!dchan)
		return -EINVAL;

	chan = to_xilinx_chan(dchan);

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		return xilinx_dma_terminate_all(chan);
	case DMA_SLAVE_CONFIG:
		reg = DMA_IN(&chan->regs->cr);

		/* If vsize is -1, it is park-related operations */
		if (cfg->vsize == -1) {
			if (cfg->park)
				reg &= ~XILINX_VDMA_CIRC_EN;
			else
				reg |= XILINX_VDMA_CIRC_EN;

			DMA_OUT(&chan->regs->cr, reg);
			return 0;
		}

		/* If hsize is -1, it is interrupt threshold settings */
		if (cfg->hsize == -1) {
			if (cfg->coalesc <= XILINX_DMA_COALESCE_MAX) {
				reg &= ~XILINX_DMA_XR_COALESCE_MASK;
				reg |= cfg->coalesc <<
					XILINX_DMA_COALESCE_SHIFT;
			}

			if (cfg->delay <= XILINX_DMA_DELAY_MAX) {
				reg &= ~XILINX_DMA_XR_DELAY_MASK;
				reg |= cfg->delay << XILINX_DMA_DELAY_SHIFT;
			}

			DMA_OUT(&chan->regs->cr, reg);
			return 0;
		}

		/* Transfer information */
		chan->config = *cfg;

		if (cfg->gen_lock) {
			if (chan->genlock) {
				reg |= XILINX_VDMA_SYNC_EN;
				reg |= cfg->master << XILINX_VDMA_MSTR_SHIFT;
			}
		}

		if (cfg->coalesc <= XILINX_DMA_COALESCE_MAX)
			reg |= cfg->coalesc << XILINX_DMA_COALESCE_SHIFT;

		if (cfg->delay <= XILINX_DMA_DELAY_MAX)
			reg |= cfg->delay << XILINX_DMA_DELAY_SHIFT;

		DMA_OUT(&chan->regs->cr, reg);
		break;
	default:
		return -ENXIO;
	}

	return 0;
}


/* Run-time device configuration for Axi DMA and Axi CDMA */
static int xilinx_dma_device_control(struct dma_chan *dchan,
				  enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct xilinx_dma_chan *chan;
	struct xilinx_dma_config *cfg = (struct xilinx_dma_config *)arg;
	u32 reg;

	if (!dchan)
		return -EINVAL;

	chan = to_xilinx_chan(dchan);

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		return xilinx_dma_terminate_all(chan);
	case DMA_SLAVE_CONFIG:
		/* Configure interrupt coalescing and delay counter
		 * Use value XILINX_DMA_NO_CHANGE to signal no change
		 */
		reg = DMA_IN(&chan->regs->cr);

		chan->config = *cfg;

		if (cfg->coalesc <= XILINX_DMA_COALESCE_MAX) {
			reg &= ~XILINX_DMA_XR_COALESCE_MASK;
			reg |= cfg->coalesc << XILINX_DMA_COALESCE_SHIFT;
		}

		if (cfg->delay <= XILINX_DMA_DELAY_MAX) {
			reg &= ~XILINX_DMA_XR_DELAY_MASK;
			reg |= cfg->delay << XILINX_DMA_DELAY_SHIFT;
		}

		DMA_OUT(&chan->regs->cr, reg);

		break;
	default:
		return -ENXIO;
	}

	return 0;
}

static void xilinx_dma_chan_remove(struct xilinx_dma_chan *chan)
{
	irq_dispose_mapping(chan->irq);
	list_del(&chan->common.device_node);
	kfree(chan);
}

/*
 * Probing channels
 *
 * . Get channel features from the device tree entry
 * . Initialize special channel handling routines
 */
static int __devinit xilinx_dma_chan_probe(struct xilinx_dma_device *xdev,
	struct device_node *node, u32 feature)
{
	struct xilinx_dma_chan *chan;
	u32 width = 0;
	int ret;

	/* alloc channel */
	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan) {
		dev_err(xdev->dev, "no free memory for DMA channels!\n");
		ret = -ENOMEM;
		goto out_return;
	}

	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->active_list);

	chan->feature = feature;
	chan->has_DRE = 0;
	chan->has_SG = 0;
	chan->max_len = XILINX_DMA_MAX_TRANS_LEN;

	of_property_read_u32(node, "xlnx,include-dre", &chan->has_DRE);
	of_property_read_u32(node, "xlnx,genlock-mode", &chan->genlock);
	of_property_read_u32(node, "xlnx,datawidth", &width);

	if (width > 0) {
		width >>= 3; /* convert bits to bytes */

		/* If data width is greater than 8 bytes, DRE is not in hw */
		if (width > 8)
			chan->has_DRE = 0;

		chan->feature |= width - 1;
	}

	chan->has_SG = (xdev->feature & XILINX_DMA_FTR_HAS_SG) >>
				XILINX_DMA_FTR_HAS_SG_SHIFT;

	if (feature & XILINX_DMA_IP_DMA) {
		chan->start_transfer = xilinx_dma_start_transfer;

		if (of_device_is_compatible(node, "xlnx,axi-dma-mm2s-channel"))
			chan->direction = DMA_MEM_TO_DEV;
		else if (of_device_is_compatible(node, "xlnx,axi-dma-s2mm-channel"))
			chan->direction = DMA_DEV_TO_MEM;
	} else if (feature & XILINX_DMA_IP_VDMA) {
		chan->start_transfer = xilinx_vdma_start_transfer;

		if (of_device_is_compatible(node,
				"xlnx,axi-vdma-mm2s-channel")) {
			chan->direction = DMA_MEM_TO_DEV;
			if (!chan->has_SG) {
				chan->addr_regs = (struct vdma_addr_regs *)
				    ((u32)xdev->regs +
					 XILINX_VDMA_DIRECT_REG_OFFSET);
			}
		}

		if (of_device_is_compatible(node,
				"xlnx,axi-vdma-s2mm-channel")) {
			chan->direction = DMA_DEV_TO_MEM;
			if (!chan->has_SG) {
				chan->addr_regs = (struct vdma_addr_regs *)
				    ((u32)xdev->regs +
					XILINX_VDMA_DIRECT_REG_OFFSET +
					XILINX_VDMA_CHAN_DIRECT_REG_SIZE);
			}
		}
	}

	if (chan->direction == DMA_MEM_TO_DEV) {
		chan->regs = (struct xdma_regs *)xdev->regs;
		chan->id = 0;
	} else {
		chan->regs = (struct xdma_regs *)((u32)xdev->regs +
					XILINX_DMA_RX_CHANNEL_OFFSET);
		chan->id = 1;
	}

	chan->debugfs_regset.regs = xilinx_dma_debugfs_regs;
	chan->debugfs_regset.nregs = ARRAY_SIZE(xilinx_dma_debugfs_regs);
	chan->debugfs_regset.base = (void __iomem *)chan->regs;

	/* Used by dmatest channel matching in slave transfers
	 * Can change it to be a structure to have more matching information
	 */
	chan->common.private = (chan->direction & 0xFF) |
		(chan->feature & XILINX_DMA_IP_MASK);

	if (!chan->has_DRE)
		xdev->common.copy_align = ilog2(width);

	chan->dev = xdev->dev;
	xdev->chan[chan->id] = chan;

	tasklet_init(&chan->tasklet, dma_do_tasklet, (unsigned long)chan);

	chan->common.device = &xdev->common;

	/* find the IRQ line, if it exists in the device tree */
	chan->irq = irq_of_parse_and_map(node, 0);
	ret = request_irq(chan->irq, dma_intr_handler, IRQF_SHARED,
				"xilinx-dma-controller", chan);
	if (ret) {
		dev_err(xdev->dev, "unable to request IRQ %d\n", ret);
		goto out_free_irq;
	}

	/* Add the channel to DMA device channel list */
	list_add_tail(&chan->common.device_node, &xdev->common.channels);

	/* Initialize the channel */
	if (xilinx_dma_reset(chan)) {
		dev_err(xdev->dev, "Reset channel failed\n");
		goto out_free_chan;
	}

	return 0;
out_free_irq:
	irq_dispose_mapping(chan->irq);
out_free_chan:
	kfree(chan);
out_return:
	return ret;
}

static int __devinit xilinx_dma_of_probe(struct platform_device *pdev)
{
	struct xilinx_dma_device *xdev;
	struct device_node *child, *node;
	struct xilinx_dma_chan *chan;
	u32 num_frames = 0;
	u32 include_sg = 0;
	unsigned int i;
	int *value;
	int ret;

	xdev = devm_kzalloc(&pdev->dev, sizeof(*xdev), GFP_KERNEL);
	if (!xdev) {
		dev_err(&pdev->dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	xdev->dev = &pdev->dev;
	INIT_LIST_HEAD(&xdev->common.channels);

	node = pdev->dev.of_node;
	xdev->feature = 0;

	/* iomap registers */
	xdev->regs = of_iomap(node, 0);
	if (!xdev->regs) {
		dev_err(&pdev->dev, "unable to iomap registers\n");
		return -ENXIO;
	}

	/* Axi DMA and VDMA only do slave transfers
	 */
	if (of_device_is_compatible(node, "xlnx,axi-dma")) {
		xdev->feature |= XILINX_DMA_IP_DMA;
		value = (int *)of_get_property(node,
				"xlnx,sg-include-stscntrl-strm",
				NULL);
		if (value) {
			xdev->feature |= XILINX_DMA_FTR_HAS_SG;
			if (be32_to_cpup(value) == 1)
				xdev->feature = XILINX_DMA_FTR_STSCNTRL_STRM;
		}

		dma_cap_set(DMA_SLAVE, xdev->common.cap_mask);
		dma_cap_set(DMA_PRIVATE, xdev->common.cap_mask);
		dma_cap_set(DMA_CYCLIC, xdev->common.cap_mask);
		xdev->common.device_prep_slave_sg = xilinx_dma_prep_slave_sg;
		xdev->common.device_prep_dma_cyclic = xilinx_dma_prep_dma_cyclic;
		xdev->common.device_control = xilinx_dma_device_control;
		xdev->common.device_issue_pending = xilinx_dma_issue_pending;
	}

	if (of_device_is_compatible(node, "xlnx,axi-vdma")) {
		xdev->feature |= XILINX_DMA_IP_VDMA;

		of_property_read_u32(node, "xlnx,include-sg", &include_sg);
		if (include_sg)
		    xdev->feature |= XILINX_DMA_FTR_HAS_SG;

		of_property_read_u32(node, "xlnx,num-fstores", &num_frames);

		dma_cap_set(DMA_SLAVE, xdev->common.cap_mask);
		dma_cap_set(DMA_PRIVATE, xdev->common.cap_mask);
		xdev->common.device_prep_slave_sg = xilinx_vdma_prep_slave_sg;
		xdev->common.device_control = xilinx_vdma_device_control;
		xdev->common.device_issue_pending = xilinx_dma_issue_pending;
	}

	xdev->common.device_alloc_chan_resources =
				xilinx_dma_alloc_chan_resources;
	xdev->common.device_free_chan_resources =
				xilinx_dma_free_chan_resources;
	xdev->common.device_tx_status = xilinx_tx_status;
	xdev->common.dev = &pdev->dev;

	for_each_child_of_node(node, child) {
		ret = xilinx_dma_chan_probe(xdev, child, xdev->feature);
		if (ret)
			goto err_free_chan;
	}

	if (xdev->feature & XILINX_DMA_IP_VDMA) {
		for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++) {
			if (xdev->chan[i])
				xdev->chan[i]->num_frms = num_frames;
		}
	}

	ret = dma_async_device_register(&xdev->common);
	if (ret)
		goto err_free_chan;

	for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++) {
		if (xdev->chan[i]) {
			chan = xdev->chan[i];
			debugfs_create_regset32(dev_name(&chan->common.dev->device), S_IRUGO, NULL, &chan->debugfs_regset);
		}
	}

	platform_set_drvdata(pdev, xdev);

	return 0;

err_free_chan:
	for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++) {
		if (xdev->chan[i])
			xilinx_dma_chan_remove(xdev->chan[i]);
	}

	return ret;
}

static int __devexit xilinx_dma_of_remove(struct platform_device *pdev)
{
	struct xilinx_dma_device *xdev = platform_get_drvdata(pdev);
	int i;

	dma_async_device_unregister(&xdev->common);

	for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++) {
		if (xdev->chan[i])
			xilinx_dma_chan_remove(xdev->chan[i]);
	}

	iounmap(xdev->regs);

	return 0;
}

static const struct of_device_id xilinx_dma_of_ids[] = {
	{ .compatible = "xlnx,axi-dma" },
	{ .compatible = "xlnx,axi-vdma" },
	{}
};

static struct platform_driver xilinx_dma_of_driver = {
	.driver = {
		.name = "xilinx-dma",
		.owner = THIS_MODULE,
		.of_match_table = xilinx_dma_of_ids,
	},
	.probe = xilinx_dma_of_probe,
	.remove = __devexit_p(xilinx_dma_of_remove),
};

static int __init xilinx_dma_init(void)
{
	return platform_driver_register(&xilinx_dma_of_driver);
}
subsys_initcall(xilinx_dma_init);

static void __exit xilinx_dma_exit(void)
{
	platform_driver_unregister(&xilinx_dma_of_driver);
}
module_exit(xilinx_dma_exit);

MODULE_DESCRIPTION("Xilinx DMA/VDMA driver");
MODULE_LICENSE("GPL");
