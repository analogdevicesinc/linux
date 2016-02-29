/*
 * Driver for the Analog Devices AXI-DMAC core
 *
 * Copyright 2013-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <dt-bindings/dma/axi-dmac.h>

#include "dmaengine.h"
#include "virt-dma.h"

/*
 * The AXI-DMAC is a soft IP core that is used in FPGA designs. The core has
 * various instantiation parameters which decided the exact feature set support
 * by the core.
 *
 * Each channel of the core has a source interface and a destination interface.
 * The number of channels and the type of the channel interfaces is selected at
 * configuration time. A interface can either be a connected to a central memory
 * interconnect, which allows access to system memory, or it can be connected to
 * a dedicated bus which is directly connected to a data port on a peripheral.
 * Given that those are configuration options of the core that are selected when
 * it is instantiated this means that they can not be changed by software at
 * runtime. By extension this means that each channel is uni-directional. It can
 * either be device to memory or memory to device, but not both. Also since the
 * device side is a dedicated data bus only connected to a single peripheral
 * there is no address than can or needs to be configured for the device side.
 */

#define AXI_DMAC_REG_IRQ_MASK		0x80
#define AXI_DMAC_REG_IRQ_PENDING	0x84
#define AXI_DMAC_REG_IRQ_SOURCE		0x88

#define AXI_DMAC_REG_CTRL		0x400
#define AXI_DMAC_REG_TRANSFER_ID	0x404
#define AXI_DMAC_REG_START_TRANSFER	0x408
#define AXI_DMAC_REG_FLAGS		0x40c
#define AXI_DMAC_REG_DEST_ADDRESS	0x410
#define AXI_DMAC_REG_SRC_ADDRESS	0x414
#define AXI_DMAC_REG_X_LENGTH		0x418
#define AXI_DMAC_REG_Y_LENGTH		0x41c
#define AXI_DMAC_REG_DEST_STRIDE	0x420
#define AXI_DMAC_REG_SRC_STRIDE		0x424
#define AXI_DMAC_REG_TRANSFER_DONE	0x428
#define AXI_DMAC_REG_ACTIVE_TRANSFER_ID 0x42c
#define AXI_DMAC_REG_STATUS		0x430
#define AXI_DMAC_REG_CURRENT_SRC_ADDR	0x434
#define AXI_DMAC_REG_CURRENT_DEST_ADDR	0x438
#define AXI_DMAC_REG_DBG0		0x43c
#define AXI_DMAC_REG_DBG1		0x440

#define AXI_DMAC_CTRL_ENABLE		BIT(0)
#define AXI_DMAC_CTRL_PAUSE		BIT(1)

#define AXI_DMAC_IRQ_SOT		BIT(0)
#define AXI_DMAC_IRQ_EOT		BIT(1)

#define AXI_DMAC_FLAG_CYCLIC		BIT(0)

#undef SPEED_TEST

struct axi_dmac_sg {
	dma_addr_t src_addr;
	dma_addr_t dest_addr;
	unsigned int x_len;
	unsigned int y_len;
	unsigned int dest_stride;
	unsigned int src_stride;
	unsigned int id;
};

struct axi_dmac_desc {
	struct virt_dma_desc vdesc;
	bool cyclic;

	unsigned int num_submitted;
	unsigned int num_completed;
	unsigned int num_sgs;
	struct axi_dmac_sg sg[];
};

struct axi_dmac_chan {
	struct virt_dma_chan vchan;

	struct axi_dmac_desc *next_desc;
	struct list_head active_descs;
	enum dma_transfer_direction direction;

	unsigned int src_width;
	unsigned int dest_width;
	unsigned int src_type;
	unsigned int dest_type;

	unsigned int max_length;
	unsigned int align_mask;

	bool hw_cyclic;
	bool hw_2d;
};

struct axi_dmac {
	void __iomem *base;
	int irq;

	struct clk *clk;

	struct dma_device dma_dev;
	struct axi_dmac_chan chan;

	struct device_dma_parameters dma_parms;

#ifdef SPEED_TEST
	void *test_virt;
	dma_addr_t test_phys;
#endif
};

static struct axi_dmac *chan_to_axi_dmac(struct axi_dmac_chan *chan)
{
	return container_of(chan->vchan.chan.device, struct axi_dmac,
		dma_dev);
}

static struct axi_dmac_chan *to_axi_dmac_chan(struct dma_chan *c)
{
	return container_of(c, struct axi_dmac_chan, vchan.chan);
}

static struct axi_dmac_desc *to_axi_dmac_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct axi_dmac_desc, vdesc);
}

static void axi_dmac_write(struct axi_dmac *axi_dmac, unsigned int reg,
	unsigned int val)
{
	writel(val, axi_dmac->base + reg);
}

static int axi_dmac_read(struct axi_dmac *axi_dmac, unsigned int reg)
{
	return readl(axi_dmac->base + reg);
}

static int axi_dmac_src_is_mem(struct axi_dmac_chan *chan)
{
	return chan->src_type == AXI_DMAC_BUS_TYPE_AXI_MM;
}

static int axi_dmac_dest_is_mem(struct axi_dmac_chan *chan)
{
	return chan->dest_type == AXI_DMAC_BUS_TYPE_AXI_MM;
}

static bool axi_dmac_check_len(struct axi_dmac_chan *chan, unsigned int len)
{
	if (len == 0 || len > chan->max_length)
		return false;
	if ((len & chan->align_mask) != 0) /* Not aligned */
		return false;
	return true;
}

static bool axi_dmac_check_addr(struct axi_dmac_chan *chan, dma_addr_t addr)
{
	if ((addr & chan->align_mask) != 0) /* Not aligned */
		return false;
	return true;
}

static void axi_dmac_start_transfer(struct axi_dmac_chan *chan)
{
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	struct virt_dma_desc *vdesc;
	struct axi_dmac_desc *desc;
	struct axi_dmac_sg *sg;
	unsigned int flags = 0;
	unsigned int val;

	val = axi_dmac_read(dmac, AXI_DMAC_REG_START_TRANSFER);
	if (val) /* Queue is full, wait for the next SOT IRQ */
		return;

	desc = chan->next_desc;

	if (!desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return;
		list_move_tail(&vdesc->node, &chan->active_descs);
		desc = to_axi_dmac_desc(vdesc);
	}
	sg = &desc->sg[desc->num_submitted];

	desc->num_submitted++;
	if (desc->num_submitted == desc->num_sgs)
		chan->next_desc = NULL;
	else
		chan->next_desc = desc;

	sg->id = axi_dmac_read(dmac, AXI_DMAC_REG_TRANSFER_ID);

	if (axi_dmac_dest_is_mem(chan)) {
		axi_dmac_write(dmac, AXI_DMAC_REG_DEST_ADDRESS, sg->dest_addr);
		axi_dmac_write(dmac, AXI_DMAC_REG_DEST_STRIDE, sg->dest_stride);
	}

	if (axi_dmac_src_is_mem(chan)) {
		axi_dmac_write(dmac, AXI_DMAC_REG_SRC_ADDRESS, sg->src_addr);
		axi_dmac_write(dmac, AXI_DMAC_REG_SRC_STRIDE, sg->src_stride);
	}

	/*
	 * If the hardware supports cyclic transfers and there is no callback to
	 * call, enable hw cyclic mode to avoid unnecessary interrupts.
	 */
	if (chan->hw_cyclic && desc->cyclic && !desc->vdesc.tx.callback)
		flags |= AXI_DMAC_FLAG_CYCLIC;

	axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, sg->x_len - 1);
	axi_dmac_write(dmac, AXI_DMAC_REG_Y_LENGTH, sg->y_len - 1);
	axi_dmac_write(dmac, AXI_DMAC_REG_FLAGS, flags);
	axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);
}

static struct axi_dmac_desc *axi_dmac_active_desc(struct axi_dmac_chan *chan)
{
	return list_first_entry_or_null(&chan->active_descs,
		struct axi_dmac_desc, vdesc.node);
}

static void axi_dmac_transfer_done(struct axi_dmac_chan *chan,
	unsigned int completed_transfers)
{
	struct axi_dmac_desc *active;
	struct axi_dmac_sg *sg;

	active = axi_dmac_active_desc(chan);
	if (!active)
		return;

	if (active->cyclic) {
		vchan_cyclic_callback(&active->vdesc);
	} else {
		do {
			sg = &active->sg[active->num_completed];
			if (!(BIT(sg->id) & completed_transfers))
				break;
			active->num_completed++;
			if (active->num_completed == active->num_sgs) {
				list_del(&active->vdesc.node);
				vchan_cookie_complete(&active->vdesc);
				active = axi_dmac_active_desc(chan);
			}
		} while (active);
	}
}

#ifdef SPEED_TEST
static s64 get_time(void)
{
	struct timespec ts;
	ktime_get_real_ts(&ts);

	return timespec_to_ns(&ts);
}

static s64 start;
static unsigned int count;

static irqreturn_t axi_dmac_interrupt_handler(int irq, void *devid)
{
	struct axi_dmac *dmac = devid;
	unsigned int pending;

	pending = axi_dmac_read(dmac, AXI_DMAC_REG_IRQ_PENDING);
	axi_dmac_write(dmac, AXI_DMAC_REG_IRQ_PENDING, pending);

	if (pending & 1) {
		if (count == 0)
			start = get_time();
		if (count < 100) {
			axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);
			count += 1;
		}
	} else if ((pending & 2) && count == 100) {
		printk("time: %lld %x\n", get_time() - start, pending);
	}

	return IRQ_HANDLED;
}
#else
static irqreturn_t axi_dmac_interrupt_handler(int irq, void *devid)
{
	struct axi_dmac *dmac = devid;
	unsigned int pending;

	pending = axi_dmac_read(dmac, AXI_DMAC_REG_IRQ_PENDING);
	axi_dmac_write(dmac, AXI_DMAC_REG_IRQ_PENDING, pending);

	spin_lock(&dmac->chan.vchan.lock);
	/* One or more transfers have finished */
	if (pending & AXI_DMAC_IRQ_EOT) {
		unsigned int completed;

		completed = axi_dmac_read(dmac, AXI_DMAC_REG_TRANSFER_DONE);
		axi_dmac_transfer_done(&dmac->chan, completed);
	}
	/* Space has become available in the descriptor queue */
	if (pending & AXI_DMAC_IRQ_SOT)
		axi_dmac_start_transfer(&dmac->chan);
	spin_unlock(&dmac->chan.vchan.lock);

	return IRQ_HANDLED;
}
#endif

static int axi_dmac_terminate_all(struct dma_chan *c)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vchan.lock, flags);
	axi_dmac_write(dmac, AXI_DMAC_REG_CTRL, 0);
	chan->next_desc = NULL;
	vchan_get_all_descriptors(&chan->vchan, &head);
	list_splice_tail_init(&chan->active_descs, &head);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	vchan_dma_desc_free_list(&chan->vchan, &head);

	return 0;
}

static void axi_dmac_issue_pending(struct dma_chan *c)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	unsigned long flags;

	axi_dmac_write(dmac, AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);

	spin_lock_irqsave(&chan->vchan.lock, flags);
	if (vchan_issue_pending(&chan->vchan))
		axi_dmac_start_transfer(chan);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static struct axi_dmac_desc *axi_dmac_alloc_desc(unsigned int num_sgs)
{
	struct axi_dmac_desc *desc;

	desc = kzalloc(sizeof(struct axi_dmac_desc) +
		sizeof(struct axi_dmac_sg) * num_sgs, GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->num_sgs = num_sgs;

	return desc;
}

static struct dma_async_tx_descriptor *axi_dmac_prep_slave_sg(
	struct dma_chan *c, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac_desc *desc;
	struct scatterlist *sg;
	unsigned int i;

	if (direction != chan->direction)
		return NULL;

	desc = axi_dmac_alloc_desc(sg_len);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		if (!axi_dmac_check_addr(chan, sg_dma_address(sg)) ||
		    !axi_dmac_check_len(chan, sg_dma_len(sg))) {
			kfree(desc);
			return NULL;
		}

		if (direction == DMA_DEV_TO_MEM)
			desc->sg[i].dest_addr = sg_dma_address(sg);
		else
			desc->sg[i].src_addr = sg_dma_address(sg);
		desc->sg[i].x_len = sg_dma_len(sg);
		desc->sg[i].y_len = 1;
	}

	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *axi_dmac_prep_dma_cyclic(
	struct dma_chan *c, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac_desc *desc;
	unsigned int num_periods, i;

	if (direction != chan->direction)
		return NULL;

	if (!axi_dmac_check_len(chan, buf_len) ||
	    !axi_dmac_check_addr(chan, buf_addr))
		return NULL;

	if (period_len == 0 || buf_len % period_len)
		return NULL;

	num_periods = buf_len / period_len;

	desc = axi_dmac_alloc_desc(num_periods);
	if (!desc)
		return NULL;

	for (i = 0; i < num_periods; i++) {
		if (direction == DMA_DEV_TO_MEM)
			desc->sg[i].dest_addr = buf_addr;
		else
			desc->sg[i].src_addr = buf_addr;
		desc->sg[i].x_len = period_len;
		desc->sg[i].y_len = 1;
		buf_addr += period_len;
	}

	desc->cyclic = true;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *axi_dmac_prep_interleaved(
	struct dma_chan *c, struct dma_interleaved_template *xt,
	unsigned long flags)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac_desc *desc;
	size_t dst_icg, src_icg;

	if (xt->frame_size != 1)
		return NULL;

	if (xt->dir != chan->direction)
		return NULL;

	if (axi_dmac_src_is_mem(chan)) {
		if (!xt->src_inc || !axi_dmac_check_addr(chan, xt->src_start))
			return NULL;
	}

	if (axi_dmac_dest_is_mem(chan)) {
		if (!xt->dst_inc || !axi_dmac_check_addr(chan, xt->dst_start))
			return NULL;
	}

	dst_icg = dmaengine_get_dst_icg(xt, &xt->sgl[0]);
	src_icg = dmaengine_get_src_icg(xt, &xt->sgl[0]);

	if (chan->hw_2d) {
		if (!axi_dmac_check_len(chan, xt->sgl[0].size) ||
		    !axi_dmac_check_len(chan, xt->numf))
			return NULL;
		if (xt->sgl[0].size + dst_icg > chan->max_length ||
		    xt->sgl[0].size + src_icg > chan->max_length)
			return NULL;
	} else {
		if (dst_icg != 0 || src_icg != 0)
			return NULL;
		if (chan->max_length / xt->sgl[0].size < xt->numf)
			return NULL;
		if (!axi_dmac_check_len(chan, xt->sgl[0].size * xt->numf))
			return NULL;
	}

	desc = axi_dmac_alloc_desc(1);
	if (!desc)
		return NULL;

	if (axi_dmac_src_is_mem(chan)) {
		desc->sg[0].src_addr = xt->src_start;
		desc->sg[0].src_stride = xt->sgl[0].size + src_icg;
	}

	if (axi_dmac_dest_is_mem(chan)) {
		desc->sg[0].dest_addr = xt->dst_start;
		desc->sg[0].dest_stride = xt->sgl[0].size + dst_icg;
	}

	if (chan->hw_2d) {
		desc->sg[0].x_len = xt->sgl[0].size;
		desc->sg[0].y_len = xt->numf;
	} else {
		desc->sg[0].x_len = xt->sgl[0].size * xt->numf;
		desc->sg[0].y_len = 1;
	}

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static void axi_dmac_free_chan_resources(struct dma_chan *c)
{
	vchan_free_chan_resources(to_virt_chan(c));
}

static void axi_dmac_desc_free(struct virt_dma_desc *vdesc)
{
	kfree(container_of(vdesc, struct axi_dmac_desc, vdesc));
}

static bool axi_dmac_regmap_rdwr(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AXI_DMAC_REG_IRQ_MASK:
	case AXI_DMAC_REG_IRQ_SOURCE:
	case AXI_DMAC_REG_IRQ_PENDING:
	case AXI_DMAC_REG_CTRL:
	case AXI_DMAC_REG_TRANSFER_ID:
	case AXI_DMAC_REG_START_TRANSFER:
	case AXI_DMAC_REG_FLAGS:
	case AXI_DMAC_REG_DEST_ADDRESS:
	case AXI_DMAC_REG_SRC_ADDRESS:
	case AXI_DMAC_REG_X_LENGTH:
	case AXI_DMAC_REG_Y_LENGTH:
	case AXI_DMAC_REG_DEST_STRIDE:
	case AXI_DMAC_REG_SRC_STRIDE:
	case AXI_DMAC_REG_TRANSFER_DONE:
	case AXI_DMAC_REG_ACTIVE_TRANSFER_ID :
	case AXI_DMAC_REG_STATUS:
	case AXI_DMAC_REG_CURRENT_SRC_ADDR:
	case AXI_DMAC_REG_CURRENT_DEST_ADDR:
	case AXI_DMAC_REG_DBG0:
	case AXI_DMAC_REG_DBG1:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config axi_dmac_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = AXI_DMAC_REG_DBG1,
	.readable_reg = axi_dmac_regmap_rdwr,
	.writeable_reg = axi_dmac_regmap_rdwr,
};

/*
 * The configuration stored in the devicetree matches the configuration
 * parameters of the peripheral instance and allows the driver to know which
 * features are implemented and how it should behave.
 */
static int axi_dmac_parse_chan_dt(struct device_node *of_chan,
	struct axi_dmac_chan *chan)
{
	u32 val;
	int ret;

	ret = of_property_read_u32(of_chan, "reg", &val);
	if (ret)
		return ret;

	/* We only support 1 channel for now */
	if (val != 0)
		return -EINVAL;

	ret = of_property_read_u32(of_chan, "adi,source-bus-type", &val);
	if (ret)
		return ret;
	if (val > AXI_DMAC_BUS_TYPE_FIFO)
		return -EINVAL;
	chan->src_type = val;

	ret = of_property_read_u32(of_chan, "adi,destination-bus-type", &val);
	if (ret)
		return ret;
	if (val > AXI_DMAC_BUS_TYPE_FIFO)
		return -EINVAL;
	chan->dest_type = val;

	ret = of_property_read_u32(of_chan, "adi,source-bus-width", &val);
	if (ret)
		return ret;
	chan->src_width = val / 8;

	ret = of_property_read_u32(of_chan, "adi,destination-bus-width", &val);
	if (ret)
		return ret;
	chan->dest_width = val / 8;

	ret = of_property_read_u32(of_chan, "adi,length-width", &val);
	if (ret)
		return ret;

	if (val >= 32)
		chan->max_length = UINT_MAX;
	else
		chan->max_length = (1ULL << val) - 1;

	chan->align_mask = max(chan->dest_width, chan->src_width) - 1;

	if (axi_dmac_dest_is_mem(chan) && axi_dmac_src_is_mem(chan))
		chan->direction = DMA_MEM_TO_MEM;
	else if (!axi_dmac_dest_is_mem(chan) && axi_dmac_src_is_mem(chan))
		chan->direction = DMA_MEM_TO_DEV;
	else if (axi_dmac_dest_is_mem(chan) && !axi_dmac_src_is_mem(chan))
		chan->direction = DMA_DEV_TO_MEM;
	else
		chan->direction = DMA_DEV_TO_DEV;

	chan->hw_cyclic = of_property_read_bool(of_chan, "adi,cyclic");
	chan->hw_2d = of_property_read_bool(of_chan, "adi,2d");

	return 0;
}

/* Support old binding */
static int axi_dmac_parse_chan_dt_compat(struct device_node *of_node,
	struct axi_dmac_chan *chan)
{
	struct device_node *of_chan;
	u32 tmp;

	of_chan = of_get_child_by_name(of_node, "dma-channel");
	if (of_chan == NULL)
		return -ENODEV;

	tmp = 0;
	of_property_read_u32(of_chan, "adi,type", &tmp);

	switch (tmp) {
	case 0:
		chan->direction = DMA_DEV_TO_MEM;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		break;
	case 1:
		chan->direction = DMA_MEM_TO_DEV;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		break;
	case 2:
		chan->direction = DMA_MEM_TO_MEM;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_MM;
		break;
	case 3:
		chan->direction = DMA_DEV_TO_DEV;
		chan->src_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		chan->dest_type = AXI_DMAC_BUS_TYPE_AXI_STREAM;
		break;
	default:
		return -EINVAL;
	}

	tmp = 64;
	of_property_read_u32(of_chan, "adi,source-bus-width", &tmp);
	chan->src_width = tmp / 8;

	tmp = 64;
	of_property_read_u32(of_chan, "adi,destination-bus-width", &tmp);
	chan->dest_width = tmp / 8;

	tmp = 24;
	of_property_read_u32(of_chan, "adi,length-width", &tmp);

	if (tmp >= 32)
		chan->max_length = UINT_MAX;
	else
		chan->max_length = (1ULL << tmp) - 1;

	chan->align_mask = max(chan->dest_width, chan->src_width) - 1;

	chan->hw_cyclic = of_property_read_bool(of_chan, "adi,cyclic");
	chan->hw_2d = true;

	return 0;
}

static int axi_dmac_probe(struct platform_device *pdev)
{
	struct device_node *of_channels, *of_chan;
	struct dma_device *dma_dev;
	struct axi_dmac *dmac;
	struct resource *res;
	int ret;

	dmac = devm_kzalloc(&pdev->dev, sizeof(*dmac), GFP_KERNEL);
	if (!dmac)
		return -ENOMEM;

	dmac->irq = platform_get_irq(pdev, 0);
	if (dmac->irq <= 0)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmac->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmac->base))
		return PTR_ERR(dmac->base);

	dmac->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dmac->clk))
		return PTR_ERR(dmac->clk);

	INIT_LIST_HEAD(&dmac->chan.active_descs);

	of_channels = of_get_child_by_name(pdev->dev.of_node, "adi,channels");
	if (of_channels == NULL) {
		ret = axi_dmac_parse_chan_dt_compat(pdev->dev.of_node, &dmac->chan);
		if (ret)
			return ret;
	} else {
		for_each_child_of_node(of_channels, of_chan) {
			ret = axi_dmac_parse_chan_dt(of_chan, &dmac->chan);
			if (ret) {
				of_node_put(of_chan);
				of_node_put(of_channels);
				return -EINVAL;
			}
		}
		of_node_put(of_channels);
	}

	pdev->dev.dma_parms = &dmac->dma_parms;
	dma_set_max_seg_size(&pdev->dev, dmac->chan.max_length);

	dma_dev = &dmac->dma_dev;
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);
	dma_dev->device_free_chan_resources = axi_dmac_free_chan_resources;
	dma_dev->device_tx_status = dma_cookie_status;
	dma_dev->device_issue_pending = axi_dmac_issue_pending;
	dma_dev->device_prep_slave_sg = axi_dmac_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = axi_dmac_prep_dma_cyclic;
	dma_dev->device_prep_interleaved_dma = axi_dmac_prep_interleaved;
	dma_dev->device_terminate_all = axi_dmac_terminate_all;
	dma_dev->dev = &pdev->dev;
	dma_dev->chancnt = 1;
	dma_dev->src_addr_widths = BIT(dmac->chan.src_width);
	dma_dev->dst_addr_widths = BIT(dmac->chan.dest_width);
	dma_dev->directions = BIT(dmac->chan.direction);
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	INIT_LIST_HEAD(&dma_dev->channels);

	dmac->chan.vchan.desc_free = axi_dmac_desc_free;
	vchan_init(&dmac->chan.vchan, dma_dev);

	ret = clk_prepare_enable(dmac->clk);
	if (ret < 0)
		return ret;

	axi_dmac_write(dmac, AXI_DMAC_REG_IRQ_MASK, 0x00);

	ret = dma_async_device_register(dma_dev);
	if (ret)
		goto err_clk_disable;

	ret = of_dma_controller_register(pdev->dev.of_node,
		of_dma_xlate_by_chan_id, dma_dev);
	if (ret)
		goto err_unregister_device;

	ret = request_irq(dmac->irq, axi_dmac_interrupt_handler, 0,
		dev_name(&pdev->dev), dmac);
	if (ret)
		goto err_unregister_of;

	platform_set_drvdata(pdev, dmac);

	devm_regmap_init_mmio(&pdev->dev, dmac->base, &axi_dmac_regmap_config);

#ifdef SPEED_TEST
	for (i = 0; i < 0x30; i += 4)
		printk("reg %x: %x\n", i, axi_dmac_read(dmac, i));
	dmac->test_virt = dma_alloc_coherent(&pdev->dev, SZ_8M,
			&dmac->test_phys, GFP_KERNEL);

	axi_dmac_write(dmac, AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);
	axi_dmac_write(dmac, AXI_DMAC_REG_DMA_ADDRESS, dmac->test_phys);
	axi_dmac_write(dmac, AXI_DMAC_REG_DMA_COUNT, SZ_8M);

	printk("Check registers\n");
	printk("CTRL: %x %x\n", AXI_DMAC_CTRL_ENABLE, axi_dmac_read(dmac, AXI_DMAC_REG_CTRL)); 
	printk("ADDR: %x %x\n", dmac->test_phys, axi_dmac_read(dmac, AXI_DMAC_REG_DMA_ADDRESS));
	printk("COUNT: %x %x\n", PAGE_SIZE, axi_dmac_read(dmac, AXI_DMAC_REG_DMA_COUNT));
	printk("MASK: %x %x\n", 0, axi_dmac_read(dmac, AXI_DMAC_REG_IRQ_MASK));

	printk("Start transfer\n");
	axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);
	printk("START: %x %x\n", 1, axi_dmac_read(dmac, AXI_DMAC_REG_START_TRANSFER));

	for (i = 0; i < 0x100; i++)
		printk("%.8x%c", ((unsigned long *)dmac->test_virt)[i],
			i % 16 == 15 ? '\n' : ' ');
	printk("Last: %x\n", ((unsigned long *)dmac->test_virt)[PAGE_SIZE/4-1]);
	printk("PROGRESS: %x %x\n", 1, axi_dmac_read(dmac, AXI_DMAC_REG_DMA_COUNT_PROGRESS));
#endif

	return 0;

err_unregister_of:
	of_dma_controller_free(pdev->dev.of_node);
err_unregister_device:
	dma_async_device_unregister(&dmac->dma_dev);
err_clk_disable:
	clk_disable_unprepare(dmac->clk);

	return ret;
}

static int axi_dmac_remove(struct platform_device *pdev)
{
	struct axi_dmac *dmac = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	free_irq(dmac->irq, dmac);
	tasklet_kill(&dmac->chan.vchan.task);
	dma_async_device_unregister(&dmac->dma_dev);
	clk_disable_unprepare(dmac->clk);

	return 0;
}

static const struct of_device_id axi_dmac_of_match_table[] = {
	{ .compatible = "adi,axi-dmac-1.00.a" },
	{ },
};

static struct platform_driver axi_dmac_driver = {
	.driver = {
		.name = "dma-axi-dmac",
		.of_match_table = axi_dmac_of_match_table,
	},
	.probe = axi_dmac_probe,
	.remove = axi_dmac_remove,
};
module_platform_driver(axi_dmac_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("DMA controller driver for the AXI-DMAC controller");
MODULE_LICENSE("GPL v2");
