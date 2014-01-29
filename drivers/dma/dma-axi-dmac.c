/*
 * Driver for the AXI-HDMI-RX core
 *
 * Copyright 2013 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_dma.h>
#include <linux/regmap.h>

#include "dmaengine.h"
#include "virt-dma.h"

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
};

struct axi_dmac {
	void __iomem *base;
	int irq;

	struct clk *clk;

	struct dma_device dma_dev;
	struct axi_dmac_chan chan;

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
	switch (chan->direction) {
	case DMA_MEM_TO_DEV:
	case DMA_MEM_TO_MEM:
		return true;
	default:
		return false;
	}
}

static int axi_dmac_dest_is_mem(struct axi_dmac_chan *chan)
{
	switch (chan->direction) {
	case DMA_DEV_TO_MEM:
	case DMA_MEM_TO_MEM:
		return true;
	default:
		return false;
	}
}

static int axi_dmac_start_transfer(struct axi_dmac_chan *chan)
{
	struct axi_dmac *dmac = chan_to_axi_dmac(chan);
	struct virt_dma_desc *vdesc;
	struct axi_dmac_sg *sg;
	unsigned int val;

	val = axi_dmac_read(dmac, AXI_DMAC_REG_START_TRANSFER);
	if (val)
		return 0;

	if (!chan->next_desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return 0;
		list_move_tail(&vdesc->node, &chan->active_descs);
		chan->next_desc = to_axi_dmac_desc(vdesc);
	}

	sg = &chan->next_desc->sg[chan->next_desc->num_submitted];

	chan->next_desc->num_submitted++;
	if (chan->next_desc->num_submitted == chan->next_desc->num_sgs)
		chan->next_desc = NULL;

	sg->id = axi_dmac_read(dmac, AXI_DMAC_REG_TRANSFER_ID);

	if (axi_dmac_dest_is_mem(chan)) {
		axi_dmac_write(dmac, AXI_DMAC_REG_DEST_ADDRESS, sg->dest_addr);
		axi_dmac_write(dmac, AXI_DMAC_REG_DEST_STRIDE, sg->dest_stride);
	}

	if (axi_dmac_src_is_mem(chan)) {
		axi_dmac_write(dmac, AXI_DMAC_REG_SRC_ADDRESS, sg->src_addr);
		axi_dmac_write(dmac, AXI_DMAC_REG_SRC_STRIDE, sg->src_stride);
	}

	axi_dmac_write(dmac, AXI_DMAC_REG_X_LENGTH, sg->x_len - 1);
	axi_dmac_write(dmac, AXI_DMAC_REG_Y_LENGTH, sg->y_len - 1);
	axi_dmac_write(dmac, AXI_DMAC_REG_START_TRANSFER, 1);

	return 0;
}

static struct axi_dmac_desc *axi_dmac_active_desc(struct axi_dmac_chan *chan)
{
	return list_first_entry_or_null(&chan->active_descs, struct axi_dmac_desc,
			vdesc.node);
}

static void axi_dmac_transfer_done(struct axi_dmac_chan *chan,
	unsigned int completed_transfers)
{
	struct axi_dmac_desc *active_desc;
	struct axi_dmac_sg *sg;

	active_desc = axi_dmac_active_desc(chan);
	if (!active_desc)
		return;

	if (active_desc->cyclic) {
		vchan_cyclic_callback(&active_desc->vdesc);
	} else {
		while (active_desc &&
			active_desc->num_completed < active_desc->num_submitted) {
			sg = &active_desc->sg[active_desc->num_completed];
			if (!(BIT(sg->id) & completed_transfers))
				break;
			active_desc->num_completed++;
			if (active_desc->num_completed == active_desc->num_sgs) {
				list_del(&active_desc->vdesc.node);
				vchan_cookie_complete(&active_desc->vdesc);
				active_desc = axi_dmac_active_desc(chan);
			}
		}
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
	if (pending & AXI_DMAC_IRQ_EOT) {
		unsigned int completed_transfers;
		completed_transfers = axi_dmac_read(dmac, AXI_DMAC_REG_TRANSFER_DONE);
		axi_dmac_transfer_done(&dmac->chan, completed_transfers);
	}
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

static int axi_dmac_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
	unsigned long arg)
{
	switch (cmd) {
	case DMA_TERMINATE_ALL:
		return axi_dmac_terminate_all(chan);
	default:
		return -ENOSYS;
	}
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
		sizeof(struct axi_dmac_sg) * num_sgs, GFP_ATOMIC);
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
	unsigned long flags, void *context)
{
	struct axi_dmac_chan *chan = to_axi_dmac_chan(c);
	struct axi_dmac_desc *desc;
	unsigned int num_periods, i;

	if (direction != chan->direction)
		return NULL;

	if (buf_len % period_len)
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

	if (xt->frame_size != 1 || xt->numf == 0)
		return NULL;

	if (xt->sgl[0].size == 0)
		return NULL;

	if (xt->dir != chan->direction)
		return NULL;

	if (axi_dmac_src_is_mem(chan)) {
		if (!xt->src_inc)
			return NULL;
	}

	if (axi_dmac_dest_is_mem(chan)) {
		if (!xt->dst_inc)
			return NULL;
	}

	desc = axi_dmac_alloc_desc(1);
	if (!desc)
		return NULL;

	if (axi_dmac_src_is_mem(chan)) {
		desc->sg[0].src_addr = xt->src_start;
		desc->sg[0].src_stride = xt->sgl[0].size;
		if (xt->src_sgl)
			desc->sg[0].src_stride += xt->sgl[0].icg;
	}

	if (axi_dmac_dest_is_mem(chan)) {
		desc->sg[0].dest_addr = xt->dst_start;
		desc->sg[0].dest_stride = xt->sgl[0].size;
		if (xt->dst_sgl)
			desc->sg[0].dest_stride += xt->sgl[0].icg;
	}

	desc->sg[0].x_len = xt->sgl[0].size;
	desc->sg[0].y_len = xt->numf;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static int axi_dmac_alloc_chan_resources(struct dma_chan *c)
{
	return 0;
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

static int axi_dmac_probe(struct platform_device *pdev)
{
	struct device_node *of_chan;
	struct dma_device *dma_dev;
	struct axi_dmac *dmac;
	struct resource *res;
	u32 chan_type;
	int ret;
	int i;

	dmac = devm_kzalloc(&pdev->dev, sizeof(*dmac), GFP_KERNEL);
	if (!dmac)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmac->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmac->base))
		return PTR_ERR(dmac->base);

	dmac->irq = platform_get_irq(pdev, 0);
	if (dmac->irq <= 0)
		return -EINVAL;

	dmac->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dmac->clk))
		return PTR_ERR(dmac->clk);

	clk_prepare_enable(dmac->clk);

	INIT_LIST_HEAD(&dmac->chan.active_descs);

	of_chan = of_get_child_by_name(pdev->dev.of_node, "dma-channel");
	if (of_chan == NULL)
		return -ENODEV;

	chan_type = 0;
	of_property_read_u32(of_chan, "adi,type", &chan_type);

	switch (chan_type) {
	case 0:
		dmac->chan.direction = DMA_DEV_TO_MEM;
		break;
	case 1:
		dmac->chan.direction = DMA_MEM_TO_DEV;
		break;
	case 2:
		dmac->chan.direction = DMA_MEM_TO_MEM;
		break;
	case 3:
		dmac->chan.direction = DMA_DEV_TO_DEV;
		break;
	default:
		return -EINVAL;
	}

	dma_dev = &dmac->dma_dev;
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);
	dma_dev->device_alloc_chan_resources = axi_dmac_alloc_chan_resources;
	dma_dev->device_free_chan_resources = axi_dmac_free_chan_resources;
	dma_dev->device_tx_status = dma_cookie_status;
	dma_dev->device_issue_pending = axi_dmac_issue_pending;
	dma_dev->device_prep_slave_sg = axi_dmac_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = axi_dmac_prep_dma_cyclic;
	dma_dev->device_prep_interleaved_dma = axi_dmac_prep_interleaved;
	dma_dev->device_control = axi_dmac_control;
	dma_dev->dev = &pdev->dev;
	dma_dev->chancnt = 1;
	INIT_LIST_HEAD(&dma_dev->channels);

	dmac->chan.vchan.desc_free = axi_dmac_desc_free;
	vchan_init(&dmac->chan.vchan, dma_dev);

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

	axi_dmac_write(dmac, AXI_DMAC_REG_IRQ_MASK, 0x00);

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

	free_irq(dmac->irq, dmac);
	of_dma_controller_free(pdev->dev.of_node);
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
		.owner = THIS_MODULE,
		.name = "dma-axi-dmac",
		.of_match_table = axi_dmac_of_match_table,
	},
	.probe = axi_dmac_probe,
	.remove = axi_dmac_remove,
};
module_platform_driver(axi_dmac_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("DMA controller driver for the AXI-DMAC controller");
MODULE_LICENSE("GPL");
