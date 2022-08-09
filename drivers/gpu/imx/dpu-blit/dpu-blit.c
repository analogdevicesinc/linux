/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019,2022 NXP
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

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include <video/imx8-prefetch.h>

#include "dpu-blit.h"
#include "dpu-blit-registers.h"
#include "dpu-prv.h"

void dpu_be_wait(struct dpu_bliteng *dpu_be);

static inline u32 dpu_be_read(struct dpu_bliteng *dpu_be, unsigned int offset)
{
	return readl(dpu_be->base + offset);
}

static inline void dpu_be_write(struct dpu_bliteng *dpu_be, u32 value,
	unsigned int offset)
{
	writel(value, dpu_be->base + offset);
}

static void dpu_cs_wait_fifo_space(struct dpu_bliteng *dpu_be)
{
	while ((dpu_be_read(dpu_be, CMDSEQ_STATUS) &
		CMDSEQ_STATUS_FIFOSPACE_MASK) < CMDSEQ_FIFO_SPACE_THRESHOLD)
		usleep_range(30, 50);
}

static void dpu_cs_wait_idle(struct dpu_bliteng *dpu_be)
{
	while ((dpu_be_read(dpu_be, CMDSEQ_STATUS) &
		CMDSEQ_STATUS_IDLE_MASK) == 0x0)
		mdelay(1);
}

static int dpu_cs_alloc_command_buffer(struct dpu_bliteng *dpu_be)
{
	/* command buffer need 32 bit address */
	dpu_be->buffer_addr_virt =
		alloc_pages_exact(COMMAND_BUFFER_SIZE,
			GFP_KERNEL | GFP_DMA | GFP_DMA32 | __GFP_ZERO);
	if (!dpu_be->buffer_addr_virt) {
		dev_err(dpu_be->dev, "memory alloc failed for dpu command buffer\n");
		return -ENOMEM;
	}

	dpu_be->buffer_addr_phy =
		(u32)virt_to_phys(dpu_be->buffer_addr_virt);

	return 0;
}

static void dpu_cs_static_setup(struct dpu_bliteng *dpu_be)
{
	dpu_cs_wait_idle(dpu_be);

	/* LockUnlock and LockUnlockHIF */
	dpu_be_write(dpu_be, CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__UNLOCK_KEY,
		CMDSEQ_LOCKUNLOCKHIF);
	dpu_be_write(dpu_be, CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__UNLOCK_KEY,
		CMDSEQ_LOCKUNLOCK);

	/* Control */
	dpu_be_write(dpu_be, 1 << CMDSEQ_CONTROL_CLEAR_SHIFT,
		CMDSEQ_CONTROL);

	/* BufferAddress and BufferSize */
	dpu_be_write(dpu_be, dpu_be->buffer_addr_phy, CMDSEQ_BUFFERADDRESS);
	dpu_be_write(dpu_be, COMMAND_BUFFER_SIZE / WORD_SIZE,
		CMDSEQ_BUFFERSIZE);
}

static struct dprc *
dpu_be_dprc_get(struct dpu_soc *dpu, int dprc_id)
{
	struct dprc *dprc;

	dprc = dprc_lookup_by_phandle(dpu->dev,
				      "fsl,dpr-channels",
				      dprc_id);

	return dprc;
}

void dpu_be_configure_prefetch(struct dpu_bliteng *dpu_be,
			       u32 width, u32 height,
			       u32 x_offset, u32 y_offset,
			       u32 stride, u32 format, u64 modifier,
			       u64 baddr, u64 uv_addr)
{
	struct dprc *dprc;
	bool dprc_en=false;

	/* Enable DPR, dprc1 is connected to plane0 */
	dprc = dpu_be->dprc[1];

	/*
	 * Force sync command sequncer in conditions:
	 * 1. tile work with dprc/prg (baddr)
	 * 2. switch tile to linear (!start)
	 */
	if (!dpu_be->start || baddr) {
		dpu_be_wait(dpu_be);
	}

	dpu_be->sync = true;

	if (baddr == 0x0) {
		if (!dpu_be->start) {
			dprc_disable(dprc);
			dpu_be->start = true;
		}
		return;
	}

	if (dpu_be->modifier != modifier && !dpu_be->start) {
		dprc_disable(dprc);
		dprc_en = true;
	}

	dpu_be->modifier = modifier;

	dprc_configure(dprc, 0,
		       width, height,
		       x_offset, y_offset,
		       stride, format, modifier,
		       baddr, uv_addr,
		       dpu_be->start,
		       dpu_be->start,
		       false);

	if (dpu_be->start || dprc_en) {
		dprc_enable(dprc);
	}

	dprc_reg_update(dprc);

	dpu_be->start = false;
}
EXPORT_SYMBOL(dpu_be_configure_prefetch);

int dpu_bliteng_get_empty_instance(struct dpu_bliteng **dpu_be,
	struct device *dev)
{
	if (!dpu_be || !dev)
		return -EINVAL;

	*dpu_be = devm_kzalloc(dev, sizeof(struct dpu_bliteng), GFP_KERNEL);
	if (!(*dpu_be))
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL(dpu_bliteng_get_empty_instance);

u32 *dpu_bliteng_get_cmd_list(struct dpu_bliteng *dpu_be)
{
	return dpu_be->cmd_list;
}
EXPORT_SYMBOL(dpu_bliteng_get_cmd_list);

s32 dpu_bliteng_get_id(struct dpu_bliteng *dpu_be)
{
	return dpu_be->id;
}
EXPORT_SYMBOL(dpu_bliteng_get_id);

void dpu_bliteng_set_id(struct dpu_bliteng *dpu_be, int id)
{
	dpu_be->id = id;
}
EXPORT_SYMBOL(dpu_bliteng_set_id);

void dpu_bliteng_set_dev(struct dpu_bliteng *dpu_be, struct device *dev)
{
	dpu_be->dev = dev;
}
EXPORT_SYMBOL(dpu_bliteng_set_dev);

int dpu_be_get(struct dpu_bliteng *dpu_be)
{
	mutex_lock(&dpu_be->mutex);

	return 0;
}
EXPORT_SYMBOL(dpu_be_get);

void dpu_be_put(struct dpu_bliteng *dpu_be)
{
	mutex_unlock(&dpu_be->mutex);
}
EXPORT_SYMBOL(dpu_be_put);

static irqreturn_t dpu_bliteng_irq_handler(int irq, void *dev_id)
{
	int i = 0;
	struct dpu_bliteng *dpu_be = dev_id;
	struct dpu_be_fence *fence = NULL;

	for (i = 0; i < 4; i++) {
		if (irq == dpu_be->irq_comctrl_sw[i])
			break;
	}

	if (i < 4) {
		/* Found and release the fence */
		fence = dpu_be->fence[i];
		dpu_be->fence[i] = NULL;
		up(&dpu_be->sema[i]);
	}

	if (fence) {
		/* Signal the fence when all triggered */
		if (atomic_dec_and_test(&fence->refcnt)) {
			dma_fence_signal(&fence->base);
			fence->signaled = true;
		}

		dma_fence_put(&fence->base);
	}

	return IRQ_HANDLED;
}

static const char *dpu_be_fence_get_driver_name(struct dma_fence *fence)
{
	return "imx-dpu-bliteng";
}

static const char *dpu_be_fence_get_timeline_name(struct dma_fence *fence)
{
	return "dpu-bliteng-fence";
}

static bool dpu_be_fence_signaled(struct dma_fence *fence)
{
	struct dpu_be_fence *bfence = (struct dpu_be_fence *)fence;

	return bfence->signaled;
}

static bool dpu_be_fence_enable_signaling(struct dma_fence *fence)
{
	return !dpu_be_fence_signaled(fence);
}

static struct dma_fence_ops dpu_be_fence_ops = {
	.get_driver_name = dpu_be_fence_get_driver_name,
	.get_timeline_name = dpu_be_fence_get_timeline_name,
	.enable_signaling = dpu_be_fence_enable_signaling,
	.signaled = dpu_be_fence_signaled,
	.release = dma_fence_free,
};

int dpu_be_get_fence(struct dpu_bliteng *dpu_be)
{
	int fd = -1;
	u64 seqno = 0;
	struct dpu_be_fence *fence = NULL;
	struct sync_file *sync = NULL;

	/* Allocate a fence pointer */
	fence = kzalloc(sizeof(struct dpu_be_fence), GFP_KERNEL);
	if (!fence)
		return -1;

	/* Init fence pointer */
	fence->signaled = false;
	spin_lock_init(&fence->lock);
	atomic_set(&fence->refcnt, 0);

	/* Init dma fence base data */
	seqno = atomic64_inc_return(&dpu_be->seqno);
	dma_fence_init(&fence->base, &dpu_be_fence_ops, &fence->lock, dpu_be->context, seqno);

	/* Create sync file pointer */
	sync = sync_file_create(&fence->base);
	if (!sync) {
		dev_err(dpu_be->dev, "failed to create fence sync file.\n");
		goto failed;
	}

	/* Get the unused file descriptor */
	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0) {
		dev_err(dpu_be->dev, "failed to get unused file descriptor.\n");
		goto failed;
	}

	/* Setup fd to fence sync */
	fd_install(fd, sync->file);

	return fd;

failed:
	if (sync)
		fput(sync->file);

	kfree(fence);

	return -1;
}
EXPORT_SYMBOL(dpu_be_get_fence);

static int dpu_be_emit_fence(struct dpu_bliteng *dpu_be, struct dpu_be_fence *fence, bool stall)
{
	int i = 0;

	/* Get the available fence index with spin-lock */
	spin_lock(&dpu_be->lock);
	i = dpu_be->next_fence_idx++;
	dpu_be->next_fence_idx &= 0x3;
	spin_unlock(&dpu_be->lock);

	/*Acquire fence semaphore released by irq handler */
	down(&dpu_be->sema[i]);

	WARN_ON(dpu_be->fence[i]);
	dpu_be->fence[i] = fence;

	dpu_cs_wait_fifo_space(dpu_be);

	/* Write comctrl interrupt PRESET to command sequencer */
	dpu_be_write(dpu_be, 0x14000001, CMDSEQ_HIF);
	dpu_be_write(dpu_be, COMCTRL_USERINTERRUPTPRESET1, CMDSEQ_HIF);
	dpu_be_write(dpu_be, 1 << (i + (IRQ_COMCTRL_SW0 & 0x1F)), CMDSEQ_HIF);

	/*Stall until semaphore released */
	if (stall) {
		down(&dpu_be->sema[i]);
		up(&dpu_be->sema[i]);
	}

	return 0;
}

int dpu_be_set_fence(struct dpu_bliteng *dpu_be, int fd)
{
	struct dpu_be_fence *fence = NULL;

	/* Retrieve fence pointer from sync file descriptor */
	fence = (struct dpu_be_fence *)sync_file_get_fence(fd);
	if (!fence) {
		dev_err(dpu_be->dev, "failed to get fence pointer.\n");
		return -1;
	}

	/* Setup the fence and active it asynchronously */
	dpu_be_emit_fence(dpu_be, fence, false);

	/* Increase fence and base reference */
	atomic_inc(&fence->refcnt);
	dma_fence_get(&fence->base);

	return 0;
}
EXPORT_SYMBOL(dpu_be_set_fence);

int dpu_be_blit(struct dpu_bliteng *dpu_be,
	u32 *cmdlist, u32 cmdnum)
{
	int i;

	if (cmdnum > CMDSEQ_FIFO_SPACE_THRESHOLD) {
		dev_err(dpu_be->dev, "dpu blit cmdnum[%d] should be less than %d !\n",
			cmdnum, CMDSEQ_FIFO_SPACE_THRESHOLD);
		return -EINVAL;
	}
	dpu_cs_wait_fifo_space(dpu_be);

	for (i = 0; i < cmdnum; i++)
		dpu_be_write(dpu_be, cmdlist[i], CMDSEQ_HIF);

	return 0;
}
EXPORT_SYMBOL(dpu_be_blit);

#define STORE9_SEQCOMPLETE_IRQ		2U
#define STORE9_SEQCOMPLETE_IRQ_MASK	(1U<<STORE9_SEQCOMPLETE_IRQ)
void dpu_be_wait(struct dpu_bliteng *dpu_be)
{
	if (!dpu_be->sync) return;

	dpu_cs_wait_fifo_space(dpu_be);

	dpu_be_write(dpu_be, 0x14000001, CMDSEQ_HIF);
	dpu_be_write(dpu_be, PIXENGCFG_STORE9_TRIGGER, CMDSEQ_HIF);
	dpu_be_write(dpu_be, 0x10, CMDSEQ_HIF);

	while ((dpu_be_read(dpu_be, COMCTRL_INTERRUPTSTATUS0) &
		STORE9_SEQCOMPLETE_IRQ_MASK) == 0)
		usleep_range(30, 50);

	dpu_be_write(dpu_be, STORE9_SEQCOMPLETE_IRQ_MASK,
		COMCTRL_INTERRUPTCLEAR0);

	dpu_be->sync = false;
}
EXPORT_SYMBOL(dpu_be_wait);

static void dpu_be_init_units(struct dpu_bliteng *dpu_be)
{
	u32 staticcontrol;
	u32 pixengcfg_unit_dynamic;

	staticcontrol =
	1 << FETCHDECODE9_STATICCONTROL_SHDEN_SHIFT |
	0 << FETCHDECODE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT |
	FETCHDECODE9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, FETCHDECODE9_STATICCONTROL);

	staticcontrol =
	1 << FETCHWARP9_STATICCONTROL_SHDEN_SHIFT |
	0 << FETCHWARP9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT |
	FETCHWARP9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, FETCHWARP9_STATICCONTROL);

	staticcontrol =
	1 << FETCHECO9_STATICCONTROL_SHDEN_SHIFT |
	0 << FETCHECO9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT |
	FETCHECO9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, FETCHECO9_STATICCONTROL);

	staticcontrol =
	1 << HSCALER9_STATICCONTROL_SHDEN_SHIFT |
	HSCALER9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, HSCALER9_STATICCONTROL);

	staticcontrol =
	1 << VSCALER9_STATICCONTROL_SHDEN_SHIFT |
	VSCALER9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, VSCALER9_STATICCONTROL);

	staticcontrol =
	1 << ROP9_STATICCONTROL_SHDEN_SHIFT |
	ROP9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, ROP9_STATICCONTROL);

	staticcontrol =
	1 << MATRIX9_STATICCONTROL_SHDEN_SHIFT |
	MATRIX9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, MATRIX9_STATICCONTROL);

	staticcontrol =
	1 << BLITBLEND9_STATICCONTROL_SHDEN_SHIFT |
	BLITBLEND9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, BLITBLEND9_STATICCONTROL);

	staticcontrol =
	1 << STORE9_STATICCONTROL_SHDEN_SHIFT |
	0 << STORE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT |
	STORE9_STATICCONTROL_RESET_VALUE;
	dpu_be_write(dpu_be, staticcontrol, STORE9_STATICCONTROL);

	/* Safety_Pixengcfg Dynamic */
	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_FETCHDECODE9_DYNAMIC_RESET_VALUE;
	dpu_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_FETCHDECODE9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_FETCHWARP9_DYNAMIC_RESET_VALUE;
	dpu_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_FETCHWARP9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_ROP9_DYNAMIC_RESET_VALUE;
	dpu_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_ROP9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_MATRIX9_DYNAMIC_RESET_VALUE;
	dpu_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_MATRIX9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_HSCALER9_DYNAMIC_RESET_VALUE;
	dpu_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_HSCALER9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_VSCALER9_DYNAMIC_RESET_VALUE;
	dpu_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_VSCALER9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_BLITBLEND9_DYNAMIC_RESET_VALUE;
	dpu_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_BLITBLEND9_DYNAMIC);
}

int dpu_bliteng_init(struct dpu_bliteng *dpu_bliteng)
{
	struct dpu_soc *dpu = dev_get_drvdata(dpu_bliteng->dev->parent);
	struct platform_device *dpu_pdev =
		container_of(dpu->dev, struct platform_device, dev);
	struct resource *res;
	unsigned long dpu_base;
	void __iomem *base;
	u32 *cmd_list;
	int ret;
	int i, virq, sw_irqs[4] = {IRQ_COMCTRL_SW0, IRQ_COMCTRL_SW1,
					IRQ_COMCTRL_SW2, IRQ_COMCTRL_SW3};

	cmd_list = kzalloc(sizeof(*cmd_list) * CMDSEQ_FIFO_SPACE_THRESHOLD,
			GFP_KERNEL);
	if (!cmd_list)
		return -ENOMEM;
	dpu_bliteng->cmd_list = cmd_list;

	res = platform_get_resource(dpu_pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	dpu_base = res->start;

	/* remap with bigger size */
	base = devm_ioremap(dpu->dev, dpu_base, 64*SZ_1K);
	dpu_bliteng->base = base;
	dpu_bliteng->dpu = dpu;

	mutex_init(&dpu_bliteng->mutex);

	/* Init the uints used by blit engine */
	/* Maybe this should be in dpu-common.c */
	dpu_be_init_units(dpu_bliteng);

	/* Init for command sequencer */
	ret = dpu_cs_alloc_command_buffer(dpu_bliteng);
	if (ret)
		return ret;

	dpu_cs_static_setup(dpu_bliteng);

	/*Request general SW interrupts to implement DPU fence */
	for (i = 0; i < 4; i++) {
		virq = dpu_map_irq(dpu, sw_irqs[i]);
		dpu_bliteng->irq_comctrl_sw[i] = virq;

		irq_set_status_flags(virq, IRQ_DISABLE_UNLAZY);
		ret = devm_request_irq(dpu->dev, virq, dpu_bliteng_irq_handler, 0,
					"imx-dpu-bliteng", dpu_bliteng);
		if (ret < 0) {
			dev_err(dpu->dev, "irq_comctrl_sw%d irq request failed with %d.\n", i, ret);
			return ret;
		}

		sema_init(&dpu_bliteng->sema[i], 1);
	}

	/* Init fence context and seqno */
	dpu_bliteng->context = dma_fence_context_alloc(1);
	atomic64_set(&dpu_bliteng->seqno, 0);
	spin_lock_init(&dpu_bliteng->lock);

	/* DPR, each blit engine has two dprc, 0 & 1 */
	dpu_bliteng->dprc[0] = dpu_be_dprc_get(dpu, 0);
	dpu_bliteng->dprc[1] = dpu_be_dprc_get(dpu, 1);

	dprc_disable(dpu_bliteng->dprc[0]);
	dprc_disable(dpu_bliteng->dprc[1]);

	dpu_bliteng->start = true;
	dpu_bliteng->sync = false;

	dpu_bliteng->modifier = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(dpu_bliteng_init);

void dpu_bliteng_fini(struct dpu_bliteng *dpu_bliteng)
{
	int i;

	/* LockUnlock and LockUnlockHIF */
	dpu_be_write(dpu_bliteng, CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__LOCK_KEY,
		CMDSEQ_LOCKUNLOCKHIF);
	dpu_be_write(dpu_bliteng, CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__LOCK_KEY,
		CMDSEQ_LOCKUNLOCK);

	kfree(dpu_bliteng->cmd_list);

	for (i = 0; i < 4; i++)
		free_irq(dpu_bliteng->irq_comctrl_sw[i], dpu_bliteng);

	if (dpu_bliteng->buffer_addr_virt)
		free_pages_exact(dpu_bliteng->buffer_addr_virt,
				 COMMAND_BUFFER_SIZE);
}
EXPORT_SYMBOL_GPL(dpu_bliteng_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX DPU BLITENG");
MODULE_ALIAS("platform:imx-dpu-bliteng");
