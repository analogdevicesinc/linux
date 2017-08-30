/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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

#include "dpu-blit.h"
#include "dpu-blit-registers.h"
#include "dpu-prv.h"

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
		usleep_range(1000, 2000);
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
	if (dpu_be->inuse) {
		mutex_unlock(&dpu_be->mutex);
		return -EBUSY;
	}

	dpu_be->inuse = true;
	mutex_unlock(&dpu_be->mutex);

	return 0;
}
EXPORT_SYMBOL(dpu_be_get);

void dpu_be_put(struct dpu_bliteng *dpu_be)
{
	mutex_lock(&dpu_be->mutex);

	dpu_be->inuse = false;

	mutex_unlock(&dpu_be->mutex);
}
EXPORT_SYMBOL(dpu_be_put);

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
	dpu_be_write(dpu_be, 0x10, PIXENGCFG_STORE9_TRIGGER);

	while ((dpu_be_read(dpu_be, COMCTRL_INTERRUPTSTATUS0) &
		STORE9_SEQCOMPLETE_IRQ_MASK) == 0)
		usleep_range(1000, 2000);

	dpu_be_write(dpu_be, STORE9_SEQCOMPLETE_IRQ_MASK,
		COMCTRL_INTERRUPTCLEAR0);
}
EXPORT_SYMBOL(dpu_be_wait);

static void dpu_be_init_units(struct dpu_bliteng *dpu_be)
{
	u32 staticcontrol;
	u32 pixengcfg_unit_static, pixengcfg_unit_dynamic;

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

	/* Safety_Pixengcfg Static */
	pixengcfg_unit_static =
	1 << PIXENGCFG_STORE9_STATIC_STORE9_SHDEN_SHIFT |
	0 << PIXENGCFG_STORE9_STATIC_STORE9_POWERDOWN_SHIFT |
	PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE__SINGLE <<
	PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE_SHIFT |
	PIXENGCFG_STORE9_STATIC_STORE9_SW_RESET__OPERATION <<
	PIXENGCFG_STORE9_STATIC_STORE9_SW_RESET_SHIFT |
	PIXENGCFG_DIVIDER_RESET <<
	PIXENGCFG_STORE9_STATIC_STORE9_DIV_SHIFT;
	dpu_be_write(dpu_be, pixengcfg_unit_static, PIXENGCFG_STORE9_STATIC);

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

	return 0;
}
EXPORT_SYMBOL_GPL(dpu_bliteng_init);

void dpu_bliteng_fini(struct dpu_bliteng *dpu_bliteng)
{
	kfree(dpu_bliteng->cmd_list);
}
EXPORT_SYMBOL_GPL(dpu_bliteng_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX DPU BLITENG");
MODULE_ALIAS("platform:imx-dpu-bliteng");
