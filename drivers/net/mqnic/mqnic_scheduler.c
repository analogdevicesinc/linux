// SPDX-License-Identifier: BSD-2-Clause-Views
/*
 * Copyright (c) 2021-2023 The Regents of the University of California
 */

#include "mqnic.h"

static void _mqnic_scheduler_enable(struct mqnic_sched *sched)
{
	iowrite32(1, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CTRL);
}

static void _mqnic_scheduler_disable(struct mqnic_sched *sched)
{
	iowrite32(0, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CTRL);
}

struct mqnic_sched *mqnic_create_scheduler(struct mqnic_sched_block *block,
		int index, struct mqnic_reg_block *rb)
{
	struct device *dev = block->dev;
	struct mqnic_sched *sched;
	u32 val;
	int k;

	sched = kzalloc(sizeof(*sched), GFP_KERNEL);
	if (!sched)
		return ERR_PTR(-ENOMEM);

	sched->dev = dev;
	sched->interface = block->interface;
	sched->sched_block = block;

	sched->index = index;

	sched->rb = rb;

	sched->type = rb->type;
	sched->offset = ioread32(rb->regs + MQNIC_RB_SCHED_RR_REG_OFFSET);
	sched->queue_count = ioread32(rb->regs + MQNIC_RB_SCHED_RR_REG_QUEUE_COUNT);
	sched->queue_stride = ioread32(rb->regs + MQNIC_RB_SCHED_RR_REG_QUEUE_STRIDE);

	sched->hw_addr = block->interface->hw_addr + sched->offset;

	val = ioread32(rb->regs + MQNIC_RB_SCHED_RR_REG_CFG);
	sched->tc_count = val & 0xff;
	sched->port_count = (val >> 8) & 0xff;
	sched->channel_count = sched->tc_count * sched->port_count;
	sched->fc_scale = 1 << ((val >> 16) & 0xff);

	sched->enable_count = 0;
	_mqnic_scheduler_disable(sched);

	dev_info(dev, "Scheduler type: 0x%08x", sched->type);
	dev_info(dev, "Scheduler offset: 0x%08x", sched->offset);
	dev_info(dev, "Scheduler queue count: %d", sched->queue_count);
	dev_info(dev, "Scheduler queue stride: %d", sched->queue_stride);
	dev_info(dev, "Scheduler TC count: %d", sched->tc_count);
	dev_info(dev, "Scheduler port count: %d", sched->port_count);
	dev_info(dev, "Scheduler channel count: %d", sched->channel_count);
	dev_info(dev, "Scheduler FC scale: %d", sched->fc_scale);

	INIT_LIST_HEAD(&sched->sched_port_list);

	for (k = 0; k < sched->port_count; k++) {
		struct mqnic_sched_port *port = mqnic_create_sched_port(sched, k);

		list_add_tail(&port->list, &sched->sched_port_list);
		mqnic_interface_register_sched_port(sched->interface, port);
	}

	return sched;
}

void mqnic_destroy_scheduler(struct mqnic_sched *sched)
{
	struct mqnic_sched_port *port, *port_safe;

	_mqnic_scheduler_disable(sched);

	list_for_each_entry_safe(port, port_safe, &sched->sched_port_list, list) {
		mqnic_interface_unregister_sched_port(sched->interface, port);
		list_del(&port->list);
		mqnic_destroy_sched_port(port);
	}

	kfree(sched);
}

int mqnic_scheduler_enable(struct mqnic_sched *sched)
{
	if (sched->enable_count == 0)
		_mqnic_scheduler_enable(sched);

	sched->enable_count++;

	return 0;
}
EXPORT_SYMBOL(mqnic_scheduler_enable);

void mqnic_scheduler_disable(struct mqnic_sched *sched)
{
	sched->enable_count--;

	if (sched->enable_count == 0)
		_mqnic_scheduler_disable(sched);
}
EXPORT_SYMBOL(mqnic_scheduler_disable);

int mqnic_scheduler_channel_enable(struct mqnic_sched *sched, int port, int tc)
{
	int ch = sched->tc_count*port + tc;

	iowrite32(1, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_CTRL + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);

	return 0;
}
EXPORT_SYMBOL(mqnic_scheduler_channel_enable);

void mqnic_scheduler_channel_disable(struct mqnic_sched *sched, int port, int tc)
{
	int ch = sched->tc_count*port + tc;

	iowrite32(0, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_CTRL + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_disable);

void mqnic_scheduler_channel_set_dest(struct mqnic_sched *sched, int port, int tc, int val)
{
	int ch = sched->tc_count*port + tc;

	iowrite16(val, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC1_DEST + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_set_dest);

int mqnic_scheduler_channel_get_dest(struct mqnic_sched *sched, int port, int tc)
{
	int ch = sched->tc_count*port + tc;

	return ioread16(sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC1_DEST + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_get_dest);

void mqnic_scheduler_channel_set_pkt_budget(struct mqnic_sched *sched, int port, int tc, int val)
{
	int ch = sched->tc_count*port + tc;

	iowrite16(val, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC1_PB + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_set_pkt_budget);

int mqnic_scheduler_channel_get_pkt_budget(struct mqnic_sched *sched, int port, int tc)
{
	int ch = sched->tc_count*port + tc;

	return ioread16(sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC1_PB + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_get_pkt_budget);

void mqnic_scheduler_channel_set_data_budget(struct mqnic_sched *sched, int port, int tc, int val)
{
	int ch = sched->tc_count*port + tc;

	val = (val + sched->fc_scale-1) / sched->fc_scale;
	iowrite16(val, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC2_DB + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_set_data_budget);

int mqnic_scheduler_channel_get_data_budget(struct mqnic_sched *sched, int port, int tc)
{
	int ch = sched->tc_count*port + tc;

	return (int)ioread16(sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC2_DB + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE) * sched->fc_scale;
}
EXPORT_SYMBOL(mqnic_scheduler_channel_get_data_budget);

void mqnic_scheduler_channel_set_pkt_limit(struct mqnic_sched *sched, int port, int tc, int val)
{
	int ch = sched->tc_count*port + tc;

	iowrite16(val, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC2_PL + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_set_pkt_limit);

int mqnic_scheduler_channel_get_pkt_limit(struct mqnic_sched *sched, int port, int tc)
{
	int ch = sched->tc_count*port + tc;

	return ioread16(sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC2_PL + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_get_pkt_limit);

void mqnic_scheduler_channel_set_data_limit(struct mqnic_sched *sched, int port, int tc, int val)
{
	int ch = sched->tc_count*port + tc;

	val = (val + sched->fc_scale-1) / sched->fc_scale;
	iowrite32(val, sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC3_DL + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE);
}
EXPORT_SYMBOL(mqnic_scheduler_channel_set_data_limit);

int mqnic_scheduler_channel_get_data_limit(struct mqnic_sched *sched, int port, int tc)
{
	int ch = sched->tc_count*port + tc;

	return (int)ioread32(sched->rb->regs + MQNIC_RB_SCHED_RR_REG_CH0_FC3_DL + ch*MQNIC_RB_SCHED_RR_REG_CH_STRIDE) * sched->fc_scale;
}
EXPORT_SYMBOL(mqnic_scheduler_channel_get_data_limit);

int mqnic_scheduler_queue_enable(struct mqnic_sched *sched, int queue)
{
	iowrite32(MQNIC_SCHED_RR_CMD_SET_QUEUE_ENABLE | 1, sched->hw_addr + sched->queue_stride*queue);

	return 0;
}
EXPORT_SYMBOL(mqnic_scheduler_queue_enable);

void mqnic_scheduler_queue_disable(struct mqnic_sched *sched, int queue)
{
	iowrite32(MQNIC_SCHED_RR_CMD_SET_QUEUE_ENABLE | 0, sched->hw_addr + sched->queue_stride*queue);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_disable);

void mqnic_scheduler_queue_set_pause(struct mqnic_sched *sched, int queue, int val)
{
	iowrite32(MQNIC_SCHED_RR_CMD_SET_QUEUE_PAUSE | (val ? 1 : 0), sched->hw_addr + sched->queue_stride*queue);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_set_pause);

int mqnic_scheduler_queue_get_pause(struct mqnic_sched *sched, int queue)
{
	return !!(ioread32(sched->hw_addr + sched->queue_stride*queue) & MQNIC_SCHED_RR_QUEUE_PAUSE);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_get_pause);

int mqnic_scheduler_queue_port_enable(struct mqnic_sched *sched, int queue, int port)
{
	iowrite32(MQNIC_SCHED_RR_CMD_SET_PORT_ENABLE | (port << 8) | 1, sched->hw_addr + sched->queue_stride*queue);

	return 0;
}
EXPORT_SYMBOL(mqnic_scheduler_queue_port_enable);

void mqnic_scheduler_queue_port_disable(struct mqnic_sched *sched, int queue, int port)
{
	iowrite32(MQNIC_SCHED_RR_CMD_SET_PORT_ENABLE | (port << 8) | 0, sched->hw_addr + sched->queue_stride*queue);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_port_disable);

void mqnic_scheduler_queue_port_set_pause(struct mqnic_sched *sched, int queue, int port, int val)
{
	iowrite32(MQNIC_SCHED_RR_CMD_SET_PORT_PAUSE | (port << 8) | (val ? 1 : 0), sched->hw_addr + sched->queue_stride*queue);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_port_set_pause);

int mqnic_scheduler_queue_port_get_pause(struct mqnic_sched *sched, int queue, int port)
{
	return !!((ioread32(sched->hw_addr + sched->queue_stride*queue) >> port*8) & MQNIC_SCHED_RR_PORT_PAUSE);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_port_get_pause);

void mqnic_scheduler_queue_port_set_tc(struct mqnic_sched *sched, int queue, int port, int val)
{
	iowrite32(MQNIC_SCHED_RR_CMD_SET_PORT_TC | (port << 8) | (val & 0x7), sched->hw_addr + sched->queue_stride*queue);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_port_set_tc);

int mqnic_scheduler_queue_port_get_tc(struct mqnic_sched *sched, int queue, int port)
{
	return !!((ioread32(sched->hw_addr + sched->queue_stride*queue) >> port*8) & MQNIC_SCHED_RR_PORT_TC);
}
EXPORT_SYMBOL(mqnic_scheduler_queue_port_get_tc);
