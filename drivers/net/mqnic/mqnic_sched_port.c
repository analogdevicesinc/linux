// SPDX-License-Identifier: BSD-2-Clause-Views
/*
 * Copyright (c) 2024 The Regents of the University of California
 */

#include "mqnic.h"

struct mqnic_sched_port *mqnic_create_sched_port(struct mqnic_sched *sched, int index)
{
	struct mqnic_sched_port *port;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	port->sched = sched;

	port->index = index;

	return port;
}

void mqnic_destroy_sched_port(struct mqnic_sched_port *port)
{
	kfree(port);
}

int mqnic_sched_port_enable(struct mqnic_sched_port *port)
{
	return mqnic_scheduler_enable(port->sched);
}
EXPORT_SYMBOL(mqnic_sched_port_enable);

void mqnic_sched_port_disable(struct mqnic_sched_port *port)
{
	mqnic_scheduler_disable(port->sched);
}
EXPORT_SYMBOL(mqnic_sched_port_disable);

int mqnic_sched_port_channel_enable(struct mqnic_sched_port *port, int tc)
{
	return mqnic_scheduler_channel_enable(port->sched, port->index, tc);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_enable);

void mqnic_sched_port_channel_disable(struct mqnic_sched_port *port, int tc)
{
	mqnic_scheduler_channel_disable(port->sched, port->index, tc);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_disable);

void mqnic_sched_port_channel_set_dest(struct mqnic_sched_port *port, int tc, int val)
{
	mqnic_scheduler_channel_set_dest(port->sched, port->index, tc, val);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_set_dest);

int mqnic_sched_port_channel_get_dest(struct mqnic_sched_port *port, int tc)
{
	return mqnic_scheduler_channel_get_dest(port->sched, port->index, tc);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_get_dest);

void mqnic_sched_port_channel_set_pkt_budget(struct mqnic_sched_port *port, int tc, int val)
{
	mqnic_scheduler_channel_set_pkt_budget(port->sched, port->index, tc, val);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_set_pkt_budget);

int mqnic_sched_port_channel_get_pkt_budget(struct mqnic_sched_port *port, int tc)
{
	return mqnic_scheduler_channel_get_pkt_budget(port->sched, port->index, tc);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_get_pkt_budget);

void mqnic_sched_port_channel_set_data_budget(struct mqnic_sched_port *port, int tc, int val)
{
	mqnic_scheduler_channel_set_data_budget(port->sched, port->index, tc, val);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_set_data_budget);

int mqnic_sched_port_channel_get_data_budget(struct mqnic_sched_port *port, int tc)
{
	return mqnic_scheduler_channel_get_data_budget(port->sched, port->index, tc);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_get_data_budget);

void mqnic_sched_port_channel_set_pkt_limit(struct mqnic_sched_port *port, int tc, int val)
{
	mqnic_scheduler_channel_set_pkt_limit(port->sched, port->index, tc, val);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_set_pkt_limit);

int mqnic_sched_port_channel_get_pkt_limit(struct mqnic_sched_port *port, int tc)
{
	return mqnic_scheduler_channel_get_pkt_limit(port->sched, port->index, tc);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_get_pkt_limit);

void mqnic_sched_port_channel_set_data_limit(struct mqnic_sched_port *port, int tc, int val)
{
	mqnic_scheduler_channel_set_data_limit(port->sched, port->index, tc, val);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_set_data_limit);

int mqnic_sched_port_channel_get_data_limit(struct mqnic_sched_port *port, int tc)
{
	return mqnic_scheduler_channel_get_data_limit(port->sched, port->index, tc);
}
EXPORT_SYMBOL(mqnic_sched_port_channel_get_data_limit);

int mqnic_sched_port_queue_enable(struct mqnic_sched_port *port, int queue)
{
	int ret = mqnic_scheduler_queue_enable(port->sched, queue);

	if (ret)
		return ret;

	return mqnic_scheduler_queue_port_enable(port->sched, port->index, queue);
}
EXPORT_SYMBOL(mqnic_sched_port_queue_enable);

void mqnic_sched_port_queue_disable(struct mqnic_sched_port *port, int queue)
{
	mqnic_scheduler_queue_port_disable(port->sched, port->index, queue);
	mqnic_scheduler_queue_disable(port->sched, queue);
}
EXPORT_SYMBOL(mqnic_sched_port_queue_disable);

void mqnic_sched_port_queue_set_pause(struct mqnic_sched_port *port, int queue, int val)
{
	mqnic_scheduler_queue_port_set_pause(port->sched, port->index, queue, val);
}
EXPORT_SYMBOL(mqnic_sched_port_queue_set_pause);

int mqnic_sched_port_queue_get_pause(struct mqnic_sched_port *port, int queue)
{
	return mqnic_scheduler_queue_port_get_pause(port->sched, port->index, queue);
}
EXPORT_SYMBOL(mqnic_sched_port_queue_get_pause);

void mqnic_sched_port_queue_set_tc(struct mqnic_sched_port *port, int queue, int val)
{
	mqnic_scheduler_queue_port_set_tc(port->sched, port->index, queue, val);
}
EXPORT_SYMBOL(mqnic_sched_port_queue_set_tc);

int mqnic_sched_port_queue_get_tc(struct mqnic_sched_port *port, int queue)
{
	return mqnic_scheduler_queue_port_get_tc(port->sched, port->index, queue);
}
EXPORT_SYMBOL(mqnic_sched_port_queue_get_tc);
