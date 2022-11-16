// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Multicore communication on dual-core ADI SC5XX processor
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/bitmap.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched/signal.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <asm/dma.h>
#include "../include/icc.h"
#include <linux/soc/adi/icc.h>
#include <linux/soc/adi/hardware.h>
#include <linux/soc/adi/cpu.h>

#define DRIVER_NAME "icc"

#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
void __iomem *core0_mcapi_sram_addr;
#endif

static struct adi_icc {
	struct miscdevice mdev;
	struct device *dev;
	void __iomem *base;
	unsigned int node;
	int irq;
	int irq_type;
	int peer_count;
	wait_queue_head_t icc_rx_wait;
	struct sm_session_table *sessions_table;
	struct sm_icc_desc *icc_info;
	struct sm_proto *sm_protos[SP_MAX];
	struct proc_dir_entry *icc_dump;
	char name[20];
	struct adi_tru *adi_tru;
} *adi_icc;

void icc_send_ipi_cpu(unsigned int cpu, int irq)
{
	adi_tru_trigger(adi_icc->adi_tru, irq);
}

void icc_clear_ipi_cpu(unsigned int cpu, int irq)
{
	//platform_clear_ipi(cpu, irq);
}

static void wakeup_icc_thread(struct sm_icc_desc *icc_info)
{
	wake_up(&adi_icc->icc_rx_wait);
}

static struct sm_icc_desc *get_icc_peer(struct sm_msg *msg)
{
	struct sm_icc_desc *icc_info = adi_icc->icc_info;
	int i;
	u32 msg_addr = (uint32_t)msg;

	if (!msg) {
		WARN_ON(!msg);
		return NULL;
	}

	for (i = 0; i < adi_icc->peer_count; i++) {
		if (((uint32_t)icc_info[i].icc_high_queue < msg_addr) &&
		    (msg_addr < (uint32_t)icc_info[i].icc_high_queue + MSGQ_SIZE))
			return &icc_info[i];
	}

	return NULL;
}

static struct sm_message_queue *icc_get_inqueue(struct sm_msg *msg)
{
	struct sm_icc_desc *icc_info;
	struct sm_message_queue *queue;

	if (!msg)
		return NULL;

	icc_info = get_icc_peer(msg);

	if (!icc_info) {
		WARN_ON(!icc_info);
		return NULL;
	}

	queue = icc_info->icc_high_queue;

	while ((uint32_t)msg > (uint32_t)queue)
		queue++;

	return queue - 1;
}

static struct sm_message *get_msg_from_tx_list(struct sm_session *session,
					       uint32_t payload)
{
	struct sm_message *message;

	if (!session) {
		sm_debug("%s, please check session(0x%p)\n", __func__, session);
		return NULL;
	}
	mutex_lock(&adi_icc->sessions_table->lock);
	list_for_each_entry(message, &session->tx_messages, next) {
		if (payload == message->msg.payload) {
			sm_debug("find TX matched free buf %x message %p %p %p\n",
				 (uint32_t)message->msg.payload, message,
				 message->next.next, message->next.prev);
			mutex_unlock(&adi_icc->sessions_table->lock);
			return message;
		}
	}
	mutex_unlock(&adi_icc->sessions_table->lock);
	sm_debug("%s, can't find matched message, payload:0x%x\n",
		 __func__, payload);
	return NULL;
}

static struct sm_message *get_message_from_rx_list(struct sm_session *session)
{
	struct sm_message *message;

	if (!session) {
		sm_debug("%s, please check session(0x%p)\n", __func__, session);
		return NULL;
	}
	mutex_lock(&adi_icc->sessions_table->lock);
	message = list_first_entry(&session->rx_messages,
				   struct sm_message, next);
	session->n_avail--;
	mutex_unlock(&adi_icc->sessions_table->lock);
	return message;
}

static int fill_user_buffer_from_message(struct sm_session *session,
					 struct sm_message *message,
					 void *user_buf, uint32_t *buf_len)
{
#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
	void __iomem *remap_addr = NULL;
#endif

	if (!session || !message || !buf_len) {
		sm_debug("%s, please check session(0x%p), message(0x%p), buf_len(0x%p)\n",
			 __func__, session, message, buf_len);
		return -ENOMEM;
	}
	if (message->msg.length < *buf_len)
		*buf_len = message->msg.length;
	if (message->msg.length && user_buf) {
#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
		remap_addr = ioremap(message->msg.payload, *buf_len);
		if (copy_to_user(user_buf, (void *)remap_addr, *buf_len))
			return -EFAULT;
		iounmap(remap_addr);
#else
		if (copy_to_user(user_buf,
				 (void *)IO_ADDRESS(message->msg.payload), *buf_len))
			return -EFAULT;
#endif
	}
	if (message->msg.type == SM_PACKET_READY)
		sm_send_packet_ack(session, message->msg.src_ep, message->src,
				   message->msg.payload, message->msg.length);
	else if (message->msg.type == SM_SESSION_PACKET_READY)
		sm_send_session_packet_ack(session, message->msg.src_ep, message->src,
					   message->msg.payload, message->msg.length);
	return 0;
}

static __maybe_unused int sm_get_icc_queue_attribute(u32 cpu,
						     u32 type, uint32_t *attribute)
{
	if (cpu > adi_icc->peer_count)
		return -EINVAL;
	if (type >= ICC_QUEUE_ATTR_MAX)
		return -EINVAL;
	if (!attribute)
		return -EINVAL;
	*attribute = *(adi_icc->icc_info[cpu - 1].icc_queue_attribute + type);
	return 0;
}

static  __maybe_unused int sm_set_icc_queue_attribute(u32 cpu,
						      u32 type, uint32_t attribute)
{
	if (cpu > adi_icc->peer_count)
		return -EINVAL;
	if (type >= ICC_QUEUE_ATTR_MAX)
		return -EINVAL;
	*(adi_icc->icc_info[cpu - 1].icc_queue_attribute + type) = attribute;
	return 0;
}

static struct sm_message_queue *sm_find_queue(struct sm_message *message,
					      struct sm_session *session)
{
	u16 dst = message->dst;
	struct sm_icc_desc *icc_info = adi_icc->icc_info;
	int i;

	if (!message)
		return NULL;

	for (i = 0; i < adi_icc->peer_count; i++) {
		if (icc_info[i].peer_cpu == dst)
			break;
	}

	if (i == adi_icc->peer_count)
		return NULL;

	message->icc_info = &icc_info[i];

	if (!session)
		return icc_info[i].icc_high_queue;

	if (session->queue_priority)
		return icc_info[i].icc_queue;

	return icc_info[i].icc_high_queue;
}

static int init_sm_session_table(struct adi_icc *icc)
{
	icc->sessions_table = kzalloc(sizeof(*icc->sessions_table), GFP_KERNEL);

	if (!icc->sessions_table)
		return -ENOMEM;

	mutex_init(&icc->sessions_table->lock);
	init_waitqueue_head(&icc->sessions_table->query_wait);
	icc->sessions_table->b_query_finish = 0;
	INIT_LIST_HEAD(&icc->sessions_table->query_message);
	icc->sessions_table->nfree = MAX_ENDPOINTS;
	return 0;
}

static int sm_message_enqueue(struct sm_message_queue *icc_queue, struct sm_msg *msg)
{
	/* icc_queue[1] is the queue to send message */
	struct sm_message_queue *outqueue = icc_queue + 1;
	u16 sent = sm_atomic_read(&outqueue->sent);
	u16 received = sm_atomic_read(&outqueue->received);
	u16 pending = sent - received;

	if (pending < 0)
		pending += USHRT_MAX;

	if (pending >= (SM_MSGQ_LEN - 1)) {
		sm_debug("over run\n");
		return -EAGAIN;
	}
	memcpy(&outqueue->messages[(sent % SM_MSGQ_LEN)], msg,
	       sizeof(struct sm_msg));
	sent++;
	sm_atomic_write(&outqueue->sent, sent);
	return 0;
}

static int sm_message_dequeue(struct sm_msg *msg)
{
	/* icc_queue[0] is the queue to receive message */
	struct sm_message_queue *inqueue = icc_get_inqueue(msg);
	u16 received;
	u16 sent;

	received = sm_atomic_read(&inqueue->received);
	sent = sm_atomic_read(&inqueue->sent);
	if (received == sent) {
		sm_debug("WARN_ON() sent %d receive %d\n", sent, received);
		WARN(1, "received == sent\n");
	}

	memset(msg, 0, sizeof(struct sm_msg));
	received++;
	sm_atomic_write(&inqueue->received, received);
	return 0;
}

static uint32_t sm_alloc_session(struct sm_session_table *table)
{
	unsigned long index;

	sm_debug("table bits1 %08x\n", table->bits[0]);
	index = find_next_zero_bit((unsigned long *)table->bits, BITS_PER_LONG, 0);
	if (index >= BITS_PER_LONG)
		return -EAGAIN;
	sm_debug("table index %ld\n", index);
	bitmap_set((unsigned long *)table->bits, index, 1);

	sm_debug("table bits2 %08x\n", table->bits[0]);
	table->nfree--;
	return index;
}

static int sm_free_session(u32 slot, struct sm_session_table *table)
{
	if (test_bit((int)slot, (unsigned long *)table->bits)) {
		memset(&table->sessions[slot], 0, sizeof(struct sm_session));
		__clear_bit((int)slot, (unsigned long *)table->bits);
		table->nfree++;
		return 0;
	}
	return -1;
}

static int
sm_find_session(u32 local_ep, uint32_t remote_ep,
		struct sm_session_table *table)
{
	unsigned long index;
	struct sm_session *session;

	sm_debug("%s bits %08x localep %d\n", __func__, table->bits[0], (int)local_ep);
	for_each_set_bit(index, (unsigned long *)table->bits, BITS_PER_LONG) {
		session = &table->sessions[index];
		sm_debug("index %ld ,local ep %d type %x\n", index,
			 (int)session->local_ep, (uint32_t)session->type);
		if (session->local_ep == local_ep) {
			if (remote_ep && session->remote_ep != remote_ep)
				return -EINVAL;
			goto found_slot;
		}
	}
	return -EINVAL;
found_slot:
	return index;
}

static int sm_create_session(u32 src_ep, uint32_t type, uint32_t queue_priority)
{
	struct sm_session_table *table = adi_icc->sessions_table;
	int index;
	struct sm_message *request;
	int ret = -EAGAIN;
	int pending = 0;

	mutex_lock(&table->lock);
	index = sm_find_session(src_ep, 0, table);
	if (index >= 0 && index < 32) {
		sm_debug("already bound index %d srcep %d\n", index, src_ep);
		ret = -EEXIST;
		goto fail_out;
	}
	if (type >= SP_MAX) {
		sm_debug("bad type %x\n", type);
		ret = -EINVAL;
		goto fail_out;
	}

	index = sm_alloc_session(table);
	if (index >= 0 && index < 32) {
		table->sessions[index].local_ep = src_ep;
		table->sessions[index].remote_ep = 0;
		table->sessions[index].pid = current->pid;
		table->sessions[index].flags = 0;
		table->sessions[index].n_uncompleted = 0;
		table->sessions[index].n_avail = 0;
		table->sessions[index].type = type;
		table->sessions[index].proto_ops = adi_icc->sm_protos[type];
		table->sessions[index].b_rx_finish = 0;
		table->sessions[index].queue_priority = queue_priority;
		INIT_LIST_HEAD(&table->sessions[index].rx_messages);
		INIT_LIST_HEAD(&table->sessions[index].tx_messages);
		init_waitqueue_head(&table->sessions[index].rx_wait);

		list_for_each_entry(request, &table->query_message, next) {
			if (request->msg.dst_ep == src_ep) {
				pending = 1;
				break;
			}
		}
		mutex_unlock(&table->lock);

		if (pending) {
			list_del(&request->next);
			sm_send_control_msg(&table->sessions[index], request->msg.src_ep,
					    request->src, 0, 0, SM_QUERY_ACK_MSG);
			kfree(request);
		}

		sm_debug("create ep index %d srcep %d type %x\n", index, src_ep, type);
		sm_debug("session %p\n", &table->sessions[index]);
		return index;
	}
fail_out:
	mutex_unlock(&table->lock);
	return ret;
}

static struct sm_session *sm_index_to_session(uint32_t session_idx)
{
	struct sm_session *session;
	struct sm_session_table *table = adi_icc->sessions_table;

	if (session_idx < 0 && session_idx >= MAX_SESSIONS)
		return NULL;
	if (!test_bit(session_idx, (unsigned long *)table->bits))
		return NULL;
	session = &table->sessions[session_idx];
	return session;
}

static __maybe_unused uint32_t sm_session_to_index(struct sm_session *session)
{
	struct sm_session_table *table = adi_icc->sessions_table;

	if ((session >= &table->sessions[0]) &&
	    (session < &table->sessions[MAX_SESSIONS])) {
		return (session - &table->sessions[0]) / sizeof(struct sm_session);
	}
	return -EINVAL;
}

static int __iccqueue_getpending(struct sm_message_queue *inqueue)
{
	u16 sent = sm_atomic_read(&inqueue->sent);
	u16 received = sm_atomic_read(&inqueue->received);
	u16 pending;

	pending = sent - received;
	if (pending < 0)
		pending += USHRT_MAX;
	pending %= SM_MSGQ_LEN;
	return pending;
}

static int iccqueue_getpending(struct sm_icc_desc *icc_info)
{
	/* icc_queue[0] is the queue to receive message */
	u16 pending;

	pending = __iccqueue_getpending(icc_info->icc_high_queue);
	if (pending)
		return pending;
	return __iccqueue_getpending(icc_info->icc_queue);
}

static int sm_send_message_internal(struct sm_session *session,
				    struct sm_message *message)
{
	struct sm_message_queue *icc_queue = sm_find_queue(message, session);
	struct sm_msg *msg = &message->msg;
	int ret = 0;
	struct sm_icc_desc *icc_info = message->icc_info;

	WARN_ON(!icc_info);
	if (!icc_queue)
		return -EINVAL;
	sm_debug("%s: dst %d %08x\n", __func__, icc_info->peer_cpu, (uint32_t)msg->type);
	ret = sm_message_enqueue(icc_queue, msg);
	if (!ret)
		icc_send_ipi_cpu(icc_info->peer_cpu, icc_info->notify);
	return ret;
}

int
sm_send_control_msg(struct sm_session *session, uint32_t remote_ep,
		    u32 dst_cpu, uint32_t payload,
			u32 len, uint32_t type)
{
	struct sm_message *message;
	int ret = 0;

	message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;

	message->msg.type = type;
	if (session)
		message->msg.src_ep = session->local_ep;
	message->msg.dst_ep = remote_ep;
	message->msg.length = len;
	message->msg.payload = payload;
	message->dst = dst_cpu;
	message->src = arm_core_id();

	ret = sm_send_message_internal(session, message);
	if (ret)
		wait_event_interruptible(message->icc_info->iccq_tx_wait,
					 !sm_send_message_internal(session, message));
	kfree(message);
	return ret;
}

int sm_send_packet_ack(struct sm_session *session, uint32_t remote_ep,
		       u32 dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_PACKET_CONSUMED);
}

int
sm_send_session_packet_ack(struct sm_session *session, uint32_t remote_ep,
			   u32 dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SESSION_PACKET_CONSUMED);
}

int sm_send_scalar_cmd(struct sm_session *session, uint32_t remote_ep,
		       u32 dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SCALAR_READY_64);
}

int sm_send_scalar_ack(struct sm_session *session, uint32_t remote_ep,
		       u32 dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SCALAR_CONSUMED);
}

int
sm_send_session_scalar_ack(struct sm_session *session, uint32_t remote_ep,
			   u32 dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SESSION_SCALAR_CONSUMED);
}

int sm_send_connect(struct sm_session *session, uint32_t remote_ep,
		    u32 dst_cpu, uint32_t type)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, type);
}

int sm_send_connect_ack(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CONNECT_ACK);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_SCALAR_CONNECT_ACK);
	else
		return -EINVAL;
}

int sm_send_connect_done(struct sm_session *session, uint32_t remote_ep,
			 uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CONNECT_DONE);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
				0, SM_SESSION_SCALAR_CONNECT_DONE);
	else
		return -EINVAL;
}

int sm_send_session_active(struct sm_session *session, uint32_t remote_ep,
			   uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_ACTIVE);
}

int sm_send_session_active_ack(struct sm_session *session, uint32_t remote_ep,
			       uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, SM_OPEN,
					0, SM_SESSION_PACKET_ACTIVE_ACK);
}

int sm_send_session_active_noack(struct sm_session *session, uint32_t remote_ep,
				 uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_ACTIVE_ACK);
}

int sm_send_close(struct sm_session *session, uint32_t remote_ep,
		  uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CLOSE);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_SCALAR_CLOSE);
	else
		return -EINVAL;
}

int sm_send_close_ack(struct sm_session *session, uint32_t remote_ep,
		      uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CLOSE_ACK);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_SCALAR_CLOSE_ACK);
	else
		return -EINVAL;
}

int sm_send_error(struct sm_session *session, uint32_t remote_ep,
		  uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_PACKET_ERROR);
}

static int
sm_send_scalar(u32 session_idx, uint32_t dst_ep, uint32_t dst_cpu,
	       u32 scalar0, uint32_t scalar1, uint32_t type, int nonblock)
{
	struct sm_session *session;
	struct sm_message *m;
	int ret = -EAGAIN;

	if (session_idx < 0 || session_idx >= MAX_SESSIONS)
		return -EINVAL;
	session = sm_index_to_session(session_idx);
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	if (session->type == SP_SESSION_SCALAR) {
		if (session->flags == SM_CONNECT)
			m->msg.dst_ep = session->remote_ep;
		else
			return -EINVAL;
	} else {
		m->msg.dst_ep = dst_ep;
	}

	m->msg.src_ep = session->local_ep;
	m->src = arm_core_id();
	m->dst = dst_cpu;
	m->msg.payload = scalar0;
	m->msg.length = scalar1;
	m->msg.type = type;

	if (session->proto_ops->sendmsg) {
		ret = session->proto_ops->sendmsg(m, session);
	} else {
		sm_debug("session type not supported\n");
		ret = 0;
	}

	if (ret)
		goto out;

	sm_debug("%s: scalar0 %x scalar1 %x type %x dst %d dstep %d src %d srcep %d\n",
		 __func__, scalar0, scalar1, (uint32_t)m->msg.type, (int)m->dst,
		 (int)m->msg.dst_ep, (int)m->src, (int)m->msg.src_ep);

	ret = sm_send_message_internal(session, m);
	if (ret) {
		sm_debug(">>>>1 sleep on send queue\n");
		wait_event_interruptible_timeout(m->icc_info->iccq_tx_wait,
						 !sm_send_message_internal(session, m), HZ);
		sm_debug(">>>>1 wakeup send queue\n");
	} else if (!nonblock) {
		sm_debug(">>>>2 sleep on send queue\n");
		wait_event_interruptible_timeout(m->icc_info->iccq_tx_wait,
						 m->b_tx_finish == 1, 10 * HZ);
		sm_debug(">>>>2 wakeup send queue\n");
	}

out:
	return ret;
}

static int
sm_send_packet(u32 session_idx, uint32_t dst_ep, uint32_t dst_cpu,
	       void *buf, uint32_t len, uint32_t *payload, int nonblock)
{
	struct sm_session *session;
	struct sm_message *m;
	void *payload_buf = NULL;
	dma_addr_t dma_handle;
	int ret = -EAGAIN;

	*payload = 0;
	if (session_idx < 0 || session_idx >= MAX_SESSIONS)
		return -EINVAL;
	session = sm_index_to_session(session_idx);
	sm_debug("%s: %u %p\n", __func__, session_idx, session);
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	if (session->type == SP_SESSION_PACKET) {
		if (session->flags == SM_CONNECT)
			m->msg.dst_ep = session->remote_ep;
		else
			return -EINVAL;
	} else {
		m->msg.dst_ep = dst_ep;
	}

	m->msg.src_ep = session->local_ep;
	m->src = arm_core_id();
	m->dst = dst_cpu;
	m->msg.length = len;
	if (session->type == SP_SESSION_PACKET)
		m->msg.type = SM_SESSION_PACKET_READY;
	else
		m->msg.type = SM_PACKET_READY;

	if (m->msg.length) {
#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC57X)
		//On the SC59x
		//dma_alloc_coherent appears to be allocating RAM
		//which is cached on the SHARC cores
		//Rather than flushing... let's use the uncached SRAM region
		//16KB @ 0x20001000

		core0_mcapi_sram_addr = ioremap(0x20001000, 0x1000);
		payload_buf = core0_mcapi_sram_addr;
		m->msg.payload = 0x20001000;
#elif defined(CONFIG_ARCH_SC58X)
		core0_mcapi_sram_addr = ioremap(0x20081000, 0x1000);
		payload_buf = core0_mcapi_sram_addr;
		m->msg.payload = 0x20081000;
#else
		payload_buf = dma_alloc_coherent(NULL, m->msg.length,
						 &dma_handle, GFP_KERNEL);
		m->msg.payload = dma_handle;
#endif

		if (!payload_buf) {
			ret = -ENOMEM;
			goto fail1;
		}
		m->vbuf = payload_buf;
		sm_debug("alloc buffer %x\n", (uint32_t)payload_buf);

		if (copy_from_user((void *)payload_buf, buf, m->msg.length)) {
			ret = -EFAULT;
			sm_debug("copy failed\n");
			goto fail2;
		}
	}

	if (session->proto_ops->sendmsg) {
		ret = session->proto_ops->sendmsg(m, session);
	} else {
		sm_debug("session type not supported\n");
		ret = 0;
	}
	if (ret)
		goto fail2;

	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__,
		 (int)m->msg.length, (int)m->msg.type, (int)m->dst,
		 (int)m->msg.dst_ep, (int)m->src, (int)m->msg.src_ep);

	ret = sm_send_message_internal(session, m);
	if (((ret == -EAGAIN) && nonblock) || (ret == -EINVAL)) {
		ret = -EFAULT;
		goto fail3;
	} else if ((ret == -EAGAIN) && !nonblock) {
		sm_debug(">>>>1 sleep on send queue\n");
		ret = wait_event_interruptible_timeout(m->icc_info->iccq_tx_wait,
						       !sm_send_message_internal(session, m), HZ);
		if (!ret) {
			sm_debug("<<<<1 timeout(1s) wake out\n");
			ret = -ETIMEDOUT;
			goto fail3;
		} else if (ret == -ERESTARTSYS) {
			sm_debug("<<<<1 woken by a signal\n");
			ret = -ERESTARTSYS;
			goto fail3;
		} else {
			sm_debug("<<<<1 wakeup send queue, ret:%d\n", ret);
			ret = 0;
		}
	}
	if (ret == 0) {
		if (m->b_tx_finish) {
			ret = 0;
			goto fail3;
		}
		if (nonblock) {
			ret = -EAGAIN;
			#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC57X)
				*payload = 0x20001000;
			#elif defined(CONFIG_ARCH_SC58X)
				*payload = 0x20081000;
			#else
				*payload = dma_handle;
			#endif
			goto out;
		} else {
			sm_debug(">>>>2 sleep on send queue\n");
			ret = wait_event_interruptible_timeout(m->icc_info->iccq_tx_wait,
							       m->b_tx_finish == 1, 10 * HZ);
			if (!ret) {
				sm_debug("<<<<2 send timeout(10s) wake out\n");
				ret = -ETIMEDOUT;
			} else if (ret == -ERESTARTSYS) {
				sm_debug("<<<<2 send woken by a signal\n");
				ret = -ERESTARTSYS;
			} else {
				sm_debug("<<<<2 wakeup send queue, ret:%d\n", ret);
				ret = 0;
			}
		}
	}
fail3:
	mutex_lock(&adi_icc->sessions_table->lock);
	list_del(&m->next);
	mutex_unlock(&adi_icc->sessions_table->lock);
fail2:
	if (dma_handle)
		dma_free_coherent(NULL, m->msg.length, payload_buf, dma_handle);
#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
	if (core0_mcapi_sram_addr) {
		iounmap(core0_mcapi_sram_addr);
		core0_mcapi_sram_addr = NULL;
	}
#endif
fail1:
	kfree(m);
out:
	return ret;
}

static int sm_recv_scalar(u32 session_idx, uint32_t *src_ep,
			  u32 *src_cpu, uint32_t *scalar0, uint32_t *scalar1,
		u32 *type, int nonblock)
{
	struct sm_session *session = NULL;
	struct sm_message *message = NULL;
	struct sm_msg *msg = NULL;
	int ret;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	mutex_lock(&adi_icc->sessions_table->lock);
	session->b_rx_finish = 0;
	mutex_unlock(&adi_icc->sessions_table->lock);

	sm_debug("recv sleep on queue index %s index %d\n", __func__, session_idx);

	if (list_empty(&session->rx_messages)) {
		sm_debug("recv sleep on queue\n");
		if (nonblock)
			return -EAGAIN;
		ret = wait_event_interruptible_timeout(session->rx_wait,
						       !list_empty(&session->rx_messages), 10 * HZ);
		if (ret)
			return ret;
	}

	mutex_lock(&adi_icc->sessions_table->lock);
	message = list_first_entry(&session->rx_messages, struct sm_message, next);
	msg = &message->msg;

	list_del(&message->next);
	mutex_unlock(&adi_icc->sessions_table->lock);

	if (src_ep)
		*src_ep = msg->src_ep;

	if (src_cpu)
		*src_cpu = message->src;

	if (scalar0)
		*scalar0 = msg->payload;
	if (scalar1)
		*scalar1 = msg->length;

	if (type) {
		switch (msg->type) {
		case SM_SCALAR_READY_8:
		case SM_SESSION_SCALAR_READY_8:
			*type = 1;
			break;
		case SM_SCALAR_READY_16:
		case SM_SESSION_SCALAR_READY_16:
			*type = 2;
			break;
		case SM_SCALAR_READY_32:
		case SM_SESSION_SCALAR_READY_32:
			*type = 4;
			break;
		case SM_SCALAR_READY_64:
		case SM_SESSION_SCALAR_READY_64:
			*type = 8;
			break;
		}
	}

	sm_debug("scalar0 %x, scalar1 %x type %d\n", *scalar0, *scalar1, *type);
	session->n_avail--;
	kfree(message);

	sm_debug("leave recv scalar\n");
	return 0;
}

static int sm_recv_packet(u32 session_idx, uint32_t *src_ep,
			  u32 *src_cpu, void *user_buf, uint32_t *buf_len, int nonblock)
{
	struct sm_session *session = NULL;
	struct sm_message *message = NULL;
	int ret = 0;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	mutex_lock(&adi_icc->sessions_table->lock);
	session->b_rx_finish = 0;
	mutex_unlock(&adi_icc->sessions_table->lock);
	if (list_empty(&session->rx_messages)) {
		if (nonblock)
			return -EAGAIN;
		sm_debug("recv sleep on queue\n");
		ret = wait_event_interruptible_timeout(session->rx_wait,
						       !list_empty(&session->rx_messages), 10 * HZ);
		if (!ret) {
			sm_debug("%s, recv: timeout\n", __func__);
			return -ETIMEDOUT;
		}

		if (ret == -ERESTARTSYS) {
			sm_debug("%s, recv: woken by a signal\n", __func__);
			return -ERESTARTSYS;
		}

		sm_debug("%s, recv: wake out, ret:%d\n", __func__, ret);
		ret = 0;
	}

	message = get_message_from_rx_list(session);

	if (!message)
		return -EINVAL;
	if (src_ep)
		*src_ep = message->msg.src_ep;

	if (src_cpu)
		*src_cpu = message->src;

	ret = fill_user_buffer_from_message(session, message, user_buf, buf_len);
	mutex_lock(&adi_icc->sessions_table->lock);
	list_del(&message->next);
	kfree(message);
	mutex_unlock(&adi_icc->sessions_table->lock);
	return ret;
}

static int sm_query_remote_ep(u32 dst_ep, uint32_t dst_cpu,
			      int timeout, int nonblock)
{
	int ret;
	struct sm_session_table *t;

	mutex_lock(&adi_icc->sessions_table->lock);
	adi_icc->sessions_table->b_query_finish = 0;
	mutex_unlock(&adi_icc->sessions_table->lock);
	sm_debug("%s dst_ep %d dst_cpu %d, nonblock:%d\n",
		 __func__, dst_ep, dst_cpu, nonblock);
	ret = sm_send_control_msg(NULL, dst_ep, dst_cpu, 0,
				  0, SM_QUERY_MSG);
	if (ret < 0) {
		sm_debug("%s send control msg error(%d)\n", __func__, ret);
		return -EFAULT;
	}

	if (adi_icc->sessions_table->b_query_finish)
		return 0;

	if (nonblock)
		return -EAGAIN;

	sm_debug("%s, query ack: ready to sleep\n", __func__);
	t = adi_icc->sessions_table;
	ret = wait_event_interruptible_timeout(t->query_wait,
					       t->b_query_finish == 1,
					       timeout);
	if (!ret) {
		sm_debug("%s, query ack: timeout(%d) wake out\n",
			 __func__, timeout);
		ret = -ETIMEDOUT;
	} else if (ret == -ERESTARTSYS) {
		sm_debug("%s, query ack: woken by signal\n", __func__);
	} else {
		sm_debug("%s, query ack: received query ack, ret:%d\n",
			 __func__, ret);
		ret = 0;
	}
	return ret;
}

static int
sm_wait_for_connect_ack(struct sm_session *session)
{
	return wait_event_interruptible(session->rx_wait,
			session->flags == SM_CONNECT);
}

static int
sm_wait_request_finish(u32 session_idx, void *user_buf, uint32_t dst_cpu,
		       int timeout, uint32_t request_type, uint32_t *buf_len,
	u32 payload, int nonblock)
{
	int ret = 0;
	struct sm_message *m;
	struct sm_session *session;
	struct sm_session_table *t;

	if (!buf_len) {
		sm_debug("%s, please check buf_len(0x%p)\n", __func__, buf_len);
		return -ENOMEM;
	}
	session = sm_index_to_session(session_idx);
	if (!session && (request_type != WAIT_GET_ENDPT))
		return -EINVAL;
	if (nonblock) {
		if (request_type == WAIT_SEND) {
			m = get_msg_from_tx_list(session, payload);
			if (!m)
				return -EINVAL;
			if (m->b_tx_finish) {
				ret = 0;
				*buf_len = m->msg.length;
				mutex_lock(&adi_icc->sessions_table->lock);
				list_del(&m->next);
				if (payload) {
#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
					if (core0_mcapi_sram_addr) {
						iounmap(core0_mcapi_sram_addr);
						core0_mcapi_sram_addr = NULL;
					}
#else
					dma_free_coherent(NULL, m->msg.length,
							  m->vbuf, payload);
#endif
				}
				kfree(m);
				mutex_unlock(&adi_icc->sessions_table->lock);
			} else {
				ret = -EAGAIN;
			}
			return ret;
		}

		if (request_type == WAIT_RECV) {
			if (session->b_rx_finish) {
				m = get_message_from_rx_list(session);
				if (!m)
					return -EINVAL;
				ret = fill_user_buffer_from_message(session,
								    m,
								    user_buf,
								    buf_len);
				mutex_lock(&adi_icc->sessions_table->lock);
				list_del(&m->next);
				kfree(m);
				mutex_unlock(&adi_icc->sessions_table->lock);
			} else {
				ret = -EAGAIN;
			}
			return ret;
		}

		if (request_type == WAIT_GET_ENDPT) {
			if (adi_icc->sessions_table->b_query_finish)
				ret = 0;
			else
				ret = -EAGAIN;

			return ret;
		}

		sm_debug("%s:%d unsupported request type:%d\n",
			 __func__, __LINE__, request_type);
		ret = -EINVAL;

	} else {
		if (request_type == WAIT_SEND) {
			m = get_msg_from_tx_list(session, payload);
			if (!m)
				return -EINVAL;
			if (!m->b_tx_finish) {
				sm_debug("%s, send: ready to sleep\n", __func__);
				ret = wait_event_interruptible_timeout(m->icc_info->iccq_tx_wait,
								       m->b_tx_finish == 1,
								       timeout);
				if (!ret) {
					sm_debug("%s, send: timeout(%d) wake out\n",
						 __func__, timeout);
					ret = -ETIMEDOUT;
				} else if (ret == -ERESTARTSYS) {
					sm_debug("%s, send: woken by a signal\n", __func__);
				} else {
					sm_debug("%s, send: wake out, ret:%d\n", __func__, ret);
					ret = 0;
				}
			}
			*buf_len = m->msg.length;
			mutex_lock(&adi_icc->sessions_table->lock);
			list_del(&m->next);
			if (payload) {
#if defined(CONFIG_ARCH_SC59X) || defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
				if (core0_mcapi_sram_addr) {
					iounmap(core0_mcapi_sram_addr);
					core0_mcapi_sram_addr = NULL;
				}
#else
				dma_free_coherent(NULL, m->msg.length,
						  m->vbuf, payload);
#endif
			}
			kfree(m);
			mutex_unlock(&adi_icc->sessions_table->lock);
		} else if (request_type == WAIT_RECV) {
			if (!session->b_rx_finish) {
				sm_debug("%s, recv: ready to sleep\n", __func__);
				ret = wait_event_interruptible_timeout(session->rx_wait,
								       session->b_rx_finish == 1,
										timeout);
				if (!ret) {
					sm_debug("%s, recv: timeout(%d) wake out\n",
						 __func__, timeout);
					ret = -ETIMEDOUT;
					return ret;
				}

				if (ret == -ERESTARTSYS) {
					sm_debug("%s, recv:  woken by a signal\n", __func__);
					return ret;
				}

				sm_debug("%s, recv: wake out, ret:%d\n", __func__, ret);
				ret = 0;
			}
			m = get_message_from_rx_list(session);
			if (!m)
				return -EINVAL;
			ret = fill_user_buffer_from_message(session, m,
							    user_buf, buf_len);
			mutex_lock(&adi_icc->sessions_table->lock);
			list_del(&m->next);
			kfree(m);
			mutex_unlock(&adi_icc->sessions_table->lock);
		} else if (request_type == WAIT_GET_ENDPT) {
			if (!adi_icc->sessions_table->b_query_finish) {
				sm_debug("%s, get_endpt: ready to sleep\n", __func__);
				t = adi_icc->sessions_table;
				ret = wait_event_interruptible_timeout(t->query_wait,
								       t->b_query_finish == 1,
								       timeout);
				if (!ret) {
					sm_debug("%s, get_endpt: timeout(%d) wake out\n",
						 __func__, timeout);
					ret = -ETIMEDOUT;
				} else if (ret == -ERESTARTSYS) {
					sm_debug("%s, get_endpt: woken by a signal\n", __func__);
				} else {
					sm_debug("%s, get_endpt: wake out, ret:%d\n",
						 __func__, ret);
					ret = 0;
				}
			}
		} else {
			sm_debug("%s:%d unsupported request type:%d\n",
				 __func__, __LINE__, request_type);
			ret = -EINVAL;
		}
	}
	return ret;
}

static int sm_get_remote_session_active(u32 session_idx,
					u32 dst_ep, uint32_t dst_cpu)
{
	struct sm_session *session;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	session->type = SP_SESSION_PACKET;
	sm_send_session_active(session, dst_ep, dst_cpu);
	wait_event_interruptible_timeout(session->rx_wait,
					 (session->flags & SM_ACTIVE), 5);

	if (session->flags & SM_ACTIVE) {
		sm_debug("received active ack\n");
		return 0;
	}
	return -EAGAIN;
}

static int sm_connect_session(u32 session_idx, uint32_t dst_ep,
			      u32 dst_cpu, uint32_t type)
{
	struct sm_session *session;
	u32 msg_type;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	if (type == SP_SESSION_SCALAR) {
		session->type = SP_SESSION_SCALAR;
		msg_type = SM_SESSION_SCALAR_CONNECT;
	} else if (type == SP_SESSION_PACKET) {
		session->type = SP_SESSION_PACKET;
		msg_type = SM_SESSION_PACKET_CONNECT;
	} else {
		return -EINVAL;
	}
	sm_send_connect(session, dst_ep, dst_cpu, msg_type);
	if (sm_wait_for_connect_ack(session))
		return -EAGAIN;
	sm_debug("received connect ack\n");
	if (session->remote_ep == dst_ep)
		sm_debug("auto accept\n");

	sm_send_connect_done(session, session->remote_ep, dst_cpu);
	return 0;
}

static __maybe_unused int sm_disconnect_session(u32 dst_ep,
						u32 src_ep, struct sm_session_table *table)
{
	u32 slot = sm_find_session(src_ep, 0, table);

	if (slot < 0)
		return -EINVAL;

	table->sessions[slot].remote_ep = 0;
	table->sessions[slot].flags = 0;
}

static int sm_open_session(uint32_t index)
{
	struct sm_session *session;

	session = sm_index_to_session(index);
	if (!session)
		return -EINVAL;
	if (session->flags == SM_CONNECT) {
		session->flags |= SM_OPEN;
		return 0;
	}
	return -EINVAL;
}

static int sm_close_session(uint32_t index)
{
	struct sm_session *session;

	session = sm_index_to_session(index);
	if (!session)
		return -EINVAL;
	if (session->flags & SM_OPEN) {
		session->flags &= ~SM_OPEN;
		return 0;
	}
	return -EINVAL;
}

static int sm_destroy_session(uint32_t session_idx)
{
	struct sm_message *message;
	struct sm_msg *msg;
	struct sm_session *session;
	struct sm_session_table *table = adi_icc->sessions_table;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;

	mutex_lock(&table->lock);
	while (!list_empty(&session->rx_messages)) {
		sm_debug("drain rx list\n");
		message = list_first_entry(&session->rx_messages,
					   struct sm_message, next);
		msg = &message->msg;

		if (session->flags == SM_CONNECT)
			sm_send_session_packet_ack(session, msg->src_ep,
						   message->src, msg->payload, msg->length);
		else
			sm_send_packet_ack(session, msg->src_ep,
					   message->src, msg->payload, msg->length);
		list_del(&message->next);
		kfree(message);
	}
	mutex_unlock(&table->lock);

	mutex_lock(&table->lock);
	while (!list_empty(&session->tx_messages)) {
		mutex_unlock(&table->lock);
		set_current_state(TASK_INTERRUPTIBLE);
		sm_debug("drain tx list\n");
		schedule_timeout(HZ * 2);
		sm_debug("drain tx list1\n");
		set_current_state(TASK_RUNNING);

		mutex_lock(&table->lock);

		if (signal_pending(current)) {
			sm_debug("signal\n");
			while (!list_empty(&session->tx_messages)) {
				message = list_first_entry(&session->tx_messages,
							   struct sm_message, next);
				list_del(&message->next);
				kfree(message);
			}
			mutex_unlock(&table->lock);
			return -ERESTARTSYS;
		}

		if (!list_empty(&session->tx_messages)) {
			message = list_first_entry(&session->tx_messages,
						   struct sm_message, next);
			list_del(&message->next);
			kfree(message);
			sm_debug("drop tx message dsp %x src %x type %x\n",
				 message->msg.dst_ep, message->msg.src_ep,
				 message->msg.type);
		}
	}
	mutex_unlock(&table->lock);

	if (session->flags == SM_CONNECT) {
		sm_send_close(session, session->remote_ep, 1);
		sm_debug("send close\n");
	}

	sm_free_session(session_idx, table);
	return 0;
}

static int
icc_open(struct inode *inode, struct file *file)
{
	int i = 0, ret = 0;
	struct miscdevice *miscdev = file->private_data;
	struct adi_icc *icc = container_of(miscdev, struct adi_icc, mdev);
	struct sm_session_table *table = icc->sessions_table;

	/* Initialize the icc message queue and the queue attribute */
	for (i = 0; i < icc->peer_count; i++) {
		memset(icc->icc_info[i].icc_high_queue, 0, MSGQ_SIZE);
		memset(icc->icc_info[i].icc_queue_attribute, 0,
		       sizeof(struct sm_message_queue));
	}

	table->refcnt++;
	file->private_data = icc;

	return ret;
}

static int
icc_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct sm_session_table *table = adi_icc->sessions_table;
	int i;
	pid_t pid = current->pid;

	table->refcnt--;
	for (i = 0; i < MAX_ENDPOINTS; i++) {
		if (table->sessions[i].pid == pid)
			sm_free_session(i, table);
	}

	return ret;
}

int icc_get_node_status(void *user_param, uint32_t size)
{
	int ret = 0;
	struct sm_node_status *param = kzalloc(sizeof(*param), GFP_KERNEL);
	struct sm_session_table *table = adi_icc->sessions_table;

	if (!param)
		return -ENOMEM;
	param->session_mask = table->session_mask;
	param->session_pending = table->session_pending;
	param->nfree = table->nfree;
	if (copy_to_user(user_param, (void *)param, size))
		ret = -EFAULT;

	kfree(param);
	return ret;
}

int icc_get_session_status(void *user_param, uint32_t size, uint32_t session_idx)
{
	int ret = 0;
	struct sm_session_status *param;
	struct sm_session *session = sm_index_to_session(session_idx);

	if (!session)
		return -EINVAL;

	param = kzalloc(sizeof(*param), GFP_KERNEL);
	if (!param)
		return -ENOMEM;

	sm_debug("session status index %d, avail %d, n_uncompleted:%d\n",
		 session_idx, (int)session->n_avail, session->n_uncompleted);

	param->n_avail = session->n_avail;
	param->n_uncompleted = session->n_uncompleted;
	param->local_ep = session->local_ep;
	param->remote_ep = session->remote_ep;
	param->type = session->type;
	param->pid = session->pid;
	param->flags = session->flags;

	if (copy_to_user(user_param, (void *)param, size))
		ret = -EFAULT;
	kfree(param);
	return ret;
}

int icc_handle_scalar_cmd(struct sm_msg *msg)
{
	u32 scalar0, scalar1;
	u16 src_cpu;
	struct sm_session *session;
	struct sm_icc_desc *icc_info;
	int index;

	if (msg->type != SM_SCALAR_READY_64)
		return 0;

	scalar0 = msg->payload;
	scalar1 = msg->length;

	icc_info = get_icc_peer(msg);
	WARN_ON(!icc_info);
	src_cpu = icc_info->peer_cpu;

	if (SM_SCALAR_CMD(scalar0) != SM_SCALAR_CMD_HEAD)
		return 0;

	switch (SM_SCALAR_CMDARG(scalar0)) {
	case SM_SCALAR_CMD_GET_SESSION_ID:
		index = sm_find_session(scalar1, 0, adi_icc->sessions_table);
		session = sm_index_to_session(index);
		if (session) {
			scalar0 = MK_SM_SCALAR_CMD_ACK(SM_SCALAR_CMD_GET_SESSION_ID);
			scalar1 = index;
			sm_debug("found scalar0 %x scalar1 %x\n", scalar0, scalar1);
			sm_send_scalar_cmd(NULL, msg->src_ep, src_cpu, scalar0,
					   scalar1);
		}
		break;
	case SM_SCALAR_CMD_GET_SESSION_TYPE:
		break;
	default:
		return 0;
	}

	return 1;
}

static long
icc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int nonblock;
	u32 local_ep;
	u32 remote_ep;
	u32 dst_cpu;
	u32 src_cpu;
	u32 len;
	u32 type;
	u32 session_idx;
	void *buf;
	u32 paddr;
	int timeout;
	struct sm_packet *pkt = kzalloc(sizeof(*pkt), GFP_KERNEL);

	if (!pkt)
		return -ENOMEM;
	if (copy_from_user(pkt, (void *)arg, sizeof(struct sm_packet)))
		return -EFAULT;

	session_idx = pkt->session_idx;
	local_ep = pkt->local_ep;
	remote_ep = pkt->remote_ep;
	type = pkt->type;
	dst_cpu = pkt->dst_cpu;
	src_cpu = arm_core_id();
	len = pkt->buf_len;
	buf = pkt->buf;
	timeout = pkt->timeout;
	nonblock = (file->f_flags & O_NONBLOCK) | (pkt->flag & O_NONBLOCK);

	switch (cmd) {
	case CMD_SM_SEND:
		if ((SM_MSG_PROTOCOL(type) == SP_SCALAR) ||
		    (SM_MSG_PROTOCOL(type) == SP_SESSION_SCALAR))
			ret = sm_send_scalar(session_idx, remote_ep, dst_cpu,
					     (uint32_t)pkt->buf, pkt->buf_len, type, nonblock);
		else
			ret = sm_send_packet(session_idx, remote_ep,
					     dst_cpu, buf, len, &pkt->payload, nonblock);
		break;
	case CMD_SM_RECV:
		if ((SM_MSG_PROTOCOL(type) == SP_SCALAR) ||
		    (SM_MSG_PROTOCOL(type) == SP_SESSION_SCALAR))
			ret = sm_recv_scalar(session_idx, &pkt->remote_ep,
					     &pkt->dst_cpu, (uint32_t *)&pkt->buf,
					     &pkt->buf_len, &pkt->type,
					     nonblock);
		else
			ret = sm_recv_packet(session_idx, &pkt->remote_ep,
					     &pkt->dst_cpu, buf, &pkt->buf_len, nonblock);
		break;
	case CMD_SM_CREATE:
		ret = sm_create_session(local_ep, type, pkt->queue_priority);
		if (ret < 0) {
			sm_debug("create session failed srcep %d\n", (int)local_ep);
			ret = -EINVAL;
		}
		pkt->session_idx = ret;
		break;
	case CMD_SM_CONNECT:
		ret = sm_connect_session(session_idx, remote_ep, dst_cpu, type);
		break;
	case CMD_SM_OPEN:
		ret = sm_open_session(session_idx);
		break;
	case CMD_SM_CLOSE:
		ret = sm_close_session(session_idx);
		break;
	case CMD_SM_ACTIVE:
		ret = sm_get_remote_session_active(session_idx, remote_ep, dst_cpu);
		break;
	case CMD_SM_SHUTDOWN:
		ret = sm_destroy_session(session_idx);
		break;
	case CMD_SM_GET_NODE_STATUS:
		ret = icc_get_node_status(pkt->param, pkt->param_len);
		break;
	case CMD_SM_GET_SESSION_STATUS:
		ret = icc_get_session_status(pkt->param, pkt->param_len, session_idx);
		break;
	case CMD_SM_REQUEST_UNCACHED_BUF:
		buf = dma_alloc_coherent(adi_icc->dev, pkt->buf_len, &paddr, GFP_KERNEL);
		if (!buf)
			ret = -ENOMEM;
		pkt->buf = buf;
		pkt->paddr = paddr;
		break;
	case CMD_SM_RELEASE_UNCACHED_BUF:
		dma_free_coherent(adi_icc->dev, pkt->buf_len, pkt->buf, pkt->paddr);
		break;
	case CMD_SM_QUERY_REMOTE_EP:
		ret = sm_query_remote_ep(remote_ep, dst_cpu, timeout, nonblock);
		break;
	case CMD_SM_WAIT:
		ret = sm_wait_request_finish(session_idx, buf, dst_cpu,
					     timeout, type, &pkt->buf_len, pkt->payload, nonblock);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (copy_to_user((void *)arg, pkt, sizeof(struct sm_packet)))
		ret = -EFAULT;
	kfree(pkt);
	return ret;
}

unsigned int icc_poll(struct file *file, poll_table *wait)
{
	struct sm_icc_desc *icc_info;
	unsigned int mask = 0, i;
	int pending;

	poll_wait(file, &adi_icc->icc_rx_wait, wait);

	for (i = 0, icc_info = adi_icc->icc_info; i < adi_icc->peer_count; i++, icc_info++) {
		pending = iccqueue_getpending(icc_info);
		if (pending) {
			mask |= POLLIN | POLLRDNORM;
			break;
		}
	}

	return mask;
}

#ifdef CONFIG_ICC_RES
int res_manage_request_peri(struct resources_t *data)
{
	unsigned short *peri_list = (unsigned short *)data->resources_array;
	char resource_name[32] = "coreb-";

	strcat(resource_name, data->label);

#ifdef CONFIG_PINCTRL
	if (pinmux_request_list(peri_list, resource_name))
#else
	if (peripheral_request_list(peri_list, resource_name))
#endif
		sm_debug("Requesting Peripherals %s failed\n", resource_name);

	return 0;
}

void res_manage_free_peri(struct resources_t *data)
{
	unsigned short *peri_list = (unsigned short *)data->resources_array;

#ifdef CONFIG_PINCTRL
	pinmux_free_list(peri_list);
#else
	peripheral_free_list(peri_list);
#endif
}

int res_manage_request_gpio(uint16_t subid)
{
	return gpio_request(subid, "COREB");
}

void res_manage_free_gpio(uint16_t subid)
{
	gpio_free(subid);
}

int res_manage_request_irq(u16 subid, unsigned int cpu)
{
	return platform_res_manage_request_irq(subid, cpu);
}

void res_manage_free_irq(uint16_t subid)
{
	return res_manage_free_irq(subid);
}

int res_manage_request_dma(uint16_t subid)
{
	return request_dma(subid, "COREB");
}

void res_manage_free_dma(uint16_t subid)
{
	free_dma(subid);
}

int res_manage_request(u16 id, struct resources_t *data,
		       struct sm_icc_desc *icc_info)
{
	int ret = 0;
	u16 type, subid;

	type = RESMGR_TYPE(id);
	subid = RESMGR_SUBID(id);
	sm_debug("%s %x %x\n", __func__, type, subid);
	switch (type) {
	case RESMGR_TYPE_PERIPHERAL:
		ret = res_manage_request_peri(data);
		break;
	case RESMGR_TYPE_GPIO:
		ret = res_manage_request_gpio(subid);
		break;
	case RESMGR_TYPE_SYS_IRQ:
		ret = res_manage_request_irq(subid, icc_info->peer_cpu);
		break;
	case RESMGR_TYPE_DMA:
		ret = res_manage_request_dma(subid);
		break;
	default:
		ret = -ENODEV;
	}
	return ret;
}

int res_manage_free(u16 id, struct resources_t *data)
{
	int ret = 0;
	u16 type, subid;

	type = RESMGR_TYPE(id);
	subid = RESMGR_SUBID(id);
	switch (type) {
	case RESMGR_TYPE_PERIPHERAL:
		res_manage_free_peri(data);
		break;
	case RESMGR_TYPE_GPIO:
		res_manage_free_gpio(subid);
		break;
	case RESMGR_TYPE_SYS_IRQ:
		res_manage_free_irq(subid);
		break;
	case RESMGR_TYPE_DMA:
		res_manage_free_dma(subid);
		break;
	default:
		ret = -ENODEV;
	}
	return ret;
}
#endif

static const struct file_operations icc_fops = {
	.owner          = THIS_MODULE,
	.open		= icc_open,
	.release	= icc_release,
	.unlocked_ioctl = icc_ioctl,
	.poll		= icc_poll,
};

static int msg_recv_internal(struct sm_msg *msg, struct sm_session *session)
{
	struct sm_icc_desc *icc_info;
	int cpu = arm_core_id();
	struct sm_message *message;
	int ret = 0;

	icc_info = get_icc_peer(msg);
	WARN_ON(!icc_info);

	message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;

	memcpy(&message->msg, msg, sizeof(struct sm_msg));

	message->dst = cpu;
	message->src = icc_info->peer_cpu;

	if (SM_MSG_PROTOCOL(msg->type) == SP_SCALAR)
		sm_send_scalar_ack(session, msg->src_ep, message->src,
				   msg->payload, msg->length);
	else if (SM_MSG_PROTOCOL(msg->type) == SP_SESSION_SCALAR)
		sm_send_session_scalar_ack(session, msg->src_ep, message->src,
					   msg->payload, msg->length);

	if (session->handle) {
		session->handle(message, session);
		return 0;
	}
	mutex_lock(&adi_icc->sessions_table->lock);
	list_add_tail(&message->next, &session->rx_messages);
	session->n_avail++;
	session->b_rx_finish = 1;
	sm_debug("%s wakeup wait thread avail %d\n", __func__, (int)session->n_avail);
	wake_up(&session->rx_wait);
	mutex_unlock(&adi_icc->sessions_table->lock);
	return ret;
}

static int sm_default_sendmsg(struct sm_message *message, struct sm_session *session)
{
	struct sm_msg *msg = &message->msg;
	struct sm_message *m = message;

	sm_debug("%s session type %x\n", __func__, (uint32_t)session->type);
	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__,
		 (int)m->msg.length, (uint32_t)m->msg.type, (int)m->dst,
		 (int)m->msg.dst_ep, (int)m->src, (int)m->msg.src_ep);

	switch (session->type) {
	case SP_PACKET:
	case SP_SESSION_PACKET:
	case SP_SCALAR:
	case SP_SESSION_SCALAR:
		mutex_lock(&adi_icc->sessions_table->lock);
		list_add_tail(&message->next, &session->tx_messages);
		session->n_uncompleted++;
		message->b_tx_finish = 0;
		mutex_unlock(&adi_icc->sessions_table->lock);
		break;
	case SM_PACKET_ERROR:
		sm_debug("SM ERROR %08x\n", (uint32_t)msg->payload);
		break;
	default:
		break;
	};
	return 0;
}

static int
sm_default_recvmsg(struct sm_msg *msg, struct sm_session *session)
{
	struct sm_icc_desc *icc_info;
	int ret = 0;
	struct sm_message *uncompleted;
	struct sm_session_table *table = adi_icc->sessions_table;

	icc_info = get_icc_peer(msg);
	WARN_ON(!icc_info);
	sm_debug("%s msg type %x\n", __func__, (uint32_t)msg->type);
	switch (msg->type) {
	case SM_PACKET_CONSUMED:
	case SM_SESSION_PACKET_CONSUMED:
		uncompleted =
			get_msg_from_tx_list(session, msg->payload);
		if (!uncompleted)
			return -EINVAL;
		mutex_lock(&table->lock);
		uncompleted->b_tx_finish = 1;
		session->n_uncompleted--;
		sm_debug("%s, packet consumed, wake up the tx wait\n", __func__);
		wake_up(&uncompleted->icc_info->iccq_tx_wait);
		mutex_unlock(&table->lock);
		break;
	case SM_SCALAR_CONSUMED:
	case SM_SESSION_SCALAR_CONSUMED:
		uncompleted =
			get_msg_from_tx_list(session, msg->payload);
		if (!uncompleted)
			return -EINVAL;
		mutex_lock(&table->lock);
		uncompleted->b_tx_finish = 1;
		session->n_uncompleted--;
		sm_debug("%s, scalar consumed, wake up the tx wait\n", __func__);
		wake_up(&uncompleted->icc_info->iccq_tx_wait);
		mutex_unlock(&table->lock);
		break;
	case SM_SESSION_PACKET_CONNECT_ACK:
	case SM_SESSION_SCALAR_CONNECT_ACK:
		sm_debug("%s wakeup wait thread\n", __func__);
		session->remote_ep = msg->src_ep;
		session->flags = SM_CONNECT;
		wake_up(&session->rx_wait);
		break;
	case SM_SESSION_PACKET_CONNECT:
	case SM_SESSION_SCALAR_CONNECT:
		session->remote_ep = msg->src_ep;
		session->flags = SM_CONNECTING;
		session->type = SM_MSG_PROTOCOL(msg->type);
		sm_send_connect_ack(session, msg->src_ep, icc_info->peer_cpu);
		break;
	case SM_SESSION_PACKET_CONNECT_DONE:
	case SM_SESSION_SCALAR_CONNECT_DONE:
		sm_debug("%s connect done\n", __func__);
		session->flags = SM_CONNECT;
		break;
	case SM_SESSION_PACKET_ACTIVE:
		if (session->flags & SM_OPEN)
			sm_send_session_active_ack(session, msg->src_ep, icc_info->peer_cpu);
		else
			sm_send_session_active_noack(session, msg->src_ep, icc_info->peer_cpu);
		break;
	case SM_SESSION_PACKET_ACTIVE_ACK:
		if (session->flags & SM_OPEN) {
			if (msg->payload == SM_OPEN) {
				session->flags |= SM_ACTIVE;
				wake_up(&session->rx_wait);
			}
		}
		break;
	case SM_SESSION_PACKET_CLOSE:
	case SM_SESSION_SCALAR_CLOSE:
		session->remote_ep = 0;
		session->flags = 0;
		sm_send_close_ack(session, msg->src_ep, icc_info->peer_cpu);
		break;
	case SM_SESSION_PACKET_CLOSE_ACK:
	case SM_SESSION_SCALAR_CLOSE_ACK:
		session->remote_ep = 0;
		session->flags = 0;
		wake_up(&session->rx_wait);
		break;
	case SM_PACKET_READY:
	case SM_SESSION_PACKET_READY:
		if (SM_MSG_PROTOCOL(msg->type) != session->type) {
			sm_debug("msg type %08x unmatch session type %08x\n",
				 (uint32_t)msg->type, (uint32_t)session->type);
			break;
		}
		msg_recv_internal(msg, session);
		break;
	case SM_SCALAR_READY_8:
	case SM_SCALAR_READY_16:
	case SM_SCALAR_READY_32:
	case SM_SCALAR_READY_64:
	case SM_SESSION_SCALAR_READY_8:
	case SM_SESSION_SCALAR_READY_16:
	case SM_SESSION_SCALAR_READY_32:
	case SM_SESSION_SCALAR_READY_64:
		msg_recv_internal(msg, session);
		break;
	case SM_PACKET_ERROR:
		sm_debug("SM ERROR %08x\n", (uint32_t)msg->payload);
		break;
	default:
		ret = -EINVAL;
	};

	sm_message_dequeue(msg);

	return ret;
}

static int sm_default_shutdown(struct sm_session *session)
{
	return 0;
}

static int sm_default_error(struct sm_msg *msg, struct sm_session *session)
{
	return 0;
}

#ifdef CONFIG_ICC_TASK
static int sm_task_sendmsg(struct sm_message *message, struct sm_session *session)
{
	struct sm_msg *msg = &message->msg;
	struct sm_task *task;

	if (msg->length >= sizeof(struct sm_task))
		msg->type = SM_TASK_RUN;
	else
		msg->type = SM_TASK_KILL;
	sm_debug("%s msg type %x\n", __func__, (uint32_t)msg->type);
	switch (msg->type) {
	case SM_TASK_RUN:
		flush_dcache_range(_ramend, physical_mem_end);
		task = (struct sm_task *)msg->payload;

		flush_dcache_range(msg->payload, msg->payload + msg->length);
		sm_debug("%s init addr%p\n", __func__, task->task_init);
		mutex_lock(&adi_icc->sessions_table->lock);
		list_add_tail(&message->next, &session->tx_messages);
		session->n_uncompleted++;
		mutex_unlock(&adi_icc->sessions_table->lock);
		break;
	case SM_TASK_KILL:
		break;
	default:
		break;
	};
	return 0;
}

static int sm_task_recvmsg(struct sm_msg *msg, struct sm_session *session)
{
	struct sm_message *message;
	struct sm_icc_desc *icc_info;

	icc_info = get_icc_peer(msg);
	WARN_ON(!icc_info);
	sm_debug("%s msg type %x\n", __func__, (uint32_t)msg->type);
	if (SM_MSG_PROTOCOL(msg->type) != session->type) {
		sm_debug("msg type %08x unmatch session type %08x\n",
			 (uint32_t)msg->type, (uint32_t)session->type);
		return 0;
	}
	switch (msg->type) {
	case SM_TASK_RUN_ACK:
		sm_debug("%s free %x\n", __func__, (uint32_t)msg->payload);
		message = list_first_entry(&session->tx_messages,
					   struct sm_message, next);
		list_del(&message->next);

		kfree((void *)msg->payload);
		wake_up(&icc_info->iccq_tx_wait);
		msg_recv_internal(msg, session);
		break;
	case SM_TASK_KILL_ACK:
		msg_recv_internal(msg, session);
		break;
	default:
		break;
	};
	sm_message_dequeue(msg);
	return 0;
}

static int sm_resouce_manage_recvmsg(struct sm_msg *msg, struct sm_session *session)
{
	int ret;
	struct sm_icc_desc *icc_info = get_icc_peer(msg);

	WARN_ON(!icc_info);

	switch (msg->type) {
	case SM_RES_MGR_REQUEST:
		ret = res_manage_request((uint16_t)msg->payload,
					 (struct resources_t *)msg->length, icc_info);
		if (ret)
			sm_send_control_msg(session, msg->src_ep, icc_info->peer_cpu, 0,
					    0, SM_RES_MGR_REQUEST_FAIL);
		else
			sm_send_control_msg(session, msg->src_ep, icc_info->peer_cpu, 0,
					    0, SM_RES_MGR_REQUEST_OK);

		break;
	case SM_RES_MGR_FREE:
		res_manage_free((uint16_t)msg->payload, (struct resources_t *)msg->length);
		sm_send_control_msg(session, msg->src_ep, icc_info->peer_cpu, 0,
				    0, SM_RES_MGR_FREE_DONE);
		break;
	default:
		break;
	};
	sm_message_dequeue(msg);
	return 0;
}
#endif

struct sm_proto core_control_proto = {
	.sendmsg = NULL,
	.recvmsg = NULL,
	.shutdown = NULL,
	.error = NULL,
};

#ifdef CONFIG_ICC_TASK
struct sm_proto task_manager_proto = {
	.sendmsg = sm_task_sendmsg,
	.recvmsg = sm_task_recvmsg,
	.shutdown = NULL,
	.error = NULL,
};
#endif

#ifdef CONFIG_ICC_RES
struct sm_proto res_manager_proto = {
	.sendmsg = NULL,
	.recvmsg = sm_resouce_manage_recvmsg,
	.shutdown = NULL,
	.error = NULL,
};
#endif

struct sm_proto packet_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

struct sm_proto session_packet_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

struct sm_proto scalar_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

struct sm_proto session_scalar_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

int __handle_general_msg(struct sm_msg *msg)
{
	int index;
	struct sm_session *session;
	struct sm_icc_desc *icc_info;
	struct sm_message *message;
	int ret = 0;

	switch (msg->type) {
	case SM_BAD_MSG:
		sm_debug("%s", (char *)IO_ADDRESS(msg->payload));
		ret = 1;
		break;
	case SM_QUERY_MSG:
		icc_info = get_icc_peer(msg);
		WARN_ON(!icc_info);
		index = sm_find_session(msg->dst_ep, 0, adi_icc->sessions_table);
		session = sm_index_to_session(index);
		if (session) {
			sm_send_control_msg(session, msg->src_ep,
					    icc_info->peer_cpu, 0, 0, SM_QUERY_ACK_MSG);
		} else {
			message = kzalloc(sizeof(*message), GFP_KERNEL);
			if (message) {
				memcpy(&message->msg, msg, sizeof(struct sm_msg));
				message->src = icc_info->peer_cpu;
				mutex_lock(&adi_icc->sessions_table->lock);
				list_add_tail(&message->next,
					      &adi_icc->sessions_table->query_message);
				mutex_unlock(&adi_icc->sessions_table->lock);
			}
		}
		ret = 1;
		break;
	case SM_QUERY_ACK_MSG:
		mutex_lock(&adi_icc->sessions_table->lock);
		adi_icc->sessions_table->b_query_finish = 1;
		mutex_unlock(&adi_icc->sessions_table->lock);
		wake_up_interruptible(&adi_icc->sessions_table->query_wait);
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

int __msg_handle(struct sm_icc_desc *icc_info, struct sm_message_queue *inqueue)
{
	u16 received = sm_atomic_read(&inqueue->received);
	u16 sent = sm_atomic_read(&inqueue->sent);
	u16 pending;
	struct sm_msg *msg;
	struct sm_session *session;
	int index;

	pending = sent - received;
	if (pending < 0)
		pending += USHRT_MAX;
	pending %= SM_MSGQ_LEN;
	if (pending == 0)
		return pending;

	msg = &inqueue->messages[(received % SM_MSGQ_LEN)];

	if (__handle_general_msg(msg)) {
		sm_message_dequeue(msg);
		return 1;
	}

	index = sm_find_session(msg->dst_ep, 0, adi_icc->sessions_table);
	session = sm_index_to_session(index);

	if (!session) {
		sm_debug("discard msg type %x dst %x src %x\n",
			 (uint32_t)msg->type, msg->dst_ep, msg->src_ep);
		sm_message_dequeue(msg);
		wake_up(&icc_info->iccq_tx_wait);
		return 1;
	}

	sm_debug("session %p index %d msg type%x\n", session, index, (uint32_t)msg->type);

	if (session->proto_ops->recvmsg)
		session->proto_ops->recvmsg(msg, session);
	else
		sm_debug("session type not supported\n");

	return 1;
}

void msg_handle(struct sm_icc_desc *icc_info)
{
	int ret;

	/* icc_queue[0] is the queue to receive message */
	ret = __msg_handle(icc_info, icc_info->icc_high_queue);
	if (!ret)
		__msg_handle(icc_info, icc_info->icc_queue);
}

static irqreturn_t message_queue_thread(int irq, void *dev_instance)
{
	struct adi_icc *icc = (struct adi_icc *)dev_instance;
	struct sm_icc_desc *icc_info;
	int i;

	for (i = 0, icc_info = icc->icc_info;
	     i < icc->peer_count; i++, icc_info++) {
		while (iccqueue_getpending(icc_info))
			msg_handle(icc_info);
	}

	return IRQ_HANDLED;
}

void register_sm_proto(struct adi_icc *icc)
{
	icc->sm_protos[SP_CORE_CONTROL] = &core_control_proto;
	//icc->sm_protos[SP_TASK_MANAGER] = &task_manager_proto;
	//icc->sm_protos[SP_RES_MANAGER] = &res_manager_proto;
	icc->sm_protos[SP_PACKET] = &packet_proto;
	icc->sm_protos[SP_SESSION_PACKET] = &session_packet_proto;
	icc->sm_protos[SP_SCALAR] = &scalar_proto;
	icc->sm_protos[SP_SESSION_SCALAR] = &session_scalar_proto;
}

static int
icc_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	char line[256];
	unsigned long val;
	int ret;
	struct sm_session *session;
	struct sm_session_table *table = adi_icc->sessions_table;

	ret = copy_from_user(line, buffer, count);
	if (ret)
		return -EFAULT;

	line[count - 1] = '\0';

	if (kstrtoul(line, 10, &val))
		return -EINVAL;

	sm_debug("session index %ld\n", val);
	if (val < 0 && val >= MAX_SESSIONS)
		return -EINVAL;

	session = &table->sessions[val];
	sm_debug(" %x", session->local_ep);
	sm_debug(" %x", session->remote_ep);
	sm_debug(" %X", (uint32_t)session->type);
	sm_debug(" %X\n", (uint32_t)session->flags);
	return count;
}

static irqreturn_t ipi_handler_int0(int irq, void *dev_instance)
{
	struct adi_icc *icc = (struct adi_icc *)dev_instance;
	struct sm_icc_desc *icc_info;
	int i;
	unsigned int cpu = arm_core_id();

	icc_clear_ipi_cpu(cpu, irq);
	for (i = 0, icc_info = icc->icc_info; i < icc->peer_count;
			i++, icc_info++)
		wakeup_icc_thread(icc_info);

	return IRQ_WAKE_THREAD;
}

static const struct proc_ops icc_proc_fops = {
	.proc_write = icc_write_proc,
	.proc_lseek = default_llseek,
};

static int adi_icc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct icc_platform_data *icc_data;
	struct adi_icc *icc = NULL;
	struct sm_icc_desc *icc_info;
	int ret, i;
	struct device_node *np;
	struct resource *res;

	if (adi_icc) {
		dev_err(&pdev->dev, "Can't register more than one adi_icc device.\n");
		return -ENOENT;
	}

	icc = devm_kzalloc(dev, sizeof(*icc), GFP_KERNEL);
	if (!icc)
		return -ENOMEM;

	icc->dev = dev;
	icc->mdev.minor	= MISC_DYNAMIC_MINOR;
	snprintf(icc->name, 20, "%s", DRIVER_NAME);
	icc->mdev.name	= icc->name;
	icc->mdev.fops	= &icc_fops;
	init_waitqueue_head(&icc->icc_rx_wait);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}

	icc->base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR((void *)icc->base)) {
		dev_err(dev, "Cannot map ICC memory\n");
		return PTR_ERR((void *)icc->base);
	}

	icc->irq = platform_get_irq(pdev, 0);
	if (icc->irq <= 0) {
		dev_err(dev, "No ICC IRQ specified\n");
		return -ENOENT;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	icc->irq_type = (res->flags & IORESOURCE_BITS) | IRQF_PERCPU;

	np = pdev->dev.of_node;
	if (np) {
		const __be32 *peer_info;
		u32 intlen;
#define PEERINFO_CELLS	2

		icc->node = 0;

		peer_info = of_get_property(np, "peerinfo", &intlen);
		if (!peer_info) {
			dev_err(dev, "Invalid peerinfo node.\n");
			return -EINVAL;
		}

		intlen /= sizeof(*peer_info);

		if (icc->irq <= 0) {
			dev_err(dev, "Invalid ICC IRQ.\n");
			return -ENOENT;
		}

		icc->peer_count = intlen / PEERINFO_CELLS;
		icc->icc_info = devm_kzalloc(dev, sizeof(struct sm_icc_desc) *
				icc->peer_count, GFP_KERNEL);
		if (!icc->icc_info)
			return -ENOMEM;

		for (i = 0, icc_info = icc->icc_info; i < icc->peer_count;
				i++, icc_info++) {
			icc_info->peer_cpu = be32_to_cpup(peer_info +
						i * PEERINFO_CELLS);
			if (icc_info->peer_cpu <= 0 || icc_info->peer_cpu > 2) {
				dev_err(dev, "Invalid ICC Peer CPU ID\n");
				return -ENOENT;
			}

			icc_info->notify = be32_to_cpup(peer_info +
						i * PEERINFO_CELLS + 1);
			if (icc_info->notify < 0) {
				dev_err(dev, "Invalid ICC Notify Pin\n");
				return -ENOENT;
			}
		}
	} else {
		icc_data = (struct icc_platform_data *)pdev->dev.platform_data;
		icc->node = icc_data->node;
		icc->peer_count = icc_data->peer_count;

		if (!icc_data || icc_data->peer_count <= 0) {
			dev_err(dev, "No ICC platform data are defined.\n");
			return -ENOENT;
		}

		icc->icc_info = devm_kzalloc(dev, sizeof(struct sm_icc_desc) *
				icc->peer_count, GFP_KERNEL);
		if (!icc->icc_info)
			return -ENOMEM;

		for (i = 0, icc_info = icc->icc_info; i < icc->peer_count;
				i++, icc_info++) {
			icc_info->peer_cpu = icc_data->peer_info[i].peerid;
			icc_info->notify = icc_data->peer_info[i].notify;
			if (icc_info->notify < 0) {
				dev_err(dev, "No ICC Notify specified\n");
				return -ENOENT;
			}
		}
	}

	ret = devm_request_threaded_irq(dev, icc->irq, ipi_handler_int0,
					message_queue_thread, icc->irq_type,
					"ICC receive IRQ", icc);
	if (ret) {
		dev_err(dev, "Fail to request ICC receive IRQ\n");
		return -ENOENT;
	}

	icc->adi_tru = get_adi_tru_from_node(dev);
	if (IS_ERR(icc->adi_tru)) {
		ret = PTR_ERR(icc->adi_tru);
		return -ENODEV;
	}

	for (i = 0, icc_info = icc->icc_info; i < icc->peer_count;
				i++, icc_info++) {
		/* icc_queue[0] is rx queue, icc_queue[1] is tx queue. */
		icc_info->icc_high_queue = (struct sm_message_queue *)
						(icc->base + i * 2 * MSGQ_SIZE);
		icc_info->icc_queue = (struct sm_message_queue *)
						icc_info->icc_high_queue + 2;
		memset(icc_info->icc_high_queue, 0, MSGQ_SIZE);

		icc_info->icc_queue_attribute = (uint32_t *)((uint32_t)
			icc->base + 3 * MSGQ_SIZE + i * 2 * sizeof(struct sm_message_queue));
		memset(icc_info->icc_queue_attribute, 0, sizeof(struct sm_message_queue));

		init_waitqueue_head(&icc_info->iccq_tx_wait);
	}

	ret = misc_register(&icc->mdev);
	if (ret) {
		dev_err(&pdev->dev, "cannot register ICC miscdev\n");
		return ret;
	}

	init_sm_session_table(icc);

	register_sm_proto(icc);

	icc->icc_dump = proc_create("icc_dump", 0644, NULL, &icc_proc_fops);

	platform_set_drvdata(pdev, icc);
	adi_icc = icc;

/*
 *	ret = sm_create_session(EP_RESMGR_SERVICE, SP_RES_MANAGER, 0);
 *	WARN_ON(ret < 0);
 */

	dev_info(&pdev->dev, "initialized\n");

	return 0;
}

static int adi_icc_remove(struct platform_device *pdev)
{
	struct adi_icc *icc = platform_get_drvdata(pdev);
	struct sm_icc_desc *icc_info;
	int i;

	adi_icc = NULL;

	if (icc) {
		misc_deregister(&icc->mdev);
		for (i = 0, icc_info = icc->icc_info; i < icc->peer_count; i++, icc_info++) {
			if (icc_info->irq)
				free_irq(icc_info->irq, icc_info);
		}
		kfree(icc->icc_info);
		kfree(icc);
	}

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id adi_icc_dt_ids[] = {
	{ .compatible = "adi,icc" },
	{},
};

MODULE_DEVICE_TABLE(of, adi_icc_dt_ids);
#endif

static struct platform_driver adi_icc_driver = {
	.probe     = adi_icc_probe,
	.remove    = adi_icc_remove,
	.driver    = {
		.name  = DRIVER_NAME,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(adi_icc_dt_ids),
#endif
	},
};

module_platform_driver(adi_icc_driver);

MODULE_AUTHOR("Steven Miao <steven.miao@analog.com>");
MODULE_DESCRIPTION("ADI ICC driver");
MODULE_LICENSE("GPL");

