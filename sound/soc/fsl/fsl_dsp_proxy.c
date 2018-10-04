// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
//
// DSP proxy driver transfers messages between DSP driver and DSP framework
//
// Copyright 2018 NXP
// Copyright (C) 2017 Cadence Design Systems, Inc.

#include <soc/imx8/sc/ipc.h>
#include "fsl_dsp_proxy.h"
#include "fsl_dsp.h"

/* ...initialize message queue */
void xf_msg_queue_init(struct xf_msg_queue *queue)
{
	queue->head = queue->tail = NULL;
}

/* ...get message queue head */
struct xf_message *xf_msg_queue_head(struct xf_msg_queue *queue)
{
	return queue->head;
}

/* ...allocate new message from the pool */
struct xf_message *xf_msg_alloc(struct xf_proxy *proxy)
{
	struct xf_message *m = proxy->free;

	/* ...make sure we have a free message item */
	if (m != NULL) {
		/* ...get message from the pool */
		proxy->free = m->next, m->next = NULL;
	}

	return m;
}

/* ...return message to the pool of free items */
void xf_msg_free(struct xf_proxy *proxy, struct xf_message *m)
{
	/* ...put message into the head of free items list */
	m->next = proxy->free, proxy->free = m;

	/* ...notify potential client waiting for message */
	wake_up(&proxy->busy);
}

/* ...return all messages from the queue to the pool of free items */
void xf_msg_free_all(struct xf_proxy *proxy, struct xf_msg_queue *queue)
{
	struct xf_message *m = queue->head;

	/* ...check if there is anything in the queue */
	if (m != NULL) {
		queue->tail->next = proxy->free;
		proxy->free = queue->head;
		queue->head = queue->tail = NULL;

		/* ...notify potential client waiting for message */
		wake_up(&proxy->busy);
	}
}

/* ...submit message to a queue */
int xf_msg_enqueue(struct xf_msg_queue *queue, struct xf_message *m)
{
	int first = (queue->head == NULL);

	/* ...set pointer to next item */
	m->next = NULL;

	/* ...advance head/tail pointer as required */
	if (first)
		queue->head = m;
	else
		queue->tail->next = m;

	/* ...new tail points to this message */
	queue->tail = m;

	return first;
}

/* ...retrieve next message from the per-task queue */
struct xf_message *xf_msg_dequeue(struct xf_msg_queue *queue)
{
	struct xf_message *m = queue->head;

	/* ...check if there is anything in the queue */
	if (m != NULL) {
		/* ...pop message from the head of the list */
		queue->head = m->next;
		if (queue->head == NULL)
			queue->tail = NULL;
	}

	return m;
}

/* ...helper function for requesting execution message from a pool */
struct xf_message *xf_msg_available(struct xf_proxy *proxy)
{
	struct xf_message *m;

	/* ...acquire global lock */
	xf_lock(&proxy->lock);

	/* ...try to allocate the message */
	m = xf_msg_alloc(proxy);
	if (m  == NULL) {
		/* ...failed to allocate message; release lock */
		xf_unlock(&proxy->lock);
	}

	/* ...if successfully allocated */
	return m;
}

/* ...helper function for receiving a message from per-client queue */
struct xf_message *xf_msg_received(struct xf_proxy *proxy,
				struct xf_msg_queue *queue)
{
	struct xf_message *m;

	/* ...acquire global lock */
	xf_lock(&proxy->lock);

	/* ...try to peek message from the queue */
	m = xf_msg_dequeue(queue);
	if (m == NULL) {
		/* ...queue is empty; release lock */
		xf_unlock(&proxy->lock);
	}

	/* ...if message is non-null, lock is held */
	return m;
}

/*
 * MU related functions
 */
u32 icm_intr_send(struct xf_proxy *proxy, u32 msg)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);

	MU_SendMessage(dsp_priv->mu_base_virtaddr, 0, msg);
	return 0;
}

int icm_intr_extended_send(struct xf_proxy *proxy,
				u32 msg,
				struct dsp_ext_msg *ext_msg)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);
	struct device *dev = dsp_priv->dev;
	union icm_header_t msghdr;

	msghdr.allbits = msg;
	if (msghdr.size != 8)
		dev_err(dev, "too much ext msg\n");

	MU_SendMessage(dsp_priv->mu_base_virtaddr, 1, ext_msg->phys);
	MU_SendMessage(dsp_priv->mu_base_virtaddr, 2, ext_msg->size);
	MU_SendMessage(dsp_priv->mu_base_virtaddr, 0, msg);

	return 0;
}

int send_dpu_ext_msg_addr(struct xf_proxy *proxy)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);
	union icm_header_t msghdr;
	struct dsp_ext_msg ext_msg;
	struct dsp_mem_msg *dpu_ext_msg =
	 (struct dsp_mem_msg *)((unsigned char *)dsp_priv->msg_buf_virt
					+ (MSG_BUF_SIZE / 2));
	int ret_val = 0;

	msghdr.allbits = 0;	/* clear all bits; */
	msghdr.ack  = 0;
	msghdr.intr = 1;
	msghdr.msg  = ICM_CORE_INIT;
	msghdr.size = 8;
	ext_msg.phys = dsp_priv->msg_buf_phys + (MSG_BUF_SIZE / 2);
	ext_msg.size = sizeof(struct dsp_mem_msg);

	dpu_ext_msg->ext_msg_phys = dsp_priv->msg_buf_phys;
	dpu_ext_msg->ext_msg_size = MSG_BUF_SIZE;
	dpu_ext_msg->scratch_phys =  dsp_priv->scratch_buf_phys;
	dpu_ext_msg->scratch_size =  dsp_priv->scratch_buf_size;
	dpu_ext_msg->dsp_config_phys =  dsp_priv->dsp_config_phys;
	dpu_ext_msg->dsp_config_size =  dsp_priv->dsp_config_size;

	icm_intr_extended_send(proxy, msghdr.allbits, &ext_msg);

	return ret_val;
}

long icm_ack_wait(struct xf_proxy *proxy, u32 msg)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);
	struct device *dev = dsp_priv->dev;
	union icm_header_t msghdr;
	int err;

	msghdr.allbits = msg;
	/* wait response from mu */
	err = wait_for_completion_timeout(&proxy->cmd_complete,
				msecs_to_jiffies(1000));
	if (!err) {
		dev_err(dev, "icm ack timeout! %x\n", msg);
		return -ETIMEDOUT;
	}

	dev_dbg(dev, "Ack recd for message 0x%08x\n", msghdr.allbits);

	return 0;
}

irqreturn_t fsl_dsp_mu_isr(int irq, void *dev_id)
{
	struct xf_proxy *proxy = dev_id;
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);
	struct device *dev = dsp_priv->dev;
	union icm_header_t msghdr;
	u32 reg;

	MU_ReceiveMsg(dsp_priv->mu_base_virtaddr, 0, &reg);
	msghdr = (union icm_header_t)reg;

	if (msghdr.intr == 1) {
		dev_dbg(dev, "INTR: Received ICM intr, msg 0x%08x\n",
						msghdr.allbits);
		switch (msghdr.msg) {
		case ICM_CORE_EXIT:
			break;
		case ICM_CORE_READY:
			send_dpu_ext_msg_addr(proxy);
			proxy->is_ready = 1;
			complete(&proxy->cmd_complete);
			break;
		case XF_SUSPEND:
		case XF_RESUME:
			complete(&proxy->cmd_complete);
			break;
		default:
			schedule_work(&proxy->work);
			break;
		}
	} else if (msghdr.ack == 1) {
		dev_dbg(dev, "INTR: Received ICM ack 0x%08x\n", msghdr.size);
		msghdr.ack = 0;
	} else {
		dev_dbg(dev, "Received false ICM intr 0x%08x\n",
							msghdr.allbits);
	}

	return IRQ_HANDLED;
}

/*
 * Proxy related functions
 */
/* ...NULL-address specification */
#define XF_PROXY_NULL           (~0U)

#define XF_PROXY_BADADDR        (dsp_priv->scratch_buf_size)

/* ...shared memory translation - kernel virtual address to shared address */
u32 xf_proxy_b2a(struct xf_proxy *proxy, void *b)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);

	if (b == NULL)
		return XF_PROXY_NULL;
	else if ((u32)(b - dsp_priv->scratch_buf_virt) <
					dsp_priv->scratch_buf_size)
		return (u32)(b - dsp_priv->scratch_buf_virt);
	else
		return XF_PROXY_BADADDR;
}

/* ...shared memory translation - shared address to kernel virtual address */
void *xf_proxy_a2b(struct xf_proxy *proxy, u32 address)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);

	if (address < dsp_priv->scratch_buf_size)
		return dsp_priv->scratch_buf_virt + address;
	else if (address == XF_PROXY_NULL)
		return NULL;
	else
		return (void *) -1;
}

/* ...process association between response received and intended client */
static void xf_cmap(struct xf_proxy *proxy, struct xf_message *m)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);
	u32 id = XF_AP_IPC_CLIENT(m->id);
	struct xf_client *client;

	/* ...process messages addressed to proxy itself */
	if (id == 0) {
		/* ...place message into local response queue */
		xf_msg_enqueue(&proxy->response, m);
		wake_up(&proxy->wait);
		return;
	}

	/* ...make sure the client ID is sane */
	client = xf_client_lookup(dsp_priv, id);
	if (!client) {
		pr_err("rsp[id:%08x]: client lookup failed", m->id);
		xf_msg_free(proxy, m);
		return;
	}

	/* ...make sure client is bound to this proxy interface */
	if (client->proxy != proxy) {
		pr_err("rsp[id:%08x]: wrong proxy interface", m->id);
		xf_msg_free(proxy, m);
		return;
	}

	/* ...place message into local response queue */
	if (xf_msg_enqueue(&client->queue, m))
		wake_up(&client->wait);
}

/* ...retrieve pending responses from shared memory ring-buffer */
static u32 xf_shmem_process_responses(struct xf_proxy *proxy)
{
	struct xf_message *m;
	u32 read_idx, write_idx;
	int status;

	status = 0;

	/* ...get current values of read/write pointers in response queue */
	read_idx = XF_PROXY_READ(proxy, rsp_read_idx);
	write_idx = XF_PROXY_READ(proxy, rsp_write_idx);

	/* ...process all committed responses */
	while (!XF_QUEUE_EMPTY(read_idx, write_idx)) {
		struct xf_proxy_message *response;

		/* ...allocate execution message */
		m = xf_msg_alloc(proxy);
		if (m  == NULL)
			break;

		/* ...mark the interface status has changed */
		status |= (XF_QUEUE_FULL(read_idx, write_idx) ? 0x3 : 0x1);

		/* ...get oldest not yet processed response */
		response = XF_PROXY_RESPONSE(proxy, XF_QUEUE_IDX(read_idx));

		/* ...fill message parameters */
		m->id = response->session_id;
		m->opcode = response->opcode;
		m->length = response->length;
		m->buffer = xf_proxy_a2b(proxy, response->address);
		m->ret = response->ret;

		/* ...advance local reading index copy */
		read_idx = XF_QUEUE_ADVANCE_IDX(read_idx);

		/* ...update shadow copy of reading index */
		XF_PROXY_WRITE(proxy, rsp_read_idx, read_idx);

		/* ...submit message to proper client */
		xf_cmap(proxy, m);
	}

	return status;
}

/* ...put pending commands into shared memory ring-buffer */
static u32 xf_shmem_process_commands(struct xf_proxy *proxy)
{
	struct xf_message *m;
	u32 read_idx, write_idx;
	int status = 0;

	/* ...get current value of peer read pointer */
	write_idx = XF_PROXY_READ(proxy, cmd_write_idx);
	read_idx = XF_PROXY_READ(proxy, cmd_read_idx);

	/* ...submit any pending commands */
	while (!XF_QUEUE_FULL(read_idx, write_idx)) {
		struct xf_proxy_message *command;

		/* ...check if we have a pending command */
		m = xf_msg_dequeue(&proxy->command);
		if (m  == NULL)
			break;

		/* ...always mark the interface status has changed */
		status |= 0x3;

		/* ...select the place for the command */
		command = XF_PROXY_COMMAND(proxy, XF_QUEUE_IDX(write_idx));

		/* ...put the response message fields */
		command->session_id = m->id;
		command->opcode = m->opcode;
		command->length = m->length;
		command->address = xf_proxy_b2a(proxy, m->buffer);
		command->ret = m->ret;

		/* ...return message back to the pool */
		xf_msg_free(proxy, m);

		/* ...advance local writing index copy */
		write_idx = XF_QUEUE_ADVANCE_IDX(write_idx);

		/* ...update shared copy of queue write pointer */
		XF_PROXY_WRITE(proxy, cmd_write_idx, write_idx);
	}

	if (status)
		icm_intr_send(proxy, 0);

	return status;
}

/* ...shared memory interface maintenance routine */
void xf_proxy_process(struct work_struct *w)
{
	struct xf_proxy *proxy = container_of(w, struct xf_proxy, work);
	int status = 0;

	/* ...get exclusive access to internal data */
	xf_lock(&proxy->lock);

	do {
		/* ...process outgoing commands first */
		status = xf_shmem_process_commands(proxy);

		/* ...process all pending responses */
		status |= xf_shmem_process_responses(proxy);

	} while (status);

	/* ...unlock internal proxy data */
	xf_unlock(&proxy->lock);
}

/* ...initialize shared memory interface */
int xf_proxy_init(struct xf_proxy *proxy)
{
	struct fsl_dsp *dsp_priv = container_of(proxy,
					struct fsl_dsp, proxy);
	struct xf_message *m;
	int i;

	/* ...create a list of all messages in a pool; set head pointer */
	proxy->free = &proxy->pool[0];

	/* ...put all messages into a single-linked list */
	for (i = 0, m = proxy->free; i < XF_CFG_MESSAGE_POOL_SIZE - 1; i++, m++)
		m->next = m + 1;

	/* ...set list tail pointer */
	m->next = NULL;

	/* ...initialize proxy lock */
	xf_lock_init(&proxy->lock);

	/* ...initialize proxy thread message queues */
	xf_msg_queue_init(&proxy->command);
	xf_msg_queue_init(&proxy->response);

	/* ...initialize global busy queue */
	init_waitqueue_head(&proxy->busy);
	init_waitqueue_head(&proxy->wait);

	/* ...create work structure */
	INIT_WORK(&proxy->work, xf_proxy_process);

	/* ...set pointer to shared memory */
	proxy->ipc.shmem = (struct xf_shmem_data *)dsp_priv->msg_buf_virt;

	/* ...initialize shared memory interface */
	XF_PROXY_WRITE(proxy, cmd_read_idx, 0);
	XF_PROXY_WRITE(proxy, cmd_write_idx, 0);
	XF_PROXY_WRITE(proxy, cmd_invalid, 0);
	XF_PROXY_WRITE(proxy, rsp_read_idx, 0);
	XF_PROXY_WRITE(proxy, rsp_write_idx, 0);
	XF_PROXY_WRITE(proxy, rsp_invalid, 0);

	return 0;
}

/* ...trigger shared memory interface processing */
void xf_proxy_notify(struct xf_proxy *proxy)
{
	schedule_work(&proxy->work);
}

/* ...submit a command to proxy pending queue (lock released upon return) */
void xf_proxy_command(struct xf_proxy *proxy, struct xf_message *m)
{
	int first;

	/* ...submit message to proxy thread */
	first = xf_msg_enqueue(&proxy->command, m);

	/* ...release the lock */
	xf_unlock(&proxy->lock);

	/* ...notify thread about command reception */
	(first ? xf_proxy_notify(proxy), 1 : 0);
}

/*
 * Proxy cmd send and receive functions
 */
int xf_cmd_send(struct xf_proxy *proxy,
				u32 id,
				u32 opcode,
				void *buffer,
				u32 length)
{
	struct xf_message *m;
	int ret;

	/* ...retrieve message handle (take the lock on success) */
	ret = wait_event_interruptible(proxy->busy,
					(m = xf_msg_available(proxy)) != NULL);
	if (ret)
		return -EINTR;

	/* ...fill-in message parameters (lock is taken) */
	m->id = id;
	m->opcode = opcode;
	m->length = length;
	m->buffer = buffer;
	m->ret = 0;

	/* ...submit command to the proxy */
	xf_proxy_command(proxy, m);

	return 0;
}

struct xf_message *xf_cmd_recv(struct xf_proxy *proxy,
						  wait_queue_head_t *wq,
						  struct xf_msg_queue *queue,
						  int wait)
{
	struct xf_message *m;
	int ret;

	/* ...wait for message reception (take lock on success) */
	ret = wait_event_interruptible(*wq,
			(m = xf_msg_received(proxy, queue)) != NULL || !wait);
	if (ret)
		return ERR_PTR(-EINTR);

	/* ...return message with a lock taken */
	return m;
}

struct xf_message *xf_cmd_recv_timeout(struct xf_proxy *proxy,
				       wait_queue_head_t *wq,
				       struct xf_msg_queue *queue, int wait)
{
	struct xf_message *m;
	int ret;

	/* ...wait for message reception (take lock on success) */
	ret = wait_event_interruptible_timeout(*wq,
			(m = xf_msg_received(proxy, queue)) != NULL || !wait,
			msecs_to_jiffies(1000));
	if (ret < 0)
		return ERR_PTR(-EINTR);

	if (ret == 0)
		return ERR_PTR(-ETIMEDOUT);

	/* ...return message with a lock taken */
	return m;
}

/* ...helper function for synchronous command execution */
struct xf_message *xf_cmd_send_recv(struct xf_proxy *proxy,
							   u32 id, u32 opcode,
							   void *buffer,
							   u32 length)
{
	int ret;

	/* ...send command to remote proxy */
	ret = xf_cmd_send(proxy, id, opcode, buffer, length);
	if (ret)
		return ERR_PTR(ret);

	/* ...wait for message delivery */
	return xf_cmd_recv(proxy, &proxy->wait, &proxy->response, 1);
}

struct xf_message *xf_cmd_send_recv_wq(struct xf_proxy *proxy, u32 id,
				       u32 opcode, void *buffer, u32 length,
				       wait_queue_head_t *wq,
				       struct xf_msg_queue *queue)
{
	int ret;

	/* ...send command to remote proxy */
	ret = xf_cmd_send(proxy, id, opcode, buffer, length);
	if (ret)
		return ERR_PTR(ret);

	/* ...wait for message delivery */
	return xf_cmd_recv(proxy, wq, queue, 1);
}

struct xf_message *xf_cmd_send_recv_complete(struct xf_client *client,
					     struct xf_proxy *proxy,
					     u32 id, u32 opcode, void *buffer,
					     u32 length,
					     struct work_struct *work,
					     struct completion *completion)
{
	struct xf_message *m;
	int ret;

	/* ...retrieve message handle (take the lock on success) */
	m = xf_msg_available(proxy);
	if (!m)
		return ERR_PTR(-EBUSY);

	/* ...fill-in message parameters (lock is taken) */
	m->id = id;
	m->opcode = opcode;
	m->length = length;
	m->buffer = buffer;
	m->ret = 0;

	init_completion(completion);

	/* ...submit command to the proxy */
	xf_proxy_command(proxy, m);

	schedule_work(work);

	/* ...wait for message reception (take lock on success) */
	ret = wait_for_completion_timeout(completion,
					  msecs_to_jiffies(1000));
	if (!ret)
		return ERR_PTR(-ETIMEDOUT);

	m = &client->m;

	/* ...return message with a lock taken */
	return m;
}
/*
 * Proxy allocate and free memory functions
 */
/* ...allocate memory buffer for kernel use */
int xf_cmd_alloc(struct xf_proxy *proxy, void **buffer, u32 length)
{
	struct xf_message *m;
	u32 id = 0;
	int ret;

	/* ...send command to remote proxy */
	m = xf_cmd_send_recv(proxy, id, XF_ALLOC, NULL, length);
	if (IS_ERR(m)) {
		ret = PTR_ERR(m);
		return ret;
	}

	/* ...check if response is expected */
	if (m->opcode == XF_ALLOC && m->buffer != NULL) {
		*buffer = m->buffer;
		ret = 0;
	} else {
		ret = -ENOMEM;
	}

	/* ...free message and release proxy lock */
	xf_msg_free(proxy, m);
	xf_unlock(&proxy->lock);

	return ret;
}

/* ...free memory buffer */
int xf_cmd_free(struct xf_proxy *proxy, void *buffer, u32 length)
{
	struct xf_message *m;
	u32 id = 0;
	int ret;

	/* ...synchronously execute freeing command */
	m = xf_cmd_send_recv(proxy, id, XF_FREE, buffer, length);
	if (IS_ERR(m)) {
		ret = PTR_ERR(m);
		return ret;
	}

	/* ...check if response is expected */
	if (m->opcode == XF_FREE)
		ret = 0;
	else
		ret = -EINVAL;

	/* ...free message and release proxy lock */
	xf_msg_free(proxy, m);
	xf_unlock(&proxy->lock);

	return ret;
}

/*
 * suspend & resume functions
 */
int xf_cmd_send_suspend(struct xf_proxy *proxy)
{
	union icm_header_t msghdr;
	int ret = 0;

	init_completion(&proxy->cmd_complete);

	msghdr.allbits = 0;	/* clear all bits; */
	msghdr.ack  = 0;
	msghdr.intr = 1;
	msghdr.msg  = XF_SUSPEND;
	msghdr.size = 0;
	icm_intr_send(proxy, msghdr.allbits);

	/* wait for response here */
	ret = icm_ack_wait(proxy, msghdr.allbits);

	return ret;
}

int xf_cmd_send_resume(struct xf_proxy *proxy)
{
	union icm_header_t msghdr;
	int ret = 0;

	init_completion(&proxy->cmd_complete);

	msghdr.allbits = 0;	/* clear all bits; */
	msghdr.ack  = 0;
	msghdr.intr = 1;
	msghdr.msg  = XF_RESUME;
	msghdr.size = 0;
	icm_intr_send(proxy, msghdr.allbits);

	/* wait for response here */
	ret = icm_ack_wait(proxy, msghdr.allbits);

	return ret;
}
