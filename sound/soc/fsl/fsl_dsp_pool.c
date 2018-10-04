// SPDX-License-Identifier: GPL-2.0+
//
// Xtensa buffer pool API
//
// Copyright 2018 NXP
// Copyright (c) 2012-2013 by Tensilica Inc.

#include <linux/slab.h>
#include <soc/imx8/sc/ipc.h>

#include "fsl_dsp_pool.h"
#include "fsl_dsp.h"

/* ...allocate buffer pool */
int xf_pool_alloc(struct xf_client *client, struct xf_proxy *proxy,
		  u32 number, u32 length, xf_pool_type_t type,
		  struct xf_pool **pool)
{
	struct xf_pool      *p;
	struct xf_buffer    *b;
	void  *data;
	struct xf_message    msg;
	struct xf_message   *rmsg;

	/* ...basic sanity checks; number of buffers is positive */
	if (number <=0)
		return -EINVAL;

	/* ...get properly aligned buffer length */
	length = ALIGN(length, XF_PROXY_ALIGNMENT);

	p = kzalloc(offsetof(struct xf_pool, buffer) +
		    number * sizeof(struct xf_buffer), GFP_KERNEL);
	if(!p)
		return -ENOMEM;

	/* ...prepare command parameters */
	msg.id = __XF_MSG_ID(__XF_AP_PROXY(0), __XF_DSP_PROXY(0));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_ALLOC;
	msg.length = length * number;
	msg.buffer = NULL;
	msg.ret = 0;

	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);
	if (IS_ERR(rmsg)) {
		kfree(p);
		return PTR_ERR(rmsg);
	}

	p->p = rmsg->buffer;
	/* TODO: review cleanup */
	/* xf_msg_free(proxy, rmsg);
	 * xf_unlock(&proxy->lock); */

	/* ...if operation is failed, do cleanup */
	/* ...set pool parameters */
	p->number = number, p->length = length;
	p->proxy = proxy;

	/* ...create individual buffers and link them into free list */
	for (p->free = b = &p->buffer[0], data = p->p; number > 0;
			number--, b++) {
		/* ...set address of the buffer (no length there) */
		b->address = data;

		/* ...file buffer into the free list */
		b->link.next = b + 1;

		/* ...advance data pointer in contiguous buffer */
		data += length;
	}

	/* ...terminate list of buffers (not too good - tbd) */
	b[-1].link.next = NULL;

	/* ...return buffer pointer */
	*pool = p;

	return 0;
}
/* ...buffer pool destruction */
int xf_pool_free(struct xf_client *client, struct xf_pool *pool)
{
	struct xf_proxy     *proxy;
	struct xf_message    msg;
	struct xf_message   *rmsg;

	/* ...basic sanity checks; pool is positive */
	if (pool == NULL)
		return -EINVAL;

	/* ...get proxy pointer */
	if ((proxy = pool->proxy) == NULL)
		return -EINVAL;

	/* ...prepare command parameters */
	msg.id = __XF_MSG_ID(__XF_AP_PROXY(0), __XF_DSP_PROXY(0));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_FREE;
	msg.length = pool->length * pool->number;
	msg.buffer = pool->p;
	msg.ret = 0;

	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);
	kfree(pool);
	if (IS_ERR(rmsg))
		return PTR_ERR(rmsg);

	/* TODO: review cleanup */
	/* xf_msg_free(proxy, rmsg);
	 * xf_unlock(&proxy->lock); */

	return 0;
}

/* ...get new buffer from a pool */
struct xf_buffer *xf_buffer_get(struct xf_pool *pool)
{
	struct xf_buffer    *b;

	xf_lock(&pool->proxy->lock);
	/* ...take buffer from a head of the free list */
	b = pool->free;
	if (b) {
		/* ...advance free list head */
		pool->free = b->link.next, b->link.pool = pool;
	}

	xf_unlock(&pool->proxy->lock);
	return b;
}

/* ...return buffer back to pool */
void xf_buffer_put(struct xf_buffer *buffer)
{
	struct xf_pool  *pool = buffer->link.pool;

	xf_lock(&pool->proxy->lock);
	/* ...use global proxy lock for pool operations protection */
	/* ...put buffer back to a pool */
	buffer->link.next = pool->free, pool->free = buffer;

	xf_unlock(&pool->proxy->lock);
}
