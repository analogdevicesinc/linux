/*******************************************************************************
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Copyright (c) 2017 Cadence Design Systems, Inc.
 * Copyright 2018 NXP
 *
 ************************************************************/
/************************************************************
 * fsl_hifi4_proxy.h
 *
 * Proxy commmand/response messages
 ************************************************************/

#ifndef __FSL_HIFI4_PROXY_H
#define __FSL_HIFI4_PROXY_H

#include <linux/wait.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/mx8_mu.h>
#include <linux/interrupt.h>

#define XF_CFG_MESSAGE_POOL_SIZE        256

/*******************************************************************************
 * Local proxy data
 ******************************************************************************/

/* ...execution message */
struct xf_message {
	/* ...pointer to next message in a list */
	struct xf_message   *next;

	/* ...session-id */
	u32                 id;

	/* ...operation code */
	u32                 opcode;

	/* ...length of data buffer */
	u32                 length;

	/* ...translated data pointer */
	void               *buffer;

	/* ...return message status */
	u32                ret;
};

/* ...message queue */
struct xf_msg_queue {
	/* ...pointer to list head */
	struct xf_message   *head;

	/* ...pointer to list tail */
	struct xf_message   *tail;
};

struct xf_proxy_message {
	/* ...session ID */
	u32 session_id;

	/* ...proxy API command/response code */
	u32 opcode;

	/* ...length of attached buffer */
	u32 length;

	/* ...physical address of message buffer */
	u32 address;

	/* ...return message status */
	u32 ret;
};
/**********************************************************************/

enum icm_action_t {
	ICM_CORE_READY = 1,
	ICM_CORE_INIT,
	ICM_CORE_EXIT,
};

/* ...adjust IPC client of message going from user-space */
#define XF_MSG_AP_FROM_USER(id, client) (((id) & ~(0xF << 2)) | (client << 2))

/* ...message id contains source and destination ports specification */
#define __XF_MSG_ID(src, dst)   (((src) & 0xFFFF) | (((dst) & 0xFFFF) << 16))

/* ...wipe out IPC client from message going to user-space */
#define XF_MSG_AP_TO_USER(id)   ((id) & ~(0xF << 18))
#define __XF_AP_PROXY(core)     ((core) | 0x8000)
#define __XF_DSP_PROXY(core)    ((core) | 0x8000)

/* ...message id contains source and destination ports specification */
#define __XF_MSG_ID(src, dst)   (((src) & 0xFFFF) | (((dst) & 0xFFFF) << 16))
#define XF_MSG_SRC_CLIENT(id)   (((id) >> 2) & 0x3F)
#define XF_MSG_DST_CLIENT(id)   (((id) >> 18) & 0x3F)

/* ...special treatment of AP-proxy destination field */
#define XF_AP_IPC_CLIENT(id)    (((id) >> 18) & 0xF)
#define __XF_AP_PROXY(core)     ((core) | 0x8000)
#define __XF_DSP_PROXY(core)    ((core) | 0x8000)

/* ...opcode composition with command/response data tags */
#define __XF_OPCODE(c, r, op)   (((c) << 31) | ((r) << 30) | ((op) & 0x3F))

/* ...shared buffer allocation */
#define XF_ALLOC                        __XF_OPCODE(0, 0, 4)

/* ...shared buffer freeing */
#define XF_FREE                         __XF_OPCODE(0, 0, 5)

/* ...resume component operation */
#define XF_RESUME                       __XF_OPCODE(0, 0, 14)

/* ...resume component operation */
#define XF_SUSPEND                      __XF_OPCODE(0, 0, 15)


/*******************************************************************************
 * Ring buffer support
 ******************************************************************************/
/* ...cache-line size on DSP */
#define XF_PROXY_ALIGNMENT              64

/* ...total length of shared memory queue (for commands and responses) */
#define XF_PROXY_MESSAGE_QUEUE_LENGTH   (1 << 6)

/* ...index mask */
#define XF_PROXY_MESSAGE_QUEUE_MASK     0x3F

/* ...ring-buffer index */
#define __XF_QUEUE_IDX(idx, counter)    \
		(((idx) & XF_PROXY_MESSAGE_QUEUE_MASK) | ((counter) << 16))

/* ...retrieve ring-buffer index */
#define XF_QUEUE_IDX(idx)               \
		((idx) & XF_PROXY_MESSAGE_QUEUE_MASK)

/* ...increment ring-buffer index */
#define XF_QUEUE_ADVANCE_IDX(idx)       \
		(((idx) + 0x10001) & (0xFFFF0000 | XF_PROXY_MESSAGE_QUEUE_MASK))

/* ...test if ring buffer is empty */
#define XF_QUEUE_EMPTY(read, write)     \
		((read) == (write))

/* ...test if ring buffer is full */
#define XF_QUEUE_FULL(read, write)      \
		((write) == (read) + (XF_PROXY_MESSAGE_QUEUE_LENGTH << 16))

/* ...basic cache operations */
#define XF_PROXY_INVALIDATE(addr, len)  { }

#define XF_PROXY_FLUSH(addr, len)       { }

/* ...data managed by host CPU (remote) - in case of shunt it is a IPC layer */
struct xf_proxy_host_data {
	/* ...command queue */
	struct xf_proxy_message command[XF_PROXY_MESSAGE_QUEUE_LENGTH];

	/* ...writing index into command queue */
	u32                 cmd_write_idx;

	/* ...reading index for response queue */
	u32                 rsp_read_idx;

	/* ...indicate command queue is valid or not */
	u32                 cmd_invalid;
};

/* ...data managed by DSP (local) */
struct xf_proxy_dsp_data {
	/* ...response queue */
	struct xf_proxy_message response[XF_PROXY_MESSAGE_QUEUE_LENGTH];

	/* ...writing index into response queue */
	u32                 rsp_write_idx;

	/* ...reading index for command queue */
	u32                 cmd_read_idx;

	/* ...indicate response queue is valid or not */
	u32                 rsp_invalid;
};

/* ...shared memory data */
struct xf_shmem_data {
	/* ...ingoing data (maintained by DSP (local side)) */
	struct xf_proxy_host_data    local;

	/* ...outgoing data (maintained by host CPU (remote side)) */
	struct xf_proxy_dsp_data   remote;

};

/* ...shared memory data accessor */
#define XF_SHMEM_DATA(proxy)                \
		((proxy)->ipc.shmem)

/* ...atomic reading */
#define __XF_PROXY_READ_ATOMIC(var)         \
	({ XF_PROXY_INVALIDATE(&(var), sizeof(var));  \
	 *(u32 *)&(var); })

/* ...atomic writing */
#define __XF_PROXY_WRITE_ATOMIC(var, value) \
	({*(u32 *)&(var) = (value);    \
	 XF_PROXY_FLUSH(&(var), sizeof(var));   \
	 (value); })

/* ...accessors */
#define XF_PROXY_READ(proxy, field)          \
		__XF_PROXY_READ_##field(XF_SHMEM_DATA(proxy))

#define XF_PROXY_WRITE(proxy, field, v)      \
		__XF_PROXY_WRITE_##field(XF_SHMEM_DATA(proxy), (v))

/* ...individual fields reading */
#define __XF_PROXY_READ_cmd_write_idx(shmem)        \
		__XF_PROXY_READ_ATOMIC(shmem->local.cmd_write_idx)

#define __XF_PROXY_READ_cmd_read_idx(shmem)         \
		shmem->remote.cmd_read_idx

#define __XF_PROXY_READ_cmd_invalid(shmem)            \
		__XF_PROXY_READ_ATOMIC(shmem->local.cmd_invalid)

#define __XF_PROXY_READ_rsp_write_idx(shmem)        \
		__XF_PROXY_READ_ATOMIC(shmem->remote.rsp_write_idx)

#define __XF_PROXY_READ_rsp_read_idx(shmem)         \
		shmem->local.rsp_read_idx

#define __XF_PROXY_READ_rsp_invalid(shmem)            \
		__XF_PROXY_READ_ATOMIC(shmem->remote.rsp_invalid)

/* ...individual fields writings */
#define __XF_PROXY_WRITE_cmd_write_idx(shmem, v)    \
		__XF_PROXY_WRITE_ATOMIC(shmem->local.cmd_write_idx, v)

#define __XF_PROXY_WRITE_cmd_read_idx(shmem, v)     \
		__XF_PROXY_WRITE_ATOMIC(shmem->remote.cmd_read_idx, v)

#define __XF_PROXY_WRITE_cmd_invalid(shmem, v)     \
		__XF_PROXY_WRITE_ATOMIC(shmem->local.cmd_invalid, v)

#define __XF_PROXY_WRITE_rsp_read_idx(shmem, v)     \
		__XF_PROXY_WRITE_ATOMIC(shmem->local.rsp_read_idx, v)

#define __XF_PROXY_WRITE_rsp_write_idx(shmem, v)    \
		__XF_PROXY_WRITE_ATOMIC(shmem->remote.rsp_write_idx, v)

#define __XF_PROXY_WRITE_rsp_invalid(shmem, v)     \
		__XF_PROXY_WRITE_ATOMIC(shmem->remote.rsp_invalid, v)

/* ...command buffer accessor */
#define XF_PROXY_COMMAND(proxy, idx)                \
		(&XF_SHMEM_DATA(proxy)->local.command[(idx)])

/* ...response buffer accessor */
#define XF_PROXY_RESPONSE(proxy, idx)               \
		(&XF_SHMEM_DATA(proxy)->remote.response[(idx)])

/*******************************************************************************
 * Local proxy data
 ******************************************************************************/

struct xf_proxy_ipc_data {
	/* ...shared memory data pointer */
	struct xf_shmem_data __iomem     *shmem;

	/* ...core identifier */
	u32                     core;

	/* ...IPC registers memory */
	void __iomem           *regs;
};

/* ...proxy data */
struct xf_proxy {
	/* ...IPC layer data */
	struct xf_proxy_ipc_data ipc;

	/* ...shared memory status change processing item */
	struct work_struct      work;

	struct completion		cmd_complete;
	int						is_ready;

	/* ...internal lock */
	spinlock_t              lock;

	/* ...busy queue (for clients waiting ON NOTIFIcation) */
	wait_queue_head_t       busy;

	/* ...waiting queue for synchronous proxy operations */
	wait_queue_head_t       wait;

	/* ...submitted commands queue */
	struct xf_msg_queue     command;

	/* ...pending responses queue */
	struct xf_msg_queue     response;

	/* ...global message pool */
	struct xf_message       pool[XF_CFG_MESSAGE_POOL_SIZE];

	/* ...pointer to first free message in the pool */
	struct xf_message       *free;
};

union icm_header_t {
	struct {
		u32 msg:6;
		u32 sub_msg:6;      // sub_msg will have ICM_MSG
		u32 rsvd:3;     /* reserved */
		u32 intr:1;     /* intr = 1 when sending msg. */
		u32 size:15;    /* =size in bytes (excluding header) */
		u32 ack:1;      /* response message when ack=1 */
	};
	u32 allbits;
};

struct hifi4_ext_msg {
	u32	phys;
	u32	size;
};

struct hifi4_mem_msg {
	u32 ext_msg_phys;
	u32 ext_msg_size;
	u32 scratch_phys;
	u32 scratch_size;
	u32 hifi_config_phys;
	u32 hifi_config_size;
};

static inline void xf_lock_init(spinlock_t *lock)
{
	spin_lock_init(lock);
}

static inline void xf_lock(spinlock_t *lock)
{
	spin_lock(lock);
}

static inline void xf_unlock(spinlock_t *lock)
{
	spin_unlock(lock);
}

/* ...init proxy */
int xf_proxy_init(struct xf_proxy *proxy);

/* ...send message to proxy */
int xf_cmd_send(struct xf_proxy *proxy,
				u32 id,
				u32 opcode,
				void *buffer,
				u32 length);

/* ...get message from proxy */
struct xf_message *xf_cmd_recv(struct xf_proxy *proxy,
					wait_queue_head_t *wq,
					struct xf_msg_queue *queue,
					int wait);

/* ...mu interrupt handle */
irqreturn_t fsl_hifi4_mu_isr(int irq, void *dev_id);

/* ...initialize client pending message queue */
void xf_msg_queue_init(struct xf_msg_queue *queue);

/* ...return current queue state */
struct xf_message *xf_msg_queue_head(struct xf_msg_queue *queue);

/* ...return the message back to a pool */
void xf_msg_free(struct xf_proxy *proxy, struct xf_message *m);

/* ...release all pending messages */
void xf_msg_free_all(struct xf_proxy *proxy, struct xf_msg_queue *queue);

/* ...wait mu interrupt */
long icm_ack_wait(struct xf_proxy *proxy, u32 msg);

/* ...shared memory translation - kernel virtual address to shared address */
u32 xf_proxy_b2a(struct xf_proxy *proxy, void *b);

/* ...shared memory translation - shared address to kernel virtual address */
void *xf_proxy_a2b(struct xf_proxy *proxy, u32 address);

int xf_cmd_send_suspend(struct xf_proxy *proxy);
int xf_cmd_send_resume(struct xf_proxy *proxy);

#endif
