
/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef _ICC_H
#define _ICC_H

/* sm protocol */
/* compose type enumeration value from protocol & subtype */
#define SM_MSG_TYPE(protocol, subtype) (((protocol) << 24) | (subtype))

/* extract subtype from type enumeration value */
#define SM_MSG_SUBTYPE(type) ((type) & 0xffffff)

/* extract protocol from type enumeration value */
#define SM_MSG_PROTOCOL(type) (((type) >> 24) & 0xff)

#ifdef CONFIG_ICC_DEBUG
#define sm_debug(fmt, ...) \
	pr_crit("sm_debug:"pr_fmt(fmt), ##__VA_ARGS__)
#else
#define sm_debug(fmt, ...) \
	({ if (0) pr_crit("sm_debug:"pr_fmt(fmt), ##__VA_ARGS__); 0; })
#endif

enum {
	SP_GENERAL = 0,
	SP_CORE_CONTROL,
	SP_TASK_MANAGER,
	SP_RES_MANAGER,
	SP_PACKET,
	SP_SESSION_PACKET,
	SP_SCALAR,
	SP_SESSION_SCALAR,
	SP_MAX,
};

enum icc_queue_attr {
	ICC_QUEUE_ATTR_STATUS = 0,
	ICC_QUEUE_ATTR_MAX,
};

enum icc_queue_status {
	ICC_QUEUE_STOP = 0,
	ICC_QUEUE_INIT,
	ICC_QUEUE_READY,
	ICC_QUEUE_STATUS_MAX,
};

enum {
	RESMGR_TYPE_PERIPHERAL = 0,
	RESMGR_TYPE_GPIO,
	RESMGR_TYPE_SYS_IRQ,
	RESMGR_TYPE_DMA,
	RESMGR_TYPE_MAX,
};

enum {
	WAIT_OTHER = 0,
	WAIT_OPEN_PKTCHAN,
	WAIT_OPEN_SCLCHAN,
	WAIT_SEND,
	WAIT_RECV,
	WAIT_GET_ENDPT,
	WAIT_MAX,
};

#define EP_RESMGR_SERVICE 0

#define RES_TYPE_OFFSET  12
#define RES_TYPE_MASK    0xF
#define RES_SUBID_MASK   0xFFF

#define RESMGR_ID(type, subid) ((type << RES_TYPE_OFFSET) | ((subid) & RES_SUBID_MASK))
#define RESMGR_SUBID(id)       ((id) & RES_SUBID_MASK)
#define RESMGR_TYPE(id)        (((id) >> RES_TYPE_OFFSET) & RES_TYPE_MASK)

#define SM_UNCONNECT 0
#define SM_CONNECT 0x1
#define SM_CONNECTING 0x2
#define SM_OPEN 0x4
#define SM_ACTIVE 0x8

#define SM_BAD_ENDPOINT SM_MSG_TYPE(SP_GENERAL, 0)
#define SM_BAD_MSG SM_MSG_TYPE(SP_GENERAL, 1)
#define SM_QUERY_MSG SM_MSG_TYPE(SP_GENERAL, 2)
#define SM_QUERY_ACK_MSG SM_MSG_TYPE(SP_GENERAL, 3)

#define SM_CORE_START		SM_MSG_TYPE(SP_CORE_CONTROL, 0)
#define SM_CORE_STARTED		SM_MSG_TYPE(SP_CORE_CONTROL, 1)
#define SM_CORE_STOP		SM_MSG_TYPE(SP_CORE_CONTROL, 2)
#define SM_CORE_STOPPED		SM_MSG_TYPE(SP_CORE_CONTROL, 3)
#define SM_CORE_RESET		SM_MSG_TYPE(SP_CORE_CONTROL, 4)
#define SM_CORE_RESETED		SM_MSG_TYPE(SP_CORE_CONTROL, 5)

#define SM_TASK_RUN		SM_MSG_TYPE(SP_TASK_MANAGER, 0)
#define SM_TASK_RUN_ACK		SM_MSG_TYPE(SP_TASK_MANAGER, 1)
#define SM_TASK_KILL		SM_MSG_TYPE(SP_TASK_MANAGER, 2)
#define SM_TASK_KILL_ACK	SM_MSG_TYPE(SP_TASK_MANAGER, 3)

#define SM_RES_MGR_REQUEST	SM_MSG_TYPE(SP_RES_MANAGER, 0)
#define SM_RES_MGR_REQUEST_OK	SM_MSG_TYPE(SP_RES_MANAGER, 1)
#define SM_RES_MGR_REQUEST_FAIL	SM_MSG_TYPE(SP_RES_MANAGER, 2)
#define SM_RES_MGR_FREE		SM_MSG_TYPE(SP_RES_MANAGER, 3)
#define SM_RES_MGR_FREE_DONE	SM_MSG_TYPE(SP_RES_MANAGER, 4)
#define SM_RES_MGR_EXPIRE	SM_MSG_TYPE(SP_RES_MANAGER, 5)
#define SM_RES_MGR_EXPIRE_DONE	SM_MSG_TYPE(SP_RES_MANAGER, 6)
#define SM_RES_MGR_LIST		SM_MSG_TYPE(SP_RES_MANAGER, 7)
#define SM_RES_MGR_LIST_OK	SM_MSG_TYPE(SP_RES_MANAGER, 8)
#define SM_RES_MGR_LIST_DONE	SM_MSG_TYPE(SP_RES_MANAGER, 9)

#define SM_PACKET_READY		SM_MSG_TYPE(SP_PACKET, 0)
#define SM_PACKET_CONSUMED	SM_MSG_TYPE(SP_PACKET, 1)
#define SM_PACKET_ERROR		SM_MSG_TYPE(SP_PACKET, 2)
#define SM_PACKET_ERROR_ACK	SM_MSG_TYPE(SP_PACKET, 3)

#define SM_SESSION_PACKET_CONNECT	SM_MSG_TYPE(SP_SESSION_PACKET, 0)
#define SM_SESSION_PACKET_CONNECT_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 1)
#define SM_SESSION_PACKET_CONNECT_DONE	SM_MSG_TYPE(SP_SESSION_PACKET, 2)
#define SM_SESSION_PACKET_ACTIVE	SM_MSG_TYPE(SP_SESSION_PACKET, 3)
#define SM_SESSION_PACKET_ACTIVE_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 4)
#define SM_SESSION_PACKET_CLOSE		SM_MSG_TYPE(SP_SESSION_PACKET, 5)
#define SM_SESSION_PACKET_CLOSE_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 6)
#define SM_SESSION_PACKET_READY		SM_MSG_TYPE(SP_SESSION_PACKET, 7)
#define SM_SESSION_PACKET_CONSUMED	SM_MSG_TYPE(SP_SESSION_PACKET, 8)
#define SM_SESSION_PACKET_ERROR		SM_MSG_TYPE(SP_SESSION_PACKET, 9)
#define SM_SESSION_PACKET_ERROR_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 10)

#define SM_SCALAR_8BIT			0
#define SM_SCALAR_16BIT			1
#define SM_SCALAR_32BIT			2
#define SM_SCALAR_64BIT			3

#define SM_SCALAR_READY_8		SM_MSG_TYPE(SP_SCALAR, SM_SCALAR_8BIT)
#define SM_SCALAR_READY_16		SM_MSG_TYPE(SP_SCALAR, SM_SCALAR_16BIT)
#define SM_SCALAR_READY_32		SM_MSG_TYPE(SP_SCALAR, SM_SCALAR_32BIT)
#define SM_SCALAR_READY_64		SM_MSG_TYPE(SP_SCALAR, SM_SCALAR_64BIT)
#define SM_SCALAR_CONSUMED		SM_MSG_TYPE(SP_SCALAR, 4)
#define SM_SCALAR_ERROR			SM_MSG_TYPE(SP_SCALAR, 5)
#define SM_SCALAR_ERROR_ACK		SM_MSG_TYPE(SP_SCALAR, 6)

#define SM_SESSION_SCALAR_READY_8	SM_MSG_TYPE(SP_SESSION_SCALAR, SM_SCALAR_8BIT)
#define SM_SESSION_SCALAR_READY_16	SM_MSG_TYPE(SP_SESSION_SCALAR, SM_SCALAR_16BIT)
#define SM_SESSION_SCALAR_READY_32	SM_MSG_TYPE(SP_SESSION_SCALAR, SM_SCALAR_32BIT)
#define SM_SESSION_SCALAR_READY_64	SM_MSG_TYPE(SP_SESSION_SCALAR, SM_SCALAR_64BIT)
#define SM_SESSION_SCALAR_CONSUMED	SM_MSG_TYPE(SP_SESSION_SCALAR, 4)
#define SM_SESSION_SCALAR_ERROR		SM_MSG_TYPE(SP_SESSION_SCALAR, 5)
#define SM_SESSION_SCALAR_ERROR_ACK	SM_MSG_TYPE(SP_SESSION_SCALAR, 6)
#define SM_SESSION_SCALAR_CONNECT	SM_MSG_TYPE(SP_SESSION_SCALAR, 7)
#define SM_SESSION_SCALAR_CONNECT_ACK	SM_MSG_TYPE(SP_SESSION_SCALAR, 8)
#define SM_SESSION_SCALAR_CONNECT_DONE	SM_MSG_TYPE(SP_SESSION_SCALAR, 9)
#define SM_SESSION_SCALAR_ACTIVE	SM_MSG_TYPE(SP_SESSION_SCALAR, 10)
#define SM_SESSION_SCALAR_ACTIVE_ACK	SM_MSG_TYPE(SP_SESSION_SCALAR, 11)
#define SM_SESSION_SCALAR_CLOSE		SM_MSG_TYPE(SP_SESSION_SCALAR, 12)
#define SM_SESSION_SCALAR_CLOSE_ACK	SM_MSG_TYPE(SP_SESSION_SCALAR, 13)

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/mutex.h>

void icc_send_ipi_cpu(unsigned int cpu, int irq);
void icc_clear_ipi(unsigned int cpu, int irq);

struct sm_msg {
	u16 dst_ep;
	u16 src_ep;
	u32 type;
	u32 length;
	u32 payload;
};

struct sm_message {
	struct list_head next;
	u16 dst;
	u16 src;
	void *vbuf;
	struct sm_msg msg;
	struct sm_icc_desc *icc_info;
	u32 flags;
	bool b_tx_finish;
};

#define SM_MSGQ_LEN 16

/* Simple FIFO buffer */
struct sm_message_queue {
	u16 sent;
	u16 received; /* head of the queue */
	struct sm_msg messages[SM_MSGQ_LEN];
};

#define SM_MSGQ_NUM		4 /* 2 low bi-direction fifos and 2 high ones */
#define MSGQ_SIZE		(sizeof(struct sm_message_queue) * SM_MSGQ_NUM)

#define DEBUG_MSG_LINE		256
#define DEBUG_MSG_BUF_SIZE	(DEBUG_MSG_LINE * SM_MSGQ_LEN)

struct sm_icc_desc {
	u32 peer_cpu;
	struct sm_message_queue *icc_queue;
	struct sm_message_queue *icc_high_queue;
	u32 *icc_queue_attribute;
	wait_queue_head_t iccq_tx_wait;
	u32 irq;
	u32 notify;
};

struct sm_session {
	struct list_head rx_messages; /*rx queue sm message*/
	struct list_head tx_messages;
	u32	n_avail;
	u32	n_uncompleted;
	u32	local_ep;
	u32	remote_ep; /*remote ep*/
	u32	type;
	pid_t		pid;
	u32	flags;
	int (*handle)(struct sm_message *msg, struct sm_session *session);
	struct sm_proto *proto_ops;
	u32	queue_priority;
	wait_queue_head_t rx_wait;
	bool b_rx_finish;
} __aligned(4);

#define MAX_ENDPOINTS 32
#define MAX_SESSIONS 32
#define MAX_TASK_NAME 64
struct sm_session_table {
	struct list_head next_table;
	struct list_head query_message;
	wait_queue_head_t query_wait;
	bool   b_query_finish;
	struct mutex lock; //sm session lock
	u32	nfree;
	u32 session_mask;
	u32 session_pending;
	u32	bits[(MAX_ENDPOINTS - 1) / BITS_PER_LONG + 1];
	u16	refcnt;
	struct sm_session sessions[MAX_ENDPOINTS];
};

struct sm_proto {
	int (*sendmsg)(struct sm_message *msg, struct sm_session *session);
	int (*recvmsg)(struct sm_msg *msg, struct sm_session *session);
	int (*shutdown)(struct sm_session *session);
	int (*error)(struct sm_msg *msg, struct sm_session *session);
};

struct sm_task {
	int (*task_init)(int argc, char *argv[]);
	void (*task_exit)(void);
	int task_argc;
	char task_argv[3][MAX_TASK_NAME];
};

#define __icc_task __section(".icc.text")
#define __icc_task_data __section(".icc.data")

int sm_send_control_msg(struct sm_session *session, uint32_t remote_ep,
			u32 dst_cpu, uint32_t payload,
			u32 len, uint32_t type);

struct icc_peer_platform_data {
	u32 peerid;
	u32 notify;
};

struct icc_platform_data {
	int	node;
	u32 peer_count;
	struct icc_peer_platform_data *peer_info;
};

struct icc_peri_resource {
	const char *name;
	struct platform_device *pdev;
	u16 resource_id;
};

extern struct icc_peri_resource icc_peri_array[];
#endif

#define CCTRL_CORE1                 1
#define CCTRL_CORE2                 2
#define CMD_CORE_START              _IO('b', 0)
#define CMD_CORE_STOP               _IO('b', 1)
#define CMD_SET_SVECT1              _IO('m', 17)
#define CMD_SET_SVECT2              _IO('m', 18)
#define CMD_SM_SEND                 _IO('m', 3)
#define CMD_SM_CREATE               _IO('m', 4)
#define CMD_SM_CONNECT              _IO('m', 5)
#define CMD_SM_RECV                 _IO('m', 6)
#define CMD_SM_SHUTDOWN             _IO('m', 7)
#define CMD_SM_GET_NODE_STATUS      _IO('m', 8)
#define CMD_SM_GET_SESSION_STATUS   _IO('m', 9)
#define CMD_SM_OPEN                 _IO('m', 10)
#define CMD_SM_CLOSE                _IO('m', 11)
#define CMD_SM_ACTIVE               _IO('m', 12)
#define CMD_SM_REQUEST_UNCACHED_BUF _IO('m', 13)
#define CMD_SM_RELEASE_UNCACHED_BUF _IO('m', 14)
#define CMD_SM_QUERY_REMOTE_EP      _IO('m', 15)
#define CMD_SM_WAIT                 _IO('m', 16)

struct sm_node_status {
	u32 session_mask;
	u32 session_pending;
	u32 nfree;
};

struct sm_session_status {
	u32	n_avail;
	u32	n_uncompleted;
	u32	local_ep;
	u32	remote_ep;
	u32	type;
	u32	pid;
	u32	flags;
};

struct sm_packet {
	u32 session_idx;
	u32 local_ep;
	u32 remote_ep;
	u32 type;
	u32 flag;
	u32 dst_cpu;
	u32 src_cpu;
	u32 buf_len;
	u32 paddr;
	u32 queue_priority;
	void *buf;
	u32 param_len;
	void *param;
	u32 payload;
	int timeout;
};

struct resources_t {
	char label[32];				/* owner name */
	u16 count;				/* resource number in next array */
	u32 resources_array;		/* address of the resource ID array */
};

#define SM_SCALAR_CMD(x) ((x) >> 16 & 0xffff)
#define SM_SCALAR_CMDARG(x) ((x) & 0xffff)
#define SM_SCALAR_CMD_HEAD 0xFE
#define SM_SCALAR_ACK_HEAD   0xFF
#define SM_SCALAR_CMD_GET_SESSION_ID        0x1
#define SM_SCALAR_CMD_GET_SESSION_TYPE      0x2

#define MK_SM_SCALAR_CMD(x) (((x) & 0xffff) | (SM_SCALAR_CMD_HEAD << 16))
#define MK_SM_SCALAR_CMD_ACK(x) (((x) & 0xffff) | (SM_SCALAR_ACK_HEAD << 16))

#define L3_TYPE_AUDIO 1
#define L3_TYPE_VIDEO 2
#define L3_TYPE_ENCODE 4
#define L3_TYPE_DECODE 8

struct l3_proto_head {
	unsigned int type;
	unsigned int todo;
	unsigned int chunk_addr;
	unsigned int chunk_size;
	unsigned int status;
};

int sm_send_packet_ack(struct sm_session *session, uint32_t remote_ep,
		       u32 dst_cpu, uint32_t payload, uint32_t len);
int sm_send_session_packet_ack(struct sm_session *session, uint32_t remote_ep,
			       u32 dst_cpu, uint32_t payload, uint32_t len);

#endif
iff --git a/include/linux/soc/adi/adi_system_config.h b/include/linux/soc/adi/adi_system_config.h
ew file mode 100644
ndex 000000000..31a0ab940
-- /dev/null
++ b/include/linux/soc/adi/adi_system_config.h
@ -0,0 +1,42 @@
/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef SOC_ADI_ADI_SYSTEM_CONFIG_H
#define SOC_ADI_ADI_SYSTEM_CONFIG_H

#include <linux/soc/adi/system_config.h>

/*
 * All possible system register IDs across all platforms supported by this driver
 */
enum adi_system_reg_id {
	ADI_SYSTEM_REG_EMAC0_PTPCLK0 = 0,   /* PTP Clock Source 0 */
	ADI_SYSTEM_REG_EMAC0_EMACRESET,     /* Reset Enable for RGMII */
	ADI_SYSTEM_REG_EMAC0_PHYISEL,       /* Select PHY Interface RGMII/RMII/MII */
	ADI_SYSTEM_REG_CNT0UDSEL,           /* CNT0 Down Input Select */
	ADI_SYSTEM_REG_CNT0DGSEL,           /* CNT0 Up Input Select */
	ADI_SYSTEM_REG_TWI0VSEL,            /* TWI2 Voltage Select */
	ADI_SYSTEM_REG_TWI1VSEL,            /* TWI1 Voltage Select */
	ADI_SYSTEM_REG_TWI2VSEL,            /* TWI0 Voltage Select */
	ADI_SYSTEM_REG_PUMSIDLC,            /* Pull-Up Enable for MSI DATA[3:0] bits and CMD Pin */
	ADI_SYSTEM_REG_PUMSIHL,             /* Pull-Up Enable for MSI DATA[7:4] bits */
	ADI_SYSTEM_REG_PUTMS,               /* Pull-Up Enable for MSI DATA[7:4] bits */
	ADI_SYSTEM_REG_EMAC0_AUXIE,         /* Input enable control for PTP_AUXIN pins */
	ADI_SYSTEM_REG_FAULT_DIS,           /* FAULT does not exist */
	ADI_SYSTEM_REG_EMAC0_ENDIANNESS,    /* EMAC0 DMA transfer endian format */
	ADI_SYSTEM_REG_EMAC1_ENDIANNESS,    /* EMAC1 DMA transfer endian format */
	ADI_SYSTEM_REG_MSHC_CCLK_DIV_EN,    /* Enable MSHC Card Clock Divider */
	ADI_SYSTEM_REG_DAI0_IE,				/* Port input enable for DAI0 */
	ADI_SYSTEM_REG_DAI1_IE,				/* Port input enable for DAI1 */
	__ADI_SYSTEM_REG_COUNT
};

#endif
