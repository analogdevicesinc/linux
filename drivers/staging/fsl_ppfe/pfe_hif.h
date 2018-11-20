/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#ifndef _PFE_HIF_H_
#define _PFE_HIF_H_

#include <linux/netdevice.h>

#define HIF_NAPI_STATS

#define HIF_CLIENT_QUEUES_MAX	16
#define HIF_RX_POLL_WEIGHT	64

#define HIF_RX_PKT_MIN_SIZE 0x800 /* 2KB */
#define HIF_RX_PKT_MIN_SIZE_MASK ~(HIF_RX_PKT_MIN_SIZE - 1)
#define ROUND_MIN_RX_SIZE(_sz) (((_sz) + (HIF_RX_PKT_MIN_SIZE - 1)) \
					& HIF_RX_PKT_MIN_SIZE_MASK)
#define PRESENT_OFST_IN_PAGE(_buf) (((unsigned long int)(_buf) & (PAGE_SIZE \
					- 1)) & HIF_RX_PKT_MIN_SIZE_MASK)

enum {
	NAPI_SCHED_COUNT = 0,
	NAPI_POLL_COUNT,
	NAPI_PACKET_COUNT,
	NAPI_DESC_COUNT,
	NAPI_FULL_BUDGET_COUNT,
	NAPI_CLIENT_FULL_COUNT,
	NAPI_MAX_COUNT
};

/*
 * HIF_TX_DESC_NT value should be always greter than 4,
 * Otherwise HIF_TX_POLL_MARK will become zero.
 */
#define HIF_RX_DESC_NT		256
#define HIF_TX_DESC_NT		2048

#define HIF_FIRST_BUFFER	BIT(0)
#define HIF_LAST_BUFFER		BIT(1)
#define HIF_DONT_DMA_MAP	BIT(2)
#define HIF_DATA_VALID		BIT(3)
#define HIF_TSO			BIT(4)

enum {
	PFE_CL_GEM0 = 0,
	PFE_CL_GEM1,
	HIF_CLIENTS_MAX
};

/*structure to store client queue info */
struct hif_rx_queue {
	struct rx_queue_desc *base;
	u32	size;
	u32	write_idx;
};

struct hif_tx_queue {
	struct tx_queue_desc *base;
	u32	size;
	u32	ack_idx;
};

/*Structure to store the client info */
struct hif_client {
	int	rx_qn;
	struct hif_rx_queue	rx_q[HIF_CLIENT_QUEUES_MAX];
	int	tx_qn;
	struct hif_tx_queue	tx_q[HIF_CLIENT_QUEUES_MAX];
};

/*HIF hardware buffer descriptor */
struct hif_desc {
	u32 ctrl;
	u32 status;
	u32 data;
	u32 next;
};

struct __hif_desc {
	u32 ctrl;
	u32 status;
	u32 data;
};

struct hif_desc_sw {
	dma_addr_t data;
	u16 len;
	u8 client_id;
	u8 q_no;
	u16 flags;
};

struct hif_hdr {
	u8 client_id;
	u8 q_num;
	u16 client_ctrl;
	u16 client_ctrl1;
};

struct __hif_hdr {
	union {
		struct hif_hdr hdr;
		u32 word[2];
	};
};

struct hif_ipsec_hdr {
	u16	sa_handle[2];
} __packed;

/*  HIF_CTRL_TX... defines */
#define HIF_CTRL_TX_CHECKSUM		BIT(2)

/*  HIF_CTRL_RX... defines */
#define HIF_CTRL_RX_OFFSET_OFST         (24)
#define HIF_CTRL_RX_CHECKSUMMED		BIT(2)
#define HIF_CTRL_RX_CONTINUED		BIT(1)

struct pfe_hif {
	/* To store registered clients in hif layer */
	struct hif_client client[HIF_CLIENTS_MAX];
	struct hif_shm *shm;
	int	irq;

	void	*descr_baseaddr_v;
	unsigned long	descr_baseaddr_p;

	struct hif_desc *rx_base;
	u32	rx_ring_size;
	u32	rxtoclean_index;
	void	*rx_buf_addr[HIF_RX_DESC_NT];
	int	rx_buf_len[HIF_RX_DESC_NT];
	unsigned int qno;
	unsigned int client_id;
	unsigned int client_ctrl;
	unsigned int started;

	struct hif_desc *tx_base;
	u32	tx_ring_size;
	u32	txtosend;
	u32	txtoclean;
	u32	txavail;
	u32	txtoflush;
	struct hif_desc_sw tx_sw_queue[HIF_TX_DESC_NT];

/* tx_lock synchronizes hif packet tx as well as pfe_hif structure access */
	spinlock_t tx_lock;
/* lock synchronizes hif rx queue processing */
	spinlock_t lock;
	struct net_device	dummy_dev;
	struct napi_struct	napi;
	struct device *dev;

#ifdef HIF_NAPI_STATS
	unsigned int napi_counters[NAPI_MAX_COUNT];
#endif
	struct tasklet_struct	tx_cleanup_tasklet;
};

void __hif_xmit_pkt(struct pfe_hif *hif, unsigned int client_id, unsigned int
			q_no, void *data, u32 len, unsigned int flags);
int hif_xmit_pkt(struct pfe_hif *hif, unsigned int client_id, unsigned int q_no,
		 void *data, unsigned int len);
void __hif_tx_done_process(struct pfe_hif *hif, int count);
void hif_process_client_req(struct pfe_hif *hif, int req, int data1, int
				data2);
int pfe_hif_init(struct pfe *pfe);
void pfe_hif_exit(struct pfe *pfe);
void pfe_hif_rx_idle(struct pfe_hif *hif);
static inline void hif_tx_done_process(struct pfe_hif *hif, int count)
{
	spin_lock_bh(&hif->tx_lock);
	__hif_tx_done_process(hif, count);
	spin_unlock_bh(&hif->tx_lock);
}

static inline void hif_tx_lock(struct pfe_hif *hif)
{
	spin_lock_bh(&hif->tx_lock);
}

static inline void hif_tx_unlock(struct pfe_hif *hif)
{
	spin_unlock_bh(&hif->tx_lock);
}

static inline int __hif_tx_avail(struct pfe_hif *hif)
{
	return hif->txavail;
}

#define __memcpy8(dst, src)		memcpy(dst, src, 8)
#define __memcpy12(dst, src)		memcpy(dst, src, 12)
#define __memcpy(dst, src, len)		memcpy(dst, src, len)

#endif /* _PFE_HIF_H_ */
