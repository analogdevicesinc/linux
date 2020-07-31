// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/slab.h>

#include <linux/io.h>
#include <asm/irq.h>

#include "pfe_mod.h"

#define HIF_INT_MASK	(HIF_INT | HIF_RXPKT_INT | HIF_TXPKT_INT)

unsigned char napi_first_batch;

static void pfe_tx_do_cleanup(unsigned long data);

static int pfe_hif_alloc_descr(struct pfe_hif *hif)
{
	void *addr;
	dma_addr_t dma_addr;
	int err = 0;

	pr_info("%s\n", __func__);
	addr = dma_alloc_coherent(pfe->dev,
				  HIF_RX_DESC_NT * sizeof(struct hif_desc) +
				  HIF_TX_DESC_NT * sizeof(struct hif_desc),
				  &dma_addr, GFP_KERNEL);

	if (!addr) {
		pr_err("%s: Could not allocate buffer descriptors!\n"
			, __func__);
		err = -ENOMEM;
		goto err0;
	}

	hif->descr_baseaddr_p = dma_addr;
	hif->descr_baseaddr_v = addr;
	hif->rx_ring_size = HIF_RX_DESC_NT;
	hif->tx_ring_size = HIF_TX_DESC_NT;

	return 0;

err0:
	return err;
}

#if defined(LS1012A_PFE_RESET_WA)
static void pfe_hif_disable_rx_desc(struct pfe_hif *hif)
{
	int ii;
	struct hif_desc	*desc = hif->rx_base;

	/*Mark all descriptors as LAST_BD */
	for (ii = 0; ii < hif->rx_ring_size; ii++) {
		desc->ctrl |= BD_CTRL_LAST_BD;
		desc++;
	}
}

struct class_rx_hdr_t {
	u32     next_ptr;       /* ptr to the start of the first DDR buffer */
	u16     length;         /* total packet length */
	u16     phyno;          /* input physical port number */
	u32     status;         /* gemac status bits */
	u32     status2;            /* reserved for software usage */
};

/* STATUS_BAD_FRAME_ERR is set for all errors (including checksums if enabled)
 * except overflow
 */
#define STATUS_BAD_FRAME_ERR            BIT(16)
#define STATUS_LENGTH_ERR               BIT(17)
#define STATUS_CRC_ERR                  BIT(18)
#define STATUS_TOO_SHORT_ERR            BIT(19)
#define STATUS_TOO_LONG_ERR             BIT(20)
#define STATUS_CODE_ERR                 BIT(21)
#define STATUS_MC_HASH_MATCH            BIT(22)
#define STATUS_CUMULATIVE_ARC_HIT       BIT(23)
#define STATUS_UNICAST_HASH_MATCH       BIT(24)
#define STATUS_IP_CHECKSUM_CORRECT      BIT(25)
#define STATUS_TCP_CHECKSUM_CORRECT     BIT(26)
#define STATUS_UDP_CHECKSUM_CORRECT     BIT(27)
#define STATUS_OVERFLOW_ERR             BIT(28) /* GPI error */
#define MIN_PKT_SIZE			64

static inline void copy_to_lmem(u32 *dst, u32 *src, int len)
{
	int i;

	for (i = 0; i < len; i += sizeof(u32))	{
		*dst = htonl(*src);
		dst++; src++;
	}
}

static void send_dummy_pkt_to_hif(void)
{
	void *lmem_ptr, *ddr_ptr, *lmem_virt_addr;
	u32 physaddr;
	struct class_rx_hdr_t local_hdr;
	static u32 dummy_pkt[] =  {
		0x33221100, 0x2b785544, 0xd73093cb, 0x01000608,
		0x04060008, 0x2b780200, 0xd73093cb, 0x0a01a8c0,
		0x33221100, 0xa8c05544, 0x00000301, 0x00000000,
		0x00000000, 0x00000000, 0x00000000, 0xbe86c51f };

	ddr_ptr = (void *)((u64)readl(BMU2_BASE_ADDR + BMU_ALLOC_CTRL));
	if (!ddr_ptr)
		return;

	lmem_ptr = (void *)((u64)readl(BMU1_BASE_ADDR + BMU_ALLOC_CTRL));
	if (!lmem_ptr)
		return;

	pr_info("Sending a dummy pkt to HIF %p %p\n", ddr_ptr, lmem_ptr);
	physaddr = (u32)DDR_VIRT_TO_PFE(ddr_ptr);

	lmem_virt_addr = (void *)CBUS_PFE_TO_VIRT((unsigned long int)lmem_ptr);

	local_hdr.phyno = htons(0); /* RX_PHY_0 */
	local_hdr.length = htons(MIN_PKT_SIZE);

	local_hdr.next_ptr = htonl((u32)physaddr);
	/*Mark checksum is correct */
	local_hdr.status = htonl((STATUS_IP_CHECKSUM_CORRECT |
				STATUS_UDP_CHECKSUM_CORRECT |
				STATUS_TCP_CHECKSUM_CORRECT |
				STATUS_UNICAST_HASH_MATCH |
				STATUS_CUMULATIVE_ARC_HIT));
	local_hdr.status2 = 0;

	copy_to_lmem((u32 *)lmem_virt_addr, (u32 *)&local_hdr,
		     sizeof(local_hdr));

	copy_to_lmem((u32 *)(lmem_virt_addr + LMEM_HDR_SIZE), (u32 *)dummy_pkt,
		     0x40);

	writel((unsigned long int)lmem_ptr, CLASS_INQ_PKTPTR);
}

void pfe_hif_rx_idle(struct pfe_hif *hif)
{
	int hif_stop_loop = 10;
	u32 rx_status;

	pfe_hif_disable_rx_desc(hif);
	pr_info("Bringing hif to idle state...");
	writel(0, HIF_INT_ENABLE);
	/*If HIF Rx BDP is busy send a dummy packet */
	do {
		rx_status = readl(HIF_RX_STATUS);
		if (rx_status & BDP_CSR_RX_DMA_ACTV)
			send_dummy_pkt_to_hif();

		usleep_range(100, 150);
	} while (--hif_stop_loop);

	if (readl(HIF_RX_STATUS) & BDP_CSR_RX_DMA_ACTV)
		pr_info("Failed\n");
	else
		pr_info("Done\n");
}
#endif

static void pfe_hif_free_descr(struct pfe_hif *hif)
{
	pr_info("%s\n", __func__);

	dma_free_coherent(pfe->dev,
			  hif->rx_ring_size * sizeof(struct hif_desc) +
			  hif->tx_ring_size * sizeof(struct hif_desc),
			  hif->descr_baseaddr_v, hif->descr_baseaddr_p);
}

void pfe_hif_desc_dump(struct pfe_hif *hif)
{
	struct hif_desc	*desc;
	unsigned long desc_p;
	int ii = 0;

	pr_info("%s\n", __func__);

	desc = hif->rx_base;
	desc_p = (u32)((u64)desc - (u64)hif->descr_baseaddr_v +
			hif->descr_baseaddr_p);

	pr_info("HIF Rx desc base %p physical %x\n", desc, (u32)desc_p);
	for (ii = 0; ii < hif->rx_ring_size; ii++) {
		pr_info("status: %08x, ctrl: %08x, data: %08x, next: %x\n",
			readl(&desc->status), readl(&desc->ctrl),
			readl(&desc->data), readl(&desc->next));
			desc++;
	}

	desc = hif->tx_base;
	desc_p = ((u64)desc - (u64)hif->descr_baseaddr_v +
			hif->descr_baseaddr_p);

	pr_info("HIF Tx desc base %p physical %x\n", desc, (u32)desc_p);
	for (ii = 0; ii < hif->tx_ring_size; ii++) {
		pr_info("status: %08x, ctrl: %08x, data: %08x, next: %x\n",
			readl(&desc->status), readl(&desc->ctrl),
			readl(&desc->data), readl(&desc->next));
		desc++;
	}
}

/* pfe_hif_release_buffers */
static void pfe_hif_release_buffers(struct pfe_hif *hif)
{
	struct hif_desc	*desc;
	int i = 0;

	hif->rx_base = hif->descr_baseaddr_v;

	pr_info("%s\n", __func__);

	/*Free Rx buffers */
	desc = hif->rx_base;
	for (i = 0; i < hif->rx_ring_size; i++) {
		if (readl(&desc->data)) {
			if ((i < hif->shm->rx_buf_pool_cnt) &&
			    (!hif->shm->rx_buf_pool[i])) {
				/*
				 * dma_unmap_single(hif->dev, desc->data,
				 * hif->rx_buf_len[i], DMA_FROM_DEVICE);
				 */
				dma_unmap_single(hif->dev,
						 DDR_PFE_TO_PHYS(
						 readl(&desc->data)),
						 hif->rx_buf_len[i],
						 DMA_FROM_DEVICE);
				hif->shm->rx_buf_pool[i] = hif->rx_buf_addr[i];
			} else {
				pr_err("%s: buffer pool already full\n"
					, __func__);
			}
		}

		writel(0, &desc->data);
		writel(0, &desc->status);
		writel(0, &desc->ctrl);
		desc++;
	}
}

/*
 * pfe_hif_init_buffers
 * This function initializes the HIF Rx/Tx ring descriptors and
 * initialize Rx queue with buffers.
 */
static int pfe_hif_init_buffers(struct pfe_hif *hif)
{
	struct hif_desc	*desc, *first_desc_p;
	u32 data;
	int i = 0;

	pr_info("%s\n", __func__);

	/* Check enough Rx buffers available in the shared memory */
	if (hif->shm->rx_buf_pool_cnt < hif->rx_ring_size)
		return -ENOMEM;

	hif->rx_base = hif->descr_baseaddr_v;
	memset(hif->rx_base, 0, hif->rx_ring_size * sizeof(struct hif_desc));

	/*Initialize Rx descriptors */
	desc = hif->rx_base;
	first_desc_p = (struct hif_desc *)hif->descr_baseaddr_p;

	for (i = 0; i < hif->rx_ring_size; i++) {
		/* Initialize Rx buffers from the shared memory */

		data = (u32)dma_map_single(hif->dev, hif->shm->rx_buf_pool[i],
				pfe_pkt_size, DMA_FROM_DEVICE);
		hif->rx_buf_addr[i] = hif->shm->rx_buf_pool[i];
		hif->rx_buf_len[i] = pfe_pkt_size;
		hif->shm->rx_buf_pool[i] = NULL;

		if (likely(dma_mapping_error(hif->dev, data) == 0)) {
			writel(DDR_PHYS_TO_PFE(data), &desc->data);
		} else {
			pr_err("%s : low on mem\n",  __func__);

			goto err;
		}

		writel(0, &desc->status);

		/*
		 * Ensure everything else is written to DDR before
		 * writing bd->ctrl
		 */
		wmb();

		writel((BD_CTRL_PKT_INT_EN | BD_CTRL_LIFM
			| BD_CTRL_DIR | BD_CTRL_DESC_EN
			| BD_BUF_LEN(pfe_pkt_size)), &desc->ctrl);

		/* Chain descriptors */
		writel((u32)DDR_PHYS_TO_PFE(first_desc_p + i + 1), &desc->next);
		desc++;
	}

	/* Overwrite last descriptor to chain it to first one*/
	desc--;
	writel((u32)DDR_PHYS_TO_PFE(first_desc_p), &desc->next);

	hif->rxtoclean_index = 0;

	/*Initialize Rx buffer descriptor ring base address */
	writel(DDR_PHYS_TO_PFE(hif->descr_baseaddr_p), HIF_RX_BDP_ADDR);

	hif->tx_base = hif->rx_base + hif->rx_ring_size;
	first_desc_p = (struct hif_desc *)hif->descr_baseaddr_p +
				hif->rx_ring_size;
	memset(hif->tx_base, 0, hif->tx_ring_size * sizeof(struct hif_desc));

	/*Initialize tx descriptors */
	desc = hif->tx_base;

	for (i = 0; i < hif->tx_ring_size; i++) {
		/* Chain descriptors */
		writel((u32)DDR_PHYS_TO_PFE(first_desc_p + i + 1), &desc->next);
		writel(0, &desc->ctrl);
		desc++;
	}

	/* Overwrite last descriptor to chain it to first one */
	desc--;
	writel((u32)DDR_PHYS_TO_PFE(first_desc_p), &desc->next);
	hif->txavail = hif->tx_ring_size;
	hif->txtosend = 0;
	hif->txtoclean = 0;
	hif->txtoflush = 0;

	/*Initialize Tx buffer descriptor ring base address */
	writel((u32)DDR_PHYS_TO_PFE(first_desc_p), HIF_TX_BDP_ADDR);

	return 0;

err:
	pfe_hif_release_buffers(hif);
	return -ENOMEM;
}

/*
 * pfe_hif_client_register
 *
 * This function used to register a client driver with the HIF driver.
 *
 * Return value:
 * 0 - on Successful registration
 */
static int pfe_hif_client_register(struct pfe_hif *hif, u32 client_id,
				   struct hif_client_shm *client_shm)
{
	struct hif_client *client = &hif->client[client_id];
	u32 i, cnt;
	struct rx_queue_desc *rx_qbase;
	struct tx_queue_desc *tx_qbase;
	struct hif_rx_queue *rx_queue;
	struct hif_tx_queue *tx_queue;
	int err = 0;

	pr_info("%s\n", __func__);

	spin_lock_bh(&hif->tx_lock);

	if (test_bit(client_id, &hif->shm->g_client_status[0])) {
		pr_err("%s: client %d already registered\n",
		       __func__, client_id);
		err = -1;
		goto unlock;
	}

	memset(client, 0, sizeof(struct hif_client));

	/* Initialize client Rx queues baseaddr, size */

	cnt = CLIENT_CTRL_RX_Q_CNT(client_shm->ctrl);
	/* Check if client is requesting for more queues than supported */
	if (cnt > HIF_CLIENT_QUEUES_MAX)
		cnt = HIF_CLIENT_QUEUES_MAX;

	client->rx_qn = cnt;
	rx_qbase = (struct rx_queue_desc *)client_shm->rx_qbase;
	for (i = 0; i < cnt; i++) {
		rx_queue = &client->rx_q[i];
		rx_queue->base = rx_qbase + i * client_shm->rx_qsize;
		rx_queue->size = client_shm->rx_qsize;
		rx_queue->write_idx = 0;
	}

	/* Initialize client Tx queues baseaddr, size */
	cnt = CLIENT_CTRL_TX_Q_CNT(client_shm->ctrl);

	/* Check if client is requesting for more queues than supported */
	if (cnt > HIF_CLIENT_QUEUES_MAX)
		cnt = HIF_CLIENT_QUEUES_MAX;

	client->tx_qn = cnt;
	tx_qbase = (struct tx_queue_desc *)client_shm->tx_qbase;
	for (i = 0; i < cnt; i++) {
		tx_queue = &client->tx_q[i];
		tx_queue->base = tx_qbase + i * client_shm->tx_qsize;
		tx_queue->size = client_shm->tx_qsize;
		tx_queue->ack_idx = 0;
	}

	set_bit(client_id, &hif->shm->g_client_status[0]);

unlock:
	spin_unlock_bh(&hif->tx_lock);

	return err;
}

/*
 * pfe_hif_client_unregister
 *
 * This function used to unregister a client  from the HIF driver.
 *
 */
static void pfe_hif_client_unregister(struct pfe_hif *hif, u32 client_id)
{
	pr_info("%s\n", __func__);

	/*
	 * Mark client as no longer available (which prevents further packet
	 * receive for this client)
	 */
	spin_lock_bh(&hif->tx_lock);

	if (!test_bit(client_id, &hif->shm->g_client_status[0])) {
		pr_err("%s: client %d not registered\n", __func__,
		       client_id);

		spin_unlock_bh(&hif->tx_lock);
		return;
	}

	clear_bit(client_id, &hif->shm->g_client_status[0]);

	spin_unlock_bh(&hif->tx_lock);
}

/*
 * client_put_rxpacket-
 * This functions puts the Rx pkt  in the given client Rx queue.
 * It actually swap the Rx pkt in the client Rx descriptor buffer
 * and returns the free buffer from it.
 *
 * If the function returns NULL means client Rx queue is full and
 * packet couldn't send to client queue.
 */
static void *client_put_rxpacket(struct hif_rx_queue *queue, void *pkt, u32 len,
				 u32 flags, u32 client_ctrl, u32 *rem_len)
{
	void *free_pkt = NULL;
	struct rx_queue_desc *desc = queue->base + queue->write_idx;

	if (readl(&desc->ctrl) & CL_DESC_OWN) {
		if (page_mode) {
			int rem_page_size = PAGE_SIZE -
					PRESENT_OFST_IN_PAGE(pkt);
			int cur_pkt_size = ROUND_MIN_RX_SIZE(len +
					pfe_pkt_headroom);
			*rem_len = (rem_page_size - cur_pkt_size);
			if (*rem_len) {
				free_pkt = pkt + cur_pkt_size;
				get_page(virt_to_page(free_pkt));
			} else {
				free_pkt = (void
				*)__get_free_page(GFP_ATOMIC | GFP_DMA_PFE);
				*rem_len = pfe_pkt_size;
			}
		} else {
			free_pkt = kmalloc(PFE_BUF_SIZE, GFP_ATOMIC |
					GFP_DMA_PFE);
			*rem_len = PFE_BUF_SIZE - pfe_pkt_headroom;
		}

		if (free_pkt) {
			desc->data = pkt;
			desc->client_ctrl = client_ctrl;
			/*
			 * Ensure everything else is written to DDR before
			 * writing bd->ctrl
			 */
			smp_wmb();
			writel(CL_DESC_BUF_LEN(len) | flags, &desc->ctrl);
			queue->write_idx = (queue->write_idx + 1)
					    & (queue->size - 1);

			free_pkt += pfe_pkt_headroom;
		}
	}

	return free_pkt;
}

/*
 * pfe_hif_rx_process-
 * This function does pfe hif rx queue processing.
 * Dequeue packet from Rx queue and send it to corresponding client queue
 */
static int pfe_hif_rx_process(struct pfe_hif *hif, int budget)
{
	struct hif_desc	*desc;
	struct hif_hdr *pkt_hdr;
	struct __hif_hdr hif_hdr;
	void *free_buf;
	int rtc, len, rx_processed = 0;
	struct __hif_desc local_desc;
	int flags;
	unsigned int desc_p;
	unsigned int buf_size = 0;

	spin_lock_bh(&hif->lock);

	rtc = hif->rxtoclean_index;

	while (rx_processed < budget) {
		desc = hif->rx_base + rtc;

		__memcpy12(&local_desc, desc);

		/* ACK pending Rx interrupt */
		if (local_desc.ctrl & BD_CTRL_DESC_EN) {
			writel(HIF_INT | HIF_RXPKT_INT, HIF_INT_SRC);

			if (rx_processed == 0) {
				if (napi_first_batch == 1) {
					desc_p = hif->descr_baseaddr_p +
					((unsigned long int)(desc) -
					(unsigned long
					int)hif->descr_baseaddr_v);
					napi_first_batch = 0;
				}
			}

			__memcpy12(&local_desc, desc);

			if (local_desc.ctrl & BD_CTRL_DESC_EN)
				break;
		}

		napi_first_batch = 0;

#ifdef HIF_NAPI_STATS
		hif->napi_counters[NAPI_DESC_COUNT]++;
#endif
		len = BD_BUF_LEN(local_desc.ctrl);
		/*
		 * dma_unmap_single(hif->dev, DDR_PFE_TO_PHYS(local_desc.data),
		 * hif->rx_buf_len[rtc], DMA_FROM_DEVICE);
		 */
		dma_unmap_single(hif->dev, DDR_PFE_TO_PHYS(local_desc.data),
				 hif->rx_buf_len[rtc], DMA_FROM_DEVICE);

		pkt_hdr = (struct hif_hdr *)hif->rx_buf_addr[rtc];

		/* Track last HIF header received */
		if (!hif->started) {
			hif->started = 1;

			__memcpy8(&hif_hdr, pkt_hdr);

			hif->qno = hif_hdr.hdr.q_num;
			hif->client_id = hif_hdr.hdr.client_id;
			hif->client_ctrl = (hif_hdr.hdr.client_ctrl1 << 16) |
						hif_hdr.hdr.client_ctrl;
			flags = CL_DESC_FIRST;

		} else {
			flags = 0;
		}

		if (local_desc.ctrl & BD_CTRL_LIFM)
			flags |= CL_DESC_LAST;

		/* Check for valid client id and still registered */
		if ((hif->client_id >= HIF_CLIENTS_MAX) ||
		    !(test_bit(hif->client_id,
			&hif->shm->g_client_status[0]))) {
			printk_ratelimited("%s: packet with invalid client id %d q_num %d\n",
					   __func__,
					   hif->client_id,
					   hif->qno);

			free_buf = pkt_hdr;

			goto pkt_drop;
		}

		/* Check to valid queue number */
		if (hif->client[hif->client_id].rx_qn <= hif->qno) {
			pr_info("%s: packet with invalid queue: %d\n"
				, __func__, hif->qno);
			hif->qno = 0;
		}

		free_buf =
		client_put_rxpacket(&hif->client[hif->client_id].rx_q[hif->qno],
				    (void *)pkt_hdr, len, flags,
			hif->client_ctrl, &buf_size);

		hif_lib_indicate_client(hif->client_id, EVENT_RX_PKT_IND,
					hif->qno);

		if (unlikely(!free_buf)) {
#ifdef HIF_NAPI_STATS
			hif->napi_counters[NAPI_CLIENT_FULL_COUNT]++;
#endif
			/*
			 * If we want to keep in polling mode to retry later,
			 * we need to tell napi that we consumed
			 * the full budget or we will hit a livelock scenario.
			 * The core code keeps this napi instance
			 * at the head of the list and none of the other
			 * instances get to run
			 */
			rx_processed = budget;

			if (flags & CL_DESC_FIRST)
				hif->started = 0;

			break;
		}

pkt_drop:
		/*Fill free buffer in the descriptor */
		hif->rx_buf_addr[rtc] = free_buf;
		hif->rx_buf_len[rtc] = min(pfe_pkt_size, buf_size);
		writel((DDR_PHYS_TO_PFE
			((u32)dma_map_single(hif->dev,
			free_buf, hif->rx_buf_len[rtc], DMA_FROM_DEVICE))),
			&desc->data);
		/*
		 * Ensure everything else is written to DDR before
		 * writing bd->ctrl
		 */
		wmb();
		writel((BD_CTRL_PKT_INT_EN | BD_CTRL_LIFM | BD_CTRL_DIR |
			BD_CTRL_DESC_EN | BD_BUF_LEN(hif->rx_buf_len[rtc])),
			&desc->ctrl);

		rtc = (rtc + 1) & (hif->rx_ring_size - 1);

		if (local_desc.ctrl & BD_CTRL_LIFM) {
			if (!(hif->client_ctrl & HIF_CTRL_RX_CONTINUED)) {
				rx_processed++;

#ifdef HIF_NAPI_STATS
				hif->napi_counters[NAPI_PACKET_COUNT]++;
#endif
			}
			hif->started = 0;
		}
	}

	hif->rxtoclean_index = rtc;
	spin_unlock_bh(&hif->lock);

	/* we made some progress, re-start rx dma in case it stopped */
	hif_rx_dma_start();

	return rx_processed;
}

/*
 * client_ack_txpacket-
 * This function ack the Tx packet in the give client Tx queue by resetting
 * ownership bit in the descriptor.
 */
static int client_ack_txpacket(struct pfe_hif *hif, unsigned int client_id,
			       unsigned int q_no)
{
	struct hif_tx_queue *queue = &hif->client[client_id].tx_q[q_no];
	struct tx_queue_desc *desc = queue->base + queue->ack_idx;

	if (readl(&desc->ctrl) & CL_DESC_OWN) {
		writel((readl(&desc->ctrl) & ~CL_DESC_OWN), &desc->ctrl);
		queue->ack_idx = (queue->ack_idx + 1) & (queue->size - 1);

		return 0;

	} else {
		/*This should not happen */
		pr_err("%s: %d %d %d %d %d %p %d\n", __func__,
		       hif->txtosend, hif->txtoclean, hif->txavail,
			client_id, q_no, queue, queue->ack_idx);
		WARN(1, "%s: doesn't own this descriptor", __func__);
		return 1;
	}
}

void __hif_tx_done_process(struct pfe_hif *hif, int count)
{
	struct hif_desc *desc;
	struct hif_desc_sw *desc_sw;
	int ttc, tx_avl;
	int pkts_done[HIF_CLIENTS_MAX] = {0, 0};

	ttc = hif->txtoclean;
	tx_avl = hif->txavail;

	while ((tx_avl < hif->tx_ring_size) && count--) {
		desc = hif->tx_base + ttc;

		if (readl(&desc->ctrl) & BD_CTRL_DESC_EN)
			break;

		desc_sw = &hif->tx_sw_queue[ttc];

		if (desc_sw->data) {
			/*
			 * dmap_unmap_single(hif->dev, desc_sw->data,
			 * desc_sw->len, DMA_TO_DEVICE);
			 */
			dma_unmap_single(hif->dev, desc_sw->data,
					 desc_sw->len, DMA_TO_DEVICE);
		}

		if (desc_sw->client_id >= HIF_CLIENTS_MAX) {
			pr_err("Invalid cl id %d\n", desc_sw->client_id);
			break;
		}

		pkts_done[desc_sw->client_id]++;

		client_ack_txpacket(hif, desc_sw->client_id, desc_sw->q_no);

		ttc = (ttc + 1) & (hif->tx_ring_size - 1);
		tx_avl++;
	}

	if (pkts_done[0])
		hif_lib_indicate_client(0, EVENT_TXDONE_IND, 0);
	if (pkts_done[1])
		hif_lib_indicate_client(1, EVENT_TXDONE_IND, 0);

	hif->txtoclean = ttc;
	hif->txavail = tx_avl;

	if (!count) {
		tasklet_schedule(&hif->tx_cleanup_tasklet);
	} else {
		/*Enable Tx done interrupt */
		writel(readl_relaxed(HIF_INT_ENABLE) | HIF_TXPKT_INT,
		       HIF_INT_ENABLE);
	}
}

static void pfe_tx_do_cleanup(unsigned long data)
{
	struct pfe_hif *hif = (struct pfe_hif *)data;

	writel(HIF_INT | HIF_TXPKT_INT, HIF_INT_SRC);

	hif_tx_done_process(hif, 64);
}

/*
 * __hif_xmit_pkt -
 * This function puts one packet in the HIF Tx queue
 */
void __hif_xmit_pkt(struct pfe_hif *hif, unsigned int client_id, unsigned int
			q_no, void *data, u32 len, unsigned int flags)
{
	struct hif_desc	*desc;
	struct hif_desc_sw *desc_sw;

	desc = hif->tx_base + hif->txtosend;
	desc_sw = &hif->tx_sw_queue[hif->txtosend];

	desc_sw->len = len;
	desc_sw->client_id = client_id;
	desc_sw->q_no = q_no;
	desc_sw->flags = flags;

	if (flags & HIF_DONT_DMA_MAP) {
		desc_sw->data = 0;
		writel((u32)DDR_PHYS_TO_PFE(data), &desc->data);
	} else {
		desc_sw->data = dma_map_single(hif->dev, data, len,
						DMA_TO_DEVICE);
		writel((u32)DDR_PHYS_TO_PFE(desc_sw->data), &desc->data);
	}

	hif->txtosend = (hif->txtosend + 1) & (hif->tx_ring_size - 1);
	hif->txavail--;

	if ((!((flags & HIF_DATA_VALID) && (flags &
				HIF_LAST_BUFFER))))
		goto skip_tx;

	/*
	 * Ensure everything else is written to DDR before
	 * writing bd->ctrl
	 */
	wmb();

	do {
		desc_sw = &hif->tx_sw_queue[hif->txtoflush];
		desc = hif->tx_base + hif->txtoflush;

		if (desc_sw->flags & HIF_LAST_BUFFER) {
			writel((BD_CTRL_LIFM |
			       BD_CTRL_BRFETCH_DISABLE | BD_CTRL_RTFETCH_DISABLE
			       | BD_CTRL_PARSE_DISABLE | BD_CTRL_DESC_EN |
				BD_CTRL_PKT_INT_EN | BD_BUF_LEN(desc_sw->len)),
				&desc->ctrl);
		} else {
			writel((BD_CTRL_DESC_EN |
				BD_BUF_LEN(desc_sw->len)), &desc->ctrl);
		}
		hif->txtoflush = (hif->txtoflush + 1) & (hif->tx_ring_size - 1);
	}
	while (hif->txtoflush != hif->txtosend)
		;

skip_tx:
	return;
}

static irqreturn_t wol_isr(int irq, void *dev_id)
{
	pr_info("WoL\n");
	gemac_set_wol(EMAC1_BASE_ADDR, 0);
	gemac_set_wol(EMAC2_BASE_ADDR, 0);
	return IRQ_HANDLED;
}

/*
 * hif_isr-
 * This ISR routine processes Rx/Tx done interrupts from the HIF hardware block
 */
static irqreturn_t hif_isr(int irq, void *dev_id)
{
	struct pfe_hif *hif = (struct pfe_hif *)dev_id;
	int int_status;
	int int_enable_mask;

	/*Read hif interrupt source register */
	int_status = readl_relaxed(HIF_INT_SRC);
	int_enable_mask = readl_relaxed(HIF_INT_ENABLE);

	if ((int_status & HIF_INT) == 0)
		return IRQ_NONE;

	int_status &= ~(HIF_INT);

	if (int_status & HIF_RXPKT_INT) {
		int_status &= ~(HIF_RXPKT_INT);
		int_enable_mask &= ~(HIF_RXPKT_INT);

		napi_first_batch = 1;

		if (napi_schedule_prep(&hif->napi)) {
#ifdef HIF_NAPI_STATS
			hif->napi_counters[NAPI_SCHED_COUNT]++;
#endif
			__napi_schedule(&hif->napi);
		}
	}

	if (int_status & HIF_TXPKT_INT) {
		int_status &= ~(HIF_TXPKT_INT);
		int_enable_mask &= ~(HIF_TXPKT_INT);
		/*Schedule tx cleanup tassklet */
		tasklet_schedule(&hif->tx_cleanup_tasklet);
	}

	/*Disable interrupts, they will be enabled after they are serviced */
	writel_relaxed(int_enable_mask, HIF_INT_ENABLE);

	if (int_status) {
		pr_info("%s : Invalid interrupt : %d\n", __func__,
			int_status);
		writel(int_status, HIF_INT_SRC);
	}

	return IRQ_HANDLED;
}

void hif_process_client_req(struct pfe_hif *hif, int req, int data1, int data2)
{
	unsigned int client_id = data1;

	if (client_id >= HIF_CLIENTS_MAX) {
		pr_err("%s: client id %d out of bounds\n", __func__,
		       client_id);
		return;
	}

	switch (req) {
	case REQUEST_CL_REGISTER:
			/* Request for register a client */
			pr_info("%s: register client_id %d\n",
				__func__, client_id);
			pfe_hif_client_register(hif, client_id, (struct
				hif_client_shm *)&hif->shm->client[client_id]);
			break;

	case REQUEST_CL_UNREGISTER:
			pr_info("%s: unregister client_id %d\n",
				__func__, client_id);

			/* Request for unregister a client */
			pfe_hif_client_unregister(hif, client_id);

			break;

	default:
			pr_err("%s: unsupported request %d\n",
			       __func__, req);
			break;
	}

	/*
	 * Process client Tx queues
	 * Currently we don't have checking for tx pending
	 */
}

/*
 * pfe_hif_rx_poll
 *  This function is NAPI poll function to process HIF Rx queue.
 */
static int pfe_hif_rx_poll(struct napi_struct *napi, int budget)
{
	struct pfe_hif *hif = container_of(napi, struct pfe_hif, napi);
	int work_done;

#ifdef HIF_NAPI_STATS
	hif->napi_counters[NAPI_POLL_COUNT]++;
#endif

	work_done = pfe_hif_rx_process(hif, budget);

	if (work_done < budget) {
		napi_complete(napi);
		writel(readl_relaxed(HIF_INT_ENABLE) | HIF_RXPKT_INT,
		       HIF_INT_ENABLE);
	}
#ifdef HIF_NAPI_STATS
	else
		hif->napi_counters[NAPI_FULL_BUDGET_COUNT]++;
#endif

	return work_done;
}

/*
 * pfe_hif_init
 * This function initializes the baseaddresses and irq, etc.
 */
int pfe_hif_init(struct pfe *pfe)
{
	struct pfe_hif *hif = &pfe->hif;
	int err;

	pr_info("%s\n", __func__);

	hif->dev = pfe->dev;
	hif->irq = pfe->hif_irq;

	err = pfe_hif_alloc_descr(hif);
	if (err)
		goto err0;

	if (pfe_hif_init_buffers(hif)) {
		pr_err("%s: Could not initialize buffer descriptors\n"
			, __func__);
		err = -ENOMEM;
		goto err1;
	}

	/* Initialize NAPI for Rx processing */
	init_dummy_netdev(&hif->dummy_dev);
	netif_napi_add(&hif->dummy_dev, &hif->napi, pfe_hif_rx_poll);
	napi_enable(&hif->napi);

	spin_lock_init(&hif->tx_lock);
	spin_lock_init(&hif->lock);

	hif_init();
	hif_rx_enable();
	hif_tx_enable();

	/* Disable tx done interrupt */
	writel(HIF_INT_MASK, HIF_INT_ENABLE);

	gpi_enable(HGPI_BASE_ADDR);

	err = request_irq(hif->irq, hif_isr, 0, "pfe_hif", hif);
	if (err) {
		pr_err("%s: failed to get the hif IRQ = %d\n",
		       __func__, hif->irq);
		goto err1;
	}

	err = request_irq(pfe->wol_irq, wol_isr, 0, "pfe_wol", pfe);
	if (err) {
		pr_err("%s: failed to get the wol IRQ = %d\n",
		       __func__, pfe->wol_irq);
		goto err1;
	}

	tasklet_init(&hif->tx_cleanup_tasklet,
		     (void(*)(unsigned long))pfe_tx_do_cleanup,
		     (unsigned long)hif);

	return 0;
err1:
	pfe_hif_free_descr(hif);
err0:
	return err;
}

/* pfe_hif_exit- */
void pfe_hif_exit(struct pfe *pfe)
{
	struct pfe_hif *hif = &pfe->hif;

	pr_info("%s\n", __func__);

	tasklet_kill(&hif->tx_cleanup_tasklet);

	spin_lock_bh(&hif->lock);
	hif->shm->g_client_status[0] = 0;
	/* Make sure all clients are disabled*/
	hif->shm->g_client_status[1] = 0;

	spin_unlock_bh(&hif->lock);

	/*Disable Rx/Tx */
	gpi_disable(HGPI_BASE_ADDR);
	hif_rx_disable();
	hif_tx_disable();

	napi_disable(&hif->napi);
	netif_napi_del(&hif->napi);

	free_irq(pfe->wol_irq, pfe);
	free_irq(hif->irq, hif);

	pfe_hif_release_buffers(hif);
	pfe_hif_free_descr(hif);
}
