// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/moduleparam.h>
#include <linux/cpu.h>

#include "pfe_mod.h"
#include "pfe_hif.h"
#include "pfe_hif_lib.h"

unsigned int lro_mode;
unsigned int page_mode;
unsigned int tx_qos = 1;
module_param(tx_qos, uint, 0444);
MODULE_PARM_DESC(tx_qos, "0: disable ,\n"
			 "1: enable (default), guarantee no packet drop at TMU level\n");
unsigned int pfe_pkt_size;
unsigned int pfe_pkt_headroom;
unsigned int emac_txq_cnt;

/*
 * @pfe_hal_lib.c.
 * Common functions used by HIF client drivers
 */

/*HIF shared memory Global variable */
struct hif_shm ghif_shm;

/* Cleanup the HIF shared memory, release HIF rx_buffer_pool.
 * This function should be called after pfe_hif_exit
 *
 * @param[in] hif_shm		Shared memory address location in DDR
 */
static void pfe_hif_shm_clean(struct hif_shm *hif_shm)
{
	int i;
	void *pkt;

	for (i = 0; i < hif_shm->rx_buf_pool_cnt; i++) {
		pkt = hif_shm->rx_buf_pool[i];
		if (pkt) {
			hif_shm->rx_buf_pool[i] = NULL;
			pkt -= pfe_pkt_headroom;

			if (page_mode)
				put_page(virt_to_page(pkt));
			else
				kfree(pkt);
		}
	}
}

/* Initialize shared memory used between HIF driver and clients,
 * allocate rx_buffer_pool required for HIF Rx descriptors.
 * This function should be called before initializing HIF driver.
 *
 * @param[in] hif_shm		Shared memory address location in DDR
 * @rerurn			0 - on succes, <0 on fail to initialize
 */
static int pfe_hif_shm_init(struct hif_shm *hif_shm)
{
	int i;
	void *pkt;

	memset(hif_shm, 0, sizeof(struct hif_shm));
	hif_shm->rx_buf_pool_cnt = HIF_RX_DESC_NT;

	for (i = 0; i < hif_shm->rx_buf_pool_cnt; i++) {
		if (page_mode) {
			pkt = (void *)__get_free_page(GFP_KERNEL |
				GFP_DMA_PFE);
		} else {
			pkt = kmalloc(PFE_BUF_SIZE, GFP_KERNEL | GFP_DMA_PFE);
		}

		if (pkt)
			hif_shm->rx_buf_pool[i] = pkt + pfe_pkt_headroom;
		else
			goto err0;
	}

	return 0;

err0:
	pr_err("%s Low memory\n", __func__);
	pfe_hif_shm_clean(hif_shm);
	return -ENOMEM;
}

/*This function sends indication to HIF driver
 *
 * @param[in] hif	hif context
 */
static void hif_lib_indicate_hif(struct pfe_hif *hif, int req, int data1, int
					data2)
{
	hif_process_client_req(hif, req, data1, data2);
}

void hif_lib_indicate_client(int client_id, int event_type, int qno)
{
	struct hif_client_s *client = pfe->hif_client[client_id];

	if (!client || (event_type >= HIF_EVENT_MAX) || (qno >=
		HIF_CLIENT_QUEUES_MAX))
		return;

	if (!test_and_set_bit(qno, &client->queue_mask[event_type]))
		client->event_handler(client->priv, event_type, qno);
}

/*This function releases Rx queue descriptors memory and pre-filled buffers
 *
 * @param[in] client	hif_client context
 */
static void hif_lib_client_release_rx_buffers(struct hif_client_s *client)
{
	struct rx_queue_desc *desc;
	int qno, ii;
	void *buf;

	for (qno = 0; qno < client->rx_qn; qno++) {
		desc = client->rx_q[qno].base;

		for (ii = 0; ii < client->rx_q[qno].size; ii++) {
			buf = (void *)desc->data;
			if (buf) {
				buf -= pfe_pkt_headroom;

				if (page_mode)
					free_page((unsigned long)buf);
				else
					kfree(buf);

				desc->ctrl = 0;
			}

			desc++;
		}
	}

	kfree(client->rx_qbase);
}

/*This function allocates memory for the rxq descriptors and pre-fill rx queues
 * with buffers.
 * @param[in] client	client context
 * @param[in] q_size	size of the rxQ, all queues are of same size
 */
static int hif_lib_client_init_rx_buffers(struct hif_client_s *client, int
						q_size)
{
	struct rx_queue_desc *desc;
	struct hif_client_rx_queue *queue;
	int ii, qno;

	/*Allocate memory for the client queues */
	client->rx_qbase = kzalloc(client->rx_qn * q_size * sizeof(struct
				rx_queue_desc), GFP_KERNEL);
	if (!client->rx_qbase)
		goto err;

	for (qno = 0; qno < client->rx_qn; qno++) {
		queue = &client->rx_q[qno];

		queue->base = client->rx_qbase + qno * q_size * sizeof(struct
				rx_queue_desc);
		queue->size = q_size;
		queue->read_idx = 0;
		queue->write_idx = 0;

		pr_debug("rx queue: %d, base: %p, size: %d\n", qno,
			 queue->base, queue->size);
	}

	for (qno = 0; qno < client->rx_qn; qno++) {
		queue = &client->rx_q[qno];
		desc = queue->base;

		for (ii = 0; ii < queue->size; ii++) {
			desc->ctrl = CL_DESC_BUF_LEN(pfe_pkt_size) |
					CL_DESC_OWN;
			desc++;
		}
	}

	return 0;

err:
	return 1;
}


static void hif_lib_client_cleanup_tx_queue(struct hif_client_tx_queue *queue)
{
	pr_debug("%s\n", __func__);

	/*
	 * Check if there are any pending packets. Client must flush the tx
	 * queues before unregistering, by calling by calling
	 * hif_lib_tx_get_next_complete()
	 *
	 * Hif no longer calls since we are no longer registered
	 */
	if (queue->tx_pending)
		pr_err("%s: pending transmit packets\n", __func__);
}

static void hif_lib_client_release_tx_buffers(struct hif_client_s *client)
{
	int qno;

	pr_debug("%s\n", __func__);

	for (qno = 0; qno < client->tx_qn; qno++)
		hif_lib_client_cleanup_tx_queue(&client->tx_q[qno]);

	kfree(client->tx_qbase);
}

static int hif_lib_client_init_tx_buffers(struct hif_client_s *client, int
						q_size)
{
	struct hif_client_tx_queue *queue;
	int qno;

	client->tx_qbase = kzalloc(client->tx_qn * q_size * sizeof(struct
					tx_queue_desc), GFP_KERNEL);
	if (!client->tx_qbase)
		return 1;

	for (qno = 0; qno < client->tx_qn; qno++) {
		queue = &client->tx_q[qno];

		queue->base = client->tx_qbase + qno * q_size * sizeof(struct
				tx_queue_desc);
		queue->size = q_size;
		queue->read_idx = 0;
		queue->write_idx = 0;
		queue->tx_pending = 0;
		queue->nocpy_flag = 0;
		queue->prev_tmu_tx_pkts = 0;
		queue->done_tmu_tx_pkts = 0;

		pr_debug("tx queue: %d, base: %p, size: %d\n", qno,
			 queue->base, queue->size);
	}

	return 0;
}

static int hif_lib_event_dummy(void *priv, int event_type, int qno)
{
	return 0;
}

int hif_lib_client_register(struct hif_client_s *client)
{
	struct hif_shm *hif_shm;
	struct hif_client_shm *client_shm;
	int err, i;
	/* int loop_cnt = 0; */

	pr_debug("%s\n", __func__);

	/*Allocate memory before spin_lock*/
	if (hif_lib_client_init_rx_buffers(client, client->rx_qsize)) {
		err = -ENOMEM;
		goto err_rx;
	}

	if (hif_lib_client_init_tx_buffers(client, client->tx_qsize)) {
		err = -ENOMEM;
		goto err_tx;
	}

	spin_lock_bh(&pfe->hif.lock);
	if (!(client->pfe) || (client->id >= HIF_CLIENTS_MAX) ||
	    (pfe->hif_client[client->id])) {
		err = -EINVAL;
		goto err;
	}

	hif_shm = client->pfe->hif.shm;

	if (!client->event_handler)
		client->event_handler = hif_lib_event_dummy;

	/*Initialize client specific shared memory */
	client_shm = (struct hif_client_shm *)&hif_shm->client[client->id];
	client_shm->rx_qbase = (unsigned long int)client->rx_qbase;
	client_shm->rx_qsize = client->rx_qsize;
	client_shm->tx_qbase = (unsigned long int)client->tx_qbase;
	client_shm->tx_qsize = client->tx_qsize;
	client_shm->ctrl = (client->tx_qn << CLIENT_CTRL_TX_Q_CNT_OFST) |
				(client->rx_qn << CLIENT_CTRL_RX_Q_CNT_OFST);
	/* spin_lock_init(&client->rx_lock); */

	for (i = 0; i < HIF_EVENT_MAX; i++) {
		client->queue_mask[i] = 0;  /*
					     * By default all events are
					     * unmasked
					     */
	}

	/*Indicate to HIF driver*/
	hif_lib_indicate_hif(&pfe->hif, REQUEST_CL_REGISTER, client->id, 0);

	pr_debug("%s: client: %p, client_id: %d, tx_qsize: %d, rx_qsize: %d\n",
		 __func__, client, client->id, client->tx_qsize,
		 client->rx_qsize);

	client->cpu_id = -1;

	pfe->hif_client[client->id] = client;
	spin_unlock_bh(&pfe->hif.lock);

	return 0;

err:
	spin_unlock_bh(&pfe->hif.lock);
	hif_lib_client_release_tx_buffers(client);

err_tx:
	hif_lib_client_release_rx_buffers(client);

err_rx:
	return err;
}

int hif_lib_client_unregister(struct hif_client_s *client)
{
	struct pfe *pfe = client->pfe;
	u32 client_id = client->id;

	pr_info(
		"%s : client: %p, client_id: %d, txQ_depth: %d, rxQ_depth: %d\n"
		, __func__, client, client->id, client->tx_qsize,
		client->rx_qsize);

	spin_lock_bh(&pfe->hif.lock);
	hif_lib_indicate_hif(&pfe->hif, REQUEST_CL_UNREGISTER, client->id, 0);

	hif_lib_client_release_tx_buffers(client);
	hif_lib_client_release_rx_buffers(client);
	pfe->hif_client[client_id] = NULL;
	spin_unlock_bh(&pfe->hif.lock);

	return 0;
}

int hif_lib_event_handler_start(struct hif_client_s *client, int event,
				int qno)
{
	struct hif_client_rx_queue *queue = &client->rx_q[qno];
	struct rx_queue_desc *desc = queue->base + queue->read_idx;

	if ((event >= HIF_EVENT_MAX) || (qno >= HIF_CLIENT_QUEUES_MAX)) {
		pr_debug("%s: Unsupported event : %d  queue number : %d\n",
			 __func__, event, qno);
		return -1;
	}

	test_and_clear_bit(qno, &client->queue_mask[event]);

	switch (event) {
	case EVENT_RX_PKT_IND:
		if (!(desc->ctrl & CL_DESC_OWN))
			hif_lib_indicate_client(client->id,
						EVENT_RX_PKT_IND, qno);
		break;

	case EVENT_HIGH_RX_WM:
	case EVENT_TXDONE_IND:
	default:
		break;
	}

	return 0;
}

/*
 * This function gets one packet from the specified client queue
 * It also refill the rx buffer
 */
void *hif_lib_receive_pkt(struct hif_client_s *client, int qno, int *len, int
				*ofst, unsigned int *rx_ctrl,
				unsigned int *desc_ctrl, void **priv_data)
{
	struct hif_client_rx_queue *queue = &client->rx_q[qno];
	struct rx_queue_desc *desc;
	void *pkt = NULL;

	/*
	 * Following lock is to protect rx queue access from,
	 * hif_lib_event_handler_start.
	 * In general below lock is not required, because hif_lib_xmit_pkt and
	 * hif_lib_event_handler_start are called from napi poll and which is
	 * not re-entrant. But if some client use in different way this lock is
	 * required.
	 */
	/*spin_lock_irqsave(&client->rx_lock, flags); */
	desc = queue->base + queue->read_idx;
	if (!(desc->ctrl & CL_DESC_OWN)) {
		pkt = desc->data - pfe_pkt_headroom;

		*rx_ctrl = desc->client_ctrl;
		*desc_ctrl = desc->ctrl;

		if (desc->ctrl & CL_DESC_FIRST) {
			u16 size = *rx_ctrl >> HIF_CTRL_RX_OFFSET_OFST;

			if (size) {
				size += PFE_PARSE_INFO_SIZE;
				*len = CL_DESC_BUF_LEN(desc->ctrl) -
						PFE_PKT_HEADER_SZ - size;
				*ofst = pfe_pkt_headroom + PFE_PKT_HEADER_SZ
								+ size;
				*priv_data = desc->data + PFE_PKT_HEADER_SZ;
			} else {
				*len = CL_DESC_BUF_LEN(desc->ctrl) -
				       PFE_PKT_HEADER_SZ - PFE_PARSE_INFO_SIZE;
				*ofst = pfe_pkt_headroom
					+ PFE_PKT_HEADER_SZ
					+ PFE_PARSE_INFO_SIZE;
				*priv_data = NULL;
			}

		} else {
			*len = CL_DESC_BUF_LEN(desc->ctrl);
			*ofst = pfe_pkt_headroom;
		}

		/*
		 * Needed so we don't free a buffer/page
		 * twice on module_exit
		 */
		desc->data = NULL;

		/*
		 * Ensure everything else is written to DDR before
		 * writing bd->ctrl
		 */
		smp_wmb();

		desc->ctrl = CL_DESC_BUF_LEN(pfe_pkt_size) | CL_DESC_OWN;
		queue->read_idx = (queue->read_idx + 1) & (queue->size - 1);
	}

	/*spin_unlock_irqrestore(&client->rx_lock, flags); */
	return pkt;
}

static inline void hif_hdr_write(struct hif_hdr *pkt_hdr, unsigned int
					client_id, unsigned int qno,
					u32 client_ctrl)
{
	/* Optimize the write since the destinaton may be non-cacheable */
	if (!((unsigned long)pkt_hdr & 0x3)) {
		((u32 *)pkt_hdr)[0] = (client_ctrl << 16) | (qno << 8) |
					client_id;
	} else {
		((u16 *)pkt_hdr)[0] = (qno << 8) | (client_id & 0xFF);
		((u16 *)pkt_hdr)[1] = (client_ctrl & 0xFFFF);
	}
}

/*This function puts the given packet in the specific client queue */
void __hif_lib_xmit_pkt(struct hif_client_s *client, unsigned int qno, void
				*data, unsigned int len, u32 client_ctrl,
				unsigned int flags, void *client_data)
{
	struct hif_client_tx_queue *queue = &client->tx_q[qno];
	struct tx_queue_desc *desc = queue->base + queue->write_idx;

	/* First buffer */
	if (flags & HIF_FIRST_BUFFER) {
		data -= sizeof(struct hif_hdr);
		len += sizeof(struct hif_hdr);

		hif_hdr_write(data, client->id, qno, client_ctrl);
	}

	desc->data = client_data;
	desc->ctrl = CL_DESC_OWN | CL_DESC_FLAGS(flags);

	__hif_xmit_pkt(&pfe->hif, client->id, qno, data, len, flags);

	queue->write_idx = (queue->write_idx + 1) & (queue->size - 1);
	queue->tx_pending++;
	queue->jiffies_last_packet = jiffies;
}

void *hif_lib_tx_get_next_complete(struct hif_client_s *client, int qno,
				   unsigned int *flags, int count)
{
	struct hif_client_tx_queue *queue = &client->tx_q[qno];
	struct tx_queue_desc *desc = queue->base + queue->read_idx;

	pr_debug("%s: qno : %d rd_indx: %d pending:%d\n", __func__, qno,
		 queue->read_idx, queue->tx_pending);

	if (!queue->tx_pending)
		return NULL;

	if (queue->nocpy_flag && !queue->done_tmu_tx_pkts) {
		u32 tmu_tx_pkts = be32_to_cpu(pe_dmem_read(TMU0_ID +
			client->id, TMU_DM_TX_TRANS, 4));

		if (queue->prev_tmu_tx_pkts > tmu_tx_pkts)
			queue->done_tmu_tx_pkts = UINT_MAX -
				queue->prev_tmu_tx_pkts + tmu_tx_pkts;
		else
			queue->done_tmu_tx_pkts = tmu_tx_pkts -
						queue->prev_tmu_tx_pkts;

		queue->prev_tmu_tx_pkts  = tmu_tx_pkts;

		if (!queue->done_tmu_tx_pkts)
			return NULL;
	}

	if (desc->ctrl & CL_DESC_OWN)
		return NULL;

	queue->read_idx = (queue->read_idx + 1) & (queue->size - 1);
	queue->tx_pending--;

	*flags = CL_DESC_GET_FLAGS(desc->ctrl);

	if (queue->done_tmu_tx_pkts && (*flags & HIF_LAST_BUFFER))
		queue->done_tmu_tx_pkts--;

	return desc->data;
}

static void hif_lib_tmu_credit_init(struct pfe *pfe)
{
	int i, q;

	for (i = 0; i < NUM_GEMAC_SUPPORT; i++)
		for (q = 0; q < emac_txq_cnt; q++) {
			pfe->tmu_credit.tx_credit_max[i][q] = (q == 0) ?
					DEFAULT_Q0_QDEPTH : DEFAULT_MAX_QDEPTH;
			pfe->tmu_credit.tx_credit[i][q] =
					pfe->tmu_credit.tx_credit_max[i][q];
		}
}

/* __hif_lib_update_credit
 *
 * @param[in] client	hif client context
 * @param[in] queue	queue number in match with TMU
 */
void __hif_lib_update_credit(struct hif_client_s *client, unsigned int queue)
{
	unsigned int tmu_tx_packets, tmp;

	if (tx_qos) {
		tmu_tx_packets = be32_to_cpu(pe_dmem_read(TMU0_ID +
			client->id, (TMU_DM_TX_TRANS + (queue * 4)), 4));

		/* tx_packets counter overflowed */
		if (tmu_tx_packets >
		    pfe->tmu_credit.tx_packets[client->id][queue]) {
			tmp = UINT_MAX - tmu_tx_packets +
			pfe->tmu_credit.tx_packets[client->id][queue];

			pfe->tmu_credit.tx_credit[client->id][queue] =
			pfe->tmu_credit.tx_credit_max[client->id][queue] - tmp;
		} else {
		/* TMU tx <= pfe_eth tx, normal case or both OF since
		 * last time
		 */
			pfe->tmu_credit.tx_credit[client->id][queue] =
			pfe->tmu_credit.tx_credit_max[client->id][queue] -
			(pfe->tmu_credit.tx_packets[client->id][queue] -
			tmu_tx_packets);
		}
	}
}

int pfe_hif_lib_init(struct pfe *pfe)
{
	int rc;

	pr_info("%s\n", __func__);

	if (lro_mode) {
		page_mode = 1;
		pfe_pkt_size = min(PAGE_SIZE, MAX_PFE_PKT_SIZE);
		pfe_pkt_headroom = 0;
	} else {
		page_mode = 0;
		pfe_pkt_size = PFE_PKT_SIZE;
		pfe_pkt_headroom = PFE_PKT_HEADROOM;
	}

	if (tx_qos)
		emac_txq_cnt = EMAC_TXQ_CNT / 2;
	else
		emac_txq_cnt = EMAC_TXQ_CNT;

	hif_lib_tmu_credit_init(pfe);
	pfe->hif.shm = &ghif_shm;
	rc = pfe_hif_shm_init(pfe->hif.shm);

	return rc;
}

void pfe_hif_lib_exit(struct pfe *pfe)
{
	pr_info("%s\n", __func__);

	pfe_hif_shm_clean(pfe->hif.shm);
}
