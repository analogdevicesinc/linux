// SPDX-License-Identifier: GPL-2.0

/* Texas Instruments ICSSG Ethernet Driver
 *
 * Copyright (C) 2018-2022 Texas Instruments Incorporated - https://www.ti.com/
 * Copyright (C) Siemens AG, 2024
 *
 */

#include <linux/dma-mapping.h>
#include <linux/dma/ti-cppi5.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/remoteproc/pruss.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>

#include "icssg_prueth.h"
#include "../k3-cppi-desc-pool.h"

/* Netif debug messages possible */
#define PRUETH_EMAC_DEBUG       (NETIF_MSG_DRV | \
				 NETIF_MSG_PROBE | \
				 NETIF_MSG_LINK | \
				 NETIF_MSG_TIMER | \
				 NETIF_MSG_IFDOWN | \
				 NETIF_MSG_IFUP | \
				 NETIF_MSG_RX_ERR | \
				 NETIF_MSG_TX_ERR | \
				 NETIF_MSG_TX_QUEUED | \
				 NETIF_MSG_INTR | \
				 NETIF_MSG_TX_DONE | \
				 NETIF_MSG_RX_STATUS | \
				 NETIF_MSG_PKTDATA | \
				 NETIF_MSG_HW | \
				 NETIF_MSG_WOL)

#define prueth_napi_to_emac(napi) container_of(napi, struct prueth_emac, napi_rx)

void prueth_cleanup_rx_chns(struct prueth_emac *emac,
			    struct prueth_rx_chn *rx_chn,
			    int max_rflows)
{
	if (rx_chn->desc_pool)
		k3_cppi_desc_pool_destroy(rx_chn->desc_pool);

	if (rx_chn->rx_chn)
		k3_udma_glue_release_rx_chn(rx_chn->rx_chn);
}

void prueth_cleanup_tx_chns(struct prueth_emac *emac)
{
	int i;

	for (i = 0; i < emac->tx_ch_num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		if (tx_chn->desc_pool)
			k3_cppi_desc_pool_destroy(tx_chn->desc_pool);

		if (tx_chn->tx_chn)
			k3_udma_glue_release_tx_chn(tx_chn->tx_chn);

		/* Assume prueth_cleanup_tx_chns() is called at the
		 * end after all channel resources are freed
		 */
		memset(tx_chn, 0, sizeof(*tx_chn));
	}
}

void prueth_ndev_del_tx_napi(struct prueth_emac *emac, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		if (tx_chn->irq)
			free_irq(tx_chn->irq, tx_chn);
		netif_napi_del(&tx_chn->napi_tx);
	}
}

void prueth_xmit_free(struct prueth_tx_chn *tx_chn,
		      struct cppi5_host_desc_t *desc)
{
	struct cppi5_host_desc_t *first_desc, *next_desc;
	dma_addr_t buf_dma, next_desc_dma;
	u32 buf_dma_len;

	first_desc = desc;
	next_desc = first_desc;

	cppi5_hdesc_get_obuf(first_desc, &buf_dma, &buf_dma_len);
	k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &buf_dma);

	dma_unmap_single(tx_chn->dma_dev, buf_dma, buf_dma_len,
			 DMA_TO_DEVICE);

	next_desc_dma = cppi5_hdesc_get_next_hbdesc(first_desc);
	k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &next_desc_dma);
	while (next_desc_dma) {
		next_desc = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool,
						       next_desc_dma);
		cppi5_hdesc_get_obuf(next_desc, &buf_dma, &buf_dma_len);
		k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &buf_dma);

		dma_unmap_page(tx_chn->dma_dev, buf_dma, buf_dma_len,
			       DMA_TO_DEVICE);

		next_desc_dma = cppi5_hdesc_get_next_hbdesc(next_desc);
		k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &next_desc_dma);

		k3_cppi_desc_pool_free(tx_chn->desc_pool, next_desc);
	}

	k3_cppi_desc_pool_free(tx_chn->desc_pool, first_desc);
}

int emac_tx_complete_packets(struct prueth_emac *emac, int chn,
			     int budget, bool *tdown)
{
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_tx;
	struct netdev_queue *netif_txq;
	struct prueth_tx_chn *tx_chn;
	unsigned int total_bytes = 0;
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int res, num_tx = 0;
	void **swdata;

	tx_chn = &emac->tx_chns[chn];

	while (true) {
		res = k3_udma_glue_pop_tx_chn(tx_chn->tx_chn, &desc_dma);
		if (res == -ENODATA)
			break;

		/* teardown completion */
		if (cppi5_desc_is_tdcm(desc_dma)) {
			if (atomic_dec_and_test(&emac->tdown_cnt))
				complete(&emac->tdown_complete);
			*tdown = true;
			break;
		}

		desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool,
						     desc_dma);
		swdata = cppi5_hdesc_get_swdata(desc_tx);

		/* was this command's TX complete? */
		if (emac->is_sr1 && *(swdata) == emac->cmd_data) {
			prueth_xmit_free(tx_chn, desc_tx);
			continue;
		}

		skb = *(swdata);
		prueth_xmit_free(tx_chn, desc_tx);

		ndev = skb->dev;
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
		total_bytes += skb->len;
		napi_consume_skb(skb, budget);
		num_tx++;
	}

	if (!num_tx)
		return 0;

	netif_txq = netdev_get_tx_queue(ndev, chn);
	netdev_tx_completed_queue(netif_txq, num_tx, total_bytes);

	if (netif_tx_queue_stopped(netif_txq)) {
		/* If the TX queue was stopped, wake it now
		 * if we have enough room.
		 */
		__netif_tx_lock(netif_txq, smp_processor_id());
		if (netif_running(ndev) &&
		    (k3_cppi_desc_pool_avail(tx_chn->desc_pool) >=
		     MAX_SKB_FRAGS))
			netif_tx_wake_queue(netif_txq);
		__netif_tx_unlock(netif_txq);
	}

	return num_tx;
}

static enum hrtimer_restart emac_tx_timer_callback(struct hrtimer *timer)
{
	struct prueth_tx_chn *tx_chns =
			container_of(timer, struct prueth_tx_chn, tx_hrtimer);

	enable_irq(tx_chns->irq);
	return HRTIMER_NORESTART;
}

static int emac_napi_tx_poll(struct napi_struct *napi_tx, int budget)
{
	struct prueth_tx_chn *tx_chn = prueth_napi_to_tx_chn(napi_tx);
	struct prueth_emac *emac = tx_chn->emac;
	bool tdown = false;
	int num_tx_packets;

	num_tx_packets = emac_tx_complete_packets(emac, tx_chn->id, budget,
						  &tdown);

	if (num_tx_packets >= budget)
		return budget;

	if (napi_complete_done(napi_tx, num_tx_packets)) {
		if (unlikely(tx_chn->tx_pace_timeout_ns && !tdown)) {
			hrtimer_start(&tx_chn->tx_hrtimer,
				      ns_to_ktime(tx_chn->tx_pace_timeout_ns),
				      HRTIMER_MODE_REL_PINNED);
		} else {
			enable_irq(tx_chn->irq);
		}
	}

	return num_tx_packets;
}

static irqreturn_t prueth_tx_irq(int irq, void *dev_id)
{
	struct prueth_tx_chn *tx_chn = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&tx_chn->napi_tx);

	return IRQ_HANDLED;
}

int prueth_ndev_add_tx_napi(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	int i, ret;

	for (i = 0; i < emac->tx_ch_num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		netif_napi_add_tx(emac->ndev, &tx_chn->napi_tx, emac_napi_tx_poll);
		hrtimer_init(&tx_chn->tx_hrtimer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL_PINNED);
		tx_chn->tx_hrtimer.function = &emac_tx_timer_callback;
		ret = request_irq(tx_chn->irq, prueth_tx_irq,
				  IRQF_TRIGGER_HIGH, tx_chn->name,
				  tx_chn);
		if (ret) {
			netif_napi_del(&tx_chn->napi_tx);
			dev_err(prueth->dev, "unable to request TX IRQ %d\n",
				tx_chn->irq);
			goto fail;
		}
	}

	return 0;
fail:
	prueth_ndev_del_tx_napi(emac, i);
	return ret;
}

int prueth_init_tx_chns(struct prueth_emac *emac)
{
	static const struct k3_ring_cfg ring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_RING,
		.flags = 0,
		.size = PRUETH_MAX_TX_DESC,
	};
	struct k3_udma_glue_tx_channel_cfg tx_cfg;
	struct device *dev = emac->prueth->dev;
	struct net_device *ndev = emac->ndev;
	int ret, slice, i;
	u32 hdesc_size;

	slice = prueth_emac_slice(emac);
	if (slice < 0)
		return slice;

	init_completion(&emac->tdown_complete);

	hdesc_size = cppi5_hdesc_calc_size(true, PRUETH_NAV_PS_DATA_SIZE,
					   PRUETH_NAV_SW_DATA_SIZE);
	memset(&tx_cfg, 0, sizeof(tx_cfg));
	tx_cfg.swdata_size = PRUETH_NAV_SW_DATA_SIZE;
	tx_cfg.tx_cfg = ring_cfg;
	tx_cfg.txcq_cfg = ring_cfg;

	for (i = 0; i < emac->tx_ch_num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		/* To differentiate channels for SLICE0 vs SLICE1 */
		snprintf(tx_chn->name, sizeof(tx_chn->name),
			 "tx%d-%d", slice, i);

		tx_chn->emac = emac;
		tx_chn->id = i;
		tx_chn->descs_num = PRUETH_MAX_TX_DESC;

		tx_chn->tx_chn =
			k3_udma_glue_request_tx_chn(dev, tx_chn->name,
						    &tx_cfg);
		if (IS_ERR(tx_chn->tx_chn)) {
			ret = PTR_ERR(tx_chn->tx_chn);
			tx_chn->tx_chn = NULL;
			netdev_err(ndev,
				   "Failed to request tx dma ch: %d\n", ret);
			goto fail;
		}

		tx_chn->dma_dev = k3_udma_glue_tx_get_dma_device(tx_chn->tx_chn);
		tx_chn->desc_pool =
			k3_cppi_desc_pool_create_name(tx_chn->dma_dev,
						      tx_chn->descs_num,
						      hdesc_size,
						      tx_chn->name);
		if (IS_ERR(tx_chn->desc_pool)) {
			ret = PTR_ERR(tx_chn->desc_pool);
			tx_chn->desc_pool = NULL;
			netdev_err(ndev, "Failed to create tx pool: %d\n", ret);
			goto fail;
		}

		ret = k3_udma_glue_tx_get_irq(tx_chn->tx_chn);
		if (ret < 0) {
			netdev_err(ndev, "failed to get tx irq\n");
			goto fail;
		}
		tx_chn->irq = ret;

		snprintf(tx_chn->name, sizeof(tx_chn->name), "%s-tx%d",
			 dev_name(dev), tx_chn->id);
	}

	return 0;

fail:
	prueth_cleanup_tx_chns(emac);
	return ret;
}

int prueth_init_rx_chns(struct prueth_emac *emac,
			struct prueth_rx_chn *rx_chn,
			char *name, u32 max_rflows,
			u32 max_desc_num)
{
	struct k3_udma_glue_rx_channel_cfg rx_cfg;
	struct device *dev = emac->prueth->dev;
	struct net_device *ndev = emac->ndev;
	u32 fdqring_id, hdesc_size;
	int i, ret = 0, slice;
	int flow_id_base;

	slice = prueth_emac_slice(emac);
	if (slice < 0)
		return slice;

	/* To differentiate channels for SLICE0 vs SLICE1 */
	snprintf(rx_chn->name, sizeof(rx_chn->name), "%s%d", name, slice);

	hdesc_size = cppi5_hdesc_calc_size(true, PRUETH_NAV_PS_DATA_SIZE,
					   PRUETH_NAV_SW_DATA_SIZE);
	memset(&rx_cfg, 0, sizeof(rx_cfg));
	rx_cfg.swdata_size = PRUETH_NAV_SW_DATA_SIZE;
	rx_cfg.flow_id_num = max_rflows;
	rx_cfg.flow_id_base = -1; /* udmax will auto select flow id base */

	/* init all flows */
	rx_chn->dev = dev;
	rx_chn->descs_num = max_desc_num;

	rx_chn->rx_chn = k3_udma_glue_request_rx_chn(dev, rx_chn->name,
						     &rx_cfg);
	if (IS_ERR(rx_chn->rx_chn)) {
		ret = PTR_ERR(rx_chn->rx_chn);
		rx_chn->rx_chn = NULL;
		netdev_err(ndev, "Failed to request rx dma ch: %d\n", ret);
		goto fail;
	}

	rx_chn->dma_dev = k3_udma_glue_rx_get_dma_device(rx_chn->rx_chn);
	rx_chn->desc_pool = k3_cppi_desc_pool_create_name(rx_chn->dma_dev,
							  rx_chn->descs_num,
							  hdesc_size,
							  rx_chn->name);
	if (IS_ERR(rx_chn->desc_pool)) {
		ret = PTR_ERR(rx_chn->desc_pool);
		rx_chn->desc_pool = NULL;
		netdev_err(ndev, "Failed to create rx pool: %d\n", ret);
		goto fail;
	}

	flow_id_base = k3_udma_glue_rx_get_flow_id_base(rx_chn->rx_chn);
	if (emac->is_sr1 && !strcmp(name, "rxmgm")) {
		emac->rx_mgm_flow_id_base = flow_id_base;
		netdev_dbg(ndev, "mgm flow id base = %d\n", flow_id_base);
	} else {
		emac->rx_flow_id_base = flow_id_base;
		netdev_dbg(ndev, "flow id base = %d\n", flow_id_base);
	}

	fdqring_id = K3_RINGACC_RING_ID_ANY;
	for (i = 0; i < rx_cfg.flow_id_num; i++) {
		struct k3_ring_cfg rxring_cfg = {
			.elm_size = K3_RINGACC_RING_ELSIZE_8,
			.mode = K3_RINGACC_RING_MODE_RING,
			.flags = 0,
		};
		struct k3_ring_cfg fdqring_cfg = {
			.elm_size = K3_RINGACC_RING_ELSIZE_8,
			.flags = K3_RINGACC_RING_SHARED,
		};
		struct k3_udma_glue_rx_flow_cfg rx_flow_cfg = {
			.rx_cfg = rxring_cfg,
			.rxfdq_cfg = fdqring_cfg,
			.ring_rxq_id = K3_RINGACC_RING_ID_ANY,
			.src_tag_lo_sel =
				K3_UDMA_GLUE_SRC_TAG_LO_USE_REMOTE_SRC_TAG,
		};

		rx_flow_cfg.ring_rxfdq0_id = fdqring_id;
		rx_flow_cfg.rx_cfg.size = max_desc_num;
		rx_flow_cfg.rxfdq_cfg.size = max_desc_num;
		rx_flow_cfg.rxfdq_cfg.mode = emac->prueth->pdata.fdqring_mode;

		ret = k3_udma_glue_rx_flow_init(rx_chn->rx_chn,
						i, &rx_flow_cfg);
		if (ret) {
			netdev_err(ndev, "Failed to init rx flow%d %d\n",
				   i, ret);
			goto fail;
		}
		if (!i)
			fdqring_id = k3_udma_glue_rx_flow_get_fdq_id(rx_chn->rx_chn,
								     i);
		ret = k3_udma_glue_rx_get_irq(rx_chn->rx_chn, i);
		if (ret <= 0) {
			if (!ret)
				ret = -ENXIO;
			netdev_err(ndev, "Failed to get rx dma irq");
			goto fail;
		}
		rx_chn->irq[i] = ret;
	}

	return 0;

fail:
	prueth_cleanup_rx_chns(emac, rx_chn, max_rflows);
	return ret;
}

int prueth_dma_rx_push(struct prueth_emac *emac,
		       struct sk_buff *skb,
		       struct prueth_rx_chn *rx_chn)
{
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_rx;
	u32 pkt_len = skb_tailroom(skb);
	dma_addr_t desc_dma;
	dma_addr_t buf_dma;
	void **swdata;

	desc_rx = k3_cppi_desc_pool_alloc(rx_chn->desc_pool);
	if (!desc_rx) {
		netdev_err(ndev, "rx push: failed to allocate descriptor\n");
		return -ENOMEM;
	}
	desc_dma = k3_cppi_desc_pool_virt2dma(rx_chn->desc_pool, desc_rx);

	buf_dma = dma_map_single(rx_chn->dma_dev, skb->data, pkt_len, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(rx_chn->dma_dev, buf_dma))) {
		k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);
		netdev_err(ndev, "rx push: failed to map rx pkt buffer\n");
		return -EINVAL;
	}

	cppi5_hdesc_init(desc_rx, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PRUETH_NAV_PS_DATA_SIZE);
	k3_udma_glue_rx_dma_to_cppi5_addr(rx_chn->rx_chn, &buf_dma);
	cppi5_hdesc_attach_buf(desc_rx, buf_dma, skb_tailroom(skb), buf_dma, skb_tailroom(skb));

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	*swdata = skb;

	return k3_udma_glue_push_rx_chn(rx_chn->rx_chn, 0,
					desc_rx, desc_dma);
}

u64 icssg_ts_to_ns(u32 hi_sw, u32 hi, u32 lo, u32 cycle_time_ns)
{
	u32 iepcount_lo, iepcount_hi, hi_rollover_count;
	u64 ns;

	iepcount_lo = lo & GENMASK(19, 0);
	iepcount_hi = (hi & GENMASK(11, 0)) << 12 | lo >> 20;
	hi_rollover_count = hi >> 11;

	ns = ((u64)hi_rollover_count) << 23 | (iepcount_hi + hi_sw);
	ns = ns * cycle_time_ns + iepcount_lo;

	return ns;
}

void emac_rx_timestamp(struct prueth_emac *emac,
		       struct sk_buff *skb, u32 *psdata)
{
	struct skb_shared_hwtstamps *ssh;
	u64 ns;

	if (emac->is_sr1) {
		ns = (u64)psdata[1] << 32 | psdata[0];
	} else {
		u32 hi_sw = readl(emac->prueth->shram.va +
				  TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET);
		ns = icssg_ts_to_ns(hi_sw, psdata[1], psdata[0],
				    IEP_DEFAULT_CYCLE_TIME_NS);
	}

	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = ns_to_ktime(ns);
}

static int emac_rx_packet(struct prueth_emac *emac, u32 flow_id)
{
	struct prueth_rx_chn *rx_chn = &emac->rx_chns;
	u32 buf_dma_len, pkt_len, port_id = 0;
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_rx;
	struct sk_buff *skb, *new_skb;
	dma_addr_t desc_dma, buf_dma;
	void **swdata;
	u32 *psdata;
	int ret;

	ret = k3_udma_glue_pop_rx_chn(rx_chn->rx_chn, flow_id, &desc_dma);
	if (ret) {
		if (ret != -ENODATA)
			netdev_err(ndev, "rx pop: failed: %d\n", ret);
		return ret;
	}

	if (cppi5_desc_is_tdcm(desc_dma)) /* Teardown ? */
		return 0;

	desc_rx = k3_cppi_desc_pool_dma2virt(rx_chn->desc_pool, desc_dma);

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;

	psdata = cppi5_hdesc_get_psdata(desc_rx);
	/* RX HW timestamp */
	if (emac->rx_ts_enabled)
		emac_rx_timestamp(emac, skb, psdata);

	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);
	k3_udma_glue_rx_cppi5_to_dma_addr(rx_chn->rx_chn, &buf_dma);
	pkt_len = cppi5_hdesc_get_pktlen(desc_rx);
	/* firmware adds 4 CRC bytes, strip them */
	pkt_len -= 4;
	cppi5_desc_get_tags_ids(&desc_rx->hdr, &port_id, NULL);

	dma_unmap_single(rx_chn->dma_dev, buf_dma, buf_dma_len, DMA_FROM_DEVICE);
	k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);

	skb->dev = ndev;
	new_skb = netdev_alloc_skb_ip_align(ndev, PRUETH_MAX_PKT_SIZE);
	/* if allocation fails we drop the packet but push the
	 * descriptor back to the ring with old skb to prevent a stall
	 */
	if (!new_skb) {
		ndev->stats.rx_dropped++;
		new_skb = skb;
	} else {
		/* send the filled skb up the n/w stack */
		skb_put(skb, pkt_len);
		skb->protocol = eth_type_trans(skb, ndev);
		napi_gro_receive(&emac->napi_rx, skb);
		ndev->stats.rx_bytes += pkt_len;
		ndev->stats.rx_packets++;
	}

	/* queue another RX DMA */
	ret = prueth_dma_rx_push(emac, new_skb, &emac->rx_chns);
	if (WARN_ON(ret < 0)) {
		dev_kfree_skb_any(new_skb);
		ndev->stats.rx_errors++;
		ndev->stats.rx_dropped++;
	}

	return ret;
}

static void prueth_rx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct prueth_rx_chn *rx_chn = data;
	struct cppi5_host_desc_t *desc_rx;
	struct sk_buff *skb;
	dma_addr_t buf_dma;
	u32 buf_dma_len;
	void **swdata;

	desc_rx = k3_cppi_desc_pool_dma2virt(rx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;
	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);
	k3_udma_glue_rx_cppi5_to_dma_addr(rx_chn->rx_chn, &buf_dma);

	dma_unmap_single(rx_chn->dma_dev, buf_dma, buf_dma_len,
			 DMA_FROM_DEVICE);
	k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);

	dev_kfree_skb_any(skb);
}

static int prueth_tx_ts_cookie_get(struct prueth_emac *emac)
{
	int i;

	/* search and get the next free slot */
	for (i = 0; i < PRUETH_MAX_TX_TS_REQUESTS; i++) {
		if (!emac->tx_ts_skb[i]) {
			emac->tx_ts_skb[i] = ERR_PTR(-EBUSY); /* reserve slot */
			return i;
		}
	}

	return -EBUSY;
}

/**
 * emac_ndo_start_xmit - EMAC Transmit function
 * @skb: SKB pointer
 * @ndev: EMAC network adapter
 *
 * Called by the system to transmit a packet  - we queue the packet in
 * EMAC hardware transmit queue
 * Doesn't wait for completion we'll check for TX completion in
 * emac_tx_complete_packets().
 *
 * Return: enum netdev_tx
 */
enum netdev_tx emac_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct cppi5_host_desc_t *first_desc, *next_desc, *cur_desc;
	struct prueth_emac *emac = netdev_priv(ndev);
	struct netdev_queue *netif_txq;
	struct prueth_tx_chn *tx_chn;
	dma_addr_t desc_dma, buf_dma;
	int i, ret = 0, q_idx;
	bool in_tx_ts = 0;
	int tx_ts_cookie;
	void **swdata;
	u32 pkt_len;
	u32 *epib;

	pkt_len = skb_headlen(skb);
	q_idx = skb_get_queue_mapping(skb);

	tx_chn = &emac->tx_chns[q_idx];
	netif_txq = netdev_get_tx_queue(ndev, q_idx);

	/* Map the linear buffer */
	buf_dma = dma_map_single(tx_chn->dma_dev, skb->data, pkt_len, DMA_TO_DEVICE);
	if (dma_mapping_error(tx_chn->dma_dev, buf_dma)) {
		netdev_err(ndev, "tx: failed to map skb buffer\n");
		ret = NETDEV_TX_OK;
		goto drop_free_skb;
	}

	first_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
	if (!first_desc) {
		netdev_dbg(ndev, "tx: failed to allocate descriptor\n");
		dma_unmap_single(tx_chn->dma_dev, buf_dma, pkt_len, DMA_TO_DEVICE);
		goto drop_stop_q_busy;
	}

	cppi5_hdesc_init(first_desc, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PRUETH_NAV_PS_DATA_SIZE);
	cppi5_hdesc_set_pkttype(first_desc, 0);
	epib = first_desc->epib;
	epib[0] = 0;
	epib[1] = 0;
	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP &&
	    emac->tx_ts_enabled) {
		tx_ts_cookie = prueth_tx_ts_cookie_get(emac);
		if (tx_ts_cookie >= 0) {
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			/* Request TX timestamp */
			epib[0] = (u32)tx_ts_cookie;
			epib[1] = 0x80000000;	/* TX TS request */
			emac->tx_ts_skb[tx_ts_cookie] = skb_get(skb);
			in_tx_ts = 1;
		}
	}

	/* set dst tag to indicate internal qid at the firmware which is at
	 * bit8..bit15. bit0..bit7 indicates port num for directed
	 * packets in case of switch mode operation
	 */
	cppi5_desc_set_tags_ids(&first_desc->hdr, 0, (emac->port_id | (q_idx << 8)));
	k3_udma_glue_tx_dma_to_cppi5_addr(tx_chn->tx_chn, &buf_dma);
	cppi5_hdesc_attach_buf(first_desc, buf_dma, pkt_len, buf_dma, pkt_len);
	swdata = cppi5_hdesc_get_swdata(first_desc);
	*swdata = skb;

	/* Handle the case where skb is fragmented in pages */
	cur_desc = first_desc;
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		u32 frag_size = skb_frag_size(frag);

		next_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
		if (!next_desc) {
			netdev_err(ndev,
				   "tx: failed to allocate frag. descriptor\n");
			goto free_desc_stop_q_busy_cleanup_tx_ts;
		}

		buf_dma = skb_frag_dma_map(tx_chn->dma_dev, frag, 0, frag_size,
					   DMA_TO_DEVICE);
		if (dma_mapping_error(tx_chn->dma_dev, buf_dma)) {
			netdev_err(ndev, "tx: Failed to map skb page\n");
			k3_cppi_desc_pool_free(tx_chn->desc_pool, next_desc);
			ret = NETDEV_TX_OK;
			goto cleanup_tx_ts;
		}

		cppi5_hdesc_reset_hbdesc(next_desc);
		k3_udma_glue_tx_dma_to_cppi5_addr(tx_chn->tx_chn, &buf_dma);
		cppi5_hdesc_attach_buf(next_desc,
				       buf_dma, frag_size, buf_dma, frag_size);

		desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool,
						      next_desc);
		k3_udma_glue_tx_dma_to_cppi5_addr(tx_chn->tx_chn, &desc_dma);
		cppi5_hdesc_link_hbdesc(cur_desc, desc_dma);

		pkt_len += frag_size;
		cur_desc = next_desc;
	}
	WARN_ON_ONCE(pkt_len != skb->len);

	/* report bql before sending packet */
	netdev_tx_sent_queue(netif_txq, pkt_len);

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool, first_desc);
	/* cppi5_desc_dump(first_desc, 64); */

	skb_tx_timestamp(skb);  /* SW timestamp if SKBTX_IN_PROGRESS not set */
	ret = k3_udma_glue_push_tx_chn(tx_chn->tx_chn, first_desc, desc_dma);
	if (ret) {
		netdev_err(ndev, "tx: push failed: %d\n", ret);
		goto drop_free_descs;
	}

	if (in_tx_ts)
		atomic_inc(&emac->tx_ts_pending);

	if (k3_cppi_desc_pool_avail(tx_chn->desc_pool) < MAX_SKB_FRAGS) {
		netif_tx_stop_queue(netif_txq);
		/* Barrier, so that stop_queue visible to other cpus */
		smp_mb__after_atomic();

		if (k3_cppi_desc_pool_avail(tx_chn->desc_pool) >=
		    MAX_SKB_FRAGS)
			netif_tx_wake_queue(netif_txq);
	}

	return NETDEV_TX_OK;

cleanup_tx_ts:
	if (in_tx_ts) {
		dev_kfree_skb_any(emac->tx_ts_skb[tx_ts_cookie]);
		emac->tx_ts_skb[tx_ts_cookie] = NULL;
	}

drop_free_descs:
	prueth_xmit_free(tx_chn, first_desc);

drop_free_skb:
	dev_kfree_skb_any(skb);

	/* error */
	ndev->stats.tx_dropped++;
	netdev_err(ndev, "tx: error: %d\n", ret);

	return ret;

free_desc_stop_q_busy_cleanup_tx_ts:
	if (in_tx_ts) {
		dev_kfree_skb_any(emac->tx_ts_skb[tx_ts_cookie]);
		emac->tx_ts_skb[tx_ts_cookie] = NULL;
	}
	prueth_xmit_free(tx_chn, first_desc);

drop_stop_q_busy:
	netif_tx_stop_queue(netif_txq);
	return NETDEV_TX_BUSY;
}

static void prueth_tx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct prueth_tx_chn *tx_chn = data;
	struct cppi5_host_desc_t *desc_tx;
	struct sk_buff *skb;
	void **swdata;

	desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_tx);
	skb = *(swdata);
	prueth_xmit_free(tx_chn, desc_tx);

	dev_kfree_skb_any(skb);
}

irqreturn_t prueth_rx_irq(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&emac->napi_rx);

	return IRQ_HANDLED;
}

void prueth_emac_stop(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	int slice;

	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		slice = ICSS_SLICE0;
		break;
	case PRUETH_PORT_MII1:
		slice = ICSS_SLICE1;
		break;
	default:
		netdev_err(emac->ndev, "invalid port\n");
		return;
	}

	emac->fw_running = 0;
	if (!emac->is_sr1)
		rproc_shutdown(prueth->txpru[slice]);
	rproc_shutdown(prueth->rtu[slice]);
	rproc_shutdown(prueth->pru[slice]);
}

void prueth_cleanup_tx_ts(struct prueth_emac *emac)
{
	int i;

	for (i = 0; i < PRUETH_MAX_TX_TS_REQUESTS; i++) {
		if (emac->tx_ts_skb[i]) {
			dev_kfree_skb_any(emac->tx_ts_skb[i]);
			emac->tx_ts_skb[i] = NULL;
		}
	}
}

int emac_napi_rx_poll(struct napi_struct *napi_rx, int budget)
{
	struct prueth_emac *emac = prueth_napi_to_emac(napi_rx);
	int rx_flow = emac->is_sr1 ?
		PRUETH_RX_FLOW_DATA_SR1 : PRUETH_RX_FLOW_DATA;
	int flow = emac->is_sr1 ?
		PRUETH_MAX_RX_FLOWS_SR1 : PRUETH_MAX_RX_FLOWS;
	int num_rx = 0;
	int cur_budget;
	int ret;

	while (flow--) {
		cur_budget = budget - num_rx;

		while (cur_budget--) {
			ret = emac_rx_packet(emac, flow);
			if (ret)
				break;
			num_rx++;
		}

		if (num_rx >= budget)
			break;
	}

	if (num_rx < budget && napi_complete_done(napi_rx, num_rx)) {
		if (unlikely(emac->rx_pace_timeout_ns)) {
			hrtimer_start(&emac->rx_hrtimer,
				      ns_to_ktime(emac->rx_pace_timeout_ns),
				      HRTIMER_MODE_REL_PINNED);
		} else {
			enable_irq(emac->rx_chns.irq[rx_flow]);
		}
	}

	return num_rx;
}

int prueth_prepare_rx_chan(struct prueth_emac *emac,
			   struct prueth_rx_chn *chn,
			   int buf_size)
{
	struct sk_buff *skb;
	int i, ret;

	for (i = 0; i < chn->descs_num; i++) {
		skb = __netdev_alloc_skb_ip_align(NULL, buf_size, GFP_KERNEL);
		if (!skb)
			return -ENOMEM;

		ret = prueth_dma_rx_push(emac, skb, chn);
		if (ret < 0) {
			netdev_err(emac->ndev,
				   "cannot submit skb for rx chan %s ret %d\n",
				   chn->name, ret);
			kfree_skb(skb);
			return ret;
		}
	}

	return 0;
}

void prueth_reset_tx_chan(struct prueth_emac *emac, int ch_num,
			  bool free_skb)
{
	int i;

	for (i = 0; i < ch_num; i++) {
		if (free_skb)
			k3_udma_glue_reset_tx_chn(emac->tx_chns[i].tx_chn,
						  &emac->tx_chns[i],
						  prueth_tx_cleanup);
		k3_udma_glue_disable_tx_chn(emac->tx_chns[i].tx_chn);
	}
}

void prueth_reset_rx_chan(struct prueth_rx_chn *chn,
			  int num_flows, bool disable)
{
	int i;

	for (i = 0; i < num_flows; i++)
		k3_udma_glue_reset_rx_chn(chn->rx_chn, i, chn,
					  prueth_rx_cleanup, !!i);
	if (disable)
		k3_udma_glue_disable_rx_chn(chn->rx_chn);
}

void emac_ndo_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	ndev->stats.tx_errors++;
}

static int emac_set_ts_config(struct net_device *ndev, struct ifreq *ifr)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		emac->tx_ts_enabled = 0;
		break;
	case HWTSTAMP_TX_ON:
		emac->tx_ts_enabled = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		emac->rx_ts_enabled = 0;
		break;
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_SOME:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
	case HWTSTAMP_FILTER_NTP_ALL:
		emac->rx_ts_enabled = 1;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		return -ERANGE;
	}

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

static int emac_get_ts_config(struct net_device *ndev, struct ifreq *ifr)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct hwtstamp_config config;

	config.flags = 0;
	config.tx_type = emac->tx_ts_enabled ? HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF;
	config.rx_filter = emac->rx_ts_enabled ? HWTSTAMP_FILTER_ALL : HWTSTAMP_FILTER_NONE;

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
			    -EFAULT : 0;
}

int emac_ndo_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCGHWTSTAMP:
		return emac_get_ts_config(ndev, ifr);
	case SIOCSHWTSTAMP:
		return emac_set_ts_config(ndev, ifr);
	default:
		break;
	}

	return phy_do_ioctl(ndev, ifr, cmd);
}

void emac_ndo_get_stats64(struct net_device *ndev,
			  struct rtnl_link_stats64 *stats)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	emac_update_hardware_stats(emac);

	stats->rx_packets     = emac_get_stat_by_name(emac, "rx_packets");
	stats->rx_bytes       = emac_get_stat_by_name(emac, "rx_bytes");
	stats->tx_packets     = emac_get_stat_by_name(emac, "tx_packets");
	stats->tx_bytes       = emac_get_stat_by_name(emac, "tx_bytes");
	stats->rx_crc_errors  = emac_get_stat_by_name(emac, "rx_crc_errors");
	stats->rx_over_errors = emac_get_stat_by_name(emac, "rx_over_errors");
	stats->multicast      = emac_get_stat_by_name(emac, "rx_multicast_frames");

	stats->rx_errors  = ndev->stats.rx_errors;
	stats->rx_dropped = ndev->stats.rx_dropped;
	stats->tx_errors  = ndev->stats.tx_errors;
	stats->tx_dropped = ndev->stats.tx_dropped;
}

int emac_ndo_get_phys_port_name(struct net_device *ndev, char *name,
				size_t len)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int ret;

	ret = snprintf(name, len, "p%d", emac->port_id);
	if (ret >= len)
		return -EINVAL;

	return 0;
}

/* get emac_port corresponding to eth_node name */
int prueth_node_port(struct device_node *eth_node)
{
	u32 port_id;
	int ret;

	ret = of_property_read_u32(eth_node, "reg", &port_id);
	if (ret)
		return ret;

	if (port_id == 0)
		return PRUETH_PORT_MII0;
	else if (port_id == 1)
		return PRUETH_PORT_MII1;
	else
		return PRUETH_PORT_INVALID;
}

/* get MAC instance corresponding to eth_node name */
int prueth_node_mac(struct device_node *eth_node)
{
	u32 port_id;
	int ret;

	ret = of_property_read_u32(eth_node, "reg", &port_id);
	if (ret)
		return ret;

	if (port_id == 0)
		return PRUETH_MAC0;
	else if (port_id == 1)
		return PRUETH_MAC1;
	else
		return PRUETH_MAC_INVALID;
}

void prueth_netdev_exit(struct prueth *prueth,
			struct device_node *eth_node)
{
	struct prueth_emac *emac;
	enum prueth_mac mac;

	mac = prueth_node_mac(eth_node);
	if (mac == PRUETH_MAC_INVALID)
		return;

	emac = prueth->emac[mac];
	if (!emac)
		return;

	if (of_phy_is_fixed_link(emac->phy_node))
		of_phy_deregister_fixed_link(emac->phy_node);

	netif_napi_del(&emac->napi_rx);

	pruss_release_mem_region(prueth->pruss, &emac->dram);
	destroy_workqueue(emac->cmd_wq);
	free_netdev(emac->ndev);
	prueth->emac[mac] = NULL;
}

int prueth_get_cores(struct prueth *prueth, int slice, bool is_sr1)
{
	struct device *dev = prueth->dev;
	enum pruss_pru_id pruss_id;
	struct device_node *np;
	int idx = -1, ret;

	np = dev->of_node;

	switch (slice) {
	case ICSS_SLICE0:
		idx = 0;
		break;
	case ICSS_SLICE1:
		idx = is_sr1 ? 2 : 3;
		break;
	default:
		return -EINVAL;
	}

	prueth->pru[slice] = pru_rproc_get(np, idx, &pruss_id);
	if (IS_ERR(prueth->pru[slice])) {
		ret = PTR_ERR(prueth->pru[slice]);
		prueth->pru[slice] = NULL;
		return dev_err_probe(dev, ret, "unable to get PRU%d\n", slice);
	}
	prueth->pru_id[slice] = pruss_id;

	idx++;
	prueth->rtu[slice] = pru_rproc_get(np, idx, NULL);
	if (IS_ERR(prueth->rtu[slice])) {
		ret = PTR_ERR(prueth->rtu[slice]);
		prueth->rtu[slice] = NULL;
		return dev_err_probe(dev, ret, "unable to get RTU%d\n", slice);
	}

	if (is_sr1)
		return 0;

	idx++;
	prueth->txpru[slice] = pru_rproc_get(np, idx, NULL);
	if (IS_ERR(prueth->txpru[slice])) {
		ret = PTR_ERR(prueth->txpru[slice]);
		prueth->txpru[slice] = NULL;
		return dev_err_probe(dev, ret, "unable to get TX_PRU%d\n", slice);
	}

	return 0;
}

void prueth_put_cores(struct prueth *prueth, int slice)
{
	if (prueth->txpru[slice])
		pru_rproc_put(prueth->txpru[slice]);

	if (prueth->rtu[slice])
		pru_rproc_put(prueth->rtu[slice]);

	if (prueth->pru[slice])
		pru_rproc_put(prueth->pru[slice]);
}

#ifdef CONFIG_PM_SLEEP
static int prueth_suspend(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			ret = ndev->netdev_ops->ndo_stop(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to stop: %d", ret);
				return ret;
			}
		}
	}

	return 0;
}

static int prueth_resume(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			ret = ndev->netdev_ops->ndo_open(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to start: %d", ret);
				return ret;
			}
			netif_device_attach(ndev);
		}
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

const struct dev_pm_ops prueth_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(prueth_suspend, prueth_resume)
};
