// SPDX-License-Identifier: GPL-2.0
/*
 * Point-to-point link over a pair of AXI-Stream DMA channels.
 *
 * Each side uses one axi-dmac per direction, joined back to back in fabric:
 *
 *   tx: MEM_TO_DEV dmac --> AXIS FIFO --> DEV_TO_MEM dmac :rx (far side)
 *   rx: DEV_TO_MEM dmac <-- AXIS FIFO <-- MEM_TO_DEV dmac :tx (far side)
 *
 * Buffers are mapped against each channel's DMA device, so the same driver
 * binds on both ends, including behind a PCIe endpoint.
 *
 * One frame per descriptor, nothing else on the wire. TLAST closes the RX
 * descriptor and TKEEP makes its residue byte exact, so the frame length is
 * rx_buf_size - residue. The RX DMAC therefore needs HAS_AXIS_TKEEP, pcore
 * >= 4.2.a and DMA_SG_TRANSFER off. A zero residue means one is missing or the
 * far side MTU is larger than ours.
 *
 * TX maps the skb in place, one segment per fragment, which needs
 * ALLOW_UNALIGNED_SRC on the TX DMAC.
 *
 * The Ethernet and L3 pipe front ends supply the netdev and the RX protocol.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <net/netdev_queues.h>

#include "adi_axis_net.h"

/*
 * The RX DMAC writes MAX_BYTES_PER_BURST bursts from the buffer start, and a
 * burst must not cross 4 KiB. Starting on a 256 byte boundary keeps every
 * burst inside one for a MAX_BYTES_PER_BURST of up to 256.
 */
#define ADI_AXIS_NET_DMA_ALIGN		256

/* Frames up to this length are copied and the RX buffer recycled */
#define ADI_AXIS_NET_COPYBREAK		256

#define ADI_AXIS_NET_TX_WAKE		(ADI_AXIS_NET_RING_SIZE / 4)

static unsigned int adi_axis_net_tx_free(struct adi_axis_net *priv)
{
	return ADI_AXIS_NET_RING_SIZE -
	       (READ_ONCE(priv->tx_head) - READ_ONCE(priv->tx_tail));
}

/* Allocate an RX skb with data aligned to ADI_AXIS_NET_DMA_ALIGN */
static struct sk_buff *adi_axis_net_alloc_rx_skb(struct adi_axis_net *priv)
{
	struct sk_buff *skb;
	unsigned int pad;

	skb = netdev_alloc_skb(priv->netdev,
			       priv->rx_buf_size + ADI_AXIS_NET_DMA_ALIGN);
	if (!skb)
		return NULL;

	pad = PTR_ALIGN(skb->data, ADI_AXIS_NET_DMA_ALIGN) - skb->data;
	skb_reserve(skb, pad);

	return skb;
}

static void adi_axis_net_rx_callback(void *param,
				     const struct dmaengine_result *res);

static int adi_axis_net_submit_rx(struct adi_axis_net *priv,
				  struct adi_axis_net_rx_slot *slot)
{
	struct dma_async_tx_descriptor *desc;

	if (!priv->running)
		return -ESHUTDOWN;

	if (!slot->skb) {
		slot->skb = adi_axis_net_alloc_rx_skb(priv);
		if (!slot->skb)
			return -ENOMEM;
	}

	slot->daddr = dma_map_single(priv->rx_dma_dev, slot->skb->data,
				     priv->rx_buf_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->rx_dma_dev, slot->daddr))
		return -ENOMEM;

	slot->mapped = true;

	desc = dmaengine_prep_slave_single(priv->rx_chan, slot->daddr,
					   priv->rx_buf_size, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT);
	if (!desc)
		goto err_unmap;

	desc->callback_result = adi_axis_net_rx_callback;
	desc->callback_param = slot;

	if (dma_submit_error(dmaengine_submit(desc)))
		goto err_unmap;

	return 0;

err_unmap:
	dma_unmap_single(priv->rx_dma_dev, slot->daddr, priv->rx_buf_size,
			 DMA_FROM_DEVICE);
	slot->mapped = false;
	return -EIO;
}

/* Serialised per channel by the dmaengine tasklet. Always reposts @slot. */
static void adi_axis_net_rx_callback(void *param,
				     const struct dmaengine_result *res)
{
	struct adi_axis_net_rx_slot *slot = param;
	struct adi_axis_net *priv = slot->priv;
	struct net_device *netdev = priv->netdev;
	struct sk_buff *skb;
	unsigned int len;
	int ret;

	dma_unmap_single(priv->rx_dma_dev, slot->daddr, priv->rx_buf_size,
			 DMA_FROM_DEVICE);
	slot->mapped = false;

	if (!priv->running)
		return;

	if (res->result != DMA_TRANS_NOERROR) {
		netdev_dbg(netdev, "RX transfer error %d\n", res->result);
		netdev->stats.rx_errors++;
		goto repost;
	}

	if (!res->residue) {
		netdev_warn_once(netdev,
				 "RX descriptor reported no residue; check the far side MTU and that the DMAC has HAS_AXIS_TKEEP with DMA_SG_TRANSFER off\n");
		netdev->stats.rx_length_errors++;
		goto repost;
	}

	if (res->residue >= priv->rx_buf_size) {
		netdev->stats.rx_length_errors++;
		goto repost;
	}

	len = priv->rx_buf_size - res->residue;

	if (len <= ADI_AXIS_NET_COPYBREAK) {
		skb = priv->hlen ? netdev_alloc_skb_ip_align(netdev, len) :
				   netdev_alloc_skb(netdev, len);
		if (!skb) {
			netdev->stats.rx_dropped++;
			goto repost;
		}
		skb_put_data(skb, slot->skb->data, len);
	} else {
		struct sk_buff *replacement = adi_axis_net_alloc_rx_skb(priv);

		if (!replacement) {
			netdev->stats.rx_dropped++;
			goto repost;
		}

		skb = slot->skb;
		slot->skb = replacement;
		skb_put(skb, len);
	}

	skb->protocol = priv->type_trans(skb, netdev);
	if (!skb->protocol) {
		netdev->stats.rx_errors++;
		dev_kfree_skb_any(skb);
		goto repost;
	}

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += len;

	netif_rx(skb);

repost:
	/* -ESHUTDOWN means ndo_stop got here first */
	ret = adi_axis_net_submit_rx(priv, slot);
	if (!ret)
		dma_async_issue_pending(priv->rx_chan);
	else if (ret != -ESHUTDOWN)
		netdev_err_once(netdev, "failed to repost RX slot %td: %d\n",
				slot - priv->rx, ret);
}

static void adi_axis_net_tx_release(struct adi_axis_net *priv,
				    struct adi_axis_net_tx_slot *slot,
				    bool sent)
{
	dma_unmap_sg(priv->tx_dma_dev, slot->sg, slot->nents, DMA_TO_DEVICE);
	if (sent)
		dev_consume_skb_any(slot->skb);
	else
		dev_kfree_skb_any(slot->skb);
	slot->skb = NULL;
}

/* Descriptors on one channel complete in order, so @slot is at tx_tail */
static void adi_axis_net_tx_callback(void *param)
{
	struct adi_axis_net_tx_slot *slot = param;
	struct adi_axis_net *priv = slot->priv;
	unsigned int len = slot->skb->len;

	adi_axis_net_tx_release(priv, slot, true);
	WRITE_ONCE(priv->tx_tail, priv->tx_tail + 1);

	__netif_txq_completed_wake(netdev_get_tx_queue(priv->netdev, 0), 1,
				   len, adi_axis_net_tx_free(priv),
				   ADI_AXIS_NET_TX_WAKE, !READ_ONCE(priv->running));
}

netdev_tx_t adi_axis_net_start_xmit(struct sk_buff *skb,
				    struct net_device *netdev)
{
	struct adi_axis_net *priv = netdev_priv(netdev);
	struct dma_async_tx_descriptor *desc;
	struct adi_axis_net_tx_slot *slot;
	unsigned int len = skb->len;
	int nents, mapped;

	slot = &priv->tx[priv->tx_head & (ADI_AXIS_NET_RING_SIZE - 1)];

	sg_init_table(slot->sg, ARRAY_SIZE(slot->sg));
	nents = skb_to_sgvec(skb, slot->sg, 0, len);
	if (nents <= 0)
		goto err_drop;

	mapped = dma_map_sg(priv->tx_dma_dev, slot->sg, nents, DMA_TO_DEVICE);
	if (!mapped)
		goto err_drop;

	slot->skb = skb;
	slot->nents = nents;

	desc = dmaengine_prep_slave_sg(priv->tx_chan, slot->sg, mapped,
				       DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc)
		goto err_release;

	desc->callback = adi_axis_net_tx_callback;
	desc->callback_param = slot;

	if (dma_submit_error(dmaengine_submit(desc)))
		goto err_release;

	/* Nothing runs before issue_pending, so the callback sees this head */
	WRITE_ONCE(priv->tx_head, priv->tx_head + 1);
	netdev_tx_sent_queue(netdev_get_tx_queue(netdev, 0), len);
	netif_subqueue_maybe_stop(netdev, 0, adi_axis_net_tx_free(priv), 1,
				  ADI_AXIS_NET_TX_WAKE);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += len;

	dma_async_issue_pending(priv->tx_chan);

	return NETDEV_TX_OK;

err_release:
	adi_axis_net_tx_release(priv, slot, false);
	netdev->stats.tx_errors++;
	return NETDEV_TX_OK;

err_drop:
	netdev->stats.tx_errors++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/* Called after dmaengine_terminate_sync(), whose discards never call back */
static void adi_axis_net_free_rx_ring(struct adi_axis_net *priv)
{
	unsigned int i;

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		struct adi_axis_net_rx_slot *slot = &priv->rx[i];

		if (slot->mapped) {
			dma_unmap_single(priv->rx_dma_dev, slot->daddr,
					 priv->rx_buf_size, DMA_FROM_DEVICE);
			slot->mapped = false;
		}

		dev_kfree_skb(slot->skb);
		slot->skb = NULL;
	}
}

/*
 * @running must be clear. The second terminate discards a repost from a
 * callback that sampled @running just before it was cleared.
 */
static void adi_axis_net_quiesce_rx(struct adi_axis_net *priv)
{
	dmaengine_terminate_sync(priv->rx_chan);
	dmaengine_terminate_sync(priv->rx_chan);

	adi_axis_net_free_rx_ring(priv);
}

int adi_axis_net_open(struct net_device *netdev)
{
	struct adi_axis_net *priv = netdev_priv(netdev);
	unsigned int i;
	int ret;

	/* The + 1 keeps a full-size frame from consuming the whole descriptor */
	priv->rx_buf_size = ALIGN(priv->hlen + READ_ONCE(netdev->mtu) + 1,
				  ADI_AXIS_NET_DMA_ALIGN);

	priv->running = true;

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		ret = adi_axis_net_submit_rx(priv, &priv->rx[i]);
		if (ret) {
			netdev_err(netdev, "failed to post RX slot %u: %d\n",
				   i, ret);
			goto err_quiesce;
		}
	}

	dma_async_issue_pending(priv->rx_chan);

	netif_start_queue(netdev);
	netif_carrier_on(netdev);
	return 0;

err_quiesce:
	priv->running = false;
	adi_axis_net_quiesce_rx(priv);

	return ret;
}

int adi_axis_net_stop(struct net_device *netdev)
{
	struct adi_axis_net *priv = netdev_priv(netdev);

	netif_carrier_off(netdev);
	netif_stop_queue(netdev);

	WRITE_ONCE(priv->running, false);

	dmaengine_terminate_sync(priv->tx_chan);

	for (; priv->tx_tail != priv->tx_head; priv->tx_tail++)
		adi_axis_net_tx_release(priv, &priv->tx[priv->tx_tail &
					(ADI_AXIS_NET_RING_SIZE - 1)], false);
	priv->tx_head = 0;
	priv->tx_tail = 0;
	netdev_tx_reset_queue(netdev_get_tx_queue(netdev, 0));

	adi_axis_net_quiesce_rx(priv);

	return 0;
}

static int adi_axis_net_check_chan(struct device *dev, struct dma_chan *chan,
				   enum dma_transfer_direction dir,
				   const char *name)
{
	if (!(chan->device->directions & BIT(dir)))
		return dev_err_probe(dev, -EINVAL,
				     "\"%s\" channel does not support %s\n", name,
				     dir == DMA_MEM_TO_DEV ? "MEM_TO_DEV" :
							     "DEV_TO_MEM");

	return 0;
}

static void adi_axis_net_release_chan(void *chan)
{
	dma_release_channel(chan);
}

static struct dma_chan *adi_axis_net_request_chan(struct device *dev,
						  const char *name,
						  enum dma_transfer_direction dir)
{
	struct dma_chan *chan;
	int ret;

	chan = dma_request_chan(dev, name);
	if (IS_ERR(chan))
		return dev_err_ptr_probe(dev, PTR_ERR(chan),
					 "failed to request \"%s\" DMA channel\n",
					 name);

	ret = devm_add_action_or_reset(dev, adi_axis_net_release_chan, chan);
	if (ret)
		return ERR_PTR(ret);

	ret = adi_axis_net_check_chan(dev, chan, dir, name);
	if (ret)
		return ERR_PTR(ret);

	return chan;
}

static void adi_axis_net_unregister(void *netdev)
{
	unregister_netdev(netdev);
}

int adi_axis_net_core_init(struct platform_device *pdev,
			   struct net_device *netdev, unsigned int hlen,
			   __be16 (*type_trans)(struct sk_buff *skb,
						struct net_device *netdev))
{
	struct adi_axis_net *priv = netdev_priv(netdev);
	struct dma_async_tx_descriptor *desc;
	struct device *dev = &pdev->dev;
	unsigned int i;
	int ret;

	SET_NETDEV_DEV(netdev, dev);
	platform_set_drvdata(pdev, netdev);

	priv->dev = dev;
	priv->netdev = netdev;
	priv->hlen = hlen;
	priv->type_trans = type_trans;

	priv->tx_chan = adi_axis_net_request_chan(dev, "tx", DMA_MEM_TO_DEV);
	if (IS_ERR(priv->tx_chan))
		return PTR_ERR(priv->tx_chan);

	priv->rx_chan = adi_axis_net_request_chan(dev, "rx", DMA_DEV_TO_MEM);
	if (IS_ERR(priv->rx_chan))
		return PTR_ERR(priv->rx_chan);

	/* The controllers, not @dev, carry the dma_map_ops */
	priv->tx_dma_dev = dmaengine_get_dma_device(priv->tx_chan);
	priv->rx_dma_dev = dmaengine_get_dma_device(priv->rx_chan);

	/* Only ALLOW_UNALIGNED_SRC accepts an odd address. Never submitted. */
	desc = dmaengine_prep_slave_single(priv->tx_chan, 1, 1, DMA_MEM_TO_DEV, 0);
	if (!desc)
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "tx DMAC lacks ALLOW_UNALIGNED_SRC\n");
	dmaengine_terminate_sync(priv->tx_chan);

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		priv->tx[i].priv = priv;
		priv->rx[i].priv = priv;
	}

	netdev->hw_features |= NETIF_F_SG;
	netdev->features |= NETIF_F_SG;

	/* No PHY: carrier follows whether the channels are armed */
	netif_carrier_off(netdev);

	ret = register_netdev(netdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register net device\n");

	return devm_add_action_or_reset(dev, adi_axis_net_unregister, netdev);
}

static struct platform_driver * const adi_axis_net_drivers[] = {
	&adi_axis_net_eth_driver,
	&adi_axis_net_pipe_driver,
};

static int __init adi_axis_net_init(void)
{
	return platform_register_drivers(adi_axis_net_drivers,
					 ARRAY_SIZE(adi_axis_net_drivers));
}
module_init(adi_axis_net_init);

static void __exit adi_axis_net_exit(void)
{
	platform_unregister_drivers(adi_axis_net_drivers,
				    ARRAY_SIZE(adi_axis_net_drivers));
}
module_exit(adi_axis_net_exit);

MODULE_AUTHOR("Analog Devices Inc.");
MODULE_DESCRIPTION("Point-to-point link over paired AXI-Stream DMA channels");
MODULE_LICENSE("GPL");
