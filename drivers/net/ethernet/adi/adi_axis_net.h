/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Point-to-point link over a pair of AXI-Stream DMA channels.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#ifndef __ADI_AXIS_NET_H__
#define __ADI_AXIS_NET_H__

#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/skbuff.h>

/* Ring depth per direction, a power of two */
#define ADI_AXIS_NET_RING_SIZE		64
#define ADI_AXIS_NET_MAX_MTU		8192

/**
 * struct adi_axis_net_tx_slot - one transmit ring entry
 * @skb: frame in flight
 * @sg: scatterlist over @skb, mapped while @skb is set
 * @nents: entries passed to dma_map_sg(), needed again to unmap
 * @priv: owning device
 */
struct adi_axis_net_tx_slot {
	struct sk_buff *skb;
	struct scatterlist sg[MAX_SKB_FRAGS + 1];
	int nents;
	struct adi_axis_net *priv;
};

/**
 * struct adi_axis_net_rx_slot - one receive ring entry
 * @skb: buffer the DMAC is filling, NULL while the interface is down
 * @daddr: streaming mapping of @skb->data, valid while @mapped is set
 * @priv: owning device
 * @mapped: set between dma_map_single() and its unmap
 */
struct adi_axis_net_rx_slot {
	struct sk_buff *skb;
	dma_addr_t daddr;
	struct adi_axis_net *priv;
	bool mapped;
};

/**
 * struct adi_axis_net - per-interface state
 * @dev: platform device
 * @netdev: the network interface
 * @tx_chan: MEM_TO_DEV channel
 * @rx_chan: DEV_TO_MEM channel
 * @tx_dma_dev: device TX buffers are mapped against
 * @rx_dma_dev: device RX buffers are mapped against
 * @tx: TX ring
 * @rx: RX ring
 * @tx_head: frames queued by ndo_start_xmit
 * @tx_tail: frames completed by the TX callback
 * @hlen: link-layer header length the RX buffer must hold beyond the MTU
 * @rx_buf_size: bytes posted per RX descriptor
 * @type_trans: sets up a received skb and returns its protocol, 0 to drop it
 * @running: cleared before teardown so completions stop resubmitting
 */
struct adi_axis_net {
	struct device *dev;
	struct net_device *netdev;

	struct dma_chan *tx_chan;
	struct dma_chan *rx_chan;
	struct device *tx_dma_dev;
	struct device *rx_dma_dev;

	struct adi_axis_net_tx_slot tx[ADI_AXIS_NET_RING_SIZE];
	struct adi_axis_net_rx_slot rx[ADI_AXIS_NET_RING_SIZE];

	unsigned int tx_head;
	unsigned int tx_tail;

	unsigned int hlen;
	size_t rx_buf_size;
	__be16 (*type_trans)(struct sk_buff *skb, struct net_device *netdev);
	bool running;
};

int adi_axis_net_open(struct net_device *netdev);
int adi_axis_net_stop(struct net_device *netdev);
netdev_tx_t adi_axis_net_start_xmit(struct sk_buff *skb,
				    struct net_device *netdev);
int adi_axis_net_core_init(struct platform_device *pdev,
			   struct net_device *netdev, unsigned int hlen,
			   __be16 (*type_trans)(struct sk_buff *skb,
						struct net_device *netdev));

extern struct platform_driver adi_axis_net_eth_driver;
extern struct platform_driver adi_axis_net_pipe_driver;

#endif /* __ADI_AXIS_NET_H__ */
