// SPDX-License-Identifier: GPL-2.0
/*
 * L3 point-to-point front end for the AXI-Stream link. Frames are raw IPv4
 * or IPv6 packets, with no link-layer header or addresses.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include "adi_axis_net.h"

#define ADI_AXIS_NET_PIPE_MIN_MTU	68

static __be16 adi_axis_net_pipe_type_trans(struct sk_buff *skb,
					   struct net_device *netdev)
{
	__be16 proto;

	switch (skb->data[0] >> 4) {
	case 4:
		if (skb->len < sizeof(struct iphdr))
			return 0;
		proto = htons(ETH_P_IP);
		break;
	case 6:
		if (skb->len < sizeof(struct ipv6hdr))
			return 0;
		proto = htons(ETH_P_IPV6);
		break;
	default:
		return 0;
	}

	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);
	skb->pkt_type = PACKET_HOST;

	return proto;
}

static const struct net_device_ops adi_axis_net_pipe_ops = {
	.ndo_open		= adi_axis_net_open,
	.ndo_stop		= adi_axis_net_stop,
	.ndo_start_xmit		= adi_axis_net_start_xmit,
};

static void adi_axis_net_pipe_setup(struct net_device *netdev)
{
	netdev->netdev_ops = &adi_axis_net_pipe_ops;
	netdev->type = ARPHRD_NONE;
	netdev->flags = IFF_POINTOPOINT | IFF_NOARP;
	netdev->hard_header_len = 0;
	netdev->addr_len = 0;
	netdev->tx_queue_len = 1000;
	netdev->mtu = ETH_DATA_LEN;
	netdev->min_mtu = ADI_AXIS_NET_PIPE_MIN_MTU;
	netdev->max_mtu = ADI_AXIS_NET_MAX_MTU;
}

static void adi_axis_net_pipe_free(void *netdev)
{
	free_netdev(netdev);
}

static int adi_axis_net_pipe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	int ret;

	netdev = alloc_netdev(sizeof(struct adi_axis_net), "axp%d",
			      NET_NAME_ENUM, adi_axis_net_pipe_setup);
	if (!netdev)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, adi_axis_net_pipe_free, netdev);
	if (ret)
		return ret;

	return adi_axis_net_core_init(pdev, netdev, 0,
				      adi_axis_net_pipe_type_trans);
}

static const struct of_device_id adi_axis_net_pipe_of_match[] = {
	{ .compatible = "adi,axis-net-pipe" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_axis_net_pipe_of_match);

struct platform_driver adi_axis_net_pipe_driver = {
	.driver = {
		.name = "adi-axis-net-pipe",
		.of_match_table = adi_axis_net_pipe_of_match,
	},
	.probe = adi_axis_net_pipe_probe,
};
