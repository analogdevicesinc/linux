// SPDX-License-Identifier: GPL-2.0
/*
 * Ethernet front end for the AXI-Stream point-to-point link.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include "adi_axis_net.h"

static __be16 adi_axis_net_eth_type_trans(struct sk_buff *skb,
					  struct net_device *netdev)
{
	if (skb->len < ETH_HLEN)
		return 0;

	return eth_type_trans(skb, netdev);
}

static const struct net_device_ops adi_axis_net_eth_ops = {
	.ndo_open		= adi_axis_net_open,
	.ndo_stop		= adi_axis_net_stop,
	.ndo_start_xmit		= adi_axis_net_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static int adi_axis_net_eth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	int ret;

	netdev = devm_alloc_etherdev(dev, sizeof(struct adi_axis_net));
	if (!netdev)
		return -ENOMEM;

	netdev->netdev_ops = &adi_axis_net_eth_ops;
	netdev->min_mtu = ETH_MIN_MTU;
	netdev->max_mtu = ADI_AXIS_NET_MAX_MTU;

	ret = platform_get_ethdev_address(dev, netdev);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret || !is_valid_ether_addr(netdev->dev_addr)) {
		eth_hw_addr_random(netdev);
		dev_info(dev, "using random MAC address %pM\n",
			 netdev->dev_addr);
	}

	return adi_axis_net_core_init(pdev, netdev, VLAN_ETH_HLEN,
				      adi_axis_net_eth_type_trans);
}

static const struct of_device_id adi_axis_net_eth_of_match[] = {
	{ .compatible = "adi,axis-net-eth" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_axis_net_eth_of_match);

struct platform_driver adi_axis_net_eth_driver = {
	.driver = {
		.name = "adi-axis-net-eth",
		.of_match_table = adi_axis_net_eth_of_match,
	},
	.probe = adi_axis_net_eth_probe,
};
