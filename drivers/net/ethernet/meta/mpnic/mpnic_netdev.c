// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#include <linux/etherdevice.h>
#include <linux/ipv6.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/types.h>

#include "mpnic.h"
#include "mpnic_netdev.h"
#include "mpnic_txrx.h"

static int mpnic_open(struct net_device *netdev)
{
	struct mpnic_net *mpn = netdev_priv(netdev);
	int err;

	err = mpnic_alloc_napi_vectors(mpn);
	if (err)
		return err;

	err = mpnic_alloc_resources(mpn);
	if (err)
		goto err_free_napi_vectors;

	err = mpnic_set_netif_queues(mpn);
	if (err)
		goto err_free_resources;

	mpnic_enable(mpn);
	mpnic_napi_enable(mpn);

	netif_tx_wake_all_queues(netdev);
	netif_carrier_on(netdev);

	return 0;

err_free_resources:
	mpnic_free_resources(mpn);
err_free_napi_vectors:
	mpnic_free_napi_vectors(mpn);
	return err;
}

static int mpnic_stop(struct net_device *netdev)
{
	struct mpnic_net *mpn = netdev_priv(netdev);

	netif_carrier_off(netdev);

	mpnic_napi_disable(mpn);
	netif_tx_disable(netdev);

	mpnic_disable(mpn);
	mpnic_wait_all_queues_idle(mpn->mpd);
	mpnic_flush(mpn);

	mpnic_reset_netif_queues(mpn);
	mpnic_free_resources(mpn);
	mpnic_free_napi_vectors(mpn);

	return 0;
}

static const struct net_device_ops mpnic_netdev_ops = {
	.ndo_open		= mpnic_open,
	.ndo_stop		= mpnic_stop,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_start_xmit		= mpnic_xmit_frame,
};

/**
 * mpnic_netdev_free - Free the netdev associated with mpnic
 * @mpd: Driver specific structure to free netdev from
 **/
void mpnic_netdev_free(struct mpnic_dev *mpd)
{
	free_netdev(mpd->netdev);
	mpd->netdev = NULL;
}

/**
 * mpnic_netdev_alloc - Allocate a netdev and associate it with mpnic
 * @mpd: Driver specific structure to associate the netdev with
 *
 * Return: NULL on failure.
 **/
struct net_device *mpnic_netdev_alloc(struct mpnic_dev *mpd)
{
	struct net_device *netdev;
	struct mpnic_net *mpn;
	unsigned int queues;

	netdev = alloc_etherdev_mq(sizeof(*mpn), MPNIC_MAX_RXQS);
	if (!netdev)
		return NULL;

	SET_NETDEV_DEV(netdev, mpd->dev);
	mpd->netdev = netdev;

	netdev->netdev_ops = &mpnic_netdev_ops;
	netdev->request_ops_lock = true;

	mpn = netdev_priv(netdev);
	mpn->netdev = netdev;
	mpn->mpd = mpd;

	mpn->txq_size = MPNIC_TXQ_SIZE_DEFAULT;

	queues = min(netif_get_num_default_rss_queues(),
		     mpd->num_irqs - MPNIC_NON_NAPI_VECTORS);
	mpn->num_tx_queues = queues;
	mpn->num_napi = queues;

	netdev->features |= NETIF_F_SG;
	netdev->hw_features |= netdev->features;
	netdev->vlan_features |= netdev->features;

	netdev->min_mtu = IPV6_MIN_MTU;
	netdev->max_mtu = MPNIC_MAX_JUMBO_FRAME_SIZE - ETH_HLEN;

	netif_carrier_off(netdev);
	netif_tx_stop_all_queues(netdev);

	return netdev;
}

static int mpnic_dsn_to_mac_addr(u64 dsn, u8 *addr)
{
	addr[0] = (dsn >> 56) & 0xFF;
	addr[1] = (dsn >> 48) & 0xFF;
	addr[2] = (dsn >> 40) & 0xFF;
	addr[3] = (dsn >> 16) & 0xFF;
	addr[4] = (dsn >> 8) & 0xFF;
	addr[5] = dsn & 0xFF;

	return is_valid_ether_addr(addr) ? 0 : -EINVAL;
}

/**
 * mpnic_netdev_register - Assign the MAC address and register the netdev
 * @netdev: Netdev to register
 *
 * The permanent address is derived from the PCIe device serial number, the
 * same way the firmware and the BMC derive it. A random address would break
 * provisioning, so refuse to spawn the interface if the serial number does
 * not yield a valid one.
 *
 * Return: non-zero on failure.
 **/
int mpnic_netdev_register(struct net_device *netdev)
{
	struct mpnic_net *mpn = netdev_priv(netdev);
	struct mpnic_dev *mpd = mpn->mpd;
	u8 addr[ETH_ALEN];
	int err;

	err = mpnic_dsn_to_mac_addr(mpd->dsn, addr);
	if (err) {
		dev_err(mpd->dev, "MAC addr %pM invalid\n", addr);
		return err;
	}

	ether_addr_copy(netdev->perm_addr, addr);
	eth_hw_addr_set(netdev, addr);

	err = netif_set_real_num_tx_queues(netdev, mpn->num_tx_queues);
	if (err)
		return err;

	/* Abort if MMIO has failed. This has to be the last check before
	 * registration, the register accessors can only detach the device
	 * once it has been registered.
	 */
	if (!mpnic_present(mpd))
		return -EIO;

	return register_netdev(netdev);
}
