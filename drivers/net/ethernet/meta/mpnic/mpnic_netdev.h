/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef _MPNIC_NETDEV_H_
#define _MPNIC_NETDEV_H_

#include <linux/types.h>

#include "mpnic.h"
#include "mpnic_txrx.h"

struct mpnic_net {
	struct mpnic_ring *tx[MPNIC_MAX_TXQS];

	struct mpnic_napi_vector *napi[MPNIC_MAX_NAPI_VECTORS];

	struct net_device *netdev;
	struct mpnic_dev *mpd;

	u32 txq_size;

	u16 num_napi;
	u16 num_tx_queues;
};

#endif /* _MPNIC_NETDEV_H_ */
