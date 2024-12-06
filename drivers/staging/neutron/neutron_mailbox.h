/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 NXP
 *
 */

#ifndef NEUTRON_MAILBOX_H
#define NEUTRON_MAILBOX_H

#include <linux/types.h>
#include "neutron_device.h"

typedef void (*mbox_rx_callback)(struct neutron_device *ndev, void *data);

/* mbox tx message: command and args */
struct neutron_mbox_tx_msg {
	u32 command;
	u32 args[4];
	u32 argc;
};

/* mbox rx message: return value and args */
struct neutron_mbox_rx_msg {
	u32 retcode;
	u32 args[2];
};

struct neutron_mbox {
	int irq;
	void __iomem *base;
	struct neutron_device *ndev;
	const struct neutron_mbox_ops *ops;
	mbox_rx_callback callback;
};

struct neutron_mbox_ops {
	/* Send data to neutron */
	int (*send_data)(struct neutron_mbox *mbox, void *data);
	/* Receive data from neutron */
	int (*recv_data)(struct neutron_mbox *mbox, void *data);
	/* Send reset command to neutron */
	int (*send_reset)(struct neutron_mbox *mbox);
	/* Read return value */
	unsigned int (*read_ret)(struct neutron_mbox *mbox);
	/* Test whether tx is successful */
	bool (*tx_done)(struct neutron_mbox *mbox);
};

struct neutron_mbox *neutron_mbox_create(struct neutron_device *ndev, int irq,
					 mbox_rx_callback callback);

void neutron_mbox_destroy(struct neutron_mbox *mbox);

#endif /* NEUTRON_MAILBOX_H */
