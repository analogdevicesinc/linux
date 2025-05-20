/* SPDX-License-Identifier: GPL-2.0-only */
/* SPDX-FileCopyrightText: Copyright Red Hat */

#ifndef _ICE_ADAPTER_H_
#define _ICE_ADAPTER_H_

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/refcount_types.h>

struct pci_dev;
struct ice_pf;

/**
 * struct ice_port_list - data used to store the list of adapter ports
 *
 * This structure contains data used to maintain a list of adapter ports
 *
 * @ports: list of ports
 * @lock: protect access to the ports list
 */
struct ice_port_list {
	struct list_head ports;
	/* To synchronize the ports list operations */
	struct mutex lock;
};

/**
 * struct ice_adapter - PCI adapter resources shared across PFs
 * @ptp_gltsyn_time_lock: Spinlock protecting access to the GLTSYN_TIME
 *                        register of the PTP clock.
 * @refcount: Reference count. struct ice_pf objects hold the references.
 * @ctrl_pf: Control PF of the adapter
 * @ports: Ports list
 * @device_serial_number: DSN cached for collision detection on 32bit systems
 */
struct ice_adapter {
	refcount_t refcount;
	/* For access to the GLTSYN_TIME register */
	spinlock_t ptp_gltsyn_time_lock;

	struct ice_pf *ctrl_pf;
	struct ice_port_list ports;
	u64 device_serial_number;
};

struct ice_adapter *ice_adapter_get(struct pci_dev *pdev);
void ice_adapter_put(struct pci_dev *pdev);

#endif /* _ICE_ADAPTER_H */
