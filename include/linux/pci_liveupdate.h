/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Live Update support (Public/Driver API)
 *
 * Copyright (c) 2026, Google LLC.
 * David Matlack <dmatlack@google.com>
 */
#ifndef LINUX_PCI_LIVEUPDATE_H
#define LINUX_PCI_LIVEUPDATE_H

#include <linux/kho/abi/pci.h>
#include <linux/liveupdate.h>
#include <linux/spinlock_types.h>
#include <linux/types.h>

/**
 * struct pci_liveupdate - PCI Live Update state for a struct pci_dev
 * @outgoing: State preserved for the next kernel.
 * @incoming: State preserved by the previous kernel.
 * @acs_ctrl: ACS features established by the previous kernel.
 * @preserve_bus_numbers: True if the PCI core should preserve the secondary and
 *                        subordinate bus numbers assigned to this device due to
 *                        an ongoing Live Update.
 * @was_preserved: True if this struct pci_dev was preserved by the previous
 *                 kernel. Unlike @incoming, this field is not cleared after
 *                 the device is finished participating in Live Update.
 */
struct pci_liveupdate {
	struct pci_dev_ser *outgoing;
	struct pci_dev_ser *incoming;
	u16 acs_ctrl;
	bool preserve_bus_numbers;
	bool was_preserved;
};

struct pci_dev;

#ifdef CONFIG_PCI_LIVEUPDATE
int pci_liveupdate_register_flb(struct liveupdate_file_handler *fh);
void pci_liveupdate_unregister_flb(struct liveupdate_file_handler *fh);
int pci_liveupdate_preserve(struct pci_dev *dev);
void pci_liveupdate_unpreserve(struct pci_dev *dev);
void pci_liveupdate_finish(struct pci_dev *dev);
bool pci_liveupdate_is_incoming(struct pci_dev *dev);
#else
static inline int pci_liveupdate_register_flb(struct liveupdate_file_handler *fh)
{
	return -EOPNOTSUPP;
}

static inline void pci_liveupdate_unregister_flb(struct liveupdate_file_handler *fh)
{
}

static inline int pci_liveupdate_preserve(struct pci_dev *dev)
{
	return -EOPNOTSUPP;
}

static inline void pci_liveupdate_unpreserve(struct pci_dev *dev)
{
}

static inline void pci_liveupdate_finish(struct pci_dev *dev)
{
}

static inline bool pci_liveupdate_is_incoming(struct pci_dev *dev)
{
	return false;
}
#endif

#endif /* LINUX_PCI_LIVEUPDATE_H */
