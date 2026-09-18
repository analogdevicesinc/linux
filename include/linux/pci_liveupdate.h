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
 */
struct pci_liveupdate {
	struct pci_dev_ser *outgoing;
};

struct pci_dev;

#ifdef CONFIG_PCI_LIVEUPDATE
int pci_liveupdate_register_flb(struct liveupdate_file_handler *fh);
void pci_liveupdate_unregister_flb(struct liveupdate_file_handler *fh);
int pci_liveupdate_preserve(struct pci_dev *dev);
void pci_liveupdate_unpreserve(struct pci_dev *dev);
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
#endif

#endif /* LINUX_PCI_LIVEUPDATE_H */
