/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Live Update support (core API)
 *
 * Copyright (c) 2026, Google LLC.
 * David Matlack <dmatlack@google.com>
 */
#ifndef DRIVERS_PCI_LIVEUPDATE_H
#define DRIVERS_PCI_LIVEUPDATE_H

#include <linux/pci.h>

#ifdef CONFIG_PCI_LIVEUPDATE
void pci_liveupdate_setup_device(struct pci_dev *dev);
void pci_liveupdate_cleanup_device(struct pci_dev *dev);
#else
static inline void pci_liveupdate_setup_device(struct pci_dev *dev)
{
}

static inline void pci_liveupdate_cleanup_device(struct pci_dev *dev)
{
}
#endif

#endif /* DRIVERS_PCI_LIVEUPDATE_H */
